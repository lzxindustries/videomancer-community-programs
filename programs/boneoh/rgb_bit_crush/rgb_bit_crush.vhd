-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   RGB Bit Crush
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Per-channel bit-depth reduction (bit crushing) effect with full YUV<->RGB
--   colour space conversion.  Each channel has an independent crush-amount knob
--   that controls how many LSBs are zeroed.  Optional per-channel rounding
--   (round to nearest vs. floor/truncate) and a global invert are provided.
--
--   Crush amount is mapped from knob (0-1023) to one of 8 step sizes:
--     step_idx = knob / 128  (8 evenly-spaced bands across full knob travel)
--   Steps: 0→8, 1→16, 2→32, 3→48, 4→64, 5→96, 6→128, 7→256.
--   Power-of-2 steps (8, 16, 32, 64, 128, 256) use bitmask truncation.
--   Steps 48 and 96 use a shift-and-case LUT to avoid carry-chain multipliers.
--
--   Colour conversion uses the same pre-computed BT.601 BRAM tables as the
--   RGB Bit Rotator colour conversion path (11 BRAMs total; 31 of 32 blocks used).
--
-- Architecture:
--   Stage 0a    - YUV->RGB BRAM Lookup + control decode (1 clock) -> T+1
--   Stage 0a-ii - G-channel offset pre-sum               (1 clock) -> T+2
--   Stage 0b    - YUV->RGB Accumulate and Clamp          (1 clock) -> T+3
--   Stage 1     - Bit Crush Operation                    (1 clock) -> T+4
--   Stage 2     - Per-Channel Blend (3x interpolator_u) (4 clocks) -> T+8
--   Stage 3     - Global Blend      (3x interpolator_u) (4 clocks) -> T+12
--   Stage 4a-i  - RGB->YUV BRAM Lookup                  (1 clock) -> T+13
--   Stage 4a-ii - RGB->YUV Partial Pre-sums             (1 clock) -> T+14
--   Stage 4a-iii- RGB->YUV Channel Sums                 (1 clock) -> T+15
--   Stage 4b    - RGB->YUV Clamp                         (1 clock) -> T+16
--
-- Videomancer UV Convention Note:
--   data_in.u / data_out.u = Cr (red-difference);  data_in.v / data_out.v = Cb.
--   U and V are swapped when indexing the BT.601 LUTs so all internal RGB
--   computations use the standard convention.
--
-- Register Map:
--   Register  0: R crush amount (0-1023, maps to crush level 0-10) rotary_potentiometer_1
--   Register  1: G crush amount (0-1023, maps to crush level 0-10) rotary_potentiometer_2
--   Register  2: B crush amount (0-1023, maps to crush level 0-10) rotary_potentiometer_3
--   Register  3: R channel blend (0=dry, 1023=wet)                 rotary_potentiometer_4
--   Register  4: G channel blend (0=dry, 1023=wet)                 rotary_potentiometer_5
--   Register  5: B channel blend (0=dry, 1023=wet)                 rotary_potentiometer_6
--   Register  6: Packed toggle bits (hardware polarity: Off='1', On='0'):
--     bit 0: Invert  (0=On/invert output, 1=Off/normal)            toggle_switch_7
--     bit 1: R Round (0=On/round to nearest, 1=Off/truncate)       toggle_switch_8
--     bit 2: G Round (0=On/round to nearest, 1=Off/truncate)       toggle_switch_9
--     bit 3: B Round (0=On/round to nearest, 1=Off/truncate)       toggle_switch_10
--     bit 4: Bypass  (0=Process, 1=Bypass)                         toggle_switch_11
--   Register  7: Global blend (0=dry, 1023=wet)                    linear_potentiometer_12
--
-- Timing:
--   Total pipeline latency: 16 clock cycles.
--   Bypass delay line and sync delays are all 16 clocks.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.rgb_yuv_tables_pkg.all;

architecture rgb_bit_crush of program_top is

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 16;  -- Stage 1 is 1 clock (not 2)
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 5;   -- dry tap for global blend
    constant C_UV_OFFSET             : integer := 512;

    --------------------------------------------------------------------------------
    -- YUV<->RGB Lookup Table Signals (data from rgb_yuv_tables_pkg)
    --------------------------------------------------------------------------------
    -- YUV->RGB
    signal s_lut_yr_r : t_lut11 := C_LUT_YR_R;
    signal s_lut_yr_gu : t_lut9 := C_LUT_YR_GU;
    signal s_lut_yr_gv : t_lut10 := C_LUT_YR_GV;
    signal s_lut_yr_b : t_lut11 := C_LUT_YR_B;
    -- RGB->YUV
    signal s_lut_ry_r : t_lut9 := C_LUT_RY_R;
    signal s_lut_ry_g : t_lut10 := C_LUT_RY_G;
    signal s_lut_ry_b : t_lut7 := C_LUT_RY_B;
    signal s_lut_ru_r : t_lut9 := C_LUT_RU_R;
    signal s_lut_ru_g : t_lut10 := C_LUT_RU_G;
    signal s_lut_rv_g : t_lut10 := C_LUT_RV_G;
    signal s_lut_rv_b : t_lut8 := C_LUT_RV_B;

    --------------------------------------------------------------------------------
    -- Functions
    --------------------------------------------------------------------------------

    -- apply_crush: quantise pixel to the nearest multiple of the selected step.
    --   step_idx: 0→8, 1→16, 2→32, 3→48, 4→64, 5→96, 6→128, 7→256.
    --   do_round='1': round to nearest; do_round='0': truncate (floor).
    --   Power-of-2 steps use bitmask with overflow saturation.
    --   Steps 48 and 96 use a direct case-statement LUT on (pixel+offset)>>shift,
    --   returning the pre-computed quantised value.  This avoids carry-chain
    --   multiplications and synthesises as a shallow LUT tree (~3-4 levels).
    --   Overflow (pixel+round_offset > 1023) saturates to the largest
    --   representable multiple of the step for power-of-2 steps.
    function apply_crush(
        pixel    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        step_idx : integer range 0 to 7;
        do_round : std_logic)
        return unsigned is
        variable v_pix : unsigned(10 downto 0);   -- pixel + offset (max 1023+128=1151)
    begin
        case step_idx is
            when 0 =>  -- step=8, round_offset=4, saturate to 1016
                if do_round = '1' then v_pix := ('0' & pixel) + 4;
                else                   v_pix := '0' & pixel; end if;
                if v_pix(10) = '1' then return to_unsigned(1016, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 3) & "000";
            when 1 =>  -- step=16, round_offset=8, saturate to 1008
                if do_round = '1' then v_pix := ('0' & pixel) + 8;
                else                   v_pix := '0' & pixel; end if;
                if v_pix(10) = '1' then return to_unsigned(1008, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 4) & "0000";
            when 2 =>  -- step=32, round_offset=16, saturate to 992
                if do_round = '1' then v_pix := ('0' & pixel) + 16;
                else                   v_pix := '0' & pixel; end if;
                if v_pix(10) = '1' then return to_unsigned(992, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 5) & "00000";
            when 3 =>  -- step=48 = 16*3, round_offset=24
                -- LUT: (pixel+offset)>>4 indexes directly into multiples of 48.
                -- Max index = (1023+24)>>4 = 65.  All 22 output values pre-computed.
                if do_round = '1' then v_pix := ('0' & pixel) + 24;
                else                   v_pix := '0' & pixel; end if;
                case to_integer(v_pix(10 downto 4)) is
                    when  0| 1| 2 => return to_unsigned(   0, C_VIDEO_DATA_WIDTH);
                    when  3| 4| 5 => return to_unsigned(  48, C_VIDEO_DATA_WIDTH);
                    when  6| 7| 8 => return to_unsigned(  96, C_VIDEO_DATA_WIDTH);
                    when  9|10|11 => return to_unsigned( 144, C_VIDEO_DATA_WIDTH);
                    when 12|13|14 => return to_unsigned( 192, C_VIDEO_DATA_WIDTH);
                    when 15|16|17 => return to_unsigned( 240, C_VIDEO_DATA_WIDTH);
                    when 18|19|20 => return to_unsigned( 288, C_VIDEO_DATA_WIDTH);
                    when 21|22|23 => return to_unsigned( 336, C_VIDEO_DATA_WIDTH);
                    when 24|25|26 => return to_unsigned( 384, C_VIDEO_DATA_WIDTH);
                    when 27|28|29 => return to_unsigned( 432, C_VIDEO_DATA_WIDTH);
                    when 30|31|32 => return to_unsigned( 480, C_VIDEO_DATA_WIDTH);
                    when 33|34|35 => return to_unsigned( 528, C_VIDEO_DATA_WIDTH);
                    when 36|37|38 => return to_unsigned( 576, C_VIDEO_DATA_WIDTH);
                    when 39|40|41 => return to_unsigned( 624, C_VIDEO_DATA_WIDTH);
                    when 42|43|44 => return to_unsigned( 672, C_VIDEO_DATA_WIDTH);
                    when 45|46|47 => return to_unsigned( 720, C_VIDEO_DATA_WIDTH);
                    when 48|49|50 => return to_unsigned( 768, C_VIDEO_DATA_WIDTH);
                    when 51|52|53 => return to_unsigned( 816, C_VIDEO_DATA_WIDTH);
                    when 54|55|56 => return to_unsigned( 864, C_VIDEO_DATA_WIDTH);
                    when 57|58|59 => return to_unsigned( 912, C_VIDEO_DATA_WIDTH);
                    when 60|61|62 => return to_unsigned( 960, C_VIDEO_DATA_WIDTH);
                    when others   => return to_unsigned(1008, C_VIDEO_DATA_WIDTH);
                end case;
            when 4 =>  -- step=64, round_offset=32, saturate to 960
                if do_round = '1' then v_pix := ('0' & pixel) + 32;
                else                   v_pix := '0' & pixel; end if;
                if v_pix(10) = '1' then return to_unsigned(960, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 6) & "000000";
            when 5 =>  -- step=96 = 32*3, round_offset=48
                -- LUT: (pixel+offset)>>5 indexes directly into multiples of 96.
                -- Max index = (1023+48)>>5 = 33.  Capped at 10*96=960 (11*96>1023).
                if do_round = '1' then v_pix := ('0' & pixel) + 48;
                else                   v_pix := '0' & pixel; end if;
                case to_integer(v_pix(10 downto 5)) is
                    when  0| 1| 2 => return to_unsigned(  0, C_VIDEO_DATA_WIDTH);
                    when  3| 4| 5 => return to_unsigned( 96, C_VIDEO_DATA_WIDTH);
                    when  6| 7| 8 => return to_unsigned(192, C_VIDEO_DATA_WIDTH);
                    when  9|10|11 => return to_unsigned(288, C_VIDEO_DATA_WIDTH);
                    when 12|13|14 => return to_unsigned(384, C_VIDEO_DATA_WIDTH);
                    when 15|16|17 => return to_unsigned(480, C_VIDEO_DATA_WIDTH);
                    when 18|19|20 => return to_unsigned(576, C_VIDEO_DATA_WIDTH);
                    when 21|22|23 => return to_unsigned(672, C_VIDEO_DATA_WIDTH);
                    when 24|25|26 => return to_unsigned(768, C_VIDEO_DATA_WIDTH);
                    when 27|28|29 => return to_unsigned(864, C_VIDEO_DATA_WIDTH);
                    when others   => return to_unsigned(960, C_VIDEO_DATA_WIDTH);
                end case;
            when 6 =>  -- step=128, round_offset=64, saturate to 896
                if do_round = '1' then v_pix := ('0' & pixel) + 64;
                else                   v_pix := '0' & pixel; end if;
                if v_pix(10) = '1' then return to_unsigned(896, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 7) & "0000000";
            when others =>  -- step=256, round_offset=128, saturate to 768
                if do_round = '1' then v_pix := ('0' & pixel) + 128;
                else                   v_pix := '0' & pixel; end if;
                if v_pix(10) = '1' then return to_unsigned(768, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 8) & "00000000";
        end case;
    end function;

    -- knob_to_crush: maps 10-bit knob (0-1023) to step index (0-7).
    --   8 equal bands of 128 knob counts each.
    --   Index: 0→8, 1→16, 2→32, 3→48, 4→64, 5→96, 6→128, 7→256.
    function knob_to_crush(knob : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return integer is
    begin
        return to_integer(knob) / 128;
    end function;

    function clamp10(v : integer) return unsigned is
    begin
        if    v <    0 then return to_unsigned(0,    C_VIDEO_DATA_WIDTH);
        elsif v > 1023 then return to_unsigned(1023, C_VIDEO_DATA_WIDTH);
        else                return to_unsigned(v,    C_VIDEO_DATA_WIDTH);
        end if;
    end function;

    -- clamp10_uv: clamp10(v + C_UV_OFFSET) with the offset folded into the
    -- comparator bounds as compile-time constants.
    function clamp10_uv(v : integer) return unsigned is
    begin
        if    v < -C_UV_OFFSET then return to_unsigned(0,    C_VIDEO_DATA_WIDTH);
        elsif v >  1023 - C_UV_OFFSET then return to_unsigned(1023, C_VIDEO_DATA_WIDTH);
        else                               return to_unsigned(v + C_UV_OFFSET, C_VIDEO_DATA_WIDTH);
        end if;
    end function;

    --------------------------------------------------------------------------------
    -- Control Signals
    --------------------------------------------------------------------------------
    signal s_bypass_enable  : std_logic;
    signal s_blend_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- Bit-crush controls (registered in Stage 0a alongside BRAM lookup)
    signal s_crush_r        : integer range 0 to 7 := 0;
    signal s_crush_g        : integer range 0 to 7 := 0;
    signal s_crush_b        : integer range 0 to 7 := 0;
    signal s_round_r        : std_logic := '0';  -- '1' = On/Round
    signal s_round_g        : std_logic := '0';
    signal s_round_b        : std_logic := '0';
    signal s_invert         : std_logic := '0';  -- '1' = On/Invert output

    --------------------------------------------------------------------------------
    -- Stage 0a: YUV->RGB BRAM Lookup Outputs (T+1)
    --------------------------------------------------------------------------------
    signal s_yr_r_off   : integer := 0;
    signal s_yr_gu_off  : integer := 0;
    signal s_yr_gv_off  : integer := 0;
    signal s_yr_b_off   : integer := 0;
    signal s_yr_y       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                      := (others => '0');
    signal s_yr_avid    : std_logic := '0';
    -- Delayed raw U/V for correct blanking passthrough in Stage 0b
    signal s_yr_u_raw   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                      := (others => '0');
    signal s_yr_v_raw   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                      := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 0a-ii: G-channel Pre-Sum (T+2)
    -- Sums the two G offsets from Stage 0a so Stage 0b needs only 1 add for G.
    -- Kept as a separate pipeline stage (not merged into Stage 0a) to preserve
    -- Yosys BRAM inference — adding arithmetic to a BRAM read expression prevents
    -- the synthesis tool from mapping the ROM to block RAM.
    --------------------------------------------------------------------------------
    signal s_yr_g_presum : integer := 0;   -- s_yr_gu_off + s_yr_gv_off
    signal s_yr_r_off_d  : integer := 0;   -- s_yr_r_off delayed 1 clock
    signal s_yr_b_off_d  : integer := 0;   -- s_yr_b_off delayed 1 clock
    signal s_yr_y_d      : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                      := (others => '0');
    signal s_yr_avid_d   : std_logic := '0';
    signal s_yr_u_raw_d  : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                      := (others => '0');
    signal s_yr_v_raw_d  : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                      := (others => '0');
    --------------------------------------------------------------------------------
    -- Stage 0b: YUV->RGB Accumulate Outputs (T+3)
    --------------------------------------------------------------------------------
    signal s_rgb_r      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rgb_g      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rgb_b      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rgb_valid  : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 1: Bit Crush Outputs (T+4)
    --------------------------------------------------------------------------------
    signal s_processed_r     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_g     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_b     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Per-Channel Blend Outputs (T+8)
    --------------------------------------------------------------------------------
    signal s_blended_r       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_g       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_b       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_r_valid : std_logic;
    signal s_blended_g_valid : std_logic;
    signal s_blended_b_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 3: Global Blend Outputs (T+12)
    --------------------------------------------------------------------------------
    signal s_global_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_r_valid  : std_logic;
    signal s_global_g_valid  : std_logic;
    signal s_global_b_valid  : std_logic;

    -- RGB original delayed 1 clock: T+3+1=T+4, aligned with s_processed (per-channel blend dry)
    signal s_r_for_blend     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_g_for_blend     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_b_for_blend     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    -- RGB original delayed 6 clocks: T+2+6=T+8, aligned with s_blended (global blend dry)
    signal s_r_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_g_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_b_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 4a-i: RGB->YUV BRAM Lookup Outputs (T+12)
    --------------------------------------------------------------------------------
    signal s_4a_ry_r    : integer := 0;   -- ry_r(R):  0..299  unsigned
    signal s_4a_ry_g    : integer := 0;   -- ry_g(G):  0..601  unsigned
    signal s_4a_ry_b    : integer := 0;   -- ry_b(B):  0..117  unsigned
    signal s_4a_ru_r    : integer := 0;   -- ru_r(R): -173..0  signed
    signal s_4a_ru_g    : integer := 0;   -- ru_g(G): -339..0  signed
    signal s_4a_rv_g    : integer := 0;   -- rv_g(G): -429..0  signed
    signal s_4a_rv_b    : integer := 0;   -- rv_b(B):  -83..0  signed
    -- Registered R and B for wire-shift operations in Stage 4a-ii
    signal s_4a_r_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_4a_b_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_4ai_valid  : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 4a-ii: RGB->YUV Partial Sums (T+14)
    -- Registered pipeline stage between BRAM outputs and the final channel sums.
    -- Keeps BRAM reads and arithmetic in separate clock cycles so Yosys can still
    -- infer block RAMs for the lookup tables (adding arithmetic inside the BRAM
    -- read expression breaks BRAM inference and inflates LC usage ~30x).
    --------------------------------------------------------------------------------
    signal s_4b_ry_rg   : integer := 0;   -- s_4a_ry_r + s_4a_ry_g
    signal s_4b_ry_b    : integer := 0;   -- s_4a_ry_b (pass-through)
    signal s_4b_ru_rg   : integer := 0;   -- s_4a_ru_r + s_4a_ru_g
    signal s_4b_rv_gb   : integer := 0;   -- s_4a_rv_g + s_4a_rv_b
    signal s_4b_r_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_4b_b_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_4b_valid   : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 4a-iii: RGB->YUV Channel Sums (T+15)
    --------------------------------------------------------------------------------
    signal s_ry_y_sum   : integer := 0;
    signal s_ry_u_sum   : integer := 0;
    signal s_ry_v_sum   : integer := 0;
    signal s_ry_valid   : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 4b: RGB->YUV Output (T+14)
    --------------------------------------------------------------------------------
    signal s_yuv_out_y      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_yuv_out_u      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_yuv_out_v      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_yuv_out_valid  : std_logic;

    --------------------------------------------------------------------------------
    -- Bypass Path Delay Line (14 clocks)
    --------------------------------------------------------------------------------
    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;
    signal s_y_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

begin

    --------------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------------
    s_blend_r       <= unsigned(registers_in(3));
    s_blend_g       <= unsigned(registers_in(4));
    s_blend_b       <= unsigned(registers_in(5));
    s_bypass_enable <= registers_in(6)(4);   -- toggle_switch_11: 0=Process, 1=Bypass
    s_global_blend  <= unsigned(registers_in(7));  -- linear_potentiometer_12

    --------------------------------------------------------------------------------
    -- Stage 0a: YUV->RGB BRAM Lookup + Control Decode
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Reads 4 pre-computed offsets (already divided by 1024) from lookup tables.
    -- Critical path: address decode + BRAM read (fast, no multiply chains).
    -- Hardware switch polarity: Off='1', On='0' (pull-up convention).
    --------------------------------------------------------------------------------
    p_yuv_rgb_lut : process(clk)
    begin
        if rising_edge(clk) then
            -- NOTE: Videomancer SDK uses a non-standard UV convention:
            --   data_in.u = Cr (red-difference),  data_in.v = Cb (blue-difference)
            -- The LUTs were built for standard BT.601 (U=Cb, V=Cr), so we swap
            -- data_in.u <-> data_in.v here to feed each LUT the correct channel.
            s_yr_r_off  <= to_integer(signed(s_lut_yr_r( to_integer(unsigned(data_in.u)))));
            s_yr_gu_off <= to_integer(signed(s_lut_yr_gu(to_integer(unsigned(data_in.v)))));
            s_yr_gv_off <= to_integer(signed(s_lut_yr_gv(to_integer(unsigned(data_in.u)))));
            s_yr_b_off  <= to_integer(signed(s_lut_yr_b( to_integer(unsigned(data_in.v)))));
            s_yr_y      <= data_in.y;
            s_yr_avid   <= data_in.avid;
            s_yr_u_raw  <= data_in.u;
            s_yr_v_raw  <= data_in.v;
            -- Pre-register decoded controls (parallel with BRAM lookup, no added latency).
            s_crush_r <= knob_to_crush(unsigned(registers_in(0)));
            s_crush_g <= knob_to_crush(unsigned(registers_in(1)));
            s_crush_b <= knob_to_crush(unsigned(registers_in(2)));
            -- Off='0'/On='1': feature active when bit='1' (On position).
            s_invert  <= registers_in(6)(0);  -- '1' = On/Invert output
            s_round_r <= registers_in(6)(1);  -- '1' = On/Round R
            s_round_g <= registers_in(6)(2);  -- '1' = On/Round G
            s_round_b <= registers_in(6)(3);  -- '1' = On/Round B
        end if;
    end process p_yuv_rgb_lut;

    --------------------------------------------------------------------------------
    -- Stage 0a-ii: G-channel Pre-Sum
    -- Latency: 1 clock. Input T+1, output T+2.
    -- Pre-registers the sum of the two G-channel offsets (gu + gv) so that
    -- Stage 0b only needs a single addition (Y + g_presum) instead of two.
    -- All other signals are passed through as _d companions.
    --------------------------------------------------------------------------------
    p_yuv_rgb_presum : process(clk)
    begin
        if rising_edge(clk) then
            s_yr_g_presum <= s_yr_gu_off + s_yr_gv_off;
            s_yr_r_off_d  <= s_yr_r_off;
            s_yr_b_off_d  <= s_yr_b_off;
            s_yr_y_d      <= s_yr_y;
            s_yr_avid_d   <= s_yr_avid;
            s_yr_u_raw_d  <= s_yr_u_raw;
            s_yr_v_raw_d  <= s_yr_v_raw;
        end if;
    end process p_yuv_rgb_presum;

    --------------------------------------------------------------------------------
    -- Stage 0b: YUV->RGB Accumulate and Clamp
    -- Latency: 1 clock. Input T+2, output T+3.
    -- Simply adds Y to the pre-divided offsets (no >>10 needed).
    -- Critical path: 1 add + clamp per channel (G pre-summed in Stage 0a-ii).
    --------------------------------------------------------------------------------
    p_yuv_rgb_acc : process(clk)
        variable v_y, v_r, v_g, v_b : integer;
    begin
        if rising_edge(clk) then
            v_y := to_integer(unsigned(s_yr_y_d));
            v_r := v_y + s_yr_r_off_d;
            v_g := v_y + s_yr_g_presum;
            v_b := v_y + s_yr_b_off_d;

            if s_yr_avid_d = '1' then
                s_rgb_r <= clamp10(v_r);
                s_rgb_g <= clamp10(v_g);
                s_rgb_b <= clamp10(v_b);
            else
                s_rgb_r <= unsigned(s_yr_y_d);
                s_rgb_g <= unsigned(s_yr_u_raw_d);
                s_rgb_b <= unsigned(s_yr_v_raw_d);
            end if;
            s_rgb_valid <= s_yr_avid_d;
        end if;
    end process p_yuv_rgb_acc;

    --------------------------------------------------------------------------------
    -- Stage 1: Bit Crush Operation
    -- Latency: 1 clock. Input T+3, output T+4.
    -- Applies apply_crush() independently to each channel.
    -- Per-channel round switches select floor (truncate) or round-to-nearest.
    -- Invert switch bitwise-NOTs all three channels after crushing.
    -- The interpolators downstream use enable=s_processed_valid so they hold
    -- their last output during blanking.
    --------------------------------------------------------------------------------
    p_bit_crush : process(clk)
        variable v_r, v_g, v_b : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_r := apply_crush(s_rgb_r, s_crush_r, s_round_r);
            v_g := apply_crush(s_rgb_g, s_crush_g, s_round_g);
            v_b := apply_crush(s_rgb_b, s_crush_b, s_round_b);
            if s_invert = '1' then
                s_processed_r <= not v_r;
                s_processed_g <= not v_g;
                s_processed_b <= not v_b;
            else
                s_processed_r <= v_r;
                s_processed_g <= v_g;
                s_processed_b <= v_b;
            end if;
            s_processed_valid <= s_rgb_valid;
        end if;
    end process p_bit_crush;

    --------------------------------------------------------------------------------
    -- Stage 2: Per-Channel Wet/Dry Blend
    -- Latency: 4 clocks. Input T+4, output T+8.
    --------------------------------------------------------------------------------
    interp_r : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_r_for_blend, b=>s_processed_r, t=>s_blend_r,
                 result=>s_blended_r, valid=>s_blended_r_valid);

    interp_g : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_g_for_blend, b=>s_processed_g, t=>s_blend_g,
                 result=>s_blended_g, valid=>s_blended_g_valid);

    interp_b : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_b_for_blend, b=>s_processed_b, t=>s_blend_b,
                 result=>s_blended_b, valid=>s_blended_b_valid);

    --------------------------------------------------------------------------------
    -- Delay Line: RGB dry inputs for per-channel and global blend stages.
    -- Single shift register tapped at two points:
    --   index 0 (1 clock):  T+3+1=T+4 — aligned with s_processed for per-channel blend
    --   index 4 (5 clocks): T+3+5=T+8 — aligned with s_blended for global blend
    -- Using one shared chain eliminates the separate s_orig_*_d1/d2 registers.
    --------------------------------------------------------------------------------
    p_global_dry_delay : process(clk)
        type t_data_delay is array (0 to C_PRE_GLOBAL_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_r_delay : t_data_delay := (others => (others => '0'));
        variable v_g_delay : t_data_delay := (others => (others => '0'));
        variable v_b_delay : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_r_delay := s_rgb_r & v_r_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_g_delay := s_rgb_g & v_g_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_b_delay := s_rgb_b & v_b_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            s_r_for_blend  <= v_r_delay(0);
            s_g_for_blend  <= v_g_delay(0);
            s_b_for_blend  <= v_b_delay(0);
            s_r_for_global <= v_r_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_g_for_global <= v_g_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_b_for_global <= v_b_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 3: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+8, output T+12.
    --------------------------------------------------------------------------------
    interp_global_r : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_r_valid,
                 a=>s_r_for_global, b=>s_blended_r, t=>s_global_blend,
                 result=>s_global_r, valid=>s_global_r_valid);

    interp_global_g : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_g_valid,
                 a=>s_g_for_global, b=>s_blended_g, t=>s_global_blend,
                 result=>s_global_g, valid=>s_global_g_valid);

    interp_global_b : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_b_valid,
                 a=>s_b_for_global, b=>s_blended_b, t=>s_global_blend,
                 result=>s_global_b, valid=>s_global_b_valid);

    --------------------------------------------------------------------------------
    -- Stage 4a-i: RGB->YUV BRAM Lookup
    -- Latency: 1 clock. Input T+12, output T+13.
    -- Reads 7 pre-computed partial values from BRAM.
    -- u_b(B) = B>>1 and v_r(R) = R>>1 are implemented as wire shifts below.
    --------------------------------------------------------------------------------
    p_rgb_yuv_lut : process(clk)
    begin
        if rising_edge(clk) then
            s_4a_ry_r  <= to_integer(unsigned(s_lut_ry_r(to_integer(s_global_r))));
            s_4a_ry_g  <= to_integer(unsigned(s_lut_ry_g(to_integer(s_global_g))));
            s_4a_ry_b  <= to_integer(unsigned(s_lut_ry_b(to_integer(s_global_b))));
            s_4a_ru_r  <= to_integer(signed(  s_lut_ru_r(to_integer(s_global_r))));
            s_4a_ru_g  <= to_integer(signed(  s_lut_ru_g(to_integer(s_global_g))));
            s_4a_rv_g  <= to_integer(signed(  s_lut_rv_g(to_integer(s_global_g))));
            s_4a_rv_b  <= to_integer(signed(  s_lut_rv_b(to_integer(s_global_b))));
            s_4a_r_d   <= s_global_r;
            s_4a_b_d   <= s_global_b;
            s_4ai_valid <= s_global_r_valid;
        end if;
    end process p_rgb_yuv_lut;

    --------------------------------------------------------------------------------
    -- Stage 4a-ii: RGB->YUV Partial Sums
    -- Latency: 1 clock. Input T+13, output T+14.
    -- Pairs of BRAM results are summed here, one pipeline stage after the BRAM
    -- reads.  Keeping the addition in a separate stage from the BRAM read is
    -- essential: Yosys only infers block RAM when the registered signal is driven
    -- directly by the ROM lookup with no arithmetic in the expression.  Adding
    -- the sum here (not in p_rgb_yuv_lut) preserves BRAM inference for all 11
    -- lookup tables and avoids the ~30x LC explosion caused by LUT-based ROMs.
    --------------------------------------------------------------------------------
    p_rgb_yuv_presum : process(clk)
    begin
        if rising_edge(clk) then
            s_4b_ry_rg <= s_4a_ry_r + s_4a_ry_g;
            s_4b_ry_b  <= s_4a_ry_b;
            s_4b_ru_rg <= s_4a_ru_r + s_4a_ru_g;
            s_4b_rv_gb <= s_4a_rv_g + s_4a_rv_b;
            s_4b_r_d   <= s_4a_r_d;
            s_4b_b_d   <= s_4a_b_d;
            s_4b_valid <= s_4ai_valid;
        end if;
    end process p_rgb_yuv_presum;

    --------------------------------------------------------------------------------
    -- Stage 4a-iii: RGB->YUV Channel Sums
    -- Latency: 1 clock. Input T+14, output T+15.
    -- Each channel now needs only one addition (pairs pre-summed in Stage 4a-ii).
    -- C_UV_OFFSET is deferred to Stage 4b to keep this stage at 2 additions max.
    --------------------------------------------------------------------------------
    p_rgb_yuv_sum : process(clk)
    begin
        if rising_edge(clk) then
            s_ry_y_sum <= s_4b_ry_rg + s_4b_ry_b;
            s_ry_u_sum <= s_4b_ru_rg
                          + to_integer(unsigned('0' & s_4b_b_d(9 downto 1)));
            s_ry_v_sum <= to_integer(unsigned('0' & s_4b_r_d(9 downto 1)))
                          + s_4b_rv_gb;
            s_ry_valid <= s_4b_valid;
        end if;
    end process p_rgb_yuv_sum;

    --------------------------------------------------------------------------------
    -- Stage 4b: RGB->YUV Clamp
    -- Latency: 1 clock. Input T+15, output T+16.
    -- Uses clamp10_uv() which folds C_UV_OFFSET into the comparator bounds so
    -- the saturation checks evaluate directly from the Stage 4a-iii sums without
    -- an adder on the compare critical path.
    --------------------------------------------------------------------------------
    p_rgb_yuv_acc : process(clk)
    begin
        if rising_edge(clk) then
            s_yuv_out_y <= clamp10(s_ry_y_sum);
            -- Videomancer SDK: u=Cr (red-diff), v=Cb (blue-diff).
            -- s_ry_u_sum holds Cb (standard U); s_ry_v_sum holds Cr (standard V).
            -- Swap assignments so the SDK receives the correct convention.
            s_yuv_out_u     <= clamp10_uv(s_ry_v_sum);  -- Cr -> data_out.u
            s_yuv_out_v     <= clamp10_uv(s_ry_u_sum);  -- Cb -> data_out.v
            s_yuv_out_valid <= s_ry_valid;
        end if;
    end process p_rgb_yuv_acc;

    --------------------------------------------------------------------------------
    -- Bypass Path Delay Line (14 clocks)
    --------------------------------------------------------------------------------
    p_bypass_delay : process(clk)
        type t_sync_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
        variable v_y_delay     : t_data_delay := (others => (others => '0'));
        variable v_u_delay     : t_data_delay := (others => (others => '0'));
        variable v_v_delay     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_y_delay     := data_in.y       & v_y_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_delay     := data_in.u       & v_u_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_delay     := data_in.v       & v_v_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_hsync_n_delayed <= v_hsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_vsync_n_delayed <= v_vsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_field_n_delayed <= v_field_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_y_delayed       <= v_y_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_u_delayed       <= v_u_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_v_delayed       <= v_v_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_bypass_delay;

    --------------------------------------------------------------------------------
    -- Output Multiplexing
    --------------------------------------------------------------------------------
    data_out.y <= std_logic_vector(s_yuv_out_y) when s_bypass_enable = '0'
                  else s_y_delayed;
    data_out.u <= std_logic_vector(s_yuv_out_u) when s_bypass_enable = '0'
                  else s_u_delayed;
    data_out.v <= std_logic_vector(s_yuv_out_v) when s_bypass_enable = '0'
                  else s_v_delayed;

    data_out.avid    <= s_yuv_out_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end architecture rgb_bit_crush;
