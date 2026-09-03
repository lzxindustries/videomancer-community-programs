-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   YUV Bit Crush
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Per-channel bit-depth reduction (bit crushing) effect operating directly in
--   YUV space — no colour conversion.
--
--   Y, U, and V channels are all bit-crushed: values are quantised to multiples
--   of a selected step size.  Y is always truncated (floor). U and V support
--   optional round-to-nearest (per-channel switches) and optional RPDF dither
--   before crushing.  Dither adds pseudorandom noise before quantising U and V,
--   reducing banding at the cost of added grain (lfsr16 for U, lfsr10 for V).
--
--   Crush amount is mapped from knob (0-1023) to one of 8 step sizes:
--     step_idx = knob / 128  (8 evenly-spaced bands across full knob travel)
--   Steps: 0→8, 1→16, 2→32, 3→48, 4→64, 5→96, 6→128, 7→256.
--
--   An Invert switch bitwise-NOTs all three processed channels.
--
-- Architecture:
--   Stage 0a    - Control Decode                          (1 clock) -> T+1
--   Stage 0b    - Bit Crush (Y/U/V)                       (1 clock) -> T+2
--   Stage 1     - Per-Channel Blend (3x interpolator_u)  (4 clocks) -> T+6
--   Stage 2     - Global Blend      (3x interpolator_u)  (4 clocks) -> T+10
--
-- Register Map:
--   Register  0: Y crush amount (0-1023, maps to step index 0-7)         rotary_potentiometer_1
--   Register  1: U crush amount (0-1023, maps to step index 0-7)         rotary_potentiometer_2
--   Register  2: V crush amount (0-1023, maps to step index 0-7)         rotary_potentiometer_3
--   Register  3: Y channel blend (0=dry, 1023=wet)                       rotary_potentiometer_4
--   Register  4: U channel blend (0=dry, 1023=wet)                       rotary_potentiometer_5
--   Register  5: V channel blend (0=dry, 1023=wet)                       rotary_potentiometer_6
--   Register  6: Packed toggle bits (Off='0', On='1'):
--     bit 0: Invert  (1=On/invert output, 0=Off/normal)                  toggle_switch_7
--     bit 1: Dither  (1=On/dither U+V,   0=Off/no dither)  Y always truncates  toggle_switch_8
--     bit 2: U Round (1=On/round nearest, 0=Off/truncate)                toggle_switch_9
--     bit 3: V Round (1=On/round nearest, 0=Off/truncate)                toggle_switch_10
--     bit 4: Bypass  (1=Bypass, 0=Process)                               toggle_switch_11
--   Register  7: Global blend (0=dry, 1023=wet)                          linear_potentiometer_12
--
-- Timing:
--   Total pipeline latency: 10 clock cycles.
--   Bypass delay line and sync delays are all 10 clocks.
--     Stage 0a (control decode):       1 clock  -> T+1
--     Stage 0b (bit crush):            1 clock  -> T+2
--     Stage 1  (per-channel blend):    4 clocks -> T+6
--     Stage 2  (global blend):         4 clocks -> T+10
--
--   Pre-global delay: 6 clocks (data_in at T+0, delayed to T+6 for global dry)
--   Per-channel dry:  2 clocks (data_in.y/u/v delayed to T+2, aligned with Stage 0b)
--
-- LFSR (dither):
--   lfsr16 free-runs continuously (period 2^16-1 = 65535).
--   lfsr10 free-runs from a zero-safe seed derived from lfsr16.
--   When dither is On: U dither = lfsr16[9:0] AND step_lower_mask(crush_u)
--                      V dither = lfsr10_out   AND step_lower_mask(crush_v)
--   step_lower_mask gates dither to the largest (2^k - 1) less than the step.
--   Dither is applied before quantising; the result is the nearest lower multiple.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture yuv_bit_crush of program_top is

    -- AUTO_IO_ALIGN_PAD declarations
    type t_io_align_stream is record
        y       : std_logic_vector(10-1 downto 0);
        u       : std_logic_vector(10-1 downto 0);
        v       : std_logic_vector(10-1 downto 0);
        avid    : std_logic;
        hsync_n : std_logic;
        vsync_n : std_logic;
        field_n : std_logic;
    end record;
    signal s_io_align_in : t_io_align_stream;
    signal s_io_pad_0 : t_io_align_stream;
    signal s_io_pad_1 : t_io_align_stream;

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    -- Total pipeline latency: 1 + 1 + 4 + 4 = 10 clocks
    constant C_PROCESSING_DELAY_CLKS : integer := 10;

    -- Delay for global blend "dry" YUV input.
    -- Original YUV valid at T+0. Stage 1 output valid at T+6.
    -- Delay original by 6 clocks: T+0+6 = T+6. Aligned.
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 6;

    -- Pixels with Y at or below this value have their U and V crush bypassed.
    -- Large chroma step sizes quantize near-neutral U/V to off-neutral grid points,
    -- producing a green tint in what should be black areas.  16 counts (~1.6% of
    -- full scale) covers signal chain noise on both YPbPr and HDMI inputs.
    constant C_BLACK_THRESHOLD : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) :=
        to_unsigned(16, C_VIDEO_DATA_WIDTH);


    --------------------------------------------------------------------------------
    -- Functions
    --------------------------------------------------------------------------------

    -- apply_crush: quantise pixel to the nearest multiple of the selected step,
    --   with optional round-to-nearest or RPDF dither.
    --   step_idx: 0→8, 1→16, 2→32, 3→48, 4→64, 5→96, 6→128, 7→256.
    --   do_round='1': add step/2 before quantising (round to nearest).
    --   do_dither='1' (only when do_round='0'): add gated LFSR noise before
    --     quantising; dither is masked to the largest (2^k-1) < step.
    --   lmask: pre-registered step-lower mask (crush_to_lmask, Stage 0a).
    --   round_off: pre-registered round offset (crush_to_roundoff, Stage 0a).
    --   Power-of-2 steps use bitmask with overflow saturation.
    --   Steps 48 and 96 use a direct case-statement LUT on (pixel+offset)>>shift,
    --   returning the pre-computed quantised value.  This avoids carry-chain
    --   multiplications and synthesises as a shallow LUT tree (~3-4 levels).
    function apply_crush(
        pixel     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        step_idx  : integer range 0 to 7;
        do_round  : std_logic;
        dither    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        do_dither : std_logic;
        lmask     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        round_off : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return unsigned is
        variable v_off  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_pix  : unsigned(10 downto 0);   -- pixel + offset (max 1023+255=1278)
    begin
        -- Select offset: round_offset takes priority over dither.
        -- lmask and round_off are pre-registered in Stage 0a (crush_to_lmask /
        -- crush_to_roundoff) to remove their case decode trees from Stage 0b's
        -- critical path.
        if do_round = '1' then
            v_off := round_off;
        elsif do_dither = '1' then
            v_off := dither and lmask;
        else
            v_off := (others => '0');
        end if;
        -- Apply quantisation
        v_pix := ('0' & pixel) + ('0' & v_off);
        case step_idx is
            when 0 =>  -- step=8, saturate to 1016
                if v_pix(10) = '1' then return to_unsigned(1016, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 3) & "000";
            when 1 =>  -- step=16, saturate to 1008
                if v_pix(10) = '1' then return to_unsigned(1008, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 4) & "0000";
            when 2 =>  -- step=32, saturate to 992
                if v_pix(10) = '1' then return to_unsigned(992, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 5) & "00000";
            when 3 =>  -- step=48 = 16*3
                -- LUT: (pixel+offset)>>4 indexes directly into multiples of 48.
                -- Max index = (1023+31)>>4 = 65; all reachable values pre-computed.
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
            when 4 =>  -- step=64, saturate to 960
                if v_pix(10) = '1' then return to_unsigned(960, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 6) & "000000";
            when 5 =>  -- step=96 = 32*3
                -- LUT: (pixel+offset)>>5 indexes directly into multiples of 96.
                -- Max index = (1023+63)>>5 = 33; capped at 10*96=960 (11*96>1023).
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
            when 6 =>  -- step=128, saturate to 896
                if v_pix(10) = '1' then return to_unsigned(896, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 7) & "0000000";
            when others =>  -- step=256, saturate to 768
                if v_pix(10) = '1' then return to_unsigned(768, C_VIDEO_DATA_WIDTH); end if;
                return v_pix(9 downto 8) & "00000000";
        end case;
    end function;

    -- crush_to_lmask: maps step index to step-lower mask for Stage 0a pre-registration.
    --   Returns the largest (2^k - 1) strictly less than the step size.
    --   Registered in Stage 0a so apply_crush sees a registered lmask rather than
    --   a combinational case decode on its critical path.
    function crush_to_lmask(step_idx : integer range 0 to 7)
        return unsigned is
    begin
        case step_idx is
            when 0      => return to_unsigned(  7, C_VIDEO_DATA_WIDTH);  -- step 8
            when 1      => return to_unsigned( 15, C_VIDEO_DATA_WIDTH);  -- step 16
            when 2      => return to_unsigned( 31, C_VIDEO_DATA_WIDTH);  -- step 32
            when 3      => return to_unsigned( 31, C_VIDEO_DATA_WIDTH);  -- step 48 → use 31
            when 4      => return to_unsigned( 63, C_VIDEO_DATA_WIDTH);  -- step 64
            when 5      => return to_unsigned( 63, C_VIDEO_DATA_WIDTH);  -- step 96 → use 63
            when 6      => return to_unsigned(127, C_VIDEO_DATA_WIDTH);  -- step 128
            when others => return to_unsigned(255, C_VIDEO_DATA_WIDTH);  -- step 256
        end case;
    end function;

    -- crush_to_roundoff: maps step index to round offset (step/2) for Stage 0a pre-registration.
    --   Registered in Stage 0a so apply_crush sees a registered round_off rather than
    --   a combinational case decode on its critical path.
    function crush_to_roundoff(step_idx : integer range 0 to 7)
        return unsigned is
    begin
        case step_idx is
            when 0      => return to_unsigned(  4, C_VIDEO_DATA_WIDTH);
            when 1      => return to_unsigned(  8, C_VIDEO_DATA_WIDTH);
            when 2      => return to_unsigned( 16, C_VIDEO_DATA_WIDTH);
            when 3      => return to_unsigned( 24, C_VIDEO_DATA_WIDTH);
            when 4      => return to_unsigned( 32, C_VIDEO_DATA_WIDTH);
            when 5      => return to_unsigned( 48, C_VIDEO_DATA_WIDTH);
            when 6      => return to_unsigned( 64, C_VIDEO_DATA_WIDTH);
            when others => return to_unsigned(128, C_VIDEO_DATA_WIDTH);
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

    --------------------------------------------------------------------------------
    -- Control Signals (combinational from registers_in)
    --------------------------------------------------------------------------------
    signal s_bypass_enable  : std_logic;
    signal s_blend_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- LFSR signals (used for dither)
    signal s_lfsr10_out     : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_lfsr16_out     : std_logic_vector(15 downto 0);
    signal s_vsync_n_prev   : std_logic := '1';   -- previous vsync_n for edge detect
    signal s_lfsr_reset     : std_logic := '0';   -- pulses '1' for one clock at vsync falling edge

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode Outputs (T+1)
    -- Registered decoded controls and 1-cycle delayed data.
    --------------------------------------------------------------------------------
    signal s_crush_y_r      : integer range 0 to 7 := 0;
    signal s_crush_u_r      : integer range 0 to 7 := 0;
    signal s_crush_v_r      : integer range 0 to 7 := 0;
    signal s_round_u_r      : std_logic := '0';      -- '1' = On/Round
    signal s_round_v_r      : std_logic := '0';
    signal s_dither_r       : std_logic := '0';      -- '1' = On/Dither
    signal s_invert_r       : std_logic := '0';      -- '1' = On/Invert
    signal s_y_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_u_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_v_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_avid_d1        : std_logic := '0';
    -- Pre-registered step-lower mask and round offset for U and V.
    -- Computed in Stage 0a to remove their case decode trees from Stage 0b's
    -- critical path. Y always truncates so needs no lmask or round_off.
    signal s_lmask_u_r     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_roundoff_u_r  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_lmask_v_r     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_roundoff_v_r  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    -- Pre-registered near-black protect flag. Carry chain lives here in Stage 0a;
    -- Stage 0b sees only a 1-bit registered result.
    signal s_black_protect_r : std_logic := '0';
    -- Pre-registered TPDF dither values for U and V.
    -- Each is the sum of two independent noise sources right-shifted by 1,
    -- giving a triangular distribution over [0, max] instead of RPDF's uniform.
    signal s_tpdf_u_r : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tpdf_v_r : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 0b: Bit Crush (Y/U/V) Outputs (T+2)
    --------------------------------------------------------------------------------
    signal s_processed_y     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_u     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_v     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_valid : std_logic;

    -- 2-cycle delayed originals for per-channel blend dry input (aligned with T+2)
    signal s_orig_y_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_u_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_v_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Blend Outputs (T+6)
    --------------------------------------------------------------------------------
    signal s_blended_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_y_valid : std_logic;
    signal s_blended_u_valid : std_logic;
    signal s_blended_v_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Blend Outputs (T+10)
    --------------------------------------------------------------------------------
    signal s_global_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_y_valid  : std_logic;
    signal s_global_u_valid  : std_logic;
    signal s_global_v_valid  : std_logic;

    -- Original YUV delayed 6 clocks: T+0+6 = T+6, aligned with s_blended (global dry)
    signal s_y_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Bypass Path Delay Line Outputs (10 clocks)
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
    s_blend_y       <= unsigned(registers_in(3));
    s_blend_u       <= unsigned(registers_in(4));
    s_blend_v       <= unsigned(registers_in(5));
    s_bypass_enable <= registers_in(6)(4);
    s_global_blend  <= unsigned(registers_in(7));

    --------------------------------------------------------------------------------
    -- LFSR Modules (used for dither on U and V channels)
    -- lfsr16 free-runs at all times (period 65535).
    -- lfsr10 free-runs with a zero-safe seed derived from lfsr16.
    -- Both run with enable='1' every clock.
    -- U dither source: lfsr16[9:0]
    -- V dither source: lfsr10_out
    --------------------------------------------------------------------------------
    u_lfsr16 : entity work.lfsr16
        port map (clk    => clk,
                  enable => '1',
                  seed   => s_lfsr16_out,  -- feedback; load never pulses
                  load   => '0',
                  q      => s_lfsr16_out);

    u_lfsr10 : entity work.lfsr
        generic map (G_DATA_WIDTH => C_VIDEO_DATA_WIDTH)
        port map (clk      => clk,
                  reset    => s_lfsr_reset,  -- reseeds from lfsr16 at each vsync falling edge
                  enable   => '1',
                  seed     => s_lfsr16_out(C_VIDEO_DATA_WIDTH - 1 downto 1) & '1',  -- bit 0 forced high: prevents zero-seed lockup
                  poly     => "1001000000",
                  lfsr_out => s_lfsr10_out);

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Pre-registers crush amounts, brightness offset, and mode flags so Stage 0b
    -- sees only registered inputs on its critical path. Also delays data_in 1 clock.
    -- Also generates a one-clock s_lfsr_reset pulse at each vsync falling edge to
    -- unconditionally reseed lfsr10 from lfsr16, guaranteeing a non-zero start state.
    --------------------------------------------------------------------------------
    p_control_decode : process(clk)
    begin
        if rising_edge(clk) then
            -- Y/U/V crush amounts
            s_crush_y_r  <= knob_to_crush(unsigned(registers_in(0)));
            s_crush_u_r  <= knob_to_crush(unsigned(registers_in(1)));
            s_crush_v_r  <= knob_to_crush(unsigned(registers_in(2)));
            -- Pre-register step-lower mask and round offset for U and V.
            -- Keeps their case decode trees out of Stage 0b's critical path.
            s_lmask_u_r    <= crush_to_lmask(knob_to_crush(unsigned(registers_in(1))));
            s_roundoff_u_r <= crush_to_roundoff(knob_to_crush(unsigned(registers_in(1))));
            s_lmask_v_r    <= crush_to_lmask(knob_to_crush(unsigned(registers_in(2))));
            s_roundoff_v_r <= crush_to_roundoff(knob_to_crush(unsigned(registers_in(2))));
            -- Off='0'/On='1': feature active when bit='1' (On position)
            s_invert_r   <= registers_in(6)(0);  -- '1' = On/Invert
            s_dither_r   <= registers_in(6)(1);  -- '1' = On/Dither
            s_round_u_r  <= registers_in(6)(2);  -- '1' = On/Round U
            s_round_v_r  <= registers_in(6)(3);  -- '1' = On/Round V
            -- Pipeline data
            s_y_d1       <= data_in.y;
            s_u_d1       <= data_in.u;
            s_v_d1       <= data_in.v;
            s_avid_d1    <= data_in.avid;
            -- Pre-register near-black protect: carry chain is resolved here in Stage 0a.
            -- Result arrives registered at T+1, aligned with s_y_d1/s_u_d1/s_v_d1.
            s_black_protect_r <= '1' when unsigned(data_in.y) <= C_BLACK_THRESHOLD else '0';
            -- TPDF dither: sum two independent noise sources and halve (>> 1).
            -- U: lfsr16[9:0] + lfsr10_out (no overlapping bits).
            -- V: lfsr16[15:6] + lfsr10_out (upper slice of lfsr16, no overlap with U slice).
            s_tpdf_u_r <= resize(shift_right(
                resize(unsigned(s_lfsr16_out(C_VIDEO_DATA_WIDTH - 1 downto 0)), C_VIDEO_DATA_WIDTH + 1) +
                resize(unsigned(s_lfsr10_out), C_VIDEO_DATA_WIDTH + 1), 1), C_VIDEO_DATA_WIDTH);
            s_tpdf_v_r <= resize(shift_right(
                resize(unsigned(s_lfsr16_out(15 downto 6)), C_VIDEO_DATA_WIDTH + 1) +
                resize(unsigned(s_lfsr10_out), C_VIDEO_DATA_WIDTH + 1), 1), C_VIDEO_DATA_WIDTH);
            -- LFSR reseed: pulse reset for one clock at vsync falling edge.
            -- Reseeds lfsr10 from lfsr16 each frame, ensuring a valid non-zero
            -- start state regardless of FPGA power-on initialisation behaviour.
            s_vsync_n_prev <= data_in.vsync_n;
            if data_in.vsync_n = '0' and s_vsync_n_prev = '1' then
                s_lfsr_reset <= '1';
            else
                s_lfsr_reset <= '0';
            end if;
        end if;
    end process p_control_decode;

    --------------------------------------------------------------------------------
    -- Stage 0b: Bit Crush (Y/U/V)
    -- Latency: 1 clock. Input T+1, output T+2.
    -- Y: truncating crush only — quantised to step multiple, no round, no dither.
    -- U/V: apply_crush with optional round or dither.
    --   Note: round and dither are mutually exclusive — round takes priority.
    --   When dither is on and round is off, RPDF dither is applied before quantising.
    -- Invert: bitwise-NOT all three processed channels when s_invert_r='1'.
    -- Registers s_orig_*_d2 for per-channel blend dry input (aligned at T+2).
    --------------------------------------------------------------------------------
    p_bit_crush : process(clk)
        variable v_y, v_u, v_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_dither_u     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_dither_v     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Y: truncating crush (no round, no dither; lmask/round_off unused)
            v_y := apply_crush(unsigned(s_y_d1), s_crush_y_r, '0', (others => '0'), '0',
                               (others => '0'), (others => '0'));

            -- TPDF dither sources (pre-registered in Stage 0a; gating inside apply_crush)
            v_dither_u := s_tpdf_u_r;
            v_dither_v := s_tpdf_v_r;

            -- U/V: crush with optional round or dither; use pre-registered lmask/round_off
            v_u := apply_crush(unsigned(s_u_d1), s_crush_u_r, s_round_u_r,
                               v_dither_u, s_dither_r, s_lmask_u_r, s_roundoff_u_r);
            v_v := apply_crush(unsigned(s_v_d1), s_crush_v_r, s_round_v_r,
                               v_dither_v, s_dither_r, s_lmask_v_r, s_roundoff_v_r);

            -- Near-black protect: skip U and V crush when Y is at or below threshold.
            -- Large step sizes place U/V grid points far from neutral (512), so small
            -- signal-chain offsets in what should be black pixels quantize to a
            -- non-neutral chroma value and produce a green tint in the background.
            -- Y is still crushed normally (quantising near-zero Y toward 0 is fine).
            if s_black_protect_r = '1' then
                v_u := unsigned(s_u_d1);
                v_v := unsigned(s_v_d1);
            end if;

            -- Invert all three channels when On
            if s_invert_r = '1' then
                s_processed_y <= not v_y;
                s_processed_u <= not v_u;
                s_processed_v <= not v_v;
            else
                s_processed_y <= v_y;
                s_processed_u <= v_u;
                s_processed_v <= v_v;
            end if;

            s_processed_valid <= s_avid_d1;

            -- Per-channel dry inputs: delay original by 2 clocks (T+0->T+2)
            s_orig_y_d2 <= unsigned(s_y_d1);
            s_orig_u_d2 <= unsigned(s_u_d1);
            s_orig_v_d2 <= unsigned(s_v_d1);
        end if;
    end process p_bit_crush;

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Wet/Dry Blend
    -- Latency: 4 clocks. Input T+2, output T+6.
    --------------------------------------------------------------------------------
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_orig_y_d2, b=>s_processed_y, t=>s_blend_y,
                 result=>s_blended_y, valid=>s_blended_y_valid);

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_orig_u_d2, b=>s_processed_u, t=>s_blend_u,
                 result=>s_blended_u, valid=>s_blended_u_valid);

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_orig_v_d2, b=>s_processed_v, t=>s_blend_v,
                 result=>s_blended_v, valid=>s_blended_v_valid);

    --------------------------------------------------------------------------------
    -- Delay Line: YUV dry inputs for global blend stage.
    -- Single shift register of length C_PRE_GLOBAL_DELAY_CLKS (6 clocks).
    -- data_in at T+0 -> index 5 at T+6, aligned with s_blended output.
    --------------------------------------------------------------------------------
    p_global_dry_delay : process(clk)
        type t_data_delay is array (0 to C_PRE_GLOBAL_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_delay : t_data_delay := (others => (others => '0'));
        variable v_u_delay : t_data_delay := (others => (others => '0'));
        variable v_v_delay : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_y_delay := unsigned(data_in.y) & v_y_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_u_delay := unsigned(data_in.u) & v_u_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_v_delay := unsigned(data_in.v) & v_v_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            s_y_for_global <= v_y_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_u_for_global <= v_u_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_v_for_global <= v_v_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+6, output T+10.
    --------------------------------------------------------------------------------
    interp_global_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_y_valid,
                 a=>s_y_for_global, b=>s_blended_y, t=>s_global_blend,
                 result=>s_global_y, valid=>s_global_y_valid);

    interp_global_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_u_valid,
                 a=>s_u_for_global, b=>s_blended_u, t=>s_global_blend,
                 result=>s_global_u, valid=>s_global_u_valid);

    interp_global_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_v_valid,
                 a=>s_v_for_global, b=>s_blended_v, t=>s_global_blend,
                 result=>s_global_v, valid=>s_global_v_valid);

    --------------------------------------------------------------------------------
    -- Bypass Path Delay Line (10 clocks)
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
    s_io_align_in.y <= std_logic_vector(s_global_y) when s_bypass_enable = '0'
                  else s_y_delayed;
    s_io_align_in.u <= std_logic_vector(s_global_u) when s_bypass_enable = '0'
                  else s_u_delayed;
    s_io_align_in.v <= std_logic_vector(s_global_v) when s_bypass_enable = '0'
                  else s_v_delayed;

    s_io_align_in.avid <= s_global_y_valid;
    s_io_align_in.hsync_n <= s_hsync_n_delayed;
    s_io_align_in.vsync_n <= s_vsync_n_delayed;
    s_io_align_in.field_n <= s_field_n_delayed;

    -- AUTO_IO_ALIGN_PAD
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_pad_0 <= s_io_align_in;
            s_io_pad_1 <= s_io_pad_0;
        end if;
    end process p_io_align_pad;

    data_out.y      <= s_io_pad_1.y;
    data_out.u      <= s_io_pad_1.u;
    data_out.v      <= s_io_pad_1.v;
    data_out.avid   <= s_io_pad_1.avid;
    data_out.hsync_n<= s_io_pad_1.hsync_n;
    data_out.vsync_n<= s_io_pad_1.vsync_n;
    data_out.field_n<= s_io_pad_1.field_n;

end architecture yuv_bit_crush;
