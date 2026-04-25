-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   RGB Window Key
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Dual-function processor: per-channel RGB multi-mode filter + window keying.
--
--   PRIMARY FUNCTION: Multi-Mode Filtering
--   Per-channel frequency-domain filtering in RGB space using four filter types
--   controlled independently by knob settings:
--     - Low Pass:  attenuates frequencies above threshold
--     - High Pass: attenuates frequencies below threshold
--     - Band Pass: isolates frequencies between Low/High thresholds
--     - Notch:     rejects frequencies between Low/High thresholds
--
--   SECONDARY FUNCTION: Window Keying / Matte Processing
--   For each channel (R, G, B) a lower and upper threshold knob define a window.
--   When the lower threshold exceeds the upper threshold the window is inverted:
--   pixels outside the normal window pass.  This inversion happens per-channel
--   automatically, requiring no additional switch.
--
--   Matte Mode (S2/S3/S4 as a 3-bit word, S2=MSB, S4=LSB):
--     000  Logical OR    - white (1023) if any channel in-window, else black
--     001  Bitwise OR    - OR of channel values; failing channels contribute 0
--     010  Logical AND   - white (1023) if all channels in-window, else black
--     011  Bitwise AND   - AND of channel values; failing channels contribute 0
--     100  Luma          - BT.601 luma of the full pixel, gated by logical AND
--     101  LFSR synced   - frame-locked noise value, gated by logical OR
--     110  PRNG          - free-running noise value, gated by logical OR
--     111  Passthrough   - original pixel on all channels (no keying)
--
--   Show Matte (S1):
--     On  — outputs the computed matte as a greyscale signal on all three channels.
--           R=G=B so the downstream YUV conversion produces a true monochrome signal.
--     Off — uses the matte as a binary gate: pixels with matte>0 pass through as
--           the original RGB pixel; pixels with matte=0 are replaced with black.
--
--   Colour conversion uses the same pre-computed BT.601 BRAM tables as the
--   RGB Bit Crush / RGB Bit Rotator colour conversion path (11 BRAMs total).
--
-- Architecture:
--   Stage 0a    - YUV->RGB BRAM Lookup + control decode (1 clock) -> T+1
--   Stage 0a-ii - G-channel offset pre-sum               (1 clock) -> T+2
--   Stage 0b    - YUV->RGB Accumulate and Clamp          (1 clock) -> T+3
--   Stage 0c    - Luma Partial Products                  (1 clock) -> T+4
--   Stage 1a    - Window Checks + Luma Sum               (1 clock) -> T+5
--   Stage 1b    - Window Key Operation                   (1 clock) -> T+6
--   Stage 2     - Global Blend      (3x interpolator_u) (4 clocks) -> T+10
--   Stage 3a-i  - RGB->YUV BRAM Lookup                  (1 clock) -> T+11
--   Stage 3a-ii - RGB->YUV Partial Pre-sums             (1 clock) -> T+12
--   Stage 3a-iii- RGB->YUV Channel Sums                 (1 clock) -> T+13
--   Stage 3b    - RGB->YUV Clamp                         (1 clock) -> T+14
--
-- Videomancer UV Convention Note:
--   data_in.u / data_out.u = Cr (red-difference);  data_in.v / data_out.v = Cb.
--   U and V are swapped when indexing the BT.601 LUTs so all internal RGB
--   computations use the standard convention.
--
-- Register Map:
--   Register  0: R channel low threshold  (0-1023)   rotary_potentiometer_1
--   Register  1: G channel low threshold  (0-1023)   rotary_potentiometer_2
--   Register  2: B channel low threshold  (0-1023)   rotary_potentiometer_3
--   Register  3: R channel high threshold (0-1023)   rotary_potentiometer_4
--   Register  4: G channel high threshold (0-1023)   rotary_potentiometer_5
--   Register  5: B channel high threshold (0-1023)   rotary_potentiometer_6
--   Register  6: Packed toggle bits (Off='0', On='1'):
--     bit 0: Show Matte (0=Off/gate pixel, 1=On/show matte)            toggle_switch_7
--     bit 3: Matte Bit2 MSB of 3-bit matte mode {S2,S3,S4}             toggle_switch_8
--     bit 2: Matte Bit1 middle bit of matte mode word                  toggle_switch_9
--     bit 1: Matte Bit0 LSB of matte mode word                         toggle_switch_10
--     bit 4: Fine       (0=Normal full-range, 1=Fine 1/8-sensitivity)   toggle_switch_11
--   Register  7: Global blend (0=dry, 1023=wet)                    linear_potentiometer_12
--
--   Fine mode: on the Normal->Fine transition, the current knob positions are
--   latched as reference values.  Each knob then controls its threshold as a
--   weighted blend of the current position and the locked reference, giving 1/8
--   the sensitivity for precise adjustment anywhere in the 0-1023 range.
--     ctrl_fine = (in + 7 * in_ref) / 8
--   Switching back to Normal restores full-range control immediately.
--
-- Timing:
--   Total pipeline latency: 14 clock cycles.
--   Sync delay line is 14 clocks.
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

architecture rgb_window_key of program_top is

    constant C_PROCESSING_DELAY_CLKS : integer := 14;
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 3;   -- 3-clock registered dry tap (Stage 0c + Stage 1a + p_global_dry_delay)
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

    -- window_check: returns '1' if pixel is within the [lower, upper] window.
    --   Normal (lower <= upper): pass if lower <= pixel <= upper.
    --   Inverted (lower > upper): pass if pixel >= lower OR pixel <= upper.
    --   Crossing the knobs inverts the key naturally, per channel.
    function window_check(
        pixel : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        lower : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        upper : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return std_logic is
    begin
        if lower <= upper then
            if pixel >= lower and pixel <= upper then
                return '1';
            else
                return '0';
            end if;
        else
            -- Inverted window: pass values outside the (upper, lower) gap
            if pixel >= lower or pixel <= upper then
                return '1';
            else
                return '0';
            end if;
        end if;
    end function;

    -- fine_knob: 1/8-sensitivity knob computation for Fine mode.
    --   Returns (raw + 7*ref) / 8, keeping the result centred on the locked
    --   reference and giving 1/8 the travel sensitivity of the full range.
    --   Implemented as (raw + 8*ref - ref) / 8 using shift arithmetic (no
    --   multiplier inferred).  Intermediate raw+8*ref fits in 14 bits (max
    --   9207); final result is at most (1023 + 7*1023)/8 = 1023, always in range.
    function fine_knob(
        raw : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        ref : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return unsigned is
        variable v_sum : unsigned(13 downto 0);  -- 14-bit to hold raw + 8*ref
    begin
        v_sum := ("0000" & unsigned(raw))
               + ("0" & ref & "000")   -- ref * 8 (shift left 3)
               - ("0000" & ref);       -- subtract ref → raw + 7*ref
        return v_sum(12 downto 3);     -- divide by 8
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
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- Fine mode: registered S5 for edge detection; locked knob reference values
    signal s_fine           : std_logic := '0';
    type t_knob_refs is array (0 to 5) of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_in_ref         : t_knob_refs := (others => (others => '0'));

    -- Window key controls (registered in Stage 0a alongside BRAM lookup)
    signal s_low_r          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_low_g          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_low_b          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_high_r         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_high_g         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_high_b         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_show_matte     : std_logic := '0';  -- '1' = On/Show Matte
    signal s_matte_mode     : std_logic_vector(2 downto 0) := "010";  -- default: Logical AND
    -- LFSR / PRNG noise generators for matte modes 101 and 110
    signal s_vsync_n_d      : std_logic := '1';  -- registered vsync_n for edge detect
    signal s_hsync_n_d      : std_logic := '1';  -- registered hsync_n for edge detect
    signal s_lfsr           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := "0101010101";
    signal s_prng           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := "0011001100";

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
    -- Stage 0c: Luma Partial Products (T+4)
    -- Registers the three BT.601 multiply results separately to ease timing.
    --------------------------------------------------------------------------------
    signal s_0c_luma_r_prod  : unsigned(16 downto 0);   -- s_rgb_r * 77   (17-bit)
    signal s_0c_luma_g_prod  : unsigned(17 downto 0);   -- s_rgb_g * 150  (18-bit)
    signal s_0c_luma_b_prod  : unsigned(14 downto 0);   -- s_rgb_b * 29   (15-bit)
    signal s_0c_rgb_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_0c_rgb_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_0c_rgb_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_0c_rgb_valid    : std_logic;
    signal s_0c_show_matte   : std_logic;
    signal s_0c_matte_mode   : std_logic_vector(2 downto 0);
    signal s_0c_lfsr         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_0c_prng         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 1a: Window Checks + Luma Sum (T+5)
    --------------------------------------------------------------------------------
    signal s_wf_in_r         : std_logic;
    signal s_wf_in_g         : std_logic;
    signal s_wf_in_b         : std_logic;
    signal s_wf_in_any       : std_logic;
    signal s_wf_in_all       : std_logic;
    signal s_wf_r_m          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_g_m          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_b_m          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_luma         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_rgb_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_rgb_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_rgb_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_rgb_valid    : std_logic;
    signal s_wf_show_matte   : std_logic;
    signal s_wf_matte_mode   : std_logic_vector(2 downto 0);
    signal s_wf_lfsr         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_wf_prng         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 1b: Window Key Outputs (T+6)
    --------------------------------------------------------------------------------
    signal s_processed_r     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_g     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_b     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Blend Outputs (T+10)
    --------------------------------------------------------------------------------
    signal s_global_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_r_valid  : std_logic;
    signal s_global_g_valid  : std_logic;
    signal s_global_b_valid  : std_logic;

    -- RGB original delayed 3 clocks: T+3+3=T+6, aligned with s_processed (global blend dry)
    -- (Stage 0c + Stage 1a propagate the pixel; p_global_dry_delay adds the final register)
    signal s_r_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_g_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_b_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 3a-i: RGB->YUV BRAM Lookup Outputs (T+11)
    --------------------------------------------------------------------------------
    signal s_3a_ry_r    : integer := 0;   -- ry_r(R):  0..299  unsigned
    signal s_3a_ry_g    : integer := 0;   -- ry_g(G):  0..601  unsigned
    signal s_3a_ry_b    : integer := 0;   -- ry_b(B):  0..117  unsigned
    signal s_3a_ru_r    : integer := 0;   -- ru_r(R): -173..0  signed
    signal s_3a_ru_g    : integer := 0;   -- ru_g(G): -339..0  signed
    signal s_3a_rv_g    : integer := 0;   -- rv_g(G): -429..0  signed
    signal s_3a_rv_b    : integer := 0;   -- rv_b(B):  -83..0  signed
    -- Registered R and B for wire-shift operations in Stage 3a-ii
    signal s_3a_r_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_3a_b_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_3ai_valid  : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 3a-ii: RGB->YUV Partial Sums (T+12)
    --------------------------------------------------------------------------------
    signal s_3b_ry_rg   : integer := 0;   -- s_3a_ry_r + s_3a_ry_g
    signal s_3b_ry_b    : integer := 0;   -- s_3a_ry_b (pass-through)
    signal s_3b_ru_rg   : integer := 0;   -- s_3a_ru_r + s_3a_ru_g
    signal s_3b_rv_gb   : integer := 0;   -- s_3a_rv_g + s_3a_rv_b
    signal s_3b_r_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_3b_b_d     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_3b_valid   : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 3a-iii: RGB->YUV Channel Sums (T+13)
    --------------------------------------------------------------------------------
    signal s_ry_y_sum   : integer := 0;
    signal s_ry_u_sum   : integer := 0;
    signal s_ry_v_sum   : integer := 0;
    signal s_ry_valid   : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 3b: RGB->YUV Output (T+14)
    --------------------------------------------------------------------------------
    signal s_yuv_out_y      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_yuv_out_u      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_yuv_out_v      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_yuv_out_valid  : std_logic;

    --------------------------------------------------------------------------------
    -- Sync Delay Line (14 clocks — aligns hsync/vsync/field with pipeline output)
    --------------------------------------------------------------------------------
    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;

begin

    --------------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------------
    s_global_blend  <= unsigned(registers_in(7));  -- linear_potentiometer_12

    --------------------------------------------------------------------------------
    -- Stage 0a: YUV->RGB BRAM Lookup + Control Decode
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Reads 4 pre-computed offsets (already divided by 1024) from lookup tables.
    -- Critical path: address decode + BRAM read (fast, no multiply chains).
    -- Hardware switch polarity: Off='0', On='1' (direct bit value).
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
            -- Off='0'/On='1': feature active when bit='1' (On position).
            s_show_matte <= registers_in(6)(0);          -- '1' = On/Show Matte
            -- S2=bit3=MSB, S3=bit2, S4=bit1=LSB → concatenate to preserve {S2,S3,S4} order
            s_matte_mode <= registers_in(6)(3) & registers_in(6)(2) & registers_in(6)(1);
            -- Fine mode: register S5; latch reference values on Normal->Fine transition
            s_fine <= registers_in(6)(4);
            if s_fine = '0' and registers_in(6)(4) = '1' then
                s_in_ref(0) <= unsigned(registers_in(0));
                s_in_ref(1) <= unsigned(registers_in(1));
                s_in_ref(2) <= unsigned(registers_in(2));
                s_in_ref(3) <= unsigned(registers_in(3));
                s_in_ref(4) <= unsigned(registers_in(4));
                s_in_ref(5) <= unsigned(registers_in(5));
            end if;
            -- Compute threshold values: 1/8-sensitivity in Fine mode, full range Normal
            if registers_in(6)(4) = '1' then
                s_low_r  <= fine_knob(registers_in(0), s_in_ref(0));
                s_low_g  <= fine_knob(registers_in(1), s_in_ref(1));
                s_low_b  <= fine_knob(registers_in(2), s_in_ref(2));
                s_high_r <= fine_knob(registers_in(3), s_in_ref(3));
                s_high_g <= fine_knob(registers_in(4), s_in_ref(4));
                s_high_b <= fine_knob(registers_in(5), s_in_ref(5));
            else
                s_low_r  <= unsigned(registers_in(0));
                s_low_g  <= unsigned(registers_in(1));
                s_low_b  <= unsigned(registers_in(2));
                s_high_r <= unsigned(registers_in(3));
                s_high_g <= unsigned(registers_in(4));
                s_high_b <= unsigned(registers_in(5));
            end if;
        end if;
    end process p_yuv_rgb_lut;

    --------------------------------------------------------------------------------
    -- Stage 0a-ii: G-channel Pre-Sum
    -- Latency: 1 clock. Input T+1, output T+2.
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
    -- LFSR Noise Generator (frame-synced)
    -- 10-bit Fibonacci LFSR, polynomial x^10 + x^7 + 1 (primitive, period 1023).
    -- Reseeds from the current PRNG state on the falling edge of vsync_n so that
    -- each frame produces a different base pattern.
    -- Output is XOR'd with the BT.601 luma of the pixel in p_window_key to make
    -- the noise content-dependent (each luma level yields a different texture).
    -- Used by matte mode 101; gating by logical OR in p_window_key ensures that
    -- only in-window pixels receive the noise value.
    --------------------------------------------------------------------------------
    p_lfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            s_vsync_n_d <= data_in.vsync_n;
            if s_vsync_n_d = '1' and data_in.vsync_n = '0' then
                -- Reseed from PRNG so each frame starts with a different pattern
                s_lfsr <= s_prng;
            else
                v_fb   := s_lfsr(9) xor s_lfsr(6);
                s_lfsr <= s_lfsr(8 downto 0) & v_fb;
            end if;
        end if;
    end process p_lfsr;

    --------------------------------------------------------------------------------
    -- PRNG Noise Generator (line-seeded)
    -- Same polynomial as the LFSR.  Reseeds from the current LFSR state on each
    -- falling edge of hsync_n so that every line starts with a different sequence.
    -- Because the LFSR itself varies each frame, the PRNG pattern is unique across
    -- both lines and frames.  Output is XOR'd with luma in p_window_key.
    -- Used by matte mode 110.
    --------------------------------------------------------------------------------
    p_prng : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            s_hsync_n_d <= data_in.hsync_n;
            if s_hsync_n_d = '1' and data_in.hsync_n = '0' then
                -- Reseed from LFSR at each line start for per-line variation
                s_prng <= s_lfsr;
            else
                v_fb   := s_prng(9) xor s_prng(6);
                s_prng <= s_prng(8 downto 0) & v_fb;
            end if;
        end if;
    end process p_prng;

    --------------------------------------------------------------------------------
    -- Stage 0c: Luma Partial Products
    -- Latency: 1 clock. Input T+3, output T+4.
    -- Registers each of the three BT.601 multiply results in its own flip-flop
    -- stage, isolating the carry chains from the subsequent addition and window
    -- comparison logic.  Also propagates pixel, valid, and control signals.
    --------------------------------------------------------------------------------
    p_luma_mult : process(clk)
    begin
        if rising_edge(clk) then
            s_0c_luma_r_prod <= s_rgb_r * to_unsigned(77,  7);   -- 10+7 = 17-bit
            s_0c_luma_g_prod <= s_rgb_g * to_unsigned(150, 8);   -- 10+8 = 18-bit
            s_0c_luma_b_prod <= s_rgb_b * to_unsigned(29,  5);   -- 10+5 = 15-bit
            s_0c_rgb_r       <= s_rgb_r;
            s_0c_rgb_g       <= s_rgb_g;
            s_0c_rgb_b       <= s_rgb_b;
            s_0c_rgb_valid   <= s_rgb_valid;
            s_0c_show_matte  <= s_show_matte;
            s_0c_matte_mode  <= s_matte_mode;
            s_0c_lfsr        <= s_lfsr;
            s_0c_prng        <= s_prng;
        end if;
    end process p_luma_mult;

    --------------------------------------------------------------------------------
    -- Stage 1a: Window Checks + Luma Sum
    -- Latency: 1 clock. Input T+4, output T+5.
    -- Runs the three per-channel window comparisons, forms masked values, and
    -- sums the registered luma partial products into a 10-bit luma result.
    -- All outputs are registered so Stage 1b sees only flip-flop outputs.
    --------------------------------------------------------------------------------
    p_window_check : process(clk)
        variable v_in_r, v_in_g, v_in_b : std_logic;
        variable v_luma_wide             : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- Per-channel window comparisons
            v_in_r := window_check(s_0c_rgb_r, s_low_r, s_high_r);
            v_in_g := window_check(s_0c_rgb_g, s_low_g, s_high_g);
            v_in_b := window_check(s_0c_rgb_b, s_low_b, s_high_b);

            s_wf_in_r   <= v_in_r;
            s_wf_in_g   <= v_in_g;
            s_wf_in_b   <= v_in_b;
            s_wf_in_any <= v_in_r or  v_in_g or  v_in_b;
            s_wf_in_all <= v_in_r and v_in_g and v_in_b;

            -- Masked values: failing channel contributes 0
            if v_in_r = '1' then s_wf_r_m <= s_0c_rgb_r; else s_wf_r_m <= (others => '0'); end if;
            if v_in_g = '1' then s_wf_g_m <= s_0c_rgb_g; else s_wf_g_m <= (others => '0'); end if;
            if v_in_b = '1' then s_wf_b_m <= s_0c_rgb_b; else s_wf_b_m <= (others => '0'); end if;

            -- Sum luma partial products; take bits [17:8] for 10-bit BT.601 luma
            -- Max sum: 1023*77 + 1023*150 + 1023*29 = 261888 < 2^18 = 262144
            v_luma_wide := resize(s_0c_luma_r_prod, 18)
                         + resize(s_0c_luma_g_prod, 18)
                         + resize(s_0c_luma_b_prod, 18);
            s_wf_luma <= v_luma_wide(17 downto 8);

            -- Propagate pixel and controls to Stage 1b
            s_wf_rgb_r      <= s_0c_rgb_r;
            s_wf_rgb_g      <= s_0c_rgb_g;
            s_wf_rgb_b      <= s_0c_rgb_b;
            s_wf_rgb_valid  <= s_0c_rgb_valid;
            s_wf_show_matte <= s_0c_show_matte;
            s_wf_matte_mode <= s_0c_matte_mode;
            s_wf_lfsr       <= s_0c_lfsr;
            s_wf_prng       <= s_0c_prng;
        end if;
    end process p_window_check;

    --------------------------------------------------------------------------------
    -- Stage 1b: Window Key Operation
    -- Latency: 1 clock. Input T+5, output T+6.
    --
    -- All inputs are registered flip-flop outputs from Stage 1a: no comparisons,
    -- multiplications, or additions longer than a simple mux in this stage.
    --
    -- Matte mode (s_wf_matte_mode = S2 & S3 & S4, S2=MSB):
    --   "000" Logical OR  : 1023 if any channel in-window (OR), else 0
    --   "001" Bitwise OR  : OR of channel values, failing channels = 0
    --   "010" Logical AND : 1023 if all channels in-window (AND), else 0
    --   "011" Bitwise AND : AND of channel values, failing channels = 0
    --   "100" Luma        : BT.601 luma of full pixel, gated by AND
    --   "101" LFSR synced : frame-seeded noise XOR luma, gated by OR
    --   "110" PRNG        : line-seeded noise XOR luma, gated by OR
    --   "111" Passthrough : original pixel, no keying
    --
    -- Show Matte On  : matte value output on all three channels (greyscale)
    -- Show Matte Off : matte > 0 passes original pixel; matte = 0 outputs black
    --------------------------------------------------------------------------------
    p_window_key : process(clk)
        variable v_matte : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Compute matte value based on selected mode
            case s_wf_matte_mode is
                when "000" =>  -- Logical OR: B/W, any channel in-window passes
                    if s_wf_in_any = '1' then
                        v_matte := to_unsigned(1023, C_VIDEO_DATA_WIDTH);
                    else
                        v_matte := (others => '0');
                    end if;
                when "001" =>  -- Bitwise OR: OR of masked values
                    v_matte := s_wf_r_m or s_wf_g_m or s_wf_b_m;
                when "010" =>  -- Logical AND: B/W, all channels in-window pass
                    if s_wf_in_all = '1' then
                        v_matte := to_unsigned(1023, C_VIDEO_DATA_WIDTH);
                    else
                        v_matte := (others => '0');
                    end if;
                when "011" =>  -- Bitwise AND: AND of masked values
                    v_matte := s_wf_r_m and s_wf_g_m and s_wf_b_m;
                when "100" =>  -- Luma: BT.601, gated by AND
                    if s_wf_in_all = '1' then
                        v_matte := s_wf_luma;
                    else
                        v_matte := (others => '0');
                    end if;
                when "101" =>  -- LFSR synced: frame-seeded noise XOR luma, gated by OR
                    if s_wf_in_any = '1' then
                        v_matte := unsigned(s_wf_lfsr) xor s_wf_luma;
                    else
                        v_matte := (others => '0');
                    end if;
                when "110" =>  -- PRNG: line-seeded noise XOR luma, gated by OR
                    if s_wf_in_any = '1' then
                        v_matte := unsigned(s_wf_prng) xor s_wf_luma;
                    else
                        v_matte := (others => '0');
                    end if;
                when others =>  -- "111" Passthrough: original pixel, no keying
                    v_matte := (others => '0');  -- unused; handled below
            end case;

            -- Drive outputs
            if s_wf_matte_mode = "111" then
                -- Passthrough: output original pixel on all three channels
                s_processed_r <= s_wf_rgb_r;
                s_processed_g <= s_wf_rgb_g;
                s_processed_b <= s_wf_rgb_b;
            elsif s_wf_show_matte = '1' then
                -- Show Matte: greyscale matte on all channels (R=G=B → true mono YUV)
                s_processed_r <= v_matte;
                s_processed_g <= v_matte;
                s_processed_b <= v_matte;
            else
                -- Normal: gate original pixel with matte
                if v_matte /= 0 then
                    s_processed_r <= s_wf_rgb_r;
                    s_processed_g <= s_wf_rgb_g;
                    s_processed_b <= s_wf_rgb_b;
                else
                    s_processed_r <= (others => '0');
                    s_processed_g <= (others => '0');
                    s_processed_b <= (others => '0');
                end if;
            end if;

            s_processed_valid <= s_wf_rgb_valid;
        end if;
    end process p_window_key;

    --------------------------------------------------------------------------------
    -- Delay Line: RGB dry input for global blend.
    -- 1-clock registered delay: T+5+1=T+6, aligned with s_processed (Stage 1b).
    -- The pixel was already propagated through Stage 0c (T+3→T+4) and Stage 1a
    -- (T+4→T+5) via s_0c_rgb_* and s_wf_rgb_*; this final register aligns the
    -- dry path with the Stage 1b wet output at T+6.
    --------------------------------------------------------------------------------
    p_global_dry_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_r_for_global <= s_wf_rgb_r;
            s_g_for_global <= s_wf_rgb_g;
            s_b_for_global <= s_wf_rgb_b;
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+6, output T+10.
    --------------------------------------------------------------------------------
    interp_global_r : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_r_for_global, b=>s_processed_r, t=>s_global_blend,
                 result=>s_global_r, valid=>s_global_r_valid);

    interp_global_g : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_g_for_global, b=>s_processed_g, t=>s_global_blend,
                 result=>s_global_g, valid=>s_global_g_valid);

    interp_global_b : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_b_for_global, b=>s_processed_b, t=>s_global_blend,
                 result=>s_global_b, valid=>s_global_b_valid);

    --------------------------------------------------------------------------------
    -- Stage 3a-i: RGB->YUV BRAM Lookup
    -- Latency: 1 clock. Input T+10, output T+11.
    -- Reads 7 pre-computed partial values from BRAM.
    -- u_b(B) = B>>1 and v_r(R) = R>>1 are implemented as wire shifts below.
    --------------------------------------------------------------------------------
    p_rgb_yuv_lut : process(clk)
    begin
        if rising_edge(clk) then
            s_3a_ry_r  <= to_integer(unsigned(s_lut_ry_r(to_integer(s_global_r))));
            s_3a_ry_g  <= to_integer(unsigned(s_lut_ry_g(to_integer(s_global_g))));
            s_3a_ry_b  <= to_integer(unsigned(s_lut_ry_b(to_integer(s_global_b))));
            s_3a_ru_r  <= to_integer(signed(  s_lut_ru_r(to_integer(s_global_r))));
            s_3a_ru_g  <= to_integer(signed(  s_lut_ru_g(to_integer(s_global_g))));
            s_3a_rv_g  <= to_integer(signed(  s_lut_rv_g(to_integer(s_global_g))));
            s_3a_rv_b  <= to_integer(signed(  s_lut_rv_b(to_integer(s_global_b))));
            s_3a_r_d   <= s_global_r;
            s_3a_b_d   <= s_global_b;
            s_3ai_valid <= s_global_r_valid;
        end if;
    end process p_rgb_yuv_lut;

    --------------------------------------------------------------------------------
    -- Stage 3a-ii: RGB->YUV Partial Sums
    -- Latency: 1 clock. Input T+11, output T+12.
    --------------------------------------------------------------------------------
    p_rgb_yuv_presum : process(clk)
    begin
        if rising_edge(clk) then
            s_3b_ry_rg <= s_3a_ry_r + s_3a_ry_g;
            s_3b_ry_b  <= s_3a_ry_b;
            s_3b_ru_rg <= s_3a_ru_r + s_3a_ru_g;
            s_3b_rv_gb <= s_3a_rv_g + s_3a_rv_b;
            s_3b_r_d   <= s_3a_r_d;
            s_3b_b_d   <= s_3a_b_d;
            s_3b_valid <= s_3ai_valid;
        end if;
    end process p_rgb_yuv_presum;

    --------------------------------------------------------------------------------
    -- Stage 3a-iii: RGB->YUV Channel Sums
    -- Latency: 1 clock. Input T+12, output T+13.
    --------------------------------------------------------------------------------
    p_rgb_yuv_sum : process(clk)
    begin
        if rising_edge(clk) then
            s_ry_y_sum <= s_3b_ry_rg + s_3b_ry_b;
            s_ry_u_sum <= s_3b_ru_rg
                          + to_integer(unsigned('0' & s_3b_b_d(9 downto 1)));
            s_ry_v_sum <= to_integer(unsigned('0' & s_3b_r_d(9 downto 1)))
                          + s_3b_rv_gb;
            s_ry_valid <= s_3b_valid;
        end if;
    end process p_rgb_yuv_sum;

    --------------------------------------------------------------------------------
    -- Stage 3b: RGB->YUV Clamp
    -- Latency: 1 clock. Input T+13, output T+14.
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
    -- Sync Delay Line (14 clocks — aligns hsync/vsync/field with pipeline output)
    --------------------------------------------------------------------------------
    p_sync_delay : process(clk)
        type t_sync_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
    begin
        if rising_edge(clk) then
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_hsync_n_delayed <= v_hsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_vsync_n_delayed <= v_vsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_field_n_delayed <= v_field_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_sync_delay;

    --------------------------------------------------------------------------------
    -- Output
    --------------------------------------------------------------------------------
    data_out.y       <= std_logic_vector(s_yuv_out_y);
    data_out.u       <= std_logic_vector(s_yuv_out_u);
    data_out.v       <= std_logic_vector(s_yuv_out_v);
    data_out.avid    <= s_yuv_out_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end architecture rgb_window_key;
