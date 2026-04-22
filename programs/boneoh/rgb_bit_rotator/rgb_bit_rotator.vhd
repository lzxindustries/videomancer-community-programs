-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   RGB Bit Rotator (BRAM Edition)
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Per-channel bit rotation effect with full YUV<->RGB colour space conversion.
--   Colour conversion uses pre-computed BT.601 full-range lookup tables stored in
--   ice40 block RAM (BRAM), replacing the multiplier trees of the original version.
--   This reduces LC utilisation from ~92% to ~83%, giving the router enough room
--   to meet HD timing (74.25 MHz) on the ice40 hx4k.
--
--   11 BRAM lookup tables (4 for YUV->RGB, 7 for RGB->YUV).
--   u_b(B) = B>>1 and v_r(R) = R>>1 are wire connections (no BRAM needed).
--   Estimated BRAM usage: 31 of 32 available blocks.
--
-- Architecture:
--   Stage 0a - YUV->RGB BRAM Lookup (1 clock):
--     - Read 4 pre-computed offsets from BRAM (indexed by U and V)
--     - Delay Y and avid by 1 clock for Stage 0b
--
--   Stage 0b - YUV->RGB Accumulate (1 clock):
--     - R = clamp(Y + r_offset)
--     - G = clamp(Y + gu_offset + gv_offset)
--     - B = clamp(Y + b_offset)
--     (No division needed; offsets are pre-divided in the LUT)
--
--   Stage 1a - Bit Rotation + dark suppress threshold comparison (1 clock)
--   Stage 1b - Dark suppress gate: zero channels at or below threshold (1 clock)
--   Stage 2 - Per-Channel Blend (4 clocks, 3x interpolator_u)
--   Stage 3 - Global Blend (4 clocks, 3x interpolator_u)
--
--   Stage 4a-i - RGB->YUV BRAM Lookup (1 clock):
--     - Read 7 pre-computed partial values from BRAM (indexed by R, G, or B)
--     - Register R and B for wire-shift use in Stage 4a-ii
--
--   Stage 4a-ii - RGB->YUV Channel Sums (1 clock):
--     - Y_sum = ry_r + ry_g + ry_b
--     - U_sum = ru_r + ru_g + (B>>1)      [B>>1 is a wire]
--     - V_sum = (R>>1) + rv_g + rv_b      [R>>1 is a wire]
--
--   Stage 4b - RGB->YUV Accumulate (1 clock):
--     - Y = clamp(Y_sum)
--     - U = clamp(U_sum + 512)
--     - V = clamp(V_sum + 512)
--     (No division needed; values are pre-divided in the LUT)
--
-- Videomancer UV Convention Note:
--   The Videomancer SDK uses a non-standard UV convention:
--     data_in.u / data_out.u  = Cr  (red-difference chrominance)
--     data_in.v / data_out.v  = Cb  (blue-difference chrominance)
--   This is the OPPOSITE of standard BT.601 (U=Cb, V=Cr).
--   The Stage 0a and Stage 4b code compensates by swapping U/V when indexing the
--   BT.601 LUTs, so all internal RGB computations are correct.
--
-- Register Map:
--   Register  0: R rotation amount (0-1023, maps to 0-10 bits)  rotary_potentiometer_1
--   Register  1: G rotation amount (0-1023, maps to 0-10 bits)  rotary_potentiometer_2
--   Register  2: B rotation amount (0-1023, maps to 0-10 bits)  rotary_potentiometer_3
--   Register  3: R channel blend   (0=fully dry, 1023=fully wet) rotary_potentiometer_4
--   Register  4: G channel blend   (0=fully dry, 1023=fully wet) rotary_potentiometer_5
--   Register  5: B channel blend   (0=fully dry, 1023=fully wet) rotary_potentiometer_6
--   Register  6: Packed toggle bits (one bit per switch):
--     bit 0: Direction     (0=ROL, 1=ROR)          toggle_switch_7
--     bit 1: Cutoff S1  (MSB, 0=Off, 1=On)         toggle_switch_8
--     bit 2: Cutoff S2  (0=Off, 1=On)              toggle_switch_9
--     bit 3: Cutoff S3  (LSB, 0=Off, 1=On)         toggle_switch_10
--     bit 4: Bypass enable (0=Process, 1=Bypass)   toggle_switch_11
--   Register  7: Global blend  (0=fully dry, 1023=fully wet) linear_potentiometer_12
--
-- Timing:
--   Total pipeline latency: 14 clock cycles (unchanged from multiplier version)
--     Stage 0a (YUV->RGB BRAM lookup):    1 clock  -> T+1
--     Stage 0b (YUV->RGB accumulate):     1 clock  -> T+2
--     Stage 1  (mask + rotate):           1 clock  -> T+3
--     Stage 2  (per-channel blend):       4 clocks -> T+7
--     Stage 3  (global blend):            4 clocks -> T+11
--     Stage 4a-i  (RGB->YUV BRAM lookup): 1 clock  -> T+12
--     Stage 4a-ii (RGB->YUV sums):        1 clock  -> T+13
--     Stage 4b  (RGB->YUV accumulate):    1 clock  -> T+14
--
-- Output avid (data_out.avid = s_yuv_out_valid, end of pipeline valid chain):
--   data_out.avid is driven from a 14-clock delay of data_in.avid
--   (same delay line as hsync/vsync/field), NOT from s_yuv_out_valid.
--   This ensures correct avid output independent of the interpolator
--   and BRAM valid chains.
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

architecture rgb_bit_rotator of program_top is

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 17;  -- +1 for Stage 4a-ii pre-sum, +1 for Stage 0a-ii pre-sum
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 6;
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

    function raw_to_shift(raw : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return integer is
        variable v : integer;
    begin
        v := to_integer(raw);
        if    v <  52 then return 0;
        elsif v < 154 then return 1;
        elsif v < 256 then return 2;
        elsif v < 359 then return 3;
        elsif v < 461 then return 4;
        elsif v < 563 then return 5;
        elsif v < 666 then return 6;
        elsif v < 768 then return 7;
        elsif v < 870 then return 8;
        elsif v < 972 then return 9;
        else               return 10;
        end if;
    end function;

    function rol10(value : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                   n     : integer)
        return unsigned is
        variable m : integer;
    begin
        m := n mod 10;
        case m is
            when 0  => return value;
            when 1  => return value(8 downto 0) & value(9);
            when 2  => return value(7 downto 0) & value(9 downto 8);
            when 3  => return value(6 downto 0) & value(9 downto 7);
            when 4  => return value(5 downto 0) & value(9 downto 6);
            when 5  => return value(4 downto 0) & value(9 downto 5);
            when 6  => return value(3 downto 0) & value(9 downto 4);
            when 7  => return value(2 downto 0) & value(9 downto 3);
            when 8  => return value(1 downto 0) & value(9 downto 2);
            when 9  => return value(0)           & value(9 downto 1);
            when others => return value;
        end case;
    end function;

    function ror10(value : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                   n     : integer)
        return unsigned is
        variable m : integer;
    begin
        m := n mod 10;
        if m = 0 then return value;
        else          return rol10(value, 10 - m);
        end if;
    end function;

    -- Dark suppress threshold: values <= threshold are gated to zero before output.
    -- s1 (Cutoff S1 / hardware S2) is MSB — highest-impact switch.
    -- 000 = all pass; each step doubles the suppressed range.
    function get_threshold(s1, s2, s3 : std_logic)
        return unsigned is
    begin
        case std_logic_vector'(s1 & s2 & s3) is
            when "000"  => return to_unsigned(0,   C_VIDEO_DATA_WIDTH);  -- all pass
            when "001"  => return to_unsigned(3,   C_VIDEO_DATA_WIDTH);  -- suppress 0–3
            when "010"  => return to_unsigned(15,  C_VIDEO_DATA_WIDTH);  -- suppress 0–15
            when "011"  => return to_unsigned(31,  C_VIDEO_DATA_WIDTH);  -- suppress 0–31
            when "100"  => return to_unsigned(63,  C_VIDEO_DATA_WIDTH);  -- suppress 0–63
            when "101"  => return to_unsigned(127, C_VIDEO_DATA_WIDTH);  -- suppress 0–127
            when "110"  => return to_unsigned(255, C_VIDEO_DATA_WIDTH);  -- suppress 0–255
            when "111"  => return to_unsigned(511, C_VIDEO_DATA_WIDTH);  -- suppress 0–511
            when others => return to_unsigned(0,   C_VIDEO_DATA_WIDTH);
        end case;
    end function;

    -- Converts a raw shift amount (0-10) and direction flag into an effective
    -- ROL-only shift (0-9).  Since ror10(x,k) = rol10(x, 10-k), folding
    -- direction into the shift amount in Stage 0a lets Stage 1a use a single
    -- 10-way ROL mux instead of a 20-way ROL/ROR mux, saving ~1 LUT level on
    -- the critical path.
    function to_eff_shift(shift : integer; direction : std_logic)
        return integer is
        variable m : integer;
    begin
        m := shift mod 10;          -- normalise to 0-9 (handles the "return 10" edge case)
        if direction = '0' then     -- ROL: use shift directly
            return m;
        else                        -- ROR k = ROL (10-k); ROR 0 = ROL 0
            return (10 - m) mod 10;
        end if;
    end function;

    function clamp10(v : integer) return unsigned is
    begin
        if    v <    0 then return to_unsigned(0,    C_VIDEO_DATA_WIDTH);
        elsif v > 1023 then return to_unsigned(1023, C_VIDEO_DATA_WIDTH);
        else                return to_unsigned(v,    C_VIDEO_DATA_WIDTH);
        end if;
    end function;

    -- clamp10_uv: clamp10(v + C_UV_OFFSET) with the offset folded into the
    -- comparator bounds as compile-time constants.  This lets synthesis evaluate
    -- the saturation checks directly on v (no adder on the compare critical path)
    -- and only compute v + C_UV_OFFSET in the non-saturating else branch, which
    -- is independent of the comparators.
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
    signal s_direction      : std_logic;
    signal s_cutoff_s1      : std_logic;
    signal s_cutoff_s2      : std_logic;
    signal s_cutoff_s3      : std_logic;
    signal s_blend_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

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
    -- Pre-registered decoded controls (registered in Stage 0a alongside BRAM lookup).
    -- Direction is folded into the shift via to_eff_shift(), so Stage 1a only
    -- needs a single 10-way ROL mux instead of a 20-way ROL/ROR mux.
    signal s_eff_shift_r : integer range 0 to 9 := 0;
    signal s_eff_shift_g : integer range 0 to 9 := 0;
    signal s_eff_shift_b : integer range 0 to 9 := 0;
    signal s_threshold   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 0b: YUV->RGB Accumulate Outputs (T+2)
    --------------------------------------------------------------------------------
    signal s_rgb_r      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rgb_g      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rgb_b      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rgb_valid  : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 1: Rotation Outputs (T+3)
    --------------------------------------------------------------------------------
    -- Stage 1a: Rotation outputs (T+3) — rotation + per-channel above-threshold flags
    signal s_rotated_pre_r     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_pre_g     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_pre_b     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_pre_valid : std_logic;
    -- Per-channel above-threshold flags (registered alongside rotation in Stage 1a)
    signal s_above_r           : std_logic := '0';
    signal s_above_g           : std_logic := '0';
    signal s_above_b           : std_logic := '0';
    -- Stage 1b: Dark suppress gate (T+4) — zero out channels at or below threshold
    signal s_rotated_r      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_g      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_b      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_valid  : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Per-Channel Blend Outputs (T+7)
    --------------------------------------------------------------------------------
    signal s_blended_r       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_g       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_b       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_r_valid : std_logic;
    signal s_blended_g_valid : std_logic;
    signal s_blended_b_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 3: Global Blend Outputs (T+11)
    --------------------------------------------------------------------------------
    signal s_global_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_g        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_b        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_r_valid  : std_logic;
    signal s_global_g_valid  : std_logic;
    signal s_global_b_valid  : std_logic;

    -- RGB original delayed 2 clocks: T+2+2=T+4, aligned with s_rotated (per-channel blend dry)
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
    s_direction     <= registers_in(6)(0);   -- toggle_switch_7: 0=ROL, 1=ROR
    s_cutoff_s1     <= registers_in(6)(1);   -- toggle_switch_8: 0=Off, 1=On
    s_cutoff_s2     <= registers_in(6)(2);   -- toggle_switch_9: 0=Off, 1=On
    s_cutoff_s3     <= registers_in(6)(3);   -- toggle_switch_10: 0=Off, 1=On
    s_bypass_enable <= registers_in(6)(4);   -- toggle_switch_11: 0=Process, 1=Bypass
    s_global_blend  <= unsigned(registers_in(7));  -- linear_potentiometer_12

    --------------------------------------------------------------------------------
    -- Stage 0a: YUV->RGB BRAM Lookup
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Reads 4 pre-computed offsets (already divided by 1024) from lookup tables.
    -- Critical path: address decode + BRAM read (fast, no multiply chains).
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
            -- Direction is folded into the shift here via to_eff_shift(), so Stage 1a
            -- only needs a 10-way ROL mux instead of a 20-way ROL/ROR mux.
            s_eff_shift_r <= to_eff_shift(raw_to_shift(unsigned(registers_in(0))), registers_in(6)(0));
            s_eff_shift_g <= to_eff_shift(raw_to_shift(unsigned(registers_in(1))), registers_in(6)(0));
            s_eff_shift_b <= to_eff_shift(raw_to_shift(unsigned(registers_in(2))), registers_in(6)(0));
            s_threshold   <= get_threshold(registers_in(6)(1), registers_in(6)(2),
                                           registers_in(6)(3));
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
    -- Stage 1a: Bit Rotation + Dark Suppress Threshold Comparison
    -- Latency: 1 clock. Input T+2, output T+3.
    -- Critical path: data -> 10-way ROL mux -> register (same as before).
    -- Above-threshold flags are registered in parallel: 10-bit comparator on
    -- registered inputs, independent of the ROL mux critical path.
    -- Stage 1b uses the flags to gate the output (pure mux, no carry chain).
    --------------------------------------------------------------------------------
    p_rotation_stage : process(clk)
    begin
        if rising_edge(clk) then
            s_rotated_pre_r <= rol10(s_rgb_r, s_eff_shift_r);
            s_rotated_pre_g <= rol10(s_rgb_g, s_eff_shift_g);
            s_rotated_pre_b <= rol10(s_rgb_b, s_eff_shift_b);
            -- Per-channel above-threshold flags: registered here alongside rotation.
            -- Stage 1b uses these to gate the rotated output; this keeps each stage
            -- at one carry-chain level (comparator here, mux in Stage 1b).
            s_above_r <= '1' when s_rgb_r > s_threshold else '0';
            s_above_g <= '1' when s_rgb_g > s_threshold else '0';
            s_above_b <= '1' when s_rgb_b > s_threshold else '0';
            s_rotated_pre_valid <= s_rgb_valid;
        end if;
    end process p_rotation_stage;

    --------------------------------------------------------------------------------
    -- Stage 1b: Dark Suppress Gate
    -- Latency: 1 clock. Input T+3, output T+4.
    -- Critical path: registered_pre -> 2:1 mux on registered_above_flag -> register.
    -- Values above threshold pass the full 10-bit rotated value (no bit reduction).
    -- Values at or below threshold output zero, preserving dark regions as black.
    --------------------------------------------------------------------------------
    p_suppress_stage : process(clk)
    begin
        if rising_edge(clk) then
            -- Gate each channel: pass rotated value if above threshold, else zero.
            -- Full 10-bit rotation is preserved for values above the cutoff.
            s_rotated_r <= s_rotated_pre_r when s_above_r = '1' else (others => '0');
            s_rotated_g <= s_rotated_pre_g when s_above_g = '1' else (others => '0');
            s_rotated_b <= s_rotated_pre_b when s_above_b = '1' else (others => '0');
            s_rotated_valid <= s_rotated_pre_valid;
        end if;
    end process p_suppress_stage;

    --------------------------------------------------------------------------------
    -- Stage 2: Per-Channel Wet/Dry Blend
    -- Latency: 4 clocks. Input T+4, output T+8.
    --------------------------------------------------------------------------------
    interp_r : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_r_for_blend, b=>s_rotated_r, t=>s_blend_r,
                 result=>s_blended_r, valid=>s_blended_r_valid);

    interp_g : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_g_for_blend, b=>s_rotated_g, t=>s_blend_g,
                 result=>s_blended_g, valid=>s_blended_g_valid);

    interp_b : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_b_for_blend, b=>s_rotated_b, t=>s_blend_b,
                 result=>s_blended_b, valid=>s_blended_b_valid);

    --------------------------------------------------------------------------------
    -- Delay Line: RGB dry inputs for per-channel and global blend stages.
    -- Single shift register tapped at two points:
    --   index 1 (2 clocks): T+2+2=T+4 — aligned with s_rotated for per-channel blend
    --   index 5 (6 clocks): T+2+6=T+8 — aligned with s_blended for global blend
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
            s_r_for_blend  <= v_r_delay(1);
            s_g_for_blend  <= v_g_delay(1);
            s_b_for_blend  <= v_b_delay(1);
            s_r_for_global <= v_r_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_g_for_global <= v_g_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_b_for_global <= v_b_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 3: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+7, output T+11.
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
    -- Latency: 1 clock. Input T+11, output T+12.
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

end architecture rgb_bit_rotator;
