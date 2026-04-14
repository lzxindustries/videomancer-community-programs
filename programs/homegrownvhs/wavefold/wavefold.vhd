-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: wavefold.vhd - Wavefolder for Videomancer
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-sdk
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
-- Program Name:        Wavefold
-- Author:              Adam Pflanzer
-- Overview:
--   Per-channel wavefolding effect with selectable waveform shape, continuous
--   fold frequency, phase offset, symmetry control, and optional RGB colorspace
--   conversion. Inspired by audio wavefolding / Buchla-style processing applied
--   to video signals.
--
--   Fold frequency is continuous (1x to ~64x) via inline multiply. Each channel
--   value is multiplied by a frequency factor, and the product bits encode the
--   fold position and direction for triangle/sine/saw/square shape output.
--
--   Parameters are latched on vsync to prevent mid-frame tearing.
--
-- Resources:
--   0 BRAM, ~3000 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + CSC forward):     1 clock  -> T+1
--   Stage 1 (phase + symmetry offset):          1 clock  -> T+2
--   Stage 2 (frequency multiply):               1 clock  -> T+3
--   Stage 3 (fold + shape select):              1 clock  -> T+4
--   Stage 4 (invert + CSC reverse + clamp):     1 clock  -> T+5
--   interpolator_u x3 (wet/dry mix):            4 clocks -> T+9
--   Total: 9 clocks
--
-- Submodules:
--   inline IIR LPF x3: line-reset IIR LPF (19-bit precision, no cross-line bleed)
--   sin_cos_full_lut_10x10 x3: sine lookup, combinational
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Fold Amount (continuous 1x-64x)
--   Pot 2  (registers_in(1)):   Phase (0-360 degrees)
--   Pot 3  (registers_in(2)):   Symmetry (DC offset before fold, center=none)
--   Pot 4  (registers_in(3)):   Chroma Offset (chroma freq offset, center=same)
--   Pot 5  (registers_in(4)):   Smoothing (LPF cutoff on source video, 0=off)
--   Pot 6  (registers_in(5)):   Brightness (post-fold Y offset, center=none)
--   Tog 7  (registers_in(6)(0)): Shape bit 0 (00=Tri,01=Sin,10=Saw,11=Sqr)
--   Tog 8  (registers_in(6)(1)): Shape bit 1
--   Tog 9  (registers_in(6)(2)): Cspace bit 0 (00=YUV,01=RGB,10=XYZ,11=Y-only)
--   Tog 10 (registers_in(6)(3)): Cspace bit 1
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 5 (inline stages)
--   C_SYNC_DELAY_CLKS       = 9 (5 + 4 interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture wavefold of program_top is

    constant C_VIDEO_DATA_WIDTH      : integer := 10;
    constant C_PROCESSING_DELAY_CLKS : integer := 5;
    constant C_SYNC_DELAY_CLKS       : integer := 9;

    -- Frequency encoding: freq_factor = (knob >> 2) + 4 = 4..259 (8 bits)
    -- product = channel * freq_factor (10x8 = 18 bits)
    -- Normalized: product >> 2, at factor=4 gives 1x, at factor=259 gives ~65x
    constant C_FREQ_OFFSET : unsigned(7 downto 0) := to_unsigned(4, 8);

    -- Vsync-latched parameters (prevents mid-frame tearing)
    signal r_freq_knob       : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal r_phase           : unsigned(9 downto 0) := (others => '0');
    signal r_symmetry        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_chroma_offset   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_smoothing       : unsigned(7 downto 0) := (others => '0');
    signal r_brightness      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_shape           : std_logic_vector(1 downto 0) := "00";
    signal r_csc_mode        : std_logic_vector(1 downto 0) := "00";
    signal s_bypass          : std_logic;
    signal s_mix             : unsigned(9 downto 0);

    -- Computed frequency factors
    signal r_luma_freq   : unsigned(7 downto 0) := to_unsigned(36, 8);
    signal r_chroma_freq : unsigned(7 downto 0) := to_unsigned(36, 8);

    -- Vsync edge detection
    signal s_prev_vsync : std_logic := '1';

    -- Stage 0 outputs: input register + optional CSC forward
    signal s0_ch0 : unsigned(9 downto 0) := (others => '0');
    signal s0_ch1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_ch2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 1 outputs: phase + symmetry offset
    signal s1_ch0 : unsigned(9 downto 0) := (others => '0');
    signal s1_ch1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_ch2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 2 outputs: frequency multiply products (18 bits each)
    signal s2_prod0 : unsigned(17 downto 0) := (others => '0');
    signal s2_prod1 : unsigned(17 downto 0) := (others => '0');
    signal s2_prod2 : unsigned(17 downto 0) := (others => '0');

    -- Stage 3 outputs: fold + shape select
    signal s3_ch0 : unsigned(9 downto 0) := (others => '0');
    signal s3_ch1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s3_ch2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 4 outputs: invert + CSC reverse + clamp
    signal s4_y : unsigned(9 downto 0) := (others => '0');
    signal s4_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s4_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Triangle fold from product bits (combinational)
    signal s_tri0 : unsigned(9 downto 0);
    signal s_tri1 : unsigned(9 downto 0);
    signal s_tri2 : unsigned(9 downto 0);

    -- Sine LUT outputs (combinational)
    signal s_sin0_raw : signed(9 downto 0);
    signal s_sin1_raw : signed(9 downto 0);
    signal s_sin2_raw : signed(9 downto 0);

    -- Dry delay / bypass delay / interp valid
    signal s_dry_y        : unsigned(9 downto 0);
    signal s_dry_u        : unsigned(9 downto 0);
    signal s_dry_v        : unsigned(9 downto 0);
    signal s_bypass_y     : std_logic_vector(9 downto 0);
    signal s_bypass_u     : std_logic_vector(9 downto 0);
    signal s_bypass_v     : std_logic_vector(9 downto 0);
    signal s_interp_valid : std_logic := '0';
    signal s_mix_y        : unsigned(9 downto 0);
    signal s_mix_u        : unsigned(9 downto 0);
    signal s_mix_v        : unsigned(9 downto 0);
    signal s_mix_y_valid  : std_logic;

    -- Inline IIR LPF with line-boundary reset and extended precision
    -- 19-bit signed state (sign + 10 data + 8 fractional) per channel.
    -- Resets on first active pixel of each line to prevent cross-line bleed.
    constant C_IIR_FRAC  : integer := 8;
    constant C_IIR_WIDTH : integer := 11 + C_IIR_FRAC;  -- 19 bits signed
    signal s_iir_y          : signed(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_iir_u          : signed(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_iir_v          : signed(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_line_first_pix : std_logic := '1';
    signal s_prev_hsync_lpf : std_logic := '1';
    signal s_lpf_mix_acc    : unsigned(4 downto 0) := (others => '0');
    signal s_src_y          : unsigned(9 downto 0);
    signal s_src_u          : unsigned(9 downto 0);
    signal s_src_v          : unsigned(9 downto 0);

begin

    -- Bypass and mix are NOT latched (need immediate response)
    s_bypass <= registers_in(6)(4);
    s_mix    <= unsigned(registers_in(7)(9 downto 0));

    -- =========================================================================
    -- Input LPF: inline IIR with line-boundary reset and extended precision
    -- 19-bit internal state (sign + 10 data + 8 fractional) per channel.
    -- Resets to input on first active pixel of each line to eliminate
    -- cross-line bleed (horizontal noise). Extra fractional bits reduce
    -- step quantization (chroma banding). Sigma-delta fine mixing from
    -- cutoff lower 4 bits gives 256 smooth cutoff steps.
    -- =========================================================================
    p_lpf : process(clk)
        variable v_in_y  : signed(C_IIR_WIDTH - 1 downto 0);
        variable v_in_u  : signed(C_IIR_WIDTH - 1 downto 0);
        variable v_in_v  : signed(C_IIR_WIDTH - 1 downto 0);
        variable v_err_y : signed(C_IIR_WIDTH - 1 downto 0);
        variable v_err_u : signed(C_IIR_WIDTH - 1 downto 0);
        variable v_err_v : signed(C_IIR_WIDTH - 1 downto 0);
        variable v_k_coarse  : natural;
        variable v_k_use     : natural;
        variable v_acc_next  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            v_k_coarse := to_integer(r_smoothing(7 downto 4));

            v_in_y := shift_left(resize(signed('0' & data_in.y), C_IIR_WIDTH), C_IIR_FRAC);
            v_in_u := shift_left(resize(signed('0' & data_in.u), C_IIR_WIDTH), C_IIR_FRAC);
            v_in_v := shift_left(resize(signed('0' & data_in.v), C_IIR_WIDTH), C_IIR_FRAC);

            if data_in.avid = '1' then
                if s_line_first_pix = '1' then
                    -- Snap to input on first pixel of line (eliminates cross-line bleed)
                    s_iir_y <= v_in_y;
                    s_iir_u <= v_in_u;
                    s_iir_v <= v_in_v;
                    s_line_first_pix <= '0';
                else
                    v_err_y := v_in_y - s_iir_y;
                    v_err_u := v_in_u - s_iir_u;
                    v_err_v := v_in_v - s_iir_v;

                    -- Sigma-delta fine mixing: alternate k and k+1
                    v_acc_next := s_lpf_mix_acc + resize(r_smoothing(3 downto 0), 5);
                    if v_acc_next >= 16 then
                        s_lpf_mix_acc <= v_acc_next - 16;
                        if v_k_coarse < 15 then
                            v_k_use := v_k_coarse + 1;
                        else
                            v_k_use := v_k_coarse;
                        end if;
                    else
                        s_lpf_mix_acc <= v_acc_next;
                        v_k_use := v_k_coarse;
                    end if;

                    s_iir_y <= s_iir_y + shift_right(v_err_y, v_k_use);
                    s_iir_u <= s_iir_u + shift_right(v_err_u, v_k_use);
                    s_iir_v <= s_iir_v + shift_right(v_err_v, v_k_use);
                end if;
            end if;

            -- Detect line boundaries
            s_prev_hsync_lpf <= data_in.hsync_n;
            if s_prev_hsync_lpf = '1' and data_in.hsync_n = '0' then
                s_line_first_pix <= '1';
            end if;
        end if;
    end process p_lpf;

    -- Extract 10-bit unsigned from 19-bit signed IIR state (bits 17:8)
    s_src_y <= to_unsigned(0, 10) when s_iir_y(C_IIR_WIDTH - 1) = '1' else
               unsigned(s_iir_y(9 + C_IIR_FRAC downto C_IIR_FRAC));
    s_src_u <= to_unsigned(0, 10) when s_iir_u(C_IIR_WIDTH - 1) = '1' else
               unsigned(s_iir_u(9 + C_IIR_FRAC downto C_IIR_FRAC));
    s_src_v <= to_unsigned(0, 10) when s_iir_v(C_IIR_WIDTH - 1) = '1' else
               unsigned(s_iir_v(9 + C_IIR_FRAC downto C_IIR_FRAC));

    -- =========================================================================
    -- Parameter latch on vsync (prevents mid-frame tearing)
    -- =========================================================================
    p_param_latch : process(clk)
        variable v_luma_freq_wide : unsigned(8 downto 0);  -- 9 bits to avoid overflow
        variable v_chroma_off_s   : signed(9 downto 0);
        variable v_chroma_freq_s  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                r_freq_knob     <= unsigned(registers_in(0)(9 downto 0));
                r_phase         <= unsigned(registers_in(1)(9 downto 0));
                r_symmetry      <= unsigned(registers_in(2)(9 downto 0));
                r_chroma_offset <= unsigned(registers_in(3)(9 downto 0));
                r_smoothing     <= unsigned(registers_in(4)(9 downto 2));
                r_brightness    <= unsigned(registers_in(5)(9 downto 0));
                r_shape         <= registers_in(6)(1) & registers_in(6)(0);
                r_csc_mode      <= registers_in(6)(3) & registers_in(6)(2);

                -- Luma frequency factor: (knob >> 2) + 4 = 4..259
                -- Use 9-bit to avoid overflow, clamp to 255
                v_luma_freq_wide := resize(unsigned(registers_in(0)(9 downto 2)), 9)
                                  + resize(C_FREQ_OFFSET, 9);
                if v_luma_freq_wide > 255 then
                    r_luma_freq <= to_unsigned(255, 8);
                else
                    r_luma_freq <= v_luma_freq_wide(7 downto 0);
                end if;

                -- Chroma frequency: luma + offset from center
                -- chroma_knob centered at 512: offset = (chroma_knob >> 1) - 256
                v_chroma_off_s := signed('0' & registers_in(3)(9 downto 1)) -
                                  to_signed(256, 10);
                v_chroma_freq_s := signed(resize(v_luma_freq_wide, 10)) +
                                   resize(v_chroma_off_s, 10);
                if v_chroma_freq_s < 4 then
                    r_chroma_freq <= to_unsigned(4, 8);
                elsif v_chroma_freq_s > 255 then
                    r_chroma_freq <= to_unsigned(255, 8);
                else
                    r_chroma_freq <= unsigned(v_chroma_freq_s(7 downto 0));
                end if;
            end if;
        end if;
    end process p_param_latch;

    -- =========================================================================
    -- Stage 0 (T+1): Input register + optional CSC forward
    -- 00=YUV (pass), 01=RGB, 10=XYZ, 11=Y-only
    -- =========================================================================
    p_stage0 : process(clk)
        variable v_u_s : signed(10 downto 0);
        variable v_v_s : signed(10 downto 0);
        variable v_r   : signed(11 downto 0);
        variable v_g   : signed(11 downto 0);
        variable v_b   : signed(11 downto 0);
        variable v_x   : signed(11 downto 0);
        variable v_yx  : signed(11 downto 0);
        variable v_z   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            case r_csc_mode is
                when "01" =>
                    -- RGB mode: Approximate BT.601 YUV -> RGB
                    v_u_s := signed(resize(s_src_u, 11)) - to_signed(512, 11);
                    v_v_s := signed(resize(s_src_v, 11)) - to_signed(512, 11);

                    v_r := signed(resize(s_src_y, 12)) +
                           resize(v_v_s, 12) +
                           resize(shift_right(v_v_s, 2), 12) +
                           resize(shift_right(v_v_s, 3), 12);
                    v_g := signed(resize(s_src_y, 12)) -
                           resize(shift_right(v_u_s, 2), 12) -
                           resize(shift_right(v_u_s, 4), 12) -
                           resize(shift_right(v_v_s, 1), 12) -
                           resize(shift_right(v_v_s, 3), 12);
                    v_b := signed(resize(s_src_y, 12)) +
                           resize(v_u_s, 12) +
                           resize(shift_right(v_u_s, 1), 12) +
                           resize(shift_right(v_u_s, 2), 12);

                    if v_r < 0 then s0_ch0 <= (others => '0');
                    elsif v_r > 1023 then s0_ch0 <= to_unsigned(1023, 10);
                    else s0_ch0 <= unsigned(v_r(9 downto 0)); end if;

                    if v_g < 0 then s0_ch1 <= (others => '0');
                    elsif v_g > 1023 then s0_ch1 <= to_unsigned(1023, 10);
                    else s0_ch1 <= unsigned(v_g(9 downto 0)); end if;

                    if v_b < 0 then s0_ch2 <= (others => '0');
                    elsif v_b > 1023 then s0_ch2 <= to_unsigned(1023, 10);
                    else s0_ch2 <= unsigned(v_b(9 downto 0)); end if;

                when "10" =>
                    -- XYZ mode: Approximate YUV -> CIE XYZ
                    v_u_s := signed(resize(s_src_u, 11)) - to_signed(512, 11);
                    v_v_s := signed(resize(s_src_v, 11)) - to_signed(512, 11);

                    -- X ~ Y + 3/16*U_s + 5/16*V_s
                    v_x := signed(resize(s_src_y, 12)) +
                           resize(shift_right(v_u_s, 3), 12) +
                           resize(shift_right(v_u_s, 4), 12) +
                           resize(shift_right(v_v_s, 2), 12) +
                           resize(shift_right(v_v_s, 4), 12);
                    -- Y_xyz ~ Y - U_s/8 - V_s/4
                    v_yx := signed(resize(s_src_y, 12)) -
                            resize(shift_right(v_u_s, 3), 12) -
                            resize(shift_right(v_v_s, 2), 12);
                    -- Z ~ Y + 13/8*U_s - V_s/16
                    v_z := signed(resize(s_src_y, 12)) +
                           resize(v_u_s, 12) +
                           resize(shift_right(v_u_s, 1), 12) +
                           resize(shift_right(v_u_s, 3), 12) -
                           resize(shift_right(v_v_s, 4), 12);

                    if v_x < 0 then s0_ch0 <= (others => '0');
                    elsif v_x > 1023 then s0_ch0 <= to_unsigned(1023, 10);
                    else s0_ch0 <= unsigned(v_x(9 downto 0)); end if;

                    if v_yx < 0 then s0_ch1 <= (others => '0');
                    elsif v_yx > 1023 then s0_ch1 <= to_unsigned(1023, 10);
                    else s0_ch1 <= unsigned(v_yx(9 downto 0)); end if;

                    if v_z < 0 then s0_ch2 <= (others => '0');
                    elsif v_z > 1023 then s0_ch2 <= to_unsigned(1023, 10);
                    else s0_ch2 <= unsigned(v_z(9 downto 0)); end if;

                when "11" =>
                    -- Y-only mode: fold luma only, neutral chroma
                    s0_ch0 <= s_src_y;
                    s0_ch1 <= to_unsigned(512, 10);
                    s0_ch2 <= to_unsigned(512, 10);

                when others =>
                    -- YUV mode (00): direct passthrough
                    s0_ch0 <= s_src_y;
                    s0_ch1 <= s_src_u;
                    s0_ch2 <= s_src_v;
            end case;
        end if;
    end process p_stage0;

    -- =========================================================================
    -- Stage 1 (T+2): Phase + symmetry offset
    -- =========================================================================
    p_stage1 : process(clk)
        variable v_sym : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_sym := r_symmetry - to_unsigned(512, 10);

            s1_ch0 <= s0_ch0 + r_phase + v_sym;
            s1_ch1 <= s0_ch1 + r_phase + v_sym;
            s1_ch2 <= s0_ch2 + r_phase + v_sym;
        end if;
    end process p_stage1;

    -- =========================================================================
    -- Stage 2 (T+3): Frequency multiply
    -- =========================================================================
    p_stage2 : process(clk)
    begin
        if rising_edge(clk) then
            s2_prod0 <= s1_ch0 * r_luma_freq;
            s2_prod1 <= s1_ch1 * r_chroma_freq;
            s2_prod2 <= s1_ch2 * r_chroma_freq;
        end if;
    end process p_stage2;

    -- =========================================================================
    -- Triangle fold (combinational from stage 2 products)
    -- Direction: product(12), Position: product(11:2)
    -- =========================================================================
    s_tri0 <= to_unsigned(1023, 10) - unsigned(s2_prod0(11 downto 2))
              when s2_prod0(12) = '1'
              else unsigned(s2_prod0(11 downto 2));

    s_tri1 <= to_unsigned(1023, 10) - unsigned(s2_prod1(11 downto 2))
              when s2_prod1(12) = '1'
              else unsigned(s2_prod1(11 downto 2));

    s_tri2 <= to_unsigned(1023, 10) - unsigned(s2_prod2(11 downto 2))
              when s2_prod2(12) = '1'
              else unsigned(s2_prod2(11 downto 2));

    -- =========================================================================
    -- Sine LUTs (combinational): sawtooth phase as angle input
    -- =========================================================================
    u_sin0 : entity work.sin_cos_full_lut_10x10
        port map (angle_in => std_logic_vector(s2_prod0(11 downto 2)),
                  sin_out  => s_sin0_raw, cos_out => open);

    u_sin1 : entity work.sin_cos_full_lut_10x10
        port map (angle_in => std_logic_vector(s2_prod1(11 downto 2)),
                  sin_out  => s_sin1_raw, cos_out => open);

    u_sin2 : entity work.sin_cos_full_lut_10x10
        port map (angle_in => std_logic_vector(s2_prod2(11 downto 2)),
                  sin_out  => s_sin2_raw, cos_out => open);

    -- =========================================================================
    -- Stage 3 (T+4): Shape select
    --   00=Triangle, 01=Sine, 10=Sawtooth, 11=Square
    -- =========================================================================
    p_stage3 : process(clk)
        variable v_sin0 : unsigned(9 downto 0);
        variable v_sin1 : unsigned(9 downto 0);
        variable v_sin2 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_sin0 := unsigned(std_logic_vector(s_sin0_raw + to_signed(512, 10)));
            v_sin1 := unsigned(std_logic_vector(s_sin1_raw + to_signed(512, 10)));
            v_sin2 := unsigned(std_logic_vector(s_sin2_raw + to_signed(512, 10)));

            case r_shape is
                when "00" =>
                    s3_ch0 <= s_tri0;
                    s3_ch1 <= s_tri1;
                    s3_ch2 <= s_tri2;
                when "01" =>
                    s3_ch0 <= v_sin0;
                    s3_ch1 <= v_sin1;
                    s3_ch2 <= v_sin2;
                when "10" =>
                    s3_ch0 <= unsigned(s2_prod0(11 downto 2));
                    s3_ch1 <= unsigned(s2_prod1(11 downto 2));
                    s3_ch2 <= unsigned(s2_prod2(11 downto 2));
                when others =>
                    if s2_prod0(12) = '0' then s3_ch0 <= to_unsigned(1023, 10);
                    else s3_ch0 <= to_unsigned(0, 10); end if;
                    if s2_prod1(12) = '0' then s3_ch1 <= to_unsigned(1023, 10);
                    else s3_ch1 <= to_unsigned(0, 10); end if;
                    if s2_prod2(12) = '0' then s3_ch2 <= to_unsigned(1023, 10);
                    else s3_ch2 <= to_unsigned(0, 10); end if;
            end case;
        end if;
    end process p_stage3;

    -- =========================================================================
    -- Stage 4 (T+5): CSC reverse + brightness + clamp
    -- 00=YUV, 01=RGB->YUV, 10=XYZ->YUV, 11=Y-only (pass original chroma)
    -- =========================================================================
    p_stage4 : process(clk)
        variable v0 : unsigned(9 downto 0);
        variable v1 : unsigned(9 downto 0);
        variable v2 : unsigned(9 downto 0);
        variable v_r_s : signed(11 downto 0);
        variable v_g_s : signed(11 downto 0);
        variable v_b_s : signed(11 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_u_s : signed(11 downto 0);
        variable v_v_s : signed(11 downto 0);
        variable v_brt : signed(11 downto 0);
        variable v_ch0_s : signed(11 downto 0);
        variable v_ch1_s : signed(11 downto 0);
        variable v_ch2_s : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v0 := s3_ch0;
            v1 := s3_ch1;
            v2 := s3_ch2;

            case r_csc_mode is
                when "01" =>
                    -- RGB -> YUV reverse
                    v_r_s := signed(resize(v0, 12));
                    v_g_s := signed(resize(v1, 12));
                    v_b_s := signed(resize(v2, 12));

                    v_y := shift_right(v_r_s, 2) +
                           shift_right(v_g_s, 1) + shift_right(v_g_s, 4) +
                           shift_right(v_b_s, 3);
                    v_u_s := -shift_right(v_r_s, 3) -
                              shift_right(v_g_s, 2) +
                              shift_right(v_b_s, 1);
                    v_v_s := shift_right(v_r_s, 1) -
                             shift_right(v_g_s, 2) - shift_right(v_g_s, 3) -
                             shift_right(v_b_s, 4);

                    v_brt := signed(resize(r_brightness, 12)) - to_signed(512, 12);
                    v_y := v_y + v_brt;

                    if v_y < 0 then s4_y <= (others => '0');
                    elsif v_y > 1023 then s4_y <= to_unsigned(1023, 10);
                    else s4_y <= unsigned(v_y(9 downto 0)); end if;

                    v_u_s := v_u_s + to_signed(512, 12);
                    if v_u_s < 0 then s4_u <= (others => '0');
                    elsif v_u_s > 1023 then s4_u <= to_unsigned(1023, 10);
                    else s4_u <= unsigned(v_u_s(9 downto 0)); end if;

                    v_v_s := v_v_s + to_signed(512, 12);
                    if v_v_s < 0 then s4_v <= (others => '0');
                    elsif v_v_s > 1023 then s4_v <= to_unsigned(1023, 10);
                    else s4_v <= unsigned(v_v_s(9 downto 0)); end if;

                when "10" =>
                    -- XYZ -> YUV reverse (approximate inverse)
                    v_ch0_s := signed(resize(v0, 12));
                    v_ch1_s := signed(resize(v1, 12));
                    v_ch2_s := signed(resize(v2, 12));

                    -- Y ~ 3/8*X + 5/8*Y_xyz
                    v_y := shift_right(v_ch0_s, 2) + shift_right(v_ch0_s, 3) +
                           shift_right(v_ch1_s, 1) + shift_right(v_ch1_s, 3);
                    -- U_s ~ -3/16*X - 1/2*Y_xyz + 5/8*Z
                    v_u_s := -shift_right(v_ch0_s, 3) - shift_right(v_ch0_s, 4) -
                              shift_right(v_ch1_s, 1) +
                              shift_right(v_ch2_s, 1) + shift_right(v_ch2_s, 3);
                    -- V_s ~ 2*X - 3/2*Y_xyz - 3/8*Z
                    v_v_s := shift_left(v_ch0_s, 1) -
                             v_ch1_s - shift_right(v_ch1_s, 1) -
                             shift_right(v_ch2_s, 2) - shift_right(v_ch2_s, 3);

                    v_brt := signed(resize(r_brightness, 12)) - to_signed(512, 12);
                    v_y := v_y + v_brt;

                    if v_y < 0 then s4_y <= (others => '0');
                    elsif v_y > 1023 then s4_y <= to_unsigned(1023, 10);
                    else s4_y <= unsigned(v_y(9 downto 0)); end if;

                    v_u_s := v_u_s + to_signed(512, 12);
                    if v_u_s < 0 then s4_u <= (others => '0');
                    elsif v_u_s > 1023 then s4_u <= to_unsigned(1023, 10);
                    else s4_u <= unsigned(v_u_s(9 downto 0)); end if;

                    v_v_s := v_v_s + to_signed(512, 12);
                    if v_v_s < 0 then s4_v <= (others => '0');
                    elsif v_v_s > 1023 then s4_v <= to_unsigned(1023, 10);
                    else s4_v <= unsigned(v_v_s(9 downto 0)); end if;

                when "11" =>
                    -- Y-only: fold luma, preserve original chroma from dry tap
                    v_brt := signed(resize(v0, 12)) +
                             signed(resize(r_brightness, 12)) - to_signed(512, 12);
                    if v_brt < 0 then s4_y <= (others => '0');
                    elsif v_brt > 1023 then s4_y <= to_unsigned(1023, 10);
                    else s4_y <= unsigned(v_brt(9 downto 0)); end if;
                    s4_u <= s_dry_u;
                    s4_v <= s_dry_v;

                when others =>
                    -- YUV mode (00): brightness on Y, pass U/V
                    v_brt := signed(resize(v0, 12)) +
                             signed(resize(r_brightness, 12)) - to_signed(512, 12);
                    if v_brt < 0 then s4_y <= (others => '0');
                    elsif v_brt > 1023 then s4_y <= to_unsigned(1023, 10);
                    else s4_y <= unsigned(v_brt(9 downto 0)); end if;

                    s4_u <= v1;
                    s4_v <= v2;
            end case;
        end if;
    end process p_stage4;

    -- =========================================================================
    -- Valid pipeline
    -- =========================================================================
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_interp_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    -- =========================================================================
    -- Interpolators: fader-controlled dry/wet mix
    -- =========================================================================
    u_interp_y : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_y, b => s4_y, t => s_mix,
                  result => s_mix_y, valid => s_mix_y_valid);

    u_interp_u : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_u, b => s4_u, t => s_mix,
                  result => s_mix_u, valid => open);

    u_interp_v : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_v, b => s4_v, t => s_mix,
                  result => s_mix_v, valid => open);

    -- =========================================================================
    -- Sync delay + bypass delay + dry delay
    -- =========================================================================
    p_delay : process(clk)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(9 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));
        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(9 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);
            s_bypass_y <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_u <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_v <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);
            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    -- Output mux
    data_out.y <= s_bypass_y when s_bypass = '1' else std_logic_vector(s_mix_y);
    data_out.u <= s_bypass_u when s_bypass = '1' else std_logic_vector(s_mix_u);
    data_out.v <= s_bypass_v when s_bypass = '1' else std_logic_vector(s_mix_v);

end architecture wavefold;
