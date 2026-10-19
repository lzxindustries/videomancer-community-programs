-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Gouache: catalogue combo #48 (seed 20260920) -- posterization (bit-depth
-- reduction per channel) + hue rotation / saturation crush-boost.  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Luma     luma levels 1..8 bits
--   K2 Chroma   chroma levels 1..8 bits
--   K3 Dither   dither amount before quantizing (0..100% of a step, centred)
--   K4 Hue      base hue rotation 0..360 degrees
--   K5 Sat      saturation 0..400% (100% = unity)
--   K6 Spread   extra hue rotation per band (rainbow bands)
--
--   S7  Order   posterize then rotate / rotate then posterize
--   S8  Sat     gain before the quantizer (crush = fewer bands) / after
--   S9  Bands   hue spread keyed by luma band / by hue sector (8 x 45 deg)
--   S10 Dither  Bayer 4x4 / noise
--   S11 Invert  alternate luma bands are negative
--   P12 Spin    hue rotation speed, bipolar, squared (centre = still)
--
-- Per-pixel chain, no line memory.  Rotation = sine-ROM cos/sin with the
-- ROM operand split hi/lo (eight 11x5/11x6 products).  Latency 19 clocks,
-- all modes, blanking-gated.
--
-- v0.2.0: dither centred on the quantizer (was +0..1 step: Dither also
-- brightened); Invert keys on the 128-code band at 1-2 luma bits (was a
-- flat picture) and mirrors about the video range (64 <-> 940); S9 Chroma
-- = hue-sector bands (was U-only: ~2 bands on real video); Spin squared
-- with a centre deadband and ~8x the old top speed; luma was one clock
-- behind chroma; LFSR dither taps decorrelated; rotation multiply split.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture gouache of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 19;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    function f_clamp511(v : signed) return signed is
    begin
        if v < -511 then
            return to_signed(-511, 11);
        elsif v > 511 then
            return to_signed(511, 11);
        else
            return resize(v, 11);
        end if;
    end function;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    type t_bayer is array (0 to 15) of integer range 0 to 15;
    constant C_BAYER : t_bayer := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);

    signal s_k_lbits, s_k_cbits, s_k_dith, s_k_hue, s_k_sat, s_k_spread : unsigned(9 downto 0);
    signal s_sw_rotfirst, s_sw_satafter, s_sw_cband, s_sw_noise, s_sw_inv : std_logic;
    signal s_p_spin : unsigned(9 downto 0);

    signal s_kl, s_kc : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_kinv   : unsigned(2 downto 0) := to_unsigned(7, 3);   -- invert key bit = min(kl, 7)
    signal s_lmask, s_lnot, s_lhalf : unsigned(9 downto 0) := (others => '0');
    signal s_cmask, s_cnot, s_chalf : unsigned(9 downto 0) := (others => '0');
    signal s_dscl, s_dscc : unsigned(9 downto 0) := (others => '0');   -- dither step scale (step/16 * amount)
    signal s_hue    : unsigned(7 downto 0) := (others => '0');
    signal s_gain   : unsigned(8 downto 0) := (others => '0');
    signal s_gain_a, s_gain_b : unsigned(8 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_gain_a : signal is true;
    attribute keep of s_gain_b : signal is true;
    signal s_spread : unsigned(7 downto 0) := (others => '0');
    signal s_phase  : unsigned(15 downto 0) := (others => '0');
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_stepl, s_stepc : unsigned(9 downto 0) := (others => '0');
    signal s_dl, s_dc : unsigned(17 downto 0) := (others => '0');

    -- spin: squared speed curve, free-running (only the vsync add reads it)
    signal s_spin_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_spin_m   : unsigned(8 downto 0) := (others => '0');
    signal s_spin_sqh : unsigned(12 downto 0) := (others => '0');
    signal s_spin_sql : unsigned(13 downto 0) := (others => '0');
    signal s_spin_mag : unsigned(10 downto 0) := (others => '0');
    signal s_spin_n1, s_spin_n2, s_spin_n3 : std_logic := '0';
    signal s_spin_inc : signed(15 downto 0) := (others => '0');

    signal s_rx : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line : unsigned(10 downto 0) := (others => '0');

    -- pipeline (signal suffix = registers from data_in)
    signal s_u2, s_v2 : signed(10 downto 0) := (others => '0');
    signal s_y2 : unsigned(9 downto 0) := (others => '0');
    signal s_dz2 : signed(3 downto 0) := (others => '0');            -- centred dither -8..7
    signal s_ug3h, s_vg3h : signed(15 downto 0) := (others => '0');
    signal s_ug3l, s_vg3l : signed(16 downto 0) := (others => '0');
    signal s_u3, s_v3 : signed(10 downto 0) := (others => '0');
    signal s_y3 : unsigned(9 downto 0) := (others => '0');
    signal s_dml3, s_dmc3 : signed(14 downto 0) := (others => '0');
    signal s_u4, s_v4 : signed(10 downto 0) := (others => '0');
    signal s_yd4 : signed(11 downto 0) := (others => '0');
    signal s_dc4 : signed(10 downto 0) := (others => '0');
    signal s_u5, s_v5 : signed(11 downto 0) := (others => '0');
    signal s_yq5 : unsigned(9 downto 0) := (others => '0');
    signal s_ik5 : std_logic := '0';
    signal s_su5, s_sv5 : std_logic := '0';
    signal s_au5, s_av5 : unsigned(9 downto 0) := (others => '0');
    signal s_u6, s_v6 : signed(10 downto 0) := (others => '0');
    signal s_yq6 : unsigned(9 downto 0) := (others => '0');
    signal s_ik6 : std_logic := '0';
    signal s_band6 : unsigned(2 downto 0) := (others => '0');
    signal s_yq7 : unsigned(9 downto 0) := (others => '0');
    signal s_ym7 : signed(11 downto 0) := (others => '0');
    signal s_inv7 : std_logic := '0';
    signal s_bs7 : unsigned(10 downto 0) := (others => '0');
    signal s_th8 : unsigned(7 downto 0) := (others => '0');
    signal s_sa9, s_ca9 : unsigned(7 downto 0) := (others => '0');
    signal s_sin10, s_cos10 : signed(9 downto 0) := (others => '0');
    signal s_sinh11, s_cosh11 : signed(4 downto 0) := (others => '0');
    signal s_sinl11, s_cosl11 : unsigned(4 downto 0) := (others => '0');
    signal s_uch12, s_ush12, s_vch12, s_vsh12 : signed(15 downto 0) := (others => '0');
    signal s_ucl12, s_usl12, s_vcl12, s_vsl12 : signed(16 downto 0) := (others => '0');
    signal s_uc13, s_us13, s_vc13, s_vs13 : signed(20 downto 0) := (others => '0');
    signal s_ur14, s_vr14 : signed(21 downto 0) := (others => '0');
    signal s_u15, s_v15 : signed(10 downto 0) := (others => '0');
    signal s_ug16h, s_vg16h : signed(15 downto 0) := (others => '0');
    signal s_ug16l, s_vg16l : signed(16 downto 0) := (others => '0');
    signal s_u16, s_v16 : signed(10 downto 0) := (others => '0');
    signal s_u17, s_v17 : signed(10 downto 0) := (others => '0');
    signal s_u18, s_v18 : signed(11 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    type t_u11 is array (7 to 11) of signed(10 downto 0);
    signal s_ud, s_vd : t_u11 := (others => (others => '0'));
    type t_y10 is array (8 to 18) of unsigned(9 downto 0);
    signal s_yd : t_y10 := (others => (others => '0'));
    type t_dz is array (5 to 17) of signed(10 downto 0);
    signal s_dcd : t_dz := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_lbits  <= unsigned(registers_in(0));
    s_k_cbits  <= unsigned(registers_in(1));
    s_k_dith   <= unsigned(registers_in(2));
    s_k_hue    <= unsigned(registers_in(3));
    s_k_sat    <= unsigned(registers_in(4));
    s_k_spread <= unsigned(registers_in(5));
    s_sw_rotfirst <= registers_in(6)(0);
    s_sw_satafter <= registers_in(6)(1);
    s_sw_cband    <= registers_in(6)(2);
    s_sw_noise    <= registers_in(6)(3);
    s_sw_inv      <= registers_in(6)(4);
    s_p_spin <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_m : unsigned(9 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_sq : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            s_kl <= to_unsigned(9, 4) - resize(s_k_lbits(9 downto 7), 4);   -- bits 1..8 -> drop 9..2
            s_kc <= to_unsigned(9, 4) - resize(s_k_cbits(9 downto 7), 4);
            if s_kl > 7 then s_kinv <= to_unsigned(7, 3); else s_kinv <= s_kl(2 downto 0); end if;
            s_hue <= s_k_hue(9 downto 2);
            s_gain <= s_k_sat(9 downto 1);          -- 0..511, unity 128
            s_gain_a <= s_gain; s_gain_b <= s_gain;
            s_spread <= s_k_spread(9 downto 2);

            -- spin speed = sign * (|P12 - centre| - 8)^2 / 128 (deadband 8, top ~2000 = 7.7 hue steps/frame)
            s_spin_r <= s_p_spin;
            if s_spin_r(9) = '1' then v_m := '0' & s_spin_r(8 downto 0); else v_m := '0' & not s_spin_r(8 downto 0); end if;
            v_d := signed(v_m) - 8;
            if v_d < 0 then s_spin_m <= (others => '0'); else s_spin_m <= unsigned(v_d(8 downto 0)); end if;
            s_spin_n1 <= not s_spin_r(9);
            s_spin_sqh <= s_spin_m * s_spin_m(8 downto 5);
            s_spin_sql <= s_spin_m * s_spin_m(4 downto 0);
            s_spin_n2 <= s_spin_n1;
            v_sq := shift_left(resize(s_spin_sqh, 18), 5) + resize(s_spin_sql, 18);
            s_spin_mag <= v_sq(17 downto 7);
            s_spin_n3 <= s_spin_n2;
            if s_spin_n3 = '1' then s_spin_inc <= -signed(resize(s_spin_mag, 16));
            else s_spin_inc <= signed(resize(s_spin_mag, 16)); end if;

            case to_integer(s_vstep) is
                when 0 =>
                    v_m := shift_left(to_unsigned(1, 10), to_integer(s_kl)) - 1;
                    s_lmask <= v_m; s_lnot <= not v_m;
                    s_lhalf <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 1);
                    s_stepl <= shift_left(to_unsigned(1, 10), to_integer(s_kl));
                    v_m := shift_left(to_unsigned(1, 10), to_integer(s_kc)) - 1;
                    s_cmask <= v_m; s_cnot <= not v_m;
                    s_chalf <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 1);
                    s_stepc <= shift_left(to_unsigned(1, 10), to_integer(s_kc));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_dl <= s_stepl * s_k_dith(9 downto 2);     -- step * amount(0..255)
                    s_dc <= s_stepc * s_k_dith(9 downto 2);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_dscl <= s_dl(17 downto 8);                -- step * amount / 256 (the 16-level dither spans one step at 100%)
                    s_dscc <= s_dc(17 downto 8);
                    s_vstep <= "111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then s_line <= s_line + 1; end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                if s_saw_active = '1' then
                    s_phase <= s_phase + unsigned(s_spin_inc);
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin10 <= to_signed(C_SINE(to_integer(s_sa9)), 10);
            s_cos10 <= to_signed(C_SINE(to_integer(s_ca9)), 10);
            -- fabric register, operand split hi/lo for the rotation products
            s_sinh11 <= s_sin10(9 downto 5); s_sinl11 <= unsigned(s_sin10(4 downto 0));
            s_cosh11 <= s_cos10(9 downto 5); s_cosl11 <= unsigned(s_cos10(4 downto 0));
        end if;
    end process;

    p_pix : process(clk)
        variable v_q : unsigned(9 downto 0);
        variable v_dz : unsigned(3 downto 0);
        variable v_u, v_v : signed(11 downto 0);
        variable v_a : signed(10 downto 0);
        variable v_g : std_logic;
    begin
        if rising_edge(clk) then
            -- 2: centred chroma, dither value (signed -8..7: the 16 levels straddle the quantizer threshold)
            s_u2 <= signed(resize(s_in_u, 11)) - 512;
            s_v2 <= signed(resize(s_in_v, 11)) - 512;
            s_y2 <= s_in_y;
            if s_sw_noise = '1' then v_dz := s_lfsr(15) & s_lfsr(10) & s_lfsr(5) & s_lfsr(0);
            else v_dz := to_unsigned(C_BAYER(to_integer(s_line(1 downto 0) & s_rx(1 downto 0))), 4); end if;
            s_dz2 <= signed((not v_dz(3)) & v_dz(2 downto 0));
            -- 3: pre-gain products, dither magnitudes
            s_ug3h <= s_u2 * signed(resize(s_gain_a(8 downto 5), 5));
            s_ug3l <= s_u2 * signed(resize(s_gain_a(4 downto 0), 6));
            s_vg3h <= s_v2 * signed(resize(s_gain_a(8 downto 5), 5));
            s_vg3l <= s_v2 * signed(resize(s_gain_a(4 downto 0), 6));
            s_u3 <= s_u2; s_v3 <= s_v2; s_y3 <= s_y2;
            s_dml3 <= s_dz2 * signed(resize(s_dscl, 11));
            s_dmc3 <= s_dz2 * signed(resize(s_dscc, 11));
            -- 4: pre-gain select, luma + dither
            if s_sw_satafter = '0' then
                s_u4 <= f_clamp511(shift_right(shift_left(resize(s_ug3h, 22), 5) + resize(s_ug3l, 22), 7));
                s_v4 <= f_clamp511(shift_right(shift_left(resize(s_vg3h, 22), 5) + resize(s_vg3l, 22), 7));
            else
                s_u4 <= s_u3; s_v4 <= s_v3;
            end if;
            s_yd4 <= signed(resize(s_y3, 12)) + resize(shift_right(s_dml3, 4), 12);
            s_dc4 <= resize(shift_right(s_dmc3, 4), 11);
            -- 5: luma quantize (+ invert key bit); chroma + dither; hue-sector inputs
            if s_yd4 < 0 then v_q := (others => '0');
            elsif s_yd4 > 1023 then v_q := (others => '1');
            else v_q := unsigned(s_yd4(9 downto 0)); end if;
            s_yq5 <= (v_q and s_lnot) or s_lhalf;
            s_ik5 <= v_q(to_integer(s_kinv));
            s_u5 <= resize(s_u4, 12) + resize(s_dc4, 12);
            s_v5 <= resize(s_v4, 12) + resize(s_dc4, 12);
            s_dcd(5) <= s_dc4;
            s_su5 <= s_u4(10); s_sv5 <= s_v4(10);
            v_a := abs(s_u4); s_au5 <= unsigned(v_a(9 downto 0));   -- slice: |-512| must stay 512
            v_a := abs(s_v4); s_av5 <= unsigned(v_a(9 downto 0));
            -- 6: chroma quantize (if posterize first), band index
            if s_sw_rotfirst = '0' then
                v_u := signed((unsigned(s_u5) and ("11" & s_cnot)) or resize(s_chalf, 12));
                v_v := signed((unsigned(s_v5) and ("11" & s_cnot)) or resize(s_chalf, 12));
                s_u6 <= f_clamp511(v_u); s_v6 <= f_clamp511(v_v);
            else
                s_u6 <= s_u4; s_v6 <= s_v4;
            end if;
            s_yq6 <= s_yq5; s_ik6 <= s_ik5;
            if s_sw_cband = '1' then
                -- 45-degree hue sector, numbered in angular order; near-grey = sector 0
                if s_au5(9 downto 4) = 0 and s_av5(9 downto 4) = 0 then
                    s_band6 <= "000";
                else
                    if s_au5 > s_av5 then v_g := '1'; else v_g := '0'; end if;
                    case std_logic_vector'(s_sv5 & s_su5 & v_g) is
                        when "000" => s_band6 <= "001";
                        when "001" => s_band6 <= "000";
                        when "010" => s_band6 <= "010";
                        when "011" => s_band6 <= "011";
                        when "110" => s_band6 <= "101";
                        when "111" => s_band6 <= "100";
                        when "100" => s_band6 <= "110";
                        when others => s_band6 <= "111";
                    end case;
                end if;
            else
                s_band6 <= s_yq5(9 downto 7);
            end if;
            -- 7: band * spread; mirror for the negative bands
            s_bs7 <= s_band6 * s_spread;
            s_yq7 <= s_yq6;
            s_ym7 <= to_signed(1004, 12) - signed(resize(s_yq6, 12));
            s_inv7 <= s_sw_inv and s_ik6;
            s_ud(7) <= s_u6; s_vd(7) <= s_v6;
            -- 8: angle; luma invert select (mirror 64 <-> 940, clamped)
            s_th8 <= s_hue + s_phase(15 downto 8) + s_bs7(10 downto 3);
            if s_inv7 = '1' then
                if s_ym7 < 64 then s_yd(8) <= to_unsigned(64, 10);
                elsif s_ym7 > 940 then s_yd(8) <= to_unsigned(940, 10);
                else s_yd(8) <= unsigned(s_ym7(9 downto 0)); end if;
            else
                s_yd(8) <= s_yq7;
            end if;
            -- 9: ROM addresses (ROM out at 10, fabric split at 11)
            s_sa9 <= s_th8; s_ca9 <= s_th8 + 64;
            -- 12: partial products
            s_uch12 <= s_ud(11) * s_cosh11; s_ucl12 <= s_ud(11) * signed(resize(s_cosl11, 6));
            s_ush12 <= s_ud(11) * s_sinh11; s_usl12 <= s_ud(11) * signed(resize(s_sinl11, 6));
            s_vch12 <= s_vd(11) * s_cosh11; s_vcl12 <= s_vd(11) * signed(resize(s_cosl11, 6));
            s_vsh12 <= s_vd(11) * s_sinh11; s_vsl12 <= s_vd(11) * signed(resize(s_sinl11, 6));
            -- 13: recombine
            s_uc13 <= shift_left(resize(s_uch12, 21), 5) + resize(s_ucl12, 21);
            s_us13 <= shift_left(resize(s_ush12, 21), 5) + resize(s_usl12, 21);
            s_vc13 <= shift_left(resize(s_vch12, 21), 5) + resize(s_vcl12, 21);
            s_vs13 <= shift_left(resize(s_vsh12, 21), 5) + resize(s_vsl12, 21);
            -- 14: rotate
            s_ur14 <= resize(s_uc13, 22) - resize(s_vs13, 22);
            s_vr14 <= resize(s_us13, 22) + resize(s_vc13, 22);
            -- 15
            s_u15 <= f_clamp511(shift_right(s_ur14, 8));
            s_v15 <= f_clamp511(shift_right(s_vr14, 8));
            -- 16: post-gain products
            s_ug16h <= s_u15 * signed(resize(s_gain_b(8 downto 5), 5));
            s_ug16l <= s_u15 * signed(resize(s_gain_b(4 downto 0), 6));
            s_vg16h <= s_v15 * signed(resize(s_gain_b(8 downto 5), 5));
            s_vg16l <= s_v15 * signed(resize(s_gain_b(4 downto 0), 6));
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17
            if s_sw_satafter = '1' then
                s_u17 <= f_clamp511(shift_right(shift_left(resize(s_ug16h, 22), 5) + resize(s_ug16l, 22), 7));
                s_v17 <= f_clamp511(shift_right(shift_left(resize(s_vg16h, 22), 5) + resize(s_vg16l, 22), 7));
            else
                s_u17 <= s_u16; s_v17 <= s_v16;
            end if;
            -- 18: chroma quantize (rotate-first path) with dither
            if s_sw_rotfirst = '1' then
                v_u := resize(s_u17, 12) + resize(s_dcd(17), 12);
                v_v := resize(s_v17, 12) + resize(s_dcd(17), 12);
                s_u18 <= signed((unsigned(v_u) and ("11" & s_cnot)) or resize(s_chalf, 12));
                s_v18 <= signed((unsigned(v_v) and ("11" & s_cnot)) or resize(s_chalf, 12));
            else
                s_u18 <= resize(s_u17, 12); s_v18 <= resize(s_v17, 12);
            end if;
            -- 19: gate, recentre
            if s_avid_sr(17) = '1' then
                s_y19 <= s_yd(18);
                s_u19 <= f_clamp10(resize(s_u18, 13) + 512);
                s_v19 <= f_clamp10(resize(s_v18, 13) + 512);
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;

            -- carry chains
            for i in 8 to 11 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            for i in 9 to 18 loop s_yd(i) <= s_yd(i - 1); end loop;
            for i in 6 to 17 loop s_dcd(i) <= s_dcd(i - 1); end loop;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y19);
    s_io_align_in.u       <= std_logic_vector(s_u19);
    s_io_align_in.v       <= std_logic_vector(s_v19);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (19 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture gouache;
