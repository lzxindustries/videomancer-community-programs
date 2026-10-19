-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Negspin: catalogue combo #76 (seed 20260922) -- hue rotation /
-- saturation crush-boost + invert / negative / bit-plane isolation.  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Hue      base hue rotation 0..360 degrees (0 = unrotated)
--   K2 Sat      saturation 0..400% (100% = unity)
--   K3 Spread   extra hue rotation per luma band (8 bands)
--   K4 Plane    Negative / Bit 8 .. Bit 2: the plane picture
--   K5 Mix      plane picture mixed over the luma (0..100%)
--   K6 Chroma   chroma bit-planes: Off / 3 .. 129 levels per channel
--
--   S7  Luma    luma as is / negative (64 <-> 940)
--   S8  Chroma  chroma as is / negative (hue + 180)
--   S9  Flip    the plane bit flips the hue by 180 degrees (at Negative the
--               source's top bit: the bright half / strongly saturated)
--   S10 Source  plane bits and the Negative picture from luma / saturation
--   S11 Order   chroma bit-planes before the rotation / after
--   P12 Spin    hue rotation speed, bipolar, squared (centre = still)
--
-- Per-pixel chain, no line memory.  Rotation = sine-ROM cos/sin with the
-- ROM operand split hi/lo (eight 11x5/11x6 products).  Output: highlight
-- knee (600 + (y - 600)/2, white ~770) and [1 2 1] on Y/U/V (the HD HDMI bar
-- fix).  Latency 22 clocks, all modes, blanking-gated.
--
-- v0.2.0: Hue 0 was a -90 degree rotation (the ROM was read a quarter turn
-- early); luma negative was 1023 - y (blacks went superwhite); the chroma
-- bit-planes floored signed chroma (grey noise flickered into full colour)
-- and now round to the nearest level; Sat-source planes x2; Mix reaches
-- 100%; K4/K6 labelled (index guard); S9/S10 live at Negative; Spin squared
-- with a centre deadband.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture negspin of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (22 + 2 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 22;

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

    -- [1 2 1] / 4 of three 10-bit taps
    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
    end function;

    -- C_SINE(a) = -cos(2 pi a / 256): sin = C_SINE(a + 64), cos = C_SINE(a + 128)
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

    signal s_k_hue, s_k_sat, s_k_spread, s_k_plane, s_k_mix, s_k_cpl : unsigned(9 downto 0);
    signal s_sw_negy, s_sw_negc, s_sw_flip, s_sw_satsrc, s_sw_after : std_logic;
    signal s_p_spin : unsigned(9 downto 0);

    signal s_hue, s_hp : unsigned(7 downto 0) := (others => '0');
    signal s_gain, s_gain_a : unsigned(8 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_gain_a : signal is true;
    signal s_spread : unsigned(7 downto 0) := (others => '0');
    signal s_plane : unsigned(2 downto 0) := (others => '0');      -- 0 = Negative, 1..7 = bit 8..2
    signal s_mix : unsigned(8 downto 0) := (others => '0');         -- 0..256
    signal s_cmask : unsigned(9 downto 0) := (others => '1');
    signal s_chalf : unsigned(9 downto 0) := (others => '0');
    signal s_phase : unsigned(15 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    -- spin: squared speed curve, free-running (only the vsync add reads it)
    signal s_spin_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_spin_m   : unsigned(8 downto 0) := (others => '0');
    signal s_spin_sqh : unsigned(12 downto 0) := (others => '0');
    signal s_spin_sql : unsigned(13 downto 0) := (others => '0');
    signal s_spin_mag : unsigned(10 downto 0) := (others => '0');
    signal s_spin_n1, s_spin_n2, s_spin_n3 : std_logic := '0';
    signal s_spin_inc : signed(15 downto 0) := (others => '0');

    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_vsync_n : std_logic := '1';

    -- 2
    signal s_u2, s_v2 : signed(10 downto 0) := (others => '0');
    signal s_yc2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_au2, s_av2 : unsigned(9 downto 0) := (others => '0');
    -- 3
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_sat3 : unsigned(9 downto 0) := (others => '0');
    signal s_u3, s_v3 : signed(10 downto 0) := (others => '0');
    -- 4
    signal s_src4, s_yn4, s_sn4 : unsigned(9 downto 0) := (others => '0');
    signal s_band4 : unsigned(2 downto 0) := (others => '0');
    signal s_u4, s_v4 : signed(10 downto 0) := (others => '0');
    -- 5
    signal s_bs5 : unsigned(10 downto 0) := (others => '0');
    signal s_bit5 : std_logic := '0';
    -- 6..9: angle, ROM addresses, ROM out, fabric split
    signal s_th6 : unsigned(7 downto 0) := (others => '0');
    signal s_sa7, s_ca7 : unsigned(7 downto 0) := (others => '0');
    signal s_sin8, s_cos8 : signed(9 downto 0) := (others => '0');
    signal s_sinh9, s_cosh9 : signed(4 downto 0) := (others => '0');
    signal s_sinl9, s_cosl9 : unsigned(4 downto 0) := (others => '0');
    type t_c11 is array (5 to 9) of signed(10 downto 0);
    signal s_ud, s_vd : t_c11 := (others => (others => '0'));
    type t_pl10 is array (5 to 15) of unsigned(9 downto 0);
    signal s_pld : t_pl10 := (others => to_unsigned(64, 10));
    type t_y10 is array (4 to 17) of unsigned(9 downto 0);
    signal s_yd : t_y10 := (others => to_unsigned(64, 10));
    -- 10..13: rotation
    signal s_uch10, s_ush10, s_vch10, s_vsh10 : signed(15 downto 0) := (others => '0');
    signal s_ucl10, s_usl10, s_vcl10, s_vsl10 : signed(16 downto 0) := (others => '0');
    signal s_uc11, s_us11, s_vc11, s_vs11 : signed(20 downto 0) := (others => '0');
    signal s_ur12, s_vr12 : signed(21 downto 0) := (others => '0');
    signal s_u13, s_v13 : signed(10 downto 0) := (others => '0');
    -- 14..18: gain, chroma planes after, recentre; luma mix
    signal s_ugh14, s_vgh14 : signed(15 downto 0) := (others => '0');
    signal s_ugl14, s_vgl14 : signed(16 downto 0) := (others => '0');
    signal s_u15, s_v15 : signed(10 downto 0) := (others => '0');
    signal s_u16, s_v16 : signed(10 downto 0) := (others => '0');
    signal s_dy16 : signed(10 downto 0) := (others => '0');
    signal s_pm17 : signed(20 downto 0) := (others => '0');
    signal s_u17, s_v17, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- 19..22: knee, neutral gate, [1 2 1], output gate
    signal s_yk19 : signed(10 downto 0) := (others => '0');
    signal s_y19, s_y20, s_ya, s_yb, s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ua, s_ub, s_va, s_vb, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_hue    <= unsigned(registers_in(0));
    s_k_sat    <= unsigned(registers_in(1));
    s_k_spread <= unsigned(registers_in(2));
    s_k_plane  <= unsigned(registers_in(3));
    s_k_mix    <= unsigned(registers_in(4));
    s_k_cpl    <= unsigned(registers_in(5));
    s_sw_negy   <= registers_in(6)(0);
    s_sw_negc   <= registers_in(6)(1);
    s_sw_flip   <= registers_in(6)(2);
    s_sw_satsrc <= registers_in(6)(3);
    s_sw_after  <= registers_in(6)(4);
    s_p_spin <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_c : unsigned(2 downto 0);
        variable v_mx : unsigned(10 downto 0);
        variable v_m : unsigned(9 downto 0);
        variable v_d : signed(10 downto 0);
        variable v_sq : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_vsync_n <= data_in.vsync_n;
            if data_in.avid = '1' then s_saw_active <= '1'; end if;

            s_hue <= s_k_hue(9 downto 2);
            s_hp <= s_hue + s_phase(15 downto 8);
            s_gain <= s_k_sat(9 downto 1); s_gain_a <= s_gain;
            s_spread <= s_k_spread(9 downto 2);
            -- K5 0..1023 -> 0..256, so 100% is the plane alone
            v_mx := resize(s_k_mix, 11) + resize(s_k_mix(9 downto 8), 11);
            s_mix <= v_mx(10 downto 2);
            -- labelled K4/K6: at program load the firmware writes the label
            -- INDEX (0..7), not a knob position, so read 0..7 as the index
            if s_k_plane < 8 then s_plane <= s_k_plane(2 downto 0); else s_plane <= s_k_plane(9 downto 7); end if;
            if s_k_cpl < 8 then v_c := s_k_cpl(2 downto 0); else v_c := s_k_cpl(9 downto 7); end if;
            -- chroma planes kept (sign + mask) and the half step that rounds
            -- to the nearest level: 2^k + 1 levels per channel, grey stays grey
            case to_integer(v_c) is
                when 0 => s_cmask <= "1111111111"; s_chalf <= "0000000000";
                when 1 => s_cmask <= "1000000000"; s_chalf <= "0100000000";
                when 2 => s_cmask <= "1100000000"; s_chalf <= "0010000000";
                when 3 => s_cmask <= "1110000000"; s_chalf <= "0001000000";
                when 4 => s_cmask <= "1111000000"; s_chalf <= "0000100000";
                when 5 => s_cmask <= "1111100000"; s_chalf <= "0000010000";
                when 6 => s_cmask <= "1111110000"; s_chalf <= "0000001000";
                when others => s_cmask <= "1111111000"; s_chalf <= "0000000100";
            end case;

            -- spin speed = sign * (|P12 - centre| - 8)^2 / 128: deadband 8,
            -- top 1976 = 7.7 hue steps a field (v0.1 topped out at 1.0)
            s_spin_r <= s_p_spin;
            if s_spin_r(9) = '1' then v_m := '0' & s_spin_r(8 downto 0); else v_m := '0' & not s_spin_r(8 downto 0); end if;
            v_d := signed('0' & v_m) - 8;
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

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_saw_active = '1' then
                    s_phase <= s_phase + unsigned(s_spin_inc);
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
            s_sin8 <= to_signed(C_SINE(to_integer(s_sa7)), 10);
            s_cos8 <= to_signed(C_SINE(to_integer(s_ca7)), 10);
            -- fabric register, operand split hi/lo for the rotation products
            s_sinh9 <= s_sin8(9 downto 5); s_sinl9 <= unsigned(s_sin8(4 downto 0));
            s_cosh9 <= s_cos8(9 downto 5); s_cosl9 <= unsigned(s_cos8(4 downto 0));
        end if;
    end process;

    p_pix : process(clk)
        variable v_bit : std_logic;
        variable v_s : unsigned(10 downto 0);
        variable v_th : unsigned(7 downto 0);
        variable v_y : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: centre chroma, legal luma, |U| |V|
            s_u2 <= signed(resize(s_in_u, 11)) - 512;
            s_v2 <= signed(resize(s_in_v, 11)) - 512;
            if s_in_y < 64 then
                s_yc2 <= to_unsigned(64, 10);
            elsif s_in_y > 940 then
                s_yc2 <= to_unsigned(940, 10);
            else
                s_yc2 <= s_in_y;
            end if;
            if s_in_u >= 512 then s_au2 <= s_in_u - 512; else s_au2 <= 512 - s_in_u; end if;
            if s_in_v >= 512 then s_av2 <= s_in_v - 512; else s_av2 <= 512 - s_in_v; end if;
            -- 3: negatives (luma mirrored about the video range), saturation x2
            if s_sw_negy = '1' then s_y3 <= to_unsigned(1004, 10) - s_yc2; else s_y3 <= s_yc2; end if;
            v_s := resize(s_au2, 11) + resize(s_av2, 11);
            if v_s(10 downto 9) /= "00" then s_sat3 <= (others => '1'); else s_sat3 <= v_s(8 downto 0) & '0'; end if;
            if s_sw_negc = '1' then s_u3 <= -s_u2; s_v3 <= -s_v2; else s_u3 <= s_u2; s_v3 <= s_v2; end if;
            -- 4: plane source, the two Negative pictures, luma band;
            --    chroma planes before the rotation (round, then mask)
            if s_sw_satsrc = '1' then s_src4 <= s_sat3; else s_src4 <= s_y3; end if;
            s_yn4 <= to_unsigned(1004, 10) - s_y3;
            if s_sat3 > 876 then s_sn4 <= to_unsigned(64, 10); else s_sn4 <= to_unsigned(940, 10) - s_sat3; end if;
            s_band4 <= s_y3(9 downto 7);
            s_yd(4) <= s_y3;
            if s_sw_after = '0' then
                s_u4 <= signed(unsigned(s_u3 + signed('0' & s_chalf)) and ('1' & s_cmask));
                s_v4 <= signed(unsigned(s_v3 + signed('0' & s_chalf)) and ('1' & s_cmask));
            else
                s_u4 <= s_u3; s_v4 <= s_v3;
            end if;
            -- 5: plane bit (bit 9 at Negative: drives Flip only) and plane picture
            v_bit := s_src4(9 - to_integer(s_plane));
            s_bit5 <= v_bit;
            if s_plane = 0 then
                if s_sw_satsrc = '1' then s_pld(5) <= s_sn4; else s_pld(5) <= s_yn4; end if;
            elsif v_bit = '1' then
                s_pld(5) <= to_unsigned(940, 10);
            else
                s_pld(5) <= to_unsigned(64, 10);
            end if;
            s_bs5 <= s_band4 * s_spread;
            s_ud(5) <= s_u4; s_vd(5) <= s_v4;
            -- 6: angle (+128 = flip)
            v_th := s_hp + s_bs5(10 downto 3);
            if s_sw_flip = '1' and s_bit5 = '1' then v_th(7) := not v_th(7); end if;
            s_th6 <= v_th;
            -- 7: ROM addresses (sin = a + 64, cos = a + 128)
            s_sa7 <= s_th6 + 64; s_ca7 <= s_th6 xor "10000000";
            for i in 6 to 9 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            for i in 6 to 15 loop s_pld(i) <= s_pld(i - 1); end loop;
            for i in 5 to 17 loop s_yd(i) <= s_yd(i - 1); end loop;
            -- 10: partial products
            s_uch10 <= s_ud(9) * s_cosh9; s_ucl10 <= s_ud(9) * signed(resize(s_cosl9, 6));
            s_ush10 <= s_ud(9) * s_sinh9; s_usl10 <= s_ud(9) * signed(resize(s_sinl9, 6));
            s_vch10 <= s_vd(9) * s_cosh9; s_vcl10 <= s_vd(9) * signed(resize(s_cosl9, 6));
            s_vsh10 <= s_vd(9) * s_sinh9; s_vsl10 <= s_vd(9) * signed(resize(s_sinl9, 6));
            -- 11: recombine
            s_uc11 <= shift_left(resize(s_uch10, 21), 5) + resize(s_ucl10, 21);
            s_us11 <= shift_left(resize(s_ush10, 21), 5) + resize(s_usl10, 21);
            s_vc11 <= shift_left(resize(s_vch10, 21), 5) + resize(s_vcl10, 21);
            s_vs11 <= shift_left(resize(s_vsh10, 21), 5) + resize(s_vsl10, 21);
            -- 12: rotate
            s_ur12 <= resize(s_uc11, 22) - resize(s_vs11, 22);
            s_vr12 <= resize(s_us11, 22) + resize(s_vc11, 22);
            -- 13
            s_u13 <= f_clamp511(shift_right(s_ur12, 8));
            s_v13 <= f_clamp511(shift_right(s_vr12, 8));
            -- 14: gain (split)
            s_ugh14 <= s_u13 * signed(resize(s_gain_a(8 downto 5), 5));
            s_ugl14 <= s_u13 * signed(resize(s_gain_a(4 downto 0), 6));
            s_vgh14 <= s_v13 * signed(resize(s_gain_a(8 downto 5), 5));
            s_vgl14 <= s_v13 * signed(resize(s_gain_a(4 downto 0), 6));
            -- 15
            s_u15 <= f_clamp511(shift_right(shift_left(resize(s_ugh14, 22), 5) + resize(s_ugl14, 22), 7));
            s_v15 <= f_clamp511(shift_right(shift_left(resize(s_vgh14, 22), 5) + resize(s_vgl14, 22), 7));
            -- 16: chroma planes after the rotation; luma plane mix difference
            if s_sw_after = '1' then
                s_u16 <= signed(unsigned(s_u15 + signed('0' & s_chalf)) and ('1' & s_cmask));
                s_v16 <= signed(unsigned(s_v15 + signed('0' & s_chalf)) and ('1' & s_cmask));
            else
                s_u16 <= s_u15; s_v16 <= s_v15;
            end if;
            s_dy16 <= signed(resize(s_pld(15), 11)) - signed(resize(s_yd(15), 11));
            -- 17: recentre; mix product
            s_u17 <= f_clamp10(resize(s_u16, 13) + 512);
            s_v17 <= f_clamp10(resize(s_v16, 13) + 512);
            s_pm17 <= s_dy16 * signed(resize(s_mix, 10));
            -- 18: luma = y + (plane - y) * mix (stays between the two: no clamp)
            v_y := signed(resize(s_yd(17), 12)) + resize(shift_right(s_pm17, 8), 12);
            s_y18 <= unsigned(v_y(9 downto 0));
            s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19-20: highlight knee (940 -> 770), neutral outside avid so the
            -- smoothing's edge taps read black
            s_yk19 <= signed(resize(s_y18, 11)) - 600;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            if s_avid_sr(18) = '0' then
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            else
                if s_yk19 > 0 then
                    s_y20 <= to_unsigned(600, 10) + unsigned(s_yk19(9 downto 1));
                else
                    s_y20 <= s_y19;
                end if;
                s_u20 <= s_u19; s_v20 <= s_v19;
            end if;
            -- 21-22: [1 2 1] on Y/U/V (output centre = s_ya), blanking gate
            s_ya <= s_y20; s_yb <= s_ya;
            s_ua <= s_u20; s_ub <= s_ua;
            s_va <= s_v20; s_vb <= s_va;
            if s_avid_sr(20) = '1' then
                s_y22 <= f_121(s_yb, s_ya, s_y20);
                s_u22 <= f_121(s_ub, s_ua, s_u20);
                s_v22 <= f_121(s_vb, s_va, s_v20);
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y22);
    s_io_align_in.u       <= std_logic_vector(s_u22);
    s_io_align_in.v       <= std_logic_vector(s_v22);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (22 -> 24).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 1 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(1);

end architecture negspin;
