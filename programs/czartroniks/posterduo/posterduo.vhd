-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Posterduo v0.2.0: catalogue combo #112 (seed 20260925) -- posterization +
-- duotone / tritone mapping from luma.  Three knobs each; the switches and
-- the slider decide how they combine.
--
--   K1 Luma     posterize luma 1..8 bits
--   K2 Dither   2x2 ordered dither, 0..100% (+-3/8 of a step at full)
--   K3 Pitch    dither cell 2 / 4 / 8 / 16 px
--   K4 Shadow   shadow ink hue (8 hues)
--   K5 Light    highlight ink hue (8 hues)
--   K6 Contrast tone ramp contrast (0..400%)
--
--   S7  Order   Post-Tone: inks from the posterized luma (one ink mix per
--               band) / Tone-Post: inks from the smooth luma (a smooth ink
--               ramp under the luma bands)
--   S8  Tritone the midtones fade to neutral (third tone)
--   S9  Luma    luma kept (posterized) / flattened to the tone ramp
--   S10 Bands   alternate posterize bands swap the inks
--   S11 Chroma  Smooth / Poster: inks snapped per channel to -/0/+ (nine
--               colours, grey included), dithered by K2; both orders
--   P12 Balance the split between the inks sweeps through the picture:
--               0% high-key (mostly light ink), 50% split at mid-grey,
--               100% low-key (mostly shadow ink).  Glided.
--
-- Luma posterizer (flutter v0.2.0): normalize (y-64)*1.168, add the centred
-- dither, keep L bits, bit-replicate, scale back by 0.855, so black 64 and
-- white ~940 are always levels.  The tone ramp t = (n - m) * contrast + 512
-- runs on the normalized luma n (posterized for Post-Tone, smooth for
-- Tone-Post); inks = shadow + (light - shadow) * t, tritone weight |t-512|.
-- Band swap inverts t for the inks only.
--
-- Hardware contracts: generated avid (parity-locked start, fixed width; the
-- program draws its own colour -- cubist / aurora recipe); output
-- conditioning ([1 2 1] on Y/U/V + luma scaled to white ~777, the damier HD
-- HDMI bar fix); blanking-gated.  Latency 20 clocks, all modes.  No RAM.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture posterduo of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LAT : integer := 20;

    type t_tab is array (0 to 7) of integer range -256 to 255;
    constant C_TU : t_tab := (200, 141, 0, -141, -200, -141, 0, 141);
    constant C_TV : t_tab := (0, 141, 200, 141, 0, -141, -200, -141);

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

    -- top L = z+1 bits of t, replicated down to 10 bits
    function f_rep10(t : unsigned(9 downto 0); z : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(z) is
            when 0 => r := (others => t(9));
            when 1 => r := t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8);
            when 2 => r := t(9 downto 7) & t(9 downto 7) & t(9 downto 7) & t(9);
            when 3 => r := t(9 downto 6) & t(9 downto 6) & t(9 downto 8);
            when 4 => r := t(9 downto 5) & t(9 downto 5);
            when 5 => r := t(9 downto 4) & t(9 downto 6);
            when 6 => r := t(9 downto 3) & t(9 downto 7);
            when others => r := t(9 downto 2) & t(9 downto 8);
        end case;
        return r;
    end function;

    signal s_k_lbits, s_k_dith, s_k_pitch, s_k_shadow, s_k_light, s_k_con : unsigned(9 downto 0);
    signal s_sw_tonepost, s_sw_tri, s_sw_flat, s_sw_bands, s_sw_cpost : std_logic;

    -- control decodes (zones hysteretic: change only 12 counts inside a new zone)
    signal s_zl : unsigned(2 downto 0) := to_unsigned(2, 3);    -- luma bits - 1
    -- frames since load; until it reaches 15 the K1/K4/K5 zones follow the
    -- knob directly (no hysteresis), so a saved default near a zone edge
    -- loads in its own zone instead of staying at the power-up zone
    signal s_zwarm : unsigned(3 downto 0) := (others => '0');
    signal s_pz : unsigned(1 downto 0) := "00";                 -- pitch zone
    signal s_live3 : std_logic := '0';
    signal s_ks : unsigned(2 downto 0) := to_unsigned(3, 3);    -- shadow ink
    signal s_kh : unsigned(2 downto 0) := to_unsigned(0, 3);    -- light ink
    signal s_amt : unsigned(7 downto 0) := (others => '0');
    signal s_dsc : unsigned(9 downto 0) := (others => '0');
    -- dither offsets by dither index (x bit, row bit): Bayer rank 0 2 3 1,
    -- offset (2 rank - 3)/8 of a step -- mean 0
    signal s_ld0, s_ld1, s_ld2, s_ld3 : signed(10 downto 0) := (others => '0');
    signal s_dcp : unsigned(15 downto 0) := (others => '0');
    signal s_dca : unsigned(7 downto 0) := (others => '0');
    signal s_dc0, s_dc1, s_dc2, s_dc3 : signed(10 downto 0) := (others => '0');
    signal s_su, s_sv, s_lu, s_lv : signed(8 downto 0) := (others => '0');
    signal s_du, s_dv : signed(9 downto 0) := (others => '0');
    signal s_con : unsigned(7 downto 0) := to_unsigned(64, 8);
    -- Balance: glided 10.2, ramp midpoint m = 48 + bal * 0.906 (normalized luma)
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gb : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_bal : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bm : unsigned(9 downto 0) := (others => '0');
    signal s_m : signed(11 downto 0) := to_signed(512, 12);

    -- frame / line
    signal s_saw_active, s_vs_go : std_logic := '0';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_avid : std_logic := '0';
    signal s_rx, s_wmax : unsigned(10 downto 0) := (others => '0');
    signal s_line : unsigned(10 downto 0) := (others => '0');
    signal s_fld, s_ilace : std_logic := '0';

    -- generated avid (parity-locked start, fixed-width end)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- pixel path (suffix = stage)
    signal s_in_y : unsigned(9 downto 0) := (others => '0');
    signal s_a2 : unsigned(9 downto 0) := (others => '0');
    type t_dz is array (2 to 14) of unsigned(1 downto 0);
    signal s_dz : t_dz := (others => "00");
    signal s_n13 : unsigned(10 downto 0) := (others => '0');
    signal s_n23 : unsigned(6 downto 0) := (others => '0');
    signal s_yn4, s_yn5, s_yn6 : unsigned(9 downto 0) := (others => '0');
    signal s_t5, s_rep6 : unsigned(9 downto 0) := (others => '0');
    type t_band is array (6 to 10) of std_logic;
    signal s_band : t_band := (others => '0');
    signal s_d17 : unsigned(9 downto 0) := (others => '0');
    signal s_d27 : unsigned(4 downto 0) := (others => '0');
    signal s_yq8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_yq is array (9 to 14) of unsigned(9 downto 0);
    signal s_yqd : t_yq := (others => to_unsigned(64, 10));
    signal s_dt7 : signed(11 downto 0) := (others => '0');
    signal s_ph8, s_pl8 : signed(16 downto 0) := (others => '0');
    signal s_p9 : signed(19 downto 0) := (others => '0');
    signal s_t10 : unsigned(9 downto 0) := (others => '0');
    signal s_tc11 : unsigned(5 downto 0) := (others => '0');
    signal s_w11, s_w12, s_w13 : unsigned(6 downto 0) := (others => '0');
    signal s_tq11, s_rf12 : unsigned(9 downto 0) := (others => '0');
    signal s_pu12, s_pv12 : signed(16 downto 0) := (others => '0');
    signal s_uc13, s_vc13 : signed(10 downto 0) := (others => '0');
    signal s_f113 : unsigned(9 downto 0) := (others => '0');
    signal s_f213 : unsigned(4 downto 0) := (others => '0');
    signal s_pwu14, s_pwv14 : signed(18 downto 0) := (others => '0');
    signal s_yf14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uo15, s_vo15, s_cu15, s_cv15 : signed(10 downto 0) := (others => '0');
    signal s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');   -- s_avid_sr(k) = source avid of the stage-(k+1) pixel
    type t_gv is array (4 to C_LAT - 1) of std_logic;
    signal s_gv : t_gv := (others => '0');                -- s_gv(k) = generated avid, aligned with s_avid_sr(k)

begin

    s_k_lbits  <= unsigned(registers_in(0));
    s_k_dith   <= unsigned(registers_in(1));
    s_k_pitch  <= unsigned(registers_in(2));
    s_k_shadow <= unsigned(registers_in(3));
    s_k_light  <= unsigned(registers_in(4));
    s_k_con    <= unsigned(registers_in(5));
    s_sw_tonepost <= registers_in(6)(0);
    s_sw_tri      <= registers_in(6)(1);
    s_sw_flat     <= registers_in(6)(2);
    s_sw_bands    <= registers_in(6)(3);
    s_sw_cpost    <= registers_in(6)(4);

    p_ctrl : process(clk)
        variable v_z  : unsigned(2 downto 0);
        variable v_lo : unsigned(6 downto 0);
        variable v_z2 : unsigned(1 downto 0);
        variable v_l2 : unsigned(7 downto 0);
        variable v_s8, v_s38 : unsigned(9 downto 0);
        variable v_c8, v_c38 : unsigned(7 downto 0);
        variable v_g  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y);
            s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;

            -- K1 / K4 / K5 zones
            v_z := s_k_lbits(9 downto 7); v_lo := s_k_lbits(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_zl <= v_z; end if;
            v_z := s_k_shadow(9 downto 7); v_lo := s_k_shadow(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_ks <= v_z; end if;
            v_z := s_k_light(9 downto 7); v_lo := s_k_light(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_kh <= v_z; end if;
            -- K3 Pitch (labelled): the firmware loads the default as the label
            -- INDEX; honour it only until the knob first reads a real position
            if s_k_pitch >= 4 then s_live3 <= '1'; end if;
            v_z2 := s_k_pitch(9 downto 8); v_l2 := s_k_pitch(7 downto 0);
            if s_live3 = '0' and s_k_pitch < 4 then
                s_pz <= s_k_pitch(1 downto 0);
            elsif (v_l2 >= 12 or v_z2 = 0) and (v_l2 <= 243 or v_z2 = 3) then
                s_pz <= v_z2;
            end if;

            -- luma dither: step = 512 >> zl (normalized units), K2 0..255/256 of it
            s_amt <= s_k_dith(9 downto 2);
            s_dsc <= shift_right(shift_left(resize(s_amt, 10), 1), to_integer(s_zl));
            v_s8 := shift_right(s_dsc, 3); v_s38 := shift_right(s_dsc, 2) + shift_right(s_dsc, 3);
            s_ld0 <= -signed(resize(v_s38, 11)); s_ld1 <= signed(resize(v_s8, 11));
            s_ld2 <= signed(resize(v_s38, 11));  s_ld3 <= -signed(resize(v_s8, 11));
            -- chroma dither: the snap step is the ink amplitude, 200
            s_dcp <= shift_left(resize(s_amt, 16), 7) + shift_left(resize(s_amt, 16), 6) + shift_left(resize(s_amt, 16), 3);
            s_dca <= s_dcp(15 downto 8);
            v_c8 := shift_right(s_dca, 3); v_c38 := shift_right(s_dca, 2) + shift_right(s_dca, 3);
            s_dc0 <= -signed(resize(v_c38, 11)); s_dc1 <= signed(resize(v_c8, 11));
            s_dc2 <= signed(resize(v_c38, 11));  s_dc3 <= -signed(resize(v_c8, 11));

            -- inks
            s_su <= to_signed(C_TU(to_integer(s_ks)), 9); s_sv <= to_signed(C_TV(to_integer(s_ks)), 9);
            s_lu <= to_signed(C_TU(to_integer(s_kh)), 9); s_lv <= to_signed(C_TV(to_integer(s_kh)), 9);
            s_du <= resize(s_lu, 10) - resize(s_su, 10);
            s_dv <= resize(s_lv, 10) - resize(s_sv, 10);
            s_con <= s_k_con(9 downto 2);

            -- Balance glide (1/4 of the gap a field) and ramp midpoint
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_gb, 13));
            if s_vs_go = '1' then
                v_g := signed(resize(s_gb, 13)) + shift_right(s_gdiff, 2);
                s_gb <= unsigned(v_g(11 downto 0));
            end if;
            s_bal <= s_gb(11 downto 2);
            s_bm <= shift_right(s_bal, 4) + shift_right(s_bal, 5);
            s_m <= to_signed(48, 12) + signed(resize(s_bal, 12)) - signed(resize(s_bm, 12));

            -- line / field events
            s_vs_go <= '0';
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line <= s_line + 1;
                if s_rx > s_wmax then s_wmax <= s_rx; end if;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                if s_saw_active = '1' then
                    if s_zwarm /= 15 then s_zwarm <= s_zwarm + 1; end if;
                    s_vs_go <= '1';
                    g_wm1 <= resize(s_wmax, 12) - 1;       -- generated-avid width
                    s_wmax <= (others => '0');
                    s_fld <= data_in.field_n;
                    s_ilace <= data_in.field_n xor s_fld;  -- field_n toggles: interlaced
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

    -- Clean avid: g_ph is the hsync->avid-rise distance parity of this line,
    -- g_par the voted usual parity; a line whose avid lands off-parity starts
    -- one clock later, so the generated start always sits at the same hsync
    -- parity.  It ends after the field-latched width, or at hsync at the
    -- latest.  g_avid lines up with s_avid_sr(3) (one later when g_sel = '1').
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            s_gv(4) <= g_avid;
            for i in 5 to C_LAT - 1 loop s_gv(i) <= s_gv(i - 1); end loop;
        end if;
    end process p_gavid;

    p_pix : process(clk)
        variable v_rb : std_logic;
        variable v_n  : unsigned(10 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_dy : signed(10 downto 0);
        variable v_dc : signed(10 downto 0);
        variable v_r  : signed(13 downto 0);
        variable v_w  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: luma above black; dither index (x bit, row bit) at the pitch.
            --    Interlaced: the field line's bit p is the frame line's bit p+1.
            if s_in_y > 64 then s_a2 <= s_in_y - 64; else s_a2 <= (others => '0'); end if;
            case to_integer(s_pz) is
                when 0 =>
                    if s_ilace = '1' then v_rb := s_line(0); else v_rb := s_line(1); end if;
                    s_dz(2) <= s_rx(1) & v_rb;
                when 1 =>
                    if s_ilace = '1' then v_rb := s_line(1); else v_rb := s_line(2); end if;
                    s_dz(2) <= s_rx(2) & v_rb;
                when 2 =>
                    if s_ilace = '1' then v_rb := s_line(2); else v_rb := s_line(3); end if;
                    s_dz(2) <= s_rx(3) & v_rb;
                when others =>
                    if s_ilace = '1' then v_rb := s_line(3); else v_rb := s_line(4); end if;
                    s_dz(2) <= s_rx(4) & v_rb;
            end case;
            for i in 3 to 14 loop s_dz(i) <= s_dz(i - 1); end loop;
            -- 3: luma * 1.168 (= 1023/876) in two halves
            s_n13 <= resize(s_a2, 11) + resize(shift_right(s_a2, 3), 11);
            s_n23 <= resize(shift_right(s_a2, 5), 7) + resize(shift_right(s_a2, 7), 7) + resize(shift_right(s_a2, 8), 7);
            -- 4: normalized luma 0..1023
            v_n := s_n13 + resize(s_n23, 11);
            if v_n(10) = '1' then s_yn4 <= (others => '1'); else s_yn4 <= v_n(9 downto 0); end if;
            -- 5: centred dither
            case to_integer(s_dz(4)) is
                when 0 => v_dy := s_ld0;
                when 1 => v_dy := s_ld1;
                when 2 => v_dy := s_ld2;
                when others => v_dy := s_ld3;
            end case;
            v_t := signed(resize(s_yn4, 12)) + resize(v_dy, 12);
            s_t5 <= f_clamp10(v_t);
            s_yn5 <= s_yn4;
            -- 6: keep L bits, replicate (level k of N -> k * 1023 / (N-1)); band = lowest kept bit
            s_rep6 <= f_rep10(s_t5, s_zl);
            s_band(6) <= s_t5(9 - to_integer(s_zl));
            for i in 7 to 10 loop s_band(i) <= s_band(i - 1); end loop;
            s_yn6 <= s_yn5;
            -- 7: luma back toward the video range; ramp input minus the midpoint
            s_d17 <= s_rep6 - shift_right(s_rep6, 3);
            s_d27 <= resize(shift_right(s_rep6, 6), 5) + resize(shift_right(s_rep6, 8), 5);
            if s_sw_tonepost = '1' then
                s_dt7 <= signed(resize(s_yn6, 12)) - s_m;
            else
                s_dt7 <= signed(resize(s_rep6, 12)) - s_m;
            end if;
            -- 8: posterized luma, 64 + rep * 0.855; contrast product split on the nibbles
            s_yq8 <= 64 + s_d17 - resize(s_d27, 10);
            s_ph8 <= s_dt7 * signed(resize(s_con(7 downto 4), 5));
            s_pl8 <= s_dt7 * signed(resize(s_con(3 downto 0), 5));
            s_yqd(9) <= s_yq8;
            for i in 10 to 14 loop s_yqd(i) <= s_yqd(i - 1); end loop;
            -- 9
            s_p9 <= shift_left(resize(s_ph8, 20), 4) + resize(s_pl8, 20);
            -- 10: ramp t = p / 64 + 512, clamped (0 = shadow ink, 1023 = light ink)
            v_r := resize(shift_right(s_p9, 6), 14) + 512;
            s_t10 <= f_clamp10(v_r);
            -- 11: inks read t (inverted on swapped bands, 6 bits); tritone
            --     weight |t - 512| / 8; Tone-Post Flat dithers the ramp for
            --     its own posterize
            if s_sw_bands = '1' and s_band(10) = '1' then
                s_tc11 <= not s_t10(9 downto 4);
            else
                s_tc11 <= s_t10(9 downto 4);
            end if;
            if s_t10 >= 512 then v_w := s_t10 - 512; else v_w := 512 - s_t10; end if;
            if s_sw_tri = '1' then s_w11 <= resize(shift_right(v_w, 3), 7); else s_w11 <= to_unsigned(64, 7); end if;
            if s_sw_tonepost = '1' then
                case to_integer(s_dz(10)) is
                    when 0 => v_dy := s_ld0;
                    when 1 => v_dy := s_ld1;
                    when 2 => v_dy := s_ld2;
                    when others => v_dy := s_ld3;
                end case;
                v_t := signed(resize(s_t10, 12)) + resize(v_dy, 12);
                s_tq11 <= f_clamp10(v_t);
            else
                s_tq11 <= s_t10;
            end if;
            -- 12: ink products (difference x t/64)
            s_pu12 <= s_du * signed(resize(s_tc11, 7));
            s_pv12 <= s_dv * signed(resize(s_tc11, 7));
            s_w12 <= s_w11;
            if s_sw_tonepost = '1' then s_rf12 <= f_rep10(s_tq11, s_zl); else s_rf12 <= s_tq11; end if;
            -- 13: ink chroma; flat luma toward the video range
            s_uc13 <= resize(s_su, 11) + resize(shift_right(s_pu12, 6), 11);
            s_vc13 <= resize(s_sv, 11) + resize(shift_right(s_pv12, 6), 11);
            s_w13 <= s_w12;
            s_f113 <= s_rf12 - shift_right(s_rf12, 3);
            s_f213 <= resize(shift_right(s_rf12, 6), 5) + resize(shift_right(s_rf12, 8), 5);
            -- 14: tritone weight (64 = unity)
            s_pwu14 <= s_uc13 * signed(resize(s_w13, 8));
            s_pwv14 <= s_vc13 * signed(resize(s_w13, 8));
            s_yf14 <= 64 + s_f113 - resize(s_f213, 10);
            -- 15: chroma offsets, plus the snap's dither; luma Keep / Flat
            case to_integer(s_dz(14)) is
                when 0 => v_dc := s_dc0;
                when 1 => v_dc := s_dc1;
                when 2 => v_dc := s_dc2;
                when others => v_dc := s_dc3;
            end case;
            s_uo15 <= resize(shift_right(s_pwu14, 6), 11);
            s_vo15 <= resize(shift_right(s_pwv14, 6), 11);
            s_cu15 <= resize(shift_right(s_pwu14, 6), 11) + v_dc;
            s_cv15 <= resize(shift_right(s_pwv14, 6), 11) + v_dc;
            if s_sw_flat = '1' then s_y15 <= s_yf14; else s_y15 <= s_yqd(14); end if;
            -- 16: Poster snaps each channel to -200 / 0 / +200 about 512
            if s_sw_cpost = '1' then
                if s_cu15 > 100 then s_u16 <= to_unsigned(712, 10);
                elsif s_cu15 < -100 then s_u16 <= to_unsigned(312, 10);
                else s_u16 <= to_unsigned(512, 10); end if;
                if s_cv15 > 100 then s_v16 <= to_unsigned(712, 10);
                elsif s_cv15 < -100 then s_v16 <= to_unsigned(312, 10);
                else s_v16 <= to_unsigned(512, 10); end if;
            else
                v_c := s_uo15 + 512; s_u16 <= unsigned(v_c(9 downto 0));
                v_c := s_vo15 + 512; s_v16 <= unsigned(v_c(9 downto 0));
            end if;
            s_y16 <= s_y15;
            -- 17: gate (source and generated avid)
            if s_avid_sr(15) = '1' and s_gv(15) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- 18..19: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y17; s_g2y <= s_g1y;
            s_g1u <= s_u17; s_g2u <= s_g1u;
            s_g1v <= s_v17; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y17, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u17, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v17, 12);
            s_y19 <= v_sy(11 downto 2); s_u19 <= v_su(11 downto 2); s_v19 <= v_sv(11 downto 2);
            -- 20: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            v_d := s_y19 - 64;
            if s_avid_sr(18) = '1' and s_gv(18) = '1' then
                s_y20 <= s_y19 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gv(C_LAT - 1);

end architecture posterduo;
