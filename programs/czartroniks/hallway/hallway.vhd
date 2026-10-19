-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Hallway v0.2.0: catalogue combo #84 (seed 20260923) -- split-screen / quad
-- + lens distortion (barrel, chromatic aberration) + mirror / kaleidoscope.
-- Two knobs each; the switches and the slider decide how they combine.
--
--   K1 Panes    1 / 2 / 3 / 4 panes
--   K2 Wipe     pane boundaries slide across the screen (glided)
--   K3 Barrel   lens strength, bipolar (pincushion .. barrel), glided
--   K4 Fringe   chromatic spread, 3 .. 50% of the lens scale
--   K5 Segment  mirror segment width, 64 .. 1024 px at 1080p (continuous,
--               exponential; the same fraction of the screen HEIGHT in
--               every mode)
--   K6 Slide    slide speed of the mirrored strip, bipolar, squared, detent
--
--   S7  Mirror  odd panes mirrored
--   S8  Lens    every pane looks at the screen centre / at its own centre
--   S9  Fold    mirror (triangle) / repeat (sawtooth), same segment width
--   S10 Edges   outside the line: black / mirrored
--   S11 Fringe  U wide / V wide
--   P12 Depth   segments narrow continuously, up to 16x (8 px floor at
--               1080p, scaled with the segment)
--
-- v0.2.2 (2026-10-17): resolution scaling.  The segment exponent used
-- log2(measured width / 1920), so segments were a fraction of the WIDTH
-- (0.72x the 1080p size on SD's 4:3 screen).  It now takes -log2(x step)
-- from the timing ID (1080 0, 720p -0.585, SD -1 octave): a segment is the
-- same fraction of the screen height in every mode, like a 1440-unit 4:3
-- window of the 1080p picture.  The 8 px floor and the slide speed scale
-- the same way (SD floor 4 px: fold exponent 3 added).  Panes, Wipe and the
-- lens profile were already fractions of the width / field height.  1080p
-- bit-identical.
--
-- v0.2.0.  Each pane folds its own coordinate (v0.1's Screen lens folded
-- the whole line, so Panes did nothing with Mirror off).  The four nested
-- power-of-two folds were one fold at the finest period; now one fold of
-- the pane coordinate scaled by g = 1..2 gives any segment width, and the
-- lens scale is divided by g so each strip still shows its own width of
-- picture.  Slide is a phase of the fold period (seamless across octaves).
-- Per field: one serial multiplier (wipe, pane third, 2^x, speed);
-- per line: another (lens profile, interpolated; Barrel; 1/g; fringe).
-- Pixel path: pane -> scale -> fold -> Y / fringe products -> three read
-- addresses.  U/V swapped on rows whose hsync->avid parity differs from
-- the stored row's (drape v1.3).  Highlight knee + [1 2 1] on Y/U/V
-- (HD HDMI bars) before the gate.  Latency 23 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture hallway of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (23 + 1 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 23;

    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
    end function;

    -- lens profile 256 / sqrt(1 - (i/64)^2); entry 64 (infinite) reads 2047
    type t_tab is array (0 to 63) of integer range 0 to 2047;
    constant C_SPH : t_tab := (256, 256, 256, 256, 257, 257, 257, 258, 258, 259, 259, 260, 261, 261, 262, 263, 264, 266, 267, 268, 269, 271, 273, 274, 276, 278, 280, 282, 285, 287, 290, 293, 296, 299, 302, 306, 310, 314, 318, 323, 328, 333, 339, 346, 353, 360, 368, 377, 387, 398, 410, 424, 439, 457, 477, 501, 529, 563, 606, 661, 736, 846, 1032, 1454);

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_panes, s_k_wipe, s_k_bulge, s_k_sep, s_k_seg, s_k_slide : unsigned(9 downto 0);
    signal s_sw_mirror, s_sw_perpane, s_sw_repeat, s_sw_medge, s_sw_flip : std_logic;
    signal s_p_depth : unsigned(9 downto 0);

    -- input, counters
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_we, s_wpar, s_rbank : std_logic := '0';
    signal s_line, s_height : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wmax : unsigned(10 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';
    signal s_hpar, s_line_par, s_cap_par, s_swap : std_logic := '0';

    -- per field
    signal s_vstep : unsigned(5 downto 0) := (others => '1');
    -- resolution scaling: per-mode constants decoded every clock from the
    -- timing ID -- log2 of the 1080p-equivalent width (1920 / x step, 1/256
    -- octave), the segment floor, and that width for the slide speed
    signal s_tid : std_logic_vector(3 downto 0) := "0011";
    signal s_lwm : unsigned(11 downto 0) := to_unsigned(2784, 12);
    signal s_lfl : signed(13 downto 0) := to_signed(768, 14);
    signal s_wsp : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_wf : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_hf : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_nw : unsigned(10 downto 0) := (others => '1');
    signal s_ne : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_lw : unsigned(11 downto 0) := (others => '0');
    signal s_wipe_g, s_bulge_g : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_lt : signed(13 downto 0) := (others => '0');
    signal s_lg : signed(13 downto 0) := to_signed(2048, 14);
    signal s_lc : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal s_n : unsigned(2 downto 0) := "010";
    signal s_kbm : unsigned(8 downto 0) := (others => '0');
    signal s_kbs : std_logic := '0';
    signal s_sep : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_spd_d : unsigned(8 downto 0) := (others => '0');
    signal s_spd_neg : std_logic := '0';
    signal s_ks : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_gx, s_gy, s_gx2, s_gy2 : unsigned(8 downto 0) := (others => '0');
    signal s_t1, s_t2 : unsigned(17 downto 0) := (others => '0');
    signal s_mg : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_mi : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_spdw : unsigned(6 downto 0) := (others => '0');
    signal s_spdu : unsigned(7 downto 0) := (others => '0');
    signal s_dphi, s_phi : unsigned(15 downto 0) := (others => '0');
    signal s_slide_u : unsigned(10 downto 0) := (others => '0');
    signal s_fmask, s_flow, s_fhalf : unsigned(10 downto 0) := (others => '0');
    signal s_quarter : unsigned(9 downto 0) := (others => '0');
    signal s_wipex : unsigned(10 downto 0) := (others => '0');
    signal s_pw3, s_pw, s_hw, s_b2, s_b3 : unsigned(10 downto 0) := (others => '0');
    signal s_e1, s_e2, s_e3 : unsigned(10 downto 0) := (others => '1');
    signal s_c0, s_c1, s_c2, s_c3 : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_cx, s_cy : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_w2m1 : signed(14 downto 0) := (others => '0');
    signal s_dq : unsigned(16 downto 0) := (others => '0');
    signal s_dr : unsigned(11 downto 0) := (others => '0');
    signal s_dbit : unsigned(4 downto 0) := (others => '0');
    signal s_inv_r : unsigned(11 downto 0) := (others => '0');
    signal s_fm_a : unsigned(13 downto 0) := (others => '0');
    signal s_fm_b : unsigned(11 downto 0) := (others => '0');
    signal s_fm_acc : unsigned(25 downto 0) := (others => '0');
    signal s_fm_n : unsigned(3 downto 0) := (others => '0');

    -- per line
    signal s_lstep : unsigned(3 downto 0) := (others => '1');
    signal s_ady : unsigned(10 downto 0) := (others => '0');
    signal s_t6 : unsigned(6 downto 0) := (others => '0');
    signal s_frac : unsigned(7 downto 0) := (others => '0');
    signal s_f0, s_f1 : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_nzy : unsigned(12 downto 0) := (others => '0');
    signal s_zy : unsigned(12 downto 0) := to_unsigned(1024, 13);   -- Y scale, 1024 = 1
    signal s_zs : unsigned(11 downto 0) := (others => '0');          -- fringe, same units
    signal s_lm_a : unsigned(12 downto 0) := (others => '0');
    signal s_lm_b : unsigned(11 downto 0) := (others => '0');
    signal s_lm_acc : unsigned(25 downto 0) := (others => '0');
    signal s_lm_n : unsigned(3 downto 0) := (others => '0');

    -- pixel path (signals named for the stage they hold)
    signal s_xw2 : unsigned(11 downto 0) := (others => '0');
    signal s_xw3, s_xw4, s_xw5 : unsigned(10 downto 0) := (others => '0');
    signal s_p4 : unsigned(1 downto 0) := (others => '0');
    signal s_c5 : unsigned(10 downto 0) := (others => '0');
    signal s_mir5 : std_logic := '0';
    signal s_a6 : signed(11 downto 0) := (others => '0');
    type t_pcs is array (6 to 15) of unsigned(10 downto 0);
    signal s_pcs : t_pcs := (others => (others => '0'));
    signal s_ph7, s_pl7 : signed(17 downto 0) := (others => '0');
    signal s_u8 : signed(21 downto 0) := (others => '0');
    signal s_u9 : unsigned(12 downto 0) := (others => '0');
    signal s_m10, s_t11 : unsigned(10 downto 0) := (others => '0');
    signal s_o12 : signed(11 downto 0) := (others => '0');
    signal s_py13h : signed(19 downto 0) := (others => '0');
    signal s_py13l, s_ps13h, s_ps13l : signed(18 downto 0) := (others => '0');
    signal s_offy14, s_offs14 : signed(25 downto 0) := (others => '0');
    signal s_offy15, s_offu15, s_offv15 : signed(13 downto 0) := (others => '0');
    signal s_ady16, s_adu16, s_adv16 : signed(14 downto 0) := (others => '0');
    signal s_addrY, s_addrU, s_addrV : unsigned(10 downto 0) := (others => '0');
    signal s_bgY, s_bgU, s_bgV : std_logic := '0';
    signal s_swp15, s_swp16, s_swp17, s_swp18 : std_logic := '0';
    signal s_a_bgY, s_a_bgU, s_a_bgV, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_yk20 : signed(10 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_ya, s_yb, s_ua, s_ub, s_va, s_vb : unsigned(9 downto 0) := (others => '0');
    signal s_y23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_panes <= unsigned(registers_in(0));
    s_k_wipe  <= unsigned(registers_in(1));
    s_k_bulge <= unsigned(registers_in(2));
    s_k_sep   <= unsigned(registers_in(3));
    s_k_seg   <= unsigned(registers_in(4));
    s_k_slide <= unsigned(registers_in(5));
    s_sw_mirror  <= registers_in(6)(0);
    s_sw_perpane <= registers_in(6)(1);
    s_sw_repeat  <= registers_in(6)(2);
    s_sw_medge   <= registers_in(6)(3);
    s_sw_flip    <= registers_in(6)(4);
    s_p_depth <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_r : unsigned(12 downto 0);
        variable v_w : unsigned(10 downto 0);
        variable v_d, v_g : signed(10 downto 0);
        variable v_l : signed(13 downto 0);
        variable v_m : unsigned(11 downto 0);
        variable v_gx, v_gy : unsigned(8 downto 0);
        variable v_pre : unsigned(12 downto 0);
        variable v_srow : unsigned(10 downto 0);
        variable v_rom : unsigned(10 downto 0);
        variable v_t : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- resolution scaling: 1080 (x step 1): Mitchell log2(1920) = 2784,
            -- floor 2^3 px; 720p (1.5): -150 (log2 1.5); SD (2): -256
            s_tid <= registers_in(8)(3 downto 0);
            case to_integer(unsigned(s_tid)) is
                when 0 | 4 | 8 | 12 =>
                    s_lwm <= to_unsigned(2528, 12); s_lfl <= to_signed(512, 14); s_wsp <= to_unsigned(960, 11);
                when 5 | 6 | 14 =>
                    s_lwm <= to_unsigned(2634, 12); s_lfl <= to_signed(618, 14); s_wsp <= to_unsigned(1280, 11);
                when others =>
                    s_lwm <= to_unsigned(2784, 12); s_lfl <= to_signed(768, 14); s_wsp <= to_unsigned(1920, 11);
            end case;

            -- chroma phase: hsync->avid parity of this line vs the stored line
            s_hpar <= not s_hpar;
            if data_in.avid = '1' and s_prev_avid = '0' then s_line_par <= s_hpar; end if;
            s_swap <= s_line_par xor s_cap_par;

            -- serial multipliers (12 steps, MSB first); a load below wins
            if s_fm_n /= 0 then
                if s_fm_b(11) = '1' then
                    s_fm_acc <= (s_fm_acc(24 downto 0) & '0') + resize(s_fm_a, 26);
                else
                    s_fm_acc <= s_fm_acc(24 downto 0) & '0';
                end if;
                s_fm_b <= s_fm_b(10 downto 0) & '0';
                s_fm_n <= s_fm_n - 1;
            end if;
            if s_lm_n /= 0 then
                if s_lm_b(11) = '1' then
                    s_lm_acc <= (s_lm_acc(24 downto 0) & '0') + resize(s_lm_a, 26);
                else
                    s_lm_acc <= s_lm_acc(24 downto 0) & '0';
                end if;
                s_lm_b <= s_lm_b(10 downto 0) & '0';
                s_lm_n <= s_lm_n - 1;
            end if;

            ---------------------------------------------------------------
            -- per field (one op per state)
            ---------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    -- widest line of the last field, its height
                    if s_wmax < 64 then v_w := to_unsigned(64, 11); else v_w := s_wmax; end if;
                    s_wf <= v_w; s_nw <= v_w; s_ne <= to_unsigned(10, 4);
                    s_wmax <= (others => '0');
                    if s_height < 16 then s_hf <= to_unsigned(16, 11); else s_hf <= s_height; end if;
                    -- panes, matched to the display's rounding of 1 + 3k/1023
                    if s_k_panes < 171 then s_n <= "001";
                    elsif s_k_panes < 512 then s_n <= "010";
                    elsif s_k_panes < 853 then s_n <= "011";
                    else s_n <= "100"; end if;
                    -- fringe 8 .. 127 (/256 of the lens scale)
                    v_t := shift_left(resize(s_k_sep, 18), 7) - shift_left(resize(s_k_sep, 18), 3);
                    s_sep <= resize(v_t(16 downto 10), 7) + 8;
                    -- slide: +-16 detent, magnitude 0..495
                    if s_k_slide >= 528 then s_spd_d <= resize(s_k_slide - 528, 9); s_spd_neg <= '0';
                    elsif s_k_slide < 496 then s_spd_d <= resize(495 - s_k_slide, 9); s_spd_neg <= '1';
                    else s_spd_d <= (others => '0'); s_spd_neg <= '0'; end if;
                    -- glides (quarter per field, 4-code deadband against pot noise)
                    v_d := signed(resize(s_k_wipe, 11)) - signed(resize(s_wipe_g, 11));
                    v_g := signed(resize(s_wipe_g, 11)) + shift_right(v_d, 2);
                    if v_d >= 4 or v_d <= -4 then s_wipe_g <= unsigned(v_g(9 downto 0)); end if;
                    v_d := signed(resize(s_k_bulge, 11)) - signed(resize(s_bulge_g, 11));
                    v_g := signed(resize(s_bulge_g, 11)) + shift_right(v_d, 2);
                    if v_d >= 4 or v_d <= -4 then s_bulge_g <= unsigned(v_g(9 downto 0)); end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- normalize the width for log2 (wf >= 64: at most 4 shifts)
                    if s_nw(10) = '0' then
                        s_nw <= s_nw(9 downto 0) & '0'; s_ne <= s_ne - 1;
                    else
                        s_vstep <= s_vstep + 1;
                    end if;
                    s_cx <= '0' & s_wf(10 downto 1); s_cy <= '0' & s_hf(10 downto 1);
                    s_w2m1 <= signed(resize(s_wf & '0', 15)) - 1;
                    if s_bulge_g >= 512 then s_kbm <= resize(s_bulge_g(9 downto 1) - 256, 9); s_kbs <= '0';
                    else s_kbm <= resize(256 - s_bulge_g(9 downto 1), 9); s_kbs <= '1'; end if;
                when 2 =>
                    -- segment exponent L = 6 + K5 (4 octaves) - P12 (4 octaves)
                    -- - log2(x step): a fraction of the screen height (was
                    -- + log2(measured width / 1920), Mitchell)
                    s_lw <= s_lwm;
                    s_lt <= signed(resize(s_k_seg, 14)) - signed(resize(s_p_depth, 14)) - 1248;
                    s_fm_a <= resize(s_wf, 14); s_fm_b <= resize(s_wipe_g, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    if s_fm_n = 0 then
                        s_wipex <= s_fm_acc(20 downto 10);
                        s_lt <= s_lt + signed(resize(s_lw, 14));
                        s_vstep <= s_vstep + 1;
                    end if;
                when 4 =>
                    v_l := s_lt - s_lg;
                    if v_l >= 4 or v_l <= -4 then s_lg <= s_lg + shift_right(v_l, 2); end if;
                    s_fm_a <= resize(s_wf, 14); s_fm_b <= to_unsigned(683, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    if s_fm_n = 0 then
                        s_pw3 <= resize(s_fm_acc(20 downto 11), 11);
                        if s_lg < s_lfl then s_lc <= unsigned(s_lfl(11 downto 0));
                        elsif s_lg > 2559 then s_lc <= to_unsigned(2559, 12);
                        else s_lc <= unsigned(s_lg(11 downto 0)); end if;
                        s_vstep <= s_vstep + 1;
                    end if;
                when 6 =>
                    -- segment = 2^ks / g: g = 2^(gx/256), 1/g = 2^(gy/256) / 2
                    s_ks <= resize(s_lc(11 downto 8), 4) + 1;
                    v_gx := to_unsigned(256, 9) - resize(s_lc(7 downto 0), 9);
                    v_gy := resize(s_lc(7 downto 0), 9);
                    s_gx <= v_gx; s_gy <= v_gy;
                    s_fm_a <= resize(v_gx, 14); s_fm_b <= resize(v_gx, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    if s_fm_n = 0 then
                        s_gx2 <= s_fm_acc(16 downto 8);
                        s_fm_a <= resize(s_gy, 14); s_fm_b <= resize(s_gy, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                        s_vstep <= s_vstep + 1;
                    end if;
                when 8 =>
                    if s_fm_n = 0 then
                        s_gy2 <= s_fm_acc(16 downto 8);
                        s_fm_a <= resize(s_spd_d, 14); s_fm_b <= resize(s_spd_d, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                        s_vstep <= s_vstep + 1;
                    end if;
                -- 2^x ~ 1 + x (168 + 88 x) / 256, x in [0, 1]
                when 9 =>
                    s_t1 <= shift_left(resize(s_gx, 18), 7) + shift_left(resize(s_gx, 18), 5);
                    s_t2 <= shift_left(resize(s_gx, 18), 3) + shift_left(resize(s_gx2, 18), 6);
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    s_t1 <= s_t1 + s_t2;
                    s_t2 <= shift_left(resize(s_gx2, 18), 4) + shift_left(resize(s_gx2, 18), 3);
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    v_t := s_t1 + s_t2;
                    s_mg <= 256 + resize(v_t(17 downto 8), 10);
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    s_t1 <= shift_left(resize(s_gy, 18), 7) + shift_left(resize(s_gy, 18), 5);
                    s_t2 <= shift_left(resize(s_gy, 18), 3) + shift_left(resize(s_gy2, 18), 6);
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    s_t1 <= s_t1 + s_t2;
                    s_t2 <= shift_left(resize(s_gy2, 18), 4) + shift_left(resize(s_gy2, 18), 3);
                    s_vstep <= s_vstep + 1;
                when 14 =>
                    v_t := s_t1 + s_t2;
                    s_mi <= 256 + resize(v_t(17 downto 8), 10);
                    s_vstep <= s_vstep + 1;
                -- slide: (d^2 >> 11) 1/16 px a field, x (1920 / x step) / 2048,
                -- x g (was x measured width: SD slid 0.75x its segments)
                when 15 =>
                    if s_fm_n = 0 then
                        s_fm_a <= resize(s_fm_acc(17 downto 11), 14); s_fm_b <= resize(s_wsp, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                        s_vstep <= s_vstep + 1;
                    end if;
                when 16 =>
                    if s_fm_n = 0 then
                        s_spdw <= s_fm_acc(17 downto 11);
                        s_fm_a <= resize(s_fm_acc(17 downto 11), 14); s_fm_b <= resize(s_mg, 12); s_fm_acc <= (others => '0'); s_fm_n <= to_unsigned(12, 4);
                        s_vstep <= s_vstep + 1;
                    end if;
                when 17 =>
                    if s_fm_n = 0 then
                        s_spdu <= s_fm_acc(15 downto 8);
                        s_vstep <= s_vstep + 1;
                    end if;
                when 18 =>
                    -- phase step (1/65536 of a fold period of 2^(ks+1))
                    case to_integer(s_ks) is
                        when 3 => s_dphi <= shift_left(resize(s_spdu, 16), 8);
                        when 4 => s_dphi <= shift_left(resize(s_spdu, 16), 7);
                        when 5 => s_dphi <= shift_left(resize(s_spdu, 16), 6);
                        when 6 => s_dphi <= shift_left(resize(s_spdu, 16), 5);
                        when 7 => s_dphi <= shift_left(resize(s_spdu, 16), 4);
                        when 8 => s_dphi <= shift_left(resize(s_spdu, 16), 3);
                        when 9 => s_dphi <= shift_left(resize(s_spdu, 16), 2);
                        when others => s_dphi <= shift_left(resize(s_spdu, 16), 1);
                    end case;
                    s_vstep <= s_vstep + 1;
                when 19 =>
                    if s_spd_neg = '1' then s_phi <= s_phi - s_dphi; else s_phi <= s_phi + s_dphi; end if;
                    s_vstep <= s_vstep + 1;
                when 20 =>
                    case to_integer(s_ks) is
                        when 3 => s_slide_u <= resize(s_phi(15 downto 12), 11);
                        when 4 => s_slide_u <= resize(s_phi(15 downto 11), 11);
                        when 5 => s_slide_u <= resize(s_phi(15 downto 10), 11);
                        when 6 => s_slide_u <= resize(s_phi(15 downto 9), 11);
                        when 7 => s_slide_u <= resize(s_phi(15 downto 8), 11);
                        when 8 => s_slide_u <= resize(s_phi(15 downto 7), 11);
                        when 9 => s_slide_u <= resize(s_phi(15 downto 6), 11);
                        when others => s_slide_u <= s_phi(15 downto 5);
                    end case;
                    s_fhalf <= shift_left(to_unsigned(1, 11), to_integer(s_ks));
                    s_flow <= shift_left(to_unsigned(1, 11), to_integer(s_ks)) - 1;
                    v_m := shift_left(to_unsigned(2, 12), to_integer(s_ks)) - 1;
                    s_fmask <= v_m(10 downto 0);
                    s_quarter <= shift_left(to_unsigned(1, 10), to_integer(s_ks) - 1);
                    s_vstep <= s_vstep + 1;
                when 21 =>
                    case to_integer(s_n) is
                        when 1 => s_pw <= s_wf;
                        when 2 => s_pw <= '0' & s_wf(10 downto 1);
                        when 3 => s_pw <= s_pw3;
                        when others => s_pw <= "00" & s_wf(10 downto 2);
                    end case;
                    s_vstep <= s_vstep + 1;
                when 22 =>
                    s_hw <= '0' & s_pw(10 downto 1); s_b2 <= s_pw + s_pw;
                    s_vstep <= s_vstep + 1;
                when 23 =>
                    s_b3 <= s_b2 + s_pw; s_c0 <= s_hw; s_c1 <= s_pw + s_hw;
                    s_vstep <= s_vstep + 1;
                when 24 =>
                    s_c2 <= s_b2 + s_hw; s_c3 <= s_b3 + s_hw;
                    if s_n >= 2 then s_e1 <= s_pw; else s_e1 <= (others => '1'); end if;
                    if s_n >= 3 then s_e2 <= s_b2; else s_e2 <= (others => '1'); end if;
                    if s_n = 4 then s_e3 <= s_b3; else s_e3 <= (others => '1'); end if;
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(16, 5);
                    s_vstep <= s_vstep + 1;
                when 25 to 41 =>
                    -- 1 / half-height: 65536 / (cy + 1), restoring
                    if s_dbit = 16 then v_r := (s_dr & '1'); else v_r := (s_dr & '0'); end if;
                    if v_r >= resize(s_cy + 1, 13) then
                        s_dr <= resize(v_r - resize(s_cy + 1, 13), 12); s_dq <= s_dq(15 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0); s_dq <= s_dq(15 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    s_vstep <= s_vstep + 1;
                when 42 =>
                    if s_dq(16 downto 12) /= 0 then s_inv_r <= (others => '1'); else s_inv_r <= s_dq(11 downto 0); end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            ---------------------------------------------------------------
            -- per line, from hsync (~60 clocks): lens scale of the next row
            ---------------------------------------------------------------
            v_rom := to_unsigned(C_SPH(to_integer(s_t6(5 downto 0))), 11);
            case to_integer(s_lstep) is
                when 0 =>
                    if s_line >= s_cy then s_ady <= s_line - s_cy; else s_ady <= s_cy - s_line; end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_lm_a <= resize(s_ady, 13); s_lm_b <= s_inv_r; s_lm_acc <= (others => '0'); s_lm_n <= to_unsigned(12, 4);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_lm_n = 0 then
                        if s_lm_acc(25 downto 16) /= 0 then
                            s_t6 <= to_unsigned(63, 7); s_frac <= (others => '1');
                        else
                            s_t6 <= resize(s_lm_acc(15 downto 10), 7); s_frac <= s_lm_acc(9 downto 2);
                        end if;
                        s_lstep <= s_lstep + 1;
                    end if;
                when 3 =>
                    s_f0 <= v_rom; s_t6 <= s_t6 + 1;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_t6(6) = '1' then s_f1 <= to_unsigned(2047, 11); else s_f1 <= v_rom; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_lm_a <= resize(s_f1 - s_f0, 13); s_lm_b <= s_frac & "0000"; s_lm_acc <= (others => '0'); s_lm_n <= to_unsigned(12, 4);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    if s_lm_n = 0 then
                        -- profile - 1, interpolated (0 .. 1791)
                        s_lm_a <= resize(s_f0 + resize(s_lm_acc(21 downto 12), 11) - 256, 13);
                        s_lm_b <= resize(s_kbm, 12); s_lm_acc <= (others => '0'); s_lm_n <= to_unsigned(12, 4);
                        s_lstep <= s_lstep + 1;
                    end if;
                when 7 =>
                    if s_lm_n = 0 then
                        -- row scale 256 +- (f - 256) * Barrel / 128, in 32 .. 2047
                        v_pre := s_lm_acc(19 downto 7);
                        if s_kbs = '0' then
                            if v_pre >= 1791 then v_srow := to_unsigned(2047, 11); else v_srow := 256 + v_pre(10 downto 0); end if;
                        else
                            if v_pre >= 224 then v_srow := to_unsigned(32, 11); else v_srow := 256 - v_pre(10 downto 0); end if;
                        end if;
                        -- divided by g: Y scale in 1/1024
                        s_lm_a <= resize(v_srow, 13); s_lm_b <= resize(s_mi, 12); s_lm_acc <= (others => '0'); s_lm_n <= to_unsigned(12, 4);
                        s_lstep <= s_lstep + 1;
                    end if;
                when 8 =>
                    if s_lm_n = 0 then
                        s_nzy <= s_lm_acc(19 downto 7);
                        s_lm_a <= s_lm_acc(19 downto 7); s_lm_b <= resize(s_sep, 12); s_lm_acc <= (others => '0'); s_lm_n <= to_unsigned(12, 4);
                        s_lstep <= s_lstep + 1;
                    end if;
                when 9 =>
                    if s_lm_n = 0 then
                        s_zy <= s_nzy; s_zs <= s_lm_acc(19 downto 8);
                        s_lstep <= (others => '1');
                    end if;
                when others => null;
            end case;

            -- line end: count lines, widest line of the field
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line <= s_line + 1; s_height <= s_line + 1;
                if s_rx > s_wmax then s_wmax <= s_rx; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_cap_par <= s_line_par;   -- parity of the line now in the read bank
                s_hpar <= '0';
            end if;
            -- resets are idempotent (serrated vsync); the field sequencer is
            -- guarded by saw-active
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_wpar <= '0'; s_rbank <= '1';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
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

    ---------------------------------------------------------------------------
    -- read address: 1 x, 2-3 wipe, 4-5 pane, 6 pane coordinate, 7-9 scale
    -- by g + slide, 10-11 fold, 12 offset, 13-15 Y / fringe products,
    -- 16 address, 17 edges
    ---------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_y, v_s : signed(13 downto 0);
        procedure edge(signal src : in signed(14 downto 0); signal bg : out std_logic; signal addr : out unsigned(10 downto 0)) is
            variable n, r : signed(14 downto 0);
        begin
            n := not src; r := s_w2m1 - src;
            if src < 0 then
                if s_sw_medge = '1' and n < signed(resize(s_wf, 15)) then bg <= '0'; addr <= unsigned(n(10 downto 0)); else bg <= '1'; addr <= (others => '0'); end if;
            elsif src >= signed(resize(s_wf, 15)) then
                if s_sw_medge = '1' and r >= 0 then bg <= '0'; addr <= unsigned(r(10 downto 0)); else bg <= '1'; addr <= (others => '0'); end if;
            else bg <= '0'; addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            -- 2, 3: wiped position (s_wr_x is x at stage 1)
            s_xw2 <= resize(s_wr_x, 12) + resize(s_wipex, 12);
            if s_xw2 >= resize(s_wf, 12) then s_xw3 <= resize(s_xw2 - resize(s_wf, 12), 11); else s_xw3 <= s_xw2(10 downto 0); end if;
            -- 4: pane (unused bounds are 2047)
            if s_xw3 >= s_e3 then s_p4 <= "11";
            elsif s_xw3 >= s_e2 then s_p4 <= "10";
            elsif s_xw3 >= s_e1 then s_p4 <= "01";
            else s_p4 <= "00"; end if;
            s_xw4 <= s_xw3;
            -- 5: pane centre
            case s_p4 is
                when "00" => s_c5 <= s_c0;
                when "01" => s_c5 <= s_c1;
                when "10" => s_c5 <= s_c2;
                when others => s_c5 <= s_c3;
            end case;
            s_xw5 <= s_xw4; s_mir5 <= s_sw_mirror and s_p4(0);
            -- 6: coordinate about the pane centre (mirrored = c - 1 - x);
            -- the lens / source centre: screen or pane
            v_d := signed('0' & s_xw5) - signed('0' & s_c5);
            if s_mir5 = '1' then s_a6 <= not v_d; else s_a6 <= v_d; end if;
            if s_sw_perpane = '1' then s_pcs(6) <= s_c5; else s_pcs(6) <= s_cx; end if;
            for i in 7 to 15 loop s_pcs(i) <= s_pcs(i - 1); end loop;
            -- 7, 8: scale by g (mantissa split hi/lo)
            s_ph7 <= s_a6 * signed('0' & s_mg(9 downto 5));
            s_pl7 <= s_a6 * signed('0' & s_mg(4 downto 0));
            s_u8 <= shift_left(resize(s_ph7, 22), 5) + resize(s_pl7, 22);
            -- 9: plus the slide phase (mod 8192; every period divides it)
            s_u9 <= unsigned(s_u8(20 downto 8)) + resize(s_slide_u, 13);
            -- 10, 11: fold; mirror = triangle 0..2^ks, repeat = sawtooth 0..2^ks-1
            s_m10 <= s_u9(10 downto 0) and s_fmask;
            if s_sw_repeat = '1' or (s_m10 and s_fhalf) /= 0 then s_t11 <= s_m10 and s_flow;
            else s_t11 <= s_fhalf - s_m10; end if;
            -- 12: offset from the strip centre (fold units)
            s_o12 <= signed(resize(s_t11, 12)) - signed(resize(s_quarter, 12));
            -- 13, 14: Y offset (o * zy) and fringe (o * zs), 1/1024
            s_py13h <= s_o12 * signed('0' & s_zy(12 downto 6));
            s_py13l <= s_o12 * signed('0' & s_zy(5 downto 0));
            s_ps13h <= s_o12 * signed('0' & s_zs(11 downto 6));
            s_ps13l <= s_o12 * signed('0' & s_zs(5 downto 0));
            s_offy14 <= shift_left(resize(s_py13h, 26), 6) + resize(s_py13l, 26);
            s_offs14 <= shift_left(resize(s_ps13h, 26), 6) + resize(s_ps13l, 26);
            -- 15: per-RAM offsets; on a U/V-swapped row RAM U is read at V's
            -- position and vice versa
            v_y := s_offy14(23 downto 10); v_s := s_offs14(23 downto 10);
            s_offy15 <= v_y;
            if (s_sw_flip xor s_swap) = '0' then
                s_offu15 <= v_y + v_s; s_offv15 <= v_y - v_s;
            else
                s_offu15 <= v_y - v_s; s_offv15 <= v_y + v_s;
            end if;
            s_swp15 <= s_swap; s_swp16 <= s_swp15; s_swp17 <= s_swp16;
            -- 16: addresses
            s_ady16 <= signed(resize(s_pcs(15), 15)) + resize(s_offy15, 15);
            s_adu16 <= signed(resize(s_pcs(15), 15)) + resize(s_offu15, 15);
            s_adv16 <= signed(resize(s_pcs(15), 15)) + resize(s_offv15, 15);
            -- 17: edges
            edge(s_ady16, s_bgY, s_addrY);
            edge(s_adu16, s_bgU, s_addrU);
            edge(s_adv16, s_bgV, s_addrV);
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrU(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrU(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrV(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrV(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- pixel: 18 RAM data, 19 bank / edges / U-V swap, 20-21 knee + neutral
    -- gate, 22-23 [1 2 1] + gate
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ru, v_rv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 18
            s_a_bgY <= s_bgY; s_a_bgU <= s_bgU; s_a_bgV <= s_bgV; s_a_rbank <= s_rbank;
            s_swp18 <= s_swp17;
            -- 19
            if s_a_bgY = '1' then s_wet_y <= to_unsigned(64, 10);
            elsif s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); else s_wet_y <= unsigned(s_rdY0); end if;
            if s_a_bgU = '1' then v_ru := to_unsigned(512, 10);
            elsif s_a_rbank = '1' then v_ru := unsigned(s_rdU1) & "00"; else v_ru := unsigned(s_rdU0) & "00"; end if;
            if s_a_bgV = '1' then v_rv := to_unsigned(512, 10);
            elsif s_a_rbank = '1' then v_rv := unsigned(s_rdV1) & "00"; else v_rv := unsigned(s_rdV0) & "00"; end if;
            if s_swp18 = '1' then s_wet_u <= v_rv; s_wet_v <= v_ru; else s_wet_u <= v_ru; s_wet_v <= v_rv; end if;
            -- 20, 21: highlight knee (600 + (y - 600) / 2, at most 784),
            -- neutral outside avid so the smoothing's edge taps read black
            s_yk20 <= signed(resize(s_wet_y, 11)) - 600;
            s_y20 <= s_wet_y; s_u20 <= s_wet_u; s_v20 <= s_wet_v;
            if s_avid_sr(19) = '0' then
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            else
                if s_yk20 > 368 then s_y21 <= to_unsigned(784, 10);
                elsif s_yk20 > 0 then s_y21 <= to_unsigned(600, 10) + resize(unsigned(s_yk20(9 downto 1)), 10);
                else s_y21 <= s_y20; end if;
                s_u21 <= s_u20; s_v21 <= s_v20;
            end if;
            -- 22, 23: [1 2 1] on Y/U/V (output centre = s_ya), blanking gate
            s_ya <= s_y21; s_yb <= s_ya;
            s_ua <= s_u21; s_ub <= s_ua;
            s_va <= s_v21; s_vb <= s_va;
            if s_avid_sr(21) = '1' then
                s_y23 <= f_121(s_yb, s_ya, s_y21);
                s_u23 <= f_121(s_ub, s_ua, s_u21);
                s_v23 <= f_121(s_vb, s_va, s_v21);
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y23);
    s_io_align_in.u       <= std_logic_vector(s_u23);
    s_io_align_in.v       <= std_logic_vector(s_v23);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (23 -> 24).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture hallway;
