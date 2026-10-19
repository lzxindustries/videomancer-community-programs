-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Vhold: catalogue combo #73 (seed 20260922) -- rolling shutter / sync
-- roll + sync loss / horizontal collapse.  Three knobs each; the switches
-- and the slider decide how they combine.
--
--   K1 Roll     vertical-hold roll speed, bipolar (centre = locked, deadband)
--   K2 Bar      blanking bar height, 0.8 .. 26% of the screen
--   K3 Jello    rows lean by a sine of row + time (up to 1/8 of the width)
--   K4 Collapse base squeeze of every row, 1x .. 9x
--   K5 Reach    how far below the bar the pull (and the tear) reaches,
--               1/32 .. 8 screen heights (quadratic)
--   K6 Beam     squeezed rows brighten (beam concentrates), stretched dim
--
--   S7  Bar     black / the pulled picture shows through the bar
--   S8  Centre  rows collapse toward the screen centre (jello pans the
--               content) / toward the jello offset (the strips wobble)
--   S9  Edges   outside the line: black / mirrored
--   S10 Tear    rows below the bar tear sideways, fading with Reach
--   S11 Pull    seam pull squeezes / stretches (the same factor, as a zoom)
--   P12 Pull    extra squeeze just below the bar, decaying with Reach
--
-- v0.2.0.  Per field (one op per state, serial shift-add multiplier and
-- restoring divider): roll phase (16 bits, seam = phase * height, so the
-- bar wraps round), bar = height * (K2 + 32) / 4096, jello amplitude from
-- the width, reach length L = height * (8 (K5/1024)^2 + 1/32), slope =
-- 255 * 4096 / L, jello line step 1.5 cycles a screen; P12 and K4 glided.
-- Per line: distance below the bar modulo the height, pull weight pw =
-- 255 .. 0 over L, squeeze q = base + pw * pull (8.8), Stretch = base *
-- 256 / (256 + pw * pull) (serial divide), tear = random * pw * width / 8.
-- The read DDA keeps only 13 (HD) / 12 (SD) source bits, so heavy squeeze
-- repeats the picture every 8192 / 4096 source pixels (the tiled strips of
-- v0.1, kept).  Line buffer read a line late; U/V swapped on rows whose
-- hsync->avid parity differs from the stored row's (drape v1.3).  Jello
-- line phase advances on active lines only and the bottom field starts
-- half a step down.  Output: luma clamp 64..940, highlight knee (600 +
-- (y - 600)/2, whites ~770) and [1 2 1] on Y/U/V (the HD HDMI bar fix).
-- Latency 16 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vhold of program_top is
    -- data_in -> data_out latency is 16 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;

    constant C_LAT : integer := 16;

    -- [1 2 1] / 4 of three 10-bit taps
    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    -- a bank is only read while the other bank is written
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- controls (registered)
    signal s_k_roll, s_k_bar, s_k_jello, s_k_col, s_k_reach, s_k_beam, s_p_pull : unsigned(9 downto 0) := (others => '0');
    signal s_sw_barshow, s_sw_cjello, s_sw_mirror, s_sw_tear, s_sw_stretch : std_logic := '0';
    signal s_rd0, s_rd : signed(10 downto 0) := (others => '0');

    -- per field
    signal s_vstep : unsigned(4 downto 0) := (others => '1');
    signal s_hf, s_wf : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_rphase, s_jphase : unsigned(15 downto 0) := (others => '0');
    signal s_pullg, s_colg : unsigned(9 downto 0) := (others => '0');
    signal s_gdp, s_gdc : signed(10 downto 0) := (others => '0');
    signal s_pullf : unsigned(10 downto 0) := to_unsigned(16, 11);
    signal s_qbase : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_beam : unsigned(7 downto 0) := (others => '0');
    signal s_seam, s_barh : unsigned(10 downto 0) := (others => '0');
    signal s_jamp, s_tq : unsigned(7 downto 0) := (others => '0');
    signal s_L, s_Lh : unsigned(14 downto 0) := to_unsigned(64, 15);
    signal s_slope : unsigned(15 downto 0) := (others => '0');
    signal s_jinc : unsigned(15 downto 0) := to_unsigned(96, 16);
    signal s_cx : unsigned(10 downto 0) := to_unsigned(135, 11);
    signal s_wsg, s_negw, s_w2m1 : signed(12 downto 0) := (others => '0');
    signal s_wrap12 : std_logic := '1';
    signal s_fpar, s_ilace : std_logic := '0';
    -- serial shift-add multiplier (MSB first) and restoring divider
    signal s_ma : unsigned(15 downto 0) := (others => '0');
    signal s_mb : unsigned(10 downto 0) := (others => '0');
    signal s_macc : unsigned(26 downto 0) := (others => '0');
    signal s_mcnt : unsigned(3 downto 0) := (others => '0');
    signal s_dnum, s_dquo : unsigned(19 downto 0) := (others => '0');
    signal s_dden : unsigned(14 downto 0) := to_unsigned(1, 15);
    signal s_drem : unsigned(15 downto 0) := (others => '0');
    signal s_dcnt : unsigned(4 downto 0) := (others => '0');

    -- input / line bookkeeping
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_saw_active : std_logic := '0';
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_hpar, s_line_par, s_cap_par, s_swap : std_logic := '0';

    -- per line (one op per state)
    signal s_lstep : unsigned(5 downto 0) := (others => '1');
    signal s_d0, s_dp0 : signed(11 downto 0) := (others => '0');
    signal s_dist, s_dp : unsigned(10 downto 0) := (others => '0');
    signal s_inbar : std_logic := '0';
    signal s_rnd : signed(9 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_dprod : unsigned(26 downto 0) := (others => '0');
    signal s_dpl, s_dph, s_pql : unsigned(18 downto 0) := (others => '0');
    signal s_pqh : unsigned(17 downto 0) := (others => '0');
    signal s_pw : unsigned(7 downto 0) := (others => '0');
    signal s_addp : unsigned(18 downto 0) := (others => '0');
    signal s_jp, s_tearp : signed(18 downto 0) := (others => '0');
    signal s_tp : unsigned(15 downto 0) := (others => '0');
    signal s_add : unsigned(13 downto 0) := (others => '0');
    signal s_jel, s_tear : signed(10 downto 0) := (others => '0');
    signal s_ta : unsigned(7 downto 0) := (others => '0');
    signal s_qsq, s_den : unsigned(14 downto 0) := to_unsigned(256, 15);
    signal s_lnum, s_lquo : unsigned(19 downto 0) := (others => '0');
    signal s_lrem : unsigned(15 downto 0) := (others => '0');
    signal s_lcnt : unsigned(4 downto 0) := (others => '0');
    signal s_pc, s_cxj : signed(11 downto 0) := (others => '0');
    signal s_s0, s_pq, s_src0n : unsigned(20 downto 0) := (others => '0');
    signal s_qm : signed(15 downto 0) := (others => '0');
    signal s_vg : unsigned(9 downto 0) := (others => '0');
    signal s_dimm : unsigned(7 downto 0) := (others => '0');
    signal s_gp : unsigned(17 downto 0) := (others => '0');
    signal s_dmp : unsigned(15 downto 0) := (others => '0');
    -- row registers read by the pixel path (written in hblank only)
    signal s_q : unsigned(14 downto 0) := to_unsigned(256, 15);
    signal s_src0 : unsigned(20 downto 0) := (others => '0');
    signal s_lift_row : unsigned(9 downto 0) := (others => '0');
    signal s_g_row : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_bar_row : std_logic := '0';

    -- pixel path
    type t_b is array (1 to 4) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc : unsigned(20 downto 0) := (others => '0');
    signal s_src, s_src5, s_ref : signed(12 downto 0) := (others => '0');
    signal s_f_neg, s_f_ltw, s_f_gemw, s_f_lt2w : std_logic := '0';
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '1';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_yd9 : signed(10 downto 0) := (others => '0');
    signal s_ym10 : signed(20 downto 0) := (others => '0');
    signal s_ys11 : signed(12 downto 0) := (others => '0');
    signal s_yk13 : signed(10 downto 0) := (others => '0');
    signal s_u9, s_v9, s_u10, s_v10, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y12, s_u12, s_v12, s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    signal s_ya, s_yb, s_ua, s_ub, s_va, s_vb : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    p_ctrl : process(clk)
        variable v_t : signed(11 downto 0);
        variable v_r : unsigned(15 downto 0);
        variable v_g : signed(10 downto 0);
        variable v_n : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            s_k_roll  <= unsigned(registers_in(0));
            s_k_bar   <= unsigned(registers_in(1));
            s_k_jello <= unsigned(registers_in(2));
            s_k_col   <= unsigned(registers_in(3));
            s_k_reach <= unsigned(registers_in(4));
            s_k_beam  <= unsigned(registers_in(5));
            s_sw_barshow <= registers_in(6)(0);
            s_sw_cjello  <= registers_in(6)(1);
            s_sw_mirror  <= registers_in(6)(2);
            s_sw_tear    <= registers_in(6)(3);
            s_sw_stretch <= registers_in(6)(4);
            s_p_pull <= unsigned(registers_in(7));

            -- roll speed with a +-16 deadband round the centre (locked)
            s_rd0 <= signed(resize(s_k_roll, 11)) - 512;
            if s_rd0 > 16 then
                s_rd <= s_rd0 - 16;
            elsif s_rd0 < -16 then
                s_rd <= s_rd0 + 16;
            else
                s_rd <= (others => '0');
            end if;

            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            -- chroma phase: hsync->avid parity of this line vs the stored line
            s_hpar <= not s_hpar;
            if data_in.avid = '1' and s_prev_avid = '0' then s_line_par <= s_hpar; end if;
            s_swap <= s_line_par xor s_cap_par;

            ---------------------------------------------------------------
            -- per field (one op per state)
            ---------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    if s_height < 64 then s_hf <= to_unsigned(64, 11); else s_hf <= s_height; end if;
                    if s_line_width < 64 then s_wf <= to_unsigned(64, 11); else s_wf <= s_line_width; end if;
                    s_rphase <= s_rphase + unsigned(shift_left(resize(s_rd, 16), 2));
                    s_jphase <= s_jphase + 700;
                    s_gdp <= signed(resize(s_p_pull, 11)) - signed(resize(s_pullg, 11));
                    s_gdc <= signed(resize(s_k_col, 11)) - signed(resize(s_colg, 11));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    v_g := signed(resize(s_pullg, 11)) + shift_right(s_gdp, 2);
                    s_pullg <= unsigned(v_g(9 downto 0));
                    v_g := signed(resize(s_colg, 11)) + shift_right(s_gdc, 2);
                    s_colg <= unsigned(v_g(9 downto 0));
                    s_cx <= '0' & s_wf(10 downto 1);
                    s_wsg <= signed(resize(s_wf, 13));
                    s_negw <= -signed(resize(s_wf, 13));
                    s_w2m1 <= signed(resize(s_wf & '0', 13)) - 1;
                    s_tq <= s_wf(10 downto 3);
                    if s_wf < 1440 then s_wrap12 <= '1'; else s_wrap12 <= '0'; end if;
                    s_beam <= s_k_beam(9 downto 2);
                    -- seam = roll phase * height / 65536
                    s_ma <= s_rphase; s_mb <= s_hf; s_macc <= (others => '0'); s_mcnt <= (others => '1');
                    s_vstep <= s_vstep + 1;
                when 2 | 4 | 6 | 8 | 10 =>
                    if s_ma(15) = '1' then
                        s_macc <= shift_left(s_macc, 1) + resize(s_mb, 27);
                    else
                        s_macc <= shift_left(s_macc, 1);
                    end if;
                    s_ma <= shift_left(s_ma, 1);
                    s_mcnt <= s_mcnt - 1;
                    if s_mcnt = 0 then s_vstep <= s_vstep + 1; end if;
                when 3 =>
                    s_seam <= s_macc(26 downto 16);
                    s_pullf <= resize(s_pullg, 11) + 16;
                    s_qbase <= to_unsigned(256, 12) + resize(s_colg & '0', 12);
                    -- bar = height * (K2 + 32) / 4096
                    s_ma <= resize(s_k_bar, 16) + 32; s_mb <= s_hf; s_macc <= (others => '0'); s_mcnt <= (others => '1');
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_barh <= s_macc(22 downto 12);
                    -- jello amplitude = width * K3 / 8192 (up to 1/8 of the width)
                    s_ma <= resize(s_k_jello, 16); s_mb <= s_wf; s_macc <= (others => '0'); s_mcnt <= (others => '1');
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_jamp <= s_macc(20 downto 13);
                    -- K5 squared
                    s_ma <= resize(s_k_reach, 16); s_mb <= '0' & s_k_reach; s_macc <= (others => '0'); s_mcnt <= (others => '1');
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    -- reach length: height * (K5^2 / 1024) / 128 (up to 8 heights)
                    s_ma <= resize(s_macc(19 downto 10), 16); s_mb <= s_hf; s_macc <= (others => '0'); s_mcnt <= (others => '1');
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    s_L <= s_macc(21 downto 7);
                    s_Lh <= resize(s_hf(10 downto 5), 15) + 1;
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    s_L <= s_L + s_Lh;            -- + height / 32 + 1
                    s_dnum <= to_unsigned(1044480, 20); s_drem <= (others => '0');
                    s_dcnt <= to_unsigned(19, 5);
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    s_dden <= s_L;                -- slope = 255 * 4096 / L
                    s_vstep <= s_vstep + 1;
                when 14 | 16 =>
                    v_r := s_drem(14 downto 0) & s_dnum(19);
                    s_dnum <= shift_left(s_dnum, 1);
                    if v_r >= resize(s_dden, 16) then
                        s_drem <= v_r - resize(s_dden, 16); s_dquo <= s_dquo(18 downto 0) & '1';
                    else
                        s_drem <= v_r; s_dquo <= s_dquo(18 downto 0) & '0';
                    end if;
                    s_dcnt <= s_dcnt - 1;
                    if s_dcnt = 0 then s_vstep <= s_vstep + 1; end if;
                when 15 =>
                    if s_dquo(19 downto 16) /= 0 then s_slope <= (others => '1'); else s_slope <= s_dquo(15 downto 0); end if;
                    -- jello line step = 98304 / height (1.5 sine cycles a screen)
                    s_dnum <= to_unsigned(98304, 20); s_drem <= (others => '0'); s_dden <= resize(s_hf, 15);
                    s_dcnt <= to_unsigned(19, 5);
                    s_vstep <= s_vstep + 1;
                when 17 =>
                    s_jinc <= s_dquo(15 downto 0);
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            ---------------------------------------------------------------
            -- per line, from hsync: the next line's row registers
            ---------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    s_d0 <= signed(resize(s_line, 12)) - signed(resize(s_seam, 12));
                    s_sin_addr <= s_line_phase(15 downto 8);
                    s_rnd <= signed(resize(s_lfsr(8 downto 0), 10)) - 256;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    -- distance below the seam, modulo the height (the bar wraps)
                    v_t := s_d0 + signed(resize(s_hf, 12));
                    if s_d0 < 0 then s_dist <= unsigned(v_t(10 downto 0)); else s_dist <= unsigned(s_d0(10 downto 0)); end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_dp0 <= signed(resize(s_dist, 12)) - signed(resize(s_barh, 12));
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    -- pull is measured from the bar's bottom; bar rows sit at the peak
                    if s_dp0 < 0 then
                        s_inbar <= '1'; s_dp <= (others => '0');
                    else
                        s_inbar <= '0'; s_dp <= unsigned(s_dp0(10 downto 0));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    -- dp * slope, operand split hi/lo (one product per state)
                    s_dpl <= s_dp * s_slope(7 downto 0);
                    s_dph <= s_dp * s_slope(15 downto 8);
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_dprod <= shift_left(resize(s_dph, 27), 8) + resize(s_dpl, 27);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    if s_dprod(26 downto 12) >= 255 then
                        s_pw <= (others => '0');
                    else
                        s_pw <= 255 - s_dprod(19 downto 12);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_addp <= s_pw * s_pullf;
                    s_jp <= s_sin_q2 * signed('0' & s_jamp);
                    s_tp <= s_pw * s_tq;
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_add <= s_addp(18 downto 5);
                    s_jel <= s_jp(18 downto 8);
                    s_ta <= s_tp(15 downto 8);
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    s_qsq <= resize(s_qbase, 15) + resize(s_add, 15);
                    s_den <= resize(s_add, 15) + 256;
                    s_tearp <= s_rnd * signed('0' & s_ta);
                    s_lnum <= shift_left(resize(s_qbase, 20), 8); s_lrem <= (others => '0');
                    s_lcnt <= to_unsigned(19, 5);
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    -- Stretch: q = base * 256 / (256 + pull)
                    v_r := s_lrem(14 downto 0) & s_lnum(19);
                    s_lnum <= shift_left(s_lnum, 1);
                    if v_r >= resize(s_den, 16) then
                        s_lrem <= v_r - resize(s_den, 16); s_lquo <= s_lquo(18 downto 0) & '1';
                    else
                        s_lrem <= v_r; s_lquo <= s_lquo(18 downto 0) & '0';
                    end if;
                    s_lcnt <= s_lcnt - 1;
                    if s_lcnt = 0 then s_lstep <= s_lstep + 1; end if;
                when 11 =>
                    if s_sw_stretch = '1' then
                        if s_lquo < 16 then s_q <= to_unsigned(16, 15); else s_q <= s_lquo(14 downto 0); end if;
                    else
                        s_q <= s_qsq;
                    end if;
                    if s_sw_tear = '1' then s_tear <= s_tearp(18 downto 8); else s_tear <= (others => '0'); end if;
                    -- Jello: the collapse centre wobbles; Screen: the content pans
                    if s_sw_cjello = '1' then
                        s_pc <= signed(resize(s_cx, 12)) + resize(s_jel, 12);
                        s_cxj <= signed(resize(s_cx, 12));
                    else
                        s_pc <= signed(resize(s_cx, 12));
                        s_cxj <= signed(resize(s_cx, 12)) + resize(s_jel, 12);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 12 =>
                    s_pc <= s_pc + resize(s_tear, 12);
                    s_s0 <= shift_left(unsigned(resize(s_cxj, 21)), 8) - resize(s_q, 21);
                    s_lstep <= s_lstep + 1;
                when 13 =>
                    -- source x = cxj + (x - pc) * q / 256, kept modulo 2^21;
                    -- pc * q split hi/lo
                    s_pql <= unsigned(s_pc(10 downto 0)) * s_q(7 downto 0);
                    s_pqh <= unsigned(s_pc(10 downto 0)) * s_q(14 downto 8);
                    s_qm <= signed(resize(s_q, 16)) - 256;
                    s_lstep <= s_lstep + 1;
                when 14 =>
                    s_pq <= shift_left(resize(s_pqh, 21), 8) + resize(s_pql, 21);
                    -- beam: squeeze lifts (as v0.1), stretch dims
                    if s_qm >= 0 then
                        if s_qm(15 downto 3) > 700 then s_vg <= to_unsigned(700, 10); else s_vg <= unsigned(s_qm(12 downto 3)); end if;
                        s_dimm <= (others => '0');
                    else
                        s_vg <= (others => '0');
                        v_n := -s_qm;
                        s_dimm <= unsigned(v_n(7 downto 0));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 15 =>
                    s_src0n <= s_s0 - s_pq;
                    s_gp <= s_vg * s_beam;
                    s_dmp <= s_dimm * s_beam;
                    s_lstep <= s_lstep + 1;
                when 16 =>
                    s_src0 <= s_src0n;
                    s_lift_row <= s_gp(17 downto 8);
                    s_g_row <= to_unsigned(256, 9) - resize(s_dmp(15 downto 8), 9);
                    s_bar_row <= s_inbar;
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            -- line end: count lines; the jello phase steps on active lines only
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                s_line_phase <= s_line_phase + s_jinc;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_cap_par <= s_line_par;   -- parity of the line now in the read bank
                s_hpar <= '0';
                -- before the first active line: seed the jello phase (idempotent,
                -- the last blanking hsync wins); the bottom field starts half a
                -- step down
                if s_saw_active = '0' then
                    if s_ilace = '1' and data_in.field_n = '0' then
                        s_line_phase <= s_jphase + shift_right(s_jinc, 1);
                    else
                        s_line_phase <= s_jphase;
                    end if;
                end if;
            end if;
            -- resets are idempotent (serrated vsync); the field sequencer and
            -- the interlace sample are guarded by saw-active
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_wpar <= '0'; s_rbank <= '1';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                    s_fpar <= data_in.field_n;
                    if data_in.field_n /= s_fpar then s_ilace <= '1'; else s_ilace <= '0'; end if;
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

    ---------------------------------------------------------------------------
    -- read address: 3 DDA, 4 source wrap, 5 region tests, 6 address
    ---------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_avd(1) <= s_in_avid;
            for i in 2 to 4 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 3
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_q, 21); end if;
            -- 4: 13 source bits (HD) / 12 (SD): heavy squeeze tiles the picture
            if s_wrap12 = '1' then
                s_src <= resize(signed(s_acc(19 downto 8)), 13);
            else
                s_src <= signed(s_acc(20 downto 8));
            end if;
            -- 5
            s_src5 <= s_src;
            s_f_neg <= s_src(12);
            if s_src < s_wsg then s_f_ltw <= '1'; else s_f_ltw <= '0'; end if;
            if s_src >= s_negw then s_f_gemw <= '1'; else s_f_gemw <= '0'; end if;
            if s_src <= s_w2m1 then s_f_lt2w <= '1'; else s_f_lt2w <= '0'; end if;
            s_ref <= s_w2m1 - s_src;
            -- 6
            if s_avd(4) = '0' then
                s_bg <= '1'; s_addr <= (others => '0');
            elsif s_f_neg = '1' then
                if s_sw_mirror = '1' and s_f_gemw = '1' then
                    s_bg <= '0'; s_addr <= unsigned(not s_src5(10 downto 0));
                else
                    s_bg <= '1'; s_addr <= (others => '0');
                end if;
            elsif s_f_ltw = '1' then
                s_bg <= '0'; s_addr <= unsigned(s_src5(10 downto 0));
            elsif s_sw_mirror = '1' and s_f_lt2w = '1' then
                s_bg <= '0'; s_addr <= unsigned(s_ref(10 downto 0));
            else
                s_bg <= '1'; s_addr <= (others => '0');
            end if;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- pixel path: 7 RAM data, 8 fetch, 9 U/V swap, 10 beam gain, 11 beam
    -- lift, 12 clamp + bar, 13-14 knee + neutral gate, 15-16 [1 2 1] + gate
    ---------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- 7 / 8
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;

            -- 9: U/V swap where this row's parity differs from the stored row's
            s_yd9 <= signed(resize(s_wet_y, 11)) - 64;
            if s_swap = '1' then
                s_u9 <= s_wet_v; s_v9 <= s_wet_u;
            else
                s_u9 <= s_wet_u; s_v9 <= s_wet_v;
            end if;

            -- 10: beam gain (stretched rows dim; 256 = unity)
            s_ym10 <= s_yd9 * signed('0' & s_g_row);
            s_u10 <= s_u9; s_v10 <= s_v9;

            -- 11: beam lift (squeezed rows)
            s_ys11 <= resize(s_ym10(18 downto 8), 13) + 64 + signed(resize(s_lift_row, 13));
            s_u11 <= s_u10; s_v11 <= s_v10;

            -- 12: legal luma, bar
            if s_bar_row = '1' and s_sw_barshow = '0' then
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            else
                if s_ys11 < 64 then
                    s_y12 <= to_unsigned(64, 10);
                elsif s_ys11 > 940 then
                    s_y12 <= to_unsigned(940, 10);
                else
                    s_y12 <= unsigned(s_ys11(9 downto 0));
                end if;
                s_u12 <= s_u11; s_v12 <= s_v11;
            end if;

            -- 13-14: highlight knee (940 -> 770), neutral outside avid so the
            -- smoothing's edge taps read black
            s_yk13 <= signed(resize(s_y12, 11)) - 600;
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            if s_avid_sr(12) = '0' then
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            else
                if s_yk13 > 0 then
                    s_y14 <= to_unsigned(600, 10) + unsigned(s_yk13(9 downto 1));
                else
                    s_y14 <= s_y13;
                end if;
                s_u14 <= s_u13; s_v14 <= s_v13;
            end if;

            -- 15-16: [1 2 1] on Y/U/V (output centre = s_ya), blanking gate
            s_ya <= s_y14; s_yb <= s_ya;
            s_ua <= s_u14; s_ub <= s_ua;
            s_va <= s_v14; s_vb <= s_va;
            if s_avid_sr(14) = '1' then
                s_y16 <= f_121(s_yb, s_ya, s_y14);
                s_u16 <= f_121(s_ub, s_ua, s_u14);
                s_v16 <= f_121(s_vb, s_va, s_v14);
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture vhold;
