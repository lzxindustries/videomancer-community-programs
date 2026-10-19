-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Outliner: catalogue combo #13 (seed 20260914) -- edge detect + split-screen
-- + invert / negative / bit-plane.  Two knobs each; the switches and the
-- slider decide how they combine.
--
--   K1 Threshold edge threshold (curved: fine steps at the low end)
--   K2 Kernel    Roberts 2x2 / Sobel 3x3 / Wide (8-px span, lines thickened
--                down 3 rows) / DoG (difference of 5- and 17-px boxes, plus a
--                vertical Laplacian)
--   K3 Seam      split position
--   K4 Panes     1 / 2 / 4 / 8 vertical strips alternating processed / original
--   K5 Plane     negative / bit-planes 8..2
--   K6 Mix       negative or plane mixed in
--
--   S7  Edges    white lines on black / lines drawn over the picture
--   S8  Invert   the edge side / the other side
--   S9  Lines    on the edge side only / on both sides
--   S10 Invert   the whole area / edge pixels only (lines take the inverted
--                picture instead of white)
--   S11 Colour   edge lines white / with the source chroma
--   P12 Sweep    the seam swings on a sine; 100% = edge to edge in ~1.7 s
--
-- Edges are 2-D.  Two line buffers hold rows r-1 and r-2 (8-bit luma), so
-- every kernel sees a 3-row window; the picture is row r, so lines sit one
-- row low (invisible).  An inverted line drawing (edge side, lines on black)
-- turns into black lines on white; a bit plane keys where the drawing flips.
--
-- Hardware contracts: blanking-gated input and output, one latency (26) in
-- all modes, the picture carried in an EBR ring instead of a deep FF pipe,
-- no kernel edges in a 12-px border or the first 3 rows of a field, and the
-- HD HDMI bar fix ([1 2 1] on Y/U/V + luma scaled toward black, white ~776).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture outliner of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (26 + 2 = 28 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 28;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    -- Pixel c enters stage 1 on clock c; stage s registers hold it after
    -- clock c + s - 1, and the avid of the pixel computed INTO stage s is
    -- s_avid_sr(s - 2).
    constant C_LAT    : integer := 26;
    constant C_MARGIN : integer := 12;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251, -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215, -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147, -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,  -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,   50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,  142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,  212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,  250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,  250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,  212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,  142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,   50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,  -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136, -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208, -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249, -250, -251, -252, -253, -254, -254, -255, -255
    );

    signal s_sw_over, s_sw_otherinv, s_sw_both, s_sw_invedges, s_sw_colour : std_logic;

    -- per-frame values, published at the end of the vblank sequence
    signal s_thr     : unsigned(11 downto 0) := to_unsigned(60, 12);
    signal s_kern    : unsigned(1 downto 0) := (others => '0');
    signal s_wide    : std_logic := '0';
    signal s_pshift  : unsigned(1 downto 0) := (others => '0');
    signal s_plane   : unsigned(2 downto 0) := (others => '0');
    -- K2/K4/K5 label decodes, free-running.  Labelled defaults: at program
    -- load the firmware writes the initial_value_label's INDEX (0..N-1) into
    -- the register, not a 0..1023 position, so a value below N is the index.
    signal s_k2_idx  : unsigned(1 downto 0) := (others => '0');
    signal s_k4_idx  : unsigned(1 downto 0) := (others => '0');
    signal s_k5_idx  : unsigned(2 downto 0) := (others => '0');
    signal s_mix     : unsigned(8 downto 0) := (others => '0');
    signal s_seam_px : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_pane_m1 : unsigned(10 downto 0) := to_unsigned(479, 11);
    signal s_xmax    : unsigned(10 downto 0) := to_unsigned(467, 11);

    -- raster measurement
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_avid, s_prev_vsync_n : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_blank_cnt  : unsigned(12 downto 0) := (others => '0');
    signal s_rows    : unsigned(1 downto 0) := (others => '0');
    signal s_rowok   : std_logic := '0';

    -- vblank sequencer on one serial shift-add multiplier
    signal s_sq      : unsigned(3 downto 0) := (others => '1');   -- 15 = idle
    signal s_k1, s_k3, s_k12 : unsigned(9 downto 0) := (others => '0');
    signal s_wl      : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_mul_a   : unsigned(21 downto 0) := (others => '0');
    signal s_mul_b   : unsigned(10 downto 0) := (others => '0');
    signal s_mul_p   : unsigned(21 downto 0) := (others => '0');
    signal s_mul_n   : unsigned(3 downto 0) := (others => '0');
    signal s_base    : unsigned(10 downto 0) := (others => '0');
    signal s_amp     : unsigned(10 downto 0) := (others => '0');
    signal s_swing   : unsigned(10 downto 0) := (others => '0');
    signal s_sneg    : std_logic := '0';
    signal s_pos     : signed(12 downto 0) := (others => '0');
    signal s_seam_n  : unsigned(10 downto 0) := (others => '0');
    signal s_thr_n   : unsigned(11 downto 0) := to_unsigned(60, 12);
    signal s_inc_t   : unsigned(10 downto 0) := (others => '0');
    signal s_inc     : unsigned(10 downto 0) := to_unsigned(16, 11);
    signal s_sweep_ph : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');

    -- line buffers (rows r-1, r-2) and the Wide kernel's flags (rows r-1..r-3)
    type t_line is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal r_lb1, r_lb2 : t_line := (others => (others => '0'));
    type t_flags is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal r_fl : t_flags := (others => (others => '0'));
    -- the picture, stage 1 to stage 18
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_pic : t_ring := (others => (others => '0'));
    -- No RAM here reads the address it writes on the same clock (line
    -- buffers and flags write the column two stages behind the read; the
    -- ring reads 15 slots back), so yosys need not emulate read-before-write
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_lb1, r_lb2, r_fl, r_pic : signal is true;
    signal s_q1, s_q2 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_fq       : std_logic_vector(3 downto 0) := (others => '0');
    signal s_pq, s_pq2 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_pwp, s_prp : unsigned(7 downto 0) := (others => '0');
    signal s_rv1      : std_logic_vector(31 downto 0) := (others => '0');

    -- stages 1..4: luma, columns, rows
    signal s_l1, s_l2 : unsigned(7 downto 0) := (others => '0');
    signal s_x1, s_x2, s_x3 : unsigned(10 downto 0) := (others => '0');
    signal s_pc       : unsigned(10 downto 0) := (others => '0');
    signal s_pi       : unsigned(3 downto 0) := (others => '0');
    signal s_c0, s_c1, s_c2 : unsigned(7 downto 0) := (others => '0');
    signal s_v4       : unsigned(9 downto 0) := (others => '0');
    signal s_d4       : signed(8 downto 0) := (others => '0');
    signal s_lp4      : signed(9 downto 0) := (others => '0');
    signal s_r0_4, s_r1_4 : unsigned(7 downto 0) := (others => '0');
    -- stage 5: column taps (index 0 newest; kernels centre on tap 8)
    type t_vt is array (0 to 16) of unsigned(9 downto 0);
    signal s_vt : t_vt := (others => (others => '0'));
    type t_dt is array (0 to 11) of signed(8 downto 0);
    signal s_dt : t_dt := (others => (others => '0'));
    type t_lt is array (0 to 9) of signed(9 downto 0);
    signal s_lt : t_lt := (others => (others => '0'));
    type t_rt is array (0 to 9) of unsigned(7 downto 0);
    signal s_r0t, s_r1t : t_rt := (others => (others => '0'));
    signal s_acc5  : unsigned(12 downto 0) := (others => '0');
    signal s_acc17 : unsigned(14 downto 0) := (others => '0');
    signal s_accd  : signed(11 downto 0) := (others => '0');
    -- stage 14: kernel terms
    signal s_rbx, s_rby : signed(8 downto 0) := (others => '0');
    signal s_sbx, s_wdx : signed(10 downto 0) := (others => '0');
    signal s_sby, s_wdy, s_lpv : signed(11 downto 0) := (others => '0');
    signal s_p5, s_p17 : unsigned(17 downto 0) := (others => '0');
    -- stages 15..19: gradient pair, magnitudes, flag
    signal s_ga, s_gb : signed(11 downto 0) := (others => '0');
    signal s_aa, s_ab : unsigned(11 downto 0) := (others => '0');
    signal s_mag : unsigned(11 downto 0) := (others => '0');
    signal s_rxl, s_x16, s_x17, s_x18 : unsigned(10 downto 0) := (others => '0');
    signal s_f18 : std_logic := '0';
    signal s_fr18 : std_logic_vector(3 downto 0) := (others => '0');
    signal s_e19 : std_logic := '0';
    -- 1-bit pipes: border (stages 2..17), processed side (stages 3..19)
    signal s_bd : std_logic_vector(2 to 17) := (others => '1');
    signal s_pr : std_logic_vector(3 to 19) := (others => '0');
    -- stages 19..26: compose, mix, conditioning, gate
    signal s_py19, s_pu19, s_pv19, s_t19 : unsigned(9 downto 0) := (others => '0');
    signal s_key19 : std_logic := '0';
    signal s_a20, s_b20 : unsigned(9 downto 0) := (others => '0');
    signal s_u20, s_v20, s_u21, s_v21, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_a21, s_a22 : unsigned(9 downto 0) := (others => '0');
    signal s_d21 : signed(10 downto 0) := (others => '0');
    signal s_p22 : signed(20 downto 0) := (others => '0');
    signal s_y23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y26 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_sw_over      <= registers_in(6)(0);
    s_sw_otherinv  <= registers_in(6)(1);
    s_sw_both      <= registers_in(6)(2);
    s_sw_invedges  <= registers_in(6)(3);
    s_sw_colour    <= registers_in(6)(4);

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Raster measurement and the vblank sequence: seam position, sweep, pane
    -- width, threshold and sweep speed, one product at a time on a serial
    -- multiplier (11 clocks each).
    ----------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_abs : unsigned(7 downto 0);
        variable v_neg : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            if unsigned(registers_in(1)) < 4 then s_k2_idx <= unsigned(registers_in(1)(1 downto 0));
            else s_k2_idx <= unsigned(registers_in(1)(9 downto 8)); end if;
            if unsigned(registers_in(3)) < 4 then s_k4_idx <= unsigned(registers_in(3)(1 downto 0));
            else s_k4_idx <= unsigned(registers_in(3)(9 downto 8)); end if;
            if unsigned(registers_in(4)) < 8 then s_k5_idx <= unsigned(registers_in(4)(2 downto 0));
            else s_k5_idx <= unsigned(registers_in(4)(9 downto 7)); end if;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_prev_avid = '1' and data_in.avid = '0' then s_line_width <= s_rx; end if;

            -- rows since the vertical blank (avid absent for 4096 clocks, two
            -- HD lines); a field's first three rows have no rows above them
            if data_in.avid = '1' then
                s_blank_cnt <= (others => '0');
            elsif s_blank_cnt(12) = '0' then
                s_blank_cnt <= s_blank_cnt + 1;
            end if;
            if s_blank_cnt(12) = '1' then
                s_rows <= (others => '0');
            elsif s_prev_avid = '1' and data_in.avid = '0' and s_rows /= 3 then
                s_rows <= s_rows + 1;
            end if;
            if s_prev_avid = '0' and data_in.avid = '1' then
                if s_rows = 3 then s_rowok <= '1'; else s_rowok <= '0'; end if;
            end if;

            if s_mul_n /= 0 then
                if s_mul_b(0) = '1' then s_mul_p <= s_mul_p + s_mul_a; end if;
                s_mul_a <= shift_left(s_mul_a, 1);
                s_mul_b <= shift_right(s_mul_b, 1);
                s_mul_n <= s_mul_n - 1;
            end if;

            case to_integer(s_sq) is
                when 0 =>
                    s_mul_a <= resize(s_k3, 22); s_mul_b <= s_wl; s_mul_p <= (others => '0'); s_mul_n <= to_unsigned(11, 4);
                    s_sq <= s_sq + 1;
                when 1 =>
                    if s_mul_n = 0 then
                        s_base <= s_mul_p(20 downto 10);
                        s_mul_a <= resize(s_k12, 22); s_mul_b <= s_wl; s_mul_p <= (others => '0'); s_mul_n <= to_unsigned(11, 4);
                        s_sq <= s_sq + 1;
                    end if;
                when 2 =>
                    if s_mul_n = 0 then
                        s_amp <= s_mul_p(20 downto 10);
                        s_sq <= s_sq + 1;
                    end if;
                when 3 =>
                    if s_sin_q2 < 0 then
                        v_neg := -s_sin_q2;
                        v_abs := unsigned(v_neg(7 downto 0));
                        s_sneg <= '1';
                    else
                        v_abs := unsigned(s_sin_q2(7 downto 0));
                        s_sneg <= '0';
                    end if;
                    s_mul_a <= resize(v_abs, 22); s_mul_b <= s_amp; s_mul_p <= (others => '0'); s_mul_n <= to_unsigned(11, 4);
                    s_sq <= s_sq + 1;
                when 4 =>
                    if s_mul_n = 0 then
                        s_swing <= s_mul_p(18 downto 8);
                        s_mul_a <= resize(s_k1, 22); s_mul_b <= '0' & s_k1; s_mul_p <= (others => '0'); s_mul_n <= to_unsigned(11, 4);
                        s_sq <= s_sq + 1;
                    end if;
                when 5 =>
                    if s_sneg = '1' then
                        s_pos <= signed(resize(s_base, 13)) - signed(resize(s_swing, 13));
                    else
                        s_pos <= signed(resize(s_base, 13)) + signed(resize(s_swing, 13));
                    end if;
                    s_sq <= s_sq + 1;
                when 6 =>
                    if s_pos < 0 then
                        s_seam_n <= (others => '0');
                    elsif s_pos > signed(resize(s_wl, 13)) then
                        s_seam_n <= s_wl;
                    else
                        s_seam_n <= unsigned(s_pos(10 downto 0));
                    end if;
                    s_sq <= s_sq + 1;
                when 7 =>
                    -- threshold K1/8 + K1^2/4096: 0..382, fine steps low down
                    if s_mul_n = 0 then
                        s_thr_n <= resize(s_k1(9 downto 3), 12) + resize(s_mul_p(19 downto 12), 12);
                        s_mul_a <= resize(s_k12, 22); s_mul_b <= '0' & s_k12; s_mul_p <= (others => '0'); s_mul_n <= to_unsigned(11, 4);
                        s_sq <= s_sq + 1;
                    end if;
                when 8 =>
                    -- sweep speed 16 + P12/8 + P12^2/2048 per frame (of 65536)
                    if s_mul_n = 0 then
                        s_inc_t <= resize(s_mul_p(19 downto 11), 11) + 16;
                        s_sq <= s_sq + 1;
                    end if;
                when 9 =>
                    s_inc <= s_inc_t + resize(s_k12(9 downto 3), 11);
                    s_sq <= s_sq + 1;
                when 10 =>
                    s_seam_px <= s_seam_n;
                    s_thr     <= s_thr_n;
                    s_xmax    <= s_wl - (C_MARGIN + 1);
                    s_kern    <= s_k2_idx;
                    if s_k2_idx = "10" then s_wide <= '1'; else s_wide <= '0'; end if;
                    s_pshift  <= s_k4_idx;
                    case to_integer(s_k4_idx) is
                        when 0      => s_pane_m1 <= s_wl - 1;
                        when 1      => s_pane_m1 <= ('0' & s_wl(10 downto 1)) - 1;
                        when 2      => s_pane_m1 <= ("00" & s_wl(10 downto 2)) - 1;
                        when others => s_pane_m1 <= ("000" & s_wl(10 downto 3)) - 1;
                    end case;
                    s_plane   <= s_k5_idx;
                    -- 0..255 -> 0..256 so the top of the knob is a full swap
                    s_mix     <= resize(unsigned(registers_in(5)(9 downto 2)), 9) + unsigned(registers_in(5)(9 downto 9));
                    s_sq <= (others => '1');
                when others => null;
            end case;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_saw_active = '1' then
                    s_sweep_ph <= s_sweep_ph + resize(s_inc, 16);
                    s_sin_addr <= s_sweep_ph(15 downto 8);
                    s_k1  <= unsigned(registers_in(0));
                    s_k3  <= unsigned(registers_in(2));
                    s_k12 <= unsigned(registers_in(7));
                    s_wl  <= s_line_width;
                    s_sq  <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;
        end if;
    end process p_frame;

    ----------------------------------------------------------------------------
    -- Block RAMs.  Line buffers: read column x at stage 1, write it two
    -- stages later (RAM 1 the current luma, RAM 2 what RAM 1 returned).
    -- Flags: read at stage 16, write at stage 18.  Ring: write the input at
    -- stage 1, read it back 15 slots later (fabric copy at stage 18).
    ----------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(1) = '1' then
                r_lb1(to_integer(s_x2)) <= std_logic_vector(s_l2);
            end if;
            s_q1 <= r_lb1(to_integer(s_x1));
        end if;
    end process;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(2) = '1' then
                r_lb2(to_integer(s_x3)) <= std_logic_vector(s_c1);
            end if;
            s_q2 <= r_lb2(to_integer(s_x1));
        end if;
    end process;

    p_fl : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(17) = '1' then
                r_fl(to_integer(s_x18)) <= '0' & s_fr18(1 downto 0) & s_f18;
            end if;
            s_fq <= r_fl(to_integer(s_x16));
        end if;
    end process;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rv1 <= "00" & data_in.y & data_in.u & data_in.v;
            r_pic(to_integer(s_pwp)) <= s_rv1;
            s_pq  <= r_pic(to_integer(s_prp));
            s_pq2 <= s_pq;
            s_pwp <= s_pwp + 1;
            s_prp <= s_pwp - 14;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Pixel pipeline
    ----------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a, v_b   : signed(11 downto 0);
        variable v_dd       : signed(18 downto 0);
        variable v_lines, v_draw, v_inv : std_logic;
        variable v_ya, v_yb : unsigned(9 downto 0);
        variable v_y        : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_dy, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: luma (black outside the active line), column
            if data_in.avid = '1' then
                s_l1 <= unsigned(data_in.y(9 downto 2));
            else
                s_l1 <= to_unsigned(16, 8);
            end if;
            s_x1 <= s_rx;

            -- stage 2: line-buffer reads land; strips; border
            s_l2 <= s_l1;
            s_x2 <= s_x1;
            if s_avid_sr(0) = '0' then
                s_pc <= (others => '0'); s_pi <= (others => '0');
            elsif s_pc >= s_pane_m1 then
                s_pc <= (others => '0'); s_pi <= s_pi + 1;
            else
                s_pc <= s_pc + 1;
            end if;
            if s_x1 < C_MARGIN or s_x1 > s_xmax then s_bd(2) <= '1'; else s_bd(2) <= '0'; end if;

            -- stage 3: the three rows of column x; processed side (right of
            -- the seam; in strip modes the seam flips the strip parity)
            s_c0 <= s_l2;
            if s_avid_sr(1) = '1' then
                s_c1 <= unsigned(s_q1); s_c2 <= unsigned(s_q2);
            else
                s_c1 <= to_unsigned(16, 8); s_c2 <= to_unsigned(16, 8);
            end if;
            s_x3 <= s_x2;
            if s_pshift = 0 then
                if s_x2 >= s_seam_px then s_pr(3) <= '1'; else s_pr(3) <= '0'; end if;
            else
                if s_x2 >= s_seam_px then s_pr(3) <= not s_pi(0); else s_pr(3) <= s_pi(0); end if;
            end if;

            -- stage 4: column terms.  V = [1 2 1] down the column, D = rows
            -- r - (r-2), L = vertical Laplacian
            s_v4  <= resize(s_c0, 10) + shift_left(resize(s_c1, 10), 1) + resize(s_c2, 10);
            s_d4  <= signed('0' & s_c0) - signed('0' & s_c2);
            s_lp4 <= signed(shift_left(resize(s_c1, 10), 1)) - signed(resize(s_c0, 10)) - signed(resize(s_c2, 10));
            s_r0_4 <= s_c0; s_r1_4 <= s_c1;

            -- stage 5: taps and running sums (exact by induction from the
            -- all-zero power-on state).  acc17 = V taps 0..16, acc5 = 6..10,
            -- accd = D taps 4..11
            s_vt(0) <= s_v4; s_dt(0) <= s_d4; s_lt(0) <= s_lp4;
            s_r0t(0) <= s_r0_4; s_r1t(0) <= s_r1_4;
            for i in 1 to 16 loop s_vt(i) <= s_vt(i - 1); end loop;
            for i in 1 to 11 loop s_dt(i) <= s_dt(i - 1); end loop;
            for i in 1 to 9 loop
                s_lt(i) <= s_lt(i - 1); s_r0t(i) <= s_r0t(i - 1); s_r1t(i) <= s_r1t(i - 1);
            end loop;
            s_acc17 <= s_acc17 + resize(s_v4, 15) - resize(s_vt(16), 15);
            s_acc5  <= s_acc5 + resize(s_vt(5), 13) - resize(s_vt(10), 13);
            s_accd  <= s_accd + resize(s_dt(3), 12) - resize(s_dt(11), 12);

            -- stage 14: kernel terms around tap 8 (lower taps are to the
            -- right).  Each kernel gives ~2H for a step of height H.
            s_rbx <= signed('0' & s_r1t(9)) - signed('0' & s_r0t(8));
            s_rby <= signed('0' & s_r1t(8)) - signed('0' & s_r0t(9));
            s_sbx <= signed('0' & s_vt(9)) - signed('0' & s_vt(7));
            s_sby <= resize(s_dt(7), 12) + shift_left(resize(s_dt(8), 12), 1) + resize(s_dt(9), 12);
            s_wdx <= signed('0' & s_vt(12)) - signed('0' & s_vt(4));
            s_wdy <= s_accd;
            s_p5  <= shift_left(resize(s_acc5, 18), 4) + resize(s_acc5, 18);
            s_p17 <= shift_left(resize(s_acc17, 18), 2) + resize(s_acc17, 18);
            s_lpv <= resize(s_lt(7), 12) + shift_left(resize(s_lt(8), 12), 1) + resize(s_lt(9), 12);

            -- stage 15: the kernel's gradient pair
            case to_integer(s_kern) is
                when 0 =>
                    v_a := resize(s_rbx, 12); v_b := resize(s_rby, 12);
                when 1 =>
                    v_a := resize(shift_right(s_sbx, 1), 12); v_b := shift_right(s_sby, 1);
                when 2 =>
                    v_a := resize(shift_right(s_wdx, 1), 12); v_b := shift_right(s_wdy, 2);
                when others =>
                    -- mean5 - mean17 = (17 acc5 - 5 acc17) / 85, taken /64
                    v_dd := signed('0' & s_p5) - signed('0' & s_p17);
                    v_a := resize(shift_right(v_dd, 6), 12); v_b := shift_right(s_lpv, 1);
            end case;
            s_ga <= v_a; s_gb <= v_b;

            -- stage 16: magnitudes
            s_aa <= unsigned(abs(s_ga)); s_ab <= unsigned(abs(s_gb));

            -- stage 17: |gx| + |gy|
            s_mag <= s_aa + s_ab;

            -- stage 18: edge flag (none in the border or a field's top rows);
            -- the Wide flags of rows r-1..r-3 land
            if s_mag > s_thr and s_bd(17) = '0' and s_rowok = '1' then s_f18 <= '1'; else s_f18 <= '0'; end if;
            s_fr18 <= s_fq;
            if s_avid_sr(14) = '1' then s_rxl <= s_rxl + 1; else s_rxl <= (others => '0'); end if;
            s_x16 <= s_rxl;
            s_x17 <= s_x16;
            s_x18 <= s_x17;

            -- stage 19: final edge (Wide: OR of four rows); picture from the
            -- ring; inversion target and plane key
            s_e19 <= s_f18 or (s_wide and (s_fr18(0) or s_fr18(1) or s_fr18(2)));
            s_py19 <= unsigned(s_pq2(29 downto 20));
            s_pu19 <= unsigned(s_pq2(19 downto 10));
            s_pv19 <= unsigned(s_pq2(9 downto 0));
            if s_plane = 0 then
                s_key19 <= '1';
                if unsigned(s_pq2(29 downto 20)) > 940 then
                    s_t19 <= to_unsigned(64, 10);
                else
                    s_t19 <= to_unsigned(1004, 10) - unsigned(s_pq2(29 downto 20));
                end if;
            else
                s_key19 <= s_pq2(29 - to_integer(s_plane));
                if s_pq2(29 - to_integer(s_plane)) = '1' then
                    s_t19 <= to_unsigned(940, 10);
                else
                    s_t19 <= to_unsigned(64, 10);
                end if;
            end if;

            -- stage 20: compose.  a = the pixel, b = its inverted value.
            -- Area on the line drawing: the plane key flips lines and
            -- ground (negative = black lines on white); Area elsewhere: the
            -- picture inverts under white lines; Edges Only: edge pixels
            -- (lines included) take the inverted picture
            v_lines := s_e19 and (s_pr(19) or s_sw_both);
            v_draw  := s_pr(19) and not s_sw_over;
            v_inv   := s_pr(19) xor s_sw_otherinv;
            if v_lines = '1' then
                v_ya := to_unsigned(940, 10);
            elsif v_draw = '1' then
                v_ya := to_unsigned(64, 10);
            else
                v_ya := s_py19;
            end if;
            if v_inv = '0' then
                v_yb := v_ya;
            elsif s_sw_invedges = '1' then
                if s_e19 = '1' then v_yb := s_t19; else v_yb := v_ya; end if;
            elsif v_draw = '1' then
                if (s_key19 xor v_lines) = '1' then v_yb := to_unsigned(940, 10); else v_yb := to_unsigned(64, 10); end if;
            elsif v_lines = '1' then
                v_yb := to_unsigned(940, 10);
            else
                v_yb := s_t19;
            end if;
            s_a20 <= v_ya; s_b20 <= v_yb;
            if (v_lines = '1' and s_sw_colour = '0') or (v_lines = '0' and v_draw = '1') then
                s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            else
                s_u20 <= s_pu19; s_v20 <= s_pv19;
            end if;

            -- stage 21: b - a
            s_d21 <= signed('0' & s_b20) - signed('0' & s_a20);
            s_a21 <= s_a20; s_u21 <= s_u20; s_v21 <= s_v20;

            -- stage 22: mix product
            s_p22 <= s_d21 * signed('0' & s_mix);
            s_a22 <= s_a21; s_u22 <= s_u21; s_v22 <= s_v21;

            -- stage 23: a + mix (b - a), always between a and b; neutral
            -- outside the active line (the smoothing's edges)
            v_y := signed(resize(s_a22, 12)) + resize(shift_right(s_p22, 8), 12);
            if s_avid_sr(21) = '1' then
                s_y23 <= unsigned(v_y(9 downto 0)); s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;

            -- stages 24..25: horizontal [1 2 1]/4 on Y, U and V (g1 the
            -- centre) -- with the luma scale below, the HD HDMI bar fix
            s_g1y <= s_y23;  s_g2y <= s_g1y;
            s_g1u <= s_u23;  s_g2u <= s_g1u;
            s_g1v <= s_v23;  s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y23, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u23, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v23, 12);
            s_y25 <= v_sy(11 downto 2);
            s_u25 <= v_su(11 downto 2);
            s_v25 <= v_sv(11 downto 2);

            -- stage 26: luma 64 + 13/16 (y - 64) (white 940 -> ~776, under the
            -- drawn-luma cap); sub-black to black; blanking gate
            v_dy := s_y25 - 64;
            v_yo := s_y25 - shift_right(v_dy, 3) - shift_right(v_dy, 4);
            if s_avid_sr(24) = '0' then
                s_y26 <= to_unsigned(64, 10); s_u26 <= to_unsigned(512, 10); s_v26 <= to_unsigned(512, 10);
            else
                if s_y25(9 downto 6) = "0000" then s_y26 <= to_unsigned(64, 10); else s_y26 <= v_yo; end if;
                s_u26 <= s_u25; s_v26 <= s_v25;
            end if;

            -- 1-bit pipes
            for i in 3 to 17 loop s_bd(i) <= s_bd(i - 1); end loop;
            for i in 4 to 19 loop s_pr(i) <= s_pr(i - 1); end loop;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y26);
    s_io_align_in.u       <= std_logic_vector(s_u26);
    s_io_align_in.v       <= std_logic_vector(s_v26);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (26 -> 28).
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

end architecture outliner;
