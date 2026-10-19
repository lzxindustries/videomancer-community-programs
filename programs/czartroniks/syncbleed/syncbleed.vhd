-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Syncbleed: catalogue combo #131 (seed 20260927) -- colour bleeding / NTSC
-- dot-crawl emulation + sync loss / horizontal collapse.  Three knobs
-- each; the switches and the slider decide how they combine.
--
--   K1 Bleed    chroma smear length (IIR: off, 2 .. 128 px; Local / P12
--               push it to 256 px)
--   K2 Lag      chroma lags luma by 0 .. 63 px
--   K3 Crawl    dot-crawl amount: subcarrier dots (188 cycles a line), phased
--               by hue, strongest on colour, a faint floor on grey
--   K4 Collapse base squeeze of every row toward the seam, 1x .. 9x
--   K5 Seam     collapse centre X, bipolar (centre = screen centre)
--   K6 Wave     the squeeze breathes down the picture with a slow rolling wave
--
--   S7  Bleed   before the collapse (smear squeezed too) / after (screen-width smear)
--   S8  Length  fixed / follows the squeeze, two steps an octave (squeezed
--               rows smear longer, stretched rows sharpen)
--   S9  Chroma  collapses with luma / collapses more (colour fringes)
--   S10 Crawl   locked dots / crawling (the subcarrier slips 90 degrees a field)
--   S11 Smear   chroma only / luma too (ghosting)
--   P12 Loss    signal loss: the squeeze multiplies x1 .. x64 (exponential),
--               bleed and crawl climb with it; 100% collapses rows to a few
--               pixels
--
-- v0.2.0.  Per field: glided K4 / K6 / P12 (1/4 a field), seam = K5 * width,
-- serial restoring divider for the dot pitch (188 cycles across the measured
-- width) and the wave line step (1.58 cycles a screen in every mode).  Per
-- line: q = (base + sine wave) * 2^(6 * loss) (8.8 fixed point, capped at
-- 512x), chroma q 1.5x for Fringe, bleed shift = K1 + P12 (+ 2 per octave of
-- q for Local).  Luma and chroma run their own DDA read addresses (chroma
-- with its own gain and a lag), both edge-filled; chroma IIR either on the
-- write port or after the fetch.  Crawl = (U+64) sin + V cos of the
-- subcarrier phase (180 degrees a line pair at HD, a line at SD), times the
-- crawl gain, added to luma.  Line buffer read a line late; U/V swapped on
-- rows whose hsync->avid parity differs from the stored row's (drape v1.3).
-- Output: luma clamp 64..940, highlight knee (600 + (y - 600)/2, whites
-- ~770), [1 2 1] on Y/U/V (the HD HDMI bar fix).  Latency 15 clocks, all
-- modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture syncbleed of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (15 + 1 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 15;
    constant C_EDGE : integer := 12;

    -- Edge clamp: reads off the line or inside its C_EDGE blanking columns take
    -- the line's own edge pixel.  Both compares run on the source in parallel.
    function f_edge_addr(v : signed(22 downto 0); we : signed(22 downto 0); we11 : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > we then
            return we11;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

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

    -- dot-crawl subcarrier: sin (high byte) and cos (low byte) * 127
    type t_sc_rom is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_SC : t_sc_rom := (
        x"007F", x"037F", x"067F", x"097F", x"0C7E", x"107E", x"137E", x"167D",
        x"197D", x"1C7C", x"1F7B", x"227A", x"257A", x"2879", x"2B78", x"2E76",
        x"3175", x"3374", x"3673", x"3971", x"3C70", x"3F6F", x"416D", x"446B",
        x"476A", x"4968", x"4C66", x"4E64", x"5162", x"5360", x"555E", x"585C",
        x"5A5A", x"5C58", x"5E55", x"6053", x"6251", x"644E", x"664C", x"6849",
        x"6A47", x"6B44", x"6D41", x"6F3F", x"703C", x"7139", x"7336", x"7433",
        x"7531", x"762E", x"782B", x"7928", x"7A25", x"7A22", x"7B1F", x"7C1C",
        x"7D19", x"7D16", x"7E13", x"7E10", x"7E0C", x"7F09", x"7F06", x"7F03",
        x"7F00", x"7FFD", x"7FFA", x"7FF7", x"7EF4", x"7EF0", x"7EED", x"7DEA",
        x"7DE7", x"7CE4", x"7BE1", x"7ADE", x"7ADB", x"79D8", x"78D5", x"76D2",
        x"75CF", x"74CD", x"73CA", x"71C7", x"70C4", x"6FC1", x"6DBF", x"6BBC",
        x"6AB9", x"68B7", x"66B4", x"64B2", x"62AF", x"60AD", x"5EAB", x"5CA8",
        x"5AA6", x"58A4", x"55A2", x"53A0", x"519E", x"4E9C", x"4C9A", x"4998",
        x"4796", x"4495", x"4193", x"3F91", x"3C90", x"398F", x"368D", x"338C",
        x"318B", x"2E8A", x"2B88", x"2887", x"2586", x"2286", x"1F85", x"1C84",
        x"1983", x"1683", x"1382", x"1082", x"0C82", x"0981", x"0681", x"0381",
        x"0081", x"FD81", x"FA81", x"F781", x"F482", x"F082", x"ED82", x"EA83",
        x"E783", x"E484", x"E185", x"DE86", x"DB86", x"D887", x"D588", x"D28A",
        x"CF8B", x"CD8C", x"CA8D", x"C78F", x"C490", x"C191", x"BF93", x"BC95",
        x"B996", x"B798", x"B49A", x"B29C", x"AF9E", x"ADA0", x"ABA2", x"A8A4",
        x"A6A6", x"A4A8", x"A2AB", x"A0AD", x"9EAF", x"9CB2", x"9AB4", x"98B7",
        x"96B9", x"95BC", x"93BF", x"91C1", x"90C4", x"8FC7", x"8DCA", x"8CCD",
        x"8BCF", x"8AD2", x"88D5", x"87D8", x"86DB", x"86DE", x"85E1", x"84E4",
        x"83E7", x"83EA", x"82ED", x"82F0", x"82F4", x"81F7", x"81FA", x"81FD",
        x"8100", x"8103", x"8106", x"8109", x"820C", x"8210", x"8213", x"8316",
        x"8319", x"841C", x"851F", x"8622", x"8625", x"8728", x"882B", x"8A2E",
        x"8B31", x"8C33", x"8D36", x"8F39", x"903C", x"913F", x"9341", x"9544",
        x"9647", x"9849", x"9A4C", x"9C4E", x"9E51", x"A053", x"A255", x"A458",
        x"A65A", x"A85C", x"AB5E", x"AD60", x"AF62", x"B264", x"B466", x"B768",
        x"B96A", x"BC6B", x"BF6D", x"C16F", x"C470", x"C771", x"CA73", x"CD74",
        x"CF75", x"D276", x"D578", x"D879", x"DB7A", x"DE7A", x"E17B", x"E47C",
        x"E77D", x"EA7D", x"ED7E", x"F07E", x"F47E", x"F77F", x"FA7F", x"FD7F"
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
    signal s_k_bleed, s_k_lag, s_k_crawl, s_k_col, s_k_seam, s_k_wave, s_p_loss : unsigned(9 downto 0) := (others => '0');
    signal s_sw_after, s_sw_local, s_sw_fringe, s_sw_drift, s_sw_lumasmear : std_logic := '0';

    -- per field
    signal s_vstep  : unsigned(4 downto 0) := (others => '1');
    signal s_hf     : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wf     : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_hd     : std_logic := '0';
    signal s_colg, s_lossg, s_waveg : unsigned(9 downto 0) := (others => '0');
    signal s_gdc, s_gdl, s_gdw : signed(10 downto 0) := (others => '0');
    signal s_lag    : unsigned(5 downto 0) := (others => '0');
    signal s_bn     : unsigned(2 downto 0) := (others => '0');
    signal s_bnl    : unsigned(1 downto 0) := (others => '0');
    signal s_cgp    : unsigned(8 downto 0) := (others => '0');
    signal s_cg     : unsigned(7 downto 0) := (others => '0');
    signal s_seam_p : unsigned(20 downto 0) := (others => '0');
    signal s_seam   : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_seam256 : unsigned(18 downto 0) := (others => '0');
    signal s_lag256 : unsigned(13 downto 0) := (others => '0');
    signal s_qb     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_le     : unsigned(12 downto 0) := (others => '0');
    signal s_lei    : unsigned(2 downto 0) := (others => '0');
    signal s_lm     : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_fphase, s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_linc   : unsigned(15 downto 0) := to_unsigned(96, 16);
    signal s_cstep  : unsigned(15 downto 0) := to_unsigned(6417, 16);
    signal s_cf     : unsigned(1 downto 0) := (others => '0');
    signal s_fpar, s_ilace : std_logic := '0';
    signal s_saw_active : std_logic := '0';
    -- serial restoring divider (24-bit numerator, 11-bit divisor)
    signal s_dnum, s_dquo : unsigned(23 downto 0) := (others => '0');
    signal s_dden   : unsigned(10 downto 0) := to_unsigned(64, 11);
    signal s_drem   : unsigned(11 downto 0) := (others => '0');
    signal s_dcnt   : unsigned(4 downto 0) := (others => '0');

    -- per line (one op per state)
    signal s_lstep  : unsigned(4 downto 0) := (others => '1');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_jp     : signed(20 downto 0) := (others => '0');
    signal s_q0     : unsigned(12 downto 0) := to_unsigned(256, 13);
    signal s_qm     : unsigned(20 downto 0) := (others => '0');
    signal s_q19    : unsigned(18 downto 0) := (others => '0');
    signal s_q      : unsigned(16 downto 0) := to_unsigned(256, 17);
    signal s_qc     : unsigned(17 downto 0) := to_unsigned(256, 18);
    signal s_lv     : signed(4 downto 0) := (others => '0');
    signal s_bn_by  : std_logic := '1';
    signal s_bn_sh  : unsigned(2 downto 0) := (others => '0');
    -- free-running split products (seam x q hi/lo, seam x qc hi/lo) with kept seam copies
    signal s_seam_a, s_seam_b, s_seam_c, s_seam_d : unsigned(10 downto 0) := to_unsigned(240, 11);
    attribute keep : boolean;
    attribute keep of s_seam_a, s_seam_b, s_seam_c, s_seam_d : signal is true;
    signal s_pqh    : unsigned(19 downto 0) := (others => '0');
    signal s_pql    : unsigned(18 downto 0) := (others => '0');
    signal s_pqch, s_pqcl : unsigned(19 downto 0) := (others => '0');
    signal s_pq     : unsigned(27 downto 0) := (others => '0');
    signal s_pqc    : unsigned(28 downto 0) := (others => '0');
    signal s_tq, s_tqc : signed(30 downto 0) := (others => '0');
    signal s_src0, s_srcc0 : signed(30 downto 0) := (others => '0');

    -- tracking
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_hpar, s_line_par, s_cap_par, s_swap : std_logic := '0';
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_we23   : signed(22 downto 0) := to_signed(200, 23);
    signal s_clc    : unsigned(1 downto 0) := (others => '0');

    -- write path (pre-bleed IIR, Q6)
    signal s_py, s_pu, s_pv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_wx2    : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_wp2 : std_logic := '0';

    -- address chain
    type t_b is array (1 to 4) of std_logic;
    signal s_avd    : t_b := (others => '0');
    signal s_accy, s_accc : signed(30 downto 0) := (others => '0');
    signal s_srcy4, s_srcc4 : signed(22 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal s_a_rbank, s_a_swap : std_logic := '0';
    signal s_wet_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_qy, s_qu, s_qv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_y9, s_y10, s_y11, s_y12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u9, s_v9, s_u10, s_v10, s_u11, s_v11, s_u12, s_v12, s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_du9, s_dv9 : signed(10 downto 0) := (others => '0');
    -- subcarrier
    signal s_cph    : unsigned(15 downto 0) := (others => '0');
    signal s_coff   : unsigned(1 downto 0) := (others => '0');
    signal s_caddr  : unsigned(7 downto 0) := (others => '0');
    signal s_crom   : std_logic_vector(15 downto 0) := x"007F";
    signal s_sk9    : std_logic_vector(15 downto 0) := x"007F";
    signal s_pu10, s_pv10 : signed(18 downto 0) := (others => '0');
    signal s_c11    : signed(10 downto 0) := (others => '0');
    signal s_pc12   : signed(19 downto 0) := (others => '0');
    signal s_v13y   : signed(12 downto 0) := to_signed(64, 13);
    signal s_y14    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ya, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ua, s_ub, s_va, s_vb : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            s_crom <= C_SC(to_integer(s_caddr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control, per-frame / per-line sequencers, write path
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_q : signed(15 downto 0);
        variable v_n : signed(5 downto 0);
        variable v_g : signed(10 downto 0);
        variable v_r : unsigned(11 downto 0);
        variable v_s : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_k_bleed <= unsigned(registers_in(0));
            s_k_lag   <= unsigned(registers_in(1));
            s_k_crawl <= unsigned(registers_in(2));
            s_k_col   <= unsigned(registers_in(3));
            s_k_seam  <= unsigned(registers_in(4));
            s_k_wave  <= unsigned(registers_in(5));
            s_sw_after     <= registers_in(6)(0);
            s_sw_local     <= registers_in(6)(1);
            s_sw_fringe    <= registers_in(6)(2);
            s_sw_drift     <= registers_in(6)(3);
            s_sw_lumasmear <= registers_in(6)(4);
            s_p_loss  <= unsigned(registers_in(7));

            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- chroma phase: hsync->avid parity of this line vs the stored line
            s_hpar <= not s_hpar;
            if data_in.avid = '1' and s_prev_avid = '0' then s_line_par <= s_hpar; end if;
            s_swap <= s_line_par xor s_cap_par;

            -- stage 2: pre-bleed IIR on the write port (stage-1 inputs);
            -- shift 1..8 = 1 + s_bn_sh, bypassed when the row's bleed is off
            v_s := to_integer(s_bn_sh);
            if s_in_avid = '0' or s_sw_after = '1' or s_bn_by = '1' then
                s_py <= s_in_y & "000000"; s_pu <= s_in_u & "000000"; s_pv <= s_in_v & "000000";
            else
                s_pu <= s_pu - shift_right(shift_right(s_pu, 1), v_s) + shift_right(shift_right(s_in_u & "000000", 1), v_s);
                s_pv <= s_pv - shift_right(shift_right(s_pv, 1), v_s) + shift_right(shift_right(s_in_v & "000000", 1), v_s);
                if s_sw_lumasmear = '1' then
                    s_py <= s_py - shift_right(shift_right(s_py, 1), v_s) + shift_right(shift_right(s_in_y & "000000", 1), v_s);
                else
                    s_py <= s_in_y & "000000";
                end if;
            end if;
            s_wx2 <= s_wr_x; s_we2 <= s_in_avid; s_wp2 <= s_wpar;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we23 <= signed(resize(s_w_edge, 23));

            ---------------------------------------------------------------
            -- per field (one op per state)
            ---------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    if s_height < 64 then s_hf <= to_unsigned(64, 11); else s_hf <= s_height; end if;
                    if s_line_width < 64 then s_wf <= to_unsigned(64, 11); else s_wf <= s_line_width; end if;
                    s_gdc <= signed(resize(s_k_col, 11)) - signed(resize(s_colg, 11));
                    s_gdl <= signed(resize(s_p_loss, 11)) - signed(resize(s_lossg, 11));
                    s_gdw <= signed(resize(s_k_wave, 11)) - signed(resize(s_waveg, 11));
                    s_lag <= s_k_lag(9 downto 4);
                    s_bn  <= s_k_bleed(9 downto 7);
                    s_bnl <= s_p_loss(9 downto 8);
                    s_cgp <= resize(s_k_crawl(9 downto 2), 9) + resize(s_p_loss(9 downto 3), 9);
                    s_fphase <= s_fphase + 300;
                    -- Drift: the subcarrier slips 270 degrees a field (NTSC)
                    if s_sw_drift = '1' then s_cf <= s_cf + 3; end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- glide K4 / P12 / K6 a quarter of the way each field
                    v_g := signed(resize(s_colg, 11)) + shift_right(s_gdc, 2);
                    s_colg <= unsigned(v_g(9 downto 0));
                    v_g := signed(resize(s_lossg, 11)) + shift_right(s_gdl, 2);
                    s_lossg <= unsigned(v_g(9 downto 0));
                    v_g := signed(resize(s_waveg, 11)) + shift_right(s_gdw, 2);
                    s_waveg <= unsigned(v_g(9 downto 0));
                    if s_cgp > 255 then s_cg <= (others => '1'); else s_cg <= s_cgp(7 downto 0); end if;
                    if s_hf >= 600 then s_hd <= '1'; else s_hd <= '0'; end if;
                    s_seam_p <= s_k_seam * s_wf;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_seam <= s_seam_p(20 downto 10);
                    s_qb <= to_unsigned(256, 12) + resize(s_colg & '0', 12);
                    -- loss exponent: 6 octaves over the slider
                    s_le <= shift_left(resize(s_lossg, 13), 2) + shift_left(resize(s_lossg, 13), 1);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_seam256 <= s_seam & x"00";
                    s_lag256 <= s_lag & x"00";
                    s_lei <= s_le(12 downto 10);
                    s_lm <= '1' & s_le(9 downto 3);          -- 1 + frac (Q7), piecewise 2^frac
                    -- dot pitch: 188 subcarrier cycles across the active line
                    s_dnum <= to_unsigned(12320768, 24); s_drem <= (others => '0'); s_dden <= s_wf;
                    s_dcnt <= to_unsigned(23, 5);
                    s_vstep <= s_vstep + 1;
                when 4 | 6 =>
                    v_r := s_drem(10 downto 0) & s_dnum(23);
                    s_dnum <= shift_left(s_dnum, 1);
                    if v_r >= resize(s_dden, 12) then
                        s_drem <= v_r - resize(s_dden, 12); s_dquo <= s_dquo(22 downto 0) & '1';
                    else
                        s_drem <= v_r; s_dquo <= s_dquo(22 downto 0) & '0';
                    end if;
                    s_dcnt <= s_dcnt - 1;
                    if s_dcnt = 0 then s_vstep <= s_vstep + 1; end if;
                when 5 =>
                    if s_dquo(23 downto 16) /= 0 then s_cstep <= (others => '1'); else s_cstep <= s_dquo(15 downto 0); end if;
                    -- wave line step = 103680 / height (1.58 cycles a screen, v0.1's HD rate)
                    s_dnum <= to_unsigned(103680, 24); s_drem <= (others => '0'); s_dden <= s_hf;
                    s_dcnt <= to_unsigned(23, 5);
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_linc <= s_dquo(15 downto 0);
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- free-running split products (operands registered per line / per frame)
            s_seam_a <= s_seam; s_seam_b <= s_seam; s_seam_c <= s_seam; s_seam_d <= s_seam;
            s_pqh  <= s_seam_a * s_q(16 downto 8);
            s_pql  <= s_seam_b * s_q(7 downto 0);
            s_pqch <= s_seam_c * s_qc(17 downto 9);
            s_pqcl <= s_seam_d * s_qc(8 downto 0);

            ---------------------------------------------------------------
            -- per line, from hsync: wave, squeeze factors, bleed, DDA starts
            ---------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    s_sin_addr <= s_line_phase(15 downto 8);
                    s_lstep <= s_lstep + 1;
                when 1 | 2 =>
                    s_lstep <= s_lstep + 1;                                   -- ROM latency
                when 3 =>
                    s_jp <= s_sin_q2 * signed('0' & s_waveg);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    v_q := signed(resize(s_qb, 16)) + resize(s_jp(20 downto 7), 16);
                    if v_q < 64 then v_q := to_signed(64, 16); end if;
                    s_q0 <= unsigned(v_q(12 downto 0));
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_qm <= s_q0 * s_lm;                                      -- x (1 + frac)
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_q19 <= shift_left(resize(s_qm(20 downto 7), 19), to_integer(s_lei));
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    if s_q19(18 downto 17) /= "00" then s_q <= (others => '1'); else s_q <= s_q19(16 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    if s_sw_fringe = '1' then s_qc <= resize(s_q, 18) + resize(s_q(16 downto 1), 18); else s_qc <= resize(s_q, 18); end if;
                    -- Local: two bleed steps per octave of squeeze about 1x
                    if s_q(16 downto 12) /= 0 then s_lv <= to_signed(8, 5);
                    elsif s_q(11) = '1' then s_lv <= to_signed(6, 5);
                    elsif s_q(10) = '1' then s_lv <= to_signed(4, 5);
                    elsif s_q(9) = '1' then s_lv <= to_signed(2, 5);
                    elsif s_q(8) = '1' then s_lv <= to_signed(0, 5);
                    elsif s_q(7) = '1' then s_lv <= to_signed(-2, 5);
                    else s_lv <= to_signed(-4, 5);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    v_n := signed(resize(s_bn, 6)) + signed(resize(s_bnl, 6));
                    if s_sw_local = '1' then v_n := v_n + resize(s_lv, 6); end if;
                    if v_n > 8 then v_n := to_signed(8, 6); elsif v_n < 0 then v_n := to_signed(0, 6); end if;
                    if v_n = 0 then
                        s_bn_by <= '1'; s_bn_sh <= (others => '0');
                    else
                        s_bn_by <= '0';
                        v_n := v_n - 1;
                        s_bn_sh <= unsigned(v_n(2 downto 0));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_lstep <= s_lstep + 1;                                   -- split products settle
                when 11 =>
                    s_pq  <= shift_left(resize(s_pqh, 28), 8) + resize(s_pql, 28);
                    s_pqc <= shift_left(resize(s_pqch, 29), 9) + resize(s_pqcl, 29);
                    s_tq  <= signed(resize(s_seam256, 31)) - signed(resize(s_q, 31));
                    s_tqc <= signed(resize(s_seam256, 31)) - signed(resize(s_lag256, 31));
                    s_lstep <= s_lstep + 1;
                when 12 =>
                    s_src0 <= s_tq - signed(resize(s_pq, 31));
                    s_tqc  <= s_tqc - signed(resize(s_qc, 31));
                    s_lstep <= s_lstep + 1;
                when 13 =>
                    s_srcc0 <= s_tqc - signed(resize(s_pqc, 31));
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            -- dot-crawl line phase: 180 degrees a line (SD) / a line pair
            -- (HD), counted on active lines at the fetch stage
            if s_avid_sr(6) = '0' and s_avid_sr(7) = '1' then s_clc <= s_clc + 1; end if;

            -- line end: count lines; the wave phase steps on active lines only
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                s_line_phase <= s_line_phase + s_linc;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_cap_par <= s_line_par;   -- parity of the line now in the read bank
                s_hpar <= '0';
                -- before the first active line: seed the wave phase (idempotent,
                -- the last blanking hsync wins); the bottom field starts half a
                -- step down
                if s_saw_active = '0' then
                    if s_ilace = '1' and data_in.field_n = '0' then
                        s_line_phase <= s_fphase + shift_right(s_linc, 1);
                    else
                        s_line_phase <= s_fphase;
                    end if;
                end if;
            end if;
            -- resets are idempotent (serrated vsync); the field sequencer and
            -- the interlace sample are guarded by saw-active
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_clc <= (others => '0');
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

    --------------------------------------------------------------------------
    -- Address chain: two DDAs (luma / chroma), edge fill (stages 2..5)
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_avd(1) <= s_in_avid;
            for i in 2 to 4 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: DDA
            if s_avd(1) = '0' then
                s_accy <= s_src0; s_accc <= s_srcc0;
            else
                s_accy <= s_accy + signed(resize(s_q, 31));
                s_accc <= s_accc + signed(resize(s_qc, 31));
            end if;
            -- 4: integer source x
            s_srcy4 <= s_accy(30 downto 8);
            s_srcc4 <= s_accc(30 downto 8);
            -- 5: edge fill
            s_addrY <= f_edge_addr(s_srcy4, s_we23, s_w_edge);
            s_addrC <= f_edge_addr(s_srcc4, s_we23, s_w_edge);
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 2, read stage 5 -> data 6)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_we2 = '1' and s_wp2 = '0' then lbY0(to_integer(s_wx2)) <= std_logic_vector(s_py(15 downto 6)); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_we2 = '1' and s_wp2 = '1' then lbY1(to_integer(s_wx2)) <= std_logic_vector(s_py(15 downto 6)); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '0' then lbU0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pu(15 downto 8)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '1' then lbU1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pu(15 downto 8)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '0' then lbV0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pv(15 downto 8)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '1' then lbV1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pv(15 downto 8)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path: 7 fetch + U/V swap, 8 post bleed, 9 chroma differences,
    -- 10 x subcarrier, 11 sum, 12 x crawl gain, 13 add, 14 clamp + knee +
    -- neutral gate, 15 [1 2 1] + gate.  Subcarrier: 6 phase, 7 address,
    -- 8 ROM, 9 fabric register.
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s : integer range 0 to 7;
        variable v_ry : std_logic_vector(9 downto 0);
        variable v_ru, v_rv : std_logic_vector(7 downto 0);
        variable v_c : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- subcarrier phase per pixel, restarting each line; the offset
            -- adds 180 degrees per line (pair) and the field slip
            if s_avid_sr(4) = '0' then s_cph <= (others => '0'); else s_cph <= s_cph + s_cstep; end if;
            if s_hd = '1' then
                s_coff <= unsigned'(s_clc(1) & '0') + s_cf;
            else
                s_coff <= unsigned'(s_clc(0) & '0') + s_cf;
            end if;
            s_caddr <= (s_cph(15 downto 14) + s_coff) & s_cph(13 downto 8);
            s_sk9 <= s_crom;

            -- 6 (rd) / 7: fetch, U/V swap where this row's parity differs
            s_a_rbank <= s_rbank; s_a_swap <= s_swap;
            if s_a_rbank = '1' then
                v_ry := s_rdY1; v_ru := s_rdU1; v_rv := s_rdV1;
            else
                v_ry := s_rdY0; v_ru := s_rdU0; v_rv := s_rdV0;
            end if;
            s_wet_y <= unsigned(v_ry);
            if s_a_swap = '1' then
                s_wet_u <= unsigned(v_rv) & "00"; s_wet_v <= unsigned(v_ru) & "00";
            else
                s_wet_u <= unsigned(v_ru) & "00"; s_wet_v <= unsigned(v_rv) & "00";
            end if;

            -- 8: post bleed IIR (Q6)
            v_s := to_integer(s_bn_sh);
            if s_avid_sr(6) = '0' or s_sw_after = '0' or s_bn_by = '1' then
                s_qy <= s_wet_y & "000000"; s_qu <= s_wet_u & "000000"; s_qv <= s_wet_v & "000000";
            else
                s_qu <= s_qu - shift_right(shift_right(s_qu, 1), v_s) + shift_right(shift_right(s_wet_u & "000000", 1), v_s);
                s_qv <= s_qv - shift_right(shift_right(s_qv, 1), v_s) + shift_right(shift_right(s_wet_v & "000000", 1), v_s);
                if s_sw_lumasmear = '1' then
                    s_qy <= s_qy - shift_right(shift_right(s_qy, 1), v_s) + shift_right(shift_right(s_wet_y & "000000", 1), v_s);
                else
                    s_qy <= s_wet_y & "000000";
                end if;
            end if;

            -- 9: chroma differences (U carries the +64 floor carrier)
            s_y9 <= s_qy(15 downto 6); s_u9 <= s_qu(15 downto 6); s_v9 <= s_qv(15 downto 6);
            s_du9 <= signed('0' & s_qu(15 downto 6)) - to_signed(448, 11);
            s_dv9 <= signed('0' & s_qv(15 downto 6)) - to_signed(512, 11);

            -- 10: chroma onto the subcarrier
            s_pu10 <= s_du9 * signed(s_sk9(15 downto 8));
            s_pv10 <= s_dv9 * signed(s_sk9(7 downto 0));
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;

            -- 11: composite carrier (about +-|chroma| + 64)
            v_c := resize(s_pu10, 20) + resize(s_pv10, 20);
            s_c11 <= v_c(18 downto 8);
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;

            -- 12: crawl amplitude
            s_pc12 <= s_c11 * signed('0' & s_cg);
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;

            -- 13: dots into luma
            s_v13y <= signed(resize(s_y12, 13)) + resize(s_pc12(19 downto 9), 13);
            s_u13 <= s_u12; s_v13 <= s_v12;

            -- 14: legal luma, highlight knee (940 -> 770), neutral outside
            -- avid so the smoothing's edge taps read black
            if s_avid_sr(12) = '0' then
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            else
                if s_v13y < 64 then
                    s_y14 <= to_unsigned(64, 10);
                elsif s_v13y > 940 then
                    s_y14 <= to_unsigned(770, 10);
                elsif s_v13y > 600 then
                    s_y14 <= to_unsigned(300, 10) + resize(unsigned(s_v13y(9 downto 1)), 10);
                else
                    s_y14 <= unsigned(s_v13y(9 downto 0));
                end if;
                s_u14 <= s_u13; s_v14 <= s_v13;
            end if;

            -- 15: [1 2 1] on Y/U/V (the HD HDMI bar fix), blanking gate
            s_ya <= s_y14; s_yb <= s_ya;
            s_ua <= s_u14; s_ub <= s_ua;
            s_va <= s_v14; s_vb <= s_va;
            if s_avid_sr(13) = '1' then
                s_y15 <= f_121(s_yb, s_ya, s_y14);
                s_u15 <= f_121(s_ub, s_ua, s_u14);
                s_v15 <= f_121(s_vb, s_va, s_v14);
            else
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y15);
    s_io_align_in.u       <= std_logic_vector(s_u15);
    s_io_align_in.v       <= std_logic_vector(s_v15);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (15 -> 16).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture syncbleed;
