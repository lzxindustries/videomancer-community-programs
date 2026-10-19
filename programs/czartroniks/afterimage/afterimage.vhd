-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- afterimage.vhd  (v1.0.2 -- "Afterimage": 1-bit frame-store feedback)
--
-- v1.0.2: resolution scaling.  Cells stay the 1080p size relative to screen
--         height and keep their shape: SD (4:3, timing ID 0/4/8/12) uses 3/4
--         of the column count (192x128 .. 24x16) instead of the old W<1280
--         halving to 128x64.  Write address = the ended cell index latched
--         with the write pulse (was the F-chain cell 3 px back), so 3-4 px SD
--         cells write the right cell even when a line runs a pixel short.
--         480i finest grid is 180x120 (exact 4 px x 2 field-line cells, same
--         shape, 6.7% larger) since 128 rows on ~243 field lines would mix
--         1- and 2-line rows; a row only one line tall (short-field
--         sources) is still sampled on that line.  1080p/720p unchanged.
-- v1.0.0: Persist grain compare fixed to the labelled 25..100% (was 12.5..50%);
--         horizontal [1 2 1]/4 luma low-pass at the output (latency 6 -> 8),
--         re-gated on blanking, against the 1-px full-swing-step HDMI bars.
--
-- Videomancer has no frame buffer, but a 256x128 ONE-BIT frame fits in
-- 8 EBR, and two of them (ping-pong by frame parity) give a genuine
-- frame-recursive picture: this frame's cell = f(this frame's dithered
-- sample, LAST frame's cell read back from a displaced address).
--
-- The picture IS the bit store: every cell is drawn as ink or paper, so
-- the block is honestly the primitive (Game Boy camera / 1-bit Mac look).
-- Feedback = stochastic: a cell's old ink survives with probability P12
-- (per-cell hash < persistence), so trails dissolve grain by grain.
-- Displaced source addresses (zoom / drift / spin) use stochastic rounding
-- of the fractional cell offset so sub-cell velocities exist even at 1 bit.
--
-- Controls:
--   K1 Feedback: Hold / Zoom In / Zoom Out / Left / Right / Up / Down / Spin
--   K2 Grid: 256x128 / 128x64 / 64x32 / 32x16 cells on 16:9 (SD 4:3:
--      192x128 / 96x64 / 48x32 / 24x16, 480i 180x120 first; same cell shape
--      and size relative to screen height)
--   K3 Exposure (luma bias before dithering)
--   K4 Dither: Bayer 8x8 / Bayer 4x4 / Threshold / Noise (boils per frame)
--   K5 Palette: 8 ink/paper pairs
--   K6 Diffuse: fraction of cells whose feedback source is randomly
--      displaced +/-3 cells (random-walk diffusion of the trails)
--   S7 Invert, S8 Fade grain/Bayer (random vs ordered dissolve), S9 Mirror
--      feedback, S10 Ink solid/video,
--   S11 Input live/freeze
--   P12 Persist: survival probability 25%..100%
--
-- Store organisation: {row(7), col(8)} -> 32768 x 1 bit per bank, read
-- every clock for display (cell of the current pixel); the feedback read
-- steals one non-cell-start slot per cell (a 1-clock deferral keeps the
-- first pixel of every cell honest), the display hold register covers it.
-- Cell sampling: one-pole IIR along the line, latched at the cell end on
-- the cell row's middle line.  All feedback math runs on a pulse-tagged
-- pixel-rate pipeline (no per-cell holds), one op per stage.
--
-- 16 EBR, 0 multipliers.  Generated colours stored U/V-swapped.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture afterimage of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (9 + 3 = 12 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY : natural := 8;

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- Palettes (stored U = Cr, V = Cb : U/V-swapped for hardware).
    -- 0 Mono  1 Paper  2 Game Boy  3 Phosphor  4 Amber  5 Blueprint
    -- 6 Ember  7 Neon
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant INK_Y : t_pal := (to_unsigned(768,10), to_unsigned( 64,10),
        to_unsigned(198,10), to_unsigned(651,10), to_unsigned(681,10),
        to_unsigned(768,10), to_unsigned(568,10), to_unsigned(533,10));
    constant INK_U : t_pal := (to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(452,10), to_unsigned(212,10), to_unsigned(701,10),
        to_unsigned(512,10), to_unsigned(783,10), to_unsigned(809,10));
    constant INK_V : t_pal := (to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(464,10), to_unsigned(275,10), to_unsigned(156,10),
        to_unsigned(512,10), to_unsigned(221,10), to_unsigned(677,10));
    constant PAP_Y : t_pal := (to_unsigned( 64,10), to_unsigned(768,10),
        to_unsigned(608,10), to_unsigned( 64,10), to_unsigned( 64,10),
        to_unsigned(268,10), to_unsigned(146,10), to_unsigned(118,10));
    constant PAP_U : t_pal := (to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(503,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(413,10), to_unsigned(652,10), to_unsigned(498,10));
    constant PAP_V : t_pal := (to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(228,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(711,10), to_unsigned(465,10), to_unsigned(600,10));

    function rotl16(x : unsigned(15 downto 0); k : natural) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x(15 - k downto 0) & x(15 downto 16 - k);
        return v;
    end function;

    -- hash round: t = x xor rotl(x, ra); x' = t + (t sll sh)
    function hround(x : unsigned(15 downto 0); ra : natural; sh : natural) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := x xor rotl16(x, ra);
        return t + shift_left(t, sh);
    end function;

    -- 3-bit hash field -> signed jitter -3..3 (4 -> 0 twice)
    function map3(h : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(h) is
            when 0 => return to_signed(-3, 4);
            when 1 => return to_signed(-2, 4);
            when 2 => return to_signed(-1, 4);
            when 5 => return to_signed( 1, 4);
            when 6 => return to_signed( 2, 4);
            when 7 => return to_signed( 3, 4);
            when others => return to_signed(0, 4);
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Control decode.
    --------------------------------------------------------------------------
    signal rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0) := (others => '0');
    signal rsw : std_logic_vector(4 downto 0) := (others => '0');
    signal s_mode : unsigned(2 downto 0) := (others => '0');
    signal s_gsel : unsigned(1 downto 0) := (others => '0');
    signal s_exp  : signed(11 downto 0) := (others => '0');
    signal s_dsel : unsigned(1 downto 0) := (others => '0');
    signal s_psel : unsigned(2 downto 0) := (others => '0');
    signal s_jp8  : unsigned(7 downto 0) := (others => '0');
    signal s_p9   : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal sw_inv, sw_ord, sw_mir, sw_vid, sw_frz : std_logic := '0';
    signal m_zoom, m_spin : std_logic := '0';
    signal s_sd : std_logic := '0';   -- 4:3 SD timing (480i/p, 576i/p)
    signal s_480i : std_logic := '0'; -- 480i (~243 field lines)

    --------------------------------------------------------------------------
    -- Raster / geometry (k=0).
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal prev_hsync_n, prev_vsync_n, prev_avid : std_logic := '1';
    signal lfsr : unsigned(15 downto 0) := x"ACE1";
    signal px   : unsigned(11 downto 0) := (others => '0');
    signal py   : unsigned(10 downto 0) := (others => '0');
    signal line_act, frame_act, frame_armed : std_logic := '0';
    signal w_meas : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal h_meas : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal g_h    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_cols : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal g_rows : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal g_cx   : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal g_ry   : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal g_lpk  : unsigned(1 downto 0) := "10";     -- IIR shift 2..4
    signal g_cm1  : signed(9 downto 0) := to_signed(255, 10);  -- cols-1
    signal g_t1   : unsigned(13 downto 0) := (others => '1');  -- 2H - 2*rows
    signal par    : std_logic := '0';
    signal fcnt   : unsigned(5 downto 0) := (others => '0');
    signal seed1 : unsigned(15 downto 0) := x"1234";
    signal seed2 : unsigned(15 downto 0) := x"9ABC";
    signal seed3 : unsigned(15 downto 0) := x"5678";

    signal xacc : unsigned(11 downto 0) := (others => '0');
    signal c    : unsigned(8 downto 0) := (others => '0');
    signal cs   : std_logic := '0';
    signal yacc : unsigned(11 downto 0) := (others => '0');
    signal r    : unsigned(7 downto 0) := (others => '0');
    signal mid_done, sample_line : std_logic := '0';
    signal af   : std_logic := '0';
    signal lp0, lp1, lp2 : unsigned(9 downto 0) := C_BLKY;
    signal wp0  : std_logic := '0';
    signal wc0  : unsigned(7 downto 0) := (others => '0');  -- ended cell

    --------------------------------------------------------------------------
    -- F-chain: free-running per-cell values (k=1..4).
    --------------------------------------------------------------------------
    signal f1_dc : signed(9 downto 0) := (others => '0');
    signal f1_dr : signed(9 downto 0) := (others => '0');
    signal f1_t8 : unsigned(5 downto 0) := (others => '0');
    signal f1_t4 : unsigned(3 downto 0) := (others => '0');
    signal f1_h1, f1_h2, f1_h3 : unsigned(15 downto 0) := (others => '0');
    signal f1_r  : unsigned(6 downto 0) := (others => '0');

    signal f2_sx, f2_sy : signed(9 downto 0) := (others => '0');
    signal f2_t6, f2_t8 : unsigned(5 downto 0) := (others => '0');
    signal f2_h1, f2_h2, f2_h3 : unsigned(15 downto 0) := (others => '0');
    signal f2_r  : unsigned(6 downto 0) := (others => '0');

    signal f3_qx, f3_qy : signed(3 downto 0) := (others => '0');
    signal f3_fx, f3_fy : unsigned(5 downto 0) := (others => '0');
    signal f3_t6, f3_t8 : unsigned(5 downto 0) := (others => '0');
    signal f3_h1, f3_h2, f3_h3 : unsigned(15 downto 0) := (others => '0');
    signal f3_r  : unsigned(6 downto 0) := (others => '0');

    signal f4_qx, f4_qy : signed(3 downto 0) := (others => '0');
    signal f4_fx, f4_fy : unsigned(5 downto 0) := (others => '0');
    signal f4_t6, f4_t8 : unsigned(5 downto 0) := (others => '0');
    signal f4_h1, f4_h2, f4_h3 : unsigned(15 downto 0) := (others => '0');
    signal f4_r  : unsigned(6 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- P-chain: pulse-tagged feedback pipeline.
    --------------------------------------------------------------------------
    signal p0_v : std_logic := '0';
    signal p0_qx, p0_qy : signed(3 downto 0) := (others => '0');
    signal p0_fx, p0_fy : unsigned(5 downto 0) := (others => '0');
    signal p0_t6, p0_t8 : unsigned(5 downto 0) := (others => '0');
    signal p0_h1, p0_h2, p0_h3 : unsigned(15 downto 0) := (others => '0');
    signal p0_c  : unsigned(7 downto 0) := (others => '0');
    signal p0_r  : unsigned(6 downto 0) := (others => '0');
    signal p0_l  : unsigned(9 downto 0) := (others => '0');

    signal p1_v : std_logic := '0';
    signal p1_t14 : unsigned(9 downto 0) := (others => '0');
    signal p1_vl  : signed(11 downto 0) := (others => '0');
    signal p1_surv, p1_jon : std_logic := '0';
    signal p1_jx, p1_jy : signed(3 downto 0) := (others => '0');
    signal p1_sx, p1_sy : signed(4 downto 0) := (others => '0');
    signal p1_c  : unsigned(7 downto 0) := (others => '0');
    signal p1_r  : unsigned(6 downto 0) := (others => '0');

    signal p2_v, p2_s, p2_surv : std_logic := '0';
    signal p2_ox, p2_oy : signed(5 downto 0) := (others => '0');
    signal p2_c  : unsigned(7 downto 0) := (others => '0');
    signal p2_r  : unsigned(6 downto 0) := (others => '0');

    signal p3_v, p3_s, p3_surv : std_logic := '0';
    signal p3_cx : signed(9 downto 0) := (others => '0');
    signal p3_ry : signed(8 downto 0) := (others => '0');
    signal p3_c  : unsigned(7 downto 0) := (others => '0');
    signal p3_r  : unsigned(6 downto 0) := (others => '0');

    signal p4_v, p4_s, p4_surv, p4_rv : std_logic := '0';
    signal p4_cx : signed(9 downto 0) := (others => '0');
    signal p4_ry : signed(8 downto 0) := (others => '0');
    signal p4_c  : unsigned(7 downto 0) := (others => '0');
    signal p4_r  : unsigned(6 downto 0) := (others => '0');

    signal p5_v, p5_s, p5_surv, p5_fbv : std_logic := '0';
    signal p5_addr : unsigned(14 downto 0) := (others => '0');
    signal p5_wa   : unsigned(14 downto 0) := (others => '0');

    -- request register
    signal req_v, req_s, req_surv, req_fbv : std_logic := '0';
    signal req_addr, req_wa : unsigned(14 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Frame store: two 32768 x 1 banks, ping-pong by frame parity.
    --------------------------------------------------------------------------
    type t_bank is array (0 to 32767) of std_logic_vector(0 downto 0);
    signal bank_a, bank_b : t_bank := (others => "0");
    signal raddr : unsigned(14 downto 0) := (others => '0');
    signal rtag1, rtag2, rtag3 : std_logic := '0';
    signal rd_a, rd_b, rda_f, rdb_f : std_logic_vector(0 downto 0) := "0";
    signal c1_s, c1_surv, c1_fbv : std_logic := '0';
    signal c1_wa : unsigned(14 downto 0) := (others => '0');
    signal c2_s, c2_surv, c2_fbv : std_logic := '0';
    signal c2_wa : unsigned(14 downto 0) := (others => '0');
    signal c3_s, c3_surv, c3_fbv : std_logic := '0';
    signal c3_wa : unsigned(14 downto 0) := (others => '0');

    signal disp4 : std_logic := '0';
    signal w4, wv4 : std_logic := '0';
    signal waddr4 : unsigned(14 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Colour / output.
    --------------------------------------------------------------------------
    signal ink4_y, ink4_u, ink4_v : unsigned(9 downto 0) := C_BLKY;
    signal pap4_y, pap4_u, pap4_v : unsigned(9 downto 0) := C_BLKY;
    signal col5_y, col5_u, col5_v : unsigned(9 downto 0) := C_BLKY;
    signal blank5 : std_logic := '1';
    signal g6_y, g6_u, g6_v : unsigned(9 downto 0) := C_BLKY;
    signal g6_bl : std_logic := '1';
    signal d7_y, d7_u, d7_v : unsigned(9 downto 0) := C_BLKY;
    signal d7_bl : std_logic := '1';
    signal d8_y : unsigned(9 downto 0) := C_BLKY;
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            rk1  <= unsigned(registers_in(0)(9 downto 0));
            rk2  <= unsigned(registers_in(1)(9 downto 0));
            rk3  <= unsigned(registers_in(2)(9 downto 0));
            rk4  <= unsigned(registers_in(3)(9 downto 0));
            rk5  <= unsigned(registers_in(4)(9 downto 0));
            rk6  <= unsigned(registers_in(5)(9 downto 0));
            rsw  <= registers_in(6)(4 downto 0);
            rp12 <= unsigned(registers_in(7)(9 downto 0));

            s_mode <= rk1(9 downto 7);
            s_gsel <= rk2(9 downto 8);
            s_exp  <= signed(resize(rk3, 12)) - to_signed(512, 12);
            s_dsel <= rk4(9 downto 8);
            s_psel <= rk5(9 downto 7);
            s_jp8  <= rk6(9 downto 2);
            -- persistence 64..256 of 256 -> 25%..100% (256 = never clears)
            s_p9   <= to_unsigned(64, 9) + resize(rp12(9 downto 2), 9) - resize(rp12(9 downto 4), 9);

            sw_inv <= rsw(0);   -- S7  Invert
            sw_ord <= rsw(1);   -- S8  Fade grain/Bayer
            sw_mir <= rsw(2);   -- S9  Mirror
            sw_vid <= rsw(3);   -- S10 Ink solid/video
            sw_frz <= rsw(4);   -- S11 Input live/freeze

            if rk1(9 downto 7) = 1 or rk1(9 downto 7) = 2 then m_zoom <= '1'; else m_zoom <= '0'; end if;
            if rk1(9 downto 7) = 7 then m_spin <= '1'; else m_spin <= '0'; end if;
            -- 4:3 SD timings (480i, 480p, 576i, 576p)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_sd <= '1';
                when others         => s_sd <= '0';
            end case;
            if registers_in(8)(3 downto 0) = "0000" then s_480i <= '1'; else s_480i <= '0'; end if;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Raster counters, cell DDAs, per-frame geometry latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_x : unsigned(12 downto 0);
        variable v_y : unsigned(12 downto 0);
        variable v_gs : unsigned(1 downto 0);
        variable v_first : std_logic;
        variable v_d, v_n : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- active-video fall = end of the line's last cell
            if prev_avid = '1' and data_in.avid = '0' then af <= '1'; else af <= '0'; end if;

            -- IIR luma sample along the line (reset at line start)
            lp1 <= lp0;
            lp2 <= lp1;

            if v_h_edge = '1' then
                if px > 0 then w_meas <= px; end if;
                px <= (others => '0');
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    h_meas   <= resize(py, 12) + 1;
                    -- row DDA, sample line = first line past the row's middle
                    v_y := ('0' & yacc) + resize(g_rows, 13);
                    if v_y >= ('0' & g_h) then
                        yacc        <= resize(v_y - ('0' & g_h), 12);
                        r           <= r + 1;
                        mid_done    <= '0';
                        -- a row only ONE line tall (rows > H/2: next line
                        -- also ends its row, i.e. yacc + 2*rows >= 2H) has
                        -- no line past its middle: sample its only line.
                        -- Safety net for short-field sources; never true in
                        -- the standard modes (all have 2+ lines/row).  Row
                        -- 128 would alias row 0.
                        if resize(yacc, 14) >= g_t1 and r /= 127 then
                            sample_line <= '1';
                        else
                            sample_line <= '0';
                        end if;
                    else
                        yacc <= v_y(11 downto 0);
                        if mid_done = '0' and v_y >= ("00" & g_h(11 downto 1)) then
                            sample_line <= '1';
                            mid_done    <= '1';
                        else
                            sample_line <= '0';
                        end if;
                    end if;
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                v_first := not line_act;
                line_act <= '1';
                px <= px + 1;
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                end if;
                -- column DDA: cell(p) = floor(p * cols / W)
                if v_first = '1' then
                    xacc <= (others => '0');
                    c    <= (others => '0');
                    cs   <= '0';
                    lp0  <= unsigned(data_in.y);
                else
                    v_x := ('0' & xacc) + resize(g_cols, 13);
                    if v_x >= ('0' & g_w) then
                        xacc <= resize(v_x - ('0' & g_w), 12);
                        c    <= c + 1;
                        cs   <= '1';
                    else
                        xacc <= v_x(11 downto 0);
                        cs   <= '0';
                    end if;
                    v_d := signed('0' & unsigned(data_in.y)) - signed('0' & lp0);
                    case g_lpk is
                        when "10"   => v_n := signed('0' & lp0) + shift_right(v_d, 2);
                        when "11"   => v_n := signed('0' & lp0) + shift_right(v_d, 3);
                        when others => v_n := signed('0' & lp0) + shift_right(v_d, 4);
                    end case;
                    lp0 <= unsigned(v_n(9 downto 0));
                end if;
            else
                cs <= '0';
            end if;

            if v_v_edge = '1' and frame_armed = '1' then
                -- frame anchor (serration-guarded): latch geometry, flip banks
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                yacc        <= (others => '0');
                r           <= (others => '0');
                mid_done    <= '0';
                sample_line <= '0';
                par         <= not par;
                fcnt        <= fcnt + 1;
                seed1 <= lfsr;
                seed2 <= rotl16(lfsr, 5) xor x"5A5A";
                seed3 <= rotl16(lfsr, 11) xor x"C3A5";
                g_w <= w_meas;
                g_h <= h_meas;
                -- grid: rows are a fraction of the field height; columns
                -- span the width, 3/4 as many on 4:3 SD so a cell keeps
                -- the 16:9 shape and size relative to screen height
                -- (SD 192 cols = 3.75 px cells; >= 3 px is all the cell
                -- pipeline needs).  Sub-640 widths (decimated sims) cap at
                -- the 64x32 grid as before.
                v_gs := s_gsel;
                if w_meas < 640 and v_gs(1) = '0' then v_gs := "10"; end if;
                case v_gs is
                    when "00" => g_rows <= to_unsigned(128, 8); g_ry <= to_unsigned(64, 7); g_lpk <= "10";
                    when "01" => g_rows <= to_unsigned(64, 8);  g_ry <= to_unsigned(32, 7); g_lpk <= "11";
                    when "10" => g_rows <= to_unsigned(32, 8);  g_ry <= to_unsigned(16, 7); g_lpk <= "00";
                    when others => g_rows <= to_unsigned(16, 8); g_ry <= to_unsigned(8, 7); g_lpk <= "00";
                end case;
                if s_sd = '0' then
                    case v_gs is
                        when "00" => g_cols <= to_unsigned(256, 9); g_cx <= to_unsigned(128, 8); g_cm1 <= to_signed(255, 10);
                        when "01" => g_cols <= to_unsigned(128, 9); g_cx <= to_unsigned(64, 8);  g_cm1 <= to_signed(127, 10);
                        when "10" => g_cols <= to_unsigned(64, 9);  g_cx <= to_unsigned(32, 8);  g_cm1 <= to_signed(63, 10);
                        when others => g_cols <= to_unsigned(32, 9); g_cx <= to_unsigned(16, 8); g_cm1 <= to_signed(31, 10);
                    end case;
                else
                    case v_gs is
                        when "00" =>
                            if s_480i = '1' then
                                -- 480i: 128 rows on ~243 field lines would mix
                                -- 1- and 2-line rows (thin stripes); 180x120 =
                                -- exact 4 px x 2 field-line cells, same shape,
                                -- 6.7% larger
                                g_cols <= to_unsigned(180, 9); g_cx <= to_unsigned(90, 8);  g_cm1 <= to_signed(179, 10);
                                g_rows <= to_unsigned(120, 8); g_ry <= to_unsigned(60, 7);
                            else
                                g_cols <= to_unsigned(192, 9); g_cx <= to_unsigned(96, 8);  g_cm1 <= to_signed(191, 10);
                            end if;
                        when "01" => g_cols <= to_unsigned(96, 9);  g_cx <= to_unsigned(48, 8);  g_cm1 <= to_signed(95, 10);
                        when "10" => g_cols <= to_unsigned(48, 9);  g_cx <= to_unsigned(24, 8);  g_cm1 <= to_signed(47, 10);
                        when others => g_cols <= to_unsigned(24, 9); g_cx <= to_unsigned(12, 8); g_cm1 <= to_signed(23, 10);
                    end case;
                end if;
            end if;

            -- write pulse: a cell just ended on the row's sample line;
            -- wc0 = the ended cell (c - 1 after a cell start, c at the
            -- active-video fall)
            if sample_line = '1' and (cs = '1' or af = '1') then
                wp0 <= '1';
            else
                wp0 <= '0';
            end if;
            if cs = '1' then
                wc0 <= c(7 downto 0) - 1;
            else
                wc0 <= c(7 downto 0);
            end if;

            -- single-line-row threshold, free-running from the frame latch
            g_t1 <= shift_left(resize(g_h, 14), 1) - shift_left(resize(g_rows, 14), 1);
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_a15 : unsigned(15 downto 0);
        variable v_x1, v_x2, v_x3 : unsigned(15 downto 0);
        variable v_y1, v_y2, v_y3 : unsigned(15 downto 0);
        variable v_sx, v_sy : signed(4 downto 0);
        variable v_ox, v_oy : signed(5 downto 0);
        variable v_jx, v_jy : signed(5 downto 0);
        variable v_rd  : std_logic;
        variable v_fbk, v_sf : std_logic;
        variable v_issue : std_logic;
        variable v_iy, v_iu, v_iv, v_py, v_pu, v_pv : unsigned(9 downto 0);
        variable v_ysum : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -----------------------------------------------------------------
            -- F1: centre offsets, Bayer, hash seed mix (k=1)
            -----------------------------------------------------------------
            f1_dc <= signed(resize(c(7 downto 0), 10)) - signed(resize(g_cx, 10));
            f1_dr <= signed(resize(r(6 downto 0), 10)) - signed(resize(g_ry, 10));
            f1_t8 <= (c(0) xor r(0)) & r(0) & (c(1) xor r(1)) & r(1) & (c(2) xor r(2)) & r(2);
            f1_t4 <= (c(0) xor r(0)) & r(0) & (c(1) xor r(1)) & r(1);
            v_a15 := '0' & c(7 downto 0) & r(6 downto 0);
            v_x1 := v_a15 xor rotl16(seed1, 3);
            v_y1 := rotl16(v_a15, 5) xor seed1;
            f1_h1 <= v_x1 + rotl16(v_y1, 7);
            v_x2 := v_a15 xor rotl16(seed2, 3);
            v_y2 := rotl16(v_a15, 5) xor seed2;
            f1_h2 <= v_x2 + rotl16(v_y2, 7);
            v_x3 := v_a15 xor rotl16(seed3, 3);
            v_y3 := rotl16(v_a15, 5) xor seed3;
            f1_h3 <= v_x3 + rotl16(v_y3, 7);
            f1_r <= r(6 downto 0);

            -----------------------------------------------------------------
            -- F2: mode source select, dither level, hash round 1 (k=2)
            -----------------------------------------------------------------
            if m_zoom = '1' then
                f2_sx <= f1_dc; f2_sy <= f1_dr;
            elsif m_spin = '1' then
                f2_sx <= f1_dr; f2_sy <= -f1_dc;
            else
                f2_sx <= (others => '0'); f2_sy <= (others => '0');
            end if;
            case s_dsel is
                when "00"   => f2_t6 <= f1_t8;
                when "01"   => f2_t6 <= f1_t4 & "00";
                when others => f2_t6 <= to_unsigned(32, 6);
            end case;
            f2_t8 <= f1_t8;
            f2_h1 <= hround(f1_h1, 5, 3);
            f2_h2 <= hround(f1_h2, 5, 3);
            f2_h3 <= hround(f1_h3, 5, 3);
            f2_r <= f1_r;

            -----------------------------------------------------------------
            -- F3: quotient/fraction of the 1/64 displacement, hash round 2
            -----------------------------------------------------------------
            f3_qx <= f2_sx(9 downto 6); f3_fx <= unsigned(f2_sx(5 downto 0));
            f3_qy <= f2_sy(9 downto 6); f3_fy <= unsigned(f2_sy(5 downto 0));
            f3_t6 <= f2_t6; f3_t8 <= f2_t8;
            f3_h1 <= hround(f2_h1, 11, 5);
            f3_h2 <= hround(f2_h2, 11, 5);
            f3_h3 <= hround(f2_h3, 11, 5);
            f3_r <= f2_r;

            -----------------------------------------------------------------
            -- F4: hash round 3 (k=4)
            -----------------------------------------------------------------
            f4_qx <= f3_qx; f4_fx <= f3_fx; f4_qy <= f3_qy; f4_fy <= f3_fy;
            f4_t6 <= f3_t6; f4_t8 <= f3_t8;
            f4_h1 <= hround(f3_h1, 7, 3);
            f4_h2 <= hround(f3_h2, 7, 3);
            f4_h3 <= hround(f3_h3, 7, 3);
            f4_r <= f3_r;

            -----------------------------------------------------------------
            -- P0: pulse capture of the ended cell.  wp0 is k=1, so F4 here
            -- is pixel p-3 (inside the ended cell for any width >= 3) and
            -- lp2 is the IIR after the cell's last pixel.  The cell index
            -- itself comes from wc0 (latched with the pulse), so even a
            -- truncated 2-px last cell writes its own address.
            -----------------------------------------------------------------
            p0_v  <= wp0;
            p0_qx <= f4_qx; p0_fx <= f4_fx; p0_qy <= f4_qy; p0_fy <= f4_fy;
            if s_dsel = "11" then p0_t6 <= f4_h2(13 downto 8); else p0_t6 <= f4_t6; end if;
            p0_t8 <= f4_t8;
            p0_h1 <= f4_h1; p0_h2 <= f4_h2; p0_h3 <= f4_h3;
            p0_c  <= wc0;   p0_r  <= f4_r;
            p0_l  <= lp2;

            -----------------------------------------------------------------
            -- P1: threshold, exposure, survival, jitter, stochastic rounding
            -----------------------------------------------------------------
            p1_v   <= p0_v;
            p1_t14 <= (p0_t6 & "0000") - ("000" & p0_t6 & '0');
            p1_vl  <= signed(resize(p0_l, 12)) - to_signed(64, 12) + s_exp;
            -- survival: random grain, or ordered (Bayer index rolled by the
            -- frame counter so every cell survives a fraction p of frames)
            if sw_ord = '1' then
                if ('0' & (p0_t8 xor fcnt) & "00") < s_p9 then p1_surv <= '1'; else p1_surv <= '0'; end if;
            else
                if resize(p0_h1(7 downto 0), 9) < s_p9 then p1_surv <= '1'; else p1_surv <= '0'; end if;
            end if;
            if p0_h2(7 downto 0) < s_jp8 then p1_jon <= '1'; else p1_jon <= '0'; end if;
            p1_jx <= map3(p0_h1(11 downto 9));
            p1_jy <= map3(p0_h1(14 downto 12));
            v_sx := resize(p0_qx, 5);
            if p0_h3(5 downto 0) < p0_fx then v_sx := v_sx + 1; end if;
            v_sy := resize(p0_qy, 5);
            if p0_h3(11 downto 6) < p0_fy then v_sy := v_sy + 1; end if;
            p1_sx <= v_sx; p1_sy <= v_sy;
            p1_c <= p0_c; p1_r <= p0_r;

            -----------------------------------------------------------------
            -- P2: dither decision, mode offset + jitter
            -----------------------------------------------------------------
            p2_v <= p1_v;
            if p1_vl > signed(resize(p1_t14, 12)) then p2_s <= '1'; else p2_s <= '0'; end if;
            p2_surv <= p1_surv;
            case s_mode is
                when "001"  => v_ox := -resize(p1_sx, 6); v_oy := -resize(p1_sy, 6);  -- zoom in
                when "010"  => v_ox :=  resize(p1_sx, 6); v_oy :=  resize(p1_sy, 6);  -- zoom out
                when "011"  => v_ox := to_signed( 1, 6);  v_oy := (others => '0');     -- left
                when "100"  => v_ox := to_signed(-1, 6);  v_oy := (others => '0');     -- right
                when "101"  => v_ox := (others => '0');   v_oy := to_signed( 1, 6);    -- up
                when "110"  => v_ox := (others => '0');   v_oy := to_signed(-1, 6);    -- down
                when "111"  => v_ox :=  resize(p1_sx, 6); v_oy :=  resize(p1_sy, 6);  -- spin
                when others => v_ox := (others => '0');   v_oy := (others => '0');     -- hold
            end case;
            if p1_jon = '1' then
                v_jx := resize(p1_jx, 6); v_jy := resize(p1_jy, 6);
            else
                v_jx := (others => '0'); v_jy := (others => '0');
            end if;
            p2_ox <= v_ox + v_jx;
            p2_oy <= v_oy + v_jy;
            p2_c <= p1_c; p2_r <= p1_r;

            -----------------------------------------------------------------
            -- P3: source cell coordinates
            -----------------------------------------------------------------
            p3_v <= p2_v; p3_s <= p2_s; p3_surv <= p2_surv;
            p3_cx <= signed(resize(p2_c, 10)) + resize(p2_ox, 10);
            p3_ry <= signed(resize(p2_r, 9)) + resize(p2_oy, 9);
            p3_c <= p2_c; p3_r <= p2_r;

            -----------------------------------------------------------------
            -- P4: mirror, row validity
            -----------------------------------------------------------------
            p4_v <= p3_v; p4_s <= p3_s; p4_surv <= p3_surv;
            if sw_mir = '1' then p4_cx <= g_cm1 - p3_cx; else p4_cx <= p3_cx; end if;
            if p3_ry >= 0 and p3_ry < signed(resize(g_rows, 9)) then p4_rv <= '1'; else p4_rv <= '0'; end if;
            p4_ry <= p3_ry;
            p4_c <= p3_c; p4_r <= p3_r;

            -----------------------------------------------------------------
            -- P5: column validity, addresses
            -----------------------------------------------------------------
            p5_v <= p4_v; p5_s <= p4_s; p5_surv <= p4_surv;
            if p4_rv = '1' and p4_cx >= 0 and p4_cx < signed(resize(g_cols, 10)) then
                p5_fbv <= '1';
            else
                p5_fbv <= '0';
            end if;
            p5_addr <= unsigned(p4_ry(6 downto 0)) & unsigned(p4_cx(7 downto 0));
            p5_wa   <= p4_r & p4_c;

            -----------------------------------------------------------------
            -- Request register + read-port arbitration (k=0 -> raddr k=1).
            -- The display read (cell of the current pixel) owns every
            -- cell-start slot; the feedback read takes the next free one.
            -----------------------------------------------------------------
            v_issue := '0';
            if req_v = '1' and cs = '0' then v_issue := '1'; end if;
            if v_issue = '1' then
                raddr <= req_addr;
                rtag1 <= '1';
                req_v <= '0';
            else
                raddr <= r(6 downto 0) & c(7 downto 0);
                rtag1 <= '0';
            end if;
            c1_s <= req_s; c1_surv <= req_surv; c1_fbv <= req_fbv; c1_wa <= req_wa;
            if p5_v = '1' then
                req_v <= '1';
                req_addr <= p5_addr; req_wa <= p5_wa;
                req_s <= p5_s; req_surv <= p5_surv; req_fbv <= p5_fbv;
            end if;

            -- k=2: EBR reads (both banks, unconditional)
            rd_a <= bank_a(to_integer(raddr));
            rd_b <= bank_b(to_integer(raddr));
            rtag2 <= rtag1;
            c2_s <= c1_s; c2_surv <= c1_surv; c2_fbv <= c1_fbv; c2_wa <= c1_wa;

            -- k=3: fabric re-register
            rda_f <= rd_a; rdb_f <= rd_b;
            rtag3 <= rtag2;
            c3_s <= c2_s; c3_surv <= c2_surv; c3_fbv <= c2_fbv; c3_wa <= c2_wa;

            -- k=4: bank select; display hold / feedback combine
            if par = '0' then v_rd := rdb_f(0); else v_rd := rda_f(0); end if;
            if rtag3 = '0' then
                disp4 <= v_rd;
                wv4   <= '0';
            else
                v_fbk := v_rd and c3_surv and c3_fbv;
                v_sf  := c3_s and not sw_frz;
                w4     <= v_sf or v_fbk;
                wv4    <= '1';
                waddr4 <= c3_wa;
            end if;

            -- k=5: single write per bank into the frame's write bank
            if wv4 = '1' then
                if par = '0' then
                    bank_a(to_integer(waddr4)) <= (0 => w4);
                else
                    bank_b(to_integer(waddr4)) <= (0 => w4);
                end if;
            end if;

            -----------------------------------------------------------------
            -- Colour: palette (k=4), compose (k=5), gate (k=6), luma LPF, output (k=8)
            -----------------------------------------------------------------
            v_iy := INK_Y(to_integer(s_psel)); v_iu := INK_U(to_integer(s_psel)); v_iv := INK_V(to_integer(s_psel));
            v_py := PAP_Y(to_integer(s_psel)); v_pu := PAP_U(to_integer(s_psel)); v_pv := PAP_V(to_integer(s_psel));
            if sw_inv = '0' then
                ink4_y <= v_iy; ink4_u <= v_iu; ink4_v <= v_iv;
                pap4_y <= v_py; pap4_u <= v_pu; pap4_v <= v_pv;
            else
                ink4_y <= v_py; ink4_u <= v_pu; ink4_v <= v_pv;
                pap4_y <= v_iy; pap4_u <= v_iu; pap4_v <= v_iv;
            end if;

            if disp4 = '1' then
                if sw_vid = '1' then
                    col5_y <= unsigned(pipe(4).y); col5_u <= unsigned(pipe(4).u); col5_v <= unsigned(pipe(4).v);
                else
                    col5_y <= ink4_y; col5_u <= ink4_u; col5_v <= ink4_v;
                end if;
            else
                col5_y <= pap4_y; col5_u <= pap4_u; col5_v <= pap4_v;
            end if;
            blank5 <= not pipe(4).avid;

            -- k=6: blanking gate
            if blank5 = '1' then
                g6_y <= C_BLKY; g6_u <= C_MID; g6_v <= C_MID;
            else
                g6_y <= col5_y; g6_u <= col5_u; g6_v <= col5_v;
            end if;
            g6_bl <= blank5;

            -- k=7: delay (centre tap of the luma low-pass)
            d7_y <= g6_y; d7_u <= g6_u; d7_v <= g6_v; d7_bl <= g6_bl;

            -- k=8: horizontal [1 2 1]/4 on luma only, re-gated on blanking
            d8_y <= d7_y;
            if d7_bl = '1' then
                s_io.y <= std_logic_vector(C_BLKY);
            else
                v_ysum := resize(d8_y, 12) + (resize(d7_y, 12) sll 1) + resize(g6_y, 12) + 2;
                s_io.y <= std_logic_vector(v_ysum(11 downto 2));
            end if;
            s_io.u <= std_logic_vector(d7_u);
            s_io.v <= std_logic_vector(d7_v);
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    s_io_align_in.y       <= s_io.y;
    s_io_align_in.u       <= s_io.u;
    s_io_align_in.v       <= s_io.v;
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.field_n <= s_io.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (9 -> 12).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 2 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(2);

end architecture afterimage;
