-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- triangulator.vhd  (v0.2 — triangle pixelation)
--
-- v1.0.2 (2026-10-17): resolution scaling.  The cell lattice runs in virtual
-- 1080-line units, stepped per pixel / per field line from the video timing
-- ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p, and
-- at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame line, so
-- the triangles keep their size relative to the screen height and stay
-- equilateral on a 4:3 screen (they were 1.5x too big at 720p and 2:1 tall
-- at SD).  The row counter runs the integer part of each field line's step
-- as unit steps after the h edge; the sample line is the first line at/after
-- H - ceil(step) (the last line at 1080p).  Line widths are virtual, clamped
-- so no edge can fall between SD/720p samples (diagonal band >= 2; a
-- horizontal edge thinner than a field-line step is drawn as the row's first
-- line).  The denoise window stays in real pixels: 8 at 1080, 4 at 720p/SD
-- (the smallest cell is 4 px wide there).
--
-- A pixelation effect that tiles the screen with a triangular lattice instead of
-- square blocks.  Each cell is Wc wide x H = Wc*7/4 tall (~equilateral; the edge
-- slope is lx*7/4 ~= lx*sqrt(3), giving a ~59.4-degree apex) and is split by ONE
-- diagonal into two triangles; the diagonal orientation alternates per cell COLUMN
-- AND per ROW (half-offset lattice) so the two triangles of adjacent columns merge
-- into the visual upright (▲) and inverted (▽) triangles, which alternate ▲▽▲▽
-- across every row and meet flat-base-to-flat-base between rows.  Each triangle is
-- flat-filled with a colour sampled from the incoming live video: an 8-px average
-- on the line just above its cell row (the last line of the row above), held down
-- the row — the c64/rollthebones memoryless hold, so any cell size works, NO divide.
--
-- The cell decomposition uses the mondrian continuous-scale counters (advance as
-- the raster scans), so P12 gives a VERY smooth 8..256 px size with no divide and
-- no power-of-2 quantisation.  The equilateral edge test is lx*7/4 vs ly (shift-add),
-- so there is no per-pixel multiply in the geometry path.
--
-- Controls:
--   K1 Grid Colour    (black / white / red / orange / yellow / green / blue / violet)
--   K2 Contrast       (fill contrast around mid; 50% = unity)
--   K3 Luma Level     (S8 off: darker triangles lifted to it; S8 on: hidden below it)
--   K4 Colorize Mode  (Original / Duotone / Posterize / Hue Spin)
--   K5 Colorize Param (Original: saturation, 50% = unity | Duotone: colour pair |
--                      Posterize: palette shift, top = grey | Hue Spin: 20..340 deg)
--   K6 Line Width     (1..4 px)
--   S7 Grid Lines     (triangle edges on/off, normal mode)
--   S8 Luma Gate      (show only triangles at or above K3)
--   S9 Fill Mode      (Flat = one colour per ▲/▽ | Faceted = per half-cell)
--   S10 Wireframe     (edges only, no fill)
--   S11 Bypass        (1-cycle raw passthrough, identical to SDK passthru)
--   P12 Triangle Size (smooth 8..256 px)
--
-- Luma Gate / Wireframe matrix:
--   Gate on,  WF off -> only lit triangles: fill + their own grid lines, else black
--   Gate on,  WF on  -> only lit triangles' grid lines (K1), no fill, else black
--   Gate off, WF on  -> full mesh, every edge coloured by the LIVE incoming video
--   Gate off, WF off -> normal fill (+ optional K1 grid)
-- Gating is per-triangle (own luma) so only COMPLETE lit triangles draw.
--
-- Fixed colours are stored with U/V (Cb/Cr) SWAPPED to match the Videomancer
-- hardware output convention (stored U = Cr, stored V = Cb).  The grid palette is
-- the scope-verified ROYGBIV table.
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

architecture triangulator of program_top is
    -- data_in -> data_out latency is 12 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;

    constant LATENCY  : natural := 11;
    constant C_MAXCOL : natural := 256;                       -- max cell columns (small triangles)

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLACK : unsigned(9 downto 0) := to_unsigned( 64, 10);

    --------------------------------------------------------------------------
    -- Colorize tables (U/V stored swapped: stored U = Cr, stored V = Cb).
    --------------------------------------------------------------------------
    -- Posterize: 8-entry rainbow indexed by luma zone (top 3 bits).
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant RNB_Y : t_pal8 := (
        to_unsigned(320,10), to_unsigned(460,10), to_unsigned(720,10), to_unsigned(560,10),
        to_unsigned(650,10), to_unsigned(300,10), to_unsigned(360,10), to_unsigned(900,10));
    constant RNB_U : t_pal8 := (   -- = Cr
        to_unsigned(850,10), to_unsigned(740,10), to_unsigned(620,10), to_unsigned(340,10),
        to_unsigned(290,10), to_unsigned(410,10), to_unsigned(620,10), to_unsigned(512,10));
    constant RNB_V : t_pal8 := (   -- = Cb
        to_unsigned(410,10), to_unsigned(360,10), to_unsigned(170,10), to_unsigned(300,10),
        to_unsigned(560,10), to_unsigned(760,10), to_unsigned(760,10), to_unsigned(512,10));

    -- Posterize: 8-step grey ramp.
    constant GRY_Y : t_pal8 := (
        to_unsigned( 64,10), to_unsigned(180,10), to_unsigned(300,10), to_unsigned(430,10),
        to_unsigned(560,10), to_unsigned(690,10), to_unsigned(820,10), to_unsigned(950,10));

    -- Grid-line colour (K1): black, white, then the scope-verified R O Y G B V.
    constant GRID_Y : t_pal8 := (
        to_unsigned( 64,10), to_unsigned(940,10), to_unsigned(328,10), to_unsigned(584,10),
        to_unsigned(840,10), to_unsigned(580,10), to_unsigned(164,10), to_unsigned(349,10));
    constant GRID_U : t_pal8 := (   -- = Cr
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(960,10), to_unsigned(772,10),
        to_unsigned(584,10), to_unsigned(136,10), to_unsigned(440,10), to_unsigned(756,10));
    constant GRID_V : t_pal8 := (   -- = Cb
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(360,10), to_unsigned(212,10),
        to_unsigned( 64,10), to_unsigned(216,10), to_unsigned(960,10), to_unsigned(852,10));

    -- Duotone: 4 shadow->highlight pairs.
    type t_duo4 is array(0 to 3) of unsigned(9 downto 0);
    constant DUO_LO_Y : t_duo4 := (to_unsigned( 80,10), to_unsigned(120,10), to_unsigned(120,10), to_unsigned(200,10));
    constant DUO_LO_U : t_duo4 := (to_unsigned(470,10), to_unsigned(470,10), to_unsigned(650,10), to_unsigned(360,10));
    constant DUO_LO_V : t_duo4 := (to_unsigned(590,10), to_unsigned(720,10), to_unsigned(680,10), to_unsigned(320,10));
    constant DUO_HI_Y : t_duo4 := (to_unsigned(900,10), to_unsigned(950,10), to_unsigned(820,10), to_unsigned(850,10));
    constant DUO_HI_U : t_duo4 := (to_unsigned(590,10), to_unsigned(512,10), to_unsigned(600,10), to_unsigned(680,10));
    constant DUO_HI_V : t_duo4 := (to_unsigned(430,10), to_unsigned(512,10), to_unsigned(180,10), to_unsigned(560,10));

    -- Hue Spin: chroma rotation as a 3-multiply complex product (u + iv)(c + is):
    --   k1 = c*(u+v), k2 = (s-c)*u, k3 = (c+s)*v;  u' = k1 - k3, v' = k1 + k2.
    -- 64 angles over 20..340 deg (never the identity), coefficients x256.
    type t_hue64 is array(0 to 63) of signed(10 downto 0);
    constant HUE_C : t_hue64 := (
        to_signed(  241,11), to_signed(  232,11), to_signed(  221,11), to_signed(  209,11), to_signed(  195,11), to_signed(  180,11), to_signed(  163,11), to_signed(  145,11),
        to_signed(  126,11), to_signed(  105,11), to_signed(   84,11), to_signed(   62,11), to_signed(   40,11), to_signed(   18,11), to_signed(   -5,11), to_signed(  -28,11),
        to_signed(  -50,11), to_signed(  -72,11), to_signed(  -94,11), to_signed( -114,11), to_signed( -134,11), to_signed( -153,11), to_signed( -170,11), to_signed( -187,11),
        to_signed( -201,11), to_signed( -215,11), to_signed( -226,11), to_signed( -236,11), to_signed( -244,11), to_signed( -250,11), to_signed( -254,11), to_signed( -256,11),
        to_signed( -256,11), to_signed( -254,11), to_signed( -250,11), to_signed( -244,11), to_signed( -236,11), to_signed( -226,11), to_signed( -215,11), to_signed( -201,11),
        to_signed( -187,11), to_signed( -170,11), to_signed( -153,11), to_signed( -134,11), to_signed( -114,11), to_signed(  -94,11), to_signed(  -72,11), to_signed(  -50,11),
        to_signed(  -28,11), to_signed(   -5,11), to_signed(   18,11), to_signed(   40,11), to_signed(   62,11), to_signed(   84,11), to_signed(  105,11), to_signed(  126,11),
        to_signed(  145,11), to_signed(  163,11), to_signed(  180,11), to_signed(  195,11), to_signed(  209,11), to_signed(  221,11), to_signed(  232,11), to_signed(  241,11));
    constant HUE_SMC : t_hue64 := (   -- (sin - cos) x256
        to_signed( -153,11), to_signed( -123,11), to_signed(  -93,11), to_signed(  -61,11), to_signed(  -30,11), to_signed(    3,11), to_signed(   35,11), to_signed(   66,11),
        to_signed(   98,11), to_signed(  128,11), to_signed(  158,11), to_signed(  186,11), to_signed(  213,11), to_signed(  238,11), to_signed(  261,11), to_signed(  282,11),
        to_signed(  301,11), to_signed(  318,11), to_signed(  332,11), to_signed(  343,11), to_signed(  352,11), to_signed(  358,11), to_signed(  361,11), to_signed(  362,11),
        to_signed(  359,11), to_signed(  354,11), to_signed(  346,11), to_signed(  335,11), to_signed(  322,11), to_signed(  306,11), to_signed(  288,11), to_signed(  267,11),
        to_signed(  244,11), to_signed(  220,11), to_signed(  193,11), to_signed(  166,11), to_signed(  136,11), to_signed(  106,11), to_signed(   75,11), to_signed(   44,11),
        to_signed(   12,11), to_signed(  -21,11), to_signed(  -52,11), to_signed(  -84,11), to_signed( -115,11), to_signed( -145,11), to_signed( -174,11), to_signed( -201,11),
        to_signed( -227,11), to_signed( -251,11), to_signed( -273,11), to_signed( -293,11), to_signed( -311,11), to_signed( -326,11), to_signed( -339,11), to_signed( -349,11),
        to_signed( -356,11), to_signed( -360,11), to_signed( -362,11), to_signed( -361,11), to_signed( -357,11), to_signed( -350,11), to_signed( -340,11), to_signed( -328,11));
    constant HUE_SPC : t_hue64 := (   -- (cos + sin) x256
        to_signed(  328,11), to_signed(  340,11), to_signed(  350,11), to_signed(  357,11), to_signed(  361,11), to_signed(  362,11), to_signed(  360,11), to_signed(  356,11),
        to_signed(  349,11), to_signed(  339,11), to_signed(  326,11), to_signed(  311,11), to_signed(  293,11), to_signed(  273,11), to_signed(  251,11), to_signed(  227,11),
        to_signed(  201,11), to_signed(  174,11), to_signed(  145,11), to_signed(  115,11), to_signed(   84,11), to_signed(   52,11), to_signed(   21,11), to_signed(  -12,11),
        to_signed(  -44,11), to_signed(  -75,11), to_signed( -106,11), to_signed( -136,11), to_signed( -166,11), to_signed( -193,11), to_signed( -220,11), to_signed( -244,11),
        to_signed( -267,11), to_signed( -288,11), to_signed( -306,11), to_signed( -322,11), to_signed( -335,11), to_signed( -346,11), to_signed( -354,11), to_signed( -359,11),
        to_signed( -362,11), to_signed( -361,11), to_signed( -358,11), to_signed( -352,11), to_signed( -343,11), to_signed( -332,11), to_signed( -318,11), to_signed( -301,11),
        to_signed( -282,11), to_signed( -261,11), to_signed( -238,11), to_signed( -213,11), to_signed( -186,11), to_signed( -158,11), to_signed( -128,11), to_signed(  -98,11),
        to_signed(  -66,11), to_signed(  -35,11), to_signed(   -3,11), to_signed(   30,11), to_signed(   61,11), to_signed(   93,11), to_signed(  123,11), to_signed(  153,11));

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then               return to_unsigned(0, 10);
        elsif v > 1023 then         return to_unsigned(1023, 10);
        else                        return unsigned(resize(v, 10));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID
    -- (registers_in(8)); 1.0 at 1080p, so 1080p is unchanged.
    --------------------------------------------------------------------------
    -- x step in half virtual units per pixel: 1080 2 (1.0), 720p 3 (1.5),
    -- SD 4 (2.0: 720 px span 1440 units, a 4:3 window of the 1080 canvas).
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y step in Q4.8 virtual units per FIELD line (1080 / active lines,
    -- doubled for interlaced modes).
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);  -- 480i: 2 x 1080/486
            when 4          => return to_unsigned(576, 12);   -- 480p: 1080/480
            when 8          => return to_unsigned(960, 12);   -- 576i: 2 x 1080/576
            when 12         => return to_unsigned(480, 12);   -- 576p: 1080/576
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p: 1.5
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i: 2.0
            when others     => return to_unsigned(256, 12);   -- 1080p: 1.0
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Position (continuous-scale cell counters: advance as the raster scans ->
    -- arbitrary cell sizes with no divide/multiply).
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    -- Resolution scaling (v1.0.2): the lattice runs in virtual 1080-line
    -- units.  x: a Q.1 accumulator (half units; step 2/3/4 = 1.0/1.5/2.0 per
    -- pixel), cellx_la(12 downto 1) is the old integer lx.  y: celly_loc still
    -- counts whole virtual lines, but each field line advances it by the
    -- integer part of a Q4.8 accumulator (1 at 1080p, 1-2 at 720p, 4-5 at
    -- 480i), one unit step per clock after the h edge.
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal cellx_la : unsigned(12 downto 0) := (others => '0');   -- lx within cell, Q.1
    signal x_first  : std_logic := '1';                           -- this pixel starts a cell
    signal x_ws     : std_logic := '0';                           -- this cell's window started
    signal wcnt     : unsigned(2 downto 0) := (others => '0');    -- window pixels still due
    signal cy_frac  : unsigned(7 downto 0) := (others => '0');
    signal cy_left  : unsigned(2 downto 0) := (others => '0');    -- unit steps still due
    signal cy_blw   : std_logic := '1';                           -- previous line short of H - k
    signal cy_rf    : std_logic := '0';                           -- a row started on this line
    signal cellx_idx  : unsigned(7 downto 0)  := (others => '0');   -- cell column index
    signal celly_loc  : unsigned(11 downto 0) := (others => '0');   -- ly within cell
    signal celly_par  : std_logic := '0';                           -- cell-row parity (half-offset + bank)
    signal celly_last : std_logic := '0';                           -- on the last line of a cell row
    signal frame_act  : std_logic := '0';                           -- seen active video this frame
    signal line0      : std_logic := '0';                           -- frame's first active line (pending from vsync)

    --------------------------------------------------------------------------
    -- Next-frame parameters, computed free-running (one small op per stage, so
    -- the vsync latch is a plain register copy and no SPI read sits in its cone).
    --------------------------------------------------------------------------
    signal k_held  : unsigned(9 downto 0)  := to_unsigned(160, 10); -- P12 after hysteresis
    signal n_csz   : unsigned(11 downto 0) := to_unsigned(47, 12);  -- cell width Wc
    signal n_wlim  : unsigned(12 downto 0) := to_unsigned(92, 13);  -- 2Wc - x step (wrap)
    signal n_cshmk : unsigned(11 downto 0) := to_unsigned(81, 12);  -- H - ceil(y step)
    signal n_k     : unsigned(3 downto 0)  := to_unsigned(1, 4);    -- ceil(y step)
    signal n_wspan : unsigned(4 downto 0)  := to_unsigned(16, 5);   -- window span, half units
    signal n_lw2   : unsigned(11 downto 0) := to_unsigned(1, 12);   -- line widths after the
    signal n_wt2   : unsigned(11 downto 0) := to_unsigned(1, 12);   -- per-mode visibility clamps
    signal n_wb2   : unsigned(11 downto 0) := to_unsigned(0, 12);
    signal n_hf    : std_logic := '0';
    signal n_csh   : unsigned(11 downto 0) := to_unsigned(82, 12);  -- row height H = Wc*7/4
    signal n_cshm1 : unsigned(11 downto 0) := to_unsigned(81, 12);
    signal n_wst   : unsigned(12 downto 0) := to_unsigned(38, 13);  -- sample window start (half units)
    signal n_line_w  : unsigned(11 downto 0) := to_unsigned(1, 12);
    signal n_line_wt : unsigned(11 downto 0) := to_unsigned(1, 12);
    signal n_line_wb : unsigned(11 downto 0) := to_unsigned(0, 12);

    -- per-frame latched geometry / switches (copied at vsync)
    signal s_wlim    : unsigned(12 downto 0) := to_unsigned(92, 13);
    signal s_csz_hmk : unsigned(11 downto 0) := to_unsigned(81, 12);
    signal s_hforce  : std_logic := '0';                             -- horiz edge = row's first line
    signal s_w4      : std_logic := '0';                             -- 4-px window (720p / SD)
    signal s_csz_h   : unsigned(11 downto 0) := to_unsigned(82, 12);
    signal s_csz_hm1 : unsigned(11 downto 0) := to_unsigned(81, 12);
    signal s_wst     : unsigned(12 downto 0) := to_unsigned(38, 13);
    signal s_line_w  : unsigned(11 downto 0) := to_unsigned(1, 12);  -- grid line width
    signal s_line_wt : unsigned(11 downto 0) := to_unsigned(1, 12);  -- horiz band, top of row
    signal s_line_wb : unsigned(11 downto 0) := to_unsigned(0, 12);  -- horiz band, bottom of row
    signal s_grid   : std_logic := '1';
    signal s_gate   : std_logic := '0';                            -- S8 luma gate
    signal s_fill   : std_logic := '0';                            -- 0 faceted, 1 flat
    signal s_wire   : std_logic := '0';                            -- S10: grid only, no fill
    signal s_bypass : std_logic := '0';                            -- S11

    -- colour parameters (free-running; a mid-frame change is invisible)
    signal a_cmode : unsigned(1 downto 0) := (others => '0');
    signal a_dlo_y, a_dlo_u, a_dlo_v : unsigned(9 downto 0) := C_MID;
    signal a_dhi_y, a_dhi_u, a_dhi_v : unsigned(9 downto 0) := C_MID;
    signal a_hc, a_hsm, a_hsp        : signed(10 downto 0) := (others => '0');
    signal a_sat   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal a_prot  : unsigned(2 downto 0) := (others => '0');
    signal a_k3a, a_k3q : unsigned(9 downto 0) := (others => '0');
    signal a_k3z   : std_logic := '1';
    signal a_gate  : std_logic := '0';
    signal b_lev   : unsigned(9 downto 0) := to_unsigned(64, 10);

    signal s_lgain   : unsigned(7 downto 0) := to_unsigned(128, 8); -- contrast (128 = unity)
    signal s_floor   : unsigned(9 downto 0) := (others => '0');     -- K3 lift (gate off)
    signal s_gate_th : unsigned(9 downto 0) := (others => '0');     -- K3 threshold (gate on)
    signal s_cmode   : unsigned(1 downto 0) := (others => '0');     -- colorize mode
    signal s_a1, s_a2, s_a3 : signed(10 downto 0) := (others => '0'); -- per-mode multiplier coefficients
    signal s_by, s_bu, s_bv : unsigned(9 downto 0) := (others => '0'); -- per-mode output bias
    signal s_prot    : unsigned(2 downto 0) := (others => '0');     -- posterize palette shift
    signal s_pgrey   : std_logic := '0';                            -- posterize grey ramp
    signal s_gy, s_gu, s_gv : unsigned(9 downto 0) := C_MID;        -- grid colour (K1)

    --------------------------------------------------------------------------
    -- Sampling: 8-px average written into a row-parity bank.
    --------------------------------------------------------------------------
    signal acc_y, acc_u, acc_v : unsigned(12 downto 0) := (others => '0');
    signal w_pend  : std_logic := '0';
    signal w_paddr : unsigned(8 downto 0) := (others => '0');
    signal w_done  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Cell colour buffer: 2 banks (row parity) x 256 columns, Y/U/V packed and
    -- padded to 32 bits, 1W1R.  A row reads the bank written on the last line of
    -- the row above, so reads never hit the address being written — except on the
    -- frame's first line (row 0 samples there), which is drawn black.  Hence
    -- no_rw_check: no read-before-write emulation logic.
    --------------------------------------------------------------------------
    type t_cbuf is array(0 to 2 * C_MAXCOL - 1) of std_logic_vector(31 downto 0);
    constant C_BUF_INIT : std_logic_vector(31 downto 0) :=
        "00" & std_logic_vector(C_MID) & std_logic_vector(C_MID) & std_logic_vector(C_MID);
    signal buf : t_cbuf := (others => C_BUF_INIT);
    attribute no_rw_check : boolean;
    attribute no_rw_check of buf : signal is true;
    signal s_we        : std_logic := '0';
    signal s_waddr     : unsigned(8 downto 0) := (others => '0');
    signal s_wdata     : std_logic_vector(31 downto 0) := C_BUF_INIT;
    signal s_buf_raddr : unsigned(8 downto 0) := (others => '0');
    signal s_rd        : std_logic_vector(31 downto 0) := (others => '0');

    -- geo0: pre-registered lx*7/4 (+ aligned ly/cx/parity) so p_comb's diagonal
    -- chain stays short enough to close HD timing (the *7/4 adds would otherwise
    -- sit in the same combinational cone as the diagonal subtract + grid compare).
    signal g0_lxs  : unsigned(10 downto 0) := (others => '0');  -- lx*7/4, <= 446
    signal g0_ly   : unsigned(11 downto 0) := (others => '0');
    signal g0_cx   : unsigned(7 downto 0)  := (others => '0');
    signal g0_even : std_logic := '0';                         -- '1' = "/" column
    signal g0_bank : std_logic := '0';                         -- buffer bank this row reads
    signal g0_top  : std_logic := '0';                         -- frame's first active line
    signal g0_rf   : std_logic := '0';                         -- a row started on this line

    -- combinational geometry outputs
    signal c_raddr : unsigned(7 downto 0) := (others => '0');
    signal c_grid  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    signal g_pipe    : std_logic_vector(0 to 8) := (others => '0'); -- grid flag
    signal top_pipe  : std_logic_vector(0 to 8) := (others => '0'); -- frame top line
    signal lown_pipe : std_logic_vector(0 to 6) := (others => '0'); -- own-triangle lit (gates fill + own edges)

    signal samp_y, samp_u, samp_v : unsigned(9 downto 0) := C_MID;          -- St1
    signal s2_lp                  : signed(20 downto 0) := (others => '0'); -- St2 luma product
    signal s2_cu, s2_cv           : signed(10 downto 0) := (others => '0'); -- St2 centred chroma
    signal s3_lm0                 : unsigned(9 downto 0) := (others => '0'); -- St3 contrast luma
    signal s3_cs                  : signed(11 downto 0) := (others => '0'); -- St3 cu + cv
    signal s3_cu, s3_cv           : signed(10 downto 0) := (others => '0');
    signal s3b_lm                 : unsigned(9 downto 0) := (others => '0'); -- St3b lifted luma
    signal s3b_b1, s3b_b2, s3b_b3 : signed(11 downto 0) := (others => '0'); -- St3b mult operands
    signal s4a_h1, s4a_h2, s4a_h3 : signed(16 downto 0) := (others => '0'); -- St4a A x B(11..6)
    signal s4a_l1, s4a_l2, s4a_l3 : signed(17 downto 0) := (others => '0'); -- St4a A x B(5..0)
    signal s4a_lm                 : unsigned(9 downto 0) := (others => '0');
    signal s4a_pidx               : unsigned(2 downto 0) := (others => '0');
    signal s4_p1, s4_p2, s4_p3    : signed(22 downto 0) := (others => '0'); -- St4b products
    signal s4_lm                  : unsigned(9 downto 0) := (others => '0');
    signal s4_pidx                : unsigned(2 downto 0) := (others => '0'); -- posterize index
    signal s5t_y, s5t_u, s5t_v    : signed(12 downto 0) := (others => '0'); -- St5 pre-bias
    signal s5_y, s5_u, s5_v       : unsigned(9 downto 0) := C_MID;          -- St5b colorized
    signal s6_y, s6_u, s6_v       : unsigned(9 downto 0) := C_MID;          -- St6 grid overlay
    signal s6_vid                 : std_logic := '0';                       -- St6: colour this edge from live video

    signal s_io    : t_video_stream_yuv444_30b;
    signal s_bpass : t_video_stream_yuv444_30b;   -- 1-cycle raw passthrough (== SDK passthru)

begin

    --------------------------------------------------------------------------
    -- Next-frame parameters (free-running pipeline from the SPI registers).
    --------------------------------------------------------------------------
    p_params : process(clk)
        variable v_dk  : signed(10 downto 0);
        variable v_x   : unsigned(11 downto 0);
        variable v_k3  : unsigned(9 downto 0);
        variable v_i2  : integer range 0 to 3;
        variable v_i6  : integer range 0 to 63;
        variable v_i3  : integer range 0 to 7;
        variable v_lw  : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            -- P12 Triangle Size, with hysteresis: the lattice is anchored at the left
            -- edge, so 1 px of cell width moves column k by k px — pot noise flipping
            -- one size step would shake the right side of the frame by up to ~240 px.
            v_dk := signed('0' & registers_in(7)) - signed('0' & k_held);
            if v_dk > 3 or v_dk < -3 then
                k_held <= unsigned(registers_in(7));
            end if;
            -- Wc = 8 + x - x/32 (x = 0..255) -> 8..256 px with no dead zone at the top.
            v_x   := resize(k_held(9 downto 2), 12);
            n_csz <= to_unsigned(8, 12) + v_x - shift_right(v_x, 5);
            -- Equilateral row height H = Wc*7/4 (~Wc*sqrt(3); apex ~59.4 deg).
            n_csh   <= n_csz + shift_right(n_csz, 1) + shift_right(n_csz, 2);
            n_cshm1 <= n_csh - 1;
            -- (virtual units) cell wrap threshold 2Wc - x step; sample-line
            -- threshold H - ceil(y step)
            n_wlim  <= (n_csz & '0') - s_xs;
            if s_ys(7 downto 0) /= 0 then
                n_k <= s_ys(11 downto 8) + 1;
            else
                n_k <= s_ys(11 downto 8);
            end if;
            n_cshmk <= n_csh - n_k;
            -- Sample window: 8 REAL px at 1080 (a filter kernel stays at pixel
            -- scale), 4 px at 720p / SD where the smallest cell (8 units) is
            -- only 5.3 / 4 px wide.  n_wspan = its span in half units.
            -- Flat: both triangles that read a cell have their axis on its
            -- RIGHT edge, so the window ends there.  Faceted: cell centre.
            if s_xs = 2 then
                n_wspan <= to_unsigned(16, 5);
            else
                n_wspan <= s_xs & "00";
            end if;
            if registers_in(6)(2) = '1' then
                n_wst  <= (n_csz & '0') - n_wspan;
            else
                n_wst  <= (shift_right(n_csz, 1) & '0') - n_wspan(4 downto 1);
            end if;

            -- K6 Line Width (top 2 bits) -> 1/2/3/4 px.  Horizontal edges are
            -- shared base-to-base between cell rows, so each row draws only HALF
            -- the width (top band wt + bottom band wb of the two adjacent rows sum
            -- to W); the diagonal's [-W,W) band (2W units) is halved back to ~W px
            -- by its ~2.0 edge gradient, so both match at ~W px.
            -- At program load the firmware writes a labelled default as its label
            -- INDEX (0..3), not a knob position, so a raw value < 4 is the index.
            if unsigned(registers_in(5)) < 4 then
                v_lw := to_integer(unsigned(registers_in(5)(1 downto 0)));
            else
                v_lw := to_integer(unsigned(registers_in(5)(9 downto 8)));
            end if;
            case v_lw is
                when 0      => n_line_w  <= to_unsigned(1, 12);
                               n_line_wt <= to_unsigned(1, 12);
                               n_line_wb <= to_unsigned(0, 12);
                when 1      => n_line_w  <= to_unsigned(2, 12);
                               n_line_wt <= to_unsigned(1, 12);
                               n_line_wb <= to_unsigned(1, 12);
                when 2      => n_line_w  <= to_unsigned(3, 12);
                               n_line_wt <= to_unsigned(2, 12);
                               n_line_wb <= to_unsigned(1, 12);
                when others => n_line_w  <= to_unsigned(4, 12);
                               n_line_wt <= to_unsigned(2, 12);
                               n_line_wb <= to_unsigned(2, 12);
            end case;

            -- Line widths are virtual units, clamped so SD/720p sampling can't
            -- skip an edge: the diagonal band [-W, W) moves up to 4 per pixel
            -- at 720p/SD (W >= 2); the two horizontal bands of a shared edge
            -- (wt + wb) must span a whole field-line step, else the edge is
            -- drawn as the row's first line instead (exactly one line).
            if s_xs /= 2 and n_line_w < 2 then
                n_lw2 <= to_unsigned(2, 12);
            else
                n_lw2 <= n_line_w;
            end if;
            if n_line_wt + n_line_wb < n_k then
                n_wt2 <= (others => '0');
                n_wb2 <= (others => '0');
                n_hf  <= '1';
            else
                n_wt2 <= n_line_wt;
                n_wb2 <= n_line_wb;
                n_hf  <= '0';
            end if;

            -- K2 Contrast, always on (512 -> gain 128 = unity).
            s_lgain <= unsigned(registers_in(1)(9 downto 2));

            -- K3 Luma Level = 64 + K3*11/16 (64..767); the bottom 8 counts are 0 so
            -- the knob's bottom is an exact no-op.  Gate off: lift darker fills to
            -- it.  Gate on: hide triangles below it.
            v_k3  := unsigned(registers_in(2));
            a_k3a <= shift_right(v_k3, 1) + shift_right(v_k3, 3);
            a_k3q <= shift_right(v_k3, 4);
            if v_k3(9 downto 3) = 0 then a_k3z <= '1'; else a_k3z <= '0'; end if;
            a_gate <= registers_in(6)(1);
            b_lev  <= to_unsigned(64, 10) + a_k3a + a_k3q;
            if a_k3z = '1' then
                s_gate_th <= (others => '0');
                s_floor   <= (others => '0');
            else
                s_gate_th <= b_lev;
                if a_gate = '1' then s_floor <= (others => '0'); else s_floor <= b_lev; end if;
            end if;

            -- K4 mode + K5 parameter -> per-mode multiplier coefficients and bias.
            v_i2 := to_integer(unsigned(registers_in(4)(9 downto 8)));
            v_i6 := to_integer(unsigned(registers_in(4)(9 downto 4)));
            a_cmode <= unsigned(registers_in(3)(9 downto 8));
            a_dlo_y <= DUO_LO_Y(v_i2);  a_dlo_u <= DUO_LO_U(v_i2);  a_dlo_v <= DUO_LO_V(v_i2);
            a_dhi_y <= DUO_HI_Y(v_i2);  a_dhi_u <= DUO_HI_U(v_i2);  a_dhi_v <= DUO_HI_V(v_i2);
            a_hc    <= HUE_C(v_i6);  a_hsm <= HUE_SMC(v_i6);  a_hsp <= HUE_SPC(v_i6);
            a_sat   <= unsigned(registers_in(4)(9 downto 1));
            a_prot  <= unsigned(registers_in(4)(9 downto 7));

            s_cmode <= a_cmode;
            case to_integer(a_cmode) is
                when 1 =>      -- Duotone: lo + (hi - lo) * luma / 1024
                    s_a1 <= signed(resize(a_dhi_y, 11)) - signed(resize(a_dlo_y, 11));
                    s_a2 <= signed(resize(a_dhi_u, 11)) - signed(resize(a_dlo_u, 11));
                    s_a3 <= signed(resize(a_dhi_v, 11)) - signed(resize(a_dlo_v, 11));
                    s_by <= a_dlo_y;  s_bu <= a_dlo_u;  s_bv <= a_dlo_v;
                when 2 =>      -- Posterize: palette (no products)
                    s_a1 <= (others => '0');  s_a2 <= (others => '0');  s_a3 <= (others => '0');
                    s_by <= (others => '0');  s_bu <= (others => '0');  s_bv <= (others => '0');
                when 3 =>      -- Hue Spin: rotate chroma by the K5 angle
                    s_a1 <= a_hc;  s_a2 <= a_hsm;  s_a3 <= a_hsp;
                    s_by <= (others => '0');  s_bu <= C_MID;  s_bv <= C_MID;
                when others => -- Original: saturation s/256 = the same product with angle 0
                    s_a1 <= signed(resize(a_sat, 11));
                    s_a2 <= -signed(resize(a_sat, 11));
                    s_a3 <= signed(resize(a_sat, 11));
                    s_by <= (others => '0');  s_bu <= C_MID;  s_bv <= C_MID;
            end case;
            -- Posterize: K5 zones 0..6 shift the rainbow, zone 7 = grey ramp.
            if a_prot = 7 then
                s_pgrey <= '1';  s_prot <= (others => '0');
            else
                s_pgrey <= '0';  s_prot <= a_prot;
            end if;

            -- K1 Grid Colour (top 3 bits).
            v_i3 := to_integer(unsigned(registers_in(0)(9 downto 7)));
            s_gy <= GRID_Y(v_i3);  s_gu <= GRID_U(v_i3);  s_gv <= GRID_V(v_i3);
        end if;
    end process p_params;

    --------------------------------------------------------------------------
    -- Position counters, sampling, and the per-frame latch (at vsync edge).
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_sline : boolean;
        variable v_sbank : std_logic;
        variable v_start : boolean;
        variable v_wend  : boolean;
        variable v_wrap  : boolean;
        variable v_step  : std_logic;
        variable v_ysum  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- X cell counter in virtual half units -> continuous cell size, no
            -- divide.  A cell wraps when the NEXT step would reach Wc
            -- (la >= 2Wc - step: at 1080p la = 2(Wc-1), the old lx = Wc-1 test).
            v_wrap := data_in.avid = '1' and cellx_la >= s_wlim;
            if v_h_edge = '1' then
                cellx_la  <= (others => '0');
                cellx_idx <= (others => '0');
                x_first   <= '1';
            elsif data_in.avid = '1' then
                if v_wrap then
                    cellx_la  <= cellx_la - s_wlim;
                    cellx_idx <= cellx_idx + 1;
                    x_first   <= '1';
                else
                    cellx_la  <= cellx_la + s_xs;
                    x_first   <= '0';
                end if;
            end if;

            -- Sample lines: the LAST line of each cell row samples the NEXT row's
            -- colours into the next row's bank, so a row never reads the bank being
            -- written and every line of a row shows that row's colours (rollthebones
            -- scheme).  Row 0 has no row above: it samples on the frame's first
            -- active line into bank 0, and that one line is drawn black.
            v_sline := line0 = '1' or celly_last = '1';
            if line0 = '1' then v_sbank := '0'; else v_sbank := not celly_par; end if;

            -- 8-px horizontal average (4 px at 720p / SD) ending at the window end
            -- (memoryless denoise: a single point sample per triangle flickers with
            -- analog noise).  The window starts at the first pixel past s_wst and
            -- is counted in real pixels (1080p: s_wst .. s_wst+7, as before).
            v_start := data_in.avid = '1' and x_ws = '0' and cellx_la >= s_wst;
            v_wend  := data_in.avid = '1' and wcnt = 1;
            if v_start then
                acc_y <= resize(unsigned(data_in.y), 13);
                acc_u <= resize(unsigned(data_in.u), 13);
                acc_v <= resize(unsigned(data_in.v), 13);
            else
                acc_y <= acc_y + unsigned(data_in.y);
                acc_u <= acc_u + unsigned(data_in.u);
                acc_v <= acc_v + unsigned(data_in.v);
            end if;
            if v_h_edge = '1' then
                x_ws <= '0';
                wcnt <= (others => '0');
            elsif v_start then
                x_ws <= '1';
                if s_w4 = '1' then wcnt <= to_unsigned(3, 3); else wcnt <= to_unsigned(7, 3); end if;
            elsif data_in.avid = '1' and wcnt /= 0 then
                wcnt <= wcnt - 1;
            end if;
            if v_wrap then x_ws <= '0'; end if;              -- next cell

            -- Window closes on this pixel: write the average next cycle.
            w_pend <= '0';
            if v_sline and v_wend then
                w_pend  <= '1';
                w_paddr <= v_sbank & cellx_idx;
            end if;
            -- Has the current cell's sample been taken?
            if data_in.avid = '1' then
                if v_wend then
                    w_done <= '1';
                elsif x_first = '1' then
                    w_done <= '0';
                end if;
            end if;

            -- Buffer write port (the RAM's single write statement is in p_buf).
            s_we <= '0';
            if w_pend = '1' then
                s_we    <= '1';
                s_waddr <= w_paddr;
                if s_w4 = '1' then
                    s_wdata <= "00" & std_logic_vector(acc_y(11 downto 2))
                                    & std_logic_vector(acc_u(11 downto 2))
                                    & std_logic_vector(acc_v(11 downto 2));
                else
                    s_wdata <= "00" & std_logic_vector(acc_y(12 downto 3))
                                    & std_logic_vector(acc_u(12 downto 3))
                                    & std_logic_vector(acc_v(12 downto 3));
                end if;
            elsif v_sline and data_in.avid = '0' and pipe(0).avid = '1' and w_done = '0' then
                -- The line ended inside a partial last cell before its window
                -- closed: point-sample the last active pixel so the right-edge
                -- strip doesn't show a stale row.
                s_we    <= '1';
                s_waddr <= v_sbank & cellx_idx;
                s_wdata <= "00" & pipe(0).y & pipe(0).u & pipe(0).v;
            end if;

            -- Sample line = the first line of a row at/after H - ceil(y step)
            -- (at 1080p the last line, H-1); still inside the row, since a
            -- line advances at most ceil(y step).
            if celly_loc >= s_csz_hmk and cy_blw = '1' then celly_last <= '1'; else celly_last <= '0'; end if;

            -- Y row counter: each h edge adds the Q4.8 field-line step; its
            -- integer part runs as that many unit steps (the first at the edge
            -- itself, the rest on the following clocks, long before active video).
            v_step := '0';
            if v_v_edge = '1' then
                celly_loc <= (others => '0');
                celly_par <= '0';
                frame_act <= '0';
                line0     <= '1';
                cy_left   <= (others => '0');

                -- per-mode virtual steps (static per mode)
                s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

                -- Per-frame geometry + switches (pre-computed in p_params).
                s_wlim    <= n_wlim;
                s_csz_h   <= n_csh;
                s_csz_hm1 <= n_cshm1;
                s_csz_hmk <= n_cshmk;
                s_wst     <= n_wst;
                if s_xs = 2 then s_w4 <= '0'; else s_w4 <= '1'; end if;
                s_line_w  <= n_lw2;
                s_line_wt <= n_wt2;
                s_line_wb <= n_wb2;
                s_hforce  <= n_hf;

                s_grid   <= registers_in(6)(0);   -- S7 Grid Lines
                s_gate   <= registers_in(6)(1);   -- S8 Luma Gate
                s_fill   <= registers_in(6)(2);   -- S9 Fill Mode
                s_wire   <= registers_in(6)(3);   -- S10 Wireframe (grid only)
                s_bypass <= registers_in(6)(4);   -- S11 Bypass
            elsif data_in.avid = '1' and frame_act = '0' then
                -- Anchor the vertical cell grid to the FIRST active line of the frame
                -- (celly_loc increments on hsync, incl. the vertical back porch).
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_par <= '0';
                cy_frac   <= (others => '0');
                cy_left   <= (others => '0');
            elsif v_h_edge = '1' then
                if frame_act = '1' then line0 <= '0'; end if;
                v_ysum  := resize(cy_frac, 12) + s_ys;
                cy_frac <= v_ysum(7 downto 0);
                cy_left <= v_ysum(10 downto 8) - 1;
                if celly_loc < s_csz_hmk then cy_blw <= '1'; else cy_blw <= '0'; end if;
                cy_rf   <= '0';
                v_step  := '1';
            elsif cy_left /= 0 then
                cy_left <= cy_left - 1;
                v_step  := '1';
            end if;
            if v_step = '1' then
                if celly_loc = s_csz_hm1 then
                    celly_loc <= (others => '0');
                    celly_par <= not celly_par;   -- flip per row: half-offsets the lattice
                    cy_rf     <= '1';
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Combinational geometry: triangle classification, read address, grid flag.
    --------------------------------------------------------------------------
    p_comb : process(g0_lxs, g0_ly, g0_cx, g0_even, s_csz_h, s_csz_hm1,
                     s_line_w, s_line_wt, s_line_wb, s_fill, s_hforce, g0_rf)
        variable v_d     : signed(12 downto 0);
        variable v_side  : std_logic;
    begin
        -- g0_lxs is already lx*7/4 (registered): equilateral edge slope.
        if g0_even = '1' then             -- "/" edge (lx*7/4 + ly = H)
            v_d := signed(resize(g0_lxs, 13)) + signed(resize(g0_ly, 13))
                 - signed(resize(s_csz_h, 13));
        else                              -- "\" edge (lx*7/4 = ly)
            v_d := signed(resize(g0_lxs, 13)) - signed(resize(g0_ly, 13));
        end if;

        if v_d >= 0 then v_side := '1'; else v_side := '0'; end if;

        -- read address: faceted = own cell; flat = the triangle's LEFT cell
        -- (side 1 -> this cell is the left cell; side 0 -> the cell to the left).
        if s_fill = '1' then
            if v_side = '1' then
                c_raddr <= g0_cx;
            elsif g0_cx = 0 then
                c_raddr <= (others => '0');
            else
                c_raddr <= g0_cx - 1;
            end if;
        else
            c_raddr <= g0_cx;
        end if;

        -- grid line: on the active diagonal, or near the TOP (▽ base) or BOTTOM
        -- (▲ base) of the cell row, so every triangle owns all three of its edges.
        -- The diagonal takes the full [-W,W) band (its ~2.0 gradient halves it to
        -- ~W px perpendicular); the horizontal bands are split wt/wb across the
        -- two rows sharing the edge so the combined line is also ~W px.
        -- Vertical cell edges are NOT triangle edges.
        if (v_d >= -signed(resize(s_line_w, 13)) and v_d < signed(resize(s_line_w, 13)))
           or (g0_ly < s_line_wt)
           or (g0_ly > s_csz_hm1 - s_line_wb)
           or (s_hforce = '1' and g0_rf = '1') then
            c_grid <= '1';
        else
            c_grid <= '0';
        end if;
    end process p_comb;

    s_buf_raddr <= g0_bank & c_raddr;

    --------------------------------------------------------------------------
    -- Cell colour buffer (1W1R, registered read).
    --------------------------------------------------------------------------
    p_buf : process(clk)
    begin
        if rising_edge(clk) then
            if s_we = '1' then
                buf(to_integer(s_waddr)) <= s_wdata;
            end if;
            s_rd <= buf(to_integer(s_buf_raddr));
        end if;
    end process p_buf;

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_lm   : signed(12 downto 0);
        variable v_l    : unsigned(9 downto 0);
        variable v_du, v_dv : signed(23 downto 0);
        variable v_idx  : integer range 0 to 7;
        variable v_b    : signed(11 downto 0);
        variable v_bl   : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line (only the first few taps carry y/u/v; the
            -- rest are pruned to syncs)
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- grid / top-line flag delays (align with the colour path at St6)
            g_pipe(0) <= c_grid;
            top_pipe(0) <= g0_top;
            for i in 1 to 8 loop
                g_pipe(i) <= g_pipe(i - 1);
                top_pipe(i) <= top_pipe(i - 1);
            end loop;

            -- geo0: pre-register lx*7/4 with ly/cx/parity aligned, so the diagonal
            -- + grid arithmetic in p_comb is a short combinational chain.
            g0_lxs  <= resize(cellx_la(12 downto 1), 11) + resize(shift_right(cellx_la(12 downto 1), 1), 11)
                     + resize(shift_right(cellx_la(12 downto 1), 2), 11);
            g0_ly   <= celly_loc;
            g0_cx   <= cellx_idx;
            -- orientation alternates per column AND per row, so every other row is
            -- half-offset and flat triangle edges (▲ base <-> ▽ base) meet.
            g0_even <= not (cellx_idx(0) xor celly_par);
            g0_bank <= celly_par;
            g0_top  <= line0;
            g0_rf   <= cy_rf;

            -- St1: capture the sampled cell colour into fabric (EBR output -> FF).
            samp_y <= unsigned(s_rd(29 downto 20));
            samp_u <= unsigned(s_rd(19 downto 10));
            samp_v <= unsigned(s_rd(9 downto 0));

            -- St2: contrast multiply about mid (isolated); centred chroma; and the
            -- per-pixel "own triangle is lit" gate (flat read), carried to St6 so
            -- only complete lit triangles draw — fill and their own edges.
            s2_lp <= (signed(resize(samp_y, 11)) - to_signed(512, 11))
                   * signed(resize(s_lgain, 10));
            s2_cu <= signed(resize(samp_u, 11)) - to_signed(512, 11);
            s2_cv <= signed(resize(samp_v, 11)) - to_signed(512, 11);
            lown_pipe(0) <= '0';
            if samp_y >= s_gate_th then
                lown_pipe(0) <= '1';
            end if;
            for i in 1 to 6 loop lown_pipe(i) <= lown_pipe(i - 1); end loop;

            -- St3: recombine + clamp luma; chroma sum for the complex product.
            v_lm := resize(shift_right(s2_lp, 7), 13) + to_signed(512, 13);
            if v_lm < 0 then v_lm := (others => '0'); end if;
            if v_lm > 1023 then v_lm := to_signed(1023, 13); end if;
            s3_lm0 <= unsigned(v_lm(9 downto 0));
            s3_cs  <= resize(s2_cu, 12) + resize(s2_cv, 12);
            s3_cu  <= s2_cu;
            s3_cv  <= s2_cv;

            -- St3b: K3 lift; select the multiplier operands for the mode
            -- (Duotone: luma x3 | Original/Hue Spin: cu+cv, cu, cv).
            if s3_lm0 < s_floor then v_l := s_floor; else v_l := s3_lm0; end if;
            s3b_lm <= v_l;
            if s_cmode = 1 then
                s3b_b1 <= signed(resize(v_l, 12));
                s3b_b2 <= signed(resize(v_l, 12));
                s3b_b3 <= signed(resize(v_l, 12));
            else
                s3b_b1 <= s3_cs;
                s3b_b2 <= resize(s3_cu, 12);
                s3b_b3 <= resize(s3_cv, 12);
            end if;

            -- St4a: the three products, operand B split hi/lo (B = hi*64 + lo, hi
            -- signed, lo unsigned): two 11x6 partial products per channel instead
            -- of one 11x12 — the single-stage 11x12 was the HD HDMI critical path.
            v_b := s3b_b1;
            s4a_h1 <= s_a1 * v_b(11 downto 6);
            v_bl := signed('0' & std_logic_vector(v_b(5 downto 0)));
            s4a_l1 <= s_a1 * v_bl;
            v_b := s3b_b2;
            s4a_h2 <= s_a2 * v_b(11 downto 6);
            v_bl := signed('0' & std_logic_vector(v_b(5 downto 0)));
            s4a_l2 <= s_a2 * v_bl;
            v_b := s3b_b3;
            s4a_h3 <= s_a3 * v_b(11 downto 6);
            v_bl := signed('0' & std_logic_vector(v_b(5 downto 0)));
            s4a_l3 <= s_a3 * v_bl;
            s4a_lm   <= s3b_lm;
            s4a_pidx <= s3b_lm(9 downto 7) + s_prot;

            -- St4b: recombine (resize BEFORE the shift).
            s4_p1   <= shift_left(resize(s4a_h1, 23), 6) + resize(s4a_l1, 23);
            s4_p2   <= shift_left(resize(s4a_h2, 23), 6) + resize(s4a_l2, 23);
            s4_p3   <= shift_left(resize(s4a_h3, 23), 6) + resize(s4a_l3, 23);
            s4_lm   <= s4a_lm;
            s4_pidx <= s4a_pidx;

            -- St5: assemble per mode (pre-bias, signed).
            case to_integer(s_cmode) is
                when 1 =>      -- Duotone: (hi - lo) * luma / 1024 (+ lo in St5b)
                    s5t_y <= resize(shift_right(s4_p1, 10), 13);
                    s5t_u <= resize(shift_right(s4_p2, 10), 13);
                    s5t_v <= resize(shift_right(s4_p3, 10), 13);
                when 2 =>      -- Posterize: luma zone -> palette
                    v_idx := to_integer(s4_pidx);
                    if s_pgrey = '1' then
                        s5t_y <= signed(resize(GRY_Y(v_idx), 13));
                        s5t_u <= signed(resize(C_MID, 13));
                        s5t_v <= signed(resize(C_MID, 13));
                    else
                        s5t_y <= signed(resize(RNB_Y(v_idx), 13));
                        s5t_u <= signed(resize(RNB_U(v_idx), 13));
                        s5t_v <= signed(resize(RNB_V(v_idx), 13));
                    end if;
                when others => -- Original / Hue Spin: u' = k1 - k3, v' = k1 + k2 (+512 in St5b)
                    v_du := resize(s4_p1, 24) - resize(s4_p3, 24);
                    v_dv := resize(s4_p1, 24) + resize(s4_p2, 24);
                    s5t_y <= signed(resize(s4_lm, 13));
                    s5t_u <= resize(shift_right(v_du, 8), 13);
                    s5t_v <= resize(shift_right(v_dv, 8), 13);
            end case;

            -- St5b: per-mode bias + clamp.
            s5_y <= clamp10(resize(s5t_y, 14) + signed(resize(s_by, 14)));
            s5_u <= clamp10(resize(s5t_u, 14) + signed(resize(s_bu, 14)));
            s5_v <= clamp10(resize(s5t_v, 14) + signed(resize(s_bv, 14)));

            -- St6: grid / wireframe / luma-gate overlay.  Edge = g_pipe(8); each
            -- pixel is gated by its OWN triangle's luma (lown_pipe), so only complete
            -- lit triangles draw (fill + their own edges) — no partial outlines.
            -- s6_vid marks edges to be coloured from the live video at the output.
            s6_vid <= '0';
            if top_pipe(8) = '1' then
                -- frame's first active line: row 0's bank is being written (EBR
                -- read-during-write is undefined) -> black
                s6_y <= C_BLACK; s6_u <= C_MID; s6_v <= C_MID;
            elsif s_wire = '1' then
                if s_gate = '1' then
                    -- only the lit triangles' edges, K1 colour, over black
                    if g_pipe(8) = '1' and lown_pipe(6) = '1' then
                        s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;
                    else
                        s6_y <= C_BLACK; s6_u <= C_MID; s6_v <= C_MID;
                    end if;
                else
                    -- full mesh, every edge coloured by the live incoming video
                    if g_pipe(8) = '1' then
                        s6_vid <= '1';
                        s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;   -- placeholder (output -> video)
                    else
                        s6_y <= C_BLACK; s6_u <= C_MID; s6_v <= C_MID;
                    end if;
                end if;
            elsif s_gate = '1' then
                -- non-wireframe + luma gate: show only lit triangles (fill) and
                -- their surrounding grid lines; black elsewhere.
                if s_grid = '1' and g_pipe(8) = '1' then
                    if lown_pipe(6) = '1' then
                        s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;
                    else
                        s6_y <= C_BLACK; s6_u <= C_MID; s6_v <= C_MID;
                    end if;
                elsif lown_pipe(6) = '1' then
                    s6_y <= s5_y; s6_u <= s5_u; s6_v <= s5_v;
                else
                    s6_y <= C_BLACK; s6_u <= C_MID; s6_v <= C_MID;
                end if;
            elsif s_grid = '1' and g_pipe(8) = '1' then
                s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;
            else
                s6_y <= s5_y;
                s6_u <= s5_u;
                s6_v <= s5_v;
            end if;

            -- Output: blanking gate (neutral outside avid).  Live-video edges come
            -- from a SHORT tap (pipe(3), 8 px ahead): raw input video carried ~12
            -- deep went green + noisy on HD HDMI (cascade, rollthebones).
            if pipe(LATENCY - 1).avid = '0' then
                s_io.y <= std_logic_vector(C_BLACK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif s6_vid = '1' then
                if pipe(3).avid = '1' then
                    s_io.y <= pipe(3).y;
                    s_io.u <= pipe(3).u;
                    s_io.v <= pipe(3).v;
                else
                    s_io.y <= std_logic_vector(C_BLACK);
                    s_io.u <= std_logic_vector(C_MID);
                    s_io.v <= std_logic_vector(C_MID);
                end if;
            else
                s_io.y <= std_logic_vector(s6_y);
                s_io.u <= std_logic_vector(s6_u);
                s_io.v <= std_logic_vector(s6_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;

            -- S11: 1-cycle raw forward, exactly SDK passthru (data_out <= data_in)
            s_bpass <= data_in;
        end if;
    end process p_pipe;

    -- S11 bypass = SDK passthru (raw input + its own syncs, 1 cycle).
    data_out <= s_bpass when s_bypass = '1' else s_io;

end architecture triangulator;
