-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- quilt.vhd  (v1.0 — per-pixel recursive-subdivision improv quilt)
--
-- v1.0.2 (2026-10-17): resolution scaling.  The picture column/row counters
-- (and the luma map's source column) run in virtual 1080-line units stepped
-- from the video timing ID (registers_in(8)): 1.0 per pixel/line at 1080p,
-- 1.5 at 720p, x 2.0 / y 4.44 per field line at 480i — so cells, pieces,
-- textures, seams and puff keep their size relative to screen height (SD
-- shows the left 1440 units of the 1080p layout, 4:3, square cells stay
-- square).  Equality tests on the counters became crossing flags; the luma
-- map blocks are 32 virtual units (box sum weighted by the step).  Seam and
-- thin texture lines keep at least one SD field line.  1080p is bit-identical;
-- latency unchanged (28).
--
-- v1.0.3 (2026-10-18): fit for the stock SDK's SD Standalone core, same
-- picture bit for bit.  Pure-delay registers removed: the pixel position is
-- no longer carried down the pipeline (each consumer reads a delayed copy of
-- the position counters, p_qpos), the luma map's per-cell dens/hide step
-- lands on S1b's clock instead of being copied down three stages, the leaf's
-- seam/puff/texture features are built two stages earlier (22 bits carried
-- instead of 48), and the three IO-align pad stages sit after S10b (12 bits)
-- instead of after s_io (32 bits).
--
-- Fills the whole screen with a loosely grid-based field of randomly coloured
-- and textured rectangular patches.  Each coarse grid cell is recursively split
-- (horizontally or vertically) into varied sub-rectangles, log-cabin fashion, so
-- patches contain nested sub-patches in a fractal way.  The layout only moves
-- when a control moves or K3 Evolve is turned off centre.
--
-- Method (no per-pixel multiply, no line buffer — modelled on ziggurat):
--   * Every output pixel marches a fixed-depth pipeline.  Stage 1 finds the
--     coarse cell (x>>g) and seeds a per-cell hash.  Then DMAX=4 subdivision
--     levels, each split across THREE shallow stages so HD timing closes:
--       (hash)    re-mix the running seed, decode split orientation + whether to
--                 subdivide (gated by the P12 "density" slider) + an off-centre
--                 jitter direction;
--       (decide)  compute the split coordinate nx = mid +/- jitter (jitter is
--                 bounded below w/2, so nx is always strictly interior — no
--                 clamp needed);
--       (descend) step the running rectangle bounds [x0,x1)x[y0,y1) into the
--                 half the pixel lies in and fold the choice into the seed so
--                 children diverge.
--     All splits are shift/add only.
--   * The leaf rectangle's seed selects a 16-colour palette entry and one of
--     16 embossed procedural textures, rendered from leaf-local coords with
--     shifts/ANDs only.  Optional seam lines border each patch.
--   * Luma Mod reads a per-frame map of 32x32-pixel block averages of the input
--     luma (see the luma-map section below), so every patch's decisions come
--     from one stable, cell-centred value.
--
-- Controls: K1 Grid Scale, K2 Seed (re-randomises the whole layout), K3 Evolve
-- (BIPOLAR — centre freezes the quilt as it stands, right = pieces change in
-- scattered order, left = the change snakes from piece to piece), K4 Threshold
-- (Luma Mod: fully off at 0, then input luma drives per-cell piecing density
-- and hides cells darker than the threshold -> black), K5 Palette Tint, K6
-- Texture Scale, S7 Seams, S8 Relief (Soft / Deep emboss), S9 Grid (Square /
-- Not-Square off-grid overlap), S10 Piecing (Crazy = random split axis / Log
-- Cabin = alternating axis, concentric frames), S11 Palette (Fabric/Rainbow),
-- P12 Density (KEY).  Textures are always on; split jitter is fixed at Medium.
--
-- Output contracts: the program makes its own avid (hsync-parity locked, see
-- p_gavid) and blanks to neutral outside it; drawn luma stays in 64..880.
-- program_top latency is C_PROCESSING_DELAY_CLKS = 28 (a multiple of 4, as the
-- SDK requires): the 24 pixel stages, three IO-align pad stages (after S10b)
-- and the s_io register.
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

architecture quilt of program_top is

    -- 24 pixel stages (+1 S1a2 candidate-hash, +1 S10a2 evolve, +2 S10h
    -- Hilbert) + the 3 IO-align pad stages, which sit after S10b where the
    -- pixel is 12 bits wide instead of after s_io (32 bits).
    constant LATENCY : natural := 27;
    -- data_in -> data_out: LATENCY pipe stages + s_io.  Must match
    -- processing_delay_clks in quilt.toml.
    constant C_PROCESSING_DELAY_CLKS : natural := LATENCY + 1;
    constant DMAX    : natural := 4;          -- subdivision levels
    constant MIN_W   : natural := 8;          -- don't split a span narrower than this
    constant SEAM_W  : natural := 2;          -- seam line half-thickness (px)
    -- texture emboss luma deltas (2-bit shade -> highlight / flat / shadow / deep)
    constant TEX_HI  : natural := 70;         -- highlight (valley between threads)
    constant TEX_D1  : natural := 110;        -- shadow
    constant TEX_D2  : natural := 210;        -- deep shadow (thread core)
    -- drawn luma range: black is 64, and the brightest palette entry (880) is
    -- the ceiling so emboss highlights never run into super-white, which clips
    -- every RGB channel on hardware.
    constant C_YBLK  : natural := 64;
    constant C_YCAP  : natural := 880;

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK   : unsigned(9 downto 0) := to_unsigned(C_YBLK, 10);

    -- Puff (S8 Puffed): luma pulled down toward a patch's edges so it reads as
    -- padded.  Index = 15 at the edge falling to 0 at the puff radius; the
    -- profile is quadratic (150*(t/15)^2) so the middle stays flat and the
    -- shading rolls off steeply into the seam, like a pillow.
    type t_puff is array (0 to 15) of unsigned(7 downto 0);
    constant C_PUFF : t_puff := (
        to_unsigned(  0,8), to_unsigned(  1,8), to_unsigned(  3,8), to_unsigned(  6,8),
        to_unsigned( 11,8), to_unsigned( 17,8), to_unsigned( 24,8), to_unsigned( 33,8),
        to_unsigned( 43,8), to_unsigned( 54,8), to_unsigned( 67,8), to_unsigned( 81,8),
        to_unsigned( 96,8), to_unsigned(113,8), to_unsigned(131,8), to_unsigned(150,8));

    -- per-level hash mixing constants (compile-time)
    type t_lc is array (0 to DMAX - 1) of unsigned(15 downto 0);
    constant C_LC : t_lc := (x"9E37", x"7C15", x"D2B9", x"3F41");

    --------------------------------------------------------------------------
    -- Pipeline node: running rectangle bounds + seed + freeze.  The pixel
    -- coordinates are not carried: each stage that needs them reads its own
    -- delayed copy of the position counters (q_x*/q_y*, see p_qpos).
    --------------------------------------------------------------------------
    type t_node is record
        x0, x1, y0, y1 : unsigned(11 downto 0);
        seed           : unsigned(15 downto 0);
        dens           : unsigned(7 downto 0);   -- per-cell subdivide threshold base
        hide           : std_logic;              -- Luma Mod: cell luma below K4 threshold
        frozen         : std_logic;
    end record;

    -- decide-stage result: split coordinate + "do it" flag.
    type t_dec is record
        nx   : unsigned(11 downto 0);
        doit : std_logic;
    end record;

    --------------------------------------------------------------------------
    -- Two 16-entry YUV palettes (BT.601: U=Cb, V=Cr).  Constant arrays -> LUTs.
    --------------------------------------------------------------------------
    type t_pal is array (0 to 15) of unsigned(9 downto 0);

    -- Fabric: saturated + muted quilt-fabric colours.
    constant FAB_Y : t_pal := (
        to_unsigned(330,10), to_unsigned(600,10), to_unsigned(820,10), to_unsigned(560,10),
        to_unsigned(470,10), to_unsigned(300,10), to_unsigned(360,10), to_unsigned(520,10),
        to_unsigned(720,10), to_unsigned(250,10), to_unsigned(620,10), to_unsigned(430,10),
        to_unsigned(880,10), to_unsigned(180,10), to_unsigned(540,10), to_unsigned(650,10));
    constant FAB_U : t_pal := (
        to_unsigned(400,10), to_unsigned(360,10), to_unsigned(300,10), to_unsigned(400,10),
        to_unsigned(620,10), to_unsigned(760,10), to_unsigned(700,10), to_unsigned(640,10),
        to_unsigned(470,10), to_unsigned(430,10), to_unsigned(450,10), to_unsigned(660,10),
        to_unsigned(500,10), to_unsigned(512,10), to_unsigned(380,10), to_unsigned(640,10));
    constant FAB_V : t_pal := (
        to_unsigned(720,10), to_unsigned(660,10), to_unsigned(560,10), to_unsigned(380,10),
        to_unsigned(360,10), to_unsigned(440,10), to_unsigned(600,10), to_unsigned(700,10),
        to_unsigned(580,10), to_unsigned(680,10), to_unsigned(470,10), to_unsigned(470,10),
        to_unsigned(540,10), to_unsigned(512,10), to_unsigned(640,10), to_unsigned(380,10));

    -- Rainbow: ROYGBIV.  Colours from the hardware-confirmed Phosphor table,
    -- stored in that table's U/V-swapped-for-hardware convention (Cr in U, Cb in
    -- V) — confirmed on hardware for this program.  Sequence R O Y G B I V x16.
    --                R    O    Y    G    B    I    V
    -- Y:           328  584  840  580  164  192  349
    -- U(=Cr):      960  772  584  136  440  608  756
    -- V(=Cb):      360  212   64  216  960  696  852
    constant RNB_Y : t_pal := (
        to_unsigned(328,10), to_unsigned(584,10), to_unsigned(840,10), to_unsigned(580,10),
        to_unsigned(164,10), to_unsigned(192,10), to_unsigned(349,10), to_unsigned(328,10),
        to_unsigned(584,10), to_unsigned(840,10), to_unsigned(580,10), to_unsigned(164,10),
        to_unsigned(192,10), to_unsigned(349,10), to_unsigned(328,10), to_unsigned(580,10));
    constant RNB_U : t_pal := (
        to_unsigned(960,10), to_unsigned(772,10), to_unsigned(584,10), to_unsigned(136,10),
        to_unsigned(440,10), to_unsigned(608,10), to_unsigned(756,10), to_unsigned(960,10),
        to_unsigned(772,10), to_unsigned(584,10), to_unsigned(136,10), to_unsigned(440,10),
        to_unsigned(608,10), to_unsigned(756,10), to_unsigned(960,10), to_unsigned(136,10));
    constant RNB_V : t_pal := (
        to_unsigned(360,10), to_unsigned(212,10), to_unsigned( 64,10), to_unsigned(216,10),
        to_unsigned(960,10), to_unsigned(696,10), to_unsigned(852,10), to_unsigned(360,10),
        to_unsigned(212,10), to_unsigned( 64,10), to_unsigned(216,10), to_unsigned(960,10),
        to_unsigned(696,10), to_unsigned(852,10), to_unsigned(360,10), to_unsigned(216,10));

    -- 4x4 Bayer matrix for the dither texture.  Index = (y[1:0]<<2)|x[1:0].
    type t_bayer is array (0 to 15) of unsigned(3 downto 0);
    constant C_BAYER : t_bayer := (
        to_unsigned( 0,4), to_unsigned( 8,4), to_unsigned( 2,4), to_unsigned(10,4),
        to_unsigned(12,4), to_unsigned( 4,4), to_unsigned(14,4), to_unsigned( 6,4),
        to_unsigned( 3,4), to_unsigned(11,4), to_unsigned( 1,4), to_unsigned( 9,4),
        to_unsigned(15,4), to_unsigned( 7,4), to_unsigned(13,4), to_unsigned( 5,4));

    --------------------------------------------------------------------------
    -- Functions (all shift/xor/add — no multiply).
    --------------------------------------------------------------------------
    function mix16(v : unsigned(9 downto 0); k : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := ("000000" & v) xor k;
        h := h xor shift_left (h, 7);
        h := h xor shift_right(h, 9);
        h := h xor shift_left (h, 8);
        return h;
    end function;

    function init_seed(comp : unsigned(15 downto 0); x0, y0 : unsigned(11 downto 0))
        return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := comp xor (x0(9 downto 2) & y0(9 downto 2));
        h := h xor shift_left (h, 5);
        h := h xor shift_right(h, 7);
        h := h xor shift_left (h, 3);
        return h;
    end function;

    function hash_lvl(seed, sub : unsigned(15 downto 0); lc : unsigned(15 downto 0))
        return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := seed xor sub xor lc;
        h := h xor shift_left (h, 6);
        h := h xor shift_right(h, 9);
        h := h xor shift_left (h, 2);
        return h;
    end function;

    -- One level of a Hilbert-curve index (Snake evolve).  The running state is
    -- the curve orientation so far — s = swap x/y, c = complement both — and
    -- each level emits one base-4 digit and updates the orientation.  Returns
    -- digit(1:0) & s' & c'.  Every output is a 4-input function: one LUT each.
    function hil_step(xb, yb, s, c : std_logic) return std_logic_vector is
        variable rx, ry : std_logic;
        variable r : std_logic_vector(3 downto 0);
    begin
        if s = '1' then rx := yb xor c; ry := xb xor c;
        else            rx := xb xor c; ry := yb xor c; end if;
        r(3) := rx;                          -- digit = 3*rx xor ry
        r(2) := rx xor ry;
        r(1) := s xor (not ry);              -- lower-half quadrants swap...
        r(0) := c xor (rx and not ry);       -- ...and the far one also flips
        return r;
    end function;

    -- Decide stage: compute the split coordinate for the active axis.  Jitter
    -- magnitude is at most 3w/8 < w/2, so nx is always strictly inside (lo,hi)
    -- and no clamp is required.
    -- w and mid (= midpoint) are precomputed in the hash stage.  Magnitudes are
    -- pure shifts of w (no add) so the only carry chain here is nx = mid +/- mag.
    function calc_split(w        : unsigned(11 downto 0);
                        mid      : unsigned(11 downto 0);
                        dosplit  : std_logic;
                        sgn      : std_logic;
                        half     : std_logic;
                        jit_mode : unsigned(1 downto 0)) return t_dec is
        variable mag : unsigned(11 downto 0);
        variable r : t_dec;
    begin
        case jit_mode is
            when "00"   => mag := (others => '0');
            when "01"   => mag := "0000" & w(11 downto 4);                    -- w/16
            when "10"   => mag := "000"  & w(11 downto 3);                    -- w/8
            when others => mag := "00"   & w(11 downto 2);                    -- w/4
        end case;
        if half = '1' then mag := '0' & mag(11 downto 1); end if;
        if sgn = '1' then r.nx := mid - mag;
        else              r.nx := mid + mag; end if;
        if (dosplit = '1') and (w >= to_unsigned(MIN_W, 12)) then
            r.doit := '1';
        else
            r.doit := '0';
        end if;
        return r;
    end function;

    -- Clamp a signed coord to a 0..4095 unsigned (negative -> 0).
    function clamp0(s : signed(13 downto 0)) return unsigned is
    begin
        if s < 0 then return to_unsigned(0, 12);
        else           return unsigned(s(11 downto 0)); end if;
    end function;

    -- Descend stage: step into the pixel's half and fold the choice into the seed.
    function descend(n : t_node; orient, doit : std_logic; nx : unsigned(11 downto 0);
                     px, py : unsigned(11 downto 0))
        return t_node is
        variable r : t_node;
        variable lo, hi, pix : unsigned(11 downto 0);
    begin
        r := n;
        if doit = '1' then
            if orient = '0' then lo := n.x0; hi := n.x1; pix := px;
            else                 lo := n.y0; hi := n.y1; pix := py; end if;
            if pix < nx then
                hi := nx;
                r.seed := n.seed xor ("00000000" & nx(7 downto 0));
            else
                lo := nx;
                r.seed := (n.seed xor ("00000000" & nx(7 downto 0))) xor x"5A5A";
            end if;
            if orient = '0' then r.x0 := lo; r.x1 := hi;
            else                 r.y0 := lo; r.y1 := hi; end if;
            r.frozen := '0';
        else
            r.frozen := '1';
        end if;
        return r;
    end function;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
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
    -- Thin horizontal features (y): seam half-width and texture-line width in
    -- virtual units, at least about one field-line step so SD never skips them.
    function f_seam_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        if to_integer(unsigned(t)) = 0 then return to_unsigned(3, 3);   -- 480i
        else                                return to_unsigned(2, 3); end if;
    end function;
    function f_thin_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0      => return to_unsigned(5, 3);   -- 480i step 4.44
            when 8      => return to_unsigned(4, 3);   -- 576i step 3.75
            when 4      => return to_unsigned(3, 3);   -- 480p step 2.25
            when others => return to_unsigned(2, 3);
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Position + per-frame parameters
    --------------------------------------------------------------------------
    signal pixel_x, pixel_y : unsigned(11 downto 0) := (others => '0');
    -- Resolution scaling: pixel_x/pixel_y are the integer parts of virtual
    -- 1080-line accumulators (x Q.1, y Q.8 per field line).  At 1080p every
    -- step is exactly 1.0, so they equal the old pixel/line counters.
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_seamy  : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_thy    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_thyt   : unsigned(2 downto 0)  := to_unsigned(2, 3);   -- s_thy capped at period/4
    signal pixel_xa : unsigned(12 downto 0) := (others => '0');
    signal pixel_ya : unsigned(19 downto 0) := (others => '0');
    signal s_cmaskh : unsigned(8 downto 0)  := to_unsigned(127, 9);  -- cell mask, half units
    signal s_xcol   : std_logic := '1';     -- this pixel is the first of a cell column
    signal s_yfst   : std_logic := '1';     -- this line is the first of a 32-unit block row
    signal prev_hsync_n     : std_logic := '1';
    signal prev_vsync_n     : std_logic := '1';

    -- pixel_x counts the GENERATED avid (the picture as it leaves); pixel_y is
    -- anchored to the first active line, so the luma map's block rows and the
    -- quilt's cell rows share one coordinate system.
    signal s_yarm       : std_logic := '1';     -- after vsync, until the first active line
    signal s_line_go    : std_logic := '0';     -- one clock after each hsync edge

    signal s_g          : integer range 5 to 8 := 6;
    signal s_cellmask   : unsigned(11 downto 0) := to_unsigned(63, 12);
    signal s_csz1       : signed(13 downto 0) := to_signed(64, 14);    -- cell size, px
    signal s_csz2       : signed(13 downto 0) := to_signed(128, 14);   -- two cells, px
    signal s_density    : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_dens_floor : unsigned(7 downto 0) := to_unsigned(95, 8);  -- 255 - density (precomputed)
    -- Split jitter is now fixed at Medium (w/8); K4 became the Luma Mod
    -- threshold.  Kept as a constant so calc_split's case folds away.
    constant C_JIT_MODE : unsigned(1 downto 0) := to_unsigned(2, 2);
    signal s_lum_thr    : unsigned(7 downto 0) := to_unsigned(128, 8);  -- K4: Luma Mod hide threshold
    signal s_tint       : unsigned(3 downto 0) := (others => '0');
    signal s_tex_shift  : integer range 3 to 6 := 5;
    -- Both layout seeds now come from the single K2 Seed knob (composition and
    -- sub-piecing used to be K2/K3; K3 is the Evolve Rate).
    signal s_comp_seed  : unsigned(15 downto 0) := x"1234";
    signal s_sub_seed   : unsigned(15 downto 0) := x"ABCD";
    signal s_seam_en    : std_logic := '1';
    signal s_relief     : std_logic := '1';     -- S8 Relief: 0 Flat (soft emboss), 1 Puffed (deep + puff)
    signal s_notsq      : std_logic := '0';     -- S9: 0 = square grid, 1 = off-grid overlap
    signal s_lumamod    : std_logic := '0';     -- derived: K4 above its off deadband
    signal s_logcabin   : std_logic := '0';     -- S10: 0 = crazy (random axis), 1 = log cabin
    signal s_palmode    : std_logic := '0';

    --------------------------------------------------------------------------
    -- Raster measurement (source side)
    --------------------------------------------------------------------------
    signal s_vx         : unsigned(11 downto 0) := (others => '0');  -- source pixel index
    signal s_vxa        : unsigned(12 downto 0) := (others => '0');  -- source virtual x (Q.1, luma map)
    signal s_wline      : unsigned(11 downto 0) := to_unsigned(1920, 12);  -- last line width
    signal s_ylast      : unsigned(11 downto 0) := to_unsigned(1079, 12);  -- running last active row
    signal s_hlast      : unsigned(11 downto 0) := to_unsigned(1079, 12);  -- previous frame's last row

    --------------------------------------------------------------------------
    -- Generated avid.  The encoders get no data-enable — they pair Cb/Cr by
    -- counting from hsync — while the core's 4:4:4->4:2:2 packer restarts on Cb
    -- at every avid rise.  A source avid that lands a clock off its usual hsync
    -- parity therefore swaps U/V for that line (thin coloured lines across flat
    -- colour).  Cubist/aurora's HW-confirmed recipe: start 3 or 4 clocks after
    -- the source's avid rise, whichever keeps the hsync->start parity at its
    -- voted usual value, and end on our own per-field width.
    --------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);

    --------------------------------------------------------------------------
    -- Evolve engine (S8).  The quilt is memoryless — every pixel re-derives its
    -- leaf from hashes — so "one piece changed" cannot be stored.  Instead each
    -- leaf owns a slot k in a 65536-slot ring, and a per-frame phase sweeps a
    -- cursor `pos` around that ring:  a leaf's GENERATION is
    --     gen = pass + (k < pos)
    -- where pass/pos are the high/low halves of the phase.  Advancing pos by one
    -- therefore steps the generation of exactly the one slot it just passed, and
    -- the change persists until the cursor laps.  Cumulative, one at a time, no
    -- memory.  gen is split into a colour generation and a texture generation
    -- that alternate (see S10a), so a change moves EITHER the colour or the
    -- texture, never both.
    --
    -- The K3 knob is BIPOLAR and picks what k means, which is what makes the two
    -- evolve modes fall out of one mechanism:
    --   * right of centre (Random) — k is a mix of all 16 leaf seed bits, so the
    --     cursor visits pieces in scattered hash order.
    --   * left of centre (Snake) — k is the leaf's own POSITION on a serpentine
    --     over 8 px bands, so consecutive visits land on pieces that touch and
    --     the change crawls across the quilt.  Cells are visited in a hash-picked
    --     order within each 2x2 block so the walk wanders instead of reading as a
    --     scanline wipe.
    --   * centre — the rate is zero, so the phase stops and the quilt FREEZES as
    --     it stands (it does not snap back to the un-evolved colours).  The
    --     Random/Snake choice is held at centre too, so passing through centre
    --     does not reassign slots.
    -- gen only ever takes the values pass, pass+1 or pass+2, so the three 4-bit
    -- slices it can contribute are precomputed here once per frame and the
    -- pixel path just picks one.
    --
    -- Snake's slot is the Hilbert-curve index of the leaf's centre (see S10h),
    -- so the cursor hops from each piece to a touching one along a meandering
    -- path that fills small regions before moving on, and the curve fills the
    -- whole ring (no dead stretch).  The path changes every lap (s_hdir/s_hst*).
    type t_u4_3 is array (0 to 2) of unsigned(3 downto 0);
    signal s_evo_rate   : unsigned(27 downto 0) := (others => '0');
    signal s_evo_ph     : unsigned(27 downto 0) := (others => '0');
    signal s_evo_pos    : unsigned(15 downto 0) := (others => '0');
    signal s_evo_g      : t_u4_3 := (others => (others => '0'));
    signal s_evo_snake  : std_logic := '0';     -- K3 left of centre: spatial walk
    signal s_fpub       : std_logic := '0';     -- publish the new phase (clock after the add)
    -- Snake path variant for this lap: which half-screen square goes first and
    -- each square's starting orientation (s & c), chosen so the curve's exit
    -- from the first square is adjacent to its entry into the second.
    signal s_hdir       : std_logic := '0';
    signal s_hstl, s_hstr : std_logic_vector(1 downto 0) := "00";
    -- Analog vsync serrates (multiple edges per field), so the phase increment
    -- needs the saw-active guard — a plain edge-triggered add would step several
    -- times per frame.  Idempotent latches in the same branch are safe as-is.
    signal s_saw_active : std_logic := '0';

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);
    signal gv   : std_logic_vector(0 to LATENCY - 1) := (others => '0');  -- generated avid

    --------------------------------------------------------------------------
    -- Delayed position copies (p_qpos).  Stage k of the pixel pipeline needs
    -- the picture position as it was k clocks ago.  Instead of carrying 24
    -- bits of px/py down every stage, each consumer gets its own copy:
    --   * x: a second column accumulator that replays pixel_xa's own update
    --     rule k clocks late — reset on the hsync edge seen in pipe(k-1/k),
    --     step on gv(k-1) — so q_xa<k> = pixel_xa delayed by k exactly.
    --   * y: pixel_y only changes on an hsync or vsync edge, so q_y<k> loads
    --     it on the same edge seen k clocks late (the edges are always more
    --     than k clocks apart, so the value loaded is the one k clocks ago).
    -- Consumers: S1b coverage (k = 3), descend levels 0-3 (k = 7, 10, 13,
    -- 16) and S10a's leaf-local coords (k = 17, one register after k = 16).
    --------------------------------------------------------------------------
    signal q_xa3, q_xa7, q_xa10, q_xa13, q_xa16 : unsigned(12 downto 0) := (others => '0');
    signal q_y3, q_y7, q_y10, q_y13, q_y16      : unsigned(11 downto 0) := (others => '0');
    signal q_x17, q_y17                         : unsigned(11 downto 0) := (others => '0');
    signal q_x3, q_x7, q_x10, q_x13, q_x16      : unsigned(11 downto 0);
    -- s_xcol 3 clocks late: the luma map's per-column dens/hide step (L_*)
    -- lands on S1b's clock, so S1c reads L_* directly (no s1a/s1a2/s1b copies).
    signal s_xcol_d  : std_logic_vector(1 to 3) := (others => '0');

    --------------------------------------------------------------------------
    -- Luma map (Luma Mod).  A patch's subdivide/hide decisions must be identical
    -- for every pixel in it, so they cannot read per-pixel luma.  Each frame the
    -- input luma is averaged over 32x32-pixel blocks (a box across the block's
    -- 32 pixels, an IIR down its lines — the IIR stays a true mean for a
    -- partial bottom block row) and written into one bank of a two-bank map;
    -- the picture reads the OTHER bank (the previous frame), so every reader of
    -- a block sees one value all frame long.  A grid cell takes the block at
    -- its centre.  In Not-Square mode each overlapping rectangle uses its OWN
    -- cell's value, so big rectangles no longer fracture along the hidden grid.
    -- Index = block_row*60 + block_col (60 blocks = 1920 px), 2048 per bank.
    --------------------------------------------------------------------------
    type t_dh is record                      -- per-cell decision inputs
        dens : unsigned(7 downto 0);         -- subdivide threshold base
        hide : std_logic;                    -- below the K4 threshold -> black
    end record;
    constant C_DH0 : t_dh := (dens => to_unsigned(160, 8), hide => '0');
    type t_dh_4 is array (0 to 3) of t_dh;

    type t_map is array (0 to 4095) of unsigned(7 downto 0);
    signal map_ram  : t_map := (others => to_unsigned(128, 8));
    signal m_raddr  : unsigned(11 downto 0) := (others => '0');
    signal m_rd     : unsigned(7 downto 0) := (others => '0');
    signal m_rq     : unsigned(7 downto 0) := (others => '0');
    signal s_bank   : std_logic := '0';      -- map bank being WRITTEN this frame
    -- the map's read and write ports always address different banks
    attribute no_rw_check : boolean;
    attribute no_rw_check of map_ram : signal is true;

    -- per-block-column running line average (IIR state, 4 fraction bits)
    type t_est is array (0 to 63) of unsigned(15 downto 0);
    signal est_ram  : t_est := (others => (others => '0'));
    signal est_rd   : unsigned(15 downto 0) := (others => '0');
    -- one RMW per 32 pixels; a column's write lands 4 clocks after its read
    attribute no_rw_check of est_ram : signal is true;

    -- write side: block box sums and the RMW pipeline E0..E4
    signal s_hsum   : unsigned(14 downto 0) := (others => '0');   -- step-weighted box sum
    signal e0_go, e1_go, e2_go, e3_go, e4_go : std_logic := '0';
    signal e0_first, e1_first, e2_first     : std_logic := '0';
    signal e0_col, e1_col, e2_col, e3_col, e4_col : unsigned(5 downto 0) := (others => '0');
    signal e0_avg, e1_avg, e2_avg : unsigned(7 downto 0) := (others => '0');
    signal e2_widx, e3_widx, e4_widx : unsigned(10 downto 0) := (others => '0');
    signal est_q, est_t, est_w : unsigned(11 downto 0) := (others => '0');
    signal e3_add   : unsigned(11 downto 0) := (others => '0');

    -- read side: per-line row bases + per-column prefetch of the next column's
    -- own-row and up-row cells.  Per-frame block geometry for the grid scale:
    -- a cell's top block row is row AND s_ctbmask, its centre block is offset
    -- by s_boff, the row above is s_bstep blocks up, and the next column's
    -- centre block is (column-start x)/32 + s_bcadd.
    signal s_ctbmask : unsigned(5 downto 0) := "111110";
    signal s_boff    : unsigned(5 downto 0) := (others => '0');
    signal s_bstep   : unsigned(5 downto 0) := to_unsigned(2, 6);
    signal s_bcadd   : unsigned(6 downto 0) := to_unsigned(2, 7);
    signal r_st      : unsigned(2 downto 0) := (others => '0');
    signal r_own     : unsigned(6 downto 0) := (others => '0');
    signal r_up      : signed(7 downto 0) := (others => '0');
    signal rb_own, rb_up, rb_w : unsigned(10 downto 0) := (others => '0');
    signal pf_go, pf_a, pf_b, pf_c, pf_d : std_logic := '0';
    signal pf_bc     : unsigned(6 downto 0) := (others => '0');
    -- dens/hide of this column's (L_*) and the next column's (N_*) cells, own
    -- row and up row; the left-column pair is the previous column's own pair.
    signal N_own, N_up, L_own, L_up, L_lown, L_lup : t_dh := C_DH0;

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    signal s0_x, s0_y : unsigned(11 downto 0);

    -- Not-Square overlap: each cell gets a random origin jitter so cells overlap.
    -- A pixel is resolved against the 2x2 neighbourhood of jittered cells; the
    -- highest-priority covering cell wins, with the plain square cell as a gap
    -- fallback.  Candidate order k: (gx,gy) (gx-1,gy) (gx,gy-1) (gx-1,gy-1).
    type t_u16_4  is array (0 to 3) of unsigned(15 downto 0);
    type t_s14_4  is array (0 to 3) of signed(13 downto 0);
    type t_u4_4   is array (0 to 3) of unsigned(3 downto 0);
    -- candidate priority without coverage: size rank(1:0) & z(3:0)
    type t_u6_4   is array (0 to 3) of unsigned(5 downto 0);
    -- candidate bounds + seed only (px/py/dens are shared across all 4)
    type t_cand   is record
        x0, x1, y0, y1 : unsigned(11 downto 0);
        seed           : unsigned(15 downto 0);
    end record;
    type t_cand_4 is array (0 to 3) of t_cand;
    constant CK_X : integer_vector(0 to 3) := (0, 1, 0, 1);
    constant CK_Y : integer_vector(0 to 3) := (0, 0, 1, 1);

    -- S1a holds only the sliced/subtracted ORIGINS; the four init_seed hashes
    -- that used to sit in the same stage were the HD critical path (slice ->
    -- 14-bit subtract -> 16-bit hash, x4, all combinational), so they now get
    -- their own stage S1a2.
    signal s1a_ax, s1a_ay  : t_s14_4;                      -- per-candidate aligned origin
    signal s1a_base        : t_node;                       -- plain square cell (seed unset)

    signal s1a2_hk         : t_u16_4;                      -- per-candidate cell hash/seed
    signal s1a2_ax, s1a2_ay: t_s14_4;
    signal s1a2_base       : t_node;                       -- plain square cell (seed filled)

    -- S1b builds all 4 clamped candidate cells in parallel; S1c just argmaxes.
    signal s1b_cand        : t_cand_4;
    signal s1b_cov         : std_logic_vector(0 to 3);
    -- pairwise priority compares, pz(i) >= pz(j), for (i,j) = 01 02 03 12 13 23
    signal s1b_p           : std_logic_vector(0 to 5);
    signal s1b_base        : t_node;

    signal s1_n  : t_node;                                 -- selected coarse cell node

    -- hash-stage outputs (decoded bits + node carrying new seed)
    signal h0_n, h1_n, h2_n, h3_n     : t_node;
    signal h0_or, h1_or, h2_or, h3_or : std_logic;
    signal h0_ds, h1_ds, h2_ds, h3_ds : std_logic;
    signal h0_sg, h1_sg, h2_sg, h3_sg : std_logic;
    signal h0_hf, h1_hf, h2_hf, h3_hf : std_logic;
    signal h0_w, h1_w, h2_w, h3_w     : unsigned(11 downto 0);   -- active-axis width
    signal h0_mid, h1_mid, h2_mid, h3_mid : unsigned(11 downto 0); -- (lo+hi)/2

    -- decide-stage outputs (node + orient passthrough + split coord + flag)
    signal d0_n, d1_n, d2_n, d3_n         : t_node;
    signal d0_or, d1_or, d2_or, d3_or     : std_logic;
    signal d0_dt, d1_dt, d2_dt, d3_dt     : std_logic;
    signal d0_nx, d1_nx, d2_nx, d3_nx     : unsigned(11 downto 0);

    -- descend-stage output nodes
    signal n0, n1, n2, n3 : t_node;

    -- leaf metrics, split across two stages
    type t_leaf is record
        lx, ly, m1, m2 : unsigned(11 downto 0);   -- leaf-local coords, edge mins
        krnd           : unsigned(15 downto 0);   -- Random evolve slot
        cbase, tbase   : unsigned(3 downto 0);    -- base colour / texture number
        p, hide        : std_logic;               -- colour/texture phase, Luma Mod hide
    end record;
    signal s10a_lf, s10h_lf, s10h2_lf : t_leaf;   -- lx..m2 are only read from s10a_lf
    -- The leaf's seam / puff / texture-coordinate features, built in S10h from
    -- lx/ly/m1/m2 (which then go no further) and carried to S10b: 22 bits
    -- instead of the 48 bits of coordinates.  Only the texture NUMBER waits for
    -- the evolve result, so S10b is just the 16-way texture pick.
    type t_tex is record
        seam           : std_logic;               -- within the seam width
        puff           : unsigned(3 downto 0);    -- puff ramp index (0 = none)
        rx, ry, rd, ra : unsigned(1 downto 0);    -- 2-bit ramp fields
        chk            : std_logic;               -- checker: x bit s = y bit s
        wxy, wyb       : std_logic;               -- block parity x xor y / y (bit s+1)
        bv             : unsigned(3 downto 0);    -- Bayer value (dither, dots)
        nv, nh         : std_logic;               -- near a vertical / horizontal cell line
    end record;
    signal s10h_tx, s10h2_tx, s10a2_tx : t_tex;
    signal s10a_sx, s10a_sy : unsigned(12 downto 0);   -- x0+x1, y0+y1 (2x leaf centre)
    signal s10h_dhi   : unsigned(5 downto 0);          -- Hilbert digits, levels 5..3
    signal s10h_st    : std_logic_vector(1 downto 0);  -- orientation after level 3
    signal s10h_xl, s10h_yl : unsigned(2 downto 0);    -- low coordinate bits
    signal s10h_ord   : std_logic;                     -- square order bit
    signal s10h2_k    : unsigned(15 downto 0);         -- the leaf's evolve slot
    -- S10a also builds the leaf's evolve ring slot; the compare against the
    -- cursor plus the generation lookup are a stage of their own (S10a2) because
    -- the 16-bit compare will not fit alongside the leaf metrics.

    -- Evolved leaf indices: the base colour/texture numbers already advanced by
    -- this leaf's colour/texture generation, so the 16-bit leaf seed never has to
    -- reach S10b.
    signal s10a2_cidx       : unsigned(3 downto 0);
    signal s10a2_tidx       : unsigned(3 downto 0);
    signal s10a2_hide       : std_logic;

    -- S10b result, then the 3 IO-align pad stages that bring the total latency
    -- to a multiple of 4 (here the pixel is 12 bits wide; after s_io it is 32)
    type t_s10 is record
        seam : std_logic;                         -- within the seam width
        puff : unsigned(3 downto 0);              -- puff ramp index (0 = none)
        pidx : unsigned(3 downto 0);
        tval : unsigned(1 downto 0);              -- texture shade (computed in S10b)
        hide : std_logic;
    end record;
    type t_s10_3 is array (0 to 2) of t_s10;
    signal s10_r   : t_s10;
    signal s10_pad : t_s10_3;

    -- palette + texture
    signal s11_y, s11_u, s11_v : unsigned(9 downto 0);
    signal s11_delta           : signed(10 downto 0);    -- emboss + puff luma delta
    signal s11_seam            : std_logic;
    signal s11_hide            : std_logic;

    -- assembled colour
    signal s12_y, s12_u, s12_v : unsigned(9 downto 0);
    signal s12_hide            : std_logic;

    -- output register
    signal s_io       : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Position counters, raster measurement + per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_rate : unsigned(27 downto 0);
        variable v_k3r  : unsigned(9 downto 0);
        variable v_mag  : unsigned(8 downto 0);     -- distance from centre
        variable v_mlo  : unsigned(6 downto 0);
        variable v_off  : boolean;
        variable v_p0, v_p1, v_p2 : unsigned(7 downto 0);   -- pass, pass+1, pass+2
        variable v_nx : unsigned(12 downto 0);
        variable v_ny : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            -- Resolution scaling: per-mode steps, free-running from the timing
            -- ID (static while a mode runs).
            s_xs    <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys    <= f_vstep_y(registers_in(8)(3 downto 0));
            s_seamy <= f_seam_y(registers_in(8)(3 downto 0));
            s_thy   <= f_thin_y(registers_in(8)(3 downto 0));
            -- texture lines keep ~1/4 of their period (Fine 8: 2, Small 16: <= 4)
            -- so a fine texture's density does not change at SD
            if s_tex_shift = 3 then
                s_thyt <= to_unsigned(2, 3);
            elsif s_tex_shift = 4 and s_thy > 4 then
                s_thyt <= to_unsigned(4, 3);
            else
                s_thyt <= s_thy;
            end if;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;
            s_line_go <= v_h_edge;

            -- picture column: counts the GENERATED avid, so the quilt lines up
            -- with the avid that actually leaves the program.  Virtual units;
            -- s_xcol flags the first pixel of each cell column (its offset in
            -- the cell is below one step — the old "(x and mask) = 0").
            if v_h_edge = '1' then
                pixel_xa <= (others => '0');
                s_xcol   <= '1';
            elsif g_avid = '1' then
                v_nx     := pixel_xa + s_xs;
                pixel_xa <= v_nx;
                if (v_nx(8 downto 0) and s_cmaskh) < resize(s_xs, 9) then s_xcol <= '1';
                else                                                     s_xcol <= '0'; end if;
            end if;

            -- source column (luma map) + per-line width (generated avid length)
            if v_h_edge = '1' then
                if s_vx /= 0 then s_wline <= s_vx; end if;
                s_vx  <= (others => '0');
                s_vxa <= (others => '0');
            elsif data_in.avid = '1' then
                s_vx  <= s_vx + 1;
                s_vxa <= s_vxa + s_xs;
            end if;

            if data_in.avid = '1' then
                s_saw_active <= '1';       -- saw-active guard for the per-frame steps
                s_yarm       <= '0';       -- first active line reached
                s_ylast      <= pixel_y;   -- runs up to the frame's last active row
            end if;

            -- rows: held at 0 from vsync until the first active line, then one
            -- per hsync, so row 0 is the top line of the picture
            -- (virtual units; s_yfst flags the first line of each 32-unit block
            -- row — the old "pixel_y(4:0) = 0" — from a toggle of bit 13)
            if v_v_edge = '1' then
                pixel_ya <= (others => '0');
                s_yfst   <= '1';
                s_yarm   <= '1';
            elsif v_h_edge = '1' and s_yarm = '0' then
                v_ny     := pixel_ya + s_ys;
                pixel_ya <= v_ny;
                s_yfst   <= v_ny(13) xor pixel_ya(13);
            end if;

            -- Once per field — guarded, because analog vsync serrates into
            -- several edges and none of these steps are idempotent: advance the
            -- evolve phase, flip the luma-map banks, latch the measurements.
            -- The next clock publishes the cursor and the generation slices
            -- (pos = ph(19:4), pass = ph(27:20); gen is pass or pass+1 and the
            -- colour/texture split needs pass+2) and the lap's Snake path.
            s_fpub <= '0';
            if v_v_edge = '1' and s_saw_active = '1' then
                s_saw_active <= '0';
                s_fpub       <= '1';
                s_evo_ph     <= s_evo_ph + s_evo_rate;
                s_bank       <= not s_bank;
                s_hlast      <= s_ylast;
                g_wm1        <= s_wline - 1;
            end if;
            if s_fpub = '1' then
                s_evo_pos <= s_evo_ph(19 downto 4);
                v_p0 := s_evo_ph(27 downto 20);
                v_p1 := v_p0 + 1;
                v_p2 := v_p0 + 2;
                s_evo_g(0) <= v_p0(4 downto 1);
                s_evo_g(1) <= v_p1(4 downto 1);
                s_evo_g(2) <= v_p2(4 downto 1);
                -- Snake path for this lap, from the lap count: 8 variants, each
                -- with the two squares joined edge to edge.
                s_hdir <= v_p0(2);
                if v_p0(2) = '0' then                       -- left square first
                    if v_p0(0) = '1' then s_hstl <= "11"; else s_hstl <= "00"; end if;
                    if v_p0(1) = '1' then s_hstr <= "10"; else s_hstr <= "00"; end if;
                else                                        -- right square first
                    if v_p0(0) = '1' then s_hstr <= "01"; else s_hstr <= "10"; end if;
                    if v_p0(1) = '1' then s_hstl <= "11"; else s_hstl <= "01"; end if;
                end if;
            end if;

            -- Control latches (idempotent, so every vsync edge is fine).
            if v_v_edge = '1' then
                -- Grid scale: K1 top 2 bits -> coarse cell 256/128/64/32 px.  The
                -- block constants put each cell's luma sample in the 32x32 block
                -- at its centre (up-left of centre for the even block counts).
                case to_integer(unsigned(registers_in(0)(9 downto 8))) is
                    when 0      => s_g <= 8; s_cellmask <= to_unsigned(255, 12); s_cmaskh <= to_unsigned(511, 9);
                                   s_csz1 <= to_signed(256, 14); s_csz2 <= to_signed(512, 14);
                                   s_ctbmask <= "111000"; s_boff <= to_unsigned(3, 6);
                                   s_bstep <= to_unsigned(8, 6); s_bcadd <= to_unsigned(11, 7);
                    when 1      => s_g <= 7; s_cellmask <= to_unsigned(127, 12); s_cmaskh <= to_unsigned(255, 9);
                                   s_csz1 <= to_signed(128, 14); s_csz2 <= to_signed(256, 14);
                                   s_ctbmask <= "111100"; s_boff <= to_unsigned(1, 6);
                                   s_bstep <= to_unsigned(4, 6); s_bcadd <= to_unsigned(5, 7);
                    when 2      => s_g <= 6; s_cellmask <= to_unsigned(63, 12); s_cmaskh <= to_unsigned(127, 9);
                                   s_csz1 <= to_signed(64, 14); s_csz2 <= to_signed(128, 14);
                                   s_ctbmask <= "111110"; s_boff <= to_unsigned(0, 6);
                                   s_bstep <= to_unsigned(2, 6); s_bcadd <= to_unsigned(2, 7);
                    when others => s_g <= 5; s_cellmask <= to_unsigned(31, 12); s_cmaskh <= to_unsigned(63, 9);
                                   s_csz1 <= to_signed(32, 14); s_csz2 <= to_signed(64, 14);
                                   s_ctbmask <= "111111"; s_boff <= to_unsigned(0, 6);
                                   s_bstep <= to_unsigned(1, 6); s_bcadd <= to_unsigned(1, 7);
                end case;

                -- K2 Seed drives BOTH layout seeds (composition + sub-piecing).
                s_comp_seed <= mix16(unsigned(registers_in(1)), x"9E37");
                s_sub_seed  <= mix16(unsigned(registers_in(1)), x"C2B5");

                -- K3 Evolve: BIPOLAR about centre.  Centre (within a deadband
                -- wide enough to swallow pot ADC noise) = rate 0 = frozen as it
                -- stands, with the last mode held; right = Random; left = Snake.
                -- Both halves ramp the same way, so the two modes run at matching
                -- speeds at matching distances from centre.
                v_k3r := unsigned(registers_in(2)(9 downto 0));
                v_off := true;
                if v_k3r >= to_unsigned(536, 10) then
                    v_off := false;
                    s_evo_snake <= '0';
                    v_mag := resize(v_k3r - 536, 9);
                elsif v_k3r <= to_unsigned(488, 10) then
                    v_off := false;
                    s_evo_snake <= '1';
                    v_mag := resize(488 - v_k3r, 9);
                else
                    v_mag := (others => '0');
                end if;

                -- Magnitude -> phase step.  Four shift zones with carried offsets
                -- give a monotonic, roughly exponential 16..42945 span.  pos
                -- advances rate/16 slots per frame over a 65536-slot ring, so the
                -- bottom is about one piece a second and the top laps the whole
                -- quilt in ~24 frames.  Resize BEFORE shifting — shift_left keeps
                -- the operand's own width.
                v_mlo := v_mag(6 downto 0);
                case to_integer(v_mag(8 downto 7)) is
                    when 0      => v_rate := shift_left(resize(v_mlo, 28), 1) + 16;
                    when 1      => v_rate := shift_left(resize(v_mlo, 28), 4) + 271;
                    when 2      => v_rate := shift_left(resize(v_mlo, 28), 6) + 2304;
                    when others => v_rate := shift_left(resize(v_mlo, 28), 8) + 10433;
                end case;
                if v_off then s_evo_rate <= (others => '0');
                else          s_evo_rate <= v_rate; end if;
                -- K4 Threshold IS the Luma Mod control: fully anticlockwise turns
                -- luma modulation off entirely, anything above the deadband turns
                -- it on and sets the hide cut point.  The deadband (16/1023) keeps
                -- pot ADC noise from flickering the mode at the bottom of travel.
                s_lum_thr   <= unsigned(registers_in(3)(9 downto 2));
                if registers_in(3)(9 downto 4) = "000000" then s_lumamod <= '0';
                else                                           s_lumamod <= '1'; end if;
                s_tint      <= unsigned(registers_in(4)(9 downto 6));

                -- K6 Texture Scale: Fine / Small / Medium / Coarse.  Periods are
                -- coarse (16/32/64/128 px) so textures read as fabric weave at full
                -- HD, not 1-2 px noise.
                case to_integer(unsigned(registers_in(5)(9 downto 8))) is
                    when 0      => s_tex_shift <= 3;   -- ~16 px
                    when 1      => s_tex_shift <= 4;   -- ~32 px
                    when 2      => s_tex_shift <= 5;   -- ~64 px
                    when others => s_tex_shift <= 6;   -- ~128 px
                end case;

                s_seam_en <= registers_in(6)(0);
                s_relief  <= registers_in(6)(1);   -- S8 Relief: soft / deep
                s_notsq   <= registers_in(6)(2);
                s_logcabin <= registers_in(6)(3);  -- S10 Piecing: Crazy / Log Cabin
                s_palmode <= registers_in(6)(4);

                s_density   <= unsigned(registers_in(7)(9 downto 2));
                s_dens_floor <= 255 - unsigned(registers_in(7)(9 downto 2));
            end if;
        end if;
    end process p_position;

    pixel_x <= pixel_xa(12 downto 1);
    pixel_y <= pixel_ya(19 downto 8);

    --------------------------------------------------------------------------
    -- Generated avid (see the declarations).  g_ph is this line's hsync->avid
    -- rise parity, g_par the voted usual parity; a line whose avid lands off
    -- parity starts one clock later, so the generated start always sits at the
    -- same hsync parity.  It ends after the per-field width, or at hsync.
    --------------------------------------------------------------------------
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
        end if;
    end process p_gavid;

    --------------------------------------------------------------------------
    -- Luma map, write side (source timing).  E0 box-sums each 32-pixel block of
    -- the line and fires on its last pixel; E1-E4 run the column's line IIR
    -- (est = est - est/16 + line average, restarted on a block row's first
    -- line), and the result goes to the est RAM and to this frame's map bank.
    -- One event per 32 clocks, so the pipeline never overlaps itself.
    --------------------------------------------------------------------------
    -- Resolution scaling: blocks are 32 VIRTUAL units (64 half units of s_vxa);
    -- each pixel's luma is weighted by its step (2/3/4 half units), so a block
    -- sums to ~64x its mean at every resolution (1080p: exactly 2x the old
    -- 32-pixel sum, so the average is bit-identical).
    p_lmap_w : process(clk)
        variable v_y8  : unsigned(7 downto 0);
        variable v_yw  : unsigned(14 downto 0);
        variable v_sum : unsigned(14 downto 0);
        variable v_end : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            v_y8  := unsigned(data_in.y(9 downto 2));
            case to_integer(s_xs) is
                when 3      => v_yw := shift_left(resize(v_y8, 15), 1) + resize(v_y8, 15);
                when 4      => v_yw := shift_left(resize(v_y8, 15), 2);
                when others => v_yw := shift_left(resize(v_y8, 15), 1);
            end case;
            v_end := resize(s_vxa(5 downto 0), 7) + resize(s_xs, 7);   -- bit 6: last pixel of the block
            e0_go <= '0';
            if data_in.avid = '1' then
                if s_vxa(5 downto 0) < resize(s_xs, 6) then v_sum := v_yw;
                else                                         v_sum := s_hsum + v_yw; end if;
                s_hsum <= v_sum;
                if v_end(6) = '1' then
                    e0_go  <= '1';
                    if v_sum(14) = '1' then e0_avg <= (others => '1');   -- 720p 22-px block
                    else                    e0_avg <= v_sum(13 downto 6); end if;
                    e0_col <= s_vxa(11 downto 6);
                    e0_first <= s_yfst;
                end if;
            end if;

            -- E1: est RAM read in flight
            e1_go <= e0_go; e1_first <= e0_first; e1_col <= e0_col; e1_avg <= e0_avg;

            -- E2: fabric register of the EBR read; map write index
            e2_go <= e1_go; e2_first <= e1_first; e2_col <= e1_col; e2_avg <= e1_avg;
            est_q   <= est_rd(11 downto 0);
            e2_widx <= rb_w + e1_col;

            -- E3: decay, or restart on the block row's first line
            e3_go <= e2_go; e3_col <= e2_col; e3_widx <= e2_widx;
            if e2_first = '1' then
                est_t  <= (others => '0');
                e3_add <= shift_left(resize(e2_avg, 12), 4);
            else
                est_t  <= est_q - resize(est_q(11 downto 4), 12);
                e3_add <= resize(e2_avg, 12);
            end if;

            -- E4: steady state is 16x the block mean
            e4_go <= e3_go; e4_col <= e3_col; e4_widx <= e3_widx;
            est_w <= est_t + e3_add;
        end if;
    end process p_lmap_w;

    p_est_wr : process(clk)
    begin
        if rising_edge(clk) then
            if e4_go = '1' then
                est_ram(to_integer(e4_col)) <= resize(est_w, 16);
            end if;
        end if;
    end process p_est_wr;

    p_est_rd : process(clk)
    begin
        if rising_edge(clk) then
            est_rd <= est_ram(to_integer(e0_col));
        end if;
    end process p_est_rd;

    p_map_wr : process(clk)
    begin
        if rising_edge(clk) then
            if e4_go = '1' then
                map_ram(to_integer(s_bank & e4_widx)) <= est_w(11 downto 4);
            end if;
        end if;
    end process p_map_wr;

    p_map_rd : process(clk)
    begin
        if rising_edge(clk) then
            m_rd <= map_ram(to_integer(m_raddr));
        end if;
    end process p_map_rd;

    --------------------------------------------------------------------------
    -- Luma map, read side (picture timing).  After each hsync a short sequence
    -- (one op per clock) finds this line's map row bases — the own-row and
    -- up-row centre blocks, clamped to the measured picture — and prefetches
    -- column 0.  At every column start the next column is prefetched: two reads
    -- through the one map port (own row, up row), each re-registered in fabric
    -- and turned straight into dens/hide with this frame's settings.  The
    -- column-start step of L_* runs 3 clocks late, on p_pipe's S1b clock, so
    -- S1c sees this column's cells (L_own/L_up) and the previous column's
    -- (L_lown/L_lup) — exactly the four Not-Square candidate cells.
    --------------------------------------------------------------------------
    p_lmap_r : process(clk)
        variable v_ctb : unsigned(5 downto 0);
        variable v_dh  : t_dh;
        variable v_bm  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            v_bm  := resize(s_hlast(10 downto 5), 7);      -- last block row (previous frame)
            pf_go <= '0';
            case to_integer(r_st) is
                when 1 =>
                    v_ctb := pixel_y(10 downto 5) and s_ctbmask;
                    r_own <= resize(v_ctb, 7) + resize(s_boff, 7);
                    r_st  <= to_unsigned(2, 3);
                when 2 =>
                    r_up <= signed(resize(r_own, 8)) - signed(resize(s_bstep, 8));
                    if r_own > v_bm then r_own <= v_bm; end if;
                    r_st <= to_unsigned(3, 3);
                when 3 =>
                    if r_up < 0 then
                        r_up <= (others => '0');
                    elsif r_up > signed(resize(v_bm, 8)) then
                        r_up <= signed(resize(v_bm, 8));
                    end if;
                    rb_own <= shift_left(resize(r_own, 11), 6) - shift_left(resize(r_own, 11), 2);
                    r_st   <= to_unsigned(4, 3);
                when 4 =>
                    rb_up <= shift_left(resize(unsigned(r_up(6 downto 0)), 11), 6)
                           - shift_left(resize(unsigned(r_up(6 downto 0)), 11), 2);
                    rb_w  <= shift_left(resize(pixel_y(10 downto 5), 11), 6)
                           - shift_left(resize(pixel_y(10 downto 5), 11), 2);
                    r_st  <= to_unsigned(5, 3);
                when 5 =>
                    pf_go <= '1';                           -- prefetch column 0
                    pf_bc <= resize(s_boff, 7);
                    r_st  <= to_unsigned(0, 3);
                when others => null;
            end case;
            if s_line_go = '1' then r_st <= to_unsigned(1, 3); end if;

            if g_avid = '1' and s_xcol = '1' then           -- first pixel of a cell column
                pf_go <= '1';
                pf_bc <= resize(pixel_x(10 downto 5), 7) + s_bcadd;
            end if;
            -- The same column step 3 clocks late (gv(2) is g_avid then, q_x3 is
            -- pixel_x then), so L_* change on S1b's clock and S1c reads them as
            -- they are.  This column's N_* prefetch lands 4 clocks after the
            -- column start, so the late step still takes the values the old
            -- on-time step did.
            s_xcol_d <= s_xcol & s_xcol_d(1 to 2);
            if gv(2) = '1' and s_xcol_d(3) = '1' then
                L_own <= N_own;
                L_up  <= N_up;
                if q_x3 = 0 then           -- left of column 0: reuse column 0
                    L_lown <= N_own;
                    L_lup  <= N_up;
                else
                    L_lown <= L_own;
                    L_lup  <= L_up;
                end if;
            end if;

            pf_a <= pf_go; pf_b <= pf_a; pf_c <= pf_b; pf_d <= pf_c;
            if pf_go = '1' then
                m_raddr <= (not s_bank) & (rb_own + pf_bc);
            elsif pf_a = '1' then
                m_raddr <= (not s_bank) & (rb_up + pf_bc);
            end if;
            m_rq <= m_rd;                           -- pf_c: own row, pf_d: up row
            if pf_c = '1' or pf_d = '1' then
                if s_lumamod = '1' then
                    if m_rq > s_dens_floor then v_dh.dens := m_rq - s_dens_floor;
                    else                        v_dh.dens := (others => '0'); end if;
                    if m_rq < s_lum_thr then v_dh.hide := '1';
                    else                     v_dh.hide := '0'; end if;
                else
                    v_dh.dens := s_density;
                    v_dh.hide := '0';
                end if;
                if pf_c = '1' then N_own <= v_dh;
                else               N_up  <= v_dh; end if;
            end if;
        end if;
    end process p_lmap_r;

    --------------------------------------------------------------------------
    -- Delayed position copies (see the declarations).  pipe(j) holds data_in
    -- from j+1 clocks ago, so (pipe(k-1), pipe(k)) show the sync edge that
    -- p_position saw k clocks ago, and gv(k-1) is g_avid from k clocks ago.
    --------------------------------------------------------------------------
    p_qpos : process(clk)
        procedure step_x(signal q : inout unsigned(12 downto 0); constant k : natural) is
        begin
            if pipe(k - 1).hsync_n = '0' and pipe(k).hsync_n = '1' then
                q <= (others => '0');
            elsif gv(k - 1) = '1' then
                q <= q + s_xs;
            end if;
        end procedure;
        procedure step_y(signal q : out unsigned(11 downto 0); constant k : natural) is
        begin
            if (pipe(k - 1).hsync_n = '0' and pipe(k).hsync_n = '1') or
               (pipe(k - 1).vsync_n = '0' and pipe(k).vsync_n = '1') then
                q <= pixel_y;
            end if;
        end procedure;
    begin
        if rising_edge(clk) then
            step_x(q_xa3, 3);   step_y(q_y3, 3);
            step_x(q_xa7, 7);   step_y(q_y7, 7);
            step_x(q_xa10, 10); step_y(q_y10, 10);
            step_x(q_xa13, 13); step_y(q_y13, 13);
            step_x(q_xa16, 16); step_y(q_y16, 16);
            q_x17 <= q_xa16(12 downto 1);
            q_y17 <= q_y16;
        end if;
    end process p_qpos;

    q_x3  <= q_xa3(12 downto 1);
    q_x7  <= q_xa7(12 downto 1);
    q_x10 <= q_xa10(12 downto 1);
    q_x13 <= q_xa13(12 downto 1);
    q_x16 <= q_xa16(12 downto 1);

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_x0, v_y0 : unsigned(11 downto 0);
        variable v_sz       : integer range 32 to 256;
        variable v_axk, v_ayk : signed(13 downto 0);
        variable v_hk       : unsigned(15 downto 0);   -- S1a2 candidate hash
        variable v_bn       : t_node;                  -- S1a2 base node + seed
        variable v_cells    : signed(13 downto 0);
        variable v_pxs, v_pys : signed(13 downto 0);
        variable v_jx, v_jy : unsigned(11 downto 0);
        variable v_x0s, v_y0s : signed(13 downto 0);
        variable v_wcx, v_wcy : signed(13 downto 0);   -- candidate extent (1 or 2 cells)
        variable v_rank     : integer range 0 to 2;    -- candidate size rank (#2-cell axes)
        variable v_pz       : t_u6_4;                  -- rank & z per candidate
        variable c01, c02, c03, c12, c13, c23 : std_logic;  -- vv(i) >= vv(j)
        variable w1, w2, w3 : std_logic;               -- winner one-hot (w0 implied)
        variable v_kw       : integer range 0 to 3;
        variable v_dh4      : t_dh_4;                  -- S1c candidate dens/hide
        variable dtmp       : t_dec;
        variable n          : t_node;
        variable lw, lh     : unsigned(11 downto 0);
        variable lx, ly     : unsigned(11 downto 0);
        variable bb, dd     : unsigned(11 downto 0);
        variable m1, m2     : unsigned(11 downto 0);
        variable seamd      : unsigned(11 downto 0);
        variable bidx       : integer range 0 to 15;
        variable vdiag      : unsigned(11 downto 0);
        variable v_adiag    : unsigned(11 downto 0);
        variable v_pm       : unsigned(11 downto 0);
        variable v_bv       : unsigned(3 downto 0);
        variable v_kmix     : unsigned(11 downto 0);   -- evolve: seed remix
        variable v_krnd, v_ks : unsigned(15 downto 0); -- evolve ring slots
        variable v_hx, v_hy : unsigned(5 downto 0);    -- Hilbert coords (16 px)
        variable v_hs       : std_logic_vector(3 downto 0);
        variable v_ho, v_hc : std_logic;               -- Hilbert orientation
        variable v_dhi, v_dlo : unsigned(5 downto 0);  -- Hilbert digits
        variable v_t4       : unsigned(3 downto 0);    -- puff ramp
        variable v_emb      : signed(10 downto 0);     -- emboss delta
        variable v_sum      : signed(11 downto 0);
        variable v_cg, v_tg : unsigned(3 downto 0);    -- colour/texture generation
        variable v_ic, v_it : integer range 0 to 2;    -- which s_evo_g slice
        variable v_nv, v_nh : boolean;
        variable v_t2       : unsigned(1 downto 0);
        variable v_rx, v_ry, v_rd, v_ra : unsigned(1 downto 0);   -- 2-bit ramp fields
        variable v_bxb, v_byb : std_logic;                        -- checker bit (bit s)
        variable v_wxb, v_wyb : std_logic;                        -- block parity (bit s+1)
        variable v_drow, v_dcol : unsigned(1 downto 0);           -- dither cell (scaled)
        variable pidx       : integer range 0 to 15;
        variable v_tx       : t_tex;                   -- S10h texture features
        variable ty, tu, tv : unsigned(9 downto 0);

        -- one hash stage: re-hash seed, decode split controls, and precompute the
        -- active-axis width w and midpoint mid (= (lo+hi)/2) so the decide stage's
        -- carry chain is just nx = mid +/- mag (one add) — keeps HD timing closing.
        procedure do_hash(signal q_n  : out t_node;
                          signal q_or : out std_logic;
                          signal q_ds : out std_logic;
                          signal q_sg : out std_logic;
                          signal q_hf : out std_logic;
                          signal q_w  : out unsigned(11 downto 0);
                          signal q_mid : out unsigned(11 downto 0);
                          constant lvl : integer;
                          n_in : t_node) is
            variable h  : unsigned(15 downto 0);
            variable nn : t_node;
            variable o  : std_logic;
            variable t8 : unsigned(7 downto 0);
            variable lo, hi : unsigned(11 downto 0);
            variable sum : unsigned(12 downto 0);
        begin
            h  := hash_lvl(n_in.seed, s_sub_seed, C_LC(lvl));
            nn := n_in; nn.seed := h;
            -- Piecing (S10): Crazy takes the split axis from the hash, Log Cabin
            -- alternates it strictly by depth so every patch nests as concentric
            -- frames.  In Log Cabin the axis is a compile-time constant per level,
            -- so it drops off the lo/hi select path entirely.
            if s_logcabin = '1' then
                if (lvl mod 2) = 1 then o := '1'; else o := '0'; end if;
            else
                o := h(0);
            end if;
            -- density tapers with depth.  Kept in unsigned(8) — an unconstrained
            -- integer here inferred a 32-bit subtract + compare in EVERY level.
            if n_in.dens > to_unsigned(lvl * 32, 8) then
                t8 := n_in.dens - to_unsigned(lvl * 32, 8);
            else
                t8 := (others => '0');
            end if;
            q_n  <= nn;
            q_or <= o;
            if (n_in.frozen = '0') and (h(15 downto 8) < t8) then
                q_ds <= '1';
            else
                q_ds <= '0';
            end if;
            q_sg <= h(2);
            q_hf <= h(3);
            -- active-axis bounds -> w and mid (parallel to the hash above)
            if o = '0' then lo := n_in.x0; hi := n_in.x1;
            else            lo := n_in.y0; hi := n_in.y1; end if;
            sum   := ('0' & lo) + ('0' & hi);
            q_w   <= hi - lo;
            q_mid <= sum(12 downto 1);                   -- (lo+hi)/2
        end procedure;
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;
            gv(0) <= g_avid;
            for i in 1 to LATENCY - 1 loop gv(i) <= gv(i - 1); end loop;

            -- S0: picture position.  p_lmap_r steps the luma map's per-cell
            -- dens/hide (L_*) three clocks later, so S1c sees them aligned.
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1a: square base cell + per-frame density, and the 2x2 candidate
            -- cells' hashes/aligned origins (for Not-Square overlap).
            case s_g is
                when 5      => v_x0 := s0_x(11 downto 5) & "00000";
                               v_y0 := s0_y(11 downto 5) & "00000";   v_sz := 32;
                when 6      => v_x0 := s0_x(11 downto 6) & "000000";
                               v_y0 := s0_y(11 downto 6) & "000000";  v_sz := 64;
                when 7      => v_x0 := s0_x(11 downto 7) & "0000000";
                               v_y0 := s0_y(11 downto 7) & "0000000"; v_sz := 128;
                when others => v_x0 := s0_x(11 downto 8) & "00000000";
                               v_y0 := s0_y(11 downto 8) & "00000000"; v_sz := 256;
            end case;
            -- square base cell (also the gap fallback in Not-Square).  Its seed
            -- is candidate 0's hash, filled in by S1a2; its dens/hide are
            -- L_own's, read by S1c.
            s1a_base.x0     <= v_x0;
            s1a_base.x1     <= v_x0 + v_sz;
            s1a_base.y0     <= v_y0;
            s1a_base.y1     <= v_y0 + v_sz;
            s1a_base.seed   <= (others => '0');
            s1a_base.dens   <= (others => '0');
            s1a_base.hide   <= '0';
            s1a_base.frozen <= '0';
            -- 2x2 candidate cell origins (current + three up-left neighbours)
            for k in 0 to 3 loop
                v_axk := signed(resize(v_x0, 14)) - to_signed(CK_X(k) * v_sz, 14);
                v_ayk := signed(resize(v_y0, 14)) - to_signed(CK_Y(k) * v_sz, 14);
                s1a_ax(k) <= v_axk;
                s1a_ay(k) <= v_ayk;
            end loop;

            -- S1a2: the four candidate cell hashes, on their own stage so the
            -- origin subtract and the 16-bit init_seed no longer chain inside one
            -- clock.  Candidate 0 IS the base cell, so its hash doubles as the
            -- base node's seed (one hash saved).
            v_bn := s1a_base;
            for k in 0 to 3 loop
                v_hk := init_seed(s_comp_seed,
                                  unsigned(s1a_ax(k)(11 downto 0)),
                                  unsigned(s1a_ay(k)(11 downto 0)));
                s1a2_hk(k) <= v_hk;
                if k = 0 then v_bn.seed := v_hk; end if;
            end loop;
            s1a2_base <= v_bn;
            s1a2_ax   <= s1a_ax;
            s1a2_ay   <= s1a_ay;

            -- S1b: size + jitter each candidate, test 2x2 coverage, and build the
            -- clamped candidate node — all 4 in parallel (no cross-candidate
            -- dependency), so S1c only has to argmax.  Each axis spans 1 or 2 grid
            -- cells (hash bits 7/6); a size-2 cell stays grid-aligned (no jitter)
            -- so its reach never exceeds 2 cells and the 2x2 neighbourhood holds.
            v_pxs   := signed(resize(q_x3, 14));
            v_pys   := signed(resize(q_y3, 14));
            for k in 0 to 3 loop
                v_rank := 0;
                if s1a2_hk(k)(7) = '1' and s1a2_hk(k)(3) = '1' then
                    v_wcx := s_csz2;  v_jx := (others => '0');  v_rank := v_rank + 1;
                else
                    v_wcx := s_csz1;  v_jx := s1a2_hk(k)(11 downto 0) and s_cellmask;
                end if;
                if s1a2_hk(k)(6) = '1' and s1a2_hk(k)(2) = '1' then
                    v_wcy := s_csz2;  v_jy := (others => '0');  v_rank := v_rank + 1;
                else
                    v_wcy := s_csz1;  v_jy := s1a2_hk(k)(15 downto 4) and s_cellmask;
                end if;
                v_x0s := s1a2_ax(k) + signed(resize(v_jx, 14));
                v_y0s := s1a2_ay(k) + signed(resize(v_jy, 14));
                -- Square mode folds in here as "nothing covers", so S1c's select
                -- is a plain zero-test and s_notsq stays off that critical path.
                if s_notsq = '1' and
                   (v_pxs >= v_x0s) and (v_pxs < v_x0s + v_wcx) and
                   (v_pys >= v_y0s) and (v_pys < v_y0s + v_wcy) then
                    s1b_cov(k) <= '1';
                else
                    s1b_cov(k) <= '0';
                end if;
                v_pz(k)          := to_unsigned(v_rank, 2) & s1a2_hk(k)(15 downto 12);
                s1b_cand(k).x0   <= clamp0(v_x0s);
                s1b_cand(k).x1   <= clamp0(v_x0s + v_wcx);
                s1b_cand(k).y0   <= clamp0(v_y0s);
                s1b_cand(k).y1   <= clamp0(v_y0s + v_wcy);
                s1b_cand(k).seed <= s1a2_hk(k);
            end loop;
            -- The coverage-free half of the S1c priority compares, done here in
            -- parallel with the (deeper) coverage test so S1c has no carry chain.
            if v_pz(0) >= v_pz(1) then s1b_p(0) <= '1'; else s1b_p(0) <= '0'; end if;
            if v_pz(0) >= v_pz(2) then s1b_p(1) <= '1'; else s1b_p(1) <= '0'; end if;
            if v_pz(0) >= v_pz(3) then s1b_p(2) <= '1'; else s1b_p(2) <= '0'; end if;
            if v_pz(1) >= v_pz(2) then s1b_p(3) <= '1'; else s1b_p(3) <= '0'; end if;
            if v_pz(1) >= v_pz(3) then s1b_p(4) <= '1'; else s1b_p(4) <= '0'; end if;
            if v_pz(2) >= v_pz(3) then s1b_p(5) <= '1'; else s1b_p(5) <= '0'; end if;
            s1b_base <= s1a2_base;

            -- S1c: argmax over covered candidates; no cover (which includes
            -- Square mode, folded into cov above) -> base.  The priority is
            -- vv = cov & rank & z: coverage first so covered always beats
            -- uncovered, then rank so larger cells win (a big cell owns its
            -- whole footprint and the grid line inside it disappears), then z.
            -- vv(i) >= vv(j) needs no compare here: it is
            --     not cov(j)  or  (cov(i) and pz(i) >= pz(j))
            -- with the pz compares already registered by S1b.  The winner is
            -- the same as the old pairwise tree (0v1, 2v3, then the two
            -- winners, ties to the lower index), one LUT level per step.
            c01 := (not s1b_cov(1)) or (s1b_cov(0) and s1b_p(0));
            c02 := (not s1b_cov(2)) or (s1b_cov(0) and s1b_p(1));
            c03 := (not s1b_cov(3)) or (s1b_cov(0) and s1b_p(2));
            c12 := (not s1b_cov(2)) or (s1b_cov(1) and s1b_p(3));
            c13 := (not s1b_cov(3)) or (s1b_cov(1) and s1b_p(4));
            c23 := (not s1b_cov(3)) or (s1b_cov(2) and s1b_p(5));
            w1 := (not c01) and c12 and c13;
            w2 := (not c02) and (not c12) and c23;
            w3 := (not c03) and (not c13) and (not c23);
            v_kw := to_integer(unsigned'((w2 or w3) & (w1 or w3)));
            -- per-candidate cell dens/hide from the luma map, in candidate order
            -- (own, left, up, up-left); L_* step on S1b's clock (p_lmap_r).
            -- With Luma Mod off they are the plain P12 density and never hide.
            v_dh4(0) := L_own;
            v_dh4(1) := L_lown;
            v_dh4(2) := L_up;
            v_dh4(3) := L_lup;
            if s1b_cov = "0000" then
                s1_n      <= s1b_base;
                s1_n.dens <= L_own.dens;
                s1_n.hide <= L_own.hide;
            else
                s1_n.x0     <= s1b_cand(v_kw).x0;
                s1_n.x1     <= s1b_cand(v_kw).x1;
                s1_n.y0     <= s1b_cand(v_kw).y0;
                s1_n.y1     <= s1b_cand(v_kw).y1;
                s1_n.seed   <= s1b_cand(v_kw).seed;
                -- the winning rectangle's OWN cell decides its piecing and
                -- whether it hides, so it stays whole across grid lines
                s1_n.dens   <= v_dh4(v_kw).dens;
                s1_n.hide   <= v_dh4(v_kw).hide;
                s1_n.frozen <= '0';
            end if;

            -- Level 0 : hash -> decide -> descend
            do_hash(h0_n, h0_or, h0_ds, h0_sg, h0_hf, h0_w, h0_mid, 0, s1_n);
            dtmp := calc_split(h0_w, h0_mid, h0_ds, h0_sg, h0_hf, C_JIT_MODE);
            d0_n <= h0_n; d0_or <= h0_or; d0_dt <= dtmp.doit; d0_nx <= dtmp.nx;
            n0 <= descend(d0_n, d0_or, d0_dt, d0_nx, q_x7, q_y7);

            -- Level 1
            do_hash(h1_n, h1_or, h1_ds, h1_sg, h1_hf, h1_w, h1_mid, 1, n0);
            dtmp := calc_split(h1_w, h1_mid, h1_ds, h1_sg, h1_hf, C_JIT_MODE);
            d1_n <= h1_n; d1_or <= h1_or; d1_dt <= dtmp.doit; d1_nx <= dtmp.nx;
            n1 <= descend(d1_n, d1_or, d1_dt, d1_nx, q_x10, q_y10);

            -- Level 2
            do_hash(h2_n, h2_or, h2_ds, h2_sg, h2_hf, h2_w, h2_mid, 2, n1);
            dtmp := calc_split(h2_w, h2_mid, h2_ds, h2_sg, h2_hf, C_JIT_MODE);
            d2_n <= h2_n; d2_or <= h2_or; d2_dt <= dtmp.doit; d2_nx <= dtmp.nx;
            n2 <= descend(d2_n, d2_or, d2_dt, d2_nx, q_x13, q_y13);

            -- Level 3
            do_hash(h3_n, h3_or, h3_ds, h3_sg, h3_hf, h3_w, h3_mid, 3, n2);
            dtmp := calc_split(h3_w, h3_mid, h3_ds, h3_sg, h3_hf, C_JIT_MODE);
            d3_n <= h3_n; d3_or <= h3_or; d3_dt <= dtmp.doit; d3_nx <= dtmp.nx;
            n3 <= descend(d3_n, d3_or, d3_dt, d3_nx, q_x16, q_y16);

            -- S10a: leaf-local coords + per-axis edge mins
            n  := n3;
            lw := n.x1 - n.x0;
            lh := n.y1 - n.y0;
            lx := q_x17 - n.x0;
            ly := q_y17 - n.y0;
            bb := (lw - 1) - lx;
            dd := (lh - 1) - ly;
            if lx < bb then m1 := lx; else m1 := bb; end if;
            if ly < dd then m2 := ly; else m2 := dd; end if;
            -- Evolve, part 1: the leaf's ring slot.  RANDOM (K3 right) mixes
            -- all 16 seed bits so consecutive slots do NOT share a base colour —
            -- k(7:4) becomes colour xor texture.  Feeding raw seed bits would
            -- put the colour field in k's low bits and the cursor would visibly
            -- sweep the palette in order.  SNAKE (K3 left) is built over the next
            -- two stages from the leaf's centre, so its doubled centre is
            -- registered here.
            v_kmix := n.seed(3 downto 0) & n.seed(15 downto 8);
            v_krnd := n.seed(15 downto 12) & (n.seed(11 downto 0) xor v_kmix);
            s10a_lf.lx    <= lx;
            s10a_lf.ly    <= ly;
            s10a_lf.m1    <= m1;
            s10a_lf.m2    <= m2;
            s10a_lf.krnd  <= v_krnd;
            s10a_lf.cbase <= n.seed(7 downto 4);
            s10a_lf.tbase <= n.seed(11 downto 8);
            s10a_lf.p     <= n.seed(12);
            s10a_lf.hide  <= n.hide;
            s10a_sx <= resize(n.x0, 13) + resize(n.x1, 13);
            s10a_sy <= resize(n.y0, 13) + resize(n.y1, 13);

            -- S10h / S10h2: SNAKE slot = the Hilbert-curve index of the leaf's
            -- centre on a 16 px grid.  The screen is two 64x64-cell squares side
            -- by side (1024 px each; the rows below 1024 fold into the last row),
            -- each walked by a Hilbert curve, the second square entered next to
            -- where the first is left.  Consecutive slots are adjacent 16 px cells
            -- and the curve fills small regions before moving on, so the change
            -- hops from each piece to one it touches and meanders instead of
            -- scanning rows.  Equal slots (tiny neighbouring pieces) just change
            -- together.  The square order and each square's orientation come
            -- from the lap count, so every lap takes a different path.
            -- S10h: levels 5..3.
            v_hx := s10a_sx(10 downto 5);
            v_hy := s10a_sy(10 downto 5);
            if s10a_sy(12) = '1' or s10a_sy(11) = '1' then v_hy := (others => '1'); end if;
            if s10a_sx(11) = '1' then v_ho := s_hstr(1); v_hc := s_hstr(0);
            else                      v_ho := s_hstl(1); v_hc := s_hstl(0); end if;
            for L in 5 downto 3 loop
                v_hs := hil_step(v_hx(L), v_hy(L), v_ho, v_hc);
                v_dhi(2*(L-3)+1 downto 2*(L-3)) := unsigned(v_hs(3 downto 2));
                v_ho := v_hs(1);
                v_hc := v_hs(0);
            end loop;
            s10h_dhi <= v_dhi;
            s10h_st  <= v_ho & v_hc;
            s10h_xl  <= v_hx(2 downto 0);
            s10h_yl  <= v_hy(2 downto 0);
            s10h_ord <= s10a_sx(11) xor s_hdir;
            s10h_lf  <= s10a_lf;

            -- S10h, in parallel: the leaf's seam distance, puff ramp and texture
            -- coordinate features from its local coords (lx/ly/m1/m2 go no
            -- further).  Texture coords are read as small fixed bit-slices
            -- (see below); S10b only picks the texture by number.
            if s10a_lf.m1 < s10a_lf.m2 then seamd := s10a_lf.m1; else seamd := s10a_lf.m2; end if;
            -- (min(m1,m2) < SEAM_W, with the horizontal seam widened to one
            -- field line at 480i so SD never skips it)
            if s10a_lf.m1 < to_unsigned(SEAM_W, 12) or s10a_lf.m2 < resize(s_seamy, 12) then
                v_tx.seam := '1';
            else
                v_tx.seam := '0';
            end if;
            -- Puff ramp: 15 at the patch edge falling to 0 at a radius that
            -- follows the grid scale (8 / 16 / 16 / 32 px), read as fixed slices
            -- of the edge distance.  Puffed mode only.
            v_t4 := (others => '0');
            case s_g is
                when 5 =>
                    if seamd(11 downto 3) = 0 then v_t4 := (not seamd(2 downto 0)) & '1'; end if;
                when 6 | 7 =>
                    if seamd(11 downto 4) = 0 then v_t4 := not seamd(3 downto 0); end if;
                when others =>
                    if seamd(11 downto 5) = 0 then v_t4 := not seamd(4 downto 1); end if;
            end case;
            if s_relief = '0' then v_t4 := (others => '0'); end if;
            v_tx.puff := v_t4;

            vdiag   := s10a_lf.lx + s10a_lf.ly;
            v_adiag := s10a_lf.lx + (not s10a_lf.ly);
            -- All texture coords are read as small fixed bit-slices selected by the
            -- Texture knob (a 4:1 mux), NOT 12-bit barrel shifters — the texture
            -- only needs a 2-bit ramp field (coord>>(s-1))(1:0) and a couple of
            -- single bits.  This both scales every pattern with the knob and keeps
            -- HD timing (barrel shifters here were the congestion bottleneck).
            --   v_r* = 2-bit ramp field at [s:s-1]; v_*b = bit s; v_w*b = bit s+1.
            case s_tex_shift is
                when 3 =>
                    v_rx := s10a_lf.lx(3 downto 2); v_ry := s10a_lf.ly(3 downto 2);
                    v_rd := vdiag(3 downto 2);   v_ra := v_adiag(3 downto 2);
                    v_bxb := s10a_lf.lx(3); v_byb := s10a_lf.ly(3);
                    v_wxb := s10a_lf.lx(4); v_wyb := s10a_lf.ly(4);
                    v_drow := s10a_lf.ly(2 downto 1); v_dcol := s10a_lf.lx(2 downto 1);
                    v_pm := to_unsigned(7, 12);
                when 4 =>
                    v_rx := s10a_lf.lx(4 downto 3); v_ry := s10a_lf.ly(4 downto 3);
                    v_rd := vdiag(4 downto 3);   v_ra := v_adiag(4 downto 3);
                    v_bxb := s10a_lf.lx(4); v_byb := s10a_lf.ly(4);
                    v_wxb := s10a_lf.lx(5); v_wyb := s10a_lf.ly(5);
                    v_drow := s10a_lf.ly(3 downto 2); v_dcol := s10a_lf.lx(3 downto 2);
                    v_pm := to_unsigned(15, 12);
                when 5 =>
                    v_rx := s10a_lf.lx(5 downto 4); v_ry := s10a_lf.ly(5 downto 4);
                    v_rd := vdiag(5 downto 4);   v_ra := v_adiag(5 downto 4);
                    v_bxb := s10a_lf.lx(5); v_byb := s10a_lf.ly(5);
                    v_wxb := s10a_lf.lx(6); v_wyb := s10a_lf.ly(6);
                    v_drow := s10a_lf.ly(4 downto 3); v_dcol := s10a_lf.lx(4 downto 3);
                    v_pm := to_unsigned(31, 12);
                when others =>
                    v_rx := s10a_lf.lx(6 downto 5); v_ry := s10a_lf.ly(6 downto 5);
                    v_rd := vdiag(6 downto 5);   v_ra := v_adiag(6 downto 5);
                    v_bxb := s10a_lf.lx(6); v_byb := s10a_lf.ly(6);
                    v_wxb := s10a_lf.lx(7); v_wyb := s10a_lf.ly(7);
                    v_drow := s10a_lf.ly(5 downto 4); v_dcol := s10a_lf.lx(5 downto 4);
                    v_pm := to_unsigned(63, 12);
            end case;
            bidx := to_integer(v_drow & v_dcol);
            v_bv := C_BAYER(bidx);
            v_nv := (s10a_lf.lx and v_pm) < 2;    -- near a vertical cell line
            v_nh := (s10a_lf.ly and v_pm) < resize(s_thyt, 12);   -- near a horizontal cell line (>= 1 SD field line)
            v_tx.rx := v_rx;  v_tx.ry := v_ry;  v_tx.rd := v_rd;  v_tx.ra := v_ra;
            if v_bxb = v_byb then v_tx.chk := '1'; else v_tx.chk := '0'; end if;
            v_tx.wxy := v_wxb xor v_wyb;
            v_tx.wyb := v_wyb;
            v_tx.bv  := v_bv;
            if v_nv then v_tx.nv := '1'; else v_tx.nv := '0'; end if;
            if v_nh then v_tx.nh := '1'; else v_tx.nh := '0'; end if;
            s10h_tx <= v_tx;

            -- S10h2: levels 2..0, then pick the slot for the mode.
            v_ho := s10h_st(1);
            v_hc := s10h_st(0);
            for L in 2 downto 0 loop
                v_hs := hil_step(s10h_xl(L), s10h_yl(L), v_ho, v_hc);
                v_dlo(2*L+1 downto 2*L) := unsigned(v_hs(3 downto 2));
                v_ho := v_hs(1);
                v_hc := v_hs(0);
            end loop;
            v_ks(15)           := s10h_ord;
            v_ks(14 downto 9)  := s10h_dhi;
            v_ks(8 downto 3)   := v_dlo;
            v_ks(2 downto 0)   := "000";
            if s_evo_snake = '1' then s10h2_k <= v_ks;
            else                      s10h2_k <= s10h_lf.krnd; end if;
            s10h2_lf <= s10h_lf;
            s10h2_tx <= s10h_tx;

            -- Evolve, part 2 (S10a2): gen = pass + (k < pos).  The colour and
            -- texture generations alternate off the per-leaf phase bit, so each
            -- step of gen moves EXACTLY ONE of them:
            --     p=0: cgen = gen,   tgen = gen+1
            --     p=1: cgen = gen+1, tgen = gen
            -- Each is one of pass/pass+1/pass+2, already sliced per frame into
            -- s_evo_g, so this is a 3:1 mux — no arithmetic on gen at pixel rate.
            -- The indices then just advance the leaf's base colour / texture
            -- number, which wraps in 4 bits for free.
            v_ic := 0;
            v_it := 0;
            if s10h2_k < s_evo_pos then v_ic := v_ic + 1; v_it := v_it + 1; end if;
            if s10h2_lf.p = '1' then v_ic := v_ic + 1; else v_it := v_it + 1; end if;
            -- No gate on K3 centre: a zero rate stops the phase, so centre
            -- freezes the quilt as it stands instead of snapping it back.
            v_cg := s_evo_g(v_ic);
            v_tg := s_evo_g(v_it);
            s10a2_tx   <= s10h2_tx;
            s10a2_cidx <= s10h2_lf.cbase + v_cg;
            s10a2_tidx <= s10h2_lf.tbase + v_tg;
            s10a2_hide <= s10h2_lf.hide;

            -- S10b: texture emboss shade.  The texture is picked HERE (one stage
            -- ahead of the palette fetch) so the 16-way pattern logic and the
            -- palette muxes don't share a stage.  Texture -> 2-bit shade:
            -- stripe/diagonal types ramp across the period (rounded "thread" ->
            -- depth); the rest are deep binary.
            v_t2 := "01";
            case to_integer(s10a2_tidx) is
                when 0  => v_t2 := "01";                                       -- solid
                when 1  => v_t2 := s10a2_tx.rx;                                -- v ridges
                when 2  => v_t2 := s10a2_tx.ry;                                -- h ridges
                when 3  => v_t2 := s10a2_tx.rd;                                -- diagonal
                when 4  => v_t2 := s10a2_tx.ra;                                -- anti-diagonal
                when 5  =>                                                     -- checker (deep)
                    if s10a2_tx.chk = '1' then v_t2 := "00"; else v_t2 := "11"; end if;
                when 6  =>                                                     -- cross-hatch
                    if s10a2_tx.nv = '1' or s10a2_tx.nh = '1' then v_t2 := "11"; else v_t2 := "00"; end if;
                when 7  => v_t2 := s10a2_tx.bv(3 downto 2);                           -- ordered dither
                when 8  =>                                                     -- dots
                    if s10a2_tx.bv > 11 then v_t2 := "11"; elsif s10a2_tx.bv > 6 then v_t2 := "10";
                    else v_t2 := "00"; end if;
                when 9  =>                                                     -- v pinstripe
                    if s10a2_tx.nv = '1' then v_t2 := "11"; else v_t2 := "01"; end if;
                when 10 =>                                                     -- h pinstripe
                    if s10a2_tx.nh = '1' then v_t2 := "11"; else v_t2 := "01"; end if;
                when 11 =>                                                     -- basketweave
                    if s10a2_tx.wxy = '0' then v_t2 := s10a2_tx.ry;
                    else v_t2 := s10a2_tx.rx; end if;
                when 12 =>                                                     -- herringbone
                    if s10a2_tx.wyb = '0' then v_t2 := s10a2_tx.rd;
                    else v_t2 := s10a2_tx.ra; end if;
                when 13 =>                                                     -- woven grid
                    if s10a2_tx.nv = '1' or s10a2_tx.nh = '1' then v_t2 := "11"; else v_t2 := s10a2_tx.rx; end if;
                when 14 => v_t2 := s10a2_tx.rd;                                -- diagonal (alt)
                when others => v_t2 := s10a2_tx.ry;                            -- h ridges (alt)
            end case;
            s10_r.tval <= v_t2;
            s10_r.seam <= s10a2_tx.seam;
            s10_r.puff <= s10a2_tx.puff;
            s10_r.pidx <= s10a2_cidx xor s_tint;
            s10_r.hide <= s10a2_hide;

            -- IO-align pads: 3 stages that bring the total latency to a
            -- multiple of 4 (24 + 3 + s_io = 28), placed where the pixel is
            -- 12 bits wide.  The per-frame controls read after them (S11/S12)
            -- only change at vsync, so their 3-clock-later read is invisible.
            s10_pad(0) <= s10_r;
            s10_pad(1) <= s10_pad(0);
            s10_pad(2) <= s10_pad(1);

            -- S11: palette fetch (texture shade just rides through)
            pidx := to_integer(s10_pad(2).pidx);
            if s_palmode = '0' then
                ty := FAB_Y(pidx); tu := FAB_U(pidx); tv := FAB_V(pidx);
            else
                ty := RNB_Y(pidx); tu := RNB_U(pidx); tv := RNB_V(pidx);
            end if;
            s11_y <= ty; s11_u <= tu; s11_v <= tv;
            -- Emboss + puff as one signed luma delta, so S12 is a single add.
            -- The 2-bit texture shade: 00 highlight, 01 flat, 10 shadow, 11
            -- deep shadow.  S8 Relief: Flat halves the emboss and has no puff;
            -- Puffed is the full emboss plus the puff shading.
            if s_relief = '1' then
                case s10_pad(2).tval is
                    when "00"   => v_emb := to_signed(TEX_HI, 11);
                    when "10"   => v_emb := to_signed(-TEX_D1, 11);
                    when "11"   => v_emb := to_signed(-TEX_D2, 11);
                    when others => v_emb := (others => '0');
                end case;
            else
                case s10_pad(2).tval is
                    when "00"   => v_emb := to_signed(TEX_HI / 2, 11);
                    when "10"   => v_emb := to_signed(-(TEX_D1 / 2), 11);
                    when "11"   => v_emb := to_signed(-(TEX_D2 / 2), 11);
                    when others => v_emb := (others => '0');
                end case;
            end if;
            s11_delta <= v_emb - signed(resize(C_PUFF(to_integer(s10_pad(2).puff)), 11));
            s11_seam  <= s10_pad(2).seam;
            s11_hide  <= s10_pad(2).hide;

            -- S12: assemble colour: seam, else palette luma + emboss/puff delta,
            -- held inside 64..880 (black up to the brightest palette entry).
            if s_seam_en = '1' and s11_seam = '1' then
                s12_y <= C_BLK;
                s12_u <= C_MID;
                s12_v <= C_MID;
            else
                v_sum := signed(resize(s11_y, 12)) + resize(s11_delta, 12);
                if v_sum < to_signed(C_YBLK, 12) then
                    s12_y <= C_BLK;
                elsif v_sum > to_signed(C_YCAP, 12) then
                    s12_y <= to_unsigned(C_YCAP, 10);
                else
                    s12_y <= unsigned(v_sum(9 downto 0));
                end if;
                s12_u <= s11_u;
                s12_v <= s11_v;
            end if;
            s12_hide <= s11_hide;

            -- S_out: register output.  Blanking gate: neutral black outside
            -- the (latency-aligned) generated avid.  Luma Mod's hidden cells are
            -- the same neutral black.
            if gv(LATENCY - 1) = '0' or s12_hide = '1' then
                s_io.y <= std_logic_vector(C_BLK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(s12_y);
                s_io.u <= std_logic_vector(s12_u);
                s_io.v <= std_logic_vector(s12_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= gv(LATENCY - 1);
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    -- (the IO-align pads that bring the latency to 28 are in the pipeline,
    -- after S10b: s10_pad)
    data_out <= s_io;

end architecture quilt;
