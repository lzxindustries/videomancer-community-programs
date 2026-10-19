-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- mondrian.vhd  (v1.0.0 — De Stijl composition generator)
--
-- v1.0.2 (2026-10-17): resolution scaling.  The position and cell counters
-- run in virtual 1080-line units, stepped per pixel / per field line from the
-- video timing ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5
-- at 720p, and at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per
-- frame line, so cells, seams and the composition keep their proportions and
-- stay square on a 4:3 screen.  Interlaced fields step two frame lines and the
-- bottom field starts half a step down.  Horizontal seams keep a half-width
-- of at least half a step so they can't fall between SD field lines.
--
-- v1.0.0 (2026-10-10): FINAL, signed off on hardware.  Same logic as v0.2.3; the
-- defaults are captured from the hardware.
--
-- v0.2 (2026-10-10): hardware-contract pass.  Declared "processing" (S10/S11
-- read data_in).  Own generated avid (parity-locked start, field-latched
-- width) drives the pixel counters and data_out.avid, and y/u/v are gated to
-- 64/512/512 outside it.  Video Fill is delayed in a 16-deep EBR ring, not
-- the FF pipe (raw video stays shallow).  Black is 64, never below.  K1 spans
-- 32..256 px over the whole travel; K6 is a continuous 2..32 px line; K4
-- reads a labelled-default INDEX (0..3) as the label itself.
--
-- v0.2.1: S11 Luma Mod no longer blacks out dark cells.  The full white
-- composition (blocks + lines) always shows; a block takes its colour only
-- where the input luma is bright, else it stays white.  Merged cells follow
-- the luma of the base cell at their origin, so a rectangle is all colour or
-- all white.
--
-- v0.2.3: K3 is now Luma Level, the Luma Mod line between colour and white
-- (0% = colour everywhere but black, 50% = mid-grey, 100% = no colour).  It
-- replaces K3 Sub-Pieces: the sub-piece seed is a constant (K3's old default),
-- and K2 still re-rolls the sub-pieces through each cell's root seed.  K5 is
-- Colour Amount only again (v0.2.2's K5-driven threshold is gone).
--
-- Paints the screen in the style of Piet Mondrian: a mostly-white canvas carved
-- into clean axis-aligned rectangles by thick black grid lines, with a scattering
-- of blocks filled in primary red, yellow and blue (and, optionally, black).
--
-- This is a repaint of the "quilt" program: the recursive binary-subdivision
-- pipeline that carves each coarse grid cell into nested sub-rectangles is reused
-- (no per-pixel multiply, no line buffer, no BRAM).  Only the *leaf colouring* and
-- the *seam rendering* differ:
--   * Quilt's uniform 16-colour palette pick becomes a WEIGHTED pick that favours
--     the white background; only a fraction of leaves take a primary colour.
--   * Quilt's thin grey seams become thick PURE-BLACK grid lines (Line Width knob).
--   * Texture is dropped — Mondrian blocks are flat colour.
--
-- S9 Not-Square = cell MERGING: in a base grid, some cells grow to span their
-- right / down / right-down neighbour (2x1, 1x2 or 2x2).  The larger cell wins the
-- overlap and renders FLAT, so the grid line between the fused cells disappears and
-- a larger rectangle forms.  Cells stay grid-aligned, so coverage is pure hash-bit
-- logic (no coordinate compares) — cheap enough to close HD timing.  Square mode
-- (S9=0) keeps every cell 1x1 = the plain grid.
--
-- Controls: K1 Grid Scale, K2 Composition, K3 Luma Level, K4 Split Jitter,
-- K5 Colour Amount, K6 Line Width, S7 Grid Lines, S8 Black Blocks, S9 Grid
-- (Square / Not-Square cell merge), S10 Video Fill, S11 Luma Mod (the whole
-- composition stays; blocks take colour only where input luma is bright, else
-- they stay white), P12 Density.
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

architecture mondrian of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (18 + 2 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY : natural := 17;
    constant DMAX    : natural := 3;          -- subdivision levels
    constant MIN_W   : natural := 8;          -- don't split a span narrower than this

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant LINE_Y  : unsigned(9 downto 0) := to_unsigned(64, 10);   -- grid-line luma (video black)
    constant C_BLK   : unsigned(9 downto 0) := to_unsigned(64, 10);   -- video black (never below)

    -- per-level hash mixing constants (compile-time)
    type t_lc is array (0 to DMAX - 1) of unsigned(15 downto 0);
    constant C_LC : t_lc := (x"9E37", x"7C15", x"D2B9");

    --------------------------------------------------------------------------
    -- Pipeline node: running rectangle bounds + seed + freeze + pixel coords.
    --------------------------------------------------------------------------
    type t_node is record
        x0, x1, y0, y1 : unsigned(11 downto 0);
        px, py         : unsigned(11 downto 0);
        seed           : unsigned(15 downto 0);
        frozen         : std_logic;
    end record;

    -- decide-stage result: split coordinate + "do it" flag.
    type t_dec is record
        nx   : unsigned(11 downto 0);
        doit : std_logic;
    end record;

    --------------------------------------------------------------------------
    -- Not-Square cell merge: the 2x2 up-left candidate neighbourhood.  A cell can
    -- grow right/down by one cell, so a pixel can be covered by its own cell or one
    -- of the three up-left neighbours grown toward it.  Order: (gx,gy) (gx-1,gy)
    -- (gx,gy-1) (gx-1,gy-1).
    --------------------------------------------------------------------------
    type t_u16_4 is array (0 to 3) of unsigned(15 downto 0);
    constant CK_X : integer_vector(0 to 3) := (0, 1, 0, 1);
    constant CK_Y : integer_vector(0 to 3) := (0, 0, 1, 1);

    --------------------------------------------------------------------------
    -- Mondrian palette.  index 0=White 1=Red 2=Yellow 3=Blue 4=Black.  Stored with
    -- U/V (Cb/Cr) SWAPPED to match the Videomancer output convention (matching the
    -- verified phosphor/CGA/C64 palettes).  Pre-swap BT.601 chroma was:
    --   Red  U=417 V=843 | Yellow U=168 V=645 | Blue U=729 V=410
    --------------------------------------------------------------------------
    constant PAL_WHITE  : integer := 0;
    constant PAL_RED    : integer := 1;
    constant PAL_YELLOW : integer := 2;
    constant PAL_BLUE   : integer := 3;
    constant PAL_BLACK  : integer := 4;

    type t_pal is array (0 to 4) of unsigned(9 downto 0);
    constant MON_Y : t_pal := (
        to_unsigned(940,10), to_unsigned(332,10), to_unsigned(729,10),
        to_unsigned(272,10), to_unsigned( 64,10));
    constant MON_U : t_pal := (
        to_unsigned(512,10), to_unsigned(843,10), to_unsigned(645,10),
        to_unsigned(410,10), to_unsigned(512,10));
    constant MON_V : t_pal := (
        to_unsigned(512,10), to_unsigned(417,10), to_unsigned(168,10),
        to_unsigned(729,10), to_unsigned(512,10));

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

    -- Per-cell hash.  Uses one ADD (carry propagation) amid rotates/xors, NOT just
    -- xor/shift: a purely linear (xor/shift) hash of (x0,y0) leaves the grow-bit
    -- field linearly structured, so whole rows / diagonals of cells merge together.
    -- The single add makes it nonlinear, scattering the merges randomly, while
    -- keeping the hash short enough to close HD timing on its own pipeline stage.
    function init_seed(comp : unsigned(15 downto 0); x0, y0 : unsigned(11 downto 0))
        return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := comp xor resize(x0, 16) xor rotate_left(resize(y0, 16), 5);
        h := h + rotate_left(h, 7);          -- single nonlinear add (carry mixing)
        h := h xor rotate_left(h, 11);
        h := h xor shift_right(h, 6);
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

    -- Decide stage: compute the split coordinate for the active axis.  Jitter
    -- magnitude is at most 3w/8 < w/2, so nx is always strictly inside (lo,hi).
    function calc_split(n        : t_node;
                        orient   : std_logic;
                        dosplit  : std_logic;
                        sgn      : std_logic;
                        half     : std_logic;
                        jit_mode : unsigned(1 downto 0)) return t_dec is
        variable lo, hi, w, mag, mid : unsigned(11 downto 0);
        variable r : t_dec;
    begin
        if orient = '0' then lo := n.x0; hi := n.x1;
        else                 lo := n.y0; hi := n.y1; end if;
        w := hi - lo;
        case jit_mode is
            when "00"   => mag := (others => '0');
            when "01"   => mag := "000" & w(11 downto 3);                     -- w/8
            when "10"   => mag := "00"  & w(11 downto 2);                     -- w/4
            when others => mag := ("00" & w(11 downto 2))
                                + ("000" & w(11 downto 3));                   -- 3w/8
        end case;
        if half = '1' then mag := '0' & mag(11 downto 1); end if;
        mid := lo + ('0' & w(11 downto 1));                                  -- lo + w/2
        if sgn = '1' then r.nx := mid - mag;
        else              r.nx := mid + mag; end if;
        if (dosplit = '1') and (n.frozen = '0') and (w >= to_unsigned(MIN_W, 12)) then
            r.doit := '1';
        else
            r.doit := '0';
        end if;
        return r;
    end function;

    -- Descend stage: step into the pixel's half and fold the choice into the seed.
    function descend(n : t_node; orient, doit : std_logic; nx : unsigned(11 downto 0))
        return t_node is
        variable r : t_node;
        variable lo, hi, pix : unsigned(11 downto 0);
    begin
        r := n;
        if doit = '1' then
            if orient = '0' then lo := n.x0; hi := n.x1; pix := n.px;
            else                 lo := n.y0; hi := n.y1; pix := n.py; end if;
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

    --------------------------------------------------------------------------
    -- Position + per-frame parameters
    --------------------------------------------------------------------------
    signal pixel_x, pixel_y : unsigned(11 downto 0) := (others => '0');

    -- Resolution scaling: the counters below run in virtual 1080-line units.
    -- x accumulators are Q.1 (half units; steps 2/3/4 = 1.0/1.5/2.0 per
    -- pixel), y accumulators Q.8 (per field line).  pixel_x/pixel_y and the
    -- cell origins are the integer parts.
    signal s_xs   : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal pix_xa : unsigned(12 downto 0) := (others => '0');
    signal pix_ya : unsigned(19 downto 0) := (others => '0');
    signal cellx_la : unsigned(12 downto 0) := (others => '0');
    signal celly_la : unsigned(19 downto 0) := (others => '0');
    signal s_cx_new : std_logic := '1';   -- first pixel of a cell column
    signal s_cx_ctr : std_logic := '0';   -- this pixel crossed the cell centre
    signal s_cy_new : std_logic := '1';   -- first line of a cell row
    signal s_lwy    : unsigned(11 downto 0) := to_unsigned(3, 12);  -- horizontal-seam half-width
    signal prev_hsync_n     : std_logic := '1';
    signal prev_vsync_n     : std_logic := '1';

    -- Continuous-scale cell counters (advanced as the raster scans -> arbitrary
    -- cell sizes with no divide/multiply).  org = cell origin px, loc = position
    -- within cell, idx = cell column index.
    signal cellx_org : unsigned(11 downto 0) := (others => '0');
    signal cellx_idx : unsigned(6 downto 0)  := (others => '0');
    signal celly_org : unsigned(11 downto 0) := (others => '0');

    -- Continuous grid scale: cell size is any 32..256 px.  Stored as unsigned (and
    -- pre-derived 2x / size-1 / centre) ONCE per frame so the per-pixel datapath
    -- never re-derives them with to_unsigned — keeps the netlist sparse for routing.
    signal s_csz   : unsigned(11 downto 0) := to_unsigned(64, 12);   -- cell size
    signal s_csz2  : unsigned(11 downto 0) := to_unsigned(128, 12);  -- 2x cell size
    signal s_cszc  : unsigned(11 downto 0) := to_unsigned(32, 12);   -- cell centre
    signal s_density    : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_jit_mode   : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal s_color_amt  : unsigned(7 downto 0) := to_unsigned(55, 8);   -- white<->colour threshold
    signal s_line_w     : unsigned(11 downto 0) := to_unsigned(3, 12);  -- grid-line half-width (px)
    signal s_comp_seed  : unsigned(15 downto 0) := x"1234";
    -- sub-piece seed: fixed at the value K3 Sub-Pieces gave at its old default
    -- (mix16(620, x"C2B5")), so the default painting is unchanged
    constant C_SUB_SEED : unsigned(15 downto 0) := x"A30F";
    signal s_seam_en    : std_logic := '1';
    signal s_blackblk   : std_logic := '0';
    signal s_notsq      : std_logic := '0';   -- S9: 0 = square grid, 1 = cell merge
    signal s_vid_en     : std_logic := '0';
    signal s_lumamod    : std_logic := '0';   -- S11: colour only where cell luma > threshold, else white
    -- Luma Mod line, set by K3 Luma Level: T = 64 + r - r/8 (r = K3), so
    -- 64 (black) at 0%, 512 (mid-grey) at 50%, 960 (white) at 100%; a cell is
    -- coloured when its luma is above T.  Two free-running stages off the SPI
    -- register, latched at vsync.
    signal s_lsum       : unsigned(9 downto 0) := (others => '0');
    signal s_lthr_f     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_luma_thr   : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- K1 cell size, free-running off the SPI register (32..256 over the whole
    -- travel: 32 + r/4 - r/32) so the vsync latch is only a -1 and two shifts.
    signal s_k1sz       : unsigned(11 downto 0) := to_unsigned(144, 12);

    --------------------------------------------------------------------------
    -- Generated avid (cubist/aurora recipe).  The encoders get no data-enable:
    -- they pair Cb/Cr by counting from hsync, while the core's 444->422 packer
    -- restarts on Cb at every avid rise.  A source avid that wanders a clock
    -- against hsync swaps U/V for that line -- thin coloured lines over the
    -- flat primaries.  So the program makes its own avid: it starts 2 or 3
    -- clocks after the source's first avid rise of the line, whichever keeps
    -- the hsync->start PARITY at the source's usual value (saturating vote),
    -- and ends after the field-latched line width (or at hsync at the latest).
    -- It drives the pixel counters, data_out.avid and the blanking gate.
    --------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);  -- width - 1, per field
    signal s_xcnt : unsigned(11 downto 0) := (others => '0');        -- source avid run
    signal s_wlin : unsigned(11 downto 0) := to_unsigned(1920, 12);  -- last line's width

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    signal s0_x, s0_y   : unsigned(11 downto 0);
    signal s0_x0, s0_y0 : unsigned(11 downto 0);   -- cell origin (px) for this pixel

    -- Per-cell Luma Mod: one dark bit per base-cell column for the current cell
    -- row (cell_hide) and the row above (cell_hide_up), sampled at the cell
    -- centre on each cell-row's top line (and on the field's first active line,
    -- for a top row that starts in vblank) and held across the cell.  A merged
    -- (Not-Square) cell takes the bit of the base cell at its ORIGIN -- left
    -- and/or up of the pixel -- so a fused rectangle is coloured or white as one.
    -- The winner's bit rides hide_pipe from S1b (stage 2) to S10a (stage 13),
    -- where the colour pick turns a dark cell's blocks white.
    signal cell_hide    : std_logic_vector(0 to 63) := (others => '0');
    signal cell_hide_up : std_logic_vector(0 to 63) := (others => '0');
    signal s0_hc, s0_hu, s0_hcl, s0_hul : std_logic := '0';  -- this/left column, this/up row
    signal s1a_h        : std_logic_vector(0 to 3) := (others => '0');   -- per 2x2 candidate
    signal hide_pipe    : std_logic_vector(2 to 13) := (others => '0');
    signal s_first      : std_logic := '0';   -- first active line of the field
    signal s_firstav    : std_logic := '0';

    -- generated avid, aligned with the colour pipeline (gv_pipe(0) = S0)
    signal gv_pipe : std_logic_vector(0 to LATENCY - 1) := (others => '0');

    -- Video Fill delay: a 16-deep EBR circular buffer, NOT the FF pipe.  Raw
    -- input carried ~15 FFs deep has gone green/noisy on HD HDMI (cascade,
    -- rollthebones), and reading pipe(LATENCY-1).y/u/v kept ~500 delay FFs
    -- alive.  Read pointer = write pointer - 15, so after the EBR output
    -- register and one fabric re-register the sample equals pipe(LATENCY-1).
    -- The read never hits the address being written -> no_rw_check.
    type t_vdly is array (0 to 15) of std_logic_vector(31 downto 0);
    signal vdly_ram : t_vdly := (others => (others => '0'));
    signal vdly_wa  : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal vdly_ra  : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal vdly_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal vdly_q2  : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of vdly_ram : signal is true;

    -- S1a: 2x2 candidate hashes + base origin + edge validity (hash folded into
    -- S1a to drop the separate S1a2 stage's registers and ease routing).
    signal s1a_hk            : t_u16_4;
    signal s1a_x0, s1a_y0    : unsigned(11 downto 0);
    signal s1a_px, s1a_py    : unsigned(11 downto 0);
    signal s1a_left_ok       : std_logic;
    signal s1a_up_ok         : std_logic;

    -- S1b: winning candidate, selected (node built in S1c to keep paths short).
    signal s1b_hkw           : unsigned(15 downto 0);
    signal s1b_bigx, s1b_bigy : std_logic;
    signal s1b_ckx, s1b_cky  : std_logic;
    signal s1b_x0, s1b_y0    : unsigned(11 downto 0);
    signal s1b_px, s1b_py    : unsigned(11 downto 0);

    signal s1_n : t_node;                                  -- selected (merged) cell node

    -- hash-stage outputs (decoded bits + node carrying new seed)
    signal h0_n, h1_n, h2_n     : t_node;
    signal h0_or, h1_or, h2_or  : std_logic;
    signal h0_ds, h1_ds, h2_ds  : std_logic;
    signal h0_sg, h1_sg, h2_sg  : std_logic;
    signal h0_hf, h1_hf, h2_hf  : std_logic;

    -- decide-stage outputs (node + orient passthrough + split coord + flag)
    signal d0_n, d1_n, d2_n         : t_node;
    signal d0_or, d1_or, d2_or     : std_logic;
    signal d0_dt, d1_dt, d2_dt     : std_logic;
    signal d0_nx, d1_nx, d2_nx     : unsigned(11 downto 0);

    -- descend-stage output nodes
    signal n0, n1, n2 : t_node;

    -- leaf metrics, split across two stages
    signal s10a_m1, s10a_m2 : unsigned(11 downto 0);
    signal s10a_seed        : unsigned(15 downto 0);

    signal s10_seam    : std_logic;
    signal s10_pidx    : unsigned(2 downto 0);   -- 0..4 palette index
    signal s10_iscolor : std_logic;              -- '1' = primary block (video-fill candidate)

    -- palette fetch
    signal s11_y, s11_u, s11_v : unsigned(9 downto 0);
    signal s11_iscolor         : std_logic;
    signal s11_seam            : std_logic;

    -- assembled colour
    signal s12_y, s12_u, s12_v : unsigned(9 downto 0);
    signal s12_vid             : std_logic;      -- this pixel may be video-filled

    -- output
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Position counters + parameter latch (at vsync edge).
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_k : unsigned(9 downto 0);
        variable v_nx : unsigned(12 downto 0);
        variable v_ny : unsigned(19 downto 0);
        variable v_ys : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- K1 Grid Scale, free-running: 32 + r/4 - r/32 = 32..256 px over the
            -- whole travel (the old 32 + r/4 clamped at 256 for the top 12%).
            v_k    := unsigned(registers_in(0));
            s_k1sz <= to_unsigned(32, 12) + resize(v_k(9 downto 2), 12)
                      - resize(v_k(9 downto 5), 12);

            -- Luma Mod line from K3 Luma Level, free-running: 64 + r - r/8
            v_k      := unsigned(registers_in(2));
            s_lsum   <= v_k - shift_right(v_k, 3);              -- 0..896
            s_lthr_f <= s_lsum + to_unsigned(64, 10);           -- 64..960

            -- Source line width (avid run), kept from the last active line; the
            -- generated avid uses it, latched once per field at vsync (stable
            -- through vblank, so a serrated vsync re-latches the same value).
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif g_avq = '1' then
                if s_xcnt > to_unsigned(16, 12) then
                    s_wlin <= s_xcnt;
                end if;
                s_xcnt <= (others => '0');
            end if;

            -- X position + X cell counter (origin / local / column index).  Counting
            -- as we scan gives arbitrary (continuous) cell sizes with no divide.
            -- Counts the GENERATED avid, so the picture and data_out.avid agree.
            -- (Virtual units: at 1080p the step is 1 and this is the old
            -- integer counter; flags replace the old loc = 0 / loc = centre
            -- tests, which a fractional step could jump over.)
            if v_h_edge = '1' then
                pix_xa    <= (others => '0');
                cellx_org <= (others => '0');
                cellx_la  <= (others => '0');
                cellx_idx <= (others => '0');
                s_cx_new  <= '1';
                s_cx_ctr  <= '0';
            elsif g_avid = '1' then
                pix_xa <= pix_xa + s_xs;
                v_nx   := cellx_la + s_xs;
                if v_nx(12 downto 1) >= s_csz then
                    cellx_la  <= v_nx - (s_csz & '0');
                    cellx_org <= cellx_org + s_csz;
                    cellx_idx <= cellx_idx + 1;
                    s_cx_new  <= '1';
                    s_cx_ctr  <= '0';
                else
                    cellx_la <= v_nx;
                    s_cx_new <= '0';
                    if v_nx(12 downto 1) >= s_cszc and cellx_la(12 downto 1) < s_cszc then
                        s_cx_ctr <= '1';
                    else
                        s_cx_ctr <= '0';
                    end if;
                end if;
            end if;

            if g_avid = '1' then s_firstav <= '1'; end if;

            if v_v_edge = '1' then
                -- per-mode steps; the bottom field starts half a field step
                -- (one frame line) down
                v_ys := f_vstep_y(registers_in(8)(3 downto 0));
                s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys <= v_ys;
                if data_in.field_n = '0' then
                    pix_ya   <= resize(v_ys(11 downto 1), 20);
                    celly_la <= resize(v_ys(11 downto 1), 20);
                else
                    pix_ya   <= (others => '0');
                    celly_la <= (others => '0');
                end if;
                celly_org <= (others => '0');
                s_cy_new  <= '1';

                -- Grid Scale (K1): continuous cell size, 32..256 px.
                s_csz   <= s_k1sz;
                s_csz2  <= s_k1sz(10 downto 0) & '0';
                s_cszc  <= '0' & s_k1sz(11 downto 1);

                -- generated-avid width for this field
                g_wm1 <= s_wlin - 1;

                s_comp_seed <= mix16(unsigned(registers_in(1)), x"9E37");

                -- Split Jitter (K4, 4 labels).  At program load the firmware
                -- writes the initial_value_label's INDEX (0..3), not a 0..1023
                -- position, so a value below the label count IS the label.
                v_k := unsigned(registers_in(3));
                if v_k < 4 then
                    s_jit_mode <= v_k(1 downto 0);
                else
                    s_jit_mode <= v_k(9 downto 8);
                end if;

                -- Colour Amount: 8-bit threshold; higher -> more coloured blocks.
                s_color_amt <= unsigned(registers_in(4)(9 downto 2));

                -- Luma Mod line (K3 Luma Level)
                s_luma_thr  <= s_lthr_f;

                -- Line Width (K6): continuous half-width 1..16 px per leaf edge,
                -- so a shared seam is 2..32 px -- bold enough for 1080p.
                s_line_w <= resize(unsigned(registers_in(5)(9 downto 6)), 12) + 1;
                -- horizontal seams: at least half a field step, so a shared
                -- seam always lands on a line (SD 480i: 3 units)
                if resize(unsigned(registers_in(5)(9 downto 6)), 12) + 1 < shift_right(v_ys + 511, 9) then
                    s_lwy <= shift_right(v_ys + 511, 9);
                else
                    s_lwy <= resize(unsigned(registers_in(5)(9 downto 6)), 12) + 1;
                end if;

                s_seam_en  <= registers_in(6)(0);
                s_blackblk <= registers_in(6)(1);
                s_notsq    <= registers_in(6)(2);
                s_vid_en   <= registers_in(6)(3);
                s_lumamod  <= registers_in(6)(4);

                s_density  <= unsigned(registers_in(7)(9 downto 2));

                -- Luma Mod samples the first active line too (idempotent under
                -- a serrated vsync); cleared at the hsync that ends that line
                s_first   <= '1';
                s_firstav <= '0';
            elsif v_h_edge = '1' then
                if s_firstav = '1' then s_first <= '0'; end if;
                pix_ya <= pix_ya + s_ys;
                -- Y cell counter advances per line
                v_ny := celly_la + s_ys;
                if v_ny(19 downto 8) >= s_csz then
                    celly_la  <= v_ny - (s_csz & x"00");
                    celly_org <= celly_org + s_csz;
                    s_cy_new  <= '1';
                else
                    celly_la <= v_ny;
                    s_cy_new <= '0';
                end if;
            end if;
        end if;
    end process p_position;

    pixel_x <= pix_xa(12 downto 1);
    pixel_y <= pix_ya(19 downto 8);

    --------------------------------------------------------------------------
    -- Generated avid (see the signal declarations).  g_ph is the hsync->avid
    -- rise distance parity of this line; g_par is the voted usual parity; a
    -- line whose avid lands off-parity starts one clock later, so the
    -- generated start always sits at the same hsync parity.
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
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_x0, v_y0   : unsigned(11 downto 0);
        variable v_x0L, v_y0U : unsigned(11 downto 0);
        variable v_col        : integer range 0 to 127;
        variable v_bigx, v_bigy, v_cov : std_logic_vector(0 to 3);
        variable v_rnk        : integer range 0 to 2;
        variable vv           : integer_vector(0 to 3);
        variable v_m01, v_m23 : integer;
        variable v_i01, v_i23, v_kw : integer range 0 to 3;
        variable v_wx0, v_wx1, v_wy0, v_wy1 : unsigned(11 downto 0);
        variable dtmp       : t_dec;
        variable n          : t_node;
        variable lw, lh     : unsigned(11 downto 0);
        variable lx, ly     : unsigned(11 downto 0);
        variable bb, dd     : unsigned(11 downto 0);
        variable m1, m2     : unsigned(11 downto 0);
        variable v_colbyte  : unsigned(7 downto 0);
        variable v_sel      : unsigned(1 downto 0);
        variable v_pidx     : integer range 0 to 4;
        variable v_iscol    : std_logic;
        variable pidx       : integer range 0 to 7;
        variable ty, tu, tv : unsigned(9 downto 0);

        -- one hash stage: re-hash seed, decode split controls
        procedure do_hash(signal q_n  : out t_node;
                          signal q_or : out std_logic;
                          signal q_ds : out std_logic;
                          signal q_sg : out std_logic;
                          signal q_hf : out std_logic;
                          constant lvl : integer;
                          n_in : t_node) is
            variable h  : unsigned(15 downto 0);
            variable nn : t_node;
            variable o  : std_logic;
            variable t8 : integer;
        begin
            h  := hash_lvl(n_in.seed, C_SUB_SEED, C_LC(lvl));
            nn := n_in; nn.seed := h;
            o  := h(0);
            t8 := to_integer(s_density) - lvl * 32;      -- density tapers with depth
            if t8 < 0 then t8 := 0; end if;
            q_n  <= nn;
            q_or <= o;
            if (n_in.frozen = '0') and (to_integer(h(15 downto 8)) < t8) then
                q_ds <= '1';
            else
                q_ds <= '0';
            end if;
            q_sg <= h(2);
            q_hf <= h(3);
        end procedure;
    begin
        if rising_edge(clk) then
            -- sync delay line (only syncs + source avid are read from it, so
            -- yosys prunes its y/u/v chains)
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;
            gv_pipe(0) <= g_avid;
            for i in 1 to LATENCY - 1 loop gv_pipe(i) <= gv_pipe(i - 1); end loop;

            -- Video Fill delay line (one write, one registered read -> EBR)
            vdly_ram(to_integer(vdly_wa)) <= "00" & data_in.y & data_in.u & data_in.v;
            vdly_q  <= vdly_ram(to_integer(vdly_ra));
            vdly_q2 <= vdly_q;
            vdly_wa <= vdly_wa + 1;
            vdly_ra <= vdly_ra + 1;

            -- S0: latch pixel position + cell origin + per-cell Luma Mod sample/hold.
            s0_x  <= pixel_x;
            s0_y  <= pixel_y;
            s0_x0 <= cellx_org;
            s0_y0 <= celly_org;
            v_col := to_integer(cellx_idx);
            if v_col > 63 then v_col := 63; end if;
            -- this column's bits for the current row and the row above; on the
            -- first pixel of a column the previous pixel's bits are the left
            -- column's, so the left neighbour costs no second 64:1 read
            if s_cx_new = '1' then
                s0_hcl <= s0_hc;
                s0_hul <= s0_hu;
            end if;
            s0_hc <= cell_hide(v_col);
            s0_hu <= cell_hide_up(v_col);
            -- sample one dark bit per cell, at the cell centre on the cell-row's
            -- top line (active video only: the centre flag holds its last value
            -- through hblank and must not resample the blanking level there);
            -- the old bit moves to the row-above array
            if g_avid = '1' and (s_cy_new = '1' or s_first = '1') and s_cx_ctr = '1' then
                cell_hide_up(v_col) <= cell_hide(v_col);
                if unsigned(data_in.y) <= s_luma_thr then
                    cell_hide(v_col) <= '1';
                else
                    cell_hide(v_col) <= '0';
                end if;
            end if;

            -- S1a: base origin + 2x2 candidate origins + hashes + edge validity.  The
            -- single-add hash is short enough to share this stage; the old separate
            -- S1a2 stage was folded in here to cut registers / ease routing.  There
            -- are only two distinct x origins (x0, x0-cell) and two y, so the four
            -- candidate hashes come from the 2x2 product.
            v_x0  := s0_x0;
            v_y0  := s0_y0;
            v_x0L := s0_x0 - s_csz;   -- left-neighbour origin
            v_y0U := s0_y0 - s_csz;   -- up-neighbour origin
            s1a_x0 <= v_x0;
            s1a_y0 <= v_y0;
            s1a_px <= s0_x;
            s1a_py <= s0_y;
            if v_x0 >= s_csz then s1a_left_ok <= '1'; else s1a_left_ok <= '0'; end if;
            if v_y0 >= s_csz then s1a_up_ok   <= '1'; else s1a_up_ok   <= '0'; end if;
            -- Luma Mod bits in candidate order (gx,gy) (gx-1,gy) (gx,gy-1) (gx-1,gy-1)
            s1a_h(0) <= s0_hc;
            s1a_h(1) <= s0_hcl;
            s1a_h(2) <= s0_hu;
            s1a_h(3) <= s0_hul;
            s1a_hk(0) <= init_seed(s_comp_seed, v_x0,  v_y0);
            s1a_hk(1) <= init_seed(s_comp_seed, v_x0L, v_y0);
            s1a_hk(2) <= init_seed(s_comp_seed, v_x0,  v_y0U);
            s1a_hk(3) <= init_seed(s_comp_seed, v_x0L, v_y0U);

            -- S1b: decode each candidate's grow-right / grow-down bits (~25% each,
            -- only in Not-Square mode), test which cover this pixel (pure hash-bit
            -- logic since cells are grid-aligned), and pick the LARGEST covering cell
            -- (size-first, hash priority breaks ties).
            for k in 0 to 3 loop
                v_bigx(k) := s_notsq and s1a_hk(k)(7) and s1a_hk(k)(3);
                v_bigy(k) := s_notsq and s1a_hk(k)(6) and s1a_hk(k)(2);
            end loop;
            v_cov(0) := '1';
            v_cov(1) := v_bigx(1) and s1a_left_ok;
            v_cov(2) := v_bigy(2) and s1a_up_ok;
            v_cov(3) := v_bigx(3) and v_bigy(3) and s1a_left_ok and s1a_up_ok;
            for k in 0 to 3 loop
                v_rnk := 0;
                if v_bigx(k) = '1' then v_rnk := v_rnk + 1; end if;
                if v_bigy(k) = '1' then v_rnk := v_rnk + 1; end if;
                if v_cov(k) = '1' then
                    vv(k) := 16 + 16 * v_rnk + to_integer(s1a_hk(k)(15 downto 12));
                else
                    vv(k) := 0;
                end if;
            end loop;
            -- depth-2 tree argmax (ties -> lower index)
            if vv(0) >= vv(1) then v_m01 := vv(0); v_i01 := 0;
            else                   v_m01 := vv(1); v_i01 := 1; end if;
            if vv(2) >= vv(3) then v_m23 := vv(2); v_i23 := 2;
            else                   v_m23 := vv(3); v_i23 := 3; end if;
            if v_m01 >= v_m23 then v_kw := v_i01; else v_kw := v_i23; end if;

            -- register only the WINNER's data; the node (a couple of adds) is
            -- built next cycle in S1c so this stage's argmax path stays short.
            s1b_hkw  <= s1a_hk(v_kw);
            hide_pipe(2) <= s1a_h(v_kw);      -- the owning cell's Luma Mod bit
            for i in 3 to 13 loop hide_pipe(i) <= hide_pipe(i - 1); end loop;
            s1b_bigx <= v_bigx(v_kw);
            s1b_bigy <= v_bigy(v_kw);
            if CK_X(v_kw) = 1 then s1b_ckx <= '1'; else s1b_ckx <= '0'; end if;
            if CK_Y(v_kw) = 1 then s1b_cky <= '1'; else s1b_cky <= '0'; end if;
            s1b_x0 <= s1a_x0;
            s1b_y0 <= s1a_y0;
            s1b_px <= s1a_px;
            s1b_py <= s1a_py;

            -- S1c: build the winning (merged) cell node — origin shifted up-left by
            -- the candidate offset, extent 1 or 2 cells.  A grown cell is one root
            -- (one seed) spanning 2 cells, so it subdivides as a unit and the old
            -- grid line between the fused cells is no longer forced.
            if s1b_ckx = '1' then v_wx0 := s1b_x0 - s_csz;
            else                  v_wx0 := s1b_x0; end if;
            if s1b_cky = '1' then v_wy0 := s1b_y0 - s_csz;
            else                  v_wy0 := s1b_y0; end if;
            if s1b_bigx = '1' then v_wx1 := v_wx0 + s_csz2;
            else                   v_wx1 := v_wx0 + s_csz; end if;
            if s1b_bigy = '1' then v_wy1 := v_wy0 + s_csz2;
            else                   v_wy1 := v_wy0 + s_csz; end if;
            s1_n.x0     <= v_wx0;
            s1_n.x1     <= v_wx1;
            s1_n.y0     <= v_wy0;
            s1_n.y1     <= v_wy1;
            s1_n.px     <= s1b_px;
            s1_n.py     <= s1b_py;
            s1_n.seed   <= s1b_hkw;
            s1_n.frozen <= '0';

            -- Level 0 : hash -> decide -> descend
            do_hash(h0_n, h0_or, h0_ds, h0_sg, h0_hf, 0, s1_n);
            dtmp := calc_split(h0_n, h0_or, h0_ds, h0_sg, h0_hf, s_jit_mode);
            d0_n <= h0_n; d0_or <= h0_or; d0_dt <= dtmp.doit; d0_nx <= dtmp.nx;
            n0 <= descend(d0_n, d0_or, d0_dt, d0_nx);

            -- Level 1
            do_hash(h1_n, h1_or, h1_ds, h1_sg, h1_hf, 1, n0);
            dtmp := calc_split(h1_n, h1_or, h1_ds, h1_sg, h1_hf, s_jit_mode);
            d1_n <= h1_n; d1_or <= h1_or; d1_dt <= dtmp.doit; d1_nx <= dtmp.nx;
            n1 <= descend(d1_n, d1_or, d1_dt, d1_nx);

            -- Level 2
            do_hash(h2_n, h2_or, h2_ds, h2_sg, h2_hf, 2, n1);
            dtmp := calc_split(h2_n, h2_or, h2_ds, h2_sg, h2_hf, s_jit_mode);
            d2_n <= h2_n; d2_or <= h2_or; d2_dt <= dtmp.doit; d2_nx <= dtmp.nx;
            n2 <= descend(d2_n, d2_or, d2_dt, d2_nx);

            -- S10a: leaf-local coords + per-axis edge mins
            n  := n2;
            lw := n.x1 - n.x0;
            lh := n.y1 - n.y0;
            lx := n.px - n.x0;
            ly := n.py - n.y0;
            bb := (lw - 1) - lx;
            dd := (lh - 1) - ly;
            if lx < bb then m1 := lx; else m1 := bb; end if;
            if ly < dd then m2 := ly; else m2 := dd; end if;
            s10a_m1   <= m1;
            s10a_m2   <= m2;
            s10a_seed <= n.seed;

            -- S10b: grid-line distance + weighted Mondrian colour pick.
            -- (per-axis half-widths: horizontal seams keep at least half a
            -- field step; at 1080p both are the K6 width, as before)
            if s10a_m1 < s_line_w or s10a_m2 < s_lwy then
                s10_seam <= '1';
            else
                s10_seam <= '0';
            end if;

            v_colbyte := s10a_seed(7 downto 0);
            v_sel     := s10a_seed(9 downto 8);
            if v_colbyte < s_color_amt then
                case v_sel is
                    when "00"   => v_pidx := PAL_RED;
                    when "01"   => v_pidx := PAL_YELLOW;
                    when "10"   => v_pidx := PAL_BLUE;
                    when others => v_pidx := PAL_RED;
                end case;
                v_iscol := '1';
                if s_blackblk = '1' and s10a_seed(13 downto 11) = "111" then
                    v_pidx  := PAL_BLACK;   -- rare solid-black block
                    v_iscol := '0';
                end if;
            else
                v_pidx  := PAL_WHITE;
                v_iscol := '0';
            end if;
            -- S11 Luma Mod: colour only where the owning cell's input luma is
            -- bright; a dark cell's blocks stay white canvas (lines still drawn)
            if s_lumamod = '1' and hide_pipe(13) = '1' then
                v_pidx  := PAL_WHITE;
                v_iscol := '0';
            end if;
            s10_pidx    <= to_unsigned(v_pidx, 3);
            s10_iscolor <= v_iscol;

            -- S11: palette fetch
            pidx := to_integer(s10_pidx);
            ty := MON_Y(pidx); tu := MON_U(pidx); tv := MON_V(pidx);
            s11_y <= ty; s11_u <= tu; s11_v <= tv;
            s11_iscolor <= s10_iscolor;
            s11_seam    <= s10_seam;

            -- S12: assemble colour (grid line > flat block)
            if s_seam_en = '1' and s11_seam = '1' then
                s12_y   <= LINE_Y;
                s12_u   <= C_MID;
                s12_v   <= C_MID;
                s12_vid <= '0';
            else
                s12_y   <= s11_y;
                s12_u   <= s11_u;
                s12_v   <= s11_v;
                s12_vid <= s11_iscolor;
            end if;

            -- S_out: blanking gate (neutral outside the generated avid) >
            -- video-fill override > flat block colour.  Video only where the
            -- SOURCE avid is live too: the generated line starts/ends a few
            -- clocks after the source's, and those columns keep the block colour
            -- rather than the receiver's blanking codes.
            if gv_pipe(LATENCY - 1) = '0' then
                s_io.y <= std_logic_vector(C_BLK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif s12_vid = '1' and s_vid_en = '1' and pipe(LATENCY - 1).avid = '1' then
                s_io.y <= vdly_q2(29 downto 20);
                s_io.u <= vdly_q2(19 downto 10);
                s_io.v <= vdly_q2(9 downto 0);
            else
                s_io.y <= std_logic_vector(s12_y);
                s_io.u <= std_logic_vector(s12_u);
                s_io.v <= std_logic_vector(s12_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= gv_pipe(LATENCY - 1);
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
    -- latency changes (18 -> 20).
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

end architecture mondrian;
