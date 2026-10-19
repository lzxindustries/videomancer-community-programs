-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- tessera.vhd  (v17 — BRAM sprite store, 1..64 triangles, depth, scale)
--
-- v1.0.5 (2026-10-18) bit-exact: the sin/cos depth-layer scaling no longer
-- has a self-add (x/2 + x/2 and its far-pair twin put one net on two pins
-- of one LC, the router1 rip-up livelock); even layers are now z with its
-- LSB cleared, odd layers a sign-split 9-bit add.
--
-- v1.0.4 (2026-10-18) fit for the stock SDK SD Standalone core, bit-exact
-- output: the y and x sizes share one R / third / half shift-add set (each
-- latched as it leaves the size multiplier), the recycle range reuses the
-- latched third, and the three per-edge outline thresholds come from one
-- unit fed by the multiplier's B operand pipeline during products 12/14/16
-- (they were three parallel units in the store state).
--
-- v1.0.3 (2026-10-17) resolution scaling: triangle sizes were measured in
-- FIELD lines and the same size was used for the x extent in pixels, so
-- interlaced modes drew them ~2x tall (480i 2.2:1, 1080i 2:1) and SD
-- non-square pixels were ignored.  The x extent now has its own size,
-- from a per-mode "screen height in pixel widths": measured_v on
-- progressive HD (1080p/720p unchanged, bit-identical), 2 x measured_v at
-- 1080i, 540 on every SD mode (720 px = a 4:3 width).  The second size runs
-- through the per-frame multiplier right behind the first.  The SD-scale
-- table (0.25x .. 1.1x) is now picked by the timing ID for every SD mode
-- (the old "400..600 lines" test also caught 1080i and missed 480i/576i,
-- whose now-correct triangle widths cost 576p-like line time).
--
-- v17: count moves to K5 and the P12 fader becomes Scale: every triangle at
-- 0.25x .. 1.5x of its count's natural size (0.75/sqrt(count) of the screen
-- height, capped at 75%), an even exponential sweep; SD progressive tops out
-- at 1.1x.  Spin moves to K2; back faces (T11) take a toned-down version of
-- the outline colour, and the BG knob uses a toned-down palette.
--
-- v16: T10 Depth = Layers: slot mod 4 picks one of 4 depth layers (0 near):
-- size and fall speed x1, 3/4, 1/2, 3/8 (near = 1.25x the base size, capped
-- at 75%), each layer on its own fall accumulator, drawn far-to-near.  The
-- y-spacing step is (phi+2)/4, so every layer's slots are golden-spaced on
-- their own.  Flutter (always on): each triangle swings side to side once per
-- tumble, amplitude R (layer-scaled), turning round face-on.  Fill overrun
-- guard: if the next line starts mid-pass, the remaining walks are skipped.
--
-- v15: up to 64 triangles.  Only ~4-5 triangles ever share a scanline (the
-- golden y-spacing guarantees it), so the per-line fill budget allows far
-- more and far bigger triangles than the old count*height = 1.5H law
-- assumed: height = 0.75/sqrt(count) * H now (worst line ~53% of the HD
-- line, ~82% at 576p).  Per-triangle random values and the size tables live
-- in one EBR ROM (64-entry constant arrays would be big LUT muxes).
--
-- v14 finalisation fixes: generated avid (parity-locked start, so the
-- encoders' Cb/Cr pairing never flips on a wandering source avid) with the
-- line-buffer beam counted from it; outputs blanked to 64/512/512 outside
-- it; once-per-field animation pulse (analog vsync serrates); palette black
-- at Y 64 and white at Y 940; no_rw_check on the three RAMs; pipelined size
-- multiply.
--
-- Up to C_MAX_TRI equilateral-triangle OUTLINES rain down the screen, each with
-- a RANDOM in-plane orientation (unique look), random x + vertical stagger, a
-- shared slow in-plane spin, and tumbling end-over-end at uniform speed.
--
-- Architecture (the "sprite engine"):
--  * GEOMETRY (vblank): per triangle, rotate the base equilateral verts by a
--    random angle phi (sin/cos LUT) then foreshorten y by a TRIANGLE-WAVE factor
--    f_tau (linear -> uniform apparent tumble speed; cos would dwell/rush).
--    Produces 3 GENERAL edges (a=dy, b=-dx), the edge value at (xmin,0), the
--    bounding box and a threshold.  ALL multiplies go through ONE shared
--    multiplier, sequenced.  Records are written into a sprite BRAM.
--  * SPRITE STORE: a block RAM of per-triangle records (one packed vector per
--    slot).  Geometry writes all slots in vblank; the fill engine reads + writes
--    them back sequentially (no runtime-indexed register arrays -> those blew up
--    LUT usage; BRAM frees the fabric and lets the slot count grow to 64).
--  * FILL (per displayed line): for each live triangle, read its record, and if
--    the line is in its y-band, walk its x bounding span with one shared edge
--    tester, marking outline pixels into the fill half of a ping-pong line
--    buffer; step its edge row for the next line; write the record back.
--  * SCANOUT: read the display half of the line buffer on the generated avid
--    (1-line latency; both buffers are wiped in vsync, so the first line is
--    clean), clear behind, colour mux.  Drawing is bounded to each span so a
--    flat triangle draws a short segment (no full-screen line).
--
-- K5 sets the count (1..64, quadratic curve so the few-big-triangle end gets
-- real knob travel); each count has a natural size 0.75/sqrt(count) * H (1 ->
-- 75%, 64 -> 9.4%), which keeps the per-line fill cost roughly flat; the P12
-- fader scales that size 0.25x .. 1.5x (the most the line budget allows).
--
-- Controls: K1 Outline Colour, K2 Spin, K3 BG Colour (colour knobs sweep
-- white -> rainbow -> black; BG and back faces use a toned-down copy),
-- K4 Fall Speed, K5 Count, K6 Tumble Speed, T10 Depth (Flat / Layers),
-- T11 Video (front = video, back = toned-down outline colour),
-- Fader P12 = Scale (0.25x .. 1.5x).
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

architecture tessera of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (7 + 1 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLACK_Y    : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- line-buffer pixel codes
    constant LB_BG    : std_logic_vector(1 downto 0) := "00";
    constant LB_LINE  : std_logic_vector(1 downto 0) := "01";  -- outline
    constant LB_SOLID : std_logic_vector(1 downto 0) := "10";  -- solid fill (back side)
    constant LB_VIDEO : std_logic_vector(1 downto 0) := "11";  -- video fill (front side)

    constant C_MAX_TRI       : integer := 64;
    -- Address the sprite store with a full BRAM depth (256) even though only
    -- C_MAX_TRI rows are used: a 20-deep memory falls below yosys's block-RAM
    -- inference heuristic and lands in LUTs/FFs (LC blow-up); a 256-deep one
    -- maps cleanly to block RAM, and the spare rows cost nothing.
    constant C_AW            : integer := 8;
    constant C_MEM_DEPTH     : integer := 2 ** C_AW;
    constant C_OUTLINE_SHIFT : integer := 1;
    constant C_LBW           : integer := 2048;
    constant C_LAW           : integer := 11;

    constant C_GW : integer := 14;
    subtype  t_g  is signed(C_GW - 1 downto 0);
    constant C_EW : integer := 24;           -- record edge value (full-frame)
    subtype  t_e  is signed(C_EW - 1 downto 0);
    constant W_AB : integer := 12;           -- edge deltas dy / -dx
    subtype  t_ab is signed(W_AB - 1 downto 0);
    constant W_TH : integer := 13;           -- per-edge outline threshold
    subtype  t_th is signed(W_TH - 1 downto 0);
    constant C_FW : integer := 22;           -- fill working edge value (in-band)
    subtype  t_fc is signed(C_FW - 1 downto 0);
    constant W_XY : integer := 12;
    subtype  t_xy is signed(W_XY - 1 downto 0);    -- screen bbox coord

    --------------------------------------------------------------------------
    -- Constants ROM (one EBR pair, 256 x 32, read by the geometry engine):
    --   rows 0..63   triangle i: X(31:20) screen-x fraction, PHI(19:10) in-plane
    --                angle offset, T(9:0) tumble phase -- 32 bits of lowbias32(i+1)
    --   rows 64..127 Count row p (K5[9:4]): count(18:12), recip(11:0)
    --   rows 128..191 Scale row p (P12[9:4]): scale k(11:0), Q10
    --                (0.25 * 6^(p/63): 0.25x .. 1.5x, even steps of ~2.9%)
    --   rows 192..255 the same for SD (all modes): 0.25x .. 1.1x (less line
    --                time per screen height), selected by one address bit
    -- count = 1 + round(p^2/63): a quadratic knob curve, so 1..4 big triangles
    -- get ~20% of the travel.  recip = round(3072/sqrt(count)) = height / H in
    -- Q12 (0.75/sqrt(count)).  Only ~4-5 triangles share any scanline (golden
    -- y-spacing), so the fill stays inside its line budget at every count.
    --------------------------------------------------------------------------
    type t_int64 is array (0 to 63) of integer;
    constant C_P12_COUNT : t_int64 := (
        1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8,
        9, 9, 10, 11, 12, 13, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25,
        26, 28, 29, 30, 32, 33, 35, 36, 38, 39, 41, 42, 44, 46, 47, 49, 51,
        53, 54, 56, 58, 60, 62, 64);
    constant C_SCALE_K : t_int64 := (
        256, 263, 271, 279, 287, 295, 304, 312, 321, 331, 340,
        350, 360, 371, 381, 392, 404, 415, 427, 439, 452, 465,
        479, 492, 507, 521, 536, 552, 568, 584, 601, 618, 636,
        654, 673, 693, 713, 733, 754, 776, 799, 822, 845, 870,
        895, 921, 947, 974, 1003, 1031, 1061, 1092, 1123, 1156, 1189,
        1223, 1259, 1295, 1332, 1371, 1410, 1451, 1493, 1536);
    constant C_SCALE_K_SD : t_int64 := (
        256, 262, 268, 275, 281, 288, 295, 302, 309, 316, 324,
        332, 339, 348, 356, 364, 373, 382, 391, 400, 410, 419,
        429, 440, 450, 461, 472, 483, 495, 506, 518, 531, 543,
        556, 570, 583, 597, 611, 626, 641, 656, 671, 687, 704,
        721, 738, 755, 773, 792, 810, 830, 849, 870, 890, 912,
        933, 955, 978, 1001, 1025, 1050, 1075, 1100, 1126);
    constant C_P12_RECIP : t_int64 := (
        3072, 3072, 3072, 3072, 3072, 3072, 2172, 2172, 2172, 2172, 1774,
        1774, 1774, 1536, 1536, 1374, 1374, 1254, 1254, 1161, 1161, 1086,
        1024, 1024, 971, 926, 887, 852, 852, 821, 793, 768, 745, 724, 705,
        687, 655, 641, 627, 614, 602, 581, 570, 561, 543, 535, 519, 512,
        498, 492, 480, 474, 463, 453, 448, 439, 430, 422, 418, 411, 403,
        397, 390, 384);

    -- lowbias32 (C. Wellons): full-avalanche 32-bit integer hash.
    function mix32(x : unsigned(31 downto 0)) return unsigned is
        variable h : unsigned(31 downto 0) := x;
    begin
        h := h xor shift_right(h, 16);
        h := resize(h * unsigned'(x"7FEB352D"), 32);
        h := h xor shift_right(h, 15);
        h := resize(h * unsigned'(x"846CA68B"), 32);
        h := h xor shift_right(h, 16);
        return h;
    end function;

    type t_rom is array (0 to 255) of std_logic_vector(31 downto 0);
    function gen_rom return t_rom is
        variable r : t_rom := (others => (others => '0'));
    begin
        for i in 0 to C_MAX_TRI - 1 loop
            r(i) := std_logic_vector(mix32(to_unsigned(i + 1, 32)));
        end loop;
        for p in 0 to 63 loop
            r(64 + p)(18 downto 12) := std_logic_vector(to_unsigned(C_P12_COUNT(p), 7));
            r(64 + p)(11 downto 0)  := std_logic_vector(to_unsigned(C_P12_RECIP(p), 12));
            r(128 + p)(11 downto 0) := std_logic_vector(to_unsigned(C_SCALE_K(p), 12));
            r(192 + p)(11 downto 0) := std_logic_vector(to_unsigned(C_SCALE_K_SD(p), 12));
        end loop;
        return r;
    end function;
    constant C_ROM : t_rom := gen_rom;

    -- Colour palette for the Outline/Fill/BG knobs: white -> rainbow -> black,
    -- selected by the knob's top 3 bits (8 steps, no arithmetic). The 6 rainbow
    -- entries are the phosphor program's hardware-confirmed BT.601 values (U/V
    -- swapped for Videomancer rev_b: the U register holds Cr, V holds Cb).
    -- White and black sit at the legal 940 / 64 levels, not the rails.
    -- Entries 8..15 are the BG knob's toned-down copy (same 8 steps): luma 3/8
    -- of the way above black, chroma at half strength -- a full-brightness
    -- palette is far too loud behind the triangles.  The 16-entry table still
    -- costs one LUT4 per output bit (index = bg flag & knob top 3 bits).
    type t_pal is array (0 to 15) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(940, 10),   -- White
        to_unsigned(328, 10),   -- Red
        to_unsigned(584, 10),   -- Orange
        to_unsigned(840, 10),   -- Yellow
        to_unsigned(580, 10),   -- Green
        to_unsigned(164, 10),   -- Blue
        to_unsigned(349, 10),   -- Violet
        to_unsigned(64, 10),    -- Black
        to_unsigned(392, 10),   -- BG grey
        to_unsigned(163, 10),   -- BG red
        to_unsigned(259, 10),   -- BG orange
        to_unsigned(355, 10),   -- BG yellow
        to_unsigned(258, 10),   -- BG green
        to_unsigned(102, 10),   -- BG blue
        to_unsigned(171, 10),   -- BG violet
        to_unsigned(64, 10));   -- BG black
    constant C_PAL_U : t_pal := (
        to_unsigned(512, 10), to_unsigned(960, 10), to_unsigned(772, 10),
        to_unsigned(584, 10), to_unsigned(136, 10), to_unsigned(440, 10),
        to_unsigned(756, 10), to_unsigned(512, 10),
        to_unsigned(512, 10), to_unsigned(736, 10), to_unsigned(642, 10),
        to_unsigned(548, 10), to_unsigned(324, 10), to_unsigned(476, 10),
        to_unsigned(634, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal := (
        to_unsigned(512, 10), to_unsigned(360, 10), to_unsigned(212, 10),
        to_unsigned(64, 10),  to_unsigned(216, 10), to_unsigned(960, 10),
        to_unsigned(852, 10), to_unsigned(512, 10),
        to_unsigned(512, 10), to_unsigned(436, 10), to_unsigned(362, 10),
        to_unsigned(288, 10), to_unsigned(364, 10), to_unsigned(736, 10),
        to_unsigned(682, 10), to_unsigned(512, 10));

    --------------------------------------------------------------------------
    -- Sprite record (one per triangle) + packed BRAM storage.
    --------------------------------------------------------------------------
    type t_sprite is record
        a0, a1, a2 : t_ab;         -- dy_e (x step)
        b0, b1, b2 : t_ab;         -- -dx_e (y step)
        e0, e1, e2 : t_e;          -- edge value at (xmin, current fill line)
        xmin, xmax : t_xy;
        ymin, ymax : t_xy;
        thr0, thr1, thr2 : t_th;   -- per-edge outline threshold (length-scaled)
        valid      : std_logic;
        front      : std_logic;    -- '1' = front face toward viewer (video side)
    end record;
    constant C_SPRITE_ZERO : t_sprite := (
        a0=>(others=>'0'), a1=>(others=>'0'), a2=>(others=>'0'),
        b0=>(others=>'0'), b1=>(others=>'0'), b2=>(others=>'0'),
        e0=>(others=>'0'), e1=>(others=>'0'), e2=>(others=>'0'),
        xmin=>(others=>'0'), xmax=>(others=>'0'),
        ymin=>(others=>'0'), ymax=>(others=>'0'),
        thr0=>(others=>'0'), thr1=>(others=>'0'), thr2=>(others=>'0'),
        valid=>'0', front=>'0');

    constant C_REC_W : integer := 6*W_AB + 3*C_EW + 4*W_XY + 3*W_TH + 2;  -- 233

    function pack(r : t_sprite) return std_logic_vector is
    begin
        return std_logic_vector(r.a0) & std_logic_vector(r.a1) & std_logic_vector(r.a2)
             & std_logic_vector(r.b0) & std_logic_vector(r.b1) & std_logic_vector(r.b2)
             & std_logic_vector(r.e0) & std_logic_vector(r.e1) & std_logic_vector(r.e2)
             & std_logic_vector(r.xmin) & std_logic_vector(r.xmax)
             & std_logic_vector(r.ymin) & std_logic_vector(r.ymax)
             & std_logic_vector(r.thr0) & std_logic_vector(r.thr1) & std_logic_vector(r.thr2)
             & r.valid & r.front;
    end function;

    function unpack(v : std_logic_vector) return t_sprite is
        variable r : t_sprite;
        variable p : integer;
    begin
        p := C_REC_W;
        r.a0 := signed(v(p-1 downto p-W_AB)); p := p - W_AB;
        r.a1 := signed(v(p-1 downto p-W_AB)); p := p - W_AB;
        r.a2 := signed(v(p-1 downto p-W_AB)); p := p - W_AB;
        r.b0 := signed(v(p-1 downto p-W_AB)); p := p - W_AB;
        r.b1 := signed(v(p-1 downto p-W_AB)); p := p - W_AB;
        r.b2 := signed(v(p-1 downto p-W_AB)); p := p - W_AB;
        r.e0 := signed(v(p-1 downto p-C_EW)); p := p - C_EW;
        r.e1 := signed(v(p-1 downto p-C_EW)); p := p - C_EW;
        r.e2 := signed(v(p-1 downto p-C_EW)); p := p - C_EW;
        r.xmin := signed(v(p-1 downto p-W_XY)); p := p - W_XY;
        r.xmax := signed(v(p-1 downto p-W_XY)); p := p - W_XY;
        r.ymin := signed(v(p-1 downto p-W_XY)); p := p - W_XY;
        r.ymax := signed(v(p-1 downto p-W_XY)); p := p - W_XY;
        r.thr0 := signed(v(p-1 downto p-W_TH)); p := p - W_TH;
        r.thr1 := signed(v(p-1 downto p-W_TH)); p := p - W_TH;
        r.thr2 := signed(v(p-1 downto p-W_TH)); p := p - W_TH;
        r.valid := v(p-1); p := p - 1;
        r.front := v(p-1);
        return r;
    end function;

    -- The record store is split in two: the fill engine only ever changes
    -- the three edge values (e0..e2, 72 bits) when it writes a record back,
    -- so the other 161 bits (written by geometry alone, in vblank) live in
    -- their own RAM with no write mux, and the fill's write-back word is 72
    -- bits instead of 233.  Both padded to whole 16-bit EBR words.
    -- Packed layout (pack/unpack): a0..b2 = 232..161, e0..e2 = 160..89,
    -- xmin..front = 88..0.
    constant C_EHI : integer := C_REC_W - 1 - 6*W_AB;     -- 160
    constant C_ELO : integer := C_EHI - 3*C_EW + 1;       -- 89
    constant C_RS_W : integer := 176;                      -- 161 used
    constant C_RD_W : integer := 80;                       -- 72 used
    type t_recmem_s is array (0 to C_MEM_DEPTH - 1) of std_logic_vector(C_RS_W - 1 downto 0);
    type t_recmem_d is array (0 to C_MEM_DEPTH - 1) of std_logic_vector(C_RD_W - 1 downto 0);
    signal s_recmem_s : t_recmem_s;
    signal s_recmem_d : t_recmem_d;
    attribute ram_style : string;
    attribute ram_style of s_recmem_s : signal is "block";
    attribute ram_style of s_recmem_d : signal is "block";
    -- No read ever needs the address being written in the same cycle (the
    -- fill writes slot n back while reading n+1; geometry pushes in vblank
    -- with the read idle), so skip yosys's read-before-write emulation.
    -- Needs the init on the read registers.
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_recmem_s : signal is true;
    attribute no_rw_check of s_recmem_d : signal is true;
    signal s_rd_s       : std_logic_vector(C_RS_W - 1 downto 0) := (others => '0');
    signal s_rd_d       : std_logic_vector(C_RD_W - 1 downto 0) := (others => '0');
    signal s_rd_rec     : std_logic_vector(C_REC_W - 1 downto 0);
    signal s_rec_raddr  : unsigned(C_AW - 1 downto 0) := (others => '0');
    signal s_fill_we    : std_logic := '0';
    signal s_fill_waddr : unsigned(C_AW - 1 downto 0) := (others => '0');
    signal s_fill_wdata : std_logic_vector(C_EHI downto C_ELO) := (others => '0');
    -- edge RAM: one muxed write port (geometry and fill never write in the
    -- same cycle so a priority mux is safe)
    signal s_mem_we     : std_logic;
    signal s_mem_waddr  : unsigned(C_AW - 1 downto 0);
    signal s_mem_wdata  : std_logic_vector(C_EHI downto C_ELO);
    signal s_stage_pk   : std_logic_vector(C_REC_W - 1 downto 0);

    signal s_stage : t_sprite := C_SPRITE_ZERO;
    signal s_cur   : t_sprite := C_SPRITE_ZERO;
    signal s_push  : std_logic := '0';
    -- write address for a push, latched WITH s_push so the slot index is the one
    -- being stored (s_gtri increments on the same edge, so it must not be used).
    signal s_push_addr : unsigned(C_AW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Parameters / timing
    --------------------------------------------------------------------------
    signal s_outline_hue : unsigned(9 downto 0);
    signal s_bg_hue      : unsigned(9 downto 0);
    signal s_fall_pot    : unsigned(9 downto 0);
    signal s_spin_pot    : unsigned(9 downto 0);   -- K2 = Spin rate
    signal s_tumble_pot  : unsigned(9 downto 0);
    signal s_count_pot   : unsigned(9 downto 0);   -- K5 = Count
    signal s_scale_pot   : unsigned(9 downto 0);   -- P12 = Scale
    signal s_video_en    : std_logic;

    signal s_timing  : t_video_timing_port;
    signal s_v_count : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';
    signal s_saw_active  : std_logic := '0';   -- once-per-field guard

    -- generated avid (parity-locked start, field-latched width)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_gv   : std_logic_vector(0 to 2) := (others => '0');  -- g_avid delays
    signal s_bcol : unsigned(11 downto 0) := (others => '0');     -- beam column

    signal s_tumble_acc  : unsigned(9 downto 0) := (others => '0');
    signal s_spin_acc    : unsigned(9 downto 0) := (others => '0');  -- in-plane spin
    -- Fall accumulators, one per depth layer (Q4: 4 fraction bits, so the
    -- slow far layers still creep at the lowest Fall setting).  Updated as a
    -- ring after each pulse: two cycles per layer through one adder.
    subtype  t_fa is signed(17 downto 0);
    type t_fa_arr is array (0 to 3) of t_fa;
    signal s_fa       : t_fa_arr := (others => (others => '0'));
    signal s_fa_sum   : t_fa := (others => '0');
    signal s_fa_new0  : t_fa := (others => '0');   -- layer 0's new value (Flat mode)
    signal s_range16  : t_fa := (others => '0');   -- recycle range, Q4
    signal s_fr_cnt   : integer range 0 to 8 := 0;  -- ring steps left
    signal s_fsel     : t_g := (others => '0');     -- this triangle's fall offset
    signal s_pdel     : std_logic_vector(0 to 23) := (others => '0');  -- pulse delays
    -- T10 Depth: Layers = 4 depth layers (slot mod 4; 0 = near): size and fall
    -- speed x1, 3/4, 1/2, 3/8 of the near layer, far layers drawn first.
    signal s_t10_r    : std_logic := '0';
    signal s_layers   : std_logic := '0';
    signal s_rb1      : unsigned(12 downto 0) := (others => '0');  -- recip * 1.25
    signal s_lsel     : std_logic_vector(1 downto 0) := "00";       -- size layer
    -- fill visiting order: far layer first (slot(1:0) = 3 .. 0)
    signal s_fslot0   : integer range 0 to 3 := 0;
    -- fill overrun guard: the next line started while this one was still
    -- filling -> skip the remaining walks (edges still step), start the next
    -- line's pass as soon as this one ends
    signal s_flate    : std_logic := '0';
    signal s_fpend    : std_logic := '0';
    signal s_count_m4 : integer range -4 to C_MAX_TRI := 0;
    signal s_rangey      : t_g := (others => '0');
    signal s_count       : integer range 1 to C_MAX_TRI := 6;
    signal s_R, s_half, s_third : t_g := (others => '0');
    signal s_size_h : t_g := (others => '0');   -- latched triangle height
    -- x-extent versions (pixels; equal to the above on progressive HD)
    signal s_Rx, s_halfx, s_thirdx : t_g := (others => '0');
    -- golden-ratio y-distribution: per-frame step + per-triangle accumulator
    signal s_golden_step : t_g := (others => '0');
    signal s_gold_acc  : t_g := (others => '0');
    signal s_gold_sum  : t_g := (others => '0');   -- pre-wrap (pipelines the wrap)


    -- count-driven size pipeline
    -- K5 Count and P12 Scale, each with a +-6 code deadband (ADC noise at a
    -- table-row boundary would otherwise flip the count, or every size, each
    -- frame).  The tests are pipelined off the vsync decision (pre-registered
    -- sums, 1-bit flags): done in one cycle at the pulse they were the
    -- critical path.
    signal s_cnt_acc  : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_scl_acc  : unsigned(9 downto 0) := to_unsigned(790, 10);
    signal s_cpot_r, s_cpot_p6, s_cacc_p6 : unsigned(10 downto 0) := (others => '0');
    signal s_spot_r, s_spot_p6, s_sacc_p6 : unsigned(10 downto 0) := (others => '0');
    signal s_cnt_move, s_scl_move : std_logic := '0';
    -- the ROM's Count and Scale rows, read alternately while geometry idles
    signal s_row_cnt  : std_logic_vector(18 downto 0) := (others => '0');
    signal s_row_k    : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_rt       : std_logic := '0';
    signal s_rsel_d1, s_rsel_d2 : std_logic_vector(1 downto 0) := "00";  -- idle & toggle
    -- size: height = measured_v * min(3072, base(count) * k) >> 22, the two
    -- products run in turn through the one per-frame multiplier
    signal s_sz_base  : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_sz_k     : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_sq       : integer range 0 to 15 := 0;
    signal s_opa, s_opb : unsigned(11 downto 0) := (others => '0');
    signal s_recip_eff  : unsigned(11 downto 0) := (others => '0');
    signal s_sdp      : std_logic := '0';   -- SD (any): less line time
    signal s_i1080    : std_logic := '0';   -- 1080i: x size = 2 x field lines
    -- x-size reference ("screen height in pixel widths"): measured_v on
    -- progressive HD, 2 x measured_v at 1080i, 540 at SD; the x size =
    -- this * recip_eff >> 12 (the y size uses measured_v)
    signal s_sz_hxr   : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- constants ROM read port: address = triangle index during the geometry
    -- pass, the Count / Scale rows while idle.  s_rom_q is the EBR output register,
    -- s_rom_r the fabric register every consumer reads.
    signal s_rom_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_rom_r : std_logic_vector(31 downto 0) := (others => '0');
    signal s_sz_mv    : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_szp_lo   : unsigned(17 downto 0) := (others => '0');
    signal s_szp_hi   : unsigned(17 downto 0) := (others => '0');
    signal s_szmul1   : unsigned(23 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Shared sin/cos LUT + multiplier
    --------------------------------------------------------------------------
    signal s_lut_angle : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_sin   : signed(9 downto 0);
    signal s_lut_cos   : signed(9 downto 0);
    signal s_sin_r, s_cos_r : signed(10 downto 0) := (others => '0');
    signal s_sin_s, s_cos_s : signed(10 downto 0) := (others => '0');  -- layer-scaled

    signal s_mul_a, s_mul_b     : signed(13 downto 0) := (others => '0');
    signal s_mul_a_r, s_mul_b_r : signed(13 downto 0) := (others => '0');
    signal s_pp_lo              : signed(21 downto 0) := (others => '0');
    signal s_pp_hi              : signed(20 downto 0) := (others => '0');
    signal s_mul_p              : signed(27 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Geometry engine
    --------------------------------------------------------------------------
    type t_gstate is (G_IDLE, G_HASH, G_PHI, G_PHIW, G_FTAU, G_FTAU2, G_P, G_PW,
                      G_VCOMB1, G_VCOMB2, G_VCOMB3, G_VCOMB4, G_STORE);
    signal s_gst   : t_gstate := G_IDLE;
    signal s_gtri  : integer range 0 to C_MAX_TRI - 1 := 0;
    signal s_pcnt  : integer range 0 to 20 := 0;
    signal s_pwait : integer range 0 to 7 := 0;
    signal s_geng_start : std_logic := '0';
    signal s_ftau  : signed(10 downto 0) := (others => '0');
    signal s_tau_acc : unsigned(9 downto 0) := (others => '0');

    signal s_cx   : t_g := (others => '0');
    signal s_cy   : t_g := (others => '0');
    signal s_cy1, s_cy2, s_cy3 : t_g := (others => '0');  -- pipelined cy + wrap
    signal s_rsin, s_rcos, s_hc, s_tsin, s_hs, s_tcos : t_g := (others => '0');
    signal s_sy0, s_sy1, s_sy2 : t_g := (others => '0');
    signal s_wx0, s_wx1, s_wx2 : t_g := (others => '0');
    signal s_wy0, s_wy1, s_wy2 : t_g := (others => '0');
    signal s_dx0, s_dx1, s_dx2 : t_g := (others => '0');
    signal s_dy0, s_dy1, s_dy2 : t_g := (others => '0');
    signal s_xmin, s_xmax, s_ymin, s_ymax : t_g := (others => '0');
    signal s_bxmn, s_bxmx, s_bymn, s_bymx : t_g := (others => '0');  -- pairwise bbox
    signal s_c0, s_c1, s_c2 : t_e := (others => '0');
    signal s_tmp : t_e := (others => '0');

    --------------------------------------------------------------------------
    -- Fill engine
    --------------------------------------------------------------------------
    type t_fstate is (F_IDLE, F_LAT, F_HEAD, F_WALK, F_DRAIN, F_DONE);
    signal s_fst  : t_fstate := F_IDLE;
    signal s_fn   : integer range 0 to C_MAX_TRI - 1 := 0;   -- slot being filled
    signal s_fx   : signed(C_GW - 1 downto 0) := (others => '0');
    signal s_fc0, s_fc1, s_fc2 : t_fc := (others => '0');
    signal s_fwe  : std_logic := '0';
    signal s_fcode : std_logic_vector(1 downto 0) := "00";
    signal s_fwaddr : unsigned(C_LAW - 1 downto 0) := (others => '0');
    -- fill test pipeline (1 stage: compare/near in cycle N, wedge+write N+1)
    signal s_pn  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_psg : std_logic_vector(2 downto 0) := (others => '0');
    signal s_pwa : unsigned(C_LAW - 1 downto 0) := (others => '0');
    signal s_pv  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Line buffers (ping-pong, 2-bit code, inferred BRAM)
    --------------------------------------------------------------------------
    type t_lb is array (0 to C_LBW - 1) of std_logic_vector(1 downto 0);
    signal s_lb0, s_lb1 : t_lb := (others => "00");
    -- collision-free: the clear-behind writes the column read one cycle
    -- earlier, and fill-bank reads are never used
    attribute no_rw_check of s_lb0 : signal is true;
    attribute no_rw_check of s_lb1 : signal is true;
    signal s_rd0, s_rd1 : std_logic_vector(1 downto 0) := "00";
    -- clear-behind: the column the beam read last cycle
    signal s_clr_we : std_logic := '0';
    signal s_clr_wa : unsigned(C_LAW - 1 downto 0) := (others => '0');
    -- vsync buffer wipe (clears stale prev-frame content -> no top-line wrap)
    signal s_clr_addr : unsigned(C_LAW - 1 downto 0) := (others => '0');
    signal s_clr_busy : std_logic := '1';

    signal s_disp_code : std_logic_vector(1 downto 0) := "00";
    signal s_par1 : std_logic := '0';
    signal s_o_y : unsigned(9 downto 0) := (others => '0');
    signal s_o_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_o_v : unsigned(9 downto 0) := C_CHROMA_MID;
    -- Latched colour per knob (off the pixel path): one knob/channel refreshed
    -- per cycle through a single shared 8:1 palette mux; idx = knob top 3 bits.
    type t_col3 is array (0 to 2) of unsigned(9 downto 0);   -- [outline, fill, bg]
    signal s_col_y, s_col_u, s_col_v : t_col3 := (others => (others => '0'));
    signal s_ck, s_cc : integer range 0 to 2 := 0;

    -- incoming-video delay to align with the scanout colour mux (front face fill)
    type t_vdel is array (0 to 3) of std_logic_vector(9 downto 0);
    signal s_vy, s_vu, s_vv : t_vdel := (others => (others => '0'));

    constant C_RENDER_DELAY : integer := 5;
    type t_sync_pipe is array (0 to C_RENDER_DELAY - 1) of std_logic_vector(3 downto 0);
    signal s_sync_pipe : t_sync_pipe := (others => (others => '0'));
    signal s_io_0 : t_video_stream_yuv444_30b;
    signal s_io_1 : t_video_stream_yuv444_30b;

begin

    s_outline_hue <= unsigned(registers_in(0));
    s_bg_hue      <= unsigned(registers_in(2));
    s_fall_pot    <= unsigned(registers_in(3));
    s_spin_pot    <= unsigned(registers_in(1));   -- K2 = Spin rate
    s_count_pot   <= unsigned(registers_in(4));   -- K5 = Count
    s_tumble_pot  <= unsigned(registers_in(5));
    s_scale_pot   <= unsigned(registers_in(7));   -- Fader P12 = Scale
    s_video_en    <= registers_in(6)(4);   -- P11 toggle

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    pixel_counter_inst : entity work.pixel_counter
        port map (clk => clk, timing => s_timing,
                  h_count => open, v_count => s_v_count);

    -- Once-per-field pulse: the first vsync edge after active video.  Analog
    -- vsync serrates (several edges per field), so the animation accumulators
    -- would otherwise step several times a field.
    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev  <= s_timing.vsync_n;
            s_vsync_pulse <= '0';
            if s_timing.avid = '1' then
                s_saw_active <= '1';
            elsif s_vsync_prev = '0' and s_timing.vsync_n = '1' and s_saw_active = '1' then
                s_saw_active  <= '0';
                s_vsync_pulse <= '1';
            end if;
        end if;
    end process;

    sincos_inst : entity work.sin_cos_full_lut_10x10
        port map (angle_in => s_lut_angle, sin_out => s_lut_sin, cos_out => s_lut_cos);

    p_lut_reg : process(clk)
        variable v_zs, v_zc : signed(10 downto 0);
        variable v_ls, v_lc : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_sin_r <= resize(s_lut_sin, 11);
            s_cos_r <= resize(s_lut_cos, 11);
            -- depth-layer size: scaling sin/cos scales every vertex offset
            -- (and the flutter) of this triangle: x1, 3/4, 1/2, 3/8 =
            -- x/2 + x/2, x/2 + x/4, x/4 + x/4, x/4 + x/8.  Written so no adder
            -- sees the same net on both operands (a self-add puts one net on
            -- two pins of one LC, which livelocks nextpnr's router1):
            --   z = x (near pair) or x/2 (far pair);
            --   even layers: z/2 + z/2 = z with its LSB cleared, no adder;
            --   odd layers:  z/2 + z/4, z in the 10-bit range of the LUT, so
            --   only bits 8..1 + 9..2 need adding (9-bit sum incl. carry) and
            --   the two top bits are z's sign.
            if s_lsel(1) = '0' then v_zs := s_sin_r;                v_zc := s_cos_r;
            else                    v_zs := shift_right(s_sin_r, 1); v_zc := shift_right(s_cos_r, 1); end if;
            v_ls := ('0' & unsigned(v_zs(8 downto 1))) + ('0' & unsigned(v_zs(9 downto 2)));
            v_lc := ('0' & unsigned(v_zc(8 downto 1))) + ('0' & unsigned(v_zc(9 downto 2)));
            if s_lsel(0) = '0' then
                s_sin_s <= v_zs(10 downto 1) & '0';
                s_cos_s <= v_zc(10 downto 1) & '0';
            else
                s_sin_s <= v_zs(9) & v_zs(9) & signed(v_ls);
                s_cos_s <= v_zc(9) & v_zc(9) & signed(v_lc);
            end if;
        end if;
    end process;

    p_mul_inst : process(clk)
        variable v_b_slv : std_logic_vector(13 downto 0);
        variable v_b_lo  : signed(7 downto 0);
        variable v_b_hi  : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            s_mul_a_r <= s_mul_a;
            s_mul_b_r <= s_mul_b;
            v_b_slv := std_logic_vector(s_mul_b_r);
            v_b_lo  := signed('0' & v_b_slv(6 downto 0));
            v_b_hi  := signed(v_b_slv(13 downto 7));
            s_pp_lo <= s_mul_a_r * v_b_lo;
            s_pp_hi <= s_mul_a_r * v_b_hi;
            s_mul_p <= resize(s_pp_lo, 28) + shift_left(resize(s_pp_hi, 28), 7);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Animation: count, size reciprocal, tumble/spin + shared fall accumulators.
    --------------------------------------------------------------------------
    p_anim : process(clk)
        variable v_count        : integer range 0 to 127;
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                s_tumble_acc <= s_tumble_acc + (shift_right(s_tumble_pot, 6) + 1);
                -- in-plane spin rate from K2 (0 = none .. ~7 units/frame)
                s_spin_acc   <= s_spin_acc + resize(shift_right(s_spin_pot, 7), 10);
                if s_cnt_move = '1' then s_cnt_acc <= s_cpot_r(9 downto 0); end if;
                if s_scl_move = '1' then s_scl_acc <= s_spot_r(9 downto 0); end if;
                -- K5 row: count (1..64) and its base size 0.75/sqrt(count)
                v_count := to_integer(unsigned(s_row_cnt(18 downto 12)));
                if v_count < 1 then v_count := 1; end if;
                if v_count > C_MAX_TRI then v_count := C_MAX_TRI; end if;
                s_count    <= v_count;
                -- Layers: the near layer is 1.25x the base size; the far
                -- layers scale down from it per triangle
                s_layers   <= s_t10_r;
                if s_t10_r = '1' then
                    s_sz_base <= s_rb1(11 downto 0);
                else
                    s_sz_base <= unsigned(s_row_cnt(11 downto 0));
                end if;
                -- P12 scale k (Q10, 0.25x .. 1.5x; every SD mode reads its
                -- own 0.25x .. 1.1x table -- less line time per screen height)
                s_sz_k <= s_row_k;
                s_sz_mv    <= s_measured_v;
                if s_sdp = '1' then
                    s_sz_hxr <= to_unsigned(540, 12);
                elsif s_i1080 = '1' then
                    s_sz_hxr <= s_measured_v(10 downto 0) & '0';
                else
                    s_sz_hxr <= s_measured_v;
                end if;
                g_wm1      <= s_measured_h - 1;   -- generated-avid width
            end if;
        end if;
    end process;

    -- Free-running per-frame helpers (sampled at the pulse / by the fill):
    -- boosted base size, T10, SD-progressive flag, fill visiting order.
    p_frame_aux : process(clk)
    begin
        if rising_edge(clk) then
            s_t10_r <= registers_in(6)(3);
            s_rb1 <= resize(unsigned(s_row_cnt(11 downto 0)), 13)
                   + resize(shift_right(unsigned(s_row_cnt(11 downto 0)), 2), 13);
            -- timing ID class: SD (480i/480p/576i/576p) / 1080i
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_sdp <= '1'; s_i1080 <= '0';
                when 1 | 2 | 10     => s_sdp <= '0'; s_i1080 <= '1';
                when others         => s_sdp <= '0'; s_i1080 <= '0';
            end case;
            if s_count >= 4 then s_fslot0 <= 3; else s_fslot0 <= s_count - 1; end if;
            s_count_m4 <= s_count - 4;
            s_pdel <= s_vsync_pulse & s_pdel(0 to 22);
        end if;
    end process;

    -- Fall ring: after each pulse (once the size pipeline has settled), each
    -- layer's accumulator steps by Fall x {1, 3/4, 1/2, 3/8} (Q4) and wraps at
    -- the recycle range: add on one cycle, wrap + rotate on the next.  Flat
    -- copies layer 0's new value into all four, so they move as one.
    p_fall : process(clk)
        variable v_f   : t_fa;
        variable v_inc : t_fa;
        variable v_w   : t_fa;
        variable v_d   : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            s_range16 <= shift_left(resize(s_rangey, 18), 4);
            if s_pdel(11) = '1' then
                s_fr_cnt <= 8;
            elsif s_fr_cnt /= 0 then
                s_fr_cnt <= s_fr_cnt - 1;
                v_d := 3 - (s_fr_cnt - 1) / 2;      -- 8,7 -> 0 .. 2,1 -> 3
                if (s_fr_cnt mod 2) = 0 then
                    v_f := resize(signed('0' & std_logic_vector(shift_right(s_fall_pot, 6))), 18) + 1;
                    if s_layers = '0' or v_d = 0 then
                        v_inc := shift_left(v_f, 4);                       -- x1
                    elsif v_d = 1 then
                        v_inc := shift_left(v_f, 3) + shift_left(v_f, 2);  -- x3/4
                    elsif v_d = 2 then
                        v_inc := shift_left(v_f, 3);                       -- x1/2
                    else
                        v_inc := shift_left(v_f, 2) + shift_left(v_f, 1);  -- x3/8
                    end if;
                    s_fa_sum <= s_fa(0) + v_inc;
                else
                    v_w := s_fa_sum;
                    if v_w >= s_range16 then v_w := v_w - s_range16; end if;
                    if v_d = 0 then
                        s_fa_new0 <= v_w;
                    elsif s_layers = '0' then
                        v_w := s_fa_new0;
                    end if;
                    s_fa(0) <= s_fa(1);
                    s_fa(1) <= s_fa(2);
                    s_fa(2) <= s_fa(3);
                    s_fa(3) <= v_w;
                end if;
            end if;
        end if;
    end process;

    -- Deadband flags: '1' when a control is more than 6 codes from its
    -- accepted value.  Free-running; p_anim only samples them at the pulse.
    p_deadband : process(clk)
    begin
        if rising_edge(clk) then
            s_cpot_r  <= resize(s_count_pot, 11);
            s_cpot_p6 <= resize(s_count_pot, 11) + 6;
            s_cacc_p6 <= resize(s_cnt_acc, 11) + 6;
            if s_cpot_r > s_cacc_p6 or s_cpot_p6 < resize(s_cnt_acc, 11) then
                s_cnt_move <= '1';
            else
                s_cnt_move <= '0';
            end if;
            s_spot_r  <= resize(s_scale_pot, 11);
            s_spot_p6 <= resize(s_scale_pot, 11) + 6;
            s_sacc_p6 <= resize(s_scl_acc, 11) + 6;
            if s_spot_r > s_sacc_p6 or s_spot_p6 < resize(s_scl_acc, 11) then
                s_scl_move <= '1';
            else
                s_scl_move <= '0';
            end if;
        end if;
    end process;

    -- Size, once per frame: recip_eff = min(3072, base * k >> 10), then
    -- height = measured_v * recip_eff >> 12, both through one multiplier (the
    -- B operand split hi/lo: a single-cycle 12x12 was once the critical path).
    -- The 3072 cap keeps the largest triangle at 75% of the screen height.
    p_sizecalc : process(clk)
    begin
        if rising_edge(clk) then
            s_szp_lo <= s_opa * s_opb(5 downto 0);
            s_szp_hi <= s_opa * s_opb(11 downto 6);
            s_szmul1 <= resize(s_szp_lo, 24) + shift_left(resize(s_szp_hi, 24), 6);
            if s_vsync_pulse = '1' then
                s_sq <= 1;
            elsif s_sq /= 0 then
                if s_sq = 9 then s_sq <= 0; else s_sq <= s_sq + 1; end if;
            end if;
            case s_sq is
                when 1 => s_opa <= s_sz_base; s_opb <= s_sz_k;
                when 4 =>
                    if shift_right(s_szmul1, 10) > 3072 then
                        s_recip_eff <= to_unsigned(3072, 12);
                    else
                        s_recip_eff <= resize(shift_right(s_szmul1, 10), 12);
                    end if;
                when 5 => s_opa <= s_sz_mv; s_opb <= s_recip_eff;
                -- x size right behind it (pipelined: same recip operand)
                when 6 => s_opa <= s_sz_hxr;
                when others => null;
            end case;
        end if;
    end process;

    -- Derive R / half / third from each size as it leaves the multiplier:
    -- one shift-add set serves both, latched into the y set at step 8 (the
    -- height) and the x set at step 9 (the x extent).  third and half share
    -- their (1/16 + 1/64) term; the recycle range reuses the latched third.
    p_size : process(clk)
        variable v_s, v_t, v_R, v_third, v_half : t_g;
    begin
        if rising_edge(clk) then
            v_s     := signed(resize(shift_right(s_szmul1, 12), C_GW));
            v_t     := shift_right(v_s, 4) + shift_right(v_s, 6);
            v_R     := shift_right(v_s, 1) + shift_right(v_s, 3)
                     + shift_right(v_s, 5) + shift_right(v_s, 7);        -- 2/3 H
            v_third := shift_right(v_s, 2) + v_t;                         -- 1/3 H
            v_half  := shift_right(v_s, 1) + v_t;                         -- 0.577 H
            if s_sq = 8 then
                s_size_h <= v_s;
                s_R <= v_R; s_third <= v_third; s_half <= v_half;
            end if;
            if s_sq = 9 then
                s_Rx <= v_R; s_thirdx <= v_third; s_halfx <= v_half;
            end if;
            -- recycle range = measured_v + 2R (R = 2/3 H) = mv + H + H/3: a
            -- triangle re-enters the top exactly as it exits the bottom, so
            -- there is no blank gap.
            s_rangey <= signed(resize(s_measured_v, C_GW)) + s_size_h + s_third;
        end if;
    end process;

    -- y-spacing step = (phi + 2)/4 = 0.6545 of the recycle range (1/2 + 1/8 +
    -- 1/32 - 1/512), via shifts.  Four steps = 2.618 = 0.618 mod 1, so the
    -- slots of each depth layer (slot mod 4) form their own golden sequence,
    -- evenly spread per layer, from the one accumulator; the whole set is
    -- still low-discrepancy for Flat (no lattice at 64, unlike 39/64).
    p_golden : process(clk)
    begin
        if rising_edge(clk) then
            s_golden_step <= shift_right(s_rangey, 1) + shift_right(s_rangey, 3)
                           + shift_right(s_rangey, 5) - shift_right(s_rangey, 9);
        end if;
    end process;

    -- Constants ROM (see C_ROM).  One read, so one EBR pair.  While the
    -- geometry engine idles it alternates between the Count row and the Scale
    -- row, and each answer is latched (two clocks later) for p_anim.
    p_rom : process(clk)
        variable v_addr : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_rt <= not s_rt;
            if s_gst = G_IDLE then
                if s_rt = '0' then v_addr := "01" & s_cnt_acc(9 downto 4);
                else               v_addr := '1' & s_sdp & s_scl_acc(9 downto 4); end if;
                s_rsel_d1 <= '1' & s_rt;
            else
                v_addr := to_unsigned(s_gtri, 8);
                s_rsel_d1 <= "00";
            end if;
            s_rom_q <= C_ROM(to_integer(v_addr));
            s_rom_r <= s_rom_q;
            s_rsel_d2 <= s_rsel_d1;
            if s_rsel_d2 = "10" then s_row_cnt <= s_rom_r(18 downto 0); end if;
            if s_rsel_d2 = "11" then s_row_k   <= unsigned(s_rom_r(11 downto 0)); end if;
        end if;
    end process;

    -- Colour knob -> palette, latched. Round-robin over 3 slots x 3 channels
    -- (one per cycle) through one shared table. idx = knob top 3 bits; the
    -- back-face fill is K1's colour from the toned-down half, BG is K3's.
    p_color : process(clk)
        variable v_kv  : unsigned(9 downto 0);
        variable v_idx : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            case s_ck is
                when 0      => v_kv := s_outline_hue;
                when 1      => v_kv := s_outline_hue;
                when others => v_kv := s_bg_hue;
            end case;
            v_idx := to_integer(v_kv(9 downto 7));
            if s_ck /= 0 then v_idx := v_idx + 8; end if;   -- fill/BG: dimmed copy
            case s_cc is
                when 0      => s_col_y(s_ck) <= C_PAL_Y(v_idx);
                when 1      => s_col_u(s_ck) <= C_PAL_U(v_idx);
                when others => s_col_v(s_ck) <= C_PAL_V(v_idx);
            end case;
            if s_cc = 2 then
                s_cc <= 0;
                if s_ck = 2 then s_ck <= 0; else s_ck <= s_ck + 1; end if;
            else
                s_cc <= s_cc + 1;
            end if;
        end if;
    end process;

    -- geometry starts 24 clocks after the pulse: the size sequence (~10),
    -- golden step and fall ring (12..20) have all settled by then
    s_geng_start <= s_pdel(23);

    --------------------------------------------------------------------------
    -- Geometry engine: per triangle -> sprite record (pushed via s_stage/s_push).
    --------------------------------------------------------------------------
    p_geom : process(clk)
        variable v_a, v_b : signed(13 downto 0);
        variable v_py1, v_py2 : t_g;
        variable v_frac   : signed(10 downto 0);
        variable v_mh     : t_g;
        variable v_adx, v_ady, v_len : unsigned(13 downto 0);
        variable v_d      : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            s_push <= '0';
            case s_gst is
                when G_IDLE =>
                    if s_geng_start = '1' then
                        s_gtri <= 0;
                        s_gold_sum <= (others => '0');   -- triangle 0 at phase 0
                        s_pwait <= 0;
                        s_gst  <= G_HASH;
                    end if;

                when G_HASH =>
                    -- the ROM word for s_gtri reaches s_rom_r two cycles after
                    -- s_gtri changes; it then holds for the whole triangle.
                    -- Layer d = slot mod 4 (0 = near): its fall offset and its
                    -- size scale (Layers only).
                    v_d := s_gtri mod 4;
                    s_fsel <= s_fa(v_d)(17 downto 4);
                    if s_layers = '1' then
                        s_lsel <= std_logic_vector(to_unsigned(v_d, 2));
                    else
                        s_lsel <= "00";
                    end if;
                    if s_pwait >= 1 then
                        s_gst <= G_PHI;
                    else
                        s_pwait <= s_pwait + 1;
                    end if;

                when G_PHI =>
                    -- in-plane orientation = random per-triangle offset + a
                    -- shared spin accumulator (constant slow rotation over time)
                    s_lut_angle <= std_logic_vector(
                        unsigned(s_rom_r(19 downto 10)) + s_spin_acc);
                    -- wrap the golden accumulator here (compare+subtract), the
                    -- pre-wrap add was done in the previous triangle's G_STORE
                    if s_gold_sum >= s_rangey then s_gold_acc <= s_gold_sum - s_rangey;
                    else                           s_gold_acc <= s_gold_sum; end if;
                    s_pwait <= 0;
                    s_gst   <= G_PHIW;

                when G_PHIW =>
                    -- cy = (layer fall + golden stagger) wrapped - margin(R),
                    -- pipelined one op per cycle and overlapped with LUT settle.
                    -- both terms are < range (two wraps cover a range that
                    -- just shrank when the count changed)
                    s_cy1 <= s_fsel + s_gold_acc;
                    if s_cy1 >= s_rangey then s_cy2 <= s_cy1 - s_rangey;
                    else                      s_cy2 <= s_cy1; end if;
                    if s_cy2 >= s_rangey then s_cy3 <= s_cy2 - s_rangey;
                    else                      s_cy3 <= s_cy2; end if;
                    s_cy <= s_cy3 - s_R;
                    if s_pwait >= 7 then
                        s_gst <= G_FTAU;
                    else
                        s_pwait <= s_pwait + 1;
                    end if;

                when G_FTAU =>
                    -- pipelined: (1) tumble phase = shared accumulator + hash
                    s_tau_acc <= s_tumble_acc + unsigned(s_rom_r(9 downto 0));
                    s_gst <= G_FTAU2;

                when G_FTAU2 =>
                    -- (2) triangle-wave foreshorten f_tau (Q9, +-512), uniform speed
                    v_frac := signed(resize(s_tau_acc(7 downto 0), 11));
                    case s_tau_acc(9 downto 8) is
                        when "00"   => s_ftau <= to_signed(512, 11) - shift_left(v_frac, 1);
                        when "01"   => s_ftau <= -shift_left(v_frac, 1);
                        when "10"   => s_ftau <= to_signed(-512, 11) + shift_left(v_frac, 1);
                        when others => s_ftau <= shift_left(v_frac, 1);
                    end case;
                    s_pcnt  <= 0;
                    s_pwait <= 0;
                    s_gst   <= G_P;

                when G_P =>
                    case s_pcnt is
                        when 0 => v_a := signed(resize(unsigned(s_rom_r(31 downto 20)), 14));
                                  v_b := resize(signed('0' & std_logic_vector(s_measured_h)), 14);
                        -- x offsets (1, 3, 4, 10) use the x-extent sizes
                        when 1 => v_a := resize(s_Rx, 14);    v_b := resize(s_sin_s, 14);
                        when 2 => v_a := resize(s_R, 14);     v_b := resize(s_cos_s, 14);
                        when 3 => v_a := resize(s_halfx, 14); v_b := resize(s_cos_s, 14);
                        when 4 => v_a := resize(s_thirdx, 14); v_b := resize(s_sin_s, 14);
                        when 5 => v_a := resize(s_half, 14);  v_b := resize(s_sin_s, 14);
                        when 6 => v_a := resize(s_third, 14); v_b := resize(s_cos_s, 14);
                        when 7 => v_a := resize(-s_rcos, 14); v_b := resize(s_ftau, 14);
                        when 8 => v_py1 := s_tcos - s_hs; v_a := resize(v_py1, 14); v_b := resize(s_ftau, 14);
                        when 9 => v_py2 := s_tcos + s_hs; v_a := resize(v_py2, 14); v_b := resize(s_ftau, 14);
                        -- flutter: R * cos(tumble phase), layer-scaled (the LUT
                        -- was switched to the tumble angle after product 6)
                        when 10 => v_a := resize(s_Rx, 14);   v_b := resize(s_cos_s, 14);
                        when 11 => v_a := resize(s_wx0, 14); v_b := resize(s_dy0, 14);
                        when 12 => v_a := resize(s_wy0, 14); v_b := resize(s_dx0, 14);
                        when 13 => v_a := resize(s_wx1, 14); v_b := resize(s_dy1, 14);
                        when 14 => v_a := resize(s_wy1, 14); v_b := resize(s_dx1, 14);
                        when 15 => v_a := resize(s_wx2, 14); v_b := resize(s_dy2, 14);
                        when 16 => v_a := resize(s_wy2, 14); v_b := resize(s_dx2, 14);
                        when 17 => v_a := resize(s_dy0, 14); v_b := resize(s_xmin, 14);
                        when 18 => v_a := resize(s_dy1, 14); v_b := resize(s_xmin, 14);
                        when others => v_a := resize(s_dy2, 14); v_b := resize(s_xmin, 14);
                    end case;
                    s_mul_a <= v_a;
                    s_mul_b <= v_b;
                    s_pwait <= 0;
                    s_gst   <= G_PW;

                when G_PW =>
                    -- product 6 has latched the last phi operand: point the LUT
                    -- at the tumble angle for the flutter product (10)
                    if s_pcnt = 6 then
                        s_lut_angle <= std_logic_vector(s_tau_acc);
                    end if;
                    -- per-edge threshold = (Manhattan length) * outline width,
                    -- so the outline stays a constant pixel width as edges
                    -- foreshorten differently while tumbling.  One unit for all
                    -- three edges: on the first wait cycle of products 12 / 14
                    -- / 16 the multiplier's B operand holds dx_e and its
                    -- registered copy dy_e (products 11..16 feed dy0, dx0, ..).
                    if s_pwait = 0 and (s_pcnt = 12 or s_pcnt = 14 or s_pcnt = 16) then
                        v_adx := unsigned(abs(s_mul_b));
                        v_ady := unsigned(abs(s_mul_b_r));
                        if v_adx > v_ady then v_len := v_adx + shift_right(v_ady, 1);
                        else                  v_len := v_ady + shift_right(v_adx, 1); end if;
                        -- (len << C_OUTLINE_SHIFT: the low bits are never
                        -- written and keep their '0' init, so yosys sees them
                        -- as constants and the static record RAM stays 10 EBR)
                        case s_pcnt is
                            when 12 => s_stage.thr0(W_TH - 1 downto C_OUTLINE_SHIFT)
                                           <= signed(resize(v_len, W_TH - C_OUTLINE_SHIFT));
                            when 14 => s_stage.thr1(W_TH - 1 downto C_OUTLINE_SHIFT)
                                           <= signed(resize(v_len, W_TH - C_OUTLINE_SHIFT));
                            when others => s_stage.thr2(W_TH - 1 downto C_OUTLINE_SHIFT)
                                           <= signed(resize(v_len, W_TH - C_OUTLINE_SHIFT));
                        end case;
                    end if;
                    if s_pwait >= 3 then
                        case s_pcnt is
                            when 0 => s_cx   <= resize(shift_right(s_mul_p, 12), C_GW);
                            when 1 => s_rsin <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 2 => s_rcos <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 3 => s_hc   <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 4 => s_tsin <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 5 => s_hs   <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 6 => s_tcos <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 7 => s_sy0  <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 8 => s_sy1  <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 9 => s_sy2  <= resize(shift_right(s_mul_p, 9), C_GW);
                            -- flutter: swing left/right once per tumble, turning
                            -- round face-on and gliding fastest edge-on
                            when 10 => s_cx  <= s_cx - resize(shift_right(s_mul_p, 9), C_GW);
                            when 11 => s_tmp <= resize(s_mul_p, C_EW);
                            when 12 => s_c0  <= -s_tmp + resize(s_mul_p, C_EW);
                            when 13 => s_tmp <= resize(s_mul_p, C_EW);
                            when 14 => s_c1  <= -s_tmp + resize(s_mul_p, C_EW);
                            when 15 => s_tmp <= resize(s_mul_p, C_EW);
                            when 16 => s_c2  <= -s_tmp + resize(s_mul_p, C_EW);
                            when 17 => s_stage.e0 <= resize(s_mul_p, C_EW) + s_c0;
                            when 18 => s_stage.e1 <= resize(s_mul_p, C_EW) + s_c1;
                            when others => s_stage.e2 <= resize(s_mul_p, C_EW) + s_c2;
                        end case;

                        if s_pcnt = 10 then
                            s_gst <= G_VCOMB1;
                        elsif s_pcnt = 19 then
                            s_gst <= G_STORE;
                        else
                            s_pcnt <= s_pcnt + 1;
                            s_gst  <= G_P;
                        end if;
                    else
                        s_pwait <= s_pwait + 1;
                    end if;

                when G_VCOMB1 =>
                    -- stage 1: register world verts (x not foreshortened)
                    s_wx0 <= s_cx + s_rsin;          s_wy0 <= s_cy + s_sy0;
                    s_wx1 <= s_cx - s_hc - s_tsin;   s_wy1 <= s_cy + s_sy1;
                    s_wx2 <= s_cx + s_hc - s_tsin;   s_wy2 <= s_cy + s_sy2;
                    s_gst <= G_VCOMB2;

                when G_VCOMB2 =>
                    -- stage 2: edges + PAIRWISE bbox (vert0 vs vert1)
                    s_dx0 <= s_wx1 - s_wx0; s_dy0 <= s_wy1 - s_wy0;
                    s_dx1 <= s_wx2 - s_wx1; s_dy1 <= s_wy2 - s_wy1;
                    s_dx2 <= s_wx0 - s_wx2; s_dy2 <= s_wy0 - s_wy2;
                    if s_wx0 <= s_wx1 then s_bxmn <= s_wx0; s_bxmx <= s_wx1;
                    else                   s_bxmn <= s_wx1; s_bxmx <= s_wx0; end if;
                    if s_wy0 <= s_wy1 then s_bymn <= s_wy0; s_bymx <= s_wy1;
                    else                   s_bymn <= s_wy1; s_bymx <= s_wy0; end if;
                    s_gst <= G_VCOMB3;

                when G_VCOMB3 =>
                    -- stage 3: fold in vert2
                    if s_wx2 < s_bxmn then s_xmin <= s_wx2; else s_xmin <= s_bxmn; end if;
                    if s_wx2 > s_bxmx then s_xmax <= s_wx2; else s_xmax <= s_bxmx; end if;
                    if s_wy2 < s_bymn then s_ymin <= s_wy2; else s_ymin <= s_bymn; end if;
                    if s_wy2 > s_bymx then s_ymax <= s_wy2; else s_ymax <= s_bymx; end if;
                    s_gst <= G_VCOMB4;

                when G_VCOMB4 =>
                    -- stage 4: clamp both x bounds to [0, screen] (flutter can
                    -- carry a triangle wholly off either side; the clamp keeps
                    -- the walk short and the 12-bit field legal)
                    v_mh := signed(resize(s_measured_h, C_GW));
                    if s_xmin < to_signed(0, C_GW) then s_xmin <= (others => '0');
                    elsif s_xmin > v_mh then          s_xmin <= v_mh; end if;
                    if s_xmax > v_mh then                s_xmax <= v_mh;
                    elsif s_xmax < to_signed(0, C_GW) then s_xmax <= (others => '0'); end if;
                    s_pcnt <= 11;
                    s_pwait <= 0;
                    s_gst  <= G_P;

                when G_STORE =>
                    s_stage.a0 <= resize(s_dy0, W_AB); s_stage.a1 <= resize(s_dy1, W_AB); s_stage.a2 <= resize(s_dy2, W_AB);
                    s_stage.b0 <= resize(-s_dx0, W_AB); s_stage.b1 <= resize(-s_dx1, W_AB); s_stage.b2 <= resize(-s_dx2, W_AB);
                    s_stage.xmin <= resize(s_xmin, W_XY); s_stage.xmax <= resize(s_xmax, W_XY);
                    s_stage.ymin <= resize(s_ymin, W_XY); s_stage.ymax <= resize(s_ymax, W_XY);
                    -- (thresholds thr0..thr2 were latched during products 12..16)
                    if s_gtri < s_count then s_stage.valid <= '1'; else s_stage.valid <= '0'; end if;
                    -- front face toward viewer while the tumble factor is >= 0
                    if s_ftau >= 0 then s_stage.front <= '1'; else s_stage.front <= '0'; end if;
                    s_push      <= '1';
                    s_push_addr <= to_unsigned(s_gtri, C_AW);   -- slot being stored

                    if s_gtri = C_MAX_TRI - 1 then
                        s_gst <= G_IDLE;
                    else
                        s_gtri <= s_gtri + 1;
                        -- advance the golden phase (add only; G_PHI does the wrap)
                        s_gold_sum <= s_gold_acc + s_golden_step;
                        s_pwait <= 0;
                        s_gst  <= G_HASH;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sprite store (BRAM): geometry writes all slots in vblank; the fill engine
    -- reads + writes records back during active.  Simple dual-port (independent
    -- read/write addresses); the two writers never collide in time.
    --------------------------------------------------------------------------
    s_stage_pk  <= pack(s_stage);
    s_mem_we    <= s_push or s_fill_we;
    s_mem_waddr <= s_push_addr when s_push = '1' else s_fill_waddr;
    s_mem_wdata <= s_stage_pk(C_EHI downto C_ELO) when s_push = '1' else s_fill_wdata;

    -- static fields: geometry only
    p_recmem_s : process(clk)
    begin
        if rising_edge(clk) then
            if s_push = '1' then
                s_recmem_s(to_integer(s_push_addr)) <= (C_RS_W - 1 downto C_REC_W - 3*C_EW => '0')
                    & s_stage_pk(C_REC_W - 1 downto C_EHI + 1) & s_stage_pk(C_ELO - 1 downto 0);
            end if;
            s_rd_s <= s_recmem_s(to_integer(s_rec_raddr));
        end if;
    end process;

    -- edge values: geometry, or the fill engine's write-back
    p_recmem_d : process(clk)
    begin
        if rising_edge(clk) then
            if s_mem_we = '1' then
                s_recmem_d(to_integer(s_mem_waddr)) <= (C_RD_W - 1 downto 3*C_EW => '0') & s_mem_wdata;
            end if;
            s_rd_d <= s_recmem_d(to_integer(s_rec_raddr));
        end if;
    end process;

    s_rd_rec <= s_rd_s(C_REC_W - 3*C_EW - 1 downto C_ELO) & s_rd_d(3*C_EW - 1 downto 0)
              & s_rd_s(C_ELO - 1 downto 0);

    --------------------------------------------------------------------------
    -- Fill engine: each displayed line, for every live slot, read the record,
    -- walk its span if in y-band, step its edge row, write it back.  Slots are
    -- visited far layer first (slot(1:0) = 3, 2, 1, 0; ascending within a
    -- layer) so near triangles paint over far ones.  Overrun guard: if the
    -- next line starts mid-pass, the walk in progress ends and the remaining
    -- walks are skipped (edges still step); the next pass starts right after.
    --------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_hd  : t_sprite;
        variable v_wr  : t_sprite;
        variable v_pk  : std_logic_vector(C_REC_W - 1 downto 0);
        variable v_vc  : t_g;
        variable v_n0, v_n1, v_n2 : std_logic;
        variable v_out, v_in : std_logic;
        variable v_t0, v_t1, v_t2 : t_fc;
    begin
        if rising_edge(clk) then
            s_fwe     <= '0';
            s_fill_we <= '0';
            if s_timing.avid_start = '1' and s_fst /= F_IDLE then
                s_flate <= '1';
                s_fpend <= '1';
            end if;

            case s_fst is
                when F_IDLE =>
                    if s_timing.avid_start = '1' or s_fpend = '1' then
                        s_fn  <= s_fslot0;
                        s_rec_raddr <= to_unsigned(s_fslot0, C_AW);
                        s_flate <= '0';
                        s_fpend <= '0';
                        s_fst <= F_LAT;
                    end if;

                when F_LAT =>
                    -- BRAM read of slot s_fn in flight; one cycle settle
                    s_fst <= F_HEAD;

                when F_HEAD =>
                    v_hd := unpack(s_rd_rec);
                    s_cur <= v_hd;
                    v_vc := signed(resize(s_v_count, C_GW));
                    s_fc0 <= resize(v_hd.e0, C_FW);
                    s_fc1 <= resize(v_hd.e1, C_FW);
                    s_fc2 <= resize(v_hd.e2, C_FW);
                    if v_hd.xmin < to_signed(0, W_XY) then
                        s_fx <= (others => '0');
                    else
                        s_fx <= resize(v_hd.xmin, C_GW);
                    end if;
                    s_pv <= '0';   -- test pipeline empty
                    if (v_hd.valid = '1') and (v_vc >= resize(v_hd.ymin, C_GW))
                       and (v_vc <= resize(v_hd.ymax, C_GW)) and s_flate = '0' then
                        s_fst <= F_WALK;
                    else
                        s_fst <= F_DONE;
                    end if;

                when F_WALK =>
                    -- (a) emit the write for the pixel tested LAST cycle:
                    --   outline -> LB_LINE; else inside (signs agree) gets
                    --   filled when P11 is on (video on the front face,
                    --   solid on the back); else nothing (background).
                    v_out := '0';
                    if s_pn(0) = '1' and s_psg(1) = s_psg(2) then v_out := '1'; end if;
                    if s_pn(1) = '1' and s_psg(0) = s_psg(2) then v_out := '1'; end if;
                    if s_pn(2) = '1' and s_psg(0) = s_psg(1) then v_out := '1'; end if;
                    if (s_psg(0) = s_psg(1)) and (s_psg(1) = s_psg(2)) then v_in := '1'; else v_in := '0'; end if;
                    s_fwaddr <= s_pwa;
                    if v_out = '1' then
                        s_fcode <= LB_LINE; s_fwe <= s_pv;
                    elsif v_in = '1' and s_video_en = '1' then
                        if s_cur.front = '1' then s_fcode <= LB_VIDEO;
                        else                      s_fcode <= LB_SOLID; end if;
                        s_fwe <= s_pv;
                    else
                        s_fcode <= LB_BG; s_fwe <= '0';
                    end if;
                    -- (b) test THIS pixel (two parallel signed compares per
                    --     edge, using each edge's own threshold)
                    v_t0 := resize(s_cur.thr0, C_FW);
                    v_t1 := resize(s_cur.thr1, C_FW);
                    v_t2 := resize(s_cur.thr2, C_FW);
                    if (s_fc0 < v_t0) and (s_fc0 > -v_t0) then v_n0 := '1'; else v_n0 := '0'; end if;
                    if (s_fc1 < v_t1) and (s_fc1 > -v_t1) then v_n1 := '1'; else v_n1 := '0'; end if;
                    if (s_fc2 < v_t2) and (s_fc2 > -v_t2) then v_n2 := '1'; else v_n2 := '0'; end if;
                    s_pn(0) <= v_n0; s_pn(1) <= v_n1; s_pn(2) <= v_n2;
                    s_psg(0) <= s_fc0(C_FW-1); s_psg(1) <= s_fc1(C_FW-1); s_psg(2) <= s_fc2(C_FW-1);
                    s_pwa <= unsigned(s_fx(C_LAW - 1 downto 0));
                    s_pv  <= '1';
                    -- (c) step
                    s_fc0 <= s_fc0 + resize(s_cur.a0, C_FW);
                    s_fc1 <= s_fc1 + resize(s_cur.a1, C_FW);
                    s_fc2 <= s_fc2 + resize(s_cur.a2, C_FW);
                    if s_fx >= resize(s_cur.xmax, C_GW) or s_flate = '1' then
                        s_fst <= F_DRAIN;
                    else
                        s_fx <= s_fx + 1;
                    end if;

                when F_DRAIN =>
                    -- write the final pixel still in the test pipeline
                    v_out := '0';
                    if s_pn(0) = '1' and s_psg(1) = s_psg(2) then v_out := '1'; end if;
                    if s_pn(1) = '1' and s_psg(0) = s_psg(2) then v_out := '1'; end if;
                    if s_pn(2) = '1' and s_psg(0) = s_psg(1) then v_out := '1'; end if;
                    if (s_psg(0) = s_psg(1)) and (s_psg(1) = s_psg(2)) then v_in := '1'; else v_in := '0'; end if;
                    s_fwaddr <= s_pwa;
                    if v_out = '1' then
                        s_fcode <= LB_LINE; s_fwe <= s_pv;
                    elsif v_in = '1' and s_video_en = '1' then
                        if s_cur.front = '1' then s_fcode <= LB_VIDEO;
                        else                      s_fcode <= LB_SOLID; end if;
                        s_fwe <= s_pv;
                    else
                        s_fcode <= LB_BG; s_fwe <= '0';
                    end if;
                    s_pv  <= '0';
                    s_fst <= F_DONE;

                when F_DONE =>
                    -- step the edge row for the next line, write the record back
                    v_wr := s_cur;
                    v_wr.e0 := s_cur.e0 + resize(s_cur.b0, C_EW);
                    v_wr.e1 := s_cur.e1 + resize(s_cur.b1, C_EW);
                    v_wr.e2 := s_cur.e2 + resize(s_cur.b2, C_EW);
                    s_fill_waddr <= to_unsigned(s_fn, C_AW);
                    v_pk := pack(v_wr);
                    s_fill_wdata <= v_pk(C_EHI downto C_ELO);
                    s_fill_we    <= '1';
                    -- next slot: +4 within the layer; else the next nearer
                    -- layer's first slot; done after layer 0
                    if s_fn < s_count_m4 then
                        s_fn <= s_fn + 4;
                        s_rec_raddr <= to_unsigned(s_fn + 4, C_AW);
                        s_fst <= F_LAT;
                    elsif (s_fn mod 4) = 0 then
                        s_fst <= F_IDLE;
                    else
                        s_fn <= (s_fn mod 4) - 1;
                        s_rec_raddr <= to_unsigned((s_fn mod 4) - 1, C_AW);
                        s_fst <= F_LAT;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Generated avid.  The encoders get no data-enable, so they pair Cb/Cr by
    -- counting from hsync, while the core's 4:2:2 packer restarts on Cb at
    -- every avid rise: a source avid that lands a clock off its usual hsync
    -- distance swaps U/V for that line (thin wrong-coloured lines).  So the
    -- program makes its own avid (cubist/aurora recipe): g_ph is the
    -- hsync->avid-rise parity of this line, g_par the voted usual parity; a
    -- line whose avid lands off-parity starts one clock later, so the start
    -- always sits at the same hsync parity.  It ends after the field-latched
    -- width, or at hsync at the latest.  The beam column counts it.
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

            if g_hsr = '1' then
                s_bcol <= (others => '0');
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
            end if;

            -- clear-behind: wipe the column the beam read this cycle, next cycle
            s_clr_we <= g_avid;
            s_clr_wa <= s_bcol(C_LAW - 1 downto 0);

            s_gv(0) <= g_avid;
            s_gv(1) <= s_gv(0);
            s_gv(2) <= s_gv(1);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Line buffers.  Fill = buffer[v_count(0)]; display = the other.  The beam
    -- reads at s_bcol (generated-avid column); v_count has already stepped
    -- when the generated avid starts, so the bank choice is settled.
    --------------------------------------------------------------------------
    -- Sweep both line buffers to 0 during vsync (fill + clear-behind are idle
    -- then), so no stale previous-frame content survives into the top lines.
    p_clear : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_clr_addr <= (others => '0');
                s_clr_busy <= '1';
            elsif s_clr_busy = '1' then
                if s_clr_addr = to_unsigned(C_LBW - 1, C_LAW) then
                    s_clr_busy <= '0';
                else
                    s_clr_addr <= s_clr_addr + 1;
                end if;
            end if;
        end if;
    end process;

    p_lb0 : process(clk)
        variable v_we : std_logic;
        variable v_wa : unsigned(C_LAW - 1 downto 0);
        variable v_wd : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            if s_clr_busy = '1' then
                v_we := '1'; v_wa := s_clr_addr; v_wd := LB_BG;
            elsif s_v_count(0) = '0' then
                v_we := s_fwe; v_wa := s_fwaddr; v_wd := s_fcode;
            else
                v_we := s_clr_we; v_wa := s_clr_wa; v_wd := LB_BG;
            end if;
            if v_we = '1' then s_lb0(to_integer(v_wa)) <= v_wd; end if;
            s_rd0 <= s_lb0(to_integer(s_bcol(C_LAW - 1 downto 0)));
        end if;
    end process;

    p_lb1 : process(clk)
        variable v_we : std_logic;
        variable v_wa : unsigned(C_LAW - 1 downto 0);
        variable v_wd : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            if s_clr_busy = '1' then
                v_we := '1'; v_wa := s_clr_addr; v_wd := LB_BG;
            elsif s_v_count(0) = '1' then
                v_we := s_fwe; v_wa := s_fwaddr; v_wd := s_fcode;
            else
                v_we := s_clr_we; v_wa := s_clr_wa; v_wd := LB_BG;
            end if;
            if v_we = '1' then s_lb1(to_integer(v_wa)) <= v_wd; end if;
            s_rd1 <= s_lb1(to_integer(s_bcol(C_LAW - 1 downto 0)));
        end if;
    end process;

    -- incoming video delayed to align with the scanout colour mux
    p_vdelay : process(clk)
    begin
        if rising_edge(clk) then
            s_vy(0) <= data_in.y; s_vu(0) <= data_in.u; s_vv(0) <= data_in.v;
            for i in 1 to 3 loop
                s_vy(i) <= s_vy(i - 1);
                s_vu(i) <= s_vu(i - 1);
                s_vv(i) <= s_vv(i - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Scanout: display buffer = NOT being filled = ~v_count(0); read latency 1.
    --------------------------------------------------------------------------
    p_scanout : process(clk)
    begin
        if rising_edge(clk) then
            s_par1 <= s_v_count(0);
            if s_par1 = '0' then s_disp_code <= s_rd1; else s_disp_code <= s_rd0; end if;

            case s_disp_code is
                when LB_LINE =>
                    s_o_y <= s_col_y(0); s_o_u <= s_col_u(0); s_o_v <= s_col_v(0);
                when LB_VIDEO =>
                    -- the generated line runs a few clocks past the source's:
                    -- show black there, not the receiver's blanking codes
                    if s_sync_pipe(3)(2) = '1' then
                        s_o_y <= unsigned(s_vy(3));
                        s_o_u <= unsigned(s_vu(3));
                        s_o_v <= unsigned(s_vv(3));
                    else
                        s_o_y <= C_BLACK_Y; s_o_u <= C_CHROMA_MID; s_o_v <= C_CHROMA_MID;
                    end if;
                when LB_SOLID =>
                    s_o_y <= s_col_y(1); s_o_u <= s_col_u(1); s_o_v <= s_col_v(1);
                when others =>
                    s_o_y <= s_col_y(2); s_o_u <= s_col_u(2); s_o_v <= s_col_v(2);
            end case;
        end if;
    end process;

    p_sync_pipe : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_pipe(0) <= data_in.field_n & data_in.avid &
                              data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_RENDER_DELAY - 1 loop
                s_sync_pipe(i) <= s_sync_pipe(i - 1);
            end loop;
        end if;
    end process;

    -- Output avid = generated avid; y/u/v blanked to neutral outside it.
    -- Latency: 7 clocks for syncs and video, all modes.
    p_io_align : process(clk)
    begin
        if rising_edge(clk) then
            if s_gv(2) = '1' then
                s_io_0.y <= std_logic_vector(s_o_y);
                s_io_0.u <= std_logic_vector(s_o_u);
                s_io_0.v <= std_logic_vector(s_o_v);
            else
                s_io_0.y <= std_logic_vector(C_BLACK_Y);
                s_io_0.u <= std_logic_vector(C_CHROMA_MID);
                s_io_0.v <= std_logic_vector(C_CHROMA_MID);
            end if;
            s_io_0.hsync_n <= s_sync_pipe(C_RENDER_DELAY - 1)(0);
            s_io_0.vsync_n <= s_sync_pipe(C_RENDER_DELAY - 1)(1);
            s_io_0.avid    <= s_gv(2);
            s_io_0.field_n <= s_sync_pipe(C_RENDER_DELAY - 1)(3);
            s_io_1 <= s_io_0;
        end if;
    end process;

    s_io_align_in.y       <= s_io_1.y;
    s_io_align_in.u       <= s_io_1.u;
    s_io_align_in.v       <= s_io_1.v;
    s_io_align_in.hsync_n <= s_io_1.hsync_n;
    s_io_align_in.vsync_n <= s_io_1.vsync_n;
    s_io_align_in.avid    <= s_io_1.avid;
    s_io_align_in.field_n <= s_io_1.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (7 -> 8).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture tessera;
