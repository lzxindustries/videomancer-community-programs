-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Author: czartroniks

-- cascade.vhd  (v2.1.2 -- 126/48 coupled discs, line-buffer mark render;
--   finalized 2026-10-10; developed + HW-verified in a diagnostic copy)
--
-- v2.1.5 (2026-10-17): RESOLUTION SCALING.  The rings keep their size relative
--   to the screen HEIGHT in every mode (virtual 1080-line canvas; SD is a
--   1440-wide 4:3 window of it).  The engine stays in REAL pixels on x; per
--   mode (timing id, latched at vsync):
--     * spacing scales by 1/x-step: log-spacing floor s_lfmin = -704 + 64*log2
--       (x-step): 1080 -704 (x1), 720p -667 (x0.67), SD -640 (x0.5), and the
--       4-px minimum clamps the knob part (s_k3max);
--     * y runs in x-pixel units: a Q8 dy accumulator steps 1/x-step of the
--       virtual y step per FIELD line (480i 2.22, 576i 1.875, 1080i 2.0,
--       480p 1.125, 576p 0.9375, 720p/1080p 1.0) -- interlace and the SD
--       pixel aspect are both in that step (replaces the field_n-toggle
--       interlace detect, which HD inputs never trip: field_n is tied high);
--       the measured height is accumulated in the same units;
--     * the BOTTOM field (field_n = '0') starts half a field step down (the
--       old code offset field_n = '1' -- the fields were swapped at SD);
--     * 720p caps the modelled ring count at 100 (1650-clk line: the span
--       pass 15*(n+2) + E_LINE 99 must end before the next active rise;
--       126 rings overran it at fine spacings, a pre-existing 720p bug);
--     * S11 Video wave depth scales too (Q1 depth, 4 shifted key terms).
--   1080p is bit-identical (every per-mode term is the old constant there).
--
-- HYPNOS CASCADE: concentric black/white discs that PUSH each other.  Move the
-- centre (K1/K2) and the innermost disc leads; each outer ring holds still until
-- the one inside closes its slack, then is shoved -- a wave of motion travelling
-- outward ring by ring.  Rings never overlap (a band of the other colour always
-- survives) and never deform.  Catch-Up (S10) glides them home concentric.
--
--   RENDER -- 1bpp dual-bank LINE BUFFER of edge MARKS.  The span engine
--     writes each disc's two scanline edges as single-bit marks; the pixel
--     path is ONE running-parity XOR over the marks.  The chain clamp keeps
--     the discs strictly nested, so edge-count parity == ring parity.
--
--   ENGINE -- runs a line EARLY: the pass for line N starts at line N-1's
--     active-video RISE (chain RAM, shared multiply and the sqrt unit are
--     all idle during active video).  E_LINE (the tracker seed) runs in
--     the hblank.
--
--   GRADIENT (K4/S9) -- the marks carry an enter/leave DIRECTION bit, so
--     the pixel path counts the true ring index; every ring carries its own
--     ramp (re-anchored at each true disc edge), K4 sets the fineness and
--     S9 picks Steps/Smooth.
--
--   VIDEO (S11/P12) -- the ring field REFRACTS the input: a one-sided
--     horizontal push of 0..53 px through a 64-tap EBR delay line, P12 =
--     depth.  Same latency (6) and sync taps as ring mode.
--
--   OUTER field -- incremental integer-radius tracker centred on the LAST
--     modelled disc's centre, seamed exactly onto it.
--
--   CHAIN -- coupled dead-zone chain in PIXEL units, C_NR entries, with an
--     OCTAGONAL coupling clamp: the slack is ~87% of the spacing in every
--     drag direction, so rings nearly touch before yielding.
--
-- v0.9 (2026-07-31), both from user reports:
--   * RING COUNT 24 -> 96 (HD) / 48 (SD).  Ring count is pure throughput,
--     not fit -- see the C_NR budget note below.  The nest now reaches the
--     screen corners for any spacing >= 12 px instead of >= 46, so the
--     cascade wave travels the whole picture rather than a centre disc.
--   * CATCH-UP now actually centres.  The settle pull was d + floor(-d/8),
--     which is a NO-OP for -7 <= d <= -1 -- and a drag right/down makes the
--     rings lag LEFT, i.e. d NEGATIVE, so the common case froze every ring
--     up to 7 px off its parent and the nest never closed.
--   * The radius clamp no longer piles rings on one radius (that dropped an
--     odd number of edge marks half the time and inverted the screen); the
--     modelled count adapts per frame to what fits under C_RCL.
--
-- v2.1 (2026-10-09, diag): S9 MOIRE -- the rings XOR a half-phase copy of
--   themselves around the outer centre (doubles the rings at rest; moire
--   flares while the cascade has them offset).  K4 setting 2 = 4 steps
--   (v2.0.3).  Built with ROUTER1 (router2 can't close this netlist).
--
-- v2.0 (2026-10-09, diag -- LEAN PASS; brief: keep the look, cascade,
--   catch-up and gradients; Zoom/Video droppable; S9 free):
--   * ONE SERIAL MULTIPLIER (shift-add, 14 steps, product at T+15) replaces
--     TWO LUT array multipliers (~1060 LUT): the vblank 14x14 was merged into
--     the engine's (v1.9), then the engine's 15x15 became serial (v2.0).  The
--     span engine is re-pipelined around it (phase 4: stage ring k-2's edges,
--     root ring k-1, multiply ring k; marks at phase 5; 15*(nreff+2) clocks),
--     E_LINE re-scheduled (89 clocks, the two roots overlap the products), the
--     vblank sequencer issues one product at a time and stalls on busy.
--     Bit-exact vs v1.9 in four 720p sims.  LINE BUDGET 15*(nreff+2)+89:
--     HD 2009/2200, SD 839/858.  E_LINE no longer fits the image-tester's
--     fake 64-clk hblank -- sim with a 280-clk hblank copy of the tool
--     (.cascade_diag_work/vit_hb).
--   * K5 WIDTH: band balance, exact circles (every even disc's radius moves
--     by delta = f*s; slack from the narrowest band; outer field per-parity
--     thresholds).  Eased, deadbanded at the centre, off in Video mode.
--   * LEFT-EDGE SEED: the per-ring phase at x=0 is seeded from the band's own
--     off-screen left edge ((r0 - r_L)*Q1, r_L = 2nd root per line), so the
--     edge band no longer "comes to a point" during catch-up.
--   * K4 = 4 settings (hard / 2 / 8 grays / smooth); S9 Smooth folded in, S9
--     free.  K3 x144 decode -> shift-adds.
--
-- v1.8 (2026-10-09, diag; brief: manual control only -- no auto-animation):
--   * v1.8.1: S7 Glide 2x quicker (C_GSH 6 -> 5: ~1/32 of the way per field,
--     a full-screen move settles in ~1.6 s instead of ~3.2 s).
--   * K1/K2 range +-0.625 W/H: the centre goes ~12% past each edge at most
--     (was +-2 W/H -- the rings got lost off-screen).
--   * P12 in RINGS mode = ZOOM: up to 3 octaves (x8) subtracted from K3's
--     log-spacing -> the rings expand from the centre (clamped at 2048 px).
--     (An exact band-WIDTH control was built first and did not fit at
--     ~101% LC; shelved in .cascade_diag_work/v18_width_shelved/.)
--   * P12 in VIDEO mode = wave depth, 8 levels, up to 53 px (was 4 zones,
--     18 px); delay line 32 -> 64 taps (same 2 EBR).
--
-- v1.7 (2026-10-08, Tier-1 finalization, no look change; HW-verified):
--   * ring luma in studio range 64..942 (was 0..1023: codes 0/1023 are the
--     BT.656/1120 sync-word codes);
--   * blanking gate covers BOTH modes (ring mode emitted the last pixel's
--     key through every hblank);
--   * the dead multi-ring smooth-fade phase (old S11 Smooth: ph_acc, Q
--     decode + multiply, line seed) removed; its sequencer/E_LINE slots
--     are kept as no-ops so the engine timing is unchanged.
--
-- No CORDIC, no cos/sin ROM, no per-pixel multiply anywhere; every edge is an
-- exact integer circle.  C_NR MUST stay even (mark parity == k parity).
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.core_config_pkg.all;
use work.video_stream_pkg.all;

architecture cascade of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (6 + 2 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 6;    -- RINGS: x/read 1 + parity 1 + g2 1 + merge 1 + key 1 + out 1
    -- VIDEO path: the refraction is a ONE-SIDED push (read 1 + disp back,
    -- disp = 0..18; v1.8: 0..53).  A constant displacement term is an invisible whole-
    -- picture shift, so 0..18 one-sided looks IDENTICAL to +-9 centred --
    -- and it lands the video pipeline at delay 6 = C_LATENCY exactly, so
    -- BOTH modes share the ring mode's PROVEN sync taps.  (v1.4's deeper
    -- 15-cycle video-only tap was the ONE structural difference from every
    -- hardware-validated build, and hd_hdmi showed green + noise lines.)
    constant C_MID     : unsigned(9 downto 0) := to_unsigned(512, 10);   -- neutral chroma

    constant C_DB    : integer := 14;                                    -- detent deadband
    constant C_GSH   : integer := 5;                                     -- glide one-pole shift (v1.8.1: 6 -> 5, 2x quicker)

    -- CASCADE chain: per-ring centre offsets.  The coupling clamp is an
    -- OCTAGON (per-axis +/-A AND 45-degree +/-C = A*sqrt2): a ring yields
    -- only once the one inside has closed ~90% of the spacing, i.e. the
    -- minimum edge gap at the push point is ~10% of the spacing in ANY drag
    -- direction (a plain box at this slack would overlap on diagonals).
    -- A_px = min(s - s/8, s - s/16 - s/32 - 3) (~0.875*s) keeps every
    -- adjacent edge pair > 1 px apart on any scanline (mark writes never
    -- collide) and nesting strict.  NO accumulated cap -- the wave travels
    -- the chain.  C_NR MUST stay even (mark parity == k parity).
    --
    -- RING COUNT BUDGET (v0.9: 96 rings at HD, 48 at SD -- v0.8 was 24/24):
    --   * LC is FLAT in C_NR (7080 at 24, 7091 at 96 -- measured): the chain
    --     scan, the span pass and the pixel path are all serialized or
    --     constant-cost, and the offsets live in EBR.  Ring count is a
    --     THROUGHPUT question, never a fit one.
    --   * The per-LINE pass is E_SPAN (15 slots/ring, starts at the active
    --     rise and spills into the hblank via l_pend) + E_LINE (46 slots),
    --     and must be back in E_IDLE before the NEXT active rise or that
    --     line's pass never launches:
    --         15*C_NR + 56  <=  clocks_per_line
    --     HD  prog_clk 74.25 MHz, 2200 clk/line  ->  142 rings
    --     SD  prog_clk 13.5  MHz,  858 clk/line  ->   53 rings
    --     (SD is 858, NOT 1716: prog_clk = i_vid_dec_clk, the 13.5 MHz
    --     decoder pixel clock -- the 27 MHz PLL feeds the ENCODER only, and
    --     the 27 MHz build constraint is pure margin.)
    --     Sim raster is W/decimation + 64, so decimation 1 (1984 clk) is
    --     needed to sim the HD count; decimation 4 (544) holds only 32.
    --   * Chain words stay 15-bit: |e[k]| is bounded by the CENTRE's own
    --     travel (~10.2k worst case at W = 4095), never by k*A, so the 640
    --     slack cap does not have to scale with C_NR.
    --   * Index words are 7-bit (t_kl is mod 128), so C_NR <= 126.
    function f_nr return integer is
    begin
        if C_ENABLE_HD then return 126; else return 48; end if;
    end function;
    constant C_NR    : integer := f_nr;                                  -- modelled ring slots
    -- Settle rate: the pull is ceil(|d| / 2^C_SET) per frame.  Catch-Up is a
    -- WAVE -- ring k can only close once the one inside it has -- so the time
    -- to reach concentric is ~(7 / 2^(C_SET-3)) frames PER RING, i.e. it
    -- scales with C_NR.  Measured (spacing 30, model): C_SET 3 = 168 frames
    -- at 24 rings but 672 (11 s) at 96; C_SET 2 = 294 (4.9 s); C_SET 1 = 105
    -- (1.8 s).  Dropped 3 -> 2 with the ring count so 96 rings still glide
    -- home in about the same few seconds v0.8 took at 24.
    constant C_SET   : integer := 2;                                     -- settle glide shift (toward inner)

    constant C_RCL   : integer := 4095;                                  -- px clamp: nest disc radii
    constant C_OCL   : integer := 4095;                                  -- outer-centre offset clamp (px)

    ----------------------------------------------------------------------
    -- exp2 mantissa ROM: round(1024 * 2^(f/64)), f = 0..63  -> [1024,2048)
    -- (used by the K3 frequency decode AND its reciprocal, the pixel spacing)
    ----------------------------------------------------------------------
    type t_mant is array(0 to 63) of unsigned(10 downto 0);
    constant C_MANT : t_mant := (
        to_unsigned(1024,11), to_unsigned(1035,11), to_unsigned(1046,11), to_unsigned(1058,11),
        to_unsigned(1069,11), to_unsigned(1081,11), to_unsigned(1093,11), to_unsigned(1105,11),
        to_unsigned(1117,11), to_unsigned(1129,11), to_unsigned(1141,11), to_unsigned(1154,11),
        to_unsigned(1166,11), to_unsigned(1179,11), to_unsigned(1192,11), to_unsigned(1205,11),
        to_unsigned(1218,11), to_unsigned(1231,11), to_unsigned(1244,11), to_unsigned(1258,11),
        to_unsigned(1272,11), to_unsigned(1286,11), to_unsigned(1300,11), to_unsigned(1314,11),
        to_unsigned(1328,11), to_unsigned(1342,11), to_unsigned(1357,11), to_unsigned(1372,11),
        to_unsigned(1387,11), to_unsigned(1402,11), to_unsigned(1417,11), to_unsigned(1433,11),
        to_unsigned(1448,11), to_unsigned(1464,11), to_unsigned(1480,11), to_unsigned(1496,11),
        to_unsigned(1512,11), to_unsigned(1529,11), to_unsigned(1545,11), to_unsigned(1562,11),
        to_unsigned(1579,11), to_unsigned(1596,11), to_unsigned(1614,11), to_unsigned(1631,11),
        to_unsigned(1649,11), to_unsigned(1667,11), to_unsigned(1685,11), to_unsigned(1704,11),
        to_unsigned(1722,11), to_unsigned(1741,11), to_unsigned(1760,11), to_unsigned(1779,11),
        to_unsigned(1798,11), to_unsigned(1818,11), to_unsigned(1838,11), to_unsigned(1858,11),
        to_unsigned(1878,11), to_unsigned(1898,11), to_unsigned(1919,11), to_unsigned(1940,11),
        to_unsigned(1961,11), to_unsigned(1983,11), to_unsigned(2004,11), to_unsigned(2026,11));

    ----------------------------------------------------------------------
    -- port (fit for the stock SDK): per-frame decode tables in EBR, generated
    -- from a bit-exact model of the v2.1.3 sequencer arithmetic
    -- (cascade_tools/genrom.py).  s_lf is always in [-704, -129], so its low
    -- 10 bits (320..895) index the tables directly.
    --   C_SPROM: spacing s = C_MANT((-lf) mod 64) shifted by (-lf)/64 - 10
    --   C_QPROM: Q1 = clamp((2^(16-sft) * fnum) >> 8)  (was a vblank product)
    --   C_FNROM: fnum = C_MANT(lf mod 64)
    --   C_SLROM: s_eff (0..1023) -> octagon slack A (bits 19:10), C (9:0);
    --            s_eff >= 1024 always gives A = 640, C = 905
    ----------------------------------------------------------------------
    type t_rom12  is array(0 to 1023) of std_logic_vector(11 downto 0);
    type t_rom16  is array(0 to 1023) of std_logic_vector(15 downto 0);
    type t_rom12s is array(0 to 255)  of std_logic_vector(11 downto 0);
    type t_rom20  is array(0 to 1023) of std_logic_vector(19 downto 0);
    constant C_SPROM : t_rom12 := (
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"800", x"7EA", x"7D4", x"7BF", x"7A9", x"794", x"77F", x"76A", x"756", x"742",
        x"72E", x"71A", x"706", x"6F3", x"6E0", x"6CD", x"6BA", x"6A8", x"695", x"683", x"671",
        x"65F", x"64E", x"63C", x"62B", x"61A", x"609", x"5F9", x"5E8", x"5D8", x"5C8", x"5B8",
        x"5A8", x"599", x"589", x"57A", x"56B", x"55C", x"54D", x"53E", x"530", x"522", x"514",
        x"506", x"4F8", x"4EA", x"4DC", x"4CF", x"4C2", x"4B5", x"4A8", x"49B", x"48E", x"482",
        x"475", x"469", x"45D", x"451", x"445", x"439", x"42D", x"422", x"416", x"40B", x"400",
        x"3F5", x"3EA", x"3DF", x"3D4", x"3CA", x"3BF", x"3B5", x"3AB", x"3A1", x"397", x"38D",
        x"383", x"379", x"370", x"366", x"35D", x"354", x"34A", x"341", x"338", x"32F", x"327",
        x"31E", x"315", x"30D", x"304", x"2FC", x"2F4", x"2EC", x"2E4", x"2DC", x"2D4", x"2CC",
        x"2C4", x"2BD", x"2B5", x"2AE", x"2A6", x"29F", x"298", x"291", x"28A", x"283", x"27C",
        x"275", x"26E", x"267", x"261", x"25A", x"254", x"24D", x"247", x"241", x"23A", x"234",
        x"22E", x"228", x"222", x"21C", x"216", x"211", x"20B", x"205", x"200", x"1FA", x"1F5",
        x"1EF", x"1EA", x"1E5", x"1DF", x"1DA", x"1D5", x"1D0", x"1CB", x"1C6", x"1C1", x"1BC",
        x"1B8", x"1B3", x"1AE", x"1AA", x"1A5", x"1A0", x"19C", x"197", x"193", x"18F", x"18A",
        x"186", x"182", x"17E", x"17A", x"176", x"172", x"16E", x"16A", x"166", x"162", x"15E",
        x"15A", x"157", x"153", x"14F", x"14C", x"148", x"145", x"141", x"13E", x"13A", x"137",
        x"133", x"130", x"12D", x"12A", x"126", x"123", x"120", x"11D", x"11A", x"117", x"114",
        x"111", x"10E", x"10B", x"108", x"105", x"102", x"100", x"0FD", x"0FA", x"0F7", x"0F5",
        x"0F2", x"0EF", x"0ED", x"0EA", x"0E8", x"0E5", x"0E3", x"0E0", x"0DE", x"0DC", x"0D9",
        x"0D7", x"0D5", x"0D2", x"0D0", x"0CE", x"0CB", x"0C9", x"0C7", x"0C5", x"0C3", x"0C1",
        x"0BF", x"0BD", x"0BB", x"0B9", x"0B7", x"0B5", x"0B3", x"0B1", x"0AF", x"0AD", x"0AB",
        x"0A9", x"0A7", x"0A6", x"0A4", x"0A2", x"0A0", x"09F", x"09D", x"09B", x"099", x"098",
        x"096", x"095", x"093", x"091", x"090", x"08E", x"08D", x"08B", x"08A", x"088", x"087",
        x"085", x"084", x"082", x"081", x"080", x"07E", x"07D", x"07B", x"07A", x"079", x"077",
        x"076", x"075", x"074", x"072", x"071", x"070", x"06F", x"06E", x"06C", x"06B", x"06A",
        x"069", x"068", x"067", x"065", x"064", x"063", x"062", x"061", x"060", x"05F", x"05E",
        x"05D", x"05C", x"05B", x"05A", x"059", x"058", x"057", x"056", x"055", x"054", x"053",
        x"053", x"052", x"051", x"050", x"04F", x"04E", x"04D", x"04C", x"04C", x"04B", x"04A",
        x"049", x"048", x"048", x"047", x"046", x"045", x"045", x"044", x"043", x"042", x"042",
        x"041", x"040", x"040", x"03F", x"03E", x"03D", x"03D", x"03C", x"03B", x"03B", x"03A",
        x"03A", x"039", x"038", x"038", x"037", x"037", x"036", x"035", x"035", x"034", x"034",
        x"033", x"032", x"032", x"031", x"031", x"030", x"030", x"02F", x"02F", x"02E", x"02E",
        x"02D", x"02D", x"02C", x"02C", x"02B", x"02B", x"02A", x"02A", x"029", x"029", x"029",
        x"028", x"028", x"027", x"027", x"026", x"026", x"026", x"025", x"025", x"024", x"024",
        x"024", x"023", x"023", x"022", x"022", x"022", x"021", x"021", x"021", x"020", x"020",
        x"020", x"01F", x"01F", x"01E", x"01E", x"01E", x"01D", x"01D", x"01D", x"01D", x"01C",
        x"01C", x"01C", x"01B", x"01B", x"01B", x"01A", x"01A", x"01A", x"01A", x"019", x"019",
        x"019", x"018", x"018", x"018", x"018", x"017", x"017", x"017", x"017", x"016", x"016",
        x"016", x"016", x"015", x"015", x"015", x"015", x"014", x"014", x"014", x"014", x"014",
        x"013", x"013", x"013", x"013", x"013", x"012", x"012", x"012", x"012", x"012", x"011",
        x"011", x"011", x"011", x"011", x"010", x"010", x"010", x"010", x"010", x"010", x"00F",
        x"00F", x"00F", x"00F", x"00F", x"00E", x"00E", x"00E", x"00E", x"00E", x"00E", x"00E",
        x"00D", x"00D", x"00D", x"00D", x"00D", x"00D", x"00D", x"00C", x"00C", x"00C", x"00C",
        x"00C", x"00C", x"00C", x"00B", x"00B", x"00B", x"00B", x"00B", x"00B", x"00B", x"00B",
        x"00A", x"00A", x"00A", x"00A", x"00A", x"00A", x"00A", x"00A", x"00A", x"009", x"009",
        x"009", x"009", x"009", x"009", x"009", x"009", x"009", x"009", x"008", x"008", x"008",
        x"008", x"008", x"008", x"008", x"008", x"008", x"008", x"008", x"007", x"007", x"007",
        x"007", x"007", x"007", x"007", x"007", x"007", x"007", x"007", x"007", x"006", x"006",
        x"006", x"006", x"006", x"006", x"006", x"006", x"006", x"006", x"006", x"006", x"006",
        x"006", x"005", x"005", x"005", x"005", x"005", x"005", x"005", x"005", x"005", x"005",
        x"005", x"005", x"005", x"005", x"005", x"005", x"005", x"004", x"004", x"004", x"004",
        x"004", x"004", x"004", x"004", x"004", x"004", x"004", x"004", x"004", x"004", x"004",
        x"004", x"004", x"004", x"004", x"004", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000", x"000",
        x"000");
    constant C_QPROM : t_rom16 := (
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0020", x"0020", x"0020", x"0021", x"0021", x"0021", x"0022", x"0022", x"0022", x"0023",
        x"0023", x"0024", x"0024", x"0024", x"0025", x"0025", x"0026", x"0026", x"0026", x"0027",
        x"0027", x"0028", x"0028", x"0029", x"0029", x"0029", x"002A", x"002A", x"002B", x"002B",
        x"002C", x"002C", x"002D", x"002D", x"002E", x"002E", x"002F", x"002F", x"0030", x"0030",
        x"0031", x"0031", x"0032", x"0032", x"0033", x"0034", x"0034", x"0035", x"0035", x"0036",
        x"0037", x"0037", x"0038", x"0038", x"0039", x"003A", x"003A", x"003B", x"003B", x"003C",
        x"003D", x"003D", x"003E", x"003F", x"0040", x"0040", x"0041", x"0042", x"0042", x"0043",
        x"0044", x"0045", x"0045", x"0046", x"0047", x"0048", x"0048", x"0049", x"004A", x"004B",
        x"004C", x"004C", x"004D", x"004E", x"004F", x"0050", x"0051", x"0052", x"0053", x"0053",
        x"0054", x"0055", x"0056", x"0057", x"0058", x"0059", x"005A", x"005B", x"005C", x"005D",
        x"005E", x"005F", x"0060", x"0061", x"0062", x"0063", x"0064", x"0065", x"0067", x"0068",
        x"0069", x"006A", x"006B", x"006C", x"006E", x"006F", x"0070", x"0071", x"0072", x"0074",
        x"0075", x"0076", x"0077", x"0079", x"007A", x"007B", x"007D", x"007E", x"0080", x"0081",
        x"0082", x"0084", x"0085", x"0087", x"0088", x"008A", x"008B", x"008D", x"008E", x"0090",
        x"0091", x"0093", x"0095", x"0096", x"0098", x"0099", x"009B", x"009D", x"009F", x"00A0",
        x"00A2", x"00A4", x"00A6", x"00A7", x"00A9", x"00AB", x"00AD", x"00AF", x"00B1", x"00B3",
        x"00B5", x"00B7", x"00B9", x"00BB", x"00BD", x"00BF", x"00C1", x"00C3", x"00C5", x"00C7",
        x"00C9", x"00CB", x"00CE", x"00D0", x"00D2", x"00D5", x"00D7", x"00D9", x"00DC", x"00DE",
        x"00E0", x"00E3", x"00E5", x"00E8", x"00EA", x"00ED", x"00EF", x"00F2", x"00F5", x"00F7",
        x"00FA", x"00FD", x"0100", x"0102", x"0105", x"0108", x"010B", x"010E", x"0111", x"0114",
        x"0117", x"011A", x"011D", x"0120", x"0123", x"0126", x"012A", x"012D", x"0130", x"0133",
        x"0137", x"013A", x"013E", x"0141", x"0145", x"0148", x"014C", x"014F", x"0153", x"0157",
        x"015A", x"015E", x"0162", x"0166", x"016A", x"016E", x"0172", x"0176", x"017A", x"017E",
        x"0182", x"0186", x"018A", x"018F", x"0193", x"0197", x"019C", x"01A0", x"01A5", x"01AA",
        x"01AE", x"01B3", x"01B8", x"01BC", x"01C1", x"01C6", x"01CB", x"01D0", x"01D5", x"01DA",
        x"01DF", x"01E5", x"01EA", x"01EF", x"01F5", x"01FA", x"0200", x"0205", x"020B", x"0211",
        x"0216", x"021C", x"0222", x"0228", x"022E", x"0234", x"023A", x"0241", x"0247", x"024D",
        x"0254", x"025A", x"0261", x"0267", x"026E", x"0275", x"027C", x"0283", x"028A", x"0291",
        x"0298", x"029F", x"02A6", x"02AE", x"02B5", x"02BD", x"02C4", x"02CC", x"02D4", x"02DC",
        x"02E4", x"02EC", x"02F4", x"02FC", x"0304", x"030D", x"0315", x"031E", x"0327", x"032F",
        x"0338", x"0341", x"034A", x"0354", x"035D", x"0366", x"0370", x"0379", x"0383", x"038D",
        x"0397", x"03A1", x"03AB", x"03B5", x"03BF", x"03CA", x"03D4", x"03DF", x"03EA", x"03F5",
        x"0400", x"040B", x"0416", x"0422", x"042D", x"0439", x"0445", x"0451", x"045D", x"0469",
        x"0475", x"0482", x"048E", x"049B", x"04A8", x"04B5", x"04C2", x"04CF", x"04DC", x"04EA",
        x"04F8", x"0506", x"0514", x"0522", x"0530", x"053E", x"054D", x"055C", x"056B", x"057A",
        x"0589", x"0599", x"05A8", x"05B8", x"05C8", x"05D8", x"05E8", x"05F9", x"0609", x"061A",
        x"062B", x"063C", x"064E", x"065F", x"0671", x"0683", x"0695", x"06A8", x"06BA", x"06CD",
        x"06E0", x"06F3", x"0706", x"071A", x"072E", x"0742", x"0756", x"076A", x"077F", x"0794",
        x"07A9", x"07BF", x"07D4", x"07EA", x"0800", x"0816", x"082C", x"0844", x"085A", x"0872",
        x"088A", x"08A2", x"08BA", x"08D2", x"08EA", x"0904", x"091C", x"0936", x"0950", x"096A",
        x"0984", x"099E", x"09B8", x"09D4", x"09F0", x"0A0C", x"0A28", x"0A44", x"0A60", x"0A7C",
        x"0A9A", x"0AB8", x"0AD6", x"0AF4", x"0B12", x"0B32", x"0B50", x"0B70", x"0B90", x"0BB0",
        x"0BD0", x"0BF2", x"0C12", x"0C34", x"0C56", x"0C78", x"0C9C", x"0CBE", x"0CE2", x"0D06",
        x"0D2A", x"0D50", x"0D74", x"0D9A", x"0DC0", x"0DE6", x"0E0C", x"0E34", x"0E5C", x"0E84",
        x"0EAC", x"0ED4", x"0EFE", x"0F28", x"0F52", x"0F7E", x"0FA8", x"0FD4", x"1000", x"102C",
        x"1058", x"1088", x"10B4", x"10E4", x"1114", x"1144", x"1174", x"11A4", x"11D4", x"1208",
        x"1238", x"126C", x"12A0", x"12D4", x"1308", x"133C", x"1370", x"13A8", x"13E0", x"1418",
        x"1450", x"1488", x"14C0", x"14F8", x"1534", x"1570", x"15AC", x"15E8", x"1624", x"1664",
        x"16A0", x"16E0", x"1720", x"1760", x"17A0", x"17E4", x"1824", x"1868", x"18AC", x"18F0",
        x"1938", x"197C", x"19C4", x"1A0C", x"1A54", x"1AA0", x"1AE8", x"1B34", x"1B80", x"1BCC",
        x"1C18", x"1C68", x"1CB8", x"1D08", x"1D58", x"1DA8", x"1DFC", x"1E50", x"1EA4", x"1EFC",
        x"1F50", x"1FA8", x"2000", x"2058", x"20B0", x"2110", x"2168", x"21C8", x"2228", x"2288",
        x"22E8", x"2348", x"23A8", x"2410", x"2470", x"24D8", x"2540", x"25A8", x"2610", x"2678",
        x"26E0", x"2750", x"27C0", x"2830", x"28A0", x"2910", x"2980", x"29F0", x"2A68", x"2AE0",
        x"2B58", x"2BD0", x"2C48", x"2CC8", x"2D40", x"2DC0", x"2E40", x"2EC0", x"2F40", x"2FC8",
        x"3048", x"30D0", x"3158", x"31E0", x"3270", x"32F8", x"3388", x"3418", x"34A8", x"3540",
        x"35D0", x"3668", x"3700", x"3798", x"3830", x"38D0", x"3970", x"3A10", x"3AB0", x"3B50",
        x"3BF8", x"3CA0", x"3D48", x"3DF8", x"3EA0", x"3F50", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000");
    constant C_FNROM : t_rom12s := (
        x"400", x"40B", x"416", x"422", x"42D", x"439", x"445", x"451", x"45D", x"469", x"475",
        x"482", x"48E", x"49B", x"4A8", x"4B5", x"4C2", x"4CF", x"4DC", x"4EA", x"4F8", x"506",
        x"514", x"522", x"530", x"53E", x"54D", x"55C", x"56B", x"57A", x"589", x"599", x"5A8",
        x"5B8", x"5C8", x"5D8", x"5E8", x"5F9", x"609", x"61A", x"62B", x"63C", x"64E", x"65F",
        x"671", x"683", x"695", x"6A8", x"6BA", x"6CD", x"6E0", x"6F3", x"706", x"71A", x"72E",
        x"742", x"756", x"76A", x"77F", x"794", x"7A9", x"7BF", x"7D4", x"7EA", x"400", x"40B",
        x"416", x"422", x"42D", x"439", x"445", x"451", x"45D", x"469", x"475", x"482", x"48E",
        x"49B", x"4A8", x"4B5", x"4C2", x"4CF", x"4DC", x"4EA", x"4F8", x"506", x"514", x"522",
        x"530", x"53E", x"54D", x"55C", x"56B", x"57A", x"589", x"599", x"5A8", x"5B8", x"5C8",
        x"5D8", x"5E8", x"5F9", x"609", x"61A", x"62B", x"63C", x"64E", x"65F", x"671", x"683",
        x"695", x"6A8", x"6BA", x"6CD", x"6E0", x"6F3", x"706", x"71A", x"72E", x"742", x"756",
        x"76A", x"77F", x"794", x"7A9", x"7BF", x"7D4", x"7EA", x"400", x"40B", x"416", x"422",
        x"42D", x"439", x"445", x"451", x"45D", x"469", x"475", x"482", x"48E", x"49B", x"4A8",
        x"4B5", x"4C2", x"4CF", x"4DC", x"4EA", x"4F8", x"506", x"514", x"522", x"530", x"53E",
        x"54D", x"55C", x"56B", x"57A", x"589", x"599", x"5A8", x"5B8", x"5C8", x"5D8", x"5E8",
        x"5F9", x"609", x"61A", x"62B", x"63C", x"64E", x"65F", x"671", x"683", x"695", x"6A8",
        x"6BA", x"6CD", x"6E0", x"6F3", x"706", x"71A", x"72E", x"742", x"756", x"76A", x"77F",
        x"794", x"7A9", x"7BF", x"7D4", x"7EA", x"400", x"40B", x"416", x"422", x"42D", x"439",
        x"445", x"451", x"45D", x"469", x"475", x"482", x"48E", x"49B", x"4A8", x"4B5", x"4C2",
        x"4CF", x"4DC", x"4EA", x"4F8", x"506", x"514", x"522", x"530", x"53E", x"54D", x"55C",
        x"56B", x"57A", x"589", x"599", x"5A8", x"5B8", x"5C8", x"5D8", x"5E8", x"5F9", x"609",
        x"61A", x"62B", x"63C", x"64E", x"65F", x"671", x"683", x"695", x"6A8", x"6BA", x"6CD",
        x"6E0", x"6F3", x"706", x"71A", x"72E", x"742", x"756", x"76A", x"77F", x"794", x"7A9",
        x"7BF", x"7D4", x"7EA");
    constant C_SLROM : t_rom20 := (
        x"00000", x"00000", x"00000", x"00000", x"00401", x"00802", x"00C03", x"01005", x"01406",
        x"01807", x"01C08", x"0200B", x"0240C", x"0280D", x"02C0E", x"03010", x"03010", x"03411",
        x"03812", x"03C13", x"04016", x"04417", x"04818", x"04C19", x"0501B", x"0541C", x"0581D",
        x"05C1E", x"06021", x"06422", x"06823", x"06C24", x"06823", x"06C24", x"07026", x"07427",
        x"07828", x"07C29", x"0802D", x"0842E", x"0882F", x"08C30", x"09032", x"09433", x"09834",
        x"09C35", x"0A038", x"0A439", x"0A439", x"0A83A", x"0AC3B", x"0B03D", x"0B43E", x"0B83F",
        x"0BC40", x"0C043", x"0C444", x"0C845", x"0CC46", x"0D048", x"0D449", x"0D84A", x"0DC4B",
        x"0E04E", x"0DC4B", x"0E04E", x"0E44F", x"0E850", x"0EC51", x"0F053", x"0F454", x"0F855",
        x"0FC56", x"1005A", x"1045B", x"1085C", x"10C5D", x"1105F", x"11460", x"11861", x"11861",
        x"11C62", x"12065", x"12466", x"12867", x"12C68", x"1306A", x"1346B", x"1346B", x"1386C",
        x"13C6D", x"14070", x"14471", x"14872", x"14C73", x"15075", x"15075", x"15476", x"15877",
        x"15C78", x"1607B", x"1647C", x"1687D", x"16C7E", x"16C7E", x"17080", x"17481", x"17882",
        x"17C83", x"18087", x"18488", x"18889", x"18889", x"18C8A", x"1908C", x"1948D", x"1988E",
        x"19C8F", x"1A092", x"1A493", x"1A493", x"1A894", x"1AC95", x"1B097", x"1B498", x"1B899",
        x"1BC9A", x"1C09D", x"1C09D", x"1C49E", x"1C89F", x"1CCA0", x"1D0A2", x"1D4A3", x"1D8A4",
        x"1DCA5", x"1DCA5", x"1E0A8", x"1E4A9", x"1E8AA", x"1ECAB", x"1F0AD", x"1F4AE", x"1F8AF",
        x"1F8AF", x"1FCB0", x"200B5", x"204B6", x"208B7", x"20CB8", x"210BA", x"214BB", x"214BB",
        x"218BC", x"21CBD", x"220C0", x"224C1", x"228C2", x"22CC3", x"230C5", x"230C5", x"234C6",
        x"238C7", x"23CC8", x"240CB", x"244CC", x"248CD", x"24CCE", x"24CCE", x"250D0", x"254D1",
        x"258D2", x"25CD3", x"260D6", x"264D7", x"268D8", x"268D8", x"26CD9", x"270DB", x"274DC",
        x"278DD", x"27CDE", x"280E2", x"284E3", x"284E3", x"288E4", x"28CE5", x"290E7", x"294E8",
        x"298E9", x"29CEA", x"2A0ED", x"2A0ED", x"2A4EE", x"2A8EF", x"2ACF0", x"2B0F2", x"2B4F3",
        x"2B8F4", x"2BCF5", x"2BCF5", x"2C0F8", x"2C4F9", x"2C8FA", x"2CCFB", x"2D0FD", x"2D4FE",
        x"2D8FF", x"2D8FF", x"2DD00", x"2E103", x"2E504", x"2E905", x"2ED06", x"2F108", x"2F509",
        x"2F509", x"2F90A", x"2FD0B", x"3010F", x"30510", x"30911", x"30D12", x"31114", x"31114",
        x"31515", x"31916", x"31D17", x"3211A", x"3251B", x"3291C", x"32D1D", x"32D1D", x"3311F",
        x"33520", x"33921", x"33D22", x"34125", x"34526", x"34927", x"34927", x"34D28", x"3512A",
        x"3552B", x"3592C", x"35D2D", x"36130", x"36531", x"36531", x"36932", x"36D33", x"37135",
        x"37536", x"37937", x"37D38", x"3813C", x"3813C", x"3853D", x"3893E", x"38D3F", x"39141",
        x"39542", x"39943", x"39D44", x"39D44", x"3A147", x"3A548", x"3A949", x"3AD4A", x"3B14C",
        x"3B54D", x"3B94E", x"3B94E", x"3BD4F", x"3C152", x"3C553", x"3C954", x"3CD55", x"3D157",
        x"3D558", x"3D558", x"3D959", x"3DD5A", x"3E15D", x"3E55E", x"3E95F", x"3ED60", x"3F162",
        x"3F162", x"3F563", x"3F964", x"3FD65", x"4016A", x"4056B", x"4096C", x"40D6D", x"40D6D",
        x"4116F", x"41570", x"41971", x"41D72", x"42175", x"42576", x"42977", x"42977", x"42D78",
        x"4317A", x"4357B", x"4397C", x"43D7D", x"44180", x"44581", x"44581", x"44982", x"44D83",
        x"45185", x"45586", x"45987", x"45D88", x"4618B", x"4618B", x"4658C", x"4698D", x"46D8E",
        x"47190", x"47591", x"47992", x"47D93", x"47D93", x"48197", x"48598", x"48999", x"48D9A",
        x"4919C", x"4959D", x"4999E", x"4999E", x"49D9F", x"4A1A2", x"4A5A3", x"4A9A4", x"4ADA5",
        x"4B1A7", x"4B5A8", x"4B5A8", x"4B9A9", x"4BDAA", x"4C1AD", x"4C5AE", x"4C9AF", x"4CDB0",
        x"4D1B2", x"4D1B2", x"4D5B3", x"4D9B4", x"4DDB5", x"4E1B8", x"4E5B9", x"4E9BA", x"4EDBB",
        x"4EDBB", x"4F1BD", x"4F5BE", x"4F9BF", x"4FDC0", x"501C4", x"505C5", x"509C6", x"509C6",
        x"50DC7", x"511C9", x"515CA", x"519CB", x"51DCC", x"521CF", x"525D0", x"525D0", x"529D1",
        x"52DD2", x"531D4", x"535D5", x"539D6", x"53DD7", x"541DA", x"541DA", x"545DB", x"549DC",
        x"54DDD", x"551DF", x"555E0", x"559E1", x"55DE2", x"55DE2", x"561E5", x"565E6", x"569E7",
        x"56DE8", x"571EA", x"575EB", x"579EC", x"579EC", x"57DED", x"581F1", x"585F2", x"589F3",
        x"58DF4", x"591F6", x"595F7", x"595F7", x"599F8", x"59DF9", x"5A1FC", x"5A5FD", x"5A9FE",
        x"5ADFF", x"5B201", x"5B201", x"5B602", x"5BA03", x"5BE04", x"5C207", x"5C608", x"5CA09",
        x"5CE0A", x"5CE0A", x"5D20C", x"5D60D", x"5DA0E", x"5DE0F", x"5E212", x"5E613", x"5EA14",
        x"5EA14", x"5EE15", x"5F217", x"5F618", x"5FA19", x"5FE1A", x"6021F", x"60620", x"60620",
        x"60A21", x"60E22", x"61224", x"61625", x"61A26", x"61E27", x"6222A", x"6222A", x"6262B",
        x"62A2C", x"62E2D", x"6322F", x"63630", x"63A31", x"63E32", x"63E32", x"64235", x"64636",
        x"64A37", x"64E38", x"6523A", x"6563B", x"65A3C", x"65A3C", x"65E3D", x"66240", x"66641",
        x"66A42", x"66E43", x"67245", x"67646", x"67646", x"67A47", x"67E48", x"6824C", x"6864D",
        x"68A4E", x"68E4F", x"69251", x"69251", x"69652", x"69A53", x"69E54", x"6A257", x"6A658",
        x"6AA59", x"6AE5A", x"6AE5A", x"6B25C", x"6B65D", x"6BA5E", x"6BE5F", x"6C262", x"6C663",
        x"6CA64", x"6CA64", x"6CE65", x"6D267", x"6D668", x"6DA69", x"6DE6A", x"6E26D", x"6E66E",
        x"6E66E", x"6EA6F", x"6EE70", x"6F272", x"6F673", x"6FA74", x"6FE75", x"70279", x"70279",
        x"7067A", x"70A7B", x"70E7C", x"7127E", x"7167F", x"71A80", x"71E81", x"71E81", x"72284",
        x"72685", x"72A86", x"72E87", x"73289", x"7368A", x"73A8B", x"73A8B", x"73E8C", x"7428F",
        x"74690", x"74A91", x"74E92", x"75294", x"75695", x"75695", x"75A96", x"75E97", x"7629A",
        x"7669B", x"76A9C", x"76E9D", x"7729F", x"7729F", x"776A0", x"77AA1", x"77EA2", x"782A6",
        x"786A7", x"78AA8", x"78EA9", x"78EA9", x"792AB", x"796AC", x"79AAD", x"79EAE", x"7A2B1",
        x"7A6B2", x"7AAB3", x"7AAB3", x"7AEB4", x"7B2B6", x"7B6B7", x"7BAB8", x"7BEB9", x"7C2BC",
        x"7C6BD", x"7C6BD", x"7CABE", x"7CEBF", x"7D2C1", x"7D6C2", x"7DAC3", x"7DEC4", x"7E2C7",
        x"7E2C7", x"7E6C8", x"7EAC9", x"7EECA", x"7F2CC", x"7F6CD", x"7FACE", x"7FECF", x"7FECF",
        x"802D4", x"806D5", x"80AD6", x"80ED7", x"812D9", x"816DA", x"81ADB", x"81ADB", x"81EDC",
        x"822DF", x"826E0", x"82AE1", x"82EE2", x"832E4", x"836E5", x"836E5", x"83AE6", x"83EE7",
        x"842EA", x"846EB", x"84AEC", x"84EED", x"852EF", x"852EF", x"856F0", x"85AF1", x"85EF2",
        x"862F5", x"866F6", x"86AF7", x"86EF8", x"86EF8", x"872FA", x"876FB", x"87AFC", x"87EFD",
        x"88301", x"88702", x"88B03", x"88B03", x"88F04", x"89306", x"89707", x"89B08", x"89F09",
        x"8A30C", x"8A70D", x"8A70D", x"8AB0E", x"8AF0F", x"8B311", x"8B712", x"8BB13", x"8BF14",
        x"8C317", x"8C317", x"8C718", x"8CB19", x"8CF1A", x"8D31C", x"8D71D", x"8DB1E", x"8DF1F",
        x"8DF1F", x"8E322", x"8E723", x"8EB24", x"8EF25", x"8F327", x"8F728", x"8FB29", x"8FB29",
        x"8FF2A", x"9032E", x"9072F", x"90B30", x"90F31", x"91333", x"91734", x"91734", x"91B35",
        x"91F36", x"92339", x"9273A", x"92B3B", x"92F3C", x"9333E", x"9333E", x"9373F", x"93B40",
        x"93F41", x"94344", x"94745", x"94B46", x"94F47", x"94F47", x"95349", x"9574A", x"95B4B",
        x"95F4C", x"9634F", x"96750", x"96B51", x"96B51", x"96F52", x"97354", x"97755", x"97B56",
        x"97F57", x"9835B", x"9875C", x"9875C", x"98B5D", x"98F5E", x"99360", x"99761", x"99B62",
        x"99F63", x"9A366", x"9A366", x"9A767", x"9AB68", x"9AF69", x"9B36B", x"9B76C", x"9BB6D",
        x"9BF6E", x"9BF6E", x"9C371", x"9C772", x"9CB73", x"9CF74", x"9D376", x"9D777", x"9DB78",
        x"9DB78", x"9DF79", x"9E37C", x"9E77D", x"9EB7E", x"9EF7F", x"9F381", x"9F782", x"9F782",
        x"9FB83", x"9FF84", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389",
        x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389", x"A0389");

    ----------------------------------------------------------------------
    -- chain offset RAM (PIXEL units).  One read port (chain scan / span
    -- engine, time-disjoint) + one write port (chain scan).  The span
    -- engine reads both axes in parallel, so no separate ring table exists.
    -- |e[k]| < 16383 by the 640-px slack cap -> 15-bit words.
    ----------------------------------------------------------------------
    type t_ring is array(0 to C_NR - 1) of signed(14 downto 0);
    signal ram_ex, ram_ey : t_ring := (others => (others => '0'));
    signal ea_addr, ew_addr : integer range 0 to C_NR - 1 := 0;
    signal ew_wex, ew_wey : std_logic := '0';
    signal ew_dinx, ew_diny : signed(14 downto 0) := (others => '0');
    signal doa_ex, doa_ey : signed(14 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- 1bpp mark line buffers, one EBR per bank.  Engine writes the NEXT
    -- line's edge marks into bank s_lb_eng while bank s_lb_disp is read
    -- (and cleared 2 px behind the read) by the pixel path.
    ----------------------------------------------------------------------
    type t_lb is array(0 to 2047) of std_logic_vector(1 downto 0);
    signal lb0, lb1 : t_lb := (others => (others => '0'));
    signal lb_ra  : unsigned(10 downto 0) := (others => '0');
    signal lbq0, lbq1 : std_logic_vector(1 downto 0) := "00";
    signal lb0_we, lb1_we : std_logic;
    signal lb0_a,  lb1_a  : unsigned(10 downto 0);
    signal lb0_d,  lb1_d  : std_logic_vector(1 downto 0);
    signal s_lb_eng, s_lb_disp : std_logic := '0';

    -- registered mark-write channel (left edge at slot 7; the right edge is
    -- staged in lbp_* and issued one cycle later -- never the same cycle)
    signal lbm_we : std_logic := '0';
    signal lbm_a  : unsigned(10 downto 0) := (others => '0');
    signal lbm_d  : std_logic_vector(1 downto 0) := "01";      -- "01" enter / "11" leave
    signal lbp_we : std_logic := '0';
    signal lbp_a  : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_cx_k, s_cy_k : signed(10 downto 0) := (others => '0');      -- (Kn-512), deadbanded
    signal s_k3_per : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_k4_gr  : unsigned(9 downto 0) := (others => '0');
    signal s_gn     : unsigned(1 downto 0) := "00";  -- K4 GRADIENT: hard / 2 / 8 grays / smooth
    signal s_qp1    : unsigned(14 downto 0) := (others => '0');  -- stepped step: 2^16/s (per-ring)
    signal s_sm1    : unsigned(11 downto 0) := to_unsigned(41, 12);  -- spacing - 1 (m wrap)
    signal s_glx : std_logic := '1';          -- GLIDE (S7) -- default on
    signal s_dyn    : std_logic := '0';              -- cascade motion (S8)
    signal s_home   : std_logic := '0';              -- Catch-Up (S10): glide home
    signal s_vsat   : std_logic := '0';              -- S11: Rings / Video (refraction)
    -- P12 (v1.8): Rings mode = ZOOM (log-domain spacing offset), Video = wave depth
    signal s_p12    : unsigned(9 downto 0) := (others => '0');   -- latched at vsync
    signal s_lfb    : signed(15 downto 0) := to_signed(-462, 16);-- K3 log-spacing, before zoom
    signal s_zm     : unsigned(7 downto 0) := (others => '0');   -- zoom, Q6 octaves (0..191 = x8)
    -- K5 WIDTH (v1.9): band balance.  Every EVEN disc's radius moves by
    -- delta = f*s (f = (K5-512)/512, deadbanded at the centre), so bands
    -- alternate s+delta / s-delta; the cascade slack is derived from the
    -- NARROWEST band, and the outer field uses per-parity thresholds.
    signal s_k5     : unsigned(9 downto 0) := to_unsigned(512, 10);  -- raw, latched at vsync
    signal s_k5f    : unsigned(12 downto 0) := to_unsigned(4096, 13);-- eased (diff>>3 per run), Q3
    signal s_fd     : signed(13 downto 0) := (others => '0');   -- k5f - centre (Q3)
    signal s_fab    : unsigned(12 downto 0) := (others => '0'); -- |fd| (f = fd/4096)
    signal s_fdz    : std_logic := '1';                         -- inside the centre deadband
    signal s_dabs   : unsigned(12 downto 0) := (others => '0'); -- |delta| = |f|*s, capped s-4
    signal s_spm4   : unsigned(12 downto 0) := to_unsigned(38, 13); -- s - 4 (port)
    signal s_dneg   : std_logic := '0';                         -- delta < 0 (thinner white)
    signal s_seff   : unsigned(12 downto 0) := to_unsigned(42, 13); -- narrowest band = s - |delta|

    ----------------------------------------------------------------------
    -- runtime raster measurement
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);   -- x-pixel units (v2.1.5)
    signal s_avid_q : std_logic := '0';

    ----------------------------------------------------------------------
    -- v2.1.5 RESOLUTION SCALING: per-mode constants from the timing id
    -- (registers_in(8)), latched at vsync.  The engine's unit is the REAL
    -- x pixel; y is stepped in the same unit (virtual y step / virtual x
    -- step per FIELD line, Q4.8), so rings stay round on the 4:3 SD screen
    -- and interlace needs no separate detect.  At 1080p every value below
    -- is the old constant.  y steps (p_frame vsync latch): 480i 569
    -- ((2 x 1080/486) / 2), 480p 288, 576i 480, 576p 240, 1080i 512,
    -- 1080p / 720p 256.
    ----------------------------------------------------------------------
    -- mode class: "00" 1080 (x-step 1), "01" 720p (1.5), "10" SD (2.0)
    function f_cls(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return "10";
            when 5 | 6 | 14     => return "01";
            when others         => return "00";
        end case;
    end function;
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);  -- y step / field line, Q4.8 px
    signal s_cft   : unsigned(11 downto 0) := to_unsigned(257, 12);  -- first-line step 1 + ys (top field)
    signal s_cfb   : unsigned(11 downto 0) := to_unsigned(385, 12);  -- 1 + ys + ys/2 (bottom field)
    signal s_dyh   : std_logic := '0';                               -- (ys/2) >= 1 px: bottom line 0 is +1
    signal s_cls   : unsigned(1 downto 0) := "00";                   -- mode class (above)
    signal s_lfmin : signed(15 downto 0) := to_signed(-704, 16);     -- log-spacing floor, mode offset
    signal s_k3max : unsigned(9 downto 0) := to_unsigned(575, 10);   -- knob part max (spacing >= 4 px)
    signal s_k3l   : unsigned(9 downto 0) := to_unsigned(242, 10);   -- K3 * 9/16, clamped
    signal s_hacc  : unsigned(19 downto 0) := (others => '0');      -- field height, Q8 px units
    signal s_dq    : unsigned(3 downto 0) := (others => '0');        -- Video wave depth, Q1, per mode
    signal s_ncok  : std_logic := '1';                               -- ring count below the 720p cap

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');          -- glided centre (px)
    signal s_cxs, s_cys : signed(21 downto 0) := (others => '0');        -- glide accum, Q8
    signal s_lf   : signed(15 downto 0) := to_signed(-704, 16);          -- freq log2, Q6
    signal s_fnum : unsigned(10 downto 0) := to_unsigned(1024, 11);      -- freq mantissa
    signal s_sft  : integer range 0 to 15 := 8;                          -- freq shift (phi Q8)
    signal s_spac : unsigned(12 downto 0) := to_unsigned(42, 13);        -- ring spacing, px
    signal s_apx  : unsigned(11 downto 0) := to_unsigned(37, 12);        -- axis slack A, px
    signal s_cpx  : unsigned(12 downto 0) := to_unsigned(52, 13);        -- 45-deg slack C = A*sqrt2, px
    -- registered decode-table outputs (EBR)
    signal rom_sp : std_logic_vector(11 downto 0) := (others => '0');
    signal rom_qp : std_logic_vector(15 downto 0) := (others => '0');
    signal rom_fn : std_logic_vector(11 downto 0) := (others => '0');
    signal rom_sl : std_logic_vector(19 downto 0) := (others => '0');

    -- vblank sequencer + one shared multiplier (centre resolve + px conversion
    -- + the frame-constant tracker terms)
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_ma    : signed(13 downto 0) := (others => '0');
    signal s_mb    : signed(13 downto 0) := (others => '0');
    -- v2.0: the sequencer / frame tail request the ONE serial multiplier
    -- (in p_engine) with s_mreq + s_ma/s_mb, and stall until it is done.
    signal s_mreq  : std_logic := '0';
    -- REGISTERED step-enable for the sequencer and frame tail (the combined
    -- avid / engine-idle / multiplier-busy / request cone feeding every
    -- slot register's enable was the HD critical path).  A slot that issues
    -- a request also holds it low, so the 1-cycle-late decision can never
    -- step past a pending product.
    signal seq_go  : std_logic := '0';

    -- CASCADE chain FSM (once per frame in vblank, 4 clocks per ring)
    signal s_cxp, s_cyp : signed(13 downto 0) := (others => '0');  -- prev inner centre (px)
    signal s_chain_cnt : integer range 0 to C_NR := C_NR;
    signal s_chain_run : std_logic := '0';
    signal s_chain_wt  : std_logic := '0';                        -- armed, waiting for the decode
    signal s_chain_ph  : integer range 0 to 13 := 0;              -- serialized octagon pipeline
    signal s_doax, s_doay : signed(14 downto 0) := (others => '0');
    signal s_settle    : std_logic := '0';
    signal s_eprevx, s_eprevy : signed(14 downto 0) := (others => '0');
    signal s_dw    : signed(15 downto 0) := (others => '0');       -- shared step work reg
    signal s_clb, s_clbn : signed(15 downto 0) := (others => '0'); -- clamp bound, pre-registered
    signal s_qdx, s_qdy : signed(12 downto 0) := (others => '0');  -- axis-clamped step
    signal s_pu,  s_pv  : signed(13 downto 0) := (others => '0');  -- rotated, clamped step
    signal s_mvx, s_mvy : signed(9 downto 0) := (others => '0');
    signal s_chain_d : std_logic := '0';
    -- local registered "blanking" + slot-4 strobe for the chain FSM (the raw
    -- core avid and the s_seq compare in its enable cones were routing-
    -- critical; the chain only ever runs in vblank anyway)
    signal ch_blank : std_logic := '0';
    signal ch_go4   : std_logic := '0';

    -- ADAPTIVE ring count.  Ring k's radius is (k+1)*spacing, and the span
    -- engine clamps radii at C_RCL -- so at coarse spacings every ring past
    -- C_RCL/spacing PILES UP at the same radius and they all write their edge
    -- mark to the SAME line-buffer cell.  The cell holds one mark, so the pile
    -- collapses to a single toggle: (pile-1) marks vanish, and when that count
    -- is odd (54% of the Period range) the mark parity INVERTS for the rest of
    -- the line -- a hard vertical seam with the screen colour-flipped beyond
    -- it.  (Only reachable on HD with the centre knob near an extreme, which
    -- is why v0.8 shipped with it.)  Fix: model only the rings that actually
    -- fit under C_RCL.  s_nreff is that count, forced EVEN (mark parity == k
    -- parity) and >= 2, recomputed per frame during the chain scan; the outer
    -- field seams onto ring s_nreff-1 instead of ring C_NR-1.  It also makes
    -- the per-line span cost scale with the spacing rather than with C_NR.
    signal s_racc  : unsigned(13 downto 0) := (others => '0');     -- (k+1)*spacing, saturating
    signal s_nrw   : unsigned(6 downto 0) := to_unsigned(C_NR, 7); -- working count during the scan
    signal s_nreff : unsigned(6 downto 0) := to_unsigned(C_NR, 7); -- published, frame-stable
    signal s_ocap  : std_logic := '0';                             -- capture e[] into the outer centre
    signal s_codd  : std_logic := '1';                             -- s_chain_cnt is odd (toggle, not a mod)

    ----------------------------------------------------------------------
    -- frame-constant tracker terms (small p_frame tail after the chain,
    -- reusing its shared multiplier)
    ----------------------------------------------------------------------
    signal s_chain_q : std_logic := '0';                          -- chain_run delayed (edge)
    signal s_tlrun : std_logic := '0';
    signal s_tlph  : integer range 0 to 3 := 0;
    signal s_cv_done : std_logic := '0';                          -- pulse: frame terms ready
    signal s_dy0   : signed(14 downto 0) := (others => '0');      -- first active line's dy
    signal s_dx0   : signed(13 downto 0) := (others => '0');      -- dx0' = line-start dx - exo
    signal s_q0    : signed(15 downto 0) := (others => '0');      -- 2*dx0'+1
    signal s_dx0q  : unsigned(26 downto 0) := (others => '0');    -- dx0'^2

    ----------------------------------------------------------------------
    -- SPAN + LINE-SEED engine and the outer-field r^2 band tracker.
    ----------------------------------------------------------------------
    type t_est is (E_IDLE, E_CLEAR, E_SPAN, E_LINE);
    signal e_st : t_est := E_IDLE;
    signal e_cnt : integer range 0 to C_NR + 1 := 0;              -- ring group counter (2 drain groups)
    signal e_ph  : integer range 0 to 14 := 0;                    -- span slot (15-slot cadence)
    signal l_ph  : integer range 0 to 98 := 0;                    -- line-seed slot
    signal e_first : std_logic := '0';                            -- cv_done pass: chain SPAN->LINE
    -- per-group ring-count flags, registered at phase 0 (the 7-bit compares
    -- against s_nreff in every phase's enable cone were routing-critical)
    signal f_lt, f_mid, f_ge2, f_last : std_logic := '0';
    signal cl_hit  : std_logic := '0';                            -- E_CLEAR exit, registered
    signal l_pend  : std_logic := '0';                            -- avid fell while SPAN still busy
    signal cl_cnt  : unsigned(11 downto 0) := (others => '0');    -- E_CLEAR address counter
    signal cl_end  : unsigned(11 downto 0) := (others => '0');    -- E_CLEAR stop (W+2, latched)
    -- v2.0 SERIAL MULTIPLIER (replaces a 15x15 LUT array, ~570 LUT): signed
    -- 15-bit multiplicand x UNSIGNED 14-bit multiplier, shift-and-add, one
    -- 17-bit add per clock.  Loaded at slot T, 14 steps, product readable at
    -- slot T+15 -- exactly one 15-slot span group.  Every user is arranged
    -- so the multiplier operand is non-negative (squares use |x|).
    signal mu_m    : signed(14 downto 0) := (others => '0');      -- multiplicand
    signal mu_a    : signed(15 downto 0) := (others => '0');      -- high product bits
    signal mu_q    : unsigned(13 downto 0) := (others => '0');    -- multiplier -> low bits
    signal mu_cnt  : integer range 0 to 14 := 0;
    signal mu_busy : std_logic := '0';
    -- next-load operand registers: every load comes from these (the operand
    -- arithmetic + load-gating compares in front of mu_* were the HD
    -- critical path).  Request at T -> load at T+1 -> product at T+16.
    signal nx_m    : signed(14 downto 0) := (others => '0');
    signal nx_q    : unsigned(13 downto 0) := (others => '0');
    signal nx_ld   : std_logic := '0';
    signal m_p     : signed(29 downto 0);                         -- product view {mu_a, mu_q}

    signal sp_dy   : signed(14 downto 0) := (others => '0');      -- NEXT line's dy (from inner centre)
    signal sp_ey   : signed(14 downto 0) := (others => '0');
    signal sp_rr   : unsigned(11 downto 0) := (others => '0');    -- ring radius R (accumulated)
    signal sp_rq   : unsigned(11 downto 0) := (others => '0');    -- R +- |delta| on even discs (WIDTH)
    signal sp_ady  : unsigned(14 downto 0) := (others => '0');    -- |dy| pre-clamp (<= 24574)
    signal sp_emp  : std_logic := '0';                            -- |dy| >= R -> empty span
    signal sp_ex   : signed(14 downto 0) := (others => '0');      -- ex of the ring in front-end
    signal sp_exc  : signed(15 downto 0) := (others => '0');      -- cx + ex (RAW screen x of centre)
    signal sp_excq0, sp_excq1 : signed(15 downto 0) := (others => '0'); -- centre x, ring k-1 / k-2
    signal sp_empq0, sp_empq1, sp_empq2 : std_logic := '0';
    signal sp_wl, sp_wr1 : signed(15 downto 0) := (others => '0');    -- staged edges, raw x (slot 6)

    -- per-pass line-start ring index (C_NR minus the discs overhanging the
    -- left edge): written by the pass (working/_done), latched live at the
    -- line's avid rise.  (The hit test is the counter itself: k < C_NR.)
    signal m_kinit      : unsigned(6 downto 0) := to_unsigned(C_NR, 7);
    signal m_kinit_done : unsigned(6 downto 0) := to_unsigned(C_NR, 7);
    signal m0_w, m0_done : std_logic := '0';          -- a mark lands at x=0 (snap the seed)
    -- v1.9 LEFT-EDGE SEED: the nearest disc edge at or left of x=0 on the
    -- NEXT line (= the left boundary of the band x=0 sits in).  E_LINE roots
    -- its outer-centre radius r_L and seeds the per-ring phase with
    -- (r0 - r_L)*Q1 -- exactly how every on-screen band starts at its own
    -- left edge -- instead of the outer-centre phase, which is wrong for an
    -- offset nest (the edge band "came to a point" during catch-up).
    signal xl_w, xl_done   : signed(15 downto 0) := (others => '0');
    signal xlv_w, xlv_done : std_logic := '0';
    signal xc_r   : signed(15 downto 0) := (others => '0');       -- candidate, registered (timing)
    signal xcv_r  : std_logic := '0';
    signal le_r0  : unsigned(13 downto 0) := (others => '0');     -- r0, kept across the 2nd root
    signal le_dl  : signed(14 downto 0) := (others => '0');       -- r0 - r_L
    -- v2.0.1: E_LINE's own copy of the spacing (s_spac's fanout made the
    -- ring-index correction the HD Analog critical path) + the correction
    -- split into 2 clocks per step (candidates, then select)
    signal s_spl   : unsigned(12 downto 0) := to_unsigned(42, 13);
    attribute keep : boolean;
    attribute keep of s_spl : signal is true;
    -- port: every RAM here is collision-free on its CONSUMED reads (see the
    -- CHANGES.md notes), so yosys's read-before-write emulation is dropped
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_ex : signal is true;
    attribute no_rw_check of ram_ey : signal is true;
    attribute no_rw_check of lb0 : signal is true;
    attribute no_rw_check of lb1 : signal is true;
    -- v2.0.2: per-consumer copies of the per-frame WIDTH / spacing registers
    -- (one copy feeding the sequencer, the span engine AND the pixel path
    -- across the die was HD HDMI's recurring critical path)
    signal s_dabs_e, s_dabs_p : unsigned(12 downto 0) := (others => '0');
    signal s_dneg_e, s_dneg_p : std_logic := '0';
    signal s_seff_p : unsigned(12 downto 0) := to_unsigned(42, 13);
    signal s_shalf_p : unsigned(11 downto 0) := to_unsigned(21, 12);   -- s/2 (S9 moire)
    attribute keep of s_shalf_p : signal is true;
    signal s_spac_e : unsigned(12 downto 0) := to_unsigned(42, 13);
    attribute keep of s_dabs_e : signal is true;
    attribute keep of s_dabs_p : signal is true;
    attribute keep of s_dneg_e : signal is true;
    attribute keep of s_dneg_p : signal is true;
    attribute keep of s_seff_p : signal is true;
    attribute keep of s_spac_e : signal is true;
    signal le_dm, le_dp : signed(14 downto 0) := (others => '0');

    -- ONE 1-step/cycle sqrt unit (15-slot ring cadence; also roots r0).  The
    -- bit mask is a SHIFT REGISTER (>>2 per step), so a step is just an OR +
    -- one 27-bit subtract/compare -- short single-cycle path, 14 steps.
    -- port: the same exact integer sqrt (root + remainder), restoring, but in
    -- SHIFT form: the radicand shifts out 2 bits per step into a 15-bit
    -- remainder, so the step is a 17-bit compare/subtract (was 27-bit,
    -- in place).  Same 14 steps; only the final root/remainder are read.
    signal sq_n   : unsigned(27 downto 0) := (others => '0');     -- radicand, shifting out
    signal sq_r   : unsigned(14 downto 0) := (others => '0');     -- remainder (<= 2*root)
    signal sq_q   : unsigned(13 downto 0) := (others => '0');     -- root
    signal sq_cnt : unsigned(3 downto 0)  := (others => '0');     -- steps left

    -- line-seed temporaries
    signal le_dyp  : signed(13 downto 0) := (others => '0');      -- dy' = dy - eyo
    signal le_dysq : unsigned(26 downto 0) := (others => '0');    -- dy'^2 (kept for r_L^2)
    signal le_k0p  : unsigned(24 downto 0) := (others => '0');    -- r0 * fnum

    -- band tracker state (seeded per line by E_LINE, stepped per active pixel)
    signal t_kl   : unsigned(6 downto 0) := (others => '0');      -- ring index mod 128 (pixel path
                                                                  -- needs only k mod 2*maxwidth)
    signal t_k    : unsigned(11 downto 0) := (others => '0');     -- ring index (E_LINE-local)
    signal t_q    : signed(15 downto 0) := (others => '0');       -- 2*dx'+1 (r^2 step)
    signal su_m   : unsigned(11 downto 0) := (others => '0');     -- r mod spacing (ring phase)
    signal k_ld1, k_ld2 : unsigned(6 downto 0) := (others => '0'); -- t_kl, delayed to merge
    signal c_e1, c_o1   : std_logic := '0';                         -- outer-field WIDTH tests
    signal h1, h2       : std_logic := '0';                         -- S9: past half a band (outer centre)

    -- SMOOTH-fade machinery: a spacing-1 SUB-TRACKER (same candidate-arm
    -- structure, taps degenerate to +4/0/+2) detects every integer-radius
    -- crossing; a 16-bit phase accumulates +/-Q per crossing and its top
    -- bits fold straight into luma.  Seeded per line by E_LINE.
    signal su_u    : signed(16 downto 0) := (others => '0');      -- r^2 - rint^2
    signal su_qmd  : signed(17 downto 0) := (others => '0');      -- q - D1
    signal su_qpdm : signed(17 downto 0) := (others => '0');      -- q + D1 - 2
    signal ph1_acc : unsigned(15 downto 0) := (others => '0');    -- PER-RING phase (period 1 ring;
                                                                  -- SNAPPED to 0 at every disc edge)
    signal ph1_f1  : unsigned(7 downto 0) := (others => '0');
    signal su_cr   : std_logic_vector(1 downto 0) := "00";        -- crossing dir, 1-cycle delayed
    signal st_v    : std_logic := '0';                            -- s_avid_p, delayed
    signal lb_lo   : signed(14 downto 0) := (others => '0');      -- B_lo maintained thru corrections
    signal le_d    : signed(14 downto 0) := (others => '0');      -- d0 = r0 - B_lo
    signal su_d1   : signed(15 downto 0) := (others => '0');      -- D1 = 2*r0+1 (seed)

    -- outer-field recentre (ring C_NR-1's converted offset), frame-stable
    signal s_exo, s_eyo : signed(12 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel-coordinate accumulators.  dx per pixel, dy per active line.
    ----------------------------------------------------------------------
    signal s_dx    : signed(14 downto 0) := (others => '0');
    signal s_dya   : signed(22 downto 0) := (others => '0');   -- v2.1.5: Q8 dy accumulator
    signal s_dyb   : signed(14 downto 0);                      -- its integer part (next line's dy)
    signal s_dyc   : signed(14 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';
    signal s_avid_p2 : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- pixel pipeline: mark-parity path + disc-23 hit toggle + merge
    ----------------------------------------------------------------------
    signal cl_a1, cl_a2 : unsigned(10 downto 0) := (others => '0'); -- clear-behind addr pipe
    signal s_kx    : unsigned(6 downto 0) := (others => '0');     -- running ring index (stage 2)
    signal g2_hit : std_logic := '0';
    signal g2_k   : unsigned(6 downto 0) := (others => '0');

    signal r10_k    : unsigned(6 downto 0) := (others => '0');  -- ring index at the merge
    signal r10_t    : unsigned(7 downto 0) := (others => '0');  -- per-ring ramp at the merge
    signal r10_h    : std_logic := '0';                         -- S9 moire bit at the merge
    signal r12_key  : unsigned(7 downto 0) := (others => '0');
    signal s_out_y  : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S11 VIDEO: the incoming picture passes through UNTOUCHED in colour --
    -- the rings act on it only as REFRACTION (P12 Wave).  The saturation
    -- mod that used to live here (0..2x chroma about neutral) was dropped
    -- by the user once the refraction landed: the wave carries the mode.
    -- The video is NOT delayed to meet the field.  It used to go through a
    -- 30-bit BRAM delay line, which simulated pixel-exact but put noisy
    -- horizontal lines on real hardware -- silicon and GHDL do not agree on
    -- that inferred memory.  It is unnecessary anyway: the ring field is a
    -- SMOOTH pattern, so letting the gain lag the picture by a few pixels
    -- shifts the saturation pattern sideways by an invisible amount.  So the
    -- video takes its own 3-cycle path and the syncs are tapped to match.
    ----------------------------------------------------------------------
    type t_vd is array(0 to 63) of std_logic_vector(29 downto 0);
    signal s_vd    : t_vd := (others => (others => '0'));
    attribute no_rw_check of s_vd : signal is true;
    signal vd_wa   : unsigned(5 downto 0) := (others => '0');
    signal vd_ra   : unsigned(5 downto 0) := (others => '0');
    signal vd_q    : std_logic_vector(29 downto 0) := (others => '0');
    signal vd_hold : std_logic_vector(29 downto 0) := (others => '0');
    signal vd_tc   : unsigned(3 downto 0) := (others => '0');
    signal vd_px   : unsigned(5 downto 0) := (others => '0');  -- pixels written this line (sat 63)
    signal r13_y, r13_u, r13_v : unsigned(9 downto 0) := (others => '0');
    signal r14_y, r14_u, r14_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_u, s_out_v : unsigned(9 downto 0) := C_MID;
    -- S9 MOIRE (v2.1): XOR the rings with a half-phase copy of the rings
    -- around the outer centre.  At rest that DOUBLES the ring count; while
    -- the cascade has the rings offset (half-way points from the outer
    -- centre vs the true ring edges), moire flares and dissolves as they
    -- catch up.
    signal s_s9    : std_logic := '0';

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- chain offset RAM.  Read addr = chain scan (vblank) or the span engine
    -- (during the previous line's active video) -- time-disjoint.
    ------------------------------------------------------------------------
    ea_addr <= s_chain_cnt when s_chain_run = '1'
               else e_cnt when e_cnt < C_NR
               else 0;

    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            if ew_wex = '1' then ram_ex(ew_addr) <= ew_dinx; end if;
            if ew_wey = '1' then ram_ey(ew_addr) <= ew_diny; end if;
            doa_ex <= ram_ex(ea_addr);
            doa_ey <= ram_ey(ea_addr);
        end if;
    end process p_ram;

    ------------------------------------------------------------------------
    -- mark line buffers.  Each bank: one sync write + one sync read port.
    -- E_CLEAR (vblank) wipes both; otherwise the engine bank takes mark
    -- writes and the display bank is cleared 2 px behind the pixel read.
    ------------------------------------------------------------------------
    lb_ra <= s_xcnt(10 downto 0);

    lb0_we <= '1'        when e_st = E_CLEAR
         else lbm_we     when s_lb_eng = '0'
         else s_avid_p2  when s_lb_disp = '0'
         else '0';
    lb0_a  <= cl_cnt(10 downto 0) when e_st = E_CLEAR
         else lbm_a               when s_lb_eng = '0'
         else cl_a2;
    lb0_d  <= "00" when e_st = E_CLEAR
         else lbm_d when s_lb_eng = '0'
         else "00";

    lb1_we <= '1'        when e_st = E_CLEAR
         else lbm_we     when s_lb_eng = '1'
         else s_avid_p2  when s_lb_disp = '1'
         else '0';
    lb1_a  <= cl_cnt(10 downto 0) when e_st = E_CLEAR
         else lbm_a               when s_lb_eng = '1'
         else cl_a2;
    lb1_d  <= "00" when e_st = E_CLEAR
         else lbm_d when s_lb_eng = '1'
         else "00";

    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            if lb0_we = '1' then lb0(to_integer(lb0_a)) <= lb0_d; end if;
            if lb1_we = '1' then lb1(to_integer(lb1_a)) <= lb1_d; end if;
            lbq0 <= lb0(to_integer(lb_ra));
            lbq1 <= lb1(to_integer(lb_ra));
        end if;
    end process p_lbram;

    ------------------------------------------------------------------------
    -- per-frame decode tables (EBR, registered reads; addresses are frame-
    -- stable registers, consumed >= 2 sequencer slots after they change)
    ------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            rom_sp <= C_SPROM(to_integer(unsigned(s_lf(9 downto 0))));
            rom_qp <= C_QPROM(to_integer(unsigned(s_lf(9 downto 0))));
            rom_fn <= C_FNROM(to_integer(unsigned(s_lf(5 downto 0))));
            rom_sl <= C_SLROM(to_integer(s_seff(9 downto 0)));
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- raster measurement: width from the avid run, height from active lines.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                if s_xcnt(11 downto 5) /= 0 or (s_xcnt(4) = '1' and s_xcnt(3 downto 0) /= 0) then
                    s_W <= s_xcnt;                    -- > 16
                end if;
                s_xcnt <= (others => '0');
                s_hacc <= s_hacc + s_ys;              -- v2.1.5: height in x-pixel units
            end if;

            if s_vs_pulse = '1' then
                if s_hacc(19 downto 12) /= 0 or (s_hacc(11) = '1' and s_hacc(10 downto 8) /= 0) then  -- > 8
                    s_H <= s_hacc(19 downto 8);
                end if;
                s_hacc <= (others => '0');
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer + px conversion pass.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        -- 499 <= k <= 525 (= |k - 512| < C_DB, C_DB = 14) without comparators
        function f_detent(k : std_logic_vector(9 downto 0)) return boolean is
        begin
            if k(9) = '0' then   -- 499..511: k(8:4) = 11111 and k(3:0) >= 3
                return k(8 downto 4) = "11111" and (k(3 downto 2) /= "00" or k(1 downto 0) = "11");
            else                 -- 512..525: k(8:4) = 00000 and k(3:0) <= 13
                return k(8 downto 4) = "00000" and (k(3 downto 2) /= "11" or k(1) = '0');
            end if;
        end function;
        variable v_kx, v_ky : signed(10 downto 0);
        variable v_ct   : signed(27 downto 0);
        variable v_targ : signed(21 downto 0);
        variable v_nl   : signed(15 downto 0);
        variable v_ip   : integer;
        variable v_dyc  : unsigned(1 downto 0);
        variable v_lz   : signed(15 downto 0);
        variable v_pe   : signed(13 downto 0);
        variable v_req  : std_logic;
        variable v_hw   : unsigned(10 downto 0);
        variable v_acc  : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_cv_done    <= '0';
            s_chain_q    <= s_chain_run;
            s_mreq       <= '0';                 -- 1-clock multiplier request
            v_req        := '0';

            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';

                -- port: k - 512 is a bit flip, and |k - 512| < C_DB (14) is
                -- 499 <= k <= 525, tested on the knob bits
                v_kx := (others => not registers_in(0)(9));
                v_kx(8 downto 0) := signed(registers_in(0)(8 downto 0));
                v_ky := (others => not registers_in(1)(9));
                v_ky(8 downto 0) := signed(registers_in(1)(8 downto 0));
                if f_detent(registers_in(0)) then v_kx := (others => '0'); end if;
                if f_detent(registers_in(1)) then v_ky := (others => '0'); end if;
                s_cx_k <= v_kx;
                s_cy_k <= v_ky;

                s_k3_per <= unsigned(registers_in(2));
                s_k4_gr  <= unsigned(registers_in(3));
                s_k5     <= unsigned(registers_in(4));      -- K5 Width
                s_glx <= registers_in(6)(0);   -- S7 GLIDE
                s_dyn <= registers_in(6)(1);   -- S8 Cascade (uniform / sequential)
                s_s9  <= registers_in(6)(2);   -- S9: Moire
                s_home<= registers_in(6)(3);   -- S10 Catch-Up (stay-dragged / glide-home)
                s_vsat<= registers_in(6)(4);   -- S11 Output: Rings / Video (refraction)
                s_p12 <= unsigned(registers_in(7));            -- P12 Zoom (Rings) / Wave depth (Video)

                -- v2.1.5 per-mode scaling constants (timing id).  Per y step
                -- ys: first-line steps 1 + ys (top) / 1 + ys + ys/2 (bottom
                -- field), and whether the bottom field's line 0 sits a whole
                -- unit lower (ys/2 >= 256), as constants (no adders).
                case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                    when 0 =>          -- 480i
                        s_ys <= to_unsigned(569, 12); s_cft <= to_unsigned(570, 12);
                        s_cfb <= to_unsigned(854, 12); s_dyh <= '1';
                    when 4 =>          -- 480p
                        s_ys <= to_unsigned(288, 12); s_cft <= to_unsigned(289, 12);
                        s_cfb <= to_unsigned(433, 12); s_dyh <= '0';
                    when 8 =>          -- 576i
                        s_ys <= to_unsigned(480, 12); s_cft <= to_unsigned(481, 12);
                        s_cfb <= to_unsigned(721, 12); s_dyh <= '0';
                    when 12 =>         -- 576p
                        s_ys <= to_unsigned(240, 12); s_cft <= to_unsigned(241, 12);
                        s_cfb <= to_unsigned(361, 12); s_dyh <= '0';
                    when 1 | 2 | 10 => -- 1080i
                        s_ys <= to_unsigned(512, 12); s_cft <= to_unsigned(513, 12);
                        s_cfb <= to_unsigned(769, 12); s_dyh <= '1';
                    when others =>     -- 1080p, 720p
                        s_ys <= to_unsigned(256, 12); s_cft <= to_unsigned(257, 12);
                        s_cfb <= to_unsigned(385, 12); s_dyh <= '0';
                end case;
                s_cls <= f_cls(registers_in(8)(3 downto 0));
                case f_cls(registers_in(8)(3 downto 0)) is
                    when "10"   => s_lfmin <= to_signed(-640, 16); s_k3max <= to_unsigned(511, 10);
                    when "01"   => s_lfmin <= to_signed(-667, 16); s_k3max <= to_unsigned(538, 10);
                    when others => s_lfmin <= to_signed(-704, 16); s_k3max <= to_unsigned(575, 10);
                end case;

                s_seq <= to_unsigned(21, 5);

            elsif s_seq /= 0 and seq_go = '1' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is
                    -- v2.0: ONE product at a time on the serial multiplier.
                    -- A request slot raises s_mreq; the sequencer then stalls
                    -- (mu_busy) and the NEXT slot reads the finished product.

                    -- ZOOM (P12 in Rings mode): the slider subtracts up to 3
                    -- octaves from K3's log-spacing -> rings expand up to x8
                    -- from the centre (clamped at the 2048 px maximum).  Off
                    -- in Video mode, where P12 is the wave depth.
                    -- v2.1.5: the knob part K3*9/16 (0..575) is kept on its
                    -- own (s_k3l), clamped at slot 20 so the spacing stays
                    -- >= 4 px after the mode offset, and added to the mode's
                    -- floor s_lfmin at slot 19 (1080p: -704, never clamps).
                    when 21 =>
                        s_k3l <= resize(shift_right(resize(s_k3_per, 14)                -- K3 * 9/16
                                   + shift_left(resize(s_k3_per, 14), 3), 4), 10);
                        if s_vsat = '1' then
                            s_zm <= (others => '0');
                        else                                -- P12 * 3/16 -> 0..191
                            s_zm <= resize(shift_right(resize(s_p12, 12)
                                    + shift_left(resize(s_p12, 12), 1), 4), 8);
                        end if;

                    -- centre: cx = W/2 + (K1-512)*1.25W/1024 (+-0.625 W: the
                    -- centre goes ~12% past each edge at most)
                    when 20 =>
                        if s_k3l > s_k3max then s_k3l <= s_k3max; end if;
                        s_ma <= resize(s_cx_k, 14);
                        s_mb <= signed(resize(resize(s_W, 13) + shift_right(resize(s_W, 13), 2), 14));
                        s_mreq <= '1';  v_req := '1';
                    -- port: ONE glide datapath for both axes (slot 19 = x,
                    -- slot 18 = y; operands muxed by the slot parity)
                    when 19 | 18 =>
                        if s_seq(0) = '1' then
                            v_hw := s_W(11 downto 1);  v_acc := s_cxs;    -- slot 19: x
                        else
                            v_hw := s_H(11 downto 1);  v_acc := s_cys;    -- slot 18: y
                        end if;
                        v_ct := resize(m_p, 28);                          -- slot-20 / 19 product
                        v_targ := shift_left(resize(signed('0' & v_hw), 22), 8)
                                  + resize(shift_right(v_ct, 2), 22);
                        if s_glx = '1' then v_acc := v_acc + shift_right(v_targ - v_acc, C_GSH);
                        else                v_acc := v_targ; end if;
                      if s_seq(0) = '1' then
                        s_cxs <= v_acc;
                        s_lfb <= s_lfmin + signed(resize(s_k3l, 16));    -- v2.1.5
                        s_ma <= resize(s_cy_k, 14);                       -- centre Y
                        s_mb <= signed(resize(resize(s_H, 13) + shift_right(resize(s_H, 13), 2), 14));
                        s_mreq <= '1';  v_req := '1';
                      else
                        s_cys <= v_acc;
                        -- FREQUENCY: Lf(Q6) = -704 + Period*9/16 - zoom
                        -- (v2.1.5: lfb - zm < lfmin <=> k3l < zm -- a short
                        -- 10-bit compare beside the subtract; 1080p: -704)
                        v_lz := s_lfb - signed(resize(s_zm, 16));
                        if s_k3l < resize(s_zm, 10) then v_lz := s_lfmin; end if;  -- spacing <= 2048 (1080p)
                        s_lf <= v_lz;
                        -- K5 ease (pot noise would shimmer the ring edges)
                        v_pe   := signed(resize(s_k5f, 14)) + shift_right(
                                  signed(shift_left(resize(s_k5, 14), 3))
                                  - signed(resize(s_k5f, 14)), 3);
                        s_k5f  <= unsigned(v_pe(12 downto 0));
                      end if;
                    -- the decode is pipelined one step per slot (a single-slot
                    -- negate+ROM+barrel cone was hd_hdmi's critical path)
                    when 17 =>
                        s_fd  <= signed(resize(s_k5f, 14)) - to_signed(4096, 14);
                    -- SPACING (px per ring) = 2^(-Lf/64) = C_MANT(frac) >> (10-int).
                    -- Lf in [-704,-129] -> int in [2,11] -> spacing in [4,2048].
                    when 16 =>
                        -- WIDTH fraction f = fd/4096 = (K5-512)/512 -> +-1 over
                        -- the full knob; the s-4 cap binds only at the ends
                        s_fab  <= unsigned(resize(abs(s_fd), 13));   -- |fd| <= 4096
                        s_dneg <= s_fd(13);
                        if s_fd > -64 and s_fd < 64 then s_fdz <= '1'; else s_fdz <= '0'; end if;
                    -- port: spacing, Q1 = 2^16/s and fnum from the s_lf tables
                    -- (s_lf changed at slot 18; the EBR output is valid from
                    -- slot 16 on).  sft = clamp(2 - lf/64, 2, 15) = (not
                    -- lf(9:6)) + 3 over the reachable lf range.
                    when 15 =>
                        s_spac <= resize(unsigned(rom_sp), 13);
                        s_spl  <= resize(unsigned(rom_sp), 13);
                        s_qp1  <= resize(unsigned(rom_qp(13 downto 0)), 15);
                        s_fnum <= resize(unsigned(rom_fn), 11);
                        s_sft  <= to_integer((not unsigned(s_lf(9 downto 6))) + 3);
                    when 14 =>
                        s_spac_e <= s_spac;                          -- span-engine copy
                        s_spm4   <= s_spac - 4;                      -- port: slot-11 clamp bound
                    -- WIDTH: |delta| = |fd| * spacing >> 12.  Video mode and
                    -- the centre deadband force classic bands.
                    when 13 =>
                        if s_vsat = '1' or s_fdz = '1' then
                            s_ma <= (others => '0');
                        else
                            s_ma <= signed(resize(s_fab, 14));
                        end if;
                        s_mb <= signed(resize(s_spac, 14));
                        s_mreq <= '1';  v_req := '1';
                    when 12 =>
                        s_dabs <= unsigned(m_p(24 downto 12));       -- slot-13 product
                        -- port: Q1 now comes from C_QPROM; this request is a
                        -- dummy (product unused) that keeps the sequencer's
                        -- cycle timing identical to v2.1.3
                        s_mreq <= '1';  v_req := '1';
                    when 11 =>
                        -- bands >= 4 px.  port: dabs + 4 > s <=> dabs > s - 4 (s >= 4),
                        -- with s - 4 pre-registered at slot 14 (this add+compare
                        -- into the s_dabs enable was the own-HD HDMI critical path)
                        if s_dabs > s_spm4 then s_dabs <= s_spm4; end if;
                    when 10 =>
                        s_seff <= s_spac - s_dabs;
                    -- octagon slack in px from the NARROWEST band s_eff (pure
                    -- shift-adds, one short step per slot):
                    --   A = min(e - e/8,  min(e - e/16 - e/32 - 3, 640))
                    -- (~0.875*e; the cap terms keep adjacent edges > 1 px apart
                    -- on any scanline -- mark-write collision safety -- and the
                    -- chain words inside 15 bits)
                    --   C = A * ~sqrt2 = A + A/4 + A/8 + A/32 + A/128
                    when 9 =>
                        s_dabs_e <= s_dabs;  s_dabs_p <= s_dabs;    -- per-consumer copies
                        s_dneg_e <= s_dneg;  s_dneg_p <= s_dneg;
                        s_seff_p <= s_seff;
                        s_shalf_p <= resize(shift_right(s_spac, 1), 12);
                        -- K4 GRADIENT: four quarters of the knob
                        s_gn <= s_k4_gr(9 downto 8);
                        -- v2.1.5: Video wave depth L = P12 top 3 bits, in Q1
                        -- and scaled per mode: 1080 L, SD L/2, 720p ~2L/3
                        case s_cls is
                            when "10" => s_dq <= '0' & s_p12(9 downto 7);
                            when "01" =>
                                case to_integer(s_p12(9 downto 7)) is
                                    when 0 => s_dq <= to_unsigned(0, 4);
                                    when 1 => s_dq <= to_unsigned(1, 4);
                                    when 2 => s_dq <= to_unsigned(3, 4);
                                    when 3 => s_dq <= to_unsigned(4, 4);
                                    when 4 => s_dq <= to_unsigned(5, 4);
                                    when 5 => s_dq <= to_unsigned(7, 4);
                                    when 6 => s_dq <= to_unsigned(8, 4);
                                    when others => s_dq <= to_unsigned(9, 4);
                                end case;
                            when others => s_dq <= s_p12(9 downto 7) & '0';
                        end case;
                    -- port: A and C from the s_eff table (s_seff changed at
                    -- slot 10; valid from slot 8 on)
                    when 7 =>
                        if s_seff(12 downto 10) /= "000" then
                            s_apx <= to_unsigned(640, 12);
                            s_cpx <= to_unsigned(905, 13);
                        else
                            s_apx <= resize(unsigned(rom_sl(19 downto 10)), 12);
                            s_cpx <= resize(unsigned(rom_sl(9 downto 0)), 13);
                        end if;
                    when 5 =>
                        -- publish the glided centre in pixels
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);
                    when 4 =>
                        s_sm1 <= resize(s_spac - 1, 12);
                    -- (slots 3..1 free; the chain scan starts at s_seq = 4,
                    -- after the slack decode -- slot 0 never executes)
                    when others => null;
                end case;
            end if;

            --------------------------------------------------------------
            -- frame-terms tail: chain done -> the tracker's frame constants
            -- through the same shared multiplier (5 cycles).
            --------------------------------------------------------------
            if s_chain_q = '1' and s_chain_run = '0' then
                s_tlrun <= '1'; s_tlph <= 0;
            elsif s_tlrun = '1' and seq_go = '1' then
                case s_tlph is
                    when 0 =>
                        s_dx0 <= resize(-s_cx, 14) - resize(s_exo, 14);
                    when 1 =>
                        s_ma <= abs(s_dx0);  s_mb <= abs(s_dx0);  -- dx0'^2 (serial mult)
                        s_mreq <= '1';  v_req := '1';
                        s_q0 <= resize(s_dx0, 15) & '1';               -- 2*dx0'+1
                    when 2 =>
                        s_dx0q <= unsigned(m_p(26 downto 0));
                    when others =>                     -- phase 3: publish + done
                        -- first active line's dy (p_acc hasn't primed it yet)
                        -- -cy (+1 on interlaced field 1) = not(cy) + 1 (+1); exact
                        -- for cy /= -8192 (s_cy is within [-2560, 4606])
                        -- v2.1.5: the BOTTOM field (field_n = '0') sits ys/2
                        -- lower; its line 0 is a whole unit lower when ys/2
                        -- >= 1 px (480i, 1080i)
                        v_dyc := "01";
                        if s_dyh = '1' and data_in.field_n = '0' then v_dyc := "10"; end if;
                        s_dy0 <= resize(not s_cy, 15) + signed(resize(v_dyc, 15));
                        s_cv_done <= '1';
                        s_tlrun <= '0';
                end case;
                if s_tlph /= 3 then s_tlph <= s_tlph + 1; end if;
            end if;

            if data_in.avid = '0' and e_st = E_IDLE and mu_busy = '0'
               and s_mreq = '0' and nx_ld = '0' and v_req = '0' then
                seq_go <= '1';
            else
                seq_go <= '0';
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- coordinate accumulators.  dx loads -cx at each active-line start and
    -- steps +1 per pixel; dy loads -cy at the FIRST active line of the frame
    -- and steps per line.  v2.1.5: dy is a Q8 accumulator stepped s_ys per
    -- FIELD line (x-pixel units: interlace + SD pixel aspect included; 1.0
    -- at 1080p/720p), the bottom field starting ys/2 lower.  s_dyb, its
    -- integer part, is what the engine reads (unchanged at 1080p).
    ------------------------------------------------------------------------
    s_dyb <= s_dya(22 downto 8);

    p_acc : process(clk)
        variable v_yb : signed(22 downto 0);
        variable v_ys : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then      -- active-line start
                -- port: one adder.  First line: (-cy)*256 (+ ys/2 bottom field)
                -- + step = (not(cy) & FF) + 1 + ys (+ ys/2); later: dya + ys.
                v_ys := s_ys;
                if s_newframe = '1' then
                    v_yb := resize(not s_cy, 15) & "11111111";
                    if data_in.field_n = '0' then v_ys := s_cfb;
                    else                          v_ys := s_cft; end if;
                    s_newframe <= '0';
                else
                    v_yb := s_dya;
                end if;
                s_dya <= v_yb + signed(resize(v_ys, 23));
                s_dx <= resize(-s_cx, 15);
            elsif data_in.avid = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- CASCADE chain: a coupled chain of per-ring centre offsets, one ring per
    -- 4 clocks during vblank.  Ring 0 = the glided centre (offset 0).  Each
    -- frame every offset shifts by how far the inner ring moved (px), then is
    -- HARD-CLAMPED to the octagon around the ring inside -- the never-overlap
    -- guarantee AND the cascade dead-zone.  No accumulated cap: the wave
    -- travels the full C_NR-ring chain.  This scan also derives s_nreff, the
    -- number of rings the span engine will actually render this frame.
    ------------------------------------------------------------------------
    p_chain : process(clk)
        variable v_mvx, v_mvy : integer;
        variable vinx, viny : signed(15 downto 0);
        variable v_sq : signed(15 downto 0);
        variable v_sc : std_logic;
        variable v_sd : signed(16 downto 0);
        variable v_smx, v_smy : std_logic;
    begin
        if rising_edge(clk) then
            ch_blank <= not data_in.avid;
            if s_seq = 4 then ch_go4 <= '1'; else ch_go4 <= '0'; end if;
            -- v2.1.5: 720p ring cap.  The 1650-clk line holds the span pass
            -- 15*(n+2) + E_LINE 99 only for n <= 100, so ring s_chain_cnt
            -- may close the count (cnt+1) only while cnt <= 99.  Registered:
            -- s_chain_cnt is >= 13 clocks old when phase 13 reads this.
            if s_cls = "01" and s_chain_cnt >= 100 then s_ncok <= '0';
            else                                         s_ncok <= '1'; end if;
            ew_wex <= '0';                              -- default: no write
            ew_wey <= '0';
            s_ocap <= '0';                              -- default: 1-cycle pulse
            if s_vs_pulse = '1' then
                v_mvx := to_integer(s_cx - s_cxp);
                v_mvy := to_integer(s_cy - s_cyp);
                if    v_mvx >  255 then v_mvx :=  255;
                elsif v_mvx < -255 then v_mvx := -255; end if;
                if    v_mvy >  255 then v_mvy :=  255;
                elsif v_mvy < -255 then v_mvy := -255; end if;
                s_mvx <= to_signed(v_mvx, 10);
                s_mvy <= to_signed(v_mvy, 10);
                s_cxp <= s_cx;
                s_cyp <= s_cy;
                -- reset ring 0 to offset 0; the scan itself starts once the
                -- sequencer has finished the A/C slack decode (slot 5) so no
                -- ring ever clamps with stale or half-accumulated bounds
                ew_addr  <= 0; ew_dinx <= (others => '0'); ew_diny <= (others => '0');
                ew_wex   <= '1';  ew_wey <= '1';
                s_eprevx <= (others => '0'); s_eprevy <= (others => '0');
                s_chain_cnt <= 1; s_chain_ph <= 0; s_chain_wt <= '1';
            elsif s_chain_wt = '1' then
                if ch_go4 = '1' then          -- s_seq reached 4: slack decode done
                    s_chain_wt  <= '0';
                    s_chain_run <= '1';
                    -- ring 1's radius = 2*spacing (this frame's spacing lands
                    -- at sequencer slot 14, so it must be read HERE, not at
                    -- vsync); floor the effective ring count at 2
                    s_racc <= shift_left(resize(s_spac, 14), 1);   -- port: 2*s as a shift (x + x livelocks router1)
                    s_nrw  <= to_unsigned(2, 7);
                    s_codd <= '1';                  -- scan starts at ring 1
                end if;
            elsif s_chain_run = '1' and ch_blank = '1' then
                case s_chain_ph is
                when 0 =>
                    s_chain_ph <= 1;                    -- RD: issue e[k]_old read
                when 1 =>
                    -- LATCH: re-register the slow BRAM output.
                    s_doax <= doa_ex;  s_doay <= doa_ey;
                    s_chain_ph <= 2;
                when 2 | 5 =>
                    -- STEP d = (e_old - mv) - e_prev.  One shared unit,
                    -- x at 2, y at 5.
                    if s_chain_ph = 2 then
                        s_dw <= resize(s_doax, 16) - resize(s_mvx, 16) - resize(s_eprevx, 16);
                    else
                        s_dw <= resize(s_doay, 16) - resize(s_mvy, 16) - resize(s_eprevy, 16);
                    end if;
                    s_chain_ph <= s_chain_ph + 1;
                when 3 | 6 =>
                    -- SETTLE glide toward the ring inside, SYMMETRIC: pull the
                    -- step's MAGNITUDE down by ceil(|d|/2^C_SET) so it lands on
                    -- exactly 0 from either side.  (v0.8 used d + floor(-d/8),
                    -- i.e. d - ceil(d/8): exact for d > 0 but a NO-OP for
                    -- -7 <= d <= -1.  d is the step from the ring INSIDE, so a
                    -- drag right/down makes the rings lag left/up and d comes
                    -- out NEGATIVE -- the common case.  Every ring froze up to
                    -- 7 px off its parent, cumulative down the chain (161 px
                    -- at ring 23), and Catch-Up never finished centring.)
                    -- Also pre-register the next clamp's bound pair (keeps the
                    -- clamp cone to register -> compare -> mux).
                    -- port: one adder.  d - ceil(d/4) (d >= 0) and d - floor(d/4)
                    -- (d < 0) are both d + not(d >> 2) + c, c = 1 unless d >= 0
                    -- has a remainder.  (Exact for |d| < 32765; |d| here is
                    -- bounded by twice the ring-offset range, < 2^14.)
                    if s_settle = '1' then
                        v_sq := not shift_right(s_dw, C_SET);
                        if s_dw(15) = '1' or s_dw(C_SET - 1 downto 0) = 0 then v_sc := '1';
                        else v_sc := '0'; end if;
                        v_sd := (s_dw & '1') + (v_sq & v_sc);
                        s_dw <= v_sd(16 downto 1);
                    end if;
                    s_clb  <= signed(resize(s_apx, 16));
                    s_clbn <= -signed(resize(s_apx, 16));
                    s_chain_ph <= s_chain_ph + 1;
                when 8 | 10 =>
                    -- ROTATE: u = dx+dy (8), v = dx-dy (10) into the same reg
                    v_sq := resize(s_qdy, 16);
                    if s_chain_ph = 10 then v_sq := not v_sq; v_sc := '1';
                    else v_sc := '0'; end if;
                    v_sd := (resize(s_qdx, 16) & '1') + (v_sq & v_sc);
                    s_dw <= v_sd(16 downto 1);
                    s_clb  <= signed(resize(s_cpx, 16));
                    s_clbn <= -signed(resize(s_cpx, 16));
                    s_chain_ph <= s_chain_ph + 1;
                when 4 | 7 | 9 | 11 =>
                    -- ONE shared clamp cone: +/-A on the axis steps (4,7),
                    -- +/-C = A*sqrt2 on the rotated pair (9,11) -- the
                    -- octagon's 45-degree faces (a box alone would let
                    -- diagonal drags overlap the rings at this much slack)
                    viny := s_dw;
                    if    viny > s_clb  then viny := s_clb;
                    elsif viny < s_clbn then viny := s_clbn; end if;
                    case s_chain_ph is
                        when 4      => s_qdx <= resize(viny, 13);
                        when 7      => s_qdy <= resize(viny, 13);
                        when 9      => s_pu  <= resize(viny, 14);
                        when others => s_pv  <= resize(viny, 14);
                    end case;
                    s_chain_ph <= s_chain_ph + 1;
                when 12 | 13 =>
                    -- BACK-rotate (exact when unclipped: (u+v)/2 = dx),
                    -- rebuild the absolute offset and WRITE (x at 12, y at
                    -- 13).  |e[k]| <= k*(1.083*A+1.42) < 16383 by the A cap
                    -- -- no storage clamp needed.  Uniform mode writes zeros
                    -- so flipping S8 on starts from a clean nest.
                    v_sq := resize(s_pv, 16);
                    if s_chain_ph = 13 then v_sq := not v_sq; v_sc := '1';
                    else v_sc := '0'; end if;
                    v_sd := (resize(s_pu, 16) & '1') + (v_sq & v_sc);   -- pu +/- pv
                    if s_chain_ph = 12 then vinx := resize(s_eprevx, 16);
                    else                    vinx := resize(s_eprevy, 16); end if;
                    vinx := vinx + shift_right(v_sd(16 downto 1), 1);
                    if s_dyn = '0' then
                        vinx := (others => '0');
                    end if;
                    ew_addr <= s_chain_cnt;
                    if s_chain_ph = 12 then
                        ew_dinx  <= resize(vinx, 15);
                        ew_wex   <= '1';
                        s_eprevx <= resize(vinx, 15);
                        s_chain_ph <= 13;
                    else
                        ew_diny  <= resize(vinx, 15);
                        ew_wey   <= '1';
                        s_eprevy <= resize(vinx, 15);
                        s_chain_ph <= 0;
                        -- adaptive count: s_racc is ring s_chain_cnt's radius.
                        -- An ODD ring that still fits under C_RCL makes the
                        -- (even) count s_chain_cnt+1, and its offset is the
                        -- outer field's anchor.  Ring 1 always qualifies so a
                        -- 2048-px spacing still has an anchor.
                        s_codd <= not s_codd;
                        if s_codd = '1'
                           and ((s_racc <= C_RCL and s_ncok = '1') or s_chain_cnt = 1) then
                            s_nrw  <= to_unsigned(s_chain_cnt + 1, 7);
                            s_ocap <= '1';
                        end if;
                        if s_racc <= C_RCL then
                            s_racc <= s_racc + resize(s_spac, 14);
                        end if;
                        if s_chain_cnt = C_NR - 1 then
                            s_chain_run <= '0';
                        else
                            s_chain_cnt <= s_chain_cnt + 1;
                        end if;
                    end if;
                end case;
            end if;

            -- Catch-Up (S10): glide home only when armed AND settled.  From
            -- the REGISTERED movement (stable all frame; first chain use is
            -- 3 cycles after vsync).
            v_smx := '0';  v_smy := '0';
            if s_mvx(9 downto 1) = 0 or s_mvx = -1 then v_smx := '1'; end if;   -- -2 < mvx < 2
            if s_mvy(9 downto 1) = 0 or s_mvy = -1 then v_smy := '1'; end if;
            if s_dyn = '1' and s_home = '1' and v_smx = '1' and v_smy = '1' then
                s_settle <= '1';
            else
                s_settle <= '0';
            end if;

            -- outer-field recentre = ring s_nreff-1's offset, clamped from the
            -- REGISTERED write value one cycle after that ring's phase 13
            -- (ew_dinx/ew_diny hold until the next write, so the pulse can
            -- read them a cycle late -- same slot the old scan-end capture
            -- used, so the frame tail still sees the new value in time).
            if s_ocap = '1' then
                if    ew_dinx >  C_OCL then s_exo <= to_signed( C_OCL, 13);
                elsif ew_dinx < -C_OCL then s_exo <= to_signed(-C_OCL, 13);
                else                        s_exo <= resize(ew_dinx, 13); end if;
                if    ew_diny >  C_OCL then s_eyo <= to_signed( C_OCL, 13);
                elsif ew_diny < -C_OCL then s_eyo <= to_signed(-C_OCL, 13);
                else                        s_eyo <= resize(ew_diny, 13); end if;
            end if;

            -- publish the effective ring count one cycle after the scan ends
            s_chain_d <= s_chain_run;
            if s_chain_d = '1' and s_chain_run = '0' then
                s_nreff <= s_nrw;
            end if;
        end if;
    end process p_chain;

    ------------------------------------------------------------------------
    -- SPAN + LINE-SEED engine + the per-pixel r^2 BAND TRACKER.
    --
    -- E_SPAN runs a line EARLY: triggered at each active-line RISE, it
    -- computes the NEXT line's disc spans (24 rings x 8 slots ~ 210 clks,
    -- inside the 720+ clk active period) and writes their edge MARKS into
    -- the engine line-buffer bank.  Only the adds-only tracker runs per
    -- pixel, so the chain RAM, engine multiply and sqrt units are free.
    -- E_LINE (the tracker seed, ~44 clks) runs in the hblank as before.
    -- A frame's first line is produced by the cv_done chain in vblank:
    -- E_CLEAR (wipe both banks) -> E_SPAN -> E_LINE.
    --
    -- Active video, per pixel (tracker; all single adds):
    --   u   += q            (r^2 advance; q = 2*dx'+1, q += 2 per pixel)
    --   out (u+q >= D):     u := u + qmD;  k++   (qmD = q - D)
    --   in  (u+q < 0):      u := u + qpDm; k--   (qpDm = q + Dm)
    ------------------------------------------------------------------------
    p_engine : process(clk)
        variable v_dy, v_ad : signed(15 downto 0);
        variable v_w  : signed(15 downto 0);
        variable v_s1, v_o1, v_i1 : signed(17 downto 0);
        variable v_q2 : std_logic_vector(1 downto 0);
        variable v_ud : signed(28 downto 0);
        variable v_ki : unsigned(6 downto 0);
        variable v_wide : signed(15 downto 0);
        variable v_xl, v_xc : signed(15 downto 0);
        variable v_xv, v_cv : std_logic;
        -- serial-multiplier load request (any slot below may raise it)
        variable v_ld  : std_logic;
        variable v_lm  : signed(14 downto 0);
        variable v_lq  : unsigned(13 downto 0);
        variable v_sum : signed(16 downto 0);
        variable v_mq  : unsigned(13 downto 0);
        variable v_dxl : signed(14 downto 0);
        -- radix-4 sqrt step temporaries
        variable v_r2, v_t2 : unsigned(16 downto 0);
        variable v_phq : unsigned(15 downto 0);
        variable v_phs : unsigned(16 downto 0);
        variable v_dq  : unsigned(11 downto 0);
        variable v_dc  : std_logic;
        variable v_rq  : unsigned(12 downto 0);
        variable v_rs  : unsigned(12 downto 0);
        variable v_zl, v_zr, v_ler, v_lel : std_logic;
        variable v_cs  : signed(14 downto 0);
        variable v_cd  : signed(15 downto 0);
        variable v_tki : unsigned(11 downto 0);
        variable v_sqb, v_sqs : unsigned(26 downto 0);
        -- E_HERM temporaries
        variable v_hd : signed(15 downto 0);
        variable v_ix : integer range 0 to 7;
        variable v_rv : unsigned(17 downto 0);
        variable v_tp : unsigned(8 downto 0);
        variable v_nb : integer range 0 to 17;
        variable v_l  : signed(15 downto 0);
        variable v_fp : signed(29 downto 0);
        variable v_dp : signed(35 downto 0);
        variable v_rm : unsigned(17 downto 0);
        variable v_iap, v_oap : std_logic;
    begin
        if rising_edge(clk) then
            v_ld := '0';
            v_lm := (others => '0');
            v_lq := (others => '0');

            -- pending right-edge mark: issue one cycle after it was staged
            -- (left/right of one ring are 8 slots apart -- never collides)
            lbm_we <= '0';
            if lbp_we = '1' then
                lbm_we <= '1';
                lbm_a  <= lbp_a;
                lbm_d  <= "11";                       -- leave: k+1
                lbp_we <= '0';
            end if;

            -- the engine bank flips at every active-line start (the pass
            -- launched this line writes the NEXT line's marks)
            if data_in.avid = '1' and s_avid_p = '0' then
                s_lb_eng <= not s_lb_eng;
            end if;

            -- E_LINE request while the (early-started) span pass is busy --
            -- only possible on rasters with very short active lines (e.g.
            -- decimated simulation)
            if s_avid_p = '1' and data_in.avid = '0' and e_st /= E_IDLE then
                l_pend <= '1';
            end if;

            ------------------------------------------------------------
            -- sqrt unit: one radix-2 step per cycle while running.
            -- Loads (below) override these assignments.
            ------------------------------------------------------------
            if sq_cnt /= 0 then
                v_r2 := sq_r & sq_n(27 downto 26);
                v_t2 := resize(sq_q, 15) & "01";
                if v_r2 >= v_t2 then
                    v_r2 := v_r2 - v_t2;
                    sq_q <= sq_q(12 downto 0) & '1';
                else
                    sq_q <= sq_q(12 downto 0) & '0';
                end if;
                sq_r <= v_r2(14 downto 0);
                sq_cnt <= sq_cnt - 1;
            end if;
            sq_n <= sq_n(25 downto 0) & "00";
            -- one shared radicand adder for all three sqrt loads:
            -- E_SPAN m_p + 0, E_LINE 18 m_p + dx0'^2, E_LINE 34 m_p + dy'^2
            v_sqb := le_dysq;
            if e_st = E_SPAN then
                v_sqb := (others => '0');
            elsif l_ph = 18 then
                v_sqb := s_dx0q;
            end if;
            v_sqs := unsigned(m_p(26 downto 0)) + v_sqb;
            ----------------------------------------------------------
            -- TRACKER STEP -- runs on s_avid_p (so the line's LAST
            -- pixel steps too, on the fall cycle).  Concurrent with
            -- the span FSM below; they share no registers (E_LINE
            -- touches t_* only in hblank, when this doesn't run).
            ----------------------------------------------------------
            if s_avid_p = '1' then
                -- INTEGER-RADIUS TRACKER (spacing 1: taps degenerate to
                -- +4/0/+2, 17-bit arithmetic).  Every crossing is one radial
                -- pixel: the m = r mod s
                -- counter wraps -> ring index +/-1 (stepped mode).  This
                -- replaced the 28-bit r^2-bracket tracker outright.
                v_s1 := resize(su_u, 18) + resize(t_q, 18);
                v_o1 := resize(su_u, 18) + su_qmd;
                v_i1 := resize(su_u, 18) + su_qpdm;
                if v_s1 < 0 then                          -- inward: r-1
                    su_u    <= resize(v_i1, 17);
                    su_qmd  <= su_qmd + 4;
                    su_cr   <= "11";                      -- -1 (applied next cycle)
                    if su_m = 0 then
                        su_m <= resize(s_sm1, 12);
                        t_kl <= t_kl - 1;
                    else
                        su_m <= su_m - 1;
                    end if;
                elsif v_o1 >= 0 then                      -- outward: r+1
                    su_u    <= resize(v_o1, 17);
                    su_qpdm <= su_qpdm + 4;
                    su_cr   <= "01";                      -- +1 (applied next cycle)
                    if su_m = s_sm1 then
                        su_m <= (others => '0');
                        t_kl <= t_kl + 1;
                    else
                        su_m <= su_m + 1;
                    end if;
                else
                    su_u    <= resize(v_s1, 17);
                    su_qmd  <= su_qmd + 2;
                    su_qpdm <= su_qpdm + 2;
                    su_cr   <= "00";
                end if;
                t_q <= t_q + 2;
            end if;

            ----------------------------------------------------------
            -- PER-RING phase, applied ONE cycle after the crossing was
            -- detected so it lands on the same edge as the mark read
            -- (the mark data trails the crossing detect by a cycle):
            -- a disc-edge mark re-anchors the ramp to 0, else +/-Q1.
            ----------------------------------------------------------
            st_v <= s_avid_p;
            if st_v = '1' then
                v_q2 := lbq0;
                if s_lb_disp = '1' then v_q2 := lbq1; end if;
                v_phq := resize(s_qp1, 16);
                if su_cr(1) = '1' then v_phq := not v_phq; end if;
                v_phs := (ph1_acc & '1') + (v_phq & su_cr(1));      -- +/-Q1 (carry-in)
                if v_q2(0) = '1' then
                    ph1_acc <= (others => '0');
                elsif su_cr(0) = '1' then                     -- "01" / "11" ("10" never occurs)
                    ph1_acc <= v_phs(16 downto 1);
                end if;
            end if;

            case e_st is

            when E_IDLE =>
                if s_cv_done = '1' then
                    s_lb_eng <= '0';                  -- line 0's marks -> bank 0
                    e_first  <= '1';
                    cl_cnt   <= (others => '0');
                    cl_end   <= resize(s_W, 12) + 1;  -- registered equality exit (cl_hit)
                    cl_hit   <= '0';
                    e_st     <= E_CLEAR;              -- chain here was critical)
                elsif data_in.avid = '1' and s_avid_p = '0' then
                    sp_rr  <= resize(s_spac_e, 12);     -- pass for the NEXT line
                    m_kinit <= s_nreff;
                    m0_w   <= '0';
                    xlv_w  <= '0';
                    e_cnt  <= 0;  e_ph <= 0;
                    e_st   <= E_SPAN;
                elsif l_pend = '1' or (s_avid_p = '1' and data_in.avid = '0') then
                    l_pend <= '0';
                    l_ph   <= 0;
                    e_st   <= E_LINE;
                end if;

            when E_CLEAR =>
                -- wipe both banks 0..W+1 (write-port muxes route this state)
                cl_cnt <= cl_cnt + 1;
                if cl_cnt = cl_end then cl_hit <= '1'; else cl_hit <= '0'; end if;
                if cl_hit = '1' then                  -- cl_cnt = W+2, as before
                    sp_dy  <= s_dy0;                  -- frame's first line
                    sp_rr  <= resize(s_spac_e, 12);
                    m_kinit <= s_nreff;
                    m0_w   <= '0';
                    xlv_w  <= '0';
                    e_cnt  <= 0;  e_ph <= 0;
                    e_st   <= E_SPAN;
                end if;

            when E_SPAN =>
                case e_ph is
                    when 0 =>
                        -- chain RAM read settling; per-line trigger grabs the
                        -- next line's dy here (s_dyb updated at the rise edge)
                        f_lt   <= '0';  f_mid <= '0';  f_ge2 <= '0';  f_last <= '0';
                        if e_cnt < to_integer(s_nreff) then f_lt <= '1'; end if;
                        if e_cnt >= 1 and e_cnt <= to_integer(s_nreff) then f_mid <= '1'; end if;
                        if e_cnt >= 2 then f_ge2 <= '1'; end if;
                        if e_cnt = to_integer(s_nreff) + 1 then f_last <= '1'; end if;
                        if e_cnt = 0 and e_first = '0' then
                            sp_dy <= s_dyb;
                        end if;
                    when 1 =>
                        if f_lt = '1' then
                            sp_ey <= doa_ey;                  -- both axes arrive together
                            sp_ex <= doa_ex;
                        end if;
                    when 2 =>
                        if f_lt = '1' then
                            v_dy := resize(sp_dy, 16) - resize(sp_ey, 16);
                            v_ad := abs(v_dy);                -- <= 12500, bit 15 = 0
                            sp_ady <= unsigned(v_ad(14 downto 0));
                            sp_exc <= resize(sp_ex, 16) + resize(s_cx, 16);  -- raw screen x
                        end if;
                    when 3 =>
                        if f_lt = '1' then
                            if sp_ady > C_RCL then sp_emp <= '1'; sp_ady <= to_unsigned(C_RCL, 15);
                            else sp_emp <= '0'; end if;
                        end if;
                        -- WIDTH: even discs get R +- |delta| (odd discs, incl.
                        -- the outermost modelled one, keep R -> the outer seam
                        -- is untouched).  Stays in 4..4095 by the s-4 cap.
                        v_dq := resize(s_dabs_e, 12);
                        v_dc := s_dneg_e;
                        if s_dneg_e = '1' then v_dq := not v_dq; end if;
                        if (e_cnt mod 2) = 1 then
                            v_dq := (others => '0');  v_dc := '0';
                        end if;
                        v_rq := (sp_rr & '1') + (v_dq & v_dc);
                        sp_rq <= v_rq(12 downto 1);
                    when 4 =>
                        -- v2.0 SERIAL-MULTIPLIER PIPELINE (request here, load
                        -- next clock, product 16 clocks later = phase 5 of
                        -- the next group).  Multiply ring k:
                        --   w^2 = (R - |dy|) * (R + |dy|)
                        if f_lt = '1' then
                            v_ld := '1';
                            v_lm := signed(resize(sp_rq, 15)) - signed(resize(sp_ady(12 downto 0), 15));
                            v_lq := resize(sp_rq, 14) + resize(sp_ady(12 downto 0), 14);
                        end if;
                    when 5 =>
                        -- all three read pre-edge values:
                        --  (a) stage ring k-2's edges from the finished root
                        v_w := signed(resize(sq_q(12 downto 0), 16));
                        sp_wl  <= sp_excq1 - v_w;
                        sp_wr1 <= sp_excq1 + v_w + 1;         -- first x OUTSIDE
                        sp_empq2 <= sp_empq1;
                        --  (b) root ring k-1's product (complete this slot)
                        if f_mid = '1' then
                            if m_p < 0 then sq_n <= (others => '0');
                            else sq_n <= '0' & v_sqs; end if;
                            sq_r <= (others => '0');
                            sq_q <= (others => '0');
                            sq_cnt <= to_unsigned(14, 4);
                        end if;
                        sp_excq1 <= sp_excq0;
                        if m_p < 0 then sp_empq1 <= '1';
                        else            sp_empq1 <= sp_empq0; end if;
                        --  (c) ring k's centre/empty flag ride with its product
                        sp_excq0 <= sp_exc;
                        sp_empq0 <= sp_emp;
                    when 6 =>
                        -- ring e_cnt-2: DIRECTION-coded edge marks -> engine
                        -- bank (left/enter now, right/leave staged one cycle);
                        -- discs overhanging the left screen edge decrement
                        -- the line-start ring index instead of a mark.
                        if f_ge2 = '1' then
                            v_ki := m_kinit;
                            v_wide := signed(resize(s_W, 16));
                            -- nearest-edge candidate at/left of x=0 (wr1 > wl,
                            -- so the right edge wins when both qualify);
                            -- compared into xl_w at phase 6 (a one-cycle cone
                            -- here was the HD critical path)
                            v_zl := '0';  v_zr := '0';
                            if sp_wl  = 0 then v_zl := '1'; end if;
                            if sp_wr1 = 0 then v_zr := '1'; end if;
                            v_ler := sp_wr1(15) or v_zr;          -- sp_wr1 <= 0
                            v_lel := sp_wl(15)  or v_zl;          -- sp_wl  <= 0
                            if v_ler = '1' then xc_r <= sp_wr1;
                            else                xc_r <= sp_wl; end if;
                            if sp_empq2 = '0' and (v_ler = '1' or v_lel = '1') then
                                xcv_r <= '1';
                            else
                                xcv_r <= '0';
                            end if;
                            if sp_empq2 = '0' then
                                if sp_wl(15) = '1' and v_ler = '0' then  -- wl < 0 and wr1 > 0
                                    v_ki := v_ki - 1;
                                end if;
                                if sp_wl(15) = '0' and sp_wl < v_wide then
                                    lbm_we <= '1';
                                    lbm_a  <= unsigned(sp_wl(10 downto 0));
                                    lbm_d  <= "01";           -- enter: k-1
                                    if v_zl = '1' then
                                        m0_w <= '1';          -- pixel 0 can't see its
                                    end if;                   -- own mark: snap the seed
                                end if;
                                if v_ler = '0' and sp_wr1 < v_wide then
                                    lbp_we <= '1';
                                    lbp_a  <= unsigned(sp_wr1(10 downto 0));
                                end if;
                            end if;
                            m_kinit <= v_ki;
                        end if;
                    when 7 =>
                        -- next ring's radius
                        if f_lt = '1' then
                            v_rs := resize(sp_rr, 13) + resize(s_spac_e, 13);
                            if v_rs(12) = '1' then                -- > C_RCL = 4095
                                sp_rr <= to_unsigned(C_RCL, 12);
                            else
                                sp_rr <= v_rs(11 downto 0);
                            end if;
                        end if;
                        -- nearest-edge update, then (last ring) capture the
                        -- line-start state and leave
                        if f_ge2 = '1' then
                            v_xl := xl_w;  v_xv := xlv_w;
                            if xcv_r = '1' and (xlv_w = '0' or xc_r > xl_w) then
                                v_xl := xc_r;  v_xv := '1';
                            end if;
                            xl_w <= v_xl;  xlv_w <= v_xv;
                            if f_last = '1' then
                                m_kinit_done <= m_kinit;
                                m0_done      <= m0_w;
                                xl_done      <= v_xl;
                                xlv_done     <= v_xv;
                                -- drain groups done
                                if e_first = '1' then         -- vblank chain
                                    e_first <= '0';
                                    l_ph <= 0;
                                    e_st <= E_LINE;
                                elsif l_pend = '1' then
                                    l_pend <= '0';
                                    l_ph <= 0;
                                    e_st <= E_LINE;
                                else
                                    e_st <= E_IDLE;
                                end if;
                            end if;
                        end if;
                    when others => null;                      -- sqrt stepping
                end case;
                if e_ph = 14 then
                    e_ph <= 0;
                    e_cnt <= e_cnt + 1;
                else
                    e_ph <= e_ph + 1;
                end if;

            when E_LINE =>
                -- v2.0: re-scheduled for the 15-slot serial multiplier (load
                -- at T, product at T+15); the two roots overlap the products.
                case l_ph is
                    when 0 =>
                        le_dyp <= resize(resize(sp_dy, 16) - resize(s_eyo, 16), 14);
                        t_q <= s_q0;                                      -- 2*dx0'+1 (frame tail)
                    when 1 =>                                             -- dy'^2
                        v_ld := '1';
                        v_lm := resize(abs(le_dyp), 15);
                        v_lq := unsigned(resize(abs(le_dyp), 14));
                    when 17 =>
                        le_dysq <= unsigned(m_p(26 downto 0));            -- dy'^2
                    when 18 =>
                        sq_n <= '0' & v_sqs;                              -- root r0^2 = dx0'^2 + dy'^2
                        sq_r <= (others => '0');
                        sq_q <= (others => '0');
                        sq_cnt <= to_unsigned(14, 4);
                        -- LEFT-EDGE SEED: (x_L - outer cx)^2, in parallel
                        v_dxl := abs(resize(xl_done, 15) + resize(s_dx0, 15));   -- <= ~10.5k
                        v_ld := '1';
                        v_lm := v_dxl;
                        v_lq := unsigned(v_dxl(13 downto 0));
                    when 33 =>
                        -- r0 done: tracker seeds (u1 = the sqrt REMAINDER,
                        -- free; D1 = 2*r0+1) and r0 * fnum
                        le_r0 <= sq_q;
                        su_u  <= signed(resize(sq_r, 17));
                        su_d1 <= signed(resize(shift_left(resize(sq_q, 15), 1), 16)) + 1;
                        v_ld := '1';
                        v_lm := signed(resize(sq_q, 15));
                        v_lq := resize(s_fnum, 14);
                    when 34 =>                                            -- root r_L^2 = dy'^2 + slot-18 product
                        sq_n <= '0' & v_sqs;                              -- (one clock earlier than v2.1.3's
                        sq_r <= (others => '0');                          --  slot 35; read at slot 50)
                        sq_q <= (others => '0');
                        sq_cnt <= to_unsigned(14, 4);
                    when 36 =>
                        su_qmd  <= resize(t_q, 18) - resize(su_d1, 18);
                        su_qpdm <= resize(t_q, 18) + resize(su_d1, 18) - 2;
                    when 49 =>
                        le_k0p <= unsigned(m_p(24 downto 0));             -- r0 * fnum
                    when 50 =>
                        t_k   <= resize(shift_right(le_k0p, s_sft + 8), 12);
                        le_dl <= signed(resize(le_r0, 15)) - signed(resize(sq_q, 15));
                    when 51 =>                                            -- B_lo = k * spacing
                        v_ld := '1';
                        v_lm := signed(resize(t_k, 15));
                        v_lq := resize(s_spl, 14);
                    when 67 =>
                        lb_lo <= m_p(14 downto 0);
                    when 68 =>
                        -- d0 = r0 - B_lo (estimate error is a few multiples
                        -- of s -- corrected LINEARLY below, 15-bit ops only)
                        le_d <= signed(resize(le_r0, 15)) - lb_lo;
                    when 69 | 71 | 73 | 75 | 77 | 79 =>
                        -- correction step, clock A: the one candidate that can
                        -- be taken: d + s if d < 0, else d - s
                        v_cs := signed(resize(s_spl, 15));
                        if le_d(14) = '0' then v_cs := not v_cs; end if;
                        v_cd := (le_d & '1') + (v_cs & (not le_d(14)));
                        le_dm <= v_cd(15 downto 1);
                    when 70 | 72 | 74 | 76 | 78 | 80 =>
                        -- clock B: select (d < 0 -> +s, k-1; d >= s -> -s, k+1)
                        v_tki := (others => le_d(14));  v_tki(0) := '1';   -- -1 / +1
                        if le_d(14) = '1' or le_dm(14) = '0' then
                            t_k  <= t_k + v_tki;
                            le_d <= le_dm;
                        end if;
                    when 81 =>
                        -- publish ring index + ring phase m = d0; seed the
                        -- per-ring phase at x=0: (r0 - r_L)*Q1 when x=0 lies
                        -- in the cascade nest and its band's left edge is
                        -- known, else the outer-centre phase d0*Q1 (exact for
                        -- the outer field and in Uniform mode)
                        t_kl  <= t_k(6 downto 0);
                        su_m  <= unsigned(le_d(11 downto 0));
                        v_ld := '1';
                        if xlv_done = '1' and s_dyn = '1' and m_kinit_done /= s_nreff then
                            v_lm := le_dl;
                        else
                            v_lm := le_d;
                        end if;
                        v_lq := resize(s_qp1, 14);
                    when 97 =>
                        if m0_done = '1' then
                            ph1_acc <= (others => '0');
                        else
                            ph1_acc <= unsigned(m_p(15 downto 0));
                        end if;
                    when 98 =>
                        e_st <= E_IDLE;
                    when others => null;
                end case;
                if l_ph < 98 then l_ph <= l_ph + 1; end if;

            end case;

            ----------------------------------------------------------
            -- SERIAL MULTIPLIER.  Sequencer / frame-tail requests are
            -- time-disjoint with the engine (they only issue, and only
            -- load, while it is idle).  Product = {mu_a, mu_q}.
            ----------------------------------------------------------
            if s_mreq = '1' and e_st = E_IDLE then
                v_ld := '1';
                v_lm := resize(s_ma, 15);
                v_lq := unsigned(s_mb);
            end if;
            nx_ld <= v_ld;
            nx_m  <= v_lm;
            nx_q  <= v_lq;
            if nx_ld = '1' then
                mu_m    <= nx_m;
                mu_q    <= nx_q;
                mu_a    <= (others => '0');
                mu_cnt  <= 14;
                mu_busy <= '1';
            elsif mu_cnt /= 0 then
                if mu_q(0) = '1' then
                    v_sum := resize(mu_a, 17) + resize(mu_m, 17);
                else
                    v_sum := resize(mu_a, 17);
                end if;
                v_mq := shift_right(mu_q, 1);
                v_mq(13) := v_sum(0);
                mu_a <= v_sum(16 downto 1);
                mu_q <= v_mq;
                mu_cnt <= mu_cnt - 1;
                if mu_cnt = 1 then mu_busy <= '0'; end if;
            end if;
        end if;
    end process p_engine;

    m_p <= shift_left(resize(mu_a, 30), 14) or signed(resize(mu_q, 30));

    ------------------------------------------------------------------------
    -- tracker parity, delayed to meet the mark-parity path at the merge
    ------------------------------------------------------------------------
    p_par : process(clk)
        variable v_kw : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            k_ld1 <= t_kl;
            -- WIDTH (outer field): band k (radius k*s .. (k+1)*s, m = r mod s)
            -- is white if  k even and m < te,  or  k odd and m < to, with
            -- delta < 0: te = s - |d|, to = 0;  delta >= 0: te = s, to = |d|.
            if s_dneg_p = '0' or su_m < s_seff_p then c_e1 <= '1'; else c_e1 <= '0'; end if;
            if s_dneg_p = '0' and su_m < s_dabs_p then c_o1 <= '1'; else c_o1 <= '0'; end if;
            -- S9 MOIRE: past half a band, aligned with k_ld1 -> k_ld2
            if su_m >= s_shalf_p then h1 <= '1'; else h1 <= '0'; end if;
            h2 <= h1;
            v_kw := k_ld1;
            v_kw(0) := not ((not k_ld1(0) and c_e1) or (k_ld1(0) and c_o1));
            k_ld2 <= v_kw;
            -- per-ring stepped ramp: triangle fold, DARK at the ring edges
            -- (phase 0), bright mid-ring.  (One delay reg only:
            -- ph1's update runs a cycle after the crossing detect.)
            if ph1_acc(15) = '1' then ph1_f1 <= not ph1_acc(14 downto 7);
            else                      ph1_f1 <= ph1_acc(14 downto 7); end if;
        end if;
    end process p_par;

    ------------------------------------------------------------------------
    -- GEO path: running XOR over the mark line buffer = parity of the
    -- covering-disc count; nesting makes that the innermost disc's parity
    -- (C_NR even).  One pair of equality toggles tracks disc C_NR-1
    -- exactly -- the handoff boundary to the outer tracker.  Gated on the
    -- DELAYED avid so the line's last pixel updates too.
    ------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_q : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p2 <= s_avid_p;
            cl_a1     <= s_xcnt(10 downto 0);
            cl_a2     <= cl_a1;

            if data_in.avid = '1' and s_avid_p = '0' then         -- line start
                s_kx    <= resize(m_kinit_done, 7);
                s_lb_disp <= s_lb_eng;      -- display the bank just written
            elsif s_avid_p = '1' then
                v_q := lbq0;
                if s_lb_disp = '1' then v_q := lbq1; end if;
                if v_q(0) = '1' then
                    if v_q(1) = '1' then s_kx <= s_kx + 1;   -- leave a disc
                    else                 s_kx <= s_kx - 1;   -- enter a disc
                    end if;
                end if;
            end if;

            -- inside the nest <=> some disc covers <=> k < C_NR (the counter
            -- IS the exact hit test -- disc 23's comparators are gone)
            if s_kx /= s_nreff and s_dyn = '1' then
                g2_hit <= '1';
            else
                g2_hit <= '0';
            end if;
            g2_k   <= s_kx;
        end if;
    end process p_geo;

    ------------------------------------------------------------------------
    -- MERGE: inside disc C_NR-1 -> the nest's mark parity (exact circles);
    -- else the outer tracker's ring parity (ring C_NR is even, matching the
    -- modelled discs' 0..C_NR-1 indexing; the tracker counts absolute rings
    -- from the outer centre, so its k parity IS the disc parity out there).
    ------------------------------------------------------------------------
    p_r10 : process(clk)
    begin
        if rising_edge(clk) then
            if g2_hit = '1' then r10_k <= g2_k;
            else                 r10_k <= k_ld2; end if;
            r10_t <= ph1_f1;     -- per-ring ramp (already edge-anchored)
            -- S9 MOIRE: XOR with the HALF-PHASE copy of the rings around the
            -- outer centre (= outer parity xor (r mod s >= s/2) = round(r/s)
            -- mod 2).  That reference only changes at HALF radii -- its two
            -- terms flip together at every full radius -- so it never meets
            -- the true ring edges (no residue dots, no d^2-1 hack needed; the
            -- hack put a 1-px line on the centre row).  At rest the result is
            -- the doubled pattern; with the nest offset, true per-pixel moire.
            r10_h <= (h2 xor k_ld2(0)) and s_s9;
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- GRADIENT (Stepped): EVERY RING carries its own ramp-in/ramp-out --
    -- the per-ring phase triangle (dark at both edges, bright mid-ring,
    -- re-anchored at every true disc edge so the shading cascades with the
    -- rings) -- and K4 picks the FINENESS: zone 0 = the classic hard look,
    -- zones 1..6 = 2/4/8/16/32/64 gray steps within each ring.
    -- S9 Smooth = the SAME per-ring ramp at full 8-bit resolution.
    ------------------------------------------------------------------------
    p_r12 : process(clk)
        variable v_t8  : unsigned(7 downto 0);
        variable v_key : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- K4 (v1.9, 4 settings; S9 Smooth folded into the top one):
            --   0 = classic hard black/white, 1 = 4 steps (black / 2 grays /
            --   white -- v2.0.3, on hardware the 2-step setting looked like the hard
            --   one), 2 = 8 steps, 3 = smooth (the full 8-bit per-ring ramp)
            v_t8 := r10_t;
            case s_gn is
                when "00" =>    v_key := (others => not r10_k(0));
                when "01" =>    v_key := v_t8(7 downto 6) & v_t8(7 downto 6)
                                         & v_t8(7 downto 6) & v_t8(7 downto 6);
                when "10" =>    v_key := v_t8(7 downto 5) & v_t8(7 downto 5) & v_t8(7 downto 6);
                when others =>  v_key := v_t8;
            end case;
            -- S9 MOIRE: invert where the half-phase reference ring is set
            if r10_h = '1' then
                v_key := not v_key;
            end if;
            r12_key <= v_key;
        end if;
    end process p_r12;

    ------------------------------------------------------------------------
    -- key -> studio-range 10-bit luma (64..942); VIDEO: refraction
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_we : std_logic;
        variable v_wd : std_logic_vector(29 downto 0);
        variable v_du : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- REFRACTION: the ring field horizontally displaces WHERE the
            -- video is sampled -- each ring bends the picture like a glass
            -- ripple, and the cascade drags the ripples with it.  The input
            -- is written to a BRAM delay line on ACTIVE pixels only and
            -- read 1 + disp back, where
            --   disp = L*key/32, 0..53; L = P12 top 3 bits.
            -- key respects K4/S9: hard rings shear in slices, S9 Smooth gives
            -- true waves.  d = 0 (slider down) realigns exactly -- no shift.
            -- Writes continue 12 cycles past line end, repeating the
            -- held last pixel: without this the write pointer freezes at
            -- avid fall while the output tail still reads, so the last ~7
            -- columns repeated one pixel.  The tail slots also make the
            -- next line's left-edge wrap a clean clamp of this line's edge.
            -- ONE write statement only: a second s_vd assignment in an
            -- elsif branch stops yosys inferring the single write port and
            -- the whole line falls back to flip-flops (9205 LC = 120% chip).
            v_we := '0';
            v_wd := vd_hold;
            if data_in.avid = '1' then
                v_wd    := data_in.y & data_in.u & data_in.v;
                v_we    := '1';
                vd_hold <= data_in.y & data_in.u & data_in.v;
                vd_tc   <= to_unsigned(12, 4);
            elsif vd_tc /= 0 then
                v_we  := '1';
                vd_tc <= vd_tc - 1;
            end if;
            if v_we = '1' then
                s_vd(to_integer(vd_wa)) <= v_wd;
                vd_wa <= vd_wa + 1;
            end if;
            -- pixels available so far on THIS line (saturating)
            if data_in.avid = '0' then
                vd_px <= (others => '0');
            elsif vd_px /= 63 then
                vd_px <= vd_px + 1;
            end if;
            -- one-sided displacement from the field key (v1.8): du =
            -- L*key/32, L = P12 top 3 bits (8 levels) as three gated
            -- shifted terms -> 0..53 px (v1.7: 4 zones, 0..18)
            -- v2.1.5: depth s_dq is Q1 and mode-scaled (1080: P12 bits & 0)
            v_du := (others => '0');
            if s_dq(3) = '1' then v_du := v_du + resize(r12_key(7 downto 3), 6); end if;
            if s_dq(2) = '1' then v_du := v_du + resize(r12_key(7 downto 4), 6); end if;
            if s_dq(1) = '1' then v_du := v_du + resize(r12_key(7 downto 5), 6); end if;
            if s_dq(0) = '1' then v_du := v_du + resize(r12_key(7 downto 6), 6); end if;
            -- LEFT-EDGE CLEANUP: never read across the line start.  The
            -- displacement is clamped to the pixels this line has actually
            -- produced, so the wave fades in over the first columns
            -- instead of smearing the previous line's tail into the edge.
            if vd_px = 0 then
                v_du := (others => '0');
            elsif v_du > vd_px - 1 then
                v_du := vd_px - 1;
            end if;
            vd_ra <= vd_wa - 1 - v_du;
            vd_q  <= s_vd(to_integer(vd_ra));

            -- video split + hold stages (two plain registers -- the depth
            -- keeps the pipeline at C_LATENCY so both modes tap together)
            r13_y <= unsigned(vd_q(29 downto 20));
            r13_u <= unsigned(vd_q(19 downto 10));
            r13_v <= unsigned(vd_q( 9 downto  0));
            r14_y <= r13_y;
            r14_u <= r13_u;
            r14_v <= r13_v;

            -- output stage.
            -- BLANKING GATE: outside active video emit exactly what ring mode
            -- emits -- neutral chroma, black luma.  Without this the free-
            -- running pipeline puts PROCESSED GARBAGE in the blanking
            -- interval (input blanking chroma 0 scaled by 0.75 = 128, an
            -- illegal non-neutral level sitting where the encoder's per-line
            -- colour reference lives) and the WHOLE picture decodes with
            -- rail-to-rail wrong colours + noisy lines, while every active
            -- pixel is bit-correct.  s_avid_sr(C_LATENCY - 2) here aligns
            -- with the s_avid_sr(C_LATENCY - 1) output tap after this register.
            -- v1.7: the gate now covers BOTH modes -- ring mode used to
            -- emit the last pixel's key (often white) through every hblank.
            -- Ring luma is legal studio range: key 0..255 -> Y 64..942
            -- (64 + 4k - k/2 - k/16, monotonic); v1.6 drove 0..1023, and
            -- codes 0/1023 are the BT.656/1120 sync-word codes.
            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            elsif s_vsat = '1' then
                s_out_y <= r14_y;
                s_out_u <= r14_u;
                s_out_v <= r14_v;
            else
                v_y := shift_left(resize(r12_key, 10), 2)
                       - resize(r12_key(7 downto 1), 10)
                       - resize(r12_key(7 downto 4), 10)
                       + to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
                s_out_y <= v_y;
            end if;
        end if;
    end process p_out;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    s_io_align_in.y       <= std_logic_vector(s_out_y);
    s_io_align_in.u       <= std_logic_vector(s_out_u);
    s_io_align_in.v       <= std_logic_vector(s_out_v);
    -- ONE set of sync taps for both modes -- the video pipeline is depth-
    -- matched to the ring path, so the mode switch changes DATA only.
    s_io_align_in.avid    <= s_avid_sr(C_LATENCY - 1);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_LATENCY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (6 -> 8).
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

end architecture cascade;
