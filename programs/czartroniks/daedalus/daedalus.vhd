-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- daedalus.vhd  (v1.0 -- the picture drawn as a labyrinth)
--
-- v1.0.3 (2026-10-17): resolution scaling.  The lattice lives on a virtual
-- 1080-line canvas: the per-pixel DDA steps are scaled by the mode's x step
-- (1.0 / 1.5 at 720p / 2.0 at SD, a 1440-wide 4:3 window) and the per-FIELD-
-- line steps by its y step (1080/active lines, doubled when interlaced; done
-- as ys repeated additions in vblank), both from the timing ID in
-- registers_in(8).  Cells, line weight and the octave threshold were already
-- in per-1080p-pixel units of `step`, so they now scale with screen height;
-- Angle stays a rotation on screen; the interlaced 2x vertical stretch is
-- gone.  Strokes of octaves 0 and 1 keep a half-width of at least half a
-- step of the mode (0 at 1080p) so SD/720p can't drop them between lines.
-- 1080p is bit-identical.
--
-- Forked from VIDEOSKY, which cut the incoming video into a copper line-screen:
-- an accumulator pair P/Q gives a rotatable, scalable coordinate frame, and the
-- rules are the LEVEL SETS of P (`phase < thickness`).  Level sets can never
-- break, which is why VideoSky's Relief can bow them around the image.
--
-- Daedalus keeps the frame and throws away the level set.  A maze is not a
-- level set, it is a TILING -- so we use the part of the accumulator VideoSky
-- discarded:
--
--   * THE LATTICE IS ALREADY THERE.  acc is 24 bits with 8 fractional bits, so
--     acc(15 downto 8) is the phase within one period -- the SUB-CELL
--     coordinate -- and acc(23 downto 16) is the period INDEX -- the CELL id.
--     VideoSky only ever read the phase byte.  Taking both bytes turns the same
--     two adds per pixel into a full cell grid that rotates with Angle and
--     scales with Pitch for no extra cost.
--
--   * ONE CONTINUOUS LINE (the Truchet weave).  Two tiles, each joining all
--     four edge-midpoint ports IN PAIRS -- N<->W + S<->E, or its mirror
--     N<->E + S<->W.  Because every tile uses every port, each port is always
--     live and always has exactly ONE path arriving from each side: degree 2
--     everywhere, no branching possible, so the ink is a set of unbroken lines
--     that snake around each other at right angles and fill the plane with no
--     gaps and no sparse patches.  (v0.1 hashed each cell EDGE for liveness and
--     drew arms to the live ports.  It connected fine, but independent ports
--     mean degree 0..4 -- T-junctions, crosses, dead ends.  That is a network,
--     not a labyrinth.)
--
--     The paths are STAIRCASES, and they have to be.  With ports at the edge
--     midpoints, a single-turn L from N to W must turn either at the cell
--     CENTRE -- where it collides with the S<->E path -- or at the cell CORNER,
--     where the paths of four cells collide.  Physical Truchet tiles dodge this
--     with quarter-circle arcs that bulge toward the corner without touching
--     it; there is no right-angle equivalent of that arc.  So N<->W runs
--     (128,0)->(128,64)->(64,64)->(64,128)->(0,128), hugging the corner
--     diagonal but reaching neither corner nor centre, and S<->E is its 180
--     degree rotation.  Tile 1 is tile 0 MIRRORED IN u (u -> 256-u, exact about
--     the port at 128), so it costs one negate and a mux rather than a second
--     set of segment tests.
--
--   * UNDER AND OVER (v0.3).  A third tile joins the ports STRAIGHT THROUGH --
--     N<->S and W<->E -- one bar crossing the other, and the under bar BREAKS
--     either side of the over bar and comes back out the far side.  Which bar
--     is on top alternates checkerboard-fashion with cell parity, so a line
--     running through several crossings goes over, under, over -- a woven
--     basket.  The cross pairs all four ports like every other tile, so the
--     degree-2 / one-continuous-line guarantee survives.
--
--   * MEANDER CELLS (v0.4).  The cells are 4x WIDER than they are tall: Q (the
--     column axis) steps at a quarter of P's rate, so every tile stroke that
--     runs along the wide axis is four times longer in pixels than a vertical
--     jog -- long horizontal runs joined by short 90-degree drops, a greek-key
--     meander locked to a grid.  Vertical strokes take a QUARTER width
--     ((w+3)/4) to come out the same thickness in pixels, and Relief's
--     displacement gets the same /4 on the Q axis so the warp stays isotropic.
--
--   * LEGIBLE SCALE (v0.5).  Pitch now spans cells ~164 px .. ~13 px tall (it
--     used to reach 4 px, and the 9.5 px default put parallel runs 2.4 px
--     apart with sub-pixel lines that fell between scan lines).  Line weight is
--     held constant in PIXELS -- the sub-cell width is the pixel weight times
--     step, one small multiply -- so Pitch changes spacing, not thickness.
--     Each stroke is extended by the perpendicular stroke's half-width, so
--     every elbow has a square outer corner instead of a bite out of it.
--
--   * FRACTAL OCTAVES (v0.3, dialled back in v0.4/v0.5).  An octave of the
--     lattice is a BIT-SHIFT of the same warped coordinate: shift the 16-bit
--     integer part left by k and the top byte is the cell id of a lattice 2^k
--     times finer, the bottom byte its sub-cell coordinate, already rescaled.
--     Three nested labyrinths share one accumulator, one warp and one Relief;
--     each pays only its own hash, mirror and segment compares.  They are
--     OR-composited, so the finer weave fills the GAPS between the big lines.
--     Octave k fades in (a per-frame width-cap ramp, no pop) only while its own
--     cells are >= 32 px, so the default shows ONE labyrinth and finer ones
--     appear only as Pitch opens out.  Line weight halves per octave in pixels.
--
--   * RELIEF WARPS THE DOMAIN, NOT THE PATTERN (P12, the headline).  The luma
--     is added to the FULL 16-bit integer part BEFORE it is split into cell id
--     and sub-cell, so the cell boundaries bow along with the strokes and
--     neighbours never disagree about where their shared edge is.  Where luma
--     is smooth the labyrinth drapes; where it FOLDS the coordinate you get a
--     mirrored crease (still continuous); where it is DISCONTINUOUS it tears.
--     Top of the slider folds everywhere at once -- the climax.  v0.5: the full
--     10-bit slider drives the warp (it was 6 bits -- visible steps).
--
-- Renderer: streaming, 12 stages, no line buffer.  Three pixel-rate multiplies
-- (contrast, relief, line weight).  The per-frame angle resolve shares ONE
-- registered 9x10 multiplier, split-operand across vblank cycles.  Rotation
-- and scale pivot on the SCREEN CENTRE: a vblank loop walks the origin back
-- from the centre by the measured half-width/half-height, one DDA step per
-- clock, so no wide multiply is needed.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture daedalus of program_top is
    -- data_in -> data_out latency is 12 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;

    constant C_LATENCY : integer := 12;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Solid ink above this tone.  The blackwork masses (S7); K1 Ink slides the
    -- whole ramp into and out of it, so it needs no knob of its own.
    constant C_BLACKOUT : unsigned(7 downto 0) := to_unsigned(224, 8);

    -- Parallel staircase segments sit 64 apart in sub-cell units, so the arm
    -- half-width has to stay under 32 or neighbouring runs merge into mush.
    constant C_WMAX : integer := 26;

    -- Nested lattice octaves.  Octave k is 2^k times finer than the base.
    constant C_NOCT : integer := 3;

    -- Octave k is gone while step*2^k >= 2048 (its cells < 32 px) and fully
    -- present below 2048 - 26*32 (cells >= ~54 px): the width cap ramps
    -- (2048 - step<<k)/32, clamped to C_WMAX.
    constant C_OCT_LIM : integer := 2048;

    -- One seed per octave, so the nested tile fields are uncorrelated.
    type t_seeds is array(0 to C_NOCT - 1) of unsigned(15 downto 0);
    constant C_SEEDS : t_seeds := (x"9E37", x"7C15", x"3A9D");

    ----------------------------------------------------------------------
    -- quarter-wave sine, 256 entries: round(511*sin(i*pi/512)).  One entry
    -- per knob LSB (0.35 deg) -- the old 64-entry table stepped 1.4 deg.
    -- Used ONLY by the vblank angle resolve.
    ----------------------------------------------------------------------
    type t_qsin is array(0 to 255) of integer range 0 to 511;
    constant C_QSIN : t_qsin := (
          0,   3,   6,   9,  13,  16,  19,  22,  25,  28,  31,  34,  38,  41,  44,  47,
         50,  53,  56,  59,  63,  66,  69,  72,  75,  78,  81,  84,  87,  90,  94,  97,
        100, 103, 106, 109, 112, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 145,
        148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 187, 190, 193,
        196, 198, 201, 204, 207, 210, 213, 216, 218, 221, 224, 227, 230, 233, 235, 238,
        241, 244, 246, 249, 252, 255, 257, 260, 263, 265, 268, 271, 273, 276, 279, 281,
        284, 286, 289, 292, 294, 297, 299, 302, 304, 307, 309, 312, 314, 317, 319, 322,
        324, 327, 329, 331, 334, 336, 338, 341, 343, 345, 348, 350, 352, 355, 357, 359,
        361, 364, 366, 368, 370, 372, 374, 377, 379, 381, 383, 385, 387, 389, 391, 393,
        395, 397, 399, 401, 403, 405, 407, 409, 410, 412, 414, 416, 418, 420, 421, 423,
        425, 427, 428, 430, 432, 433, 435, 437, 438, 440, 441, 443, 445, 446, 448, 449,
        451, 452, 454, 455, 456, 458, 459, 461, 462, 463, 465, 466, 467, 468, 470, 471,
        472, 473, 474, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488,
        489, 490, 491, 492, 492, 493, 494, 495, 496, 496, 497, 498, 499, 499, 500, 501,
        501, 502, 502, 503, 503, 504, 505, 505, 505, 506, 506, 507, 507, 508, 508, 508,
        509, 509, 509, 509, 510, 510, 510, 510, 510, 511, 511, 511, 511, 511, 511, 511);

    ----------------------------------------------------------------------
    -- Cell hash -> the tile bits.  Three rounds over three pipeline stages so
    -- no stage carries more than a couple of adds.  Every step is a bijection
    -- on 16 bits (odd multiplies, and xor-with-own-high-bits), so distinct
    -- cells never collide.  Shift-and-ADD, not shift-and-xor: xor alone is
    -- linear over GF(2); the carries are the nonlinearity.
    --
    -- v0.5 adds f_mix_c.  Without it the top bits (the mirror field) barely
    -- saw the cell's low byte: the mirror bit was 0.67 correlated between
    -- horizontal neighbours and had a 43x FFT peak -- visible periodic
    -- structure.  With it every neighbour correlation is < 0.01 and the
    -- spectrum is flat (checked numerically for all three seeds).
    ----------------------------------------------------------------------
    function f_mix_a(c1, c2 : unsigned(7 downto 0); seed : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := (c1 & c2) xor seed;
        v := v + shift_left(v, 3);          -- *9
        v := v xor shift_right(v, 7);
        return v;
    end function;

    function f_mix_b(a : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := a + shift_left(a, 5);          -- *33
        v := v xor shift_right(v, 9);
        v := v + shift_left(v, 2);          -- *5
        return v;
    end function;

    function f_mix_c(a : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := a xor shift_right(a, 7);
        v := v + shift_left(v, 4) + shift_left(v, 9);   -- *529
        return v;
    end function;

    -- Clamp into the legal 10-bit luma swing.  Returns UNSIGNED on purpose:
    -- unsigned(resize(v,10)) of a signed value saturates at the SIGNED range,
    -- so anything >= 512 wraps to black (the penrose lesson).
    function f_cy(v : signed) return unsigned is
    begin
        if    v < 16   then return to_unsigned(16, 10);
        elsif v > 1008 then return to_unsigned(1008, 10);
        else                return resize(unsigned(v(9 downto 0)), 10);
        end if;
    end function;

    -- Clamp a tone into 0..255.
    function f_ct(v : signed) return unsigned is
    begin
        if    v < 0   then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else               return resize(unsigned(v(7 downto 0)), 8);
        end if;
    end function;

    -- Clamp an arm half-width into 0..C_WMAX.
    function f_cw(v : signed) return unsigned is
    begin
        if    v < 0      then return to_unsigned(0, 5);
        elsif v > C_WMAX then return to_unsigned(C_WMAX, 5);
        else                  return resize(unsigned(v(4 downto 0)), 5);
        end if;
    end function;

    -- |a - b| on 8-bit sub-cell coordinates.
    function f_ad(a : unsigned(7 downto 0); b : integer) return unsigned is
        variable v_b : unsigned(7 downto 0);
    begin
        v_b := to_unsigned(b, 8);
        if a >= v_b then return a - v_b; else return v_b - a; end if;
    end function;

    -- Saturating a - b and a + b on 8-bit sub-cell coordinates (b <= 26).
    function f_ssub(a : unsigned(7 downto 0); b : unsigned(4 downto 0)) return unsigned is
    begin
        if a >= resize(b, 8) then return a - resize(b, 8); else return to_unsigned(0, 8); end if;
    end function;

    function f_sadd(a : unsigned(7 downto 0); b : unsigned(4 downto 0)) return unsigned is
        variable v : unsigned(8 downto 0);
    begin
        v := resize(a, 9) + resize(b, 9);
        if v(8) = '1' then return to_unsigned(255, 8); else return v(7 downto 0); end if;
    end function;

    -- Knob deadband for the geometry controls: Pitch and Angle move the far
    -- side of the frame by many pixels per LSB, so ADC noise would shimmer the
    -- whole lattice.  Follow the knob only once it has moved 3 LSB; snap to
    -- the ends so Angle 0 is exactly upright.
    function f_db(raw, held : unsigned(9 downto 0)) return unsigned is
        variable v_d : signed(10 downto 0);
    begin
        if raw < 3 then
            return to_unsigned(0, 10);
        elsif raw > 1020 then
            return to_unsigned(1023, 10);
        end if;
        v_d := signed('0' & raw) - signed('0' & held);
        if v_d > 2 or v_d < -2 then
            return raw;
        else
            return held;
        end if;
    end function;

    ----------------------------------------------------------------------
    -- Resolution scaling (v1.0.3): per-mode steps from the video timing ID.
    -- The lattice is laid on a virtual 1080-line canvas: the DDA steps are
    -- scaled per pixel by xs and per FIELD line by ys, so a cell is the same
    -- fraction of the screen height in every mode, stays square on a 4:3 SD
    -- screen, and Angle stays a rotation on screen.  1080p: both 1.0, so the
    -- steps (and the picture) are exactly the old ones.
    ----------------------------------------------------------------------
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
    -- Thinnest stroke that a mode can still sample: half-width of at least
    -- half a step on the coarser axis, in quarter virtual pixels.  0 at
    -- 1080p (no clamp -- the picture is unchanged there).
    function f_wmin(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(9, 4);   -- 480i: 4.44/2
            when 4          => return to_unsigned(5, 4);   -- 480p: 2.25/2
            when 8          => return to_unsigned(8, 4);   -- 576i: 3.75/2
            when 12         => return to_unsigned(4, 4);   -- 576p: x 2.0/2
            when 5 | 6 | 14 => return to_unsigned(3, 4);   -- 720p: 1.5/2
            when 1 | 2 | 10 => return to_unsigned(4, 4);   -- 1080i: 2.0/2
            when others     => return to_unsigned(0, 4);   -- 1080p
        end case;
    end function;
    -- v * xs/2 for xs = 2/3/4 (1.0/1.5/2.0).
    function f_xsc(v : signed(17 downto 0); xs : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(xs) is
            when 3      => return v + shift_right(v, 1);
            when 4      => return shift_left(v, 1);
            when others => return v;
        end case;
    end function;

    ----------------------------------------------------------------------
    -- per-octave signal bundles
    ----------------------------------------------------------------------
    type t_u8a  is array(0 to C_NOCT - 1) of unsigned(7 downto 0);
    type t_u16a is array(0 to C_NOCT - 1) of unsigned(15 downto 0);
    type t_u5a  is array(0 to C_NOCT - 1) of unsigned(4 downto 0);

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k1_ink   : unsigned(9 downto 0) := to_unsigned(512, 10);   -- K1 Ink
    signal s_k2_con   : unsigned(9 downto 0) := to_unsigned(256, 10);   -- K2 Contrast
    signal s_k3_raw   : unsigned(9 downto 0) := to_unsigned(400, 10);   -- K3 Pitch (raw)
    signal s_k4_raw   : unsigned(9 downto 0) := (others => '0');        -- K4 Angle (raw)
    signal s_k3_pit   : unsigned(9 downto 0) := to_unsigned(400, 10);   -- K3, deadbanded
    signal s_k4_ang   : unsigned(9 downto 0) := (others => '0');        -- K4, deadbanded
    signal s_k5_tan   : unsigned(9 downto 0) := to_unsigned(1023, 10);  -- K5 Tangle
    signal s_scheme   : unsigned(9 downto 0) := (others => '0');        -- K6 Scheme
    -- K6 with labelled-default remap, free-running (kept out of the vsync
    -- cone).  At program load the firmware writes the initial_value_label's
    -- INDEX (0..5) into the register, not a 0..1023 position, so a value
    -- below 6 is replaced by the centre of that label's zone.
    signal s_k6_fix   : unsigned(9 downto 0) := (others => '0');
    signal s_p12      : unsigned(9 downto 0) := (others => '0');        -- P12 Relief
    signal s_black    : std_logic := '0';                               -- S7  Blackout
    signal s_invert   : std_logic := '0';                               -- S8  Invert
    signal s_tint     : std_logic := '0';                               -- S9  Tint
    signal s_wmod     : std_logic := '0';                               -- S10 Width
    signal s_drift    : std_logic := '0';                               -- S11 Drift

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_cos, s_sin : signed(9 downto 0) := (others => '0');
    signal s_step   : unsigned(12 downto 0) := to_unsigned(1200, 13);
    signal s_pex    : unsigned(11 downto 0) := (others => '0');
    signal s_dpx, s_dpy : signed(17 downto 0) := (others => '0');
    signal s_dqx, s_dqy : signed(17 downto 0) := (others => '0');
    signal s_bias   : signed(9 downto 0)  := (others => '0');   -- Ink, +/-256
    signal s_gain   : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_tang6  : unsigned(5 downto 0) := to_unsigned(31, 6);
    signal s_crs6   : unsigned(5 downto 0) := to_unsigned(15, 6);
    signal s_fixedw : unsigned(4 downto 0) := to_unsigned(9, 5);
    signal s_wscale : unsigned(7 downto 0) := to_unsigned(37, 8);

    -- Per-octave width caps: the fade-in ramp.  Octave 0 is always fully
    -- present; only 1..C_NOCT-1 are written by the sequencer.
    signal s_wcap : t_u5a := (others => to_unsigned(C_WMAX, 5));

    signal s_ink_y, s_ink_u, s_ink_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pap_y, s_pap_u, s_pap_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- vblank sequencer: one shared, fully registered 9x10 multiplier, and a
    -- 4-deep registered sine port (address, fold, table, sign).
    ----------------------------------------------------------------------
    signal s_seq   : unsigned(5 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_i  : unsigned(7 downto 0) := (others => '0');
    signal s_qs_pk1, s_qs_ng1 : std_logic := '0';
    signal s_qs_t  : integer range 0 to 511 := 0;
    signal s_qs_pk2, s_qs_ng2 : std_logic := '0';
    signal s_qs_r  : signed(9 downto 0)  := (others => '0');
    signal s_ma    : signed(8 downto 0) := (others => '0');
    signal s_mb    : signed(9 downto 0) := (others => '0');
    signal s_mp    : signed(18 downto 0) := (others => '0');
    signal s_pah_c, s_pal_c : signed(18 downto 0) := (others => '0');
    signal s_pah_s, s_pal_s : signed(18 downto 0) := (others => '0');
    signal s_prod_c, s_prod_s : signed(22 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_vs_pulse : std_logic := '0';
    signal s_saw_act  : std_logic := '0';
    signal s_fbot  : std_logic := '0';
    signal s_dracc : unsigned(15 downto 0) := (others => '0');

    -- Resolution scaling: per-mode steps (free-running decode of the timing
    -- ID), the per-line scale done as ys repeated additions in vblank, and
    -- the thin-stroke floor (octave-0 sub-cell units, then per octave).
    signal s_xs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_wminq : unsigned(3 downto 0)  := (others => '0');
    signal s_ycnt  : unsigned(11 downto 0) := (others => '0');
    signal s_yap, s_yaq : signed(23 downto 0) := (others => '0');
    signal s_wmin  : unsigned(7 downto 0) := (others => '0');
    signal s_wlo   : t_u5a := (others => (others => '0'));

    -- screen measurement, for the centre pivot
    signal s_xcnt, s_wmeas : unsigned(10 downto 0) := (others => '0');
    signal s_lcnt, s_hmeas : unsigned(10 downto 0) := (others => '0');
    signal s_pcx, s_pcy    : unsigned(9 downto 0) := (others => '0');
    signal s_org_p, s_org_q   : signed(23 downto 0) := (others => '0');
    signal s_lin0p, s_lin0q   : signed(23 downto 0) := (others => '0');
    signal s_load  : std_logic := '0';

    ----------------------------------------------------------------------
    -- screen-coordinate accumulators.  24 bits with 8 fractional bits.  The
    -- integer part acc(23 downto 8) splits into CELL = acc(23 downto 16) and
    -- SUB-CELL = acc(15 downto 8).  A 24-bit wrap is a multiple of 2^16, so
    -- both bytes are continuous across the wrap and the width is free.
    ----------------------------------------------------------------------
    signal s_accp, s_accq : signed(23 downto 0) := (others => '0');
    signal s_linp, s_linq : signed(23 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal r1_ydiff : signed(10 downto 0) := (others => '0');
    signal r1_gain  : signed(7 downto 0)  := (others => '0');
    signal r1_pi, r1_qi : unsigned(15 downto 0) := (others => '0');

    signal r2_yp    : signed(18 downto 0) := (others => '0');
    signal r2_pi, r2_qi : unsigned(15 downto 0) := (others => '0');

    signal r3_y8    : unsigned(7 downto 0) := (others => '0');
    signal r3_pi, r3_qi : unsigned(15 downto 0) := (others => '0');

    signal r4_rel   : unsigned(9 downto 0) := (others => '0');
    signal r4_t8    : signed(9 downto 0)  := (others => '0');
    signal r4_pi, r4_qi : unsigned(15 downto 0) := (others => '0');

    signal r5_cx, r5_cy : t_u8a := (others => (others => '0'));
    signal r5_u,  r5_v  : t_u8a := (others => (others => '0'));
    signal r5_t         : unsigned(7 downto 0) := (others => '0');

    signal r6_h       : t_u16a := (others => (others => '0'));
    signal r6_u, r6_v : t_u8a := (others => (others => '0'));
    signal r6_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r6_t       : unsigned(7 downto 0) := (others => '0');

    signal r7_h       : t_u16a := (others => (others => '0'));
    signal r7_u, r7_v : t_u8a := (others => (others => '0'));
    signal r7_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r7_wb      : unsigned(4 downto 0) := (others => '0');
    signal r7_solid   : std_logic := '0';

    signal r8_h       : t_u16a := (others => (others => '0'));
    signal r8_u, r8_v : t_u8a := (others => (others => '0'));
    signal r8_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r8_wu      : unsigned(7 downto 0) := (others => '0');
    signal r8_solid   : std_logic := '0';

    signal r9_tile    : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r9_cross   : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r9_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r9_u, r9_v : t_u8a := (others => (others => '0'));
    signal r9_w, r9_wv : t_u5a := (others => (others => '0'));
    signal r9_solid   : std_logic := '0';

    signal r10_a64, r10_a128, r10_a192 : t_u8a := (others => (others => '0'));
    signal r10_b64, r10_b128, r10_b192 : t_u8a := (others => (others => '0'));
    signal r10_um, r10_up, r10_vm, r10_vp : t_u8a := (others => (others => '0'));
    signal r10_w, r10_wv : t_u5a := (others => (others => '0'));
    signal r10_gap, r10_gapv : t_u8a := (others => (others => '0'));
    signal r10_cross   : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r10_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r10_solid   : std_logic := '0';

    signal r11_ink : std_logic := '0';

    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u, s_out_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- chroma delay to the colour resolve
    type t_c is array(0 to C_LATENCY - 2) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_c := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared sine port: the sequencer presents the cos angle in state 62 and
    -- the sin angle in state 61, and reads them back in states 58 and 57.
    -- Four registers deep: address, fold, table (EBR-able), sign.
    ------------------------------------------------------------------------
    s_qs_a <= s_k4_ang + to_unsigned(256, 10) when s_seq = 62 else s_k4_ang;

    p_qsin : process(clk)
    begin
        if rising_edge(clk) then
            s_qs_ar <= s_qs_a;

            -- second and fourth quadrants read the table backwards; index 256
            -- (the peak) is past the table, so it is flagged instead
            if s_qs_ar(8) = '0' then
                s_qs_i <= s_qs_ar(7 downto 0);
            else
                s_qs_i <= to_unsigned(0, 8) - s_qs_ar(7 downto 0);
            end if;
            if s_qs_ar(8) = '1' and s_qs_ar(7 downto 0) = 0 then
                s_qs_pk1 <= '1';
            else
                s_qs_pk1 <= '0';
            end if;
            s_qs_ng1 <= s_qs_ar(9);

            s_qs_t   <= C_QSIN(to_integer(s_qs_i));
            s_qs_pk2 <= s_qs_pk1;
            s_qs_ng2 <= s_qs_ng1;

            if s_qs_pk2 = '1' then
                if s_qs_ng2 = '1' then
                    s_qs_r <= to_signed(-511, 10);
                else
                    s_qs_r <= to_signed(511, 10);
                end if;
            elsif s_qs_ng2 = '1' then
                s_qs_r <= -to_signed(s_qs_t, 10);
            else
                s_qs_r <= to_signed(s_qs_t, 10);
            end if;
        end if;
    end process p_qsin;

    ------------------------------------------------------------------------
    -- Screen measurement.  Width = active clocks on a line; height = active
    -- lines, captured in the counter's increment branch (never at vsync --
    -- the count is zeroed by then).
    ------------------------------------------------------------------------
    p_meas : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            else
                s_xcnt <= (others => '0');
            end if;
            if data_in.avid = '0' and s_avid_p = '1' then
                s_wmeas <= s_xcnt;
            end if;

            if s_vs_pulse = '1' then
                s_lcnt <= (others => '0');
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_lcnt  <= s_lcnt + 1;
                s_hmeas <= s_lcnt + 1;
            end if;
        end if;
    end process p_meas;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_t  : signed(11 downto 0);
        variable v_sk : unsigned(17 downto 0);
        variable v_c  : unsigned(11 downto 0);
        variable v_r13 : unsigned(12 downto 0);
        variable v_wl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_mp    <= s_ma * s_mb;

            s_vs_pulse <= '0';
            s_load     <= '0';

            if data_in.avid = '1' then
                s_saw_act <= '1';
            end if;

            -- Resolution scaling: the timing ID is static, so decode it
            -- free-running (kept out of the vsync cone); it is only read in
            -- vblank.
            s_xs    <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys    <= f_vstep_y(registers_in(8)(3 downto 0));
            s_wminq <= f_wmin(registers_in(8)(3 downto 0));

            -- dpy*ys and dpx*ys by repeated addition, ys clocks (<= 1138)
            -- from state 46; state 38 waits for it.
            if s_ycnt /= 0 then
                s_yap  <= s_yap + resize(s_dpy, 24);
                s_yaq  <= s_yaq + resize(s_dpx, 24);
                s_ycnt <= s_ycnt - 1;
            end if;

            case to_integer(unsigned(registers_in(5))) is
                when 0      => s_k6_fix <= to_unsigned(51, 10);    -- Parchment
                when 1      => s_k6_fix <= to_unsigned(204, 10);   -- Gallery
                when 2      => s_k6_fix <= to_unsigned(409, 10);   -- Night
                when 3      => s_k6_fix <= to_unsigned(613, 10);   -- Blueprint
                when 4      => s_k6_fix <= to_unsigned(818, 10);   -- Cyanotype
                when 5      => s_k6_fix <= to_unsigned(971, 10);   -- Copper
                when others => s_k6_fix <= unsigned(registers_in(5));
            end case;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_vs_pulse <= '1';

                -- Latching and restarting the sequencer are idempotent, so
                -- every serrated vsync edge may do them.
                s_k1_ink <= unsigned(registers_in(0));
                s_k2_con <= unsigned(registers_in(1));
                s_k3_raw <= unsigned(registers_in(2));
                s_k4_raw <= unsigned(registers_in(3));
                s_k5_tan <= unsigned(registers_in(4));
                s_scheme <= s_k6_fix;
                s_p12    <= unsigned(registers_in(7));
                s_black  <= registers_in(6)(0);      -- S7
                s_invert <= registers_in(6)(1);      -- S8
                s_tint   <= registers_in(6)(2);      -- S9
                s_wmod   <= registers_in(6)(3);      -- S10
                s_drift  <= registers_in(6)(4);      -- S11

                -- Toggles and increments are NOT idempotent: analog vsync
                -- serrates (several edges per field), so act only on the
                -- first edge after active video.
                if s_saw_act = '1' then
                    s_saw_act <= '0';

                    -- Bottom field (field_n '0') starts half a field step
                    -- down.  Inert on hardware (core_top ties field_n high).
                    s_fbot <= not data_in.field_n;

                    -- 16 bits so the wrap lands on the accumulator's own
                    -- 2^24 wrap -- a narrower counter jumps the cell ids and
                    -- re-randomises the whole maze when it wraps.
                    if registers_in(6)(4) = '1' then
                        s_dracc <= s_dracc + 2;
                    end if;
                end if;

                s_seq <= to_unsigned(63, 6);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    when 63 =>
                        s_k3_pit <= f_db(s_k3_raw, s_k3_pit);
                        s_k4_ang <= f_db(s_k4_raw, s_k4_ang);

                    -- 62/61 present the cos and sin angles into the sine port.
                    -- Pitch is linear in FREQUENCY, with a steeper top half:
                    -- 400 + 2*K3 (cell ~164 px .. ~48 px at K3=512), then
                    -- +5 per LSB above 512 -- 5001 at the top, cell ~13 px.
                    -- One cell = 65536 phase units.
                    when 62 =>
                        if s_k3_pit(9) = '1' then
                            s_pex <= shift_left(resize(s_k3_pit(8 downto 0), 12), 2)
                                     + resize(s_k3_pit(8 downto 0), 12);
                        else
                            s_pex <= (others => '0');
                        end if;

                    when 61 =>
                        s_step <= to_unsigned(400, 13)
                                  + shift_left(resize(s_k3_pit, 13), 1)
                                  + resize(s_pex, 13);

                    when 58 => s_cos <= s_qs_r;
                    when 57 => s_sin <= s_qs_r;

                    -- step*cos and step*sin, split-operand through the one
                    -- registered multiplier: step = sh*256 + sl.
                    when 56 => s_ma <= signed(resize(s_step(12 downto 8), 9));
                               s_mb <= s_cos;
                    when 55 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_cos;
                    when 54 => s_pah_c <= s_mp;
                    when 53 => s_pal_c <= s_mp;
                    when 52 => s_ma <= signed(resize(s_step(12 downto 8), 9));
                               s_mb <= s_sin;
                    when 51 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_sin;
                    when 50 => s_pah_s <= s_mp;
                    when 49 => s_pal_s <= s_mp;

                    when 48 =>
                        s_prod_c <= shift_left(resize(s_pah_c, 23), 8)
                                    + resize(s_pal_c, 23);
                        s_prod_s <= shift_left(resize(s_pah_s, 23), 8)
                                    + resize(s_pal_s, 23);

                    -- Angle 0 must give an upright lattice, so cos drives the
                    -- per-LINE step and sin the per-pixel one.  sin/cos are
                    -- +/-511, so >>9 leaves step*cos.
                    when 47 =>
                        s_dpy <= resize(shift_right(s_prod_c, 9), 18);
                        s_dpx <= resize(shift_right(s_prod_s, 9), 18);

                    -- Resolution scaling: start dpy*ys and dpx*ys (the
                    -- per-field-line steps), finished by state 38.
                    when 46 =>
                        s_ycnt <= s_ys;
                        s_yap  <= (others => '0');
                        s_yaq  <= (others => '0');

                    when 45 =>      -- Ink: a bias on the whole tone ramp
                        s_bias <= signed(resize(shift_right(s_k1_ink, 1), 10))
                                  - to_signed(256, 10);
                        -- thin-stroke floor in sub-cell units: wmin_q*step/1024
                        -- through the shared multiplier, read in state 43
                        s_ma <= signed(resize(s_wminq, 9));
                        s_mb <= signed(resize(s_step(12 downto 5), 10));

                    when 44 =>      -- Contrast: 1.0x .. 4.9x about mid-grey
                        s_gain <= to_unsigned(16, 7) + resize(s_k2_con(9 downto 4), 7);

                    -- Tangle: the tile-mirror probability, 0..31 out of 64.
                    -- Past 1/2 the pattern is just the mirror of below it, so
                    -- the knob stops there.  Crossings at HALF the mirror rate
                    -- (max ~24% of cells).
                    when 43 =>
                        s_tang6 <= resize(s_k5_tan(9 downto 5), 6);
                        s_crs6  <= resize(s_k5_tan(9 downto 6), 6);
                        v_r13   := unsigned(s_mp(12 downto 0)) + 31;    -- ceil
                        s_wmin  <= v_r13(12 downto 5);

                    when 42 =>      -- line weight when Width is Fixed, 1/4 px
                        v_t := to_signed(9, 12) + resize(shift_right(s_bias, 5), 12);
                        s_fixedw <= f_cw(v_t);

                    -- Pixel weight -> sub-cell units: w_units = w_qpx*step/1024
                    -- = w_qpx * (step>>5) >> 5, one small per-pixel multiply.
                    when 41 =>
                        s_wscale <= s_step(12 downto 5);

                    when 40 =>      -- print scheme (K6)
                        if s_scheme < 102 then                  -- Parchment
                            s_pap_y <= to_unsigned(740, 10);
                            s_pap_u <= to_unsigned(470, 10); s_pap_v <= to_unsigned(540, 10);
                            s_ink_y <= to_unsigned(30, 10);
                            s_ink_u <= to_unsigned(495, 10); s_ink_v <= to_unsigned(522, 10);
                        elsif s_scheme < 307 then               -- Gallery
                            s_pap_y <= to_unsigned(830, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(40, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_scheme < 511 then               -- Night
                            s_pap_y <= to_unsigned(120, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(750, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_scheme < 716 then               -- Blueprint
                            s_pap_y <= to_unsigned(300, 10);
                            s_pap_u <= to_unsigned(700, 10); s_pap_v <= to_unsigned(450, 10);
                            s_ink_y <= to_unsigned(850, 10);
                            s_ink_u <= to_unsigned(500, 10); s_ink_v <= to_unsigned(506, 10);
                        elsif s_scheme < 920 then               -- Cyanotype
                            s_pap_y <= to_unsigned(200, 10);
                            s_pap_u <= to_unsigned(640, 10); s_pap_v <= to_unsigned(470, 10);
                            s_ink_y <= to_unsigned(880, 10);
                            s_ink_u <= to_unsigned(520, 10); s_ink_v <= to_unsigned(505, 10);
                        else                                    -- Copper
                            s_pap_y <= to_unsigned(800, 10);
                            s_pap_u <= to_unsigned(480, 10); s_pap_v <= to_unsigned(530, 10);
                            s_ink_y <= to_unsigned(300, 10);
                            s_ink_u <= to_unsigned(430, 10); s_ink_v <= to_unsigned(600, 10);
                        end if;

                    -- Octave fade ramps.  Shallow cones off the registered
                    -- s_step, but still STA-timed, so they get their own state.
                    when 39 =>
                        for k in 1 to C_NOCT - 1 loop
                            v_sk := shift_left(resize(s_step, 18), k);
                            if v_sk >= to_unsigned(C_OCT_LIM, 18) then
                                s_wcap(k) <= (others => '0');
                            else
                                v_c := resize(shift_right(to_unsigned(C_OCT_LIM, 18) - v_sk, 5), 12);
                                if v_c > C_WMAX then
                                    s_wcap(k) <= to_unsigned(C_WMAX, 5);
                                else
                                    s_wcap(k) <= resize(v_c, 5);
                                end if;
                            end if;
                        end loop;

                    -- RESOLUTION SCALING.  Wait for the ys loop, then lay the
                    -- steps on the virtual canvas: per pixel x xs, per field
                    -- line x ys.  Q is P rotated 90 degrees, at a QUARTER of
                    -- the rate: the cells come out 4x wider than tall, which
                    -- is what turns the staircases into long runs with short
                    -- jogs.  At 1080p this is exactly dqx = -dpy/4, dqy =
                    -- dpx/4 as before.  Also the stroke floor of octaves 0
                    -- and 1, held under the fade-in cap.  Octave 2's pen is
                    -- sub-pixel even at 1080p (it draws dashed there), so it
                    -- keeps no floor: flooring it made SD's coarse pitches
                    -- half again as dense as 1080p.
                    when 38 =>
                        if s_ycnt /= 0 then
                            s_seq <= s_seq;
                        else
                            s_dpx <= f_xsc(s_dpx, s_xs);
                            s_dqx <= -resize(shift_right(f_xsc(s_dpy, s_xs), 2), 18);
                            s_dpy <= resize(shift_right(s_yap, 8), 18);
                            s_dqy <= resize(shift_right(s_yaq, 10), 18);
                        end if;
                        for k in 0 to 1 loop
                            v_wl := shift_left(resize(s_wmin, 10), k);
                            if v_wl > resize(s_wcap(k), 10) then
                                s_wlo(k) <= s_wcap(k);
                            else
                                s_wlo(k) <= v_wl(4 downto 0);
                            end if;
                        end loop;

                    -- CENTRE PIVOT.  The screen centre sits at the drift
                    -- offset; walk the origin back from it by half a width of
                    -- dpx/dqx and half a height of dpy/dqy, one step per clock
                    -- (~1500 clocks of a ~99k-clock HD vblank).  Real pixels
                    -- and field lines times the virtual-canvas steps, so the
                    -- pivot is the screen centre in every mode.
                    when 37 =>
                        s_org_p <= shift_left(resize(signed('0' & s_dracc), 24), 8);
                        s_org_q <= (others => '0');
                        s_pcx   <= s_wmeas(10 downto 1);
                        s_pcy   <= s_hmeas(10 downto 1);

                    when 36 =>
                        if s_pcx /= 0 then
                            s_org_p <= s_org_p - resize(s_dpx, 24);
                            s_org_q <= s_org_q - resize(s_dqx, 24);
                            s_pcx   <= s_pcx - 1;
                            s_seq   <= s_seq;
                        end if;

                    when 35 =>
                        if s_pcy /= 0 then
                            s_org_p <= s_org_p - resize(s_dpy, 24);
                            s_org_q <= s_org_q - resize(s_dqy, 24);
                            s_pcy   <= s_pcy - 1;
                            s_seq   <= s_seq;
                        end if;

                    -- The bottom field starts half a field step down so the
                    -- two fields interleave (inert on hardware: field_n is
                    -- tied high there).
                    when 34 =>
                        if s_fbot = '1' then
                            s_lin0p <= s_org_p + shift_right(resize(s_dpy, 24), 1);
                            s_lin0q <= s_org_q + shift_right(resize(s_dqy, 24), 1);
                        else
                            s_lin0p <= s_org_p;
                            s_lin0q <= s_org_q;
                        end if;

                    when 33 => s_load <= '1';

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- Screen accumulators.  The sequencer hands over the field's origin in
    -- vblank.  Advance on ACTIVE lines only -- a blanking-hsync advance makes
    -- the phase field-dependent and the whole lattice twitters at 30 Hz.
    ------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_load = '1' then
                s_linp <= s_lin0p;
                s_linq <= s_lin0q;
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_accp <= s_linp;
                s_accq <= s_linq;
                s_linp <= s_linp + resize(s_dpy, 24);
                s_linq <= s_linq + resize(s_dqy, 24);
            elsif data_in.avid = '1' then
                s_accp <= s_accp + resize(s_dpx, 24);
                s_accq <= s_accq + resize(s_dqx, 24);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- R1: capture.  The contrast multiply's operands are registered here.
    -- The accumulators' INTEGER parts come across whole -- cell id and
    -- sub-cell both, since Relief has to displace them together.
    ------------------------------------------------------------------------
    p_r1 : process(clk)
    begin
        if rising_edge(clk) then
            r1_ydiff <= signed('0' & unsigned(data_in.y)) - to_signed(512, 11);
            r1_gain  <= signed('0' & s_gain);
            r1_pi    <= unsigned(s_accp(23 downto 8));
            r1_qi    <= unsigned(s_accq(23 downto 8));
            s_ud(0)  <= unsigned(data_in.u);
            s_vd(0)  <= unsigned(data_in.v);
            for i in 1 to C_LATENCY - 2 loop
                s_ud(i) <= s_ud(i - 1);
                s_vd(i) <= s_vd(i - 1);
            end loop;
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: contrast multiply.
    ------------------------------------------------------------------------
    p_r2 : process(clk)
    begin
        if rising_edge(clk) then
            r2_yp <= r1_ydiff * r1_gain;
            r2_pi <= r1_pi;
            r2_qi <= r1_qi;
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- R3: clamp the stretched luma back into the legal swing and take the
    -- top 8 bits as the working tone.
    ------------------------------------------------------------------------
    p_r3 : process(clk)
        variable v_y : signed(13 downto 0);
        variable v_c : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y := to_signed(512, 14) + resize(shift_right(r2_yp, 4), 14);
            v_c := f_cy(v_y);
            r3_y8 <= v_c(9 downto 2);
            r3_pi <= r2_pi;
            r3_qi <= r2_qi;
        end if;
    end process p_r3;

    ------------------------------------------------------------------------
    -- R4: the shaping multiply, at the slider's full 10 bits.
    --   rel = luma * Relief / 256 -- up to ~4 cells of displacement
    -- and the raw ink tone: 255 = black = full ink.
    ------------------------------------------------------------------------
    p_r4 : process(clk)
        variable v_m : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            v_m    := r3_y8 * s_p12;
            r4_rel <= v_m(17 downto 8);
            r4_t8  <= to_signed(255, 10) - signed(resize(r3_y8, 10));
            r4_pi  <= r3_pi;
            r4_qi  <= r3_qi;
        end if;
    end process p_r4;

    ------------------------------------------------------------------------
    -- R5: THE WARP, then the octave split.  rel lands on the full 16-bit
    -- integer coordinate BEFORE any splitting, so the cell boundaries of
    -- EVERY octave bow together.  Then octave k is a bit-shift of the warped
    -- coordinate: shift left by k and the top byte is the k-th octave's cell
    -- id, the bottom byte its sub-cell coordinate.  Both adds wrap mod 2^16,
    -- which is exactly right: the cell id is only ever used as hash input.
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable v_wp, v_wq : unsigned(15 downto 0);
        variable v_sp, v_sq : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- Q's displacement is /4 to match its /4 step: the warp stays
            -- isotropic in PIXELS even though the cells are anisotropic.
            v_wp := r4_pi + resize(r4_rel, 16);
            v_wq := r4_qi + resize(shift_right(r4_rel, 2), 16);
            for k in 0 to C_NOCT - 1 loop
                v_sp := shift_left(v_wp, k);
                v_sq := shift_left(v_wq, k);
                r5_cx(k) <= v_sp(15 downto 8);
                r5_u(k)  <= v_sp(7 downto 0);
                r5_cy(k) <= v_sq(15 downto 8);
                r5_v(k)  <= v_sq(7 downto 0);
            end loop;
            r5_t <= f_ct(r4_t8 + resize(s_bias, 10));
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- R6: hash round A on each octave's cell id, each with its own seed so
    -- the nested tile fields are uncorrelated.  The parity bit is the
    -- checkerboard that decides which bar of a crossing goes over.
    ------------------------------------------------------------------------
    p_r6 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                r6_h(k)   <= f_mix_a(r5_cx(k), r5_cy(k), C_SEEDS(k));
                r6_par(k) <= r5_cx(k)(0) xor r5_cy(k)(0);
                r6_u(k)   <= r5_u(k);
                r6_v(k)   <= r5_v(k);
            end loop;
            r6_t <= r5_t;
        end if;
    end process p_r6;

    ------------------------------------------------------------------------
    -- R7: hash round B; the base line weight in quarter-pixels (Fixed, or
    -- tone-modulated so the shadows fatten the line); Blackout.
    ------------------------------------------------------------------------
    p_r7 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                r7_h(k)   <= f_mix_b(r6_h(k));
                r7_par(k) <= r6_par(k);
                r7_u(k)   <= r6_u(k);
                r7_v(k)   <= r6_v(k);
            end loop;

            if s_wmod = '0' then
                r7_wb <= s_fixedw;
            else
                r7_wb <= r6_t(7 downto 3);
            end if;

            -- Blackout: the shadows go to solid ink, no maze.  This is what
            -- makes it read as blackwork rather than as a texture filter.
            if r6_t > C_BLACKOUT and s_black = '1' then
                r7_solid <= '1';
            else
                r7_solid <= '0';
            end if;
        end if;
    end process p_r7;

    ------------------------------------------------------------------------
    -- R8: hash round C, and the pixel weight converted to sub-cell units so
    -- the line stays the same thickness on screen at every Pitch.
    ------------------------------------------------------------------------
    p_r8 : process(clk)
        variable v_m : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                r8_h(k)   <= f_mix_c(r7_h(k));
                r8_par(k) <= r7_par(k);
                r8_u(k)   <= r7_u(k);
                r8_v(k)   <= r7_v(k);
            end loop;
            v_m      := r7_wb * s_wscale;
            r8_wu    <= v_m(12 downto 5);
            r8_solid <= r7_solid;
        end if;
    end process p_r8;

    ------------------------------------------------------------------------
    -- R9: each octave's tile bits, and its line weight.  Every octave takes
    -- the SAME sub-cell width, which is half as many pixels per octave -- the
    -- finer labyrinths draw with a finer pen.  The cap does double duty: it
    -- ramps a newly-enabled octave in smoothly, and it holds every octave
    -- under C_WMAX so parallel runs never merge.  A capped-out octave has
    -- width 0 and the segment tests below can never fire.
    ------------------------------------------------------------------------
    p_r9 : process(clk)
        variable v_w : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                if r8_h(k)(15 downto 10) < s_tang6 then
                    r9_tile(k) <= '1';
                else
                    r9_tile(k) <= '0';
                end if;

                -- The next bit-field of the same hash, at half the mirror
                -- rate: Tangle raises mirrors and crossings together.
                if r8_h(k)(9 downto 4) < s_crs6 then
                    r9_cross(k) <= '1';
                else
                    r9_cross(k) <= '0';
                end if;

                -- Resolution scaling: a stroke that exists is held at least
                -- as thick as the mode can sample (s_wlo is 0 at 1080p and
                -- for octave 2, and never above the cap).
                if r8_wu > resize(s_wcap(k), 8) then
                    v_w := s_wcap(k);
                elsif r8_wu < resize(s_wlo(k), 8) and r8_wu /= 0 then
                    v_w := s_wlo(k);
                else
                    v_w := resize(r8_wu, 5);
                end if;
                r9_w(k)  <= v_w;
                -- Vertical strokes take a quarter width (rounded up): their
                -- thickness is measured across the 4x-wide axis.
                r9_wv(k) <= resize(shift_right(resize(v_w, 7) + 3, 2), 5);

                r9_par(k) <= r8_par(k);
                r9_u(k)   <= r8_u(k);
                r9_v(k)   <= r8_v(k);
            end loop;
            r9_solid <= r8_solid;
        end if;
    end process p_r9;

    ------------------------------------------------------------------------
    -- R10: mirror and measure, per octave.  Tile 1 is tile 0 reflected in u
    -- (u -> 256-u, exact about the ports at 128), so ONE negate buys the
    -- second tile and the segment tests never need to know which tile they
    -- are drawing.  um/up/vm/vp are the coordinates pulled in by the
    -- perpendicular stroke's half-width: testing a segment's extent on them
    -- runs each stroke through to the outer edge of the stroke it turns into,
    -- so every elbow gets a square outer corner.
    ------------------------------------------------------------------------
    p_r10 : process(clk)
        variable v_uu : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                if r9_tile(k) = '1' then
                    v_uu := to_unsigned(0, 8) - r9_u(k);
                else
                    v_uu := r9_u(k);
                end if;

                r10_a64(k)  <= f_ad(v_uu,     64);
                r10_a128(k) <= f_ad(v_uu,    128);
                r10_a192(k) <= f_ad(v_uu,    192);
                r10_b64(k)  <= f_ad(r9_v(k),  64);
                r10_b128(k) <= f_ad(r9_v(k), 128);
                r10_b192(k) <= f_ad(r9_v(k), 192);
                r10_um(k)   <= f_ssub(v_uu, r9_w(k));
                r10_up(k)   <= f_sadd(v_uu, r9_w(k));
                r10_vm(k)   <= f_ssub(r9_v(k), r9_wv(k));
                r10_vp(k)   <= f_sadd(r9_v(k), r9_wv(k));
                r10_w(k)    <= r9_w(k);
                r10_wv(k)   <= r9_wv(k);

                -- The under bar stops this far from the over bar's centre
                -- line, per axis: one full width of the over bar clear each
                -- side, plus a hair.
                r10_gap(k)   <= shift_left(resize(r9_w(k), 8), 1) + 4;
                r10_gapv(k)  <= shift_left(resize(r9_wv(k), 8), 1) + 2;
                r10_cross(k) <= r9_cross(k);
                r10_par(k)   <= r9_par(k);
            end loop;
            r10_solid <= r9_solid;
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- R11: the rasteriser -- two staircases, eight axis-aligned segments per
    -- octave, pure compares.
    --
    --   N<->W : (128,0)->(128,64)->(64,64)->(64,128)->(0,128)
    --   S<->E : (128,255)->(128,192)->(192,192)->(192,128)->(255,128)
    --
    -- Each segment's extent runs to the far edge of the stroke it meets
    -- (um/up/vm/vp), open-ended at the ports.  The octaves OR together.
    ------------------------------------------------------------------------
    p_r11 : process(clk)
        variable v_ink : std_logic;
        variable v_w8  : unsigned(7 downto 0);
        variable v_v8  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_ink := '0';

            for k in 0 to C_NOCT - 1 loop
                -- a-tests bound strokes that run ALONG the wide axis (the
                -- long horizontal runs): full width.  b-tests bound the
                -- vertical jogs: quarter width, so the pixels match.
                v_w8 := resize(r10_w(k), 8);
                v_v8 := resize(r10_wv(k), 8);

                if r10_cross(k) = '1' then
                    -- Crossing: N<->S and W<->E straight through.  The over
                    -- bar is unbroken; the under bar keeps clear of it by
                    -- gap and comes back out the far side.  Parity picks
                    -- the over bar, checkerboard-fashion.
                    if r10_par(k) = '1' then
                        if r10_a128(k) < v_w8 then v_ink := '1'; end if;
                        if r10_b128(k) < v_v8 and r10_a128(k) >= r10_gap(k) then v_ink := '1'; end if;
                    else
                        if r10_b128(k) < v_v8 then v_ink := '1'; end if;
                        if r10_a128(k) < v_w8 and r10_b128(k) >= r10_gapv(k) then v_ink := '1'; end if;
                    end if;
                else
                    -- N <-> W
                    if r10_a128(k) < v_w8 and r10_vm(k) < 64 then v_ink := '1'; end if;
                    if r10_b64(k)  < v_v8 and r10_up(k) > 64 and r10_um(k) < 128 then v_ink := '1'; end if;
                    if r10_a64(k)  < v_w8 and r10_vp(k) > 64 and r10_vm(k) < 128 then v_ink := '1'; end if;
                    if r10_b128(k) < v_v8 and r10_um(k) < 64 then v_ink := '1'; end if;

                    -- S <-> E
                    if r10_a128(k) < v_w8 and r10_vp(k) > 192 then v_ink := '1'; end if;
                    if r10_b192(k) < v_v8 and r10_up(k) > 128 and r10_um(k) < 192 then v_ink := '1'; end if;
                    if r10_a192(k) < v_w8 and r10_vp(k) > 128 and r10_vm(k) < 192 then v_ink := '1'; end if;
                    if r10_b128(k) < v_v8 and r10_up(k) > 192 then v_ink := '1'; end if;
                end if;
            end loop;

            r11_ink <= v_ink or r10_solid;
        end if;
    end process p_r11;

    ------------------------------------------------------------------------
    -- R12: ink or paper, gated to neutral black outside active video (the
    -- encoders take their colour reference from blanking).  Tint hands the
    -- ink the source's own chroma.
    ------------------------------------------------------------------------
    p_r12 : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_CHROMA_MID;
                s_out_v <= C_CHROMA_MID;
            elsif (r11_ink xor s_invert) = '1' then
                s_out_y <= s_ink_y;
                if s_tint = '1' then
                    s_out_u <= s_ud(C_LATENCY - 2);
                    s_out_v <= s_vd(C_LATENCY - 2);
                else
                    s_out_u <= s_ink_u;
                    s_out_v <= s_ink_v;
                end if;
            else
                s_out_y <= s_pap_y;
                s_out_u <= s_pap_u;
                s_out_v <= s_pap_v;
            end if;
        end if;
    end process p_r12;

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

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture daedalus;
