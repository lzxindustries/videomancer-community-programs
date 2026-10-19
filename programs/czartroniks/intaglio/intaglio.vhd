-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- intaglio.vhd
--
-- v0.2.2 (2026-10-17): resolution scaling.  The stripe-field constants are
-- now built in virtual 1080-line units from the video timing ID
-- (registers_in(8)): the per-pixel (cos) taps use the pitch mantissa times
-- the x step (1 / 1.5 / 2 at 1080 / 720p / SD) and the per-line (sin) taps
-- the mantissa times the y step per FIELD line (1 / 1.5 / 4.44 at 1080p /
-- 720p / 480i, interlace included), both Q.2 through the existing vblank
-- shift-add.  The stipple lattice runs on virtual coordinates and the
-- direction hold run is 63 virtual units, so stroke pitch, dots and fills
-- keep their size relative to screen height (SD = 4:3 window).  For the
-- interlaced modes the gradient's y is halved before direction bucketing
-- (a field line is ~2x a pixel's virtual width), so strokes follow the form
-- at true angles.  The field_n parity-toggle line doubling is gone (field_n
-- is tied high on hardware, so SD strokes were drawn 2:1 tall).  Sobel,
-- pits, tremor and the warm-up guard stay at pixel scale.  1080p is
-- bit-identical; latency unchanged (16).
--
-- INTAGLIO -- the picture, engraved.
--
-- The frame is redrawn as a copperplate engraving: hatch strokes that
-- follow the image's form (stroke direction = the local luma-gradient
-- direction, bucketed into 4 coherent stroke families 45 degrees apart),
-- stroke THICKNESS carrying the tone, crosshatch engaging level by level
-- in the shadows, clean paper in the highlights, and dark contour ink on
-- edges. Everything is streaming and stateless -- no coarse field, no
-- frame memory, no warm-up: every control reacts on the next frame.
--
--   P12 "Bite" (headline): acid depth. 0 = contours only; 0-60% walks the
--        tonal ladder in (hatch / cross / third direction); 60-100% is the
--        over-bitten climax -- strokes swell until the shadows go solid,
--        crosshatch floods the midtones, foul-bite pits speckle the paper
--        and the contours thicken.
--
-- Controls:
--   K1 Pitch    stroke spacing, continuous ~6..89 px (exponential)
--   K2 Angle    plate rotation (continuous 0..180, per-frame qsin consts)
--   K3 Ink      stroke width gain + contour ink weight
--   K4 Palette  8 ink/paper pairs (copper, sepia, sanguine, prussian,
--               banknote, iron gall, gold-on-black, blueprint)
--   K5 Hand     hand tremor: per-line stroke-phase jitter
--   K6 Style    stipple  /  engraving  /  woodcut (parameter presets)
--   S7 Invert   scratchboard (ink and paper swap roles)
--   S8 Wash     half-saturation video colour under-print on the paper
--   S9 Contour  edge ink on/off
--   S10 Cross   crosshatch at 90 (classic) / 45 (freehand) degrees
--   S11 Drift   live plate: stroke phases crawl slowly
--
-- Architecture: single streaming pipeline S0..S12 (C_LATENCY = 13).
-- ZERO per-pixel multiplies, 4 EBR (two 2048x8 luma line buffers).
--
--   * 3x3 Sobel over [1 2 1]-h-blurred luma (prism/mercurial window, 2
--     line buffers). Direction is bucketed into 4 bins (0/45/90/135 deg)
--     with two shift-approximated tan(22.5)/tan(67.5) compares -- no
--     atan2. The gradient's y is measured UPWARD (row above minus row);
--     the stripe fields step DOWN per line, so the diagonal bins are
--     assigned in screen (y-down) space: signs differing = 45 deg.
--   * STRIPE-FIELD ACCUMULATORS (the trick that makes it cheap): the 4
--     stroke families are global stripe fields p_k = x*cos_k + y*sin_k,
--     maintained as running accumulators (+cos_k per pixel, +sin_k per
--     line) -- zero multiplies, and every pixel that picks family k sees
--     the same coherent field, so strokes are continuous across bucket
--     regions. Phase = p(15:8) for every pitch: the PITCH scale lives in
--     the constants (qsin * 2^-(n/32), one serial shift-add multiply per
--     tap in vblank), which makes K1 continuous and K2 rotation smooth.
--     Interlace is detected (field parity toggle) and the line step
--     doubled so stroke ANGLES are true in frame space.
--   * Tone -> ink: dark = 255 - luma; the tonal ladder t1/t2/t3 slides
--     down as Bite rises (signed: past 60% it goes negative = overbite);
--     stroke width = base + (dark - t)>>wsh, distance-to-stripe-center
--     compare with a half-ink antialias band. Contour ink from the Sobel
--     magnitude on top. Stipple swaps the stripe test for a hash-lattice
--     dot test (diamond sub-cell distance); woodcut is a pure per-frame
--     parameter preset (fat strokes).
--   * Generated colours (ink/paper palette) are stored U/V-SWAPPED -- the
--     hardware swaps U/V; the Wash video under-print is a dry path and
--     is not swapped.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture intaglio of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (13 + 3 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 13;

    function f_absu(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(-resize(v, v'length + 1));
        else
            return unsigned(resize(v, v'length + 1));
        end if;
    end function;

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks lineage)
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- pitch mantissa: round(64 * 2^(-f/32)), f = 0..31 (32 steps/octave)
    type t_mant is array(0 to 31) of unsigned(6 downto 0);
    constant C_MANT : t_mant := (
        to_unsigned(64, 7), to_unsigned(63, 7), to_unsigned(61, 7), to_unsigned(60, 7),
        to_unsigned(59, 7), to_unsigned(57, 7), to_unsigned(56, 7), to_unsigned(55, 7),
        to_unsigned(54, 7), to_unsigned(53, 7), to_unsigned(52, 7), to_unsigned(50, 7),
        to_unsigned(49, 7), to_unsigned(48, 7), to_unsigned(47, 7), to_unsigned(46, 7),
        to_unsigned(45, 7), to_unsigned(44, 7), to_unsigned(43, 7), to_unsigned(42, 7),
        to_unsigned(41, 7), to_unsigned(41, 7), to_unsigned(40, 7), to_unsigned(39, 7),
        to_unsigned(38, 7), to_unsigned(37, 7), to_unsigned(36, 7), to_unsigned(36, 7),
        to_unsigned(35, 7), to_unsigned(34, 7), to_unsigned(33, 7), to_unsigned(33, 7));

    -- ink / paper palette, authored in standard BT.601 10-bit (paper luma
    -- held near 768 so its tint survives). Separate 1-D constants: GHDL
    -- synth crashes on parallel reads of an array-of-arrays constant.
    --   0 copper  1 sepia  2 sanguine  3 prussian  4 banknote
    --   5 iron gall  6 gold on black  7 blueprint
    type t_pal is array(0 to 7) of unsigned(9 downto 0);
    constant C_INK_Y : t_pal := (
        to_unsigned(110, 10), to_unsigned(221, 10), to_unsigned(278, 10), to_unsigned(170, 10),
        to_unsigned(228, 10), to_unsigned(201, 10), to_unsigned(634, 10), to_unsigned(760, 10));
    constant C_INK_U : t_pal := (
        to_unsigned(508, 10), to_unsigned(457, 10), to_unsigned(429, 10), to_unsigned(623, 10),
        to_unsigned(493, 10), to_unsigned(574, 10), to_unsigned(274, 10), to_unsigned(540, 10));
    constant C_INK_V : t_pal := (
        to_unsigned(518, 10), to_unsigned(576, 10), to_unsigned(708, 10), to_unsigned(460, 10),
        to_unsigned(430, 10), to_unsigned(553, 10), to_unsigned(640, 10), to_unsigned(494, 10));
    constant C_PAP_Y : t_pal := (
        to_unsigned(760, 10), to_unsigned(760, 10), to_unsigned(765, 10), to_unsigned(765, 10),
        to_unsigned(762, 10), to_unsigned(760, 10), to_unsigned(111, 10), to_unsigned(269, 10));
    constant C_PAP_U : t_pal := (
        to_unsigned(440, 10), to_unsigned(428, 10), to_unsigned(452, 10), to_unsigned(494, 10),
        to_unsigned(455, 10), to_unsigned(458, 10), to_unsigned(515, 10), to_unsigned(672, 10));
    constant C_PAP_V : t_pal := (
        to_unsigned(549, 10), to_unsigned(559, 10), to_unsigned(564, 10), to_unsigned(515, 10),
        to_unsigned(504, 10), to_unsigned(546, 10), to_unsigned(516, 10), to_unsigned(427, 10));

    -- direction floor: low enough that soft form shading steers the
    -- strokes, above dither noise
    constant C_MAGFL : unsigned(12 downto 0) := to_unsigned(24, 13);

    -- two-stage value-noise hash (inferno lesson: 1-add hashes weave)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := ((cx & cy)) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h(9 downto 2);
    end function;

    -- Resolution scaling: per-mode steps from the video timing ID.
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
    -- interlaced modes: a field line spans ~2x a pixel's virtual width
    function f_gyh(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_p12   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k1    : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal s_k3    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k4    : unsigned(9 downto 0) := (others => '0');
    signal s_k5    : unsigned(9 downto 0) := (others => '0');
    signal s_inv   : std_logic := '0';                       -- S7
    signal s_wash  : std_logic := '0';                       -- S8
    signal s_edge_on : std_logic := '1';                     -- S9
    signal s_cross45 : std_logic := '0';                     -- S10

    -- style decode (K6): 0 stipple, 1 engraving, 2 woodcut
    signal s_style : unsigned(1 downto 0) := "01";

    -- stripe-field constants (per frame): 4 families at th0 + j*45 deg,
    -- pitch-scaled; the line step is doubled when interlaced
    type t_s16arr is array (0 to 3) of signed(15 downto 0);
    signal s_ck, s_sk2 : t_s16arr := (others => (others => '0'));
    signal s_xoff  : unsigned(1 downto 0) := "10";   -- cross family offset
    signal s_toff  : unsigned(1 downto 0) := "01";   -- third family offset

    -- pitch: n = K1/8 + 16 -> octave e = n(7:5), mantissa C_MANT(n(4:0))
    signal s_pn    : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_pe    : unsigned(2 downto 0) := "010";
    signal s_mant  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal s_pitch_sh : natural range 0 to 3 := 1;   -- stipple lattice only
    -- stipple half-cell and radius cap, per frame (4 << sh, (4 << sh) - 2) so
    -- the S9 dot test starts from registers
    signal s_hf7   : signed(6 downto 0)   := to_signed(8, 7);
    signal s_cap7  : unsigned(6 downto 0) := to_unsigned(6, 7);

    -- serial shift-add multiplier (qsin * mantissa), vblank only.  The
    -- mantissa is Q.2 and mode-scaled: x taps mant*xs*2 (xs = half units per
    -- pixel), y taps mant*ys>>6 (ys = Q4.8 units per field line); 1080p = 4*mant.
    signal s_mul_a : signed(9 downto 0) := (others => '0');
    signal s_mul_m : unsigned(10 downto 0) := (others => '0');
    signal s_mul_p : signed(21 downto 0) := (others => '0');
    signal s_mx4   : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_my4   : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_mya   : unsigned(18 downto 0) := (others => '0');
    signal s_ysh   : unsigned(11 downto 0) := (others => '0');

    -- Resolution scaling: virtual 1080-line steps from the timing ID
    signal s_xs    : unsigned(2 downto 0)  := to_unsigned(2, 3);     -- half units/pixel
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);  -- Q4.8 units/field line
    signal s_gyh   : std_logic := '0';     -- interlaced: halve gy for direction bucketing

    -- tonal ladder + widths (per frame). Ladder thresholds are signed:
    -- negative = overbite (every tone exceeds them, with extra width)
    signal s_pa    : unsigned(9 downto 0) := (others => '0');
    signal s_ob    : unsigned(8 downto 0) := (others => '0');
    signal s_tq    : unsigned(9 downto 0) := (others => '0');
    signal s_t1, s_t2, s_t3 : signed(10 downto 0) := to_signed(511, 11);
    signal s_wsh   : natural range 2 to 4 := 3;
    signal s_wbase : unsigned(6 downto 0) := to_unsigned(10, 7);
    signal s_wmax  : unsigned(6 downto 0) := to_unsigned(96, 7);
    signal s_ethr  : unsigned(11 downto 0) := to_unsigned(400, 12);
    signal s_jsh   : natural range 0 to 4 := 4;               -- hand tremor
    signal s_stip  : std_logic := '0';
    signal s_pd    : unsigned(7 downto 0) := (others => '0'); -- pit density
    signal s_pit_seed : unsigned(15 downto 0) := x"C3A5";

    -- ink / paper palette (per frame, stored U/V-swapped)
    signal s_ink_u, s_ink_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ink_y  : unsigned(9 downto 0) := to_unsigned(110, 10);
    signal s_pap_y  : unsigned(9 downto 0) := to_unsigned(760, 10);
    signal s_pap_u  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pap_v  : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_dracc : unsigned(11 downto 0) := (others => '0');  -- drift

    signal s_seq   : unsigned(7 downto 0) := (others => '0');
    signal s_run   : std_logic := '0';
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0) := (others => '0');
    signal s_th0   : unsigned(9 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';
    -- serrated analog vsync: act on the first edge after active video only
    signal s_saw   : std_logic := '0';

    -- interlace detect: field parity of this and the previous field
    signal s_fpar  : std_logic := '0';
    signal s_ilace : std_logic := '1';

    ----------------------------------------------------------------------
    -- pipeline
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 12) := (others => '0');

    signal r0_y, r0_u, r0_v : unsigned(9 downto 0) := (others => '0');
    signal s_aline   : unsigned(10 downto 0) := (others => '0');
    -- virtual coordinates (stipple lattice): x Q.1 per pixel, y Q.8 per line
    signal s_xva     : unsigned(11 downto 0) := (others => '0');
    signal s_yva     : unsigned(18 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';

    -- per-line tremor jitter (lstep sequencer)
    signal s_lstep  : unsigned(1 downto 0) := (others => '0');
    signal s_lh_a   : unsigned(11 downto 0) := (others => '0');
    signal s_ljit8  : unsigned(7 downto 0) := (others => '0');

    -- S1: write-path blur history + read counters
    signal a1_y8 : unsigned(7 downto 0) := (others => '0');
    signal h1, h2 : unsigned(7 downto 0) := (others => '0');
    signal wb8   : unsigned(7 downto 0) := (others => '0');
    signal lrx, lwx : unsigned(10 downto 0) := (others => '0');

    -- line buffers (2 rows above the current)
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb1, lb2 : t_lb8 := (others => (others => '0'));
    signal q1, q2   : std_logic_vector(7 downto 0) := (others => '0');

    -- S2 land, S3 sums, S4 gradient
    signal qr1, qr2 : unsigned(7 downto 0) := (others => '0');
    signal y8d      : unsigned(7 downto 0) := (others => '0');
    signal ss, ss1, ss2 : unsigned(9 downto 0) := (others => '0');
    signal dy, dy1, dy2 : signed(8 downto 0) := (others => '0');
    signal cl3, cl4, cl5 : unsigned(7 downto 0) := (others => '0');
    signal s_gx, s_gy : signed(11 downto 0) := (others => '0');

    -- S5 magnitude + |g| pair, S6 bucket
    signal mag5 : unsigned(12 downto 0) := (others => '0');
    signal ax5, ay5 : unsigned(11 downto 0) := (others => '0');
    signal sgn5 : std_logic := '0';
    signal bin6 : unsigned(1 downto 0) := (others => '0');
    signal bin_hold : unsigned(1 downto 0) := (others => '0');
    signal hold_cnt : unsigned(6 downto 0) := (others => '0');   -- half virtual units
    signal mag6 : unsigned(12 downto 0) := (others => '0');
    signal x_s6 : unsigned(10 downto 0) := (others => '0');
    -- stipple lattice track
    signal st1_8, st2_8 : unsigned(7 downto 0) := (others => '0');
    signal d1_6, d2_6 : unsigned(5 downto 0) := (others => '0');
    signal sh7  : unsigned(11 downto 0) := (others => '0');
    signal sj8  : unsigned(7 downto 0) := (others => '0');
    signal d1_7, d2_7 : unsigned(5 downto 0) := (others => '0');
    signal d1_8, d2_8 : unsigned(5 downto 0) := (others => '0');
    -- foul-bite pit track (2x2-pixel cells)
    signal ph7  : unsigned(11 downto 0) := (others => '0');
    signal pj8  : unsigned(7 downto 0) := (others => '0');
    signal pit9 : unsigned(1 downto 0) := (others => '0');

    -- stripe-field accumulators (advance under av(6); read same clock)
    type t_p16 is array (0 to 3) of unsigned(15 downto 0);
    signal p_acc : t_p16 := (others => (others => '0'));
    signal b_acc : t_p16 := (others => (others => '0'));

    -- S7 phase picks + widths, S8 distances, S9 ink decisions
    signal ph_h7, ph_x7, ph_t7 : unsigned(7 downto 0) := (others => '0');
    signal mag7 : unsigned(12 downto 0) := (others => '0');
    signal dh8, dx8, dt8 : unsigned(6 downto 0) := (others => '0');
    signal mag8 : unsigned(12 downto 0) := (others => '0');
    -- tone -> width ladder, spread over S6 (subtract) / S7 (shift+clamp)
    -- / S8 (pipe): fused it was the v0.1 HD critical path
    signal wd1_6, wd2_6, wd3_6 : signed(10 downto 0) := (others => '0');
    signal w1_7, w2_7, w3_7 : unsigned(6 downto 0) := (others => '0');
    signal en1_7, en2_7, en3_7 : std_logic := '0';
    signal w1_8, w2_8, w3_8 : unsigned(6 downto 0) := (others => '0');
    signal en1_8, en2_8, en3_8 : std_logic := '0';
    signal ink9  : unsigned(1 downto 0) := (others => '0');
    signal edge9 : std_logic := '0';

    -- chroma/luma delay to compose (S9/S10)
    type t_d10 is array (1 to 10) of unsigned(9 downto 0);
    signal du, dv, dyy : t_d10 := (others => (others => '0'));

    -- S10 lanes, S11 select, S12 out
    signal ln_iy, ln_iu, ln_iv : unsigned(9 downto 0) := (others => '0');
    signal ln_py, ln_pu, ln_pv : unsigned(9 downto 0) := (others => '0');
    signal ln_hy, ln_hu, ln_hv : unsigned(9 downto 0) := (others => '0');
    signal sel11 : unsigned(1 downto 0) := (others => '0');
    signal ink10 : unsigned(1 downto 0) := (others => '0');
    signal edge10 : std_logic := '0';
    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u, s_out_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin port: during the tap slots (s_seq(7) = '1') it presents
    -- family j = s_seq(5:4) at th0 + j*45 deg, +90 for the cos taps
    -- (s_seq(6) = '0'), steady for the whole 16-step slot. Address and ROM
    -- output are both registered (mercurial rule).
    ------------------------------------------------------------------------
    s_qs_a <= s_th0 + shift_left(resize(s_seq(5 downto 4), 10), 7)
                    + to_unsigned(256, 10) when s_seq(6) = '0' else
              s_th0 + shift_left(resize(s_seq(5 downto 4), 10), 7);

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one op per step).
    -- s_seq 0..127  : scalar steps (pitch, ladder, widths, palette)
    -- s_seq 128..255: 8 tap slots of 16 steps -- present angle, capture
    --                 qsin at step 3, 7-step shift-add multiply by the
    --                 pitch mantissa, octave shift + store at step 11
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_w  : unsigned(7 downto 0);
        variable v_j  : integer range 0 to 3;
        variable v_c  : signed(21 downto 0);
        variable v_pi : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);
            -- Resolution scaling: per-mode steps, free-running from the timing
            -- ID (static while a mode runs).
            s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
            s_gyh <= f_gyh(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_saw <= '1';
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1'
               and s_saw = '1' then
                s_saw <= '0';
                s_p12 <= unsigned(registers_in(7));
                s_k1  <= unsigned(registers_in(0));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_inv     <= registers_in(6)(0);
                s_wash    <= registers_in(6)(1);
                s_edge_on <= registers_in(6)(2);
                s_cross45 <= registers_in(6)(3);

                s_style    <= unsigned(registers_in(5)(9 downto 8));

                -- interlace detect: parity toggled since last field?
                s_fpar   <= data_in.field_n;
                if data_in.field_n /= s_fpar then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;

                -- plate angle: K2 sweeps 0..180 degrees; drift crawls the
                -- stroke phases when S11 is on
                s_th0 <= resize(unsigned(registers_in(1)(9 downto 1)), 10);
                if registers_in(6)(4) = '1' then
                    s_dracc <= s_dracc + 5;
                end if;

                s_seq <= (others => '0');
                s_run <= '1';
            elsif s_run = '1' and data_in.avid = '0' then
                if s_seq(7) = '1' then
                    -- tap slot: family v_j, cos (s_seq(6)='0') or sin
                    v_j := to_integer(s_seq(5 downto 4));
                    case to_integer(s_seq(3 downto 0)) is
                        when 3 =>
                            s_mul_a <= s_qs_r;
                            -- cos taps step per pixel (x), sin taps per
                            -- field line (y): mode-scaled Q.2 mantissas
                            if s_seq(6) = '0' then s_mul_m <= s_mx4;
                            else                   s_mul_m <= s_my4; end if;
                            s_mul_p <= (others => '0');
                        when 4 to 14 =>
                            -- Horner, mantissa MSB first
                            if s_mul_m(10) = '1' then
                                s_mul_p <= shift_left(s_mul_p, 1)
                                           + resize(s_mul_a, 22);
                            else
                                s_mul_p <= shift_left(s_mul_p, 1);
                            end if;
                            s_mul_m <= shift_left(s_mul_m, 1);
                        when 15 =>
                            -- octave: c = qsin*mant >> (e+1) (+2: Q.2
                            -- mantissa); phase = acc(15:8), so period =
                            -- 65536/|c| virtual units.  The low 16 bits are
                            -- the step mod 2^16, all the wrapping
                            -- accumulators need.
                            case s_pe is
                                when "000"  => v_c := shift_right(s_mul_p, 3);
                                when "001"  => v_c := shift_right(s_mul_p, 4);
                                when "010"  => v_c := shift_right(s_mul_p, 5);
                                when "011"  => v_c := shift_right(s_mul_p, 6);
                                when others => v_c := shift_right(s_mul_p, 7);
                            end case;
                            if s_seq(6) = '0' then
                                s_ck(v_j) <= v_c(15 downto 0);
                            else
                                -- per FIELD line (the y step carries interlace)
                                s_sk2(v_j) <= v_c(15 downto 0);
                            end if;
                        when others =>
                            null;
                    end case;
                else
                    case to_integer(s_seq(6 downto 0)) is
                        when 0 =>
                            -- pitch index: 4 octaves, 32 steps each
                            s_pn <= resize(s_k1(9 downto 3), 8) + 16;
                        when 1 =>
                            s_pe   <= s_pn(7 downto 5);
                            s_mant <= C_MANT(to_integer(s_pn(4 downto 0)));
                            -- stipple lattice stays power-of-2 (cell ~ pitch)
                            case s_pn(7 downto 5) is
                                when "000" | "001" => s_pitch_sh <= 0;
                                when "010"         => s_pitch_sh <= 1;
                                when "011"         => s_pitch_sh <= 2;
                                when others        => s_pitch_sh <= 3;
                            end case;
                        when 2 =>
                            -- Bite split: 0..60% walks the ladder, the top
                            -- 40% is overbite
                            if s_p12 > 614 then
                                s_pa <= to_unsigned(614, 10);
                                s_ob <= resize(s_p12 - 614, 9);
                            else
                                s_pa <= s_p12;
                                s_ob <= (others => '0');
                            end if;
                        when 3 =>
                            -- 0.39 * pa: t1 runs 240 -> 0 over 0..60%
                            s_tq <= shift_right(s_pa, 2) + shift_right(s_pa, 3);
                        when 4 =>
                            s_tq <= s_tq + shift_right(s_pa, 6);
                        when 5 =>
                            s_t1 <= to_signed(240, 11) - signed(resize(s_tq, 11));
                        when 6 =>
                            s_t1 <= s_t1 - signed(resize(shift_right(s_ob, 4), 11));
                        when 7 =>
                            s_t2 <= s_t1 + 128;
                        when 8 =>
                            s_t2 <= s_t2 - signed(resize(shift_right(s_ob, 2), 11));
                        when 9 =>
                            s_t3 <= s_t2 + 96;
                        when 10 =>
                            s_t3 <= s_t3 - signed(resize(shift_right(s_ob, 3), 11));
                        when 11 =>
                            -- ink width gain
                            case s_k3(9 downto 8) is
                                when "00"   => s_wsh <= 4;
                                when "01"   => s_wsh <= 3;
                                when others => s_wsh <= 2;
                            end case;
                        when 12 =>
                            -- contour threshold: Ink opens it, Bite lowers it
                            s_ethr <= to_unsigned(160, 12)
                                      + resize(not s_k3(9 downto 3), 12)
                                      + resize(not s_p12(9 downto 3), 12);
                        when 13 =>
                            -- overbite floods the contours
                            s_ethr <= s_ethr - resize(shift_right(s_ob, 2), 12);
                        when 14 =>
                            -- style presets (parameter-only: woodcut = fat
                            -- strokes, stipple flag swaps the test)
                            s_stip <= '0';
                            case s_style is
                                when "00" =>            -- stipple
                                    s_stip  <= '1';
                                    s_wbase <= to_unsigned(8, 7);
                                    s_wmax  <= to_unsigned(80, 7);
                                when "11" =>            -- woodcut
                                    s_wbase <= to_unsigned(22, 7);
                                    s_wmax  <= to_unsigned(116, 7);
                                when others =>          -- engraving
                                    s_wbase <= to_unsigned(10, 7);
                                    s_wmax  <= to_unsigned(96, 7);
                            end case;
                            -- families: 90 mode = hatch / +90 cross / +45
                            -- third; 45 mode = hatch / +45 cross / +135 third
                            if s_cross45 = '1' then
                                s_xoff <= "01";
                                s_toff <= "11";
                            else
                                s_xoff <= "10";
                                s_toff <= "01";
                            end if;
                            -- tremor depth
                            if s_k5(9 downto 7) = 0 then
                                s_jsh <= 0;
                            else
                                s_jsh <= to_integer(s_k5(9 downto 8)) + 1;
                            end if;
                        when 15 =>
                            -- overbite swells every stroke
                            s_wbase <= s_wbase + resize(shift_right(s_ob, 5), 7);
                        when 16 =>
                            -- and lets the shadow strokes merge solid
                            v_w := resize(s_wmax, 8) + resize(shift_right(s_ob, 2), 8);
                            if v_w > 127 then
                                s_wmax <= to_unsigned(127, 7);
                            else
                                s_wmax <= v_w(6 downto 0);
                            end if;
                        when 17 =>
                            -- palette; generated colours stored U/V-swapped
                            -- (the hardware swaps U/V)
                            v_pi := to_integer(s_k4(9 downto 7));
                            s_ink_y <= C_INK_Y(v_pi);
                            s_ink_u <= C_INK_V(v_pi);
                            s_ink_v <= C_INK_U(v_pi);
                            s_pap_y <= C_PAP_Y(v_pi);
                            s_pap_u <= C_PAP_V(v_pi);
                            s_pap_v <= C_PAP_U(v_pi);
                        when 18 =>
                            -- foul-bite pits: density rises through the
                            -- overbite; they boil with the live plate
                            s_pd <= resize(shift_right(s_ob, 2), 8);
                            s_pit_seed <= x"C3A5" + resize(s_dracc(11 downto 5), 16);
                        -- Resolution scaling: Q.2 mode-scaled mantissas.
                        -- x: mant * xs * 2 (1080p 4*mant); y: mant * ys >> 6
                        -- by a 12-step serial shift-add (1080p 4*mant).
                        when 19 =>
                            case to_integer(s_xs) is
                                when 3      => s_mx4 <= shift_left(resize(s_mant, 11), 2)
                                                        + shift_left(resize(s_mant, 11), 1);
                                when 4      => s_mx4 <= shift_left(resize(s_mant, 11), 3);
                                when others => s_mx4 <= shift_left(resize(s_mant, 11), 2);
                            end case;
                            s_mya <= (others => '0');
                            s_ysh <= s_ys;
                        when 20 to 31 =>
                            if s_ysh(11) = '1' then
                                s_mya <= shift_left(s_mya, 1) + resize(s_mant, 19);
                            else
                                s_mya <= shift_left(s_mya, 1);
                            end if;
                            s_ysh <= shift_left(s_ysh, 1);
                        when 32 =>
                            s_my4 <= s_mya(16 downto 6);
                        when 33 =>
                            s_hf7  <= signed(resize(shift_left(to_unsigned(4, 6), s_pitch_sh), 7));
                            s_cap7 <= resize(shift_left(to_unsigned(4, 6), s_pitch_sh), 7) - 2;
                        when others =>
                            null;
                    end case;
                end if;
                if s_seq = 255 then
                    s_run <= '0';
                else
                    s_seq <= s_seq + 1;
                end if;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a, v_b : unsigned(11 downto 0);
        variable v_lo, v_hi : unsigned(14 downto 0);
        variable v_d  : unsigned(1 downto 0);
        variable v_ph : unsigned(7 downto 0);
        variable v_w  : unsigned(8 downto 0);
        variable v_ink : unsigned(1 downto 0);
        variable v_iy : unsigned(10 downto 0);
        variable v_dd : unsigned(6 downto 0);
        variable v_hf : signed(6 downto 0);
        variable v_rad, v_cap : unsigned(6 downto 0);
        variable v_sub, v_sb2 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 12 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S0: input register
            if data_in.avid = '1' then
                r0_y <= unsigned(data_in.y);
                r0_u <= unsigned(data_in.u);
                r0_v <= unsigned(data_in.v);
            end if;

            -- S1: [1 2 1]/4 write-path blur history; read addr counter
            if av(1) = '1' then
                a1_y8 <= r0_y(9 downto 2);
                h1 <= a1_y8;
                h2 <= h1;
                wb8 <= resize(shift_right(resize(h2, 10)
                              + shift_left(resize(h1, 10), 1)
                              + resize(a1_y8, 10), 2), 8);
                lrx <= lrx + 1;
            end if;

            -- S2: land the line-buffer reads; buffer writes ride this
            -- stage (p_lb1/2)
            if av(2) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                y8d <= wb8;
                lwx <= lwx + 1;
            end if;

            -- S3: Sobel column sums + two-column histories; center-row
            -- luma tap rides along
            if av(3) = '1' then
                ss  <= resize(qr2, 10) + shift_left(resize(qr1, 10), 1)
                       + resize(y8d, 10);
                dy  <= signed(resize(qr2, 9)) - signed(resize(y8d, 9));
                ss1 <= ss;  ss2 <= ss1;
                dy1 <= dy;  dy2 <= dy1;
                cl3 <= qr1;
            end if;

            -- S4: gradients (gx rightward, gy UPWARD: row above minus row)
            if av(4) = '1' then
                s_gx <= signed(resize(ss, 12)) - signed(resize(ss2, 12));
                s_gy <= resize(dy, 12) + shift_left(resize(dy1, 12), 1)
                        + resize(dy2, 12);
                cl4 <= cl3;
            end if;

            -- S5: |gx|, |gy|, magnitude, quadrant sign
            if av(5) = '1' then
                ax5 <= resize(f_absu(s_gx), 12);
                ay5 <= resize(f_absu(s_gy), 12);
                mag5 <= resize(f_absu(s_gx), 13) + resize(f_absu(s_gy), 13);
                if (s_gx < 0) /= (s_gy < 0) then
                    sgn5 <= '1';
                else
                    sgn5 <= '0';
                end if;
                cl5 <= cl4;
            end if;

            -- S6: direction bucket -- |gy| against tan(22.5) ~ .406 and
            -- tan(67.5) ~ 2.406 of |gx| (symmetric 45-degree bins). The
            -- stripe families step y DOWN, gy points UP: in screen space
            -- a gradient whose components' signs differ points at +45.
            -- Flat areas (below the mag floor) fall to family 0 = the
            -- plate angle. The stripe accumulators advance this stage;
            -- the warm-up guard blanks the window's first lines/columns.
            if av(6) = '1' then
                v_a := ax5;
                -- interlaced modes: halve gy (a field line is ~2x a pixel's
                -- virtual width) so the bins see true screen angles
                if s_gyh = '1' then v_b := '0' & ay5(11 downto 1);
                else                v_b := ay5; end if;
                v_lo := resize(shift_right(v_a, 2), 15)
                        + resize(shift_right(v_a, 3), 15)
                        + resize(shift_right(v_a, 5), 15);
                v_hi := shift_left(resize(v_a, 15), 1)
                        + resize(shift_right(v_a, 2), 15)
                        + resize(shift_right(v_a, 3), 15)
                        + resize(shift_right(v_a, 5), 15);
                if resize(v_b, 15) < v_lo then
                    v_d := "00";
                elsif resize(v_b, 15) > v_hi then
                    v_d := "10";
                elsif sgn5 = '1' then
                    v_d := "01";
                else
                    v_d := "11";
                end if;
                if s_aline < 4 or x_s6 < 8 then
                    bin6 <= "00";
                    hold_cnt <= (others => '0');
                elsif mag5 >= C_MAGFL then
                    bin6 <= v_d;
                    bin_hold <= v_d;
                    hold_cnt <= to_unsigned(126, 7);   -- 63 virtual units
                elsif hold_cnt /= 0 then
                    -- ridge/valley zones (gradient through zero) inherit
                    -- the last strong direction on this line instead of
                    -- striping with the default family
                    bin6 <= bin_hold;
                    if hold_cnt > resize(s_xs, 7) then hold_cnt <= hold_cnt - s_xs;
                    else                                hold_cnt <= (others => '0'); end if;
                else
                    bin6 <= "00";
                end if;
                mag6 <= mag5;
                x_s6 <= x_s6 + 1;
                s_xva <= s_xva + s_xs;

                -- stipple lattice coords + sub-cell diagonal coords (virtual)
                v_sub := s_xva(11 downto 1) + s_yva(18 downto 8);
                v_sb2 := s_xva(11 downto 1) - s_yva(18 downto 8);
                st1_8 <= resize(shift_right(v_sub, s_pitch_sh + 3), 8);
                st2_8 <= resize(shift_right(v_sb2, s_pitch_sh + 3), 8);
                case s_pitch_sh is
                    when 0 =>
                        d1_6 <= resize(v_sub(2 downto 0), 6);
                        d2_6 <= resize(v_sb2(2 downto 0), 6);
                    when 1 =>
                        d1_6 <= resize(v_sub(3 downto 0), 6);
                        d2_6 <= resize(v_sb2(3 downto 0), 6);
                    when 2 =>
                        d1_6 <= resize(v_sub(4 downto 0), 6);
                        d2_6 <= resize(v_sb2(4 downto 0), 6);
                    when others =>
                        d1_6 <= resize(v_sub(5 downto 0), 6);
                        d2_6 <= resize(v_sb2(5 downto 0), 6);
                end case;

                -- width ladder stage 1: tone excess over each threshold
                wd1_6 <= signed(resize(not cl5, 11)) - s_t1;
                wd2_6 <= signed(resize(not cl5, 11)) - s_t2;
                wd3_6 <= signed(resize(not cl5, 11)) - s_t3;

                for k in 0 to 3 loop
                    p_acc(k) <= p_acc(k) + unsigned(s_ck(k));
                end loop;
            end if;

            -- S7: pick the three stroke phases (hatch / cross / third) by
            -- bucket mux -- phase is the static slice p(15:8) at every
            -- pitch; hand tremor jitters the phase per line; stipple and
            -- pit hashes stage A
            if av(7) = '1' then
                v_ph := p_acc(to_integer(bin6))(15 downto 8);
                if s_jsh /= 0 then
                    v_ph := v_ph + shift_right(s_ljit8, 4 - s_jsh);
                end if;
                ph_h7 <= v_ph;
                v_ph := p_acc(to_integer(bin6 + s_xoff))(15 downto 8);
                if s_jsh /= 0 then
                    v_ph := v_ph + shift_right(s_ljit8, 4 - s_jsh);
                end if;
                ph_x7 <= v_ph;
                ph_t7 <= p_acc(to_integer(bin6 + s_toff))(15 downto 8);
                mag7 <= mag6;

                -- width ladder stage 2: shift + base + clamp per level
                en1_7 <= '0';
                if wd1_6 >= 0 then
                    en1_7 <= '1';
                    v_w := resize(s_wbase, 9)
                           + resize(shift_right(unsigned(wd1_6), s_wsh), 9);
                    if v_w > resize(s_wmax, 9) then
                        w1_7 <= s_wmax;
                    else
                        w1_7 <= resize(v_w, 7);
                    end if;
                end if;
                en2_7 <= '0';
                if wd2_6 >= 0 then
                    en2_7 <= '1';
                    v_w := resize(s_wbase, 9)
                           + resize(shift_right(unsigned(wd2_6), s_wsh), 9);
                    if v_w > resize(s_wmax, 9) then
                        w2_7 <= s_wmax;
                    else
                        w2_7 <= resize(v_w, 7);
                    end if;
                end if;
                en3_7 <= '0';
                if wd3_6 >= 0 then
                    en3_7 <= '1';
                    v_w := resize(s_wbase, 9)
                           + resize(shift_right(unsigned(wd3_6), s_wsh), 9);
                    if v_w > resize(s_wmax, 9) then
                        w3_7 <= s_wmax;
                    else
                        w3_7 <= resize(v_w, 7);
                    end if;
                end if;

                sh7  <= f_hash_a(st1_8, st2_8, x"7A3C");
                ph7  <= f_hash_a(x_s6(8 downto 1), s_aline(8 downto 1),
                                 s_pit_seed);
                d1_7 <= d1_6;
                d2_7 <= d2_6;
            end if;

            -- S8: stripe-center distances + tonal widths; hashes stage B
            if av(8) = '1' then
                if ph_h7 >= 128 then
                    dh8 <= resize(shift_right(ph_h7 - 128, 1), 7);
                else
                    dh8 <= resize(shift_right(128 - ph_h7, 1), 7);
                end if;
                if ph_x7 >= 128 then
                    dx8 <= resize(shift_right(ph_x7 - 128, 1), 7);
                else
                    dx8 <= resize(shift_right(128 - ph_x7, 1), 7);
                end if;
                if ph_t7 >= 128 then
                    dt8 <= resize(shift_right(ph_t7 - 128, 1), 7);
                else
                    dt8 <= resize(shift_right(128 - ph_t7, 1), 7);
                end if;

                -- width ladder stage 3: plain pipes
                w1_8 <= w1_7;  en1_8 <= en1_7;
                w2_8 <= w2_7;  en2_8 <= en2_7;
                w3_8 <= w3_7;  en3_8 <= en3_7;
                mag8 <= mag7;

                sj8  <= f_hash_b(sh7);
                pj8  <= f_hash_b(ph7);
                d1_8 <= d1_7;
                d2_8 <= d2_7;
            end if;

            -- S9: ink decision. Engraving: union of the three stroke
            -- levels with a half-ink antialias band. Stipple: hash-gated
            -- diamond dots sized by tone. Contour ink on top (S9 switch).
            -- Foul-bite pits decided alongside, merged at S10.
            if av(9) = '1' then
                v_ink := "00";
                if s_stip = '0' then
                    if en1_8 = '1' then
                        if resize(dh8, 8) < resize(w1_8(6 downto 1), 8) then
                            v_ink := "10";
                        elsif resize(dh8, 8) < resize(w1_8(6 downto 1), 8) + 4
                              and v_ink = "00" then
                            v_ink := "01";
                        end if;
                    end if;
                    if en2_8 = '1' and v_ink /= "10" then
                        if resize(dx8, 8) < resize(w2_8(6 downto 1), 8) then
                            v_ink := "10";
                        elsif resize(dx8, 8) < resize(w2_8(6 downto 1), 8) + 4
                              and v_ink = "00" then
                            v_ink := "01";
                        end if;
                    end if;
                    if en3_8 = '1' and v_ink /= "10" then
                        if resize(dt8, 8) < resize(w3_8(6 downto 1), 8) then
                            v_ink := "10";
                        end if;
                    end if;
                else
                    -- stipple: cell fires by hash vs tone, dot radius by
                    -- tone (diamond metric |d1-c| + |d2-c|, c = half cell)
                    -- with the radius CAPPED below the half-cell so dots
                    -- stay separate instead of merging into chunks
                    v_hf := s_hf7;
                    v_rad := resize(w1_8(6 downto 2), 7) + 1;
                    v_cap := s_cap7;
                    if v_rad > v_cap then
                        v_rad := v_cap;
                    end if;
                    if en1_8 = '1'
                       and resize(sj8, 9) < w1_8 & "00" then
                        v_dd := resize(f_absu(signed(resize(d1_8, 7)) - v_hf), 7)
                                + resize(f_absu(signed(resize(d2_8, 7)) - v_hf), 7);
                        if v_dd < v_rad then
                            v_ink := "10";
                        end if;
                    end if;
                end if;
                ink9 <= v_ink;
                if pj8 < shift_right(s_pd, 1) then
                    pit9 <= "10";
                elsif pj8 < s_pd then
                    pit9 <= "01";
                else
                    pit9 <= "00";
                end if;
                if s_edge_on = '1' and mag8 > resize(s_ethr, 13) then
                    edge9 <= '1';
                else
                    edge9 <= '0';
                end if;
            end if;

            -- S10: lanes fully assembled one stage before the mux
            if av(10) = '1' then
                -- ink lane
                ln_iy <= s_ink_y;
                ln_iu <= s_ink_u;
                ln_iv <= s_ink_v;
                -- paper lane (wash = half-sat video under-print, luma held
                -- under ~768 so the colour survives; else paper + grain)
                if s_wash = '1' then
                    v_iy := to_unsigned(440, 11)
                            + resize(dyy(10)(9 downto 2), 11)
                            + resize(dyy(10)(9 downto 3), 11);
                    if v_iy > 768 then
                        ln_py <= to_unsigned(768, 10);
                    else
                        ln_py <= v_iy(9 downto 0);
                    end if;
                    ln_pu <= resize(shift_right(resize(du(10), 11)
                                    + to_unsigned(512, 11), 1), 10);
                    ln_pv <= resize(shift_right(resize(dv(10), 11)
                                    + to_unsigned(512, 11), 1), 10);
                else
                    ln_py <= s_pap_y - resize(sj8(2 downto 0), 10);
                    ln_pu <= s_pap_u;
                    ln_pv <= s_pap_v;
                end if;
                -- half-ink lane (paper/ink midpoint, cheap antialias)
                ln_hy <= ('0' & s_pap_y(9 downto 1))
                         + ('0' & s_ink_y(9 downto 1));
                ln_hu <= ('0' & s_pap_u(9 downto 1))
                         + ('0' & s_ink_u(9 downto 1));
                ln_hv <= ('0' & s_pap_v(9 downto 1))
                         + ('0' & s_ink_v(9 downto 1));
                -- pits land only on bare paper
                if ink9 = "00" then
                    ink10 <= pit9;
                else
                    ink10 <= ink9;
                end if;
                edge10 <= edge9;
            end if;

            -- S11: priority select (contour > full ink > half > paper)
            if av(11) = '1' then
                if edge10 = '1' then
                    sel11 <= "11";
                elsif ink10 = "10" then
                    sel11 <= "10";
                elsif ink10 = "01" then
                    sel11 <= "01";
                else
                    sel11 <= "00";
                end if;
            end if;

            -- S12: mux -> output (Invert swaps ink and paper roles:
            -- scratchboard). Blanking gate: neutral outside active video.
            if av(12) = '1' then
                case sel11 is
                    when "11" | "10" =>
                        if s_inv = '1' then
                            s_out_y <= ln_py;
                            s_out_u <= ln_pu;
                            s_out_v <= ln_pv;
                        else
                            s_out_y <= ln_iy;
                            s_out_u <= ln_iu;
                            s_out_v <= ln_iv;
                        end if;
                    when "01" =>
                        s_out_y <= ln_hy;
                        s_out_u <= ln_hu;
                        s_out_v <= ln_hv;
                    when others =>
                        if s_inv = '1' then
                            s_out_y <= ln_iy;
                            s_out_u <= ln_iu;
                            s_out_v <= ln_iv;
                        else
                            s_out_y <= ln_py;
                            s_out_u <= ln_pu;
                            s_out_v <= ln_pv;
                        end if;
                end case;
            else
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= to_unsigned(512, 10);
                s_out_v <= to_unsigned(512, 10);
            end if;

            -- video delay pipes for the wash lane
            dyy(1) <= r0_y;
            du(1)  <= r0_u;
            dv(1)  <= r0_v;
            for i in 2 to 10 loop
                dyy(i) <= dyy(i - 1);
                du(i)  <= du(i - 1);
                dv(i)  <= dv(i - 1);
            end loop;

            -- per-line tremor jitter (2-step lstep sequencer)
            if s_lstep = 1 then
                -- tremor pattern boils at ~10 Hz when the plate is live
                -- (full-rate reseeding strobes at field rate); locked
                -- plate = static hand wobble
                s_lh_a  <= f_hash_a(s_aline(7 downto 0),
                                    resize(s_dracc(11 downto 5), 8), x"B33F");
                s_lstep <= "10";
            elsif s_lstep = 2 then
                s_ljit8 <= f_hash_b(s_lh_a);
                s_lstep <= "00";
            end if;

            -- line bookkeeping: reload the stripe accumulators from the
            -- row bases, step the bases by the per-FIELD-line step (virtual
            -- units; the y step already carries the interlace x2)
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                x_s6 <= (others => '0');
                s_xva <= (others => '0');
                s_lstep <= "01";

                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline /= 2047 then
                        s_aline <= s_aline + 1;
                    end if;
                    s_yva <= s_yva + s_ys;
                    for k in 0 to 3 loop
                        b_acc(k) <= b_acc(k) + unsigned(s_sk2(k));
                        p_acc(k) <= b_acc(k) + unsigned(s_sk2(k));
                    end loop;
                else
                    s_aline <= (others => '0');
                    s_yva   <= (others => '0');
                    -- field start: base = drift phase (pitch-independent,
                    -- 2.5 phase units per field) + half a field step for
                    -- the BOTTOM field (field_n = '0') when interlaced
                    for k in 0 to 3 loop
                        if s_ilace = '1' and data_in.field_n = '0' then
                            b_acc(k) <= shift_left(resize(s_dracc(8 downto 0), 16), 7)
                                        + unsigned(shift_right(s_sk2(k), 1));
                            p_acc(k) <= shift_left(resize(s_dracc(8 downto 0), 16), 7)
                                        + unsigned(shift_right(s_sk2(k), 1));
                        else
                            b_acc(k) <= shift_left(resize(s_dracc(8 downto 0), 16), 7);
                            p_acc(k) <= shift_left(resize(s_dracc(8 downto 0), 16), 7);
                        end if;
                    end loop;
                end if;
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- luma line buffers: canonical 1W1R, read leads write by one stage
    ------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(2) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(wb8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(2) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

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
    s_io_align_in.avid    <= s_avid_sr(C_LATENCY - 1);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_LATENCY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (13 -> 16).
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

end architecture intaglio;
