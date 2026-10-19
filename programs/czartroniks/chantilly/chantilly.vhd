-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- chantilly.vhd  (v1.0.2 -- black-lace halftone: video as cream circles on black)
--
-- v1.0.2 (resolution scaling): the lace DDA steps per mode from the video
--   timing ID (registers_in(8)) in virtual 1080-line units -- x step
--   1.0/1.5/2.0 per pixel, y step per FIELD line (1080i 2.0, 720p 1.5, 480i
--   4.44, ...) -- so 720p and SD show the 1080p lace at the same size
--   relative to screen height, round on a 4:3 SD screen.  Replaces the
--   old SD-only "s<<1 if W < 1000" and the field_n-toggle interlace
--   detection (field_n is tied high on hardware: SD rosettes were 2:1
--   tall).  1080p is bit-identical (every step is exactly 1.0 there).
--
-- v0.3 (look pass):
--   * ROUNDER DOTS: dot size reads a 2-D blurred luma -- 8-px running box,
--     then a vertical one-pole IIR (k=1/4) kept in a dual-bank line buffer
--     (8 EBR), so a dot no longer traces the video's contours.
--   * 2-D INK: the same buffer gives a vertical gradient, so horizontal
--     edges (eyes, mouths, horizons) ink too.
--   * K4 Ink at 0 = no ink; S9 is now COLOUR (dots take the source chroma,
--     ~1.75x saturation, scaled by the dot key; S7 Ivory warms it).
--   * Ghost works on the White ground too (bright video in the ground).
--   * K3 FLOW = scroll + SPIN: every fan/rosette rotates, adjacent bands
--     counter-rotate (one angle add before theta*ring, seamless by mod 4096).
--   * P12 spans +-1.75 octaves (HD s = 861..76): fine veil -> giant rosettes.
--
-- v0.2 (pre-first-flash hardware-contract pass):
--   * P12 inverted (up = bigger lace) and SD gets a FINER lace (s<<1), not
--     coarser -- s is lace units per pixel, so bigger s = smaller lace.
--   * Luma capped at 766 (was 957: full-scale Y clips RGB and kills chroma).
--   * Ivory tint ~2x stronger and U/V-swapped at the output compose.
--   * Interlace: the +1-line offset goes on the BOTTOM field (field_n='0').
--   * W/H latched with +-2 hysteresis (analog line-length jitter).
--   * Generated avid (hsync-parity locked, fixed width) for the encoders.
--   * Raw luma tapped early; only derived radius/ink/ghost values ride deep.
--
-- Source image: black Chantilly-style lace -- bands of white/cream dots in
-- fans, rosettes and pebble rows on black ground, with a thick black wavy
-- cordonnet breaking over the dots.  This program re-renders incoming video
-- as that lace.
--
-- THE MAPPING (grid-free by construction):
--   The frame is cut into horizontal LACE BANDS (256 lace units tall).  Each
--   band owns a row of scallop CENTRES (spaced P apart, half-P staggered on
--   alternate bands); every pixel is taken to polar (r, theta) about its
--   nearest centre by a CORDIC vectoring pipeline.  Dots sit on CONCENTRIC
--   ARCS: ring = r/G, and the number of dots around a ring is 6*ring, so the
--   arc cells stay ~square at any radius yet never line up radially -- a
--   phyllotaxis-like packing with no Cartesian grid anywhere.  The dot test
--   is  frac_r^2 + frac_t^2 < radius^2  in NORMALIZED CELL UNITS (Q7,
--   128 = half a cell), so one compare renders every dot at every scale.
--
--   Four band styles cycle down the frame (the image's texture variety):
--     0 PEBBLE  centre 2 bands deep, G=32 -> gentle arcs of medium circles
--     1 FAN     centre at band base,  G=32 -> radiating spiderweb fans
--     2 SEED    centre 2 bands deep,  G=16 -> dense small-dot packing
--     3 BLOOM   centre mid-band, P=256, G=16 -> full rosette flowers
--
--   Dot radius comes from the DELAYED INPUT LUMA at the pixel (analog
--   halftone): bright video = fat merging cream dots, dark = pinpricks.
--
-- THE CORDONNET (black linework), two sources:
--   * r > R_edge of the band's fans is painted black -- the fan's own outer
--     radius IS a scalloped thick black border, cusping exactly at the fan
--     seams (no sine tables, no extra geometry).  A thin seam wisp at
--     |dx| ~ P/2 joins the cusps.  K5 sets the border weight.
--   * luma-gradient INK: strong vertical edges in the video become black
--     strokes that cut down through the dots (S9, weight on K4).
--
-- SCALE (P12) is a Q8 DDA: x,y are accumulated in LACE UNITS (u += s per
-- pixel), so every pitch stays a power of two in lace space while the screen
-- scale sweeps continuously -- the whole doily zooms without a single
-- variable modulo.  DRIFT (K3) scrolls lace space vertically; band structure
-- wraps mod 4096 lace units so the cycle is seamless.
--
-- Streaming, no BRAM, no line buffer.  Per-pixel multiplies: theta*ring,
-- ring*twist, three Q7 squares, luma*gain -- all narrow, each in its own
-- stage.  Blanking-gated output (Y=64, U=V=512 outside avid).
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture chantilly of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 19;
    constant C_MID     : unsigned(9 downto 0) := to_unsigned(512, 10);

    constant C_DB   : integer := 14;      -- knob detent deadband

    -- lace-space anchors (Q8): keep u,v positive; both wrap mod 2^20 = 4096
    -- lace units, which the band cycle (16 bands) divides exactly.
    constant C_UOFF : unsigned(19 downto 0) := to_unsigned(524288, 20);
    constant C_VOFF : unsigned(19 downto 0) := to_unsigned(524288, 20);

    -- band style constants (lace units, Q2 where noted)
    --   style 0 PEBBLE   1 FAN      2 SEED     3 BLOOM
    -- depth (Q2):  2048      1024       2048       512
    -- P shift:     9         9          9          8
    -- G shift:     5         5          4          4
    -- R_edge(Q2):  2240      1344       2240       704
    constant C_DEPTH0 : signed(12 downto 0) := to_signed(2048, 13);
    constant C_DEPTH1 : signed(12 downto 0) := to_signed(1024, 13);
    constant C_DEPTH3 : signed(12 downto 0) := to_signed(512, 13);
    constant C_REDGE0 : integer := 2240;
    constant C_REDGE1 : integer := 1344;
    constant C_REDGE3 : integer := 704;
    constant C_SEAM9  : integer := 1012;  -- |dx| beyond this (Q2) = seam wisp, P=512
    constant C_SEAM8  : integer := 500;   -- P=256

    ----------------------------------------------------------------------
    -- exp2 mantissa ROM: round(1024 * 2^(f/64)), f = 0..63  -> [1024,2048)
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
    -- Resolution scaling: per-mode steps from the video timing ID
    -- (mondrian pilot).  x step in half virtual units per pixel: 1080 2
    -- (1.0), 720p 3 (1.5), SD 4 (2.0: 720 px span 1440 units, a 4:3 window
    -- of the 1080 canvas).  y step in Q4.8 virtual units per FIELD line
    -- (1080 / active lines, doubled for interlaced modes).
    ----------------------------------------------------------------------
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    -- interlaced from the timing ID only (field_n is tied high on hardware)
    function f_ilace(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k_fill  : unsigned(9 downto 0) := to_unsigned(220, 10);
    signal s_k_gain  : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_k_drift : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k_ink   : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_k_bord  : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_k_twist : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_zoom_k  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ivory   : std_logic := '1';    -- S7 Ivory/Mono
    signal s_inv     : std_logic := '0';    -- S8 invert (white ground)
    signal s_inkon   : std_logic := '1';    -- K4 above detent = ink on
    signal s_colour  : std_logic := '0';    -- S9 dots take source chroma
    signal s_weave   : std_logic := '0';    -- S10 0=Doily(4 styles) 1=Pebble(calm)
    signal s_ghost   : std_logic := '0';    -- S11 dim video in the ground

    ----------------------------------------------------------------------
    -- runtime raster measurement
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ilace : std_logic := '0';
    signal s_avid_q : std_logic := '0';
    signal s_xl, s_hl : unsigned(11 downto 0) := (others => '0');
    signal s_xl_go, s_hl_go : std_logic := '0';
    signal s_wp2, s_wm2, s_hp2, s_hm2 : unsigned(11 downto 0) := (others => '0');

    -- generated avid (aurora/cubist pattern): start 3-4 clocks after the
    -- source avid rise at the source's usual hsync PARITY (saturating vote),
    -- end after W pixels.  The encoders pair Cb/Cr by counting from hsync,
    -- so a wandering avid edge swaps U/V for that line (thin tint lines).
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal s_wm1  : unsigned(11 downto 0) := to_unsigned(719, 12);

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_scl   : unsigned(10 downto 0) := to_unsigned(256, 11); -- scale s, Q8
                                         -- (lace units per VIRTUAL unit)
    -- per-mode virtual steps (latched at vsync) and the lace steps they give
    signal s_xs    : unsigned(2 downto 0)  := to_unsigned(2, 3);    -- Q.1 / pixel
    signal s_ysq   : unsigned(11 downto 0) := to_unsigned(256, 12); -- Q8 / field line
    signal s_sx    : unsigned(10 downto 0) := to_unsigned(256, 11); -- lace/pixel, Q8
    signal s_sy    : unsigned(11 downto 0) := to_unsigned(256, 12); -- lace/field line, Q8
    signal s_spin  : unsigned(15 downto 0) := (others => '0');      -- spin angle, Q4
    signal s_spinp, s_spinn : unsigned(11 downto 0) := (others => '0'); -- +-spin
    signal s_u0    : unsigned(19 downto 0) := (others => '0');      -- line-start u
    signal s_v0    : unsigned(19 downto 0) := (others => '0');      -- frame-start v
    signal s_scroll : unsigned(19 downto 0) := (others => '0');     -- drift accum, Q8
    signal s_gain  : unsigned(8 downto 0) := to_unsigned(120, 9);   -- luma slope
    signal s_bias  : unsigned(7 downto 0) := to_unsigned(55, 8);    -- dot radius floor
    signal s_twist : signed(7 downto 0) := (others => '0');
    signal s_inkth : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_pth, s_nth : signed(9 downto 0) := (others => '0');    -- +-inkth
    signal s_bw    : unsigned(8 downto 0) := to_unsigned(150, 9);   -- border weight Q2
    signal s_re0, s_re1, s_re2, s_re3 : unsigned(11 downto 0) := (others => '1');
    signal s_svi   : integer range -2 to 1 := 0;                    -- scale exp int part
    signal s_svf   : unsigned(5 downto 0) := (others => '0');       -- scale exp frac

    -- relay copies of pixel-rate multiply operands (placement decoupling)
    signal s_twist_d : signed(7 downto 0) := (others => '0');
    signal s_gain_d  : unsigned(8 downto 0) := to_unsigned(120, 9);

    -- vblank sequencer + one shared multiplier
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_lgt8, s_vtrig : std_logic := '0';  -- registered vsync trigger
    signal s_vs_pulse   : std_logic := '0';
    signal s_ma    : signed(12 downto 0) := (others => '0');
    signal s_mb    : signed(12 downto 0) := (others => '0');
    signal s_mp    : signed(25 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- lace-space DDA accumulators (Q8)
    ----------------------------------------------------------------------
    signal s_u     : unsigned(19 downto 0) := (others => '0');
    signal s_vb    : unsigned(19 downto 0) := (others => '0');     -- next line
    signal s_vc    : unsigned(19 downto 0) := (others => '0');     -- current line
    signal s_avid_p : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    -- A1 (#2): band/style split, local centre coords (Q2)
    signal a1_dx   : signed(12 downto 0) := (others => '0');
    signal a1_dy   : signed(12 downto 0) := (others => '0');
    signal a1_sty  : unsigned(1 downto 0) := (others => '0');
    signal a1_seam : std_logic := '0';
    signal a1_kb   : std_logic := '0';                          -- band parity

    -- A2 (#3): abs + signs
    signal a2_a, a2_b : unsigned(11 downto 0) := (others => '0');
    signal a2_dxs, a2_dys : std_logic := '0';

    -- A3 (#4): max/min swap
    signal a3_hi, a3_lo : unsigned(11 downto 0) := (others => '0');
    signal a3_swap, a3_dxs, a3_dys : std_logic := '0';

    -- CORDIC (#5..#10): 10 iterations, 2 per stage, guard 5
    constant C_NS : integer := 5;
    constant C_CG : integer := 5;
    type t_cord is array(0 to C_NS) of signed(18 downto 0);
    type t_cang is array(0 to C_NS) of signed(12 downto 0);
    type t_cb   is array(0 to C_NS) of std_logic;
    signal cordx, cordy : t_cord := (others => (others => '0'));
    signal cordz        : t_cang := (others => (others => '0'));
    signal cordsw, corddxs, corddys : t_cb := (others => '0');

    type t_atan is array(0 to 9) of integer;
    constant C_ATAN : t_atan := (512, 302, 160, 81, 41, 20, 10, 5, 3, 1);

    -- RU (#11): unscaled radius (Q2) + folded angle
    signal ru_r   : unsigned(11 downto 0) := (others => '0');
    signal ru_ang : signed(12 downto 0) := (others => '0');

    -- RB (#12): ring split, border test
    signal rb_ring : unsigned(5 downto 0) := (others => '0');
    signal rb_fr   : signed(7 downto 0) := (others => '0');   -- radial frac, Q7
    signal rb_rz   : std_logic := '0';                        -- ring-0 flag
    signal rb_bord : std_logic := '0';
    signal rb_ang  : signed(12 downto 0) := (others => '0');

    -- T1 (#13): theta*ring, ring*twist
    signal t1_m   : signed(19 downto 0) := (others => '0');
    signal t1_tw  : signed(14 downto 0) := (others => '0');
    signal t1_fr  : signed(7 downto 0) := (others => '0');
    signal t1_rz  : std_logic := '0';

    -- T2 (#14): tangential frac
    signal t2_ft  : signed(7 downto 0) := (others => '0');
    signal t2_fr  : signed(7 downto 0) := (others => '0');

    -- D1 (#15): squares
    signal d1_fp2, d1_ft2 : unsigned(14 downto 0) := (others => '0');

    -- D2 (#16): normalized distance^2
    signal d2_d2  : unsigned(15 downto 0) := (others => '0');

    -- C1a (#17): dot key (subtract + clamp only)
    signal c1a_key : unsigned(7 downto 0) := (others => '0');
    -- C1b (#18): lace compose (invert / cut / ghost select)
    signal c1_key : unsigned(7 downto 0) := (others => '0');
    signal c1_gh  : unsigned(6 downto 0) := (others => '0');
    signal c1_useg : std_logic := '0';
    signal c1_gw   : std_logic := '0';                          -- ghost on White
    signal c1_cu, c1_cv : signed(8 downto 0) := (others => '0'); -- chroma x key3

    ----------------------------------------------------------------------
    -- luma side pipeline
    ----------------------------------------------------------------------
    -- Raw input luma is tapped EARLY (y_sr 0..4 only): carrying raw video
    -- 12-15 FFs deep went green+noisy on HD HDMI elsewhere (cascade,
    -- rollthebones).  Radius, ink and ghost are computed off the early taps
    -- and only those derived values are delayed to the compose stage.
    type t_ysr is array(0 to 8) of unsigned(9 downto 0);
    signal y_sr : t_ysr := (others => (others => '0'));

    -- 8-px running box (8-bit luma) -> vertical IIR via the line buffer
    signal bx_d   : signed(8 downto 0) := (others => '0');
    signal bx_s   : signed(11 downto 0) := (others => '0');
    signal s_avid_c : std_logic := '0';
    signal s_col  : unsigned(10 downto 0) := (others => '1');  -- saturates at 2047
    signal s_lpar : std_logic := '0';                          -- write-bank parity
    signal s_line0, s_l0arm : std_logic := '0';                -- first line of frame
    type t_lb is array(0 to 4095) of std_logic_vector(7 downto 0);
    signal lb_mem : t_lb;
    signal lb_ra, lb_wa : unsigned(11 downto 0) := (others => '0');
    signal lb_rd, lb_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal lb_rdq : unsigned(7 downto 0) := (others => '0');
    signal vi     : unsigned(7 downto 0) := (others => '0');   -- blurred luma (#V1)

    signal lp1_p  : unsigned(16 downto 0) := (others => '0');  -- vi*gain
    signal lp2_l  : unsigned(7 downto 0) := (others => '0');   -- dot radius Q7ish
    type t_l8 is array(0 to 5) of unsigned(7 downto 0);
    signal ld_sr  : t_l8 := (others => (others => '0'));       -- radius delay
    signal lp3_rsq : unsigned(15 downto 0) := (others => '0'); -- radius^2 (#16)

    signal gx_r, gy_r : signed(9 downto 0) := (others => '0'); -- h/v gradients
    signal g1_ink : std_logic := '0';                          -- raw edge flag
    signal g1_sr  : std_logic_vector(0 to 4) := (others => '0');
    signal g2_ink : std_logic := '0';                          -- dilated
    signal i_sr   : std_logic_vector(0 to 7) := (others => '0'); -- (.. #17)

    -- source chroma, 5-bit (steps of 32, 512 = exact code 16), as offsets
    type t_c5 is array(0 to 16) of signed(4 downto 0);
    signal cu_sr, cv_sr : t_c5 := (others => (others => '0'));
    type t_g7 is array(0 to 15) of unsigned(6 downto 0);
    signal gh_sr  : t_g7 := (others => (others => '0'));       -- ghost luma/8 (#2..#17)

    ----------------------------------------------------------------------
    -- 1-bit carries
    ----------------------------------------------------------------------
    signal seam_sr : std_logic_vector(0 to 14) := (others => '0'); -- A1 -> C1b
    signal bord_sr : std_logic_vector(0 to 4) := (others => '0');  -- RB -> C1b
    type t_sty is array(0 to 8) of unsigned(1 downto 0);
    signal sty_sr : t_sty := (others => (others => '0'));          -- A1 -> RB
    signal kb_sr  : std_logic_vector(0 to 8) := (others => '0');   -- A1 -> RB

    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u : unsigned(9 downto 0) := C_MID;
    signal s_out_v : unsigned(9 downto 0) := C_MID;

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    -- generated avid lags the source avid by 4 clocks (5 off-parity), so its
    -- delay line is 4 shorter: gav_sr(14) lines up with the old sync tap 18.
    signal gav_sr       : std_logic_vector(0 to C_LATENCY - 5) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- relay the pixel-rate multiply operands (frame-stable)
    ------------------------------------------------------------------------
    p_relay : process(clk)
    begin
        if rising_edge(clk) then
            s_twist_d <= s_twist;
            s_gain_d  <= s_gain;
        end if;
    end process p_relay;

    ------------------------------------------------------------------------
    -- raster measurement (hypnos pattern)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                s_xcnt <= (others => '0');
                s_lcnt <= s_lcnt + 1;
            end if;

            -- +-2 hysteresis: analog line lengths wander by a pixel, and W/2
            -- sets the frame's u origin (1-px whole-frame shimmer).  Same for
            -- H (interlaced fields can differ by a line).  Two-step: capture
            -- the count, then compare against free-running W+-2 / H+-2 --
            -- the fused add+compare was the hd_analog critical path.
            s_wp2 <= s_W + 2;
            s_wm2 <= s_W - 2;
            s_hp2 <= s_H + 2;
            s_hm2 <= s_H - 2;

            s_xl_go <= '0';
            if data_in.avid = '0' and s_avid_q = '1' then
                s_xl    <= s_xcnt;
                s_xl_go <= '1';
            end if;
            if s_xl_go = '1' and s_xl > 16 and (s_xl > s_wp2 or s_xl < s_wm2) then
                s_W <= s_xl;
            end if;

            s_hl_go <= '0';
            -- H in FIELD lines: the per-mode y step (s_sy) already carries
            -- the interlace x2, so (H/2)*s_sy is half the virtual 1080
            if s_vs_pulse = '1' then
                s_hl <= s_lcnt;
                s_hl_go <= '1';
                s_lcnt <= (others => '0');
            end if;
            if s_hl_go = '1' and (s_hl > s_hp2 or s_hl < s_hm2) then
                s_H <= s_hl;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- generated avid (aurora v3.0.2 pattern).  g_ph = hsync->avid-rise
    -- distance parity of this line; g_par = the voted usual parity; a line
    -- whose avid lands off-parity starts one clock later, so the output
    -- avid always starts at the same hsync parity.  It ends after W pixels
    -- (field-stable s_wm1) or at hsync at the latest.
    ------------------------------------------------------------------------
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
                if g_cnt = s_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.  The vsync trigger is
    -- guarded with "active lines seen" so serrated analog vsync edges can't
    -- re-run the sequencer (and double-step the drift) within one blank.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_d   : signed(10 downto 0);
        variable v_e   : signed(10 downto 0);
        variable v_q6  : signed(10 downto 0);
        variable v_sh  : integer range 0 to 7;
        variable v_s   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';

            -- the trigger is REGISTERED (free-running lines-seen flag, then
            -- edge AND flag): v0.3's fused 12-bit compare fanned straight
            -- into every sequencer-slot enable and was the HD critical path
            if s_lcnt > 8 then s_lgt8 <= '1'; else s_lgt8 <= '0'; end if;
            s_vtrig <= (not data_in.vsync_n) and s_prev_vsync and s_lgt8;
            s_mp <= s_ma * s_mb;

            if s_vtrig = '1' then
                s_vs_pulse <= '1';

                s_k_fill  <= unsigned(registers_in(0));
                s_k_gain  <= unsigned(registers_in(1));
                s_k_drift <= unsigned(registers_in(2));
                s_k_ink   <= unsigned(registers_in(3));
                s_k_bord  <= unsigned(registers_in(4));
                s_k_twist <= unsigned(registers_in(5));
                s_zoom_k  <= unsigned(registers_in(7));
                s_ivory   <= registers_in(6)(0);
                s_inv     <= registers_in(6)(1);
                s_colour  <= registers_in(6)(2);
                s_weave   <= registers_in(6)(3);
                s_ghost   <= registers_in(6)(4);

                -- resolution scaling: per-mode virtual steps + interlace,
                -- all from the timing ID (field_n is tied high on hardware)
                s_xs    <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ysq   <= f_vstep_y(registers_in(8)(3 downto 0));
                s_ilace <= f_ilace(registers_in(8)(3 downto 0));

                s_seq <= to_unsigned(24, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- SCALE: s = lace units per VIRTUAL unit (Q8), so
                    -- BIGGER s = SMALLER lace.  s = 2^(1.75*(511-P12)/512):
                    -- P12 up = bigger lace, +-1.75 octaves (s = 861..76;
                    -- exponent Q6 = e/4 - e/32).  Same s in every mode; the
                    -- per-mode steps below turn it into lace per pixel/line.
                    when 24 =>
                        v_e  := to_signed(511, 11) - signed('0' & s_zoom_k);
                        v_q6 := shift_right(v_e, 2) - shift_right(v_e, 5);
                        s_svi <= to_integer(shift_right(v_q6, 6));
                        s_svf <= unsigned(v_q6(5 downto 0));
                    when 23 =>
                        v_sh := 2 - s_svi;
                        v_s := shift_right(C_MANT(to_integer(s_svf)), v_sh);
                        s_scl <= v_s;

                    -- lace per pixel sx = s*xstep (1, 1.5, 2: <= 1722, 11
                    -- bits); lace per field line sy = s*ystep>>8 on the
                    -- shared multiplier (480i 4.44: <= 3827, 12 bits).
                    -- 1080p: both are exactly s.
                    when 22 =>
                        case to_integer(s_xs) is
                            when 4      => s_sx <= shift_left(s_scl, 1);
                            when 3      => s_sx <= s_scl + shift_right(s_scl, 1);
                            when others => s_sx <= s_scl;
                        end case;
                        s_ma <= signed(resize(s_ysq, 13));
                        s_mb <= signed(resize(s_scl, 13));
                    when 20 => s_sy <= unsigned(s_mp(19 downto 8));

                    -- u0 = UOFF - (W/2)*sx   (shared multiplier)
                    when 19 => s_ma <= signed(resize(s_W(11 downto 1), 13));
                               s_mb <= signed(resize(s_sx, 13));
                    when 17 => s_u0 <= C_UOFF - unsigned(s_mp(19 downto 0));

                    -- v0 = VOFF + scroll - (H/2)*sy  (H in field lines)
                    when 16 => s_ma <= signed(resize(s_H(11 downto 1), 13));
                               s_mb <= signed(resize(s_sy, 13));
                    when 14 => s_v0 <= C_VOFF + s_scroll - unsigned(s_mp(19 downto 0));

                    -- DRIFT: scroll += (K3-512), deadbanded (Q8 lace/frame)
                    when 12 =>
                        v_d := signed('0' & s_k_drift) - to_signed(512, 11);
                        if abs(v_d) >= C_DB then
                            s_scroll <= s_scroll + unsigned(resize(v_d, 20));
                        end if;

                    -- SPIN rides the same knob: angle += (K3-512)/4 Q4/frame
                    -- (max ~8/4096 turn per frame), same deadband
                    when 13 =>
                        v_d := signed('0' & s_k_drift) - to_signed(512, 11);
                        if abs(v_d) >= C_DB then
                            s_spin <= s_spin + unsigned(resize(shift_right(v_d, 2), 16));
                        end if;

                    -- +-threshold for the abs-free gradient compare (one
                    -- frame behind s_inkth -- harmless)
                    when 18 =>
                        s_pth <= signed(resize(s_inkth, 10));
                        s_nth <= to_signed(0, 10) - signed(resize(s_inkth, 10));

                    -- K4 at its bottom detent = no ink at all
                    when 15 =>
                        if s_k_ink >= C_DB then s_inkon <= '1';
                        else                    s_inkon <= '0'; end if;

                    -- luma -> dot radius mapping
                    when 11 => s_gain <= s_k_gain(9 downto 1);
                    when 10 => s_bias <= s_k_fill(9 downto 2);

                    -- TWIST: per-ring cell rotation, deadbanded
                    when 9 =>
                        v_d := signed('0' & s_k_twist) - to_signed(512, 11);
                        if abs(v_d) < C_DB then s_twist <= (others => '0');
                        else s_twist <= resize(shift_right(v_d, 2), 8); end if;

                    -- INK threshold (K4 up = more ink)
                    when 8 =>
                        s_inkth <= to_unsigned(8, 8)
                                   + resize(shift_right(to_unsigned(1023, 10) - s_k_ink, 3), 8);

                    -- BORDER weight (Q2) and effective fan edges
                    when 7 => s_bw <= s_k_bord(9 downto 1);
                    when 6 => s_re0 <= to_unsigned(C_REDGE0, 12) - resize(s_bw, 12);
                    when 5 => s_re1 <= to_unsigned(C_REDGE1, 12) - resize(s_bw, 12);
                    when 4 => s_re2 <= to_unsigned(C_REDGE0, 12) - resize(s_bw, 12);
                    when 3 => s_re3 <= to_unsigned(C_REDGE3, 12) - resize(s_bw, 12);

                    -- generated-avid width
                    when 2 => s_wm1 <= s_W - 1;

                    -- +-spin for the counter-rotating bands
                    when 1 =>
                        s_spinp <= s_spin(15 downto 4);
                        s_spinn <= to_unsigned(0, 12) - s_spin(15 downto 4);

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- lace-space DDA.  u loads u0 at each active-line start and steps +sx per
    -- pixel; v loads v0 at the frame's first active line (+sy/2 on the bottom
    -- field when interlaced) and steps +sy per field line (per-mode steps).
    -- Advances on ACTIVE lines only (interlace screen-phase rule).
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_v0 : unsigned(19 downto 0);
        variable v_st : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then      -- active-line start
                -- per-field-line lace step (interlace x2 already inside)
                v_st := resize(s_sy, 20);
                if s_newframe = '1' then
                    -- bottom field (odd frame rows) = field_n '0' starts
                    -- one frame line (half a field step) down.  Inert on
                    -- hardware (field_n tied high), kept for the sim/1080i.
                    if s_ilace = '1' and data_in.field_n = '0' then
                        v_v0 := s_v0 + resize(s_sy(11 downto 1), 20);
                    else
                        v_v0 := s_v0;
                    end if;
                    s_vc <= v_v0;
                    s_vb <= v_v0 + v_st;
                    s_newframe <= '0';
                else
                    s_vc <= s_vb;
                    s_vb <= s_vb + v_st;
                end if;
                s_u <= s_u0;
            elsif data_in.avid = '1' then
                s_u <= s_u + resize(s_sx, 20);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- A1: band split + style decode + local centre coordinates (Q2).
    -- Style from band index k = v[17:16]; Weave B drops fans/blooms.
    -- Half-P stagger on even bands = one XOR on the top local-x bit.
    ------------------------------------------------------------------------
    p_a1 : process(clk)
        variable v_k    : unsigned(1 downto 0);
        variable v_sty  : unsigned(1 downto 0);
        variable v_u9   : unsigned(8 downto 0);
        variable v_u8   : unsigned(7 downto 0);
        variable v_ux10 : unsigned(9 downto 0);
        variable v_ux11 : unsigned(10 downto 0);
        variable v_dx   : signed(12 downto 0);
        variable v_dep  : signed(12 downto 0);
        variable v_seam : integer;
        variable v_adx  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_k := s_vc(17 downto 16);
            if s_weave = '1' then
                if v_k(0) = '1' then v_sty := "10"; else v_sty := "00"; end if;
            else
                v_sty := v_k;
            end if;

            if v_sty = "11" then                                  -- BLOOM, P=256
                v_u8 := s_u(15 downto 8);
                v_u8(7) := v_u8(7) xor (not v_k(0));
                v_ux10 := v_u8 & s_u(7 downto 6);
                v_dx := signed(resize(v_ux10, 13)) - to_signed(512, 13);
                v_dep := C_DEPTH3;
                v_seam := C_SEAM8;
            else                                                  -- P=512
                v_u9 := s_u(16 downto 8);
                v_u9(8) := v_u9(8) xor (not v_k(0));
                v_ux11 := v_u9 & s_u(7 downto 6);
                v_dx := signed(resize(v_ux11, 13)) - to_signed(1024, 13);
                if v_sty = "01" then v_dep := C_DEPTH1;
                else                 v_dep := C_DEPTH0; end if;
                v_seam := C_SEAM9;
            end if;

            a1_dx  <= v_dx;
            a1_dy  <= signed(resize(s_vc(15 downto 6), 13)) - v_dep;
            a1_sty <= v_sty;
            a1_kb  <= v_k(0);

            if v_dx < 0 then v_adx := -v_dx; else v_adx := v_dx; end if;
            if to_integer(v_adx) > v_seam then a1_seam <= '1';
            else                               a1_seam <= '0'; end if;
        end if;
    end process p_a1;

    ------------------------------------------------------------------------
    -- A2: |dx|,|dy| + signs      A3: max/min swap
    ------------------------------------------------------------------------
    -- |dy| can be exactly 2048 (needs all 12 bits): SLICE the negated value,
    -- never resize it (the signed-resize-on-abs trap).
    p_a2 : process(clk)
        variable v_nx, v_ny : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if a1_dx(12) = '1' then v_nx := -a1_dx; else v_nx := a1_dx; end if;
            if a1_dy(12) = '1' then v_ny := -a1_dy; else v_ny := a1_dy; end if;
            a2_a <= unsigned(v_nx(11 downto 0));
            a2_b <= unsigned(v_ny(11 downto 0));
            a2_dxs <= a1_dx(12);
            a2_dys <= a1_dy(12);
        end if;
    end process p_a2;

    p_a3 : process(clk)
    begin
        if rising_edge(clk) then
            if a2_a >= a2_b then a3_hi <= a2_a; a3_lo <= a2_b; a3_swap <= '0';
            else                 a3_hi <= a2_b; a3_lo <= a2_a; a3_swap <= '1'; end if;
            a3_dxs <= a2_dxs;
            a3_dys <= a2_dys;
        end if;
    end process p_a3;

    ------------------------------------------------------------------------
    -- CORDIC vectoring: 10 iterations, 2 per stage, 5 guard bits.
    ------------------------------------------------------------------------
    p_cordic : process(clk)
        variable x0, y0, x1, y1 : signed(18 downto 0);
        variable z0, z1         : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            cordx(0)   <= shift_left(signed(resize(a3_hi, 19)), C_CG);
            cordy(0)   <= shift_left(signed(resize(a3_lo, 19)), C_CG);
            cordz(0)   <= (others => '0');
            cordsw(0)  <= a3_swap;
            corddxs(0) <= a3_dxs;
            corddys(0) <= a3_dys;
            for s in 0 to C_NS - 1 loop
                if cordy(s) >= 0 then
                    x0 := cordx(s) + shift_right(cordy(s), 2 * s);
                    y0 := cordy(s) - shift_right(cordx(s), 2 * s);
                    z0 := cordz(s) + to_signed(C_ATAN(2 * s), 13);
                else
                    x0 := cordx(s) - shift_right(cordy(s), 2 * s);
                    y0 := cordy(s) + shift_right(cordx(s), 2 * s);
                    z0 := cordz(s) - to_signed(C_ATAN(2 * s), 13);
                end if;
                if y0 >= 0 then
                    x1 := x0 + shift_right(y0, 2 * s + 1);
                    y1 := y0 - shift_right(x0, 2 * s + 1);
                    z1 := z0 + to_signed(C_ATAN(2 * s + 1), 13);
                else
                    x1 := x0 - shift_right(y0, 2 * s + 1);
                    y1 := y0 + shift_right(x0, 2 * s + 1);
                    z1 := z0 - to_signed(C_ATAN(2 * s + 1), 13);
                end if;
                cordx(s + 1)   <= x1;
                cordy(s + 1)   <= y1;
                cordz(s + 1)   <= z1;
                cordsw(s + 1)  <= cordsw(s);
                corddxs(s + 1) <= corddxs(s);
                corddys(s + 1) <= corddys(s);
            end loop;
        end if;
    end process p_cordic;

    ------------------------------------------------------------------------
    -- RU: undo CORDIC gain+guard (r = x*312 >> 14, Q2) and fold the angle
    -- out of the first octant into a full polar angle (4096 = full circle).
    ------------------------------------------------------------------------
    p_ru : process(clk)
        variable vc   : unsigned(18 downto 0);
        variable vp   : unsigned(26 downto 0);
        variable vmag : signed(12 downto 0);
        variable vang : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            vc := unsigned(cordx(C_NS));
            vp := resize(shift_left(resize(vc, 27), 8), 27)
                  + resize(shift_left(resize(vc, 27), 5), 27)
                  + resize(shift_left(resize(vc, 27), 4), 27)
                  + resize(shift_left(resize(vc, 27), 3), 27);
            ru_r <= resize(shift_right(vp, 14), 12);

            if cordsw(C_NS) = '1' then vmag := to_signed(1024, 13) - cordz(C_NS);
            else                       vmag := cordz(C_NS); end if;
            if corddxs(C_NS) = '0' and corddys(C_NS) = '0' then    vang := vmag;
            elsif corddxs(C_NS) = '1' and corddys(C_NS) = '0' then vang := to_signed(2048, 13) - vmag;
            elsif corddxs(C_NS) = '1' and corddys(C_NS) = '1' then vang := to_signed(2048, 13) + vmag;
            else                                                   vang := -vmag; end if;
            ru_ang <= vang;
        end if;
    end process p_ru;

    ------------------------------------------------------------------------
    -- RB: ring split at the style's pitch, radial cell fraction (Q7),
    -- ring-0 centre dot, and the scalloped border test (r > R_edge - K5).
    ------------------------------------------------------------------------
    p_rb : process(clk)
        variable v_sty  : unsigned(1 downto 0);
        variable v_ring : unsigned(5 downto 0);
        variable v_fr8  : unsigned(7 downto 0);
        variable v_re   : unsigned(11 downto 0);
        variable v_a12  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_sty := sty_sr(8);

            if v_sty(1) = '1' then                       -- SEED/BLOOM, G=16
                v_ring := ru_r(11 downto 6);
                v_fr8  := ru_r(5 downto 0) & "00";
            else                                         -- PEBBLE/FAN, G=32
                v_ring := '0' & ru_r(11 downto 7);
                v_fr8  := ru_r(6 downto 0) & '0';
            end if;

            rb_ring <= v_ring;
            if v_ring = 0 then
                rb_rz <= '1';
                -- centre dot: distance measured straight from r (always +ve)
                rb_fr <= signed(resize(v_fr8(7 downto 1), 8));
            else
                rb_rz <= '0';
                rb_fr <= signed(v_fr8 - to_unsigned(128, 8));
            end if;

            case v_sty is
                when "00"   => v_re := s_re0;
                when "01"   => v_re := s_re1;
                when "10"   => v_re := s_re2;
                when others => v_re := s_re3;
            end case;
            if ru_r > v_re then rb_bord <= '1'; else rb_bord <= '0'; end if;

            -- spin: one angle add before theta*ring; mod 4096 keeps the
            -- 6*ring cell wrap seamless.  Odd bands turn the other way.
            if kb_sr(8) = '1' then v_a12 := unsigned(ru_ang(11 downto 0)) + s_spinp;
            else                   v_a12 := unsigned(ru_ang(11 downto 0)) + s_spinn; end if;
            rb_ang <= signed('0' & v_a12);
        end if;
    end process p_rb;

    ------------------------------------------------------------------------
    -- T1: the two narrow multiplies -- theta*ring and ring*twist
    ------------------------------------------------------------------------
    p_t1 : process(clk)
    begin
        if rising_edge(clk) then
            t1_m  <= rb_ang * signed(resize(rb_ring, 7));
            t1_tw <= signed(resize(rb_ring, 7)) * s_twist_d;
            t1_fr <= rb_fr;
            t1_rz <= rb_rz;
        end if;
    end process p_t1;

    ------------------------------------------------------------------------
    -- T2: cells around the ring = 6*ring, so cell coordinate = theta*ring*6
    -- / 4096.  The mod-4096 slice makes the wrap seamless; twist shifts each
    -- ring's dots by ring*K6 (phyllotaxis shear).
    ------------------------------------------------------------------------
    p_t2 : process(clk)
        variable v_m6 : signed(22 downto 0);
        variable v_mt : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            v_m6 := shift_left(resize(t1_m, 23), 2) + shift_left(resize(t1_m, 23), 1);
            v_mt := v_m6 + shift_left(resize(t1_tw, 23), 5);
            if t1_rz = '1' then
                t2_ft <= (others => '0');
            else
                t2_ft <= signed(unsigned(v_mt(11 downto 4)) - to_unsigned(128, 8));
            end if;
            t2_fr <= t1_fr;
        end if;
    end process p_t2;

    ------------------------------------------------------------------------
    -- D1/D2: normalized distance^2 = fr^2 + ft^2  (Q14, 16384 = half-cell^2)
    ------------------------------------------------------------------------
    p_d1 : process(clk)
        variable vp, vq : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            vp := t2_fr * t2_fr;
            vq := t2_ft * t2_ft;
            d1_fp2 <= unsigned(vp(14 downto 0));
            d1_ft2 <= unsigned(vq(14 downto 0));
        end if;
    end process p_d1;

    p_d2 : process(clk)
    begin
        if rising_edge(clk) then
            d2_d2 <= resize(d1_fp2, 16) + resize(d1_ft2, 16);
        end if;
    end process p_d2;

    ------------------------------------------------------------------------
    -- luma side.  Timing labels: y_sr(k) at cycle c holds input pixel
    -- d(c-1-k).  The box/IIR/ink values at the V1 stage are centred on
    -- d(c-6); radius, ink and ghost are then delayed to land at C1.
    --
    -- Line buffer (one 4096x8 memory = 8 EBR, ONE write, ONE read): bank =
    -- line parity, so a line reads the other bank (previous line) and writes
    -- its own -- no same-address read/write.  Column label L(c) = s_col(c)-5
    -- for the value at the V1 stage; the read is issued 3 cycles ahead
    -- (s_col-2 -> EBR -> fabric register).  s_col saturates at 2047 so the
    -- blanking tail writes a single dead label instead of wrapping onto the
    -- start of this line's bank.
    ------------------------------------------------------------------------
    p_col : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_c <= data_in.avid;
            if data_in.avid = '1' and s_avid_c = '0' then       -- line start
                s_col   <= (others => '0');
                s_lpar  <= not s_lpar;
                s_line0 <= s_l0arm;
                s_l0arm <= '0';
            elsif s_col /= 2047 then
                s_col <= s_col + 1;
            end if;
            if s_vs_pulse = '1' then
                s_l0arm <= '1';
            end if;
        end if;
    end process p_col;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            lb_mem(to_integer(lb_wa)) <= lb_wd;
            lb_rd <= lb_mem(to_integer(lb_ra));
        end if;
    end process p_lb;

    p_luma : process(clk)
        variable v_li : integer;
        variable v_h, v_p, v_n : unsigned(7 downto 0);
        variable v_dd, v_t : signed(9 downto 0);
        variable v_ga, v_gb : unsigned(7 downto 0);
        variable v_e  : std_logic;
        variable v_qu, v_qv : unsigned(10 downto 0);
        variable v_u5, v_v5 : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            y_sr(0) <= unsigned(data_in.y);
            for i in 1 to 8 loop
                y_sr(i) <= y_sr(i - 1);
            end loop;

            -- 8-px running box (exact: add the new sample, drop the 8th)
            bx_d <= signed(resize(y_sr(0)(9 downto 2), 9)) - signed(resize(y_sr(8)(9 downto 2), 9));
            bx_s <= bx_s + resize(bx_d, 12);

            -- line-buffer read address (3 cycles ahead of label L)
            lb_ra  <= (not s_lpar) & (s_col - 2);
            lb_rdq <= unsigned(lb_rd);

            -- V1: vertical one-pole IIR, new = above + (box - above)/4.  The
            -- first line of a frame takes the box straight (no stale rows).
            v_h  := unsigned(bx_s(10 downto 3));
            v_p  := lb_rdq;
            v_dd := signed(resize(v_h, 10)) - signed(resize(v_p, 10));
            if s_line0 = '1' then
                v_n := v_h;
            else
                -- result is 0..255 (between above and box): SLICE, never
                -- resize a signed value down (sign + 7 bits would fold it)
                v_t := signed(resize(v_p, 10)) + shift_right(v_dd, 2);
                v_n := unsigned(v_t(7 downto 0));
            end if;
            vi    <= v_n;
            lb_wd <= std_logic_vector(v_n);
            lb_wa <= s_lpar & (s_col - 5);

            -- dot radius: l = clamp(bias + vi*gain/256, 0, 250); the 8-bit
            -- radius rides 6 stages to reach the square at #16.
            lp1_p <= vi * s_gain_d;
            v_li  := to_integer(s_bias) + to_integer(lp1_p(16 downto 8));
            if v_li > 250 then v_li := 250; end if;
            lp2_l <= to_unsigned(v_li, 8);
            ld_sr(0) <= lp2_l;
            for i in 1 to 5 loop
                ld_sr(i) <= ld_sr(i - 1);
            end loop;
            lp3_rsq <= ld_sr(5) * ld_sr(5);

            -- ink: horizontal gradient (4-px baseline, centred on d(c-6))
            -- OR vertical gradient (box vs the IIR of the rows above), then
            -- a 6-px dilate; i_sr(6) lands the stroke centred at C1.  Two
            -- stages: register the signed differences, then compare each
            -- against +th and -th in parallel (no abs) -- the fused
            -- EBR -> subtract -> abs -> compare was the HD HDMI critical path.
            v_ga := y_sr(3)(9 downto 2);
            v_gb := y_sr(7)(9 downto 2);
            gx_r <= signed(resize(v_ga, 10)) - signed(resize(v_gb, 10));
            if s_line0 = '1' then gy_r <= (others => '0');
            else                  gy_r <= v_dd; end if;
            v_e := '0';
            if gx_r > s_pth or gx_r < s_nth then v_e := '1'; end if;
            if gy_r > s_pth or gy_r < s_nth then v_e := '1'; end if;
            g1_ink <= v_e;
            g1_sr <= g1_ink & g1_sr(0 to 3);
            g2_ink <= g1_ink or g1_sr(0) or g1_sr(1) or g1_sr(2) or g1_sr(3) or g1_sr(4);
            i_sr(0) <= g2_ink;
            for i in 1 to 7 loop
                i_sr(i) <= i_sr(i - 1);
            end loop;

            -- ghost: dim video for the ground (luma/8, 7 bits), from the
            -- first tap; gh_sr(15) matches v0.1's y_sr(14) + 2 registers.
            gh_sr(0) <= y_sr(0)(9 downto 3);
            for i in 1 to 15 loop
                gh_sr(i) <= gh_sr(i - 1);
            end loop;

            -- colour: source chroma rounded to 5 bits (steps of 32; 512 is
            -- exactly code 16, so grey stays neutral), stored as an offset
            -- (MSB flip = minus 16).  Input and output are both in the
            -- hardware's U/V order -- a dry path, so no swap.
            v_qu := resize(unsigned(data_in.u), 11) + 16;
            v_qv := resize(unsigned(data_in.v), 11) + 16;
            if v_qu(10) = '1' then v_u5 := (others => '1'); else v_u5 := v_qu(9 downto 5); end if;
            if v_qv(10) = '1' then v_v5 := (others => '1'); else v_v5 := v_qv(9 downto 5); end if;
            cu_sr(0) <= signed((not v_u5(4)) & v_u5(3 downto 0));
            cv_sr(0) <= signed((not v_v5(4)) & v_v5(3 downto 0));
            for i in 1 to 16 loop
                cu_sr(i) <= cu_sr(i - 1);
                cv_sr(i) <= cv_sr(i - 1);
            end loop;
        end if;
    end process p_luma;

    ------------------------------------------------------------------------
    -- 1-bit / 2-bit carries
    ------------------------------------------------------------------------
    p_carry : process(clk)
    begin
        if rising_edge(clk) then
            seam_sr(0) <= a1_seam;
            for i in 1 to 14 loop
                seam_sr(i) <= seam_sr(i - 1);
            end loop;
            bord_sr(0) <= rb_bord;
            for i in 1 to 4 loop
                bord_sr(i) <= bord_sr(i - 1);
            end loop;
            sty_sr(0) <= a1_sty;
            kb_sr(0)  <= a1_kb;
            for i in 1 to 8 loop
                sty_sr(i) <= sty_sr(i - 1);
                kb_sr(i)  <= kb_sr(i - 1);
            end loop;
        end if;
    end process p_carry;

    ------------------------------------------------------------------------
    -- C1a: dot key with a soft edge (subtract + clamp ONLY -- keeping the
    -- compose muxes out of this cone was the hd_hdmi critical path).
    ------------------------------------------------------------------------
    p_c1a : process(clk)
        variable v_df : signed(17 downto 0);
        variable v_k  : integer;
    begin
        if rising_edge(clk) then
            v_df := signed(resize(lp3_rsq, 18)) - signed(resize(d2_d2, 18));
            v_k  := 128 + to_integer(shift_right(v_df, 3));
            if v_k < 0 then v_k := 0; end if;
            if v_k > 255 then v_k := 255; end if;
            c1a_key <= to_unsigned(v_k, 8);
        end if;
    end process p_c1a;

    ------------------------------------------------------------------------
    -- C1b: lace composition -- invert the figure/ground if asked, then cut
    -- every black line (border scallops, seam wisps, ink strokes) on top.
    ------------------------------------------------------------------------
    p_c1b : process(clk)
        variable v_k   : unsigned(7 downto 0);
        variable v_kc  : unsigned(7 downto 0);
        variable v_cut : std_logic;
    begin
        if rising_edge(clk) then
            v_k := c1a_key;
            if s_inv = '1' then v_k := not v_k; end if;
            v_kc := v_k;

            v_cut := bord_sr(4) or seam_sr(14);
            if s_inkon = '1' then v_cut := v_cut or i_sr(6); end if;
            if v_cut = '1' then v_k := (others => '0'); end if;

            c1_key <= v_k;
            c1_gh  <= gh_sr(15);

            -- colour: source chroma x 3-bit key (cut lines stay neutral)
            if v_cut = '1' then
                c1_cu <= (others => '0');
                c1_cv <= (others => '0');
            else
                c1_cu <= cu_sr(16) * signed('0' & v_kc(7 downto 5));
                c1_cv <= cv_sr(16) * signed('0' & v_kc(7 downto 5));
            end if;

            -- ghost: Black ground = dim video in the ground and the cuts;
            -- White ground = bright video in the (white) ground, cuts stay black
            c1_gw <= s_inv;
            if s_ghost = '1' and s_inv = '0' and (v_cut = '1' or c1a_key = 0) then
                c1_useg <= '1';
            elsif s_ghost = '1' and s_inv = '1' and v_cut = '0' and c1a_key = 0 then
                c1_useg <= '1';
            else
                c1_useg <= '0';
            end if;
        end if;
    end process p_c1b;

    ------------------------------------------------------------------------
    -- C2: colour map (cream on black) + blanking gate
    ------------------------------------------------------------------------
    p_c2 : process(clk)
        variable v_y : unsigned(10 downto 0);   -- 11-bit: 64+1020 overflows 10
        variable v_cu, v_cv : signed(8 downto 0);
        variable v_ou, v_ov : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            if gav_sr(C_LATENCY - 6) = '0' then              -- generated avid
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            elsif c1_useg = '1' then
                if c1_gw = '1' then               -- 384..765: light video
                    s_out_y <= to_unsigned(384, 10) + resize(c1_gh, 10)
                               + shift_left(resize(c1_gh, 10), 1);
                else                              -- 64..191: dim video
                    s_out_y <= to_unsigned(64, 10) + resize(c1_gh, 10);
                end if;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                -- Y = 64 + key*2.75 -> 64..766 (cap ~768: full-scale Y
                -- clips every RGB channel and kills the ivory chroma)
                v_y := to_unsigned(64, 11)
                       + shift_left(resize(c1_key, 11), 1)
                       + resize(c1_key, 11)
                       - resize(c1_key(7 downto 2), 11);
                s_out_y <= v_y(9 downto 0);

                -- chroma offset = colour (source chroma x key, clamped to
                -- +-56 steps of 8 = +-448) + ivory.  Max |offset| 511, so
                -- 512 + offset is just the offset with bit 9 flipped.
                v_ou := (others => '0');
                v_ov := (others => '0');
                if s_colour = '1' then
                    v_cu := c1_cu;
                    v_cv := c1_cv;
                    if v_cu > 56 then v_cu := to_signed(56, 9); end if;
                    if v_cu < -56 then v_cu := to_signed(-56, 9); end if;
                    if v_cv > 56 then v_cv := to_signed(56, 9); end if;
                    if v_cv < -56 then v_cv := to_signed(-56, 9); end if;
                    v_ou := shift_left(resize(v_cu, 10), 3);
                    v_ov := shift_left(resize(v_cv, 10), 3);
                end if;
                if s_ivory = '1' then
                    -- ivory = BT.601 U -key/4 (up to -63), V +key/8 (+31),
                    -- U/V-SWAPPED here at the output compose (hardware)
                    v_ou := v_ou + signed(resize(c1_key(7 downto 3), 10));
                    v_ov := v_ov - signed(resize(c1_key(7 downto 2), 10));
                end if;
                s_out_u <= unsigned((not v_ou(9)) & std_logic_vector(v_ou(8 downto 0)));
                s_out_v <= unsigned((not v_ov(9)) & std_logic_vector(v_ov(8 downto 0)));
            end if;
        end if;
    end process p_c2;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            gav_sr       <= g_avid          & gav_sr      (0 to C_LATENCY - 6);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    s_io_align_in.y       <= std_logic_vector(s_out_y);
    s_io_align_in.u       <= std_logic_vector(s_out_u);
    s_io_align_in.v       <= std_logic_vector(s_out_v);
    s_io_align_in.avid    <= gav_sr(C_LATENCY - 5);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_LATENCY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (19 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture chantilly;
