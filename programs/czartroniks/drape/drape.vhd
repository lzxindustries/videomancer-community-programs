-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Drape v1.3: captures the scanline at a split point and drapes it downward
-- like hanging fabric, fanning out from centre, leaning, swaying and fading
-- to black.
--
-- Concept:
--   Rows above the Split show the input untouched. The Split row itself is
--   written into a line buffer. Every row below replays that line through a
--   perspective scale about the centre column:
--     src_x = base + (x - centre) * s,   s = 1 / (1 + e/256)        (Fan)
--                                        s = triangle(e), +1..-1    (Pleat)
--   where e grows with depth below the Stretch line and with P12 Spread.
--   Fan gives columns that are straight lines converging on a vanishing
--   point; Pleat folds the line back and forth -- columns fan out, pinch to
--   a point, flip and return, repeatedly. s is signed 4096 = 1.0; the Fan
--   reciprocal comes from a 13-step serial divider that runs in each hblank.
--   base = centre + lean * dy, lean = Twist + Sway wave (+ blur group-delay
--   compensation). Out-of-range reads are REFLECTED at the picture edges.
--
-- Chroma: the encoder decides Cb vs Cr by counting from hsync, but the
-- program's U/V labels follow avid, and the hsync->avid offset parity can
-- differ between the captured row and a display row. Replaying one row's
-- U/V on every row swapped Cb/Cr on mismatched rows (v1.1: green/magenta
-- cast on hardware). Each pixel keeps its own U/V pair (coherent, so no
-- edge fringing -- v1.2's parity-forced single chroma read mixed Cb/Cr
-- from different pixels = dark lines at colour edges) and U/V are
-- SWAPPED on rows whose hsync->avid parity differs from the capture's.
--
-- Rows are counted on ACTIVE lines only (advance at avid fall, reset at
-- vsync, height captured on the increment branch). Per-row values are
-- recomputed in the hblank after each avid fall (~45 clocks) and sampled
-- once per pixel at stage 2.
--
-- v1.3.2 resolution scaling: rows are VIRTUAL 1080-line units (a Q.8 row
-- accumulator stepped per active line from the timing ID), so the split,
-- stretch, fade, onset, sway wavelength and spread depth are the same
-- fractions of the screen height at SD / 720p / 1080p (1080p bit-
-- identical). The lean (Twist + Sway) depth is divided by the x step so
-- the lean angle matches too. Pixel-scale: blur, its 27 px compensation.
--
-- Pixel pipeline (LATENCY = 10, identical in every mode):
--   s1   register counters; blur IIRs on data_in
--   s2   dx = x - centre; sample per-row values
--   s3   dx * s split into hi/lo partial products
--   s3a  partial-product sum
--   s3b  rd = base + (dx*s >> 12)
--   s3c  edge reflection; capture write data/addr/enable
--   s4   BRAM write + read (EBR output register)
--   s4b  fabric register on BRAM data
--   s6   fade toward black (Y 64, U/V 512)
--   s7   blanking gate / passthrough+bypass / drape (U/V row swap)
--
-- Register map:
--   registers_in(0)    K1 Split   capture row, 0..H-1 (deadbanded, per field)
--   registers_in(1)    K2 Stretch where the fan begins, as a fraction of the
--                                 distance from the split to the bottom
--   registers_in(2)    K3 Fade    fade length below Stretch; 100% = no fade
--   registers_in(3)    K4 Twist   lean, centred at 512, +/-0.5 px per row
--   registers_in(4)    K5 Onset   0 = fan starts with a kink, up = eases in
--                                 over up to ~480 rows
--   registers_in(5)    K6 Sway    travelling-wave sway, amplitude grows with
--                                 depth; speed rises with the knob; the
--                                 bottom 1/16 of travel is a still zone
--   registers_in(6)(0) S7  Blur   one-pole IIR on the captured line (source
--                                 space, so it widens with the fan)
--   registers_in(6)(1) S8  Shape  Fan / Pleat
--   registers_in(6)(2) S9  Freeze stop capturing: the held line keeps
--                                 draping while Split moves
--   registers_in(6)(4) S11 Bypass latency-matched passthrough
--   registers_in(7)    P12 Spread fan strength; two-slope curve, 100% runs
--                                 the drape into its vanishing point
--
-- BRAM: 3 x 2048x10 line buffers (Y/U/V, 15 EBR) + 256x8 sine ROM.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture drape of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (10 + 2 = 12 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY     : integer := 10;
    constant C_ADDR_BITS : integer := 11;
    constant C_BLUR_COMP : integer := 27;  -- one-pole IIR group delay (px)
    constant C_Y_BLACK   : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_UV_ZERO   : unsigned(9 downto 0) := to_unsigned(512, 10);


    -- Fade slope, quarter-alpha per row: rows-to-black = 4 + 2*i^2
    -- (i = K3 top 5 bits); the last entry is 0 = no fade.
    type t_u10_32 is array(0 to 31) of unsigned(9 downto 0);
    constant FADE_SLOPE_LUT : t_u10_32 := (
        to_unsigned(1023, 10), to_unsigned( 682, 10), to_unsigned( 341, 10), to_unsigned( 186, 10),
        to_unsigned( 114, 10), to_unsigned(  76, 10), to_unsigned(  54, 10), to_unsigned(  40, 10),
        to_unsigned(  31, 10), to_unsigned(  25, 10), to_unsigned(  20, 10), to_unsigned(  17, 10),
        to_unsigned(  14, 10), to_unsigned(  12, 10), to_unsigned(  10, 10), to_unsigned(   9, 10),
        to_unsigned(   8, 10), to_unsigned(   7, 10), to_unsigned(   6, 10), to_unsigned(   6, 10),
        to_unsigned(   5, 10), to_unsigned(   5, 10), to_unsigned(   4, 10), to_unsigned(   4, 10),
        to_unsigned(   4, 10), to_unsigned(   3, 10), to_unsigned(   3, 10), to_unsigned(   3, 10),
        to_unsigned(   3, 10), to_unsigned(   2, 10), to_unsigned(   2, 10), to_unsigned(   0, 10)
    );

    -- Onset slope, gain per row (1023 = full): ramp width = 1 + i^2/2 rows.
    constant ONSET_SLOPE_LUT : t_u10_32 := (
        to_unsigned(1023, 10), to_unsigned( 683, 10), to_unsigned( 341, 10), to_unsigned( 186, 10),
        to_unsigned( 114, 10), to_unsigned(  76, 10), to_unsigned(  54, 10), to_unsigned(  40, 10),
        to_unsigned(  31, 10), to_unsigned(  25, 10), to_unsigned(  20, 10), to_unsigned(  17, 10),
        to_unsigned(  14, 10), to_unsigned(  12, 10), to_unsigned(  10, 10), to_unsigned(   9, 10),
        to_unsigned(   8, 10), to_unsigned(   7, 10), to_unsigned(   6, 10), to_unsigned(   6, 10),
        to_unsigned(   5, 10), to_unsigned(   5, 10), to_unsigned(   4, 10), to_unsigned(   4, 10),
        to_unsigned(   4, 10), to_unsigned(   3, 10), to_unsigned(   3, 10), to_unsigned(   3, 10),
        to_unsigned(   3, 10), to_unsigned(   2, 10), to_unsigned(   2, 10), to_unsigned(   2, 10)
    );

    type t_sin_lut is array(0 to 255) of signed(7 downto 0);
    constant SIN_LUT : t_sin_lut := (
        to_signed(   0, 8), to_signed(   3, 8), to_signed(   6, 8), to_signed(   9, 8), to_signed(  12, 8), to_signed(  16, 8), to_signed(  19, 8), to_signed(  22, 8),
        to_signed(  25, 8), to_signed(  28, 8), to_signed(  31, 8), to_signed(  34, 8), to_signed(  37, 8), to_signed(  40, 8), to_signed(  43, 8), to_signed(  46, 8),
        to_signed(  49, 8), to_signed(  51, 8), to_signed(  54, 8), to_signed(  57, 8), to_signed(  60, 8), to_signed(  63, 8), to_signed(  65, 8), to_signed(  68, 8),
        to_signed(  71, 8), to_signed(  73, 8), to_signed(  76, 8), to_signed(  78, 8), to_signed(  81, 8), to_signed(  83, 8), to_signed(  85, 8), to_signed(  88, 8),
        to_signed(  90, 8), to_signed(  92, 8), to_signed(  94, 8), to_signed(  96, 8), to_signed(  98, 8), to_signed( 100, 8), to_signed( 102, 8), to_signed( 104, 8),
        to_signed( 106, 8), to_signed( 107, 8), to_signed( 109, 8), to_signed( 111, 8), to_signed( 112, 8), to_signed( 113, 8), to_signed( 115, 8), to_signed( 116, 8),
        to_signed( 117, 8), to_signed( 118, 8), to_signed( 120, 8), to_signed( 121, 8), to_signed( 122, 8), to_signed( 122, 8), to_signed( 123, 8), to_signed( 124, 8),
        to_signed( 125, 8), to_signed( 125, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8),
        to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 125, 8),
        to_signed( 125, 8), to_signed( 124, 8), to_signed( 123, 8), to_signed( 122, 8), to_signed( 122, 8), to_signed( 121, 8), to_signed( 120, 8), to_signed( 118, 8),
        to_signed( 117, 8), to_signed( 116, 8), to_signed( 115, 8), to_signed( 113, 8), to_signed( 112, 8), to_signed( 111, 8), to_signed( 109, 8), to_signed( 107, 8),
        to_signed( 106, 8), to_signed( 104, 8), to_signed( 102, 8), to_signed( 100, 8), to_signed(  98, 8), to_signed(  96, 8), to_signed(  94, 8), to_signed(  92, 8),
        to_signed(  90, 8), to_signed(  88, 8), to_signed(  85, 8), to_signed(  83, 8), to_signed(  81, 8), to_signed(  78, 8), to_signed(  76, 8), to_signed(  73, 8),
        to_signed(  71, 8), to_signed(  68, 8), to_signed(  65, 8), to_signed(  63, 8), to_signed(  60, 8), to_signed(  57, 8), to_signed(  54, 8), to_signed(  51, 8),
        to_signed(  49, 8), to_signed(  46, 8), to_signed(  43, 8), to_signed(  40, 8), to_signed(  37, 8), to_signed(  34, 8), to_signed(  31, 8), to_signed(  28, 8),
        to_signed(  25, 8), to_signed(  22, 8), to_signed(  19, 8), to_signed(  16, 8), to_signed(  12, 8), to_signed(   9, 8), to_signed(   6, 8), to_signed(   3, 8),
        to_signed(   0, 8), to_signed(  -3, 8), to_signed(  -6, 8), to_signed(  -9, 8), to_signed( -12, 8), to_signed( -16, 8), to_signed( -19, 8), to_signed( -22, 8),
        to_signed( -25, 8), to_signed( -28, 8), to_signed( -31, 8), to_signed( -34, 8), to_signed( -37, 8), to_signed( -40, 8), to_signed( -43, 8), to_signed( -46, 8),
        to_signed( -49, 8), to_signed( -51, 8), to_signed( -54, 8), to_signed( -57, 8), to_signed( -60, 8), to_signed( -63, 8), to_signed( -65, 8), to_signed( -68, 8),
        to_signed( -71, 8), to_signed( -73, 8), to_signed( -76, 8), to_signed( -78, 8), to_signed( -81, 8), to_signed( -83, 8), to_signed( -85, 8), to_signed( -88, 8),
        to_signed( -90, 8), to_signed( -92, 8), to_signed( -94, 8), to_signed( -96, 8), to_signed( -98, 8), to_signed(-100, 8), to_signed(-102, 8), to_signed(-104, 8),
        to_signed(-106, 8), to_signed(-107, 8), to_signed(-109, 8), to_signed(-111, 8), to_signed(-112, 8), to_signed(-113, 8), to_signed(-115, 8), to_signed(-116, 8),
        to_signed(-117, 8), to_signed(-118, 8), to_signed(-120, 8), to_signed(-121, 8), to_signed(-122, 8), to_signed(-122, 8), to_signed(-123, 8), to_signed(-124, 8),
        to_signed(-125, 8), to_signed(-125, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8),
        to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-125, 8),
        to_signed(-125, 8), to_signed(-124, 8), to_signed(-123, 8), to_signed(-122, 8), to_signed(-122, 8), to_signed(-121, 8), to_signed(-120, 8), to_signed(-118, 8),
        to_signed(-117, 8), to_signed(-116, 8), to_signed(-115, 8), to_signed(-113, 8), to_signed(-112, 8), to_signed(-111, 8), to_signed(-109, 8), to_signed(-107, 8),
        to_signed(-106, 8), to_signed(-104, 8), to_signed(-102, 8), to_signed(-100, 8), to_signed( -98, 8), to_signed( -96, 8), to_signed( -94, 8), to_signed( -92, 8),
        to_signed( -90, 8), to_signed( -88, 8), to_signed( -85, 8), to_signed( -83, 8), to_signed( -81, 8), to_signed( -78, 8), to_signed( -76, 8), to_signed( -73, 8),
        to_signed( -71, 8), to_signed( -68, 8), to_signed( -65, 8), to_signed( -63, 8), to_signed( -60, 8), to_signed( -57, 8), to_signed( -54, 8), to_signed( -51, 8),
        to_signed( -49, 8), to_signed( -46, 8), to_signed( -43, 8), to_signed( -40, 8), to_signed( -37, 8), to_signed( -34, 8), to_signed( -31, 8), to_signed( -28, 8),
        to_signed( -25, 8), to_signed( -22, 8), to_signed( -19, 8), to_signed( -16, 8), to_signed( -12, 8), to_signed(  -9, 8), to_signed(  -6, 8), to_signed(  -3, 8)
    );

    ----------------------------------------------------------------------------
    -- Counters
    ----------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    -- Rows are VIRTUAL 1080-line units (v1.3.2): aline_a / row_ya are Q.8
    -- accumulators stepped by r_ys per active line, so the split, stretch,
    -- fade, onset, sway wavelength and spread depth are fractions of the
    -- screen height in every mode (1080p: step 1.0, bit-identical).
    signal aline_a      : unsigned(19 downto 0) := (others => '0');
    signal h_cnt        : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal max_y_r      : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal row_ya       : unsigned(19 downto 0) := (others => '0');
    signal r_ys         : unsigned(11 downto 0) := to_unsigned(256, 12);  -- Q4.8 per field line
    signal r_xcl        : unsigned(1 downto 0)  := "00";  -- x step 0: 1.0, 1: 1.5 (720p), 2: 2.0 (SD)
    signal rdep         : unsigned(10 downto 0) := (others => '0');  -- real rows below the split
    signal saw_active   : std_logic := '0';
    signal sway_ph      : unsigned(9 downto 0) := (others => '0');
    signal hpar         : std_logic := '0';  -- clock parity since hsync
    signal line_par     : std_logic := '0';  -- hsync->avid parity, this line
    signal cap_par      : std_logic := '0';  -- same, of the captured line
    signal row_swap_r   : std_logic := '0';  -- U/V swap for this display row

    ----------------------------------------------------------------------------
    -- Registered controls + per-frame geometry
    ----------------------------------------------------------------------------
    signal k1_r, k1_held : unsigned(9 downto 0) := to_unsigned(650, 10);
    signal k1_diff       : signed(10 downto 0) := (others => '0');
    signal k1_move       : std_logic := '0';
    signal k2_r, k3_r, k4_r, k5_r, k6_r, k12_r : unsigned(9 downto 0) := (others => '0');
    signal blur_r, pleat_r, freeze_r, bypass_r : std_logic := '0';
    signal cap_blur_r    : std_logic := '0';

    signal split_mul     : unsigned(20 downto 0) := (others => '0');
    signal split_y_r     : unsigned(10 downto 0) := (others => '0');
    signal rem_r         : unsigned(10 downto 0) := (others => '0');
    signal stretch_mul   : unsigned(20 downto 0) := (others => '0');
    signal stretch_off_r : unsigned(10 downto 0) := to_unsigned(8, 11);
    signal centre_r      : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal centre_c_r    : signed(14 downto 0) := to_signed(960, 15);
    signal wm1_r         : signed(14 downto 0) := to_signed(1919, 15);
    signal w2m1_r        : signed(14 downto 0) := to_signed(3839, 15);
    signal fade_slope_r  : unsigned(9 downto 0) := (others => '0');
    signal onset_slope_r : unsigned(9 downto 0) := (others => '0');
    signal twist_t_r     : signed(10 downto 0) := (others => '0');
    signal sway_step_r   : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal sp_lin_r      : unsigned(10 downto 0) := (others => '0');
    signal sp_hi_r       : unsigned(8 downto 0) := (others => '0');
    signal sp_hi13_r     : unsigned(12 downto 0) := (others => '0');
    signal g_r           : unsigned(13 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Per-row pipeline (free-running off row_y; settles within ~45 clocks)
    ----------------------------------------------------------------------------
    signal r1_dysq       : signed(20 downto 0) := (others => '0');  -- Q.8 virtual rows
    signal r2_dy         : unsigned(10 downto 0) := (others => '0');
    signal r2_split      : std_logic := '0';
    signal r2_drape      : std_logic := '0';
    signal r3_dyst       : unsigned(10 downto 0) := (others => '0');
    signal r3_dy         : unsigned(10 downto 0) := (others => '0');
    signal r3_drape      : std_logic := '0';
    signal r3_sidx       : unsigned(7 downto 0) := (others => '0');
    signal r4_sin        : signed(7 downto 0) := (others => '0');
    signal r4_gain_mul   : unsigned(20 downto 0) := (others => '0');
    signal r4_fade_mul   : unsigned(20 downto 0) := (others => '0');
    signal r4_dyst       : unsigned(10 downto 0) := (others => '0');
    signal r4_dy         : unsigned(10 downto 0) := (others => '0');
    signal r4_drape      : std_logic := '0';
    signal r5_sin        : signed(7 downto 0) := (others => '0');
    signal r5_gain       : unsigned(9 downto 0) := (others => '0');
    signal r5_fade_a     : unsigned(9 downto 0) := (others => '0');
    signal r5_dyst       : unsigned(10 downto 0) := (others => '0');
    signal r5_dy         : unsigned(10 downto 0) := (others => '0');
    signal r6_sa         : signed(14 downto 0) := (others => '0');
    signal r6_dyeff_mul  : unsigned(20 downto 0) := (others => '0');
    signal r6_dy         : unsigned(10 downto 0) := (others => '0');
    signal r7_lean       : signed(12 downto 0) := (others => '0');
    signal r7_dyeff      : unsigned(10 downto 0) := (others => '0');
    signal r7_dy         : unsigned(10 downto 0) := (others => '0');
    signal r8_lean_mul   : signed(24 downto 0) := (others => '0');
    signal r8_ep_hi      : unsigned(17 downto 0) := (others => '0');
    signal r8_ep_lo      : unsigned(17 downto 0) := (others => '0');
    signal r9_off        : signed(12 downto 0) := (others => '0');
    signal r9_e_mul      : unsigned(24 downto 0) := (others => '0');
    signal r10_e         : unsigned(13 downto 0) := (others => '0');
    signal r11_e         : unsigned(13 downto 0) := (others => '0');
    signal r11_pm        : signed(13 downto 0) := (others => '0');
    signal r11_pneg      : std_logic := '0';
    signal r12_qp        : signed(13 downto 0) := (others => '0');

    -- Serial divider: Q = 2^20 / (256 + e), 13 quotient bits, 16-clock loop
    signal div_cnt       : unsigned(3 downto 0) := (others => '0');
    signal div_d         : unsigned(14 downto 0) := to_unsigned(256, 15);
    signal div_rem       : unsigned(16 downto 0) := (others => '0');
    signal div_q         : unsigned(12 downto 0) := (others => '0');
    signal div_res       : unsigned(12 downto 0) := to_unsigned(4096, 13);

    -- Values sampled by the pixel path at stage 2
    signal row_drape_r   : std_logic := '0';
    signal row_capture_r : std_logic := '0';
    signal row_fade_a_r  : unsigned(9 downto 0) := (others => '0');
    signal row_q_r       : signed(13 downto 0) := to_signed(4096, 14);
    signal row_base_r    : signed(14 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Pixel pipeline
    ----------------------------------------------------------------------------
    signal s1_avid       : std_logic := '0';
    signal s1_px         : unsigned(11 downto 0) := (others => '0');
    -- Blur: one-pole (k=5, ~31 px) per channel
    signal iir_y, iir_u, iir_v : unsigned(13 downto 0) := (others => '0');

    signal s2_dx         : signed(12 downto 0) := (others => '0');
    signal s2_px         : unsigned(11 downto 0) := (others => '0');
    signal s2_avid, s2_drape, s2_cap : std_logic := '0';
    signal s2_fade       : unsigned(9 downto 0) := (others => '0');
    signal s2_q          : signed(13 downto 0) := (others => '0');
    signal s2_base       : signed(14 downto 0) := (others => '0');

    signal s3_ph         : signed(19 downto 0) := (others => '0');
    signal s3_pl         : signed(20 downto 0) := (others => '0');
    signal s3_px         : unsigned(11 downto 0) := (others => '0');
    signal s3_avid, s3_drape, s3_cap : std_logic := '0';
    signal s3_fade       : unsigned(9 downto 0) := (others => '0');
    signal s3_base       : signed(14 downto 0) := (others => '0');

    signal s3a_psum      : signed(26 downto 0) := (others => '0');
    signal s3a_px        : unsigned(11 downto 0) := (others => '0');
    signal s3a_avid, s3a_drape, s3a_cap : std_logic := '0';
    signal s3a_fade      : unsigned(9 downto 0) := (others => '0');
    signal s3a_base      : signed(14 downto 0) := (others => '0');

    signal s3b_rd        : signed(14 downto 0) := (others => '0');
    signal s3b_px        : unsigned(11 downto 0) := (others => '0');
    signal s3b_avid, s3b_drape, s3b_cap : std_logic := '0';
    signal s3b_fade      : unsigned(9 downto 0) := (others => '0');

    signal s3c_rd        : unsigned(C_ADDR_BITS-1 downto 0) := (others => '0');
    signal s3c_wa        : unsigned(C_ADDR_BITS-1 downto 0) := (others => '0');
    signal s3c_we        : std_logic := '0';
    signal s3c_wy, s3c_wu, s3c_wv : std_logic_vector(9 downto 0) := (others => '0');
    signal s3c_drape     : std_logic := '0';
    signal s3c_fade      : unsigned(9 downto 0) := (others => '0');

    -- Line buffers, initialised to neutral black (never green)
    type t_line_ram is array(0 to 2047) of std_logic_vector(9 downto 0);
    signal s_ram_y : t_line_ram := (others => "0001000000");
    signal s_ram_u : t_line_ram := (others => "1000000000");
    signal s_ram_v : t_line_ram := (others => "1000000000");

    signal s4_lb_y, s4_lb_u, s4_lb_v : std_logic_vector(9 downto 0) := (others => '0');
    signal s4_drape      : std_logic := '0';
    signal s4_fade       : unsigned(9 downto 0) := (others => '0');
    signal s4b_lb_y, s4b_lb_u, s4b_lb_v : unsigned(9 downto 0) := (others => '0');
    signal s4b_drape     : std_logic := '0';
    signal s4b_fade      : unsigned(9 downto 0) := (others => '0');

    signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');
    signal s6_drape      : std_logic := '0';
    signal s7_y          : unsigned(9 downto 0) := C_Y_BLACK;
    signal s7_u, s7_v    : unsigned(9 downto 0) := C_UV_ZERO;

    ----------------------------------------------------------------------------
    -- Sync + dry-video delay lines (entry i = i+1 clocks old)
    ----------------------------------------------------------------------------
    type t_bit_sr is array(0 to LATENCY-1) of std_logic;
    signal s_hsync_sr : t_bit_sr := (others => '1');
    signal s_vsync_sr : t_bit_sr := (others => '1');
    signal s_field_sr : t_bit_sr := (others => '1');
    signal s_avid_sr  : t_bit_sr := (others => '0');

    type t_data_sr is array(0 to LATENCY-2) of std_logic_vector(9 downto 0);
    signal s_y_dly : t_data_sr := (others => (others => '0'));
    signal s_u_dly : t_data_sr := (others => (others => '0'));
    signal s_v_dly : t_data_sr := (others => (others => '0'));

    -- One-pole low-pass step: acc + ((x << 4) - acc) >> k, acc in 10.4
    function f_iir(acc : unsigned(13 downto 0); x : std_logic_vector(9 downto 0);
                   k : natural) return unsigned is
        variable xs : unsigned(14 downto 0);
        variable d  : signed(14 downto 0);
        variable r  : signed(14 downto 0);
    begin
        xs := shift_left(resize(unsigned(x), 15), 4);
        d  := signed(xs) - signed(resize(acc, 15));
        r  := signed(resize(acc, 15)) + shift_right(d, k);
        return unsigned(r(13 downto 0));
    end function;

    -- Resolution scaling: virtual rows per FIELD line, Q4.8 (1080 / active
    -- field lines, doubled for interlace), from the timing ID.
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

begin

    main_p : process(clk)
        variable v_rem     : signed(12 downto 0);
        variable v_an      : unsigned(19 downto 0);
        variable v_dyst    : signed(11 downto 0);
        variable v_dy3     : unsigned(12 downto 0);
        variable v_rd      : signed(14 downto 0);
        variable v_div_sub : unsigned(16 downto 0);
        variable y_mul, u_mul, v_mul : unsigned(19 downto 0);
        variable alpha_inv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- Controls
            -------------------------------------------------------------------
            k1_r     <= unsigned(registers_in(0)(9 downto 0));
            k2_r     <= unsigned(registers_in(1)(9 downto 0));
            k3_r     <= unsigned(registers_in(2)(9 downto 0));
            k4_r     <= unsigned(registers_in(3)(9 downto 0));
            k5_r     <= unsigned(registers_in(4)(9 downto 0));
            k6_r     <= unsigned(registers_in(5)(9 downto 0));
            k12_r    <= unsigned(registers_in(7)(9 downto 0));
            blur_r   <= registers_in(6)(0);
            pleat_r  <= registers_in(6)(1);
            freeze_r <= registers_in(6)(2);
            bypass_r <= registers_in(6)(4);

            -- Split deadband: free-running decision, loaded once per field
            k1_diff <= signed('0' & k1_r) - signed('0' & k1_held);
            if k1_diff > 2 or k1_diff < -2 then
                k1_move <= '1';
            else
                k1_move <= '0';
            end if;

            -- Resolution scaling: per-mode steps, decoded every clock (the
            -- timing ID is static while a mode runs)
            r_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => r_xcl <= "10";
                when 5 | 6 | 14     => r_xcl <= "01";
                when others         => r_xcl <= "00";
            end case;

            -------------------------------------------------------------------
            -- Counters. Rows count ACTIVE lines: advance at avid fall, reset
            -- at vsync (idempotent, serration-safe). Height is taken from
            -- h_cnt, which only changes on active lines.  Rows and height
            -- are virtual 1080-line units (integer part of the Q.8 count).
            -------------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;

            if data_in.avid = '1' then
                pixel_x    <= pixel_x + 1;
                saw_active <= '1';
            end if;

            hpar <= not hpar;
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_x > to_unsigned(32, 12) then
                    max_x_r <= pixel_x;
                end if;
                pixel_x <= (others => '0');
                hpar    <= '0';
            end if;

            if data_in.avid = '1' and prev_avid = '0' then
                line_par <= hpar;
            end if;

            if data_in.avid = '0' and prev_avid = '1' then
                v_an    := aline_a + resize(r_ys, 20);
                aline_a <= v_an;
                h_cnt   <= v_an(18 downto 8);
                row_ya  <= v_an;
                -- real rows below the split (720p lean depth, see R7)
                if r2_split = '1' then
                    rdep <= to_unsigned(1, 11);
                else
                    rdep <= rdep + 1;
                end if;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                aline_a <= (others => '0');
                row_ya  <= (others => '0');
                if h_cnt > to_unsigned(32, 11) then
                    max_y_r <= h_cnt;
                end if;
                if k1_move = '1' then
                    k1_held <= k1_r;
                end if;
                -- Sway phase: an increment, so guard against serrated vsync
                if saw_active = '1' then
                    sway_ph    <= sway_ph + resize(sway_step_r, 10);
                    saw_active <= '0';
                end if;
            end if;

            -------------------------------------------------------------------
            -- Per-frame geometry (continuous, one op per stage)
            -------------------------------------------------------------------
            split_mul <= k1_held * max_y_r;
            split_y_r <= split_mul(20 downto 10);

            v_rem := signed(resize(max_y_r, 13)) - signed(resize(split_y_r, 13))
                   - to_signed(9, 13);
            if v_rem < 0 then
                rem_r <= (others => '0');
            else
                rem_r <= unsigned(v_rem(10 downto 0));
            end if;
            stretch_mul   <= rem_r * k2_r;
            stretch_off_r <= stretch_mul(20 downto 10) + to_unsigned(8, 11);

            centre_r <= max_x_r(11 downto 1);
            if cap_blur_r = '1' then
                centre_c_r <= signed(resize(centre_r, 15)) + to_signed(C_BLUR_COMP, 15);
            else
                centre_c_r <= signed(resize(centre_r, 15));
            end if;
            wm1_r  <= signed(resize(max_x_r, 15)) - 1;
            w2m1_r <= signed(shift_left(resize(max_x_r, 15), 1)) - 1;

            fade_slope_r  <= FADE_SLOPE_LUT(to_integer(k3_r(9 downto 5)));
            onset_slope_r <= ONSET_SLOPE_LUT(to_integer(k5_r(9 downto 5)));
            twist_t_r     <= signed('0' & k4_r) - to_signed(512, 11);
            sway_step_r   <= resize(k6_r(9 downto 6), 5) + to_unsigned(4, 5);

            -- Spread gain: 64 floor + 1.5x linear, then x13 steeper above 50%
            sp_lin_r <= to_unsigned(64, 11) + resize(k12_r, 11)
                      + resize(k12_r(9 downto 1), 11);
            if k12_r(9) = '1' then
                sp_hi_r <= k12_r(8 downto 0);
            else
                sp_hi_r <= (others => '0');
            end if;
            sp_hi13_r <= shift_left(resize(sp_hi_r, 13), 3)
                       + shift_left(resize(sp_hi_r, 13), 2)
                       + resize(sp_hi_r, 13);
            g_r <= resize(sp_lin_r, 14) + resize(sp_hi13_r, 14);

            -------------------------------------------------------------------
            -- Per-row pipeline
            -------------------------------------------------------------------
            -- R1: signed distance from the split (Q.8 virtual rows)
            r1_dysq <= signed(resize(row_ya, 21))
                     - signed(shift_left(resize(split_y_r, 21), 8));

            -- R2: depth + row class.  The split row is the one whose span
            -- [row, row + step) holds the split (a fractional step can jump
            -- over an exact value); drape rows lie a full step or more below.
            -- At 1080p (step 1.0) these are the old dys = 0 / dys > 0 tests.
            if r1_dysq >= signed(resize(r_ys, 21)) then
                r2_dy    <= unsigned(r1_dysq(18 downto 8));
                r2_drape <= '1';
            else
                r2_dy    <= (others => '0');
                r2_drape <= '0';
            end if;
            if r1_dysq >= 0 and r1_dysq < signed(resize(r_ys, 21)) then
                r2_split <= '1';
            else
                r2_split <= '0';
            end if;

            -- R3: depth past Stretch, sway wave index
            v_dyst := signed(resize(r2_dy, 12)) - signed(resize(stretch_off_r, 12));
            if v_dyst < 0 then
                r3_dyst <= (others => '0');
            else
                r3_dyst <= unsigned(v_dyst(10 downto 0));
            end if;
            r3_dy    <= r2_dy;
            r3_drape <= r2_drape;
            v_dy3    := resize(r2_dy, 13) + shift_left(resize(r2_dy, 13), 1);
            r3_sidx  <= v_dy3(10 downto 3) - sway_ph(9 downto 2);

            -- R4: sine ROM read, onset + fade products
            r4_sin      <= SIN_LUT(to_integer(r3_sidx));
            r4_gain_mul <= r3_dyst * onset_slope_r;
            r4_fade_mul <= r3_dyst * fade_slope_r;
            r4_dyst     <= r3_dyst;
            r4_dy       <= r3_dy;
            r4_drape    <= r3_drape;

            -- R5: saturate onset gain and fade alpha
            r5_sin <= r4_sin;
            if r4_gain_mul(20 downto 10) /= 0 then
                r5_gain <= to_unsigned(1023, 10);
            else
                r5_gain <= r4_gain_mul(9 downto 0);
            end if;
            if r4_drape = '0' then
                r5_fade_a <= (others => '0');
            elsif r4_fade_mul(20 downto 12) /= 0 then
                r5_fade_a <= to_unsigned(1023, 10);
            else
                r5_fade_a <= r4_fade_mul(11 downto 2);
            end if;
            r5_dyst <= r4_dyst;
            r5_dy   <= r4_dy;

            -- R6: sway amplitude (6-bit, still below 1/16 of travel), eased
            -- depth
            if k6_r(9 downto 6) = 0 then
                r6_sa <= (others => '0');
            else
                r6_sa <= r5_sin * signed('0' & k6_r(9 downto 4));
            end if;
            r6_dyeff_mul <= r5_dyst * r5_gain;
            r6_dy        <= r5_dy;

            -- R7: lean per row = 4*twist + sway (x 1/4096 px per row)
            r7_lean  <= shift_left(resize(twist_t_r, 13), 2)
                      + resize(shift_right(r6_sa, 3), 13);
            r7_dyeff <= r6_dyeff_mul(20 downto 10);
            -- lean depth in x pixels: lean is px per 1080p row, so the
            -- virtual depth is divided by the x step (720p: the real row
            -- count below the split = virtual / 1.5; SD: virtual / 2)
            case r_xcl is
                when "01"   => r7_dy <= rdep;
                when "10"   => r7_dy <= shift_right(r6_dy, 1);
                when others => r7_dy <= r6_dy;
            end case;

            -- R8: lean grows with depth; e = dyeff * g split hi/lo
            r8_lean_mul <= r7_lean * signed('0' & r7_dy);
            r8_ep_hi    <= r7_dyeff * g_r(13 downto 7);
            r8_ep_lo    <= r7_dyeff * g_r(6 downto 0);

            -- R9
            r9_off   <= resize(shift_right(r8_lean_mul, 12), 13);
            r9_e_mul <= shift_left(resize(r8_ep_hi, 25), 7) + resize(r8_ep_lo, 25);

            -- R10: base column; e = sat(e_mul >> 9, 16383)
            row_base_r <= centre_c_r + resize(r9_off, 15);
            if r9_e_mul(24 downto 23) /= "00" then
                r10_e <= (others => '1');
            else
                r10_e <= r9_e_mul(22 downto 9);
            end if;

            -- R11: Pleat scale is a triangle wave in e: s swings
            -- +1 -> -1 -> +1 every 2048 of e (x4096 fixed point).
            r11_e    <= r10_e;
            r11_pm   <= to_signed(4096, 14)
                      - signed(shift_left(resize(r10_e(9 downto 0), 14), 3));
            r11_pneg <= r10_e(10);

            -- R12
            if r11_pneg = '1' then
                r12_qp <= -r11_pm;
            else
                r12_qp <= r11_pm;
            end if;

            -- Fan scale s = 256 / (256 + e) (x4096) by restoring division;
            -- the loop runs forever and re-latches the same value once the
            -- row inputs are stable.
            div_cnt <= div_cnt + 1;
            if div_cnt = 0 then
                div_d   <= resize(r11_e, 15) + to_unsigned(256, 15);
                div_rem <= to_unsigned(256, 17);
                div_q   <= (others => '0');
            elsif div_cnt <= 13 then
                v_div_sub := div_rem - resize(div_d, 17);
                if div_rem >= resize(div_d, 17) then
                    div_q   <= div_q(11 downto 0) & '1';
                    div_rem <= shift_left(v_div_sub, 1);
                else
                    div_q   <= div_q(11 downto 0) & '0';
                    div_rem <= shift_left(div_rem, 1);
                end if;
            elsif div_cnt = 14 then
                div_res <= div_q;
            end if;

            if pleat_r = '1' then
                row_q_r <= r12_qp;
            else
                row_q_r <= signed(resize(div_res, 14));
            end if;
            row_drape_r   <= r2_drape;
            row_capture_r <= r2_split and not freeze_r;
            row_fade_a_r  <= r5_fade_a;
            if row_capture_r = '1' then
                cap_blur_r <= blur_r;
            end if;
            row_swap_r <= line_par xor cap_par;

            -------------------------------------------------------------------
            -- s1: counters in, blur IIRs
            -------------------------------------------------------------------
            s1_avid <= data_in.avid;
            s1_px   <= pixel_x;
            if data_in.avid = '1' and prev_avid = '0' then
                iir_y <= unsigned(data_in.y) & "0000";
                iir_u <= unsigned(data_in.u) & "0000";
                iir_v <= unsigned(data_in.v) & "0000";
            else
                iir_y <= f_iir(iir_y, data_in.y, 5);
                iir_u <= f_iir(iir_u, data_in.u, 5);
                iir_v <= f_iir(iir_v, data_in.v, 5);
            end if;

            -------------------------------------------------------------------
            -- s2: dx, sample row values
            -------------------------------------------------------------------
            s2_dx    <= signed(resize(s1_px, 13)) - signed(resize(centre_r, 13));
            s2_px    <= s1_px;
            s2_avid  <= s1_avid;
            s2_drape <= row_drape_r;
            s2_cap   <= row_capture_r;
            s2_fade  <= row_fade_a_r;
            s2_q     <= row_q_r;
            s2_base  <= row_base_r;

            -------------------------------------------------------------------
            -- s3: dx * s, split into signed hi (s[13:7]) and lo (s[6:0])
            -------------------------------------------------------------------
            s3_ph    <= s2_dx * s2_q(13 downto 7);
            s3_pl    <= s2_dx * signed('0' & s2_q(6 downto 0));
            s3_px    <= s2_px;
            s3_avid  <= s2_avid;
            s3_drape <= s2_drape;
            s3_cap   <= s2_cap;
            s3_fade  <= s2_fade;
            s3_base  <= s2_base;

            -- s3a
            s3a_psum  <= shift_left(resize(s3_ph, 27), 7) + resize(s3_pl, 27);
            s3a_px    <= s3_px;
            s3a_avid  <= s3_avid;
            s3a_drape <= s3_drape;
            s3a_cap   <= s3_cap;
            s3a_fade  <= s3_fade;
            s3a_base  <= s3_base;

            -- s3b
            s3b_rd    <= s3a_base + resize(shift_right(s3a_psum, 12), 15);
            s3b_px    <= s3a_px;
            s3b_avid  <= s3a_avid;
            s3b_drape <= s3a_drape;
            s3b_cap   <= s3a_cap;
            s3b_fade  <= s3a_fade;

            -------------------------------------------------------------------
            -- s3c: reflect at the picture edges; capture write port
            -------------------------------------------------------------------
            if s3b_rd < 0 then
                v_rd := -s3b_rd;
            elsif s3b_rd > wm1_r then
                v_rd := w2m1_r - s3b_rd;
            else
                v_rd := s3b_rd;
            end if;
            s3c_rd <= unsigned(v_rd(C_ADDR_BITS-1 downto 0));

            s3c_we <= s3b_cap and s3b_avid;
            s3c_wa <= s3b_px(C_ADDR_BITS-1 downto 0);
            if s3b_cap = '1' and s3b_avid = '1' then
                cap_par <= line_par;
            end if;
            if blur_r = '1' then
                s3c_wy <= std_logic_vector(iir_y(13 downto 4));
                s3c_wu <= std_logic_vector(iir_u(13 downto 4));
                s3c_wv <= std_logic_vector(iir_v(13 downto 4));
            else
                s3c_wy <= s_y_dly(4);
                s3c_wu <= s_u_dly(4);
                s3c_wv <= s_v_dly(4);
            end if;
            s3c_drape <= s3b_drape;
            s3c_fade  <= s3b_fade;

            -------------------------------------------------------------------
            -- s4: BRAM (one write, one unconditional read per RAM)
            -------------------------------------------------------------------
            if s3c_we = '1' then
                s_ram_y(to_integer(s3c_wa)) <= s3c_wy;
                s_ram_u(to_integer(s3c_wa)) <= s3c_wu;
                s_ram_v(to_integer(s3c_wa)) <= s3c_wv;
            end if;
            s4_lb_y  <= s_ram_y(to_integer(s3c_rd));
            s4_lb_u  <= s_ram_u(to_integer(s3c_rd));
            s4_lb_v  <= s_ram_v(to_integer(s3c_rd));
            s4_drape <= s3c_drape;
            s4_fade  <= s3c_fade;

            -- s4b: fabric register before any logic
            s4b_lb_y  <= unsigned(s4_lb_y);
            s4b_lb_u  <= unsigned(s4_lb_u);
            s4b_lb_v  <= unsigned(s4_lb_v);
            s4b_drape <= s4_drape;
            s4b_fade  <= s4_fade;

            -------------------------------------------------------------------
            -- s6: fade toward black: out = (src*(1023-a) + target*a) >> 10
            -------------------------------------------------------------------
            alpha_inv := not s4b_fade;
            y_mul := s4b_lb_y * alpha_inv + shift_left(resize(s4b_fade, 20), 6);
            u_mul := s4b_lb_u * alpha_inv + shift_left(resize(s4b_fade, 20), 9);
            v_mul := s4b_lb_v * alpha_inv + shift_left(resize(s4b_fade, 20), 9);
            s6_y     <= y_mul(19 downto 10);
            s6_u     <= u_mul(19 downto 10);
            s6_v     <= v_mul(19 downto 10);
            s6_drape <= s4b_drape;

            -------------------------------------------------------------------
            -- s7: blanking gate; passthrough rows + bypass take the dry video
            -- (9 clocks old here); drape rows swap U/V on parity-mismatched
            -- rows (row_swap_r is row-constant; line_par changes >=64
            -- clocks before any pixel of its line reaches this stage)
            -------------------------------------------------------------------
            if s_avid_sr(LATENCY-2) = '0' then
                s7_y <= C_Y_BLACK;
                s7_u <= C_UV_ZERO;
                s7_v <= C_UV_ZERO;
            elsif bypass_r = '1' or s6_drape = '0' then
                s7_y <= unsigned(s_y_dly(LATENCY-2));
                s7_u <= unsigned(s_u_dly(LATENCY-2));
                s7_v <= unsigned(s_v_dly(LATENCY-2));
            elsif row_swap_r = '1' then
                s7_y <= s6_y;
                s7_u <= s6_v;
                s7_v <= s6_u;
            else
                s7_y <= s6_y;
                s7_u <= s6_u;
                s7_v <= s6_v;
            end if;

            -------------------------------------------------------------------
            -- Delay lines
            -------------------------------------------------------------------
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to LATENCY-1 loop
                s_hsync_sr(i) <= s_hsync_sr(i-1);
                s_vsync_sr(i) <= s_vsync_sr(i-1);
                s_field_sr(i) <= s_field_sr(i-1);
                s_avid_sr(i)  <= s_avid_sr(i-1);
            end loop;
            s_y_dly(0) <= data_in.y;
            s_u_dly(0) <= data_in.u;
            s_v_dly(0) <= data_in.v;
            for i in 1 to LATENCY-2 loop
                s_y_dly(i) <= s_y_dly(i-1);
                s_u_dly(i) <= s_u_dly(i-1);
                s_v_dly(i) <= s_v_dly(i-1);
            end loop;
        end if;
    end process main_p;

    s_io_align_in.hsync_n <= s_hsync_sr(LATENCY-1);
    s_io_align_in.vsync_n <= s_vsync_sr(LATENCY-1);
    s_io_align_in.field_n <= s_field_sr(LATENCY-1);
    s_io_align_in.avid    <= s_avid_sr(LATENCY-1);
    s_io_align_in.y       <= std_logic_vector(s7_y);
    s_io_align_in.u       <= std_logic_vector(s7_u);
    s_io_align_in.v       <= std_logic_vector(s7_v);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (10 -> 12).
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

end architecture drape;
