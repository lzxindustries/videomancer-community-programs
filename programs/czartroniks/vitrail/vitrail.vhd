-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Vitrail: catalogue combo #20 (seed 20260915) -- bloom / glow + palette
-- quantization + mirror / kaleidoscope.  Two knobs each; the switches and
-- the slider decide how the three combine.
--
--   K1 Strip    width of the mirrored source strip, 1/32 .. 1/2 of the
--               picture (square-law knob; the top is a two-pane bookmatch)
--   K2 Axis     where the strip is taken from (always inside the picture)
--   K3 Thresh   bloom threshold, black .. near white
--   K4 Radius   bloom width 4 .. 128 px (exponential knob)
--   K5 Levels   luma levels 2 .. 16, black and white always included
--   K6 Chroma   grey .. a few bold glass colours .. near-original colour
--
--   S7  Wash    the glow is luma only / it also bleaches chroma to white
--   S8  Mirror  kaleidoscope fold / plain tiling
--   S9  Order   palette after the bloom / palette before the bloom
--   S10 Motion  fixed / the strip sweeps back and forth across the picture
--   S11 Bloom   highlights glow / shadows bloom dark
--   P12 Glow    bloom gain 0.25 .. 4 (square law), up to blow-out
--
-- Bloom: running box sum of the luma excess over the threshold (EBR ring
-- read n-1 back), centred on its pixel: the base picture is delayed n/2
-- through a second EBR ring and written n/2 columns back, the line's last
-- n/2 columns going in after active video (write bank latched per line).
-- The sums are re-zeroed in vblank (no avid for 512 clocks), where n changes.
-- Palette: vitrail_quant, before (S9 Pal First) or after the bloom.
-- Mirror: the read side folds x about the picture centre with a pixel
-- counter (any strip width, no seams), then adds the axis.  All reads are
-- of the previous line.  Per-frame constants come from a vblank sequencer
-- with one shared multiplier; K1/K2/K4/K5/K6 and the width have a 2-LSB
-- deadband so pot noise does not shimmer the panes.
-- Latency 8 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vitrail of program_top is
    -- data_in -> data_out latency is 8 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;

    constant C_LAT  : integer := 8;
    constant C_EDGE : integer := 12;     -- the strip stays this far inside the picture
    constant C_WS   : integer := 20;     -- write-side stage of the line-buffer write

    -- Radius (K4 top 6 bits) -> box width n (exponential 4..128), shift nr = ceil(log2 n),
    -- gain mantissa m = 256 * 2^nr / n (so acc >> nr scaled by m/256 is the true mean)
    subtype t_rt is std_logic_vector(20 downto 0);   -- n(8) & nr(3) & m(10)
    function f_rtab(i : unsigned(5 downto 0)) return t_rt is
    begin
        case to_integer(i) is
            when   0 => return "000001000100100000000";   -- n   4, nr 2, m 256
            when   1 => return "000001000100100000000";   -- n   4, nr 2, m 256
            when   2 => return "000001000100100000000";   -- n   4, nr 2, m 256
            when   3 => return "000001010110110011010";   -- n   5, nr 3, m 410
            when   4 => return "000001010110110011010";   -- n   5, nr 3, m 410
            when   5 => return "000001010110110011010";   -- n   5, nr 3, m 410
            when   6 => return "000001100110101010101";   -- n   6, nr 3, m 341
            when   7 => return "000001100110101010101";   -- n   6, nr 3, m 341
            when   8 => return "000001100110101010101";   -- n   6, nr 3, m 341
            when   9 => return "000001110110100100101";   -- n   7, nr 3, m 293
            when  10 => return "000001110110100100101";   -- n   7, nr 3, m 293
            when  11 => return "000001110110100100101";   -- n   7, nr 3, m 293
            when  12 => return "000010000110100000000";   -- n   8, nr 3, m 256
            when  13 => return "000010000110100000000";   -- n   8, nr 3, m 256
            when  14 => return "000010011000111000111";   -- n   9, nr 4, m 455
            when  15 => return "000010011000111000111";   -- n   9, nr 4, m 455
            when  16 => return "000010101000110011010";   -- n  10, nr 4, m 410
            when  17 => return "000010101000110011010";   -- n  10, nr 4, m 410
            when  18 => return "000010111000101110100";   -- n  11, nr 4, m 372
            when  19 => return "000010111000101110100";   -- n  11, nr 4, m 372
            when  20 => return "000011001000101010101";   -- n  12, nr 4, m 341
            when  21 => return "000011011000100111011";   -- n  13, nr 4, m 315
            when  22 => return "000011011000100111011";   -- n  13, nr 4, m 315
            when  23 => return "000011101000100100101";   -- n  14, nr 4, m 293
            when  24 => return "000011111000100010001";   -- n  15, nr 4, m 273
            when  25 => return "000100001000100000000";   -- n  16, nr 4, m 256
            when  26 => return "000100011010111100010";   -- n  17, nr 5, m 482
            when  27 => return "000100101010111000111";   -- n  18, nr 5, m 455
            when  28 => return "000100111010110101111";   -- n  19, nr 5, m 431
            when  29 => return "000101001010110011010";   -- n  20, nr 5, m 410
            when  30 => return "000101011010110000110";   -- n  21, nr 5, m 390
            when  31 => return "000101101010101110100";   -- n  22, nr 5, m 372
            when  32 => return "000101111010101100100";   -- n  23, nr 5, m 356
            when  33 => return "000110011010101001000";   -- n  25, nr 5, m 328
            when  34 => return "000110101010100111011";   -- n  26, nr 5, m 315
            when  35 => return "000110111010100101111";   -- n  27, nr 5, m 303
            when  36 => return "000111011010100011010";   -- n  29, nr 5, m 282
            when  37 => return "000111111010100001000";   -- n  31, nr 5, m 264
            when  38 => return "001000001010100000000";   -- n  32, nr 5, m 256
            when  39 => return "001000101100111100010";   -- n  34, nr 6, m 482
            when  40 => return "001001001100111000111";   -- n  36, nr 6, m 455
            when  41 => return "001001101100110101111";   -- n  38, nr 6, m 431
            when  42 => return "001010001100110011010";   -- n  40, nr 6, m 410
            when  43 => return "001010111100101111101";   -- n  43, nr 6, m 381
            when  44 => return "001011011100101101100";   -- n  45, nr 6, m 364
            when  45 => return "001100001100101010101";   -- n  48, nr 6, m 341
            when  46 => return "001100101100101001000";   -- n  50, nr 6, m 328
            when  47 => return "001101011100100110101";   -- n  53, nr 6, m 309
            when  48 => return "001110001100100100101";   -- n  56, nr 6, m 293
            when  49 => return "001110111100100010110";   -- n  59, nr 6, m 278
            when  50 => return "001111111100100000100";   -- n  63, nr 6, m 260
            when  51 => return "010000101110111110000";   -- n  66, nr 7, m 496
            when  52 => return "010001101110111010100";   -- n  70, nr 7, m 468
            when  53 => return "010010101110110111011";   -- n  74, nr 7, m 443
            when  54 => return "010011101110110100100";   -- n  78, nr 7, m 420
            when  55 => return "010100101110110010000";   -- n  82, nr 7, m 400
            when  56 => return "010101111110101111001";   -- n  87, nr 7, m 377
            when  57 => return "010111001110101100100";   -- n  92, nr 7, m 356
            when  58 => return "011000011110101010010";   -- n  97, nr 7, m 338
            when  59 => return "011001111110100111110";   -- n 103, nr 7, m 318
            when  60 => return "011011011110100101101";   -- n 109, nr 7, m 301
            when  61 => return "011100111110100011101";   -- n 115, nr 7, m 285
            when  62 => return "011110011110100001111";   -- n 121, nr 7, m 271
            when  63 => return "100000001110100000000";   -- n 128, nr 7, m 256
            when others => return "000001000100100000000";
        end case;
    end function;

    -- Levels index (0..14 -> L = 2..16): q = (t * ML) >> 16 over t = y - 64 in 0..875, out = 64 + q * SL
    subtype t_lt is std_logic_vector(20 downto 0);   -- ML(11) & SL(10)
    function f_ltab(i : unsigned(3 downto 0)) return t_lt is
    begin
        case to_integer(i) is
            when  0 => return "000100101011101101100";   -- L  2, ML 149, SL 876
            when  1 => return "000111000000110110110";   -- L  3, ML 224, SL 438
            when  2 => return "001001010110100100100";   -- L  4, ML 299, SL 292
            when  3 => return "001011101100011011011";   -- L  5, ML 374, SL 219
            when  4 => return "001110000000010101111";   -- L  6, ML 448, SL 175
            when  5 => return "010000010110010010010";   -- L  7, ML 523, SL 146
            when  6 => return "010010101100001111101";   -- L  8, ML 598, SL 125
            when  7 => return "010101000010001101101";   -- L  9, ML 673, SL 109
            when  8 => return "010111011000001100001";   -- L 10, ML 748, SL 97
            when  9 => return "011001101100001010111";   -- L 11, ML 822, SL 87
            when 10 => return "011100000010001001111";   -- L 12, ML 897, SL 79
            when 11 => return "011110011000001001001";   -- L 13, ML 972, SL 73
            when 12 => return "100000101110001000011";   -- L 14, ML 1047, SL 67
            when 13 => return "100011000100000111110";   -- L 15, ML 1122, SL 62
            when 14 => return "100101011010000111010";   -- L 16, ML 1197, SL 58
            when others => return "100101011010000111010";
        end case;
    end function;

    -- Chroma (K6 top 6 bits): d' = d * g / 16, rounded to a multiple of 2^k (mid-tread: 512 stays
    -- neutral), clamped to +-448.  Coarse end = 3 bold levels per axis, fine end = near-original.
    subtype t_ct is std_logic_vector(8 downto 0);    -- g(6) & k-4(3)
    function f_ctab(i : unsigned(5 downto 0)) return t_ct is
    begin
        case to_integer(i) is
            when  0 => return "000000101";   -- grey
            when  1 => return "000000101";   -- grey
            when  2 => return "000000101";   -- grey
            when  3 => return "011010101";   -- step in  315.1, out 512, gain 1.62
            when  4 => return "011011101";   -- step in  303.4, out 512, gain 1.69
            when  5 => return "011100101";   -- step in  292.6, out 512, gain 1.75
            when  6 => return "011110101";   -- step in  273.1, out 512, gain 1.88
            when  7 => return "011111101";   -- step in  264.3, out 512, gain 1.94
            when  8 => return "100001101";   -- step in  248.2, out 512, gain 2.06
            when  9 => return "100011101";   -- step in  234.1, out 512, gain 2.19
            when 10 => return "100100101";   -- step in  227.6, out 512, gain 2.25
            when 11 => return "100110101";   -- step in  215.6, out 512, gain 2.38
            when 12 => return "101000101";   -- step in  204.8, out 512, gain 2.50
            when 13 => return "101010101";   -- step in  195.0, out 512, gain 2.62
            when 14 => return "101100101";   -- step in  186.2, out 512, gain 2.75
            when 15 => return "101111101";   -- step in  174.3, out 512, gain 2.94
            when 16 => return "011000100";   -- step in  170.7, out 256, gain 1.50
            when 17 => return "011010100";   -- step in  157.5, out 256, gain 1.62
            when 18 => return "011011100";   -- step in  151.7, out 256, gain 1.69
            when 19 => return "011100100";   -- step in  146.3, out 256, gain 1.75
            when 20 => return "011110100";   -- step in  136.5, out 256, gain 1.88
            when 21 => return "011111100";   -- step in  132.1, out 256, gain 1.94
            when 22 => return "100001100";   -- step in  124.1, out 256, gain 2.06
            when 23 => return "100011100";   -- step in  117.0, out 256, gain 2.19
            when 24 => return "100101100";   -- step in  110.7, out 256, gain 2.31
            when 25 => return "100110100";   -- step in  107.8, out 256, gain 2.38
            when 26 => return "010100011";   -- step in  102.4, out 128, gain 1.25
            when 27 => return "010101011";   -- step in   97.5, out 128, gain 1.31
            when 28 => return "010110011";   -- step in   93.1, out 128, gain 1.38
            when 29 => return "010111011";   -- step in   89.0, out 128, gain 1.44
            when 30 => return "011001011";   -- step in   81.9, out 128, gain 1.56
            when 31 => return "011010011";   -- step in   78.8, out 128, gain 1.62
            when 32 => return "011011011";   -- step in   75.9, out 128, gain 1.69
            when 33 => return "011101011";   -- step in   70.6, out 128, gain 1.81
            when 34 => return "011110011";   -- step in   68.3, out 128, gain 1.88
            when 35 => return "100000011";   -- step in   64.0, out 128, gain 2.00
            when 36 => return "010001010";   -- step in   60.2, out  64, gain 1.06
            when 37 => return "010001010";   -- step in   60.2, out  64, gain 1.06
            when 38 => return "010010010";   -- step in   56.9, out  64, gain 1.12
            when 39 => return "010011010";   -- step in   53.9, out  64, gain 1.19
            when 40 => return "010100010";   -- step in   51.2, out  64, gain 1.25
            when 41 => return "010101010";   -- step in   48.8, out  64, gain 1.31
            when 42 => return "010110010";   -- step in   46.5, out  64, gain 1.38
            when 43 => return "011000010";   -- step in   42.7, out  64, gain 1.50
            when 44 => return "011001010";   -- step in   41.0, out  64, gain 1.56
            when 45 => return "011010010";   -- step in   39.4, out  64, gain 1.62
            when 46 => return "011011010";   -- step in   37.9, out  64, gain 1.69
            when 47 => return "001110001";   -- step in   36.6, out  32, gain 0.88
            when 48 => return "001111001";   -- step in   34.1, out  32, gain 0.94
            when 49 => return "010000001";   -- step in   32.0, out  32, gain 1.00
            when 50 => return "010001001";   -- step in   30.1, out  32, gain 1.06
            when 51 => return "010010001";   -- step in   28.4, out  32, gain 1.12
            when 52 => return "010010001";   -- step in   28.4, out  32, gain 1.12
            when 53 => return "010011001";   -- step in   26.9, out  32, gain 1.19
            when 54 => return "010100001";   -- step in   25.6, out  32, gain 1.25
            when 55 => return "010101001";   -- step in   24.4, out  32, gain 1.31
            when 56 => return "010111001";   -- step in   22.3, out  32, gain 1.44
            when 57 => return "001100000";   -- step in   21.3, out  16, gain 0.75
            when 58 => return "001100000";   -- step in   21.3, out  16, gain 0.75
            when 59 => return "001101000";   -- step in   19.7, out  16, gain 0.81
            when 60 => return "001110000";   -- step in   18.3, out  16, gain 0.88
            when 61 => return "001110000";   -- step in   18.3, out  16, gain 0.88
            when 62 => return "001111000";   -- step in   17.1, out  16, gain 0.94
            when 63 => return "010000000";   -- step in   16.0, out  16, gain 1.00
            when others => return "000000101";
        end case;
    end function;

    -- line buffers (two banks by line parity) and the bloom rings
    type t_lb_y  is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c  is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ering is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_bring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal ering : t_ering := (others => x"0000");
    signal bring : t_bring := (others => x"00408080");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0  : signal is true;
    attribute no_rw_check of lbY1  : signal is true;
    attribute no_rw_check of lbU0  : signal is true;
    attribute no_rw_check of lbU1  : signal is true;
    attribute no_rw_check of lbV0  : signal is true;
    attribute no_rw_check of lbV1  : signal is true;
    attribute no_rw_check of ering : signal is true;
    attribute no_rw_check of bring : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_ring_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_bq     : std_logic_vector(31 downto 0) := x"00408080";
    signal s_ring_w : unsigned(7 downto 0) := (others => '0');

    -- controls
    signal s_k_strip, s_k_axis, s_k_thr, s_k_radius, s_k_levels, s_k_chroma : unsigned(9 downto 0);
    signal s_sw_white, s_sw_tile, s_sw_prepal, s_sw_sweep, s_sw_shadow : std_logic;
    signal s_postpal : std_logic;
    signal s_p_glow : unsigned(9 downto 0);

    -- per-frame sequencer
    signal s_vstep  : unsigned(4 downto 0) := (others => '1');
    signal s_k1l, s_k2l, s_k4l, s_k5l, s_k6l : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12l   : unsigned(9 downto 0) := (others => '0');
    signal s_wl     : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_rt     : t_rt := (others => '0');
    signal s_lt     : t_lt := (others => '0');
    signal s_ct     : t_ct := (others => '0');
    signal s_lidx   : unsigned(3 downto 0) := (others => '0');
    signal s_ma, s_mb : unsigned(10 downto 0) := (others => '0');
    signal s_mph    : unsigned(15 downto 0) := (others => '0');
    signal s_mpl    : unsigned(16 downto 0) := (others => '0');
    signal s_mp     : unsigned(21 downto 0) := (others => '0');
    signal s_k1sq, s_p2 : unsigned(9 downto 0) := (others => '0');
    signal s_f1024  : unsigned(8 downto 0) := (others => '0');
    signal s_gq     : unsigned(10 downto 0) := (others => '0');
    signal s_mn     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_range  : unsigned(10 downto 0) := (others => '0');
    signal s_dr     : unsigned(10 downto 0) := (others => '0');
    signal s_diff   : signed(11 downto 0) := (others => '0');
    signal s_qpar   : std_logic := '0';
    signal s_k2pos  : unsigned(10 downto 0) := (others => '0');
    signal s_t16    : unsigned(15 downto 0) := (others => '0');
    signal s_d16    : signed(16 downto 0) := (others => '0');
    signal s_r2     : unsigned(11 downto 0) := (others => '0');
    signal s_su     : signed(13 downto 0) := (others => '0');
    signal s_sdu    : signed(13 downto 0) := (others => '0');
    signal s_sfl    : signed(13 downto 0) := (others => '0');
    signal s_cfold  : std_logic := '0';

    -- per-frame results
    signal s_thr    : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_nr     : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_ndel   : unsigned(7 downto 0) := to_unsigned(15, 8);
    signal s_dly    : unsigned(7 downto 0) := to_unsigned(8, 8);     -- base delay n/2
    signal s_gainn  : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_ml     : unsigned(10 downto 0) := to_unsigned(673, 11);
    signal s_sl     : unsigned(9 downto 0) := to_unsigned(109, 10);
    signal s_cg     : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_ck     : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_s      : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_sm1    : unsigned(10 downto 0) := to_unsigned(255, 11);
    signal s_half   : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_p0     : unsigned(10 downto 0) := (others => '0');
    signal s_par0   : std_logic := '0';
    signal s_axis   : unsigned(10 downto 0) := to_unsigned(C_EDGE, 11);

    -- tracking
    signal s_rx, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_blank  : unsigned(9 downto 0) := (others => '0');   -- clocks without avid, saturates at 512
    signal s_wpar, s_rbank, s_a_rbank : std_logic := '0';

    -- write side
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_qa_y, s_qa_u, s_qa_v : unsigned(9 downto 0);
    signal s_e7     : unsigned(9 downto 0) := (others => '0');
    signal s_b7_y, s_b7_u, s_b7_v : unsigned(9 downto 0) := (others => '0');
    signal s_acc    : unsigned(16 downto 0) := (others => '0');
    signal s_bl9    : unsigned(9 downto 0) := (others => '0');
    signal s_ph10, s_pl10 : unsigned(15 downto 0) := (others => '0');
    signal s_gs11   : unsigned(20 downto 0) := (others => '0');
    signal s_vy12   : signed(14 downto 0) := (others => '0');
    signal s_ws12   : unsigned(7 downto 0) := (others => '0');
    signal s_du12, s_dv12 : signed(10 downto 0) := (others => '0');
    signal s_y13    : unsigned(9 downto 0) := (others => '0');
    signal s_pu13, s_pv13 : signed(19 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    type t_b10 is array (9 to 13) of unsigned(9 downto 0);
    signal s_by, s_bu, s_bv : t_b10 := (others => (others => '0'));
    signal s_qb_y, s_qb_u, s_qb_v : unsigned(9 downto 0);
    signal s_wcol   : signed(12 downto 0) := (others => '0');
    signal s_wbank  : std_logic := '0';
    signal s_wa     : unsigned(10 downto 0) := (others => '0');
    signal s_we, s_wb : std_logic := '0';
    signal s_wy     : unsigned(9 downto 0) := (others => '0');
    signal s_wu, s_wv : unsigned(7 downto 0) := (others => '0');
    type t_wav is array (0 to C_WS - 1) of std_logic;
    signal s_wav    : t_wav := (others => '0');

    -- read side
    signal s_first1, s_left1, s_mid1 : std_logic := '0';
    signal s_cw2    : unsigned(10 downto 0) := (others => '0');
    signal s_cpar2, s_left2 : std_logic := '0';
    signal s_f3     : unsigned(10 downto 0) := (others => '0');
    signal s_addr4  : unsigned(10 downto 0) := (others => '0');
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

    -- 2-LSB deadband: follow the pot only when it has moved by 2 or more
    function f_db(k, l : unsigned(9 downto 0)) return unsigned is
        variable d : signed(10 downto 0);
    begin
        d := signed('0' & k) - signed('0' & l);
        if d > 1 or d < -1 then return k; else return l; end if;
    end function;

begin

    s_k_strip  <= unsigned(registers_in(0));
    s_k_axis   <= unsigned(registers_in(1));
    s_k_thr    <= unsigned(registers_in(2));
    s_k_radius <= unsigned(registers_in(3));
    s_k_levels <= unsigned(registers_in(4));
    s_k_chroma <= unsigned(registers_in(5));
    s_sw_white  <= registers_in(6)(0);
    s_sw_tile   <= registers_in(6)(1);
    s_sw_prepal <= registers_in(6)(2);
    s_sw_sweep  <= registers_in(6)(3);
    s_sw_shadow <= registers_in(6)(4);
    s_postpal   <= not registers_in(6)(2);
    s_p_glow   <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Palette before the bloom (S9 Pal First), input stage 1 -> stage 6
    --------------------------------------------------------------------------
    quant_a : entity work.vitrail_quant
        port map (clk => clk, en => s_sw_prepal, ml => s_ml, sl => s_sl, cg => s_cg, ck => s_ck,
                  y_in => s_in_y, u_in => s_in_u, v_in => s_in_v,
                  y_out => s_qa_y, u_out => s_qa_u, v_out => s_qa_v);

    --------------------------------------------------------------------------
    -- Palette after the bloom (S9 Pal Last), stage 14 -> stage 19
    --------------------------------------------------------------------------
    quant_b : entity work.vitrail_quant
        port map (clk => clk, en => s_postpal, ml => s_ml, sl => s_sl, cg => s_cg, ck => s_ck,
                  y_in => s_y14, u_in => s_u14, v_in => s_v14,
                  y_out => s_qb_y, u_out => s_qb_u, v_out => s_qb_v);

    --------------------------------------------------------------------------
    -- Bloom rings: luma excess read n-1 back; base picture read n/2 back
    --------------------------------------------------------------------------
    p_ering : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_q <= ering(to_integer(s_ring_w - s_ndel));
            ering(to_integer(s_ring_w)) <= "000000" & std_logic_vector(s_e7);
        end if;
    end process;
    p_bring : process(clk)
    begin
        if rising_edge(clk) then
            s_bq <= bring(to_integer(s_ring_w - s_dly));
            bring(to_integer(s_ring_w)) <= "000000" & std_logic_vector(s_b7_y)
                                           & std_logic_vector(s_b7_u(9 downto 2))
                                           & std_logic_vector(s_b7_v(9 downto 2));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control, write side, per-frame sequencer
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(11 downto 0);
        variable v_g  : unsigned(12 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_r  : signed(12 downto 0);
        variable v_cu, v_cv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if data_in.avid = '1' then s_blank <= (others => '0'); elsif s_blank(9) = '0' then s_blank <= s_blank + 1; end if;
            s_ring_w <= s_ring_w + 1;

            -- stage 1
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_wav(0) <= data_in.avid;
            for i in 1 to C_WS - 1 loop s_wav(i) <= s_wav(i - 1); end loop;

            -- stage 7: excess over / under the threshold (pre-palette picture)
            if s_sw_shadow = '1' then
                v_e := signed(resize(s_thr, 12)) - signed(resize(s_qa_y, 12));
            else
                v_e := signed(resize(s_qa_y, 12)) - signed(resize(s_thr, 12));
            end if;
            if v_e < 0 or s_wav(5) = '0' then s_e7 <= (others => '0'); else s_e7 <= unsigned(v_e(9 downto 0)); end if;
            s_b7_y <= s_qa_y; s_b7_u <= s_qa_u; s_b7_v <= s_qa_v;

            -- stage 8: running sum over the last n columns, held at zero once
            -- the input has had no active video for 512 clocks (vblank only:
            -- hblank is 138..280).  Never the vsync LEVEL -- it can read low
            -- across active video on hardware (v0.2.0's bloom was dead).
            if s_blank(9) = '1' then
                s_acc <= (others => '0');
            else
                s_acc <= s_acc + resize(s_e7, 17) - resize(unsigned(s_ring_q(9 downto 0)), 17);
            end if;
            -- (s_bq = stage-8 base, column n/2 behind the newest summed column)

            -- stage 9: box mean scaled to 10 bits by 2^nr
            case to_integer(s_nr) is
                when 2      => s_bl9 <= s_acc(11 downto 2);
                when 3      => s_bl9 <= s_acc(12 downto 3);
                when 4      => s_bl9 <= s_acc(13 downto 4);
                when 5      => s_bl9 <= s_acc(14 downto 5);
                when 6      => s_bl9 <= s_acc(15 downto 6);
                when others => s_bl9 <= s_acc(16 downto 7);
            end case;
            s_by(9) <= unsigned(s_bq(25 downto 16));
            s_bu(9) <= unsigned(s_bq(15 downto 8)) & "00";
            s_bv(9) <= unsigned(s_bq(7 downto 0)) & "00";
            for i in 10 to 13 loop
                s_by(i) <= s_by(i - 1); s_bu(i) <= s_bu(i - 1); s_bv(i) <= s_bv(i - 1);
            end loop;

            -- stage 10: gain products (split on the mean)
            s_ph10 <= s_bl9(9 downto 5) * s_gainn;
            s_pl10 <= s_bl9(4 downto 0) * s_gainn;
            -- stage 11
            s_gs11 <= shift_left(resize(s_ph10, 21), 5) + resize(s_pl10, 21);

            -- stage 12: glow added / subtracted; wash amount; chroma pull
            v_g := s_gs11(20 downto 8);
            if s_sw_shadow = '1' then
                s_vy12 <= signed(resize(s_by(11), 15)) - signed(resize(v_g, 15));
            else
                s_vy12 <= signed(resize(s_by(11), 15)) + signed(resize(v_g, 15));
            end if;
            if s_sw_white = '0' then
                s_ws12 <= (others => '0');
            elsif v_g(12 downto 8) /= 0 then
                s_ws12 <= x"FF";
            else
                s_ws12 <= v_g(7 downto 0);
            end if;
            s_du12 <= to_signed(512, 11) - signed('0' & s_bu(11));
            s_dv12 <= to_signed(512, 11) - signed('0' & s_bv(11));

            -- stage 13: luma clamp 64..940; wash products
            if s_vy12 < 64 then
                s_y13 <= to_unsigned(64, 10);
            elsif s_vy12 > 940 then
                s_y13 <= to_unsigned(940, 10);
            else
                s_y13 <= unsigned(s_vy12(9 downto 0));
            end if;
            s_pu13 <= s_du12 * signed('0' & s_ws12);
            s_pv13 <= s_dv12 * signed('0' & s_ws12);

            -- stage 14: chroma pulled toward 512 (stays inside 0..1023)
            s_y14 <= s_y13;
            v_cu := signed(resize(s_bu(13), 12)) + resize(shift_right(s_pu13, 8), 12);
            v_cv := signed(resize(s_bv(13), 12)) + resize(shift_right(s_pv13, 8), 12);
            s_u14 <= unsigned(v_cu(9 downto 0));
            s_v14 <= unsigned(v_cv(9 downto 0));

            -- stage 19 (post-palette out): column counter, n/2 behind the summed column;
            -- it keeps counting past active video so the last n/2 columns get written
            if s_wav(17) = '1' and s_wav(18) = '0' then
                s_wcol  <= -signed(resize(s_dly, 13));
                s_wbank <= s_wpar;
            elsif s_wcol(12) = '1' or s_wcol(11) = '0' then
                s_wcol <= s_wcol + 1;
            end if;
            -- stage 20: line-buffer write port
            s_wa <= unsigned(s_wcol(10 downto 0));
            if s_wcol(12) = '0' and s_wcol(11) = '0' and unsigned(s_wcol(10 downto 0)) < s_wl then
                s_we <= '1';
            else
                s_we <= '0';
            end if;
            s_wb <= s_wbank;
            s_wy <= s_qb_y; s_wu <= s_qb_u(9 downto 2); s_wv <= s_qb_v(9 downto 2);

            ------------------------------------------------------------------
            -- per-frame sequencer (runs in vsync; one shared multiplier,
            -- product valid 3 states after its operands are set)
            ------------------------------------------------------------------
            s_mph <= s_ma(10 downto 6) * s_mb;
            s_mpl <= s_ma(5 downto 0) * s_mb;
            s_mp  <= shift_left(resize(s_mph, 22), 6) + resize(s_mpl, 22);

            case to_integer(s_vstep) is
                when 0 =>
                    s_k1l <= f_db(s_k_strip, s_k1l);
                    s_k2l <= f_db(s_k_axis, s_k2l);
                    s_k4l <= f_db(s_k_radius, s_k4l);
                    s_k5l <= f_db(s_k_levels, s_k5l);
                    s_k6l <= f_db(s_k_chroma, s_k6l);
                    s_p12l <= s_p_glow;
                    v_r := signed(resize(s_line_width, 13)) - signed(resize(s_wl, 13));
                    if v_r > 1 or v_r < -1 then s_wl <= s_line_width; end if;
                    s_thr <= resize(resize(s_k_thr, 11) + 64 - shift_right(s_k_thr, 3) - shift_right(s_k_thr, 4), 10);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_rt <= f_rtab(s_k4l(9 downto 4));
                    s_ct <= f_ctab(s_k6l(9 downto 4));
                    v_t := shift_left(resize(s_k5l, 14), 4) - resize(s_k5l, 14);
                    s_lidx <= v_t(13 downto 10);
                    s_ma <= resize(s_k1l, 11); s_mb <= resize(s_k1l, 11);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_lt <= f_ltab(s_lidx);
                    s_ma <= resize(s_p12l, 11); s_mb <= resize(s_p12l, 11);
                    s_nr   <= unsigned(s_rt(12 downto 10));
                    s_ndel <= unsigned(s_rt(20 downto 13)) - 1;
                    s_dly  <= shift_right(unsigned(s_rt(20 downto 13)), 1);
                    s_mn   <= unsigned(s_rt(9 downto 0));
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_ml <= unsigned(s_lt(20 downto 10));
                    s_sl <= unsigned(s_lt(9 downto 0));
                    s_cg <= unsigned(s_ct(8 downto 3));
                    s_ck <= unsigned(s_ct(2 downto 0));
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_k1sq <= s_mp(19 downto 10);                   -- K1^2 / 1024
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_p2 <= s_mp(19 downto 10);                     -- P12^2 / 1024
                    v_t := shift_left(resize(s_k1sq, 14), 4) - resize(s_k1sq, 14);
                    s_f1024 <= resize(shift_right(v_t, 5), 9) + 32;  -- strip / width, 1/1024 units
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_ma <= s_wl; s_mb <= resize(s_f1024, 11);
                    s_gq <= resize(s_p2, 11) + 64 - resize(shift_right(s_p2, 4), 11);   -- gain, 1/256 units
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_ma <= s_gq; s_mb <= resize(s_mn, 11);
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    s_s <= s_mp(20 downto 10);                      -- strip width
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    if s_mp(19) = '1' then s_gainn <= (others => '1'); else s_gainn <= s_mp(18 downto 8); end if;
                    s_sm1  <= s_s - 1;
                    s_half <= shift_right(s_wl, 1);
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    v_r := signed(resize(s_wl, 13)) - (2 * C_EDGE) - signed(resize(s_s, 13));
                    if v_r < 0 then s_range <= (others => '0'); else s_range <= unsigned(v_r(10 downto 0)); end if;
                    s_dr   <= s_half - 1;
                    s_qpar <= '0';
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    s_ma <= resize(s_k2l, 11); s_mb <= s_range;
                    s_diff <= signed(resize(s_dr, 12)) - signed(resize(s_s, 12));
                    s_vstep <= s_vstep + 1;
                when 13 =>                                          -- (half-1) mod / div strip
                    if s_diff(11) = '0' and s_s /= 0 then
                        s_dr <= unsigned(s_diff(10 downto 0));
                        s_qpar <= not s_qpar;
                        s_vstep <= to_unsigned(14, 5);
                    else
                        s_vstep <= to_unsigned(15, 5);
                    end if;
                when 14 =>
                    s_diff <= signed(resize(s_dr, 12)) - signed(resize(s_s, 12));
                    s_vstep <= to_unsigned(13, 5);
                when 15 =>
                    s_p0   <= s_dr;
                    s_par0 <= s_qpar;
                    s_vstep <= s_vstep + 1;
                when 16 =>
                    s_k2pos <= s_mp(20 downto 10);                  -- K2 * range / 1024
                    if s_sw_sweep = '1' then
                        s_t16 <= s_t16 + resize(shift_right(s_wl, 5), 16);   -- width/512 px per frame
                    else
                        s_t16 <= (others => '0');
                    end if;
                    s_r2 <= shift_left(resize(s_range, 12), 1);
                    s_vstep <= s_vstep + 1;
                when 17 =>
                    s_d16 <= signed(resize(s_t16, 17)) - signed(resize(shift_left(resize(s_r2, 16), 4), 17));
                    s_vstep <= s_vstep + 1;
                when 18 =>                                          -- sweep phase mod 2 * range
                    if s_d16(16) = '0' and s_r2 /= 0 then
                        s_t16 <= unsigned(s_d16(15 downto 0));
                        s_vstep <= to_unsigned(17, 5);
                    else
                        s_vstep <= s_vstep + 1;
                    end if;
                when 19 =>
                    s_su <= signed(resize(s_k2pos, 14)) + signed(resize(s_t16(15 downto 4), 14));
                    s_vstep <= s_vstep + 1;
                when 20 =>
                    s_sdu <= s_su - signed(resize(s_r2, 14));
                    s_vstep <= s_vstep + 1;
                when 21 =>
                    if s_sdu(13) = '0' then s_su <= s_sdu; end if;
                    s_vstep <= s_vstep + 1;
                when 22 =>                                          -- ping-pong over 0..range
                    s_sfl <= signed(resize(s_r2, 14)) - s_su;
                    if s_su > signed(resize(s_range, 14)) then s_cfold <= '1'; else s_cfold <= '0'; end if;
                    s_vstep <= s_vstep + 1;
                when 23 =>
                    if s_cfold = '1' then s_su <= s_sfl; end if;
                    s_vstep <= s_vstep + 1;
                when 24 =>
                    if s_su < 0 then
                        s_axis <= to_unsigned(C_EDGE, 11);
                    elsif s_su > signed(resize(s_range, 14)) then
                        s_axis <= s_range + C_EDGE;
                    else
                        s_axis <= unsigned(s_su(10 downto 0)) + C_EDGE;
                    end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Read side: fold x about the picture centre (pixel counter), add axis
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 1: column + its side of the centre
            s_first1 <= data_in.avid and not s_avid_sr(0);
            if s_rx < s_half then s_left1 <= '1'; else s_left1 <= '0'; end if;
            if s_rx = s_half then s_mid1 <= '1'; else s_mid1 <= '0'; end if;
            -- stage 2: distance from the centre, mod strip width, and pane parity
            if s_first1 = '1' then
                s_cw2 <= s_p0; s_cpar2 <= s_par0;
            elsif s_mid1 = '1' then
                s_cw2 <= (others => '0'); s_cpar2 <= '0';
            elsif s_left1 = '1' then
                if s_cw2 = 0 then s_cw2 <= s_sm1; s_cpar2 <= not s_cpar2; else s_cw2 <= s_cw2 - 1; end if;
            else
                if s_cw2 = s_sm1 then s_cw2 <= (others => '0'); s_cpar2 <= not s_cpar2; else s_cw2 <= s_cw2 + 1; end if;
            end if;
            s_left2 <= s_left1;
            -- stage 3: kaleido mirrors odd panes; tile mirrors the left half's count
            if (s_sw_tile = '1' and s_left2 = '1') or (s_sw_tile = '0' and s_cpar2 = '1') then
                s_f3 <= s_sm1 - s_cw2;
            else
                s_f3 <= s_cw2;
            end if;
            -- stage 4: into the strip
            s_addr4 <= s_axis + s_f3;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers: written at stage 20 (column n/2 back), read at stage 4
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr4));
            if s_we = '1' and s_wb = '0' then lbY0(to_integer(s_wa)) <= std_logic_vector(s_wy); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr4));
            if s_we = '1' and s_wb = '1' then lbY1(to_integer(s_wa)) <= std_logic_vector(s_wy); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wb = '0' then lbU0(to_integer(s_wa(10 downto 1))) <= std_logic_vector(s_wu); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wb = '1' then lbU1(to_integer(s_wa(10 downto 1))) <= std_logic_vector(s_wu); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wb = '0' then lbV0(to_integer(s_wa(10 downto 1))) <= std_logic_vector(s_wv); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wb = '1' then lbV1(to_integer(s_wa(10 downto 1))) <= std_logic_vector(s_wv); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch, gate, output (stages 6..8)
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            s_a_rbank <= s_rbank;
            -- stage 6: bank select
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 7: gate
            if s_avid_sr(5) = '1' then
                s_y7 <= s_wet_y; s_u7 <= s_wet_u; s_v7 <= s_wet_v;
            else
                s_y7 <= to_unsigned(64, 10); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
            end if;
            -- stage 8: output
            s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y8);
    data_out.u       <= std_logic_vector(s_u8);
    data_out.v       <= std_logic_vector(s_v8);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture vitrail;
