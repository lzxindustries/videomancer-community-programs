-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Reflector v0.2.0: catalogue combo #8 (seed 20260912) -- mirror /
-- kaleidoscope + bloom / glow.  Three knobs each; the switches and the
-- slider decide how they combine.
--
--   K1 Axis      first mirror position (0..100% of the width).  The mirrors
--                keep bouncing, so below 50% the panes multiply (to 16)
--   K2 Folds     None / One / Two / Three: pane width = Axis, /2, /4
--   K3 Slide     the source scrolls under the mirrors, bipolar; squared
--                speed curve with a centre detent (to 15 px a frame; 1080-
--                line px, scaled per mode)
--   K4 Glow      bloom gain
--   K5 Threshold luma above which the blur blooms (black .. ~white)
--   K6 Radius    blur width 7..255 px (two cascaded boxes, exponential;
--                1080-line px, scaled per mode)
--
--   S7  Glow     glow fetched mirrored with the picture / unmirrored
--   S8  Panes    mirrored panes shown as negative
--   S9  Tint     warm glow
--   S10 Seams    soft light lines along the mirror seams (and the slide's
--                wrap), lit even with Glow at 0
--   S11 Focus    base picture replaced by its blur
--   P12 Sweep    the axis swings on a sine around K1; depth (to +-half the
--                width) and speed (22 s .. 4 s a cycle) both grow with it
--
-- Mirrors: a two-mirror kaleidoscope.  The source column bounces between 0
-- and the pane width P (a per-pixel up/down counter, started each line from
-- a state the vblank sequencer works out), so every column is filled at any
-- Axis.  Panes read the source inset by C_EDGE columns (no black stripes
-- from an analog capture's edge columns at the seams).
-- Blur: halation's engine -- 8-bit luma through two cascaded running boxes
-- of width n (4..128, 64 max in SD) sharing one 256x16 EBR ring, padded with
-- the line's own inset edge pixels; the glow goes to its own line buffer at
-- its centre column (newest sample - n - 7), so it sits on its highlights.
-- Output: highlight knee (600 + (y - 600)/2, cap 784; whites ~770 -- the HD
-- HDMI bar fix's luma swing without dimming the mid-tones) and [1 2 1] on
-- Y/U/V.  Latency 14 clocks, all modes, blanking-gated.  All line-buffer
-- reads are of the previous line.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The pane layout stays a
-- fraction of the width (Axis, Sweep depth: the pane count is a count), but
-- the drawn sizes are 1080-line pixels, scaled per mode from the video
-- timing ID (registers_in(8)): seam distances are measured in virtual half
-- units (x 2 / 3 / 4 per pixel at 1080 / 720p / SD, saturated beyond 63 px),
-- so the seam ridge is 32 units wide at every resolution (16 px at SD as
-- before, 21.3 px at 720p, was 32) and narrows by the pane width in units;
-- the Radius table index drops 15 / 26 steps at 720p / SD (box width x 2/3 /
-- x 1/2; the table runs 26 steps per octave); the Slide speed is x 2/3 /
-- x 1/2 (taken one frame late there).  1080p is unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture reflector of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (14 + 2 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 14;
    constant C_EDGE : integer := 12;
    constant C_GLAG : integer := 7;    -- box-2 mean is centred n + C_GLAG behind its newest sample
    constant C_KNEE : integer := 600;
    constant C_YMAX : integer := 784;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    -- Resolution scaling: virtual x step from the timing ID, in half units
    -- per pixel (1080: 2, 720p: 3, SD: 4 -- 720 px span 1440 units)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- a seam distance (px) in half units: d * x step, or all-ones beyond 63 px
    -- (no seam reaches that far) or when the seam is not valid
    function f_sdist(d : unsigned(10 downto 0); ok : boolean; xs : unsigned(2 downto 0)) return unsigned is
        variable v : unsigned(8 downto 0);
    begin
        case xs is
            when "011"  => v := resize(d(5 downto 0), 9) + shift_left(resize(d(5 downto 0), 9), 1);
            when "100"  => v := shift_left(resize(d(5 downto 0), 9), 2);
            when others => v := shift_left(resize(d(5 downto 0), 9), 1);
        end case;
        if ok and d(10 downto 6) = 0 then
            return v;
        else
            return to_unsigned(511, 9);
        end if;
    end function;

    -- Radius (K6 top 7 bits) -> box width n (exponential 4..128) and 1/n ~ rm / 2^(9 + sh)
    subtype t_rt is std_logic_vector(18 downto 0);   -- n(8) & rm(8) & sh(3)
    function f_rtab(i : unsigned(6 downto 0)) return t_rt is
    begin
        case to_integer(i) is
            when   0 => return "0000010010000000000";   -- n   4, 1/n ~ 128/2^9
            when   1 => return "0000010010000000000";   -- n   4, 1/n ~ 128/2^9
            when   2 => return "0000010010000000000";   -- n   4, 1/n ~ 128/2^9
            when   3 => return "0000010010000000000";   -- n   4, 1/n ~ 128/2^9
            when   4 => return "0000010010000000000";   -- n   4, 1/n ~ 128/2^9
            when   5 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when   6 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when   7 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when   8 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when   9 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when  10 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when  11 => return "0000010111001100001";   -- n   5, 1/n ~ 204/2^10
            when  12 => return "0000011010101010001";   -- n   6, 1/n ~ 170/2^10
            when  13 => return "0000011010101010001";   -- n   6, 1/n ~ 170/2^10
            when  14 => return "0000011010101010001";   -- n   6, 1/n ~ 170/2^10
            when  15 => return "0000011010101010001";   -- n   6, 1/n ~ 170/2^10
            when  16 => return "0000011010101010001";   -- n   6, 1/n ~ 170/2^10
            when  17 => return "0000011010101010001";   -- n   6, 1/n ~ 170/2^10
            when  18 => return "0000011110010010001";   -- n   7, 1/n ~ 146/2^10
            when  19 => return "0000011110010010001";   -- n   7, 1/n ~ 146/2^10
            when  20 => return "0000011110010010001";   -- n   7, 1/n ~ 146/2^10
            when  21 => return "0000011110010010001";   -- n   7, 1/n ~ 146/2^10
            when  22 => return "0000011110010010001";   -- n   7, 1/n ~ 146/2^10
            when  23 => return "0000011110010010001";   -- n   7, 1/n ~ 146/2^10
            when  24 => return "0000100010000000001";   -- n   8, 1/n ~ 128/2^10
            when  25 => return "0000100010000000001";   -- n   8, 1/n ~ 128/2^10
            when  26 => return "0000100010000000001";   -- n   8, 1/n ~ 128/2^10
            when  27 => return "0000100010000000001";   -- n   8, 1/n ~ 128/2^10
            when  28 => return "0000100111100011010";   -- n   9, 1/n ~ 227/2^11
            when  29 => return "0000100111100011010";   -- n   9, 1/n ~ 227/2^11
            when  30 => return "0000100111100011010";   -- n   9, 1/n ~ 227/2^11
            when  31 => return "0000100111100011010";   -- n   9, 1/n ~ 227/2^11
            when  32 => return "0000101011001100010";   -- n  10, 1/n ~ 204/2^11
            when  33 => return "0000101011001100010";   -- n  10, 1/n ~ 204/2^11
            when  34 => return "0000101011001100010";   -- n  10, 1/n ~ 204/2^11
            when  35 => return "0000101011001100010";   -- n  10, 1/n ~ 204/2^11
            when  36 => return "0000101110111010010";   -- n  11, 1/n ~ 186/2^11
            when  37 => return "0000101110111010010";   -- n  11, 1/n ~ 186/2^11
            when  38 => return "0000101110111010010";   -- n  11, 1/n ~ 186/2^11
            when  39 => return "0000110010101010010";   -- n  12, 1/n ~ 170/2^11
            when  40 => return "0000110010101010010";   -- n  12, 1/n ~ 170/2^11
            when  41 => return "0000110010101010010";   -- n  12, 1/n ~ 170/2^11
            when  42 => return "0000110110011101010";   -- n  13, 1/n ~ 157/2^11
            when  43 => return "0000110110011101010";   -- n  13, 1/n ~ 157/2^11
            when  44 => return "0000110110011101010";   -- n  13, 1/n ~ 157/2^11
            when  45 => return "0000111010010010010";   -- n  14, 1/n ~ 146/2^11
            when  46 => return "0000111010010010010";   -- n  14, 1/n ~ 146/2^11
            when  47 => return "0000111010010010010";   -- n  14, 1/n ~ 146/2^11
            when  48 => return "0000111110001000010";   -- n  15, 1/n ~ 136/2^11
            when  49 => return "0000111110001000010";   -- n  15, 1/n ~ 136/2^11
            when  50 => return "0001000010000000010";   -- n  16, 1/n ~ 128/2^11
            when  51 => return "0001000010000000010";   -- n  16, 1/n ~ 128/2^11
            when  52 => return "0001000111110000011";   -- n  17, 1/n ~ 240/2^12
            when  53 => return "0001000111110000011";   -- n  17, 1/n ~ 240/2^12
            when  54 => return "0001000111110000011";   -- n  17, 1/n ~ 240/2^12
            when  55 => return "0001001011100011011";   -- n  18, 1/n ~ 227/2^12
            when  56 => return "0001001011100011011";   -- n  18, 1/n ~ 227/2^12
            when  57 => return "0001001111010111011";   -- n  19, 1/n ~ 215/2^12
            when  58 => return "0001001111010111011";   -- n  19, 1/n ~ 215/2^12
            when  59 => return "0001010011001100011";   -- n  20, 1/n ~ 204/2^12
            when  60 => return "0001010111000011011";   -- n  21, 1/n ~ 195/2^12
            when  61 => return "0001010111000011011";   -- n  21, 1/n ~ 195/2^12
            when  62 => return "0001011010111010011";   -- n  22, 1/n ~ 186/2^12
            when  63 => return "0001011010111010011";   -- n  22, 1/n ~ 186/2^12
            when  64 => return "0001011110110010011";   -- n  23, 1/n ~ 178/2^12
            when  65 => return "0001100010101010011";   -- n  24, 1/n ~ 170/2^12
            when  66 => return "0001100010101010011";   -- n  24, 1/n ~ 170/2^12
            when  67 => return "0001100110100011011";   -- n  25, 1/n ~ 163/2^12
            when  68 => return "0001101010011101011";   -- n  26, 1/n ~ 157/2^12
            when  69 => return "0001101010011101011";   -- n  26, 1/n ~ 157/2^12
            when  70 => return "0001101110010111011";   -- n  27, 1/n ~ 151/2^12
            when  71 => return "0001110010010010011";   -- n  28, 1/n ~ 146/2^12
            when  72 => return "0001110110001101011";   -- n  29, 1/n ~ 141/2^12
            when  73 => return "0001110110001101011";   -- n  29, 1/n ~ 141/2^12
            when  74 => return "0001111010001000011";   -- n  30, 1/n ~ 136/2^12
            when  75 => return "0001111110000100011";   -- n  31, 1/n ~ 132/2^12
            when  76 => return "0010000010000000011";   -- n  32, 1/n ~ 128/2^12
            when  77 => return "0010000111111000100";   -- n  33, 1/n ~ 248/2^13
            when  78 => return "0010001011110000100";   -- n  34, 1/n ~ 240/2^13
            when  79 => return "0010001111101010100";   -- n  35, 1/n ~ 234/2^13
            when  80 => return "0010001111101010100";   -- n  35, 1/n ~ 234/2^13
            when  81 => return "0010010011100011100";   -- n  36, 1/n ~ 227/2^13
            when  82 => return "0010010111011101100";   -- n  37, 1/n ~ 221/2^13
            when  83 => return "0010011111010010100";   -- n  39, 1/n ~ 210/2^13
            when  84 => return "0010100011001100100";   -- n  40, 1/n ~ 204/2^13
            when  85 => return "0010100111000111100";   -- n  41, 1/n ~ 199/2^13
            when  86 => return "0010101011000011100";   -- n  42, 1/n ~ 195/2^13
            when  87 => return "0010101110111110100";   -- n  43, 1/n ~ 190/2^13
            when  88 => return "0010110010111010100";   -- n  44, 1/n ~ 186/2^13
            when  89 => return "0010110110110110100";   -- n  45, 1/n ~ 182/2^13
            when  90 => return "0010111110101110100";   -- n  47, 1/n ~ 174/2^13
            when  91 => return "0011000010101010100";   -- n  48, 1/n ~ 170/2^13
            when  92 => return "0011000110100111100";   -- n  49, 1/n ~ 167/2^13
            when  93 => return "0011001110100000100";   -- n  51, 1/n ~ 160/2^13
            when  94 => return "0011010010011101100";   -- n  52, 1/n ~ 157/2^13
            when  95 => return "0011010110011010100";   -- n  53, 1/n ~ 154/2^13
            when  96 => return "0011011110010100100";   -- n  55, 1/n ~ 148/2^13
            when  97 => return "0011100010010010100";   -- n  56, 1/n ~ 146/2^13
            when  98 => return "0011101010001101100";   -- n  58, 1/n ~ 141/2^13
            when  99 => return "0011110010001000100";   -- n  60, 1/n ~ 136/2^13
            when 100 => return "0011110110000110100";   -- n  61, 1/n ~ 134/2^13
            when 101 => return "0011111110000010100";   -- n  63, 1/n ~ 130/2^13
            when 102 => return "0100000111111100101";   -- n  65, 1/n ~ 252/2^14
            when 103 => return "0100001011111000101";   -- n  66, 1/n ~ 248/2^14
            when 104 => return "0100010011110000101";   -- n  68, 1/n ~ 240/2^14
            when 105 => return "0100011011101010101";   -- n  70, 1/n ~ 234/2^14
            when 106 => return "0100100011100011101";   -- n  72, 1/n ~ 227/2^14
            when 107 => return "0100101011011101101";   -- n  74, 1/n ~ 221/2^14
            when 108 => return "0100110011010111101";   -- n  76, 1/n ~ 215/2^14
            when 109 => return "0100111011010010101";   -- n  78, 1/n ~ 210/2^14
            when 110 => return "0101000011001100101";   -- n  80, 1/n ~ 204/2^14
            when 111 => return "0101001111000101101";   -- n  83, 1/n ~ 197/2^14
            when 112 => return "0101010111000000101";   -- n  85, 1/n ~ 192/2^14
            when 113 => return "0101011110111100101";   -- n  87, 1/n ~ 188/2^14
            when 114 => return "0101101010110110101";   -- n  90, 1/n ~ 182/2^14
            when 115 => return "0101110010110010101";   -- n  92, 1/n ~ 178/2^14
            when 116 => return "0101111110101100101";   -- n  95, 1/n ~ 172/2^14
            when 117 => return "0110000110101000101";   -- n  97, 1/n ~ 168/2^14
            when 118 => return "0110010010100011101";   -- n 100, 1/n ~ 163/2^14
            when 119 => return "0110011110011111101";   -- n 103, 1/n ~ 159/2^14
            when 120 => return "0110101010011010101";   -- n 106, 1/n ~ 154/2^14
            when 121 => return "0110110110010110101";   -- n 109, 1/n ~ 150/2^14
            when 122 => return "0111000010010010101";   -- n 112, 1/n ~ 146/2^14
            when 123 => return "0111001110001110101";   -- n 115, 1/n ~ 142/2^14
            when 124 => return "0111011010001010101";   -- n 118, 1/n ~ 138/2^14
            when 125 => return "0111100110000111101";   -- n 121, 1/n ~ 135/2^14
            when 126 => return "0111110110000011101";   -- n 125, 1/n ~ 131/2^14
            when 127 => return "1000000010000000101";   -- n 128, 1/n ~ 128/2^14
            when others => return "1000000010000000101";
        end case;
    end function;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    --------------------------------------------------------------------------
    -- Line buffers: Y, U, V (base), G (glow, half rate); blur ring
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal lbG0, lbG1 : t_lb_c := (others => x"10");
    signal r_ring     : t_ring := (others => (others => '0'));
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdG0, s_rdG1 : std_logic_vector(7 downto 0) := x"10";
    signal s_rd           : std_logic_vector(15 downto 0) := (others => '0');
    -- No read ever needs the address being written: the line buffers are read
    -- only from the bank not being written while the output is active (the
    -- glow's hblank tail lands on a bank whose reads are don't-care until the
    -- next line starts), and the ring reads n >= 4 behind its write pointer.
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;
    attribute no_rw_check of lbG0 : signal is true;
    attribute no_rw_check of lbG1 : signal is true;
    attribute no_rw_check of r_ring : signal is true;

    --------------------------------------------------------------------------
    -- Controls
    --------------------------------------------------------------------------
    signal s_k_axis, s_k_folds, s_k_slide, s_k_glow, s_k_thr, s_k_radius : unsigned(9 downto 0);
    signal s_sw_gunm, s_sw_neg, s_sw_tint, s_sw_seams, s_sw_soft : std_logic;
    signal s_p_sweep : unsigned(9 downto 0);
    signal s_kax, s_ksl, s_kp : unsigned(9 downto 0) := (others => '0');
    signal s_kf_seen : std_logic := '0';
    signal s_folds  : unsigned(1 downto 0) := "01";
    signal s_none   : std_logic := '0';
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_sg     : unsigned(8 downto 0) := to_unsigned(64, 9);    -- seam gain 64 + gain
    signal s_thr_a, s_thr_b : unsigned(9 downto 0) := (others => '0');
    signal s_thr    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_phinc  : unsigned(15 downto 0) := to_unsigned(24, 16);
    -- blur width
    signal s_rtab   : t_rt := (others => '0');
    signal s_sd     : std_logic := '0';
    -- resolution scaling (static per mode, decoded every clock)
    signal s_vxs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ridx   : unsigned(6 downto 0)  := (others => '0');   -- Radius table index
    signal s_Px     : unsigned(12 downto 0) := (others => '0');   -- pane width in half units
    signal s_q168, s_q3r : unsigned(15 downto 0) := (others => '0');
    signal s_sqx    : unsigned(7 downto 0)  := (others => '0');   -- Slide speed, 720p / SD
    signal s_n      : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_rm     : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_sh     : unsigned(2 downto 0) := "010";
    signal s_rofs   : unsigned(7 downto 0) := to_unsigned(243, 8);
    signal s_gofs   : unsigned(11 downto 0) := to_unsigned(23, 12);
    signal s_ghi    : unsigned(11 downto 0) := to_unsigned(503, 12);

    --------------------------------------------------------------------------
    -- Per-frame sequencer: axis, sweep, slide, mirror-counter start state
    --------------------------------------------------------------------------
    signal s_vstep  : unsigned(3 downto 0) := "1111";
    signal s_ph     : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2, s_sin : signed(9 downto 0) := (others => '0');
    signal s_wmax_run : unsigned(10 downto 0) := (others => '0');
    signal s_wmax, s_wf : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wm1    : unsigned(10 downto 0) := to_unsigned(479, 11);
    signal s_w16    : unsigned(14 downto 0) := to_unsigned(7680, 15);
    signal s_mh, s_ml, s_dh, s_dl : unsigned(15 downto 0) := (others => '0');
    signal s_kw, s_dw : unsigned(20 downto 0) := (others => '0');
    signal s_abase  : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_depth  : unsigned(9 downto 0) := (others => '0');
    signal s_amin   : unsigned(10 downto 0) := to_unsigned(30, 11);
    signal s_amax   : unsigned(10 downto 0) := to_unsigned(455, 11);
    signal s_swh, s_swl : signed(15 downto 0) := (others => '0');
    signal s_sw     : signed(20 downto 0) := (others => '0');
    signal s_ax     : signed(12 downto 0) := to_signed(240, 13);
    signal s_P      : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_twoP   : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_r      : unsigned(11 downto 0) := (others => '0');
    signal s_dr, s_dd : signed(12 downto 0) := (others => '0');
    signal s_u0     : unsigned(10 downto 0) := (others => '0');
    signal s_dir0   : std_logic := '1';
    signal s_t0     : unsigned(10 downto 0) := (others => '0');
    signal s_wrapok : std_logic := '0';
    signal s_sk     : unsigned(1 downto 0) := "00";
    -- slide: offset in 1/16 px, speed = sign * (|K3 - 512| - 16)^2 / 1024
    signal s_sds    : signed(10 downto 0) := (others => '0');
    signal s_asd0   : unsigned(9 downto 0) := (others => '0');
    signal s_asd    : unsigned(8 downto 0) := (others => '0');
    signal s_sqh    : unsigned(13 downto 0) := (others => '0');
    signal s_sql    : unsigned(12 downto 0) := (others => '0');
    signal s_sq     : unsigned(17 downto 0) := (others => '0');
    signal s_spd    : signed(9 downto 0) := (others => '0');
    signal s_off    : unsigned(14 downto 0) := (others => '0');
    signal s_offv, s_offlo, s_offhi : signed(16 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Input / tracking / base write port
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(467, 11);   -- line_width - 1 - C_EDGE
    signal s_wpar, s_rbank, s_gpar : std_logic := '0';
    signal s_bcnt    : unsigned(9 downto 0) := (others => '1');        -- blank clocks, saturating
    signal s_wy1     : unsigned(9 downto 0) := (others => '0');
    signal s_wu1, s_wv1 : unsigned(7 downto 0) := x"80";
    signal s_wx1     : unsigned(10 downto 0) := (others => '0');
    signal s_we1, s_wp1 : std_logic := '0';

    --------------------------------------------------------------------------
    -- Blur: two cascaded running boxes of width n over one ring
    --------------------------------------------------------------------------
    signal s_yfirst, s_ylast : unsigned(7 downto 0) := x"10";
    signal s_bge     : std_logic := '0';
    signal s_v1      : unsigned(7 downto 0) := (others => '0');
    signal s_vc1     : unsigned(11 downto 0) := (others => '1');
    signal s_wptr    : unsigned(7 downto 0) := (others => '0');
    signal s_raddr   : unsigned(7 downto 0) := x"FE";
    signal s_rd2     : std_logic_vector(15 downto 0) := (others => '0');
    signal s_a1, s_a2 : unsigned(14 downto 0) := (others => '0');
    signal s_p1l, s_p1h, s_p2l, s_p2h : unsigned(18 downto 0) := (others => '0');
    signal s_p1s, s_p2s : unsigned(22 downto 0) := (others => '0');
    signal s_b1, s_b2 : unsigned(7 downto 0) := (others => '0');
    signal s_gaddr   : unsigned(11 downto 0) := (others => '0');
    signal s_gwe     : std_logic := '0';
    signal s_gwd     : unsigned(7 downto 0) := x"10";

    --------------------------------------------------------------------------
    -- Mirror counter (stage 0) and address / seam chain (stage n = n clocks
    -- after the input sample)
    --------------------------------------------------------------------------
    signal s_du, s_dt : unsigned(10 downto 0) := (others => '0');
    signal s_ddir    : std_logic := '1';
    signal s_src1, s_ga1 : unsigned(10 downto 0) := (others => '0');
    signal s_odd1, s_odd2, s_odd3 : std_logic := '0';
    signal s_da1, s_db1, s_xl1, s_xr1, s_tl1, s_tr1 : unsigned(10 downto 0) := (others => '1');
    signal s_xget1   : std_logic := '0';
    -- seam distances in half units (x step x px), 511 = none within reach
    signal s_ma2, s_mb2, s_mw2, s_m3, s_mw3, s_d4 : unsigned(8 downto 0) := (others => '1');
    signal s_rb2     : std_logic := '0';
    signal s_r5      : unsigned(4 downto 0) := (others => '0');
    signal s_rr6     : unsigned(4 downto 0) := (others => '0');
    signal s_sp7     : unsigned(13 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel chain
    --------------------------------------------------------------------------
    signal s_wy3     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wu3, s_wv3 : unsigned(7 downto 0) := x"80";
    signal s_wg3     : unsigned(7 downto 0) := x"10";
    signal s_gd4     : signed(11 downto 0) := (others => '0');
    signal s_y4, s_u4, s_v4 : signed(11 downto 0) := (others => '0');
    signal s_gx5     : unsigned(9 downto 0) := (others => '0');
    type t_c is array (5 to 9) of signed(11 downto 0);
    signal s_yc, s_uc, s_vc : t_c := (others => (others => '0'));
    signal s_gph6, s_gpl6 : unsigned(13 downto 0) := (others => '0');
    signal s_gp7     : unsigned(17 downto 0) := (others => '0');
    signal s_gt8     : unsigned(12 downto 0) := (others => '0');
    signal s_ys9     : signed(13 downto 0) := (others => '0');
    signal s_tu9, s_tv9 : unsigned(9 downto 0) := (others => '0');
    signal s_ys10, s_yk10 : signed(13 downto 0) := (others => '0');
    signal s_us10, s_vs10 : signed(12 downto 0) := (others => '0');
    signal s_y11     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y13     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_axis   <= unsigned(registers_in(0));
    s_k_folds  <= unsigned(registers_in(1));
    s_k_slide  <= unsigned(registers_in(2));
    s_k_glow   <= unsigned(registers_in(3));
    s_k_thr    <= unsigned(registers_in(4));
    s_k_radius <= unsigned(registers_in(5));
    s_sw_gunm  <= registers_in(6)(0);
    s_sw_neg   <= registers_in(6)(1);
    s_sw_tint  <= registers_in(6)(2);
    s_sw_seams <= registers_in(6)(3);
    s_sw_soft  <= registers_in(6)(4);
    s_p_sweep  <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q  <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;      -- the ROM register is absorbed into the EBR
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Tracking, controls, base write port, per-frame sequencer, sync delay
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_abs : signed(10 downto 0);
        variable v_sk  : unsigned(1 downto 0);
        variable v_ri  : unsigned(6 downto 0);
        variable v_q   : unsigned(15 downto 0);
        variable v_qh  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
                s_bcnt <= (others => '0');
            else
                s_rx <= (others => '0');
                if s_bcnt /= 1023 then
                    s_bcnt <= s_bcnt + 1;
                end if;
            end if;

            -- base write port: input one clock deep
            s_wy1 <= unsigned(data_in.y);
            s_wu1 <= unsigned(data_in.u(9 downto 2));
            s_wv1 <= unsigned(data_in.v(9 downto 2));
            s_wx1 <= s_rx;
            s_we1 <= data_in.avid;
            s_wp1 <= s_wpar;

            -- controls, re-registered off the SPI bus
            s_kax <= s_k_axis;
            s_ksl <= s_k_slide;
            s_kp  <= s_p_sweep;
            s_phinc <= to_unsigned(24, 16) + resize(s_kp(9 downto 2), 16);
            -- Folds: the firmware loads a labelled default as its label INDEX
            -- (0..3); read it as an index until the knob first shows a real
            -- position
            if s_k_folds >= 4 then
                s_kf_seen <= '1';
            end if;
            if s_kf_seen = '0' and s_k_folds < 4 then
                s_folds <= s_k_folds(1 downto 0);
            else
                s_folds <= s_k_folds(9 downto 8);
            end if;
            if s_folds = 0 then
                s_none <= '1';
            else
                s_none <= '0';
            end if;
            s_gain  <= s_k_glow(9 downto 2);
            s_sg    <= resize(s_gain, 9) + 64;
            -- threshold spans black .. ~white (64 + 0.8125 * K5)
            s_thr_a <= s_k_thr - shift_right(s_k_thr, 2);
            s_thr_b <= shift_right(s_k_thr, 4) + 64;
            s_thr   <= s_thr_a + s_thr_b;
            -- blur width: latched in vblank just ahead of the running-sum re-zero
            -- resolution scaling: x step (static per mode); Radius index -15
            -- at 720p (x 2/3), -26 at SD (x 1/2), floored at 0
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            v_ri := s_k_radius(9 downto 3);
            case s_vxs is
                when "011"  => if v_ri < 15 then s_ridx <= (others => '0'); else s_ridx <= v_ri - 15; end if;
                when "100"  => if v_ri < 26 then s_ridx <= (others => '0'); else s_ridx <= v_ri - 26; end if;
                when others => s_ridx <= v_ri;
            end case;
            s_rtab <= f_rtab(s_ridx);
            -- pane width in half units (seam narrowing)
            case s_vxs is
                when "011"  => s_Px <= resize(s_P, 13) + shift_left(resize(s_P, 13), 1);
                when "100"  => s_Px <= shift_left(resize(s_P, 13), 2);
                when others => s_Px <= shift_left(resize(s_P, 13), 1);
            end case;
            -- Slide speed in 1080-line px -> real px (720p x 2/3 = (q*171 +
            -- 128) >> 8, SD x 1/2), free-running from last frame's s_sq
            s_q168 <= shift_left(resize(s_sq(17 downto 10), 16), 7) + shift_left(resize(s_sq(17 downto 10), 16), 5) + shift_left(resize(s_sq(17 downto 10), 16), 3);
            s_q3r  <= shift_left(resize(s_sq(17 downto 10), 16), 1) + resize(s_sq(17 downto 10), 16) + 128;
            v_q  := s_q168 + s_q3r;
            v_qh := resize(s_sq(17 downto 10), 9) + 1;
            if s_vxs = "011" then
                s_sqx <= v_q(15 downto 8);
            else
                s_sqx <= v_qh(8 downto 1);
            end if;
            if s_line_width > 1024 then
                s_sd <= '0';
            else
                s_sd <= '1';
            end if;
            if s_bcnt = 511 then
                if s_sd = '1' and unsigned(s_rtab(18 downto 11)) > 64 then
                    s_n  <= to_unsigned(64, 8);     -- SD hblank holds 2n-1 of padding only up to 64
                    s_rm <= to_unsigned(128, 8);
                    s_sh <= "100";
                else
                    s_n  <= unsigned(s_rtab(18 downto 11));
                    s_rm <= unsigned(s_rtab(10 downto 3));
                    s_sh <= unsigned(s_rtab(2 downto 0));
                end if;
            end if;
            s_rofs <= to_unsigned(3, 8) - s_n;
            s_gofs <= resize(s_n, 12) + C_GLAG;
            s_ghi  <= s_gofs + resize(s_line_width, 12);

            ------------------------------------------------------------------
            -- Per-frame sequencer (one small op per state; products split)
            --   axis  a = K1 * w / 1024 + sin * depth, clamped [w/16, w - 25]
            --   depth = P12 * w / 2048 (to +-half the width)
            --   P     = a >> (Folds - 1)
            --   slide offset (1/16 px) += speed, wrapped to the width
            --   counter start: r = off mod 2P -> u0 = tri(off), direction
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_wf <= s_wmax;
                    s_sin_addr <= s_ph(15 downto 8);
                    s_sds <= signed('0' & s_ksl) - to_signed(512, 11);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_mh <= s_kax(9 downto 5) * s_wf;
                    s_ml <= s_kax(4 downto 0) * s_wf;
                    s_dh <= s_kp(9 downto 5) * s_wf;
                    s_dl <= s_kp(4 downto 0) * s_wf;
                    v_abs  := abs(s_sds);
                    s_asd0 <= unsigned(v_abs(9 downto 0));
                    s_wm1  <= s_wf - 1;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_kw <= shift_left(resize(s_mh, 21), 5) + resize(s_ml, 21);
                    s_dw <= shift_left(resize(s_dh, 21), 5) + resize(s_dl, 21);
                    if s_asd0 > 16 then
                        s_asd <= resize(s_asd0 - 16, 9);
                    else
                        s_asd <= (others => '0');
                    end if;
                    s_w16 <= s_wf & "0000";
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_abase <= s_kw(20 downto 10);
                    s_depth <= s_dw(20 downto 11);
                    s_sin   <= s_sin_q2;
                    s_amin  <= shift_right(s_wf, 4);
                    s_amax  <= s_wf - (2 * C_EDGE + 1);
                    s_sqh   <= s_asd * s_asd(8 downto 4);
                    s_sql   <= s_asd * s_asd(3 downto 0);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_swh <= s_sin * signed('0' & s_depth(9 downto 5));
                    s_swl <= s_sin * signed('0' & s_depth(4 downto 0));
                    s_sq  <= shift_left(resize(s_sqh, 18), 4) + resize(s_sql, 18);
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_sw <= shift_left(resize(s_swh, 21), 5) + resize(s_swl, 21);
                    if s_vxs = "010" then
                        if s_sds < 0 then
                            s_spd <= -signed(resize(s_sq(17 downto 10), 10));
                        else
                            s_spd <= signed(resize(s_sq(17 downto 10), 10));
                        end if;
                    elsif s_sds < 0 then
                        s_spd <= -signed(resize(s_sqx, 10));
                    else
                        s_spd <= signed(resize(s_sqx, 10));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_ax   <= signed(resize(s_abase, 13)) + resize(shift_right(s_sw, 8), 13);
                    s_offv <= signed(resize(s_off, 17)) + resize(s_spd, 17);
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    if s_ax < signed(resize(s_amin, 13)) then
                        s_ax <= signed(resize(s_amin, 13));
                    end if;
                    s_offlo <= s_offv + signed(resize(s_w16, 17));
                    s_offhi <= s_offv - signed(resize(s_w16, 17));
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    if s_ax > signed(resize(s_amax, 13)) then
                        s_ax <= signed(resize(s_amax, 13));
                    end if;
                    if s_offv < 0 then
                        s_off <= unsigned(s_offlo(14 downto 0));
                    elsif s_offhi >= 0 then
                        s_off <= unsigned(s_offhi(14 downto 0));
                    else
                        s_off <= unsigned(s_offv(14 downto 0));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    case to_integer(s_folds) is
                        when 2      => s_P <= unsigned(s_ax(11 downto 1));
                        when 3      => s_P <= unsigned(s_ax(12 downto 2));
                        when others => s_P <= unsigned(s_ax(10 downto 0));
                    end case;
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    s_twoP <= s_P & '0';
                    if s_off(14 downto 4) > s_wm1 then
                        s_r  <= (others => '0');
                        s_t0 <= (others => '0');
                    else
                        s_r  <= resize(s_off(14 downto 4), 12);
                        s_t0 <= s_off(14 downto 4);
                    end if;
                    if s_off(14 downto 4) = 0 then
                        s_wrapok <= '0';
                    else
                        s_wrapok <= '1';
                    end if;
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    s_dr <= signed(resize(s_r, 13)) - signed(resize(s_twoP, 13));
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    if s_dr >= 0 then
                        s_r <= unsigned(s_dr(11 downto 0));
                        s_vstep <= to_unsigned(11, 4);
                    else
                        s_vstep <= s_vstep + 1;
                    end if;
                when 13 =>
                    s_dd <= signed(resize(s_r, 13)) - signed(resize(s_P, 13));
                    s_vstep <= s_vstep + 1;
                when 14 =>
                    if s_dd <= 0 then
                        s_u0   <= s_r(10 downto 0);
                        s_dir0 <= '1';
                    else
                        s_u0   <= s_P - unsigned(s_dd(10 downto 0));
                        s_dir0 <= '0';
                    end if;
                    -- seam half-width 32 units (32 px at 1080, 16 at SD), halved
                    -- again for each octave the panes fall under 64 units
                    v_sk := "00";
                    if s_Px < 128 then
                        v_sk := v_sk + 1;
                    end if;
                    if s_Px < 64 then
                        v_sk := v_sk + 1;
                    end if;
                    s_sk <= v_sk;
                    s_vstep <= (others => '1');
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Line / frame events
            ------------------------------------------------------------------
            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                if s_rx > s_wmax_run then
                    s_wmax_run <= s_rx;
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_ph       <= s_ph + s_phinc;
                    s_wmax     <= s_wmax_run;      -- widest line of the field
                    s_wmax_run <= (others => '0');
                    s_vstep    <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Blur.  Box k: a(t) = a(t-1) + x(t) - x(t-n), with x(t-n) read back from
    -- the ring; mean = a * rm >> (9 + sh).  Adds and subtracts are the same
    -- stored values, so the sums are exact; the vblank zone re-zeroes them
    -- (and the ring) after n is latched.
    --------------------------------------------------------------------------
    p_blur : process(clk)
        variable v_x    : unsigned(7 downto 0);
        variable v_zone : boolean;
    begin
        if rising_edge(clk) then
            v_zone := s_bcnt(9) = '1' and s_bcnt(8) = '0';
            -- input, padded: the capture's black edge columns and the blanking
            -- take the line's inset edge pixels -- its right edge first, then
            -- (n clocks on) its left edge as the next line's lead-in
            if v_zone then
                v_x := (others => '0');
            elsif data_in.avid = '1' then
                if s_rx < C_EDGE then
                    v_x := s_yfirst;
                elsif s_rx > s_w_edge then
                    v_x := s_ylast;
                else
                    v_x := unsigned(data_in.y(9 downto 2));
                end if;
            elsif s_bge = '1' then
                v_x := s_yfirst;
            else
                v_x := s_ylast;
            end if;
            s_v1 <= v_x;
            if data_in.avid = '1' and s_rx = C_EDGE then
                s_yfirst <= unsigned(data_in.y(9 downto 2));
            end if;
            if data_in.avid = '1' and s_rx = s_w_edge then
                s_ylast <= unsigned(data_in.y(9 downto 2));
            end if;
            if s_bcnt >= s_n then
                s_bge <= '1';
            else
                s_bge <= '0';
            end if;
            -- column of s_v1; runs on into the blanking for the glow's tail.
            -- The glow bank is captured per line so the tail survives the
            -- hsync bank toggle.
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_vc1  <= (others => '0');
                s_gpar <= s_wpar;
            elsif s_vc1 /= 4095 then
                s_vc1 <= s_vc1 + 1;
            end if;

            -- ring: written every clock at s_wptr, read n back (2-clock read path)
            s_wptr  <= s_wptr + 1;
            s_raddr <= s_wptr + s_rofs;
            s_rd2   <= s_rd;

            -- box 1 over the padded luma
            if v_zone then
                s_a1 <= (others => '0');
            else
                s_a1 <= s_a1 + resize(s_v1, 15) - resize(unsigned(s_rd2(15 downto 8)), 15);
            end if;
            s_p1l <= s_a1 * s_rm(3 downto 0);
            s_p1h <= s_a1 * s_rm(7 downto 4);
            s_p1s <= shift_left(resize(s_p1h, 23), 4) + resize(s_p1l, 23);
            case to_integer(s_sh) is
                when 0 => s_b1 <= s_p1s(16 downto 9);
                when 1 => s_b1 <= s_p1s(17 downto 10);
                when 2 => s_b1 <= s_p1s(18 downto 11);
                when 3 => s_b1 <= s_p1s(19 downto 12);
                when 4 => s_b1 <= s_p1s(20 downto 13);
                when others => s_b1 <= s_p1s(21 downto 14);
            end case;

            -- box 2 over box 1's mean
            if v_zone then
                s_a2 <= (others => '0');
            else
                s_a2 <= s_a2 + resize(s_b1, 15) - resize(unsigned(s_rd2(7 downto 0)), 15);
            end if;
            s_p2l <= s_a2 * s_rm(3 downto 0);
            s_p2h <= s_a2 * s_rm(7 downto 4);
            s_p2s <= shift_left(resize(s_p2h, 23), 4) + resize(s_p2l, 23);
            case to_integer(s_sh) is
                when 0 => s_b2 <= s_p2s(16 downto 9);
                when 1 => s_b2 <= s_p2s(17 downto 10);
                when 2 => s_b2 <= s_p2s(18 downto 11);
                when 3 => s_b2 <= s_p2s(19 downto 12);
                when 4 => s_b2 <= s_p2s(20 downto 13);
                when others => s_b2 <= s_p2s(21 downto 14);
            end case;

            -- glow write port: s_b2 is centred on column s_vc1 - (n + 7)
            s_gaddr <= s_vc1 - s_gofs;
            if s_vc1 >= s_gofs and s_vc1 < s_ghi then
                s_gwe <= '1';
            else
                s_gwe <= '0';
            end if;
            s_gwd <= s_b2;
        end if;
    end process p_blur;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rd <= r_ring(to_integer(s_raddr));
            r_ring(to_integer(s_wptr)) <= std_logic_vector(s_v1) & std_logic_vector(s_b1);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Mirror counter and read addresses (stages 0..3)
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 0: two-mirror counter, in step with s_rx.  u bounces between
            -- 0 and P while the source column t walks the line; t wraps at the
            -- width (the slide), where u restarts at the source's left edge.
            if data_in.avid = '0' then
                s_dt   <= s_t0;
                s_du   <= s_u0;
                s_ddir <= s_dir0;
            elsif s_dt = s_wm1 then
                s_dt   <= (others => '0');
                s_du   <= (others => '0');
                s_ddir <= '1';
            else
                s_dt <= s_dt + 1;
                if s_ddir = '1' then
                    if s_du = s_P then
                        s_du   <= s_du - 1;
                        s_ddir <= '0';
                    else
                        s_du <= s_du + 1;
                    end if;
                else
                    if s_du = 0 then
                        s_du   <= s_du + 1;
                        s_ddir <= '1';
                    else
                        s_du <= s_du - 1;
                    end if;
                end if;
            end if;

            -- stage 1: read addresses (panes read the source inset by C_EDGE;
            -- the unmirrored glow reads at the screen column)
            if s_none = '1' then
                s_src1 <= s_dt;
            else
                s_src1 <= s_du + C_EDGE;
            end if;
            if s_sw_gunm = '1' then
                s_ga1 <= s_rx;
            elsif s_none = '1' then
                s_ga1 <= s_dt;
            else
                s_ga1 <= s_du + C_EDGE;
            end if;
            s_odd1 <= (not s_none) and (not s_ddir);
            -- seam distances to the turning points ahead (rising: the peak;
            -- falling: the floor) and behind.  One counts only if it lies inside
            -- the screen and is a real mirror, not the source's own edge where t
            -- wraps -- so no half-glow at the borders.  The slide's wrap glows
            -- too, on whichever side of it the pixel is.
            if s_ddir = '1' then
                s_da1 <= s_P - s_du;
                s_db1 <= s_du;
            else
                s_da1 <= s_du;
                s_db1 <= s_P - s_du;
            end if;
            s_xl1 <= s_rx;
            s_xr1 <= s_wm1 - s_rx;
            s_tl1 <= s_dt;
            s_tr1 <= s_wm1 - s_dt;
            if s_rx >= s_dt then
                s_xget1 <= '1';
            else
                s_xget1 <= '0';
            end if;

            -- stage 2: validity (on screen, before the wrap / after the source edge)
            s_odd2 <= s_odd1;
            s_rb2  <= s_rbank;
            -- (distances in half units, saturated: see f_sdist)
            s_ma2 <= f_sdist(s_da1, s_none = '0' and s_da1 < s_xr1 and s_da1 <= s_tr1, s_vxs);
            s_mb2 <= f_sdist(s_db1, s_none = '0' and s_db1 < s_xl1 and s_db1 < s_tl1, s_vxs);
            if s_xget1 = '1' then
                s_mw2 <= f_sdist(s_tl1, s_wrapok = '1', s_vxs);
            else
                s_mw2 <= f_sdist(s_tr1, s_wrapok = '1', s_vxs);
            end if;

            -- stages 3..4: nearest seam
            s_odd3 <= s_odd2;
            if s_mb2 < s_ma2 then
                s_m3 <= s_mb2;
            else
                s_m3 <= s_ma2;
            end if;
            s_mw3 <= s_mw2;
            if s_mw3 < s_m3 then
                s_d4 <= s_mw3;
            else
                s_d4 <= s_m3;
            end if;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers.  Base written one clock after the input; glow written at
    -- its centre column through the line and its hblank tail.  Reads issued
    -- at stage 1, data at stage 2.
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_src1));
            if s_we1 = '1' and s_wp1 = '0' then
                lbY0(to_integer(s_wx1)) <= std_logic_vector(s_wy1);
            end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_src1));
            if s_we1 = '1' and s_wp1 = '1' then
                lbY1(to_integer(s_wx1)) <= std_logic_vector(s_wy1);
            end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_src1(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '0' then
                lbU0(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wu1);
            end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_src1(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '1' then
                lbU1(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wu1);
            end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_src1(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '0' then
                lbV0(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wv1);
            end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_src1(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '1' then
                lbV1(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wv1);
            end if;
        end if;
    end process;
    p_lbG0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG0 <= lbG0(to_integer(s_ga1(10 downto 1)));
            if s_gwe = '1' and s_gpar = '0' then
                lbG0(to_integer(s_gaddr(10 downto 1))) <= std_logic_vector(s_gwd);
            end if;
        end if;
    end process;
    p_lbG1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG1 <= lbG1(to_integer(s_ga1(10 downto 1)));
            if s_gwe = '1' and s_gpar = '1' then
                lbG1(to_integer(s_gaddr(10 downto 1))) <= std_logic_vector(s_gwd);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 3..14: fetch, glow excess, gains, seam ridge, tint, knee,
    -- smoothing, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_g10 : unsigned(9 downto 0);
        variable v_u10, v_v10 : unsigned(9 downto 0);
        variable v_rr  : unsigned(9 downto 0);
        variable v_s   : unsigned(11 downto 0);
        variable v_r7  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 3: bank select
            if s_rb2 = '1' then
                s_wy3 <= unsigned(s_rdY1);
                s_wu3 <= unsigned(s_rdU1);
                s_wv3 <= unsigned(s_rdV1);
                s_wg3 <= unsigned(s_rdG1);
            else
                s_wy3 <= unsigned(s_rdY0);
                s_wu3 <= unsigned(s_rdU0);
                s_wv3 <= unsigned(s_rdV0);
                s_wg3 <= unsigned(s_rdG0);
            end if;

            -- stage 4: glow excess (subtract here, clip next); base picture
            -- (soft = its blur; negative panes inverted about black / white)
            v_g10 := s_wg3 & "00";
            v_u10 := s_wu3 & "00";
            v_v10 := s_wv3 & "00";
            s_gd4 <= signed(resize(v_g10, 12)) - signed(resize(s_thr, 12));
            if s_sw_soft = '1' then
                s_y4 <= signed(resize(v_g10, 12));
            elsif s_sw_neg = '1' and s_odd3 = '1' then
                s_y4 <= to_signed(1004, 12) - signed(resize(s_wy3, 12));
            else
                s_y4 <= signed(resize(s_wy3, 12));
            end if;
            if s_sw_neg = '1' and s_odd3 = '1' then
                s_u4 <= to_signed(1024, 12) - signed(resize(v_u10, 12));
                s_v4 <= to_signed(1024, 12) - signed(resize(v_v10, 12));
            else
                s_u4 <= signed(resize(v_u10, 12));
                s_v4 <= signed(resize(v_v10, 12));
            end if;
            -- stage 5: clip the excess; carry the base
            if s_gd4 < 0 then
                s_gx5 <= (others => '0');
            else
                s_gx5 <= unsigned(s_gd4(9 downto 0));
            end if;
            s_yc(5) <= s_y4;
            s_uc(5) <= s_u4;
            s_vc(5) <= s_v4;
            for i in 6 to 9 loop
                s_yc(i) <= s_yc(i - 1);
                s_uc(i) <= s_uc(i - 1);
                s_vc(i) <= s_vc(i - 1);
            end loop;
            -- seam ridge (64 - (d << k)) / 2 with d in half units (1080: 32 -
            -- (px << k)), capped at 31, zero beyond
            if s_d4 = 0 then
                s_r5 <= "11111";
            else
                case to_integer(s_sk) is
                    when 0 =>
                        if s_d4 < 64 then
                            v_r7 := to_unsigned(64, 7) - ('0' & s_d4(5 downto 0));
                            s_r5 <= v_r7(5 downto 1);
                        else
                            s_r5 <= (others => '0');
                        end if;
                    when 1 =>
                        if s_d4 < 32 then
                            s_r5 <= resize(to_unsigned(32, 6) - ('0' & s_d4(4 downto 0)), 5);
                        else
                            s_r5 <= (others => '0');
                        end if;
                    when 2 =>
                        if s_d4 < 16 then
                            s_r5 <= resize(to_unsigned(32, 6) - ('0' & s_d4(3 downto 0) & '0'), 5);
                        else
                            s_r5 <= (others => '0');
                        end if;
                    when others =>
                        if s_d4 < 8 then
                            s_r5 <= resize(to_unsigned(32, 6) - ('0' & s_d4(2 downto 0) & "00"), 5);
                        else
                            s_r5 <= (others => '0');
                        end if;
                end case;
            end if;

            -- stage 6: glow gain (Q6, split on the gain nibbles); ridge squared
            s_gph6 <= s_gx5 * s_gain(7 downto 4);
            s_gpl6 <= s_gx5 * s_gain(3 downto 0);
            v_rr   := s_r5 * s_r5;
            s_rr6  <= v_rr(9 downto 5);

            -- stage 7
            s_gp7 <= shift_left(resize(s_gph6, 18), 4) + resize(s_gpl6, 18);
            s_sp7 <= s_rr6 * s_sg;

            -- stage 8: total glow; the seams add their ridge
            if s_sw_seams = '1' then
                s_gt8 <= resize(s_gp7(17 downto 6), 13) + resize(s_sp7(13 downto 3), 13);
            else
                s_gt8 <= resize(s_gp7(17 downto 6), 13);
            end if;

            -- stage 9: glow onto luma; warm tint terms
            s_ys9 <= resize(s_yc(8), 14) + signed(resize(s_gt8, 14));
            if s_sw_tint = '1' then
                s_tu9 <= resize(s_gt8(12 downto 4), 10);
                s_tv9 <= s_gt8(12 downto 3);
            else
                s_tu9 <= (others => '0');
                s_tv9 <= (others => '0');
            end if;

            -- stage 10
            s_us10 <= resize(s_uc(9), 13) + signed(resize(s_tu9, 13));
            s_vs10 <= resize(s_vc(9), 13) - signed(resize(s_tv9, 13));
            s_yk10 <= s_ys9 - C_KNEE;
            s_ys10 <= s_ys9;

            -- stage 11: highlight knee + cap, chroma clamp; neutral outside the
            -- avid (so the smoothing's edges are clean)
            if s_avid_sr(9) = '0' then
                s_y11 <= to_unsigned(64, 10);
                s_u11 <= to_unsigned(512, 10);
                s_v11 <= to_unsigned(512, 10);
            else
                if s_yk10 <= 0 then
                    if s_ys10 < 0 then
                        s_y11 <= (others => '0');
                    else
                        s_y11 <= unsigned(s_ys10(9 downto 0));
                    end if;
                elsif s_yk10 >= 2 * (C_YMAX - C_KNEE) then
                    s_y11 <= to_unsigned(C_YMAX, 10);
                else
                    s_y11 <= to_unsigned(C_KNEE, 10) + unsigned(s_yk10(9 downto 1));
                end if;
                s_u11 <= f_clamp10(s_us10);
                s_v11 <= f_clamp10(s_vs10);
            end if;

            -- stages 12..13: horizontal [1 2 1]/4 on the pixel one back (g1 the
            -- centre, g2 and stage 11 its neighbours)
            s_g1y <= s_y11;  s_g2y <= s_g1y;
            s_g1u <= s_u11;  s_g2u <= s_g1u;
            s_g1v <= s_v11;  s_g2v <= s_g1v;
            v_s := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y11, 12);
            s_y13 <= v_s(11 downto 2);
            v_s := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u11, 12);
            s_u13 <= v_s(11 downto 2);
            v_s := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v11, 12);
            s_v13 <= v_s(11 downto 2);

            -- stage 14: blanking gate
            if s_avid_sr(12) = '1' then
                s_y14 <= s_y13;
                s_u14 <= s_u13;
                s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10);
                s_u14 <= to_unsigned(512, 10);
                s_v14 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y14);
    s_io_align_in.u       <= std_logic_vector(s_u14);
    s_io_align_in.v       <= std_logic_vector(s_v14);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (14 -> 16).
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

end architecture reflector;
