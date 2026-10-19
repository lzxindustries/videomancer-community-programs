-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Halation: catalogue combo #5 (seed 20260912) -- bloom / glow / soft focus
-- + magnetic warp (wobble).  Three knobs each; the switches and the slider
-- decide how they combine.
--
--   K1 Glow      bloom gain
--   K2 Threshold luma above which the blur blooms (black .. ~white)
--   K3 Radius    blur width 7..255 px (two cascaded boxes, exponential knob;
--                1080-line px, scaled per mode)
--   K4 Wobble    per-line sine displacement amplitude, 0..+-255 px (1080-
--                line px, scaled per mode)
--   K5 Rate      wobble roll speed, bipolar, centre = frozen
--   K6 Period    vertical wavelength of the wobble
--
--   S7  Warp     wobble moves both layers / the glow layer only
--   S8  Tint     the glow carries a warm chroma cast
--   S9  Energy   wobble amplitude scaled by each row's mean luma
--   S10 Blend    glow added / screened
--   S11 Soft     the base picture is replaced by its blur (soft focus)
--   P12 Halo     red halation: the un-warped glow read at +-d and added on
--                top -- a wider glow that splits into twin ghosts as d grows
--                (d to 1/8 of the 1080p width, scaled with the height)
--
-- Blur: 8-bit luma through two cascaded running boxes of width n (4..128,
-- 64 max in SD) that share one 256x16 EBR ring ({luma, box-1 mean} written
-- together, read n back); 1/n is an 8-bit mantissa + shift from a per-frame
-- table.  The running sums are re-zeroed in vblank (avid absent 512+ clocks),
-- where n is also latched, so they never drift.  The blur input is padded
-- with the line's own inset edge pixels, so the glow reaches the borders.
-- The glow goes to its own line buffer at its centre column (newest sample
-- - n - 7), half rate, plus a quarter-rate copy read by the halo; base Y/U/V
-- are written one clock after the input.  All reads are of the previous line.
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Latency 14 clocks, all modes, blanking-gated.
--
-- v1.0.2 (2026-10-17): resolution scaling.  Sizes are 1080-line pixels,
-- scaled per mode from the video timing ID (registers_in(8)): the Radius
-- table index drops 15 / 26 steps at 720p / SD (the table runs 26 steps per
-- octave, so the box width is x 2/3 / x 1/2); the Wobble amplitude is x 2/3
-- at 720p and x 1/2 at SD (Energy: the row mean is normalised per mode, so
-- the same factors hold); the Period phase step per field line is scaled by
-- the virtual line step (1.5 at 720p, 1080/486 x 2 per 480i field line; a
-- serial multiplier); the halo spread uses a per-mode 1080-line width
-- (1920 / 1280 / 960 px) instead of the measured width, so the ghosts sit
-- the same distance from a highlight relative to the screen height.  1080p
-- is unchanged (every step is exactly 1.0).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture halation of program_top is
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

    -- Resolution scaling: virtual 1080-line steps from the timing ID.  x step
    -- in half units per pixel (1080: 2, 720p: 3, SD: 4), y step Q4.8 per
    -- FIELD line (1080 / active lines, doubled for interlace).
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

    -- Radius (K3 top 7 bits) -> box width n (exponential 4..128) and 1/n ~ rm / 2^(9 + sh)
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

    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset.
    function f_edge_addr(v : signed; we : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > signed(resize(we, v'length)) then
            return we;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Line buffers: Y, U, V (base), G (glow, half rate), H (glow, quarter
    -- rate, the halo's read port); blur ring
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_lb_h is array (0 to 511) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal lbG0, lbG1 : t_lb_c := (others => x"10");
    signal lbH0, lbH1 : t_lb_h := (others => x"10");
    signal r_ring     : t_ring := (others => (others => '0'));
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdG0, s_rdG1 : std_logic_vector(7 downto 0) := x"10";
    signal s_rdH0, s_rdH1 : std_logic_vector(7 downto 0) := x"10";
    signal s_rd           : std_logic_vector(15 downto 0) := (others => '0');
    -- No read ever needs the address being written: the line buffers are read
    -- only from the bank not being written while the output is active (the
    -- glow's hblank tail lands on a bank whose reads are gated), and the ring
    -- reads n >= 4 behind its write pointer.
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;
    attribute no_rw_check of lbG0 : signal is true;
    attribute no_rw_check of lbG1 : signal is true;
    attribute no_rw_check of lbH0 : signal is true;
    attribute no_rw_check of lbH1 : signal is true;
    attribute no_rw_check of r_ring : signal is true;

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_glow, s_k_thr, s_k_radius, s_k_wob, s_k_rate, s_k_period : unsigned(9 downto 0);
    signal s_sw_glowonly, s_sw_tint, s_sw_energy, s_sw_screen, s_sw_soft : std_logic;
    signal s_p_halo : unsigned(9 downto 0);
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_thr_a, s_thr_b : unsigned(9 downto 0) := (others => '0');
    signal s_thr    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rtab   : t_rt := (others => '0');
    signal s_sd     : std_logic := '0';
    signal s_n      : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_rm     : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_sh     : unsigned(2 downto 0) := "010";
    signal s_rofs   : unsigned(7 downto 0) := to_unsigned(243, 8);
    signal s_gofs   : unsigned(11 downto 0) := to_unsigned(23, 12);
    signal s_ghi    : unsigned(11 downto 0) := to_unsigned(503, 12);
    signal s_freq   : unsigned(11 downto 0) := (others => '0');
    signal s_rate   : signed(13 downto 0) := (others => '0');
    signal s_frame_phase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(3 downto 0) := "1111";
    signal s_d      : signed(10 downto 0) := (others => '0');
    signal s_absd   : unsigned(9 downto 0) := (others => '0');
    signal s_kper   : unsigned(9 downto 0) := (others => '0');
    signal s_dsq_h, s_dsq_l, s_k6_h, s_k6_l : unsigned(14 downto 0) := (others => '0');
    signal s_dsq    : unsigned(19 downto 0) := (others => '0');
    signal s_k6sq   : unsigned(19 downto 0) := (others => '0');
    signal s_pp     : unsigned(15 downto 0) := (others => '0');
    signal s_dp     : unsigned(18 downto 0) := (others => '0');
    signal s_hd     : unsigned(7 downto 0) := (others => '0');
    signal s_hg     : unsigned(7 downto 0) := (others => '0');
    signal s_hoff_l, s_hoff_r : signed(13 downto 0) := (others => '0');
    -- resolution scaling (static per mode, decoded every clock)
    signal s_vxs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_vys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ridx   : unsigned(6 downto 0)  := (others => '0');   -- Radius table index
    signal s_eoff   : unsigned(9 downto 0)  := to_unsigned(683, 10); -- Energy-off amplitude factor (720p / SD)
    signal s_esel   : unsigned(9 downto 0)  := (others => '0');   -- amplitude factor fed to the line multiply
    signal s_hw     : unsigned(10 downto 0) := to_unsigned(1920, 11); -- 1080-line width in real px (halo)
    signal s_esd    : unsigned(9 downto 0)  := (others => '0');   -- SD row sum / 256
    signal s_esd_t  : unsigned(9 downto 0)  := (others => '0');
    -- Period line step = (s_freq * y step) >> 8 on a free-running 16-clock
    -- serial multiplier (both operands change only at vsync)
    signal s_sm_cnt : unsigned(3 downto 0)  := (others => '0');
    signal s_sm_a   : unsigned(13 downto 0) := (others => '0');
    signal s_sm_b   : unsigned(11 downto 0) := (others => '0');
    signal s_sm_p   : unsigned(25 downto 0) := (others => '0');
    signal s_fstep  : unsigned(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Input / tracking
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    signal s_wpar, s_rbank, s_gpar : std_logic := '0';
    signal s_bcnt    : unsigned(9 downto 0) := (others => '1');        -- blank clocks, saturating
    signal s_esum    : unsigned(18 downto 0) := (others => '0');
    signal s_erow    : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- base write port, one clock behind the input
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
    -- Per-line sequencer
    --------------------------------------------------------------------------
    signal s_lstep    : unsigned(3 downto 0) := "1111";
    signal s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q    : signed(9 downto 0) := (others => '0');
    signal s_sin      : signed(9 downto 0) := (others => '0');
    signal s_amp      : unsigned(9 downto 0) := (others => '0');
    signal s_kwob     : unsigned(9 downto 0) := (others => '0');
    signal s_ampe_h, s_ampe_l : unsigned(14 downto 0) := (others => '0');
    signal s_ampe     : unsigned(19 downto 0) := (others => '0');
    signal s_prod_h, s_prod_l : signed(15 downto 0) := (others => '0');
    signal s_prod     : signed(20 downto 0) := (others => '0');
    signal s_off_row  : signed(9 downto 0) := (others => '0');
    signal s_offb, s_offg : signed(13 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Address / pixel chain (stage n = n clocks after the input sample)
    --------------------------------------------------------------------------
    signal s_hsel, s_hpar0, s_hpar1, s_hpar2, s_hpar3 : std_logic := '0';
    signal s_hoff    : signed(13 downto 0) := (others => '0');
    signal s_ug1, s_uh1, s_ub5 : signed(13 downto 0) := (others => '0');
    type t_rxd is array (1 to 4) of unsigned(10 downto 0);
    signal s_rxd     : t_rxd := (others => (others => '0'));
    signal s_ag2, s_ah2, s_ab6 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_g4, s_hl4, s_hr4 : unsigned(7 downto 0) := (others => '0');
    signal s_gx5, s_hlx5, s_hrx5 : unsigned(9 downto 0) := (others => '0');
    signal s_g5, s_g6, s_gq7 : unsigned(7 downto 0) := (others => '0');
    signal s_gp6     : unsigned(17 downto 0) := (others => '0');
    signal s_hs6     : unsigned(10 downto 0) := (others => '0');
    signal s_hp7     : unsigned(18 downto 0) := (others => '0');
    signal s_g7      : unsigned(11 downto 0) := (others => '0');
    signal s_a8      : unsigned(12 downto 0) := (others => '0');
    signal s_tu8     : unsigned(10 downto 0) := (others => '0');
    signal s_tv8     : unsigned(9 downto 0) := (others => '0');
    signal s_by8, s_bu8, s_bv8 : unsigned(9 downto 0) := (others => '0');
    signal s_a9, s_a10, s_a11, s_a12 : unsigned(9 downto 0) := (others => '0');
    signal s_as10    : unsigned(10 downto 0) := (others => '0');
    signal s_md9     : signed(10 downto 0) := (others => '0');
    signal s_m10     : unsigned(9 downto 0) := (others => '0');
    signal s_sph11, s_spl11 : unsigned(15 downto 0) := (others => '0');
    signal s_sp12    : unsigned(10 downto 0) := (others => '0');
    type t_c10 is array (9 to 12) of unsigned(9 downto 0);
    type t_c9  is array (9 to 12) of unsigned(8 downto 0);
    signal s_byc, s_buc, s_bvc : t_c10 := (others => (others => '0'));
    signal s_tuc, s_tvc : t_c9 := (others => (others => '0'));
    signal s_ypre13, s_upre13 : unsigned(11 downto 0) := (others => '0');
    signal s_vpre13  : signed(11 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_glow    <= unsigned(registers_in(0));
    s_k_thr     <= unsigned(registers_in(1));
    s_k_radius  <= unsigned(registers_in(2));
    s_k_wob     <= unsigned(registers_in(3));
    s_k_rate    <= unsigned(registers_in(4));
    s_k_period  <= unsigned(registers_in(5));
    s_sw_glowonly <= registers_in(6)(0);
    s_sw_tint     <= registers_in(6)(1);
    s_sw_energy   <= registers_in(6)(2);
    s_sw_screen   <= registers_in(6)(3);
    s_sw_soft     <= registers_in(6)(4);
    s_p_halo    <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Tracking, controls, base write port, sequencers, sync delay
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_fp  : unsigned(15 downto 0);
        variable v_abs : signed(10 downto 0);
        variable v_mag : unsigned(13 downto 0);
        variable v_ri  : unsigned(6 downto 0);
        variable v_sm  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
                s_bcnt <= (others => '0');
                s_esum <= s_esum + resize(unsigned(data_in.y(9 downto 2)), 19);
            else
                s_rx <= (others => '0');
                if s_bcnt /= 1023 then
                    s_bcnt <= s_bcnt + 1;
                end if;
                s_esum <= (others => '0');
            end if;

            -- base write port: input one clock deep
            s_wy1 <= unsigned(data_in.y);
            s_wu1 <= unsigned(data_in.u(9 downto 2));
            s_wv1 <= unsigned(data_in.v(9 downto 2));
            s_wx1 <= s_rx;
            s_we1 <= data_in.avid;
            s_wp1 <= s_wpar;

            -- controls: threshold spans black .. ~white (64 + 0.8125 * K2)
            s_gain  <= s_k_glow(9 downto 2);
            s_thr_a <= s_k_thr - shift_right(s_k_thr, 2);
            s_thr_b <= shift_right(s_k_thr, 4) + 64;
            s_thr   <= s_thr_a + s_thr_b;
            -- blur width: latched in vblank just ahead of the running-sum re-zero
            -- resolution scaling: mode steps (static per mode)
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_vys <= f_vstep_y(registers_in(8)(3 downto 0));
            -- Radius index: -15 at 720p (x 2/3), -26 at SD (x 1/2), floored at 0
            v_ri := s_k_radius(9 downto 3);
            case s_vxs is
                when "011" =>
                    if v_ri < 15 then s_ridx <= (others => '0'); else s_ridx <= v_ri - 15; end if;
                    s_eoff <= to_unsigned(683, 10);
                    s_hw   <= to_unsigned(1280, 11);
                when "100" =>
                    if v_ri < 26 then s_ridx <= (others => '0'); else s_ridx <= v_ri - 26; end if;
                    s_eoff <= to_unsigned(512, 10);
                    s_hw   <= to_unsigned(960, 11);
                when others =>
                    s_ridx <= v_ri;
                    s_eoff <= to_unsigned(683, 10);
                    s_hw   <= to_unsigned(1920, 11);
            end case;
            s_rtab <= f_rtab(s_ridx);
            -- Wobble amplitude factor: the row energy, or (Energy off) 1024 at
            -- 1080 (state 1's plain shift), 683 at 720p, 512 at SD
            if s_sw_energy = '1' then
                s_esel <= s_erow;
            else
                s_esel <= s_eoff;
            end if;
            -- SD row energy: (sum / 256) x 0.664 = sum / 385, half the 1080 mean
            -- scale (720p's sum / 512 over 1280 px is already 2/3 of it)
            s_esd_t <= ('0' & s_esd(9 downto 1)) + ("000" & s_esd(9 downto 3));
            if s_vxs = "100" then
                s_erow <= s_esd_t + resize(s_esd_t(9 downto 4), 10);
            end if;
            -- Period line step = (s_freq * y step) >> 8 (1080p: s_freq): load at
            -- 0, 12 shift-add steps, latch at 13
            s_sm_cnt <= s_sm_cnt + 1;
            if s_sm_cnt = 0 then
                s_sm_a <= resize(s_freq, 14); s_sm_b <= s_vys; s_sm_p <= (others => '0');
            elsif s_sm_cnt <= 12 then
                if s_sm_b(0) = '1' then v_sm := ('0' & s_sm_p(25 downto 12)) + ('0' & s_sm_a);
                else v_sm := '0' & s_sm_p(25 downto 12); end if;
                s_sm_p <= v_sm & s_sm_p(11 downto 1);
                s_sm_b <= '0' & s_sm_b(11 downto 1);
            elsif s_sm_cnt = 13 then
                s_fstep <= s_sm_p(23 downto 8);
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
            s_hoff_r <= signed(resize(s_hd, 14));
            s_hoff_l <= -signed(resize(s_hd, 14));

            ------------------------------------------------------------------
            -- Per-line sequencer: wobble offset for the coming line (both
            -- products split hi/lo, the knob re-registered off the SPI bus)
            ------------------------------------------------------------------
            s_kwob <= s_k_wob;
            case to_integer(s_lstep) is
                when 0 =>
                    s_sin_addr <= s_line_phase(15 downto 8);
                    s_ampe_h <= s_kwob * s_esel(9 downto 5);
                    s_ampe_l <= s_kwob * s_esel(4 downto 0);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    if s_sw_energy = '1' or s_vxs /= "010" then
                        s_ampe <= shift_left(resize(s_ampe_h, 20), 5) + resize(s_ampe_l, 20);
                    else
                        s_ampe <= s_kwob & "0000000000";
                    end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_amp <= s_ampe(19 downto 10);
                    s_sin <= s_sin_q;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_prod_h <= s_sin * signed('0' & s_amp(9 downto 5));
                    s_prod_l <= s_sin * signed('0' & s_amp(4 downto 0));
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_prod <= shift_left(resize(s_prod_h, 21), 5) + resize(s_prod_l, 21);
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_off_row <= resize(shift_right(s_prod, 10), 10);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_offg <= resize(s_off_row, 14);
                    if s_sw_glowonly = '1' then
                        s_offb <= (others => '0');
                    else
                        s_offb <= resize(s_off_row, 14);
                    end if;
                    s_lstep <= s_lstep + 1;
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Per-frame sequencer: rate curve (bipolar, centre detent), period,
            -- halo spread d = (P12/4)^2 * width / 2^19 and halo gain.  The
            -- squares are split hi/lo (states 2, 8).
            ------------------------------------------------------------------
            s_kper <= s_k_period;
            case to_integer(s_vstep) is
                when 0 =>
                    s_d <= signed('0' & s_k_rate) - to_signed(512, 11);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    v_abs  := abs(s_d);
                    s_absd <= unsigned(v_abs(9 downto 0));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_dsq_h <= s_absd * s_absd(9 downto 5);
                    s_dsq_l <= s_absd * s_absd(4 downto 0);
                    s_k6_h  <= s_kper * s_kper(9 downto 5);
                    s_k6_l  <= s_kper * s_kper(4 downto 0);
                    s_vstep <= to_unsigned(8, 4);
                when 8 =>
                    s_dsq  <= shift_left(resize(s_dsq_h, 20), 5) + resize(s_dsq_l, 20);
                    s_k6sq <= shift_left(resize(s_k6_h, 20), 5) + resize(s_k6_l, 20);
                    s_vstep <= to_unsigned(3, 4);
                when 3 =>
                    v_mag := s_dsq(19 downto 6);
                    if s_absd < 8 then
                        s_rate <= (others => '0');
                    elsif s_d(10) = '1' then
                        s_rate <= -signed(v_mag);
                    else
                        s_rate <= signed(v_mag);
                    end if;
                    s_freq <= s_k6sq(19 downto 8);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    v_fp := unsigned(signed(s_frame_phase) + resize(s_rate, 16));
                    s_frame_phase <= v_fp;
                    s_line_phase  <= v_fp;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_pp <= s_p_halo(9 downto 2) * s_p_halo(9 downto 2);
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_dp <= s_pp(15 downto 8) * s_hw;
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_hd <= s_dp(18 downto 11);
                    if s_p_halo(9 downto 8) /= "00" then
                        s_hg <= (others => '1');                  -- full gain from 25% up
                    else
                        s_hg <= s_p_halo(7 downto 2) & "00";
                    end if;
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
                -- row energy = mean luma, scaled to ~10 bits (sum / 512 HD and
                -- 720p; SD: sum / 256, then x 0.664 above)
                if s_vxs = "100" then
                    s_esd <= s_esum(17 downto 8);
                else
                    s_erow <= s_esum(18 downto 9);
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
                s_line_phase <= s_line_phase + s_fstep;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
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
    -- Read addresses.  Glow / halo from stage 1, base 4 clocks later (its
    -- arithmetic is shorter), so both meet at stage 8.
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            -- the halo port alternates between its two ghost reads every clock
            s_hsel <= not s_hsel;
            if s_hsel = '1' then
                s_hoff <= s_hoff_r;
            else
                s_hoff <= s_hoff_l;
            end if;
            s_hpar0 <= s_hsel;
            -- stage 1: glow (wobbled) and halo (un-warped, +-d) addresses
            s_ug1   <= signed(resize(s_rx, 14)) + s_offg;
            s_uh1   <= signed(resize(s_rx, 14)) + s_hoff;
            s_hpar1 <= s_hpar0;
            -- base column
            s_rxd(1) <= s_rx;
            for i in 2 to 4 loop
                s_rxd(i) <= s_rxd(i - 1);
            end loop;
            s_ub5 <= signed(resize(s_rxd(4), 14)) + s_offb;
            -- stage 2 (base: 6): edge clamp
            s_ag2   <= f_edge_addr(s_ug1, s_w_edge);
            s_ah2   <= f_edge_addr(s_uh1, s_w_edge);
            s_hpar2 <= s_hpar1;
            s_ab6   <= f_edge_addr(s_ub5, s_w_edge);
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers.  Base written one clock after the input; glow written at
    -- its centre column through the line and its hblank tail.
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ab6));
            if s_we1 = '1' and s_wp1 = '0' then
                lbY0(to_integer(s_wx1)) <= std_logic_vector(s_wy1);
            end if;
        end if;
    end process;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ab6));
            if s_we1 = '1' and s_wp1 = '1' then
                lbY1(to_integer(s_wx1)) <= std_logic_vector(s_wy1);
            end if;
        end if;
    end process;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_ab6(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '0' then
                lbU0(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wu1);
            end if;
        end if;
    end process;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_ab6(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '1' then
                lbU1(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wu1);
            end if;
        end if;
    end process;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_ab6(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '0' then
                lbV0(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wv1);
            end if;
        end if;
    end process;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_ab6(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '1' then
                lbV1(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wv1);
            end if;
        end if;
    end process;

    p_lbG0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG0 <= lbG0(to_integer(s_ag2(10 downto 1)));
            if s_gwe = '1' and s_gpar = '0' then
                lbG0(to_integer(s_gaddr(10 downto 1))) <= std_logic_vector(s_gwd);
            end if;
        end if;
    end process;

    p_lbG1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG1 <= lbG1(to_integer(s_ag2(10 downto 1)));
            if s_gwe = '1' and s_gpar = '1' then
                lbG1(to_integer(s_gaddr(10 downto 1))) <= std_logic_vector(s_gwd);
            end if;
        end if;
    end process;

    p_lbH0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdH0 <= lbH0(to_integer(s_ah2(10 downto 2)));
            if s_gwe = '1' and s_gpar = '0' then
                lbH0(to_integer(s_gaddr(10 downto 2))) <= std_logic_vector(s_gwd);
            end if;
        end if;
    end process;

    p_lbH1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdH1 <= lbH1(to_integer(s_ah2(10 downto 2)));
            if s_gwe = '1' and s_gpar = '1' then
                lbH1(to_integer(s_gaddr(10 downto 2))) <= std_logic_vector(s_gwd);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 3..14: fetch, glow / halo excess, gains, screen, tint, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_h   : unsigned(7 downto 0);
        variable v_g10, v_hl10, v_hr10 : unsigned(9 downto 0);
        variable v_y   : unsigned(9 downto 0);
        variable v_u, v_v : std_logic_vector(7 downto 0);
        variable v_sp  : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 3 context
            s_a_rbank <= s_rbank;
            s_hpar3   <= s_hpar2;
            -- stage 4: bank select; the halo port's reads alternate left / right ghost
            if s_a_rbank = '1' then
                s_g4 <= unsigned(s_rdG1);
                v_h  := unsigned(s_rdH1);
            else
                s_g4 <= unsigned(s_rdG0);
                v_h  := unsigned(s_rdH0);
            end if;
            if s_hpar3 = '1' then
                s_hr4 <= v_h;
            else
                s_hl4 <= v_h;
            end if;
            -- stage 5: excess above threshold
            v_g10  := s_g4 & "00";
            v_hl10 := s_hl4 & "00";
            v_hr10 := s_hr4 & "00";
            if v_g10 > s_thr then
                s_gx5 <= v_g10 - s_thr;
            else
                s_gx5 <= (others => '0');
            end if;
            if v_hl10 > s_thr then
                s_hlx5 <= v_hl10 - s_thr;
            else
                s_hlx5 <= (others => '0');
            end if;
            if v_hr10 > s_thr then
                s_hrx5 <= v_hr10 - s_thr;
            else
                s_hrx5 <= (others => '0');
            end if;
            s_g5 <= s_g4;
            -- stage 6: glow gain (Q6); ghost pair
            s_gp6 <= s_gx5 * s_gain;
            s_hs6 <= resize(s_hlx5, 11) + resize(s_hrx5, 11);
            s_g6  <= s_g5;
            -- stage 7: halo gain (Q7 per ghost)
            s_hp7 <= s_hs6 * s_hg;
            s_g7  <= s_gp6(17 downto 6);
            s_gq7 <= s_g6;
            -- stage 8: total light added; chroma = warm glow (S8) + red halo,
            -- u-heavy (the hardware carries Cr on u)
            s_a8 <= resize(s_g7, 13) + resize(s_hp7(18 downto 7), 13);
            if s_sw_tint = '1' then
                s_tu8 <= resize(s_g7(11 downto 4), 11) + resize(s_hp7(18 downto 9), 11);
                s_tv8 <= resize(s_g7(11 downto 3), 10) + resize(s_hp7(18 downto 11), 10);
            else
                s_tu8 <= resize(s_hp7(18 downto 9), 11);
                s_tv8 <= resize(s_hp7(18 downto 11), 10);
            end if;
            -- stage 8 (base): bank select; Soft swaps the blur in as the base luma
            if s_a_rbank = '1' then
                v_y := unsigned(s_rdY1); v_u := s_rdU1; v_v := s_rdV1;
            else
                v_y := unsigned(s_rdY0); v_u := s_rdU0; v_v := s_rdV0;
            end if;
            if s_sw_soft = '1' then
                s_by8 <= s_gq7 & "00";
            else
                s_by8 <= v_y;
            end if;
            s_bu8 <= unsigned(v_u) & "00";
            s_bv8 <= unsigned(v_v) & "00";
            -- stage 9: saturate
            if s_a8 > 1023 then
                s_a9 <= (others => '1');
            else
                s_a9 <= s_a8(9 downto 0);
            end if;
            if s_tu8 > 448 then
                s_tuc(9) <= to_unsigned(448, 9);
            else
                s_tuc(9) <= s_tu8(8 downto 0);
            end if;
            if s_tv8 > 448 then
                s_tvc(9) <= to_unsigned(448, 9);
            else
                s_tvc(9) <= s_tv8(8 downto 0);
            end if;
            s_md9 <= to_signed(940, 11) - signed(resize(s_by8, 11));   -- headroom (sign-tested next)
            s_byc(9) <= s_by8; s_buc(9) <= s_bu8; s_bvc(9) <= s_bv8;
            for i in 10 to 12 loop
                s_byc(i) <= s_byc(i - 1); s_buc(i) <= s_buc(i - 1); s_bvc(i) <= s_bvc(i - 1);
                s_tuc(i) <= s_tuc(i - 1); s_tvc(i) <= s_tvc(i - 1);
            end loop;
            -- stage 10: screen operands -- light x 1.125 (~1024/876 per code),
            -- headroom 940 - y floored at 0
            s_as10 <= resize(s_a9, 11) + resize(shift_right(s_a9, 3), 11);
            if s_md9(10) = '1' then
                s_m10 <= (others => '0');
            else
                s_m10 <= unsigned(s_md9(9 downto 0));
            end if;
            s_a10 <= s_a9;
            -- stage 11: screen product, split on the headroom
            s_sph11 <= s_as10 * s_m10(9 downto 5);
            s_spl11 <= s_as10 * s_m10(4 downto 0);
            s_a11 <= s_a10;
            -- stage 12
            v_sp := shift_left(resize(s_sph11, 21), 5) + resize(s_spl11, 21);
            s_sp12 <= v_sp(20 downto 10);
            s_a12 <= s_a11;
            -- stage 13: blend + tint
            if s_sw_screen = '1' then
                s_ypre13 <= resize(s_byc(12), 12) + resize(s_sp12, 12);
            else
                s_ypre13 <= resize(s_byc(12), 12) + resize(s_a12, 12);
            end if;
            s_upre13 <= resize(s_buc(12), 12) + resize(s_tuc(12), 12);
            s_vpre13 <= signed(resize(s_bvc(12), 12)) - signed(resize(s_tvc(12), 12));
            -- stage 14: clamp (luma to legal white) + gate
            if s_avid_sr(12) = '1' then
                if s_ypre13 > 940 then
                    s_y14 <= to_unsigned(940, 10);
                else
                    s_y14 <= s_ypre13(9 downto 0);
                end if;
                if s_upre13 > 1023 then
                    s_u14 <= (others => '1');
                else
                    s_u14 <= s_upre13(9 downto 0);
                end if;
                s_v14 <= f_clamp10(s_vpre13);
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

end architecture halation;
