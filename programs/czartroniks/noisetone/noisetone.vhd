-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Noisetone v0.2.0: catalogue combo #137 (seed 20260927) -- Perlin /
-- value-noise displacement + gamma curves / S-curve contrast.  Three knobs
-- each; the switches and the slider decide how they combine.
--
--   K1 Scale    noise cell size 4 .. 512 px, log (64 steps an octave); the
--               field zooms about the screen centre
--   K2 Amount   displacement, square law, up to +-1/8 of the width
--   K3 Drift    the field scrolls up (right of centre) or down (left), up
--               to 16 lines a frame; centre = still
--   K4 Curve    a continuous sweep through 8 curves, neighbours blended:
--               gamma 0.5, 0.7, 1.5, 2.2, inverse S, soft S, hard S, crush
--   K5 Contrast linear gain about the pivot before the curve (25 .. 420%)
--   K6 Pivot    grey level the contrast pivots about (0 .. 100%); the
--               pivot lands on mid grey, so it also sets the exposure
--
--   S7  Patchy  one exposure everywhere / the noise field shifts the pivot (patchy exposure)
--   S8  Displace all channels / luma only (colour stays put under the boiling luma)
--   S9  Stretch noise cells tall (vertical streaks) / wide (horizontal streaks)
--   S10 Field   smooth value noise / hard cells (chunked tears)
--   S11 Chroma  saturation flat / saturation follows the contrast gain
--   P12 Storm   amplitude (+1/8 of the width) and contrast (+2x) climb together
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel.  2-D value noise: the lattice is
-- stepped by DDAs (x per pixel, y per active line) at a per-frame step from
-- a 2^(-i/64) ROM + octave shift, from starts that put the screen centre on
-- a fixed lattice point.  Lattice key = per-row hash (row index mod 1024,
-- the hash input wrapping with it) + cell column; four add/xor-shift hashes
-- per pixel, bilinear lerp with Q7 fractions, driving a horizontal read of
-- the dual-bank line buffer.  The fetched luma is contrast-stretched about a
-- (noise-shifted) pivot and mapped through a 512-entry transfer LUT that a
-- vblank builder fills from the curve ROM (linear between ROM samples and
-- between neighbouring curves).
--
-- HARDWARE CONTRACTS: each row replays the row above, so rows whose
-- hsync->avid parity differs from the replayed row's get U/V swapped back
-- (drape v1.3).  Output conditioning before the gate: [1 2 1] on Y/U/V and
-- luma 64 + 13/16 (y - 64) (the HD HDMI bar fix, damier v0.4.4; crush and
-- hard S at high contrast make full-swing edges in motion).  Latency 30
-- clocks, all modes, blanking-gated.
--
-- v0.2.2: resolution scaling.  The lattice steps are virtual (1080-line)
-- units scaled per real pixel / field line by the x / y step of the video
-- timing ID (registers_in(8)), so cells keep their size relative to the
-- screen height at SD and 720p (still centred on the screen); drift stays in
-- virtual lines; the displacement amplitude is 1/8 of the 1080p width scaled
-- by 1/x step (height-based, so SD 4:3 warps like the HD picture).  1080p is
-- unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture noisetone of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (30 + 2 = 32 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 32;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT   : integer := 30;
    constant C_SEED  : unsigned(15 downto 0) := x"5A3C";
    constant C_XBIAS : unsigned(24 downto 0) := to_unsigned(16777216, 25);   -- cell 256 at the centre
    constant C_FLOOR : integer := 24;    -- Amount floor: +-5.6 px in HD

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

    function f_clamp511(v : signed) return signed is
    begin
        if v < -511 then
            return to_signed(-511, 11);
        elsif v > 511 then
            return to_signed(511, 11);
        else
            return resize(v, 11);
        end if;
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

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;

    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;

    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

    -- lattice step per pixel (Q16 cells) for a 4 px cell * 2^(-m/64);
    -- the octave shift divides it down to 512 px
    type t_step_rom is array (0 to 63) of integer range 0 to 16384;
    constant C_STEP : t_step_rom := (
        16384, 16208, 16033, 15860, 15689, 15520, 15353, 15188,
        15024, 14862, 14702, 14544, 14387, 14232, 14079, 13927,
        13777, 13629, 13482, 13337, 13193, 13051, 12910, 12771,
        12634, 12498, 12363, 12230, 12098, 11968, 11839, 11711,
        11585, 11460, 11337, 11215, 11094, 10975, 10856, 10739,
        10624, 10509, 10396, 10284, 10173, 10064,  9955,  9848,
         9742,  9637,  9533,  9431,  9329,  9228,  9129,  9031,
         8933,  8837,  8742,  8648,  8555,  8463,  8371,  8281
    );

    -- 8 curves x 128 point samples at luma codes 8i (generated by
    -- tonecurve/tools/gen_curves.py; the same formulas as v0.1's table,
    -- which held bin centres and lifted gamma 0.5's black to 123)
    type t_curve is array (0 to 1023) of integer range 0 to 1023;
    constant C_CURVE : t_curve := (
        -- gamma 0.5
         64,  64,  64,  64,  64,  64,  64,  64,  64, 148, 182, 209, 231, 251, 269, 285,
        301, 315, 329, 342, 354, 366, 377, 388, 399, 409, 419, 429, 438, 448, 457, 465,
        474, 483, 491, 499, 507, 515, 523, 530, 538, 545, 552, 559, 566, 573, 580, 587,
        593, 600, 607, 613, 619, 626, 632, 638, 644, 650, 656, 662, 668, 673, 679, 685,
        690, 696, 702, 707, 712, 718, 723, 728, 734, 739, 744, 749, 754, 759, 764, 769,
        774, 779, 784, 789, 794, 799, 803, 808, 813, 817, 822, 827, 831, 836, 840, 845,
        849, 854, 858, 863, 867, 871, 876, 880, 884, 888, 893, 897, 901, 905, 909, 914,
        918, 922, 926, 930, 934, 938, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        -- gamma 0.7
         64,  64,  64,  64,  64,  64,  64,  64,  64,  97, 117, 135, 150, 165, 179, 192,
        204, 216, 228, 239, 250, 261, 272, 282, 292, 302, 312, 321, 330, 340, 349, 358,
        367, 376, 384, 393, 401, 410, 418, 426, 434, 442, 450, 458, 466, 474, 482, 489,
        497, 504, 512, 519, 527, 534, 541, 549, 556, 563, 570, 577, 584, 591, 598, 605,
        612, 619, 625, 632, 639, 646, 652, 659, 666, 672, 679, 685, 692, 698, 704, 711,
        717, 724, 730, 736, 742, 749, 755, 761, 767, 773, 779, 786, 792, 798, 804, 810,
        816, 822, 828, 834, 839, 845, 851, 857, 863, 869, 875, 880, 886, 892, 898, 903,
        909, 915, 920, 926, 932, 937, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        -- gamma 1.5
         64,  64,  64,  64,  64,  64,  64,  64,  64,  65,  66,  68,  70,  73,  75,  78,
         81,  85,  88,  92,  96, 100, 104, 108, 113, 118, 122, 127, 132, 138, 143, 148,
        154, 160, 165, 171, 177, 183, 190, 196, 202, 209, 216, 222, 229, 236, 243, 250,
        257, 265, 272, 280, 287, 295, 303, 310, 318, 326, 334, 342, 351, 359, 367, 376,
        384, 393, 402, 410, 419, 428, 437, 446, 455, 465, 474, 483, 493, 502, 512, 521,
        531, 541, 551, 561, 571, 581, 591, 601, 611, 621, 632, 642, 653, 663, 674, 684,
        695, 706, 717, 728, 739, 750, 761, 772, 783, 794, 806, 817, 829, 840, 852, 863,
        875, 887, 898, 910, 922, 934, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        -- gamma 2.2
         64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  65,  65,  65,  66,
         67,  68,  69,  70,  71,  72,  73,  75,  77,  79,  80,  83,  85,  87,  90,  92,
         95,  98, 101, 104, 108, 111, 115, 119, 122, 127, 131, 135, 140, 145, 149, 154,
        160, 165, 170, 176, 182, 188, 194, 200, 207, 213, 220, 227, 234, 242, 249, 257,
        264, 272, 280, 289, 297, 306, 315, 324, 333, 342, 352, 361, 371, 381, 391, 402,
        412, 423, 434, 445, 456, 468, 479, 491, 503, 515, 528, 540, 553, 566, 579, 592,
        606, 619, 633, 647, 661, 676, 690, 705, 720, 735, 750, 766, 781, 797, 813, 830,
        846, 863, 880, 897, 914, 931, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        -- inverse S
         64,  64,  64,  64,  64,  64,  64,  64,  64, 117, 140, 157, 171, 184, 196, 207,
        217, 226, 235, 244, 252, 260, 268, 275, 283, 290, 297, 304, 310, 317, 323, 330,
        336, 342, 348, 354, 360, 365, 371, 377, 383, 388, 394, 399, 405, 410, 415, 421,
        426, 431, 436, 442, 447, 452, 457, 462, 468, 473, 478, 483, 488, 493, 498, 503,
        508, 513, 519, 524, 529, 534, 539, 544, 549, 555, 560, 565, 570, 575, 581, 586,
        591, 597, 602, 608, 613, 619, 624, 630, 636, 641, 647, 653, 659, 665, 671, 678,
        684, 690, 697, 704, 711, 718, 725, 732, 740, 748, 756, 764, 773, 783, 792, 803,
        814, 826, 840, 855, 875, 902, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        -- soft S
         64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  65,  66,  67,  68,  70,  73,
         75,  79,  82,  86,  90,  94,  99, 104, 109, 115, 121, 127, 134, 141, 148, 156,
        164, 172, 180, 189, 198, 207, 216, 226, 236, 246, 256, 267, 278, 288, 300, 311,
        322, 334, 345, 357, 369, 381, 393, 405, 418, 430, 442, 455, 467, 480, 493, 505,
        518, 530, 543, 555, 568, 580, 592, 605, 617, 629, 641, 653, 665, 676, 688, 699,
        710, 721, 732, 742, 753, 763, 773, 783, 792, 802, 811, 819, 828, 836, 844, 852,
        859, 866, 873, 880, 886, 892, 897, 903, 908, 912, 916, 920, 924, 927, 930, 932,
        935, 936, 938, 939, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        -- hard S
         70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  71,  72,  72,  73,  74,  75,
         76,  77,  78,  80,  81,  83,  85,  87,  89,  91,  94,  96,  99, 102, 106, 110,
        114, 118, 123, 128, 134, 140, 147, 154, 161, 170, 178, 188, 198, 209, 220, 232,
        245, 258, 272, 287, 303, 319, 336, 353, 371, 390, 408, 428, 447, 467, 487, 507,
        527, 547, 567, 586, 605, 624, 642, 660, 677, 693, 709, 724, 739, 753, 766, 778,
        790, 801, 811, 821, 830, 838, 846, 854, 861, 867, 873, 878, 883, 888, 892, 896,
        900, 903, 906, 909, 912, 914, 916, 918, 920, 922, 923, 925, 926, 927, 928, 929,
        930, 931, 932, 933, 933, 934, 934, 934, 934, 934, 934, 934, 934, 934, 934, 934,
        -- crush
         64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  74,  90, 106, 122, 138, 154, 170, 186, 202, 218, 234, 250,
        266, 282, 298, 314, 330, 346, 362, 378, 394, 410, 426, 442, 458, 474, 490, 506,
        522, 538, 554, 570, 586, 602, 618, 634, 650, 666, 682, 698, 714, 730, 746, 762,
        778, 794, 810, 826, 842, 858, 874, 890, 906, 922, 938, 940, 940, 940, 940, 940,
        940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940
    );

    -- line buffers: bank = line parity; a bank is read only while the other
    -- is written.  Transfer LUT: written only in vblank (reads there are
    -- gated).  Neither reads the address it writes in a visible pixel.
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_lut is array (0 to 511) of std_logic_vector(9 downto 0);
    function f_lut_init return t_lut is
        variable v : t_lut;
    begin
        for i in 0 to 511 loop
            v(i) := std_logic_vector(to_unsigned(2 * i, 10));
        end loop;
        return v;
    end function;
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal lut : t_lut := f_lut_init;
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1, lut : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_lut_q  : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_q2 : unsigned(9 downto 0) := (others => '0');
    signal s_lut_ra, s_lut_wa : unsigned(8 downto 0) := (others => '0');
    signal s_lut_wd : unsigned(9 downto 0) := (others => '0');
    signal s_lut_we : std_logic := '0';

    -- controls / per-frame
    signal s_k_scale, s_k_amount, s_k_drift, s_k_curve, s_k_con, s_k_pivot : unsigned(9 downto 0);
    signal s_sw_patchy, s_sw_lumaonly, s_sw_wide, s_sw_blocky, s_sw_satcon : std_logic;
    signal s_p_storm : unsigned(9 downto 0);
    signal s_con0, s_con : unsigned(8 downto 0) := to_unsigned(80, 9);  -- Q6 gain
    signal s_con_a, s_con_b, s_con_c : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal s_pivot  : unsigned(9 downto 0) := to_unsigned(502, 10);
    signal s_cpa, s_cpb : unsigned(10 downto 0) := (others => '0');
    signal s_cpos   : unsigned(10 downto 0) := to_unsigned(1280, 11);   -- Q8 0 .. 7.0
    signal s_sidx   : unsigned(8 downto 0) := to_unsigned(192, 9);      -- scale: octave & mantissa
    signal s_srom_addr : unsigned(5 downto 0) := (others => '0');
    signal s_srom_q, s_srom_q2 : unsigned(14 downto 0) := to_unsigned(16384, 15);
    signal s_step   : unsigned(14 downto 0) := to_unsigned(2048, 15);
    -- lattice steps: virtual (per 1080-line unit) and real (per pixel / field line)
    signal s_xstep_v, s_ystep_v : unsigned(14 downto 0) := to_unsigned(2048, 15);
    signal s_xstep, s_ystep : unsigned(16 downto 0) := to_unsigned(2048, 17);
    -- resolution scaling (free-running from the timing ID)
    signal s_xs     : unsigned(2 downto 0) := to_unsigned(2, 3);        -- x step, Q.1
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);    -- y step per field line, Q.8
    signal s_wv     : unsigned(10 downto 0) := to_unsigned(1920, 11);   -- 1080p width in real px
    signal s_dspd   : unsigned(9 downto 0) := (others => '0');
    signal s_dup    : std_logic := '0';
    signal s_dinc   : unsigned(20 downto 0) := (others => '0');
    signal s_ydrift, s_yoff : unsigned(25 downto 0) := (others => '0');
    signal s_xst    : unsigned(24 downto 0) := C_XBIAS;
    signal s_hw, s_hh : unsigned(9 downto 0) := (others => '0');
    signal s_c      : unsigned(11 downto 0) := (others => '0');
    signal s_amp    : unsigned(8 downto 0) := (others => '0');          -- px at full noise
    signal s_vstep  : unsigned(4 downto 0) := "11111";
    signal s_ma, s_macc : unsigned(25 downto 0) := (others => '0');
    signal s_mb     : unsigned(11 downto 0) := (others => '0');
    signal s_mret   : unsigned(4 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_con_a : signal is true;
    attribute keep of s_con_b : signal is true;
    attribute keep of s_con_c : signal is true;

    -- tracking / per-line
    signal s_rx, s_line, s_height : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_noav   : unsigned(9 downto 0) := (others => '0');          -- clocks without avid
    signal s_zone   : std_logic := '0';                                 -- vblank: LUT writes allowed
    signal s_wpar, s_rbank, s_rb_line, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_xacc   : unsigned(24 downto 0) := C_XBIAS;                 -- lattice x (Q16 cells)
    signal s_yacc   : unsigned(25 downto 0) := (others => '0');         -- lattice y (Q16 cells)
    signal s_fy     : unsigned(6 downto 0) := (others => '0');
    signal s_h0, s_h1 : unsigned(15 downto 0) := (others => '0');
    signal s_rk0, s_rk0p, s_rk1, s_rk1p : unsigned(15 downto 0) := (others => '0');

    -- LUT builder (one entry per 18 clocks; params latched at entry 0)
    signal s_bst    : unsigned(4 downto 0) := (others => '0');
    signal s_be     : unsigned(8 downto 0) := (others => '0');
    signal s_bt     : unsigned(9 downto 0) := (others => '0');
    signal s_bcrv_a, s_bcrv_b : unsigned(2 downto 0) := (others => '0');
    signal s_bf     : unsigned(7 downto 0) := (others => '0');
    signal s_bmA    : signed(11 downto 0) := (others => '0');
    signal s_bmB    : unsigned(9 downto 0) := (others => '0');
    signal s_bmpl, s_bmph : signed(17 downto 0) := (others => '0');
    signal s_bmp    : signed(22 downto 0) := (others => '0');
    signal s_bft    : unsigned(2 downto 0) := (others => '0');
    signal s_bi1    : unsigned(6 downto 0) := (others => '0');
    signal s_cv_addr : unsigned(9 downto 0) := (others => '0');
    signal s_cv_q, s_cv_q2 : unsigned(9 downto 0) := (others => '0');
    signal s_q00, s_q01, s_q10, s_q11 : unsigned(9 downto 0) := (others => '0');
    signal s_ba, s_bb, s_bc : signed(11 downto 0) := (others => '0');

    -- noise pipeline
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_cx1    : unsigned(8 downto 0) := (others => '0');
    signal s_fx1    : unsigned(6 downto 0) := (others => '0');
    signal s_ka, s_kb, s_kc, s_kd : unsigned(15 downto 0) := (others => '0');
    signal s_ha3, s_hb3, s_hc3, s_hd3 : unsigned(15 downto 0) := (others => '0');
    signal s_ha4, s_hb4, s_hc4, s_hd4 : unsigned(15 downto 0) := (others => '0');
    signal s_ha5, s_hb5, s_hc5, s_hd5 : unsigned(15 downto 0) := (others => '0');
    signal s_a6, s_b6, s_c6, s_d6 : unsigned(7 downto 0) := (others => '0');
    signal s_a7, s_c7 : unsigned(7 downto 0) := (others => '0');
    signal s_dab7, s_dcd7 : signed(8 downto 0) := (others => '0');
    signal s_a8, s_c8 : unsigned(7 downto 0) := (others => '0');
    signal s_pab8, s_pcd8 : signed(16 downto 0) := (others => '0');
    signal s_top9, s_bot9 : unsigned(7 downto 0) := (others => '0');
    signal s_top10  : unsigned(7 downto 0) := (others => '0');
    signal s_dtb10  : signed(8 downto 0) := (others => '0');
    signal s_top11  : unsigned(7 downto 0) := (others => '0');
    signal s_ptb11  : signed(16 downto 0) := (others => '0');
    signal s_n12    : unsigned(7 downto 0) := (others => '0');
    signal s_nc13   : signed(8 downto 0) := (others => '0');
    signal s_d14    : signed(18 downto 0) := (others => '0');
    signal s_off15  : signed(9 downto 0) := (others => '0');
    type t_fx is array (2 to 7) of unsigned(6 downto 0);
    signal s_fxd    : t_fx := (others => (others => '0'));
    type t_xs is array (2 to 16) of unsigned(10 downto 0);
    signal s_xd     : t_xs := (others => (others => '0'));
    type t_n8 is array (13 to 19) of unsigned(7 downto 0);
    signal s_nd     : t_n8 := (others => (others => '0'));

    -- address / fetch / curve
    signal s_u16    : signed(12 downto 0) := (others => '0');
    signal s_addr17, s_caddr17 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank, s_a_swap : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_piv20  : unsigned(9 downto 0) := (others => '0');
    signal s_y20    : unsigned(9 downto 0) := (others => '0');
    signal s_uc20, s_vc20 : signed(10 downto 0) := (others => '0');
    signal s_yc21   : signed(10 downto 0) := (others => '0');
    signal s_uc21, s_vc21 : signed(10 downto 0) := (others => '0');
    signal s_pu21, s_pv21 : signed(20 downto 0) := (others => '0');
    signal s_pc22   : signed(20 downto 0) := (others => '0');
    type t_c11 is array (22 to 25) of signed(10 downto 0);
    signal s_ucd, s_vcd : t_c11 := (others => (others => '0'));
    -- output conditioning: neutral outside avid, [1 2 1] window (g1 the
    -- centre), luma toward black, gate
    signal s_y26, s_y27, s_g1y, s_g2y, s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26, s_u27, s_v27, s_g1u, s_g2u, s_g1v, s_g2v, s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y30    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
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

begin

    s_k_scale  <= unsigned(registers_in(0));
    s_k_amount <= unsigned(registers_in(1));
    s_k_drift  <= unsigned(registers_in(2));
    s_k_curve  <= unsigned(registers_in(3));
    s_k_con    <= unsigned(registers_in(4));
    s_k_pivot  <= unsigned(registers_in(5));
    s_sw_patchy   <= registers_in(6)(0);
    s_sw_lumaonly <= registers_in(6)(1);
    s_sw_wide     <= registers_in(6)(2);
    s_sw_blocky   <= registers_in(6)(3);
    s_sw_satcon   <= registers_in(6)(4);
    s_p_storm  <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_cv_q    <= to_unsigned(C_CURVE(to_integer(s_cv_addr)), 10);
            s_cv_q2   <= s_cv_q;
            s_srom_q  <= to_unsigned(C_STEP(to_integer(s_srom_addr)), 15);
            s_srom_q2 <= s_srom_q;
        end if;
    end process;

    p_lut : process(clk)
    begin
        if rising_edge(clk) then
            s_lut_q <= lut(to_integer(s_lut_ra));
            if s_lut_we = '1' then lut(to_integer(s_lut_wa)) <= std_logic_vector(s_lut_wd); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control: tracking, per-line and per-frame sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_cy : unsigned(9 downto 0);
        variable v_s13 : unsigned(12 downto 0);
        variable v_dv, v_dn : signed(10 downto 0);
        variable v_da : unsigned(9 downto 0);
        variable v_dw : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            -- resolution scaling: per-mode steps; the 1920-unit 1080p width
            -- in real pixels (2 / x step), the displacement reference
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(s_xs) is
                when 4      => s_wv <= to_unsigned(960, 11);
                when 3      => s_wv <= to_unsigned(1280, 11);
                when others => s_wv <= to_unsigned(1920, 11);
            end case;
            -- lattice x DDA: the line starts at the per-frame centred start
            if data_in.avid = '1' then
                s_xacc <= s_xacc + resize(s_xstep, 25);
            else
                s_xacc <= s_xst;
            end if;
            -- vblank zone: no avid for 512 clocks (hblank is 138..370)
            if data_in.avid = '1' then
                s_noav <= (others => '0');
            elsif s_noav(9) = '0' then
                s_noav <= s_noav + 1;
            end if;
            s_zone <= s_noav(9);

            -- per-line sequencer: row fraction and the two hashed row keys
            case to_integer(s_lstep) is
                when 0 =>
                    v_cy := s_yacc(25 downto 16);
                    s_fy <= s_yacc(15 downto 9);
                    s_h0 <= C_SEED + resize(v_cy, 16);
                    s_h1 <= C_SEED + resize(v_cy + 1, 16);   -- row index wraps at 1024 with the key
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_h0 <= f_round1(s_h0); s_h1 <= f_round1(s_h1);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_h0 <= f_round2(s_h0); s_h1 <= f_round2(s_h1);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_rk0 <= f_round3(s_h0); s_rk1 <= f_round3(s_h1);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_rk0p <= s_rk0 + 1;
                    s_rk1p <= s_rk1 + 1;
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame sequencer: decode, scale steps, drift, amount, centring
            case to_integer(s_vstep) is
                when 0 =>
                    -- scale index 0..447 = 7 octaves x 64
                    v_s13 := shift_left(resize(s_k_scale, 13), 3) - resize(s_k_scale, 13);
                    s_sidx <= v_s13(12 downto 4);
                    -- two-way drift about the knob centre (deadband +-8);
                    -- abs at full width, then slice (|-512| must stay 512)
                    v_dv := signed(resize(s_k_drift, 11)) - to_signed(512, 11);
                    if v_dv < 0 then v_dn := -v_dv; else v_dn := v_dv; end if;
                    v_da := unsigned(v_dn(9 downto 0));
                    if v_da < 8 then s_dspd <= (others => '0'); else s_dspd <= v_da; end if;
                    if v_dv > 0 then s_dup <= '1'; else s_dup <= '0'; end if;
                    s_con0 <= to_unsigned(16, 9) + resize(s_k_con(9 downto 2), 9);       -- Q6: 0.25 .. 4.2
                    s_pivot <= s_k_pivot;
                    -- curve position K4 x 7/4 (+ K4/256 so 1023 lands on 7.0)
                    s_cpa <= resize(s_k_curve, 11) + resize(s_k_curve(9 downto 1), 11);
                    s_cpb <= resize(s_k_curve(9 downto 2), 11) + resize(s_k_curve(9 downto 8), 11);
                    s_hw <= s_line_width(10 downto 1);
                    s_hh <= s_height(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_srom_addr <= s_sidx(5 downto 0);
                    s_con <= s_con0 + resize(s_p_storm(9 downto 3), 9);                  -- up to 398 (6.2x)
                    s_cpos <= s_cpa + s_cpb;
                    s_w_edge <= s_line_width - (C_EDGE + 1);
                    s_vstep <= s_vstep + 1;
                when 2 | 3 => s_vstep <= s_vstep + 1;
                when 4 =>
                    s_step <= shift_right(s_srom_q2, to_integer(s_sidx(8 downto 6)));
                    s_con_a <= s_con; s_con_b <= s_con; s_con_c <= s_con;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    -- the long axis of a cell is 4x the short one (virtual steps)
                    if s_sw_wide = '1' then
                        s_xstep_v <= shift_right(s_step, 2); s_ystep_v <= s_step;
                    else
                        s_xstep_v <= s_step; s_ystep_v <= shift_right(s_step, 2);
                    end if;
                    s_vstep <= to_unsigned(14, 5);
                when 14 =>
                    -- real steps: x per pixel = virtual x (x step / 2) ...
                    case to_integer(s_xs) is
                        when 4      => s_xstep <= shift_left(resize(s_xstep_v, 17), 1);
                        when 3      => s_xstep <= resize(s_xstep_v, 17) + resize(shift_right(s_xstep_v, 1), 17);
                        when others => s_xstep <= resize(s_xstep_v, 17);
                    end case;
                    -- ... y per field line = virtual y x (y step / 256)
                    s_ma <= resize(s_ystep_v, 26);
                    s_mb <= s_ys;
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(15, 5);
                    s_vstep <= to_unsigned(30, 5);
                when 15 =>
                    s_ystep <= s_macc(24 downto 8);
                    s_vstep <= to_unsigned(6, 5);
                when 6 =>
                    -- drift per frame = (speed / 32 virtual lines) * virtual y step
                    s_ma <= resize(s_ystep_v, 26);
                    s_mb <= resize(s_dspd, 12);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(7, 5);
                    s_vstep <= to_unsigned(30, 5);
                when 7 =>
                    s_dinc <= s_macc(25 downto 5);
                    -- Amount: square law
                    s_ma <= resize(s_k_amount, 26);
                    s_mb <= resize(s_k_amount, 12);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(8, 5);
                    s_vstep <= to_unsigned(30, 5);
                when 8 =>
                    -- c = Amount^2 / 1024 + Storm + floor (0 .. 2070)
                    s_c <= resize(s_macc(19 downto 10), 12) + resize(s_p_storm, 12) + C_FLOOR;
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    -- displacement = c * width / 8192: 1/8 of the width each from
                    -- Amount and Storm (the 1080p width in real px: height-based)
                    s_ma <= resize(s_wv, 26);
                    s_mb <= s_c;
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(10, 5);
                    s_vstep <= to_unsigned(30, 5);
                when 10 =>
                    if s_macc(25 downto 22) /= 0 then s_amp <= (others => '1'); else s_amp <= s_macc(21 downto 13); end if;
                    -- centring: x start = cell 256 - (width / 2) * x step
                    s_ma <= resize(s_xstep, 26);
                    s_mb <= resize(s_hw, 12);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(11, 5);
                    s_vstep <= to_unsigned(30, 5);
                when 11 =>
                    s_xst <= C_XBIAS - s_macc(24 downto 0);
                    s_ma <= resize(s_ystep, 26);
                    s_mb <= resize(s_hh, 12);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(12, 5);
                    s_vstep <= to_unsigned(30, 5);
                when 12 =>
                    s_yoff <= s_macc;
                    if s_dup = '1' then
                        s_ydrift <= s_ydrift + resize(s_dinc, 26);
                    else
                        s_ydrift <= s_ydrift - resize(s_dinc, 26);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    -- y start: the drift phase sits at the screen centre
                    s_yacc <= s_ydrift - s_yoff;
                    s_vstep <= (others => '1');
                when 30 =>
                    -- serial shift-add multiplier: macc = ma * mb, then mret
                    if s_mb = 0 then
                        s_vstep <= s_mret;
                    else
                        if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                        s_ma <= shift_left(s_ma, 1);
                        s_mb <= shift_right(s_mb, 1);
                    end if;
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                -- +-1 hysteresis: analog line lengths wander
                v_dw := signed(resize(s_rx, 12)) - signed(resize(s_line_width, 12));
                if v_dw > 1 or v_dw < -1 then s_line_width <= s_rx; end if;
                s_line <= s_line + 1;
                s_height <= s_line + 1;                     -- captured on active lines
                s_yacc <= s_yacc + resize(s_ystep, 26);     -- lattice y, active lines only
            end if;
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                -- this line's read bank and parity; the write bank's capture parity
                s_rb_line <= s_rbank;
                s_line_par <= s_hpar;
                if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_rb_line = '1' then
                s_rowswap <= s_line_par xor s_cap_par1;
            else
                s_rowswap <= s_line_par xor s_cap_par0;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
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
    -- Transfer LUT builder.  Entry e holds the curved luma for code 2e:
    --   a = curve A at 2e, b = curve B at 2e   (ROM samples every 8 codes, linear between)
    --   out = a + (b - a) x K4 fraction        (blend of neighbouring curves; 64..940)
    -- Shared multiplier: operands at state k, product usable at k + 3.
    -- Computing runs any time; the write waits for the vblank zone.
    --------------------------------------------------------------------------
    p_build : process(clk)
    begin
        if rising_edge(clk) then
            s_bmpl <= s_bmA * signed('0' & s_bmB(4 downto 0));
            s_bmph <= s_bmA * signed('0' & s_bmB(9 downto 5));
            s_bmp  <= shift_left(resize(s_bmph, 23), 5) + resize(s_bmpl, 23);
            s_lut_we <= '0';
            s_bst <= s_bst + 1;
            case to_integer(s_bst) is
                when 0 =>
                    if s_be = 0 then
                        s_bcrv_a <= s_cpos(10 downto 8);
                        if s_cpos(10 downto 8) = 7 then s_bcrv_b <= "111"; else s_bcrv_b <= s_cpos(10 downto 8) + 1; end if;
                        s_bf <= s_cpos(7 downto 0);
                    end if;
                    s_bt <= s_be & '0';
                when 1 =>
                    s_cv_addr <= s_bcrv_a & s_bt(9 downto 3);
                    s_bft <= s_bt(2 downto 0);
                    if s_bt(9 downto 3) = 127 then s_bi1 <= s_bt(9 downto 3); else s_bi1 <= s_bt(9 downto 3) + 1; end if;
                when 2 =>
                    s_cv_addr <= s_bcrv_a & s_bi1;
                when 3 =>
                    s_cv_addr <= s_bcrv_b & s_bt(9 downto 3);
                when 4 =>
                    s_cv_addr <= s_bcrv_b & s_bi1;
                    s_q00 <= s_cv_q2;
                when 5 =>
                    s_q01 <= s_cv_q2;
                when 6 =>
                    s_q10 <= s_cv_q2;
                    s_bmA <= signed(resize(s_q01, 12)) - signed(resize(s_q00, 12));
                    s_bmB <= resize(s_bft, 10);
                when 7 =>
                    s_q11 <= s_cv_q2;
                when 8 =>
                    s_bmA <= signed(resize(s_q11, 12)) - signed(resize(s_q10, 12));
                    s_bmB <= resize(s_bft, 10);
                when 9 =>
                    s_ba <= signed(resize(s_q00, 12)) + resize(shift_right(s_bmp, 3), 12);
                when 11 =>
                    s_bb <= signed(resize(s_q10, 12)) + resize(shift_right(s_bmp, 3), 12);
                when 12 =>
                    s_bmA <= s_bb - s_ba;
                    s_bmB <= resize(s_bf, 10);
                when 15 =>
                    s_bc <= s_ba + resize(shift_right(s_bmp, 8), 12);
                when 16 =>
                    if s_bc < 64 then
                        s_lut_wd <= to_unsigned(64, 10);
                    elsif s_bc > 940 then
                        s_lut_wd <= to_unsigned(940, 10);
                    else
                        s_lut_wd <= unsigned(s_bc(9 downto 0));
                    end if;
                    s_lut_wa <= s_be;
                when 17 =>
                    if s_zone = '1' then
                        s_lut_we <= '1';
                        s_be <= s_be + 1;
                        s_bst <= (others => '0');
                    else
                        s_bst <= s_bst;
                    end if;
                when others => null;
            end case;
        end if;
    end process p_build;

    --------------------------------------------------------------------------
    -- Noise + address pipeline (stages 1..17)
    --------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_t, v_b : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: cell index and Q7 fraction (slices of the x DDA)
            s_x1  <= s_rx;
            s_cx1 <= s_xacc(24 downto 16);
            s_fx1 <= s_xacc(15 downto 9);
            -- stage 2: lattice keys; screen x
            s_ka <= s_rk0  + resize(s_cx1, 16);
            s_kb <= s_rk0p + resize(s_cx1, 16);
            s_kc <= s_rk1  + resize(s_cx1, 16);
            s_kd <= s_rk1p + resize(s_cx1, 16);
            s_xd(2) <= s_x1;
            for i in 3 to 16 loop s_xd(i) <= s_xd(i - 1); end loop;
            if s_sw_blocky = '1' then s_fxd(2) <= (others => '0'); else s_fxd(2) <= s_fx1; end if;
            for i in 3 to 7 loop s_fxd(i) <= s_fxd(i - 1); end loop;
            -- stages 3..5: hash rounds
            s_ha3 <= f_round1(s_ka); s_hb3 <= f_round1(s_kb); s_hc3 <= f_round1(s_kc); s_hd3 <= f_round1(s_kd);
            s_ha4 <= f_round2(s_ha3); s_hb4 <= f_round2(s_hb3); s_hc4 <= f_round2(s_hc3); s_hd4 <= f_round2(s_hd3);
            s_ha5 <= f_round3(s_ha4); s_hb5 <= f_round3(s_hb4); s_hc5 <= f_round3(s_hc4); s_hd5 <= f_round3(s_hd4);
            -- stage 6: corner values
            s_a6 <= s_ha5(15 downto 8); s_b6 <= s_hb5(15 downto 8); s_c6 <= s_hc5(15 downto 8); s_d6 <= s_hd5(15 downto 8);
            -- stage 7: x differences
            s_dab7 <= signed('0' & s_b6) - signed('0' & s_a6);
            s_dcd7 <= signed('0' & s_d6) - signed('0' & s_c6);
            s_a7 <= s_a6; s_c7 <= s_c6;
            -- stage 8: x lerp products
            s_pab8 <= s_dab7 * signed('0' & s_fxd(7));
            s_pcd8 <= s_dcd7 * signed('0' & s_fxd(7));
            s_a8 <= s_a7; s_c8 <= s_c7;
            -- stage 9: top / bottom row values
            v_t := signed(resize(s_a8, 10) & '0') + resize(shift_right(s_pab8, 6), 11);
            v_b := signed(resize(s_c8, 10) & '0') + resize(shift_right(s_pcd8, 6), 11);
            s_top9 <= unsigned(v_t(8 downto 1));
            s_bot9 <= unsigned(v_b(8 downto 1));
            -- stage 10: y difference
            s_dtb10 <= signed('0' & s_bot9) - signed('0' & s_top9);
            s_top10 <= s_top9;
            -- stage 11: y lerp product
            if s_sw_blocky = '1' then
                s_ptb11 <= (others => '0');
            else
                s_ptb11 <= s_dtb10 * signed('0' & s_fy);
            end if;
            s_top11 <= s_top10;
            -- stage 12: noise value
            v_t := signed(resize(s_top11, 10) & '0') + resize(shift_right(s_ptb11, 6), 11);
            s_n12 <= unsigned(v_t(8 downto 1));
            -- stage 13: centred; noise rides along to the pivot stage
            s_nc13 <= signed('0' & s_n12) - to_signed(128, 9);
            s_nd(13) <= s_n12;
            for i in 14 to 19 loop s_nd(i) <= s_nd(i - 1); end loop;
            -- stage 14: amplitude
            s_d14 <= s_nc13 * signed('0' & s_amp);
            -- stage 15: displacement (+-511 px)
            s_off15 <= s_d14(16 downto 7);
            -- stage 16: source x
            s_u16 <= signed(resize(s_xd(15), 13)) + resize(s_off15, 13);
            -- stage 17: addresses (luma displaced; chroma displaced or not)
            s_addr17 <= f_edge_addr(s_u16, s_w_edge);
            if s_sw_lumaonly = '1' then s_caddr17 <= s_xd(16); else s_caddr17 <= f_edge_addr(s_u16, s_w_edge); end if;
        end if;
    end process p_noise;

    --------------------------------------------------------------------------
    -- Line buffers: written at stage 1 (s_in), read at stage 17 -> data 18
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr17));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr17));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch + tone curve + output conditioning (stages 18..30)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p : signed(11 downto 0);
        variable v_n : signed(7 downto 0);
        variable v_i : signed(12 downto 0);
        variable v_u, v_v : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_ys : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 18: bank / row-swap context; 19: fetch (U/V swapped on rows
            -- whose hsync->avid parity differs from the line they replay)
            s_a_rbank <= s_rb_line; s_a_swap <= s_rowswap;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                if s_a_swap = '1' then
                    s_wet_u <= unsigned(s_rdV1) & "00"; s_wet_v <= unsigned(s_rdU1) & "00";
                else
                    s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
                end if;
            else
                s_wet_y <= unsigned(s_rdY0);
                if s_a_swap = '1' then
                    s_wet_u <= unsigned(s_rdV0) & "00"; s_wet_v <= unsigned(s_rdU0) & "00";
                else
                    s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
                end if;
            end if;
            -- stage 20: pivot (noise-shifted when patchy), centred chroma
            v_n := signed(s_nd(19) xor x"80");                                  -- noise - 128
            if s_sw_patchy = '1' then
                v_p := signed(resize(s_pivot, 12)) + shift_left(resize(v_n, 12), 1);
            else
                v_p := signed(resize(s_pivot, 12));
            end if;
            s_piv20 <= f_clamp10(v_p);
            s_y20 <= s_wet_y;
            s_uc20 <= signed(resize(s_wet_u, 11)) - 512;
            s_vc20 <= signed(resize(s_wet_v, 11)) - 512;
            -- stage 21: luma about the pivot; chroma contrast products
            s_yc21 <= signed(resize(s_y20, 11)) - signed(resize(s_piv20, 11));
            s_pu21 <= s_uc20 * signed('0' & s_con_b);
            s_pv21 <= s_vc20 * signed('0' & s_con_c);
            s_uc21 <= s_uc20; s_vc21 <= s_vc20;
            -- stage 22: luma contrast product; chroma select
            s_pc22 <= s_yc21 * signed('0' & s_con_a);
            if s_sw_satcon = '1' then
                s_ucd(22) <= f_clamp511(shift_right(s_pu21, 6));
                s_vcd(22) <= f_clamp511(shift_right(s_pv21, 6));
            else
                s_ucd(22) <= s_uc21; s_vcd(22) <= s_vc21;
            end if;
            -- stage 23: LUT address = stretched luma code / 2 (the pivot lands on 502)
            v_i := resize(shift_right(s_pc22, 6), 13) + to_signed(502, 13);
            if v_i < 0 then s_lut_ra <= (others => '0');
            elsif v_i > 1023 then s_lut_ra <= (others => '1');
            else s_lut_ra <= unsigned(v_i(9 downto 1)); end if;
            -- 24..25: LUT read (q at 24, fabric register at 25)
            s_lut_q2 <= unsigned(s_lut_q);
            -- stage 26: curved luma, recentred chroma
            s_y26 <= s_lut_q2;
            v_u := resize(s_ucd(25), 12) + 512;
            v_v := resize(s_vcd(25), 12) + 512;
            s_u26 <= unsigned(v_u(9 downto 0));
            s_v26 <= unsigned(v_v(9 downto 0));
            -- stage 27: neutral outside avid (the smoothing's edge taps read black)
            if s_avid_sr(25) = '1' then
                s_y27 <= s_y26; s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
            -- 28..29: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y27; s_g2y <= s_g1y;
            s_g1u <= s_u27; s_g2u <= s_g1u;
            s_g1v <= s_v27; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y27, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u27, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v27, 12);
            s_y29 <= v_sy(11 downto 2); s_u29 <= v_su(11 downto 2); s_v29 <= v_sv(11 downto 2);
            -- stage 30: luma toward black, (13 y + 192) / 16 (64 stays, 940 -> 775); gate
            v_ys := (shift_left(resize(s_y29, 14), 3) + shift_left(resize(s_y29, 14), 2)) + (resize(s_y29, 14) + 192);
            if s_avid_sr(28) = '1' then
                s_y30 <= v_ys(13 downto 4); s_u30 <= s_u29; s_v30 <= s_v29;
            else
                s_y30 <= to_unsigned(64, 10); s_u30 <= to_unsigned(512, 10); s_v30 <= to_unsigned(512, 10);
            end if;

            for i in 23 to 25 loop s_ucd(i) <= s_ucd(i - 1); s_vcd(i) <= s_vcd(i - 1); end loop;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y30);
    s_io_align_in.u       <= std_logic_vector(s_u30);
    s_io_align_in.v       <= std_logic_vector(s_v30);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (30 -> 32).
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

end architecture noisetone;
