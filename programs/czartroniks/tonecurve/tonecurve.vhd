-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Tonecurve v0.2.0: catalogue combo #34 (seed 20260918) -- tone curves +
-- picture-in-picture with warping + colour bleed / lag.  Two knobs each; the
-- switches and the slider decide how the three combine.
--
--   K1 Curve    a continuous sweep through 8 curves, neighbours blended:
--               gamma 0.5, 0.7, 1.5, 2.2, inverse S, soft S, hard S, crush
--   K2 Contrast linear gain about mid grey before the curve, 0.25 .. 4.2x
--   K3 Size     inset 1/2, 1/4, 1/8 (labelled)
--   K4 Warp     inset wobble (sine per line, drifting), 0 .. +-255 source px
--   K5 Bleed    chroma IIR smear, 1 .. 128 px in octave steps
--   K6 Lag      chroma behind luma, square law, 0 .. 1/8 of the width
--
--   S7  Curve In  main picture / inset
--   S8  Place     inset in the corner / centred
--   S9  Bleed In  main / inset
--   S10 Inset     picture / negative (luma and chroma, before the curve)
--   S11 Lag In    main / inset
--   P12 Amount    how hard the curve is applied: 0.25x .. 2x its deviation
--                 from the original (1x at 43%); above 1x it overdrives
--
-- One dual-bank line buffer (each line shows the line above), read at the
-- main or inset address; chroma has its own read address, so Lag is an
-- address offset (edge-filled, no ring).  Luma goes through a 512-entry
-- transfer LUT that a vblank builder fills from the curve ROM: contrast,
-- a linear interpolation between ROM samples and between neighbouring curves
-- (K1), then Amount.  One shared split multiplier.  The non-curved region
-- bypasses the LUT; the inset negative is applied before it.  Output:
-- per-row U/V parity fix, [1 2 1] on Y/U/V, luma 64 + 13/16 (y - 64) (the
-- HD HDMI bar conditioning), blanking gate.  Latency 17 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tonecurve of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (17 + 3 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 17;
    constant C_EDGE : integer := 12;   -- capture blanking columns at each line end

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

    -- 8 curves x 128 point samples at luma codes 8i (tools/gen_curves.py)
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
    signal s_k_curve, s_k_contrast, s_k_size, s_k_warp, s_k_bleed, s_k_lag : unsigned(9 downto 0);
    signal s_p_amount : unsigned(9 downto 0);
    signal s_sw_curve, s_sw_centre, s_sw_bleed, s_sw_neg, s_sw_lag : std_logic := '0';
    signal s_cpa, s_cpb : unsigned(10 downto 0) := (others => '0');
    signal s_cpos   : unsigned(10 downto 0) := to_unsigned(1280, 11);   -- Q8 0 .. 7.0
    signal s_con    : unsigned(8 downto 0) := to_unsigned(64, 9);       -- Q6 0.25 .. 4.2
    signal s_ama, s_amb : unsigned(9 downto 0) := (others => '0');
    signal s_amt    : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Q8 0.25 .. 2.0
    signal s_ssh    : unsigned(1 downto 0) := "10";
    signal s_warp   : unsigned(7 downto 0) := (others => '0');
    signal s_bn     : unsigned(2 downto 0) := (others => '0');
    signal s_lag8   : unsigned(7 downto 0) := (others => '0');
    signal s_lsq    : unsigned(15 downto 0) := (others => '0');
    signal s_lag    : unsigned(7 downto 0) := (others => '0');
    signal s_lagm, s_lagi : unsigned(7 downto 0) := (others => '0');
    signal s_hd     : std_logic := '1';
    signal s_iw, s_ih, s_px, s_py, s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_pxe, s_pye, s_px2, s_py2, s_pxe2, s_pye2 : unsigned(11 downto 0) := (others => '0');
    signal s_hi     : signed(12 downto 0) := to_signed(707, 13);        -- last real column
    signal s_wphase, s_lphase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";

    -- tracking / per-line
    signal s_rx, s_line, s_height : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_noav   : unsigned(9 downto 0) := (others => '0');          -- clocks without avid
    signal s_zone   : std_logic := '0';                                 -- vblank: LUT writes allowed
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_wp     : signed(18 downto 0) := (others => '0');
    signal s_off_warp : signed(10 downto 0) := (others => '0');
    signal s_row_in, s_row_edge : std_logic := '0';

    -- LUT builder (one entry per 27 clocks; params latched at entry 0)
    signal s_bst    : unsigned(4 downto 0) := (others => '0');
    signal s_be     : unsigned(8 downto 0) := (others => '0');
    signal s_bx     : unsigned(9 downto 0) := (others => '0');
    signal s_bcrv_a, s_bcrv_b : unsigned(2 downto 0) := (others => '0');
    signal s_bf     : unsigned(7 downto 0) := (others => '0');
    signal s_bcon   : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_bm     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_mA     : signed(11 downto 0) := (others => '0');
    signal s_mB     : unsigned(9 downto 0) := (others => '0');
    signal s_mpl, s_mph : signed(17 downto 0) := (others => '0');
    signal s_mp     : signed(22 downto 0) := (others => '0');
    signal s_btr    : signed(13 downto 0) := (others => '0');
    signal s_bt     : unsigned(9 downto 0) := (others => '0');
    signal s_bft    : unsigned(2 downto 0) := (others => '0');
    signal s_bi1    : unsigned(6 downto 0) := (others => '0');
    signal s_cv_addr : unsigned(9 downto 0) := (others => '0');
    signal s_cv_q, s_cv_q2 : unsigned(9 downto 0) := (others => '0');
    signal s_q00, s_q01, s_q10, s_q11 : unsigned(9 downto 0) := (others => '0');
    signal s_ba, s_bb, s_bc : signed(11 downto 0) := (others => '0');
    signal s_bo     : signed(13 downto 0) := (others => '0');

    -- address chain (pixel p's index is in s_x1 at stage 1)
    signal s_x1, s_x2, s_x3, s_x4 : unsigned(10 downto 0) := (others => '0');
    signal s_di2    : signed(11 downto 0) := (others => '0');
    signal s_di3    : signed(12 downto 0) := (others => '0');
    signal s_sel4, s_selc4 : signed(12 downto 0) := (others => '0');
    signal s_in2, s_in3, s_in4, s_in5 : std_logic := '0';
    signal s_edge2, s_edge3, s_edge4, s_edge5 : std_logic := '0';
    signal s_xlt2, s_xlt3, s_xlt4, s_xlt5 : std_logic := '0';          -- x <= C_EDGE
    signal s_addr5, s_addrc5 : unsigned(10 downto 0) := (others => '0');
    signal s_ind    : std_logic_vector(6 to 12) := (others => '0');
    signal s_edged  : std_logic_vector(6 to 12) := (others => '0');
    signal s_xltd   : std_logic_vector(6 to 8) := (others => '0');

    -- pixel path
    signal s_a_rbank, s_a_swap : std_logic := '0';
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_y10, s_y11, s_y12 : unsigned(9 downto 0) := (others => '0');
    signal s_qu, s_qv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_u10, s_v10, s_u11, s_v11, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: neutral outside avid, [1 2 1] window (g1 the
    -- centre), luma toward black, gate
    signal s_y13, s_g1y, s_g2y, s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13, s_g1u, s_g2u, s_g1v, s_g2v, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_curve    <= unsigned(registers_in(0));
    s_k_contrast <= unsigned(registers_in(1));
    s_k_size     <= unsigned(registers_in(2));
    s_k_warp     <= unsigned(registers_in(3));
    s_k_bleed    <= unsigned(registers_in(4));
    s_k_lag      <= unsigned(registers_in(5));
    s_p_amount   <= unsigned(registers_in(7));

    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q  <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
            s_cv_q   <= to_unsigned(C_CURVE(to_integer(s_cv_addr)), 10);
            s_cv_q2  <= s_cv_q;
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
    -- Control
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_sz : unsigned(1 downto 0);
        variable v_dw : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            -- vblank zone: no avid for 512 clocks (hblank is 138..370)
            if data_in.avid = '1' then
                s_noav <= (others => '0');
            elsif s_noav(9) = '0' then
                s_noav <= s_noav + 1;
            end if;
            s_zone <= s_noav(9);

            -- per-line: inset rows, warp
            case to_integer(s_lstep) is
                when 0 =>
                    s_sin_addr <= s_lphase(15 downto 8);
                    if s_line >= s_py and resize(s_line, 12) < s_pye then s_row_in <= '1'; else s_row_in <= '0'; end if;
                    if (s_line >= s_py and resize(s_line, 12) < s_py2) or
                       (resize(s_line, 12) >= s_pye2 and resize(s_line, 12) < s_pye) then
                        s_row_edge <= '1';
                    else
                        s_row_edge <= '0';
                    end if;
                    s_lstep <= s_lstep + 1;
                when 1 | 2 => s_lstep <= s_lstep + 1;
                when 3 =>
                    s_wp <= s_sin_q2 * signed('0' & s_warp);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_off_warp <= resize(shift_right(s_wp, 8), 11);
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    -- curve position K1 x 7/4 (+ K1/256 so 1023 lands on 7.0)
                    s_cpa <= resize(s_k_curve, 11) + resize(s_k_curve(9 downto 1), 11);
                    s_cpb <= resize(s_k_curve(9 downto 2), 11) + resize(s_k_curve(9 downto 8), 11);
                    s_con <= resize(s_k_contrast(9 downto 2), 9) + 16;
                    -- Size is labelled; the firmware loads its default as the label index
                    if s_k_size < 3 then
                        v_sz := s_k_size(1 downto 0);
                    elsif s_k_size < 342 then
                        v_sz := "00";
                    elsif s_k_size < 683 then
                        v_sz := "01";
                    else
                        v_sz := "10";
                    end if;
                    s_ssh  <= v_sz + 1;
                    s_warp <= s_k_warp(9 downto 2);
                    s_bn   <= s_k_bleed(9 downto 7);
                    s_lag8 <= s_k_lag(9 downto 2);
                    -- amount 0.25 + P12 x 7/4: 64 .. 512 (Q8)
                    s_ama <= to_unsigned(64, 10) + resize(s_p_amount(9 downto 1), 10);
                    s_amb <= resize(s_p_amount(9 downto 4), 10);
                    s_sw_curve  <= registers_in(6)(0);
                    s_sw_centre <= registers_in(6)(1);
                    s_sw_bleed  <= registers_in(6)(2);
                    s_sw_neg    <= registers_in(6)(3);
                    s_sw_lag    <= registers_in(6)(4);
                    if s_line_width > 1024 then s_hd <= '1'; else s_hd <= '0'; end if;
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_height(10 downto 1);
                    s_wphase <= s_wphase + 700;
                    s_lphase <= s_wphase;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_cpos <= s_cpa + s_cpb;
                    s_amt  <= s_ama - s_amb;
                    s_iw   <= shift_right(s_line_width, to_integer(s_ssh));
                    s_ih   <= shift_right(s_height, to_integer(s_ssh));
                    s_lsq  <= s_lag8 * s_lag8;
                    s_hi   <= signed(resize(s_line_width, 13)) - (C_EDGE + 1);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    if s_sw_centre = '1' then
                        s_px <= s_cx - ('0' & s_iw(10 downto 1));
                        s_py <= s_cy - ('0' & s_ih(10 downto 1));
                    else
                        s_px <= s_line_width - s_iw - 8;
                        s_py <= s_height - s_ih - 8;
                    end if;
                    -- lag: HD 0..254 px, SD 0..127 px (square law)
                    if s_hd = '1' then s_lag <= s_lsq(15 downto 8); else s_lag <= '0' & s_lsq(15 downto 9); end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_pxe <= resize(s_px, 12) + resize(s_iw, 12);
                    s_pye <= resize(s_py, 12) + resize(s_ih, 12);
                    s_px2 <= resize(s_px, 12) + 2;
                    s_py2 <= resize(s_py, 12) + 2;
                    if s_sw_lag = '1' then
                        s_lagi <= s_lag; s_lagm <= (others => '0');
                    else
                        s_lagm <= s_lag; s_lagi <= (others => '0');
                    end if;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_pxe2 <= s_pxe - 2;
                    s_pye2 <= s_pye - 2;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                -- +-1 hysteresis: analog line lengths wander
                v_dw := signed(resize(s_rx, 12)) - signed(resize(s_line_width, 12));
                if v_dw > 1 or v_dw < -1 then s_line_width <= s_rx; end if;
                s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_line_par <= s_hpar;
                if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_rbank = '1' then
                s_rowswap <= s_line_par xor s_cap_par1;
            else
                s_rowswap <= s_line_par xor s_cap_par0;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_lphase <= s_lphase + 160;
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
    --   t = (2e - 502) x contrast + 502              (clamped 0..1023)
    --   a = curve A at t, b = curve B at t           (ROM samples every 8 codes, linear between)
    --   c = a + (b - a) x K1 fraction                (blend of neighbouring curves)
    --   out = 2e + (c - 2e) x amount                 (clamped 64..940)
    -- Shared multiplier: operands at state k, product usable at k + 3.
    -- Computing runs any time; the write waits for the vblank zone.
    --------------------------------------------------------------------------
    p_build : process(clk)
        variable v_t : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_mpl <= s_mA * signed('0' & s_mB(4 downto 0));
            s_mph <= s_mA * signed('0' & s_mB(9 downto 5));
            s_mp  <= shift_left(resize(s_mph, 23), 5) + resize(s_mpl, 23);
            s_lut_we <= '0';
            s_bst <= s_bst + 1;
            case to_integer(s_bst) is
                when 0 =>
                    if s_be = 0 then
                        s_bcrv_a <= s_cpos(10 downto 8);
                        if s_cpos(10 downto 8) = 7 then s_bcrv_b <= "111"; else s_bcrv_b <= s_cpos(10 downto 8) + 1; end if;
                        s_bf <= s_cpos(7 downto 0);
                        s_bcon <= s_con; s_bm <= s_amt;
                    end if;
                    s_bx <= s_be & '0';
                when 1 =>
                    s_mA <= signed(resize(s_bx, 12)) - 502;
                    s_mB <= resize(s_bcon, 10);
                when 4 =>
                    s_btr <= resize(shift_right(s_mp, 6), 14) + 502;
                when 5 =>
                    if s_btr < 0 then
                        s_bt <= (others => '0');
                    elsif s_btr > 1023 then
                        s_bt <= (others => '1');
                    else
                        s_bt <= unsigned(s_btr(9 downto 0));
                    end if;
                when 6 =>
                    s_cv_addr <= s_bcrv_a & s_bt(9 downto 3);
                    s_bft <= s_bt(2 downto 0);
                    if s_bt(9 downto 3) = 127 then s_bi1 <= s_bt(9 downto 3); else s_bi1 <= s_bt(9 downto 3) + 1; end if;
                when 7 =>
                    s_cv_addr <= s_bcrv_a & s_bi1;
                when 8 =>
                    s_cv_addr <= s_bcrv_b & s_bt(9 downto 3);
                when 9 =>
                    s_cv_addr <= s_bcrv_b & s_bi1;
                    s_q00 <= s_cv_q2;
                when 10 =>
                    s_q01 <= s_cv_q2;
                when 11 =>
                    s_q10 <= s_cv_q2;
                    s_mA <= signed(resize(s_q01, 12)) - signed(resize(s_q00, 12));
                    s_mB <= resize(s_bft, 10);
                when 12 =>
                    s_q11 <= s_cv_q2;
                when 13 =>
                    s_mA <= signed(resize(s_q11, 12)) - signed(resize(s_q10, 12));
                    s_mB <= resize(s_bft, 10);
                when 14 =>
                    s_ba <= signed(resize(s_q00, 12)) + resize(shift_right(s_mp, 3), 12);
                when 16 =>
                    s_bb <= signed(resize(s_q10, 12)) + resize(shift_right(s_mp, 3), 12);
                when 17 =>
                    s_mA <= s_bb - s_ba;
                    s_mB <= resize(s_bf, 10);
                when 20 =>
                    s_bc <= s_ba + resize(shift_right(s_mp, 8), 12);
                when 21 =>
                    s_mA <= s_bc - signed(resize(s_bx, 12));
                    s_mB <= s_bm;
                when 24 =>
                    s_bo <= signed(resize(s_bx, 14)) + resize(shift_right(s_mp, 8), 14);
                when 25 =>
                    if s_bo < 64 then
                        s_lut_wd <= to_unsigned(64, 10);
                    elsif s_bo > 940 then
                        s_lut_wd <= to_unsigned(940, 10);
                    else
                        s_lut_wd <= unsigned(s_bo(9 downto 0));
                    end if;
                    s_lut_wa <= s_be;
                when 26 =>
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
    -- Address chain (stages 1..5).  Luma and chroma have separate read
    -- addresses: chroma = luma address - lag of that region.  The inset is
    -- clamped to the line's real columns (edge fill); the main picture reads
    -- its own column, and its lagged chroma fills from column C_EDGE.
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_sel : signed(12 downto 0);
        variable v_lag : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: pixel index (s_rx counts 1..W during active video)
            s_x1 <= s_rx - 1;
            -- 2: inset rectangle and its 2-px frame
            s_di2 <= signed(resize(s_x1, 12)) - signed(resize(s_px, 12));
            if s_row_in = '1' and s_x1 >= s_px and resize(s_x1, 12) < s_pxe then s_in2 <= '1'; else s_in2 <= '0'; end if;
            if s_row_edge = '1' or resize(s_x1, 12) < s_px2 or resize(s_x1, 12) >= s_pxe2 then s_edge2 <= '1'; else s_edge2 <= '0'; end if;
            if s_x1 <= C_EDGE then s_xlt2 <= '1'; else s_xlt2 <= '0'; end if;
            s_x2 <= s_x1;
            -- 3: inset scale + warp
            s_di3 <= shift_left(resize(s_di2, 13), to_integer(s_ssh)) + resize(s_off_warp, 13);
            s_x3 <= s_x2; s_in3 <= s_in2; s_edge3 <= s_edge2; s_xlt3 <= s_xlt2;
            -- 4: select; chroma address = luma address - lag
            if s_in3 = '1' then
                v_sel := s_di3; v_lag := s_lagi;
            else
                v_sel := signed(resize(s_x3, 13)); v_lag := s_lagm;
            end if;
            s_sel4  <= v_sel;
            s_selc4 <= v_sel - signed(resize(v_lag, 13));
            s_x4 <= s_x3; s_in4 <= s_in3; s_edge4 <= s_edge3; s_xlt4 <= s_xlt3;
            -- 5: clamp
            if s_in4 = '1' and s_sel4 < C_EDGE then
                s_addr5 <= to_unsigned(C_EDGE, 11);
            elsif s_in4 = '1' and s_sel4 > s_hi then
                s_addr5 <= unsigned(s_hi(10 downto 0));
            else
                s_addr5 <= unsigned(s_sel4(10 downto 0));
            end if;
            if s_selc4 < C_EDGE then
                if s_in4 = '0' and s_xlt4 = '1' then
                    s_addrc5 <= s_x4;
                else
                    s_addrc5 <= to_unsigned(C_EDGE, 11);
                end if;
            elsif s_in4 = '1' and s_selc4 > s_hi then
                s_addrc5 <= unsigned(s_hi(10 downto 0));
            else
                s_addrc5 <= unsigned(s_selc4(10 downto 0));
            end if;
            s_in5 <= s_in4; s_edge5 <= s_edge4 and s_in4; s_xlt5 <= s_xlt4;
            s_ind(6) <= s_in5; s_edged(6) <= s_edge5; s_xltd(6) <= s_xlt5;
            for i in 7 to 12 loop s_ind(i) <= s_ind(i - 1); s_edged(i) <= s_edged(i - 1); end loop;
            for i in 7 to 8 loop s_xltd(i) <= s_xltd(i - 1); end loop;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 1, read stage 5 -> data 6)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrc5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrc5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrc5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrc5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 6..16)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n  : integer range 0 to 7;
        variable v_ng : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_ys : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 6: bank / row-swap context; 7: fetch (U/V swapped on rows whose
            -- hsync->avid parity differs from the line they replay)
            s_a_rbank <= s_rbank; s_a_swap <= s_rowswap;
            if s_a_rbank = '1' then
                s_y7 <= unsigned(s_rdY1);
                if s_a_swap = '1' then
                    s_u7 <= unsigned(s_rdV1) & "00"; s_v7 <= unsigned(s_rdU1) & "00";
                else
                    s_u7 <= unsigned(s_rdU1) & "00"; s_v7 <= unsigned(s_rdV1) & "00";
                end if;
            else
                s_y7 <= unsigned(s_rdY0);
                if s_a_swap = '1' then
                    s_u7 <= unsigned(s_rdV0) & "00"; s_v7 <= unsigned(s_rdU0) & "00";
                else
                    s_u7 <= unsigned(s_rdU0) & "00"; s_v7 <= unsigned(s_rdV0) & "00";
                end if;
            end if;
            -- 8: inset negative, luma about 502 (64 <-> 940) and chroma
            if s_sw_neg = '1' and s_ind(7) = '1' then
                v_ng := to_signed(1004, 12) - signed(resize(s_y7, 12));
                if v_ng < 0 then s_y8 <= (others => '0'); else s_y8 <= unsigned(v_ng(9 downto 0)); end if;
                s_u8 <= not s_u7; s_v8 <= not s_v7;
            else
                s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            end if;
            -- 9: LUT address; bleed IIR (restarts at the first real column)
            s_lut_ra <= s_y8(9 downto 1);
            s_y9 <= s_y8;
            v_n := to_integer(s_bn);
            if s_avid_sr(8) = '0' or s_xltd(8) = '1' or s_ind(8) /= s_sw_bleed then
                s_qu <= s_u8 & "000000"; s_qv <= s_v8 & "000000";
            else
                s_qu <= s_qu - shift_right(s_qu, v_n) + shift_right(s_u8 & "000000", v_n);
                s_qv <= s_qv - shift_right(s_qv, v_n) + shift_right(s_v8 & "000000", v_n);
            end if;
            -- 10..11: LUT read (q at 10, fabric register at 11)
            s_lut_q2 <= unsigned(s_lut_q);
            s_y10 <= s_y9; s_y11 <= s_y10;
            s_u10 <= s_qu(15 downto 6); s_v10 <= s_qv(15 downto 6);
            s_u11 <= s_u10; s_v11 <= s_v10;
            -- 12: curved region takes the LUT
            if s_ind(11) = s_sw_curve then s_y12 <= s_lut_q2; else s_y12 <= s_y11; end if;
            s_u12 <= s_u11; s_v12 <= s_v11;
            -- 13: neutral outside avid (the smoothing's edge taps read
            -- black), inset frame
            if s_avid_sr(12) = '0' then
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            elsif s_edged(12) = '1' then
                s_y13 <= to_unsigned(940, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            else
                s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            end if;
            -- 14..15: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y13; s_g2y <= s_g1y;
            s_g1u <= s_u13; s_g2u <= s_g1u;
            s_g1v <= s_v13; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y13, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u13, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v13, 12);
            s_y15 <= v_sy(11 downto 2); s_u15 <= v_su(11 downto 2); s_v15 <= v_sv(11 downto 2);
            -- 16: luma toward black, (13 y + 192) / 16 (64 stays, 940 -> 775); gate
            v_ys := (shift_left(resize(s_y15, 14), 3) + shift_left(resize(s_y15, 14), 2)) + (resize(s_y15, 14) + 192);
            if s_avid_sr(15) = '1' then
                s_y16 <= v_ys(13 downto 4); s_u16 <= s_u15; s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y16);
    s_io_align_in.u       <= std_logic_vector(s_u16);
    s_io_align_in.v       <= std_logic_vector(s_v16);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (17 -> 20).
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

end architecture tonecurve;
