-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- aurora.vhd  (v3.0)
--
-- AURORA: a big, smooth, travelling horizontal rainbow wave.
--
-- ARCHITECTURE (no framebuffer; one line buffer, one column per clock)
--
--   Once per line period a scanner sweeps left to right, ONE COLUMN PER
--   CLOCK, and stamps {hue[10], alpha[6]} into a 2048 x 16 line buffer for
--   the next display row.  The wave's centre line at column x is
--
--       y(x) = C + A*sin(th1) + N*A*( sin(th2) + sin(th3) )/2
--       th1 = k*x - ph1(t)        the wave itself; Speed moves ph1
--       th2 = 1.61*k*x + ph2(t)   two incommensurate noise lobes that drift
--       th3 = 0.38*k*x + ph3(t)   on their own, so the wave never repeats
--
--   and the band is a constant VERTICAL thickness H about it (the classic
--   flag-style rainbow wave: no slope correction, so it stays smooth at any
--   steepness and the amplitude can reach half the screen).  Noise also
--   breathes the thickness, H_eff = H*(1 + noise/2), so the band bulges
--   where it bends.  All of C, A, H derive from the MEASURED raster.
--
--   Per column the pipeline is: three interpolated quarter-wave sine reads
--   (6 ROMs) -> noise x N -> sum x A -> y -> vertical distance to this row
--   (and to the row REFLECTED about the centre line) -> nearer copy wins ->
--   alpha = feather(H_eff - |d|), hue = colour phase - d x hue-scale.  A
--   mirrored copy costs only a second subtract and a compare, not a second
--   pass.  It is a TRUE reflection -- the reflected band carries the colour
--   of the row it mirrors, so its spectrum runs violet-on-top -- which makes
--   both copies identical on the centre line: a seamless kaleidoscope fold.
--   The axis is the true centre of the raster, (fh-1)/2 rows (539.5 at
--   1080), and the wave is centred on it too.
--
--   Line buffer: ONE bank, 2048 x 16 (8 EBR).  Column c of row R+1 is
--   stamped ~16 clocks AFTER the beam read column c of row R, so one buffer
--   serves both.  Post-active vblank lines restamp row 0 (idempotent).
--
--   Beam: word -> hue -> (Solid: snap to the 7 pure phosphor colours) ->
--   Y/U/V from six 256 x 10 constant ROMs -> alpha-composite over black
--   (Y 64) or the live picture.  Fill = Video replaces the band's luma with
--   the live picture's (half-mixed with the rainbow's) and keeps the rainbow
--   chroma.  The LUT is authored DIRECTLY in the verified hardware
--   convention (the u register carries BT.601 Cr), so the sim shows
--   mirrored hues; the screen shows ROYGBIV.
--
--   Geometry is computed in FRAME rows on every raster; interlaced fields
--   compare against R*2+parity (bottom field = field_n '0').  All motion
--   phases advance once per field on a saw-active-guarded vsync (analog
--   vsync serrates).  K1/K2/K4/K6/P12 are slewed (1-pole, ~250 ms).
--   Stillness is exact: Speed and Flow have true centre deadbands, and at
--   Noise 0 the drifting noise lobes contribute nothing.
--
-- CONTROLS
--   K1  Height     band thickness: hairline -> the whole screen
--   K2  Waves      wavelength: 0.5 -> 5 waves across the screen
--   K3  Speed      wave travel, bipolar; centre = still
--   K4  Noise      organic wobble of path and thickness; 0 = pure sine
--   K5  Flow       rainbow drift; centre = stopped
--   K6  Spectrum   rainbow cycles across the band (or screen), 0.5 .. 4
--   S7  Color      Gradient / Solid (7 phosphor stripes)
--   S8  Backdrop   Black / live video
--   S9  Fill       Rainbow / Video (band luma from the live picture)
--   S10 Rainbow    Across the band / Along the screen
--   S11 Mirror     Off / On: top half reflects the bottom, seamless fold
--   P12 SWELL      amplitude: gentle ripple -> crests at the screen edges
--
--   CLEAN AVID.  The encoders get no data-enable -- only hsync and the
--   4:2:2 bus -- so they pair Cb/Cr by counting from hsync, while the core's
--   4:4:4->4:2:2 packer restarts on Cb at every avid rise.  A source avid
--   that lands a clock off its usual hsync distance swaps U/V for that line:
--   thin wrong-coloured lines across the band (black is immune).  So the
--   program makes its own avid (cubist's HW-confirmed recipe): it starts 3
--   or 4 clocks after the source's first avid rise of the line, whichever
--   keeps the hsync->start PARITY at the source's usual value (saturating
--   vote), and ends after the field-latched line width.  The beam reads the
--   line buffer on that avid; the output is blanked to 64/512/512 outside
--   it; the backdrop video is gated to black where the source is blank.
--
-- Latency: 8 clocks, constant across all modes.  EBR: 20.  No DSP.
--
-- v3.0.5 (2026-10-18): resolution scaling.  The band's edge feather was a
--   fixed 31 quarter-rows (~8 rows), so it read 1.5x softer at 720p and
--   2-4x softer at SD.  It now spans the same fraction of the screen height
--   everywhere: the feather depth is multiplied by 1080/fh (a second divide
--   in the vblank sequencer; x1 at 1080 rows, so 1080p is unchanged).
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture aurora of program_top is
    -- data_in -> data_out latency is 8 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;

    constant C_LAT : integer := 8;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- 12-bit quarter-wave sine, qsin(i) = round(4095*sin((i+0.5)*pi/512)):
    -- the amplitude reaches ~2000 quarter-rows, so every LSB shows; one
    -- 256x16 EBR per read
    type t_qsin13 is array(0 to 255) of unsigned(11 downto 0);
    constant C_QSIN13 : t_qsin13 := (
        to_unsigned(  13, 12), to_unsigned(  38, 12), to_unsigned(  63, 12), to_unsigned(  88, 12),
        to_unsigned( 113, 12), to_unsigned( 138, 12), to_unsigned( 163, 12), to_unsigned( 188, 12),
        to_unsigned( 213, 12), to_unsigned( 239, 12), to_unsigned( 264, 12), to_unsigned( 289, 12),
        to_unsigned( 314, 12), to_unsigned( 339, 12), to_unsigned( 364, 12), to_unsigned( 389, 12),
        to_unsigned( 414, 12), to_unsigned( 439, 12), to_unsigned( 464, 12), to_unsigned( 489, 12),
        to_unsigned( 514, 12), to_unsigned( 539, 12), to_unsigned( 564, 12), to_unsigned( 588, 12),
        to_unsigned( 613, 12), to_unsigned( 638, 12), to_unsigned( 663, 12), to_unsigned( 688, 12),
        to_unsigned( 712, 12), to_unsigned( 737, 12), to_unsigned( 762, 12), to_unsigned( 787, 12),
        to_unsigned( 811, 12), to_unsigned( 836, 12), to_unsigned( 860, 12), to_unsigned( 885, 12),
        to_unsigned( 909, 12), to_unsigned( 934, 12), to_unsigned( 958, 12), to_unsigned( 983, 12),
        to_unsigned(1007, 12), to_unsigned(1032, 12), to_unsigned(1056, 12), to_unsigned(1080, 12),
        to_unsigned(1104, 12), to_unsigned(1128, 12), to_unsigned(1153, 12), to_unsigned(1177, 12),
        to_unsigned(1201, 12), to_unsigned(1225, 12), to_unsigned(1249, 12), to_unsigned(1273, 12),
        to_unsigned(1296, 12), to_unsigned(1320, 12), to_unsigned(1344, 12), to_unsigned(1368, 12),
        to_unsigned(1391, 12), to_unsigned(1415, 12), to_unsigned(1439, 12), to_unsigned(1462, 12),
        to_unsigned(1485, 12), to_unsigned(1509, 12), to_unsigned(1532, 12), to_unsigned(1555, 12),
        to_unsigned(1579, 12), to_unsigned(1602, 12), to_unsigned(1625, 12), to_unsigned(1648, 12),
        to_unsigned(1671, 12), to_unsigned(1694, 12), to_unsigned(1717, 12), to_unsigned(1739, 12),
        to_unsigned(1762, 12), to_unsigned(1785, 12), to_unsigned(1807, 12), to_unsigned(1830, 12),
        to_unsigned(1852, 12), to_unsigned(1875, 12), to_unsigned(1897, 12), to_unsigned(1919, 12),
        to_unsigned(1941, 12), to_unsigned(1964, 12), to_unsigned(1986, 12), to_unsigned(2007, 12),
        to_unsigned(2029, 12), to_unsigned(2051, 12), to_unsigned(2073, 12), to_unsigned(2094, 12),
        to_unsigned(2116, 12), to_unsigned(2137, 12), to_unsigned(2159, 12), to_unsigned(2180, 12),
        to_unsigned(2201, 12), to_unsigned(2223, 12), to_unsigned(2244, 12), to_unsigned(2265, 12),
        to_unsigned(2285, 12), to_unsigned(2306, 12), to_unsigned(2327, 12), to_unsigned(2348, 12),
        to_unsigned(2368, 12), to_unsigned(2389, 12), to_unsigned(2409, 12), to_unsigned(2429, 12),
        to_unsigned(2449, 12), to_unsigned(2470, 12), to_unsigned(2490, 12), to_unsigned(2509, 12),
        to_unsigned(2529, 12), to_unsigned(2549, 12), to_unsigned(2569, 12), to_unsigned(2588, 12),
        to_unsigned(2608, 12), to_unsigned(2627, 12), to_unsigned(2646, 12), to_unsigned(2665, 12),
        to_unsigned(2684, 12), to_unsigned(2703, 12), to_unsigned(2722, 12), to_unsigned(2741, 12),
        to_unsigned(2759, 12), to_unsigned(2778, 12), to_unsigned(2796, 12), to_unsigned(2815, 12),
        to_unsigned(2833, 12), to_unsigned(2851, 12), to_unsigned(2869, 12), to_unsigned(2887, 12),
        to_unsigned(2904, 12), to_unsigned(2922, 12), to_unsigned(2940, 12), to_unsigned(2957, 12),
        to_unsigned(2974, 12), to_unsigned(2992, 12), to_unsigned(3009, 12), to_unsigned(3026, 12),
        to_unsigned(3043, 12), to_unsigned(3059, 12), to_unsigned(3076, 12), to_unsigned(3093, 12),
        to_unsigned(3109, 12), to_unsigned(3125, 12), to_unsigned(3141, 12), to_unsigned(3157, 12),
        to_unsigned(3173, 12), to_unsigned(3189, 12), to_unsigned(3205, 12), to_unsigned(3221, 12),
        to_unsigned(3236, 12), to_unsigned(3251, 12), to_unsigned(3267, 12), to_unsigned(3282, 12),
        to_unsigned(3297, 12), to_unsigned(3311, 12), to_unsigned(3326, 12), to_unsigned(3341, 12),
        to_unsigned(3355, 12), to_unsigned(3370, 12), to_unsigned(3384, 12), to_unsigned(3398, 12),
        to_unsigned(3412, 12), to_unsigned(3426, 12), to_unsigned(3439, 12), to_unsigned(3453, 12),
        to_unsigned(3466, 12), to_unsigned(3480, 12), to_unsigned(3493, 12), to_unsigned(3506, 12),
        to_unsigned(3519, 12), to_unsigned(3532, 12), to_unsigned(3544, 12), to_unsigned(3557, 12),
        to_unsigned(3569, 12), to_unsigned(3581, 12), to_unsigned(3594, 12), to_unsigned(3606, 12),
        to_unsigned(3617, 12), to_unsigned(3629, 12), to_unsigned(3641, 12), to_unsigned(3652, 12),
        to_unsigned(3663, 12), to_unsigned(3675, 12), to_unsigned(3686, 12), to_unsigned(3696, 12),
        to_unsigned(3707, 12), to_unsigned(3718, 12), to_unsigned(3728, 12), to_unsigned(3739, 12),
        to_unsigned(3749, 12), to_unsigned(3759, 12), to_unsigned(3769, 12), to_unsigned(3778, 12),
        to_unsigned(3788, 12), to_unsigned(3798, 12), to_unsigned(3807, 12), to_unsigned(3816, 12),
        to_unsigned(3825, 12), to_unsigned(3834, 12), to_unsigned(3843, 12), to_unsigned(3851, 12),
        to_unsigned(3860, 12), to_unsigned(3868, 12), to_unsigned(3876, 12), to_unsigned(3884, 12),
        to_unsigned(3892, 12), to_unsigned(3900, 12), to_unsigned(3908, 12), to_unsigned(3915, 12),
        to_unsigned(3922, 12), to_unsigned(3929, 12), to_unsigned(3936, 12), to_unsigned(3943, 12),
        to_unsigned(3950, 12), to_unsigned(3957, 12), to_unsigned(3963, 12), to_unsigned(3969, 12),
        to_unsigned(3975, 12), to_unsigned(3981, 12), to_unsigned(3987, 12), to_unsigned(3993, 12),
        to_unsigned(3998, 12), to_unsigned(4004, 12), to_unsigned(4009, 12), to_unsigned(4014, 12),
        to_unsigned(4019, 12), to_unsigned(4023, 12), to_unsigned(4028, 12), to_unsigned(4033, 12),
        to_unsigned(4037, 12), to_unsigned(4041, 12), to_unsigned(4045, 12), to_unsigned(4049, 12),
        to_unsigned(4053, 12), to_unsigned(4056, 12), to_unsigned(4059, 12), to_unsigned(4063, 12),
        to_unsigned(4066, 12), to_unsigned(4069, 12), to_unsigned(4071, 12), to_unsigned(4074, 12),
        to_unsigned(4076, 12), to_unsigned(4079, 12), to_unsigned(4081, 12), to_unsigned(4083, 12),
        to_unsigned(4085, 12), to_unsigned(4087, 12), to_unsigned(4088, 12), to_unsigned(4089, 12),
        to_unsigned(4091, 12), to_unsigned(4092, 12), to_unsigned(4093, 12), to_unsigned(4093, 12),
        to_unsigned(4094, 12), to_unsigned(4095, 12), to_unsigned(4095, 12), to_unsigned(4095, 12));

    -- Ribbon colour LUT: the seven HW-verified phosphor colours interpolated
    -- to 512 steps.  The u register carries BT.601 Cr on this hardware, so
    -- these are the raw register values.  Segment starts (addr = k*73) hold
    -- the PURE colours, which is what Solid mode addresses.
    --
    -- SIX independent 256 x 10 ROMs (Y/U/V x low/high half), NOT one packed
    -- 512 x 32 array: the packed form had two constant-zero bits, yosys
    -- pruned it to 30 live bits and spread it over THREE EBR blocks -- the
    -- odd-mapping trap that reads garbage on hardware while every sim is
    -- bit-exact (this was the v1.6 "colours railed red/blue" regression and
    -- the thin coloured lines across the band).  Each ROM here is the same
    -- 256-deep, <=16-bit shape as the sine ROMs, which render correctly.
    type t_c10 is array(0 to 255) of unsigned(9 downto 0);
    constant C_CY_LO : t_c10 := (
        to_unsigned( 328, 10), to_unsigned( 331, 10), to_unsigned( 335, 10), to_unsigned( 338, 10), to_unsigned( 342, 10), to_unsigned( 345, 10), to_unsigned( 349, 10), to_unsigned( 352, 10),
        to_unsigned( 356, 10), to_unsigned( 359, 10), to_unsigned( 363, 10), to_unsigned( 366, 10), to_unsigned( 370, 10), to_unsigned( 373, 10), to_unsigned( 377, 10), to_unsigned( 380, 10),
        to_unsigned( 384, 10), to_unsigned( 387, 10), to_unsigned( 391, 10), to_unsigned( 394, 10), to_unsigned( 398, 10), to_unsigned( 401, 10), to_unsigned( 405, 10), to_unsigned( 408, 10),
        to_unsigned( 412, 10), to_unsigned( 415, 10), to_unsigned( 419, 10), to_unsigned( 422, 10), to_unsigned( 426, 10), to_unsigned( 429, 10), to_unsigned( 433, 10), to_unsigned( 436, 10),
        to_unsigned( 440, 10), to_unsigned( 443, 10), to_unsigned( 447, 10), to_unsigned( 450, 10), to_unsigned( 454, 10), to_unsigned( 457, 10), to_unsigned( 461, 10), to_unsigned( 464, 10),
        to_unsigned( 468, 10), to_unsigned( 471, 10), to_unsigned( 475, 10), to_unsigned( 478, 10), to_unsigned( 482, 10), to_unsigned( 485, 10), to_unsigned( 489, 10), to_unsigned( 492, 10),
        to_unsigned( 496, 10), to_unsigned( 499, 10), to_unsigned( 503, 10), to_unsigned( 506, 10), to_unsigned( 510, 10), to_unsigned( 513, 10), to_unsigned( 517, 10), to_unsigned( 520, 10),
        to_unsigned( 524, 10), to_unsigned( 527, 10), to_unsigned( 531, 10), to_unsigned( 534, 10), to_unsigned( 538, 10), to_unsigned( 541, 10), to_unsigned( 545, 10), to_unsigned( 548, 10),
        to_unsigned( 552, 10), to_unsigned( 555, 10), to_unsigned( 559, 10), to_unsigned( 562, 10), to_unsigned( 566, 10), to_unsigned( 569, 10), to_unsigned( 573, 10), to_unsigned( 576, 10),
        to_unsigned( 580, 10), to_unsigned( 583, 10), to_unsigned( 587, 10), to_unsigned( 590, 10), to_unsigned( 594, 10), to_unsigned( 597, 10), to_unsigned( 601, 10), to_unsigned( 604, 10),
        to_unsigned( 608, 10), to_unsigned( 611, 10), to_unsigned( 615, 10), to_unsigned( 618, 10), to_unsigned( 622, 10), to_unsigned( 625, 10), to_unsigned( 629, 10), to_unsigned( 632, 10),
        to_unsigned( 636, 10), to_unsigned( 639, 10), to_unsigned( 643, 10), to_unsigned( 646, 10), to_unsigned( 650, 10), to_unsigned( 653, 10), to_unsigned( 657, 10), to_unsigned( 660, 10),
        to_unsigned( 664, 10), to_unsigned( 667, 10), to_unsigned( 671, 10), to_unsigned( 674, 10), to_unsigned( 678, 10), to_unsigned( 681, 10), to_unsigned( 685, 10), to_unsigned( 688, 10),
        to_unsigned( 692, 10), to_unsigned( 695, 10), to_unsigned( 699, 10), to_unsigned( 702, 10), to_unsigned( 706, 10), to_unsigned( 709, 10), to_unsigned( 713, 10), to_unsigned( 716, 10),
        to_unsigned( 720, 10), to_unsigned( 723, 10), to_unsigned( 727, 10), to_unsigned( 730, 10), to_unsigned( 734, 10), to_unsigned( 737, 10), to_unsigned( 741, 10), to_unsigned( 744, 10),
        to_unsigned( 748, 10), to_unsigned( 751, 10), to_unsigned( 755, 10), to_unsigned( 758, 10), to_unsigned( 762, 10), to_unsigned( 765, 10), to_unsigned( 769, 10), to_unsigned( 772, 10),
        to_unsigned( 776, 10), to_unsigned( 779, 10), to_unsigned( 783, 10), to_unsigned( 786, 10), to_unsigned( 790, 10), to_unsigned( 793, 10), to_unsigned( 797, 10), to_unsigned( 800, 10),
        to_unsigned( 804, 10), to_unsigned( 807, 10), to_unsigned( 811, 10), to_unsigned( 814, 10), to_unsigned( 818, 10), to_unsigned( 821, 10), to_unsigned( 825, 10), to_unsigned( 828, 10),
        to_unsigned( 832, 10), to_unsigned( 835, 10), to_unsigned( 839, 10), to_unsigned( 837, 10), to_unsigned( 833, 10), to_unsigned( 830, 10), to_unsigned( 826, 10), to_unsigned( 823, 10),
        to_unsigned( 819, 10), to_unsigned( 816, 10), to_unsigned( 812, 10), to_unsigned( 809, 10), to_unsigned( 805, 10), to_unsigned( 801, 10), to_unsigned( 798, 10), to_unsigned( 794, 10),
        to_unsigned( 791, 10), to_unsigned( 787, 10), to_unsigned( 784, 10), to_unsigned( 780, 10), to_unsigned( 777, 10), to_unsigned( 773, 10), to_unsigned( 769, 10), to_unsigned( 766, 10),
        to_unsigned( 762, 10), to_unsigned( 759, 10), to_unsigned( 755, 10), to_unsigned( 752, 10), to_unsigned( 748, 10), to_unsigned( 745, 10), to_unsigned( 741, 10), to_unsigned( 737, 10),
        to_unsigned( 734, 10), to_unsigned( 730, 10), to_unsigned( 727, 10), to_unsigned( 723, 10), to_unsigned( 720, 10), to_unsigned( 716, 10), to_unsigned( 713, 10), to_unsigned( 709, 10),
        to_unsigned( 705, 10), to_unsigned( 702, 10), to_unsigned( 698, 10), to_unsigned( 695, 10), to_unsigned( 691, 10), to_unsigned( 688, 10), to_unsigned( 684, 10), to_unsigned( 681, 10),
        to_unsigned( 677, 10), to_unsigned( 673, 10), to_unsigned( 670, 10), to_unsigned( 666, 10), to_unsigned( 663, 10), to_unsigned( 659, 10), to_unsigned( 656, 10), to_unsigned( 652, 10),
        to_unsigned( 649, 10), to_unsigned( 645, 10), to_unsigned( 641, 10), to_unsigned( 638, 10), to_unsigned( 634, 10), to_unsigned( 631, 10), to_unsigned( 627, 10), to_unsigned( 624, 10),
        to_unsigned( 620, 10), to_unsigned( 617, 10), to_unsigned( 613, 10), to_unsigned( 609, 10), to_unsigned( 606, 10), to_unsigned( 602, 10), to_unsigned( 599, 10), to_unsigned( 595, 10),
        to_unsigned( 592, 10), to_unsigned( 588, 10), to_unsigned( 585, 10), to_unsigned( 581, 10), to_unsigned( 576, 10), to_unsigned( 571, 10), to_unsigned( 565, 10), to_unsigned( 559, 10),
        to_unsigned( 554, 10), to_unsigned( 548, 10), to_unsigned( 542, 10), to_unsigned( 536, 10), to_unsigned( 531, 10), to_unsigned( 525, 10), to_unsigned( 519, 10), to_unsigned( 514, 10),
        to_unsigned( 508, 10), to_unsigned( 502, 10), to_unsigned( 497, 10), to_unsigned( 491, 10), to_unsigned( 485, 10), to_unsigned( 480, 10), to_unsigned( 474, 10), to_unsigned( 468, 10),
        to_unsigned( 463, 10), to_unsigned( 457, 10), to_unsigned( 451, 10), to_unsigned( 445, 10), to_unsigned( 440, 10), to_unsigned( 434, 10), to_unsigned( 428, 10), to_unsigned( 423, 10),
        to_unsigned( 417, 10), to_unsigned( 411, 10), to_unsigned( 406, 10), to_unsigned( 400, 10), to_unsigned( 394, 10), to_unsigned( 389, 10), to_unsigned( 383, 10), to_unsigned( 377, 10));
    constant C_CY_HI : t_c10 := (
        to_unsigned( 372, 10), to_unsigned( 366, 10), to_unsigned( 360, 10), to_unsigned( 354, 10), to_unsigned( 349, 10), to_unsigned( 343, 10), to_unsigned( 337, 10), to_unsigned( 332, 10),
        to_unsigned( 326, 10), to_unsigned( 320, 10), to_unsigned( 315, 10), to_unsigned( 309, 10), to_unsigned( 303, 10), to_unsigned( 298, 10), to_unsigned( 292, 10), to_unsigned( 286, 10),
        to_unsigned( 281, 10), to_unsigned( 275, 10), to_unsigned( 269, 10), to_unsigned( 263, 10), to_unsigned( 258, 10), to_unsigned( 252, 10), to_unsigned( 246, 10), to_unsigned( 241, 10),
        to_unsigned( 235, 10), to_unsigned( 229, 10), to_unsigned( 224, 10), to_unsigned( 218, 10), to_unsigned( 212, 10), to_unsigned( 207, 10), to_unsigned( 201, 10), to_unsigned( 195, 10),
        to_unsigned( 190, 10), to_unsigned( 184, 10), to_unsigned( 178, 10), to_unsigned( 172, 10), to_unsigned( 167, 10), to_unsigned( 164, 10), to_unsigned( 164, 10), to_unsigned( 164, 10),
        to_unsigned( 165, 10), to_unsigned( 165, 10), to_unsigned( 166, 10), to_unsigned( 166, 10), to_unsigned( 166, 10), to_unsigned( 167, 10), to_unsigned( 167, 10), to_unsigned( 167, 10),
        to_unsigned( 168, 10), to_unsigned( 168, 10), to_unsigned( 169, 10), to_unsigned( 169, 10), to_unsigned( 169, 10), to_unsigned( 170, 10), to_unsigned( 170, 10), to_unsigned( 171, 10),
        to_unsigned( 171, 10), to_unsigned( 171, 10), to_unsigned( 172, 10), to_unsigned( 172, 10), to_unsigned( 172, 10), to_unsigned( 173, 10), to_unsigned( 173, 10), to_unsigned( 174, 10),
        to_unsigned( 174, 10), to_unsigned( 174, 10), to_unsigned( 175, 10), to_unsigned( 175, 10), to_unsigned( 176, 10), to_unsigned( 176, 10), to_unsigned( 176, 10), to_unsigned( 177, 10),
        to_unsigned( 177, 10), to_unsigned( 177, 10), to_unsigned( 178, 10), to_unsigned( 178, 10), to_unsigned( 179, 10), to_unsigned( 179, 10), to_unsigned( 179, 10), to_unsigned( 180, 10),
        to_unsigned( 180, 10), to_unsigned( 181, 10), to_unsigned( 181, 10), to_unsigned( 181, 10), to_unsigned( 182, 10), to_unsigned( 182, 10), to_unsigned( 182, 10), to_unsigned( 183, 10),
        to_unsigned( 183, 10), to_unsigned( 184, 10), to_unsigned( 184, 10), to_unsigned( 184, 10), to_unsigned( 185, 10), to_unsigned( 185, 10), to_unsigned( 185, 10), to_unsigned( 186, 10),
        to_unsigned( 186, 10), to_unsigned( 187, 10), to_unsigned( 187, 10), to_unsigned( 187, 10), to_unsigned( 188, 10), to_unsigned( 188, 10), to_unsigned( 189, 10), to_unsigned( 189, 10),
        to_unsigned( 189, 10), to_unsigned( 190, 10), to_unsigned( 190, 10), to_unsigned( 190, 10), to_unsigned( 191, 10), to_unsigned( 191, 10), to_unsigned( 192, 10), to_unsigned( 194, 10),
        to_unsigned( 196, 10), to_unsigned( 199, 10), to_unsigned( 201, 10), to_unsigned( 203, 10), to_unsigned( 205, 10), to_unsigned( 207, 10), to_unsigned( 209, 10), to_unsigned( 211, 10),
        to_unsigned( 214, 10), to_unsigned( 216, 10), to_unsigned( 218, 10), to_unsigned( 220, 10), to_unsigned( 222, 10), to_unsigned( 224, 10), to_unsigned( 226, 10), to_unsigned( 229, 10),
        to_unsigned( 231, 10), to_unsigned( 233, 10), to_unsigned( 235, 10), to_unsigned( 237, 10), to_unsigned( 239, 10), to_unsigned( 241, 10), to_unsigned( 244, 10), to_unsigned( 246, 10),
        to_unsigned( 248, 10), to_unsigned( 250, 10), to_unsigned( 252, 10), to_unsigned( 254, 10), to_unsigned( 257, 10), to_unsigned( 259, 10), to_unsigned( 261, 10), to_unsigned( 263, 10),
        to_unsigned( 265, 10), to_unsigned( 267, 10), to_unsigned( 269, 10), to_unsigned( 272, 10), to_unsigned( 274, 10), to_unsigned( 276, 10), to_unsigned( 278, 10), to_unsigned( 280, 10),
        to_unsigned( 282, 10), to_unsigned( 284, 10), to_unsigned( 287, 10), to_unsigned( 289, 10), to_unsigned( 291, 10), to_unsigned( 293, 10), to_unsigned( 295, 10), to_unsigned( 297, 10),
        to_unsigned( 299, 10), to_unsigned( 302, 10), to_unsigned( 304, 10), to_unsigned( 306, 10), to_unsigned( 308, 10), to_unsigned( 310, 10), to_unsigned( 312, 10), to_unsigned( 314, 10),
        to_unsigned( 317, 10), to_unsigned( 319, 10), to_unsigned( 321, 10), to_unsigned( 323, 10), to_unsigned( 325, 10), to_unsigned( 327, 10), to_unsigned( 329, 10), to_unsigned( 332, 10),
        to_unsigned( 334, 10), to_unsigned( 336, 10), to_unsigned( 338, 10), to_unsigned( 340, 10), to_unsigned( 342, 10), to_unsigned( 345, 10), to_unsigned( 347, 10), to_unsigned( 348, 10),
        to_unsigned( 348, 10), to_unsigned( 348, 10), to_unsigned( 348, 10), to_unsigned( 347, 10), to_unsigned( 347, 10), to_unsigned( 347, 10), to_unsigned( 346, 10), to_unsigned( 346, 10),
        to_unsigned( 346, 10), to_unsigned( 346, 10), to_unsigned( 345, 10), to_unsigned( 345, 10), to_unsigned( 345, 10), to_unsigned( 344, 10), to_unsigned( 344, 10), to_unsigned( 344, 10),
        to_unsigned( 344, 10), to_unsigned( 343, 10), to_unsigned( 343, 10), to_unsigned( 343, 10), to_unsigned( 342, 10), to_unsigned( 342, 10), to_unsigned( 342, 10), to_unsigned( 342, 10),
        to_unsigned( 341, 10), to_unsigned( 341, 10), to_unsigned( 341, 10), to_unsigned( 340, 10), to_unsigned( 340, 10), to_unsigned( 340, 10), to_unsigned( 340, 10), to_unsigned( 339, 10),
        to_unsigned( 339, 10), to_unsigned( 339, 10), to_unsigned( 338, 10), to_unsigned( 338, 10), to_unsigned( 338, 10), to_unsigned( 338, 10), to_unsigned( 337, 10), to_unsigned( 337, 10),
        to_unsigned( 337, 10), to_unsigned( 336, 10), to_unsigned( 336, 10), to_unsigned( 336, 10), to_unsigned( 336, 10), to_unsigned( 335, 10), to_unsigned( 335, 10), to_unsigned( 335, 10),
        to_unsigned( 334, 10), to_unsigned( 334, 10), to_unsigned( 334, 10), to_unsigned( 334, 10), to_unsigned( 333, 10), to_unsigned( 333, 10), to_unsigned( 333, 10), to_unsigned( 332, 10),
        to_unsigned( 332, 10), to_unsigned( 332, 10), to_unsigned( 332, 10), to_unsigned( 331, 10), to_unsigned( 331, 10), to_unsigned( 331, 10), to_unsigned( 330, 10), to_unsigned( 330, 10),
        to_unsigned( 330, 10), to_unsigned( 330, 10), to_unsigned( 329, 10), to_unsigned( 329, 10), to_unsigned( 329, 10), to_unsigned( 328, 10), to_unsigned( 328, 10), to_unsigned( 328, 10));
    constant C_CU_LO : t_c10 := (
        to_unsigned( 960, 10), to_unsigned( 957, 10), to_unsigned( 954, 10), to_unsigned( 952, 10), to_unsigned( 949, 10), to_unsigned( 947, 10), to_unsigned( 944, 10), to_unsigned( 942, 10),
        to_unsigned( 939, 10), to_unsigned( 936, 10), to_unsigned( 934, 10), to_unsigned( 931, 10), to_unsigned( 929, 10), to_unsigned( 926, 10), to_unsigned( 924, 10), to_unsigned( 921, 10),
        to_unsigned( 918, 10), to_unsigned( 916, 10), to_unsigned( 913, 10), to_unsigned( 911, 10), to_unsigned( 908, 10), to_unsigned( 906, 10), to_unsigned( 903, 10), to_unsigned( 900, 10),
        to_unsigned( 898, 10), to_unsigned( 895, 10), to_unsigned( 893, 10), to_unsigned( 890, 10), to_unsigned( 888, 10), to_unsigned( 885, 10), to_unsigned( 882, 10), to_unsigned( 880, 10),
        to_unsigned( 877, 10), to_unsigned( 875, 10), to_unsigned( 872, 10), to_unsigned( 870, 10), to_unsigned( 867, 10), to_unsigned( 864, 10), to_unsigned( 862, 10), to_unsigned( 859, 10),
        to_unsigned( 857, 10), to_unsigned( 854, 10), to_unsigned( 852, 10), to_unsigned( 849, 10), to_unsigned( 846, 10), to_unsigned( 844, 10), to_unsigned( 841, 10), to_unsigned( 839, 10),
        to_unsigned( 836, 10), to_unsigned( 834, 10), to_unsigned( 831, 10), to_unsigned( 828, 10), to_unsigned( 826, 10), to_unsigned( 823, 10), to_unsigned( 821, 10), to_unsigned( 818, 10),
        to_unsigned( 816, 10), to_unsigned( 813, 10), to_unsigned( 810, 10), to_unsigned( 808, 10), to_unsigned( 805, 10), to_unsigned( 803, 10), to_unsigned( 800, 10), to_unsigned( 798, 10),
        to_unsigned( 795, 10), to_unsigned( 792, 10), to_unsigned( 790, 10), to_unsigned( 787, 10), to_unsigned( 785, 10), to_unsigned( 782, 10), to_unsigned( 780, 10), to_unsigned( 777, 10),
        to_unsigned( 774, 10), to_unsigned( 772, 10), to_unsigned( 769, 10), to_unsigned( 767, 10), to_unsigned( 764, 10), to_unsigned( 762, 10), to_unsigned( 759, 10), to_unsigned( 756, 10),
        to_unsigned( 754, 10), to_unsigned( 751, 10), to_unsigned( 749, 10), to_unsigned( 746, 10), to_unsigned( 744, 10), to_unsigned( 741, 10), to_unsigned( 738, 10), to_unsigned( 736, 10),
        to_unsigned( 733, 10), to_unsigned( 731, 10), to_unsigned( 728, 10), to_unsigned( 726, 10), to_unsigned( 723, 10), to_unsigned( 720, 10), to_unsigned( 718, 10), to_unsigned( 715, 10),
        to_unsigned( 713, 10), to_unsigned( 710, 10), to_unsigned( 708, 10), to_unsigned( 705, 10), to_unsigned( 702, 10), to_unsigned( 700, 10), to_unsigned( 697, 10), to_unsigned( 695, 10),
        to_unsigned( 692, 10), to_unsigned( 690, 10), to_unsigned( 687, 10), to_unsigned( 684, 10), to_unsigned( 682, 10), to_unsigned( 679, 10), to_unsigned( 677, 10), to_unsigned( 674, 10),
        to_unsigned( 672, 10), to_unsigned( 669, 10), to_unsigned( 666, 10), to_unsigned( 664, 10), to_unsigned( 661, 10), to_unsigned( 659, 10), to_unsigned( 656, 10), to_unsigned( 654, 10),
        to_unsigned( 651, 10), to_unsigned( 648, 10), to_unsigned( 646, 10), to_unsigned( 643, 10), to_unsigned( 641, 10), to_unsigned( 638, 10), to_unsigned( 636, 10), to_unsigned( 633, 10),
        to_unsigned( 631, 10), to_unsigned( 628, 10), to_unsigned( 625, 10), to_unsigned( 623, 10), to_unsigned( 620, 10), to_unsigned( 618, 10), to_unsigned( 615, 10), to_unsigned( 613, 10),
        to_unsigned( 610, 10), to_unsigned( 607, 10), to_unsigned( 605, 10), to_unsigned( 602, 10), to_unsigned( 600, 10), to_unsigned( 597, 10), to_unsigned( 595, 10), to_unsigned( 592, 10),
        to_unsigned( 589, 10), to_unsigned( 587, 10), to_unsigned( 584, 10), to_unsigned( 579, 10), to_unsigned( 573, 10), to_unsigned( 567, 10), to_unsigned( 561, 10), to_unsigned( 555, 10),
        to_unsigned( 549, 10), to_unsigned( 542, 10), to_unsigned( 536, 10), to_unsigned( 530, 10), to_unsigned( 524, 10), to_unsigned( 518, 10), to_unsigned( 512, 10), to_unsigned( 506, 10),
        to_unsigned( 500, 10), to_unsigned( 493, 10), to_unsigned( 487, 10), to_unsigned( 481, 10), to_unsigned( 475, 10), to_unsigned( 469, 10), to_unsigned( 463, 10), to_unsigned( 457, 10),
        to_unsigned( 451, 10), to_unsigned( 444, 10), to_unsigned( 438, 10), to_unsigned( 432, 10), to_unsigned( 426, 10), to_unsigned( 420, 10), to_unsigned( 414, 10), to_unsigned( 408, 10),
        to_unsigned( 402, 10), to_unsigned( 395, 10), to_unsigned( 389, 10), to_unsigned( 383, 10), to_unsigned( 377, 10), to_unsigned( 371, 10), to_unsigned( 365, 10), to_unsigned( 359, 10),
        to_unsigned( 353, 10), to_unsigned( 346, 10), to_unsigned( 340, 10), to_unsigned( 334, 10), to_unsigned( 328, 10), to_unsigned( 322, 10), to_unsigned( 316, 10), to_unsigned( 310, 10),
        to_unsigned( 304, 10), to_unsigned( 297, 10), to_unsigned( 291, 10), to_unsigned( 285, 10), to_unsigned( 279, 10), to_unsigned( 273, 10), to_unsigned( 267, 10), to_unsigned( 261, 10),
        to_unsigned( 255, 10), to_unsigned( 248, 10), to_unsigned( 242, 10), to_unsigned( 236, 10), to_unsigned( 230, 10), to_unsigned( 224, 10), to_unsigned( 218, 10), to_unsigned( 212, 10),
        to_unsigned( 206, 10), to_unsigned( 199, 10), to_unsigned( 193, 10), to_unsigned( 187, 10), to_unsigned( 181, 10), to_unsigned( 175, 10), to_unsigned( 169, 10), to_unsigned( 163, 10),
        to_unsigned( 157, 10), to_unsigned( 150, 10), to_unsigned( 144, 10), to_unsigned( 138, 10), to_unsigned( 138, 10), to_unsigned( 142, 10), to_unsigned( 146, 10), to_unsigned( 150, 10),
        to_unsigned( 155, 10), to_unsigned( 159, 10), to_unsigned( 163, 10), to_unsigned( 167, 10), to_unsigned( 171, 10), to_unsigned( 175, 10), to_unsigned( 179, 10), to_unsigned( 184, 10),
        to_unsigned( 188, 10), to_unsigned( 192, 10), to_unsigned( 196, 10), to_unsigned( 200, 10), to_unsigned( 204, 10), to_unsigned( 209, 10), to_unsigned( 213, 10), to_unsigned( 217, 10),
        to_unsigned( 221, 10), to_unsigned( 225, 10), to_unsigned( 229, 10), to_unsigned( 233, 10), to_unsigned( 238, 10), to_unsigned( 242, 10), to_unsigned( 246, 10), to_unsigned( 250, 10),
        to_unsigned( 254, 10), to_unsigned( 258, 10), to_unsigned( 263, 10), to_unsigned( 267, 10), to_unsigned( 271, 10), to_unsigned( 275, 10), to_unsigned( 279, 10), to_unsigned( 283, 10));
    constant C_CU_HI : t_c10 := (
        to_unsigned( 288, 10), to_unsigned( 292, 10), to_unsigned( 296, 10), to_unsigned( 300, 10), to_unsigned( 304, 10), to_unsigned( 308, 10), to_unsigned( 312, 10), to_unsigned( 317, 10),
        to_unsigned( 321, 10), to_unsigned( 325, 10), to_unsigned( 329, 10), to_unsigned( 333, 10), to_unsigned( 337, 10), to_unsigned( 342, 10), to_unsigned( 346, 10), to_unsigned( 350, 10),
        to_unsigned( 354, 10), to_unsigned( 358, 10), to_unsigned( 362, 10), to_unsigned( 366, 10), to_unsigned( 371, 10), to_unsigned( 375, 10), to_unsigned( 379, 10), to_unsigned( 383, 10),
        to_unsigned( 387, 10), to_unsigned( 391, 10), to_unsigned( 396, 10), to_unsigned( 400, 10), to_unsigned( 404, 10), to_unsigned( 408, 10), to_unsigned( 412, 10), to_unsigned( 416, 10),
        to_unsigned( 421, 10), to_unsigned( 425, 10), to_unsigned( 429, 10), to_unsigned( 433, 10), to_unsigned( 437, 10), to_unsigned( 440, 10), to_unsigned( 443, 10), to_unsigned( 445, 10),
        to_unsigned( 447, 10), to_unsigned( 450, 10), to_unsigned( 452, 10), to_unsigned( 454, 10), to_unsigned( 457, 10), to_unsigned( 459, 10), to_unsigned( 461, 10), to_unsigned( 463, 10),
        to_unsigned( 466, 10), to_unsigned( 468, 10), to_unsigned( 470, 10), to_unsigned( 473, 10), to_unsigned( 475, 10), to_unsigned( 477, 10), to_unsigned( 480, 10), to_unsigned( 482, 10),
        to_unsigned( 484, 10), to_unsigned( 486, 10), to_unsigned( 489, 10), to_unsigned( 491, 10), to_unsigned( 493, 10), to_unsigned( 496, 10), to_unsigned( 498, 10), to_unsigned( 500, 10),
        to_unsigned( 503, 10), to_unsigned( 505, 10), to_unsigned( 507, 10), to_unsigned( 509, 10), to_unsigned( 512, 10), to_unsigned( 514, 10), to_unsigned( 516, 10), to_unsigned( 519, 10),
        to_unsigned( 521, 10), to_unsigned( 523, 10), to_unsigned( 525, 10), to_unsigned( 528, 10), to_unsigned( 530, 10), to_unsigned( 532, 10), to_unsigned( 535, 10), to_unsigned( 537, 10),
        to_unsigned( 539, 10), to_unsigned( 542, 10), to_unsigned( 544, 10), to_unsigned( 546, 10), to_unsigned( 548, 10), to_unsigned( 551, 10), to_unsigned( 553, 10), to_unsigned( 555, 10),
        to_unsigned( 558, 10), to_unsigned( 560, 10), to_unsigned( 562, 10), to_unsigned( 565, 10), to_unsigned( 567, 10), to_unsigned( 569, 10), to_unsigned( 571, 10), to_unsigned( 574, 10),
        to_unsigned( 576, 10), to_unsigned( 578, 10), to_unsigned( 581, 10), to_unsigned( 583, 10), to_unsigned( 585, 10), to_unsigned( 587, 10), to_unsigned( 590, 10), to_unsigned( 592, 10),
        to_unsigned( 594, 10), to_unsigned( 597, 10), to_unsigned( 599, 10), to_unsigned( 601, 10), to_unsigned( 604, 10), to_unsigned( 606, 10), to_unsigned( 608, 10), to_unsigned( 610, 10),
        to_unsigned( 612, 10), to_unsigned( 614, 10), to_unsigned( 616, 10), to_unsigned( 618, 10), to_unsigned( 620, 10), to_unsigned( 622, 10), to_unsigned( 624, 10), to_unsigned( 626, 10),
        to_unsigned( 628, 10), to_unsigned( 630, 10), to_unsigned( 632, 10), to_unsigned( 634, 10), to_unsigned( 636, 10), to_unsigned( 638, 10), to_unsigned( 640, 10), to_unsigned( 642, 10),
        to_unsigned( 645, 10), to_unsigned( 647, 10), to_unsigned( 649, 10), to_unsigned( 651, 10), to_unsigned( 653, 10), to_unsigned( 655, 10), to_unsigned( 657, 10), to_unsigned( 659, 10),
        to_unsigned( 661, 10), to_unsigned( 663, 10), to_unsigned( 665, 10), to_unsigned( 667, 10), to_unsigned( 669, 10), to_unsigned( 671, 10), to_unsigned( 673, 10), to_unsigned( 675, 10),
        to_unsigned( 677, 10), to_unsigned( 679, 10), to_unsigned( 681, 10), to_unsigned( 683, 10), to_unsigned( 685, 10), to_unsigned( 687, 10), to_unsigned( 689, 10), to_unsigned( 691, 10),
        to_unsigned( 693, 10), to_unsigned( 695, 10), to_unsigned( 697, 10), to_unsigned( 699, 10), to_unsigned( 701, 10), to_unsigned( 703, 10), to_unsigned( 705, 10), to_unsigned( 707, 10),
        to_unsigned( 709, 10), to_unsigned( 711, 10), to_unsigned( 713, 10), to_unsigned( 715, 10), to_unsigned( 717, 10), to_unsigned( 719, 10), to_unsigned( 721, 10), to_unsigned( 723, 10),
        to_unsigned( 725, 10), to_unsigned( 727, 10), to_unsigned( 729, 10), to_unsigned( 732, 10), to_unsigned( 734, 10), to_unsigned( 736, 10), to_unsigned( 738, 10), to_unsigned( 740, 10),
        to_unsigned( 742, 10), to_unsigned( 744, 10), to_unsigned( 746, 10), to_unsigned( 748, 10), to_unsigned( 750, 10), to_unsigned( 752, 10), to_unsigned( 754, 10), to_unsigned( 756, 10),
        to_unsigned( 759, 10), to_unsigned( 761, 10), to_unsigned( 764, 10), to_unsigned( 767, 10), to_unsigned( 770, 10), to_unsigned( 773, 10), to_unsigned( 775, 10), to_unsigned( 778, 10),
        to_unsigned( 781, 10), to_unsigned( 784, 10), to_unsigned( 787, 10), to_unsigned( 789, 10), to_unsigned( 792, 10), to_unsigned( 795, 10), to_unsigned( 798, 10), to_unsigned( 801, 10),
        to_unsigned( 803, 10), to_unsigned( 806, 10), to_unsigned( 809, 10), to_unsigned( 812, 10), to_unsigned( 814, 10), to_unsigned( 817, 10), to_unsigned( 820, 10), to_unsigned( 823, 10),
        to_unsigned( 826, 10), to_unsigned( 828, 10), to_unsigned( 831, 10), to_unsigned( 834, 10), to_unsigned( 837, 10), to_unsigned( 840, 10), to_unsigned( 842, 10), to_unsigned( 845, 10),
        to_unsigned( 848, 10), to_unsigned( 851, 10), to_unsigned( 854, 10), to_unsigned( 856, 10), to_unsigned( 859, 10), to_unsigned( 862, 10), to_unsigned( 865, 10), to_unsigned( 867, 10),
        to_unsigned( 870, 10), to_unsigned( 873, 10), to_unsigned( 876, 10), to_unsigned( 879, 10), to_unsigned( 881, 10), to_unsigned( 884, 10), to_unsigned( 887, 10), to_unsigned( 890, 10),
        to_unsigned( 893, 10), to_unsigned( 895, 10), to_unsigned( 898, 10), to_unsigned( 901, 10), to_unsigned( 904, 10), to_unsigned( 907, 10), to_unsigned( 909, 10), to_unsigned( 912, 10),
        to_unsigned( 915, 10), to_unsigned( 918, 10), to_unsigned( 920, 10), to_unsigned( 923, 10), to_unsigned( 926, 10), to_unsigned( 929, 10), to_unsigned( 932, 10), to_unsigned( 934, 10),
        to_unsigned( 937, 10), to_unsigned( 940, 10), to_unsigned( 943, 10), to_unsigned( 946, 10), to_unsigned( 948, 10), to_unsigned( 951, 10), to_unsigned( 954, 10), to_unsigned( 957, 10));
    constant C_CV_LO : t_c10 := (
        to_unsigned( 360, 10), to_unsigned( 357, 10), to_unsigned( 355, 10), to_unsigned( 353, 10), to_unsigned( 351, 10), to_unsigned( 349, 10), to_unsigned( 347, 10), to_unsigned( 345, 10),
        to_unsigned( 343, 10), to_unsigned( 341, 10), to_unsigned( 339, 10), to_unsigned( 337, 10), to_unsigned( 335, 10), to_unsigned( 333, 10), to_unsigned( 331, 10), to_unsigned( 329, 10),
        to_unsigned( 327, 10), to_unsigned( 325, 10), to_unsigned( 323, 10), to_unsigned( 321, 10), to_unsigned( 319, 10), to_unsigned( 317, 10), to_unsigned( 315, 10), to_unsigned( 313, 10),
        to_unsigned( 311, 10), to_unsigned( 309, 10), to_unsigned( 307, 10), to_unsigned( 305, 10), to_unsigned( 303, 10), to_unsigned( 301, 10), to_unsigned( 299, 10), to_unsigned( 297, 10),
        to_unsigned( 295, 10), to_unsigned( 293, 10), to_unsigned( 291, 10), to_unsigned( 289, 10), to_unsigned( 287, 10), to_unsigned( 285, 10), to_unsigned( 283, 10), to_unsigned( 281, 10),
        to_unsigned( 279, 10), to_unsigned( 277, 10), to_unsigned( 275, 10), to_unsigned( 272, 10), to_unsigned( 270, 10), to_unsigned( 268, 10), to_unsigned( 266, 10), to_unsigned( 264, 10),
        to_unsigned( 262, 10), to_unsigned( 260, 10), to_unsigned( 258, 10), to_unsigned( 256, 10), to_unsigned( 254, 10), to_unsigned( 252, 10), to_unsigned( 250, 10), to_unsigned( 248, 10),
        to_unsigned( 246, 10), to_unsigned( 244, 10), to_unsigned( 242, 10), to_unsigned( 240, 10), to_unsigned( 238, 10), to_unsigned( 236, 10), to_unsigned( 234, 10), to_unsigned( 232, 10),
        to_unsigned( 230, 10), to_unsigned( 228, 10), to_unsigned( 226, 10), to_unsigned( 224, 10), to_unsigned( 222, 10), to_unsigned( 220, 10), to_unsigned( 218, 10), to_unsigned( 216, 10),
        to_unsigned( 214, 10), to_unsigned( 212, 10), to_unsigned( 210, 10), to_unsigned( 208, 10), to_unsigned( 206, 10), to_unsigned( 204, 10), to_unsigned( 202, 10), to_unsigned( 200, 10),
        to_unsigned( 198, 10), to_unsigned( 196, 10), to_unsigned( 194, 10), to_unsigned( 192, 10), to_unsigned( 190, 10), to_unsigned( 188, 10), to_unsigned( 185, 10), to_unsigned( 183, 10),
        to_unsigned( 181, 10), to_unsigned( 179, 10), to_unsigned( 177, 10), to_unsigned( 175, 10), to_unsigned( 173, 10), to_unsigned( 171, 10), to_unsigned( 169, 10), to_unsigned( 167, 10),
        to_unsigned( 165, 10), to_unsigned( 163, 10), to_unsigned( 161, 10), to_unsigned( 159, 10), to_unsigned( 157, 10), to_unsigned( 155, 10), to_unsigned( 153, 10), to_unsigned( 151, 10),
        to_unsigned( 149, 10), to_unsigned( 147, 10), to_unsigned( 145, 10), to_unsigned( 143, 10), to_unsigned( 141, 10), to_unsigned( 139, 10), to_unsigned( 137, 10), to_unsigned( 135, 10),
        to_unsigned( 133, 10), to_unsigned( 131, 10), to_unsigned( 129, 10), to_unsigned( 127, 10), to_unsigned( 125, 10), to_unsigned( 123, 10), to_unsigned( 121, 10), to_unsigned( 119, 10),
        to_unsigned( 117, 10), to_unsigned( 115, 10), to_unsigned( 113, 10), to_unsigned( 111, 10), to_unsigned( 109, 10), to_unsigned( 107, 10), to_unsigned( 105, 10), to_unsigned( 103, 10),
        to_unsigned( 101, 10), to_unsigned(  98, 10), to_unsigned(  96, 10), to_unsigned(  94, 10), to_unsigned(  92, 10), to_unsigned(  90, 10), to_unsigned(  88, 10), to_unsigned(  86, 10),
        to_unsigned(  84, 10), to_unsigned(  82, 10), to_unsigned(  80, 10), to_unsigned(  78, 10), to_unsigned(  76, 10), to_unsigned(  74, 10), to_unsigned(  72, 10), to_unsigned(  70, 10),
        to_unsigned(  68, 10), to_unsigned(  66, 10), to_unsigned(  64, 10), to_unsigned(  65, 10), to_unsigned(  67, 10), to_unsigned(  69, 10), to_unsigned(  71, 10), to_unsigned(  73, 10),
        to_unsigned(  75, 10), to_unsigned(  77, 10), to_unsigned(  80, 10), to_unsigned(  82, 10), to_unsigned(  84, 10), to_unsigned(  86, 10), to_unsigned(  88, 10), to_unsigned(  90, 10),
        to_unsigned(  92, 10), to_unsigned(  94, 10), to_unsigned(  96, 10), to_unsigned(  98, 10), to_unsigned( 100, 10), to_unsigned( 102, 10), to_unsigned( 104, 10), to_unsigned( 107, 10),
        to_unsigned( 109, 10), to_unsigned( 111, 10), to_unsigned( 113, 10), to_unsigned( 115, 10), to_unsigned( 117, 10), to_unsigned( 119, 10), to_unsigned( 121, 10), to_unsigned( 123, 10),
        to_unsigned( 125, 10), to_unsigned( 127, 10), to_unsigned( 129, 10), to_unsigned( 131, 10), to_unsigned( 134, 10), to_unsigned( 136, 10), to_unsigned( 138, 10), to_unsigned( 140, 10),
        to_unsigned( 142, 10), to_unsigned( 144, 10), to_unsigned( 146, 10), to_unsigned( 148, 10), to_unsigned( 150, 10), to_unsigned( 152, 10), to_unsigned( 154, 10), to_unsigned( 156, 10),
        to_unsigned( 159, 10), to_unsigned( 161, 10), to_unsigned( 163, 10), to_unsigned( 165, 10), to_unsigned( 167, 10), to_unsigned( 169, 10), to_unsigned( 171, 10), to_unsigned( 173, 10),
        to_unsigned( 175, 10), to_unsigned( 177, 10), to_unsigned( 179, 10), to_unsigned( 181, 10), to_unsigned( 183, 10), to_unsigned( 186, 10), to_unsigned( 188, 10), to_unsigned( 190, 10),
        to_unsigned( 192, 10), to_unsigned( 194, 10), to_unsigned( 196, 10), to_unsigned( 198, 10), to_unsigned( 200, 10), to_unsigned( 202, 10), to_unsigned( 204, 10), to_unsigned( 206, 10),
        to_unsigned( 208, 10), to_unsigned( 210, 10), to_unsigned( 213, 10), to_unsigned( 215, 10), to_unsigned( 221, 10), to_unsigned( 231, 10), to_unsigned( 242, 10), to_unsigned( 252, 10),
        to_unsigned( 262, 10), to_unsigned( 272, 10), to_unsigned( 282, 10), to_unsigned( 293, 10), to_unsigned( 303, 10), to_unsigned( 313, 10), to_unsigned( 323, 10), to_unsigned( 333, 10),
        to_unsigned( 343, 10), to_unsigned( 354, 10), to_unsigned( 364, 10), to_unsigned( 374, 10), to_unsigned( 384, 10), to_unsigned( 394, 10), to_unsigned( 404, 10), to_unsigned( 415, 10),
        to_unsigned( 425, 10), to_unsigned( 435, 10), to_unsigned( 445, 10), to_unsigned( 455, 10), to_unsigned( 465, 10), to_unsigned( 476, 10), to_unsigned( 486, 10), to_unsigned( 496, 10),
        to_unsigned( 506, 10), to_unsigned( 516, 10), to_unsigned( 526, 10), to_unsigned( 537, 10), to_unsigned( 547, 10), to_unsigned( 557, 10), to_unsigned( 567, 10), to_unsigned( 577, 10));
    constant C_CV_HI : t_c10 := (
        to_unsigned( 588, 10), to_unsigned( 598, 10), to_unsigned( 608, 10), to_unsigned( 618, 10), to_unsigned( 628, 10), to_unsigned( 638, 10), to_unsigned( 649, 10), to_unsigned( 659, 10),
        to_unsigned( 669, 10), to_unsigned( 679, 10), to_unsigned( 689, 10), to_unsigned( 699, 10), to_unsigned( 710, 10), to_unsigned( 720, 10), to_unsigned( 730, 10), to_unsigned( 740, 10),
        to_unsigned( 750, 10), to_unsigned( 760, 10), to_unsigned( 771, 10), to_unsigned( 781, 10), to_unsigned( 791, 10), to_unsigned( 801, 10), to_unsigned( 811, 10), to_unsigned( 821, 10),
        to_unsigned( 832, 10), to_unsigned( 842, 10), to_unsigned( 852, 10), to_unsigned( 862, 10), to_unsigned( 872, 10), to_unsigned( 882, 10), to_unsigned( 893, 10), to_unsigned( 903, 10),
        to_unsigned( 913, 10), to_unsigned( 923, 10), to_unsigned( 933, 10), to_unsigned( 944, 10), to_unsigned( 954, 10), to_unsigned( 958, 10), to_unsigned( 954, 10), to_unsigned( 951, 10),
        to_unsigned( 947, 10), to_unsigned( 944, 10), to_unsigned( 940, 10), to_unsigned( 936, 10), to_unsigned( 933, 10), to_unsigned( 929, 10), to_unsigned( 925, 10), to_unsigned( 922, 10),
        to_unsigned( 918, 10), to_unsigned( 915, 10), to_unsigned( 911, 10), to_unsigned( 907, 10), to_unsigned( 904, 10), to_unsigned( 900, 10), to_unsigned( 897, 10), to_unsigned( 893, 10),
        to_unsigned( 889, 10), to_unsigned( 886, 10), to_unsigned( 882, 10), to_unsigned( 879, 10), to_unsigned( 875, 10), to_unsigned( 871, 10), to_unsigned( 868, 10), to_unsigned( 864, 10),
        to_unsigned( 861, 10), to_unsigned( 857, 10), to_unsigned( 853, 10), to_unsigned( 850, 10), to_unsigned( 846, 10), to_unsigned( 842, 10), to_unsigned( 839, 10), to_unsigned( 835, 10),
        to_unsigned( 832, 10), to_unsigned( 828, 10), to_unsigned( 824, 10), to_unsigned( 821, 10), to_unsigned( 817, 10), to_unsigned( 814, 10), to_unsigned( 810, 10), to_unsigned( 806, 10),
        to_unsigned( 803, 10), to_unsigned( 799, 10), to_unsigned( 796, 10), to_unsigned( 792, 10), to_unsigned( 788, 10), to_unsigned( 785, 10), to_unsigned( 781, 10), to_unsigned( 777, 10),
        to_unsigned( 774, 10), to_unsigned( 770, 10), to_unsigned( 767, 10), to_unsigned( 763, 10), to_unsigned( 759, 10), to_unsigned( 756, 10), to_unsigned( 752, 10), to_unsigned( 749, 10),
        to_unsigned( 745, 10), to_unsigned( 741, 10), to_unsigned( 738, 10), to_unsigned( 734, 10), to_unsigned( 731, 10), to_unsigned( 727, 10), to_unsigned( 723, 10), to_unsigned( 720, 10),
        to_unsigned( 716, 10), to_unsigned( 713, 10), to_unsigned( 709, 10), to_unsigned( 705, 10), to_unsigned( 702, 10), to_unsigned( 698, 10), to_unsigned( 696, 10), to_unsigned( 698, 10),
        to_unsigned( 700, 10), to_unsigned( 703, 10), to_unsigned( 705, 10), to_unsigned( 707, 10), to_unsigned( 709, 10), to_unsigned( 711, 10), to_unsigned( 713, 10), to_unsigned( 715, 10),
        to_unsigned( 717, 10), to_unsigned( 720, 10), to_unsigned( 722, 10), to_unsigned( 724, 10), to_unsigned( 726, 10), to_unsigned( 728, 10), to_unsigned( 730, 10), to_unsigned( 732, 10),
        to_unsigned( 735, 10), to_unsigned( 737, 10), to_unsigned( 739, 10), to_unsigned( 741, 10), to_unsigned( 743, 10), to_unsigned( 745, 10), to_unsigned( 747, 10), to_unsigned( 749, 10),
        to_unsigned( 752, 10), to_unsigned( 754, 10), to_unsigned( 756, 10), to_unsigned( 758, 10), to_unsigned( 760, 10), to_unsigned( 762, 10), to_unsigned( 764, 10), to_unsigned( 766, 10),
        to_unsigned( 769, 10), to_unsigned( 771, 10), to_unsigned( 773, 10), to_unsigned( 775, 10), to_unsigned( 777, 10), to_unsigned( 779, 10), to_unsigned( 781, 10), to_unsigned( 784, 10),
        to_unsigned( 786, 10), to_unsigned( 788, 10), to_unsigned( 790, 10), to_unsigned( 792, 10), to_unsigned( 794, 10), to_unsigned( 796, 10), to_unsigned( 798, 10), to_unsigned( 801, 10),
        to_unsigned( 803, 10), to_unsigned( 805, 10), to_unsigned( 807, 10), to_unsigned( 809, 10), to_unsigned( 811, 10), to_unsigned( 813, 10), to_unsigned( 816, 10), to_unsigned( 818, 10),
        to_unsigned( 820, 10), to_unsigned( 822, 10), to_unsigned( 824, 10), to_unsigned( 826, 10), to_unsigned( 828, 10), to_unsigned( 830, 10), to_unsigned( 833, 10), to_unsigned( 835, 10),
        to_unsigned( 837, 10), to_unsigned( 839, 10), to_unsigned( 841, 10), to_unsigned( 843, 10), to_unsigned( 845, 10), to_unsigned( 848, 10), to_unsigned( 850, 10), to_unsigned( 851, 10),
        to_unsigned( 844, 10), to_unsigned( 837, 10), to_unsigned( 830, 10), to_unsigned( 824, 10), to_unsigned( 817, 10), to_unsigned( 810, 10), to_unsigned( 803, 10), to_unsigned( 797, 10),
        to_unsigned( 790, 10), to_unsigned( 783, 10), to_unsigned( 777, 10), to_unsigned( 770, 10), to_unsigned( 763, 10), to_unsigned( 756, 10), to_unsigned( 750, 10), to_unsigned( 743, 10),
        to_unsigned( 736, 10), to_unsigned( 729, 10), to_unsigned( 723, 10), to_unsigned( 716, 10), to_unsigned( 709, 10), to_unsigned( 703, 10), to_unsigned( 696, 10), to_unsigned( 689, 10),
        to_unsigned( 682, 10), to_unsigned( 676, 10), to_unsigned( 669, 10), to_unsigned( 662, 10), to_unsigned( 655, 10), to_unsigned( 649, 10), to_unsigned( 642, 10), to_unsigned( 635, 10),
        to_unsigned( 629, 10), to_unsigned( 622, 10), to_unsigned( 615, 10), to_unsigned( 608, 10), to_unsigned( 602, 10), to_unsigned( 595, 10), to_unsigned( 588, 10), to_unsigned( 581, 10),
        to_unsigned( 575, 10), to_unsigned( 568, 10), to_unsigned( 561, 10), to_unsigned( 555, 10), to_unsigned( 548, 10), to_unsigned( 541, 10), to_unsigned( 534, 10), to_unsigned( 528, 10),
        to_unsigned( 521, 10), to_unsigned( 514, 10), to_unsigned( 507, 10), to_unsigned( 501, 10), to_unsigned( 494, 10), to_unsigned( 487, 10), to_unsigned( 481, 10), to_unsigned( 474, 10),
        to_unsigned( 467, 10), to_unsigned( 460, 10), to_unsigned( 454, 10), to_unsigned( 447, 10), to_unsigned( 440, 10), to_unsigned( 433, 10), to_unsigned( 427, 10), to_unsigned( 420, 10),
        to_unsigned( 413, 10), to_unsigned( 407, 10), to_unsigned( 400, 10), to_unsigned( 393, 10), to_unsigned( 386, 10), to_unsigned( 380, 10), to_unsigned( 373, 10), to_unsigned( 366, 10));

    -- ==== timing / raster ====
    signal s_timing  : t_video_timing_port;
    signal s_hcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_meas_v  : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace   : std_logic := '0';
    signal s_botf    : std_logic := '0';

    -- ==== per-raster class constants (latched at vsync) ====
    signal s_cols  : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_kwav  : unsigned(11 downto 0) := to_unsigned(34, 12);   -- 65536/cols
    signal s_kw45  : unsigned(11 downto 0) := to_unsigned(153, 12);  -- kwav * 4.5
    signal s_fh    : unsigned(11 downto 0) := to_unsigned(1080, 12); -- frame rows

    -- ==== controls (latched once per field; slewed where noted) ====
    signal s_solid  : std_logic := '0';   -- S7
    signal s_bgvid  : std_logic := '0';   -- S8
    signal s_fill   : std_logic := '0';   -- S9
    signal s_along  : std_logic := '0';   -- S10
    signal s_mirror : std_logic := '0';   -- S11
    signal s_k3     : unsigned(9 downto 0) := C_MID;   -- speed (raw)
    signal s_k5     : unsigned(9 downto 0) := C_MID;   -- flow (raw)
    -- analog vsync serrates into several edges per field: act on the first
    -- edge after active video only (saw-active guard)
    signal s_saw    : std_logic := '0';
    -- generated avid (parity-locked start, fixed-width end)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
    signal s_vs_go  : std_logic := '0';

    -- slew accumulators, Q10.4
    signal s_a1  : unsigned(13 downto 0) := to_unsigned(360*16, 14);
    signal s_a2  : unsigned(13 downto 0) := to_unsigned(400*16, 14);
    signal s_a4  : unsigned(13 downto 0) := to_unsigned(250*16, 14);
    signal s_a6  : unsigned(13 downto 0) := to_unsigned(256*16, 14);
    signal s_a12 : unsigned(13 downto 0) := to_unsigned(350*16, 14);
    signal s_k1s, s_k2s, s_k4s, s_k6s, s_p12s : unsigned(9 downto 0) := (others => '0');

    -- motion phases (Q16 = one wave / one hue cycle)
    signal s_pht1 : unsigned(15 downto 0) := (others => '0');
    signal s_pht2 : unsigned(15 downto 0) := to_unsigned(9000, 16);
    signal s_pht3 : unsigned(15 downto 0) := to_unsigned(41000, 16);
    -- colour phase: 0.4375 cycle puts red at the band's top edge and violet
    -- at its bottom at Spectrum 100% with Flow still
    signal s_phc  : unsigned(15 downto 0) := to_unsigned(28672, 16);
    signal s_vel  : signed(11 downto 0) := (others => '0');   -- wave speed / frame
    signal s_vel2, s_vel3 : signed(11 downto 0) := (others => '0');
    signal s_flow : signed(11 downto 0) := (others => '0');
    signal s_sad, s_fad : unsigned(9 downto 0) := (others => '0');
    signal s_sneg, s_fneg : std_logic := '0';
    signal s_nq   : unsigned(5 downto 0) := (others => '0');  -- noise level Q6

    -- ==== per-frame parameters (written by the vblank sequencer) ====
    signal s_cyc   : unsigned(10 downto 0) := to_unsigned(256, 11);  -- 256 = 1 cycle
    signal s_Pq2   : unsigned(13 downto 0) := to_unsigned(2160, 14); -- centre, q2
    signal s_Amin  : unsigned(11 downto 0) := to_unsigned(135, 12);
    signal s_fh2m  : unsigned(12 downto 0) := to_unsigned(2025, 13); -- fh*2 - Amin
    signal s_A     : unsigned(12 downto 0) := to_unsigned(800, 13);  -- amplitude, q2
    signal s_hk    : unsigned(9 downto 0) := (others => '0');
    signal s_Hq2   : unsigned(12 downto 0) := to_unsigned(300, 13);  -- half-thickness, q2
    signal s_H31   : unsigned(12 downto 0) := to_unsigned(331, 13);  -- + feather
    signal s_wsq   : unsigned(9 downto 0) := (others => '0');
    signal s_k1add : unsigned(11 downto 0) := (others => '0');
    signal s_k1inc : unsigned(11 downto 0) := to_unsigned(40, 12);
    signal s_k2inc : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal s_k3inc : unsigned(11 downto 0) := to_unsigned(15, 12);
    signal s_kx    : unsigned(11 downto 0) := to_unsigned(34, 12);   -- hue/column (Along)
    signal s_hscale : unsigned(11 downto 0) := to_unsigned(99, 12);  -- hue/q2-row (Across)

    -- shared sequential multiplier + divider (all inside blanking)
    signal s_op   : integer range 0 to 31 := 31;
    signal s_bit  : integer range 0 to 24 := 0;
    signal s_mA   : unsigned(23 downto 0) := (others => '0');
    signal s_mB   : unsigned(10 downto 0) := (others => '0');
    signal s_acc  : unsigned(23 downto 0) := (others => '0');
    signal s_dn   : unsigned(23 downto 0) := (others => '0');    -- divide numerator
    signal s_dr   : unsigned(14 downto 0) := (others => '0');    -- remainder
    signal s_dq   : unsigned(23 downto 0) := (others => '0');    -- quotient
    -- divisor is REGISTERED per divide (a mux feeding the borrow chain was
    -- once the design's critical path; nextpnr times vblank logic at the
    -- pixel clock)
    signal s_dvsr : unsigned(14 downto 0) := to_unsigned(1, 15);
    -- feather scale (v3.0.5): 1080 / fh in Q.8 (256 at 1080 rows), so the
    -- edge feather spans the same fraction of the screen height on every
    -- raster (31 q2 at 1080p; fewer real rows at 720p / SD)
    signal s_vs   : unsigned(11 downto 0) := to_unsigned(256, 12);

    -- ==== scanner (monotone left-to-right, one column per clock) ====
    signal s_run     : std_logic := '0';
    signal s_Rq2     : unsigned(13 downto 0) := (others => '0');
    signal s_R2q2    : unsigned(13 downto 0) := (others => '0');  -- reflected row
    signal s_fhm4    : unsigned(13 downto 0) := to_unsigned(4316, 14); -- (fh-1)*4
    signal s_nrow    : unsigned(11 downto 0) := (others => '0');
    signal s_trig_ok : std_logic := '0';
    signal s_vb_ok   : std_logic := '0';
    signal s_rq2n    : unsigned(13 downto 0) := (others => '0');
    signal s_col     : unsigned(11 downto 0) := (others => '0');
    signal s_wcol    : unsigned(11 downto 0) := (others => '0');
    signal s_hxw     : unsigned(15 downto 0) := (others => '0');  -- Along hue
    signal s_th1, s_th2, s_th3 : unsigned(15 downto 0) := (others => '0');

    signal s_v : std_logic_vector(1 to 15) := (others => '0');

    -- m1: quarter-wave fold, each lobe at idx and idx+1 (interpolated)
    signal m1_i1a, m1_i1b, m1_i2a, m1_i2b, m1_i3a, m1_i3b : unsigned(7 downto 0) := (others => '0');
    signal m1_s1a, m1_s1b, m1_s2a, m1_s2b, m1_s3a, m1_s3b : std_logic := '0';
    signal m1_f1, m1_f2, m1_f3 : unsigned(5 downto 0) := (others => '0');
    -- m2: ROM data
    signal m2_q1a, m2_q1b, m2_q2a, m2_q2b, m2_q3a, m2_q3b : unsigned(11 downto 0) := (others => '0');
    signal m2_s1a, m2_s1b, m2_s2a, m2_s2b, m2_s3a, m2_s3b : std_logic := '0';
    signal m2_f1, m2_f2, m2_f3 : unsigned(5 downto 0) := (others => '0');
    -- m2b: registered signed ROM data + neighbour deltas
    signal m2b_v1, m2b_v2, m2b_v3 : signed(12 downto 0) := (others => '0');
    signal m2b_d1, m2b_d2, m2b_d3 : signed(7 downto 0) := (others => '0');
    signal m2b_f1, m2b_f2, m2b_f3 : unsigned(5 downto 0) := (others => '0');
    -- m3: interpolated sines
    signal m3_s1, m3_s2, m3_s3 : signed(12 downto 0) := (others => '0');
    -- m3b: wave lobe + summed noise lobes
    signal m3b_sf : signed(12 downto 0) := (others => '0');
    signal m3b_ns : signed(13 downto 0) := (others => '0');
    -- m4a: noise x level
    signal m4a_sf : signed(12 downto 0) := (others => '0');
    signal m4a_nz : signed(20 downto 0) := (others => '0');
    -- m4s: wave + noise; thickness x noise
    signal m4s_sum : signed(13 downto 0) := (others => '0');
    signal m4s_hb  : signed(21 downto 0) := (others => '0');
    -- m4b: amplitude x sum (split), breathed half-thickness
    signal m4_hi  : signed(20 downto 0) := (others => '0');
    signal m4_lo  : signed(21 downto 0) := (others => '0');
    signal m4_h   : unsigned(12 downto 0) := (others => '0');
    -- m4c: recombine
    signal m4_p1  : signed(27 downto 0) := (others => '0');
    signal m4c_h  : unsigned(12 downto 0) := (others => '0');
    -- m5: centre height at this column
    signal m5_y   : signed(15 downto 0) := (others => '0');
    signal m5_h   : unsigned(12 downto 0) := (others => '0');
    -- m6: signed distances to this row and to the mirrored row
    signal m6_d1  : signed(15 downto 0) := (others => '0');
    signal m6_n2  : signed(15 downto 0) := (others => '0');
    signal m6_a1, m6_a2 : unsigned(14 downto 0) := (others => '0');
    signal m6_h   : unsigned(12 downto 0) := (others => '0');
    -- m6b: nearer copy wins
    signal m6b_dp : unsigned(12 downto 0) := (others => '0');
    signal m6b_po : signed(15 downto 0) := (others => '0');
    signal m6b_h  : unsigned(12 downto 0) := (others => '0');
    -- m6c: saturated signed offset
    signal m6c_po : signed(13 downto 0) := (others => '0');
    signal m6c_u  : unsigned(4 downto 0) := (others => '0');  -- feather depth
    signal m6c_in : std_logic := '0';                         -- inside the core
    -- m9: scaled feather depth + hue partials, m10: alpha, recombined hue
    signal m9_t   : unsigned(16 downto 0) := (others => '0');
    signal m9_in  : std_logic := '0';
    signal m9_hhi, m9_hlo : signed(20 downto 0) := (others => '0');
    signal m10_a  : unsigned(5 downto 0) := (others => '0');
    signal m10_hp : unsigned(15 downto 0) := (others => '0');

    signal s_st_we   : std_logic := '0';
    signal s_st_addr : unsigned(10 downto 0) := (others => '0');
    signal s_st_data : std_logic_vector(15 downto 0) := (others => '0');

    -- ==== line buffer: ONE bank, 2048 x 16, one writer, one reader ====
    -- 16 LIVE bits (hue10 & alpha6) so the word is a power of two wide
    type t_lb is array(0 to 2047) of std_logic_vector(15 downto 0);
    signal s_lb     : t_lb := (others => (others => '0'));
    signal s_rd     : std_logic_vector(15 downto 0) := (others => '0');
    signal s_raddr  : unsigned(10 downto 0) := (others => '0');
    signal s_rar    : unsigned(10 downto 0) := (others => '0');

    -- ==== beam pipeline (color from the packed phosphor LUT) ====
    signal b2_addr : unsigned(8 downto 0) := (others => '0');
    signal b2_a    : unsigned(5 downto 0) := (others => '0');
    signal b3_ylo, b3_yhi, b3_ulo, b3_uhi, b3_vlo, b3_vhi : unsigned(9 downto 0) := (others => '0');
    signal b3_hi   : std_logic := '0';
    signal b3_a    : unsigned(5 downto 0) := (others => '0');
    signal b5_y, b5_u, b5_v : unsigned(9 downto 0) := (others => '0');
    signal b5_a    : unsigned(5 downto 0) := (others => '0');
    signal b6_dy, b6_du, b6_dv : signed(11 downto 0) := (others => '0');
    signal b6_g    : unsigned(6 downto 0) := (others => '0');
    signal b7_py, b7_pu, b7_pv : signed(19 downto 0) := (others => '0');

    -- ==== sync / video delay (u carries Cr on this hardware; kept verbatim) ====
    type t_dsr is array(1 to C_LAT - 1) of unsigned(9 downto 0);
    type t_bsr is array(1 to C_LAT - 1) of std_logic;
    signal d_y, d_u, d_v : t_dsr := (others => (others => '0'));
    signal d_hs, d_vs, d_fl, d_av : t_bsr := (others => '0');
    signal d_gv : t_bsr := (others => '0');            -- generated avid
    signal s_bg5, s_bg7 : std_logic := '0';            -- video backdrop live

    signal s_io : t_video_stream_yuv444_30b
        := ((others => '0'), (others => '0'), (others => '0'), '0', '1', '1', '1');

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement + per-frame class constants
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_v <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                s_ilace <= s_timing.is_interlaced;
                s_botf  <= not s_timing.field_n;      -- bottom field = field_n '0'

                if s_timing.is_interlaced = '1' then
                    s_fh <= s_meas_v(10 downto 0) & '0';
                else
                    s_fh <= s_meas_v;
                end if;
                if s_meas_h >= to_unsigned(1600, 12) then       -- 1080p / 1080i
                    s_cols <= to_unsigned(1920, 11);
                    s_kwav <= to_unsigned(34, 12);
                    s_kw45 <= to_unsigned(153, 12);
                elsif s_meas_h >= to_unsigned(1000, 12) then    -- 720p
                    s_cols <= to_unsigned(1280, 11);
                    s_kwav <= to_unsigned(51, 12);
                    s_kw45 <= to_unsigned(230, 12);
                else                                            -- SD (NTSC / PAL)
                    s_cols <= to_unsigned(720, 11);
                    s_kwav <= to_unsigned(91, 12);
                    s_kw45 <= to_unsigned(410, 12);
                end if;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Per-field control latch, motion phases, free-running derived terms
    -- ====================================================================
    p_frame : process(clk)
        variable v_d   : signed(11 downto 0);
        variable v_a   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- once-per-field pulse: the first vsync edge after active video
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;

            if s_vs_go = '1' then
                g_wm1    <= s_meas_h - 1;          -- generated-avid width
                s_solid  <= registers_in(6)(0);
                s_bgvid  <= registers_in(6)(1);
                s_fill   <= registers_in(6)(2);
                s_along  <= registers_in(6)(3);
                s_mirror <= registers_in(6)(4);
                s_k3 <= unsigned(registers_in(2));
                s_k5 <= unsigned(registers_in(4));

                -- the wave travels (positive Speed = rightward); the noise
                -- lobes ride at 0.62x / -0.38x of it plus a slow drift of
                -- their own, so the shape keeps evolving even when still
                s_pht1 <= s_pht1 - unsigned(resize(s_vel, 16));
                s_pht2 <= s_pht2 + unsigned(resize(s_vel2, 16)) + 23;
                s_pht3 <= s_pht3 - unsigned(resize(s_vel3, 16)) + 17;
                s_phc  <= unsigned(signed(s_phc) + resize(s_flow, 16));
            end if;

            s_k1s  <= s_a1(13 downto 4);
            s_k2s  <= s_a2(13 downto 4);
            s_k4s  <= s_a4(13 downto 4);
            s_k6s  <= s_a6(13 downto 4);
            s_p12s <= s_a12(13 downto 4);
            s_nq   <= s_k4s(9 downto 4);

            -- rainbow cycles: 128 .. 1023 in units of 256 = 0.5 .. 4
            s_cyc <= to_unsigned(128, 11)
                     + resize(shift_right(resize(s_k6s * 7, 13), 3), 11);
            -- geometry from the measured raster, in quarter-rows
            -- centre on the TRUE middle of the raster, (fh-1)/2 rows, so
            -- the Mirror reflection axis and the wave's centre coincide
            s_Pq2  <= resize((s_fh - 1) & '0', 14);
            s_fhm4 <= resize((s_fh - 1) & "00", 14);
            s_Amin <= resize(shift_right(s_fh, 3), 12);           -- ~3% ripple
            s_fh2m <= resize(s_fh & '0', 13) - resize(s_Amin, 13);

            -- noise lobe speeds from the registered wave speed
            s_vel2 <= shift_right(s_vel, 1) + shift_right(s_vel, 3) - shift_right(s_vel, 7);
            s_vel3 <= shift_right(s_vel, 2) + shift_right(s_vel, 3) + shift_right(s_vel, 7);

            -- bipolar deadbands (+-48) for Speed and Flow; square law in
            -- the sequencer
            v_d := resize(signed('0' & s_k3), 12) - to_signed(512, 12);
            if v_d < 0 then v_a := resize(unsigned(-v_d), 11); s_sneg <= '1';
            else            v_a := resize(unsigned(v_d), 11);  s_sneg <= '0'; end if;
            if v_a > to_unsigned(48, 11) then s_sad <= resize(v_a - 48, 10);
            else                              s_sad <= (others => '0'); end if;
            v_d := resize(signed('0' & s_k5), 12) - to_signed(512, 12);
            if v_d < 0 then v_a := resize(unsigned(-v_d), 11); s_fneg <= '1';
            else            v_a := resize(unsigned(v_d), 11);  s_fneg <= '0'; end if;
            if v_a > to_unsigned(48, 11) then s_fad <= resize(v_a - 48, 10);
            else                              s_fad <= (others => '0'); end if;
        end if;
    end process p_frame;

    -- ====================================================================
    -- Per-frame parameter sequencer: 5 knob slews, 8 products on ONE shared
    -- shift-add multiplier (13 cycles each), combines, one divide.
    -- ~160 cycles, all inside the blanking interval.
    -- ====================================================================
    p_seq : process(clk)
        variable v_t   : unsigned(13 downto 0);
        variable v_d   : signed(14 downto 0);
        variable v_n   : signed(15 downto 0);
        variable v_dr  : unsigned(14 downto 0);
        variable v_h   : unsigned(13 downto 0);
        variable v_k   : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_go = '1' then
                s_op  <= 0;
                s_bit <= 0;
            elsif s_timing.avid = '0' then
                case s_op is
                    -- 1-pole slews (Q10.4): acc += (target - acc) / 16.
                    -- 16-bit intermediate, then SLICE (a signed resize here
                    -- mangles values >= 8192)
                    when 0 =>
                        v_t := unsigned(registers_in(0)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a1);
                        v_n := resize(signed('0' & s_a1), 16) + resize(shift_right(v_d, 4), 16);
                        s_a1 <= unsigned(v_n(13 downto 0));
                        s_op <= 1;
                    when 1 =>
                        v_t := unsigned(registers_in(1)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a2);
                        v_n := resize(signed('0' & s_a2), 16) + resize(shift_right(v_d, 4), 16);
                        s_a2 <= unsigned(v_n(13 downto 0));
                        s_op <= 2;
                    when 2 =>
                        v_t := unsigned(registers_in(3)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a4);
                        v_n := resize(signed('0' & s_a4), 16) + resize(shift_right(v_d, 4), 16);
                        s_a4 <= unsigned(v_n(13 downto 0));
                        s_op <= 3;
                    when 3 =>
                        v_t := unsigned(registers_in(5)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a6);
                        v_n := resize(signed('0' & s_a6), 16) + resize(shift_right(v_d, 4), 16);
                        s_a6 <= unsigned(v_n(13 downto 0));
                        s_op <= 4;
                    when 4 =>
                        v_t := unsigned(registers_in(7)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a12);
                        v_n := resize(signed('0' & s_a12), 16) + resize(shift_right(v_d, 4), 16);
                        s_a12 <= unsigned(v_n(13 downto 0));
                        s_op <= 5;

                    -- shared multiplier: mB is 11 bits, consumed at bits
                    -- 1..11; the store at bit 12 sees the complete sum
                    when 5 to 12 =>
                        if s_bit = 0 then
                            s_acc <= (others => '0');
                            case s_op is
                                when 5      => s_mA <= resize(s_fh2m, 24); s_mB <= resize(s_p12s, 11);
                                when 6      => s_mA <= resize(s_k1s, 24);  s_mB <= resize(s_k1s, 11);
                                when 7      => s_mA <= resize(s_Pq2, 24);  s_mB <= resize(s_hk, 11);
                                when 8      => s_mA <= resize(s_k2s, 24);  s_mB <= resize(s_k2s, 11);
                                when 9      => s_mA <= resize(s_kw45, 24); s_mB <= resize(s_wsq, 11);
                                when 10     => s_mA <= resize(s_sad, 24);  s_mB <= resize(s_sad, 11);
                                when 11     => s_mA <= resize(s_fad, 24);  s_mB <= resize(s_fad, 11);
                                when others => s_mA <= resize(s_cyc, 24);  s_mB <= resize(s_kwav, 11);
                            end case;
                            s_bit <= 1;
                        else
                            if s_mB(0) = '1' then
                                s_acc <= s_acc + s_mA;
                            end if;
                            s_mA <= s_mA(22 downto 0) & '0';
                            s_mB <= '0' & s_mB(10 downto 1);
                            if s_bit = 12 then
                                s_bit <= 0;
                                s_op  <= s_op + 1;
                                case s_op is
                                    -- amplitude: ripple floor + Swell x headroom
                                    when 5      => s_A  <= resize(s_Amin, 13) + resize(s_acc(23 downto 10), 13);
                                    -- height squared, so the thin end has resolution
                                    when 6      => s_hk <= s_acc(19 downto 10);
                                    -- half-thickness = half the screen x height^2
                                    when 7      => if s_acc(23 downto 10) < 8 then
                                                       s_Hq2 <= to_unsigned(8, 13);
                                                   else
                                                       s_Hq2 <= resize(s_acc(23 downto 10), 13);
                                                   end if;
                                    when 8      => s_wsq <= s_acc(19 downto 10);
                                    -- waves across the screen: 0.5 + 4.5 x waves^2
                                    when 9      => s_k1add <= resize(s_acc(23 downto 10), 12);
                                    when 10     => if s_sneg = '1' then
                                                       s_vel <= -signed('0' & s_acc(18 downto 8));
                                                   else
                                                       s_vel <=  signed('0' & s_acc(18 downto 8));
                                                   end if;
                                    when 11     => if s_fneg = '1' then
                                                       s_flow <= -signed('0' & s_acc(19 downto 9));
                                                   else
                                                       s_flow <=  signed('0' & s_acc(19 downto 9));
                                                   end if;
                                    -- Along: hue step per column = cycles x 65536 / cols
                                    when others => s_kx <= resize(s_acc(19 downto 8), 12);
                                end case;
                            else
                                s_bit <= s_bit + 1;
                            end if;
                        end if;

                    when 13 =>
                        v_k := resize(shift_right(s_kwav, 1), 13) + resize(s_k1add, 13);
                        s_k1inc <= resize(v_k, 12);
                        s_H31   <= s_Hq2 + 31;
                        s_op <= 14;
                    when 14 =>
                        -- noise lobes at 1.61x and 0.38x the wavenumber
                        v_k := resize(s_k1inc, 13) + resize(shift_right(s_k1inc, 1), 13)
                               + resize(shift_right(s_k1inc, 3), 13)
                               - resize(shift_right(s_k1inc, 6), 13);
                        s_k2inc <= resize(v_k, 12);
                        s_k3inc <= resize(shift_right(s_k1inc, 2), 12)
                                   + resize(shift_right(s_k1inc, 3), 12)
                                   + resize(shift_right(s_k1inc, 7), 12);
                        s_op <= 21;

                    -- hue across the band: hscale = cyc*112 / Hq2, so at
                    -- Spectrum 100% (cyc 256) the solid core spans 0.875 of
                    -- the hue cycle = red .. violet, WITHOUT the violet->red
                    -- wrap (a full cycle showed pink/red again at the bottom)
                    when 21 =>
                        s_dn  <= resize(s_cyc & "0000000", 24)
                                 - resize(s_cyc & "0000", 24);
                        s_dvsr <= resize(s_Hq2, 15);
                        s_dr  <= (others => '0');
                        s_dq  <= (others => '0');
                        s_bit <= 0;
                        s_op  <= 22;
                    when 22 | 25 =>
                        v_dr := s_dr(13 downto 0) & s_dn(23);
                        s_dn <= s_dn(22 downto 0) & '0';
                        if v_dr >= s_dvsr then
                            s_dr <= v_dr - s_dvsr;
                            s_dq <= s_dq(22 downto 0) & '1';
                        else
                            s_dr <= v_dr;
                            s_dq <= s_dq(22 downto 0) & '0';
                        end if;
                        if s_bit = 23 then
                            s_bit <= 0;
                            if s_op = 22 then s_op <= 23;
                            else              s_op <= 26; end if;
                        else
                            s_bit <= s_bit + 1;
                        end if;
                    when 23 =>
                        if s_dq > to_unsigned(4095, 24) then
                            s_hscale <= to_unsigned(4095, 12);
                        else
                            s_hscale <= resize(s_dq, 12);
                        end if;
                        s_op <= 24;
                    -- feather scale: vs = 1080*256 / fh (same divider)
                    when 24 =>
                        s_dn   <= to_unsigned(276480, 24);
                        s_dvsr <= resize(s_fh, 15);
                        s_dr   <= (others => '0');
                        s_dq   <= (others => '0');
                        s_bit  <= 0;
                        s_op   <= 25;
                    when 26 =>
                        if s_dq > to_unsigned(4095, 24) then
                            s_vs <= to_unsigned(4095, 12);
                        elsif s_dq < to_unsigned(256, 24) then
                            s_vs <= to_unsigned(256, 12);
                        else
                            s_vs <= resize(s_dq, 12);
                        end if;
                        s_op <= 31;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Scanner: trigger + curve pipeline + stamp writer.  Exactly one column
    -- per clock, every column written once.
    -- ====================================================================
    p_march : process(clk)
        variable v_ph   : unsigned(9 downto 0);
        variable v_s0, v_s1 : signed(12 downto 0);
        variable v_yn   : signed(15 downto 0);
        variable v_d1, v_d2 : signed(15 downto 0);
        variable v_he   : signed(15 downto 0);
        variable v_al   : signed(13 downto 0);
        variable v_hpp  : signed(26 downto 0);
        variable v_h16  : unsigned(15 downto 0);
        variable v_c1, v_c2 : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_st_we <= '0';

            -- registered next-row helpers (fed by a stable s_vcnt)
            s_nrow <= s_vcnt + 1;
            if (s_vcnt + 1) < s_meas_v then s_trig_ok <= '1';
            else                            s_trig_ok <= '0'; end if;
            if s_vcnt >= s_meas_v then s_vb_ok <= '1';
            else                       s_vb_ok <= '0'; end if;
            if s_ilace = '1' then
                if s_botf = '1' then
                    s_rq2n <= resize(s_nrow(10 downto 0) & "000", 14) + 4;
                else
                    s_rq2n <= resize(s_nrow(10 downto 0) & "000", 14);
                end if;
            else
                s_rq2n <= resize(s_nrow(11 downto 0) & "00", 14);
            end if;
            -- the row reflected about the centre line: (fh-1) - row, in
            -- quarter-rows (row r <-> row fh-1-r exactly)
            s_R2q2 <= s_fhm4 - s_Rq2;

            -- ---- triggers ----
            if s_run = '0' and s_timing.avid_start = '1' and s_trig_ok = '1' then
                s_Rq2  <= s_rq2n;
                s_run  <= '1';
                s_col  <= (others => '0');
                s_wcol <= (others => '0');
                s_hxw  <= (others => '0');
                s_th1  <= s_pht1;
                s_th2  <= s_pht2;
                s_th3  <= s_pht3;
                s_v    <= (others => '0');
            elsif s_run = '0' and s_timing.hsync_start = '1'
                  and s_vb_ok = '1' then
                if s_ilace = '1' and s_botf = '1' then
                    s_Rq2 <= to_unsigned(4, 14);
                else
                    s_Rq2 <= (others => '0');
                end if;
                s_run  <= '1';
                s_col  <= (others => '0');
                s_wcol <= (others => '0');
                s_hxw  <= (others => '0');
                s_th1  <= s_pht1;
                s_th2  <= s_pht2;
                s_th3  <= s_pht3;
                s_v    <= (others => '0');
            elsif s_run = '1' then
                s_col <= s_col + 1;
                s_th1 <= s_th1 + resize(s_k1inc, 16);
                s_th2 <= s_th2 + resize(s_k2inc, 16);
                s_th3 <= s_th3 + resize(s_k3inc, 16);
                s_v(1) <= '1';
                s_v(2 to 15) <= s_v(1 to 14);
                if s_col >= resize(s_cols, 12) + 20 then
                    s_run <= '0';
                    s_v   <= (others => '0');
                end if;
            end if;

            -- ---- wave pipeline ----
            if s_run = '1' then
                -- m1: quarter-wave fold, idx and idx+1 for each lobe
                v_ph := s_th1(15 downto 6);
                m1_s1a <= v_ph(9);
                if v_ph(8) = '1' then m1_i1a <= not v_ph(7 downto 0);
                else                  m1_i1a <= v_ph(7 downto 0); end if;
                v_ph := s_th1(15 downto 6) + 1;
                m1_s1b <= v_ph(9);
                if v_ph(8) = '1' then m1_i1b <= not v_ph(7 downto 0);
                else                  m1_i1b <= v_ph(7 downto 0); end if;
                m1_f1 <= s_th1(5 downto 0);

                v_ph := s_th2(15 downto 6);
                m1_s2a <= v_ph(9);
                if v_ph(8) = '1' then m1_i2a <= not v_ph(7 downto 0);
                else                  m1_i2a <= v_ph(7 downto 0); end if;
                v_ph := s_th2(15 downto 6) + 1;
                m1_s2b <= v_ph(9);
                if v_ph(8) = '1' then m1_i2b <= not v_ph(7 downto 0);
                else                  m1_i2b <= v_ph(7 downto 0); end if;
                m1_f2 <= s_th2(5 downto 0);

                v_ph := s_th3(15 downto 6);
                m1_s3a <= v_ph(9);
                if v_ph(8) = '1' then m1_i3a <= not v_ph(7 downto 0);
                else                  m1_i3a <= v_ph(7 downto 0); end if;
                v_ph := s_th3(15 downto 6) + 1;
                m1_s3b <= v_ph(9);
                if v_ph(8) = '1' then m1_i3b <= not v_ph(7 downto 0);
                else                  m1_i3b <= v_ph(7 downto 0); end if;
                m1_f3 <= s_th3(5 downto 0);

                -- m2: ROM data settles in the p_rom* processes
                m2_s1a <= m1_s1a;  m2_s1b <= m1_s1b;  m2_f1 <= m1_f1;
                m2_s2a <= m1_s2a;  m2_s2b <= m1_s2b;  m2_f2 <= m1_f2;
                m2_s3a <= m1_s3a;  m2_s3b <= m1_s3b;  m2_f3 <= m1_f3;

                -- m2b: signs + neighbour deltas (EBR data into fabric FFs
                -- before any arithmetic)
                if m2_s1a = '1' then v_s0 := -signed('0' & m2_q1a);
                else                 v_s0 :=  signed('0' & m2_q1a); end if;
                if m2_s1b = '1' then v_s1 := -signed('0' & m2_q1b);
                else                 v_s1 :=  signed('0' & m2_q1b); end if;
                m2b_v1 <= v_s0;
                m2b_d1 <= resize(v_s1 - v_s0, 8);
                m2b_f1 <= m2_f1;
                if m2_s2a = '1' then v_s0 := -signed('0' & m2_q2a);
                else                 v_s0 :=  signed('0' & m2_q2a); end if;
                if m2_s2b = '1' then v_s1 := -signed('0' & m2_q2b);
                else                 v_s1 :=  signed('0' & m2_q2b); end if;
                m2b_v2 <= v_s0;
                m2b_d2 <= resize(v_s1 - v_s0, 8);
                m2b_f2 <= m2_f2;
                if m2_s3a = '1' then v_s0 := -signed('0' & m2_q3a);
                else                 v_s0 :=  signed('0' & m2_q3a); end if;
                if m2_s3b = '1' then v_s1 := -signed('0' & m2_q3b);
                else                 v_s1 :=  signed('0' & m2_q3b); end if;
                m2b_v3 <= v_s0;
                m2b_d3 <= resize(v_s1 - v_s0, 8);
                m2b_f3 <= m2_f3;

                -- m3: interpolate all three lobes
                m3_s1 <= m2b_v1 + resize(shift_right(m2b_d1 * signed('0' & m2b_f1), 6), 13);
                m3_s2 <= m2b_v2 + resize(shift_right(m2b_d2 * signed('0' & m2b_f2), 6), 13);
                m3_s3 <= m2b_v3 + resize(shift_right(m2b_d3 * signed('0' & m2b_f3), 6), 13);

                -- m3b: wave lobe, summed noise lobes
                m3b_sf <= m3_s1;
                m3b_ns <= resize(m3_s2, 14) + resize(m3_s3, 14);

                -- m4a: noise x level (Q6; the result is Q12 after >> 7, so
                -- full Noise puts each lobe at ~half the wave's amplitude)
                m4a_sf <= m3b_sf;
                m4a_nz <= m3b_ns * signed('0' & s_nq);

                -- m4s: wave + noise; half-thickness x noise (the band
                -- breathes with the same noise that bends it)
                m4s_sum <= resize(m4a_sf, 14) + resize(m4a_nz(20 downto 7), 14);
                m4s_hb  <= signed('0' & s_Hq2) * m4a_nz(20 downto 13);

                -- m4b: amplitude x sum, split hi/lo; H_eff = H + H*noise/2
                m4_hi <= signed('0' & s_A) * m4s_sum(13 downto 7);
                m4_lo <= signed('0' & s_A) * signed('0' & m4s_sum(6 downto 0));
                v_he  := resize(signed('0' & s_H31), 16) + resize(m4s_hb(21 downto 7), 16);
                m4_h  <= unsigned(v_he(12 downto 0));

                -- m4c: recombine
                m4_p1 <= shift_left(resize(m4_hi, 28), 7) + resize(m4_lo, 28);
                m4c_h <= m4_h;

                -- m5: the wave's centre height at this column (q2 rows)
                v_yn := signed('0' & resize(s_Pq2, 15))
                        + resize(shift_right(m4_p1, 12), 16);
                m5_y <= v_yn;
                m5_h <= m4c_h;

                -- m6: vertical distance to this row, and to the row
                -- reflected about the centre line.  The reflected copy's
                -- colour offset is the REFLECTED row's own offset (not
                -- negated), so it is a true mirror image: on the centre
                -- line both copies are identical and the fold is seamless.
                v_d1 := m5_y - signed('0' & resize(s_Rq2, 15));
                v_d2 := m5_y - signed('0' & resize(s_R2q2, 15));
                m6_d1 <= v_d1;
                m6_n2 <= v_d2;
                if v_d1 < 0 then m6_a1 <= resize(unsigned(-v_d1), 15);
                else             m6_a1 <= resize(unsigned(v_d1), 15); end if;
                if v_d2 < 0 then m6_a2 <= resize(unsigned(-v_d2), 15);
                else             m6_a2 <= resize(unsigned(v_d2), 15); end if;
                m6_h <= m5_h;

                -- m6b: the nearer copy wins (clamped distances)
                if m6_a1 > 8191 then v_c1 := to_unsigned(8191, 13);
                else                 v_c1 := resize(m6_a1, 13); end if;
                if m6_a2 > 8191 then v_c2 := to_unsigned(8191, 13);
                else                 v_c2 := resize(m6_a2, 13); end if;
                if s_mirror = '1' and m6_a2 < m6_a1 then
                    m6b_dp <= v_c2;
                    m6b_po <= m6_n2;
                else
                    m6b_dp <= v_c1;
                    m6b_po <= m6_d1;
                end if;
                m6b_h <= m6_h;

                -- m6c: saturate the signed offset for the hue multiply
                if m6b_po > 8191 then
                    m6c_po <= to_signed(8191, 14);
                elsif m6b_po < -8191 then
                    m6c_po <= to_signed(-8191, 14);
                else
                    m6c_po <= m6b_po(13 downto 0);
                end if;
                -- feather depth from the breathed half-thickness: u = how
                -- far outside the solid core (0 = on its edge, 31 = past
                -- the 1080p feather), in = inside the core
                v_al := signed('0' & m6b_h) - signed('0' & m6b_dp);
                if v_al > 31 then m6c_in <= '1';
                else              m6c_in <= '0'; end if;
                if v_al <= 0 then
                    m6c_u <= to_unsigned(31, 5);
                else
                    m6c_u <= to_unsigned(31, 5) - unsigned(v_al(4 downto 0));
                end if;

                -- m9: depth x (1080 / fh), so the feather is the same
                -- fraction of the screen height on every raster (x1 at
                -- 1080 rows: alpha = H31 - |d| exactly as before); hue
                -- partials
                m9_t  <= m6c_u * s_vs;
                m9_in <= m6c_in;
                m9_hhi <= m6c_po * signed('0' & s_hscale(11 downto 6));
                m9_hlo <= m6c_po * signed('0' & s_hscale(5 downto 0));

                -- m10: recombine the hue product
                v_hpp := shift_left(resize(m9_hhi, 27), 6) + resize(m9_hlo, 27);
                m10_hp <= unsigned(v_hpp(15 downto 0));
                if m9_in = '1' then
                    m10_a <= to_unsigned(63, 6);
                elsif m9_t(16 downto 8) >= 31 then
                    m10_a <= (others => '0');
                else
                    m10_a <= (to_unsigned(31, 5) - m9_t(12 downto 8)) & '0';
                end if;
            end if;

            -- ---- writer: one write per column, in order ----
            -- Across: hue rises downward from the colour phase (R..V top
            -- to bottom).  Along: hue advances with the column instead.
            if s_along = '1' then
                v_h16 := s_phc + s_hxw;
            else
                v_h16 := s_phc - m10_hp;
            end if;
            if s_run = '1' and s_v(15) = '1' and s_wcol < s_cols then
                s_st_we   <= '1';
                s_st_addr <= s_wcol(10 downto 0);
                s_st_data <= std_logic_vector(v_h16(15 downto 6))
                             & std_logic_vector(m10_a);
                s_wcol <= s_wcol + 1;
                s_hxw  <= s_hxw + resize(s_kx, 16);
            end if;
        end if;
    end process p_march;

    -- ==== sine ROMs: 3 lobes x (idx, idx+1) -- one inferred EBR each ====
    p_rom1a : process(clk) begin if rising_edge(clk) then m2_q1a <= C_QSIN13(to_integer(m1_i1a)); end if; end process;
    p_rom1b : process(clk) begin if rising_edge(clk) then m2_q1b <= C_QSIN13(to_integer(m1_i1b)); end if; end process;
    p_rom2a : process(clk) begin if rising_edge(clk) then m2_q2a <= C_QSIN13(to_integer(m1_i2a)); end if; end process;
    p_rom2b : process(clk) begin if rising_edge(clk) then m2_q2b <= C_QSIN13(to_integer(m1_i2b)); end if; end process;
    p_rom3a : process(clk) begin if rising_edge(clk) then m2_q3a <= C_QSIN13(to_integer(m1_i3a)); end if; end process;
    p_rom3b : process(clk) begin if rising_edge(clk) then m2_q3b <= C_QSIN13(to_integer(m1_i3b)); end if; end process;

    -- ==== ribbon colour LUT: six 256 x 10 ROMs, one inferred EBR each ====
    p_cylo : process(clk) begin if rising_edge(clk) then b3_ylo <= C_CY_LO(to_integer(b2_addr(7 downto 0))); end if; end process;
    p_cyhi : process(clk) begin if rising_edge(clk) then b3_yhi <= C_CY_HI(to_integer(b2_addr(7 downto 0))); end if; end process;
    p_culo : process(clk) begin if rising_edge(clk) then b3_ulo <= C_CU_LO(to_integer(b2_addr(7 downto 0))); end if; end process;
    p_cuhi : process(clk) begin if rising_edge(clk) then b3_uhi <= C_CU_HI(to_integer(b2_addr(7 downto 0))); end if; end process;
    p_cvlo : process(clk) begin if rising_edge(clk) then b3_vlo <= C_CV_LO(to_integer(b2_addr(7 downto 0))); end if; end process;
    p_cvhi : process(clk) begin if rising_edge(clk) then b3_vhi <= C_CV_HI(to_integer(b2_addr(7 downto 0))); end if; end process;

    -- ====================================================================
    -- Line buffer: ONE bank.  Column c of row R+1 is stamped ~16 clocks
    -- AFTER the beam read column c of row R, so every read precedes its own
    -- overwrite.  Every column is stamped (alpha 0 outside the band), so
    -- there is nothing to clear: one write statement, one read.
    -- ====================================================================
    s_raddr <= s_bcol;

    -- ====================================================================
    -- Clean avid (see header).  g_ph is the hsync->avid-rise distance
    -- parity of this line; g_par is the voted usual parity; a line whose
    -- avid lands off-parity starts one clock later, so the generated start
    -- always sits at the same hsync parity.  It ends after the width
    -- latched per field, or at hsync at the latest.
    -- ====================================================================
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
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            -- beam column: counts the generated avid from hsync
            if g_hsr = '1' then
                s_bcol <= (others => '0');
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
            end if;
        end if;
    end process p_gavid;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_rar <= s_raddr;
            if s_st_we = '1' then
                s_lb(to_integer(s_st_addr)) <= s_st_data;
            end if;
            s_rd <= s_lb(to_integer(s_rar));
        end if;
    end process p_lb;

    -- ====================================================================
    -- Beam: readout, hue -> YUV, alpha composite (C_LAT = 8, all modes)
    -- ====================================================================
    p_beam : process(clk)
        variable v_w    : std_logic_vector(15 downto 0);
        variable v_hue  : unsigned(9 downto 0);
        variable v_m6   : unsigned(12 downto 0);
        variable v_k    : unsigned(2 downto 0);
        variable v_yr, v_ur, v_vr : unsigned(9 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_g    : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- b2: decode word -> color LUT address.  Gradient reads the hue
            -- directly; Solid snaps to the segment start (k*73 = k*64+k*8+k),
            -- where the LUT holds the pure phosphor color.
            v_w := s_rd;
            b2_a <= unsigned(v_w(5 downto 0));
            v_hue := unsigned(v_w(15 downto 6));
            v_m6  := resize(v_hue & "00", 13) + resize(v_hue & '0', 13)
                     + resize(v_hue, 13);                    -- hue10 * 7
            if s_solid = '1' then
                v_k := v_m6(12 downto 10);
                b2_addr <= resize(v_k & "000000", 9) + resize(v_k & "000", 9)
                           + resize(v_k, 9);
            else
                b2_addr <= v_hue(9 downto 1);
            end if;

            -- b3: the six ROM reads happen in p_c*; carry the half select
            b3_hi <= b2_addr(8);
            b3_a  <= b2_a;

            -- b4 (named b5_*): pick the half, into fabric FFs before any
            -- arithmetic (one 2:1 mux after the EBR output)
            if b3_hi = '1' then
                b5_y <= b3_yhi;  b5_u <= b3_uhi;  b5_v <= b3_vhi;
            else
                b5_y <= b3_ylo;  b5_u <= b3_ulo;  b5_v <= b3_vlo;
            end if;
            b5_a  <= b3_a;

            -- b6: background select, differences, gain.  Black is Y 64
            -- (code 0 is super-black).  Fill = Video: the band's luma is
            -- the live picture's, half-mixed with the rainbow's so dark
            -- video still shows the band; chroma stays the rainbow's.
            if s_fill = '1' and d_av(5) = '1' then
                v_yr := resize(shift_right(resize(b5_y, 11) + resize(d_y(5), 11), 1), 10);
            else
                v_yr := b5_y;
            end if;
            v_ur := b5_u;  v_vr := b5_v;
            if s_bg5 = '1' then
                v_by := d_y(5);
                v_bu := d_u(5);
                v_bv := d_v(5);
            else
                v_by := C_BLK;
                v_bu := C_MID;
                v_bv := C_MID;
            end if;
            if d_gv(5) = '0' then
                v_g := (others => '0');
            elsif b5_a = to_unsigned(63, 6) then
                v_g := to_unsigned(64, 7);
            else
                v_g := resize(b5_a, 7);
            end if;
            b6_dy <= resize(signed('0' & v_yr), 12) - resize(signed('0' & v_by), 12);
            b6_du <= resize(signed('0' & v_ur), 12) - resize(signed('0' & v_bu), 12);
            b6_dv <= resize(signed('0' & v_vr), 12) - resize(signed('0' & v_bv), 12);
            b6_g  <= v_g;

            -- b7: alpha products
            b7_py <= b6_dy * signed('0' & b6_g);
            b7_pu <= b6_du * signed('0' & b6_g);
            b7_pv <= b6_dv * signed('0' & b6_g);

            -- b8 -> s_io: composite over the SAME background sample, re-tapped
            -- from the delay line two stages later (convex blend: no clamp)
            if s_bg7 = '1' then
                v_by := d_y(7);
                v_bu := d_u(7);
                v_bv := d_v(7);
            else
                v_by := C_BLK;
                v_bu := C_MID;
                v_bv := C_MID;
            end if;
            s_io.y <= std_logic_vector(resize(unsigned(
                signed('0' & v_by) + resize(shift_right(b7_py, 6), 11)), 10));
            s_io.u <= std_logic_vector(resize(unsigned(
                signed('0' & v_bu) + resize(shift_right(b7_pu, 6), 11)), 10));
            s_io.v <= std_logic_vector(resize(unsigned(
                signed('0' & v_bv) + resize(shift_right(b7_pv, 6), 11)), 10));
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_gv(C_LAT - 1);
            -- backdrop video only where BOTH avids are live: outside the
            -- generated avid the gain is 0 and the background is black, so
            -- the output is exactly neutral (blanking gate), and the few
            -- columns where the generated line outruns the source are black
            -- rather than the receiver's blanking codes
            s_bg5 <= s_bgvid and d_av(4) and d_gv(4);
            s_bg7 <= s_bgvid and d_av(6) and d_gv(6);
        end if;
    end process p_beam;

    -- ====================================================================
    -- Sync / video delay
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_y(1) <= unsigned(data_in.y);
            d_u(1) <= unsigned(data_in.u);
            d_v(1) <= unsigned(data_in.v);
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_av(1) <= data_in.avid;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_y(i)  <= d_y(i - 1);
                d_u(i)  <= d_u(i - 1);
                d_v(i)  <= d_v(i - 1);
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_av(i) <= d_av(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture aurora;
