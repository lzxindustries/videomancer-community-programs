-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Lagoon: underwater look filter.
--
-- Incoming video is written into a dual-bank line buffer (from sidewinder)
-- and read back one line later at a horizontally displaced address, then
-- graded blue-green with dancing caustic highlights:
--
--   offset(x,y) = sin(swell) * Waves                     (big per-line swell)
--               + [tri(r1) + tri(r2)/2] * Ripple          (fine 2D interference)
--   caustic     = tri(c1 + 32*tri(r1)) + tri(c2 - 32*tri(r2)) + dither,
--                 with each line's caustic phase seeded +base_off*fc so the
--                 light field SWAYS WITH the swell-displaced picture;
--                 highlight where it tops a K6 threshold. The ripple waves
--                 PHASE-MODULATE the caustic waves, bending the interference
--                 lattice into writhing curves, and a small +-16 hash dither
--                 roughens the contour edges -- organic instead of a rigid
--                 diamond grid. The dither uses x bits >= 1 and a per-LINE
--                 LFSR (no pixel/line LSBs: those made a checkerboard).
--   grade       = chroma lerp toward aqua/abyss target, depth-gradient murk
--
-- Controls:
--   K1 Ripple    fine refraction amplitude (two incommensurate triangle
--                waves at ~1.6x frequency ratio -> netting interference)
--   K2 Waves     per-line sine swell amplitude, 0..255 px (at 1080p)
--   K3 Speed     bidirectional, centre detent = frozen; quadratic curve
--   K4 Scale     spatial frequency of ripple/caustics/swell (quadratic)
--   K5 Tint      chroma lerp amount toward the S7 target colour
--   K6 Caustics  highlight amount (lowers the interference threshold --
--                no per-pixel gain multiply needed)
--   S7 Hue       Aqua / Abyss tint target
--   S8 Glitter   blown-white twinkling cores on caustic peaks (frame LFSR
--                gates 1/4 of cores per frame)
--   S9 Murk      depth gradient: Y darkens toward screen bottom (max -288),
--                per-line accumulator, resolution-adaptive step
--   S10 Sea      Calm / Storm: swell 3rd harmonic + 1.5x amplitude + 2x rate
--   S11 Prism    chromatic refraction: U and V read the line buffer at
--                their own displaced addresses, split to either side of
--                luma (+-1/2 swell, +-~1.2% of the line width fixed; both
--                follow luma's ripple) -- broad colour fringes that sway
--                with the waves.  Tint is halved while Prism is on so the
--                fringes keep the source colours.  (v1.4.2: v1.4.0's
--                ripple split + 4 px read as a faint smear on HW.)
--                (Replaces v1.3's Bypass, which duplicated Depth = 0.)
--   P12 Depth    master dry/wet -- the headline fader
--
-- Resolution scaling (v1.4.3): ripple, caustics and swell are drawn on a
-- virtual 1080-line canvas, so SD (a 1440-unit 4:3 window) and 720p show
-- the 1080p water at the same size relative to screen height: per-pixel
-- frequencies x xstep (timing ID: 1.0 / 1.5 / 2.0), per-line phases advance
-- once per virtual line crossed (Q4.8 y step per field line, x2 for
-- interlace), ripple/swell/storm/prism amplitudes in pixels / xstep.  1080p
-- is bit-identical to v1.4.2.  Dither, left-fill and murk stay pixel/line
-- based.
--
-- Colour convention: tint targets are stored U/V (Cb/Cr) SWAPPED to match the
-- Videomancer output convention (matching the HW-verified mondrian/prism/CGA
-- palettes). Pre-swap BT.601: Aqua Cb=600 Cr=390, Abyss Cb=690 Cr=465. NOTE:
-- the simulator renders standard BT.601, so tints look warm/olive in sim.
--
-- Implementation notes:
--  * All knob/switch registers are pre-registered every clock (SPI-reg
--    lesson) before any arithmetic sees them.
--  * Wave setup runs in step sequencers during blanking (sidewinder
--    pattern): per-frame = rate/scale curves + temporal phase advance;
--    per-line = swell sine + harmonic multiplies + accumulator seeding.
--  * Per-pixel work is 4 phase accumulators, combinational triangle folds,
--    ONE registered multiply (ripple amplitude), an add/clamp for the read
--    address, and a threshold subtract for the caustic highlight.
--  * Read address clamps to [1, width] (addr 0 is never written -- the
--    write column counter is one ahead of the data register). LEFT-edge
--    overshoot does not smear the border pixel: each line's average colour
--    (first 64 active pixels, sum >> 6) is latched at hsync and painted
--    into the uncovered strip instead, so the reveal reads as water-toned
--    haze rather than black; caustics/murk/tint still apply on top.
--    Right-edge overshoot keeps the border-pixel smear.
--
-- Pipeline (rising-edge clocks, data_in -> data_out), latency 13:
--   E1   p_ctrl: register inputs/knobs, pixel_x + 4 phase accumulators,
--        parity, line + frame step sequencers
--   E2   ripple triangle folds registered; caustic accumulators delayed;
--        live pixel write lands in its bank
--   E3   ripple sum; caustic folds (phase-modulated by the ripple folds)
--   E4   ripple multiply; base read positions (Y, Prism U/V); caustic
--        sum + hash dither
--   E5   read address add + clamp (registered; Y, U, V); highlight threshold
--   E6   bank reads registered; Y-add (caustic + glitter)
--   E7   bank select -> wet pixel
--   E8   grade: Y + caustic - murk (saturating)
--   E9   tint: U/V shift-lerp toward target, 16-level knob (no multiply
--        wider than 11x5 -- convex, so no clamp needed)
--   E10-13 depth-mix interpolators (dry tap = shift register index 8)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lagoon of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (13 + 3 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_Y_AW          : integer := 11;                  -- 2048 (full)
    constant C_UV_AW         : integer := 10;                  -- 1024 (2:1 dec)
    constant C_TOTAL_LATENCY : integer := 13;
    constant C_DRY_TAP       : integer := 8;   -- dry data tap for the mix

    -- Left fill takes over for reads of source columns < C_LEDGE, not just
    -- off-screen reads: capture sources carry a few black blanking-edge
    -- columns, which otherwise show as a black seam between the fill and
    -- the picture. The average window starts past the edge for the same
    -- reason.
    constant C_LEDGE : integer := 6;


    -- Tint targets, U/V swapped for hardware (see header).
    constant C_AQUA_U  : unsigned(C_DW - 1 downto 0) := to_unsigned(390, C_DW);
    constant C_AQUA_V  : unsigned(C_DW - 1 downto 0) := to_unsigned(600, C_DW);
    constant C_ABYSS_U : unsigned(C_DW - 1 downto 0) := to_unsigned(465, C_DW);
    constant C_ABYSS_V : unsigned(C_DW - 1 downto 0) := to_unsigned(690, C_DW);

    --------------------------------------------------------------------------
    -- Sine ROM: 256 x signed, value = round(-255*cos(2*pi*i/256)) so index 0
    -- is the wave minimum, matching the triangle fold below.
    --------------------------------------------------------------------------
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
    -- Resolution scaling (v1.4.3): the water is designed on a virtual
    -- 1080-line canvas.  Per-pixel wave frequencies are scaled by the x step
    -- (virtual units per pixel: 1080 1.0, 720p 1.5, SD 2.0 -- SD shows a
    -- 1440-unit 4:3 window), per-line frequencies by the y step (units per
    -- FIELD line, Q4.8, doubled for interlace: the line phases advance once
    -- per virtual line crossed), and pixel amplitudes (ripple, swell, storm
    -- chop, prism split) by 1/xstep.  At 1080p every step is exactly 1 and
    -- the picture is bit-identical to v1.4.2.
    --------------------------------------------------------------------------
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

    -- Per-pixel frequency x xstep: + v (SD, x2), + v/2 (720p, x1.5), + 0.
    function f_xsc(v : unsigned; sd, h7 : std_logic) return unsigned is
        variable r : unsigned(15 downto 0);
    begin
        r := resize(v, 16);
        if sd = '1' then
            return r + r;
        elsif h7 = '1' then
            return r + shift_right(r, 1);
        else
            return r;
        end if;
    end function;

    -- Triangle fold of a 16-bit phase to -256..255 (min at phase 0).
    function f_tri(phase : unsigned(15 downto 0)) return signed is
        variable v_ramp : signed(9 downto 0);
    begin
        v_ramp := signed(resize(phase(14 downto 6), 10));  -- 0..511
        if phase(15) = '0' then
            return v_ramp - to_signed(256, 10);            -- -256 .. 255
        else
            return to_signed(255, 10) - v_ramp;            --  255 .. -256
        end if;
    end function f_tri;

    --------------------------------------------------------------------------
    -- Inlined dual-bank line buffers
    --------------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(C_DW - 1 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(C_DW - 1 downto 0);

    signal lbY0, lbY1 : t_lb_y  := (others => (others => '0'));
    signal lbU0, lbU1 : t_lb_uv := (others => (others => '0'));
    signal lbV0, lbV1 : t_lb_uv := (others => (others => '0'));

    signal s_rdY0, s_rdY1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pre-registered controls (E1)
    --------------------------------------------------------------------------
    signal s_rip_k   : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_wave_k  : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_rate_k  : unsigned(C_DW - 1 downto 0) := to_unsigned(512, C_DW);
    signal s_scale_k : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_tint_k  : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_caus_k  : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_hue     : std_logic := '0';
    signal s_glitter : std_logic := '0';
    signal s_murk_en : std_logic := '0';
    signal s_storm   : std_logic := '0';
    signal s_prism   : std_logic := '0';
    signal s_tgt_u   : unsigned(C_DW - 1 downto 0) := C_AQUA_U;
    signal s_tgt_v   : unsigned(C_DW - 1 downto 0) := C_AQUA_V;

    --------------------------------------------------------------------------
    -- Input registers / position tracking (E1)
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_in_avid              : std_logic := '0';

    signal s_pixel_x      : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_par          : std_logic := '0';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';

    signal s_line_width   : unsigned(C_Y_AW - 1 downto 0) := (others => '0');

    -- Per-pixel phase accumulators (seeded per line, +f per active pixel)
    signal s_acc_r1, s_acc_r2 : unsigned(15 downto 0) := (others => '0');
    signal s_acc_c1, s_acc_c2 : unsigned(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-frame step sequencer (vertical blanking)
    --------------------------------------------------------------------------
    signal s_vstep       : unsigned(3 downto 0) := "1111";
    signal s_d           : signed(10 downto 0) := (others => '0');
    signal s_absd        : unsigned(9 downto 0) := (others => '0');
    signal s_rate_mag    : unsigned(12 downto 0) := (others => '0');
    signal s_scale_v     : unsigned(12 downto 0) := (others => '0');
    signal s_rate        : signed(13 downto 0) := (others => '0');
    signal s_rate_eff    : signed(14 downto 0) := (others => '0');
    signal s_swamp       : unsigned(9 downto 0) := (others => '0');
    signal s_fx1         : unsigned(12 downto 0) := to_unsigned(700, 13);
    signal s_fx2         : unsigned(13 downto 0) := to_unsigned(1138, 14);
    signal s_fc1         : unsigned(11 downto 0) := to_unsigned(300, 12);
    signal s_fc2         : unsigned(12 downto 0) := to_unsigned(404, 13);
    signal s_fsw         : unsigned(9 downto 0) := to_unsigned(60, 10);
    signal s_fy1         : unsigned(10 downto 0) := (others => '0');
    signal s_fy2         : unsigned(12 downto 0) := (others => '0');
    signal s_fyc1        : unsigned(11 downto 0) := (others => '0');
    signal s_fyc2        : unsigned(11 downto 0) := (others => '0');
    signal s_thr         : signed(10 downto 0) := to_signed(528, 11);
    signal s_lfsr        : unsigned(15 downto 0) := x"ACE1";
    signal s_lfsr_ln     : unsigned(15 downto 0) := x"5A5A";  -- steps per line

    -- Temporal (frame) phases and their per-line running copies
    signal s_rip_f1, s_rip_f2   : unsigned(15 downto 0) := (others => '0');
    signal s_caus_f1, s_caus_f2 : unsigned(15 downto 0) := x"4000";
    signal s_sw_f               : unsigned(15 downto 0) := (others => '0');
    signal s_rip_l1, s_rip_l2   : unsigned(15 downto 0) := (others => '0');
    signal s_caus_l1, s_caus_l2 : unsigned(15 downto 0) := (others => '0');
    signal s_sw_l               : unsigned(15 downto 0) := (others => '0');

    -- Left-fill line average: first 64 active pixels of the line being
    -- written; latched at hsync so it matches the line being read.
    signal s_sum_y, s_sum_u, s_sum_v : unsigned(15 downto 0) := (others => '0');
    signal s_avg_cnt : unsigned(6 downto 0) := (others => '0');
    signal s_avg_y : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_avg_u : unsigned(C_DW - 1 downto 0) := to_unsigned(512, C_DW);
    signal s_avg_v : unsigned(C_DW - 1 downto 0) := to_unsigned(512, C_DW);

    -- Murk depth gradient
    signal s_line_cnt    : unsigned(10 downto 0) := (others => '0');
    signal s_lines_total : unsigned(10 downto 0) := (others => '0');
    signal s_murk_step   : unsigned(6 downto 0) := (others => '0');
    signal s_murk_acc    : unsigned(15 downto 0) := (others => '0');
    signal s_murk_sub    : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-line step sequencer (horizontal blanking)
    --------------------------------------------------------------------------
    signal s_lstep    : unsigned(2 downto 0) := "111";
    signal s_sw_ph3   : unsigned(15 downto 0) := (others => '0');
    signal s_sw_sin   : signed(9 downto 0) := (others => '0');
    signal s_sw_sin_r : signed(9 downto 0) := (others => '0');
    signal s_swh_tri  : signed(9 downto 0) := (others => '0');
    signal s_prod_sw  : signed(18 downto 0) := (others => '0');
    signal s_base_off : signed(10 downto 0) := (others => '0');
    -- Swell-tracking caustic seed offsets: base_off (px) * fc (phase/px),
    -- mod 2^16 -- the caustic field sways with the displaced picture.
    signal s_prod_c1  : signed(23 downto 0) := (others => '0');
    signal s_prod_c2  : signed(24 downto 0) := (others => '0');
    -- Prism: per-line U/V base offsets (= base_off when Prism is off)
    signal s_base_u   : signed(10 downto 0) := (others => '0');
    signal s_base_v   : signed(10 downto 0) := (others => '0');
    -- Prism fixed split: (w >> 7) + (w >> 8) of the measured line width
    -- (~22 px at 1920, ~8 px at 720), free-running register
    signal s_psplit   : signed(10 downto 0) := (others => '0');
    -- tint lerp factor (top 4 knob bits; halved while Prism is on)
    signal s_tint_f   : unsigned(3 downto 0) := (others => '0');

    -- Resolution scaling (see f_vstep_y): timing class, y step, the per-line
    -- virtual-line repeat counter, x-scaled per-pixel frequencies and the
    -- 1/xstep pixel amplitudes.
    signal s_sd       : std_logic := '0';                 -- SD: x step 2.0
    signal s_h7       : std_logic := '0';                 -- 720p: x step 1.5
    signal s_ys       : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_yfrac    : unsigned(7 downto 0) := (others => '0');
    signal s_yrep     : unsigned(2 downto 0) := (others => '0');
    signal s_fx1x     : unsigned(15 downto 0) := to_unsigned(700, 16);
    signal s_fx2x     : unsigned(15 downto 0) := to_unsigned(1138, 16);
    signal s_fc1x     : unsigned(15 downto 0) := to_unsigned(300, 16);
    signal s_fc2x     : unsigned(15 downto 0) := to_unsigned(404, 16);
    signal s_ripa     : unsigned(9 downto 0) := (others => '0');
    signal s_swa      : unsigned(9 downto 0) := (others => '0');
    signal s_chop     : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-pixel pipeline (E2..E8)
    --------------------------------------------------------------------------
    signal s2_t1, s2_t2 : signed(9 downto 0) := (others => '0');
    signal s2_ac1, s2_ac2 : unsigned(15 downto 0) := (others => '0');
    signal s2_x  : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s2_g  : unsigned(1 downto 0) := (others => '0');

    signal s3_fine : signed(10 downto 0) := (others => '0');
    signal s3_c1, s3_c2 : signed(9 downto 0) := (others => '0');
    signal s3_x    : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s3_g    : unsigned(1 downto 0) := (others => '0');

    signal s4_rip  : signed(19 downto 0) := (others => '0');
    signal s4_q    : signed(12 downto 0) := (others => '0');
    signal s4_qu, s4_qv : signed(12 downto 0) := (others => '0');
    signal s4_caus : signed(10 downto 0) := (others => '0');
    signal s4_g    : unsigned(1 downto 0) := (others => '0');

    signal s_rd_addr_y  : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_rd_addr_u  : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_rd_addr_v  : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s5_hl : unsigned(9 downto 0) := (others => '0');
    signal s5_g  : unsigned(1 downto 0) := (others => '0');
    signal s5_lfill, s6_lfill : std_logic := '0';

    signal s6_rbank : std_logic := '0';
    signal s6_yadd  : unsigned(9 downto 0) := (others => '0');

    signal s_wet_y, s_wet_u, s_wet_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s7_yadd : unsigned(9 downto 0) := (others => '0');

    signal s8_y, s8_u, s8_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    -- Tinted wet pixel (E9)
    signal s9_y, s9_u, s9_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Dry / sync delay shift registers (dry data only to the mix tap)
    --------------------------------------------------------------------------
    type t_data_shift is array (0 to C_DRY_TAP)
        of std_logic_vector(C_DW - 1 downto 0);
    type t_bit_shift  is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_y_sr     : t_data_shift := (others => (others => '0'));
    signal s_u_sr     : t_data_shift := (others => (others => '0'));
    signal s_v_sr     : t_data_shift := (others => (others => '0'));
    signal s_hsync_sr : t_bit_shift  := (others => '1');
    signal s_vsync_sr : t_bit_shift  := (others => '1');
    signal s_field_sr : t_bit_shift  := (others => '1');
    signal s_avid_sr  : t_bit_shift  := (others => '0');

    --------------------------------------------------------------------------
    -- Interpolator I/O
    --------------------------------------------------------------------------
    signal s_mix_t : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_mix_y_a, s_mix_u_a, s_mix_v_a : unsigned(C_DW - 1 downto 0);
    signal s_mix_y_r, s_mix_u_r, s_mix_v_r : unsigned(C_DW - 1 downto 0);
    signal s_mix_y_valid, s_mix_u_valid, s_mix_v_valid : std_logic;

begin

    --------------------------------------------------------------------------
    -- E1: register inputs and knobs, run counters/accumulators, and both
    -- step sequencers (single driver for all phase state).
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_abs : signed(10 downto 0);
        variable v_mag : unsigned(13 downto 0);
        variable v_sw  : unsigned(11 downto 0);
        variable v_r   : signed(15 downto 0);
        variable v_yf  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- SPI registers pre-registered before any arithmetic sees them
            s_rip_k   <= unsigned(registers_in(0));
            s_wave_k  <= unsigned(registers_in(1));
            s_rate_k  <= unsigned(registers_in(2));
            s_scale_k <= unsigned(registers_in(3));
            s_tint_k  <= unsigned(registers_in(4));
            s_caus_k  <= unsigned(registers_in(5));
            s_hue     <= registers_in(6)(0);
            s_glitter <= registers_in(6)(1);
            s_murk_en <= registers_in(6)(2);
            s_storm   <= registers_in(6)(3);
            s_prism   <= registers_in(6)(4);
            -- Prism split: 22 virtual units (= (1920 >> 7) + (1920 >> 8), the
            -- v1.4.2 1080p value) in real pixels: 22 / 15 / 11.
            if s_sd = '1' then
                s_psplit <= to_signed(11, 11);
            elsif s_h7 = '1' then
                s_psplit <= to_signed(15, 11);
            else
                s_psplit <= to_signed(22, 11);
            end if;
            -- timing class + y step, free-running from the timing ID
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_sd <= '1'; s_h7 <= '0';
                when 5 | 6 | 14     => s_sd <= '0'; s_h7 <= '1';
                when others         => s_sd <= '0'; s_h7 <= '0';
            end case;
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            if s_prism = '1' then
                s_tint_f <= '0' & s_tint_k(9 downto 7);
            else
                s_tint_f <= s_tint_k(9 downto 6);
            end if;
            s_mix_t   <= unsigned(registers_in(7));

            if s_hue = '1' then
                s_tgt_u <= C_ABYSS_U;
                s_tgt_v <= C_ABYSS_V;
            else
                s_tgt_u <= C_AQUA_U;
                s_tgt_v <= C_AQUA_V;
            end if;

            s_in_y    <= unsigned(data_in.y);
            s_in_u    <= unsigned(data_in.u);
            s_in_v    <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Per-pixel: write column counter and the four phase
            -- accumulators run along together.
            if data_in.avid = '1' then
                s_pixel_x <= s_pixel_x + 1;
                s_acc_r1  <= s_acc_r1 + s_fx1x;
                s_acc_r2  <= s_acc_r2 + s_fx2x;
                s_acc_c1  <= s_acc_c1 + s_fc1x;
                s_acc_c2  <= s_acc_c2 + s_fc2x;
            end if;

            -- Left-fill average: 64 registered pixels, starting past the
            -- (often black) source edge columns
            if s_in_avid = '1' and s_pixel_x > to_unsigned(8, C_Y_AW)
               and s_avg_cnt < to_unsigned(64, 7) then
                s_sum_y   <= s_sum_y + resize(s_in_y, 16);
                s_sum_u   <= s_sum_u + resize(s_in_u, 16);
                s_sum_v   <= s_sum_v + resize(s_in_v, 16);
                s_avg_cnt <= s_avg_cnt + 1;
            end if;

            ------------------------------------------------------------------
            -- Per-line sequencer (horizontal blanking): swell wave + seeding
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>       -- 3x harmonic phase; main swell sine
                                -- (waits for the virtual-line repeats)
                    s_sw_ph3 <= shift_left(s_sw_l, 1) + s_sw_l;
                    s_sw_sin <= to_signed(C_SINE(to_integer(s_sw_l(15 downto 8))), 10);
                    if s_yrep = 0 then
                        s_lstep <= s_lstep + 1;
                    end if;
                when 1 =>       -- re-register the ROM read data in fabric: the
                                -- BRAM output register absorbs s_sw_sin, so a
                                -- direct multiply would time from RAM.RDATA
                                -- (spiroscope EBR lesson)
                    s_sw_sin_r <= s_sw_sin;
                    s_swh_tri  <= f_tri(s_sw_ph3);
                    s_lstep    <= s_lstep + 1;
                when 2 =>       -- main swell multiply (8-bit amp: 4-px steps);
                                -- storm chop +-64 virtual units in pixels
                    s_prod_sw <= s_sw_sin_r * signed('0' & s_swa(9 downto 2));
                    if s_sd = '1' then
                        s_chop <= shift_right(s_swh_tri, 3);
                    elsif s_h7 = '1' then
                        s_chop <= shift_right(s_swh_tri, 3) + shift_right(s_swh_tri, 5);
                    else
                        s_chop <= shift_right(s_swh_tri, 2);
                    end if;
                    s_lstep   <= s_lstep + 1;
                when 3 =>       -- base read offset; storm adds a fixed +-63 px
                                -- 3rd harmonic chop (pure shift, no multiply)
                    if s_storm = '1' then
                        s_base_off <= resize(shift_right(s_prod_sw, 8), 11)
                                    + resize(s_chop, 11);
                    else
                        s_base_off <= resize(shift_right(s_prod_sw, 8), 11);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>       -- caustic swell-tracking multiplies; murk;
                                -- Prism U/V bases (+-1/2 swell, +-s_psplit)
                    s_prod_c1 <= s_base_off * signed('0' & s_fc1x(11 downto 0));
                    s_prod_c2 <= s_base_off * signed('0' & s_fc2x(12 downto 0));
                    if s_prism = '1' then
                        s_base_u <= s_base_off + shift_right(s_base_off, 1)
                                  + s_psplit;
                        s_base_v <= s_base_off - shift_right(s_base_off, 1)
                                  - s_psplit;
                    else
                        s_base_u <= s_base_off;
                        s_base_v <= s_base_off;
                    end if;
                    if s_murk_en = '1' then
                        if s_murk_acc(15 downto 6) > to_unsigned(288, 10) then
                            s_murk_sub <= to_unsigned(288, 10);
                        else
                            s_murk_sub <= s_murk_acc(15 downto 6);
                        end if;
                    else
                        s_murk_sub <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>       -- seed per-pixel accumulators; the caustic
                                -- seeds carry the swell offset so the light
                                -- pattern rides the displaced picture
                    s_acc_r1 <= s_rip_l1;
                    s_acc_r2 <= s_rip_l2;
                    s_acc_c1 <= s_caus_l1 + unsigned(s_prod_c1(15 downto 0));
                    s_acc_c2 <= s_caus_l2 + unsigned(s_prod_c2(15 downto 0));
                    s_lstep  <= s_lstep + 1;
                when others =>
                    null;
            end case;

            -- Virtual-line repeats: the line phases advance once per virtual
            -- line crossed; the hsync edge below does the first advance, these
            -- clocks the rest (SD up to 4 more; never at 1080p).  Placed
            -- before the frame sequencer so its line-phase copy wins.
            if s_yrep /= 0 then
                s_rip_l1  <= s_rip_l1 + resize(s_fy1, 16);
                s_rip_l2  <= s_rip_l2 + resize(s_fy2, 16);
                s_caus_l1 <= s_caus_l1 + resize(s_fyc1, 16);
                s_caus_l2 <= s_caus_l2 + resize(s_fyc2, 16);
                s_sw_l    <= s_sw_l + resize(s_fsw, 16);
                s_yrep    <= s_yrep - 1;
            end if;

            ------------------------------------------------------------------
            -- Per-frame sequencer (vertical blanking): control curves,
            -- frequencies, temporal phase advance.
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_d <= signed('0' & s_rate_k) - to_signed(512, 11);
                    if s_lfsr(0) = '1' then
                        s_lfsr <= ('0' & s_lfsr(15 downto 1)) xor x"B400";
                    else
                        s_lfsr <= '0' & s_lfsr(15 downto 1);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    v_abs  := abs(s_d);
                    s_absd <= unsigned(v_abs(9 downto 0));
                    -- Scale curve: piecewise-linear knee replacing K4^2 (no
                    -- multiply). Matches the quadratic at 0/512, hotter top.
                    if s_scale_k <= to_unsigned(512, 10) then
                        s_scale_v <= resize(s_scale_k & '0', 13);
                    else
                        s_scale_v <= to_unsigned(1024, 13)
                                   + resize((s_scale_k - 512) & "000", 13);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- Rate curve: piecewise-linear knee replacing d^2 (no
                    -- multiply); gentle 1:1 below half, x16 slope above.
                    if s_absd <= to_unsigned(256, 10) then
                        s_rate_mag <= resize(s_absd, 13);
                    else
                        s_rate_mag <= resize(s_absd, 13)
                                    + resize((s_absd - 256) & "0000", 13);
                    end if;
                    if s_storm = '1' then
                        v_sw := resize(s_wave_k, 12) + resize(s_wave_k(9 downto 1), 12);
                        if v_sw > to_unsigned(1023, 12) then
                            s_swamp <= to_unsigned(1023, 10);
                        else
                            s_swamp <= v_sw(9 downto 0);
                        end if;
                    else
                        s_swamp <= s_wave_k;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    v_mag := resize(s_rate_mag, 14);
                    if s_absd < to_unsigned(8, 10) then     -- centre detent
                        s_rate <= (others => '0');
                    elsif s_d(10) = '1' then
                        s_rate <= -signed(v_mag);
                    else
                        s_rate <= signed(v_mag);
                    end if;
                    s_fx1 <= to_unsigned(128, 13) + s_scale_v;
                    s_fc1 <= resize(to_unsigned(64, 12) + s_scale_v(12 downto 2), 12);
                    s_fsw <= resize(to_unsigned(24, 10) + s_scale_v(12 downto 4), 10);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_fx2  <= resize(s_fx1, 14) + resize(s_fx1(12 downto 1), 14)
                            + resize(s_fx1(12 downto 3), 14);
                    s_fc2  <= resize(s_fc1, 13) + resize(s_fc1(11 downto 2), 13)
                            + to_unsigned(29, 13);
                    s_fy1  <= s_fx1(12 downto 2);
                    s_fyc1 <= s_fc1;
                    if s_storm = '1' then
                        s_rate_eff <= shift_left(resize(s_rate, 15), 1);
                    else
                        s_rate_eff <= resize(s_rate, 15);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_fy2  <= s_fx2(13 downto 1);
                    s_fyc2 <= s_fc2(12 downto 1);
                    v_r    := resize(s_rate_eff, 16);
                    s_rip_f1  <= unsigned(signed(s_rip_f1) + v_r);
                    s_rip_f2  <= unsigned(signed(s_rip_f2) - v_r - shift_right(v_r, 1));
                    s_caus_f1 <= unsigned(signed(s_caus_f1) + shift_right(v_r, 1));
                    s_caus_f2 <= unsigned(signed(s_caus_f2) - shift_right(v_r, 1)
                                                            - shift_right(v_r, 2));
                    s_sw_f    <= unsigned(signed(s_sw_f) + shift_right(v_r, 1));
                    s_vstep   <= s_vstep + 1;
                when 6 =>
                    s_rip_l1  <= s_rip_f1;
                    s_rip_l2  <= s_rip_f2;
                    s_caus_l1 <= s_caus_f1;
                    s_caus_l2 <= s_caus_f2;
                    s_sw_l    <= s_sw_f;
                    s_thr     <= to_signed(528, 11)
                               - signed(resize(s_caus_k(9 downto 1), 11))
                               - signed(resize(s_caus_k(9 downto 3), 11));
                    if s_lines_total >= to_unsigned(900, 11) then
                        s_murk_step <= to_unsigned(17, 7);
                    elsif s_lines_total >= to_unsigned(650, 11) then
                        s_murk_step <= to_unsigned(26, 7);
                    elsif s_lines_total >= to_unsigned(450, 11) then
                        s_murk_step <= to_unsigned(34, 7);
                    elsif s_lines_total >= to_unsigned(260, 11) then
                        s_murk_step <= to_unsigned(64, 7);
                    else
                        s_murk_step <= to_unsigned(76, 7);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 7 =>       -- resolution scaling: per-pixel frequencies
                                -- x xstep; amplitudes / xstep (SD 1/2, 720p
                                -- 1/2 + 1/8 + 1/32 ~ 2/3, 1080 1)
                    s_fx1x <= f_xsc(s_fx1, s_sd, s_h7);
                    s_fx2x <= f_xsc(s_fx2, s_sd, s_h7);
                    s_fc1x <= f_xsc(s_fc1, s_sd, s_h7);
                    s_fc2x <= f_xsc(s_fc2, s_sd, s_h7);
                    if s_sd = '1' or s_h7 = '1' then
                        s_ripa <= shift_right(s_rip_k, 1);
                        s_swa  <= shift_right(s_swamp, 1);
                    else
                        s_ripa <= s_rip_k;
                        s_swa  <= s_swamp;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    if s_h7 = '1' then
                        s_ripa <= s_ripa + shift_right(s_rip_k, 3);
                        s_swa  <= s_swa + shift_right(s_swamp, 3);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    if s_h7 = '1' then
                        s_ripa <= s_ripa + shift_right(s_rip_k, 5);
                        s_swa  <= s_swa + shift_right(s_swamp, 5);
                    end if;
                    s_vstep <= (others => '1');
                when others =>
                    null;
            end case;


            -- hsync falling edge = new line: latch width, reset counters,
            -- flip parity, advance the per-line phases, kick the sequencer.
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_pixel_x /= 0 then
                    s_line_width <= s_pixel_x;
                end if;
                s_pixel_x  <= (others => '0');
                s_par      <= not s_par;
                s_rip_l1   <= s_rip_l1 + resize(s_fy1, 16);
                s_rip_l2   <= s_rip_l2 + resize(s_fy2, 16);
                s_caus_l1  <= s_caus_l1 + resize(s_fyc1, 16);
                s_caus_l2  <= s_caus_l2 + resize(s_fyc2, 16);
                s_sw_l     <= s_sw_l + resize(s_fsw, 16);
                v_yf       := resize(s_yfrac, 12) + s_ys;
                s_yfrac    <= v_yf(7 downto 0);
                s_yrep     <= v_yf(10 downto 8) - 1;
                s_murk_acc <= s_murk_acc + resize(s_murk_step, 16);
                s_line_cnt <= s_line_cnt + 1;
                if s_lfsr_ln(0) = '1' then
                    s_lfsr_ln <= ('0' & s_lfsr_ln(15 downto 1)) xor x"B400";
                else
                    s_lfsr_ln <= '0' & s_lfsr_ln(15 downto 1);
                end if;
                s_lstep    <= (others => '0');
                -- Latch the finished line's average colour for the left fill
                -- (that line is the one read back during the next line);
                -- neutral black when the line had no video (vblank).
                if s_avg_cnt = to_unsigned(64, 7) then
                    s_avg_y <= s_sum_y(15 downto 6);
                    s_avg_u <= s_sum_u(15 downto 6);
                    s_avg_v <= s_sum_v(15 downto 6);
                else
                    s_avg_y <= (others => '0');
                    s_avg_u <= to_unsigned(512, C_DW);
                    s_avg_v <= to_unsigned(512, C_DW);
                end if;
                s_sum_y   <= (others => '0');
                s_sum_u   <= (others => '0');
                s_sum_v   <= (others => '0');
                s_avg_cnt <= (others => '0');
            end if;

            -- vsync falling edge = new frame (placed last so it wins on a
            -- coincident hsync edge).
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_par         <= '0';
                s_vstep       <= (others => '0');
                s_lines_total <= s_line_cnt;
                s_line_cnt    <= (others => '0');
                s_yfrac       <= (others => '0');
                s_murk_acc    <= (others => '0');
            end if;

            -- Dry / sync shift registers
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_DRY_TAP loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- E2..E8: per-pixel wet pipeline (folds, ripple multiply, read address,
    -- caustic highlight, bank select, grade).
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_hl  : signed(11 downto 0);
        variable v_add : unsigned(10 downto 0);
        variable v_p1, v_p2 : unsigned(15 downto 0);
        variable v_h4  : unsigned(3 downto 0);
        variable v_q   : signed(12 downto 0);
        variable v_w   : signed(12 downto 0);
        variable v_ru, v_rv : signed(12 downto 0);
        variable v_qu, v_qv : signed(12 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_du, v_dv : signed(10 downto 0);
        variable v_pu, v_pv : signed(15 downto 0);
        variable v_tu, v_tv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- E2: ripple folds; caustic accumulators ride one stage so the
            -- ripple folds can phase-modulate them at E3
            s2_t1  <= f_tri(s_acc_r1);
            s2_t2  <= f_tri(s_acc_r2);
            s2_ac1 <= s_acc_c1;
            s2_ac2 <= s_acc_c2;
            s2_x   <= s_pixel_x;
            s2_g   <= (s_acc_c1(13) xor s_acc_c2(12))
                    & (s_acc_c1(12) xor s_acc_c2(11));

            -- E3: ripple sum; caustic folds, phase-modulated by the ripple
            -- waves (+-1/8 period) so the lattice bends into moving curves
            s3_fine <= resize(s2_t1, 11) + resize(shift_right(s2_t2, 1), 11);
            v_p1 := unsigned(signed(s2_ac1) + shift_left(resize(s2_t1, 16), 5));
            v_p2 := unsigned(signed(s2_ac2) - shift_left(resize(s2_t2, 16), 5));
            s3_c1 <= f_tri(v_p1);
            s3_c2 <= f_tri(v_p2);
            s3_x  <= s2_x;
            s3_g  <= s2_g;

            -- E4: ripple multiply (8-bit amp: same 0..95 px range, 4-px
            -- amplitude steps), base position, caustic sum + hash dither
            -- (+-32, reseeded per frame) to roughen the contour edges
            s4_rip <= s3_fine * signed('0' & s_ripa(9 downto 2));
            s4_q   <= signed(resize(s3_x, 13)) + resize(s_base_off, 13);
            s4_qu  <= signed(resize(s3_x, 13)) + resize(s_base_u, 13);
            s4_qv  <= signed(resize(s3_x, 13)) + resize(s_base_v, 13);
            v_h4   := s3_x(4 downto 1) xor s3_x(8 downto 5)
                      xor s_lfsr_ln(3 downto 0);
            s4_caus <= resize(s3_c1, 11) + resize(s3_c2, 11)
                     + shift_left(resize(signed('0' & v_h4)
                                         - to_signed(8, 5), 11), 1);
            s4_g <= s3_g;

            -- E5: read address add + clamp; highlight threshold. U and V
            -- follow luma's ripple (Prism splits them by the swell and a
            -- fixed offset, set per line in the sequencer).
            v_q := s4_q + resize(shift_right(s4_rip, 10), 13);
            v_ru := resize(shift_right(s4_rip, 10), 13);
            v_rv := resize(shift_right(s4_rip, 10), 13);
            v_qu := s4_qu + v_ru;
            v_qv := s4_qv + v_rv;
            v_w := signed(resize(s_line_width, 13));
            s5_lfill <= '0';
            if s_line_width > to_unsigned(8, C_Y_AW) then
                if v_q < to_signed(C_LEDGE, 13) then
                    v_q := to_signed(C_LEDGE, 13);
                    s5_lfill <= '1';   -- left edge zone: paint line avg
                elsif v_q > v_w then
                    v_q := v_w;
                end if;
                if v_qu < to_signed(C_LEDGE, 13) then
                    v_qu := to_signed(C_LEDGE, 13);
                elsif v_qu > v_w then
                    v_qu := v_w;
                end if;
                if v_qv < to_signed(C_LEDGE, 13) then
                    v_qv := to_signed(C_LEDGE, 13);
                elsif v_qv > v_w then
                    v_qv := v_w;
                end if;
            else
                v_q  := to_signed(1, 13);
                v_qu := to_signed(1, 13);
                v_qv := to_signed(1, 13);
            end if;
            s_rd_addr_y <= unsigned(v_q(C_Y_AW - 1 downto 0));
            s_rd_addr_u <= unsigned(v_qu(C_Y_AW - 1 downto 1));
            s_rd_addr_v <= unsigned(v_qv(C_Y_AW - 1 downto 1));

            v_hl := resize(s4_caus, 12) - resize(s_thr, 12);
            if v_hl < 0 then
                s5_hl <= (others => '0');
            elsif v_hl > to_signed(511, 12) then
                s5_hl <= to_unsigned(511, 10);
            else
                s5_hl <= unsigned(v_hl(9 downto 0));
            end if;
            s5_g <= s4_g;

            -- E6: context aligned with the bank read data; Y-add (v1.1.1:
            -- halved -- 1x gain capped at +192, glitter cores +256)
            s6_rbank <= not s_par;
            s6_lfill <= s5_lfill;
            v_add := resize(s5_hl, 11);
            if v_add > to_unsigned(192, 11) then
                v_add := to_unsigned(192, 11);
            end if;
            if s_glitter = '1' and s5_hl > to_unsigned(300, 10)
               and ((s5_g xor s_lfsr(1 downto 0)) = "11") then
                v_add := to_unsigned(256, 11);
            end if;
            s6_yadd <= v_add(9 downto 0);

            -- E7: bank select -> wet pixel; left overshoot gets the line
            -- average instead of a border-pixel smear
            if s6_lfill = '1' then
                s_wet_y <= s_avg_y;
                s_wet_u <= s_avg_u;
                s_wet_v <= s_avg_v;
            elsif s6_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                s_wet_u <= unsigned(s_rdU1);
                s_wet_v <= unsigned(s_rdV1);
            else
                s_wet_y <= unsigned(s_rdY0);
                s_wet_u <= unsigned(s_rdU0);
                s_wet_v <= unsigned(s_rdV0);
            end if;
            s7_yadd <= s6_yadd;

            -- E8: grade (caustic add, murk subtract, saturate)
            v_y := signed(resize(s_wet_y, 12)) + signed(resize(s7_yadd, 12))
                 - signed(resize(s_murk_sub, 12));
            if v_y < 0 then
                s8_y <= (others => '0');
            elsif v_y > to_signed(1023, 12) then
                s8_y <= to_unsigned(1023, C_DW);
            else
                s8_y <= unsigned(v_y(9 downto 0));
            end if;
            s8_u <= s_wet_u;
            s8_v <= s_wet_v;

            -- E9: tint shift-lerp toward the target colour. t is the top 4
            -- knob bits (16 levels); convex combination, so no clamp needed.
            v_du := signed(resize(s_tgt_u, 11)) - signed(resize(s8_u, 11));
            v_dv := signed(resize(s_tgt_v, 11)) - signed(resize(s8_v, 11));
            v_pu := v_du * signed('0' & s_tint_f);
            v_pv := v_dv * signed('0' & s_tint_f);
            v_tu := signed(resize(s8_u, 12)) + resize(shift_right(v_pu, 4), 12);
            v_tv := signed(resize(s8_v, 12)) + resize(shift_right(v_pv, 4), 12);
            s9_y <= s8_y;
            s9_u <= unsigned(v_tu(9 downto 0));
            s9_v <= unsigned(v_tv(9 downto 0));
        end if;
    end process p_pix;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank (single driver, single write port).
    -- Write = live E1-registered pixel into this line's parity bank; read =
    -- E5-registered displaced address, registered out (valid E6).
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_rd_addr_y));
            if s_in_avid = '1' and s_par = '0' then
                lbY0(to_integer(s_pixel_x)) <= std_logic_vector(s_in_y);
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_rd_addr_y));
            if s_in_avid = '1' and s_par = '1' then
                lbY1(to_integer(s_pixel_x)) <= std_logic_vector(s_in_y);
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_rd_addr_u));
            if s_in_avid = '1' and s_par = '0' then
                lbU0(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_u);
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_rd_addr_u));
            if s_in_avid = '1' and s_par = '1' then
                lbU1(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_u);
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_rd_addr_v));
            if s_in_avid = '1' and s_par = '0' then
                lbV0(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_v);
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_rd_addr_v));
            if s_in_avid = '1' and s_par = '1' then
                lbV1(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_v);
            end if;
        end if;
    end process p_lbV1;

    --------------------------------------------------------------------------
    -- Depth mix: dry -> wet master fader (E10..E13). Wet is valid at E9,
    -- so the dry tap is shift register index 8.
    --------------------------------------------------------------------------
    s_mix_y_a <= unsigned(s_y_sr(C_DRY_TAP));
    s_mix_u_a <= unsigned(s_u_sr(C_DRY_TAP));
    s_mix_v_a <= unsigned(s_v_sr(C_DRY_TAP));

    mix_y_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => s_mix_y_a,
            b      => s9_y,
            t      => s_mix_t,
            result => s_mix_y_r,
            valid  => s_mix_y_valid
        );

    mix_u_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => s_mix_u_a,
            b      => s9_u,
            t      => s_mix_t,
            result => s_mix_u_r,
            valid  => s_mix_u_valid
        );

    mix_v_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => s_mix_v_a,
            b      => s9_v,
            t      => s_mix_t,
            result => s_mix_v_r,
            valid  => s_mix_v_valid
        );

    --------------------------------------------------------------------------
    -- Output at E13. Sync from the shift register; video from the mix
    -- interpolators.
    --------------------------------------------------------------------------
    s_io_align_in.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    s_io_align_in.y <= std_logic_vector(s_mix_y_r);
    s_io_align_in.u <= std_logic_vector(s_mix_u_r);
    s_io_align_in.v <= std_logic_vector(s_mix_v_r);

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

end architecture lagoon;
