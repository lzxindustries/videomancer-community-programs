-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Starfield: shader-inspired glowing parallax starfield generator
--
-- 8 depth layers, cell-based star placement: 16x16 px cells on the six
-- far layers, 32x32 px cells on the near pair (6, 7). Each star is
-- rendered from a per-layer glow-kernel ROM in EBR (see gen_kernels.py):
-- address {glow_level(2b), shape(2b), |dy|, |dx|} -> 8-bit weight.
-- Four star shapes (gaussian glow / soft square / diffraction spike /
-- point+halo), smooth baked falloff, per-layer depth fade baked into the
-- ROM contents. Kernels are cell-safe (the neighbouring cell draws only
-- its own star, so a kernel must be zero before its cell edge): far
-- layers use offsets 4..11 of 16 and kernels zero from 4 units (|d| <= 3);
-- the near pair uses offsets 8..23 of 32 and v3.0's full-size kernels
-- (zero beyond 7.2, |d| <= 7) -- the size range runs from pinpoint dust
-- to big near stars. Per-star colours
-- from eight palettes. Luma Mode maps incoming video brightness to star
-- density and size; T8 Video composites the stars OVER the input picture
-- (alpha 'over': opaque cores, transparent glow edges -- warpfield's
-- HW-approved composite).
--
-- v3.1.3 (2026-10-17): resolution scaling. The pixel and line counters run
-- in virtual 1080-line units, stepped per pixel / per field line from the
-- video timing ID (registers_in(8)): 1.0 at 1080p (identical to before),
-- 1.5 at 720p, and at SD 2.0 across (a 1440-wide 4:3 window) and ~4.44 per
-- 480i field line, so cells, stars, nebula and scroll keep their size
-- relative to the screen height and stars stay round on a 4:3 screen. The
-- cost is paid by dropping the line-position delay chain (pixel_y is
-- constant across every active line, so the stages read it directly).
--
-- Optimised for iCE40 HX4K: kernels live in otherwise-unused EBR; the
-- only wide multiplier besides the 8 hash multipliers (canonical
-- signed-digit shift-adds) is one shared 10x10 brightness multiply after
-- the composite trees.
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Layers     (top 3 bits: 000=1 .. 111=8)
--   registers_in(1) = Density    (0-1023, manual density / luma gain)
--   registers_in(2) = Brightness (0-1023, linear multiply)
--   registers_in(3) = Palette    (bits 9:7 = Silver/Candle/Twilight/Neon/
--                     Ember/Honey/Rainbow/B&W)
--   registers_in(4) = Glow       (top 2 bits kernel level, low 8 dither)
--   registers_in(5) = Depth      (top 3 bits: layer parallax spread)
--   registers_in(6) = Switches   (bit0=Dir bit1=Video bit2=Luma
--                     bit3=BG bit4=Gate)
--   registers_in(7) = Speed      (P12 slider: quadratic curve from a slow
--                     drift to ~40 px/frame; top half = Lively mood)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.starfield_rom_pkg.all;
use work.all;

architecture starfield of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY   : natural := 18;
    constant NUM_LYRS  : natural := 8;

    type t_pipe    is array (natural range <>) of t_video_stream_yuv444_30b;
    type t_u24_arr is array(0 to NUM_LYRS-1) of unsigned(23 downto 0);
    type t_u18_arr is array(0 to NUM_LYRS-1) of unsigned(17 downto 0);
    type t_u17_arr is array(0 to NUM_LYRS-1) of unsigned(16 downto 0);
    type t_u16_arr is array(0 to NUM_LYRS-1) of unsigned(15 downto 0);
    type t_u8_arr  is array(0 to NUM_LYRS-1) of unsigned(7 downto 0);
    type t_u5_arr  is array(0 to NUM_LYRS-1) of unsigned(4 downto 0);
    type t_u4_arr  is array(0 to NUM_LYRS-1) of unsigned(3 downto 0);
    type t_u3_arr  is array(0 to NUM_LYRS-1) of unsigned(2 downto 0);
    type t_u2_arr  is array(0 to NUM_LYRS-1) of unsigned(1 downto 0);

    -- Layers FIRST_BIG.. (the near pair) sample 32 px cells with 5-bit
    -- sub-cell positions; every per-layer branch on it is static.
    constant FIRST_BIG : natural := 6;

    constant SEEDS : t_u16_arr := (
        x"A3B1", x"7C2D", x"D917", x"4E6F",
        x"B5C3", x"62E8", x"F147", x"8D9A"
    );

    -- Far-layer sub-cell star position map: 4..11 only, so a kernel of
    -- reach 4 can NEVER cross the cell edge (offsets 2..13 sliced big
    -- stars flat at cell borders -- warpfield, HW-confirmed 2026-10-07).
    -- Near layers use 8..23 of 32 (8 + hash nibble: pure wiring).
    type t_offmap is array (0 to 15) of unsigned(3 downto 0);
    constant C_OFFMAP : t_offmap := (
        x"4", x"4", x"5", x"5", x"6", x"6", x"7", x"7",
        x"8", x"8", x"9", x"9", x"A", x"A", x"B", x"B"
    );

    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Video delay for the overlay: an EBR circular buffer, NOT the FF pipe.
    -- Only pipe(0..1).y and the syncs are otherwise read, so yosys prunes
    -- the pipe's y/u/v chains; reading a deep pipe tap un-prunes ~450 FFs
    -- (warpfield overflowed to 7722 LC that way). Read pointer = write
    -- pointer - 14 so that, after the EBR output register and one fabric
    -- re-register, the sample lands at stage 5b (two before the output
    -- register) aligned with the star path (LATENCY 18). The pointers are
    -- never equal, so the RAM never reads the address being written:
    -- no_rw_check drops yosys's read-during-write emulation logic.
    type t_vdly is array (0 to 15) of std_logic_vector(31 downto 0);
    signal vdly_ram : t_vdly := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of vdly_ram : signal is true;
    signal vdly_wa  : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal vdly_ra  : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal vdly_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal vdly_q2  : std_logic_vector(31 downto 0) := (others => '0');

    -- Position, in virtual 1080-line units (resolution scaling, v3.1.3):
    -- pix_xa is Q.1 (half units; steps 2/3/4 = 1.0/1.5/2.0 per pixel),
    -- pix_ya Q.4 per FIELD line; pixel_x / pixel_y are the integer parts.
    -- At 1080p both steps are exactly 1.0, so they equal the old counters.
    signal s_xs         : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys         : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal pix_xa       : unsigned(12 downto 0) := (others => '0');
    signal pix_ya       : unsigned(15 downto 0) := (others => '0');
    signal pixel_x      : unsigned(11 downto 0);
    signal pixel_y      : unsigned(11 downto 0);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    -- active video seen since the last frame start: analog vsync serrates
    -- (several edges per field) and every per-field step must run once
    signal s_sawact     : std_logic := '0';

    -- 24-bit scroll accumulators, 18.6 fixed point (pixels . 1/64)
    signal scroll : t_u24_arr := (others => (others => '0'));

    -- Scroll pre-computation pipeline: always-updating registered helpers so
    -- the once-per-frame scroll() adder sees a single 24-bit add from reg->reg.
    -- Speeds are in 1/64 px/frame.
    signal spd_addr_r      : unsigned(7 downto 0) := (others => '0');
    signal spd_q           : std_logic_vector(15 downto 0) := (others => '0');
    signal speed16_r       : unsigned(15 downto 0) := (others => '0');
    signal floor_r         : unsigned(15 downto 0) := (others => '0');
    signal decel_1step_r   : unsigned(15 downto 0) := (others => '0');
    -- per-field scroll step, one layer per clock through one datapath
    -- (it was eight parallel copies computed every clock): sc_acc walks the
    -- deceleration ladder (7 - i) * decel from layer 7 down to layer 0, then
    -- speed - ladder, the floor clamp, and the add/subtract on scroll(7)
    -- while the scroll registers rotate one place (8 steps = back in place)
    signal sc_cnt   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal sc_acc   : unsigned(15 downto 0) := (others => '0');
    signal sc_p1    : unsigned(15 downto 0) := (others => '0');
    signal sc_p2    : unsigned(16 downto 0) := (others => '0');
    signal sc_p3    : unsigned(15 downto 0) := (others => '0');
    signal sc_v1, sc_v2, sc_v3 : std_logic := '0';
    signal dir_r           : std_logic := '0';

    -- Stage 2. (No line-position pipeline: pixel_y only changes at the
    -- hsync edge, in blanking, so every stage reads it directly -- the
    -- same value its pixel would have carried down, minus 32 FFs.)
    signal s2_sx : t_u18_arr;

    -- Density threshold = base * gain / 2, two registered stages from
    -- pipe(0).y, so it aligns with the pixel stage 3p sees.
    signal threshold_r : unsigned(12 downto 0) := (others => '0');
    signal thr_base_r  : unsigned(9 downto 0)  := (others => '0');
    signal thr_gain_r  : unsigned(4 downto 0)  := (others => '0');
    signal gate_r      : std_logic := '0';

    -- Registered knob-derived helpers (the SPI register file sits in one
    -- corner of the die; routing its outputs straight into the 8-layer
    -- property cone was the critical path). Knob values are quasi-static,
    -- so one cycle of staleness is invisible.
    signal glow_base_r : unsigned(1 downto 0) := (others => '0');
    -- Glow level in 1/256-level units: Glow fraction + (Luma Mode) the
    -- pixel's luma - 512, so dark video shrinks stars by up to ~1.75 levels
    -- and bright video grows them by up to ~1.7. lt_r is the sum; frac_r
    -- (bits 7:0) dithers stars between adjacent levels at 3p; lbase_r =
    -- Glow base + the signed whole-level part, one stage later for 4a1.
    signal lt_r        : signed(10 downto 0) := (others => '0');
    signal frac_r      : unsigned(7 downto 0) := (others => '0');
    signal lw_r        : signed(2 downto 0)  := (others => '0');
    signal lbase_r     : signed(3 downto 0)  := (others => '0');
    signal luma_mode_r : std_logic := '0';
    signal first_act_r : unsigned(2 downto 0) := (others => '0');
    signal pal_sel_r   : unsigned(2 downto 0) := (others => '0');
    signal bw_r        : std_logic := '0';
    signal bg_on_r     : std_logic := '0';
    signal video_on_r  : std_logic := '0';
    signal bright_s_r  : unsigned(9 downto 0) := (others => '0');  -- Brightness, registered

    -- Twinkle / fade animation state. Phases advance once per field;
    -- rates are mood-selected (Lively = Speed slider top half) and
    -- pre-registered like the other knob helpers.
    signal tw_phase    : unsigned(7 downto 0)  := (others => '0');
    signal fade_cnt    : unsigned(15 downto 0) := (others => '0');
    signal shim_cnt    : unsigned(15 downto 0) := (others => '0');

    -- Deep-space background: nebula texture scroll (12.10 fixed point,
    -- advances by speed16_r per frame -> drifts at 1/16 of the front
    -- layer's speed) and the short background sampling pipeline. The
    -- background path is only 4 stages, so it reaches the composite ~7
    -- pixels "early" — a fixed horizontal shift of a wrapping texture,
    -- which is invisible and costs no delay-matching registers.
    signal neb_scroll : unsigned(21 downto 0) := (others => '0');
    signal bg1_addr   : unsigned(11 downto 0) := (others => '0');
    signal bg2_neb    : std_logic_vector(7 downto 0) := (others => '0');
    signal bg2_grad   : unsigned(6 downto 0) := (others => '0');
    signal bg3_w      : unsigned(7 downto 0) := (others => '0');
    signal mood_r      : std_logic := '0';
    signal tw_rate_r   : unsigned(2 downto 0) := (others => '0');
    signal fade_rate_r : unsigned(7 downto 0) := (others => '0');
    signal shim_rate_r : unsigned(7 downto 0) := (others => '0');

    -- Stage 3a: hash pre-multiply (XOR part)
    signal s3a_h     : t_u16_arr;
    signal s3a_sub_x : t_u5_arr;

    -- Stage 3: hash finalised (after multiply + xor-shift)
    signal s3_hash  : t_u16_arr;
    signal s3_sub_x : t_u5_arr;

    -- Stage 3p: per-layer star properties straight off the hash (offsets,
    -- shape, colour, dither verdict, magnitude penalty, threshold check).
    -- Split from the glow-level combine: the 8-bit dither compare plus the
    -- add/clamp chain in one stage was the HD critical path.
    signal s3p_offset_x   : t_u5_arr;
    signal s3p_offset_y   : t_u5_arr;
    signal s3p_sub_x      : t_u5_arr;
    signal s3p_shape      : t_u2_arr;
    signal s3p_color_idx  : t_u2_arr;
    signal s3p_mag        : t_u2_arr;
    signal s3p_dith       : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    -- per-star twinkle / fade phase positions and permanent-star flag
    signal s3p_pt         : t_u8_arr;
    signal s3p_pf         : t_u8_arr;
    signal s3p_perm       : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    -- layer_active AND hash_v < threshold (AND not Gate-thinned), per layer
    signal s3p_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');

    -- Stage 4a1: glow-level combine (base + dither + luma carry -
    -- magnitude, clamped) and the twinkle x fade envelope
    signal s4a1_offset_x   : t_u5_arr;
    signal s4a1_offset_y   : t_u5_arr;
    signal s4a1_sub_x      : t_u5_arr;
    signal s4a1_shape      : t_u2_arr;
    signal s4a1_lvl        : t_u2_arr;
    signal s4a1_color_idx  : t_u2_arr;
    signal s4a1_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    -- combined twinkle*fade brightness envelope, 0..16 (16 = unity)
    signal s4a1_env        : t_u5_arr;

    -- Stage 4ax: absolute differences |sub - offset| per axis, folded to
    -- the kernel reach with an in-range kill (far layers |d| <= 3, near
    -- pair |d| <= 7). These registers feed the kernel-ROM addresses.
    signal s4ax_adx        : t_u3_arr;
    signal s4ax_ady        : t_u3_arr;
    signal s4ax_shape      : t_u2_arr;
    signal s4ax_lvl        : t_u2_arr;
    signal s4ax_color_idx  : t_u2_arr;
    signal s4ax_hit        : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4ax_env        : t_u5_arr;

    -- Stage 4k0: kernel-ROM synchronous read (the EBR output register)
    signal s4k0_w   : t_u8_arr := (others => (others => '0'));
    signal s4k0_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4k0_ci  : t_u2_arr;
    signal s4k0_env : t_u5_arr;

    -- Stage 4k1: registered ROM read data, masked by the hit flag
    signal s4k1_w   : t_u8_arr := (others => (others => '0'));
    signal s4k1_ci  : t_u2_arr;
    signal s4k1_env : t_u5_arr;

    -- Stage 4e: per-layer envelope multiply (8x5, weight*env >> 4)
    signal s4e_w  : t_u8_arr := (others => (others => '0'));
    signal s4e_ci : t_u2_arr;

    -- Stage 4c1: pairwise sum (8->4) and pairwise max (8->4) of weights
    signal s4c1_sum_01, s4c1_sum_23, s4c1_sum_45, s4c1_sum_67 : unsigned(8 downto 0) := (others => '0');
    signal s4c1_max01_w, s4c1_max23_w, s4c1_max45_w, s4c1_max67_w     : unsigned(7 downto 0) := (others => '0');
    signal s4c1_max01_ci, s4c1_max23_ci, s4c1_max45_ci, s4c1_max67_ci : unsigned(1 downto 0) := (others => '0');

    -- Stage 4c2: quad sum (4->2) and quad max (4->2)
    signal s4c2_sum_0123, s4c2_sum_4567       : unsigned(9 downto 0) := (others => '0');
    signal s4c2_max0123_w, s4c2_max4567_w     : unsigned(7 downto 0) := (others => '0');
    signal s4c2_max0123_ci, s4c2_max4567_ci   : unsigned(1 downto 0) := (others => '0');

    -- Stage 4c3: total weight (clamped) + winning star colour index +
    -- alpha band from the brightest star's kernel weight: 4 = opaque,
    -- 3/2/1 = 3/4, 1/2, 1/4 (the glow edges), 0 = no star (background
    -- chroma at stage 5a)
    signal s4c3_w  : unsigned(9 downto 0) := (others => '0');
    signal s4c3_ci : unsigned(1 downto 0) := (others => '0');
    signal s4c3_a  : unsigned(2 downto 0) := (others => '0');
    signal s4m1_a  : unsigned(2 downto 0) := (others => '0');
    signal s4m2_a  : unsigned(2 downto 0) := (others => '0');

    -- Stage 4m1/4m2: shared brightness multiplier (inputs registered at
    -- 4m1, product registered at 4m2 — product valid two cycles after
    -- the operands, per the registered-multiplier rule). The shimmer
    -- palette EBR read runs in parallel: address registered into the
    -- read at 4m1, data registered at 4m2.
    signal s4m1_w      : unsigned(9 downto 0) := (others => '0');
    signal s4m1_bright : unsigned(9 downto 0) := (others => '0');
    signal s4m1_pal    : std_logic_vector(19 downto 0) := (others => '0');
    signal s4m2_y      : unsigned(11 downto 0) := (others => '0');
    signal s4m2_u      : unsigned(9 downto 0) := (others => '0');
    signal s4m2_v      : unsigned(9 downto 0) := (others => '0');

    -- Stage 5a: star colour over black + alpha band
    signal s5a_y  : unsigned(9 downto 0) := (others => '0');
    signal s5a_u  : unsigned(9 downto 0) := (others => '0');
    signal s5a_v  : unsigned(9 downto 0) := (others => '0');
    signal s5a_a  : unsigned(2 downto 0) := (others => '0');
    -- Stage 5b: under-layer (video, or black with alpha forced to 1) and
    -- star-minus-under differences
    signal s5b_vy, s5b_vu, s5b_vv : unsigned(9 downto 0) := (others => '0');
    signal s5b_dy, s5b_du, s5b_dv : signed(10 downto 0)  := (others => '0');
    signal s5b_a  : unsigned(2 downto 0) := (others => '0');
    -- Stage 5c: alpha mix + blanking gate
    signal s5_y : unsigned(9 downto 0);
    signal s5_u : unsigned(9 downto 0);
    signal s5_v : unsigned(9 downto 0);

    -- out = under + d * a, a in {0, 1/4, 1/2, 3/4, 1} by shift-adds. Both
    -- layers are legal-range codes, so the convex mix needs no clamp.
    function f_mix(under : unsigned(9 downto 0); d : signed(10 downto 0);
                   a : unsigned(2 downto 0)) return unsigned is
        variable dd : signed(10 downto 0);
        variable t  : signed(11 downto 0);
    begin
        case a is
            when "100"  => dd := d;
            when "011"  => dd := d - shift_right(d, 2);
            when "010"  => dd := shift_right(d, 1);
            when "001"  => dd := shift_right(d, 2);
            when others => dd := (others => '0');
        end case;
        t := signed(resize(under, 12)) + resize(dd, 12);
        return unsigned(t(9 downto 0));
    end function;

    -- Resolution scaling: per-mode steps from the video timing ID
    -- (registers_in(8)). x in half units per pixel: 1080 2 (1.0), 720p 3
    -- (1.5), SD 4 (2.0: 720 px span 1440 units, a 4:3 window of the 1080
    -- canvas, so stars stay round on a 4:3 screen).
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y in Q.4 units per FIELD line (1080 / active lines, doubled for
    -- interlace; 480i 71/16 = 4.4375 ~ 2 x 1080/486).
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(71, 7);   -- 480i
            when 4          => return to_unsigned(36, 7);   -- 480p: 2.25
            when 8          => return to_unsigned(60, 7);   -- 576i: 3.75
            when 12         => return to_unsigned(30, 7);   -- 576p: 1.875
            when 5 | 6 | 14 => return to_unsigned(24, 7);   -- 720p: 1.5
            when 1 | 2 | 10 => return to_unsigned(32, 7);   -- 1080i: 2.0
            when others     => return to_unsigned(16, 7);   -- 1080p: 1.0
        end case;
    end function;

begin

    pixel_x <= pix_xa(12 downto 1);
    pixel_y <= pix_ya(15 downto 4);

    process(clk)
        -- Scroll
        variable depth_3       : unsigned(2 downto 0);

        -- Controls
        variable v_knob        : unsigned(9 downto 0);
        variable num_layers_3  : unsigned(2 downto 0);
        variable density       : unsigned(9 downto 0);
        variable bright        : unsigned(9 downto 0);
        variable dim_v         : std_logic_vector(1 downto 0);
        variable v_diff_y      : unsigned(9 downto 0);
        variable v_thr_prod    : unsigned(14 downto 0);

        -- Stage 3
        variable cx16          : unsigned(15 downto 0);
        variable cell_y_16     : unsigned(15 downto 0);
        variable cy_rot        : unsigned(15 downto 0);
        variable hash_tmp      : unsigned(15 downto 0);

        -- Stage 4 per-layer
        variable hash_v        : unsigned(15 downto 0);
        variable dith8         : unsigned(7 downto 0);
        variable lvl_i         : integer range -5 to 8;
        variable d4            : unsigned(3 downto 0);
        variable d5            : unsigned(4 downto 0);
        variable inr_x         : boolean;
        variable inr_y         : boolean;
        variable addr_v        : unsigned(7 downto 0);
        variable addr_b        : unsigned(9 downto 0);
        variable hit_thr       : unsigned(13 downto 0);
        variable layer_active  : boolean;

        -- Twinkle / fade envelope
        variable pt_v          : unsigned(7 downto 0);
        variable pf_v          : unsigned(7 downto 0);
        variable tri7          : unsigned(6 downto 0);
        variable depth4        : unsigned(3 downto 0);
        variable env_t         : unsigned(4 downto 0);
        variable env_f         : unsigned(4 downto 0);
        variable env_v         : unsigned(4 downto 0);
        variable prod13        : unsigned(12 downto 0);

        -- Background path
        variable sum12         : unsigned(11 downto 0);

        -- Accumulation
        variable total_w       : unsigned(10 downto 0);

        -- Stage 4c3 final tree-reduction intermediates
        variable best_ci       : unsigned(1 downto 0);
        variable max_w         : unsigned(7 downto 0);

        -- Stage 4m1 shimmer-palette address / 4m2 brightness product
        variable paddr_v       : unsigned(8 downto 0);
        variable prod_v        : unsigned(19 downto 0);

        -- Stage 5b under-layer
        variable vy, vu, vv    : unsigned(9 downto 0);

    begin
        if rising_edge(clk) then

            ----------------------------------------------------------------
            -- Delay pipeline
            ----------------------------------------------------------------
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- video delay line (one write, one registered read -> EBR)
            vdly_ram(to_integer(vdly_wa)) <= "00" & data_in.y & data_in.u & data_in.v;
            vdly_q  <= vdly_ram(to_integer(vdly_ra));
            vdly_q2 <= vdly_q;
            vdly_wa <= vdly_wa + 1;
            vdly_ra <= vdly_ra + 1;

            ----------------------------------------------------------------
            -- STAGE 1 : Position tracking & per-field updates
            ----------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            -- virtual-unit steps, free-running (the timing ID is static
            -- while a mode runs)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                pix_xa   <= pix_xa + s_xs;
                s_sawact <= '1';
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                pix_xa <= (others => '0');
                pix_ya <= pix_ya + s_ys;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                -- line reset is idempotent: safe on every serration edge
                pix_ya <= (others => '0');

                -- Frame start = the FIRST vsync edge after active video:
                -- every increment below must happen exactly once per field
                if s_sawact = '1' then
                    s_sawact <= '0';

                    -- Twinkle / fade / shimmer phase advance (rates pre-registered)
                    tw_phase <= tw_phase + resize(tw_rate_r, 8);
                    fade_cnt <= fade_cnt + resize(fade_rate_r, 16);
                    shim_cnt <= shim_cnt + resize(shim_rate_r, 16);

                    -- Nebula drift: 1/16 of the front-layer speed, same direction
                    if dir_r = '0' then
                        neb_scroll <= neb_scroll + resize(speed16_r, 22);
                    else
                        neb_scroll <= neb_scroll - resize(speed16_r, 22);
                    end if;

                    -- Scroll update: start the serial per-layer step below.
                    -- The speed curve never reaches zero, so the field always
                    -- drifts (Direction and Depth stay live at slider 0).
                    sc_cnt <= (others => '0');
                    sc_acc <= (others => '0');
                end if;
            end if;

            ----------------------------------------------------------------
            -- Scroll speed pre-computation (always updating every clock).
            -- Registered stages break the "decel cascade -> subtract ->
            -- floor -> scroll DFF" chain so that the once-per-field scroll
            -- update needs only a single 24-bit add.
            ----------------------------------------------------------------

            -- Stage A: P12 speed curve (EBR ROM, re-registered into fabric),
            -- the far-layer floor (1/16 of the front speed) and direction
            spd_addr_r <= unsigned(registers_in(7)(9 downto 2));
            spd_q      <= C_SPEED(to_integer(spd_addr_r));
            speed16_r  <= unsigned(spd_q);
            floor_r    <= shift_right(speed16_r, 4);
            dir_r      <= registers_in(6)(0);

            -- K6 Depth. Labelled defaults: at program load the firmware
            -- writes the initial_value_label's INDEX (0..N-1) into the
            -- register, not a 0..1023 position (HW-measured 2026-10-02), so
            -- a value below the label count is taken as the label index.
            v_knob := unsigned(registers_in(5));
            if v_knob < 8 then
                depth_3 := v_knob(2 downto 0);
            else
                depth_3 := v_knob(9 downto 7);
            end if;
            case to_integer(depth_3) is
                when 0 =>
                    decel_1step_r <= to_unsigned(1, 16);
                when 1 =>
                    decel_1step_r <= shift_right(speed16_r, 5) + to_unsigned(1, 16);
                when 2 =>
                    decel_1step_r <= shift_right(speed16_r, 4) + to_unsigned(1, 16);
                when 3 =>
                    decel_1step_r <= shift_right(speed16_r, 3) + to_unsigned(1, 16);
                when 4 =>
                    decel_1step_r <= shift_right(speed16_r, 3)
                                   + shift_right(speed16_r, 4)
                                   + to_unsigned(1, 16);
                when 5 =>
                    decel_1step_r <= shift_right(speed16_r, 2) + to_unsigned(1, 16);
                when 6 =>
                    decel_1step_r <= shift_right(speed16_r, 2)
                                   + shift_right(speed16_r, 3)
                                   + to_unsigned(1, 16);
                when others =>
                    decel_1step_r <= shift_right(speed16_r, 1) + to_unsigned(1, 16);
            end case;

            -- Stage B: the deceleration ladder, (7 - i) * decel for layer i,
            -- built one add per clock from layer 7 (0) down to layer 0
            if sc_cnt <= 7 then
                sc_p1  <= sc_acc;
                sc_v1  <= '1';
                sc_acc <= sc_acc + decel_1step_r;
                sc_cnt <= sc_cnt + 1;
            else
                sc_v1  <= '0';
            end if;

            -- Stage C: speed - ladder, one bit wider (bit 16 = borrow, i.e.
            -- the layer would run backwards)
            sc_p2 <= resize(speed16_r, 17) - resize(sc_p1, 17);
            sc_v2 <= sc_v1;

            -- Stage D: floor at 1/16 of the front speed, so the far layers
            -- of a deep spread keep drifting (sub-pixel speeds: no layer is
            -- forced to a whole pixel per frame any more)
            if sc_p2(16) = '1' or sc_p2(15 downto 0) < floor_r then
                sc_p3 <= floor_r;
            else
                sc_p3 <= sc_p2(15 downto 0);
            end if;
            sc_v3 <= sc_v2;

            -- Stage E: step scroll(7) and rotate (layer 7 first, so after the
            -- eighth step every layer is back in its own register)
            if sc_v3 = '1' then
                for i in 1 to NUM_LYRS-1 loop
                    scroll(i) <= scroll(i - 1);
                end loop;
                if dir_r = '0' then
                    scroll(0) <= scroll(NUM_LYRS-1) + resize(sc_p3, 24);
                else
                    scroll(0) <= scroll(NUM_LYRS-1) - resize(sc_p3, 24);
                end if;
            end if;

            ----------------------------------------------------------------
            -- STAGE 2 : Scrolled X per layer (integer pixels)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                s2_sx(i) <= resize(pixel_x, 18) + scroll(i)(23 downto 6);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 3a : hash pre-multiply (XOR of cell_x, rotated cell_y, seed)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if i >= FIRST_BIG then         -- near pair: 32 px cells
                    cx16      := resize(s2_sx(i)(17 downto 5), 16);
                    cell_y_16 := resize(pixel_y(11 downto 5), 16);
                    s3a_sub_x(i) <= s2_sx(i)(4 downto 0);
                else                           -- far layers: 16 px cells
                    cx16      := resize(s2_sx(i)(17 downto 4), 16);
                    cell_y_16 := resize(pixel_y(11 downto 4), 16);
                    s3a_sub_x(i) <= '0' & s2_sx(i)(3 downto 0);
                end if;
                cy_rot := cell_y_16(8 downto 0) & cell_y_16(15 downto 9);
                s3a_h(i)     <= cx16 xor cy_rot xor SEEDS(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 3 : hash multiply + finalise (consumes registered 3a)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                -- x 0x9E37 mod 2^16 in canonical signed-digit form
                -- (2^15 + 2^13 - 2^9 + 2^6 - 2^3 - 1): 5 adders instead of
                -- the 9 yosys makes of the 10-set-bit constant; bit-identical
                hash_tmp  := shift_left(s3a_h(i), 15) + shift_left(s3a_h(i), 13)
                           - shift_left(s3a_h(i), 9)  + shift_left(s3a_h(i), 6)
                           - shift_left(s3a_h(i), 3)  - s3a_h(i);
                hash_tmp  := hash_tmp xor shift_right(hash_tmp, 5);
                hash_tmp  := hash_tmp xor (hash_tmp(10 downto 0) & "00000");
                s3_hash(i)  <= hash_tmp;
                s3_sub_x(i) <= s3a_sub_x(i);
            end loop;

            ----------------------------------------------------------------
            -- Registered knob helpers (quasi-static, one cycle stale)
            ----------------------------------------------------------------
            density := unsigned(registers_in(1));

            -- K1 Layers (labelled-default index rule, see K6 above)
            v_knob := unsigned(registers_in(0));
            if v_knob < 8 then
                num_layers_3 := v_knob(2 downto 0);
            else
                num_layers_3 := v_knob(9 downto 7);
            end if;
            first_act_r <= to_unsigned(7, 3) - num_layers_3;

            glow_base_r <= unsigned(registers_in(4)(9 downto 8));
            luma_mode_r <= registers_in(6)(2);
            gate_r      <= registers_in(6)(4);
            bg_on_r     <= registers_in(6)(3);
            video_on_r  <= registers_in(6)(1);
            pal_sel_r   <= unsigned(registers_in(3)(9 downto 7));
            if registers_in(3)(9 downto 7) = "111" then   -- K4 position 8 = B&W
                bw_r <= '1';
            else
                bw_r <= '0';
            end if;

            -- Brightness, registered. Unscaled (v3.0's glow brightness): the
            -- stage-5a cap at Y 768 keeps star cores off full scale, where
            -- Y clips RGB and kills star chroma; the halos keep their reach.
            bright := unsigned(registers_in(2));
            bright_s_r <= bright;

            -- Lively mood rides the Speed slider's top half: deeper, faster
            -- twinkle, ~3x faster fade churn and faster shimmer
            mood_r <= registers_in(7)(9);
            if registers_in(7)(9) = '1' then
                tw_rate_r   <= to_unsigned(5, 3);
                fade_rate_r <= to_unsigned(192, 8);
                shim_rate_r <= to_unsigned(128, 8);  -- hue orbit ~8.5 s
            else
                tw_rate_r   <= to_unsigned(2, 3);
                fade_rate_r <= to_unsigned(64, 8);
                shim_rate_r <= to_unsigned(36, 8);   -- hue orbit ~30 s
            end if;

            -- Glow level offset, three stages from pipe(0).y: frac_r lines
            -- up with 3p's pixel, lbase_r with 4a1's. In Luma Mode the
            -- pixel's luma - 512 (bit 9 inverted = the signed offset) is
            -- added: black ~-1.75 levels (pinpoints), white ~+1.7 (big glow).
            if registers_in(6)(2) = '1' then
                lt_r <= signed(resize(unsigned(registers_in(4)(7 downto 0)), 11))
                      + resize(signed((not pipe(0).y(9)) & pipe(0).y(8 downto 0)), 11);
            else
                lt_r <= signed(resize(unsigned(registers_in(4)(7 downto 0)), 11));
            end if;
            frac_r  <= unsigned(lt_r(7 downto 0));
            lw_r    <= lt_r(10 downto 8);
            lbase_r <= signed(resize(glow_base_r, 4)) + resize(lw_r, 4);

            -- Density threshold = base * gain / 2, two stages:
            --   manual : base = Density knob, gain 16  (= density * 8)
            --   luma   : base = input luma (Gate: (luma-256)*2 saturating,
            --            a steeper map), gain = Density(9:6) 0..15, so K2
            --            stays live as the luma gain
            if registers_in(6)(2) = '1' then
                thr_gain_r <= '0' & density(9 downto 6);
                if registers_in(6)(4) = '1' then
                    if unsigned(pipe(0).y) > to_unsigned(256, 10) then
                        v_diff_y := unsigned(pipe(0).y) - to_unsigned(256, 10);
                        if v_diff_y(9) = '1' then
                            thr_base_r <= (others => '1');
                        else
                            thr_base_r <= v_diff_y(8 downto 0) & '0';
                        end if;
                    else
                        thr_base_r <= (others => '0');
                    end if;
                else
                    thr_base_r <= unsigned(pipe(0).y);
                end if;
            else
                thr_gain_r <= to_unsigned(16, 5);
                thr_base_r <= density;
            end if;
            v_thr_prod  := thr_base_r * thr_gain_r;     -- <= 1023*16 = 16368
            threshold_r <= v_thr_prod(13 downto 1);

            ----------------------------------------------------------------
            -- STAGE 3p : per-layer star properties straight off the hash.
            --
            -- Star properties draw from the LOW hash bits: the presence
            -- test (hash < threshold) forces the high bits of surviving
            -- stars toward zero, so bits 15:12 carry almost no entropy for
            -- stars that actually exist. Bits 11:0 are used directly and
            -- xor-mixed where more fields are needed.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                layer_active := i >= to_integer(first_act_r);

                hash_v := s3_hash(i);
                if i >= FIRST_BIG then
                    -- 8 + nibble = 8..23 of a 32-unit cell
                    s3p_offset_x(i) <= hash_v(3) & (not hash_v(3)) & hash_v(2 downto 0);
                    s3p_offset_y(i) <= hash_v(7) & (not hash_v(7)) & hash_v(6 downto 4);
                else
                    s3p_offset_x(i) <= '0' & C_OFFMAP(to_integer(hash_v(3 downto 0)));
                    s3p_offset_y(i) <= '0' & C_OFFMAP(to_integer(hash_v(7 downto 4)));
                end if;
                s3p_shape(i)    <= hash_v(9 downto 8);
                -- Colour from bits 5:4 (uniform even at low densities,
                -- where the threshold test biases bits 11:10 toward zero),
                -- decorrelated from shape by the xor.
                s3p_color_idx(i) <= hash_v(5 downto 4) xor hash_v(9 downto 8);
                -- Per-star glow dither verdict (the 8-bit compare that
                -- forced this stage split) and the magnitude-penalty selector.
                dith8 := hash_v(11 downto 4) xor hash_v(7 downto 0);
                if dith8 < frac_r then
                    s3p_dith(i) <= '1';
                else
                    s3p_dith(i) <= '0';
                end if;
                s3p_mag(i)  <= hash_v(7 downto 6) xor hash_v(1 downto 0);
                s3p_sub_x(i) <= s3_sub_x(i);
                -- Near-pair cells are 4x the area: a doubled threshold
                -- gives them half the far layers' stars per screen area
                -- (near = sparser). Manual-mode Gate thins the field to
                -- the two bold magnitude classes (mag(1) = hash(7) xor hash(1)).
                if i >= FIRST_BIG then
                    hit_thr := threshold_r & '0';
                else
                    hit_thr := '0' & threshold_r;
                end if;
                if layer_active and hash_v < resize(hit_thr, 16)
                   and not (gate_r = '1' and luma_mode_r = '0'
                            and (hash_v(7) xor hash_v(1)) = '1') then
                    s3p_active_hit(i) <= '1';
                else
                    s3p_active_hit(i) <= '0';
                end if;

                -- Twinkle phase position: per-star offset (rotated dither
                -- byte — uniform even at low density) + global phase, with
                -- a per-star rate select (1x / 2x, the 2x phase being the
                -- global phase shifted left = wrapping twice as fast).
                if (hash_v(10) xor hash_v(0)) = '1' then
                    pt_v := (dith8(3 downto 0) & dith8(7 downto 4))
                          + (tw_phase(6 downto 0) & '0');
                else
                    pt_v := (dith8(3 downto 0) & dith8(7 downto 4))
                          + tw_phase;
                end if;
                s3p_pt(i) <= pt_v;

                -- Fade lifecycle position: per-star offset + slow counter.
                s3p_pf(i) <= (hash_v(3 downto 0) & hash_v(7 downto 4))
                           + fade_cnt(15 downto 8);

                -- ~25% of stars are permanent (never fade out), from
                -- xor-folded bits that stay uniform under the threshold
                -- test's high-bit bias.
                if (hash_v(6) xor hash_v(9)) = '1'
                   and (hash_v(2) xor hash_v(11)) = '1' then
                    s3p_perm(i) <= '1';
                else
                    s3p_perm(i) <= '0';
                end if;
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4a1 : glow-level combine + envelope.
            --
            -- Glow level 0..3 selects the kernel-ROM page:
            --   knob base (Glow top 2 bits) + the Luma-Mode signed whole
            --   levels (pre-added: lbase_r)
            --   + per-star dither verdict from 3p, so the population
            --     blends smoothly between adjacent levels as the knob turns
            --     (and, in Luma Mode, across the picture's luma)
            --   - per-star magnitude penalty (50% none, 25% -1, 25% -2)
            --     for size variety within the field
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                lvl_i := to_integer(lbase_r);
                if s3p_dith(i) = '1' then
                    lvl_i := lvl_i + 1;
                end if;
                if s3p_mag(i) = "10" then
                    lvl_i := lvl_i - 1;
                elsif s3p_mag(i) = "11" then
                    lvl_i := lvl_i - 2;
                end if;
                if lvl_i < 0 then
                    lvl_i := 0;
                elsif lvl_i > 3 then
                    lvl_i := 3;
                end if;

                -- Twinkle envelope: triangle fold of the phase position.
                -- Calm keeps stars between 9/16 and full; Lively dips to
                -- 4/16. Never zero — twinkle dims, fade extinguishes.
                pt_v := s3p_pt(i);
                if pt_v(7) = '1' then
                    tri7 := not pt_v(6 downto 0);
                else
                    tri7 := pt_v(6 downto 0);
                end if;
                if mood_r = '1' then
                    depth4 := tri7(6 downto 3);
                    if depth4 > to_unsigned(12, 4) then
                        depth4 := to_unsigned(12, 4);
                    end if;
                else
                    depth4 := '0' & tri7(6 downto 4);
                end if;
                env_t := to_unsigned(16, 5) - resize(depth4, 5);

                -- Fade envelope: quarter-cycle fade-in, half-cycle hold,
                -- quarter-cycle fade-out; permanent stars hold forever.
                pf_v := s3p_pf(i);
                case pf_v(7 downto 6) is
                    when "00"   => env_f := '0' & pf_v(5 downto 2);
                    when "11"   => env_f := '0' & not pf_v(5 downto 2);
                    when others => env_f := to_unsigned(16, 5);
                end case;
                if s3p_perm(i) = '1' then
                    env_f := to_unsigned(16, 5);
                end if;

                if env_t < env_f then
                    env_v := env_t;
                else
                    env_v := env_f;
                end if;

                s4a1_offset_x(i)  <= s3p_offset_x(i);
                s4a1_offset_y(i)  <= s3p_offset_y(i);
                s4a1_sub_x(i)     <= s3p_sub_x(i);
                s4a1_shape(i)     <= s3p_shape(i);
                s4a1_lvl(i)       <= to_unsigned(lvl_i, 2);
                s4a1_color_idx(i) <= s3p_color_idx(i);
                s4a1_active_hit(i) <= s3p_active_hit(i);
                s4a1_env(i)       <= env_v;
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4ax : absolute differences |sub - offset| per axis with
            -- in-range kill (outside the layer's kernel reach).
            -- These registers feed the kernel-ROM addresses directly.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if i >= FIRST_BIG then
                    -- near pair: 5-bit positions, reach |d| <= 7
                    if s4a1_sub_x(i) >= s4a1_offset_x(i) then
                        d5 := s4a1_sub_x(i) - s4a1_offset_x(i);
                    else
                        d5 := s4a1_offset_x(i) - s4a1_sub_x(i);
                    end if;
                    inr_x := d5(4 downto 3) = "00";
                    s4ax_adx(i) <= d5(2 downto 0);
                    if pixel_y(4 downto 0) >= s4a1_offset_y(i) then
                        d5 := pixel_y(4 downto 0) - s4a1_offset_y(i);
                    else
                        d5 := s4a1_offset_y(i) - pixel_y(4 downto 0);
                    end if;
                    inr_y := d5(4 downto 3) = "00";
                    s4ax_ady(i) <= d5(2 downto 0);
                else
                    -- far layers: 4-bit positions (explicitly narrow -- the
                    -- chip is full), reach |d| <= 3
                    if s4a1_sub_x(i)(3 downto 0) >= s4a1_offset_x(i)(3 downto 0) then
                        d4 := s4a1_sub_x(i)(3 downto 0) - s4a1_offset_x(i)(3 downto 0);
                    else
                        d4 := s4a1_offset_x(i)(3 downto 0) - s4a1_sub_x(i)(3 downto 0);
                    end if;
                    inr_x := d4(3 downto 2) = "00";
                    s4ax_adx(i) <= '0' & d4(1 downto 0);
                    if pixel_y(3 downto 0) >= s4a1_offset_y(i)(3 downto 0) then
                        d4 := pixel_y(3 downto 0) - s4a1_offset_y(i)(3 downto 0);
                    else
                        d4 := s4a1_offset_y(i)(3 downto 0) - pixel_y(3 downto 0);
                    end if;
                    inr_y := d4(3 downto 2) = "00";
                    s4ax_ady(i) <= '0' & d4(1 downto 0);
                end if;

                if s4a1_active_hit(i) = '1' and inr_x and inr_y then
                    s4ax_hit(i) <= '1';
                else
                    s4ax_hit(i) <= '0';
                end if;
                s4ax_shape(i)     <= s4a1_shape(i);
                s4ax_lvl(i)       <= s4a1_lvl(i);
                s4ax_color_idx(i) <= s4a1_color_idx(i);
                s4ax_env(i)       <= s4a1_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4k0 : kernel-ROM synchronous read (one ROM per layer:
            -- 256 entries far, 1024 near, addressed straight from the 4ax
            -- registers; this read
            -- register is the EBR's internal output register). The eight
            -- reads are unrolled onto eight separately-named constants:
            -- indexing a single array-of-arrays constant makes GHDL
            -- synthesis merge them into one deep ROM and crash (internal
            -- error in netlists-memories.adb).
            ----------------------------------------------------------------
            addr_v := s4ax_lvl(0) & s4ax_shape(0) & s4ax_ady(0)(1 downto 0) & s4ax_adx(0)(1 downto 0);
            s4k0_w(0) <= unsigned(C_KERNEL_L0(to_integer(addr_v)));
            addr_v := s4ax_lvl(1) & s4ax_shape(1) & s4ax_ady(1)(1 downto 0) & s4ax_adx(1)(1 downto 0);
            s4k0_w(1) <= unsigned(C_KERNEL_L1(to_integer(addr_v)));
            addr_v := s4ax_lvl(2) & s4ax_shape(2) & s4ax_ady(2)(1 downto 0) & s4ax_adx(2)(1 downto 0);
            s4k0_w(2) <= unsigned(C_KERNEL_L2(to_integer(addr_v)));
            addr_v := s4ax_lvl(3) & s4ax_shape(3) & s4ax_ady(3)(1 downto 0) & s4ax_adx(3)(1 downto 0);
            s4k0_w(3) <= unsigned(C_KERNEL_L3(to_integer(addr_v)));
            addr_v := s4ax_lvl(4) & s4ax_shape(4) & s4ax_ady(4)(1 downto 0) & s4ax_adx(4)(1 downto 0);
            s4k0_w(4) <= unsigned(C_KERNEL_L4(to_integer(addr_v)));
            addr_v := s4ax_lvl(5) & s4ax_shape(5) & s4ax_ady(5)(1 downto 0) & s4ax_adx(5)(1 downto 0);
            s4k0_w(5) <= unsigned(C_KERNEL_L5(to_integer(addr_v)));
            addr_b := s4ax_lvl(6) & s4ax_shape(6) & s4ax_ady(6) & s4ax_adx(6);
            s4k0_w(6) <= unsigned(C_KERNEL_L6(to_integer(addr_b)));
            addr_b := s4ax_lvl(7) & s4ax_shape(7) & s4ax_ady(7) & s4ax_adx(7);
            s4k0_w(7) <= unsigned(C_KERNEL_L7(to_integer(addr_b)));
            for i in 0 to NUM_LYRS-1 loop
                s4k0_hit(i) <= s4ax_hit(i);
                s4k0_ci(i)  <= s4ax_color_idx(i);
                s4k0_env(i) <= s4ax_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4k1 : register the ROM read data (EBR reads need a
            -- register before fabric use) and mask by the hit flag.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if s4k0_hit(i) = '1' then
                    s4k1_w(i) <= s4k0_w(i);
                else
                    s4k1_w(i) <= (others => '0');
                end if;
                s4k1_ci(i)  <= s4k0_ci(i);
                s4k1_env(i) <= s4k0_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4e : per-layer twinkle/fade envelope multiply.
            -- weight(8b) * env(5b) >> 4, env 16 = unity. Small 8x5
            -- multiplier, product registered here and consumed by the
            -- 4c1 trees next cycle.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                prod13 := s4k1_w(i) * s4k1_env(i);
                s4e_w(i)  <= prod13(11 downto 4);
                s4e_ci(i) <= s4k1_ci(i);
            end loop;

            ----------------------------------------------------------------
            -- BACKGROUND path (3 stages, weight domain): scrolling nebula
            -- texture + vertical gradient, gated by S10. Composites into
            -- the star weight sum at 4c3, so it scales with Brightness and
            -- sits under the stars automatically.
            ----------------------------------------------------------------
            -- BG1: texture address (4x stretch, 256-px wrapping tile)
            sum12 := resize(pixel_x, 12) + neb_scroll(21 downto 10);
            bg1_addr <= pixel_y(7 downto 2) & sum12(7 downto 2);

            -- BG2: nebula ROM read (EBR output register) + subtle gradient
            -- (24 -> 0 over the top ~768 lines)
            bg2_neb <= C_NEBULA(to_integer(bg1_addr));
            if pixel_y(11 downto 5) < to_unsigned(24, 7) then
                bg2_grad <= to_unsigned(24, 7) - resize(pixel_y(11 downto 5), 7);
            else
                bg2_grad <= (others => '0');
            end if;

            -- BG3: register the read data and combine (nebula/4 + gradient)
            if bg_on_r = '1' then
                bg3_w <= resize(unsigned(bg2_neb(7 downto 2)), 8)
                       + resize(bg2_grad, 8);
            else
                bg3_w <= (others => '0');
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c1 : pairwise (8->4) sum and max — registered
            ----------------------------------------------------------------
            s4c1_sum_01 <= resize(s4e_w(0), 9) + resize(s4e_w(1), 9);
            s4c1_sum_23 <= resize(s4e_w(2), 9) + resize(s4e_w(3), 9);
            s4c1_sum_45 <= resize(s4e_w(4), 9) + resize(s4e_w(5), 9);
            s4c1_sum_67 <= resize(s4e_w(6), 9) + resize(s4e_w(7), 9);

            if s4e_w(0) >= s4e_w(1) then
                s4c1_max01_w <= s4e_w(0); s4c1_max01_ci <= s4e_ci(0);
            else
                s4c1_max01_w <= s4e_w(1); s4c1_max01_ci <= s4e_ci(1);
            end if;
            if s4e_w(2) >= s4e_w(3) then
                s4c1_max23_w <= s4e_w(2); s4c1_max23_ci <= s4e_ci(2);
            else
                s4c1_max23_w <= s4e_w(3); s4c1_max23_ci <= s4e_ci(3);
            end if;
            if s4e_w(4) >= s4e_w(5) then
                s4c1_max45_w <= s4e_w(4); s4c1_max45_ci <= s4e_ci(4);
            else
                s4c1_max45_w <= s4e_w(5); s4c1_max45_ci <= s4e_ci(5);
            end if;
            if s4e_w(6) >= s4e_w(7) then
                s4c1_max67_w <= s4e_w(6); s4c1_max67_ci <= s4e_ci(6);
            else
                s4c1_max67_w <= s4e_w(7); s4c1_max67_ci <= s4e_ci(7);
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c2 : quad (4->2) sum and max — registered
            ----------------------------------------------------------------
            s4c2_sum_0123 <= resize(s4c1_sum_01, 10) + resize(s4c1_sum_23, 10);
            s4c2_sum_4567 <= resize(s4c1_sum_45, 10) + resize(s4c1_sum_67, 10);

            if s4c1_max01_w >= s4c1_max23_w then
                s4c2_max0123_w <= s4c1_max01_w; s4c2_max0123_ci <= s4c1_max01_ci;
            else
                s4c2_max0123_w <= s4c1_max23_w; s4c2_max0123_ci <= s4c1_max23_ci;
            end if;
            if s4c1_max45_w >= s4c1_max67_w then
                s4c2_max4567_w <= s4c1_max45_w; s4c2_max4567_ci <= s4c1_max45_ci;
            else
                s4c2_max4567_w <= s4c1_max67_w; s4c2_max4567_ci <= s4c1_max67_ci;
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c3 : final (2->1) sum + clamp + final max + palette
            ----------------------------------------------------------------
            total_w := resize(s4c2_sum_0123, 11) + resize(s4c2_sum_4567, 11)
                     + resize(bg3_w, 11);

            if s4c2_max0123_w >= s4c2_max4567_w then
                best_ci := s4c2_max0123_ci;
                max_w   := s4c2_max0123_w;
            else
                best_ci := s4c2_max4567_ci;
                max_w   := s4c2_max4567_w;
            end if;

            if total_w > to_unsigned(1023, 11) then
                s4c3_w <= (others => '1');
            else
                s4c3_w <= total_w(9 downto 0);
            end if;
            s4c3_ci <= best_ci;
            if    max_w >= to_unsigned(160, 8) then s4c3_a <= "100";
            elsif max_w >= to_unsigned(96, 8)  then s4c3_a <= "011";
            elsif max_w >= to_unsigned(48, 8)  then s4c3_a <= "010";
            elsif max_w >= to_unsigned(16, 8)  then s4c3_a <= "001";
            else                                    s4c3_a <= "000";
            end if;

            ----------------------------------------------------------------
            -- STAGE 4m1 : register the brightness-multiplier operands and
            -- perform the shimmer-palette EBR read (this register is the
            -- EBR output register; all address bits come from registers).
            -- Per-entry luma dim (silver, gold, amber...) scales the
            -- brightness operand for STAR pixels only -- with no star the
            -- winning colour index is a random hash bit, and dimming the
            -- nebula by it would be noise.
            ----------------------------------------------------------------
            s4m1_w  <= s4c3_w;
            dim_v := C_PALDIM(to_integer(pal_sel_r & s4c3_ci));
            if s4c3_a = "000" or bw_r = '1' then
                dim_v := "00";
            end if;
            case dim_v is
                when "01"   => s4m1_bright <= bright_s_r - shift_right(bright_s_r, 3);
                when "10"   => s4m1_bright <= bright_s_r - shift_right(bright_s_r, 2);
                when "11"   => s4m1_bright <= shift_right(bright_s_r, 1);
                when others => s4m1_bright <= bright_s_r;
            end case;
            s4m1_a      <= s4c3_a;
            paddr_v     := shim_cnt(15 downto 12) & pal_sel_r & s4c3_ci;
            s4m1_pal    <= C_SHIMPAL(to_integer(paddr_v));

            ----------------------------------------------------------------
            -- STAGE 4m2 : shared 10x10 brightness multiply, product
            -- registered here (valid two cycles after the operands).
            -- Shimmer palette data is registered off the EBR read, with
            -- the B&W override.
            ----------------------------------------------------------------
            prod_v := s4m1_w * s4m1_bright;
            s4m2_y <= prod_v(19 downto 8);
            s4m2_a  <= s4m1_a;
            if bw_r = '1' then
                s4m2_u <= to_unsigned(512, 10);
                s4m2_v <= to_unsigned(512, 10);
            else
                s4m2_u <= unsigned(s4m1_pal(19 downto 10));
                s4m2_v <= unsigned(s4m1_pal(9 downto 0));
            end if;

            ----------------------------------------------------------------
            -- STAGE 5a : star colour over black. Y = 64 + product, capped
            -- at 768 (black is 64; full-scale Y clips RGB and kills star
            -- chroma on hardware). Chroma = palette on a star, deep-space
            -- tint / neutral off it (B&W forces neutral).
            ----------------------------------------------------------------
            if s4m2_y > to_unsigned(704, 12) then
                s5a_y <= to_unsigned(768, 10);
            else
                s5a_y <= to_unsigned(64, 10) + s4m2_y(9 downto 0);
            end if;
            if s4m2_a /= "000" then
                s5a_u <= s4m2_u;
                s5a_v <= s4m2_v;
            elsif bg_on_r = '1' and bw_r = '0' then
                s5a_u <= to_unsigned(430, 10);
                s5a_v <= to_unsigned(620, 10);
            else
                s5a_u <= to_unsigned(512, 10);
                s5a_v <= to_unsigned(512, 10);
            end if;
            s5a_a <= s4m2_a;

            ----------------------------------------------------------------
            -- STAGE 5b : under-layer + differences. Video (T8): the delayed
            -- input picture, with a faint blue-violet cast under Deep
            -- Space. Off: black with alpha forced to 1, so the same mix
            -- path renders the plain starfield.
            ----------------------------------------------------------------
            if video_on_r = '1' then
                vy := unsigned(vdly_q2(29 downto 20));
                vu := unsigned(vdly_q2(19 downto 10));
                vv := unsigned(vdly_q2(9 downto 0));
                if bg_on_r = '1' and bw_r = '0' then
                    if vu > to_unsigned(20, 10) then
                        vu := vu - to_unsigned(20, 10);
                    end if;
                    if vv < to_unsigned(996, 10) then
                        vv := vv + to_unsigned(27, 10);
                    end if;
                end if;
                s5b_a <= s5a_a;
            else
                vy := to_unsigned(64, 10);
                vu := to_unsigned(512, 10);
                vv := to_unsigned(512, 10);
                s5b_a <= "100";
            end if;
            s5b_vy <= vy;
            s5b_vu <= vu;
            s5b_vv <= vv;
            s5b_dy <= signed(resize(s5a_y, 11)) - signed(resize(vy, 11));
            s5b_du <= signed(resize(s5a_u, 11)) - signed(resize(vu, 11));
            s5b_dv <= signed(resize(s5a_v, 11)) - signed(resize(vv, 11));

            ----------------------------------------------------------------
            -- STAGE 5c : alpha mix + blanking gate. Outside the latency-
            -- aligned avid the output is neutral black.
            ----------------------------------------------------------------
            if pipe(LATENCY - 2).avid = '0' then
                s5_y <= to_unsigned(64, 10);
                s5_u <= to_unsigned(512, 10);
                s5_v <= to_unsigned(512, 10);
            else
                s5_y <= f_mix(s5b_vy, s5b_dy, s5b_a);
                s5_u <= f_mix(s5b_vu, s5b_du, s5b_a);
                s5_v <= f_mix(s5b_vv, s5b_dv, s5b_a);
            end if;

            ----------------------------------------------------------------
            -- OUTPUT
            ----------------------------------------------------------------
            s_io_align_in    <= pipe(LATENCY - 1);
            s_io_align_in.y  <= std_logic_vector(s5_y);
            s_io_align_in.u  <= std_logic_vector(s5_u);
            s_io_align_in.v  <= std_logic_vector(s5_v);

        end if;
    end process;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (19 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture starfield;
