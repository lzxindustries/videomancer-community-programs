-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Warpfield: starfield's radial sibling -- stars fly straight at you
-- (or away from you) instead of scrolling sideways.
--
-- Engine: 6 zoom SHELLS instead of starfield's 8 parallax layers. Each
-- shell samples the same cell-hash star field as starfield, but in a
-- scaled coordinate space S = BIAS + (pixel - centre) * k, where the
-- per-shell zoom step k sweeps one octave (4096 -> 2048 in 4.12 fixed
-- point) per cycle. As k shrinks, a fixed star in scaled space moves
-- radially outward from the centre with speed proportional to its
-- distance -- the classic warp field -- and its glow kernel (sampled in
-- scaled space) magnifies 1x-2x on screen for free. Shell phases sit 1/6
-- apart; when a shell finishes its octave it re-seeds (new population,
-- drawn far) while its brightness envelope passes through zero, so the
-- flow never pops. The zoom phase drives the glow-kernel level (per-star
-- dither blends adjacent levels for smooth growth) and a brightness ramp
-- per level is baked into the kernel ROM.
--
-- Per-pixel the shell coordinates are pure accumulators (+k per pixel,
-- +k per active line). All frame set-up (quadratic speed curve, centre
-- drift Lissajous via parabola sine, centre*k line origins, and a linear
-- interpolation of the exp2 zoom table -- 256 raw steps would give
-- visible radial judder at slow warp) runs through ONE shared 13x13
-- multiplier in a vblank sequencer. Rendering downstream of the hash
-- (kernel ROMs in EBR, twinkle, shimmer palettes, nebula background,
-- luma mode) is inherited from starfield; T8 overlays the stars on the
-- input picture (alpha 'over' composite: opaque cores, transparent glow
-- edges, alpha from the star's kernel weight). 6 shells instead of 8 is the
-- fit budget: the render path scales linearly in shells and 8 was 115%
-- of the HX4K.
--
-- v1.1.3 (2026-10-17): K1/K4 labelled defaults (community fix, folded in):
-- the firmware loads a labelled default as its label INDEX, so K1 below 6
-- and K4 below 8 are taken as the label, not a knob position.
--
-- v1.1.4 (2026-10-17): resolution scaling. The shell coordinates run in
-- virtual 1080-line units, stepped from the video timing ID
-- (registers_in(8)): the per-pixel shell step is k * (1.0 / 1.5 / 2.0 at
-- 1080 / 720p / SD) and the per-line step k * (1080 / active lines, doubled
-- for interlace), both made by the vblank sequencer's shared multiplier; the
-- measured width/height (centre, drift) and the nebula raster count in the
-- same units. At 1080p every step is exactly 1.0 (identical to before); at
-- 720p and SD the cells, stars and nebula keep their size relative to the
-- screen height, and stars stay round on a 4:3 screen (SD = a 1440-wide
-- window of the 1080 canvas).
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Layers     (6 zones: 1..6 shells)
--   registers_in(1) = Density    (0-1023, manual density / luma gain)
--   registers_in(2) = Brightness (0-1023, linear multiply)
--   registers_in(3) = Palette (bits 9:7 = Silver/Candle/Twilight/Neon/
--                     Ember/Honey/Rainbow/B&W -- 8 distinct colour sets)
--   registers_in(4) = Size       (adds 0..3 glow levels, dithered)
--   registers_in(5) = Drift      (vanishing-point Lissajous amplitude)
--   registers_in(6) = Switches   (bit0=Dir bit1=Video bit2=Luma bit3=BG bit4=Gate)
--                     Video = stars composited OVER the input picture
--                     Lively mood (deeper, faster twinkle/shimmer) = Warp >= 50%
--   registers_in(7) = Warp       (P12 slider: speed, quadratic curve)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.warpfield_rom_pkg.all;
use work.all;

architecture warpfield of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (18 + 2 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY   : natural := 17;
    constant NUM_LYRS  : natural := 6;

    -- Scaled-space origin bias keeps shell coordinates unsigned:
    -- S spans BIAS +- ~5.9M for the worst-case centre/zoom, inside 24b.
    constant C_BIAS : unsigned(23 downto 0) := to_unsigned(8388608, 24);

    type t_pipe    is array (natural range <>) of t_video_stream_yuv444_30b;
    type t_u24_arr is array(0 to NUM_LYRS-1) of unsigned(23 downto 0);
    type t_u16_arr is array(0 to NUM_LYRS-1) of unsigned(15 downto 0);
    type t_u15_arr is array(0 to NUM_LYRS-1) of unsigned(14 downto 0);
    type t_u14_arr is array(0 to NUM_LYRS-1) of unsigned(13 downto 0);
    type t_u8_arr  is array(0 to NUM_LYRS-1) of unsigned(7 downto 0);
    type t_u5_arr  is array(0 to NUM_LYRS-1) of unsigned(4 downto 0);
    type t_u4_arr  is array(0 to NUM_LYRS-1) of unsigned(3 downto 0);
    type t_u3_arr  is array(0 to NUM_LYRS-1) of unsigned(2 downto 0);
    type t_u2_arr  is array(0 to NUM_LYRS-1) of unsigned(1 downto 0);

    constant SEEDS : t_u16_arr := (
        x"A3B1", x"7C2D", x"D917", x"4E6F", x"B5C3", x"62E8"
    );

    -- Shell phase offsets: i * 65536/6 (one octave = 65536 phase counts)
    constant C_LOFF : t_u24_arr := (
        to_unsigned(0, 24),     to_unsigned(10923, 24),
        to_unsigned(21845, 24), to_unsigned(32768, 24),
        to_unsigned(43691, 24), to_unsigned(54613, 24)
    );

    -- Sub-cell star position map: 4..11 only, so a kernel of reach 4 can
    -- NEVER cross the cell edge (offsets 2..13 sliced big stars flat at
    -- cell borders -- the neighbouring cell draws only its own star).
    type t_offmap is array (0 to 15) of unsigned(3 downto 0);
    constant C_OFFMAP : t_offmap := (
        x"4", x"4", x"5", x"5", x"6", x"6", x"7", x"7",
        x"8", x"8", x"9", x"9", x"A", x"A", x"B", x"B"
    );

    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Video delay for the overlay: an EBR circular buffer, NOT the FF pipe.
    -- Only pipe(0..3).y and the syncs are otherwise read, so yosys prunes
    -- the pipe's y/u/v chains; reading pipe(14).u/v made ~450 FFs live and
    -- overflowed the HX4K (7722 LC). Read pointer = write pointer - 13 so
    -- that, after the EBR output register and one fabric re-register, the
    -- sample lands at stage 5b (two before the output register) aligned
    -- with the star path (LATENCY 17).
    type t_vdly is array (0 to 15) of std_logic_vector(31 downto 0);
    signal vdly_ram : t_vdly := (others => (others => '0'));
    signal vdly_wa  : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal vdly_ra  : unsigned(3 downto 0) := to_unsigned(3, 4);
    signal vdly_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal vdly_q2  : std_logic_vector(31 downto 0) := (others => '0');

    -- Raster tracking / measurement (width from the avid run, height from
    -- active-line count -- cascade's pattern; both halved for the centre).
    -- Resolution scaling: all of it counts virtual 1080-line units. s_xs is
    -- the x step in half units per pixel (2/3/4 = 1.0/1.5/2.0), s_ys the y
    -- step in 1/16 units per FIELD line. s_xcnt (Q.1) also serves as the
    -- nebula's pixel position (it replaces the old pixel_x): reset at hsync,
    -- so it holds the line's final count from avid fall to hsync, which the
    -- ~7-pixel-early nebula path samples at the right edge.
    signal s_xs         : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys         : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal pix_ya       : unsigned(15 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal s_sawact     : std_logic := '0';   -- active video seen since last frame start
    signal s_avid_q     : std_logic := '0';
    signal s_xcnt       : unsigned(12 downto 0) := (others => '0');
    signal s_lcnt       : unsigned(15 downto 0) := (others => '0');
    signal halfw_r      : unsigned(10 downto 0) := to_unsigned(640, 11);
    signal cyh_r        : unsigned(10 downto 0) := to_unsigned(240, 11);

    -- Global zoom phase: ph24(15:8) is the base shell phase, one octave
    -- per 65536 counts; shells add C_LOFF so bits 19:16 of the per-shell
    -- sum are its re-seed epoch (changes exactly when the shell wraps).
    signal ph24  : unsigned(23 downto 0) := (others => '0');

    -- Per-shell frame constants published by the vblank sequencer: the
    -- per-pixel shell step k16 = k * x step and the per-line step ky = k *
    -- y step (both exactly k at 1080p)
    signal k16     : t_u14_arr := (others => to_unsigned(4096, 14));
    signal ky      : t_u15_arr := (others => to_unsigned(4096, 15));
    -- line origins x0 live in one EBR (8 x 32, shells 0-5): written by the
    -- vblank sequencer, read back one shell per clock after each hsync to
    -- reload the horizontal accumulators (x0_walk 0..5, 6 = idle), long
    -- before the active line starts
    type t_x0ram is array (0 to 7) of std_logic_vector(31 downto 0);
    signal x0_ram  : t_x0ram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of x0_ram : signal is true;
    signal x0_rd   : std_logic_vector(31 downto 0) := (others => '0');
    signal x0_we   : std_logic := '0';
    signal x0_wa   : unsigned(2 downto 0) := (others => '0');
    signal x0_wd   : unsigned(23 downto 0) := (others => '0');
    signal x0_walk : unsigned(2 downto 0) := to_unsigned(6, 3);
    signal x0_ra   : unsigned(2 downto 0) := (others => '0');
    signal x0_v1, x0_v2 : std_logic := '0';
    signal x0_j1, x0_j2 : unsigned(2 downto 0) := (others => '0');
    signal epoch4  : t_u4_arr  := (others => (others => '0'));
    signal env_l   : t_u5_arr  := (others => (others => '0'));  -- shell fade 0..16
    signal lint_l  : t_u3_arr  := (others => (others => '0'));  -- level int 0..7
    signal gfrac_l : t_u8_arr  := (others => (others => '0'));  -- level dither frac

    -- Per-shell live coordinate accumulators (the whole per-pixel cost of
    -- the zoom: one 24-bit add per shell per pixel / per active line)
    signal sx_acc : t_u24_arr := (others => (others => '0'));
    signal sy_acc : t_u24_arr := (others => (others => '0'));

    -- Vblank sequencer (one step every 4 clocks) + shared 13x13 serial
    -- multiplier (13 shift-add steps; started by s_mgo when a step writes
    -- s_ma/s_mb, done within the 16 clocks before the product is read)
    signal s_seq    : unsigned(8 downto 0) := (others => '0');
    signal s_run    : std_logic := '0';
    signal s_ce     : unsigned(1 downto 0) := (others => '0');
    signal s_mgo    : std_logic := '0';
    signal s_mcnt   : unsigned(3 downto 0) := (others => '0');
    signal s_ma     : unsigned(12 downto 0) := (others => '0');
    signal s_mb     : unsigned(12 downto 0) := (others => '0');
    signal s_mprod  : unsigned(25 downto 0) := (others => '0');
    signal step12   : unsigned(11 downto 0) := (others => '0');
    signal speed10_r : unsigned(9 downto 0) := (others => '0');
    signal amp10_r   : unsigned(9 downto 0) := (others => '0');
    signal size10_r  : unsigned(9 downto 0) := (others => '0');
    signal size_eff_r : unsigned(9 downto 0) := (others => '0');  -- size * 3/4: adds 0..3 levels
    signal p24_i    : unsigned(23 downto 0) := (others => '0');
    signal phl8_r   : unsigned(7 downto 0) := (others => '0');
    signal kfrac_r  : unsigned(7 downto 0) := (others => '0');
    signal t0_r     : unsigned(12 downto 0) := (others => '0');
    signal exp_a    : unsigned(7 downto 0) := (others => '0');
    signal exp_d    : std_logic_vector(12 downto 0) := (others => '0');

    -- Centre drift: two slow phase accumulators (Lissajous), parabola
    -- sine via the shared multiplier, amplitude by K6, scaled by the
    -- measured half-dimensions so the wander is aspect-proportional.
    signal dphx  : unsigned(15 downto 0) := (others => '0');
    signal dphy  : unsigned(15 downto 0) := (others => '0');
    signal sgx_r, sgy_r : std_logic := '0';
    signal sxp_r, syp_r : unsigned(10 downto 0) := (others => '0');
    signal gx_r,  gy_r  : unsigned(9 downto 0)  := (others => '0');
    signal cx_r  : unsigned(10 downto 0) := to_unsigned(640, 11);
    signal cy_r  : unsigned(10 downto 0) := to_unsigned(240, 11);

    -- Registered knob helpers (threshold_r pattern from starfield)
    signal threshold_r : unsigned(12 downto 0) := (others => '0');
    signal thr_base_r  : unsigned(9 downto 0)  := (others => '0');
    signal thr_gain_r  : unsigned(4 downto 0)  := (others => '0');
    signal gate_r      : std_logic := '0';
    signal luma_mode_r : std_logic := '0';
    signal first_act_r : unsigned(2 downto 0) := (others => '0');
    signal pal_sel_r   : unsigned(2 downto 0) := (others => '0');
    signal bw_r        : std_logic := '0';
    signal bg_on_r     : std_logic := '0';
    signal dir_r       : std_logic := '0';
    signal mood_r      : std_logic := '0';
    signal video_on_r  : std_logic := '0';
    signal tw_rate_r   : unsigned(2 downto 0) := (others => '0');
    signal shim_rate_r : unsigned(7 downto 0) := (others => '0');

    -- Twinkle / shimmer animation state (advance once per field)
    signal tw_phase : unsigned(7 downto 0)  := (others => '0');
    signal shim_cnt : unsigned(15 downto 0) := (others => '0');

    -- Deep-space background (starfield's nebula, fixed slow drift)
    signal neb_scroll : unsigned(15 downto 0) := (others => '0');
    signal bg1_addr   : unsigned(11 downto 0) := (others => '0');
    signal bg2_neb    : std_logic_vector(7 downto 0) := (others => '0');
    signal bg2_grad   : unsigned(6 downto 0) := (others => '0');
    signal bg3_w      : unsigned(7 downto 0) := (others => '0');

    -- Stage 3a: hash pre-multiply (cell concat XOR seed XOR epoch)
    signal s3a_hx    : t_u8_arr := (others => (others => '0'));
    -- the hash input is (x cell, y cell): (hx*256 + hy) * 0x9E37 mod 2^16
    -- = ((hx * 0x37 + Y_hi) mod 256) & Y_lo with Y = hy * 0x9E37 mod 2^16,
    -- and hy is constant along a line -- so Y is computed once per line per
    -- shell (in the hsync walk) and only an 8-bit multiply-add stays per pixel
    signal y_hash    : t_u16_arr := (others => (others => '0'));
    signal yh_a      : unsigned(15 downto 0) := (others => '0');
    signal yh_p1, yh_p2 : unsigned(15 downto 0) := (others => '0');
    signal s3a_sub_x : t_u4_arr;
    signal s3a_sub_y : t_u4_arr;

    -- Stage 3: hash finalised (after multiply + xor-shift)
    signal s3_hash  : t_u16_arr;
    signal s3_sub_x : t_u4_arr;
    signal s3_sub_y : t_u4_arr;

    -- Stage 3p: per-shell star properties straight off the hash
    signal s3p_offset_x   : t_u4_arr;
    signal s3p_offset_y   : t_u4_arr;
    signal s3p_sub_x      : t_u4_arr;
    signal s3p_sub_y      : t_u4_arr;
    signal s3p_shape      : t_u2_arr;
    signal s3p_color_idx  : t_u2_arr;
    signal s3p_mag        : t_u2_arr;
    signal s3p_dith       : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s3p_lboost     : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s3p_pt         : t_u8_arr;
    signal s3p_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');

    -- Stage 4a1: glow-level combine + spike-offset clamp + envelope
    signal s4a1_offset_x   : t_u4_arr;
    signal s4a1_offset_y   : t_u4_arr;
    signal s4a1_sub_x      : t_u4_arr;
    signal s4a1_sub_y      : t_u4_arr;
    signal s4a1_shape      : t_u2_arr;
    signal s4a1_lvl        : t_u2_arr;
    signal s4a1_color_idx  : t_u2_arr;
    signal s4a1_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4a1_env        : t_u5_arr;

    -- Stage 4ax: |sub - offset| per axis with in-range kill
    signal s4ax_adx        : t_u3_arr;
    signal s4ax_ady        : t_u3_arr;
    signal s4ax_shape      : t_u2_arr;
    signal s4ax_lvl        : t_u2_arr;
    signal s4ax_color_idx  : t_u2_arr;
    signal s4ax_hit        : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4ax_env        : t_u5_arr;

    -- Stage 4k0/4k1: kernel-ROM read + registered/masked read data
    signal s4k0_w   : t_u8_arr := (others => (others => '0'));
    signal s4k0_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4k0_ci  : t_u2_arr;
    signal s4k0_env : t_u5_arr;
    signal s4k1_w   : t_u8_arr := (others => (others => '0'));
    signal s4k1_ci  : t_u2_arr;
    signal s4k1_env : t_u5_arr;

    -- Stage 4e: per-shell envelope multiply (8x5)
    signal s4e_w  : t_u8_arr := (others => (others => '0'));
    signal s4e_ci : t_u2_arr;

    -- Stage 4c1..4c3: sum/max composite trees (6 shells: 3 pairs -> final)
    signal s4c1_sum_01, s4c1_sum_23, s4c1_sum_45 : unsigned(8 downto 0) := (others => '0');
    signal s4c1_max01_w, s4c1_max23_w, s4c1_max45_w    : unsigned(7 downto 0) := (others => '0');
    signal s4c1_max01_ci, s4c1_max23_ci, s4c1_max45_ci : unsigned(1 downto 0) := (others => '0');
    signal s4c2_sum_0123, s4c2_sum_45d        : unsigned(9 downto 0) := (others => '0');
    signal s4c2_max0123_w, s4c2_max45_w       : unsigned(7 downto 0) := (others => '0');
    signal s4c2_max0123_ci, s4c2_max45_ci     : unsigned(1 downto 0) := (others => '0');
    signal s4c3_w  : unsigned(9 downto 0) := (others => '0');
    signal s4c3_ci : unsigned(1 downto 0) := (others => '0');
    -- alpha band from the brightest star's kernel weight: 4 = opaque,
    -- 3/2/1 = 3/4, 1/2, 1/4 (the glow edges), 0 = no star
    signal s4c3_a  : unsigned(2 downto 0) := (others => '0');
    signal s4m1_a  : unsigned(2 downto 0) := (others => '0');
    signal s4m2_a  : unsigned(2 downto 0) := (others => '0');
    signal bright_s_r : unsigned(9 downto 0) := (others => '0');  -- Brightness * 11/16

    -- Stage 4m1/4m2: shared 10x10 brightness multiply + shimmer palette
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

    -- Resolution scaling: per-mode steps from the video timing ID.
    -- x in half units per pixel: 1080 2 (1.0), 720p 3 (1.5), SD 4 (2.0:
    -- 720 px span 1440 units, a 4:3 window of the 1080 canvas).
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y in 1/16 units per FIELD line (1080 / active lines, doubled for
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

    pixel_y <= pix_ya(15 downto 4);

    -- line-origin RAM: one write port, one registered read
    p_x0ram : process(clk)
    begin
        if rising_edge(clk) then
            if x0_we = '1' then
                x0_ram(to_integer(x0_wa)) <= "00000000" & std_logic_vector(x0_wd);
            end if;
            x0_rd <= x0_ram(to_integer(x0_ra));
        end if;
    end process;

    process(clk)
        -- Controls
        variable v_k1          : unsigned(9 downto 0);
        variable density       : unsigned(9 downto 0);
        variable palette_sel   : unsigned(2 downto 0);
        variable bright        : unsigned(9 downto 0);
        variable dim_v         : std_logic_vector(1 downto 0);
        variable v_diff_y      : unsigned(9 downto 0);
        variable v_thr_prod    : unsigned(14 downto 0);

        -- Sequencer
        variable v_li   : integer range 0 to NUM_LYRS-1;
        variable v_tick : integer range 0 to 31;
        variable v_pos  : unsigned(11 downto 0);
        variable v_msum : unsigned(13 downto 0);
        variable v_phl8 : unsigned(7 downto 0);
        variable v_f9   : unsigned(8 downto 0);
        variable v_t1   : unsigned(12 downto 0);
        variable v_k    : unsigned(12 downto 0);

        -- Stage 3/4 per-shell
        variable hash_tmp      : unsigned(15 downto 0);
        variable v_hx8         : unsigned(7 downto 0);
        variable v_wj          : integer range 0 to NUM_LYRS-1;
        variable hash_v        : unsigned(15 downto 0);
        variable offset_x      : unsigned(3 downto 0);
        variable offset_y      : unsigned(3 downto 0);
        variable dith8         : unsigned(7 downto 0);
        variable frac9         : unsigned(8 downto 0);
        variable lvl_i         : integer range -3 to 13;
        variable d5            : signed(4 downto 0);
        variable inr_x         : boolean;
        variable inr_y         : boolean;
        variable addr_v        : unsigned(9 downto 0);
        variable layer_active  : boolean;

        -- Twinkle envelope
        variable pt_v          : unsigned(7 downto 0);
        variable tri7          : unsigned(6 downto 0);
        variable depth4        : unsigned(3 downto 0);
        variable env_t         : unsigned(4 downto 0);
        variable env_v         : unsigned(4 downto 0);
        variable prod13        : unsigned(12 downto 0);
        variable prod12        : unsigned(11 downto 0);

        -- Background path
        variable sum12         : unsigned(11 downto 0);

        -- Composite
        variable vy, vu, vv    : unsigned(9 downto 0);
        variable total_w       : unsigned(10 downto 0);
        variable best_ci       : unsigned(1 downto 0);
        variable max_w         : unsigned(7 downto 0);
        variable paddr_v       : unsigned(8 downto 0);
        variable prod_v        : unsigned(19 downto 0);
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
            -- Raster tracking + measurement + shell coordinate accumulators
            ----------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            s_avid_q     <= data_in.avid;

            -- virtual-unit steps, free-running (the timing ID is static
            -- while a mode runs)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_xcnt  <= s_xcnt + s_xs;
                for i in 0 to NUM_LYRS-1 loop
                    sx_acc(i) <= sx_acc(i) + resize(k16(i), 24);
                end loop;
            elsif s_avid_q = '1' then
                -- avid just fell: close the line measurement, step the
                -- vertical shell coordinates (ACTIVE lines only)
                if s_xcnt(12 downto 1) > to_unsigned(16, 12) then
                    halfw_r <= s_xcnt(12 downto 2);
                end if;
                s_lcnt <= s_lcnt + s_ys;
                for i in 0 to NUM_LYRS-1 loop
                    sy_acc(i) <= sy_acc(i) + resize(ky(i), 24);
                end loop;
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                s_xcnt  <= (others => '0');
                pix_ya  <= pix_ya + s_ys;
                x0_walk <= (others => '0');
            end if;

            -- line-origin reload: address, EBR read, then the load; the
            -- hash's per-line half Y rides the same walk (y cell, two adder
            -- stages, then the write)
            if x0_walk <= NUM_LYRS-1 then
                v_wj := to_integer(x0_walk);
            else
                v_wj := 0;
            end if;
            yh_a  <= resize(sy_acc(v_wj)(23 downto 16) xor SEEDS(v_wj)(7 downto 0)
                            xor (epoch4(v_wj) & epoch4(v_wj)), 16);
            yh_p1 <= shift_left(yh_a, 15) + shift_left(yh_a, 13) - shift_left(yh_a, 9);
            yh_p2 <= shift_left(yh_a, 6) - shift_left(yh_a, 3) - yh_a;
            if x0_v2 = '1' then
                for i in 0 to NUM_LYRS-1 loop
                    if to_integer(x0_j2) = i then
                        y_hash(i) <= yh_p1 + yh_p2;
                    end if;
                end loop;
            end if;
            if x0_walk <= NUM_LYRS-1 then
                x0_ra   <= x0_walk;
                x0_j1   <= x0_walk;
                x0_v1   <= '1';
                x0_walk <= x0_walk + 1;
            else
                x0_v1   <= '0';
            end if;
            x0_v2 <= x0_v1;
            x0_j2 <= x0_j1;
            if x0_v2 = '1' then
                for i in 0 to NUM_LYRS-1 loop
                    if to_integer(x0_j2) = i then
                        sx_acc(i) <= unsigned(x0_rd(23 downto 0));
                    end if;
                end loop;
            end if;
            x0_we <= '0';

            ----------------------------------------------------------------
            -- Vblank sequencer: at vsync, latch knobs and start; then one
            -- small op every few clocks through the shared multiplier.
            -- Preamble (seq 0..47): speed curve, phase update, centre drift.
            -- Shell slots (seq 64 + 32*i): phase/epoch, zoom k from the
            -- exp2 EBR with linear interpolation, line origin x0 and the
            -- vertical accumulator seed, level/dither/fade envelope.
            -- Done at 256, well before the first active line.
            ----------------------------------------------------------------
            -- Frame start = the FIRST vsync edge after active video: analog
            -- vsync serrates (several edges per field), and every per-field
            -- step below must happen exactly once.
            s_ce  <= s_ce + 1;
            s_mgo <= '0';
            if data_in.avid = '1' then
                s_sawact <= '1';
            end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' and s_sawact = '1' then
                s_sawact <= '0';
                s_seq <= (others => '0');
                s_run <= '1';
                speed10_r <= unsigned(registers_in(7));
                amp10_r   <= unsigned(registers_in(5));
                size10_r  <= unsigned(registers_in(4));

                -- slow Lissajous wander (~29.5 s / ~47.5 s periods)
                dphx <= dphx + to_unsigned(37, 16);
                dphy <= dphy + to_unsigned(23, 16);

                -- animation phases + gentle nebula drift
                tw_phase   <= tw_phase + resize(tw_rate_r, 8);
                shim_cnt   <= shim_cnt + resize(shim_rate_r, 16);
                neb_scroll <= neb_scroll + to_unsigned(8, 16);

                -- height measurement (counted lines per field; halved)
                if s_lcnt(15 downto 4) > to_unsigned(8, 12) then
                    cyh_r <= s_lcnt(15 downto 5);
                end if;
                s_lcnt  <= (others => '0');
                pix_ya  <= (others => '0');
            elsif s_run = '1' and s_ce = 3 then
                s_seq <= s_seq + 1;
                if s_seq < to_unsigned(48, 9) then
                    case to_integer(s_seq) is
                        when 0 =>
                            -- quadratic speed curve: step = speed^2 / 256
                            s_ma <= resize(speed10_r, 13);
                            s_mb <= resize(speed10_r, 13);
                            s_mgo <= '1';
                        when 4 =>
                            step12 <= s_mprod(19 downto 8);
                        when 6 =>
                            if dir_r = '0' then
                                ph24 <= ph24 + resize(step12, 24);
                            else
                                ph24 <= ph24 - resize(step12, 24);
                            end if;
                        when 8 =>
                            -- parabola sine, X axis: quarter-fold the phase
                            if dphx(14) = '1' then
                                v_pos := not dphx(13 downto 2);
                            else
                                v_pos := dphx(13 downto 2);
                            end if;
                            s_ma  <= '0' & v_pos;
                            s_mb  <= to_unsigned(4096, 13) - resize(v_pos, 13);
                            s_mgo <= '1';
                            sgx_r <= dphx(15);
                        when 12 =>
                            sxp_r <= s_mprod(22 downto 12);
                            if dphy(14) = '1' then
                                v_pos := not dphy(13 downto 2);
                            else
                                v_pos := dphy(13 downto 2);
                            end if;
                            s_ma  <= '0' & v_pos;
                            s_mb  <= to_unsigned(4096, 13) - resize(v_pos, 13);
                            s_mgo <= '1';
                            sgy_r <= dphy(15);
                        when 16 =>
                            syp_r <= s_mprod(22 downto 12);
                            s_ma  <= resize(sxp_r, 13);
                            s_mb  <= resize(amp10_r, 13);
                            s_mgo <= '1';
                        when 20 =>
                            gx_r <= s_mprod(20 downto 11);
                            s_ma <= resize(syp_r, 13);
                            s_mb <= resize(amp10_r, 13);
                            s_mgo <= '1';
                        when 24 =>
                            gy_r <= s_mprod(20 downto 11);
                            s_ma <= resize(gx_r, 13);
                            s_mb <= resize(halfw_r, 13);
                            s_mgo <= '1';
                        when 28 =>
                            -- drift <= half-dim/2, so the centre never
                            -- leaves the picture (and is 0 until measured)
                            if sgx_r = '1' then
                                cx_r <= halfw_r - resize(s_mprod(19 downto 10), 11);
                            else
                                cx_r <= halfw_r + resize(s_mprod(19 downto 10), 11);
                            end if;
                            s_ma <= resize(gy_r, 13);
                            s_mb <= resize(cyh_r, 13);
                            s_mgo <= '1';
                        when 32 =>
                            if sgy_r = '1' then
                                cy_r <= cyh_r - resize(s_mprod(19 downto 10), 11);
                            else
                                cy_r <= cyh_r + resize(s_mprod(19 downto 10), 11);
                            end if;
                        when others => null;
                    end case;
                elsif s_seq >= to_unsigned(64, 9) and s_seq < to_unsigned(256, 9) then
                    v_li   := to_integer(s_seq(8 downto 5) - 2);
                    v_tick := to_integer(s_seq(4 downto 0));
                    case v_tick is
                        when 0 =>
                            p24_i <= ph24 + C_LOFF(v_li);
                        when 2 =>
                            v_phl8 := p24_i(15 downto 8);
                            exp_a   <= v_phl8;
                            phl8_r  <= v_phl8;
                            kfrac_r <= p24_i(7 downto 0);
                            -- re-seed epoch (XOR-folded into the hash at 3a)
                            epoch4(v_li) <= p24_i(19 downto 16);
                            -- shell fade envelope: in over the first 32
                            -- phase counts, out over the last 32 (16 was one
                            -- field at full Warp -- stars popped)
                            if v_phl8(7 downto 5) = "000" then
                                env_l(v_li) <= '0' & v_phl8(4 downto 1);
                            elsif v_phl8(7 downto 5) = "111" then
                                env_l(v_li) <= '0' & not v_phl8(4 downto 1);
                            else
                                env_l(v_li) <= to_unsigned(16, 5);
                            end if;
                            -- glow level = depth phase + Size knob, in
                            -- integer + dither-fraction form
                            v_f9 := ('0' & v_phl8(5 downto 0) & "00")
                                  + resize(size_eff_r(7 downto 0), 9);
                            lint_l(v_li)  <= resize(v_phl8(7 downto 6), 3)
                                           + resize(size_eff_r(9 downto 8), 3)
                                           + ("00" & v_f9(8));
                            gfrac_l(v_li) <= v_f9(7 downto 0);
                        when 6 =>
                            t0_r  <= unsigned(exp_d);
                            exp_a <= phl8_r + 1;   -- wraps 255 -> 0 (guarded below)
                        when 10 =>
                            -- interpolate k between adjacent exp2 entries:
                            -- k = T0 - frac * (T0 - T1) / 256. T[256] is the
                            -- octave end, 2048, not table entry 0 (= 4096).
                            if phl8_r = x"FF" then
                                v_t1 := to_unsigned(2048, 13);
                            else
                                v_t1 := unsigned(exp_d);
                            end if;
                            s_ma <= resize(kfrac_r, 13);
                            s_mb <= t0_r - v_t1;   -- 2..12
                            s_mgo <= '1';
                        when 14 =>
                            v_k := t0_r - resize(s_mprod(12 downto 8), 13);
                            s_ma <= resize(cx_r, 13);
                            s_mb <= v_k;
                            s_mgo <= '1';
                        when 18 =>
                            x0_wd <= C_BIAS - resize(s_mprod(22 downto 0), 24);
                            x0_wa <= to_unsigned(v_li, 3);
                            x0_we <= '1';
                            -- s_mb holds this shell's k from tick 14 to 26
                            s_ma <= resize(cy_r, 13);
                            s_mgo <= '1';
                        when 22 =>
                            sy_acc(v_li) <= C_BIAS - resize(s_mprod(22 downto 0), 24);
                            -- per-line step: k * y step (Q.4)
                            s_ma <= resize(s_ys, 13);
                            s_mgo <= '1';
                        when 26 =>
                            ky(v_li) <= s_mprod(18 downto 4);
                            -- per-pixel step: k * x step (Q.1)
                            s_ma <= resize(s_xs, 13);
                            s_mgo <= '1';
                        when 30 =>
                            k16(v_li) <= s_mprod(14 downto 1);
                        when others => null;
                    end case;
                else
                    if s_seq = to_unsigned(256, 9) then
                        s_run <= '0';
                    end if;
                end if;
            end if;

            -- shared serial multiplier: s_mprod = [H | L], L preloaded with
            -- the multiplier; each step adds s_ma to H when L(0) is set and
            -- shifts the pair right one bit.  13 steps leave s_ma * s_mb.
            if s_mgo = '1' then
                s_mprod <= (25 downto 13 => '0') & s_mb;
                s_mcnt  <= to_unsigned(13, 4);
            elsif s_mcnt /= 0 then
                if s_mprod(0) = '1' then
                    v_msum := ('0' & s_mprod(25 downto 13)) + ('0' & s_ma);
                else
                    v_msum := '0' & s_mprod(25 downto 13);
                end if;
                s_mprod <= v_msum & s_mprod(12 downto 1);
                s_mcnt  <= s_mcnt - 1;
            end if;
            -- Size knob scaled to 0..3 added glow levels (free-running, so
            -- the vsync latch stays a plain register copy)
            size_eff_r <= size10_r - shift_right(size10_r, 2);
            -- Brightness * 11/16 (free-running): over black stars then peak
            -- at Y 768 -- full-scale Y clips RGB and kills star chroma
            bright := unsigned(registers_in(2));
            bright_s_r <= bright - shift_right(bright, 2) - shift_right(bright, 4);
            -- zoom table read (EBR: registered address, sync read,
            -- consumed into fabric registers a few ticks later)
            exp_d <= C_EXP2(to_integer(exp_a));

            ----------------------------------------------------------------
            -- Registered knob helpers (quasi-static, one cycle stale)
            ----------------------------------------------------------------
            density      := unsigned(registers_in(1));
            -- labelled defaults (K1, K4): the firmware loads the label INDEX,
            -- so a value below the label count is the label, not a position
            if unsigned(registers_in(3)(9 downto 3)) = 0 then
                palette_sel := unsigned(registers_in(3)(2 downto 0));
            else
                palette_sel := unsigned(registers_in(3)(9 downto 7));
            end if;

            -- K1 Layers: 6 zones -> first active shell index 5..0
            v_k1 := unsigned(registers_in(0));
            if    v_k1 < to_unsigned(6, 10) then first_act_r <= to_unsigned(5, 3) - v_k1(2 downto 0);
            elsif v_k1 < to_unsigned(171, 10) then first_act_r <= to_unsigned(5, 3);
            elsif v_k1 < to_unsigned(342, 10) then first_act_r <= to_unsigned(4, 3);
            elsif v_k1 < to_unsigned(512, 10) then first_act_r <= to_unsigned(3, 3);
            elsif v_k1 < to_unsigned(683, 10) then first_act_r <= to_unsigned(2, 3);
            elsif v_k1 < to_unsigned(853, 10) then first_act_r <= to_unsigned(1, 3);
            else                                   first_act_r <= to_unsigned(0, 3);
            end if;

            luma_mode_r <= registers_in(6)(2);
            pal_sel_r   <= palette_sel;
            if palette_sel = "111" then          -- K4 position 8 = B&W
                bw_r <= '1';
            else
                bw_r <= '0';
            end if;
            bg_on_r     <= registers_in(6)(3);
            dir_r       <= registers_in(6)(0);

            video_on_r <= registers_in(6)(1);
            -- Lively mood rides the Warp slider: top half = deeper, faster
            -- twinkle and faster shimmer (the slider's climax)
            mood_r <= speed10_r(9);
            if speed10_r(9) = '1' then
                tw_rate_r   <= to_unsigned(5, 3);
                shim_rate_r <= to_unsigned(128, 8);  -- hue orbit ~8.5 s
            else
                tw_rate_r   <= to_unsigned(2, 3);
                shim_rate_r <= to_unsigned(36, 8);   -- hue orbit ~30 s
            end if;

            -- Density threshold = base * gain / 2, two stages:
            --   manual : base = Density knob, gain 16  (= density * 8)
            --   luma   : base = input luma (Gate: (luma-256)*2 saturating,
            --            a steeper map), gain = Density(9:6) 0..15, so K2
            --            stays live as the luma gain
            gate_r <= registers_in(6)(4);
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
            -- STAGE 3a : cell hash pre-multiply. Shell cell coordinates
            -- come straight off the live accumulators: cell = S(23:16),
            -- sub-cell = S(15:12) (cells are 16 scaled units). The 4-bit
            -- re-seed epoch is replicated and XOR-folded with the constant
            -- shell seed (3-input XOR per bit, still one LUT).
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                s3a_hx(i) <= sx_acc(i)(23 downto 16) xor SEEDS(i)(15 downto 8)
                             xor (epoch4(i) & epoch4(i));
                s3a_sub_x(i) <= sx_acc(i)(15 downto 12);
                s3a_sub_y(i) <= sy_acc(i)(15 downto 12);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 3 : hash multiply + finalise
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                -- x 0x9E37 mod 2^16 in canonical signed-digit form
                -- (2^15 + 2^13 - 2^9 + 2^6 - 2^3 - 1): 5 adders instead of
                -- the 9 yosys makes of the 10-set-bit constant (-384 LUT)
                v_hx8     := shift_left(s3a_hx(i), 6) - shift_left(s3a_hx(i), 3)
                           - s3a_hx(i) + y_hash(i)(15 downto 8);
                hash_tmp  := v_hx8 & y_hash(i)(7 downto 0);
                hash_tmp  := hash_tmp xor shift_right(hash_tmp, 5);
                hash_tmp  := hash_tmp xor (hash_tmp(10 downto 0) & "00000");
                s3_hash(i)  <= hash_tmp;
                s3_sub_x(i) <= s3a_sub_x(i);
                s3_sub_y(i) <= s3a_sub_y(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 3p : per-shell star properties straight off the hash.
            -- Low hash bits carry the entropy for surviving stars (the
            -- presence test biases high bits toward zero) -- see starfield.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                layer_active := i >= to_integer(first_act_r);

                hash_v := s3_hash(i);
                s3p_offset_x(i) <= C_OFFMAP(to_integer(hash_v(3 downto 0)));
                s3p_offset_y(i) <= C_OFFMAP(to_integer(hash_v(7 downto 4)));
                s3p_shape(i)    <= hash_v(9 downto 8);
                s3p_color_idx(i) <= hash_v(5 downto 4) xor hash_v(9 downto 8);
                -- Level fraction = shell depth/Size fraction (+ in Luma
                -- Mode the pixel's luma, so bright video adds up to ONE
                -- level, ~+40% size at white, ~+20% at mid grey); the
                -- carry is a whole level, the rest dithers per star.
                dith8 := hash_v(11 downto 4) xor hash_v(7 downto 0);
                if luma_mode_r = '1' then
                    frac9 := resize(gfrac_l(i), 9)
                           + resize(unsigned(pipe(2).y(9 downto 2)), 9);
                else
                    frac9 := resize(gfrac_l(i), 9);
                end if;
                s3p_lboost(i) <= frac9(8);
                if dith8 < frac9(7 downto 0) then
                    s3p_dith(i) <= '1';
                else
                    s3p_dith(i) <= '0';
                end if;
                s3p_mag(i)  <= hash_v(7 downto 6) xor hash_v(1 downto 0);
                s3p_sub_x(i) <= s3_sub_x(i);
                s3p_sub_y(i) <= s3_sub_y(i);
                -- Manual-mode Gate thins the field to the two bold
                -- magnitude classes (mag(1) = hash(7) xor hash(1))
                if layer_active and hash_v < resize(threshold_r, 16)
                   and not (gate_r = '1' and luma_mode_r = '0'
                            and (hash_v(7) xor hash_v(1)) = '1') then
                    s3p_active_hit(i) <= '1';
                else
                    s3p_active_hit(i) <= '0';
                end if;

                -- Twinkle phase position: per-star offset + global phase,
                -- per-star 1x/2x rate select
                if (hash_v(10) xor hash_v(0)) = '1' then
                    pt_v := (dith8(3 downto 0) & dith8(7 downto 4))
                          + (tw_phase(6 downto 0) & '0');
                else
                    pt_v := (dith8(3 downto 0) & dith8(7 downto 4))
                          + tw_phase;
                end if;
                s3p_pt(i) <= pt_v;
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4a1 : glow-level combine.
            -- Level = shell depth phase + Size knob (integer part, with the
            -- per-star dither verdict blending adjacent levels, and the
            -- Luma-Mode luma carry) - per-star magnitude penalty, clamp 0..3.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                offset_x := s3p_offset_x(i);
                offset_y := s3p_offset_y(i);

                lvl_i := to_integer(lint_l(i));
                if s3p_dith(i) = '1' then
                    lvl_i := lvl_i + 1;
                end if;
                if s3p_lboost(i) = '1' then
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

                -- Twinkle envelope (triangle fold; Lively dips deeper)
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

                -- Combined envelope: twinkle x shell spawn/wrap fade
                if env_t < env_l(i) then
                    env_v := env_t;
                else
                    env_v := env_l(i);
                end if;

                s4a1_offset_x(i)  <= offset_x;
                s4a1_offset_y(i)  <= offset_y;
                s4a1_sub_x(i)     <= s3p_sub_x(i);
                s4a1_sub_y(i)     <= s3p_sub_y(i);
                s4a1_shape(i)     <= s3p_shape(i);
                s4a1_lvl(i)       <= to_unsigned(lvl_i, 2);
                s4a1_color_idx(i) <= s3p_color_idx(i);
                s4a1_active_hit(i) <= s3p_active_hit(i);
                s4a1_env(i)       <= env_v;
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4ax : |sub - offset| per axis with in-range kill
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                -- one signed subtract, then |d| <= 7 and |d| mod 8
                d5 := signed('0' & s4a1_sub_x(i)) - signed('0' & s4a1_offset_x(i));
                if d5(4) = '0' then
                    inr_x := d5(3) = '0';
                    s4ax_adx(i) <= unsigned(d5(2 downto 0));
                else
                    inr_x := d5(3) = '1' and d5(2 downto 0) /= "000";
                    s4ax_adx(i) <= unsigned(not d5(2 downto 0)) + 1;
                end if;

                d5 := signed('0' & s4a1_sub_y(i)) - signed('0' & s4a1_offset_y(i));
                if d5(4) = '0' then
                    inr_y := d5(3) = '0';
                    s4ax_ady(i) <= unsigned(d5(2 downto 0));
                else
                    inr_y := d5(3) = '1' and d5(2 downto 0) /= "000";
                    s4ax_ady(i) <= unsigned(not d5(2 downto 0)) + 1;
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
            -- STAGE 4k0 : kernel-ROM synchronous read (6 separately-named
            -- constants -- GHDL merges an array-of-arrays into one ROM and
            -- crashes, see starfield)
            ----------------------------------------------------------------
            addr_v := s4ax_lvl(0) & s4ax_shape(0) & s4ax_ady(0) & s4ax_adx(0);
            s4k0_w(0) <= unsigned(C_KERNEL_L0(to_integer(addr_v)));
            addr_v := s4ax_lvl(1) & s4ax_shape(1) & s4ax_ady(1) & s4ax_adx(1);
            s4k0_w(1) <= unsigned(C_KERNEL_L1(to_integer(addr_v)));
            addr_v := s4ax_lvl(2) & s4ax_shape(2) & s4ax_ady(2) & s4ax_adx(2);
            s4k0_w(2) <= unsigned(C_KERNEL_L2(to_integer(addr_v)));
            addr_v := s4ax_lvl(3) & s4ax_shape(3) & s4ax_ady(3) & s4ax_adx(3);
            s4k0_w(3) <= unsigned(C_KERNEL_L3(to_integer(addr_v)));
            addr_v := s4ax_lvl(4) & s4ax_shape(4) & s4ax_ady(4) & s4ax_adx(4);
            s4k0_w(4) <= unsigned(C_KERNEL_L4(to_integer(addr_v)));
            addr_v := s4ax_lvl(5) & s4ax_shape(5) & s4ax_ady(5) & s4ax_adx(5);
            s4k0_w(5) <= unsigned(C_KERNEL_L5(to_integer(addr_v)));
            for i in 0 to NUM_LYRS-1 loop
                s4k0_hit(i) <= s4ax_hit(i);
                s4k0_ci(i)  <= s4ax_color_idx(i);
                s4k0_env(i) <= s4ax_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4k1 : register the ROM data, masked by the hit flag
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
            -- STAGE 4e : per-shell envelope multiply (8x5, >> 4)
            ----------------------------------------------------------------
            -- (env is 0..16: 16 is the only value with bit 4 set, and
            -- passes w unchanged; the rest is an 8x4 multiply)
            for i in 0 to NUM_LYRS-1 loop
                if s4k1_env(i)(4) = '1' then
                    s4e_w(i) <= s4k1_w(i);
                else
                    prod12 := s4k1_w(i) * s4k1_env(i)(3 downto 0);
                    s4e_w(i) <= prod12(11 downto 4);
                end if;
                s4e_ci(i) <= s4k1_ci(i);
            end loop;

            ----------------------------------------------------------------
            -- BACKGROUND path: drifting nebula + vertical gradient (S10)
            ----------------------------------------------------------------
            sum12 := s_xcnt(12 downto 1) + neb_scroll(15 downto 4);
            bg1_addr <= pixel_y(7 downto 2) & sum12(7 downto 2);

            bg2_neb <= C_NEBULA(to_integer(bg1_addr));
            if pixel_y(11 downto 5) < to_unsigned(24, 7) then
                bg2_grad <= to_unsigned(24, 7) - resize(pixel_y(11 downto 5), 7);
            else
                bg2_grad <= (others => '0');
            end if;

            if bg_on_r = '1' then
                bg3_w <= resize(unsigned(bg2_neb(7 downto 2)), 8)
                       + resize(bg2_grad, 8);
            else
                bg3_w <= (others => '0');
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c1 : pairwise (6->3) sum and max
            ----------------------------------------------------------------
            s4c1_sum_01 <= resize(s4e_w(0), 9) + resize(s4e_w(1), 9);
            s4c1_sum_23 <= resize(s4e_w(2), 9) + resize(s4e_w(3), 9);
            s4c1_sum_45 <= resize(s4e_w(4), 9) + resize(s4e_w(5), 9);

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

            ----------------------------------------------------------------
            -- STAGE 4c2 : 3->2 sum and max (pair 45 rides through)
            ----------------------------------------------------------------
            s4c2_sum_0123 <= resize(s4c1_sum_01, 10) + resize(s4c1_sum_23, 10);
            s4c2_sum_45d  <= resize(s4c1_sum_45, 10);

            if s4c1_max01_w >= s4c1_max23_w then
                s4c2_max0123_w <= s4c1_max01_w; s4c2_max0123_ci <= s4c1_max01_ci;
            else
                s4c2_max0123_w <= s4c1_max23_w; s4c2_max0123_ci <= s4c1_max23_ci;
            end if;
            s4c2_max45_w  <= s4c1_max45_w;
            s4c2_max45_ci <= s4c1_max45_ci;

            ----------------------------------------------------------------
            -- STAGE 4c3 : final sum + clamp + final max + alpha band
            ----------------------------------------------------------------
            total_w := resize(s4c2_sum_0123, 11) + resize(s4c2_sum_45d, 11)
                     + resize(bg3_w, 11);

            if s4c2_max0123_w >= s4c2_max45_w then
                best_ci := s4c2_max0123_ci;
                max_w   := s4c2_max0123_w;
            else
                best_ci := s4c2_max45_ci;
                max_w   := s4c2_max45_w;
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
            -- STAGE 4m1 : brightness operand + shimmer-palette EBR read.
            -- Per-entry luma dim (silver, gold, amber...) scales the
            -- brightness operand for STAR pixels only -- with no star the
            -- winning colour index is a random hash bit, and dimming the
            -- nebula by it would be noise.
            ----------------------------------------------------------------
            s4m1_w <= s4c3_w;
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
            -- STAGE 4m2 : shared 10x10 brightness multiply + palette reg
            ----------------------------------------------------------------
            prod_v := s4m1_w * s4m1_bright;
            s4m2_y <= prod_v(19 downto 8);
            s4m2_a <= s4m1_a;
            if bw_r = '1' then
                s4m2_u <= to_unsigned(512, 10);
                s4m2_v <= to_unsigned(512, 10);
            else
                s4m2_u <= unsigned(s4m1_pal(19 downto 10));
                s4m2_v <= unsigned(s4m1_pal(9 downto 0));
            end if;

            ----------------------------------------------------------------
            -- STAGE 5a : star colour over black. Y = 64 + product, capped
            -- at 768; chroma = palette on a star, nebula tint / neutral off.
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
            -- aligned avid the output is neutral black (the accumulators
            -- freeze in blanking and would repeat a star).
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
    -- latency changes (18 -> 20).
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

end architecture warpfield;
