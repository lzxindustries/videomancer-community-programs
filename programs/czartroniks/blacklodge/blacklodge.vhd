-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Blacklodge: Red Room (Twin Peaks) synthesiser
--
-- Crimson velvet curtain (fold-phase DDA, velvet sin^2 shading, sway =
-- two travelling waves phase-modulating the folds, pinned at the top)
-- hanging over a cream/brown chevron floor in forced perspective
-- (reciprocal-LUT wavelength DDA, Rays or Rows chevrons, K5 Walk flow).
-- S10 Flicker = random Lynch light bursts; S11 Key = input video floor.
--
-- v1.0.2 (2026-10-17): resolution scaling.  The x/line counters run in
-- virtual 1080-line units, stepped per pixel / per field line from the video
-- timing ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at
-- 720p, and at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame
-- line.  Fold pitch, sway waves, hem bulge + shadow, horizon, apex, chevron
-- width, the floor's per-row perspective growth and the vignette ramp all
-- follow, so 720p and SD look like the 1080p room scaled (SD no longer has
-- wide folds or a vignette that darkens the whole field).  The floor's per-row
-- increments (row phase, wavelength growth) are scaled by the y step on the
-- hblank serial multiplier; the sway amplitude always grows as line/32.
--
-- v1.0.0 (2026-10-07, merged from the blacklodge_diag test-bed):
--   * processing program_type + blanking gate
--   * active-line counter; frame height captured from active lines,
--     per-frame work guarded by a saw-active flag (analog vsync serrates)
--   * luma never below black 64, chroma fades with luma, cream Y <= 760
--   * apex x row-frequency on a serial shift-add multiplier in hblank
--   * smooth sway, velvet profile, continuous K3/K6, K5 Walk, S10 Flicker
--
-- Register map:
--   registers_in(0) = Folds    (fold density, continuous)
--   registers_in(1) = Depth    (fold shadow level)
--   registers_in(2) = Width    (stripe width at the horizon, 4..36 units)
--   registers_in(3) = Apex     (vanishing-point x)
--   registers_in(4) = Walk     (bipolar floor flow speed, centre = still)
--   registers_in(5) = ViewHt   (perspective growth rate)
--   registers_in(6) = Switches (bit0 Sway, bit1 Chevrons Rays/Rows,
--                               bit2 Brown, bit3 Flicker, bit4 Key)
--   registers_in(7) = Horizon  (slider, top = curtain raised)
--
-- License: GPL-3.0
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture blacklodge of program_top is
    -- data_in -> data_out latency is 8 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    ----------------------------------------------------------------------
    -- Palette, stored as offsets: luma above black (Y - 64), chroma signed
    -- about 512.  Videomancer convention: BT.601 Cr -> U, Cb -> V.
    ----------------------------------------------------------------------
    constant C_BLACK_Y    : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Curtain crease (Y 70, U 593, V 490); ridge adds the lerp LUTs.
    constant C_CUR_LO_YR : unsigned(9 downto 0) := to_unsigned(6, 10);
    constant C_CUR_LO_US : signed(10 downto 0)  := to_signed(81, 11);
    constant C_CUR_LO_VS : signed(10 downto 0)  := to_signed(-22, 11);

    -- Warm cream (Y 760 -- capped below the ~768 clip, U 544, V 456)
    constant C_FLR_LT_YR : unsigned(9 downto 0) := to_unsigned(696, 10);
    constant C_FLR_LT_US : signed(10 downto 0)  := to_signed(32, 11);
    constant C_FLR_LT_VS : signed(10 downto 0)  := to_signed(-56, 11);

    -- Near-black warm brown (Y 122, U 545, V 497)
    constant C_FLR_DK_YR : unsigned(9 downto 0) := to_unsigned(58, 10);
    constant C_FLR_DK_US : signed(10 downto 0)  := to_signed(33, 11);
    constant C_FLR_DK_VS : signed(10 downto 0)  := to_signed(-15, 11);

    -- Saturated brown, S9 Brown (Y 170, U 620, V 458)
    constant C_FLR_BR_YR : unsigned(9 downto 0) := to_unsigned(106, 10);
    constant C_FLR_BR_US : signed(10 downto 0)  := to_signed(108, 11);
    constant C_FLR_BR_VS : signed(10 downto 0)  := to_signed(-54, 11);

    ----------------------------------------------------------------------
    -- Curtain hem LUT: parabolic bulge 0..12 px across each fold.
    ----------------------------------------------------------------------
    type t_hem_lut is array (0 to 63) of unsigned(3 downto 0);

    function gen_hem_lut return t_hem_lut is
        variable r   : t_hem_lut;
        variable val : integer;
    begin
        for i in 0 to 63 loop
            val := (4 * 12 * i * (63 - i) + 1985) / 3969;
            if val > 15 then
                val := 15;
            end if;
            r(i) := to_unsigned(val, 4);
        end loop;
        return r;
    end function;

    constant C_HEM_LUT : t_hem_lut := gen_hem_lut;

    ----------------------------------------------------------------------
    -- Velvet lerp LUTs, indexed by the 6-bit blend factor.
    --   Y : 280 * sin^2(pi/2 * x^0.75)  -- flat at crease AND ridge, so
    --       the folds read as rounded cloth instead of triangle kinks;
    --       x^0.75 widens the lit ridges and narrows the creases.
    --   U : 240 * sin(pi/2 * x^0.75)    -- saturation rises fast out of
    --   V :  45 * sin(pi/2 * x^0.75)       the crease (maroon, not grey)
    ----------------------------------------------------------------------
    type t_lerp_y_lut is array (0 to 63) of natural range 0 to 511;
    type t_lerp_u_lut is array (0 to 63) of natural range 0 to 255;
    type t_lerp_v_lut is array (0 to 63) of natural range 0 to 63;

    constant C_LERP_Y_LUT : t_lerp_y_lut := (
          0,   1,   4,   7,  11,  15,  20,  25,  30,  36,  41,  47,  54,  60,  66,  73,
         79,  86,  93, 100, 106, 113, 120, 127, 133, 140, 147, 153, 159, 166, 172, 178,
        184, 190, 195, 201, 206, 212, 217, 222, 226, 231, 235, 239, 243, 247, 251, 254,
        257, 260, 263, 265, 268, 270, 272, 274, 275, 276, 278, 278, 279, 280, 280, 280);
    constant C_LERP_U_LUT : t_lerp_u_lut := (
          0,  17,  28,  38,  47,  56,  64,  71,  79,  86,  92,  99, 105, 111, 117, 122,
        128, 133, 138, 143, 148, 153, 157, 161, 166, 170, 174, 177, 181, 185, 188, 191,
        195, 198, 201, 203, 206, 209, 211, 213, 216, 218, 220, 222, 224, 225, 227, 229,
        230, 231, 233, 234, 235, 236, 236, 237, 238, 238, 239, 239, 240, 240, 240, 240);
    constant C_LERP_V_LUT : t_lerp_v_lut := (
          0,   3,   5,   7,   9,  10,  12,  13,  15,  16,  17,  19,  20,  21,  22,  23,
         24,  25,  26,  27,  28,  29,  29,  30,  31,  32,  33,  33,  34,  35,  35,  36,
         36,  37,  38,  38,  39,  39,  40,  40,  40,  41,  41,  42,  42,  42,  43,  43,
         43,  43,  44,  44,  44,  44,  44,  44,  45,  45,  45,  45,  45,  45,  45,  45);

    ----------------------------------------------------------------------
    -- Reciprocal LUT for wavelen in [2, 127] -> 2^17 / wavelen.
    ----------------------------------------------------------------------
    type t_freq_lut is array (0 to 127) of unsigned(16 downto 0);

    function gen_freq_lut return t_freq_lut is
        variable r : t_freq_lut;
    begin
        r := (others => to_unsigned(1032, 17));
        for i in 2 to 127 loop
            r(i) := to_unsigned(131072 / i, 17);
        end loop;
        return r;
    end function;

    constant C_FREQ_LUT : t_freq_lut := gen_freq_lut;

    -- Sway waves: per-pixel phase steps (period ~640 px and ~1075 px,
    -- travelling in opposite directions) and per-frame drift speeds.
    constant C_WAVE1_K  : unsigned(15 downto 0) := to_unsigned(102, 16);
    constant C_WAVE2_K  : unsigned(15 downto 0) := to_unsigned(61, 16);
    constant C_WAVE1_DT : unsigned(15 downto 0) := to_unsigned(197, 16);
    constant C_WAVE2_DT : unsigned(15 downto 0) := to_unsigned(131, 16);

    ----------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
    ----------------------------------------------------------------------
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

    ----------------------------------------------------------------------
    -- Position / timing.  pixel_x and aline are the integer parts of
    -- virtual-unit accumulators (x Q.1 per pixel, y Q.8 per active field
    -- line); at 1080p they are the plain pixel / active-line counters.
    -- max_x_r / max_y_r are therefore the VIRTUAL width / height (SD 1440
    -- wide, every mode ~1080 high); nothing here addresses memory or
    -- generates avid, so no real-pixel width is needed.
    ----------------------------------------------------------------------
    signal s_xs         : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys         : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal vxa          : unsigned(12 downto 0) := (others => '0');
    signal vya          : unsigned(19 downto 0) := (others => '0');
    signal vy_nx_r      : unsigned(11 downto 0) := to_unsigned(1, 12);
    signal pixel_x      : unsigned(11 downto 0);
    signal aline        : unsigned(11 downto 0);
    -- per-pixel steps scaled by the x step
    signal fold_step_r  : unsigned(12 downto 0) := to_unsigned(1484, 13);
    signal wav1_k_r     : unsigned(15 downto 0) := C_WAVE1_K;
    signal wav2_k_r     : unsigned(15 downto 0) := C_WAVE2_K;
    signal freq_px_r    : unsigned(17 downto 0) := to_unsigned(3744, 18);
    -- per-row steps scaled by the y step (serial multiplier products)
    signal yinc_r       : unsigned(19 downto 0) := to_unsigned(3744, 20);
    signal stepv_r      : unsigned(12 downto 0) := to_unsigned(640, 13);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';
    signal saw_active   : std_logic := '0';
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal xm1_r        : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal ym1_r        : unsigned(11 downto 0) := to_unsigned(1079, 12);

    ----------------------------------------------------------------------
    -- Free-running parameter precompute (latched at the frame tick)
    ----------------------------------------------------------------------
    signal hz_lo_r      : unsigned(16 downto 0) := (others => '0');
    signal hz_hi_r      : unsigned(16 downto 0) := (others => '0');
    signal hz_full_r    : unsigned(21 downto 0) := (others => '0');
    signal ap_lo_r      : unsigned(16 downto 0) := (others => '0');
    signal ap_hi_r      : unsigned(16 downto 0) := (others => '0');
    signal ap_full_r    : unsigned(21 downto 0) := (others => '0');
    signal k5_mag_r     : unsigned(8 downto 0)  := (others => '0');
    signal k5_fwd_r     : std_logic := '0';
    signal k5_sq_r      : unsigned(17 downto 0) := (others => '0');
    signal next_floor_r : std_logic := '0';
    signal amp_next_r   : unsigned(6 downto 0)  := to_unsigned(16, 7);

    ----------------------------------------------------------------------
    -- Per-frame state
    ----------------------------------------------------------------------
    signal horizon_r       : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal eff_horizon_x_r : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal fold_freq_r     : unsigned(11 downto 0) := to_unsigned(1484, 12);
    signal hem_shift_r     : unsigned(1 downto 0)  := "01";
    signal depth_off_s_r   : signed(11 downto 0)   := (others => '0');
    signal chev_min_int_r  : unsigned(6 downto 0)  := to_unsigned(35, 7);
    signal chev_min_frac_r : unsigned(12 downto 0) := (others => '0');
    -- Perspective step in 1/8192 px of wavelen growth per row.
    signal step_r          : unsigned(12 downto 0) := to_unsigned(640, 13);
    signal sway_en_r       : std_logic := '1';
    signal chev_dir_r      : std_logic := '0';
    signal flick_en_r      : std_logic := '0';
    signal key_en_r        : std_logic := '0';
    signal dk_yr_r         : unsigned(9 downto 0) := C_FLR_BR_YR;
    signal dk_us_r         : signed(10 downto 0)  := C_FLR_BR_US;
    signal dk_vs_r         : signed(10 downto 0)  := C_FLR_BR_VS;
    signal walk_phase_r    : unsigned(19 downto 0) := (others => '0');
    signal sway_t1_r       : unsigned(15 downto 0) := (others => '0');
    signal sway_t2_r       : unsigned(15 downto 0) := to_unsigned(16#5A00#, 16);
    signal rnd_r           : unsigned(15 downto 0) := to_unsigned(16#ACE1#, 16);
    signal flick_cnt_r     : unsigned(3 downto 0)  := (others => '0');
    signal flick_shift_r   : unsigned(2 downto 0)  := (others => '0');

    ----------------------------------------------------------------------
    -- Curtain: fold phase DDA + sway phase modulation
    ----------------------------------------------------------------------
    signal fold_phase_r : unsigned(15 downto 0) := (others => '0');
    signal wph1_r       : unsigned(15 downto 0) := (others => '0');
    signal wph2_r       : unsigned(15 downto 0) := (others => '0');
    signal amp_r        : unsigned(6 downto 0)  := to_unsigned(16, 7);
    signal sw_w_r       : signed(10 downto 0)   := (others => '0');
    signal sw_prod_r    : signed(18 downto 0)   := (others => '0');
    signal sway_off_r   : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Floor: per-row wavelength (7.13 fixed point) + reciprocal pipeline
    ----------------------------------------------------------------------
    signal wavelen_int_r   : unsigned(6 downto 0)  := to_unsigned(35, 7);
    signal wavelen_frac_r  : unsigned(12 downto 0) := (others => '0');
    signal wavelen_carry_r : std_logic := '0';
    signal freq_a_r        : unsigned(16 downto 0) := (others => '0');
    signal freq_b_r        : unsigned(16 downto 0) := (others => '0');
    signal freq_diff_r     : unsigned(16 downto 0) := (others => '0');
    signal freq_a_d1_r     : unsigned(16 downto 0) := (others => '0');
    signal freq_a_d2_r     : unsigned(16 downto 0) := (others => '0');
    signal freq_a_d3_r     : unsigned(16 downto 0) := (others => '0');
    signal freq_ip_hi_r    : unsigned(20 downto 0) := (others => '0');
    signal freq_ip_lo_r    : unsigned(20 downto 0) := (others => '0');
    signal freq_interp_r   : unsigned(24 downto 0) := (others => '0');
    signal freq_row_r      : unsigned(16 downto 0) := to_unsigned(3744, 17);

    -- Row sequencer (counts hblank clocks from avid fall / vsync) and the
    -- serial shift-add multiplier: apex_x * freq_row (mod 2^20), then
    -- freq_row * ys and step * ys (Q.8, per-row increments in virtual units).
    signal row_seq_r : unsigned(5 downto 0)  := (others => '1');
    signal mul_acc_r : unsigned(27 downto 0) := (others => '0');
    signal mul_a_r   : unsigned(16 downto 0) := (others => '0');
    signal mul_b_r   : unsigned(11 downto 0) := (others => '0');
    signal dx_init_r : unsigned(19 downto 0) := (others => '0');

    -- Chevron phase accumulators (modular, 20-bit; bit 17 = stripe)
    signal dx_phase_r : unsigned(19 downto 0) := (others => '0');
    signal y_acc_r    : unsigned(19 downto 0) := (others => '0');

    signal chev_x_color_d1 : std_logic := '0';
    signal chev_y_color_d1 : std_logic := '0';
    signal chev_color_d2   : std_logic := '0';
    signal chev_color_d3   : std_logic := '0';

    ----------------------------------------------------------------------
    -- Vignette (always on) + flicker gain.  fade 128 = unity.
    ----------------------------------------------------------------------
    signal dist_l_r   : unsigned(11 downto 0) := (others => '0');
    signal dist_r_r   : unsigned(11 downto 0) := (others => '0');
    signal dist_t_r   : unsigned(11 downto 0) := (others => '0');
    signal dist_b_r   : unsigned(11 downto 0) := (others => '0');
    signal dist_x_r   : unsigned(11 downto 0) := (others => '0');
    signal dist_y_r   : unsigned(11 downto 0) := (others => '0');
    signal min_dist_r : unsigned(11 downto 0) := (others => '0');
    signal fade_lvl_r : unsigned(7 downto 0)  := to_unsigned(128, 8);

    ----------------------------------------------------------------------
    -- Pixel pipeline
    ----------------------------------------------------------------------
    signal s1_y          : unsigned(11 downto 0) := (others => '0');
    signal s1_phase      : unsigned(15 downto 0) := (others => '0');

    signal s2_y          : unsigned(11 downto 0) := (others => '0');
    signal s2_curtain_bot: unsigned(11 downto 0) := (others => '0');
    signal s2_tri_a      : unsigned(9 downto 0) := (others => '0');

    signal s3_is_floor     : std_logic := '0';
    signal s3_shadow_amt   : unsigned(3 downto 0) := (others => '0');
    signal s3_blend_factor : unsigned(5 downto 0) := (others => '0');

    signal s4_is_floor : std_logic := '0';
    signal s4_cur_yr   : unsigned(9 downto 0) := (others => '0');
    signal s4_cur_us   : signed(10 downto 0)  := (others => '0');
    signal s4_cur_vs   : signed(10 downto 0)  := (others => '0');
    signal s4_flr_yr   : unsigned(9 downto 0) := (others => '0');
    signal s4_flr_us   : signed(10 downto 0)  := (others => '0');
    signal s4_flr_vs   : signed(10 downto 0)  := (others => '0');

    signal s5_yr       : unsigned(9 downto 0) := (others => '0');
    signal s5_us       : signed(10 downto 0)  := (others => '0');
    signal s5_vs       : signed(10 downto 0)  := (others => '0');
    signal s5_fade     : unsigned(7 downto 0) := (others => '0');
    signal s5_is_floor : std_logic := '0';

    signal s6_ym       : unsigned(17 downto 0) := (others => '0');
    signal s6_um       : signed(19 downto 0)   := (others => '0');
    signal s6_vm       : signed(19 downto 0)   := (others => '0');
    signal s6_is_floor : std_logic := '0';

    signal s7_y        : unsigned(9 downto 0) := C_BLACK_Y;
    signal s7_u        : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s7_v        : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s7_is_floor : std_logic := '0';

    signal out_y : unsigned(9 downto 0) := C_BLACK_Y;
    signal out_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal out_v : unsigned(9 downto 0) := C_CHROMA_MID;

begin

    process(clk)
        variable v_hs_edge     : boolean;
        variable v_vs_edge     : boolean;
        variable v_avid_fall   : boolean;
        variable v_hz_pot      : unsigned(9 downto 0);
        variable v_ap_pot      : unsigned(9 downto 0);
        variable v_k5s         : signed(10 downto 0);
        variable v_k5abs       : unsigned(10 downto 0);
        variable v_amp_src     : unsigned(11 downto 0);
        variable v_fold_pre    : unsigned(12 downto 0);
        variable v_fold_freq   : unsigned(11 downto 0);
        variable v_rnd         : unsigned(15 downto 0);
        variable v_tri1        : unsigned(8 downto 0);
        variable v_tri2        : unsigned(8 downto 0);
        variable v_tooth_y     : unsigned(19 downto 0);
        variable v_tooth_x     : unsigned(19 downto 0);
        variable v_chev_x      : unsigned(19 downto 0);
        variable v_chev_y      : unsigned(19 downto 0);
        variable v_fade        : unsigned(7 downto 0);
        variable v_tri_a       : unsigned(9 downto 0);
        variable v_hem_raw     : unsigned(3 downto 0);
        variable v_hem         : unsigned(3 downto 0);
        variable v_shadow_d    : unsigned(11 downto 0);
        variable v_blend       : signed(11 downto 0);
        variable v_flr_yr      : unsigned(9 downto 0);
        variable v_bf          : integer range 0 to 63;
        variable v_c11         : signed(10 downto 0);
        variable v_vya_nx      : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then

            ------------------------------------------------------------
            -- Input delay pipeline (syncs + Key video)
            ------------------------------------------------------------
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_hs_edge   := data_in.hsync_n = '0' and prev_hsync_n = '1';
            v_vs_edge   := data_in.vsync_n = '0' and prev_vsync_n = '1';
            v_avid_fall := prev_avid = '1' and data_in.avid = '0';

            ------------------------------------------------------------
            -- Counters.  aline counts ACTIVE lines (advances at avid
            -- fall); frame height is captured from it at the guarded
            -- frame tick, never from a vsync-edge line count.
            ------------------------------------------------------------
            -- (Virtual units: pixel_x = vxa / 2, aline = vya / 256.)
            -- Per-mode steps from the timing ID, free-running (a static
            -- register; latching it at the frame tick left the first field
            -- after load counting at the 1080p step, and its wrong height
            -- set the next field's horizon).
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            if data_in.avid = '1' then
                vxa <= vxa + s_xs;
            end if;
            if v_hs_edge then
                if pixel_x > to_unsigned(0, 12) then
                    max_x_r <= pixel_x;
                end if;
                vxa <= (others => '0');
            end if;
            if v_avid_fall then
                vya <= vya + s_ys;
            end if;
            -- virtual line of the NEXT active line (was aline + 1)
            v_vya_nx := vya + s_ys;
            vy_nx_r  <= v_vya_nx(19 downto 8);
            xm1_r <= max_x_r - 1;
            ym1_r <= max_y_r - 1;

            -- Per-pixel steps x the x step (1.0 / 1.5 / 2.0).
            case to_integer(s_xs) is
                when 3 =>
                    fold_step_r <= resize(fold_freq_r, 13)
                                 + resize(fold_freq_r(11 downto 1), 13);
                    wav1_k_r  <= to_unsigned(153, 16);
                    wav2_k_r  <= to_unsigned(92, 16);
                    freq_px_r <= resize(freq_row_r, 18)
                               + resize(freq_row_r(16 downto 1), 18);
                when 4 =>
                    fold_step_r <= fold_freq_r & '0';
                    wav1_k_r  <= to_unsigned(204, 16);
                    wav2_k_r  <= to_unsigned(122, 16);
                    freq_px_r <= freq_row_r & '0';
                when others =>
                    fold_step_r <= resize(fold_freq_r, 13);
                    wav1_k_r  <= C_WAVE1_K;
                    wav2_k_r  <= C_WAVE2_K;
                    freq_px_r <= resize(freq_row_r, 18);
            end case;

            ------------------------------------------------------------
            -- Free-running parameter precompute
            ------------------------------------------------------------
            -- Horizon = height * (1023 - P12) / 1024, split 12x5 + 12x5.
            v_hz_pot  := to_unsigned(1023, 10) - unsigned(registers_in(7));
            hz_lo_r   <= max_y_r * v_hz_pot(4 downto 0);
            hz_hi_r   <= max_y_r * v_hz_pot(9 downto 5);
            hz_full_r <= shift_left(resize(hz_hi_r, 22), 5) + resize(hz_lo_r, 22);
            -- Apex x = width * K4 / 1024.
            v_ap_pot  := unsigned(registers_in(3));
            ap_lo_r   <= max_x_r * v_ap_pot(4 downto 0);
            ap_hi_r   <= max_x_r * v_ap_pot(9 downto 5);
            ap_full_r <= shift_left(resize(ap_hi_r, 22), 5) + resize(ap_lo_r, 22);

            -- K5 Walk: bipolar, +/-16 deadband, squared for fine control
            -- near centre.  Max ~15k phase/frame = ~3.5 band pairs/s.
            v_k5s := signed(resize(unsigned(registers_in(4)), 11))
                   - to_signed(512, 11);
            v_k5abs := unsigned(abs(v_k5s));
            if v_k5abs > to_unsigned(16, 11) then
                k5_mag_r <= resize(v_k5abs - 16, 9);
            else
                k5_mag_r <= (others => '0');
            end if;
            if v_k5s > 0 then
                k5_fwd_r <= '1';
            else
                k5_fwd_r <= '0';
            end if;
            k5_sq_r <= k5_mag_r * k5_mag_r;

            if vy_nx_r >= horizon_r then
                next_floor_r <= '1';
            else
                next_floor_r <= '0';
            end if;

            -- Sway amplitude for the next line: 16 at the top (pinned),
            -- growing with depth down the screen (~49 at the bottom).
            -- Virtual line / 32 in every mode (was a per-height-class
            -- shift that gave 720p a ~25% larger sway).
            v_amp_src := shift_right(vy_nx_r, 5);
            if v_amp_src > to_unsigned(47, 12) then
                amp_next_r <= to_unsigned(63, 7);
            else
                amp_next_r <= resize(v_amp_src, 7) + to_unsigned(16, 7);
            end if;

            ------------------------------------------------------------
            -- Frame tick: first vsync edge after active video.  Analog
            -- vsync serrates, so everything that latches, increments or
            -- toggles per frame lives behind saw_active.
            ------------------------------------------------------------
            if data_in.avid = '1' then
                saw_active <= '1';
            end if;
            if v_vs_edge then
                -- bottom field (SD only; HD ties field_n high) starts half
                -- a field step down
                if data_in.field_n = '0' then
                    vya <= resize(s_ys(11 downto 1), 20);
                else
                    vya <= (others => '0');
                end if;
            end if;
            if v_vs_edge and saw_active = '1' then
                saw_active <= '0';
                if aline > to_unsigned(0, 12) then
                    max_y_r <= aline;
                end if;

                horizon_r       <= hz_full_r(21 downto 10);
                eff_horizon_x_r <= ap_full_r(21 downto 10);

                -- K3 Width: 4.0 .. 35.97 (1080-line units) at the horizon, continuous
                -- (top 5 bits -> integer px, low 5 bits -> fraction).
                chev_min_int_r  <= resize(unsigned(registers_in(2)(9 downto 5)), 7)
                                 + to_unsigned(4, 7);
                chev_min_frac_r <= unsigned(registers_in(2)(4 downto 0))
                                 & "00000000";

                -- K6 ViewHt: wavelen growth 1152 - K6 (/8192 px per row)
                -- = 36/256 .. 4/256, continuous.
                step_r <= to_unsigned(1152, 13)
                        - resize(unsigned(registers_in(5)), 13);

                v_fold_pre := shift_left(resize(unsigned(registers_in(0)), 13), 2)
                            + to_unsigned(256, 13);
                if v_fold_pre > to_unsigned(3936, 13) then
                    v_fold_freq := to_unsigned(3936, 12);
                else
                    v_fold_freq := v_fold_pre(11 downto 0);
                end if;
                fold_freq_r <= v_fold_freq;
                hem_shift_r <= v_fold_freq(11 downto 10);

                depth_off_s_r <= signed(resize(unsigned(registers_in(1)), 12))
                               - to_signed(512, 12);

                sway_en_r  <= registers_in(6)(0);
                chev_dir_r <= registers_in(6)(1);
                flick_en_r <= registers_in(6)(3);
                key_en_r   <= registers_in(6)(4);

                if registers_in(6)(2) = '1' then
                    dk_yr_r <= C_FLR_BR_YR;
                    dk_us_r <= C_FLR_BR_US;
                    dk_vs_r <= C_FLR_BR_VS;
                else
                    dk_yr_r <= C_FLR_DK_YR;
                    dk_us_r <= C_FLR_DK_US;
                    dk_vs_r <= C_FLR_DK_VS;
                end if;

                -- Walk: K5 right = bands flow toward the viewer.
                if k5_fwd_r = '1' then
                    walk_phase_r <= walk_phase_r - resize(k5_sq_r(17 downto 4), 20);
                else
                    walk_phase_r <= walk_phase_r + resize(k5_sq_r(17 downto 4), 20);
                end if;

                -- Sway off freezes the drape in place (keeps its shape).
                if sway_en_r = '1' then
                    sway_t1_r <= sway_t1_r + C_WAVE1_DT;
                    sway_t2_r <= sway_t2_r + C_WAVE2_DT;
                end if;

                -- xorshift16 (7,9,8) per frame
                v_rnd := rnd_r;
                v_rnd := v_rnd xor shift_left(v_rnd, 7);
                v_rnd := v_rnd xor shift_right(v_rnd, 9);
                v_rnd := v_rnd xor shift_left(v_rnd, 8);
                rnd_r <= v_rnd;

                -- Flicker: ~3/256 chance per frame of a 4..11-frame
                -- burst; each burst frame picks gain 1, 1/2, 1/4 or 1/16.
                if flick_en_r = '0' then
                    flick_cnt_r   <= (others => '0');
                    flick_shift_r <= (others => '0');
                elsif flick_cnt_r = 0 then
                    flick_shift_r <= (others => '0');
                    if v_rnd(7 downto 0) < to_unsigned(3, 8) then
                        flick_cnt_r <= resize(v_rnd(10 downto 8), 4)
                                     + to_unsigned(4, 4);
                    end if;
                else
                    flick_cnt_r <= flick_cnt_r - 1;
                    case v_rnd(13 downto 12) is
                        when "00"   => flick_shift_r <= to_unsigned(0, 3);
                        when "01"   => flick_shift_r <= to_unsigned(1, 3);
                        when "10"   => flick_shift_r <= to_unsigned(2, 3);
                        when others => flick_shift_r <= to_unsigned(4, 3);
                    end case;
                end if;
            end if;

            ------------------------------------------------------------
            -- Curtain fold phase + sway waves (reset each line)
            ------------------------------------------------------------
            if v_hs_edge then
                fold_phase_r <= (others => '0');
                wph1_r       <= sway_t1_r;
                wph2_r       <= sway_t2_r;
            elsif data_in.avid = '1' then
                fold_phase_r <= fold_phase_r + resize(fold_step_r, 16);
                wph1_r       <= wph1_r + wav1_k_r;
                wph2_r       <= wph2_r - wav2_k_r;
            end if;

            -- Two travelling triangle waves summed (signed, +/-511),
            -- times the per-line amplitude, /2 -> fold phase offset of
            -- up to ~1/4 fold at the bottom.  A few px of lag vs the
            -- fold DDA is invisible on a wave this slow.
            if wph1_r(15) = '0' then
                v_tri1 := wph1_r(14 downto 6);
            else
                v_tri1 := not wph1_r(14 downto 6);
            end if;
            if wph2_r(15) = '0' then
                v_tri2 := wph2_r(14 downto 6);
            else
                v_tri2 := not wph2_r(14 downto 6);
            end if;
            sw_w_r     <= signed(resize(v_tri1, 11)) + signed(resize(v_tri2, 11))
                        - to_signed(511, 11);
            sw_prod_r  <= sw_w_r * signed(resize(amp_r, 8));
            sway_off_r <= unsigned(sw_prod_r(16 downto 1));

            ------------------------------------------------------------
            -- Floor per-row update: at avid fall (active lines only) and
            -- reset at vsync (idempotent, safe under serration).
            ------------------------------------------------------------
            -- (stepv / yinc = per-row step and row phase x the y step, so
            -- the perspective spans the same virtual depth in every mode;
            -- stepv <= 1152 x 4.45 < 8192, so at most one carry per row.)
            if resize(wavelen_frac_r, 14) + resize(stepv_r, 14)
                    >= to_unsigned(8192, 14) then
                wavelen_carry_r <= '1';
            else
                wavelen_carry_r <= '0';
            end if;

            if v_vs_edge then
                wavelen_int_r  <= chev_min_int_r;
                wavelen_frac_r <= chev_min_frac_r;
                y_acc_r        <= walk_phase_r;
                amp_r          <= to_unsigned(16, 7);
                row_seq_r      <= (others => '0');
            elsif v_avid_fall then
                row_seq_r <= (others => '0');
                amp_r     <= amp_next_r;
                if next_floor_r = '1' then
                    wavelen_frac_r <= wavelen_frac_r + stepv_r;
                    if wavelen_carry_r = '1'
                            and wavelen_int_r < to_unsigned(127, 7) then
                        wavelen_int_r <= wavelen_int_r + 1;
                    end if;
                    y_acc_r <= y_acc_r + yinc_r;
                else
                    wavelen_int_r  <= chev_min_int_r;
                    wavelen_frac_r <= chev_min_frac_r;
                    y_acc_r        <= walk_phase_r;
                end if;
            elsif row_seq_r /= to_unsigned(63, 6) then
                row_seq_r <= row_seq_r + 1;
            end if;

            -- Reciprocal pipeline: freq_row = lerp(LUT(int), LUT(int+1))
            -- by the top 8 bits of the fraction.  Settles ~6 clocks
            -- after a row update.
            freq_a_r <= C_FREQ_LUT(to_integer(wavelen_int_r));
            if wavelen_int_r >= to_unsigned(127, 7) then
                freq_b_r <= C_FREQ_LUT(127);
            else
                freq_b_r <= C_FREQ_LUT(to_integer(wavelen_int_r) + 1);
            end if;
            freq_diff_r   <= freq_a_r - freq_b_r;
            freq_a_d1_r   <= freq_a_r;
            freq_a_d2_r   <= freq_a_d1_r;
            freq_a_d3_r   <= freq_a_d2_r;
            freq_ip_hi_r  <= freq_diff_r * wavelen_frac_r(12 downto 9);
            freq_ip_lo_r  <= freq_diff_r * wavelen_frac_r(8 downto 5);
            freq_interp_r <= shift_left(resize(freq_ip_hi_r, 25), 4)
                           + resize(freq_ip_lo_r, 25);
            freq_row_r    <= freq_a_d3_r - freq_interp_r(24 downto 8);

            -- Serial multiply in hblank, MSB-first, 12 clocks per job:
            --   10..23  dx_init = -(apex_x * freq_row) mod 2^20, done ~24
            --           clocks after avid fall (SD hblank 138, sim 64)
            --   23..36  yinc  = freq_row * ys >> 8  (row phase step)
            --   36..49  stepv = step * ys >> 8      (wavelength growth)
            -- yinc / stepv are used at the NEXT avid fall; at 1080p
            -- (ys = 256) they equal freq_row / step exactly.
            case to_integer(row_seq_r) is
                when 10 =>
                    mul_acc_r <= (others => '0');
                    mul_a_r   <= freq_row_r;
                    mul_b_r   <= eff_horizon_x_r;
                when 11 to 22 | 24 to 35 | 37 to 48 =>
                    if mul_b_r(11) = '1' then
                        mul_acc_r <= shift_left(mul_acc_r, 1) + resize(mul_a_r, 28);
                    else
                        mul_acc_r <= shift_left(mul_acc_r, 1);
                    end if;
                    mul_b_r <= shift_left(mul_b_r, 1);
                when 23 =>
                    dx_init_r <= to_unsigned(0, 20) - mul_acc_r(19 downto 0);
                    mul_acc_r <= (others => '0');
                    mul_a_r   <= freq_row_r;
                    mul_b_r   <= s_ys;
                when 36 =>
                    yinc_r    <= mul_acc_r(27 downto 8);
                    mul_acc_r <= (others => '0');
                    mul_a_r   <= resize(step_r, 17);
                    mul_b_r   <= s_ys;
                when 49 =>
                    stepv_r   <= mul_acc_r(20 downto 8);
                when others =>
                    null;
            end case;

            -- Per-pixel dx phase: zero at the vanishing point (freq_px =
            -- freq_row x the x step, so the phase is in virtual units).
            if data_in.avid = '0' then
                dx_phase_r <= dx_init_r;
            else
                dx_phase_r <= dx_phase_r + resize(freq_px_r, 20);
            end if;

            -- Rays: stripe = dx_phase + tri(y_acc).  Rows: stripe =
            -- y_acc + tri(dx_phase).  Both 90-degree apex at every depth.
            if y_acc_r(18) = '0' then
                v_tooth_y := "00" & y_acc_r(17 downto 0);
            else
                v_tooth_y := "00" & (not y_acc_r(17 downto 0));
            end if;
            if dx_phase_r(18) = '0' then
                v_tooth_x := "00" & dx_phase_r(17 downto 0);
            else
                v_tooth_x := "00" & (not dx_phase_r(17 downto 0));
            end if;
            v_chev_x := dx_phase_r + v_tooth_y;
            v_chev_y := y_acc_r + v_tooth_x;
            chev_x_color_d1 <= v_chev_x(17);
            chev_y_color_d1 <= v_chev_y(17);
            if chev_dir_r = '1' then
                chev_color_d2 <= chev_y_color_d1;
            else
                chev_color_d2 <= chev_x_color_d1;
            end if;
            chev_color_d3 <= chev_color_d2;

            ------------------------------------------------------------
            -- Vignette (4 stages, aligned with s4) x flicker gain
            ------------------------------------------------------------
            dist_l_r <= pixel_x;
            dist_r_r <= xm1_r - pixel_x;
            dist_t_r <= aline;
            dist_b_r <= ym1_r - aline;

            if dist_l_r < dist_r_r then
                dist_x_r <= dist_l_r;
            else
                dist_x_r <= dist_r_r;
            end if;
            if dist_t_r < dist_b_r then
                dist_y_r <= dist_t_r;
            else
                dist_y_r <= dist_b_r;
            end if;

            if dist_x_r < dist_y_r then
                min_dist_r <= dist_x_r;
            else
                min_dist_r <= dist_y_r;
            end if;

            if min_dist_r >= to_unsigned(128, 12) then
                v_fade := to_unsigned(128, 8);
            else
                v_fade := '0' & min_dist_r(6 downto 0);
            end if;
            case to_integer(flick_shift_r) is
                when 0      => fade_lvl_r <= v_fade;
                when 1      => fade_lvl_r <= shift_right(v_fade, 1);
                when 2      => fade_lvl_r <= shift_right(v_fade, 2);
                when others => fade_lvl_r <= shift_right(v_fade, 4);
            end case;

            ------------------------------------------------------------
            -- Stage 1: swayed fold phase
            ------------------------------------------------------------
            s1_y     <= aline;
            s1_phase <= fold_phase_r + sway_off_r;

            ------------------------------------------------------------
            -- Stage 2: fold triangle + curtain bottom from the SWAYED
            -- phase, so each rounded bottom moves with its fold.
            ------------------------------------------------------------
            if s1_phase(15) = '0' then
                v_tri_a := s1_phase(14 downto 5);
            else
                v_tri_a := not s1_phase(14 downto 5);
            end if;
            s2_tri_a <= v_tri_a;

            v_hem_raw := C_HEM_LUT(to_integer(s1_phase(15 downto 10)));
            case to_integer(hem_shift_r) is
                when 0      => v_hem := v_hem_raw;
                when 1      => v_hem := shift_right(v_hem_raw, 1);
                when 2      => v_hem := shift_right(v_hem_raw, 2);
                when others => v_hem := shift_right(v_hem_raw, 3);
            end case;
            s2_curtain_bot <= horizon_r + resize(v_hem, 12);
            s2_y           <= s1_y;

            ------------------------------------------------------------
            -- Stage 3: floor / hem-shadow test + curtain blend factor
            -- (K2 Depth offset, clamped)
            ------------------------------------------------------------
            v_shadow_d := s2_y - s2_curtain_bot;
            if s2_y >= s2_curtain_bot then
                s3_is_floor <= '1';
                if v_shadow_d < to_unsigned(16, 12) then
                    s3_shadow_amt <= to_unsigned(15, 4) - v_shadow_d(3 downto 0);
                else
                    s3_shadow_amt <= (others => '0');
                end if;
            else
                s3_is_floor   <= '0';
                s3_shadow_amt <= (others => '0');
            end if;
            v_blend := signed(resize(s2_tri_a, 12)) + depth_off_s_r;
            if v_blend < 0 then
                s3_blend_factor <= (others => '0');
            elsif v_blend > to_signed(1023, 12) then
                s3_blend_factor <= to_unsigned(63, 6);
            else
                s3_blend_factor <= unsigned(v_blend(9 downto 4));
            end if;

            ------------------------------------------------------------
            -- Stage 4: colours (offset domain)
            ------------------------------------------------------------
            s4_is_floor <= s3_is_floor;
            v_bf := to_integer(s3_blend_factor);
            s4_cur_yr <= C_CUR_LO_YR + to_unsigned(C_LERP_Y_LUT(v_bf), 10);
            s4_cur_us <= C_CUR_LO_US + to_signed(C_LERP_U_LUT(v_bf), 11);
            s4_cur_vs <= C_CUR_LO_VS - to_signed(C_LERP_V_LUT(v_bf), 11);

            if chev_color_d3 = '1' then
                v_flr_yr  := C_FLR_LT_YR;
                s4_flr_us <= C_FLR_LT_US;
                s4_flr_vs <= C_FLR_LT_VS;
            else
                v_flr_yr  := dk_yr_r;
                s4_flr_us <= dk_us_r;
                s4_flr_vs <= dk_vs_r;
            end if;
            -- Hem drop shadow, scaling luma above black (never sub-black).
            case s3_shadow_amt(3 downto 2) is
                when "11"   => v_flr_yr := shift_right(v_flr_yr, 1);
                when "10"   => v_flr_yr := v_flr_yr - shift_right(v_flr_yr, 2);
                when "01"   => v_flr_yr := v_flr_yr - shift_right(v_flr_yr, 3);
                when others => null;
            end case;
            s4_flr_yr <= v_flr_yr;

            ------------------------------------------------------------
            -- Stage 5: layer select
            ------------------------------------------------------------
            if s4_is_floor = '1' then
                s5_yr <= s4_flr_yr;
                s5_us <= s4_flr_us;
                s5_vs <= s4_flr_vs;
            else
                s5_yr <= s4_cur_yr;
                s5_us <= s4_cur_us;
                s5_vs <= s4_cur_vs;
            end if;
            s5_fade     <= fade_lvl_r;
            s5_is_floor <= s4_is_floor;

            ------------------------------------------------------------
            -- Stage 6: vignette/flicker gain on luma AND chroma offsets
            ------------------------------------------------------------
            s6_ym       <= s5_yr * s5_fade;
            s6_um       <= s5_us * signed(resize(s5_fade, 9));
            s6_vm       <= s5_vs * signed(resize(s5_fade, 9));
            s6_is_floor <= s5_is_floor;

            ------------------------------------------------------------
            -- Stage 7: restore black / chroma centre
            ------------------------------------------------------------
            s7_y <= C_BLACK_Y + s6_ym(16 downto 7);
            v_c11 := signed(resize(C_CHROMA_MID, 11)) + s6_um(17 downto 7);
            s7_u  <= unsigned(v_c11(9 downto 0));
            v_c11 := signed(resize(C_CHROMA_MID, 11)) + s6_vm(17 downto 7);
            s7_v  <= unsigned(v_c11(9 downto 0));
            s7_is_floor <= s6_is_floor;

            ------------------------------------------------------------
            -- Output register: blanking gate, then Key (S11).  pipe(6)
            -- here lines up with pipe(7) syncs at the output.
            ------------------------------------------------------------
            if pipe(LATENCY - 2).avid = '0' then
                out_y <= C_BLACK_Y;
                out_u <= C_CHROMA_MID;
                out_v <= C_CHROMA_MID;
            elsif key_en_r = '1' and s7_is_floor = '1' then
                out_y <= unsigned(pipe(LATENCY - 2).y);
                out_u <= unsigned(pipe(LATENCY - 2).u);
                out_v <= unsigned(pipe(LATENCY - 2).v);
            else
                out_y <= s7_y;
                out_u <= s7_u;
                out_v <= s7_v;
            end if;

        end if;
    end process;

    pixel_x <= vxa(12 downto 1);
    aline   <= vya(19 downto 8);

    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.y       <= std_logic_vector(out_y);
    data_out.u       <= std_logic_vector(out_u);
    data_out.v       <= std_logic_vector(out_v);

end architecture blacklodge;
