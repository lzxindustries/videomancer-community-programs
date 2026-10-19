-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- oracle.vhd  (v2.0.4)
--
-- ORACLE: a divinatory eye opening in a wall of static, with ordered spectra
-- above and below.  The v1 triptych, redrawn at full resolution:
--
--   TOP     Spectrograph -- fine vertical spectral bars from a chirped phase
--           DDA (dense at the edges, wide at the centre), each bar one full
--           spectrum crossed by soft dark absorption lines; the bars ripple
--           row by row on two sines.  S9 lays a second, 1/32 faster line
--           screen over them: sliding colour moire.
--   MIDDLE  A bevelled window (bright rule, inner shadow, spectrum leaking
--           through the gutter) full of static: 1-px grain that re-rolls
--           every frame, horizontal streak runs (random length/brightness/
--           sepia, latched per run), a drifting two-wave cloud that sets the
--           density, prismatic sparks (runs that reveal the spectrum behind),
--           and Newton's rings rippling outward from the eye.
--   EYE     A mandorla (vesica -> circle as it opens) built per line from a
--           half-width profile ROM, so each pixel is one compare plus a 1-px
--           anti-aliased edge whose ramp follows the boundary slope.  A white
--           rim, a rainbow corona that falls off in elliptical rho^2, and an
--           interior of lensed bands (S8: concentric iris rings + pupil).
--   BOTTOM  Horizontal spectral bands swaying on two travelling triangle
--           waves.  S10 Horizon puts them in perspective (receding into a
--           glowing haze under the window) with the eye reflected in the
--           floor as a shimmering column of light.
--   TEAR    S7: random bursts of rows displaced sideways (chroma a few px
--           off the luma) and jagged-tooth seams.  Done by reading the
--           composed picture back out of a 256-px circular buffer with a
--           per-line delay, so every layer tears together.
--
-- ENGINE
--   Raster: measured active width/height (frame rows on interlace, from the
--   timing ID; bottom field = field_n '0'), generated parity-locked avid
--   (aurora/cubist recipe, the HW fix for U/V-swap lines on saturated
--   colour), output gated to
--   64/512/512 outside it, one latency for all modes.
--   Control is a MICROCODED ENGINE (port of cubist's, HW-validated): one
--   32-bit ALU, a 256-word block-RAM register file (two copies, dual read),
--   a 512-word program ROM (gen/oracle_ucode.py -> oracle_ucode_pkg.vhd),
--   a serial Booth multiplier and a serial divider.  Each field it smooths
--   the knobs, derives every per-field constant (eye size and wander, bar
--   chirp, band frequency and perspective numerator, aura scales, tear
--   bands) and writes them to output ports; then, one line ahead of the
--   beam, it computes each line's parameter block (ripple, cloud warps, eye
--   half-width and AA ramp, rho^2 row term, perspective band phase, haze,
--   reflection, tear delay), which XFER copies into the pixel path while
--   the previous line is in blanking.  A hardwired block derives the zone /
--   window / seam flags from the line's row.  Motion phases advance once
--   per FIELD on a saw-active-guarded vsync.
--
-- CONTROLS
--   K1  Size      eye height
--   K2  Gaze      eye wanders on a slow Lissajous (0 = fixed at centre)
--   K3  Aura      corona width + ring density
--   K4  Static    fine grain .. long streaks
--   K5  Flow      one speed for every drift (0 = still; static still boils)
--   K6  Spectrum  bar and band density
--   S7  Tear      off / glitch bursts + torn seams
--   S8  Eye       Lens (bands through the eye) / Iris (rings + pupil)
--   S9  Moire     off / sliding line screens on bars and bands
--   S10 Horizon   Flat bands / perspective floor + haze + reflection
--   S11 Palette   Spectrum / Opal (cool iridescent fold of the spectrum)
--   P12 VISION    the eye opens: glowing hairline slit -> almond -> full
--                 circle; corona, rings, sparks, ripple and sway all swell
--
-- Palette: oracle_tables_pkg (gen/gen_tables.py), hardware convention (the
-- u register carries Cr), so the sim shows mirrored hues.
--
-- v2.0.4 (2026-10-18): resolution scaling.  Interlace now comes from the
--   timing ID (480i / 576i / 1080i), not the SDK field detector: when the
--   detector reports progressive (it keys on a half-line vsync offset) the
--   SD raster was laid out in field rows, so the eye came out half as wide
--   and the seam teeth twice as tall.  The torn-seam teeth now follow the
--   screen: height x(fh/1080) by a timing-ID class table, one step per 8
--   virtual columns.  Tear-burst shifts (S7) are applied in virtual columns
--   (p_tsc: d - D0 scaled by 2/xs where the line delay is latched).
--   1080p is unchanged (class 1080, x step 1).
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
use work.oracle_tables_pkg.all;
use work.oracle_ucode_pkg.all;

architecture oracle of program_top is
    -- Synthesis program: processing_delay_clks is declared 0 in the TOML (the SDK
    -- allows 0 for synthesis programs, and it saves the IO-align pad registers).
    -- The true data_in -> data_out sync latency is 61 clocks.
    constant C_PROCESSING_DELAY_CLKS : natural := 0;

    -- zones
    constant Z_A : unsigned(2 downto 0) := "000";   -- spectrograph bars
    constant Z_G : unsigned(2 downto 0) := "001";   -- gutter (bands leak)
    constant Z_R : unsigned(2 downto 0) := "010";   -- window rule
    constant Z_W : unsigned(2 downto 0) := "011";   -- window static
    constant Z_C : unsigned(2 downto 0) := "100";   -- horizon bands

    -- output buffer: base read delay and the stage that writes it
    constant C_D0 : integer := 40;
    constant C_NW : integer := 17;
    -- line-register block size (gen/oasm.py XFER_FIELDS)
    constant C_NXF : integer := 23;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- ==== timing / raster ====
    signal s_timing  : t_video_timing_port;
    signal s_hcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_meas_v  : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace   : std_logic := '0';
    signal s_botf    : std_logic := '0';
    signal s_fh      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    -- resolution scaling (v2.0.4), decoded from the timing ID every clock:
    -- interlace (480i / 576i / 1080i: core_top ties field_n high, so the
    -- field detector cannot be trusted), seam-tooth height class, and the
    -- virtual x step per pixel (Q.1: 2 / 3 / 4 = 1080p / 720p / SD)
    signal s_tid_il  : std_logic := '0';
    signal s_tcls    : unsigned(1 downto 0) := "00";
    signal s_xs      : unsigned(2 downto 0) := to_unsigned(2, 3);

    -- ==== generated avid (aurora / cubist) ====
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_bcol : unsigned(10 downto 0) := (others => '0');

    -- ==== field tick, line trigger, switches ====
    signal s_saw   : std_logic := '0';
    signal s_vs_go : std_logic := '0';
    signal tq      : std_logic := '0';
    signal tcnt    : unsigned(4 downto 0) := (others => '0');
    signal sw_tear, sw_iris, sw_moire, sw_opal : std_logic := '0';
    signal s_lfsr  : unsigned(15 downto 0) := x"ACE1";

    -- ==== microcoded engine (cubist's design, 24-bit datapath) ====
    -- register file: two copies (dual read), each a 256x16 + a 256x8 RAM
    type t_rf16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_rf8  is array (0 to 255) of std_logic_vector(7 downto 0);
    function rf_lo(c : t_rf) return t_rf16 is
        variable r : t_rf16;
    begin
        for i in 0 to 255 loop r(i) := c(i)(15 downto 0); end loop;
        return r;
    end function;
    function rf_hi(c : t_rf) return t_rf8 is
        variable r : t_rf8;
    begin
        for i in 0 to 255 loop r(i) := c(i)(23 downto 16); end loop;
        return r;
    end function;
    signal rf_al, rf_bl : t_rf16 := rf_lo(C_RFINIT);
    signal rf_ah, rf_bh : t_rf8  := rf_hi(C_RFINIT);
    attribute ram_style : string;
    attribute ram_style of rf_al : signal is "block";
    attribute ram_style of rf_ah : signal is "block";
    attribute ram_style of rf_bl : signal is "block";
    attribute ram_style of rf_bh : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf_al : signal is true;
    attribute no_rw_check of rf_ah : signal is true;
    attribute no_rw_check of rf_bl : signal is true;
    attribute no_rw_check of rf_bh : signal is true;
    signal u_pc  : unsigned(8 downto 0) := (others => '0');
    signal u_st  : unsigned(1 downto 0) := (others => '0');
    signal u_gwt : unsigned(2 downto 0) := (others => '0');
    signal u_ir  : std_logic_vector(31 downto 0) := (others => '0');
    signal u_rom : std_logic_vector(31 downto 0);     -- ROM port: no init
    signal u_fa  : unsigned(8 downto 0) := (others => '0');
    signal u_wlast : std_logic := '0';
    signal u_wa  : unsigned(7 downto 0) := (others => '0');
    signal u_rdal, u_rdbl : std_logic_vector(15 downto 0) := (others => '0');
    signal u_rdah, u_rdbh : std_logic_vector(7 downto 0) := (others => '0');
    signal u_rda, u_rdb : std_logic_vector(23 downto 0);
    signal u_wd  : std_logic_vector(23 downto 0) := (others => '0');
    signal u_ctlv, u_ctlr : signed(23 downto 0) := (others => '0');
    signal u_we  : std_logic := '0';
    signal u_sh  : signed(23 downto 0) := (others => '0');   -- operand A / shifter
    signal u_b   : signed(23 downto 0) := (others => '0');   -- operand B (inverted for compares)
    signal u_shc : unsigned(3 downto 0) := (others => '0');
    signal u_rsel : unsigned(3 downto 0) := (others => '0');
    signal u_rwr, u_cin, u_ismin, u_bltr : std_logic := '0';
    signal u_romr : signed(13 downto 0) := (others => '0');
    signal u_fpend, u_lpend : std_logic := '0';
    -- serial multiplier (unsigned 16 x 12, right-shifting) and restoring
    -- divider (unsigned 25 / 12); operands are magnitudes by microcode
    -- contract (gen/oemu.py asserts it)
    signal mu_pa  : unsigned(15 downto 0) := (others => '0');
    signal mu_pl  : unsigned(11 downto 0) := (others => '0');
    signal mu_ph  : unsigned(16 downto 0) := (others => '0');
    signal mu_bsy : std_logic := '0';
    signal mu_cnt : unsigned(3 downto 0) := (others => '0');
    signal dv_na  : unsigned(24 downto 0) := (others => '0');
    signal dv_ra  : unsigned(12 downto 0) := (others => '0');
    signal dv_da  : unsigned(11 downto 0) := to_unsigned(1, 12);
    signal dv_cnt : unsigned(4 downto 0) := (others => '0');
    signal dv_bsy, dv_post, dv_ovf : std_logic := '0';
    -- line-register block transfer
    signal xf_run, xf_d1 : std_logic := '0';
    signal xf_i, xf_i1   : unsigned(4 downto 0) := (others => '0');
    -- engine ROM ports: quarter sine (10-bit angle), eye profiles
    signal qs_ang  : unsigned(9 downto 0) := (others => '0');
    signal qs_q    : unsigned(11 downto 0);
    signal qs_d    : unsigned(11 downto 0) := (others => '0');
    signal qs_n1, qs_n2 : std_logic := '0';
    signal pr_a    : unsigned(7 downto 0) := (others => '0');
    signal pr_qv, pr_qc : unsigned(11 downto 0);
    signal pr_dv, pr_dc : unsigned(11 downto 0) := (others => '0');

    -- ==== per-field constants (engine output ports) ====
    signal f_cx    : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal f_cA    : unsigned(23 downto 0) := (others => '0');
    signal f_kS1, f_kS2, f_kW1x, f_kW2x : unsigned(9 downto 0) := (others => '0');
    signal t_M, t_G, t_Rg : unsigned(15 downto 0) := (others => '0');
    signal f_spk   : unsigned(7 downto 0) := (others => '0');
    signal f_lenm  : unsigned(5 downto 0) := (others => '0');
    signal f_as4   : unsigned(3 downto 0) := (others => '0');
    signal f_kR    : unsigned(5 downto 0) := (others => '0');
    signal f_se    : unsigned(2 downto 0) := (others => '0');
    signal f_sm    : unsigned(1 downto 0) := (others => '0');
    signal f_g3    : unsigned(2 downto 0) := (others => '0');
    signal f_rsh   : unsigned(1 downto 0) := (others => '0');
    signal f_pup2  : unsigned(13 downto 0) := (others => '0');
    signal f_boost : unsigned(5 downto 0) := (others => '0');
    signal f_wx0, f_wx1m, f_xhm1 : unsigned(10 downto 0) := (others => '0');
    signal f_tseed : unsigned(15 downto 0) := x"ACE1";
    signal f_hs2   : unsigned(17 downto 0) := (others => '0');
    signal f_hxm1  : signed(11 downto 0) := (others => '1');
    -- rho^2 DDA (stage 5): u^2 in Q24, saturating at 16.0
    signal h_q     : unsigned(28 downto 0) := to_unsigned(2**28 - 1, 29);
    signal h_d     : signed(27 downto 0) := (others => '0');
    signal h_x     : unsigned(10 downto 0) := (others => '0');
    signal h_run   : std_logic := '0';
    signal h_sat   : std_logic := '0';
    signal f_yA, f_yC, f_wy0, f_wy1 : unsigned(11 downto 0) := (others => '0');

    -- ==== per-line parameters (XFER) and the derived zone flags ====
    signal cu_rc   : unsigned(11 downto 0) := (others => '0');
    signal cu_lb, cu_phC : unsigned(15 downto 0) := (others => '0');
    signal cu_v2, cu_w4 : unsigned(13 downto 0) := (others => '0');
    signal cu_eye, cu_ref : std_logic := '0';
    signal cu_k    : unsigned(2 downto 0) := (others => '0');
    signal cu_swA  : unsigned(6 downto 0) := (others => '0');
    signal cu_ah, cu_aref : unsigned(4 downto 0) := (others => '0');
    signal cu_rx0, cu_rx1 : unsigned(10 downto 0) := (others => '0');
    signal cu_d, cu_dc : unsigned(7 downto 0) := to_unsigned(C_D0, 8);
    signal cu_zA, cu_zC, cu_win, cu_rr, cu_nA, cu_nC : std_logic := '0';
    signal cu_dt, cu_db : unsigned(4 downto 0) := (others => '0');
    signal cu_sA, cu_sC : signed(5 downto 0) := (others => '0');

    -- ==== stage 0: per-pixel DDAs, RNG, run engine, seam teeth ====
    signal dA_ph, dA_fr : unsigned(23 downto 0) := (others => '0');
    signal dA_lf   : std_logic := '1';
    signal dS1, dS2, dW1, dW2 : unsigned(15 downto 0) := (others => '0');
    signal rng     : unsigned(31 downto 0) := x"6D2B79F5";
    signal rn_rem  : unsigned(6 downto 0) := (others => '0');
    signal rn_b    : unsigned(7 downto 0) := (others => '0');
    signal rn_sp   : std_logic := '0';
    signal rn_tn   : unsigned(2 downto 0) := (others => '0');
    signal tl      : unsigned(15 downto 0) := x"ACE1";
    signal tx_acc  : unsigned(3 downto 0) := (others => '0');  -- virtual x mod 8, Q.1

    -- ==== pixel pipeline ====
    signal av      : std_logic_vector(1 to 8) := (others => '0');
    signal av12, av16 : std_logic := '0';
    signal zn2, zn10, zn11      : unsigned(2 downto 0) := Z_W;
    signal zA3, zA4, zA5, zA6 : std_logic := '0';
    -- s1
    signal p1_hA   : unsigned(7 downto 0) := (others => '0');
    signal p1_ma   : unsigned(15 downto 0) := (others => '0');
    signal c_ad1, c_ad2 : unsigned(8 downto 0) := (others => '0');
    signal p1_t1, p1_t2 : unsigned(8 downto 0) := (others => '0');
    signal p1_dxs, p1_dl, p1_dr : signed(12 downto 0) := (others => '0');
    signal p1_rf   : std_logic := '0';
    signal p1_rb, p1_g : unsigned(7 downto 0) := (others => '0');
    signal p1_sp   : std_logic := '0';
    signal p1_tn   : unsigned(2 downto 0) := (others => '0');
    signal p1_tA, p1_tC : signed(3 downto 0) := (others => '0');
    -- s2
    signal p2_iA   : unsigned(7 downto 0) := (others => '0');
    signal p2_ma   : unsigned(15 downto 0) := (others => '0');
    signal p2_sw   : signed(10 downto 0) := (others => '0');
    signal p2_dx   : unsigned(10 downto 0) := (others => '0');
    signal p2_dlc, p2_drc : unsigned(4 downto 0) := (others => '0');
    signal p2_md   : signed(8 downto 0) := (others => '0');
    signal p2_g    : unsigned(7 downto 0) := (others => '0');
    signal p2_sp, p2_rf : std_logic := '0';
    signal p2_tn   : unsigned(2 downto 0) := (others => '0');
    signal c_q1, c_q2 : signed(7 downto 0);
    -- s3
    signal p3_c1, p3_c2 : signed(7 downto 0) := (others => '0');
    signal p3_e    : signed(15 downto 0) := (others => '0');
    signal p3_swp  : unsigned(11 downto 0) := (others => '0');
    signal p3_swn  : std_logic := '0';
    signal p3_mp   : unsigned(11 downto 0) := (others => '0');
    signal p3_mpn  : std_logic := '0';
    signal p3_scA  : unsigned(3 downto 0) := (others => '0');
    signal p3_g    : unsigned(7 downto 0) := (others => '0');
    -- s4
    signal p4_a5   : unsigned(4 downto 0) := (others => '0');
    signal p4_hC   : unsigned(15 downto 0) := (others => '0');
    signal p4_m    : unsigned(7 downto 0) := (others => '0');
    signal p4_D    : unsigned(4 downto 0) := (others => '0');
    signal p4_scA  : unsigned(3 downto 0) := (others => '0');
    -- s5
    signal p5_a    : unsigned(4 downto 0) := (others => '0');
    signal p5_iC   : unsigned(7 downto 0) := (others => '0');
    signal p5_mc   : unsigned(15 downto 0) := (others => '0');
    signal p5_L    : unsigned(12 downto 0) := (others => '0');
    signal p5_scA  : unsigned(3 downto 0) := (others => '0');
    -- s6
    signal p6_r2   : unsigned(13 downto 0) := (others => '0');
    signal p6_mc   : unsigned(15 downto 0) := (others => '0');
    signal p6_Ys   : unsigned(9 downto 0) := (others => '0');
    signal p6_a    : unsigned(4 downto 0) := (others => '0');
    signal p6_scA  : unsigned(3 downto 0) := (others => '0');
    signal p6_dlc, p6_drc : unsigned(4 downto 0) := (others => '0');
    -- s7
    signal p7_gd   : unsigned(13 downto 0) := (others => '0');
    signal p7_rp   : unsigned(19 downto 0) := (others => '0');
    signal p7_hl, p7_hi : unsigned(15 downto 0) := (others => '0');
    signal p7_pd   : signed(14 downto 0) := (others => '0');
    signal p7_sc   : unsigned(3 downto 0) := (others => '0');
    signal p7_sl   : unsigned(1 downto 0) := (others => '0');
    signal p7_lift : unsigned(6 downto 0) := (others => '0');
    signal p7_Ys   : unsigned(9 downto 0) := (others => '0');
    signal p7_a    : unsigned(4 downto 0) := (others => '0');
    -- s8
    signal p8_gq   : unsigned(16 downto 0) := (others => '0');
    signal p8_ring : unsigned(7 downto 0) := (others => '0');
    signal p8_hE   : unsigned(7 downto 0) := (others => '0');
    signal p8_Ysh  : unsigned(9 downto 0) := (others => '0');
    signal p8_pa   : unsigned(1 downto 0) := (others => '0');
    signal p8_lift : unsigned(6 downto 0) := (others => '0');
    -- s9
    signal p9_t6   : unsigned(5 downto 0) := (others => '0');
    signal p9_ring : unsigned(7 downto 0) := (others => '0');
    signal p9_iE   : unsigned(7 downto 0) := (others => '0');
    signal p9_Yf   : unsigned(9 downto 0) := (others => '0');
    signal rA_ad, rC_ad : unsigned(7 downto 0) := (others => '0');
    signal rA_qy, rA_qu, rA_qv, rC_qy, rC_qu, rC_qv : unsigned(9 downto 0);
    -- s10
    signal p10_G2  : unsigned(5 downto 0) := (others => '0');
    signal p10_rt  : unsigned(7 downto 0) := (others => '0');
    signal p10_Yf  : unsigned(9 downto 0) := (others => '0');
    signal p10_sp, p10_rf : std_logic := '0';
    signal p10_tn  : unsigned(2 downto 0) := (others => '0');
    -- s11
    signal p11_Gv  : unsigned(4 downto 0) := (others => '0');
    signal p11_rl  : unsigned(7 downto 0) := (others => '0');
    signal p11_Yf  : unsigned(9 downto 0) := (others => '0');
    signal p11_sc  : unsigned(3 downto 0) := (others => '0');
    signal p11_a   : unsigned(4 downto 0) := (others => '0');
    signal p11_sp, p11_rf : std_logic := '0';
    signal p11_tn  : unsigned(2 downto 0) := (others => '0');
    -- s12: base colour, blend targets, raw eye colour
    signal p12_by, p12_bu, p12_bv : unsigned(9 downto 0) := (others => '0');
    signal p12_ty, p12_tu, p12_tv : unsigned(9 downto 0) := (others => '0');
    signal p12_a1  : unsigned(3 downto 0) := (others => '0');
    signal p12_aE  : unsigned(4 downto 0) := (others => '0');
    signal p12_rim : unsigned(4 downto 0) := (others => '0');
    signal p12_pa  : unsigned(1 downto 0) := (others => '0');
    signal rE_ad   : unsigned(7 downto 0) := (others => '0');
    signal rE_qy, rE_qu, rE_qv : unsigned(9 downto 0);
    signal p9_iC, p10_iE : unsigned(7 downto 0) := (others => '0');
    -- s13..s16: ONE blend toward the stronger target (eye/corona or the
    -- moire/haze/reflection target), as |difference| x 4-bit alpha
    signal p13_ty, p13_tu, p13_tv : unsigned(9 downto 0) := (others => '0');
    signal p13_by, p13_bu, p13_bv : unsigned(9 downto 0) := (others => '0');
    signal p13_a   : unsigned(4 downto 0) := (others => '0');
    signal p13_rim : unsigned(4 downto 0) := (others => '0');
    signal p14_my : unsigned(9 downto 0) := (others => '0');
    signal p14_mu, p14_mv : unsigned(7 downto 0) := (others => '0');
    signal p14_ny, p14_nu, p14_nv : std_logic := '0';
    signal p14_by, p14_bu, p14_bv : unsigned(9 downto 0) := (others => '0');
    signal p14_a   : unsigned(4 downto 0) := (others => '0');
    signal p14_rim : unsigned(4 downto 0) := (others => '0');
    signal p15_py : unsigned(13 downto 0) := (others => '0');
    signal p15_pu, p15_pv : unsigned(11 downto 0) := (others => '0');
    signal p15_ny, p15_nu, p15_nv : std_logic := '0';
    signal p15_by, p15_bu, p15_bv : unsigned(9 downto 0) := (others => '0');
    signal p15_rim : unsigned(4 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := (others => '0');
    signal p16_rim : unsigned(4 downto 0) := (others => '0');
    -- s17: composed pixel -> output buffer
    signal p17_y, p17_u, p17_v : unsigned(9 downto 0) := (others => '0');

    -- ==== output buffer (tear displacement) + sync delay ====
    type t_ram10 is array(0 to 255) of unsigned(9 downto 0);
    type t_ram4  is array(0 to 255) of std_logic_vector(3 downto 0);
    signal ob_y, ob_u, ob_v : t_ram10 := (others => (others => '0'));
    signal ob_s   : t_ram4 := (others => (others => '0'));
    signal ob_qy, ob_qu, ob_qv : unsigned(9 downto 0) := (others => '0');
    signal ob_qs  : std_logic_vector(3 downto 0) := (others => '0');
    attribute no_rw_check of ob_y : signal is true;
    attribute no_rw_check of ob_u : signal is true;
    attribute no_rw_check of ob_v : signal is true;
    attribute no_rw_check of ob_s : signal is true;

    -- ==== EBR delay lines for the pure pipeline delays (one EBR each,
    -- written every clock at dl_wa, read d clocks later; read and write
    -- addresses never coincide) ====
    type t_dl16 is array (0 to 15) of std_logic_vector(15 downto 0);
    signal dl7  : t_dl16 := (others => x"0003");
    signal dl4a, dl4b : t_dl16 := (others => (others => '0'));
    attribute ram_style of dl7  : signal is "block";
    attribute ram_style of dl4a : signal is "block";
    attribute ram_style of dl4b : signal is "block";
    attribute no_rw_check of dl7  : signal is true;
    attribute no_rw_check of dl4a : signal is true;
    attribute no_rw_check of dl4b : signal is true;
    signal dl_wa  : unsigned(3 downto 0) := (others => '0');
    signal dl_ra7 : unsigned(3 downto 0) := to_unsigned(9, 4);    -- dl_wa - 7 (8 clocks)
    signal dl_ra4 : unsigned(3 downto 0) := to_unsigned(13, 4);   -- dl_wa - 3
    signal dl7_q  : std_logic_vector(15 downto 0) := x"0003";
    signal dl4a_q, dl4b_q : std_logic_vector(15 downto 0) := (others => '0');

    -- ==== palettes: Y|U|V re-cut into a 16-bit (Y, U high) and a 14-bit
    -- (U low, V) ROM, each fitting one EBR: two EBRs per port, not three ====
    type t_palh is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_pall is array (0 to 255) of std_logic_vector(13 downto 0);
    function palh(y, u : t_rom256x10) return t_palh is
        variable r : t_palh;
    begin
        for i in 0 to 255 loop
            r(i) := std_logic_vector(y(i)) & std_logic_vector(u(i)(9 downto 4));
        end loop;
        return r;
    end function;
    function pall(u, v : t_rom256x10) return t_pall is
        variable r : t_pall;
    begin
        for i in 0 to 255 loop
            r(i) := std_logic_vector(u(i)(3 downto 0)) & std_logic_vector(v(i));
        end loop;
        return r;
    end function;
    constant C_PA_H : t_palh := palh(C_PA_Y, C_PA_U);
    constant C_PA_L : t_pall := pall(C_PA_U, C_PA_V);
    constant C_PS_H : t_palh := palh(C_PS_Y, C_PS_U);
    constant C_PS_L : t_pall := pall(C_PS_U, C_PS_V);
    signal rA_qh, rC_qh, rE_qh : std_logic_vector(15 downto 0);
    signal rA_ql, rC_ql, rE_ql : std_logic_vector(13 downto 0);
    signal o_wp   : unsigned(7 downto 0) := (others => '0');
    signal o_ray, o_rac, o_ras : unsigned(7 downto 0) := (others => '0');
    signal o_dY, o_dC : unsigned(7 downto 0) := to_unsigned(C_D0, 8);
    -- tear shift in virtual columns (v2.0.4): the line's shift d - D0 is
    -- scaled by 2/xs (x1 at 1080, x21/32 at 720p, x1/2 at SD); the chroma
    -- offset dc - d stays at pixel scale.  Per line, settles in 3 clocks.
    signal ts_s, ts_o, ts_sc, ts_o2 : signed(9 downto 0) := (others => '0');
    signal cu_ds, cu_dcs : unsigned(7 downto 0) := to_unsigned(C_D0, 8);
    signal o_lq   : std_logic := '0';
    signal o_lcnt : unsigned(4 downto 0) := (others => '0');
    signal s_io   : t_video_stream_yuv444_30b
        := ((others => '0'), (others => '0'), (others => '0'), '0', '1', '1', '1');

    -- ==== helpers ====
    function tri9(a : unsigned(9 downto 0)) return unsigned is
    begin
        if a(9) = '1' then
            return not a(8 downto 0);
        else
            return a(8 downto 0);
        end if;
    end function;

    function tri8(h : unsigned(7 downto 0)) return unsigned is
        variable r : unsigned(7 downto 0);
    begin
        if h(7) = '1' then
            r := (not h(6 downto 0)) & '1';
        else
            r := h(6 downto 0) & '0';
        end if;
        return r;
    end function;

    -- Opal: fold the spectrum into its cool third (green..violet)
    function opal(h : unsigned(7 downto 0); en : std_logic) return unsigned is
        variable t : unsigned(7 downto 0);
    begin
        if en = '0' then
            return h;
        end if;
        t := tri8(h);
        return to_unsigned(110, 8) + shift_right(t, 1) - shift_right(t, 4);
    end function;

    -- moire screen: phase -> brightness factor 0..15 (soft-edged bar)
    function screen(h : unsigned(7 downto 0)) return unsigned is
        variable t : unsigned(7 downto 0);
        variable s : unsigned(4 downto 0);
    begin
        t := tri8(h);
        s := t(7 downto 3);
        if s < 4 then
            return to_unsigned(0, 4);
        elsif s >= 19 then
            return to_unsigned(15, 4);
        else
            return resize(s - 4, 4);
        end if;
    end function;

    function clamp5(v : signed(12 downto 0)) return unsigned is
    begin
        if v(12) = '1' then
            return to_unsigned(0, 5);
        elsif v(11 downto 5) /= "0000000" then
            return to_unsigned(31, 5);
        else
            return unsigned(v(4 downto 0));
        end if;
    end function;

    function min5(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;

    -- seam tooth height scaled to the raster (v2.0.4): the 1080p tooth
    -- is -8..7 rows; 720p x2/3, 480-line SD x0.45, 576-line SD x0.53
    -- (class from the timing ID), so the torn seam stays the same
    -- fraction of the screen height and inside the gutter
    function f_tooth(t : signed(3 downto 0); c : unsigned(1 downto 0)) return signed is
    begin
        case c is
            when "01" =>
                case to_integer(t) is
                    when -8 | -7        => return to_signed(-5, 4);
                    when -6             => return to_signed(-4, 4);
                    when -5 | -4        => return to_signed(-3, 4);
                    when -3             => return to_signed(-2, 4);
                    when -2 | -1        => return to_signed(-1, 4);
                    when 1 | 2          => return to_signed( 1, 4);
                    when 3              => return to_signed( 2, 4);
                    when 4 | 5          => return to_signed( 3, 4);
                    when 6              => return to_signed( 4, 4);
                    when 7              => return to_signed( 5, 4);
                    when others => return to_signed(0, 4);
                end case;
            when "10" =>
                case to_integer(t) is
                    when -8             => return to_signed(-4, 4);
                    when -7 | -6        => return to_signed(-3, 4);
                    when -5 | -4        => return to_signed(-2, 4);
                    when -3 | -2        => return to_signed(-1, 4);
                    when 2 | 3          => return to_signed( 1, 4);
                    when 4 | 5          => return to_signed( 2, 4);
                    when 6 | 7          => return to_signed( 3, 4);
                    when others => return to_signed(0, 4);
                end case;
            when "11" =>
                case to_integer(t) is
                    when -8 | -7        => return to_signed(-4, 4);
                    when -6 | -5        => return to_signed(-3, 4);
                    when -4 | -3        => return to_signed(-2, 4);
                    when -2 | -1        => return to_signed(-1, 4);
                    when 1 | 2          => return to_signed( 1, 4);
                    when 3 | 4          => return to_signed( 2, 4);
                    when 5 | 6          => return to_signed( 3, 4);
                    when 7              => return to_signed( 4, 4);
                    when others => return to_signed(0, 4);
                end case;
            when others => return t;
        end case;
    end function;

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement (aurora): active width per line, active lines per
    -- field, interlace + field parity at vsync.
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt /= 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt /= 0 then s_meas_v <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                s_ilace <= s_tid_il;
                s_botf  <= not s_timing.field_n;      -- bottom field = field_n '0'
                if s_tid_il = '1' then
                    s_fh <= s_meas_v(10 downto 0) & '0';
                else
                    s_fh <= s_meas_v;
                end if;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;
        end if;
    end process p_measure;

    -- timing-ID classes (static while a mode runs)
    p_tid : process(clk)
    begin
        if rising_edge(clk) then
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => s_tid_il <= '1';
                when others             => s_tid_il <= '0';
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 5 | 6 | 14 => s_tcls <= "01"; s_xs <= to_unsigned(3, 3);   -- 720p
                when 0 | 4      => s_tcls <= "10"; s_xs <= to_unsigned(4, 3);   -- 480 lines
                when 8 | 12     => s_tcls <= "11"; s_xs <= to_unsigned(4, 3);   -- 576 lines
                when others     => s_tcls <= "00"; s_xs <= to_unsigned(2, 3);   -- 1080
            end case;
        end if;
    end process p_tid;

    -- ====================================================================
    -- Generated avid (aurora v3.0.2, HW-confirmed): start 2-3 clocks after
    -- the source's first avid rise at the voted usual hsync parity, run the
    -- field-latched width.  s_bcol counts it from hsync.
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

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

            if g_hsr = '1' then
                s_bcol <= (others => '0');
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Field tick (first vsync edge after active video: analog vsync
    -- serrates), switch latch, line trigger (the drawn line has left the
    -- part of the pixel pipe that reads the line registers), random source.
    -- ====================================================================
    p_ctl : process(clk)
    begin
        if rising_edge(clk) then
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;
            if s_vs_go = '1' then
                g_wm1    <= s_meas_h - 1;
                sw_tear  <= registers_in(6)(0);
                sw_iris  <= registers_in(6)(1);
                sw_moire <= registers_in(6)(2);
                sw_opal  <= registers_in(6)(4);
            end if;

            tq <= g_avid;
            if tq = '1' and g_avid = '0' then
                tcnt <= to_unsigned(20, 5);
            elsif tcnt /= 0 then
                tcnt <= tcnt - 1;
            end if;

            if s_lfsr(0) = '1' then
                s_lfsr <= ('0' & s_lfsr(15 downto 1)) xor x"B400";
            else
                s_lfsr <= '0' & s_lfsr(15 downto 1);
            end if;
        end if;
    end process p_ctl;

    -- ====================================================================
    -- MICROCODED ENGINE (port of cubist's p_ueng, HW-validated).
    --
    -- One ALU, the register file in two block-RAM copies (dual read), the
    -- program in block RAM.  DECODE -> LATCH -> EXEC: three clocks per
    -- straight-line instruction; a taken jump refetches (4).  The ROM reads
    -- pc+1 during EXEC so the next instruction decodes on the next clock.
    -- Forwarding covers the previous result landing on the same edge as
    -- this instruction's register read (iCE40 block RAM returns undefined
    -- data for a read-during-write).  Every unit launches at LATCH from the
    -- register-file outputs; EXEC reads only registers.
    -- ====================================================================
    p_ueng : process(clk)
        variable v_r      : signed(23 downto 0);
        variable v_op     : integer range 0 to 31;
        variable v_op2    : integer range 0 to 31;
        variable v_stl    : boolean;
        variable v_jmp    : boolean;
        variable v_dr     : unsigned(12 downto 0);
        variable v_dd     : unsigned(13 downto 0);
        variable v_ms     : unsigned(16 downto 0);
        variable v_pr     : unsigned(27 downto 0);
        variable v_fa, v_fb : std_logic_vector(23 downto 0);
        variable v_sum    : signed(24 downto 0);
        variable v_s26    : signed(25 downto 0);
        variable v_blt    : std_logic;
        variable v_bt     : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            u_wlast <= u_we;
            if s_vs_go = '1' then u_fpend <= '1'; end if;
            if tcnt = 1 then u_lpend <= '1'; end if;

            -- serial multiply, 12 steps: {mu_ph, mu_pl} = mu_pa * B
            if mu_bsy = '1' then
                if mu_pl(0) = '1' then
                    v_ms := mu_ph + resize(mu_pa, 17);
                else
                    v_ms := mu_ph;
                end if;
                mu_ph <= '0' & v_ms(16 downto 1);
                mu_pl <= v_ms(0) & mu_pl(11 downto 1);
                if mu_cnt = 0 then mu_bsy <= '0';
                else mu_cnt <= mu_cnt - 1; end if;
            end if;

            -- serial restoring divider: the numerator shifts out of the top
            -- of dv_na as the quotient shifts in at the bottom
            if dv_bsy = '1' and dv_post = '0' then
                v_dr := dv_ra(11 downto 0) & dv_na(24);
                v_dd := ('0' & v_dr) - ("00" & dv_da);      -- borrow = v_dr < dv_da
                if v_dd(13) = '0' then
                    dv_ra <= v_dd(12 downto 0);
                    dv_na <= dv_na(23 downto 0) & '1';
                else
                    dv_ra <= v_dr;
                    dv_na <= dv_na(23 downto 0) & '0';
                end if;
                if dv_cnt = 0 then dv_post <= '1';
                else dv_cnt <= dv_cnt - 1; end if;
            end if;
            if dv_post = '1' then
                if dv_na(24 downto 20) /= 0 then dv_ovf <= '1';
                else dv_ovf <= '0'; end if;
                dv_post <= '0';
                dv_bsy  <= '0';
            end if;

            -- line-register block transfer: rf[base + i] arrives a clock
            -- after its address
            xf_d1 <= xf_run;
            xf_i1 <= xf_i;
            if xf_run = '1' then
                if xf_i = C_NXF - 1 then xf_run <= '0'; end if;
                xf_i <= xf_i + 1;
            end if;

            u_we <= '0';

            case to_integer(u_st) is

                -- FETCH (after a jump only): the ROM answers next cycle
                when 0 =>
                    u_st <= "01";

                -- DECODE: latch the instruction; the register-file reads
                -- are addressed straight off the ROM output this cycle
                when 1 =>
                    u_ir <= u_rom;
                    u_st <= "10";

                -- OPERAND LATCH: block-RAM outputs into fabric (forwarding
                -- the previous result, which lands on this read's edge).
                -- Nothing else reads the forwarded nets: every compare,
                -- branch and unit launch works from these two registers.
                when 2 =>
                    v_op2 := to_integer(unsigned(u_ir(31 downto 27)));
                    if u_wlast = '1' and unsigned(u_ir(18 downto 11)) = u_wa then
                        v_fa := u_wd;
                    else
                        v_fa := u_rda;
                    end if;
                    if u_wlast = '1' and unsigned(u_ir(10 downto 3)) = u_wa then
                        v_fb := u_wd;
                    else
                        v_fb := u_rdb;
                    end if;
                    u_sh <= signed(v_fa);
                    -- SUB, compares and MIN/MAX use A + not B + 1
                    case v_op2 is
                        when 3 | 20 | 21 | 28 | 29 =>
                            u_b <= signed(not v_fb);  u_cin <= '1';
                        when others =>
                            u_b <= signed(v_fb);      u_cin <= '0';
                    end case;
                    if v_op2 = 28 then u_ismin <= '1'; else u_ismin <= '0'; end if;
                    u_ctlr <= u_ctlv;
                    case v_op2 is
                        when 1 | 4 | 5    => u_rsel <= x"0"; u_rwr <= '1';   -- MOV SHR SHL
                        when 2 | 3        => u_rsel <= x"1"; u_rwr <= '1';   -- ADD SUB
                        when 27           => u_rsel <= x"2"; u_rwr <= '1';   -- AND
                        when 15           => u_rsel <= x"3"; u_rwr <= '1';   -- LDI
                        when 11           => u_rsel <= x"4"; u_rwr <= '1';   -- MRD
                        when 13           => u_rsel <= x"5"; u_rwr <= '1';   -- DRD
                        when 14 | 16 | 23 => u_rsel <= x"6"; u_rwr <= '1';   -- SIN PRV PRC
                        when 28 | 29      => u_rsel <= x"7"; u_rwr <= '1';   -- MIN MAX
                        when 25           => u_rsel <= x"8"; u_rwr <= '1';   -- CTL
                        when others       => u_rsel <= x"0"; u_rwr <= '0';
                    end case;
                    u_shc <= unsigned(u_ir(6 downto 3));
                    u_wa  <= unsigned(u_ir(26 downto 19));
                    u_st  <= "11";

                -- EXECUTE.  Everything this state reads is a register.
                when others =>
                    v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                    v_stl := false;
                    v_jmp := false;
                    -- the one adder: A + B (+1); its 25th bit is A < B for
                    -- the compares (B arrived inverted)
                    -- (one carry chain: the carry-in rides in a guard bit)
                    v_s26 := (u_sh(23) & u_sh & '1') + (u_b(23) & u_b & u_cin);
                    v_sum := v_s26(25 downto 1);
                    v_blt := v_sum(24);
                    v_bt  := not u_b;                    -- B again, for MIN/MAX

                    case to_integer(u_rsel) is
                        when 0 => v_r := u_sh;
                        when 1 => v_r := v_sum(23 downto 0);
                        when 2 => v_r := u_sh and signed(resize(unsigned(u_ir(10 downto 0)), 24));
                        when 3 => v_r := resize(signed(u_ir(18 downto 0)), 24);
                        when 4 =>                    -- MRD: 28-bit product (>> 12)
                            v_pr := mu_ph(15 downto 0) & mu_pl;
                            if u_ir(0) = '1' then v_r := signed(resize(v_pr(27 downto 12), 24));
                            else                  v_r := signed(v_pr(23 downto 0)); end if;
                        when 5 =>                    -- DRD: clamped magnitude
                            if dv_ovf = '1' then
                                v_r := to_signed(1048575, 24);
                            else
                                v_r := signed(resize(dv_na(19 downto 0), 24));
                            end if;
                        when 6 => v_r := resize(u_romr, 24);
                        when 7 => if (u_bltr xor u_ismin) = '0' then v_r := u_sh; else v_r := v_bt; end if;
                        when others => v_r := u_ctlr;
                    end case;

                    case v_op is
                        when 4 | 5 =>                  -- SHR / SHL, serial
                            if u_shc /= 0 then
                                if v_op = 4 then u_sh <= shift_right(u_sh, 1);
                                else             u_sh <= shift_left(u_sh, 1); end if;
                                u_shc <= u_shc - 1;
                                v_stl := true;
                            end if;
                        when 10 =>                     -- MUL (magnitudes)
                            mu_pa  <= unsigned(u_sh(15 downto 0));
                            mu_pl  <= unsigned(u_b(11 downto 0));
                            mu_ph  <= (others => '0');
                            mu_cnt <= to_unsigned(11, 4);
                            mu_bsy <= '1';
                        when 12 =>                     -- DIV (magnitudes)
                            dv_na <= resize(unsigned(u_sh), 25);
                            if unsigned(u_b(11 downto 0)) = 0 then
                                dv_da <= to_unsigned(1, 12);
                            else
                                dv_da <= unsigned(u_b(11 downto 0));
                            end if;
                            dv_ra   <= (others => '0');
                            dv_cnt  <= to_unsigned(24, 5);
                            dv_bsy  <= '1';
                        when 28 | 29 =>                -- MIN / MAX: compare, then select
                            if u_gwt = 0 then
                                u_bltr <= v_blt;
                                u_gwt  <= "001";
                                v_stl  := true;
                            else
                                u_gwt <= (others => '0');
                            end if;
                        when 11 => if mu_bsy = '1' then v_stl := true; end if;
                        when 13 => if dv_bsy = '1' then v_stl := true; end if;
                        when 14 | 16 | 23 =>           -- ROM reads
                            -- 0: address; 1: EBR reads; 2: into fabric;
                            -- 3: into the result source; 4: done
                            if u_gwt = 0 then
                                qs_ang <= unsigned(u_sh(9 downto 0));
                                pr_a   <= unsigned(u_sh(7 downto 0));
                            end if;
                            if u_gwt = 3 then
                                case v_op is
                                    when 14     => if qs_n2 = '1' then u_romr <= -signed(resize(qs_d, 14));
                                                   else u_romr <= signed(resize(qs_d, 14)); end if;
                                    when 16     => u_romr <= signed(resize(pr_dv, 14));
                                    when others => u_romr <= signed(resize(pr_dc, 14));
                                end case;
                            end if;
                            if u_gwt = 4 then
                                u_gwt <= (others => '0');
                            else
                                u_gwt <= u_gwt + 1;
                                v_stl := true;
                            end if;
                        when 17 => null;               -- OUT: p_ports writes from u_sh
                        when 22 =>                     -- XFER: stall until done
                            if u_gwt = 0 then
                                xf_run <= '1';
                                xf_i   <= (others => '0');
                                u_gwt  <= "001";
                                v_stl  := true;
                            elsif xf_run = '1' or xf_d1 = '1' then
                                v_stl := true;
                            else
                                u_gwt <= (others => '0');
                            end if;
                        when 24 =>                     -- WAIT
                            if u_ir(11) = '0' then
                                if u_fpend = '1' then
                                    u_fpend <= '0';
                                    u_lpend <= '0';
                                else
                                    v_stl := true;
                                end if;
                            else
                                if u_fpend = '1' then
                                    v_jmp := true;          -- restart the field program
                                    u_pc  <= (others => '0');
                                elsif u_lpend = '1' then
                                    u_lpend <= '0';
                                else
                                    v_stl := true;
                                end if;
                            end if;
                        when 18 => v_jmp := true;
                                   u_pc <= unsigned(u_ir(8 downto 0));
                        when 19 => if u_sh /= 0 then
                                       v_jmp := true;
                                       u_pc <= unsigned(u_ir(8 downto 0));
                                   end if;
                        when 20 => if v_blt = '1' then
                                       v_jmp := true;
                                       u_pc <= unsigned(u_ir(24 downto 19)) & unsigned(u_ir(2 downto 0));
                                   end if;
                        when 21 => if v_blt = '0' then
                                       v_jmp := true;
                                       u_pc <= unsigned(u_ir(24 downto 19)) & unsigned(u_ir(2 downto 0));
                                   end if;
                        when 26 => v_jmp := true;      -- JR
                                   u_pc <= unsigned(u_sh(8 downto 0));
                        when others => null;
                    end case;

                    if v_stl then
                        u_st <= "11";
                    elsif v_jmp then
                        u_st <= "00";
                    else
                        u_st <= "01";
                        u_pc <= u_pc + 1;
                    end if;

                    u_wd <= std_logic_vector(v_r);
                    if u_rwr = '1' and not v_stl then
                        u_we <= '1';
                    end if;
            end case;
        end if;
    end process p_ueng;

    -- CTL reads the outside world (u_ir is stable from DECODE)
    with to_integer(unsigned(u_ir(14 downto 11))) select u_ctlv <=
        signed(resize(unsigned(registers_in(0)), 24)) when 0,
        signed(resize(unsigned(registers_in(1)), 24)) when 1,
        signed(resize(unsigned(registers_in(2)), 24)) when 2,
        signed(resize(unsigned(registers_in(3)), 24)) when 3,
        signed(resize(unsigned(registers_in(4)), 24)) when 4,
        signed(resize(unsigned(registers_in(5)), 24)) when 5,
        signed(resize(unsigned(registers_in(6)), 24)) when 6,
        signed(resize(unsigned(registers_in(7)), 24)) when 7,
        signed(resize(s_fh, 24))                      when 8,
        signed(resize(s_meas_h, 24))                  when 9,
        (0 => s_ilace, others => '0')                 when 10,
        (0 => s_botf, others => '0')                  when 11,
        signed(resize(s_lfsr, 24))                    when 12,
        (others => '0')                               when others;

    -- the ROM reads pc+1 during EXEC, so a straight-line instruction
    -- decodes on the very next clock
    u_fa <= u_pc + 1 when u_st = "11" else u_pc;

    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_fa));
        end if;
    end process p_urom;

    -- register file: the A port also serves XFER
    p_rfa : process(clk)
        variable v_ra : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if u_we = '1' then
                rf_al(to_integer(u_wa)) <= u_wd(15 downto 0);
                rf_ah(to_integer(u_wa)) <= u_wd(23 downto 16);
            end if;
            if xf_run = '1' then
                v_ra := to_unsigned(C_XFER_BASE, 8) + resize(xf_i, 8);
            else
                v_ra := unsigned(u_rom(18 downto 11));
            end if;
            u_rdal <= rf_al(to_integer(v_ra));
            u_rdah <= rf_ah(to_integer(v_ra));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then
                rf_bl(to_integer(u_wa)) <= u_wd(15 downto 0);
                rf_bh(to_integer(u_wa)) <= u_wd(23 downto 16);
            end if;
            u_rdbl <= rf_bl(to_integer(unsigned(u_rom(10 downto 3))));
            u_rdbh <= rf_bh(to_integer(unsigned(u_rom(10 downto 3))));
        end if;
    end process p_rfb;

    u_rda <= u_rdah & u_rdal;
    u_rdb <= u_rdbh & u_rdbl;

    -- engine ROM ports (one inferred EBR each)
    p_qrom : process(clk)
        variable v_i : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if qs_ang(8) = '1' then v_i := not qs_ang(7 downto 0); else v_i := qs_ang(7 downto 0); end if;
            qs_q  <= C_QSIN(to_integer(v_i));
            qs_n1 <= qs_ang(9);
            qs_d  <= qs_q;
            qs_n2 <= qs_n1;
        end if;
    end process p_qrom;

    p_prv : process(clk) begin if rising_edge(clk) then pr_qv <= C_PVES(to_integer(pr_a)); pr_dv <= pr_qv; end if; end process;
    p_prc : process(clk) begin if rising_edge(clk) then pr_qc <= C_PCIR(to_integer(pr_a)); pr_dc <= pr_qc; end if; end process;

    -- ====================================================================
    -- OUT ports: per-field constants for the pixel path (gen/oasm.py PORTS)
    -- ====================================================================
    p_ports : process(clk)
        variable v : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            if u_st = "11" and u_ir(31 downto 27) = "10001" then
                v := unsigned(u_sh);
                case to_integer(unsigned(u_ir(24 downto 19))) is
                    when 0  => f_cx    <= v(10 downto 0);
                    when 2  => f_cA    <= v(23 downto 0);
                    when 4  => f_kS1   <= v(9 downto 0);
                    when 5  => f_kS2   <= v(9 downto 0);
                    when 6  => f_kW1x  <= v(9 downto 0);
                    when 7  => f_kW2x  <= v(9 downto 0);
                    when 10 => t_M     <= v(15 downto 0);
                    when 11 => t_G     <= v(15 downto 0);
                    when 12 => t_Rg    <= v(15 downto 0);
                    when 13 => f_spk   <= v(7 downto 0);
                    when 14 => f_lenm  <= v(5 downto 0);
                    when 15 => f_as4   <= v(3 downto 0);
                    when 16 => f_kR    <= v(5 downto 0);
                    when 17 => f_se    <= v(2 downto 0);
                    when 18 => f_sm    <= v(1 downto 0);
                    when 19 => f_g3    <= v(2 downto 0);
                    when 20 => f_rsh   <= v(1 downto 0);
                    when 21 => f_pup2  <= v(13 downto 0);
                    when 22 => f_boost <= v(5 downto 0);
                    when 23 => f_wx0   <= v(10 downto 0);
                    when 24 => f_wx1m  <= v(10 downto 0);
                    when 25 => f_xhm1  <= v(10 downto 0);
                    when 26 => f_tseed <= v(15 downto 0);
                    when 29 => f_yA    <= v(11 downto 0);
                    when 30 => f_yC    <= v(11 downto 0);
                    when 31 => f_wy0   <= v(11 downto 0);
                    when 32 => f_wy1   <= v(11 downto 0);
                    when 35 => f_hs2   <= v(17 downto 0);
                    when 36 => f_hxm1  <= signed(v(11 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_ports;

    -- ====================================================================
    -- XFER loader: the line-register block, one word a clock
    -- (gen/oasm.py XFER_FIELDS order)
    -- ====================================================================
    p_xfer : process(clk)
        variable v : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            if xf_d1 = '1' then
                v := unsigned(u_rda);
                case to_integer(xf_i1) is
                    when 0  => cu_rc   <= v(11 downto 0);
                    when 4  => cu_v2   <= v(13 downto 0);
                    when 5  => cu_lb   <= v(15 downto 0);
                    when 6  => cu_w4   <= v(13 downto 0);
                    when 7  => cu_eye  <= v(0);
                    when 8  => cu_k    <= v(2 downto 0);
                    when 9  => cu_phC  <= v(15 downto 0);
                    when 10 => cu_swA  <= v(6 downto 0);
                    when 11 => cu_ah   <= v(4 downto 0);
                    when 12 => cu_aref <= v(4 downto 0);
                    when 13 => cu_ref  <= v(0);
                    when 14 => cu_rx0  <= v(10 downto 0);
                    when 15 => cu_rx1  <= v(10 downto 0);
                    when 16 => cu_d    <= v(7 downto 0);
                    when 17 => cu_dc   <= v(7 downto 0);
                    when others => null;
                end case;
            end if;
        end if;
    end process p_xfer;

    -- ====================================================================
    -- Zone / window / seam flags from the line's row (hardwired; settles
    -- in two clocks after XFER, well before the line starts)
    -- ====================================================================
    -- (one 13-bit difference per bound; every flag is a sign / bit test on
    -- it.  The original 12-bit "wy0 + 2" and "rc + 2" wrap above 4093, which
    -- the all-ones guards reproduce.)
    p_zone : process(clk)
        variable v_dA, v_dC, v_dt, v_db : signed(12 downto 0);
        variable v_t0, v_b0 : std_logic;
    begin
        if rising_edge(clk) then
            v_dA := signed('0' & cu_rc) - signed('0' & f_yA);
            v_dC := signed('0' & cu_rc) - signed('0' & f_yC);
            v_dt := signed('0' & cu_rc) - signed('0' & f_wy0);
            v_db := signed('0' & f_wy1) - signed('0' & cu_rc) - 1;
            cu_zA <= v_dA(12);
            cu_zC <= not v_dC(12);
            cu_win <= not v_dt(12) and not v_db(12);
            if v_dt(12 downto 1) = 0 and f_wy0(11 downto 1) /= "11111111111" then v_t0 := '1'; else v_t0 := '0'; end if;
            if v_db(12 downto 1) = 0 and cu_rc(11 downto 1) /= "11111111111" then v_b0 := '1'; else v_b0 := '0'; end if;
            cu_rr <= v_t0 or v_b0;
            cu_dt <= clamp5(v_dt);
            cu_db <= clamp5(v_db);
            if v_dA(12 downto 4) = "000000000" or v_dA(12 downto 4) = "111111111" then cu_nA <= '1'; else cu_nA <= '0'; end if;
            cu_sA <= resize(v_dA(4 downto 0), 6);
            if v_dC(12 downto 4) = "000000000" or v_dC(12 downto 4) = "111111111" then cu_nC <= '1'; else cu_nC <= '0'; end if;
            cu_sC <= resize(v_dC(4 downto 0), 6);
        end if;
    end process p_zone;

    -- ====================================================================
    -- Stage 0: per-pixel DDAs (loaded from the line/frame parameters in
    -- blanking, stepped on the generated avid), RNG, streak runs, teeth.
    -- ====================================================================
    p_s0 : process(clk)
        variable v : unsigned(31 downto 0);
        variable v_fr : unsigned(25 downto 0);
        variable v5   : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- chirped bars: frequency falls toward the centre, then rises
            if g_avid = '1' then
                dA_ph <= dA_ph + dA_fr;
                -- dA_fr -/+ f_cA on one carry chain (A + not B + 1)
                v_fr := ('0' & dA_fr & '1') + ('0' & (f_cA xor (23 downto 0 => dA_lf)) & dA_lf);
                dA_fr <= v_fr(24 downto 1);
                if s_bcol < f_xhm1 then dA_lf <= '1'; else dA_lf <= '0'; end if;
                dS1 <= dS1 + f_kS1;
                dS2 <= dS2 - f_kS2;
                dW1 <= dW1 + f_kW1x;
                dW2 <= dW2 - f_kW2x;
            else
                -- blanking: the line's start values arrive by XFER (straight
                -- from the engine's register file, no holding registers)
                dA_lf <= '1';
                if xf_d1 = '1' then
                    case to_integer(xf_i1) is
                        when 1  => dA_ph <= unsigned(u_rda(23 downto 8)) & x"00";
                        when 2  => dS1   <= unsigned(u_rda(15 downto 0));
                        when 3  => dS2   <= unsigned(u_rda(15 downto 0));
                        when 18 => dA_fr <= unsigned(u_rda(23 downto 0));
                        when 19 => dW1   <= unsigned(u_rda(15 downto 0));
                        when 20 => dW2   <= unsigned(u_rda(15 downto 0));
                        when others => null;
                    end case;
                end if;
            end if;

            -- xorshift32, every clock
            v := rng;
            v := v xor shift_left(v, 13);
            v := v xor shift_right(v, 17);
            v := v xor shift_left(v, 5);
            rng <= v;

            -- streak runs: even lengths from x = 0, so a run never splits
            -- a 4:2:2 chroma pair
            if g_avid = '0' then
                rn_rem <= (others => '0');
            elsif rn_rem = 0 then
                rn_rem <= (rng(13 downto 8) and f_lenm) & '1';
                rn_b   <= rng(21 downto 14);
                if rng(29 downto 22) < f_spk then rn_sp <= '1'; else rn_sp <= '0'; end if;
                rn_tn  <= rng(31 downto 30) & rng(0);
            else
                rn_rem <= rn_rem - 1;
            end if;

            -- seam teeth: one random step per 8 VIRTUAL columns (8 px at
            -- 1080p, 5.3 at 720p, 4 at SD), same on every row
            v5 := ('0' & tx_acc) + resize(s_xs, 5);
            if g_avid = '0' then
                tl <= f_tseed;
                tx_acc <= (others => '0');
            else
                tx_acc <= v5(3 downto 0);
            end if;
            if g_avid = '1' and v5(4) = '1' then
                if tl(0) = '1' then
                    tl <= ('0' & tl(15 downto 1)) xor x"B400";
                else
                    tl <= '0' & tl(15 downto 1);
                end if;
            end if;
        end if;
    end process p_s0;

    -- ====================================================================
    -- rho^2 DDA, aligned to stage 5: u^2 = s*(x - cx)^2 in Q24 by second
    -- differences (two adders, exact), from n columns left of the eye
    -- (n = min(cx, 4 Wa): further out rho^2 >= 16 anyway); holds 16.0
    -- before the start and once it passes 16 on the far side.  Start value,
    -- step and column come from the microcode once per field.
    -- ====================================================================
    p_rho : process(clk)
        variable v : signed(29 downto 0);
    begin
        if rising_edge(clk) then
            if av(5) = '0' then
                h_x <= (others => '0');
                if xf_d1 = '1' and xf_i1 = 21 then h_d <= resize(signed(u_rda), 28); end if;
                if xf_d1 = '1' and xf_i1 = 22 then h_q <= shift_left(resize(unsigned(u_rda(20 downto 0)), 29), 8); end if;
                if f_hxm1 = -1 then h_run <= '1'; else h_run <= '0'; end if;
            else
                h_x <= h_x + 1;
                if h_run = '1' then
                    v := signed(resize(h_q, 30)) + resize(h_d, 30);
                    if v(29) = '1' then
                        h_q <= (others => '0');
                    elsif v(28) = '1' then
                        -- 16.0 exactly: h_q(28:14) + v^2 must not wrap 15 bits
                        h_q <= to_unsigned(2**28 - 1, 29);
                        if h_d(27) = '0' and h_d /= 0 then h_run <= '0'; h_sat <= '1'; end if;
                    else
                        h_q <= unsigned(v(28 downto 0));
                    end if;
                    h_d <= h_d + signed(resize(f_hs2, 28));
                elsif signed(resize(h_x, 12)) = f_hxm1 then
                    h_run <= '1';                 -- h_q has held Q0 since XFER
                end if;
            end if;
            -- before the start column the field reads 16.0
            if av(5) = '0' then
                h_sat <= '0';
            end if;
        end if;
    end process p_rho;

    -- ==== cloud sine ROMs (512 x 8, one EBR each) ====
    p_cr1 : process(clk) begin if rising_edge(clk) then c_q1 <= C_CSIN(to_integer(c_ad1)); end if; end process;
    p_cr2 : process(clk) begin if rising_edge(clk) then c_q2 <= C_CSIN(to_integer(c_ad2)); end if; end process;

    -- ==== palette ports: A (bars + sparks), C (bands), E (eye + corona) ====
    p_rah : process(clk) begin if rising_edge(clk) then rA_qh <= C_PA_H(to_integer(rA_ad)); end if; end process;
    p_ral : process(clk) begin if rising_edge(clk) then rA_ql <= C_PA_L(to_integer(rA_ad)); end if; end process;
    p_rch : process(clk) begin if rising_edge(clk) then rC_qh <= C_PS_H(to_integer(rC_ad)); end if; end process;
    p_rcl : process(clk) begin if rising_edge(clk) then rC_ql <= C_PS_L(to_integer(rC_ad)); end if; end process;
    p_reh : process(clk) begin if rising_edge(clk) then rE_qh <= C_PS_H(to_integer(rE_ad)); end if; end process;
    p_rel : process(clk) begin if rising_edge(clk) then rE_ql <= C_PS_L(to_integer(rE_ad)); end if; end process;
    rA_qy <= unsigned(rA_qh(15 downto 6));  rA_qu <= unsigned(rA_qh(5 downto 0)) & unsigned(rA_ql(13 downto 10));
    rA_qv <= unsigned(rA_ql(9 downto 0));
    rC_qy <= unsigned(rC_qh(15 downto 6));  rC_qu <= unsigned(rC_qh(5 downto 0)) & unsigned(rC_ql(13 downto 10));
    rC_qv <= unsigned(rC_ql(9 downto 0));
    rE_qy <= unsigned(rE_qh(15 downto 6));  rE_qu <= unsigned(rE_qh(5 downto 0)) & unsigned(rE_ql(13 downto 10));
    rE_qv <= unsigned(rE_ql(9 downto 0));

    -- ==== pipeline delay lines ====
    --   dl7  (7 clocks): iA s2 -> rA_ad (s9), sp/tn/rf s2 -> s9, zone s2 -> s9
    --   dl4a (4 clocks): iC s5 -> rC_ad (s9), sc s7 -> s11, pa s8 -> s12, av 12 -> 16
    --   dl4b (4 clocks): dlc/drc s2 -> s6, a s7 -> s11, av 8 -> 12
    p_dlp : process(clk)
    begin
        if rising_edge(clk) then
            dl_wa  <= dl_wa + 1;
            dl_ra7 <= dl_ra7 + 1;
            dl_ra4 <= dl_ra4 + 1;
        end if;
    end process p_dlp;
    p_dl7 : process(clk)
    begin
        if rising_edge(clk) then
            dl7(to_integer(dl_wa)) <= std_logic_vector(p2_iA) & p2_sp & std_logic_vector(p2_tn) & p2_rf
                                      & std_logic_vector(zn2);
            dl7_q <= dl7(to_integer(dl_ra7));
        end if;
    end process p_dl7;
    p_dl4a : process(clk)
    begin
        if rising_edge(clk) then
            dl4a(to_integer(dl_wa)) <= std_logic_vector(p5_iC) & std_logic_vector(p7_sc) & std_logic_vector(p8_pa)
                                       & av12 & '0';
            dl4a_q <= dl4a(to_integer(dl_ra4));
        end if;
    end process p_dl4a;
    p_dl4b : process(clk)
    begin
        if rising_edge(clk) then
            dl4b(to_integer(dl_wa)) <= std_logic_vector(p2_dlc) & std_logic_vector(p2_drc) & std_logic_vector(p7_a)
                                       & av(8);
            dl4b_q <= dl4b(to_integer(dl_ra4));
        end if;
    end process p_dl4b;
    rA_ad  <= unsigned(dl7_q(15 downto 8));
    p10_sp <= dl7_q(7);
    p10_tn <= unsigned(dl7_q(6 downto 4));
    p10_rf <= dl7_q(3);
    zn10   <= unsigned(dl7_q(2 downto 0));
    p9_iC  <= unsigned(dl4a_q(15 downto 8));
    p11_sc <= unsigned(dl4a_q(7 downto 4));
    p12_pa <= unsigned(dl4a_q(3 downto 2));
    av16   <= dl4a_q(1);
    p6_dlc <= unsigned(dl4b_q(15 downto 11));
    p6_drc <= unsigned(dl4b_q(10 downto 6));
    p11_a  <= unsigned(dl4b_q(5 downto 1));
    av12   <= dl4b_q(0);

    -- ====================================================================
    -- Pixel pipeline, stages 1..17 (one op per stage after each EBR read,
    -- every multiply in a stage of its own).
    -- ====================================================================
    p_pix : process(clk)
        variable v_inA, v_inC, v_inx : std_logic;
        variable v_s6  : signed(6 downto 0);
        variable v_z   : unsigned(2 downto 0);
        variable v_ab  : signed(12 downto 0);
        variable v_e12 : signed(11 downto 0);
        variable v_ms  : signed(9 downto 0);
        variable v_cs  : signed(9 downto 0);
        variable v_r2  : unsigned(14 downto 0);
        variable v_10  : unsigned(9 downto 0);
        variable v_11  : unsigned(10 downto 0);
        variable v_t8  : unsigned(7 downto 0);
        variable v_m   : unsigned(4 downto 0);
        variable v_gp  : unsigned(16 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_df  : signed(10 downto 0);
        variable v_sw  : signed(10 downto 0);
        variable v_md  : signed(8 downto 0);
        variable v_13  : unsigned(12 downto 0);
        variable v_a11 : unsigned(10 downto 0);
        variable v_py  : unsigned(13 downto 0);
        variable v_pu, v_pv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            av <= g_avid & av(1 to 7);

            -- ---- s1 ----
            p1_hA <= dA_ph(23 downto 16);
            p1_ma <= dA_ph(23 downto 8) + resize(dA_ph(23 downto 13), 16);
            c_ad1 <= dS1(15 downto 7);
            c_ad2 <= dS2(15 downto 7);
            p1_t1 <= tri9(dW1(15 downto 6));
            p1_t2 <= tri9(dW2(15 downto 6));
            p1_dxs <= signed(resize(s_bcol, 13)) - signed(resize(f_cx, 13));
            p1_dl  <= signed(resize(s_bcol, 13)) - signed(resize(f_wx0, 13));
            p1_dr  <= signed(resize(f_wx1m, 13)) - signed(resize(s_bcol, 13));
            if s_bcol >= cu_rx0 and s_bcol <= cu_rx1 then
                p1_rf <= cu_ref;
            else
                p1_rf <= '0';
            end if;
            if rn_rem = 0 then
                p1_rb <= rng(21 downto 14);
                if rng(29 downto 22) < f_spk then p1_sp <= '1'; else p1_sp <= '0'; end if;
                p1_tn <= rng(31 downto 30) & rng(0);
            else
                p1_rb <= rn_b;
                p1_sp <= rn_sp;
                p1_tn <= rn_tn;
            end if;
            p1_g  <= rng(7 downto 0);
            p1_tA <= f_tooth(signed(std_logic_vector'(tl(15) & tl(11) & tl(7) & tl(3))), s_tcls);
            p1_tC <= f_tooth(signed(std_logic_vector'(tl(13) & tl(9) & tl(5) & tl(1))), s_tcls);

            -- ---- s2 ----
            p2_iA <= opal(p1_hA, sw_opal);
            p2_ma <= p1_ma + t_M;
            p2_sw <= signed(resize(p1_t1, 11)) + signed(resize(p1_t2, 11)) - 511;
            if p1_dxs < 0 then v_ab := -p1_dxs; else v_ab := p1_dxs; end if;
            if v_ab(12) = '0' and v_ab(11) = '1' then p2_dx <= (others => '1'); else p2_dx <= unsigned(v_ab(10 downto 0)); end if;
            p2_dlc <= clamp5(p1_dl);
            p2_drc <= clamp5(p1_dr);
            v_inx := not p1_dl(12) and not p1_dr(12);
            -- zone, with torn seams near the A/B and B/C cuts
            v_inA := cu_zA;
            v_inC := cu_zC;
            if sw_tear = '1' and cu_nA = '1' then
                v_s6 := resize(cu_sA, 7) + resize(p1_tA, 7);
                if v_s6 < 0 then v_inA := '1'; else v_inA := '0'; end if;
            end if;
            if sw_tear = '1' and cu_nC = '1' then
                v_s6 := resize(cu_sC, 7) + resize(p1_tC, 7);
                if v_s6 >= 0 then v_inC := '1'; else v_inC := '0'; end if;
            end if;
            if v_inA = '1' then
                v_z := Z_A;
            elsif v_inC = '1' then
                v_z := Z_C;
            elsif cu_win = '1' and v_inx = '1' then
                if cu_rr = '1' or p1_dl(12) = '1' or p1_dl(11 downto 1) = 0 or p1_dr(12) = '1' or p1_dr(11 downto 1) = 0 then v_z := Z_R; else v_z := Z_W; end if;
            else
                v_z := Z_G;
            end if;
            zn2 <= v_z;
            p2_md <= signed(resize(p1_rb, 9)) - signed(resize(p1_g, 9));
            p2_g  <= p1_g;
            p2_sp <= p1_sp;
            p2_tn <= p1_tn;
            p2_rf <= p1_rf;

            -- ---- s3 ----
            p3_c1  <= c_q1;
            p3_c2  <= c_q2;
            p3_e   <= (signed(resize(cu_w4(13 downto 4), 12)) - signed(resize(p2_dx, 12))) & signed(cu_w4(3 downto 0));
            -- sway and static mix as |value| x factor (sign carried)
            -- (a negative term is stored one's-complemented; s4 adds it with
            -- the sign as carry-in: one adder instead of add, subtract, mux)
            if p2_sw < 0 then
                p3_swn <= '1';  v_sw := -p2_sw;
            else
                p3_swn <= '0';  v_sw := p2_sw;
            end if;
            p3_swp <= (unsigned(v_sw(9 downto 4)) * cu_swA(6 downto 1)) xor (11 downto 0 => p2_sw(10));
            if p2_md < 0 then
                p3_mpn <= '1';  v_md := -p2_md;
            else
                p3_mpn <= '0';  v_md := p2_md;
            end if;
            p3_mp <= (unsigned(v_md(7 downto 0)) * f_as4) xor (11 downto 0 => p2_md(8));
            p3_scA <= screen(p2_ma(15 downto 8));
            p3_g   <= p2_g;
            if zn2 = Z_A then zA3 <= '1'; else zA3 <= '0'; end if;

            -- ---- s4 ----
            -- eye coverage ramp: 0.5 + e / 2^k px
            -- (the edge distance is clamped to +-127 px first: narrower barrel)
            -- (12-bit shift; coverage = clamp(e' + 8, 0, 16) taken straight
            -- from the shifted bits: in range iff e' is in -8..8, and then
            -- e' + 8 only flips bits 3 and 4.  Checked exhaustively.)
            if p3_e(15) = '0' and p3_e(14 downto 11) /= "0000" then v_e12 := to_signed(2047, 12);
            elsif p3_e(15) = '1' and p3_e(14 downto 11) /= "1111" then v_e12 := to_signed(-2048, 12);
            else v_e12 := p3_e(11 downto 0); end if;
            case to_integer(cu_k) is
                when 0      => v_e12 := v_e12;
                when 1      => v_e12 := shift_right(v_e12, 1);
                when 2      => v_e12 := shift_right(v_e12, 2);
                when 3      => v_e12 := shift_right(v_e12, 3);
                when 4      => v_e12 := shift_right(v_e12, 4);
                when 5      => v_e12 := shift_right(v_e12, 5);
                when others => v_e12 := shift_right(v_e12, 6);
            end case;
            if v_e12(11) = '0' and (v_e12(10 downto 4) /= "0000000" or (v_e12(3) = '1' and v_e12(2 downto 0) /= "000")) then
                p4_a5 <= to_unsigned(16, 5);
            elsif v_e12(11) = '1' and v_e12(10 downto 3) /= "11111111" then
                p4_a5 <= (others => '0');
            else
                p4_a5 <= unsigned(std_logic_vector'((v_e12(4) xor v_e12(3)) & (not v_e12(3)) & std_logic_vector(v_e12(2 downto 0))));
            end if;
            v_13  := (cu_phC(15 downto 4) & '1') + (p3_swp & p3_swn);
            p4_hC <= v_13(12 downto 1) & cu_phC(3 downto 0);
            v_a11 := ("00" & p3_g & '1') + (p3_mpn & p3_mpn & p3_mp(11 downto 4) & p3_mpn);
            v_ms  := signed(v_a11(10 downto 1));
            if v_ms(9) = '1' then p4_m <= (others => '0');
            elsif v_ms(8) = '1' then p4_m <= (others => '1');
            else p4_m <= unsigned(v_ms(7 downto 0)); end if;
            v_cs := to_signed(96, 10) + resize(shift_right(resize(p3_c1, 10) + resize(p3_c2, 10), 1), 10)
                    + resize(shift_right(resize(p3_c1, 10) + resize(p3_c2, 10), 2), 10)
                    + signed(resize(f_boost, 10));
            if v_cs(9) = '1' then p4_D <= (others => '0');
            elsif v_cs(8) = '1' then p4_D <= (others => '1');
            else p4_D <= unsigned(v_cs(7 downto 3)); end if;
            p4_scA <= p3_scA;
            zA4 <= zA3;

            -- ---- s5 ----
            if cu_eye = '0' then
                p5_a <= (others => '0');
            else
                p5_a <= p4_a5;
            end if;
            p5_iC <= opal(p4_hC(15 downto 8), sw_opal);
            p5_mc <= p4_hC + shift_right(p4_hC, 4);
            p5_L  <= p4_m * p4_D;
            p5_scA <= p4_scA;
            zA5 <= zA4;

            -- ---- s6 ----
            -- (outside the run the field reads 16383 + v2, which always clamps)
            v_r2 := resize(h_q(28 downto 14), 15) + resize(cu_v2, 15);
            if (h_run = '0' and h_sat = '0') or v_r2(14) = '1' then p6_r2 <= (others => '1'); else p6_r2 <= v_r2(13 downto 0); end if;
            p6_mc <= p5_mc + t_M;
            v_10  := resize(p5_L(12 downto 5), 10);
            p6_Ys <= C_BLK + shift_left(v_10, 1) + shift_right(v_10, 1);
            p6_a  <= p5_a;
            p6_scA <= p5_scA;
            zA6 <= zA5;

            -- ---- s7 ----
            if p6_r2(13 downto 11) /= "000" or (p6_r2(10) = '1' and p6_r2(9 downto 0) /= 0) then
                p7_gd <= (p6_r2(13 downto 10) - 1) & p6_r2(9 downto 0);
            else
                p7_gd <= (others => '0');
            end if;
            p7_rp <= resize(p6_r2(13 downto 2) * f_kR, 20);
            p7_hl <= cu_lb + shift_left(resize(p6_r2, 16), 4) + shift_left(resize(p6_r2, 16), 3);
            p7_hi <= (not t_Rg) - shift_left(resize(p6_r2, 16), 8) - shift_left(resize(p6_r2, 16), 7);
            p7_pd <= signed(resize(p6_r2, 15)) - signed(resize(f_pup2, 15));
            if zA6 = '1' then
                p7_sc <= p6_scA;
            else
                p7_sc <= screen(p6_mc(15 downto 8));
            end if;
            -- inset bevel: shadow on the inner top/left walls ...
            v_m := min5(p6_dlc, cu_dt);
            if    v_m(4 downto 2) = "000" and v_m(1 downto 0) /= "11" then p7_sl <= "00";
            elsif v_m(4 downto 3) = "00" and v_m(2 downto 0) /= "111" then p7_sl <= "01";
            elsif v_m(4) = '0' and v_m(3 downto 1) /= "111" then p7_sl <= "10";
            else                p7_sl <= "11";
            end if;
            -- ... light on the bottom/right ones
            v_m := min5(p6_drc, cu_db);
            case to_integer(v_m) is
                when 0      => p7_lift <= to_unsigned(72, 7);
                when 1      => p7_lift <= to_unsigned(60, 7);
                when 2      => p7_lift <= to_unsigned(48, 7);
                when 3      => p7_lift <= to_unsigned(36, 7);
                when 4      => p7_lift <= to_unsigned(24, 7);
                when 5      => p7_lift <= to_unsigned(12, 7);
                when others => p7_lift <= (others => '0');
            end case;
            p7_Ys <= p6_Ys;
            p7_a  <= p6_a;

            -- ---- s8 ----
            -- corona falloff numerator gd * (4 + sm); the 2^se is in s9
            v_gp := shift_left(resize(p7_gd, 17), 2);
            if f_sm(1) = '1' then v_gp := v_gp + shift_left(resize(p7_gd, 17), 1); end if;
            if f_sm(0) = '1' then v_gp := v_gp + resize(p7_gd, 17); end if;
            p8_gq <= v_gp;
            -- Newton's rings expanding outward
            v_t8 := tri8(p7_rp(7 downto 0) - t_Rg(15 downto 8));
            -- (v_t8 - 200) * 4 <= 220 never reaches the 255 clamp
            if v_t8(7 downto 6) = "11" and (v_t8(5 downto 4) /= "00" or (v_t8(3) = '1' and v_t8(2 downto 0) /= "000")) then
                v_t8 := v_t8 - 200;
                p8_ring <= v_t8(5 downto 0) & "00";
            else
                p8_ring <= (others => '0');
            end if;
            if p7_a(4 downto 3) /= "00" then
                if sw_iris = '1' then p8_hE <= p7_hi(15 downto 8); else p8_hE <= p7_hl(15 downto 8); end if;
            else
                -- corona hue runs 0.6 cycle per unit of rho^2 outward
                p8_hE <= t_G(15 downto 8) + resize(shift_right(p7_gd, 3), 8) + resize(shift_right(p7_gd, 5), 8);
            end if;
            v_10 := p7_Ys - C_BLK;
            case p7_sl is
                when "00"   => p8_Ysh <= shift_right(v_10, 2);
                when "01"   => p8_Ysh <= shift_right(v_10, 1);
                when "10"   => p8_Ysh <= shift_right(v_10, 1) + shift_right(v_10, 2);
                when others => p8_Ysh <= v_10;
            end case;
            if sw_iris = '1' and p7_a /= 0 and (p7_pd(14) = '1' or (p7_pd(13 downto 3) = 0 and p7_pd(2 downto 1) /= "11")) then
                if p7_pd(14) = '1' then p8_pa <= "10"; else p8_pa <= "01"; end if;
            else
                p8_pa <= "00";
            end if;
            p8_lift <= p7_lift;

            -- ---- s9 ----
            -- t6 = 63 - (gd * sA) >> 10, with sA = (4+sm)/4 * 2^se
            case to_integer(f_se) is
                when 0      => v_gp := shift_right(p8_gq, 12);
                when 1      => v_gp := shift_right(p8_gq, 11);
                when 2      => v_gp := shift_right(p8_gq, 10);
                when 3      => v_gp := shift_right(p8_gq, 9);
                when 4      => v_gp := shift_right(p8_gq, 8);
                when 5      => v_gp := shift_right(p8_gq, 7);
                when 6      => v_gp := shift_right(p8_gq, 6);
                when others => v_gp := shift_right(p8_gq, 5);
            end case;
            if v_gp(16 downto 6) /= 0 or v_gp(5 downto 0) = "111111" then p9_t6 <= (others => '0'); else p9_t6 <= not v_gp(5 downto 0); end if;
            p9_ring <= p8_ring;
            p9_iE <= opal(p8_hE, sw_opal);
            -- (palette A/C addresses arrive at s10 from the delay lines; their
            -- EBR outputs feed s12 directly)
            v_11 := resize(p8_Ysh, 11) + C_BLK + resize(p8_lift, 11);
            if v_11(10) = '1' then p9_Yf <= (others => '1'); else p9_Yf <= v_11(9 downto 0); end if;


            -- ---- s10 ----
            -- (t6 * t6) >> 6 as a table
            case to_integer(p9_t6) is
                when  0 => p10_G2 <= to_unsigned(0, 6);
                when  1 => p10_G2 <= to_unsigned(0, 6);
                when  2 => p10_G2 <= to_unsigned(0, 6);
                when  3 => p10_G2 <= to_unsigned(0, 6);
                when  4 => p10_G2 <= to_unsigned(0, 6);
                when  5 => p10_G2 <= to_unsigned(0, 6);
                when  6 => p10_G2 <= to_unsigned(0, 6);
                when  7 => p10_G2 <= to_unsigned(0, 6);
                when  8 => p10_G2 <= to_unsigned(1, 6);
                when  9 => p10_G2 <= to_unsigned(1, 6);
                when 10 => p10_G2 <= to_unsigned(1, 6);
                when 11 => p10_G2 <= to_unsigned(1, 6);
                when 12 => p10_G2 <= to_unsigned(2, 6);
                when 13 => p10_G2 <= to_unsigned(2, 6);
                when 14 => p10_G2 <= to_unsigned(3, 6);
                when 15 => p10_G2 <= to_unsigned(3, 6);
                when 16 => p10_G2 <= to_unsigned(4, 6);
                when 17 => p10_G2 <= to_unsigned(4, 6);
                when 18 => p10_G2 <= to_unsigned(5, 6);
                when 19 => p10_G2 <= to_unsigned(5, 6);
                when 20 => p10_G2 <= to_unsigned(6, 6);
                when 21 => p10_G2 <= to_unsigned(6, 6);
                when 22 => p10_G2 <= to_unsigned(7, 6);
                when 23 => p10_G2 <= to_unsigned(8, 6);
                when 24 => p10_G2 <= to_unsigned(9, 6);
                when 25 => p10_G2 <= to_unsigned(9, 6);
                when 26 => p10_G2 <= to_unsigned(10, 6);
                when 27 => p10_G2 <= to_unsigned(11, 6);
                when 28 => p10_G2 <= to_unsigned(12, 6);
                when 29 => p10_G2 <= to_unsigned(13, 6);
                when 30 => p10_G2 <= to_unsigned(14, 6);
                when 31 => p10_G2 <= to_unsigned(15, 6);
                when 32 => p10_G2 <= to_unsigned(16, 6);
                when 33 => p10_G2 <= to_unsigned(17, 6);
                when 34 => p10_G2 <= to_unsigned(18, 6);
                when 35 => p10_G2 <= to_unsigned(19, 6);
                when 36 => p10_G2 <= to_unsigned(20, 6);
                when 37 => p10_G2 <= to_unsigned(21, 6);
                when 38 => p10_G2 <= to_unsigned(22, 6);
                when 39 => p10_G2 <= to_unsigned(23, 6);
                when 40 => p10_G2 <= to_unsigned(25, 6);
                when 41 => p10_G2 <= to_unsigned(26, 6);
                when 42 => p10_G2 <= to_unsigned(27, 6);
                when 43 => p10_G2 <= to_unsigned(28, 6);
                when 44 => p10_G2 <= to_unsigned(30, 6);
                when 45 => p10_G2 <= to_unsigned(31, 6);
                when 46 => p10_G2 <= to_unsigned(33, 6);
                when 47 => p10_G2 <= to_unsigned(34, 6);
                when 48 => p10_G2 <= to_unsigned(36, 6);
                when 49 => p10_G2 <= to_unsigned(37, 6);
                when 50 => p10_G2 <= to_unsigned(39, 6);
                when 51 => p10_G2 <= to_unsigned(40, 6);
                when 52 => p10_G2 <= to_unsigned(42, 6);
                when 53 => p10_G2 <= to_unsigned(43, 6);
                when 54 => p10_G2 <= to_unsigned(45, 6);
                when 55 => p10_G2 <= to_unsigned(47, 6);
                when 56 => p10_G2 <= to_unsigned(49, 6);
                when 57 => p10_G2 <= to_unsigned(50, 6);
                when 58 => p10_G2 <= to_unsigned(52, 6);
                when 59 => p10_G2 <= to_unsigned(54, 6);
                when 60 => p10_G2 <= to_unsigned(56, 6);
                when 61 => p10_G2 <= to_unsigned(58, 6);
                when 62 => p10_G2 <= to_unsigned(60, 6);
                when others => p10_G2 <= to_unsigned(62, 6);
            end case;
            case p9_t6(5 downto 4) is
                when "11"   => p10_rt <= resize(p9_ring, 8);
                when "10"   => p10_rt <= resize(shift_right(p9_ring, 1), 8);
                when "01"   => p10_rt <= resize(shift_right(p9_ring, 2), 8);
                when others => p10_rt <= resize(shift_right(p9_ring, 4), 8);
            end case;

            p10_Yf <= p9_Yf;
            rC_ad  <= p9_iC;
            p10_iE <= p9_iE;

            -- ---- s11 ----
            p11_Gv <= resize(shift_right(p10_G2 * f_g3, 5), 5);
            p11_rl <= shift_right(p10_rt, to_integer(f_rsh));
            rE_ad  <= p10_iE;                      -- eye/corona colour out of the EBR at s12
            p11_Yf <= p10_Yf;
            p11_sp <= p10_sp;  p11_tn <= p10_tn;  p11_rf <= p10_rf;
            zn11 <= zn10;

            -- ---- s12: base colour, blend targets, raw eye colour ----
            v_y := C_BLK; v_u := C_MID; v_v := C_MID;
            p12_ty <= C_BLK; p12_tu <= C_MID; p12_tv <= C_MID;
            p12_a1 <= (others => '0');
            case zn11 is
                when Z_A =>
                    v_y := rA_qy; v_u := rA_qu; v_v := rA_qv;
                    if sw_moire = '1' then p12_a1 <= 15 - p11_sc; end if;
                when Z_C | Z_G =>
                    v_y := rC_qy; v_u := rC_qu; v_v := rC_qv;
                    if zn11 = Z_C and p11_rf = '1' then
                        p12_ty <= to_unsigned(820, 10);
                        p12_tu <= to_unsigned(560, 10);
                        p12_tv <= to_unsigned(470, 10);
                        p12_a1 <= cu_aref(3 downto 0);
                    elsif zn11 = Z_C and cu_ah /= 0 then
                        p12_ty <= to_unsigned(640, 10);
                        p12_a1 <= cu_ah(3 downto 0);
                    elsif sw_moire = '1' then
                        p12_a1 <= 15 - p11_sc;
                    end if;
                when Z_R =>
                    v_y := to_unsigned(720, 10);
                when others =>
                    if p11_sp = '1' then
                        v_y := rA_qy; v_u := rA_qu; v_v := rA_qv;
                    else
                        v_11 := resize(p11_Yf, 11) + resize(p11_rl, 11);
                        if (v_11(10) = '1' or (v_11(9 downto 7) = "111" and (v_11(6 downto 3) /= "0000" or (v_11(2) = '1' and v_11(1 downto 0) /= "00")))) then v_y := to_unsigned(900, 10); else v_y := v_11(9 downto 0); end if;
                        v_u := C_MID + resize(p11_tn & "00", 10) + resize(p11_tn, 10);
                        v_v := C_MID - resize(p11_tn & "00", 10);
                    end if;
            end case;
            p12_by <= v_y;  p12_bu <= v_u;  p12_bv <= v_v;

            if p11_a > resize(p11_Gv, 5) then p12_aE <= p11_a; else p12_aE <= resize(p11_Gv, 5); end if;
            -- rim: peaks at half coverage, 0 inside and out
            -- (a * (16 - a)) >> 2 (5-bit wrap) as a table; it has period 16 in a
            case to_integer(p11_a(3 downto 0)) is
                when  0 => p12_rim <= to_unsigned(0, 5);
                when  1 => p12_rim <= to_unsigned(3, 5);
                when  2 => p12_rim <= to_unsigned(7, 5);
                when  3 => p12_rim <= to_unsigned(9, 5);
                when  4 => p12_rim <= to_unsigned(12, 5);
                when  5 => p12_rim <= to_unsigned(13, 5);
                when  6 => p12_rim <= to_unsigned(15, 5);
                when  7 => p12_rim <= to_unsigned(15, 5);
                when  8 => p12_rim <= to_unsigned(16, 5);
                when  9 => p12_rim <= to_unsigned(15, 5);
                when 10 => p12_rim <= to_unsigned(15, 5);
                when 11 => p12_rim <= to_unsigned(13, 5);
                when 12 => p12_rim <= to_unsigned(12, 5);
                when 13 => p12_rim <= to_unsigned(9, 5);
                when 14 => p12_rim <= to_unsigned(7, 5);
                when others => p12_rim <= to_unsigned(3, 5);
            end case;

            -- ---- s13: the stronger target wins (the iris pupil darkens the eye) ----
            if p12_aE > resize(p12_a1, 5) then
                case p12_pa is
                    when "00" =>
                        p13_ty <= rE_qy; p13_tu <= rE_qu; p13_tv <= rE_qv;
                    when "01" =>
                        p13_ty <= C_BLK + shift_right(rE_qy - C_BLK, 2);
                        p13_tu <= shift_right(rE_qu, 2) + 384;
                        p13_tv <= shift_right(rE_qv, 2) + 384;
                    when others =>
                        p13_ty <= C_BLK; p13_tu <= C_MID; p13_tv <= C_MID;
                end case;
                p13_a <= p12_aE;
            else
                p13_ty <= p12_ty; p13_tu <= p12_tu; p13_tv <= p12_tv;
                p13_a <= resize(p12_a1, 5);
            end if;
            p13_by <= p12_by;  p13_bu <= p12_bu;  p13_bv <= p12_bv;
            p13_rim <= p12_rim;

            -- ---- s14: |target - base| ----
            v_df := signed(resize(p13_ty, 11)) - signed(resize(p13_by, 11));
            if v_df < 0 then p14_ny <= '1'; p14_my <= unsigned(resize(-v_df, 10));
            else             p14_ny <= '0'; p14_my <= unsigned(v_df(9 downto 0)); end if;
            v_df := signed(resize(p13_tu, 11)) - signed(resize(p13_bu, 11));
            if v_df < 0 then p14_nu <= '1'; v_df := -v_df; else p14_nu <= '0'; end if;
            p14_mu <= unsigned(v_df(9 downto 2));
            v_df := signed(resize(p13_tv, 11)) - signed(resize(p13_bv, 11));
            if v_df < 0 then p14_nv <= '1'; v_df := -v_df; else p14_nv <= '0'; end if;
            p14_mv <= unsigned(v_df(9 downto 2));
            p14_by <= p13_by;  p14_bu <= p13_bu;  p14_bv <= p13_bv;
            p14_a <= p13_a;  p14_rim <= p13_rim;

            -- ---- s15: x alpha (full coverage takes the whole difference) ----
            -- (stored one's-complemented when s16 subtracts: s16 is then one
            -- adder with the sign as carry-in)
            if p14_a(4) = '1' then
                v_py := p14_my & "0000";
                v_pu := p14_mu & "0000";
                v_pv := p14_mv & "0000";
            else
                v_py := p14_my * p14_a(3 downto 0);
                v_pu := p14_mu * p14_a(3 downto 0);
                v_pv := p14_mv * p14_a(3 downto 0);
            end if;
            p15_py <= v_py xor (13 downto 0 => p14_ny);
            p15_pu <= v_pu xor (11 downto 0 => p14_nu);
            p15_pv <= v_pv xor (11 downto 0 => p14_nv);
            p15_ny <= p14_ny;  p15_nu <= p14_nu;  p15_nv <= p14_nv;
            p15_by <= p14_by;  p15_bu <= p14_bu;  p15_bv <= p14_bv;
            p15_rim <= p14_rim;

            -- ---- s16 ----
            v_a11 := (p15_by & '1') + (p15_py(13 downto 4) & p15_ny);
            p16_y <= v_a11(10 downto 1);
            v_a11 := (p15_bu & '1') + (p15_pu(11 downto 2) & p15_nu);
            p16_u <= v_a11(10 downto 1);
            v_a11 := (p15_bv & '1') + (p15_pv(11 downto 2) & p15_nv);
            p16_v <= v_a11(10 downto 1);
            p16_rim <= p15_rim;

            -- ---- s17: white rim, blanking gate ----
            if av16 = '0' then
                p17_y <= C_BLK;  p17_u <= C_MID;  p17_v <= C_MID;
            else
                v_11 := resize(p16_y, 11) + resize(p16_rim & "0000", 11) + resize(p16_rim & "000", 11)
                        + resize(p16_rim & "00", 11);
                if (v_11(10) = '1' or (v_11(9 downto 7) = "111" and (v_11(6 downto 3) /= "0000" or (v_11(2) = '1' and v_11(1 downto 0) /= "00")))) then p17_y <= to_unsigned(900, 10); else p17_y <= v_11(9 downto 0); end if;
                if p16_rim(4 downto 3) /= "00" then
                    p17_u <= shift_right(p16_u, 1) + 256;
                    p17_v <= shift_right(p16_v, 1) + 256;
                else
                    p17_u <= p16_u;
                    p17_v <= p16_v;
                end if;
            end if;
        end if;
    end process p_pix;

    -- ====================================================================
    -- Output buffer: the composed picture (stage 17) and the stage-0 syncs
    -- go into 256-deep circular RAMs written at the same pointer.  Syncs are
    -- read back a fixed D0+17 later; video D0 later on normal rows, so the
    -- two line up.  Tear rows read video with their own delay (and chroma a
    -- little further), displacing the whole composed line.  The EBR output
    -- registers feed the output stage directly, so every read is one clock
    -- older than the delay (D + 1); reads never touch the address being
    -- written (delays 2..244 -> 3..245).
    -- ====================================================================
    p_oby : process(clk) begin if rising_edge(clk) then ob_y(to_integer(o_wp)) <= p17_y; ob_qy <= ob_y(to_integer(o_ray)); end if; end process;
    p_obu : process(clk) begin if rising_edge(clk) then ob_u(to_integer(o_wp)) <= p17_u; ob_qu <= ob_u(to_integer(o_rac)); end if; end process;
    p_obv : process(clk) begin if rising_edge(clk) then ob_v(to_integer(o_wp)) <= p17_v; ob_qv <= ob_v(to_integer(o_rac)); end if; end process;
    p_obs : process(clk)
    begin
        if rising_edge(clk) then
            ob_s(to_integer(o_wp)) <= std_logic_vector'(data_in.hsync_n & data_in.vsync_n & data_in.field_n & g_avid);
            ob_qs <= ob_s(to_integer(o_ras));
        end if;
    end process;

    p_tsc : process(clk)
        variable v_m : signed(14 downto 0);
        variable v_d : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            ts_s <= signed(resize(cu_d, 10)) - to_signed(C_D0, 10);
            ts_o <= signed(resize(cu_dc, 10)) - signed(resize(cu_d, 10));
            case s_xs is
                when "011" =>                       -- 720p: x21/32, rounded
                    v_m := shift_left(resize(ts_s, 15), 4) + shift_left(resize(ts_s, 15), 2)
                           + resize(ts_s, 15) + 16;
                    ts_sc <= resize(shift_right(v_m, 5), 10);
                when "100" =>                       -- SD: x1/2
                    ts_sc <= shift_right(ts_s, 1);
                when others =>
                    ts_sc <= ts_s;
            end case;
            ts_o2 <= ts_o;
            v_d := ts_sc + to_signed(C_D0, 10);
            cu_ds  <= unsigned(v_d(7 downto 0));
            cu_dcs <= unsigned(v_d(7 downto 0)) + unsigned(ts_o2(7 downto 0));
        end if;
    end process p_tsc;

    p_out : process(clk)
    begin
        if rising_edge(clk) then
            o_wp  <= o_wp + 1;
            -- (one clock older than the line delay: the EBR output register
            -- feeds the output stage directly)
            o_ray <= o_wp + not o_dY;
            o_rac <= o_wp + not o_dC;
            o_ras <= o_wp - to_unsigned(C_D0 + C_NW + 1, 8);

            -- per-line read delay, latched while the line is mid-pipe
            o_lq <= g_avid;
            if o_lq = '0' and g_avid = '1' then
                o_lcnt <= to_unsigned(30, 5);
            elsif o_lcnt /= 0 then
                o_lcnt <= o_lcnt - 1;
                if o_lcnt = 1 then
                    o_dY <= cu_ds;
                    o_dC <= cu_dcs;
                end if;
            end if;


            if ob_qs(0) = '1' then
                s_io.y <= std_logic_vector(ob_qy);
                s_io.u <= std_logic_vector(ob_qu);
                s_io.v <= std_logic_vector(ob_qv);
            else
                s_io.y <= std_logic_vector(C_BLK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= ob_qs(3);
            s_io.vsync_n <= ob_qs(2);
            s_io.field_n <= ob_qs(1);
            s_io.avid    <= ob_qs(0);
        end if;
    end process p_out;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture oracle;
