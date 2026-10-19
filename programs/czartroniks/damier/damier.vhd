-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- damier.vhd  (v0.4.8) -- op-art checkerboard engine
--
-- v0.4.8: resolution scaling.  Sizes are set on a virtual 1080-line canvas:
-- the engine scales the lattice step per pixel by the pixel's width in
-- virtual units (x1.5 at 720p, x2 at SD), and Y runs in the same
-- pixel-width units (Q.8, from the video timing ID), so squares, rings and
-- waves keep their size against the screen height, folds stay isotropic on
-- SD's 4:3 screen, and 1080 is bit-identical.
--
-- DAMIER (French: checkerboard; Vasarely's word for his grids).  A clean
-- checkerboard, re-drawn per square from a small vocabulary of shapes,
-- folded by up to three mirrors, bent by travelling waves, and filled with
-- concentric rings that tighten into moire.  Synthesis: no input video is
-- read.  Loosely descended from GILT's Kiss robe.
--
-- PICTURE PIPELINE
--
--  1. FOLDED SCREEN COORDINATES.  X = x - W/2, Y = y - H/2 (frame lines,
--     in pixel-width units: x 1.111 at 480 lines, 0.9375 at 576).
--     Mirror folds X -> |X|, Flip folds Y -> |Y|, Kaleido folds the
--     diagonal (P,Q) = (max, min).  Everything later works on (P,Q), so the
--     whole picture obeys the symmetry.  With any mirror on, the lattice
--     also spins (rate = Speed): a kaleidoscope, reflections counter-rotating.
--
--  2. LATTICE ACCUMULATORS.  U = a.P - b.Q, V = b.P + a.Q with
--     (a,b) = s.(cos,sin)(Angle + spin).  Moving one pixel changes exactly
--     ONE of P,Q by +-1, so each accumulator adds one of {a,-a,b,-b} per
--     pixel (registered one pixel ahead).  The line seeds (U, V at the left
--     edge) come from a LINE SEEDER: one serial shift-add multiplier forms
--     them exactly during the line before (~80 clocks).
--
--  3. WAVES (every 4th pixel).  Two waves run along the lattice axes (A
--     along U, B along V) from 128-entry interpolated tables, evaluated once
--     per 4 pixels and interpolated; Bend displaces across the travel,
--     Squeeze along it.  The coordinate waits in a block-RAM ring meanwhile.
--
--  4. SQUARES (K1).  The cell index (i,j) and in-cell (fu,fv) pick a layout
--     recipe: per cell a shape type (square / diamond / circle / triangle /
--     Truchet arc / bar / corner disc / sub-checker), solid or concentric, a
--     size, an orientation, a colour zone.  Colour = palette ROM indexed by
--     {scheme, zone, cell parity, ink}.
--
--  5. RESONANCE (P12).  Every cell fills with concentric rings of its
--     layout's own shape metric m (square rings on the plain checker,
--     circles, diamonds, Truchet arcs ...), XORed over the cell: ring bit =
--     bit 7 of (m F - Q).  The ring period is set in pixels (so the look is
--     the same at any Scale) and tightens from none at 0 to ~17 px at 100%
--     (1080 pixels; the period scales with the picture at 720p / SD).
--     The rings stand still: they change only with the slider.
--
--  6. ENGINE.  A small microcoded processor (cubist's design at 16 bits:
--     block-RAM program and register file, one ALU, Booth multiplier, sine
--     ROM) runs the per-field sequencer (glides, time, zoom, rotation, wave
--     phases, drift, the ring frequency and phase) and rebuilds the
--     double-buffered wave tables (damier_ucode.py, assembled by
--     damier_uasm.py, emulated by damier_uemu.py).
--
-- CONTROLS
--   K1 Layout   16 arrangements (Classic .. Shimmer)
--   K2 Scale    square size, 8 .. 256 px at 1080 (same share of the
--               screen height at 720p / SD), exponential
--   K3 Wave     amplitude; a second wave crosses past 50%
--   K4 Angle    lattice rotation (the waves ride the lattice axes)
--   K5 Speed    everything that moves: waves, drift, spin, pulses
--   K6 Palette  16 schemes
--   S7 Mirror  S8 Flip  S9 Kaleido   (fold switches; any fold = spin)
--   S10 Zigzag  wave shape sine -> zigzag (chevrons)
--   S11 Squeeze wave displacement across the travel -> along it
--   P12 Resonance  0 untouched; rings appear at the cell corners and
--               multiply inward (each cell a target in its shape),
--               tightening to ~17 px apart at 100%
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed width:
-- the encoders pair Cb/Cr from hsync); one latency for all modes; outputs
-- gated to neutral outside the generated avid; generated colours stored
-- U/V-swapped (damier.py); per-field work on the first vsync EDGE after
-- active video; line seeds advance on active lines only; bottom field
-- (field_n = '0') starts one frame line down.
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
use work.damier_pkg.all;

architecture damier of program_top is
    -- data_in -> data_out latency is 40 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 40;

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- shape types
    constant T_NONE : unsigned(3 downto 0) := x"0";
    constant T_SQ   : unsigned(3 downto 0) := x"1";
    constant T_DIA  : unsigned(3 downto 0) := x"2";
    constant T_CIRC : unsigned(3 downto 0) := x"3";
    constant T_TRI  : unsigned(3 downto 0) := x"5";
    constant T_TRU  : unsigned(3 downto 0) := x"6";
    constant T_BAR  : unsigned(3 downto 0) := x"7";
    constant T_SUB  : unsigned(3 downto 0) := x"8";
    constant T_CORN : unsigned(3 downto 0) := x"9";


    -- 8-bit quarter sine, 64 entries (half-step centred), for the layout's
    -- wave sample w8
    type t_q8 is array(0 to 63) of unsigned(6 downto 0);
    constant C_Q8 : t_q8 := (
        to_unsigned(  2, 7), to_unsigned(  5, 7), to_unsigned(  8, 7), to_unsigned( 11, 7),
        to_unsigned( 14, 7), to_unsigned( 17, 7), to_unsigned( 20, 7), to_unsigned( 23, 7),
        to_unsigned( 27, 7), to_unsigned( 30, 7), to_unsigned( 33, 7), to_unsigned( 36, 7),
        to_unsigned( 39, 7), to_unsigned( 42, 7), to_unsigned( 45, 7), to_unsigned( 48, 7),
        to_unsigned( 51, 7), to_unsigned( 54, 7), to_unsigned( 56, 7), to_unsigned( 59, 7),
        to_unsigned( 62, 7), to_unsigned( 65, 7), to_unsigned( 67, 7), to_unsigned( 70, 7),
        to_unsigned( 73, 7), to_unsigned( 75, 7), to_unsigned( 78, 7), to_unsigned( 80, 7),
        to_unsigned( 82, 7), to_unsigned( 85, 7), to_unsigned( 87, 7), to_unsigned( 89, 7),
        to_unsigned( 91, 7), to_unsigned( 94, 7), to_unsigned( 96, 7), to_unsigned( 98, 7),
        to_unsigned(100, 7), to_unsigned(102, 7), to_unsigned(103, 7), to_unsigned(105, 7),
        to_unsigned(107, 7), to_unsigned(108, 7), to_unsigned(110, 7), to_unsigned(112, 7),
        to_unsigned(113, 7), to_unsigned(114, 7), to_unsigned(116, 7), to_unsigned(117, 7),
        to_unsigned(118, 7), to_unsigned(119, 7), to_unsigned(120, 7), to_unsigned(121, 7),
        to_unsigned(122, 7), to_unsigned(123, 7), to_unsigned(123, 7), to_unsigned(124, 7),
        to_unsigned(125, 7), to_unsigned(125, 7), to_unsigned(126, 7), to_unsigned(126, 7),
        to_unsigned(126, 7), to_unsigned(127, 7), to_unsigned(127, 7), to_unsigned(127, 7));

    function f_sin8(k : unsigned(7 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_m : signed(7 downto 0);
    begin
        if k(6) = '0' then v_i := k(5 downto 0); else v_i := not k(5 downto 0); end if;
        v_m := signed('0' & C_Q8(to_integer(v_i)));
        if k(7) = '1' then return -v_m; else return v_m; end if;
    end function;

    -- zigzag, cosine-aligned: +max at k=0, -max at k=128 (scale 2/count)
    function f_tri8(k : unsigned(7 downto 0)) return signed is
        variable v_d : unsigned(7 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if k(7) = '1' then v_d := k - 128; else v_d := 128 - k; end if;   -- |k-128| (0..128)
        v_t := shift_left(signed(resize(v_d, 10)) - 64, 1);                -- -128..128
        if v_t > 127 then v_t := to_signed(127, 10); end if;
        if v_t < -127 then v_t := to_signed(-127, 10); end if;
        return v_t(7 downto 0);
    end function;

    -- saturate a signed value to n bits
    function f_sat(v : signed; n : integer) return signed is
        variable v_r : signed(n - 1 downto 0);
    begin
        if v > to_signed(2 ** (n - 1) - 1, v'length) then
            v_r := to_signed(2 ** (n - 1) - 1, n);
        elsif v < to_signed(-(2 ** (n - 1)) + 1, v'length) then
            v_r := to_signed(-(2 ** (n - 1)) + 1, n);
        else
            v_r := resize(v, n);
        end if;
        return v_r;
    end function;

    -- clamp a signed value into [lo, hi] (0..255), returned unsigned 8
    function f_clamp8(v : signed; lo : integer; hi : integer) return unsigned is
    begin
        if v < to_signed(lo, v'length) then
            return to_unsigned(lo, 8);
        elsif v > to_signed(hi, v'length) then
            return to_unsigned(hi, 8);
        else
            return unsigned(v(7 downto 0));
        end if;
    end function;

    function f_rot1(b : unsigned(7 downto 0)) return unsigned is
    begin
        return b(6 downto 0) & b(7);
    end function;

    function f_rot3(b : unsigned(7 downto 0)) return unsigned is
    begin
        return b(4 downto 0) & b(7 downto 5);
    end function;

    ------------------------------------------------------------------------
    -- raster timing
    ------------------------------------------------------------------------
    signal s_timing : t_video_timing_port;
    signal s_hcnt, s_meas_w : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt, s_meas_h : unsigned(10 downto 0) := (others => '0');
    signal s_ilace  : std_logic := '0';
    signal s_saw    : std_logic := '0';
    signal s_vs_go  : std_logic := '0';

    -- generated avid (aurora / cubist recipe)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_go, g_go_a : std_logic := '0';   -- g_go_a: the pixel accumulators' copy

    ------------------------------------------------------------------------
    -- controls
    ------------------------------------------------------------------------
    signal f_layout : unsigned(3 downto 0) := (others => '0');
    signal f_scheme : unsigned(3 downto 0) := (others => '0');
    -- K1/K6 label decodes, free-running.  Labelled defaults: at program load
    -- the firmware writes the initial_value_label's INDEX (0..15) into the
    -- register, not a 0..1023 position, so a value below 16 is the label
    -- index itself.
    signal s_k1_idx : unsigned(3 downto 0) := (others => '0');
    signal s_k6_idx : unsigned(3 downto 0) := (others => '0');
    signal f_mir, f_flip, f_diag, f_zz, f_sq : std_logic := '0';
    signal f_fold   : std_logic := '0';
    signal f_scrl   : std_logic := '1';

    -- per-mode
    signal m_W2, m_H2 : signed(11 downto 0) := (others => '0');
    -- Resolution scaling (v0.4.8): the picture is drawn on a virtual
    -- 1080-line canvas.  X stays in pixels and Y is kept in the same
    -- pixel-width units (so the folds stay isotropic); the engine scales
    -- the lattice step per pixel by the pixel's width in virtual units.
    --   f_cls  0: 1080 (x1)  1: 720p (x1.5)  2: SD (x2, a 1440-wide 4:3 window)
    --   f_r9   Y units per frame line, Q.9 (512 = 1 at 1080 and 720p)
    --   f_ys   Y step per field line, Q.8 (doubled when interlaced)
    --   f_y0   the field's top Y, Q.4 (from the engine: -H2 f_r9)
    signal f_cls : unsigned(1 downto 0) := (others => '0');
    signal f_r9  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_ys  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_y0  : signed(15 downto 0) := (others => '0');

    -- generic per-layout size datapath: s = clamp(k0 + (w >> gw) [- rr terms], lo, hi)
    signal g_k0   : signed(9 downto 0) := to_signed(128, 10);
    signal g_gw   : unsigned(1 downto 0) := "11";
    signal g_lo, g_hi : unsigned(7 downto 0) := (others => '0');
    signal g_vega : std_logic := '0';

    ------------------------------------------------------------------------
    -- microcoded engine (cubist's, 16-bit): program in block RAM, register
    -- file in block RAM (two copies = two read ports), one adder ALU, a
    -- serial shifter, a radix-4 Booth multiplier, a quarter-sine unit
    ------------------------------------------------------------------------
    type t_rf is array (0 to 127) of std_logic_vector(15 downto 0);
    signal rf_a, rf_b : t_rf := (others => (others => '0'));
    signal u_pc, u_fa, u_jr : unsigned(8 downto 0) := (others => '0');
    signal u_st  : unsigned(1 downto 0) := (others => '0');
    signal u_gwt : unsigned(1 downto 0) := (others => '0');
    signal u_ir, u_rom : std_logic_vector(31 downto 0) := (others => '0');
    signal u_wlast, u_we : std_logic := '0';
    signal u_wa  : unsigned(6 downto 0) := (others => '0');
    signal u_rda, u_rdb, u_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal u_sh  : signed(15 downto 0) := (others => '0');
    signal u_shc : unsigned(3 downto 0) := (others => '0');
    signal u_cin, u_rwr, u_blt, u_bnz : std_logic := '0';
    signal u_rsel : unsigned(2 downto 0) := (others => '0');
    signal u_done : std_logic := '1';
    signal u_romr : signed(15 downto 0) := (others => '0');
    signal u_ctlv, u_ctlr : signed(15 downto 0) := (others => '0');
    signal u_prt : std_logic := '0';
    signal u_prp : unsigned(4 downto 0) := (others => '0');
    signal u_prv, u_pra : std_logic_vector(15 downto 0) := (others => '0');
    signal u_tsel : std_logic := '0';

    signal mu_pa  : signed(15 downto 0) := (others => '0');
    signal mu_ph  : signed(17 downto 0) := (others => '0');
    signal mu_pl  : unsigned(15 downto 0) := (others => '0');
    signal mu_bp  : std_logic := '0';
    signal mu_cnt : unsigned(2 downto 0) := (others => '0');
    signal mu_bsy, mu_d1, mu_d2 : std_logic := '0';
    -- MRD: the product shifted in two registered steps (by 8s, then 0..7)
    signal u_msh : unsigned(4 downto 0) := (others => '0');
    signal u_mra : signed(22 downto 0) := (others => '0');
    signal u_mrd : signed(15 downto 0) := (others => '0');
    signal u_mwt : std_logic := '0';

    signal sn_neg, sn_spec : std_logic := '0';
    signal eaddr : unsigned(8 downto 0) := (others => '0');
    signal erom_q, erom_q2 : std_logic_vector(15 downto 0) := (others => '0');

    -- engine outputs (ports) consumed by the seeder and the pixel path
    signal f_a, f_b, f_na, f_nb : signed(15 downto 0) := (others => '0');
    signal f_cu, f_cv : unsigned(15 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of g_go, g_go_a : signal is true;
    -- Resonance (P12): ring frequency (cycles per half cell x 16) and phase
    signal f_rf : unsigned(10 downto 0) := (others => '0');
    signal f_rq : unsigned(7 downto 0) := (others => '0');
    signal f_t8, f_t2, f_nt2, f_t3, f_tq : unsigned(7 downto 0) := (others => '0');
    signal f_cwa, f_cwb : unsigned(15 downto 0) := (others => '0');
    signal f_tw   : unsigned(10 downto 0) := (others => '0');
    signal s_fseq_done : std_logic := '0';
    signal s_bank : std_logic := '0';
    signal bld_done : std_logic := '0';

    -- table write port (shared address / data, one enable per table)
    signal e_wa_we, e_wb_we : std_logic := '0';
    signal e_waddr : unsigned(7 downto 0) := (others => '0');
    signal e_wdata : std_logic_vector(15 downto 0) := (others => '0');

    -- tables: wave A, wave B (2 banks x 128 x 16)
    type t_ram16 is array(0 to 255) of std_logic_vector(15 downto 0);
    signal r_wa, r_wb : t_ram16 := (others => (others => '0'));

    ------------------------------------------------------------------------
    -- line seeder: the left-edge seeds of the next line, by a serial
    -- multiplier
    ------------------------------------------------------------------------
    -- Y and the folded target are Q.8 (pixel-width units); the multiplier
    -- takes them at Q.4 (16-bit operand), the pixel path's diagonal-fold
    -- threshold is Q0 rounded
    signal wl_U, wl_V : signed(26 downto 0) := (others => '0');
    signal wl_tP, wl_tQ : signed(15 downto 0) := (others => '0');
    signal wl_Y, wl_Q0n : signed(19 downto 0) := (others => '0');
    signal wl_Q0 : signed(11 downto 0) := (others => '0');
    signal wl_pl, wl_ql : signed(19 downto 0) := (others => '0');
    signal wl_tgo : unsigned(1 downto 0) := (others => '0');
    signal ls_run : std_logic := '0';
    signal ls_p   : unsigned(2 downto 0) := (others => '0');
    signal ls_k   : unsigned(4 downto 0) := (others => '0');
    signal ls_M   : signed(15 downto 0) := (others => '0');
    signal ls_H   : signed(16 downto 0) := (others => '0');
    signal ls_L   : std_logic_vector(15 downto 0) := (others => '0');
    signal ls_S   : signed(31 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel accumulators (stage 0)
    ------------------------------------------------------------------------
    signal acc_U, acc_V : signed(26 downto 0) := (others => '0');
    signal px_X, px_Q0 : signed(11 downto 0) := (others => '0');
    signal px_A : signed(12 downto 0) := (others => '0');   -- folded X, one pixel ahead
    signal px_N : std_logic := '0';
    signal f_al : signed(11 downto 0) := (others => '0');  -- folded left edge
    signal op_U, op_V : signed(15 downto 0) := (others => '0');
    signal bk_cnt : unsigned(1 downto 0) := (others => '0');   -- pixel x mod 4 at stage 0

    ------------------------------------------------------------------------
    -- stage 1: the coordinate for the main path; the block sample
    ------------------------------------------------------------------------
    signal m1_U, m1_V : unsigned(15 downto 0) := (others => '0');
    signal s_wa, s_wb : unsigned(15 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- block engine: wave lookups (B1..B6), result queue
    ------------------------------------------------------------------------
    signal b1_wa, b1_wb : unsigned(7 downto 0) := (others => '0');
    signal b2_waq, b2_wbq : std_logic_vector(15 downto 0) := (others => '0');
    signal b3_waq, b3_wbq : std_logic_vector(15 downto 0) := (others => '0');
    signal b4_a, b4_b : signed(9 downto 0) := (others => '0');
    signal b4_ia, b4_ib : signed(9 downto 0) := (others => '0');
    signal b5_wa, b5_wb : signed(10 downto 0) := (others => '0');
    signal b6_wa, b6_wb : signed(10 downto 0) := (others => '0');
    -- per-block extras (wave displacements), two deep
    signal xq0_wa, xq0_wb, xq1_wa, xq1_wb : signed(10 downto 0) := (others => '0');
    signal qc_U, qc_V, qp_U, qp_V : unsigned(15 downto 0) := (others => '0');

    -- main path: the coordinate rides a block-RAM ring from stage 1 to 19
    type t_r32 is array(0 to 255) of std_logic_vector(31 downto 0);
    signal r_mr : t_r32 := (others => (others => '0'));
    signal mr_wp, mr_rp : unsigned(7 downto 0) := (others => '0');
    signal mr_q, mr_q2 : std_logic_vector(31 downto 0) := (others => '0');

    signal m20_U, m20_V : unsigned(15 downto 0) := (others => '0');
    signal m20_DU, m20_DV : unsigned(17 downto 0) := (others => '0');   -- 2 fraction bits
    signal m20_lU, m20_lV : unsigned(17 downto 0) := (others => '0');
    signal p9_wk  : unsigned(10 downto 0) := (others => '0');          -- stage 21
    signal p9_Uf, p9_Vf : unsigned(15 downto 0) := (others => '0');     -- stage 21
    -- layout stages
    signal p10_i, p10_j, p10_fu, p10_fv : unsigned(7 downto 0) := (others => '0');
    signal p10_c  : std_logic := '0';
    signal p10_w8 : signed(7 downto 0) := (others => '0');
    signal p10_ai, p10_aj, p10_ring : unsigned(6 downto 0) := (others => '0');
    signal p10_ha, p10_hb, p10_h1 : unsigned(7 downto 0) := (others => '0');

    signal p11_ring : unsigned(6 downto 0) := (others => '0');
    signal p11_dmd, p11_dg : unsigned(7 downto 0) := (others => '0');
    signal p11_h3 : unsigned(7 downto 0) := (others => '0');
    signal p11_gpre : signed(10 downto 0) := (others => '0');
    signal p11_rterm : unsigned(10 downto 0) := (others => '0');
    signal p11_tv : unsigned(7 downto 0) := (others => '0');
    signal p11_i, p11_j, p11_fu, p11_fv : unsigned(7 downto 0) := (others => '0');
    signal p11_c, p11_wneg : std_logic := '0';
    signal p11_ai2, p11_aj2 : unsigned(1 downto 0) := (others => '0');

    signal p12_hs : unsigned(7 downto 0) := (others => '0');
    signal p12_sgen, p12_salb, p12_snalb, p12_sbul : unsigned(7 downto 0) := (others => '0');
    signal p12_lvl : unsigned(1 downto 0) := (others => '0');
    signal p12_rk, p12_zpin : unsigned(1 downto 0) := (others => '0');
    signal p12_ring : unsigned(6 downto 0) := (others => '0');
    signal p12_dmd, p12_dg : unsigned(7 downto 0) := (others => '0');
    signal p12_i, p12_j, p12_fu, p12_fv : unsigned(7 downto 0) := (others => '0');
    signal p12_c, p12_wneg : std_logic := '0';

    signal p13_typ : unsigned(3 downto 0) := (others => '0');
    signal p13_sol : std_logic := '1';
    signal p13_s   : unsigned(7 downto 0) := (others => '0');
    signal p13_q   : unsigned(1 downto 0) := (others => '0');   -- 0:bit5 1:bit6 2:bit7
    signal p13_ori : unsigned(1 downto 0) := (others => '0');
    signal p13_inv : std_logic := '0';
    signal p13_zone : unsigned(1 downto 0) := (others => '0');
    signal p13_lvl : unsigned(1 downto 0) := (others => '0');
    signal p13_c   : std_logic := '0';
    signal p13_fu, p13_fv : unsigned(7 downto 0) := (others => '0');

    -- shape stages: control bundle carried along
    type t_ctl is record
        typ  : unsigned(3 downto 0);
        sol  : std_logic;
        s    : unsigned(7 downto 0);
        q    : unsigned(1 downto 0);
        inv  : std_logic;
        zone : unsigned(1 downto 0);
        c    : std_logic;
        sub  : std_logic;
    end record;
    constant C_CTL0 : t_ctl := (x"0", '1', x"00", "00", '0', "00", '0', '0');
    signal k14, k15, k16, k17, k18, k19, k20, kr1, kr2 : t_ctl := C_CTL0;

    signal p14_lvl : unsigned(1 downto 0) := (others => '0');
    signal p14_sw  : std_logic := '0';
    signal p14_fu, p14_fv : unsigned(7 downto 0) := (others => '0');
    signal p14_a0, p14_b0 : unsigned(6 downto 0) := (others => '0');
    signal p15_au, p15_av, p15_mx, p15_mn : unsigned(6 downto 0) := (others => '0');
    signal p15_ts : unsigned(8 downto 0) := (others => '0');
    signal p15_fu, p15_fv : unsigned(7 downto 0) := (others => '0');
    signal p16_mx, p16_mn : unsigned(6 downto 0) := (others => '0');
    signal p16_tx, p16_ty : unsigned(7 downto 0) := (others => '0');
    signal p16_mo : unsigned(8 downto 0) := (others => '0');
    signal p17_ox, p17_oy : unsigned(7 downto 0) := (others => '0');
    signal p17_mo : unsigned(8 downto 0) := (others => '0');
    signal p18_ox : unsigned(7 downto 0) := (others => '0');
    signal p18_rb : unsigned(8 downto 0) := (others => '0');
    signal p18_mo : unsigned(8 downto 0) := (others => '0');
    signal p19_r  : unsigned(8 downto 0) := (others => '0');
    signal p19_mo : unsigned(8 downto 0) := (others => '0');
    signal p20_m  : unsigned(8 downto 0) := (others => '0');
    -- Resonance stages: m x frequency (split operand), then the ring bit
    signal r1_m, r2_m : unsigned(8 downto 0) := (others => '0');
    signal r1_pl : unsigned(14 downto 0) := (others => '0');
    signal r1_ph : unsigned(13 downto 0) := (others => '0');
    signal r2_ring : std_logic := '0';
    signal p21_pa : unsigned(7 downto 0) := (others => '0');
    signal p22_pq : std_logic_vector(31 downto 0) := (others => '0');
    signal p23_y, p23_u, p23_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (the HD HDMI bar fix, HW-confirmed by A/B on
    -- v0.4.4): [1 2 1] horizontal smoothing, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p24_y, p24_u, p24_v : unsigned(9 downto 0) := (others => '0');

    -- side ring (EBR): the four syncs from stage 1 to 39
    type t_ring is array(0 to 255) of std_logic_vector(7 downto 0);
    signal r_sy : t_ring := (others => (others => '0'));
    signal sy_wp, sy_rp : unsigned(7 downto 0) := (others => '0');
    signal sy_q, sy_q2 : std_logic_vector(7 downto 0) := (others => '0');
    signal d_s1 : std_logic_vector(3 downto 0) := "0110";

    -- output
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_s : std_logic_vector(3 downto 0) := "0110";   -- avid hs vs fl

    -- No RAM here ever reads an address on the clock it is written (tables:
    -- other bank; rings: other slot; register file: forwarded), so yosys
    -- need not emulate read-before-write (a delayed write port + bypass mux
    -- per RAM, ~90 LC each)
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf_a, rf_b, r_wa, r_wb, r_mr, r_sy : signal is true;

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement + once-per-field anchor
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_w <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_h <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                -- interlaced = the timing ID says so (480i, 576i, 1080i);
                -- the sync-phase detector is not trusted for this (v0.4.8)
                case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                    when 0 | 8 | 1 | 2 | 10 => s_ilace <= '1';
                    when others             => s_ilace <= '0';
                end case;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

            -- first vsync edge after active video only (analog vsync serrates)
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Clean avid: starts 3 or 4 clocks after the source's first avid rise,
    -- whichever keeps the hsync->start parity at its voted usual value; ends
    -- after the field-latched width.  g_go is the cycle BEFORE it rises,
    -- when the pixel accumulators load their line seeds.
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
            -- registered: g_sel never changes within two clocks of g_rsr(0)
            if (g_sel = '0' and g_rsr(0) = '1') or (g_sel = '1' and g_rsr(1) = '1') then
                g_go   <= '1';
                g_go_a <= '1';
            else
                g_go   <= '0';
                g_go_a <= '0';
            end if;
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

            if g_go = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Controls: switches and selectors latched once per field (the engine
    -- reads the geometry knobs straight off the registers and glides them
    -- itself); per-layout constants and time derivatives free-running
    -- ====================================================================
    p_knobs : process(clk)
        variable v_h : unsigned(11 downto 0);
        variable v_r9 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if unsigned(registers_in(0)) < 16 then
                s_k1_idx <= unsigned(registers_in(0)(3 downto 0));
            else
                s_k1_idx <= unsigned(registers_in(0)(9 downto 6));
            end if;
            if unsigned(registers_in(5)) < 16 then
                s_k6_idx <= unsigned(registers_in(5)(3 downto 0));
            else
                s_k6_idx <= unsigned(registers_in(5)(9 downto 6));
            end if;
            if s_vs_go = '1' then
                f_layout <= s_k1_idx;
                f_scheme <= s_k6_idx;
                f_mir  <= registers_in(6)(0);
                f_flip <= registers_in(6)(1);
                f_diag <= registers_in(6)(2);
                f_zz   <= registers_in(6)(3);
                f_sq   <= registers_in(6)(4);
                g_wm1  <= s_meas_w - 1;

                v_h := resize(s_meas_h, 12);
                if s_ilace = '1' then v_h := v_h(10 downto 0) & '0'; end if;
                m_H2 <= signed('0' & v_h(11 downto 1));
                m_W2 <= signed('0' & s_meas_w(11 downto 1));

                -- resolution class and Y units from the video timing ID
                case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                    when 0      => f_cls <= "10"; v_r9 := to_unsigned(569, 10);  -- 480i: 1080/486/2
                    when 4      => f_cls <= "10"; v_r9 := to_unsigned(576, 10);  -- 480p: 1080/480/2
                    when 8 | 12 => f_cls <= "10"; v_r9 := to_unsigned(480, 10);  -- 576: 1080/576/2
                    when 5 | 6 | 14 => f_cls <= "01"; v_r9 := to_unsigned(512, 10);  -- 720p
                    when others => f_cls <= "00"; v_r9 := to_unsigned(512, 10);  -- 1080p/i
                end case;
                f_r9 <= v_r9;
                if s_ilace = '1' then f_ys <= v_r9;
                else                  f_ys <= '0' & v_r9(9 downto 1); end if;
            end if;

            f_fold <= f_mir or f_flip or f_diag;
            if f_mir = '1' then f_al <= m_W2; else f_al <= -m_W2; end if;

            -- per-layout: drift, generic size constants
            g_k0 <= to_signed(128, 10); g_gw <= "11"; g_lo <= x"00"; g_hi <= x"FF";
            g_vega <= '0'; f_scrl <= '1';
            case to_integer(f_layout) is
                when 1 =>       -- Vega: 220 - 12 ring + w/2 + pulse
                    g_k0 <= to_signed(188, 10); g_gw <= "01";
                    g_lo <= to_unsigned(30, 8); g_hi <= to_unsigned(250, 8);
                    g_vega <= '1'; f_scrl <= '0';
                when 2 | 8 | 9 | 12 =>
                    f_scrl <= '0';
                when 3 =>       -- Harlequin
                    g_k0 <= to_signed(200, 10); g_gw <= "01";
                    g_lo <= to_unsigned(60, 8); g_hi <= to_unsigned(255, 8);
                when 6 =>       -- Truchet
                    g_k0 <= to_signed(70, 10); g_gw <= "10";
                when 10 =>      -- Quartered
                    g_k0 <= to_signed(150, 10); g_gw <= "01";
                    g_lo <= to_unsigned(40, 8); g_hi <= to_unsigned(250, 8);
                when 15 =>      -- Shimmer
                    g_k0 <= to_signed(128, 10); g_gw <= "00";
                    g_lo <= to_unsigned(4, 8); g_hi <= to_unsigned(254, 8);
                when others => null;
            end case;

            f_t2  <= f_t8(6 downto 0) & '0';
            f_nt2 <= 0 - (f_t8(6 downto 0) & '0');
            f_t3  <= (f_t8(6 downto 0) & '0') + f_t8;
            f_tq  <= "0000" & f_t8(7 downto 4);
        end if;
    end process p_knobs;

    -- ====================================================================
    -- Microcoded engine (cubist's design, 16-bit).  Restarts at every field
    -- anchor; the program (damier_ucode.py) runs the frame sequencer,
    -- releases the line engine (FDONE), rebuilds the back-bank tables and
    -- halts (END).  Three clocks per straight-line instruction; MRD (until
    -- the product and its two-step shift are ready), SHR / SHL, SIN and ROM
    -- stall in EXEC.
    -- ====================================================================
    u_fa <= u_pc + 1 when u_st = "11" else u_pc;

    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_fa));
        end if;
    end process p_urom;

    p_rfa : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_a(to_integer(u_wa)) <= u_wd; end if;
            u_rda <= rf_a(to_integer(unsigned(u_rom(19 downto 13))));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_b(to_integer(u_wa)) <= u_wd; end if;
            u_rdb <= rf_b(to_integer(unsigned(u_rom(12 downto 6))));
        end if;
    end process p_rfb;

    p_erom : process(clk)
    begin
        if rising_edge(clk) then
            erom_q  <= C_SIN(to_integer(eaddr));
            erom_q2 <= erom_q;
        end if;
    end process p_erom;

    with to_integer(unsigned(u_ir(3 downto 0))) select u_ctlv <=
        signed(resize(unsigned(registers_in(1)), 16)) when 0,
        signed(resize(unsigned(registers_in(2)), 16)) when 1,
        signed(resize(unsigned(registers_in(3)), 16)) when 2,
        signed(resize(unsigned(registers_in(7)), 16)) when 3,
        signed(resize(unsigned(registers_in(4)), 16)) when 4,
        signed(resize(unsigned'(f_cls & f_scrl & f_sq & f_zz & f_fold), 16)) when 5,
        resize(m_W2, 16)         when 6,
        resize(m_H2, 16)         when 7,
        signed(resize(f_r9, 16)) when 8,
        (others => '0')          when others;

    p_ueng : process(clk)
        variable v_a, v_b, v_r : signed(15 downto 0);
        variable v_op, v_op2 : integer range 0 to 31;
        variable v_stl, v_jmp : boolean;
        variable v_ms, v_md : signed(17 downto 0);
        variable v_pr, v_pr2 : signed(31 downto 0);
        variable v_ma : signed(22 downto 0);
        variable v_fa, v_fb : std_logic_vector(15 downto 0);
        variable v_ang : unsigned(15 downto 0);
        variable v_pm : unsigned(14 downto 0);
        variable v_sv : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            u_wlast <= u_we;
            -- serial multiply, radix-4 Booth, 8 steps: {mu_ph, mu_pl} =
            -- mu_pa * B for a two's-complement B
            if mu_bsy = '1' then
                -- one adder: the digit's magnitude (0, A, 2A), conditionally
                -- inverted, plus a carry-in for the negative digits
                case std_logic_vector'(mu_pl(1) & mu_pl(0) & mu_bp) is
                    when "001" | "010" | "101" | "110" => v_md := resize(mu_pa, 18);
                    when "011" | "100"                 => v_md := shift_left(resize(mu_pa, 18), 1);
                    when others                        => v_md := (others => '0');
                end case;
                if mu_pl(1) = '1' then
                    v_ms := mu_ph + (not v_md) + 1;
                else
                    v_ms := mu_ph + v_md;
                end if;
                mu_ph <= shift_right(v_ms, 2);
                mu_pl <= unsigned(v_ms(1 downto 0)) & mu_pl(15 downto 2);
                mu_bp <= mu_pl(1);
                if mu_cnt = 0 then mu_bsy <= '0';
                else mu_cnt <= mu_cnt - 1; end if;
            end if;
            v_pr := signed(std_logic_vector'(std_logic_vector(mu_ph(15 downto 0)) & std_logic_vector(mu_pl)));
            mu_d1 <= mu_bsy;
            mu_d2 <= mu_d1;
            -- MRD shifter, free-running on the current product and shift
            v_pr2 := shift_right(v_pr, 8 * to_integer(u_msh(4 downto 3)));
            u_mra <= v_pr2(22 downto 0);
            v_ma  := shift_right(u_mra, to_integer(u_msh(2 downto 0)));
            u_mrd <= v_ma(15 downto 0);

            u_we  <= '0';
            u_prt <= '0';

            if s_vs_go = '1' then
                u_pc   <= (others => '0');
                u_st   <= "00";
                u_gwt  <= (others => '0');
                u_done <= '0';
            elsif u_done = '0' then
                case u_st is

                    -- FETCH (after a jump only): the ROM answers next cycle
                    when "00" =>
                        u_st <= "01";

                    -- DECODE: latch the instruction; the register-file reads
                    -- are addressed straight off the ROM output this cycle
                    when "01" =>
                        u_ir  <= u_rom;
                        u_msh <= unsigned(u_rom(4 downto 0));
                        u_st <= "10";

                    -- OPERAND LATCH: block-RAM outputs into fabric; every unit
                    -- launches here from the register-file outputs
                    when "10" =>
                        v_op2 := to_integer(unsigned(u_ir(31 downto 27)));
                        -- forwarding: a just-written source comes from the
                        -- write-data register (EBR read-during-write is undefined)
                        if u_wlast = '1' and unsigned(u_ir(19 downto 13)) = u_wa then
                            v_fa := u_wd;
                        else
                            v_fa := u_rda;
                        end if;
                        if u_wlast = '1' and unsigned(u_ir(12 downto 6)) = u_wa then
                            v_fb := u_wd;
                        else
                            v_fb := u_rdb;
                        end if;
                        u_sh <= signed(v_fa);
                        -- SUB = A + not B + 1 on the one adder
                        if v_op2 = 3 then
                            u_wd <= not v_fb;  u_cin <= '1';
                        else
                            u_wd <= v_fb;      u_cin <= '0';
                        end if;
                        u_ctlr <= u_ctlv;
                        case v_op2 is
                            when 1 | 4 | 5 => u_rsel <= "000"; u_rwr <= '1';
                            when 2 | 3     => u_rsel <= "001"; u_rwr <= '1';
                            when 27        => u_rsel <= "010"; u_rwr <= '1';
                            when 15        => u_rsel <= "011"; u_rwr <= '1';
                            when 11        => u_rsel <= "100"; u_rwr <= '1';
                            when 14        => u_rsel <= "101"; u_rwr <= '1';
                            when 25        => u_rsel <= "110"; u_rwr <= '1';
                            when others    => u_rsel <= "000"; u_rwr <= '0';
                        end case;
                        u_shc <= unsigned(u_ir(3 downto 0));
                        u_mwt <= '1';
                        u_wa  <= unsigned(u_ir(26 downto 20));
                        if signed(v_fa) < signed(v_fb) then u_blt <= '1'; else u_blt <= '0'; end if;
                        if v_fa /= x"0000" then u_bnz <= '1'; else u_bnz <= '0'; end if;
                        u_jr <= unsigned(v_fa(8 downto 0));
                        case v_op2 is
                            when 10 =>                     -- MUL
                                mu_pa  <= signed(v_fa);
                                mu_pl  <= unsigned(v_fb);
                                mu_ph  <= (others => '0');
                                mu_bp  <= '0';
                                mu_cnt <= to_unsigned(7, 3);
                                mu_bsy <= '1';
                            when 14 =>                     -- SIN: quarter-wave fold
                                v_ang := unsigned(v_fa);
                                if u_ir(0) = '1' then
                                    v_ang(15 downto 14) := v_ang(15 downto 14) + 1;
                                end if;
                                if v_ang(14) = '1' then
                                    v_pm := to_unsigned(16384, 15) - resize(v_ang(13 downto 0), 15);
                                else
                                    v_pm := resize(v_ang(13 downto 0), 15);
                                end if;
                                sn_spec <= v_pm(14);
                                sn_neg  <= v_ang(15);
                                eaddr   <= v_pm(13 downto 5);
                            when 17 =>                     -- PRT operands
                                u_prp <= unsigned(u_ir(4 downto 0));
                                u_prv <= v_fa;
                                u_pra <= v_fb;
                            when others => null;
                        end case;
                        u_st <= "11";

                    -- EXECUTE
                    when others =>
                        v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                        v_a   := u_sh;
                        v_b   := signed(u_wd);
                        v_stl := false;
                        v_jmp := false;

                        case to_integer(u_rsel) is
                            when 0 => v_r := u_sh;                          -- MOV, SHR/SHL
                            when 1 => if u_cin = '1' then v_r := v_a + v_b + 1;
                                      else                v_r := v_a + v_b; end if;
                            when 2 => v_r := v_a and signed(resize(unsigned(u_ir(12 downto 0)), 16));
                            when 3 => v_r := resize(signed(u_ir(13 downto 0)), 16);
                            when 4 => v_r := u_mrd;                         -- MRD
                            when 5 => v_r := u_romr;                        -- SIN / ROM
                            when others => v_r := u_ctlr;                   -- CTL
                        end case;

                        case v_op is
                            when 4 | 5 =>                  -- SHR / SHL
                                if u_shc /= 0 then
                                    if v_op = 4 then u_sh <= shift_right(u_sh, 1);
                                    else             u_sh <= shift_left(u_sh, 1); end if;
                                    u_shc <= u_shc - 1;
                                    v_stl := true;
                                end if;
                            when 11 =>                     -- MRD: product final, shifted
                                if mu_bsy = '1' or mu_d1 = '1' or mu_d2 = '1' or u_mwt = '1' then
                                    u_mwt <= '0';
                                    v_stl := true;
                                end if;
                            when 14 =>                     -- SIN: the ROM answers in 2,
                                case to_integer(u_gwt) is  -- into fabric in 3
                                    when 0 | 1 =>
                                        u_gwt <= u_gwt + 1;
                                        v_stl := true;
                                    when 2 =>
                                        if sn_spec = '1' then
                                            v_sv := to_signed(32767, 17);
                                        else
                                            v_sv := signed('0' & erom_q2);
                                        end if;
                                        if sn_neg = '1' then u_romr <= -v_sv(15 downto 0);
                                        else                 u_romr <= v_sv(15 downto 0); end if;
                                        u_gwt <= "11";
                                        v_stl := true;
                                    when others =>
                                        u_gwt <= (others => '0');
                                end case;
                            when 17 => u_prt <= '1';
                            when 18 => v_jmp := true;
                                       u_pc <= unsigned(u_ir(8 downto 0));
                            when 19 => if u_bnz = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(8 downto 0));
                                       end if;
                            when 20 => if u_blt = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(22 downto 20)) & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 21 => if u_blt = '0' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(22 downto 20)) & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 26 => v_jmp := true;
                                       u_pc <= u_jr;
                            when 24 => u_done <= '1';
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
        end if;
    end process p_ueng;

    -- ====================================================================
    -- Engine ports: the per-field constants the seeder and the pixel path
    -- use (drift and wave travel already folded in), the table writes, and
    -- the bank swap (only after a COMPLETE build).
    -- ====================================================================
    p_ports : process(clk)
        variable v : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            e_wa_we <= '0';
            e_wb_we <= '0';
            s_fseq_done <= '0';
            v := signed(u_prv);
            if s_vs_go = '1' then
                if bld_done = '1' then
                    s_bank <= not s_bank;
                end if;
                bld_done <= '0';
            elsif u_prt = '1' then
                case to_integer(u_prp) is
                    when 0  => f_a    <= v;
                    when 1  => f_b    <= v;
                    when 2  => f_cu   <= unsigned(v);
                    when 3  => f_cv   <= unsigned(v);
                    when 4  => f_rf   <= unsigned(v(10 downto 0));
                    when 5  => f_rq   <= unsigned(v(7 downto 0));
                    when 7  => f_t8   <= unsigned(v(15 downto 8));
                    when 8  => f_cwa  <= unsigned(v);
                    when 9  => f_cwb  <= unsigned(v);
                    when 10 => f_tw   <= unsigned(v(10 downto 0));
                    when 13 => u_tsel <= u_prv(0);
                    when 14 =>                      -- wave table, back bank
                        e_waddr <= (not s_bank) & unsigned(u_pra(6 downto 0));
                        e_wdata <= u_prv;
                        if u_tsel = '0' then e_wa_we <= '1'; else e_wb_we <= '1'; end if;
                    when 16 => s_fseq_done <= '1';
                    when 17 => bld_done <= '1';
                    when 19 => f_na   <= v;
                    when 20 => f_nb   <= v;
                    when 21 => f_y0   <= v;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_ports;

    -- ====================================================================
    -- Tables: one write port (the engine, back bank), one registered read
    -- port (the block engine, display bank).  Never the same bank, so a read
    -- can never meet a write.
    -- ====================================================================
    p_rwa : process(clk)
    begin
        if rising_edge(clk) then
            if e_wa_we = '1' then r_wa(to_integer(e_waddr)) <= e_wdata; end if;
            b2_waq <= r_wa(to_integer(b1_wa));
        end if;
    end process p_rwa;

    p_rwb : process(clk)
    begin
        if rising_edge(clk) then
            if e_wb_we = '1' then r_wb(to_integer(e_waddr)) <= e_wdata; end if;
            b2_wbq <= r_wb(to_integer(b1_wb));
        end if;
    end process p_rwb;


    -- ====================================================================
    -- LINE SEEDER.  The seeds of a line's left edge are exact functions of
    -- its folded position (tP, tQ): U = a tP - b tQ, V = b tP + a tQ.  At
    -- each line start the target moves one field line down (f_ys, Q.8
    -- pixel-width units: 1 at 1080p / 720p, 2 at 1080i, 2.22 at 480i);
    -- three clocks later a serial signed multiplier (radix 2, 18 clocks a
    -- product, the target at Q.4) forms the four products and two sums for
    -- the NEXT line: ~80 clocks of the line's 858+.  After the frame
    -- sequencer (FDONE) it seeds the field's first line (f_y0) the same way.
    -- ====================================================================
    p_walk : process(clk)
        variable v_q0, v_al, v_pl, v_ql : signed(19 downto 0);
        variable v_h : signed(16 downto 0);
        variable v_s : signed(31 downto 0);
    begin
        if rising_edge(clk) then
            -- target: the left edge of line wl_Y, folded (two clocks); Q.8
            if f_flip = '1' and wl_Y < 0 then v_q0 := -wl_Y; else v_q0 := wl_Y; end if;
            v_al := shift_left(resize(f_al, 20), 8);
            if f_diag = '1' then
                if v_al >= v_q0 then v_pl := v_al; v_ql := v_q0;
                else v_pl := v_q0; v_ql := v_al; end if;
            else
                v_pl := v_al; v_ql := v_q0;
            end if;
            wl_Q0n <= v_q0;
            wl_pl  <= v_pl;
            wl_ql  <= v_ql;
            -- the pixel path's diagonal threshold: Q0 rounded (a fractional
            -- SD row crosses the diagonal mid-pixel; exact at 1080 / 720p)
            wl_Q0  <= wl_Q0n(19 downto 8) + signed'("00000000000" & wl_Q0n(7));
            wl_tP  <= wl_pl(19 downto 4);
            wl_tQ  <= wl_ql(19 downto 4);

            if s_fseq_done = '1' or g_go = '1' then
                wl_tgo <= "00";
                ls_run <= '0';
                if s_fseq_done = '1' then
                    -- bottom field (field_n = '0') starts one frame line down
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        wl_Y <= shift_left(resize(f_y0, 20), 4) + signed(resize(f_r9(9 downto 1), 20));
                    else
                        wl_Y <= shift_left(resize(f_y0, 20), 4);
                    end if;
                else
                    -- one field line down (two frame lines when interlaced)
                    wl_Y <= wl_Y + signed(resize(f_ys, 20));
                end if;
            elsif wl_tgo /= "11" then
                -- let the folded target settle after Y moves, then start
                wl_tgo <= wl_tgo + 1;
                if wl_tgo = "10" then
                    ls_run <= '1';
                    ls_p   <= (others => '0');
                    ls_k   <= (others => '0');
                end if;
            elsif ls_run = '1' then
                if ls_k = 0 then
                    -- load: (a, tP) (-b, tQ) (b, tP) (a, tQ)
                    case to_integer(ls_p) is
                        when 0 | 3  => ls_M <= f_a;
                        when 1      => ls_M <= f_nb;
                        when others => ls_M <= f_b;
                    end case;
                    if ls_p(0) = '0' then ls_L <= std_logic_vector(wl_tP);
                    else                  ls_L <= std_logic_vector(wl_tQ); end if;
                    ls_H <= (others => '0');
                    ls_k <= ls_k + 1;
                elsif ls_k /= 17 then
                    -- one multiplier bit: add M (subtract on the sign bit),
                    -- shift {H, L} right
                    v_h := ls_H;
                    if ls_L(0) = '1' then
                        if ls_k = 16 then v_h := ls_H - resize(ls_M, 17);
                        else              v_h := ls_H + resize(ls_M, 17); end if;
                    end if;
                    ls_H <= v_h(16) & v_h(16 downto 1);
                    ls_L <= v_h(0) & ls_L(15 downto 1);
                    ls_k <= ls_k + 1;
                else
                    -- product {H, L} (x16: the target is Q.4): start or
                    -- finish a sum
                    if ls_p(0) = '0' then
                        v_s := (others => '0');
                    else
                        v_s := ls_S;
                    end if;
                    v_s := v_s + signed(std_logic_vector'(std_logic_vector(ls_H(15 downto 0)) & ls_L));
                    ls_S <= v_s;
                    case to_integer(ls_p) is
                        when 1      => wl_U <= v_s(30 downto 4);
                        when 3      => wl_V <= v_s(30 downto 4);
                                       ls_run <= '0';
                        when others => null;
                    end case;
                    ls_p <= ls_p + 1;
                    ls_k <= (others => '0');
                end if;
            end if;
        end if;
    end process p_walk;

    -- ====================================================================
    -- Stage 0: fold-aware accumulators.  Moving one pixel changes exactly
    -- one folded coordinate (P or Q) by +-1; that choice and the signed
    -- operands are registered one pixel AHEAD, so each accumulate is a
    -- plain add.
    -- ====================================================================
    p_acc : process(clk)
        variable v_q0 : signed(11 downto 0);
        variable v_a  : signed(12 downto 0);
        variable v_n, v_p : std_logic;
    begin
        if rising_edge(clk) then
            -- pixel x mod 4 at stage 0 (free-running: stays valid past the line)
            if g_go_a = '1' then bk_cnt <= "00"; else bk_cnt <= bk_cnt + 1; end if;

            -- choose the step from pixel X to X+1 from the folded |X| and Q0,
            -- registered one pixel ahead (the left edge's wait through
            -- blanking); the step operands load every clock
            v_a  := px_A;
            v_n  := px_N;
            v_q0 := px_Q0;
            if f_diag = '0' then
                v_p := '1';
            elsif v_n = '0' then
                if v_a >= resize(v_q0, 13) then v_p := '1'; else v_p := '0'; end if;
            else
                if v_a > resize(v_q0, 13) then v_p := '1'; else v_p := '0'; end if;
            end if;
            if v_p = '1' then
                if v_n = '0' then op_U <= f_a;  op_V <= f_b;
                else              op_U <= f_na; op_V <= f_nb; end if;
            else
                if v_n = '0' then op_U <= f_nb; op_V <= f_a;
                else              op_U <= f_b;  op_V <= f_na; end if;
            end if;
            if g_go_a = '1' or g_avid = '1' then
                px_N <= f_mir and px_X(11);
                if f_mir = '1' and px_X(11) = '1' then px_A <= -resize(px_X, 13);
                else                                   px_A <= resize(px_X, 13); end if;
                px_X <= px_X + 1;
            else
                px_N  <= f_mir;
                px_A  <= resize(f_al, 13);
                px_Q0 <= wl_Q0;
                px_X  <= 1 - m_W2;
            end if;

            if g_go_a = '1' then
                acc_U  <= wl_U;
                acc_V  <= wl_V;
            elsif g_avid = '1' then
                acc_U <= acc_U + op_U;
                acc_V <= acc_V + op_V;
            end if;
        end if;
    end process p_acc;

    -- ====================================================================
    -- Stage 1: the absolute coordinate for the main path, and the block
    -- sample (every 4th pixel): the wave phases (along U and V, minus their
    -- travel)
    -- ====================================================================
    p_samp : process(clk)
    begin
        if rising_edge(clk) then
            m1_U <= unsigned(acc_U(23 downto 8)) + f_cu;
            m1_V <= unsigned(acc_V(23 downto 8)) + f_cv;
            if bk_cnt = 0 then
                s_wa  <= unsigned(acc_U(23 downto 8)) + f_cwa;
                s_wb  <= unsigned(acc_V(23 downto 8)) + f_cwb;
            end if;
        end if;
    end process p_samp;

    -- ====================================================================
    -- BLOCK ENGINE, part 1: wave lookups.  The sample holds for 4 clocks
    -- (bk_cnt 1, 2, 3, 0): B1 -> EBR -> B3 (bk_cnt 0) -> B4 slope x frac (the
    -- wave fraction is still this sample's at bk_cnt 0) -> B5 sums -> B6.
    -- Wave A: 8 squares per wavelength along U; wave B: 16 along V; 128
    -- entries each, 16 interpolation steps.
    -- ====================================================================
    p_blook : process(clk)
        variable v_t  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- B1
            b1_wa <= s_bank & s_wa(10 downto 4);
            b1_wb <= s_bank & s_wb(11 downto 5);

            -- B3: into fabric
            b3_waq <= b2_waq;
            b3_wbq <= b2_wbq;

            -- B4: wave values and slope x frac
            b4_a  <= signed(b3_waq(15 downto 6));
            b4_b  <= signed(b3_wbq(15 downto 6));
            v_t := (others => '0');
            if s_wa(3) = '1' then v_t := v_t + shift_left(resize(signed(b3_waq(5 downto 0)), 10), 3); end if;
            if s_wa(2) = '1' then v_t := v_t + shift_left(resize(signed(b3_waq(5 downto 0)), 10), 2); end if;
            if s_wa(1) = '1' then v_t := v_t + shift_left(resize(signed(b3_waq(5 downto 0)), 10), 1); end if;
            if s_wa(0) = '1' then v_t := v_t + resize(signed(b3_waq(5 downto 0)), 10); end if;
            b4_ia <= v_t;
            v_t := (others => '0');
            if s_wb(4) = '1' then v_t := v_t + shift_left(resize(signed(b3_wbq(5 downto 0)), 10), 3); end if;
            if s_wb(3) = '1' then v_t := v_t + shift_left(resize(signed(b3_wbq(5 downto 0)), 10), 2); end if;
            if s_wb(2) = '1' then v_t := v_t + shift_left(resize(signed(b3_wbq(5 downto 0)), 10), 1); end if;
            if s_wb(1) = '1' then v_t := v_t + resize(signed(b3_wbq(5 downto 0)), 10); end if;
            b4_ib <= v_t;

            -- B5: wave sums
            b5_wa  <= resize(b4_a, 11) + resize(shift_right(b4_ia, 4), 11);
            b5_wb  <= resize(b4_b, 11) + resize(shift_right(b4_ib, 4), 11);

            -- B6
            b6_wa  <= b5_wa;
            b6_wb  <= b5_wb;
        end if;
    end process p_blook;

    -- ====================================================================
    -- BLOCK ENGINE, part 2: the wave displacements, two deep, in the fixed
    -- 4-clock rhythm the main path's ring delay is set for (the block's
    -- lookups sit in B6 when bk_cnt = 3; the block at pixel 4k lands in qc
    -- at stage 15 of 4k; the main path reads qp = block k and qc = block
    -- k+1 at stage 19).  A runs along U, B along V; Squeeze displaces along
    -- the travel, Bend across it.
    -- ====================================================================
    p_block : process(clk)
    begin
        if rising_edge(clk) then
            if bk_cnt = 3 then
                xq0_wa <= b6_wa;  xq1_wa <= xq0_wa;
                xq0_wb <= b6_wb;  xq1_wb <= xq0_wb;
            end if;
            if bk_cnt = 2 then
                qp_U <= qc_U;
                qp_V <= qc_V;
                if f_sq = '1' then
                    qc_U <= unsigned(resize(xq1_wa, 16));
                    qc_V <= unsigned(resize(xq1_wb, 16));
                else
                    qc_U <= unsigned(resize(xq1_wb, 16));
                    qc_V <= unsigned(resize(xq1_wa, 16));
                end if;
            end if;
        end if;
    end process p_block;

    -- ====================================================================
    -- MAIN PATH ring: the lattice coordinate (32 bits) waits in block RAM
    -- from stage 1 to stage 19 (write at wp, read wp - 16: 15 + 3 = 18
    -- clocks).  Read and write never share an address.
    -- ====================================================================
    p_mring : process(clk)
    begin
        if rising_edge(clk) then
            r_mr(to_integer(mr_wp)) <= std_logic_vector(m1_U) & std_logic_vector(m1_V);
            mr_q  <= r_mr(to_integer(mr_rp));
            mr_q2 <= mr_q;
            mr_wp <= mr_wp + 1;
            mr_rp <= mr_wp - 15;
        end if;
    end process p_mring;

    -- ====================================================================
    -- Stages 20..21: the block displacement interpolated across the block,
    -- incrementally (D = 4 qp + j (qc - qp), j = x mod 4 = bk_cnt + 1 at
    -- stage 19; qp, qc hold from bk_cnt = 3 to 2), and the final coordinate;
    -- the layout's wave sample is taken per pixel from the final coordinate
    -- ====================================================================
    p_main : process(clk)
    begin
        if rising_edge(clk) then
            m20_U  <= unsigned(mr_q2(31 downto 16));
            m20_V  <= unsigned(mr_q2(15 downto 0));
            if bk_cnt = 3 then
                m20_DU <= qp_U & "00";
                m20_DV <= qp_V & "00";
                m20_lU <= unsigned(resize(signed(qc_U), 18) - resize(signed(qp_U), 18));
                m20_lV <= unsigned(resize(signed(qc_V), 18) - resize(signed(qp_V), 18));
            else
                m20_DU <= m20_DU + m20_lU;
                m20_DV <= m20_DV + m20_lV;
            end if;

            p9_Uf  <= m20_U + m20_DU(17 downto 2);
            p9_Vf  <= m20_V + m20_DV(17 downto 2);
            -- the layout's wave sample: wave A's phase at the final coordinate
            p9_wk  <= m20_U(10 downto 0) + m20_DU(12 downto 2) - f_tw;
        end if;
    end process p_main;

    -- ====================================================================
    -- Stages 22..25: cell fields, hash, layout recipe
    -- ====================================================================
    p_layout : process(clk)
        variable v_i, v_j, v_jj, v_ha, v_hb, v_h : unsigned(7 downto 0);
        variable v_w  : signed(10 downto 0);
        variable v_t  : unsigned(7 downto 0);
        variable v_s  : signed(10 downto 0);
        variable v_ai, v_aj : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- S22
            v_i := p9_Uf(15 downto 8);
            v_j := p9_Vf(15 downto 8);
            p10_i  <= v_i;
            p10_j  <= v_j;
            p10_fu <= p9_Uf(7 downto 0);
            p10_fv <= p9_Vf(7 downto 0);
            p10_c  <= v_i(0) xor v_j(0);
            if f_zz = '1' then p10_w8 <= f_tri8(p9_wk(10 downto 3));
            else               p10_w8 <= f_sin8(p9_wk(10 downto 3)); end if;
            if v_i(7) = '1' then v_ai := not v_i(6 downto 0); else v_ai := v_i(6 downto 0); end if;
            if v_j(7) = '1' then v_aj := not v_j(6 downto 0); else v_aj := v_j(6 downto 0); end if;
            p10_ai <= v_ai;
            p10_aj <= v_aj;
            if v_ai >= v_aj then p10_ring <= v_ai; else p10_ring <= v_aj; end if;
            -- Robe hashes tall blocks (j >> 2)
            if f_layout = 14 then
                v_jj := v_j(7) & v_j(7) & v_j(7 downto 2);
            else
                v_jj := v_j;
            end if;
            v_ha := v_i xor x"5A";
            v_hb := v_jj xor x"C3";
            v_h  := v_ha xor f_rot3(v_hb);
            p10_ha <= v_ha;
            p10_hb <= v_hb;
            p10_h1 <= v_h + (v_h(5 downto 0) & "00") + to_unsigned(16#9D#, 8);

            -- S23
            p11_ring <= p10_ring;
            p11_dmd <= resize(p10_ai, 8) + resize(p10_aj, 8);
            p11_dg  <= p10_i + p10_j - f_tq;
            v_h := p10_h1 xor ("0000" & p10_h1(7 downto 4));
            p11_h3  <= v_h + f_rot1(p10_hb) + p10_ha;
            -- generic size: k0 + w >> gw  (Vega also: - rr/2 - rr/8 + pulse/2)
            v_w := resize(p10_w8, 11);
            case g_gw is
                when "00"   => p11_gpre <= resize(g_k0, 11) + v_w;
                when "01"   => p11_gpre <= resize(g_k0, 11) + shift_right(v_w, 1);
                when "10"   => p11_gpre <= resize(g_k0, 11) + shift_right(v_w, 2);
                when others => p11_gpre <= resize(g_k0, 11);
            end case;
            -- Vega: dots shrink ring by ring from the centre, and a size
            -- pulse runs outward through the rings
            p11_rterm <= resize(p10_ring & "000", 11) + resize(p10_ring & "00", 11);
            v_t := f_t2 - (p10_ring(3 downto 0) & "0000");
            if v_t(7) = '1' then p11_tv <= v_t - 128; else p11_tv <= 128 - v_t; end if;
            p11_i  <= p10_i;
            p11_j  <= p10_j;
            p11_fu <= p10_fu;
            p11_fv <= p10_fv;
            p11_c  <= p10_c;
            p11_wneg <= p10_w8(7);
            p11_ai2 <= p10_ai(2 downto 1);
            p11_aj2 <= p10_aj(2 downto 1);

            -- S24
            v_h := p11_h3 + (p11_h3(4 downto 0) & "000");
            p12_hs <= v_h xor ("0000" & v_h(7 downto 4));
            if g_vega = '1' then
                v_s := p11_gpre - signed(p11_rterm) + signed(resize(p11_tv(7 downto 1), 11));
            else
                v_s := p11_gpre;
            end if;
            if v_s < signed(resize(g_lo, 11)) then
                p12_sgen <= g_lo;
            elsif v_s > signed(resize(g_hi, 11)) then
                p12_sgen <= g_hi;
            else
                p12_sgen <= unsigned(v_s(7 downto 0));
            end if;
            p12_salb  <= f_t2 + (p11_ring(3 downto 0) & "0000");
            p12_snalb <= f_nt2 - (p11_ring(3 downto 0) & "0000");
            p12_sbul  <= (p11_ring(2 downto 0) & "00000") + (p11_ring(4 downto 0) & "000") - f_t3;
            v_t := resize(p11_ring, 8) + f_tq;
            p12_lvl   <= v_t(3 downto 2);
            v_t := resize(p11_ring, 8) - f_tq;
            p12_rk    <= v_t(2 downto 1);
            p12_zpin  <= p11_ai2 + p11_aj2;
            p12_ring  <= p11_ring;
            p12_dmd   <= p11_dmd;
            p12_dg    <= p11_dg;
            p12_i     <= p11_i;
            p12_j     <= p11_j;
            p12_fu    <= p11_fu;
            p12_fv    <= p11_fv;
            p12_c     <= p11_c;
            p12_wneg  <= p11_wneg;

            -- S25: the layout recipe (selects only)
            p13_typ  <= T_NONE;
            p13_sol  <= '1';
            p13_s    <= p12_sgen;
            p13_q    <= "00";
            p13_ori  <= "00";
            p13_inv  <= '0';
            p13_zone <= "00";
            p13_lvl  <= "00";
            case to_integer(f_layout) is
                when 0 =>       -- Classic: plain checker, colour bands drift
                    p13_zone <= p12_dg(3 downto 2);
                when 1 =>       -- Vega: dots, big at the centre, pulsing outward
                    p13_typ <= T_CIRC;
                    p13_zone <= p12_ring(3 downto 2);
                when 2 =>       -- Albers: nested squares flowing, alternate cells reversed
                    p13_typ <= T_SQ; p13_sol <= '0'; p13_q <= "01";
                    if p12_c = '1' then p13_s <= p12_snalb; else p13_s <= p12_salb; end if;
                    p13_zone <= p12_ring(2 downto 1);
                when 3 =>       -- Harlequin: diamonds breathing to touch
                    p13_typ <= T_DIA;
                    p13_zone <= p12_dmd(2 downto 1);
                when 4 =>       -- Pinwheel: half-square triangles turned around each 2x2
                    p13_typ <= T_TRI; p13_s <= x"80";
                    case std_logic_vector'(p12_i(0) & p12_j(0)) is
                        when "00"   => p13_ori <= "00";
                        when "01"   => p13_ori <= "01";
                        when "10"   => p13_ori <= "11";
                        when others => p13_ori <= "10";
                    end case;
                    p13_zone <= p12_zpin;
                when 5 =>       -- Chevron: triangle rows -> zigzags
                    p13_typ <= T_TRI; p13_s <= x"80";
                    if p12_i(0) = '1' then p13_ori <= "11"; end if;
                    p13_zone <= p12_j(2 downto 1);
                when 6 =>       -- Truchet: random quarter arcs -> meanders
                    p13_typ <= T_TRU;
                    p13_ori <= '0' & p12_hs(3);
                    p13_zone <= p12_hs(6 downto 5);
                when 7 =>       -- Weave: stripes, alternate cells turned
                    p13_typ <= T_BAR; p13_sol <= '0'; p13_q <= "01"; p13_s <= x"00";
                    p13_ori <= '0' & p12_c;
                    p13_zone <= p12_i(2 downto 1) xor p12_j(2 downto 1);
                when 8 =>       -- Bullseye: ripples through the rings of cells
                    p13_typ <= T_CIRC; p13_sol <= '0'; p13_q <= "01"; p13_s <= p12_sbul;
                    p13_zone <= p12_ring(2 downto 1);
                when 9 =>       -- Fractal: the checker subdivides ring by ring
                    p13_typ <= T_SUB; p13_lvl <= p12_lvl;
                    p13_zone <= p12_lvl;
                when 10 =>      -- Quartered: discs on the corners, quartered by parity
                    p13_typ <= T_CORN;
                    p13_zone <= p12_ring(3 downto 2);
                when 11 =>      -- Mosaic: a random tile in every cell
                    case p12_hs(1 downto 0) is
                        when "00"   => p13_typ <= T_CIRC; p13_s <= x"C8";
                        when "01"   => p13_typ <= T_TRI;  p13_s <= x"80";
                        when "10"   => p13_typ <= T_TRU;  p13_s <= x"50";
                        when others => p13_typ <= T_CORN; p13_s <= x"C8";
                    end case;
                    if p12_hs(4 downto 2) = "000" then p13_sol <= '0'; end if;
                    p13_ori <= p12_hs(6 downto 5);
                    p13_zone <= p12_hs(4 downto 3);
                when 12 =>      -- Rings: a different pattern per ring band, flowing out
                    case p12_rk is
                        when "00"   => p13_typ <= T_NONE;
                        when "01"   => p13_typ <= T_CIRC; p13_s <= x"C8";
                        when "10"   => p13_typ <= T_DIA;  p13_s <= x"FE";
                        when others => p13_typ <= T_SQ;   p13_sol <= '0'; p13_s <= x"00";
                    end case;
                    p13_zone <= p12_rk;
                when 13 =>      -- Zebra: one continuous stripe field (Riley)
                    p13_typ <= T_BAR; p13_sol <= '0'; p13_q <= "10"; p13_s <= x"00";
                    p13_inv <= p12_c;
                    p13_zone <= p12_ring(3 downto 2);
                when 14 =>      -- Robe (Kiss): tall blocks and bars
                    p13_typ <= T_BAR;
                    if p12_hs(1 downto 0) = "00" then
                        p13_s <= x"FE";
                    else
                        p13_s <= to_unsigned(60, 8) + (p12_hs(7 downto 2) & '0');
                    end if;
                    p13_zone <= p12_hs(7 downto 6);
                when others =>  -- Shimmer: dots that turn into holes with the wave
                    p13_typ <= T_CIRC;
                    p13_inv <= p12_wneg;
                    p13_zone <= p12_ring(4 downto 3);
            end case;
            p13_c  <= p12_c;
            p13_fu <= p12_fu;
            p13_fv <= p12_fv;
        end if;
    end process p_layout;

    -- ====================================================================

    -- ====================================================================
    -- Stages 26..35: the shape engine (with Resonance's rings).  Near-round radius
    -- r = max(mx, 7/8 mx + 1/2 mn), within 3% of the true distance.
    -- ====================================================================
    p_shape : process(clk)
        variable v_k   : t_ctl;
        variable v_mx, v_mn : unsigned(6 downto 0);
        variable v_d   : signed(9 downto 0);
        variable v_sum : unsigned(9 downto 0);
        variable v_ink, v_bit : std_logic;
        variable v_rb  : unsigned(8 downto 0);
        variable v_rs  : unsigned(11 downto 0);
        variable v_rr  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- S26: orientation (quarter turns) and centre distances; |x-128|
            -- is invariant under x -> not x, so a turn only swaps au/av
            v_k := (p13_typ, p13_sol, p13_s, p13_q, p13_inv, p13_zone, p13_c, '0');
            k14 <= v_k;
            p14_lvl <= p13_lvl;
            p14_sw  <= p13_ori(0);
            case p13_ori is
                when "00"   => p14_fu <= p13_fu;     p14_fv <= p13_fv;
                when "01"   => p14_fu <= not p13_fv; p14_fv <= p13_fu;
                when "10"   => p14_fu <= not p13_fu; p14_fv <= not p13_fv;
                when others => p14_fu <= p13_fv;     p14_fv <= not p13_fu;
            end case;
            if p13_fu(7) = '1' then p14_a0 <= p13_fu(6 downto 0); else p14_a0 <= not p13_fu(6 downto 0); end if;
            if p13_fv(7) = '1' then p14_b0 <= p13_fv(6 downto 0); else p14_b0 <= not p13_fv(6 downto 0); end if;

            -- S27: swap, max/min, the corner sum, sub-checker bit
            v_k := k14;
            case p14_lvl is
                when "00"   => v_k.sub := p14_fu(7) xor p14_fv(7);
                when "01"   => v_k.sub := p14_fu(6) xor p14_fv(6);
                when "10"   => v_k.sub := p14_fu(5) xor p14_fv(5);
                when others => v_k.sub := p14_fu(4) xor p14_fv(4);
            end case;
            k15 <= v_k;
            if p14_sw = '1' then p15_au <= p14_b0; p15_av <= p14_a0;
            else p15_au <= p14_a0; p15_av <= p14_b0; end if;
            if p14_a0 >= p14_b0 then p15_mx <= p14_a0; p15_mn <= p14_b0;
            else p15_mx <= p14_b0; p15_mn <= p14_a0; end if;
            p15_ts <= resize(p14_fu, 9) + resize(p14_fv, 9);
            p15_fu <= p14_fu;
            p15_fv <= p14_fv;

            -- S28: Truchet corner fold; the straight-edged metric
            k16 <= k15;
            p16_mx <= p15_mx;
            p16_mn <= p15_mn;
            if p15_ts(8) = '0' then p16_tx <= p15_fu; p16_ty <= p15_fv;
            else p16_tx <= not p15_fu; p16_ty <= not p15_fv; end if;
            case k15.typ is
                when T_SQ | T_NONE | T_SUB =>        -- (the plain checker's
                               p16_mo <= '0' & p15_mx & '0';  -- rings are square)
                when T_DIA  => p16_mo <= resize(p15_au, 9) + resize(p15_av, 9);
                when T_TRI  => p16_mo <= '0' & p15_ts(8 downto 1);
                when others => p16_mo <= '0' & p15_au & '0';
            end case;

            -- S29: inputs of the round metric
            k17 <= k16;
            case k16.typ is
                when T_CORN =>
                    p17_ox <= '0' & (127 - p16_mn);
                    p17_oy <= '0' & (127 - p16_mx);
                when T_TRU =>
                    if p16_tx >= p16_ty then p17_ox <= p16_tx; p17_oy <= p16_ty;
                    else p17_ox <= p16_ty; p17_oy <= p16_tx; end if;
                when others =>
                    p17_ox <= '0' & p16_mx;
                    p17_oy <= '0' & p16_mn;
            end case;
            p17_mo <= p16_mo;

            -- S30: 7/8 ox + 1/2 oy
            k18 <= k17;
            p18_ox <= p17_ox;
            p18_rb <= resize(p17_ox, 9) - resize(p17_ox(7 downto 3), 9) + resize(p17_oy(7 downto 1), 9);
            p18_mo <= p17_mo;

            -- S31: r = max
            k19 <= k18;
            if p18_rb > resize(p18_ox, 9) then p19_r <= p18_rb; else p19_r <= resize(p18_ox, 9); end if;
            p19_mo <= p18_mo;

            -- S32: pick the metric: 2r (circles, corner discs), 2|r-128|
            -- (Truchet arcs), or the straight one
            k20 <= k19;
            case k19.typ is
                when T_CIRC | T_CORN =>
                    if p19_r(8) = '1' then p20_m <= (others => '1');
                    else p20_m <= p19_r(7 downto 0) & '0'; end if;
                when T_TRU =>
                    v_d := signed('0' & p19_r) - 128;
                    if v_d < 0 then v_d := -v_d; end if;
                    if v_d > 255 then p20_m <= (others => '1');
                    else p20_m <= unsigned(v_d(7 downto 0)) & '0'; end if;
                when others =>
                    p20_m <= p19_mo;
            end case;

            -- S33: RESONANCE (P12), the metric times the ring frequency
            -- (cycles per half cell x 16; operand split hi / lo)
            kr1   <= k20;
            r1_m  <= p20_m;
            r1_pl <= p20_m * f_rf(5 downto 0);
            r1_ph <= p20_m * f_rf(10 downto 6);

            -- S34: ring phase r = m f - Q (256 per cycle); the ring bit
            kr2  <= kr1;
            r2_m <= r1_m;
            v_rs := r1_pl(11 downto 0) + (r1_ph(5 downto 0) & "000000");
            v_rr := v_rs(11 downto 4) - f_rq;
            r2_ring <= v_rr(7);

            -- S35: solid (m < s) or concentric (bit q of m + s); the rings
            -- XORed over the cell; palette address
            v_sum := resize(r2_m, 10) + resize(kr2.s, 10);
            case kr2.q is
                when "00"   => v_bit := v_sum(5);
                when "01"   => v_bit := v_sum(6);
                when others => v_bit := v_sum(7);
            end case;
            if kr2.typ = T_NONE then
                v_ink := '0';
            elsif kr2.typ = T_SUB then
                v_ink := kr2.sub;
            elsif kr2.sol = '1' then
                if r2_m < resize(kr2.s, 9) then v_ink := '1'; else v_ink := '0'; end if;
            else
                v_ink := v_bit;
            end if;
            p21_pa <= f_scheme & kr2.zone & kr2.c & (v_ink xor kr2.inv xor r2_ring);
        end if;
    end process p_shape;

    -- ====================================================================
    -- Stages 36..39: palette, output conditioning, blanking gate
    -- ====================================================================
    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            p22_pq <= C_DPAL(to_integer(p21_pa));
        end if;
    end process p_pal;

    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S37
            p23_y <= unsigned(p22_pq(31 downto 22));
            p23_u <= unsigned(p22_pq(21 downto 12));
            p23_v <= unsigned(p22_pq(11 downto 2));

            -- S38: horizontal [1 2 1]/4 on the pixel one back (g1 is the
            -- centre, g2 and p23 its neighbours).  With the luma scale below
            -- this stops the thick bright bars HD HDMI showed on DAMIER's
            -- sharp high-contrast moving edges (A/B on one placement, v0.4.4)
            g1_y <= p23_y;  g2_y <= g1_y;
            g1_u <= p23_u;  g2_u <= g1_u;
            g1_v <= p23_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(p23_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(p23_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(p23_v, 12);
            p24_y <= v_sy(11 downto 2);
            p24_u <= v_su(11 downto 2);
            p24_v <= v_sv(11 downto 2);

            -- S39: luma scaled toward black, 64 + 13/16 (y - 64) (white
            -- ~775, under the ~768 drawn-luma cap give or take); neutral
            -- outside the generated avid
            v_d  := p24_y - 64;
            v_yo := p24_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sy_q2(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p24_u;
                s_out_v <= p24_v;
            end if;
            s_out_s <= sy_q2(3 downto 0);
        end if;
    end process p_out;

    -- ====================================================================
    -- Side ring (block RAM).  The syncs (generated avid, hsync, vsync,
    -- field) from stage 1 to 39: write at wp, read wp - 36 (35 + 3 = 38
    -- clocks).  Read and write never share an address.
    -- ====================================================================
    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            d_s1 <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            r_sy(to_integer(sy_wp)) <= "0000" & d_s1;
            sy_q  <= r_sy(to_integer(sy_rp));
            sy_q2 <= sy_q;
            sy_wp <= sy_wp + 1;
            sy_rp <= sy_wp - 35;
        end if;
    end process p_rings;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_out_s(3);
    data_out.hsync_n <= s_out_s(2);
    data_out.vsync_n <= s_out_s(1);
    data_out.field_n <= s_out_s(0);

end architecture damier;
