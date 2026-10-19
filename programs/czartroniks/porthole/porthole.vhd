-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Porthole v0.2.1: catalogue combo #6 (seed 20260912, phosphor slot redrawn)
-- -- a porthole over the picture: a horizontal lens (spherize) fitted to the
-- window, scrolling value-noise displacement, and the picture outside left
-- original or warped too.
--
--   K1 Curve    lens strength, bipolar: CCW bulges (fisheye), CW pinches;
--               fitted to the window's rows (to the screen's rows outside
--               when S7 = Warped)
--   K2 Zoom     0.25x .. 4x (log, 256 steps; CW magnifies)
--   K3 Noise    value-noise displacement amount (64x32 lattice, scrolling)
--   K4 Speed    lattice scroll speed (continuous, 0 .. 16 lines a frame)
--   K5 Size     window width 10 .. ~98% of the picture; the height follows
--               the screen's aspect
--   K6 Position window centre column
--
--   S7  Outside  original picture / the warped picture outside the window
--   S8  Noise    inside the window only / everywhere
--   S9  Frame    none / black rim (thickness follows the window size)
--   S10 Shape    rectangle / round porthole (diameter = Size; the circle
--                is aspect-corrected, so it stays round on SD too)
--   S11 Window   Lens: magnifies what is behind it / Fit: the whole picture
--                squeezed into the window
--   P12 Drift    the porthole swings in a figure-eight; the swing (up to
--                half the screen each way) and the speed grow with the
--                slider, 100% = an edge-to-edge swoop every ~2 s (glided)
--
-- Two DDAs (global map, window map) selected per pixel by the window
-- (rectangle, or circle: a running dx^2 per pixel against a per-line
-- threshold R^2 * (1 - rho^2), the same 1 - rho^2 that weights the lens,
-- with the rim an exact ring at threshold - (2Rt - t^2)).  Both are linear in x from the line start (u = uc + (x-cx)*k),
-- so a window clipped by the screen edge keeps its content in place.  The
-- per-line sequencer runs the screen lens and the window lens interleaved
-- on one shared operand-split 14x14 multiplier.  Reads that fall off the
-- source are mirrored back in once (black beyond), so noise and zoom-out
-- never leave ragged black edges.  Curve, Zoom, Size, Position and Drift
-- are glided (ease by diff/8 a frame).
--
-- v0.2.3 (2026-10-17) resolution scaling: the porthole's size follows the
-- screen HEIGHT (window width = Size x the width of a 16:9 screen this tall,
-- 4/3 W on SD, capped at W - W/64), so it is the same size and still round
-- on the 4:3 SD screen; its centre and Drift stay fractions of the screen.
-- The noise lattice (64 x 32), its scroll and its displacement run in
-- virtual 1080-line units (timing-ID steps; interlace-corrected), and the
-- rect rim's top/bottom is halved on interlaced fields.  1080p unchanged.
--
-- HARDWARE CONTRACTS: one latency (12 clocks) in all modes; outputs gated to
-- neutral outside avid; source avid passed through (the picture is the
-- source's own; the only drawn colour is the neutral-chroma black rim).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture porthole of program_top is
    -- data_in -> data_out latency is 12 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;

    constant C_LAT : integer := 12;
    constant C_K_MIN : integer := 96;
    constant C_SEED_NOISE : unsigned(15 downto 0) := x"3C6E";

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

    -- glide: g + (target - g)/8, 10.3 fixed point
    function f_glide(g : unsigned(12 downto 0); d : signed(13 downto 0)) return unsigned is
        variable v : signed(13 downto 0);
    begin
        v := signed(resize(g, 14)) + shift_right(d, 3);
        return unsigned(v(12 downto 0));
    end function;
    function f_gdiff(k : unsigned(9 downto 0); g : unsigned(12 downto 0)) return signed is
    begin
        return signed(shift_left(resize(k, 14), 3)) - signed(resize(g, 14));
    end function;

    -- clip a signed screen coordinate to 0 .. 2047
    function f_clip11(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 11);
        elsif v > 2047 then
            return to_unsigned(2047, 11);
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

    -- lens row scale from |c| * w: 1024 -/+ the product, clamped
    function f_lens(mp : unsigned(27 downto 0); neg : std_logic) return unsigned is
        variable v_kk : signed(22 downto 0);
    begin
        if neg = '1' then
            v_kk := to_signed(1024, 23) - signed(resize(mp(22 downto 9), 23));
        else
            v_kk := to_signed(1024, 23) + signed(resize(mp(22 downto 8), 23));
        end if;
        if v_kk < C_K_MIN then v_kk := to_signed(C_K_MIN, 23); end if;
        if v_kk > 4095 then v_kk := to_signed(4095, 23); end if;
        return unsigned(v_kk(12 downto 0));
    end function;

    -- Resolution scaling (v0.2.3): per-mode steps from the video timing ID.
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

    -- 1024 * 2^(i/64): one octave of zoom steps
    type t_zoom_rom is array (0 to 63) of unsigned(10 downto 0);
    constant C_ZOOM_ROM : t_zoom_rom := (
        to_unsigned(1024, 11), to_unsigned(1035, 11), to_unsigned(1046, 11), to_unsigned(1058, 11), to_unsigned(1069, 11), to_unsigned(1081, 11), to_unsigned(1093, 11), to_unsigned(1105, 11),
        to_unsigned(1117, 11), to_unsigned(1129, 11), to_unsigned(1141, 11), to_unsigned(1154, 11), to_unsigned(1166, 11), to_unsigned(1179, 11), to_unsigned(1192, 11), to_unsigned(1205, 11),
        to_unsigned(1218, 11), to_unsigned(1231, 11), to_unsigned(1244, 11), to_unsigned(1258, 11), to_unsigned(1272, 11), to_unsigned(1286, 11), to_unsigned(1300, 11), to_unsigned(1314, 11),
        to_unsigned(1328, 11), to_unsigned(1342, 11), to_unsigned(1357, 11), to_unsigned(1372, 11), to_unsigned(1387, 11), to_unsigned(1402, 11), to_unsigned(1417, 11), to_unsigned(1433, 11),
        to_unsigned(1448, 11), to_unsigned(1464, 11), to_unsigned(1480, 11), to_unsigned(1496, 11), to_unsigned(1512, 11), to_unsigned(1529, 11), to_unsigned(1545, 11), to_unsigned(1562, 11),
        to_unsigned(1579, 11), to_unsigned(1596, 11), to_unsigned(1614, 11), to_unsigned(1631, 11), to_unsigned(1649, 11), to_unsigned(1667, 11), to_unsigned(1685, 11), to_unsigned(1704, 11),
        to_unsigned(1722, 11), to_unsigned(1741, 11), to_unsigned(1760, 11), to_unsigned(1779, 11), to_unsigned(1798, 11), to_unsigned(1818, 11), to_unsigned(1838, 11), to_unsigned(1858, 11),
        to_unsigned(1878, 11), to_unsigned(1898, 11), to_unsigned(1919, 11), to_unsigned(1940, 11), to_unsigned(1961, 11), to_unsigned(1983, 11), to_unsigned(2004, 11), to_unsigned(2026, 11)
    );

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
    -- Line buffers
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    -- each bank is read only while the other one is written (a same-bank
    -- read is discarded by the bank mux), so no transparency emulation
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    --------------------------------------------------------------------------
    -- Controls (registered locally) and glides (10.3)
    --------------------------------------------------------------------------
    signal s_k_curve, s_k_zoom, s_k_noise, s_k_speed, s_k_size, s_k_pos : unsigned(9 downto 0) := (others => '0');
    signal s_p_drift : unsigned(9 downto 0) := (others => '0');
    signal s_sw_outside, s_sw_noise_all, s_sw_border, s_sw_round, s_sw_fit : std_logic := '0';
    signal s_g_curve, s_g_zoom, s_g_size, s_g_pos : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_g_drift : unsigned(12 downto 0) := (others => '0');
    signal s_d_curve, s_d_zoom, s_d_size, s_d_pos, s_d_drift : signed(13 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Tracking
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_line       : unsigned(10 downto 0) := (others => '0');
    signal s_height     : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wf         : unsigned(10 downto 0) := to_unsigned(480, 11);   -- per-frame width
    signal s_half_h     : unsigned(10 downto 0) := to_unsigned(135, 11);
    signal s_half_w     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_w15, s_w2, s_w2m1, s_nw : signed(14 downto 0) := (others => '0');  -- W, 2W, 2W-1, -W
    signal s_oyf        : unsigned(21 downto 0) := (others => '0');          -- noise scroll, 16.6 lines
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    --------------------------------------------------------------------------
    -- Vblank sequencer (shared 11x11 multiplier, operand-split, 3-clock)
    --------------------------------------------------------------------------
    signal s_vstep   : unsigned(4 downto 0) := "11111";
    signal s_div_rem : unsigned(11 downto 0) := (others => '0');
    signal s_div_q   : unsigned(20 downto 0) := (others => '0');
    signal s_div_i   : unsigned(4 downto 0) := (others => '0');
    signal s_div_d   : unsigned(10 downto 0) := (others => '0');
    signal s_div_n   : unsigned(20 downto 0) := (others => '0');
    signal s_inv_hh  : unsigned(13 downto 0) := to_unsigned(7767, 14);  -- 2^20 / half_h
    signal s_inv_whh : unsigned(13 downto 0) := to_unsigned(3855, 14);  -- 2^18 / whh
    signal s_kw      : unsigned(13 downto 0) := to_unsigned(1024, 14);  -- Q10 W/ww
    signal s_kwsel   : unsigned(13 downto 0) := to_unsigned(1024, 14);  -- kw (Fit) or 1 (Lens)
    signal s_cabs    : unsigned(9 downto 0) := (others => '0');
    signal s_cneg    : std_logic := '0';
    signal s_zrom    : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_zoct    : unsigned(1 downto 0) := "10";
    signal s_zoom    : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_qa, s_qb : unsigned(10 downto 0) := (others => '0');
    signal s_vph     : unsigned(15 downto 0) := (others => '0');
    signal s_vpl     : unsigned(16 downto 0) := (others => '0');
    signal s_vp      : unsigned(21 downto 0) := (others => '0');
    signal s_amp_eff : unsigned(9 downto 0) := (others => '0');
    signal s_qfrac   : unsigned(10 downto 0) := to_unsigned(512, 11);   -- window width fraction, Q10
    signal s_ww      : unsigned(10 downto 0) := to_unsigned(240, 11);   -- window width
    signal s_whh     : unsigned(10 downto 0) := to_unsigned(68, 11);    -- window half height
    signal s_wcx0    : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_wcx     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_cy      : unsigned(10 downto 0) := to_unsigned(135, 11);
    signal s_ucw     : unsigned(10 downto 0) := to_unsigned(240, 11);   -- window content centre
    signal s_rim     : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal s_wx0s, s_wx1s, s_wy0s, s_wy1s : signed(12 downto 0) := (others => '0');
    signal s_wx0, s_wx1 : unsigned(10 downto 0) := (others => '0');
    signal s_wy0, s_wy1 : unsigned(10 downto 0) := (others => '0');
    signal s_bx0, s_bx1 : unsigned(10 downto 0) := (others => '0');
    signal s_by0, s_by1 : unsigned(10 downto 0) := (others => '0');
    signal s_drift_ph : unsigned(15 downto 0) := (others => '0');
    signal s_phinc   : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q   : signed(9 downto 0) := (others => '0');
    signal s_sx, s_sy : unsigned(7 downto 0) := (others => '0');
    signal s_sxneg, s_syneg : std_logic := '0';
    signal s_tx, s_ty : unsigned(9 downto 0) := (others => '0');
    signal s_dx      : unsigned(9 downto 0) := (others => '0');
    signal s_dyv     : unsigned(8 downto 0) := (others => '0');
    signal s_round   : std_logic := '0';                                -- per-frame shape
    signal s_kasp    : unsigned(11 downto 0) := to_unsigned(1024, 12);  -- Q10 px per line of a circle
    signal s_r2q     : unsigned(13 downto 0) := (others => '0');        -- R^2 / 64, R = ww/2
    signal s_dring   : unsigned(15 downto 0) := (others => '0');        -- 2Rt - t^2
    signal s_sqp     : unsigned(21 downto 0) := (others => '0');        -- (wcx + 1)^2
    signal s_sqip    : signed(12 downto 0) := (others => '0');          -- -(2 wcx + 1)

    --------------------------------------------------------------------------
    -- Per-line sequencer (shared 14x14 multiplier, operand-split; the
    -- post-processed product is valid 4 clocks after the operands are set)
    --------------------------------------------------------------------------
    signal s_lstep    : unsigned(4 downto 0) := "11111";
    signal s_ma, s_mb : unsigned(13 downto 0) := (others => '0');
    signal s_mph      : unsigned(20 downto 0) := (others => '0');
    signal s_mpl      : unsigned(20 downto 0) := (others => '0');
    signal s_mp       : unsigned(27 downto 0) := (others => '0');
    signal s_pp_rho   : unsigned(10 downto 0) := (others => '0');
    signal s_pp_w     : unsigned(10 downto 0) := (others => '0');
    signal s_pp_f     : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_pp_kz    : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_pp_kk    : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_dy       : unsigned(10 downto 0) := (others => '0');
    signal s_dyw4     : unsigned(12 downto 0) := (others => '0');
    signal s_kzg      : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_kk       : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_pg       : unsigned(23 downto 0) := (others => '0');
    signal s_pw       : unsigned(24 downto 0) := (others => '0');
    signal s_initg, s_initw : signed(25 downto 0) := (others => '0');
    signal s_stepw    : signed(14 downto 0) := (others => '0');
    signal s_inrows, s_brow : std_logic := '0';
    signal s_lww      : unsigned(10 downto 0) := (others => '0');
    signal s_tout, s_tin : unsigned(19 downto 0) := (others => '0');   -- circle / inner ring thresholds
    signal s_ncy, s_ncy1 : unsigned(8 downto 0) := (others => '0');
    signal s_nfy      : unsigned(4 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Noise lattice engine
    --------------------------------------------------------------------------
    signal s_nstep  : unsigned(3 downto 0) := "1111";
    signal s_ct     : unsigned(6 downto 0) := (others => '0');
    signal s_nrem   : unsigned(1 downto 0) := (others => '0');
    signal s_nshift : std_logic := '0';
    signal s_ha, s_hb : unsigned(15 downto 0) := (others => '0');
    signal s_h0     : unsigned(7 downto 0) := (others => '0');
    signal s_hd     : signed(8 downto 0) := (others => '0');
    signal s_hp     : signed(14 downto 0) := (others => '0');
    signal s_hl     : signed(9 downto 0) := (others => '0');
    signal s_hlc    : signed(8 downto 0) := (others => '0');
    signal s_hq     : signed(19 downto 0) := (others => '0');
    signal s_vA, s_vB, s_vC : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Resolution scaling (v0.2.3): the noise lattice (64 x 32), its
    -- displacement and the window size live on the 1080-line canvas.
    --------------------------------------------------------------------------
    signal s_xs    : unsigned(2 downto 0)  := to_unsigned(2, 3);     -- Q.1 x step per pixel
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);  -- Q4.8 y step per field line
    signal s_ilace : std_logic := '0';                               -- interlaced timing
    signal s_nxa   : unsigned(7 downto 0) := (others => '0');        -- lattice x, Q.1 (mod 128 units)
    signal s_nnode : std_logic := '0';                               -- this pixel crossed a lattice column
    signal s_lya   : unsigned(21 downto 0) := (others => '0');       -- lattice y, Q.8 virtual lines
    signal s_dcb, s_dba : signed(10 downto 0) := (others => '0');    -- vC - vB, vB - vA
    signal s_scb, s_sba : signed(11 downto 0) := (others => '0');    -- ... per pixel (x xs/2)
    signal s_amp_raw : unsigned(9 downto 0) := (others => '0');      -- amp in virtual units
    signal s_amp_h, s_amp_l : unsigned(9 downto 0) := (others => '0');
    signal s_wq1, s_wq2 : unsigned(10 downto 0) := (others => '0');
    signal s_wref  : unsigned(10 downto 0) := to_unsigned(480, 11);  -- W of a 16:9 screen this tall
    signal s_wmax  : unsigned(10 downto 0) := to_unsigned(472, 11);  -- widest window, W - W/64
    signal s_rimy  : unsigned(4 downto 0) := to_unsigned(4, 5);      -- rect rim in field lines

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    signal s_accg, s_accw : signed(25 downto 0) := (others => '0');
    signal s_nacc : signed(15 downto 0) := (others => '0');
    signal s_nstp : signed(11 downto 0) := (others => '0');
    signal s_x1   : unsigned(10 downto 0) := (others => '0');
    signal s_ug2, s_uw2 : signed(14 downto 0) := (others => '0');
    signal s_n2   : signed(9 downto 0) := (others => '0');
    signal s_sq   : unsigned(21 downto 0) := (others => '0');          -- (x - wcx)^2
    signal s_sqi  : signed(12 downto 0) := (others => '0');
    signal s_inr2, s_bordr2, s_inc2, s_bordc2 : std_logic := '0';
    signal s_x2   : unsigned(10 downto 0) := (others => '0');
    signal s_u3   : signed(14 downto 0) := (others => '0');
    signal s_bord3 : std_logic := '0';
    signal s_u4, s_rhi4 : signed(14 downto 0) := (others => '0');
    signal s_neg4, s_hi4, s_far4, s_bord4 : std_logic := '0';
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg5, s_bord5 : std_logic := '0';
    signal s_a_bg, s_a_rbank, s_a_bord : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    type t_y is array (8 to 11) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_y := (others => (others => '0'));
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    p_regs : process(clk)
    begin
        if rising_edge(clk) then
            s_k_curve <= unsigned(registers_in(0));
            s_k_zoom  <= unsigned(registers_in(1));
            s_k_noise <= unsigned(registers_in(2));
            s_k_speed <= unsigned(registers_in(3));
            s_k_size  <= unsigned(registers_in(4));
            s_k_pos   <= unsigned(registers_in(5));
            s_sw_outside   <= registers_in(6)(0);
            s_sw_noise_all <= registers_in(6)(1);
            s_sw_border    <= registers_in(6)(2);
            s_sw_round     <= registers_in(6)(3);
            s_sw_fit       <= registers_in(6)(4);
            s_p_drift <= unsigned(registers_in(7));
            -- per-mode steps, free-running (the timing ID is static per mode)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => s_ilace <= '1';
                when others             => s_ilace <= '0';
            end case;
        end if;
    end process;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 1 + sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_key  : unsigned(15 downto 0);
        variable v_ysum : unsigned(15 downto 0);
        variable v_v    : signed(9 downto 0);
        variable v_rem  : unsigned(12 downto 0);
        variable v_c    : signed(10 downto 0);
        variable v_x    : signed(12 downto 0);
        variable v_d    : unsigned(10 downto 0);
        variable v_kz   : unsigned(14 downto 0);
        variable v_kkq  : unsigned(17 downto 0);
        variable v_j    : unsigned(8 downto 0);
        variable v_wc   : unsigned(10 downto 0);
        variable v_ka   : unsigned(11 downto 0);
        variable v_nx   : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
            else
                s_rx <= (others => '0');
            end if;
            if s_in_avid = '1' then
                s_wr_x <= s_wr_x + 1;
            else
                s_wr_x <= (others => '0');
            end if;
            s_we <= data_in.avid;
            s_x1 <= s_rx;
            -- shared multipliers (operand-split, product valid 3 clocks after set)
            s_mph <= s_ma * s_mb(13 downto 7);
            s_mpl <= s_ma * s_mb(6 downto 0);
            s_mp  <= shift_left(resize(s_mph, 28), 7) + resize(s_mpl, 28);
            s_vph <= s_qa * s_qb(10 downto 6);
            s_vpl <= s_qa * s_qb(5 downto 0);
            s_vp  <= shift_left(resize(s_vph, 22), 6) + resize(s_vpl, 22);

            -- per-line product post-processing, free-running (one copy of
            -- each clamp, shared by the screen and window lens chains)
            if s_mp(27 downto 20) /= 0 then
                s_pp_rho <= to_unsigned(1024, 11);
            else
                s_pp_rho <= s_mp(20 downto 10);
            end if;
            if s_mp(21 downto 10) >= 1024 then
                s_pp_w <= (others => '0');
            else
                s_pp_w <= to_unsigned(1024, 11) - s_mp(20 downto 10);
            end if;
            s_pp_f <= f_lens(s_mp, s_cneg);
            v_kz := s_mp(24 downto 10);
            if s_mp(27 downto 25) /= 0 or v_kz > 8191 then
                s_pp_kz <= to_unsigned(8191, 13);
            elsif v_kz < 64 then
                s_pp_kz <= to_unsigned(64, 13);
            else
                s_pp_kz <= v_kz(12 downto 0);
            end if;
            v_kkq := s_mp(27 downto 10);
            if v_kkq > 16383 then
                s_pp_kk <= (others => '1');
            else
                s_pp_kk <= v_kkq(13 downto 0);
            end if;

            -- Resolution scaling: lattice steps per pixel (x 1 / 1.5 / 2 at
            -- 1080 / 720p / SD), free-running; the lattice values change
            -- at least a dozen clocks before they are used
            s_dcb <= resize(s_vC, 11) - resize(s_vB, 11);
            s_dba <= resize(s_vB, 11) - resize(s_vA, 11);
            case s_xs is
                when "011" =>
                    s_scb <= resize(s_dcb, 12) + resize(shift_right(s_dcb, 1), 12);
                    s_sba <= resize(s_dba, 12) + resize(shift_right(s_dba, 1), 12);
                when "100" =>
                    s_scb <= shift_left(resize(s_dcb, 12), 1);
                    s_sba <= shift_left(resize(s_dba, 12), 1);
                when others =>
                    s_scb <= resize(s_dcb, 12);
                    s_sba <= resize(s_dba, 12);
            end case;
            -- displacement in pixels: virtual amp x 2/xs (720p ~2/3, SD 1/2)
            s_amp_h <= ('0' & s_amp_raw(9 downto 1)) + ("000" & s_amp_raw(9 downto 3));
            s_amp_l <= ("00000" & s_amp_raw(9 downto 5)) + ("0000000" & s_amp_raw(9 downto 7));
            case s_xs is
                when "011"  => s_amp_eff <= s_amp_h + s_amp_l;
                when "100"  => s_amp_eff <= '0' & s_amp_raw(9 downto 1);
                when others => s_amp_eff <= s_amp_raw;
            end case;
            -- window width reference: W on 16:9 screens, 4/3 W on 4:3 SD
            -- (so the porthole follows the screen height); widest = W - W/64
            s_wq1 <= s_wf + shift_right(s_wf, 2);
            s_wq2 <= shift_right(s_wf, 4) + shift_right(s_wf, 6);
            if s_wf >= 1024 then s_wref <= s_wf; else s_wref <= s_wq1 + s_wq2; end if;
            s_wmax <= s_wf - shift_right(s_wf, 6);
            -- rect rim in field lines: half the pixel rim on interlaced fields
            if s_ilace = '1' then s_rimy <= '0' & s_rim(4 downto 1); else s_rimy <= s_rim; end if;

            -- lattice column crossings on the virtual x axis (64 units)
            if data_in.avid = '1' then
                v_nx    := s_nxa + s_xs;
                s_nxa   <= v_nx;
                s_nnode <= s_nxa(7) xor v_nx(7);
            else
                s_nxa   <= (others => '0');
                s_nnode <= '0';
            end if;

            -- DDAs: global map (step kz), window map (step +-kk); both start
            -- at the line start, so the window map needs no reset at its edge
            if data_in.avid = '1' then
                s_accg <= s_accg + signed(resize(s_kzg, 26));
                s_accw <= s_accw + resize(s_stepw, 26);
                s_sq   <= s_sq + unsigned(resize(s_sqi, 22));             -- (dx+1)^2 = dx^2 + 2dx + 1
                s_sqi  <= s_sqi + 2;
                if s_nnode = '1' then
                    s_nacc  <= shift_left(resize(s_vB, 16), 6);
                    s_nstp  <= s_scb;
                    s_vA    <= s_vB;
                    s_vB    <= s_vC;
                    s_nrem  <= "01";
                    s_nshift <= '0';
                    s_nstep <= (others => '0');
                else
                    s_nacc <= s_nacc + resize(s_nstp, 16);
                end if;
            else
                s_accg <= s_initg;
                s_accw <= s_initw;
                s_sq   <= s_sqp;
                s_sqi  <= s_sqip;
                s_nstp <= s_sba;
                s_nacc <= shift_left(resize(s_vA, 16), 6) - resize(s_sba, 16);
            end if;

            ------------------------------------------------------------------
            -- Noise lattice engine (roulette)
            ------------------------------------------------------------------
            case to_integer(s_nstep) is
                when 0 =>
                    v_key := s_ncy & s_ct;
                    s_ha  <= C_SEED_NOISE + v_key;
                    v_key := s_ncy1 & s_ct;
                    s_hb  <= C_SEED_NOISE + v_key;
                    s_nstep <= s_nstep + 1;
                when 1 => s_ha <= f_round1(s_ha); s_hb <= f_round1(s_hb); s_nstep <= s_nstep + 1;
                when 2 => s_ha <= f_round2(s_ha); s_hb <= f_round2(s_hb); s_nstep <= s_nstep + 1;
                when 3 => s_ha <= f_round3(s_ha); s_hb <= f_round3(s_hb); s_nstep <= s_nstep + 1;
                when 4 =>
                    s_h0 <= s_ha(15 downto 8);
                    s_hd <= signed('0' & s_hb(15 downto 8)) - signed('0' & s_ha(15 downto 8));
                    s_nstep <= s_nstep + 1;
                when 5 => s_hp <= s_hd * signed('0' & s_nfy); s_nstep <= s_nstep + 1;
                when 6 => s_hl <= signed(resize(s_h0, 10)) + resize(shift_right(s_hp, 5), 10); s_nstep <= s_nstep + 1;
                when 7 => s_hlc <= resize(s_hl - to_signed(128, 10), 9); s_nstep <= s_nstep + 1;
                when 8 => s_hq <= s_hlc * signed('0' & s_amp_eff); s_nstep <= s_nstep + 1;
                when 9 =>
                    v_v := resize(shift_right(s_hq, 9), 10);
                    if s_nshift = '1' then
                        s_vA <= s_vB;
                        s_vB <= s_vC;
                    end if;
                    s_vC <= v_v;
                    s_ct <= s_ct + 1;
                    if s_nrem > 1 then
                        s_nrem  <= s_nrem - 1;
                        s_nstep <= (others => '0');
                    else
                        s_nrem  <= (others => '0');
                        s_nstep <= "1111";
                    end if;
                when others => null;
            end case;

            ------------------------------------------------------------------
            -- Per-line sequencer: the screen lens (G) and the window lens (W)
            -- interleaved on the shared multiplier, one product per clock
            -- in flight, each read from the post-processed copy 4 later
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    if s_line >= s_half_h then s_dy <= s_line - s_half_h; else s_dy <= s_half_h - s_line; end if;
                    if s_line >= s_cy then v_d := s_line - s_cy; else v_d := s_cy - s_line; end if;
                    s_dyw4 <= shift_left(resize(v_d, 13), 2);
                    v_ysum := resize(s_lya(21 downto 8), 16) + s_oyf(21 downto 6);   -- virtual lines
                    s_ncy  <= v_ysum(13 downto 5);
                    s_ncy1 <= v_ysum(13 downto 5) + 1;
                    s_nfy  <= v_ysum(4 downto 0);
                    if s_line >= s_wy0 and s_line < s_wy1 then s_inrows <= '1'; else s_inrows <= '0'; end if;
                    if s_line < s_by0 or s_line >= s_by1 then s_brow <= '1'; else s_brow <= '0'; end if;
                    s_ct <= (others => '0'); s_nrem <= "11"; s_nshift <= '1'; s_nstep <= (others => '0');
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_ma <= resize(s_dy, 14); s_mb <= s_inv_hh;               -- G: rho = dy / half_h
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_ma <= resize(s_dyw4, 14); s_mb <= s_inv_whh;            -- W: rho = dyw / whh
                    s_lstep <= s_lstep + 1;
                when 5 | 6 =>
                    s_ma <= resize(s_pp_rho, 14); s_mb <= resize(s_pp_rho, 14);  -- rho^2
                    s_lstep <= s_lstep + 1;
                when 9 | 10 =>
                    s_lww <= s_pp_w;                                          -- (state 10: the window's)
                    s_ma <= resize(s_cabs, 14); s_mb <= resize(s_pp_w, 14);   -- |c| * (1 - rho^2)
                    s_lstep <= s_lstep + 1;
                when 11 =>
                    s_ma <= s_r2q; s_mb <= resize(s_lww, 14);                 -- circle: R^2 * (1 - rho^2)
                    s_lstep <= s_lstep + 1;
                when 13 | 14 =>
                    s_tout <= s_mp(23 downto 4);                              -- (state 14: the circle's)
                    s_ma <= resize(s_zoom, 14); s_mb <= resize(s_pp_f, 14);   -- kz = zoom * f
                    s_lstep <= s_lstep + 1;
                when 15 =>
                    if s_tout > s_dring then
                        s_tin <= s_tout - resize(s_dring, 20);                -- inner edge of the rim
                    else
                        s_tin <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when 17 =>
                    s_kzg <= s_pp_kz;
                    s_ma <= resize(s_half_w, 14); s_mb <= resize(s_pp_kz, 14); -- G: pg = half_w * kz
                    s_lstep <= s_lstep + 1;
                when 18 =>
                    s_ma <= s_kwsel; s_mb <= resize(s_pp_kz, 14);             -- W: kk = kw * kz
                    s_lstep <= s_lstep + 1;
                when 20 =>
                    s_pg <= s_mp(23 downto 0);
                    s_lstep <= s_lstep + 1;
                when 21 =>
                    s_initg <= signed(shift_left(resize(s_half_w, 26), 10)) - signed(resize(s_pg, 26)) - signed(resize(s_kzg, 26));
                    s_lstep <= s_lstep + 1;
                when 22 =>
                    s_kk <= s_pp_kk;
                    s_ma <= resize(s_wcx, 14); s_mb <= s_pp_kk;              -- W: pw = wcx * kk
                    s_lstep <= s_lstep + 1;
                when 25 =>
                    s_pw <= s_mp(24 downto 0);
                    s_lstep <= s_lstep + 1;
                when 26 =>
                    -- u(x) = uc + (x - wcx) * kk, preset one step before x = 0
                    s_initw <= signed(shift_left(resize(s_ucw, 26), 10)) - signed(resize(s_pw, 26)) - signed(resize(s_kk, 26));
                    s_stepw <= signed(resize(s_kk, 15));
                    s_lstep <= s_lstep + 1;
                when 31 => null;
                when others => s_lstep <= s_lstep + 1;
            end case;

            ------------------------------------------------------------------
            -- Vblank sequencer: glides, window geometry, drift, dividers
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_half_h <= resize(s_height(10 downto 1), 11);
                    s_half_w <= resize(s_line_width(10 downto 1), 11);
                    s_wf     <= s_line_width;
                    s_d_curve <= f_gdiff(s_k_curve, s_g_curve);
                    s_d_zoom  <= f_gdiff(s_k_zoom, s_g_zoom);
                    s_d_size  <= f_gdiff(s_k_size, s_g_size);
                    s_d_pos   <= f_gdiff(s_k_pos, s_g_pos);
                    s_d_drift <= f_gdiff(s_p_drift, s_g_drift);
                    s_qa <= resize(s_k_noise, 11); s_qb <= resize(s_k_noise, 11);   -- amp = k^2
                    s_sin_addr <= s_drift_ph(15 downto 8);                          -- sin(theta)
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_g_curve <= f_glide(s_g_curve, s_d_curve);
                    s_g_zoom  <= f_glide(s_g_zoom, s_d_zoom);
                    s_g_size  <= f_glide(s_g_size, s_d_size);
                    s_g_pos   <= f_glide(s_g_pos, s_d_pos);
                    s_g_drift <= f_glide(s_g_drift, s_d_drift);
                    s_w15  <= signed(resize(s_wf, 15));
                    s_w2   <= signed(shift_left(resize(s_wf, 15), 1));
                    s_w2m1 <= signed(shift_left(resize(s_wf, 15), 1)) - 1;
                    s_nw   <= -signed(resize(s_wf, 15));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    if s_sin_q < 0 then
                        s_sx <= resize(unsigned(-s_sin_q), 8); s_sxneg <= '1';
                    else
                        s_sx <= resize(unsigned(s_sin_q), 8); s_sxneg <= '0';
                    end if;
                    s_sin_addr <= s_drift_ph(14 downto 7);                          -- sin(2 theta)
                    s_qa <= resize(s_g_drift(12 downto 3), 11); s_qb <= resize(s_g_drift(12 downto 3), 11);  -- speed
                    -- |c| up to 512: negate at full width, then slice
                    v_c := signed(resize(s_g_curve(12 downto 3), 11)) - to_signed(512, 11);
                    if v_c < 0 then
                        v_c := -v_c; s_cneg <= '1';
                    else
                        s_cneg <= '0';
                    end if;
                    s_cabs <= unsigned(v_c(9 downto 0));
                    -- zoom index runs CW = magnify: j = 256 - i, 128 = 1x
                    v_j := to_unsigned(256, 9) - resize(s_g_zoom(12 downto 5), 9);
                    if v_j(8) = '1' then v_j := to_unsigned(255, 9); end if;
                    s_zrom <= C_ZOOM_ROM(to_integer(v_j(5 downto 0)));
                    s_zoct <= v_j(7 downto 6);
                    s_qfrac <= to_unsigned(103, 11) + resize(s_g_size(12 downto 3), 11) - resize(s_g_size(12 downto 6), 11);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_amp_raw <= s_vp(19 downto 10);
                    s_qa <= resize(s_sx, 11); s_qb <= resize(s_g_drift(12 downto 3), 11);   -- tx = |sin| * drift
                    case s_zoct is
                        when "00"   => s_zoom <= resize(s_zrom(10 downto 2), 13);
                        when "01"   => s_zoom <= resize(s_zrom(10 downto 1), 13);
                        when "10"   => s_zoom <= resize(s_zrom, 13);
                        when others => s_zoom <= resize(s_zrom, 12) & '0';
                    end case;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    if s_sin_q < 0 then
                        s_sy <= resize(unsigned(-s_sin_q), 8); s_syneg <= '1';
                    else
                        s_sy <= resize(unsigned(s_sin_q), 8); s_syneg <= '0';
                    end if;
                    s_qa <= s_qfrac; s_qb <= s_wref;                                -- ww (follows H)
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_phinc <= to_unsigned(32, 10) + resize(s_vp(19 downto 11), 10);  -- 32 + drift^2/2048
                    s_qa <= resize(s_sy, 11); s_qb <= resize(s_g_drift(12 downto 3), 11);   -- ty
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_tx <= s_vp(17 downto 8);
                    s_qa <= s_qfrac; s_qb <= s_height;                              -- whh
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    if s_vp(20 downto 10) < 16 then
                        s_ww <= to_unsigned(16, 11);
                    elsif s_vp(20 downto 10) > s_wmax then
                        s_ww <= s_wmax;                                             -- SD only (never at HD)
                    else
                        s_ww <= s_vp(20 downto 10);
                    end if;
                    s_qa <= resize(s_g_pos(12 downto 3), 11); s_qb <= s_wf;         -- position
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    s_ty <= s_vp(17 downto 8);
                    s_qa <= resize(s_tx, 11); s_qb <= s_half_w;                     -- dx
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    if s_vp(20 downto 11) < 4 then s_whh <= to_unsigned(4, 11); else s_whh <= resize(s_vp(20 downto 11), 11); end if;
                    s_qa <= resize(s_ty, 11); s_qb <= s_half_h;                     -- dy
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    s_wcx0 <= s_vp(20 downto 10);
                    s_rim  <= resize(s_ww(10 downto 7), 5) + 2;
                    s_qa <= resize(s_ww(10 downto 1), 11); s_qb <= resize(s_ww(10 downto 1), 11);  -- R^2
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    s_dx <= s_vp(19 downto 10);
                    s_qa <= shift_left(resize(s_ww(10 downto 1), 11), 1) - resize(s_rim, 11);       -- (2R - t) * t
                    s_qb <= resize(s_rim, 11);
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    s_dyv <= s_vp(19 downto 11);
                    if s_sxneg = '1' then
                        v_x := signed(resize(s_wcx0, 13)) - signed(resize(s_dx, 13));
                    else
                        v_x := signed(resize(s_wcx0, 13)) + signed(resize(s_dx, 13));
                    end if;
                    if v_x < 0 then
                        s_wcx <= (others => '0');
                    elsif v_x > signed(resize(s_wf, 13)) then
                        s_wcx <= s_wf;
                    else
                        s_wcx <= unsigned(v_x(10 downto 0));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    s_r2q  <= s_vp(19 downto 6);
                    s_qa <= s_wcx + 1; s_qb <= s_wcx + 1;                           -- (wcx + 1)^2, held
                    s_sqip <= to_signed(-1, 13) - signed(shift_left(resize(s_wcx, 13), 1));
                    if s_syneg = '1' then
                        s_cy <= s_half_h - resize(s_dyv, 11);
                    else
                        s_cy <= s_half_h + resize(s_dyv, 11);
                    end if;
                    s_wx0s <= signed(resize(s_wcx, 13)) - signed(resize(s_ww(10 downto 1), 13));
                    s_wx1s <= signed(resize(s_wcx, 13)) + signed(resize(s_ww(10 downto 1), 13));
                    s_oyf <= s_oyf + resize(s_k_speed, 22);
                    s_drift_ph <= s_drift_ph + resize(s_phinc, 16);
                    s_vstep <= s_vstep + 1;
                when 14 =>
                    s_dring <= s_vp(15 downto 0);
                    s_wy0s <= signed(resize(s_cy, 13)) - signed(resize(s_whh, 13));
                    s_wy1s <= signed(resize(s_cy, 13)) + signed(resize(s_whh, 13));
                    s_wx0 <= f_clip11(s_wx0s);
                    s_wx1 <= f_clip11(s_wx1s);
                    s_bx0 <= f_clip11(s_wx0s + signed(resize(s_rim, 13)));
                    s_bx1 <= f_clip11(s_wx1s - signed(resize(s_rim, 13)));
                    if s_sw_fit = '1' then s_ucw <= s_half_w; else s_ucw <= s_wcx; end if;
                    s_vstep <= s_vstep + 1;
                when 15 =>
                    s_wy0 <= f_clip11(s_wy0s);
                    s_wy1 <= f_clip11(s_wy1s);
                    s_by0 <= f_clip11(s_wy0s + signed(resize(s_rimy, 13)));
                    s_by1 <= f_clip11(s_wy1s - signed(resize(s_rimy, 13)));
                    s_div_n <= "100000000000000000000";                             -- 2^20 / half_h
                    s_div_d <= s_half_h;
                    s_div_rem <= (others => '0'); s_div_q <= (others => '0'); s_div_i <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 16 | 18 | 20 | 22 =>
                    v_rem := s_div_rem & s_div_n(20);
                    if v_rem >= resize(s_div_d, 13) then
                        s_div_rem <= resize(v_rem - resize(s_div_d, 13), 12);
                        s_div_q   <= s_div_q(19 downto 0) & '1';
                    else
                        s_div_rem <= v_rem(11 downto 0);
                        s_div_q   <= s_div_q(19 downto 0) & '0';
                    end if;
                    s_div_n <= s_div_n(19 downto 0) & '0';
                    if s_div_i = 20 then
                        s_vstep <= s_vstep + 1;
                    end if;
                    s_div_i <= s_div_i + 1;
                when 17 =>
                    if s_half_h < 16 then s_inv_hh <= (others => '0'); else s_inv_hh <= s_div_q(13 downto 0); end if;
                    s_sqp <= s_vp(21 downto 0);
                    -- circle aspect: px per line = W * (9/16 HD, 3/4 SD) / H
                    if s_wf >= 1024 then
                        v_wc := resize(s_wf(10 downto 1), 11) + resize(s_wf(10 downto 4), 11);
                    else
                        v_wc := resize(s_wf(10 downto 1), 11) + resize(s_wf(10 downto 2), 11);
                    end if;
                    s_div_n <= shift_left(resize(v_wc, 21), 10);
                    s_div_d <= s_height;
                    s_div_rem <= (others => '0'); s_div_q <= (others => '0'); s_div_i <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 19 =>
                    if s_div_q > 4095 then v_ka := (others => '1'); else v_ka := s_div_q(11 downto 0); end if;
                    s_kasp  <= v_ka;
                    s_round <= s_sw_round;
                    -- window rho divisor: circle (kasp/R) or rectangle (1/whh)
                    if s_sw_round = '1' then
                        s_div_n <= shift_left(resize(v_ka, 21), 8);                 -- kasp * 2^8 / R
                        s_div_d <= resize(s_ww(10 downto 1), 11);
                    else
                        s_div_n <= "001000000000000000000";                         -- 2^18 / whh
                        s_div_d <= s_whh;
                    end if;
                    s_div_rem <= (others => '0'); s_div_q <= (others => '0'); s_div_i <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 21 =>
                    if s_div_q > 16383 then s_inv_whh <= (others => '1'); else s_inv_whh <= s_div_q(13 downto 0); end if;
                    s_div_n <= shift_left(resize(s_wf, 21), 10);                    -- W * 1024 / ww
                    s_div_d <= s_ww;
                    s_div_rem <= (others => '0'); s_div_q <= (others => '0'); s_div_i <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 23 =>
                    if s_div_q > 16383 then s_kw <= (others => '1'); else s_kw <= s_div_q(13 downto 0); end if;
                    s_vstep <= s_vstep + 1;
                when 24 =>
                    if s_sw_fit = '1' then s_kwsel <= s_kw; else s_kwsel <= to_unsigned(1024, 14); end if;
                    s_vstep <= "11111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line   <= s_line + 1;
                s_lya    <= s_lya + s_ys;
                s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_lya  <= (others => '0');
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
    -- Address chain 2..5
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_u : signed(14 downto 0);
        variable v_in, v_bd : std_logic;
    begin
        if rising_edge(clk) then
            -- stage 2: window test (per-frame bounds, per-line row flags)
            s_ug2 <= s_accg(24 downto 10);
            s_uw2 <= s_accw(24 downto 10);
            s_n2  <= s_nacc(15 downto 6);
            if s_inrows = '1' and s_x1 >= s_wx0 and s_x1 < s_wx1 then v_in := '1'; else v_in := '0'; end if;
            s_inr2 <= v_in;
            if v_in = '1' and (s_brow = '1' or s_x1 < s_bx0 or s_x1 >= s_bx1) then
                s_bordr2 <= '1';
            else
                s_bordr2 <= '0';
            end if;
            if s_sq < resize(s_tout, 22) then v_in := '1'; else v_in := '0'; end if;
            s_inc2 <= v_in;
            if v_in = '1' and s_sq >= resize(s_tin, 22) then
                s_bordc2 <= '1';
            else
                s_bordc2 <= '0';
            end if;
            s_x2 <= s_x1;
            -- stage 3: select shape and map, add noise
            if s_round = '1' then
                v_in := s_inc2; v_bd := s_bordc2;
            else
                v_in := s_inr2; v_bd := s_bordr2;
            end if;
            if v_in = '1' then
                v_u := s_uw2 + resize(s_n2, 15);
            elsif s_sw_outside = '1' then
                v_u := s_ug2;
                if s_sw_noise_all = '1' then v_u := v_u + resize(s_n2, 15); end if;
            else
                v_u := signed(resize(s_x2, 15));
                if s_sw_noise_all = '1' then v_u := v_u + resize(s_n2, 15); end if;
            end if;
            s_u3 <= v_u;
            s_bord3 <= v_bd and s_sw_border;
            -- stage 4: range tests on the source value, mirror candidate
            s_u4   <= s_u3;
            s_rhi4 <= s_w2m1 - s_u3;
            s_neg4 <= s_u3(14);
            if s_u3 >= s_w15 then s_hi4 <= '1'; else s_hi4 <= '0'; end if;
            if s_u3 < s_nw or s_u3 >= s_w2 then s_far4 <= '1'; else s_far4 <= '0'; end if;
            s_bord4 <= s_bord3;
            -- stage 5: mirror once off either edge, black beyond
            if s_neg4 = '1' then
                s_addr <= unsigned(not s_u4(10 downto 0));
            elsif s_hi4 = '1' then
                s_addr <= unsigned(s_rhi4(10 downto 0));
            else
                s_addr <= unsigned(s_u4(10 downto 0));
            end if;
            s_bg5 <= s_far4;
            s_bord5 <= s_bord4;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (read at stage 6)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 6..12
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            s_a_bg <= s_bg5; s_a_rbank <= s_rbank; s_a_bord <= s_bord5;
            if s_a_bg = '1' or s_a_bord = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            s_yd(8) <= s_wet_y; s_ud(8) <= s_wet_u; s_vd(8) <= s_wet_v;
            for i in 9 to 11 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
            end loop;
            if s_avid_sr(10) = '1' then
                s_y12 <= s_yd(11); s_u12 <= s_ud(11); s_v12 <= s_vd(11);
            else
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y12);
    data_out.u       <= std_logic_vector(s_u12);
    data_out.v       <= std_logic_vector(s_v12);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture porthole;
