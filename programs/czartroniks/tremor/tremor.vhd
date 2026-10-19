-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Tremor: catalogue combo #17 (seed 20260915) -- Perlin-style displacement
-- + scanline shear + duotone.  Two knobs each; the switches and the slider
-- decide how the three combine.
--
--   K1 Scale    noise cell size 4 .. 512 px, log (64 steps per octave);
--               the field zooms about the screen centre
--   K2 Drift    two-way: centre = still, right = the field rises, left =
--               it sinks, up to 8 px a frame.  The field boils as it
--               drifts: every row of lattice points also slides sideways
--               at its own random speed (up to the drift speed), so the
--               noise shears and churns instead of moving as one sheet
--   K3 Skew     scanline shear: rows lean by (row in period - P/2) * skew
--   K4 Period   shear band height 2 .. 257 lines
--   K5 Hue      highlight tone hue
--   K6 Spread   angle between shadow and highlight hues (0 = one tint)
--
--   S7  Noise   warps the picture horizontally / grains the luma instead
--   S8  Shear   sawtooth / triangle
--   S9  Bands   Shade: every other band's tone key is lifted a little (the
--               bands always show) / Flip: every other band swaps the tones
--   S10 Field   smooth value noise / hard cells
--   S11 Tone    duotone keyed from luma / from the noise field itself
--   P12 Amount  displacement (or grain), glided, squared curve: a small
--               always-on tremor at 0 up to +-1/4 width (+-511 luma) at
--               100%.  Above 75% the quake comes in: slabs of lines (random
--               heights, new every frame) torn sideways, up to +-1/8 width
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- 2-D value noise: the lattice is stepped by DDAs (x per pixel, one DDA
-- each for the cell's top and bottom lattice rows, which slide separately;
-- y per active line) at a per-frame step from a 2^(-i/64) ROM + octave
-- shift, from starts that put the screen centre at a fixed lattice point.
-- Lattice key = per-row hash (row index mod 1024, the hash input wrapping
-- with it) + cell column mod 512; four 16-bit add/xor-shift hashes per
-- pixel; bilinear lerp with Q7 fractions.  Per-line shear, row slides,
-- quake slab (24-bit LFSR) and row keys from an hblank sequencer; duotone
-- vectors, scale step, drift, centring offsets and the amount / quake gains
-- per frame (one serial shift-add multiplier).
--
-- HARDWARE CONTRACTS: Tremor draws all of its own chroma (the duotone
-- replaces the source's), so it runs its own generated avid (parity-locked
-- start, frame-latched width; damier/aurora/infrared) and conditions the
-- output ([1 2 1] on Y/U/V + luma scaled to white ~775: the HD HDMI bar fix
-- from damier v0.4.4).  Latency 30 clocks in all modes; outputs gated to
-- neutral outside the generated avid.
--
-- v0.3.2: resolution scaling.  Lattice steps, shear band period, shear slope
-- and quake slab heights run in virtual 1080-line units (x step per pixel
-- and y step per field line from the video timing ID in registers_in(8));
-- warp, shear and quake displacements are scaled by 1/x step (height-based),
-- so cells, bands and tears keep their size relative to the screen height
-- at SD and 720p.  1080p is unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tremor of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (30 + 2 = 32 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 32;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT   : integer := 30;
    constant C_SEED1 : unsigned(15 downto 0) := x"5A3C";
    constant C_XBIAS : unsigned(24 downto 0) := to_unsigned(16777216, 25);   -- cell 256 at the centre
    constant C_SHADE : integer := 24;    -- S9 Shade: key lift on alternate bands

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return unsigned(to_unsigned(1023, 10));
        else
            return unsigned(v(9 downto 0));
        end if;
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

    -- lattice step per pixel (Q16 cells) for a 4 px cell * 2^(m/64);
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

    -- luma line buffers (the duotone replaces the chroma, so only luma is
    -- buffered).  Collision-free in use: the bank being written is never the
    -- one whose read data is selected.
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;

    -- controls / per-frame
    signal s_k_scale, s_k_drift, s_k_skew, s_k_period, s_k_hue, s_k_spread : unsigned(9 downto 0);
    signal s_sw_grain, s_sw_tri, s_sw_flip, s_sw_blocky, s_sw_tonenoise : std_logic;
    signal s_p_amount : unsigned(9 downto 0);
    signal s_skg    : unsigned(12 downto 0) := (others => '0');    -- Skew glide (Q3)
    signal s_pg     : unsigned(13 downto 0) := (others => '0');    -- Amount glide (Q4)
    signal s_skew_v : unsigned(9 downto 0) := (others => '0');      -- skew, virtual px per virtual line (x128)
    signal s_skew   : unsigned(9 downto 0) := (others => '0');      -- ... real px per virtual line
    signal s_sk_a, s_sk_b : unsigned(9 downto 0) := (others => '0');
    signal s_period : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_peff   : unsigned(8 downto 0) := to_unsigned(64, 9);   -- period, at least one field line
    -- resolution scaling (free-running from the timing ID)
    signal s_vxs    : unsigned(2 downto 0) := to_unsigned(2, 3);        -- x step, Q.1
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);    -- y step per field line, Q.8
    signal s_ymin   : unsigned(8 downto 0) := to_unsigned(1, 9);        -- ceil(y step)
    signal s_wv     : unsigned(10 downto 0) := to_unsigned(1920, 11);   -- 1080p width in real px
    signal s_shc    : signed(12 downto 0) := to_signed(511, 13);        -- shear clamp, real px
    signal s_qthr   : unsigned(7 downto 0) := to_unsigned(16, 8);       -- quake: new slab if r8 < this
    signal s_xstep, s_ystep : unsigned(16 downto 0) := to_unsigned(2048, 17);  -- real lattice steps
    signal s_sidx   : unsigned(8 downto 0) := to_unsigned(192, 9);  -- scale: octave & mantissa
    signal s_srom_addr : unsigned(5 downto 0) := (others => '0');
    signal s_srom_q, s_srom_q2 : unsigned(14 downto 0) := to_unsigned(16384, 15);
    signal s_step   : unsigned(14 downto 0) := to_unsigned(2048, 15);
    signal s_dspd   : unsigned(6 downto 0) := (others => '0');      -- drift speed, Drift/16 px a frame
    signal s_dup    : std_logic := '0';                             -- '1': the field rises
    signal s_dinc   : unsigned(17 downto 0) := (others => '0');
    signal s_ydrift : unsigned(25 downto 0) := (others => '0');
    signal s_yoff   : unsigned(25 downto 0) := (others => '0');
    signal s_bph    : unsigned(31 downto 0) := (others => '0');     -- boil phase, Q16 cells x 128 (row slides)
    signal s_xst    : unsigned(24 downto 0) := C_XBIAS;
    signal s_hw, s_hh : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_c      : unsigned(10 downto 0) := (others => '0');
    signal s_m      : unsigned(8 downto 0) := (others => '0');     -- px (warp) or luma (grain) at full noise
    signal s_grain  : std_logic := '0';
    signal s_q8     : unsigned(7 downto 0) := (others => '0');
    signal s_qamp   : unsigned(7 downto 0) := (others => '0');     -- quake px at full tear
    signal s_vstep  : unsigned(5 downto 0) := "111111";
    signal s_ma, s_macc : unsigned(25 downto 0) := (others => '0');
    signal s_mb     : unsigned(10 downto 0) := (others => '0');
    signal s_mret   : unsigned(5 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_sB, s_cB, s_sA, s_cA : signed(9 downto 0) := (others => '0');
    signal s_uA, s_vA, s_uB, s_vB : signed(11 downto 0) := to_signed(512, 12);
    signal s_du, s_dv : signed(11 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_du : signal is true;
    attribute keep of s_dv : signal is true;

    -- tracking / per-line
    signal s_rx, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_line   : unsigned(10 downto 0) := (others => '0');
    signal s_height : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y   : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_lstep  : unsigned(3 downto 0) := "1111";
    signal s_lpa    : unsigned(16 downto 0) := (others => '0');     -- line in band, Q.8 virtual
    signal s_dir, s_band : std_logic := '0';
    signal s_lq     : unsigned(8 downto 0) := (others => '0');
    signal s_lt     : signed(9 downto 0) := (others => '0');
    signal s_shph, s_shpl : signed(15 downto 0) := (others => '0');
    signal s_shp    : signed(18 downto 0) := (others => '0');
    signal s_sh_base : signed(10 downto 0) := (others => '0');
    signal s_sh_off : signed(10 downto 0) := (others => '0');
    signal s_lfsr   : unsigned(23 downto 0) := x"ACE1F3";
    signal s_qhold  : unsigned(7 downto 0) := x"80";
    signal s_qa, s_qacc : signed(16 downto 0) := (others => '0');
    signal s_qb     : unsigned(7 downto 0) := (others => '0');
    signal s_bpa, s_bacc_t, s_bacc_b : unsigned(31 downto 0) := (others => '0');
    signal s_brt, s_brb : unsigned(7 downto 0) := (others => '0');
    signal s_bi     : unsigned(2 downto 0) := (others => '0');
    signal s_xst_t, s_xst_b : unsigned(24 downto 0) := C_XBIAS;
    signal s_yacc   : unsigned(25 downto 0) := (others => '0');
    signal s_fy     : unsigned(6 downto 0) := (others => '0');
    signal s_h0, s_h1 : unsigned(15 downto 0) := (others => '0');
    signal s_rk0, s_rk1 : unsigned(15 downto 0) := (others => '0');
    signal s_band_row : std_logic := '0';
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- width - 1 - C_EDGE

    --------------------------------------------------------------------------
    -- Generated avid: starts 3 or 4 clocks after the source's first avid
    -- rise, whichever keeps the hsync->start parity at its voted usual value;
    -- ends after the frame-latched width (damier / aurora, HW-confirmed)
    --------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_go : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(10 downto 0) := (others => '0');
    signal g_wm1 : unsigned(10 downto 0) := to_unsigned(479, 11);
    -- lattice x (Q16 cells) along the cell's top and bottom lattice rows
    signal s_xacc_t, s_xacc_b : unsigned(24 downto 0) := C_XBIAS;

    -- noise pipeline
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_cxt, s_cxt1, s_cxb, s_cxb1 : unsigned(8 downto 0) := (others => '0');
    signal s_fxt, s_fxb : unsigned(6 downto 0) := (others => '0');
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
    signal s_nd13   : unsigned(7 downto 0) := (others => '0');
    signal s_d14    : signed(18 downto 0) := (others => '0');
    signal s_off15  : signed(9 downto 0) := (others => '0');
    signal s_gr15   : signed(9 downto 0) := (others => '0');
    type t_fx is array (2 to 7) of unsigned(6 downto 0);
    signal s_fxtd, s_fxbd : t_fx := (others => (others => '0'));
    type t_xs is array (2 to 15) of signed(12 downto 0);
    signal s_xs     : t_xs := (others => (others => '0'));
    type t_n8 is array (14 to 23) of unsigned(7 downto 0);
    signal s_nt     : t_n8 := (others => (others => '0'));          -- noise tone key (+ shade)
    type t_g10 is array (16 to 21) of signed(9 downto 0);
    signal s_gd     : t_g10 := (others => (others => '0'));

    -- address / fetch / duotone
    signal s_u16    : signed(12 downto 0) := (others => '0');
    signal s_addr17 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y  : unsigned(9 downto 0) := (others => '0');
    signal s_yc20   : unsigned(9 downto 0) := (others => '0');
    signal s_yo20   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ys21   : unsigned(11 downto 0) := (others => '0');
    signal s_yo21   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y8_22  : unsigned(7 downto 0) := (others => '0');
    signal s_gq22   : signed(8 downto 0) := (others => '0');
    signal s_yo22   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y8g23  : unsigned(7 downto 0) := (others => '0');
    signal s_yo23   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_t24    : unsigned(7 downto 0) := (others => '0');
    signal s_yo24   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pu25, s_pv25 : signed(20 downto 0) := (others => '0');
    signal s_yo25   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y26    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y30    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bandd  : std_logic_vector(2 to 23) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
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

    -- 24-bit Galois LFSR (x^24 + x^23 + x^22 + x^17 + 1), 8 steps
    function f_lfsr8(v : unsigned(23 downto 0)) return unsigned is
        variable t : unsigned(23 downto 0);
    begin
        t := v;
        for i in 0 to 7 loop
            if t(0) = '1' then
                t := shift_right(t, 1) xor x"E10000";
            else
                t := shift_right(t, 1);
            end if;
        end loop;
        return t;
    end function;

begin

    s_k_scale  <= unsigned(registers_in(0));
    s_k_drift  <= unsigned(registers_in(1));
    s_k_skew   <= unsigned(registers_in(2));
    s_k_period <= unsigned(registers_in(3));
    s_k_hue    <= unsigned(registers_in(4));
    s_k_spread <= unsigned(registers_in(5));
    s_sw_grain     <= registers_in(6)(0);
    s_sw_tri       <= registers_in(6)(1);
    s_sw_flip      <= registers_in(6)(2);
    s_sw_blocky    <= registers_in(6)(3);
    s_sw_tonenoise <= registers_in(6)(4);
    s_p_amount <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q  <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
            s_srom_q  <= to_unsigned(C_STEP(to_integer(s_srom_addr)), 15);
            s_srom_q2 <= s_srom_q;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Generated avid
    --------------------------------------------------------------------------
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
                g_go <= '1';
            else
                g_go <= '0';
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

    --------------------------------------------------------------------------
    -- Control: tracking, per-line and per-frame sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t  : signed(12 downto 0);
        variable v_cy : unsigned(9 downto 0);
        variable v_s13 : unsigned(12 downto 0);
        variable v_sd : signed(14 downto 0);
        variable v_pd : signed(15 downto 0);
        variable v_dv, v_dn : signed(10 downto 0);
        variable v_da : unsigned(9 downto 0);
        variable v_c  : unsigned(11 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_a  : unsigned(9 downto 0);
        variable v_ln : unsigned(16 downto 0);
        variable v_r8 : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- resolution scaling: per-mode steps; the 1920-unit 1080p width
            -- and the shear clamp in real pixels (2 / x step); skew from
            -- virtual to real px (720p: 1/2+1/8+1/32+1/128); quake slab odds
            -- 1 in 16 virtual lines
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            if s_ys(7 downto 0) = 0 then
                s_ymin <= resize(s_ys(11 downto 8), 9);
            else
                s_ymin <= resize(s_ys(11 downto 8), 9) + 1;
            end if;
            s_qthr <= s_ys(11 downto 4);
            case to_integer(s_vxs) is
                when 4 =>
                    s_wv <= to_unsigned(960, 11);  s_shc <= to_signed(255, 13);
                    s_sk_a <= shift_right(s_skew_v, 1); s_sk_b <= (others => '0');
                when 3 =>
                    s_wv <= to_unsigned(1280, 11); s_shc <= to_signed(340, 13);
                    s_sk_a <= shift_right(s_skew_v, 1) + shift_right(s_skew_v, 3);
                    s_sk_b <= shift_right(s_skew_v, 5) + shift_right(s_skew_v, 7);
                when others =>
                    s_wv <= to_unsigned(1920, 11); s_shc <= to_signed(511, 13);
                    s_sk_a <= s_skew_v; s_sk_b <= (others => '0');
            end case;
            s_skew <= s_sk_a + s_sk_b;

            -- lattice x DDAs (top and bottom lattice rows of the cell),
            -- stepped over the generated avid from per-line starts
            if g_avid = '1' then
                s_xacc_t <= s_xacc_t + resize(s_xstep, 25);
                s_xacc_b <= s_xacc_b + resize(s_xstep, 25);
            else
                s_xacc_t <= s_xst_t;
                s_xacc_b <= s_xst_b;
            end if;

            -- per-line sequencer: shear offset, quake slab, row keys, row slides
            case to_integer(s_lstep) is
                when 0 =>
                    if s_sw_tri = '1' and s_dir = '1' then
                        s_lq <= s_peff - 1 - s_lpa(16 downto 8);
                    else
                        s_lq <= s_lpa(16 downto 8);
                    end if;
                    s_band_row <= s_band;
                    v_cy := s_yacc(25 downto 16);
                    s_fy <= s_yacc(15 downto 9);
                    -- row index wraps at 1024 with the key
                    s_h0 <= C_SEED1 + resize(v_cy, 16);
                    s_h1 <= C_SEED1 + resize(v_cy + 1, 16);
                    s_lfsr <= f_lfsr8(s_lfsr);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_lt <= signed('0' & s_lq) - signed('0' & s_peff(8 downto 1));
                    s_h0 <= f_round1(s_h0); s_h1 <= f_round1(s_h1);
                    -- quake: a new slab on 1 virtual line in 16 (and on the
                    -- first); at 1080p the test is lfsr(3:0) = 0
                    v_r8 := s_lfsr(3 downto 0) & s_lfsr(7 downto 4);
                    if v_r8 < s_qthr or s_line = 0 then
                        s_qhold <= s_lfsr(15 downto 8);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_shph <= s_lt * signed('0' & s_skew(9 downto 5));
                    s_shpl <= s_lt * signed('0' & s_skew(4 downto 0));
                    s_h0 <= f_round2(s_h0); s_h1 <= f_round2(s_h1);
                    s_qa <= resize(signed('0' & s_qhold) - to_signed(128, 10), 17);
                    s_qb <= s_qamp;
                    s_qacc <= (others => '0');
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_shp <= shift_left(resize(s_shph, 19), 5) + resize(s_shpl, 19);
                    s_rk0 <= f_round3(s_h0); s_rk1 <= f_round3(s_h1);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    v_t := resize(shift_right(s_shp, 7), 13);
                    if v_t > s_shc then v_t := s_shc; elsif v_t < -s_shc then v_t := -s_shc; end if;
                    s_sh_base <= resize(v_t, 11);
                    -- row slides: (signed byte of each row's key) * boil phase / 128,
                    -- products mod 2^32 so the slide stays seamless as the phase wraps
                    s_bpa <= s_bph;
                    s_brt <= s_rk0(7 downto 0);
                    s_brb <= s_rk1(7 downto 0);
                    s_bacc_t <= (others => '0');
                    s_bacc_b <= (others => '0');
                    s_bi <= (others => '0');
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    -- 8 serial shift-add steps: row slides (signed, mod 2^32)
                    -- and the quake tear (slab offset * gain)
                    if s_brt(0) = '1' then
                        if s_bi = 7 then s_bacc_t <= s_bacc_t - s_bpa; else s_bacc_t <= s_bacc_t + s_bpa; end if;
                    end if;
                    if s_brb(0) = '1' then
                        if s_bi = 7 then s_bacc_b <= s_bacc_b - s_bpa; else s_bacc_b <= s_bacc_b + s_bpa; end if;
                    end if;
                    s_bpa <= shift_left(s_bpa, 1);
                    s_brt <= shift_right(s_brt, 1);
                    s_brb <= shift_right(s_brb, 1);
                    if s_qb(0) = '1' then s_qacc <= s_qacc + s_qa; end if;
                    s_qa <= shift_left(s_qa, 1);
                    s_qb <= shift_right(s_qb, 1);
                    s_bi <= s_bi + 1;
                    if s_bi = 7 then s_lstep <= s_lstep + 1; end if;
                when 6 =>
                    s_sh_off <= s_sh_base + resize(shift_right(s_qacc, 7), 11);
                    s_xst_t  <= s_xst + s_bacc_t(31 downto 7);
                    s_xst_b  <= s_xst + s_bacc_b(31 downto 7);
                    s_lstep <= (others => '1');
                when others => null;
            end case;

            -- per-frame sequencer: duotone vectors, scale step, drift, boil,
            -- centring offsets, amount and quake gains
            case to_integer(s_vstep) is
                when 0 =>
                    s_sin_addr <= s_k_hue(9 downto 2);
                    s_period <= resize(s_k_period(9 downto 2), 9) + 2;
                    s_skew_v <= s_skg(12 downto 3);
                    s_grain  <= s_sw_grain;
                    g_wm1    <= s_line_width - 1;
                    s_w_edge <= s_line_width - (C_EDGE + 1);
                    s_hw     <= s_line_width(10 downto 1);
                    s_hh     <= s_height(10 downto 1);
                    -- scale index 0..447 = 7 octaves x 64
                    v_s13 := shift_left(resize(s_k_scale, 13), 3) - resize(s_k_scale, 13);
                    s_sidx <= v_s13(12 downto 4);
                    -- two-way drift about the knob centre (deadband +-8);
                    -- abs at full width, then slice (|-512| must stay 512)
                    v_dv := signed(resize(s_k_drift, 11)) - to_signed(512, 11);
                    if v_dv < 0 then v_dn := -v_dv; else v_dn := v_dv; end if;
                    v_da := unsigned(v_dn(9 downto 0));
                    if v_da < 8 then
                        s_dspd <= (others => '0');
                    elsif v_da(9) = '1' then
                        s_dspd <= (others => '1');
                    else
                        s_dspd <= v_da(8 downto 2);
                    end if;
                    if v_dv > 0 then s_dup <= '1'; else s_dup <= '0'; end if;
                    -- quake: the top quarter of the glided slider
                    v_a := s_pg(13 downto 4);
                    if v_a(9 downto 8) = "11" then s_q8 <= v_a(7 downto 0); else s_q8 <= (others => '0'); end if;
                    -- glides (ease by diff/8)
                    v_sd := signed(resize(shift_left(resize(s_k_skew, 13), 3), 15)) - signed(resize(s_skg, 15));
                    v_sd := signed(resize(s_skg, 15)) + shift_right(v_sd, 3);
                    s_skg <= unsigned(v_sd(12 downto 0));
                    v_pd := signed(resize(shift_left(resize(s_p_amount, 14), 4), 16)) - signed(resize(s_pg, 16));
                    v_pd := signed(resize(s_pg, 16)) + shift_right(v_pd, 3);
                    s_pg <= unsigned(v_pd(13 downto 0));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- the band period never drops below one field line
                    if s_period < s_ymin then s_peff <= s_ymin; else s_peff <= s_period; end if;
                    s_vstep <= s_vstep + 1;
                when 2 => s_vstep <= s_vstep + 1;
                when 3 =>
                    s_sB <= s_sin_q2;
                    s_sin_addr <= s_k_hue(9 downto 2) + 64;
                    s_srom_addr <= s_sidx(5 downto 0);
                    s_vstep <= s_vstep + 1;
                when 4 | 5 => s_vstep <= s_vstep + 1;
                when 6 =>
                    s_cB <= s_sin_q2;
                    s_sin_addr <= s_k_hue(9 downto 2) + s_k_spread(9 downto 3);
                    s_vstep <= s_vstep + 1;
                when 7 | 8 => s_vstep <= s_vstep + 1;
                when 9 =>
                    s_sA <= s_sin_q2;
                    s_sin_addr <= s_k_hue(9 downto 2) + s_k_spread(9 downto 3) + 64;
                    s_step <= shift_right(s_srom_q2, to_integer(s_sidx(8 downto 6)));
                    s_vstep <= s_vstep + 1;
                when 10 | 11 => s_vstep <= s_vstep + 1;
                when 12 =>
                    s_cA <= s_sin_q2;
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    -- chroma vectors: 512 + 1.5 * sin (+-382)
                    s_uB <= to_signed(512, 12) + resize(s_sB, 12) + resize(shift_right(s_sB, 1), 12);
                    s_vB <= to_signed(512, 12) + resize(s_cB, 12) + resize(shift_right(s_cB, 1), 12);
                    s_uA <= to_signed(512, 12) + resize(s_sA, 12) + resize(shift_right(s_sA, 1), 12);
                    s_vA <= to_signed(512, 12) + resize(s_cA, 12) + resize(shift_right(s_cA, 1), 12);
                    -- real lattice steps: x per pixel = step x (x step / 2) ...
                    case to_integer(s_vxs) is
                        when 4      => s_xstep <= shift_left(resize(s_step, 17), 1);
                        when 3      => s_xstep <= resize(s_step, 17) + resize(shift_right(s_step, 1), 17);
                        when others => s_xstep <= resize(s_step, 17);
                    end case;
                    s_vstep <= to_unsigned(24, 6);
                when 24 =>
                    -- ... y per field line = step x (y step / 256)
                    s_ma <= resize(s_step, 26);
                    s_mb <= resize(s_ys, 11);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(25, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 25 =>
                    s_ystep <= s_macc(24 downto 8);
                    s_vstep <= to_unsigned(14, 6);
                when 14 =>
                    s_du <= s_uB - s_uA;
                    s_dv <= s_vB - s_vA;
                    -- drift per frame = (speed/16 virtual px) * virtual step
                    s_ma <= resize(s_step, 26);
                    s_mb <= resize(s_dspd, 11);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(16, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 15 =>
                    -- serial shift-add multiplier: macc = ma * mb, then mret
                    if s_mb = 0 then
                        s_vstep <= s_mret;
                    else
                        if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                        s_ma <= shift_left(s_ma, 1);
                        s_mb <= shift_right(s_mb, 1);
                    end if;
                when 16 =>
                    s_dinc <= s_macc(21 downto 4);
                    -- Amount curve: square of the glided slider
                    s_ma <= resize(s_pg(13 downto 4), 26);
                    s_mb <= resize(s_pg(13 downto 4), 11);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(17, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 17 =>
                    v_c := resize(s_macc(19 downto 9), 12) + 24;
                    if v_c > 2047 then s_c <= (others => '1'); else s_c <= v_c(10 downto 0); end if;
                    s_vstep <= s_vstep + 1;
                when 18 =>
                    -- displacement = c * width / 8192 (a quarter of the width at 100%;
                    -- the 1080p width in real px: height-based)
                    s_ma <= resize(s_wv, 26);
                    s_mb <= s_c;
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(19, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 19 =>
                    v_g := resize(s_c(10 downto 2), 10) + 24;
                    if s_grain = '1' then
                        if v_g > 511 then s_m <= (others => '1'); else s_m <= v_g(8 downto 0); end if;
                    else
                        if s_macc(25 downto 22) /= 0 then s_m <= (others => '1'); else s_m <= s_macc(21 downto 13); end if;
                    end if;
                    -- centring: x start = cell 256 - (width/2) * step
                    s_ma <= resize(s_xstep, 26);
                    s_mb <= resize(s_hw, 11);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(20, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 20 =>
                    s_xst <= C_XBIAS - s_macc(24 downto 0);
                    s_ma <= resize(s_ystep, 26);
                    s_mb <= resize(s_hh, 11);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(21, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 21 =>
                    s_yoff <= s_macc;
                    if s_dup = '1' then
                        s_ydrift <= s_ydrift + resize(s_dinc, 26);
                    else
                        s_ydrift <= s_ydrift - resize(s_dinc, 26);
                    end if;
                    -- boil phase: the fastest row slides at the drift speed
                    s_bph <= s_bph + resize(s_dinc, 32);
                    s_vstep <= s_vstep + 1;
                when 22 =>
                    -- y start: the drift phase sits at the screen centre
                    s_yacc <= s_ydrift - s_yoff;
                    -- quake gain = q8 * width / 2048 (1/8 of the width at 100%;
                    -- height-based like the warp)
                    s_ma <= resize(s_wv, 26);
                    s_mb <= resize(s_q8, 11);
                    s_macc <= (others => '0');
                    s_mret <= to_unsigned(23, 6);
                    s_vstep <= to_unsigned(15, 6);
                when 23 =>
                    s_qamp <= s_macc(18 downto 11);
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line   <= s_line + 1;
                s_height <= s_line + 1;      -- captured on active lines
                s_yacc <= s_yacc + resize(s_ystep, 26);   -- lattice y, active lines only
                -- band position in virtual lines (Q.8); at 1080p the old
                -- lp >= P - 1 wrap.  The period is >= the y step, so one
                -- wrap per line at most.
                v_ln := s_lpa + resize(s_ys, 17);
                if v_ln(16 downto 8) >= s_peff then
                    s_lpa <= (v_ln(16 downto 8) - s_peff) & v_ln(7 downto 0);
                    s_dir <= not s_dir;
                    s_band <= not s_band;
                else
                    s_lpa <= v_ln;
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0'; s_line <= (others => '0');
                s_lpa <= (others => '0'); s_dir <= '0'; s_band <= '0';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0)   <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i)   <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Noise + address pipeline (stages 1..17)
    --------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_t, v_b : signed(10 downto 0);
        variable v_ns : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: cell columns (mod 512, and the next one) and Q7
            -- fractions along the cell's top and bottom lattice rows
            s_x1   <= g_cnt;
            s_cxt  <= s_xacc_t(24 downto 16);
            s_cxt1 <= s_xacc_t(24 downto 16) + 1;
            s_fxt  <= s_xacc_t(15 downto 9);
            s_cxb  <= s_xacc_b(24 downto 16);
            s_cxb1 <= s_xacc_b(24 downto 16) + 1;
            s_fxb  <= s_xacc_b(15 downto 9);
            -- stage 2: lattice keys; sheared x
            s_ka <= s_rk0 + resize(s_cxt,  16);
            s_kb <= s_rk0 + resize(s_cxt1, 16);
            s_kc <= s_rk1 + resize(s_cxb,  16);
            s_kd <= s_rk1 + resize(s_cxb1, 16);
            s_xs(2) <= signed(resize(s_x1, 13)) + resize(s_sh_off, 13);
            for i in 3 to 15 loop s_xs(i) <= s_xs(i - 1); end loop;
            if s_sw_blocky = '1' then
                s_fxtd(2) <= (others => '0'); s_fxbd(2) <= (others => '0');
            else
                s_fxtd(2) <= s_fxt; s_fxbd(2) <= s_fxb;
            end if;
            for i in 3 to 7 loop s_fxtd(i) <= s_fxtd(i - 1); s_fxbd(i) <= s_fxbd(i - 1); end loop;
            s_bandd(2) <= s_band_row;
            for i in 3 to 23 loop s_bandd(i) <= s_bandd(i - 1); end loop;
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
            -- stage 8: x lerp products (each row with its own fraction)
            s_pab8 <= s_dab7 * signed('0' & s_fxtd(7));
            s_pcd8 <= s_dcd7 * signed('0' & s_fxbd(7));
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
            -- stage 13: centred
            s_nc13 <= signed('0' & s_n12) - to_signed(128, 9);
            s_nd13 <= s_n12;
            -- stage 14: amount (px for the warp, luma codes for the grain);
            -- noise tone key with the band shade
            s_d14 <= s_nc13 * signed('0' & s_m);
            if s_bandd(13) = '1' and s_sw_flip = '0' then
                v_ns := signed(resize(s_nd13, 10)) + to_signed(C_SHADE, 10);
                if v_ns > 255 then s_nt(14) <= x"FF"; else s_nt(14) <= unsigned(v_ns(7 downto 0)); end if;
            else
                s_nt(14) <= s_nd13;
            end if;
            for i in 15 to 23 loop s_nt(i) <= s_nt(i - 1); end loop;
            -- stage 15: displacement / grain, +-511
            if s_grain = '1' then
                s_off15 <= (others => '0');
                s_gr15  <= s_d14(16 downto 7);
            else
                s_off15 <= s_d14(16 downto 7);
                s_gr15  <= (others => '0');
            end if;
            -- stage 16: source x
            s_u16 <= s_xs(15) + resize(s_off15, 13);
            s_gd(16) <= s_gr15;
            for i in 17 to 21 loop s_gd(i) <= s_gd(i - 1); end loop;
            -- stage 17: address / edge clamp
            s_addr17 <= f_edge_addr(s_u16, s_w_edge);
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

    --------------------------------------------------------------------------
    -- Fetch + duotone (stages 18..26), conditioning (27..30)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y  : signed(12 downto 0);
        variable v_g  : signed(10 downto 0);
        variable v_m  : unsigned(7 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 18 ctx / 19 fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
            else
                s_wet_y <= unsigned(s_rdY0);
            end if;
            -- stage 20: luma above black; output luma with grain, 64 .. 940
            if s_wet_y < 64 then s_yc20 <= (others => '0'); else s_yc20 <= s_wet_y - 64; end if;
            v_y := signed(resize(s_wet_y, 13)) + resize(s_gd(19), 13);
            if v_y > 940 then
                s_yo20 <= to_unsigned(940, 10);
            elsif v_y < 64 then
                s_yo20 <= to_unsigned(64, 10);
            else
                s_yo20 <= unsigned(v_y(9 downto 0));
            end if;
            -- stage 21: stretch x1.156
            s_ys21 <= resize(s_yc20, 12) + resize(shift_right(s_yc20, 3), 12) + resize(shift_right(s_yc20, 5), 12);
            s_yo21 <= s_yo20;
            -- stage 22: 8-bit key; the key's grain (a quarter of the luma
            -- grain) plus the band shade
            if s_ys21 > 1023 then s_y8_22 <= x"FF"; else s_y8_22 <= s_ys21(9 downto 2); end if;
            if s_bandd(21) = '1' and s_sw_flip = '0' then
                s_gq22 <= resize(shift_right(s_gd(21), 2), 9) + to_signed(C_SHADE, 9);
            else
                s_gq22 <= resize(shift_right(s_gd(21), 2), 9);
            end if;
            s_yo22 <= s_yo21;
            -- stage 23: grain + shade into the key
            v_g := signed(resize(s_y8_22, 11)) + resize(s_gq22, 11);
            if v_g < 0 then s_y8g23 <= (others => '0'); elsif v_g > 255 then s_y8g23 <= x"FF"; else s_y8g23 <= unsigned(v_g(7 downto 0)); end if;
            s_yo23 <= s_yo22;
            -- stage 24: tone key select + band flip
            if (s_bandd(23) and s_sw_flip) = '1' then v_m := x"FF"; else v_m := x"00"; end if;
            if s_sw_tonenoise = '1' then
                s_t24 <= s_nt(23) xor v_m;
            else
                s_t24 <= s_y8g23 xor v_m;
            end if;
            s_yo24 <= s_yo23;
            -- stage 25: chroma lerp products
            s_pu25 <= s_du * signed('0' & s_t24);
            s_pv25 <= s_dv * signed('0' & s_t24);
            s_yo25 <= s_yo24;
            -- stage 26: chroma
            s_u26 <= f_clamp10(resize(s_uA, 13) + resize(shift_right(s_pu25, 8), 13));
            s_v26 <= f_clamp10(resize(s_vA, 13) + resize(shift_right(s_pv25, 8), 13));
            s_y26 <= s_yo25;
            -- stage 27: neutral outside the generated avid (the smoothing's
            -- edge neighbours)
            if s_gav_sr(25) = '1' then
                s_y27 <= s_y26; s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
            -- stages 28..29: horizontal [1 2 1]/4 on the pixel one back (g1
            -- the centre, g2 and stage 27 its neighbours)
            s_g1y <= s_y27;  s_g2y <= s_g1y;
            s_g1u <= s_u27;  s_g2u <= s_g1u;
            s_g1v <= s_v27;  s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y27, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u27, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v27, 12);
            s_y29 <= v_sy(11 downto 2);
            s_u29 <= v_su(11 downto 2);
            s_v29 <= v_sv(11 downto 2);
            -- stage 30: luma scaled toward black, 64 + 13/16 (y - 64) (white
            -- ~775, under the drawn-luma cap); neutral outside the avid
            v_d  := s_y29 - 64;
            v_yo := s_y29 - shift_right(v_d, 3) - shift_right(v_d, 4);
            if s_gav_sr(28) = '1' then
                s_y30 <= v_yo; s_u30 <= s_u29; s_v30 <= s_v29;
            else
                s_y30 <= to_unsigned(64, 10); s_u30 <= to_unsigned(512, 10); s_v30 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y30);
    s_io_align_in.u       <= std_logic_vector(s_u30);
    s_io_align_in.v       <= std_logic_vector(s_v30);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_gav_sr(C_LAT - 1);

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

end architecture tremor;
