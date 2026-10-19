-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Hatchglass v0.3.0: catalogue combo #93 (seed 20260924) -- mirror /
-- kaleidoscope + crosshatch + luma / saturation key.  Two knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Ink       stroke colour: black ink at 0%, the picture's own colour
--                (half its brightness, 1.5x its saturation) at 100%
--   K2 Slide     the mirror lattice slides sideways, bipolar; squared speed
--                curve with a centre detent (to 15 px a frame)
--   K3 Weight    stroke weight in the darkest tones (9..69% of the pitch)
--   K4 Pitch     stroke spacing 4..48 px (exponential)
--   K5 Depth     the panes multiply: 2 (a mirror at the centre) to 128
--                (exponential, glided)
--   K6 Soft      key edge softness (16 half-octave steps)
--
--   S7  Key      luma / saturation
--   S8  Invert   engrave the darks (or greys) first instead of the lights
--   S9  Space    screen / glass: Glass mirrors the strokes AND the key with
--                the panes (chevrons at every seam, a symmetric key); Screen
--                keeps both on the real picture (the subject cuts through)
--   S10 Paper    hatch on white paper / hatch over the mirrored picture
--   S11 Fold     mirror (alternate panes reversed) / repeat (tiles)
--   P12 Engrave  the key level sweeps the picture into hatch: none at 0%,
--                everything at 100% (whatever Soft and Invert are)
--
-- Mirrors: a per-pixel up/down counter bounces the source column u between 0
-- and the pane width P (or wraps, for Repeat), started each line from a state
-- the vblank sequencer works out; with no slide a seam sits at the screen
-- centre.  Panes read w/2 + u, reflected at the inset right edge.
-- Hatch: tonal engraving.  Darkness (940 - y) picks how many of the four
-- stroke directions show (/, \, -, |) and the newest one thickens from 0 to
-- the full weight.  Phases come from modular counters (any pitch) and the
-- true frame line (interlace-aware); Glass strokes take the x phase from the
-- mirror counter.
-- The first row of each field has no row above to replay: it is drawn from
-- black instead of the previous field's stale line.
-- Output: highlight knee (600 + (y - 600)/2, cap 784; paper ~770) and
-- [1 2 1] on Y/U/V (the HD HDMI bar conditioning).  U/V swapped on rows whose
-- hsync->avid parity differs from the row above (each row replays the row
-- above).  Latency 23 clocks, all modes, blanking-gated.
--
-- v0.3.2 (2026-10-17) resolution scaling: the hatch pitch is a size on the
-- 1080-line canvas: K4's 4..48 px at 1080 becomes round(2/3 x) at 720p and
-- round(1/2 x) at SD (at least 3 px), so the strokes keep their spacing
-- relative to the screen height; the stroke weight follows the pitch (a
-- 1-px stroke stays 1 px).  Interlace (line phase +2 per field line) now
-- comes from the timing ID: field_n is tied high on hardware, so the old
-- field_n test never fired and SD strokes were 2:1 stretched.  The Slide
-- speed is in canvas units (x 2/3 at 720p, 1/2 at SD).  The panes stay a
-- count (fractions of the width).  1080p unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture hatchglass of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (23 + 1 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT   : integer := 23;
    constant C_EDGE  : integer := 12;
    constant C_KNEE  : integer := 600;
    constant C_YMAX  : integer := 784;
    constant C_INK   : integer := 64;
    constant C_PAPER : integer := 940;
    constant C_IDLE  : integer := 31;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    function f_clamp255(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        else
            return unsigned(v(7 downto 0));
        end if;
    end function;

    -- |c - 512|
    function f_dist(c : unsigned(9 downto 0)) return unsigned is
    begin
        if c >= 512 then
            return c - 512;
        else
            return 512 - c;
        end if;
    end function;

    -- saturation key |u| + |v|, capped
    function f_satkey(a, b : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(10 downto 0);
    begin
        v := resize(a, 11) + resize(b, 11);
        if v > 1023 then
            return to_unsigned(1023, 10);
        else
            return v(9 downto 0);
        end if;
    end function;

    -- key weight: kv (= kd x4 or x3) shifted by j (x8 .. /16), clipped 0..255
    function f_gain(kv : signed(14 downto 0); j : unsigned(2 downto 0)) return unsigned is
        variable v : signed(18 downto 0);
    begin
        case to_integer(j) is
            when 0      => v := shift_left(resize(kv, 19), 3);
            when 1      => v := shift_left(resize(kv, 19), 2);
            when 2      => v := shift_left(resize(kv, 19), 1);
            when 3      => v := resize(kv, 19);
            when 4      => v := resize(shift_right(kv, 1), 19);
            when 5      => v := resize(shift_right(kv, 2), 19);
            when 6      => v := resize(shift_right(kv, 3), 19);
            when others => v := resize(shift_right(kv, 4), 19);
        end case;
        return f_clamp255(v);
    end function;

    -- glide: move 1/8 of the gap to the target (10-bit knob, 4 fraction bits)
    function f_glide(g : unsigned(13 downto 0); t : unsigned(9 downto 0)) return unsigned is
        variable v_g, v_d : signed(15 downto 0);
    begin
        v_g := signed(resize(g, 16));
        v_d := signed(shift_left(resize(t, 16), 4)) - v_g;
        v_g := v_g + shift_right(v_d, 3);
        return unsigned(v_g(13 downto 0));
    end function;

    -- Depth mantissa: round(256 * 2^(-i/32))
    function f_mant(i : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when  0 => return to_unsigned(256, 10);
            when  1 => return to_unsigned(251, 10);
            when  2 => return to_unsigned(245, 10);
            when  3 => return to_unsigned(240, 10);
            when  4 => return to_unsigned(235, 10);
            when  5 => return to_unsigned(230, 10);
            when  6 => return to_unsigned(225, 10);
            when  7 => return to_unsigned(220, 10);
            when  8 => return to_unsigned(215, 10);
            when  9 => return to_unsigned(211, 10);
            when 10 => return to_unsigned(206, 10);
            when 11 => return to_unsigned(202, 10);
            when 12 => return to_unsigned(197, 10);
            when 13 => return to_unsigned(193, 10);
            when 14 => return to_unsigned(189, 10);
            when 15 => return to_unsigned(185, 10);
            when 16 => return to_unsigned(181, 10);
            when 17 => return to_unsigned(177, 10);
            when 18 => return to_unsigned(173, 10);
            when 19 => return to_unsigned(170, 10);
            when 20 => return to_unsigned(166, 10);
            when 21 => return to_unsigned(162, 10);
            when 22 => return to_unsigned(159, 10);
            when 23 => return to_unsigned(156, 10);
            when 24 => return to_unsigned(152, 10);
            when 25 => return to_unsigned(149, 10);
            when 26 => return to_unsigned(146, 10);
            when 27 => return to_unsigned(143, 10);
            when 28 => return to_unsigned(140, 10);
            when 29 => return to_unsigned(137, 10);
            when 30 => return to_unsigned(134, 10);
            when others => return to_unsigned(131, 10);
        end case;
    end function;

    -- K4 Pitch: round(4 * 12^(i/31)), 4..48 px
    function f_pitch(i : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when  0 => return to_unsigned( 4, 6);
            when  1 => return to_unsigned( 4, 6);
            when  2 => return to_unsigned( 5, 6);
            when  3 => return to_unsigned( 5, 6);
            when  4 => return to_unsigned( 6, 6);
            when  5 => return to_unsigned( 6, 6);
            when  6 => return to_unsigned( 6, 6);
            when  7 => return to_unsigned( 7, 6);
            when  8 => return to_unsigned( 8, 6);
            when  9 => return to_unsigned( 8, 6);
            when 10 => return to_unsigned( 9, 6);
            when 11 => return to_unsigned(10, 6);
            when 12 => return to_unsigned(10, 6);
            when 13 => return to_unsigned(11, 6);
            when 14 => return to_unsigned(12, 6);
            when 15 => return to_unsigned(13, 6);
            when 16 => return to_unsigned(14, 6);
            when 17 => return to_unsigned(16, 6);
            when 18 => return to_unsigned(17, 6);
            when 19 => return to_unsigned(18, 6);
            when 20 => return to_unsigned(20, 6);
            when 21 => return to_unsigned(22, 6);
            when 22 => return to_unsigned(23, 6);
            when 23 => return to_unsigned(25, 6);
            when 24 => return to_unsigned(27, 6);
            when 25 => return to_unsigned(30, 6);
            when 26 => return to_unsigned(32, 6);
            when 27 => return to_unsigned(35, 6);
            when 28 => return to_unsigned(38, 6);
            when 29 => return to_unsigned(41, 6);
            when 30 => return to_unsigned(44, 6);
            when others => return to_unsigned(48, 6);
        end case;
    end function;

    -- Resolution scaling: Pitch at 720p, round(2/3 x f_pitch), at least 3 px
    function f_pitch_720(i : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when  0 => return to_unsigned( 3, 6);
            when  1 => return to_unsigned( 3, 6);
            when  2 => return to_unsigned( 3, 6);
            when  3 => return to_unsigned( 3, 6);
            when  4 => return to_unsigned( 4, 6);
            when  5 => return to_unsigned( 4, 6);
            when  6 => return to_unsigned( 4, 6);
            when  7 => return to_unsigned( 5, 6);
            when  8 => return to_unsigned( 5, 6);
            when  9 => return to_unsigned( 5, 6);
            when 10 => return to_unsigned( 6, 6);
            when 11 => return to_unsigned( 7, 6);
            when 12 => return to_unsigned( 7, 6);
            when 13 => return to_unsigned( 7, 6);
            when 14 => return to_unsigned( 8, 6);
            when 15 => return to_unsigned( 9, 6);
            when 16 => return to_unsigned( 9, 6);
            when 17 => return to_unsigned(11, 6);
            when 18 => return to_unsigned(11, 6);
            when 19 => return to_unsigned(12, 6);
            when 20 => return to_unsigned(13, 6);
            when 21 => return to_unsigned(15, 6);
            when 22 => return to_unsigned(15, 6);
            when 23 => return to_unsigned(17, 6);
            when 24 => return to_unsigned(18, 6);
            when 25 => return to_unsigned(20, 6);
            when 26 => return to_unsigned(21, 6);
            when 27 => return to_unsigned(23, 6);
            when 28 => return to_unsigned(25, 6);
            when 29 => return to_unsigned(27, 6);
            when 30 => return to_unsigned(29, 6);
            when others => return to_unsigned(32, 6);
        end case;
    end function;

    -- Resolution scaling: Pitch at SD, round(1/2 x f_pitch), at least 3 px
    function f_pitch_sd(i : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when  0 => return to_unsigned( 3, 6);
            when  1 => return to_unsigned( 3, 6);
            when  2 => return to_unsigned( 3, 6);
            when  3 => return to_unsigned( 3, 6);
            when  4 => return to_unsigned( 3, 6);
            when  5 => return to_unsigned( 3, 6);
            when  6 => return to_unsigned( 3, 6);
            when  7 => return to_unsigned( 4, 6);
            when  8 => return to_unsigned( 4, 6);
            when  9 => return to_unsigned( 4, 6);
            when 10 => return to_unsigned( 5, 6);
            when 11 => return to_unsigned( 5, 6);
            when 12 => return to_unsigned( 5, 6);
            when 13 => return to_unsigned( 6, 6);
            when 14 => return to_unsigned( 6, 6);
            when 15 => return to_unsigned( 7, 6);
            when 16 => return to_unsigned( 7, 6);
            when 17 => return to_unsigned( 8, 6);
            when 18 => return to_unsigned( 9, 6);
            when 19 => return to_unsigned( 9, 6);
            when 20 => return to_unsigned(10, 6);
            when 21 => return to_unsigned(11, 6);
            when 22 => return to_unsigned(12, 6);
            when 23 => return to_unsigned(13, 6);
            when 24 => return to_unsigned(14, 6);
            when 25 => return to_unsigned(15, 6);
            when 26 => return to_unsigned(16, 6);
            when 27 => return to_unsigned(18, 6);
            when 28 => return to_unsigned(19, 6);
            when 29 => return to_unsigned(21, 6);
            when 30 => return to_unsigned(22, 6);
            when others => return to_unsigned(24, 6);
        end case;
    end function;

    -- key ramp width 256 / gain for each Soft step (8 * 2^(i/2); odd = x4/3)
    function f_ramp(i : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when  0 => return to_unsigned(   8, 11);
            when  1 => return to_unsigned(  11, 11);
            when  2 => return to_unsigned(  16, 11);
            when  3 => return to_unsigned(  21, 11);
            when  4 => return to_unsigned(  32, 11);
            when  5 => return to_unsigned(  43, 11);
            when  6 => return to_unsigned(  64, 11);
            when  7 => return to_unsigned(  85, 11);
            when  8 => return to_unsigned( 128, 11);
            when  9 => return to_unsigned( 171, 11);
            when 10 => return to_unsigned( 256, 11);
            when 11 => return to_unsigned( 341, 11);
            when 12 => return to_unsigned( 512, 11);
            when 13 => return to_unsigned( 683, 11);
            when 14 => return to_unsigned(1024, 11);
            when others => return to_unsigned(1365, 11);
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Line buffers (Y full rate, U/V half rate, two banks)
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    -- A bank is only read for display while the other one is written; the
    -- written bank's read data is discarded, so no read needs the address
    -- being written.
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    --------------------------------------------------------------------------
    -- Controls
    --------------------------------------------------------------------------
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_k12 : unsigned(9 downto 0);
    signal s_sw_sat, s_sw_inv, s_sw_glass, s_sw_over, s_sw_rep : std_logic := '0';
    -- deadbanded knobs: 0 Ink, 1 Weight, 2 Pitch, 3 Depth, 4 Soft, 5 Engrave
    type t_k is array (0 to 5) of unsigned(9 downto 0);
    type t_kd is array (0 to 5) of signed(10 downto 0);
    signal s_kl : t_k := (others => (others => '0'));
    signal s_kh : t_k := (to_unsigned(0, 10), to_unsigned(400, 10), to_unsigned(300, 10),
                          to_unsigned(340, 10), to_unsigned(300, 10), to_unsigned(400, 10));
    signal s_kdiff : t_kd := (others => (others => '0'));
    signal s_gd : unsigned(13 downto 0) := to_unsigned(340 * 16, 14);   -- glided Depth
    -- per-frame constants
    signal s_sat_f, s_inv_f : std_logic := '0';
    signal s_rep    : std_logic := '0';
    signal s_lvl    : signed(12 downto 0) := to_signed(578, 13);
    signal s_sidx   : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_R      : unsigned(10 downto 0) := to_unsigned(32, 11);
    signal s_span   : unsigned(11 downto 0) := to_unsigned(924, 12);
    signal s_e10    : unsigned(10 downto 0) := to_unsigned(400, 11);
    signal s_pit    : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_pitm1  : unsigned(5 downto 0) := to_unsigned(7, 6);
    signal s_f64    : unsigned(5 downto 0) := to_unsigned(21, 6);
    signal s_W4     : unsigned(7 downto 0) := to_unsigned(10, 8);
    signal s_t64    : unsigned(6 downto 0) := (others => '0');          -- ink 0..64
    signal s_t96    : unsigned(6 downto 0) := (others => '0');          -- ink x1.5

    --------------------------------------------------------------------------
    -- Per-frame sequencer (one small op per state; one serial multiplier)
    --------------------------------------------------------------------------
    signal s_vstep  : unsigned(4 downto 0) := to_unsigned(C_IDLE, 5);
    signal s_ret    : unsigned(4 downto 0) := (others => '0');
    signal s_ma     : unsigned(22 downto 0) := (others => '0');
    signal s_mb     : unsigned(10 downto 0) := (others => '0');
    signal s_acc    : unsigned(22 downto 0) := (others => '0');
    signal s_wmax_run : unsigned(10 downto 0) := (others => '0');
    signal s_wf     : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_cx     : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_hi     : unsigned(10 downto 0) := to_unsigned(467, 11);    -- wf - 1 - C_EDGE
    signal s_hi2    : unsigned(11 downto 0) := to_unsigned(934, 12);
    signal s_d3     : unsigned(9 downto 0) := (others => '0');
    signal s_oct    : unsigned(2 downto 0) := (others => '0');
    signal s_P      : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_Pend   : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_L      : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_L16    : unsigned(14 downto 0) := to_unsigned(7680, 15);
    signal s_sds    : signed(10 downto 0) := (others => '0');
    signal s_asd    : unsigned(8 downto 0) := (others => '0');
    signal s_s16, s_t16 : signed(16 downto 0) := (others => '0');       -- slide, 1/16 px, mod L
    signal s_v      : signed(12 downto 0) := (others => '0');
    signal s_u0     : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_dir0   : std_logic := '1';
    signal s_uct    : unsigned(9 downto 0) := (others => '0');
    signal s_uc0    : unsigned(5 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Input tracking, write port
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_bcnt    : unsigned(9 downto 0) := (others => '1');          -- blank clocks, saturating
    signal s_wpar, s_lw : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    -- first row of the field; interlace (field_n toggles) and bottom field
    signal s_first, s_row0 : std_logic := '0';
    signal s_fld_prev : std_logic := '1';
    signal s_ilace, s_bot : std_logic := '0';
    -- Resolution scaling (v0.3.2): mode class from the timing ID
    signal s_c720, s_csd, s_cil : std_logic := '0';
    signal s_spdm : unsigned(7 downto 0) := (others => '0');            -- slide speed magnitude
    signal s_spdn : std_logic := '0';
    signal s_lc      : unsigned(5 downto 0) := (others => '0');          -- field line mod pitch (x2 interlaced)
    signal s_plf     : unsigned(5 downto 0) := (others => '0');          -- frame line mod pitch
    signal s_wy1     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wu1, s_wv1 : unsigned(7 downto 0) := x"80";
    signal s_iu1, s_iv1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_wx1     : unsigned(10 downto 0) := (others => '0');
    signal s_we1, s_wp1 : std_logic := '0';

    --------------------------------------------------------------------------
    -- Mirror counter (S1) and read address (S2..S4).  Sn = n clocks after
    -- data_in; s_avid_sr(n - 1) is the avid of stage n.
    --------------------------------------------------------------------------
    signal s_u       : unsigned(9 downto 0) := (others => '0');
    signal s_dir     : std_logic := '1';
    signal s_uc      : unsigned(5 downto 0) := (others => '0');
    signal s_src2, s_sr3, s_rh3 : signed(12 downto 0) := (others => '0');
    signal s_hi3     : std_logic := '0';
    signal s_addr4   : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Keys: screen (dry pixel, S2..S7) and glass (mirrored pixel, S7..S12),
    -- both carried to S14
    --------------------------------------------------------------------------
    signal s_au2, s_av2, s_y2 : unsigned(9 downto 0) := (others => '0');
    signal s_key3    : unsigned(9 downto 0) := (others => '0');
    signal s_kd4     : signed(12 downto 0) := (others => '0');
    signal s_kv5     : signed(14 downto 0) := (others => '0');
    signal s_w6      : unsigned(7 downto 0) := (others => '0');
    type t_wd is array (7 to 14) of unsigned(8 downto 0);
    signal s_wd      : t_wd := (others => (others => '0'));
    signal s_gau7, s_gav7, s_gy7 : unsigned(9 downto 0) := (others => '0');
    signal s_gkey8   : unsigned(9 downto 0) := (others => '0');
    signal s_gkd9    : signed(12 downto 0) := (others => '0');
    signal s_gkv10   : signed(14 downto 0) := (others => '0');
    signal s_gw11    : unsigned(7 downto 0) := (others => '0');
    type t_gwd is array (12 to 14) of unsigned(8 downto 0);
    signal s_gwd     : t_gwd := (others => (others => '0'));

    --------------------------------------------------------------------------
    -- Wet picture (S6), ink (S7..S13), hatch (S7..S13), blend (S14..S18),
    -- output (S19..S23)
    --------------------------------------------------------------------------
    type t_pc is array (7 to 17) of unsigned(9 downto 0);
    signal s_py, s_pu, s_pv : t_pc := (others => (others => '0'));
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := (others => '0');
    -- ink
    signal s_idy7    : unsigned(9 downto 0) := (others => '0');
    signal s_idu7, s_idv7 : signed(10 downto 0) := (others => '0');
    signal s_iyh8    : unsigned(13 downto 0) := (others => '0');
    signal s_iyl8    : unsigned(12 downto 0) := (others => '0');
    signal s_iuh8, s_ivh8 : signed(15 downto 0) := (others => '0');
    signal s_iul8, s_ivl8 : signed(14 downto 0) := (others => '0');
    signal s_iy9     : unsigned(16 downto 0) := (others => '0');
    signal s_iu9, s_iv9 : signed(17 downto 0) := (others => '0');
    type t_ik is array (10 to 13) of unsigned(9 downto 0);
    signal s_iky     : t_ik := (others => to_unsigned(C_INK, 10));
    signal s_iku, s_ikv : t_ik := (others => to_unsigned(512, 10));
    -- hatch
    signal s_dk7     : signed(10 downto 0) := (others => '0');
    signal s_D8      : unsigned(9 downto 0) := (others => '0');
    signal s_D9      : unsigned(7 downto 0) := (others => '0');
    signal s_tp10    : unsigned(7 downto 0) := (others => '0');
    signal s_L10     : unsigned(1 downto 0) := (others => '0');
    type t_th is array (0 to 3) of unsigned(7 downto 0);
    signal s_th11, s_th12 : t_th := (others => (others => '0'));
    -- stroke phases
    type t_ucd is array (2 to 10) of unsigned(5 downto 0);
    signal s_ucd     : t_ucd := (others => (others => '0'));
    signal s_xs10    : unsigned(5 downto 0) := (others => '0');
    signal s_pl10    : unsigned(5 downto 0) := (others => '0');
    signal s_pa11    : unsigned(6 downto 0) := (others => '0');
    signal s_pb11    : signed(6 downto 0) := (others => '0');
    signal s_pc11, s_pd11 : unsigned(5 downto 0) := (others => '0');
    signal s_pa12, s_pb12, s_pc12, s_pd12 : unsigned(5 downto 0) := (others => '0');
    signal s_hatch13 : std_logic := '0';
    -- blend and output
    signal s_hy14, s_hu14, s_hv14 : unsigned(9 downto 0) := (others => '0');
    signal s_dy15, s_du15, s_dv15 : signed(10 downto 0) := (others => '0');
    signal s_w15     : unsigned(8 downto 0) := (others => '0');
    signal s_phy16, s_phu16, s_phv16 : signed(16 downto 0) := (others => '0');
    signal s_ply16, s_plu16, s_plv16 : signed(15 downto 0) := (others => '0');
    signal s_pry17, s_pru17, s_prv17 : signed(20 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_yk19    : signed(10 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    signal s_y20     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23     : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k1  <= unsigned(registers_in(0));
    s_k2  <= unsigned(registers_in(1));
    s_k3  <= unsigned(registers_in(2));
    s_k4  <= unsigned(registers_in(3));
    s_k5  <= unsigned(registers_in(4));
    s_k6  <= unsigned(registers_in(5));
    s_k12 <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Tracking, write port, per-frame sequencer, sync delay
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_abs : signed(10 downto 0);
        variable v_p   : unsigned(9 downto 0);
        variable v_off : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_sw_sat   <= registers_in(6)(0);
            s_sw_inv   <= registers_in(6)(1);
            s_sw_glass <= registers_in(6)(2);
            s_sw_over  <= registers_in(6)(3);
            s_sw_rep   <= registers_in(6)(4);
            -- Resolution scaling: mode class (static while a mode runs)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8      => s_csd <= '1'; s_c720 <= '0'; s_cil <= '1';   -- 480i, 576i
                when 4 | 12     => s_csd <= '1'; s_c720 <= '0'; s_cil <= '0';   -- 480p, 576p
                when 5 | 6 | 14 => s_csd <= '0'; s_c720 <= '1'; s_cil <= '0';   -- 720p
                when 1 | 2 | 10 => s_csd <= '0'; s_c720 <= '0'; s_cil <= '1';   -- 1080i
                when others     => s_csd <= '0'; s_c720 <= '0'; s_cil <= '0';   -- 1080p
            end case;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
                s_bcnt <= (others => '0');
            else
                s_rx <= (others => '0');
                if s_bcnt /= 1023 then
                    s_bcnt <= s_bcnt + 1;
                end if;
            end if;

            -- write port: input one clock deep (S1)
            s_wy1 <= unsigned(data_in.y);
            s_wu1 <= unsigned(data_in.u(9 downto 2));
            s_wv1 <= unsigned(data_in.v(9 downto 2));
            s_iu1 <= unsigned(data_in.u);
            s_iv1 <= unsigned(data_in.v);
            s_wx1 <= s_rx;
            s_we1 <= data_in.avid;
            s_wp1 <= s_wpar;

            ------------------------------------------------------------------
            -- Per-frame sequencer, started in the vblank:
            --   knobs latched, deadbanded (3 counts), Depth glided
            --   P   = (w/2) * 2^(-6 * Depth) via mantissa table, at least 2
            --   L   = 2P (mirror) or P (repeat)
            --   key level swept by Engrave across [lo - 8 - ramp, hi + 8]
            --   slide offset (1/16 px) += speed, kept in [0, 16L)
            --   counter start m0 = (slide - w/2) mod L -> u0, dir0, u0 mod pitch
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_kl(0) <= s_k1;
                    s_kl(1) <= s_k3;
                    s_kl(2) <= s_k4;
                    s_kl(3) <= s_k5;
                    s_kl(4) <= s_k6;
                    s_kl(5) <= s_k12;
                    s_sds   <= signed(resize(s_k2, 11)) - 512;
                    s_rep   <= s_sw_rep;
                    s_sat_f <= s_sw_sat;
                    s_inv_f <= s_sw_inv;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    for i in 0 to 5 loop
                        s_kdiff(i) <= signed(resize(s_kl(i), 11)) - signed(resize(s_kh(i), 11));
                    end loop;
                    s_cx <= s_wf(10 downto 1);
                    s_hi <= s_wf - (C_EDGE + 1);
                    v_abs := abs(s_sds);
                    if v_abs > 8 then
                        s_asd <= resize(unsigned(v_abs) - 8, 9);
                    else
                        s_asd <= (others => '0');
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    for i in 0 to 5 loop
                        if s_kdiff(i) > 2 or s_kdiff(i) < -2 then
                            s_kh(i) <= s_kl(i);
                        end if;
                    end loop;
                    s_hi2 <= s_hi & '0';
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_gd   <= f_glide(s_gd, s_kh(3));
                    if s_csd = '1' then
                        s_pit <= f_pitch_sd(s_kh(2)(9 downto 5));
                    elsif s_c720 = '1' then
                        s_pit <= f_pitch_720(s_kh(2)(9 downto 5));
                    else
                        s_pit <= f_pitch(s_kh(2)(9 downto 5));
                    end if;
                    s_f64  <= to_unsigned(6, 6) + s_kh(1)(9 downto 5) + s_kh(1)(9 downto 7);
                    s_sidx <= s_kh(4)(9 downto 6);
                    s_t64  <= resize(s_kh(0)(9 downto 4), 7) + resize(s_kh(0)(9 downto 9), 7);
                    s_e10  <= resize(s_kh(5), 11) + resize(s_kh(5)(9 downto 9), 11);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    -- Depth * 3/4 -> 6 octaves over the travel
                    s_d3  <= resize(s_gd(13 downto 5), 10) + resize(s_gd(13 downto 6), 10);
                    s_pitm1 <= s_pit - 1;
                    s_R   <= f_ramp(s_sidx);
                    s_t96 <= s_t64 + shift_right(s_t64, 1);
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    -- sweep span (hi - lo) + 16 + ramp: luma 64..940, sat 0..640
                    if s_sat_f = '1' then
                        s_span <= to_unsigned(656, 12) + resize(s_R, 12);
                    else
                        s_span <= to_unsigned(892, 12) + resize(s_R, 12);
                    end if;
                    s_oct <= s_d3(9 downto 7);
                    s_ma  <= resize(s_cx, 23);
                    s_mb  <= resize(f_mant(s_d3(6 downto 2)), 11);
                    s_acc <= (others => '0');
                    s_ret <= to_unsigned(7, 5);
                    s_vstep <= to_unsigned(6, 5);
                when 6 =>
                    -- serial shift-add multiply: acc += ma * mb, then return
                    if s_mb = 0 then
                        s_vstep <= s_ret;
                    else
                        if s_mb(0) = '1' then
                            s_acc <= s_acc + s_ma;
                        end if;
                        s_ma <= shift_left(s_ma, 1);
                        s_mb <= shift_right(s_mb, 1);
                    end if;
                when 7 =>
                    -- pane width P = cx * mant >> (8 + oct), at least 2
                    case to_integer(s_oct) is
                        when 0      => v_p := s_acc(17 downto 8);
                        when 1      => v_p := s_acc(18 downto 9);
                        when 2      => v_p := s_acc(19 downto 10);
                        when 3      => v_p := s_acc(20 downto 11);
                        when 4      => v_p := resize(s_acc(20 downto 12), 10);
                        when others => v_p := resize(s_acc(20 downto 13), 10);
                    end case;
                    if v_p < 2 then
                        v_p := to_unsigned(2, 10);
                    end if;
                    s_P <= v_p;
                    -- Engrave: offset = span * e / 1024
                    s_ma  <= resize(s_span, 23);
                    s_mb  <= s_e10;
                    s_acc <= (others => '0');
                    s_ret <= to_unsigned(8, 5);
                    s_vstep <= to_unsigned(6, 5);
                when 8 =>
                    if s_rep = '1' then
                        s_L    <= resize(s_P, 11);
                        s_Pend <= s_P - 1;
                    else
                        s_L    <= s_P & '0';
                        s_Pend <= s_P;
                    end if;
                    -- key level: lights first falls from hi + 8; darks first
                    -- rises from lo - 8; 100% passes the far end by the ramp
                    v_off := signed(resize(s_acc(21 downto 10), 13));
                    if s_inv_f = '1' then
                        if s_sat_f = '1' then
                            s_lvl <= v_off - 8;
                        else
                            s_lvl <= v_off + (C_INK - 8);
                        end if;
                    else
                        if s_sat_f = '1' then
                            s_lvl <= to_signed(640 + 8, 13) - v_off;
                        else
                            s_lvl <= to_signed(C_PAPER + 8, 13) - v_off;
                        end if;
                    end if;
                    -- slide speed = (|K2 - 512| - 8)^2 / 1024 in 1/16 px a frame
                    s_ma  <= resize(s_asd, 23);
                    s_mb  <= resize(s_asd, 11);
                    s_acc <= (others => '0');
                    s_ret <= to_unsigned(9, 5);
                    s_vstep <= to_unsigned(6, 5);
                when 9 =>
                    -- slide speed in canvas units: x 1/2 at SD, ~2/3 at 720p
                    if s_csd = '1' then
                        s_spdm <= shift_right(s_acc(17 downto 10), 1);
                    elsif s_c720 = '1' then
                        s_spdm <= shift_right(s_acc(17 downto 10), 1) + shift_right(s_acc(17 downto 10), 3)
                                  + shift_right(s_acc(17 downto 10), 5);
                    else
                        s_spdm <= s_acc(17 downto 10);
                    end if;
                    if s_sds < 0 then s_spdn <= '1'; else s_spdn <= '0'; end if;
                    s_L16 <= s_L & "0000";
                    -- full stroke weight in quarter px: pitch * f64 / 16
                    s_ma  <= resize(s_pit, 23);
                    s_mb  <= resize(s_f64, 11);
                    s_acc <= (others => '0');
                    s_ret <= to_unsigned(10, 5);
                    s_vstep <= to_unsigned(6, 5);
                when 10 =>
                    s_W4  <= s_acc(11 downto 4);
                    if s_spdn = '1' then
                        s_s16 <= s_s16 - signed(resize(s_spdm, 17));
                    else
                        s_s16 <= s_s16 + signed(resize(s_spdm, 17));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    if s_s16 < 0 then
                        s_s16 <= s_s16 + signed(resize(s_L16, 17));
                    else
                        s_vstep <= s_vstep + 1;
                    end if;
                when 12 =>
                    s_t16 <= s_s16 - signed(resize(s_L16, 17));
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    if s_t16 >= 0 then
                        s_s16 <= s_t16;
                        s_vstep <= to_unsigned(12, 5);
                    else
                        s_vstep <= s_vstep + 1;
                    end if;
                when 14 =>
                    s_v <= signed(resize(unsigned(s_s16(15 downto 4)), 13)) - signed(resize(s_cx, 13));
                    s_vstep <= s_vstep + 1;
                when 15 =>
                    if s_v < 0 then
                        s_v <= s_v + signed(resize(s_L, 13));
                    else
                        s_vstep <= s_vstep + 1;
                    end if;
                when 16 =>
                    -- m0 = v in [0, L); the mirror runs back down above P
                    if s_v > signed(resize(s_P, 13)) then
                        s_u0   <= resize(unsigned(signed(resize(s_L, 13)) - s_v), 10);
                        s_dir0 <= '0';
                    else
                        s_u0   <= unsigned(s_v(9 downto 0));
                        s_dir0 <= '1';
                    end if;
                    s_vstep <= s_vstep + 1;
                when 17 =>
                    s_uct <= s_u0;
                    s_vstep <= s_vstep + 1;
                when 18 =>
                    if s_uct >= resize(s_pit, 10) then
                        s_uct <= s_uct - resize(s_pit, 10);
                    else
                        s_uc0 <= s_uct(5 downto 0);
                        s_vstep <= to_unsigned(C_IDLE, 5);
                    end if;
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Line / field events
            ------------------------------------------------------------------
            s_hpar <= not s_hpar;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar;
            end if;
            if s_prev_avid = '0' and data_in.avid = '1' then
                -- this line writes bank s_wpar and reads the other (the line above)
                s_lw <= s_wpar;
                s_line_par <= s_hpar;
                if s_wpar = '1' then
                    s_cap_par1 <= s_hpar;
                else
                    s_cap_par0 <= s_hpar;
                end if;
                -- the field's first row has no row above.  Interlace from the
                -- timing ID (field_n is tied high on hardware, so a field_n
                -- toggle test never fired); bottom field = field_n '0'
                s_row0 <= s_first;
                if s_first = '1' then
                    s_first    <= '0';
                    s_fld_prev <= data_in.field_n;
                    s_ilace    <= s_cil;
                    s_bot      <= s_cil and not data_in.field_n;
                end if;
            end if;
            if s_lw = '1' then
                s_rowswap <= s_line_par xor s_cap_par0;
            else
                s_rowswap <= s_line_par xor s_cap_par1;
            end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                if s_rx > s_wmax_run then
                    s_wmax_run <= s_rx;
                end if;
                -- next field line: +1 frame line, or +2 when interlaced
                if s_ilace = '1' then
                    if s_lc + 2 >= s_pit then
                        s_lc <= s_lc + 2 - s_pit;
                    else
                        s_lc <= s_lc + 2;
                    end if;
                elsif s_lc >= s_pitm1 then
                    s_lc <= (others => '0');
                else
                    s_lc <= s_lc + 1;
                end if;
            end if;
            -- frame line mod pitch: the bottom field sits one line down
            if s_bot = '1' then
                if s_lc >= s_pitm1 then
                    s_plf <= (others => '0');
                else
                    s_plf <= s_lc + 1;
                end if;
            else
                s_plf <= s_lc;
            end if;
            -- field event: no avid for 512 clocks (hblank is 138..370)
            if s_bcnt = 511 and s_saw_active = '1' then
                s_saw_active <= '0';
                s_first      <= '1';
                s_wf         <= s_wmax_run;      -- widest line of the field
                s_wmax_run   <= (others => '0');
                s_lc         <= (others => '0');
                s_vstep      <= (others => '0');
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
    -- Mirror counter, read address, screen key
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            -- S1: u bounces between 0 and P (mirror) or wraps at P (repeat);
            -- uc = u mod pitch follows it for the Glass strokes
            if s_avid_sr(0) = '0' then
                s_u   <= s_u0;
                s_dir <= s_dir0;
                s_uc  <= s_uc0;
            elsif s_dir = '1' then
                if s_u = s_Pend then
                    if s_rep = '1' then
                        s_u  <= (others => '0');
                        s_uc <= (others => '0');
                    else
                        s_u   <= s_u - 1;
                        s_dir <= '0';
                        if s_uc = 0 then s_uc <= s_pitm1; else s_uc <= s_uc - 1; end if;
                    end if;
                else
                    s_u <= s_u + 1;
                    if s_uc >= s_pitm1 then s_uc <= (others => '0'); else s_uc <= s_uc + 1; end if;
                end if;
            else
                if s_u = 0 then
                    s_u   <= to_unsigned(1, 10);
                    s_dir <= '1';
                    if s_uc >= s_pitm1 then s_uc <= (others => '0'); else s_uc <= s_uc + 1; end if;
                else
                    s_u <= s_u - 1;
                    if s_uc = 0 then s_uc <= s_pitm1; else s_uc <= s_uc - 1; end if;
                end if;
            end if;

            -- S2: source column (panes start at the picture centre)
            s_src2 <= signed(resize(s_cx, 13)) + signed(resize(s_u, 13));
            -- S3: reflect at the inset right edge (test on the source, in parallel)
            s_sr3 <= s_src2;
            if s_src2 > signed(resize(s_hi, 13)) then s_hi3 <= '1'; else s_hi3 <= '0'; end if;
            s_rh3 <= signed(resize(s_hi2, 13)) - s_src2;
            -- S4: read address
            if s_hi3 = '1' then
                s_addr4 <= unsigned(s_rh3(10 downto 0));
            else
                s_addr4 <= unsigned(s_sr3(10 downto 0));
            end if;

            -- screen key (the dry pixel).  S2: |u|, |v|
            s_au2 <= f_dist(s_iu1);
            s_av2 <= f_dist(s_iv1);
            s_y2  <= s_wy1;
            -- S3: luma, or saturation |u| + |v|
            if s_sat_f = '1' then
                s_key3 <= f_satkey(s_au2, s_av2);
            else
                s_key3 <= s_y2;
            end if;
            -- S4: distance past the level
            if s_inv_f = '1' then
                s_kd4 <= s_lvl - signed(resize(s_key3, 13));
            else
                s_kd4 <= signed(resize(s_key3, 13)) - s_lvl;
            end if;
            -- S5..S6: gain 32 * 2^(-idx/2): x4 (even) / x3 (odd), then a shift
            if s_sidx(0) = '1' then
                s_kv5 <= shift_left(resize(s_kd4, 15), 1) + resize(s_kd4, 15);
            else
                s_kv5 <= shift_left(resize(s_kd4, 15), 2);
            end if;
            s_w6 <= f_gain(s_kv5, s_sidx(3 downto 1));
            -- S7: 0..256
            s_wd(7) <= resize(s_w6, 9) + resize(s_w6(7 downto 7), 9);
            for i in 8 to 14 loop
                s_wd(i) <= s_wd(i - 1);
            end loop;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr4));
            if s_we1 = '1' and s_wp1 = '0' then
                lbY0(to_integer(s_wx1)) <= std_logic_vector(s_wy1);
            end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr4));
            if s_we1 = '1' and s_wp1 = '1' then
                lbY1(to_integer(s_wx1)) <= std_logic_vector(s_wy1);
            end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr4(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '0' then
                lbU0(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wu1);
            end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr4(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '1' then
                lbU1(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wu1);
            end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr4(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '0' then
                lbV0(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wv1);
            end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr4(10 downto 1)));
            if s_we1 = '1' and s_wp1 = '1' then
                lbV1(to_integer(s_wx1(10 downto 1))) <= std_logic_vector(s_wv1);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch, glass key, ink, hatch, blend, knee, smoothing, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x  : unsigned(5 downto 0);
        variable v_s  : unsigned(11 downto 0);
        variable v_h  : std_logic;
        variable v_pb : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            -- S6: the bank written by the line above; U/V swapped when its
            -- hsync->avid parity differs from this line's.  The field's first
            -- row has no row above: black.
            if s_row0 = '1' then
                s_y6 <= to_unsigned(C_INK, 10);
                s_u6 <= to_unsigned(512, 10);
                s_v6 <= to_unsigned(512, 10);
            elsif s_lw = '0' then
                s_y6 <= unsigned(s_rdY1);
                if s_rowswap = '1' then
                    s_u6 <= unsigned(s_rdV1) & "00"; s_v6 <= unsigned(s_rdU1) & "00";
                else
                    s_u6 <= unsigned(s_rdU1) & "00"; s_v6 <= unsigned(s_rdV1) & "00";
                end if;
            else
                s_y6 <= unsigned(s_rdY0);
                if s_rowswap = '1' then
                    s_u6 <= unsigned(s_rdV0) & "00"; s_v6 <= unsigned(s_rdU0) & "00";
                else
                    s_u6 <= unsigned(s_rdU0) & "00"; s_v6 <= unsigned(s_rdV0) & "00";
                end if;
            end if;

            -- picture carried S7..S17
            s_py(7) <= s_y6; s_pu(7) <= s_u6; s_pv(7) <= s_v6;
            for i in 8 to 17 loop
                s_py(i) <= s_py(i - 1); s_pu(i) <= s_pu(i - 1); s_pv(i) <= s_pv(i - 1);
            end loop;

            -- glass key (the mirrored pixel), S7..S12, carried to S14
            s_gau7 <= f_dist(s_u6);
            s_gav7 <= f_dist(s_v6);
            s_gy7  <= s_y6;
            if s_sat_f = '1' then
                s_gkey8 <= f_satkey(s_gau7, s_gav7);
            else
                s_gkey8 <= s_gy7;
            end if;
            if s_inv_f = '1' then
                s_gkd9 <= s_lvl - signed(resize(s_gkey8, 13));
            else
                s_gkd9 <= signed(resize(s_gkey8, 13)) - s_lvl;
            end if;
            if s_sidx(0) = '1' then
                s_gkv10 <= shift_left(resize(s_gkd9, 15), 1) + resize(s_gkd9, 15);
            else
                s_gkv10 <= shift_left(resize(s_gkd9, 15), 2);
            end if;
            s_gw11 <= f_gain(s_gkv10, s_sidx(3 downto 1));
            s_gwd(12) <= resize(s_gw11, 9) + resize(s_gw11(7 downto 7), 9);
            s_gwd(13) <= s_gwd(12);
            s_gwd(14) <= s_gwd(13);

            -- ink: black + Ink * half the picture's luma above black, and
            -- Ink * 1.5 its chroma.  S7: offsets; S8: products split on the
            -- amount; S9: recombine; S10: ink colour, carried to S13
            if s_y6 > C_INK then
                s_idy7 <= s_y6 - C_INK;
            else
                s_idy7 <= (others => '0');
            end if;
            s_idu7 <= signed(resize(s_u6, 11)) - 512;
            s_idv7 <= signed(resize(s_v6, 11)) - 512;
            s_iyh8 <= s_idy7 * s_t64(6 downto 3);
            s_iyl8 <= s_idy7 * s_t64(2 downto 0);
            s_iuh8 <= s_idu7 * signed('0' & s_t96(6 downto 3));
            s_iul8 <= s_idu7 * signed('0' & s_t96(2 downto 0));
            s_ivh8 <= s_idv7 * signed('0' & s_t96(6 downto 3));
            s_ivl8 <= s_idv7 * signed('0' & s_t96(2 downto 0));
            s_iy9 <= shift_left(resize(s_iyh8, 17), 3) + resize(s_iyl8, 17);
            s_iu9 <= shift_left(resize(s_iuh8, 18), 3) + resize(s_iul8, 18);
            s_iv9 <= shift_left(resize(s_ivh8, 18), 3) + resize(s_ivl8, 18);
            s_iky(10) <= to_unsigned(C_INK, 10) + resize(shift_right(s_iy9, 7), 10);
            s_iku(10) <= f_clamp10(to_signed(512, 13) + resize(shift_right(s_iu9, 6), 13));
            s_ikv(10) <= f_clamp10(to_signed(512, 13) + resize(shift_right(s_iv9, 6), 13));
            for i in 11 to 13 loop
                s_iky(i) <= s_iky(i - 1); s_iku(i) <= s_iku(i - 1); s_ikv(i) <= s_ikv(i - 1);
            end loop;

            -- S7..S9: darkness 0..876 -> 0..252 (x 0.289)
            s_dk7 <= to_signed(C_PAPER, 11) - signed(resize(s_y6, 11));
            if s_dk7 < 0 then
                s_D8 <= (others => '0');
            elsif s_dk7 > C_PAPER - C_INK then
                s_D8 <= to_unsigned(C_PAPER - C_INK, 10);     -- sub-black reads as black
            else
                s_D8 <= unsigned(s_dk7(9 downto 0));
            end if;
            s_D9 <= resize(shift_right(s_D8, 2) + shift_right(s_D8, 5) + shift_right(s_D8, 7), 8);
            -- S10: layer count and the newest layer's weight (quarter px)
            s_L10  <= s_D9(7 downto 6);
            s_tp10 <= resize(shift_right(s_W4 * s_D9(5 downto 0), 6), 8);
            -- S11: per-direction weights (/, \, -, |)
            for k in 0 to 3 loop
                if to_integer(s_L10) > k then
                    s_th11(k) <= s_W4;
                elsif to_integer(s_L10) = k then
                    s_th11(k) <= s_tp10;
                else
                    s_th11(k) <= (others => '0');
                end if;
            end loop;
            s_th12 <= s_th11;

            -- stroke phases.  S1 glass phase carried to S10; S10 screen x
            s_ucd(2) <= s_uc;
            for i in 3 to 10 loop
                s_ucd(i) <= s_ucd(i - 1);
            end loop;
            if s_avid_sr(9) = '0' then
                s_xs10 <= (others => '0');
                s_pl10 <= s_plf;              -- line phase, held through the line
            elsif s_xs10 >= s_pitm1 then
                s_xs10 <= (others => '0');
            else
                s_xs10 <= s_xs10 + 1;
            end if;
            -- S11: x +- line
            if s_sw_glass = '1' then
                v_x := s_ucd(10);
            else
                v_x := s_xs10;
            end if;
            s_pa11 <= resize(v_x, 7) + resize(s_pl10, 7);
            s_pb11 <= signed(resize(v_x, 7)) - signed(resize(s_pl10, 7));
            s_pc11 <= s_pl10;
            s_pd11 <= v_x;
            -- S12: wrap into [0, pitch)
            if s_pa11 >= resize(s_pit, 7) then
                s_pa12 <= resize(s_pa11 - resize(s_pit, 7), 6);
            else
                s_pa12 <= s_pa11(5 downto 0);
            end if;
            if s_pb11 < 0 then
                v_pb := s_pb11 + signed(resize(s_pit, 7));
                s_pb12 <= unsigned(v_pb(5 downto 0));
            else
                s_pb12 <= unsigned(s_pb11(5 downto 0));
            end if;
            s_pc12 <= s_pc11;
            s_pd12 <= s_pd11;
            -- S13: a stroke where any direction's phase is under its weight
            v_h := '0';
            if (s_pa12 & "00") < s_th12(0) then v_h := '1'; end if;
            if (s_pb12 & "00") < s_th12(1) then v_h := '1'; end if;
            if (s_pc12 & "00") < s_th12(2) then v_h := '1'; end if;
            if (s_pd12 & "00") < s_th12(3) then v_h := '1'; end if;
            s_hatch13 <= v_h;
            -- S14: ink, paper, or the picture under the strokes
            if s_hatch13 = '1' then
                s_hy14 <= s_iky(13); s_hu14 <= s_iku(13); s_hv14 <= s_ikv(13);
            elsif s_sw_over = '1' then
                s_hy14 <= s_py(13); s_hu14 <= s_pu(13); s_hv14 <= s_pv(13);
            else
                s_hy14 <= to_unsigned(C_PAPER, 10); s_hu14 <= to_unsigned(512, 10); s_hv14 <= to_unsigned(512, 10);
            end if;

            -- S15: hatch - picture; key weight from the screen or the glass
            s_dy15 <= signed(resize(s_hy14, 11)) - signed(resize(s_py(14), 11));
            s_du15 <= signed(resize(s_hu14, 11)) - signed(resize(s_pu(14), 11));
            s_dv15 <= signed(resize(s_hv14, 11)) - signed(resize(s_pv(14), 11));
            if s_sw_glass = '1' then
                s_w15 <= s_gwd(14);
            else
                s_w15 <= s_wd(14);
            end if;
            -- S16: products split on the weight (high 5 / low 4 bits)
            s_phy16 <= s_dy15 * signed('0' & s_w15(8 downto 4));
            s_phu16 <= s_du15 * signed('0' & s_w15(8 downto 4));
            s_phv16 <= s_dv15 * signed('0' & s_w15(8 downto 4));
            s_ply16 <= s_dy15 * signed('0' & s_w15(3 downto 0));
            s_plu16 <= s_du15 * signed('0' & s_w15(3 downto 0));
            s_plv16 <= s_dv15 * signed('0' & s_w15(3 downto 0));
            -- S17
            s_pry17 <= shift_left(resize(s_phy16, 21), 4) + resize(s_ply16, 21);
            s_pru17 <= shift_left(resize(s_phu16, 21), 4) + resize(s_plu16, 21);
            s_prv17 <= shift_left(resize(s_phv16, 21), 4) + resize(s_plv16, 21);
            -- S18: picture + (hatch - picture) * w / 256
            s_y18 <= f_clamp10(signed(resize(s_py(17), 14)) + resize(shift_right(s_pry17, 8), 14));
            s_u18 <= f_clamp10(signed(resize(s_pu(17), 14)) + resize(shift_right(s_pru17, 8), 14));
            s_v18 <= f_clamp10(signed(resize(s_pv(17), 14)) + resize(shift_right(s_prv17, 8), 14));

            -- S19..S20: highlight knee + cap; neutral outside the avid (so
            -- the smoothing's edges are clean)
            s_yk19 <= signed(resize(s_y18, 11)) - C_KNEE;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            if s_avid_sr(18) = '0' then
                s_y20 <= to_unsigned(64, 10);
                s_u20 <= to_unsigned(512, 10);
                s_v20 <= to_unsigned(512, 10);
            else
                if s_yk19 <= 0 then
                    s_y20 <= s_y19;
                elsif s_yk19 >= 2 * (C_YMAX - C_KNEE) then
                    s_y20 <= to_unsigned(C_YMAX, 10);
                else
                    s_y20 <= to_unsigned(C_KNEE, 10) + unsigned(s_yk19(9 downto 1));
                end if;
                s_u20 <= s_u19;
                s_v20 <= s_v19;
            end if;

            -- S21..S22: horizontal [1 2 1]/4 on the pixel one back (g1 the
            -- centre, g2 and S20 its neighbours)
            s_g1y <= s_y20;  s_g2y <= s_g1y;
            s_g1u <= s_u20;  s_g2u <= s_g1u;
            s_g1v <= s_v20;  s_g2v <= s_g1v;
            v_s := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y20, 12);
            s_y22 <= v_s(11 downto 2);
            v_s := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u20, 12);
            s_u22 <= v_s(11 downto 2);
            v_s := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v20, 12);
            s_v22 <= v_s(11 downto 2);

            -- S23: blanking gate
            if s_avid_sr(21) = '1' then
                s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10);
                s_u23 <= to_unsigned(512, 10);
                s_v23 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y23);
    s_io_align_in.u       <= std_logic_vector(s_u23);
    s_io_align_in.v       <= std_logic_vector(s_v23);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (23 -> 24).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture hatchglass;
