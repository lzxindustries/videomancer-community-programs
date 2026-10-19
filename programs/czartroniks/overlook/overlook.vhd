-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- overlook.vhd  (v1.3.1 -- the Overlook Hotel corridor carpet)
--
-- v1.3.3 (2026-10-17): resolution scaling.  The texture step is now per
-- virtual 1080-line unit: the measured width/height, the row counter that
-- drives the perspective engine, and the row step are in those units
-- (stepped from the video timing ID, registers_in(8): 1.0 at 1080p,
-- identical to before; 1.5 at 720p; at SD 2.0 across -- a 1440-wide 4:3
-- window -- and 1080/486 per frame line, doubled per interlaced field
-- line), and the per-pixel u step is the line's du times the x step.  The
-- carpet tile, zoom, drift and corridor now follow screen height and stay
-- square on a 4:3 SD screen.  Interlace comes from the timing ID (field_n
-- is tied high on hardware, so the old field_n toggle test never saw it).
--
-- David Hicks' "Hexagon" -- the Shining carpet -- from the user's exact
-- vector spec.  The orange figure is a set of identical horizontal
-- SERPENTINE polylines stacked vertically at the same phase; their upward
-- peaks and downward V-tips interlock without touching, and the hexagon
-- "cells" are pure negative space.  Solid red hexagons float in the dark
-- field touching nothing.
--
-- Tile: user 100x142 px scaled x10.24 -> 1024 x 1454 texture units
-- (u period = 1024 = the free wrap; stroke 12.5px = 128 exactly).
-- Half-period polyline (x-fold symmetric about 0 and 512):
--   A(0,0) B(374,210) C(374,650) D(143,773) E(143,1213) F(512,1423)
-- stroked with half-width 64 and MITER joins.  Red hexagons: regular
-- pointy-top, circumradius 169, at (0,430) and (512,993).
--
-- Rendering: per-segment slabs clipped by the miter lines at each join --
-- 24 half-plane tests fed by FIVE shared x-products (9/16, 17/32, 73/128,
-- 19/32, 37/64 of the folded x, single-truncation) and five adders; all
-- coordinates carry 4 fractional bits so edges resolve at 1/16 texel.
--
-- v1.3: PERSPECTIVE.  A per-line engine (runs during the previous line,
-- swapped in at active-video rise) maps every row through a projective
-- floor:  D = H/2 + K*s'  (s' = frame row from centre, |s| under Mirror),
-- r = (H/2)/D by a 16-step serial divider, du = du0*r, u0 = ucen - du*W/2,
-- v = vcen + s'*du.  K = 0 is the flat carpet exactly; rising K tilts it
-- into a corridor floor with the horizon entering from the top, fogged to
-- black by depth (and by texel density, which also hides aliasing).  The
-- same engine pre-fades the palette per line (fog + Night), so the pixel
-- path only selects colours.
--
-- Controls:
--   K1 Drift X / K2 Drift Y : bipolar drift, constant screen speed at any zoom
--   K3 Perspective          : flat carpet -> corridor floor with horizon
--   K4 Weight               : stroke half-width ~20..84 texels
--   K5 Hex Size             : red hexagons ~60..140%
--   K6 Scheme               : Classic / Gold Room / Lobby Blue / Deep Night /
--                             Mono Amber (labelled; v<5 read as label index)
--   S7 Mirror   (rows mirror about centre; with Perspective a carpet valley)
--   S8 Weave    (texture-anchored pile grain, hashed with adds)
--   S9 Halftone (input luma sets each red hexagon's size per pixel)
--   S10 Inverse (field and serpentine colours swap: orange ground,
--               plum serpentines, the hexagon gaps read as orange cells)
--   S11 Night   (field 1/4, orange 1/2, reds full -- toward black, not below)
--   P12 Zoom    (exponential, continuous octave law, centre-anchored, glided)
--
-- Hardware contracts: processing type (S9 reads data_in), outputs gated to
-- neutral outside avid, serrated-vsync guard (saw-active flag), bottom
-- field = field_n '0', one 6-clock latency for all modes.
--
-- Palette stored U/V (Cb/Cr) swapped for the Videomancer output convention.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture overlook of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (6 + 2 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 6;

    -- texture tile
    constant C_VWRAP   : integer := 93056;       -- 1454 * 64
    constant C_VW64    : integer := 93056 * 64;  -- drift origin, 6 frac bits

    -- chroma magnitude/sign about 512 (magnitude of a negative offset is
    -- one short -- an invisible 1-code error that saves an adder)
    function f_cmag(c : unsigned(9 downto 0)) return unsigned is
    begin
        if c(9) = '1' then
            return c(8 downto 0);
        else
            return not c(8 downto 0);
        end if;
    end function;

    -- depth fog, linear in 1/r (= linear in screen rows): full at r <= 2,
    -- gone at the r = 16 horizon clamp.  idx = floor(4r) = q(15 downto 10).
    function f_fog(idx : unsigned(5 downto 0)) return unsigned is
    begin
        case to_integer(idx) is
            when 0 to 7 => return to_unsigned(16, 5);
            when 8 => return to_unsigned(15, 5);
            when 9 => return to_unsigned(13, 5);
            when 10 => return to_unsigned(12, 5);
            when 11 => return to_unsigned(10, 5);
            when 12 to 13 => return to_unsigned(9, 5);
            when 14 => return to_unsigned(8, 5);
            when 15 to 16 => return to_unsigned(7, 5);
            when 17 to 18 => return to_unsigned(6, 5);
            when 19 to 21 => return to_unsigned(5, 5);
            when 22 to 24 => return to_unsigned(4, 5);
            when 25 to 30 => return to_unsigned(3, 5);
            when 31 to 38 => return to_unsigned(2, 5);
            when 39 to 52 => return to_unsigned(1, 5);
            when others => return to_unsigned(0, 5);
        end case;
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

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sw    : std_logic_vector(4 downto 0) := "00000";  -- S7..S11
    signal s_p12   : unsigned(9 downto 0) := to_unsigned(618, 10);

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_vs_pulse     : std_logic := '0';
    signal s_saw          : std_logic := '0';
    signal s_first        : std_logic := '0';
    -- registered sync events: keep the input pins out of the decision cone
    signal s_e_vs, s_e_hs, s_e_av : std_logic := '0';
    signal s_avid_d       : std_logic := '0';

    -- resolution scaling: x step per pixel in half virtual units (2/3/4 =
    -- 1.0/1.5/2.0), y step per FIELD line in Q4.8 virtual units, from the
    -- video timing ID (static while a mode runs; decoded every clock)
    signal s_xs   : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);

    -- measured raster, virtual units (width Q.1, height Q.8)
    signal s_wcnt : unsigned(12 downto 0) := (others => '0');
    signal s_w2   : unsigned(10 downto 0) := to_unsigned(360, 11);
    signal s_hacc : unsigned(19 downto 0) := (others => '0');
    signal s_h2   : unsigned(10 downto 0) := to_unsigned(120, 11);

    ----------------------------------------------------------------------
    -- vblank sequencer + per-line engine: one shared registered multiplier
    ----------------------------------------------------------------------
    signal s_seq  : unsigned(5 downto 0) := (others => '0');
    signal s_lseq : unsigned(5 downto 0) := (others => '0');

    signal s_ma   : unsigned(11 downto 0) := (others => '0');
    signal s_mb   : unsigned(7 downto 0)  := (others => '0');
    signal s_mp   : unsigned(19 downto 0) := (others => '0');

    signal s_t    : unsigned(9 downto 0) := (others => '0');
    signal s_t2   : unsigned(9 downto 0) := (others => '0');
    signal s_oct  : unsigned(2 downto 0) := (others => '0');
    signal s_mant : unsigned(7 downto 0) := (others => '0');
    signal s_dut  : unsigned(13 downto 0) := to_unsigned(176, 14);
    signal s_du   : unsigned(13 downto 0) := to_unsigned(176, 14);
    signal s_due  : unsigned(13 downto 0) := to_unsigned(176, 14);

    -- glide pipeline scratch
    signal s_gdd  : signed(14 downto 0) := (others => '0');
    signal s_gsh  : signed(14 downto 0) := (others => '0');

    -- X drift sign
    signal s_bsg  : std_logic := '0';

    -- drift origin (6 fractional bits)
    signal s_ucen  : unsigned(21 downto 0) := to_unsigned(2097152, 22);
    signal s_vcen  : unsigned(22 downto 0) := to_unsigned(4067328, 23);
    signal s_dsgy  : std_logic := '0';
    signal s_dmagy : unsigned(7 downto 0) := (others => '0');
    signal s_vraw  : signed(24 downto 0) := (others => '0');

    -- frame palette (Classic defaults)
    signal s_pbg_y : unsigned(9 downto 0) := to_unsigned(184, 10);
    signal s_pbg_u : unsigned(9 downto 0) := to_unsigned(555, 10);
    signal s_pbg_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_por_y : unsigned(9 downto 0) := to_unsigned(504, 10);
    signal s_por_u : unsigned(9 downto 0) := to_unsigned(793, 10);
    signal s_por_v : unsigned(9 downto 0) := to_unsigned(357, 10);
    signal s_pre_y : unsigned(9 downto 0) := to_unsigned(331, 10);
    signal s_pre_u : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_pre_v : unsigned(9 downto 0) := to_unsigned(467, 10);
    signal s_sidx  : unsigned(2 downto 0) := (others => '0');

    -- per-frame pattern thresholds (Weight / Hex Size), x16 fixed point
    signal s_hs    : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_hd    : signed(15 downto 0)   := to_signed(1168, 16);
    signal s_c1dl  : signed(15 downto 0)   := to_signed(22096, 16);
    signal s_c1dh  : signed(15 downto 0)   := to_signed(24432, 16);
    signal s_c3l   : signed(15 downto 0)   := to_signed(12408, 16);
    signal s_c3h   : signed(15 downto 0)   := to_signed(14744, 16);
    signal s_c5l   : signed(15 downto 0)   := to_signed(16936, 16);
    signal s_c5h   : signed(15 downto 0)   := to_signed(19272, 16);
    signal s_c5ul  : signed(15 downto 0)   := to_signed(-6328, 16);
    signal s_c5uh  : signed(15 downto 0)   := to_signed(-3992, 16);
    signal s_c2xl  : unsigned(13 downto 0) := to_unsigned(4960, 14);
    signal s_c2xh  : unsigned(13 downto 0) := to_unsigned(7008, 14);
    signal s_c4xl  : unsigned(13 downto 0) := to_unsigned(1264, 14);
    signal s_c4xh  : unsigned(13 downto 0) := to_unsigned(3312, 14);
    signal s_hr    : unsigned(12 downto 0) := to_unsigned(2704, 13);
    signal s_hrb   : signed(13 downto 0)   := to_signed(-196, 14);

    -- per-frame geometry for the line engine
    signal s_kq    : unsigned(7 downto 0)  := (others => '0');
    signal s_h2f   : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_s0e   : signed(11 downto 0)   := to_signed(-240, 12);

    ----------------------------------------------------------------------
    -- per-line engine state
    ----------------------------------------------------------------------
    signal s_snext  : signed(19 downto 0) := (others => '0');   -- virtual row, Q.8
    signal s_sabs   : unsigned(9 downto 0) := (others => '0');
    signal s_sneg   : std_logic := '0';
    signal s_dq     : signed(19 downto 0) := (others => '0');
    signal s_dd     : unsigned(17 downto 0) := to_unsigned(15360, 18);
    signal s_beyond : std_logic := '0';
    signal s_rem    : unsigned(17 downto 0) := (others => '0');
    signal s_q      : unsigned(15 downto 0) := (others => '0');
    signal s_ar     : unsigned(4 downto 0) := (others => '0');
    signal s_adu    : unsigned(4 downto 0) := (others => '0');
    signal s_a      : unsigned(4 downto 0) := (others => '0');
    signal s_fbg, s_for, s_fre : unsigned(4 downto 0) := (others => '0');
    signal s_pa     : unsigned(19 downto 0) := (others => '0');
    signal s_t32    : unsigned(31 downto 0) := (others => '0');
    signal s_vm     : unsigned(25 downto 0) := (others => '0');
    signal s_modc   : unsigned(24 downto 0) := (others => '0');
    signal s_vr     : signed(18 downto 0) := (others => '0');

    -- generic lerp pipeline (ref +- mag*f/16)
    signal s_lsg0, s_lsg1 : std_logic := '0';
    signal s_lyc0, s_lyc1 : std_logic := '0';
    signal s_lr     : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- next-line results
    signal s_dun   : unsigned(21 downto 0) := to_unsigned(11264, 22);
    signal s_dupn  : unsigned(21 downto 0) := to_unsigned(11264, 22);  -- du * x step
    signal s_u0n   : unsigned(21 downto 0) := (others => '0');
    signal s_vn    : unsigned(16 downto 0) := (others => '0');
    signal s_wampn : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal n_bg_y, n_or_y, n_re_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal n_bg_u, n_bg_v, n_or_u, n_or_v, n_re_u, n_re_v
                   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- current-line values (swapped in at active-video rise)
    signal s_du_px  : unsigned(21 downto 0) := to_unsigned(11264, 22);
    signal s_wamp   : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal c_bg_y, c_or_y, c_re_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_bg_u, c_bg_v, c_or_u, c_or_v, c_re_u, c_re_v
                   : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- texture accumulators
    ----------------------------------------------------------------------
    signal s_accu   : unsigned(21 downto 0) := (others => '0');  -- 6 frac bits
    signal s_vline  : unsigned(16 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';
    signal s_lum4   : unsigned(11 downto 0) := (others => '0');  -- (Y-64)*4

    ----------------------------------------------------------------------
    -- pixel pipeline (all pattern coordinates carry 4 fractional bits)
    ----------------------------------------------------------------------
    signal r1_xf : unsigned(13 downto 0) := (others => '0');
    signal r1_v  : unsigned(14 downto 0) := (others => '0');
    signal r1_hr : unsigned(12 downto 0) := (others => '0');
    signal r1_cu : unsigned(7 downto 0)  := (others => '0');
    signal r1_cv : unsigned(7 downto 0)  := (others => '0');

    signal r2_xf   : unsigned(13 downto 0) := (others => '0');
    signal r2_v    : unsigned(14 downto 0) := (others => '0');
    signal r2_p36  : unsigned(13 downto 0) := (others => '0');
    signal r2_p17  : unsigned(13 downto 0) := (others => '0');
    signal r2_p73  : unsigned(13 downto 0) := (others => '0');
    signal r2_p19  : unsigned(13 downto 0) := (others => '0');
    signal r2_h1   : unsigned(15 downto 0) := (others => '0');
    signal r2_h2   : unsigned(15 downto 0) := (others => '0');
    signal r2_hr   : unsigned(12 downto 0) := (others => '0');
    signal r2_hwr  : unsigned(12 downto 0) := (others => '0');
    signal r2_hs   : unsigned(15 downto 0) := (others => '0');

    signal r3_xf    : unsigned(13 downto 0) := (others => '0');
    signal r3_vm36  : signed(15 downto 0) := (others => '0');
    signal r3_vp17  : signed(15 downto 0) := (others => '0');
    signal r3_vm73  : signed(15 downto 0) := (others => '0');
    signal r3_vp19  : signed(15 downto 0) := (others => '0');
    signal r3_vm19  : signed(15 downto 0) := (others => '0');
    signal r3_h1    : unsigned(15 downto 0) := (others => '0');
    signal r3_h2    : unsigned(15 downto 0) := (others => '0');
    signal r3_hr    : unsigned(12 downto 0) := (others => '0');
    signal r3_hx1, r3_hx2 : std_logic := '0';
    signal r3_hs    : unsigned(15 downto 0) := (others => '0');

    signal r4_code : unsigned(1 downto 0) := (others => '0');
    signal r4_wh   : unsigned(3 downto 0) := (others => '0');

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement (height updated on active lines, never at vsync)
    ------------------------------------------------------------------------
    p_meas : process(clk)
        variable v_h : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- width / height in virtual units (1080p: the pixel / line counts)
            if data_in.avid = '1' then
                s_wcnt <= s_wcnt + s_xs;
            elsif s_avid_p = '1' then
                s_w2   <= s_wcnt(12 downto 2);
                s_wcnt <= (others => '0');
            end if;

            if s_vs_pulse = '1' then
                s_hacc <= (others => '0');
            elsif data_in.avid = '1' and s_avid_p = '0' then
                v_h    := s_hacc + s_ys;
                s_hacc <= v_h;
                s_h2   <= v_h(19 downto 9);
            end if;
        end if;
    end process p_meas;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (63 steps) + per-line
    -- engine (57 steps, during the previous line).  One process so the two
    -- can share the registered multiplier; they are disjoint in time.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_st  : signed(14 downto 0);
        variable v_dk  : signed(10 downto 0);
        variable v_d9  : signed(9 downto 0);
        variable v_t12 : unsigned(11 downto 0);
        variable v_s   : signed(11 downto 0);
        variable v_sa  : signed(11 downto 0);
        variable v_ab  : signed(9 downto 0);
        variable v_lim : unsigned(17 downto 0);
        variable v_r   : unsigned(18 downto 0);
        variable v_w28 : unsigned(27 downto 0);
        variable v_adu : signed(23 downto 0);
        variable v_ref : unsigned(9 downto 0);
        variable v_lp  : unsigned(9 downto 0);
        variable v_ix  : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_hsync_n <= data_in.hsync_n;
            s_mp <= s_ma * s_mb;

            -- generic lerp result, two clocks behind the operand issue
            s_lsg1 <= s_lsg0;
            s_lyc1 <= s_lyc0;
            v_lp := s_mp(13 downto 4);
            if s_lyc1 = '1' then
                v_ref := to_unsigned(64, 10);
            else
                v_ref := to_unsigned(512, 10);
            end if;
            if s_lsg1 = '1' then
                s_lr <= v_ref - v_lp;
            else
                s_lr <= v_ref + v_lp;
            end if;

            s_vs_pulse <= '0';
            if data_in.avid = '1' then
                s_saw <= '1';
            end if;

            -- edges registered one clock (all events act a clock late)
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_e_vs <= '1';
            else
                s_e_vs <= '0';
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_e_hs <= '1';
            else
                s_e_hs <= '0';
            end if;
            if data_in.avid = '1' and s_avid_p = '0' then
                s_e_av <= '1';
            else
                s_e_av <= '0';
            end if;
            s_avid_d <= data_in.avid;

            ----------------------------------------------------------------
            -- vblank sequencer
            ----------------------------------------------------------------
            if s_seq /= 0 and s_avid_d = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- ==== zoom target: continuous octave law over 6
                    -- octaves (du 32..2040: tile 2048 px .. 32 px) ====
                    when 63 => s_t <= to_unsigned(1023, 10) - s_p12;
                    when 62 =>
                        v_t12 := resize(s_t, 12) + (resize(s_t, 11) & '0');
                        s_t2  <= v_t12(11 downto 2);
                    when 61 => s_oct  <= s_t2(9 downto 7);
                               s_mant <= to_unsigned(128, 8) + resize(s_t2(6 downto 0), 8);
                    when 60 => s_dut <= resize(shift_right(
                                   shift_left(resize(s_mant, 16), to_integer(s_oct)), 2), 14);

                    -- ==== fixed 1/16 glide (swallows P12 ADC jitter) ====
                    when 59 =>
                        s_gdd <= signed(resize(s_dut, 15)) - signed(resize(s_du, 15));
                    when 58 =>
                        s_gsh <= shift_right(s_gdd, 4);
                    when 57 =>
                        v_st := s_gsh;
                        if v_st = 0 and s_gdd /= 0 then
                            if s_gdd > 0 then v_st := to_signed(1, 15);
                            else              v_st := to_signed(-1, 15);
                            end if;
                        end if;
                        s_du <= unsigned(resize(signed(resize(s_du, 15)) + v_st, 14));

                    when 52 => s_due <= s_du;

                    -- ==== drift (K1/K2, deadband 8): step = d * du * 2,
                    -- 6 frac bits -> constant screen speed at any zoom,
                    -- up to ~8 px/frame ====
                    when 51 =>
                        v_dk := signed('0' & s_k1) - to_signed(512, 11);
                        if v_dk > 8 then
                            v_d9 := resize(shift_right(v_dk - 8, 1), 10);
                        elsif v_dk < -8 then
                            v_d9 := resize(shift_right(v_dk + 8, 1), 10);
                        else
                            v_d9 := (others => '0');
                        end if;
                        s_bsg <= v_d9(9);                    -- X drift sign
                        s_ma  <= s_due(11 downto 0);
                        v_ab  := abs(v_d9);
                        s_mb  <= unsigned(v_ab(7 downto 0));
                    when 50 =>
                        v_dk := signed('0' & s_k2) - to_signed(512, 11);
                        if v_dk > 8 then
                            v_d9 := resize(shift_right(v_dk - 8, 1), 10);
                        elsif v_dk < -8 then
                            v_d9 := resize(shift_right(v_dk + 8, 1), 10);
                        else
                            v_d9 := (others => '0');
                        end if;
                        s_dsgy  <= v_d9(9);
                        v_ab    := abs(v_d9);
                        s_dmagy <= unsigned(v_ab(7 downto 0));
                    when 49 =>
                        s_mb <= s_dmagy;
                        if s_bsg = '0' then
                            s_ucen <= s_ucen + resize(s_mp & '0', 22);
                        else
                            s_ucen <= s_ucen - resize(s_mp & '0', 22);
                        end if;
                    when 47 =>
                        if s_dsgy = '0' then
                            s_vraw <= signed(resize(s_vcen, 25)) + signed(resize(s_mp & '0', 25));
                        else
                            s_vraw <= signed(resize(s_vcen, 25)) - signed(resize(s_mp & '0', 25));
                        end if;
                    when 46 =>
                        if s_vraw < 0 then
                            s_vraw <= s_vraw + to_signed(C_VW64, 25);
                        elsif s_vraw >= to_signed(C_VW64, 25) then
                            s_vraw <= s_vraw - to_signed(C_VW64, 25);
                        end if;
                    when 45 =>
                        s_vcen <= unsigned(s_vraw(22 downto 0));

                    -- ==== Weight (K4): stroke half-width + slab bounds ====
                    when 44 => s_hs <= to_unsigned(320, 14) + resize(s_k4, 14)
                                       + resize(s_k4(9 downto 2), 14);
                    when 43 => s_hd <= signed(resize(s_hs + shift_right(s_hs, 3)
                                       + shift_right(s_hs, 6), 16));
                    when 42 => s_c2xl <= to_unsigned(5984, 14) - s_hs;
                               s_c2xh <= to_unsigned(5984, 14) + s_hs;
                    when 41 => s_c4xl <= to_unsigned(2288, 14) - s_hs;
                               s_c4xh <= to_unsigned(2288, 14) + s_hs;
                    when 40 => s_c1dl <= to_signed(23264, 16) - s_hd;
                               s_c1dh <= to_signed(23264, 16) + s_hd;
                    when 39 => s_c5l <= to_signed(18104, 16) - s_hd;
                               s_c5h <= to_signed(18104, 16) + s_hd;
                    when 38 => s_c5ul <= to_signed(-5160, 16) - s_hd;
                               s_c5uh <= to_signed(-5160, 16) + s_hd;
                    when 37 => s_c3l <= to_signed(13576, 16) - s_hd;
                               s_c3h <= to_signed(13576, 16) + s_hd;

                    -- ==== Hex Size (K5): hr = 1622 + 2.125*K5; halftone
                    -- base hrb = hr - 2900 (luma*4 is added per pixel) ====
                    when 36 => s_hr <= resize(to_unsigned(1622, 13)
                                       + resize(s_k5 & '0', 13)
                                       + resize(s_k5(9 downto 3), 13), 13);
                    when 35 => s_hrb <= signed(resize(s_hr, 14)) - to_signed(2900, 14);

                    -- ==== Scheme (K6): labelled; v<5 is a raw label index
                    -- (firmware load-default quirk), else equal fifths ====
                    when 34 =>
                        if s_k6 < 5 then
                            s_sidx <= s_k6(2 downto 0);
                        elsif s_k6 < 205 then s_sidx <= to_unsigned(0, 3);
                        elsif s_k6 < 410 then s_sidx <= to_unsigned(1, 3);
                        elsif s_k6 < 614 then s_sidx <= to_unsigned(2, 3);
                        elsif s_k6 < 819 then s_sidx <= to_unsigned(3, 3);
                        else                  s_sidx <= to_unsigned(4, 3);
                        end if;
                    when 33 =>
                        v_ix := to_integer(s_sidx);
                        case v_ix is
                            when 0 =>                            -- Classic
                                s_pbg_y <= to_unsigned(184, 10);
                                s_pbg_u <= to_unsigned(555, 10);
                                s_pbg_v <= to_unsigned(512, 10);
                                s_por_y <= to_unsigned(504, 10);
                                s_por_u <= to_unsigned(793, 10);
                                s_por_v <= to_unsigned(357, 10);
                                s_pre_y <= to_unsigned(331, 10);
                                s_pre_u <= to_unsigned(768, 10);
                                s_pre_v <= to_unsigned(467, 10);
                            when 1 =>                            -- Gold Room
                                s_pbg_y <= to_unsigned(159, 10);
                                s_pbg_u <= to_unsigned(538, 10);
                                s_pbg_v <= to_unsigned(477, 10);
                                s_por_y <= to_unsigned(709, 10);
                                s_por_u <= to_unsigned(673, 10);
                                s_por_v <= to_unsigned(255, 10);
                                s_pre_y <= to_unsigned(282, 10);
                                s_pre_u <= to_unsigned(729, 10);
                                s_pre_v <= to_unsigned(465, 10);
                            when 2 =>                            -- Lobby Blue
                                s_pbg_y <= to_unsigned(136, 10);
                                s_pbg_u <= to_unsigned(500, 10);
                                s_pbg_v <= to_unsigned(546, 10);
                                s_por_y <= to_unsigned(514, 10);
                                s_por_u <= to_unsigned(414, 10);
                                s_por_v <= to_unsigned(645, 10);
                                s_pre_y <= to_unsigned(315, 10);
                                s_pre_u <= to_unsigned(755, 10);
                                s_pre_v <= to_unsigned(470, 10);
                            when 3 =>                            -- Deep Night
                                s_pbg_y <= to_unsigned(100, 10);
                                s_pbg_u <= to_unsigned(521, 10);
                                s_pbg_v <= to_unsigned(519, 10);
                                s_por_y <= to_unsigned(239, 10);
                                s_por_u <= to_unsigned(625, 10);
                                s_por_v <= to_unsigned(451, 10);
                                s_pre_y <= to_unsigned(346, 10);
                                s_pre_u <= to_unsigned(782, 10);
                                s_pre_v <= to_unsigned(464, 10);
                            when others =>                       -- Mono Amber
                                s_pbg_y <= to_unsigned( 93, 10);
                                s_pbg_u <= to_unsigned(516, 10);
                                s_pbg_v <= to_unsigned(507, 10);
                                s_por_y <= to_unsigned(728, 10);
                                s_por_u <= to_unsigned(599, 10);
                                s_por_v <= to_unsigned(366, 10);
                                s_pre_y <= to_unsigned(453, 10);
                                s_pre_u <= to_unsigned(579, 10);
                                s_pre_v <= to_unsigned(406, 10);
                        end case;

                    -- ==== line-engine frame constants ====
                    when 32 =>
                        s_kq <= s_k3(9 downto 2);
                        s_h2f <= s_h2;          -- virtual: interlace is in s_ys
                    when 31 =>
                        s_s0e <= -signed(resize(s_h2f, 12));

                    -- ==== S10 Inverse: field and serpentine colours trade
                    -- places (re-applied after every frame's fresh load) ====
                    when 30 =>
                        if s_sw(3) = '1' then
                            s_pbg_y <= s_por_y;  s_por_y <= s_pbg_y;
                            s_pbg_u <= s_por_u;  s_por_u <= s_pbg_u;
                            s_pbg_v <= s_por_v;  s_por_v <= s_pbg_v;
                        end if;

                    when others => null;
                end case;

            end if;

            ----------------------------------------------------------------
            -- per-line engine
            ----------------------------------------------------------------
            if s_lseq /= 0 then
                s_lseq <= s_lseq - 1;

                case to_integer(s_lseq) is

                    -- s' = frame row from centre (|s| under Mirror), virtual
                    -- units: the integer part of a Q.8 row accumulator
                    when 63 =>
                        v_s := s_snext(19 downto 8);
                        s_snext <= s_snext + signed(resize(s_ys, 20));
                        if v_s < 0 and s_sw(0) = '0' then
                            s_sneg <= '1';
                        else
                            s_sneg <= '0';
                        end if;
                        v_sa   := abs(v_s);
                        s_sabs <= unsigned(v_sa(9 downto 0));
                        s_ma   <= resize(unsigned(v_sa(9 downto 0)), 12);
                        s_mb   <= s_kq;

                    -- D = 64*H/2 +- K*|s'|  (6 frac bits)
                    when 61 =>
                        if s_sneg = '1' then
                            s_dq <= signed(resize(s_h2f & "000000", 20)) - signed(resize(s_mp, 20));
                        else
                            s_dq <= signed(resize(s_h2f & "000000", 20)) + signed(resize(s_mp, 20));
                        end if;

                    -- beyond the horizon (D <= H/32 * 2) clamps; divider init
                    when 60 =>
                        v_lim := resize(s_h2f & "00", 18);
                        if s_dq <= signed(resize(v_lim, 20)) then
                            s_dd     <= v_lim + 1;
                            s_beyond <= '1';
                        else
                            s_dd     <= unsigned(s_dq(17 downto 0));
                            s_beyond <= '0';
                        end if;
                        s_rem <= v_lim;
                        s_q   <= (others => '0');

                    -- r = (H/2 * 2^18) / D  ->  1/w, Q4.12
                    when 44 to 59 =>
                        v_r := s_rem & '0';
                        if v_r >= ('0' & s_dd) then
                            v_r := v_r - ('0' & s_dd);
                            s_q <= s_q(14 downto 0) & '1';
                        else
                            s_q <= s_q(14 downto 0) & '0';
                        end if;
                        s_rem <= v_r(17 downto 0);

                    -- du = du0 * r  (6 frac bits)
                    when 43 =>
                        s_ma <= s_due(11 downto 0);
                        s_mb <= s_q(15 downto 8);
                        s_ar <= f_fog(s_q(15 downto 10));
                    when 42 => s_mb <= s_q(7 downto 0);
                    when 41 => s_pa <= s_mp;
                    when 40 =>
                        v_w28 := shift_left(resize(s_pa, 28), 8) + resize(s_mp, 28);
                        s_dun <= v_w28(27 downto 6);

                    -- u0 = ucen - du * W/2  (mod 2^22)
                    when 39 =>
                        s_ma <= resize(s_w2, 12);
                        s_mb <= "00" & s_dun(21 downto 16);
                    when 38 =>
                        s_mb <= s_dun(15 downto 8);
                        -- density fog: full to du 3072, gone at du 5120
                        v_adu := to_signed(327680, 24) - signed(resize(s_dun, 24));
                        if v_adu <= 0 then
                            s_adu <= (others => '0');
                        elsif v_adu >= to_signed(131072, 24) then
                            s_adu <= to_unsigned(16, 5);
                        else
                            s_adu <= '0' & unsigned(v_adu(16 downto 13));
                        end if;
                    when 37 =>
                        s_mb <= s_dun(7 downto 0);
                        s_pa <= s_mp;
                    when 36 =>
                        s_t32 <= shift_left(resize(s_pa, 32), 16) + shift_left(resize(s_mp, 32), 8);
                        if s_beyond = '1' then
                            s_a <= (others => '0');
                        elsif s_ar < s_adu then
                            s_a <= s_ar;
                        else
                            s_a <= s_adu;
                        end if;
                    when 35 =>
                        s_t32 <= s_t32 + resize(s_mp, 32);
                        s_ma  <= resize(s_sabs, 12);
                        s_mb  <= "00" & s_dun(21 downto 16);
                    when 34 =>
                        s_u0n <= s_ucen - s_t32(21 downto 0);
                        s_mb  <= s_dun(15 downto 8);

                    -- v = vcen +- |s'| * du  (mod 93056)
                    when 33 =>
                        s_mb <= s_dun(7 downto 0);
                        s_pa <= s_mp;
                    when 32 =>
                        s_t32 <= shift_left(resize(s_pa, 32), 16) + shift_left(resize(s_mp, 32), 8);
                    when 31 =>
                        s_t32 <= s_t32 + resize(s_mp, 32);
                    when 30 =>
                        s_vm   <= s_t32(31 downto 6);
                        s_modc <= to_unsigned(C_VWRAP * 256, 25);
                        -- fog factors: Night = field 1/4, orange 1/2
                        s_fre <= s_a;
                        if s_sw(4) = '1' then
                            s_fbg <= "00" & s_a(4 downto 2);
                            s_for <= '0' & s_a(4 downto 1);
                        else
                            s_fbg <= s_a;
                            s_for <= s_a;
                        end if;
                        s_wampn <= s_a(4 downto 2);
                    when 21 to 29 =>
                        if ('0' & s_vm) >= ("00" & s_modc) then
                            s_vm <= s_vm - resize(s_modc, 26);
                        end if;
                        s_modc <= '0' & s_modc(24 downto 1);
                    when 20 =>
                        if s_sneg = '1' then
                            s_vr <= signed(resize(s_vcen(22 downto 6), 19))
                                    - signed(resize(s_vm(16 downto 0), 19));
                        else
                            s_vr <= signed(resize(s_vcen(22 downto 6), 19))
                                    + signed(resize(s_vm(16 downto 0), 19));
                        end if;
                    when 19 =>
                        if s_vr < 0 then
                            s_vn <= unsigned(resize(s_vr + to_signed(C_VWRAP, 19), 17));
                        elsif s_vr >= to_signed(C_VWRAP, 19) then
                            s_vn <= unsigned(resize(s_vr - to_signed(C_VWRAP, 19), 17));
                        else
                            s_vn <= unsigned(s_vr(16 downto 0));
                        end if;

                    -- palette fade: issue (18..10), result copied 3 later
                    when 18 =>
                        s_ma <= resize(s_pbg_y - 64, 12); s_mb <= "000" & s_fbg;
                        s_lsg0 <= '0'; s_lyc0 <= '1';
                    when 17 =>
                        s_ma <= resize(f_cmag(s_pbg_u), 12); s_mb <= "000" & s_fbg;
                        s_lsg0 <= not s_pbg_u(9); s_lyc0 <= '0';
                    when 16 =>
                        s_ma <= resize(f_cmag(s_pbg_v), 12); s_mb <= "000" & s_fbg;
                        s_lsg0 <= not s_pbg_v(9); s_lyc0 <= '0';
                    when 15 =>
                        n_bg_y <= s_lr;
                        s_ma <= resize(s_por_y - 64, 12); s_mb <= "000" & s_for;
                        s_lsg0 <= '0'; s_lyc0 <= '1';
                    when 14 =>
                        n_bg_u <= s_lr;
                        s_ma <= resize(f_cmag(s_por_u), 12); s_mb <= "000" & s_for;
                        s_lsg0 <= not s_por_u(9); s_lyc0 <= '0';
                    when 13 =>
                        n_bg_v <= s_lr;
                        s_ma <= resize(f_cmag(s_por_v), 12); s_mb <= "000" & s_for;
                        s_lsg0 <= not s_por_v(9); s_lyc0 <= '0';
                    when 12 =>
                        n_or_y <= s_lr;
                        s_ma <= resize(s_pre_y - 64, 12); s_mb <= "000" & s_fre;
                        s_lsg0 <= '0'; s_lyc0 <= '1';
                    when 11 =>
                        n_or_u <= s_lr;
                        s_ma <= resize(f_cmag(s_pre_u), 12); s_mb <= "000" & s_fre;
                        s_lsg0 <= not s_pre_u(9); s_lyc0 <= '0';
                    when 10 =>
                        n_or_v <= s_lr;
                        s_ma <= resize(f_cmag(s_pre_v), 12); s_mb <= "000" & s_fre;
                        s_lsg0 <= not s_pre_v(9); s_lyc0 <= '0';
                    when 9 => n_re_y <= s_lr;
                    when 8 => n_re_u <= s_lr;
                    when 7 => n_re_v <= s_lr;

                    -- per-pixel u step = du (per virtual unit) * x step
                    when 6 =>
                        case s_xs is
                            when "100"  => s_dupn <= s_dun(20 downto 0) & '0';
                            when "011"  => s_dupn <= s_dun + ('0' & s_dun(21 downto 1));
                            when others => s_dupn <= s_dun;
                        end case;

                    when others => null;
                end case;
            end if;

            -- serrated analog vsync: act on the first edge after active
            -- video only
            if s_e_vs = '1' and s_saw = '1' then
                s_saw      <= '0';
                s_vs_pulse <= '1';
                s_first    <= '1';

                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_sw  <= registers_in(6)(4 downto 0);
                s_p12 <= unsigned(registers_in(7));

                s_seq <= to_unsigned(63, 6);
            end if;

            ----------------------------------------------------------------
            -- line start: swap the precomputed line in, compute the next
            ----------------------------------------------------------------
            if s_e_av = '1' then
                s_first  <= '0';
                s_wamp   <= s_wampn;
                c_bg_y <= n_bg_y;  c_bg_u <= n_bg_u;  c_bg_v <= n_bg_v;
                c_or_y <= n_or_y;  c_or_u <= n_or_u;  c_or_v <= n_or_v;
                c_re_y <= n_re_y;  c_re_u <= n_re_u;  c_re_v <= n_re_v;
                s_lseq <= to_unsigned(63, 6);

            -- before the first active line: (re)compute line 0 on every
            -- blanking hsync -- idempotent, the last run before avid wins.
            -- The row accumulator starts at -H/2 + 0.5 (rounds the virtual
            -- row; 1080p integer rows unchanged).  field_n is tied high on
            -- hardware, so there is no bottom-field seed.
            elsif s_e_hs = '1' and s_first = '1' and s_seq = 0 then
                s_snext <= s_s0e & "10000000";
                s_lseq <= to_unsigned(63, 6);
            end if;

        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- texture accumulators (u per pixel; u0 and v per line from the engine)
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_y : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            v_y := unsigned(data_in.y);
            if v_y > 64 then
                s_lum4 <= resize(v_y - 64, 10) & "00";
            else
                s_lum4 <= (others => '0');
            end if;

            if data_in.avid = '1' and s_avid_p = '0' then
                s_accu  <= s_u0n;
                s_vline <= s_vn;
                s_du_px <= s_dupn;
            elsif data_in.avid = '1' then
                s_accu <= s_accu + s_du_px;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- R1: fold u to |x| (14-bit, 4 frac bits), raw v, halftone radius,
    -- weave cell indices (cell ~1-2 px at the frame zoom octave).
    ------------------------------------------------------------------------
    p_r1 : process(clk)
        variable v_u  : unsigned(13 downto 0);
        variable v_s  : signed(14 downto 0);
        variable v_h  : signed(14 downto 0);
        variable v_cu : unsigned(8 downto 0);
        variable v_cv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_u := s_accu(21 downto 8);
            if v_u(13) = '1' then
                v_s := to_signed(16384, 15) - signed('0' & v_u);
                r1_xf <= unsigned(v_s(13 downto 0));
            else
                r1_xf <= v_u;
            end if;
            r1_v <= s_vline(16 downto 2);

            -- S9 Halftone: bright input = big red hexagon
            if s_sw(2) = '1' then
                v_h := resize(s_hrb, 15) + signed(resize(s_lum4, 15));
                if v_h < 0 then
                    r1_hr <= (others => '0');
                elsif v_h > 6880 then
                    r1_hr <= to_unsigned(6880, 13);
                else
                    r1_hr <= unsigned(v_h(12 downto 0));
                end if;
            else
                r1_hr <= s_hr;
            end if;

            v_cu := shift_right(s_accu(21 downto 13), to_integer(s_oct));
            v_cv := shift_right(s_vline(16 downto 7), to_integer(s_oct));
            r1_cu <= v_cu(7 downto 0);
            r1_cv <= v_cv(7 downto 0);
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: the five shared x-products (single truncation) + hex sums.
    ------------------------------------------------------------------------
    p_r2 : process(clk)
        variable v_x   : unsigned(13 downto 0);
        variable v_x2  : unsigned(13 downto 0);
        variable v_w   : unsigned(20 downto 0);
        variable v_d1, v_d2 : signed(15 downto 0);
        variable v_a1, v_a2 : unsigned(14 downto 0);
        variable v_p37 : unsigned(13 downto 0);
        variable v_hk  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            v_x := r1_xf;
            r2_xf <= v_x;
            r2_v  <= r1_v;

            v_w := shift_left(resize(v_x, 21), 5) + shift_left(resize(v_x, 21), 2);
            r2_p36 <= resize(shift_right(v_w, 6), 14);
            v_w := shift_left(resize(v_x, 21), 4) + resize(v_x, 21);
            r2_p17 <= resize(shift_right(v_w, 5), 14);
            v_w := shift_left(resize(v_x, 21), 6) + shift_left(resize(v_x, 21), 3)
                   + resize(v_x, 21);
            r2_p73 <= resize(shift_right(v_w, 7), 14);
            v_w := shift_left(resize(v_x, 21), 4) + shift_left(resize(v_x, 21), 1)
                   + resize(v_x, 21);
            r2_p19 <= resize(shift_right(v_w, 5), 14);

            v_d1 := signed('0' & r1_v) - to_signed(6880, 16);
            if v_d1 < 0 then v_d1 := -v_d1; end if;
            v_a1 := unsigned(v_d1(14 downto 0));
            v_w := shift_left(resize(v_x, 21), 5) + shift_left(resize(v_x, 21), 2)
                   + resize(v_x, 21);
            v_p37 := resize(shift_right(v_w, 6), 14);
            r2_h1 <= resize(v_a1, 16) + resize(v_p37, 16);

            v_x2 := to_unsigned(8192, 14) - v_x;
            v_d2 := signed('0' & r1_v) - to_signed(15888, 16);
            if v_d2 < 0 then v_d2 := -v_d2; end if;
            v_a2 := unsigned(v_d2(14 downto 0));
            v_w := shift_left(resize(v_x2, 21), 5) + shift_left(resize(v_x2, 21), 2)
                   + resize(v_x2, 21);
            v_p37 := resize(shift_right(v_w, 6), 14);
            r2_h2 <= resize(v_a2, 16) + resize(v_p37, 16);

            -- hex half-width bound = hr * (1 - 1/8 - 1/64)
            r2_hr  <= r1_hr;
            r2_hwr <= r1_hr - shift_right(r1_hr, 3) - shift_right(r1_hr, 6);

            -- weave hash round 1
            v_hk  := r1_cu & r1_cv;
            r2_hs <= v_hk + shift_left(v_hk, 4) + x"7F4A";
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- R3: the five signed plane sums + hex bound tests.
    ------------------------------------------------------------------------
    p_r3 : process(clk)
        variable v_sv : signed(15 downto 0);
        variable v_hk : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            v_sv := signed('0' & r2_v);
            r3_vm36 <= v_sv - signed(resize(r2_p36, 16));
            r3_vp17 <= v_sv + signed(resize(r2_p17, 16));
            r3_vm73 <= v_sv - signed(resize(r2_p73, 16));
            r3_vp19 <= v_sv + signed(resize(r2_p19, 16));
            r3_vm19 <= v_sv - signed(resize(r2_p19, 16));

            r3_h1 <= r2_h1;
            r3_h2 <= r2_h2;
            r3_hr <= r2_hr;
            if r2_xf <= resize(r2_hwr, 14) then r3_hx1 <= '1'; else r3_hx1 <= '0'; end if;
            if resize(r2_xf, 15) + resize(r2_hwr, 15) >= to_unsigned(8192, 15) then
                r3_hx2 <= '1';
            else
                r3_hx2 <= '0';
            end if;

            r3_xf <= r2_xf;

            -- weave hash round 2
            v_hk  := r2_hs xor shift_right(r2_hs, 7);
            r3_hs <= v_hk + shift_left(v_hk, 3);
        end if;
    end process p_r3;

    ------------------------------------------------------------------------
    -- R4: the serpentine (per-frame slab bounds, fixed miter clips) and
    -- the red hexagons.  Red > orange > ground.
    ------------------------------------------------------------------------
    p_r4 : process(clk)
        variable s1, s1d, s2, s3, s4, s5, s5u : std_logic;
        variable v_org, v_red : std_logic;
        variable v_hk, v_h2 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s1 := '0'; s1d := '0'; s2 := '0'; s3 := '0';
            s4 := '0'; s5 := '0'; s5u := '0';

            if r3_vm36 >= -s_hd and r3_vm36 <= s_hd
               and r3_vp19 <= to_signed(6912, 16) then
                s1 := '1';
            end if;
            if r3_vm36 >= s_c1dl and r3_vm36 <= s_c1dh
               and r3_vp19 <= to_signed(30176, 16) then
                s1d := '1';
            end if;
            if r3_xf >= s_c2xl and r3_xf <= s_c2xh
               and r3_vp19 >= to_signed(6912, 16)
               and r3_vm19 <= to_signed(6848, 16) then
                s2 := '1';
            end if;
            if r3_vp17 >= s_c3l and r3_vp17 <= s_c3h
               and r3_vm19 >= to_signed(6848, 16)
               and r3_vm19 <= to_signed(11008, 16) then
                s3 := '1';
            end if;
            if r3_xf >= s_c4xl and r3_xf <= s_c4xh
               and r3_vm19 >= to_signed(11008, 16)
               and r3_vp19 <= to_signed(20768, 16) then
                s4 := '1';
            end if;
            if r3_vm73 >= s_c5l and r3_vm73 <= s_c5h
               and r3_vp19 >= to_signed(20768, 16) then
                s5 := '1';
            end if;
            if r3_vm73 >= s_c5ul and r3_vm73 <= s_c5uh then
                s5u := '1';
            end if;

            v_org := s1 or s1d or s2 or s3 or s4 or s5 or s5u;

            -- strict compare: a zero halftone radius draws nothing
            v_red := '0';
            if (r3_hx1 = '1' and r3_h1 < resize(r3_hr, 16))
               or (r3_hx2 = '1' and r3_h2 < resize(r3_hr, 16)) then
                v_red := '1';
            end if;

            if v_red = '1' then
                r4_code <= "10";
            elsif v_org = '1' then
                r4_code <= "01";
            else
                r4_code <= "00";
            end if;

            -- weave hash round 3
            v_hk  := r3_hs xor shift_right(r3_hs, 9);
            v_h2  := v_hk + shift_left(v_hk, 5);
            r4_wh <= v_h2(11 downto 8);
        end if;
    end process p_r4;

    ------------------------------------------------------------------------
    -- R5: paint from the line's pre-faded palette; Weave grain (scaled by
    -- the line's fog); neutral outside active video.
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_d  : signed(4 downto 0);
        variable v_dd : signed(7 downto 0);
        variable v_ys : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            case to_integer(r4_code) is
                when 1 =>
                    v_y := c_or_y; v_u := c_or_u; v_v := c_or_v;
                when 2 =>
                    v_y := c_re_y; v_u := c_re_u; v_v := c_re_v;
                when others =>
                    v_y := c_bg_y; v_u := c_bg_u; v_v := c_bg_v;
            end case;

            if s_sw(1) = '1' then
                v_d := signed('0' & r4_wh) - to_signed(8, 5);
                case to_integer(s_wamp) is
                    when 0      => v_dd := (others => '0');
                    when 1      => v_dd := resize(v_d, 8);
                    when 2      => v_dd := shift_left(resize(v_d, 8), 1);
                    when 3      => v_dd := shift_left(resize(v_d, 8), 1) + resize(v_d, 8);
                    when others => v_dd := shift_left(resize(v_d, 8), 2);
                end case;
                v_ys := signed(resize(v_y, 12)) + resize(v_dd, 12);
                if v_ys < 64 then
                    v_ys := to_signed(64, 12);
                end if;
                v_y := unsigned(v_ys(9 downto 0));
            end if;

            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= to_unsigned(512, 10);
                s_out_v <= to_unsigned(512, 10);
            else
                s_out_y <= v_y;
                s_out_u <= v_u;
                s_out_v <= v_v;
            end if;
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    s_io_align_in.y       <= std_logic_vector(s_out_y);
    s_io_align_in.u       <= std_logic_vector(s_out_u);
    s_io_align_in.v       <= std_logic_vector(s_out_v);
    s_io_align_in.avid    <= s_avid_sr(C_LATENCY - 1);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_LATENCY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (6 -> 8).
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

end architecture overlook;
