-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- orbifold.vhd  (v0.3 — iterated fract-ring rosette fractal)
--
-- v0.3.3 (2026-10-17): resolution scaling.  The pixel/line counters (and the
-- measured width/height that centre the zoom) run in virtual 1080-line units,
-- stepped per pixel / per field line from the video timing ID
-- (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p, and at
-- SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame line
-- (doubled per interlaced field line).  The rosette cells and the hue
-- gradient now follow screen height and stay round on a 4:3 SD screen.  The
-- duplicate width/height counters are folded into the accumulators.  Also
-- (bit-identical): the S17 chroma sum no longer adds a sign-extension net to
-- itself (SB_CARRY I0 == I1, the router1 livelock shape).
--
-- The whole image is two shader lines iterated:
--     uv = fract(uv * divisions);
--     if (t < length(uv) && length(uv) < t + w)  break;   // draw ring, stop
-- Rings around every lattice corner, folded across a few scales, make the
-- interlocking scallop / quatrefoil rosettes.
--
-- Hardware form (streaming, no BRAM, no line buffer):
--   * divisions is locked to 2 per level, so every fract(uv*2) is a BIT-SLICE
--     of one base coordinate — NLEV=6 depth levels come for free as six
--     8-bit slices of the same 17-bit zoomed coordinate.
--   * length() per level: r2 = x^2 + y^2 with the squares decomposed as
--     (16h+l)^2 = (h^2 & l^2) + 32*h*l.  The other shapes are linear
--     distances (max / sum / min combos of |x|,|y|, pre-scaled to 8 bits)
--     fed through the same squarer with y = 0, so every shape shares one
--     squared-domain ring test.
--   * the shallowest level that hits wins (= the shader's break), or the
--     hit parity in XOR mode.
--   * zoom: one 13x12 multiply per axis (four 13x3 partial products), the
--     mantissa an exp2 table so the zoom rate is constant across the octave,
--     plus a 0..3 octave barrel shift.  Ring identity is the continuous
--     "size class" c = level + E, so hue, pulse and orbit phases stay
--     continuous through the drift wrap; the coarsest level thins out and the
--     finest thins in across the last quarter octave before the wrap.
--   * a vblank sequencer with one serial shift-add multiplier computes the
--     per-level ring thresholds (radius pulse, wrap fade, shape scale,
--     squared), orbit offsets, hues, and the zoom mantissa.
--
-- Controls: K1 Shape, K2 Radius, K3 Thickness, K4 Depth (1..6), K5 Scale
-- (0..3 octaves), K6 Zoom (bipolar drift, centre = still), S7 ring centres
-- Corners/Middles, S8 Rings/Discs, S9 Video Off/Weight (input luma fattens
-- the rings: a fractal halftone), S10 Overlap Stack/XOR, S11 palette
-- Rainbow/Phosphor, P12 Animate (KEY): per-level orbit + radius pulse
-- rippling down through the scales; 100% = rosettes swing half a cell and
-- collapse/swell.
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

architecture orbifold of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY : natural := 18;   -- stage regs s0..s17; s_io pairs pipe(LATENCY-1)
    constant NLEV    : natural := 6;    -- fract depth levels

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- exp2 mantissa: 4096*2^(k/16) - 4096, and the step to the next entry
    type t_expl is array (0 to 15) of unsigned(11 downto 0);
    constant EXP_L : t_expl := (
        to_unsigned(   0, 12), to_unsigned( 181, 12), to_unsigned( 371, 12), to_unsigned( 568, 12),
        to_unsigned( 775, 12), to_unsigned( 991, 12), to_unsigned(1216, 12), to_unsigned(1451, 12),
        to_unsigned(1697, 12), to_unsigned(1953, 12), to_unsigned(2221, 12), to_unsigned(2501, 12),
        to_unsigned(2793, 12), to_unsigned(3098, 12), to_unsigned(3416, 12), to_unsigned(3749, 12));
    type t_expd is array (0 to 15) of unsigned(8 downto 0);
    constant EXP_D : t_expd := (
        to_unsigned(181, 9), to_unsigned(190, 9), to_unsigned(197, 9), to_unsigned(207, 9),
        to_unsigned(216, 9), to_unsigned(225, 9), to_unsigned(235, 9), to_unsigned(246, 9),
        to_unsigned(256, 9), to_unsigned(268, 9), to_unsigned(280, 9), to_unsigned(292, 9),
        to_unsigned(305, 9), to_unsigned(318, 9), to_unsigned(333, 9), to_unsigned(347, 9));

    -- quarter-wave sine, 127 * sin((j+0.5) * pi/128)
    type t_sinq is array (0 to 63) of unsigned(6 downto 0);
    constant SIN_Q : t_sinq := (
        to_unsigned(  2, 7), to_unsigned(  5, 7), to_unsigned(  8, 7), to_unsigned( 11, 7),
        to_unsigned( 14, 7), to_unsigned( 17, 7), to_unsigned( 20, 7), to_unsigned( 23, 7),
        to_unsigned( 26, 7), to_unsigned( 29, 7), to_unsigned( 32, 7), to_unsigned( 35, 7),
        to_unsigned( 38, 7), to_unsigned( 41, 7), to_unsigned( 44, 7), to_unsigned( 47, 7),
        to_unsigned( 50, 7), to_unsigned( 53, 7), to_unsigned( 56, 7), to_unsigned( 58, 7),
        to_unsigned( 61, 7), to_unsigned( 64, 7), to_unsigned( 67, 7), to_unsigned( 69, 7),
        to_unsigned( 72, 7), to_unsigned( 74, 7), to_unsigned( 77, 7), to_unsigned( 79, 7),
        to_unsigned( 82, 7), to_unsigned( 84, 7), to_unsigned( 86, 7), to_unsigned( 89, 7),
        to_unsigned( 91, 7), to_unsigned( 93, 7), to_unsigned( 95, 7), to_unsigned( 97, 7),
        to_unsigned( 99, 7), to_unsigned(101, 7), to_unsigned(103, 7), to_unsigned(105, 7),
        to_unsigned(106, 7), to_unsigned(108, 7), to_unsigned(110, 7), to_unsigned(111, 7),
        to_unsigned(113, 7), to_unsigned(114, 7), to_unsigned(115, 7), to_unsigned(117, 7),
        to_unsigned(118, 7), to_unsigned(119, 7), to_unsigned(120, 7), to_unsigned(121, 7),
        to_unsigned(122, 7), to_unsigned(123, 7), to_unsigned(124, 7), to_unsigned(124, 7),
        to_unsigned(125, 7), to_unsigned(125, 7), to_unsigned(126, 7), to_unsigned(126, 7),
        to_unsigned(127, 7), to_unsigned(127, 7), to_unsigned(127, 7), to_unsigned(127, 7));

    -- triangle wave 0..254 over an 8-bit phase (cheap hue wheel)
    function tri8(a : unsigned(7 downto 0)) return unsigned is
    begin
        if a(7) = '0' then return a(6 downto 0) & '0';
        else               return (not a(6 downto 0)) & '0';
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
    --------------------------------------------------------------------------
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

    --------------------------------------------------------------------------
    -- Position counters + measured active area (centre of zoom)
    --------------------------------------------------------------------------
    -- Virtual 1080-line units: pix_xa / late_xa are Q.1 (steps 2/3/4),
    -- pix_ya Q.8 per field line; pixel_x / late_x / pixel_y are the integer
    -- parts (at 1080p the step is 1 and these are the old integer counters).
    -- measured_h / measured_v are the virtual width / height.
    signal pixel_x, pixel_y : unsigned(11 downto 0) := (others => '0');
    signal late_x           : unsigned(11 downto 0) := (others => '0');
    signal s_xs             : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys             : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal pix_xa, late_xa  : unsigned(12 downto 0) := (others => '0');
    signal pix_ya           : unsigned(19 downto 0) := (others => '0');
    signal prev_hsync_n     : std_logic := '1';
    signal prev_vsync_n     : std_logic := '1';
    signal line_had_avid    : std_logic := '0';
    signal measured_h       : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal measured_v       : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_cx, s_cy       : unsigned(11 downto 0) := (others => '0');
    signal saw_avid         : std_logic := '0';
    signal seq_go           : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame parameters (written by the vblank sequencer)
    --------------------------------------------------------------------------
    signal s_dmax     : integer range 0 to NLEV - 1 := NLEV - 1;
    signal s_center   : std_logic := '1';                     -- S7 fold to cell centres
    signal s_fill     : std_logic := '0';                     -- S8 discs
    signal s_vid      : std_logic := '0';                     -- S9 video weight
    signal s_xor      : std_logic := '0';                     -- S10 XOR overlap
    signal s_mono     : std_logic := '0';                     -- S11 phosphor palette
    signal s_vg       : unsigned(1 downto 0) := "10";         -- video gain 4/5/6 -> 0/1/2
    signal s_oct      : unsigned(1 downto 0) := "01";
    signal s_f        : unsigned(11 downto 0) := (others => '0');   -- zoom mantissa - 1, Q.12

    type t_q3  is array (0 to NLEV - 1) of unsigned(2 downto 0);
    type t_q8  is array (0 to NLEV - 1) of unsigned(7 downto 0);
    type t_q9  is array (0 to NLEV - 1) of unsigned(8 downto 0);
    type t_q10 is array (0 to NLEV - 1) of unsigned(9 downto 0);
    type t_q16 is array (0 to NLEV - 1) of unsigned(15 downto 0);
    type t_q17 is array (0 to NLEV - 1) of unsigned(16 downto 0);
    type t_q18 is array (0 to NLEV - 1) of unsigned(17 downto 0);
    type t_q20 is array (0 to NLEV - 1) of unsigned(19 downto 0);
    type t_s19 is array (0 to NLEV - 1) of signed(18 downto 0);

    -- per-level tables (shift chains, filled level 0 first)
    signal lv_lo2, lv_hi2 : t_q20 := (others => to_unsigned(60025, 20));
    signal lv_ox, lv_oy   : t_q8  := (others => (others => '0'));
    signal lv_hue         : t_q8  := (others => (others => '0'));
    signal lv_y           : t_q10 := (others => to_unsigned(640, 10));
    signal lv_shp         : t_q3  := (others => (others => '0'));

    --------------------------------------------------------------------------
    -- Vblank sequencer
    --------------------------------------------------------------------------
    signal q_st   : integer range 0 to 63 := 0;
    signal mul_a   : unsigned(15 downto 0) := (others => '0');
    signal mul_b   : unsigned(9 downto 0)  := (others => '0');
    signal mul_acc : unsigned(25 downto 0) := (others => '0');
    signal mul_cnt : integer range 0 to 9 := 0;
    signal mul_run : std_logic := '0';

    signal sin_ph  : unsigned(7 downto 0) := (others => '0');
    signal sin_mag : unsigned(6 downto 0) := (others => '0');
    signal sin_neg : std_logic := '0';

    -- free-running state: zoom drift (one octave = 2^20), animation clocks,
    -- glided P12 (Q10.3)
    signal zphase  : unsigned(19 downto 0) := x"80000";
    signal orb_t   : unsigned(15 downto 0) := (others => '0');
    signal pul_t   : unsigned(15 downto 0) := (others => '0');
    signal q_ag    : unsigned(12 downto 0) := (others => '0');

    signal q_k1, q_k2, q_k3, q_k4, q_k5, q_k6, q_p12 : unsigned(9 downto 0) := (others => '0');
    signal q_sw    : std_logic_vector(4 downto 0) := (others => '0');
    signal q_shape : unsigned(2 downto 0) := (others => '0');
    signal q_d6    : unsigned(12 downto 0) := (others => '0');
    signal q_t     : unsigned(9 downto 0) := (others => '0');
    signal q_w, q_hw : unsigned(9 downto 0) := (others => '0');
    signal q_v5    : unsigned(9 downto 0) := (others => '0');
    signal q_dz    : signed(10 downto 0) := (others => '0');
    signal q_mag   : unsigned(9 downto 0) := (others => '0');
    signal q_vv    : unsigned(9 downto 0) := (others => '0');
    signal q_zin   : std_logic := '0';
    signal q_gd    : signed(13 downto 0) := (others => '0');
    signal q_s     : unsigned(13 downto 0) := (others => '0');
    signal q_e     : unsigned(13 downto 0) := (others => '0');
    signal q_lf    : unsigned(11 downto 0) := (others => '0');
    signal q_ld    : unsigned(8 downto 0) := (others => '0');
    signal q_pulE, q_orbE, q_hueE : unsigned(7 downto 0) := (others => '0');
    signal q_pa    : unsigned(7 downto 0) := (others => '0');
    signal q_oa    : unsigned(6 downto 0) := (others => '0');
    signal q_speed : unsigned(13 downto 0) := (others => '0');
    signal q_f0, q_fN, q_fmin, q_fd : unsigned(8 downto 0) := (others => '0');
    signal q_lvl   : integer range 0 to NLEV - 1 := 0;
    signal q_hacc  : unsigned(7 downto 0) := (others => '0');
    signal q_yacc  : unsigned(9 downto 0) := (others => '0');
    signal q_pph, q_oph : unsigned(7 downto 0) := (others => '0');
    signal q_shp   : unsigned(2 downto 0) := (others => '0');
    signal q_k     : unsigned(8 downto 0) := (others => '0');
    signal q_neg   : std_logic := '0';
    signal q_dt    : unsigned(8 downto 0) := (others => '0');
    signal q_ti    : unsigned(9 downto 0) := (others => '0');
    signal q_c     : unsigned(10 downto 0) := (others => '0');
    signal q_h     : unsigned(8 downto 0) := (others => '0');
    signal q_lo, q_hi   : unsigned(9 downto 0) := (others => '0');
    signal q_lok, q_hik : unsigned(9 downto 0) := (others => '0');
    signal q_lo2, q_hi2 : unsigned(19 downto 0) := (others => '0');
    signal q_ox    : unsigned(7 downto 0) := (others => '0');
    signal q_rate  : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    type t_pp is array (0 to 3) of signed(16 downto 0);

    -- S0: centred coords
    signal s0_dx, s0_dy : signed(12 downto 0) := (others => '0');
    -- S1: zoom partial products, dx * f in four 3-bit chunks
    signal s1_px, s1_py : t_pp := (others => (others => '0'));
    signal s1_dx, s1_dy : signed(12 downto 0) := (others => '0');
    -- S2/S3: partial sums
    signal s2_ax, s2_bx, s2_ay, s2_by : signed(20 downto 0) := (others => '0');
    signal s2_dx, s2_dy : signed(12 downto 0) := (others => '0');
    signal s3_ax, s3_bx, s3_ay, s3_by : signed(20 downto 0) := (others => '0');
    -- S4: zoomed base coords (mod 2^17 — the coarsest cell period)
    signal s4_u, s4_v : unsigned(16 downto 0) := (others => '0');
    -- S5: octave-shifted coords
    signal s5_u, s5_v : unsigned(16 downto 0) := (others => '0');
    -- S6: per-level fract slices + orbit offset
    signal s6_qx, s6_qy : t_q8 := (others => (others => '0'));
    -- S7: folded |x|,|y|
    signal s7_ax, s7_ay : t_q8 := (others => (others => '0'));
    -- S8: max / min / sum
    signal s8_mx, s8_mn : t_q8 := (others => (others => '0'));
    signal s8_s         : t_q9 := (others => (others => '0'));
    -- S9: shape terms
    signal s9_mx, s9_mn, s9_m7, s9_sh : t_q8 := (others => (others => '0'));
    signal s9_t4        : t_q10 := (others => (others => '0'));
    -- S10: squarer inputs
    signal s10_qa, s10_qb : t_q8 := (others => (others => '0'));
    -- S11/S12: squarer
    -- 8-bit squares: one 256 x 16 ROM per squarer (12 EBRs; they were
    -- 4-bit-square + h*l decompositions in fabric).  The ROM's output register is
    -- the S11 stage; S12 re-registers it into fabric before the S13 add.
    type t_sq8 is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_sq8 return t_sq8 is
        variable v : t_sq8;
    begin
        for i in 0 to 255 loop
            v(i) := std_logic_vector(to_unsigned(i * i, 16));
        end loop;
        return v;
    end function;
    constant C_SQ8 : t_sq8 := f_sq8;
    signal s11_sa, s11_sb : t_q16 := (others => (others => '0'));
    signal s12_a2, s12_b2 : t_q16 := (others => (others => '0'));
    signal s12_yn         : unsigned(9 downto 0) := (others => '0');
    -- S13: r^2, video weight, gradient
    signal s13_r2   : t_q17 := (others => (others => '0'));
    signal s13_V    : unsigned(15 downto 0) := (others => '0');
    signal s13_gdx, s13_gdy : signed(12 downto 0) := (others => '0');
    -- S14: video-adjusted radii
    signal s14_rp   : t_q18 := (others => (others => '0'));
    signal s14_rm   : t_s19 := (others => (others => '0'));
    signal s14_gax, s14_gay : unsigned(11 downto 0) := (others => '0');
    -- S15: ring hits
    signal s15_hit  : std_logic_vector(0 to NLEV - 1) := (others => '0');
    signal s15_g    : unsigned(7 downto 0) := (others => '0');
    -- S16: winner
    signal s16_hit  : std_logic := '0';
    signal s16_hue  : unsigned(7 downto 0) := (others => '0');
    signal s16_y    : unsigned(9 downto 0) := (others => '0');
    -- S17: colour
    signal s17_hit           : std_logic := '0';
    signal s17_y, s17_u, s17_v : unsigned(9 downto 0) := (others => '0');

    -- output
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Position counters, active-area measurement, frame trigger.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- per-mode virtual steps (the timing ID is static while a mode
            -- runs; decoded every clock so the first field is right too)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- pixel/line counters (active lines only, so the centre is right);
            -- the end-of-line / end-of-field values are the measured active
            -- width / height (centre of the zoom), in virtual units
            if v_h_edge = '1' then
                if pix_xa(12 downto 1) > 0 then measured_h <= pix_xa(12 downto 1); end if;
                pix_xa <= (others => '0');
                if line_had_avid = '1' then pix_ya <= pix_ya + s_ys; end if;
                line_had_avid <= '0';
            elsif data_in.avid = '1' then
                pix_xa <= pix_xa + s_xs;
                line_had_avid <= '1';
            end if;

            -- late column counter for the screen-space hue gradient
            if pipe(12).avid = '1' then
                late_xa <= late_xa + s_xs;
            else
                late_xa <= (others => '0');
            end if;

            -- frame trigger: once per field (analog vsync serrates — act only
            -- on the first edge after active video)
            seq_go <= '0';
            if data_in.avid = '1' then saw_avid <= '1'; end if;
            if v_v_edge = '1' then
                if pix_ya(19 downto 8) > 0 then measured_v <= pix_ya(19 downto 8); end if;
                pix_ya <= (others => '0');
                s_cx <= '0' & measured_h(11 downto 1);
                s_cy <= '0' & measured_v(11 downto 1);
                if saw_avid = '1' then
                    saw_avid <= '0';
                    seq_go   <= '1';
                end if;
            end if;
        end if;
    end process p_position;

    pixel_x <= pix_xa(12 downto 1);
    late_x  <= late_xa(12 downto 1);
    pixel_y <= pix_ya(19 downto 8);

    --------------------------------------------------------------------------
    -- Sine ROM (registered; valid one state after sin_ph is set)
    --------------------------------------------------------------------------
    p_sin : process(clk)
        variable v_j : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            v_j := sin_ph(5 downto 0);
            if sin_ph(6) = '1' then v_j := not v_j; end if;
            sin_mag <= SIN_Q(to_integer(v_j));
            sin_neg <= sin_ph(7);
        end if;
    end process p_sin;

    --------------------------------------------------------------------------
    -- Vblank sequencer: one op per state, one serial 16x10 multiplier
    -- (MSB-first shift-add, 10 cycles; the sequencer stalls while it runs).
    -- ~700 cycles per field, well inside every mode's vertical blank.
    --------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_k4 : unsigned(9 downto 0);
        variable v_m  : unsigned(6 downto 0);
        variable v_dz : signed(10 downto 0);
        variable v_ag : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            if mul_run = '1' then
                if mul_b(9) = '1' then
                    mul_acc <= shift_left(mul_acc, 1) + resize(mul_a, 26);
                else
                    mul_acc <= shift_left(mul_acc, 1);
                end if;
                mul_b <= shift_left(mul_b, 1);
                if mul_cnt = 9 then
                    mul_run <= '0';
                else
                    mul_cnt <= mul_cnt + 1;
                end if;
            else
                case q_st is
                    when 0 =>
                        if seq_go = '1' then q_st <= 1; end if;

                    -- latch controls
                    when 1 =>
                        q_k1  <= unsigned(registers_in(0));
                        q_k2  <= unsigned(registers_in(1));
                        q_k3  <= unsigned(registers_in(2));
                        q_k4  <= unsigned(registers_in(3));
                        q_k5  <= unsigned(registers_in(4));
                        q_k6  <= unsigned(registers_in(5));
                        q_sw  <= registers_in(6)(4 downto 0);
                        q_p12 <= unsigned(registers_in(7));
                        q_st  <= 2;

                    when 2 =>
                        -- labelled knobs: firmware loads the default as the
                        -- label INDEX, so small values are indexes
                        if q_k1 < 8 then q_shape <= q_k1(2 downto 0);
                        else             q_shape <= q_k1(9 downto 7); end if;
                        q_d6 <= shift_left(resize(q_k4, 13), 2) + shift_left(resize(q_k4, 13), 1);
                        q_t  <= '0' & q_k2(9 downto 1);                       -- 0..511
                        q_w  <= resize(q_k3(9 downto 2), 10) + 2;             -- 2..257
                        q_v5 <= not q_k5;                                     -- Scale: up = bigger
                        q_dz <= signed(resize(q_k6, 11)) - to_signed(512, 11);
                        q_gd <= signed(resize(q_p12 & "000", 14)) - signed(resize(q_ag, 14));
                        s_center <= q_sw(0);
                        s_fill   <= q_sw(1);
                        s_vid    <= q_sw(2);
                        s_xor    <= q_sw(3);
                        s_mono   <= q_sw(4);
                        q_st <= 3;

                    when 3 =>
                        v_k4 := q_k4;
                        if v_k4 < 6 then s_dmax <= to_integer(v_k4(2 downto 0));
                        else             s_dmax <= to_integer(q_d6(12 downto 10)); end if;
                        -- Scale: 0..3 octaves (x12 in Q.12 per 1/1024)
                        q_s  <= shift_left(resize(q_v5, 14), 3) + shift_left(resize(q_v5, 14), 2);
                        v_dz := -q_dz;
                        if q_dz(10) = '1' then
                            q_mag <= unsigned(v_dz(9 downto 0));      -- slice, not resize (512 fits)
                            q_zin <= '0';
                        else
                            q_mag <= unsigned(q_dz(9 downto 0));
                            q_zin <= '1';
                        end if;
                        q_hw <= '0' & q_w(9 downto 1);
                        v_ag := signed(resize(q_ag, 14)) + shift_right(q_gd, 3);
                        q_ag <= unsigned(v_ag(12 downto 0));
                        q_st <= 4;

                    when 4 =>
                        -- total zoom exponent E (Q2.12 octaves)
                        q_e <= q_s + resize(zphase(19 downto 8), 14);
                        if q_mag > 16 then q_vv <= q_mag - 16;                -- deadband
                        else               q_vv <= (others => '0'); end if;
                        q_st <= 5;

                    when 5 =>
                        q_lf  <= EXP_L(to_integer(q_e(11 downto 8)));
                        q_ld  <= EXP_D(to_integer(q_e(11 downto 8)));
                        s_oct <= q_e(13 downto 12);
                        q_pulE <= q_e(13 downto 6);                           -- 64*E
                        q_orbE <= q_e(13 downto 6) + ('0' & q_e(13 downto 7)); -- 96*E
                        q_pa  <= q_ag(12 downto 5);                           -- P12/4
                        q_oa  <= q_ag(12 downto 6);                           -- P12/8
                        q_st  <= 6;

                    when 6 =>                                                 -- D * interp
                        mul_a <= resize(q_ld, 16);
                        mul_b <= "00" & q_e(7 downto 0);
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 7;

                    when 7 =>
                        s_f   <= q_lf + resize(mul_acc(16 downto 8), 12);
                        mul_a <= resize(q_e, 16);                             -- E * 43
                        mul_b <= to_unsigned(43, 10);
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 8;

                    when 8 =>
                        q_hueE <= mul_acc(19 downto 12);
                        mul_a <= resize(q_vv, 16);                            -- speed = vv^2/16
                        mul_b <= q_vv;
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 9;

                    when 9 =>
                        q_speed <= mul_acc(17 downto 4);
                        -- wrap fades (Q.8): coarsest thins as phi -> 0,
                        -- finest thins as phi -> 1, over a quarter octave
                        if zphase(19 downto 18) /= "00" then q_f0 <= to_unsigned(256, 9);
                        else q_f0 <= '0' & zphase(17 downto 10); end if;
                        if zphase(19 downto 18) /= "11" then q_fN <= to_unsigned(256, 9);
                        else q_fN <= '0' & (not zphase(17 downto 10)); end if;
                        q_hacc <= q_hueE;
                        q_yacc <= to_unsigned(680, 10) - resize(zphase(19 downto 16), 10);
                        q_pph  <= pul_t(15 downto 8) - q_pulE;
                        q_oph  <= orb_t(15 downto 8) + q_orbE;
                        q_lvl  <= 0;
                        q_st   <= 10;

                    when 10 =>
                        if q_f0 < q_fN then q_fmin <= q_f0; else q_fmin <= q_fN; end if;
                        case q_shape is
                            when "000"                 => s_vg <= "10";
                            when "001" | "010" | "011" => s_vg <= "01";
                            when "111"                 => s_vg <= "01";
                            when others                => s_vg <= "00";
                        end case;
                        q_st <= 20;

                    ----------------------------------------------------------
                    -- per-level loop
                    ----------------------------------------------------------
                    when 20 =>
                        sin_ph <= q_pph;
                        if q_shape = "111" then                 -- Mixed: circle / star
                            if q_lvl mod 2 = 1 then q_shp <= "100";
                            else                    q_shp <= "000"; end if;
                        else
                            q_shp <= q_shape;
                        end if;
                        q_st <= 21;

                    when 21 =>
                        -- shape scale (Q.8): circle 1, square/diamond/octagon
                        -- 1/sqrt2, star/sparkle/cross 1/2
                        case q_shp is
                            when "000"                 => q_k <= to_unsigned(256, 9);
                            when "001" | "010" | "011" => q_k <= to_unsigned(181, 9);
                            when others                => q_k <= to_unsigned(128, 9);
                        end case;
                        if s_dmax = 0 then          q_fd <= q_fmin;
                        elsif q_lvl = 0 then        q_fd <= q_f0;
                        elsif q_lvl = s_dmax then   q_fd <= q_fN;
                        else                        q_fd <= to_unsigned(256, 9);
                        end if;
                        q_st <= 22;

                    when 22 =>                                  -- pulse: pa * sin
                        q_neg <= sin_neg;
                        mul_a <= resize(q_pa, 16);
                        mul_b <= resize(sin_mag, 10);
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 23;

                    when 23 =>
                        q_dt <= mul_acc(15 downto 7);
                        q_st <= 24;

                    when 24 =>
                        if q_neg = '1' then
                            if q_t > resize(q_dt, 10) then q_ti <= q_t - resize(q_dt, 10);
                            else                            q_ti <= (others => '0'); end if;
                        else
                            q_ti <= q_t + resize(q_dt, 10);
                        end if;
                        q_st <= 25;

                    when 25 =>                                  -- fade the band
                        if s_fill = '1' then
                            mul_a <= resize(q_ti, 16) + resize(q_w, 16);
                        else
                            mul_a <= resize(q_hw, 16);
                        end if;
                        mul_b <= '0' & q_fd;
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_c <= resize(q_ti, 11) + resize(q_hw, 11);
                        q_st <= 26;

                    when 26 =>
                        if s_fill = '1' then
                            q_lo <= (others => '0');
                            q_hi <= mul_acc(17 downto 8);
                        else
                            q_h  <= mul_acc(16 downto 8);
                        end if;
                        q_st <= 27;

                    when 27 =>
                        if s_fill = '0' then
                            q_lo <= resize(q_c - resize(q_h, 11), 10);
                            q_hi <= resize(q_c + resize(q_h, 11), 10);
                        end if;
                        q_st <= 28;

                    when 28 =>                                  -- shape scale
                        mul_a <= resize(q_lo, 16);
                        mul_b <= '0' & q_k;
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 29;

                    when 29 =>
                        q_lok <= mul_acc(17 downto 8);
                        mul_a <= resize(q_hi, 16);
                        mul_b <= '0' & q_k;
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 30;

                    when 30 =>
                        q_hik <= mul_acc(17 downto 8);
                        mul_a <= resize(q_lok, 16);             -- lo^2
                        mul_b <= q_lok;
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 31;

                    when 31 =>
                        q_lo2 <= mul_acc(19 downto 0);
                        mul_a <= resize(q_hik, 16);             -- hi^2
                        mul_b <= q_hik;
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 32;

                    when 32 =>
                        q_hi2  <= mul_acc(19 downto 0);
                        sin_ph <= q_oph + to_unsigned(64, 8);   -- cos
                        q_st <= 33;

                    when 33 =>
                        q_st <= 34;

                    when 34 =>                                  -- orbit x
                        q_neg <= sin_neg;
                        mul_a <= resize(q_oa, 16);
                        mul_b <= resize(sin_mag, 10);
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        sin_ph <= q_oph;                        -- sin
                        q_st <= 35;

                    when 35 =>
                        v_m := mul_acc(13 downto 7);
                        if q_neg = '1' then q_ox <= to_unsigned(0, 8) - resize(v_m, 8);
                        else                q_ox <= resize(v_m, 8); end if;
                        q_neg <= sin_neg;                       -- orbit y
                        mul_a <= resize(q_oa, 16);
                        mul_b <= resize(sin_mag, 10);
                        mul_acc <= (others => '0'); mul_cnt <= 0; mul_run <= '1';
                        q_st <= 36;

                    when 36 =>
                        v_m := mul_acc(13 downto 7);
                        for i in 0 to NLEV - 2 loop
                            lv_lo2(i) <= lv_lo2(i + 1);
                            lv_hi2(i) <= lv_hi2(i + 1);
                            lv_ox(i)  <= lv_ox(i + 1);
                            lv_oy(i)  <= lv_oy(i + 1);
                            lv_hue(i) <= lv_hue(i + 1);
                            lv_y(i)   <= lv_y(i + 1);
                            lv_shp(i) <= lv_shp(i + 1);
                        end loop;
                        lv_lo2(NLEV - 1) <= q_lo2;
                        lv_hi2(NLEV - 1) <= q_hi2;
                        lv_ox(NLEV - 1)  <= q_ox;
                        if q_neg = '1' then lv_oy(NLEV - 1) <= to_unsigned(0, 8) - resize(v_m, 8);
                        else                lv_oy(NLEV - 1) <= resize(v_m, 8); end if;
                        lv_hue(NLEV - 1) <= q_hacc;
                        lv_y(NLEV - 1)   <= q_yacc;
                        lv_shp(NLEV - 1) <= q_shp;
                        q_hacc <= q_hacc + to_unsigned(43, 8);
                        q_yacc <= q_yacc - to_unsigned(16, 10);
                        q_pph  <= q_pph - to_unsigned(64, 8);
                        q_oph  <= q_oph + to_unsigned(96, 8);
                        if q_lvl = NLEV - 1 then
                            q_st <= 40;
                        else
                            q_lvl <= q_lvl + 1;
                            q_st <= 20;
                        end if;

                    ----------------------------------------------------------
                    -- advance the free-running clocks for the next field
                    ----------------------------------------------------------
                    when 40 =>
                        if q_zin = '1' then zphase <= zphase - resize(q_speed, 20);  -- zoom in
                        else                zphase <= zphase + resize(q_speed, 20); end if;
                        q_rate <= to_unsigned(48, 10) + resize(q_ag(12 downto 4), 10);
                        q_st <= 41;

                    when 41 =>
                        orb_t <= orb_t + resize(q_rate, 16);
                        pul_t <= pul_t + resize(q_rate, 16);
                        q_st <= 42;

                    when 42 =>
                        pul_t <= pul_t + resize(q_rate(9 downto 1), 16);   -- pulse 1.5x orbit
                        q_st <= 0;

                    when others =>
                        q_st <= 0;
                end case;
            end if;
        end if;
    end process p_seq;

    --------------------------------------------------------------------------
    -- Main per-pixel pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_u21      : signed(20 downto 0);
        variable v_q        : unsigned(7 downto 0);
        variable v_f        : unsigned(7 downto 0);
        variable v_a, v_b   : unsigned(7 downto 0);
        variable v_t10      : unsigned(9 downto 0);
        variable v_yn       : signed(10 downto 0);
        variable v_hit      : std_logic;
        variable v_par      : std_logic;
        variable v_sel      : integer range 0 to NLEV - 1;
        variable v_tu, v_tv : unsigned(7 downto 0);
        variable v_g13      : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- S0: centred pixel coords
            s0_dx <= signed(resize(pixel_x, 13)) - signed(resize(s_cx, 13));
            s0_dy <= signed(resize(pixel_y, 13)) - signed(resize(s_cy, 13));

            -- S1: dx * f as four 13x3 partial products
            for k in 0 to 3 loop
                s1_px(k) <= s0_dx * signed('0' & s_f(3 * k + 2 downto 3 * k));
                s1_py(k) <= s0_dy * signed('0' & s_f(3 * k + 2 downto 3 * k));
            end loop;
            s1_dx <= s0_dx;
            s1_dy <= s0_dy;

            -- S2: a = pp0 + pp1<<3, b = pp2 + pp3<<3  (all mod 2^21)
            s2_ax <= resize(s1_px(0), 21) + shift_left(resize(s1_px(1), 21), 3);
            s2_bx <= resize(s1_px(2), 21) + shift_left(resize(s1_px(3), 21), 3);
            s2_ay <= resize(s1_py(0), 21) + shift_left(resize(s1_py(1), 21), 3);
            s2_by <= resize(s1_py(2), 21) + shift_left(resize(s1_py(3), 21), 3);
            s2_dx <= s1_dx;
            s2_dy <= s1_dy;

            -- S3: b += dx<<6  (the implicit leading 1 of the mantissa)
            s3_ax <= s2_ax;
            s3_bx <= s2_bx + shift_left(resize(s2_dx, 21), 6);
            s3_ay <= s2_ay;
            s3_by <= s2_by + shift_left(resize(s2_dy, 21), 6);

            -- S4: u = dx*(4096+f) = a + b<<6, keep bits 20..4 (px*256 units)
            v_u21 := s3_ax + shift_left(s3_bx, 6);
            s4_u  <= unsigned(v_u21(20 downto 4));
            v_u21 := s3_ay + shift_left(s3_by, 6);
            s4_v  <= unsigned(v_u21(20 downto 4));

            -- S5: octave barrel shift mod 2^17
            s5_u <= shift_left(s4_u, to_integer(s_oct));
            s5_v <= shift_left(s4_v, to_integer(s_oct));

            -- S6: per-level fract slices (fract(uv*2^i) = fixed bit-slice),
            -- each level's lattice shifted by its orbit offset
            for i in 0 to NLEV - 1 loop
                s6_qx(i) <= s5_u(16 - i downto 9 - i) + lv_ox(i);
                s6_qy(i) <= s5_v(16 - i downto 9 - i) + lv_oy(i);
            end loop;

            -- S7: optional fold to distance-from-cell-centre
            for i in 0 to NLEV - 1 loop
                v_q := s6_qx(i);
                if s_center = '1' then
                    if v_q(7) = '1' then v_f := v_q - to_unsigned(128, 8);
                    else                 v_f := to_unsigned(128, 8) - v_q; end if;
                    if v_f(7) = '1' then v_q := (others => '1');       -- 128 -> 255
                    else                 v_q := v_f(6 downto 0) & '0'; end if;
                end if;
                s7_ax(i) <= v_q;

                v_q := s6_qy(i);
                if s_center = '1' then
                    if v_q(7) = '1' then v_f := v_q - to_unsigned(128, 8);
                    else                 v_f := to_unsigned(128, 8) - v_q; end if;
                    if v_f(7) = '1' then v_q := (others => '1');
                    else                 v_q := v_f(6 downto 0) & '0'; end if;
                end if;
                s7_ay(i) <= v_q;
            end loop;

            -- S8: max, min, sum
            for i in 0 to NLEV - 1 loop
                if s7_ax(i) >= s7_ay(i) then
                    s8_mx(i) <= s7_ax(i); s8_mn(i) <= s7_ay(i);
                else
                    s8_mx(i) <= s7_ay(i); s8_mn(i) <= s7_ax(i);
                end if;
                s8_s(i) <= resize(s7_ax(i), 9) + resize(s7_ay(i), 9);
            end loop;

            -- S9: shape terms (all pre-scaled by 1/sqrt2 to stay in 8 bits):
            -- m7 = 0.69*max, sh = (|x|+|y|)/2, t4 = max + 3*min
            for i in 0 to NLEV - 1 loop
                s9_m7(i) <= ('0' & s8_mx(i)(7 downto 1)) + ("000" & s8_mx(i)(7 downto 3))
                          + ("0000" & s8_mx(i)(7 downto 4));
                s9_sh(i) <= s8_s(i)(8 downto 1);
                s9_t4(i) <= resize(s8_mx(i), 10) + shift_left(resize(s8_mn(i), 10), 1)
                          + resize(s8_mn(i), 10);
                s9_mx(i) <= s8_mx(i);
                s9_mn(i) <= s8_mn(i);
            end loop;

            -- S10: per-level shape distance -> squarer inputs
            -- (circle squares both axes; linear shapes square d, y = 0)
            for i in 0 to NLEV - 1 loop
                v_b := (others => '0');
                case lv_shp(i) is
                    when "000" =>                                   -- circle
                        v_a := s9_mx(i); v_b := s9_mn(i);
                    when "001" =>                                   -- square
                        v_a := s9_mx(i);
                    when "010" =>                                   -- diamond
                        v_a := s9_sh(i);
                    when "011" =>                                   -- octagon
                        if s9_m7(i) >= s9_sh(i) then v_a := s9_m7(i); else v_a := s9_sh(i); end if;
                    when "100" =>                                   -- 8-point star
                        if s9_m7(i) >= s9_sh(i) then v_a := s9_sh(i); else v_a := s9_m7(i); end if;
                    when "101" =>                                   -- 4-point sparkle
                        v_t10 := s9_t4(i);
                        if v_t10(9) = '1' then v_a := (others => '1');
                        else                   v_a := v_t10(8 downto 1); end if;
                    when others =>                                  -- cross
                        v_a := s9_mn(i);
                end case;
                s10_qa(i) <= v_a;
                s10_qb(i) <= v_b;
            end loop;

            -- S11: squares from ROM (exact q^2, q = 0..255)
            for i in 0 to NLEV - 1 loop
                s11_sa(i) <= unsigned(C_SQ8(to_integer(s10_qa(i))));
                s11_sb(i) <= unsigned(C_SQ8(to_integer(s10_qb(i))));
            end loop;

            -- S12: squares re-registered into fabric; input luma
            for i in 0 to NLEV - 1 loop
                s12_a2(i) <= s11_sa(i);
                s12_b2(i) <= s11_sb(i);
            end loop;
            v_yn := signed(resize(unsigned(pipe(11).y), 11)) - to_signed(64, 11);
            if v_yn(10) = '1' then s12_yn <= (others => '0');
            else                   s12_yn <= unsigned(v_yn(9 downto 0)); end if;

            -- S13: r^2; video weight V = luma << gain; gradient deltas
            for i in 0 to NLEV - 1 loop
                s13_r2(i) <= resize(s12_a2(i), 17) + resize(s12_b2(i), 17);
            end loop;
            if s_vid = '0' then
                s13_V <= (others => '0');
            else
                case s_vg is
                    when "10"   => s13_V <= shift_left(resize(s12_yn, 16), 6);
                    when "01"   => s13_V <= shift_left(resize(s12_yn, 16), 5);
                    when others => s13_V <= shift_left(resize(s12_yn, 16), 4);
                end case;
            end if;
            s13_gdx <= signed(resize(late_x, 13)) - signed(resize(s_cx, 13));
            s13_gdy <= signed(resize(pixel_y, 13)) - signed(resize(s_cy, 13));

            -- S14: widen the band by V both ways (bright input = fat rings)
            for i in 0 to NLEV - 1 loop
                s14_rp(i) <= resize(s13_r2(i), 18) + resize(s13_V, 18);
                s14_rm(i) <= signed(resize(s13_r2(i), 19)) - signed(resize(s13_V, 19));
            end loop;
            v_g13 := abs(s13_gdx);
            s14_gax <= unsigned(v_g13(11 downto 0));
            v_g13 := abs(s13_gdy);
            s14_gay <= unsigned(v_g13(11 downto 0));

            -- S15: ring test per enabled level (Discs: lo2 = 0)
            for i in 0 to NLEV - 1 loop
                if (i <= s_dmax)
                   and (resize(s14_rm(i), 21) < signed('0' & lv_hi2(i)))
                   and (resize(s14_rp(i), 20) >= lv_lo2(i)) then
                    s15_hit(i) <= '1';
                else
                    s15_hit(i) <= '0';
                end if;
            end loop;
            v_t10 := resize(shift_right(s14_gax + s14_gay, 3), 10);
            s15_g <= v_t10(7 downto 0);

            -- S16: shallowest hit wins (= shader break), or hit parity
            v_hit := '0';
            v_par := '0';
            v_sel := 0;
            for i in NLEV - 1 downto 0 loop
                if s15_hit(i) = '1' then v_sel := i; v_hit := '1'; end if;
                v_par := v_par xor s15_hit(i);
            end loop;
            if s_xor = '1' then s16_hit <= v_par;
            else                s16_hit <= v_hit; end if;
            s16_hue <= lv_hue(v_sel) + s15_g;
            s16_y   <= lv_y(v_sel);

            -- S17: hue -> YUV via triangle wheel (U/V in quadrature, gain 3)
            v_tu := tri8(s16_hue);
            v_tv := tri8(s16_hue + to_unsigned(64, 8));
            if s_mono = '1' then
                -- phosphor: green, luma rides the hue gradient
                s17_y <= to_unsigned(256, 10) + shift_left(resize(v_tu, 10), 1);
                s17_u <= to_unsigned(300, 10);
                s17_v <= to_unsigned(280, 10);
            else
                s17_y <= s16_y;
                -- 512 + 3*(tri-128) = 128 + tri + 2*tri: range 128..890.
                -- (v0.3.3: summed from the UNSIGNED tri -- the signed form
                -- added the sign-extension net to itself, SB_CARRY I0 == I1,
                -- which can livelock router1; bit-identical result.)
                s17_u <= to_unsigned(128, 10) + resize(v_tu, 10) + shift_left(resize(v_tu, 10), 1);
                s17_v <= to_unsigned(128, 10) + resize(v_tv, 10) + shift_left(resize(v_tv, 10), 1);
            end if;
            s17_hit <= s16_hit;

            -- S_out: ring colour on black; neutral outside active video
            if pipe(LATENCY - 1).avid = '0' then
                s_io.y <= std_logic_vector(to_unsigned(64, 10));
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif s17_hit = '1' then
                s_io.y <= std_logic_vector(s17_y);
                s_io.u <= std_logic_vector(s17_u);
                s_io.v <= std_logic_vector(s17_v);
            else
                s_io.y <= std_logic_vector(to_unsigned(64, 10));
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    s_io_align_in.y       <= s_io.y;
    s_io_align_in.u       <= s_io.u;
    s_io_align_in.v       <= s_io.v;
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.field_n <= s_io.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (19 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture orbifold;
