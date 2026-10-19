-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- videosky.vhd  (v1.0 -- the picture cut into a copper line-screen)
--
-- v1.0.2 (2026-10-17): resolution scaling.  The screen phase now advances in
-- virtual 1080-line units: the per-pixel steps are scaled by the timing ID's
-- x step (1 / 1.5 / 2 at 1080 / 720p / SD) and the per-line steps by its y
-- step per FIELD line (1 / 1.5 / 4.44 at 1080p / 720p / 480i), resolved in
-- vblank with a serial shift-add multiply, so the rule pitch, waver and cross
-- keep their size relative to screen height at every resolution.  Thin rules:
-- off 1080p, a rule that is whole at 1080p (weight >= the 1080p per-sample
-- phase advance) is floored at this mode's per-sample advance (capped at half
-- a period), so it cannot vanish between SD/720p samples.  1080p is
-- bit-identical; latency unchanged (8).
--
-- Lifted out of PENROSE, whose "Video Sky" switch engraved the incoming video
-- into the sky behind the impossible staircase.  That was eight lines of code:
-- horizontal rules at a fixed 8-pixel pitch, each rule's thickness set by the
-- top three bits of the inverted luma, drawn in ink on paper (penrose.vhd,
-- stage C2).  Dark video -> fat rule -> the paper reads dark.  It is the oldest
-- reproduction trick there is: a mezzotint line screen.
--
-- VideoSky keeps that kernel exactly and gives it a whole instrument:
--
--  * CONTINUOUS PITCH AND ANGLE.  Penrose tested `v_count mod 8`, so the screen
--    was locked horizontal at a power-of-two spacing.  Here the screen
--    coordinate is a fixed-point ACCUMULATOR pair -- P steps by dpx per pixel
--    and dpy per line, Q is P rotated 90 degrees -- where (dpx,dpy) =
--    pitch * (sin,cos) of the angle knob, resolved once per frame.  Lines are
--    the level sets of P, so ANY angle and ANY spacing cost the same two adds
--    per pixel and no multiply.  Q comes free with the rotation and pays for
--    itself twice: it is the crosshatch field AND the waver argument.
--
--  * RELIEF (P12, the headline).  Luma is added to P's phase before the
--    thickness compare, so the rules bow AROUND the image instead of merely
--    thickening: bank-note engraving, Unknown Pleasures.  At 0 the screen is
--    dead flat and you have Penrose's sky verbatim; at max the luma swings the
--    phase through ~4 line periods and the field turns into pure topography.
--
--  * Waver (K5) wobbles the rules along their length by a sine of Q -- the
--    burin wandering in the hand.  Cross (S7) lays a second field down the Q
--    axis, but only into the dark half of the tone ramp, which is how a real
--    engraver builds shadow.  Width (S10) freezes the rules at a constant
--    weight so Relief is the ONLY thing shaping the picture.
--
-- Renderer: streaming, 7 stages, no BRAM, no line buffer.  Three small
-- pixel-rate multiplies (contrast, relief, waver); the per-frame angle resolve
-- shares ONE registered 9x8 multiplier, split-operand across vblank cycles, so
-- no wide combinational product ever lands on a vsync-latched register
-- (the ziffern trap: those paths are still timed).
--
-- Hardware contracts (v1.0): output gated to neutral outside active video;
-- generated palette U/V-swapped at the compose (the hardware swaps U/V; the
-- Tint path carries source chroma and is NOT swapped); vsync edges act once
-- per field via a saw-active flag (analog vsync serrates); the interlace
-- half-step goes to the BOTTOM field (field_n = '0').
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture videosky of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (7 + 1 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 7;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- quarter-wave folded sine, 64-entry logic ROM (intaglio lineage)
    ----------------------------------------------------------------------
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- Clamp into the legal 10-bit luma swing.  Returns UNSIGNED on purpose:
    -- unsigned(resize(v,10)) of a signed value saturates at the SIGNED range,
    -- so anything >= 512 wraps to black (the penrose lesson).
    function f_cy(v : signed) return unsigned is
    begin
        if    v < 16   then return to_unsigned(16, 10);
        elsif v > 1008 then return to_unsigned(1008, 10);
        else                return resize(unsigned(v(9 downto 0)), 10);
        end if;
    end function;

    -- Clamp a tone into 0..255 (the thickness/phase domain).
    function f_ct(v : signed) return unsigned is
    begin
        if    v < 0   then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else               return resize(unsigned(v(7 downto 0)), 8);
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

    -- x-step scaling of a per-pixel term: x1 / x1.5 / x2
    function f_sx(v : signed(17 downto 0); xs : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(xs) is
            when 3      => return v + shift_right(v, 1);
            when 4      => return shift_left(v, 1);
            when others => return v;
        end case;
    end function;

    function f_abs20(v : signed) return unsigned is
        variable r : signed(19 downto 0);
    begin
        r := resize(v, 20);
        if r < 0 then r := -r; end if;
        return unsigned(r);
    end function;

    -- phase advance (8.8) -> whole phase units, capped at half a period
    function f_fl(v : unsigned(19 downto 0)) return unsigned is
    begin
        if v(19 downto 15) /= 0 then return to_unsigned(128, 8);
        else                         return resize(v(14 downto 8), 8); end if;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k1_ink   : unsigned(9 downto 0) := to_unsigned(512, 10);   -- K1 Ink
    signal s_k2_con   : unsigned(9 downto 0) := to_unsigned(256, 10);   -- K2 Contrast
    signal s_k3_pit   : unsigned(9 downto 0) := to_unsigned(480, 10);   -- K3 Pitch
    signal s_k4_ang   : unsigned(9 downto 0) := (others => '0');        -- K4 Angle
    signal s_k5_wav   : unsigned(9 downto 0) := (others => '0');        -- K5 Waver
    signal s_scheme   : unsigned(9 downto 0) := (others => '0');        -- K6 Scheme
    signal s_p12      : unsigned(9 downto 0) := (others => '0');        -- P12 Relief
    signal s_cross    : std_logic := '0';                               -- S7  Cross
    signal s_invert   : std_logic := '0';                               -- S8  Invert
    signal s_tint     : std_logic := '0';                               -- S9  Tint
    signal s_wmod     : std_logic := '1';                               -- S10 Width
    signal s_drift    : std_logic := '0';                               -- S11 Drift

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_cos8, s_sin8 : signed(7 downto 0) := (others => '0');
    signal s_step   : unsigned(14 downto 0) := to_unsigned(8192, 15);
    signal s_dpx, s_dpy : signed(17 downto 0) := (others => '0');
    signal s_dqx, s_dqy : signed(17 downto 0) := (others => '0');

    -- Resolution scaling: virtual 1080-line steps from the timing ID.  The
    -- accumulators step by these mode-scaled terms: per pixel dpx*sx / -dpy*sx,
    -- per FIELD line dpy*sy / dpx*sy (sy via a serial shift-add in vblank).
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);     -- half units/pixel
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);  -- Q4.8 units/field line
    signal s_dpxv, s_dqxv : signed(17 downto 0) := (others => '0');
    signal s_dpyv, s_dqyv : signed(19 downto 0) := (others => '0');
    signal s_mlp, s_mlq   : signed(31 downto 0) := (others => '0');  -- serial products
    signal s_ysh    : unsigned(11 downto 0) := (others => '0');
    -- thin-rule floor (0 at 1080p): per-sample phase advance at this mode,
    -- applied to rules at least as heavy as the 1080p advance (s_athr)
    signal s_a1, s_a2, s_a3, s_a4, s_a5, s_a6 : unsigned(19 downto 0) := (others => '0');
    signal s_fp, s_fq, s_fa : unsigned(19 downto 0) := (others => '0');
    signal s_floorp, s_floorq : unsigned(7 downto 0) := (others => '0');
    signal s_athr   : unsigned(7 downto 0) := to_unsigned(1, 8);
    signal s_tq     : unsigned(8 downto 0) := to_unsigned(129, 9);
    signal s_bias   : signed(9 downto 0)  := (others => '0');   -- Ink, +/-256
    signal s_gain   : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_depth6 : unsigned(5 downto 0) := (others => '0');
    signal s_wav4   : unsigned(3 downto 0) := (others => '0');
    signal s_fixedw : unsigned(7 downto 0) := to_unsigned(64, 8);

    signal s_ink_y, s_ink_u, s_ink_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pap_y, s_pap_u, s_pap_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- vblank sequencer: one shared, fully registered 9x8 multiplier
    ----------------------------------------------------------------------
    signal s_seq   : unsigned(5 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0)  := (others => '0');
    signal s_ma    : signed(8 downto 0) := (others => '0');
    signal s_mb    : signed(7 downto 0) := (others => '0');
    signal s_mp    : signed(16 downto 0) := (others => '0');
    signal s_pah_c, s_pal_c : signed(16 downto 0) := (others => '0');
    signal s_pah_s, s_pal_s : signed(16 downto 0) := (others => '0');
    signal s_prod_c, s_prod_s : signed(24 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_vs_pulse : std_logic := '0';
    signal s_saw   : std_logic := '0';     -- active video seen since last acted vsync
    signal s_sch3  : unsigned(2 downto 0) := (others => '0');
    signal s_fpar  : std_logic := '0';
    signal s_ilace : std_logic := '0';
    signal s_dracc : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- screen-coordinate accumulators.  24 bits with 8 fractional bits; the
    -- phase byte is acc(15 downto 8).  A 24-bit wrap is a multiple of 2^16,
    -- so the phase is continuous across the wrap and the width is free.
    ----------------------------------------------------------------------
    signal s_accp, s_accq : signed(23 downto 0) := (others => '0');
    signal s_linp, s_linq : signed(23 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal r1_ydiff : signed(10 downto 0) := (others => '0');
    signal r1_gain  : signed(7 downto 0)  := (others => '0');
    signal r1_qsa   : unsigned(9 downto 0) := (others => '0');
    signal r1_p, r1_q : unsigned(7 downto 0) := (others => '0');

    signal r2_yp    : signed(18 downto 0) := (others => '0');
    signal r2_qs    : signed(9 downto 0)  := (others => '0');
    signal r2_p, r2_q : unsigned(7 downto 0) := (others => '0');

    signal r3_y8    : unsigned(7 downto 0) := (others => '0');
    signal r3_qs    : signed(9 downto 0)  := (others => '0');
    signal r3_p, r3_q : unsigned(7 downto 0) := (others => '0');

    signal r4_wob   : signed(11 downto 0) := (others => '0');
    signal r4_rel   : signed(11 downto 0) := (others => '0');
    signal r4_t8    : signed(9 downto 0)  := (others => '0');
    signal r4_p, r4_q : unsigned(7 downto 0) := (others => '0');

    signal r5_ph1, r5_ph2 : unsigned(7 downto 0) := (others => '0');
    signal r5_t     : unsigned(7 downto 0) := (others => '0');

    signal r6_ink   : std_logic := '0';

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- chroma delay to the colour resolve
    type t_c is array(0 to 5) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_c := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin port: the sequencer takes the cos and sin taps two cycles
    -- after presenting each angle.  Address and ROM output both registered.
    ------------------------------------------------------------------------
    s_qs_a <= s_k4_ang + to_unsigned(256, 10) when s_seq = 63 else s_k4_ang;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_t : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            -- Resolution scaling: per-mode steps, free-running from the timing
            -- ID (static while a mode runs).
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);
            s_mp    <= s_ma * s_mb;

            s_vs_pulse <= '0';

            if data_in.avid = '1' then
                s_saw <= '1';
            end if;

            -- act on the FIRST vsync edge after active video only: analog
            -- vsync serrates into several edges per field.
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw = '1' then
                s_saw <= '0';
                s_vs_pulse <= '1';

                s_k1_ink <= unsigned(registers_in(0));
                s_k2_con <= unsigned(registers_in(1));
                s_k3_pit <= unsigned(registers_in(2));
                s_k4_ang <= unsigned(registers_in(3));
                s_k5_wav <= unsigned(registers_in(4));
                s_scheme <= unsigned(registers_in(5));
                s_p12    <= unsigned(registers_in(7));
                s_cross  <= registers_in(6)(0);      -- S7
                s_invert <= registers_in(6)(1);      -- S8
                s_tint   <= registers_in(6)(2);      -- S9
                s_wmod   <= registers_in(6)(3);      -- S10
                s_drift  <= registers_in(6)(4);      -- S11

                -- interlace detect: did the field parity toggle since last vsync?
                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;

                if registers_in(6)(4) = '1' then
                    s_dracc <= s_dracc + 8;     -- one period per 32 frames
                end if;

                s_seq <= to_unsigned(63, 6);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- 63/62 present the cos and sin angles into the qsin port.
                    when 62 =>
                        -- Pitch is linear in FREQUENCY: 512 (period 128 px) up to
                        -- 16880 (period ~4 px).  Phase unit 256 = one line period,
                        -- 8 fractional bits.
                        s_step <= to_unsigned(512, 15)
                                  + shift_left(resize(s_k3_pit, 15), 4);

                    when 61 => s_cos8 <= resize(shift_right(s_qs_r, 2), 8);
                    when 60 => s_sin8 <= resize(shift_right(s_qs_r, 2), 8);

                    -- step*cos and step*sin, split-operand through the one
                    -- registered multiplier: step = sh*256 + sl.
                    when 59 => s_ma <= signed('0' & s_step(14 downto 8) & '0');
                               s_mb <= s_cos8;
                    when 58 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_cos8;
                    when 57 => s_pah_c <= s_mp;
                    when 56 => s_pal_c <= s_mp;
                    when 55 => s_ma <= signed('0' & s_step(14 downto 8) & '0');
                               s_mb <= s_sin8;
                    when 54 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_sin8;
                    when 53 => s_pah_s <= s_mp;
                    when 52 => s_pal_s <= s_mp;

                    -- sh was pre-shifted left by 1 above, so its partial lands
                    -- at <<7 not <<8; the two halves then recombine.
                    when 51 =>
                        s_prod_c <= shift_left(resize(s_pah_c, 25), 7)
                                    + resize(s_pal_c, 25);
                        s_prod_s <= shift_left(resize(s_pah_s, 25), 7)
                                    + resize(s_pal_s, 25);

                    -- Angle 0 must give Penrose's HORIZONTAL rules, so cos drives
                    -- the per-LINE step and sin the per-pixel one.
                    when 50 =>
                        s_dpy <= resize(shift_right(s_prod_c, 7), 18);
                        s_dpx <= resize(shift_right(s_prod_s, 7), 18);

                    -- Q is P rotated 90 degrees.
                    when 49 =>
                        s_dqx <= -s_dpy;
                        s_dqy <=  s_dpx;

                    when 48 =>      -- Ink: a bias on the whole tone ramp
                        s_bias <= signed(resize(shift_right(s_k1_ink, 1), 10))
                                  - to_signed(256, 10);

                    when 47 =>      -- Contrast: 1.0x .. 4.9x about mid-grey
                        s_gain <= to_unsigned(16, 7) + resize(s_k2_con(9 downto 4), 7);

                    when 46 => s_depth6 <= s_p12(9 downto 4);
                    when 45 =>
                        s_wav4 <= s_k5_wav(9 downto 6);
                        -- K6 Scheme: the firmware loads a labelled default as
                        -- its raw label INDEX (0..5), a knob/preset as 0..1023.
                        if    s_scheme < 6   then s_sch3 <= s_scheme(2 downto 0);
                        elsif s_scheme < 102 then s_sch3 <= "000";   -- Parchment
                        elsif s_scheme < 307 then s_sch3 <= "001";   -- Gallery
                        elsif s_scheme < 511 then s_sch3 <= "010";   -- Night
                        elsif s_scheme < 716 then s_sch3 <= "011";   -- Blueprint
                        elsif s_scheme < 920 then s_sch3 <= "100";   -- Cyanotype
                        else                      s_sch3 <= "101";   -- Copper
                        end if;

                    when 44 =>      -- constant rule weight when Width is Fixed
                        -- floored at 8 so Ink can never blank the plate
                        v_t := to_signed(64, 12) + resize(shift_right(s_bias, 2), 12);
                        if v_t < 8 then v_t := to_signed(8, 12); end if;
                        s_fixedw <= f_ct(v_t);

                    when 43 =>      -- print scheme (K6), authored standard
                                    -- BT.601 and U/V-swapped at the compose
                        if s_sch3 = 0 then                      -- Parchment
                            s_pap_y <= to_unsigned(740, 10);
                            s_pap_u <= to_unsigned(470, 10); s_pap_v <= to_unsigned(540, 10);
                            s_ink_y <= to_unsigned(30, 10);
                            s_ink_u <= to_unsigned(495, 10); s_ink_v <= to_unsigned(522, 10);
                        elsif s_sch3 = 1 then                   -- Gallery
                            s_pap_y <= to_unsigned(830, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(40, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_sch3 = 2 then                   -- Night
                            s_pap_y <= to_unsigned(120, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(750, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_sch3 = 3 then                   -- Blueprint
                            s_pap_y <= to_unsigned(300, 10);
                            s_pap_u <= to_unsigned(700, 10); s_pap_v <= to_unsigned(450, 10);
                            s_ink_y <= to_unsigned(850, 10);
                            s_ink_u <= to_unsigned(500, 10); s_ink_v <= to_unsigned(506, 10);
                        elsif s_sch3 = 4 then                   -- Cyanotype
                            s_pap_y <= to_unsigned(200, 10);
                            s_pap_u <= to_unsigned(640, 10); s_pap_v <= to_unsigned(470, 10);
                            s_ink_y <= to_unsigned(880, 10);
                            s_ink_u <= to_unsigned(520, 10); s_ink_v <= to_unsigned(505, 10);
                        else                                    -- Copper
                            s_pap_y <= to_unsigned(800, 10);
                            s_pap_u <= to_unsigned(480, 10); s_pap_v <= to_unsigned(530, 10);
                            s_ink_y <= to_unsigned(300, 10);
                            s_ink_u <= to_unsigned(430, 10); s_ink_v <= to_unsigned(600, 10);
                        end if;

                    -- Resolution scaling.  Per-pixel terms scale by the x step
                    -- (shift/add); per-line terms by the Q4.8 y step, through a
                    -- 12-state serial shift-add (MSB first), one add per state.
                    when 40 =>
                        s_dpxv <= f_sx(s_dpx, s_xs);
                        s_dqxv <= -f_sx(s_dpy, s_xs);
                        s_mlp  <= (others => '0');
                        s_mlq  <= (others => '0');
                        s_ysh  <= s_ys;
                    when 39 | 38 | 37 | 36 | 35 | 34 | 33 | 32 | 31 | 30 | 29 | 28 =>
                        if s_ysh(11) = '1' then
                            s_mlp <= shift_left(s_mlp, 1) + resize(s_dpy, 32);
                            s_mlq <= shift_left(s_mlq, 1) + resize(s_dpx, 32);
                        else
                            s_mlp <= shift_left(s_mlp, 1);
                            s_mlq <= shift_left(s_mlq, 1);
                        end if;
                        s_ysh <= shift_left(s_ysh, 1);
                    when 27 =>
                        s_dpyv <= resize(shift_right(s_mlp, 8), 20);   -- dpy * sy
                        s_dqyv <= resize(shift_right(s_mlq, 8), 20);   -- dqy * sy (dqy = dpx)
                    -- thin-rule floor: per-sample phase advance, this mode vs 1080p
                    when 26 =>
                        s_a1 <= f_abs20(s_dpxv); s_a2 <= f_abs20(s_dpyv);
                        s_a3 <= f_abs20(s_dqxv); s_a4 <= f_abs20(s_dqyv);
                        s_a5 <= f_abs20(s_dpx);  s_a6 <= f_abs20(s_dpy);
                    when 25 =>
                        if s_a1 > s_a2 then s_fp <= s_a1; else s_fp <= s_a2; end if;
                        if s_a3 > s_a4 then s_fq <= s_a3; else s_fq <= s_a4; end if;
                        if s_a5 > s_a6 then s_fa <= s_a5; else s_fa <= s_a6; end if;
                    when 24 =>
                        if s_xs = 2 and s_ys = 256 then             -- 1080p: no floor
                            s_floorp <= (others => '0');
                            s_floorq <= (others => '0');
                        else
                            s_floorp <= f_fl(s_fp);
                            s_floorq <= f_fl(s_fq);
                        end if;
                        if f_fl(s_fa) = 0 then s_athr <= to_unsigned(1, 8);
                        else                   s_athr <= f_fl(s_fa); end if;
                    when 23 =>
                        -- Cross weight 2(t-128) >= athr  <=>  t >= 128 + ceil(athr/2)
                        s_tq <= to_unsigned(128, 9) + resize(shift_right(resize(s_athr, 9) + 1, 1), 9);

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- Screen accumulators.  Origin at the top-left active pixel; Drift walks
    -- the frame origin so the rules crawl.  On an interlaced source the BOTTOM
    -- field (field_n = '0', odd frame rows) starts half a line step down so the
    -- two fields interleave instead of printing the same rules twice.
    ------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            -- (virtual units: the steps are the mode-scaled s_d*v; the bottom
            -- field's offset is half a FIELD step, which already includes the
            -- interlace x2)
            if s_vs_pulse = '1' then
                if s_ilace = '1' and data_in.field_n = '0' then
                    s_linp <= shift_left(resize(signed('0' & s_dracc), 24), 8)
                              + shift_right(resize(s_dpyv, 24), 1);
                    s_linq <= shift_right(resize(s_dqyv, 24), 1);
                else
                    s_linp <= shift_left(resize(signed('0' & s_dracc), 24), 8);
                    s_linq <= (others => '0');
                end if;
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_accp <= s_linp;
                s_accq <= s_linq;
                s_linp <= s_linp + resize(s_dpyv, 24);
                s_linq <= s_linq + resize(s_dqyv, 24);
            elsif data_in.avid = '1' then
                s_accp <= s_accp + resize(s_dpxv, 24);
                s_accq <= s_accq + resize(s_dqxv, 24);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- R1: capture.  The contrast multiply's operands are registered here.
    -- The waver argument is Q's HIGH byte pair, so one sine period spans four
    -- line spacings along the rule -- a slow wander, not a buzz.
    ------------------------------------------------------------------------
    p_r1 : process(clk)
    begin
        if rising_edge(clk) then
            r1_ydiff <= signed('0' & unsigned(data_in.y)) - to_signed(512, 11);
            r1_gain  <= signed('0' & s_gain);
            r1_p     <= unsigned(s_accp(15 downto 8));
            r1_q     <= unsigned(s_accq(15 downto 8));
            r1_qsa   <= unsigned(s_accq(17 downto 8));
            s_ud(0)  <= unsigned(data_in.u);
            s_vd(0)  <= unsigned(data_in.v);
            for i in 1 to 5 loop
                s_ud(i) <= s_ud(i - 1);
                s_vd(i) <= s_vd(i - 1);
            end loop;
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: contrast multiply + the sine tap.
    ------------------------------------------------------------------------
    p_r2 : process(clk)
    begin
        if rising_edge(clk) then
            r2_yp <= r1_ydiff * r1_gain;
            r2_qs <= f_qsin(r1_qsa);
            r2_p  <= r1_p;
            r2_q  <= r1_q;
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- R3: clamp the stretched luma back into the legal swing and take the
    -- top 8 bits as the working tone.
    ------------------------------------------------------------------------
    p_r3 : process(clk)
        variable v_y : signed(13 downto 0);
        variable v_c : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y := to_signed(512, 14) + resize(shift_right(r2_yp, 4), 14);
            v_c := f_cy(v_y);
            r3_y8 <= v_c(9 downto 2);
            r3_qs <= r2_qs;
            r3_p  <= r2_p;
            r3_q  <= r2_q;
        end if;
    end process p_r3;

    ------------------------------------------------------------------------
    -- R4: the two shaping multiplies.
    --   rel = luma * Relief  -- up to ~4 line periods of phase swing
    --   wob = sin(Q) * Waver -- up to ~+/-60, a quarter period of wander
    -- and the raw ink tone: 255 = black = full ink (Penrose's `not luma`).
    ------------------------------------------------------------------------
    p_r4 : process(clk)
    begin
        if rising_edge(clk) then
            r4_rel <= resize(shift_right(signed('0' & r3_y8) * signed('0' & s_depth6), 4), 12);
            r4_wob <= resize(shift_right(r3_qs * signed('0' & s_wav4), 7), 12);
            r4_t8  <= to_signed(255, 10) - signed(resize(r3_y8, 10));
            r4_p   <= r3_p;
            r4_q   <= r3_q;
        end if;
    end process p_r4;

    ------------------------------------------------------------------------
    -- R5: displace both fields and settle the tone.  Only the low 8 bits of
    -- the sum matter -- (phase + wob + rel) mod 256 -- so the 8-bit phase byte
    -- carries all the information the wider accumulator had.
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable v_1, v_2 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_1 := signed(resize(r4_p, 12)) + r4_wob + r4_rel;
            v_2 := signed(resize(r4_q, 12)) + r4_wob + r4_rel;
            r5_ph1 <= unsigned(v_1(7 downto 0));
            r5_ph2 <= unsigned(v_2(7 downto 0));
            r5_t   <= f_ct(r4_t8 + resize(s_bias, 10));
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- R6: the compare that has been the whole point since Penrose --
    -- phase-within-period < thickness.  Cross lays its field only into the
    -- dark half of the ramp, doubled, the way an engraver builds a shadow.
    ------------------------------------------------------------------------
    p_r6 : process(clk)
        variable v_w1, v_w2 : unsigned(7 downto 0);
        variable v_i1, v_i2 : std_logic;
    begin
        if rising_edge(clk) then
            if s_wmod = '1' then
                v_w1 := r5_t;
            else
                v_w1 := s_fixedw;
            end if;

            if r5_t < 128 then
                v_w2 := (others => '0');
            else
                v_w2 := shift_left(r5_t - 128, 1);
            end if;

            -- (off 1080p, a rule at least as heavy as the 1080p per-sample
            -- advance is floored at this mode's advance so SD/720p samples
            -- cannot step over it; floors are 0 at 1080p)
            if r5_ph1 < v_w1 or (v_w1 >= s_athr and r5_ph1 < s_floorp) then v_i1 := '1';
            else v_i1 := '0'; end if;
            if (r5_ph2 < v_w2 or (resize(r5_t, 9) >= s_tq and r5_ph2 < s_floorq)) and s_cross = '1' then
                v_i2 := '1';
            else
                v_i2 := '0';
            end if;

            r6_ink <= v_i1 or v_i2;
        end if;
    end process p_r6;

    ------------------------------------------------------------------------
    -- R7: ink or paper.  Tint hands BOTH ink and paper the source's own
    -- chroma, so the print takes the colour of whatever it is cutting (the
    -- paper carries it visibly; dark ink alone clips to black on hardware).
    -- Generated colours are U/V-SWAPPED here (the hardware swaps U/V); the
    -- Tint chroma is a dry path and is not.  Blanking gate: neutral outside
    -- active video, one stage before the output register.
    ------------------------------------------------------------------------
    p_r7 : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_CHROMA_MID;
                s_out_v <= C_CHROMA_MID;
            elsif (r6_ink xor s_invert) = '1' then
                s_out_y <= s_ink_y;
                if s_tint = '1' then
                    s_out_u <= s_ud(5);
                    s_out_v <= s_vd(5);
                else
                    s_out_u <= s_ink_v;
                    s_out_v <= s_ink_u;
                end if;
            else
                s_out_y <= s_pap_y;
                if s_tint = '1' then
                    s_out_u <= s_ud(5);
                    s_out_v <= s_vd(5);
                else
                    s_out_u <= s_pap_v;
                    s_out_v <= s_pap_u;
                end if;
            end if;
        end if;
    end process p_r7;

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
    -- latency changes (7 -> 8).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture videosky;
