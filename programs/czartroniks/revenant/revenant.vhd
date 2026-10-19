-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- revenant.vhd  (v1.0.0 — "Ghost in the Signal": analog luma-contour glitch)
--
-- v1.0.3 (2026-10-17): resolution scaling.  The stroke pitch and the wobble
-- follow the screen height in every mode: the x phase DDA step is scaled by
-- the virtual x step from the video timing ID (registers_in(8): 1.0 at 1080p,
-- 1.5 at 720p, 2.0 at SD, a 1440-wide 4:3 window), and the line index that
-- drives the wobble and row jitter is a virtual 1080-line row (1080/486 per
-- SD frame line, already doubled for interlace).  S8's fringe is 2/3 of the
-- HD width at 720p (it used the HD width there).  1080p is bit-identical.
--
-- Re-draws a live source (a portrait on near-black ground) as a degrading analog
-- signal: the BRIGHT parts of the figure are redrawn by swarms of electric contour
-- strokes that hug the volume, the DARK parts fall into true black, the very
-- brightest cores (eyes) blow out into a solid icy fill, and the whole thing
-- frays / fringes / jitters under a master "signal decay" fader.
--
-- This is a STREAMING pipeline (no frame buffer, no BRAM): every effect is
-- computed per pixel in the coordinate + luma domain.  The strokes are iso-lines
-- of a phase field = (x phase DDA) + (luma * bend gain) + per-line offset; px
-- dominates, so they run vertically and bow left/right around the form.  Stroke
-- presence is gated by a luma key, so the silhouette frays into the black exactly
-- where the body ends (S10 Invert flips the key: strokes fill the dark instead).
--
-- Fixed colours are stored with U/V (Cb/Cr) SWAPPED to match the Videomancer
-- hardware output convention used by the HW-validated mondrian / c64 palettes
-- (stored U = Cr, stored V = Cb).
--
-- Hardware shell (v0.5, back-ported from glitchscape / rollthebones, both
-- HW-validated): output gated to neutral outside a generated, hsync-parity-locked
-- avid; the sync pipe carries syncs only (no raw video rides deep); S11 Bypass is
-- the SDK's 1-cycle passthrough; drawn luma stays within 64..900.
--
-- Signal quality (v0.6): a 4-tap horizontal luma box filter calms analog noise
-- before the crush and bend amplify it (key edges / bends boiled); output chroma
-- is 4:2:2 pair-averaged (U with the next pixel, V with the previous) so a thin
-- stroke reaches the encoder with both axes; grain comes from xorshift32 with
-- disjoint bit fields; strokes just below the eye threshold glow icy (halo).
--
-- Controls:
--   K1 Luma Key     (crushed luma above which strokes appear; high = sparse)
--   K2 Line Density (CONTINUOUS stroke pitch, 48 px .. 4.9 px at 1080p --
--                    follows the screen height in every mode -- phase-DDA)
--   K3 Contrast     (black-point crush, fixed x1.5 gain; high = more falls to black)
--   K4 Hue Rotate   (spin the violet/cyan/white tritone in 90-deg steps)
--   K5 Drift Speed  (0 = frozen; up = a constant ONE-WAY scroll of the contour
--                    phase, ~1 stroke every 4 s at mid travel to ~7/s wide open)
--   K6 Eye Glow     (0 = off; up = more of the brightest raw highlights flip to a
--                    solid icy fill -- raw-luma threshold 960 -> 642 -- with a
--                    halo of icy strokes in the 64 codes below it)
--   S7 Warp         (Off = gentle base bend; On = 3x deeper bend, P12 adds more)
--   S8 Chroma Sep   (U/V fringe, 3..15 px at 1080 (2..10 720p, 2..8 SD);
--                    amount rides P12)
--   S9 Motion       (Smooth = continuous drift; Glitch = frame-tear walk)
--   S10 Invert      (strokes fill the DARK, the figure becomes a black silhouette)
--   S11 Bypass      (SDK passthrough of the raw input)
--   P12 Signal Decay (HERO fader, all CONTINUOUS: warp bend, ripple, row jitter,
--                     glitch walk + rate, chroma fringe, grain and line bloom.
--                     0 = nearly clean stable portrait; 1 = fully possessed.)
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

architecture revenant of program_top is
    -- data_in -> data_out latency is 28 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 28;

    -- Total datapath latency: 11 assembly stages + CHR_MAX chroma-delay taps.
    constant CHR_MAX : natural := 16;
    constant NSTG    : natural := 11;
    constant LATENCY : natural := NSTG + CHR_MAX;     -- = 27, every mode

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLACK : unsigned(9 downto 0) := to_unsigned( 64, 10);
    -- Drawn-luma caps: full-scale Y clips RGB and kills chroma on hardware.
    constant Y_CAP   : unsigned(9 downto 0) := to_unsigned(768, 10);  -- coloured strokes
    constant Y_CAPW  : unsigned(9 downto 0) := to_unsigned(900, 10);  -- near-white top zone

    --------------------------------------------------------------------------
    -- Electric tritone palette: dim indigo -> violet -> blue -> cyan ->
    -- bright cyan -> near-white, indexed by crushed-luma zone (top 3 bits).
    -- Stored U = Cr, stored V = Cb (HW convention).
    --------------------------------------------------------------------------
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant PAL_Y : t_pal8 := (
        to_unsigned( 96,10), to_unsigned(150,10), to_unsigned(240,10), to_unsigned(320,10),
        to_unsigned(440,10), to_unsigned(580,10), to_unsigned(720,10), to_unsigned(880,10));
    constant PAL_U : t_pal8 := (   -- = Cr
        to_unsigned(536,10), to_unsigned(560,10), to_unsigned(590,10), to_unsigned(486,10),
        to_unsigned(360,10), to_unsigned(220,10), to_unsigned(330,10), to_unsigned(500,10));
    constant PAL_V : t_pal8 := (   -- = Cb
        to_unsigned(600,10), to_unsigned(684,10), to_unsigned(720,10), to_unsigned(760,10),
        to_unsigned(700,10), to_unsigned(640,10), to_unsigned(600,10), to_unsigned(520,10));

    -- Eye solid-key colour (icy blue-white, kept clear of RGB clipping) and the
    -- halo: STROKES (never a fill -- a halo fill flooded lit areas) recoloured a
    -- stronger icy cyan where raw luma sits within HALO_W codes below the eye key.
    constant EYE_Y  : unsigned(9 downto 0) := to_unsigned(880, 10);
    constant EYE_U  : unsigned(9 downto 0) := to_unsigned(488, 10);   -- = Cr
    constant EYE_V  : unsigned(9 downto 0) := to_unsigned(540, 10);   -- = Cb
    constant HALO_Y : unsigned(9 downto 0) := to_unsigned(740, 10);
    constant HALO_U : unsigned(9 downto 0) := to_unsigned(440, 10);   -- = Cr
    constant HALO_V : unsigned(9 downto 0) := to_unsigned(590, 10);   -- = Cb
    constant HALO_W : unsigned(9 downto 0) := to_unsigned( 64, 10);

    -- Stroke duty as a fraction of the (continuous) pitch: 3/8 of 65536.
    constant THICK_PH  : unsigned(15 downto 0) := to_unsigned(24576, 16);

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then       return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else                return unsigned(resize(v, 10));
        end if;
    end function;

    -- Triangle wave: 8-bit phase -> signed -64..63.  Summed at several frequencies
    -- to build an organic, multi-octave wobble (no single straight zig-zag).
    function tri(p : unsigned(7 downto 0)) return signed is
        variable q : unsigned(7 downto 0);
    begin
        q := p;
        if q(7) = '1' then q := not q; end if;     -- fold to a 0..127 up/down ramp
        return resize(signed('0' & q(6 downto 0)), 9) - 64;
    end function;

    -- Deadbanded pot capture for QUANTISED decodes (hue step, drift zone): a
    -- decode flips whenever the pot's ADC noise straddles a step boundary, and a
    -- flip re-draws the whole picture.  Only accept a new reading once it has
    -- moved more than KNOB_DB codes.  Within KNOB_DB of either end the capture
    -- snaps to the rail, so 0 / 1023 are always reachable (without the snap a
    -- slow sweep parks 1..24 codes short -- K5 then creeps instead of freezing).
    constant KNOB_DB : unsigned(9 downto 0) := to_unsigned(24, 10);
    constant KNOB_HI : unsigned(9 downto 0) := to_unsigned(1023 - 24, 10);
    function db_upd(cur : unsigned(9 downto 0); raw : unsigned(9 downto 0))
        return unsigned is
        variable d : signed(10 downto 0);
    begin
        d := signed(resize(raw, 11)) - signed(resize(cur, 11));   -- one subtract
        if raw < KNOB_DB then
            return to_unsigned(0, 10);
        elsif raw > KNOB_HI then
            return to_unsigned(1023, 10);
        elsif d > 24 or d < -24 then
            return raw;
        end if;
        return cur;
    end function;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
    --------------------------------------------------------------------------
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    -- Raster position / animation.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal px        : unsigned(11 downto 0) := (others => '0');  -- x in active line
    signal py        : unsigned(10 downto 0) := (others => '0');  -- line index
    signal frame_act : std_logic := '0';
    signal max_px    : unsigned(11 downto 0) := (others => '0');  -- widest line this field
    -- Resolution scaling: per-mode virtual steps from the timing ID
    -- (registers_in(8)), free-running.  x is Q.1 (2/3/4 = 1.0/1.5/2.0 per
    -- pixel), y Q4.8 per FIELD line.  py is the integer part of the virtual
    -- row accumulator pya (= the old line count at 1080p).
    signal s_xs      : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys      : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal pya       : unsigned(18 downto 0) := (others => '0');

    -- xorshift32 noise (period 2^32-1).  Disjoint bit fields per use: grain 7..2,
    -- glitch walk 18..12, glitch lottery 31..22.  (The old 1-bit-per-clock LFSR
    -- gave neighbouring pixels shifted copies of the same bits = diagonal grain.)
    signal xs        : unsigned(31 downto 0) := x"2545F491";
    signal noise_walk: signed(9 downto 0)    := (others => '0');  -- glitch per-row walk
    signal s_glitchframe : std_logic := '0';                     -- this frame tears
    signal s_glitchp : integer range 0 to 1023 := 32;            -- glitch-frame odds /1024
    signal vs_armed  : std_logic := '0';                         -- serrated-vsync one-shot arm

    -- SPI snapshot: keeps SPI routing out of the frame-latch cone.
    type t_reg10 is array(0 to 7) of unsigned(9 downto 0);
    signal reg_q     : t_reg10 := (others => (others => '0'));
    signal db_k4, db_k5 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- next captures, computed free-running and latched at hsync (keeps the
    -- compare chain out of the hsync-gated enable -- it was the HD HDMI path)
    signal db_k4_n, db_k5_n : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- K2 Line Density is a phase-DDA step: 1 LSB of pot noise = 1/3 of a stroke
    -- pitch of phase error at the right edge.  Glide, never snap (the >>4
    -- truncation is its own 15-code dead zone when parked).
    signal k2_f      : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- P12 feeds continuous gains whose 1-LSB steps move every contour; glide it
    -- (>>3: 7-code dead zone, ~0.4 s to settle a full throw).
    signal p12_f     : unsigned(9 downto 0) := to_unsigned(300, 10);

    -- Fractional line-phase accumulator: stroke pitch = 65536 / s_step pixels.
    signal xacc      : unsigned(15 downto 0) := (others => '0');
    signal row_ph    : unsigned(15 downto 0) := (others => '0');  -- per-line phase offset
    -- Smooth drift: a constant one-way scroll of the contour phase.  The field is
    -- periodic in phase, so the 16-bit wrap is exactly one stroke pitch.
    signal drift     : unsigned(15 downto 0) := (others => '0');
    signal s_drinc   : unsigned(12 downto 0) := (others => '0');  -- phase per frame (K5)
    signal s_smooth  : std_logic := '1';                         -- S9: smooth / glitch

    -- Free-running per-line offset chain.  py and noise_walk change only at
    -- hsync, so every term settles a few clocks into the line and the sum is
    -- latched into row_ph at the next hsync (a constant one-line lag).
    signal fw_wave   : signed(8 downto 0)    := (others => '0');  -- 3-octave wobble -112..109
    signal fw_t1, fw_t2, fw_t3 : unsigned(10 downto 0) := (others => '0');  -- row hash rounds
    signal fw_hash   : signed(6 downto 0)    := (others => '0');  -- row hash -32..31
    signal fw_rip    : signed(16 downto 0)   := (others => '0');  -- wobble * ck
    signal fw_jit    : signed(14 downto 0)   := (others => '0');  -- hash * jk
    signal fw_walk   : signed(17 downto 0)   := (others => '0');  -- glitch walk * ck
    signal fw_s1     : unsigned(15 downto 0) := (others => '0');
    signal fw_sum    : unsigned(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-frame latched parameters.
    --------------------------------------------------------------------------
    signal s_blk     : unsigned(9 downto 0) := (others => '0');   -- black point (crush)
    signal s_key     : unsigned(9 downto 0) := to_unsigned(300,10);-- luma key (stroke start)
    signal s_step_b  : unsigned(13 downto 0) := to_unsigned(4096, 14); -- step at 1080p
    signal s_step    : unsigned(14 downto 0) := to_unsigned(4096, 15); -- phase step / px
    signal s_hue     : unsigned(1 downto 0) := (others => '0');   -- hue rotation (x90)
    -- K4 label decode, free-running.  Labelled default: at program load the
    -- firmware writes the initial_value_label's INDEX (0..3) into the
    -- register, not a 0..1023 position, so a raw value below 4 is the label
    -- index itself (the deadband would otherwise snap it to 0 = Violet).
    signal s_k4_idx  : unsigned(1 downto 0) := (others => '0');
    signal s_eyek    : unsigned(9 downto 0) := to_unsigned(898,10);-- eye raw-luma threshold
    signal s_halok   : unsigned(9 downto 0) := to_unsigned(834,10);-- halo raw-luma threshold
    signal s_eye_en  : std_logic := '1';

    signal s_invert  : std_logic := '0';
    signal s_bypass  : std_logic := '0';

    -- P12-derived (continuous) gains
    signal s_g8      : unsigned(7 downto 0) := to_unsigned(32, 8); -- luma bend: lc*g8*2 phase
    signal s_ck      : unsigned(6 downto 0) := to_unsigned(8, 7);  -- ripple + walk gain 8..63
    signal s_jk      : unsigned(6 downto 0) := to_unsigned(16, 7); -- row-jitter gain 16..63
    signal s_chroff  : integer range 0 to CHR_MAX := 0;            -- chroma fringe (px)
    signal s_noisem  : unsigned(5 downto 0) := (others => '0');    -- grain mask
    signal s_bright  : unsigned(9 downto 0) := (others => '0');    -- stroke bloom

    --------------------------------------------------------------------------
    -- Generated avid (parity-locked start, fixed-width end) — see p_gavid.
    --------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);

    --------------------------------------------------------------------------
    -- Sync alignment (syncs only — no raw video is carried).
    --------------------------------------------------------------------------
    signal hs_d, vs_d, fl_d : std_logic_vector(0 to LATENCY - 1) := (others => '1');
    signal gv_d             : std_logic_vector(0 to LATENCY - 1) := (others => '0');
    -- source avid for the content gate: 3 extra taps cover the luma pre-filter
    signal av_d             : std_logic_vector(0 to NSTG + 1)    := (others => '0');

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s0: luma pre-filter (4-tap horizontal box) + raw-luma taps for the eye key
    signal y_d1, y_d2, y_d3 : unsigned(9 downto 0) := (others => '0');
    signal f_p, f_p1, f_p2  : unsigned(10 downto 0) := (others => '0');  -- pair sums
    signal f_box     : unsigned(9 downto 0) := (others => '0');
    -- s1: black-point subtract, eye / halo keys on RAW luma
    signal r1_lsub   : signed(11 downto 0) := (others => '0');
    signal r1_xac    : unsigned(15 downto 0) := (others => '0');
    signal r1_rip    : unsigned(15 downto 0) := (others => '0');
    signal r1_eye, r1_halo : std_logic := '0';
    -- s2: crush x1.5; line offset folded into the DDA
    signal r2_lp     : signed(13 downto 0) := (others => '0');
    signal r2_xac    : unsigned(15 downto 0) := (others => '0');
    signal r2_eye, r2_halo : std_logic := '0';
    -- s3: crushed luma Lc
    signal r3_lc     : unsigned(9 downto 0) := (others => '0');
    signal r3_xac    : unsigned(15 downto 0) := (others => '0');
    signal r3_eye, r3_halo : std_logic := '0';
    -- s4: bend product halves
    signal r4_plo    : unsigned(13 downto 0) := (others => '0');
    signal r4_phi    : unsigned(13 downto 0) := (others => '0');
    signal r4_lc     : unsigned(9 downto 0) := (others => '0');
    signal r4_xac    : unsigned(15 downto 0) := (others => '0');
    signal r4_eye, r4_halo : std_logic := '0';
    -- s5: bend phase, key test, palette zone
    signal r5_disp   : unsigned(15 downto 0) := (others => '0');
    signal r5_xac    : unsigned(15 downto 0) := (others => '0');
    signal r5_keyok  : std_logic := '0';
    signal r5_idx    : unsigned(2 downto 0) := (others => '0');
    signal r5_eye, r5_halo : std_logic := '0';
    -- s6: stroke decision + flags
    signal r6_line, r6_eye, r6_halo : std_logic := '0';
    signal r6_idx    : unsigned(2 downto 0) := (others => '0');
    -- s7: palette lookup
    signal r7_line, r7_eye, r7_halo, r7_top : std_logic := '0';
    signal r7_paly, r7_pu, r7_pv : unsigned(9 downto 0) := C_MID;
    -- s8: hue rotation + bloom add
    signal r8_line, r8_eye, r8_halo : std_logic := '0';
    signal r8_u, r8_v : unsigned(9 downto 0) := C_MID;
    signal r8_lsum   : unsigned(10 downto 0) := (others => '0');
    signal r8_cap    : unsigned(9 downto 0) := Y_CAP;
    -- s9: colour select (eye / halo / stroke / gap)
    signal r9_y      : unsigned(9 downto 0) := C_BLACK;
    signal r9_u, r9_v : unsigned(9 downto 0) := C_MID;
    -- s10: analog degradation (grain)
    signal r10_y     : unsigned(9 downto 0) := C_BLACK;
    signal r10_u, r10_v : unsigned(9 downto 0) := C_MID;

    -- chroma-separation delay lines (length 0..2*CHR_MAX).  The chroma lines
    -- hold 4:2:2 PAIR-AVERAGED values: the core's packer point-samples U on even
    -- and V on odd avid pixels, so a 1-2 px stroke on black reached the encoder
    -- with one chroma axis (bishop: blue strokes read red/yellow).  uf(k) = mean
    -- of pixel k's U and the NEXT pixel's; vb(k) = mean of pixel k's V and the
    -- PREVIOUS pixel's.  Whichever pixel the packer samples then carries its
    -- pair's average -- no parity tracking, no extra latency.  uf(k) aligns with
    -- yd(k+1), vb(k) with yd(k).
    type t_chr is array(0 to 2*CHR_MAX) of unsigned(9 downto 0);
    signal yd : t_chr := (others => C_BLACK);
    signal uf : t_chr := (others => C_MID);
    signal vb : t_chr := (others => C_MID);
    signal u_prev, v_prev : unsigned(9 downto 0) := C_MID;

    signal s_io    : t_video_stream_yuv444_30b;
    signal s_bpass : t_video_stream_yuv444_30b;   -- 1-cycle raw passthrough (S11)

begin

    --------------------------------------------------------------------------
    -- Raster position, animation, per-line offsets, per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_k5    : integer;
        variable v_k2raw, v_praw : unsigned(9 downto 0);
        variable v_nw    : signed(9 downto 0);
        variable v_ph1, v_ph2, v_ph3 : unsigned(7 downto 0);
        variable v_wave  : signed(11 downto 0);
        variable v_t     : unsigned(10 downto 0);
        variable v_pw    : unsigned(13 downto 0);
        variable v_p7, v_p3, v_p5, v_p13 : unsigned(13 downto 0);
        variable v_cs    : unsigned(4 downto 0);
        variable v_c11   : unsigned(8 downto 0);
        variable v_x1, v_x2 : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- per-mode virtual steps (the timing ID is static while a mode
            -- runs, so free-running registers are enough)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            -- stroke pitch follows the screen height: the per-pixel phase step
            -- grows with the virtual x step (1080p: unchanged)
            case s_xs is
                when "011"  => s_step <= resize(s_step_b, 15) + resize(s_step_b(13 downto 1), 15);
                when "100"  => s_step <= s_step_b & '0';
                when others => s_step <= resize(s_step_b, 15);
            end case;

            -- xorshift32 (13, 17, 5)
            v_x1 := xs xor shift_left(xs, 13);
            v_x2 := v_x1 xor shift_right(v_x1, 17);
            xs   <= v_x2 xor shift_left(v_x2, 5);

            for i in 0 to 7 loop
                reg_q(i) <= unsigned(registers_in(i)(9 downto 0));
            end loop;
            db_k4_n <= db_upd(db_k4, reg_q(3));
            if reg_q(3) < 4 then
                s_k4_idx <= reg_q(3)(1 downto 0);
            else
                s_k4_idx <= db_k4(9 downto 8);
            end if;
            db_k5_n <= db_upd(db_k5, reg_q(4));

            -----------------------------------------------------------------
            -- Per-line offset chain (free-running; see fw_* declarations).
            -- STATIC multi-octave wobble (function of py only, so the strokes
            -- stand still) + deterministic per-row jitter + the glitch walk.
            -- Everything is a phase, so its amplitude is measured in stroke
            -- periods and is independent of the density knob.
            -----------------------------------------------------------------
            v_ph1 := py(9 downto 2);   -- slow
            v_ph2 := py(8 downto 1);   -- medium
            v_ph3 := py(7 downto 0);   -- fast
            v_wave := resize(tri(v_ph1), 12)
                    + resize(shift_right(tri(v_ph2), 1), 12)
                    + resize(shift_right(tri(v_ph3), 2), 12);
            fw_wave <= resize(v_wave, 9);
            -- Row hash: x*0x6D+0x3B; ^>>4; *0x35; ^>>5; bits 10..5 (glitchscape's
            -- mix at 11 bits).  Spot-checked over 1080 rows: lag correlations
            -- <= 0.12, no equal-step runs (the old py xor hash was a sawtooth).
            fw_t1 <= resize(py * to_unsigned(16#6D#, 7), 11) + to_unsigned(16#3B#, 11);
            fw_t2 <= fw_t1 xor shift_right(fw_t1, 4);
            fw_t3 <= resize(fw_t2 * to_unsigned(16#35#, 6), 11);
            v_t   := fw_t3 xor shift_right(fw_t3, 5);
            fw_hash <= signed(resize(v_t(10 downto 5), 7)) - 32;
            -- gains: ripple = wave*ck<<6 (0.9..6.9 periods), jitter = hash*jk<<3
            -- (+-1/16..+-1/4 period: contours stay legible at the top), walk =
            -- walk*ck<<2 (+-0.23..+-1.85 periods).  16-bit wrap = contour modulo.
            fw_rip  <= fw_wave * signed('0' & s_ck);
            fw_jit  <= fw_hash * signed('0' & s_jk);
            fw_walk <= noise_walk * signed('0' & s_ck);
            fw_s1   <= shift_left(unsigned(fw_rip(15 downto 0)), 6)
                     + shift_left(unsigned(resize(fw_jit, 16)), 3);
            fw_sum  <= fw_s1 + shift_left(unsigned(fw_walk(15 downto 0)), 2);

            -- Arm the once-per-field work; analog vsync SERRATES (many edges per
            -- field) so the lottery / glides / drift step must fire once only.
            if data_in.avid = '1' then vs_armed <= '1'; end if;

            -- X counter + per-line work
            if v_h_edge = '1' then
                if px > max_px then max_px <= px; end if;
                px   <= (others => '0');
                xacc <= (others => '0');
                row_ph <= fw_sum + drift;
                -- deadbanded zone decodes, refreshed once per line
                db_k4 <= db_k4_n;
                db_k5 <= db_k5_n;
                -- random-walk drift ONLY during a glitch frame -> mostly still, with
                -- occasional tearing.  Otherwise noise_walk settles (below).
                if s_glitchframe = '1' and s_smooth = '0' then
                    v_nw := noise_walk + resize(signed(resize(xs(18 downto 12), 8)) - 64, 10);
                    if    v_nw >  480 then v_nw := to_signed( 480, 10);
                    elsif v_nw < -480 then v_nw := to_signed(-480, 10);
                    end if;
                    noise_walk <= v_nw;
                end if;
            elsif data_in.avid = '1' then
                px   <= px + 1;
                xacc <= xacc + resize(s_step, 16);
            end if;

            -- Y counter (anchored to first active line)
            if v_v_edge = '1' then
                -- virtual row origin; the bottom field starts half a field
                -- step down (inert on hardware, which ties field_n high)
                if data_in.field_n = '0' then
                    pya <= resize(s_ys(11 downto 1), 19);
                else
                    pya <= (others => '0');
                end if;
                frame_act <= '0';
                if vs_armed = '1' then
                    vs_armed  <= '0';
                    -- K2 glide (see k2_f): ease toward the pot, never snap.
                    v_k2raw := reg_q(1);
                    if v_k2raw > k2_f then
                        k2_f <= k2_f + shift_right(v_k2raw - k2_f, 4);
                    elsif v_k2raw < k2_f then
                        k2_f <= k2_f - shift_right(k2_f - v_k2raw, 4);
                    end if;
                    -- P12 glide (see p12_f).
                    v_praw := reg_q(7);
                    if v_praw > p12_f then
                        p12_f <= p12_f + shift_right(v_praw - p12_f, 3);
                    elsif v_praw < p12_f then
                        p12_f <= p12_f - shift_right(p12_f - v_praw, 3);
                    end if;
                    -- measured width -> generated-avid end + HD flag
                    if max_px > 64 then g_wm1 <= max_px - 1; end if;
                    max_px <= (others => '0');

                    -- SMOOTH drift: one constant-velocity scroll of the whole contour
                    -- phase, K5 sets the rate.  It runs in BOTH motion modes, so K5
                    -- is never dead (S9 Glitch only ADDS the tearing walk on top).
                    drift <= drift + resize(s_drinc, 16);

                    -- GLITCH motion (S9): some frames get the tearing random walk
                    -- (1 in 32 at P12 = 0, ~28% at full); calm frames settle back.
                    if s_smooth = '0'
                       and to_integer(xs(31 downto 22)) < s_glitchp then
                        s_glitchframe <= '1';
                    else
                        s_glitchframe <= '0';
                        if noise_walk > 8 or noise_walk < -8 then
                            noise_walk <= noise_walk - shift_right(noise_walk, 2);
                        else
                            noise_walk <= (others => '0');
                        end if;
                    end if;
                end if;

                ---------------------------------------------------------------
                -- Parameter latch (idempotent: safe on every serration edge).
                ---------------------------------------------------------------
                -- K3 Contrast: black point (crush is a fixed x1.5 in the datapath).
                s_blk <= resize(reg_q(2)(9 downto 2), 10);                 -- 0..255
                -- K1 Luma Key.
                s_key <= reg_q(0);
                -- K2 Line Density: step = 1365 + 12*k2 -> pitch 48.0 .. 4.9 px
                -- at 1080p (scaled per mode into s_step, see above).
                s_step_b <= to_unsigned(1365, 14)
                        + shift_left(resize(k2_f, 14), 3)
                        + shift_left(resize(k2_f, 14), 2);
                -- K4 Hue rotate (90-deg steps).
                s_hue <= s_k4_idx;
                -- K5 Drift Speed: phase per frame on a pseudo-exponential ramp.
                -- 65536 = one stroke pitch: ~2 min per stroke just off zero, ~7
                -- strokes/s wide open.  0 is exact (the deadband snaps to the rail).
                v_k5 := to_integer(db_k5);
                case v_k5 / 256 is
                    when 0      => s_drinc <= to_unsigned(        v_k5,           13);
                    when 1      => s_drinc <= to_unsigned( 256 + (v_k5 - 256) * 2, 13);
                    when 2      => s_drinc <= to_unsigned( 768 + (v_k5 - 512) * 4, 13);
                    when others => s_drinc <= to_unsigned(1792 + (v_k5 - 768) * 24, 13);
                end case;
                -- K6 Eye Glow: raw-luma threshold 960 (off for legal video) down to
                -- 642 over the whole knob (below ~640 lit areas flood solid).
                s_eyek <= to_unsigned(960, 10) - shift_right(reg_q(5), 2)
                                               - shift_right(reg_q(5), 4);
                s_halok <= to_unsigned(960, 10) - shift_right(reg_q(5), 2)
                                                - shift_right(reg_q(5), 4) - HALO_W;
                if reg_q(5) >= 24 then s_eye_en <= '1'; else s_eye_en <= '0'; end if;

                -- toggles
                s_smooth  <= not reg_q(6)(2);    -- S9 (0 = Smooth, 1 = Glitch)
                s_invert  <= reg_q(6)(3);        -- S10
                s_bypass  <= reg_q(6)(4);        -- S11

                -- P12 Signal Decay -> continuous gains (glided fader).
                v_pw  := resize(p12_f, 14);
                -- S7 Warp: bend phase = lc*g8*2.  Off = 64/code (1 period per full
                -- luma swing); On = 192..510/code (3..8 periods), P12 riding it.
                v_p5 := shift_left(v_pw, 2) + v_pw;                         -- x5
                if reg_q(6)(0) = '1' then
                    s_g8 <= to_unsigned(96, 8) + resize(v_p5(13 downto 5), 8);  -- 96..255
                else
                    s_g8 <= to_unsigned(32, 8);
                end if;
                v_p7 := shift_left(v_pw, 3) - v_pw;                         -- x7
                s_ck <= to_unsigned(8, 7) + resize(v_p7(13 downto 7), 7);    -- 8..63
                v_p3 := shift_left(v_pw, 1) + v_pw;                         -- x3
                s_jk <= to_unsigned(16, 7) + resize(v_p3(13 downto 6), 7);   -- 16..63
                -- S8 Chroma Sep: 3..15 px at 1080, x11/16 (2..10) at 720p,
                -- halved (2..8) at SD -- the same width in virtual units.
                v_p13 := shift_left(v_pw, 3) + shift_left(v_pw, 2) + v_pw;  -- x13
                v_cs  := to_unsigned(3, 5) + resize(v_p13(13 downto 10), 5);
                v_c11 := shift_left(resize(v_cs, 9), 3) + shift_left(resize(v_cs, 9), 1)
                         + resize(v_cs, 9);
                if reg_q(6)(1) = '0' then
                    s_chroff <= 0;
                elsif s_xs = "011" then
                    s_chroff <= to_integer(v_c11(8 downto 4));
                elsif s_xs = "100" then
                    s_chroff <= to_integer(shift_right(v_cs + 1, 1));
                else
                    s_chroff <= to_integer(v_cs);
                end if;
                s_noisem  <= resize(p12_f(9 downto 5), 6);                  -- grain 0..31
                s_glitchp <= 32 + to_integer(p12_f(9 downto 2));            -- 32..287 /1024
                s_bright  <= resize(p12_f(9 downto 2), 10);                 -- bloom 0..255

            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                if data_in.field_n = '0' then
                    pya <= resize(s_ys(11 downto 1), 19);
                else
                    pya <= (others => '0');
                end if;
            elsif v_h_edge = '1' then
                pya <= pya + s_ys;
            end if;
        end if;
    end process p_position;

    py <= pya(18 downto 8);

    --------------------------------------------------------------------------
    -- Clean avid (cubist/aurora/glitchscape recipe).  The encoders get no
    -- data-enable and pair Cb/Cr by counting from hsync, so an avid that wanders
    -- a clock against hsync swaps U/V for that line.  g_ph is this line's
    -- hsync->avid-rise parity, g_par the voted usual parity; an off-parity line
    -- starts one clock later.  Ends after the field-latched width, or at hsync.
    --------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising

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
        end if;
    end process p_gavid;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_ph    : unsigned(15 downto 0);
        variable v_bp    : unsigned(15 downto 0);
        variable v_cu, v_cv : unsigned(10 downto 0);
        variable v_u, v_v   : unsigned(10 downto 0);
        variable v_dimy  : signed(12 downto 0);
        variable v_noise : signed(7 downto 0);
        variable v_box   : unsigned(11 downto 0);
        variable v_gy, v_gu, v_gv : unsigned(9 downto 0);
        variable v_sum   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- sync delay line (the output avid is the generated one)
            hs_d(0) <= data_in.hsync_n;
            vs_d(0) <= data_in.vsync_n;
            fl_d(0) <= data_in.field_n;
            gv_d(0) <= g_avid;
            for i in 1 to LATENCY - 1 loop
                hs_d(i) <= hs_d(i - 1);
                vs_d(i) <= vs_d(i - 1);
                fl_d(i) <= fl_d(i - 1);
                gv_d(i) <= gv_d(i - 1);
            end loop;
            -- source avid for the content gate (s11).  r1 holds the pixel 3 clocks
            -- behind data_in (luma pre-filter), so the gate reads 3 taps deeper.
            av_d(0) <= data_in.avid;
            for i in 1 to NSTG + 1 loop av_d(i) <= av_d(i - 1); end loop;

            -----------------------------------------------------------------
            -- s0: luma pre-filter.  Analog source noise is amplified x1.5 by the
            -- crush and by up to 510 phase/code in the bend, so key edges and
            -- bends crawled on a still picture; a 4-tap horizontal box halves it.
            -- Box = pixels t-1..t-4 of the s1 sample; the 1.5 px whole-picture
            -- shift is invisible (the stroke DDA phase is arbitrary).
            -----------------------------------------------------------------
            y_d1  <= unsigned(data_in.y);
            y_d2  <= y_d1;
            y_d3  <= y_d2;
            f_p   <= resize(unsigned(data_in.y), 11) + resize(y_d1, 11);
            f_p1  <= f_p;
            f_p2  <= f_p1;
            v_box := resize(f_p, 12) + resize(f_p2, 12);
            f_box <= v_box(11 downto 2);

            -----------------------------------------------------------------
            -- s1: black-point subtract; eye + halo keys on RAW luma (the crush
            -- clamps whole lit areas to full Lc, so only raw luma can tell a true
            -- specular highlight).  y_d3 lines up with the box centre.  Raw video
            -- goes no deeper than this stage.
            -----------------------------------------------------------------
            r1_lsub <= signed(resize(f_box, 12)) - signed(resize(s_blk, 12));
            r1_eye <= '0'; r1_halo <= '0';
            if s_eye_en = '1' then
                if y_d3 >= s_eyek then
                    r1_eye <= '1';
                elsif y_d3 >= s_halok then
                    r1_halo <= '1';
                end if;
            end if;
            r1_xac <= xacc;
            r1_rip <= row_ph;

            -----------------------------------------------------------------
            -- s2: crush — fixed x1.5 gain via shift-add; fold in the line offset.
            -----------------------------------------------------------------
            r2_lp  <= resize(r1_lsub, 14) + resize(shift_right(r1_lsub, 1), 14);
            r2_xac <= r1_xac + r1_rip;
            r2_eye <= r1_eye; r2_halo <= r1_halo;

            -----------------------------------------------------------------
            -- s3: crushed luma Lc.
            -----------------------------------------------------------------
            r3_lc  <= clamp10(resize(r2_lp, 13));
            r3_xac <= r2_xac;
            r3_eye <= r2_eye; r3_halo <= r2_halo;

            -----------------------------------------------------------------
            -- s4: luma bend = lc * g8, as two 10x4 halves (own stage).
            -----------------------------------------------------------------
            r4_plo <= r3_lc * s_g8(3 downto 0);
            r4_phi <= r3_lc * s_g8(7 downto 4);
            r4_lc  <= r3_lc;
            r4_xac <= r3_xac;
            r4_eye <= r3_eye; r4_halo <= r3_halo;

            -----------------------------------------------------------------
            -- s5: bend phase (lc*g8*2; the 16-bit wrap IS the contour modulo),
            -- key test (S10 Invert flips it) and palette zone (Invert flips the
            -- ramp so the dark field gets the bright end).
            -----------------------------------------------------------------
            v_bp    := shift_left(resize(r4_phi, 16), 4) + resize(r4_plo, 16);
            r5_disp <= shift_left(v_bp, 1);
            r5_xac  <= r4_xac;
            r5_eye  <= r4_eye; r5_halo <= r4_halo;
            if (r4_lc >= s_key) xor (s_invert = '1') then
                r5_keyok <= '1';
            else
                r5_keyok <= '0';
            end if;
            if s_invert = '1' then
                r5_idx <= not r4_lc(9 downto 7);
            else
                r5_idx <= r4_lc(9 downto 7);
            end if;

            -----------------------------------------------------------------
            -- s6: stroke decision.  Field = x phase + bend + line offset; px
            -- dominates, so strokes run vertically and bow around the form.
            -----------------------------------------------------------------
            v_ph := r5_xac + r5_disp;
            r6_line <= '0'; r6_eye <= '0'; r6_halo <= '0';
            if r5_eye = '1' then
                r6_eye <= '1';
            elsif r5_keyok = '1' and v_ph < THICK_PH then
                if r5_halo = '1' then
                    r6_halo <= '1';     -- icy stroke just below the eye key
                else
                    r6_line <= '1';
                end if;
            end if;
            r6_idx <= r5_idx;

            -----------------------------------------------------------------
            -- s7: palette lookup (stroke colour by luma zone).
            -----------------------------------------------------------------
            r7_paly <= PAL_Y(to_integer(r6_idx));
            r7_pu   <= PAL_U(to_integer(r6_idx));
            r7_pv   <= PAL_V(to_integer(r6_idx));
            if r6_idx = 7 then r7_top <= '1'; else r7_top <= '0'; end if;
            r7_line <= r6_line; r7_eye <= r6_eye; r7_halo <= r6_halo;

            -----------------------------------------------------------------
            -- s8: hue rotation (90-deg steps about 512) + bloom add.
            -----------------------------------------------------------------
            v_cu := resize(r7_pu, 11);
            v_cv := resize(r7_pv, 11);
            case to_integer(s_hue) is
                when 1 =>      v_u := 1024 - v_cv; v_v := v_cu;
                when 2 =>      v_u := 1024 - v_cu; v_v := 1024 - v_cv;
                when 3 =>      v_u := v_cv;        v_v := 1024 - v_cu;
                when others => v_u := v_cu;        v_v := v_cv;
            end case;
            r8_u <= v_u(9 downto 0);
            r8_v <= v_v(9 downto 0);
            r8_lsum <= resize(r7_paly, 11) + resize(s_bright, 11);
            if r7_top = '1' then r8_cap <= Y_CAPW; else r8_cap <= Y_CAP; end if;
            r8_line <= r7_line; r8_eye <= r7_eye; r8_halo <= r7_halo;

            -----------------------------------------------------------------
            -- s9: colour select (eye solid / halo / stroke / black gap).  Bloom
            -- brightens strokes up to the cap instead of clipping.
            -----------------------------------------------------------------
            if r8_eye = '1' then
                r9_y <= EYE_Y;  r9_u <= EYE_U; r9_v <= EYE_V;
            elsif r8_halo = '1' then
                r9_y <= HALO_Y; r9_u <= HALO_U; r9_v <= HALO_V;
            elsif r8_line = '1' then
                if r8_lsum > resize(r8_cap, 11) then
                    r9_y <= r8_cap;
                else
                    r9_y <= r8_lsum(9 downto 0);
                end if;
                r9_u <= r8_u; r9_v <= r8_v;
            else
                r9_y <= C_BLACK; r9_u <= C_MID; r9_v <= C_MID;
            end if;

            -----------------------------------------------------------------
            -- s10: analog degradation — fine luma grain (floored at black).
            -----------------------------------------------------------------
            v_noise := signed(resize(xs(7 downto 2) and s_noisem, 8))
                     - signed(resize('0' & s_noisem(5 downto 1), 8));
            v_dimy := signed(resize(r9_y, 13)) + resize(v_noise, 13);
            if v_dimy < 64 then
                r10_y <= C_BLACK;
            elsif v_dimy > 1023 then
                r10_y <= to_unsigned(1023, 10);
            else
                r10_y <= unsigned(v_dimy(9 downto 0));
            end if;
            r10_u <= r9_u; r10_v <= r9_v;

            -----------------------------------------------------------------
            -- s11: content gate (source avid) -> 4:2:2 pair averages -> chroma-
            -- separation delay lines.  Outside the source line the fringe taps
            -- pull neutral.
            -----------------------------------------------------------------
            if av_d(NSTG + 1) = '1' then
                v_gy := r10_y; v_gu := r10_u; v_gv := r10_v;
            else
                v_gy := C_BLACK; v_gu := C_MID; v_gv := C_MID;
            end if;
            yd(0)  <= v_gy;
            u_prev <= v_gu;
            v_prev <= v_gv;
            v_sum  := resize(v_gu, 11) + resize(u_prev, 11);   -- u_prev's pixel + next
            uf(0)  <= v_sum(10 downto 1);
            v_sum  := resize(v_gv, 11) + resize(v_prev, 11);   -- this pixel + previous
            vb(0)  <= v_sum(10 downto 1);
            for i in 1 to 2*CHR_MAX loop
                yd(i) <= yd(i-1); uf(i) <= uf(i-1); vb(i) <= vb(i-1);
            end loop;

            -----------------------------------------------------------------
            -- output: chroma sep pulls U ahead / pushes V back.  Blanking gate:
            -- neutral outside the generated active window.
            -----------------------------------------------------------------
            if gv_d(LATENCY - 1) = '1' then
                s_io.y <= std_logic_vector(yd(CHR_MAX));
                s_io.u <= std_logic_vector(uf(CHR_MAX - 1 - s_chroff));   -- = yd(CHR_MAX - s_chroff)
                s_io.v <= std_logic_vector(vb(CHR_MAX + s_chroff));
            else
                s_io.y <= std_logic_vector(C_BLACK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= hs_d(LATENCY - 1);
            s_io.vsync_n <= vs_d(LATENCY - 1);
            s_io.avid    <= gv_d(LATENCY - 1);
            s_io.field_n <= fl_d(LATENCY - 1);

            -- S11: 1-cycle raw forward, exactly SDK passthru (data_out <= data_in)
            s_bpass <= data_in;
        end if;
    end process p_pipe;

    -- S11 bypass = SDK passthru (raw input + its own syncs, 1 cycle); a deep
    -- raw-video path read green + noisy on HD HDMI hardware (rollthebones).
    data_out <= s_bpass when s_bypass = '1' else s_io;

end architecture revenant;
