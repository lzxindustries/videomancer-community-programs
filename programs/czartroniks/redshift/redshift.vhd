-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- redshift.vhd
--
-- REDSHIFT v2 -- luma as mass. Pure gravity, no subtlety.
--
-- Brightness is matter. It bends the picture around itself, shears it into
-- rotation, splits its light red/blue by infall direction, rings itself in
-- glowing photon shells, swallows debris, and past the horizon collapses
-- into voids (or white holes). P12 is the gravitational constant: 0 = flat
-- spacetime (exact dry video), 100% = full collapse -- black space, ringed
-- singularities, spaghettified light. EVERY control acts on this one
-- always-on system; nothing is zone-gated.
--
-- Controls:
--   P12 Collapse  master G: deflection (to +/-255 px), space darkening,
--                 horizon engagement, debris speed -- the whole spectacle
--   K1 Mass       mass-formation threshold: what brightness counts as
--                 matter (low = only highlights; high = everything, x4)
--   K2 Spin       frame-dragging: signed shear of the deflection by the
--                 field's vertical slope -- masses visibly twist the image
--                 (center detent = no spin)
--   K3 Shells     width of THREE concentric photon shells (glowing rings
--                 at stepped field levels, hues rotating together)
--   K4 Doppler    signed chroma shift by deflection (blue toward, red
--                 away) + the chromatic-aberration fringe zones
--   K5 Debris     density of white-hot streaks raining into the masses
--   K6 Horizon    collapse threshold (how easily cores form)
--   S7 Polarity   Bright Mass / Dark Mass (darkness attracts)
--   S8 Core       Void (black) / White Hole (core = inverted video)
--   S9 Force      Attract / Repel (flips the deflection sign)
--   S10 Turbulence boiling-spacetime jitter on the deflection
--   S11 Pulse     slow auto-oscillation of G (breathing gravity)
--
-- Architecture: single streaming pipeline S0..S13 (C_LATENCY = 14), no
-- frame buffer. ONE full-color warp line buffer, SINGLE BANK (11 EBR vs
-- sidewinder's dual-bank 22): writing line N progressively overwrites line
-- N-1, so a read at x+dx behind the write pointer returns line N and ahead
-- of it line N-1 -- a one-line vertical error, invisible. Lens, spin shear
-- and turbulence share this one read (offsets summed, clamped s9). U and V
-- are separate arrays with independent read addresses, so the chromatic
-- fringe is just +/-fr_dx on the chroma read addresses (zero multiplies).
-- Reads landing in the border columns (black on capture sources) paint the
-- line's sampled average colour instead (lagoon's left-fill, both edges,
-- gated on nonzero deflection so Collapse = 0 stays exactly dry).
--
-- Resolution scaling (v2.2.2): geometry on a virtual 1080-line canvas (x
-- step 1.0/1.5/2.0 per pixel, y step Q4.8 per field line, from the timing
-- ID). Mass cells are 32x32 virtual units everywhere (720p used 32-px cells
-- = 1.5x coarse), deflection/spin/turbulence are virtual units shown in
-- pixels (gains x 1/xstep, clamp 255 units: SD +/-127 px, 720p +/-170),
-- Doppler follows the virtual deflection, turbulence wavelength is 128
-- units, debris rows are half virtual lines and the debris column density
-- is compensated for the 8-px columns. 1080p is bit-identical to v2.2.1.
--
-- Mass field: mercurial's coarse-grid machinery -- per-column vacc IIR
-- (K1's threshold-gain curve + S7 polarity applied at the accumulator),
-- one cell row updated per hblank ({mb,gx} packed in one 2048x16 RAM,
-- single write port, updater delayed behind the corner prefetch), rolling
-- corner prefetch + DUAL bilinear upsample (m -> shells/horizon, gx ->
-- lens). The stored field is the blurred one, so the IIR is diffusive:
-- equilibrium is a ~2-cell gaussian of the mass = long-range pull. The
-- updater's gradient is pre-amplified x4 (<<1 instead of >>1) -- v1's
-- polite gradient was the reason the lens read as subtle.
--
-- Debris: one particle per 8-px column ({pos 10.2, spd 4} in a 256x16 RAM)
-- + a bright-surface map ({bsurf 10}, raster min-writes / +1-per-field
-- drift re-adapt): streaks fall INTO the detected masses and are consumed.
-- Vblank column-walk FSM (matrix_rain), LAND->CALC->DECIDE staged.
-- EBR: warp 11 + mass 8 + vacc 1 + particles 2 = 22 of 32.
--
-- Timing discipline (mercurial): BRAM reads land in plain FFs before use;
-- the qsin ROM output is registered before any arithmetic; every multiply
-- is alone in its stage with registered inputs; lanes are fully assembled
-- one stage before the priority mux; the vblank sequencer computes all
-- per-frame terms in single-add steps (those paths are STA-timed at full
-- clock even though they only fire in blanking).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture redshift of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (14 + 2 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 14;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu8(v : unsigned) return unsigned is
    begin
        if v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    function f_cs8(v : signed) return signed is
    begin
        if v > 127 then
            return to_signed(127, 8);
        elsif v < -127 then
            return to_signed(-127, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    function f_cs9(v : signed) return signed is
    begin
        if v > 255 then
            return to_signed(255, 9);
        elsif v < -255 then
            return to_signed(-255, 9);
        else
            return resize(v, 9);
        end if;
    end function;

    -- deflection clamp to +/-lim (255 virtual units in pixels: 255/170/127)
    function f_csl(v : signed; lim : unsigned(8 downto 0)) return signed is
    begin
        if v > signed('0' & lim) then
            return signed(resize(lim, 9));
        elsif v < -signed('0' & lim) then
            return -signed(resize(lim, 9));
        else
            return resize(v, 9);
        end if;
    end function;

    -- per-frame gain x 1/xstep, first step: SD 1/2, 720p 1/2 + 1/8 (the
    -- second step adds 1/32: ~2/3), 1080 1
    function f_rx1(v : signed; sd, h7 : std_logic) return signed is
    begin
        if sd = '1' then
            return shift_right(v, 1);
        elsif h7 = '1' then
            return shift_right(v, 1) + shift_right(v, 3);
        else
            return v;
        end if;
    end function;

    function f_cs10(v : signed) return signed is
    begin
        if v > 511 then
            return to_signed(511, 10);
        elsif v < -511 then
            return to_signed(-511, 10);
        else
            return resize(v, 10);
        end if;
    end function;

    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks/mercurial):
    -- the SDK's 1024-entry LUT infers as 6 EBRs; this is 0.
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
            v_i := not a(7 downto 2);      -- 63 - i
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- value-noise hash, split across two pipeline stages (inferno lesson:
    -- 1-add hashes show diagonal weave)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := ((cx & cy)) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h(9 downto 2);
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms (vblank sequencer)
    ----------------------------------------------------------------------
    signal s_p12      : unsigned(9 downto 0) := (others => '0');
    signal s_k4       : unsigned(9 downto 0) := (others => '0');
    signal s_k6       : unsigned(9 downto 0) := (others => '0');
    signal s_pol      : std_logic := '0';                         -- S7
    signal s_white    : std_logic := '0';                         -- S8
    signal s_repel    : std_logic := '0';                         -- S9
    signal s_turb     : std_logic := '0';                         -- S10
    signal s_pulse_on : std_logic := '0';                         -- S11
    signal s_k5       : unsigned(9 downto 0) := (others => '0');

    -- per-frame terms (built one add per sequencer step)
    signal s_gain8    : unsigned(7 downto 0) := (others => '0');  -- G = P12
    signal s_geff9    : signed(8 downto 0) := (others => '0');    -- + pulse,
                                            -- negated when S9 Repel is on
    signal s_thr10    : unsigned(9 downto 0) := to_unsigned(766, 10);
    -- shell bands as PRE-REGISTERED bounds (compares must see registered
    -- bounds -- mercurial strata lesson; computing c +/- w per pixel was
    -- on the v2 critical path)
    signal s_c1, s_c2, s_c3 : unsigned(9 downto 0) := (others => '1');
    signal s_c1l, s_c1h : unsigned(9 downto 0) := (others => '1');
    signal s_c2l, s_c2h : unsigned(9 downto 0) := (others => '1');
    signal s_c3l, s_c3h : unsigned(9 downto 0) := (others => '1');
    signal s_w5       : unsigned(4 downto 0) := (others => '0');  -- shell width
    signal s_dk8      : unsigned(7 downto 0) := (others => '0');  -- space darken
    signal s_dkc10    : unsigned(9 downto 0) := (others => '0');  -- 2*dk8
    signal s_dop8     : unsigned(7 downto 0) := (others => '0');  -- doppler gain
    signal s_spin8    : signed(7 downto 0) := (others => '0');    -- spin gain
    signal s_mthr8    : unsigned(7 downto 0) := (others => '1');  -- mass threshold
    signal s_hazeg8   : unsigned(7 downto 0) := (others => '0');  -- turbulence amp
    signal s_h1u, s_h1v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_h2u, s_h2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_h3u, s_h3v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rimph    : unsigned(9 downto 0) := (others => '0');

    -- debris per-frame terms
    signal s_pgate8    : unsigned(7 downto 0) := (others => '0');
    signal s_pspd4     : unsigned(3 downto 0) := "0010";

    ----------------------------------------------------------------------
    -- Resolution scaling (v2.2.2): geometry on a virtual 1080-line canvas.
    -- x step Q.1 per pixel (1080 1.0, 720p 1.5, SD 2.0: a 1440-unit 4:3
    -- window), y step Q4.8 per FIELD line (x2 for interlace), both from the
    -- timing ID.  Mass cells are 32x32 virtual units at every resolution
    -- (60 x 34 at HD, 45 x 34 at SD), deflection / spin / turbulence gains
    -- are virtual units converted to pixels (x 1/xstep, clamp 255 units),
    -- Doppler follows the virtual deflection (x xstep), the turbulence
    -- wavelength is 128 units, and debris rows are half virtual lines (the
    -- 540-row design).  At 1080p every step is 1: bit-identical to v2.2.1.
    ----------------------------------------------------------------------
    signal s_sd, s_h7 : std_logic := '0';
    signal s_xs     : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_dlim   : unsigned(8 downto 0) := to_unsigned(255, 9);
    signal s_geffx  : signed(8 downto 0) := (others => '0');
    signal s_spinx  : signed(7 downto 0) := (others => '0');
    signal s_hazex  : unsigned(7 downto 0) := (others => '0');
    signal s_dopx   : unsigned(8 downto 0) := (others => '0');
    signal s_pgx    : unsigned(7 downto 0) := (others => '0');
    -- cell-span tracking: half units within the cell (0..63), pixel
    -- within the span, cell column
    signal s_cxh    : unsigned(6 downto 0) := (others => '0');
    signal s_cpx    : unsigned(4 downto 0) := (others => '0');
    signal s_ccol   : unsigned(7 downto 0) := (others => '0');
    -- cell-row tracking (Q.8 position of the next line within its row)
    signal s_cyacc  : unsigned(12 downto 0) := (others => '0');
    signal s_rowfirst : std_logic := '1';
    -- debris rows: Q.8 half virtual lines
    signal s_dla    : unsigned(17 downto 0) := (others => '0');
    -- turbulence phase position (half units, wraps every 128 units)
    signal s_xvt    : unsigned(7 downto 0) := (others => '0');
    -- vacc write, one clock after the span end (per-mode average)
    signal e_go     : std_logic := '0';
    signal e_first  : std_logic := '0';
    signal e_sum    : unsigned(12 downto 0) := (others => '0');
    signal e_col    : unsigned(7 downto 0) := (others => '0');

    -- fringe decode (K4 zones)
    signal s_fr_on    : std_logic := '0';
    signal s_fr_sh    : natural range 2 to 4 := 3;

    -- animation phases
    signal s_fph      : unsigned(9 downto 0) := (others => '0');  -- jitter roll
    signal s_rise8    : unsigned(7 downto 0) := (others => '0');  -- jitter rise
    signal s_pulse_acc : unsigned(9 downto 0) := (others => '0');

    signal s_seq      : unsigned(3 downto 0) := (others => '0');
    signal s_qs_a     : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar    : unsigned(9 downto 0) := (others => '0');  -- registered angle
    signal s_qs_r     : signed(9 downto 0) := (others => '0');    -- registered ROM out

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';
    -- serrated-vsync guard: analog vsync has several falling edges per
    -- field; the per-field increments (animation phases, debris walk)
    -- fire once, on the first edge after active video
    signal s_saw_act  : std_logic := '0';
    signal s_vfire    : std_logic := '0';

    ----------------------------------------------------------------------
    -- valid chain + coordinates
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 13) := (others => '0');

    signal s_x_count : unsigned(10 downto 0) := (others => '0');  -- input side
    signal wx        : unsigned(10 downto 0) := (others => '0');  -- write (S1)
    signal x_s2      : unsigned(10 downto 0) := (others => '0');
    signal rd_x      : unsigned(10 downto 0) := (others => '0');  -- read (S7)
    -- active line: 11 bits (1080p is PROGRESSIVE -- the old 10-bit count
    -- saturated at line 1000 and froze the bottom 80 lines into vertical
    -- stripes). s_dline = the debris system's row, halved on tall rasters
    -- so 1080p debris keeps its 540-row design scale.
    signal s_aline   : unsigned(10 downto 0) := (others => '0');
    signal s_dline   : unsigned(9 downto 0) := (others => '0');
    signal s_fheight : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_seen    : std_logic := '0';
    signal s_lwidth  : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_wmax    : unsigned(10 downto 0) := to_unsigned(1919, 11);

    -- per-line terms (hblank lstep sequencer)
    signal s_lstep    : unsigned(1 downto 0) := (others => '0');
    signal s_lh_a     : unsigned(11 downto 0) := (others => '0');
    signal s_ljit10   : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S0..S6: input reg, turbulence amp, phase, qsin, lens/spin mults, dx
    ----------------------------------------------------------------------
    signal r0_y, r0_u, r0_v : unsigned(9 downto 0) := (others => '0');

    signal amp1   : unsigned(7 downto 0) := (others => '0');      -- turb amp
    signal amp2, amp3, amp4 : unsigned(7 downto 0) := (others => '0');
    signal ph2    : unsigned(9 downto 0) := (others => '0');

    -- spin (frame drag): dv2m x spin gain, alone at S3, summed at S6
    signal spp    : signed(17 downto 0) := (others => '0');       -- M10 (S3)
    signal sp4, sp5 : signed(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- MASS FIELD (v0.2)
    --
    -- Mercurial's coarse-grid machinery: cells 32 x 32 VIRTUAL units
    -- (v2.2.2; 1080p 32 px x 32 lines -> 60 cols x 34 rows, stride 60;
    -- 1080i 32 x 16 field lines; 720p ~21 px x ~21 lines, 60 cols; SD 16 px
    -- x ~7 field lines -> 45 cols, stride 48; max address 2039 < 2048). Per-column vacc accumulates the area-averaged luma
    -- (polarity applied here -- Dark Mass inverts the whole universe with
    -- one mux); the updater walks one cell row per hblank (delayed 8
    -- clocks so the corner prefetch owns the read port first): temporal
    -- IIR m += (t-m+4)>>3, [1 2 1]/4 horizontal blur, 2-cell-baseline
    -- gradient, then ONE packed write {mb u8, gx s8} per cell, two cells
    -- late (a single write port, so BRAM inference holds). Storing the
    -- BLURRED field makes the IIR diffusive: the equilibrium is a ~2-cell
    -- gaussian of the target, which is exactly the long-range lens reach
    -- we want. Display side: rolling corner prefetch + hblank prefetch of
    -- cols 0..1, then dual bilinear (m for the horizon, gx for the lens).
    ----------------------------------------------------------------------
    signal s_cols   : unsigned(5 downto 0) := to_unsigned(60, 6);
    signal s_stride : unsigned(5 downto 0) := to_unsigned(60, 6);
    -- bottom-edge replicate: the last cell row's lower bilinear corners
    -- read the last row itself (the row below it is unwritten / aliased)
    signal s_row     : unsigned(5 downto 0) := (others => '0');
    signal s_lastrow : unsigned(5 downto 0) := to_unsigned(33, 6);
    signal s_lastm1  : unsigned(5 downto 0) := to_unsigned(32, 6);
    -- vacc line subsample on 32-line cell rows (keeps the IIR's coverage
    -- at half the cell row, as on 16-line rows)
    signal s_vsamp   : std_logic := '1';

    -- display-side cell tracking
    signal fx5, fy5 : unsigned(4 downto 0) := (others => '0');
    signal dsp_rb   : unsigned(10 downto 0) := (others => '0');
    signal dsp_rb1  : unsigned(10 downto 0) := (others => '0');
    signal pf_seq   : unsigned(2 downto 0) := (others => '0');
    signal mass_raddr : unsigned(10 downto 0) := (others => '0');

    -- corner registers {mb u8, gx s8} + span-stable diffs
    signal c00, c01, c10, c11 : std_logic_vector(15 downto 0) := (others => '0');
    signal p_top, p_bot       : std_logic_vector(15 downto 0) := (others => '0');
    signal d0m, d1m, d0g, d1g : signed(8 downto 0) := (others => '0');

    -- dual bilinear pipeline (S1..S4)
    signal prtm, prbm, prtg, prbg : signed(14 downto 0) := (others => '0');
    signal c00m_1, c01m_1 : unsigned(7 downto 0) := (others => '0');
    signal c00g_1, c01g_1 : signed(7 downto 0) := (others => '0');
    signal rt2m : signed(8 downto 0) := (others => '0');
    signal dv2m : signed(9 downto 0) := (others => '0');
    signal rt2g : signed(8 downto 0) := (others => '0');
    signal dv2g : signed(9 downto 0) := (others => '0');
    signal pv3m, pv3g : signed(15 downto 0) := (others => '0');
    signal rt3m, rt3g : signed(8 downto 0) := (others => '0');
    signal m_bil4  : unsigned(7 downto 0) := (others => '0');
    signal gx_bil4 : signed(7 downto 0) := (others => '0');

    -- per-column target accumulator
    signal l8_r        : unsigned(7 downto 0) := (others => '0');
    signal acc_y       : unsigned(12 downto 0) := (others => '0');
    signal acc_pre_col : unsigned(7 downto 0) := (others => '0');
    signal vq_disp     : unsigned(11 downto 0) := (others => '0');
    signal vacc_we     : std_logic := '0';
    signal vacc_waddr  : unsigned(7 downto 0) := (others => '0');
    signal vacc_wdata  : unsigned(15 downto 0) := (others => '0');

    -- updater (one cell row per hblank, 1 cell/clk, 5-deep)
    signal u_go, u_wait, u_active : std_logic := '0';
    signal u_rb    : unsigned(10 downto 0) := (others => '0');
    signal u_cx    : unsigned(5 downto 0) := (others => '0');
    signal u_v     : std_logic_vector(1 to 4) := (others => '0');
    signal u_a0    : unsigned(10 downto 0) := (others => '0');
    signal u_vca   : unsigned(7 downto 0) := (others => '0');
    signal u_c0, u_c1, u_c2, u_c3 : unsigned(5 downto 0) := (others => '0');
    signal u_m2, u_t2 : unsigned(7 downto 0) := (others => '0');
    signal u_A3    : unsigned(7 downto 0) := (others => '0');
    signal u_am1, u_am2 : unsigned(7 downto 0) := (others => '0');   -- m'[c-1], m'[c-2]
    signal u_bm1, u_bm2 : unsigned(7 downto 0) := (others => '0');   -- mb[c-1], mb[c-2]
    signal u_wm4   : unsigned(7 downto 0) := (others => '0');
    signal u_wg4   : signed(7 downto 0) := (others => '0');
    signal u_wa4   : unsigned(10 downto 0) := (others => '0');
    signal u_we4   : std_logic := '0';
    signal s_pend_row : std_logic := '0';

    -- memories
    type t_mass is array (0 to 2047) of std_logic_vector(15 downto 0);
    type t_vacc is array (0 to 255)  of std_logic_vector(15 downto 0);
    signal mass_ram : t_mass := (others => (others => '0'));
    signal vacc     : t_vacc := (others => (others => '0'));
    signal mass_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal vacc_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal mass_addr_eff : unsigned(10 downto 0) := (others => '0');
    signal vacc_raddr    : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- DEBRIS
    --
    -- One particle per 8-px column. part_ram {pos 10.2, spd 4}; surf_ram
    -- {bsurf 10, 6 spare}: bsurf = topmost bright row per column (raster
    -- lane detect, min-writes during active video; the updater drifts it
    -- +1 row per field so it re-adapts when the object moves). White-hot
    -- streaks fall INTO the bright surface and are consumed (respawn in
    -- the off-screen sky). Render side prefetches the next column's words
    -- mid-span (mass corner prefetch pattern) and pipes a 1-bit hit flag
    -- down to the S12 lane select. Vblank column-walk FSM (matrix_rain),
    -- LAND -> CALC -> DECIDE staged.
    ----------------------------------------------------------------------
    type t_p256 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal part_ram : t_p256 := (others => (others => '0'));
    signal surf_ram : t_p256 := (others => "1111111111" & "000000");
    signal part_q, surf_q : std_logic_vector(15 downto 0) := (others => '0');
    signal part_raddr, surf_raddr : unsigned(7 downto 0) := (others => '0');
    signal prf_col : unsigned(7 downto 0) := (others => '0');

    -- render-side column registers (current + prefetched next)
    signal pc_pos : unsigned(11 downto 0) := (others => '0');
    signal pn_pos : unsigned(11 downto 0) := (others => '0');
    signal sc_bs, sn_bs : unsigned(9 downto 0) := (others => '1');
    signal pc_h8, pn_h8 : unsigned(7 downto 0) := (others => '0');
    signal p_ha : unsigned(11 downto 0) := (others => '0');
    signal r0_ph3 : unsigned(2 downto 0) := (others => '0');

    -- hit-flag pipe (S1 -> S12)
    signal fl_sr  : std_logic_vector(2 to 12) := (others => '0');

    -- raster bright-surface detect write
    signal sw_we   : std_logic := '0';
    signal sw_addr : unsigned(7 downto 0) := (others => '0');
    signal sw_data : std_logic_vector(15 downto 0) := (others => '0');

    -- vblank updater FSM (the one-state USE version consumed the BRAM
    -- read data in a single deep clock and was the v0.5 critical path --
    -- land first, then two shallow decision stages; vblank has room)
    type t_pu is (PU_IDLE, PU_REQ, PU_W1, PU_W2, PU_LAND, PU_CALC, PU_DECIDE);
    signal pu_state : t_pu := PU_IDLE;
    signal pu_cnt   : unsigned(7 downto 0) := (others => '0');
    signal pl_pos   : unsigned(11 downto 0) := (others => '0');
    signal pl_spd   : unsigned(3 downto 0) := (others => '0');
    signal pl_bs    : unsigned(9 downto 0) := (others => '0');
    signal pl_np    : unsigned(11 downto 0) := (others => '0');
    signal pl_rs    : unsigned(11 downto 0) := (others => '0');
    signal pl_rspd  : unsigned(3 downto 0) := (others => '0');
    signal pu_busy  : std_logic := '0';
    signal pu_we, pu_swe : std_logic := '0';
    signal pu_waddr : unsigned(7 downto 0) := (others => '0');
    signal pu_wdata, pu_swdata : std_logic_vector(15 downto 0) := (others => '0');
    signal s_nrows  : unsigned(9 downto 0) := to_unsigned(540, 10);
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";

    signal surf_waddr : unsigned(7 downto 0) := (others => '0');
    signal surf_wdata : std_logic_vector(15 downto 0) := (others => '0');
    signal surf_we    : std_logic := '0';

    -- debris streak color
    constant C_DB_Y : unsigned(9 downto 0) := to_unsigned(950, 10);
    constant C_DB_U : unsigned(9 downto 0) := to_unsigned(478, 10);
    constant C_DB_V : unsigned(9 downto 0) := to_unsigned(540, 10);

    signal lp     : signed(16 downto 0) := (others => '0');       -- M7 (S5)
    signal hp     : signed(18 downto 0) := (others => '0');       -- M9 (S5)
    signal dx6    : signed(8 downto 0) := (others => '0');
    -- deflection piped to the doppler mult at S10
    signal dx7, dx8, dx9 : signed(8 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S7..S9: warp addresses, BRAM reads, land
    ----------------------------------------------------------------------
    signal q_addr  : unsigned(10 downto 0) := (others => '0');
    signal u_addr  : unsigned(9 downto 0) := (others => '0');
    signal v_addr  : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- EDGE FILL (lagoon pattern, both edges): each line's average colour
    -- (64 pixels starting past the often-black blanking columns) is
    -- latched at hsync and painted into warp reads that land within
    -- C_EDGE columns of either border -- the deflection reveal reads as
    -- per-line sampled streaks instead of smeared black. Gated on
    -- nonzero deflection so Collapse = 0 stays exactly dry.
    ----------------------------------------------------------------------
    constant C_EDGE : integer := 6;
    signal s_sum_y, s_sum_u, s_sum_v : unsigned(15 downto 0) := (others => '0');
    signal s_avg_cnt : unsigned(6 downto 0) := (others => '0');
    signal s_lavg_y  : unsigned(9 downto 0) := (others => '0');
    signal s_lavg_u  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_lavg_v  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_wmax6   : unsigned(10 downto 0) := to_unsigned(1913, 11);
    signal efill7, efill8 : std_logic := '0';

    type t_lb_y  is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(9 downto 0);
    signal lbY : t_lb_y  := (others => (others => '0'));
    signal lbU : t_lb_uv := (others => (others => '0'));
    signal lbV : t_lb_uv := (others => (others => '0'));
    signal by_q, bu_q, bv_q : std_logic_vector(9 downto 0) := (others => '0');

    signal wy9, wu9, wv9 : unsigned(9 downto 0) := (others => '0');

    -- m_bil delayed to the gate stage
    signal m_d5, m_d6, m_d7, m_d8, m_d9 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S10..S13: grade/doppler mults, apply + gates, lanes, priority mux
    ----------------------------------------------------------------------
    signal pgy  : unsigned(15 downto 0) := (others => '0');       -- MG1 darken
    signal pdop : signed(18 downto 0) := (others => '0');         -- MG2 doppler
    signal wy10, wu10, wv10 : unsigned(9 downto 0) := (others => '0');

    signal y11, u11, v11 : unsigned(9 downto 0) := (others => '0');
    -- gates computed at S10, re-registered at S11 so they land at the S12
    -- select in step with y11
    signal core11 : std_logic := '0';
    signal sg1_11, sg2_11, sg3_11 : std_logic := '0';
    signal core12 : std_logic := '0';
    signal sg1_12, sg2_12, sg3_12 : std_logic := '0';

    -- S12 lanes (fully assembled one stage before the mux)
    signal sel12 : unsigned(2 downto 0) := (others => '0');
    signal ln_vy, ln_vu, ln_vv : unsigned(9 downto 0) := (others => '0');
    signal ln_cy, ln_cu, ln_cv : unsigned(9 downto 0) := (others => '0');

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
    -- shared quarter-wave sine: the vblank sequencer (pulse gain + rim hue)
    -- borrows the per-pixel haze tap's angle port. Both the angle and the
    -- ROM output are registered (mercurial: that path was the routed HD
    -- critical path twice; STA-timed at full clock even though the
    -- sequencer captures only fire in vblank). Consumers see qsin two
    -- clocks late; the sequencer captures two-behind.
    ------------------------------------------------------------------------
    s_qs_a <= s_pulse_acc                     when s_seq = 9 else
              s_rimph                         when s_seq = 8 else
              s_rimph + to_unsigned(256, 10)  when s_seq = 7 else
              s_rimph + to_unsigned(341, 10)  when s_seq = 6 else
              s_rimph + to_unsigned(597, 10)  when s_seq = 5 else
              s_rimph + to_unsigned(682, 10)  when s_seq = 4 else
              s_rimph + to_unsigned(938, 10)  when s_seq = 3 else
              ph2;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one add per step)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_ge : signed(10 downto 0);
        variable v_pz : unsigned(7 downto 0);
        variable v_h  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            -- timing class + steps, free-running from the timing ID
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 =>
                    s_sd <= '1'; s_h7 <= '0';
                    s_xs <= to_unsigned(4, 3); s_dlim <= to_unsigned(127, 9);
                when 5 | 6 | 14 =>
                    s_sd <= '0'; s_h7 <= '1';
                    s_xs <= to_unsigned(3, 3); s_dlim <= to_unsigned(170, 9);
                when others =>
                    s_sd <= '0'; s_h7 <= '0';
                    s_xs <= to_unsigned(2, 3); s_dlim <= to_unsigned(255, 9);
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ys <= to_unsigned(1138, 12);  -- 480i
                when 4          => s_ys <= to_unsigned(576, 12);   -- 480p
                when 8          => s_ys <= to_unsigned(960, 12);   -- 576i
                when 12         => s_ys <= to_unsigned(480, 12);   -- 576p
                when 5 | 6 | 14 => s_ys <= to_unsigned(384, 12);   -- 720p
                when 1 | 2 | 10 => s_ys <= to_unsigned(512, 12);   -- 1080i
                when others     => s_ys <= to_unsigned(256, 12);   -- 1080p
            end case;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            -- serrated-vsync guard (one registered fire per field)
            s_vfire <= '0';
            if data_in.avid = '1' then
                s_saw_act <= '1';
            elsif data_in.vsync_n = '0' and s_prev_vsync_n = '1'
                  and s_saw_act = '1' then
                s_saw_act <= '0';
                s_vfire   <= '1';
            end if;

            if s_vfire = '1' then
                s_p12  <= unsigned(registers_in(7));
                s_k4   <= unsigned(registers_in(3));
                s_k6   <= unsigned(registers_in(5));
                s_k5   <= unsigned(registers_in(4));
                s_pol      <= registers_in(6)(0);   -- switch 7
                s_white    <= registers_in(6)(1);   -- switch 8
                s_repel    <= registers_in(6)(2);   -- switch 9
                s_turb     <= registers_in(6)(3);   -- switch 10
                s_pulse_on <= registers_in(6)(4);   -- switch 11

                -- K1 mass-formation threshold: t = clamp((l8 - off) << 2),
                -- off = 255 - K1(9:2) -- continuous, low K1 = only
                -- highlights are matter, high = everything, saturated
                s_mthr8 <= to_unsigned(255, 8) - unsigned(registers_in(0)(9 downto 2));

                -- K2 spin gain, signed around the center detent (Repel
                -- flips it along with the lens -- the sign lives in the
                -- per-frame operands so S6 stays a plain add)
                if registers_in(6)(2) = '1' then
                    s_spin8 <= f_cs8(-shift_right(
                        signed(unsigned(registers_in(1)) - 512), 1));
                else
                    s_spin8 <= f_cs8(shift_right(
                        signed(unsigned(registers_in(1)) - 512), 1));
                end if;

                -- K3 shell width (0 = no shells)
                s_w5 <= unsigned(registers_in(2)(9 downto 5));

                -- K4 fringe zones (top zone 0 = off) + doppler gain
                if registers_in(3)(9 downto 8) = "00" then
                    s_fr_on <= '0';
                    s_fr_sh <= 4;
                else
                    s_fr_on <= '1';
                    s_fr_sh <= 5 - to_integer(unsigned(registers_in(3)(9 downto 8)));
                end if;
                s_dop8 <= unsigned(registers_in(3)(9 downto 2));

                -- grid geometry: 32 x 32 virtual-unit cells at every
                -- resolution (34 rows; 60 x 60-stride columns at HD = 2040
                -- < 2048 entries, 45 x 48-stride on the 1440-unit SD window)
                if s_sd = '1' then
                    s_cols <= to_unsigned(45, 6); s_stride <= to_unsigned(48, 6);
                else
                    s_cols <= to_unsigned(60, 6); s_stride <= to_unsigned(60, 6);
                end if;

                -- animation phases
                s_fph   <= s_fph + 16;
                s_rise8 <= s_rise8 + 2;
                if s_pulse_on = '1' then
                    s_pulse_acc <= s_pulse_acc + 3;
                end if;
                s_rimph <= unsigned(registers_in(3))
                           + shift_left(resize(s_fph, 10), 1);

                s_seq <= "1001";        -- 9-step capture chain
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 9 =>
                        -- master G and its derivatives, all from P12
                        -- (presenting the pulse angle)
                        s_gain8  <= s_p12(9 downto 2);
                        s_dk8    <= '0' & s_p12(9 downto 3);    -- darken
                        s_hazeg8 <= '0' & s_p12(9 downto 3);    -- turbulence
                        s_pspd4  <= "0010" + resize(s_p12(9 downto 8) & '0', 4);
                        -- (last cell row index: latched at the field end)
                    when 8 =>
                        -- horizon: fully engaged only at P12 max
                        s_thr10 <= to_unsigned(255, 10)
                                   - resize(s_k6(9 downto 3), 10)
                                   + resize(shift_right(to_unsigned(1023, 10)
                                            - s_p12, 1), 10);
                        s_dkc10 <= resize(s_dk8 & '0', 10);     -- 2*dk8
                        s_lastm1 <= s_lastrow - 1;
                        -- debris density: K5 capped by P12 engagement
                        if s_p12(9 downto 2) > 127 then
                            v_pz := (others => '1');
                        else
                            v_pz := s_p12(8 downto 2) & '0';
                        end if;
                        if s_k5(9 downto 2) < v_pz then
                            s_pgate8 <= s_k5(9 downto 2);
                        else
                            s_pgate8 <= v_pz;
                        end if;
                    when 7 =>
                        -- capture two-behind: s_qs_r = qsin(pulse angle);
                        -- Repel folds into the sign of the effective gain
                        if s_pulse_on = '1' then
                            v_ge := signed('0' & resize(s_gain8, 10))
                                    + resize(shift_right(s_qs_r, 3), 11);
                            if v_ge < 0 then
                                v_ge := (others => '0');
                            elsif v_ge > 255 then
                                v_ge := to_signed(255, 11);
                            end if;
                        else
                            v_ge := signed('0' & resize(s_gain8, 10));
                        end if;
                        if s_repel = '1' then
                            s_geff9 <= resize(-v_ge, 9);
                        else
                            s_geff9 <= resize(v_ge, 9);
                        end if;
                        -- shell centers, stepped below the horizon
                        s_c1 <= s_thr10 - 40;
                    when 6 =>
                        -- resolution scaling of the per-frame gains:
                        -- deflection terms / xstep, Doppler and debris
                        -- column density x xstep
                        s_geffx <= f_rx1(s_geff9, s_sd, s_h7);
                        s_spinx <= f_rx1(s_spin8, s_sd, s_h7);
                        s_hazex <= unsigned(f_rx1(signed(s_hazeg8), s_sd, s_h7));
                        if s_sd = '1' then
                            s_dopx <= resize(s_dop8 & '0', 9);
                            if s_pgate8(7) = '1' then
                                s_pgx <= (others => '1');
                            else
                                s_pgx <= s_pgate8(6 downto 0) & '0';
                            end if;
                        elsif s_h7 = '1' then
                            s_dopx <= resize(s_dop8, 9) + resize(s_dop8(7 downto 1), 9);
                            if s_pgate8 > 170 then
                                s_pgx <= (others => '1');
                            else
                                s_pgx <= s_pgate8 + ('0' & s_pgate8(7 downto 1));
                            end if;
                        else
                            s_dopx <= resize(s_dop8, 9);
                            s_pgx  <= s_pgate8;
                        end if;
                        s_h1u <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qs_r, 1), 11));
                        s_c2 <= s_c1 - 56;
                        s_c1l <= s_c1 - resize(s_w5, 10);
                        s_c1h <= s_c1 + resize(s_w5, 10);
                    when 5 =>
                        if s_h7 = '1' then          -- 720p: + 1/32 (~2/3)
                            s_geffx <= s_geffx + shift_right(s_geff9, 5);
                            s_spinx <= s_spinx + shift_right(s_spin8, 5);
                            s_hazex <= s_hazex + shift_right(s_hazeg8, 5);
                        end if;
                        s_h1v <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qs_r, 1), 11));
                        s_c3 <= s_c2 - 56;
                        s_c2l <= s_c2 - resize(s_w5, 10);
                        s_c2h <= s_c2 + resize(s_w5, 10);
                    when 4 =>
                        s_h2u <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qs_r, 1), 11));
                        s_c3l <= s_c3 - resize(s_w5, 10);
                        s_c3h <= s_c3 + resize(s_w5, 10);
                    when 3 =>
                        s_h2v <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qs_r, 1), 11));
                    when 2 =>
                        s_h3u <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qs_r, 1), 11));
                    when 1 =>
                        s_h3v <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qs_r, 1), 11));
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline (S0..S13)
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_q   : signed(12 downto 0);
        variable v_qu  : signed(12 downto 0);
        variable v_qv  : signed(12 downto 0);
        variable v_fr  : signed(7 downto 0);
        variable v_qa  : unsigned(10 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_u   : signed(11 downto 0);
        variable v_v   : signed(11 downto 0);
        variable v_pos : unsigned(4 downto 0);
        variable v_end : boolean;
        variable v_col : unsigned(7 downto 0);
        variable v_l8  : unsigned(7 downto 0);
        variable v_t8  : unsigned(7 downto 0);
        variable v_sum : unsigned(12 downto 0);
        variable v_avg : unsigned(7 downto 0);
        variable v_rtm : signed(8 downto 0);
        variable v_rbm : signed(8 downto 0);
        variable v_rtg : signed(8 downto 0);
        variable v_rbg : signed(8 downto 0);
        variable v_mv  : signed(9 downto 0);
        variable v_gv  : signed(9 downto 0);
        variable v_rowend : boolean;
        variable v_nal : unsigned(10 downto 0);
        variable v_pd  : unsigned(9 downto 0);
        variable v_fx3 : unsigned(2 downto 0);
        variable v_cx  : unsigned(7 downto 0);
        variable v_a3  : unsigned(14 downto 0);
        variable v_ncy : unsigned(13 downto 0);
        variable v_dl  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 13 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S0: input register + mass-grid display tracking
            sw_we <= '0';
            e_go  <= '0';      -- one-clock pulse (set at a span end below)
            if data_in.avid = '1' then
                r0_y <= unsigned(data_in.y);
                r0_u <= unsigned(data_in.u);
                r0_v <= unsigned(data_in.v);
                r0_ph3 <= s_x_count(2 downto 0);
                s_x_count <= s_x_count + 1;

                -- debris column prefetch (8-px spans): shift at phase 0,
                -- fetch column c+1 mid-span, hash in two stages
                case s_x_count(2 downto 0) is
                    when "000" =>
                        pc_pos <= pn_pos;
                        sc_bs  <= sn_bs;
                        pc_h8  <= pn_h8;
                    when "001" =>
                        prf_col <= s_x_count(10 downto 3) + 1;
                    when "010" =>
                        -- hash the REGISTERED prefetch column (recomputing
                        -- x>>3 + 1 here was on the v0.5 critical path)
                        p_ha <= f_hash_a(prf_col, x"6B", x"9D2C");
                    when "011" =>
                        pn_pos <= unsigned(part_q(15 downto 4));
                        sn_bs  <= unsigned(surf_q(15 downto 6));
                    when "100" =>
                        pn_h8 <= f_hash_b(p_ha);
                    when "101" =>
                        -- raster bright-surface detect on this column's
                        -- lane pixel (r0_y is x%8 = 4): min-write
                        if r0_y(9 downto 2) > 168
                           and s_dline < sc_bs then
                            sw_we   <= '1';
                            sw_addr <= s_x_count(10 downto 3);
                            sw_data <= std_logic_vector(s_dline) & "000000";
                        end if;
                    when others =>
                        null;
                end case;

                -- span position within the current cell: cells are 32
                -- virtual units (64 half units); the pixel count within the
                -- span drives the prefetch events, the virtual position the
                -- bilinear fraction (1080p: x(4:0), x >> 5 -- unchanged)
                v_pos := s_cpx;
                v_col := s_ccol;
                v_cx  := resize(s_cxh, 8) + resize(s_xs, 8);
                v_end := v_cx(6) = '1';
                if v_cx(6) = '1' then
                    s_cxh  <= '0' & v_cx(5 downto 0);
                    s_cpx  <= (others => '0');
                    s_ccol <= s_ccol + 1;
                else
                    s_cxh <= v_cx(6 downto 0);
                    s_cpx <= s_cpx + 1;
                end if;

                -- bilinear x-fraction for THIS pixel
                fx5 <= s_cxh(5 downto 1);

                -- per-column target accumulator + IIR. Polarity and K1's
                -- mass-formation curve both apply here: what accumulates
                -- IS the matter. t = clamp((l8 - mthr) << 2): low K1 =
                -- only highlights are mass; high K1 = everything, x4.
                -- The curve is REGISTERED (l8_r) before the accumulator
                -- add -- inline it was the v2 critical path; the 1-px
                -- skew is invisible in a 32-px cell average.
                if s_pol = '1' then
                    v_l8 := not r0_y(9 downto 2);
                else
                    v_l8 := r0_y(9 downto 2);
                end if;
                if v_l8 <= s_mthr8 then
                    v_l8 := (others => '0');
                else
                    v_t8 := v_l8 - s_mthr8;
                    if v_t8 >= 64 then
                        v_l8 := (others => '1');
                    else
                        v_l8 := v_t8(5 downto 0) & "00";
                    end if;
                end if;
                l8_r <= v_l8;
                if v_pos = 0 then
                    v_sum := resize(l8_r, 13);
                else
                    v_sum := acc_y + resize(l8_r, 13);
                end if;
                acc_y <= v_sum;
                if v_pos = 12 then
                    acc_pre_col <= v_col;      -- vacc read for the IIR
                end if;
                -- span end: the vacc write lands one clock later (below),
                -- after the per-mode span average
                if v_end then
                    e_go    <= s_vsamp;
                    e_sum   <= v_sum;
                    e_col   <= v_col;
                    e_first <= s_rowfirst;     -- first line of the cell row
                end if;
                if v_pos = 14 then
                    vq_disp <= unsigned(vacc_q(11 downto 0));
                end if;

                -- edge-fill average: 64 registered pixels, starting past
                -- the (often black) source edge columns
                if s_x_count > to_unsigned(8, 11)
                   and s_avg_cnt < to_unsigned(64, 7) then
                    s_sum_y   <= s_sum_y + resize(r0_y, 16);
                    s_sum_u   <= s_sum_u + resize(r0_u, 16);
                    s_sum_v   <= s_sum_v + resize(r0_v, 16);
                    s_avg_cnt <= s_avg_cnt + 1;
                end if;

                -- rolling corner prefetch: fetch column col+2's two rows
                -- mid-span, shift corners at span start -- but NOT at
                -- column 0, whose corners the hblank prefetch just loaded
                if v_pos = 0 and v_col /= 0 then
                    c00 <= c10;  c01 <= c11;
                    c10 <= p_top;  c11 <= p_bot;
                    d0m <= signed(resize(unsigned(p_top(15 downto 8)), 9))
                           - signed(resize(unsigned(c10(15 downto 8)), 9));
                    d1m <= signed(resize(unsigned(p_bot(15 downto 8)), 9))
                           - signed(resize(unsigned(c11(15 downto 8)), 9));
                    d0g <= resize(signed(p_top(7 downto 0)), 9)
                           - resize(signed(c10(7 downto 0)), 9);
                    d1g <= resize(signed(p_bot(7 downto 0)), 9)
                           - resize(signed(c11(7 downto 0)), 9);
                elsif v_pos = 4 then
                    if v_col + 2 >= resize(s_cols, 8) then
                        mass_raddr <= resize(dsp_rb, 11)
                                      + resize(s_cols - 1, 11);
                    else
                        mass_raddr <= resize(dsp_rb, 11) + resize(v_col, 11) + 2;
                    end if;
                elsif v_pos = 6 then
                    p_top <= mass_q;
                elsif v_pos = 8 then
                    if v_col + 2 >= resize(s_cols, 8) then
                        mass_raddr <= resize(dsp_rb1, 11)
                                      + resize(s_cols - 1, 11);
                    else
                        mass_raddr <= resize(dsp_rb1, 11) + resize(v_col, 11) + 2;
                    end if;
                elsif v_pos = 10 then
                    p_bot <= mass_q;
                end if;
            end if;

            -- vacc write (one clock after the span end; e_go is a one-clock
            -- pulse).  Span average: 1080 sum/32, 720p sum*3/64 (21-22 px
            -- spans), SD sum/16.
            vacc_we <= '0';
            if e_go = '1' then
                if s_sd = '1' then
                    v_avg := e_sum(11 downto 4);
                elsif s_h7 = '1' then
                    v_a3 := resize(e_sum, 15) + shift_left(resize(e_sum, 15), 1);
                    if v_a3(14) = '1' then
                        v_avg := (others => '1');
                    else
                        v_avg := v_a3(13 downto 6);
                    end if;
                else
                    v_avg := e_sum(12 downto 5);
                end if;
                vacc_waddr <= e_col;
                if e_first = '1' then
                    vacc_wdata <= resize(v_avg & "0000", 16);
                else
                    vacc_wdata <= resize(
                        unsigned(signed(resize(vq_disp, 14))
                                 + shift_right(signed(resize(v_avg & "0000", 14))
                                               - signed(resize(vq_disp, 14)), 2)),
                        16);
                end if;
                vacc_we <= '1';
            end if;

            -- hblank prefetch of the coming line's first two columns (and
            -- particle column 0)
            if pf_seq /= 0 then
                case pf_seq is
                    when "001" => mass_raddr <= resize(dsp_rb, 11);
                                  prf_col <= (others => '0');
                    when "010" => mass_raddr <= resize(dsp_rb1, 11);
                                  p_ha <= f_hash_a(x"00", x"6B", x"9D2C");
                    when "011" => c00 <= mass_q;
                                  mass_raddr <= resize(dsp_rb, 11) + 1;
                                  pn_pos <= unsigned(part_q(15 downto 4));
                                  sn_bs  <= unsigned(surf_q(15 downto 6));
                    when "100" => c01 <= mass_q;
                                  mass_raddr <= resize(dsp_rb1, 11) + 1;
                                  pn_h8 <= f_hash_b(p_ha);
                    when "101" => c10 <= mass_q;
                    when "110" => c11 <= mass_q;
                    when others =>
                        d0m <= signed(resize(unsigned(c10(15 downto 8)), 9))
                               - signed(resize(unsigned(c00(15 downto 8)), 9));
                        d1m <= signed(resize(unsigned(c11(15 downto 8)), 9))
                               - signed(resize(unsigned(c01(15 downto 8)), 9));
                        d0g <= resize(signed(c10(7 downto 0)), 9)
                               - resize(signed(c00(7 downto 0)), 9);
                        d1g <= resize(signed(c11(7 downto 0)), 9)
                               - resize(signed(c01(7 downto 0)), 9);
                end case;
                if pf_seq = "111" then
                    pf_seq <= (others => '0');
                else
                    pf_seq <= pf_seq + 1;
                end if;
            end if;

            -- S1: turbulence amplitude (luma-zone shift, S10-gated) + the
            -- four bilinear horizontal mults M1/M2/M4/M5 (d0/d1 are span-
            -- stable; the base corners are pipelined alongside the
            -- products -- consuming c00/c01 directly at S2 glitches every
            -- span boundary, mercurial); warp buffer write rides this
            -- stage (see p_lb*)
            if av(1) = '1' then
                if s_turb = '1' then
                    amp1 <= shift_right(s_hazex,
                                        3 - to_integer(r0_y(9 downto 8)));
                else
                    amp1 <= (others => '0');
                end if;
                prtm <= d0m * signed('0' & fx5);
                prbm <= d1m * signed('0' & fx5);
                prtg <= d0g * signed('0' & fx5);
                prbg <= d1g * signed('0' & fx5);
                c00m_1 <= unsigned(c00(15 downto 8));
                c01m_1 <= unsigned(c01(15 downto 8));
                c00g_1 <= signed(c00(7 downto 0));
                c01g_1 <= signed(c01(7 downto 0));
                wx   <= wx + 1;

                -- debris hit test (1-bit flag piped to the S12 lane
                -- select; the S0-aligned column registers are close enough
                -- for 8-px columns). 1x3 streak; x wanders as it falls.
                v_pd  := s_dline - pc_pos(11 downto 2);
                v_fx3 := pc_h8(4 downto 2) + pc_pos(4 downto 2);
                if pc_h8 < s_pgx
                   and v_pd(9 downto 2) = 0 and v_pd(1 downto 0) /= "11"
                   and r0_ph3 = v_fx3 then
                    fl_sr(2) <= '1';
                else
                    fl_sr(2) <= '0';
                end if;
            end if;
            fl_sr(3 to 12) <= fl_sr(2 to 11);

            -- S2: bilinear horizontal lerp sums + vertical diffs; land
            -- turbulence amp; assemble the jitter phase for this pixel
            if av(2) = '1' then
                v_rtm := signed(resize(c00m_1, 9)) + resize(shift_right(prtm, 5), 9);
                v_rbm := signed(resize(c01m_1, 9)) + resize(shift_right(prbm, 5), 9);
                rt2m  <= v_rtm;
                dv2m  <= resize(v_rbm, 10) - resize(v_rtm, 10);
                v_rtg := resize(c00g_1, 9) + resize(shift_right(prtg, 5), 9);
                v_rbg := resize(c01g_1, 9) + resize(shift_right(prbg, 5), 9);
                rt2g  <= v_rtg;
                dv2g  <= resize(v_rbg, 10) - resize(v_rtg, 10);

                amp2 <= amp1;
                -- (turbulence wavelength 128 virtual units: s_xvt counts
                -- half units, so x 4 = the old x_s2 x 8 at 1080p)
                ph2  <= shift_left(resize(s_xvt, 10), 2)
                        + s_ljit10
                        + shift_left(resize(s_fph, 10), 1);
                x_s2 <= x_s2 + 1;
                s_xvt <= s_xvt + s_xs;
            end if;

            -- S3: bilinear vertical mults M3/M6 + the spin mult M10
            -- (frame drag: the field's free vertical slope, signed gain),
            -- alone in their stage; qsin angle registers via the shared
            -- port (s_qs_ar)
            if av(3) = '1' then
                pv3m <= dv2m * signed('0' & fy5);
                pv3g <= dv2g * signed('0' & fy5);
                spp  <= dv2m * s_spinx;
                rt3m <= rt2m;
                rt3g <= rt2g;
                amp3 <= amp2;
            end if;

            -- S4: assemble m_bil/gx_bil (+ warm-up guard for the first
            -- lines/columns before the prefetch has primed); land the
            -- spin product; qsin ROM output registers (s_qs_r)
            if av(4) = '1' then
                amp4 <= amp3;
                sp4  <= f_cs10(resize(shift_right(spp, 6), 12));
                v_mv := resize(rt3m, 10) + resize(shift_right(pv3m, 5), 10);
                v_gv := resize(rt3g, 10) + resize(shift_right(pv3g, 5), 10);
                if s_aline < 4 or x_s2 < 8 then
                    m_bil4  <= (others => '0');
                    gx_bil4 <= (others => '0');
                else
                    if v_mv < 0 then
                        m_bil4 <= (others => '0');
                    elsif v_mv > 255 then
                        m_bil4 <= (others => '1');
                    else
                        m_bil4 <= resize(unsigned(v_mv), 8);
                    end if;
                    gx_bil4 <= f_cs8(v_gv);
                end if;
            end if;

            -- S5: the displacement mults, alone in their stage:
            -- M7 lens = gx_bil * gain (gain carries the Repel sign),
            -- M9 turbulence = qsin * amp
            if av(5) = '1' then
                lp <= gx_bil4 * s_geffx;
                hp <= s_qs_r * signed('0' & amp4);
                sp5  <= sp4;
                m_d5 <= m_bil4;
            end if;

            -- S6: total displacement = lens + spin shear + turbulence,
            -- clamped s9
            if av(6) = '1' then
                dx6 <= f_csl(resize(shift_right(lp, 4), 14)
                             + resize(shift_right(hp, 12), 14)
                             + resize(sp5, 14), s_dlim);
                m_d6 <= m_d5;
            end if;

            -- S7: warp read addresses (clamped to the measured line width;
            -- space stretches at the frame edge). U/V get +/- the fringe
            -- offset -- true spectral separation, zero multiplies.
            if av(7) = '1' then
                v_q := signed(resize(rd_x, 13)) + resize(dx6, 13);
                if s_fr_on = '1' then
                    v_fr := resize(shift_right(dx6, s_fr_sh), 8);
                else
                    v_fr := (others => '0');
                end if;
                v_qu := v_q + resize(v_fr, 13);
                v_qv := v_q - resize(v_fr, 13);

                if v_q < 0 then
                    v_qa := (others => '0');
                elsif v_q > signed(resize(s_wmax, 13)) then
                    v_qa := s_wmax;
                else
                    v_qa := unsigned(v_q(10 downto 0));
                end if;
                q_addr <= v_qa;

                if v_qu < 0 then
                    v_qa := (others => '0');
                elsif v_qu > signed(resize(s_wmax, 13)) then
                    v_qa := s_wmax;
                else
                    v_qa := unsigned(v_qu(10 downto 0));
                end if;
                u_addr <= v_qa(10 downto 1);

                if v_qv < 0 then
                    v_qa := (others => '0');
                elsif v_qv > signed(resize(s_wmax, 13)) then
                    v_qa := s_wmax;
                else
                    v_qa := unsigned(v_qv(10 downto 0));
                end if;
                v_addr <= v_qa(10 downto 1);

                -- edge-fill zone: the read landed in the border columns
                -- (which smear black on capture sources) and the field is
                -- actually deflecting -- paint the line's sampled colour
                if (v_q < to_signed(C_EDGE, 13)
                    or v_q > signed(resize(s_wmax6, 13)))
                   and dx6 /= 0 then
                    efill7 <= '1';
                else
                    efill7 <= '0';
                end if;

                rd_x <= rd_x + 1;
                dx7  <= dx6;
                m_d7 <= m_d6;
            end if;

            -- S8: BRAM reads happen this cycle (see p_lb*)
            if av(8) = '1' then
                dx8    <= dx7;
                m_d8   <= m_d7;
                efill8 <= efill7;
            end if;

            -- S9: land the warp reads (edge-fill zone lands the line's
            -- sampled colour instead -- one stable-select mux on the BRAM
            -- outputs, sidewinder p_wet precedent); grades and Doppler
            -- apply on top downstream, so the fill inherits the red/blue
            -- drag near masses
            if av(9) = '1' then
                if efill8 = '1' then
                    wy9 <= s_lavg_y;
                    wu9 <= s_lavg_u;
                    wv9 <= s_lavg_v;
                else
                    wy9 <= unsigned(by_q);
                    wu9 <= unsigned(bu_q);
                    wv9 <= unsigned(bv_q);
                end if;
                dx9  <= dx8;
                m_d9 <= m_d8;
            end if;

            -- S10: the darken mult MG1 (pure reg x reg -- the fused
            -- (255-y) pre-subtract stays folded into the per-frame
            -- constant s_dkc10, the v0.1 critical-path lesson) + the
            -- Doppler mult MG2 (signed deflection x gain); shell gates
            if av(10) = '1' then
                pgy  <= wy9(9 downto 2) * s_dk8;
                pdop <= dx9 * signed('0' & s_dopx);
                wy10 <= wy9;
                wu10 <= wu9;
                wv10 <= wv9;

                -- horizon core + three photon shells on the (delayed)
                -- upsampled mass field
                if resize(m_d9, 10) > s_thr10 then
                    core11 <= '1';
                else
                    core11 <= '0';
                end if;
                if resize(m_d9, 10) >= s_c1l
                   and resize(m_d9, 10) <= s_c1h then
                    sg1_11 <= '1';
                else
                    sg1_11 <= '0';
                end if;
                if resize(m_d9, 10) >= s_c2l
                   and resize(m_d9, 10) <= s_c2h then
                    sg2_11 <= '1';
                else
                    sg2_11 <= '0';
                end if;
                if resize(m_d9, 10) >= s_c3l
                   and resize(m_d9, 10) <= s_c3h then
                    sg3_11 <= '1';
                else
                    sg3_11 <= '0';
                end if;
            end if;

            -- S11: apply space darkening (shadows collapse first, the
            -- identity trick: (255-y)*dk = 255*dk - y*dk) + the Doppler
            -- chroma split (signed deflection into V up / U down: blue on
            -- the infalling side, red on the receding)
            if av(11) = '1' then
                v_y := signed(resize(wy10, 12))
                       - signed(resize(s_dkc10, 12))
                       + signed(resize(pgy(15 downto 7), 12));
                y11 <= f_cu10(v_y);

                v_u := signed(resize(wu10, 12))
                       - resize(shift_right(pdop, 8), 12);
                v_v := signed(resize(wv10, 12))
                       + resize(shift_right(pdop, 7), 12);
                u11 <= f_cu10(v_u);
                v11 <= f_cu10(v_v);

                core12 <= core11;
                sg1_12 <= sg1_11;
                sg2_12 <= sg2_11;
                sg3_12 <= sg3_11;
            end if;

            -- S12: lanes fully assembled one stage before the mux
            if av(12) = '1' then
                -- graded video lane
                ln_vy <= y11;
                ln_vu <= u11;
                ln_vv <= v11;
                -- core lane: black void, or the White Hole (inverted
                -- warped video, chroma mirrored)
                if s_white = '1' then
                    ln_cy <= not y11;
                    ln_cu <= not u11;
                    ln_cv <= not v11;
                else
                    ln_cy <= to_unsigned(48, 10);
                    ln_cu <= to_unsigned(512, 10);
                    ln_cv <= to_unsigned(512, 10);
                end if;

                -- priority select: core > shells > debris > video
                if core12 = '1' then
                    sel12 <= "100";
                elsif sg1_12 = '1' then
                    sel12 <= "101";
                elsif sg2_12 = '1' then
                    sel12 <= "110";
                elsif sg3_12 = '1' then
                    sel12 <= "111";
                elsif fl_sr(12) = '1' then
                    sel12 <= "010";
                else
                    sel12 <= "000";
                end if;
            end if;

            -- S13: priority mux -> output registers
            if av(13) = '1' then
                case sel12 is
                    when "100" =>                       -- collapsed core
                        s_out_y <= ln_cy;
                        s_out_u <= ln_cu;
                        s_out_v <= ln_cv;
                    when "101" =>                       -- photon shell 1
                        s_out_y <= to_unsigned(832, 10);
                        s_out_u <= s_h1u;
                        s_out_v <= s_h1v;
                    when "110" =>                       -- photon shell 2
                        s_out_y <= to_unsigned(768, 10);
                        s_out_u <= s_h2u;
                        s_out_v <= s_h2v;
                    when "111" =>                       -- photon shell 3
                        s_out_y <= to_unsigned(704, 10);
                        s_out_u <= s_h3u;
                        s_out_v <= s_h3v;
                    when "010" =>                       -- debris streak
                        s_out_y <= C_DB_Y;
                        s_out_u <= C_DB_U;
                        s_out_v <= C_DB_V;
                    when others =>                      -- graded video
                        s_out_y <= ln_vy;
                        s_out_u <= ln_vu;
                        s_out_v <= ln_vv;
                end case;
            else
                -- blanking contract: neutral outside (aligned) avid
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= to_unsigned(512, 10);
                s_out_v <= to_unsigned(512, 10);
            end if;

            -- line bookkeeping: width measure, counter resets, aline,
            -- per-line jitter terms (2-step lstep sequencer)
            if s_lstep = 1 then
                s_lh_a  <= f_hash_a(resize(s_aline, 8) - s_rise8,
                                    resize(s_fph(9 downto 2), 8), x"B33F");
                s_lstep <= "10";
            elsif s_lstep = 2 then
                s_ljit10 <= shift_left(resize(f_hash_b(s_lh_a), 10), 1);
                s_lstep <= "00";
            end if;

            u_go <= '0';
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then
                    s_lwidth <= s_x_count;
                    s_wmax   <= s_x_count - 1;
                    s_wmax6  <= s_x_count - (C_EDGE + 1);
                end if;

                -- latch this line's sampled fill colour, restart the sum
                if s_avg_cnt = to_unsigned(64, 7) then
                    s_lavg_y <= s_sum_y(15 downto 6);
                    s_lavg_u <= s_sum_u(15 downto 6);
                    s_lavg_v <= s_sum_v(15 downto 6);
                end if;
                s_sum_y   <= (others => '0');
                s_sum_u   <= (others => '0');
                s_sum_v   <= (others => '0');
                s_avg_cnt <= (others => '0');
                s_x_count <= (others => '0');
                s_cxh  <= (others => '0');
                s_cpx  <= (others => '0');
                s_ccol <= (others => '0');
                s_xvt  <= (others => '0');
                wx    <= (others => '0');
                x_s2  <= (others => '0');
                rd_x  <= (others => '0');
                s_lstep <= "01";

                -- cell rows are 32 virtual lines: the next line's position
                -- within its row (Q.8); a wrap ends this row (1080p: line
                -- 31 of 32, unchanged)
                v_ncy    := resize(s_cyacc, 14) + resize(s_ys, 14);
                v_rowend := v_ncy(13) = '1';
                v_dl     := s_dla + resize(s_ys(11 downto 1), 18);

                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 2000 then
                        s_aline <= s_aline + 1;
                    end if;
                    v_nal := s_aline + 1;
                    if v_rowend then
                        -- this cell row is complete: update it, then
                        -- advance the display row bases (the last row's
                        -- lower corners replicate the last row itself)
                        u_go       <= '1';
                        u_rb       <= dsp_rb;
                        s_pend_row <= '0';
                        s_row      <= s_row + 1;
                        dsp_rb     <= dsp_rb + resize(s_stride, 11);
                        if s_row /= s_lastm1 then
                            dsp_rb1 <= dsp_rb1 + resize(s_stride, 11);
                        end if;
                    else
                        s_pend_row <= '1';
                    end if;
                else
                    -- vblank: fire the final partial cell row once, then
                    -- reset the field-vertical state
                    if s_pend_row = '1' then
                        u_go       <= '1';
                        u_rb       <= dsp_rb;
                        s_pend_row <= '0';
                    end if;
                    if s_aline /= 0 then
                        s_fheight <= s_aline;   -- measured field height
                        -- last cell row index, and the debris rows (the
                        -- row of the line after the last: 540 at 1080p)
                        if s_pend_row = '1' then
                            s_lastrow <= s_row;
                        else
                            s_lastrow <= s_row - 1;
                        end if;
                        s_nrows <= s_dline;
                    end if;
                    s_aline <= (others => '0');
                    v_nal   := (others => '0');
                    s_row   <= (others => '0');
                    dsp_rb  <= (others => '0');
                    dsp_rb1 <= resize(s_stride, 11);
                end if;

                -- y fraction / first-line flag / debris row for the
                -- coming line, from the virtual-line accumulators
                if s_seen = '1' then
                    s_cyacc    <= v_ncy(12 downto 0);
                    fy5        <= v_ncy(12 downto 8);
                    if v_rowend then
                        s_rowfirst <= '1';
                    else
                        s_rowfirst <= '0';
                    end if;
                    s_dla   <= v_dl;
                    s_dline <= v_dl(17 downto 8);
                else
                    s_cyacc    <= (others => '0');
                    fy5        <= (others => '0');
                    s_rowfirst <= '1';
                    s_dla      <= (others => '0');
                    s_dline    <= (others => '0');
                end if;
                -- 1080p (32 lines per cell row): accumulate even lines only;
                -- every line where rows are fewer field lines
                if s_ys = 256 then
                    s_vsamp <= not v_nal(0);
                else
                    s_vsamp <= '1';
                end if;

                pf_seq <= "001";   -- prefetch the coming line's cols 0..1
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- mass-field updater: one cell row per hblank, 1 cell/clk, 5-deep.
    -- Starts 8 clocks after u_go so the hblank corner prefetch owns the
    -- mass read port first (worst case 8 + 62 + 5 clocks vs the 138-clock
    -- minimum hblank). All shifts, zero multiplies. The walk runs 2 cells
    -- past cols (edge-replicated flush) because blur and gradient each
    -- trail one cell; the packed {mb, gx} write lands two cells late
    -- through ONE write port.
    ------------------------------------------------------------------------
    p_upd : process(clk)
        variable v_e  : signed(8 downto 0);
        variable v_a  : unsigned(7 downto 0);
        variable v_m  : signed(9 downto 0);
        variable v_mb : unsigned(7 downto 0);
        variable v_gx : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            -- delayed issue loop
            if u_go = '1' then
                u_wait <= '1';
            elsif u_wait = '1' and pf_seq = 0 then
                u_wait   <= '0';
                u_active <= '1';
                u_cx     <= (others => '0');
            elsif u_active = '1' then
                if u_cx = s_cols + 1 then
                    u_active <= '0';
                end if;
                u_cx <= u_cx + 1;
            end if;
            if u_active = '1' then
                if u_cx >= s_cols then
                    u_a0  <= resize(u_rb, 11) + resize(s_cols - 1, 11);
                    u_vca <= resize(s_cols - 1, 8);
                else
                    u_a0  <= resize(u_rb, 11) + resize(u_cx, 11);
                    u_vca <= resize(u_cx, 8);
                end if;
                u_c0 <= u_cx;
            end if;
            u_v(1) <= u_active;
            for i in 2 to 4 loop
                u_v(i) <= u_v(i - 1);
            end loop;
            u_c1 <= u_c0;

            -- U1 (v(2)): land the BRAM reads in plain FFs
            if u_v(2) = '1' then
                u_m2 <= unsigned(mass_q(15 downto 8));
                u_t2 <= unsigned(vacc_q(11 downto 4));
                u_c2 <= u_c1;
            end if;

            -- U2 (v(3)): temporal IIR (rounded so the upward creep is not
            -- floor-biased); flush cells replicate the previous cell
            if u_v(3) = '1' then
                if u_c2 >= s_cols then
                    v_a := u_A3;
                else
                    v_e := signed(resize(u_t2, 9)) - signed(resize(u_m2, 9));
                    v_m := signed(resize(u_m2, 10))
                           + shift_right(resize(v_e, 10) + 4, 3);
                    if v_m < 0 then
                        v_a := (others => '0');
                    elsif v_m > 255 then
                        v_a := (others => '1');
                    else
                        v_a := resize(unsigned(v_m), 8);
                    end if;
                end if;
                u_A3 <= v_a;
                u_c3 <= u_c2;
            end if;

            -- U3 (v(4)): [1 2 1]/4 blur (edge-replicated), 2-cell-baseline
            -- gradient, write registers -- all against the OLD shift regs
            u_we4 <= '0';
            if u_v(4) = '1' then
                v_mb := resize(shift_right(resize(u_am2, 10)
                              + shift_left(resize(u_am1, 10), 1)
                              + resize(u_A3, 10), 2), 8);
                if u_c3 = 0 then
                    u_am1 <= u_A3;
                    u_am2 <= u_A3;
                else
                    u_am2 <= u_am1;
                    u_am1 <= u_A3;
                end if;
                v_gx := f_cs8(shift_right(signed(resize(v_mb, 10))
                                          - signed(resize(u_bm2, 10)), 1));
                if u_c3 = 1 then
                    u_bm1 <= v_mb;
                    u_bm2 <= v_mb;
                else
                    u_bm2 <= u_bm1;
                    u_bm1 <= v_mb;
                end if;
                u_wm4 <= u_bm1;                 -- mb[c-2]
                u_wg4 <= v_gx;                  -- gx[c-2]
                u_wa4 <= resize(u_rb, 11) + resize(u_c3, 11) - 2;
                if u_c3 >= 2 then
                    u_we4 <= '1';
                end if;
            end if;
        end if;
    end process p_upd;

    ------------------------------------------------------------------------
    -- mass memories. The mass read port is time-muxed: the updater owns it
    -- during its u_v(1) window (blanking only, after the prefetch), the
    -- display prefetch otherwise -- port-exclusive by construction, no
    -- arbiter (mercurial vacc pattern). One write port, updater only.
    ------------------------------------------------------------------------
    mass_addr_eff <= u_a0 when u_v(1) = '1' else mass_raddr;
    vacc_raddr    <= u_vca when u_v(1) = '1' else acc_pre_col;

    p_mass : process(clk)
    begin
        if rising_edge(clk) then
            mass_q <= mass_ram(to_integer(mass_addr_eff));
            if u_we4 = '1' then
                mass_ram(to_integer(u_wa4)) <=
                    std_logic_vector(u_wm4) & std_logic_vector(u_wg4);
            end if;
        end if;
    end process p_mass;

    p_vacc : process(clk)
    begin
        if rising_edge(clk) then
            vacc_q <= vacc(to_integer(vacc_raddr));
            if vacc_we = '1' then
                vacc(to_integer(vacc_waddr)) <= std_logic_vector(vacc_wdata);
            end if;
        end if;
    end process p_vacc;

    ------------------------------------------------------------------------
    -- debris updater: matrix_rain's column walk, once per field during
    -- vblank, LAND -> CALC -> DECIDE staged (the one-state version
    -- consumed BRAM reads in one deep clock and was a critical path).
    -- Streaks fall into the bright surface and are consumed; bsurf drifts
    -- +1 row per field so the raster min-writes re-adapt to motion.
    ------------------------------------------------------------------------
    p_part : process(clk)
        variable v_spd  : unsigned(3 downto 0);
        variable v_bs   : unsigned(9 downto 0);
        variable v_np   : unsigned(11 downto 0);
        variable v_fb   : std_logic;
    begin
        if rising_edge(clk) then
            -- free-running 16-bit Fibonacci LFSR (respawn randomness)
            v_fb := s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10);
            s_lfsr <= s_lfsr(14 downto 0) & v_fb;

            pu_we  <= '0';
            pu_swe <= '0';
            case pu_state is
                when PU_IDLE =>
                    if s_vfire = '1' then
                        pu_cnt   <= (others => '0');
                        pu_busy  <= '1';
                        pu_state <= PU_REQ;
                    end if;
                when PU_REQ =>
                    pu_state <= PU_W1;      -- read address presented (mux)
                when PU_W1 =>
                    pu_state <= PU_W2;      -- BRAM read edge
                when PU_W2 =>
                    pu_state <= PU_LAND;    -- data valid
                when PU_LAND =>
                    -- land the BRAM reads in plain FFs before ANY use
                    pl_pos <= unsigned(part_q(15 downto 4));
                    pl_spd <= unsigned(part_q(3 downto 0));
                    pl_bs  <= unsigned(surf_q(15 downto 6));
                    pu_state <= PU_CALC;
                when PU_CALC =>
                    -- advance + respawn snapshots
                    pl_np <= pl_pos + resize(s_pspd4, 12)
                             + resize(pl_spd(1 downto 0), 12);
                    pl_rs   <= (to_unsigned(960, 10)
                                + resize(unsigned(s_lfsr(5 downto 0)), 10))
                               & "00";
                    pl_rspd <= unsigned(s_lfsr(3 downto 0));
                    pu_state <= PU_DECIDE;
                when PU_DECIDE =>
                    v_np  := pl_np;
                    v_spd := pl_spd;
                    v_bs  := pl_bs;
                    -- consumed by the mass, or fell past everything ->
                    -- respawn in the off-screen sky
                    if (pl_np(11 downto 2) >= pl_bs
                        and pl_np(11 downto 2) <= pl_bs + 16
                        and pl_bs < s_nrows)
                       or (pl_np(11 downto 2) > s_nrows + 16
                           and pl_np(11 downto 2) < 900) then
                        v_np  := pl_rs;
                        v_spd := pl_rspd;
                    end if;

                    -- surface drift-down (re-adapt), then write both
                    if v_bs /= "1111111111" then
                        v_bs := v_bs + 1;
                    end if;

                    pu_waddr  <= pu_cnt;
                    pu_wdata  <= std_logic_vector(v_np) & std_logic_vector(v_spd);
                    pu_swdata <= std_logic_vector(v_bs) & "000000";
                    pu_we     <= '1';
                    pu_swe    <= '1';
                    if pu_cnt = 255 then
                        pu_busy  <= '0';
                        pu_state <= PU_IDLE;
                    else
                        pu_cnt   <= pu_cnt + 1;
                        pu_state <= PU_REQ;
                    end if;
            end case;
        end if;
    end process p_part;

    ------------------------------------------------------------------------
    -- particle memories: read ports muxed updater (vblank) / render
    -- prefetch (active); surf write port muxed updater / raster detect --
    -- all pre-muxed single ports, port-exclusive by construction.
    ------------------------------------------------------------------------
    part_raddr <= pu_cnt when pu_busy = '1' else prf_col;
    surf_raddr <= pu_cnt when pu_busy = '1' else prf_col;
    surf_waddr <= pu_waddr when pu_swe = '1' else sw_addr;
    surf_wdata <= pu_swdata when pu_swe = '1' else sw_data;
    surf_we    <= pu_swe or sw_we;

    p_partram : process(clk)
    begin
        if rising_edge(clk) then
            part_q <= part_ram(to_integer(part_raddr));
            if pu_we = '1' then
                part_ram(to_integer(pu_waddr)) <= pu_wdata;
            end if;
        end if;
    end process p_partram;

    p_surfram : process(clk)
    begin
        if rising_edge(clk) then
            surf_q <= surf_ram(to_integer(surf_raddr));
            if surf_we = '1' then
                surf_ram(to_integer(surf_waddr)) <= surf_wdata;
            end if;
        end if;
    end process p_surfram;

    ------------------------------------------------------------------------
    -- warp line buffers: SINGLE BANK, canonical 1W1R. The write lands the
    -- S0-registered pixel at wx while the read address (registered at S7)
    -- is 6 stages behind, so dx <= +6 reads the current line and dx >= +7
    -- reads the not-yet-overwritten previous line -- a one-line vertical
    -- seam at the sign change, invisible in practice. U/V are 2:1
    -- decimated with independent read ports (chromatic fringe).
    ------------------------------------------------------------------------
    p_lbY : process(clk)
    begin
        if rising_edge(clk) then
            by_q <= lbY(to_integer(q_addr));
            if av(1) = '1' then
                lbY(to_integer(wx)) <= std_logic_vector(r0_y);
            end if;
        end if;
    end process p_lbY;

    p_lbU : process(clk)
    begin
        if rising_edge(clk) then
            bu_q <= lbU(to_integer(u_addr));
            if av(1) = '1' then
                lbU(to_integer(wx(10 downto 1))) <= std_logic_vector(r0_u);
            end if;
        end if;
    end process p_lbU;

    p_lbV : process(clk)
    begin
        if rising_edge(clk) then
            bv_q <= lbV(to_integer(v_addr));
            if av(1) = '1' then
                lbV(to_integer(wx(10 downto 1))) <= std_logic_vector(r0_v);
            end if;
        end if;
    end process p_lbV;

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
    -- latency changes (14 -> 16).
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

end architecture redshift;
