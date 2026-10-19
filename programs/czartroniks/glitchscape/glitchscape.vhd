-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- glitchscape.vhd  (v1.0.2 — "Pixel-Sort Landscape")
--
-- Databends a live source into the glitch-landscape look: a dense rectangular
-- mass of vertically-streaked, hyper-saturated jewel-tone color floating on a
-- flat teal field, punctured by blown-white hot-zones, threaded with fine white
-- contour filaments, with a jagged torn top edge and columns dripping downward.
--
-- Streaming pipeline forked from the revenant engine.  The bright parts of the
-- source become the colored mass; everything below the Mass Key falls into the
-- teal "sky".  The "sorted" streaks are iso-bars of a PHASE field: a per-pixel
-- DDA (xacc += step, so the bar pitch is continuous) plus luma bend, diagonal
-- ripple and a per-row random walk, all expressed in phase so their amplitude
-- is measured in bar pitches.  The integer part of the field is the bar index;
-- its hash picks the bar's hue, the fraction gives a 3/4-duty bar.  Brightness
-- rides the live luma so the source still reads through.
--
-- Line buffers (12 EBR):
--   * contour filaments: 4-bit luma band of the line above (2048x4, single
--     bank -- write data never depends on the read).
--   * drips: per-column (hue, bar, energy) of the line above (2 x 2048x8,
--     ping-pong by row parity: the new state depends on the read, and yosys
--     cannot infer a read-modify-write RAM).  A column's state carries down
--     while its energy beats the pixel below, decaying stochastically -- the
--     colored bars pour down over darker image and sky (vertical pixel sort).
--   * macroblocks: one colour per 16x16 (HD) / 8x8 (SD) block, sampled on the
--     block's first row (256x16 chroma + 256x8 luma).
--
-- Fixed colours are stored with U/V (Cb/Cr) SWAPPED to match the Videomancer
-- hardware output convention (stored U = Cr, stored V = Cb).
--
-- Controls:
--   K1 Mass Key      (crushed-luma above which the mass is drawn; high = sparse)
--   K2 Contrast      (black-point crush of the source before keying)
--   K3 Saturation    (chroma gain of the palette, 0..400%, continuous)
--   K4 Palette       (Jewel / Neon / Vaporwave / Inferno / Toxic / Glacier /
--                     Candy / Mono) -- each with its own matching sky colour
--   K5 Line Spacing  (bar pitch 64 px -> 4 px, continuous, exponential sweep)
--   K6 Filaments     (number of white luma-contour lines, 0..63)
--   S7 Channel Sep   (U/V fringe, 4..7 px; amount rides P12)
--   S8 Hot Zones     (blown-white cores + bloom at the brightest source spots)
--   S9 Drips         (bars pour down the frame; length rides P12)
--   S10 Compress     (macroblock chroma + blocky luma + posterize)
--   S11 Sort Axis    (Vertical streaks / Horizontal streaks)
--   P12 Glitch Intensity (HERO fader: bend, ripple, row wander, edge tear,
--        fringe, bloom, hot-zone reach, drip length and grain, all continuous.)
--
-- v1.0.2 (2026-10-18) resolution scaling: the geometry runs in virtual
-- 1080-line units from the video timing ID (registers_in(8)) -- 1080p output
-- unchanged bit for bit.  The bar pitch (K5) is a size: the x DDA steps
-- 1.5x at 720p and 2x at SD, the Horizontal row DDA once per virtual line.
-- Drip decay odds scale with the virtual lines per field line, so drips
-- run the same share of the screen.  Ripple waves, the torn-edge columns
-- and the macroblocks (16 x 16 virtual units: 720p ~10.7 px, SD 8 px x 3.6
-- field lines) use virtual coordinates, replacing the lines >= 960 switch
-- that gave 720p and SD the same 8-px blocks.  The channel-separation
-- fringe scales too (720p 3..5 px, SD 2..4 px).  Filaments, grain and the
-- row wander stay at pixel scale.
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

architecture glitchscape of program_top is
    -- data_in -> data_out latency is 28 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 28;

    -- Total datapath latency: 19 assembly stages + CHR_MAX (8) chroma-delay taps.
    constant CHR_MAX : natural := 8;
    constant NSTG    : natural := 19;
    constant LATENCY : natural := NSTG + CHR_MAX;          -- = 27

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLACK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant Y_CAP   : unsigned(9 downto 0) := to_unsigned(768, 10);  -- coloured luma cap


    --------------------------------------------------------------------------
    -- Bar palettes.  Indexed 0..7 by a per-bar hash.  Only chroma is stored
    -- (U=Cr, V=Cb); bar luma rides the live source luma.  8 palettes by K4.
    --------------------------------------------------------------------------
    type t_pal8   is array(0 to 7) of unsigned(9 downto 0);
    type t_palset is array(0 to 7) of t_pal8;

    function p8(a0, a1, a2, a3, a4, a5, a6, a7 : natural) return t_pal8 is
    begin
        return (to_unsigned(a0,10), to_unsigned(a1,10), to_unsigned(a2,10), to_unsigned(a3,10),
                to_unsigned(a4,10), to_unsigned(a5,10), to_unsigned(a6,10), to_unsigned(a7,10));
    end function;

    -- U = Cr (red-difference)
    constant PAL_U : t_palset := (
        p8(760, 700, 630, 440, 300, 340, 648, 610),   -- Jewel
        p8(820, 760, 680, 400, 250, 300, 700, 680),   -- Neon
        p8(690, 660, 600, 470, 360, 470, 660, 620),   -- Vaporwave
        p8(755, 792, 757, 689, 616, 772, 785, 634),   -- Inferno
        p8(338, 440, 511, 236, 300, 432, 330, 531),   -- Toxic
        p8(430, 336, 232, 402, 497, 425, 285, 491),   -- Glacier
        p8(714, 677, 571, 309, 568, 792, 355, 718),   -- Candy
        p8(512, 512, 512, 512, 512, 512, 512, 512));  -- Mono

    -- V = Cb (blue-difference)
    constant PAL_V : t_palset := (
        p8(360, 636, 700, 760, 600, 330, 360, 520),   -- Jewel
        p8(330, 660, 730, 790, 640, 300, 340, 540),   -- Neon
        p8(560, 660, 700, 720, 640, 560, 520, 600),   -- Vaporwave
        p8(443, 379, 313, 259, 232, 502, 381, 239),   -- Inferno
        p8(297, 263, 255, 525, 498, 346, 445, 232),   -- Toxic
        p8(727, 730, 648, 614, 745, 612, 612, 642),   -- Glacier
        p8(551, 362, 647, 480, 292, 565, 652, 443),   -- Candy
        p8(512, 512, 512, 512, 512, 512, 512, 512));  -- Mono

    -- Sky colour per palette (stored U=Cr, V=Cb; luma capped 768).  Jewel and
    -- Mono keep the signature teal #18A9A6; dark skies get boosted chroma so
    -- they read as colour, not grey.
    --   Jewel teal, Neon ink indigo, Vaporwave dusk purple, Inferno night navy,
    --   Toxic bruise purple, Glacier snow grey, Candy cream, Mono teal.
    constant SKY_Y : t_pal8 := p8(502, 119, 216, 177, 159, 740, 740, 502);
    constant SKY_U : t_pal8 := p8(224, 540, 580, 470, 590, 480, 560, 224);   -- = Cr
    constant SKY_V : t_pal8 := p8(604, 620, 640, 610, 600, 560, 420, 604);   -- = Cb

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then       return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else                return unsigned(resize(v, 10));
        end if;
    end function;

    -- Deadbanded pot capture (revenant v0.2): a quantised decode flips on pot
    -- ADC noise at a step boundary and redraws the whole frame.  Accept a new
    -- reading only once it has moved more than KNOB_DB codes.
    constant KNOB_DB : unsigned(9 downto 0) := to_unsigned(24, 10);
    function db_upd(cur : unsigned(9 downto 0); raw : unsigned(9 downto 0))
        return unsigned is
    begin
        if raw > cur then
            if raw - cur > KNOB_DB then return raw; end if;
        else
            if cur - raw > KNOB_DB then return raw; end if;
        end if;
        return cur;
    end function;

    -- Resolution scaling: the pilot's (mondrian) per-mode steps.
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
    -- Line-buffer RAMs (full power-of-2 depth for inference).
    --------------------------------------------------------------------------
    type t_ram4  is array(0 to 2047) of unsigned(3 downto 0);
    type t_ram8  is array(0 to 2047) of unsigned(7 downto 0);
    type t_blk16 is array(0 to 255)  of unsigned(15 downto 0);
    type t_blk8  is array(0 to 255)  of unsigned(7 downto 0);
    signal ram_f  : t_ram4  := (others => (others => '0'));   -- contour band, line above
    signal ram_da : t_ram8  := (others => (others => '0'));   -- drip state, even rows write
    signal ram_db : t_ram8  := (others => (others => '0'));   -- drip state, odd rows write
    signal ram_bc : t_blk16 := (others => x"8080");           -- block chroma u8 & v8
    signal ram_by : t_blk8  := (others => x"10");             -- block luma y8
    signal rd_f   : unsigned(3 downto 0)  := (others => '0');
    signal rd_da, rd_db : unsigned(7 downto 0) := (others => '0');
    signal rd_bc  : unsigned(15 downto 0) := x"8080";
    signal rd_by  : unsigned(7 downto 0)  := x"10";

    --------------------------------------------------------------------------
    -- Raster position / animation.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal px        : unsigned(11 downto 0) := (others => '0');
    signal py        : unsigned(10 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';
    signal frame_cnt : unsigned(9 downto 0) := (others => '0');
    signal vs_armed  : std_logic := '0';   -- serrated-vsync one-shot arm
    signal line_av   : std_logic := '0';   -- current line carried active video

    signal max_px    : unsigned(11 downto 0) := (others => '0');

    -- Resolution scaling (v1.0.2): virtual 1080-line coordinates.  Steps are
    -- decoded free-running from the timing ID (static while a mode runs):
    -- s_vx Q.1 units per pixel (1080 2, 720p 3, SD 4), s_vy Q4.8 virtual lines
    -- per FIELD line, s_cls 0 = 1080 lines, 1 = 720p, 2 = SD.  pxv / pyv move
    -- in lockstep with px / py; at 1080p their integer parts ARE px / py.
    signal s_vx      : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_vy      : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_cls     : unsigned(1 downto 0)  := (others => '0');
    signal pxv       : unsigned(12 downto 0) := (others => '0');   -- Q12.1
    signal pyv       : unsigned(19 downto 0) := (others => '0');   -- Q12.8
    signal bx0_r     : std_logic := '1';   -- this pixel starts a 16-unit block column
    signal by0_r     : std_logic := '1';   -- this row starts a 16-unit block row
    signal s_stepx   : unsigned(15 downto 0) := to_unsigned(4096, 16); -- x DDA phase / px
    signal yfr       : unsigned(7 downto 0)  := (others => '0');   -- row DDA virtual fraction
    signal yn_r      : unsigned(2 downto 0)  := to_unsigned(1, 3); -- virtual lines next row
    signal s_stepn   : unsigned(17 downto 0) := to_unsigned(4096, 18); -- yn_r x s_step
    signal s_pkm     : unsigned(16 downto 0) := (others => '0');
    signal s_pk2     : unsigned(7 downto 0)  := to_unsigned(16, 8);  -- drip odds /256 per field line

    -- xorshift32 noise (period 2^32-1; disjoint bit fields per use:
    -- grain 7..2, drip decay 18..12, row wander 27..22)
    signal xs        : unsigned(31 downto 0) := x"2545F491";
    signal rwalk     : signed(8 downto 0)    := (others => '0');  -- leaky brown-noise wander
    signal rw_step   : signed(8 downto 0)    := (others => '0');
    signal rw_ph     : unsigned(24 downto 0) := (others => '0');  -- rwalk in phase

    -- phase DDAs: pitch = 65536 / s_step px.  Field bits 24..16 = bar index.
    signal xacc      : unsigned(24 downto 0) := (others => '0');
    signal yacc      : unsigned(24 downto 0) := (others => '0');

    -- SPI snapshot + filtered controls
    type t_reg10 is array(0 to 7) of unsigned(9 downto 0);
    signal reg_q     : t_reg10 := (others => (others => '0'));
    signal db_k4     : unsigned(9 downto 0) := (others => '0');
    signal db_p12    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal k5_f      : unsigned(9 downto 0) := to_unsigned(512, 10);  -- K5 glide

    --------------------------------------------------------------------------
    -- Per-frame latched parameters.
    --------------------------------------------------------------------------
    signal s_mask    : unsigned(9 downto 0) := to_unsigned(300,10);   -- mass key
    signal s_blk     : unsigned(9 downto 0) := to_unsigned(64,10);    -- contrast black point
    signal s_sk      : unsigned(7 downto 0) := to_unsigned(96,8);     -- saturation gain /64
    signal s_step    : unsigned(14 downto 0) := to_unsigned(4096,15); -- DDA phase / px
    signal s_palsel  : unsigned(2 downto 0) := (others => '0');       -- palette select
    signal s_fk      : unsigned(5 downto 0) := (others => '0');       -- contour count
    signal s_sky_y   : unsigned(9 downto 0) := to_unsigned(502, 10);  -- sky colour (K4)
    signal s_sky_u   : unsigned(9 downto 0) := to_unsigned(224, 10);
    signal s_sky_v   : unsigned(9 downto 0) := to_unsigned(604, 10);

    signal s_hot     : std_logic := '0';
    signal s_drip    : std_logic := '0';
    signal s_comp    : std_logic := '0';
    signal s_horiz   : std_logic := '0';

    -- P12-derived (continuous)
    signal s_bk      : unsigned(7 downto 0) := to_unsigned(80, 8);   -- luma bend gain
    signal s_rk      : unsigned(6 downto 0) := to_unsigned(40, 7);   -- ripple gain
    signal s_jk      : unsigned(5 downto 0) := to_unsigned(20, 6);   -- row-wander gain
    signal s_tk      : unsigned(3 downto 0) := to_unsigned(4, 4);    -- edge-tear gain
    signal s_pk      : unsigned(6 downto 0) := to_unsigned(8, 7);    -- drip decay odds /128
    signal s_chroff  : integer range 0 to CHR_MAX := 0;              -- chroma fringe (px)
    signal s_noisem  : unsigned(5 downto 0) := (others => '0');
    signal s_hotkey  : unsigned(9 downto 0) := to_unsigned(900,10);  -- raw-luma hot key
    signal s_halokey : unsigned(9 downto 0) := to_unsigned(780,10);
    signal s_bloom   : unsigned(9 downto 0) := (others => '0');
    signal s_qmask   : unsigned(9 downto 0) := (others => '0');      -- compress luma quantize

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

    --------------------------------------------------------------------------
    -- Datapath stage registers.  The carried bundle cr(k) travels with the
    -- pixel: px (11b column), py0 (row parity), top (first active row),
    -- bcol/bx0/by0 (macroblock position).
    --------------------------------------------------------------------------
    type t_carry is record
        px   : unsigned(10 downto 0);
        py0  : std_logic;
        top  : std_logic;
        bcol : unsigned(7 downto 0);
        bx0  : std_logic;
        by0  : std_logic;
    end record;
    constant C_CARRY0 : t_carry := ((others => '0'), '0', '0', (others => '0'), '0', '0');
    type t_carry_arr is array(1 to NSTG) of t_carry;
    signal cr : t_carry_arr := (others => C_CARRY0);

    -- s1
    signal r1_lsub  : signed(11 downto 0) := (others => '0');
    signal r1_raw   : unsigned(9 downto 0) := (others => '0');
    signal r1_base  : unsigned(24 downto 0) := (others => '0');
    signal r1_curve : signed(9 downto 0) := (others => '0');
    signal r1_tx    : unsigned(8 downto 0) := (others => '0');
    -- s2
    signal r2_lp    : signed(13 downto 0) := (others => '0');
    signal r2_ripm  : signed(17 downto 0) := (others => '0');
    signal r2_hot, r2_halo : std_logic := '0';
    signal r2_base  : unsigned(24 downto 0) := (others => '0');
    signal r2_tx    : unsigned(8 downto 0) := (others => '0');
    -- s3
    signal r3_lc    : unsigned(9 downto 0) := (others => '0');
    signal r3_rip   : unsigned(24 downto 0) := (others => '0');
    signal r3_hot, r3_halo : std_logic := '0';
    signal r3_base  : unsigned(24 downto 0) := (others => '0');
    signal r3_tx    : unsigned(8 downto 0) := (others => '0');
    -- s4
    signal r4_br    : unsigned(24 downto 0) := (others => '0');
    signal r4_plo, r4_phi : unsigned(11 downto 0) := (others => '0');
    signal r4_bprod : unsigned(15 downto 0) := (others => '0');
    signal r4_th    : unsigned(5 downto 0) := (others => '0');
    signal r4_lc    : unsigned(9 downto 0) := (others => '0');
    signal r4_hot, r4_halo : std_logic := '0';
    -- s5
    signal r5_br    : unsigned(24 downto 0) := (others => '0');
    signal r5_bend  : unsigned(15 downto 0) := (others => '0');
    signal r5_a     : unsigned(9 downto 0) := (others => '0');
    signal r5_g     : unsigned(8 downto 0) := (others => '0');
    signal r5_l64   : unsigned(9 downto 0) := C_BLACK;   -- 64 + lc>>4
    signal r5_tear  : signed(10 downto 0) := (others => '0');
    signal r5_band, r5_left, r5_abv : unsigned(3 downto 0) := (others => '0');
    signal r5_lc    : unsigned(9 downto 0) := (others => '0');
    signal r5_hot, r5_halo : std_logic := '0';
    -- s6
    signal r6_f     : unsigned(24 downto 0) := (others => '0');
    signal r6_by, r6_gy : unsigned(9 downto 0) := C_BLACK;
    signal r6_thr   : signed(11 downto 0) := (others => '0');
    signal r6_lc    : unsigned(9 downto 0) := (others => '0');
    signal r6_hot, r6_halo, r6_fil : std_logic := '0';
    -- s7
    signal r7_idx   : unsigned(2 downto 0) := (others => '0');
    signal r7_streak, r7_mass : std_logic := '0';
    signal r7_by, r7_gy : unsigned(9 downto 0) := C_BLACK;
    signal r7_lc4   : unsigned(3 downto 0) := (others => '0');
    signal r7_da, r7_db : unsigned(7 downto 0) := (others => '0');
    signal r7_hot, r7_halo, r7_fil : std_logic := '0';
    -- s8: drip compare
    signal r8_take  : std_logic := '0';
    signal r8_idxa  : unsigned(2 downto 0) := (others => '0');
    signal r8_sta   : std_logic := '0';
    signal r8_ea2, r8_ecur : unsigned(3 downto 0) := (others => '0');
    signal r8_bydr, r8_gydr : unsigned(9 downto 0) := C_BLACK;
    signal r8_idx   : unsigned(2 downto 0) := (others => '0');
    signal r8_streak, r8_mass : std_logic := '0';
    signal r8_by, r8_gy : unsigned(9 downto 0) := C_BLACK;
    signal r8_hot, r8_halo, r8_fil : std_logic := '0';
    -- s9: drip substitute
    signal r9_state : unsigned(7 downto 0) := (others => '0');
    signal r9_idx   : unsigned(2 downto 0) := (others => '0');
    signal r9_streak, r9_mass : std_logic := '0';
    signal r9_by, r9_gy : unsigned(9 downto 0) := C_BLACK;
    signal r9_hot, r9_halo, r9_fil : std_logic := '0';
    -- s10: palette lookup
    signal r10_du, r10_dv : signed(10 downto 0) := (others => '0');
    signal r10_streak, r10_mass : std_logic := '0';
    signal r10_by, r10_gy : unsigned(9 downto 0) := C_BLACK;
    signal r10_hot, r10_halo, r10_fil : std_logic := '0';
    -- s11: saturation product halves
    signal r11_ulo, r11_uhi, r11_vlo, r11_vhi : signed(15 downto 0) := (others => '0');
    signal r11_streak, r11_mass : std_logic := '0';
    signal r11_by, r11_gy : unsigned(9 downto 0) := C_BLACK;
    signal r11_hot, r11_halo, r11_fil : std_logic := '0';
    -- s12
    signal r12_su, r12_sv : signed(13 downto 0) := (others => '0');
    signal r12_hy, r12_sy : unsigned(10 downto 0) := (others => '0');
    signal r12_streak, r12_mass : std_logic := '0';
    signal r12_gy   : unsigned(9 downto 0) := C_BLACK;
    signal r12_hot, r12_halo, r12_fil : std_logic := '0';
    -- s13
    signal r13_cu, r13_cv, r13_gu, r13_gv : unsigned(9 downto 0) := C_MID;
    signal r13_hy, r13_sy, r13_gy : unsigned(9 downto 0) := C_BLACK;
    signal r13_streak, r13_mass : std_logic := '0';
    signal r13_hot, r13_halo, r13_fil : std_logic := '0';
    -- s14: select
    signal r14_y    : unsigned(9 downto 0) := C_BLACK;
    signal r14_u, r14_v : unsigned(9 downto 0) := C_MID;
    -- s15..s18: macroblock compress
    signal r15_y, r15_yq : unsigned(9 downto 0) := C_BLACK;
    signal r15_u, r15_v : unsigned(9 downto 0) := C_MID;
    signal r16_y, r16_yq : unsigned(9 downto 0) := C_BLACK;
    signal r16_u, r16_v : unsigned(9 downto 0) := C_MID;
    signal r16_bc   : unsigned(15 downto 0) := x"8080";
    signal r16_by   : unsigned(7 downto 0)  := x"10";
    signal r17_y, r17_yq : unsigned(9 downto 0) := C_BLACK;
    signal r17_u, r17_v : unsigned(9 downto 0) := C_MID;
    signal r17_hold : unsigned(23 downto 0) := x"108080";   -- y8 & u8 & v8
    signal r18_y    : unsigned(10 downto 0) := to_unsigned(128, 11);   -- 2 x luma
    signal r18_u, r18_v : unsigned(9 downto 0) := C_MID;
    signal r_noise  : signed(7 downto 0) := (others => '0');   -- registered grain

    -- chroma-separation delay lines (s19 writes tap 0).
    type t_chr is array(0 to 2*CHR_MAX) of unsigned(9 downto 0);
    signal yd : t_chr := (others => C_BLACK);
    signal ud : t_chr := (others => C_MID);
    signal vd : t_chr := (others => C_MID);

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Raster position, animation counters, per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_x1, v_x2 : unsigned(31 downto 0);
        variable v_pi, v_dz : integer;
        variable v_k5raw : unsigned(9 downto 0);
        variable v_m9    : unsigned(8 downto 0);
        variable v_yn    : unsigned(11 downto 0);
        variable v_pxv   : unsigned(12 downto 0);
        variable v_pyv   : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- xorshift32 (13, 17, 5)
            v_x1 := xs xor shift_left(xs, 13);
            v_x2 := v_x1 xor shift_right(v_x1, 17);
            xs   <= v_x2 xor shift_left(v_x2, 5);

            -- SPI snapshot: keeps SPI routing out of the frame-latch cone.
            for i in 0 to 7 loop
                reg_q(i) <= unsigned(registers_in(i)(9 downto 0));
            end loop;

            -- Resolution-scaling steps from the timing ID, free-running, and
            -- the scaled per-frame steps (register-in / register-out, they
            -- settle in the blanking after the frame latch; 1080p = identity).
            s_vx <= f_vstep_x(registers_in(8)(3 downto 0));
            s_vy <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_cls <= "10";
                when 5 | 6 | 14     => s_cls <= "01";
                when others         => s_cls <= "00";
            end case;
            -- x DDA: pitch is a size -> 1.5x / 2x phase per pixel
            case s_cls is
                when "00"   => s_stepx <= resize(s_step, 16);
                when "01"   => s_stepx <= resize(s_step, 16) + resize(shift_right(s_step, 1), 16);
                when others => s_stepx <= shift_left(resize(s_step, 16), 1);
            end case;
            -- row DDA (Horizontal): one s_step per VIRTUAL line; yn_r = the
            -- virtual lines the next active field line covers (1..5)
            v_yn := resize(yfr, 12) + s_vy;
            yn_r <= v_yn(10 downto 8);
            case yn_r is
                when "001"  => s_stepn <= resize(s_step, 18);
                when "010"  => s_stepn <= shift_left(resize(s_step, 18), 1);
                when "011"  => s_stepn <= resize(s_step, 18) + shift_left(resize(s_step, 18), 1);
                when "100"  => s_stepn <= shift_left(resize(s_step, 18), 2);
                when others => s_stepn <= resize(s_step, 18) + shift_left(resize(s_step, 18), 2);
            end case;
            -- drip decay odds per FIELD line = odds per virtual line x lines
            -- covered (/256; 1080p = 2 x s_pk, the old /128 test exactly)
            s_pkm <= s_pk(4 downto 0) * s_vy;
            if s_pkm(16 downto 15) /= "00" then
                s_pk2 <= (others => '1');
            else
                s_pk2 <= s_pkm(14 downto 7);
            end if;

            -- free-running per-line helpers (registered so the per-line update
            -- below is a single add)
            rw_step <= resize(shift_right(signed(xs(27 downto 22)) * signed('0' & s_jk), 6), 9);
            if s_horiz = '1' then
                rw_ph <= (others => '0');          -- a row wander would shatter H bars
            else
                rw_ph <= unsigned(shift_left(resize(rwalk, 25), 12));
            end if;

            -- deadbanded zone decodes, refreshed once per line
            if v_h_edge = '1' then
                db_k4  <= db_upd(db_k4,  reg_q(3));
                db_p12 <= db_upd(db_p12, reg_q(7));
            end if;

            if data_in.avid = '1' then vs_armed <= '1'; end if;

            -- X counter + per-line work
            if v_h_edge = '1' then
                line_av <= '0';
                if line_av = '1' then
                    if px > max_px then max_px <= px; end if;
                end if;
                px    <= (others => '0');
                pxv   <= (others => '0');
                bx0_r <= '1';
                xacc  <= (others => '0');
                -- leaky random walk -> smooth organic row wander (brown noise)
                rwalk <= rwalk - shift_right(rwalk, 2) + rw_step;
            elsif data_in.avid = '1' then
                line_av <= '1';
                px   <= px + 1;
                xacc <= xacc + s_stepx;
                v_pxv := pxv + s_vx;
                pxv   <= v_pxv;
                if v_pxv(12 downto 5) /= pxv(12 downto 5) then
                    bx0_r <= '1';
                else
                    bx0_r <= '0';
                end if;
            end if;

            -- row DDA for Horizontal sort: advances on ACTIVE lines only; the
            -- bottom field starts half a row-step in (interlace).
            if data_in.avid = '1' and frame_act = '0' and v_v_edge = '0' then
                if data_in.field_n = '0' then
                    yacc <= resize(shift_right(s_step, 1), 25);
                else
                    yacc <= (others => '0');
                end if;
                yfr <= (others => '0');
            elsif v_h_edge = '1' and line_av = '1' then
                yacc <= yacc + s_stepn;
                yfr  <= v_yn(7 downto 0);
            end if;

            -- Y counter (anchored to first active line)
            if v_v_edge = '1' then
                py        <= (others => '0');
                pyv       <= (others => '0');
                by0_r     <= '1';
                frame_act <= '0';

                -- once per field (serrated vsync fires many edges)
                if vs_armed = '1' then
                    vs_armed  <= '0';
                    frame_cnt <= frame_cnt + 1;

                    -- measured active size (captured on active lines above)
                    if max_px > 64 then g_wm1 <= max_px - 1; end if;
                    max_px <= (others => '0');

                    -- K5 glide: a DDA step must not jitter (1 LSB = visible
                    -- phase error at the right edge); ease, never snap.
                    v_k5raw := reg_q(4);
                    if v_k5raw > k5_f then
                        k5_f <= k5_f + shift_right(v_k5raw - k5_f, 4);
                    elsif v_k5raw < k5_f then
                        k5_f <= k5_f - shift_right(k5_f - v_k5raw, 4);
                    end if;

                    ---------------------------------------------------------------
                    -- Parameter latch.
                    ---------------------------------------------------------------
                    s_mask   <= reg_q(0);                              -- K1 Mass Key
                    s_blk    <= resize(reg_q(1)(9 downto 2), 10);      -- K2 black point 0..255
                    s_sk     <= reg_q(2)(9 downto 2);                  -- K3 gain/64: 0..4x
                    s_palsel <= db_k4(9 downto 7);                     -- K4 palette (8)
                    s_sky_y  <= SKY_Y(to_integer(db_k4(9 downto 7)));  -- + its sky
                    s_sky_u  <= SKY_U(to_integer(db_k4(9 downto 7)));
                    s_sky_v  <= SKY_V(to_integer(db_k4(9 downto 7)));
                    -- K5 Line Spacing: step = (256+frac) << (2+zone) = 1024..16352
                    -- -> pitch 64 .. 4 px on an exponential sweep (each quarter of
                    -- the knob halves the pitch).
                    v_m9 := '1' & k5_f(7 downto 0);
                    case k5_f(9 downto 8) is
                        when "00"   => s_step <= shift_left(resize(v_m9, 15), 2);
                        when "01"   => s_step <= shift_left(resize(v_m9, 15), 3);
                        when "10"   => s_step <= shift_left(resize(v_m9, 15), 4);
                        when others => s_step <= shift_left(resize(v_m9, 15), 5);
                    end case;
                    s_fk <= reg_q(5)(9 downto 4);                      -- K6 contour count

                    s_hot   <= reg_q(6)(1);    -- S8
                    s_drip  <= reg_q(6)(2);    -- S9
                    s_comp  <= reg_q(6)(3);    -- S10
                    s_horiz <= reg_q(6)(4);    -- S11

                    -- P12 master chaos -> continuous per-effect gains.
                    v_pi := to_integer(reg_q(7));
                    s_bk      <= to_unsigned(16 + v_pi / 8, 8);        -- bend 2..18 pitches
                    s_rk      <= to_unsigned(8 + v_pi / 16, 7);        -- ripple 8..71
                    s_jk      <= to_unsigned(4 + v_pi / 32, 6);        -- row wander 4..35
                    s_tk      <= to_unsigned(1 + v_pi / 128, 4);       -- edge tear 1..8
                    s_pk      <= to_unsigned(16 - v_pi / 64, 7);       -- drip decay 16..1 /128
                    s_noisem  <= to_unsigned(v_pi / 32, 6);            -- grain 0..31
                    s_hotkey  <= to_unsigned(900 - v_pi / 8 - v_pi / 16, 10);  -- 900..709
                    s_halokey <= to_unsigned(780 - v_pi / 8 - v_pi / 16, 10);
                    s_bloom   <= to_unsigned(v_pi / 8, 10);            -- 0..127
                    -- stepped effects use the deadbanded fader
                    v_dz := to_integer(db_p12) / 256;                  -- 0..3
                    if reg_q(6)(0) = '1' then                          -- S7
                        -- 4..7 px at 1080p, x2/3 at 720p, x1/2 at SD (rounded)
                        case s_cls is
                            when "00" =>
                                s_chroff <= 4 + v_dz;                  -- 4..7 px
                            when "01" =>
                                case v_dz is
                                    when 0 | 1  => s_chroff <= 3;
                                    when 2      => s_chroff <= 4;
                                    when others => s_chroff <= 5;
                                end case;
                            when others =>
                                case v_dz is
                                    when 0      => s_chroff <= 2;
                                    when 1 | 2  => s_chroff <= 3;
                                    when others => s_chroff <= 4;
                                end case;
                        end case;
                    else
                        s_chroff <= 0;
                    end if;
                    s_qmask <= to_unsigned((2 ** (3 + v_dz)) - 1, 10); -- 7..63
                end if;

            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                py        <= (others => '0');
                pyv       <= (others => '0');
                by0_r     <= '1';
            elsif v_h_edge = '1' then
                py <= py + 1;
                v_pyv := pyv + s_vy;
                pyv   <= v_pyv;
                if v_pyv(19 downto 12) /= pyv(19 downto 12) then
                    by0_r <= '1';
                else
                    by0_r <= '0';
                end if;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Clean avid (cubist/aurora recipe).  The encoders get no data-enable and
    -- pair Cb/Cr by counting from hsync, so an avid that wanders a clock
    -- against hsync swaps U/V for that line.  g_ph is this line's
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
    -- Line-buffer RAMs: one write statement and one registered read each.
    --------------------------------------------------------------------------
    -- contour band of the line above: read at s4 (pixel n+1) while pixel n is
    -- written, so every read precedes its own overwrite (single bank).
    p_ram_f : process(clk)
    begin
        if rising_edge(clk) then
            ram_f(to_integer(cr(4).px)) <= r4_bprod(13 downto 10);
            rd_f <= ram_f(to_integer(cr(3).px));
        end if;
    end process p_ram_f;

    -- drip state ping-pong: even rows (py0='0') write A / read B, odd rows
    -- write B / read A.  Read issued at s6, written at s10.
    p_ram_da : process(clk)
    begin
        if rising_edge(clk) then
            if cr(9).py0 = '0' then
                ram_da(to_integer(cr(9).px)) <= r9_state;
            end if;
            rd_da <= ram_da(to_integer(cr(5).px));
        end if;
    end process p_ram_da;

    p_ram_db : process(clk)
    begin
        if rising_edge(clk) then
            if cr(9).py0 = '1' then
                ram_db(to_integer(cr(9).px)) <= r9_state;
            end if;
            rd_db <= ram_db(to_integer(cr(5).px));
        end if;
    end process p_ram_db;

    -- macroblock colour: written on a block's first row at its first column
    -- (s17), read on its other rows (s15).  Row-0 reads are ignored, so the
    -- undefined same-address read-during-write never reaches the picture.
    p_ram_b : process(clk)
    begin
        if rising_edge(clk) then
            if cr(16).bx0 = '1' and cr(16).by0 = '1' then
                ram_bc(to_integer(cr(16).bcol)) <= r16_u(9 downto 2) & r16_v(9 downto 2);
                ram_by(to_integer(cr(16).bcol)) <= r16_y(9 downto 2);
            end if;
            rd_bc <= ram_bc(to_integer(cr(14).bcol));
            rd_by <= ram_by(to_integer(cr(14).bcol));
        end if;
    end process p_ram_b;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_a, v_b   : unsigned(7 downto 0);
        variable v_ta, v_tb : signed(8 downto 0);
        variable v_t10      : unsigned(9 downto 0);
        variable v_x9       : unsigned(8 downto 0);
        variable v_w        : signed(19 downto 0);
        variable v_u, v_v   : signed(14 downto 0);
        variable v_dimy     : signed(13 downto 0);
        variable v_lb       : unsigned(7 downto 0);
        variable v_ea, v_ea2, v_ecur : unsigned(3 downto 0);
        variable v_e10      : unsigned(9 downto 0);
        variable v_hold     : unsigned(23 downto 0);
        variable v_hy       : unsigned(9 downto 0);
        variable v_c        : t_carry;
        variable v_r8       : unsigned(7 downto 0);
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

            -- carried position bundle
            v_c.px  := px(10 downto 0);
            v_c.py0 := py(0);
            if py = 0 then v_c.top := '1'; else v_c.top := '0'; end if;
            -- macroblocks: 16 x 16 VIRTUAL units in every mode (block-start
            -- flags formed with the counters: a fractional step can skip a
            -- "low bits = 0" value)
            v_c.bcol := pxv(12 downto 5);
            v_c.bx0  := bx0_r;
            v_c.by0  := by0_r;
            cr(1) <= v_c;
            for i in 2 to NSTG loop cr(i) <= cr(i - 1); end loop;

            -----------------------------------------------------------------
            -- s1: capture luma; DDA base; ripple waves; tear hash round 1.
            -----------------------------------------------------------------
            r1_lsub <= signed(resize(unsigned(data_in.y), 12)) - signed(resize(s_blk, 12));
            r1_raw  <= unsigned(data_in.y);
            if s_horiz = '1' then r1_base <= yacc; else r1_base <= xacc; end if;
            -- organic bend: two diagonal triangle waves mixing px & py at
            -- different angles/periods (rounded, column-varying curves).
            -- (virtual coordinates: the same waves at every resolution)
            v_a := resize(pyv(16 downto 9), 8) + resize(pxv(10 downto 6), 8)
                 + resize(frame_cnt(6 downto 0), 8);
            if v_a(7) = '1' then v_a := not v_a; end if;          -- fold -> 0..127
            v_b := resize(pyv(15 downto 8), 8) - resize(pxv(9 downto 5), 8)
                 + resize(frame_cnt(5 downto 0), 8);
            if v_b(7) = '1' then v_b := not v_b; end if;
            v_ta := signed(resize(v_a(6 downto 0), 9)) - 64;      -- -64..63
            v_tb := signed(resize(v_b(6 downto 0), 9)) - 64;
            r1_curve <= resize(v_ta, 10) + resize(shift_right(v_tb, 1), 10);
            -- torn-edge column hash (4-unit virtual columns), spread over s1..s4:
            -- x*0x6D+0x3B; ^>>4; *0x35; ^>>5; top 6 bits.  Spot-checked: lag
            -- correlations <= 0.12, no linear runs.
            r1_tx <= resize(pxv(11 downto 3) * to_unsigned(16#6D#, 7), 9) + to_unsigned(16#3B#, 9);

            -----------------------------------------------------------------
            -- s2: contrast crush x1.5; ripple gain; hot key (RAW luma: the
            -- crushed luma clamps flat over every lit area).
            -----------------------------------------------------------------
            r2_lp   <= resize(r1_lsub, 14) + resize(shift_right(r1_lsub, 1), 14);
            r2_ripm <= r1_curve * signed('0' & s_rk);
            r2_hot <= '0'; r2_halo <= '0';
            if s_hot = '1' then
                if r1_raw >= s_hotkey then
                    r2_hot <= '1';
                elsif r1_raw >= s_halokey then
                    r2_halo <= '1';
                end if;
            end if;
            r2_tx   <= r1_tx xor shift_right(r1_tx, 4);
            r2_base <= r1_base;

            -----------------------------------------------------------------
            -- s3: crushed luma; ripple phase.
            -----------------------------------------------------------------
            r3_lc  <= clamp10(resize(r2_lp, 13));
            r3_rip <= unsigned(shift_left(resize(r2_ripm, 25), 9)) + rw_ph;
            r3_tx  <= resize(r2_tx * to_unsigned(16#35#, 6), 9);
            r3_hot <= r2_hot; r3_halo <= r2_halo; r3_base <= r2_base;

            -----------------------------------------------------------------
            -- s4: base+ripple; bend product halves; contour band product.
            -----------------------------------------------------------------
            r4_br    <= r3_base + r3_rip;
            r4_plo   <= r3_lc(9 downto 2) * s_bk(3 downto 0);
            r4_phi   <= r3_lc(9 downto 2) * s_bk(7 downto 4);
            r4_bprod <= r3_lc * s_fk;                      -- band = bits 13..10
            v_x9     := r3_tx xor shift_right(r3_tx, 5);
            r4_th    <= v_x9(8 downto 3);
            r4_lc <= r3_lc; r4_hot <= r3_hot; r4_halo <= r3_halo;

            -----------------------------------------------------------------
            -- s5: bend; bar/gap luma partials; tear; contour neighbours.
            -----------------------------------------------------------------
            r5_bend <= shift_left(resize(r4_phi, 16), 4) + resize(r4_plo, 16);
            r5_a    <= resize(r4_lc(9 downto 1), 10) + resize(r4_lc(9 downto 3), 10);  -- lc*5/8
            r5_g    <= resize(r4_lc(9 downto 3), 9) + resize(r4_lc(9 downto 4), 9);    -- lc*3/16
            v_t10   := resize(r4_lc(9 downto 4), 10);
            v_t10(6) := '1';                                   -- + 64 (lc>>4 < 64)
            r5_l64  <= v_t10;
            r5_tear <= resize((signed('0' & r4_th) - 32) * signed('0' & s_tk), 11);
            r5_band <= r4_bprod(13 downto 10);
            r5_left <= r5_band;                                -- previous pixel's band
            r5_abv  <= rd_f;                                   -- line above (fabric reg)
            r5_br   <= r4_br;
            r5_lc   <= r4_lc; r5_hot <= r4_hot; r5_halo <= r4_halo;

            -----------------------------------------------------------------
            -- s6: full field phase; bar luma 64 + lc*11/16 (<= 765), gap luma;
            -- mass threshold; contour filament = band change left or above.
            -----------------------------------------------------------------
            r6_f   <= r5_br + shift_left(resize(r5_bend, 25), 5);
            r6_by  <= r5_a + r5_l64;
            r6_gy  <= resize(r5_g, 10) + C_BLACK;
            r6_thr <= signed(resize(s_mask, 12)) + resize(r5_tear, 12);
            if s_fk /= 0 and cr(5).top = '0'
               and (r5_band /= r5_left or r5_band /= r5_abv) then
                r6_fil <= '1';
            else
                r6_fil <= '0';
            end if;
            r6_lc <= r5_lc; r6_hot <= r5_hot; r6_halo <= r5_halo;

            -----------------------------------------------------------------
            -- s7: bar index hash -> hue; 3/4-duty bar; mass; drip reads in.
            -----------------------------------------------------------------
            r7_idx    <= r6_f(18 downto 16) xor r6_f(21 downto 19) xor r6_f(24 downto 22);
            r7_streak <= not (r6_f(15) and r6_f(14));
            if signed(resize(r6_lc, 12)) >= r6_thr then r7_mass <= '1'; else r7_mass <= '0'; end if;
            r7_lc4 <= r6_lc(9 downto 6);
            r7_da  <= rd_da; r7_db <= rd_db;
            r7_by <= r6_by; r7_gy <= r6_gy;
            r7_hot <= r6_hot; r7_halo <= r6_halo; r7_fil <= r6_fil;

            -----------------------------------------------------------------
            -- s8: drip compare.  The column state from the line above decays by
            -- one with odds s_pk/128 and wins if it beats this pixel's energy.
            -----------------------------------------------------------------
            if cr(7).py0 = '0' then v_lb := r7_db; else v_lb := r7_da; end if;
            v_ea := v_lb(3 downto 0);
            if s_drip = '0' or cr(7).top = '1' then v_ea := (others => '0'); end if;
            -- odds per field line s_pk2/256 (1080p: exactly the old
            -- xs(18 downto 12) < s_pk test; bit 11 is otherwise unused)
            v_r8 := xs(18 downto 11);
            if v_ea /= 0 and v_r8 < s_pk2 then
                v_ea2 := v_ea - 1;
            else
                v_ea2 := v_ea;
            end if;
            if r7_mass = '1' then v_ecur := r7_lc4; else v_ecur := (others => '0'); end if;
            if v_ea2 > v_ecur then r8_take <= '1'; else r8_take <= '0'; end if;
            r8_idxa <= v_lb(7 downto 5);
            r8_sta  <= v_lb(4);
            r8_ea2  <= v_ea2;
            r8_ecur <= v_ecur;
            v_e10   := resize(v_lb(3 downto 0), 10);
            r8_bydr <= shift_left(v_e10, 5) + shift_left(v_e10, 3) + C_BLACK;   -- 64 + e*40
            r8_gydr <= shift_left(v_e10, 3) + shift_left(v_e10, 1) + C_BLACK;   -- 64 + e*10
            r8_idx <= r7_idx; r8_streak <= r7_streak; r8_mass <= r7_mass;
            r8_by  <= r7_by;  r8_gy <= r7_gy;
            r8_hot <= r7_hot; r8_halo <= r7_halo; r8_fil <= r7_fil;

            -----------------------------------------------------------------
            -- s9: drip substitute + new column state (written at s10).
            -----------------------------------------------------------------
            if r8_take = '1' then
                r9_idx <= r8_idxa; r9_streak <= r8_sta; r9_mass <= '1';
                r9_by  <= r8_bydr; r9_gy <= r8_gydr;
                r9_hot <= '0'; r9_halo <= '0'; r9_fil <= '0';
                r9_state <= r8_idxa & r8_sta & r8_ea2;
            else
                r9_idx <= r8_idx; r9_streak <= r8_streak; r9_mass <= r8_mass;
                r9_by  <= r8_by;  r9_gy <= r8_gy;
                r9_hot <= r8_hot; r9_halo <= r8_halo; r9_fil <= r8_fil;
                r9_state <= r8_idx & r8_streak & r8_ecur;
            end if;

            -----------------------------------------------------------------
            -- s10: palette chroma lookup.
            -----------------------------------------------------------------
            r10_du <= signed(resize(PAL_U(to_integer(s_palsel))(to_integer(r9_idx)), 11)) - 512;
            r10_dv <= signed(resize(PAL_V(to_integer(s_palsel))(to_integer(r9_idx)), 11)) - 512;
            r10_streak <= r9_streak; r10_mass <= r9_mass;
            r10_by <= r9_by; r10_gy <= r9_gy;
            r10_hot <= r9_hot; r10_halo <= r9_halo; r10_fil <= r9_fil;

            -----------------------------------------------------------------
            -- s11: saturation product halves (du * s_sk, split hi/lo nibble).
            -----------------------------------------------------------------
            r11_ulo <= r10_du * signed('0' & s_sk(3 downto 0));
            r11_uhi <= r10_du * signed('0' & s_sk(7 downto 4));
            r11_vlo <= r10_dv * signed('0' & s_sk(3 downto 0));
            r11_vhi <= r10_dv * signed('0' & s_sk(7 downto 4));
            r11_streak <= r10_streak; r11_mass <= r10_mass;
            r11_by <= r10_by; r11_gy <= r10_gy;
            r11_hot <= r10_hot; r11_halo <= r10_halo; r11_fil <= r10_fil;

            -----------------------------------------------------------------
            -- s12: sum products, /64 -> gain 0..4x (unity at K3 25%); bloom.
            -----------------------------------------------------------------
            v_w := shift_left(resize(r11_uhi, 20), 4) + resize(r11_ulo, 20);
            r12_su <= resize(shift_right(v_w, 6), 14);
            v_w := shift_left(resize(r11_vhi, 20), 4) + resize(r11_vlo, 20);
            r12_sv <= resize(shift_right(v_w, 6), 14);
            r12_hy <= resize(r11_by, 11) + resize(s_bloom, 11);
            r12_sy <= resize(r11_by, 11) + resize(shift_right(s_bloom, 1), 11);
            r12_streak <= r11_streak; r12_mass <= r11_mass; r12_gy <= r11_gy;
            r12_hot <= r11_hot; r12_halo <= r11_halo; r12_fil <= r11_fil;

            -----------------------------------------------------------------
            -- s13: clamp chroma; cap coloured luma at 768.
            -----------------------------------------------------------------
            v_u := to_signed(512, 15) + resize(r12_su, 15);
            v_v := to_signed(512, 15) + resize(r12_sv, 15);
            r13_cu <= clamp10(v_u);
            r13_cv <= clamp10(v_v);
            r13_gu <= clamp10(to_signed(512, 15) + resize(shift_right(r12_su, 1), 15));
            r13_gv <= clamp10(to_signed(512, 15) + resize(shift_right(r12_sv, 1), 15));
            if r12_hy > resize(Y_CAP, 11) then r13_hy <= Y_CAP; else r13_hy <= r12_hy(9 downto 0); end if;
            if r12_sy > resize(Y_CAP, 11) then r13_sy <= Y_CAP; else r13_sy <= r12_sy(9 downto 0); end if;
            r13_gy <= r12_gy; r13_streak <= r12_streak; r13_mass <= r12_mass;
            r13_hot <= r12_hot; r13_halo <= r12_halo; r13_fil <= r12_fil;

            -----------------------------------------------------------------
            -- s14: select.
            --   sky       -> outside the mass (and not dripped on)
            --   hot/halo  -> white cores + bloom
            --   filament  -> white contour veins
            --   streak    -> bar: live luma + palette chroma
            --   gap       -> dark shadow between bars
            -----------------------------------------------------------------
            if r13_mass = '0' then
                r14_y <= s_sky_y; r14_u <= s_sky_u; r14_v <= s_sky_v;
            elsif r13_hot = '1' then
                r14_y <= to_unsigned(940, 10); r14_u <= C_MID; r14_v <= C_MID;
            elsif r13_fil = '1' then
                r14_y <= to_unsigned(900, 10); r14_u <= C_MID; r14_v <= C_MID;
            elsif r13_halo = '1' then
                r14_y <= r13_hy; r14_u <= r13_cu; r14_v <= r13_cv;
            elsif r13_streak = '1' then
                r14_y <= r13_sy; r14_u <= r13_cu; r14_v <= r13_cv;
            else
                r14_y <= r13_gy; r14_u <= r13_gu; r14_v <= r13_gv;
            end if;

            -----------------------------------------------------------------
            -- s15: luma posterize (block RAM read issued this edge).
            -----------------------------------------------------------------
            r15_y  <= r14_y; r15_u <= r14_u; r15_v <= r14_v;
            r15_yq <= r14_y and not s_qmask;

            -----------------------------------------------------------------
            -- s16: block RAM data into fabric.
            -----------------------------------------------------------------
            r16_bc <= rd_bc; r16_by <= rd_by;
            r16_y  <= r15_y; r16_yq <= r15_yq; r16_u <= r15_u; r16_v <= r15_v;

            -----------------------------------------------------------------
            -- s17: block hold.  A block's first column loads its colour: live
            -- on the block's first row (which also writes the RAM), stored on
            -- the rows below.
            -----------------------------------------------------------------
            v_hold := r17_hold;
            if cr(16).bx0 = '1' then
                if cr(16).by0 = '1' then
                    v_hold := r16_y(9 downto 2) & r16_u(9 downto 2) & r16_v(9 downto 2);
                else
                    v_hold := r16_by & r16_bc;
                end if;
            end if;
            r17_hold <= v_hold;
            r17_y <= r16_y; r17_yq <= r16_yq; r17_u <= r16_u; r17_v <= r16_v;

            -----------------------------------------------------------------
            -- s18: compress mix: block chroma; luma = (posterized + block)/2.
            -- r18_y carries 2x luma (11 bits).
            -----------------------------------------------------------------
            if s_comp = '1' then
                v_hy  := shift_left(resize(r17_hold(23 downto 16), 10), 2);
                r18_y <= resize(r17_yq, 11) + resize(v_hy, 11);
                r18_u <= shift_left(resize(r17_hold(15 downto 8), 10), 2);
                r18_v <= shift_left(resize(r17_hold(7 downto 0), 10), 2);
            else
                r18_y <= shift_left(resize(r17_y, 11), 1);
                r18_u <= r17_u; r18_v <= r17_v;
            end if;
            -- grain term, registered free-running (rides chaos)
            r_noise <= signed(resize(xs(7 downto 2) and s_noisem, 8))
                     - signed(resize('0' & s_noisem(5 downto 1), 8));

            -----------------------------------------------------------------
            -- s19: grain + clamp to 64..1023, then the chroma-separation delay.
            -----------------------------------------------------------------
            v_dimy := signed(resize(r18_y(10 downto 1), 14)) + resize(r_noise, 14);
            if v_dimy < 64 then
                yd(0) <= C_BLACK;
            else
                yd(0) <= clamp10(v_dimy);
            end if;
            ud(0) <= r18_u;
            vd(0) <= r18_v;
            for i in 1 to 2*CHR_MAX loop
                yd(i) <= yd(i-1); ud(i) <= ud(i-1); vd(i) <= vd(i-1);
            end loop;

            -----------------------------------------------------------------
            -- output: chroma sep pulls U ahead / pushes V back.  Blanking gate:
            -- neutral outside the (generated) active window.
            -----------------------------------------------------------------
            if gv_d(LATENCY - 1) = '0' then
                s_io.y <= std_logic_vector(C_BLACK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(yd(CHR_MAX));
                s_io.u <= std_logic_vector(ud(CHR_MAX - s_chroff));
                s_io.v <= std_logic_vector(vd(CHR_MAX + s_chroff));
            end if;
            s_io.hsync_n <= hs_d(LATENCY - 1);
            s_io.vsync_n <= vs_d(LATENCY - 1);
            s_io.avid    <= gv_d(LATENCY - 1);
            s_io.field_n <= fl_d(LATENCY - 1);
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture glitchscape;
