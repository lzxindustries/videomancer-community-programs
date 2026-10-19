-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- livewire.vhd
--
-- LIVEWIRE -- the picture's edges as a living material.
--
-- Not an edge detector with a colour scheme on top: one edge engine feeding a
-- bank of materials (neon gas, corona discharge, thin-film interference,
-- painted cel, wet ink), any of which can eat the frame.
--
--   P12 "Emerge" (headline): 0 clean video -> strongest silhouettes over the
--        picture -> interior fades to the material's own ground -> full weight
--        -> overdrive, until the image is consumed by its own outlines.
--
-- Controls:
--   K1 Mode     5 materials, clean cuts (hysteresis on the zones)
--   K2 Scale    continuous fine -> coarse (blends the luma PYRAMID, one Sobel)
--   K3 Thresh   edge sensitivity
--   K4 Material line weight + how far the material reaches past the edge
--   K5 Character per mode: Neon halo hue vs the tube / Kirlian + Ink filament
--               scale / Spectral swirl speed / Cel flat count; S9's lit side
--   K6 Colour   per mode palette (tube hue, ink tint, hue-wheel offset, shade)
--   S7  Invert     dark-on-light <-> light-on-dark (a true negative)
--   S8  Interior   video (tinted) <-> the material's native fill
--   S9  Polarity   one-sided edges (lit from one side) <-> both polarities
--   S10 Rainbow    orientation-hue over any mode; in Spectral it flips back
--                  to the flat K6 palette
--   S11 Persist    the field becomes a memory: trails instead of a halo
--
-- ARCHITECTURE -- two planes, both shared by every mode.
--
-- SHARP (full rate, in frame).  K2 blends the luma pyramid BEFORE the window,
-- so there is exactly one Sobel and orientation keeps its meaning across the
-- whole scale sweep.  Magnitude is block-float normalised by a per-frame shift
-- into "multiples of threshold", which makes every material band (glow 0.55x,
-- body 1x, core 3x) a compare against a per-frame constant -- the entire edge
-- path has NO per-pixel multiply.  Orientation is 16 bins over the full 360
-- degrees (3 shift-approximated tangent compares + the two gradient signs), so
-- polarity is inherent rather than bolted on.
--
-- SOFT (cell rate, across frames).  A 128x64 cell field (16 x ~17 px on HD)
-- holds edge energy, and each frame it is re-injected, diffused one cell in
-- every direction and decayed:
--     F = max( inject, spread * max(4 neighbours), persist * F )
-- Bilinear-upsampled, that one field is the neon halo, the Kirlian corona, the
-- ink bleed, the cel shade and the persistence trails.  A halo wider than
-- the ~3 px a line-buffer window can see needs lines the raster has not
-- delivered yet, so on this part large spread MUST be built across frames --
-- and that is also what makes trails free.  PERSIST does not add to the field
-- (a big halo simply swallows a wake); it changes what the field IS, cutting
-- neighbour coupling and raising self-retention.
--
-- The coarse pyramid level is horizontally symmetric but vertically CAUSAL (a
-- per-column one-pole pair down the frame): a symmetric vertical blur needs
-- lookahead.  It is therefore used only for offset-tolerant work -- tone,
-- levels, field injection, scale blending -- while every positional decision
-- comes off the registered medium scale.
--
-- BRAM: 2 luma line buffers (8) + field (8) + field aux (1) + coarse luma (1)
-- + dry-video ring (2) + sine ROM (1) + hue-wheel ROM (2) ~ 23 EBR.  Latency
-- C_LATENCY = 19, identical in every mode.
--
-- v1.1.4 (2026-10-18): resolution scaling.  The field's cells were 16 px wide
-- at 720p AND 1080p (a measured-width switch), so 720p halos and trails were
-- 1.5x wider relative to the picture.  The cell clock now counts 16 VIRTUAL
-- units (timing-ID x step), giving 10.67-px cells at 720p, and the horizontal
-- interpolation steps the same units (this also fixes SD, whose 8-px cells
-- only ramped half way and then jumped at every cell edge).  720p's 12-line
-- cell rows overflowed the vertical lerp weight (bands on the last three
-- lines of every row); fixed.  Kirlian/Ink filaments are projected in
-- virtual units, so their pitch follows the screen height and the diagonals
-- stay at 45 degrees at SD.  1080p is bit-identical.  Edge kernels, the
-- coarse pyramid box, grain and the dither stay at pixel scale.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture livewire of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LATENCY : integer := 19;

    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
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

    function f_absu(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(-resize(v, v'length + 1));
        else
            return unsigned(resize(v, v'length + 1));
        end if;
    end function;

    function f_c8(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        end if;
        return resize(unsigned(v(7 downto 0)), 8);
    end function;

    -- saturating 8-bit
    function f_s8(v : unsigned) return unsigned is
    begin
        if v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    -- The alpha ramp: clamp a signed difference to 0..255 after a left
    -- shift.  Written as resize-to-16 / shift / compare-to-255 it put a wide
    -- shift and a wide compare in every band, three of them in one stage,
    -- and that was the design's critical path.  The shift amount is a
    -- compile-time constant per call site, so the saturation point is one
    -- too: compare the INPUT against it and the wide arithmetic vanishes.
    function f_ramp(d : signed; sh : integer) return unsigned is
    begin
        if d <= 0 then
            return to_unsigned(0, 8);
        elsif d >= 256 / (2 ** sh) then
            return to_unsigned(255, 8);
        end if;
        return shift_left(resize(unsigned(d(7 downto 0)), 8), sh);
    end function;

    -- Resolution scaling (v1.1.4, mondrian pilot): virtual 1080-line units.
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

    -- quarter-wave folded sine, 64-entry logic ROM (intaglio lineage)
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

    -- quarter-wave fold, address side only; the table itself is a BRAM ROM
    function f_qsin_addr(a : unsigned(9 downto 0)) return unsigned is
    begin
        if a(8) = '0' then
            return a(7 downto 2);
        else
            return not a(7 downto 2);
        end if;
    end function;

    -- K1's five evenly divided label zones (the platform's own split)
    function f_zone5(k : unsigned(10 downto 0)) return unsigned is
    begin
        if k < 205 then
            return "000";
        elsif k < 410 then
            return "001";
        elsif k < 614 then
            return "010";
        elsif k < 819 then
            return "011";
        else
            return "100";
        end if;
    end function;

    type t_qsrom is array (0 to 255) of std_logic_vector(9 downto 0);
    function f_qsrom return t_qsrom is
        variable r : t_qsrom;
    begin
        for i in 0 to 255 loop
            r(i) := std_logic_vector(C_QSIN(i mod 64));
        end loop;
        return r;
    end function;
    signal qs_rom  : t_qsrom := f_qsrom;
    attribute ram_style : string;
    attribute ram_style of qs_rom : signal is "block";
    signal qs_addr : unsigned(7 downto 0) := (others => '0');
    signal qs_data : std_logic_vector(9 downto 0) := (others => '0');
    signal qs_neg1, qs_neg2 : std_logic := '0';


    ----------------------------------------------------------------------
    -- controls (decoded free-running -- deadchannel lesson: latching every
    -- parameter in one frame-event branch made a whole program deaf)
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_p12 : unsigned(9 downto 0)
        := to_unsigned(512, 10);
    signal s_inv, s_int_vid, s_1pol, s_spec, s_persist : std_logic := '0';

    -- glide (ease by diff>>4; always on -- pot noise is visible without it)
    signal g_k2, g_k3, g_k4, g_k5, g_k6, g_p12 : unsigned(13 downto 0)
        := (others => '0');
    signal s_mode  : unsigned(2 downto 0) := (others => '0');
    -- stepped decodes with hysteresis: a decode only moves once the knob is
    -- clear of the boundary on BOTH sides (k-8 and k+8 agree), so pot noise
    -- parked on a boundary cannot flicker the whole frame between steps
    signal s_k1p, s_k1m : unsigned(10 downto 0) := (others => '0');
    signal s_k1zp, s_k1zm : unsigned(2 downto 0) := (others => '0');
    signal s_k5zp, s_k5zm : unsigned(3 downto 0) := (others => '0');
    signal s_k5h   : unsigned(3 downto 0) := "1000";
    -- K5 + K6 as one angle (NEON's halo hue sits K5 away from the tube's)
    signal s_k65   : unsigned(9 downto 0) := (others => '0');
    signal s_qhu, s_qhv : signed(9 downto 0) := (others => '0');
    -- SPECTRAL swirl: the hue wheel turns at K5's rate, 8.8 of a turn
    signal s_hdrift : unsigned(15 downto 0) := (others => '0');
    signal s_hoff   : unsigned(7 downto 0) := (others => '0');
    -- EMERGE dissolve level per frame (16 = fully in)
    signal s_wetn  : unsigned(4 downto 0) := (others => '0');
    -- per-frame LFSR: Kirlian crackle and Neon buzz
    signal s_lfsr  : unsigned(15 downto 0) := x"ACE1";
    signal s_crk   : unsigned(3 downto 0) := (others => '0');
    signal s_jx, s_jy : unsigned(1 downto 0) := (others => '0');
    signal s_xj    : unsigned(11 downto 0) := (others => '0');
    signal s_alj   : unsigned(10 downto 0) := (others => '0');
    signal s_buzzc : unsigned(3 downto 0) := (others => '0');
    signal s_buzz  : std_logic := '0';

    ----------------------------------------------------------------------
    -- per-frame constants
    ----------------------------------------------------------------------
    signal s_msh    : integer range 0 to 6 := 2;      -- block-float shift
    signal s_tbody  : unsigned(7 downto 0) := to_unsigned(24, 8);
    signal s_tcore  : unsigned(7 downto 0) := to_unsigned(72, 8);
    signal s_tglow  : unsigned(7 downto 0) := to_unsigned(13, 8);
    signal s_tinj   : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_w2     : unsigned(4 downto 0) := (others => '0');  -- scale lerp b
    signal s_wet    : unsigned(7 downto 0) := (others => '0');
    signal s_over   : unsigned(7 downto 0) := (others => '0');
    signal s_spread : unsigned(3 downto 0) := to_unsigned(11, 4);
    signal s_perst  : unsigned(3 downto 0) := (others => '0');
    -- CEL posterizer: bits kept (1..4, 5 = untouched) and the chroma grid
    signal s_pkeep   : unsigned(2 downto 0) := "101";
    signal s_pmask_c : unsigned(9 downto 0) := (others => '1');
    signal s_phalf_c : unsigned(9 downto 0) := (others => '0');
    signal s_qc, s_qsn   : signed(9 downto 0) := (others => '0');
    signal s_bu, s_bv     : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- spread/persist coefficients.  These were a 16-entry rebuilt-per-frame
    -- register file until measurement: as a runtime-indexed array it cost
    -- ~300 LUT and a mux tree on the field's critical path, where the 4x4
    -- multiply it replaced costs ~16.  Not every table beats its multiply.

    -- ONE shared per-frame multiplier.  Every per-frame product is muxed
    -- through it two states apart; writing the products inline inferred a
    -- separate multiplier per expression (fringe/pinboard lesson).
    signal mula : unsigned(10 downto 0) := (others => '0');
    signal mulb : unsigned(7 downto 0) := (others => '0');
    signal mulp : unsigned(18 downto 0) := (others => '0');
    signal s_thrr : unsigned(10 downto 0) := to_unsigned(80, 11);
    signal s_thi  : unsigned(10 downto 0) := to_unsigned(160, 11);
    signal s_vsc  : unsigned(10 downto 0) := (others => '0');
    signal s_tlo  : unsigned(7 downto 0) := to_unsigned(20, 8);
    signal s_tfloor : unsigned(7 downto 0) := to_unsigned(15, 8);

    signal s_seq   : unsigned(6 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0) := (others => '0');
    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_vs_armed : std_logic := '0';     -- serrated-vsync guard

    ----------------------------------------------------------------------
    -- geometry (active-region counters: titler pattern)
    ----------------------------------------------------------------------
    signal s_x     : unsigned(11 downto 0) := (others => '0');
    signal s_aline : unsigned(10 downto 0) := (others => '0');
    signal s_seen  : std_logic := '0';
    signal s_meas_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_meas_h : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_cwsh  : integer range 3 to 4 := 4;    -- cell width shift
    signal s_ch    : unsigned(5 downto 0) := to_unsigned(17, 6);  -- cell height
    signal s_inrow : unsigned(5 downto 0) := (others => '0');
    signal s_crow  : unsigned(5 downto 0) := (others => '0');
    signal s_lastl : std_logic := '0';             -- last line of a cell row
    -- vertical edge guards.  The field is a FRAME-RECURSIVE memory, so any
    -- wrap at the cell-row edges hands the bottom of the picture to the top:
    -- s_crow+1 wraps 63 -> 0 (the below-row read), and the aux handoff row 63
    -- writes on its last line is still sitting there when row 0 reads "above"
    -- at the top of the next frame.  Registered off s_crow, which only moves
    -- at hsync, so the pixel path sees plain signals.
    signal s_crowb : unsigned(5 downto 0) := to_unsigned(1, 6);
    signal s_row0  : std_logic := '1';             -- no row above this one
    signal s_rowl  : std_logic := '0';             -- no row below this one
    signal s_rowf  : std_logic := '1';             -- first line of a cell row
    -- last active line index, latched at vsync.  s_meas_h itself tracks the
    -- CURRENT line mid-frame, so "is this the frame's last line" needs a copy
    -- that holds still: 64 cell rows never divide the height exactly, so the
    -- bottom row is always partial and without this it never gets its
    -- last-line write -- its inject accumulator sat in aux over vblank and
    -- row 0 of the next frame read it as its own.
    signal s_hlast : unsigned(10 downto 0) := to_unsigned(1079, 11);

    ----------------------------------------------------------------------
    -- pipeline
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to C_LATENCY) := (others => '0');

    signal r0_y, r0_u, r0_v : unsigned(9 downto 0) := (others => '0');

    -- S1..S3 pyramid blend
    signal a1_y8, h1, h2, wb8 : unsigned(7 downto 0) := (others => '0');

    signal d2c   : signed(9 downto 0) := (others => '0');
    signal mix2p : unsigned(7 downto 0) := (others => '0');
    signal mix2, mix3 : unsigned(7 downto 0) := (others => '0');
    signal d2cp  : signed(14 downto 0) := (others => '0');
    signal s_w2p : unsigned(4 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_w2p : signal is true;
    signal co3 : unsigned(7 downto 0) := (others => '0');

    -- line buffers
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb1, lb2 : t_lb8 := (others => (others => '0'));
    signal q1, q2   : std_logic_vector(7 downto 0) := (others => '0');
    signal lrx, lwx : unsigned(10 downto 0) := (others => '0');
    signal qr1, qr2, y8d : unsigned(7 downto 0) := (others => '0');

    -- S6..S8 gradient
    signal ss, ss1, ss2 : unsigned(9 downto 0) := (others => '0');
    signal dy, dy1, dy2 : signed(8 downto 0) := (others => '0');
    signal s_gx, s_gy   : signed(11 downto 0) := (others => '0');
    signal ax8, ay8     : unsigned(11 downto 0) := (others => '0');
    signal mag8         : unsigned(12 downto 0) := (others => '0');
    signal sgx8, sgy8   : std_logic := '0';
    signal gx8, gy8     : signed(11 downto 0) := (others => '0');

    -- S9 normalise + orientation
    signal magn9, magn9r : unsigned(7 downto 0) := (others => '0');
    signal bin9  : unsigned(3 downto 0) := (others => '0');
    signal gx9, gy9 : signed(11 downto 0) := (others => '0');

    -- S10 material bands
    signal a_body, a_core, a_glow : unsigned(7 downto 0) := (others => '0');
    signal fil10, fil11, fil12 : unsigned(7 downto 0) := (others => '0');
    signal fil_r10, fil_r11 : unsigned(3 downto 0) := (others => '0');
    signal grain : unsigned(5 downto 0) := (others => '0');
    signal s_h1, s_h2, s_h3 : unsigned(15 downto 0) := (others => '0');
    signal inj10 : unsigned(3 downto 0) := (others => '0');
    signal bin10 : unsigned(3 downto 0) := (others => '0');
    signal pol10 : unsigned(7 downto 0) := (others => '0');

    -- S11..S12 mode primitives
    signal a_body11, a_core11, a_glow11 : unsigned(7 downto 0) := (others => '0');
    signal a_body12, a_core12, a_glow12 : unsigned(7 downto 0) := (others => '0');
    signal bin11, bin12 : unsigned(3 downto 0) := (others => '0');
    signal hal11, hal12 : unsigned(7 downto 0) := (others => '0');

    -- compose

    signal m14_u, m14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a14_pres : unsigned(7 downto 0) := (others => '0');
    signal a14_glow, a14_body, a14_core : unsigned(7 downto 0) := (others => '0');
    signal c15_y, c15_u, c15_v : unsigned(9 downto 0) := (others => '0');
    signal c16_y, c18_y : unsigned(9 downto 0) := (others => '0');
    signal hue14_u, hue14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- orientation hue wheel at full chroma amplitude.  SPECTRAL colours
    -- every edge by its angle, so a circle becomes a rainbow ring and
    -- parallel edges share one hue; S10 lays the same wheel over any mode.
    -- 256 steps around the circle: the address is {orientation bin, 0000}
    -- plus an 8-bit per-frame offset (K6 + the Spectral swirl), so the
    -- wheel turns smoothly instead of in 22.5-degree jumps.  Held in BRAM,
    -- not LUTs: as a runtime-indexed constant array it was a 16:1 mux of
    -- 20 bits sitting directly in the chroma path.
    type t_huerom is array (0 to 255) of std_logic_vector(19 downto 0);
    function f_qsin256(i : integer) return integer is
        variable j : integer := i mod 64;
    begin
        case (i mod 256) / 64 is
            when 0      => return  to_integer(C_QSIN(j));
            when 1      => return  to_integer(C_QSIN(63 - j));
            when 2      => return -to_integer(C_QSIN(j));
            when others => return -to_integer(C_QSIN(63 - j));
        end case;
    end function;
    function f_huerom return t_huerom is
        variable r : t_huerom;
    begin
        for i in 0 to 255 loop
            r(i) := std_logic_vector(to_unsigned(
                        512 + (440 * f_qsin256(i + 64)) / 511, 10))
                    & std_logic_vector(to_unsigned(
                        512 + (440 * f_qsin256(i)) / 511, 10));
        end loop;
        return r;
    end function;
    signal hue_rom  : t_huerom := f_huerom;
    attribute ram_style of hue_rom : signal is "block";
    signal hue_addr : unsigned(7 downto 0) := (others => '0');
    signal hue_q    : std_logic_vector(19 downto 0) := (others => '0');
    signal hal14g : unsigned(7 downto 0) := (others => '0');
    signal g14_grain : unsigned(5 downto 0) := (others => '0');

    -- per-frame mode constants
    signal s_gnd_y : unsigned(9 downto 0) := to_unsigned(70, 10);
    signal s_gnd_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gnd_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_dark : std_logic := '0';   -- ink material vs light material
    signal s_qu, s_qv : unsigned(9 downto 0) := to_unsigned(512, 10);  -- halo hue
    signal s_fil   : std_logic := '0';          -- filament / fibre modulation
    signal s_filsh : integer range 2 to 5 := 3; -- filament pitch
    signal s_ragged : std_logic := '0';         -- ink: fibre-perturbed edge
    signal s_sbias : unsigned(9 downto 0) := (others => '0');  -- native scale
    signal s_injm  : unsigned(2 downto 0) := "010";            -- inject mult
    signal s_cu, s_cv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_is_cel, s_is_spectral : std_logic := '0';
    signal a15_body : unsigned(7 downto 0) := (others => '0');
    signal s_halog : unsigned(7 downto 0) := to_unsigned(160, 8);
    -- interior weight, already folded with S8 (video-through-the-material).
    -- The ground is a per-frame constant, so the lerp splits into one
    -- per-pixel product plus a constant term computed once a frame -- half
    -- the depth of a general lerp and no clamp needed (it stays convex).
    signal s_intw : unsigned(5 downto 0) := (others => '0');
    -- the buzz overrides land in their OWN registers: a second
    -- writer state on s_tcore/s_halog (and s_intw) merged into the existing
    -- per-frame cones and became the routed critical path (HD HDMI 73.5)
    signal s_tcorex : unsigned(7 downto 0) := to_unsigned(72, 8);
    signal s_halogx : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_gndw_y, s_gndw_u, s_gndw_v : unsigned(9 downto 0) := (others => '0');
    signal pa_y, pa_u, pa_v : unsigned(13 downto 0) := (others => '0');
    signal pb_y, pb_u, pb_v : unsigned(12 downto 0) := (others => '0');
    signal s_intwc : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal m15_u, m15_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a15_pres : unsigned(7 downto 0) := (others => '0');
    signal g15_y, g15_u, g15_v : unsigned(9 downto 0) := (others => '0');
    signal v15_mat : unsigned(8 downto 0) := (others => '0');
    signal d16_u, d16_v : signed(11 downto 0) := (others => '0');
    signal q17_u, q17_v : signed(17 downto 0) := (others => '0');
    signal g17_u, g17_v : unsigned(9 downto 0) := (others => '0');
    signal a16_pres : unsigned(7 downto 0) := (others => '0');
    signal g16_u, g16_v : unsigned(9 downto 0) := (others => '0');
    signal c16b_y : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- coarse level: per-column one-pole pair (fast/slow) down the frame.
    -- Horizontally symmetric (a 2^cwsh box), vertically causal.
    ----------------------------------------------------------------------
    type t_cl is array (0 to 127) of std_logic_vector(15 downto 0);
    signal clum   : t_cl := (others => (others => '0'));
    signal cl_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal cl_rd, cl_wr : unsigned(6 downto 0) := (others => '0');
    signal cl_wdat : std_logic_vector(15 downto 0) := (others => '0');
    signal cl_we   : std_logic := '0';
    signal cacc    : unsigned(15 downto 0) := (others => '0');
    signal cmean   : unsigned(7 downto 0) := (others => '0');
    signal cfast, cslow : unsigned(7 downto 0) := (others => '0');
    signal cfast_l, cfast_r : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- field: 128 x 64 cells, 4-bit energy
    ----------------------------------------------------------------------
    type t_fld is array (0 to 8191) of std_logic_vector(3 downto 0);
    signal fld    : t_fld := (others => (others => '0'));
    signal f_q    : std_logic_vector(3 downto 0) := (others => '0');
    signal f_addr : unsigned(12 downto 0) := (others => '0');
    signal f_wadr : unsigned(12 downto 0) := (others => '0');
    signal f_wdat : std_logic_vector(3 downto 0) := (others => '0');
    signal f_we   : std_logic := '0';

    -- aux: {above-row old value, this row's running inject} per column
    type t_aux is array (0 to 127) of std_logic_vector(7 downto 0);
    signal aux    : t_aux := (others => (others => '0'));
    signal x_q    : std_logic_vector(7 downto 0) := (others => '0');
    signal x_radr, x_wadr : unsigned(6 downto 0) := (others => '0');
    signal x_wdat : std_logic_vector(7 downto 0) := (others => '0');
    signal x_we   : std_logic := '0';

    -- field pipeline state
    signal fcol   : unsigned(6 downto 0) := (others => '0');   -- current cell col
    signal fsub   : unsigned(4 downto 0) := (others => '0');   -- clock within cell
    signal f_a, f_b : unsigned(3 downto 0) := (others => '0'); -- old(r,c+1), old(r+1,c+1)
    signal f_al, f_bl : unsigned(3 downto 0) := (others => '0');-- ... at column c
    signal f_prev : unsigned(3 downto 0) := (others => '0');   -- old(r,c-1)
    signal f_above : unsigned(3 downto 0) := (others => '0');
    signal f_inj  : unsigned(3 downto 0) := (others => '0');
    signal f_injr : unsigned(3 downto 0) := (others => '0');
    signal f_top, f_bot : unsigned(7 downto 0) := (others => '0');
    signal acc_t, acc_b : signed(13 downto 0) := (others => '0');
    signal stp_t, stp_b : signed(5 downto 0) := (others => '0');
    signal f_wy3 : unsigned(3 downto 0) := (others => '0');
    signal f_disp : unsigned(7 downto 0) := (others => '0');
    signal f_nbr, f_injm, f_spv, f_pev, f_newv : unsigned(3 downto 0)
        := (others => '0');

    -- field engine registers (one cell of prefetch, so every pixel of a cell
    -- sees settled values -- reading mid-cell would stripe each cell boundary)
    signal fa_p2, fa_p1, fa_c, fa_n, fa_pf : unsigned(3 downto 0) := (others => '0');
    signal fb_p1, fb_c, fb_n, fb_pf : unsigned(3 downto 0) := (others => '0');
    -- v1.1.4: the field's cell clock.  A cell is 16 VIRTUAL units wide:
    -- s_fva is the Q.1 position inside the cell (step 2/3/4 per pixel from
    -- the timing ID), s_fsub the clock count since the cell started (the
    -- engine's state), s_fcol the cell column.  At 1080p (16 px) and SD
    -- (8 px) they equal the old s_xf(3:0)/(2:0) and s_xf >> 4/3; 720p gets
    -- 10.67-px cells (11/11/10 clocks), so the halo is no longer 1.5x wider.
    signal s_fva  : unsigned(5 downto 0) := (others => '0');
    signal s_fsub : unsigned(3 downto 0) := (others => '0');
    signal s_fcol : unsigned(6 downto 0) := (others => '0');
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    -- virtual x (Q.1) and active-line y (Q.8) for the filament projection
    signal s_vxa  : unsigned(12 downto 0) := (others => '0');
    signal s_vya  : unsigned(19 downto 0) := (others => '0');
    -- 720p corner steps (1.5 x the difference), precomputed per cell
    signal s_dt4, s_db4, s_dth, s_dbh : signed(5 downto 0) := (others => '0');
    signal s_d15t, s_d15b : signed(5 downto 0) := (others => '0');
    signal fsub_r : unsigned(4 downto 0) := (others => '0');
    signal f_injd : unsigned(3 downto 0) := (others => '0');
    signal f_inja : unsigned(3 downto 0) := (others => '0');
    signal f_wx : unsigned(4 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dry video ring (edgelab technique: an EBR ring instead of ~500 FFs)
    ----------------------------------------------------------------------
    type t_ring is array (0 to 31) of std_logic_vector(31 downto 0);
    signal vring : t_ring := (others => (others => '0'));
    signal ring_w : unsigned(4 downto 0) := (others => '0');
    signal ring_q : std_logic_vector(31 downto 0) := (others => '0');
    signal d_y, d_u, d_v : unsigned(9 downto 0) := (others => '0');
    signal r12_y, r12_u, r12_v : unsigned(9 downto 0) := (others => '0');
    -- 4x4 ordered dither for the EMERGE dissolve
    type t_bayer is array (0 to 15) of unsigned(3 downto 0);
    constant C_BAYER : t_bayer := (
        to_unsigned( 0, 4), to_unsigned( 8, 4), to_unsigned( 2, 4), to_unsigned(10, 4),
        to_unsigned(12, 4), to_unsigned( 4, 4), to_unsigned(14, 4), to_unsigned( 6, 4),
        to_unsigned( 3, 4), to_unsigned(11, 4), to_unsigned( 1, 4), to_unsigned( 9, 4),
        to_unsigned(15, 4), to_unsigned( 7, 4), to_unsigned(13, 4), to_unsigned( 5, 4));
    signal s_dith : unsigned(3 downto 0) := (others => '0');
    signal s_matoff : std_logic := '1';


    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin port for the palette taps
    ------------------------------------------------------------------------
    -- one state earlier than the capture states now that the table read
    -- costs a clock: fold at N, read at N-1, sign-flip at N-2, capture at N-3
    s_qs_a <= resize(g_k6(13 downto 4), 10) + to_unsigned(256, 10) when s_seq = 37 else
              s_k65 + to_unsigned(768, 10)                           when s_seq = 35 else
              s_k65                                                  when s_seq = 33 else
              resize(g_k6(13 downto 4), 10);

    ------------------------------------------------------------------------
    -- control decode + per-frame sequencer.  Controls are read continuously
    -- and eased one op per cycle; only genuinely per-frame state hangs off
    -- the vsync anchor, which is guarded against analog serration.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        -- The glide accumulator MUST be sliced, not resized.  resize() on a
        -- signed value keeps the sign bit plus the rightmost N-1 bits, so any
        -- knob past half range folded and the glide oscillated (5765 -> 844
        -- -> 6104) instead of converging.  Exactly the trap in the
        -- vhdl_slew_resize_trap note; the picture just looked "wrong-ish".
        variable v_d  : signed(15 downto 0);
        variable v_g  : signed(15 downto 0);
        variable v_e  : unsigned(9 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_thr : unsigned(13 downto 0);
        variable v_sc : unsigned(10 downto 0);
        variable v_i  : integer range -8 to 12;
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            qs_addr <= resize(f_qsin_addr(s_qs_a), 8);
            qs_neg1 <= s_qs_a(9);
            qs_neg2 <= qs_neg1;
            if qs_neg2 = '1' then
                s_qs_r <= -signed(qs_data);
            else
                s_qs_r <= signed(qs_data);
            end if;

            -- free-running raw control read
            s_k1 <= unsigned(registers_in(0));
            s_k2 <= unsigned(registers_in(1));
            s_k3 <= unsigned(registers_in(2));
            s_k4 <= unsigned(registers_in(3));
            s_k5 <= unsigned(registers_in(4));
            s_k6 <= unsigned(registers_in(5));
            s_p12 <= unsigned(registers_in(7));
            s_inv     <= registers_in(6)(0);
            s_int_vid <= registers_in(6)(1);
            s_1pol    <= registers_in(6)(2);
            s_spec    <= registers_in(6)(3);
            s_persist <= registers_in(6)(4);

            -- glide: ease by diff>>4, no snap (always on)
            v_d := signed(resize(s_k2 & "0000", 16)) - signed(resize(g_k2, 16));
            v_g := signed(resize(g_k2, 16)) + shift_right(v_d, 4);
            g_k2 <= unsigned(v_g(13 downto 0));
            v_d := signed(resize(s_k3 & "0000", 16)) - signed(resize(g_k3, 16));
            v_g := signed(resize(g_k3, 16)) + shift_right(v_d, 4);
            g_k3 <= unsigned(v_g(13 downto 0));
            v_d := signed(resize(s_k4 & "0000", 16)) - signed(resize(g_k4, 16));
            v_g := signed(resize(g_k4, 16)) + shift_right(v_d, 4);
            g_k4 <= unsigned(v_g(13 downto 0));
            -- K5/K6 are set-and-forget: pot noise on a hue or a light angle
            -- is invisible, and six eased accumulators were ~300 LUT
            g_k5 <= s_k5 & "0000";
            g_k6 <= s_k6 & "0000";
            v_d := signed(resize(s_p12 & "0000", 16)) - signed(resize(g_p12, 16));
            v_g := signed(resize(g_p12, 16)) + shift_right(v_d, 4);
            g_p12 <= unsigned(v_g(13 downto 0));

            -- MODE: five materials across K1 (hard zones).  Hysteresis: the
            -- zone of k+8 and of k-8 must agree before the mode moves, so a
            -- knob parked on a boundary holds one mode instead of flickering.
            s_k1p <= resize(s_k1, 11) + 8;
            if s_k1 < 8 then
                s_k1m <= (others => '0');
            else
                s_k1m <= resize(s_k1 - 8, 11);
            end if;
            s_k1zp <= f_zone5(s_k1p);
            s_k1zm <= f_zone5(s_k1m);
            -- Labelled default: at program load the firmware writes the
            -- initial_value_label's INDEX (0..4) into the register, not a
            -- 0..1023 position, so a value below 5 is the mode itself.
            if s_k1 < 5 then
                s_mode <= s_k1(2 downto 0);
            elsif s_k1zp = s_k1zm then
                s_mode <= s_k1zp;     -- 0 NEON 1 KIRLIAN 2 SPECTRAL 3 CEL 4 INK
            end if;

            -- K5's 16 steps (S9's lit side; its top two bits are the Cel
            -- level count and the filament scale), same hysteresis.  A held
            -- 4-bit index is a held 2-bit index too: every 2-bit boundary
            -- is a 4-bit one.
            if s_k5 >= 1016 then
                s_k5zp <= "1111";
            else
                s_k5zp <= resize(shift_right(s_k5 + 8, 6), 4);
            end if;
            if s_k5 < 8 then
                s_k5zm <= "0000";
            else
                s_k5zm <= resize(shift_right(s_k5 - 8, 6), 4);
            end if;
            if s_k5zp = s_k5zm then
                s_k5h <= s_k5zp;
            end if;

            -- NEON halo angle: K6 + K5 (wraps; continuous, so no hysteresis)
            s_k65 <= g_k6(13 downto 4) + g_k5(13 downto 4);

            -- vsync anchor, guarded (analog vsync serrates: act once per field)
            if data_in.avid = '1' then
                s_vs_armed <= '1';
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_vs_armed = '1' then
                s_vs_armed <= '0';
                s_seq <= to_unsigned(60, 7);
                -- cell geometry from the measured raster
                if s_meas_w > 1024 then
                    s_cwsh <= 4;
                else
                    s_cwsh <= 3;
                end if;
                s_ch <= resize(shift_right(s_meas_h, 6), 6) + 1;
                s_hlast <= s_meas_h;
            end if;

            ------------------------------------------------------------
            -- vblank sequencer: one small op per state, every per-frame
            -- product on the shared multiplier (set operands, read 2 later)
            ------------------------------------------------------------
            mulp <= mula * mulb;

            if s_seq /= 0 then
                v_e := resize(g_p12(13 downto 4), 10);      -- EMERGE 0..1023
                case to_integer(s_seq) is

                    when 60 =>
                        -- wet mix: exactly dry video at EMERGE 0
                        if v_e >= 51 then
                            s_wet <= to_unsigned(255, 8);
                        else
                            s_wet <= resize(v_e(7 downto 0) & '0', 8);
                        end if;

                    when 59 =>
                        -- per-frame LFSR (Galois, x^16+x^14+x^13+x^11+1)
                        if s_lfsr(0) = '1' then
                            s_lfsr <= shift_right(s_lfsr, 1) xor x"B400";
                        else
                            s_lfsr <= shift_right(s_lfsr, 1);
                        end if;

                    when 58 =>
                        -- interior: video -> the material's own ground.  S8
                        -- caps it so the video stays visible THROUGH the
                        -- material instead of being replaced.
                        if v_e <= 225 then
                            s_intw <= (others => '0');
                        elsif v_e >= 563 then
                            s_intw <= to_unsigned(63, 6);
                        else
                            s_intw <= resize(shift_right(v_e - 225, 3), 6)
                                      + resize(shift_right(v_e - 225, 5), 6);
                        end if;

                    when 57 =>
                        -- the EMERGE dissolve level (16 = fully in)
                        if s_wet = 255 then
                            s_wetn <= to_unsigned(16, 5);
                        else
                            s_wetn <= resize(s_wet(7 downto 4), 5);
                        end if;

                    when 56 =>
                        if s_int_vid = '1' and s_intw > 26 then
                            s_intw <= to_unsigned(26, 6);
                        end if;
                        -- overdrive: nothing until 75%
                        if v_e <= 768 then
                            s_over <= (others => '0');
                        else
                            s_over <= f_s8(resize(v_e - 768, 9) & '0');
                        end if;

                    when 55 =>
                        -- SCALE: K2, offset by EMERGE (coarse structures
                        -- first, releasing to the knob by 45%)
                        v_sc := resize(s_sbias, 11)
                                + resize(g_k2(13 downto 5), 11) - 256;
                        if v_e < 460 then
                            v_sc := v_sc + resize(shift_right(460 - v_e, 1), 11);
                        end if;
                        if v_sc > 1023 then
                            v_sc := to_unsigned(1023, 11);
                        end if;
                        s_vsc <= v_sc;

                    when 54 =>
                        -- the knob's lower half is the fine level outright;
                        -- only its upper half blends toward coarse
                        if s_vsc < 512 then
                            s_w2 <= (others => '0');
                        else
                            s_w2 <= resize(shift_right(s_vsc - 512, 5), 5);
                        end if;

                    when 53 =>
                        -- the floor only exists to keep the coarse level's
                        -- own quantisation out of the detector, so scale it
                        -- with how much coarse is actually in the blend
                        if s_w2 >= 11 then
                            s_tfloor <= to_unsigned(30, 8);
                        elsif s_w2 >= 5 then
                            s_tfloor <= to_unsigned(15, 8);
                        else
                            s_tfloor <= to_unsigned(6, 8);
                        end if;
                        s_thi <= to_unsigned(40, 11)
                                 + resize(shift_right(1023 - resize(g_k3(13 downto 4), 10), 2), 11);

                    when 52 =>
                        -- K3 sets BOTH ends of the sweep: the threshold at
                        -- EMERGE 0 and the one at full EMERGE.  Scaling a
                        -- single base by the decay let the floor eat the knob.
                        s_tlo <= s_tfloor
                                 + resize(shift_right(1023 - resize(g_k3(13 downto 4), 10), 5), 8);
                        mula <= s_thi - resize(s_tlo, 11);
                        if v_e < 256 then
                            mulb <= to_unsigned(255, 8)
                                    - resize(shift_right(v_e * 5, 3)(7 downto 0), 8);
                        elsif v_e < 717 then
                            mulb <= to_unsigned(95, 8)
                                    - resize(shift_right(v_e - 256, 3)(7 downto 0), 8);
                        else
                            mulb <= to_unsigned(38, 8);
                        end if;

                    when 50 =>
                        null;                              -- multiplier

                    when 49 =>
                        -- CEL flats: EMERGE takes bits away from the video
                        -- in eighths of its travel, K5 sets how many it
                        -- starts with; untouched (5) for the first eighth so
                        -- the outlines emerge before the flats do.  S8 Video
                        -- keeps at least 8 tones so the picture stays legible.
                        v_i := 5 + to_integer(s_k5h(3 downto 2))
                               - to_integer(v_e(9 downto 7));
                        if s_int_vid = '1' and v_i < 3 then
                            v_i := 3;
                        end if;
                        if v_i < 1 then
                            v_i := 1;
                        elsif v_i > 5 or s_mode /= 3 then
                            v_i := 5;
                        end if;
                        s_pkeep <= to_unsigned(v_i, 3);

                    when 48 =>
                        -- WEIGHT thickens every line by lowering the
                        -- threshold (lagoon: move the threshold, not a gain)
                        mula <= resize(s_tlo, 11) + resize(shift_right(mulp, 8), 11);
                        mulb <= to_unsigned(150, 8)
                                + resize(shift_right(1023 - resize(g_k4(13 downto 4), 10), 3), 8);

                    when 47 =>
                        -- CEL: the flats ARE the interior replacement, so the
                        -- paper lift only gets a quarter of the weight
                        if s_mode = 3 then
                            s_intw <= resize(s_intw(5 downto 2), 6);
                        end if;

                    when 46 =>
                        null;

                    when 45 =>
                        -- KIRLIAN crackle: on a random ~half of frames the
                        -- filaments re-deal their reaches and hop a pixel
                        -- or two, so the discharge flickers instead of
                        -- sitting still.  Off in every other mode (Ink's
                        -- fibre must not shimmer).
                        if s_mode = 1 then
                            if s_lfsr(8) = '1' then
                                s_crk <= s_lfsr(12 downto 9);
                                s_jx  <= s_lfsr(14 downto 13);
                                s_jy  <= s_lfsr(15) & s_lfsr(3);
                            end if;
                        else
                            s_crk <= (others => '0');
                            s_jx  <= (others => '0');
                            s_jy  <= (others => '0');
                        end if;

                    when 41 =>
                        -- NEON buzz: now and then (~1 frame in 64) the tube
                        -- stutters -- 1..4 on/off flickers of a frame each
                        s_buzz <= '0';
                        if s_buzzc /= 0 then
                            s_buzzc <= s_buzzc - 1;
                            if s_mode = 0 then
                                s_buzz <= s_buzzc(0);
                            end if;
                        elsif s_lfsr(5 downto 0) = 0 then
                            s_buzzc <= '0' & s_lfsr(7 downto 6) & '1';
                        end if;

                    when 44 =>
                        v_t := resize(shift_right(mulp, 8), 14);
                        if v_t < resize(s_tfloor, 14) then
                            v_t := resize(s_tfloor, 14);
                        elsif v_t > 900 then
                            v_t := to_unsigned(900, 14);
                        end if;
                        s_thrr <= resize(v_t, 11);

                    when 42 =>
                        -- block-float: normalise so the threshold lands in
                        -- 16..31 and every material band becomes a compare
                        if s_thrr >= 512 then
                            s_msh <= 5;
                        elsif s_thrr >= 256 then
                            s_msh <= 4;
                        elsif s_thrr >= 128 then
                            s_msh <= 3;
                        elsif s_thrr >= 64 then
                            s_msh <= 2;
                        elsif s_thrr >= 32 then
                            s_msh <= 1;
                        else
                            s_msh <= 0;
                        end if;

                    when 40 =>
                        case s_msh is
                            when 0 => s_tbody <= f_s8(resize(s_thrr, 11));
                            when 1 => s_tbody <= f_s8(resize(s_thrr(10 downto 1), 10));
                            when 2 => s_tbody <= f_s8(resize(s_thrr(10 downto 2), 9));
                            when 3 => s_tbody <= f_s8(resize(s_thrr(10 downto 3), 8));
                            when 4 => s_tbody <= f_s8(resize(s_thrr(10 downto 4), 7));
                            when others => s_tbody <= f_s8(resize(s_thrr(10 downto 5), 6));
                        end case;

                    when 38 =>
                        -- the bands are fixed multiples of that one threshold
                        s_tcore <= f_s8(resize(s_tbody, 10) + resize(s_tbody & '0', 10)
                                   + resize(shift_right(resize(s_tbody, 10)
                                     * resize(s_over(7 downto 6), 10), 0), 10));
                        s_tglow <= f_s8(resize(shift_right(resize(s_tbody, 10), 1), 10)
                                   + resize(shift_right(resize(s_tbody, 10), 4), 10));
                        s_tinj  <= f_s8(resize(s_tbody, 10)
                                   * resize(s_injm, 10));

                    when 37 =>
                        -- a buzz frame drops the white-hot core out
                        if s_buzz = '1' then
                            s_tcorex <= (others => '1');
                        else
                            s_tcorex <= s_tcore;
                        end if;

                    when 36 =>
                        s_qc <= s_qs_r;

                    when 34 =>
                        s_qsn <= s_qs_r;

                    when 32 =>
                        s_qhu <= s_qs_r;                   -- sin(K6+K5-90)
                        -- CEL chroma grid: one bit finer than the luma, on a
                        -- round-to-nearest grid so neutral stays neutral
                        case to_integer(s_pkeep) is
                            when 1 => s_pmask_c <= "1100000000"; s_phalf_c <= "0010000000";
                            when 2 => s_pmask_c <= "1110000000"; s_phalf_c <= "0001000000";
                            when 3 => s_pmask_c <= "1111000000"; s_phalf_c <= "0000100000";
                            when 4 => s_pmask_c <= "1111100000"; s_phalf_c <= "0000010000";
                            when others => s_pmask_c <= (others => '1'); s_phalf_c <= (others => '0');
                        end case;

                    when 30 =>
                        s_qhv <= s_qs_r;                   -- cos(K6+K5-90)
                        -- SPECTRAL swirl: K5 is the wheel's turning rate,
                        -- still at 0, about a turn a second at full
                        if s_mode = 2 then
                            s_hdrift <= s_hdrift + resize(g_k5(13 downto 4), 16);
                        end if;

                    when 29 =>
                        s_hoff <= g_k6(13 downto 6) + s_hdrift(15 downto 8);

                    when 28 =>
                        -- field character.  MATERIAL sets the reach; PERSIST
                        -- cuts neighbour coupling and hands the field to its
                        -- own history, so the glow tightens and a wake
                        -- appears -- adding persistence to a wide halo just
                        -- hides the wake inside it.
                        if s_persist = '1' then
                            s_spread <= to_unsigned(4, 4)
                                        + resize(g_k4(13 downto 12), 4);
                            s_perst  <= to_unsigned(15, 4);
                        else
                            s_spread <= to_unsigned(9, 4)
                                        + resize(g_k4(13 downto 12), 4);
                            s_perst  <= (others => '0');
                        end if;
                        -- MATERIAL is reach, so 0% must mean none: the fog
                        -- ramps in over K4's bottom eighth in EVERY mode,
                        -- leaving a bare line.  CEL keeps its own fade on K6
                        -- as well; a priority chain is a cheaper "whichever
                        -- is lower" than a comparator.
                        if g_k4(13 downto 11) = 0 then
                            s_halog <= resize(g_k4(10 downto 4), 8);
                        elsif s_mode = 3 and g_k6(13 downto 11) = 0 then
                            s_halog <= resize(g_k6(10 downto 4), 8);
                        else
                            s_halog <= to_unsigned(120, 8)
                                       + resize(shift_right(s_over, 1), 8);
                        end if;

                    when 27 =>
                        -- ...and halves the glow
                        if s_buzz = '1' then
                            s_halogx <= '0' & s_halog(7 downto 1);
                        else
                            s_halogx <= s_halog;
                        end if;

                    when 26 =>
                        -- material palette.  Saturation is per material:
                        -- a gas tube is full-swing chroma, struck metal is a
                        -- quarter of it, ink is a whisper.
                        s_cu <= to_unsigned(512, 10);
                        s_cv <= to_unsigned(512, 10);
                        case to_integer(s_mode) is
                            when 1 =>            -- KIRLIAN: cyan body
                                s_bu <= to_unsigned(660, 10);
                                s_bv <= to_unsigned(300, 10);
                            when 3 =>            -- CEL: the line is black
                                s_bu <= to_unsigned(512, 10);
                                s_bv <= to_unsigned(512, 10);
                            when 4 =>            -- INK: barely-tinted black
                                s_bu <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qc, 3), 11));
                                s_bv <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qsn, 3), 11));
                            when others =>       -- NEON / SPECTRAL: full swing
                                s_bu <= f_cu10(to_signed(512, 11) + resize(s_qc, 11));
                                s_bv <= f_cu10(to_signed(512, 11) + resize(s_qsn, 11));
                        end case;
                        if s_mode = 3 then
                            s_is_cel <= '1';
                        else
                            s_is_cel <= '0';
                        end if;
                        if s_mode = 2 then
                            s_is_spectral <= '1';
                        else
                            s_is_spectral <= '0';
                        end if;
                        -- each material has a native SCALE it wants to be
                        -- read at (K2 offsets it) and its own field
                        -- character: a wide halo needs room, so the modes
                        -- with big halos inject only from strong boundaries
                        case to_integer(s_mode) is
                            when 0 =>            -- NEON: glass tube
                                s_gnd_y <= to_unsigned(70, 10);
                                s_gnd_u <= to_unsigned(512, 10);
                                s_gnd_v <= to_unsigned(512, 10);
                                s_dark  <= '0';
                                -- the halo hue sits K5 round the wheel from
                                -- the tube's (K5 50% = a quarter turn, the
                                -- original fixed separation)
                                s_qu <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qhu, 1), 11));
                                s_qv <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qhv, 1), 11));
                                s_fil <= '0';
                                s_ragged <= '0';
                                s_sbias <= to_unsigned(552, 10);
                                s_injm <= "011";
                            when 1 =>            -- KIRLIAN: corona discharge
                                s_gnd_y <= to_unsigned(66, 10);
                                s_gnd_u <= to_unsigned(512, 10);
                                s_gnd_v <= to_unsigned(512, 10);
                                s_dark  <= '0';
                                -- violet at the reach, cyan in the body,
                                -- white at the source
                                s_qu <= to_unsigned(700, 10);    -- violet reach
                                s_qv <= to_unsigned(640, 10);
                                s_fil <= '1';
                                s_ragged <= '0';
                                s_sbias <= to_unsigned(790, 10);
                                s_injm <= "100";
                            when 3 =>            -- CEL: painted flats
                                s_gnd_y <= to_unsigned(900, 10);
                                s_gnd_u <= to_unsigned(512, 10);
                                s_gnd_v <= to_unsigned(512, 10);
                                s_dark  <= '1';
                                -- K6 tints the shade that hugs the outline
                                s_qu <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qc, 1), 11));
                                s_qv <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qsn, 1), 11));
                                s_fil <= '0';
                                s_ragged <= '0';
                                s_sbias <= to_unsigned(470, 10);
                                s_injm <= "010";
                            when 4 =>            -- INK BLEED: wet on paper
                                s_gnd_y <= to_unsigned(918, 10);
                                s_gnd_u <= to_unsigned(504, 10);
                                s_gnd_v <= to_unsigned(522, 10);
                                s_dark  <= '1';
                                s_qu <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qc, 3), 11));
                                s_qv <= f_cu10(to_signed(512, 11)
                                        + resize(shift_right(s_qsn, 3), 11));
                                s_fil <= '1';
                                s_ragged <= '1';
                                s_sbias <= to_unsigned(590, 10);
                                s_injm <= "010";
                            when others =>       -- SPECTRAL: thin film
                                s_gnd_y <= to_unsigned(70, 10);
                                s_gnd_u <= to_unsigned(512, 10);
                                s_gnd_v <= to_unsigned(512, 10);
                                s_dark  <= '0';
                                s_qu <= f_cu10(to_signed(512, 11) + resize(s_qc, 11));
                                s_qv <= f_cu10(to_signed(512, 11) + resize(s_qsn, 11));
                                s_fil <= '0';
                                s_ragged <= '0';
                                s_sbias <= to_unsigned(330, 10);
                                s_injm <= "011";
                        end case;
                        s_filsh <= 5 - to_integer(s_k5h(3 downto 2));

                    when 24 =>
                        s_intwc <= to_unsigned(64, 7) - resize(s_intw, 7);
                        mula <= resize(s_gnd_y, 11);
                        mulb <= resize(s_intw, 8);

                    when 22 =>
                        s_gndw_y <= resize(shift_right(mulp, 6), 10);
                        mula <= resize(s_gnd_u, 11);
                        mulb <= resize(s_intw, 8);

                    when 20 =>
                        s_gndw_u <= resize(shift_right(mulp, 6), 10);
                        mula <= resize(s_gnd_v, 11);
                        mulb <= resize(s_intw, 8);

                    when 18 =>
                        s_gndw_v <= resize(shift_right(mulp, 6), 10);

                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a, v_b : unsigned(11 downto 0);
        variable v_t1, v_t2, v_t3 : unsigned(13 downto 0);
        variable v_sub : unsigned(1 downto 0);
        variable v_bin : unsigned(3 downto 0);
        variable v_m   : unsigned(12 downto 0);
        variable v_l   : unsigned(15 downto 0);
        variable v_mat : unsigned(8 downto 0);
        variable v_cu, v_cv : unsigned(10 downto 0);
        variable v_rep : unsigned(9 downto 0);
        variable v_mg  : unsigned(7 downto 0);
        variable v_lim : unsigned(9 downto 0);
        variable v_pj  : unsigned(12 downto 0);
        variable v_fv  : unsigned(6 downto 0);

        variable v_d   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to C_LATENCY loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S0: input register.  Chroma is SWAPPED here so everything
            -- inside sees true Cb/Cr; the generated output is swapped back
            -- at compose and the dry path is never swapped.
            if data_in.avid = '1' then
                r0_y <= unsigned(data_in.y);
                r0_u <= unsigned(data_in.v);
                r0_v <= unsigned(data_in.u);
                s_x  <= s_x + 1;
                s_vxa <= s_vxa + s_xs;
            end if;
            -- virtual steps, free-running from the timing ID (static per mode)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            s_w2p <= s_w2;

            -- S1: 8-bit luma, [1 2 1]/4 horizontal blur history
            if av(1) = '1' then
                a1_y8 <= r0_y(9 downto 2);
                h1 <= a1_y8;
                h2 <= h1;
                wb8 <= resize(shift_right(resize(h2, 10)
                       + shift_left(resize(h1, 10), 1)
                       + resize(a1_y8, 10), 2), 8);

            end if;

            -- S2: the coarse delta.  co3 is sampled one pixel earlier than it
            -- used to be -- a 16-column one-pole does not notice.
            if av(2) = '1' then
                mix2 <= wb8;
                d2c  <= signed(resize(co3, 10)) - signed(resize(wb8, 10));
            end if;

            -- S3: the scale product on a stage of its own.  Fused with the
            -- add+clamp this was the routed critical path on every seed
            -- (4.5 ns logic / 9.6 ns routing); s_w2p is a pixel-path copy of
            -- the per-frame weight so the placer can sit it by the multiplier.
            if av(3) = '1' then
                d2cp  <= resize(d2c * signed('0' & s_w2p), 15);
                mix2p <= mix2;
            end if;

            -- S4: pyramid lerp B -- toward the coarse level.  The blended
            -- luma is what goes into the window, so there is one Sobel.
            if av(4) = '1' then
                mix3 <= f_c8(signed(resize(mix2p, 12))
                        + resize(shift_right(d2cp, 4), 12));
            end if;

            -- S4: line-buffer read address advances (writes ride S5)
            if av(4) = '1' then
                lrx <= lrx + 1;
            end if;

            -- S5: land the two line-buffer reads
            if av(5) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                y8d <= mix3;
                lwx <= lwx + 1;
            end if;

            -- S6: Sobel column sums + column histories
            if av(6) = '1' then
                ss  <= resize(qr2, 10) + shift_left(resize(qr1, 10), 1)
                       + resize(y8d, 10);
                dy  <= signed(resize(qr2, 9)) - signed(resize(y8d, 9));
                ss1 <= ss;  ss2 <= ss1;
                dy1 <= dy;  dy2 <= dy1;
            end if;

            -- S7: gradients
            if av(7) = '1' then
                s_gx <= signed(resize(ss, 12)) - signed(resize(ss2, 12));
                s_gy <= resize(dy, 12) + shift_left(resize(dy1, 12), 1)
                        + resize(dy2, 12);
            end if;

            -- S8: |gx|, |gy|, octagonal magnitude, signs
            if av(8) = '1' then
                v_a := resize(f_absu(s_gx), 12);
                v_b := resize(f_absu(s_gy), 12);
                ax8 <= v_a;
                ay8 <= v_b;
                if v_a >= v_b then
                    mag8 <= resize(v_a, 13) + resize(shift_right(v_b, 1), 13);
                else
                    mag8 <= resize(v_b, 13) + resize(shift_right(v_a, 1), 13);
                end if;
                if s_gx < 0 then sgx8 <= '1'; else sgx8 <= '0'; end if;
                if s_gy < 0 then sgy8 <= '1'; else sgy8 <= '0'; end if;
                gx8 <= s_gx;
                gy8 <= s_gy;
            end if;

            -- paper/discharge hash, one op per stage and free-running: it
            -- depends only on the counters, so it can be computed early and
            -- read whenever.  Fused into the alpha stage it was the ceiling.
            -- (A pack-xor-rotate-ADD; the ADD is what breaks GF(2)
            -- linearity, and constant multiplies cost ~200 LUT here.)
            s_h1 <= (s_x(7 downto 0) & s_aline(7 downto 0)) xor x"5BD1";
            s_h2 <= s_h1 xor ("0000" & s_h1(15 downto 4));
            s_h3 <= s_h2 + rotate_left(s_h2, 5);

            -- S9: block-float normalise (a case over fixed slices, never a
            -- barrel) + the 16-bin signed orientation.  Three tangent
            -- compares give the sub-sector; the two signs give the quadrant,
            -- so light->dark and dark->light land in opposite bins.
            if av(9) = '1' then
                case s_msh is
                    when 0 => v_m := mag8;
                    when 1 => v_m := resize(mag8(12 downto 1), 13);
                    when 2 => v_m := resize(mag8(12 downto 2), 13);
                    when 3 => v_m := resize(mag8(12 downto 3), 13);
                    when 4 => v_m := resize(mag8(12 downto 4), 13);
                    when others => v_m := resize(mag8(12 downto 5), 13);
                end case;
                magn9 <= f_s8(resize(v_m, 13));
                -- INK's fibre-perturbed magnitude, formed here so the alpha
                -- stage is a pure set of compares
                if f_s8(resize(v_m, 13)) > resize(s_h3(5 downto 2), 8) then
                    magn9r <= f_s8(resize(v_m, 13)) - resize(s_h3(5 downto 2), 8);
                else
                    magn9r <= (others => '0');
                end if;

                v_t1 := resize(shift_right(ax8, 1), 14)
                        - resize(shift_right(ax8, 4), 14);      -- 0.4375
                v_t2 := resize(ax8, 14);                        -- 1.0
                v_t3 := shift_left(resize(ax8, 14), 1)
                        + resize(shift_right(ax8, 1), 14)
                        - resize(shift_right(ax8, 4), 14);      -- 2.4375
                if resize(ay8, 14) < v_t1 then
                    v_sub := "00";
                elsif resize(ay8, 14) < v_t2 then
                    v_sub := "01";
                elsif resize(ay8, 14) < v_t3 then
                    v_sub := "10";
                else
                    v_sub := "11";
                end if;
                if sgx8 = '0' and sgy8 = '0' then
                    v_bin := "00" & v_sub;
                elsif sgx8 = '1' and sgy8 = '0' then
                    v_bin := ("01" & "11") - ("00" & v_sub);
                elsif sgx8 = '1' and sgy8 = '1' then
                    v_bin := "10" & v_sub;
                else
                    v_bin := ("11" & "11") - ("00" & v_sub);
                end if;
                bin9 <= v_bin;
                gx9 <= gx8;
                gy9 <= gy8;
            end if;

            -- S10: material bands -- every one a compare against a per-frame
            -- constant, then a fixed shift for the anti-aliased shoulder.
            -- INK's edge threshold is perturbed by the paper fibre, which is
            -- what makes its line ragged instead of drafted.
            if av(10) = '1' then
                if s_ragged = '1' then
                    v_mg := magn9r;
                else
                    v_mg := magn9;
                end if;
                a_body <= f_ramp(signed(resize(v_mg, 10))
                          - signed(resize(s_tbody, 10)), 4);
                a_core <= f_ramp(signed(resize(v_mg, 10))
                          - signed(resize(s_tcorex, 10)), 3);
                a_glow <= f_ramp(signed(resize(v_mg, 10))
                          - signed(resize(s_tglow, 10)), 2);
                inj10  <= resize(shift_right(f_ramp(signed(resize(magn9, 10))
                          - signed(resize(s_tinj, 10)), 2), 4), 4);
                -- filament coordinate: the projection perpendicular to the
                -- local gradient, so discharge runs OUTWARD from a boundary
                -- and ink fibre runs along the stroke
                case to_integer(bin9(3 downto 2)) is
                    when 0      => v_pj := resize(s_xj, 13);
                    when 1      => v_pj := resize(s_xj, 13) + resize(s_alj, 13);
                    when 2      => v_pj := resize(s_alj, 13);
                    when others => v_pj := resize(s_xj, 13) - resize(s_alj, 13);
                end case;
                -- per-stripe reach, straight out of the projection's own
                -- bits: constant along a filament, different between them
                fil_r10 <= v_pj(11 downto 8) xor v_pj(7 downto 4) xor s_crk;
                case s_filsh is
                    when 2      => fil10 <= resize(v_pj(4 downto 0) & "000", 8);
                    when 3      => fil10 <= resize(v_pj(5 downto 0) & "00", 8);
                    when 4      => fil10 <= resize(v_pj(6 downto 1) & "00", 8);
                    when others => fil10 <= resize(v_pj(7 downto 2) & "00", 8);
                end case;
                grain <= resize(s_h3(11 downto 9), 6);
                bin10  <= bin9;
                -- EMERGE 0..5%: the material dissolves in on an ordered
                -- dither.  At 0 every alpha is zero, and a zero-alpha,
                -- zero-interior compose passes the video through bit-exact.
                if s_matoff = '1' or s_x < 28 or s_aline < 3 then
                    a_body <= (others => '0');
                    a_core <= (others => '0');
                    a_glow <= (others => '0');
                    inj10  <= (others => '0');
                end if;
                -- S9 POLARITY: keep one half of the wheel (a one-sided,
                -- lit-from-one-direction drawing) with a soft shoulder
                if s_1pol = '1' and (bin9 - s_k5h) >= 8 then
                    pol10 <= (others => '0');
                else
                    pol10 <= (others => '1');
                end if;
            end if;

            -- S11: polarity gate, hue-wheel address, filament gate
            if av(11) = '1' then
                -- S9 POLARITY attenuates the wrong-facing half of the wheel
                -- by a shift; a multiply here bought nothing visible
                if pol10 = 0 then
                    a_body11 <= shift_right(a_body, 3);
                    a_core11 <= shift_right(a_core, 3);
                else
                    a_body11 <= a_body;
                    a_core11 <= a_core;
                end if;
                a_glow11 <= a_glow;
                bin11 <= bin10;
                hue_addr <= (bin10 & "0000") + s_hoff;
                hal11 <= f_disp;
                -- filaments: a stripe centred gate, its length varied per
                -- stripe so the discharge branches instead of glowing
                if fil10 < 26 then
                    fil11 <= to_unsigned(255, 8);
                elsif fil10 < 60 then
                    fil11 <= to_unsigned(120, 8);
                else
                    fil11 <= (others => '0');
                end if;
                fil_r11 <= fil_r10;
            end if;

            -- S12
            if av(12) = '1' then
                a_body12 <= a_body11;
                a_core12 <= a_core11;
                a_glow12 <= a_glow11;
                bin12 <= bin11;
                fil12 <= fil11;
                -- KIRLIAN and INK modulate their reach by the filament; the
                -- other materials take the field flat
                if s_fil = '1' then
                    if hal11 > 190 then
                        hal12 <= shift_right(hal11, 3);   -- interior: no corona
                    elsif hal11(7 downto 4) < fil_r11 then
                        hal12 <= shift_right(hal11, 3);   -- past this filament's reach
                    elsif fil11 = 255 then
                        hal12 <= hal11;
                    elsif fil11 = 120 then
                        hal12 <= shift_right(hal11, 1);
                    else
                        hal12 <= shift_right(hal11, 3);
                    end if;
                else
                    hal12 <= hal11;
                end if;
                -- land the dry video (BRAM out -> fabric FF before any use)
                r12_y <= unsigned(ring_q(29 downto 20));
                r12_u <= unsigned(ring_q(19 downto 10));
                r12_v <= unsigned(ring_q(9 downto 0));
            end if;

            -- S13: CEL posterizer, a stage of its own.  Luma keeps its top
            -- s_pkeep bits and replicates them down (0 -> black, full -> white
            -- at every depth), clamped to legal video; chroma rounds to a grid
            -- one bit finer, centred on 512.  s_pkeep = 5 is the identity, so
            -- every other material -- and CEL at EMERGE 0 -- passes clean.
            if av(13) = '1' then
                case to_integer(s_pkeep) is
                    when 1 => v_rep := (others => r12_y(9));
                    when 2 => v_rep := r12_y(9 downto 8) & r12_y(9 downto 8) & r12_y(9 downto 8)
                                       & r12_y(9 downto 8) & r12_y(9 downto 8);
                    when 3 => v_rep := r12_y(9 downto 7) & r12_y(9 downto 7) & r12_y(9 downto 7)
                                       & r12_y(9);
                    when 4 => v_rep := r12_y(9 downto 6) & r12_y(9 downto 6) & r12_y(9 downto 8);
                    when others => v_rep := r12_y;
                end case;
                if v_rep < 64 then
                    d_y <= to_unsigned(64, 10);
                elsif v_rep > 940 then
                    d_y <= to_unsigned(940, 10);
                else
                    d_y <= v_rep;
                end if;
                v_cu := resize(r12_u, 11) + resize(s_phalf_c, 11);
                v_cv := resize(r12_v, 11) + resize(s_phalf_c, 11);
                if v_cu(10) = '1' then
                    d_u <= s_pmask_c;
                else
                    d_u <= v_cu(9 downto 0) and s_pmask_c;
                end if;
                if v_cv(10) = '1' then
                    d_v <= s_pmask_c;
                else
                    d_v <= v_cv(9 downto 0) and s_pmask_c;
                end if;
                if s_matoff = '1' then
                    hal14g <= (others => '0');
                else
                    hal14g <= resize(shift_right(hal12 * s_halogx, 8), 8);
                end if;
                g14_grain <= grain;
                a14_glow <= a_glow12;
                a14_body <= a_body12;
                a14_core <= a_core12;
                -- the hue wheel lands off its EBR (address issued at S11)
                hue14_u <= unsigned(hue_q(19 downto 10));
                hue14_v <= unsigned(hue_q(9 downto 0));
            end if;

            -- S14: interior -- the video crossfading to the material's own
            -- ground (S8 keeps the video visible THROUGH the material)
            if av(14) = '1' then
                pa_y <= resize(d_y * s_intwc(6 downto 3), 14);
                pb_y <= resize(d_y * s_intwc(2 downto 0), 13);
                pa_u <= resize(d_u * s_intwc(6 downto 3), 14);
                pb_u <= resize(d_u * s_intwc(2 downto 0), 13);
                pa_v <= resize(d_v * s_intwc(6 downto 3), 14);
                pb_v <= resize(d_v * s_intwc(2 downto 0), 13);

                -- material chroma: priority select over per-frame constants
                -- (core > body > halo), one lerp instead of three
                -- S10 lays the wheel over any mode, and in SPECTRAL (already
                -- the wheel) flips it back to the flat K6 palette
                if (s_is_spectral xor s_spec) = '1' then
                    m14_u <= hue14_u;
                    m14_v <= hue14_v;
                elsif a14_core > 128 then
                    m14_u <= s_cu;                 -- core: white hot
                    m14_v <= s_cv;
                elsif a14_body > 64 then
                    m14_u <= s_bu;                 -- body: the tube colour
                    m14_v <= s_bv;
                else
                    m14_u <= s_qu;                 -- reach: the halo colour
                    m14_v <= s_qv;
                end if;

                -- presence: the material's own alpha, or the field halo
                if a14_body > hal14g then
                    a14_pres <= a14_body;
                else
                    a14_pres <= hal14g;
                end if;
            end if;

            -- S15: finish the interior (one add against the per-frame
            -- constant term) and total the material amount
            if av(15) = '1' then
                if s_dark = '1' then
                    g15_y <= resize(shift_right(resize(pa_y & "000", 17)
                             + resize(pb_y, 17), 6), 10) + s_gndw_y
                             - resize(g14_grain, 10);
                else
                    g15_y <= resize(shift_right(resize(pa_y & "000", 17)
                             + resize(pb_y, 17), 6), 10) + s_gndw_y;
                end if;
                g15_u <= resize(shift_right(resize(pa_u & "000", 17)
                         + resize(pb_u, 17), 6), 10) + s_gndw_u;
                g15_v <= resize(shift_right(resize(pa_v & "000", 17)
                         + resize(pb_v, 17), 6), 10) + s_gndw_v;
                v15_mat <= resize(shift_right(a14_glow, 2), 9)
                           + resize(shift_right(a14_body, 1), 9)
                           + resize(shift_right(a14_core, 1), 9);
                m15_u <= m14_u;
                m15_v <= m14_v;
                a15_pres <= a14_pres;
                a15_body <= a14_body;
            end if;

            -- S16: material luma (capped well below full scale -- a
            -- full-scale Y clips every RGB primary and kills chroma), and
            -- the chroma product on its own stage
            if av(16) = '1' then
                v_mat := v15_mat;
                g16_u <= g15_u;
                g16_v <= g15_v;
                a16_pres <= a15_pres;
                -- the subtract sits here so the product below gets a stage
                -- to itself: fused, this was the design's ceiling
                d16_u <= signed(resize(m15_u, 12)) - signed(resize(g15_u, 12));
                d16_v <= signed(resize(m15_v, 12)) - signed(resize(g15_v, 12));
                if s_is_cel = '1' then
                    -- a clean black line: the body alpha alone, x4, so the
                    -- stroke reaches black on any flat and the AA edge is a
                    -- pixel wide instead of a wash
                    if resize(g15_y, 12) < resize(a15_body & "00", 12) + 64 then
                        c16b_y <= to_unsigned(64, 10);
                    else
                        c16b_y <= resize(resize(g15_y, 12) - resize(a15_body & "00", 12), 10);
                    end if;
                elsif s_dark = '1' then
                    -- ink materials take light away from the paper
                    if resize(g15_y, 12) < resize(v_mat, 12) + 64 then
                        c16b_y <= to_unsigned(64, 10);
                    else
                        c16b_y <= resize(g15_y - resize(v_mat, 10), 10);
                    end if;
                else
                    -- the 768 cap applies to DRAWN material, not to video:
                    -- clamping the sum outright crushed a white passthrough
                    -- to 80%.  Material may never push luma past 768, but it
                    -- must not pull already-brighter video down either.
                    if g15_y > 768 then
                        v_lim := g15_y;
                    else
                        v_lim := to_unsigned(768, 10);
                    end if;
                    if resize(g15_y, 12) + resize(v_mat & '0', 12) > resize(v_lim, 12) then
                        c16b_y <= v_lim;
                    else
                        c16b_y <= resize(g15_y + resize(v_mat & '0', 10), 10);
                    end if;
                end if;
            end if;

            -- S17: the chroma products, alone on their stage
            if av(17) = '1' then
                q17_u <= resize(d16_u * signed('0' & a16_pres(7 downto 2)), 18);
                q17_v <= resize(d16_v * signed('0' & a16_pres(7 downto 2)), 18);
                g17_u <= g16_u;
                g17_v <= g16_v;
                if s_inv = '1' then
                    c16_y <= to_unsigned(1004, 10) - c16b_y;
                else
                    c16_y <= c16b_y;
                end if;
            end if;

            -- S18: finish the chroma lerp
            if av(18) = '1' then
                c18_y <= c16_y;
                c15_u <= f_cu10(signed(resize(g17_u, 12))
                         + resize(shift_right(q17_u, 6), 12));
                c15_v <= f_cu10(signed(resize(g17_v, 12))
                         + resize(shift_right(q17_v, 6), 12));
            end if;

            s_dith <= C_BAYER(to_integer(s_x(1 downto 0) & s_aline(1 downto 0)));
            -- Kirlian crackle's per-frame jitter, pre-added off the S10 path
            -- (a clock of lag on a free-running coordinate is invisible)
            -- v1.1.4: in virtual units, so the filament pitch follows the
            -- screen height and the diagonals stay 45 degrees at SD
            s_xj  <= s_vxa(12 downto 1) + resize(s_jx, 12);
            s_alj <= s_vya(18 downto 8) + resize(s_jy, 11);
            -- one gate for the whole material, alphas AND field halo: the
            -- field keeps decaying for a few frames after EMERGE drops, and
            -- an ungated halo still tinted chroma at 0
            if resize(s_dith, 5) >= s_wetn then
                s_matoff <= '1';
            else
                s_matoff <= '0';
            end if;

            -- the field engine is keyed to the pixel at stage 10, so its
            -- injection and its display value are the same pixel
            if av(10) = '1' then
                v_fv := resize(s_fva, 7) + resize(s_xs, 7);
                if v_fv >= 32 then
                    s_fva  <= resize(v_fv - 32, 6);
                    s_fsub <= (others => '0');
                    s_fcol <= s_fcol + 1;
                else
                    s_fva  <= v_fv(5 downto 0);
                    s_fsub <= s_fsub + 1;
                end if;
            end if;

            -- video delay: the dry path rides an EBR ring, never swapped
            ring_w <= ring_w + 1;

            -- line bookkeeping
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x   <= (others => '0');
                s_vxa <= (others => '0');
                s_fva  <= (others => '0');
                s_fsub <= (others => '0');
                s_fcol <= (others => '0');
                lrx   <= (others => '0');
                lwx   <= (others => '0');
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 2000 then
                        s_aline <= s_aline + 1;
                        s_vya   <= s_vya + s_ys;
                    end if;
                    -- cell-row bookkeeping: the field updates on the last
                    -- line of each cell row, when its injection is complete
                    if s_inrow + 1 >= s_ch then
                        s_inrow <= (others => '0');
                        if s_crow < 63 then
                            s_crow <= s_crow + 1;
                        end if;
                    else
                        s_inrow <= s_inrow + 1;
                    end if;
                    -- the frame's last active line closes whatever row it
                    -- is in, so the partial bottom row completes too
                    if s_inrow + 2 >= s_ch or s_aline + 1 = s_hlast then
                        s_lastl <= '1';
                    else
                        s_lastl <= '0';
                    end if;
                else
                    s_aline <= (others => '0');
                    s_vya   <= (others => '0');
                    s_inrow <= (others => '0');
                    s_crow  <= (others => '0');
                    s_lastl <= '0';
                end if;
            end if;

            -- measured raster (captured on active lines, never at vsync)
            if data_in.avid = '1' and s_x > 100 then
                s_meas_w <= s_x;
            end if;
            if s_seen = '1' and s_aline > 100 then
                s_meas_h <= s_aline;
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- luma line buffers: canonical 1W1R, read leads write by one stage
    ------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(5) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(mix3);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(5) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    ------------------------------------------------------------------------
    -- coarse level: one word per cell column holding a fast and a slow
    -- one-pole down the frame.  Their difference is the vertical gradient
    -- (a difference of time constants IS a derivative when time is Y).
    ------------------------------------------------------------------------
    p_clum : process(clk)
    begin
        if rising_edge(clk) then
            cl_q <= clum(to_integer(cl_rd));
            if cl_we = '1' then
                clum(to_integer(cl_wr)) <= cl_wdat;
            end if;
        end if;
    end process p_clum;

    ------------------------------------------------------------------------
    -- field + aux memories
    ------------------------------------------------------------------------
    p_fld : process(clk)
    begin
        if rising_edge(clk) then
            f_q <= fld(to_integer(f_addr));
            if f_we = '1' then
                fld(to_integer(f_wadr)) <= f_wdat;
            end if;
        end if;
    end process p_fld;

    p_aux : process(clk)
    begin
        if rising_edge(clk) then
            x_q <= aux(to_integer(x_radr));
            if x_we = '1' then
                aux(to_integer(x_wadr)) <= x_wdat;
            end if;
        end if;
    end process p_aux;

    p_qsin : process(clk)
    begin
        if rising_edge(clk) then
            qs_data <= qs_rom(to_integer(qs_addr));
        end if;
    end process p_qsin;

    p_hue : process(clk)
    begin
        if rising_edge(clk) then
            hue_q <= hue_rom(to_integer(hue_addr));
        end if;
    end process p_hue;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ring_q <= vring(to_integer(ring_w - 9));
            -- the ring stores chroma in the INTERNAL (true Cb/Cr) order, the
            -- same swap the pixel path takes at S0.  The dry video now flows
            -- through the compose, so the output swap has to cancel an input
            -- swap or passthrough comes out cold.
            vring(to_integer(ring_w)) <=
                "00" & std_logic_vector(r0_y) & std_logic_vector(data_in.v)
                & std_logic_vector(data_in.u);
        end if;
    end process p_ring;


    ------------------------------------------------------------------------
    -- coarse level.  One word per cell column: a fast and a slow one-pole
    -- down the frame.  Horizontally it is a symmetric 2^cwsh box (the whole
    -- line is in hand); vertically it is CAUSAL, because a symmetric
    -- vertical blur would need lines the raster has not delivered.  So it
    -- feeds tone, levels, injection and the scale blend -- never a position.
    ------------------------------------------------------------------------
    p_coarse : process(clk)
        variable v_c : unsigned(6 downto 0);
        variable v_f, v_s : unsigned(7 downto 0);
        variable v_co     : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            cl_we <= '0';
            if av(1) = '1' then
                -- accumulate one cell column, then hand over the mean
                if (s_cwsh = 4 and s_x(3 downto 0) = "0000")
                   or (s_cwsh = 3 and s_x(2 downto 0) = "000") then
                    cacc <= resize(a1_y8, 16);
                    if s_cwsh = 4 then
                        cmean <= resize(shift_right(cacc, 4), 8);
                        v_c := resize(shift_right(s_x, 4), 7) - 1;
                    else
                        cmean <= resize(shift_right(cacc, 3), 8);
                        v_c := resize(shift_right(s_x, 3), 7) - 1;
                    end if;
                    -- the one-poles for the column just closed
                    cl_rd <= v_c + 1;
                    cl_wr <= v_c;
                    v_f := unsigned(cl_q(15 downto 8));
                    v_s := unsigned(cl_q(7 downto 0));
                    if s_aline < 2 then
                        v_f := cmean;
                        v_s := cmean;
                    else
                        v_f := v_f - shift_right(v_f, 2) + shift_right(cmean, 2);
                        v_s := v_s - shift_right(v_s, 4) + shift_right(cmean, 4);
                    end if;
                    cl_wdat <= std_logic_vector(v_f) & std_logic_vector(v_s);
                    cl_we <= '1';
                    cfast_l <= cfast;
                    cfast   <= v_f;
                    cslow   <= v_s;
                else
                    cacc <= cacc + resize(a1_y8, 16);
                end if;
            end if;
            -- The coarse level is one value per cell column, and the window
            -- that consumes it is a GRADIENT operator -- so a staircase is
            -- read as a comb of vertical edges, one per cell, which at high
            -- SCALE drew bright streaks down the whole frame.  Interpolating
            -- between column means only weakens it: piecewise-linear has
            -- discontinuous slope at every knot, and slope is exactly what
            -- Sobel sees.  A one-pole along the line has no knots at all.
            if av(2) = '1' then
                v_co := signed(resize(co3, 10))
                        + shift_right(signed(resize(cfast, 10))
                                      - signed(resize(co3, 10)), 2);
                co3 <= unsigned(v_co(7 downto 0));
            end if;
        end if;
    end process p_coarse;

    ------------------------------------------------------------------------
    -- field engine.  Per cell: two prefetch reads, one recursion write, one
    -- aux read/write.  Every access is on ONE port at cell rate, and the
    -- cell's four corner values are prefetched a cell ahead so the display
    -- never reads a value mid-update.
    --
    --     F = max( inject, spread * max(4 neighbours), persist * F )
    --
    -- The write lands on the LAST line of a cell row, when that row's
    -- injection is complete.  aux carries {above-row old value, running
    -- inject} per column, so the whole recursion needs one 4-bit memory.
    ------------------------------------------------------------------------
    p_field : process(clk)
        variable v_sub : unsigned(4 downto 0);
        variable v_col : unsigned(6 downto 0);
        variable v_nb  : unsigned(3 downto 0);
        variable v_new : unsigned(3 downto 0);
        variable v_sp, v_pe : unsigned(3 downto 0);
        variable v_t, v_b : unsigned(8 downto 0);
        variable v_dt, v_db : signed(5 downto 0);
    begin
        if rising_edge(clk) then
            f_we  <= '0';
            x_we  <= '0';

            -- cell-row edge flags (s_crow moves at hsync, the engine is idle
            -- then, so one clock of registration is free)
            if s_crow = 63 then
                s_crowb <= to_unsigned(63, 6);     -- clamp: no row 64
                s_rowl  <= '1';
            else
                s_crowb <= s_crow + 1;
                s_rowl  <= '0';
            end if;
            if s_crow = 0 then
                s_row0 <= '1';
            else
                s_row0 <= '0';
            end if;
            if s_inrow = 0 then
                s_rowf <= '1';
            else
                s_rowf <= '0';
            end if;

            -- v1.1.4: the cell clock runs in virtual units (16 per cell)
            v_sub := resize(s_fsub, 5);
            v_col := s_fcol;
            fcol   <= v_col;
            fsub_r <= v_sub;

            -- vertical lerp weight: the line index inside the cell row,
            -- scaled to 0..8 by a shift chosen from the cell height.
            -- v1.1.4: 720p's 12-line rows took the unscaled index (0..11),
            -- so 8 - w wrapped on the last three lines of every row and
            -- striped the halo; 10..12-line rows now take 3/4 of it.
            if s_ch >= 13 then
                f_wy3 <= resize(shift_right(s_inrow, 1), 4);
            elsif s_ch >= 10 then
                f_wy3 <= resize(shift_right(resize(s_inrow, 8)
                                 + shift_left(resize(s_inrow, 8), 1), 2), 4);
            elsif s_ch >= 8 then
                f_wy3 <= resize(s_inrow, 4);
            else
                f_wy3 <= resize(s_inrow & '0', 4);
            end if;

            if av(10) = '1' then
                -- running per-cell injection maximum
                if v_sub = 0 then
                    f_inja <= inj10;
                elsif inj10 > f_inja then
                    f_inja <= inj10;
                end if;

                case to_integer(v_sub) is
                    when 0 =>
                        -- prefetch the cell after next; shuffle the chain
                        f_addr <= s_crow & (v_col + 2);
                        x_radr <= v_col - 1;
                        fa_p2 <= fa_p1;
                        fa_p1 <= fa_c;
                        fa_c  <= fa_n;
                        fa_n  <= fa_pf;
                        -- the bottom row has no row below it: the DISPLAY
                        -- still reads row 63 (clamped, so the bottom edge
                        -- stays flat instead of fading), but the recursion
                        -- must not take a neighbour from there
                        if s_rowl = '1' then
                            fb_p1 <= (others => '0');
                        else
                            fb_p1 <= fb_c;
                        end if;
                        fb_c  <= fb_n;
                        fb_n  <= fb_pf;
                        f_injd <= f_inja;

                    when 1 =>
                        f_addr <= s_crowb & (v_col + 2);

                    when 2 =>
                        fa_pf <= unsigned(f_q);

                    when 3 =>
                        fb_pf <= unsigned(f_q);
                        -- the top row's "above" is last frame's row 63
                        if s_row0 = '1' then
                            f_above <= (others => '0');
                        else
                            f_above <= unsigned(x_q(7 downto 4));
                        end if;
                        -- a row's inject accumulator starts empty; guards
                        -- the frame boundary if the height ever changes
                        if s_rowf = '1' then
                            f_injr <= (others => '0');
                        else
                            f_injr <= unsigned(x_q(3 downto 0));
                        end if;

                    when 4 =>
                        -- the next reload's corner differences (fa_pf landed
                        -- at state 2, fb_pf at 3; neither moves again until
                        -- the next cell's state 0 consumes them)
                        s_dt4 <= resize(signed('0' & fa_pf), 6)
                                 - resize(signed('0' & fa_n), 6);
                        s_db4 <= resize(signed('0' & fb_pf), 6)
                                 - resize(signed('0' & fb_n), 6);
                        -- the recursion is spread over four of the cell's
                        -- spare clocks; fused into one state it was a
                        -- 36-level path and the whole design's ceiling
                        v_nb := fa_p2;
                        if fa_c    > v_nb then v_nb := fa_c;    end if;
                        if f_above > v_nb then v_nb := f_above; end if;
                        if fb_p1   > v_nb then v_nb := fb_p1;   end if;
                        f_nbr <= v_nb;
                        if f_injd > f_injr then
                            f_injm <= f_injd;
                        else
                            f_injm <= f_injr;
                        end if;

                    when 5 =>
                        f_spv <= resize(shift_right(f_nbr * s_spread, 4), 4);
                        f_pev <= resize(shift_right(fa_p1 * s_perst, 4), 4);
                        -- half rounded toward zero: a floored -1/2 made
                        -- the step overshoot a falling ramp below 0 within
                        -- an 11-px cell, which wrapped to a bright pixel
                        if s_dt4(5) = '1' then
                            s_dth <= shift_right(s_dt4 + 1, 1);
                        else
                            s_dth <= shift_right(s_dt4, 1);
                        end if;
                        if s_db4(5) = '1' then
                            s_dbh <= shift_right(s_db4 + 1, 1);
                        else
                            s_dbh <= shift_right(s_db4, 1);
                        end if;

                    when 6 =>
                        -- 720p's 1.5x corner step, built in the cell's spare
                        -- states from two registers (x + x/2 on ONE signal
                        -- gives a LUT one net on two pins: router1 hangs)
                        s_d15t <= s_dt4 + s_dth;
                        s_d15b <= s_db4 + s_dbh;
                        v_new := f_injm;
                        if f_spv > v_new then v_new := f_spv; end if;
                        if f_pev > v_new then v_new := f_pev; end if;
                        f_newv <= v_new;

                    when 7 =>
                        -- SD cells are 8 clocks wide, so both writes land in
                        -- this last slot; their operands settled at state 4
                        if s_lastl = '1' then
                            f_wadr <= s_crow & (v_col - 1);
                            f_wdat <= std_logic_vector(f_newv);
                            f_we   <= '1';
                        end if;
                        -- aux: on the last line hand this row's value up to
                        -- the next row and clear the injection accumulator
                        x_wadr <= v_col - 1;
                        if s_lastl = '1' then
                            x_wdat <= std_logic_vector(fa_p1) & "0000";
                        else
                            if f_injd > f_injr then
                                x_wdat <= std_logic_vector(f_above)
                                          & std_logic_vector(f_injd);
                            else
                                x_wdat <= std_logic_vector(f_above)
                                          & std_logic_vector(f_injr);
                            end if;
                        end if;
                        x_we <= '1';

                    when others =>
                        null;
                end case;

                -- horizontal interpolation as a DDA: reload at the cell
                -- edge from the values the chain is about to shift in, then
                -- one add per pixel.  Exact, not an approximation -- after 16
                -- adds the accumulator IS the next cell's value.
                -- v1.1.4: a cell spans 16 virtual units, so the per-pixel
                -- step is the corner difference times the x step (1x 1080p,
                -- 1.5x 720p, 2x SD).  SD used to take 1x over its 8-px
                -- cells: half a ramp, then a jump at every cell edge.
                if v_sub = 0 then
                    acc_t <= shift_left(resize(signed('0' & fa_n), 14), 4);
                    acc_b <= shift_left(resize(signed('0' & fb_n), 14), 4);
                    v_dt := resize(signed('0' & fa_pf), 6)
                            - resize(signed('0' & fa_n), 6);
                    v_db := resize(signed('0' & fb_pf), 6)
                            - resize(signed('0' & fb_n), 6);
                    case to_integer(s_xs) is
                        when 4 =>
                            stp_t <= shift_left(v_dt, 1);
                            stp_b <= shift_left(v_db, 1);
                        when 3 =>
                            stp_t <= s_d15t;
                            stp_b <= s_d15b;
                        when others =>
                            stp_t <= v_dt;
                            stp_b <= v_db;
                    end case;
                else
                    acc_t <= acc_t + resize(stp_t, 14);
                    acc_b <= acc_b + resize(stp_b, 14);
                end if;
                f_top <= unsigned(acc_t(8 downto 1));
                f_bot <= unsigned(acc_b(8 downto 1));
                f_disp <= resize(shift_right(
                          resize(f_top * (to_unsigned(8, 4) - f_wy3), 13)
                          + resize(f_bot * f_wy3, 13), 2), 8);
            end if;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                fa_p2 <= (others => '0');
                fa_p1 <= (others => '0');
                fa_c  <= (others => '0');
                fa_n  <= (others => '0');
                fb_p1 <= (others => '0');
                fb_c  <= (others => '0');
                fb_n  <= (others => '0');
                f_inja <= (others => '0');
                s_d15t <= (others => '0');
                s_d15b <= (others => '0');
            end if;
        end if;
    end process p_field;

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

    -- S17: EMERGE wet mix against the untouched dry video, then the
    -- blanking gate.  A program that computes from data_in MUST drive
    -- neutral outside avid or the encoder's colour reference is destroyed --
    -- and no simulation catches it.
    p_out : process(clk)
        variable v_u, v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- the swap back to hardware order happens here, in BOTH arms:
            -- Invert is a true negative (both axes negated), and dropping
            -- the swap under it turned passthrough into a swapped negative
            if s_inv = '1' then
                v_u := f_cu10(to_signed(1024, 12) - signed(resize(c15_v, 12)));
                v_v := f_cu10(to_signed(1024, 12) - signed(resize(c15_u, 12)));
            else
                v_u := c15_v;
                v_v := c15_u;
            end if;
            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= C_BLKY;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= c18_y;
                s_out_u <= v_u;
                s_out_v <= v_v;
            end if;
        end if;
    end process p_out;

    s_io_align_in.y       <= std_logic_vector(s_out_y);
    s_io_align_in.u       <= std_logic_vector(s_out_u);
    s_io_align_in.v       <= std_logic_vector(s_out_v);
    s_io_align_in.avid    <= s_avid_sr(C_LATENCY - 1);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_LATENCY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (19 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture livewire;
