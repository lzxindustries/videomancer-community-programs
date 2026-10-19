-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- rollthebones.vhd  (v1.0 — video as a lit tabletop of six-sided dice)
--
-- v1.0.3 (2026-10-17): resolution scaling.  The cell lattice runs in virtual
-- 1080-line units, stepped per pixel / per field line from the video timing
-- ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p, and
-- at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame line, so
-- the dice keep their size relative to the screen height and stay square on
-- a 4:3 screen (they were 1.5x too big at 720p and huge 2:1 slabs at SD).
-- The row counter runs the integer part of each field line's step as unit
-- steps after the h edge; the sample line is the first line at/after
-- D - ceil(step) (the last line at 1080p).  The 8-px denoise window stays 8
-- REAL pixels (counted), starting Dh - 7 steps into the cell.
--
-- The incoming picture is re-rendered as a full-screen grid of six-sided dice.
-- Every cell holds one die: a rounded-corner square face carrying 1..6 drilled
-- pips.  The pip count tracks the local video (luma, or chroma saturation), so
-- the field of dice reads as a halftone portrait of the source.  An auto-level
-- loop keeps the halftone centred on the picture's median, so every source
-- (and Chroma mode) uses all six faces.
--
-- Look: dice are lit by a movable light (fixed in screen space).  The flat top
-- face carries a soft pillow bevel (bright toward the light, dark on the far
-- side), normalised to die size so it reads the same at every zoom.  Gloss
-- (K5) runs matte chalk -> lacquer: the shade depth grows, and from 25% a white
-- glint spreads along the lit edges.  Each recessed pip is a shallow dimple
-- with a tiny rim glint.  Drop shadows fall away from the light.
--
-- Colour: eight inks — Casino White, Blood Red, Onyx, Ivory, Jade, Gold, plus
-- per-die Rainbow (each die a different wheel hue) and Spectrum (die hue driven
-- by the sampled video chroma).  K3 Hue rotates the felt and painted inks (unity
-- at centre) and, on the achromatic inks, tints the pips.  Ink=Video tints the
-- die bodies with the live picture's colour, and there K3 becomes the bodies'
-- SATURATION instead (0 = grey, 50% = unchanged, 100% = ~2x).
--
-- Geometry / timing (modelled on gumball): a single square lattice; each cell's
-- die parameters are sampled once, on the LAST line of the previous cell row,
-- into a ping-pong BRAM (buf0/buf1 by row parity) so a row's dice are ready
-- before their first pixel.  Only the die radius R is stored; the inner/corner/
-- pip radii are slices or one add of R in the pixel path.  Per-frame values
-- settle in a vblank FSM on one shared multiplier.  No pixel-path divide.
--
-- Rotation is exact without a cosine: the pixel path rotates by u = x + y*t,
-- v = y - x*t (t = tan, |t| <= 1, i.e. +/-45 deg), which is a true rotation
-- scaled by sec; the per-die radius is pre-scaled by sec in the sample pipe.
-- (A rot90 bit transposing faces 2/3/6 is kept for angles past 45 deg.)
--
-- Tumble (P12) spills the neat aligned grid into a scattered, per-die-rotated,
-- shadow-casting pile: dice shrink, jitter, cast longer shadows and tilt by a
-- stable per-die angle that grows as (P12)^2 up to +/-45 deg at 100%.
--
-- Colour convention: all constant inks are stored with U/V (Cb/Cr) SWAPPED to
-- match the Videomancer hardware output convention (as in mondrian/CGA/C64).
-- Live video (backdrop / bypass / Ink=Video) passes through in its native
-- convention.  NOTE: headless BT.601 sims therefore show the inks U/V-swapped;
-- verify geometry + luma in sim, trust colour from the verified-palette rule.
--
-- Controls:
--   K1 Zoom  K2 Palette  K3 Hue  K4 Light  K5 Gloss  K6 Contrast
--   S7 Source(Luma/Chroma)  S8 Luma Size  S9 Table  S10 Ink  S11 Bypass
--   P12 Tumble
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

architecture rollthebones of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (13 + 3 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY  : natural := 12;                 -- video pipe depth: sync/bypass
                                                       -- tap (pipe 11) aligns with the
                                                       -- dice content (c9); bypass uses
                                                       -- the SAME tap, so every mode
                                                       -- presents one latency.
    constant C_MAXCOL : natural := 128;
    constant C_MID    : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_YMAX   : integer := 800;                -- drawn-luma cap (HW: full-scale
                                                       -- Y clips RGB and kills chroma)

    --------------------------------------------------------------------------
    -- Scheme (ink) colour tables.  Chroma stored U/V-SWAPPED (HW convention).
    -- idx: 0 White 1 Red 2 Onyx 3 Ivory 4 Jade 5 Gold 6 Rainbow 7 Spectrum
    --------------------------------------------------------------------------
    type t_pal is array(0 to 7) of integer;
    constant SCH_Y : t_pal := (600, 300, 110, 620, 430, 560, 600, 600);
    constant SCH_U : t_pal := (512, 850, 512, 540, 356, 650, 512, 512);  -- = Cr
    constant SCH_V : t_pal := (512, 400, 512, 500, 432, 300, 512, 512);  -- = Cb
    constant SCH_POL : std_logic_vector(0 to 7) := "01101000";  -- '1' dark body, light pips

    -- 16-point chroma wheel, stored U/V-swapped (HW convention).
    type t_h16 is array(0 to 15) of integer;
    constant HUE_U : t_h16 := (512,619,710,771,792,771,710,619,512,405,314,253,232,253,314,405);
    constant HUE_V : t_h16 := (792,771,710,619,512,405,314,253,232,253,314,405,512,619,710,771);

    -- Felt table colour (HW-swapped); K3 rotates its hue per frame.
    constant FELT_Y : integer := 150;
    constant FELT_U : integer := 356;
    constant FELT_V : integer := 430;

    -- 64-step cosine (x255) for the per-frame K3 hue rotation.
    type t_c64 is array(0 to 63) of integer range -255 to 255;
    constant COS64 : t_c64 := (
        255,254,250,244,236,225,212,197,180,162,142,120,98,74,50,25,
        0,-25,-50,-74,-98,-120,-142,-162,-180,-197,-212,-225,-236,-244,-250,-254,
        -255,-254,-250,-244,-236,-225,-212,-197,-180,-162,-142,-120,-98,-74,-50,-25,
        0,25,50,74,98,120,142,162,180,197,212,225,236,244,250,254);

    constant C_SEED     : unsigned(15 downto 0) := x"ACE1";
    constant C_SEEDSTEP : unsigned(15 downto 0) := x"9E37";   -- per-row Weyl step

    -- Face (+ rot90) -> 3x3 pip mask.  node n = (gv+1)*3+(gu+1); mask(n)='1' -> pip.
    -- A quarter turn transposes the asymmetric faces (2, 3, 6).
    function facemask(f : unsigned; rot : std_logic) return std_logic_vector is
    begin
        case to_integer(f) is
            when 1      => return "000010000";
            when 2      => if rot = '1' then return "001000100"; else return "100000001"; end if;
            when 3      => if rot = '1' then return "001010100"; else return "100010001"; end if;
            when 4      => return "101000101";
            when 5      => return "101010101";
            when others => if rot = '1' then return "111000111"; else return "101101101"; end if;
        end case;
    end function;

    -- signed chroma offset -> 10-bit code (offset + 512, mod 1024)
    function f_c512(x : signed) return unsigned is
    begin
        return unsigned(resize(x, 10)) + 512;
    end function;

    -- 16-step LFSR advance (a fresh 16-bit state per die).
    function lfsr16(s : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0) := s;
    begin
        for i in 1 to 16 loop
            v := v(14 downto 0) & (v(15) xor v(13) xor v(12) xor v(10));
        end loop;
        return v;
    end function;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID
    -- (registers_in(8)); 1.0 at 1080p, so 1080p is unchanged.
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
    -- Raster position.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_anchor     : std_logic := '0';

    -- Resolution scaling (v1.0.3): the lattice runs in virtual 1080-line
    -- units.  x: a Q.1 accumulator (half units; step 2/3/4 = 1.0/1.5/2.0 per
    -- pixel), xla(8 downto 1) is the old integer xloc.  y: celly_loc still
    -- counts whole virtual lines, but each field line advances it by the
    -- integer part of a Q4.8 accumulator (1 at 1080p, 1-2 at 720p, 4-5 at
    -- 480i), one unit step per clock after the h edge.
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal xla      : unsigned(8 downto 0) := (others => '0');
    signal x_ws     : std_logic := '0';                    -- this cell's sample window started
    signal wcnt     : unsigned(3 downto 0) := (others => '0');  -- window: 8..2 add, 1 strobe
    signal cy_frac  : unsigned(7 downto 0) := (others => '0');
    signal cy_left  : unsigned(2 downto 0) := (others => '0');  -- unit steps still due
    signal cy_blw   : std_logic := '1';                    -- previous line short of D - k
    signal srow_r   : std_logic := '0';                    -- this line samples the next row
    signal xidx      : unsigned(6 downto 0) := (others => '0');
    signal celly_loc : unsigned(7 downto 0) := (others => '0');
    signal celly_par : std_logic := '0';
    signal frame_act : std_logic := '0';
    signal line0     : std_logic := '0';
    signal dyc_line  : signed(9 downto 0) := (others => '0');
    signal rowseed   : unsigned(15 downto 0) := C_SEED;

    --------------------------------------------------------------------------
    -- Per-frame parameters.
    --------------------------------------------------------------------------
    signal rk1, rp12 : unsigned(9 downto 0) := (others => '0');     -- raw (pre-deadband)
    signal d1, d12   : signed(10 downto 0) := (others => '0');
    signal lk1,lk2,lk3,lk4,lk5,lk6,lp12 : unsigned(9 downto 0) := (others => '0');

    signal s_d    : unsigned(7 downto 0) := to_unsigned(71, 8);
    signal s_dm1  : unsigned(7 downto 0) := to_unsigned(70, 8);
    signal s_dh   : unsigned(7 downto 0) := to_unsigned(35, 8);
    signal s_dhlo : unsigned(8 downto 0) := to_unsigned(56, 9);   -- 8-px denoise window start (half units)
    signal s_dlim : unsigned(8 downto 0) := to_unsigned(140, 9);  -- 2D - x step: cell wrap threshold
    signal s_dmk  : unsigned(7 downto 0) := to_unsigned(70, 8);   -- D - ceil(y step): sample-line threshold
    signal s_r0   : unsigned(7 downto 0) := to_unsigned(31, 8);
    signal s_bw   : unsigned(7 downto 0) := to_unsigned(7, 8);
    signal s_rmax : unsigned(7 downto 0) := to_unsigned(34, 8);
    signal s_rotsc: unsigned(7 downto 0) := (others => '0');
    signal s_jitsc: unsigned(7 downto 0) := (others => '0');
    signal s_shad : unsigned(7 downto 0) := (others => '0');
    signal s_q    : unsigned(7 downto 0) := (others => '0');      -- tilt scale = (P12/8)^2/128
    signal s_span   : unsigned(8 downto 0) := to_unsigned(179, 9);  -- gloss: dark-side shade depth
    signal s_spanh  : unsigned(8 downto 0) := to_unsigned(89, 9);   -- lit-side shade = span/2
    signal s_spth   : signed(11 downto 0) := to_signed(210, 12);    -- glint threshold (gloss)
    signal s_glossy : std_logic := '0';                             -- glint enabled (K5 >= 25%)
    signal s_bsh    : unsigned(1 downto 0) := "01";                 -- bevel shift 3..6 ~ log2(R)
    signal lx, ly   : signed(7 downto 0) := (others => '0');        -- light vector (cos/4, sin/4)
    signal lx0, ly0 : signed(7 downto 0) := (others => '0');        -- ROM output (EBR-absorbed)
    signal s_hueoff : unsigned(3 downto 0) := (others => '0');
    signal s_idx    : unsigned(2 downto 0) := (others => '0');
    signal s_pol    : std_logic := '0';

    -- face thresholds relative to the auto-level centre: +/-sp, +/-2sp
    signal s_sp, s_sp2, s_nsp, s_nsp2 : signed(11 downto 0) := (others => '0');
    signal s_cen    : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal cen_l    : unsigned(10 downto 0) := to_unsigned(512, 11);  -- luma centre
    signal cen_c    : unsigned(10 downto 0) := to_unsigned(128, 11);  -- chroma centre
    signal lv_diff  : signed(14 downto 0) := (others => '0');     -- above - below
    signal lv_tot   : unsigned(13 downto 0) := (others => '0');   -- samples this frame
    signal lv_ad    : unsigned(14 downto 0) := (others => '0');
    signal lv_neg, lv_mv, lv_big : std_logic := '0';
    signal s_censrc : std_logic := '0';                           -- source s_cen belongs to
    signal lv_st, lv_c : signed(11 downto 0) := (others => '0');
    signal s_pend   : std_logic := '0';

    -- K3 hue rotation (per frame)
    signal s_hix    : unsigned(5 downto 0) := (others => '0');
    signal hc, hs, hsn : signed(10 downto 0) := (others => '0');
    signal tdl      : signed(10 downto 0) := (others => '0');
    signal tsat     : unsigned(7 downto 0) := (others => '0');
    signal idu, idv : signed(9 downto 0) := (others => '0');
    signal acc      : signed(20 downto 0) := (others => '0');
    signal tint_u, tint_v : signed(10 downto 0) := (others => '0');
    signal s_inku, s_inkv   : unsigned(9 downto 0) := C_MID;
    signal s_sat    : unsigned(3 downto 0) := to_unsigned(8, 4);    -- body chroma gain /8 (Ink=Video: K3)
    signal s_feltu  : unsigned(9 downto 0) := to_unsigned(FELT_U, 10);
    signal s_feltv  : unsigned(9 downto 0) := to_unsigned(FELT_V, 10);
    signal s_pipu, s_pipv   : unsigned(9 downto 0) := C_MID;
    signal s_by     : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_py     : unsigned(9 downto 0) := to_unsigned(64, 10);

    signal s_src    : std_logic := '0';
    signal s_lsize  : std_logic := '0';
    signal s_tabvid : std_logic := '0';
    signal s_inkvid : std_logic := '0';
    signal s_bypass : std_logic := '0';

    signal fsm_t  : unsigned(7 downto 0) := (others => '1');
    signal ma     : signed(9 downto 0)  := (others => '0');
    signal mb     : signed(10 downto 0) := (others => '0');
    signal mp     : signed(20 downto 0) := (others => '0');

    signal lut_sin, lut_cos : signed(9 downto 0);

    --------------------------------------------------------------------------
    -- Cell buffers.  word (64b, index 63..0):
    --  face 63..61 | rot90 60 | t 59..51 | jx 50..43 | jy 42..35 | R 34..27 |
    --  bU 26..17 | bV 16..7 | spare 6..0   (R is sec-scaled; Rin/P/pr/rc are
    --  derived from R in the pixel path; body Y and pip polarity are per-frame)
    --------------------------------------------------------------------------
    constant CW : natural := 63;
    subtype t_word is std_logic_vector(CW downto 0);
    type t_cbuf is array(0 to C_MAXCOL - 1) of t_word;
    signal buf0, buf1 : t_cbuf := (others => (others => '0'));
    signal rd0, rd1   : t_word := (others => '0');

    --------------------------------------------------------------------------
    -- Sample pipe.
    --------------------------------------------------------------------------
    signal lfsr : unsigned(15 downto 0) := C_SEED;
    signal acc_y, acc_u, acc_v : unsigned(12 downto 0) := (others => '0');  -- 8px denoise

    signal w0_stb : std_logic := '0';
    signal w0_par : std_logic := '0';
    signal w0_idx : unsigned(6 downto 0) := (others => '0');
    signal w0_u, w0_v : unsigned(9 downto 0) := C_MID;
    signal w0_drv : unsigned(10 downto 0) := (others => '0');
    signal w0_rnd : unsigned(15 downto 0) := (others => '0');

    -- W1: multiplies, level offset, random fields, octant
    signal w1_stb : std_logic := '0';
    signal w1_par : std_logic := '0';
    signal w1_idx : unsigned(6 downto 0) := (others => '0');
    signal w1_u, w1_v : unsigned(9 downto 0) := C_MID;
    signal w1_dl  : signed(11 downto 0) := (others => '0');
    signal w1_rjx, w1_rjy : signed(7 downto 0) := (others => '0');
    signal w2_rjy : signed(7 downto 0) := (others => '0');
    -- one shared per-die multiplier (one die in flight; strobes >= 18 clks apart)
    signal sm_a : signed(9 downto 0) := (others => '0');          -- registered operands
    signal sm_b : signed(8 downto 0) := (others => '0');
    signal sm_p : signed(18 downto 0) := (others => '0');
    signal w1_oct  : unsigned(3 downto 0) := (others => '0');
    signal w1_rrlo : unsigned(3 downto 0) := (others => '0');

    -- W2: face, static tilt, jitter, luma-size sum, hue index
    signal w2_stb : std_logic := '0';
    signal w2_par : std_logic := '0';
    signal w2_idx : unsigned(6 downto 0) := (others => '0');
    signal w2_u, w2_v : unsigned(9 downto 0) := C_MID;
    signal w2_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w2_rs   : signed(11 downto 0) := (others => '0');      -- r0 + dl*3/16
    signal w2_hidx : unsigned(3 downto 0) := (others => '0');

    -- W3: angle, radius, face, body colour
    signal w3_stb : std_logic := '0';
    signal w3_par : std_logic := '0';
    signal w3_idx : unsigned(6 downto 0) := (others => '0');
    signal w3_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w3_a    : signed(9 downto 0) := to_signed(256, 10);
    signal w3_r    : unsigned(7 downto 0) := (others => '0');
    signal w3_bu, w3_bv : unsigned(9 downto 0) := C_MID;

    -- W4: tan + rot90 + sec terms (two selected shifts of R)
    signal w4_stb : std_logic := '0';
    signal w4_par : std_logic := '0';
    signal w4_idx : unsigned(6 downto 0) := (others => '0');
    signal w4_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w4_rot  : std_logic := '0';
    signal w4_t    : signed(8 downto 0) := (others => '0');
    signal w4_r, w4_s1, w4_s2 : unsigned(7 downto 0) := (others => '0');
    signal w4_jx, w4_jy : signed(7 downto 0) := (others => '0');
    signal w4_bv : unsigned(9 downto 0) := C_MID;

    -- W5: packed word ready to write
    -- W5: R', body U (gained); W6: packed word
    signal w5_stb : std_logic := '0';
    signal w5_par : std_logic := '0';
    signal w5_idx : unsigned(6 downto 0) := (others => '0');
    signal w5_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w5_rot  : std_logic := '0';
    signal w5_t    : signed(8 downto 0) := (others => '0');
    signal w5_jx, w5_jy : signed(7 downto 0) := (others => '0');
    signal w5_rp   : unsigned(7 downto 0) := (others => '0');
    signal w6_stb : std_logic := '0';
    signal w6_par : std_logic := '0';
    signal w6_idx : unsigned(6 downto 0) := (others => '0');
    signal w6_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w6_rot  : std_logic := '0';
    signal w6_t    : signed(8 downto 0) := (others => '0');
    signal w6_jx, w6_jy : signed(7 downto 0) := (others => '0');
    signal w6_rp   : unsigned(7 downto 0) := (others => '0');
    signal w6_cu   : signed(9 downto 0) := (others => '0');
    signal w7_stb : std_logic := '0';
    signal w7_par : std_logic := '0';
    signal w7_idx : unsigned(6 downto 0) := (others => '0');
    signal w7_word : t_word := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline (c1..c9 + c10 output register in s_io).
    --------------------------------------------------------------------------
    signal c1_dx0 : signed(9 downto 0) := (others => '0');
    signal c1_par : std_logic := '0';

    signal c2_word : t_word := (others => '0');
    signal c2_dxj, c2_dyj : signed(9 downto 0) := (others => '0');

    signal c3_dxj, c3_dyj : signed(9 downto 0) := (others => '0');
    signal c3_rsx, c3_rsy : signed(16 downto 0) := (others => '0');
    signal c3_word : t_word := (others => '0');

    signal c4_u, c4_v   : signed(9 downto 0) := (others => '0');
    signal c4_ub, c4_vb : signed(7 downto 0) := (others => '0');   -- screen-space, clamped
    signal c4_au, c4_av : unsigned(8 downto 0) := (others => '0');
    signal c4_rin, c4_pr : unsigned(7 downto 0) := (others => '0');
    signal c4_word : t_word := (others => '0');

    signal c5_cu, c5_cv   : unsigned(8 downto 0) := (others => '0');
    signal c5_incorner    : std_logic := '0';
    signal c5_edgein      : std_logic := '0';
    signal c5_bx, c5_by   : signed(15 downto 0) := (others => '0');
    signal c5_ru, c5_rv   : signed(8 downto 0) := (others => '0');
    signal c5_node        : unsigned(3 downto 0) := (others => '0');
    signal c5_inband      : std_logic := '0';
    signal c5_su, c5_sv   : std_logic := '0';
    signal c5_maxa        : unsigned(8 downto 0) := (others => '0');
    signal c5_face        : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal c5_rot         : std_logic := '0';
    signal c5_pr          : unsigned(7 downto 0) := (others => '0');
    signal c5_word        : t_word := (others => '0');

    signal c6_cdoct : unsigned(9 downto 0) := (others => '0');
    signal c6_pdoct : unsigned(9 downto 0) := (others => '0');
    signal c6_bev  : signed(16 downto 0) := (others => '0');
    signal c6_incorner : std_logic := '0';
    signal c6_edgein   : std_logic := '0';
    signal c6_active   : std_logic := '0';
    signal c6_inband   : std_logic := '0';
    signal c6_su, c6_sv : std_logic := '0';
    signal c6_maxa     : unsigned(8 downto 0) := (others => '0');
    signal c6_ru, c6_rv : signed(8 downto 0) := (others => '0');
    signal c6_pr       : unsigned(7 downto 0) := (others => '0');
    signal c6_word     : t_word := (others => '0');

    signal c7_indie, c7_inpip : std_logic := '0';
    signal c7_bev  : signed(16 downto 0) := (others => '0');
    signal c7_inband : std_logic := '0';
    signal c7_incorner : std_logic := '0';
    signal c7_shadow   : std_logic := '0';
    signal c7_pdoct : unsigned(9 downto 0) := (others => '0');
    signal c7_pr    : unsigned(7 downto 0) := (others => '0');
    signal c7_ru, c7_rv : signed(8 downto 0) := (others => '0');
    signal c7_word : t_word := (others => '0');

    -- c7b: dimple direction, raw shade, glint, colour unpack (offloads c8a).
    signal c7b_shraw  : signed(11 downto 0) := (others => '0');
    signal c7b_pipdot : signed(9 downto 0) := (others => '0');
    signal c7b_bu, c7b_bv : unsigned(9 downto 0) := C_MID;
    signal c7b_glint  : std_logic := '0';
    signal c7b_inpip, c7b_indie, c7b_shadow : std_logic := '0';
    signal c7b_incorner, c7b_inband : std_logic := '0';

    -- c8a: pre-computed shade / pip value / flags; c8b: final assembly.
    signal c8a_shade : signed(11 downto 0) := (others => '0');
    signal c8a_pipb  : signed(11 downto 0) := (others => '0');
    signal c8a_bu, c8a_bv : unsigned(9 downto 0) := C_MID;
    signal c8a_spec  : std_logic := '0';
    signal c8a_inpip, c8a_indie, c8a_shadow : std_logic := '0';

    signal c8_class : unsigned(1 downto 0) := (others => '0');  -- 0 tbl 1 shadow 2 body 3 pip
    signal c8_y : signed(11 downto 0) := to_signed(64, 12);
    signal c8_u, c8_v : unsigned(9 downto 0) := C_MID;

    signal c9_y, c9_u, c9_v : unsigned(9 downto 0) := C_MID;

    signal s_io   : t_video_stream_yuv444_30b;
    signal s_bpass : t_video_stream_yuv444_30b;   -- 1-cycle raw passthrough (== SDK passthru)

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

begin

    u_sincos : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(lk4),
            sin_out  => lut_sin,
            cos_out  => lut_cos
        );

    --------------------------------------------------------------------------
    -- Raster counters + per-cell sample trigger + per-die random state.
    -- The LFSR is reseeded at every hsync from a per-row Weyl seed and stepped
    -- 16 bits per die, so a die's randomness depends only on its (row, column)
    -- — a jittered analog line that gains/loses the right-edge partial die no
    -- longer reshuffles every die below it.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h, v_v : std_logic;
        variable v_anc    : std_logic;
        variable v_trig   : std_logic;
        variable v_tp     : std_logic;
        variable v_su, v_sv : signed(11 downto 0);
        variable v_online : boolean;
        variable v_avy, v_avu, v_avv : unsigned(9 downto 0);
        variable v_step   : std_logic;
        variable v_ysum   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h := '0'; v_v := '0'; v_anc := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v := '1'; end if;
            if v_v = '0' and data_in.avid = '1' and frame_act = '0' then v_anc := '1'; end if;
            r_anchor <= '0';

            -- per-mode virtual steps (static per mode; re-latching at every
            -- vsync edge is idempotent)
            if v_v = '1' then
                s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            end if;

            -- X lattice in virtual half units.  A cell wraps when the NEXT
            -- step would reach D (xla >= 2D - step: at 1080p xla = 2(D-1),
            -- the old xloc = D-1 test).
            if v_h = '1' then
                xla  <= (others => '0');
                xidx <= (others => '0');
            elsif data_in.avid = '1' then
                if xla >= s_dlim then
                    xla  <= xla - s_dlim;
                    xidx <= xidx + 1;
                else
                    xla <= xla + s_xs;
                end if;
            end if;

            -- Y row counter.  Each h edge adds the Q4.8 field-line step; its
            -- integer part runs as that many unit steps (the first at the
            -- edge itself, the rest on the following clocks — all long before
            -- active video), so the row wrap and row seeds are unchanged.
            v_step := '0';
            if v_v = '1' then
                celly_loc <= (others => '0');
                celly_par <= '0';
                frame_act <= '0';
                cy_left   <= (others => '0');
            elsif v_anc = '1' then
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_par <= '0';
                line0     <= '1';
                r_anchor  <= '1';
                rowseed   <= C_SEED + C_SEEDSTEP;        -- seed for row 1
                cy_frac   <= (others => '0');
                cy_left   <= (others => '0');
            elsif v_h = '1' then
                line0 <= '0';
                v_ysum  := resize(cy_frac, 12) + s_ys;
                cy_frac <= v_ysum(7 downto 0);
                cy_left <= v_ysum(10 downto 8) - 1;
                if celly_loc < s_dmk then cy_blw <= '1'; else cy_blw <= '0'; end if;
                v_step  := '1';
            elsif cy_left /= 0 then
                cy_left <= cy_left - 1;
                v_step  := '1';
            end if;
            if v_step = '1' then
                if celly_loc = s_dm1 then
                    celly_loc <= (others => '0');
                    celly_par <= not celly_par;
                    rowseed   <= rowseed + C_SEEDSTEP;   -- seed for the row after next
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;

            -- dy of the line = the pre-step position of its LAST unit step
            -- (at 1080p the previous line's, exactly as before)
            if v_step = '1' or r_anchor = '1' then
                dyc_line <= signed(resize(celly_loc, 10)) - signed(resize(s_dh, 10));
            end if;

            -- Sample line: the first line of a row at/after D - ceil(y step)
            -- (at 1080p the last line, D-1).  It is still inside the row
            -- above, since a line advances at most ceil(y step).
            if celly_loc >= s_dmk and cy_blw = '1' then srow_r <= '1'; else srow_r <= '0'; end if;

            -- 8-px horizontal area-average around each cell centre on the sample
            -- line (memoryless denoise).  The sample line is the LAST line of
            -- the row above (line 0 for row 0), so the dice sit half a die
            -- lower than the picture instead of a whole die.  The window is 8
            -- REAL pixels (a filter kernel stays at pixel scale): it starts at
            -- the first pixel past s_dhlo (Dh - 7 steps, at least the cell
            -- start), adds 7 more and strobes on the next one (1080p: Dh-7..Dh,
            -- strobe at Dh+1, as before).
            v_online := frame_act = '1' and data_in.avid = '1'
                        and (line0 = '1' or srow_r = '1');
            v_trig := '0';
            v_tp   := not celly_par;
            if v_h = '1' then
                x_ws <= '0';
                wcnt <= (others => '0');
            elsif v_online and x_ws = '0' and xla >= s_dhlo then
                acc_y <= resize(unsigned(data_in.y), 13);
                acc_u <= resize(unsigned(data_in.u), 13);
                acc_v <= resize(unsigned(data_in.v), 13);
                x_ws  <= '1';
                wcnt  <= to_unsigned(8, 4);
            elsif v_online and wcnt > 1 then
                acc_y <= acc_y + unsigned(data_in.y);
                acc_u <= acc_u + unsigned(data_in.u);
                acc_v <= acc_v + unsigned(data_in.v);
                wcnt  <= wcnt - 1;
            elsif v_online and wcnt = 1 then
                -- strobe one pixel after the 8px window closes
                if line0 = '1' then v_tp := '0'; end if;
                v_trig := '1';
                wcnt   <= (others => '0');
            end if;
            if data_in.avid = '1' and xla >= s_dlim then
                x_ws <= '0';                                 -- next cell
            end if;

            w0_stb <= v_trig;
            w0_par <= v_tp;
            w0_idx <= xidx;
            if v_trig = '1' then
                v_avy := resize(shift_right(acc_y, 3), 10);
                v_avu := resize(shift_right(acc_u, 3), 10);
                v_avv := resize(shift_right(acc_v, 3), 10);
                w0_u <= v_avu;
                w0_v <= v_avv;
                v_su := signed(resize(v_avu, 12)) - 512;
                v_sv := signed(resize(v_avv, 12)) - 512;
                if s_src = '1' then
                    w0_drv <= resize(unsigned(abs(v_su)), 11) + resize(unsigned(abs(v_sv)), 11);
                else
                    w0_drv <= '0' & v_avy;
                end if;
            end if;

            if v_anc = '1' then
                lfsr <= C_SEED;                 -- row 0
            elsif v_h = '1' then
                lfsr <= rowseed;
            elsif v_trig = '1' then
                lfsr <= lfsr16(lfsr);
            end if;
            w0_rnd <= lfsr;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Sample pipe.
    --------------------------------------------------------------------------
    p_sample : process(clk)
        variable v_lvl  : integer range 0 to 5;
        variable v_face : integer range 1 to 6;
        variable v_r    : signed(9 downto 0);
        variable v_bu, v_bv : unsigned(9 downto 0);
        variable v_hi   : integer range 0 to 15;
        variable v_du, v_dv : signed(10 downto 0);
        variable v_au, v_av : unsigned(10 downto 0);
        variable v_oct  : integer range 0 to 15;
        variable v_sb   : unsigned(2 downto 0);
        variable v_ls   : signed(13 downto 0);
        variable v_cu   : signed(18 downto 0);
        variable v_sa   : signed(9 downto 0);
        variable v_sbm  : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- Shared multiplier: the stage holding the die picks the
            -- operands (registered), the product registers a cycle later,
            -- and is read two stages after the pick:
            --   W0: tilt  rr*q   -> W3      W1: jx  rjx*jit -> W4
            --   W2: jy   rjy*jit -> W5      W3: U gain      -> W6
            --   W4: V gain       -> W7
            ----------------------------------------------------------------
            if w0_stb = '1' then
                v_sa  := resize(signed((not w0_rnd(7)) & w0_rnd(6 downto 0)), 10);   -- rnd - 128
                v_sbm := signed('0' & s_q);
            elsif w1_stb = '1' then
                v_sa  := resize(w1_rjx, 10);  v_sbm := signed('0' & s_jitsc);
            elsif w2_stb = '1' then
                v_sa  := resize(w2_rjy, 10);  v_sbm := signed('0' & s_jitsc);
            elsif w3_stb = '1' then
                v_sa  := signed((not w3_bu(9)) & w3_bu(8 downto 0));                 -- U - 512
                v_sbm := signed(resize(s_sat, 9));
            else
                v_sa  := signed((not w4_bv(9)) & w4_bv(8 downto 0));                 -- V - 512
                v_sbm := signed(resize(s_sat, 9));
            end if;
            sm_a <= v_sa;  sm_b <= v_sbm;
            sm_p <= sm_a * sm_b;

            ----------------------------------------------------------------
            -- W1: level offset, the three scaling multiplies, random fields,
            ----------------------------------------------------------------
            w1_rjx <= signed((not w0_rnd(15)) & w0_rnd(14 downto 8));
            w1_rjy <= signed((not w0_rnd(3))  & w0_rnd(2 downto 0) & w0_rnd(15 downto 12));
            w1_dl   <= signed(resize(w0_drv, 12)) - signed(resize(s_cen, 12));

            w1_rrlo <= w0_rnd(7 downto 4) + w0_rnd(15 downto 12);

            v_du := signed(resize(w0_u, 11)) - 512;
            v_dv := signed(resize(w0_v, 11)) - 512;
            v_au := unsigned(abs(v_du));
            v_av := unsigned(abs(v_dv));
            if v_du >= 0 and v_dv >= 0 then
                if v_au > v_av then v_oct := 0; else v_oct := 4; end if;
            elsif v_du < 0 and v_dv >= 0 then
                if v_au > v_av then v_oct := 8; else v_oct := 4; end if;
            elsif v_du < 0 and v_dv < 0 then
                if v_au > v_av then v_oct := 8; else v_oct := 12; end if;
            else
                if v_au > v_av then v_oct := 0; else v_oct := 12; end if;
            end if;
            w1_oct  <= to_unsigned(v_oct, 4);

            w1_stb <= w0_stb;  w1_par <= w0_par;  w1_idx <= w0_idx;
            w1_u <= w0_u;  w1_v <= w0_v;

            ----------------------------------------------------------------
            -- W2: face (offset vs +/-sp, +/-2sp), static tilt, jitter,
            -- luma-size sum, hue index.
            ----------------------------------------------------------------
            v_lvl := 0;
            if w1_dl >= s_nsp2 then v_lvl := v_lvl + 1; end if;
            if w1_dl >= s_nsp  then v_lvl := v_lvl + 1; end if;
            if w1_dl >= 0      then v_lvl := v_lvl + 1; end if;
            if w1_dl >= s_sp   then v_lvl := v_lvl + 1; end if;
            if w1_dl >= s_sp2  then v_lvl := v_lvl + 1; end if;
            if s_pol = '1' then v_face := 1 + v_lvl; else v_face := 6 - v_lvl; end if;
            w2_face <= to_unsigned(v_face, 3);

            w2_rjy <= w1_rjy;
            v_ls   := shift_right(resize(w1_dl, 14) + shift_left(resize(w1_dl, 14), 1), 4);  -- dl*24/128
            w2_rs  <= signed(resize(s_r0, 12)) + resize(v_ls, 12);

            if s_idx = 6 then
                v_hi := to_integer((w1_rrlo + s_hueoff) and "1111");
            else
                v_hi := to_integer((w1_oct + s_hueoff) and "1111");
            end if;
            w2_hidx <= to_unsigned(v_hi, 4);

            w2_stb <= w1_stb;  w2_par <= w1_par;  w2_idx <= w1_idx;
            w2_u <= w1_u;  w2_v <= w1_v;

            ----------------------------------------------------------------
            -- W3: angle (256 = square), radius, face, body colour.
            ----------------------------------------------------------------
            w3_a <= to_signed(256, 10) + resize(shift_right(sm_p, 6), 10);   -- tilt: up to +/-252 (~45 deg)

            -- (radius: Luma Size or the per-frame base)
            if s_lsize = '1' then
                if    w2_rs < 6 then v_r := to_signed(6, 10);
                elsif w2_rs > signed(resize(s_rmax, 12)) then v_r := signed(resize(s_rmax, 10));
                else  v_r := w2_rs(9 downto 0); end if;
            else
                v_r := signed(resize(s_r0, 10));
            end if;
            w3_r    <= unsigned(v_r(7 downto 0));
            w3_face <= w2_face;

            if s_idx = 6 or s_idx = 7 then
                v_bu := to_unsigned(HUE_U(to_integer(w2_hidx)), 10);
                v_bv := to_unsigned(HUE_V(to_integer(w2_hidx)), 10);
            else
                v_bu := s_inku;  v_bv := s_inkv;
            end if;
            if s_inkvid = '1' then
                v_bu := w2_u;  v_bv := w2_v;
            end if;
            w3_bu <= v_bu;  w3_bv <= v_bv;

            w3_stb <= w2_stb;  w3_par <= w2_par;  w3_idx <= w2_idx;

            ----------------------------------------------------------------
            -- W4: t = a - 256 (within a quarter turn), rot90 = a(9), and
            -- R*(sec-1) as two selected shifts of R, bucketed on |t|/32
            -- (sec-1: .002 .017 .048 .092 .149 .214 .288 .367; <=2.5% off).
            ----------------------------------------------------------------
            w4_t   <= signed((not w3_a(8)) & std_logic_vector(w3_a(7 downto 0)));
            w4_rot <= w3_a(9);
            v_sb := unsigned(w3_a(7 downto 5));
            if w3_a(8) = '0' then v_sb := not v_sb; end if;          -- ~|t| bucket
            case to_integer(v_sb) is
                when 0      => w4_s1 <= (others => '0');           w4_s2 <= (others => '0');
                when 1      => w4_s1 <= shift_right(w3_r, 6);      w4_s2 <= (others => '0');
                when 2      => w4_s1 <= shift_right(w3_r, 5);      w4_s2 <= shift_right(w3_r, 6);
                when 3      => w4_s1 <= shift_right(w3_r, 4);      w4_s2 <= shift_right(w3_r, 5);
                when 4      => w4_s1 <= shift_right(w3_r, 3);      w4_s2 <= shift_right(w3_r, 5);
                when 5      => w4_s1 <= shift_right(w3_r, 3);      w4_s2 <= shift_right(w3_r, 4);
                when 6      => w4_s1 <= shift_right(w3_r, 2);      w4_s2 <= shift_right(w3_r, 5);
                when others => w4_s1 <= shift_right(w3_r, 2);      w4_s2 <= shift_right(w3_r, 3);
            end case;
            w4_r <= w3_r;  w4_face <= w3_face;
            w4_jx <= resize(shift_right(sm_p, 8), 8);          -- jitter x
            w4_bv <= w3_bv;
            w4_stb <= w3_stb;  w4_par <= w3_par;  w4_idx <= w3_idx;

            ----------------------------------------------------------------
            -- W5: R' = R * sec, jitter y.  W6: body U through the chroma gain
            -- (unity 8 except Ink=Video, where K3 is saturation), saturated
            -- to +/-511.  W7: body V likewise, pack.
            ----------------------------------------------------------------
            w5_rp <= w4_r + w4_s1 + w4_s2;
            w5_jy <= resize(shift_right(sm_p, 8), 8);          -- jitter y
            w5_face <= w4_face;  w5_rot <= w4_rot;  w5_t <= w4_t;  w5_jx <= w4_jx;
            w5_stb <= w4_stb;  w5_par <= w4_par;  w5_idx <= w4_idx;

            v_cu := shift_right(sm_p, 3);
            if    v_cu >  511 then w6_cu <= to_signed( 511, 10);
            elsif v_cu < -511 then w6_cu <= to_signed(-511, 10);
            else  w6_cu <= v_cu(9 downto 0); end if;
            w6_rp <= w5_rp;  w6_face <= w5_face;  w6_rot <= w5_rot;  w6_t <= w5_t;
            w6_jx <= w5_jx;  w6_jy <= w5_jy;
            w6_stb <= w5_stb;  w6_par <= w5_par;  w6_idx <= w5_idx;

            v_cu := shift_right(sm_p, 3);
            if    v_cu >  511 then v_cu := to_signed( 511, 19);
            elsif v_cu < -511 then v_cu := to_signed(-511, 19); end if;
            w7_word <= std_logic_vector(w6_face)
                     & w6_rot
                     & std_logic_vector(w6_t)
                     & std_logic_vector(w6_jx)
                     & std_logic_vector(w6_jy)
                     & std_logic_vector(w6_rp)
                     & (not w6_cu(9)) & std_logic_vector(w6_cu(8 downto 0))   -- +512
                     & (not v_cu(9))  & std_logic_vector(v_cu(8 downto 0))
                     & "0000000";
            w7_stb <= w6_stb;  w7_par <= w6_par;  w7_idx <= w6_idx;
        end if;
    end process p_sample;

    --------------------------------------------------------------------------
    -- Cell BRAMs.
    --------------------------------------------------------------------------
    p_buf0 : process(clk)
    begin
        if rising_edge(clk) then
            if w7_stb = '1' and w7_par = '0' then
                buf0(to_integer(w7_idx)) <= w7_word;
            end if;
            rd0 <= buf0(to_integer(xidx));
        end if;
    end process p_buf0;

    p_buf1 : process(clk)
    begin
        if rising_edge(clk) then
            if w7_stb = '1' and w7_par = '1' then
                buf1(to_integer(w7_idx)) <= w7_word;
            end if;
            rd1 <= buf1(to_integer(xidx));
        end if;
    end process p_buf1;

    --------------------------------------------------------------------------
    -- Frame FSM (vblank).  One small op per state; products on one shared
    -- multiplier, valid 2 states after their operands are set.  Serrated
    -- vsync re-runs it: every state is idempotent except the auto-level step,
    -- which is guarded by s_pend (set once per frame at the anchor).
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_d   : integer range 0 to 255;
        variable v_gl  : integer range 0 to 1023;
        variable v_r0  : integer range -256 to 255;
        variable v_sp  : unsigned(7 downto 0);
        variable v_ach : boolean;
        variable v_lo  : signed(10 downto 0);
        variable v_k   : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            mp <= ma * mb;

            -- auto-level census: samples above / below the current centre
            -- (ties count neither way, so a flat field settles instead of
            -- bouncing the centre +/-2 every frame and flickering every die)
            if w1_stb = '1' then
                lv_tot <= lv_tot + 1;
                if w1_dl > 0 then lv_diff <= lv_diff + 1;
                elsif w1_dl < 0 then lv_diff <= lv_diff - 1; end if;
            end if;

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                rk1 <=unsigned(registers_in(0)(9 downto 0));
                lk2 <=unsigned(registers_in(1)(9 downto 0));
                lk3 <=unsigned(registers_in(2)(9 downto 0));
                lk4 <=unsigned(registers_in(3)(9 downto 0));
                lk5 <=unsigned(registers_in(4)(9 downto 0));
                lk6 <=unsigned(registers_in(5)(9 downto 0));
                rp12<=unsigned(registers_in(7)(9 downto 0));
                s_src    <= registers_in(6)(0);
                s_lsize  <= registers_in(6)(1);
                s_tabvid <= registers_in(6)(2);
                s_inkvid <= registers_in(6)(3);
                s_bypass <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"FF" then
                fsm_t <= fsm_t + 1;
                case to_integer(fsm_t) is
                    -- deadband Zoom + Tumble (pot noise at a >>3 step boundary
                    -- would otherwise flip the whole grid / every die)
                    when 0 =>
                        d1  <= signed(resize(rk1, 11))  - signed(resize(lk1, 11));
                        d12 <= signed(resize(rp12, 11)) - signed(resize(lp12, 11));
                    when 1 =>
                        if d1  > 4 or d1  < -4 then lk1  <= rk1;  end if;
                        if d12 > 4 or d12 < -4 then lp12 <= rp12; end if;
                    when 2 =>
                        v_d := 18 + to_integer(lk1(9 downto 3));
                        if v_d > 150 then v_d := 150; end if;
                        s_d   <= to_unsigned(v_d, 8);
                        s_idx <= lk2(9 downto 7);
                        s_pol <= SCH_POL(to_integer(lk2(9 downto 7)));
                        s_hueoff <= lk3(9 downto 6);
                        s_rotsc  <= '0' & lp12(9 downto 3);
                        s_shad   <= '0' & lp12(9 downto 3);
                        -- Contrast: up = narrower face spacing = punchier
                        -- (top end collapses to binary 1/6 dice).
                        v_sp := to_unsigned(4, 8) + ('0' & (not lk6(9 downto 3)));
                        s_sp <= signed(resize(v_sp, 12));
                        -- Gloss: matte chalk (flat) -> lacquer (deep dome + a white
                        -- glint along the lit edges that grows with the knob).
                        v_gl := to_integer(lk5(9 downto 0));
                        s_span <= to_unsigned(24 + (v_gl / 4), 9);        -- 24..279
                        if lk5 >= 256 then s_glossy <= '1'; else s_glossy <= '0'; end if;
                        s_spth <= to_signed(300 - (v_gl / 4), 12);        -- (300..44; used >= 256)
                    when 3 =>
                        s_dm1 <= s_d - 1;
                        s_dh  <= '0' & s_d(7 downto 1);
                        s_dlim <= (s_d & '0') - s_xs;                  -- 2D - x step
                        ma <= signed(resize('0' & s_d(7 downto 2), 10));   -- Dh/2 = D/4
                        mb <= signed(resize(lp12, 11));
                        s_sp2 <= shift_left(s_sp, 1);
                        s_nsp <= -s_sp;
                    when 4 =>
                        -- precomputed denoise window start, half units:
                        -- Dh - 7 steps, at least the cell start (keeps the
                        -- arithmetic out of p_position)
                        v_lo := signed(resize(s_dh & '0', 11))
                              - signed(resize(s_xs & "000", 11)) + signed(resize(s_xs, 11));
                        if v_lo < 0 then
                            s_dhlo <= (others => '0');
                        else
                            s_dhlo <= unsigned(v_lo(8 downto 0));
                        end if;
                        -- sample-line threshold D - ceil(y step)
                        v_k := s_ys(11 downto 8);
                        if s_ys(7 downto 0) /= 0 then v_k := v_k + 1; end if;
                        s_dmk <= s_d - v_k;
                        s_nsp2 <= -s_sp2;
                    when 5 =>
                        v_r0 := to_integer(s_dh) - 2 - to_integer(unsigned(mp(17 downto 10)));
                        if v_r0 < 6 then v_r0 := 6; end if;
                        s_r0 <= to_unsigned(v_r0, 8);
                    when 6 =>
                        s_spanh <= '0' & s_span(8 downto 1);
                        -- bevel normalised to die size: shade ~ +/-250 at the edge
                        if    s_r0 < 16 then s_bsh <= "00";
                        elsif s_r0 < 32 then s_bsh <= "01";
                        elsif s_r0 < 64 then s_bsh <= "10";
                        else                 s_bsh <= "11"; end if;
                        if s_r0 < 8 then s_bw <= to_unsigned(2, 8);
                        else s_bw <= "00" & s_r0(7 downto 2); end if;
                        s_rmax  <= s_dh - 1;
                        ma <= signed(resize(s_dh - s_r0, 10));
                        mb <= signed(resize(lp12, 11));
                    when 8 =>
                        s_jitsc <= resize(unsigned(mp(17 downto 11)), 8);
                    when 9 =>                                   -- tilt scale (P12/8)^2
                        ma <= signed(resize(s_rotsc, 10));
                        mb <= signed(resize(s_rotsc, 11));

                    -- K3 hue: rotate felt + painted ink by (K3 - centre);
                    -- achromatic inks get pips tinted, saturation |K3 - centre|.
                    when 10 =>
                        if s_inkvid = '1' then                  -- K3 = video-ink saturation
                            s_sat <= lk3(9 downto 6);           -- 0..15, 8 (50%) = unity
                            s_hix <= (others => '0');           -- felt unrotated
                            tdl   <= (others => '0');           -- no pip tint
                        else
                            s_sat <= to_unsigned(8, 4);
                            s_hix <= lk3(9 downto 4) + 32;      -- centre (512) = 0 deg
                            tdl   <= signed(resize(lk3, 11)) - 512;
                        end if;
                        idu   <= to_signed(SCH_U(to_integer(s_idx)) - 512, 10);
                        idv   <= to_signed(SCH_V(to_integer(s_idx)) - 512, 10);
                        s_by  <= to_unsigned(SCH_Y(to_integer(s_idx)), 10);
                    when 11 =>
                        s_q <= resize(unsigned(mp(13 downto 7)), 8);   -- 0..126
                        hc <= to_signed(COS64(to_integer(s_hix)), 11);
                        s_hix <= s_hix - 16;                    -- cos(a - 90) = sin(a)
                        if abs(tdl) > 510 then tsat <= to_unsigned(255, 8);
                        else tsat <= resize(unsigned(shift_right(abs(tdl), 1)), 8); end if;
                    when 12 =>
                        hs <= to_signed(COS64(to_integer(s_hix)), 11);
                    when 13 =>
                        hsn <= -hs;
                        ma <= idu;  mb <= hc;
                    when 14 =>
                        ma <= idv;  mb <= hsn;
                    when 15 =>
                        acc <= mp;
                        ma <= idu;  mb <= hs;
                    when 16 =>
                        acc <= acc + mp;
                        ma <= idv;  mb <= hc;
                    when 17 =>
                        s_inku <= f_c512(shift_right(acc, 8));
                        acc <= mp;
                        ma <= to_signed(FELT_U - 512, 10);  mb <= hc;
                    when 18 =>
                        acc <= acc + mp;
                        ma <= to_signed(FELT_V - 512, 10);  mb <= hsn;
                    when 19 =>
                        s_inkv <= f_c512(shift_right(acc, 8));
                        acc <= mp;
                        ma <= to_signed(FELT_U - 512, 10);  mb <= hs;
                    when 20 =>
                        acc <= acc + mp;
                        ma <= to_signed(FELT_V - 512, 10);  mb <= hc;
                    when 21 =>
                        s_feltu <= f_c512(shift_right(acc, 8));
                        acc <= mp;
                        ma <= signed(resize(tsat, 10));  mb <= hc;
                    when 22 =>
                        acc <= acc + mp;
                        ma <= signed(resize(tsat, 10));  mb <= hs;
                    when 23 =>
                        s_feltv <= f_c512(shift_right(acc, 8));
                        tint_u  <= resize(shift_right(mp, 8), 11);
                    when 24 =>
                        tint_v  <= resize(shift_right(mp, 8), 11);
                    when 25 =>
                        v_ach := s_idx = 0 or s_idx = 2 or s_idx = 3;
                        if v_ach then
                            s_pipu <= f_c512(tint_u);
                            s_pipv <= f_c512(tint_v);
                            if s_pol = '1' then
                                s_py <= to_unsigned(760, 10) - ("00" & tsat);
                            else
                                s_py <= to_unsigned(64, 10) + ("00" & tsat);
                            end if;
                        else
                            s_pipu <= C_MID;  s_pipv <= C_MID;
                            if s_pol = '1' then s_py <= to_unsigned(760, 10);
                            else s_py <= to_unsigned(64, 10); end if;
                        end if;

                    -- Auto-level: nudge the centre toward the frame's median
                    -- (12.5% deadband; 4x step when the imbalance exceeds 25%).
                    when 26 =>
                        lv_ad  <= unsigned(abs(lv_diff));
                        lv_neg <= lv_diff(14);
                    when 27 =>
                        if lv_ad > ('0' & shift_right(lv_tot, 3)) then lv_mv  <= '1'; else lv_mv  <= '0'; end if;
                        if lv_ad > ('0' & shift_right(lv_tot, 2)) then lv_big <= '1'; else lv_big <= '0'; end if;
                    when 28 =>
                        if lv_mv = '0' then lv_st <= (others => '0');
                        elsif lv_big = '1' then
                            if lv_neg = '1' then lv_st <= to_signed(-8, 12); else lv_st <= to_signed(8, 12); end if;
                        else
                            if lv_neg = '1' then lv_st <= to_signed(-2, 12); else lv_st <= to_signed(2, 12); end if;
                        end if;
                    when 29 =>
                        lv_c <= signed(resize(s_cen, 12)) + lv_st;
                    when 30 =>
                        if s_censrc = '1' then
                            if    lv_c < 16  then lv_c <= to_signed(16, 12);
                            elsif lv_c > 600 then lv_c <= to_signed(600, 12); end if;
                        else
                            if    lv_c < 80  then lv_c <= to_signed(80, 12);
                            elsif lv_c > 900 then lv_c <= to_signed(900, 12); end if;
                        end if;
                    when 31 =>
                        if s_pend = '1' then
                            if s_censrc = '1' then cen_c <= unsigned(lv_c(10 downto 0));
                            else                   cen_l <= unsigned(lv_c(10 downto 0)); end if;
                            lv_diff <= (others => '0');
                            lv_tot  <= (others => '0');
                            s_pend  <= '0';
                        end if;
                    when 32 =>
                        if s_src = '1' then s_cen <= cen_c; else s_cen <= cen_l; end if;
                        s_censrc <= s_src;
                    when others => null;
                end case;
            end if;

            -- once per frame (frame_act-guarded anchor, immune to serrated vsync)
            if r_anchor = '1' then
                s_pend  <= '1';
            end if;

            lx0 <= resize(shift_right(lut_cos, 2), 8);  -- cos/4  (+/-127)
            ly0 <= resize(shift_right(lut_sin, 2), 8);  -- sin/4
            lx  <= lx0;                                 -- 2nd register: the 1st is absorbed
            ly  <= ly0;                                 -- into the ROM's EBR
        end if;
    end process p_frame;

    --------------------------------------------------------------------------
    -- Pixel pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_word : t_word;
        variable v_u, v_v : signed(9 downto 0);
        variable v_absu, v_absv : unsigned(9 downto 0);
        variable v_r, v_rin, v_p, v_ph : unsigned(7 downto 0);
        variable v_au, v_av : unsigned(8 downto 0);
        variable v_gu, v_gv : integer range -1 to 1;
        variable v_gup, v_gvp : signed(9 downto 0);
        variable v_maxa : unsigned(8 downto 0);
        variable v_shade, v_shraw : signed(11 downto 0);
        variable v_yo, v_uo, v_vo : signed(11 downto 0);
        variable v_mask : std_logic_vector(8 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_t1, v_t2, v_pipdot : signed(9 downto 0);
        variable v_pipb : signed(11 downto 0);
        variable v_ru9, v_rv9 : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            ----------------------------------------------------------------
            -- c1
            ----------------------------------------------------------------
            c1_dx0 <= signed(resize(xla(8 downto 1), 10)) - signed(resize(s_dh, 10));
            c1_par <= celly_par;

            ----------------------------------------------------------------
            -- c2: select word, apply jitter.
            ----------------------------------------------------------------
            if c1_par = '1' then v_word := rd1; else v_word := rd0; end if;
            c2_word <= v_word;
            c2_dxj  <= c1_dx0   - resize(signed(v_word(50 downto 43)), 10);   -- - jx
            c2_dyj  <= dyc_line - resize(signed(v_word(42 downto 35)), 10);   -- - jy

            ----------------------------------------------------------------
            -- c3: rotation products (t at 59..51, |t| <= 1.0 at x256).
            ----------------------------------------------------------------
            c3_rsx  <= resize(c2_dxj, 8) * signed(c2_word(59 downto 51));
            c3_rsy  <= resize(c2_dyj, 8) * signed(c2_word(59 downto 51));
            c3_dxj  <= c2_dxj;
            c3_dyj  <= c2_dyj;
            c3_word <= c2_word;

            ----------------------------------------------------------------
            -- c4: u,v = sec-scaled rotation; magnitudes; derived radii.
            ----------------------------------------------------------------
            v_u := c3_dxj + resize(shift_right(c3_rsy, 8), 10);
            v_v := c3_dyj - resize(shift_right(c3_rsx, 8), 10);
            c4_u <= v_u;  c4_v <= v_v;
            v_absu := unsigned(abs(v_u));
            v_absv := unsigned(abs(v_v));
            c4_au <= v_absu(8 downto 0);
            c4_av <= v_absv(8 downto 0);
            -- bevel works in SCREEN space (light stays put while dice spin);
            -- clamp to +/-127 for the 8x8 mult (die interior is well inside).
            if    c3_dxj >  127 then c4_ub <= to_signed( 127, 8);
            elsif c3_dxj < -128 then c4_ub <= to_signed(-128, 8);
            else                     c4_ub <= resize(c3_dxj, 8); end if;
            if    c3_dyj >  127 then c4_vb <= to_signed( 127, 8);
            elsif c3_dyj < -128 then c4_vb <= to_signed(-128, 8);
            else                     c4_vb <= resize(c3_dyj, 8); end if;
            v_r := unsigned(c3_word(34 downto 27));
            c4_rin <= v_r - ("00" & v_r(7 downto 2));                              -- 0.75 R
            c4_pr  <= ("000" & v_r(7 downto 3)) + ("0000" & v_r(7 downto 4));      -- ~0.19 R
            c4_word <= c3_word;

            ----------------------------------------------------------------
            -- c5
            ----------------------------------------------------------------
            v_word := c4_word;
            v_r    := unsigned(v_word(34 downto 27));
            v_rin  := c4_rin;
            v_p    := '0'  & v_r(7 downto 1);
            v_ph   := "00" & v_r(7 downto 2);
            v_au   := c4_au;  v_av := c4_av;

            if v_au <= ('0' & v_r) and v_av <= ('0' & v_r) then
                c5_edgein <= '1'; else c5_edgein <= '0'; end if;
            if v_au > ('0' & v_rin) and v_av > ('0' & v_rin) then
                c5_incorner <= '1'; else c5_incorner <= '0'; end if;
            if v_au > ('0' & v_rin) then c5_cu <= v_au - ('0' & v_rin); else c5_cu <= (others=>'0'); end if;
            if v_av > ('0' & v_rin) then c5_cv <= v_av - ('0' & v_rin); else c5_cv <= (others=>'0'); end if;

            c5_bx <= c4_ub * lx;
            c5_by <= c4_vb * ly;

            if    c4_u < -signed('0' & v_ph) then v_gu := -1;
            elsif c4_u >  signed('0' & v_ph) then v_gu :=  1;
            else  v_gu := 0; end if;
            if    c4_v < -signed('0' & v_ph) then v_gv := -1;
            elsif c4_v >  signed('0' & v_ph) then v_gv :=  1;
            else  v_gv := 0; end if;
            case v_gu is
                when -1     => v_gup := resize(-signed('0' & v_p), 10);
                when  1     => v_gup := resize( signed('0' & v_p), 10);
                when others => v_gup := (others => '0');
            end case;
            case v_gv is
                when -1     => v_gvp := resize(-signed('0' & v_p), 10);
                when  1     => v_gvp := resize( signed('0' & v_p), 10);
                when others => v_gvp := (others => '0');
            end case;
            c5_ru <= resize(c4_u - v_gup, 9);
            c5_rv <= resize(c4_v - v_gvp, 9);
            c5_node <= to_unsigned((v_gv + 1) * 3 + (v_gu + 1), 4);
            c5_face <= unsigned(v_word(63 downto 61));
            c5_rot  <= v_word(60);
            c5_pr   <= c4_pr;

            if v_au > v_av then v_maxa := v_au; else v_maxa := v_av; end if;
            c5_maxa <= v_maxa;
            -- edge band flag only (bevel is whole-face; the flag just gates the
            -- corner specular) -> a compare, no subtract.
            if v_maxa > (v_r - s_bw) then c5_inband <= '1'; else c5_inband <= '0'; end if;
            c5_su <= c4_u(9);
            c5_sv <= c4_v(9);
            c5_word <= v_word;

            ----------------------------------------------------------------
            -- c6
            ----------------------------------------------------------------
            -- octagonal distance = max(a,b) + min(a,b)/2 (adds/shifts only), a
            -- near-circular metric -> round pips and rounded corners with no
            -- distance-squared multiply.
            v_ru9 := unsigned(abs(c5_ru));  v_rv9 := unsigned(abs(c5_rv));
            if v_ru9 >= v_rv9 then c6_pdoct <= resize(v_ru9 + ('0' & v_rv9(8 downto 1)), 10);
            else                   c6_pdoct <= resize(v_rv9 + ('0' & v_ru9(8 downto 1)), 10); end if;
            if c5_cu >= c5_cv then c6_cdoct <= resize(c5_cu + ('0' & c5_cv(8 downto 1)), 10);
            else                   c6_cdoct <= resize(c5_cv + ('0' & c5_cu(8 downto 1)), 10); end if;
            c6_bev <= resize(c5_bx, 17) + resize(c5_by, 17);
            v_mask := facemask(c5_face, c5_rot);
            c6_active   <= v_mask(to_integer(c5_node));
            c6_incorner <= c5_incorner;
            c6_edgein   <= c5_edgein;
            c6_inband   <= c5_inband;
            c6_su <= c5_su;  c6_sv <= c5_sv;  c6_maxa <= c5_maxa;
            c6_ru <= c5_ru;  c6_rv <= c5_rv;
            c6_pr <= c5_pr;
            c6_word <= c5_word;

            ----------------------------------------------------------------
            -- c7
            ----------------------------------------------------------------
            v_word := c6_word;
            v_r    := unsigned(v_word(34 downto 27));
            if c6_edgein = '1' and (c6_incorner = '0' or c6_cdoct <= ("00" & v_r(7 downto 2))) then
                c7_indie <= '1'; else c7_indie <= '0'; end if;
            if c6_active = '1' and c6_pdoct <= c6_pr then
                c7_inpip <= '1'; else c7_inpip <= '0'; end if;
            -- drop shadow on the side AWAY from the light (quadrant of -L)
            if c6_su = not lx(7) and c6_sv = not ly(7)
               and signed('0' & c6_maxa) > signed('0' & v_r)
               and signed('0' & c6_maxa) <= (signed('0' & v_r) + signed('0' & s_shad(7 downto 1))) then
                c7_shadow <= '1'; else c7_shadow <= '0'; end if;
            c7_bev <= c6_bev;
            c7_inband <= c6_inband;
            c7_incorner <= c6_incorner;
            c7_pdoct <= c6_pdoct;
            c7_pr    <= c6_pr;
            c7_ru <= c6_ru;  c7_rv <= c6_rv;
            c7_word <= v_word;

            ----------------------------------------------------------------
            -- c7b: dimple direction, raw shade, glint, colour unpack.
            ----------------------------------------------------------------
            v_word := c7_word;
            c7b_bu <= unsigned(v_word(26 downto 17));
            c7b_bv <= unsigned(v_word(16 downto 7));

            case s_bsh is
                when "00"   => c7b_shraw <= resize(shift_right(c7_bev, 3), 12);
                when "01"   => c7b_shraw <= resize(shift_right(c7_bev, 4), 12);
                when "10"   => c7b_shraw <= resize(shift_right(c7_bev, 5), 12);
                when others => c7b_shraw <= resize(shift_right(c7_bev, 6), 12);
            end case;

            if lx(7) = '0' then v_t1 := resize(c7_ru, 10); else v_t1 := resize(-c7_ru, 10); end if;
            if ly(7) = '0' then v_t2 := resize(c7_rv, 10); else v_t2 := resize(-c7_rv, 10); end if;
            v_pipdot := resize(v_t1 + v_t2, 10);
            c7b_pipdot <= v_pipdot;
            if c7_pdoct > (c7_pr - ('0' & c7_pr(7 downto 2))) and v_pipdot < -2 then
                c7b_glint <= '1'; else c7b_glint <= '0'; end if;

            c7b_inpip <= c7_inpip;  c7b_indie <= c7_indie;  c7b_shadow <= c7_shadow;
            c7b_incorner <= c7_incorner;  c7b_inband <= c7_inband;

            ----------------------------------------------------------------
            -- c8a: shade clamp / pip value / specular flag (shallow).
            ----------------------------------------------------------------
            v_shraw := c7b_shraw;
            v_shade := v_shraw;
            if v_shraw >  signed('0' & s_spanh) then v_shade := resize( signed('0' & s_spanh), 12); end if;
            if v_shraw < -signed('0' & s_span)  then v_shade := resize(-signed('0' & s_span), 12);  end if;
            c8a_shade <= v_shade;

            v_pipb := signed(resize(s_py, 12)) - resize(shift_left(c7b_pipdot, 3), 12);
            if c7b_glint = '1' then v_pipb := v_pipb + 260; end if;
            c8a_pipb <= v_pipb;

            -- glint: lit-facing edge band, area grows as Gloss lowers the threshold
            if s_glossy = '1' and c7b_inband = '1' and v_shraw >= s_spth then
                c8a_spec <= '1'; else c8a_spec <= '0'; end if;

            c8a_bu <= c7b_bu;  c8a_bv <= c7b_bv;
            c8a_inpip <= c7b_inpip;  c8a_indie <= c7b_indie;  c8a_shadow <= c7b_shadow;

            ----------------------------------------------------------------
            -- c8b: assemble (luma clamp is in c9).
            ----------------------------------------------------------------
            if c8a_inpip = '1' then
                v_yo := c8a_pipb;
                v_uo := signed(resize(s_pipu, 12));
                v_vo := signed(resize(s_pipv, 12));
                c8_class <= "11";
            elsif c8a_indie = '1' then
                if c8a_spec = '1' then                      -- white glint, half-desaturated
                    v_yo := to_signed(C_YMAX, 12);
                    v_uo := signed(resize(c8a_bu(9 downto 1), 12)) + 256;
                    v_vo := signed(resize(c8a_bv(9 downto 1), 12)) + 256;
                else
                    v_yo := signed(resize(s_by, 12)) + c8a_shade;
                    v_uo := signed(resize(c8a_bu, 12));
                    v_vo := signed(resize(c8a_bv, 12));
                end if;
                c8_class <= "10";
            else
                v_yo := to_signed(64, 12);
                v_uo := to_signed(512, 12);
                v_vo := to_signed(512, 12);
                if c8a_shadow = '1' then c8_class <= "01"; else c8_class <= "00"; end if;
            end if;
            c8_y <= v_yo;                                    -- clamped in c9
            c8_u <= unsigned(v_uo(9 downto 0));
            c8_v <= unsigned(v_vo(9 downto 0));

            ----------------------------------------------------------------
            -- c9: compose over table; blanking gate (neutral outside avid)
            -- and the first active line shown as table (row 0's buffer is
            -- being written on that line -> EBR read-during-write).
            ----------------------------------------------------------------
            -- Video table from a SHORT tap (pipe(4), 6 px ahead of the dice):
            -- raw input video carried through the full ~12-deep delay went
            -- green + noisy on HD HDMI (here and in Cascade v1.4); the 6 px
            -- shift is invisible under the dice.  Black where the tap has
            -- already left active video (right edge).
            if s_tabvid = '1' then
                if pipe(4).avid = '1' then
                    v_bgy := resize(unsigned(pipe(4).y(9 downto 1)), 10) + 40;
                    v_bgu := unsigned(pipe(4).u);
                    v_bgv := unsigned(pipe(4).v);
                else
                    v_bgy := to_unsigned(64, 10);  v_bgu := C_MID;  v_bgv := C_MID;
                end if;
            else
                v_bgy := to_unsigned(FELT_Y, 10);
                v_bgu := s_feltu;
                v_bgv := s_feltv;
            end if;

            if pipe(10).avid = '0' then
                c9_y <= to_unsigned(64, 10);  c9_u <= C_MID;  c9_v <= C_MID;
            elsif line0 = '1' then
                c9_y <= v_bgy;  c9_u <= v_bgu;  c9_v <= v_bgv;
            else
                case to_integer(c8_class) is
                    when 2 | 3 =>
                        -- clamp to [black 64, C_YMAX]
                        if    c8_y < 64     then c9_y <= to_unsigned(64, 10);
                        elsif c8_y > C_YMAX then c9_y <= to_unsigned(C_YMAX, 10);
                        else                     c9_y <= unsigned(c8_y(9 downto 0)); end if;
                        c9_u <= c8_u;  c9_v <= c8_v;
                    when 1 =>
                        c9_y <= ('0' & v_bgy(9 downto 1)) + 32;   -- halve above black
                        c9_u <= v_bgu;  c9_v <= v_bgv;
                    when others =>
                        c9_y <= v_bgy;  c9_u <= v_bgu;  c9_v <= v_bgv;
                end case;
            end if;

            ----------------------------------------------------------------
            -- c10: processed output (sync latency-aligned to c9 content).
            ----------------------------------------------------------------
            s_io.y <= std_logic_vector(c9_y);
            s_io.u <= std_logic_vector(c9_u);
            s_io.v <= std_logic_vector(c9_v);
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;

            -- S11: 1-cycle raw forward, exactly SDK passthru (data_out <= data_in)
            s_bpass <= data_in;

        end if;
    end process p_pipe;

    -- S11 bypass = SDK passthru (raw input + its own syncs, 1 cycle); the
    -- 12-deep raw-video path read green + noisy on HD HDMI hardware.
    s_io_align_in <= s_bpass when s_bypass = '1' else s_io;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (13 -> 16).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 2 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(2);

end architecture rollthebones;
