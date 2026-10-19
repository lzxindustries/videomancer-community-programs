-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ziffern.vhd  (sub-pixel multi-plane perspective, P12 zoom position)
--
-- A Kraftwerk-inspired green "computer screen" wall of pixelated digits 1-9.
-- The digits change at random, staggered times; the field zooms toward the viewer
-- in true perspective (centred on screen), driven by P12 as a ZOOM POSITION.
--
-- Smooth zoom: each plane is a fractional PHASE ACCUMULATOR (DDA), NOT an integer
-- cell grid.  Per pixel a fixed-point phase advances by step = (1<<FRAC)/cellsize;
-- the cell index is the integer part, the glyph column the top fraction bits.
-- Centring is phase = (x - centre)*step (continuous), so the grid GLIDES as the
-- cell size changes instead of snapping on a per-pixel modulo.  No per-pixel
-- multiply/divide (step + centre offset are per-frame; per pixel is just an add).
--
-- Glide filter: the geometry is NOT driven by the raw P12 reading.  One knob tick
-- moves an edge cell ~3 px (and pot ADC noise trembles the whole field), so the
-- zoom position is a Q10.6 low-pass that eases toward the knob target each frame
-- (pos += (target-pos) >> C_GLIDE_SH).  Knob ticks become an exponential glide,
-- ADC noise is crushed to sub-tick level, and the 64x finer position feeds a
-- widened step lerp (zprod >> 16).  The field settles ~1/8 s after P12 stops.
--
-- Depth: NP=4 planes composited front-to-back, each at its own cell size that
-- scales from dense/far to large/near at a different speed (front fastest); the
-- numbers are SPLIT across the planes (a balanced per-cell hash assigns each cell
-- to one plane) so each number lives at one depth.  P12=100% = all far/dense/small
-- (one plane), pull toward 0% = enlarge in perspective at different speeds,
-- 0% = all co-planar at the front.  Movement only when P12 moves.
--
-- Background is black or a dark tint; S9 keys incoming video into the black areas.
--
-- Controls: K1 Color, K2 Change Rate, K3 Number Set, K4 Size (smooth font
--           scale; brightness is uniform across planes), K5 Spacing (gap
--           between numbers, font size unchanged: pitch step = font step *
--           RT via the vblank multiply chain), K6 Glow = ENERGY: digits idle
--           at 50% brightness, K6 drives them to 100% AND draws a 3-ring
--           screen-space halo at 75/50/25% of the digit brightness (window
--           on the composited lit mask -- independent of digit size),
--           S7 Luma Mod, S8 Background, S9 Video, S10 Font, S11 Invert,
--           P12 Zoom.  K2 at 0% freezes the digits (no Animate switch).
--
-- Luma Mod (S7): the planes become a SIZE LADDER instead of a zoom envelope
-- (back rung pinned at the far wall; others at 1/4, 1/2, full of the
-- P12-scaled zoom-in).  Each plane samples the incoming video's luma once
-- per cell (top-left pixel, held in a 256x2 buffer) and draws its cells only
-- where the 2-bit level (vy(9:8)) matches its rung -- dark areas stay a far
-- dense wall, bright areas pop individual numbers forward proportionally.
--
-- Resolution scaling (v1.0.2): the planes' phase accumulators run in virtual
-- 1080-line units, stepped per pixel and per FIELD line by the per-mode steps
-- from the video timing ID (1080p 1.0/1.0, 1080i 1.0/2.0, 720p 1.5/1.5, SD
-- 2.0/~4.44 at 480i -- SD is a 1440-unit-wide 4:3 window).  Cell sizes are
-- the 1080p ones in virtual units everywhere (far 45, near 176), the zoom
-- centre is half the virtual width/height (core_top ties field_n high, so
-- interlaced fields render alike: SD comes out line-doubled).  Before, one step served both axes and the near
-- cell came from FIELD lines, so 480i/1080i digits were ~2x tall (and 1080i
-- near cells half size); the far cell came from the width (SD smaller).
-- 1080p is bit-identical (both steps are exactly the old step there).
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

architecture ziffern of program_top is
    -- data_in -> data_out latency is 12 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;

    -- NP=5 dies in synthesis: yosys abc9 "Boxes are not in a topological order"
    -- crash mapping the 5-plane netlist (BRAM boxes + carries).  4 is the max
    -- this toolchain will build; plain-LUT-ROM NP=4 also fails (router congestion,
    -- 2 permanently-overused wires) -- the BRAM glyph ROM is what makes 4 fit.
    constant NP   : integer := 4;        -- depth planes (front=0 .. back=NP-1)
    constant FRAC : integer := 18;       -- phase fraction bits (sub-pixel, smooth zoom)
    constant C_GLIDE_SH : integer := 3;  -- zoom glide: pos += diff>>3 per frame (~8-frame tau)

    -- per-plane speed: cz = prog + triangle(prog)>>gshift.  triangle is 0 at both
    -- throw ends (so all planes start together / converge at the ends) and peaks
    -- mid-throw, bumped differently per plane so the front plane leads.  shift 1 =
    -- biggest lead (fastest), 15 = ~none (slowest, straight).
    type t_gsh is array (0 to NP - 1) of integer range 0 to 15;
    constant C_GSHIFT : t_gsh := (1, 2, 3, 15);

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLACK : unsigned(9 downto 0) := to_unsigned(64, 10);

    --------------------------------------------------------------------------
    -- Reciprocal LUT: C_RECIP(w) = (1<<FRAC)/w  (step = cells-per-pixel, Q.FRAC).
    --------------------------------------------------------------------------
    type t_recip is array (0 to 255) of unsigned(17 downto 0);
    function gen_recip return t_recip is
        variable r : t_recip;
        variable v : integer;
    begin
        r(0) := (others => '0');
        for w in 1 to 255 loop
            v := (2 ** FRAC) / w;
            if v > 262143 then v := 262143; end if;   -- clamp to 18 bits (only w<2)
            r(w) := to_unsigned(v, 18);
        end loop;
        return r;
    end function;
    constant C_RECIP : t_recip := gen_recip;

    --------------------------------------------------------------------------
    -- Hue ring (16 entries; index = hue_knob(9 downto 6)).  U/V-swapped, green=7.
    --------------------------------------------------------------------------
    constant C_NHUE : integer := 16;
    type t_hue is array (0 to C_NHUE - 1) of unsigned(9 downto 0);
    constant C_HUE_Y : t_hue := (
        to_unsigned(492,10), to_unsigned(660,10), to_unsigned(828,10), to_unsigned(996,10),
        to_unsigned(1023,10),to_unsigned(1023,10),to_unsigned(1017,10),to_unsigned(831,10),
        to_unsigned(558,10), to_unsigned(285,10), to_unsigned(261,10), to_unsigned(280,10),
        to_unsigned(346,10), to_unsigned(450,10), to_unsigned(519,10), to_unsigned(505,10));
    constant C_HUE_U : t_hue := (
        to_unsigned(960,10), to_unsigned(878,10), to_unsigned(796,10), to_unsigned(713,10),
        to_unsigned(631,10), to_unsigned(500,10), to_unsigned(304,10), to_unsigned(155,10),
        to_unsigned(288,10), to_unsigned(421,10), to_unsigned(503,10), to_unsigned(576,10),
        to_unsigned(645,10), to_unsigned(710,10), to_unsigned(782,10), to_unsigned(871,10));
    constant C_HUE_V : t_hue := (
        to_unsigned(360,10), to_unsigned(295,10), to_unsigned(230,10), to_unsigned(166,10),
        to_unsigned(101,10), to_unsigned( 92,10), to_unsigned(159,10), to_unsigned(262,10),
        to_unsigned(588,10), to_unsigned(914,10), to_unsigned(861,10), to_unsigned(746,10),
        to_unsigned(735,10), to_unsigned(803,10), to_unsigned(790,10), to_unsigned(575,10));

    --------------------------------------------------------------------------
    -- Digit glyphs (8x8, MSB = leftmost). idx 0..8 = plain '1'..'9';
    -- idx 9..17 = alien alphabet.
    --------------------------------------------------------------------------
    type t_digrom is array (0 to 17, 0 to 7) of std_logic_vector(7 downto 0);
    constant C_DIGITS : t_digrom := (
         0 => (x"10",x"30",x"10",x"10",x"10",x"10",x"38",x"00"),  -- 1
         1 => (x"38",x"44",x"04",x"08",x"10",x"20",x"7C",x"00"),  -- 2
         2 => (x"78",x"04",x"04",x"38",x"04",x"04",x"78",x"00"),  -- 3
         3 => (x"08",x"18",x"28",x"48",x"7C",x"08",x"08",x"00"),  -- 4
         4 => (x"7C",x"40",x"78",x"04",x"04",x"44",x"38",x"00"),  -- 5
         5 => (x"38",x"44",x"40",x"78",x"44",x"44",x"38",x"00"),  -- 6
         6 => (x"7C",x"04",x"08",x"10",x"20",x"20",x"20",x"00"),  -- 7
         7 => (x"38",x"44",x"44",x"38",x"44",x"44",x"38",x"00"),  -- 8
         8 => (x"38",x"44",x"44",x"3C",x"04",x"44",x"38",x"00"),  -- 9
         9 => (x"00",x"20",x"20",x"20",x"28",x"3C",x"10",x"10"),  -- alien
        10 => (x"40",x"40",x"40",x"40",x"60",x"60",x"70",x"00"),
        11 => (x"40",x"48",x"40",x"40",x"78",x"40",x"00",x"00"),
        12 => (x"40",x"40",x"70",x"40",x"40",x"60",x"70",x"20"),
        13 => (x"00",x"20",x"20",x"38",x"24",x"20",x"20",x"20"),
        14 => (x"00",x"78",x"40",x"40",x"40",x"40",x"48",x"00"),
        15 => (x"00",x"50",x"78",x"40",x"40",x"40",x"40",x"00"),
        16 => (x"00",x"20",x"20",x"3C",x"3C",x"20",x"20",x"00"),
        17 => (x"00",x"20",x"20",x"20",x"34",x"30",x"38",x"20"));

    -- Flattened glyph ROM for BRAM inference: one row byte per (glyph, row),
    -- addr = slot*8 + gy.  Plain digits in slots 0..8, alien in 16..24, so the
    -- S10 font select is address bit 7 (no +9 add on the hash path).  One synchronous read per plane replaces the wide
    -- LUT muxes whose fanout wiring congests the router at NP=4.  (Glow no
    -- longer needs neighbour rows: it is a SCREEN-space window on the
    -- composited lit mask, not a glyph-space ring.)
    type t_grom is array (0 to 255) of std_logic_vector(7 downto 0);
    function gen_grom return t_grom is
        variable r : t_grom := (others => (others => '0'));
    begin
        for idx in 0 to 17 loop
            for gy in 0 to 7 loop
                if idx < 9 then
                    r(idx * 8 + gy) := C_DIGITS(idx, gy);
                else
                    r((idx + 7) * 8 + gy) := C_DIGITS(idx, gy);
                end if;
            end loop;
        end loop;
        return r;
    end function;
    constant C_GROM : t_grom := gen_grom;

    --------------------------------------------------------------------------
    -- Per-plane array types
    --------------------------------------------------------------------------
    type t_uarr   is array (0 to NP - 1) of unsigned(11 downto 0);
    type t_sarr   is array (0 to NP - 1) of signed(8 downto 0);
    type t_u3arr  is array (0 to NP - 1) of unsigned(2 downto 0);
    type t_u16arr is array (0 to NP - 1) of unsigned(15 downto 0);
    type t_slv8arr is array (0 to NP - 1) of std_logic_vector(7 downto 0);
    type t_idxarr is array (0 to NP - 1) of integer range 0 to 31;
    type t_slarr  is array (0 to NP - 1) of std_logic;
    type t_zparr  is array (0 to NP - 1) of unsigned(31 downto 0);
    type t_czarr  is array (0 to NP - 1) of unsigned(16 downto 0);   -- Q10.6 progress
    type t_steparr is array (0 to NP - 1) of unsigned(17 downto 0);  -- step (cells/px)
    type t_stepharr is array (0 to NP - 1) of unsigned(18 downto 0); -- x step (cells/pixel)
    type t_stepvarr is array (0 to NP - 1) of unsigned(20 downto 0); -- y step (cells/field line)
    type t_marr   is array (0 to NP - 1) of unsigned(27 downto 0);   -- centre*step
    type t_pharr  is array (0 to NP - 1) of signed(27 downto 0);     -- phase Q10.18
    type t_frarr  is array (0 to NP - 1) of unsigned(17 downto 0);   -- cell fraction

    --------------------------------------------------------------------------
    -- Helpers
    --------------------------------------------------------------------------
    function mix16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x;
        v := v xor shift_left(v, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        v := v xor shift_right(v, 5);
        return v;
    end function;

    function spreadcr(col, row : signed(8 downto 0)) return unsigned is
        variable cu, ru : unsigned(15 downto 0);
    begin
        cu := resize(unsigned(col(7 downto 0)), 16);
        ru := resize(unsigned(row(7 downto 0)), 16);
        return (shift_left(cu, 8) + cu) + (shift_left(ru, 12) + ru);
    end function;

    function getbit(rb : std_logic_vector(7 downto 0); p : integer) return std_logic is
    begin
        if p >= 0 and p <= 7 then return rb(7 - p); else return '0'; end if;
    end function;

    function scale8(val : unsigned(9 downto 0); sel : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case sel is
            when "000" => r := shift_right(val, 3);
            when "001" => r := shift_right(val, 2);
            when "010" => r := shift_right(val, 2) + shift_right(val, 3);
            when "011" => r := shift_right(val, 1);
            when "100" => r := shift_right(val, 1) + shift_right(val, 3);
            when "101" => r := shift_right(val, 1) + shift_right(val, 2);
            when "110" => r := shift_right(val, 1) + shift_right(val, 2) + shift_right(val, 3);
            when others => r := val;
        end case;
        return r;
    end function;


    --------------------------------------------------------------------------
    -- Knob / switch reads
    --------------------------------------------------------------------------
    signal s_hue_knob    : unsigned(9 downto 0);
    signal s_change_rate : unsigned(9 downto 0);
    signal s_seed_knob   : unsigned(9 downto 0);
    signal s_zoom        : unsigned(9 downto 0);
    signal s_space       : unsigned(9 downto 0);
    signal s_glow        : unsigned(9 downto 0);
    signal s_sizek       : unsigned(9 downto 0);
    signal s_lumamod     : std_logic;
    signal s_bg_tint     : std_logic;
    signal s_video       : std_logic;
    signal s_font_sel    : std_logic;
    signal s_invert      : std_logic;

    --------------------------------------------------------------------------
    -- Timing / resolution
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';
    signal s_sawact      : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame constants
    --------------------------------------------------------------------------
    signal s_delta   : unsigned(17 downto 0) := (others => '0');    -- stepfar - stepnear
    signal s_stepfar : unsigned(17 downto 0) := to_unsigned(6554, 18);  -- (1<<FRAC)/Gbg
    signal s_stepnear: unsigned(17 downto 0) := to_unsigned(1092, 18);  -- (1<<FRAC)/Gfg
    -- pitch step per pixel / per field line: the per-virtual-unit pitch step
    -- times the mode's x / y step (both = the pitch step at 1080p)
    signal s_pstep_h : t_stepharr := (others => to_unsigned(6554, 19));
    signal s_pstep_v : t_stepvarr := (others => to_unsigned(6554, 21));

    -- Resolution scaling: timing-ID class (registered, free-running)
    --   s_tx   x step: 0 = 1.0 (1080), 1 = 1.5 (720p), 2 = 2.0 (SD)
    --   s_ys   y step per FIELD line, Q8
    signal s_tx      : unsigned(1 downto 0) := "00";
    signal s_t1080p  : std_logic := '1';
    signal s_ys      : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_wv2     : unsigned(11 downto 0) := to_unsigned(960, 12);  -- virtual W/2
    signal s_hc      : unsigned(11 downto 0) := to_unsigned(540, 12);  -- virtual H/2
    signal s_nmh, s_nmv : t_pharr := (others => (others => '0'));     -- -(centre*step)

    -- K4 Size / K5 Spacing (per-frame constants):
    --   R    = step scale from K4 (1536-K4: 1536=0.67x font .. 513=2x, 1024=unity@512)
    --   T10  = glyph's share of the cell from K5 (1023=packed .. 512=gap one font wide)
    --   RT   = (R*T10)>>10, single combined factor: pitch step = (font step * RT)>>10
    --   tk/t8 = glyph column thresholds k*(T10>>3) and gate 8*(T10>>3) on frac(17:8)
    signal s_rr    : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_t10   : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s_rt    : unsigned(10 downto 0) := to_unsigned(1024, 11);
    type t_thr is array (1 to 7) of unsigned(7 downto 0);
    signal s_tk    : t_thr := (to_unsigned(31,8),  to_unsigned(62,8),
                               to_unsigned(93,8),  to_unsigned(124,8),
                               to_unsigned(155,8), to_unsigned(186,8),
                               to_unsigned(217,8));
    signal s_t8    : unsigned(7 downto 0) := to_unsigned(248, 8);

    -- per-frame geometry engine (one serial multiplier, see p_frameeng)
    type t_eng is (E_IDLE, E_RT, E_CZ, E_SF, E_PT, E_MH, E_MV, E_VS);
    signal e_st  : t_eng := E_IDLE;
    signal e_pl  : integer range 0 to NP - 1 := 0;
    signal e_ez, e_tri : unsigned(16 downto 0) := (others => '0');
    signal e_lm  : std_logic := '0';
    signal e_a   : unsigned(17 downto 0) := (others => '0');   -- multiplicand
    signal e_hi  : unsigned(18 downto 0) := (others => '0');   -- product high part
    signal e_lo  : unsigned(17 downto 0) := (others => '0');   -- multiplier / product low
    signal e_cnt : integer range 0 to 17 := 0;
    signal e_run : std_logic := '0';
    signal e_ps  : unsigned(17 downto 0) := (others => '0');   -- this plane's pitch step

    signal s_seed  : unsigned(15 downto 0) := (others => '0');
    signal s_T     : unsigned(23 downto 0) := (others => '0');

    signal s_ny_raw : unsigned(9 downto 0) := (others => '0');  -- staged hue Y (LUT mux only)
    signal s_num_y, s_num_u, s_num_v : unsigned(9 downto 0) := (others => '0');
    signal s_bg_y,  s_bg_u,  s_bg_v  : unsigned(9 downto 0) := (others => '0');
    signal s_glow1_y, s_glow2_y, s_glow3_y
                       : unsigned(9 downto 0) := (others => '0');
    signal s_glow_on : std_logic := '0';

    -- filtered zoom position (Q10.6): eases toward (1023-P12)<<6 each frame
    signal s_zoomf  : unsigned(16 downto 0) := (others => '0');
    signal s_ztgt   : unsigned(16 downto 0) := (others => '0');  -- staged knob target


    --------------------------------------------------------------------------
    -- Per-pixel / per-line phase accumulators
    --------------------------------------------------------------------------
    signal s_phh, s_phv : t_pharr := (others => (others => '0'));
    signal s_firstline : std_logic := '1';

    --------------------------------------------------------------------------
    -- Pipeline registers (per plane)
    --------------------------------------------------------------------------
    signal s0_col, s0_row : t_sarr := (others => (others => '0'));
    signal s0_frh, s0_frv : t_frarr := (others => (others => '0'));   -- cell fraction
    signal s0_newrow : t_slarr := (others => '0');   -- this line starts a new cell row

    signal s1_base : t_u16arr := (others => (others => '0'));
    signal s1_gx, s1_gy : t_u3arr := (others => (others => '0'));
    signal s1_render : t_slarr := (others => '1');

    -- Luma Mod: per-plane per-cell luma level, sampled at each cell's
    -- top-left pixel into a small 1W1R buffer (BRAM), read back per pixel.
    -- Level (2 bits, vy(9:8)) selects which ladder rung (plane) draws the cell.
    type t_u8arr is array (0 to NP - 1) of unsigned(7 downto 0);
    type t_lvl2  is array (0 to NP - 1) of std_logic_vector(1 downto 0);
    type t_lvlbuf is array (0 to 255) of std_logic_vector(1 downto 0);
    signal r_colprev : t_sarr := (others => (others => '0'));
    signal r_rowprev : t_sarr := (others => (others => '0'));
    signal r_as_d    : std_logic := '0';
    signal s1_addr : t_u8arr := (others => (others => '0'));
    signal s1_wr   : t_slarr := (others => '0');
    signal sA_lvl  : t_lvl2 := (others => (others => '0'));

    signal sA_gen8 : t_slv8arr := (others => (others => '0'));
    signal sA_base : t_u16arr := (others => (others => '0'));
    signal sA_gx, sA_gy : t_u3arr := (others => (others => '0'));
    signal sA_owned, sA_render : t_slarr := (others => '0');

    signal s2_idx : t_idxarr := (others => 0);
    signal s2_gx, s2_gy : t_u3arr := (others => (others => '0'));
    signal s2_vis : t_slarr := (others => '0');

    signal s3_word : t_slv8arr := (others => (others => '0'));    -- BRAM glyph row
    signal s3_gx : t_u3arr := (others => (others => '0'));
    signal s3_vis : t_slarr := (others => '0');

    signal s4_center : t_slarr := (others => '0');

    -- screen-space glow: composited lit mask through a +/-3 px window; the
    -- output pixel is the window centre, glow level = distance to nearest lit
    signal s_litw : std_logic_vector(0 to 6) := (others => '0');

    -- sync delay pipeline (aligned through the glow window to the colour
    -- stage: S0..S4 + lit + 3-tap centre = 10 deep): [field avid vsync hsync]
    type t_syncp is array (0 to 9) of std_logic_vector(3 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));

    -- incoming video delayed to align with the colour stage (S9 video key)
    type t_vidp is array (0 to 9) of unsigned(9 downto 0);
    signal s_vy, s_vu, s_vv : t_vidp := (others => (others => '0'));

    signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');
    signal s5_sync : std_logic_vector(3 downto 0) := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

begin

    s_hue_knob    <= unsigned(registers_in(0));
    s_change_rate <= unsigned(registers_in(1));
    s_seed_knob   <= unsigned(registers_in(2));
    s_sizek       <= unsigned(registers_in(3));   -- K4 Size (smooth font scale)
    s_space       <= unsigned(registers_in(4));    -- K5 Spacing (gap, font unchanged)
    s_glow        <= unsigned(registers_in(5));
    s_zoom        <= unsigned(registers_in(7));    -- P12 Zoom (100%=far, 0%=front)
    s_lumamod     <= registers_in(6)(0);   -- S7 Luma Mod
    s_bg_tint     <= registers_in(6)(1);
    s_video       <= registers_in(6)(2);
    s_font_sel    <= registers_in(6)(3);
    s_invert      <= registers_in(6)(4);

    --------------------------------------------------------------------------
    -- Timing infrastructure
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    -- Analog vsync serrates (several edges per field): fire the per-frame
    -- pulse once per field only, on the first edge after active video.
    -- Without the guard s_T and the glide filter step several times per
    -- field on analog sources (faster, uneven change rate and glide).
    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev <= s_timing.vsync_n;
            if s_timing.avid = '1' then s_sawact <= '1'; end if;
            if s_vsync_prev = '0' and s_timing.vsync_n = '1' and s_sawact = '1' then
                s_vsync_pulse <= '1';
                s_sawact <= '0';
            else
                s_vsync_pulse <= '0';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Zoom glide filter: Q10.6 position easing toward the knob target each
    -- frame.  Raw prog = 1023 - P12; target = prog<<6.  pos += diff>>GLIDE_SH
    -- (signed; the >>3 truncation parks it <=7 sub-ticks (~0.3 px) from the
    -- target -- invisible, and NEVER snap the residual: a 1-tick snap is the
    -- 3 px pop this filter exists to remove).
    --------------------------------------------------------------------------
    p_zoomfilt : process(clk)
        variable v_diff : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            -- target staged every cycle (knob sub off the filter's add cone)
            s_ztgt <= shift_left(resize(to_unsigned(1023, 11) - resize(s_zoom, 11), 17), 6);
            if s_vsync_pulse = '1' then
                v_diff := signed(resize(s_ztgt, 19)) - signed(resize(s_zoomf, 19));
                s_zoomf <= unsigned(resize(signed(resize(s_zoomf, 19))
                                           + shift_right(v_diff, C_GLIDE_SH), 17));
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame geometry engine.  ONE serial unsigned shift-add multiplier
    -- (multiplier shifted out to the right, fixed shifts: no barrel shifter)
    -- sequenced after the vsync pulse through: RT = (R*T10)>>10, then per plane
    -- cz (prog + triangle bump, or the Luma Mod ladder), zprod = cz*delta, the
    -- font step stepf = max(stepfar - zprod>>16, 64), the pitch step
    -- pstep = clamp((stepf*RT)>>10, 64, 262143), and the line/column start
    -- phases -(centre*pstep) mod 2^28, stored pre-negated.  Same integers as the
    -- old four-wide per-stage multipliers (each with its own barrel), a few
    -- hundred clocks after the pulse -- long before the first active line.
    --------------------------------------------------------------------------
    p_frameeng : process(clk)
        variable v_cz  : unsigned(17 downto 0);
        variable v_st  : unsigned(17 downto 0);
        variable v_p   : unsigned(19 downto 0);
        variable v_sum : unsigned(18 downto 0);
        variable v_pr  : unsigned(36 downto 0);
    begin
        if rising_edge(clk) then
            v_pr := e_hi & e_lo;                      -- product of the last multiply
            if e_run = '1' then
                if e_lo(0) = '1' then v_sum := e_hi + resize(e_a, 19);
                else                  v_sum := e_hi; end if;
                e_hi <= '0' & v_sum(18 downto 1);
                e_lo <= v_sum(0) & e_lo(17 downto 1);
                if e_cnt = 17 then e_run <= '0'; else e_cnt <= e_cnt + 1; end if;
            else
                case e_st is
                    when E_IDLE =>
                        if s_vsync_pulse = '1' then
                            -- the filter updates s_zoomf on this same edge: latch
                            -- the pre-update value, as the old p_zmul did
                            e_ez <= s_zoomf;
                            if s_zoomf < to_unsigned(1023 * 32, 17) then e_tri <= s_zoomf;
                            else e_tri <= to_unsigned(1023 * 64, 17) - s_zoomf; end if;
                            e_lm <= s_lumamod;
                            e_pl <= 0;
                            e_st <= E_RT;             -- s_rr/s_t10 land on this edge
                        end if;
                    when E_RT =>
                        e_a <= resize(s_rr, 18); e_lo <= resize(s_t10, 18);
                        e_hi <= (others => '0'); e_cnt <= 0; e_run <= '1';
                        e_st <= E_CZ;
                    when E_CZ =>
                        if e_pl = 0 then s_rt <= v_pr(20 downto 10); end if;
                        if e_lm = '1' then
                            if e_pl = NP - 1 then v_cz := (others => '0');
                            else v_cz := resize(shift_right(e_ez, e_pl), 18); end if;
                        else
                            v_cz := resize(e_ez, 18) + resize(shift_right(e_tri, C_GSHIFT(e_pl)), 18);
                        end if;
                        e_a <= s_delta; e_lo <= resize(v_cz(16 downto 0), 18);
                        e_hi <= (others => '0'); e_cnt <= 0; e_run <= '1';
                        e_st <= E_SF;
                    when E_SF =>
                        v_st := s_stepfar - resize(v_pr(31 downto 16), 18);
                        if v_st < to_unsigned(64, 18) then v_st := to_unsigned(64, 18); end if;
                        e_a <= v_st; e_lo <= resize(s_rt, 18);
                        e_hi <= (others => '0'); e_cnt <= 0; e_run <= '1';
                        e_st <= E_PT;
                    when E_PT =>
                        v_p := v_pr(29 downto 10);
                        if v_p > to_unsigned(262143, 20) then v_p := to_unsigned(262143, 20); end if;
                        if v_p < to_unsigned(64, 20) then v_p := to_unsigned(64, 20); end if;
                        e_ps <= v_p(17 downto 0);
                        -- start phases from the VIRTUAL centre (W/2, H/2)
                        e_a <= resize(s_wv2, 18); e_lo <= v_p(17 downto 0);
                        e_hi <= (others => '0'); e_cnt <= 0; e_run <= '1';
                        e_st <= E_MH;
                    when E_MH =>
                        s_nmh(e_pl) <= -signed(v_pr(27 downto 0));
                        -- x step per pixel = pitch step * 1 / 1.5 / 2
                        case s_tx is
                            when "00"   => s_pstep_h(e_pl) <= resize(e_ps, 19);
                            when "01"   => s_pstep_h(e_pl) <= resize(e_ps, 19)
                                                              + resize(shift_right(e_ps, 1), 19);
                            when others => s_pstep_h(e_pl) <= e_ps & '0';
                        end case;
                        e_a <= resize(s_hc, 18); e_lo <= e_ps;
                        e_hi <= (others => '0'); e_cnt <= 0; e_run <= '1';
                        e_st <= E_MV;
                    when E_MV =>
                        s_nmv(e_pl) <= -signed(v_pr(27 downto 0));
                        -- y step per field line = pitch step * ys (Q8)
                        e_a <= e_ps; e_lo <= resize(s_ys, 18);
                        e_hi <= (others => '0'); e_cnt <= 0; e_run <= '1';
                        e_st <= E_VS;
                    when E_VS =>
                        s_pstep_v(e_pl) <= v_pr(28 downto 8);
                        if e_pl = NP - 1 then
                            e_st <= E_IDLE;
                        else
                            e_pl <= e_pl + 1;
                            e_st <= E_CZ;
                        end if;
                end case;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame cell sizes, colour, change-time (latched at vsync).
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_Gbg, v_Gfg : unsigned(11 downto 0);
        variable v_sf, v_sn : unsigned(17 downto 0);
        variable v_hidx : integer range 0 to 15;
        variable v_t10 : unsigned(9 downto 0);
        variable v_t1  : unsigned(7 downto 0);
        variable v_d : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- far cell 25% smaller than measured>>5 -> zoom starts deeper (~42 across)
            -- near cell anchored to picture HEIGHT: ~0.164*V (~6 rows at K4
            -- unity), so K4 max (x1.49) lands at the 4-numbers-high ceiling.
            -- Cell sizes are in virtual units: the 1080p values (45 / 176)
            -- in every other mode, measured at 1080p (bit-identical).
            if s_t1080p = '1' then
                v_Gbg := shift_right(s_measured_h, 5) - shift_right(s_measured_h, 7);
                v_Gfg := shift_right(s_measured_v, 3) + shift_right(s_measured_v, 5)
                         + shift_right(s_measured_v, 7);
            else
                v_Gbg := to_unsigned(45, 12);
                v_Gfg := to_unsigned(176, 12);
            end if;
            -- step endpoints (cells/pixel) = (1<<FRAC)/cellsize, via the recip LUT.
            v_sf := C_RECIP(to_integer(v_Gbg(7 downto 0)));   -- far  (bigger step)
            v_sn := C_RECIP(to_integer(v_Gfg(7 downto 0)));   -- near (smaller step)
            s_stepfar  <= v_sf;
            s_stepnear <= v_sn;

            -- delta from the REGISTERED endpoints (keeps the LUT-read off the subtract cone);
            -- clamp to 15 bits so cz(Q10.6)*delta stays within the 32-bit accumulator
            -- (only reachable with garbage timing measurements, Gbg < 8).
            if s_stepfar > s_stepnear then
                if s_stepfar - s_stepnear > to_unsigned(32767, 18) then
                    s_delta <= to_unsigned(32767, 18);
                else
                    s_delta <= s_stepfar - s_stepnear;
                end if;
            else s_delta <= (others => '0'); end if;

            if s_vsync_pulse = '1' then
                s_seed <= mix16(resize(s_seed_knob, 16) xor x"1B7F");
                -- no +1 floor: K2 at 0% = digits frozen (replaces the old
                -- Animate switch, freeing S7 for Luma Mod)
                s_T <= s_T + resize(shift_right(s_change_rate, 6), 24);

                -- uniform across planes; only the raw LUT reads here --
                -- the 50%->100% K6 brightness sum is computed NEXT cycle in
                -- p_numy off the REGISTERED s_ny_raw (LUT-mux + scale8 + add
                -- in one cone was STA-timed every cycle and capped Fmax)
                v_hidx := to_integer(s_hue_knob(9 downto 6));
                s_ny_raw <= C_HUE_Y(v_hidx);
                s_num_u <= C_HUE_U(v_hidx);
                s_num_v <= C_HUE_V(v_hidx);

                -- K4 Size: piecewise step scale, font 0.5x (K4=0) .. 1x (512)
                -- .. ~1.49x (1023).  The top is a hard visual ceiling: 1.49 *
                -- the height-anchored near cell = 4 numbers high at full zoom.
                if s_sizek(9) = '0' then
                    s_rr <= to_unsigned(2048, 12)
                            - shift_left(resize(s_sizek, 12), 1);
                else
                    v_d := resize(s_sizek(8 downto 0), 12);
                    s_rr <= to_unsigned(1024, 12)
                            - (shift_right(v_d, 1) + shift_right(v_d, 3)
                                                   + shift_right(v_d, 5));
                end if;
                -- K5 Spacing: glyph share of the cell T10 = 1023 - K5/2
                v_t10 := to_unsigned(1023, 10) - ("0" & s_space(9 downto 1));
                s_t10 <= v_t10;
                -- glyph column thresholds in 8-bit cell-fraction units
                -- (t1 = T10/4/8) + exact gate 8*t1
                v_t1 := resize(shift_right(v_t10, 5), 8);
                s_tk(1) <= v_t1;
                s_tk(2) <= shift_left(v_t1, 1);
                s_tk(3) <= shift_left(v_t1, 1) + v_t1;
                s_tk(4) <= shift_left(v_t1, 2);
                s_tk(5) <= shift_left(v_t1, 2) + v_t1;
                s_tk(6) <= shift_left(v_t1, 2) + shift_left(v_t1, 1);
                s_tk(7) <= shift_left(v_t1, 2) + shift_left(v_t1, 1) + v_t1;
                s_t8   <= shift_left(v_t1, 3);
            end if;
        end if;
    end process;





    -- Digit Y from the staged hue: idle 50%, K6 drives toward 100%.
    p_numy : process(clk)
    begin
        if rising_edge(clk) then
            if s_glow > to_unsigned(32, 10) then
                s_num_y <= shift_right(s_ny_raw, 1)
                           + shift_right(scale8(s_ny_raw, s_glow(9 downto 7)), 1);
            else
                s_num_y <= shift_right(s_ny_raw, 1);
            end if;
        end if;
    end process;

    -- Glow + background colours from the REGISTERED number colour.  Halo =
    -- three rings at 75/50/25% of the CURRENT digit Y (which itself rises
    -- 50%->100% with K6) in the full digit chroma.  Background sits at legal
    -- black (Y 64); Tint lifts it by digit Y/8 so the tint is visible (the
    -- old Y/16 with no pedestal was sub-black).
    p_colaux : process(clk)
    begin
        if rising_edge(clk) then
            s_glow1_y <= shift_right(s_num_y, 1) + shift_right(s_num_y, 2);
            s_glow2_y <= shift_right(s_num_y, 1);
            s_glow3_y <= shift_right(s_num_y, 2);
            if s_glow > to_unsigned(32, 10) then s_glow_on <= '1'; else s_glow_on <= '0'; end if;
            if s_bg_tint = '1' then
                s_bg_y <= C_BLACK + shift_right(s_num_y, 3); s_bg_u <= s_num_u; s_bg_v <= s_num_v;
            else
                s_bg_y <= C_BLACK; s_bg_u <= C_MID; s_bg_v <= C_MID;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Resolution scaling: timing-ID class, virtual centre.
    --------------------------------------------------------------------------
    p_tclass : process(clk)
    begin
        if rising_edge(clk) then
            s_t1080p <= '0';
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_tx <= "10"; s_ys <= to_unsigned(1138, 11);                  -- 480i
                when 4          => s_tx <= "10"; s_ys <= to_unsigned(576, 11);                   -- 480p
                when 8          => s_tx <= "10"; s_ys <= to_unsigned(960, 11);                   -- 576i
                when 12         => s_tx <= "10"; s_ys <= to_unsigned(480, 11);                   -- 576p
                when 5 | 6 | 14 => s_tx <= "01"; s_ys <= to_unsigned(384, 11);                   -- 720p
                when 1 | 2 | 10 => s_tx <= "00"; s_ys <= to_unsigned(512, 11);                   -- 1080i
                when others     => s_tx <= "00"; s_t1080p <= '1'; s_ys <= to_unsigned(256, 11);  -- 1080p
            end case;
            -- virtual W/2: measured_h/2 * x step
            case s_tx is
                when "00"   => s_wv2 <= shift_right(s_measured_h, 1);
                when "01"   => s_wv2 <= shift_right(s_measured_h, 1) + shift_right(s_measured_h, 2);
                when others => s_wv2 <= s_measured_h;
            end case;
            -- virtual H/2 (measured at 1080p)
            if s_t1080p = '1' then
                s_hc <= shift_right(s_measured_v, 1);
            else
                s_hc <= to_unsigned(540, 12);
            end if;
        end if;
    end process;

    p_firstline : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_timing.avid_start = '1' then
                s_firstline <= '0';
            end if;
        end if;
    end process;

    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syncp(0) <= data_in.field_n & data_in.avid &
                          data_in.vsync_n & data_in.hsync_n;
            s_vy(0) <= unsigned(data_in.y);
            s_vu(0) <= unsigned(data_in.u);
            s_vv(0) <= unsigned(data_in.v);
            for k in 1 to 9 loop
                s_syncp(k) <= s_syncp(k - 1);
                s_vy(k) <= s_vy(k - 1);
                s_vu(k) <= s_vu(k - 1);
                s_vv(k) <= s_vv(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: per-pixel phase accumulators -> cell index + glyph column, all planes.
    --   phase advances by step each pixel (horiz) / each line (vert); cell =
    --   integer part, glyph col/row = top 3 fraction bits.  Smooth = no modulo.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable stp, stv : signed(27 downto 0);
        variable cur_h, cur_v, nx_v : signed(27 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                stp := signed(resize(s_pstep_h(i), 28));   -- x step
                stv := signed(resize(s_pstep_v(i), 28));   -- y step

                if s_timing.avid_start = '1' then
                    -- vertical advance (per line)
                    if s_firstline = '1' then
                        nx_v := s_nmv(i);
                    else
                        nx_v := s_phv(i) + stv;
                    end if;
                    s_phv(i) <= nx_v;
                    cur_v := nx_v;
                    -- horizontal restart at line's first pixel
                    cur_h := s_nmh(i);
                    s_phh(i) <= cur_h + stp;
                elsif s_timing.avid = '1' then
                    cur_h := s_phh(i);
                    cur_v := s_phv(i);
                    s_phh(i) <= cur_h + stp;
                else
                    cur_h := s_phh(i);
                    cur_v := s_phv(i);
                end if;

                -- register cell index + fraction; the spacing slice happens in S1.
                s0_col(i) <= cur_h(26 downto 18);
                s0_row(i) <= cur_v(26 downto 18);
                s0_frh(i) <= unsigned(cur_h(17 downto 0));
                s0_frv(i) <= unsigned(cur_v(17 downto 0));
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- New-cell-row detect, one cycle after line start from REGISTERED rows
    -- (comparing on the fresh 28-bit phase add capped Fmax).  s0_newrow
    -- settles at avid_start+2, so the leftmost edge cell's luma sample can
    -- lag a cell-row -- invisible at the screen edge.
    --------------------------------------------------------------------------
    p_rowdet : process(clk)
    begin
        if rising_edge(clk) then
            r_as_d <= s_timing.avid_start;
            if r_as_d = '1' then
                for i in 0 to NP - 1 loop
                    if s0_row(i) /= r_rowprev(i) then s0_newrow(i) <= '1';
                    else s0_newrow(i) <= '0'; end if;
                    r_rowprev(i) <= s0_row(i);
                end loop;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: spatial de-correlation.
    --------------------------------------------------------------------------
    p_hashA : process(clk)
        variable v_fh, v_fv : unsigned(7 downto 0);
        variable v_gx, v_gy : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                s1_base(i) <= spreadcr(s0_col(i), s0_row(i));
                -- glyph occupies [0, t8) of the cell (t8 = 8*(T8/8), K5-set);
                -- column/row = popcount of thresholds passed (k * T8/8).
                -- Frac compared at 8 bits (1/256 cell): <=1 px boundary
                -- quantisation at the biggest cells, invisible, and 56
                -- narrower comparators at 90%+ utilisation.
                v_fh := s0_frh(i)(17 downto 10);
                v_fv := s0_frv(i)(17 downto 10);
                v_gx := (others => '0');
                v_gy := (others => '0');
                for k in 1 to 7 loop
                    if v_fh >= s_tk(k) then v_gx := v_gx + 1; end if;
                    if v_fv >= s_tk(k) then v_gy := v_gy + 1; end if;
                end loop;
                s1_gx(i) <= v_gx;
                s1_gy(i) <= v_gy;
                if (v_fh < s_t8) and (v_fv < s_t8) then s1_render(i) <= '1';
                else s1_render(i) <= '0'; end if;
                -- luma-mod cell sampling: cell's first pixel on its first line
                r_colprev(i) <= s0_col(i);
                s1_addr(i) <= unsigned(std_logic_vector(s0_col(i)(7 downto 0)));
                if (s0_col(i) /= r_colprev(i)) and (s0_newrow(i) = '1')
                   and (s_syncp(1)(2) = '1') then
                    s1_wr(i) <= '1';
                else
                    s1_wr(i) <= '0';
                end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Luma-mod level buffers: one tiny 256x2 1W1R memory per plane (written
    -- at each cell's top-left pixel, read every pixel by cell column).
    --------------------------------------------------------------------------
    g_lvl : for i in 0 to NP - 1 generate
        signal lvlbuf : t_lvlbuf := (others => (others => '0'));
    begin
        p_lvl : process(clk)
        begin
            if rising_edge(clk) then
                if s1_wr(i) = '1' then
                    lvlbuf(to_integer(s1_addr(i)))
                        <= std_logic_vector(s_vy(1)(9 downto 8));
                end if;
                sA_lvl(i) <= lvlbuf(to_integer(s1_addr(i)));
            end if;
        end process;
    end generate;

    --------------------------------------------------------------------------
    -- SA: per-cell hash -> staggered generation index + balanced plane split.
    --------------------------------------------------------------------------
    p_hashMix : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_pc : integer range 0 to 4;
        variable v_rate : integer range 0 to 10;
        variable v_gen  : unsigned(23 downto 0);
        variable v_hi : unsigned(7 downto 0);
        variable v_pof : integer range 0 to NP - 1;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_h := mix16(s1_base(i) xor s_seed);
                v_pc := 0;
                for b in 8 to 11 loop
                    if v_h(b) = '1' then v_pc := v_pc + 1; end if;
                end loop;
                v_rate := 6 + v_pc;                                  -- 6..10
                v_gen  := shift_right(s_T + resize(v_h(7 downto 0), 24), v_rate);
                sA_gen8(i) <= std_logic_vector(v_gen(7 downto 0));
                -- balanced NP-way split (well-mixed high byte -> equal shares)
                v_hi := v_h(15 downto 8);
                v_pof := NP - 1;
                for k in NP - 2 downto 0 loop
                    if v_hi < to_unsigned(((k + 1) * 256) / NP, 8) then
                        v_pof := k;
                    end if;
                end loop;
                if v_pof = i then sA_owned(i) <= '1'; else sA_owned(i) <= '0'; end if;
                sA_base(i) <= s1_base(i);
                sA_gx(i) <= s1_gx(i); sA_gy(i) <= s1_gy(i);
                sA_render(i) <= s1_render(i);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2: digit glyph index.
    --------------------------------------------------------------------------
    p_hashB : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_m : unsigned(11 downto 0);
        variable v_off : integer range 0 to 8;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_h := mix16(sA_base(i) xor s_seed xor (unsigned(sA_gen8(i)) & unsigned(sA_gen8(i))));
                -- balanced 1-of-9 pick: (h8 * 9) >> 8 -> each glyph gets 28
                -- or 29 of 256 hash values (the old 16->9 table gave 8 and 9
                -- half the share of 1..7)
                v_m := shift_left(resize(v_h(11 downto 4), 12), 3)
                       + resize(v_h(11 downto 4), 12);
                v_off := to_integer(v_m(11 downto 8));
                if s_font_sel = '1' then s2_idx(i) <= v_off + 16;
                else                     s2_idx(i) <= v_off; end if;
                s2_gx(i) <= sA_gx(i); s2_gy(i) <= sA_gy(i);
                -- Luma Mod: ownership is LUMA-selected, not hash-split --
                -- plane i draws cells whose sampled level is 3-i (dark ->
                -- back/far rung, bright -> front/zoomed-in rung).
                if s_lumamod = '1' then
                    if sA_lvl(i) = std_logic_vector(to_unsigned(NP - 1 - i, 2)) then
                        s2_vis(i) <= sA_render(i);
                    else
                        s2_vis(i) <= '0';
                    end if;
                else
                    s2_vis(i) <= sA_owned(i) and sA_render(i);
                end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3: glyph row byte, one BRAM read per plane (own generate block so each
    -- plane reliably infers its own EBR instead of a 4-read-port memory).
    --------------------------------------------------------------------------
    g_font : for i in 0 to NP - 1 generate
        p_font : process(clk)
        begin
            if rising_edge(clk) then
                s3_word(i) <= C_GROM(to_integer(
                                  to_unsigned(s2_idx(i), 5) & s2_gy(i)));
                s3_gx(i) <= s2_gx(i);
                s3_vis(i) <= s2_vis(i);
            end if;
        end process;
    end generate;

    --------------------------------------------------------------------------
    -- S4: per-plane centre pixel.
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable p : integer range -1 to 8;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                p := to_integer(s3_gx(i));
                s4_center(i) <= getbit(s3_word(i), p) and s3_vis(i);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5w: composite the planes' lit bits and slide them through a 7-tap
    -- window.  The colour stage outputs the window CENTRE (litw(3)); taps
    -- 2/4, 1/5, 0/6 are the pixels 1..3 SCREEN px away, giving a glow that
    -- fades with true screen distance regardless of digit size.
    --------------------------------------------------------------------------
    p_lit : process(clk)
        variable v_any : std_logic;
    begin
        if rising_edge(clk) then
            v_any := '0';
            for i in 0 to NP - 1 loop
                if s4_center(i) = '1' then v_any := '1'; end if;
            end loop;
            s_litw <= v_any & s_litw(0 to 5);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5: composite, colour, glow rings, video key, invert.
    --------------------------------------------------------------------------
    p_color : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_lit : std_logic;
        variable v_g1, v_g2, v_g3 : std_logic;
        variable v_on_y, v_on_u, v_on_v : unsigned(9 downto 0);
        variable v_off_y, v_off_u, v_off_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_video = '1' then
                if s_bg_tint = '1' then
                    v_bgy := resize(shift_right(resize(s_bg_y,11) + resize(s_vy(9),11), 1), 10);
                    v_bgu := resize(shift_right(resize(s_bg_u,11) + resize(s_vu(9),11), 1), 10);
                    v_bgv := resize(shift_right(resize(s_bg_v,11) + resize(s_vv(9),11), 1), 10);
                else
                    v_bgy := s_vy(9); v_bgu := s_vu(9); v_bgv := s_vv(9);
                end if;
            else
                v_bgy := s_bg_y; v_bgu := s_bg_u; v_bgv := s_bg_v;
            end if;

            -- window centre is the output pixel; taps outward = screen px away
            v_lit := s_litw(3);
            v_g1  := s_litw(2) or s_litw(4);
            v_g2  := s_litw(1) or s_litw(5);
            v_g3  := s_litw(0) or s_litw(6);

            -- uniform brightness across all planes (no depth fade)
            if s_invert = '0' then
                v_on_y := s_num_y; v_on_u := s_num_u; v_on_v := s_num_v;
                v_off_y := v_bgy;  v_off_u := v_bgu; v_off_v := v_bgv;
            else
                v_on_y := v_bgy;   v_on_u := v_bgu; v_on_v := v_bgv;
                v_off_y := s_num_y; v_off_u := s_num_u; v_off_v := s_num_v;
            end if;

            if v_lit = '1' then
                v_y := v_on_y; v_u := v_on_u; v_v := v_on_v;
            elsif (v_g1 = '1') and (s_glow_on = '1') then
                v_y := s_glow1_y; v_u := s_num_u; v_v := s_num_v;
            elsif (v_g2 = '1') and (s_glow_on = '1') then
                v_y := s_glow2_y; v_u := s_num_u; v_v := s_num_v;
            elsif (v_g3 = '1') and (s_glow_on = '1') then
                v_y := s_glow3_y; v_u := s_num_u; v_v := s_num_v;
            else
                v_y := v_off_y; v_u := v_off_u; v_v := v_off_v;
            end if;

            s5_y <= v_y; s5_u <= v_u; s5_v <= v_v;
            s5_sync <= s_syncp(9);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output.  Blanking gate: outside active video the accumulators hold, so
    -- the pipeline would keep emitting the last pixel (digit colour, or the
    -- whole blanking in digit colour under Invert/Tint) -- corrupts the
    -- encoder's colour reference.  Force neutral whenever the aligned avid
    -- is low (constant load = FF sync set/reset, ~free).
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            if s5_sync(2) = '1' then
                s_io.y <= std_logic_vector(s5_y);
                s_io.u <= std_logic_vector(s5_u);
                s_io.v <= std_logic_vector(s5_v);
            else
                s_io.y <= std_logic_vector(C_BLACK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= s5_sync(0);
            s_io.vsync_n <= s5_sync(1);
            s_io.avid    <= s5_sync(2);
            s_io.field_n <= s5_sync(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture ziffern;
