-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- gumball.vhd  (v1.0.1 — video as a wall of glossy spheres)
--
-- v1.0.3 (2026-10-17): resolution scaling.  The two X lattices and the row
-- counter run in virtual 1080-line units, stepped per pixel / per field line
-- from the video timing ID (registers_in(8)): 1.0 at 1080p (identical to
-- before), 1.5 at 720p, and at SD 2.0 across (a 1440-wide 4:3 window) and
-- 1080/486 per frame line, so the balls keep their size relative to the
-- screen height and stay round on a 4:3 screen (they were 1.5x too big at
-- 720p and huge 2:1 eggs at SD).  The row counter keeps its unit steps (and
-- the incremental per-line squares) by running the integer part of each
-- field line's step as that many unit steps after the h edge; centre tests
-- became crossings.  The AA ring stays ~0.9 real px.  Also fixes a 1080p
-- column-index wrap: Ball Size below ~15 px gave >128 columns and the 7-bit
-- cell index aliased the right-hand balls' colours onto the left fifth of
-- the screen (cell index now 8 bits, buffers 256 / 512 deep).
--
-- Replaces the incoming video with a hex-packed field of shaded spheres, one
-- per cell, coloured by the live video (a pixelation, but every "pixel" is a
-- lit 3D ball).  Rows are offset by half a cell (hex packing) and the sphere
-- radius can exceed the cell, in which case neighbouring spheres SQUASH flat
-- against each other along the hex Voronoi boundary — round balls with gently
-- flattened contact facets, like pressed bubbles.
--
-- Geometry: two half-offset X lattices (A = even rows, B = odd rows) run in
-- parallel; per pixel the squared distance to the nearest own-row centre and
-- the nearest adjacent-row centre is compared (nearest-of-2 IS the exact hex
-- Voronoi for offset rows), the winner supplies the cell colour and the local
-- (dx, dy) used for shading.  Row height H = W*7/8 (~hex).  No divide and no
-- wide multiply on the pixel path.
--
-- Shading: one offset-light distance field e2 = (dx-ox)^2 + (dy-oy)^2 drives
-- both the diffuse dome (bright at the light spot, falling to ambient at the
-- far rim) and the specular hotspot (tight ramp of the SAME field).  Both are
-- normalised without a pixel divide: per frame a shift sl and an 8-bit gain
-- k are found such that value*2^sl/256*k/128 == value/range, so the pixel path
-- is a 2-stage pipelined barrel shift + one small multiply per field.  The
-- per-frame maths (r, light vector, ranges, two serial divides) runs in a
-- ~85-cycle FSM on ONE shared 12x12 multiplier inside the vsync cone.  The
-- per-LINE squares are updated incrementally at each h edge (adds only), so
-- no line-rate multiplier exists for STA to time at the pixel clock.
--
-- Cell colours are point-sampled once per cell row: row R's colours are
-- written (sat-boosted, plus the optional luma->radius term) into its parity's
-- buffer while the raster scans the CENTRE line of row R-1 — before any pixel
-- of row R's spheres can win the Voronoi test (they first win ~0.16*H later),
-- so the buffer swap is never visible.  Row 0 is sampled on the first active
-- line (which is drawn as backdrop: it is read while being written).  A
-- right-edge partial cell whose centre lies past the line end is sampled at
-- the last active pixel.  Per-cell radius^2 is stored alongside the colour so
-- Luma Size costs nothing per pixel.
--
-- Buffers (v1.0): rsq (15 b) + sample-line hsync->avid parity live in two
-- 128x16 memories read for both candidates at c1; the colour (Y-64 10 b,
-- U/V offsets 11 b = 32 b) lives in ONE 256x32 memory addressed {parity,
-- cell} and is re-read for the Voronoi WINNER at c10 — no colour delay pipes.
-- Hardware contracts: blanking gate at compose; U/V swapped where the sample
-- line's hsync->avid parity differs from the display line's (drape); row
-- anchor re-armed by 2 avid-less lines, not the vsync edge; luma drawn
-- black-referenced with a soft knee capped at 800.
--
-- Controls:
--   K1  Ball Size    (smooth 12..128 px)
--   K2  Light Angle  (0..360 deg — spins the highlight/shadow around each ball)
--   K3  Specular     (highlight intensity, up to a deliberate blowout)
--   K4  Shine        (highlight tightness: broad satin sheen -> pinpoint gloss)
--   K5  Squash       (radius vs cell: separated beads -> touching -> pressed
--                     flat into a full-coverage domed mosaic)
--   K6  Candy        (chroma gain on the sampled colours; 0.75x .. 2.7x;
--                     under Chrome: tint amount, mono .. 0.5x colour)
--   S7  Backdrop     (Black / dimmed live video behind and between the balls)
--   S8  Luma Size    (bright cells grow, dark cells shrink, +/- r/2)
--   S9  Packing      (Hex offset rows / straight square grid)
--   S10 Chrome       (ball-bearings: chroma down to a K6-set tint, mono backdrop)
--   S11 Bypass
--   P12 Relief       (THE fader: 0 = soft low-relief satin balls, max = deep
--                     shadowed, wet-gloss 3D — shading span and specular
--                     master gain ride it together; floored so K2-K4 always
--                     read)
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

architecture gumball of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (17 + 3 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY  : natural := 16;                 -- video pipe 0..15, out at c17
    constant C_MAXCOL : natural := 256;                -- max cell columns (W >= 12 virtual -> <= 161)

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- normalising shift: smallest sl with (v << sl) in [2^15, 2^16)
    function norm_shift(v : unsigned(15 downto 0)) return natural is
    begin
        for i in 15 downto 0 loop
            if v(i) = '1' then
                return 15 - i;
            end if;
        end loop;
        return 15;
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
    -- Raster position: two half-offset X lattices + row counter.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';
    signal line_av      : std_logic := '0';            -- avid seen since last h edge
    signal idle_ln      : unsigned(1 downto 0) := (others => '0');  -- avid-less lines
    signal hpar         : std_logic := '0';            -- clock parity since hsync
    signal line_par     : std_logic := '0';            -- hsync->avid parity, this line
    signal srow_r       : std_logic := '0';            -- this line is a sample line
    signal tp_r         : std_logic := '0';            -- its target row parity
    signal r_step       : std_logic := '0';            -- registered row unit step (line maths kick)
    signal r_anchor     : std_logic := '0';            -- first-active-line event

    -- Resolution scaling (v1.0.3): the lattices run in virtual 1080-line
    -- units.  x: Q.1 accumulators (half units; step 2/3/4 = 1.0/1.5/2.0 per
    -- pixel), xa_la(8 downto 1) is the old integer xa_loc.  y: celly_loc
    -- still counts whole virtual lines, but each field line advances it by
    -- the integer part of a Q4.8 accumulator (1 at 1080p, 1-2 at 720p, 4-5
    -- at 480i), one unit step per clock after the h edge, so every row
    -- rule below (wrap, mid-row jump, incremental squares) is unchanged.
    signal s_xs   : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal xa_la, xb_la   : unsigned(8 downto 0) := (others => '0');
    signal xa_lt, xb_lt   : std_logic := '1';          -- previous pixel short of the centre
    signal s_wlim   : unsigned(8 downto 0) := to_unsigned(94, 9);   -- 2W - step: wrap threshold
    signal cy_frac  : unsigned(7 downto 0) := (others => '0');
    signal cy_left  : unsigned(2 downto 0) := (others => '0');      -- unit steps still due
    signal cy_hit   : std_logic := '0';                -- this line reached the row centre
    signal xa_idx, xb_idx : unsigned(7 downto 0) := (others => '0');
    signal celly_loc      : unsigned(7 downto 0) := (others => '0');
    signal celly_par      : std_logic := '0';
    signal frame_act      : std_logic := '0';
    signal band0          : std_logic := '0';          -- top band: no row above
    signal line0          : std_logic := '0';          -- first active line (row-0 sample)

    --------------------------------------------------------------------------
    -- Per-frame parameters (latched at vsync, then refined by the frame FSM).
    --------------------------------------------------------------------------
    signal lk1, lk2, lk3, lk4, lk5, lk6, lp12 : unsigned(9 downto 0) := (others => '0');
    signal k1_mv, k4_mv, k5_mv : std_logic := '0';     -- knob left its deadband

    signal s_w      : unsigned(7 downto 0) := to_unsigned(48, 8);   -- cell width W
    signal s_wh     : unsigned(7 downto 0) := to_unsigned(24, 8);   -- W/2
    signal s_h      : unsigned(7 downto 0) := to_unsigned(42, 8);   -- row height H
    signal s_hm1    : unsigned(7 downto 0) := to_unsigned(41, 8);
    signal s_hh     : unsigned(7 downto 0) := to_unsigned(21, 8);   -- H/2
    signal s_rfrac  : unsigned(10 downto 0) := to_unsigned(1000, 11);
    signal s_kshine : unsigned(10 downto 0) := to_unsigned(576, 11);
    signal s_r      : unsigned(7 downto 0) := to_unsigned(25, 8);   -- sphere radius
    signal s_rim2   : unsigned(9 downto 0) := to_unsigned(75, 10);  -- 1.75r (x step/2): ~0.9 px AA band
    signal s_ls     : unsigned(7 downto 0) := to_unsigned(15, 8);   -- light offset 5r/8
    signal s_es     : unsigned(7 downto 0) := to_unsigned(12, 8);   -- specular radius
    signal s_ox, s_oy : signed(7 downto 0) := (others => '0');      -- light offset vector
    signal s_e2     : unsigned(15 downto 0) := to_unsigned(1600, 16);
    signal s_es2    : unsigned(15 downto 0) := to_unsigned(144, 16);
    signal s_sle    : unsigned(3 downto 0) := (others => '0');      -- diffuse norm shift
    signal s_sls    : unsigned(3 downto 0) := (others => '0');      -- spec norm shift
    signal s_kd8    : unsigned(7 downto 0) := (others => '0');      -- diffuse norm gain
    signal s_ks9    : unsigned(9 downto 0) := (others => '0');      -- spec norm gain
    signal s_sp     : unsigned(7 downto 0) := (others => '0');      -- shading span (P12)
    signal s_amb    : unsigned(7 downto 0) := to_unsigned(255, 8);  -- ambient = 255 - span
    signal s_si     : unsigned(8 downto 0) := (others => '0');      -- spec intensity (K3)
    signal s_speff  : unsigned(9 downto 0) := (others => '0');      -- spec * relief master
    signal s_sg     : unsigned(7 downto 0) := to_unsigned(64, 8);   -- sat gain (64 = unity)

    signal s_bgvid  : std_logic := '0';                -- S7
    signal s_lsize  : std_logic := '0';                -- S8
    signal s_square : std_logic := '0';                -- S9
    signal s_mono   : std_logic := '0';                -- S10
    signal s_bypass : std_logic := '0';                -- S11

    -- sine LUT (per-frame light angle)
    signal lut_sin, lut_cos : signed(9 downto 0);
    signal sinv, cosv       : signed(9 downto 0) := (others => '0');

    -- frame FSM shared multiplier + serial divider
    signal fsm_t   : unsigned(7 downto 0) := (others => '1');       -- FF = idle
    signal ma, mb  : signed(11 downto 0) := (others => '0');
    signal mp      : signed(23 downto 0) := (others => '0');
    signal div_run : std_logic := '0';
    signal div_cnt : unsigned(4 downto 0) := (others => '0');
    signal div_d   : unsigned(17 downto 0) := (others => '0');      -- dividend (shifting)
    signal div_q   : unsigned(17 downto 0) := (others => '0');      -- quotient
    signal div_rem : unsigned(8 downto 0)  := (others => '0');
    signal div_v   : unsigned(7 downto 0)  := (others => '1');      -- divisor (128..255)

    --------------------------------------------------------------------------
    -- Per-line constants (dy / light-relative dy, squared).  dy advances by
    -- exactly +1 per line, so the squares are updated INCREMENTALLY at each
    -- h edge — (d+1)^2 = d^2 + 2(d+1) - 1, adds only, no multiplier for STA
    -- to time — with the band-wrap / mid-band-jump reset values precomputed
    -- per frame by the FSM (hh2q / hsq / eyw*sq / eyj2sq).
    --------------------------------------------------------------------------
    signal lm_dy1   : signed(8 downto 0) := (others => '0');
    signal lm_dy2   : signed(8 downto 0) := (others => '0');
    signal lm_ey1   : signed(8 downto 0) := (others => '0');
    signal lm_ey2   : signed(8 downto 0) := (others => '0');
    signal dy1sq_r  : unsigned(15 downto 0) := (others => '0');
    signal dy2sq_r  : unsigned(15 downto 0) := (others => '0');
    signal ey1sq_r  : unsigned(15 downto 0) := (others => '0');
    signal ey2sq_r  : unsigned(15 downto 0) := (others => '0');
    signal hh2q     : unsigned(15 downto 0) := (others => '0');     -- (H/2)^2
    signal hsq      : unsigned(15 downto 0) := (others => '0');     -- H^2
    signal eyw1sq   : unsigned(15 downto 0) := (others => '0');     -- (H/2 + oy)^2
    signal eyw2sq   : unsigned(15 downto 0) := (others => '0');     -- (H/2 - oy)^2
    signal eyj2sq   : unsigned(15 downto 0) := (others => '0');     -- (H + oy)^2

    --------------------------------------------------------------------------
    -- Cell buffers (all 1W1R, registered read, power-of-2 word widths):
    --   bufr0/bufr1 : per parity, word = line_par & rsq15 (both read at c1)
    --   bufc        : {parity, cell} -> (Y-64)10 & Uoff11 & Voff11 (winner
    --                 re-read at c10; chroma offsets signed about 512)
    --------------------------------------------------------------------------
    subtype t_rword is std_logic_vector(15 downto 0);
    type t_rbuf is array(0 to C_MAXCOL - 1) of t_rword;
    signal bufr0, bufr1 : t_rbuf := (others => (others => '0'));
    signal rd0, rd1     : t_rword := (others => '0');
    subtype t_cword is std_logic_vector(31 downto 0);
    type t_cbuf is array(0 to 2 * C_MAXCOL - 1) of t_cword;
    signal bufc : t_cbuf := (others => (others => '0'));   -- 0 offsets = neutral
    signal rdc  : t_cword := (others => '0');

    -- sample/write pipe (W0..W3 -> write)
    signal w0_stb : std_logic := '0';
    signal w0_par : std_logic := '0';
    signal w0_idx : unsigned(7 downto 0) := (others => '0');
    signal w0_y, w0_u, w0_v : unsigned(9 downto 0) := C_MID;
    signal w1_stb : std_logic := '0';
    signal w1_par : std_logic := '0';
    signal w1_idx : unsigned(7 downto 0) := (others => '0');
    signal w1_y   : unsigned(9 downto 0) := (others => '0');   -- Y - 64
    signal w1_pu, w1_pv : signed(19 downto 0) := (others => '0');
    signal w1_pl  : signed(19 downto 0) := (others => '0');
    signal w2_stb : std_logic := '0';
    signal w2_par : std_logic := '0';
    signal w2_idx : unsigned(7 downto 0) := (others => '0');
    signal w2_y   : unsigned(9 downto 0) := (others => '0');
    signal w2_u, w2_v : signed(10 downto 0) := (others => '0');
    signal w2_rr  : unsigned(7 downto 0) := (others => '0');
    signal w3_stb : std_logic := '0';
    signal w3_par : std_logic := '0';
    signal w3_idx : unsigned(7 downto 0) := (others => '0');
    signal w3_y   : unsigned(9 downto 0) := (others => '0');
    signal w3_u, w3_v : signed(10 downto 0) := (others => '0');
    signal w3_rsq : unsigned(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline (c1..c16).
    --------------------------------------------------------------------------
    signal g0_dx1, g0_dx2 : signed(7 downto 0) := (others => '0');
    signal g0_a0, g0_a1   : unsigned(7 downto 0) := (others => '0');
    signal g0_noadj       : std_logic := '0';
    signal g0_blank       : std_logic := '0';

    signal p2_dx1, p2_dx2 : signed(7 downto 0) := (others => '0');
    signal p2_noadj       : std_logic := '0';
    signal p2_blank       : std_logic := '0';
    signal p2_a0, p2_a1   : unsigned(7 downto 0) := (others => '0');
    signal q1, q2         : unsigned(13 downto 0) := (others => '0');

    signal p3_dx1, p3_dx2 : signed(7 downto 0) := (others => '0');
    signal p3_rd0, p3_rd1 : t_rword := (others => '0');
    signal p3_blank       : std_logic := '0';
    signal p3_a0, p3_a1   : unsigned(7 downto 0) := (others => '0');
    signal d1, d2         : unsigned(15 downto 0) := (others => '0');

    signal p4_dsq  : unsigned(15 downto 0) := (others => '0');
    signal p4_dx   : signed(7 downto 0) := (others => '0');
    signal p4_eys  : unsigned(15 downto 0) := (others => '0');
    signal p4_word : t_rword := (others => '0');
    signal p4_blank : std_logic := '0';
    signal p4_ca   : unsigned(8 downto 0) := (others => '0');  -- winner {parity, cell}

    signal p5_ex   : signed(7 downto 0) := (others => '0');
    signal p5_in   : std_logic := '0';
    signal p5_core : std_logic := '0';
    signal p5_eys  : unsigned(15 downto 0) := (others => '0');

    signal p6_exq  : unsigned(13 downto 0) := (others => '0');
    signal p6_eys  : unsigned(15 downto 0) := (others => '0');

    signal p7_e2   : unsigned(16 downto 0) := (others => '0');
    signal p8_dl   : unsigned(14 downto 0) := (others => '0');
    signal p8_ds   : unsigned(13 downto 0) := (others => '0');
    signal p9_dlc  : unsigned(15 downto 0) := (others => '0');
    signal p9_dsc  : unsigned(15 downto 0) := (others => '0');
    signal p10_dlb : unsigned(7 downto 0) := (others => '0');
    signal p10_dsb : unsigned(7 downto 0) := (others => '0');
    signal p11_pl  : unsigned(15 downto 0) := (others => '0');
    signal p11_ps  : unsigned(17 downto 0) := (others => '0');
    signal p12_l   : unsigned(7 downto 0) := (others => '0');
    signal p12_s   : unsigned(8 downto 0) := (others => '0');
    signal p13_ym  : unsigned(17 downto 0) := (others => '0');
    signal p13_cf  : unsigned(7 downto 0) := (others => '0');
    signal p13_s   : unsigned(8 downto 0) := (others => '0');
    signal p14_um, p14_vm : signed(19 downto 0) := (others => '0');
    signal p14_yd  : unsigned(10 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : unsigned(9 downto 0) := C_MID;
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := C_MID;

    -- winner colour re-read: address pipe c5..c10, data c12, swapped c13
    type t_ca_p is array (natural range <>) of unsigned(8 downto 0);
    signal ca_p   : t_ca_p(5 to 10) := (others => (others => '0'));
    signal wpar_p : std_logic_vector(5 to 12) := (others => '0');
    signal c12_y  : unsigned(9 downto 0) := (others => '0');
    signal c12_u, c12_v : signed(10 downto 0) := (others => '0');
    signal c13_u, c13_v : signed(10 downto 0) := (others => '0');

    -- flag pipes
    signal in_p   : std_logic_vector(0 to 9) := (others => '0');
    signal core_p : std_logic_vector(0 to 9) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal s_io : t_video_stream_yuv444_30b;

begin

    u_sincos : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(lk2),
            sin_out  => lut_sin,
            cos_out  => lut_cos
        );

    --------------------------------------------------------------------------
    -- Raster position counters + sample trigger.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_tp     : std_logic;
        variable v_trig   : std_logic;
        variable v_loc    : unsigned(7 downto 0);
        variable v_ge     : std_logic;
        variable v_lt     : std_logic;
        variable v_bwrap  : std_logic;
        variable v_step   : std_logic;
        variable v_ysum   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;

            v_h_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;

            r_anchor <= '0';

            -- per-mode virtual steps (static per mode; re-latching at every
            -- vsync edge is idempotent)
            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            end if;

            -- hsync->avid offset parity of this line (encoder Cb/Cr pairing)
            hpar <= not hpar;
            if v_h_edge = '1' then hpar <= '0'; end if;
            if data_in.avid = '1' and prev_avid = '0' then line_par <= hpar; end if;

            -- avid-less line count: re-arms the row anchor (not the vsync edge,
            -- which decoders may assert before the last active line ends)
            if data_in.avid = '1' then line_av <= '1'; end if;
            if v_h_edge = '1' then
                line_av <= '0';
                if line_av = '1' then
                    idle_ln <= (others => '0');
                elsif idle_ln /= 3 then
                    idle_ln <= idle_ln + 1;
                end if;
            end if;

            -- X lattices: A aligned, B half-offset (or aligned too in square
            -- mode), in virtual half units.  A cell wraps when the NEXT step
            -- would reach W (la >= 2W - step: at 1080p la = 2(W-1), the old
            -- loc = W-1 test); xa_lt/xb_lt register "short of the centre" so
            -- the centre test is a crossing a fractional step can't jump.
            v_bwrap := '0';
            if xa_la < (s_wh & '0') then xa_lt <= '1'; else xa_lt <= '0'; end if;
            if xb_la < (s_wh & '0') then xb_lt <= '1'; else xb_lt <= '0'; end if;
            if v_h_edge = '1' then
                xa_la  <= (others => '0');
                xa_idx <= (others => '0');
                if s_square = '1' then
                    xb_la <= (others => '0');
                else
                    xb_la <= s_wh & '0';
                end if;
                xb_idx <= (others => '0');
            elsif data_in.avid = '1' then
                if xa_la >= s_wlim then
                    xa_la  <= xa_la - s_wlim;
                    xa_idx <= xa_idx + 1;
                else
                    xa_la <= xa_la + s_xs;
                end if;
                if xb_la >= s_wlim then
                    xb_la  <= xb_la - s_wlim;
                    xb_idx <= xb_idx + 1;
                    v_bwrap := '1';
                else
                    xb_la <= xb_la + s_xs;
                end if;
            end if;

            -- Y row counter, anchored to the first active line of the frame.
            -- Each h edge adds the Q4.8 field-line step; its integer part is
            -- run as that many unit steps (the first at the edge itself, the
            -- rest on the following clocks — all long before active video).
            if celly_loc = s_hh then cy_hit <= '1'; end if;
            v_step := '0';
            if data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_par <= '0';
                band0     <= '1';
                line0     <= '1';
                r_anchor  <= '1';
                cy_frac   <= (others => '0');
                cy_left   <= (others => '0');
            elsif v_h_edge = '1' then
                line0 <= '0';
                if idle_ln(1) = '1' then
                    frame_act <= '0';
                end if;
                v_ysum  := resize(cy_frac, 12) + s_ys;
                cy_frac <= v_ysum(7 downto 0);
                cy_left <= v_ysum(10 downto 8) - 1;
                cy_hit  <= '0';
                v_step  := '1';
            elsif cy_left /= 0 then
                cy_left <= cy_left - 1;
                v_step  := '1';
            end if;
            if v_step = '1' then
                if celly_loc = s_hm1 then
                    celly_loc <= (others => '0');
                    celly_par <= not celly_par;
                    band0     <= '0';
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;
            r_step <= v_step;

            -- Sample trigger: row R's colours are grabbed on the centre line of
            -- row R-1 (target parity = not current), at the target lattice's
            -- cell centres; row 0 is grabbed on the first active line.  A
            -- right-edge cell whose centre was never reached is grabbed from
            -- the last active pixel (held in w0_*) on the first blank clock.
            -- Hex lattice B's cell 0 is a half-cell centred ON the left edge:
            -- its centre is the first active pixel, which analog decoders
            -- deliver black (offset rows began with a black ball), so it is
            -- grabbed at its LAST pixel instead (~W/2 in, like lattice A).
            -- (row-level terms change only at h edges / the anchor, >= 6 clocks
            -- before any sample point, so they are registered to keep the
            -- trigger cone short; the w0 data hold is ANY avid fall — w0 is
            -- only consumed under w0_stb — so no trigger logic drives the
            -- 30-bit clock enable)
            v_tp := not celly_par;
            if line0 = '1' then v_tp := '0'; end if;
            tp_r <= v_tp;
            -- (virtual units: the centre line is the first line at/after the
            -- row centre, cy_hit; the centre pixel is the crossing of W/2 —
            -- at 1080p exactly the old loc = H/2 / loc = W/2 tests)
            if frame_act = '1' and (line0 = '1' or cy_hit = '1') then
                srow_r <= '1';
            else
                srow_r <= '0';
            end if;

            if tp_r = '0' then
                v_loc := xa_la(8 downto 1);  v_lt := xa_lt;
            else
                v_loc := xb_la(8 downto 1);  v_lt := xb_lt;
            end if;
            if v_loc >= s_wh then v_ge := '1'; else v_ge := '0'; end if;
            v_trig := '0';
            if srow_r = '1' then
                if data_in.avid = '1' then
                    if tp_r = '1' and xb_idx = 0 and s_square = '0' then
                        if v_bwrap = '1' then v_trig := '1'; end if;
                    elsif v_ge = '1' and v_lt = '1' then
                        v_trig := '1';
                    end if;
                elsif prev_avid = '1' then
                    -- the line ended before this cell's centre
                    if v_lt = '1' then
                        v_trig := '1';
                    end if;
                end if;
            end if;

            w0_stb <= v_trig;
            w0_par <= tp_r;
            if tp_r = '0' then
                w0_idx <= xa_idx;
            else
                w0_idx <= xb_idx;
            end if;
            if not (prev_avid = '1' and data_in.avid = '0') then
                w0_y <= unsigned(data_in.y);
                w0_u <= unsigned(data_in.u);
                w0_v <= unsigned(data_in.v);
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Sample write pipe: sat boost + optional luma->radius, then radius^2.
    --------------------------------------------------------------------------
    p_sample : process(clk)
        variable v_u, v_vv : signed(11 downto 0);
        variable v_rr      : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- W1: chroma gain + luma*r products (dedicated small mults)
            w1_pu  <= (signed(resize(w0_u, 11)) - to_signed(512, 11))
                    * signed(resize(s_sg, 9));
            w1_pv  <= (signed(resize(w0_v, 11)) - to_signed(512, 11))
                    * signed(resize(s_sg, 9));
            w1_pl  <= (signed(resize(w0_y, 11)) - to_signed(512, 11))
                    * signed(resize(s_r, 9));
            w1_stb <= w0_stb;
            w1_par <= w0_par;
            w1_idx <= w0_idx;
            -- luma stored black-referenced so shading darkens toward black 64
            if w0_y < 64 then
                w1_y <= (others => '0');
            else
                w1_y <= w0_y - 64;
            end if;

            -- W2: chroma offsets, clamped +/-1023 (11-bit signed; the output
            -- clamp after shading keeps hue on boosted, darkened cells)
            v_u := resize(shift_right(w1_pu, 6), 12);
            if v_u < -1023 then v_u := to_signed(-1023, 12); end if;
            if v_u > 1023 then v_u := to_signed(1023, 12); end if;
            v_vv := resize(shift_right(w1_pv, 6), 12);
            if v_vv < -1023 then v_vv := to_signed(-1023, 12); end if;
            if v_vv > 1023 then v_vv := to_signed(1023, 12); end if;

            if s_lsize = '1' then
                v_rr := signed(resize(s_r, 12)) + resize(shift_right(w1_pl, 10), 12);
                if v_rr < 3 then v_rr := to_signed(3, 12); end if;
                if v_rr > 180 then v_rr := to_signed(180, 12); end if;
            else
                v_rr := signed(resize(s_r, 12));
            end if;

            w2_u   <= v_u(10 downto 0);
            w2_v   <= v_vv(10 downto 0);
            w2_rr  <= unsigned(v_rr(7 downto 0));
            w2_stb <= w1_stb;
            w2_par <= w1_par;
            w2_idx <= w1_idx;
            w2_y   <= w1_y;

            -- W3: radius^2 (dedicated squarer)
            w3_rsq <= resize(w2_rr * w2_rr, 16);
            w3_stb <= w2_stb;
            w3_par <= w2_par;
            w3_idx <= w2_idx;
            w3_y   <= w2_y;
            w3_u   <= w2_u;
            w3_v   <= w2_v;
        end if;
    end process p_sample;

    --------------------------------------------------------------------------
    -- Cell buffers (canonical 1W1R, registered read, pre-muxed write word).
    --------------------------------------------------------------------------
    p_bufr0 : process(clk)
    begin
        if rising_edge(clk) then
            if w3_stb = '1' and w3_par = '0' then
                bufr0(to_integer(w3_idx)) <= line_par & std_logic_vector(w3_rsq(14 downto 0));
            end if;
            rd0 <= bufr0(to_integer(g0_a0));
        end if;
    end process p_bufr0;

    p_bufr1 : process(clk)
    begin
        if rising_edge(clk) then
            if w3_stb = '1' and w3_par = '1' then
                bufr1(to_integer(w3_idx)) <= line_par & std_logic_vector(w3_rsq(14 downto 0));
            end if;
            rd1 <= bufr1(to_integer(g0_a1));
        end if;
    end process p_bufr1;

    p_bufc : process(clk)
    begin
        if rising_edge(clk) then
            if w3_stb = '1' then
                bufc(to_integer(w3_par & w3_idx)) <= std_logic_vector(w3_y)
                    & std_logic_vector(w3_u) & std_logic_vector(w3_v);
            end if;
            rdc <= bufc(to_integer(ca_p(10)));
        end if;
    end process p_bufc;

    --------------------------------------------------------------------------
    -- Per-line maths: dy / (dy - oy) squared for both row candidates,
    -- updated incrementally (adds only) one cycle after each h edge.
    --------------------------------------------------------------------------
    p_line : process(clk)
        -- sq((d+1)) from sq(d) and the NEW d: sq + 2d_new - 1
        function upd(sq : unsigned(15 downto 0); dn : signed(8 downto 0))
            return unsigned is
            variable v : signed(17 downto 0);
        begin
            v := signed(resize(sq, 18)) + resize(dn & '0', 18) - 1;
            return unsigned(v(15 downto 0));
        end function;
        variable v_hh, v_h : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            if r_step = '1' or r_anchor = '1' then
                v_hh := signed(resize(s_hh, 9));
                v_h  := signed(resize(s_h, 9));
                if celly_loc = 0 then
                    -- band wrap (or frame anchor)
                    lm_dy1  <= -v_hh;
                    lm_dy2  <= v_hh;
                    lm_ey1  <= -v_hh - resize(s_oy, 9);
                    lm_ey2  <= v_hh - resize(s_oy, 9);
                    dy1sq_r <= hh2q;
                    dy2sq_r <= hh2q;
                    ey1sq_r <= eyw1sq;
                    ey2sq_r <= eyw2sq;
                elsif celly_loc = s_hh then
                    -- adjacent row flips from above to below: dy2 jumps to -H
                    lm_dy1  <= lm_dy1 + 1;
                    dy1sq_r <= upd(dy1sq_r, lm_dy1 + 1);
                    lm_ey1  <= lm_ey1 + 1;
                    ey1sq_r <= upd(ey1sq_r, lm_ey1 + 1);
                    lm_dy2  <= -v_h;
                    dy2sq_r <= hsq;
                    lm_ey2  <= -v_h - resize(s_oy, 9);
                    ey2sq_r <= eyj2sq;
                else
                    lm_dy1  <= lm_dy1 + 1;
                    dy1sq_r <= upd(dy1sq_r, lm_dy1 + 1);
                    lm_ey1  <= lm_ey1 + 1;
                    ey1sq_r <= upd(ey1sq_r, lm_ey1 + 1);
                    lm_dy2  <= lm_dy2 + 1;
                    dy2sq_r <= upd(dy2sq_r, lm_dy2 + 1);
                    lm_ey2  <= lm_ey2 + 1;
                    ey2sq_r <= upd(ey2sq_r, lm_ey2 + 1);
                end if;
            end if;
        end if;
    end process p_line;

    --------------------------------------------------------------------------
    -- Frame FSM: knob shaping, light vector, ranges, 2 serial divides.
    -- One shared 14x14 multiplier; everything settles inside the vsync cone.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_w   : integer range 0 to 255;
        variable v_e   : unsigned(8 downto 0);
        variable v_acc : unsigned(9 downto 0);

        -- '1' when a knob has moved >= ~4 codes from its latched value:
        -- geometry knobs only follow real turns, so ADC noise on a cell-size
        -- or radius step boundary cannot flicker the whole grid.
        function moved(reg : std_logic_vector; l : unsigned(9 downto 0))
            return std_logic is
            variable d : signed(10 downto 0);
        begin
            d := signed(resize(unsigned(reg(9 downto 0)), 11)) - signed(resize(l, 11));
            if d(10 downto 2) = "000000000" or d(10 downto 2) = "111111111" then
                return '0';
            end if;
            return '1';
        end function;
    begin
        if rising_edge(clk) then
            mp <= ma * mb;

            -- free-running deadband flags (kept out of the vsync cone)
            k1_mv <= moved(registers_in(0), lk1);
            k4_mv <= moved(registers_in(3), lk4);
            k5_mv <= moved(registers_in(4), lk5);

            -- serial restoring divider (18-bit dividend / 8-bit divisor)
            if div_run = '1' then
                v_acc := (div_rem & div_d(17)) - resize(div_v, 10);
                if v_acc(9) = '0' then
                    div_rem <= v_acc(8 downto 0);
                    div_q   <= div_q(16 downto 0) & '1';
                else
                    div_rem <= div_rem(7 downto 0) & div_d(17);
                    div_q   <= div_q(16 downto 0) & '0';
                end if;
                div_d <= div_d(16 downto 0) & '0';
                if div_cnt = 17 then
                    div_run <= '0';
                else
                    div_cnt <= div_cnt + 1;
                end if;
            end if;

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                if k1_mv = '1' then lk1 <= unsigned(registers_in(0)(9 downto 0)); end if;
                lk2  <= unsigned(registers_in(1)(9 downto 0));
                lk3  <= unsigned(registers_in(2)(9 downto 0));
                if k4_mv = '1' then lk4 <= unsigned(registers_in(3)(9 downto 0)); end if;
                if k5_mv = '1' then lk5 <= unsigned(registers_in(4)(9 downto 0)); end if;
                lk6  <= unsigned(registers_in(5)(9 downto 0));
                lp12 <= unsigned(registers_in(7)(9 downto 0));
                s_bgvid  <= registers_in(6)(0);
                s_lsize  <= registers_in(6)(1);
                s_square <= registers_in(6)(2);
                s_mono   <= registers_in(6)(3);
                s_bypass <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"FF" then
                fsm_t <= fsm_t + 1;

                case to_integer(fsm_t) is
                    when 0 =>
                        v_w := 12 + to_integer(lk1(9 downto 3));
                        if v_w > 128 then v_w := 128; end if;
                        s_w      <= to_unsigned(v_w, 8);
                        -- relief span 63..255: floored so light angle /
                        -- specular / shine still read at P12 = 0
                        s_sp     <= to_unsigned(63, 8) + lp12(9 downto 2)
                                  - ("00" & lp12(9 downto 4));
                        s_si     <= lk3(9 downto 1);
                        if s_mono = '1' then
                            s_sg <= "000" & lk6(9 downto 5);           -- tint 0..0.48x
                        else
                            s_sg <= resize(to_unsigned(48, 8) + lk6(9 downto 3), 8);
                        end if;
                        s_rfrac  <= resize(to_unsigned(676, 11) + lk5(9 downto 1)
                                         + lk5(9 downto 3), 11);
                        s_kshine <= to_unsigned(1088, 11) - resize(lk4, 11);
                    when 1 =>
                        s_amb <= 255 - s_sp;
                        s_wh  <= '0' & s_w(7 downto 1);
                        s_wlim <= (s_w & '0') - s_xs;                  -- 2W - x step
                        if s_square = '1' then
                            s_h   <= s_w;
                        else
                            s_h   <= s_w - ('0' & s_w(7 downto 3));       -- H = W*7/8
                        end if;
                        ma <= signed(resize(s_w, 12));
                        mb <= signed(resize(s_rfrac, 12));
                    when 2 =>
                        s_hm1 <= s_h - 1;
                        s_hh  <= '0' & s_h(7 downto 1);
                    when 4 =>
                        -- r = W*rfrac/2048, 4..82
                        if mp(18 downto 11) < 4 then
                            s_r <= to_unsigned(4, 8);
                        else
                            s_r <= unsigned(mp(18 downto 11));
                        end if;
                    when 5 =>
                        s_ls   <= ('0' & s_r(7 downto 1)) + ("000" & s_r(7 downto 3));
                        s_rim2 <= resize(s_r + ('0' & s_r(7 downto 1)), 10)
                                + ("0000" & s_r(7 downto 2));             -- 1.75r: ~0.9px AA ring
                        ma <= signed(resize(s_r, 12));
                        mb <= signed(resize(s_kshine, 12));
                        sinv <= lut_sin;
                        cosv <= lut_cos;
                    when 6 =>
                        -- AA ring stays ~0.9 REAL px: x1.5 at 720p, x2 at SD
                        if s_xs(2) = '1' then
                            s_rim2 <= s_rim2(8 downto 0) & '0';
                        elsif s_xs(0) = '1' then
                            s_rim2 <= s_rim2 + ('0' & s_rim2(9 downto 1));
                        end if;
                    when 8 =>
                        -- Es = 3 + r*(1088-K4)/1024
                        s_es <= resize(to_unsigned(3, 8) + unsigned(mp(17 downto 10)), 8);
                        ma <= signed(resize(s_ls, 12));
                        mb <= resize(cosv, 12);
                    when 11 =>
                        s_ox <= resize(shift_right(mp, 9), 8);
                        ma <= signed(resize(s_ls, 12));
                        mb <= resize(sinv, 12);
                    when 14 =>
                        s_oy <= resize(shift_right(mp, 9), 8);
                        v_e := resize(s_r, 9) + resize(s_ls, 9);
                        ma <= signed(resize(v_e, 12));
                        mb <= signed(resize(v_e, 12));
                    when 17 =>
                        s_e2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_es, 12));
                        mb <= signed(resize(s_es, 12));
                    when 20 =>
                        s_es2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_si, 12));
                        mb <= signed(resize(s_sp, 12));
                    when 23 =>
                        s_speff <= unsigned(mp(16 downto 7));
                    when 24 =>
                        s_sle <= to_unsigned(norm_shift(s_e2), 4);
                    when 25 =>
                        div_v   <= resize(shift_right(shift_left(resize(s_e2, 16),
                                          to_integer(s_sle)), 8), 8);
                        div_d   <= resize(s_sp, 18) sll 7;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 45 =>
                        if div_q > 255 then
                            s_kd8 <= (others => '1');
                        else
                            s_kd8 <= div_q(7 downto 0);
                        end if;
                        s_sls <= to_unsigned(norm_shift(s_es2), 4);
                    when 46 =>
                        div_v   <= resize(shift_right(shift_left(resize(s_es2, 16),
                                          to_integer(s_sls)), 8), 8);
                        div_d   <= resize(s_speff, 18) sll 7;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 66 =>
                        if div_q > 1023 then
                            s_ks9 <= (others => '1');
                        else
                            s_ks9 <= div_q(9 downto 0);
                        end if;
                        ma <= signed(resize(s_hh, 12));
                        mb <= signed(resize(s_hh, 12));
                    when 69 =>
                        hh2q <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_h, 12));
                        mb <= signed(resize(s_h, 12));
                    when 72 =>
                        hsq <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_hh, 12)) + resize(s_oy, 12);
                        mb <= signed(resize(s_hh, 12)) + resize(s_oy, 12);
                    when 75 =>
                        eyw1sq <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_hh, 12)) - resize(s_oy, 12);
                        mb <= signed(resize(s_hh, 12)) - resize(s_oy, 12);
                    when 78 =>
                        eyw2sq <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_h, 12)) + resize(s_oy, 12);
                        mb <= signed(resize(s_h, 12)) + resize(s_oy, 12);
                    when 81 =>
                        eyj2sq <= unsigned(mp(15 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    --------------------------------------------------------------------------
    -- Pixel pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_par    : std_logic;
        variable v_word   : t_rword;
        variable v_rsq    : unsigned(15 downto 0);
        variable v_rin    : signed(17 downto 0);
        variable v_dl     : signed(17 downto 0);
        variable v_ds     : signed(17 downto 0);
        variable v_l      : unsigned(9 downto 0);
        variable v_s      : unsigned(10 downto 0);
        variable v_cf     : signed(10 downto 0);
        variable v_yd     : unsigned(10 downto 0);
        variable v_yk     : unsigned(10 downto 0);
        variable v_su     : signed(11 downto 0);
        variable v_sy, v_uu, v_vv : unsigned(9 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_oy2, v_ou, v_ov : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            ------------------------------------------------------------------
            -- c1 (G0): local dx for both lattices, buffer addresses, top guard.
            -- Own row = parity's lattice; adjacent row = the other one.
            ------------------------------------------------------------------
            if celly_par = '0' then
                g0_dx1 <= signed(xa_la(8 downto 1)) - signed(s_wh);
                g0_dx2 <= signed(xb_la(8 downto 1)) - signed(s_wh);
            else
                g0_dx1 <= signed(xb_la(8 downto 1)) - signed(s_wh);
                g0_dx2 <= signed(xa_la(8 downto 1)) - signed(s_wh);
            end if;
            g0_a0 <= xa_idx;
            g0_a1 <= xb_idx;
            if band0 = '1' and celly_loc < s_hh then
                g0_noadj <= '1';
            else
                g0_noadj <= '0';
            end if;
            -- row 0's line is read while being written (undefined EBR data)
            g0_blank <= line0 or not frame_act;

            -- c2: dx squares (rsq reads land in rd0/rd1 this cycle too)
            q1 <= resize(unsigned(g0_dx1 * g0_dx1), 14);
            q2 <= resize(unsigned(g0_dx2 * g0_dx2), 14);
            p2_dx1   <= g0_dx1;
            p2_dx2   <= g0_dx2;
            p2_noadj <= g0_noadj;
            p2_blank <= g0_blank;
            p2_a0    <= g0_a0;
            p2_a1    <= g0_a1;

            -- c3: full squared distances to both candidate centres
            d1 <= resize(q1, 16) + resize(dy1sq_r, 16);
            if p2_noadj = '1' then
                d2 <= (others => '1');
            else
                d2 <= resize(q2, 16) + resize(dy2sq_r, 16);
            end if;
            p3_dx1   <= p2_dx1;
            p3_dx2   <= p2_dx2;
            p3_rd0   <= rd0;
            p3_rd1   <= rd1;
            p3_blank <= p2_blank;
            p3_a0    <= p2_a0;
            p3_a1    <= p2_a1;

            ------------------------------------------------------------------
            -- c4: Voronoi select.  Winning candidate's parity = row parity xor
            -- sel, which picks the rsq word and the colour address directly.
            ------------------------------------------------------------------
            if d2 < d1 then
                p4_dsq <= d2;
                p4_dx  <= p3_dx2;
                p4_eys <= ey2sq_r;
                v_par  := not celly_par;
            else
                p4_dsq <= d1;
                p4_dx  <= p3_dx1;
                p4_eys <= ey1sq_r;
                v_par  := celly_par;
            end if;
            if v_par = '1' then
                p4_word <= p3_rd1;
                p4_ca   <= '1' & p3_a1;
            else
                p4_word <= p3_rd0;
                p4_ca   <= '0' & p3_a0;
            end if;
            p4_blank <= p3_blank;

            -- c5: inside / core tests + light-relative dx
            v_word := p4_word;
            v_rsq  := "0" & unsigned(v_word(14 downto 0));
            v_rin  := signed(resize(v_rsq, 18)) - signed(resize(s_rim2, 18));
            if p4_dsq < v_rsq and p4_blank = '0' then
                p5_in <= '1';
            else
                p5_in <= '0';
            end if;
            if signed(resize(p4_dsq, 18)) < v_rin then
                p5_core <= '1';
            else
                p5_core <= '0';
            end if;
            p5_ex  <= p4_dx - s_ox;
            p5_eys <= p4_eys;

            -- c6: light-relative dx squared
            p6_exq <= resize(unsigned(p5_ex * p5_ex), 14);
            p6_eys <= p5_eys;

            -- c7: light-relative squared distance
            p7_e2 <= resize(p6_exq, 17) + resize(p6_eys, 17);

            -- c8: diffuse / specular head-room (clamped at 0)
            v_dl := signed(resize(s_e2, 18)) - signed(resize(p7_e2, 18));
            if v_dl < 0 then v_dl := (others => '0'); end if;
            p8_dl <= unsigned(v_dl(14 downto 0));
            v_ds := signed(resize(s_es2, 18)) - signed(resize(p7_e2, 18));
            if v_ds < 0 then v_ds := (others => '0'); end if;
            p8_ds <= unsigned(v_ds(13 downto 0));

            -- c9/c10: 2-stage barrel normalise (coarse 0/4/8/12, then fine 0..3)
            p9_dlc <= shift_left(resize(p8_dl, 16),
                                 4 * to_integer(s_sle(3 downto 2)));
            p9_dsc <= shift_left(resize(p8_ds, 16),
                                 4 * to_integer(s_sls(3 downto 2)));
            p10_dlb <= shift_left(p9_dlc, to_integer(s_sle(1 downto 0)))(15 downto 8);
            p10_dsb <= shift_left(p9_dsc, to_integer(s_sls(1 downto 0)))(15 downto 8);

            -- c11: normalise gains (winner colour read issued from ca_p(10))
            p11_pl <= p10_dlb * s_kd8;
            p11_ps <= p10_dsb * s_ks9;

            -- c12: diffuse level (ambient floor) + specular level; colour
            -- word re-registered in fabric before any arithmetic
            v_l := resize(s_amb, 10) + resize(p11_pl(15 downto 7), 10);
            if v_l > 255 then v_l := to_unsigned(255, 10); end if;
            p12_l <= v_l(7 downto 0);
            v_s := resize(p11_ps(17 downto 7), 11);
            if v_s > 511 then v_s := to_unsigned(511, 11); end if;
            p12_s <= v_s(8 downto 0);
            c12_y <= unsigned(rdc(31 downto 22));
            c12_u <= signed(rdc(21 downto 11));
            c12_v <= signed(rdc(10 downto 0));

            -- c13: Y * diffuse; chroma factor = diffuse minus spec desat;
            -- U/V swapped where the sample line's hsync->avid parity differs
            -- from this line's (the encoder pairs Cb/Cr from hsync)
            p13_ym <= c12_y * p12_l;
            v_cf := signed(resize(p12_l, 11)) - signed(resize(p12_s(8 downto 1), 11));
            if v_cf < 0 then v_cf := (others => '0'); end if;
            p13_cf <= unsigned(v_cf(7 downto 0));
            p13_s  <= p12_s;
            if (wpar_p(12) xor line_par) = '1' then
                c13_u <= c12_v;
                c13_v <= c12_u;
            else
                c13_u <= c12_u;
                c13_v <= c12_v;
            end if;

            -- c14: chroma * factor; Y + specular
            p14_um <= c13_u * signed(resize(p13_cf, 9));
            p14_vm <= c13_v * signed(resize(p13_cf, 9));
            p14_yd <= resize(p13_ym(17 downto 8), 11) + resize(p13_s, 11);

            -- c15: luma back onto black 64 with a soft knee (slope 1/4 above
            -- 704, capped at 800: full-scale Y clips RGB on hardware);
            -- chroma back about 512, clamped
            if p14_yd < 640 then
                v_yk := p14_yd + 64;
            else
                v_yk := to_unsigned(704, 11) + shift_right(p14_yd - 640, 2);
                if v_yk > 800 then v_yk := to_unsigned(800, 11); end if;
            end if;
            p15_y <= v_yk(9 downto 0);
            v_su := to_signed(512, 12) + resize(shift_right(p14_um, 8), 12);
            if v_su < 0 then v_su := (others => '0'); end if;
            if v_su > 1023 then v_su := to_signed(1023, 12); end if;
            p15_u <= unsigned(v_su(9 downto 0));
            v_su := to_signed(512, 12) + resize(shift_right(p14_vm, 8), 12);
            if v_su < 0 then v_su := (others => '0'); end if;
            if v_su > 1023 then v_su := to_signed(1023, 12); end if;
            p15_v <= unsigned(v_su(9 downto 0));

            ------------------------------------------------------------------
            -- c16: blanking gate / bypass / sphere over backdrop (rim row gets
            -- a 50% AA blend).
            ------------------------------------------------------------------
            v_sy := p15_y;  v_uu := p15_u;  v_vv := p15_v;

            if s_bgvid = '1' then
                v_bgy := resize(unsigned(pipe(14).y(9 downto 1)), 10) + 32;
                v_bgu := resize(unsigned(pipe(14).u(9 downto 1)), 10) + 256;
                v_bgv := resize(unsigned(pipe(14).v(9 downto 1)), 10) + 256;
            else
                v_bgy := to_unsigned(64, 10);
                v_bgu := C_MID;
                v_bgv := C_MID;
            end if;
            if s_mono = '1' then
                v_bgu := C_MID;
                v_bgv := C_MID;
            end if;

            if in_p(9) = '1' then
                if core_p(9) = '1' then
                    v_oy2 := v_sy;  v_ou := v_uu;  v_ov := v_vv;
                else
                    v_oy2 := ('0' & v_sy(9 downto 1)) + ('0' & v_bgy(9 downto 1));
                    v_ou  := ('0' & v_uu(9 downto 1)) + ('0' & v_bgu(9 downto 1));
                    v_ov  := ('0' & v_vv(9 downto 1)) + ('0' & v_bgv(9 downto 1));
                end if;
            else
                v_oy2 := v_bgy;  v_ou := v_bgu;  v_ov := v_bgv;
            end if;

            if pipe(14).avid = '0' then
                p16_y <= to_unsigned(64, 10);
                p16_u <= C_MID;
                p16_v <= C_MID;
            elsif s_bypass = '1' then
                p16_y <= unsigned(pipe(14).y);
                p16_u <= unsigned(pipe(14).u);
                p16_v <= unsigned(pipe(14).v);
            else
                p16_y <= v_oy2;
                p16_u <= v_ou;
                p16_v <= v_ov;
            end if;

            -- c17: sync attach
            s_io.y       <= std_logic_vector(p16_y);
            s_io.u       <= std_logic_vector(p16_u);
            s_io.v       <= std_logic_vector(p16_v);
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;

            -- flag / address pipes
            in_p(0)   <= p5_in;
            core_p(0) <= p5_core;
            for i in 1 to 9 loop
                in_p(i)   <= in_p(i - 1);
                core_p(i) <= core_p(i - 1);
            end loop;
            ca_p(5)   <= p4_ca;
            wpar_p(5) <= p4_word(15);
            for i in 6 to 10 loop ca_p(i) <= ca_p(i - 1); end loop;
            for i in 6 to 12 loop wpar_p(i) <= wpar_p(i - 1); end loop;
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
    -- latency changes (17 -> 20).
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

end architecture gumball;
