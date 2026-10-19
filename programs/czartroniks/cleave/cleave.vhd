-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- cleave.vhd  (v0.9 — "Cleave": dark/light split re-texturizer)
--
-- v0.10.3 (2026-10-17): resolution scaling.  Band, wobble, static-cell,
-- patch and outline geometry run in virtual 1080-line units stepped from
-- the video timing ID, so 720p and SD show the 1080p picture scaled by
-- screen height (SD = a 1440-unit 4:3 window).  Band/wobble steps per
-- pixel are x xs/2 and per field line x ys/256 (vblank serial multiply);
-- Medium/Coarse/Chunky static cells and the 128-unit patches key on the
-- virtual coordinates; the outline width is a per-mode table (1080p
-- 2/6 px, 720p 1/4, SD 1/3; floors of one pixel and one field line).
-- Fine static, the 1px band fuzz/dither hash, the serration and the line
-- buffer stay at real-pixel scale.  1080p output is bit-identical.
--
-- Full-screen two-texture split:
--
--   DARK  -> thermal scan-bands through a 16-entry UNEQUAL palette: white
--            and black bars exactly THREE TIMES the width of every color
--            stripe (whites straddle the cycle wrap at 15/0/1, blacks at
--            8/9/10, ten single-width colors between).  Bowed by a
--            triangle wobble, displaced by a high-frequency per-column
--            random serration, bowed by live luma relief, and fuzzed
--            per-pixel by the static field.
--   LIGHT -> TV STATIC: per-pixel (or 1/2/4/8 px chunk) random black/white,
--            density thresholded by inverted luma so the content reads
--            through the noise; boils per frame by default; optional
--            rainbow fringe tints it with the local band hue.
--   OUTLINE: black contour wherever the dark/light class flips —
--            horizontal via a class history, vertical via a {class,age}
--            ping-pong line buffer.  S9 selects Thin / Thick.
--
-- S7 Patchwork is a FULL EFFECT when on — every 128px patch gets: a large
-- vertical band-phase displacement (sync-tear tiles), a palette rotation
-- (bar positions shift per patch), band frequency doubling (~25%), a
-- strong static density swing (+/-64), per-patch static POLARITY
-- inversion (negative-static patches), and a serration boost.  Off =
-- genuinely uniform textures; the band rhythm still pulses the static
-- density (stripe-shaped, not a grid).
--
-- S8 "Contrast" applies a x2 centered luma stretch BEFORE posterization,
-- sharpening the class split, the band relief and the static density —
-- the incoming image defines the whole picture harder.
--
-- P12 "Flip" rotates the posterized class wheel: class = (level*128 + 64 +
-- wobble/4 + dither + flip) mod 1024, dark when < 512 (levels sit at slot
-- centres).  Mid-travel bands the luma extremes and statics the midtones;
-- full travel is the exact inversion.
--
-- Controls:
--   K1 Band Size (right = bigger; exponential, glided, zooms about the
--      screen centre)
--   K2 Wave (STAGED: grows the chevron, then stretches it — frequency
--      halves while amplitude climbs, full-swing 512 px swells at the end;
--      serration strength rides the same knob)
--   K3 Drift (BIDIRECTIONAL: centre stopped, left up, right down)
--   K4 Static Size K5 Boil (rate / manual pattern)  K6 Fringe
--      S11 Boil: K5 sets the boil RATE — the grain DISSOLVES cell by cell
--      from one random pattern into the next (~4 s simmer .. a fresh
--      pattern every frame).  S11 Still: K5 scrubs the grain by hand,
--      dissolving through 16 fixed patterns as the knob turns.
--   S7 Patchwork (Off/On: 128px patch tiles — see above)
--   S8 Contrast (Normal/Boost)   S9 Outline (Thin/Thick)
--   S10 Bands (Horizontal/Vertical)
--   S11 Grain (Still/Boil — STATIC LAYER ONLY: the band serration, band
--       fuzz and class boundary run on fixed seeds and never shimmer)
--   P12 Flip
--
-- The band fuzz and class-boundary dither run on a dedicated ALWAYS-1px
-- hash, decoupled from K4's static cell size — reusing the static cell
-- hash there put visible square "cubes" into the band layer.
--
-- 4 EBR (class+age line buffer), 0 multipliers.  Generated colors stored
-- U/V-swapped (stored U = Cr) per the HW-verified palette convention.
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

architecture cleave of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (13 + 3 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY : natural := 12;

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- 16-entry thermal cycle (10-bit YUV, stored U = Cr / V = Cb, already
    -- U/V-swapped; luma capped at 768).  White bars = slots 15+0+1, black
    -- bars = slots 8+9+10 (each exactly 3x a color stripe); colors 1 slot.
    -- Order per the user's reference strip: white -> (faint green) -> blue
    -- -> magenta -> purple -> indigo (darkest beside black) -> BLACK ->
    -- dark red -> red -> orange -> yellow -> (faint green-white) -> white.
    type t_pal16 is array (0 to 15) of unsigned(9 downto 0);
    constant PAL_Y : t_pal16 := (
        to_unsigned(768,10),   -- 0  white
        to_unsigned(768,10),   -- 1  white
        to_unsigned(730,10),   -- 2  pale mint (faint green hint)
        to_unsigned(538,10),   -- 3  light blue
        to_unsigned(164,10),   -- 4  blue
        to_unsigned(426,10),   -- 5  magenta
        to_unsigned(349,10),   -- 6  purple
        to_unsigned(192,10),   -- 7  indigo (darkest magenta, beside black)
        to_unsigned( 64,10),   -- 8  black
        to_unsigned( 64,10),   -- 9  black
        to_unsigned( 64,10),   -- 10 black
        to_unsigned(208,10),   -- 11 dark red
        to_unsigned(328,10),   -- 12 red
        to_unsigned(584,10),   -- 13 orange
        to_unsigned(768,10),   -- 14 yellow
        to_unsigned(768,10));  -- 15 near-white (faint green-yellow)
    constant PAL_U : t_pal16 := (
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(424,10),
        to_unsigned(342,10), to_unsigned(440,10), to_unsigned(887,10),
        to_unsigned(756,10), to_unsigned(608,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(758,10),
        to_unsigned(960,10), to_unsigned(772,10), to_unsigned(584,10),
        to_unsigned(480,10));
    constant PAL_V : t_pal16 := (
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(508,10),
        to_unsigned(744,10), to_unsigned(960,10), to_unsigned(809,10),
        to_unsigned(852,10), to_unsigned(696,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(429,10),
        to_unsigned(360,10), to_unsigned(212,10), to_unsigned( 64,10),
        to_unsigned(448,10));

    constant PSEED : unsigned(9 downto 0) := "0101010101";  -- patch seed

    -- 16 manual grain patterns (S11 Still, K5 scrubs through them).
    -- Chosen so each neighbouring pair's static fields share <=3.8% of
    -- values at any shift, so the dissolve between them reads as new grain.
    type t_seed16 is array (0 to 15) of unsigned(9 downto 0);
    constant SEEDTAB : t_seed16 := (
        "0110100101", "1110011110", "1110011101", "0110000100",
        "0101111010", "0101111101", "0011000000", "1001101101",
        "0100100010", "0010111001", "0001010101", "1100101011",
        "0000011110", "0010000001", "0001111001", "1010011100");
    -- The 1px hashes key on 10-bit px/py, which repeat every 1024 px/lines
    -- at 1080p (the right 896 px of Fine static was an exact copy of the
    -- left).  Bit 10 of each coordinate xors one of these into the seed
    -- ahead of the nonlinear rounds — checked: <=3% value match at any
    -- shift (random ~0.1%), no spatially coherent copy.
    constant C_HX  : unsigned(9 downto 0) := "1011001101";
    constant C_HY  : unsigned(9 downto 0) := "0110110011";
    constant C_JX  : unsigned(9 downto 0) := "1100101011";

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
    -- Outline width per mode: (8:3) = horizontal history mask (width in
    -- px), (2:0) = age reload (extra lines below the edge line).  1080p
    -- Thin = 2 px x 2 lines, Thick = 6 x 6 (virtual units); other modes
    -- round those to their pixel / field-line pitch, floored at 1 px and
    -- 1 line (Thick kept wider than Thin so S9 stays visible).
    function f_outline(t : std_logic_vector(3 downto 0); thick : std_logic)
        return std_logic_vector is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 =>            -- 480i / 576i: 2 units/px, 3.75-4.4/line
                if thick = '1' then return "000111" & "001";
                else                return "000001" & "000"; end if;
            when 4 | 12 =>           -- 480p / 576p: 1.9-2.25 units/line
                if thick = '1' then return "000111" & "010";
                else                return "000001" & "000"; end if;
            when 5 | 6 | 14 =>       -- 720p: 1.5 units/px and /line
                if thick = '1' then return "001111" & "011";
                else                return "000001" & "000"; end if;
            when 1 | 2 | 10 =>       -- 1080i: 2 units per field line
                if thick = '1' then return "111111" & "010";
                else                return "000011" & "000"; end if;
            when others =>           -- 1080p (the original widths)
                if thick = '1' then return "111111" & "101";
                else                return "000011" & "001"; end if;
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Raster position / accumulators.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal px          : unsigned(11 downto 0) := (others => '0');
    signal py          : unsigned(10 downto 0) := (others => '0');
    signal frame_act   : std_logic := '0';
    signal frame_armed : std_logic := '0';
    signal line_act    : std_logic := '0';

    signal lfsr   : unsigned(15 downto 0) := x"ACE1";
    -- static grain: two seeds + a per-cell dissolve fraction
    signal seed_a : unsigned(9 downto 0) := "0110100101";
    signal seed_b : unsigned(9 downto 0) := "1110011110";
    signal s_frac : unsigned(9 downto 0) := (others => '0');
    signal bacc   : unsigned(12 downto 0) := (others => '0');

    -- band phase: 10 integer + 8 fractional bits (continuous K1 zoom)
    signal bph_v : unsigned(17 downto 0) := (others => '0');
    signal bph_h : unsigned(17 downto 0) := (others => '0');
    signal bph0  : unsigned(17 downto 0) := (others => '0');
    -- wobble phase: wob_va Q.8 per field line, wob_ha Q.1 per pixel
    -- (integer parts = the old 10-bit wob_v / wob_h)
    signal wob_va : unsigned(17 downto 0) := (others => '0');
    signal wob_ha : unsigned(10 downto 0) := (others => '0');

    -- Resolution scaling (latched per field at the armed vsync): s_xs =
    -- virtual x units per pixel (Q.1), s_ys = virtual lines per field line
    -- (Q4.8).  vx_a / vy_a are the virtual raster position (integer parts
    -- equal px / py at 1080p).  s_hstep / s_lstep are the K1 band step per
    -- pixel / per field line, s_whstep (Q.1) / s_wlstep (Q.8) the wobble
    -- step per pixel / per field line.
    signal s_tid    : std_logic_vector(3 downto 0) := "0011";
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal vx_a     : unsigned(12 downto 0) := (others => '0');
    signal vy_a     : unsigned(18 downto 0) := (others => '0');
    signal s_hstep  : unsigned(13 downto 0) := to_unsigned(3584, 14);
    signal s_lstep  : unsigned(15 downto 0) := to_unsigned(3584, 16);
    signal s_whstep : unsigned(6 downto 0)  := to_unsigned(32, 7);
    signal s_wlstep : unsigned(16 downto 0) := to_unsigned(4096, 17);
    signal s_hmask  : std_logic_vector(5 downto 0) := "111111";
    signal s_age    : unsigned(2 downto 0) := "101";

    signal scroll_acc : unsigned(15 downto 0) := (others => '0');
    signal wobt       : unsigned(9 downto 0) := (others => '0');

    -- measured raster (active lines / pixels) for the centre-anchored zoom
    signal w_meas : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal h_meas : unsigned(10 downto 0) := to_unsigned(1080, 11);
    -- vblank serial multiplier, three products per field: s_lstep =
    -- vstep*ys/256, s_wlstep = wstep*ys, bph0 = scroll - step*(half screen)
    signal seq_cnt : unsigned(5 downto 0) := (others => '0');
    signal m_acc   : unsigned(23 downto 0) := (others => '0');
    signal m_cand  : unsigned(23 downto 0) := (others => '0');
    signal m_plr   : unsigned(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    signal rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0) := (others => '0');
    signal rsw : std_logic_vector(4 downto 0) := (others => '0');

    signal k1g      : unsigned(9 downto 0) := to_unsigned(213, 10);
    signal s_k1t    : unsigned(9 downto 0) := to_unsigned(608, 10);
    signal s_vstep  : unsigned(12 downto 0) := to_unsigned(3584, 13);  -- x/256
    signal k2g      : unsigned(9 downto 0) := to_unsigned(560, 10);
    signal s_wstep  : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_wamp   : integer range 0 to 6 := 3;
    signal s_wfrac  : unsigned(1 downto 0) := "00";
    signal s_jbase  : unsigned(1 downto 0) := "10";
    signal rd3s     : signed(10 downto 0) := (others => '0');
    signal s_spd    : unsigned(11 downto 0) := (others => '0');
    signal s_dneg   : std_logic := '0';
    signal s_gsel   : unsigned(1 downto 0) := "01";
    signal k5g      : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s_bstep  : unsigned(12 downto 0) := to_unsigned(8064, 13);
    signal s_frsel  : unsigned(2 downto 0) := (others => '0');
    signal s_flip   : unsigned(9 downto 0) := (others => '0');

    signal sw_patch, sw_boost, sw_thick, sw_vert, sw_boil : std_logic := '0';

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Class+age line buffer: {class, age(3)} ping-pong by line parity.
    --------------------------------------------------------------------------
    type t_lb is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal lb_a, lb_b   : t_lb := (others => (others => '0'));
    signal rd_a, rd_b   : std_logic_vector(3 downto 0) := (others => '0');
    signal rda_f, rdb_f : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s1: capture
    signal r1_l    : unsigned(9 downto 0) := (others => '0');
    signal r1_bph  : unsigned(9 downto 0) := (others => '0');
    signal r1_warg : unsigned(9 downto 0) := (others => '0');
    signal r1_jarg : unsigned(9 downto 0) := (others => '0');
    signal r1_cx, r1_cy : unsigned(9 downto 0) := (others => '0');
    signal r1_ppx  : unsigned(9 downto 0) := (others => '0');
    signal r1_ppy  : unsigned(9 downto 0) := (others => '0');
    signal r1_pyl  : unsigned(9 downto 0) := (others => '0');
    signal r1_px   : unsigned(11 downto 0) := (others => '0');
    signal r1_par  : std_logic := '0';
    signal r1_pok  : std_logic := '0';
    signal r1_top  : std_logic := '0';
    signal r1_shi, r1_qhi : std_logic_vector(1 downto 0) := (others => '0');
    signal r1_jhi  : std_logic := '0';
    -- s2: contrast boost, posterize, tri fold, base hash products
    signal r2_tri  : signed(9 downto 0) := (others => '0');
    signal r2_lev  : unsigned(2 downto 0) := (others => '0');
    signal r2_l    : unsigned(9 downto 0) := (others => '0');
    signal r2_bph  : unsigned(9 downto 0) := (others => '0');
    signal r2_ha, r2_hb : unsigned(9 downto 0) := (others => '0');
    signal r2_qa, r2_qb : unsigned(9 downto 0) := (others => '0');
    signal r2_jh   : unsigned(9 downto 0) := (others => '0');
    signal r2_pa, r2_pb : unsigned(9 downto 0) := (others => '0');
    signal r2_px   : unsigned(11 downto 0) := (others => '0');
    signal r2_par, r2_pok, r2_top : std_logic := '0';
    signal r2_shi, r2_qhi : std_logic_vector(1 downto 0) := (others => '0');
    -- s3: wobble sel, hash combine, 1D/patch rounds, relief
    signal r3_wob  : signed(9 downto 0) := (others => '0');
    signal r3_h1   : unsigned(9 downto 0) := (others => '0');
    signal r3_g1   : unsigned(9 downto 0) := (others => '0');  -- seed_b chain
    signal r3_q1   : unsigned(9 downto 0) := (others => '0');
    signal r3_jh2  : unsigned(9 downto 0) := (others => '0');
    signal r3_p1   : unsigned(9 downto 0) := (others => '0');
    signal r3_rel  : unsigned(7 downto 0) := (others => '0');
    signal r3_lev  : unsigned(2 downto 0) := (others => '0');
    signal r3_l    : unsigned(9 downto 0) := (others => '0');
    signal r3_bph  : unsigned(9 downto 0) := (others => '0');
    signal r3_px   : unsigned(11 downto 0) := (others => '0');
    signal r3_par, r3_pok, r3_top : std_logic := '0';
    -- s4: hash round B, band+wobble, jitter value, patch round, class base
    signal r4_h1b  : unsigned(9 downto 0) := (others => '0');
    signal r4_g1b  : unsigned(9 downto 0) := (others => '0');
    signal r4_q1b  : unsigned(9 downto 0) := (others => '0');
    signal r4_bph  : unsigned(9 downto 0) := (others => '0');
    signal r4_jsv  : signed(10 downto 0) := (others => '0');
    signal r4_p2   : unsigned(9 downto 0) := (others => '0');
    signal r4_c1   : signed(11 downto 0) := (others => '0');
    signal r4_rel  : unsigned(7 downto 0) := (others => '0');
    signal r4_l    : unsigned(9 downto 0) := (others => '0');
    signal r4_px   : unsigned(11 downto 0) := (others => '0');
    signal r4_par, r4_pok, r4_top : std_logic := '0';
    -- s5: hash round C, band+relief, patch final, class+dither
    signal r5_h2   : unsigned(9 downto 0) := (others => '0');
    signal r5_g2   : unsigned(9 downto 0) := (others => '0');
    signal r5_q2   : unsigned(9 downto 0) := (others => '0');
    signal r5_bph  : unsigned(9 downto 0) := (others => '0');
    signal r5_pf   : unsigned(9 downto 0) := (others => '0');
    signal r5_c2   : signed(11 downto 0) := (others => '0');
    signal r5_jsv  : signed(10 downto 0) := (others => '0');
    signal r5_l    : unsigned(9 downto 0) := (others => '0');
    signal r5_px   : unsigned(11 downto 0) := (others => '0');
    signal r5_par, r5_pok, r5_top : std_logic := '0';
    -- s6: static value final, scaled jitter, class+flip
    signal r6_hn   : unsigned(9 downto 0) := (others => '0');
    signal r6_gn   : unsigned(9 downto 0) := (others => '0');
    signal r6_qn   : unsigned(9 downto 0) := (others => '0');
    signal r6_bph  : unsigned(9 downto 0) := (others => '0');
    signal r6_jit  : signed(9 downto 0) := (others => '0');
    signal r6_pf   : unsigned(9 downto 0) := (others => '0');
    signal r6_class: unsigned(9 downto 0) := (others => '0');
    signal r6_l    : unsigned(9 downto 0) := (others => '0');
    signal r6_px   : unsigned(11 downto 0) := (others => '0');
    signal r6_par, r6_pok, r6_top : std_logic := '0';
    -- s7: band + serration, dark bit + h-edge, base ink threshold
    signal r7_hn   : unsigned(9 downto 0) := (others => '0');
    signal r7_qn   : unsigned(9 downto 0) := (others => '0');
    signal r7_bph  : unsigned(9 downto 0) := (others => '0');
    signal r7_dark : std_logic := '0';
    signal r7_hedge: std_logic := '0';
    signal r7_thr  : unsigned(7 downto 0) := (others => '0');
    signal r7_pf   : unsigned(9 downto 0) := (others => '0');
    signal r7_px   : unsigned(11 downto 0) := (others => '0');
    signal r7_par, r7_top : std_logic := '0';
    signal d_hist  : std_logic_vector(5 downto 0) := (others => '0');
    -- s8: static-fuzzed band phase, contour resolve
    signal r8_hn   : unsigned(9 downto 0) := (others => '0');
    signal r8_bph  : unsigned(9 downto 0) := (others => '0');
    signal r8_dark : std_logic := '0';
    signal r8_ctr  : std_logic := '0';
    signal r8_age  : unsigned(2 downto 0) := (others => '0');
    signal r8_thr  : unsigned(7 downto 0) := (others => '0');
    signal r8_pf   : unsigned(9 downto 0) := (others => '0');
    signal r8_px   : unsigned(11 downto 0) := (others => '0');
    signal r8_par  : std_logic := '0';
    -- s9: palette index, patch density mod, band pulse
    signal r9_hn   : unsigned(9 downto 0) := (others => '0');
    signal r9_idx  : unsigned(3 downto 0) := (others => '0');
    signal r9_thr  : unsigned(7 downto 0) := (others => '0');
    signal r9_bp   : signed(9 downto 0) := (others => '0');
    signal r9_dark, r9_ctr, r9_pol : std_logic := '0';
    -- s10: palette lookup, pulsed threshold
    signal r10_hn  : unsigned(9 downto 0) := (others => '0');
    signal r10_py  : unsigned(9 downto 0) := C_BLKY;
    signal r10_du, r10_dv : signed(9 downto 0) := (others => '0');
    signal r10_thr : unsigned(7 downto 0) := (others => '0');
    signal r10_dark, r10_ctr, r10_pol : std_logic := '0';
    -- s11: fringe shift
    signal r11_hn  : unsigned(9 downto 0) := (others => '0');
    signal r11_py  : unsigned(9 downto 0) := C_BLKY;
    signal r11_du, r11_dv : signed(9 downto 0) := (others => '0');
    signal r11_fu, r11_fv : signed(9 downto 0) := (others => '0');
    signal r11_thr : unsigned(7 downto 0) := (others => '0');
    signal r11_dark, r11_ctr, r11_pol : std_logic := '0';
    -- s12: ink decision, light luma, blanking flag
    signal r12_py  : unsigned(9 downto 0) := C_BLKY;
    signal r12_du, r12_dv : signed(9 downto 0) := (others => '0');
    signal r12_fu, r12_fv : signed(9 downto 0) := (others => '0');
    signal r12_ly  : unsigned(9 downto 0) := (others => '0');
    signal r12_dark, r12_ctr : std_logic := '0';
    signal r12_blank : std_logic := '1';

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_m12 : signed(11 downto 0);
        variable v_mag : unsigned(10 downto 0);
        variable v_u   : unsigned(9 downto 0);
        variable v_man : unsigned(7 downto 0);
        variable v_bm  : unsigned(5 downto 0);
        variable v_ol  : std_logic_vector(8 downto 0);
    begin
        if rising_edge(clk) then
            rk1  <= unsigned(registers_in(0)(9 downto 0));
            rk2  <= unsigned(registers_in(1)(9 downto 0));
            rk3  <= unsigned(registers_in(2)(9 downto 0));
            rk4  <= unsigned(registers_in(3)(9 downto 0));
            rk5  <= unsigned(registers_in(4)(9 downto 0));
            rk6  <= unsigned(registers_in(5)(9 downto 0));
            rsw  <= registers_in(6)(4 downto 0);
            rp12 <= unsigned(registers_in(7)(9 downto 0));

            -- K1 Band Size from the GLIDED k1g: right = BIGGER bands.
            -- Exponential over 6 octaves: t = (1023-k1g)*3/4, step =
            -- (128 + t(6:0)) << t(9:7), in 1/256 phase units per line/px
            -- -> 0.5 (knob max, 2048-line cycle) .. 32 (knob min).
            v_u   := not k1g;
            s_k1t <= v_u - shift_right(v_u, 2);
            v_man := '1' & s_k1t(6 downto 0);
            case to_integer(s_k1t(9 downto 7)) is
                when 0      => s_vstep <= resize(v_man, 13);
                when 1      => s_vstep <= shift_left(resize(v_man, 13), 1);
                when 2      => s_vstep <= shift_left(resize(v_man, 13), 2);
                when 3      => s_vstep <= shift_left(resize(v_man, 13), 3);
                when 4      => s_vstep <= shift_left(resize(v_man, 13), 4);
                when others => s_vstep <= shift_left(resize(v_man, 13), 5);
            end case;

            -- K2 staged wave, decoded from the GLIDED value k2g: base amp
            -- shift from the top 3 bits with a 2-bit mantissa (25% sub-
            -- steps), wavelength constant 64 px in the lower half and a
            -- CONTINUOUS unit-step stretch (step 16 down to 2) in the top
            -- half — every change is accumulator-phase-continuous.
            case to_integer(k2g(9 downto 7)) is
                when 0      => s_wamp <= 6;
                when 1      => s_wamp <= 5;
                when 2      => s_wamp <= 4;
                when 3      => s_wamp <= 3;
                when 4      => s_wamp <= 2;
                when 5      => s_wamp <= 1;
                when 6      => s_wamp <= 1;
                when others => s_wamp <= 0;
            end case;
            s_wfrac <= k2g(6 downto 5);
            if k2g(9) = '0' then
                s_wstep <= to_unsigned(16, 5);
            elsif k2g(8 downto 5) >= 14 then
                s_wstep <= to_unsigned(2, 5);
            else
                s_wstep <= to_unsigned(16, 5) - resize(k2g(8 downto 5), 5);
            end if;
            s_jbase  <= k2g(9 downto 8);

            -- Per-pixel band and wobble steps scaled by the x step (xs/2;
            -- at 1080p xs = 2 and these are the unscaled K1/K2 steps).  The
            -- per-line steps come from the vblank multiplier (p_position).
            case to_integer(s_xs) is
                when 3 =>
                    s_hstep  <= resize(s_vstep, 14) + resize(s_vstep(12 downto 1), 14);
                    s_whstep <= resize(s_wstep, 7) + shift_left(resize(s_wstep, 7), 1);
                when 4 =>
                    s_hstep  <= shift_left(resize(s_vstep, 14), 1);
                    s_whstep <= shift_left(resize(s_wstep, 7), 2);
                when others =>
                    s_hstep  <= resize(s_vstep, 14);
                    s_whstep <= shift_left(resize(s_wstep, 7), 1);
            end case;
            -- outline width for this mode and S9
            v_ol    := f_outline(s_tid, sw_thick);
            s_hmask <= v_ol(8 downto 3);
            s_age   <= unsigned(v_ol(2 downto 0));

            -- K3 bidirectional drift: centre = stopped (16-count deadband),
            -- right = down, left = up, speed grows toward the extremes.
            rd3s <= signed(resize(rk3, 11)) - to_signed(512, 11);
            v_m12 := abs(resize(rd3s, 12));
            v_mag := unsigned(v_m12(10 downto 0));
            s_dneg <= rd3s(10);
            if v_mag <= 16 then
                s_spd <= (others => '0');
            else
                s_spd <= resize(shift_left(v_mag - 16, 2), 12);
            end if;

            -- Labelled default: at program load the firmware writes the
            -- initial_value_label's INDEX (0..3) into the register, not a
            -- 0..1023 position, so a value below 4 is the label index itself.
            if rk4 < 4 then
                s_gsel <= rk4(1 downto 0);
            else
                s_gsel <= rk4(9 downto 8);
            end if;
            -- K5 boil rate, exponential over 8 octaves from the glided
            -- k5g: step = (32 + k5g(6:2)) << k5g(9:7) into a 13-bit
            -- accumulator -> one full dissolve per 256 frames .. ~1 frame
            v_bm := '1' & k5g(6 downto 2);
            case to_integer(k5g(9 downto 7)) is
                when 0      => s_bstep <= resize(v_bm, 13);
                when 1      => s_bstep <= shift_left(resize(v_bm, 13), 1);
                when 2      => s_bstep <= shift_left(resize(v_bm, 13), 2);
                when 3      => s_bstep <= shift_left(resize(v_bm, 13), 3);
                when 4      => s_bstep <= shift_left(resize(v_bm, 13), 4);
                when 5      => s_bstep <= shift_left(resize(v_bm, 13), 5);
                when 6      => s_bstep <= shift_left(resize(v_bm, 13), 6);
                when others => s_bstep <= shift_left(resize(v_bm, 13), 7);
            end case;
            s_frsel  <= rk6(9 downto 7);
            -- (rp12+1)/2 reaches 512 at full travel = the EXACT class
            -- complement (511 left the blackest level unflipped, flickering
            -- on the boundary at slider max)
            s_flip   <= resize(shift_right(resize(rp12, 11) + 1, 1), 10);

            sw_patch <= rsw(0);   -- S7  Patchwork (freq-doubling) Off/On
            sw_boost <= rsw(1);   -- S8  Contrast Normal/Boost
            sw_thick <= rsw(2);   -- S9  Outline Thin/Thick
            sw_vert  <= rsw(3);   -- S10
            sw_boil  <= rsw(4);   -- S11
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Raster counters, accumulators, per-frame housekeeping.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_kd, v_ks, v_kn : signed(11 downto 0);
        variable v_b0 : unsigned(17 downto 0);
        variable v_bs : unsigned(13 downto 0);
        variable v_si : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- free-running 16-bit Galois LFSR
            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- frame anchor, guarded against serrated analog vsync
            if v_v_edge = '1' and frame_armed = '1' then
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                vy_a        <= (others => '0');
                seq_cnt     <= to_unsigned(1, 6);   -- steps + bph0 via vblank mult
                wob_va      <= wobt & x"00";
                -- per-mode virtual steps for this field (field_n is tied
                -- high on hardware: no bottom-field offset)
                s_tid <= registers_in(8)(3 downto 0);
                s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
                if s_dneg = '1' then
                    scroll_acc <= scroll_acc - resize(s_spd, 16);
                else
                    scroll_acc <= scroll_acc + resize(s_spd, 16);
                end if;
                wobt <= wobt + 2;

                -- K2 glide: ease toward the pot by diff/8 per frame, with
                -- a +/-1 nudge so it always reaches the target (no snap)
                v_kd := signed(resize(rk2, 12)) - signed(resize(k2g, 12));
                v_ks := shift_right(v_kd, 3);
                if v_ks = 0 and v_kd > 0 then
                    v_ks := to_signed(1, 12);
                elsif v_ks = 0 and v_kd < 0 then
                    v_ks := to_signed(-1, 12);
                end if;
                v_kn := signed(resize(k2g, 12)) + v_ks;
                k2g  <= unsigned(v_kn(9 downto 0));

                -- K1 glide: same ease, but with a +/-2 deadband so pot
                -- noise can't toggle the band pitch (one step of pitch
                -- shifts the screen edges by a whole colour slot)
                v_kd := signed(resize(rk1, 12)) - signed(resize(k1g, 12));
                if v_kd > 2 or v_kd < -2 then
                    v_ks := shift_right(v_kd, 3);
                    if v_ks = 0 and v_kd > 0 then
                        v_ks := to_signed(1, 12);
                    elsif v_ks = 0 and v_kd < 0 then
                        v_ks := to_signed(-1, 12);
                    end if;
                    v_kn := signed(resize(k1g, 12)) + v_ks;
                    k1g  <= unsigned(v_kn(9 downto 0));
                end if;

                -- K5 glide, +/-2 deadband (in Still mode one count moves the
                -- dissolve by 16/1024 — pot noise must not sparkle the grain)
                v_kd := signed(resize(rk5, 12)) - signed(resize(k5g, 12));
                if v_kd > 2 or v_kd < -2 then
                    v_ks := shift_right(v_kd, 3);
                    if v_ks = 0 and v_kd > 0 then
                        v_ks := to_signed(1, 12);
                    elsif v_ks = 0 and v_kd < 0 then
                        v_ks := to_signed(-1, 12);
                    end if;
                    v_kn := signed(resize(k5g, 12)) + v_ks;
                    k5g  <= unsigned(v_kn(9 downto 0));
                end if;

                -- Static grain: each cell shows seed_a's pattern until the
                -- dissolve fraction passes its own mask value, then seed_b's.
                if sw_boil = '1' then
                    -- Boil: the fraction ramps at the K5 rate; on wrap the
                    -- new pattern becomes the old one and a fresh one queues
                    v_bs := resize(bacc, 14) + resize(s_bstep, 14);
                    bacc <= v_bs(12 downto 0);
                    if v_bs(13) = '1' then
                        seed_a <= seed_b;
                        seed_b <= lfsr(9 downto 0);
                    end if;
                    s_frac <= v_bs(12 downto 3);
                else
                    -- Still: K5 position = pattern index + dissolve fraction
                    v_si   := k5g(9 downto 6);
                    seed_a <= SEEDTAB(to_integer(v_si));
                    seed_b <= SEEDTAB(to_integer(v_si + 1));
                    s_frac <= k5g(5 downto 0) & "0000";
                end if;
            elsif v_h_edge = '1' then
                px     <= (others => '0');
                vx_a   <= (others => '0');
                bph_h  <= bph0;
                wob_ha <= wobt & '0';
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    vy_a     <= vy_a + s_ys;
                    w_meas   <= px;          -- measured on active lines only
                    h_meas   <= py + 1;
                    bph_v    <= bph_v + resize(s_lstep, 18);
                    wob_va   <= wob_va + resize(s_wlstep, 18);
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                line_act    <= '1';
                px     <= px + 1;
                vx_a   <= vx_a + s_xs;
                bph_h  <= bph_h + resize(s_hstep, 18);
                wob_ha <= wob_ha + resize(s_whstep, 11);
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                    vy_a      <= (others => '0');
                end if;
            end if;

            -- Vblank serial multiplier (starts at the armed vsync, done in
            -- ~45 clocks, long before the first active line), three
            -- products, 12 shift-add steps each:
            --   A: s_lstep  = vstep * ys / 256  (K1 band step per field line)
            --   B: s_wlstep = wstep * ys        (wobble step per field line, Q.8)
            --   C: bph0 = scroll - step * (H/2, or W/2 for vertical bands):
            --      the band phase is anchored at the screen CENTRE, so K1
            --      zooms the bands about the middle.  (mod 2^18 phase)
            -- At 1080p ys = 256: s_lstep = vstep and s_wlstep = wstep << 8.
            if seq_cnt /= 0 then
                seq_cnt <= seq_cnt + 1;
                if seq_cnt = 6 then        -- s_vstep has settled from k1g
                    m_acc  <= (others => '0');
                    m_cand <= resize(s_vstep, 24);
                    m_plr  <= s_ys;
                elsif seq_cnt = 19 then
                    s_lstep <= m_acc(23 downto 8);
                    m_acc  <= (others => '0');
                    m_cand <= resize(s_ys, 24);
                    m_plr  <= resize(s_wstep, 12);
                elsif seq_cnt = 32 then
                    s_wlstep <= m_acc(16 downto 0);
                    m_acc  <= (others => '0');
                    if sw_vert = '1' then
                        m_cand <= resize(s_hstep, 24);
                        m_plr  <= resize(w_meas(11 downto 1), 12);
                    else
                        m_cand <= resize(s_lstep, 24);
                        m_plr  <= resize(h_meas(10 downto 1), 12);
                    end if;
                elsif seq_cnt = 45 then
                    v_b0    := (scroll_acc(14 downto 0) & "000") - m_acc(17 downto 0);
                    bph0    <= v_b0;
                    bph_v   <= v_b0;
                    seq_cnt <= (others => '0');
                else                       -- 7..18, 20..31, 33..44
                    if m_plr(0) = '1' then
                        m_acc <= m_acc + m_cand;
                    end if;
                    m_cand <= shift_left(m_cand, 1);
                    m_plr  <= shift_right(m_plr, 1);
                end if;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_lsub : signed(11 downto 0);
        variable v_lb   : signed(11 downto 0);
        variable v_t9   : unsigned(8 downto 0);
        variable v_wob  : signed(9 downto 0);
        variable v_sum  : signed(11 downto 0);
        variable v_jf   : unsigned(9 downto 0);
        variable v_hr   : unsigned(9 downto 0);
        variable v_p3   : unsigned(9 downto 0);
        variable v_dith : signed(11 downto 0);
        variable v_cls  : signed(11 downto 0);
        variable v_jsel : unsigned(2 downto 0);
        variable v_dark : std_logic;
        variable v_wrp  : signed(11 downto 0);
        variable v_prvc : std_logic;
        variable v_agep : unsigned(2 downto 0);
        variable v_edge : std_logic;
        variable v_bt9  : unsigned(8 downto 0);
        variable v_thrx : signed(9 downto 0);
        variable v_idx4 : unsigned(3 downto 0);
        variable v_fu   : signed(9 downto 0);
        variable v_fv   : signed(9 downto 0);
        variable v_sx   : unsigned(9 downto 0);
        variable v_lc   : unsigned(9 downto 0);
        variable v_dk6  : std_logic_vector(5 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -----------------------------------------------------------------
            -- s1: capture — luma clamp, phase/wobble/jitter args, static
            -- chunk coords, patch coords.
            -----------------------------------------------------------------
            v_lsub := signed(resize(unsigned(data_in.y), 12)) - 64;
            if v_lsub < 0 then
                r1_l <= (others => '0');
            else
                r1_l <= unsigned(v_lsub(9 downto 0));
            end if;

            -- (serration jitter keys on the real column / field line: a
            -- pixel-level texture)
            if sw_vert = '1' then
                r1_bph  <= bph_h(17 downto 8);
                r1_warg <= wob_va(17 downto 8);
                r1_jarg <= py(9 downto 0);
                r1_jhi  <= py(10);
            else
                r1_bph  <= bph_v(17 downto 8);
                r1_warg <= wob_ha(10 downto 1);
                r1_jarg <= px(9 downto 0);
                r1_jhi  <= px(10);
            end if;

            -- Static cells: Fine is a 1-pixel grain at every resolution;
            -- Medium/Coarse/Chunky are 2/4/8-unit cells on the virtual
            -- raster (1080p px; 720p x2/3; SD 1-4 px x ~1-2 field lines).
            r1_shi <= "00";
            case to_integer(s_gsel) is
                when 0 =>      -- 1 px static (bit 10 goes to the seed)
                    r1_cx  <= px(9 downto 0);
                    r1_cy  <= resize(py(9 downto 0), 10);
                    r1_shi <= px(10) & py(10);
                when 1 =>      -- 2 units
                    r1_cx <= vx_a(11 downto 2);
                    r1_cy <= resize(vy_a(18 downto 9), 10);
                when 2 =>      -- 4 units
                    r1_cx <= vx_a(12 downto 3);
                    r1_cy <= resize(vy_a(18 downto 10), 10);
                when others => -- 8 units
                    r1_cx <= resize(vx_a(12 downto 4), 10);
                    r1_cy <= resize(vy_a(18 downto 11), 10);
            end case;

            -- 128-unit patches on the virtual raster
            r1_ppx <= resize(vx_a(12 downto 8), 10);
            r1_ppy <= resize(vy_a(18 downto 15), 10);
            r1_pyl <= resize(py(9 downto 0), 10);
            r1_px  <= px;
            r1_par <= py(0);
            r1_qhi <= px(10) & py(10);
            if px >= 8 and py /= 0 then r1_pok <= '1'; else r1_pok <= '0'; end if;
            -- first line: the line buffer still holds the previous frame's
            -- LAST line — no vertical edge / age carry-over on line 0
            if py = 0 then r1_top <= '1'; else r1_top <= '0'; end if;

            -----------------------------------------------------------------
            -- s2: contrast boost (x2 centered stretch), posterize, triangle
            -- fold, base hash products.
            -----------------------------------------------------------------
            if sw_boost = '1' then
                v_lb := shift_left(signed(resize(r1_l, 12)), 1)
                      - to_signed(512, 12);
                if v_lb < 0 then
                    r2_l   <= (others => '0');
                    r2_lev <= (others => '0');
                elsif v_lb > 1023 then
                    r2_l   <= (others => '1');
                    r2_lev <= (others => '1');
                else
                    r2_l   <= unsigned(v_lb(9 downto 0));
                    r2_lev <= unsigned(v_lb(9 downto 7));
                end if;
            else
                r2_l   <= r1_l;
                r2_lev <= r1_l(9 downto 7);
            end if;

            v_t9 := r1_warg(8 downto 0);
            if r1_warg(9) = '1' then v_t9 := not v_t9; end if;
            r2_tri <= signed(resize(v_t9, 10)) - to_signed(256, 10);

            r2_bph <= r1_bph;

            r2_ha <= r1_cx + shift_left(r1_cx, 3);                  -- cx*9
            r2_hb <= r1_cy + shift_left(r1_cy, 2);                  -- cy*5
            -- dedicated ALWAYS-1px hash (band fuzz + class dither): keeps
            -- those free of K4's static cell size (no cube artifacts).
            -- FIXED seeds here and on the serration — only the static
            -- layer's hash boils (S11 must never shimmer the bands).
            r2_qa <= r1_px(9 downto 0) + shift_left(r1_px(9 downto 0), 3);
            r2_qb <= r1_pyl + shift_left(r1_pyl, 2);
            if r1_jhi = '1' then v_sx := C_JX; else v_sx := (others => '0'); end if;
            r2_jh <= (r1_jarg + shift_left(r1_jarg, 3))
                   xor "0011010110" xor v_sx;                       -- 1D col
            r2_pa <= r1_ppx + shift_left(r1_ppx, 3);                -- patch
            r2_pb <= r1_ppy + shift_left(r1_ppy, 2);

            r2_px <= r1_px; r2_par <= r1_par; r2_pok <= r1_pok; r2_top <= r1_top;
            r2_shi <= r1_shi; r2_qhi <= r1_qhi;

            -----------------------------------------------------------------
            -- s3: wobble amp (case), hash combine + seed, 1D/patch rounds.
            -----------------------------------------------------------------
            case s_wamp is
                when 6      => v_wob := shift_right(r2_tri, 6);
                when 5      => v_wob := shift_right(r2_tri, 5);
                when 4      => v_wob := shift_right(r2_tri, 4);
                when 3      => v_wob := shift_right(r2_tri, 3);
                when 2      => v_wob := shift_right(r2_tri, 2);
                when 1      => v_wob := shift_right(r2_tri, 1);
                when others => v_wob := r2_tri;
            end case;
            -- 2-bit mantissa: x1.0 / x1.25 / x1.5 / x1.75 between stages
            if s_wfrac(1) = '1' then
                v_wob := v_wob + shift_right(v_wob, 1);
            end if;
            if s_wfrac(0) = '1' then
                v_wob := v_wob + shift_right(v_wob, 2);
            end if;
            r3_wob <= v_wob;

            v_sx := (others => '0');
            if r2_shi(1) = '1' then v_sx := v_sx xor C_HX; end if;
            if r2_shi(0) = '1' then v_sx := v_sx xor C_HY; end if;
            r3_h1  <= (r2_ha xor rotate_left(r2_hb, 5)) xor seed_a xor v_sx;
            r3_g1  <= (r2_ha xor rotate_left(r2_hb, 5)) xor seed_b xor v_sx;
            v_sx := (others => '0');
            if r2_qhi(1) = '1' then v_sx := v_sx xor C_HX; end if;
            if r2_qhi(0) = '1' then v_sx := v_sx xor C_HY; end if;
            r3_q1  <= (r2_qa xor rotate_left(r2_qb, 5)) xor "1010011100" xor v_sx;
            r3_jh2 <= (r2_jh + shift_left(r2_jh, 2)) + to_unsigned(341, 10);
            r3_p1  <= (r2_pa xor rotate_left(r2_pb, 5)) xor PSEED;
            r3_rel <= r2_l(9 downto 2);
            r3_lev <= r2_lev;
            r3_l   <= r2_l;
            r3_bph <= r2_bph;
            r3_px <= r2_px; r3_par <= r2_par; r3_pok <= r2_pok; r3_top <= r2_top;

            -----------------------------------------------------------------
            -- s4: hash round B, band+wobble, jitter value, patch round B,
            -- class base.
            -----------------------------------------------------------------
            v_hr := r3_h1 + shift_left(r3_h1, 2);
            r4_h1b <= v_hr xor shift_right(v_hr, 4);
            v_hr := r3_g1 + shift_left(r3_g1, 2);
            r4_g1b <= v_hr xor shift_right(v_hr, 4);
            v_hr := r3_q1 + shift_left(r3_q1, 2);
            r4_q1b <= v_hr xor shift_right(v_hr, 4);

            v_sum  := signed(resize(r3_bph, 12)) + resize(r3_wob, 12);
            r4_bph <= unsigned(v_sum(9 downto 0));

            v_jf   := r3_jh2 xor shift_right(r3_jh2, 4);
            r4_jsv <= signed(resize(v_jf, 11)) - to_signed(512, 11);

            r4_p2  <= (r3_p1 + shift_left(r3_p1, 2))
                    xor shift_right(r3_p1 + shift_left(r3_p1, 2), 4);

            -- each level sits at the CENTRE of its 128-wide class slot
            -- (lev*128 + 64): with levels on the slot edges the +/-32 dither
            -- split levels 0 and 4 pixel-by-pixel at flip 0 and max, the
            -- outline fired everywhere and they rendered solid black —
            -- the real cause of "black doesn't flip"
            v_lc   := r3_lev & "1000000";
            r4_c1  <= signed(resize(v_lc, 12))
                    + resize(shift_right(r3_wob, 2), 12);

            r4_rel <= r3_rel;
            r4_l   <= r3_l;
            r4_px <= r3_px; r4_par <= r3_par; r4_pok <= r3_pok; r4_top <= r3_top;

            -----------------------------------------------------------------
            -- s5: hash round C, band+relief, patch final, class + dither.
            -----------------------------------------------------------------
            r5_h2  <= (r4_h1b + shift_left(r4_h1b, 2)) + to_unsigned(341, 10);
            r5_g2  <= (r4_g1b + shift_left(r4_g1b, 2)) + to_unsigned(341, 10);
            r5_q2  <= (r4_q1b + shift_left(r4_q1b, 2)) + to_unsigned(341, 10);

            r5_bph <= r4_bph + resize(r4_rel, 10);

            v_p3  := (r4_p2 + shift_left(r4_p2, 2)) + to_unsigned(341, 10);
            r5_pf <= v_p3 xor shift_right(v_p3, 4);

            -- class dither from the 1px hash (K4-independent boundary)
            v_dith := shift_left(signed(resize(r4_q1b(3 downto 0), 12)), 2)
                    - to_signed(32, 12);
            r5_c2  <= r4_c1 + v_dith;

            r5_jsv <= r4_jsv;
            r5_l   <= r4_l;
            r5_px <= r4_px; r5_par <= r4_par; r5_pok <= r4_pok; r5_top <= r4_top;

            -----------------------------------------------------------------
            -- s6: final static value, serration jitter (K2 + patch scaled),
            -- class + flip.  Also issue the class line-buffer read.
            -----------------------------------------------------------------
            r6_hn <= r5_h2 xor shift_right(r5_h2, 4);
            r6_gn <= r5_g2 xor shift_right(r5_g2, 4);
            r6_qn <= r5_q2 xor shift_right(r5_q2, 4);

            -- serration: K2 sets +/-32..+/-192 phase units (a colour slot
            -- is 64 — the old +/-8 floor was invisible); per-patch
            -- boost/cut when Patchwork is on (+/-16..+/-256)
            if sw_patch = '1' then
                v_jsel := resize(s_jbase, 3)
                        + resize(r5_pf(1 downto 0), 3) + 1;
            else
                v_jsel := resize(s_jbase, 3) + 2;
            end if;
            case to_integer(v_jsel) is
                when 1      => r6_jit <= resize(shift_right(r5_jsv, 5), 10);
                when 2      => r6_jit <= resize(shift_right(r5_jsv, 4), 10);
                when 3      => r6_jit <= resize(shift_right(r5_jsv, 3), 10);
                when 4      => r6_jit <= resize(shift_right(r5_jsv, 2), 10);
                when 5      => r6_jit <= resize(shift_right(r5_jsv, 2), 10)
                                       + resize(shift_right(r5_jsv, 3), 10);
                when others => r6_jit <= resize(shift_right(r5_jsv, 1), 10);
            end case;

            v_cls    := r5_c2 + signed(resize(s_flip, 12));
            r6_class <= unsigned(v_cls(9 downto 0));

            -- Patchwork: large per-patch band-phase displacement (tiles of
            -- bands tear vertically like lost sync)
            if sw_patch = '1' then
                r6_bph <= r5_bph + shift_left(resize(r5_pf(7 downto 4), 10), 4);
            else
                r6_bph <= r5_bph;
            end if;
            r6_pf  <= r5_pf;
            r6_l   <= r5_l;
            r6_px <= r5_px; r6_par <= r5_par; r6_pok <= r5_pok; r6_top <= r5_top;

            -- class line-buffer read (both banks, unconditional)
            rd_a <= lb_a(to_integer(r5_px(10 downto 0)));
            rd_b <= lb_b(to_integer(r5_px(10 downto 0)));

            -----------------------------------------------------------------
            -- s7: band + serration, dark bit + h-edge, base ink threshold;
            -- fabric re-register of LB reads.
            -----------------------------------------------------------------
            v_sum  := signed(resize(r6_bph, 12)) + resize(r6_jit, 12);
            r7_bph <= unsigned(v_sum(9 downto 0));

            v_dark  := not r6_class(9);
            r7_dark <= v_dark;
            d_hist  <= d_hist(4 downto 0) & v_dark;
            -- horizontal outline: class differs from any of the last N
            -- pixels, N = the per-mode / S9 width (s_hmask)
            v_dk6 := (others => v_dark);
            if ((v_dk6 xor d_hist) and s_hmask) /= "000000" then
                r7_hedge <= r6_pok;
            else
                r7_hedge <= '0';
            end if;

            -- ink threshold: density bias + inverted luma (content term)
            -- (fixed density bias = the old K5 Ink default)
            r7_thr <= to_unsigned(48, 8) + resize(not r6_l(9 downto 3), 8);

            rda_f <= rd_a;
            rdb_f <= rd_b;

            -- grain dissolve: mask = hn xor gn (uniform and independent of
            -- the chosen value, so ink density never shifts mid-dissolve)
            if (r6_hn xor r6_gn) < s_frac then
                r7_hn <= r6_gn;
            else
                r7_hn <= r6_hn;
            end if;
            r7_qn <= r6_qn;
            r7_pf <= r6_pf;
            r7_px <= r6_px; r7_par <= r6_par; r7_top <= r6_top;

            -----------------------------------------------------------------
            -- s8: static fuzz on the band phase (cross-mod static->bands),
            -- contour resolve (h-edge + v-edge + age).
            -----------------------------------------------------------------
            -- per-pixel band fuzz from the 1px hash (never cell-blocky)
            v_wrp  := shift_right(signed(resize(r7_qn, 12))
                                  - to_signed(512, 12), 5);        -- +/-16
            r8_bph <= r7_bph + unsigned(v_wrp(9 downto 0));

            if r7_par = '0' then
                v_prvc := rdb_f(3);
                v_agep := unsigned(rdb_f(2 downto 0));
            else
                v_prvc := rda_f(3);
                v_agep := unsigned(rda_f(2 downto 0));
            end if;
            if r7_top = '1' then
                v_prvc := r7_dark;
                v_agep := (others => '0');
            end if;
            v_edge := r7_hedge or (v_prvc xor r7_dark);
            if v_edge = '1' or v_agep /= 0 then
                r8_ctr <= '1';
            else
                r8_ctr <= '0';
            end if;
            if v_edge = '1' then
                r8_age <= s_age;           -- per-mode / S9 outline depth
            elsif v_agep /= 0 then
                r8_age <= v_agep - 1;
            else
                r8_age <= (others => '0');
            end if;

            r8_hn   <= r7_hn;
            r8_dark <= r7_dark;
            r8_thr  <= r7_thr;
            r8_pf   <= r7_pf;
            r8_px <= r7_px; r8_par <= r7_par;

            -----------------------------------------------------------------
            -- s9: palette index (patch frequency doubling), patch density
            -- mod, band pulse (cross-mod bands->static).  LB write.
            -----------------------------------------------------------------
            -- S7 Patchwork: ~25% of patches at double band freq, plus a
            -- per-patch palette rotation (bars shift position per patch)
            if sw_patch = '1' and r8_pf(3 downto 2) = "11" then
                v_idx4 := r8_bph(8 downto 5);
            else
                v_idx4 := r8_bph(9 downto 6);
            end if;
            if sw_patch = '1' then
                v_idx4 := v_idx4 + resize(r8_pf(1 downto 0), 4);
            end if;
            r9_idx <= v_idx4;

            -- per-patch static density swing (strong) when Patchwork is on
            if sw_patch = '1' then
                case to_integer(r8_pf(5 downto 4)) is
                    when 0      => v_thrx := signed(resize(r8_thr, 10)) - 64;
                    when 1      => v_thrx := signed(resize(r8_thr, 10)) - 24;
                    when 2      => v_thrx := signed(resize(r8_thr, 10)) + 24;
                    when others => v_thrx := signed(resize(r8_thr, 10)) + 64;
                end case;
            else
                v_thrx := signed(resize(r8_thr, 10));
            end if;
            -- per-patch static polarity: some patches render negative
            r9_pol <= sw_patch and r8_pf(6);
            if v_thrx < 0 then
                r9_thr <= (others => '0');
            elsif v_thrx > 255 then
                r9_thr <= (others => '1');
            else
                r9_thr <= unsigned(v_thrx(7 downto 0));
            end if;

            v_bt9 := r8_bph(8 downto 0);
            if r8_bph(9) = '1' then v_bt9 := not v_bt9; end if;
            r9_bp <= resize(shift_right(signed(resize(v_bt9, 10))
                                        - to_signed(256, 10), 4), 10);

            r9_hn   <= r8_hn;
            r9_dark <= r8_dark;
            r9_ctr  <= r8_ctr;

            -- line-buffer write: single write per bank, aligned-avid gated
            if pipe(7).avid = '1' then
                if r8_par = '0' then
                    lb_a(to_integer(r8_px(10 downto 0)))
                        <= r8_dark & std_logic_vector(r8_age);
                else
                    lb_b(to_integer(r8_px(10 downto 0)))
                        <= r8_dark & std_logic_vector(r8_age);
                end if;
            end if;

            -----------------------------------------------------------------
            -- s10: palette lookup, pulsed threshold.
            -----------------------------------------------------------------
            r10_py <= PAL_Y(to_integer(r9_idx));
            r10_du <= signed(PAL_U(to_integer(r9_idx)) xor "1000000000");
            r10_dv <= signed(PAL_V(to_integer(r9_idx)) xor "1000000000");

            v_thrx := signed(resize(r9_thr, 10)) + r9_bp;
            if v_thrx < 0 then
                r10_thr <= (others => '0');
            elsif v_thrx > 255 then
                r10_thr <= (others => '1');
            else
                r10_thr <= unsigned(v_thrx(7 downto 0));
            end if;

            r10_hn   <= r9_hn;
            r10_dark <= r9_dark;
            r10_ctr  <= r9_ctr;
            r10_pol  <= r9_pol;

            -----------------------------------------------------------------
            -- s11: fringe chroma shift (case).
            -----------------------------------------------------------------
            case to_integer(s_frsel) is
                when 0      => v_fu := shift_right(r10_du, 7);
                                v_fv := shift_right(r10_dv, 7);
                when 1      => v_fu := shift_right(r10_du, 6);
                                v_fv := shift_right(r10_dv, 6);
                when 2      => v_fu := shift_right(r10_du, 5);
                                v_fv := shift_right(r10_dv, 5);
                when 3      => v_fu := shift_right(r10_du, 4);
                                v_fv := shift_right(r10_dv, 4);
                when 4      => v_fu := shift_right(r10_du, 3);
                                v_fv := shift_right(r10_dv, 3);
                when 5      => v_fu := shift_right(r10_du, 2);
                                v_fv := shift_right(r10_dv, 2);
                when 6      => v_fu := shift_right(r10_du, 1);
                                v_fv := shift_right(r10_dv, 1);
                when others => v_fu := r10_du;
                                v_fv := r10_dv;
            end case;
            r11_fu <= v_fu;
            r11_fv <= v_fv;

            r11_hn <= r10_hn;
            r11_py <= r10_py; r11_du <= r10_du; r11_dv <= r10_dv;
            r11_thr <= r10_thr;
            r11_dark <= r10_dark; r11_ctr <= r10_ctr;
            r11_pol <= r10_pol;

            -----------------------------------------------------------------
            -- s12: static ink decision, light luma, blanking flag.
            -----------------------------------------------------------------
            if (r11_hn(9 downto 2) < r11_thr) xor (r11_pol = '1') then
                r12_ly <= to_unsigned(96, 10);      -- static: dark fleck
            else
                r12_ly <= to_unsigned(720, 10);     -- static: light fleck
            end if;

            r12_py <= r11_py; r12_du <= r11_du; r12_dv <= r11_dv;
            r12_fu <= r11_fu; r12_fv <= r11_fv;
            r12_dark <= r11_dark; r12_ctr <= r11_ctr;
            r12_blank <= not pipe(LATENCY - 2).avid;

            -----------------------------------------------------------------
            -- s13 (output register): contour / bands / static + blanking.
            -----------------------------------------------------------------
            if r12_blank = '1' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif r12_ctr = '1' then
                -- thick black separating outline
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif r12_dark = '1' then
                s_io.y <= std_logic_vector(r12_py);
                -- +512 on a 10-bit signed delta = flip the MSB
                s_io.u <= std_logic_vector(unsigned(r12_du) xor "1000000000");
                s_io.v <= std_logic_vector(unsigned(r12_dv) xor "1000000000");
            else
                s_io.y <= std_logic_vector(r12_ly);
                s_io.u <= std_logic_vector(unsigned(r12_fu) xor "1000000000");
                s_io.v <= std_logic_vector(unsigned(r12_fv) xor "1000000000");
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

end architecture cleave;
