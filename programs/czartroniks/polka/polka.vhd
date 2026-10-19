-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- polka.vhd  (v2.0 — retro polka dots: palettes, layouts, and a swell fader)
--
-- v2.2.3 (2026-10-17, resolution scaling): interlace and the pixel aspect
-- come from the video timing ID (registers_in(8)) -- the core ties field_n
-- high, so 480i/576i were drawn at half the vertical step (2:1 tall dots).
-- Dot size, pitch, their floors and the sway speed follow the screen HEIGHT:
-- they are fractions of a 16:9 reference width (act_w on HD, 4/3 act_w on
-- SD), so a 4:3 SD screen shows the 1080p picture's centre with round dots
-- of the same size, and 720p is the 1080p picture at 2/3.  1080p unchanged.
--
-- POLKA renders a full-screen field of PERFECT CIRCLES over a configurable
-- background.  v2 reframes the program around two zoned selectors:
--
--   K3 LAYOUT — how the dots are spaced (dot size is ABSOLUTE, so Spacing
--   genuinely spreads dots apart rather than zooming):
--     Grid      square lattice
--     Brick     running bond: alternate rows offset half a pitch
--     Honey     staggered hex packing
--     Diamond   tight diamond lattice (stagger at half row pitch)
--     Cluster   dots huddle in 2x2 groups (deterministic pair offsets)
--     Tumble    every row shifted by its own hashed offset (hand-set look)
--     Scatter   per-dot hashed x jitter + 2-level y jitter, budgeted so dots
--               never clip their cells; the budget dies at the kiss point so
--               the Swell merge/invert arc stays exact
--     Fade      print-halftone ramp: dot size shrinks down the screen
--
--   K4 PALETTE — eight 50s background+dot combinations (Diner Cherry,
--     Seafoam & Coral, Atomic Teal, Butter & Turquoise, Blush & Charcoal,
--     Mint & Chocolate, Midnight Fiesta, Linen & Ink).  Every palette entry
--     is stored as (luma, saturation, hue angle); K5 TINT adds a global hue
--     offset so the whole combo rotates through the colour wheel together —
--     the lightness/saturation relationships (the "retro-ness") survive any
--     knob position.  S1 SWAP exchanges background and primary dot colour.
--
--   K6 COLOR MODE — how dots pick from the palette's three dot colours:
--     Solid / Duotone (checker) / Rows (rows cycle the trio) / Confetti
--     (stable per-dot hash — colours ride with the dots) / Cycle (slow
--     auto-rotation of the tint; K5 = speed, centre = frozen).
--
-- Dot colours go through the exact 1024-entry sine ROM in the pixel path;
-- the background colour is computed once per frame by borrowing that ROM
-- during the vsync cone (address mux hijack) and the shared serial
-- multiplier.  Geometry, circle purity, anti-aliasing (hard = edge band,
-- soft = parabolic dome), the porthole/video modes, the aspect-corrected
-- vertical DDA (16/18/15 Q4 per line, interlace-doubled) and the Swell
-- grow -> kiss -> merge -> invert arc are carried over from v1 unchanged.
--
-- Controls:
--   K1 Dot Size    K2 Spacing    K3 Layout    K4 Palette
--   K5 Tint (hue rotate; speed in Cycle)      K6 Color Mode
--   S1 Swap (bg <-> dot)   S2 Backdrop (palette/video)
--   S3 Fill (paint/porthole)   S4 Edges (hard/soft)
--   (S2 Video + S3 Porthole = spotlight: colour video portholes over a
--    half-bright grey copy of the video)
--   S5 Motion (still/sway: slow drift + wave animation)
--   P12 SWELL
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture polka of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY : natural := 17;

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    --------------------------------------------------------------------------
    -- Sin/cos ROM: 512 x 20 ((19:10) = 511*sin, (9:0) = 511*cos); hue angles
    -- quantize to 0.7 degree steps (invisible), halving the EBR footprint.
    --------------------------------------------------------------------------
    type t_scrom is array (0 to 511) of std_logic_vector(19 downto 0);
    function f_scrom return t_scrom is
        variable r    : t_scrom;
        variable s, c : integer;
    begin
        for i in 0 to 511 loop
            s := integer(round(511.0 * sin(real(i) * MATH_2_PI / 512.0)));
            c := integer(round(511.0 * cos(real(i) * MATH_2_PI / 512.0)));
            r(i) := std_logic_vector(to_signed(s, 10))
                  & std_logic_vector(to_signed(c, 10));
        end loop;
        return r;
    end function;
    constant C_SINCOS : t_scrom := f_scrom;

    --------------------------------------------------------------------------
    -- Retro palette ROM: 8 palettes x 4 entries (0 = background, 1..3 dots),
    -- each entry (28:19) = Y, (18:10) = sat, (9:0) = hue angle.  Values are
    -- BT.601, derived from period RGB swatches (see build notes); luma capped
    -- at 768 per the hardware rule.
    --------------------------------------------------------------------------
    subtype t_palv is std_logic_vector(28 downto 0);
    type t_palrom is array (0 to 31) of t_palv;
    function pv(y, s, a : integer) return t_palv is
    begin
        return std_logic_vector(to_unsigned(y, 10))
             & std_logic_vector(to_unsigned(s, 9))
             & std_logic_vector(to_unsigned(a, 10));
    end function;
    constant C_PALV : t_palrom := (
        -- 0 Diner Cherry: cream / cherry / charcoal / bubblegum
        0  => pv(768,  53, 856), 1  => pv(381, 280, 971),
        2  => pv(252,  11, 899), 3  => pv(696, 121, 1014),
        -- 1 Seafoam & Coral: seafoam / coral / cream / brick
        4  => pv(744,  75, 511), 5  => pv(570, 241, 944),
        6  => pv(768,  53, 856), 7  => pv(383, 212, 950),
        -- 2 Atomic Teal: teal / mustard / cream / burnt orange
        8  => pv(363, 151, 450), 9  => pv(643, 275, 842),
        10 => pv(768,  72, 851), 11 => pv(468, 262, 911),
        -- 3 Butter & Turquoise: butter / turquoise / white / tomato
        12 => pv(768, 164, 835), 13 => pv(514, 218, 468),
        14 => pv(768,  21, 847), 15 => pv(443, 276, 949),
        -- 4 Blush & Charcoal: blush / charcoal / white / lipstick
        16 => pv(768,  80, 971), 17 => pv(218,  13, 217),
        18 => pv(768,  21, 847), 19 => pv(376, 229, 1000),
        -- 5 Mint & Chocolate: mint / chocolate / cream / dusty rose
        20 => pv(768,  56, 580), 21 => pv(247,  62, 929),
        22 => pv(768,  53, 856), 23 => pv(603, 117, 971),
        -- 6 Midnight Fiesta: charcoal / aqua / coral / gold
        24 => pv(190,  13, 217), 25 => pv(594, 217, 473),
        26 => pv(570, 241, 944), 27 => pv(679, 261, 841),
        -- 7 Linen & Ink: linen / ink / signal red / warm gray
        28 => pv(768,  41, 854), 29 => pv(188,  11, 843),
        30 => pv(381, 280, 971), 31 => pv(521,  29, 841));

    --------------------------------------------------------------------------
    -- Enum-knob zone decider ROM (one EBR pair, replaces three compare banks).
    -- Index = knob select (1 = K6 colour mode, 5 even bands; 2 = K3 layout,
    -- 3 = K4 palette, 8 bands of 128) & current zone & knob(9:4).  The panel
    -- divides 0..1023 EVENLY across a knob's labels, so the bands must match.
    -- Stay in the current zone within +/-12 of its band, else jump to the
    -- band holding the 16-step bucket's centre.
    --------------------------------------------------------------------------
    type t_zrom is array (0 to 2047) of std_logic_vector(3 downto 0);
    function f_zrom return t_zrom is
        variable r : t_zrom;
        variable nz, cz, sel, q, v, lo, hi, nom : integer;
    begin
        for i in 0 to 2047 loop
            sel := i / 512;
            cz  := (i / 64) mod 8;
            q   := i mod 64;
            v   := q * 16 + 8;
            if sel = 1 then
                nz  := 5;
                nom := (v * 5) / 1024;
                lo  := (cz * 1024) / 5 - 12;
                hi  := ((cz + 1) * 1024) / 5 + 12;
            else
                nz  := 8;
                nom := v / 128;
                lo  := cz * 128 - 12;
                hi  := (cz + 1) * 128 + 12;
            end if;
            if cz < nz and v >= lo and v < hi then
                r(i) := std_logic_vector(to_unsigned(cz, 4));
            else
                r(i) := std_logic_vector(to_unsigned(nom, 4));
            end if;
        end loop;
        return r;
    end function;
    constant C_ZROM : t_zrom := f_zrom;

    function norm18(v : unsigned(17 downto 0)) return natural is
    begin
        for i in 17 downto 0 loop
            if v(i) = '1' then
                return 17 - i;
            end if;
        end loop;
        return 17;
    end function;

    function b2sl(b : boolean) return std_logic is
    begin
        if b then
            return '1';
        end if;
        return '0';
    end function;

    -- one glide step toward raw (~1/8 of the error, min +1 upward)
    function slstep(cur : unsigned(9 downto 0); d : signed(11 downto 0)) return unsigned is
        variable t, n : signed(11 downto 0);
    begin
        t := shift_right(d, 3);
        if t = 0 and d > 0 then
            t := to_signed(1, 12);
        end if;
        n := signed(resize(cur, 12)) + t;
        return unsigned(n(9 downto 0));
    end function;

    -- mod 3 of a 2-bit counter's successor / small values
    function m3lut(h : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(h) is
            when 0 | 3 | 6 => return to_unsigned(0, 2);
            when 1 | 4 | 7 => return to_unsigned(1, 2);
            when others    => return to_unsigned(2, 2);
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Raster position / measurement
    --------------------------------------------------------------------------
    signal vs_r, hs_r, av_r, fl_r : std_logic := '0';
    signal vy_r, vu_r, vv_r : std_logic_vector(9 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';
    signal r_hedge      : std_logic := '0';
    signal r_anchor     : std_logic := '0';
    signal r_arise      : std_logic := '0';
    signal frame_act    : std_logic := '0';

    signal xa_loc, xb_loc : unsigned(9 downto 0) := (others => '0');
    signal xa_idx, xb_idx : unsigned(7 downto 0) := (others => '0');
    signal vacc           : unsigned(13 downto 0) := (others => '0');
    signal iy             : unsigned(6 downto 0) := (others => '0');
    signal px             : unsigned(11 downto 0) := (others => '0');

    signal aline    : unsigned(10 downto 0) := (others => '0');
    signal act_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal act_h    : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal ilace    : std_logic := '0';
    -- Resolution scaling (v2.2.3): mode class from the video timing ID
    -- (registers_in(8)), latched once per frame.  s_sd = SD (480/576, shown
    -- 4:3), s_720 = 720p.  field_n is tied high on hardware, so interlace
    -- comes from the timing ID too, never from field_n toggling.
    signal s_sd, s_720 : std_logic := '0';
    signal s_pfl    : unsigned(4 downto 0) := to_unsigned(20, 5);  -- pitch floor
    signal field_lat : std_logic := '0';
    signal anchor_seen : std_logic := '1';

    --------------------------------------------------------------------------
    -- Frame-level parameters
    --------------------------------------------------------------------------
    signal slk1, slk2, slk5 : unsigned(9 downto 0) := (others => '0');
    signal slp12 : unsigned(9 downto 0) := (others => '0');
    signal rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0) := (others => '0');

    signal s_swap, s_bgvid, s_port, s_soft, s_sway : std_logic := '0';
    signal m_mode : unsigned(2 downto 0) := (others => '0');    -- colour mode 0..4
    signal m_lay  : unsigned(2 downto 0) := (others => '0');    -- layout 0..5
    signal s_pal  : unsigned(2 downto 0) := (others => '0');    -- palette 0..7

    signal vstep_base : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal vstep_eff  : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_xc       : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_ycq4     : unsigned(14 downto 0) := to_unsigned(8640, 15);
    signal s_wmax     : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal s_w        : unsigned(9 downto 0) := to_unsigned(120, 10);
    signal s_wm1      : unsigned(9 downto 0) := to_unsigned(119, 10);
    signal s_wh       : unsigned(9 downto 0) := to_unsigned(60, 10);
    signal s_hq4      : unsigned(12 downto 0) := to_unsigned(1920, 13);
    signal s_hpx      : unsigned(9 downto 0) := to_unsigned(120, 10);
    signal s_hh       : unsigned(9 downto 0) := to_unsigned(60, 10);
    signal s_hph      : unsigned(10 downto 0) := (others => '0');
    signal s_rbase    : unsigned(8 downto 0) := to_unsigned(40, 9);
    signal s_octc     : unsigned(9 downto 0) := (others => '0');
    signal s_octc7    : unsigned(9 downto 0) := (others => '0');
    signal s_ksq      : signed(10 downto 0) := (others => '0');
    signal s_topr     : unsigned(9 downto 0) := (others => '0');
    signal s_wsix     : unsigned(9 downto 0) := (others => '0');
    signal s_rcap     : unsigned(8 downto 0) := (others => '0');
    signal s_rtop     : unsigned(8 downto 0) := (others => '0');
    signal s_reff     : unsigned(8 downto 0) := to_unsigned(40, 9);
    signal s_vel      : unsigned(7 downto 0) := (others => '0');
    signal s_bdot     : std_logic := '0';
    signal s_rholef   : unsigned(8 downto 0) := (others => '0');
    signal s_hueoff   : unsigned(9 downto 0) := (others => '0');
    signal s_crate    : signed(5 downto 0) := (others => '0');
    signal s_ddot     : unsigned(17 downto 0) := to_unsigned(120, 18);
    signal s_dbuse    : unsigned(17 downto 0) := to_unsigned(120, 18);
    signal s_shA      : unsigned(4 downto 0) := (others => '0');
    signal s_gA       : unsigned(6 downto 0) := (others => '0');
    signal s_shB      : unsigned(4 downto 0) := (others => '0');
    signal s_gB       : unsigned(6 downto 0) := (others => '0');
    signal hole_off   : std_logic := '1';
    signal s_ebg : unsigned(1 downto 0) := "00";
    signal s_ed1 : unsigned(1 downto 0) := "01";
    signal s_jb       : unsigned(6 downto 0) := (others => '0');   -- jitter budget
    signal s_jsh      : unsigned(1 downto 0) := (others => '0');   -- x-jitter shift (jmax = 8<<jsh)
    signal s_jon      : std_logic := '0';
    signal s_jq       : unsigned(5 downto 0) := (others => '0');   -- y-jitter quantum
    signal s_clu      : std_logic := '0';
    signal s_jclu     : unsigned(5 downto 0) := (others => '0');   -- cluster huddle
    signal s_rmax     : unsigned(7 downto 0) := to_unsigned(120, 8);
    signal s_vspan    : unsigned(15 downto 0) := to_unsigned(17280, 16);
    signal s_fstep    : unsigned(7 downto 0) := (others => '0');
    signal s_RA2f     : unsigned(16 downto 0) := (others => '0');
    signal s_iy0m3    : unsigned(1 downto 0) := (others => '0');
    signal s_i0am3, s_i0bm3 : unsigned(1 downto 0) := (others => '0');
    signal s_bgy      : unsigned(9 downto 0) := C_BLKY;
    signal s_bgu      : unsigned(9 downto 0) := C_MID;
    signal s_bgv      : unsigned(9 downto 0) := C_MID;
    signal ix0a, ix0b : unsigned(6 downto 0) := (others => '0');
    signal iy0_r      : unsigned(6 downto 0) := (others => '0');
    signal vseed_r    : unsigned(13 downto 0) := (others => '0');
    signal xa0_r, xb0_r : unsigned(9 downto 0) := (others => '0');
    signal bnd_x      : unsigned(20 downto 0) := to_unsigned(245760, 21);
    signal bnd_y      : unsigned(20 downto 0) := to_unsigned(245760, 21);

    -- accumulators
    signal hpos : unsigned(20 downto 0) := (others => '0');
    signal cph  : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Shared FSM: frame maths + next-line maths, serial multiplier, divider.
    --------------------------------------------------------------------------
    signal fsm_t   : unsigned(8 downto 0) := (others => '1');
    signal lf_seed : std_logic := '0';
    signal fsm_run : std_logic := '0';     -- registered "fsm_t /= idle" (keeps
                                           -- the 9-bit idle compare out of every
                                           -- state register's enable cone)

    signal sm_run  : std_logic := '0';
    signal sm_cnt  : unsigned(3 downto 0) := (others => '0');
    signal smae    : unsigned(19 downto 0) := (others => '0');
    signal smb     : unsigned(10 downto 0) := (others => '0');
    signal smp     : unsigned(19 downto 0) := (others => '0');

    signal div_run : std_logic := '0';
    signal div_cnt : unsigned(4 downto 0) := (others => '0');
    signal div_d   : unsigned(21 downto 0) := (others => '0');
    signal div_q   : unsigned(21 downto 0) := (others => '0');
    signal div_rem : unsigned(12 downto 0) := (others => '0');
    signal div_v   : unsigned(12 downto 0) := (others => '1');
    signal facc    : unsigned(21 downto 0) := (others => '0');
    signal norm_t  : unsigned(17 downto 0) := (others => '0');

    signal fr_t    : unsigned(10 downto 0) := (others => '0');
    signal fr_hv   : signed(10 downto 0) := (others => '0');
    signal fr_hw   : signed(10 downto 0) := (others => '0');
    signal fr_h1, fr_h2, fr_h3 : signed(10 downto 0) := (others => '0');
    signal fr_rh2  : unsigned(16 downto 0) := (others => '0');
    signal fr_m3s  : unsigned(3 downto 0) := (others => '0');
    signal fr_z    : std_logic_vector(3 downto 0) := (others => '0');
    signal zi_r    : std_logic_vector(10 downto 0) := (others => '0');
    signal fr_jb   : signed(10 downto 0) := (others => '0');
    signal fr_sneg  : std_logic := '0';
    -- Per-frame palette table: the dot colour of each of the 4 palette
    -- entries (u, v after the saturation multiply), filled in vblank through
    -- the same pixel-path hijack the background colour uses, with the FSM's
    -- serial multiplier.  The pixel path looks the entry up instead of
    -- multiplying sin/cos by saturation on every pixel.
    signal hij_e    : unsigned(1 downto 0) := "00";
    type t_ptab is array (0 to 3) of unsigned(9 downto 0);
    signal pt_u, pt_v : t_ptab := (others => to_unsigned(512, 10));
    signal pal_hij : std_logic := '0';
    signal acc_t   : signed(21 downto 0) := (others => '0');
    signal xr_t    : unsigned(9 downto 0) := (others => '0');
    signal xq_t    : unsigned(6 downto 0) := (others => '0');
    signal xbs, xbs2 : unsigned(10 downto 0) := (others => '0');
    signal xb_ge   : std_logic := '0';
    signal sl_cur  : unsigned(9 downto 0) := (others => '0');
    signal sl_raw  : unsigned(9 downto 0) := (others => '0');
    signal sl_d    : signed(11 downto 0) := (others => '0');

    -- line FSM working registers
    signal lw_va   : unsigned(13 downto 0) := (others => '0');
    signal lw_iy   : unsigned(6 downto 0) := (others => '0');
    signal lw_dyA  : signed(10 downto 0) := (others => '0');
    signal lw_dyB  : signed(10 downto 0) := (others => '0');
    signal lw_adm  : unsigned(9 downto 0) := (others => '0');
    signal lw_adp  : unsigned(9 downto 0) := (others => '0');
    signal lw_adB  : unsigned(9 downto 0) := (others => '0');
    signal lw_up   : std_logic := '0';
    signal lw_m3   : unsigned(1 downto 0) := (others => '0');
    signal lw_ro   : unsigned(8 downto 0) := (others => '0');
    signal lw_dt   : unsigned(9 downto 0) := (others => '0');
    signal lw_sr   : unsigned(6 downto 0) := (others => '0');
    signal lw_r2   : signed(10 downto 0) := (others => '0');

    -- next-line shadow constants -> live copies at each h edge
    signal n_dyA2m, l_dyA2m : unsigned(15 downto 0) := (others => '0');
    signal n_dyA2p, l_dyA2p : unsigned(15 downto 0) := (others => '0');
    signal n_dyB2, l_dyB2 : unsigned(17 downto 0) := (others => '1');
    signal n_RA2,  l_RA2  : unsigned(16 downto 0) := (others => '0');
    signal n_RB2,  l_RB2  : unsigned(16 downto 0) := (others => '0');
    signal n_lsh,  l_lsh  : unsigned(9 downto 0) := (others => '0');
    signal n_m3,   l_m3   : unsigned(1 downto 0) := (others => '0');
    signal n_m3u,  l_m3u  : unsigned(1 downto 0) := (others => '0');
    signal n_jhy,  l_jhy  : unsigned(9 downto 0) := (others => '0');
    signal n_jhyb, l_jhyb : unsigned(9 downto 0) := (others => '0');
    signal n_up,   l_up   : std_logic := '0';

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    signal c1_axl, c1_bxl : unsigned(9 downto 0) := (others => '0');
    signal c1_ixa, c1_ixb : unsigned(6 downto 0) := (others => '0');
    signal c1_jp          : unsigned(9 downto 0) := (others => '0');

    signal c2_dxA, c2_dxB : signed(10 downto 0) := (others => '0');
    signal c2_jx          : signed(6 downto 0) := (others => '0');
    signal c2_jys         : std_logic := '0';
    signal c2_ixa, c2_ixb : unsigned(6 downto 0) := (others => '0');
    signal c2_iyb         : unsigned(6 downto 0) := (others => '0');
    signal c2_ha          : unsigned(9 downto 0) := (others => '0');

    -- 11-bit signed d times a 6-bit coverage (0..63), radix-4 Booth: 4
    -- partials of {0, +-d, +-2d} (one's complement, +1s gathered in a
    -- correction word) instead of 6 AND-gated rows
    function booth_d11_c6(d : signed(10 downto 0); w : unsigned(5 downto 0)) return signed is
        variable we   : std_logic_vector(8 downto 0);
        variable dg   : std_logic_vector(2 downto 0);
        variable mag  : signed(17 downto 0);
        variable pp   : signed(17 downto 0);
        variable d1   : signed(17 downto 0);
        variable d2   : signed(17 downto 0);
        variable corr : signed(17 downto 0);
        type t_pp is array (0 to 3) of signed(17 downto 0);
        variable p    : t_pp;
    begin
        we := "00" & std_logic_vector(w) & '0';
        d1 := resize(d, 18);
        d2 := shift_left(d1, 1);
        corr := (others => '0');
        for i in 0 to 3 loop
            dg := we(2 * i + 2 downto 2 * i);
            case dg is
                when "001" | "010" | "101" | "110" => mag := d1;
                when "011" | "100"                 => mag := d2;
                when others                        => mag := (others => '0');
            end case;
            if dg(2) = '1' and dg /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        return (p(0) + p(1)) + (p(2) + (p(3) + corr));
    end function;

    -- 8-bit squares from two 256 x 16 ROMs (one EBR each).  Each ROM is
    -- addressed by the |dx| that used to load the c3 operand registers, so its
    -- own address register is the c3 stage and c4_sqA/c4_sqB re-register it.
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
    signal c3_sqA, c3_sqB : std_logic_vector(15 downto 0) := (others => '0');
    signal c3_jys         : std_logic := '0';
    signal c3_ha          : unsigned(9 downto 0) := (others => '0');
    signal c3_hxb, c3_hyb : unsigned(9 downto 0) := (others => '0');
    signal c3_apA, c3_apB : std_logic := '0';

    signal c4_sqA, c4_sqB : unsigned(15 downto 0) := (others => '0');
    signal c4_jys         : std_logic := '0';
    signal c4_hA, c4_h1B  : unsigned(9 downto 0) := (others => '0');
    signal c4_apA, c4_apB : std_logic := '0';

    signal c5_sumA        : unsigned(16 downto 0) := (others => '0');
    signal c5_sumB        : unsigned(18 downto 0) := (others => '0');
    signal c5_hA, c5_hB   : unsigned(9 downto 0) := (others => '0');
    signal c5_apA, c5_apB : std_logic := '0';

    signal c6_dA, c6_dB   : unsigned(17 downto 0) := (others => '0');
    signal c6_ceA, c6_ceB : unsigned(1 downto 0) := (others => '0');
    signal c6_apA, c6_apB : std_logic := '0';

    signal c7_shA, c7_shB : unsigned(17 downto 0) := (others => '0');
    signal c7_satA, c7_satB : std_logic := '0';
    signal c7_entA, c7_entB : unsigned(1 downto 0) := (others => '0');
    signal c7_ceA, c7_ceB : unsigned(1 downto 0) := (others => '0');

    signal c8_byA, c8_byB : unsigned(5 downto 0) := (others => '0');
    signal c8_satA, c8_satB : std_logic := '0';
    signal c8_entA, c8_entB : unsigned(1 downto 0) := (others => '0');
    signal c8_ceA, c8_ceB : unsigned(1 downto 0) := (others => '0');

    signal c9_pA, c9_pB   : unsigned(12 downto 0) := (others => '0');
    signal c9_satA, c9_satB : std_logic := '0';
    signal c9_entA, c9_entB : unsigned(1 downto 0) := (others => '0');
    signal c9_ceA, c9_ceB : unsigned(1 downto 0) := (others => '0');
    signal c9_m3A, c9_m3B : unsigned(1 downto 0) := (others => '0');

    signal c10_cov : unsigned(6 downto 0) := (others => '0');
    signal c10_ap  : std_logic := '0';
    signal c10_ce  : unsigned(1 downto 0) := (others => '0');
    signal c10_m3  : unsigned(1 downto 0) := (others => '0');

    signal c11_hue : unsigned(9 downto 0) := (others => '0');
    signal c11_sat : unsigned(8 downto 0) := (others => '0');
    signal c11_y   : unsigned(9 downto 0) := (others => '0');
    signal c11_cov : unsigned(6 downto 0) := (others => '0');

    signal rom_rd  : std_logic_vector(19 downto 0) := (others => '0');
    signal c11_e, c12_e, c13_e, c14_e : unsigned(1 downto 0) := "00";
    signal c12_y   : unsigned(9 downto 0) := (others => '0');
    signal c12_cov : unsigned(6 downto 0) := (others => '0');

    signal c13_su, c13_sv : signed(9 downto 0) := (others => '0');
    signal c13_y   : unsigned(9 downto 0) := (others => '0');
    signal c13_cov : unsigned(6 downto 0) := (others => '0');

    signal c14_y   : unsigned(9 downto 0) := (others => '0');
    signal c14_cov : unsigned(6 downto 0) := (others => '0');

    signal c14_fgy : unsigned(9 downto 0) := C_BLKY;
    signal c14_fgu : unsigned(9 downto 0) := C_MID;
    signal c14_fgv : unsigned(9 downto 0) := C_MID;
    signal c14_bgy : unsigned(9 downto 0) := C_BLKY;
    signal c14_bgu : unsigned(9 downto 0) := C_MID;
    signal c14_bgv : unsigned(9 downto 0) := C_MID;
    signal c14_cov2 : unsigned(6 downto 0) := (others => '0');

    signal c15_dy, c15_du, c15_dv : signed(10 downto 0) := (others => '0');
    signal c15_bgy : unsigned(9 downto 0) := C_BLKY;
    signal c15_bgu : unsigned(9 downto 0) := C_MID;
    signal c15_bgv : unsigned(9 downto 0) := C_MID;
    signal c15_cov : unsigned(6 downto 0) := (others => '0');

    signal c16_my, c16_mu2, c16_mv2 : signed(17 downto 0) := (others => '0');
    signal c16_bgy : unsigned(9 downto 0) := C_BLKY;
    signal c16_bgu : unsigned(9 downto 0) := C_MID;
    signal c16_bgv : unsigned(9 downto 0) := C_MID;

    --------------------------------------------------------------------------
    -- Video delay: EBR ring + sync-bit FF shifts
    --------------------------------------------------------------------------
    type t_vring is array (0 to 31) of std_logic_vector(31 downto 0);
    signal vring : t_vring := (others => (others => '0'));
    signal wptr  : unsigned(4 downto 0) := (others => '0');
    signal vr_rd : std_logic_vector(31 downto 0) := (others => '0');
    -- spotlight backdrop (S2 Video + S3 Porthole): half-bright grey video,
    -- dimmed at the input and delayed in its own 16-bit ring so no maths
    -- ever sits on EBR read data
    type t_dring is array (0 to 31) of std_logic_vector(15 downto 0);
    signal dring : t_dring := (others => (others => '0'));
    signal vd_r  : unsigned(9 downto 0) := C_BLKY;
    signal vd_rd : std_logic_vector(15 downto 0) := (others => '0');

    signal sh_av, sh_hs, sh_vs, sh_fl : std_logic_vector(0 to LATENCY - 1)
        := (others => '0');

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Raster position, measurement, per-line live-constant latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_va : unsigned(14 downto 0);
        variable v_sd : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            vs_r <= data_in.vsync_n;
            hs_r <= data_in.hsync_n;
            av_r <= data_in.avid;
            fl_r <= data_in.field_n;
            prev_hsync_n <= hs_r;
            prev_vsync_n <= vs_r;
            prev_avid    <= av_r;

            v_h_edge := '0'; v_v_edge := '0';
            if hs_r = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if vs_r = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;
            r_hedge  <= v_h_edge;
            r_anchor <= '0';
            r_arise  <= '0';

            if v_h_edge = '1' then
                px <= (others => '0');
            elsif av_r = '1' then
                px <= px + 1;
            end if;
            if prev_avid = '1' and av_r = '0' then
                if px > 100 then
                    act_w <= px;
                end if;
                aline <= aline + 1;
            end if;

            -- X lattices, seeded with the layout's per-line shift applied
            if v_h_edge = '1' then
                v_sd := resize(xa0_r, 11) + resize(l_lsh, 11);
                if v_sd >= resize(s_w, 11) then
                    v_sd := v_sd - resize(s_w, 11);
                end if;
                xa_loc <= v_sd(9 downto 0);
                v_sd := resize(xb0_r, 11) + resize(l_lsh, 11);
                if v_sd >= resize(s_w, 11) then
                    v_sd := v_sd - resize(s_w, 11);
                end if;
                xb_loc <= v_sd(9 downto 0);
                xa_idx <= (others => '0');
                xb_idx <= (others => '0');
            elsif av_r = '1' then
                if xa_loc = s_wm1 then
                    xa_loc <= (others => '0');
                    xa_idx <= xa_idx + 1;
                else
                    xa_loc <= xa_loc + 1;
                end if;
                if xb_loc = s_wm1 then
                    xb_loc <= (others => '0');
                    xb_idx <= xb_idx + 1;
                else
                    xb_loc <= xb_loc + 1;
                end if;
            end if;

            -- frame anchor + vertical DDA (aspect-corrected Q4 step)
            if v_v_edge = '1' then
                frame_act <= '0';
            elsif av_r = '1' and frame_act = '0' then
                frame_act <= '1';
                r_anchor  <= '1';
                r_arise   <= '1';
                vacc      <= vseed_r;
                iy        <= iy0_r;
                if aline > 50 then
                    act_h <= aline;
                end if;
                aline <= (others => '0');
            elsif av_r = '1' and prev_avid = '0' then
                r_arise <= '1';
            elsif v_h_edge = '1' and frame_act = '1' then
                v_va := resize(vacc, 15) + resize(vstep_eff, 15);
                if v_va >= resize(s_hq4, 15) then
                    vacc <= resize(v_va - resize(s_hq4, 15), 14);
                    iy   <= iy + 1;
                else
                    vacc <= v_va(13 downto 0);
                end if;
            end if;

            -- shadow -> live per-line constants
            if v_h_edge = '1' then
                l_dyA2m <= n_dyA2m;
                l_dyA2p <= n_dyA2p;
                l_dyB2  <= n_dyB2;
                l_RA2   <= n_RA2;
                l_RB2   <= n_RB2;
                l_lsh   <= n_lsh;
                l_m3    <= n_m3;
                l_m3u   <= n_m3u;
                l_jhy   <= n_jhy;
                l_jhyb  <= n_jhyb;
                l_up    <= n_up;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Frame + line FSM.
    --------------------------------------------------------------------------
    p_fsm : process(clk)
        variable v_acc  : unsigned(13 downto 0);
        variable v_z    : integer range 0 to 5;
        variable v_raw  : integer range 0 to 1023;
        variable v_dk   : signed(10 downto 0);
        variable v_w    : unsigned(9 downto 0);
        variable v_span : signed(10 downto 0);
        variable v_r9   : signed(10 downto 0);
        variable v_vaN  : unsigned(14 downto 0);
        variable v_int  : signed(10 downto 0);
        variable v_pw   : t_palv;
        variable v_u12  : signed(11 downto 0);
        variable v_l11  : signed(10 downto 0);
        variable v_zi   : std_logic_vector(10 downto 0);
    begin
        if rising_edge(clk) then
            -- serial unsigned shift-add multiplier: 11 cycles, smp = a*b
            if sm_run = '1' then
                if smb(0) = '1' then
                    smp <= smp + smae;
                end if;
                smae <= shift_left(smae, 1);
                smb  <= shift_right(smb, 1);
                if sm_cnt = 10 then
                    sm_run <= '0';
                else
                    sm_cnt <= sm_cnt + 1;
                end if;
            end if;

            -- serial restoring divider: 22-bit dividend / 13-bit divisor
            if div_run = '1' then
                v_acc := (div_rem & div_d(21)) - resize(div_v, 14);
                if v_acc(13) = '0' then
                    div_rem <= v_acc(12 downto 0);
                    div_q   <= div_q(20 downto 0) & '1';
                else
                    div_rem <= div_rem(11 downto 0) & div_d(21);
                    div_q   <= div_q(20 downto 0) & '0';
                end if;
                div_d <= div_d(20 downto 0) & '0';
                if div_cnt = 21 then
                    div_run <= '0';
                else
                    div_cnt <= div_cnt + 1;
                end if;
            end if;

            -- enum zone ROM: states 1/2/3 index K6/K3/K4, result lands two
            -- states later (3/4/5).  The index is registered in fabric so the
            -- EBR address never sees the FSM's next-state cone.
            case fsm_t(1 downto 0) is
                when "01"   => v_zi := "01" & std_logic_vector(m_mode) & std_logic_vector(rk6(9 downto 4));
                when "10"   => v_zi := "10" & std_logic_vector(m_lay)  & std_logic_vector(rk3(9 downto 4));
                when others => v_zi := "11" & std_logic_vector(s_pal)  & std_logic_vector(rk4(9 downto 4));
            end case;
            zi_r <= v_zi;
            fr_z <= C_ZROM(to_integer(unsigned(zi_r)));

            ------------------------------------------------------------------
            -- State bodies run on fsm_run alone; the triggers below come
            -- AFTER the case so they override it without sitting in every
            -- state register's enable cone
            ------------------------------------------------------------------
            if fsm_run = '1' then
                fsm_t <= fsm_t + 1;

                case to_integer(fsm_t) is

                    ----------------------------------------------------------
                    -- FRAME maths (one small op per state; states are free)
                    ----------------------------------------------------------
                    when 0 =>
                        rk1  <= unsigned(registers_in(0)(9 downto 0));
                        rk2  <= unsigned(registers_in(1)(9 downto 0));
                        rk3  <= unsigned(registers_in(2)(9 downto 0));
                        rk4  <= unsigned(registers_in(3)(9 downto 0));
                        rk5  <= unsigned(registers_in(4)(9 downto 0));
                        rk6  <= unsigned(registers_in(5)(9 downto 0));
                        rp12 <= unsigned(registers_in(7)(9 downto 0));
                        s_swap  <= registers_in(6)(0);
                        s_bgvid <= registers_in(6)(1);
                        s_port  <= registers_in(6)(2);
                        s_soft  <= registers_in(6)(3);
                        s_sway  <= registers_in(6)(4);
                        field_lat <= fl_r;
                        case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                            when 0 | 8      => ilace <= '1'; s_sd <= '1'; s_720 <= '0';
                            when 4 | 12     => ilace <= '0'; s_sd <= '1'; s_720 <= '0';
                            when 1 | 2 | 10 => ilace <= '1'; s_sd <= '0'; s_720 <= '0';
                            when 5 | 6 | 14 => ilace <= '0'; s_sd <= '0'; s_720 <= '1';
                            when others     => ilace <= '0'; s_sd <= '0'; s_720 <= '0';
                        end case;
                    when 1 =>
                        -- (zone ROM indexes K6 this state -> m_mode in 3)
                        -- aspect-corrected vertical step (Q4 hpx per frame
                        -- line; doubled in state 3 when interlaced): 480-line
                        -- SD 18, 576-line SD 15, HD square pixels 16
                        case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                            when 0 | 4  => vstep_base <= to_unsigned(18, 6);
                            when 8 | 12 => vstep_base <= to_unsigned(15, 6);
                            when others => vstep_base <= to_unsigned(16, 6);
                        end case;
                        -- sizes follow screen HEIGHT: the reference width is
                        -- a 16:9 frame at this height (SD: 4/3 act_w, so SD
                        -- shows the 1080p picture's 4:3 centre); the floors
                        -- scale with it (1080: 20, 720p: 13, SD: 10 px)
                        if s_sd = '1' then
                            s_pfl <= to_unsigned(10, 5);
                        elsif s_720 = '1' then
                            s_pfl <= to_unsigned(13, 5);
                        else
                            s_pfl <= to_unsigned(20, 5);
                        end if;
                        s_xc <= act_w(11 downto 1);
                    when 2 =>
                        null;
                    when 3 =>
                        m_mode <= unsigned(fr_z(2 downto 0));
                        if ilace = '1' then
                            vstep_eff <= shift_left(resize(vstep_base, 7), 1);
                        else
                            vstep_eff <= resize(vstep_base, 7);
                        end if;
                        if s_sd = '1' then
                            smae <= to_unsigned(57, 20);               -- (4/3 act_w)/6
                        else
                            smae <= to_unsigned(43, 20);               -- act_w/6
                        end if;
                        smb  <= resize(act_w, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    -- knob glide, one shared sub/step unit, 3 states per knob
                    when 4  => sl_cur <= slk2; sl_raw <= rk2;
                               m_lay <= unsigned(fr_z(2 downto 0));
                    when 5  => sl_d <= signed(resize(sl_raw, 12)) - signed(resize(sl_cur, 12));
                               s_pal <= unsigned(fr_z(2 downto 0));
                    when 6  => slk2 <= slstep(sl_cur, sl_d);
                    when 7  => sl_cur <= slk1; sl_raw <= rk1;
                    when 8  => sl_d <= signed(resize(sl_raw, 12)) - signed(resize(sl_cur, 12));
                    when 9  => slk1 <= slstep(sl_cur, sl_d);
                    when 10 => sl_cur <= slk5; sl_raw <= rk5;
                    when 11 => sl_d <= signed(resize(sl_raw, 12)) - signed(resize(sl_cur, 12));
                    when 12 => slk5 <= slstep(sl_cur, sl_d);
                    when 13 => sl_cur <= slp12; sl_raw <= rp12;
                    when 14 => sl_d <= signed(resize(sl_raw, 12)) - signed(resize(sl_cur, 12));
                    when 15 =>
                        slp12 <= slstep(sl_cur, sl_d);
                        s_wmax <= resize(smp(17 downto 8), 10);        -- act_w/6
                        smae <= resize(vstep_eff, 20);
                        smb  <= resize(act_h, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 16 =>
                        -- dot rmax = Wref/16: act_w/16 (HD), wmax*3/8 (SD)
                        if s_sd = '1' then
                            s_rmax <= resize(s_wmax(9 downto 2), 8) + resize(s_wmax(9 downto 3), 8);
                        else
                            s_rmax <= act_w(11 downto 4);
                        end if;
                    when 28 =>
                        s_ycq4 <= smp(15 downto 1);                    -- (act_h*vstep)/2
                        s_vspan <= smp(15 downto 0);
                        v_span := signed(resize(s_wmax, 11)) - signed(resize(s_pfl, 11));
                        if v_span < 0 then
                            v_span := (others => '0');
                        end if;
                        smae <= resize(unsigned(v_span(9 downto 0)), 20);
                        smb  <= resize(slk2, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 41 =>
                        v_w := resize(s_pfl, 10) + smp(19 downto 10);
                        s_w <= v_w;
                        smae <= resize(v_w, 20);
                        smb  <= to_unsigned(887, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 42 =>
                        s_wm1 <= s_w - 1;
                        s_wh  <= '0' & s_w(9 downto 1);
                    when 54 =>
                        -- row pitch by layout: honey 0.866W, diamond 0.5W, else W
                        case to_integer(m_lay) is
                            when 2      => s_hq4 <= smp(18 downto 6);
                            when 3      => s_hq4 <= shift_left(resize(s_w, 13), 3);
                            when others => s_hq4 <= shift_left(resize(s_w, 13), 4);
                        end case;
                        bnd_x <= shift_left(resize(s_w, 21), 11);
                    when 55 =>
                        s_hpx <= resize(s_hq4(12 downto 4), 10);
                        bnd_y <= shift_left(resize(s_hq4, 21), 7);
                    when 56 =>
                        s_hh <= '0' & s_hpx(9 downto 1);
                    when 57 =>
                        if s_wh >= s_hh then
                            s_octc <= resize(s_wh, 10) + resize(s_hh(9 downto 1), 10);
                        else
                            s_octc <= resize(s_hh, 10) + resize(s_wh(9 downto 1), 10);
                        end if;
                        s_hph <= resize(s_hpx, 11) + resize(s_hh, 11);
                    when 58 =>
                        s_rcap  <= resize(s_octc, 9) + resize(s_w(9 downto 3), 9);
                        s_octc7 <= s_octc - resize(s_octc(9 downto 3), 10);
                    when 59 =>
                        s_rtop <= s_rcap - resize(s_w(9 downto 4), 9);
                        s_wsix <= resize(s_w(9 downto 3), 10) + resize(s_w(9 downto 5), 10);
                    when 60 =>
                        s_ksq  <= signed(resize(s_octc7, 11))
                                - signed(resize(s_wh, 11))
                                + signed(resize(s_wh(9 downto 2), 11));
                        s_topr <= resize(s_rtop, 10) + resize(s_w(9 downto 4), 10);
                        -- ABSOLUTE dot size: independent of the pitch, so the
                        -- Spacing knob spreads dots instead of zooming
                        smae <= resize(s_rmax, 20);
                        smb  <= resize(slk1, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 73 =>
                        if s_sd = '1' or s_720 = '1' then
                            s_rbase <= to_unsigned(1, 9) + smp(17 downto 10);
                        else
                            s_rbase <= to_unsigned(2, 9) + smp(17 downto 10);
                        end if;
                    when 74 =>
                        -- Swell always owns kiss -> merge -> invert: the Size
                        -- radius stops at the kiss (W/2), so it can never eat
                        -- the fader's travel (rtop >= 0.69W in every layout)
                        if s_rbase > resize(s_wh, 9) then
                            s_rbase <= resize(s_wh, 9);
                        end if;
                    when 75 =>
                        v_span := signed(resize(s_rtop, 11)) - signed(resize(s_rbase, 11));
                        if v_span < 0 then
                            v_span := (others => '0');
                        end if;
                        smae <= resize(unsigned(v_span(9 downto 0)), 20);
                        smb  <= resize(slp12, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 88 =>
                        fr_t <= resize(s_rbase, 11) + resize(smp(18 downto 10), 11);
                    when 89 =>
                        if fr_t > resize(s_rcap, 11) then
                            s_reff <= s_rcap;
                        else
                            s_reff <= fr_t(8 downto 0);
                        end if;
                    when 90 =>
                        if (m_lay = 1 or m_lay = 2 or m_lay = 3)
                           and s_reff < resize(s_octc, 9) then
                            s_bdot <= '1';
                        else
                            s_bdot <= '0';
                        end if;
                        -- jitter/huddle budget: shrinks to zero at the kiss
                        fr_jb <= signed(resize(s_wh, 11)) - signed(resize(s_reff, 11));
                        -- entry mapping (S1 swap)
                        if s_swap = '1' then
                            s_ebg <= "01";
                            s_ed1 <= "00";
                        else
                            s_ebg <= "00";
                            s_ed1 <= "01";
                        end if;
                    when 91 =>
                        v_l11 := fr_jb;
                        if v_l11 > signed(resize(s_w(9 downto 2), 11)) then
                            v_l11 := signed(resize(s_w(9 downto 2), 11));
                        end if;
                        if v_l11 < 0 or m_lay /= 6 then
                            v_l11 := (others => '0');
                        end if;
                        s_jb <= unsigned(v_l11(6 downto 0));
                        -- cluster huddle distance: W/8 capped by the budget
                        v_l11 := fr_jb;
                        if v_l11 > signed(resize(s_w(9 downto 3), 11)) then
                            v_l11 := signed(resize(s_w(9 downto 3), 11));
                        end if;
                        if v_l11 < 0 or m_lay /= 4 then
                            v_l11 := (others => '0');
                        end if;
                        s_jclu <= unsigned(v_l11(5 downto 0));
                    when 92 =>
                        -- x-jitter magnitude 8<<jsh <= jb (quantized budget)
                        if s_jb >= 64 then
                            s_jsh <= "11"; s_jon <= '1';
                        elsif s_jb >= 32 then
                            s_jsh <= "10"; s_jon <= '1';
                        elsif s_jb >= 16 then
                            s_jsh <= "01"; s_jon <= '1';
                        elsif s_jb >= 8 then
                            s_jsh <= "00"; s_jon <= '1';
                        else
                            s_jsh <= "00"; s_jon <= '0';
                        end if;
                        if m_lay = 4 then
                            s_jq <= s_jclu;
                        else
                            s_jq <= s_jb(6 downto 1);
                        end if;
                        if m_lay = 4 then
                            s_clu <= '1';
                        else
                            s_clu <= '0';
                        end if;
                        -- tint: knob in most modes, accumulator in Cycle
                        if m_mode = 4 then
                            s_hueoff <= cph;
                            v_dk := signed(resize(slk5, 11)) - to_signed(512, 11);
                            if v_dk > 20 then
                                s_crate <= resize(shift_right(v_dk - 20, 4), 6);
                            elsif v_dk < -20 then
                                s_crate <= resize(shift_right(v_dk + 20 + 15, 4), 6);
                            else
                                s_crate <= (others => '0');
                            end if;
                        else
                            s_hueoff <= slk5;
                            s_crate  <= (others => '0');
                        end if;
                        -- sway speed (Q4 px/frame), scaled like the sizes
                        if s_sd = '1' then
                            s_vel <= to_unsigned(10, 8);
                        elsif s_720 = '1' then
                            s_vel <= to_unsigned(13, 8);
                        else
                            s_vel <= to_unsigned(20, 8);
                        end if;
                    -- hole-radius ramp at R_eff (single-writer chain)
                    when 93 =>
                        if m_lay = 1 or m_lay = 2 or m_lay = 3 then
                            fr_hv <= signed(resize(s_reff, 11)) - signed(resize(s_octc, 11));
                        else
                            fr_hv <= s_ksq - signed(resize(s_reff(8 downto 2), 11));
                        end if;
                        fr_hw <= signed(resize(s_topr, 11)) - signed(resize(s_reff, 11));
                    when 94 =>
                        if (m_lay = 1 or m_lay = 2 or m_lay = 3) and fr_hv > signed(resize(s_wsix, 11)) then
                            fr_h1 <= signed(resize(s_wsix, 11));
                        else
                            fr_h1 <= fr_hv;
                        end if;
                    when 95 =>
                        if (m_lay = 1 or m_lay = 2 or m_lay = 3) and fr_h1 > fr_hw then
                            fr_h2 <= fr_hw;
                        else
                            fr_h2 <= fr_h1;
                        end if;
                    when 96 =>
                        if fr_h2 > signed(resize(s_w(9 downto 2), 11)) then
                            fr_h3 <= signed(resize(s_w(9 downto 2), 11));
                        else
                            fr_h3 <= fr_h2;
                        end if;
                    when 97 =>
                        if fr_h3 < 0 then
                            s_rholef <= (others => '0');
                        else
                            s_rholef <= unsigned(fr_h3(8 downto 0));
                        end if;
                    when 98 =>
                        if s_rholef < 3 then
                            hole_off <= '1';
                        else
                            hole_off <= '0';
                        end if;
                    when 99 =>
                        smae <= resize(s_reff, 20);
                        smb  <= resize(s_reff, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 112 =>
                        s_RA2f <= smp(16 downto 0);
                        if s_soft = '1' then
                            s_ddot <= resize(smp(17 downto 0), 18);    -- R^2 dome
                        else
                            s_ddot <= resize(s_reff, 18)
                                    + shift_left(resize(s_reff, 18), 1);
                        end if;
                        smae <= resize(s_rholef, 20);
                        smb  <= resize(s_rholef, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 125 =>
                        fr_rh2 <= smp(16 downto 0);
                        if s_bdot = '1' then
                            n_RB2 <= s_RA2f;
                        else
                            n_RB2 <= smp(16 downto 0);
                        end if;
                        if s_soft = '1' then
                            s_dbuse <= resize(smp(17 downto 0), 18);
                        else
                            s_dbuse <= resize(s_rholef, 18)
                                     + shift_left(resize(s_rholef, 18), 1);
                        end if;
                    when 126 =>
                        if s_ddot < 4 then
                            s_ddot <= to_unsigned(4, 18);
                        end if;
                        if s_dbuse < 4 then
                            s_dbuse <= to_unsigned(4, 18);
                        end if;
                    when 127 =>
                        s_shA <= to_unsigned(norm18(s_ddot), 5);
                        s_shB <= to_unsigned(norm18(s_dbuse), 5);
                    when 128 =>
                        case to_integer(s_shA(4 downto 2)) is
                            when 0      => norm_t <= s_ddot;
                            when 1      => norm_t <= s_ddot sll 4;
                            when 2      => norm_t <= s_ddot sll 8;
                            when 3      => norm_t <= s_ddot sll 12;
                            when others => norm_t <= s_ddot sll 16;
                        end case;
                    when 129 =>
                        case to_integer(s_shA(1 downto 0)) is
                            when 0      => div_v <= resize(norm_t(17 downto 12), 13);
                            when 1      => div_v <= resize(norm_t(16 downto 11), 13);
                            when 2      => div_v <= resize(norm_t(15 downto 10), 13);
                            when others => div_v <= resize(norm_t(14 downto 9), 13);
                        end case;
                        div_d   <= to_unsigned(2048, 22);
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 152 =>
                        if div_q > 127 then
                            s_gA <= (others => '1');
                        else
                            s_gA <= div_q(6 downto 0);
                        end if;
                        case to_integer(s_shB(4 downto 2)) is
                            when 0      => norm_t <= s_dbuse;
                            when 1      => norm_t <= s_dbuse sll 4;
                            when 2      => norm_t <= s_dbuse sll 8;
                            when 3      => norm_t <= s_dbuse sll 12;
                            when others => norm_t <= s_dbuse sll 16;
                        end case;
                    when 153 =>
                        case to_integer(s_shB(1 downto 0)) is
                            when 0      => div_v <= resize(norm_t(17 downto 12), 13);
                            when 1      => div_v <= resize(norm_t(16 downto 11), 13);
                            when 2      => div_v <= resize(norm_t(15 downto 10), 13);
                            when others => div_v <= resize(norm_t(14 downto 9), 13);
                        end case;
                        div_d   <= to_unsigned(2048, 22);
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 154 =>
                        facc <= resize(hpos(20 downto 4), 22)
                              + shift_left(resize(s_w, 22), 7);
                    when 155 =>
                        facc <= facc + resize(s_wh, 22) - resize(s_xc, 22);
                    when 176 =>
                        if s_bdot = '1' then
                            s_shB  <= s_shA;
                            s_gB   <= s_gA;
                        elsif hole_off = '1' or m_lay = 7 then
                            s_gB <= (others => '0');
                        elsif div_q > 127 then
                            s_gB <= (others => '1');
                        else
                            s_gB <= div_q(6 downto 0);
                        end if;
                        -- X grid seed: (hpos_px + 128W + W/2 - xc) mod W
                        div_v   <= resize(s_w, 13);
                        div_d   <= facc;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 177 =>
                        facc <= to_unsigned(0, 22)
                              + shift_left(resize(s_hq4, 22), 7);
                    when 178 =>
                        facc <= facc + resize(s_hq4(12 downto 1), 22) - resize(s_ycq4, 22);
                    when 179 =>
                        if ilace = '1' and field_lat = '0' then
                            facc <= facc + resize(vstep_base, 22);
                        end if;
                    when 199 =>
                        xa0_r <= div_rem(9 downto 0);
                        ix0a  <= div_q(6 downto 0);
                        xr_t  <= div_rem(9 downto 0);
                        xq_t  <= div_q(6 downto 0);
                        -- Y grid seed: (128H + H/2 - yc + field) mod H
                        div_v   <= s_hq4;
                        div_d   <= facc;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 200 =>
                        xbs <= resize(xr_t, 11) + resize(s_wh, 11);
                    when 201 =>
                        if xbs >= resize(s_w, 11) then
                            xb_ge <= '1';
                        else
                            xb_ge <= '0';
                        end if;
                        xbs2 <= xbs - resize(s_w, 11);
                    when 202 =>
                        if xb_ge = '1' then
                            xb0_r <= xbs2(9 downto 0);
                            ix0b  <= xq_t + 1;
                        else
                            xb0_r <= xbs(9 downto 0);
                            ix0b  <= xq_t;
                        end if;
                    when 222 =>
                        vseed_r <= resize(div_rem, 14);
                        iy0_r   <= div_q(6 downto 0);
                    when 223 =>
                        -- mod-3 via base-4 digit sum (4 == 1 mod 3); never a
                        -- literal "mod 3" (synthesizes a divider chain)
                        fr_m3s <= resize(iy0_r(1 downto 0), 4)
                                + resize(iy0_r(3 downto 2), 4)
                                + resize(iy0_r(5 downto 4), 4)
                                + resize(iy0_r(6 downto 6), 4);
                    ----------------------------------------------------------
                    -- Background colour via the pixel path's sine ROM
                    -- (address hijack during the vsync cone; avid is low, so
                    -- the pixel pipe's colours are gated anyway)
                    ----------------------------------------------------------
                    when 224 =>
                        pal_hij <= '1';
                        hij_e   <= s_ebg;
                        smae <= resize(s_hq4, 20);
                        smb  <= resize(s_reff, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 237 =>
                        facc <= resize(smp(19 downto 4), 22);
                    when 238 =>
                        div_v   <= resize(s_vspan(15 downto 4), 13);
                        div_d   <= facc;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                        case to_integer(fr_m3s) is
                            when 0 | 3 | 6 | 9 | 12 => s_iy0m3 <= "00";
                            when 1 | 4 | 7 | 10     => s_iy0m3 <= "01";
                            when others             => s_iy0m3 <= "10";
                        end case;
                    when 242 =>
                        -- pixel path stages carried the bg entry: c11_y is the
                        -- bg luma, rom_rd holds sin/cos of its tinted hue
                        s_bgy <= c11_y;
                        -- read the SECOND-registered ROM copy (c13_su), never
                        -- RDATA: the EBR's absorbed output register is not a
                        -- fabric FF (this cone was hd_hdmi's last 0.14 MHz)
                        if c13_su < 0 then
                            fr_sneg <= '1';
                            smae <= resize(unsigned(-c13_su), 20);
                        else
                            fr_sneg <= '0';
                            smae <= resize(unsigned(c13_su), 20);
                        end if;
                        smb  <= resize(c11_sat, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 255 =>
                        if fr_sneg = '1' then
                            v_u12 := to_signed(512, 12) - signed(resize(smp(18 downto 9), 12));
                        else
                            v_u12 := to_signed(512, 12) + signed(resize(smp(18 downto 9), 12));
                        end if;
                        s_bgu <= unsigned(v_u12(9 downto 0));
                        if c13_sv < 0 then
                            fr_sneg <= '1';
                            smae <= resize(unsigned(-c13_sv), 20);
                        else
                            fr_sneg <= '0';
                            smae <= resize(unsigned(c13_sv), 20);
                        end if;
                        smb  <= resize(c11_sat, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 268 =>
                        if fr_sneg = '1' then
                            v_u12 := to_signed(512, 12) - signed(resize(smp(18 downto 9), 12));
                        else
                            v_u12 := to_signed(512, 12) + signed(resize(smp(18 downto 9), 12));
                        end if;
                        s_bgv <= unsigned(v_u12(9 downto 0));
                        pal_hij <= '0';
                        if div_q > 255 then
                            s_fstep <= (others => '1');
                        else
                            s_fstep <= div_q(7 downto 0);
                        end if;
                    when 269 =>
                        lf_seed <= '1';
                        fsm_t   <= to_unsigned(373, 9);

                    ----------------------------------------------------------
                    -- Palette table: entry e at 373 + 34e.  Hijack the colour
                    -- path to entry e, then |sin|*sat and |cos|*sat on the
                    -- serial multiplier; u = 512 + floor(sin*sat / 512) as the
                    -- pixel path's signed multiply and shift gave it
                    -- (a negative product rounds down: subtract the ceiling).
                    ----------------------------------------------------------
                    when 373 => pal_hij <= '1'; hij_e <= "00";
                    when 407 => pal_hij <= '1'; hij_e <= "01";
                    when 441 => pal_hij <= '1'; hij_e <= "10";
                    when 475 => pal_hij <= '1'; hij_e <= "11";
                    when 379 | 413 | 447 | 481 =>
                        if c13_su < 0 then
                            fr_sneg <= '1';
                            smae <= resize(unsigned(-c13_su), 20);
                        else
                            fr_sneg <= '0';
                            smae <= resize(unsigned(c13_su), 20);
                        end if;
                        smb  <= resize(c11_sat, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 392 | 426 | 460 | 494 =>
                        if fr_sneg = '1' then
                            v_u12 := to_signed(512, 12)
                                     - signed(resize(shift_right(smp + 511, 9), 12));
                        else
                            v_u12 := to_signed(512, 12)
                                     + signed(resize(shift_right(smp, 9), 12));
                        end if;
                        pt_u(to_integer(hij_e)) <= unsigned(v_u12(9 downto 0));
                        if c13_sv < 0 then
                            fr_sneg <= '1';
                            smae <= resize(unsigned(-c13_sv), 20);
                        else
                            fr_sneg <= '0';
                            smae <= resize(unsigned(c13_sv), 20);
                        end if;
                        smb  <= resize(c11_sat, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 405 | 439 | 473 | 507 =>
                        if fr_sneg = '1' then
                            v_u12 := to_signed(512, 12)
                                     - signed(resize(shift_right(smp + 511, 9), 12));
                        else
                            v_u12 := to_signed(512, 12)
                                     + signed(resize(shift_right(smp, 9), 12));
                        end if;
                        pt_v(to_integer(hij_e)) <= unsigned(v_u12(9 downto 0));
                        if fsm_t = 507 then
                            pal_hij <= '0';
                            fsm_t   <= to_unsigned(300, 9);
                        end if;

                    ----------------------------------------------------------
                    -- Anchor microflow: drift/colour/wave accumulators
                    ----------------------------------------------------------
                    when 290 =>
                        anchor_seen <= '1';
                        cph <= unsigned(signed(cph) + resize(s_crate, 10));
                        acc_t <= signed(resize(hpos, 22));
                    when 291 =>
                        if s_sway = '1' then
                            acc_t <= acc_t + signed(resize(s_vel, 22));
                        end if;
                    when 292 =>
                        if acc_t >= signed(resize(bnd_x, 22)) then
                            acc_t <= acc_t - signed(resize(bnd_x, 22));
                        end if;
                    when 293 =>
                        hpos <= unsigned(acc_t(20 downto 0));
                        fsm_t <= to_unsigned(300, 9);

                    ----------------------------------------------------------
                    -- LINE maths (next line's constants; runs during active)
                    ----------------------------------------------------------
                    when 300 =>
                        if lf_seed = '1' then
                            lw_va  <= vseed_r;
                            lw_iy  <= iy0_r;
                            lw_m3  <= s_iy0m3;
                            lw_sr  <= (others => '0');
                        else
                            v_vaN := resize(vacc, 15) + resize(vstep_eff, 15);
                            if v_vaN >= resize(s_hq4, 15) then
                                lw_va <= resize(v_vaN - resize(s_hq4, 15), 14);
                                lw_iy <= iy + 1;
                                lw_m3 <= m3lut(resize(lw_m3, 3) + 1);
                                lw_sr <= lw_sr + 1;
                            else
                                lw_va <= v_vaN(13 downto 0);
                                lw_iy <= iy;
                            end if;
                        end if;
                    when 301 =>
                        v_int := signed(resize(lw_va(13 downto 4), 11));
                        lw_dyA <= v_int - signed(resize(s_hh, 11));
                        if v_int < signed(resize(s_hh, 11)) then
                            lw_up <= '1';
                            if s_bdot = '1' then
                                lw_dyB <= v_int + signed(resize(s_hh, 11));
                            else
                                lw_dyB <= v_int;
                            end if;
                        else
                            lw_up <= '0';
                            if s_bdot = '1' then
                                lw_dyB <= v_int - signed(resize(s_hph, 11));
                            else
                                lw_dyB <= v_int - signed(resize(s_hpx, 11));
                            end if;
                        end if;
                    when 302 =>
                        n_up <= lw_up;
                        -- 89*iy for the hashes (line constant, both lanes)
                        n_jhy <= shift_left(resize(lw_iy, 10), 6) + shift_left(resize(lw_iy, 10), 4)
                               + shift_left(resize(lw_iy, 10), 3) + resize(lw_iy, 10);
                        n_m3 <= lw_m3;
                        if lw_up = '1' then
                            n_m3u <= m3lut(resize(lw_m3, 3) + 2);      -- iy-1 mod 3
                        else
                            n_m3u <= m3lut(resize(lw_m3, 3) + 1);      -- iy+1 mod 3
                        end if;
                        -- |dy| +/- the y-jitter quantum (0 when not scatter)
                        v_int := lw_dyA - signed(resize(s_jq, 11));
                        if v_int < 0 then
                            v_int := -v_int;
                        end if;
                        lw_adm <= unsigned(v_int(9 downto 0));
                        v_int := lw_dyA + signed(resize(s_jq, 11));
                        if v_int < 0 then
                            v_int := -v_int;
                        end if;
                        lw_adp <= unsigned(v_int(9 downto 0));
                        v_int := lw_dyB;
                        if v_int < 0 then
                            v_int := -v_int;
                        end if;
                        lw_adB <= unsigned(v_int(9 downto 0));
                    when 303 =>
                        smae <= resize(lw_adm, 20);
                        smb  <= resize(lw_adm, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                        if lw_up = '1' then
                            n_jhyb <= n_jhy - 89;
                        else
                            n_jhyb <= n_jhy + 89;
                        end if;
                    when 316 =>
                        n_dyA2m <= smp(15 downto 0);
                        smae <= resize(lw_adp, 20);
                        smb  <= resize(lw_adp, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 329 =>
                        n_dyA2p <= smp(15 downto 0);
                        smae <= resize(lw_adB, 20);
                        smb  <= resize(lw_adB, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 342 =>
                        n_dyB2 <= smp(17 downto 0);
                        -- layout-specific per-line product:
                        --   Tumble: row hash * W  ->  per-row lattice shift
                        --   Fade:   iy * fstep    ->  size drop for this row
                        if m_lay = 7 then
                            smae <= resize(s_fstep, 20);
                            smb  <= resize(lw_sr, 11);
                        else
                            smae <= resize(n_jhy(7 downto 0) xor n_jhy(9 downto 2), 20);
                            smb  <= resize(s_w, 11);
                        end if;
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 355 =>
                        if m_lay = 5 then
                            -- tumble: (hash8 * W) >> 8 is a wrapped row shift
                            n_lsh <= resize(smp(17 downto 8), 10);
                        else
                            n_lsh <= (others => '0');
                        end if;
                        lw_dt <= smp(9 downto 0);
                    when 356 =>
                        lw_r2 <= signed(resize(s_reff, 11)) - signed(resize(lw_dt, 11));
                    when 357 =>
                        -- fade: this row's radius, floor of 2 px
                        if lw_r2 < 2 then
                            lw_ro <= to_unsigned(2, 9);
                        else
                            lw_ro <= unsigned(lw_r2(8 downto 0));
                        end if;
                    when 358 =>
                        smae <= resize(lw_ro, 20);
                        smb  <= resize(lw_ro, 11);
                        smp  <= (others => '0');
                        sm_cnt <= (others => '0');
                        sm_run <= '1';
                    when 371 =>
                        if m_lay = 7 then
                            n_RA2 <= smp(16 downto 0);
                        else
                            n_RA2 <= s_RA2f;
                        end if;
                    when 372 =>
                        fsm_t <= (others => '1');
                        fsm_run <= '0';

                    when others => null;
                end case;
            end if;

            ------------------------------------------------------------------
            -- Triggers
            ------------------------------------------------------------------
            if prev_vsync_n = '1' and vs_r = '0' and anchor_seen = '1' then
                anchor_seen <= '0';
                fsm_t <= (others => '0');
                fsm_run <= '1';
            elsif r_arise = '1' and fsm_run = '0' then
                fsm_run <= '1';
                lf_seed <= '0';
                if r_anchor = '1' then
                    fsm_t <= to_unsigned(290, 9);
                else
                    fsm_t <= to_unsigned(300, 9);
                end if;
            end if;
        end if;
    end process p_fsm;

    --------------------------------------------------------------------------
    -- Pixel pipeline (c1..c17, one multiply per stage).
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_abs  : unsigned(9 downto 0);
        variable v_d    : signed(19 downto 0);
        variable v_a    : signed(11 downto 0);
        variable v_hx   : unsigned(9 downto 0);
        variable v_aA, v_aB : unsigned(6 downto 0);
        variable v_cov  : unsigned(6 downto 0);
        variable v_u    : signed(11 downto 0);
        variable v_dy10, v_du10, v_dv10 : unsigned(9 downto 0);
        variable v_o    : signed(11 downto 0);
        variable v_sh   : unsigned(17 downto 0);
        variable v_st   : std_logic;
        variable v_e    : unsigned(1 downto 0);
        variable v_a10  : unsigned(9 downto 0);
        variable v_ap   : std_logic;
        variable v_ce   : unsigned(1 downto 0);
        variable v_m3   : unsigned(1 downto 0);
        variable v_pw   : t_palv;
        variable v_jx   : signed(6 downto 0);
        variable v_dAr  : signed(18 downto 0);
        variable v_dBr  : signed(19 downto 0);
        variable v_dy2  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- video delay: EBR ring (chroma swapped ONCE at the input latch)
            vy_r <= data_in.y;
            vu_r <= data_in.u;
            vv_r <= data_in.v;
            vring(to_integer(wptr)) <= "00" & vy_r & vv_r & vu_r;
            vr_rd <= vring(to_integer(wptr - 14));
            vd_r  <= resize(unsigned(data_in.y(9 downto 1)), 10) + 32;   -- 64 + (y-64)/2
            dring(to_integer(wptr)) <= "000000" & std_logic_vector(vd_r);
            vd_rd <= dring(to_integer(wptr - 14));
            wptr  <= wptr + 1;
            sh_av(0) <= av_r;
            sh_hs(0) <= hs_r;
            sh_vs(0) <= vs_r;
            sh_fl(0) <= fl_r;
            for i in 1 to LATENCY - 1 loop
                sh_av(i) <= sh_av(i - 1);
                sh_hs(i) <= sh_hs(i - 1);
                sh_vs(i) <= sh_vs(i - 1);
                sh_fl(i) <= sh_fl(i - 1);
            end loop;

            ------------------------------------------------------------------
            -- c1: lane operand select; absolute indices; jitter-hash partial
            ------------------------------------------------------------------
            if (m_lay = 1 or m_lay = 2 or m_lay = 3) and iy(0) = '1' then
                c1_axl <= xb_loc;
                c1_bxl <= xa_loc;
                c1_ixa <= resize(xb_idx, 7) + ix0b;
                c1_ixb <= resize(xa_idx, 7) + ix0a;
            else
                c1_axl <= xa_loc;
                c1_bxl <= xb_loc;
                c1_ixa <= resize(xa_idx, 7) + ix0a;
                c1_ixb <= resize(xb_idx, 7) + ix0b;
            end if;
            -- 53 * ixA (selected lane-A lattice; shared by jitter + confetti)
            if (m_lay = 1 or m_lay = 2 or m_lay = 3) and iy(0) = '1' then
                v_hx := resize(xb_idx, 10) + resize(ix0b, 10);
            else
                v_hx := resize(xa_idx, 10) + resize(ix0a, 10);
            end if;
            c1_jp <= shift_left(resize(v_hx(6 downto 0), 10), 5)
                   + shift_left(resize(v_hx(6 downto 0), 10), 4)
                   + shift_left(resize(v_hx(6 downto 0), 10), 2)
                   + resize(v_hx(6 downto 0), 10);

            -- c2: signed dx; jitter hash fold -> jx / y-select
            c2_dxA <= signed(resize(c1_axl, 11)) - signed(resize(s_wh, 11));
            c2_dxB <= signed(resize(c1_bxl, 11)) - signed(resize(s_wh, 11));
            v_hx := c1_jp + l_jhy + to_unsigned(173, 10);
            v_hx := v_hx xor shift_right(v_hx, 5);
            c2_ha <= v_hx;
            if s_jon = '1' then
                case to_integer(s_jsh) is
                    when 0      => v_jx := resize(signed(v_hx(3 downto 0)), 7);
                    when 1      => v_jx := shift_left(resize(signed(v_hx(3 downto 0)), 7), 1);
                    when 2      => v_jx := shift_left(resize(signed(v_hx(3 downto 0)), 7), 2);
                    when others => v_jx := shift_left(resize(signed(v_hx(3 downto 0)), 7), 3);
                end case;
                c2_jx  <= v_jx;
                c2_jys <= v_hx(9);
            elsif s_clu = '1' then
                -- cluster: neighbouring columns/rows huddle in 2x2 groups
                if c1_ixa(0) = '1' then
                    c2_jx <= -signed(resize(s_jclu, 7));
                else
                    c2_jx <= signed(resize(s_jclu, 7));
                end if;
                c2_jys <= iy(0);
            else
                c2_jx  <= (others => '0');
                c2_jys <= '0';
            end if;
            c2_ixa <= c1_ixa;
            c2_ixb <= c1_ixb;
            if l_up = '1' then
                c2_iyb <= iy - 1;
            else
                c2_iyb <= iy + 1;
            end if;

            -- c3: apply x jitter, abs; colour-hash partials; parities
            v_a := resize(c2_dxA, 12) - resize(c2_jx, 12);
            if v_a < 0 then
                v_a := -v_a;
            end if;
            c3_sqA <= C_SQ8(to_integer(unsigned(v_a(7 downto 0))));
            v_a := resize(c2_dxB, 12);
            if v_a < 0 then
                v_a := -v_a;
            end if;
            c3_sqB <= C_SQ8(to_integer(unsigned(v_a(7 downto 0))));
            c3_jys <= c2_jys;
            c3_ha <= c2_ha;
            c3_hxb <= shift_left(resize(c2_ixb, 10), 5) + shift_left(resize(c2_ixb, 10), 4)
                    + shift_left(resize(c2_ixb, 10), 2) + resize(c2_ixb, 10)
                    + to_unsigned(173, 10);
            c3_hyb <= l_jhyb;
            c3_apA <= c2_ixa(0) xor iy(0);
            c3_apB <= c2_ixb(0) xor c2_iyb(0);

            -- c4: squares (ROM data re-registered); hash combine
            c4_sqA <= unsigned(c3_sqA);
            c4_sqB <= unsigned(c3_sqB);
            c4_jys <= c3_jys;
            c4_hA  <= c3_ha;
            c4_h1B <= c3_hxb + c3_hyb;
            c4_apA <= c3_apA;
            c4_apB <= c3_apB;

            -- c5: squared sums (lane A picks the y-jitter dy^2 variant)
            if c4_jys = '1' then
                v_dy2 := l_dyA2p;
            else
                v_dy2 := l_dyA2m;
            end if;
            c5_sumA <= resize(c4_sqA, 17) + resize(v_dy2, 17);
            c5_sumB <= resize(c4_sqB, 19) + resize(l_dyB2, 19);
            c5_hA <= c4_hA;
            c5_hB <= c4_h1B xor shift_right(c4_h1B, 5);
            c5_apA <= c4_apA;
            c5_apB <= c4_apB;

            -- c6: R^2 - d^2 clamp0; per-lane confetti entry
            v_dAr := signed(resize(l_RA2, 19)) - signed(resize(c5_sumA, 19));
            if v_dAr < 0 then
                c6_dA <= (others => '0');
            else
                c6_dA <= unsigned(v_dAr(17 downto 0));
            end if;
            v_dBr := signed(resize(l_RB2, 20)) - signed(resize(c5_sumB, 20));
            if v_dBr < 0 then
                c6_dB <= (others => '0');
            else
                c6_dB <= unsigned(v_dBr(17 downto 0));
            end if;
            case to_integer(c5_hA(2 downto 0) xor c5_hA(5 downto 3)) is
                when 0 | 3 | 6 => c6_ceA <= "01";
                when 1 | 4 | 7 => c6_ceA <= "10";
                when others    => c6_ceA <= "11";
            end case;
            case to_integer(c5_hB(2 downto 0) xor c5_hB(5 downto 3)) is
                when 0 | 3 | 6 => c6_ceB <= "01";
                when 1 | 4 | 7 => c6_ceB <= "10";
                when others    => c6_ceB <= "11";
            end case;
            c6_apA <= c5_apA;
            c6_apB <= c5_apB;

            -- c7: coarse normalise shift + saturation flag; entry mux
            case to_integer(s_shA(4 downto 2)) is
                when 0      => v_sh := c6_dA;                 v_st := '0';
                when 1      => v_sh := c6_dA sll 4;
                               v_st := b2sl(c6_dA(17 downto 14) /= 0);
                when 2      => v_sh := c6_dA sll 8;
                               v_st := b2sl(c6_dA(17 downto 10) /= 0);
                when 3      => v_sh := c6_dA sll 12;
                               v_st := b2sl(c6_dA(17 downto 6) /= 0);
                when others => v_sh := c6_dA sll 16;
                               v_st := b2sl(c6_dA(17 downto 2) /= 0);
            end case;
            c7_shA  <= v_sh;
            c7_satA <= v_st;
            case to_integer(s_shB(4 downto 2)) is
                when 0      => v_sh := c6_dB;                 v_st := '0';
                when 1      => v_sh := c6_dB sll 4;
                               v_st := b2sl(c6_dB(17 downto 14) /= 0);
                when 2      => v_sh := c6_dB sll 8;
                               v_st := b2sl(c6_dB(17 downto 10) /= 0);
                when 3      => v_sh := c6_dB sll 12;
                               v_st := b2sl(c6_dB(17 downto 6) /= 0);
                when others => v_sh := c6_dB sll 16;
                               v_st := b2sl(c6_dB(17 downto 2) /= 0);
            end case;
            c7_shB  <= v_sh;
            c7_satB <= v_st;
            -- per-lane entry SELECTORS ride the pipe; the mode mux happens
            -- once, after the winner is known
            c7_entA <= c6_apA & c6_ceA(0);
            c7_entB <= c6_apB & c6_ceB(0);
            c7_ceA  <= c6_ceA;
            c7_ceB  <= c6_ceB;

            -- c8: fine shift, extract 6-bit byte + saturation
            case to_integer(s_shA(1 downto 0)) is
                when 0      => v_sh := c7_shA;                v_st := c7_satA;
                when 1      => v_sh := c7_shA sll 1;
                               v_st := c7_satA or c7_shA(17);
                when 2      => v_sh := c7_shA sll 2;
                               v_st := c7_satA or b2sl(c7_shA(17 downto 16) /= 0);
                when others => v_sh := c7_shA sll 3;
                               v_st := c7_satA or b2sl(c7_shA(17 downto 15) /= 0);
            end case;
            c8_byA  <= v_sh(17 downto 12);
            c8_satA <= v_st;
            case to_integer(s_shB(1 downto 0)) is
                when 0      => v_sh := c7_shB;                v_st := c7_satB;
                when 1      => v_sh := c7_shB sll 1;
                               v_st := c7_satB or c7_shB(17);
                when 2      => v_sh := c7_shB sll 2;
                               v_st := c7_satB or b2sl(c7_shB(17 downto 16) /= 0);
                when others => v_sh := c7_shB sll 3;
                               v_st := c7_satB or b2sl(c7_shB(17 downto 15) /= 0);
            end case;
            c8_byB  <= v_sh(17 downto 12);
            c8_satB <= v_st;
            c8_entA <= c7_entA;
            c8_entB <= c7_entB;
            c8_ceA  <= c7_ceA;
            c8_ceB  <= c7_ceB;

            -- c9: alpha gain multiplies
            c9_pA <= resize(c8_byA * s_gA, 13);
            c9_pB <= resize(c8_byB * s_gB, 13);
            c9_satA <= c8_satA;
            c9_satB <= c8_satB;
            c9_entA <= c8_entA;
            c9_entB <= c8_entB;
            c9_ceA  <= c8_ceA;
            c9_ceB  <= c8_ceB;
            c9_m3A  <= l_m3;
            c9_m3B  <= l_m3u;

            -- c10: alphas, coverage combine, winner selectors
            if c9_satA = '1' or c9_pA(12 downto 5) > 63 then
                v_aA := to_unsigned(63, 7);
            else
                v_aA := resize(c9_pA(11 downto 5), 7);
            end if;
            if c9_satB = '1' or c9_pB(12 downto 5) > 63 then
                v_aB := to_unsigned(63, 7);
            else
                v_aB := resize(c9_pB(11 downto 5), 7);
            end if;
            if s_bdot = '1' then
                if v_aB > v_aA then
                    v_cov := v_aB;
                else
                    v_cov := v_aA;
                end if;
            else
                if v_aA < to_unsigned(63, 7) - v_aB then
                    v_cov := v_aA;
                else
                    v_cov := to_unsigned(63, 7) - v_aB;
                end if;
            end if;
            c10_cov <= v_cov;
            if s_bdot = '1' and v_aB > v_aA then
                c10_ap <= c9_entB(1);
                c10_ce <= c9_ceB;
                c10_m3 <= c9_m3B;
            else
                c10_ap <= c9_entA(1);
                c10_ce <= c9_ceA;
                c10_m3 <= c9_m3A;
            end if;

            -- c11: colour-mode entry mux, palette lookup, tint add
            case to_integer(m_mode) is
                when 1 =>                                      -- Duotone
                    if c10_ap = '1' then
                        v_e := "10";
                    else
                        v_e := s_ed1;
                    end if;
                when 2 =>                                      -- Rows
                    case to_integer(c10_m3) is
                        when 0      => v_e := s_ed1;
                        when 1      => v_e := "10";
                        when others => v_e := "11";
                    end case;
                when 3 =>                                      -- Confetti
                    if c10_ce = "01" then
                        v_e := s_ed1;
                    else
                        v_e := c10_ce;
                    end if;
                when others =>                                 -- Solid / Cycle
                    v_e := s_ed1;
            end case;
            if pal_hij = '1' then
                v_e := hij_e;
            end if;
            v_pw := C_PALV(to_integer(s_pal & v_e));
            c11_hue <= unsigned(v_pw(9 downto 0));
            c11_sat <= unsigned(v_pw(18 downto 10));
            c11_y   <= unsigned(v_pw(28 downto 19));
            c11_cov <= c10_cov;
            c11_e   <= v_e;

            -- c12: sine ROM read (the FSM borrows the colour path during the
            -- vsync cone via pal_hij to fetch the background colour)
            v_a10 := c11_hue + s_hueoff;
            rom_rd  <= C_SINCOS(to_integer(v_a10(9 downto 1)));
            c12_e   <= c11_e;
            c12_y   <= c11_y;
            c12_cov <= c11_cov;

            -- c13: second fabric register after the EBR ROM
            c13_su  <= signed(rom_rd(19 downto 10));
            c13_sv  <= signed(rom_rd(9 downto 0));
            c13_e   <= c12_e;
            c13_y   <= c12_y;
            c13_cov <= c12_cov;

            -- c14: saturation multiplies
            c14_e   <= c13_e;
            c14_y   <= c13_y;
            c14_cov <= c13_cov;

            -- c15: compose dot colour; fg/bg source select
            v_du10 := pt_u(to_integer(c14_e));
            v_dv10 := pt_v(to_integer(c14_e));
            v_dy10 := c14_y;
            if s_port = '1' then
                c14_fgy <= unsigned(vr_rd(29 downto 20));
                c14_fgu <= unsigned(vr_rd(19 downto 10));
                c14_fgv <= unsigned(vr_rd(9 downto 0));
                if s_bgvid = '1' then
                    -- spotlight: colour portholes in a dim grey video field
                    c14_bgy <= unsigned(vd_rd(9 downto 0));
                    c14_bgu <= C_MID;
                    c14_bgv <= C_MID;
                else
                    c14_bgy <= s_bgy;
                    c14_bgu <= s_bgu;
                    c14_bgv <= s_bgv;
                end if;
            else
                c14_fgy <= v_dy10;
                c14_fgu <= v_du10;
                c14_fgv <= v_dv10;
                if s_bgvid = '1' then
                    c14_bgy <= unsigned(vr_rd(29 downto 20));
                    c14_bgu <= unsigned(vr_rd(19 downto 10));
                    c14_bgv <= unsigned(vr_rd(9 downto 0));
                else
                    c14_bgy <= s_bgy;
                    c14_bgu <= s_bgu;
                    c14_bgv <= s_bgv;
                end if;
            end if;
            c14_cov2 <= c14_cov;

            -- c16: deltas
            c15_dy <= signed(resize(c14_fgy, 11)) - signed(resize(c14_bgy, 11));
            c15_du <= signed(resize(c14_fgu, 11)) - signed(resize(c14_bgu, 11));
            c15_dv <= signed(resize(c14_fgv, 11)) - signed(resize(c14_bgv, 11));
            c15_bgy <= c14_bgy;
            c15_bgu <= c14_bgu;
            c15_bgv <= c14_bgv;
            c15_cov <= c14_cov2;

            -- c17: blend multiplies
            c16_my  <= booth_d11_c6(c15_dy, c15_cov(5 downto 0));
            c16_mu2 <= booth_d11_c6(c15_du, c15_cov(5 downto 0));
            c16_mv2 <= booth_d11_c6(c15_dv, c15_cov(5 downto 0));
            c16_bgy <= c15_bgy;
            c16_bgu <= c15_bgu;
            c16_bgv <= c15_bgv;

            -- c18: recombine, blanking gate, sync attach (output reg)
            if sh_av(LATENCY - 1) = '0' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                -- alpha blend result is bounded by [min(fg,bg), max(fg,bg)]
                v_o := signed(resize(c16_bgy, 12)) + resize(shift_right(c16_my, 6), 12);
                s_io.y <= std_logic_vector(v_o(9 downto 0));
                v_o := signed(resize(c16_bgu, 12)) + resize(shift_right(c16_mu2, 6), 12);
                s_io.u <= std_logic_vector(v_o(9 downto 0));
                v_o := signed(resize(c16_bgv, 12)) + resize(shift_right(c16_mv2, 6), 12);
                s_io.v <= std_logic_vector(v_o(9 downto 0));
            end if;
            s_io.hsync_n <= sh_hs(LATENCY - 1);
            s_io.vsync_n <= sh_vs(LATENCY - 1);
            s_io.avid    <= sh_av(LATENCY - 1);
            s_io.field_n <= sh_fl(LATENCY - 1);
        end if;
    end process p_pipe;

    -- chroma swapped ONCE at output compose (dry path stays identity)
    s_io_align_in.y       <= s_io.y;
    s_io_align_in.u       <= s_io.v;
    s_io_align_in.v       <= s_io.u;
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

end architecture polka;
