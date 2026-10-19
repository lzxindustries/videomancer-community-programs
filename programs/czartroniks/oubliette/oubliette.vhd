-- oubliette.vhd
-- Copyright (C) 2026  czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Oubliette - a top-down, turn-based roguelike for the Videomancer.
--
-- Driven entirely by the front panel (6 knobs, 5 switches, 1 slider): pick a
-- class, explore procedurally generated dungeon levels, fight monsters in
-- turn-based combat, loot treasure, and descend.  Pure synthesis - the incoming
-- video is ignored except for its sync, to which the output is genlocked.
--
-- Milestones:
--   M0  skeleton            M1  static tile map      M2  camera + player
--   M3  monsters + combat + HUD      M4  procedural generation
--   M5  items / win-lose                                                 <-- v0.3.0
--   Tier 3 batch A (feel)                                                <-- v0.4.0
--   monster glide + space reclaim                                        <-- v0.4.1
--   Tier 3 batch B (look) + colour parity fix                            <-- v0.5.0
--
-- v1.0.3 (2026-10-17): resolution scaling.  The screen coordinates run in
--   virtual 1080-line units, stepped per pixel / per field line from the
--   video timing ID (registers_in(8)): 1.0 at 1080p (bit-identical to
--   v1.0.2), 1.5 at 720p, and at SD 2.0 across (a 1440-wide 4:3 window) and
--   1080/486 per frame line (interlaced fields step two frame lines; the
--   hardware ties field_n high, so both fields sample alike and the
--   field_n = '0' half-step offset is inert there).  Cells, sprites, HUD, logo,
--   HP bar, hurt border and Map view keep their 1080p proportions and stay
--   square on a 4:3 screen.  The HUD line is a 1920-unit layout band at
--   every resolution (its own x step: at SD the glyphs are 3/4 wide), so all
--   48 columns and the HP bar fit the SD screen.  The band flags are
--   magnitude windows (a step can jump an equality); the y bands are
--   computed a line ahead.  Layout uses the screen in virtual units; the
--   generated avid keeps the real measured width.  At 480i a field line
--   (4.44 units) is longer than a HUD font row (4), so the HUD lines run one
--   font row per field line (8 lines, ~11% taller than proportional) rather
--   than skip a row of every glyph.  The reticle outline is at least one
--   field step wide on each axis.
--
-- v0.5.0 (2026-10-10): output avid moved to aurora's hsync distance (the
--   colours had been U/V-swapped since v0.2.0: yellow reticle showed cyan);
--   P12 top = N; room furniture (pillars, pools, lava, rubble) + doors; HP
--   bar; full-width HUD; K6 palette themes (S8) + per-depth tint; title
--   screen (big class sprite under a 2x OUBLIETTE logo); palette ROM (S6)
--   with precomputed dim colours replacing the colour mux and dim adders.
--
-- v0.4.0 (2026-10-10): fog of war + torchlight (K5 Torch: radius 2-8 or
--   full light; a seen-cell RAM, remembered cells dimmed, monsters only in
--   the light), zoom (K4: 16/32/64/128 px cells) and Map view (S8: the
--   whole level), smooth walking (the camera glides 1/8 cell per field, the
--   player drawn at the screen centre in target coordinates), hit feedback
--   (monster flash, red border when hurt, pulsing at low HP), facing
--   (mirror toward heading / the player) and idle / walking bob.  One more
--   pipeline stage (S7 dim).
--
-- v0.3.0 (2026-10-10): the game is finishable.  Class casts on K3 (Warrior
--   BASH stuns, Mage FIRE ignores DEF, Rogue STAB crits, Cleric HEAL) paid
--   in MP (+1 every 4th turn, +2 on Defend); potions (S9, or K3 Item) from
--   chests and drops; depth-weighted monsters whose stats grow with depth;
--   the dragon guards depth 7 and killing it wins; monsters chase within 6
--   cells (else wander) and sidestep once when blocked; fleeing buys two
--   free turns; level-ups raise HP/ATK/DEF/MP.  HUD text now comes from a
--   generated layout ROM (oubliette_hud_pkg) via a field RAM.
--
-- v0.2.0 (2026-10-10): hardware video contracts (never flashed before):
--   generated parity-locked avid (aurora/cubist recipe), blanking gate,
--   studio-range palette (black 64, luma <= ~768), once-per-field tick with
--   the saw-active guard, camera refreshed through vblank (was one frame
--   late on every step), control hysteresis, labelled K1 Class, and
--   generator soft-lock fixes.  Every action (turn, combat round, new game)
--   now resolves at the field tick, inside vertical blank.
--
-- =====================================================================
-- Register / front-panel map
--   K1 Class (labelled) | K2 Seed | K3 Action (labelled) | K4 Zoom (labelled)
--   K5 Torch
--   S7 Step/Act (edge=turn) | S8 Map | S9 Potion (edge) | S10 Peaceful
--   S11 New game (edge) | P12 Heading
--   K6 Palette (labelled: Colour / Game Boy / Amber / CGA / C64)
-- =====================================================================
--
-- Colour: BT.601 YUV stored U/V-SWAPPED for the HW convention.
--
-- Render pipeline (one pixel/clock; 64 px power-of-two cells, 16 px art):
--   S0 p_acc   world + player coords, HUD strips, border band (GENERATED avid)
--   S1 p_cell  zoomed cell decode / in-player / reticle compares
--   S2 p_map   tilemap + seen reads ||  p_ent  entity match, torch offset
--   S3 p_spr   sprite ROM read, |offset|
--   S4 p_trow  tile ROM read, torch core / ring
--   S5 p_sel   layer select -> 7-bit palette address (blanking gate: outside
--              avid -> the BLANK entry, 64/512/512)
--   S6 p_color palette ROM read (EBR; dimmed tile colours precomputed)
--   S7 p_dim   fabric copy of the ROM word
--   S8 p_theme per-depth tint / K6 palette theme
-- Syncs: 9 clocks from data_in in every mode; the generated avid rides the
-- pixel pipeline itself, so avid and pixels are aligned by construction.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.oubliette_tiles_pkg.all;
use work.oubliette_sprites_pkg.all;
use work.oubliette_font_pkg.all;
use work.oubliette_tables_pkg.all;
use work.oubliette_hud_pkg.all;
use work.oubliette_pal_pkg.all;

architecture oubliette of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (10 + 2 = 12 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    constant C_MAP_LOG2  : integer := 6;
    constant C_MAP       : integer := 2 ** C_MAP_LOG2;            -- 64
    -- On-screen cell size = 16 << s_cl px (K4 Zoom: 16/32/64/128; Map view
    -- 16).  The art is 16x16, scaled up nearest-neighbour; power-of-two cells
    -- keep screen<->cell a bit-slice (cell_dec, no dividers).
    constant C_MONS      : integer := 6;                          -- monster slots
    constant C_MON_BASE  : integer := C_S_RAT;                    -- type t -> sprite base+t

    -- sync delay line length.  Syncs reach data_out C_SYNCD+6 clocks after
    -- data_in and the generated avid g_avid+10, the same avid-vs-hsync distance
    -- as aurora / cubist, where the U/V-swapped palette is HW-correct.  (Until
    -- v0.4.1 the syncs ran one clock later: the opposite chroma pairing, so
    -- every colour showed U/V-swapped: the yellow reticle came out cyan.)
    constant C_SYNCD : integer := 4;

    -- which layer wins a pixel (decided in the select stage, coloured in lookup)

    constant C_SPAWN_X : integer := 16;
    constant C_SPAWN_Y : integer := 16;

    -- All drawn colours are studio range (black 64, luma <= ~768) and live in
    -- the palette ROM (oubliette_pal_pkg, from oubliette.py yuv_swapped()).
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- HUD: a 48-column context line at the top and a status bar at the
    -- bottom (8x8 font scaled to 2**C_HUD_LOG2 units).  All text comes from the
    -- generated layout ROM (oubliette_hud_pkg: C_HUDN, layouts, fields, names).
    -- Sizes and positions are virtual 1080-line units (= px at 1080p; see
    -- f_vstep_x / f_vstep_y): they scale with the screen height.
    constant C_HUD_LOG2 : integer := 5;         -- 32-unit glyph cell (8x8 font at 4x)
    constant C_HUD_H    : integer := 2 ** C_HUD_LOG2;  -- bar height (units)
    constant C_HUD_MARG : integer := 4;         -- top/bottom margin (units)
    -- hit feedback: monster hit flash, screen-edge border when hurt and
    -- pulsing at low HP (colours in the palette ROM)
    constant C_EDGE    : integer := 24;            -- border width (units)
    -- HP bar: right part of the bottom HUD line (HUD x: a 1920-unit line at
    -- every resolution), 3 units per HP point (no divide);
    -- green above half, amber above a quarter, red below
    constant C_BAR_X0 : integer := 1552;
    -- K6 palette themes: luma quantised to 4 levels, each mapped to a 4-colour
    -- table (1 Game Boy, 2 amber, 3 CGA, 4 C64); 0 = full colour
    type t_th is array (0 to 15) of integer range 0 to 1023;   -- (theme-1)*4 + level
    constant C_TH_Y : t_th := (198, 330, 559, 608,   111, 290, 515, 681,
                               64, 597, 693, 768,    64, 284, 476, 768);
    constant C_TH_U : t_th := (452, 438, 499, 503,   533, 623, 684, 648,
                               512, 762, 244, 512,   512, 512, 512, 512);
    constant C_TH_V : t_th := (464, 454, 256, 228,   485, 382, 271, 283,
                               512, 710, 602, 512,   512, 665, 663, 512);

    -- 5-zone knob decode (labelled K3 / K6), indexed by the knob's top 6 bits:
    -- zone 0..4, or 5 = hold (the 16-code unit straddling each label
    -- boundary 205 / 410 / 614 / 819).  A LUT ROM instead of eight carry-chain
    -- compares per knob.
    type t_z5 is array (0 to 63) of integer range 0 to 5;
    function build_z5 return t_z5 is
        variable t : t_z5;
    begin
        for u in 0 to 63 loop
            if    u <= 11 then t(u) := 0;
            elsif u = 12  then t(u) := 5;
            elsif u <= 24 then t(u) := 1;
            elsif u = 25  then t(u) := 5;
            elsif u <= 37 then t(u) := 2;
            elsif u = 38  then t(u) := 5;
            elsif u <= 50 then t(u) := 3;
            elsif u = 51  then t(u) := 5;
            else               t(u) := 4;
            end if;
        end loop;
        return t;
    end function;
    constant C_Z5 : t_z5 := build_z5;

    -- squares 0..15 for the torch distance test (a LUT ROM, no multiplier)
    type t_sq is array (0 to 15) of integer range 0 to 225;
    constant C_SQ : t_sq := (0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 169, 196, 225);

    --------------------------------------------------------------------------
    -- Resolution scaling (v1.0.3): the screen coordinates run in virtual
    -- 1080-line units, stepped per pixel / per field line from the video
    -- timing ID (registers_in(8)).  1080p steps 1.0 (the old pixel counters).
    --------------------------------------------------------------------------
    -- x step in half units per pixel: 1080 2 (1.0), 720p 3 (1.5), SD 4 (2.0:
    -- 720 px span 1440 units, a 4:3 window of the 1080 canvas).
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- HUD x step in 1/16 units per pixel: the HUD line is a 1920-unit layout
    -- band at every resolution (SD 43/16: 720 px span 1935 units), so all 48
    -- columns and the HP bar stay on the narrower SD screen (glyphs 3/4 wide).
    function f_hstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(43, 6);
            when 5 | 6 | 14     => return to_unsigned(24, 6);
            when others         => return to_unsigned(16, 6);
        end case;
    end function;
    -- y step in Q4.8 units per FIELD line (1080 / active lines, doubled for
    -- interlaced modes).
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
    -- reticle outline width in art pixels (cell = 16 << cl units, art pixel
    -- 1 << cl): at least one step on each axis, so a fractional step can't
    -- skip the 1-art-pixel rim (1080p: always 1, the old outline).
    function f_rim_x(t : std_logic_vector(3 downto 0); cl : unsigned(1 downto 0)) return unsigned is
    begin
        if cl = "00" then
            case to_integer(unsigned(t)) is
                when 0 | 4 | 8 | 12 | 5 | 6 | 14 => return to_unsigned(2, 3);
                when others                      => return to_unsigned(1, 3);
            end case;
        end if;
        return to_unsigned(1, 3);
    end function;
    function f_rim_y(t : std_logic_vector(3 downto 0); cl : unsigned(1 downto 0)) return unsigned is
    begin
        case cl is
            when "00" =>
                case to_integer(unsigned(t)) is
                    when 0                     => return to_unsigned(5, 3);  -- 4.44
                    when 8                     => return to_unsigned(4, 3);  -- 3.75
                    when 4                     => return to_unsigned(3, 3);  -- 2.25
                    when 5 | 6 | 14 | 1 | 2 | 10 | 12 => return to_unsigned(2, 3);
                    when others                => return to_unsigned(1, 3);
                end case;
            when "01" =>
                case to_integer(unsigned(t)) is
                    when 0          => return to_unsigned(3, 3);
                    when 4 | 8      => return to_unsigned(2, 3);
                    when others     => return to_unsigned(1, 3);
                end case;
            when "10" =>
                if to_integer(unsigned(t)) = 0 then return to_unsigned(2, 3); end if;
                return to_unsigned(1, 3);
            when others =>
                return to_unsigned(1, 3);
        end case;
    end function;

    -- cell decode at zoom cl (cell = 16 << cl px): map column/row, art pixel
    -- (16 per cell, nearest-neighbour) and in-bounds.  Bit slices only.
    procedure cell_dec(w : in signed(13 downto 0); cl : in unsigned(1 downto 0);
                       col : out unsigned(5 downto 0); g : out unsigned(3 downto 0);
                       inb : out std_logic) is
    begin
        case cl is
            when "00" =>
                col := unsigned(w(9 downto 4));  g := unsigned(w(3 downto 0));
                if w(13 downto 10) = "0000" then inb := '1'; else inb := '0'; end if;
            when "01" =>
                col := unsigned(w(10 downto 5)); g := unsigned(w(4 downto 1));
                if w(13 downto 11) = "000" then inb := '1'; else inb := '0'; end if;
            when "10" =>
                col := unsigned(w(11 downto 6)); g := unsigned(w(5 downto 2));
                if w(13 downto 12) = "00" then inb := '1'; else inb := '0'; end if;
            when others =>
                col := unsigned(w(12 downto 7)); g := unsigned(w(6 downto 3));
                inb := not w(13);
        end case;
    end procedure;

    -- glide decode: offset o = w mod cell; o + f and o - f (f <= one cell):
    -- carry c (o+f reaches the next cell) / borrow b (o-f falls into the
    -- previous one), and the art pixel of each.  8-bit arithmetic only.
    procedure glide_dec(w : in signed(13 downto 0); cl : in unsigned(1 downto 0);
                        f : in unsigned(7 downto 0);
                        c : out std_logic; gp : out unsigned(3 downto 0);
                        b : out std_logic; gm : out unsigned(3 downto 0)) is
        variable o  : unsigned(7 downto 0);
        variable sp : unsigned(7 downto 0);
        variable sm : unsigned(8 downto 0);
    begin
        case cl is
            when "00"   => o := "0000" & unsigned(w(3 downto 0));
            when "01"   => o := "000"  & unsigned(w(4 downto 0));
            when "10"   => o := "00"   & unsigned(w(5 downto 0));
            when others => o := "0"    & unsigned(w(6 downto 0));
        end case;
        sp := o + f;                              -- < 2 cells <= 255
        sm := ('0' & o) - ('0' & f);              -- 9-bit two's complement
        b  := sm(8);
        case cl is
            when "00"   => c := sp(4); gp := sp(3 downto 0); gm := sm(3 downto 0);
            when "01"   => c := sp(5); gp := sp(4 downto 1); gm := sm(4 downto 1);
            when "10"   => c := sp(6); gp := sp(5 downto 2); gm := sm(5 downto 2);
            when others => c := sp(7); gp := sp(6 downto 3); gm := sm(6 downto 3);
        end case;
    end procedure;

    --------------------------------------------------------------------------
    -- Combat stat tables (small -> kept as constants; all combat math is
    -- adds / compares / saturating subtract + LFSR variance, no multiply).
    --------------------------------------------------------------------------
    type t_cls is array (0 to 3) of integer;
    constant C_CLS_HP   : t_cls := (24, 16, 18, 20);  -- War Mag Rog Clr
    constant C_CLS_ATK  : t_cls := (6, 4, 5, 4);
    constant C_CLS_DEF  : t_cls := (4, 2, 3, 3);
    constant C_CLS_MP   : t_cls := (6, 12, 6, 10);
    -- class cast (K3 "Cast"): War BASH = hit + the monster loses its turn,
    -- Mag FIRE = ATK+8 ignoring DEF, Rog STAB = 50% crit for 2x ATK,
    -- Clr HEAL = +maxHP/2.  MP cost:
    constant C_CLS_COST : t_cls := (3, 4, 2, 4);
    constant C_CLS_ROG  : integer := 2;          -- rogue: flee always works

    -- monster types: 0 rat 1 skeleton 2 slime 3 bat 4 orc 5 wraith 6 DRAGON
    -- (the depth-7 boss: stationary, its death wins the game).  Stats grow
    -- with depth: HP +2/level, ATK +1, DEF +1 per 2, XP +1 (see s_dbon_*).
    constant C_BOSS : integer := 6;
    type t_mon is array (0 to 6) of integer;
    constant C_MON_HP  : t_mon := (4, 8, 6, 5, 12, 10, 48);
    constant C_MON_ATK : t_mon := (2, 4, 3, 3, 5, 6, 10);
    constant C_MON_DEF : t_mon := (1, 2, 1, 1, 3, 2, 4);
    constant C_MON_XP  : t_mon := (2, 5, 4, 3, 8, 10, 0);

    -- monsters chase only within this many cells (Chebyshev); farther ones wander
    constant C_AGGRO : integer := 6;

    -- depth-weighted spawn table: 4 candidates per depth, weakest first
    function mon_pool(d : unsigned(2 downto 0); r : std_logic_vector(1 downto 0))
        return integer is
        variable t : integer range 0 to 5;
    begin
        case to_integer(d) is
            when 0 | 1 => case r is when "00" | "01" => t := 0; when "10" => t := 3; when others => t := 2; end case;
            when 2 =>     case r is when "00" => t := 0; when "01" => t := 3; when "10" => t := 2; when others => t := 1; end case;
            when 3 =>     case r is when "00" => t := 3; when "01" => t := 2; when others => t := 1; end case;
            when 4 =>     case r is when "00" => t := 2; when "10" => t := 4; when others => t := 1; end case;
            when 5 =>     case r is when "00" => t := 1; when "11" => t := 5; when others => t := 4; end case;
            when 6 =>     case r is when "11" => t := 1; when "10" => t := 5; when others => t := 4; end case;
            when others => case r is when "01" | "10" => t := 5; when others => t := 4; end case;
        end case;
        return t;
    end function;

    --------------------------------------------------------------------------
    -- Timing / resolution
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_measured_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_measured_v : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_firstline : std_logic := '1';   -- vsync start .. first active line
    -- once-per-field tick: the first vsync edge after active video (analog
    -- vsync serrates into several edges per field - saw-active guard)
    signal s_saw   : std_logic := '0';
    signal s_ftick : std_logic := '0';
    -- generated avid (aurora/cubist recipe): starts 3-4 clocks after the
    -- source's avid rise at a FIXED hsync->start parity, ends after the
    -- field-latched width.  The encoders pair Cb/Cr by counting from hsync,
    -- so a source avid that wanders a clock swaps U/V for that line.
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_gav_start : std_logic;
    -- Resolution scaling: virtual screen coordinates (1080-line canvas).
    -- s_xa Q.1 (world, logo, border), s_hxa Q.4 (HUD line: 1920 units wide
    -- at every resolution), s_ya Q.8 per field line; s_yn = the next line's
    -- y, so the per-line band flags are ready before the line starts.
    -- s_vw / s_vh = the screen in virtual units (layout only; the generated
    -- avid keeps the real measured width).
    signal s_xs  : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_hxs : unsigned(5 downto 0)  := to_unsigned(16, 6);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xa  : unsigned(12 downto 0) := (others => '0');
    signal s_hxa : unsigned(15 downto 0) := (others => '0');
    signal s_ya  : unsigned(19 downto 0) := (others => '0');
    signal s_yn  : unsigned(19 downto 0) := (others => '0');
    signal s_fbot : std_logic := '0';      -- bottom field (field_n = '0', SD only)
    signal s_fbcur : std_logic := '0';     -- this field started half a step down
    signal s_tid  : std_logic_vector(3 downto 0) := x"D";   -- timing ID, per field
    signal s_rx, s_ry : unsigned(2 downto 0) := to_unsigned(1, 3);  -- reticle rim (art px)
    signal s_vw  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vh  : unsigned(11 downto 0) := to_unsigned(1080, 12);
    -- next line's band flags / glyph rows (from s_yn)
    signal s_n_ht, s_n_hb, s_n_ye, s_n_ly : std_logic := '0';
    signal s_n_hc  : unsigned(4 downto 0) := (others => '0');
    signal s_n_lcy : unsigned(5 downto 0) := (others => '0');
    -- field step longer than a HUD font row (4 units: 480i only): the HUD
    -- lines take one font row per field line instead (see p_acc)
    signal s_hsnap : std_logic := '0';

    --------------------------------------------------------------------------
    -- Player / camera state
    --------------------------------------------------------------------------
    signal s_px : unsigned(5 downto 0) := to_unsigned(C_SPAWN_X, 6);
    signal s_py : unsigned(5 downto 0) := to_unsigned(C_SPAWN_Y, 6);
    signal s_class : unsigned(1 downto 0) := (others => '0');
    signal s_cls_sel : unsigned(1 downto 0) := (others => '0');  -- K1 decoded (hysteresis)
    signal s_dir   : unsigned(2 downto 0) := (others => '0');  -- 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW
    -- camera: s_pscroll = target (player cell centred, or the whole map in
    -- Map view), refreshed through vblank; s_scroll = the eased world scroll
    -- that glides toward it (snaps on jumps > 2 cells: new level, zoom, map)
    signal s_scroll_x  : signed(13 downto 0) := (others => '0');
    signal s_scroll_y  : signed(13 downto 0) := (others => '0');
    signal s_pscroll_x : signed(13 downto 0) := (others => '0');
    signal s_pscroll_y : signed(13 downto 0) := (others => '0');
    signal s_ecnt : unsigned(9 downto 0) := (others => '0');
    signal s_ev   : unsigned(7 downto 0) := (others => '0');     -- glide step per field
    signal s_moving : std_logic := '0';
    -- monster glide: every monster that moved this turn keeps its step
    -- direction (s_mdx/s_mdy: "01" +1, "11" -1, "00" none) and is drawn pulled
    -- back toward its old cell by s_mgf px, which counts down from one cell
    -- at the camera's rate.  The pull-back is one of three shared offsets per
    -- axis, so the render derives world+f and world-f once for all entities
    -- (in-cell offset +/- f: the carry/borrow moves the column, p_cell).
    signal s_mgf : unsigned(7 downto 0) := (others => '0');
    signal s_mg_start : std_logic := '0';
    -- the player's last step direction: the camera trails the target by
    -- s_pd * s_mgf, and the player (an entity like the monsters) is drawn
    -- pulled back by the same amount, i.e. at the screen centre
    signal s_pdx, s_pdy : std_logic_vector(1 downto 0) := "00";
    -- zoom / map / torch (K4 / S8 / K5), latched once per field
    signal s_zoom_sel  : unsigned(1 downto 0) := "10";       -- K4 decoded
    signal s_torch_sel : unsigned(2 downto 0) := "011";      -- K5 decoded
    signal s_sw_map    : std_logic := '0';
    signal s_cl        : unsigned(1 downto 0) := "10";       -- cell = 16 << s_cl px
    signal s_map_on    : std_logic := '0';
    signal s_r2, s_ri2 : integer range 0 to 81 := 0;         -- torch R^2, (R-1)^2
    signal s_torch_full : std_logic := '0';
    signal s_rv_r      : integer range 0 to 9 := 5;          -- reveal radius
    signal s_fcnt      : unsigned(7 downto 0) := (others => '0');  -- field counter
    signal s_edge_r, s_edge_b : unsigned(11 downto 0) := (others => '0');
    signal s_yedge : std_logic := '0';            -- border band y flag (p_acc)
    -- HUD bands: per-line flags + the glyph row (y - band start), taken from
    -- the next-line flags at each line start (full width: no x limit)
    signal s_htop, s_hbot : std_logic := '0';
    signal s_hcnt : unsigned(4 downto 0) := (others => '0');
    -- title screen (class select): the class sprite at the Close zoom on a
    -- black screen, under a 2x "OUBLIETTE" logo band (HUD RAM top line, cols
    -- 48-56; centred on the measured screen)
    signal s_title : std_logic := '0';
    signal s_logo_y0, s_logo_x0 : unsigned(11 downto 0) := (others => '0');
    signal s_lgy : std_logic := '0';
    signal s_lcy : unsigned(5 downto 0) := (others => '0');
    signal s0_logo, s1_logo, s2_logo, s3_logo, s4_logo : std_logic := '0';
    -- hit feedback + facing (see p_game / p_mmob)
    signal s_hitflash, s_hurtflash : unsigned(2 downto 0) := (others => '0');
    -- HP bar ends and colour (from hp / maxhp, refreshed continuously)
    signal s_bar_hpx, s_bar_mxx : unsigned(11 downto 0) := (others => '0');
    signal s_bar_col : integer range 0 to 2 := 0;
    -- K6 theme and the per-depth tint of the tiles (Colour theme)
    signal s_theme_sel, s_theme : integer range 0 to 4 := 0;
    signal s_theme_m1 : unsigned(1 downto 0) := "00";            -- theme - 1
    signal s_tint_u, s_tint_v : signed(6 downto 0) := (others => '0');
    signal s_flash_on, s_bord_hurt, s_bord_low : std_logic := '0';
    signal s_pmir, s_pbob : std_logic := '0';
    signal s_sw_step : std_logic := '0';

    -- facing reticle: the cell the player would step into (live from heading)
    signal s_tgt_x : unsigned(5 downto 0) := (others => '0');
    signal s_tgt_y : unsigned(5 downto 0) := (others => '0');
    signal s_show_tgt : std_logic := '0';

    -- Player stats (HUD-visible; combat mutates these from M3c).
    signal s_hp    : unsigned(7 downto 0) := to_unsigned(20, 8);
    signal s_maxhp : unsigned(7 downto 0) := to_unsigned(20, 8);
    signal s_gold  : unsigned(11 downto 0) := (others => '0');
    signal s_level : unsigned(4 downto 0) := to_unsigned(1, 5);
    signal s_depth : unsigned(2 downto 0) := to_unsigned(1, 3);

    -- HUD geometry latched per frame (bottom-aligned to measured resolution).
    signal s_hud_y0 : unsigned(11 downto 0) := to_unsigned(1080 - C_HUD_H - C_HUD_MARG, 12);

    --------------------------------------------------------------------------
    -- Master game state + combat
    --------------------------------------------------------------------------
    type t_game is (G_CREATE, G_GEN, G_EXPLORE, G_COMBAT, G_GAMEOVER, G_VICTORY);
    signal s_game : t_game := G_CREATE;

    -- Procedural dungeon generator (runs into the tilemap BRAM behind a splash).
    constant C_NROOM : integer := 8;
    type t_gen is (GN_CLEAR, GN_RPICK, GN_RLIM, GN_RCLAMP, GN_RSET, GN_RCARVE,
                   GN_CORR, GN_CORR2, GN_CRD, GN_CWT, GN_CLT, GN_CWR,
                   GN_PLACE, GN_MON, GN_MONH, GN_MONW, GN_DONE);
    signal s_gst   : t_gen := GN_DONE;
    signal s_gensp : std_logic := '0';                       -- splash active
    signal s_glfsr : std_logic_vector(23 downto 0) := x"ACE159";
    signal s_gaddr : unsigned(11 downto 0) := (others => '0');
    -- room centres: an EBR (address = room) written by GN_RSET through a
    -- registered write port.  Its read address runs one state AHEAD of the
    -- state that uses the data (s_rc_ra below), so the generator keeps its
    -- exact clock-by-clock schedule (the level LFSR steps every clock).
    -- Corridor j's first centre (room j-1) is the previous corridor's second
    -- one (s_bx/s_by), or room 0 (s_r0x/s_r0y) for the first corridor and
    -- the spawn.
    type t_rcram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal s_rcram : t_rcram := (others => (others => '0'));
    signal s_rc_we : std_logic := '0';
    signal s_rc_wa : unsigned(2 downto 0) := (others => '0');
    signal s_rc_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rc_ra : unsigned(2 downto 0);
    signal s_rc_rd : std_logic_vector(15 downto 0);
    signal s_rc_q  : std_logic_vector(15 downto 0);
    signal s_r0x, s_r0y : unsigned(5 downto 0) := (others => '0');
    signal s_gi  : integer range 0 to C_NROOM := 0;          -- room/work index
    signal s_rx0, s_ry0, s_rx1, s_ry1 : unsigned(5 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(5 downto 0) := (others => '0');  -- carve cursor
    signal s_ca, s_cb : unsigned(5 downto 0) := (others => '0');  -- corridor span
    signal s_rfloor : std_logic_vector(3 downto 0) := (others => '0');  -- room floor type
    -- room draw, one op per state (GN_RPICK -> RLIM -> RCLAMP -> RSET): the
    -- whole chain in one state was the routed critical path
    signal s_rw, s_rh : unsigned(3 downto 0) := (others => '0');        -- size 5..12
    signal s_rxr, s_ryr : unsigned(6 downto 0) := (others => '0');      -- raw corner 2..65
    signal s_xlim, s_ylim : unsigned(5 downto 0) := (others => '0');    -- 62 - size
    signal s_rwm1, s_rhm1 : unsigned(5 downto 0) := (others => '0');    -- size - 1
    -- room furniture: feature per room (0-3 plain, 4 pillar grid, 5 pool,
    -- 6 pool (lava from depth 4), 7 rubble) and the pool's inner rectangle
    signal s_rfeat : std_logic_vector(2 downto 0) := (others => '0');
    signal s_rwm3, s_rhm3 : unsigned(5 downto 0) := (others => '0');    -- size - 3
    signal s_rix0, s_rix1, s_riy0, s_riy1 : unsigned(5 downto 0) := (others => '0');
    -- corridor carving reads each cell first: the first wall cell of a
    -- crossing becomes a door
    signal s_cvert, s_cw2, s_pw2 : std_logic := '0';
    -- corridor: s_gp is unused since the room-centre RAM (the first centre is
    -- s_r0x/y or the previous s_bx/y); both centres latched, then ordered
    signal s_gp : integer range 0 to C_NROOM - 1 := 0;
    signal s_ax, s_bx, s_ay, s_by : unsigned(5 downto 0) := (others => '0');
    signal s_vlo, s_vhi : unsigned(5 downto 0) := (others => '0');
    -- monster spawn: s_gr walks rooms 3..NROOM-1 alongside slot s_gi (no
    -- s_gi+3 adder in front of the room-table mux); GN_MON latches the
    -- candidate into scalars, GN_MONW writes the slot (one op per state).
    signal s_gr : integer range 0 to C_NROOM - 1 := 0;
    signal s_gx_guard : unsigned(5 downto 0) := (others => '0');  -- stairs x - 1
    signal s_nm_x, s_nm_y : unsigned(5 downto 0) := (others => '0');
    signal s_nm_t  : unsigned(2 downto 0) := (others => '0');
    signal s_nm_hp : unsigned(7 downto 0) := (others => '0');
    signal s_nm_en : std_logic := '0';                             -- slot spawns?
    signal s_nm_oh : std_logic_vector(0 to C_MONS - 1) := (others => '0');  -- slot one-hot

    -- peaceful mode (S10): monsters don't chase or attack
    signal s_sw_peace : std_logic := '0';

    -- transient top-line message (a layout id, M_*), shown while the timer runs
    constant C_MSG_FRAMES : integer := 120;
    signal s_msg_lay : integer range 0 to 31 := 0;
    signal s_msgtmr  : unsigned(6 downto 0) := (others => '0');   -- fields remaining

    -- S9 Potion: drink one (a turn in EXPLORE, the round's action in COMBAT)
    signal s_sw_use, s_use_prev, s_use_pending : std_logic := '0';

    -- player resources beyond HP/ATK/DEF
    signal s_mp, s_maxmp : unsigned(4 downto 0) := (others => '0');
    signal s_cost  : unsigned(4 downto 0) := (others => '0');     -- class cast cost
    signal s_pots  : unsigned(3 downto 0) := (others => '0');     -- potions 0..9
    signal s_tcnt  : unsigned(1 downto 0) := (others => '0');     -- MP +1 every 4th turn
    signal s_grace : unsigned(1 downto 0) := (others => '0');     -- monster turns skipped after a flee
    -- per-depth monster stat bonuses, computed while the level clears
    signal s_dbon_hp  : integer range 0 to 15 := 0;
    signal s_dbon_atk : integer range 0 to 7 := 0;
    signal s_dbon_def : integer range 0 to 3 := 0;
    signal s_dbon_xp  : integer range 0 to 7 := 0;

    signal s_ctarget   : integer range 0 to C_MONS - 1 := 0;  -- monster in combat
    signal s_cb_action : integer range 0 to 4 := 0;           -- 0 Atk 1 Def 2 Cast 3 Item 4 Flee
    signal s_clfsr     : std_logic_vector(15 downto 0) := x"1234";

    --------------------------------------------------------------------------
    -- Monster table (flip-flops so the render layer can test all 16 in parallel).
    -- M3a: statically populated; AI + combat arrive in M3c.
    --------------------------------------------------------------------------
    type t_m6 is array (0 to C_MONS - 1) of unsigned(5 downto 0);
    type t_m3 is array (0 to C_MONS - 1) of unsigned(2 downto 0);
    type t_m8 is array (0 to C_MONS - 1) of unsigned(7 downto 0);

    signal s_mx : t_m6 := (0 => to_unsigned(18, 6), 1 => to_unsigned(14, 6),
                           2 => to_unsigned(20, 6), 3 => to_unsigned(16, 6),
                           others => (others => '0'));
    signal s_my : t_m6 := (0 => to_unsigned(16, 6), 1 => to_unsigned(18, 6),
                           2 => to_unsigned(20, 6), 3 => to_unsigned(12, 6),
                           others => (others => '0'));
    signal s_mtype : t_m3 := (0 => "000", 1 => "001", 2 => "010", 3 => "100",
                              others => "000");
    -- monster HP: only the turn / combat FSMs use it, so it lives in an EBR
    -- (slot = address) instead of 48 FFs with index muxes.  One registered
    -- read port whose address is the turn FSM's slot in TS_MREQ (data used
    -- in TS_MCALC) and otherwise the highest slot standing on (s_tx, s_ty)
    -- (the same compare p_mmatch registers, so in TS_PSCAN the data is that
    -- of the highest s_mmatch bit).  Writes (GN_MONW, CS_PHIT2) go through a
    -- registered write port; no read is used within a clock of a write.
    type t_mhpram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal s_mhpram : t_mhpram := (others => x"0A");
    signal s_hpr_addr : unsigned(2 downto 0);
    signal s_hpr_rd   : std_logic_vector(7 downto 0);
    signal s_hpr_q    : std_logic_vector(7 downto 0);
    signal s_hpw_we   : std_logic := '0';
    signal s_hpw_addr : unsigned(2 downto 0) := (others => '0');
    signal s_hpw_data : std_logic_vector(7 downto 0) := (others => '0');
    signal s_mactive : std_logic_vector(0 to C_MONS - 1) :=
        (0 => '1', 1 => '1', 2 => '1', 3 => '1', others => '0');

    --------------------------------------------------------------------------
    -- Dungeon tilemap BRAM.  4 bits/cell (tile id in bits 2:0; bit 3 reserved
    -- for a fog/seen flag) - half the block-RAM footprint of an 8-bit cell,
    -- which relieves routing congestion on the HX4K.
    --------------------------------------------------------------------------
    type t_map is array (0 to C_MAP * C_MAP - 1) of std_logic_vector(3 downto 0);

    function build_map return t_map is
        variable m : t_map := (others => (others => '0'));
        variable id : integer;
    begin
        for r in 0 to C_MAP - 1 loop
            for c in 0 to C_MAP - 1 loop
                if r = 0 or r = C_MAP - 1 or c = 0 or c = C_MAP - 1 then
                    id := C_T_WALL;
                elsif (r = 32) and (c > 8) and (c < 55) and (c /= 32) then
                    id := C_T_WALL;
                elsif (c = 32) and (r > 8) and (r < 55)
                      and (r /= 16) and (r /= 32) and (r /= 48) then
                    id := C_T_WALL;
                elsif (r >= 4 and r <= 9) and (c >= 4 and c <= 11) then
                    id := C_T_WATER;
                elsif (r = 20 and c = 44) then
                    id := C_T_STAIRS;
                elsif (r >= 48 and r <= 52) and (c >= 10 and c <= 16) then
                    id := C_T_RUBBLE;
                else
                    id := C_T_FLOOR;
                end if;
                m(r * C_MAP + c) := std_logic_vector(to_unsigned(id, 4));
            end loop;
        end loop;
        return m;
    end function;

    signal s_map : t_map := build_map;
    attribute ram_style : string;
    attribute ram_style of s_map : signal is "block";
    -- no_rw_check: no read-during-write emulation (it costs ~74 FF + 43 LUT
    -- per RAM).  Collision-free in use: the render reads during active video,
    -- game reads/writes happen in vblank, and the one same-address case (a
    -- pickup clearing the cell just read) has no consumer for that read.
    -- Honoured: the read is async + a plain FF (no init), merged into the EBR.
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_map : signal is true;
    attribute ram_style of s_mhpram : signal is "block";
    attribute no_rw_check of s_mhpram : signal is true;
    -- room-centre RAM: reads only matter in the states after all 8 rooms are
    -- written (a read colliding with a room write is never used)
    attribute ram_style of s_rcram : signal is "block";
    attribute no_rw_check of s_rcram : signal is true;

    signal s_g_raddr : unsigned(2 * C_MAP_LOG2 - 1 downto 0) := (others => '0');
    -- shared registered map read output (render during active video, game in vblank)
    signal s_map_ra  : unsigned(2 * C_MAP_LOG2 - 1 downto 0);
    signal s_map_rd  : std_logic_vector(3 downto 0);
    signal s_map_q   : std_logic_vector(3 downto 0);
    -- generator write port (driven by p_game, applied in p_gmap)
    signal s_mw_we   : std_logic := '0';
    signal s_mw_addr : unsigned(2 * C_MAP_LOG2 - 1 downto 0) := (others => '0');
    signal s_mw_data : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Tile ROM + Sprite ROM (addr = id*16 + row).
    --------------------------------------------------------------------------
    constant C_TROM_DEPTH : integer := C_NTILES * 16;
    type t_trom is array (0 to C_TROM_DEPTH - 1) of std_logic_vector(15 downto 0);
    function build_trom return t_trom is
        variable r : t_trom;
    begin
        for t in 0 to C_NTILES - 1 loop
            for y in 0 to 15 loop r(t * 16 + y) := C_TILES(t, y); end loop;
        end loop;
        return r;
    end function;
    signal s_trom : t_trom := build_trom;
    attribute ram_style of s_trom : signal is "block";

    constant C_SPROM_DEPTH : integer := C_NSPR * 16;
    type t_sprom is array (0 to C_SPROM_DEPTH - 1) of std_logic_vector(15 downto 0);
    function build_sprom return t_sprom is
        variable r : t_sprom;
    begin
        for s in 0 to C_NSPR - 1 loop
            for y in 0 to 15 loop r(s * 16 + y) := C_SPRITES(s, y); end loop;
        end loop;
        return r;
    end function;
    signal s_sprom : t_sprom := build_sprom;
    attribute ram_style of s_sprom : signal is "block";

    --------------------------------------------------------------------------
    -- Font ROM (8x8, addr = glyph*8 + row) + BCD digit ROM (value -> 3 digits).
    --------------------------------------------------------------------------
    constant C_FROM_DEPTH : integer := C_FONT_CHAR_COUNT * 8;
    type t_from is array (0 to C_FROM_DEPTH - 1) of std_logic_vector(7 downto 0);
    function build_from return t_from is
        variable r : t_from;
    begin
        for g in 0 to C_FONT_CHAR_COUNT - 1 loop
            for y in 0 to 7 loop r(g * 8 + y) := C_FONT_ROM(g, y); end loop;
        end loop;
        return r;
    end function;
    signal s_from : t_from := build_from;
    attribute ram_style of s_from : signal is "block";

    signal s_bcd : t_bcd := C_BCD;
    attribute ram_style of s_bcd : signal is "block";
    signal s_bcd_addr : unsigned(9 downto 0) := (others => '0');
    signal s_bcd_q    : std_logic_vector(11 downto 0);   -- EBR output register: no init

    --------------------------------------------------------------------------
    -- HUD character RAM (block RAM, 128 x 7 - address bit 6 selects the top
    -- context line, bits 5:0 the column).  Replacing the old flip-flop char
    -- arrays kills two 28-way 7-bit read muxes (a recurring critical path)
    -- and their write decoders.  The update FSM rewrites every cell - labels
    -- included - once per frame during vblank, one cell per cycle.
    --------------------------------------------------------------------------
    type t_hudram is array (0 to 127) of std_logic_vector(6 downto 0);
    signal s_hudram : t_hudram := (others => (others => '0'));
    attribute ram_style of s_hudram : signal is "block";
    -- written in vblank only, read in active video only (see s_map)
    attribute no_rw_check of s_hudram : signal is true;
    signal s_hr_we    : std_logic := '0';
    signal s_hr_waddr : unsigned(6 downto 0) := (others => '0');
    signal s_hr_wdata : std_logic_vector(6 downto 0) := (others => '0');

    -- HUD sequencer (runs once per field, in vblank, after the turn settles):
    --   1. numbers -> decimal digits (BCD ROM) -> field RAM slots
    --   2. name glyphs (class / monster / action / result) copied from the
    --      layout ROM's name rows -> field RAM slots
    --   3. both lines: layout ROM glyph, or field RAM slot for 64+k codes,
    --      streamed one column per clock into the HUD char RAM.
    type t_hu is (HU_IDLE, HU_DLY, HU_FREQ, HU_FWAIT, HU_FLAT, HU_FDIG, HU_NAME,
                  HU_NDRAIN, HU_LINE, HU_DRAIN);
    signal s_hu_state : t_hu := HU_IDLE;
    signal s_hu_dly   : unsigned(2 downto 0) := (others => '0');   -- drain waits
    signal s_hu_f     : integer range 0 to C_NFLD - 1 := 0;     -- numeric field
    signal s_hu_w     : integer range 0 to 3 := 0;              -- its width
    signal s_hu_j     : integer range 0 to 3 := 0;              -- digit index
    signal s_hu_k     : unsigned(5 downto 0) := (others => '0'); -- field RAM write slot
    signal s_hu_n     : integer range 0 to C_NNAME - 1 := 0;    -- name glyph index
    signal s_hu_blank : std_logic := '0';                       -- write spaces, not digits
    signal s_hu_bcd   : std_logic_vector(11 downto 0) := (others => '0');  -- aligned digits
    signal s_hu_col   : unsigned(5 downto 0) := (others => '0');
    signal s_hu_line  : std_logic := '0';                       -- 0 top, 1 bottom
    signal s_hu_ltop  : integer range 0 to 31 := 0;             -- top layout id
    -- field RAM (64 x 7): digit / name glyphs; written in phases 1-2, read in 3
    type t_fram is array (0 to 63) of std_logic_vector(6 downto 0);
    signal s_fram : t_fram := (others => (others => '0'));
    attribute ram_style of s_fram : signal is "block";
    attribute no_rw_check of s_fram : signal is true;
    signal s_fw_we   : std_logic := '0';
    signal s_fw_addr : unsigned(5 downto 0) := (others => '0');
    signal s_fw_data : std_logic_vector(6 downto 0) := (others => '0');
    signal s_fr_rd   : std_logic_vector(6 downto 0);
    signal s_fr_q    : std_logic_vector(6 downto 0);
    -- layout ROM (32 layouts x 64 columns)
    signal s_lrom : t_hudlay := C_HUDLAY;
    attribute ram_style of s_lrom : signal is "block";
    signal s_lr_addr : unsigned(10 downto 0) := (others => '0');
    signal s_lr_q    : std_logic_vector(6 downto 0);   -- EBR output register: no init
    -- line-stream pipeline: issue -> ROM -> fabric reg -> field RAM -> write
    signal s_p0_v, s_p1_v, s_p2_v, s_p3_v : std_logic := '0';
    -- name mode: the pipeline copies a ROM name glyph into the field RAM
    signal s_p0_n, s_p1_n, s_p2_n, s_p3_n : std_logic := '0';
    -- (the stream's write addresses are counters: the line stream writes HUD
    -- RAM 64..127 then 0..63 one per clock, the name copy writes field RAM
    -- slots C_NAME_BASE.. one per clock, so no column carry chain is needed)
    signal s_p2_g, s_p3_g : std_logic_vector(6 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Turn FSM (player move + greedy monster chase) + combat sub-FSM.
    --------------------------------------------------------------------------
    -- Turn FSM is split into short single-purpose states so no one cycle has a
    -- long combinational path (each runs in vblank with whole-frame slack).
    type t_turn is (TS_IDLE, TS_PMON, TS_PSW, TS_PSCAN, TS_PWAIT, TS_PMOVE,
                    TS_USE, TS_USE2, TS_MSTART,
                    TS_MREQ, TS_MCALC, TS_MAGG, TS_MSTEP, TS_MBUMP, TS_MWAIT, TS_MWALK,
                    TS_MAPPLY);
    signal s_ts : t_turn := TS_IDLE;
    signal s_mi : integer range 0 to C_MONS - 1 := 0;
    signal s_tx : unsigned(5 downto 0) := (others => '0');
    signal s_ty : unsigned(5 downto 0) := (others => '0');
    signal s_mocc : std_logic := '0';
    -- map-read results latched into fabric flags one state before they are
    -- used (EBR clk-to-q + routing straight into the move decisions was the
    -- routed critical path)
    signal s_t_walk, s_t_stairs, s_t_weapon, s_t_chest, s_mwalk : std_logic := '0';
    signal s_hpfull : std_logic := '0';   -- hp >= maxhp, registered (TS_USE)
    -- selected monster's position latched out of the 16:1 table mux (so the
    -- greedy-chase arithmetic doesn't share a cycle with the index mux)
    signal s_sel_mx : unsigned(5 downto 0) := (others => '0');
    signal s_sel_my : unsigned(5 downto 0) := (others => '0');
    signal s_sel_mt : unsigned(2 downto 0) := (others => '0');
    signal s_sel_mhp : unsigned(7 downto 0) := (others => '0');
    -- in-combat monster stats, latched into scalars at encounter start so combat
    -- math and the HUD never re-index the monster table by s_ctarget.
    signal s_cmt  : unsigned(2 downto 0) := (others => '0');
    signal s_cmhp : unsigned(7 downto 0) := (others => '0');
    -- combat stats pre-resolved to scalars (no stat-table lookups on the combat
    -- arithmetic path): player atk/def latched at CREATE, monster at encounter.
    signal s_patk, s_pdef : integer range 0 to 31 := 0;
    signal s_cmatk, s_cmdef : integer range 0 to 31 := 0;
    signal s_cmxp : integer range 0 to 63 := 0;
    -- registered chase deltas (abs + sign/nonzero) so the axis-select + step is
    -- a separate short cycle from the subtract/abs.
    signal s_adx, s_ady : integer range 0 to 64 := 0;
    signal s_ddxp, s_ddxn, s_ddyp, s_ddyn : std_logic := '0';
    -- monster AI: chase within C_AGGRO cells (else wander); a blocked chase
    -- step retries once along the other axis
    signal s_aggro, s_wmove : std_logic := '0';
    signal s_wdir : std_logic_vector(1 downto 0) := (others => '0');
    signal s_alt_tx, s_alt_ty : unsigned(5 downto 0) := (others => '0');
    signal s_alt_ok, s_alt_used : std_logic := '0';
    -- "slot j moves this turn": active and not the stationary dragon
    signal s_mmob : std_logic_vector(0 to C_MONS - 1) := (others => '0');
    -- monster faces the player (sprite mirrored when the player is to its left)
    signal s_mface : std_logic_vector(0 to C_MONS - 1) := (others => '0');
    -- monster glide: last step direction per slot, and the step being tried
    type t_m2 is array (0 to C_MONS - 1) of std_logic_vector(1 downto 0);
    signal s_mdx, s_mdy : t_m2 := (others => "00");
    signal s_cdx, s_cdy, s_adxd, s_adyd : std_logic_vector(1 downto 0) := "00";
    -- continuously registered "monster j stands on (s_tx,s_ty)" match vector -
    -- the FSM scan states just OR these bits instead of doing 6 parallel 12-bit
    -- coordinate compares deep inside the state-decode tree (was ~13 levels).
    signal s_mmatch : std_logic_vector(0 to C_MONS - 1) := (others => '0');
    -- per-slot spawn-enable mask, precomputed at generation start so GN_MON
    -- does no depth arithmetic.
    signal s_mon_en : std_logic_vector(0 to C_MONS - 1) := (others => '0');
    signal s_step_prev : std_logic := '0';
    signal s_new_prev  : std_logic := '0';
    signal s_turn_pending : std_logic := '0';
    signal s_sw_new : std_logic := '0';

    -- combat resolves one arithmetic op per cycle (turn-based: latency is free,
    -- and every cycle's combinational path stays short for full-clock timing).
    type t_cs is (CS_INIT, CS_INIT2, CS_INPUT, CS_PLAYER, CS_PSUB, CS_PCLAMP,
                  CS_PHIT, CS_PHIT2, CS_POUT, CS_LVL, CS_LVL2,
                  CS_MONSTER, CS_MSUB, CS_MCLAMP, CS_MHIT);
    signal s_cs : t_cs := CS_INPUT;
    signal s_cb_act_l : integer range 0 to 4 := 0;
    -- what the player's action does this round
    type t_pk is (PK_NONE, PK_DMG, PK_FIRE, PK_HEAL, PK_DEF, PK_FLEE);
    signal s_pk : t_pk := PK_NONE;
    signal s_raw  : integer range 0 to 63 := 0;        -- pre-DEF damage
    signal s_diff : integer range -32 to 63 := 0;      -- after DEF, before clamp
    signal s_hsum : integer range 0 to 255 := 0;       -- hp + heal, before cap
    signal s_mhd  : integer range -64 to 255 := 0;     -- monster hp after the hit
    signal s_xpd  : integer range -256 to 255 := 0;    -- xp - level threshold
    signal s_stun, s_fled : std_logic := '0';
    signal s_dmg, s_mdmg : integer range 0 to 63 := 0;
    signal s_mon_dead : std_logic := '0';
    -- experience: kills award XP; level-up at level*8 XP (shift, no multiply)
    signal s_xp : unsigned(7 downto 0) := (others => '0');
    -- HUD feedback: last round's result word, amount dealt/healed, damage
    -- taken (each with a valid flag: invalid shows blank), last XP gained
    signal s_fb_res : integer range 0 to 15 := 0;
    signal s_fb_dlt, s_fb_tkn : integer range 0 to 63 := 0;
    signal s_fb_dv, s_fb_tv : std_logic := '0';
    signal s_xpg : integer range 0 to 63 := 0;


    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    -- S0
    signal s0_wx, s0_wy : signed(13 downto 0) := (others => '0');     -- world
    signal s0_render, s0_edge : std_logic := '0';
    -- S1
    signal s1_addr : unsigned(2 * C_MAP_LOG2 - 1 downto 0) := (others => '0');
    signal s1_gx, s1_gy : unsigned(3 downto 0) := (others => '0');
    signal s1_colp, s1_colm, s1_rowp, s1_rowm : unsigned(5 downto 0) := (others => '0');
    signal s1_gxp, s1_gxm, s1_gyp, s1_gym : unsigned(3 downto 0) := (others => '0');
    signal s1_inb, s1_intgt, s1_render, s1_edge : std_logic := '0';
    -- S2
    signal s2_ehit : std_logic := '0';
    signal s2_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s2_gx, s2_gy, s2_gxp, s2_gxm, s2_gyp, s2_gym : unsigned(3 downto 0) := (others => '0');
    signal s2_wdx, s2_wdy : std_logic_vector(1 downto 0) := "00";    -- winner's step
    signal s2_smir, s2_sbob, s2_isp, s2_etgt : std_logic := '0';
    signal s2_dx, s2_dy : signed(6 downto 0) := (others => '0');      -- cell - player
    signal s2_inb, s2_intgt, s2_render, s2_edge : std_logic := '0';
    -- S3
    signal s3_sprrow : std_logic_vector(15 downto 0);   -- EBR output register: no init
    signal s3_ehit : std_logic := '0';
    signal s3_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s3_tile : integer range 0 to C_NTILES - 1 := 0;
    signal s3_gx, s3_gy, s3_sgx : unsigned(3 downto 0) := (others => '0');
    signal s3_smir, s3_isp, s3_etgt, s3_seen, s3_far : std_logic := '0';
    signal s3_adx, s3_ady : unsigned(3 downto 0) := (others => '0');
    signal s3_inb, s3_intgt, s3_render, s3_edge : std_logic := '0';
    -- S4
    signal s4_row : std_logic_vector(15 downto 0);   -- EBR output register: no init
    signal s4_sprrow : std_logic_vector(15 downto 0) := (others => '0');
    signal s4_ehit : std_logic := '0';
    signal s4_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s4_tile : integer range 0 to C_NTILES - 1 := 0;
    signal s4_gx, s4_gy, s4_sgx : unsigned(3 downto 0) := (others => '0');
    signal s4_smir, s4_isp, s4_etgt, s4_seen, s4_core, s4_ring : std_logic := '0';
    signal s4_inb, s4_intgt, s4_render, s4_edge : std_logic := '0';
    signal s4_rim : std_logic := '0';   -- art pixel on the reticle's rim
    -- S5 (select: layer decided, palette index carried)
    signal s5_pa   : unsigned(6 downto 0) := to_unsigned(C_PA_BLANK, 7);  -- palette address
    signal s5_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal s5_av, s5_tint : std_logic := '0';
    -- S6 (palette ROM read: one EBR pair, registered output)
    signal s_pal : t_palrom := C_PALROM;
    attribute ram_style of s_pal : signal is "block";
    signal s6_pal  : std_logic_vector(29 downto 0);   -- EBR output register: no init
    signal s6_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal s6_av, s6_tint : std_logic := '0';
    -- S7 (fabric copy of the ROM word)
    signal s7_y : unsigned(9 downto 0) := C_BLK;
    signal s7_u, s7_v : unsigned(9 downto 0) := C_MID;
    signal s7_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal s7_av, s7_tint : std_logic := '0';
    -- S8 (K6 palette theme)
    signal s8_y : unsigned(9 downto 0) := C_BLK;
    signal s8_u, s8_v : unsigned(9 downto 0) := C_MID;
    signal s8_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal s8_av   : std_logic := '0';
    -- HP bar flags along the HUD pipeline
    signal s0_barf, s0_bare, s1_barf, s1_bare, s2_barf, s2_bare : std_logic := '0';
    signal s3_barf, s3_bare, s4_barf, s4_bare : std_logic := '0';

    -- Fog of war: one bit per map cell, set for every cell the torch has lit
    -- (the reveal sweep, once per field in vblank), cleared with the level.
    -- Read by the render pipeline at the same address/stage as the tilemap.
    type t_seen is array (0 to C_MAP * C_MAP - 1) of std_logic;
    signal s_seen : t_seen := (others => '0');
    attribute ram_style of s_seen : signal is "block";
    attribute no_rw_check of s_seen : signal is true;
    signal s_seen_rd : std_logic;
    signal s_seen_q : std_logic;
    signal s_sv_we, s_sv_data : std_logic := '0';
    signal s_sv_addr : unsigned(11 downto 0) := (others => '0');
    type t_rv is (RV_IDLE, RV_DLY, RV_RUN);
    signal s_rv_state : t_rv := RV_IDLE;
    signal s_rv_dx, s_rv_dy : integer range -9 to 9 := 0;
    signal s_rv_rr : integer range 0 to 9 := 0;
    signal s_rva_v : std_logic := '0';
    signal s_rva_tx, s_rva_ty : integer range -9 to 72 := 0;
    signal s_rva_adx, s_rva_ady : integer range 0 to 9 := 0;

    -- HUD pipeline (screen-space, parallel to the world pipeline; same depth).
    signal s0_hud_active : std_logic := '0';
    signal s0_hud_top : std_logic := '0';
    signal s0_hud_col : unsigned(5 downto 0) := (others => '0');
    signal s0_frow, s0_fcol : unsigned(2 downto 0) := (others => '0');
    signal s1_glyph_rd : std_logic_vector(6 downto 0);
    signal s1_glyph : unsigned(6 downto 0);
    signal s1_hud_active : std_logic := '0';
    signal s1_frow, s1_fcol : unsigned(2 downto 0) := (others => '0');
    signal s2_fontrow : std_logic_vector(7 downto 0);   -- EBR output register: no init
    signal s2_hud_active : std_logic := '0';
    signal s2_fcol : unsigned(2 downto 0) := (others => '0');
    signal s3_fontrow : std_logic_vector(7 downto 0) := (others => '0');
    signal s3_hud_active : std_logic := '0';
    signal s3_fcol : unsigned(2 downto 0) := (others => '0');
    signal s4_fontrow : std_logic_vector(7 downto 0) := (others => '0');
    signal s4_hud_active : std_logic := '0';
    signal s4_fcol : unsigned(2 downto 0) := (others => '0');

    -- source syncs only (field_n & vsync_n & hsync_n); avid is generated
    type t_syncp is array (0 to C_SYNCD) of std_logic_vector(2 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));
    -- The two pure delay lines run through EBR circular buffers instead of
    -- FF chains (an FF fed by an FF costs a whole logic cell):
    --  * the source syncs, data_in -> s8 (9 clocks): written every clock,
    --    read 8 entries behind (+1 for the EBR output register);
    --  * the finished y/u/v, s8 -> data_out (the old s_io + 2-stage IO pad,
    --    3 clocks): read 2 entries behind.
    -- Read and write addresses never meet, so no read-during-write logic.
    type t_sdl is array (0 to 15) of std_logic_vector(3 downto 0);
    signal s_sdl    : t_sdl := (others => (others => '0'));
    signal s_sdl_wa : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_sdl_ra : unsigned(3 downto 0) := to_unsigned(8, 4);     -- wa - 8
    signal s_sdl_q  : std_logic_vector(3 downto 0) := (others => '0');
    type t_odl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal s_odl    : t_odl := (others => (others => '0'));
    signal s_odl_wa : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_odl_ra : unsigned(3 downto 0) := to_unsigned(14, 4);    -- wa - 2
    signal s_odl_rd : std_logic_vector(31 downto 0);
    signal s_odl_q  : std_logic_vector(31 downto 0);
    attribute ram_style of s_sdl : signal is "block";
    attribute no_rw_check of s_sdl : signal is true;
    attribute ram_style of s_odl : signal is "block";
    attribute no_rw_check of s_odl : signal is true;

    signal s_io : t_video_stream_yuv444_30b;

    function walkable(id : std_logic_vector(3 downto 0)) return boolean is
        variable t : integer;
    begin
        t := to_integer(unsigned(id));
        return (t /= C_T_WALL) and (t /= C_T_WATER) and (t /= C_T_WALL2)
               and (t /= C_T_PILLAR) and (t /= C_T_LAVA);
    end function;

    -- 24-bit Fibonacci LFSR step (taps 24,23,22,17) for dungeon generation.
    function lf24(x : std_logic_vector(23 downto 0)) return std_logic_vector is
        variable fb : std_logic;
    begin
        fb := x(23) xor x(22) xor x(21) xor x(16);
        return x(22 downto 0) & fb;
    end function;

begin

    s_sw_step  <= registers_in(6)(0);
    s_sw_map   <= registers_in(6)(1);   -- S8: whole-dungeon map view
    s_sw_use   <= registers_in(6)(2);   -- S9: drink a potion (edge)
    s_sw_peace <= registers_in(6)(3);   -- S10: monsters passive when on
    s_sw_new   <= registers_in(6)(4);

    --------------------------------------------------------------------------
    -- Timing infrastructure (genlocked to the incoming sync).
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

    -- s_firstline spans vsync start .. the first generated-avid line, so it
    -- is high for the whole vertical blank.  The field tick fires once, on
    -- the first vsync edge after active video; every game action, the HUD
    -- rewrite and the avid width latch hang off it.
    p_firstline : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_gav_start = '1' then
                s_firstline <= '0';
            end if;

            s_ftick <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_ftick <= '1';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Camera.  The TARGET scroll (s_pscroll: player cell centred at zoom
    -- s_cl, or the whole 64x64 map centred in Map view) is refreshed every
    -- clock through vertical blank, so it always matches the turn that ran
    -- at the field tick.  The world scroll trails it by the player's last
    -- step times the shared glide distance s_mgf (one cell at the turn,
    -- counting down 1/8 cell per field); the player and monsters are drawn
    -- pulled back by the same amount, so the player stays at the screen
    -- centre and everything walks in step.  New levels, zoom and map
    -- changes have no step direction, so the camera simply jumps.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_mx, v_my : signed(13 downto 0);
        variable v_op       : signed(13 downto 0);
        variable v_ci       : std_logic;
        variable v_a15, v_b15, v_s15 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            if s_ftick = '1' then
                g_wm1  <= s_measured_h - 1;         -- generated-avid width
                s_fcnt <= s_fcnt + 1;
                -- per-field controls: zoom (Map view forces 16 px), torch
                s_map_on <= s_sw_map;
                if s_title = '1' then
                    s_cl <= "11";                   -- big class sprite
                    s_ev <= to_unsigned(16, 8);
                elsif s_sw_map = '1' then
                    s_cl <= "00";
                    s_ev <= to_unsigned(2, 8);
                else
                    s_cl <= s_zoom_sel;
                    case s_zoom_sel is
                        when "00"   => s_ev <= to_unsigned(2, 8);
                        when "01"   => s_ev <= to_unsigned(4, 8);
                        when "10"   => s_ev <= to_unsigned(8, 8);
                        when others => s_ev <= to_unsigned(16, 8);
                    end case;
                end if;
                -- palette theme; the tiles' tint by depth (stored U = Cr, V = Cb):
                -- crypt, ice, moss, amethyst, sea, ember, hellfire
                s_theme <= s_theme_sel;
                if s_theme_sel /= 0 then
                    s_theme_m1 <= to_unsigned(s_theme_sel - 1, 2);
                end if;
                case to_integer(s_depth) is
                    when 2      => s_tint_u <= to_signed(-10, 7); s_tint_v <= to_signed(28, 7);
                    when 3      => s_tint_u <= to_signed(-24, 7); s_tint_v <= to_signed(-12, 7);
                    when 4      => s_tint_u <= to_signed(18, 7);  s_tint_v <= to_signed(24, 7);
                    when 5      => s_tint_u <= to_signed(-22, 7); s_tint_v <= to_signed(14, 7);
                    when 6      => s_tint_u <= to_signed(30, 7);  s_tint_v <= to_signed(-18, 7);
                    when 7      => s_tint_u <= to_signed(40, 7);  s_tint_v <= to_signed(-8, 7);
                    when others => s_tint_u <= to_signed(0, 7);   s_tint_v <= to_signed(0, 7);
                end case;
                -- torch radius 2..8 (top zone: full light, no fog)
                s_torch_full <= '0';
                case s_torch_sel is
                    when "000"  => s_r2 <= 4;  s_ri2 <= 1;  s_rv_r <= 2;
                    when "001"  => s_r2 <= 9;  s_ri2 <= 4;  s_rv_r <= 3;
                    when "010"  => s_r2 <= 16; s_ri2 <= 9;  s_rv_r <= 4;
                    when "011"  => s_r2 <= 25; s_ri2 <= 16; s_rv_r <= 5;
                    when "100"  => s_r2 <= 36; s_ri2 <= 25; s_rv_r <= 6;
                    when "101"  => s_r2 <= 49; s_ri2 <= 36; s_rv_r <= 7;
                    when "110"  => s_r2 <= 64; s_ri2 <= 49; s_rv_r <= 8;
                    when others => s_r2 <= 64; s_ri2 <= 49; s_rv_r <= 8; s_torch_full <= '1';
                end case;
            end if;

            if s_firstline = '1' then
                -- target: scroll = px*cell + cell/2 - screen/2 (or map centre);
                -- px*cell + cell/2 = (2*px + 1) << (cl + 3): a bit placement
                v_mx := (others => '0');
                v_my := (others => '0');
                if s_map_on = '1' then
                    v_mx(9) := '1';                 -- 512
                    v_my(9) := '1';
                else
                    case s_cl is
                        when "00"   => v_mx(9 downto 3)  := signed(s_px & '1'); v_my(9 downto 3)  := signed(s_py & '1');
                        when "01"   => v_mx(10 downto 4) := signed(s_px & '1'); v_my(10 downto 4) := signed(s_py & '1');
                        when "10"   => v_mx(11 downto 5) := signed(s_px & '1'); v_my(11 downto 5) := signed(s_py & '1');
                        when others => v_mx(12 downto 6) := signed(s_px & '1'); v_my(12 downto 6) := signed(s_py & '1');
                    end case;
                end if;
                -- layout in virtual units (s_vw x s_vh: 1920x1080, SD 1440x1080)
                s_pscroll_x <= v_mx - signed(resize(s_vw(11 downto 1), 14));
                s_pscroll_y <= v_my - signed(resize(s_vh(11 downto 1), 14));
                s_hud_y0 <= s_vh - (C_HUD_H + C_HUD_MARG);
                s_logo_y0 <= ('0' & s_vh(11 downto 1)) - 220;
                s_logo_x0 <= ('0' & s_vw(11 downto 1)) - 288;
                s_edge_r <= s_vw - C_EDGE;
                s_edge_b <= s_vh - C_EDGE;
            end if;

            -- Glide distance s_mgf: zeroed when a turn is about to run on this
            -- tick (finishing any glide in progress), restarted at one cell by
            -- the turn's monster phase, then counted down 1/8 cell per field
            -- (512 clocks after the tick, in vblank).
            if s_ftick = '1' then
                s_ecnt <= (others => '0');
            elsif s_ecnt /= 1023 then
                s_ecnt <= s_ecnt + 1;
            end if;
            if s_ftick = '1' and (s_turn_pending = '1' or s_use_pending = '1') then
                s_mgf <= (others => '0');
            elsif s_mg_start = '1' then
                case s_cl is
                    when "00"   => s_mgf <= to_unsigned(16, 8);
                    when "01"   => s_mgf <= to_unsigned(32, 8);
                    when "10"   => s_mgf <= to_unsigned(64, 8);
                    when others => s_mgf <= to_unsigned(128, 8);
                end case;
            elsif s_ecnt = 511 then
                if s_mgf > s_ev then
                    s_mgf <= s_mgf - s_ev;
                else
                    s_mgf <= (others => '0');
                end if;
            end if;

            -- camera = target - (player step) * glide; stable through active
            -- video (target, glide and step only change in vblank)
            -- one adder per axis: target + (step "01": -mgf = not mgf + 1,
            -- "11": +mgf, else 0); the carry-in rides an extra low bit
            v_op := signed(resize(s_mgf, 14)); v_ci := '0';
            case s_pdx is
                when "01"   => v_op := not v_op; v_ci := '1';
                when "11"   => null;
                when others => v_op := (others => '0');
            end case;
            v_a15 := s_pscroll_x & '1';
            v_b15 := v_op & v_ci;
            v_s15 := v_a15 + v_b15;
            s_scroll_x <= v_s15(14 downto 1);
            v_op := signed(resize(s_mgf, 14)); v_ci := '0';
            case s_pdy is
                when "01"   => v_op := not v_op; v_ci := '1';
                when "11"   => null;
                when others => v_op := (others => '0');
            end case;
            v_a15 := s_pscroll_y & '1';
            v_b15 := v_op & v_ci;
            v_s15 := v_a15 + v_b15;
            s_scroll_y <= v_s15(14 downto 1);
            if (s_pdx /= "00" or s_pdy /= "00") and s_mgf /= 0 then
                s_moving <= '1';
            else
                s_moving <= '0';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control decode with hysteresis: a zone only changes once the value is
    -- well inside the new zone (the outer edges of the end zones are open),
    -- so pot noise at a boundary can't flicker the reticle / action / class
    -- or send a step the wrong way.
    --   P12 Heading: 8 zones of 128, 16-code margin (top zone = N).
    --   K3 Action:   5 labelled zones (205/410/614/819): C_Z5 table, a
    --                16-code hold unit straddling each boundary.
    --   K1 Class:    4 labelled zones of 256, 32-code margin.
    --   K4 Zoom:     4 labelled zones of 256, 32-code margin.
    --   K5 Torch:    8 zones of 128, 16-code margin.
    --   K6 Palette:  5 labelled zones, as K3.
    --   (Margins are bit tests on the low bits, not compares.)
    --   Labelled defaults: at program load the firmware writes the label
    --   INDEX (0..N-1), not a position, so a value below N is the index.
    --------------------------------------------------------------------------
    p_ctl : process(clk)
        variable v_p   : unsigned(9 downto 0);
        variable v_lo7 : unsigned(6 downto 0);
        variable v_lo8 : unsigned(7 downto 0);
        variable v_z   : integer range 0 to 5;
    begin
        if rising_edge(clk) then
            -- zone + 1: the top of the travel is N (up), and pushing up from
            -- the bottom turns clockwise NE, E, SE, S, SW, W, NW, N
            v_p   := unsigned(registers_in(7));
            v_lo7 := v_p(6 downto 0);
            if (v_lo7(6 downto 4) /= "000" or v_p(9 downto 7) = "000") and
               (v_lo7(6 downto 4) /= "111" or v_p(9 downto 7) = "111") then
                s_dir <= v_p(9 downto 7) + 1;
            end if;

            v_p := unsigned(registers_in(2));
            v_z := C_Z5(to_integer(v_p(9 downto 4)));
            if v_p(9 downto 3) = "0000000" and v_p(2 downto 0) <= "100" then
                s_cb_action <= to_integer(v_p(2 downto 0));
            elsif v_z /= 5 then
                s_cb_action <= v_z;
            end if;

            -- K4 Zoom: 4 labelled zones (Far / Wide / Normal / Close)
            v_p   := unsigned(registers_in(3));
            v_lo8 := v_p(7 downto 0);
            if v_p(9 downto 2) = "00000000" then
                s_zoom_sel <= v_p(1 downto 0);
            elsif (v_lo8(7 downto 5) /= "000" or v_p(9 downto 8) = "00") and
                  (v_lo8(7 downto 5) /= "111" or v_p(9 downto 8) = "11") then
                s_zoom_sel <= v_p(9 downto 8);
            end if;

            -- K6 Palette: 5 labelled zones (Colour / Game Boy / Amber / CGA / C64)
            v_p := unsigned(registers_in(5));
            v_z := C_Z5(to_integer(v_p(9 downto 4)));
            if v_p(9 downto 3) = "0000000" and v_p(2 downto 0) <= "100" then
                s_theme_sel <= to_integer(v_p(2 downto 0));
            elsif v_z /= 5 then
                s_theme_sel <= v_z;
            end if;

            -- K5 Torch: 8 zones of 128 (radius 2..8, top = full light)
            v_p   := unsigned(registers_in(4));
            v_lo7 := v_p(6 downto 0);
            if (v_lo7(6 downto 4) /= "000" or v_p(9 downto 7) = "000") and
               (v_lo7(6 downto 4) /= "111" or v_p(9 downto 7) = "111") then
                s_torch_sel <= v_p(9 downto 7);
            end if;

            v_p   := unsigned(registers_in(0));
            v_lo8 := v_p(7 downto 0);
            if v_p(9 downto 2) = "00000000" then
                s_cls_sel <= v_p(1 downto 0);
            elsif (v_lo8(7 downto 5) /= "000" or v_p(9 downto 8) = "00") and
                  (v_lo8(7 downto 5) /= "111" or v_p(9 downto 8) = "11") then
                s_cls_sel <= v_p(9 downto 8);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Movement FSM: Switch-7 edge -> step one cell in the heading direction if
    -- the target tile is walkable.
    --------------------------------------------------------------------------
    -- Single-port tilemap controller: 1 write (generator) + 1 read whose address
    -- is time-multiplexed - the render pipeline reads during active video, the
    -- game logic reads during vertical blank.  This keeps the 32 Kbit map in ONE
    -- block-RAM copy (no duplication), which is essential for routing/timing.
    -- The select is the render-valid flag at the same stage as s1_addr (the
    -- raw source avid ended 2 clocks early: the last 2 columns read the
    -- game address).
    s_map_ra <= s1_addr when s1_render = '1' else s_g_raddr;   -- render / game read
    s_map_rd <= s_map(to_integer(s_map_ra));
    p_mapport : process(clk)
    begin
        if rising_edge(clk) then
            if s_mw_we = '1' then
                s_map(to_integer(s_mw_addr)) <= s_mw_data;
            end if;
            s_map_q <= s_map_rd;
        end if;
    end process;

    -- Free-running combat RNG (16-bit Fibonacci LFSR).
    p_clfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            v_fb := s_clfsr(15) xor s_clfsr(13) xor s_clfsr(12) xor s_clfsr(10);
            s_clfsr <= s_clfsr(14 downto 0) & v_fb;
        end if;
    end process;

    -- Continuous monster-at-target matcher: registered every cycle, consumed
    -- by the turn FSM one state later (a wait state guarantees freshness).
    p_mmatch : process(clk)
    begin
        if rising_edge(clk) then
            for j in 0 to C_MONS - 1 loop
                if s_mactive(j) = '1' and s_mx(j) = s_tx and s_my(j) = s_ty then
                    s_mmatch(j) <= '1';
                else
                    s_mmatch(j) <= '0';
                end if;
            end loop;
        end if;
    end process;

    -- monster HP RAM read address: the turn FSM's slot in TS_MREQ, else the
    -- highest slot matching (s_tx, s_ty) this clock (= s_mmatch next clock)
    p_hpraddr : process(all)
        variable v_a : unsigned(2 downto 0);
    begin
        v_a := (others => '0');
        for j in 0 to C_MONS - 1 loop
            if s_mactive(j) = '1' and s_mx(j) = s_tx and s_my(j) = s_ty then
                v_a := to_unsigned(j, 3);
            end if;
        end loop;
        if s_ts = TS_MREQ then
            v_a := to_unsigned(s_mi, 3);
        end if;
        s_hpr_addr <= v_a;
    end process;

    -- room-centre RAM.  The read address is the room the NEXT state reads:
    --   RCARVE -> CORR (room 1)      CWR -> CORR (room s_gi+1) / PLACE (room 1)
    --   PLACE  -> PLACE (rooms 2, 7) / MON (room 3)
    --   MONW   -> MON (room s_gr+1, which stays 7 once reached)
    p_rcraddr : process(all)
    begin
        case s_gst is
            when GN_RCARVE => s_rc_ra <= to_unsigned(1, 3);
            when GN_CWR =>
                if s_gi = C_NROOM - 1 then s_rc_ra <= to_unsigned(1, 3);
                else s_rc_ra <= to_unsigned(s_gi + 1, 3); end if;
            when GN_PLACE =>
                if s_gi = 0 then    s_rc_ra <= to_unsigned(2, 3);
                elsif s_gi = 1 then s_rc_ra <= to_unsigned(C_NROOM - 1, 3);
                else                s_rc_ra <= to_unsigned(3, 3); end if;
            when others =>          -- GN_MONW
                if s_gr = C_NROOM - 1 then s_rc_ra <= to_unsigned(C_NROOM - 1, 3);
                else s_rc_ra <= to_unsigned(s_gr + 1, 3); end if;
        end case;
    end process;
    s_rc_rd <= s_rcram(to_integer(s_rc_ra));
    p_rcram : process(clk)
    begin
        if rising_edge(clk) then
            if s_rc_we = '1' then
                s_rcram(to_integer(s_rc_wa)) <= s_rc_wd;
            end if;
            s_rc_q <= s_rc_rd;
        end if;
    end process;

    s_hpr_rd <= s_mhpram(to_integer(s_hpr_addr));
    p_mhpram : process(clk)
    begin
        if rising_edge(clk) then
            if s_hpw_we = '1' then
                s_mhpram(to_integer(s_hpw_addr)) <= s_hpw_data;
            end if;
            s_hpr_q <= s_hpr_rd;
        end if;
    end process;

    p_mmob : process(clk)
    begin
        if rising_edge(clk) then
            for j in 0 to C_MONS - 1 loop
                if s_mactive(j) = '1' and s_mtype(j) /= to_unsigned(C_BOSS, 3) then
                    s_mmob(j) <= '1';
                else
                    s_mmob(j) <= '0';
                end if;
                -- sprites face right: mirror when the player is to the left
                if s_px < s_mx(j) then s_mface(j) <= '1'; else s_mface(j) <= '0'; end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Master game FSM: CREATE -> EXPLORE <-> COMBAT, EXPLORE/COMBAT -> GAMEOVER.
    -- A turn (player move + greedy monster chase) runs serially during vblank so
    -- the monster table never changes while the render layer is reading it.
    --------------------------------------------------------------------------
    p_game : process(clk)
        -- All ranges constrained so synthesis uses narrow (not 32-bit) arithmetic.
        variable v_edge, v_newedge, v_uedge, v_go, v_ugo : std_logic;
        variable v_tx, v_ty : integer range 0 to 63;
        variable mxi, myi, pxi, pyi : integer range 0 to 63;
        variable ddx, ddy : integer range -64 to 64;
        variable v_occ : std_logic;
        variable cls : integer range 0 to 3;
        variable mt  : integer range 0 to 6;
        variable v_act : integer range 0 to 4;
        variable v_xp  : integer range 0 to 511;
        variable v_d   : integer range -64 to 63;
        variable v_mp  : integer range 0 to 63;
        variable v_dep : integer range 0 to 7;
        variable newhp : integer range -64 to 255;
        variable var, var2 : integer range 0 to 3;
        variable v_dest : integer range 0 to 15;

        -- begin generating a depth-d dungeon level (seeded by K2 + depth)
        -- (the level seed, spawn mask and stat bonuses are derived from s_depth
        -- inside GN_CLEAR, keeping this hand-over - called from the move
        -- state - to a few registers)
        procedure start_gen(d : integer) is
        begin
            s_depth <= to_unsigned(d, 3);
            s_pdx   <= "00";                 -- a new level: the camera jumps
            s_pdy   <= "00";
            s_gaddr <= (others => '0');
            s_gi    <= 0;
            s_gst   <= GN_CLEAR;
            s_gensp <= '1';
            s_game  <= G_GEN;
        end procedure;

        -- show a top-line message (layout M_*) for C_MSG_FRAMES fields
        procedure say(l : integer) is
        begin
            s_msg_lay <= l;
            s_msgtmr  <= to_unsigned(C_MSG_FRAMES, 7);
        end procedure;
    begin
        if rising_edge(clk) then
            s_step_prev <= s_sw_step;
            s_new_prev  <= s_sw_new;
            s_use_prev  <= s_sw_use;
            v_edge    := '0';
            v_newedge := '0';
            v_uedge   := '0';
            if s_sw_step /= s_step_prev then v_edge := '1'; end if;
            if s_sw_new  /= s_new_prev  then v_newedge := '1'; end if;
            if s_sw_use  /= s_use_prev  then v_uedge := '1'; end if;
            s_mw_we <= '0';   -- pulse high only when a generator state writes
            s_hpw_we <= '0';
            s_rc_we  <= '0';
            s_mg_start <= '0';
            -- an S7 / S9 flip is held pending and acted on at the next field
            -- tick, so every action resolves inside vertical blank (no
            -- mid-frame sprite tears; the level generator finishes within
            -- vblank).  S7 wins when both are pending.
            v_go  := s_turn_pending and s_ftick;
            v_ugo := s_use_pending and s_ftick;

            -- facing reticle target cell (live from heading), shown in EXPLORE
            v_tx := to_integer(s_px); v_ty := to_integer(s_py);
            case to_integer(s_dir) is
                when 0 => v_ty := v_ty - 1;                       -- N
                when 1 => v_tx := v_tx + 1; v_ty := v_ty - 1;     -- NE
                when 2 => v_tx := v_tx + 1;                       -- E
                when 3 => v_tx := v_tx + 1; v_ty := v_ty + 1;     -- SE
                when 4 => v_ty := v_ty + 1;                       -- S
                when 5 => v_tx := v_tx - 1; v_ty := v_ty + 1;     -- SW
                when 6 => v_tx := v_tx - 1;                       -- W
                when others => v_tx := v_tx - 1; v_ty := v_ty - 1; -- NW
            end case;
            s_tgt_x <= to_unsigned(v_tx, 6);
            s_tgt_y <= to_unsigned(v_ty, 6);
            if s_game = G_EXPLORE then s_show_tgt <= '1'; else s_show_tgt <= '0'; end if;

            -- message countdown (once per field); say() below restarts it
            if s_ftick = '1' and s_msgtmr /= 0 then
                s_msgtmr <= s_msgtmr - 1;
            end if;

            -- facing: the player sprite (drawn facing right) mirrors for the
            -- westward headings and keeps its last side for N / S; it bobs
            -- while the camera glides (walking)
            if s_dir = 5 or s_dir = 6 or s_dir = 7 then s_pmir <= '1';
            elsif s_dir = 1 or s_dir = 2 or s_dir = 3 then s_pmir <= '0'; end if;
            s_pbob <= s_moving and s_fcnt(2);

            if s_hp >= s_maxhp then s_hpfull <= '1'; else s_hpfull <= '0'; end if;
            if s_game = G_CREATE then s_title <= '1'; else s_title <= '0'; end if;
            -- HP bar: ends at x0 + 3*hp (fill) and x0 + 3*maxhp (frame)
            s_bar_hpx <= to_unsigned(C_BAR_X0, 12) + resize(s_hp & '0', 12) + resize(s_hp, 12);
            s_bar_mxx <= to_unsigned(C_BAR_X0, 12) + resize(s_maxhp & '0', 12) + resize(s_maxhp, 12);
            if s_hp > ('0' & s_maxhp(7 downto 1)) then     s_bar_col <= 0;
            elsif s_hp > ("00" & s_maxhp(7 downto 2)) then s_bar_col <= 1;
            else                                            s_bar_col <= 2; end if;

            -- hit feedback: timers count fields down; the combat states set them
            if s_ftick = '1' then
                if s_hitflash /= 0 then s_hitflash <= s_hitflash - 1; end if;
                if s_hurtflash /= 0 then s_hurtflash <= s_hurtflash - 1; end if;
            end if;
            if s_hitflash /= 0 then s_flash_on <= '1'; else s_flash_on <= '0'; end if;
            if s_hurtflash /= 0 then s_bord_hurt <= '1'; else s_bord_hurt <= '0'; end if;
            -- low HP (<= 1/4 max): the border pulses, 16 fields on / 16 off
            if shift_left(resize(s_hp, 10), 2) <= resize(s_maxhp, 10) and s_fcnt(4) = '1' then
                s_bord_low <= '1';
            else
                s_bord_low <= '0';
            end if;

            -- global new-game
            if v_newedge = '1' then
                s_game <= G_CREATE;
                s_ts <= TS_IDLE;
                s_turn_pending <= '0';
                s_use_pending  <= '0';
            else
                case s_game is

                    ----------------------------------------------------------
                    -- Class select: the HUD previews the chosen class's stats.
                    when G_CREATE =>
                        cls := to_integer(s_cls_sel);
                        s_class <= s_cls_sel;
                        s_hp    <= to_unsigned(C_CLS_HP(cls), 8);
                        s_maxhp <= to_unsigned(C_CLS_HP(cls), 8);
                        s_patk  <= C_CLS_ATK(cls);
                        s_pdef  <= C_CLS_DEF(cls);
                        s_mp    <= to_unsigned(C_CLS_MP(cls), 5);
                        s_maxmp <= to_unsigned(C_CLS_MP(cls), 5);
                        s_cost  <= to_unsigned(C_CLS_COST(cls), 5);
                        s_gold  <= (others => '0');
                        s_level <= to_unsigned(1, 5);
                        s_xp    <= (others => '0');
                        s_pots  <= to_unsigned(1, 4);
                        s_depth <= to_unsigned(1, 3);
                        s_grace <= (others => '0');
                        s_msgtmr <= (others => '0');
                        s_use_pending <= '0';
                        if v_go = '1' then
                            s_turn_pending <= '0';
                            s_ts <= TS_IDLE;
                            start_gen(1);          -- generate dungeon level 1
                        end if;

                    ----------------------------------------------------------
                    -- Procedural dungeon generation (one BRAM write per cycle;
                    -- ~6.5k clocks, so it finishes inside vertical blank).
                    when G_GEN =>
                        s_glfsr <= lf24(s_glfsr);
                        case s_gst is
                            when GN_CLEAR =>
                                -- this level's monster stat bonuses (depth 1 -> 0)
                                v_dep := to_integer(s_depth) - 1;
                                s_dbon_hp  <= 2 * v_dep;
                                s_dbon_atk <= v_dep;
                                s_dbon_def <= v_dep / 2;
                                s_dbon_xp  <= v_dep;
                                -- spawn-enable mask: slot i gets a monster if
                                -- i < depth+1 (and its room exists); slot 5 is the
                                -- dragon, at depth 7 only (GN_MON does no depth math)
                                for i in 0 to C_MONS - 1 loop
                                    if (i < to_integer(s_depth) + 1 and i + 3 < C_NROOM)
                                       or (i = C_MONS - 1 and s_depth = 7) then
                                        s_mon_en(i) <= '1';
                                    else
                                        s_mon_en(i) <= '0';
                                    end if;
                                end loop;
                                s_mw_addr <= s_gaddr;
                                s_mw_data <= std_logic_vector(to_unsigned(C_T_WALL, 4));
                                s_mw_we   <= '1';
                                if s_gaddr = C_MAP * C_MAP - 1 then
                                    s_gaddr <= (others => '0');
                                    s_gi <= 0;
                                    -- seed the level's LFSR (K2 + depth) for the rooms
                                    s_glfsr <= x"A5" & "000" & std_logic_vector(s_depth)
                                               & registers_in(1);
                                    s_gst <= GN_RPICK;
                                else
                                    s_gaddr <= s_gaddr + 1;
                                end if;
                            when GN_RPICK =>
                                -- draw: size 5..12, raw corner 2..65
                                s_rw  <= to_unsigned(5 + to_integer(unsigned(s_glfsr(2 downto 0))), 4);
                                s_rh  <= to_unsigned(5 + to_integer(unsigned(s_glfsr(5 downto 3))), 4);
                                s_rxr <= to_unsigned(2 + to_integer(unsigned(s_glfsr(13 downto 8))), 7);
                                s_ryr <= to_unsigned(2 + to_integer(unsigned(s_glfsr(21 downto 16))), 7);
                                s_rfeat <= s_glfsr(7 downto 6) & s_glfsr(14);
                                -- pick this room's floor terrain from the LFSR
                                case to_integer(unsigned(s_glfsr(23 downto 21))) is
                                    when 2 => s_rfloor <= std_logic_vector(to_unsigned(C_T_GRASS, 4));
                                    when 3 => s_rfloor <= std_logic_vector(to_unsigned(C_T_SAND, 4));
                                    when 4 => s_rfloor <= std_logic_vector(to_unsigned(C_T_MOSS, 4));
                                    when 5 => s_rfloor <= std_logic_vector(to_unsigned(C_T_CRACKED, 4));
                                    when 6 => s_rfloor <= std_logic_vector(to_unsigned(C_T_BONES, 4));
                                    when others => s_rfloor <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                end case;
                                s_gst <= GN_RLIM;
                            when GN_RLIM =>
                                -- largest corner that keeps the room inside the border
                                s_xlim <= to_unsigned(C_MAP - 2 - to_integer(s_rw), 6);
                                s_ylim <= to_unsigned(C_MAP - 2 - to_integer(s_rh), 6);
                                s_rwm1 <= resize(s_rw, 6) - 1;
                                s_rhm1 <= resize(s_rh, 6) - 1;
                                s_rwm3 <= resize(s_rw, 6) - 3;
                                s_rhm3 <= resize(s_rh, 6) - 3;
                                s_gst <= GN_RCLAMP;
                            when GN_RCLAMP =>
                                -- corner = min(raw, limit)  (raw + size > 62 <=> raw > limit)
                                if s_rxr > resize(s_xlim, 7) then
                                    s_rx0 <= s_xlim; s_cx <= s_xlim;
                                else
                                    s_rx0 <= s_rxr(5 downto 0); s_cx <= s_rxr(5 downto 0);
                                end if;
                                if s_ryr > resize(s_ylim, 7) then
                                    s_ry0 <= s_ylim; s_cy <= s_ylim;
                                else
                                    s_ry0 <= s_ryr(5 downto 0); s_cy <= s_ryr(5 downto 0);
                                end if;
                                s_gst <= GN_RSET;
                            when GN_RSET =>
                                -- far corner and centre
                                s_rx1 <= s_rx0 + s_rwm1;
                                s_ry1 <= s_ry0 + s_rhm1;
                                s_rc_we <= '1';
                                s_rc_wa <= to_unsigned(s_gi, 3);
                                s_rc_wd <= "0000" & std_logic_vector(s_ry0 + resize(s_rh(3 downto 1), 6))
                                                  & std_logic_vector(s_rx0 + resize(s_rw(3 downto 1), 6));
                                if s_gi = 0 then
                                    s_r0x <= s_rx0 + resize(s_rw(3 downto 1), 6);
                                    s_r0y <= s_ry0 + resize(s_rh(3 downto 1), 6);
                                end if;
                                s_rix0 <= s_rx0 + 2;
                                s_riy0 <= s_ry0 + 2;
                                s_rix1 <= s_rx0 + s_rwm3;
                                s_riy1 <= s_ry0 + s_rhm3;
                                s_gst <= GN_RCARVE;
                            when GN_RCARVE =>
                                -- room interior = floor terrain; perimeter = outline rock
                                s_mw_addr <= s_cy & s_cx;
                                if s_cx = s_rx0 or s_cx = s_rx1 or s_cy = s_ry0 or s_cy = s_ry1 then
                                    s_mw_data <= std_logic_vector(to_unsigned(C_T_WALL2, 4));
                                else
                                    -- furniture (corridors, carved later, cut through
                                    -- it, so every room stays connected)
                                    s_mw_data <= s_rfloor;
                                    case s_rfeat is
                                        when "100" =>                -- pillar grid
                                            if (s_cx(0) xor s_rx0(0)) = '0' and
                                               (s_cy(0) xor s_ry0(0)) = '0' then
                                                s_mw_data <= std_logic_vector(to_unsigned(C_T_PILLAR, 4));
                                            end if;
                                        when "101" | "110" =>        -- pool
                                            if s_cx >= s_rix0 and s_cx <= s_rix1 and
                                               s_cy >= s_riy0 and s_cy <= s_riy1 then
                                                if s_rfeat(0) = '0' and s_depth >= 4 then
                                                    s_mw_data <= std_logic_vector(to_unsigned(C_T_LAVA, 4));
                                                else
                                                    s_mw_data <= std_logic_vector(to_unsigned(C_T_WATER, 4));
                                                end if;
                                            end if;
                                        when "111" =>                -- rubble scatter
                                            if s_glfsr(1 downto 0) = "00" then
                                                s_mw_data <= std_logic_vector(to_unsigned(C_T_RUBBLE, 4));
                                            end if;
                                        when others => null;
                                    end case;
                                end if;
                                s_mw_we   <= '1';
                                if s_cx = s_rx1 then
                                    s_cx <= s_rx0;
                                    if s_cy = s_ry1 then
                                        if s_gi = C_NROOM - 1 then s_gi <= 1; s_gp <= 0; s_gst <= GN_CORR;
                                        else s_gi <= s_gi + 1; s_gst <= GN_RPICK; end if;
                                    else
                                        s_cy <= s_cy + 1;
                                    end if;
                                else
                                    s_cx <= s_cx + 1;
                                end if;
                            when GN_CORR =>
                                -- latch both centres (previous room s_gp, this room s_gi)
                                -- room s_gp = s_gi - 1: room 0, or the previous
                                -- corridor's second room; room s_gi from the RAM
                                if s_gi = 1 then
                                    s_ax <= s_r0x; s_ay <= s_r0y;
                                else
                                    s_ax <= s_bx;  s_ay <= s_by;
                                end if;
                                s_bx <= unsigned(s_rc_q(5 downto 0));
                                s_by <= unsigned(s_rc_q(11 downto 6));
                                s_gst <= GN_CORR2;
                            when GN_CORR2 =>
                                -- horizontal run at the previous room's row, between
                                -- centres; the vertical run's ends are ordered here too
                                if s_ax <= s_bx then
                                    s_ca <= s_ax; s_cb <= s_bx; s_cx <= s_ax;
                                else
                                    s_ca <= s_bx; s_cb <= s_ax; s_cx <= s_bx;
                                end if;
                                s_cy <= s_ay;
                                if s_ay <= s_by then
                                    s_vlo <= s_ay; s_vhi <= s_by;
                                else
                                    s_vlo <= s_by; s_vhi <= s_ay;
                                end if;
                                s_cvert <= '0';
                                s_pw2   <= '0';
                                s_gst <= GN_CRD;
                            -- carve one corridor cell in 4 steps: read it, wait,
                            -- latch "is room wall", write FLOOR (or DOOR for the
                            -- first wall cell of a crossing) and advance
                            when GN_CRD =>
                                s_g_raddr <= s_cy & s_cx;
                                s_gst <= GN_CWT;
                            when GN_CWT =>
                                s_gst <= GN_CLT;
                            when GN_CLT =>
                                if s_map_q = std_logic_vector(to_unsigned(C_T_WALL2, 4)) then
                                    s_cw2 <= '1';
                                else
                                    s_cw2 <= '0';
                                end if;
                                s_gst <= GN_CWR;
                            when GN_CWR =>
                                s_mw_addr <= s_cy & s_cx;
                                if s_cw2 = '1' and s_pw2 = '0' then
                                    s_mw_data <= std_logic_vector(to_unsigned(C_T_DOOR, 4));
                                else
                                    s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                end if;
                                s_mw_we <= '1';
                                s_pw2   <= s_cw2;
                                s_gst   <= GN_CRD;
                                if s_cvert = '0' then
                                    if s_cx = s_cb then
                                        -- switch to the vertical run at this room's column
                                        s_cx <= s_bx;
                                        s_cy <= s_vlo;
                                        s_cb <= s_vhi;
                                        s_cvert <= '1';
                                        s_pw2   <= '0';
                                    else
                                        s_cx <= s_cx + 1;
                                    end if;
                                else
                                    if s_cy = s_cb then
                                        if s_gi = C_NROOM - 1 then s_gi <= 0; s_gst <= GN_PLACE;
                                        else s_gi <= s_gi + 1; s_gp <= s_gi; s_gst <= GN_CORR; end if;
                                    else
                                        s_cy <= s_cy + 1;
                                    end if;
                                end if;
                            when GN_PLACE =>
                                -- room 0 spawn; room 1 a sword (odd depths) or a second
                                -- chest (even); room 2 chest; last room stairs (depth 7:
                                -- the dragon's lair, no stairs).  Stairs go LAST: if two
                                -- room centres coincide the item is lost, never the exit.
                                case s_gi is
                                    when 0 =>
                                        s_px <= s_r0x;                     -- room 0
                                        s_py <= s_r0y;
                                        s_mw_addr <= unsigned(s_rc_q(11 downto 0));  -- room 1
                                        if s_depth(0) = '1' then
                                            s_mw_data <= std_logic_vector(to_unsigned(C_T_WEAPON, 4));
                                        else
                                            s_mw_data <= std_logic_vector(to_unsigned(C_T_CHEST, 4));
                                        end if;
                                        s_mw_we   <= '1';
                                        s_gi <= 1;
                                    when 1 =>
                                        s_mw_addr <= unsigned(s_rc_q(11 downto 0));  -- room 2
                                        s_mw_data <= std_logic_vector(to_unsigned(C_T_CHEST, 4));
                                        s_mw_we   <= '1';
                                        s_gi <= 2;
                                    when others =>
                                        s_mw_addr <= unsigned(s_rc_q(11 downto 0));  -- room 7
                                        s_mw_data <= std_logic_vector(to_unsigned(C_T_STAIRS, 4));
                                        if s_depth /= 7 then s_mw_we <= '1'; end if;
                                        -- the stairs room's guard stands one cell west
                                        -- of the stairs, not on them (room width >= 5
                                        -- and that room is carved last, so the cell is
                                        -- its own interior)
                                        s_gx_guard <= unsigned(s_rc_q(5 downto 0)) - 1;
                                        s_gi <= 0;
                                        s_gr <= 3;
                                        s_gst <= GN_MON;
                                end case;
                            when GN_MON =>
                                -- monsters in rooms 3..NROOM-1 (spawn/weapon/chest rooms
                                -- 0..2 stay clear), one per room, count via the mask
                                -- from start_gen; type from the depth table.  Slot 5
                                -- is the dragon, on the depth-7 stairs cell.
                                -- s_rc_q = room s_gr (and s_gr = 7 for slot 5)
                                if s_gi = C_MONS - 1 then
                                    s_nm_t <= to_unsigned(C_BOSS, 3);
                                    s_nm_x <= unsigned(s_rc_q(5 downto 0));
                                else
                                    s_nm_t <= to_unsigned(mon_pool(s_depth, s_glfsr(1 downto 0)), 3);
                                    if s_gr = C_NROOM - 1 then
                                        s_nm_x <= s_gx_guard;
                                    else
                                        s_nm_x <= unsigned(s_rc_q(5 downto 0));
                                    end if;
                                end if;
                                s_nm_y <= unsigned(s_rc_q(11 downto 6));
                                -- slot select resolved here (one-hot + enable), so
                                -- GN_MONW's table writes carry no index decode
                                s_nm_en <= s_mon_en(s_gi);
                                for j in 0 to C_MONS - 1 loop
                                    if j = s_gi then s_nm_oh(j) <= '1';
                                    else             s_nm_oh(j) <= '0'; end if;
                                end loop;
                                s_gst <= GN_MONH;
                            when GN_MONH =>
                                s_nm_hp <= to_unsigned(C_MON_HP(to_integer(s_nm_t)) + s_dbon_hp, 8);
                                s_gst <= GN_MONW;
                            when GN_MONW =>
                                -- an unspawned slot just goes inactive (its stale
                                -- position/type are never read while inactive)
                                for j in 0 to C_MONS - 1 loop
                                    if s_nm_oh(j) = '1' then
                                        s_mdx(j) <= "00";          -- a fresh spawn doesn't glide
                                        s_mdy(j) <= "00";
                                        s_mactive(j) <= s_nm_en;
                                        s_mx(j)    <= s_nm_x;
                                        s_my(j)    <= s_nm_y;
                                        s_mtype(j) <= s_nm_t;

                                    end if;
                                end loop;
                                s_hpw_we   <= '1';                  -- s_nm_oh = slot s_gi
                                s_hpw_addr <= to_unsigned(s_gi, 3);
                                s_hpw_data <= std_logic_vector(s_nm_hp);
                                if s_gi = C_MONS - 1 then
                                    s_gst <= GN_DONE;
                                else
                                    s_gi <= s_gi + 1;
                                    if s_gr /= C_NROOM - 1 then s_gr <= s_gr + 1; end if;
                                    s_gst <= GN_MON;
                                end if;
                            when GN_DONE =>
                                s_gensp <= '0';
                                s_ts <= TS_IDLE;
                                s_game <= G_EXPLORE;
                                if s_depth = 7 then say(M_DRAGON); else say(M_DEPTH); end if;
                        end case;

                    ----------------------------------------------------------
                    when G_EXPLORE =>
                        case s_ts is
                            when TS_IDLE =>
                                if v_go = '1' then
                                    s_turn_pending <= '0';
                                    s_tcnt <= s_tcnt + 1;
                                    s_ts <= TS_PMON;
                                elsif v_ugo = '1' then
                                    s_use_pending <= '0';
                                    s_ts <= TS_USE;
                                end if;
                            when TS_PMON =>
                                -- compute the player target cell (heading, clamped)
                                v_tx := to_integer(s_px); v_ty := to_integer(s_py);
                                case to_integer(s_dir) is
                                    when 0 => v_ty := v_ty - 1;                       -- N
                                    when 1 => v_tx := v_tx + 1; v_ty := v_ty - 1;     -- NE
                                    when 2 => v_tx := v_tx + 1;                       -- E
                                    when 3 => v_tx := v_tx + 1; v_ty := v_ty + 1;     -- SE
                                    when 4 => v_ty := v_ty + 1;                       -- S
                                    when 5 => v_tx := v_tx - 1; v_ty := v_ty + 1;     -- SW
                                    when 6 => v_tx := v_tx - 1;                       -- W
                                    when others => v_tx := v_tx - 1; v_ty := v_ty - 1; -- NW
                                end case;
                                if v_tx < 1 then v_tx := 1; elsif v_tx > C_MAP - 2 then v_tx := C_MAP - 2; end if;
                                if v_ty < 1 then v_ty := 1; elsif v_ty > C_MAP - 2 then v_ty := C_MAP - 2; end if;
                                s_tx <= to_unsigned(v_tx, 6);
                                s_ty <= to_unsigned(v_ty, 6);
                                s_g_raddr <= to_unsigned(v_ty, 6) & to_unsigned(v_tx, 6);
                                s_ts <= TS_PSW;
                            when TS_PSW =>
                                -- (p_mmatch sees the new target this cycle)  MP
                                -- regenerates one point every 4th turn
                                if s_tcnt = "00" and s_mp < s_maxmp then
                                    s_mp <= s_mp + 1;
                                end if;
                                s_ts <= TS_PSCAN;
                            when TS_PSCAN =>
                                -- attack if a monster occupies the target cell
                                v_occ := '0';
                                for j in 0 to C_MONS - 1 loop
                                    if s_mmatch(j) = '1' then
                                        v_occ := '1';
                                        s_ctarget <= j;
                                        s_cmt  <= s_mtype(j);
                                        s_cmhp <= unsigned(s_hpr_q);   -- highest match
                                    end if;
                                end loop;
                                if v_occ = '1' then
                                    s_game <= G_COMBAT;
                                    s_cs <= CS_INIT;
                                    s_ts <= TS_IDLE;
                                else
                                    s_ts <= TS_PWAIT;
                                end if;
                            when TS_PWAIT =>
                                -- latch the target tile's flags (map read issued
                                -- in TS_PMON, valid since TS_PSW)
                                v_dest := to_integer(unsigned(s_map_q));
                                if walkable(s_map_q) then s_t_walk <= '1'; else s_t_walk <= '0'; end if;
                                if v_dest = C_T_STAIRS then s_t_stairs <= '1'; else s_t_stairs <= '0'; end if;
                                if v_dest = C_T_WEAPON then s_t_weapon <= '1'; else s_t_weapon <= '0'; end if;
                                if v_dest = C_T_CHEST  then s_t_chest  <= '1'; else s_t_chest  <= '0'; end if;
                                s_ts <= TS_PMOVE;
                            when TS_PMOVE =>
                                s_pdx <= "00"; s_pdy <= "00";
                                if s_t_walk = '1' then
                                    s_px <= s_tx;
                                    s_py <= s_ty;
                                    -- the step taken (heading; border clamps can't
                                    -- shorten it: the border is wall)
                                    case to_integer(s_dir) is
                                        when 0 =>      s_pdy <= "11";
                                        when 1 => s_pdx <= "01"; s_pdy <= "11";
                                        when 2 => s_pdx <= "01";
                                        when 3 => s_pdx <= "01"; s_pdy <= "01";
                                        when 4 =>      s_pdy <= "01";
                                        when 5 => s_pdx <= "11"; s_pdy <= "01";
                                        when 6 => s_pdx <= "11";
                                        when others => s_pdx <= "11"; s_pdy <= "11";
                                    end case;
                                    if s_t_stairs = '1' then
                                        -- descend: regenerate a deeper level
                                        if to_integer(s_depth) < 7 then
                                            start_gen(to_integer(s_depth) + 1);
                                        else
                                            start_gen(7);
                                        end if;
                                    else
                                        if s_t_weapon = '1' then
                                            if s_patk < 27 then s_patk <= s_patk + 2; end if;
                                            s_mw_addr <= s_ty & s_tx;   -- pick up: clear cell
                                            s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                            s_mw_we   <= '1';
                                            say(M_SWORD);
                                        elsif s_t_chest = '1' then
                                            -- gold (the score), and a potion half the time
                                            s_gold <= s_gold
                                                + to_unsigned(10 + to_integer(unsigned(s_clfsr(4 downto 0))), 12);
                                            s_mw_addr <= s_ty & s_tx;
                                            s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                            s_mw_we   <= '1';
                                            if s_clfsr(5) = '1' and s_pots < 9 then
                                                s_pots <= s_pots + 1;
                                                say(M_GOLDPOT);
                                            else
                                                say(M_GOLD);
                                            end if;
                                        end if;
                                        s_ts <= TS_MSTART;
                                    end if;
                                else
                                    s_ts <= TS_MSTART;
                                end if;
                            when TS_USE =>
                                -- S9: drink a potion (+maxHP/2); costs the turn.  None
                                -- left, or already at full HP: a message, no turn.
                                s_pdx <= "00"; s_pdy <= "00";
                                if s_pots = 0 then
                                    say(M_NOPOT);
                                    s_ts <= TS_IDLE;
                                elsif s_hpfull = '1' then
                                    say(M_FULL);
                                    s_ts <= TS_IDLE;
                                else
                                    s_pots <= s_pots - 1;
                                    s_hsum <= to_integer(s_hp) + to_integer(s_maxhp(6 downto 1));
                                    s_fb_dlt <= to_integer(s_maxhp(6 downto 1));
                                    s_ts <= TS_USE2;
                                end if;
                            when TS_USE2 =>
                                if s_hsum > to_integer(s_maxhp) then
                                    s_hp <= s_maxhp;
                                else
                                    s_hp <= to_unsigned(s_hsum, 8);
                                end if;
                                say(M_DRANK);
                                s_ts <= TS_MSTART;
                            when TS_MSTART =>
                                -- the monsters' half of the turn (held off for two
                                -- turns after a successful flee).  Glide: forget
                                -- last turn's steps, restart the glide distance.
                                s_mi <= 0;
                                s_mdx <= (others => "00");
                                s_mdy <= (others => "00");
                                s_mg_start <= '1';
                                if s_grace /= 0 then
                                    s_grace <= s_grace - 1;
                                    s_ts <= TS_IDLE;
                                else
                                    s_ts <= TS_MREQ;
                                end if;
                            when TS_MREQ =>
                                -- isolate the index mux: just latch this monster
                                -- (skip inactive slots and the stationary dragon)
                                if s_sw_peace = '1' then
                                    s_ts <= TS_IDLE;    -- peaceful: monsters don't act
                                elsif s_mmob(s_mi) = '0' then
                                    if s_mi = C_MONS - 1 then s_ts <= TS_IDLE;
                                    else s_mi <= s_mi + 1; end if;
                                else
                                    s_sel_mx <= s_mx(s_mi);
                                    s_sel_my <= s_my(s_mi);
                                    s_sel_mt <= s_mtype(s_mi);
                                    s_wdir  <= s_clfsr(1 downto 0);
                                    s_wmove <= s_clfsr(2);
                                    s_ts <= TS_MCALC;
                                end if;
                            when TS_MCALC =>
                                -- deltas + abs + sign/nonzero (no compare/step yet)
                                mxi := to_integer(s_sel_mx);
                                myi := to_integer(s_sel_my);
                                pxi := to_integer(s_px); pyi := to_integer(s_py);
                                ddx := pxi - mxi; ddy := pyi - myi;
                                s_sel_mhp <= unsigned(s_hpr_q);   -- slot s_mi, read in TS_MREQ
                                if ddx < 0 then s_adx <= -ddx; else s_adx <= ddx; end if;
                                if ddy < 0 then s_ady <= -ddy; else s_ady <= ddy; end if;
                                if ddx > 0 then s_ddxp <= '1'; else s_ddxp <= '0'; end if;
                                if ddx /= 0 then s_ddxn <= '1'; else s_ddxn <= '0'; end if;
                                if ddy > 0 then s_ddyp <= '1'; else s_ddyp <= '0'; end if;
                                if ddy /= 0 then s_ddyn <= '1'; else s_ddyn <= '0'; end if;
                                s_ts <= TS_MAGG;
                            when TS_MAGG =>
                                -- chase only within C_AGGRO cells
                                if s_adx <= C_AGGRO and s_ady <= C_AGGRO then
                                    s_aggro <= '1';
                                else
                                    s_aggro <= '0';
                                end if;
                                s_ts <= TS_MSTEP;
                            when TS_MSTEP =>
                                mxi := to_integer(s_sel_mx);
                                myi := to_integer(s_sel_my);
                                v_tx := mxi; v_ty := myi;
                                s_alt_tx <= s_sel_mx;
                                s_alt_ty <= s_sel_my;
                                s_alt_ok <= '0';
                                -- step direction codes for the glide ("01" +1,
                                -- "11" -1, "00" none), main step and fallback
                                s_cdx <= "00"; s_cdy <= "00";
                                s_adxd <= "00"; s_adyd <= "00";
                                if s_aggro = '1' then
                                    -- chase: step along the longer axis; the other
                                    -- axis is the fallback if that cell is blocked
                                    if s_adx >= s_ady and s_ddxn = '1' then
                                        if s_ddxp = '1' then v_tx := mxi + 1; s_cdx <= "01";
                                        else                 v_tx := mxi - 1; s_cdx <= "11"; end if;
                                        if s_ddyn = '1' then
                                            s_alt_ok <= '1';
                                            if s_ddyp = '1' then s_alt_ty <= s_sel_my + 1; s_adyd <= "01";
                                            else                 s_alt_ty <= s_sel_my - 1; s_adyd <= "11"; end if;
                                        end if;
                                    elsif s_ddyn = '1' then
                                        if s_ddyp = '1' then v_ty := myi + 1; s_cdy <= "01";
                                        else                 v_ty := myi - 1; s_cdy <= "11"; end if;
                                        if s_ddxn = '1' then
                                            s_alt_ok <= '1';
                                            if s_ddxp = '1' then s_alt_tx <= s_sel_mx + 1; s_adxd <= "01";
                                            else                 s_alt_tx <= s_sel_mx - 1; s_adxd <= "11"; end if;
                                        end if;
                                    end if;
                                elsif s_wmove = '1' then
                                    -- out of range: wander one cell, half the turns
                                    case s_wdir is
                                        when "00"   => v_ty := myi - 1; s_cdy <= "11";
                                        when "01"   => v_tx := mxi + 1; s_cdx <= "01";
                                        when "10"   => v_ty := myi + 1; s_cdy <= "01";
                                        when others => v_tx := mxi - 1; s_cdx <= "11";
                                    end case;
                                end if;
                                s_tx <= to_unsigned(v_tx, 6);
                                s_ty <= to_unsigned(v_ty, 6);
                                s_alt_used <= '0';
                                s_ts <= TS_MBUMP;
                            when TS_MBUMP =>
                                -- bump check on the registered target (short path)
                                if s_tx = s_px and s_ty = s_py then
                                    s_ctarget <= s_mi;          -- bump -> combat
                                    s_cmt  <= s_sel_mt;
                                    s_cmhp <= s_sel_mhp;
                                    s_game <= G_COMBAT;
                                    s_cs <= CS_INIT;
                                    s_ts <= TS_IDLE;
                                else
                                    s_g_raddr <= s_ty & s_tx;
                                    s_ts <= TS_MWAIT;
                                end if;
                            when TS_MWAIT =>
                                -- occupancy = any registered match except self
                                v_occ := '0';
                                for j in 0 to C_MONS - 1 loop
                                    if j /= s_mi and s_mmatch(j) = '1' then
                                        v_occ := '1';
                                    end if;
                                end loop;
                                s_mocc <= v_occ;
                                s_ts <= TS_MWALK;
                            when TS_MWALK =>
                                -- latch the target's walkability (map read issued
                                -- in TS_MBUMP, valid since TS_MWAIT)
                                if walkable(s_map_q) then s_mwalk <= '1'; else s_mwalk <= '0'; end if;
                                s_ts <= TS_MAPPLY;
                            when TS_MAPPLY =>
                                if s_mwalk = '1' and s_mocc = '0' then
                                    s_mx(s_mi) <= s_tx;
                                    s_my(s_mi) <= s_ty;
                                    s_mdx(s_mi) <= s_cdx;           -- glide from the old cell
                                    s_mdy(s_mi) <= s_cdy;
                                    if s_mi = C_MONS - 1 then s_ts <= TS_IDLE;
                                    else s_mi <= s_mi + 1; s_ts <= TS_MREQ; end if;
                                elsif s_alt_ok = '1' and s_alt_used = '0' then
                                    -- blocked: one retry along the other axis
                                    s_tx <= s_alt_tx;
                                    s_ty <= s_alt_ty;
                                    s_cdx <= s_adxd;
                                    s_cdy <= s_adyd;
                                    s_alt_used <= '1';
                                    s_ts <= TS_MBUMP;
                                else
                                    if s_mi = C_MONS - 1 then s_ts <= TS_IDLE;
                                    else s_mi <= s_mi + 1; s_ts <= TS_MREQ; end if;
                                end if;
                        end case;

                    ----------------------------------------------------------
                    -- Combat: one round per S7 flip (K3 picks the action) or S9
                    -- flip (drink a potion).  One arithmetic op per state.
                    when G_COMBAT =>
                        case s_cs is
                            when CS_INIT =>
                                -- pre-resolve monster combat stats to scalars
                                mt := to_integer(s_cmt);
                                s_cmatk <= C_MON_ATK(mt);
                                s_cmdef <= C_MON_DEF(mt);
                                s_cmxp  <= C_MON_XP(mt);
                                s_fb_res <= R_NONE;
                                s_fb_dv  <= '0';
                                s_fb_tv  <= '0';
                                s_cs <= CS_INIT2;
                            when CS_INIT2 =>
                                -- deeper monsters hit harder
                                s_cmatk <= s_cmatk + s_dbon_atk;
                                s_cmdef <= s_cmdef + s_dbon_def;
                                s_cmxp  <= s_cmxp + s_dbon_xp;
                                s_cs <= CS_INPUT;
                            when CS_INPUT =>
                                if v_go = '1' or v_ugo = '1' then
                                    if v_go = '1' then
                                        s_turn_pending <= '0';
                                        v_act := s_cb_action;      -- latch action
                                    else
                                        s_use_pending <= '0';
                                        v_act := 3;                -- S9 = Item
                                    end if;
                                    s_cb_act_l <= v_act;
                                    s_stun <= '0';
                                    s_fled <= '0';
                                    s_cs <= CS_PLAYER;
                                end if;
                            when CS_PLAYER =>
                                -- resolve the action: kind + pre-DEF damage
                                var := to_integer(unsigned(s_clfsr(1 downto 0)));
                                s_fb_dv <= '0';
                                s_fb_tv <= '0';
                                s_pk <= PK_NONE;
                                case s_cb_act_l is
                                    when 0 =>                              -- Attack
                                        s_pk <= PK_DMG;
                                        s_raw <= s_patk + var;
                                        s_fb_res <= R_HIT;
                                    when 1 =>                              -- Defend
                                        s_pk <= PK_DEF;
                                        s_fb_res <= R_BLCK;
                                    when 2 =>                              -- Cast
                                        if s_mp < s_cost then
                                            s_fb_res <= R_NOMP;            -- wasted round
                                        else
                                            s_mp <= s_mp - s_cost;
                                            case to_integer(s_class) is
                                                when 0 =>                  -- Warrior BASH
                                                    s_pk <= PK_DMG;
                                                    s_raw <= s_patk + var;
                                                    s_stun <= '1';
                                                    s_fb_res <= R_BASH;
                                                when 1 =>                  -- Mage FIRE
                                                    s_pk <= PK_FIRE;
                                                    s_raw <= s_patk + 8;
                                                    s_fb_res <= R_FIRE;
                                                when 2 =>                  -- Rogue STAB
                                                    s_pk <= PK_DMG;
                                                    if s_clfsr(2) = '1' then
                                                        s_raw <= s_patk + s_patk;
                                                        s_fb_res <= R_CRIT;
                                                    else
                                                        s_raw <= s_patk + var;
                                                        s_fb_res <= R_STAB;
                                                    end if;
                                                when others =>             -- Cleric HEAL
                                                    s_pk <= PK_HEAL;
                                                    s_fb_res <= R_HEAL;
                                            end case;
                                        end if;
                                    when 3 =>                              -- Item: potion
                                        if s_pots = 0 then
                                            s_fb_res <= R_NOPOT;           -- wasted round
                                        else
                                            s_pots <= s_pots - 1;
                                            s_pk <= PK_HEAL;
                                            s_fb_res <= R_HEAL;
                                        end if;
                                    when others =>                         -- Flee
                                        s_pk <= PK_FLEE;
                                        s_fb_res <= R_FAIL;
                                        if to_integer(s_class) = C_CLS_ROG or s_clfsr(2) = '1' then
                                            s_fled <= '1';
                                        end if;
                                end case;
                                s_cs <= CS_PSUB;
                            when CS_PSUB =>
                                case s_pk is
                                    when PK_DMG  => s_diff <= s_raw - s_cmdef;
                                    when PK_FIRE => s_diff <= s_raw;           -- ignores DEF
                                    when PK_HEAL =>
                                        s_hsum <= to_integer(s_hp) + to_integer(s_maxhp(6 downto 1));
                                        s_fb_dlt <= to_integer(s_maxhp(6 downto 1));
                                        s_fb_dv  <= '1';
                                    when PK_DEF =>                             -- +2 MP
                                        v_mp := to_integer(s_mp) + 2;
                                        if v_mp > to_integer(s_maxmp) then s_mp <= s_maxmp;
                                        else s_mp <= to_unsigned(v_mp, 5); end if;
                                    when others => null;
                                end case;
                                s_cs <= CS_PCLAMP;
                            when CS_PCLAMP =>
                                if s_pk = PK_DMG or s_pk = PK_FIRE then
                                    if s_diff < 1 then v_d := 1; else v_d := s_diff; end if;
                                    s_dmg <= v_d;
                                    s_fb_dlt <= v_d;
                                    s_fb_dv  <= '1';
                                elsif s_pk = PK_HEAL then
                                    if s_hsum > to_integer(s_maxhp) then s_hp <= s_maxhp;
                                    else s_hp <= to_unsigned(s_hsum, 8); end if;
                                end if;
                                s_cs <= CS_PHIT;
                            when CS_PHIT =>
                                s_mon_dead <= '0';
                                s_mhd <= to_integer(s_cmhp) - s_dmg;
                                s_cs <= CS_PHIT2;
                            when CS_PHIT2 =>
                                -- apply to monster hp; flag a kill
                                if s_pk = PK_DMG or s_pk = PK_FIRE then
                                    s_hitflash <= "110";           -- monster flashes white
                                    if s_mhd <= 0 then
                                        s_cmhp <= (others => '0');
                                        s_hpw_we   <= '1';
                                        s_hpw_addr <= to_unsigned(s_ctarget, 3);
                                        s_hpw_data <= (others => '0');
                                        s_mon_dead <= '1';
                                    else
                                        s_cmhp <= to_unsigned(s_mhd, 8);
                                        s_hpw_we   <= '1';
                                        s_hpw_addr <= to_unsigned(s_ctarget, 3);
                                        s_hpw_data <= std_logic_vector(to_unsigned(s_mhd, 8));
                                    end if;
                                end if;
                                s_cs <= CS_POUT;
                            when CS_POUT =>
                                -- outcome branch (kill / flee / bash / counterattack)
                                if s_mon_dead = '1' then
                                    s_mactive(s_ctarget) <= '0';
                                    if s_cmt = to_unsigned(C_BOSS, 3) then
                                        s_game <= G_VICTORY;           -- the dragon is dead
                                        s_cs <= CS_INPUT;
                                    else
                                        v_xp := to_integer(s_xp) + s_cmxp;
                                        if v_xp > 255 then v_xp := 255; end if;
                                        s_xp  <= to_unsigned(v_xp, 8);
                                        s_xpg <= s_cmxp;
                                        if s_clfsr(1 downto 0) = "00" and s_pots < 9 then
                                            s_pots <= s_pots + 1;      -- 1 in 4 drop a potion
                                            say(M_SLAINPOT);
                                        else
                                            say(M_SLAIN);
                                        end if;
                                        s_cs <= CS_LVL;
                                    end if;
                                elsif s_pk = PK_FLEE and s_fled = '1' then
                                    s_game <= G_EXPLORE;               -- fled
                                    s_cs <= CS_INPUT;
                                    s_grace <= "10";
                                    say(M_FLED);
                                elsif s_stun = '1' then
                                    s_cs <= CS_INPUT;                  -- bashed: it loses its turn
                                else
                                    s_cs <= CS_MONSTER;
                                end if;
                            when CS_LVL =>
                                -- level-up when xp >= level*8 (a shift)
                                s_xpd <= to_integer(s_xp) - to_integer(s_level) * 8;
                                s_cs <= CS_LVL2;
                            when CS_LVL2 =>
                                -- level up: +3 max HP, +1 ATK, +1 DEF on even
                                -- levels, +1 max MP, full HP and MP
                                if s_xpd >= 0 and s_level < 30 then
                                    s_xp    <= to_unsigned(s_xpd, 8);
                                    s_level <= s_level + 1;
                                    s_maxhp <= s_maxhp + 3;
                                    s_hp    <= s_maxhp + 3;
                                    if s_patk < 28 then s_patk <= s_patk + 1; end if;
                                    if s_level(0) = '1' and s_pdef < 28 then s_pdef <= s_pdef + 1; end if;
                                    if s_maxmp < 31 then
                                        s_maxmp <= s_maxmp + 1;
                                        s_mp    <= s_maxmp + 1;
                                    else
                                        s_mp    <= s_maxmp;
                                    end if;
                                    say(M_LEVEL);
                                end if;
                                s_game <= G_EXPLORE;
                                s_cs <= CS_INPUT;
                            when CS_MONSTER =>
                                -- monster damage (one op per state)
                                var2 := to_integer(unsigned(s_clfsr(4 downto 3)));
                                s_raw <= s_cmatk + var2;
                                s_cs <= CS_MSUB;
                            when CS_MSUB =>
                                s_diff <= s_raw - s_pdef;
                                s_cs <= CS_MCLAMP;
                            when CS_MCLAMP =>
                                v_d := s_diff;
                                if s_pk = PK_DEF then                  -- Defend halves it
                                    v_d := to_integer(shift_right(to_signed(v_d, 8), 1));
                                end if;
                                if v_d < 1 then v_d := 1; end if;
                                s_mdmg   <= v_d;
                                s_fb_tkn <= v_d;
                                s_fb_tv  <= '1';
                                s_cs <= CS_MHIT;
                            when CS_MHIT =>
                                s_hurtflash <= "110";              -- screen edge flashes red
                                newhp := to_integer(s_hp) - s_mdmg;
                                if newhp <= 0 then
                                    s_hp <= (others => '0');
                                    s_game <= G_GAMEOVER;
                                else
                                    s_hp <= to_unsigned(newhp, 8);
                                end if;
                                s_cs <= CS_INPUT;
                        end case;

                    ----------------------------------------------------------
                    when G_GAMEOVER | G_VICTORY =>
                        if v_go = '1' then
                            s_turn_pending <= '0';
                            s_game <= G_CREATE;
                        end if;

                end case;
            end if;

            -- latch flips for the next field tick (after the case, so a flip
            -- landing on the tick cycle itself is kept, not cleared).  Flips
            -- during a running turn or level generation are dropped; S9 only
            -- counts while exploring or fighting.
            if v_edge = '1' and v_newedge = '0' and s_ts = TS_IDLE
               and s_game /= G_GEN then
                s_turn_pending <= '1';
            end if;
            if v_uedge = '1' and v_newedge = '0' and s_ts = TS_IDLE
               and (s_game = G_EXPLORE or s_game = G_COMBAT) then
                s_use_pending <= '1';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- HUD: BCD digit ROM, layout ROM + field RAM, and the per-field sequencer.
    --------------------------------------------------------------------------
    p_bcd : process(clk)
    begin
        if rising_edge(clk) then
            s_bcd_q <= s_bcd(to_integer(s_bcd_addr));
        end if;
    end process;

    s_fr_rd <= s_fram(to_integer(unsigned(s_p2_g(5 downto 0))));
    p_hudmem : process(clk)
    begin
        if rising_edge(clk) then
            s_lr_q <= s_lrom(to_integer(s_lr_addr));
            if s_fw_we = '1' then
                s_fram(to_integer(s_fw_addr)) <= s_fw_data;
            end if;
            s_fr_q <= s_fr_rd;
        end if;
    end process;

    -- Once per field, 256 clocks after the tick (so the turn or combat round
    -- that started on the same tick has settled): numbers -> digits ->
    -- field RAM, names -> field RAM, then both lines streamed into the HUD
    -- char RAM through a 4-stage pipeline:
    --   issue (layout ROM addr) -> ROM data -> fabric reg (= field RAM addr)
    --   -> field RAM data -> HUD write (field glyph for 64+k codes).
    -- ~450 clocks in all, well inside vertical blank.
    p_hud : process(clk)
        variable v_val : integer range 0 to 4095;
    begin
        if rising_edge(clk) then
            s_fw_we <= '0';
            s_p0_v  <= '0';
            s_p0_n  <= '0';
            -- line-stream pipeline
            s_p1_v <= s_p0_v;
            s_p2_v <= s_p1_v;
            s_p3_v <= s_p2_v;
            s_p1_n <= s_p0_n;
            s_p2_n <= s_p1_n;
            s_p3_n <= s_p2_n;
            s_p2_g <= s_lr_q;
            s_p3_g <= s_p2_g;
            s_hr_we    <= s_p3_v and not s_p3_n;
            -- HUD RAM write address: 64 at the stream start, +1 per write
            if s_hu_state = HU_NDRAIN then
                s_hr_waddr <= to_unsigned(64, 7);
            elsif s_hr_we = '1' then
                s_hr_waddr <= s_hr_waddr + 1;
            end if;
            if s_p3_g(6) = '1' then
                s_hr_wdata <= s_fr_q;
            else
                s_hr_wdata <= s_p3_g;
            end if;

            case s_hu_state is
                when HU_IDLE =>
                    if s_ftick = '1' then
                        s_hu_state <= HU_DLY;
                    end if;
                when HU_DLY =>
                    -- s_ecnt restarted on the same tick: 256 clocks
                    if s_ecnt = 255 then
                        s_hu_f <= 0;
                        case s_game is
                            when G_CREATE   => s_hu_ltop <= L_CREATE;
                            when G_GEN      => s_hu_ltop <= L_GEN;
                            when G_COMBAT   => s_hu_ltop <= L_FIGHT;
                            when G_GAMEOVER => s_hu_ltop <= L_DIED;
                            when G_VICTORY  => s_hu_ltop <= L_WIN;
                            when others =>
                                if s_msgtmr /= 0 then s_hu_ltop <= s_msg_lay;
                                else                  s_hu_ltop <= L_STATS; end if;
                        end case;
                        s_hu_state <= HU_FREQ;
                    end if;
                when HU_FREQ =>
                    -- present field s_hu_f's value to the BCD ROM
                    case s_hu_f is
                        when F_HP   => v_val := to_integer(s_hp);
                        when F_MHP  => v_val := to_integer(s_maxhp);
                        when F_MP   => v_val := to_integer(s_mp);
                        when F_MMP  => v_val := to_integer(s_maxmp);
                        when F_GOLD => v_val := to_integer(s_gold);
                        when F_LV   => v_val := to_integer(s_level);
                        when F_D    => v_val := to_integer(s_depth);
                        when F_ATK  => v_val := s_patk;
                        when F_DEF  => v_val := s_pdef;
                        when F_XP   => v_val := to_integer(s_xp);
                        when F_XPN  => v_val := to_integer(s_level) * 8;
                        when F_POT  => v_val := to_integer(s_pots);
                        when F_EHP  => v_val := to_integer(s_cmhp);
                        when F_DLT  => v_val := s_fb_dlt;
                        when F_TKN  => v_val := s_fb_tkn;
                        when others => v_val := s_xpg;
                    end case;
                    if v_val > 999 then v_val := 999; end if;
                    s_bcd_addr <= to_unsigned(v_val, 10);
                    s_hu_w <= C_FLD_W(s_hu_f);
                    s_hu_k <= to_unsigned(C_FLD_BASE(s_hu_f), 6);
                    if (s_hu_f = F_DLT and s_fb_dv = '0') or (s_hu_f = F_TKN and s_fb_tv = '0') then
                        s_hu_blank <= '1';
                    else
                        s_hu_blank <= '0';
                    end if;
                    s_hu_state <= HU_FWAIT;
                when HU_FWAIT =>
                    s_hu_state <= HU_FLAT;               -- BCD ROM read in flight
                when HU_FLAT =>
                    -- align the shown digits to bits 11:8 (width 3 / 2 / 1)
                    case s_hu_w is
                        when 3      => s_hu_bcd <= s_bcd_q;
                        when 2      => s_hu_bcd <= s_bcd_q(7 downto 0) & "0000";
                        when others => s_hu_bcd <= s_bcd_q(3 downto 0) & "00000000";
                    end case;
                    s_hu_j <= 0;
                    s_hu_state <= HU_FDIG;
                when HU_FDIG =>
                    -- one digit per clock into the field RAM
                    s_fw_we   <= '1';
                    s_fw_addr <= s_hu_k;
                    if s_hu_blank = '1' then
                        s_fw_data <= std_logic_vector(to_unsigned(C_G_SP, 7));
                    else
                        s_fw_data <= std_logic_vector(to_unsigned(C_G_D0, 7)
                                     + resize(unsigned(s_hu_bcd(11 downto 8)), 7));
                    end if;
                    s_hu_bcd <= s_hu_bcd(7 downto 0) & "0000";
                    s_hu_k   <= s_hu_k + 1;
                    if s_hu_j = s_hu_w - 1 then
                        if s_hu_f = C_NFLD - 1 then
                            s_hu_n <= 0;
                            s_hu_k <= to_unsigned(C_NAME_BASE, 6);
                            s_hu_state <= HU_NAME;
                        else
                            s_hu_f <= s_hu_f + 1;
                            s_hu_state <= HU_FREQ;
                        end if;
                    else
                        s_hu_j <= s_hu_j + 1;
                    end if;
                when HU_NAME =>
                    -- copy the 13 name glyphs from the layout ROM's name rows
                    -- (column = index*4 + k) through the pipeline into field
                    -- RAM slots C_NAME_BASE + n (written at stage 2, below)
                    case s_hu_n is
                        when 0 | 1 | 2 =>
                            s_lr_addr <= to_unsigned(C_NROW_CLS, 5) & "00" & s_class
                                         & to_unsigned(s_hu_n, 2);
                        when 3 | 4 | 5 =>
                            s_lr_addr <= to_unsigned(C_NROW_MON, 5) & '0' & s_cmt
                                         & to_unsigned(s_hu_n - 3, 2);
                        when 6 | 7 | 8 =>
                            if s_cb_action = 2 then          -- Cast: the class's spell
                                s_lr_addr <= to_unsigned(C_NROW_ACT, 5)
                                             & to_unsigned(C_ACT_CAST0 + to_integer(s_class), 4)
                                             & to_unsigned(s_hu_n - 6, 2);
                            else
                                s_lr_addr <= to_unsigned(C_NROW_ACT, 5)
                                             & to_unsigned(s_cb_action, 4)
                                             & to_unsigned(s_hu_n - 6, 2);
                            end if;
                        when others =>
                            s_lr_addr <= to_unsigned(C_NROW_RES, 5)
                                         & to_unsigned(s_fb_res, 4)
                                         & to_unsigned(s_hu_n - 9, 2);
                    end case;
                    s_p0_v    <= '1';
                    s_p0_n    <= '1';
                    if s_hu_n = C_NNAME - 1 then
                        s_hu_dly <= (others => '0');
                        s_hu_state <= HU_NDRAIN;
                    else
                        s_hu_n <= s_hu_n + 1;
                    end if;
                when HU_NDRAIN =>
                    -- the last name glyph leaves the pipeline
                    s_hu_dly <= s_hu_dly + 1;
                    if s_hu_dly = 4 then
                        s_hu_col  <= (others => '0');
                        s_hu_line <= '1';                -- top line first
                        s_hu_state <= HU_LINE;
                    end if;
                when HU_LINE =>
                    -- issue one column per clock (HUD RAM bit 6 = top line)
                    if s_hu_line = '1' then
                        s_lr_addr <= to_unsigned(s_hu_ltop, 5) & s_hu_col;
                    else
                        s_lr_addr <= to_unsigned(L_BAR, 5) & s_hu_col;
                    end if;
                    s_p0_v    <= '1';
                    if s_hu_col = 63 then              -- all 64 (logo: 48-56)
                        s_hu_col <= (others => '0');
                        if s_hu_line = '1' then
                            s_hu_line <= '0';
                        else
                            s_hu_dly <= (others => '0');
                            s_hu_state <= HU_DRAIN;
                        end if;
                    else
                        s_hu_col <= s_hu_col + 1;
                    end if;
                when HU_DRAIN =>
                    -- let the last columns leave the pipeline
                    s_hu_dly <= s_hu_dly + 1;
                    if s_hu_dly = 7 then
                        s_hu_state <= HU_IDLE;
                    end if;
            end case;

            -- name mode, stage 2: the fabric copy of the ROM glyph -> field RAM
            if s_p2_v = '1' and s_p2_n = '1' then
                s_fw_we   <= '1';
                s_fw_addr <= s_hu_k;               -- C_NAME_BASE + n (set at HU_FDIG's end)
                s_hu_k    <= s_hu_k + 1;
                s_fw_data <= s_p2_g;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- HUD char RAM: 1 write port (update FSM, vblank) + 1 registered read port
    -- (render, addr = top-flag & column).  A block RAM, so the render-side
    -- glyph fetch is a clean registered read with no wide mux.
    --------------------------------------------------------------------------
    s1_glyph_rd <= s_hudram(to_integer(unsigned'(s0_hud_top & s0_hud_col)));
    p_hudram : process(clk)
    begin
        if rising_edge(clk) then
            if s_hr_we = '1' then
                s_hudram(to_integer(s_hr_waddr)) <= s_hr_wdata;
            end if;
            s1_glyph <= unsigned(s1_glyph_rd);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sync passthrough delay line (hsync / vsync / field only: 9 clocks from
    -- data_in to data_out in every mode).
    --------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            -- s_sdl_q = the syncs data_in carried 9 clocks ago (the old
            -- s_syncp(0..4) -> s5..s8_sync chain, '0' before that)
            s_sdl(to_integer(s_sdl_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            s_sdl_q  <= s_sdl(to_integer(s_sdl_ra));
            s_sdl_wa <= s_sdl_wa + 1;
            s_sdl_ra <= s_sdl_ra + 1;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Generated avid (cubist / aurora recipe, HW-confirmed on both).  g_ph is
    -- the hsync->avid-rise distance parity of this line, g_par the voted
    -- usual parity; a line whose source avid lands off-parity starts one
    -- clock later, so the generated start always sits at the same hsync
    -- parity.  It ends after the field-latched width, or at hsync at the
    -- latest.  The pixel pipeline is clocked from it, and it rides that
    -- pipeline to data_out.avid.
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
            g_avd <= g_avid;
        end if;
    end process;

    s_gav_start <= g_avid and not g_avd;

    --------------------------------------------------------------------------
    -- S0: screen pixel -> world coords (eased camera scroll), HUD strips,
    -- title logo, HP bar, screen-edge border band.
    -- Resolution scaling (v1.0.3): cur_x / cur_y are virtual 1080-line units
    -- (f_vstep_x / f_vstep_y; SD is a 1440-wide 4:3 window), cur_hx the HUD
    -- line's 1920-unit x (f_hstep_x).  A fractional step can jump over a
    -- value, so the bands are magnitude windows on the coordinates (at
    -- 1080p's step 1 they flip on exactly the pixels / lines the old equality
    -- flags did).  The y bands are computed a line ahead from s_yn.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable cur_x, cur_y, cur_hx : unsigned(11 downto 0);
        variable v_xe, v_ye, v_ht, v_hb, v_bx, v_bf, v_ly, v_lx : std_logic;
        variable v_lcy : unsigned(5 downto 0);
        variable v_hc : unsigned(4 downto 0);
        variable v_ny, v_xi, v_hxi : unsigned(11 downto 0);
        variable v_d, v_lcx : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- per-field steps (the field tick is in vblank, guarded); the
            -- screen in virtual units = the last line's end x / next line's y
            if s_ftick = '1' then
                s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
                s_hxs <= f_hstep_x(registers_in(8)(3 downto 0));
                s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
                s_tid <= registers_in(8)(3 downto 0);
                s_vw  <= s_xa(12 downto 1);
                -- (from fields starting at 0 only: a bottom field's half-step
                -- start would move every band down with it)
                if s_fbcur = '0' then
                    s_vh <= s_yn(19 downto 8);
                end if;
            end if;
            s_fbot <= not data_in.field_n;
            s_rx <= f_rim_x(s_tid, s_cl);
            s_ry <= f_rim_y(s_tid, s_cl);
            s_hsnap <= s_ys(11) or s_ys(10);       -- step >= 4.0 units

            -- x: this pixel's virtual position (0 at the generated avid start)
            if s_gav_start = '1' then
                cur_x := (others => '0'); cur_hx := (others => '0');
                s_xa  <= resize(s_xs, 13);
                s_hxa <= resize(s_hxs, 16);
            elsif g_avid = '1' then
                cur_x := s_xa(12 downto 1); cur_hx := s_hxa(15 downto 4);
                s_xa  <= s_xa + s_xs;
                s_hxa <= s_hxa + s_hxs;
            else
                cur_x := s_xa(12 downto 1); cur_hx := s_hxa(15 downto 4);
            end if;
            -- y: per line; the bottom field starts half a field step down
            s_yn <= s_ya + s_ys;
            if s_gav_start = '1' then
                if s_firstline = '1' then
                    s_fbcur <= s_fbot;
                    if s_fbot = '1' then
                        s_ya  <= resize(s_ys(11 downto 1), 20);
                        cur_y := resize(s_ys(11 downto 9), 12);
                    else
                        s_ya  <= (others => '0');
                        cur_y := (others => '0');
                    end if;
                else
                    s_ya  <= s_yn;
                    cur_y := s_yn(19 downto 8);
                end if;
            else
                cur_y := s_ya(19 downto 8);
            end if;

            s0_wx  <= signed(resize(cur_x, 14)) + s_scroll_x;
            s0_wy  <= signed(resize(cur_y, 14)) + s_scroll_y;
            s0_render <= g_avid;

            -- next line's bands (y is constant over a line): HUD top line
            -- [4, 36), bottom line [H-36, H-4) with the glyph row = y - start,
            -- logo [H/2-220, +64), border y < 24 or >= H-24
            v_ny := s_yn(19 downto 8);
            v_d := ('0' & v_ny) - C_HUD_MARG;
            if v_d(12 downto C_HUD_LOG2) = 0 then s_n_ht <= '1'; else s_n_ht <= '0'; end if;
            v_d := ('0' & v_ny) - ('0' & s_hud_y0);
            if v_d(12 downto C_HUD_LOG2) = 0 then
                s_n_hb <= '1'; s_n_hc <= v_d(4 downto 0);
            else
                s_n_hb <= '0'; s_n_hc <= v_ny(4 downto 0) - C_HUD_MARG;
            end if;
            v_d := ('0' & v_ny) - ('0' & s_logo_y0);
            if v_d(12 downto 6) = 0 then s_n_ly <= '1'; else s_n_ly <= '0'; end if;
            s_n_lcy <= v_d(5 downto 0);
            if v_ny < C_EDGE or v_ny >= s_edge_b then s_n_ye <= '1'; else s_n_ye <= '0'; end if;

            -- per-line flags, taken at the line start (the first line: y 0)
            v_ht := s_htop; v_hb := s_hbot; v_hc := s_hcnt;
            v_ly := s_lgy; v_lcy := s_lcy; v_ye := s_yedge;
            if s_gav_start = '1' then
                if s_firstline = '1' then
                    v_ht := '0'; v_hb := '0'; v_ly := '0'; v_ye := '1';
                else
                    v_ly := s_n_ly; v_lcy := s_n_lcy; v_ye := s_n_ye;
                    if s_hsnap = '0' then
                        v_ht := s_n_ht; v_hb := s_n_hb; v_hc := s_n_hc;
                    -- 480i: a field line (4.44 units) is longer than a font
                    -- row (4), so sampling by y would skip one row in eight
                    -- (the hardware sends both fields alike: field_n is tied
                    -- high).  Each HUD line instead runs 8 field lines from its
                    -- first line, one font row per line (glyphs 8 field lines
                    -- tall, ~11% over the proportional 7.2).
                    elsif (s_htop = '1' or s_hbot = '1') and s_hcnt(4 downto 2) /= "111" then
                        v_hc := s_hcnt + 4;
                    elsif s_htop = '0' and s_hbot = '0' and (s_n_ht = '1' or s_n_hb = '1') then
                        v_ht := s_n_ht; v_hb := s_n_hb; v_hc := (others => '0');
                    else
                        v_ht := '0'; v_hb := '0';
                    end if;
                end if;
            end if;
            s_htop <= v_ht; s_hbot <= v_hb; s_hcnt <= v_hc;
            s_lgy <= v_ly; s_lcy <= v_lcy; s_yedge <= v_ye;

            -- x bands off the registered counters (cur_x = s_xa off the line
            -- start; at the line start every x band is in its x = 0 state)
            v_xi  := s_xa(12 downto 1);
            v_hxi := s_hxa(15 downto 4);
            -- border band: x < 24 or x >= W-24
            if s_gav_start = '1' or v_xi < C_EDGE or v_xi >= s_edge_r then
                v_xe := '1';
            else
                v_xe := '0';
            end if;
            s0_edge <= v_xe or v_ye;
            -- title logo columns [W/2-288, +576): 9 glyphs of 64 units
            v_lcx := ('0' & v_xi) - ('0' & s_logo_x0);
            if s_gav_start = '0' and v_lcx(12) = '0' and v_lcx(11 downto 0) < 576 then
                v_lx := '1';
            else
                v_lx := '0';
            end if;

            if s_title = '1' and v_ly = '1' and v_lx = '1' then
                -- logo: 64 px glyphs from HUD RAM top-line columns 48..56
                s0_hud_active <= '1';
                s0_hud_top    <= '1';
                s0_logo       <= '1';
            else
                s0_hud_active <= v_ht or v_hb;
                s0_hud_top    <= v_ht;
                s0_logo       <= '0';
            end if;
            -- HP bar: band rows 8-23 of the bottom line, right of the text
            -- (HUD x): frame [x0, x0 + 3*maxhp), fill [x0, x0 + 3*hp)
            v_bx := '0'; v_bf := '0';
            if s_gav_start = '0' and v_hxi >= C_BAR_X0 and v_hxi < s_bar_mxx then
                v_bx := '1';
                if v_hxi < s_bar_hpx then v_bf := '1'; end if;
            end if;
            s0_barf <= '0'; s0_bare <= '0';
            if v_hb = '1' and (v_hc(4 downto 3) = "01" or v_hc(4 downto 3) = "10")
               and v_bx = '1' then
                if v_bf = '1' then s0_barf <= '1'; else s0_bare <= '1'; end if;
            end if;
            -- cell = 2**C_HUD_LOG2 units; 8x8 glyph scaled by 2**(C_HUD_LOG2-3).
            if s_title = '1' and v_ly = '1' and v_lx = '1' then
                s0_hud_col <= "11" & v_lcx(9 downto 6);     -- 48 + glyph
                s0_frow    <= v_lcy(5 downto 3);
                s0_fcol    <= v_lcx(5 downto 3);
            else
                s0_hud_col <= cur_hx(C_HUD_LOG2 + 5 downto C_HUD_LOG2);
                s0_frow <= v_hc(C_HUD_LOG2 - 1 downto C_HUD_LOG2 - 3);
                s0_fcol <= cur_hx(C_HUD_LOG2 - 1 downto C_HUD_LOG2 - 3);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: cell decode at the current zoom (bit slices picked by s_cl) for the
    -- world and player coordinate systems; in-player / reticle compares.
    --------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_col, v_row : unsigned(5 downto 0);
        variable v_gx, v_gy : unsigned(3 downto 0);
        variable v_ibx, v_iby : std_logic;
        variable v_c, v_b : std_logic;
        variable v_gp, v_gm : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            cell_dec(s0_wx,  s_cl, v_col, v_gx,  v_ibx);
            cell_dec(s0_wy,  s_cl, v_row, v_gy,  v_iby);

            -- glide variants world +/- f, per axis: in-cell offset +/- f; the
            -- carry / borrow moves the column one cell (f <= one cell).  An
            -- off-map coordinate aliases to row/column 0 or 63, the border wall,
            -- where no entity ever stands, so no in-bounds test is needed.
            glide_dec(s0_wx, s_cl, s_mgf, v_c, v_gp, v_b, v_gm);
            s1_colp <= v_col + ("00000" & v_c);  s1_gxp <= v_gp;
            s1_colm <= v_col - ("00000" & v_b);  s1_gxm <= v_gm;
            glide_dec(s0_wy, s_cl, s_mgf, v_c, v_gp, v_b, v_gm);
            s1_rowp <= v_row + ("00000" & v_c);  s1_gyp <= v_gp;
            s1_rowm <= v_row - ("00000" & v_b);  s1_gym <= v_gm;

            s1_inb <= v_ibx and v_iby;
            if s_show_tgt = '1' and v_ibx = '1' and v_iby = '1'
               and v_col = s_tgt_x and v_row = s_tgt_y then
                s1_intgt <= '1';
            else
                s1_intgt <= '0';
            end if;

            s1_addr <= v_row & v_col;
            s1_gx  <= v_gx;  s1_gy  <= v_gy;
            s1_render <= s0_render;
            s1_edge   <= s0_edge;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2a: pipeline carries (the tile value comes from the shared s_map_q,
    -- which p_mapport reads using s1_addr while rendering - same latency;
    -- the seen bit likewise from s_seen_q).
    --------------------------------------------------------------------------
    p_map : process(clk)
    begin
        if rising_edge(clk) then
            s2_gx <= s1_gx; s2_gy <= s1_gy;
            s2_gxp <= s1_gxp; s2_gxm <= s1_gxm; s2_gyp <= s1_gyp; s2_gym <= s1_gym;
            s2_inb <= s1_inb; s2_intgt <= s1_intgt; s2_render <= s1_render;
            s2_edge <= s1_edge;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2b: entity layer - the player, else the first active monster, whose
    -- glide-shifted cell holds this pixel.  An entity that stepped this turn
    -- is tested in world+f or world-f (per axis, by its step direction), so it
    -- is drawn pulled back toward its old cell by the glide distance; each
    -- entity just compares its cell against the one shared coordinate its
    -- direction picks.  Only the winner's direction codes go on: S3 picks the
    -- matching art pixel.  Carries mirror (facing), bob, combat-target flash;
    -- plus the cell's offset from the player for the torch.
    --------------------------------------------------------------------------
    p_ent : process(clk)
        variable v_col, v_row : unsigned(5 downto 0);
        variable v_hit, v_found, v_mir, v_bob, v_isp, v_tgt, v_hx, v_hy : std_logic;
        variable v_eid : integer range 0 to C_NSPR - 1;
        variable v_wdx, v_wdy : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            v_col := s1_addr(5 downto 0);
            v_row := s1_addr(2 * C_MAP_LOG2 - 1 downto C_MAP_LOG2);
            v_hit := '0'; v_found := '0'; v_eid := 0;
            v_mir := '0'; v_bob := '0'; v_isp := '0'; v_tgt := '0';
            v_wdx := "00"; v_wdy := "00";

            -- player (its direction = the camera's glide direction)
            case s_pdx is
                when "01"   => if s1_colp = s_px then v_hx := '1'; else v_hx := '0'; end if;
                when "11"   => if s1_colm = s_px then v_hx := '1'; else v_hx := '0'; end if;
                when others => if v_col   = s_px then v_hx := '1'; else v_hx := '0'; end if;
            end case;
            case s_pdy is
                when "01"   => if s1_rowp = s_py then v_hy := '1'; else v_hy := '0'; end if;
                when "11"   => if s1_rowm = s_py then v_hy := '1'; else v_hy := '0'; end if;
                when others => if v_row   = s_py then v_hy := '1'; else v_hy := '0'; end if;
            end case;

            if s1_inb = '1' and v_hx = '1' and v_hy = '1' then
                v_hit := '1';
                v_isp := '1';
                v_eid := to_integer(s_class);   -- 0..3
                v_wdx := s_pdx; v_wdy := s_pdy;
                v_mir := s_pmir;
                v_bob := s_pbob;
            elsif s1_inb = '1' then
                for i in 0 to C_MONS - 1 loop
                    case s_mdx(i) is
                        when "01"   => if s1_colp = s_mx(i) then v_hx := '1'; else v_hx := '0'; end if;
                        when "11"   => if s1_colm = s_mx(i) then v_hx := '1'; else v_hx := '0'; end if;
                        when others => if v_col   = s_mx(i) then v_hx := '1'; else v_hx := '0'; end if;
                    end case;
                    case s_mdy(i) is
                        when "01"   => if s1_rowp = s_my(i) then v_hy := '1'; else v_hy := '0'; end if;
                        when "11"   => if s1_rowm = s_my(i) then v_hy := '1'; else v_hy := '0'; end if;
                        when others => if v_row   = s_my(i) then v_hy := '1'; else v_hy := '0'; end if;
                    end case;
                    if v_found = '0' and s_mactive(i) = '1' and v_hx = '1' and v_hy = '1' then
                        v_hit := '1';
                        v_found := '1';
                        v_eid := C_MON_BASE + to_integer(s_mtype(i));
                        v_wdx := s_mdx(i); v_wdy := s_mdy(i);
                        v_mir := s_mface(i);
                        -- idle bob: every 32 fields, odd/even slots alternate
                        if i mod 2 = 0 then v_bob := s_fcnt(5);
                        else                v_bob := not s_fcnt(5); end if;
                        if i = s_ctarget then v_tgt := s_flash_on; end if;
                    end if;
                end loop;
            end if;

            s2_ehit <= v_hit;
            s2_eid  <= v_eid;
            s2_wdx  <= v_wdx;
            s2_wdy  <= v_wdy;
            s2_smir <= v_mir;
            s2_sbob <= v_bob;
            s2_isp  <= v_isp;
            s2_etgt <= v_tgt;
            -- torch: this cell's offset from the player's cell
            s2_dx <= signed(resize(v_col, 7)) - signed(resize(s_px, 7));
            s2_dy <= signed(resize(v_row, 7)) - signed(resize(s_py, 7));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- HUD read pipeline (parallel to the world pipeline, same depth):
    --   S1 glyph fetch  ->  S2 font ROM read  ->  S3/S4 carry to colour.
    --------------------------------------------------------------------------
    p_hud_pipe : process(clk)
        variable v_faddr : integer range 0 to C_FROM_DEPTH - 1;
    begin
        if rising_edge(clk) then
            -- S1: carries (the glyph itself is fetched by p_hudram in parallel).
            s1_hud_active <= s0_hud_active;
            s1_frow <= s0_frow; s1_fcol <= s0_fcol;
            s1_barf <= s0_barf; s1_bare <= s0_bare;
            s2_barf <= s1_barf; s2_bare <= s1_bare;
            s3_barf <= s2_barf; s3_bare <= s2_bare;
            s4_barf <= s3_barf; s4_bare <= s3_bare;
            s1_logo <= s0_logo; s2_logo <= s1_logo; s3_logo <= s2_logo; s4_logo <= s3_logo;

            -- S2: font ROM read (addr = glyph*8 + frow).
            v_faddr := to_integer(s1_glyph) * 8 + to_integer(s1_frow);
            s2_fontrow <= s_from(v_faddr);
            s2_hud_active <= s1_hud_active;
            s2_fcol <= s1_fcol;

            -- S3/S4: carry to align with the world pipeline colour stage.
            s3_fontrow <= s2_fontrow;
            s3_hud_active <= s2_hud_active;
            s3_fcol <= s2_fcol;
            s4_fontrow <= s3_fontrow;
            s4_hud_active <= s3_hud_active;
            s4_fcol <= s3_fcol;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3: entity sprite ROM read (addr = eid*16 + row, the row sunk by the
    -- bob; row 15 of every sprite is blank, so the wrap is invisible) + tile
    -- id + seen bit + |torch offset| saturated to 15.
    --------------------------------------------------------------------------
    p_spr : process(clk)
        variable v_saddr : integer range 0 to C_SPROM_DEPTH - 1;
        variable v_id : integer range 0 to 15;
        variable v_row, v_sgx, v_sgy : unsigned(3 downto 0);
        variable v_ax, v_ay : integer range 0 to 64;
    begin
        if rising_edge(clk) then
            -- the winning entity's art pixel, from its glide-shifted coordinate
            case s2_wdx is
                when "01"   => v_sgx := s2_gxp;
                when "11"   => v_sgx := s2_gxm;
                when others => v_sgx := s2_gx;
            end case;
            case s2_wdy is
                when "01"   => v_sgy := s2_gyp;
                when "11"   => v_sgy := s2_gym;
                when others => v_sgy := s2_gy;
            end case;
            if s2_sbob = '1' then v_row := v_sgy - 1; else v_row := v_sgy; end if;
            v_saddr := s2_eid * 16 + to_integer(v_row);
            s3_sprrow <= s_sprom(v_saddr);
            s3_ehit <= s2_ehit;
            s3_eid  <= s2_eid;
            s3_sgx  <= v_sgx;
            s3_smir <= s2_smir;
            s3_isp  <= s2_isp;
            s3_etgt <= s2_etgt;

            v_id := to_integer(unsigned(s_map_q));
            if v_id >= C_NTILES then v_id := 0; end if;
            s3_tile <= v_id;
            s3_seen <= s_seen_q;

            v_ax := to_integer(abs(s2_dx));
            v_ay := to_integer(abs(s2_dy));
            if v_ax > 15 or v_ay > 15 then s3_far <= '1'; else s3_far <= '0'; end if;
            if v_ax > 15 then s3_adx <= "1111"; else s3_adx <= to_unsigned(v_ax, 4); end if;
            if v_ay > 15 then s3_ady <= "1111"; else s3_ady <= to_unsigned(v_ay, 4); end if;

            s3_gx <= s2_gx; s3_gy <= s2_gy;
            s3_inb <= s2_inb; s3_intgt <= s2_intgt; s3_render <= s2_render;
            s3_edge <= s2_edge;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S4: tile ROM read (addr = tile*16 + gy) + torch classification:
    -- core (d^2 <= (R-1)^2, fully lit) or ring (<= R^2, dithered).
    --------------------------------------------------------------------------
    p_trow : process(clk)
        variable v_addr : integer range 0 to C_TROM_DEPTH - 1;
        variable v_d2 : integer range 0 to 450;
    begin
        if rising_edge(clk) then
            v_addr := s3_tile * 16 + to_integer(s3_gy);
            s4_row    <= s_trom(v_addr);
            s4_sprrow <= s3_sprrow;
            s4_ehit   <= s3_ehit;
            s4_eid    <= s3_eid;
            s4_tile   <= s3_tile;
            s4_gx     <= s3_gx;
            s4_gy     <= s3_gy;
            s4_sgx    <= s3_sgx;
            s4_smir   <= s3_smir;
            s4_isp    <= s3_isp;
            s4_etgt   <= s3_etgt;
            s4_seen   <= s3_seen;
            s4_inb    <= s3_inb; s4_intgt <= s3_intgt; s4_render <= s3_render;
            -- reticle rim: within s_rx / s_ry art pixels of the cell edge
            -- (15 - g = not g)
            if s3_gx < s_rx or (not s3_gx) < s_rx or
               s3_gy < s_ry or (not s3_gy) < s_ry then
                s4_rim <= '1';
            else
                s4_rim <= '0';
            end if;
            s4_edge   <= s3_edge;

            v_d2 := C_SQ(to_integer(s3_adx)) + C_SQ(to_integer(s3_ady));
            s4_core <= '0';
            s4_ring <= '0';
            if s_torch_full = '1' then
                s4_core <= '1';
            elsif s3_far = '0' then
                if v_d2 <= s_ri2 then
                    s4_core <= '1';
                elsif v_d2 <= s_r2 then
                    s4_ring <= '1';
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5 (select): bit-extract all layers + decide which one wins.  Lit =
    -- torch core, or the ring on a checkerboard of art pixels; remembered
    -- (seen, unlit) cells draw dimmed (S7); unexplored is black.  Monsters
    -- show only on lit cells.
    --------------------------------------------------------------------------
    p_sel : process(clk)
        variable v_tbit, v_sbit, v_text, v_ring, v_lit, v_vis : std_logic;
        variable p, sp : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            p  := to_integer(s4_gx);
            sp := to_integer(s4_sgx);
            v_tbit := s4_row(15 - p);
            if s4_smir = '1' then v_sbit := s4_sprrow(sp);
            else                  v_sbit := s4_sprrow(15 - sp); end if;
            v_text := s4_fontrow(7 - to_integer(s4_fcol));
            -- reticle ring = the target cell's outline (1 art-pixel border,
            -- wider where a fractional step would skip it: s4_rim)
            if s4_intgt = '1' and s4_rim = '1' then
                v_ring := '1';
            else
                v_ring := '0';
            end if;
            v_lit := s4_core or (s4_ring and (s4_gx(0) xor s4_gy(0)));
            v_vis := s4_core or s4_ring;

            s5_tint <= '0';
            -- blanking gate: outside the (generated) avid the output is
            -- neutral black 64/512/512, never a held tile/HUD colour.
            -- Palette address: 00t tile fg, 01t tile bg, 10e sprite, 11k fixed;
            -- bit 6 = dimmed (remembered under the fog).
            if s4_render = '0' then
                s5_pa <= to_unsigned(C_PA_BLANK, 7);
            elsif s4_hud_active = '1' then
                if v_text = '1' then     s5_pa <= to_unsigned(C_PA_INK, 7);
                elsif s4_logo = '1' then s5_pa <= to_unsigned(C_PA_VOID, 7);
                elsif s4_barf = '1' then s5_pa <= "01111" & to_unsigned(s_bar_col, 2);
                elsif s4_bare = '1' then s5_pa <= to_unsigned(C_PA_HPBG, 7);
                else                     s5_pa <= to_unsigned(C_PA_HUDBAR, 7); end if;
            elsif s_gensp = '1' then
                s5_pa <= to_unsigned(C_PA_VOID, 7);   -- generation splash hides the map
            elsif s_title = '1' and not (s4_isp = '1' and s4_ehit = '1' and v_sbit = '1') then
                s5_pa <= to_unsigned(C_PA_VOID, 7);   -- title: only the (big) class sprite
            elsif s4_edge = '1' and s_bord_hurt = '1' then
                s5_pa <= to_unsigned(C_PA_HURT, 7);
            elsif s4_edge = '1' and s_bord_low = '1' then
                s5_pa <= to_unsigned(C_PA_LOWHP, 7);
            elsif s4_inb = '0' then
                s5_pa <= to_unsigned(C_PA_VOID, 7);
            elsif s4_ehit = '1' and v_sbit = '1' and (s4_isp = '1' or v_vis = '1') then
                if s4_etgt = '1' then s5_pa <= to_unsigned(C_PA_FLASH, 7);
                else                  s5_pa <= "010" & to_unsigned(s4_eid, 4); end if;
            elsif v_ring = '1' then
                s5_pa <= to_unsigned(C_PA_TGT, 7);
            elsif v_lit = '1' or s4_seen = '1' then
                s5_pa <= (not v_lit) & '0' & (not v_tbit) & to_unsigned(s4_tile, 4);
                s5_tint <= '1';
            else
                s5_pa <= to_unsigned(C_PA_VOID, 7);   -- unexplored
            end if;

            s5_av   <= s4_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S6: palette ROM read (EBR, registered).  Dimmed tile colours are
    -- precomputed in the ROM (oubliette.py write_palette).
    --------------------------------------------------------------------------
    p_color : process(clk)
    begin
        if rising_edge(clk) then
            s6_pal  <= s_pal(to_integer(s5_pa));
            s6_av   <= s5_av;
            s6_tint <= s5_tint;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S7: fabric copy of the ROM word (no arithmetic straight off an EBR).
    --------------------------------------------------------------------------
    p_dim : process(clk)
    begin
        if rising_edge(clk) then
            s7_y    <= unsigned(s6_pal(29 downto 20));
            s7_u    <= unsigned(s6_pal(19 downto 10));
            s7_v    <= unsigned(s6_pal(9 downto 0));
            s7_av   <= s6_av;
            s7_tint <= s6_tint;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S8: the tiles' per-depth tint (Colour theme; small offsets, stays in
    -- range) and the K6 palette theme: luma quantised to 4 levels (bit
    -- slices), each mapped to the theme's 4-colour table.  Active video only,
    -- so blanking stays exactly neutral.
    --------------------------------------------------------------------------
    p_theme : process(clk)
        variable v_lv : unsigned(1 downto 0);
        variable v_ix : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            case s7_y(9 downto 7) is
                when "000" | "001" => v_lv := "00";
                when "010"         => v_lv := "01";
                when "011"         => v_lv := "10";
                when others        => v_lv := "11";
            end case;
            -- a 4-bit index (theme-1 & level): each output bit is one LUT4
            v_ix := to_integer(s_theme_m1 & v_lv);
            if s7_av = '1' and s_theme /= 0 then
                s8_y <= to_unsigned(C_TH_Y(v_ix), 10);
                s8_u <= to_unsigned(C_TH_U(v_ix), 10);
                s8_v <= to_unsigned(C_TH_V(v_ix), 10);
            elsif s7_tint = '1' then
                s8_y <= s7_y;
                s8_u <= unsigned(signed(s7_u) + resize(s_tint_u, 10));
                s8_v <= unsigned(signed(s7_v) + resize(s_tint_v, 10));
            else
                s8_y <= s7_y;
                s8_u <= s7_u;
                s8_v <= s7_v;
            end if;
            s8_av   <= s7_av;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fog of war RAM: render read at the tilemap's address/stage; one write
    -- port fed by the level clear (mirrors the generator's GN_CLEAR sweep with
    -- '0') or the reveal sweep ('1').  Writes happen in vertical blank only.
    --------------------------------------------------------------------------
    s_seen_rd <= s_seen(to_integer(s1_addr));
    p_seenram : process(clk)
    begin
        if rising_edge(clk) then
            if s_sv_we = '1' then
                s_seen(to_integer(s_sv_addr)) <= s_sv_data;
            end if;
            s_seen_q <= s_seen_rd;
        end if;
    end process;

    -- Reveal sweep: once per field, 300 clocks after the tick (the turn has
    -- settled) and not while a level generates, mark every cell within the
    -- torch radius of the player as seen.  Two pipeline stages per cell:
    -- A = target cell + |offset|, B = range / circle test -> write.
    p_reveal : process(clk)
    begin
        if rising_edge(clk) then
            s_sv_we <= '0';
            if s_game = G_GEN and s_gst = GN_CLEAR then
                s_sv_we   <= '1';
                s_sv_addr <= s_gaddr;
                s_sv_data <= '0';
            elsif s_rva_v = '1' then
                if s_rva_tx >= 0 and s_rva_tx <= C_MAP - 1 and
                   s_rva_ty >= 0 and s_rva_ty <= C_MAP - 1 and
                   C_SQ(s_rva_adx) + C_SQ(s_rva_ady) <= s_r2 then
                    s_sv_we   <= '1';
                    s_sv_addr <= to_unsigned(s_rva_ty, 6) & to_unsigned(s_rva_tx, 6);
                    s_sv_data <= '1';
                end if;
            end if;

            s_rva_v <= '0';
            case s_rv_state is
                when RV_IDLE =>
                    if s_ftick = '1' then
                        s_rv_state <= RV_DLY;
                    end if;
                when RV_DLY =>
                    -- s_ecnt restarted on the same tick: 301 clocks
                    if s_ecnt = 300 then
                        if s_game = G_GEN then
                            s_rv_state <= RV_IDLE;
                        else
                            s_rv_rr <= s_rv_r;
                            s_rv_dx <= -s_rv_r;
                            s_rv_dy <= -s_rv_r;
                            s_rv_state <= RV_RUN;
                        end if;
                    end if;
                when RV_RUN =>
                    s_rva_v   <= '1';
                    s_rva_tx  <= to_integer(s_px) + s_rv_dx;
                    s_rva_ty  <= to_integer(s_py) + s_rv_dy;
                    if s_rv_dx < 0 then s_rva_adx <= -s_rv_dx; else s_rva_adx <= s_rv_dx; end if;
                    if s_rv_dy < 0 then s_rva_ady <= -s_rv_dy; else s_rva_ady <= s_rv_dy; end if;
                    if s_rv_dx = s_rv_rr then
                        s_rv_dx <= -s_rv_rr;
                        if s_rv_dy = s_rv_rr then
                            s_rv_state <= RV_IDLE;
                        else
                            s_rv_dy <= s_rv_dy + 1;
                        end if;
                    else
                        s_rv_dx <= s_rv_dx + 1;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output registers.
    --------------------------------------------------------------------------
    s_odl_rd <= s_odl(to_integer(s_odl_ra));
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.hsync_n <= s_sdl_q(0);
            s_io.vsync_n <= s_sdl_q(1);
            s_io.field_n <= s_sdl_q(2);
            -- y/u/v: s8 -> data_out 3 clocks later (old s_io + IO pad)
            s_odl(to_integer(s_odl_wa)) <= std_logic_vector(s8_y) & std_logic_vector(s8_u)
                                           & std_logic_vector(s8_v) & "00";
            s_odl_q  <= s_odl_rd;
            s_odl_wa <= s_odl_wa + 1;
            s_odl_ra <= s_odl_ra + 1;
            s_io.avid    <= s8_av;            -- generated avid, pixel-aligned
        end if;
    end process;

    s_io_align_in.y       <= (others => '0');   -- y/u/v ride s_odl
    s_io_align_in.u       <= (others => '0');
    s_io_align_in.v       <= (others => '0');
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.field_n <= s_io.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (10 -> 12).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 1 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out.hsync_n <= s_io_align_pad(1).hsync_n;
    data_out.vsync_n <= s_io_align_pad(1).vsync_n;
    data_out.field_n <= s_io_align_pad(1).field_n;
    data_out.avid    <= s_io_align_pad(1).avid;
    data_out.y       <= s_odl_q(31 downto 22);
    data_out.u       <= s_odl_q(21 downto 12);
    data_out.v       <= s_odl_q(11 downto 2);

end architecture oubliette;
