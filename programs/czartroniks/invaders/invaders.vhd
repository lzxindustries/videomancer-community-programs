-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Wizard Invaders (v0.7.7)
--
-- v0.7.7 (2026-10-18): fit for the stock SDK's SD Standalone core again,
-- bit-exact with v0.7.6 (formally proven equivalent).  Compares against a
-- constant are plain bit logic (f_ult / f_slt) instead of carry chains --
-- above all the edge-shadow's 64 group thermometers -- and the 3-operand
-- position adds use bit fields.  Nothing drawn changes.
--
-- v0.7.6 (2026-10-17): resolution scaling.  The whole game (render
-- coordinates, measured width/height, layout, sprite sizes, speeds, the
-- wizard's zoom DDA) runs in virtual 1080-line units, stepped per pixel /
-- per field line from the video timing ID (registers_in(8)): 1.0 at 1080p
-- (identical to before), 1.5 at 720p, and at SD 2.0 across (a 1440-wide 4:3
-- window) and 1080/486 per frame line.  So 720p is the 1080p game at 2/3
-- scale, and SD is the same game in a 4:3 window: sprites, formation,
-- bunkers, shots, death TV and trail keep their size relative to the screen
-- height and stay square.  Interlaced fields step two frame lines and the
-- bottom field starts half a step down.  The 4x4 Bayer dissolve stays at
-- real-pixel scale; the floor line is 5 units tall at 480i so it can't fall
-- between field lines.  Before: fixed pixel sizes on a per-field measured
-- height (SD: formation wider than the screen, shields clamped to the top,
-- sprites 2x tall), and 720p lumped with 1080p for speeds.
--
-- Videomancer-themed Space Invaders clone using the wizard_invaders_sprites
-- package (WIZARD player, CRT_TV alien with 2-frame walk cycle, VIDEOMANCER
-- bunker, EXPLOSION burst).
--
-- Gameplay
--   * 3 rows x 6 cols of CRT-TV aliens (64x64 art drawn 2x = 128x128 cells);
--     each TV screen plays a 5-frame sawtooth / lightning waveform loop at
--     its own random offset.  Aliens keep their colour in every band and
--     crunch the shields as they descend (pieces fall away like bomb hits)
--   * 4 destructible bunkers (128x112 sprite, 4x4 grid of 32x32 chunks)
--   * Wizard ship (128x128) along the bottom, slid by Fader 12; mirrors
--     to face the direction of travel.  Fixed colours (purple body, white
--     eyeball — not affected by K3 / cellophane), rainbow edge-shadow on its
--     trailing side while he moves (length = K2 x speed, gone when he
--     stops), allowed to run past the sprite box and also filling the
--     outside pockets between body parts (under the arm etc.)
--   * Up to 4 wand-shots in flight — fires on each Switch 7 edge
--   * Aliens drop up to 4 bombs (K1 Bomb Rate); bombs (and shots) knock
--     bunker chunks out, and each piece falls off the bottom of the screen
--   * A bomb hit plays a ~2.4 s death: the aliens and shields dissolve away
--     as the wizard flies to the screen centre growing to ~3.2x and a giant
--     TV dissolves in around him (he fills its screen); he rumbles (building
--     shake + rainbow strobe) and blows up in an 8x rainbow cloud inside the
--     TV; halfway through the blast the TV crossfades back to the aliens and
--     shields, then he blinks back in at the bottom — no lives; only the
--     aliens landing
--     resets the game (after the same death) — when the lowest live alien
--     row reaches the bottom of the shields
--   * Clearing the wave: rainbow puff, the wizard grows to ~6.4x and flies
--     up and away trailing his rainbow shadow, then the game resets
--   * S9 Cellophane = the arcade colour overlay (red top band, bottom band
--     green); K3 Hue rotates the sprite colour / overlay palette
--   * Switch 11 force-resets the wave
--
-- Designed for HD (1920x1080 at the full 74.25 MHz clock); every other mode
-- draws the same game in virtual 1080-line units (see v0.7.6).  Layout
-- scales from the auto-measured (virtual) raster.
--
-- Collisions are detected PIXEL-ACCURATELY in the render pipeline (like the
-- arcade's hardware sprite-collision registers): wherever a lit bullet pixel
-- meets a lit alien / bunker pixel, or a bomb meets a lit wizard pixel, the
-- first such hit of the frame is latched with its object indices.  The FSM
-- commits the latched hits at vsync.  An unresolved overlap re-latches next
-- frame, so one latch per collision type is enough.
--
-- Render pipeline (per pixel):
--   1 clk : video_timing_generator
--   1 clk : pixel_counter
--   1 clk : S1 — register signed coords
--   1 clk : S2 — per-region deltas (live positions read directly: game
--                state changes only during vblank)
--   1 clk : S3 — in-bbox flags + alien_idx + ROM addresses
--   1 clk : S4 — sprite ROM word reads (16-px words) + state lookups + band
--   1 clk : S5 — 16:1 bit selects + per-region pixel combine + band colour
--   1 clk : S6 — color mux (+ collision latches, off the output path)
--   2 clk : IO alignment (blanking gate in front of the first IO register)
--   total : 8 render + 2 IO = 10 clocks
--
-- The wizard is drawn by a sprite-space DDA (p_zoom/p_dda: u = u_start +
-- h*k), which gives the win flight's zoom without per-pixel multiplies.
--
-- Physics FSM (vsync-triggered):
--   Phase 0 : vsync edge — latch reset / run / fire flags only
--   Phase 1 : reset, or tick step counter + animation + bomb cooldown
--             (collision snapshot lands)
--   Phase 2 : commit bullet hits (alien kill + explosion, bunker chunk)
--   Phase 3 : commit bomb hits (bunker chunk, wizard = game over)
--   Phase 4 : projectile advance; wand-shot spawn; bomb spawn; form predict
--   Phase 5 : formation commit (+ drop on bounce)
--   Phase 6 : aliens-clear respawn

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.wizard_invaders_sprites.all;

architecture invaders of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (10 + 2 = 12 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    -- ========================================================================
    -- Constants
    -- ========================================================================
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DELAY      : integer := 8;

    -- Alien formation: 3 rows × 6 cols of 64×64 CRT_TV sprites drawn at
    -- 2× nearest-neighbour scale = 128×128 on screen.  Cells are a
    -- power-of-two 128 × 128 so the row/col a pixel falls in is a plain
    -- bit-slice (no comparator cascade).  The sprite fills the cell, so
    -- aliens sit edge-to-edge — the CRT_TV art's own transparent margin
    -- keeps a small visual gap.
    --
    -- The full 768 × 384 grid is taller than the gap above the bunker,
    -- so the formation starts with its top row(s) off-screen above y=0
    -- and marches into view as it drops after each bounce — the classic
    -- Space Invaders entry pattern.
    constant C_COLS         : integer := 6;
    constant C_ROWS         : integer := 3;
    -- Cells are a power of two (128) so the column/row a pixel falls in is a
    -- plain bit-slice of form_dx/dy — no 12-bit comparator cascade or
    -- priority encoder.  The 128-wide sprite fills the cell (no padding), so
    -- aliens sit edge-to-edge; the CRT_TV art's own transparent margin keeps
    -- a small visual gap.
    constant C_CELL_W       : integer := 128;
    constant C_CELL_H       : integer := 128;
    constant C_FORM_W       : integer := C_COLS * C_CELL_W;             -- 768
    constant C_FORM_H       : integer := C_ROWS * C_CELL_H;             -- 384
    constant C_FORM_X_INIT  : integer := 96;                  -- (960-768)/2
    -- Formation starts with its top row off-screen and marches in as it
    -- drops after each bounce — the classic entry pattern.  p_layout
    -- recomputes the live init from the measured resolution.
    constant C_FORM_Y_INIT  : integer := -128;
    constant C_SPRITE_W     : integer := 128;
    constant C_SPRITE_H     : integer := 128;
    constant C_SPRITE_OFF   : integer := 0;   -- sprite fills the 128 cell
    -- alien_idx = {row(1..0), col(2..0)} with col stride 8.  Slots 6, 7,
    -- 14, 15, 22..31 unused, but the 5-bit packing is cheaper than
    -- multiplying by 6.
    constant C_ALIEN_IDX_W  : integer := 5;
    constant C_ALIEN_SLOTS  : integer := 2 ** C_ALIEN_IDX_W;  -- 32 (18 used)

    -- Player ship + projectiles
    constant C_SHIP_W       : integer := 128;
    constant C_SHIP_H       : integer := 128;
    constant C_BULLET_W     : integer := 8;
    constant C_BULLET_H     : integer := 16;
    constant C_BOMB_W       : integer := 6;
    constant C_BOMB_H       : integer := 14;
    -- Up to 4 wand-shots and 4 bombs in flight (5 shots pushed v0.5 to 91%
    -- LC and flaky HD timing; 4 was chosen).  (The render-time collision
    -- rework freed the serial hit-scan logic).
    constant C_BULLETS      : integer := 4;
    constant C_BOMBS        : integer := 4;
    -- Wand offset inside the wizard sprite (upper-right of the 128x128
    -- bitmap; mirrored to upper-left when wizard faces left)
    constant C_WAND_OFFSET_X : integer := 88;
    constant C_WAND_OFFSET_Y : integer := 32;

    -- Bunkers.  Chunks are 32 × 32 (16 per bunker, was 32) — even coarser
    -- destruction to free more LCs for the dynamic-init logic.
    constant C_BUNKER_W       : integer := 128;
    constant C_BUNKER_H       : integer := 112;
    constant C_BUNKER_COUNT   : integer := 4;
    constant C_CHUNK_W        : integer := 32;
    constant C_CHUNK_H        : integer := 32;
    constant C_CHUNK_COLS     : integer := C_BUNKER_W / C_CHUNK_W;   -- 4
    constant C_CHUNK_ROWS_PWR : integer := 4;
    constant C_CHUNKS_PER_BNK : integer := C_CHUNK_COLS * C_CHUNK_ROWS_PWR;
                                                -- 16
    -- The bunker is 112 tall = 3.5 chunk rows, so one chunk row is a
    -- half-height (16px) partial.  Offsetting the chunk-row split by this puts
    -- the partial row at the TOP (by the aliens) and makes the BOTTOM row the
    -- wizard shoots into a full 32px — a fast shot dwells in a thin 16px row
    -- too briefly for the serial hit-scan to reliably catch it.
    constant C_CHUNK_ROW_OFF  : integer := C_CHUNK_H
                                           - (C_BUNKER_H mod C_CHUNK_H);  -- 16

    -- March step per alien tick, x2 (= 12 virtual units; it used to be
    -- doubled on HD rasters only, and every raster is virtual-HD now).
    constant C_FORM_STEP_X  : integer := 6;
    constant C_BORDER       : integer := 8;
    -- Vertical gap (px) between bunker bottom and wizard top.  Bumped from
    -- 10 → 26 (= 10 + one chunk-row) so the bunkers sit one chunk higher.
    constant C_BUNKER_GAP   : integer := 26;

    -- ------------------------------------------------------------------
    -- Rainbow edge-ghost: a rainbow band that hugs the wizard's TRAILING edge,
    -- built from the silhouette's own edge pixels (not a full-sprite copy).
    -- Per pixel we sample the wizard ROM row s_fringe_step px toward the body;
    -- where that is set but the live pixel is off, we draw the band.  Reuses
    -- the live wizard ROM row (no history buffer) — one tap only, since each
    -- extra variable index into the 128-bit row is routing-heavy.
    --
    -- The band width (= the tap distance) is DYNAMIC: it tracks the wizard's
    -- per-frame speed: the trail only shows while the wizard MOVES.  Target
    -- = K2 max x min(speed - 1, 16) / 16 (speed = |dx| px/frame, averaged
    -- over 2 frames; 1 px deadband), so a slow move gives a short trail in
    -- proportion to K2 and >= 17 px/frame (~1000 px/s on HD) reaches the
    -- full K2 length.  It eases in at C_FRINGE_RISE and out at
    -- C_FRINGE_FALL px/frame, and at 0 it is hidden entirely.
    constant C_FRINGE_RISE  : integer := 6;   -- grow rate (px/frame)
    -- The shadow may extend this far beyond the 128-px sprite box (max K2
    -- length is 127 px), so long trails follow the silhouette instead of
    -- being cut flat at the box edge.
    constant C_TRAIL_EXT    : integer := 128;
    constant C_FRINGE_FALL  : integer := 4;   -- shrink rate (px/frame)
    -- Dim luma for the ghost silhouette so it reads as a faded afterimage
    -- rather than a second solid wizard.
    constant C_GHOST_LUMA  : integer := 470;

    -- ROYGBIV phosphor chroma palette, 10-bit U/V in the Videomancer
    -- U/V-swapped convention (hardware-confirmed; see Phosphor program).
    -- Looked up once per ghost at drop time (off the hot path) so the render
    -- path carries plain chroma.  Index 7 duplicates red as an out-of-range
    -- guard for the 3-bit colour counter.
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_TRAIL_PAL_U : t_pal := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10),
        to_unsigned(136, 10), to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(960, 10));
    constant C_TRAIL_PAL_V : t_pal := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10),
        to_unsigned(216, 10), to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(360, 10));

    -- Sprite colour wheel for K3 Hue: entry 0 = white, 1..15 = hues 24 deg
    -- apart starting at red (HSV s=0.85 → BT.601, stored U/V-swapped like the
    -- phosphor palette; luma capped at 768 and floored at 360 so every hue
    -- reads on black).  Cellophane bands index it at h+1 (top, red at h=0),
    -- h (middle, white at h=0) and h+6 (bottom, green at h=0).
    type t_hue_lut is array (0 to 15) of unsigned(9 downto 0);
    constant C_HUE_Y : t_hue_lut := (
        to_unsigned(768, 10), to_unsigned(418, 10), to_unsigned(593, 10),
        to_unsigned(768, 10), to_unsigned(768, 10), to_unsigned(722, 10),
        to_unsigned(632, 10), to_unsigned(666, 10), to_unsigned(700, 10),
        to_unsigned(630, 10), to_unsigned(455, 10), to_unsigned(360, 10),
        to_unsigned(369, 10), to_unsigned(458, 10), to_unsigned(486, 10),
        to_unsigned(452, 10));
    constant C_HUE_U : t_hue_lut := (
        to_unsigned(512, 10), to_unsigned(893, 10), to_unsigned(765, 10),
        to_unsigned(638, 10), to_unsigned(498, 10), to_unsigned(345, 10),
        to_unsigned(193, 10), to_unsigned(168, 10), to_unsigned(144, 10),
        to_unsigned(195, 10), to_unsigned(323, 10), to_unsigned(450, 10),
        to_unsigned(602, 10), to_unsigned(755, 10), to_unsigned(843, 10),
        to_unsigned(868, 10));
    constant C_HUE_V : t_hue_lut := (
        to_unsigned(512, 10), to_unsigned(383, 10), to_unsigned(283, 10),
        to_unsigned(182, 10), to_unsigned(157, 10), to_unsigned(208, 10),
        to_unsigned(260, 10), to_unsigned(412, 10), to_unsigned(564, 10),
        to_unsigned(691, 10), to_unsigned(792, 10), to_unsigned(893, 10),
        to_unsigned(841, 10), to_unsigned(790, 10), to_unsigned(688, 10),
        to_unsigned(536, 10));
    constant C_BLACK_Y : integer := 64;

    -- ------------------------------------------------------------------
    -- Sprite ROMs, stored as 16-PIXEL WORDS: address = {row, word}, and S5
    -- picks one bit of the registered 16-bit word (a 16:1 mux) instead of
    -- a 128:1 / 64:1 / 48:1 mux over a whole registered row.  Word w of a
    -- row holds sprite columns 16w .. 16w+15, column 16w in bit 15.  So for
    -- a sprite column c: word = c(hi bits), bit = not c(3 downto 0).
    -- ------------------------------------------------------------------
    type t_word_rom_1k  is array (0 to 1023) of std_logic_vector(15 downto 0);
    type t_word_rom_512 is array (0 to 511)  of std_logic_vector(15 downto 0);

    -- Wizard (128×128): address {row(6..0), word(2..0)}.
    function build_wizard_rom return t_word_rom_1k is
        variable v : t_word_rom_1k;
    begin
        for r in 0 to 127 loop
            for w in 0 to 7 loop
                v(r * 8 + w) := WIZARD_SPRITE(r)(127 - 16 * w downto 112 - 16 * w);
            end loop;
        end loop;
        return v;
    end function;
    constant C_WIZARD_ROM : t_word_rom_1k := build_wizard_rom;

    -- Wizard eyeball + trail-gap mask, same layout.  On a LIT pixel the bit
    -- means eyeball (drawn white; the rest of the wizard is purple); on an
    -- UNLIT pixel it means trail gap (an outside pocket between body parts,
    -- which the edge-shadow fills).  The two masks are disjoint.
    function build_eye_rom return t_word_rom_1k is
        variable v : t_word_rom_1k;
    begin
        for r in 0 to 127 loop
            for w in 0 to 7 loop
                v(r * 8 + w) :=
                    WIZARD_EYE_MASK(r)(127 - 16 * w downto 112 - 16 * w) or
                    WIZARD_GAP_MASK(r)(127 - 16 * w downto 112 - 16 * w);
            end loop;
        end loop;
        return v;
    end function;
    constant C_EYE_ROM : t_word_rom_1k := build_eye_rom;

    -- Wizard silhouette in 4-px groups, for the edge-shadow prefix-OR:
    -- bit g = OR of WIZARD_SPRITE row bits 4g+3 .. 4g (row-vector indexing,
    -- bit 127 = leftmost pixel).  One 32-bit word per row.
    type t_grp_rom is array (0 to 127) of std_logic_vector(31 downto 0);
    function build_grp_rom return t_grp_rom is
        variable v : t_grp_rom;
    begin
        for r in 0 to 127 loop
            for g in 0 to 31 loop
                v(r)(g) := WIZARD_SPRITE(r)(4 * g) or WIZARD_SPRITE(r)(4 * g + 1)
                        or WIZARD_SPRITE(r)(4 * g + 2) or WIZARD_SPRITE(r)(4 * g + 3);
            end loop;
        end loop;
        return v;
    end function;
    constant C_GRP_ROM : t_grp_rom := build_grp_rom;

    -- Bunker (128×112): address {row(6..0), word(2..0)}; rows 112-127 are
    -- blank so the 7-bit row needs no range guard.
    function build_bunker_rom return t_word_rom_1k is
        variable v : t_word_rom_1k := (others => (others => '0'));
    begin
        for r in 0 to 111 loop
            for w in 0 to 7 loop
                v(r * 8 + w) := VIDEOMANCER_BUNKER(r)(127 - 16 * w downto 112 - 16 * w);
            end loop;
        end loop;
        return v;
    end function;
    constant C_BUNKER_ROM : t_word_rom_1k := build_bunker_rom;

    -- Alien = CRT_TV_F0/F1 (64×64): address {frame, row(5..0), word(1..0)}.
    function build_alien_rom return t_word_rom_512 is
        variable v : t_word_rom_512;
    begin
        for r in 0 to 63 loop
            for w in 0 to 3 loop
                v(r * 4 + w)       := CRT_TV_F0(r)(63 - 16 * w downto 48 - 16 * w);
                v(256 + r * 4 + w) := CRT_TV_F1(r)(63 - 16 * w downto 48 - 16 * w);
            end loop;
        end loop;
        return v;
    end function;
    constant C_ALIEN_ROM : t_word_rom_512 := build_alien_rom;

    -- Alien TV screens: the 32×30 screen interior (source rows 18-49, cols
    -- 12-41 of CRT_TV) is drawn from a 5-frame waveform animation instead of
    -- the body frames' "+" / bars.  Address {frame(2..0), screen_row(4..0),
    -- word}; word 1 holds cols 16-29 plus two blank pixels; frames 5-7 blank.
    constant C_SCR_R0 : integer := 18;
    constant C_SCR_R1 : integer := 49;
    constant C_SCR_C0 : integer := 12;
    constant C_SCR_C1 : integer := 41;
    constant C_SCR_FRAMES     : integer := 5;
    constant C_SCR_FRAME_TIME : integer := 6;     -- vsyncs per screen frame
    function build_screen_rom return t_word_rom_512 is
        variable v : t_word_rom_512 := (others => (others => '0'));
    begin
        for f in 0 to C_SCR_FRAMES - 1 loop
            for r in 0 to 31 loop
                v(f * 64 + r * 2)     := CRT_SCREEN_ANIM(f * 32 + r)(29 downto 14);
                v(f * 64 + r * 2 + 1) := CRT_SCREEN_ANIM(f * 32 + r)(13 downto 0)
                                         & "00";
            end loop;
        end loop;
        return v;
    end function;
    constant C_SCREEN_ROM : t_word_rom_512 := build_screen_rom;
    -- Per-alien starting frame (alien_idx = row*8 + col), random but with no
    -- two touching aliens (incl. diagonals) on the same frame.
    type t_scr_off is array (0 to 31) of unsigned(2 downto 0);
    constant C_SCR_OFF : t_scr_off := (
        to_unsigned(1, 3), to_unsigned(3, 3), to_unsigned(4, 3), to_unsigned(1, 3), to_unsigned(3, 3), to_unsigned(1, 3), to_unsigned(0, 3), to_unsigned(0, 3),
        to_unsigned(2, 3), to_unsigned(0, 3), to_unsigned(2, 3), to_unsigned(0, 3), to_unsigned(4, 3), to_unsigned(2, 3), to_unsigned(0, 3), to_unsigned(0, 3),
        to_unsigned(4, 3), to_unsigned(3, 3), to_unsigned(4, 3), to_unsigned(3, 3), to_unsigned(1, 3), to_unsigned(3, 3), to_unsigned(0, 3), to_unsigned(0, 3),
        to_unsigned(0, 3), to_unsigned(0, 3), to_unsigned(0, 3), to_unsigned(0, 3), to_unsigned(0, 3), to_unsigned(0, 3), to_unsigned(0, 3), to_unsigned(0, 3)
    );

    -- Explosion = EXPLOSION_F0..F3 (48×48): address {frame(1..0), row(5..0),
    -- word(1..0)}; word 3 and rows 48-63 blank.
    constant C_EXPL_W : integer := 48;
    function build_expl_rom return t_word_rom_1k is
        variable v : t_word_rom_1k := (others => (others => '0'));
    begin
        for r in 0 to 47 loop
            for w in 0 to 2 loop
                v(  0 + r * 4 + w) := EXPLOSION_F0(r)(47 - 16 * w downto 32 - 16 * w);
                v(256 + r * 4 + w) := EXPLOSION_F1(r)(47 - 16 * w downto 32 - 16 * w);
                v(512 + r * 4 + w) := EXPLOSION_F2(r)(47 - 16 * w downto 32 - 16 * w);
                v(768 + r * 4 + w) := EXPLOSION_F3(r)(47 - 16 * w downto 32 - 16 * w);
            end loop;
        end loop;
        return v;
    end function;
    constant C_EXPL_ROM : t_word_rom_1k := build_expl_rom;

    -- Falling bunker pieces: a second bunker-art ROM laid out PER CHUNK so a
    -- 32×32 debris sprite needs no address arithmetic: address = {chunk(3..0)
    -- = {chunk_row, chunk_col}, piece_row(4..0), word}.  Chunk row r covers
    -- bunker art rows r*32-16 .. r*32+15 (the half-height row 0 is offset,
    -- matching the hit/render chunk split); rows outside the art are blank.
    function build_debris_rom return t_word_rom_1k is
        variable v  : t_word_rom_1k := (others => (others => '0'));
        variable ar : integer;
        variable ac : integer;
    begin
        for chk in 0 to 15 loop
            for dy in 0 to 31 loop
                ar := (chk / 4) * 32 - 16 + dy;
                if ar >= 0 and ar < 112 then
                    for w in 0 to 1 loop
                        ac := (chk mod 4) * 32 + 16 * w;
                        v(chk * 64 + dy * 2 + w) :=
                            VIDEOMANCER_BUNKER(ar)(127 - ac downto 112 - ac);
                    end loop;
                end if;
            end loop;
        end loop;
        return v;
    end function;
    constant C_DEBRIS_ROM : t_word_rom_1k := build_debris_rom;
    constant C_DEBRIS     : integer := 3;     -- falling-piece slots

    -- Death / win timelines (frames).  Death (~2.4 s): the wizard eases to
    -- the screen centre while growing to ~3.2x, rumbles with a shake that
    -- builds from +-2 to +-16 px (D_FLY .. D_CLOUD-1, rainbow strobe from
    -- D_STROBE), then blows up in the 8x rainbow cloud (wizard hidden, world
    -- frozen; 4 frames x 8 = 32 frames) and blinks back in at the bottom
    -- (D_RESUME .. D_END-1, world runs, invulnerable).  Win: cloud puff +
    -- flight for W_FLY frames (grow to ~6.4x while flying up and across),
    -- reset at W_END.
    constant C_D_FLY    : integer := 20;
    constant C_D_STROBE : integer := 65;
    constant C_D_CLOUD  : integer := 80;
    constant C_D_RESUME : integer := 112;     -- = cloud end
    constant C_D_END    : integer := 142;
    constant C_DEATH_K  : integer := 80;      -- 3.2x
    -- Death TV.  Two dissolve levels (0..16, 16 = solid): WORLD (aliens,
    -- shields, floor, shots, pieces) and TV.  The world dissolves out (dt
    -- 0..15) as the wizard flies to the centre; the giant TV dissolves in
    -- around him (D_TV_IN .. +15) and stays fully up as the explosion starts;
    -- over the explosion's second half the TV dissolves out WHILE the world
    -- dissolves back in (D_TV_OUT .. +15, tv + world = 16: a per-pixel
    -- crossfade, see S2), the TV gone exactly as the explosion ends.  The
    -- zoomed death view is held until then (the TV lives in the wizard's
    -- view space; snapping the view back early drew a 1x-sized TV at the
    -- ship).  Ramp starts are multiples of 16 so dt(3..0) is the step.
    constant C_D_TV_IN  : integer := 16;
    constant C_D_TV_OUT : integer := 96;      -- = D_CLOUD + half the cloud
    type t_bayer4 is array (0 to 15) of unsigned(4 downto 0);
    constant C_BAYER4 : t_bayer4 := (         -- index {y(1..0), x(1..0)}
        to_unsigned( 0, 5), to_unsigned( 8, 5), to_unsigned( 2, 5), to_unsigned(10, 5),
        to_unsigned(12, 5), to_unsigned( 4, 5), to_unsigned(14, 5), to_unsigned( 6, 5),
        to_unsigned( 3, 5), to_unsigned(11, 5), to_unsigned( 1, 5), to_unsigned( 9, 5),
        to_unsigned(15, 5), to_unsigned( 7, 5), to_unsigned(13, 5), to_unsigned( 5, 5));
    constant C_HUGE_FRAME_TIME : integer := 7;     -- 4 frames x 8 vsyncs
    constant C_HUGE_SCR : integer := 8 * C_EXPL_W;                   -- 384
    constant C_W_FLY    : integer := 100;
    constant C_W_END    : integer := 120;
    constant C_CLOUD_FRAME_TIME : integer := 12;
    constant C_CLOUD_SCR : integer := 4 * C_EXPL_W;                  -- 192
    constant C_CLOUD_Y  : integer := 600;     -- rainbow cloud luma
    constant C_STROBE_Y : integer := 700;     -- electrified wizard luma

    -- Wizard colours (fixed — not affected by K3 Hue or the cellophane
    -- bands): purple body, white eyeball.  U/V-swapped like the palettes.
    constant C_WIZ_Y   : integer := 427;
    constant C_WIZ_U   : integer := 618;
    constant C_WIZ_V   : integer := 747;
    constant C_EYE_Y   : integer := 768;
    -- Drawn 2x (nearest neighbour) so the burst reads against 128 px aliens;
    -- spawned centred in the 128 cell (+16).
    constant C_EXPL_SCR : integer := 2 * C_EXPL_W;                   -- 96
    constant C_EXPL_OFF : integer := (C_CELL_W - C_EXPL_SCR) / 2;    -- 16
    -- 6 vsync ticks per animation frame ≈ 100 ms × 4 frames = 400 ms burst
    constant C_EXPL_FRAME_TIME : integer := 6;

    -- Alien-step-period LUT indexed by top 4 bits of the Alien-Spd pot.
    -- Pot = 0 → 12 frames (the previous default), pot = 1023 → 1 frame
    -- per step (fastest).  Saturates at the bottom so pot values above
    -- the 12th step still hold at 1.
    type t_period_lut is array (0 to 15) of unsigned(5 downto 0);
    constant C_PERIOD_LUT : t_period_lut := (
        to_unsigned(12, 6), to_unsigned(11, 6),
        to_unsigned(10, 6), to_unsigned( 9, 6),
        to_unsigned( 8, 6), to_unsigned( 7, 6),
        to_unsigned( 6, 6), to_unsigned( 5, 6),
        to_unsigned( 4, 6), to_unsigned( 3, 6),
        to_unsigned( 2, 6), to_unsigned( 1, 6),
        to_unsigned( 1, 6), to_unsigned( 1, 6),
        to_unsigned( 1, 6), to_unsigned( 1, 6)
    );

    -- ------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
    -- ------------------------------------------------------------------
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

    -- ------------------------------------------------------------------
    -- Compares against a CONSTANT, as plain bit logic (v0.7.7).  yosys maps
    -- every "<" / ">=" to a carry chain even when one side is a constant (a
    -- carry-only LC per bit, plus chain legalisation); these build the same
    -- predicate from AND/OR terms, which abc packs into a few shared LUTs.
    -- ------------------------------------------------------------------
    -- '1' when unsigned x < c.
    function f_ult(x : unsigned; c : integer) return std_logic is
        constant N    : natural := x'length;
        alias    xv   : unsigned(N - 1 downto 0) is x;
        variable cb   : unsigned(N - 1 downto 0);
        variable v_eq : std_logic := '1';   -- all higher bits equal to c's
        variable v_lt : std_logic := '0';
    begin
        if c <= 0 then
            return '0';
        elsif c >= 2 ** N then
            return '1';
        end if;
        cb := to_unsigned(c, N);
        for i in N - 1 downto 0 loop
            if cb(i) = '1' then
                v_lt := v_lt or (v_eq and not xv(i));
                v_eq := v_eq and xv(i);
            else
                v_eq := v_eq and not xv(i);
            end if;
        end loop;
        return v_lt;
    end function;
    -- '1' when signed x < c (offset binary: flip the sign bit).
    function f_slt(x : signed; c : integer) return std_logic is
        constant N : natural := x'length;
        variable u : unsigned(N - 1 downto 0);
    begin
        u := unsigned(x);
        u(N - 1) := not u(N - 1);
        return f_ult(u, c + 2 ** (N - 1));
    end function;

    -- ========================================================================
    -- Parameters
    -- ========================================================================
    signal s_alien_spd_pot  : unsigned(9 downto 0);
    signal s_bullet_spd_pot : unsigned(9 downto 0);
    signal s_bomb_spd_pot   : unsigned(9 downto 0);
    signal s_bomb_rate_pot  : unsigned(9 downto 0);
    signal s_trail_pot      : unsigned(9 downto 0);
    signal s_hue_pot        : unsigned(9 downto 0);
    signal s_bomb_speed     : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal s_sw_fire        : std_logic;
    signal s_sw_bg_black    : std_logic;
    signal s_sw_color       : std_logic;
    signal s_sw_pause       : std_logic;
    signal s_sw_reset       : std_logic;
    signal s_ship_pos_pot   : unsigned(9 downto 0);

    signal s_sw_fire_prev  : std_logic := '0';
    signal s_sw_reset_prev : std_logic := '0';
    signal s_fire_pending  : std_logic := '0';
    signal s_reset_pending : std_logic := '0';
    -- Round-robin wand-shot slot — every Switch-7 edge spawns at the
    -- next slot in rotation (overwriting if all 4 are in flight), so
    -- every toggle produces a visible shot.
    signal s_fire_idx      : unsigned(2 downto 0) := (others => '0');

    signal s_step_period   : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_bullet_speed  : unsigned(4 downto 0) := to_unsigned(12, 5);
    signal s_bomb_cd_reset : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_form_step     : signed(11 downto 0) := to_signed(12, 12);
    -- K2 Trail: edge-shadow rest width and max stretch.
    signal s_fr_max        : unsigned(6 downto 0) := to_unsigned(46, 7);

    -- ========================================================================
    -- Timing + auto-measured resolution
    -- ========================================================================
    signal s_timing        : t_video_timing_port;
    -- Real pixel / line counts: only the 4x4 Bayer dissolve uses them (a
    -- pixel-scale texture); everything else uses the virtual coordinates.
    signal s_h_count       : unsigned(11 downto 0);
    signal s_v_count       : unsigned(11 downto 0);

    -- Virtual 1080-line coordinates (p_vcoord), advanced on exactly the same
    -- events as pixel_counter: x is Q.1 (steps 2/3/4 = 1.0/1.5/2.0 per
    -- pixel), y is Q.8 per field line.  s_vx / s_vy are the integer parts;
    -- at 1080p they equal h_count / v_count.
    signal s_xs            : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys            : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xa            : unsigned(12 downto 0) := (others => '0');
    signal s_ya            : unsigned(19 downto 0) := (others => '0');
    signal s_ya_nx         : unsigned(19 downto 0);
    signal s_vx            : unsigned(11 downto 0);
    signal s_vy            : unsigned(11 downto 0);
    -- '1' during an interlaced bottom field (starts half a step down; its
    -- end-of-field height is not measured).
    signal s_bot           : std_logic := '0';
    -- (integer y step - 1) for this field; extra DDA adds per line.
    signal s_ysm1          : unsigned(2 downto 0) := (others => '0');
    signal s_vrep_ld       : unsigned(2 downto 0);
    signal s_vrep          : unsigned(2 downto 0) := (others => '0');
    -- Floor line height: 4, or 5 at 480i (> the 4.44-unit field step).
    signal s_floor_t       : unsigned(2 downto 0) := to_unsigned(4, 3);

    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);

    signal s_ship_y        : signed(11 downto 0) := to_signed(412, 12);
    signal s_bunker_y      : signed(11 downto 0) := to_signed(290, 12);
    -- Dynamic formation init.  At v_active = 540 the formation can only
    -- fit with its top row off-screen (3 rows × 144 = 432 > 274 px above
    -- bunker), so we slot the bottom row just above the bunker.  At
    -- v_active ≥ 720 (HD HDMI / 1080p) all 3 rows fit, so we drop the
    -- whole formation to a small top margin.
    signal s_form_x_init   : signed(11 downto 0) := to_signed(48, 12);
    signal s_form_y_init   : signed(11 downto 0) := to_signed(-160, 12);
    -- One-shot boot init.  s_boot_count counts up to 9 and saturates;
    -- phase 0 of the FSM triggers a single init reset on the vsync where
    -- the counter holds value 8 — short enough that the wizard can
    -- shoot immediately after power-on, but long enough for v_active to
    -- settle and bomb_cooldown to be loaded.
    signal s_boot_count    : unsigned(3 downto 0) := (others => '0');
    type t_bunker_x_arr is array (0 to C_BUNKER_COUNT - 1) of signed(11 downto 0);
    signal s_bunker_x      : t_bunker_x_arr := (others => (others => '0'));
    signal s_bx_k          : unsigned(1 downto 0) := (others => '0');
    signal s_bx_acc        : signed(11 downto 0) := (others => '0');

    signal s_ship_x_max    : unsigned(11 downto 0)
                              := to_unsigned(960 - C_SHIP_W, 12);
    signal s_form_x_max    : signed(11 downto 0)
                              := to_signed(960 - C_FORM_W - C_BORDER, 12);
    constant C_FORM_X_MIN  : signed(11 downto 0) := to_signed(C_BORDER, 12);

    -- ========================================================================
    -- Game state
    -- ========================================================================
    signal s_aliens        : std_logic_vector(0 to C_ALIEN_SLOTS - 1)
                              := (others => '1');
    type t_bunker_bits is array (0 to C_BUNKER_COUNT - 1)
                          of std_logic_vector(0 to C_CHUNKS_PER_BNK - 1);
    -- Bunker chunk bits live in two 32 x 1 EBRs: even bunkers (0, 2) and odd
    -- (1, 3), address = bunker(1) & chunk.  A pixel needs one chunk bit per
    -- RAM (neighbouring bunkers can overlap only on rasters under 512 px,
    -- and are never in the same RAM); the writes are one erosion per frame
    -- and the 32-clock refill on a game reset, all in vertical blanking.
    type t_bnk_ram is array (0 to 31) of std_logic_vector(0 downto 0);
    signal s_bnk_ram_e, s_bnk_ram_o : t_bnk_ram := (others => (others => '1'));
    attribute ram_style : string;
    attribute ram_style of s_bnk_ram_e : signal is "block";
    attribute ram_style of s_bnk_ram_o : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_bnk_ram_e : signal is true;
    attribute no_rw_check of s_bnk_ram_o : signal is true;
    signal s_bnk_qe, s_bnk_qo   : std_logic_vector(0 downto 0) := (others => '0');
    signal s_bnk_rae, s_bnk_rao : unsigned(4 downto 0);
    signal s_bnk_ene, s_bnk_eno : std_logic := '0';
    signal s_bnk_we             : std_logic_vector(1 downto 0) := "00";
    signal s_bnk_wa             : unsigned(4 downto 0) := (others => '0');
    signal s_bnk_wd             : std_logic := '1';
    signal s_bnk_rcnt           : unsigned(5 downto 0) := (others => '1');
    signal s_hit_mb_v_d         : std_logic := '0';
    signal s_hit_mb_bnk_d       : unsigned(1 downto 0) := (others => '0');
    signal s_hit_mb_chk_d       : unsigned(3 downto 0) := (others => '0');
    signal s_bnk_rst_stb   : std_logic := '0';   -- = whole-game reset strobe
    signal s_bnk_clr_stb   : std_logic := '0';

    signal s_form_x        : signed(11 downto 0) := to_signed(C_FORM_X_INIT, 12);
    signal s_form_y        : signed(11 downto 0) := to_signed(C_FORM_Y_INIT, 12);
    signal s_form_dir      : std_logic := '1';
    signal s_step_counter  : unsigned(5 downto 0) := (others => '0');
    signal s_alien_frame   : std_logic := '0';

    -- Ship X + facing tracker
    -- Serial shift-add multiplier for ship_x = pot * ship_x_max / 1024
    -- (11 clocks per result, replaces a 10x11 array multiplier).
    signal s_sm_cnt        : unsigned(3 downto 0) := (others => '0');
    signal s_sm_a          : unsigned(9 downto 0) := (others => '0');
    signal s_sm_m          : unsigned(10 downto 0) := (others => '0');
    signal s_sm_acc        : unsigned(20 downto 0) := (others => '0');
    signal s_ship_x        : signed(11 downto 0) := to_signed(416, 12);
    signal s_ship_x_prev   : signed(11 downto 0) := to_signed(416, 12);
    -- '0' = sprite as authored (faces left visually based on the bitmap);
    -- '1' = mirrored (faces right).  Set by p_facing so the wizard ends
    -- up facing the direction of travel.
    signal s_wizard_facing : std_logic := '0';

    -- Per-bullet state
    type t_pos_arr_b is array (0 to C_BULLETS - 1) of signed(11 downto 0);
    -- Bullets park well off-screen-bottom when inactive so the registered
    -- "at top" flag is never stale-true for a slot about to be re-fired (a
    -- y of 0 used to make every fresh slot's first shot self-deactivate).
    constant C_BULLET_PARK : integer := 2000;
    signal s_bullet_x      : t_pos_arr_b := (others => (others => '0'));
    signal s_bullet_y      : t_pos_arr_b
                              := (others => to_signed(C_BULLET_PARK, 12));
    signal s_bullet_active : std_logic_vector(0 to C_BULLETS - 1)
                              := (others => '0');
    signal s_bullet_at_top : std_logic_vector(0 to C_BULLETS - 1)
                              := (others => '0');

    -- Per-bomb state
    type t_pos_arr_m is array (0 to C_BOMBS - 1) of signed(11 downto 0);
    signal s_bomb_x        : t_pos_arr_m := (others => (others => '0'));
    signal s_bomb_y        : t_pos_arr_m := (others => (others => '0'));
    signal s_bomb_active   : std_logic_vector(0 to C_BOMBS - 1)
                              := (others => '0');
    signal s_bomb_at_bot   : std_logic_vector(0 to C_BOMBS - 1)
                              := (others => '0');
    -- Initialise high so no bombs drop during the 8-vsync boot window
    -- (before s_bomb_cd_reset can be loaded by the reset path).
    signal s_bomb_cooldown : unsigned(7 downto 0) := to_unsigned(180, 8);

    -- Bomb-column random source: 16-bit Galois LFSR stepped 3 times per
    -- FRAME (phase 1).  The old 8-bit LFSR stepped every clock and was
    -- sampled once per frame; 2,475,000 clocks/frame = 225 (mod 255), so the
    -- samples walked a 17-state cycle dominated by columns 3 and 4.
    signal s_rng           : std_logic_vector(15 downto 0) := x"ACE1";

    -- Pre-registered bomb-spawn pick (LFSR column + its lowest live row),
    -- computed continuously in p_bomb_pick so the s_aliens read stays off the
    -- phase-1 bomb-X write path.
    signal s_bsp_pick      : unsigned(2 downto 0) := (others => '0');
    signal s_bsp_col       : unsigned(2 downto 0) := (others => '0');
    signal s_bsp_row       : unsigned(1 downto 0) := (others => '0');
    signal s_bsp_can       : std_logic := '0';

    -- Kill counter for the "fewer aliens → faster march" speed boost.
    -- Increments on each alien kill, resets in any reset path.  Max 18
    -- (= 3×6 grid; 5-bit wide fits 0..31).
    signal s_dead_count    : unsigned(4 downto 0) := (others => '0');

    -- '1' = the lowest LIVE alien row has reached the bottom of the bunkers
    -- (game over: death animation, then reset).  Pipelined: live-row offset,
    -- then bottom row, then the compare.
    signal s_form_at_ship  : std_logic := '0';
    signal s_land_off      : signed(11 downto 0) := to_signed(384, 12);
    signal s_land_any      : std_logic := '1';
    signal s_land_any_q    : std_logic := '1';
    signal s_land_bot      : signed(11 downto 0) := (others => '0');
    signal s_bunker_bot    : signed(11 downto 0) := to_signed(402, 12);
    signal s_aliens_clear  : std_logic := '0';
    -- Pre-registered reset trigger — collapses (reset_pending OR
    -- game_over OR form_at_ship OR boot_count=8) into a single
    -- registered bit so phase 0's reset mux selector doesn't fan out
    -- across the chip and stretch the critical path.  Costs 1 cycle of
    -- latency on reset detection (imperceptible).
    signal s_do_reset      : std_logic := '0';

    -- Explosion (single slot — spawned in phase 4 when a wand-shot kills
    -- an alien, animates F0→F3 then disappears).
    signal s_expl_active   : std_logic := '0';
    signal s_expl_x        : signed(11 downto 0) := (others => '0');
    signal s_expl_y        : signed(11 downto 0) := (others => '0');
    signal s_expl_frame    : unsigned(1 downto 0) := (others => '0');
    signal s_expl_timer    : unsigned(3 downto 0) := (others => '0');
    -- '1' = the slot holds the 4x rainbow cloud (death / win launch).
    signal s_expl_big      : std_logic := '0';
    -- '1' = the cloud is the 8x death explosion (384 px) at screen centre.
    signal s_expl_huge     : std_logic := '0';
    signal s_cloud_huge    : std_logic := '0';
    -- Death-sequence dissolve levels 0..16 (16 = solid): the TV and the
    -- world (see C_D_TV_IN).
    signal s_tv_level      : unsigned(4 downto 0) := (others => '0');
    signal s_world_level   : unsigned(4 downto 0) := to_unsigned(16, 5);
    -- The 384-px death cloud's top-left: screen centre - 192 (p_layout).
    signal s_huge_x        : signed(11 downto 0) := to_signed(768, 12);
    signal s_huge_y        : signed(11 downto 0) := to_signed(348, 12);

    -- Story: death animation, win flight (p_story).  Requests are one-clock
    -- strobes from p_physics; the frame flags below are registered and only
    -- change in vblank.
    signal s_death_req     : std_logic := '0';
    signal s_death_end_req : std_logic := '0';  -- aliens landed: reset after
    signal s_win_req       : std_logic := '0';
    signal s_dying         : std_logic := '0';
    signal s_dt            : unsigned(8 downto 0) := (others => '0');
    signal s_end_reset     : std_logic := '0';
    signal s_winning       : std_logic := '0';
    signal s_wt            : unsigned(7 downto 0) := (others => '0');
    signal s_win_dir       : std_logic := '1';    -- '1' = flies right
    signal s_req_reset     : std_logic := '0';
    signal s_frozen        : std_logic := '0';
    signal s_wiz_vis       : std_logic := '1';
    signal s_wiz_strobe    : std_logic := '0';
    signal s_cloud_req     : std_logic := '0';
    signal s_cloud_x       : signed(11 downto 0) := (others => '0');
    signal s_cloud_y       : signed(11 downto 0) := (others => '0');
    -- Win flight: centre position + velocity in 12.4 fixed point, and the
    -- zoom step k (sprite px per screen px x256: 256 = 1x, 40 = 6.4x).
    signal s_fx            : signed(15 downto 0) := (others => '0');
    signal s_fy            : signed(15 downto 0) := (others => '0');
    signal s_fvx           : signed(11 downto 0) := (others => '0');
    signal s_fvy           : signed(11 downto 0) := (others => '0');
    signal s_zk            : unsigned(8 downto 0) := to_unsigned(256, 9);
    -- Per-frame wizard view (centre + k) handed to p_zoom.
    signal s_zoom_xc       : signed(11 downto 0) := (others => '0');
    signal s_zoom_yc       : signed(11 downto 0) := (others => '0');
    signal s_zoom_k        : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_phase8        : unsigned(2 downto 0) := (others => '0');
    -- Alien TV-screen animation frame (0..4) and its divider.
    signal s_scr_anim      : unsigned(2 downto 0) := (others => '0');
    signal s_scr_div       : unsigned(2 downto 0) := (others => '0');

    -- Wizard rendering by DDA: u/v = sprite-space coords x256 of the current
    -- pixel, u = u_start + h_count*k, v = v_start + v_count*k, so any zoom
    -- (and 1x, the normal case) needs no per-pixel multiply.  u_start /
    -- v_start = 64*256 - centre*k, built once per frame by p_zoom's serial
    -- multiplier.
    -- (20-bit: |u|,|v| stay under 2^19 for every view the game produces.)
    signal s_zs_go         : std_logic := '0';
    signal s_zm_cnt        : unsigned(4 downto 0) := (others => '0');
    signal s_zm_op         : signed(11 downto 0) := (others => '0');
    signal s_zm_k          : unsigned(8 downto 0) := (others => '0');
    signal s_zm_ksh        : unsigned(8 downto 0) := (others => '0');
    signal s_zm_acc        : signed(19 downto 0) := (others => '0');
    signal s_u_start       : signed(19 downto 0) := (others => '0');
    signal s_v_start       : signed(19 downto 0) := (others => '0');
    signal s_k_frame       : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_v_load        : std_logic := '0';
    signal s_u_acc         : signed(19 downto 0) := (others => '0');
    signal s_v_acc         : signed(19 downto 0) := (others => '0');

    -- Falling bunker pieces (p_debris).
    -- A piece falls straight down from its chunk, so its column is fixed:
    -- x is kept as {bunker, chunk col} and rendered from the bunker's own
    -- S2 delta (no per-piece x subtractor); y is absolute.
    type t_deb_pos is array (0 to C_DEBRIS - 1) of signed(11 downto 0);
    type t_deb_vy  is array (0 to C_DEBRIS - 1) of signed(5 downto 0);
    type t_deb_chk is array (0 to C_DEBRIS - 1) of unsigned(3 downto 0);
    type t_deb_bnk is array (0 to C_DEBRIS - 1) of unsigned(1 downto 0);
    signal s_deb_a         : std_logic_vector(0 to C_DEBRIS - 1)
                              := (others => '0');
    signal s_deb_y         : t_deb_pos := (others => (others => '0'));
    signal s_deb_vy        : t_deb_vy := (others => (others => '0'));
    signal s_deb_chk       : t_deb_chk := (others => (others => '0'));
    signal s_deb_bnk       : t_deb_bnk := (others => (others => '0'));
    signal s_deb_spawn_stb : std_logic := '0';
    signal s_deb_step_stb  : std_logic := '0';
    -- Spawn row (chunk top) for the frame's bomb bunker hit.
    signal s_dsp_mb_y      : signed(11 downto 0) := (others => '0');

    -- Rainbow edge-shadow.  No history buffer: a SOLID rainbow block grows
    -- from the wizard's trailing edge (a prefix-OR of the silhouette row, so
    -- internal art holes are filled).  Always touching the wizard; its width
    -- tracks speed.  s_ghost_phase animates the hue over time.
    signal s_ghost_phase   : unsigned(2 downto 0) := (others => '0');
    signal s_ghost_pdiv    : unsigned(2 downto 0) := (others => '0');
    signal s_ship_x_vprev  : signed(11 downto 0) := (others => '0');
    -- Per-frame speed (|Δship_x|, clamped) and the smoothed band width.
    signal s_ghost_dx      : signed(11 downto 0) := (others => '0');
    signal s_ghost_spd     : unsigned(7 downto 0) := (others => '0');
    signal s_fringe_step   : unsigned(6 downto 0) := (others => '0');
    -- Pipelined easing (see p_ghost).
    signal s_spd_prev      : unsigned(7 downto 0) := (others => '0');
    signal s_gh_avg        : unsigned(8 downto 0) := (others => '0');
    signal s_gh_f          : unsigned(4 downto 0) := (others => '0');
    signal s_gh_prod       : unsigned(11 downto 0) := (others => '0');
    -- '1' while the trail has any length (it is hidden at 0).
    signal s_trail_on      : std_logic := '0';
    signal s_gh_tgt        : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_gh_tgt_q      : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_gh_up         : std_logic := '0';
    signal s_gh_dn         : std_logic := '0';
    signal s_gh_snap_up    : std_logic := '0';
    signal s_gh_snap_dn    : std_logic := '0';

    -- ========================================================================
    -- Physics FSM intermediates
    -- ========================================================================
    signal s_ph_phase      : unsigned(2 downto 0) := (others => '0');
    signal s_ph_new_fx     : signed(11 downto 0) := (others => '0');
    -- Latched in phase 0: '1' when this frame advances (no reset, not
    -- paused) — gates the hit commits so a fresh board never eats stale
    -- collisions; plus the fire request (s_fire_pending clears at vsync).
    signal s_ph_run        : std_logic := '0';
    signal s_ph_reset      : std_logic := '0';
    -- '1' = animations advance this frame (not reset, not paused) — the
    -- world itself (s_ph_run) is also frozen during death and the win.
    signal s_ph_anim       : std_logic := '0';
    signal s_ph_fire       : std_logic := '0';

    -- ========================================================================
    -- Render-time collision latches (hardware sprite-collision style)
    -- ========================================================================
    -- Set by p_collide on the FIRST lit-pixel overlap of each kind in a frame
    -- (raster order); snapshotted into s_hit_* and cleared at vsync_start;
    -- committed by the FSM in phases 1/2.  An overlap the FSM didn't resolve
    -- this frame (second simultaneous hit) is still overlapping next frame
    -- and latches then.  Projectile speeds stay < 32 px/frame so a shot can
    -- never jump a 16-px chunk row between frames.
    -- ab = bullet x alien, mb = shield damage (shot / bomb / alien x
    -- bunker — every knocked-out chunk falls away), ms = bomb x wizard.
    signal s_cl_ab_v       : std_logic := '0';
    signal s_cl_ab_alien   : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                              := (others => '0');
    signal s_cl_ab_bul     : unsigned(2 downto 0) := (others => '0');
    signal s_cl_mb_v       : std_logic := '0';
    signal s_cl_mb_bnk     : unsigned(1 downto 0) := (others => '0');
    signal s_cl_mb_chk     : unsigned(3 downto 0) := (others => '0');
    signal s_cl_mb_bomb    : unsigned(1 downto 0) := (others => '0');
    signal s_cl_ms_v       : std_logic := '0';
    -- What knocked the chunk out (shot > bomb > alien at the same pixel), and
    -- the shot's slot so it can be retired.
    signal s_cl_mb_isbul   : std_logic := '0';
    signal s_cl_mb_isbomb  : std_logic := '0';
    signal s_cl_mb_bul     : unsigned(2 downto 0) := (others => '0');

    signal s_hit_ab_v      : std_logic := '0';
    signal s_hit_ab_alien  : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                              := (others => '0');
    signal s_hit_ab_bul    : unsigned(2 downto 0) := (others => '0');
    signal s_hit_mb_v      : std_logic := '0';
    signal s_hit_mb_bnk    : unsigned(1 downto 0) := (others => '0');
    signal s_hit_mb_chk    : unsigned(3 downto 0) := (others => '0');
    signal s_hit_mb_bomb   : unsigned(1 downto 0) := (others => '0');
    signal s_hit_ms_v      : std_logic := '0';
    signal s_hit_mb_isbomb : std_logic := '0';
    signal s_hit_mb_isbul  : std_logic := '0';
    signal s_hit_mb_bul    : unsigned(2 downto 0) := (others => '0');
    -- Explosion spawn for the alien hit, built from the latched cell at
    -- snapshot time (form_x/y are stable then: the FSM moves them later).
    signal s_hit_expl_x    : signed(11 downto 0) := (others => '0');
    signal s_vs_col        : std_logic := '0';
    attribute keep : boolean;
    attribute keep of s_vs_col : signal is true;
    signal s_hit_expl_y    : signed(11 downto 0) := (others => '0');

    -- ========================================================================
    -- Render pipeline registers
    -- ========================================================================
    -- S1: coords
    signal s_stg1_hx       : signed(11 downto 0) := (others => '0');
    signal s_stg1_vy       : signed(11 downto 0) := (others => '0');
    -- Wizard sprite-space coords (integer part of the DDA accumulators).
    signal s_stg1_u        : signed(11 downto 0) := (others => '0');
    signal s_stg1_v        : signed(11 downto 0) := (others => '0');

    -- S2: deltas (live game state)
    signal s_stg2_form_dx  : signed(11 downto 0) := (others => '0');
    signal s_stg2_form_dy  : signed(11 downto 0) := (others => '0');
    signal s_stg2_ship_dx  : signed(11 downto 0) := (others => '0');
    signal s_stg2_ship_dy  : signed(11 downto 0) := (others => '0');
    type t_d_arr_b is array (0 to C_BULLETS - 1) of signed(11 downto 0);
    type t_d_arr_m is array (0 to C_BOMBS - 1) of signed(11 downto 0);
    signal s_stg2_bull_dx  : t_d_arr_b := (others => (others => '0'));
    signal s_stg2_bull_dy  : t_d_arr_b := (others => (others => '0'));
    signal s_stg2_bomb_dx  : t_d_arr_m := (others => (others => '0'));
    signal s_stg2_bomb_dy  : t_d_arr_m := (others => (others => '0'));
    type t_bunker_d_arr is array (0 to C_BUNKER_COUNT - 1) of signed(11 downto 0);
    signal s_stg2_bnk_dx   : t_bunker_d_arr := (others => (others => '0'));
    signal s_stg2_bnk_dy   : signed(11 downto 0) := (others => '0');
    signal s_stg2_expl_dx  : signed(11 downto 0) := (others => '0');
    signal s_stg2_expl_dy  : signed(11 downto 0) := (others => '0');
    signal s_stg2_expl_a   : std_logic := '0';
    signal s_stg2_expl_fr  : unsigned(1 downto 0) := (others => '0');
    signal s_stg2_expl_big : std_logic := '0';
    signal s_stg2_expl_huge : std_logic := '0';
    signal s_stg2_tv_u     : signed(11 downto 0) := (others => '0');
    signal s_stg2_tv_v     : signed(11 downto 0) := (others => '0');
    -- Per-pixel dissolve decisions (Bayer cell b): the alien sprite path
    -- draws the TV here if b < tv_level; world objects show if
    -- (15 - b) < world_level.  With tv + world = 16 every pixel is exactly
    -- one of them — a crossfade through one sprite path.
    signal s_stg2_use_tv   : std_logic := '0';
    signal s_stg2_wshow    : std_logic := '1';
    type t_deb_d is array (0 to C_DEBRIS - 1) of signed(11 downto 0);
    signal s_stg2_deb_dy   : t_deb_d := (others => (others => '0'));
    signal s_stg2_deb_a    : std_logic_vector(0 to C_DEBRIS - 1)
                              := (others => '0');
    signal s_stg2_frame    : std_logic := '0';
    signal s_stg2_bull_a   : std_logic_vector(0 to C_BULLETS - 1)
                              := (others => '0');
    signal s_stg2_bomb_a   : std_logic_vector(0 to C_BOMBS - 1)
                              := (others => '0');

    -- S3: in-bbox flags + indices + ROM word addresses + in-word bit indices
    signal s_stg3_in_form    : std_logic := '0';
    signal s_stg3_alien_idx  : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                                 := (others => '0');
    signal s_stg3_alien_addr : unsigned(8 downto 0) := (others => '0');
    signal s_stg3_alien_bit  : unsigned(3 downto 0) := (others => '0');
    signal s_stg3_in_scr     : std_logic := '0';
    signal s_stg3_tv         : std_logic := '0';
    signal s_stg3_dith       : std_logic := '1';
    signal s_stg3_wshow      : std_logic := '1';
    signal s_stg3_scr_addr   : unsigned(8 downto 0) := (others => '0');
    signal s_stg3_scr_bit    : unsigned(3 downto 0) := (others => '0');
    signal s_stg3_in_ship    : std_logic := '0';
    -- Trail box: the ship box widened by C_TRAIL_EXT px each side so a long
    -- edge-shadow isn't clipped flat at the 128-px sprite edge.
    signal s_stg3_in_trail   : std_logic := '0';
    signal s_stg3_wiz_addr   : unsigned(9 downto 0) := (others => '0');
    signal s_stg3_wiz_bit    : unsigned(3 downto 0) := (others => '0');
    signal s_stg3_grp_row    : unsigned(6 downto 0) := (others => '0');
    -- Edge-shadow base = this pixel's row-vector index (127 - sprite col
    -- when not mirrored), extended past the box: -128 .. 255.
    signal s_stg3_gh_base    : signed(9 downto 0) := (others => '0');
    signal s_stg3_hue_col    : unsigned(2 downto 0) := (others => '0');
    signal s_stg3_in_bullet  : std_logic_vector(0 to C_BULLETS - 1)
                                := (others => '0');
    signal s_stg3_in_bomb    : std_logic_vector(0 to C_BOMBS - 1)
                                := (others => '0');
    signal s_stg3_in_bunker  : std_logic_vector(0 to C_BUNKER_COUNT - 1)
                                := (others => '0');
    signal s_stg3_bnk_addr   : unsigned(9 downto 0) := (others => '0');
    signal s_stg3_bnk_bit    : unsigned(3 downto 0) := (others => '0');
    signal s_stg3_chunk_idx  : unsigned(3 downto 0) := (others => '0');
    -- One copy of the chunk index per bunker (keep): the shared register's
    -- fanout into all four 16:1 chunk-alive muxes was a seed-dependent
    -- 8.5 ns routing hop.
    type t_chunk_idx_arr is array (0 to C_BUNKER_COUNT - 1) of unsigned(3 downto 0);
    signal s_stg3_chunk_idx_b : t_chunk_idx_arr := (others => (others => '0'));
    attribute keep of s_stg3_chunk_idx_b : signal is true;
    signal s_stg3_bnk_id     : unsigned(1 downto 0) := (others => '0');
    signal s_stg3_in_expl    : std_logic := '0';
    signal s_stg3_expl_addr  : unsigned(9 downto 0) := (others => '0');
    signal s_stg3_expl_bit   : unsigned(3 downto 0) := (others => '0');
    signal s_stg3_in_cloud   : std_logic := '0';
    -- |src - 24| per axis of the cloud art (diamond-ring rainbow index).
    signal s_stg3_cl_ar      : unsigned(4 downto 0) := (others => '0');
    signal s_stg3_cl_ac      : unsigned(4 downto 0) := (others => '0');
    signal s_stg3_in_deb     : std_logic := '0';
    signal s_stg3_deb_addr   : unsigned(9 downto 0) := (others => '0');
    signal s_stg3_deb_bit    : unsigned(3 downto 0) := (others => '0');

    -- S4: sync ROM word reads + state lookups
    signal s_stg4_alien_word : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_wiz_word   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_eye_word   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_grp        : std_logic_vector(31 downto 0) := (others => '0');
    signal s_stg4_bunker_word : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_expl_word  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_in_form    : std_logic := '0';
    signal s_stg4_alien_alive : std_logic := '0';
    signal s_stg4_alien_idx  : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                                 := (others => '0');
    signal s_stg4_alien_bit  : unsigned(3 downto 0) := (others => '0');
    signal s_stg4_scr_word   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_in_scr     : std_logic := '0';
    signal s_stg4_dith       : std_logic := '1';
    signal s_stg4_wshow      : std_logic := '1';
    signal s_stg4_scr_bit    : unsigned(3 downto 0) := (others => '0');
    signal s_stg4_chunk_idx  : unsigned(3 downto 0) := (others => '0');
    signal s_stg4_bnk_id     : unsigned(1 downto 0) := (others => '0');
    -- Cellophane band of this pixel: "00" top, "01" middle, "10" bottom.
    signal s_stg4_band       : unsigned(1 downto 0) := "01";
    signal s_stg4_in_ship    : std_logic := '0';
    signal s_stg4_in_trail   : std_logic := '0';
    signal s_stg4_wiz_bit    : unsigned(3 downto 0) := (others => '0');
    -- Edge-shadow group indices (4-px groups), clamped to -1 .. 32: -1 =
    -- before the row, 32 = past it.  S5 prefix-ORs the group word at both.
    -- (7-bit signed, values -1 .. 32)
    signal s_stg4_gh_base_g  : signed(6 downto 0) := (others => '0');
    signal s_stg4_gh_tap_g   : signed(6 downto 0) := (others => '0');
    signal s_stg4_in_bullet  : std_logic_vector(0 to C_BULLETS - 1)
                                := (others => '0');
    signal s_stg4_in_bomb    : std_logic_vector(0 to C_BOMBS - 1)
                                := (others => '0');
    signal s_stg4_in_bunker_any : std_logic := '0';
    signal s_stg4_bnk_bit    : unsigned(3 downto 0) := (others => '0');
    signal s_stg4_chunk_alive : std_logic;
    signal s_stg4_in_expl    : std_logic := '0';
    signal s_stg4_expl_bit   : unsigned(3 downto 0) := (others => '0');
    signal s_stg4_in_cloud   : std_logic := '0';
    signal s_stg4_deb_word   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stg4_in_deb     : std_logic := '0';
    signal s_stg4_deb_bit    : unsigned(3 downto 0) := (others => '0');
    -- Rainbow palette index for this pixel (cloud rings / electrified
    -- strobe bands / edge-shadow hue), resolved here so S5 is one lookup.
    signal s_stg4_cidx       : unsigned(2 downto 0) := (others => '0');

    -- S5: combined per-region pixel flags
    signal s_stg5_on_alien   : std_logic := '0';
    signal s_stg5_on_ship    : std_logic := '0';
    signal s_stg5_on_eye     : std_logic := '0';
    signal s_stg5_on_bullet  : std_logic := '0';
    signal s_stg5_on_bomb    : std_logic := '0';
    signal s_stg5_on_bunker  : std_logic := '0';
    signal s_stg5_on_expl    : std_logic := '0';   -- small, band-coloured
    signal s_stg5_on_cloud   : std_logic := '0';   -- 4x rainbow cloud
    signal s_stg5_on_debris  : std_logic := '0';
    signal s_stg5_wshow      : std_logic := '1';
    signal s_stg5_on_floor   : std_logic := '0';
    signal s_stg5_tr_out     : std_logic := '0';   -- outer edge-shadow
    signal s_stg5_tr_gap     : std_logic := '0';   -- gap fill
    signal s_stg5_trail_u    : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg5_trail_v    : unsigned(9 downto 0) := C_CHROMA_MID;
    -- Collision indices carried alongside the S5 pixel flags.
    signal s_stg5_alien_idx  : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                                 := (others => '0');
    signal s_stg5_bul_id     : unsigned(2 downto 0) := (others => '0');
    signal s_stg5_bomb_id    : unsigned(1 downto 0) := (others => '0');
    signal s_stg5_chunk_idx  : unsigned(3 downto 0) := (others => '0');
    signal s_stg5_bnk_id     : unsigned(1 downto 0) := (others => '0');
    -- Foreground colour for this pixel's cellophane band.
    signal s_stg5_fg_y       : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_stg5_fg_u       : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg5_fg_v       : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Floor line geometry — 4-px stripe at the wizard's feet, full
    -- screen width.  Computed in p_layout based on ship_y.
    constant C_FLOOR_H     : integer := 4;
    signal s_floor_y       : signed(11 downto 0) := to_signed(536, 12);
    signal s_floor_y_end   : signed(11 downto 0) := to_signed(540, 12);

    -- S6: color mux outputs
    signal s_out_y         : unsigned(9 downto 0) := (others => '0');
    signal s_out_u         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_out_v         : unsigned(9 downto 0) := C_CHROMA_MID;
    -- Frame-constant band colours (K3 Hue + S9 Cellophane), resolved in
    -- p_fg_color (one shared LUT read, round-robin over the 3 bands) so S5
    -- is a plain 3:1 select.
    type t_band_col is array (0 to 2) of unsigned(9 downto 0);
    signal s_band_y        : t_band_col := (others => to_unsigned(768, 10));
    signal s_band_u        : t_band_col := (others => C_CHROMA_MID);
    signal s_band_v        : t_band_col := (others => C_CHROMA_MID);
    signal s_band_k        : integer range 0 to 2 := 0;
    signal s_band_kq       : integer range 0 to 2 := 0;
    signal s_band_idx_q    : unsigned(3 downto 0) := (others => '0');
    -- Band boundaries (rows): top band above s_band_top_y, bottom band from
    -- the bunker top down.
    signal s_band_top_y    : signed(11 downto 0) := to_signed(135, 12);
    signal s_out_show      : std_logic := '0';
    signal s_out_avid      : std_logic := '0';

    -- ========================================================================
    -- Sync delay: 8 render stages, then straight to the 2 IO stages.
    -- ========================================================================
    type t_sync_pipe is array (0 to C_DELAY - 1) of std_logic_vector(3 downto 0);
    signal s_sync_pipe     : t_sync_pipe := (others => (others => '0'));

    -- Background video is tapped EARLY (2 clocks deep, gated to black outside
    -- its own avid) instead of riding an 8-deep 30-bit FF delay: the picture
    -- leads the sync by 6 px (invisible), saves ~180 LCs, and keeps raw
    -- input video shallow (the rollthebones HD-HDMI fix).
    signal s_vid_y0        : std_logic_vector(9 downto 0) := (others => '0');
    signal s_vid_u0        : std_logic_vector(9 downto 0) := (others => '0');
    signal s_vid_v0        : std_logic_vector(9 downto 0) := (others => '0');
    signal s_vid_y         : std_logic_vector(9 downto 0) := (others => '0');
    signal s_vid_u         : std_logic_vector(9 downto 0) := (others => '0');
    signal s_vid_v         : std_logic_vector(9 downto 0) := (others => '0');

    signal s_wet_y : std_logic_vector(9 downto 0);
    signal s_wet_u : std_logic_vector(9 downto 0);
    signal s_wet_v : std_logic_vector(9 downto 0);

    signal s_io_0 : t_video_stream_yuv444_30b;
    signal s_io_1 : t_video_stream_yuv444_30b;

begin

    -- ========================================================================
    -- Auto-Measure Active Resolution
    -- ========================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            -- The virtual counters use exactly the reset/advance this needs
            -- (zeroed on hsync_start / vsync_start, counting active pixels /
            -- lines), so latch them as they reset — no private duplicate
            -- counters.  Virtual: 1920 x 1080 at HD, 1440 x 1080 at SD.  A
            -- bottom field ends half a step lower, so only other fields are
            -- measured (HD inputs and progressive sources: every field).
            if s_timing.hsync_start = '1' and s_vx > 0 then
                s_measured_h <= s_vx;
            end if;
            if s_timing.vsync_start = '1' and s_vy > 0 and s_bot = '0' then
                s_measured_v <= s_vy;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Virtual 1080-line coordinates (resolution scaling)
    -- ========================================================================
    -- Same events as pixel_counter (x: zero at hsync_start, advance on avid;
    -- y: zero at vsync_start, advance at avid_start), with per-mode steps
    -- latched at vsync_start.  At 1080p the steps are 1.0 and the integer
    -- parts are h_count / v_count exactly.
    p_vcoord : process(clk)
        variable v_ys : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                s_xa <= (others => '0');
            elsif s_timing.avid = '1' then
                s_xa <= s_xa + s_xs;
            end if;

            if s_timing.vsync_start = '1' then
                v_ys  := f_vstep_y(registers_in(8)(3 downto 0));
                s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys  <= v_ys;
                s_ysm1 <= v_ys(10 downto 8) - 1;
                if v_ys(10 downto 8) = "100" then
                    s_floor_t <= to_unsigned(5, 3);
                else
                    s_floor_t <= to_unsigned(4, 3);
                end if;
                s_bot <= not data_in.field_n;
                if data_in.field_n = '0' then
                    s_ya <= resize(v_ys(11 downto 1), 20);
                else
                    s_ya <= (others => '0');
                end if;
            elsif s_timing.avid_start = '1' then
                s_ya <= s_ya_nx;
            end if;
        end if;
    end process;

    s_vx <= s_xa(12 downto 1);
    s_vy <= s_ya(19 downto 8);
    -- Extra wizard-DDA adds for the NEXT line's step: (integer y step - 1),
    -- plus the carry out of the fraction (the y adder's bit-8 carry-in).
    s_ya_nx   <= s_ya + resize(s_ys, 20);
    s_vrep_ld <= s_ysm1 + ("00" & (s_ya(8) xor s_ys(8) xor s_ya_nx(8)));

    p_layout : process(clk)
        variable v_h_div_8 : integer range 0 to 511;
        variable v_bx      : signed(11 downto 0);
        variable v_v       : integer range 0 to 4095;
    begin
        if rising_edge(clk) then

            v_v := to_integer(s_measured_v);
            -- Wizard hugs the bottom
            if f_ult(s_measured_v, C_SHIP_H + 1) = '0' then
                s_ship_y <= to_signed(v_v - C_SHIP_H, 12);
            else
                s_ship_y <= to_signed(0, 12);
            end if;
            -- Bunker sits C_BUNKER_GAP px above the wizard top (closer than
            -- before so it really crowds the player).
            if f_ult(s_measured_v, C_SHIP_H + C_BUNKER_H + C_BUNKER_GAP + 1) = '0' then
                s_bunker_y <= to_signed(v_v - C_SHIP_H - C_BUNKER_GAP
                                        - C_BUNKER_H, 12);
            else
                s_bunker_y <= to_signed(0, 12);
            end if;

            -- Bunker X at fractions 1/8, 3/8, 5/8, 7/8 of h_active: one
            -- accumulator walks them in turn (bunker k = bunker k-1 + h/4),
            -- re-deriving all four every 4 clocks from the measured width
            v_h_div_8 := to_integer(s_measured_h) / 8;
            s_bx_k <= s_bx_k + 1;
            if s_bx_k = 0 then
                v_bx := to_signed(v_h_div_8 - C_BUNKER_W / 2, 12);
            else
                v_bx := s_bx_acc + to_signed(2 * v_h_div_8, 12);
            end if;
            s_bx_acc <= v_bx;
            s_bunker_x(to_integer(s_bx_k)) <= v_bx;

            if f_ult(s_measured_h, C_SHIP_W + 1) = '0' then
                s_ship_x_max <= s_measured_h - to_unsigned(C_SHIP_W, 12);
            else
                s_ship_x_max <= (others => '0');
            end if;
            if f_ult(s_measured_h, C_FORM_W + C_BORDER + 1) = '0' then
                s_form_x_max <= to_signed(
                    to_integer(s_measured_h) - C_FORM_W - C_BORDER, 12);
            else
                s_form_x_max <= to_signed(C_BORDER, 12);
            end if;

            -- Cellophane top (red) band = the top 1/8 of the raster, the
            -- arcade overlay's saucer strip.
            s_band_top_y <= to_signed(v_v / 8, 12);

            -- Dynamic formation init derived from the auto-measured
            -- layout.  If C_FORM_H fits above the bunker below the red band,
            -- start the formation just under the band (all 3 rows visible,
            -- arcade layout).  Otherwise (cramped SD modes) park the bottom
            -- row 2 px above the bunker so it descends from the top.
            if s_bunker_y >= s_band_top_y + to_signed(C_FORM_H + 8, 12) then
                s_form_y_init <= s_band_top_y + to_signed(8, 12);
            else
                s_form_y_init <= s_bunker_y - to_signed(C_FORM_H + 2, 12);
            end if;
            -- Centre formation horizontally
            if f_ult(s_measured_h, C_FORM_W + 1) = '0' then
                s_form_x_init <= to_signed(
                    (to_integer(s_measured_h) - C_FORM_W) / 2, 12);
            else
                s_form_x_init <= to_signed(0, 12);
            end if;

            -- Floor line just under the wizard (at the very bottom of
            -- the sprite cell), full screen width.  C_FLOOR_H (4) units
            -- tall; 5 at 480i, whose field lines are 4.44 units apart.
            s_floor_y     <= signed(s_measured_v) - signed(resize(s_floor_t, 12));
            s_bunker_bot  <= s_bunker_y + to_signed(C_BUNKER_H, 12);
            s_huge_x      <= signed('0' & s_measured_h(11 downto 1))
                             - to_signed(C_HUGE_SCR / 2, 12);
            s_huge_y      <= signed('0' & s_measured_v(11 downto 1))
                             - to_signed(C_HUGE_SCR / 2, 12);
            s_floor_y_end <= to_signed(v_v, 12);
        end if;
    end process;

    -- ========================================================================
    -- Register Mapping
    -- ========================================================================
    s_bomb_rate_pot  <= unsigned(registers_in(0));   -- Pot 1: Bomb Rate
    s_trail_pot      <= unsigned(registers_in(1));   -- Pot 2: Trail
    s_hue_pot        <= unsigned(registers_in(2));   -- Pot 3: Hue
    s_alien_spd_pot  <= unsigned(registers_in(3));   -- Pot 4: Alien Spd
    s_bomb_spd_pot   <= unsigned(registers_in(4));   -- Pot 5: Bomb Drop Spd
    s_bullet_spd_pot <= unsigned(registers_in(5));   -- Pot 6: Shot Spd
    s_sw_fire        <= registers_in(6)(0);
    s_sw_bg_black    <= registers_in(6)(1);
    s_sw_color       <= registers_in(6)(2);
    s_sw_pause       <= registers_in(6)(3);
    s_sw_reset       <= registers_in(6)(4);
    s_ship_pos_pot   <= unsigned(registers_in(7));

    p_speeds : process(clk)
        variable v_period_raw : unsigned(5 downto 0);
        variable v_boost      : unsigned(5 downto 0);
        variable v_bul        : unsigned(4 downto 0);
        variable v_bomb       : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- Alien step period via LUT (pot=0 → 12 frames, pot≥0xC00 → 1),
            -- with a per-kill boost so the march speeds up a little with
            -- EVERY alien destroyed (saturates at 1 for the finale pace).
            v_period_raw := C_PERIOD_LUT(
                                to_integer(s_alien_spd_pot(9 downto 6)));
            v_boost := resize(s_dead_count, 6);
            if v_period_raw <= v_boost + to_unsigned(1, 6) then
                s_step_period <= to_unsigned(1, 6);
            else
                s_step_period <= v_period_raw - v_boost;
            end if;
            -- Speeds in virtual units per frame (the old HD-raster doubling
            -- of SD-scale speeds; every raster is virtual-HD now).  Shot
            -- 12..26 units/frame, bomb 4..18 — all < 32 so render-time
            -- collisions can't skip a chunk row.
            v_bul  := resize(s_bullet_spd_pot(9 downto 7), 5) + to_unsigned(6, 5);
            v_bomb := resize(s_bomb_spd_pot(9 downto 7), 5) + to_unsigned(2, 5);
            s_bullet_speed <= shift_left(v_bul, 1);
            s_bomb_speed   <= shift_left(v_bomb, 1);
            s_form_step    <= to_signed(2 * C_FORM_STEP_X, 12);
            -- K1 Bomb Rate: frames between drop attempts, 71 (pot 0, ~1.2 s)
            -- down to 8 (pot max, ~7/s — all 4 bomb slots stay busy).
            s_bomb_cd_reset <= to_unsigned(71, 8) -
                               resize(s_bomb_rate_pot(9 downto 4), 8);
            -- K2 Trail: edge-shadow rest width 4..19 px and max stretch
            -- (rest + 16)..(rest + 16 + 63) px.
            -- The win flight stretches the trail to 120 px (it eases out at
            -- the usual 3 px/frame, so it extends behind him as he takes off).
            -- K2 Trail = the trail's full length at speed, 0 (off) .. 127 px.
            s_fr_max <= s_trail_pot(9 downto 3);
        end if;
    end process;

    -- ========================================================================
    -- Video Timing Generator + Pixel Counter
    -- ========================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    pixel_counter_inst : entity work.pixel_counter
        port map (
            clk     => clk,
            timing  => s_timing,
            h_count => s_h_count,
            v_count => s_v_count
        );

    -- ========================================================================
    -- Ship X (2-stage pipeline)
    -- ========================================================================
    p_ship_mul : process(clk)
    begin
        if rising_edge(clk) then
            -- Serial shift-add: load, then 10 MSB-first steps of
            -- acc = 2*acc + (a(9) ? m : 0); publish acc >> 10 and reload.
            -- A fresh ship_x every 11 clocks — far faster than the pot moves.
            if s_sm_cnt = 0 then
                s_sm_a   <= s_ship_pos_pot;
                s_sm_m   <= resize(s_ship_x_max, 11);
                s_sm_acc <= (others => '0');
                s_sm_cnt <= s_sm_cnt + 1;
            else
                if s_sm_a(9) = '1' then
                    s_sm_acc <= shift_left(s_sm_acc, 1) + resize(s_sm_m, 21);
                else
                    s_sm_acc <= shift_left(s_sm_acc, 1);
                end if;
                s_sm_a <= shift_left(s_sm_a, 1);
                if s_sm_cnt = 10 then
                    s_sm_cnt <= (others => '0');
                else
                    s_sm_cnt <= s_sm_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    p_ship_x : process(clk)
    begin
        if rising_edge(clk) then
            -- s_sm_acc holds the finished product on the clock the counter
            -- wraps to 0 (the 10th step landed on the previous edge).
            if s_sm_cnt = 0 then
                s_ship_x <= signed(resize(s_sm_acc(20 downto 10), 12));
            end if;
        end if;
    end process;

    -- Wizard facing tracker: face the direction of travel.  With the
    -- authored sprite (un-mirrored) showing the wizard angled one way, we
    -- mirror when moving RIGHT and keep the default when moving LEFT (the
    -- opposite of the previous build, which the player found backwards).
    p_facing : process(clk)
    begin
        if rising_edge(clk) then
            s_ship_x_prev <= s_ship_x;
            if s_winning = '1' then
                s_wizard_facing <= s_win_dir;   -- face the flight direction
            elsif s_ship_x > s_ship_x_prev then
                s_wizard_facing <= '1';   -- moving right → mirrored
            elsif s_ship_x < s_ship_x_prev then
                s_wizard_facing <= '0';   -- moving left → default
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Edge-ghost animation + speed-driven band width
    -- ========================================================================
    -- Continuously track |Δship_x| (pipelined: delta then abs, registered, so
    -- no long carry chain).  Once per vsync: reload the linger timer if the
    -- ship moved, advance the hue phase, and ease the band width toward a
    -- target set by speed — fast to stretch, slow to retract.
    p_ghost : process(clk)
        variable v_abs    : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Speed pipeline (runs every clock; sampled at vsync below).
            s_ghost_dx <= s_ship_x - s_ship_x_vprev;
            if s_ghost_dx < 0 then
                v_abs := -s_ghost_dx;
            else
                v_abs := s_ghost_dx;
            end if;
            if f_slt(v_abs, 256) = '0' then
                s_ghost_spd <= to_unsigned(255, 8);
            else
                s_ghost_spd <= unsigned(v_abs(7 downto 0));
            end if;

            -- Target width = K2 max x speed fraction (see C_FRINGE_RISE), and
            -- the easing decisions — all pipelined one op per stage (the
            -- inputs only change at vsync, so the registered decisions are
            -- settled long before the next vsync samples them).
            s_gh_avg <= resize(s_ghost_spd, 9) + resize(s_spd_prev, 9);  -- 2x avg
            if f_ult(s_gh_avg, 4) = '1' then              -- avg <= 1 px
                s_gh_f <= (others => '0');
            elsif f_ult(s_gh_avg, 34) = '0' then          -- avg >= 17 px
                s_gh_f <= to_unsigned(16, 5);
            else
                s_gh_f <= resize(s_gh_avg(8 downto 1), 5) - 1;
            end if;
            s_gh_prod <= s_fr_max * s_gh_f;
            if s_winning = '1' then
                s_gh_tgt <= to_unsigned(120, 7);       -- the win flight's trail
            elsif s_dying = '1' then
                s_gh_tgt <= (others => '0');
            else
                s_gh_tgt <= s_gh_prod(10 downto 4);
            end if;
            s_gh_up   <= '0';
            s_gh_dn   <= '0';
            if s_gh_tgt > s_fringe_step then
                s_gh_up <= '1';
            elsif s_gh_tgt < s_fringe_step then
                s_gh_dn <= '1';
            end if;
            -- Would one rise/fall step reach (or pass) the target?
            if resize(s_fringe_step, 8) + C_FRINGE_RISE >= resize(s_gh_tgt, 8) then
                s_gh_snap_up <= '1';
            else
                s_gh_snap_up <= '0';
            end if;
            if resize(s_fringe_step, 8) <= resize(s_gh_tgt, 8) + C_FRINGE_FALL then
                s_gh_snap_dn <= '1';
            else
                s_gh_snap_dn <= '0';
            end if;
            s_gh_tgt_q <= s_gh_tgt;

            if s_fringe_step /= 0 then
                s_trail_on <= '1';
            else
                s_trail_on <= '0';
            end if;

            if s_timing.vsync_start = '1' then
                s_ship_x_vprev <= s_ship_x;
                s_spd_prev     <= s_ghost_spd;

                -- Ease toward the target.
                if s_gh_up = '1' then
                    if s_gh_snap_up = '1' then
                        s_fringe_step <= s_gh_tgt_q;
                    else
                        s_fringe_step <= s_fringe_step + C_FRINGE_RISE;
                    end if;
                elsif s_gh_dn = '1' then
                    if s_gh_snap_dn = '1' then
                        s_fringe_step <= s_gh_tgt_q;
                    else
                        s_fringe_step <= s_fringe_step - C_FRINGE_FALL;
                    end if;
                end if;

                -- Hue phase advances one step every few frames (slow shimmer).
                if f_ult(s_ghost_pdiv, 2) = '0' then
                    s_ghost_pdiv <= (others => '0');
                    if f_ult(s_ghost_phase, 6) = '0' then
                        s_ghost_phase <= (others => '0');
                    else
                        s_ghost_phase <= s_ghost_phase + 1;
                    end if;
                else
                    s_ghost_pdiv <= s_ghost_pdiv + 1;
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Continuous predicates
    -- ========================================================================
    p_predicates : process(clk)
        variable v_or : std_logic;
    begin
        if rising_edge(clk) then
            s_sw_fire_prev  <= s_sw_fire;
            s_sw_reset_prev <= s_sw_reset;

            -- Boot-init counter: gives v_active 8 vsyncs (≈130 ms) to
            -- settle.  Saturates at 9.  Phase 0 of the FSM uses
            -- s_boot_count = 8 as a single-shot init trigger (rather
            -- than the previous "boot_done=0 → reset every vsync" path
            -- which locked the player out of firing during boot).
            if s_timing.vsync_start = '1' and f_ult(s_boot_count, 9) = '1' then
                s_boot_count <= s_boot_count + 1;
            end if;

            if s_timing.vsync_start = '1' then
                s_fire_pending  <= '0';
                s_reset_pending <= '0';
            else
                if s_sw_fire /= s_sw_fire_prev then
                    s_fire_pending <= '1';
                end if;
                if s_sw_reset = '1' and s_sw_reset_prev = '0' then
                    s_reset_pending <= '1';
                end if;
            end if;

            for i in 0 to C_BULLETS - 1 loop
                if s_bullet_y(i) <= signed(resize(s_bullet_speed, 12)) then
                    s_bullet_at_top(i) <= '1';
                else
                    s_bullet_at_top(i) <= '0';
                end if;
            end loop;
            for i in 0 to C_BOMBS - 1 loop
                if s_bomb_y(i) >= signed(s_measured_v) then
                    s_bomb_at_bot(i) <= '1';
                else
                    s_bomb_at_bot(i) <= '0';
                end if;
            end loop;

            -- Aliens landed = the bottom of the lowest row with a live alien
            -- reaches the bottom of the bunkers (they crunch through the
            -- shields on the way down).  Live cols are 0..5 of each row.
            if s_aliens(16 to 21) /= "000000" then
                s_land_off <= to_signed(3 * C_CELL_H, 12);
            elsif s_aliens(8 to 13) /= "000000" then
                s_land_off <= to_signed(2 * C_CELL_H, 12);
            else
                s_land_off <= to_signed(C_CELL_H, 12);
            end if;
            if s_aliens(16 to 21) /= "000000" or s_aliens(8 to 13) /= "000000"
               or s_aliens(0 to 5) /= "000000" then
                s_land_any <= '1';
            else
                s_land_any <= '0';
            end if;
            s_land_bot   <= s_form_y + s_land_off;
            s_land_any_q <= s_land_any;
            if s_land_bot >= s_bunker_bot and s_land_any_q = '1' then
                s_form_at_ship <= '1';
            else
                s_form_at_ship <= '0';
            end if;

            -- aliens_clear from the kill counter — 5-bit compare vs the
            -- old 32-bit OR-reduce.
            if f_ult(s_dead_count, C_ROWS * C_COLS) = '0' then
                s_aliens_clear <= '1';
            else
                s_aliens_clear <= '0';
            end if;

            -- Pre-register reset trigger so phase 0's mux selector is
            -- a single flop read (was a 4-input OR with high fanout on
            -- the critical path).
            -- (Bomb hits and the aliens landing now play the death
            -- animation; p_story raises s_req_reset when a reset is due.)
            if s_reset_pending = '1' or s_req_reset = '1' or
               s_boot_count = to_unsigned(8, 4) then
                s_do_reset <= '1';
            else
                s_do_reset <= '0';
            end if;
        end if;
    end process;

    -- Bomb-column random source: 16-bit Galois LFSR (x^16+x^14+x^13+x^11+1),
    -- stepped 3 times per frame so each frame's 3 low bits are fresh.
    p_rng : process(clk)
        variable v_r : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            if s_ph_phase = to_unsigned(1, 3) then
                v_r := s_rng;
                for i in 0 to 2 loop
                    if v_r(0) = '1' then
                        v_r := ('0' & v_r(15 downto 1)) xor x"B400";
                    else
                        v_r := '0' & v_r(15 downto 1);
                    end if;
                end loop;
                s_rng <= v_r;
            end if;
        end if;
    end process;

    -- Bomb-spawn pick: map the random bits to a column 0..5 (first 3-bit
    -- field that is < 6, else a 2-bit fallback), then resolve its lowest
    -- live alien from s_aliens — two registered steps, so phase 4 consumes
    -- flops.  A dead column simply retries next frame with fresh bits.
    p_bomb_pick : process(clk)
        variable v_can : std_logic;
        variable v_row : unsigned(1 downto 0);
        variable v_a2, v_a1, v_a0 : std_logic;
    begin
        if rising_edge(clk) then
            if f_ult(unsigned(s_rng(2 downto 0)), C_COLS) = '1' then
                s_bsp_pick <= unsigned(s_rng(2 downto 0));
            elsif f_ult(unsigned(s_rng(5 downto 3)), C_COLS) = '1' then
                s_bsp_pick <= unsigned(s_rng(5 downto 3));
            else
                s_bsp_pick <= resize(unsigned(s_rng(7 downto 6)), 3);
            end if;

            v_can := '0';
            v_row := "00";
            -- s_aliens is packed {row, col} = row*8 + col (NOT row*6).
            -- (constant-index selects: no index arithmetic)
            v_a2 := '0';
            v_a1 := '0';
            v_a0 := '0';
            for c in 0 to 7 loop
                if s_bsp_pick = c then
                    v_a2 := s_aliens(16 + c);
                    v_a1 := s_aliens(8 + c);
                    v_a0 := s_aliens(c);
                end if;
            end loop;
            if v_a2 = '1' then
                v_row := "10"; v_can := '1';
            elsif v_a1 = '1' then
                v_row := "01"; v_can := '1';
            elsif v_a0 = '1' then
                v_row := "00"; v_can := '1';
            end if;
            s_bsp_col <= s_bsp_pick;
            s_bsp_row <= v_row;
            s_bsp_can <= v_can;
        end if;
    end process;

    -- ========================================================================
    -- Physics FSM (7 phases, vsync-triggered)
    -- ========================================================================
    p_physics : process(clk)
        variable v_spawned     : std_logic;
        variable v_x           : signed(11 downto 0);
        variable v_y           : signed(11 downto 0);
        variable v_fi          : integer range 0 to 7;
        variable v_rp1         : std_logic_vector(2 downto 0);
        variable v_cx, v_cy    : std_logic_vector(11 downto 0);
    begin
        if rising_edge(clk) then

            -- One-clock strobes to p_bunkers / p_story / p_debris / p_zoom.
            s_bnk_rst_stb   <= '0';
            s_bnk_clr_stb   <= '0';
            s_death_req     <= '0';
            s_death_end_req <= '0';
            s_win_req       <= '0';
            s_zs_go         <= '0';
            s_deb_spawn_stb <= '0';
            s_deb_step_stb  <= '0';

            -- Explosion animator: advance once every C_EXPL_FRAME_TIME
            -- vsyncs (C_CLOUD_FRAME_TIME for the 4x rainbow cloud); clear the
            -- slot at end of F3.  A spawn later in this process overrides.
            if s_timing.vsync_start = '1' and s_expl_active = '1' then
                if s_expl_timer = 0 then
                    if s_expl_frame = "11" then
                        s_expl_active <= '0';
                        s_expl_big    <= '0';
                        s_expl_huge   <= '0';
                    else
                        s_expl_frame <= s_expl_frame + 1;
                        if s_expl_huge = '1' then
                            s_expl_timer <= to_unsigned(C_HUGE_FRAME_TIME, 4);
                        elsif s_expl_big = '1' then
                            s_expl_timer <= to_unsigned(C_CLOUD_FRAME_TIME, 4);
                        else
                            s_expl_timer <= to_unsigned(C_EXPL_FRAME_TIME, 4);
                        end if;
                    end if;
                else
                    s_expl_timer <= s_expl_timer - 1;
                end if;
            end if;
            -- p_story's rainbow-cloud request (death, win launch) takes the
            -- explosion slot.
            if s_cloud_req = '1' then
                s_expl_active <= '1';
                s_expl_big    <= '1';
                s_expl_huge   <= s_cloud_huge;
                s_expl_frame  <= "00";
                if s_cloud_huge = '1' then
                    s_expl_timer <= to_unsigned(C_HUGE_FRAME_TIME, 4);
                else
                    s_expl_timer <= to_unsigned(C_CLOUD_FRAME_TIME, 4);
                end if;
                s_expl_x      <= s_cloud_x;
                s_expl_y      <= s_cloud_y;
            end if;

            case to_integer(s_ph_phase) is

                -- --------------------------------------------------
                -- Phase 0: vsync trigger.  Only latches three flags: the
                -- raw vsync_start edge must not reach the wide game-state
                -- write enables (it was the HD critical path into the 64
                -- bunker-bit enables).
                -- --------------------------------------------------
                when 0 =>
                    if s_timing.vsync_start = '1' then
                        s_ph_reset <= s_do_reset;
                        s_ph_run   <= (not s_do_reset) and (not s_sw_pause)
                                      and (not s_frozen);
                        s_ph_anim  <= (not s_do_reset) and (not s_sw_pause);
                        s_ph_fire  <= s_fire_pending;
                        s_ph_phase <= to_unsigned(1, 3);
                    end if;

                -- --------------------------------------------------
                -- Phase 1: reset (reset_pending, game_over, form_at_ship,
                -- boot), or tick the step counter + animation + bomb
                -- cooldown.  p_collide's snapshot also lands this clock, so
                -- the commits start at phase 2.
                -- --------------------------------------------------
                when 1 =>
                    if s_ph_reset = '1' then
                        s_aliens         <= (others => '1');
                        s_bnk_rst_stb    <= '1';     -- p_bunkers restores
                        s_form_x         <= s_form_x_init;
                        s_form_y         <= s_form_y_init;
                        s_form_dir       <= '1';
                        s_step_counter   <= (others => '0');
                        s_bullet_active  <= (others => '0');
                        s_bomb_active    <= (others => '0');
                        s_bomb_cooldown  <= s_bomb_cd_reset;
                        s_alien_frame    <= '0';
                        s_dead_count     <= (others => '0');
                    elsif s_ph_run = '1' then
                        if s_step_counter >= s_step_period then
                            s_step_counter <= (others => '0');
                            s_alien_frame  <= not s_alien_frame;
                        else
                            s_step_counter <= s_step_counter + 1;
                        end if;
                        if s_bomb_cooldown /= 0 then
                            s_bomb_cooldown <= s_bomb_cooldown - 1;
                        end if;
                    end if;
                    s_ph_phase <= to_unsigned(2, 3);

                -- --------------------------------------------------
                -- Phase 2: commit the frame's bullet collisions.
                -- Alien: kill it, retire the shot, bump the kill count,
                -- spawn the explosion.  Bunker: retire the shot and strobe
                -- p_bunkers to erode.  (Both may retire the same shot.)
                -- --------------------------------------------------
                when 2 =>
                    if s_ph_run = '1' then
                        if s_hit_ab_v = '1' then
                            s_aliens(to_integer(s_hit_ab_alien)) <= '0';
                            s_bullet_active(to_integer(s_hit_ab_bul)) <= '0';
                            s_bullet_y(to_integer(s_hit_ab_bul)) <=
                                to_signed(C_BULLET_PARK, 12);
                            if f_ult(s_dead_count, 31) = '1' then
                                s_dead_count <= s_dead_count + 1;
                            end if;
                            -- (No small burst over a running rainbow cloud.)
                            if s_expl_active = '0' or s_expl_big = '0' then
                                s_expl_active <= '1';
                                s_expl_big    <= '0';
                                s_expl_huge   <= '0';
                                s_expl_frame  <= "00";
                                s_expl_timer  <=
                                    to_unsigned(C_EXPL_FRAME_TIME, 4);
                                s_expl_x      <= s_hit_expl_x;
                                s_expl_y      <= s_hit_expl_y;
                            end if;
                        end if;
                        -- Chunk erosion (shot / bomb / alien) is applied by
                        -- p_bunkers next clock from its registered mask, and
                        -- p_debris drops the piece; a shot that did it retires.
                        s_bnk_clr_stb <= '1';
                        if s_hit_mb_v = '1' and s_hit_mb_isbul = '1' then
                            s_bullet_active(to_integer(s_hit_mb_bul)) <= '0';
                            s_bullet_y(to_integer(s_hit_mb_bul)) <=
                                to_signed(C_BULLET_PARK, 12);
                        end if;
                    end if;
                    s_ph_phase <= to_unsigned(3, 3);

                -- --------------------------------------------------
                -- Phase 3: commit the frame's bomb collisions — retire the
                -- bomb (its chunk erodes in p_bunkers this clock; p_debris
                -- drops the piece).  A lit-pixel wizard hit (unless he is
                -- already dying / respawning) starts the death animation,
                -- and so do the aliens landing (followed by a reset).  Bombs
                -- in flight are cleared so he can't die again on respawn.
                -- --------------------------------------------------
                when 3 =>
                    if s_ph_run = '1' then
                        s_deb_spawn_stb <= '1';
                        if s_hit_mb_v = '1' and s_hit_mb_isbomb = '1' then
                            s_bomb_active(to_integer(s_hit_mb_bomb)) <= '0';
                        end if;
                        if s_dying = '0' and
                           (s_hit_ms_v = '1' or s_form_at_ship = '1') then
                            s_death_req     <= '1';
                            s_death_end_req <= s_form_at_ship;
                            s_bomb_active   <= (others => '0');
                        end if;
                    end if;
                    s_ph_phase <= to_unsigned(4, 3);

                -- --------------------------------------------------
                -- Phase 4: projectile advance + wand-shot spawn + bomb
                -- spawn + formation x predict
                -- --------------------------------------------------
                when 4 =>
                    -- Falling bunker pieces keep falling during the death
                    -- freeze (they are animation, not game state).
                    s_deb_step_stb <= s_ph_anim;
                    if s_ph_run = '1' then
                        -- Advance each bullet; on reaching the top, deactivate
                        -- AND park it off-screen so its "at top" flag clears
                        -- before the slot is re-fired.
                        for i in 0 to C_BULLETS - 1 loop
                            if s_bullet_active(i) = '1' then
                                if s_bullet_at_top(i) = '1' then
                                    s_bullet_active(i) <= '0';
                                    s_bullet_y(i) <=
                                        to_signed(C_BULLET_PARK, 12);
                                else
                                    s_bullet_y(i) <= s_bullet_y(i) -
                                        signed(resize(s_bullet_speed, 12));
                                end if;
                            end if;
                        end loop;

                        -- Round-robin wand-shot spawn (after the advance loop
                        -- so a re-fired slot takes the new position): every
                        -- Switch-7 edge spawns at s_fire_idx, replacing the
                        -- oldest shot if all are in flight.
                        if s_ph_fire = '1' then
                            v_fi := to_integer(s_fire_idx);
                            s_bullet_active(v_fi) <= '1';
                            if s_wizard_facing = '0' then
                                s_bullet_x(v_fi) <= s_ship_x +
                                    to_signed(C_WAND_OFFSET_X, 12);
                            else
                                s_bullet_x(v_fi) <= s_ship_x +
                                    to_signed(C_SHIP_W - C_WAND_OFFSET_X
                                              - C_BULLET_W, 12);
                            end if;
                            s_bullet_y(v_fi) <= s_ship_y +
                                to_signed(C_WAND_OFFSET_Y - C_BULLET_H, 12);
                            if f_ult(s_fire_idx, C_BULLETS - 1) = '0' then
                                s_fire_idx <= (others => '0');
                            else
                                s_fire_idx <= s_fire_idx + 1;
                            end if;
                        end if;

                        -- Advance each bomb
                        for i in 0 to C_BOMBS - 1 loop
                            if s_bomb_active(i) = '1' then
                                if s_bomb_at_bot(i) = '1' then
                                    s_bomb_active(i) <= '0';
                                else
                                    s_bomb_y(i) <= s_bomb_y(i) +
                                        signed(resize(s_bomb_speed, 12));
                                end if;
                            end if;
                        end loop;

                        -- Bomb spawn from the LOWEST live alien of the
                        -- LFSR-picked column (pre-resolved in p_bomb_pick);
                        -- dead/out-of-range columns skip without consuming
                        -- the cooldown so the next pick retries.
                        if s_bomb_cooldown = 0 and s_aliens_clear = '0' then
                            if s_bsp_can = '1' then
                                -- col * 128 + 61 and (row + 1) * 128,
                                -- as bit fields (one adder each)
                                v_cx := "00" & std_logic_vector(s_bsp_col)
                                        & "0111101";
                                v_x := s_form_x + signed(v_cx);
                                v_rp1(2) := s_bsp_row(1) and s_bsp_row(0);
                                v_rp1(1) := s_bsp_row(1) xor s_bsp_row(0);
                                v_rp1(0) := not s_bsp_row(0);
                                v_cy := "00" & v_rp1 & "0000000";
                                v_y := s_form_y + signed(v_cy);
                                v_spawned := '0';
                                for i in 0 to C_BOMBS - 1 loop
                                    if v_spawned = '0' and
                                       s_bomb_active(i) = '0' then
                                        s_bomb_x(i)      <= v_x;
                                        s_bomb_y(i)      <= v_y;
                                        s_bomb_active(i) <= '1';
                                        v_spawned := '1';
                                    end if;
                                end loop;
                                s_bomb_cooldown <= s_bomb_cd_reset;
                            end if;
                        end if;

                        -- Formation x predict
                        if s_step_counter = 0 then
                            if s_form_dir = '1' then
                                s_ph_new_fx <= s_form_x + s_form_step;
                            else
                                s_ph_new_fx <= s_form_x - s_form_step;
                            end if;
                        else
                            s_ph_new_fx <= s_form_x;
                        end if;
                    else
                        s_ph_new_fx <= s_form_x;
                    end if;
                    s_ph_phase <= to_unsigned(5, 3);

                -- --------------------------------------------------
                -- Phase 5: formation commit (+ drop on bounce)
                -- --------------------------------------------------
                when 5 =>
                    if s_ph_run = '1' and s_step_counter = 0 then
                        if s_ph_new_fx >= s_form_x_max then
                            s_form_x   <= s_form_x_max;
                            s_form_dir <= '0';
                            s_form_y   <= s_form_y + to_signed(32, 12);
                        elsif f_slt(s_ph_new_fx, C_BORDER + 1) = '1' then
                            s_form_x   <= C_FORM_X_MIN;
                            s_form_dir <= '1';
                            s_form_y   <= s_form_y + to_signed(32, 12);
                        else
                            s_form_x <= s_ph_new_fx;
                        end if;
                    end if;
                    s_ph_phase <= to_unsigned(6, 3);

                -- --------------------------------------------------
                -- Phase 6: wave cleared → the win flight (p_story), which
                -- ends in a full reset (new wave, fresh bunkers).  Shots and
                -- bombs in flight are cleared.  Also kicks p_zoom to build
                -- this frame's wizard view (p_story updated it at phase 5).
                -- --------------------------------------------------
                when 6 =>
                    if s_ph_run = '1' and s_aliens_clear = '1' then
                        s_win_req       <= '1';
                        s_bullet_active <= (others => '0');
                        s_bomb_active   <= (others => '0');
                    end if;
                    s_zs_go    <= '1';
                    s_ph_phase <= to_unsigned(0, 3);

                when others =>
                    s_ph_phase <= to_unsigned(0, 3);
            end case;
        end if;
    end process;

    -- ========================================================================
    -- Render-time collision latches
    -- ========================================================================
    -- Fed from the S5 pixel flags (lit pixels only: alive alien art, alive
    -- bunker-chunk art, wizard silhouette).  First overlap of each kind per
    -- frame wins; vsync_start snapshots into s_hit_* for the FSM and clears.
    p_collide : process(clk)
        variable v_ox, v_oy : std_logic_vector(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Private registered vsync: keeps the ~40-register snapshot off
            -- the shared vsync_start net (it was a seed-3 critical path).
            s_vs_col <= s_timing.vsync_start;
            if s_vs_col = '1' then
                s_hit_ab_v     <= s_cl_ab_v;
                s_hit_ab_alien <= s_cl_ab_alien;
                s_hit_ab_bul   <= s_cl_ab_bul;
                s_hit_mb_v     <= s_cl_mb_v;
                s_hit_mb_bnk   <= s_cl_mb_bnk;
                s_hit_mb_chk   <= s_cl_mb_chk;
                s_hit_mb_bomb  <= s_cl_mb_bomb;
                s_hit_ms_v     <= s_cl_ms_v;
                s_hit_mb_isbomb <= s_cl_mb_isbomb;
                s_hit_mb_isbul  <= s_cl_mb_isbul;
                s_hit_mb_bul    <= s_cl_mb_bul;
                -- Explosion centred in the hit cell: cell origin =
                -- form + {row,col} << 7, plus C_EXPL_OFF.
                -- ({col} << 7) + 16 and ({row} << 7) + 16 as bit fields
                v_ox := "00" & std_logic_vector(s_cl_ab_alien(2 downto 0))
                        & "0010000";
                v_oy := "000" & std_logic_vector(s_cl_ab_alien(4 downto 3))
                        & "0010000";
                s_hit_expl_x <= s_form_x + signed(v_ox);
                s_hit_expl_y <= s_form_y + signed(v_oy);
                s_cl_ab_v <= '0';
                s_cl_mb_v <= '0';
                s_cl_ms_v <= '0';
            else
                if s_cl_ab_v = '0' and
                   s_stg5_on_bullet = '1' and s_stg5_on_alien = '1' then
                    s_cl_ab_v     <= '1';
                    s_cl_ab_alien <= s_stg5_alien_idx;
                    s_cl_ab_bul   <= s_stg5_bul_id;
                end if;
                if s_cl_mb_v = '0' and s_stg5_on_bunker = '1' and
                   (s_stg5_on_bullet = '1' or s_stg5_on_bomb = '1' or
                    s_stg5_on_alien = '1') then
                    s_cl_mb_v      <= '1';
                    s_cl_mb_bnk    <= s_stg5_bnk_id;
                    s_cl_mb_chk    <= s_stg5_chunk_idx;
                    s_cl_mb_bomb   <= s_stg5_bomb_id;
                    s_cl_mb_bul    <= s_stg5_bul_id;
                    s_cl_mb_isbul  <= s_stg5_on_bullet;
                    s_cl_mb_isbomb <= s_stg5_on_bomb and not s_stg5_on_bullet;
                end if;
                if s_stg5_on_bomb = '1' and s_stg5_on_ship = '1' then
                    s_cl_ms_v <= '1';
                end if;

            end if;
        end if;
    end process;

    -- ========================================================================
    -- Bunker state
    -- ========================================================================
    -- The 64 chunk bits are read all over the render path, so their write
    -- enables must be shallow: a registered one-hot clear mask (built every
    -- clock from the collision snapshot) and two registered strobes from the
    -- FSM.  Each bit is then one LUT: rst OR (bit AND NOT (stb AND clr)).
    -- (Decoding the hit indices inside the FSM phase was the HD critical
    -- path, ~11 ns of routing.)
    p_bunkers : process(clk)
    begin
        if rising_edge(clk) then
            -- the erosion uses the hit as it stood one clock before the
            -- strobe (the old registered clear mask)
            s_hit_mb_v_d   <= s_hit_mb_v;
            s_hit_mb_bnk_d <= s_hit_mb_bnk;
            s_hit_mb_chk_d <= s_hit_mb_chk;
            s_bnk_we <= "00";
            if s_bnk_rst_stb = '1' then
                s_bnk_rcnt <= (others => '0');
            elsif s_bnk_rcnt(5) = '0' then
                -- game reset: every chunk back, both RAMs together
                s_bnk_we   <= "11";
                s_bnk_wa   <= s_bnk_rcnt(4 downto 0);
                s_bnk_wd   <= '1';
                s_bnk_rcnt <= s_bnk_rcnt + 1;
            elsif s_bnk_clr_stb = '1' and s_hit_mb_v_d = '1' then
                if s_hit_mb_bnk_d(0) = '0' then s_bnk_we <= "01"; else s_bnk_we <= "10"; end if;
                s_bnk_wa <= s_hit_mb_bnk_d(1) & s_hit_mb_chk_d;
                s_bnk_wd <= '0';
            end if;
        end if;
    end process;

    p_bnk_ram_e : process(clk)
    begin
        if rising_edge(clk) then
            if s_bnk_we(0) = '1' then
                s_bnk_ram_e(to_integer(s_bnk_wa)) <= (0 => s_bnk_wd);
            end if;
            s_bnk_qe <= s_bnk_ram_e(to_integer(s_bnk_rae));
        end if;
    end process;

    p_bnk_ram_o : process(clk)
    begin
        if rising_edge(clk) then
            if s_bnk_we(1) = '1' then
                s_bnk_ram_o(to_integer(s_bnk_wa)) <= (0 => s_bnk_wd);
            end if;
            s_bnk_qo <= s_bnk_ram_o(to_integer(s_bnk_rao));
        end if;
    end process;

    -- read addresses from the stage-3 bunker flags (one bunker per RAM)
    s_bnk_rae <= '0' & s_stg3_chunk_idx_b(0) when s_stg3_in_bunker(0) = '1'
                 else '1' & s_stg3_chunk_idx_b(2);
    s_bnk_rao <= '0' & s_stg3_chunk_idx_b(1) when s_stg3_in_bunker(1) = '1'
                 else '1' & s_stg3_chunk_idx_b(3);
    s_stg4_chunk_alive <= (s_bnk_qe(0) and s_bnk_ene) or (s_bnk_qo(0) and s_bnk_eno);

    -- ========================================================================
    -- Story: death animation and win flight
    -- ========================================================================
    -- Steps once per frame (FSM phase 5, unless paused).  Requests arrive as
    -- one-clock strobes from p_physics; a whole-game reset clears everything.
    -- Outputs are registered frame flags (they change only in vblank) plus
    -- the wizard view (centre + zoom k) that p_zoom turns into DDA starts.
    p_story : process(clk)
        variable v_shx   : signed(4 downto 0);
        variable v_shy   : signed(4 downto 0);
        variable v_dx    : signed(15 downto 0);
        variable v_dy    : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            s_cloud_req <= '0';

            -- Frame counter for the rainbow animations (mod 8).
            if s_ph_phase = to_unsigned(5, 3) and s_ph_anim = '1' then
                s_phase8 <= s_phase8 + 1;
                -- alien TV screens: next frame every C_SCR_FRAME_TIME vsyncs
                if s_scr_div = to_unsigned(C_SCR_FRAME_TIME - 1, 3) then
                    s_scr_div <= (others => '0');
                    if s_scr_anim = to_unsigned(C_SCR_FRAMES - 1, 3) then
                        s_scr_anim <= (others => '0');
                    else
                        s_scr_anim <= s_scr_anim + 1;
                    end if;
                else
                    s_scr_div <= s_scr_div + 1;
                end if;
            end if;

            if s_bnk_rst_stb = '1' then
                s_dying     <= '0';
                s_winning   <= '0';
                s_req_reset <= '0';
                s_end_reset <= '0';
                s_dt        <= (others => '0');
                s_wt        <= (others => '0');
            elsif s_win_req = '1' then
                -- Launch: rainbow puff where he stands, then fly toward the
                -- side with more room, rising and growing.
                s_winning <= '1';
                s_dying   <= '0';
                s_wt      <= (others => '0');
                s_fx      <= shift_left(resize(s_ship_x + 64, 16), 4);
                s_fy      <= shift_left(resize(s_ship_y + 64, 16), 4);
                s_fvx     <= (others => '0');
                s_fvy     <= to_signed(-32, 12);           -- -2 px/frame
                s_zk      <= to_unsigned(256, 9);
                if s_ship_x + 64 < signed('0' & s_measured_h(11 downto 1)) then
                    s_win_dir <= '1';
                else
                    s_win_dir <= '0';
                end if;
                s_cloud_req <= '1';
            elsif s_death_req = '1' then
                -- Death starts where he stands, at 1x.
                s_dying     <= '1';
                s_dt        <= (others => '0');
                s_end_reset <= s_death_end_req;
                s_fx        <= shift_left(resize(s_ship_x + 64, 16), 4);
                s_fy        <= shift_left(resize(s_ship_y + 64, 16), 4);
                s_fvx       <= (others => '0');
                s_fvy       <= (others => '0');
                s_zk        <= to_unsigned(256, 9);
            elsif s_ph_phase = to_unsigned(5, 3) and s_ph_anim = '1' then
                if s_dying = '1' then
                    s_dt <= s_dt + 1;
                    if f_ult(s_dt, C_D_CLOUD) = '1' then
                        -- Ease to the screen centre: velocity = 1/8 of the
                        -- remaining distance (one frame late, which still
                        -- settles smoothly), while growing to C_DEATH_K.
                        -- (screen centre in 12.4 = (size/2)*16 = size*8)
                        v_dx := signed(shift_left(resize(s_measured_h, 16), 3))
                                - s_fx;
                        v_dy := signed(shift_left(resize(s_measured_v, 16), 3))
                                - s_fy;
                        s_fvx <= resize(shift_right(v_dx, 3), 12);
                        s_fvy <= resize(shift_right(v_dy, 3), 12);
                        s_fx  <= s_fx + resize(s_fvx, 16);
                        s_fy  <= s_fy + resize(s_fvy, 16);
                        if f_ult(s_zk, C_DEATH_K + 9) = '0' then
                            s_zk <= s_zk - 8;
                        else
                            s_zk <= to_unsigned(C_DEATH_K, 9);
                        end if;
                    end if;
                    if s_dt = to_unsigned(C_D_CLOUD - 1, 9) then
                        s_cloud_req <= '1';               -- the big one
                    end if;
                    if s_dt = to_unsigned(C_D_END - 1, 9) then
                        s_dying <= '0';
                        if s_end_reset = '1' then
                            s_req_reset <= '1';
                        end if;
                    end if;
                end if;
                if s_winning = '1' then
                    if s_wt /= to_unsigned(255, 8) then
                        s_wt <= s_wt + 1;
                    end if;
                    if f_ult(s_wt, C_W_FLY) = '1' then
                        -- accelerate sideways (3/16 px/frame^2) toward the
                        -- far side and up (6/16).  Over W_FLY frames that is
                        -- <= 947 px sideways from a start on the near half
                        -- and ~2090 px up from the bottom, so the 12.4
                        -- positions never leave +-2048 px (no saturation).
                        if s_win_dir = '1' then
                            s_fvx <= s_fvx + 3;
                        else
                            s_fvx <= s_fvx - 3;
                        end if;
                        s_fvy <= s_fvy - 6;
                        s_fx  <= s_fx + resize(s_fvx, 16);
                        s_fy  <= s_fy + resize(s_fvy, 16);
                        -- grow: k 256 → 40 (6.4x) at 3 per frame
                        if f_ult(s_zk, 44) = '0' then
                            s_zk <= s_zk - 3;
                        else
                            s_zk <= to_unsigned(40, 9);
                        end if;
                    end if;
                    if s_wt = to_unsigned(C_W_END - 1, 8) then
                        s_req_reset <= '1';
                    end if;
                end if;
            end if;

            -- Cloud spot: death = the 384-px cloud at screen centre (where the
            -- enlarged wizard is); win launch = the 192-px cloud centred on
            -- the 128-px sprite.
            s_cloud_huge <= s_dying;
            if s_dying = '1' then
                s_cloud_x <= s_huge_x;
                s_cloud_y <= s_huge_y;
            else
                s_cloud_x <= s_ship_x - to_signed(32, 12);
                s_cloud_y <= s_ship_y - to_signed(32, 12);
            end if;

            -- Registered frame flags.
            -- (a landing death stays frozen until its reset)
            if s_winning = '1' or
               (s_dying = '1' and (f_ult(s_dt, C_D_RESUME) = '1' or
                                   s_end_reset = '1')) then
                s_frozen <= '1';
            else
                s_frozen <= '0';
            end if;
            -- Death dissolve schedule (see C_D_TV_IN / C_D_TV_OUT).
            if s_dying = '0' then
                s_world_level <= to_unsigned(16, 5);
                s_tv_level    <= (others => '0');
            elsif f_ult(s_dt, 16) = '1' then                  -- world out
                s_world_level <= to_unsigned(16, 5) - resize(s_dt(3 downto 0), 5);
                s_tv_level    <= (others => '0');
            elsif f_ult(s_dt, C_D_TV_IN) = '1' then
                s_world_level <= (others => '0');
                s_tv_level    <= (others => '0');
            elsif f_ult(s_dt, C_D_TV_IN + 16) = '1' then      -- TV in
                s_world_level <= (others => '0');
                s_tv_level    <= resize(s_dt(3 downto 0), 5) + 1;
            elsif f_ult(s_dt, C_D_TV_OUT) = '1' then          -- TV up
                s_world_level <= (others => '0');
                s_tv_level    <= to_unsigned(16, 5);
            elsif f_ult(s_dt, C_D_TV_OUT + 16) = '1' then     -- crossfade
                s_world_level <= resize(s_dt(3 downto 0), 5) + 1;
                s_tv_level    <= to_unsigned(15, 5) - resize(s_dt(3 downto 0), 5);
            else
                s_world_level <= to_unsigned(16, 5);
                s_tv_level    <= (others => '0');
            end if;

            -- rainbow strobe = the charge-up at the end of the rumble
            if s_dying = '1' and f_ult(s_dt, C_D_STROBE) = '0' and
               f_ult(s_dt, C_D_CLOUD) = '1' then
                s_wiz_strobe <= '1';
            else
                s_wiz_strobe <= '0';
            end if;
            if s_winning = '1' then
                if f_ult(s_wt, C_W_FLY) = '1' then
                    s_wiz_vis <= '1';
                else
                    s_wiz_vis <= '0';
                end if;
            elsif s_dying = '1' then
                if f_ult(s_dt, C_D_CLOUD) = '1' then
                    s_wiz_vis <= '1';                       -- fly + rumble
                elsif f_ult(s_dt, C_D_RESUME) = '1' then
                    s_wiz_vis <= '0';                       -- inside the cloud
                else
                    s_wiz_vis <= not s_dt(2);               -- respawn blink
                end if;
            else
                s_wiz_vis <= '1';
            end if;

            -- Wizard view for this frame: win = the flight centre and zoom;
            -- dying (before the cloud) = the eased centre and zoom plus the
            -- rumble shake (random per frame, +-2 → +-16 px as it builds);
            -- otherwise 1x at the ship.
            v_shx := signed(s_rng(4 downto 0));
            v_shy := signed(s_rng(9 downto 5));
            -- (rumble D_FLY .. D_CLOUD-1, 15 frames per level; the TV holds
            -- still once the explosion starts)
            if f_ult(s_dt, C_D_FLY) = '1' or
               f_ult(s_dt, C_D_CLOUD) = '0' then
                v_shx := (others => '0');
                v_shy := (others => '0');
            elsif f_ult(s_dt, C_D_FLY + 15) = '1' then
                v_shx := shift_right(v_shx, 3);
                v_shy := shift_right(v_shy, 3);
            elsif f_ult(s_dt, C_D_FLY + 30) = '1' then
                v_shx := shift_right(v_shx, 2);
                v_shy := shift_right(v_shy, 2);
            elsif f_ult(s_dt, C_D_FLY + 45) = '1' then
                v_shx := shift_right(v_shx, 1);
                v_shy := shift_right(v_shy, 1);
            end if;
            if s_winning = '1' then
                s_zoom_xc <= s_fx(15 downto 4);
                s_zoom_yc <= s_fy(15 downto 4);
                s_zoom_k  <= s_zk;
            elsif s_dying = '1' and f_ult(s_dt, C_D_TV_OUT + 16) = '1' then
                -- (held until the TV has fully dissolved — the wizard
                -- itself is hidden from D_CLOUD)
                s_zoom_xc <= s_fx(15 downto 4) + resize(v_shx, 12);
                s_zoom_yc <= s_fy(15 downto 4) + resize(v_shy, 12);
                s_zoom_k  <= s_zk;
            else
                s_zoom_xc <= s_ship_x + to_signed(64, 12);
                s_zoom_yc <= s_ship_y + to_signed(64, 12);
                s_zoom_k  <= to_unsigned(256, 9);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Wizard view → DDA starts (once per frame, serial multiply)
    -- ========================================================================
    -- u_start = 64*256 - xc*k, v_start = 64*256 - yc*k (9 MSB-first
    -- shift-add steps over k), then load v_acc.  Runs a few clocks after
    -- vsync, long before the first active line.
    p_zoom : process(clk)
    begin
        if rising_edge(clk) then
            s_v_load <= '0';
            -- cnt 20..12: u pass (op = xc), 11: store u_start, load yc;
            -- 10..2: v pass, 1: store v_start + k_frame, load v_acc.
            if s_zs_go = '1' then
                s_zm_op  <= s_zoom_xc;
                s_zm_k   <= s_zoom_k;
                s_zm_ksh <= s_zoom_k;
                s_zm_acc <= (others => '0');
                s_zm_cnt <= to_unsigned(20, 5);
            elsif s_zm_cnt = 11 then
                s_u_start <= to_signed(16384, 20) - s_zm_acc;
                s_zm_op   <= s_zoom_yc;
                s_zm_ksh  <= s_zm_k;
                s_zm_acc  <= (others => '0');
                s_zm_cnt  <= s_zm_cnt - 1;
            elsif s_zm_cnt = 1 then
                s_v_start <= to_signed(16384, 20) - s_zm_acc;
                s_k_frame <= s_zm_k;
                s_v_load  <= '1';
                s_zm_cnt  <= (others => '0');
            elsif s_zm_cnt /= 0 then
                if s_zm_ksh(8) = '1' then
                    s_zm_acc <= shift_left(s_zm_acc, 1) + resize(s_zm_op, 20);
                else
                    s_zm_acc <= shift_left(s_zm_acc, 1);
                end if;
                s_zm_ksh <= shift_left(s_zm_ksh, 1);
                s_zm_cnt <= s_zm_cnt - 1;
            end if;
        end if;
    end process;

    -- Sprite-space DDA, mirroring the virtual counters exactly: u = u_start +
    -- vx*k, v = v_start + vy*k.  (u restarts every line, so analog
    -- line-length wobble can't accumulate.)  vx steps by 1 or 2 per pixel
    -- (2 at SD, alternating at 720p: when the half bit is set), so u adds k
    -- or 2k.  vy steps by 1..5 per line: one k is added at avid_start as
    -- before, and the extra (step - 1) k's are added AHEAD, on the clocks
    -- after the previous line's avid_end (s_vrep, at most 4 at 480i), so
    -- every pixel of a line sees the finished v.  The first line's extras and
    -- a bottom field's start (vy 0..2 units down) are added the same way
    -- after the vblank load.  At 1080p every step is 1 and this is the old
    -- DDA.
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                s_u_acc <= s_u_start;
            elsif s_timing.avid = '1' then
                if (s_xs(2) or s_xa(0)) = '1' then
                    s_u_acc <= s_u_acc + signed(shift_left(resize(s_k_frame, 20), 1));
                else
                    s_u_acc <= s_u_acc + signed(resize(s_k_frame, 20));
                end if;
            end if;
            if s_v_load = '1' then
                s_v_acc <= s_v_start;
                s_vrep  <= s_vrep_ld + s_ya(9 downto 8);
            elsif s_timing.avid_start = '1' then
                s_v_acc <= s_v_acc + signed(resize(s_k_frame, 20));
            elsif s_timing.avid_end = '1' then
                s_vrep  <= s_vrep_ld;
            elsif s_vrep /= 0 then
                s_v_acc <= s_v_acc + signed(resize(s_k_frame, 20));
                s_vrep  <= s_vrep - 1;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Falling bunker pieces
    -- ========================================================================
    -- Each chunk knocked out (by a shot, a bomb or a crunching alien) drops
    -- as a 32×32 piece of the bunker art: a small pop up, then gravity
    -- (1 px/frame^2) until it leaves the bottom of the screen.
    p_debris : process(clk)
        variable v_slot  : integer range 0 to C_DEBRIS;
        variable v_ny    : signed(11 downto 0);
        variable v_off   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- chunk top = bunker_y - 16 + chunk_row * 32: one adder, the
            -- row offset a 4-way constant
            case s_hit_mb_chk(3 downto 2) is
                when "00"   => v_off := to_signed(0 * 32 - C_CHUNK_ROW_OFF, 12);
                when "01"   => v_off := to_signed(1 * 32 - C_CHUNK_ROW_OFF, 12);
                when "10"   => v_off := to_signed(2 * 32 - C_CHUNK_ROW_OFF, 12);
                when others => v_off := to_signed(3 * 32 - C_CHUNK_ROW_OFF, 12);
            end case;
            s_dsp_mb_y <= s_bunker_y + v_off;

            if s_bnk_rst_stb = '1' then
                s_deb_a <= (others => '0');
            elsif s_deb_spawn_stb = '1' then
                -- first free slot (a full pool just skips the piece; the
                -- chunk is still removed).  Shots, bombs and alien crunches.
                v_slot := C_DEBRIS;
                if s_hit_mb_v = '1' then
                    for i in C_DEBRIS - 1 downto 0 loop
                        if s_deb_a(i) = '0' then
                            v_slot := i;
                        end if;
                    end loop;
                    if v_slot < C_DEBRIS then
                        s_deb_a(v_slot)   <= '1';
                        s_deb_y(v_slot)   <= s_dsp_mb_y;
                        s_deb_vy(v_slot)  <= to_signed(-4, 6);
                        s_deb_chk(v_slot) <= s_hit_mb_chk;
                        s_deb_bnk(v_slot) <= s_hit_mb_bnk;
                    end if;
                end if;
            elsif s_deb_step_stb = '1' then
                for i in 0 to C_DEBRIS - 1 loop
                    if s_deb_a(i) = '1' then
                        v_ny := s_deb_y(i) + resize(s_deb_vy(i), 12);
                        s_deb_y(i) <= v_ny;
                        if f_slt(s_deb_vy(i), 30) = '1' then
                            s_deb_vy(i) <= s_deb_vy(i) + 1;
                        end if;
                        if v_ny >= signed(s_measured_v) then
                            s_deb_a(i) <= '0';
                        end if;
                    end if;
                end loop;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Render — S1: register coords
    -- ========================================================================
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            -- virtual coordinates (= h_count / v_count at 1080p)
            s_stg1_hx <= signed(s_vx);
            s_stg1_vy <= signed(s_vy);
            -- integer sprite px of the DDA (|u|,|v| < 2048 by construction)
            s_stg1_u  <= s_u_acc(19 downto 8);
            s_stg1_v  <= s_v_acc(19 downto 8);
        end if;
    end process;

    -- ========================================================================
    -- Render — S2: per-region deltas (live state reads)
    -- ========================================================================
    p_stage2 : process(clk)
        variable v_b : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            s_stg2_form_dx <= s_stg1_hx - s_form_x;
            s_stg2_form_dy <= s_stg1_vy - s_form_y;
            -- Wizard: sprite-space coords straight from the DDA (= hx - ship_x
            -- at 1x; scaled about the flight centre during the win).
            s_stg2_ship_dx <= s_stg1_u;
            s_stg2_ship_dy <= s_stg1_v;
            -- death-sequence TV, in wizard sprite space (see S3)
            s_stg2_tv_u    <= s_stg1_u + to_signed(44, 12);
            s_stg2_tv_v    <= s_stg1_v + to_signed(72, 12);
            -- per-pixel dissolve / crossfade decisions (any fixed pixel
            -- phase of the 4x4 Bayer cell is fine; REAL pixels, so the
            -- dither stays at pixel scale in every mode)
            v_b := C_BAYER4(to_integer(s_v_count(1 downto 0) & s_h_count(1 downto 0)));
            if v_b < s_tv_level then
                s_stg2_use_tv <= '1';
            else
                s_stg2_use_tv <= '0';
            end if;
            if resize(not v_b(3 downto 0), 5) < s_world_level then
                s_stg2_wshow <= '1';
            else
                s_stg2_wshow <= '0';
            end if;
            for i in 0 to C_DEBRIS - 1 loop
                s_stg2_deb_dy(i) <= s_stg1_vy - s_deb_y(i);
            end loop;
            s_stg2_deb_a    <= s_deb_a;
            s_stg2_expl_big  <= s_expl_big;
            s_stg2_expl_huge <= s_expl_huge;
            for i in 0 to C_BULLETS - 1 loop
                s_stg2_bull_dx(i) <= s_stg1_hx - s_bullet_x(i);
                s_stg2_bull_dy(i) <= s_stg1_vy - s_bullet_y(i);
            end loop;
            for i in 0 to C_BOMBS - 1 loop
                s_stg2_bomb_dx(i) <= s_stg1_hx - s_bomb_x(i);
                s_stg2_bomb_dy(i) <= s_stg1_vy - s_bomb_y(i);
            end loop;
            for b in 0 to C_BUNKER_COUNT - 1 loop
                s_stg2_bnk_dx(b) <= s_stg1_hx - s_bunker_x(b);
            end loop;
            s_stg2_bnk_dy <= s_stg1_vy - s_bunker_y;
            s_stg2_expl_dx <= s_stg1_hx - s_expl_x;
            s_stg2_expl_dy <= s_stg1_vy - s_expl_y;
            s_stg2_expl_a  <= s_expl_active;
            s_stg2_expl_fr <= s_expl_frame;
            s_stg2_frame  <= s_alien_frame;
            s_stg2_bull_a <= s_bullet_active;
            s_stg2_bomb_a <= s_bomb_active;
        end if;
    end process;

    -- ========================================================================
    -- Render — S3: in-bbox flags + indices + ROM word addresses
    -- ========================================================================
    -- Every sprite ROM holds 16-pixel words (see C_WIZARD_ROM): the word
    -- address takes the column's high bits and S5 picks bit
    -- "not col(3 downto 0)" of the word.  Mirroring (wizard facing right) is
    -- col → 127 - col = bitwise NOT, applied here to the address and index.
    p_stage3 : process(clk)
        variable v_ship_in_x   : std_logic;
        variable v_ship_in_y   : std_logic;
        variable v_trail_in_x  : std_logic;
        variable v_in_x        : std_logic;
        variable v_in_y        : std_logic;
        variable v_chunk_idx   : unsigned(3 downto 0);
        variable v_bnk_col_use : unsigned(6 downto 0);
        variable v_bnk_in_y    : std_logic;
        variable v_bnk_chrow   : unsigned(7 downto 0);
        variable v_row_bits    : unsigned(1 downto 0);
        variable v_col_bits    : unsigned(2 downto 0);
        variable v_bnk_id      : unsigned(1 downto 0);
        variable v_dx          : signed(9 downto 0);
        variable v_lim_x       : std_logic;
        variable v_lim_y       : std_logic;
        variable v_er          : unsigned(5 downto 0);
        variable v_ec          : unsigned(5 downto 0);
        variable v_deb_in      : std_logic;
        variable v_deb_addr    : unsigned(9 downto 0);
        variable v_deb_bit     : unsigned(3 downto 0);
        variable v_bdx         : signed(11 downto 0);
        variable v_sr          : unsigned(5 downto 0);
        variable v_sc          : unsigned(5 downto 0);
        variable v_sf          : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- Power-of-two (128) cells: column = form_dx[9:7], row =
            -- form_dy[8:7], in-cell coordinate = the low 7 bits.  The sprite
            -- fills the cell, so "in grid" == "on sprite".  The in-grid range
            -- tests use the registered S2 deltas (no duplicate subtractors).
            -- The alien sprite path draws EITHER the formation or (during the
            -- death sequence, while the formation is faded out) the one big TV
            -- around the enlarged wizard.  Formation: power-of-two 128 cells,
            -- column = form_dx[9:7], row = form_dy[8:7], source px = in-cell
            -- coord >> 1.  TV: drawn in the wizard's sprite space at 4x his
            -- scale with its screen centre (src 27, 34) on his centre (64,
            -- 64): src = (u + 44) >> 2, (v + 72) >> 2 (S2 adds), in range
            -- when the high bits are zero.
            if s_stg2_use_tv = '1' then
                v_col_bits := (others => '0');
                v_row_bits := (others => '0');
                if s_stg2_tv_u(11 downto 8) = "0000" and
                   s_stg2_tv_v(11 downto 8) = "0000" then
                    s_stg3_in_form <= '1';
                else
                    s_stg3_in_form <= '0';
                end if;
                v_sr := unsigned(s_stg2_tv_v(7 downto 2));
                v_sc := unsigned(s_stg2_tv_u(7 downto 2));
            else
                v_col_bits := unsigned(s_stg2_form_dx(9 downto 7));
                v_row_bits := unsigned(s_stg2_form_dy(8 downto 7));
                if s_stg2_form_dx(11 downto 10) = 0 and f_ult(unsigned(s_stg2_form_dx(9 downto 0)), C_FORM_W) = '1' and
                   s_stg2_form_dy(11 downto 9) = 0 and f_ult(unsigned(s_stg2_form_dy(8 downto 0)), C_FORM_H) = '1' then
                    s_stg3_in_form <= '1';
                else
                    s_stg3_in_form <= '0';
                end if;
                v_sr := unsigned(s_stg2_form_dy(6 downto 1));
                v_sc := unsigned(s_stg2_form_dx(6 downto 1));
            end if;
            s_stg3_tv    <= s_stg2_use_tv;
            s_stg3_dith  <= s_stg2_use_tv or s_stg2_wshow;
            s_stg3_wshow <= s_stg2_wshow;

            -- alien_idx = {row(1..0), col(2..0)} → 5 bits
            s_stg3_alien_idx <= v_row_bits & v_col_bits;

            -- Word address = {frame, src_row(5..0), src_col(5..4)}.
            s_stg3_alien_addr <= s_stg2_frame & v_sr & v_sc(5 downto 4);
            s_stg3_alien_bit  <= not v_sc(3 downto 0);
            -- TV screen interior: drawn from the waveform animation, at this
            -- alien's own frame offset ((anim + offset) mod 5).
            if f_ult(v_sr, C_SCR_R0) = '0' and f_ult(v_sr, C_SCR_R1 + 1) = '1' and
               f_ult(v_sc, C_SCR_C0) = '0' and f_ult(v_sc, C_SCR_C1 + 1) = '1' then
                s_stg3_in_scr <= '1';
            else
                s_stg3_in_scr <= '0';
            end if;
            v_sf := resize(s_scr_anim, 4) +
                    resize(C_SCR_OFF(to_integer(v_row_bits & v_col_bits)), 4);
            if f_ult(v_sf, C_SCR_FRAMES) = '0' then
                v_sf := v_sf - C_SCR_FRAMES;
            end if;
            v_sr := v_sr - C_SCR_R0;
            v_sc := v_sc - C_SCR_C0;
            s_stg3_scr_addr <= v_sf(2 downto 0) & v_sr(4 downto 0) & v_sc(4 downto 4);
            s_stg3_scr_bit  <= not v_sc(3 downto 0);

            -- Ship (sprite box) and trail box (sprite box widened by
            -- C_TRAIL_EXT each side).
            if s_stg2_ship_dx(11 downto 7) = 0 then
                v_ship_in_x := '1';
            else
                v_ship_in_x := '0';
            end if;
            if f_slt(s_stg2_ship_dx, -C_TRAIL_EXT) = '0' and
               f_slt(s_stg2_ship_dx, C_SHIP_W + C_TRAIL_EXT) = '1' then
                v_trail_in_x := '1';
            else
                v_trail_in_x := '0';
            end if;
            if s_stg2_ship_dy(11 downto 7) = 0 then
                v_ship_in_y := '1';
            else
                v_ship_in_y := '0';
            end if;
            s_stg3_in_ship  <= v_ship_in_x and v_ship_in_y;
            s_stg3_in_trail <= v_trail_in_x and v_ship_in_y;
            s_stg3_grp_row  <= unsigned(s_stg2_ship_dy(6 downto 0));
            v_dx := s_stg2_ship_dx(9 downto 0);
            if s_wizard_facing = '0' then
                -- as authored: sprite col = dx; row-vector index = 127 - dx
                s_stg3_wiz_addr <= unsigned(s_stg2_ship_dy(6 downto 0)) &
                                   unsigned(s_stg2_ship_dx(6 downto 4));
                s_stg3_wiz_bit  <= not unsigned(s_stg2_ship_dx(3 downto 0));
                s_stg3_gh_base  <= to_signed(127, 10) - v_dx;
            else
                -- mirrored: sprite col = 127 - dx; row-vector index = dx
                s_stg3_wiz_addr <= unsigned(s_stg2_ship_dy(6 downto 0)) &
                                   not unsigned(s_stg2_ship_dx(6 downto 4));
                s_stg3_wiz_bit  <= unsigned(s_stg2_ship_dx(3 downto 0));
                s_stg3_gh_base  <= v_dx;
            end if;
            s_stg3_hue_col <= unsigned(s_stg2_ship_dx(5 downto 3));

            -- Bullets (each WAND_SHOT 8x16)
            for i in 0 to C_BULLETS - 1 loop
                if s_stg2_bull_dx(i)(11 downto 3) = 0 then
                    v_in_x := '1';
                else
                    v_in_x := '0';
                end if;
                if s_stg2_bull_dy(i)(11 downto 4) = 0 then
                    v_in_y := '1';
                else
                    v_in_y := '0';
                end if;
                s_stg3_in_bullet(i) <= v_in_x and v_in_y and s_stg2_bull_a(i);
            end loop;

            -- Bombs (rectangles)
            for i in 0 to C_BOMBS - 1 loop
                if s_stg2_bomb_dx(i)(11 downto 3) = 0 and f_ult(unsigned(s_stg2_bomb_dx(i)(2 downto 0)), C_BOMB_W) = '1' then
                    v_in_x := '1';
                else
                    v_in_x := '0';
                end if;
                if s_stg2_bomb_dy(i)(11 downto 4) = 0 and f_ult(unsigned(s_stg2_bomb_dy(i)(3 downto 0)), C_BOMB_H) = '1' then
                    v_in_y := '1';
                else
                    v_in_y := '0';
                end if;
                s_stg3_in_bomb(i) <= v_in_x and v_in_y and s_stg2_bomb_a(i);
            end loop;

            -- Bunkers (4 in parallel, shared dy)
            if s_stg2_bnk_dy(11 downto 7) = 0 and f_ult(unsigned(s_stg2_bnk_dy(6 downto 0)), C_BUNKER_H) = '1' then
                v_bnk_in_y := '1';
            else
                v_bnk_in_y := '0';
            end if;
            v_chunk_idx   := (others => '0');
            v_bnk_col_use := (others => '0');
            v_bnk_id      := (others => '0');
            for b in 0 to C_BUNKER_COUNT - 1 loop
                if s_stg2_bnk_dx(b)(11 downto 7) = 0 then
                    v_in_x := '1';
                else
                    v_in_x := '0';
                end if;
                if v_in_x = '1' and v_bnk_in_y = '1' then
                    s_stg3_in_bunker(b) <= '1';
                    -- chunk 32 × 32: col = dx(6..5); row = (dy+OFF)(6..5) so
                    -- the half-height partial row sits at the top, not bottom.
                    v_bnk_chrow   := unsigned(s_stg2_bnk_dy(7 downto 0)) +
                                     to_unsigned(C_CHUNK_ROW_OFF, 8);
                    v_chunk_idx   := v_bnk_chrow(6 downto 5) &
                                     unsigned(s_stg2_bnk_dx(b)(6 downto 5));
                    v_bnk_col_use := unsigned(s_stg2_bnk_dx(b)(6 downto 0));
                    v_bnk_id      := to_unsigned(b, 2);
                else
                    s_stg3_in_bunker(b) <= '0';
                end if;
            end loop;
            s_stg3_chunk_idx <= v_chunk_idx;
            for b in 0 to C_BUNKER_COUNT - 1 loop
                s_stg3_chunk_idx_b(b) <= v_chunk_idx;
            end loop;
            s_stg3_bnk_id    <= v_bnk_id;
            s_stg3_bnk_addr  <= unsigned(s_stg2_bnk_dy(6 downto 0)) &
                                v_bnk_col_use(6 downto 4);
            s_stg3_bnk_bit   <= not v_bnk_col_use(3 downto 0);

            -- Explosion (48×48 art) drawn 2× = 96 px for an alien, or 4× =
            -- 192 px for the rainbow cloud.  Word address = {frame(1..0),
            -- src_row(5..0), src_col(5..4)}; source row/col = screen offset
            -- >> 1 (or >> 2).  The cloud also gets |src - 24| per axis for
            -- its diamond rainbow rings.
            if s_stg2_expl_huge = '1' then
                v_lim_x  := f_slt(s_stg2_expl_dx, C_HUGE_SCR);
                v_lim_y  := f_slt(s_stg2_expl_dy, C_HUGE_SCR);
                v_er     := unsigned(s_stg2_expl_dy(8 downto 3));
                v_ec     := unsigned(s_stg2_expl_dx(8 downto 3));
            elsif s_stg2_expl_big = '1' then
                v_lim_x  := f_slt(s_stg2_expl_dx, C_CLOUD_SCR);
                v_lim_y  := f_slt(s_stg2_expl_dy, C_CLOUD_SCR);
                v_er     := unsigned(s_stg2_expl_dy(7 downto 2));
                v_ec     := unsigned(s_stg2_expl_dx(7 downto 2));
            else
                v_lim_x  := f_slt(s_stg2_expl_dx, C_EXPL_SCR);
                v_lim_y  := f_slt(s_stg2_expl_dy, C_EXPL_SCR);
                v_er     := unsigned(s_stg2_expl_dy(6 downto 1));
                v_ec     := unsigned(s_stg2_expl_dx(6 downto 1));
            end if;
            if s_stg2_expl_dx(11) = '0' and v_lim_x = '1' then
                v_in_x := '1';
            else
                v_in_x := '0';
            end if;
            if s_stg2_expl_dy(11) = '0' and v_lim_y = '1' then
                v_in_y := '1';
            else
                v_in_y := '0';
            end if;
            s_stg3_in_expl   <= v_in_x and v_in_y and s_stg2_expl_a;
            s_stg3_in_cloud  <= v_in_x and v_in_y and s_stg2_expl_a and
                                s_stg2_expl_big;
            s_stg3_expl_addr <= s_stg2_expl_fr & v_er & v_ec(5 downto 4);
            s_stg3_expl_bit  <= not v_ec(3 downto 0);
            if f_ult(v_er, 24) = '0' then
                s_stg3_cl_ar <= resize(v_er - 24, 5);
            else
                s_stg3_cl_ar <= resize(24 - v_er, 5);
            end if;
            if f_ult(v_ec, 24) = '0' then
                s_stg3_cl_ac <= resize(v_ec - 24, 5);
            else
                s_stg3_cl_ac <= resize(24 - v_ec, 5);
            end if;

            -- Falling bunker pieces (32×32): in-box is a zero test of the
            -- delta's high bits; the first covering slot drives the debris
            -- ROM address {chunk, row(4..0), col(4)}.
            v_deb_in   := '0';
            v_deb_addr := (others => '0');
            v_deb_bit  := (others => '0');
            -- Column test from the piece's bunker delta: inside the bunker's
            -- 128-px span AND in the piece's 32-px chunk column.
            for i in C_DEBRIS - 1 downto 0 loop
                v_bdx := s_stg2_bnk_dx(to_integer(s_deb_bnk(i)));
                if s_stg2_deb_a(i) = '1' and
                   v_bdx(11 downto 7) = "00000" and
                   unsigned(v_bdx(6 downto 5)) = s_deb_chk(i)(1 downto 0) and
                   s_stg2_deb_dy(i)(11 downto 5) = "0000000" then
                    v_deb_in   := '1';
                    v_deb_addr := s_deb_chk(i) &
                                  unsigned(s_stg2_deb_dy(i)(4 downto 0)) &
                                  unsigned(v_bdx(4 downto 4));
                    v_deb_bit  := not unsigned(v_bdx(3 downto 0));
                end if;
            end loop;
            s_stg3_in_deb   <= v_deb_in;
            s_stg3_deb_addr <= v_deb_addr;
            s_stg3_deb_bit  <= v_deb_bit;
        end if;
    end process;

    -- ========================================================================
    -- Render — S4: sync ROM word reads + state lookups
    -- ========================================================================
    p_stage4 : process(clk)
        variable v_alv       : std_logic_vector(C_ALIEN_SLOTS - 1 downto 0);
        variable v_chunk_hit : std_logic;
        variable v_any_bnk   : std_logic;
        variable v_tap       : signed(9 downto 0);
        variable v_base_c    : signed(9 downto 0);
        variable v_tap_c     : signed(9 downto 0);
        variable v_ring      : unsigned(5 downto 0);
        variable v_hue       : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- ROM word reads (BRAM-inferred)
            s_stg4_alien_word  <= C_ALIEN_ROM(to_integer(s_stg3_alien_addr));
            s_stg4_scr_word    <= C_SCREEN_ROM(to_integer(s_stg3_scr_addr));
            s_stg4_in_scr      <= s_stg3_in_scr;
            s_stg4_scr_bit     <= s_stg3_scr_bit;
            s_stg4_wiz_word    <= C_WIZARD_ROM(to_integer(s_stg3_wiz_addr));
            s_stg4_eye_word    <= C_EYE_ROM(to_integer(s_stg3_wiz_addr));
            s_stg4_grp         <= C_GRP_ROM(to_integer(s_stg3_grp_row));
            s_stg4_bunker_word <= C_BUNKER_ROM(to_integer(s_stg3_bnk_addr));
            s_stg4_expl_word   <= C_EXPL_ROM(to_integer(s_stg3_expl_addr));
            s_stg4_deb_word    <= C_DEBRIS_ROM(to_integer(s_stg3_deb_addr));
            s_stg4_in_deb      <= s_stg3_in_deb;
            s_stg4_deb_bit     <= s_stg3_deb_bit;
            s_stg4_in_cloud    <= s_stg3_in_cloud;

            -- Rainbow palette index (index 7 = red again, so mod-8 wraps
            -- cleanly): cloud = diamond rings flowing outward; electrified
            -- wizard = 16-px bands racing up the sprite; otherwise the
            -- edge-shadow's left-to-right hue (mod 7, as before).
            if s_stg3_in_cloud = '1' then
                v_ring := resize(s_stg3_cl_ar, 6) + resize(s_stg3_cl_ac, 6);
                s_stg4_cidx <= v_ring(5 downto 3) - s_phase8;
            elsif s_wiz_strobe = '1' then
                s_stg4_cidx <= s_stg3_grp_row(6 downto 4) + s_phase8;
            else
                v_hue := resize(s_stg3_hue_col, 4) + resize(s_ghost_phase, 4);
                if f_ult(v_hue, 7) = '0' then
                    v_hue := v_hue - 7;
                end if;
                s_stg4_cidx <= v_hue(2 downto 0);
            end if;

            -- Alien-alive lookup (32-to-1)
            for a in 0 to C_ALIEN_SLOTS - 1 loop
                v_alv(a) := s_aliens(a);
            end loop;
            s_stg4_alien_alive <= v_alv(to_integer(s_stg3_alien_idx)) or s_stg3_tv;
            -- Dissolve decisions made in S2 (TV pixel, or a world pixel
            -- that is showing).
            s_stg4_dith  <= s_stg3_dith;
            s_stg4_wshow <= s_stg3_wshow;

            -- Bunker chunk-alive: the chunk RAMs (p_bunkers) answer at this
            -- stage; the enables say which RAM's answer counts
            v_any_bnk := '0';
            for b in 0 to C_BUNKER_COUNT - 1 loop
                if s_stg3_in_bunker(b) = '1' then
                    v_any_bnk := '1';
                end if;
            end loop;
            s_bnk_ene <= s_stg3_in_bunker(0) or s_stg3_in_bunker(2);
            s_bnk_eno <= s_stg3_in_bunker(1) or s_stg3_in_bunker(3);
            s_stg4_in_bunker_any <= v_any_bnk;
            s_stg4_bnk_bit       <= s_stg3_bnk_bit;

            s_stg4_in_expl   <= s_stg3_in_expl;
            s_stg4_expl_bit  <= s_stg3_expl_bit;
            s_stg4_in_form   <= s_stg3_in_form;
            s_stg4_alien_bit <= s_stg3_alien_bit;
            s_stg4_in_ship   <= s_stg3_in_ship;
            s_stg4_in_trail  <= s_stg3_in_trail;
            s_stg4_wiz_bit   <= s_stg3_wiz_bit;
            s_stg4_in_bullet <= s_stg3_in_bullet;
            s_stg4_in_bomb   <= s_stg3_in_bomb;
            -- Collision indices ride along to S5.
            s_stg4_alien_idx <= s_stg3_alien_idx;
            s_stg4_chunk_idx <= s_stg3_chunk_idx;
            s_stg4_bnk_id    <= s_stg3_bnk_id;
            -- Cellophane band of this row (s_stg1_vy is a few pixels ahead
            -- on the same line; rows only change in blanking).
            if s_stg1_vy < s_band_top_y then
                s_stg4_band <= "00";
            elsif s_stg1_vy >= s_bunker_y then
                s_stg4_band <= "10";
            else
                s_stg4_band <= "01";
            end if;
            -- Edge-shadow group indices: tap = base + band width; both
            -- clamped to -4 .. 128 then floor-divided by 4 → -1 .. 32 (-1 =
            -- outside the row on the trailing side, 32 = past the far end).
            -- (bit-logic clamps: f_slt(x, 129) = '0' is x > 128)
            v_tap := s_stg3_gh_base + signed(resize(s_fringe_step, 10));
            if f_slt(s_stg3_gh_base, -4) = '1' then
                v_base_c := to_signed(-4, 10);
            elsif f_slt(s_stg3_gh_base, 129) = '0' then
                v_base_c := to_signed(128, 10);
            else
                v_base_c := s_stg3_gh_base;
            end if;
            if f_slt(v_tap, -4) = '1' then
                v_tap_c := to_signed(-4, 10);
            elsif f_slt(v_tap, 129) = '0' then
                v_tap_c := to_signed(128, 10);
            else
                v_tap_c := v_tap;
            end if;
            s_stg4_gh_base_g <= resize(shift_right(v_base_c, 2), 7);
            s_stg4_gh_tap_g  <= resize(shift_right(v_tap_c, 2), 7);
        end if;
    end process;

    -- ========================================================================
    -- Render — S5: bit selects + per-region combine
    -- ========================================================================
    p_stage5 : process(clk)
        variable v_alien_bit  : std_logic;
        variable v_wizard_bit : std_logic;
        variable v_bunker_bit : std_logic;
        variable v_expl_bit   : std_logic;
        variable v_on_bullet  : std_logic;
        variable v_on_bomb    : std_logic;
        variable v_pfx_b      : std_logic;
        variable v_pfx_t      : std_logic;
        variable v_win        : std_logic;
        variable v_lt_b       : std_logic_vector(31 downto 0);
        variable v_le_t       : std_logic_vector(31 downto 0);
        variable v_t_b        : std_logic_vector(31 downto 0);
        variable v_t_t        : std_logic_vector(31 downto 0);
        variable v_t_w        : std_logic_vector(31 downto 0);
        variable v_eyegap     : std_logic;
        variable v_bul_id     : unsigned(2 downto 0);
        variable v_bomb_id    : unsigned(1 downto 0);
        variable v_band       : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            -- 16:1 bit selects out of the registered ROM words.
            if s_stg4_in_scr = '1' then
                v_alien_bit := s_stg4_scr_word(to_integer(s_stg4_scr_bit));
            else
                v_alien_bit := s_stg4_alien_word(to_integer(s_stg4_alien_bit));
            end if;
            v_wizard_bit := s_stg4_wiz_word(to_integer(s_stg4_wiz_bit));
            v_bunker_bit := s_stg4_bunker_word(to_integer(s_stg4_bnk_bit));
            v_expl_bit   := s_stg4_expl_word(to_integer(s_stg4_expl_bit));

            s_stg5_on_alien  <= s_stg4_in_form and s_stg4_dith
                                and s_stg4_alien_alive and v_alien_bit;
            -- (hidden inside the cloud, blinking on respawn, gone after the
            -- win flight leaves the screen)
            s_stg5_on_ship   <= s_stg4_in_ship and v_wizard_bit and s_wiz_vis;
            -- Eye/gap ROM bit: on a lit pixel = eyeball (S6 only reads it
            -- under on_ship); on an unlit one = trail gap (below).
            v_eyegap := s_stg4_eye_word(to_integer(s_stg4_wiz_bit));
            s_stg5_on_eye    <= v_eyegap;
            s_stg5_on_bunker <= s_stg4_in_bunker_any and
                                s_stg4_chunk_alive and v_bunker_bit;
            s_stg5_on_expl   <= s_stg4_in_expl and v_expl_bit and
                                not s_stg4_in_cloud;
            s_stg5_on_cloud  <= s_stg4_in_cloud and v_expl_bit;
            s_stg5_wshow     <= s_stg4_wshow;
            s_stg5_on_debris <= s_stg4_in_deb and
                                s_stg4_deb_word(to_integer(s_stg4_deb_bit));

            -- Wand-shots and bombs render as solid rectangles.
            -- The id of the (lowest-numbered) shot/bomb covering the pixel
            -- feeds the collision latches.
            v_on_bullet := '0';
            v_bul_id    := (others => '0');
            for i in C_BULLETS - 1 downto 0 loop
                if s_stg4_in_bullet(i) = '1' then
                    v_on_bullet := '1';
                    v_bul_id    := to_unsigned(i, 3);
                end if;
            end loop;
            s_stg5_on_bullet <= v_on_bullet;
            s_stg5_bul_id    <= v_bul_id;

            v_on_bomb := '0';
            v_bomb_id := (others => '0');
            for i in C_BOMBS - 1 downto 0 loop
                if s_stg4_in_bomb(i) = '1' then
                    v_on_bomb := '1';
                    v_bomb_id := to_unsigned(i, 2);
                end if;
            end loop;
            s_stg5_on_bomb <= v_on_bomb;
            s_stg5_bomb_id <= v_bomb_id;

            s_stg5_alien_idx <= s_stg4_alien_idx;
            s_stg5_chunk_idx <= s_stg4_chunk_idx;
            s_stg5_bnk_id    <= s_stg4_bnk_id;

            -- Foreground colour for this row's cellophane band.
            v_band := to_integer(s_stg4_band);
            if v_band > 2 then
                v_band := 1;
            end if;
            s_stg5_fg_y <= s_band_y(v_band);
            s_stg5_fg_u <= s_band_u(v_band);
            s_stg5_fg_v <= s_band_v(v_band);

            -- Floor line: full-width horizontal stripe under the wizard.
            if s_stg1_vy >= s_floor_y and s_stg1_vy < s_floor_y_end then
                s_stg5_on_floor <= '1';
            else
                s_stg5_on_floor <= '0';
            end if;

            -- Solid edge-shadow via a prefix-OR of the silhouette's 4-px
            -- groups (precomputed in C_GRP_ROM).  v_pfx_b = any body group
            -- strictly BEFORE this pixel's group (pixel not yet past the
            -- trailing edge); v_pfx_t = any body group within the band toward
            -- the body.  Shadow = trailing margin AND body within reach → a
            -- SOLID block hugging the edge, always touching the wizard.  The
            -- group indices run -1 .. 32, so the shadow continues past the
            -- sprite box (trail box) instead of being clipped flat.
            -- Plus the trail gaps (outside pockets between body parts, e.g.
            -- under the raised arm): a gap pixel is shadow when a body group
            -- lies within the band toward the body (the window base..tap),
            -- so the trail "emits" from every trailing-facing surface.  The
            -- art's linework is never a gap pixel (see WIZARD_GAP_MASK).
            -- Independent per-group terms, each reduced by ONE balanced OR (an
            -- if/elsif accumulation loop synthesised as a ~10-LUT ripple).
            -- Thermometers as bit logic: g < base is "not base < g + 1", and
            -- g <= tap is "not tap < g" (a 7-bit carry-chain compare per
            -- group was 64 chains).
            for g in 0 to 31 loop
                v_lt_b(g) := not f_slt(s_stg4_gh_base_g, g + 1);
                v_le_t(g) := not f_slt(s_stg4_gh_tap_g, g);
            end loop;
            v_t_b := s_stg4_grp and v_lt_b;
            v_t_t := s_stg4_grp and v_le_t;
            v_t_w := s_stg4_grp and v_le_t and not v_lt_b;
            if v_t_b /= x"00000000" then v_pfx_b := '1'; else v_pfx_b := '0'; end if;
            if v_t_t /= x"00000000" then v_pfx_t := '1'; else v_pfx_t := '0'; end if;
            if v_t_w /= x"00000000" then v_win   := '1'; else v_win   := '0'; end if;
            -- (outer trail and gap fill are registered separately and ORed
            -- in S6 to keep this stage short)
            -- Hue is a left-to-right rainbow (by screen column) that scrolls
            -- with the animation phase.
            -- (palette index resolved in S4: shadow hue / cloud rings /
            -- electrified bands)
            s_stg5_tr_out <= s_stg4_in_trail and v_pfx_t and (not v_pfx_b)
                             and s_wiz_vis and s_trail_on;
            s_stg5_tr_gap <= s_stg4_in_ship and (not v_wizard_bit) and v_eyegap
                             and v_win and s_wiz_vis and s_trail_on;
            s_stg5_trail_u  <= C_TRAIL_PAL_U(to_integer(s_stg4_cidx));
            s_stg5_trail_v  <= C_TRAIL_PAL_V(to_integer(s_stg4_cidx));
        end if;
    end process;

    -- ========================================================================
    -- Render — S6: color mux
    -- ========================================================================
    -- Band colours from K3 Hue and S9 Cellophane, one shared wheel-LUT read
    -- per clock, round-robin over the 3 bands (frame-constant in practice).
    -- Solid: every band = wheel(h) (h = 0 is white).  Cellophane: top =
    -- wheel(h+1) (red at h = 0), middle = wheel(h), bottom = wheel(h+6)
    -- (green at h = 0) — K3 rotates the whole overlay.
    p_fg_color : process(clk)
        variable v_h : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            v_h := s_hue_pot(9 downto 6);
            if s_sw_color = '1' and s_band_k = 0 then
                s_band_idx_q <= v_h + 1;
            elsif s_sw_color = '1' and s_band_k = 2 then
                s_band_idx_q <= v_h + 6;
            else
                s_band_idx_q <= v_h;
            end if;
            s_band_kq <= s_band_k;
            if s_band_k = 2 then
                s_band_k <= 0;
            else
                s_band_k <= s_band_k + 1;
            end if;
            s_band_y(s_band_kq) <= C_HUE_Y(to_integer(s_band_idx_q));
            s_band_u(s_band_kq) <= C_HUE_U(to_integer(s_band_idx_q));
            s_band_v(s_band_kq) <= C_HUE_V(to_integer(s_band_idx_q));
        end if;
    end process;

    p_stage6 : process(clk)
        variable v_fg  : std_logic;
    begin
        if rising_edge(clk) then
            -- Foreground sprites/lines in the band colour (everything except
            -- the wizard and the edge-shadow).
            -- (dissolved with the aliens during the death sequence)
            v_fg := (s_stg5_on_bullet or s_stg5_on_bomb or
                     s_stg5_on_bunker or s_stg5_on_expl or s_stg5_on_floor or
                     s_stg5_on_debris) and s_stg5_wshow;
            s_out_show <= v_fg or s_stg5_on_alien or s_stg5_on_ship or
                          s_stg5_on_cloud or s_stg5_tr_out or s_stg5_tr_gap;

            -- Layering, top first: the wizard (fixed purple + white eyeball,
            -- or rainbow bands while electrified) — he sits inside the death
            -- TV, and a fresh shot emerges from behind his wand hand; the
            -- rainbow cloud (over the TV it blows up in); the aliens / death
            -- TV, always in their own middle-band colour (also down in the
            -- green shield band, and in front of the shields they crunch);
            -- band-coloured shots, bombs, shields, pieces, bursts, floor;
            -- then the faded rainbow edge-shadow.
            if s_stg5_on_ship = '1' and s_wiz_strobe = '1' then
                s_out_y <= to_unsigned(C_STROBE_Y, 10);
                s_out_u <= s_stg5_trail_u;
                s_out_v <= s_stg5_trail_v;
            elsif s_stg5_on_ship = '1' then
                if s_stg5_on_eye = '1' then
                    s_out_y <= to_unsigned(C_EYE_Y, 10);
                    s_out_u <= C_CHROMA_MID;
                    s_out_v <= C_CHROMA_MID;
                else
                    s_out_y <= to_unsigned(C_WIZ_Y, 10);
                    s_out_u <= to_unsigned(C_WIZ_U, 10);
                    s_out_v <= to_unsigned(C_WIZ_V, 10);
                end if;
            elsif s_stg5_on_cloud = '1' then
                s_out_y <= to_unsigned(C_CLOUD_Y, 10);
                s_out_u <= s_stg5_trail_u;
                s_out_v <= s_stg5_trail_v;
            elsif s_stg5_on_alien = '1' then
                s_out_y <= s_band_y(1);
                s_out_u <= s_band_u(1);
                s_out_v <= s_band_v(1);
            elsif v_fg = '1' then
                s_out_y <= s_stg5_fg_y;
                s_out_u <= s_stg5_fg_u;
                s_out_v <= s_stg5_fg_v;
            else
                s_out_y <= to_unsigned(C_GHOST_LUMA, 10);
                s_out_u <= s_stg5_trail_u;
                s_out_v <= s_stg5_trail_v;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Sync delay + early background-video tap
    -- ========================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_pipe(0) <= data_in.field_n & data_in.avid &
                              data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_DELAY - 1 loop
                s_sync_pipe(i) <= s_sync_pipe(i - 1);
            end loop;

            -- Background video, 2 clocks deep, blacked outside its own avid
            -- (so the 6 px it leads the sync never shows input blanking).
            s_vid_y0 <= data_in.y;
            s_vid_u0 <= data_in.u;
            s_vid_v0 <= data_in.v;
            if s_sync_pipe(0)(2) = '1' then
                s_vid_y <= s_vid_y0;
                s_vid_u <= s_vid_u0;
                s_vid_v <= s_vid_v0;
            else
                s_vid_y <= std_logic_vector(to_unsigned(C_BLACK_Y, 10));
                s_vid_u <= std_logic_vector(C_CHROMA_MID);
                s_vid_v <= std_logic_vector(C_CHROMA_MID);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Wet pixel mux
    -- ========================================================================
    -- Blanking gate: outside the latency-aligned avid the output is forced
    -- to neutral black (sprites at h_count/v_count = 0 and the full-width
    -- floor line would otherwise draw into the blanking intervals).  Sits in
    -- front of the first IO register.
    s_out_avid <= s_sync_pipe(C_DELAY - 1)(2);

    p_wet_select : process(s_out_show, s_sw_bg_black, s_out_avid,
                           s_out_y, s_out_u, s_out_v,
                           s_vid_y, s_vid_u, s_vid_v)
    begin
        if s_out_avid = '0' then
            s_wet_y <= std_logic_vector(to_unsigned(C_BLACK_Y, 10));
            s_wet_u <= std_logic_vector(C_CHROMA_MID);
            s_wet_v <= std_logic_vector(C_CHROMA_MID);
        elsif s_out_show = '1' then
            s_wet_y <= std_logic_vector(s_out_y);
            s_wet_u <= std_logic_vector(s_out_u);
            s_wet_v <= std_logic_vector(s_out_v);
        elsif s_sw_bg_black = '1' then
            s_wet_y <= std_logic_vector(to_unsigned(C_BLACK_Y, 10));
            s_wet_u <= std_logic_vector(C_CHROMA_MID);
            s_wet_v <= std_logic_vector(C_CHROMA_MID);
        else
            s_wet_y <= s_vid_y;
            s_wet_u <= s_vid_u;
            s_wet_v <= s_vid_v;
        end if;
    end process;

    -- ========================================================================
    -- IO alignment
    -- ========================================================================
    -- The wet pixel goes straight to the output (no wet/dry interpolation —
    -- mix was hardcoded fully wet).  Sync is tapped at s_sync_pipe(C_DELAY-1),
    -- matching the wet pixel's 8-clock render depth.  Bit 2 of the sync word
    -- is avid (field_n & avid & vsync_n & hsync_n).
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io_0.y       <= s_wet_y;
            s_io_0.u       <= s_wet_u;
            s_io_0.v       <= s_wet_v;
            s_io_0.hsync_n <= s_sync_pipe(C_DELAY - 1)(0);
            s_io_0.vsync_n <= s_sync_pipe(C_DELAY - 1)(1);
            s_io_0.avid    <= s_sync_pipe(C_DELAY - 1)(2);
            s_io_0.field_n <= s_sync_pipe(C_DELAY - 1)(3);
            s_io_1 <= s_io_0;
        end if;
    end process;

    s_io_align_in.y       <= s_io_1.y;
    s_io_align_in.u       <= s_io_1.u;
    s_io_align_in.v       <= s_io_1.v;
    s_io_align_in.hsync_n <= s_io_1.hsync_n;
    s_io_align_in.vsync_n <= s_io_1.vsync_n;
    s_io_align_in.avid    <= s_io_1.avid;
    s_io_align_in.field_n <= s_io_1.field_n;

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

    data_out <= s_io_align_pad(1);

end architecture invaders;
