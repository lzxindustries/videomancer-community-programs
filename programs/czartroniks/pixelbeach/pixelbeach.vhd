-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Pixel Beach v3 "Shorebreak": side-profile 8-bit ocean, real physics.
--
-- The screen is a cross-section of a beach: sky and blocky sun above, ocean
-- blue wedge in the middle, sand below-left.  Every visible cell is ONE solid
-- colour (16x16 px, or 8x8 via S8) -- no dithering, no sub-cell texture.
--
-- The water is not an oscillator stack: each cell-column carries a surface
-- height eta and a momentum u in BRAM, and a vblank FSM integrates the 1D
-- shallow-water equations every frame:
--
--   u[i]   += (eta[i] - eta[i+1]) >> 3          gravity on the surface slope
--   u[i]   -= u[i] >> fr                        friction (stronger when shallow)
--   flux_i  = u[i] * depth_upwind               momentum transports water
--   eta[i] += flux[i-1] - flux[i]               exact conservation (each
--                                               interface flux quantized once)
--   eta[i] += df[i-1] - df[i]                   pairwise diffusion (kills the
--                                               odd-even checkerboard mode)
--
-- Because wave speed rises with depth, the swell forced at the right edge
-- steepens as the bottom shoals toward the beach, topples, runs up the sand,
-- and drains back to sea on its own momentum -- crash and backwash are not
-- animated, they fall out of the physics.  Verified against the integer
-- Python prototype (beach_proto.py): stable, mass-conserving, breaks on the
-- slope.
--
-- Foam is a per-column quantity: injected where a steep leftward-moving front
-- crosses shallow water, advected with u, decayed every tick.  Spray cells
-- pop up one row above the surface on a per-column hash while foam is high --
-- each crash tip breaks a little differently.  Sand darkens where the wash
-- has covered it: every sand cell keeps its own wetness counter in BRAM, set
-- to full the frame water is on it and drying back over ~3 s once it leaves.
--
-- Controls (bold, always-on):
--   K1 Rhythm  tempo of both the waves and the slow surge cycle
--   K2 Tide    water level +-4 rows: walks the shoreline up and down the sand
--   K3 Foam    foam injection strength
--   K4 Chop    secondary short-period wave + per-column crest jitter
--   K5 Swell   sea energy: wave amplitude AND surge depth.  Max is the
--              designed climax: storm faces walling up and dumping.
--   K6 Time    physics speed: 1-4 physics ticks per field
--   S7 Gulls   three low-res gulls flapping across the sky
--   (stars)    once the sun is down (or S10 Night) a sparse field of one-cell
--              stars fills the upper sky, each twinkling on its own phase
--   S8 Pixels  16 px / 8 px cells
--   S9 Storm   +50% wave energy, hotter foam injection
--   S10 Sky    Day / Night
--   S11 Crab   a red crab wanders the whole sand area below the beach line,
--              pausing, sometimes walking off the left edge; when the wash
--              comes down over the crest near it, it runs away down across the
--              sand.  Off = it sinks into a 1-row hole (one row per frame),
--              On = a hole opens somewhere new and it climbs out top row
--              first; holes fade like wet sand
--   P12 Sun    sun (or moon, S10) elevation: slider rides it through the sky;
--              in Day the palette follows it down Day -> Golden -> Sunset.
--
-- Swell noise (v3.2): a 16-bit LFSR re-rolls each wave's height (+-40%) at
-- every wave-phase wrap and jitters the phase rate, so no two waves match.
-- A third slow oscillator (ph3, K1-paced) is the SURGE: it raises the forced
-- boundary level so water piles up against the beach, then lets it collapse
-- back out to the right; while the surge is high the individual waves are
-- damped (big swell = lower waves), giving the in/out breathing rhythm.
--
-- Structure:
--   p_position  raster counters, cell lattice, interlace detect, n_cols/rows
--   p_vseq      per-frame render params + gulls + FSM kick     (vblank)
--   p_fsm       physics: PREP -> V pass -> F pass -> boundary  (vblank)
--   p_disp      per-cell state prefetch + derived geometry     (active video)
--   p_pix       P1 diffs -> P2 classify -> P3 palette -> P4 out (per pixel)
--
-- State BRAMs (256 x 16 each, canonical 1W1R, FSM writes in vblank only, read
-- address muxed FSM/display by fsm_run): eta, u, foam & wet packed.
-- One shared 13x10 multiplier (registered in/out) serves flux and swell.
--
-- v3.24.2 (2026-10-18): resolution scaling.  The cell lattice runs in virtual
-- 1080-line units (x step 1/1.5/2 per pixel, y step per field line from the
-- timing ID), so a 16 px cell is 16/1080 of the screen height in every mode:
-- 720p cells are 10-11 px, SD cells 8 px x 3-4 field lines (square on 4:3),
-- and every cell-based layout (sun, gulls, rows, beach) scales with them.
-- The display prefetch needs >= 8 clocks per cell, so the 8 px mode is
-- clamped to 8 px at 720p (12 units, 1.5x) and SD (16 units: the 16 px
-- mode's size there; S8 then only changes the beach slope and wash tail).
-- The old field_n interlace detect (dead on hardware) is replaced by the
-- interlaced y steps.  1080p is unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture pixelbeach of program_top is
    -- Synthesis program: processing_delay_clks is declared 0 in the TOML (the SDK
    -- allows 0 for synthesis programs, and it saves the IO-align pad registers).
    -- The true data_in -> data_out sync latency is 5 clocks.
    constant C_PROCESSING_DELAY_CLKS : natural := 0;

    constant C_DW  : integer := C_VIDEO_DATA_WIDTH;   -- 10
    constant C_LAT : integer := 5;

    ----------------------------------------------------------------------------
    -- Geometry constants (heights in Q6: 64 units = one cell-row)
    ----------------------------------------------------------------------------
    constant C_B0     : signed(11 downto 0) := to_signed(192, 12);  -- beach top +3 rows
    constant C_DCLAMP : signed(11 downto 0) := to_signed(768, 12);  -- depth cap 12 rows
    constant C_UCLAMP : signed(11 downto 0) := to_signed(255, 12);

    ----------------------------------------------------------------------------
    -- Palette ROM: addr = pal(2b) & idx(5b); word = Y(10) & U(10) & V(10).
    -- Stored U/V-SWAPPED (U slot = Cr, V slot = Cb): HW-confirmed 2026-07-18 --
    -- the standard-order palette rendered the ocean orange and the sand light
    -- blue (same convention mondrian needed).  Generated by genpal.py.
    ----------------------------------------------------------------------------
    constant C_I_SKYHI : unsigned(4 downto 0) := to_unsigned( 0, 5);
    constant C_I_SKYLO : unsigned(4 downto 0) := to_unsigned( 1, 5);
    constant C_I_SUN   : unsigned(4 downto 0) := to_unsigned( 2, 5);
    constant C_I_SAND  : unsigned(4 downto 0) := to_unsigned( 3, 5);
    constant C_I_SANDW : unsigned(4 downto 0) := to_unsigned( 4, 5);
    constant C_I_SURF  : unsigned(4 downto 0) := to_unsigned( 5, 5);
    constant C_I_AQUA  : unsigned(4 downto 0) := to_unsigned( 6, 5);
    constant C_I_MID   : unsigned(4 downto 0) := to_unsigned( 7, 5);
    constant C_I_DEEP  : unsigned(4 downto 0) := to_unsigned( 8, 5);
    constant C_I_FOAM  : unsigned(4 downto 0) := to_unsigned( 9, 5);
    constant C_I_GULL  : unsigned(4 downto 0) := to_unsigned(10, 5);
    constant C_I_SAND2 : unsigned(4 downto 0) := to_unsigned(11, 5);
    constant C_I_SANDM : unsigned(4 downto 0) := to_unsigned(12, 5);
    constant C_I_GULL2 : unsigned(4 downto 0) := to_unsigned(13, 5);
    constant C_I_STAR  : unsigned(4 downto 0) := to_unsigned(14, 5);
    constant C_I_STARD : unsigned(4 downto 0) := to_unsigned(15, 5);
    constant C_I_CRAB  : unsigned(4 downto 0) := to_unsigned(16, 5);
    constant C_I_CRABD : unsigned(4 downto 0) := to_unsigned(17, 5);

    -- Wash-crest foam mask: a hand-placed, aperiodic 64-column pattern of
    -- scattered dots + short (2-col) runs, ~23% coverage.  A ROM (not an XOR
    -- hash) because any bit-XOR of x is periodic and tiles a visible
    -- "dot dot space" motif; this has no algebraic structure so the foam looks
    -- random and doesn't repeat across the ~80-col screen.  '1' = foam here.
    constant FOAM_MASK : std_logic_vector(0 to 63) :=
        "00110000" & "01000100" & "00010000" & "11000001" &
        "00001000" & "00100010" & "10000100" & "00011001";

    type t_pal is array (0 to 127) of std_logic_vector(29 downto 0);
    constant PAL_ROM : t_pal := (
        -- 0: Day (U/V swapped for hardware)
          0 => "101000001101010111101010100001",  -- SKYHI
          1 => "110011101101100100101001001011",  -- SKYLO
          2 => "110110111010010111000100001101",  -- SUN
          3 => "110010110110010100100101111010",  -- SAND
          4 => "100101000010010100000110001010",  -- SANDW
          5 => "101110101001010000111000110001",  -- SURF
          6 => "100100001101000000111001110100",  -- AQUA
          7 => "011000101001010000101011101110",  -- MID
          8 => "001110111101100000111011010000",  -- DEEP
          9 => "111110101101111110001000000011",  -- FOAM
         10 => "001111010101111111001000011000",  -- GULL
         11 => "101110100110010101010101101010",  -- SAND2
         12 => "101100111010010100100101111110",  -- SANDM
         13 => "111011111101111110111000001001",  -- GULL2
         14 => "111010110110000011010111011101",  -- STAR
         15 => "100001100101111000111001010101",  -- STARD
         16 => "011011010011010010000101100100",  -- CRAB
         17 => "001101001110110000000110111111",  -- CRABD
         18 => "000000000000000000000000000000",
         19 => "000000000000000000000000000000",
         20 => "000000000000000000000000000000",
         21 => "000000000000000000000000000000",
         22 => "000000000000000000000000000000",
         23 => "000000000000000000000000000000",
         24 => "000000000000000000000000000000",
         25 => "000000000000000000000000000000",
         26 => "000000000000000000000000000000",
         27 => "000000000000000000000000000000",
         28 => "000000000000000000000000000000",
         29 => "000000000000000000000000000000",
         30 => "000000000000000000000000000000",
         31 => "000000000000000000000000000000",
        -- 1: Golden (U/V swapped for hardware)
         32 => "101011100110101001010101010000",  -- SKYHI
         33 => "110100111110011001110101010101",  -- SKYLO
         34 => "110010010010100110100011001011",  -- SUN
         35 => "110001001110011001010101100101",  -- SAND
         36 => "100010011110010101100110010000",  -- SANDW
         37 => "101111001001100110000111110110",  -- SURF
         38 => "100010110101010000001001000001",  -- AQUA
         39 => "010111101001011110111010100110",  -- MID
         40 => "001101011101101010111010010110",  -- DEEP
         41 => "111101000010000101100111010011",  -- FOAM
         42 => "001110000010000101111000000000",  -- GULL
         43 => "101101001110011001010101100101",  -- SAND2
         44 => "101000110110010111100101111010",  -- SANDM
         45 => "111010110110000011010111101001",  -- GULL2
         46 => "111010110110000011010111011101",  -- STAR
         47 => "100001100101111000111001010101",  -- STARD
         48 => "011110000111001101000101010100",  -- CRAB
         49 => "001110101010101110110110110010",  -- CRABD
         50 => "000000000000000000000000000000",
         51 => "000000000000000000000000000000",
         52 => "000000000000000000000000000000",
         53 => "000000000000000000000000000000",
         54 => "000000000000000000000000000000",
         55 => "000000000000000000000000000000",
         56 => "000000000000000000000000000000",
         57 => "000000000000000000000000000000",
         58 => "000000000000000000000000000000",
         59 => "000000000000000000000000000000",
         60 => "000000000000000000000000000000",
         61 => "000000000000000000000000000000",
         62 => "000000000000000000000000000000",
         63 => "000000000000000000000000000000",
        -- 2: Sunset (U/V swapped for hardware)
         64 => "010110011110010101101001111010",  -- SKYHI
         65 => "100110010110111110000101111111",  -- SKYLO
         66 => "100110000111001001010100010101",  -- SUN
         67 => "101001101010011000000110000101",  -- SAND
         68 => "011011111010010100010110110000",  -- SANDW
         69 => "100110101110001001101000110000",  -- SURF
         70 => "010110110001100001101001100110",  -- AQUA
         71 => "010010111001110010011010011011",  -- MID
         72 => "001010101001110010111010001011",  -- DEEP
         73 => "111001100110001100110111100000",  -- FOAM
         74 => "001011000110000010111000011011",  -- GULL
         75 => "100110000010010110110110001010",  -- SAND2
         76 => "100001110010010110000110011010",  -- SANDM
         77 => "110110111010000111100111110110",  -- GULL2
         78 => "111010110110000011010111011101",  -- STAR
         79 => "100001100101111000111001010101",  -- STARD
         80 => "010111011011000110010110011010",  -- CRAB
         81 => "001011111010100101100111011001",  -- CRABD
         82 => "000000000000000000000000000000",
         83 => "000000000000000000000000000000",
         84 => "000000000000000000000000000000",
         85 => "000000000000000000000000000000",
         86 => "000000000000000000000000000000",
         87 => "000000000000000000000000000000",
         88 => "000000000000000000000000000000",
         89 => "000000000000000000000000000000",
         90 => "000000000000000000000000000000",
         91 => "000000000000000000000000000000",
         92 => "000000000000000000000000000000",
         93 => "000000000000000000000000000000",
         94 => "000000000000000000000000000000",
         95 => "000000000000000000000000000000",
        -- 3: Night (U/V swapped for hardware)
         96 => "000110100101111000111001010101",  -- SKYHI
         97 => "001110001101110100001001101011",  -- SKYLO
         98 => "111011101001111011011000010101",  -- SUN
         99 => "011000010001111111011000010000",  -- SAND
        100 => "010000010101111111001000011000",  -- SANDW
        101 => "100011011101011111011000111011",  -- SURF
        102 => "010110110001100001101001100110",  -- AQUA
        103 => "001110011001101000001001111011",  -- MID
        104 => "000111011001110000111001100000",  -- DEEP
        105 => "111001000101110111011000011011",  -- FOAM
        106 => "001000011101111110111000100000",  -- GULL
        107 => "010110010001111111011000010000",  -- SAND2
        108 => "010011010101111111011000010100",  -- SANDM
        109 => "110011101101111011011000011001",  -- GULL2
        110 => "111010110110000011010111011101",  -- STAR
        111 => "100001100101111000111001010101",  -- STARD
        112 => "010000011110100001000111101010",  -- CRAB
        113 => "001000011010010000001000000001",  -- CRABD
        114 => "000000000000000000000000000000",
        115 => "000000000000000000000000000000",
        116 => "000000000000000000000000000000",
        117 => "000000000000000000000000000000",
        118 => "000000000000000000000000000000",
        119 => "000000000000000000000000000000",
        120 => "000000000000000000000000000000",
        121 => "000000000000000000000000000000",
        122 => "000000000000000000000000000000",
        123 => "000000000000000000000000000000",
        124 => "000000000000000000000000000000",
        125 => "000000000000000000000000000000",
        126 => "000000000000000000000000000000",
        127 => "000000000000000000000000000000"
    );

    type t_sin is array (0 to 63) of signed(7 downto 0);
    constant SIN_LUT : t_sin := (
        to_signed(   0, 8), to_signed(  12, 8), to_signed(  25, 8), to_signed(  37, 8),
        to_signed(  49, 8), to_signed(  60, 8), to_signed(  71, 8), to_signed(  81, 8),
        to_signed(  90, 8), to_signed(  98, 8), to_signed( 106, 8), to_signed( 112, 8),
        to_signed( 117, 8), to_signed( 122, 8), to_signed( 125, 8), to_signed( 126, 8),
        to_signed( 127, 8), to_signed( 126, 8), to_signed( 125, 8), to_signed( 122, 8),
        to_signed( 117, 8), to_signed( 112, 8), to_signed( 106, 8), to_signed(  98, 8),
        to_signed(  90, 8), to_signed(  81, 8), to_signed(  71, 8), to_signed(  60, 8),
        to_signed(  49, 8), to_signed(  37, 8), to_signed(  25, 8), to_signed(  12, 8),
        to_signed(   0, 8), to_signed( -12, 8), to_signed( -25, 8), to_signed( -37, 8),
        to_signed( -49, 8), to_signed( -60, 8), to_signed( -71, 8), to_signed( -81, 8),
        to_signed( -90, 8), to_signed( -98, 8), to_signed(-106, 8), to_signed(-112, 8),
        to_signed(-117, 8), to_signed(-122, 8), to_signed(-125, 8), to_signed(-126, 8),
        to_signed(-127, 8), to_signed(-126, 8), to_signed(-125, 8), to_signed(-122, 8),
        to_signed(-117, 8), to_signed(-112, 8), to_signed(-106, 8), to_signed( -98, 8),
        to_signed( -90, 8), to_signed( -81, 8), to_signed( -71, 8), to_signed( -60, 8),
        to_signed( -49, 8), to_signed( -37, 8), to_signed( -25, 8), to_signed( -12, 8)
    );


    -- v3.8 sunset gradients: 16 P12-keyframes x {day-hi, day-lo, night-hi,
    -- night-lo} for the sky; 16 x {day, night} for the sun disc; per-palette
    -- deep-water bases for the ocean dimmer.  All separate ROMs with single
    -- read sites (addr registers) -- never share a site with PAL_ROM.
    type t_skyg_rom is array (0 to 127) of std_logic_vector(29 downto 0);
    constant SKYG_ROM : t_skyg_rom := (
         0 => "000000000010000000001000000000",
         1 => "000010101010000010101000010101",
         2 => "000101011110000110001000101001",
         3 => "001000110010001010111000111101",
         4 => "001100000110010001001001001000",
         5 => "001111010110011010011000110011",
         6 => "010010101110100110001000010101",
         7 => "010110101010110001100111110110",
         8 => "011011100110111101010111010111",
         9 => "100000000111000010110111000000",
        10 => "100011010011000011000110101111",
        11 => "101000000010111010100101111000",
        12 => "101101110010101101000101011011",
        13 => "110100001110010010110110010010",
        14 => "110000010001101100001001010100",
        15 => "101000001101010111101010100001",
        16 => "001101011101101010111010010110",
        17 => "001101011101101010111010010110",
        18 => "001101011101101010111010010110",
        19 => "001101011101101010111010010110",
        20 => "001101011101101010111010010110",
        21 => "001101011101101010111010010110",
        22 => "001101011101101010111010010110",
        23 => "001111001001101000111010011000",
        24 => "010000110101100110111010011011",
        25 => "010010100101100100111010011110",
        26 => "010101000101100001111010100010",
        27 => "010111101001011110111010100110",
        28 => "011101010001010111101001110011",
        29 => "100000000001010011111001011010",
        30 => "100010110101010000001001000001",
        31 => "100010110101010000001001000001",
        32 => "000000000010000000001000000000",
        33 => "000100000010000101111000010111",
        34 => "001000010010001101101000100110",
        35 => "001101010010011001001000100110",
        36 => "010010001110100110001000010101",
        37 => "010111100110110100100111110010",
        38 => "011101100110111101110111010010",
        39 => "100011010011000000110110101100",
        40 => "101000110010111011010101111011",
        41 => "101101111110101111010101011110",
        42 => "110011001110100000010101011100",
        43 => "110111000110010001100110001010",
        44 => "111000000110000100000111010101",
        45 => "110110100001110110111000011111",
        46 => "110100001101101111011001000010",
        47 => "110011101101100100101001001011",
        48 => "000000000010000000001000000000",
        49 => "000000000010000000001000000000",
        50 => "000000000010000000001000000000",
        51 => "000000000010000000001000000000",
        52 => "000000000010000000001000000000",
        53 => "000000000010000000001000000000",
        54 => "000000000010000000001000000000",
        55 => "000000000010000000001000000000",
        56 => "000000000010000000001000000000",
        57 => "000000000010000000001000000000",
        58 => "000000000010000000001000000000",
        59 => "000000000010000000001000000000",
        60 => "000000000010000000001000000000",
        61 => "000000000010000000001000000000",
        62 => "000000000010000000001000000000",
        63 => "000000000010000000001000000000",
        64 => "000000000010000000001000000000",
        65 => "000001001001111111001000001101",
        66 => "000010010101111101111000010110",
        67 => "000011011001111100111000100001",
        68 => "000100101001111011101000101100",
        69 => "000101101101111010101000110111",
        70 => "000110111001111001011001000000",
        71 => "000111111001110111111001001001",
        72 => "001000111001110111001001010000",
        73 => "001001101001110110011001010111",
        74 => "001010010001110110001001011101",
        75 => "001010101101110101101001100001",
        76 => "001011000001110101011001100011",
        77 => "001011010001110101011001100011",
        78 => "001011011101110100111001100100",
        79 => "001011011101110100111001100100",
        80 => "000111011001110000111001100000",
        81 => "000111011001110000111001100000",
        82 => "000111011001110000111001100000",
        83 => "000111011001110000111001100000",
        84 => "000111011001110000111001100000",
        85 => "000111011001110000111001100000",
        86 => "000111011001110000111001100000",
        87 => "001000101001101110101001100111",
        88 => "001001111101101101001001101101",
        89 => "001011010001101011011001110011",
        90 => "001100110101101001111001110111",
        91 => "001110011001101000001001111011",
        92 => "010001010001100101101001110011",
        93 => "010100001001100011001001101011",
        94 => "010110110001100001101001100110",
        95 => "010110110001100001101001100110",
        96 => "000000000010000000001000000000",
        97 => "000010000101111110011000010001",
        98 => "000100001001111100111000011111",
        99 => "000110001001111011101000101100",
        100 => "001000001001111010001000110111",
        101 => "001001101101111001001001000010",
        102 => "001011001001110111111001001011",
        103 => "001100011101110111001001010010",
        104 => "001101011101110110011001011001",
        105 => "001110010001110110001001011101",
        106 => "001110111101110101101001011111",
        107 => "001111011001110101001001100010",
        108 => "001111101101110100111001100100",
        109 => "001111111101110100111001100100",
        110 => "010000001101110100111001100100",
        111 => "010000011101110100111001100100",
        112 => "000000000010000000001000000000",
        113 => "000000000010000000001000000000",
        114 => "000000000010000000001000000000",
        115 => "000000000010000000001000000000",
        116 => "000000000010000000001000000000",
        117 => "000000000010000000001000000000",
        118 => "000000000010000000001000000000",
        119 => "000000000010000000001000000000",
        120 => "000000000010000000001000000000",
        121 => "000000000010000000001000000000",
        122 => "000000000010000000001000000000",
        123 => "000000000010000000001000000000",
        124 => "000000000010000000001000000000",
        125 => "000000000010000000001000000000",
        126 => "000000000010000000001000000000",
        127 => "000000000010000000001000000000"
    );
    type t_sung_rom is array (0 to 31) of std_logic_vector(29 downto 0);
    constant SUNG_ROM : t_sung_rom := (
         0 => "001111001010111000110111000110",
         1 => "001111001010111000110111000110",
         2 => "001111001010111000110111000110",
         3 => "001111001010111000110111000110",
         4 => "001111001010111000110111000110",
         5 => "001111110110111010010111000010",
         6 => "010000101110111100110110111101",
         7 => "010010001011000010000110110010",
         8 => "010101111011001100110110010101",
         9 => "011010010111010011010101110001",
        10 => "100000000011001110100101000111",
        11 => "100110001111000001110100100001",
        12 => "101011101110110101100100000110",
        13 => "110000010110101010000011111000",
        14 => "110100000110100000000011111010",
        15 => "110110111010010111000100001101",
        16 => "001010111101111101011000010011",
        17 => "010010110101111100011000011000",
        18 => "011010101101111011011000011001",
        19 => "100010001101111011011000011001",
        20 => "101000011101111011011000010111",
        21 => "101101001101111011111000010101",
        22 => "110001010001111100011000010100",
        23 => "110100100001111100101000010010",
        24 => "110110110101111101001000001111",
        25 => "111000100001111101001000001101",
        26 => "111001100101111101101000001101",
        27 => "111010010101111101001000001111",
        28 => "111010110101111101001000010001",
        29 => "111011010001111100101000010010",
        30 => "111011011101111011111000010101",
        31 => "111011101001111011011000010101"
    );
    type t_deepb_rom is array (0 to 3) of std_logic_vector(29 downto 0);
    constant DEEPB_ROM : t_deepb_rom := (
         0 => "001110111101100000111011010000",
         1 => "001101011101101010111010010110",
         2 => "001010101001110010111010001011",
         3 => "000111011001110000111001100000"
    );

    type t_skyb is array (0 to 7) of std_logic_vector(29 downto 0);
    constant SKYB_ROM : t_skyb := (
        0 => "101000001101010111101010100001", 1 => "110011101101100100101001001011", 2 => "101011100110101001010101010000", 3 => "110100111110011001110101010101",
        4 => "010110011110010101101001111010", 5 => "100110010110111110000101111111", 6 => "000110100101111000111001010101", 7 => "001110001101110100001001101011");

    -- Droplet arc: cell-rows above the surface over a 16-frame flight
    type t_arc is array (0 to 15) of unsigned(2 downto 0);
    constant ARC_ROM : t_arc := (
        "001", "010", "011", "100", "101", "101", "110", "110",
        "110", "101", "101", "100", "011", "010", "001", "000");

    -- band-boundary undulation: one sine period over 16 cells, +-1.5 rows
    type t_ofb is array (0 to 15) of signed(7 downto 0);
    constant OFB1_ROM : t_ofb := (
        to_signed(0,8), to_signed(37,8), to_signed(68,8), to_signed(89,8),
        to_signed(96,8), to_signed(89,8), to_signed(68,8), to_signed(37,8),
        to_signed(0,8), to_signed(-37,8), to_signed(-68,8), to_signed(-89,8),
        to_signed(-96,8), to_signed(-89,8), to_signed(-68,8), to_signed(-37,8));
    -- Sky keyframes split per channel for the one-channel-at-a-time lerp, plus
    -- the FSM's sine in the unused corner, so the whole sunset engine and the
    -- wave oscillators share ONE registered ROM port (one block RAM pair).
    -- Address = night(8) & target(7:6) [00 hi, 01 lo, 10 glow] & channel(5:4)
    -- & keyframe(3:0); target 11 (night 0) = SIN_LUT(0..63) in the low 8 bits.
    type t_skych is array (0 to 511) of unsigned(9 downto 0);
    function f_skych return t_skych is
        variable r : t_skych := (others => (others => '0'));
        variable w : std_logic_vector(29 downto 0);
    begin
        for nt in 0 to 1 loop
            for tg in 0 to 2 loop
                for k in 0 to 15 loop
                    w := SKYG_ROM(nt * 64 + tg * 16 + k);
                    r(nt * 256 + tg * 64 + k)      := unsigned(w(29 downto 20));
                    r(nt * 256 + tg * 64 + 16 + k) := unsigned(w(19 downto 10));
                    r(nt * 256 + tg * 64 + 32 + k) := unsigned(w(9 downto 0));
                end loop;
            end loop;
        end loop;
        for i in 0 to 63 loop
            r(192 + i) := unsigned(resize(SIN_LUT(i), 10));
        end loop;
        return r;
    end function;
    constant SKY_CH : t_skych := f_skych;

    -- Blocky sun: 6x6 circle in an 8x8 cell field, row-major, bit 63 = (0,0)
    constant SUN_PAT : std_logic_vector(63 downto 0) :=
        "00000000" & "00111100" & "01111110" & "01111110" &
        "01111110" & "01111110" & "00111100" & "00000000";

    ----------------------------------------------------------------------------
    -- Symmetric (toward-zero) shift: keeps +/- wave halves identical, and the
    -- conservative flux/diffusion pairs exact.
    ----------------------------------------------------------------------------
    -- sign bit of a 15-bit difference (true when the value is negative)
    -- Resolution scaling: per-mode steps from the video timing ID.
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

    function sgn15(x : signed(14 downto 0)) return std_logic is
    begin
        return x(14);
    end function;

    -- PRESENT cipher 4-bit S-box, one LUT4 per output bit (a case, not a
    -- constant array: parallel array-of-arrays reads crash GHDL synth)
    function f_sb4(n : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(n) is
            when 0  => return x"C";  when 1  => return x"5";
            when 2  => return x"6";  when 3  => return x"B";
            when 4  => return x"9";  when 5  => return x"0";
            when 6  => return x"A";  when 7  => return x"D";
            when 8  => return x"3";  when 9  => return x"E";
            when 10 => return x"F";  when 11 => return x"8";
            when 12 => return x"4";  when 13 => return x"7";
            when 14 => return x"1";  when others => return x"2";
        end case;
    end function;

    function f_sb16(v : unsigned(15 downto 0)) return unsigned is
    begin
        return f_sb4(v(15 downto 12)) & f_sb4(v(11 downto 8))
             & f_sb4(v(7 downto 4))   & f_sb4(v(3 downto 0));
    end function;

    -- Crab sprite, 5 wide x 3 tall cells: "01" body red, "10" dark (eyes,
    -- legs), "00" clear.  Frames 0/1 = walking (legs alternate), 2 = claws
    -- lowered (claws wave while it pauses).
    --   F0  B D . D B    F1  B D . D B    F2  . D . D .
    --       B B B B B        B B B B B        B B B B B
    --       . D . D .        D . D . D        . D . D .
    function f_crab(frm : unsigned(1 downto 0); dy : unsigned(1 downto 0);
                    dx : unsigned(2 downto 0)) return unsigned is
        -- plain logic, not a case table: yosys turns case tables into ROMs
        -- and put this one in an EBR feeding P2 straight from RDATA.
        -- dx is 0..4 here (the caller gates the column range).
        variable b, d : std_logic;
    begin
        b := '0'; d := '0';
        if dy = "01" then
            b := '1';                                   -- body row
        elsif dy = "00" then
            if dx(0) = '1' then
                d := '1';                               -- eyes (dx 1, 3)
            elsif dx(1) = '0' and frm /= "10" then
                b := '1';                               -- claws up (dx 0, 4)
            end if;
        elsif dy = "10" then
            if frm = "01" then d := not dx(0);          -- legs on 0, 2, 4
            else               d := dx(0); end if;      -- legs on 1, 3
        end if;
        return d & b;
    end function;

    function sshr(x : signed; n : natural) return signed is
    begin
        if x < 0 then
            return -shift_right(-x, n);
        else
            return shift_right(x, n);
        end if;
    end function;

    ----------------------------------------------------------------------------
    -- State BRAMs: 256 x 16 each (eta / u sign-extended s13, foam(15:8) &
    -- wet(7:0) packed).  Power-of-2 width and depth (inferno BRAM lesson).
    ----------------------------------------------------------------------------
    type t_mem16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal eta_mem : t_mem16 := (others => (others => '0'));
    signal u_mem   : t_mem16 := (others => (others => '0'));
    signal fw_mem  : t_mem16 := (others => (others => '0'));

    signal e_rd, u_rd, fw_rd : std_logic_vector(15 downto 0) := (others => '0');

    -- display snapshot banks: copied from the live state once per frame in
    -- vblank, so the physics FSM owns the live BRAMs ALL frame (hundreds of
    -- ticks/frame possible) and the display never sees a mid-tick state
    signal eta_dsp : t_mem16 := (others => (others => '0'));
    signal fw_dsp  : t_mem16 := (others => (others => '0'));
    signal e_rd_d, fw_rd_d : std_logic_vector(15 downto 0) := (others => '0');
    type t_grad is array (0 to 7) of std_logic_vector(15 downto 0);
    signal gradY_mem, gradU_mem, gradV_mem : t_grad := (others => (others => '0'));

    -- No read ever lands on an address while it is being written (every FSM
    -- read re-issues its address >= 2 states before use; display banks and
    -- the gradient only collide in vblank, unread), so skip yosys's
    -- read-before-write emulation (~95 LUT + 214 FF across these 7 RAMs).
    -- Needs the read-data registers' init values to take effect.
    attribute no_rw_check : boolean;
    attribute no_rw_check of eta_mem   : signal is true;
    attribute no_rw_check of u_mem     : signal is true;
    attribute no_rw_check of fw_mem    : signal is true;
    attribute no_rw_check of eta_dsp   : signal is true;
    attribute no_rw_check of fw_dsp    : signal is true;
    attribute no_rw_check of gradY_mem : signal is true;
    attribute no_rw_check of gradU_mem : signal is true;
    attribute no_rw_check of gradV_mem : signal is true;

    -- Per-cell sand wetness: 4-bit counter for every (column, row below the
    -- sand surface) cell, address = x_idx & depth row (16 rows covers the
    -- deepest wash).  Read once per cell by the display prefetch (next cell,
    -- issued mid-cell), written back once per field on the cell row's first
    -- line: 15 while the wash covers the cell, else -1 every 12 fields.  The
    -- write lands on the cell being shown while the read port already holds
    -- the same address, but that read is never used (the next prefetch
    -- re-reads after it), so no read-before-write emulation is needed.
    type t_sw is array (0 to 4095) of std_logic_vector(3 downto 0);
    signal sw_mem : t_sw := (others => (others => '0'));
    attribute no_rw_check of sw_mem : signal is true;
    signal sw_rd   : std_logic_vector(3 downto 0) := (others => '0');
    signal sw_ra   : unsigned(11 downto 0) := (others => '0');
    signal sw_we   : std_logic := '0';
    signal sw_wa   : unsigned(11 downto 0) := (others => '0');
    signal sw_wd   : std_logic_vector(3 downto 0) := (others => '0');
    signal sw_an, sw_acur : unsigned(11 downto 0) := (others => '0');
    signal sw_vn, sw_vcur : std_logic := '0';     -- cell is 0..15 rows into sand
    signal sw_cur  : unsigned(3 downto 0) := (others => '0');
    signal sw_div  : unsigned(3 downto 0) := (others => '0');
    signal sw_tick : std_logic := '0';            -- this field dries one step
    signal rowmslb : signed(14 downto 0) := (others => '0');  -- row - sea line

    -- Stars: per-cell hash of (row, column) = sbox(v ^ K) -> +rotl7 -> sbox ->
    -- +rotl11 (two S-box layers + two adds: XOR/rotate-only mixes left
    -- visible diagonal streaks; this one tested lattice-free like true random).
    -- Built for the NEXT cell over x_loc 5/6, finished at the cell promote.
    -- A cell is a star when hash(15:4) < star_thr (0..48 = up to ~1.2%), so
    -- stars appear in a fixed order as the sun sets and never jump.
    signal st_a, st_b, st_cur : unsigned(15 downto 0) := (others => '0');
    signal st_d     : signed(14 downto 0) := (others => '0');  -- sun top - horizon
    signal star_thr : unsigned(5 downto 0) := (others => '0');
    signal p1_star, p1_sdim : std_logic := '0';

    -- Crab (S11).  Lives IN the sand area below the beach line, over the wash
    -- half of the screen.  Position = left column crab_x (5 wide, may go
    -- negative when it walks off the left edge) and crab_d = rows from the
    -- first sand row (under its centre column) down to its top row, so it
    -- follows the slope.  Per-field behaviour in p_vseq steps 7-15, one small
    -- op per step.  The ground under the centre column is the beach line
    -- b = 192 - slope*x, exactly as p_disp sweeps it.
    constant CS_WALK   : unsigned(2 downto 0) := "000";  -- tmr /= 0 = running
    constant CS_PAUSE  : unsigned(2 downto 0) := "001";
    constant CS_BURROW : unsigned(2 downto 0) := "010";
    constant CS_HIDDEN : unsigned(2 downto 0) := "011";
    constant CS_EMERGE : unsigned(2 downto 0) := "100";
    constant CS_OFF    : unsigned(2 downto 0) := "101";
    signal s_crab    : std_logic := '1';
    signal crab_st   : unsigned(2 downto 0) := CS_HIDDEN;
    signal crab_x    : signed(8 downto 0) := to_signed(4, 9);
    signal crab_d    : unsigned(5 downto 0) := to_unsigned(6, 6);
    signal crab_cand : signed(8 downto 0) := to_signed(6, 9);
    signal crab_cd   : unsigned(5 downto 0) := to_unsigned(6, 6);
    signal crab_xc   : unsigned(7 downto 0) := (others => '0');
    signal crab_cm   : unsigned(11 downto 0) := (others => '0');
    signal crab_b    : signed(11 downto 0) := (others => '0');
    signal crab_bot  : signed(14 downto 0) := (others => '0');
    signal crab_gnd  : unsigned(7 downto 0) := (others => '0');
    signal crab_nr3  : unsigned(7 downto 0) := (others => '0');  -- n_rows - 3
    signal crab_base : unsigned(7 downto 0) := (others => '0');  -- top row, standing
    signal crab_wr   : unsigned(6 downto 0) := (others => '0');  -- wash rows + 2
    signal crab_dir  : std_logic := '1';                -- '1' = right
    signal crab_vy   : unsigned(1 downto 0) := "00";    -- 00 level, 01 down, 11 up
    signal crab_sub  : unsigned(3 downto 0) := (others => '0');
    signal crab_tmr  : unsigned(7 downto 0) := (others => '0');
    signal crab_sink : unsigned(1 downto 0) := "11";
    signal crab_frm  : unsigned(1 downto 0) := "00";
    signal crab_lfsr : unsigned(15 downto 0) := x"B5AD";
    signal cf_x0, cf_xoff, cf_xmax, cf_dtop, cf_dbot, cf_wash, cf_ok, cf_inw : std_logic := '0';
    -- render: crab_x / crab_frm are read directly (they only change in
    -- vblank); top/clip rows are derived once per field
    signal crab_top  : unsigned(7 downto 0) := (others => '0');
    signal crab_vis  : std_logic := '0';
    signal p1_crab, p1_crabd : std_logic := '0';
    -- Its hole: 1 row under its feet, as wide as it, at any depth (so not in
    -- the wet-sand RAM, which covers 16 rows), soaked while it digs in or out
    -- and then dried like wet sand.
    signal hole_x    : signed(8 downto 0) := to_signed(-16, 9);
    signal hole_row  : unsigned(7 downto 0) := (others => '1');
    signal hole_lvl  : unsigned(3 downto 0) := (others => '0');
    signal hole_shd  : unsigned(1 downto 0) := "00";   -- per-field wet shade
    signal ho_coln, ho_colc, ho_rowl : std_logic := '0';
    signal crab_xlim : unsigned(7 downto 0) := (others => '0');  -- n_cols - 3
    signal crab_cbase : unsigned(7 downto 0) := (others => '0'); -- candidate top row
    signal s8_prev   : std_logic := '0';
    signal p1_hole   : std_logic := '0';
    signal gY_rd, gU_rd, gV_rd : std_logic_vector(15 downto 0) := (others => '0');
    signal band  : unsigned(2 downto 0) := (others => '0');
    signal bacc  : unsigned(14 downto 0) := (others => '0');
    signal ed_we,  fwd_we  : std_logic := '0';
    signal ed_wa,  fwd_wa  : unsigned(7 downto 0) := (others => '0');
    signal ed_wd,  fwd_wd  : std_logic_vector(15 downto 0) := (others => '0');

    -- FSM-side ports
    signal e_ra_f, u_ra_f, fw_ra_f : unsigned(7 downto 0) := (others => '0');
    signal e_we,  u_we,  fw_we     : std_logic := '0';
    signal e_wa,  u_wa,  fw_wa     : unsigned(7 downto 0) := (others => '0');
    signal e_wd,  u_wd,  fw_wd     : std_logic_vector(15 downto 0) := (others => '0');
    -- display-side read addresses
    signal e_ra_d, fw_ra_d : unsigned(7 downto 0) := (others => '0');

    signal fsm_run : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Raster position / lattice
    ----------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_hedge      : std_logic := '0';
    signal fcnt         : unsigned(9 downto 0) := (others => '0');
    signal field_tgl    : std_logic := '0';

    signal x_loc     : unsigned(4 downto 0) := (others => '0');
    signal x_idx     : unsigned(7 downto 0) := (others => '0');
    signal celly_loc : unsigned(4 downto 0) := (others => '0');
    signal celly_idx : unsigned(7 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';

    signal n_cols : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal n_rows : unsigned(8 downto 0) := to_unsigned(45, 9);

    ----------------------------------------------------------------------------
    -- Latched controls
    ----------------------------------------------------------------------------
    signal lk1, lk2, lk3, lk4, lk5, lk6, lp12 : unsigned(9 downto 0) := (others => '0');
    signal s_gulls  : std_logic := '0';
    signal s_size8  : std_logic := '0';
    signal s_storm  : std_logic := '0';
    signal s_night  : std_logic := '0';

    -- Resolution scaling: the lattice steps through virtual 1080-line units.
    -- x_a = Q.1 position in the cell (x_loc still counts CLOCKS into the
    -- cell for the prefetch sequence), y_a = Q.8 position in the row.
    -- csz / rsz = cell width (Q.1) / row height (Q.8): 16 or 8 units, the
    -- 8 px mode clamped to >= 8 clocks per cell at 720p / SD.
    signal s_xs  : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_csz : unsigned(5 downto 0)  := to_unsigned(32, 6);
    signal s_rsz : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cl1 : unsigned(5 downto 0)  := to_unsigned(30, 6);     -- csz - xs
    signal s_cl2 : unsigned(5 downto 0)  := to_unsigned(28, 6);     -- csz - 2 xs
    signal s_rl1 : unsigned(12 downto 0) := to_unsigned(3840, 13);  -- rsz - ys
    signal x_a   : unsigned(5 downto 0)  := (others => '0');
    signal y_a   : unsigned(12 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Per-frame render parameters (p_vseq)
    ----------------------------------------------------------------------------
    signal vstep     : unsigned(4 downto 0) := "11111";
    signal slb_full  : unsigned(15 downto 0) := (others => '0');
    signal slb_q     : signed(14 downto 0) := to_signed(1710, 15);  -- base sea level, Q6
    signal tide_q    : signed(9 downto 0) := (others => '0');
    signal slr_q     : signed(14 downto 0) := to_signed(1710, 15);  -- with tide
    signal skyband_q : signed(14 downto 0) := to_signed(1000, 15);
    signal skystep_q : unsigned(11 downto 0) := to_unsigned(125, 12);
    signal w_edge    : signed(9 downto 0) := (others => '0');  -- wash edge column
    signal w_tail    : signed(9 downto 0) := (others => '0');  -- where the taper ends
    -- Beach wash: a self-contained sheet that grows ~8 cells down from the
    -- sand's diagonal surface (DEEP->MID->AQUA->SURF + foam edge), sine-shaped
    -- along the diagonal, then recedes; the whole beach face pulses together.
    signal wph    : unsigned(11 downto 0) := (others => '0');  -- wash phase
    signal wreach : signed(14 downto 0) := (others => '0');    -- global reach Q6
    signal rmax_a, rmax_b, reach_r : signed(14 downto 0) := (others => '0');
    signal b_floor   : signed(11 downto 0) := to_signed(-720, 12);
    signal slope_c   : signed(11 downto 0) := to_signed(24, 12);
    signal chopj     : unsigned(5 downto 0) := (others => '0');
    signal pal_sel   : unsigned(1 downto 0) := (others => '0');
    signal sun_x0    : unsigned(7 downto 0) := to_unsigned(58, 8);
    signal sun_cell  : unsigned(7 downto 0) := to_unsigned(3, 8);
    signal ncols_m1  : unsigned(7 downto 0) := to_unsigned(79, 8);

    signal sun_q15   : signed(14 downto 0) := (others => '0');
    signal slb2_q    : signed(14 downto 0) := (others => '0');
    signal stk_len15 : signed(14 downto 0) := (others => '0');
    signal mir_on    : std_logic := '0';
    signal tri_on    : std_logic := '0';
    signal sun_y_r, sun_u_r, sun_v_r : std_logic_vector(9 downto 0) := (others => '0');

    signal gx0, gx1, gx2 : unsigned(7 downto 0) := to_unsigned(20, 8);
    -- plumage bit per gull (0 = dark, 1 = white); FLIPS each time the bird
    -- wraps, so every direction carries every colour over time
    signal gcol : std_logic_vector(2 downto 0) := "010";
    signal gulls_on      : std_logic := '0';
    signal gy0, gy1, gy2 : unsigned(7 downto 0) := to_unsigned(2, 8);
    signal ghz2          : unsigned(7 downto 0) := (others => '0');  -- 2*horizon row

    -- kick handshake: vseq toggles kick_t once per field; the half-rate FSM
    -- compares against kick_seen so the request can never be missed
    signal kick_t    : std_logic := '0';
    signal kick_seen : std_logic := '0';
    signal fsm_ticks : unsigned(5 downto 0) := (others => '0');
    -- the physics FSM runs on every OTHER clock: its whole control/datapath
    -- cone gets two clock periods (27 ns) on silicon -- the v3.5.x one-hot
    -- next-state cone was corrupting at 13.47 ns on hardware (HUD-diagnosed)
    signal fsm_ce    : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Physics FSM
    ----------------------------------------------------------------------------
    type t_fst is (FI, CP0, CP1, CP2,
                   FP0, FP1, FP2, FP3, FP4, FP5, FP6, FP7, FP8, FP9,
                   FP10, FP11, FP12,
                   SK0, SKA, SKB, SKC, SKD, SKG,
                   FE0, FE1, FE2,
                   FVL0, FVL1, FVL2, FV0, FV1, FV2, FV3, FV4, FV5, FV6,
                   FFL0, FFL1, FFL2, FF0, FF1, FF2, FF3, FF4, FF5, FF6, FF7, FF8,
                   FB0, FB1);
    signal fst : t_fst := FI;
    -- 37 states in binary encoding made every FSM register's clock-enable a
    -- ~7-LUT decode of the state vector -- the v3.2 post-route critical path.
    -- One-hot turns each enable into a single state bit.
    -- "none": forbid yosys fsm_detect/fsm_extract from touching this state
    -- register.  Post-synthesis simulation proved the extraction pass was
    -- MIS-COMPILING the 40-state machine (dropped gating -> free-running
    -- ticks on hardware while RTL sim stayed correct).  Plain registers and
    -- muxes are timing-safe here because the FSM runs at half rate (27 ns).
    attribute fsm_encoding : string;
    attribute fsm_encoding of fst : signal is "none";

    signal ticks_left : unsigned(5 downto 0) := (others => '0');
    -- hard per-field tick budget: ticks_run resets ONLY on the (HW-verified
    -- clean) frame edge and gates every tick start.  Whatever lets the run
    -- loop retrigger on silicon, it cannot exceed the budget.
    signal field_seen : std_logic := '0';
    signal ticks_run  : unsigned(5 downto 0) := (others => '0');
    signal ncf        : unsigned(8 downto 0) := to_unsigned(80, 9);

    -- physics knobs (registered in PREP)
    signal p_inj  : unsigned(7 downto 0) := to_unsigned(90, 8);
    signal p_inc1 : unsigned(4 downto 0) := to_unsigned(6, 5);
    signal p_inc2 : unsigned(6 downto 0) := to_unsigned(15, 7);
    signal p_amp  : unsigned(8 downto 0) := to_unsigned(192, 9);
    signal p_camp : unsigned(7 downto 0) := (others => '0');

    signal ph1, ph2, ph3 : unsigned(9 downto 0) := (others => '0');
    signal p_inc3   : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal sin_v1, sin_v2, sin_v3 : signed(7 downto 0) := (others => '0');
    -- sine ROM ports (registered, one read site each so yosys can use block
    -- RAM): the FSM sequences ph1/ph2/ph3 through sin_a; the wash edge reads
    -- dsin_a, issued two clocks before it is used
    signal dsin_a : unsigned(5 downto 0) := (others => '0');
    signal dsin_q : signed(7 downto 0) := (others => '0');
    signal sung_q : std_logic_vector(29 downto 0) := (others => '0');
    signal swell_a  : signed(11 downto 0) := (others => '0');
    signal chop_v   : signed(11 downto 0) := (others => '0');
    signal swell    : signed(11 downto 0) := (others => '0');

    -- swell noise + surge (v3.2)
    signal lfsr      : unsigned(15 downto 0) := x"ACE1";
    signal wave_new  : std_logic := '0';
    signal inc_j     : unsigned(9 downto 0) := (others => '0');
    signal amp_mod   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal wave_jit  : signed(4 downto 0) := (others => '0');
    signal surge_amp : unsigned(8 downto 0) := (others => '0');
    signal surge_off : signed(11 downto 0) := (others => '0');
    signal th_mid, th_hi : signed(11 downto 0) := (others => '0');

    -- shared multiplier
    -- sunset colour engine (computed once per kick via the shared mult):
    -- sky hi/lo = lerp between adjacent P12 keyframes; ocean deep = base
    -- scaled toward black/neutral by dscale
    signal sky_a  : unsigned(8 downto 0) := (others => '0');   -- SKY_CH port
    signal sky_q  : unsigned(9 downto 0) := (others => '0');
    signal sk_a   : unsigned(9 downto 0) := (others => '0');   -- keyframe k value
    signal sk_hi  : unsigned(9 downto 0) := (others => '0');   -- this channel's sky top
    signal sk_ch  : unsigned(1 downto 0) := (others => '0');   -- 0 Y, 1 U, 2 V
    signal sk_tg  : unsigned(1 downto 0) := (others => '0');   -- 00 hi, 10 glow, 01 lo
    signal g_acc, g_d : signed(10 downto 0) := (others => '0');
    signal gk : unsigned(2 downto 0) := (others => '0');
    signal grad_we : std_logic_vector(2 downto 0) := "000";    -- one per channel
    signal grad_wa : unsigned(2 downto 0) := (others => '0');
    signal grad_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal t_r       : unsigned(3 downto 0) := (others => '0');
    -- geometry-scaled sun position: 64 .. skyband+512 across full P12 travel,
    -- computed here (mult) and consumed by vseq next field (1-field lag)
    signal sun_q_f   : unsigned(11 downto 0) := to_unsigned(500, 12);
    signal skylo_y, skylo_u, skylo_v : std_logic_vector(9 downto 0) := (others => '0');

    signal ma : signed(11 downto 0) := (others => '0');
    signal mb : signed(9 downto 0) := (others => '0');
    signal mp : signed(21 downto 0);
    -- Serial multiplier (was a 12x10 LUT array, 294 LUTs, and a 1-clock path
    -- the FSM depended on): 13-bit add/shift per clock, 10 clocks per
    -- product.  It starts by itself when ma/mb change, and the FSM simply
    -- stalls while it is busy, so every state still sees the product of the
    -- operands the previous state wrote -- same schedule as before.
    signal op_a      : signed(11 downto 0) := (others => '0');
    signal op_b      : signed(9 downto 0) := (others => '0');
    signal m_hi      : signed(12 downto 0) := (others => '0');
    signal m_lo      : signed(9 downto 0) := (others => '0');
    signal m_cnt     : unsigned(3 downto 0) := (others => '0');
    signal m_run, m_done : std_logic := '0';
    signal m_stall   : std_logic;
    signal fsm_en    : std_logic := '0';

    -- sweep registers
    signal vi        : unsigned(8 downto 0) := (others => '0');
    signal b_raw     : signed(11 downto 0) := (others => '0');
    signal b_nx_raw  : signed(11 downto 0) := (others => '0');
    signal b_eff_i   : signed(11 downto 0) := (others => '0');
    signal b_eff_ip1 : signed(11 downto 0) := (others => '0');

    signal eta_i, eta_ip1 : signed(11 downto 0) := (others => '0');
    signal u_i            : signed(11 downto 0) := (others => '0');
    signal d_i, d_ip1     : signed(11 downto 0) := (others => '0');
    signal de_r           : signed(11 downto 0) := (others => '0');
    signal grav_r, fric_r : signed(11 downto 0) := (others => '0');
    signal neg_sl         : signed(11 downto 0) := (others => '0');
    signal sum_a, sum_b   : signed(11 downto 0) := (others => '0');
    signal un_r           : signed(11 downto 0) := (others => '0');
    signal efin_r         : signed(11 downto 0) := (others => '0');

    signal eta_pend  : signed(11 downto 0) := (others => '0');
    signal foam_pend : unsigned(8 downto 0) := (others => '0');
    signal wet_pend  : unsigned(7 downto 0) := (others => '0');
    signal fw_jp1    : std_logic_vector(15 downto 0) := (others => '0');
    signal qf_prev   : signed(11 downto 0) := (others => '0');
    signal qf_j_r    : signed(11 downto 0) := (others => '0');
    signal df_prev   : signed(11 downto 0) := (others => '0');
    signal df_j      : signed(11 downto 0) := (others => '0');
    signal slope_w   : signed(11 downto 0) := (others => '0');
    signal brk_r     : std_logic := '0';
    signal cap_r     : std_logic := '0';
    signal foam_adj  : unsigned(8 downto 0) := (others => '0');
    signal sub_next  : unsigned(7 downto 0) := (others => '0');
    signal add_next  : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Display prefetch / per-cell derived
    ----------------------------------------------------------------------------
    signal pf_cnt   : unsigned(2 downto 0) := "111";
    signal eta_cur, eta_nxt : signed(11 downto 0) := (others => '0');
    signal foam_cur, foam_nxt : unsigned(7 downto 0) := (others => '0');
    signal wet_cur, wet_nxt   : unsigned(7 downto 0) := (others => '0');
    signal b_cur, b_nxt       : signed(11 downto 0) := C_B0;

    signal surf_q  : signed(14 downto 0) := (others => '0');
    signal wreach_q  : signed(14 downto 0) := (others => '0');  -- per-col wash reach
    -- the next cell's wash edge, built mid-cell so the cell boundary only
    -- does the final min (the whole x_idx -> wreach_q cone in one cycle was
    -- 17-21 ns routed)
    signal wr_n, cap_n : signed(14 downto 0) := (others => '0');
    signal pr_cell : std_logic := '0';
    -- The aqua/mid (off_b) and mid/deep (off_c) band boundaries each ride their
    -- OWN per-column, time-drifting offset so all three blue layers slide
    -- independently -- but stored as just the small offset (surf_q + off), so
    -- the band flags reuse the single row_q-surf_q subtract (saves 2 refs).
    signal off_b   : signed(7 downto 0) := (others => '0');
    signal off_c   : signed(7 downto 0) := (others => '0');
    signal bot_q   : signed(14 downto 0) := (others => '0');
    signal wat_c   : std_logic := '0';
    -- droplet: one cell per column arcing off the crest on a staggered cycle
    -- (stored as a cell-row index so the pixel test is a single compare)
    signal drop_cell  : unsigned(7 downto 0) := (others => '0');
    signal drop_on    : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Pixel pipeline P1..P4
    ----------------------------------------------------------------------------
    signal row_q : signed(14 downto 0) := (others => '0');

    -- band membership as parallel subtract-sign flags (one 15-bit carry chain
    -- each, all in P1) so P2 is a pure boolean priority mux with no chains
    signal p1_fsand : std_logic := '0';
    signal p1_uw    : std_logic := '0';   -- pixel under the active wash sheet
    signal p1_lead  : std_logic := '0';   -- wash leading (foam) edge
    signal p1_lgap  : std_logic := '0';   -- breakup gaps in the wash foam edge
    signal p1_band  : unsigned(1 downto 0) := (others => '0');  -- DEEP..SURF
    signal p1_res   : std_logic := '0';   -- residual wet sand (fading)
    signal p1_wl    : unsigned(1 downto 0) := (others => '0');  -- wet shade   -- row at/below the sand line
    signal p1_b0    : std_logic := '0';   -- at/below the water surface
    signal p1_b64   : std_logic := '0';   -- within the crest row
    signal p1_b128  : std_logic := '0';   -- within 2 rows of the surface
    signal p1_ba192 : std_logic := '0';   -- above the aqua/mid boundary
    signal p1_bm448 : std_logic := '0';   -- above the mid/deep boundary
    signal p1_spr   : std_logic := '0';   -- within 1 row above the surface
    signal p1_b256  : std_logic := '0';   -- top row of the MID band
    signal p1_b512  : std_logic := '0';   -- top row of the DEEP band
    signal p1_fM    : std_logic := '0';   -- mid-layer foam gate
    signal p1_fD    : std_logic := '0';   -- deep-layer foam gate
    signal p1_gullw : std_logic := '0';   -- white gull (flying right)
    signal p1_mir   : std_logic := '0';   -- mirrored sun disc on the ocean
    signal p1_stk   : std_logic := '0';   -- horizon streak (sun-wide, ocean only)
    signal p2_sky, p3_sky : std_logic_vector(1 downto 0) := "00";
    signal p1_sky2  : std_logic := '0';
    signal p1_wat       : std_logic := '0';
    signal p1_fhi, p1_fmid, p1_fspray : std_logic := '0';
    signal p1_fcrest    : std_logic := '0';
    signal p1_gap       : std_logic := '0';
    signal p1_sand2     : std_logic := '0';

    signal p1_sun       : std_logic := '0';
    signal p1_gull      : std_logic := '0';
    signal p1_gullr     : std_logic := '0';   -- gull reflection on the ocean
    signal p1_drop      : std_logic := '0';   -- flying crest droplet (over water)
    signal p1_sprayh    : std_logic := '0';

    signal p2_idx : unsigned(4 downto 0) := C_I_SKYHI;
    signal p3_yuv : std_logic_vector(29 downto 0) := (others => '0');
    signal p4_y, p4_u, p4_v : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Sync pipe
    ----------------------------------------------------------------------------
    type t_pipe is array (0 to C_LAT - 1) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe;

begin

    ----------------------------------------------------------------------------
    -- Raster counters, lattice, frame bookkeeping.
    ----------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;
            r_hedge <= v_h_edge;
            if v_v_edge = '1' then
                field_tgl <= not field_tgl;
            end if;

            -- X lattice
            if v_h_edge = '1' then
                if x_idx > to_unsigned(4, 8) then
                    n_cols <= resize(x_idx, 9) + 1;
                end if;
                x_loc <= (others => '0');
                x_idx <= (others => '0');
                x_a   <= (others => '0');
            elsif data_in.avid = '1' then
                -- last pixel of the cell when the next one crosses csz
                -- (x_a + xs >= csz  <=>  x_a >= csz - xs)
                if x_a >= s_cl1 then
                    x_loc <= (others => '0');
                    x_a   <= x_a - s_cl1;
                    if x_idx /= to_unsigned(255, 8) then
                        x_idx <= x_idx + 1;
                    end if;
                else
                    x_loc <= x_loc + 1;
                    x_a   <= x_a + s_xs;
                end if;
            end if;

            -- Y lattice, anchored to the first active line of the field.
            -- Rows are COUNTED during active video and latched at the v edge
            -- (never derived from the line counter at vsync -- height trap).
            if v_v_edge = '1' then
                if celly_idx > to_unsigned(4, 8) then
                    n_rows <= resize(celly_idx, 9) + 1;
                end if;
                celly_loc <= (others => '0');
                celly_idx <= (others => '0');
                y_a       <= (others => '0');
                frame_act <= '0';
                band      <= (others => '0');
                bacc      <= resize(skystep_q, 15);
                fcnt      <= fcnt + 1;

                -- latch controls once per field
                lk1  <= unsigned(registers_in(0));
                lk2  <= unsigned(registers_in(1));
                lk3  <= unsigned(registers_in(2));
                lk4  <= unsigned(registers_in(3));
                lk5  <= unsigned(registers_in(4));
                lk6  <= unsigned(registers_in(5));
                lp12 <= unsigned(registers_in(7));
                s_gulls  <= registers_in(6)(0);
                s_size8  <= registers_in(6)(1);
                s_storm  <= registers_in(6)(2);
                s_night  <= registers_in(6)(3);
                s_crab   <= registers_in(6)(4);
            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_idx <= (others => '0');
                y_a       <= (others => '0');
            elsif v_h_edge = '1' and frame_act = '1' then
                if y_a >= s_rl1 then
                    celly_loc <= (others => '0');
                    y_a       <= y_a - s_rl1;
                    if celly_idx /= to_unsigned(255, 8) then
                        celly_idx <= celly_idx + 1;
                        -- vertical sky band advance (skyband/8 per band)
                        if resize(celly_idx + 1, 15) & "000000" >= bacc
                           and band /= "111" then
                            band <= band + 1;
                            bacc <= bacc + resize(skystep_q, 15);
                        end if;
                    end if;
                else
                    celly_loc <= celly_loc + 1;
                    y_a       <= y_a + s_ys;
                end if;
            end if;

            -- per-mode steps (the timing ID is static while a mode runs) and
            -- cell pitch 16/8 virtual units (S8); the 8 px mode keeps >= 8
            -- clocks per cell: 12 units at 720p, 16 at SD
            s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
            if s_size8 = '0' or s_xs = 4 then
                s_csz <= to_unsigned(32, 6);  s_rsz <= to_unsigned(4096, 13);
            elsif s_xs = 3 then
                s_csz <= to_unsigned(24, 6);  s_rsz <= to_unsigned(3072, 13);
            else
                s_csz <= to_unsigned(16, 6);  s_rsz <= to_unsigned(2048, 13);
            end if;
            s_cl1 <= s_csz - s_xs;
            s_cl2 <= s_csz - shift_left(resize(s_xs, 6), 1);
            s_rl1 <= s_rsz - s_ys;
        end if;
    end process p_position;

    ----------------------------------------------------------------------------
    -- Per-frame sequencer: render geometry, gull motion, physics kick.
    ----------------------------------------------------------------------------
    p_vseq : process(clk)
        variable v_ticks : unsigned(5 downto 0);
        variable v_sunq  : unsigned(11 downto 0);
        variable v_nq    : unsigned(8 downto 0);
        variable v_tf    : unsigned(8 downto 0);
        variable v_rmax  : signed(14 downto 0);
        variable v_reach : signed(14 downto 0);
        variable v_swp   : signed(14 downto 0);
        variable v_sd    : signed(14 downto 0);
        variable v_cx    : signed(8 downto 0);
        variable v_cg    : signed(14 downto 0);
        variable v_dir, v_run, v_mv : std_logic;
        variable v_tmr   : unsigned(7 downto 0);
        variable v_st    : unsigned(2 downto 0);
        variable v_vy    : unsigned(1 downto 0);
        variable v_cbase : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            crab_lfsr <= crab_lfsr(14 downto 0)
                         & (crab_lfsr(15) xor crab_lfsr(13)
                            xor crab_lfsr(12) xor crab_lfsr(10));
            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                vstep <= (others => '0');
            end if;

            case to_integer(vstep) is
                when 0 =>
                    slb_full <= shift_left(resize(n_rows, 16), 6);
                    tide_q   <= resize(sshr(signed(resize(lk2, 11))
                                            - to_signed(512, 11), 1), 10);
                    -- wash animation clock (~3.4 s cycle: a little slower trigger)
                    wph <= wph + to_unsigned(20, 12);
                    -- wph upper half = PAUSE (reach 0, sand fully recovers);
                    -- lower half = rise then fall.  PEAK depth now correlates to
                    -- WAVE BEHAVIOUR: taller waves (swell) and more foam (p_inj)
                    -- send the over-wash further down, with a small tide nudge.
                    -- min(v_reach, rmax) later clamps the depth at 8 px.
                    -- (split over steps 0/1: lk2 is latched on the vsync edge
                    -- itself, so step 0 has exactly one clock after it)
                    v_tf   := lk2(9 downto 1);          -- tide fraction = lk2/2
                    if swell > to_signed(0, 12) then v_swp := resize(swell, 15);
                    else v_swp := (others => '0'); end if;
                    rmax_a <= to_signed(96, 15)
                              + signed(resize(shift_right(v_tf, 2), 15));  -- tide
                    rmax_b <= shift_right(v_swp, 1)                         -- taller wave
                              + signed(resize(shift_right(p_inj, 1), 15)); -- more foam
                    if wph(11) = '1' then
                        reach_r <= (others => '0');
                    elsif wph(10) = '0' then
                        reach_r <= signed(resize(shift_right(wph(9 downto 0), 1), 15));
                    else
                        reach_r <= signed(resize(shift_right(
                            to_unsigned(1023, 10) - wph(9 downto 0), 1), 15));
                    end if;
                    -- wet sand dries one step every 12 fields (15 steps ~3 s)
                    if sw_div = to_unsigned(11, 4) then
                        sw_div  <= (others => '0');
                        sw_tick <= '1';
                    else
                        sw_div  <= sw_div + 1;
                        sw_tick <= '0';
                    end if;
                    vstep    <= vstep + 1;
                when 1 =>
                    v_rmax := rmax_a + rmax_b;
                    if reach_r > v_rmax then wreach <= v_rmax;
                    else wreach <= reach_r; end if;
                    -- sea level at ~58% of screen height
                    slb_q <= signed(resize(shift_right(slb_full, 1), 15))
                           + signed(resize(shift_right(slb_full, 4), 15))
                           + signed(resize(shift_right(slb_full, 5), 15));
                    vstep <= vstep + 1;
                when 2 =>
                    slr_q     <= slb_q - resize(tide_q, 15);
                    skyband_q <= slb_q - signed(shift_left(resize(n_rows, 15), 4));
                    if n_rows > to_unsigned(63, 9) then
                        b_floor <= to_signed(-1008, 12);
                    else
                        b_floor <= -signed(shift_left(resize(n_rows, 12), 4));
                    end if;
                    -- gentler sand slope => wider exposed beach (the wash + wet
                    -- span more of the screen); also gentles wave shoaling a bit
                    if s_size8 = '1' then
                        slope_c <= to_signed(5, 12);
                    else
                        slope_c <= to_signed(8, 12);
                    end if;
                    skystep_q <= unsigned(skyband_q(14 downto 3));
                    -- Wash reaches the SCREEN MIDPOINT (ncols/2), resolution-
                    -- independent, +/- a tide swing (higher tide -> a bit wider).
                    -- The per-column cap = (w_edge - x)*128 rounds the far tip.
                    w_edge <= signed(resize(shift_right(n_cols, 1), 10))
                              + resize(shift_right(tide_q, 4), 10);
                    chopj <= lk4(9 downto 4);
                    -- palette rides the sun: Night switch pins the night set;
                    -- in Day the palette follows P12 down through sunset
                    if s_night = '1' then
                        pal_sel <= "11";
                    elsif lp12 > to_unsigned(640, 10) then
                        pal_sel <= "00";
                    elsif lp12 > to_unsigned(320, 10) then
                        pal_sel <= "01";
                    else
                        pal_sel <= "10";
                    end if;
                    vstep <= vstep + 1;
                when 3 =>
                    -- 8px: a long soft tail (half a row per column, ending 15
                    -- columns past w_edge); 16px keeps its 2-rows-per-column end
                    if s_size8 = '1' then w_tail <= w_edge + 15;
                    else                  w_tail <= w_edge; end if;
                    ncols_m1 <= resize(n_cols - 1, 8);
                    -- quantize the width so per-field +-1/2 jitter on n_cols
                    -- (HW active-pixel-count noise) can't wobble the sun's x
                    v_nq := n_cols and to_unsigned(508, 9);   -- mask low 2 bits
                    sun_x0 <= resize(v_nq - shift_right(v_nq, 2) - 4, 8);
                    -- P12 is the sun/moon elevation: slider up = high in the
                    -- sky, slider down = sinking into the sea
                    v_sunq := sun_q_f;
                    sun_cell <= resize(v_sunq(11 downto 6), 8);
                    sun_q15  <= signed(resize(v_sunq, 15));
                    vstep    <= vstep + 1;
                when 4 =>
                    -- horizon = the SKY/OCEAN line (skyband_q).  The sun sets
                    -- BEHIND the ocean strip: mirrored disc while fully above
                    -- the line, sun-wide streak while straddling it, nothing
                    -- once fully behind
                    -- mirrored disc stays WHOLE until the sun's bottom is
                    -- one line past the horizon; then it blooms into the
                    -- column, which grows to the full ocean depth (x4 rate)
                    -- disc stays WHOLE until the sun's bottom is ~2 lines
                    -- past the horizon, THEN the column forms.  A 320 offset
                    -- (= real 448 height minus 2 lines) against the plain
                    -- horizon is identical to 448 vs skyband+128 but adds no
                    -- extra adder -- stays inside the LC budget.
                    slb2_q  <= shift_left(skyband_q, 1);
                    sun_y_r <= sung_q(29 downto 20);
                    sun_u_r <= sung_q(19 downto 10);
                    sun_v_r <= sung_q(9 downto 0);

                    -- gulls: leftward drift at three rates, wrap at 0
                    if s_gulls = '1' then
                        if fcnt(0) = '0' then
                            if gx0 = 0 then
                                gx0 <= ncols_m1; gcol(0) <= not gcol(0);
                            else gx0 <= gx0 - 1; end if;
                        end if;
                        if fcnt(1 downto 0) = "00" then
                            if gx1 = ncols_m1 then
                                gx1 <= (others => '0'); gcol(1) <= not gcol(1);
                            else gx1 <= gx1 + 1; end if;
                        end if;
                        if fcnt(2 downto 0) = "000" then
                            if gx2 = ncols_m1 then
                                gx2 <= (others => '0'); gcol(2) <= not gcol(2);
                            else gx2 <= gx2 + 1; end if;
                        end if;
                    end if;
                    -- gulls ride just above the horizon (geometry-derived)
                    -- so their reflections land inside the ocean strip
                    gy0 <= resize(unsigned(skyband_q(11 downto 6)), 8)
                           - 3 + ("0000000" & fcnt(5));
                    gy1 <= resize(unsigned(skyband_q(11 downto 6)), 8)
                           - 6 + ("0000000" & fcnt(6));
                    gy2 <= resize(unsigned(skyband_q(11 downto 6)), 8) - 9;
                    -- twice the horizon row: reflected cell = 2*horizon - celly,
                    -- so the SAME gy0/1/2 targets locate the mirror images too
                    ghz2 <= resize(unsigned(skyband_q(11 downto 6)) & '0', 8);
                    -- birds roost when the sky is nearly dark
                    if lp12 < to_unsigned(96, 10) then
                        gulls_on <= '0';
                    else
                        gulls_on <= s_gulls;
                    end if;

                    -- physics rate: K6 zones 1/4x, 1/2x, 1x, 2x
                    -- physics rate: 4/8/16/32 ticks per field.  The display
                    -- reads its own snapshot banks, so ticks are free to run
                    -- beside active video -- the budget is the whole frame
                    case lk6(9 downto 8) is
                        when "00"   => v_ticks := "000001";
                        when "01"   => v_ticks := "000010";
                        when "10"   => v_ticks := "000011";
                        when others => v_ticks := "000100";
                    end case;
                    fsm_ticks <= v_ticks;
                    kick_t    <= not kick_t;
                    vstep <= vstep + 1;
                when 5 =>
                    st_d  <= sun_q15 - skyband_q;
                    vstep <= vstep + 1;
                when 6 =>
                    -- (mirror / streak flags from st_d = sun - horizon, the same
                    -- compares as sun+320 < horizon etc., one adder instead of four)
                    if st_d < to_signed(-320, 15) then mir_on <= '1';
                    else                               mir_on <= '0'; end if;
                    if st_d >= to_signed(-320, 15) and st_d < to_signed(-64, 15) then
                        tri_on <= '1';
                    else
                        tri_on <= '0';
                    end if;
                    stk_len15 <= shift_left(st_d + to_signed(320, 15), 2);
                    -- stars: first ones once the sun is 2/3 set (top 2 rows
                    -- above the horizon), full field 4 rows below it; Night
                    -- always full.  Geometry-relative, so every pixel size
                    -- and resolution sets at the same point.
                    if s_night = '1' then
                        star_thr <= to_unsigned(48, 6);
                    elsif st_d < to_signed(-128, 15) then
                        star_thr <= (others => '0');
                    elsif st_d > to_signed(255, 15) then
                        star_thr <= to_unsigned(48, 6);
                    else
                        v_sd := st_d + to_signed(128, 15);
                        star_thr <= unsigned(v_sd(8 downto 3));
                    end if;
                    vstep <= vstep + 1;

                ----------------------------------------------------------------
                -- CRAB
                ----------------------------------------------------------------
                when 7 =>
                    -- centre column: the crab's, or a random pop-up candidate
                    -- (column + depth) while hidden, accepted at step 13
                    -- (the candidate is drawn at step 14 and held, so these
                    -- steps evaluate it on the next field)
                    if crab_st = CS_HIDDEN then v_cx := crab_cand;
                    else                        v_cx := crab_x; end if;
                    v_cx := v_cx + 2;                    -- centre column
                    if v_cx < 0 then
                        crab_xc <= (others => '0');
                    else
                        crab_xc <= unsigned(v_cx(7 downto 0));
                    end if;
                    vstep <= vstep + 1;
                when 8 =>
                    if s_size8 = '1' then     -- slope_c 5 / 8
                        crab_cm <= shift_left(resize(crab_xc, 12), 2) + resize(crab_xc, 12);
                    else
                        crab_cm <= shift_left(resize(crab_xc, 12), 3);
                    end if;
                    vstep <= vstep + 1;
                when 9 =>
                    crab_b <= C_B0 - signed(crab_cm);
                    vstep <= vstep + 1;
                when 10 =>
                    crab_bot <= slb_q - resize(crab_b, 15);    -- beach line, Q6 rows
                    vstep <= vstep + 1;
                when 11 =>
                    -- first sand row under the centre column
                    v_cg := crab_bot + to_signed(63, 15);
                    crab_gnd <= unsigned(v_cg(13 downto 6));
                    -- (bit tests, not compares: each compare is a carry chain)
                    if crab_x = 0 then cf_x0 <= '1'; else cf_x0 <= '0'; end if;
                    if crab_x(8) = '1' and crab_x(7 downto 2) /= "111111" then
                        cf_xoff <= '1'; else cf_xoff <= '0'; end if;         -- < -4
                    -- right limit: the screen edge (the whole sand area)
                    if crab_xc >= crab_xlim then cf_xmax <= '1';
                    else                         cf_xmax <= '0'; end if;
                    -- the wash only runs on the left half (w_edge)
                    if signed(resize(crab_xc, 10)) < w_edge then cf_inw <= '1';
                    else                                         cf_inw <= '0'; end if;
                    -- the wash's lower edge in rows (+2 rows of fear)
                    crab_wr <= ("0" & unsigned(wreach(11 downto 6))) + 2;
                    vstep <= vstep + 1;
                when 12 =>
                    crab_nr3  <= n_rows(7 downto 0) - 3;
                    crab_xlim <= ncols_m1 - 2;
                    -- the held pop-up candidate's own top row: its ground was
                    -- computed THIS field (steps 7-11), so depth and ground
                    -- match.  (Using crab_base from step 15 mixed the new
                    -- candidate's depth with the last one's ground, and could
                    -- accept a spot below the screen bottom in 16px.)
                    crab_cbase <= crab_gnd + resize(crab_cd, 8);
                    vstep <= vstep + 1;
                when 13 =>
                    -- 8px -> 16px: halve its cell position so it stays where
                    -- it was on screen (an 8px column/depth can be off the
                    -- 16px screen); drop the old-size hole.  16 -> 8 keeps it.
                    s8_prev <= s_size8;
                    if s8_prev = '1' and s_size8 = '0' then
                        crab_x   <= shift_right(crab_x, 1);
                        crab_d   <= shift_right(crab_d, 1);
                        hole_lvl <= (others => '0');
                    end if;
                    if crab_d(5 downto 1) = "00000" then cf_dtop <= '1';   -- d <= 1
                    else                                 cf_dtop <= '0'; end if;
                    -- (last field's top row: one field of lag is invisible)
                    if crab_base >= crab_nr3 then cf_dbot <= '1';
                    else                          cf_dbot <= '0'; end if;
                    -- wash coming down over the crest and near it
                    if wph(11 downto 10) = "00" and wreach(14) = '0' and cf_inw = '1'
                       and resize(crab_d, 7) <= crab_wr then
                        cf_wash <= '1';
                    else
                        cf_wash <= '0';
                    end if;
                    -- pop-up candidate: on screen and inside the sand (its base
                    -- row was computed for it at step 15 of the last field)
                    if cf_xmax = '0' and crab_cd /= 0 and crab_cbase < crab_nr3 then
                        cf_ok <= '1';
                    else
                        cf_ok <= '0';
                    end if;
                    vstep <= vstep + 1;
                when 14 =>
                    case crab_st is
                        when CS_HIDDEN =>
                            if s_crab = '1' and cf_ok = '1' then
                                crab_x    <= crab_cand;
                                crab_d    <= crab_cd;
                                crab_sink <= "11";
                                crab_sub  <= (others => '0');
                                crab_frm  <= "00";
                                crab_st   <= CS_EMERGE;
                            else
                                -- anywhere in the sand: column, depth
                                if s_size8 = '1' then
                                    crab_cand <= signed(resize(crab_lfsr(7 downto 0), 9));
                                    crab_cd   <= crab_lfsr(13 downto 8);
                                else
                                    crab_cand <= signed(resize(crab_lfsr(6 downto 0), 9));
                                    crab_cd   <= resize(crab_lfsr(12 downto 8), 6);
                                end if;
                            end if;
                        when CS_EMERGE =>
                            if s_crab = '0' then
                                crab_sub <= (others => '0');
                                crab_st  <= CS_BURROW;
                            elsif crab_sub = 7 then
                                crab_sub <= (others => '0');
                                if crab_sink = 0 then
                                    crab_dir <= crab_lfsr(0);
                                    crab_vy  <= "00";
                                    crab_tmr <= (others => '0');
                                    crab_st  <= CS_WALK;
                                else
                                    crab_sink <= crab_sink - 1;
                                end if;
                            else
                                crab_sub <= crab_sub + 1;
                            end if;
                        when CS_BURROW =>
                            crab_frm <= "00";
                            if crab_sub = 7 then
                                crab_sub <= (others => '0');
                                if crab_sink = 3 then
                                    crab_st <= CS_HIDDEN;
                                else
                                    crab_sink <= crab_sink + 1;
                                end if;
                            else
                                crab_sub <= crab_sub + 1;
                            end if;
                        when CS_OFF =>
                            if s_crab = '0' then
                                crab_st <= CS_HIDDEN;
                            elsif crab_tmr = 0 then
                                crab_x   <= to_signed(-5, 9);
                                crab_dir <= '1';
                                crab_st  <= CS_WALK;
                            else
                                crab_tmr <= crab_tmr - 1;
                            end if;
                        when CS_PAUSE =>
                            if fcnt(4) = '1' then crab_frm <= "10";   -- claws wave
                            else                  crab_frm <= "00"; end if;
                            if s_crab = '0' then
                                crab_sink <= "00";
                                crab_sub  <= (others => '0');
                                crab_st   <= CS_BURROW;
                            elsif cf_wash = '1' then
                                crab_tmr <= to_unsigned(24, 8);
                                crab_st  <= CS_WALK;
                            elsif crab_tmr = 0 then
                                crab_dir <= crab_lfsr(1);
                                crab_st  <= CS_WALK;
                            else
                                crab_tmr <= crab_tmr - 1;
                            end if;
                        when others =>            -- CS_WALK
                            if s_crab = '0' then
                                crab_sink <= "00";
                                crab_sub  <= (others => '0');
                                if cf_xoff = '1' then crab_st <= CS_HIDDEN;
                                else                  crab_st <= CS_BURROW; end if;
                            else
                                -- the wash closing in: run down across the sand,
                                -- ~1.8x, and keep going a few steps after
                                v_dir := crab_dir;
                                v_vy  := crab_vy;
                                v_tmr := crab_tmr;
                                v_st  := CS_WALK;
                                if cf_wash = '1' then
                                    v_tmr := to_unsigned(24, 8);
                                elsif v_tmr /= 0 then
                                    v_tmr := v_tmr - 1;
                                end if;
                                if cf_wash = '1' or crab_tmr /= 0 then v_run := '1';
                                else                                   v_run := '0'; end if;
                                -- speed: a step every 7 fields (4 running)
                                if (v_run = '1' and (crab_sub(3) = '1' or crab_sub(2) = '1'
                                                     or (crab_sub(1) = '1' and crab_sub(0) = '1')))
                                   or crab_sub(3) = '1' or (crab_sub(2) = '1' and crab_sub(1) = '1') then
                                    crab_sub <= (others => '0');
                                    crab_frm <= "0" & not crab_frm(0);
                                    if v_run = '1' then v_vy := "01"; end if;
                                    -- vertical: bounce off the beach line and the
                                    -- screen bottom (running at the bottom = sideways)
                                    if v_vy = "01" and cf_dbot = '1' then
                                        v_vy := "00";
                                    elsif v_vy = "11" and cf_dtop = '1' then
                                        v_vy := "01";
                                    end if;
                                    if v_vy = "01" then crab_d <= crab_d + 1;
                                    elsif v_vy = "11" then crab_d <= crab_d - 1; end if;
                                    -- sideways
                                    v_mv := '0';
                                    if v_dir = '1' then
                                        if cf_xmax = '1' then v_dir := '0';
                                        else                  v_mv := '1'; end if;
                                    elsif cf_x0 = '1' and v_run = '0'
                                          and crab_lfsr(4 downto 2) /= "000" then
                                        v_dir := '1';                -- usually turns at the edge
                                    elsif cf_xoff = '1' then
                                        v_tmr := '1' & crab_lfsr(12 downto 6);
                                        v_st  := CS_OFF;             -- walked off: back in 2-4 s
                                    else
                                        v_mv := '1';
                                    end if;
                                    if v_mv = '1' then               -- one shared +-1 adder
                                        if crab_dir = '1' then crab_x <= crab_x + 1;
                                        else                   crab_x <= crab_x - 1; end if;
                                    end if;
                                    -- wander (not while running)
                                    if v_run = '0' and v_st = CS_WALK then
                                        if crab_lfsr(3 downto 0) = "0000" then
                                            v_tmr := '0' & (crab_lfsr(13 downto 8) or "001000") & '1';
                                            v_st  := CS_PAUSE;       -- stops for 0.3-2.1 s
                                        elsif crab_lfsr(7 downto 4) = "0000" then
                                            v_dir := not v_dir;
                                        end if;
                                        if crab_lfsr(8 downto 7) = "00" then
                                            case crab_lfsr(11 downto 10) is
                                                when "01"   => v_vy := "01";  -- drift down
                                                when "11"   => v_vy := "11";  -- drift up
                                                when others => v_vy := "00";  -- level
                                            end case;
                                        end if;
                                    end if;
                                else
                                    crab_sub <= crab_sub + 1;
                                end if;
                                crab_dir <= v_dir;
                                crab_vy  <= v_vy;
                                crab_tmr <= v_tmr;
                                crab_st  <= v_st;
                            end if;
                    end case;
                    vstep <= vstep + 1;
                when 15 =>
                    -- render rows; sinking shifts it down under its own bottom
                    -- row, so it disappears into the sand legs-first
                    if crab_st = CS_HIDDEN then
                        v_cbase := crab_gnd + resize(crab_cd, 8);   -- candidate
                    else
                        v_cbase := crab_gnd + resize(crab_d, 8);
                    end if;
                    crab_base <= v_cbase;
                    crab_top  <= v_cbase + resize(crab_sink, 8);
                    if crab_st = CS_HIDDEN or crab_st = CS_OFF then crab_vis <= '0';
                    else                                            crab_vis <= '1'; end if;
                    -- its hole: the row under its feet, open while it digs in
                    -- or out, then drying like wet sand (a step / 12 fields)
                    if crab_st = CS_BURROW or crab_st = CS_EMERGE then
                        hole_x   <= crab_x;
                        hole_row <= v_cbase + 2;
                        hole_lvl <= (others => '1');
                    elsif sw_tick = '1' and hole_lvl /= 0 then
                        hole_lvl <= hole_lvl - 1;
                    end if;
                    if hole_lvl >= 11 then    hole_shd <= "10";   -- SANDW
                    elsif hole_lvl >= 6 then  hole_shd <= "01";   -- SANDM
                    else                      hole_shd <= "00";   -- SAND2
                    end if;
                    vstep <= vstep + 1;
                when others =>
                    null;
            end case;
        end if;
    end process p_vseq;

    ----------------------------------------------------------------------------
    -- Physics FSM: runs entirely inside vertical blanking.
    ----------------------------------------------------------------------------
    p_fsm : process(clk)
        variable v_d    : signed(11 downto 0);
        variable v_dmax : signed(11 downto 0);
        variable v_un   : signed(11 downto 0);
        variable v_dep  : signed(11 downto 0);
        variable v_f    : unsigned(9 downto 0);
        variable v_m    : unsigned(7 downto 0);
        variable v_fin  : signed(11 downto 0);
        variable v_fp   : unsigned(9 downto 0);
        variable v_i1   : unsigned(9 downto 0);
        variable v_cy   : signed(10 downto 0);
        variable v_lr   : unsigned(9 downto 0);
        variable v_iw   : signed(6 downto 0);
        variable v_pa   : unsigned(9 downto 0);
        variable v_ph11 : unsigned(10 downto 0);
        variable v_ae   : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            fsm_ce <= not fsm_ce;
            -- the FSM's enable, registered: fsm_ce's half-rate cadence, held
            -- off while the serial multiplier works.  One register drives
            -- every FSM CEN (the stall comparator in front of that fan-out
            -- was a 26-29 ns path); m_stall only feeds this one D input.
            fsm_en <= (not fsm_ce) and (not m_stall);
            if fsm_en = '1' then
            if field_seen /= field_tgl then
                field_seen <= field_tgl;
                ticks_run  <= (others => '0');
            end if;
            e_we   <= '0';
            u_we   <= '0';
            fw_we  <= '0';
            ed_we  <= '0';
            fwd_we <= '0';
            grad_we  <= "000";

            case fst is
                when FI =>
                    fsm_run <= '0';
                    if kick_seen /= kick_t
                       and ticks_run < fsm_ticks then
                        kick_seen  <= kick_t;
                        fsm_run    <= '1';
                        ticks_left <= fsm_ticks;
                        ncf        <= n_cols;
                        vi         <= (others => '0');
                        fst        <= CP0;
                    end if;

                ----------------------------------------------------------------
                -- Copy pass: snapshot live eta/foam/wet into the display banks
                -- (runs right after vsync, ~3 clocks per column)
                ----------------------------------------------------------------
                when CP0 =>
                    e_ra_f  <= resize(vi, 8);
                    fw_ra_f <= resize(vi, 8);
                    fst     <= CP1;
                when CP1 =>
                    fst <= CP2;
                when CP2 =>
                    ed_we  <= '1';
                    ed_wa  <= resize(vi, 8);
                    ed_wd  <= e_rd;
                    fwd_we <= '1';
                    fwd_wa <= resize(vi, 8);
                    fwd_wd <= fw_rd;
                    if vi = ncf - 1 then
                        vi  <= (others => '0');
                        fst <= FP0;
                    else
                        vi  <= vi + 1;
                        fst <= CP0;
                    end if;

                ----------------------------------------------------------------
                -- PREP: knob curves, LFSR noise, wave/chop/surge oscillators,
                -- forced boundary height.  Shared-mult schedule (operands set
                -- in state N are readable as mp in state N+2):
                --   FP3: amp*amp_mod  FP5: sin1*amp_eff  FP6: sin2*chop
                --   FP7: sin3*surge
                ----------------------------------------------------------------
                when FP0 =>
                    t_r    <= lp12(5 downto 2);
                    p_inc1 <= to_unsigned(2, 5) + ("0" & lk1(9 downto 6));
                    p_inc3 <= to_unsigned(1, 4) + resize(lk1(9 downto 8), 4);
                    p_camp <= lk4(9 downto 2);
                    if s_storm = '1' then
                        -- Storm: +50% wave energy, hotter foam
                        v_pa := ("0" & lk5(9 downto 1)) + ("00" & lk5(9 downto 2));
                        if v_pa > to_unsigned(511, 10) then
                            v_pa := to_unsigned(511, 10);
                        end if;
                        p_amp <= v_pa(8 downto 0);
                        p_inj <= to_unsigned(160, 8) + ("00" & lk3(9 downto 4));
                    else
                        p_amp <= lk5(9 downto 1);
                        p_inj <= to_unsigned(96, 8) + ("0" & lk3(9 downto 3));
                    end if;
                    surge_amp <= ("0" & lk5(9 downto 2)) + to_unsigned(32, 9);
                    fst       <= FP1;
                when FP1 =>
                    lfsr <= lfsr(14 downto 0)
                            & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
                    p_inc2 <= ("0" & p_inc1 & "0") + resize(p_inc1(4 downto 1), 7);
                    -- wave rate = knob rate + THIS WAVE's rolled period
                    -- offset (+-8 -> ~2:1 period spread wave to wave) + a
                    -- per-tick +-1 dither.  Registered ahead of the phase add.
                    v_iw := signed(resize(p_inc1, 7)) + resize(wave_jit, 7);
                    if v_iw < to_signed(1, 7) then
                        v_iw := to_signed(1, 7);
                    end if;
                    v_i1 := resize(unsigned(v_iw(5 downto 0)), 10);
                    if lfsr(0) = '1' then v_i1 := v_i1 + 1; end if;
                    if lfsr(1) = '1' and v_i1 /= 0 then v_i1 := v_i1 - 1; end if;
                    inc_j  <= v_i1;
                    th_mid <= signed(resize(surge_amp(8 downto 1), 12));
                    fst    <= FP2;
                when FP2 =>
                    -- wrap = carry out of the 11-bit add (no compare chain)
                    v_ph11 := ("0" & ph1) + ("0" & inc_j);
                    ph1 <= v_ph11(9 downto 0);
                    if v_ph11(10) = '1' then
                        wave_new <= '1';
                    end if;
                    ph2 <= ph2 + resize(p_inc2, 10);
                    ph3 <= ph3 + resize(p_inc3, 10);
                    th_hi <= th_mid + signed(resize(surge_amp(8 downto 2), 12));
                    fst <= FP3;
                when FP3 =>
                    -- each new wave rolls its own height (x0.5..x1.5) and
                    -- its own period (+-8 on the phase rate)
                    if wave_new = '1' then
                        amp_mod  <= to_unsigned(128, 9) + ("0" & lfsr(7 downto 0));
                        wave_jit <= signed('0' & lfsr(11 downto 8)) - to_signed(8, 5);
                        wave_new <= '0';
                    end if;
                    sky_a  <= "011" & ph1(9 downto 4);   -- sine corner
                    fst    <= FP4;
                when FP4 =>
                    sin_v1 <= signed(sky_q(7 downto 0));   -- sine(ph1), issued in FP3
                    sky_a  <= "011" & ph2(9 downto 4);
                    ma     <= signed(resize(p_amp, 12));
                    mb     <= signed(resize(amp_mod, 10));
                    fst    <= FP5;
                when FP5 =>
                    sin_v2 <= signed(sky_q(7 downto 0));   -- sine(ph2), issued in FP4
                    sky_a  <= "011" & ph3(9 downto 4);
                    fst    <= FP6;
                when FP6 =>
                    sin_v3 <= signed(sky_q(7 downto 0));   -- sine(ph3), issued in FP5
                    -- mp = amp * amp_mod (Q8 factor)
                    v_ae := unsigned(mp(17 downto 8));
                    if v_ae > to_unsigned(511, 10) then
                        v_ae := to_unsigned(511, 10);
                    end if;
                    ma  <= resize(sin_v1, 12);
                    mb  <= signed(resize(v_ae, 10));
                    fst <= FP7;
                when FP7 =>
                    ma  <= resize(sin_v2, 12);
                    mb  <= signed(resize(p_camp, 10));
                    fst <= FP8;
                when FP8 =>
                    swell_a <= resize(shift_right(mp, 7), 12);   -- wave component
                    ma      <= resize(sin_v3, 12);
                    mb      <= signed(resize(surge_amp, 10));
                    fst     <= FP9;
                when FP9 =>
                    chop_v <= resize(shift_right(mp, 9), 12);
                    fst    <= FP10;
                when FP10 =>
                    -- surge level 0..surge_amp (half-offset sine)
                    surge_off <= resize(shift_right(mp, 8), 12)
                                 + signed(resize(surge_amp(8 downto 1), 12));
                    fst <= FP11;
                when FP11 =>
                    -- swell too big -> waves lower: damp the wave component
                    -- through the top half of the surge cycle (thresholds
                    -- pre-registered in FP1/FP2)
                    if surge_off > th_hi then
                        swell_a <= swell_a - shift_right(swell_a, 2);
                    end if;
                    fst <= FP12;
                when FP12 =>
                    swell <= swell_a + chop_v + surge_off;
                    fst   <= SK0;

                ----------------------------------------------------------------
                -- Sunset colour engine, one channel at a time through ONE ROM
                -- port and ONE lerp (same arithmetic as the old parallel one):
                -- per channel, the sky top (hi) and horizon glow lerp between
                -- P12 keyframes k and k+1 with t = P12(5:2), then the 8
                -- gradient bands hi -> glow go to that channel's band RAM,
                -- then the ocean-strip colour (lo).  mp is valid in the state
                -- after ma is set (the FSM stalls on the serial multiplier).
                ----------------------------------------------------------------
                when SK0 =>
                    mb    <= signed(resize(t_r, 10));
                    sk_ch <= "00";
                    sk_tg <= "00";
                    fst   <= SKA;
                when SKA =>
                    sky_a <= s_night & sk_tg & sk_ch & lp12(9 downto 6);
                    fst   <= SKB;
                when SKB =>
                    sk_a <= sky_q;                                 -- keyframe k
                    if lp12(9 downto 6) = "1111" then
                        sky_a <= s_night & sk_tg & sk_ch & "1111";
                    else
                        sky_a <= s_night & sk_tg & sk_ch & (lp12(9 downto 6) + 1);
                    end if;
                    fst <= SKC;
                when SKC =>
                    ma  <= resize(signed('0' & sky_q) - signed('0' & sk_a), 12);
                    fst <= SKD;
                when SKD =>
                    v_lr := resize(unsigned(signed('0' & sk_a)
                                            + resize(shift_right(mp, 4), 11)), 10);
                    if sk_tg = "00" then                           -- sky top
                        sk_hi <= v_lr;
                        sk_tg <= "10";
                        fst   <= SKA;
                    elsif sk_tg = "10" then                        -- glow: bands
                        g_d   <= shift_right(signed('0' & v_lr) - signed('0' & sk_hi), 3);
                        g_acc <= signed('0' & sk_hi);
                        gk    <= (others => '0');
                        fst   <= SKG;
                    else                                           -- ocean strip
                        case sk_ch is
                            when "00"   => skylo_y <= std_logic_vector(v_lr);
                            when "01"   => skylo_u <= std_logic_vector(v_lr);
                            when others => skylo_v <= std_logic_vector(v_lr);
                        end case;
                        if sk_ch = "10" then
                            fst <= FE0;
                        else
                            sk_ch <= sk_ch + 1;
                            sk_tg <= "00";
                            fst   <= SKA;
                        end if;
                    end if;
                when SKG =>
                    -- clamp: floor-rounded negative deltas can walk a near-black
                    -- band below zero, and the unsigned slice would wrap to WHITE
                    v_cy := g_acc;
                    if v_cy < 0 then v_cy := (others => '0'); end if;
                    case sk_ch is
                        when "00"   => grad_we <= "001";
                        when "01"   => grad_we <= "010";
                        when others => grad_we <= "100";
                    end case;
                    grad_wa <= gk;
                    grad_wd <= "000000" & std_logic_vector(v_cy(9 downto 0));
                    g_acc   <= g_acc + g_d;
                    if gk = "111" then
                        sk_tg <= "01";                             -- then the ocean strip
                        fst   <= SKA;
                    else
                        gk <= gk + 1;
                    end if;
                when FE0 =>
                    -- span/2 keeps ma inside the 12-bit mult input even in 8px
                    -- modes (skyband ~2x larger); mb = (1023-P12)/2 fits the
                    -- 10-bit input; the /4 vs the old /2 is recovered by the
                    -- one-bit-smaller shift in FE2 -- 16px result unchanged
                    ma <= resize(shift_right(skyband_q + to_signed(448, 15), 1), 12);
                    mb <= signed(resize(shift_right(to_unsigned(1023, 10) - lp12, 1), 10));
                    fst <= FE1;
                when FE1 =>
                    fst <= FE2;
                when FE2 =>
                    sun_q_f <= to_unsigned(64, 12)
                               + resize(unsigned(mp(19 downto 8)), 12);
                    fst <= FVL0;
                ----------------------------------------------------------------
                -- V pass: velocity update, interfaces 0 .. ncf-2
                ----------------------------------------------------------------
                when FVL0 =>
                    e_ra_f  <= (others => '0');
                    vi      <= (others => '0');
                    b_raw   <= C_B0;
                    b_eff_i <= C_B0 - resize(tide_q, 12);
                    fst     <= FVL1;
                when FVL1 =>
                    fst <= FVL2;
                when FVL2 =>
                    eta_i <= resize(signed(e_rd(11 downto 0)), 12);
                    fst   <= FV0;

                when FV0 =>
                    e_ra_f <= resize(vi + 1, 8);
                    u_ra_f <= resize(vi, 8);
                    if b_raw - slope_c < b_floor then
                        b_nx_raw <= b_floor;
                    else
                        b_nx_raw <= b_raw - slope_c;
                    end if;
                    fst <= FV1;
                when FV1 =>
                    b_eff_ip1 <= b_nx_raw - resize(tide_q, 12);
                    fst       <= FV2;
                when FV2 =>
                    eta_ip1 <= resize(signed(e_rd(11 downto 0)), 12);
                    u_i     <= resize(signed(u_rd(11 downto 0)), 12);
                    v_d := eta_i - b_eff_i;
                    if v_d < 0 then v_d := (others => '0'); end if;
                    if v_d > C_DCLAMP then v_d := C_DCLAMP; end if;
                    d_i <= v_d;
                    fst <= FV3;
                when FV3 =>
                    v_d := eta_ip1 - b_eff_ip1;
                    if v_d < 0 then v_d := (others => '0'); end if;
                    if v_d > C_DCLAMP then v_d := C_DCLAMP; end if;
                    d_ip1 <= v_d;
                    de_r  <= eta_i - eta_ip1;
                    fst   <= FV4;
                when FV4 =>
                    -- split the deep cone: register the two shifted terms, add
                    -- them next state (each sshr is its own negate/shift chain)
                    -- friction and gravity MUST use symmetric shifts: floor
                    -- shifts never damp small positive u (shift_right(+u,n)=0
                    -- for u < 2^n) while always nudging negative u -- a
                    -- constant energy pump that blew the sim up (v3.4 bug)
                    if d_i > d_ip1 then v_dmax := d_i; else v_dmax := d_ip1; end if;
                    grav_r <= sshr(de_r, 3);
                    if v_dmax < to_signed(128, 12) then
                        fric_r <= sshr(u_i, 7);
                    else
                        fric_r <= sshr(u_i, 7);
                    end if;
                    fst <= FV5;
                when FV5 =>
                    un_r <= u_i + grav_r - fric_r;
                    fst  <= FV6;
                when FV6 =>
                    v_un := un_r;
                    if v_un > C_UCLAMP then v_un := C_UCLAMP; end if;
                    if v_un < -C_UCLAMP then v_un := -C_UCLAMP; end if;
                    if d_i = 0 and d_ip1 = 0 then
                        v_un := (others => '0');
                    end if;
                    u_we <= '1';
                    u_wa <= resize(vi, 8);
                    u_wd <= std_logic_vector(resize(v_un, 16));

                    eta_i   <= eta_ip1;
                    b_raw   <= b_nx_raw;
                    b_eff_i <= b_eff_ip1;
                    if vi = ncf - 2 then
                        fst <= FFL0;
                    else
                        vi  <= vi + 1;
                        fst <= FV0;
                    end if;

                ----------------------------------------------------------------
                -- F pass: conservative flux + diffusion + foam/wet,
                -- finalizes columns 0 .. ncf-2 (ncf-1 is the forced boundary)
                ----------------------------------------------------------------
                when FFL0 =>
                    e_ra_f   <= (others => '0');
                    fw_ra_f  <= (others => '0');
                    vi       <= (others => '0');
                    b_raw    <= C_B0;
                    b_eff_i  <= C_B0 - resize(tide_q, 12);
                    qf_prev  <= (others => '0');
                    df_prev  <= (others => '0');
                    sub_next <= (others => '0');
                    add_next <= (others => '0');
                    fst      <= FFL1;
                when FFL1 =>
                    fst <= FFL2;
                when FFL2 =>
                    eta_pend  <= resize(signed(e_rd(11 downto 0)), 12);
                    foam_pend <= "0" & unsigned(fw_rd(15 downto 8));
                    wet_pend  <= unsigned(fw_rd(7 downto 0));
                    fst       <= FF0;

                when FF0 =>
                    e_ra_f  <= resize(vi + 1, 8);
                    fw_ra_f <= resize(vi + 1, 8);
                    u_ra_f  <= resize(vi, 8);
                    if b_raw - slope_c < b_floor then
                        b_nx_raw <= b_floor;
                    else
                        b_nx_raw <= b_raw - slope_c;
                    end if;
                    fst <= FF1;
                when FF1 =>
                    b_eff_ip1 <= b_nx_raw - resize(tide_q, 12);
                    fst       <= FF2;
                when FF2 =>
                    eta_ip1 <= resize(signed(e_rd(11 downto 0)), 12);
                    u_i     <= resize(signed(u_rd(11 downto 0)), 12);
                    fw_jp1  <= fw_rd;
                    v_d := eta_pend - b_eff_i;
                    if v_d < 0 then v_d := (others => '0'); end if;
                    if v_d > C_DCLAMP then v_d := C_DCLAMP; end if;
                    d_i <= v_d;
                    fst <= FF3;
                when FF3 =>
                    v_d := eta_ip1 - b_eff_ip1;
                    if v_d < 0 then v_d := (others => '0'); end if;
                    if v_d > C_DCLAMP then v_d := C_DCLAMP; end if;
                    d_ip1   <= v_d;
                    slope_w <= eta_ip1 - eta_pend;
                    neg_sl  <= eta_pend - eta_ip1;
                    fst     <= FF4;
                when FF4 =>
                    if u_i(11) = '1' then v_dep := d_ip1; else v_dep := d_i; end if;
                    ma   <= u_i;
                    mb   <= signed(resize(unsigned(v_dep(9 downto 4)), 10));
                    df_j <= sshr(neg_sl, 5);
                    -- breaking starts sooner: deeper trigger zone, gentler
                    -- slope and speed thresholds than v3.0
                    if d_i > 0 and d_i < to_signed(512, 12)
                       and slope_w > to_signed(8, 12)
                       and u_i < to_signed(-6, 12) then
                        brk_r <= '1';
                    else
                        brk_r <= '0';
                    end if;
                    -- whitecap: any steep advancing front seeds a little foam
                    -- regardless of depth, so the whole wave top carries bits
                    if slope_w > to_signed(24, 12)
                       and u_i < to_signed(-6, 12) then
                        cap_r <= '1';
                    else
                        cap_r <= '0';
                    end if;
                    fst <= FF5;
                when FF5 =>
                    -- foam: inject on break, exchange with the right neighbour
                    v_f := resize(foam_pend, 10) + resize(add_next, 10);
                    if brk_r = '1' then
                        v_f := v_f + resize(p_inj, 10);
                    elsif cap_r = '1' then
                        v_f := v_f + resize(p_inj(7 downto 2), 10);
                    end if;
                    add_next <= (others => '0');
                    sub_next <= (others => '0');
                    if u_i < to_signed(-32, 12) then
                        -- leftward flow drags the neighbour's foam onto us
                        v_m := "00" & unsigned(fw_jp1(15 downto 10));
                        v_f := v_f + resize(v_m, 10);
                        sub_next <= v_m;
                    elsif u_i > to_signed(32, 12) then
                        -- rightward flow carries our foam away
                        v_m := resize(shift_right(v_f, 2), 8);
                        v_f := v_f - resize(v_m, 10);
                        add_next <= v_m;
                    end if;
                    if v_f > to_unsigned(255, 10) then
                        v_f := to_unsigned(255, 10);
                    end if;
                    foam_adj <= resize(v_f, 9);
                    fst      <= FF6;
                when FF6 =>
                    -- pre-sum the terms that are already registered so FF7 is
                    -- only two adders deep (this cone was the HD critical path)
                    qf_j_r <= resize(sshr(mp, 5), 12);
                    sum_a  <= eta_pend + qf_prev;
                    sum_b  <= df_prev - df_j;
                    fst    <= FF7;
                when FF7 =>
                    efin_r <= sum_a + sum_b - qf_j_r;
                    fst    <= FF8;
                when FF8 =>
                    v_fin := efin_r;
                    if v_fin > to_signed(1023, 12) then v_fin := to_signed(1023, 12); end if;
                    if v_fin < to_signed(-1023, 12) then v_fin := to_signed(-1023, 12); end if;
                    e_we <= '1';
                    e_wa <= resize(vi, 8);
                    e_wd <= std_logic_vector(resize(v_fin, 16));

                    -- foam decay only on the field's LAST tick so the foam's
                    -- visual wash-away rate is independent of K6 tick rate
                    if ticks_left = to_unsigned(1, 6) then
                        v_fp := resize(foam_adj, 10)
                              - resize(foam_adj(8 downto 6), 10);
                        if v_fp /= 0 then
                            v_fp := v_fp - 1;
                        end if;
                    else
                        v_fp := resize(foam_adj, 10);
                        if v_fp > to_unsigned(255, 10) then
                            v_fp := to_unsigned(255, 10);
                        end if;
                    end if;
                    -- wetness: even a thin swash soaks the sand, then it dries
                    -- slowly (1/tick ~ 4 s to fully dry) through the darker
                    -- crash-mark browns
                    fw_we <= '1';
                    fw_wa <= resize(vi, 8);
                    fw_wd <= std_logic_vector(v_fp(7 downto 0)) & x"00";

                    -- advance chains
                    eta_pend <= eta_ip1;
                    if unsigned(fw_jp1(15 downto 8)) > resize(sub_next, 8) then
                        foam_pend <= resize(unsigned(fw_jp1(15 downto 8)) - sub_next, 9);
                    else
                        foam_pend <= (others => '0');
                    end if;
                    wet_pend <= unsigned(fw_jp1(7 downto 0));
                    qf_prev  <= qf_j_r;
                    df_prev  <= df_j;
                    b_raw    <= b_nx_raw;
                    b_eff_i  <= b_eff_ip1;
                    if vi = ncf - 2 then
                        fst <= FB0;
                    else
                        vi  <= vi + 1;
                        fst <= FF0;
                    end if;

                ----------------------------------------------------------------
                -- Boundary: force the deep-water swell at the right edge
                ----------------------------------------------------------------
                when FB0 =>
                    e_we  <= '1';
                    e_wa  <= resize(ncf - 1, 8);
                    e_wd  <= std_logic_vector(resize(swell, 16));
                    fw_we <= '1';
                    fw_wa <= resize(ncf - 1, 8);
                    fw_wd <= x"00FF";
                    fst   <= FB1;
                when FB1 =>
                    ticks_run <= ticks_run + 1;
                    if ticks_left > 1
                       and (ticks_run + 1) < fsm_ticks then
                        ticks_left <= ticks_left - 1;
                        fst        <= FP1;
                    else
                        fst <= FI;
                    end if;
            end case;
            end if;  -- fsm_ce
        end if;
    end process p_fsm;

    ----------------------------------------------------------------------------
    -- Shared multiplier (serial, see op_a/m_hi)
    ----------------------------------------------------------------------------
    -- m_stall = operands differ from the last finished product.  Both sides
    -- change only on the FSM's half-rate rhythm (ma/mb on its CE edges;
    -- op_a/op_b released right after one), so stall -> every FSM CEN gets
    -- two clocks like the rest of the FSM cone.  (A busy flag set one clock
    -- after the operand write was a 1-clock, all-FSM-fanout path.)
    m_stall <= '1' when ma /= op_a or mb /= op_b else '0';

    p_mult : process(clk)
        variable v_hi : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if m_run = '0' then
                if m_stall = '1' and m_done = '0' then
                    m_hi  <= (others => '0');     -- start (ma/mb held: FSM stalled)
                    m_lo  <= mb;
                    m_cnt <= (others => '0');
                    m_run <= '1';
                end if;
            else
                -- one Robertson step: add the multiplicand on a 1 bit (the
                -- sign bit, last, subtracts), then shift {hi, lo} right
                v_hi := m_hi;
                if m_lo(0) = '1' then
                    if m_cnt = 9 then v_hi := m_hi - resize(ma, 13);
                    else              v_hi := m_hi + resize(ma, 13); end if;
                end if;
                m_lo <= v_hi(0) & m_lo(9 downto 1);
                m_hi <= v_hi(12) & v_hi(12 downto 1);
                if m_cnt = 9 then
                    m_run  <= '0';
                    m_done <= '1';
                else
                    m_cnt <= m_cnt + 1;
                end if;
            end if;
            -- release on an edge right after an FSM CE edge (fsm_ce='1' now
            -- -> '0' next), so the FSM sees it two clocks later
            if m_done = '1' and fsm_ce = '1' then
                op_a   <= ma;
                op_b   <= mb;
                m_done <= '0';
            end if;
        end if;
    end process p_mult;
    mp <= m_hi(11 downto 0) & m_lo;

    ----------------------------------------------------------------------------
    -- Constant-table ROM ports, each a free-running registered read with a
    -- single read site (every extra read site of a constant array is its own
    -- copy of the table).
    ----------------------------------------------------------------------------
    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            sky_q  <= SKY_CH(to_integer(sky_a));
            dsin_q <= SIN_LUT(to_integer(dsin_a));
            sung_q <= SUNG_ROM(to_integer(s_night & lp12(9 downto 6)));
        end if;
    end process p_roms;

    ----------------------------------------------------------------------------
    -- State BRAMs: canonical 1W1R, read address muxed FSM/display.
    ----------------------------------------------------------------------------
    p_eta_mem : process(clk)
    begin
        if rising_edge(clk) then
            e_rd <= eta_mem(to_integer(e_ra_f));
            if e_we = '1' then
                eta_mem(to_integer(e_wa)) <= e_wd;
            end if;
        end if;
    end process p_eta_mem;

    p_u_mem : process(clk)
    begin
        if rising_edge(clk) then
            u_rd <= u_mem(to_integer(u_ra_f));
            if u_we = '1' then
                u_mem(to_integer(u_wa)) <= u_wd;
            end if;
        end if;
    end process p_u_mem;

    p_fw_mem : process(clk)
    begin
        if rising_edge(clk) then
            fw_rd <= fw_mem(to_integer(fw_ra_f));
            if fw_we = '1' then
                fw_mem(to_integer(fw_wa)) <= fw_wd;
            end if;
        end if;
    end process p_fw_mem;

    -- vertical sky gradient (8 bands, written per frame by FG states)
    p_gradY : process(clk)
    begin
        if rising_edge(clk) then
            gY_rd <= gradY_mem(to_integer(band));
            if grad_we(0) = '1' then
                gradY_mem(to_integer(grad_wa)) <= grad_wd;
            end if;
        end if;
    end process p_gradY;

    p_gradU : process(clk)
    begin
        if rising_edge(clk) then
            gU_rd <= gradU_mem(to_integer(band));
            if grad_we(1) = '1' then
                gradU_mem(to_integer(grad_wa)) <= grad_wd;
            end if;
        end if;
    end process p_gradU;

    p_gradV : process(clk)
    begin
        if rising_edge(clk) then
            gV_rd <= gradV_mem(to_integer(band));
            if grad_we(2) = '1' then
                gradV_mem(to_integer(grad_wa)) <= grad_wd;
            end if;
        end if;
    end process p_gradV;

    -- display snapshot banks (write = FSM copy pass, read = display only)
    p_eta_dsp : process(clk)
    begin
        if rising_edge(clk) then
            e_rd_d <= eta_dsp(to_integer(e_ra_d));
            if ed_we = '1' then
                eta_dsp(to_integer(ed_wa)) <= ed_wd;
            end if;
        end if;
    end process p_eta_dsp;

    p_sw_mem : process(clk)
    begin
        if rising_edge(clk) then
            sw_rd <= sw_mem(to_integer(sw_ra));
            if sw_we = '1' then
                sw_mem(to_integer(sw_wa)) <= sw_wd;
            end if;
        end if;
    end process p_sw_mem;

    p_fw_dsp : process(clk)
    begin
        if rising_edge(clk) then
            fw_rd_d <= fw_dsp(to_integer(fw_ra_d));
            if fwd_we = '1' then
                fw_dsp(to_integer(fwd_wa)) <= fwd_wd;
            end if;
        end if;
    end process p_fw_dsp;

    ----------------------------------------------------------------------------
    -- Display prefetch: cell 0 loaded in the back porch; then, while cell k is
    -- on screen, cell k+1 is fetched into the staging regs (issue at x_loc 1,
    -- latch at 4, promote at the cell boundary).  b runs along the sweep.
    ----------------------------------------------------------------------------
    p_disp : process(clk)
        variable v_dc  : signed(14 downto 0);
        variable v_bot : signed(14 downto 0);
        variable v_wr  : signed(14 downto 0);
        variable v_h, v_s : unsigned(15 downto 0);
        variable v_hcol   : signed(8 downto 0);
        variable v_dcol : signed(9 downto 0);
        variable v_jit : signed(7 downto 0);
        variable v_h2  : unsigned(1 downto 0);
        variable v_hs  : unsigned(3 downto 0);
        variable v_cyc : unsigned(6 downto 0);
        variable v_dy  : unsigned(2 downto 0);
        variable v_drow : signed(14 downto 0);
        variable v_sn  : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            if r_hedge = '1' then
                pf_cnt  <= (others => '0');
                e_ra_d  <= (others => '0');
                fw_ra_d <= (others => '0');
            elsif pf_cnt /= "111" then
                pf_cnt <= pf_cnt + 1;
            end if;

            case to_integer(pf_cnt) is
                when 1 =>
                    -- depth into sand for any cell = row - sea line + b
                    rowmslb <= signed(shift_left(resize(celly_idx, 15), 6)) - slb_q;
                    if celly_idx = hole_row then ho_rowl <= '1';   -- crab hole row
                    else                         ho_rowl <= '0'; end if;

                when 2 =>
                    -- wetness prefetch for cell 0 (b = C_B0)
                    v_wr := rowmslb + resize(C_B0, 15);
                    sw_ra <= resize(unsigned(v_wr(9 downto 6)), 12);
                    sw_an <= resize(unsigned(v_wr(9 downto 6)), 12);
                    if v_wr(14 downto 10) = "00000" then sw_vn <= '1';
                    else sw_vn <= '0'; end if;
                    eta_cur  <= resize(signed(e_rd_d(11 downto 0)), 12);
                    foam_cur <= unsigned(fw_rd_d(15 downto 8));
                    wet_cur  <= unsigned(fw_rd_d(7 downto 0));
                    b_cur    <= C_B0;
                when 4 =>
                    -- derived geometry for cell 0 (no jitter on the sand edge)
                    surf_q   <= slr_q - resize(eta_cur, 15);
                    off_b    <= (others => '0');
                    off_c    <= (others => '0');
                    drop_on  <= '0';
                    -- cell 0 is far from the wash edge: full reach (else it
                    -- carries the previous line's rightmost/submerged value and
                    -- the leftmost column loses its wash/foam)
                    wreach_q <= wreach;
                    st_cur   <= (others => '1');   -- no star in column 0 (saves a hash path)
                    if hole_x = 0 or (hole_x(8) = '1' and hole_x(7 downto 2) = "111111") then
                        ho_colc <= '1'; else ho_colc <= '0'; end if;   -- hole over column 0

                    sw_cur   <= unsigned(sw_rd);
                    sw_acur  <= sw_an;
                    sw_vcur  <= sw_vn;
                    bot_q    <= slb_q - resize(b_cur, 15);
                    v_dc := resize(eta_cur, 15) + resize(tide_q, 15)
                            - resize(b_cur, 15);
                    if v_dc > to_signed(16, 15) then wat_c <= '1';
                    else wat_c <= '0'; end if;
                when others =>
                    null;
            end case;

            sw_we <= '0';
            if data_in.avid = '1' and x_loc = to_unsigned(3, 5)
               and celly_loc = 0 and sw_vcur = '1' then
                sw_we <= '1';
                sw_wa <= sw_acur;
                if p1_uw = '1' then
                    sw_wd <= "1111";                     -- water on it: soaked
                elsif sw_cur /= 0 and sw_tick = '1' then
                    sw_wd <= std_logic_vector(sw_cur - 1);
                else
                    sw_wd <= std_logic_vector(sw_cur);
                end if;
            end if;

            -- promote strobe, registered one pixel early: the promote fans out
            -- to ~200 register muxes, and decoding x_loc = cellw_m1 in front
            -- of that fan-out was a 19 ns path at 98% LC
            -- (virtual lattice: the NEXT pixel is the cell's last one)
            if data_in.avid = '1' and x_a < s_cl1 and x_a >= s_cl2 then
                pr_cell <= '1';
            else
                pr_cell <= '0';
            end if;

            -- the promote first, on its registered strobe alone: as the last
            -- arm of the x_loc chain its 200-way fan-out still waited on the
            -- x_loc = 1/4/5/6 compares (22-28 ns x_loc -> wreach_q/bot_q/surf_q).
            -- pr_cell implies the cell's last clock (x_loc >= 7), so the arms
            -- never overlap.
            if pr_cell = '1' then
                    -- entering the next cell: promote staged state, derive
                    eta_cur  <= eta_nxt;
                    foam_cur <= foam_nxt;
                    wet_cur  <= wet_nxt;
                    b_cur    <= b_nxt;
                    sw_cur   <= unsigned(sw_rd);
                    sw_acur  <= sw_an;
                    sw_vcur  <= sw_vn;
                    v_s      := f_sb16(st_b);
                    st_cur   <= v_s + rotate_left(v_s, 11);
                    ho_colc  <= ho_coln;

                    v_h2 := x_idx(1 downto 0) xor fcnt(6 downto 5);
                    case v_h2 is
                        when "00"   => v_jit := signed(resize(chopj, 8));
                        when "10"   => v_jit := -signed(resize(chopj, 8));
                        when "11"   => v_jit := signed(resize(chopj(5 downto 1), 8));
                        when others => v_jit := (others => '0');
                    end case;

                    -- band-boundary undulation: drifting sine offsets with
                    -- different spatial phases, so the MID and DEEP layers
                    -- roll organically and sometimes crest ABOVE the bands in
                    -- front of them
                    surf_q  <= slr_q - resize(eta_nxt, 15) + resize(v_jit, 15);
                    off_b   <= OFB1_ROM(to_integer(x_idx(3 downto 0)
                                                   + fcnt(7 downto 4)));
                    off_c   <= OFB1_ROM(to_integer(
                                   x_idx(3 downto 0) + 5 + fcnt(8 downto 5)));
                    bot_q   <= slb_q - resize(b_nxt, 15);
                    -- wash edge staged at x_loc 5 (same x_idx, per-field inputs)
                    if wr_n < cap_n then wreach_q <= wr_n;
                    else wreach_q <= cap_n; end if;

                    -- droplet: every ~2 s each column gets a 16-frame window
                    -- (staggered by a static column hash); while local foam is
                    -- high a single foam cell flies the ARC_ROM trajectory
                    v_hs  := x_idx(3 downto 0) xor x_idx(7 downto 4);
                    v_cyc := fcnt(6 downto 0) + (v_hs & "000");
                    v_dy  := ARC_ROM(to_integer(v_cyc(3 downto 0)));
                    if x_idx(5) = '1' then
                        v_dy := '0' & v_dy(2 downto 1);   -- some columns hop low
                    end if;
                    if v_cyc(6 downto 4) = "000" and v_dy /= 0
                       and foam_nxt > to_unsigned(100, 8) then
                        drop_on <= '1';
                    else
                        drop_on <= '0';
                    end if;
                    v_drow := slr_q - resize(eta_nxt, 15) + resize(v_jit, 15)
                              - signed(resize(v_dy & "000000", 15));
                    drop_cell <= unsigned(v_drow(13 downto 6));

                    v_dc := resize(eta_nxt, 15) + resize(tide_q, 15)
                            - resize(b_nxt, 15);
                    if v_dc > to_signed(16, 15) then wat_c <= '1';
                    else wat_c <= '0'; end if;
            elsif data_in.avid = '1' then
                if x_loc = to_unsigned(1, 5) then
                    -- fetch the next cell while this one is on screen
                    if x_idx >= ncols_m1 then
                        e_ra_d  <= ncols_m1;
                        fw_ra_d <= ncols_m1;
                    else
                        e_ra_d  <= x_idx + 1;
                        fw_ra_d <= x_idx + 1;
                    end if;
                elsif x_loc = to_unsigned(3, 5) then
                    dsin_a <= x_idx(5 downto 0) + wph(9 downto 4);
                elsif x_loc = to_unsigned(4, 5) then
                    eta_nxt  <= resize(signed(e_rd_d(11 downto 0)), 12);
                    foam_nxt <= unsigned(fw_rd_d(15 downto 8));
                    wet_nxt  <= unsigned(fw_rd_d(7 downto 0));
                    if b_cur - slope_c < b_floor then
                        b_nxt <= b_floor;
                    else
                        b_nxt <= b_cur - slope_c;
                    end if;
                elsif x_loc = to_unsigned(5, 5) then
                    -- sine-shaped, left-crawling leading edge along the
                    -- diagonal, tapering to a rounded point near the wash's
                    -- far edge.  Cap = columns left of the taper's end, times
                    -- 128 (16px: full until ~5 columns before w_edge, then 0 at
                    -- it) or 32 (8px: starts at the same place, a long soft
                    -- tail to w_edge+15).  Clamp to [-1, 31] BEFORE the shift:
                    -- at 1080 8px (240 cols) the distance passes +-127 and
                    -- wrapped the 15-bit cap (blocky triangle at the left/right
                    -- edge with Tide).  Reach + sine <= ~640, so 31 columns is
                    -- uncapped in both sizes.
                    v_sn := resize(dsin_q, 15);   -- ROM(x_idx + wph), issued at x_loc 3
                    wr_n <= wreach + v_sn;
                    v_dcol := w_tail - signed(resize(x_idx, 10));
                    if v_dcol < 0 then
                        v_dcol := to_signed(-1, 10);
                    elsif v_dcol > to_signed(31, 10) then
                        v_dcol := to_signed(31, 10);
                    end if;
                    if s_size8 = '1' then
                        cap_n <= shift_left(resize(v_dcol, 15), 5);
                    else
                        cap_n <= shift_left(resize(v_dcol, 15), 7);
                    end if;
                    v_h  := celly_idx & (x_idx + 1);           -- star hash, next cell
                    st_a <= f_sb16(v_h xor x"5BD1");
                    -- crab hole: next cell inside its 5 columns?
                    v_hcol := signed(resize(x_idx + 1, 9)) - hole_x;
                    if v_hcol(8 downto 3) = "000000"
                       and not (v_hcol(2) = '1' and v_hcol(1 downto 0) /= "00") then
                        ho_coln <= '1'; else ho_coln <= '0'; end if;

                    v_wr := rowmslb + resize(b_nxt, 15);
                    sw_ra <= (x_idx + 1) & unsigned(v_wr(9 downto 6));
                    sw_an <= (x_idx + 1) & unsigned(v_wr(9 downto 6));
                    if v_wr(14 downto 10) = "00000" then sw_vn <= '1';
                    else sw_vn <= '0'; end if;
                elsif x_loc = to_unsigned(6, 5) then
                    st_b <= st_a + rotate_left(st_a, 7);
                end if;
            end if;
        end if;
    end process p_disp;

    ----------------------------------------------------------------------------
    -- Pixel pipeline
    ----------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_dy, v_dx : unsigned(7 downto 0);
        variable v_g        : std_logic;
        variable v_gw       : std_logic;
        variable v_gr       : std_logic;
        variable v_cr       : unsigned(7 downto 0);
        variable v_dxg      : unsigned(7 downto 0);
        variable v_idx      : unsigned(4 downto 0);
        variable v_sky      : std_logic_vector(1 downto 0);
        variable v_wd       : signed(14 downto 0);
        variable v_sw       : unsigned(3 downto 0);
        variable v_tw       : unsigned(3 downto 0);
        variable v_cdx      : signed(8 downto 0);
        variable v_cdy      : unsigned(7 downto 0);
        variable v_cel      : unsigned(1 downto 0);
        variable v_dsurf    : signed(14 downto 0);
        variable v_db, v_dc2 : signed(14 downto 0);
        variable v_rq       : signed(14 downto 0);
        variable v_dqm      : signed(14 downto 0);
        variable v_shx      : unsigned(7 downto 0);
        variable v_dxs      : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------------
            -- P1: subtractions + membership flags (inputs are cell-rate regs)
            ------------------------------------------------------------------
            row_q <= signed(shift_left(resize(celly_idx, 15), 6));
            p1_fsand <= not sgn15(row_q - bot_q);
            -- wash sheet: depth below the sand surface.  wreach_q already goes
            -- <=0 outside the tide-set extent, so no separate beach gate needed.
            v_wd := row_q - bot_q;                     -- depth into sand
            if v_wd >= to_signed(0, 15) and v_wd < wreach_q then
                p1_uw <= '1';
            else
                p1_uw <= '0';
            end if;
            -- foamy leading band (up to ~2 px into the wash); broken up in the
            -- render (p1_lgap) so most of the crest is bare and the foam reads
            -- as a chaotic mix of short LINES and DOTS, like real swash foam.
            -- p1_lgap='1' = open (no foam).  Look the column up in the aperiodic
            -- FOAM_MASK ROM (dots + short runs, no repeating motif); XOR the low
            -- index bits with the cell row so the pattern shifts as the 2-px
            -- crest sweeps up/down each wave (liveliness, no global time term ->
            -- no flashing).
            p1_lead <= not sgn15(v_wd - wreach_q + to_signed(128, 15));
            p1_lgap <= not FOAM_MASK(to_integer(
                           x_idx(5 downto 0) xor resize(celly_idx(1 downto 0), 6)));
            -- colour bands (DEEP/MID/AQUA/SURF) as clean horizontal stripes
            if v_wd(14) = '0' and ((or std_logic_vector(v_wd(13 downto 9))) = '1'
                                   or (v_wd(8) = '1' and v_wd(7) = '1')) then p1_band <= "11";  -- >= 384
            else p1_band <= unsigned(v_wd(8 downto 7)); end if;
            -- wet sand: this cell's own counter (15 = water just left it).
            -- On the cell row's first line the counter is rewritten this
            -- line, so show the value being written (no 1-line lag).
            if sw_vcur = '1' then
                v_sw := sw_cur;
                if celly_loc = 0 and sw_tick = '1' and sw_cur /= 0 then
                    v_sw := sw_cur - 1;
                end if;
            else
                v_sw := (others => '0');
            end if;
            if v_sw /= 0 then
                p1_res <= '1';
                if v_sw >= to_unsigned(11, 4) then    p1_wl <= "10";  -- SANDW
                elsif v_sw >= to_unsigned(6, 4) then  p1_wl <= "01";  -- SANDM
                else                                  p1_wl <= "00";  -- SAND2
                end if;
            else
                p1_res <= '0';
            end if;

            -- crab: 5x3 sprite in the sand; rows at/below its clip row (its own
            -- bottom when standing) are hidden, so sinking = burrowing
            v_cdx := signed(resize(x_idx, 9)) - crab_x;
            v_cdy := celly_idx - crab_top;
            v_cel := f_crab(crab_frm, v_cdy(1 downto 0), unsigned(v_cdx(2 downto 0)));
            -- 0 <= dx <= 4 and dy <= 2 as bit tests
            -- visible rows: dy + sink <= 2 (3-bit: at sink 3 nothing shows;
            -- a 2-bit "2 - sink" wrapped to 3 there and drew it whole, lower)
            p1_crab  <= '0';
            p1_crabd <= '0';
            if crab_vis = '1' and v_cdx(8 downto 3) = "000000"
               and not (v_cdx(2) = '1' and v_cdx(1 downto 0) /= "00")
               and v_cdy(7 downto 2) = "000000"
               and resize(v_cdy(1 downto 0), 3) + resize(crab_sink, 3) <= 2 then
                p1_crab  <= v_cel(0);
                p1_crabd <= v_cel(1);
            end if;
            if ho_colc = '1' and ho_rowl = '1' and hole_lvl /= 0 then
                p1_hole <= '1';
            else
                p1_hole <= '0';
            end if;

            -- stars: in the upper sky only (P2's last branch); in Day keep
            -- them out of the two glow bands nearest the horizon.  A quarter
            -- are always faint; the rest dip faint for ~0.27 s every ~2.1 s,
            -- each on its own phase (per-star add, never a global XOR, so the
            -- sky never blinks as a whole)
            if st_cur(15 downto 10) = 0 and st_cur(9 downto 4) < star_thr
               and (s_night = '1' or band < 6) then
                p1_star <= '1';
            else
                p1_star <= '0';
            end if;
            v_tw := fcnt(6 downto 3) + st_cur(3 downto 0);
            if st_cur(5 downto 4) = "00" or v_tw(3 downto 1) = "000" then
                p1_sdim <= '1';
            else
                p1_sdim <= '0';
            end if;

            -- all band flags off ONE row_q-surf_q subtract; the aqua/mid and
            -- mid/deep boundaries just add their independent per-column offset
            -- the thresholds are whole rows (multiples of 64), so "x < k*64" is a
            -- bit test (sign, or the bits above the row unit spell < k): one
            -- subtract per boundary instead of a carry chain per threshold
            v_dsurf := row_q - surf_q;
            v_db    := v_dsurf - resize(off_b, 15);
            v_dc2   := v_dsurf - resize(off_c, 15);
            p1_b0    <= not v_dsurf(14);
            p1_b64   <= v_dsurf(14) or not (or std_logic_vector(v_dsurf(13 downto 6)));
            p1_b128  <= v_dsurf(14) or not (or std_logic_vector(v_dsurf(13 downto 7)));
            p1_ba192 <= v_db(14) or (not (or std_logic_vector(v_db(13 downto 8)))
                                     and not (v_db(7) and v_db(6)));
            p1_b256  <= v_db(14) or not (or std_logic_vector(v_db(13 downto 8)));
            p1_bm448 <= v_dc2(14) or (not (or std_logic_vector(v_dc2(13 downto 9)))
                                      and not (v_dc2(8) and v_dc2(7) and v_dc2(6)));
            p1_b512  <= v_dc2(14) or not (or std_logic_vector(v_dc2(13 downto 9)));

            -- crest droplet: single cell at drop_cell while airborne
            if celly_idx = drop_cell and drop_on = '1' then
                p1_drop <= '1';
            else
                p1_drop <= '0';
            end if;


            -- per-layer foam gates: same foam physics, different thresholds,
            -- gap patterns and drift clocks so no two layers match
            if foam_cur > to_unsigned(50, 8)
               and (x_idx(2 downto 1) xor fcnt(6 downto 5)) /= "10" then
                p1_fM <= '1';
            else
                p1_fM <= '0';
            end if;
            if foam_cur > to_unsigned(70, 8)
               and (x_idx(3 downto 2) xor fcnt(8 downto 7)) /= "01" then
                p1_fD <= '1';
            else
                p1_fD <= '0';
            end if;

            -- side-to-side shimmer for everything reflected on the ocean
            if (celly_idx(0) xor fcnt(3)) = '1' then
                if (celly_idx(1) xor fcnt(4)) = '1' then
                    v_shx := x_idx + 1;
                else
                    v_shx := x_idx - 1;
                end if;
            else
                v_shx := x_idx;
            end if;
            v_dxs := v_shx - sun_x0;

            -- mirrored sun disc on the flat ocean: reflect the row across
            -- the horizon and reuse SUN_PAT (vertically symmetric)
            v_rq  := slb2_q - row_q;
            v_dqm := v_rq - sun_q15;
            if mir_on = '1'
               and v_dqm >= to_signed(0, 15) and v_dqm < to_signed(512, 15)
               and v_dxs(7 downto 3) = 0
               and SUN_PAT(63 - to_integer(unsigned(v_dqm(8 downto 6)) & v_dxs(2 downto 0))) = '1' then
                p1_mir <= '1';
            else
                p1_mir <= '0';
            end if;


            -- horizon streak: sun-wide solid column down the ocean strip
            -- while the sun straddles the sky/ocean line
            if tri_on = '1'
               and v_dxs >= to_unsigned(1, 8) and v_dxs <= to_unsigned(6, 8)
               and sgn15(row_q - skyband_q) = '0'
               and sgn15(row_q - skyband_q - stk_len15) = '1' then
                p1_stk <= '1';
            else
                p1_stk <= '0';
            end if;
            -- v >= -64: non-negative, or bits 13..6 all ones (-64 .. -1)
            p1_spr   <= not v_dsurf(14) or (and std_logic_vector(v_dsurf(13 downto 6)));
            p1_sky2  <= not sgn15(row_q - skyband_q);   -- same subtract as the streak
            p1_wat <= wat_c;
            if foam_cur > to_unsigned(96, 8)  then p1_fhi <= '1'; else p1_fhi <= '0'; end if;
            if foam_cur > to_unsigned(32, 8)  then p1_fmid <= '1'; else p1_fmid <= '0'; end if;
            if foam_cur > to_unsigned(120, 8) then p1_fspray <= '1'; else p1_fspray <= '0'; end if;
            if foam_cur > to_unsigned(12, 8)  then p1_fcrest <= '1'; else p1_fcrest <= '0'; end if;

            -- crest-foam gap: ~1 in 4 columns stays open, drifting over time
            if (x_idx(1 downto 0) xor fcnt(6 downto 5)) = "11" then
                p1_gap <= '1';
            else
                p1_gap <= '0';
            end if;

            -- static sand speckle: ~1 in 4 cells takes the second tan
            p1_sand2 <= (x_idx(1) xor celly_idx(0))
                        and (x_idx(0) xor celly_idx(2));

            -- sun/moon: 6x6 circle in an 8x8 cell field
            v_dy := celly_idx - sun_cell;
            v_dx := x_idx - sun_x0;
            if v_dy(7 downto 3) = 0 and v_dx(7 downto 3) = 0
               and SUN_PAT(63 - to_integer(v_dy(2 downto 0) & v_dx(2 downto 0))) = '1' then
                p1_sun <= '1';
            else
                p1_sun <= '0';
            end if;

            -- gulls: per-bird hits, grouped by each bird's CURRENT plumage.
            -- The mirror row (v_cr = 2*horizon - celly) reuses the SAME x-hit
            -- and gy target, so reflections cost only one subtract + 3 compares
            v_g := '0'; v_gw := '0'; v_gr := '0';
            v_cr := ghz2 - celly_idx;
            v_dxg := x_idx - gx0;
            if v_dxg <= to_unsigned(2, 8)
               and (v_dxg(0) = '1' or fcnt(4 downto 3) /= "11") then
                if celly_idx = gy0 then
                    if gcol(0) = '1' then v_gw := '1'; else v_g := '1'; end if;
                end if;
                if v_cr = gy0 then v_gr := '1'; end if;
            end if;
            v_dxg := x_idx - gx1;
            if v_dxg <= to_unsigned(2, 8)
               and (v_dxg(0) = '1' or fcnt(4 downto 3) /= "01") then
                if celly_idx = gy1 then
                    if gcol(1) = '1' then v_gw := '1'; else v_g := '1'; end if;
                end if;
                if v_cr = gy1 then v_gr := '1'; end if;
            end if;
            v_dxg := x_idx - gx2;
            if v_dxg <= to_unsigned(2, 8)
               and (v_dxg(0) = '1' or fcnt(5 downto 4) /= "10") then
                if celly_idx = gy2 then
                    if gcol(2) = '1' then v_gw := '1'; else v_g := '1'; end if;
                end if;
                if v_cr = gy2 then v_gr := '1'; end if;
            end if;
            if gulls_on = '1' then
                p1_gull <= v_g; p1_gullw <= v_gw; p1_gullr <= v_gr;
            else
                p1_gull <= '0'; p1_gullw <= '0'; p1_gullr <= '0';
            end if;

            -- per-column spray hash, reshuffles a few times a second
            p1_sprayh <= x_idx(0) xor x_idx(2) xor fcnt(2)
                         xor (x_idx(1) and fcnt(4));

            ------------------------------------------------------------------
            -- P2: classify -> palette index
            ------------------------------------------------------------------
            if p1_crab = '1' then
                v_idx := C_I_CRAB;
            elsif p1_crabd = '1' then
                v_idx := C_I_CRABD;
            elsif p1_fsand = '1' then
                if p1_uw = '1' then
                    if p1_lead = '1' and p1_lgap = '0' then
                        v_idx := C_I_FOAM;                    -- broken foam edge
                    else
                        v_idx := C_I_DEEP - ("000" & p1_band); -- DEEP MID AQUA SURF
                    end if;
                elsif p1_hole = '1' then
                    case hole_shd is
                        when "10"   => v_idx := C_I_SANDW;
                        when "01"   => v_idx := C_I_SANDM;
                        when others => v_idx := C_I_SAND2;
                    end case;
                elsif p1_res = '1' then
                    case p1_wl is
                        when "10"   => v_idx := C_I_SANDW;    -- wettest
                        when "01"   => v_idx := C_I_SANDM;
                        when others => v_idx := C_I_SAND2;    -- nearly dry
                    end case;
                elsif p1_sand2 = '1' then
                    v_idx := C_I_SAND2;
                else
                    v_idx := C_I_SAND;
                end if;
            elsif p1_wat = '1' and p1_b0 = '1' then
                if p1_b64 = '1' then
                    if p1_fmid = '1' then
                        v_idx := C_I_FOAM;
                    elsif p1_fcrest = '1' and p1_gap = '0' then
                        v_idx := C_I_FOAM;
                    else
                        v_idx := C_I_SURF;
                    end if;
                elsif p1_b128 = '1' then
                    if p1_fhi = '1' then v_idx := C_I_FOAM; else v_idx := C_I_AQUA; end if;
                elsif p1_ba192 = '1' then
                    v_idx := C_I_AQUA;
                elsif p1_bm448 = '1' then
                    -- MID band; its top row carries its own foam line
                    if p1_b256 = '1' and p1_fM = '1' then
                        v_idx := C_I_FOAM;
                    else
                        v_idx := C_I_MID;
                    end if;
                else
                    -- DEEP band; its own foam line on its top row
                    if p1_b512 = '1' and p1_fD = '1' then
                        v_idx := C_I_FOAM;
                    else
                        v_idx := C_I_DEEP;
                    end if;
                end if;
            elsif p1_wat = '1' and p1_fspray = '1' and p1_sprayh = '1'
                  and p1_spr = '1' then
                v_idx := C_I_FOAM;
            elsif p1_wat = '1' and p1_drop = '1' then
                v_idx := C_I_FOAM;               -- airborne crest droplet
            elsif p1_gull = '1' then
                v_idx := C_I_GULL;
            elsif p1_gullw = '1' then
                if s_night = '1' then v_idx := C_I_GULL;
                else v_idx := C_I_GULL2; end if;
            elsif p1_sun = '1' and p1_sky2 = '0' then
                v_idx := C_I_SUN;               -- sun lives in the UPPER sky
            elsif p1_sky2 = '1' then
                -- the OCEAN strip: occludes the setting sun; carries the
                -- mirrored disc / horizon streak
                if p1_mir = '1' or p1_stk = '1' then
                    v_idx := C_I_SUN;
                elsif p1_gullr = '1' then
                    v_idx := C_I_GULL;          -- gull mirrored on the ocean
                else
                    v_idx := C_I_SKYLO;
                end if;
            elsif p1_star = '1' then
                if p1_sdim = '1' then v_idx := C_I_STARD;
                else                  v_idx := C_I_STAR; end if;
            else
                v_idx := C_I_SKYHI;
            end if;
            v_sky := "00";
            if v_idx = C_I_SKYHI then v_sky := "01"; end if;
            if v_idx = C_I_SKYLO then v_sky := "10"; end if;
            if v_idx = C_I_SUN   then v_sky := "11"; end if;
            p2_idx <= v_idx;
            p2_sky <= v_sky;

            ------------------------------------------------------------------
            -- P3: palette lookup
            ------------------------------------------------------------------
            p3_yuv <= PAL_ROM(to_integer(pal_sel & p2_idx));
            p3_sky <= p2_sky;

            ------------------------------------------------------------------
            -- P4: output registers
            ------------------------------------------------------------------
            -- per-frame sunset colours override the static palette: sky
            -- gradient, travelling sun colour (disc/mirror/streak), dimming
            -- ocean deep band
            if p3_sky = "01" then
                p4_y <= gY_rd(9 downto 0);
                p4_u <= gU_rd(9 downto 0);
                p4_v <= gV_rd(9 downto 0);
            elsif p3_sky = "10" then
                p4_y <= skylo_y; p4_u <= skylo_u; p4_v <= skylo_v;
            elsif p3_sky = "11" then
                p4_y <= sun_y_r; p4_u <= sun_u_r; p4_v <= sun_v_r;
            else
                p4_y <= p3_yuv(29 downto 20);
                p4_u <= p3_yuv(19 downto 10);
                p4_v <= p3_yuv(9 downto 0);
            end if;
        end if;
    end process p_pix;

    ----------------------------------------------------------------------------
    -- Sync pipe + output
    ----------------------------------------------------------------------------
    p_pipe : process(clk)
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to C_LAT - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;
        end if;
    end process p_pipe;

    data_out.hsync_n <= pipe(C_LAT - 1).hsync_n;
    data_out.vsync_n <= pipe(C_LAT - 1).vsync_n;
    data_out.field_n <= pipe(C_LAT - 1).field_n;
    data_out.avid    <= pipe(C_LAT - 1).avid;

    data_out.y <= p4_y;
    data_out.u <= p4_u;
    data_out.v <= p4_v;

end architecture pixelbeach;
