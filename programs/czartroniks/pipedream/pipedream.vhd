-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- pipedream.vhd  (v3.0 -- a woven Truchet network of glossy 3D pipes)
--
-- v3.3.2 (2026-10-17): resolution scaling.  The raster counters run in
-- virtual 1080-line units stepped from the video timing ID (registers_in(8)):
-- x 1 / 1.5 / 2 units per pixel (1080 / 720p / SD), y 1 / 1.5 / 4.44 per
-- field line (1080p / 720p / 480i).  Each pixel and line advances the cell
-- counters by the INTEGER part of its step (1 or 2 per pixel; 1..5 unit steps
-- per line, run one every other clock after hsync), so the exact incremental
-- corner squares are kept and every pixel is a nearest-neighbour sample of the
-- 1080p picture: cells, pipes, the rainbow and the swell keep their size
-- relative to screen height (SD = the left 1440 units, 4:3, round elbows
-- stay round).  The old field_n interlace path is gone (field_n is tied high
-- on hardware, so SD was drawn 2:1 tall); the y step carries the interlace.
-- 1080p is bit-identical; latency unchanged (16).
--
-- The screen is tiled by an invisible cell grid and every cell holds one of
-- three Truchet tiles: two quarter-circle elbows (either diagonal pairing) or
-- a straight over/under CROSSING.  Every tile uses each of its four edge
-- midpoints exactly once, so every pipe is a continuous closed loop (or runs
-- off-screen) -- no dead ends, no tees, no seams.  Weave (K2) sets how many
-- tiles are crossings: 0 = pure looping Truchet, 100% = a basket weave whose
-- over/under alternates on a checkerboard.  Rings (S7) replaces the random
-- elbow choice with the checkerboard, closing every elbow pair into a ring
-- (chain mail once crossings join them).
--
-- IMAGE (S11 on, P12 bipolar): the input's luma sets the tube THICKNESS per
-- pixel, so the picture is drawn by the weave itself.  Right of centre the
-- pipes start thin and grow where the picture is bright; left of centre they
-- start fat and thin where it is bright; centre = plain Fatness.  With Image
-- off, P12 drives the same thickness from a slow diagonal swell wave.
--
-- Elbows: the exact cross-section rho - wh comes from a sqrt table rebuilt
-- every vblank (add-only walk), indexed by the incrementally kept rho^2 --
-- within 0.17 px at every scale, so elbows meet each other and the straights
-- flush.  Highlights: the light's projection on the local normal goes
-- through a smooth saturating ROM (soft knee), the same for both families.
--
-- Shading: every tube cross-section is normalised to unit radius (d * 64/r,
-- r = the luma-scaled radius, reciprocal from an EBR ROM), so ONE 256-entry
-- shading ROM gives the same Phong (Gloss) or banded Chrome profile at any
-- thickness.  The highlight sits at the light's projection on the local
-- normal (radial on elbows), clamped so it runs concentric round every bend.
-- Flow pulses circulate ALONG each loop: the phase is continuous at every
-- port (N/S ports = 0, E/W ports = half a period) and its direction follows
-- the Truchet 2-colouring, so a loop's pulses all travel the same way round.
--
-- Controls:
--   K1  Scale      (cell size, 32..158 px, always even)
--   K2  Weave      (pure loops -> all crossings, over/under)
--   K3  Fatness    (tube radius)
--   K4  Flow       (liquid-light pulses circulating along the loops)
--   K5  Hue        (base colour)
--   K6  Spread     (one solid colour -> full rainbow across the screen)
--   S7  Pattern    (Random / Rings)
--   S8  Finish     (Gloss / Chrome)
--   S9  Morph      (Static / slowly reshuffling network)
--   S10 Light      (Fixed / Spinning highlight)
--   S11 Image      (thickness from the input's luma / from a swell wave)
--   P12 Image      (bipolar: bright grows <- centre -> bright thins)
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

architecture pipedream of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (15 + 1 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant LATENCY : natural := 14;
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MAXCOL : natural := 128;                -- <= this many cells wide

    type t_rom256 is array(0 to 255) of std_logic_vector(15 downto 0);
    -- GEN-BEGIN (gen_tables.py)
    constant C_KROM : t_rom256 := (
        x"0000", x"4000", x"2000", x"1555", x"1000", x"0CCD", x"0AAB", x"0925",
        x"0800", x"071C", x"0666", x"05D1", x"0555", x"04EC", x"0492", x"0444",
        x"0400", x"03C4", x"038E", x"035E", x"0333", x"030C", x"02E9", x"02C8",
        x"02AB", x"028F", x"0276", x"025F", x"0249", x"0235", x"0222", x"0211",
        x"0200", x"01F0", x"01E2", x"01D4", x"01C7", x"01BB", x"01AF", x"01A4",
        x"019A", x"0190", x"0186", x"017D", x"0174", x"016C", x"0164", x"015D",
        x"0155", x"014E", x"0148", x"0141", x"013B", x"0135", x"012F", x"012A",
        x"0125", x"011F", x"011A", x"0116", x"0111", x"010D", x"0108", x"0104",
        x"0100", x"00FC", x"00F8", x"00F5", x"00F1", x"00ED", x"00EA", x"00E7",
        x"00E4", x"00E0", x"00DD", x"00DA", x"00D8", x"00D5", x"00D2", x"00CF",
        x"00CD", x"00CA", x"00C8", x"00C5", x"00C3", x"00C1", x"00BF", x"00BC",
        x"00BA", x"00B8", x"00B6", x"00B4", x"00B2", x"00B0", x"00AE", x"00AC",
        x"00AB", x"00A9", x"00A7", x"00A5", x"00A4", x"00A2", x"00A1", x"009F",
        x"009E", x"009C", x"009B", x"0099", x"0098", x"0096", x"0095", x"0094",
        x"0092", x"0091", x"0090", x"008E", x"008D", x"008C", x"008B", x"008A",
        x"0089", x"0087", x"0086", x"0085", x"0084", x"0083", x"0082", x"0081",
        x"0080", x"007F", x"007E", x"007D", x"007C", x"007B", x"007A", x"0079",
        x"0078", x"0078", x"0077", x"0076", x"0075", x"0074", x"0073", x"0073",
        x"0072", x"0071", x"0070", x"006F", x"006F", x"006E", x"006D", x"006D",
        x"006C", x"006B", x"006A", x"006A", x"0069", x"0068", x"0068", x"0067",
        x"0066", x"0066", x"0065", x"0065", x"0064", x"0063", x"0063", x"0062",
        x"0062", x"0061", x"0060", x"0060", x"005F", x"005F", x"005E", x"005E",
        x"005D", x"005D", x"005C", x"005C", x"005B", x"005B", x"005A", x"005A",
        x"0059", x"0059", x"0058", x"0058", x"0057", x"0057", x"0056", x"0056",
        x"0055", x"0055", x"0054", x"0054", x"0054", x"0053", x"0053", x"0052",
        x"0052", x"0052", x"0051", x"0051", x"0050", x"0050", x"0050", x"004F",
        x"004F", x"004E", x"004E", x"004E", x"004D", x"004D", x"004D", x"004C",
        x"004C", x"004C", x"004B", x"004B", x"004A", x"004A", x"004A", x"0049",
        x"0049", x"0049", x"0048", x"0048", x"0048", x"0048", x"0047", x"0047",
        x"0047", x"0046", x"0046", x"0046", x"0045", x"0045", x"0045", x"0045",
        x"0044", x"0044", x"0044", x"0043", x"0043", x"0043", x"0043", x"0042",
        x"0042", x"0042", x"0042", x"0041", x"0041", x"0041", x"0041", x"0040");
    constant C_SHADE : t_rom256 := (
        x"C814", x"C816", x"C81C", x"C825", x"C832", x"C841", x"C853", x"C866",
        x"C47B", x"BA90", x"B2A4", x"A9B8", x"A1CA", x"9ADA", x"94E7", x"8FF1",
        x"8CF8", x"8AFD", x"88FF", x"88FF", x"87FF", x"86FF", x"86FF", x"85FF",
        x"84FF", x"84FF", x"83FF", x"82FF", x"81FF", x"80FF", x"80FF", x"7FFF",
        x"7EFF", x"7DFF", x"7CFF", x"7BFF", x"7AFF", x"79FF", x"78FF", x"77FF",
        x"75FF", x"74FF", x"73FF", x"72FF", x"70FF", x"6FFF", x"6EFF", x"6CFF",
        x"6BFF", x"6AFF", x"68FF", x"67FF", x"65FF", x"64FF", x"62FF", x"60FF",
        x"5FFF", x"5DFF", x"5BFF", x"5AFF", x"58FF", x"56FF", x"54FF", x"53FF",
        x"51FF", x"4FFF", x"4DFF", x"4BFF", x"49FF", x"47FF", x"45FF", x"43FF",
        x"41FF", x"3FFF", x"3DFF", x"3AFF", x"38FF", x"36FF", x"34FF", x"31FF",
        x"2FFF", x"2DFF", x"2AFF", x"28FF", x"25FF", x"23FF", x"20FF", x"1FFF",
        x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF",
        x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF",
        x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF",
        x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF",
        x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF", x"1FFF",
        x"C800", x"C800", x"C800", x"C800", x"C800", x"C800", x"C805", x"C815",
        x"BF2A", x"B23C", x"AA46", x"A746", x"A446", x"A146", x"9D46", x"9946",
        x"9546", x"9146", x"8C46", x"8846", x"8446", x"8046", x"7C46", x"7946",
        x"7546", x"7246", x"7046", x"6E46", x"6D46", x"6246", x"4446", x"2646",
        x"1746", x"1746", x"1264", x"1264", x"1264", x"1764", x"2064", x"2864",
        x"2B64", x"2C64", x"2D64", x"2F64", x"3164", x"3364", x"3664", x"3864",
        x"3B64", x"3E64", x"4264", x"4564", x"4864", x"4C64", x"4F64", x"5364",
        x"5664", x"5964", x"5C64", x"5F64", x"6264", x"6464", x"6664", x"6864",
        x"6A64", x"6B64", x"6C64", x"6C64", x"6C64", x"6B64", x"6A64", x"6964",
        x"6764", x"6564", x"6264", x"6064", x"5D64", x"5A64", x"5764", x"5464",
        x"5164", x"4F64", x"4C64", x"4964", x"4764", x"4564", x"4364", x"4264",
        x"4164", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064",
        x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064",
        x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064",
        x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064",
        x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064", x"4064");
    constant C_SATOFF : t_rom256 := (
        x"0000", x"0000", x"0001", x"0001", x"0002", x"0002", x"0003", x"0003",
        x"0004", x"0004", x"0004", x"0005", x"0005", x"0006", x"0006", x"0007",
        x"0007", x"0007", x"0008", x"0008", x"0008", x"0009", x"0009", x"000A",
        x"000A", x"000A", x"000B", x"000B", x"000B", x"000C", x"000C", x"000C",
        x"000C", x"000D", x"000D", x"000D", x"000E", x"000E", x"000E", x"000E",
        x"000F", x"000F", x"000F", x"000F", x"000F", x"0010", x"0010", x"0010",
        x"0010", x"0010", x"0011", x"0011", x"0011", x"0011", x"0011", x"0012",
        x"0012", x"0012", x"0012", x"0012", x"0012", x"0012", x"0013", x"0013",
        x"0013", x"0013", x"0013", x"0013", x"0013", x"0013", x"0013", x"0014",
        x"0014", x"0014", x"0014", x"0014", x"0014", x"0014", x"0014", x"0014",
        x"0014", x"0015", x"0015", x"0015", x"0015", x"0015", x"0015", x"0015",
        x"0015", x"0015", x"0015", x"0015", x"0015", x"0015", x"0015", x"0015",
        x"0016", x"0016", x"0016", x"0016", x"0016", x"0016", x"0016", x"0016",
        x"0016", x"0016", x"0016", x"0016", x"0016", x"0016", x"0016", x"0016",
        x"0016", x"0016", x"0016", x"0016", x"0016", x"0016", x"0016", x"0016",
        x"0017", x"0017", x"0017", x"0017", x"0017", x"0017", x"0017", x"0017",
        x"FFE9", x"FFE9", x"FFE9", x"FFE9", x"FFE9", x"FFE9", x"FFE9", x"FFE9",
        x"FFE9", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA",
        x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA",
        x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA", x"FFEA",
        x"FFEA", x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB",
        x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB", x"FFEB",
        x"FFEC", x"FFEC", x"FFEC", x"FFEC", x"FFEC", x"FFEC", x"FFEC", x"FFEC",
        x"FFEC", x"FFEC", x"FFED", x"FFED", x"FFED", x"FFED", x"FFED", x"FFED",
        x"FFED", x"FFED", x"FFED", x"FFEE", x"FFEE", x"FFEE", x"FFEE", x"FFEE",
        x"FFEE", x"FFEE", x"FFEF", x"FFEF", x"FFEF", x"FFEF", x"FFEF", x"FFF0",
        x"FFF0", x"FFF0", x"FFF0", x"FFF0", x"FFF1", x"FFF1", x"FFF1", x"FFF1",
        x"FFF1", x"FFF2", x"FFF2", x"FFF2", x"FFF2", x"FFF3", x"FFF3", x"FFF3",
        x"FFF4", x"FFF4", x"FFF4", x"FFF4", x"FFF5", x"FFF5", x"FFF5", x"FFF6",
        x"FFF6", x"FFF6", x"FFF7", x"FFF7", x"FFF8", x"FFF8", x"FFF8", x"FFF9",
        x"FFF9", x"FFF9", x"FFFA", x"FFFA", x"FFFB", x"FFFB", x"FFFC", x"FFFC",
        x"FFFC", x"FFFD", x"FFFD", x"FFFE", x"FFFE", x"FFFF", x"FFFF", x"0000");
    -- GEN-END

    function f_absu(v : signed) return unsigned is
    begin
        if v(v'high) = '1' then return unsigned(-v); else return unsigned(v); end if;
    end function;

    function f_b(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

    -- quarter-wave sine ROM (fireworks): sin/cos over 0..2pi, 0 EBR.
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

    -- the same quarter-wave pre-scaled by the vivid chroma saturation
    -- (430/512), so the pixel path needs no saturation multiply
    constant C_QSAT : t_qsin := (
        to_signed(  0, 10), to_signed( 11, 10), to_signed( 21, 10), to_signed( 32, 10), to_signed( 42, 10), to_signed( 53, 10), to_signed( 63, 10), to_signed( 73, 10),
        to_signed( 84, 10), to_signed( 94, 10), to_signed(104, 10), to_signed(114, 10), to_signed(124, 10), to_signed(134, 10), to_signed(144, 10), to_signed(155, 10),
        to_signed(165, 10), to_signed(174, 10), to_signed(183, 10), to_signed(193, 10), to_signed(202, 10), to_signed(212, 10), to_signed(221, 10), to_signed(229, 10),
        to_signed(239, 10), to_signed(247, 10), to_signed(255, 10), to_signed(264, 10), to_signed(272, 10), to_signed(281, 10), to_signed(288, 10), to_signed(296, 10),
        to_signed(303, 10), to_signed(311, 10), to_signed(318, 10), to_signed(325, 10), to_signed(332, 10), to_signed(338, 10), to_signed(344, 10), to_signed(351, 10),
        to_signed(357, 10), to_signed(363, 10), to_signed(368, 10), to_signed(374, 10), to_signed(379, 10), to_signed(383, 10), to_signed(388, 10), to_signed(392, 10),
        to_signed(396, 10), to_signed(401, 10), to_signed(404, 10), to_signed(407, 10), to_signed(411, 10), to_signed(413, 10), to_signed(417, 10), to_signed(419, 10),
        to_signed(421, 10), to_signed(422, 10), to_signed(424, 10), to_signed(426, 10), to_signed(427, 10), to_signed(428, 10), to_signed(428, 10), to_signed(429, 10));

    function f_qsat(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then v_i := a(7 downto 2); else v_i := not a(7 downto 2); end if;
        v_t := C_QSAT(to_integer(v_i));
        if a(9) = '1' then return -v_t; else return v_t; end if;
    end function;

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then v_i := a(7 downto 2); else v_i := not a(7 downto 2); end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then return -v_t; else return v_t; end if;
    end function;

    -- one add / xorshift hash round (the carries are the nonlinearity)
    function f_round(h : unsigned(15 downto 0); s1, s2 : natural) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := h + shift_left(h, s1);
        return v xor shift_right(v, s2);
    end function;

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

    ----------------------------------------------------------------------------
    -- Raster position + animation phases.
    ----------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_hedge      : std_logic := '0';
    signal r_anchor     : std_logic := '0';

    signal x_loc     : unsigned(7 downto 0) := (others => '0');
    signal x_idx     : unsigned(6 downto 0) := (others => '0');
    signal celly_loc : unsigned(7 downto 0) := (others => '0');
    signal celly_idx : unsigned(6 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';
    signal x_f  : unsigned(11 downto 0) := (others => '0');
    signal y_ln : unsigned(10 downto 0) := (others => '0');
    signal y_f  : unsigned(11 downto 0) := (others => '0');

    -- Resolution scaling (virtual 1080-line units).  s_xs: x step in half
    -- units (2/3/4); x_fr: the half-unit phase, so a pixel's integer step is
    -- 1 or 2.  y: Q.8 fraction accumulator per field line; the integer part
    -- of each line's advance becomes 1..5 unit steps (y_pend), one every other
    -- clock after the hsync, each the old one-line update.
    signal s_xs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal x_fr    : std_logic := '0';
    signal s_d2    : std_logic := '0';     -- this pixel's step to the next is 2
    signal s_xlast : std_logic := '0';     -- this pixel is the last of its cell
    -- 2wh - x_loc, stepped alongside x_loc so the qxE update is one carry chain
    signal x_e     : unsigned(8 downto 0) := to_unsigned(96, 9);
    signal s_twm1  : unsigned(8 downto 0) := to_unsigned(95, 9);      -- 2wh - 1
    signal y_fr    : unsigned(7 downto 0) := (others => '0');
    signal y_pend  : unsigned(3 downto 0) := (others => '0');
    signal y_pph   : std_logic := '0';
    signal r_ustep : std_logic := '0';     -- a unit y step happened last clock
    signal s_yfirst : std_logic := '1';    -- this line is the first of its cell row

    signal s_phase   : unsigned(15 downto 0) := (others => '0');  -- flow
    signal s_huep    : unsigned(15 downto 0) := (others => '0');  -- hue drift
    signal s_morphp  : unsigned(7 downto 0)  := (others => '0');  -- network reshuffle
    signal s_lightp  : unsigned(9 downto 0)  := (others => '0');  -- spinning light

    -- per-cell tile, hashed a few clocks ahead on the NEXT cell index and
    -- latched at each cell boundary (the hash changes only once per cell)
    signal hs_a, hs_b : unsigned(15 downto 0) := (others => '0');
    signal hs_1, hs_2, hs_3, hs_4, hs_5 : unsigned(15 downto 0) := (others => '0');
    signal hs_xc, hs_or : std_logic := '0';
    signal s_cxc, s_cor : std_logic := '0';    -- current cell: crossing / elbow pairing

    -- Ring breaker (Random pattern): the tiles of the cell row ABOVE live in a
    -- 256-entry EBR (bit1 = crossing, bit0 = pairing), rewritten with the
    -- current row's FINAL tiles on the first line of each cell row and read
    -- back on its other lines, so every line of a row sees the same tiles.
    type t_rt is array (0 to 255) of std_logic_vector(7 downto 0);
    signal rt_mem  : t_rt;
    signal rt_ra   : unsigned(7 downto 0) := (others => '0');
    signal rt_rd, rt_rd2 : std_logic_vector(7 downto 0) := (others => '0');
    signal rt_we1, rt_we2 : std_logic := '0';
    signal rt_wa1, rt_wa2 : unsigned(7 downto 0) := (others => '0');
    signal rt_wd1, rt_wd2 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_pab   : std_logic_vector(1 downto 0) := (others => '0');   -- above-left's final tile
    signal s_avid_d : std_logic := '0';

    signal s_yprev : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Per-frame parameters (latched at vsync, refined by the frame FSM).
    ----------------------------------------------------------------------------
    signal lk1, lk2, lk3, lk4, lk5, lk6, lp12 : unsigned(9 downto 0) := (others => '0');

    signal s_w     : unsigned(7 downto 0) := to_unsigned(96, 8);    -- always EVEN (2*wh)
    signal s_wm1   : unsigned(7 downto 0) := to_unsigned(95, 8);
    signal s_wm2   : unsigned(7 downto 0) := to_unsigned(94, 8);
    signal s_wh    : unsigned(7 downto 0) := to_unsigned(48, 8);
    signal s_wh3   : unsigned(9 downto 0) := to_unsigned(144, 10);   -- 3*wh
    signal s_wh4   : signed(10 downto 0) := to_signed(192, 11);      -- 4*wh
    -- wh fans out to every pixel stage: one keep'd copy per consumer so the
    -- placer can park each beside its sink
    signal s_whp, s_whq, s_whr, s_whl : unsigned(7 downto 0) := to_unsigned(48, 8);
    attribute keep : boolean;
    attribute keep of s_whp, s_whq, s_whr, s_whl : signal is true;
    signal s_rp    : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_rpm1  : unsigned(7 downto 0) := to_unsigned(95, 8);
    signal s_rfrac : unsigned(10 downto 0) := to_unsigned(300, 11);
    signal s_r     : unsigned(7 downto 0) := to_unsigned(14, 8);     -- Fatness radius
    signal s_rmax  : unsigned(7 downto 0) := to_unsigned(19, 8);     -- largest clean radius (Fatness top)
    signal s_rimg  : unsigned(7 downto 0) := to_unsigned(30, 8);     -- Image swell top (pipes merge)
    signal s_pv, s_ph : signed(7 downto 0) := (others => '0');       -- straights' light projection
    signal s_lx, s_ly : signed(9 downto 0) := (others => '0');
    signal s_lx6, s_ly6 : signed(6 downto 0) := (others => '0');
    signal s_lxw, s_lyw : signed(7 downto 0) := (others => '0');  -- light dir scaled by 1/wh

    signal s_xthresh : unsigned(8 downto 0) := to_unsigned(100, 9); -- crossing density
    signal s_seed   : unsigned(7 downto 0) := to_unsigned(90, 8);
    signal s_hue    : unsigned(9 downto 0) := (others => '0');
    signal s_hshift : unsigned(2 downto 0) := to_unsigned(3, 3);    -- rainbow scale

    -- Image (P12, bipolar): radius = c0 + c1 * L'/64 (quarter px), where
    -- L' = luma (right half: bright grows) or 63-luma (left half: bright thins)
    signal s_a6    : unsigned(5 downto 0) := (others => '0');       -- |P12 - centre|
    signal s_pos   : std_logic := '1';
    signal s_c0q   : unsigned(8 downto 0) := to_unsigned(56, 9);
    signal s_c1q   : unsigned(8 downto 0) := (others => '0');
    signal s_swp   : unsigned(7 downto 0) := (others => '0');       -- swell-wave phase (Image off)

    -- elbow geometry: quarter-torus of radius wh centred on the cell corner
    signal s_wh2   : unsigned(15 downto 0) := to_unsigned(2304, 16);   -- wh^2
    signal s_krec  : unsigned(6 downto 0)  := to_unsigned(43, 7);      -- round(4096/(2wh))
    signal s_twh   : unsigned(8 downto 0)  := to_unsigned(96, 9);      -- 2*wh
    signal s_w2    : unsigned(16 downto 0) := to_unsigned(9216, 17);   -- (2wh)^2  (qxE/qyS reset)
    signal s_w2o   : unsigned(16 downto 0) := to_unsigned(9025, 17);   -- (2wh-1)^2 (x wrap onto loc 1)
    signal s_sqs   : unsigned(1 downto 0)  := to_unsigned(1, 2);       -- sqrt-table rho^2 shift

    signal s_flowspd : unsigned(9 downto 0) := (others => '0');
    signal s_floww   : unsigned(6 downto 0) := to_unsigned(24, 7);
    signal s_flowg   : unsigned(2 downto 0) := (others => '0');
    signal s_flowon  : std_logic := '0';
    signal s_huespd  : unsigned(9 downto 0) := to_unsigned(6, 10);

    signal s_rings  : std_logic := '0';                -- S7
    signal s_chrome : std_logic := '0';                -- S8
    signal s_morph  : std_logic := '0';                -- S9
    signal s_spin   : std_logic := '0';                -- S10
    signal s_image  : std_logic := '1';                -- S11

    signal fsm_t   : unsigned(7 downto 0) := (others => '1');
    signal ma, mb  : signed(12 downto 0) := (others => '0');
    signal mpa, mp : signed(25 downto 0) := (others => '0');
    signal div_run : std_logic := '0';
    signal div_cnt : unsigned(4 downto 0) := (others => '0');
    signal div_d   : unsigned(17 downto 0) := (others => '0');
    signal div_q   : unsigned(17 downto 0) := (others => '0');
    signal div_rem : unsigned(8 downto 0)  := (others => '0');
    signal div_v   : unsigned(7 downto 0)  := (others => '1');

    -- Exact elbow cross-section: a 2048-entry table of round(4*rho) - 4wh
    -- against rho^2 >> sqs, rebuilt in vblank by an add-only integer sqrt
    -- walk (~2.6k clocks), read once per pixel.
    type t_sq is array (0 to 2047) of std_logic_vector(15 downto 0);
    signal sqtab   : t_sq;
    signal sb_go, sb_run : std_logic := '0';
    signal sb_k    : unsigned(10 downto 0) := (others => '0');
    signal sb_r4   : unsigned(9 downto 0)  := (others => '0');
    signal sb_msq, sb_inc, sb_tgt, sb_step : unsigned(21 downto 0) := (others => '0');
    signal bw_we   : std_logic := '0';
    signal bw_addr : unsigned(10 downto 0) := (others => '0');
    signal bw_data : std_logic_vector(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Per-line dy and the corner-distance squares, incremental.
    ----------------------------------------------------------------------------
    signal lm_dy   : signed(9 downto 0) := (others => '0');
    signal ln_n, ln_s : signed(10 downto 0) := (others => '0');   -- next line's (dy+wh), (dy-wh), precomputed
    -- qxW/qxE = (dx+wh)^2 / (dx-wh)^2 per pixel; qyN/qyS = (dy+wh)^2 /
    -- (dy-wh)^2 per line.  rho^2 to an elbow's corner = one of each, added.
    signal qxW, qxE : unsigned(15 downto 0) := (others => '0');
    signal qyN, qyS : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Pixel pipeline.
    ----------------------------------------------------------------------------
    signal g0_dx, g0_dy : signed(9 downto 0) := (others => '0');
    signal g0_qx, g0_qy : unsigned(15 downto 0) := (others => '0');
    signal g0_cN, g0_cE : std_logic := '0';
    signal g0_xc, g0_par : std_logic := '0';
    signal g0_l    : unsigned(5 downto 0) := (others => '0');

    signal p2_rho2 : unsigned(16 downto 0) := (others => '0');
    signal p2_excx, p2_eycy : signed(9 downto 0) := (others => '0');
    signal p2_aex, p2_d1 : signed(10 downto 0) := (others => '0');  -- |excx|, |eycy| + wh
    signal p2_lp   : unsigned(5 downto 0) := (others => '0');       -- L' (Image drive)
    signal p2_dxc, p2_dyc : signed(6 downto 0) := (others => '0');
    signal p2_adx, p2_ady : unsigned(6 downto 0) := (others => '0');
    signal p2_xc, p2_par : std_logic := '0';

    signal p3_sqa  : unsigned(10 downto 0) := (others => '0');      -- sqrt-table address
    signal p3_mx, p3_my : signed(17 downto 0) := (others => '0');
    signal p3_rpm  : unsigned(14 downto 0) := (others => '0');
    signal p3_alarc : signed(10 downto 0) := (others => '0');
    signal p3_dxc, p3_dyc : signed(6 downto 0) := (others => '0');
    signal p3_adx, p3_ady : unsigned(6 downto 0) := (others => '0');
    signal p3_xc, p3_par : std_logic := '0';

    signal p4_sq   : std_logic_vector(15 downto 0) := (others => '0');  -- sqrt table (EBR) out
    signal p4_do   : signed(7 downto 0) := (others => '0');
    signal p4_rp4  : unsigned(7 downto 0) := (others => '0');     -- tube radius, 1/4 px
    signal p4_alarc : signed(10 downto 0) := (others => '0');
    signal p4_dxc, p4_dyc : signed(6 downto 0) := (others => '0');
    signal p4_adx, p4_ady : unsigned(6 downto 0) := (others => '0');
    signal p4_xc, p4_par : std_logic := '0';

    signal p5_e4   : signed(9 downto 0) := (others => '0');       -- elbow cross-section, 1/4 px
    signal p5_k    : unsigned(14 downto 0) := (others => '0');    -- K ROM (EBR) out
    signal p5_saddr : unsigned(7 downto 0) := (others => '0');
    signal p5_dcr  : signed(6 downto 0) := (others => '0');
    signal p5_rp4  : unsigned(7 downto 0) := (others => '0');
    signal p5_alarc : signed(10 downto 0) := (others => '0');
    signal p5_adx, p5_ady : unsigned(6 downto 0) := (others => '0');
    signal p5_strH, p5_shad, p5_xc, p5_par : std_logic := '0';

    signal p6_off  : std_logic_vector(15 downto 0) := (others => '0');  -- offset ROM (EBR) out
    signal p6_k    : unsigned(14 downto 0) := (others => '0');
    signal p6_d4   : signed(8 downto 0) := (others => '0');
    signal p6_hit  : std_logic := '0';
    signal p6_al   : signed(10 downto 0) := (others => '0');
    signal p6_shad : std_logic := '0';

    signal p7_off  : signed(7 downto 0) := (others => '0');
    signal p7_ph   : signed(17 downto 0) := (others => '0');      -- d4 * K split hi/lo
    signal p7_pl   : signed(16 downto 0) := (others => '0');
    signal p7_aln  : unsigned(7 downto 0) := (others => '0');
    signal p7_shad : std_logic := '0';

    signal p8_dn   : signed(7 downto 0) := (others => '0');
    signal p8_off  : signed(7 downto 0) := (others => '0');
    signal p8_aln  : unsigned(7 downto 0) := (others => '0');
    signal p8_shad : std_logic := '0';

    signal p9_addr : unsigned(7 downto 0) := (others => '0');
    signal p9_rim  : unsigned(1 downto 0) := (others => '0');
    signal p9_fph  : unsigned(6 downto 0) := (others => '0');
    signal p9_shad : std_logic := '0';

    signal p10_sh   : std_logic_vector(15 downto 0) := (others => '0');  -- shade ROM (EBR) out
    signal p10_rim  : unsigned(1 downto 0) := (others => '0');
    signal p10_flow : unsigned(9 downto 0) := (others => '0');
    signal p10_shad : std_logic := '0';

    signal p11_y8, p11_cf : unsigned(7 downto 0) := (others => '0');
    signal p11_rim  : unsigned(1 downto 0) := (others => '0');
    signal p11_flow : unsigned(9 downto 0) := (others => '0');
    signal p11_shad : std_logic := '0';
    signal p11_ang  : unsigned(9 downto 0) := (others => '0');

    signal p12_y    : unsigned(9 downto 0) := (others => '0');
    signal p12_cf   : unsigned(5 downto 0) := (others => '0');
    signal cosR, sinR : signed(9 downto 0) := (others => '0');
    signal p13_um, p13_vm : signed(16 downto 0) := (others => '0');
    signal p13_y    : unsigned(9 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := C_MID;

    signal hitA : std_logic_vector(0 to 6) := (others => '0');

    -- syncs ride a 4-bit pipe (hsync, vsync, avid, field)
    type t_sync is array (natural range <>) of std_logic_vector(3 downto 0);
    signal syncp : t_sync(0 to LATENCY - 1) := (others => (others => '1'));
    signal s_io : t_video_stream_yuv444_30b;

begin

    ----------------------------------------------------------------------------
    -- Raster counters, interlace, frame coords, animation phases.
    ----------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_yn  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;
            r_hedge  <= v_h_edge;
            r_anchor <= '0';
            r_ustep  <= '0';

            -- Resolution scaling: per-mode steps, free-running from the timing
            -- ID (static while a mode runs).
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if v_v_edge = '1' then
                s_phase  <= s_phase + s_flowspd;
                s_huep   <= s_huep + s_huespd;
                if s_morph = '1' then s_morphp <= s_morphp + 1; end if;
                if s_spin  = '1' then s_lightp <= s_lightp + 3; end if;
                s_swp    <= s_swp + 1;
            end if;

            -- x: each pixel advances the cell counters by its integer step,
            -- 1 or 2 (s_d2); the corner squares follow exactly.  A cell wraps
            -- onto loc 0 or 1.
            if v_h_edge = '1' then
                x_loc <= (others => '0'); x_idx <= (others => '0'); x_f <= (others => '0');
                x_fr  <= '0';
                x_e   <= s_twh;
                s_xlast <= '0';
                qxW <= (others => '0'); qxE <= s_w2(15 downto 0);   -- x_loc^2 / (2wh-x_loc)^2 at x_loc=0
            elsif data_in.avid = '1' then
                -- s_xlast is registered one pixel ahead: a pixel and the next
                -- together always advance exactly s_xs units (Q.1 steps), so
                -- the NEXT pixel is the cell's last iff 2wh - x_loc <= s_xs.
                -- After a wrap the next pixel is near the cell start (never last).
                if s_xlast = '0' and x_e <= resize(s_xs, 9) then s_xlast <= '1';
                else                                              s_xlast <= '0'; end if;
                x_fr <= s_xs(0) xor x_fr;
                if s_d2 = '1' then x_f <= x_f + 2; else x_f <= x_f + 1; end if;
                if s_xlast = '1' then
                    if x_idx /= C_MAXCOL - 1 then x_idx <= x_idx + 1; end if;
                    if s_d2 = '1' and x_e(0) = '1' then              -- x_loc = 2wh-1: lands on loc 1
                        x_loc <= to_unsigned(1, 8);
                        x_e   <= s_twm1;
                        qxW <= to_unsigned(1, 16); qxE <= s_w2o(15 downto 0);
                    else
                        x_loc <= (others => '0');
                        x_e   <= s_twh;
                        qxW <= (others => '0'); qxE <= s_w2(15 downto 0);
                    end if;
                elsif s_d2 = '1' then
                    x_loc <= x_loc + 2;
                    x_e   <= x_e - 2;
                    qxW <= qxW + (resize(x_loc, 16) sll 2) + 4;                       -- (x_loc+2)^2
                    qxE <= qxE - (resize(x_e, 16) sll 2) + 4;                         -- (2wh-x_loc-2)^2
                else
                    x_loc <= x_loc + 1;
                    x_e   <= x_e - 1;
                    qxW <= qxW + (resize(x_loc, 16) sll 1) + 1;                       -- (x_loc+1)^2
                    qxE <= qxE - (resize(x_e, 16) sll 1) + 1;                         -- (2wh-x_loc-1)^2
                end if;
            end if;

            -- y: each line's advance (integer part of the Q.8 step) becomes
            -- unit steps, one every other clock (p_line needs a clock between
            -- updates), all done long before the line's active video.
            if v_v_edge = '1' then
                celly_loc <= (others => '0'); celly_idx <= (others => '0');
                frame_act <= '0'; y_ln <= (others => '0');
                y_fr <= (others => '0'); y_pend <= (others => '0'); s_yfirst <= '1';
            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1'; r_anchor <= '1';
                celly_loc <= (others => '0'); celly_idx <= (others => '0'); y_ln <= (others => '0');
                y_fr <= (others => '0'); y_pend <= (others => '0'); s_yfirst <= '1';
            elsif v_h_edge = '1' and frame_act = '1' then
                v_yn := resize(y_fr, 12) + s_ys;
                y_fr   <= v_yn(7 downto 0);
                y_pend <= v_yn(11 downto 8);
                y_pph  <= '0';
                s_yfirst <= '0';
            elsif y_pend /= 0 then
                y_pph <= not y_pph;
                if y_pph = '0' then
                    y_pend  <= y_pend - 1;
                    r_ustep <= '1';
                    y_ln <= y_ln + 1;
                    if celly_loc = s_rpm1 then
                        celly_loc <= (others => '0');
                        s_yfirst  <= '1';
                        if celly_idx /= 127 then celly_idx <= celly_idx + 1; end if;
                    else
                        celly_loc <= celly_loc + 1;
                    end if;
                end if;
            end if;

            y_f <= '0' & y_ln;
        end if;
    end process p_position;

    -- this pixel's integer step to the next is 2 (SD always, 720p every other)
    s_d2    <= '1' when s_xs(2) = '1' or (s_xs(0) = '1' and x_fr = '1') else '0';

    ----------------------------------------------------------------------------
    -- Cell hash pipe: hashes the cell that comes NEXT (x_idx+1 during active
    -- video, x_idx = 0 in blanking); 7 clocks deep, cells are >= 32 px wide.
    ----------------------------------------------------------------------------
    p_hash : process(clk)
        variable v_a : unsigned(6 downto 0);
        variable v_m : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then v_a := x_idx + 1; else v_a := x_idx; end if;
            hs_a <= (resize(v_a, 16) sll 5) + (resize(v_a, 16) sll 2) + resize(v_a, 16);          -- a*37
            hs_b <= (resize(celly_idx, 16) sll 7) + (resize(celly_idx, 16) sll 4)
                  + (resize(celly_idx, 16) sll 1) + resize(s_seed, 16);                            -- b*146 + seed
            hs_1 <= hs_a + hs_b;
            hs_2 <= f_round(hs_1, 5, 3);
            hs_3 <= f_round(hs_2, 4, 10);
            hs_4 <= f_round(hs_3, 9, 7);
            hs_5 <= f_round(hs_4, 8, 4);
            hs_xc <= f_b(('0' & (hs_5(7 downto 0) + s_morphp)) < s_xthresh);
            v_m := hs_5(15 downto 8) + s_morphp;
            hs_or <= v_m(7);
        end if;
    end process p_hash;

    ----------------------------------------------------------------------------
    -- Tile latch + ring breaker.  A lone ring closes round a grid vertex
    -- exactly when the four cells around it are elbows with the vertex-facing
    -- pairing: above-left NW/SE, above NE/SW, left NE/SW, this cell NW/SE.
    -- Each cell checks its own top-left vertex against the FINAL tiles of its
    -- left, above and above-left neighbours and, if it would close a ring,
    -- takes the NE/SW pairing instead.  Scanning left-to-right, top-to-bottom,
    -- every vertex is checked once with its other three cells already final,
    -- so no lone ring survives anywhere.  Random pattern only (Rings wants
    -- them).  Target t = the cell the hash pipe is working on (x_idx+1 in
    -- active video, 0 in blanking); its row-above tile is read 3 clocks
    -- after t settles, the hash 7 clocks after, the latch >= 32 clocks after.
    ----------------------------------------------------------------------------
    p_tiles : process(clk)
        variable v_first, v_ring : boolean;
        variable v_fin : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_d <= data_in.avid;
            if data_in.avid = '1' then rt_ra <= resize(x_idx, 8) + 1; else rt_ra <= (others => '0'); end if;
            rt_rd  <= rt_mem(to_integer(rt_ra));
            rt_rd2 <= rt_rd;

            v_first := s_yfirst = '1';                                 -- first line of a cell row
            v_ring  := s_rings = '0' and celly_idx /= 0 and data_in.avid = '1'
                       and hs_xc = '0' and hs_or = '1'                  -- this cell: NW/SE elbows
                       and s_cxc = '0' and s_cor = '0'                  -- left:  NE/SW elbows
                       and rt_rd2(1 downto 0) = "00"                    -- above: NE/SW elbows
                       and s_pab = "01";                                -- above-left: NW/SE elbows
            if not v_first then
                v_fin := rt_rd2(1 downto 0);                           -- this row, stored on its first line
            elsif v_ring then
                v_fin := "00";
            else
                v_fin := hs_xc & hs_or;
            end if;

            rt_we1 <= '0';
            if data_in.avid = '0' or s_xlast = '1' then
                s_cxc <= v_fin(1); s_cor <= v_fin(0);
                if v_first then
                    s_pab <= rt_rd2(1 downto 0);
                    if data_in.avid = '1' then                           -- store cell t's final tile
                        rt_we1 <= '1'; rt_wa1 <= resize(x_idx, 8) + 1; rt_wd1 <= "000000" & v_fin;
                    end if;
                end if;
            end if;
            if v_first and data_in.avid = '1' and s_avid_d = '0' then     -- cell 0, at the line's first pixel
                rt_we1 <= '1'; rt_wa1 <= (others => '0'); rt_wd1 <= "000000" & s_cxc & s_cor;
            end if;
            -- writes land two clocks late, after the read address has moved on
            -- (EBR read-during-write at one address is undefined)
            rt_we2 <= rt_we1; rt_wa2 <= rt_wa1; rt_wd2 <= rt_wd1;
            if rt_we2 = '1' then rt_mem(to_integer(rt_wa2)) <= rt_wd2; end if;
        end if;
    end process p_tiles;

    ----------------------------------------------------------------------------
    -- Per-line dy and the elbow corner y-squares (adds only).
    ----------------------------------------------------------------------------
    p_line : process(clk)
        function upd1(sq : unsigned(15 downto 0); dn : signed(10 downto 0)) return unsigned is
            variable v : signed(18 downto 0);
        begin v := signed(resize(sq, 19)) + resize(dn & '0', 19) - 1; return unsigned(v(15 downto 0)); end function;
        variable v_hh  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- one update per unit y step (r_ustep, >= 2 clocks apart) or at the
            -- frame anchor
            if r_ustep = '1' or r_anchor = '1' then
                v_hh := signed(resize(s_whl, 10));
                if celly_loc = 0 then
                    lm_dy <= -v_hh;
                    qyN <= (others => '0'); qyS <= s_w2(15 downto 0);        -- (0)^2 / (2wh)^2
                else
                    lm_dy <= lm_dy + 1;
                    qyN <= upd1(qyN, ln_n); qyS <= upd1(qyS, ln_s);
                end if;
            end if;
            -- lm_dy moves at most every other clock, so the next unit step's
            -- corner offsets are precomputed here, off the update path
            ln_n <= resize(lm_dy + 1, 11) + signed(resize(s_whl, 11));
            ln_s <= resize(lm_dy + 1, 11) - signed(resize(s_whl, 11));
        end if;
    end process p_line;

    ----------------------------------------------------------------------------
    -- Frame FSM: knob shaping, radius, Image coefficients, light vector, the
    -- 1/(2wh) divide; kicks the sqrt-table builder.
    ----------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_wh  : integer range 0 to 255;
        variable v_acc : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            mpa <= ma * mb;                      -- product valid 3 clocks after ma/mb
            mp  <= mpa;
            sb_go <= '0';
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
                if div_cnt = 17 then div_run <= '0'; else div_cnt <= div_cnt + 1; end if;
            end if;

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                lk1  <= unsigned(registers_in(0)(9 downto 0));
                lk2  <= unsigned(registers_in(1)(9 downto 0));
                lk3  <= unsigned(registers_in(2)(9 downto 0));
                lk4  <= unsigned(registers_in(3)(9 downto 0));
                lk5  <= unsigned(registers_in(4)(9 downto 0));
                lk6  <= unsigned(registers_in(5)(9 downto 0));
                lp12 <= unsigned(registers_in(7)(9 downto 0));
                s_rings   <= registers_in(6)(0);
                s_chrome  <= registers_in(6)(1);
                s_morph   <= registers_in(6)(2);
                s_spin    <= registers_in(6)(3);
                s_image   <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"FF" then
                fsm_t <= fsm_t + 1;
                case to_integer(fsm_t) is
                    when 0 =>
                        -- cells are always an EVEN width, so every grid vertex sits
                        -- on a pixel centre and neighbouring cells share it exactly
                        v_wh := 16 + to_integer(lk1(9 downto 4));   -- 16..79
                        s_wh     <= to_unsigned(v_wh, 8);
                        s_w      <= to_unsigned(2 * v_wh, 8);
                        -- r = w*rfrac/2048 = (0.12..0.40)*wh: elbow pairs kiss
                        -- at the cell centre at 0.41*wh, so the top stays clear
                        s_rfrac  <= resize(to_unsigned(120, 11) + lk3(9 downto 2) + lk3(9 downto 5), 11);
                        s_xthresh <= resize(lk2(9 downto 2), 9) + resize(lk2(9 downto 9), 9);  -- 0..256
                        s_hue    <= lk5;
                        s_hshift <= resize(7 - lk6(9 downto 7), 3);                      -- high Spread = more rainbow
                        if lp12(9) = '1' then s_pos <= '1'; s_a6 <= lp12(8 downto 3);
                        else                  s_pos <= '0'; s_a6 <= not lp12(8 downto 3); end if;
                        s_flowspd <= resize(lk4(9 downto 3), 10);
                        s_floww   <= resize(to_unsigned(8, 7) + lk4(9 downto 5), 7);
                        s_flowg   <= resize(lk4(9 downto 8), 3);
                        if lk4 > 32 then s_flowon <= '1'; else s_flowon <= '0'; end if;
                        if s_spin = '1' then s_lx <= f_qsin(s_lightp + 256); s_ly <= f_qsin(s_lightp);
                        else                 s_lx <= f_qsin(to_unsigned(672, 10)); s_ly <= f_qsin(to_unsigned(672, 10)); end if;  -- upper-left
                    when 1 =>
                        s_wm1 <= s_w - 1;
                        s_wm2 <= s_w - 2;
                        s_twh <= s_wh & '0';                 -- 2*wh
                        s_rp <= s_w;                         -- cell height (virtual units; the y step carries interlace)
                        s_whp <= s_wh; s_whq <= s_wh; s_whr <= s_wh; s_whl <= s_wh;
                        s_lx6 <= resize(shift_right(s_lx, 3), 7);
                        s_ly6 <= resize(shift_right(s_ly, 3), 7);
                        ma <= signed(resize(s_w, 13)); mb <= signed(resize(s_rfrac, 13));
                    when 2 =>
                        s_rpm1 <= s_rp - 1;
                        s_twm1 <= s_twh - 1;
                        s_wh3  <= resize(s_twh, 10) + resize(s_wh, 10);
                        s_wh4  <= signed(resize(s_twh & '0', 11));
                        -- s_krec = round(4096 / 2wh): normalises the light and the
                        -- flow phase (period 4wh) at every scale
                        div_d <= resize(to_unsigned(4096, 13) + resize(s_wh, 13), 18);
                        div_v <= s_twh(7 downto 0);
                        div_q <= (others => '0'); div_rem <= (others => '0'); div_cnt <= (others => '0'); div_run <= '1';
                    when 4 =>
                        if mp(18 downto 11) < 2 then s_r <= to_unsigned(2, 8); else s_r <= unsigned(mp(18 downto 11)); end if;
                        ma <= signed(resize(s_wh, 13)); mb <= signed(resize(s_wh, 13));               -- wh^2
                    when 7 =>
                        s_wh2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_w, 13)); mb <= to_signed(406, 13);                     -- max radius
                    when 8 =>
                        s_w2  <= resize(s_wh2 & "00", 17);                                            -- (2wh)^2
                        s_w2o <= resize(s_wh2 & "00", 17) - resize(s_wh & "00", 17) + 1;              -- (2wh-1)^2 (x wrap)
                        -- sqrt table covers rho^2 up to 2048 << sqs >= 2.55 wh^2: the
                        -- 0.6 wh band Image can swell to (worst error 0.20 px)
                        if    s_wh2 <= 803  then s_sqs <= "00";
                        elsif s_wh2 <= 1606 then s_sqs <= "01";
                        elsif s_wh2 <= 3212 then s_sqs <= "10";
                        else                     s_sqs <= "11"; end if;
                    when 10 =>
                        s_rmax <= unsigned(mp(18 downto 11));
                        ma <= signed(resize(s_w, 13)); mb <= to_signed(634, 13);                     -- Image swell top
                    when 11 =>
                        sb_go <= '1';
                    when 13 =>
                        -- rimg = 0.62 wh: Image swells bright (or dark) areas past
                        -- the 0.41 wh kiss, so neighbouring pipes merge -- the climax
                        s_rimg <= unsigned(mp(18 downto 11));
                        ma <= signed(resize(s_r & "00", 13)); mb <= signed(resize(s_a6, 13));
                    when 16 =>
                        -- c0 = 4r(1-a): the radius left where L' = 0
                        s_c0q <= resize(s_r & "00", 9) - unsigned(mp(14 downto 6));
                        ma <= signed(resize(s_rimg & "00", 13)); mb <= signed(resize(s_a6, 13));
                    when 19 =>
                        -- c1 = 4 rimg a: added in proportion to L'
                        s_c1q <= unsigned(mp(14 downto 6));
                    when 20 =>
                        -- never let a pipe vanish: squeeze the radius range to
                        -- r' = 7/8 r + rmax/8 (floor 1/8 of Fatness top, >= 1.25 px)
                        s_c0q <= s_c0q - shift_right(s_c0q, 3) + resize(shift_right(s_rmax & "00", 3), 9);
                        s_c1q <= s_c1q - shift_right(s_c1q, 3);
                    when 21 =>
                        if s_c0q < 5 then s_c0q <= to_unsigned(5, 9); end if;
                    when 24 =>
                        if div_q > 127 then s_krec <= (others => '1');
                        else s_krec <= div_q(6 downto 0); end if;
                    when 25 =>
                        ma <= resize(s_lx6, 13); mb <= signed(resize(s_krec, 13));
                    when 28 =>
                        s_lxw <= resize(shift_right(mp, 6), 8);                                       -- lx6*krec/64
                        ma <= resize(s_ly6, 13); mb <= signed(resize(s_krec, 13));
                    when 31 =>
                        s_lyw <= resize(shift_right(mp, 6), 8);
                    when 32 =>
                        ma <= signed(resize(s_wh, 13)); mb <= resize(s_lxw, 13);
                    when 35 =>
                        -- straights' light projection = the elbow's evaluated at the
                        -- port (|wh*lxw| <= 32*63, so >>4 stays within +/-127); both
                        -- go through the same offset ROM, so joints stay flush
                        s_pv <= resize(shift_right(mp, 4), 8);                                        -- vertical tube
                        ma <= signed(resize(s_wh, 13)); mb <= resize(s_lyw, 13);
                    when 38 =>
                        s_ph <= resize(shift_right(mp, 4), 8);                                        -- horizontal tube
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ----------------------------------------------------------------------------
    -- Sqrt-table builder (vblank): for k = 0..2047, rho^2 = (k+1/2) << sqs,
    -- walk r4 = round(4 rho) upward with adds only -- advance while
    -- (2 r4 + 1)^2 <= 64 rho^2 -- and write r4 - 4wh.  <= ~2.6k clocks.
    ----------------------------------------------------------------------------
    p_sqb : process(clk)
    begin
        if rising_edge(clk) then
            bw_we <= '0';
            if sb_go = '1' then
                sb_run  <= '1';
                sb_k    <= (others => '0');
                sb_r4   <= (others => '0');
                sb_msq  <= to_unsigned(1, 22);
                sb_inc  <= to_unsigned(8, 22);
                sb_tgt  <= shift_left(to_unsigned(32, 22), to_integer(s_sqs));
                sb_step <= shift_left(to_unsigned(64, 22), to_integer(s_sqs));
            elsif sb_run = '1' then
                if sb_msq <= sb_tgt then
                    sb_r4  <= sb_r4 + 1;
                    sb_msq <= sb_msq + sb_inc;
                    sb_inc <= sb_inc + 8;
                else
                    bw_we   <= '1';
                    bw_addr <= sb_k;
                    bw_data <= std_logic_vector(resize(signed('0' & sb_r4) - s_wh4, 16));
                    sb_tgt  <= sb_tgt + sb_step;
                    sb_k    <= sb_k + 1;
                    if sb_k = 2047 then sb_run <= '0'; end if;
                end if;
            end if;
        end if;
    end process p_sqb;

    ----------------------------------------------------------------------------
    -- Pixel pipeline.
    ----------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_dx   : signed(9 downto 0);
        variable v_par, v_or, v_dA, v_dB : std_logic;
        variable v_cN, v_cE : std_logic;
        variable v_ys   : unsigned(10 downto 0);
        variable v_lv   : signed(11 downto 0);
        variable v_l    : unsigned(7 downto 0);
        variable v_sw   : unsigned(7 downto 0);
        variable v_cx, v_cy : signed(9 downto 0);
        variable v_r2   : unsigned(16 downto 0);
        variable v_do   : signed(18 downto 0);
        variable v_rp   : unsigned(9 downto 0);
        variable v_inH, v_inV, v_shH, v_shV, v_sH : boolean;
        variable v_rp6  : unsigned(9 downto 0);
        variable v_e4   : signed(9 downto 0);
        variable v_d4   : signed(8 downto 0);
        variable v_dn   : signed(24 downto 0);
        variable v_al   : signed(19 downto 0);
        variable v_et   : signed(9 downto 0);
        variable v_aet  : unsigned(9 downto 0);
        variable v_adn  : unsigned(7 downto 0);
        variable v_tri  : unsigned(6 downto 0);
        variable v_y    : unsigned(10 downto 0);
        variable v_cf   : signed(8 downto 0);
        variable v_su   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            syncp(0) <= data_in.hsync_n & data_in.vsync_n & data_in.avid & data_in.field_n;
            for i in 1 to LATENCY - 1 loop syncp(i) <= syncp(i - 1); end loop;

            ------------------------------------------------------------------
            -- c1: local coords; pick this pixel's elbow corner (the nearer of
            -- the cell's pair) and its corner-distance squares; Image drive.
            ------------------------------------------------------------------
            v_dx := signed(resize(x_loc, 10)) - signed(resize(s_whp, 10));
            g0_dx <= v_dx;
            g0_dy <= lm_dy;
            v_par := x_idx(0) xor celly_idx(0);
            if s_rings = '1' then v_or := v_par; else v_or := s_cor; end if;
            v_dA := f_b(v_dx > lm_dy);                                    -- nearer NE than SW
            v_dB := f_b(resize(v_dx, 11) + resize(lm_dy, 11) < 0);        -- nearer NW than SE
            if v_or = '0' then v_cN := v_dA; v_cE := v_dA;                -- NE / SW pair
            else               v_cN := v_dB; v_cE := not v_dB; end if;    -- NW / SE pair
            if v_cE = '1' then g0_qx <= qxE; else g0_qx <= qxW; end if;
            if v_cN = '1' then g0_qy <= qyN; else g0_qy <= qyS; end if;
            g0_cN <= v_cN; g0_cE <= v_cE;
            g0_xc <= s_cxc; g0_par <= v_par;

            -- Image drive 0..63: the input's luma (2-tap smoothed), or with
            -- Image off a slow diagonal swell wave so P12 still breathes
            s_yprev <= unsigned(data_in.y);
            v_ys := resize(unsigned(data_in.y), 11) + resize(s_yprev, 11);
            -- contrast-stretched: Y 150..770 -> 0..63, so mid-tone video still
            -- drives the pipes nearly end to end
            v_lv := signed(resize(v_ys(10 downto 1), 12)) - 150;
            if v_lv < 0 then v_lv := (others => '0'); end if;
            v_l := resize(shift_right(unsigned(v_lv), 4), 8) + resize(shift_right(unsigned(v_lv), 5), 8)
                 + resize(shift_right(unsigned(v_lv), 7), 8);
            v_sw := resize(shift_right(resize(x_f, 13) + resize(y_f, 13), 2), 8) + s_swp;
            if s_image = '0' then
                if v_sw(7) = '1' then g0_l <= not v_sw(6 downto 1); else g0_l <= v_sw(6 downto 1); end if;
            elsif v_l > 63 then g0_l <= (others => '1');
            else                g0_l <= v_l(5 downto 0); end if;

            ------------------------------------------------------------------
            -- c2: rho^2 to the corner, radial coords, |d|, L'.
            ------------------------------------------------------------------
            p2_rho2 <= resize(g0_qx, 17) + resize(g0_qy, 17);
            if g0_cE = '1' then v_cx := signed(resize(s_whq, 10)); else v_cx := -signed(resize(s_whq, 10)); end if;
            if g0_cN = '1' then v_cy := -signed(resize(s_whq, 10)); else v_cy := signed(resize(s_whq, 10)); end if;
            p2_excx <= g0_dx - v_cx;
            p2_eycy <= g0_dy - v_cy;
            -- |excx| and |eycy|+wh without an abs (|dx|,|dy| <= wh)
            if g0_cE = '1' then p2_aex <= signed(resize(s_whq, 11)) - resize(g0_dx, 11);
            else                p2_aex <= signed(resize(s_whq, 11)) + resize(g0_dx, 11); end if;
            if g0_cN = '1' then p2_d1 <= signed(resize(s_whq & '0', 11)) + resize(g0_dy, 11);
            else                p2_d1 <= signed(resize(s_whq & '0', 11)) - resize(g0_dy, 11); end if;
            p2_adx <= resize(f_absu(g0_dx), 7);
            p2_ady <= resize(f_absu(g0_dy), 7);
            if    g0_dx > 63  then p2_dxc <= to_signed(63, 7);
            elsif g0_dx < -63 then p2_dxc <= to_signed(-63, 7);
            else                   p2_dxc <= resize(g0_dx, 7); end if;
            if    g0_dy > 63  then p2_dyc <= to_signed(63, 7);
            elsif g0_dy < -63 then p2_dyc <= to_signed(-63, 7);
            else                   p2_dyc <= resize(g0_dy, 7); end if;
            if s_pos = '1' then p2_lp <= g0_l; else p2_lp <= 63 - g0_l; end if;
            p2_xc <= g0_xc; p2_par <= g0_par;

            ------------------------------------------------------------------
            -- c3: sqrt-table address, light . radial, Image radius product,
            -- flow phase on the elbow.
            ------------------------------------------------------------------
            case s_sqs is
                when "00"   => v_r2 := p2_rho2;
                when "01"   => v_r2 := shift_right(p2_rho2, 1);
                when "10"   => v_r2 := shift_right(p2_rho2, 2);
                when others => v_r2 := shift_right(p2_rho2, 3);
            end case;
            if v_r2 > 2047 then p3_sqa <= (others => '1'); else p3_sqa <= v_r2(10 downto 0); end if;
            p3_mx  <= p2_excx * s_lxw;
            p3_my  <= p2_eycy * s_lyw;
            p3_rpm <= s_c1q * p2_lp;
            -- Flow phase "along" the pipe, continuous at every port: N/S ports
            -- sit at 0, E/W ports at 2wh (half the 4wh period).  The sign
            -- follows the Truchet 2-colouring (cell parity) so every loop
            -- circulates one way.  (elbow: +/-(|ey| - |ex| + wh))
            if p2_par = '1' then p3_alarc <= p2_d1 - p2_aex;
            else                 p3_alarc <= p2_aex - p2_d1; end if;
            p3_dxc <= p2_dxc; p3_dyc <= p2_dyc;
            p3_adx <= p2_adx; p3_ady <= p2_ady;
            p3_xc  <= p2_xc; p3_par <= p2_par;

            ------------------------------------------------------------------
            -- c4: sqrt table read (EBR); projection; this pixel's radius.
            ------------------------------------------------------------------
            if bw_we = '1' then sqtab(to_integer(bw_addr)) <= bw_data; end if;
            p4_sq <= sqtab(to_integer(p3_sqa));
            v_do := shift_right(resize(p3_mx, 19) + resize(p3_my, 19), 4);
            if    v_do > 127  then p4_do <= to_signed(127, 8);
            elsif v_do < -127 then p4_do <= to_signed(-127, 8);
            else                   p4_do <= resize(v_do, 8); end if;
            v_rp := resize(s_c0q, 10) + resize(p3_rpm(14 downto 6), 10);
            if v_rp > 255 then p4_rp4 <= (others => '1'); else p4_rp4 <= v_rp(7 downto 0); end if;
            p4_alarc <= p3_alarc;
            p4_dxc <= p3_dxc; p4_dyc <= p3_dyc;
            p4_adx <= p3_adx; p4_ady <= p3_ady;
            p4_xc  <= p3_xc; p4_par <= p3_par;

            ------------------------------------------------------------------
            -- c5: elbow section into the fabric; K ROM read; crossing strand
            -- pick (over strand by parity, under strand shadowed beside it);
            -- highlight-offset ROM address.
            ------------------------------------------------------------------
            p5_e4 <= signed(p4_sq(9 downto 0));
            p5_k  <= unsigned(C_KROM(to_integer(p4_rp4))(14 downto 0));
            v_rp6 := resize(p4_rp4, 10) + resize(p4_rp4(7 downto 1), 10);   -- 1.5 r
            v_inH := (resize(p4_ady, 10) sll 2) + 2 < resize(p4_rp4, 10);
            v_inV := (resize(p4_adx, 10) sll 2) + 2 < resize(p4_rp4, 10);
            v_shH := (resize(p4_ady, 10) sll 2) < v_rp6;
            v_shV := (resize(p4_adx, 10) sll 2) < v_rp6;
            if p4_par = '1' then v_sH := v_inH or not v_inV;               -- H strand over
            else                 v_sH := v_inH and not v_inV; end if;      -- V strand over
            p5_strH <= f_b(v_sH);
            p5_shad <= '0';
            if p4_xc = '1' then
                if p4_par = '1' and not v_inH and v_inV and v_shH then p5_shad <= '1'; end if;
                if p4_par = '0' and not v_inV and v_inH and v_shV then p5_shad <= '1'; end if;
            end if;
            if v_sH then p5_dcr <= p4_dyc; else p5_dcr <= p4_dxc; end if;
            if p4_xc = '0' then p5_saddr <= unsigned(p4_do);
            elsif v_sH then     p5_saddr <= unsigned(s_ph);
            else                p5_saddr <= unsigned(s_pv); end if;
            p5_rp4 <= p4_rp4; p5_alarc <= p4_alarc;
            p5_adx <= p4_adx; p5_ady <= p4_ady;
            p5_xc  <= p4_xc; p5_par <= p4_par;

            ------------------------------------------------------------------
            -- c6: offset ROM read (EBR); cross-section d (1/4 px) and the ONE
            -- membership test both families share; flow phase pick.
            ------------------------------------------------------------------
            p6_off <= C_SATOFF(to_integer(p5_saddr));
            p6_k   <= p5_k;
            if p5_xc = '1' then
                v_d4 := shift_left(resize(p5_dcr, 9), 2);
            else
                v_e4 := p5_e4;
                if    v_e4 > 255  then v_e4 := to_signed(255, 10);
                elsif v_e4 < -255 then v_e4 := to_signed(-255, 10); end if;
                v_d4 := resize(v_e4, 9);
            end if;
            p6_d4  <= v_d4;
            p6_hit <= f_b(resize(f_absu(v_d4), 10) + 2 < resize(p5_rp4, 10));
            if p5_xc = '0' then
                p6_al <= p5_alarc;
            elsif p5_strH = '1' then
                if p5_par = '1' then p6_al <= signed(resize(p5_adx, 11)) + signed(resize(s_whr, 11));
                else                 p6_al <= signed(resize(s_wh3, 11)) - signed(resize(p5_adx, 11)); end if;
            else
                if p5_par = '1' then p6_al <= signed(resize(s_whr, 11)) - signed(resize(p5_ady, 11));
                else                 p6_al <= signed(resize(p5_ady, 11)) - signed(resize(s_whr, 11)); end if;
            end if;
            p6_shad <= p5_shad;

            ------------------------------------------------------------------
            -- c7: unit-radius normalise, split on K's operand bits; flow phase.
            ------------------------------------------------------------------
            p7_off <= signed(p6_off(7 downto 0));
            p7_ph  <= p6_d4 * signed('0' & p6_k(14 downto 7));
            p7_pl  <= p6_d4 * signed('0' & p6_k(6 downto 0));
            v_al := resize(p6_al * signed('0' & s_krec), 20);
            p7_aln <= unsigned(v_al(12 downto 5));                       -- period 4wh -> 256
            p7_shad <= p6_shad;
            hitA(0) <= p6_hit;
            for i in 1 to 6 loop hitA(i) <= hitA(i - 1); end loop;

            ------------------------------------------------------------------
            -- c8: dn = d4 * K >> 8 (|dn| = 64 at the wall at any radius).
            ------------------------------------------------------------------
            v_dn := shift_right(shift_left(resize(p7_ph, 25), 7) + resize(p7_pl, 25), 8);
            if    v_dn > 127  then p8_dn <= to_signed(127, 8);
            elsif v_dn < -127 then p8_dn <= to_signed(-127, 8);
            else                   p8_dn <= resize(v_dn, 8); end if;
            p8_off <= p7_off; p8_aln <= p7_aln; p8_shad <= p7_shad;

            ------------------------------------------------------------------
            -- c9: shading ROM address |dn - off|, rim, flow phase.
            ------------------------------------------------------------------
            v_et  := resize(p8_dn, 10) - resize(p8_off, 10);
            v_aet := f_absu(v_et);
            if v_aet > 127 then v_aet := to_unsigned(127, 10); end if;
            p9_addr <= s_chrome & v_aet(6 downto 0);
            v_adn := f_absu(p8_dn);
            if    v_adn >= 60 then p9_rim <= "10";
            elsif v_adn >= 53 then p9_rim <= "01";
            else                   p9_rim <= "00"; end if;
            p9_fph  <= p8_aln(7 downto 1) + s_phase(14 downto 8);
            p9_shad <= p8_shad;

            ------------------------------------------------------------------
            -- c10: shading ROM (EBR); flow pulse.  c11: into the fabric; the
            -- hue angle is taken late from the live raster counters.
            ------------------------------------------------------------------
            p10_sh <= C_SHADE(to_integer(p9_addr));
            p10_rim <= p9_rim; p10_shad <= p9_shad;
            if s_flowon = '1' and p9_fph < s_floww then
                if p9_fph < (s_floww - p9_fph) then v_tri := p9_fph; else v_tri := s_floww - p9_fph; end if;
                p10_flow <= resize(shift_left(resize(v_tri, 10), to_integer(s_flowg)), 10);
            else
                p10_flow <= (others => '0');
            end if;

            p11_y8 <= unsigned(p10_sh(15 downto 8));
            p11_cf <= unsigned(p10_sh(7 downto 0));
            p11_rim <= p10_rim; p11_flow <= p10_flow; p11_shad <= p10_shad;
            p11_ang <= resize(shift_right(resize(x_f, 13) + resize(y_f, 13), to_integer(s_hshift)), 10)
                     + s_hue + s_huep(15 downto 6);

            ------------------------------------------------------------------
            -- c12: luma = shade + flow, rim fall-off + crossing shadow; chroma
            -- factor (flow whitens); saturated hue sin/cos.
            ------------------------------------------------------------------
            v_y := resize(p11_y8 & "00", 11) + resize(p11_flow, 11);
            if v_y > 800 then v_y := to_unsigned(800, 11); end if;
            if p11_shad = '1' or p11_rim = "10" then
                v_y := shift_right(v_y + 64, 1);
            elsif p11_rim = "01" then
                v_y := v_y - shift_right(v_y - 64, 2);
            end if;
            p12_y <= v_y(9 downto 0);
            v_cf := signed(resize(p11_cf, 9)) - signed(resize(p11_flow(9 downto 2), 9));
            if v_cf < 0 then v_cf := (others => '0'); end if;
            if p11_shad = '1' then p12_cf <= unsigned(v_cf(8 downto 3));
            else                   p12_cf <= unsigned(v_cf(7 downto 2)); end if;
            cosR <= f_qsat(p11_ang + 256);
            sinR <= f_qsat(p11_ang);

            ------------------------------------------------------------------
            -- c13: hue chroma * chroma factor.
            ------------------------------------------------------------------
            p13_um <= cosR * signed('0' & p12_cf);
            p13_vm <= sinR * signed('0' & p12_cf);
            p13_y  <= p12_y;

            ------------------------------------------------------------------
            -- c14: compose over black; neutral outside active video.
            ------------------------------------------------------------------
            if syncp(12)(1) = '1' and hitA(6) = '1' then
                v_su := to_signed(512, 12) + resize(shift_right(p13_um, 6), 12);
                p14_u <= unsigned(v_su(9 downto 0));
                v_su := to_signed(512, 12) + resize(shift_right(p13_vm, 6), 12);
                p14_v <= unsigned(v_su(9 downto 0));
                p14_y <= p13_y;
            else
                p14_y <= C_BLK; p14_u <= C_MID; p14_v <= C_MID;
            end if;

            s_io.y <= std_logic_vector(p14_y); s_io.u <= std_logic_vector(p14_u); s_io.v <= std_logic_vector(p14_v);
            s_io.hsync_n <= syncp(LATENCY - 1)(3);
            s_io.vsync_n <= syncp(LATENCY - 1)(2);
            s_io.avid    <= syncp(LATENCY - 1)(1);
            s_io.field_n <= syncp(LATENCY - 1)(0);
        end if;
    end process p_pipe;

    s_io_align_in.y <= s_io.y;
    s_io_align_in.u <= s_io.u;
    s_io_align_in.v <= s_io.v;
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.field_n <= s_io.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (15 -> 16).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture pipedream;
