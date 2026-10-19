-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- hypnos.vhd  (v1.0 -- razor-sharp op-art rings: circle / square / hexagon / flower)
--
-- v1.0.4 (2026-10-17): resolution scaling.  The engine keeps working in real
-- pixels across a line (so the 1-px box-filter AA stays exactly one pixel),
-- and the per-mode scale is folded into per-mode constants from the video
-- timing ID (registers_in(8)): the line step dy is in those same x-pixel
-- units (720p 1.0, 480i 2.22 per field line, 1080i 2.0, ...; Q.8, rounded),
-- the measured height follows it, and the log distance becomes the 1080-line
-- one by an additive log2(s) (s = x step: 1.5 at 720p, 2 at SD), folded into
-- the LREF subtract (Q14, exact), plus log2(s) on the radial AA gradient
-- (now per pixel again).  Ring period, warp and centre follow screen height,
-- rings stay round on a 4:3 SD screen, and interlace is taken from the
-- timing ID (field_n is tied high on hardware, so the old field_n toggle
-- test never saw it and SD rendered 2:1 tall).  1080p is unchanged (apart
-- from the first frame after power-up, which the old toggle test briefly
-- took for an interlaced field).
--
-- 1960s op-art target: Vasarely / Bridget Riley / the Vertigo spiral.  No input
-- video.  Everything happens in the LOG domain -- no CORDIC:
--
--   hi,lo    = max/min(|dx|,|dy|)    centre travels +/-2 screens off-frame
--   log2 r   = log2 hi + 0.5 log2(1 + 4^-u),  theta_oct = atan(2^-u),
--              u = log2(hi/lo)       -- 13-bit integer logs + interpolated EBR
--                                    tables; ~1e-5 accurate, far cheaper than a
--                                    12-iteration CORDIC.
--   log2 d   = shape: circle log2 r ; square log2 hi (exact Linf) ; hexagon /
--              flower log2 r + log2 S(6 theta) from an interpolated EBR table.
--   d'       = Rref*(d/Rref)^p       WARP (K4): p*(log2 d - LREF) ; detent p = 1.
--   phase    = 2^(log2 d') * f       exp2 mantissa x freq mantissa, ONE shift.
--              + arms*theta (SPIRAL K5, integer arms = seamless) - travel (P12).
--   coverage = box filter of the band square-wave over ONE PIXEL, using the
--              ANALYTIC phase gradient: radial  Lf + log2 p + (p-1)(log2 d -
--              LREF) [+ flower slope], angular log2(arms/(2 pi r)), combined in
--              log2.  Every edge is a 1-px anti-aliased line at any period /
--              warp / spiral; rings finer than ~3 px fade to the duty grey.
--   colour   = ink over paper: S9 Ink (white / colour), S10 Paper (black /
--              colour): B/W, Rainbow, Duo, Complementary.  S11 Invert.
--
-- Long-carried values ride in EBR delay lines.  The program makes its own avid
-- (parity-locked to hsync, fixed length) so coloured output never U/V-swaps a
-- line; outputs are gated to neutral outside it.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture hypnos of program_top is
    -- Synthesis program: processing_delay_clks is declared 0 in the TOML (the SDK
    -- allows 0 for synthesis programs, and it saves the IO-align pad registers).
    -- The true data_in -> data_out sync latency is 46 clocks.
    constant C_PROCESSING_DELAY_CLKS : natural := 0;

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB    : integer := 14;                 -- centre-knob detent deadband
    constant C_GL    : integer := 5;                  -- glide one-pole shift (frames)
    constant C_LREF  : integer := 10 * 16384;         -- log2(1024 px), Q14

    -- stage map (stage 0 = s_dx).  An EBR delay line written from stage W and
    -- read (straight from the EBR output register) by the logic building stage
    -- C runs its read address X = C - 2 - W behind the write address.
    constant ST_SPW : integer := 16;   constant ST_SPC : integer := 35;   -- spiral term
    constant ST_BW  : integer := 16;   constant ST_BC  : integer := 27;   -- angular gradient
    constant ST_OUT : integer := 45;                                      -- output register
    constant ST_HW  : integer := 8;    constant ST_HC  : integer := 15;   -- log2 hi + flags
    constant ST_LW  : integer := 16;   constant ST_LC  : integer := 22;   -- log2 d
    constant C_XH   : integer := ST_HC  - 2 - ST_HW;
    constant C_XL   : integer := ST_LC  - 2 - ST_LW;
    constant C_XC   : integer := ST_SPC - 2 - ST_SPW;
    constant C_XD   : integer := ST_BC  - 2 - ST_BW;

    type t_exp8  is array(0 to 63) of unsigned(8 downto 0);
    type t_exp6  is array(0 to 63) of unsigned(6 downto 0);
    type t_ht    is array(0 to 63) of unsigned(5 downto 0);
    type t_log2p is array(0 to 31) of signed(9 downto 0);
    type t_larm  is array(0 to 8)  of signed(9 downto 0);
    type t_rom256 is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_rom512 is array(0 to 511) of std_logic_vector(15 downto 0);

    -- tables generated by .hypnos_work/gen_tables2.py
    constant C_LOGV : t_rom256 := (
        x"0000", x"005C", x"00B8", x"0113", x"016E", x"01C9", x"0224", x"027E",
        x"02D7", x"0331", x"038A", x"03E2", x"043B", x"0493", x"04EB", x"0542",
        x"0599", x"05F0", x"0646", x"069C", x"06F2", x"0748", x"079D", x"07F2",
        x"0846", x"089A", x"08EE", x"0942", x"0995", x"09E9", x"0A3B", x"0A8E",
        x"0AE0", x"0B32", x"0B84", x"0BD5", x"0C26", x"0C77", x"0CC7", x"0D18",
        x"0D68", x"0DB7", x"0E07", x"0E56", x"0EA5", x"0EF4", x"0F42", x"0F90",
        x"0FDE", x"102C", x"1079", x"10C6", x"1113", x"1160", x"11AC", x"11F8",
        x"1244", x"1290", x"12DB", x"1326", x"1371", x"13BC", x"1406", x"1450",
        x"149A", x"14E4", x"152E", x"1577", x"15C0", x"1609", x"1652", x"169A",
        x"16E2", x"172A", x"1772", x"17B9", x"1801", x"1848", x"188F", x"18D5",
        x"191C", x"1962", x"19A8", x"19EE", x"1A33", x"1A79", x"1ABE", x"1B03",
        x"1B48", x"1B8D", x"1BD1", x"1C15", x"1C59", x"1C9D", x"1CE1", x"1D24",
        x"1D67", x"1DAA", x"1DED", x"1E30", x"1E72", x"1EB5", x"1EF7", x"1F39",
        x"1F7B", x"1FBC", x"1FFD", x"203F", x"2080", x"20C1", x"2101", x"2142",
        x"2182", x"21C2", x"2202", x"2242", x"2282", x"22C1", x"2300", x"233F",
        x"237E", x"23BD", x"23FC", x"243A", x"2479", x"24B7", x"24F5", x"2532",
        x"2570", x"25AE", x"25EB", x"2628", x"2665", x"26A2", x"26DE", x"271B",
        x"2757", x"2794", x"27D0", x"280C", x"2847", x"2883", x"28BE", x"28FA",
        x"2935", x"2970", x"29AB", x"29E6", x"2A20", x"2A5B", x"2A95", x"2ACF",
        x"2B09", x"2B43", x"2B7D", x"2BB6", x"2BF0", x"2C29", x"2C62", x"2C9B",
        x"2CD4", x"2D0D", x"2D45", x"2D7E", x"2DB6", x"2DEE", x"2E26", x"2E5E",
        x"2E96", x"2ECE", x"2F05", x"2F3D", x"2F74", x"2FAB", x"2FE2", x"3019",
        x"3050", x"3087", x"30BD", x"30F4", x"312A", x"3160", x"3196", x"31CC",
        x"3202", x"3237", x"326D", x"32A2", x"32D8", x"330D", x"3342", x"3377",
        x"33AC", x"33E0", x"3415", x"3449", x"347E", x"34B2", x"34E6", x"351A",
        x"354E", x"3582", x"35B6", x"35E9", x"361D", x"3650", x"3683", x"36B6",
        x"36E9", x"371C", x"374F", x"3781", x"37B4", x"37E7", x"3819", x"384B",
        x"387D", x"38AF", x"38E1", x"3913", x"3945", x"3976", x"39A8", x"39D9",
        x"3A0A", x"3A3C", x"3A6D", x"3A9E", x"3ACF", x"3AFF", x"3B30", x"3B61",
        x"3B91", x"3BC2", x"3BF2", x"3C22", x"3C52", x"3C82", x"3CB2", x"3CE2",
        x"3D12", x"3D41", x"3D71", x"3DA0", x"3DCF", x"3DFF", x"3E2E", x"3E5D",
        x"3E8C", x"3EBB", x"3EE9", x"3F18", x"3F47", x"3F75", x"3FA3", x"3FD2");
    constant C_LOGS : t_rom256 := (
        x"005C", x"005C", x"005B", x"005B", x"005B", x"005B", x"005A", x"0059",
        x"005A", x"0059", x"0058", x"0059", x"0058", x"0058", x"0057", x"0057",
        x"0057", x"0056", x"0056", x"0056", x"0056", x"0055", x"0055", x"0054",
        x"0054", x"0054", x"0054", x"0053", x"0054", x"0052", x"0053", x"0052",
        x"0052", x"0052", x"0051", x"0051", x"0051", x"0050", x"0051", x"0050",
        x"004F", x"0050", x"004F", x"004F", x"004F", x"004E", x"004E", x"004E",
        x"004E", x"004D", x"004D", x"004D", x"004D", x"004C", x"004C", x"004C",
        x"004C", x"004B", x"004B", x"004B", x"004B", x"004A", x"004A", x"004A",
        x"004A", x"004A", x"0049", x"0049", x"0049", x"0049", x"0048", x"0048",
        x"0048", x"0048", x"0047", x"0048", x"0047", x"0047", x"0046", x"0047",
        x"0046", x"0046", x"0046", x"0045", x"0046", x"0045", x"0045", x"0045",
        x"0045", x"0044", x"0044", x"0044", x"0044", x"0044", x"0043", x"0043",
        x"0043", x"0043", x"0043", x"0042", x"0043", x"0042", x"0042", x"0042",
        x"0041", x"0041", x"0042", x"0041", x"0041", x"0040", x"0041", x"0040",
        x"0040", x"0040", x"0040", x"0040", x"003F", x"003F", x"003F", x"003F",
        x"003F", x"003F", x"003E", x"003F", x"003E", x"003E", x"003D", x"003E",
        x"003E", x"003D", x"003D", x"003D", x"003D", x"003C", x"003D", x"003C",
        x"003D", x"003C", x"003C", x"003B", x"003C", x"003B", x"003C", x"003B",
        x"003B", x"003B", x"003B", x"003A", x"003B", x"003A", x"003A", x"003A",
        x"003A", x"003A", x"0039", x"003A", x"0039", x"0039", x"0039", x"0039",
        x"0039", x"0038", x"0039", x"0038", x"0038", x"0038", x"0038", x"0038",
        x"0038", x"0037", x"0038", x"0037", x"0037", x"0037", x"0037", x"0037",
        x"0037", x"0036", x"0037", x"0036", x"0036", x"0036", x"0036", x"0036",
        x"0035", x"0036", x"0035", x"0036", x"0035", x"0035", x"0035", x"0035",
        x"0034", x"0035", x"0034", x"0035", x"0034", x"0034", x"0034", x"0034",
        x"0034", x"0034", x"0033", x"0034", x"0033", x"0033", x"0033", x"0033",
        x"0033", x"0033", x"0032", x"0033", x"0033", x"0032", x"0032", x"0032",
        x"0032", x"0032", x"0032", x"0032", x"0031", x"0032", x"0031", x"0031",
        x"0032", x"0031", x"0031", x"0031", x"0030", x"0031", x"0031", x"0030",
        x"0031", x"0030", x"0030", x"0030", x"0030", x"0030", x"0030", x"0030",
        x"002F", x"0030", x"002F", x"002F", x"0030", x"002F", x"002F", x"002F",
        x"002F", x"002E", x"002F", x"002F", x"002E", x"002E", x"002F", x"002E");
    constant C_UG : t_rom256 := (
        x"2000", x"1E0B", x"1C2C", x"1A64", x"18B1", x"1713", x"158B", x"1417",
        x"12B8", x"116C", x"1034", x"0F0E", x"0DFA", x"0CF7", x"0C04", x"0B21",
        x"0A4D", x"0987", x"08CF", x"0823", x"0784", x"06F0", x"0666", x"05E6",
        x"0570", x"0503", x"049D", x"0440", x"03E9", x"0399", x"034F", x"030B",
        x"02CC", x"0293", x"025D", x"022C", x"01FF", x"01D6", x"01AF", x"018C",
        x"016C", x"014E", x"0133", x"0119", x"0102", x"00ED", x"00DA", x"00C8",
        x"00B7", x"00A8", x"009A", x"008E", x"0082", x"0077", x"006D", x"0064",
        x"005C", x"0054", x"004D", x"0047", x"0041", x"003C", x"0037", x"0032",
        x"002E", x"002A", x"0027", x"0024", x"0021", x"001E", x"001B", x"0019",
        x"0017", x"0015", x"0013", x"0012", x"0010", x"000F", x"000E", x"000D",
        x"000C", x"000B", x"000A", x"0009", x"0008", x"0007", x"0007", x"0006",
        x"0006", x"0005", x"0005", x"0004", x"0004", x"0004", x"0003", x"0003",
        x"0003", x"0003", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_UGS : t_rom256 := (
        x"FE0B", x"FE21", x"FE38", x"FE4D", x"FE62", x"FE78", x"FE8C", x"FEA1",
        x"FEB4", x"FEC8", x"FEDA", x"FEEC", x"FEFD", x"FF0D", x"FF1D", x"FF2C",
        x"FF3A", x"FF48", x"FF54", x"FF61", x"FF6C", x"FF76", x"FF80", x"FF8A",
        x"FF93", x"FF9A", x"FFA3", x"FFA9", x"FFB0", x"FFB6", x"FFBC", x"FFC1",
        x"FFC7", x"FFCA", x"FFCF", x"FFD3", x"FFD7", x"FFD9", x"FFDD", x"FFE0",
        x"FFE2", x"FFE5", x"FFE6", x"FFE9", x"FFEB", x"FFED", x"FFEE", x"FFEF",
        x"FFF1", x"FFF2", x"FFF4", x"FFF4", x"FFF5", x"FFF6", x"FFF7", x"FFF8",
        x"FFF8", x"FFF9", x"FFFA", x"FFFA", x"FFFB", x"FFFB", x"FFFB", x"FFFC",
        x"FFFC", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFE", x"FFFE",
        x"FFFE", x"FFFE", x"FFFF", x"FFFE", x"FFFF", x"FFFF", x"FFFF", x"FFFF",
        x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"0000", x"FFFF", x"0000",
        x"FFFF", x"0000", x"FFFF", x"0000", x"0000", x"FFFF", x"0000", x"0000",
        x"0000", x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000", x"FFFF",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_UA : t_rom256 := (
        x"8000", x"7C79", x"78F3", x"7570", x"71F3", x"6E7C", x"6B0E", x"67A9",
        x"644F", x"6101", x"5DC1", x"5A8F", x"576D", x"545C", x"515B", x"4E6C",
        x"4B90", x"48C7", x"4610", x"436D", x"40DE", x"3E62", x"3BF9", x"39A4",
        x"3763", x"3534", x"3318", x"310F", x"2F19", x"2D34", x"2B61", x"299E",
        x"27ED", x"264C", x"24BA", x"2338", x"21C5", x"2060", x"1F09", x"1DC0",
        x"1C84", x"1B54", x"1A31", x"1919", x"180D", x"170B", x"1614", x"1527",
        x"1444", x"136A", x"129A", x"11D1", x"1111", x"1059", x"0FA9", x"0F00",
        x"0E5E", x"0DC3", x"0D2E", x"0CA0", x"0C17", x"0B94", x"0B17", x"0A9F",
        x"0A2C", x"09BE", x"0955", x"08F0", x"088F", x"0832", x"07D9", x"0784",
        x"0733", x"06E5", x"069A", x"0652", x"060E", x"05CC", x"058D", x"0551",
        x"0517", x"04E0", x"04AB", x"0479", x"0448", x"041A", x"03ED", x"03C3",
        x"039A", x"0373", x"034D", x"0329", x"0307", x"02E6", x"02C7", x"02A9",
        x"028C", x"0270", x"0256", x"023C", x"0224", x"020D", x"01F7", x"01E1",
        x"01CD", x"01B9", x"01A7", x"0195", x"0184", x"0173", x"0163", x"0154",
        x"0146", x"0138", x"012B", x"011E", x"0112", x"0106", x"00FB", x"00F1",
        x"00E6", x"00DD", x"00D3", x"00CA", x"00C2", x"00BA", x"00B2", x"00AA",
        x"00A3", x"009C", x"0095", x"008F", x"0089", x"0083", x"007E", x"0078",
        x"0073", x"006E", x"006A", x"0065", x"0061", x"005D", x"0059", x"0055",
        x"0051", x"004E", x"004B", x"0048", x"0045", x"0042", x"003F", x"003C",
        x"003A", x"0037", x"0035", x"0033", x"0030", x"002E", x"002C", x"002B",
        x"0029", x"0027", x"0025", x"0024", x"0022", x"0021", x"001F", x"001E",
        x"001D", x"001C", x"001A", x"0019", x"0018", x"0017", x"0016", x"0015",
        x"0014", x"0014", x"0013", x"0012", x"0011", x"0010", x"0010", x"000F",
        x"000E", x"000E", x"000D", x"000D", x"000C", x"000C", x"000B", x"000B",
        x"000A", x"000A", x"0009", x"0009", x"0009", x"0008", x"0008", x"0008",
        x"0007", x"0007", x"0007", x"0006", x"0006", x"0006", x"0006", x"0005",
        x"0005", x"0005", x"0005", x"0004", x"0004", x"0004", x"0004", x"0004",
        x"0004", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003",
        x"0003", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002",
        x"0002", x"0002", x"0002", x"0002", x"0002", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001");
    constant C_UAS : t_rom256 := (
        x"FC79", x"FC7A", x"FC7D", x"FC83", x"FC89", x"FC92", x"FC9B", x"FCA6",
        x"FCB2", x"FCC0", x"FCCE", x"FCDE", x"FCEF", x"FCFF", x"FD11", x"FD24",
        x"FD37", x"FD49", x"FD5D", x"FD71", x"FD84", x"FD97", x"FDAB", x"FDBF",
        x"FDD1", x"FDE4", x"FDF7", x"FE0A", x"FE1B", x"FE2D", x"FE3D", x"FE4F",
        x"FE5F", x"FE6E", x"FE7E", x"FE8D", x"FE9B", x"FEA9", x"FEB7", x"FEC4",
        x"FED0", x"FEDD", x"FEE8", x"FEF4", x"FEFE", x"FF09", x"FF13", x"FF1D",
        x"FF26", x"FF30", x"FF37", x"FF40", x"FF48", x"FF50", x"FF57", x"FF5E",
        x"FF65", x"FF6B", x"FF72", x"FF77", x"FF7D", x"FF83", x"FF88", x"FF8D",
        x"FF92", x"FF97", x"FF9B", x"FF9F", x"FFA3", x"FFA7", x"FFAB", x"FFAF",
        x"FFB2", x"FFB5", x"FFB8", x"FFBC", x"FFBE", x"FFC1", x"FFC4", x"FFC6",
        x"FFC9", x"FFCB", x"FFCE", x"FFCF", x"FFD2", x"FFD3", x"FFD6", x"FFD7",
        x"FFD9", x"FFDA", x"FFDC", x"FFDE", x"FFDF", x"FFE1", x"FFE2", x"FFE3",
        x"FFE4", x"FFE6", x"FFE6", x"FFE8", x"FFE9", x"FFEA", x"FFEA", x"FFEC",
        x"FFEC", x"FFEE", x"FFEE", x"FFEF", x"FFEF", x"FFF0", x"FFF1", x"FFF2",
        x"FFF2", x"FFF3", x"FFF3", x"FFF4", x"FFF4", x"FFF5", x"FFF6", x"FFF5",
        x"FFF7", x"FFF6", x"FFF7", x"FFF8", x"FFF8", x"FFF8", x"FFF8", x"FFF9",
        x"FFF9", x"FFF9", x"FFFA", x"FFFA", x"FFFA", x"FFFB", x"FFFA", x"FFFB",
        x"FFFB", x"FFFC", x"FFFB", x"FFFC", x"FFFC", x"FFFC", x"FFFC", x"FFFC",
        x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFE",
        x"FFFD", x"FFFE", x"FFFE", x"FFFD", x"FFFE", x"FFFE", x"FFFF", x"FFFE",
        x"FFFE", x"FFFE", x"FFFF", x"FFFE", x"FFFF", x"FFFE", x"FFFF", x"FFFF",
        x"FFFF", x"FFFE", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"FFFF",
        x"0000", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"0000", x"FFFF", x"FFFF",
        x"0000", x"FFFF", x"0000", x"FFFF", x"0000", x"FFFF", x"0000", x"FFFF",
        x"0000", x"FFFF", x"0000", x"0000", x"FFFF", x"0000", x"0000", x"FFFF",
        x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000", x"FFFF", x"0000",
        x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_SHV : t_rom512 := (
        x"F2B8", x"F2D4", x"F2F0", x"F30B", x"F327", x"F342", x"F35D", x"F378",
        x"F393", x"F3AE", x"F3C9", x"F3E3", x"F3FE", x"F418", x"F432", x"F44C",
        x"F466", x"F480", x"F499", x"F4B3", x"F4CC", x"F4E6", x"F4FF", x"F518",
        x"F531", x"F549", x"F562", x"F57B", x"F593", x"F5AB", x"F5C3", x"F5DB",
        x"F5F3", x"F60B", x"F623", x"F63A", x"F652", x"F669", x"F680", x"F697",
        x"F6AE", x"F6C5", x"F6DC", x"F6F2", x"F709", x"F71F", x"F735", x"F74B",
        x"F761", x"F777", x"F78D", x"F7A2", x"F7B8", x"F7CD", x"F7E2", x"F7F8",
        x"F80D", x"F822", x"F836", x"F84B", x"F860", x"F874", x"F888", x"F89C",
        x"F8B1", x"F8C5", x"F8D8", x"F8EC", x"F900", x"F913", x"F927", x"F93A",
        x"F94D", x"F960", x"F973", x"F986", x"F999", x"F9AB", x"F9BE", x"F9D0",
        x"F9E2", x"F9F4", x"FA06", x"FA18", x"FA2A", x"FA3C", x"FA4D", x"FA5F",
        x"FA70", x"FA81", x"FA93", x"FAA4", x"FAB5", x"FAC5", x"FAD6", x"FAE7",
        x"FAF7", x"FB07", x"FB18", x"FB28", x"FB38", x"FB48", x"FB58", x"FB67",
        x"FB77", x"FB86", x"FB96", x"FBA5", x"FBB4", x"FBC3", x"FBD2", x"FBE1",
        x"FBF0", x"FBFE", x"FC0D", x"FC1B", x"FC29", x"FC38", x"FC46", x"FC54",
        x"FC62", x"FC6F", x"FC7D", x"FC8A", x"FC98", x"FCA5", x"FCB2", x"FCC0",
        x"FCCD", x"FCD9", x"FCE6", x"FCF3", x"FD00", x"FD0C", x"FD18", x"FD25",
        x"FD31", x"FD3D", x"FD49", x"FD55", x"FD60", x"FD6C", x"FD78", x"FD83",
        x"FD8E", x"FD9A", x"FDA5", x"FDB0", x"FDBB", x"FDC5", x"FDD0", x"FDDB",
        x"FDE5", x"FDF0", x"FDFA", x"FE04", x"FE0E", x"FE18", x"FE22", x"FE2C",
        x"FE35", x"FE3F", x"FE48", x"FE52", x"FE5B", x"FE64", x"FE6D", x"FE76",
        x"FE7F", x"FE88", x"FE90", x"FE99", x"FEA1", x"FEAA", x"FEB2", x"FEBA",
        x"FEC2", x"FECA", x"FED2", x"FEDA", x"FEE1", x"FEE9", x"FEF0", x"FEF8",
        x"FEFF", x"FF06", x"FF0D", x"FF14", x"FF1B", x"FF21", x"FF28", x"FF2E",
        x"FF35", x"FF3B", x"FF41", x"FF48", x"FF4E", x"FF53", x"FF59", x"FF5F",
        x"FF65", x"FF6A", x"FF70", x"FF75", x"FF7A", x"FF7F", x"FF84", x"FF89",
        x"FF8E", x"FF93", x"FF97", x"FF9C", x"FFA0", x"FFA4", x"FFA9", x"FFAD",
        x"FFB1", x"FFB5", x"FFB9", x"FFBC", x"FFC0", x"FFC3", x"FFC7", x"FFCA",
        x"FFCD", x"FFD0", x"FFD3", x"FFD6", x"FFD9", x"FFDC", x"FFDF", x"FFE1",
        x"FFE4", x"FFE6", x"FFE8", x"FFEA", x"FFEC", x"FFEE", x"FFF0", x"FFF2",
        x"FFF3", x"FFF5", x"FFF6", x"FFF8", x"FFF9", x"FFFA", x"FFFB", x"FFFC",
        x"FFFD", x"FFFE", x"FFFE", x"FFFF", x"FFFF", x"0000", x"0000", x"0000",
        x"16CB", x"16CB", x"16CA", x"16C8", x"16C5", x"16C1", x"16BD", x"16B8",
        x"16B2", x"16AC", x"16A4", x"169C", x"1693", x"1689", x"167F", x"1674",
        x"1667", x"165B", x"164D", x"163F", x"1630", x"1620", x"160F", x"15FE",
        x"15EB", x"15D8", x"15C5", x"15B0", x"159B", x"1585", x"156E", x"1556",
        x"153E", x"1525", x"150B", x"14F1", x"14D5", x"14B9", x"149C", x"147F",
        x"1460", x"1441", x"1422", x"1401", x"13E0", x"13BE", x"139B", x"1377",
        x"1353", x"132E", x"1309", x"12E2", x"12BB", x"1293", x"126B", x"1241",
        x"1217", x"11ED", x"11C1", x"1195", x"1168", x"113B", x"110C", x"10DE",
        x"10AE", x"107E", x"104D", x"101B", x"0FE9", x"0FB6", x"0F82", x"0F4E",
        x"0F19", x"0EE3", x"0EAD", x"0E76", x"0E3E", x"0E06", x"0DCD", x"0D94",
        x"0D5A", x"0D1F", x"0CE3", x"0CA7", x"0C6B", x"0C2E", x"0BF0", x"0BB1",
        x"0B72", x"0B33", x"0AF3", x"0AB2", x"0A71", x"0A2F", x"09EC", x"09AA",
        x"0966", x"0922", x"08DD", x"0898", x"0853", x"080D", x"07C6", x"077F",
        x"0737", x"06EF", x"06A6", x"065D", x"0614", x"05CA", x"057F", x"0534",
        x"04E9", x"049D", x"0451", x"0405", x"03B8", x"036A", x"031D", x"02CE",
        x"0280", x"0231", x"01E2", x"0192", x"0143", x"00F2", x"00A2", x"0051",
        x"0000", x"FFAF", x"FF5D", x"FF0B", x"FEB9", x"FE67", x"FE14", x"FDC1",
        x"FD6E", x"FD1B", x"FCC8", x"FC74", x"FC20", x"FBCD", x"FB79", x"FB24",
        x"FAD0", x"FA7C", x"FA28", x"F9D3", x"F97F", x"F92A", x"F8D5", x"F881",
        x"F82C", x"F7D8", x"F783", x"F72F", x"F6DA", x"F686", x"F631", x"F5DD",
        x"F589", x"F535", x"F4E1", x"F48E", x"F43A", x"F3E7", x"F394", x"F341",
        x"F2EE", x"F29C", x"F24A", x"F1F8", x"F1A6", x"F155", x"F104", x"F0B4",
        x"F063", x"F014", x"EFC4", x"EF75", x"EF27", x"EED9", x"EE8B", x"EE3E",
        x"EDF1", x"EDA5", x"ED5A", x"ED0F", x"ECC4", x"EC7B", x"EC31", x"EBE9",
        x"EBA1", x"EB5A", x"EB13", x"EACD", x"EA88", x"EA43", x"EA00", x"E9BD",
        x"E97B", x"E939", x"E8F9", x"E8B9", x"E87A", x"E83C", x"E7FF", x"E7C3",
        x"E788", x"E74D", x"E714", x"E6DB", x"E6A4", x"E66E", x"E638", x"E604",
        x"E5D0", x"E59E", x"E56D", x"E53D", x"E50E", x"E4E0", x"E4B3", x"E487",
        x"E45D", x"E433", x"E40B", x"E3E4", x"E3BE", x"E39A", x"E377", x"E354",
        x"E334", x"E314", x"E2F6", x"E2D9", x"E2BD", x"E2A3", x"E289", x"E272",
        x"E25B", x"E246", x"E232", x"E220", x"E20E", x"E1FF", x"E1F0", x"E1E3",
        x"E1D7", x"E1CD", x"E1C4", x"E1BC", x"E1B6", x"E1B1", x"E1AE", x"E1AC");
    constant C_SHS : t_rom512 := (
        x"1C00", x"1C00", x"1B00", x"1C00", x"1B00", x"1B00", x"1B00", x"1B00",
        x"1B00", x"1B00", x"1A00", x"1B00", x"1A00", x"1A00", x"1A00", x"1A00",
        x"1A00", x"1900", x"1A00", x"1900", x"1A00", x"1900", x"1900", x"1900",
        x"1800", x"1900", x"1900", x"1800", x"1800", x"1800", x"1800", x"1800",
        x"1800", x"1800", x"1700", x"1800", x"1700", x"1700", x"1700", x"1700",
        x"1700", x"1700", x"1600", x"1700", x"1600", x"1600", x"1600", x"1600",
        x"1600", x"1600", x"1500", x"1600", x"1500", x"1500", x"1600", x"1500",
        x"1500", x"1400", x"1500", x"1500", x"1400", x"1400", x"1400", x"1500",
        x"1400", x"1300", x"1400", x"1400", x"1300", x"1400", x"1300", x"1300",
        x"1300", x"1300", x"1300", x"1300", x"1200", x"1300", x"1200", x"1200",
        x"1200", x"1200", x"1200", x"1200", x"1200", x"1100", x"1200", x"1100",
        x"1100", x"1200", x"1100", x"1100", x"1000", x"1100", x"1100", x"1000",
        x"1000", x"1100", x"1000", x"1000", x"1000", x"1000", x"0F00", x"1000",
        x"0F00", x"1000", x"0F00", x"0F00", x"0F00", x"0F00", x"0F00", x"0F00",
        x"0E00", x"0F00", x"0E00", x"0E00", x"0F00", x"0E00", x"0E00", x"0E00",
        x"0D00", x"0E00", x"0D00", x"0E00", x"0D00", x"0D00", x"0E00", x"0D00",
        x"0C00", x"0D00", x"0D00", x"0D00", x"0C00", x"0C00", x"0D00", x"0C00",
        x"0C00", x"0C00", x"0C00", x"0B00", x"0C00", x"0C00", x"0B00", x"0B00",
        x"0C00", x"0B00", x"0B00", x"0B00", x"0A00", x"0B00", x"0B00", x"0A00",
        x"0B00", x"0A00", x"0A00", x"0A00", x"0A00", x"0A00", x"0A00", x"0900",
        x"0A00", x"0900", x"0A00", x"0900", x"0900", x"0900", x"0900", x"0900",
        x"0900", x"0800", x"0900", x"0800", x"0900", x"0800", x"0800", x"0800",
        x"0800", x"0800", x"0800", x"0700", x"0800", x"0700", x"0800", x"0700",
        x"0700", x"0700", x"0700", x"0700", x"0600", x"0700", x"0600", x"0700",
        x"0600", x"0600", x"0700", x"0600", x"0500", x"0600", x"0600", x"0600",
        x"0500", x"0600", x"0500", x"0500", x"0500", x"0500", x"0500", x"0500",
        x"0500", x"0400", x"0500", x"0400", x"0400", x"0500", x"0400", x"0400",
        x"0400", x"0400", x"0300", x"0400", x"0300", x"0400", x"0300", x"0300",
        x"0300", x"0300", x"0300", x"0300", x"0300", x"0300", x"0200", x"0300",
        x"0200", x"0200", x"0200", x"0200", x"0200", x"0200", x"0200", x"0100",
        x"0200", x"0100", x"0200", x"0100", x"0100", x"0100", x"0100", x"0100",
        x"0100", x"0000", x"0100", x"0000", x"0100", x"0000", x"0000", x"0000",
        x"0017", x"FF17", x"FE17", x"FD17", x"FC17", x"FC17", x"FB17", x"FA17",
        x"FA17", x"F818", x"F818", x"F718", x"F618", x"F619", x"F519", x"F319",
        x"F419", x"F21A", x"F21A", x"F11A", x"F01B", x"EF1B", x"EF1B", x"ED1C",
        x"ED1C", x"ED1D", x"EB1D", x"EB1E", x"EA1E", x"E91F", x"E81F", x"E81F",
        x"E720", x"E620", x"E621", x"E421", x"E422", x"E322", x"E323", x"E123",
        x"E124", x"E125", x"DF25", x"DF26", x"DE26", x"DD27", x"DC27", x"DC28",
        x"DB28", x"DB29", x"D929", x"D92A", x"D82A", x"D82B", x"D62B", x"D62C",
        x"D62C", x"D42D", x"D42D", x"D32E", x"D32E", x"D12F", x"D22F", x"D030",
        x"D030", x"CF31", x"CE31", x"CE32", x"CD32", x"CC33", x"CC33", x"CB33",
        x"CA34", x"CA34", x"C935", x"C835", x"C835", x"C736", x"C736", x"C637",
        x"C537", x"C437", x"C438", x"C438", x"C338", x"C239", x"C139", x"C139",
        x"C13A", x"C03A", x"BF3A", x"BF3A", x"BE3B", x"BD3B", x"BE3B", x"BC3B",
        x"BC3C", x"BB3C", x"BB3C", x"BB3C", x"BA3C", x"B93D", x"B93D", x"B83D",
        x"B83D", x"B73D", x"B73D", x"B73D", x"B63E", x"B53E", x"B53E", x"B53E",
        x"B43E", x"B43E", x"B43E", x"B33E", x"B23E", x"B33E", x"B13E", x"B23E",
        x"B13E", x"B13E", x"B03E", x"B13E", x"AF3E", x"B03E", x"AF3E", x"AF3E",
        x"AF3E", x"AE3E", x"AE3E", x"AE3E", x"AE3D", x"AD3D", x"AD3D", x"AD3D",
        x"AD3D", x"AD3D", x"AC3D", x"AC3C", x"AD3C", x"AC3C", x"AB3C", x"AC3B",
        x"AC3B", x"AC3B", x"AB3B", x"AC3A", x"AB3A", x"AB3A", x"AC3A", x"AB39",
        x"AC39", x"AB39", x"AC38", x"AB38", x"AC37", x"AB37", x"AC37", x"AC36",
        x"AC36", x"AC35", x"AD35", x"AC34", x"AD34", x"AD33", x"AD33", x"AD32",
        x"AE32", x"AE31", x"AE31", x"AE30", x"AF30", x"AF2F", x"B02E", x"AF2E",
        x"B12D", x"B02D", x"B12C", x"B22B", x"B22B", x"B22A", x"B329", x"B328",
        x"B428", x"B527", x"B526", x"B525", x"B725", x"B624", x"B823", x"B822",
        x"B921", x"B920", x"BA1F", x"BB1F", x"BB1E", x"BD1D", x"BD1C", x"BE1B",
        x"BE1A", x"C019", x"C018", x"C117", x"C216", x"C315", x"C414", x"C513",
        x"C511", x"C710", x"C70F", x"C90E", x"CA0D", x"CA0C", x"CC0B", x"CC09",
        x"CE08", x"CF07", x"D006", x"D105", x"D203", x"D302", x"D401", x"D600",
        x"D6FE", x"D8FD", x"D9FC", x"DAFA", x"DCF9", x"DDF8", x"DDF7", x"E0F5",
        x"E0F4", x"E2F3", x"E3F2", x"E4F1", x"E6EF", x"E6EE", x"E9ED", x"E9EC",
        x"EBEB", x"ECEA", x"EEE9", x"EEE8", x"F1E7", x"F1E6", x"F3E6", x"F4E5",
        x"F6E4", x"F7E4", x"F8E3", x"FAE3", x"FBE2", x"FDE2", x"FEE2", x"FFE2");
    constant C_EXM : t_rom256 := (
        x"4000", x"402C", x"4059", x"4086", x"40B2", x"40DF", x"410C", x"4139",
        x"4167", x"4194", x"41C2", x"41EF", x"421D", x"424B", x"4279", x"42A7",
        x"42D5", x"4304", x"4332", x"4361", x"4390", x"43BF", x"43EE", x"441D",
        x"444C", x"447B", x"44AB", x"44DB", x"450A", x"453A", x"456A", x"459B",
        x"45CB", x"45FB", x"462C", x"465D", x"468D", x"46BE", x"46F0", x"4721",
        x"4752", x"4784", x"47B5", x"47E7", x"4819", x"484B", x"487D", x"48AF",
        x"48E2", x"4914", x"4947", x"497A", x"49AD", x"49E0", x"4A13", x"4A47",
        x"4A7A", x"4AAE", x"4AE2", x"4B16", x"4B4A", x"4B7E", x"4BB3", x"4BE7",
        x"4C1C", x"4C51", x"4C86", x"4CBB", x"4CF0", x"4D26", x"4D5B", x"4D91",
        x"4DC7", x"4DFD", x"4E33", x"4E69", x"4E9F", x"4ED6", x"4F0D", x"4F44",
        x"4F7B", x"4FB2", x"4FE9", x"5021", x"5058", x"5090", x"50C8", x"5100",
        x"5138", x"5171", x"51A9", x"51E2", x"521B", x"5254", x"528D", x"52C6",
        x"52FF", x"5339", x"5373", x"53AD", x"53E7", x"5421", x"545B", x"5496",
        x"54D1", x"550C", x"5547", x"5582", x"55BD", x"55F9", x"5634", x"5670",
        x"56AC", x"56E8", x"5725", x"5761", x"579E", x"57DB", x"5818", x"5855",
        x"5892", x"58CF", x"590D", x"594B", x"5989", x"59C7", x"5A05", x"5A44",
        x"5A82", x"5AC1", x"5B00", x"5B3F", x"5B7F", x"5BBE", x"5BFE", x"5C3E",
        x"5C7E", x"5CBE", x"5CFE", x"5D3F", x"5D80", x"5DC1", x"5E02", x"5E43",
        x"5E84", x"5EC6", x"5F08", x"5F4A", x"5F8C", x"5FCE", x"6011", x"6053",
        x"6096", x"60D9", x"611C", x"6160", x"61A3", x"61E7", x"622B", x"626F",
        x"62B4", x"62F8", x"633D", x"6382", x"63C7", x"640C", x"6451", x"6497",
        x"64DD", x"6523", x"6569", x"65AF", x"65F6", x"663D", x"6684", x"66CB",
        x"6712", x"675A", x"67A2", x"67E9", x"6832", x"687A", x"68C2", x"690B",
        x"6954", x"699D", x"69E6", x"6A30", x"6A7A", x"6AC4", x"6B0E", x"6B58",
        x"6BA2", x"6BED", x"6C38", x"6C83", x"6CCF", x"6D1A", x"6D66", x"6DB2",
        x"6DFE", x"6E4A", x"6E97", x"6EE4", x"6F30", x"6F7E", x"6FCB", x"7019",
        x"7066", x"70B4", x"7103", x"7151", x"71A0", x"71EF", x"723E", x"728D",
        x"72DD", x"732C", x"737C", x"73CC", x"741D", x"746D", x"74BE", x"750F",
        x"7560", x"75B2", x"7604", x"7655", x"76A8", x"76FA", x"774D", x"779F",
        x"77F2", x"7846", x"7899", x"78ED", x"7941", x"7995", x"79E9", x"7A3E",
        x"7A93", x"7AE8", x"7B3D", x"7B93", x"7BE8", x"7C3E", x"7C95", x"7CEB",
        x"7D42", x"7D99", x"7DF0", x"7E47", x"7E9F", x"7EF7", x"7F4F", x"7FA7");
    constant C_EXS : t_rom256 := (
        x"002C", x"002D", x"002D", x"002C", x"002D", x"002D", x"002D", x"002E",
        x"002D", x"002E", x"002D", x"002E", x"002E", x"002E", x"002E", x"002E",
        x"002F", x"002E", x"002F", x"002F", x"002F", x"002F", x"002F", x"002F",
        x"002F", x"0030", x"0030", x"002F", x"0030", x"0030", x"0031", x"0030",
        x"0030", x"0031", x"0031", x"0030", x"0031", x"0032", x"0031", x"0031",
        x"0032", x"0031", x"0032", x"0032", x"0032", x"0032", x"0032", x"0033",
        x"0032", x"0033", x"0033", x"0033", x"0033", x"0033", x"0034", x"0033",
        x"0034", x"0034", x"0034", x"0034", x"0034", x"0035", x"0034", x"0035",
        x"0035", x"0035", x"0035", x"0035", x"0036", x"0035", x"0036", x"0036",
        x"0036", x"0036", x"0036", x"0036", x"0037", x"0037", x"0037", x"0037",
        x"0037", x"0037", x"0038", x"0037", x"0038", x"0038", x"0038", x"0038",
        x"0039", x"0038", x"0039", x"0039", x"0039", x"0039", x"0039", x"0039",
        x"003A", x"003A", x"003A", x"003A", x"003A", x"003A", x"003B", x"003B",
        x"003B", x"003B", x"003B", x"003B", x"003C", x"003B", x"003C", x"003C",
        x"003C", x"003D", x"003C", x"003D", x"003D", x"003D", x"003D", x"003D",
        x"003D", x"003E", x"003E", x"003E", x"003E", x"003E", x"003F", x"003E",
        x"003F", x"003F", x"003F", x"0040", x"003F", x"0040", x"0040", x"0040",
        x"0040", x"0040", x"0041", x"0041", x"0041", x"0041", x"0041", x"0041",
        x"0042", x"0042", x"0042", x"0042", x"0042", x"0043", x"0042", x"0043",
        x"0043", x"0043", x"0044", x"0043", x"0044", x"0044", x"0044", x"0045",
        x"0044", x"0045", x"0045", x"0045", x"0045", x"0045", x"0046", x"0046",
        x"0046", x"0046", x"0046", x"0047", x"0047", x"0047", x"0047", x"0047",
        x"0048", x"0048", x"0047", x"0049", x"0048", x"0048", x"0049", x"0049",
        x"0049", x"0049", x"004A", x"004A", x"004A", x"004A", x"004A", x"004A",
        x"004B", x"004B", x"004B", x"004C", x"004B", x"004C", x"004C", x"004C",
        x"004C", x"004D", x"004D", x"004C", x"004E", x"004D", x"004E", x"004D",
        x"004E", x"004F", x"004E", x"004F", x"004F", x"004F", x"004F", x"0050",
        x"004F", x"0050", x"0050", x"0051", x"0050", x"0051", x"0051", x"0051",
        x"0052", x"0052", x"0051", x"0053", x"0052", x"0053", x"0052", x"0053",
        x"0054", x"0053", x"0054", x"0054", x"0054", x"0054", x"0055", x"0055",
        x"0055", x"0055", x"0056", x"0055", x"0056", x"0057", x"0056", x"0057",
        x"0057", x"0057", x"0057", x"0058", x"0058", x"0058", x"0058", x"0059");
    constant C_EXP8 : t_exp8 := (
        to_unsigned(256,9), to_unsigned(259,9), to_unsigned(262,9), to_unsigned(264,9), to_unsigned(267,9), to_unsigned(270,9),
        to_unsigned(273,9), to_unsigned(276,9), to_unsigned(279,9), to_unsigned(282,9), to_unsigned(285,9), to_unsigned(288,9),
        to_unsigned(292,9), to_unsigned(295,9), to_unsigned(298,9), to_unsigned(301,9), to_unsigned(304,9), to_unsigned(308,9),
        to_unsigned(311,9), to_unsigned(314,9), to_unsigned(318,9), to_unsigned(321,9), to_unsigned(325,9), to_unsigned(328,9),
        to_unsigned(332,9), to_unsigned(336,9), to_unsigned(339,9), to_unsigned(343,9), to_unsigned(347,9), to_unsigned(350,9),
        to_unsigned(354,9), to_unsigned(358,9), to_unsigned(362,9), to_unsigned(366,9), to_unsigned(370,9), to_unsigned(374,9),
        to_unsigned(378,9), to_unsigned(382,9), to_unsigned(386,9), to_unsigned(391,9), to_unsigned(395,9), to_unsigned(399,9),
        to_unsigned(403,9), to_unsigned(408,9), to_unsigned(412,9), to_unsigned(417,9), to_unsigned(421,9), to_unsigned(426,9),
        to_unsigned(431,9), to_unsigned(435,9), to_unsigned(440,9), to_unsigned(445,9), to_unsigned(450,9), to_unsigned(454,9),
        to_unsigned(459,9), to_unsigned(464,9), to_unsigned(470,9), to_unsigned(475,9), to_unsigned(480,9), to_unsigned(485,9),
        to_unsigned(490,9), to_unsigned(496,9), to_unsigned(501,9), to_unsigned(506,9));
    constant C_EXP6 : t_exp6 := (
        to_unsigned(64,7), to_unsigned(65,7), to_unsigned(65,7), to_unsigned(66,7), to_unsigned(67,7), to_unsigned(68,7), to_unsigned(68,7), to_unsigned(69,7),
        to_unsigned(70,7), to_unsigned(71,7), to_unsigned(71,7), to_unsigned(72,7), to_unsigned(73,7), to_unsigned(74,7), to_unsigned(74,7), to_unsigned(75,7),
        to_unsigned(76,7), to_unsigned(77,7), to_unsigned(78,7), to_unsigned(79,7), to_unsigned(79,7), to_unsigned(80,7), to_unsigned(81,7), to_unsigned(82,7),
        to_unsigned(83,7), to_unsigned(84,7), to_unsigned(85,7), to_unsigned(86,7), to_unsigned(87,7), to_unsigned(88,7), to_unsigned(89,7), to_unsigned(90,7),
        to_unsigned(91,7), to_unsigned(91,7), to_unsigned(92,7), to_unsigned(93,7), to_unsigned(95,7), to_unsigned(96,7), to_unsigned(97,7), to_unsigned(98,7),
        to_unsigned(99,7), to_unsigned(100,7), to_unsigned(101,7), to_unsigned(102,7), to_unsigned(103,7), to_unsigned(104,7), to_unsigned(105,7), to_unsigned(106,7),
        to_unsigned(108,7), to_unsigned(109,7), to_unsigned(110,7), to_unsigned(111,7), to_unsigned(112,7), to_unsigned(114,7), to_unsigned(115,7), to_unsigned(116,7),
        to_unsigned(117,7), to_unsigned(119,7), to_unsigned(120,7), to_unsigned(121,7), to_unsigned(123,7), to_unsigned(124,7), to_unsigned(125,7), to_unsigned(127,7));
    constant C_HT : t_ht := (
        to_unsigned(32,6), to_unsigned(30,6), to_unsigned(28,6), to_unsigned(26,6), to_unsigned(25,6), to_unsigned(23,6),
        to_unsigned(22,6), to_unsigned(20,6), to_unsigned(19,6), to_unsigned(17,6), to_unsigned(16,6), to_unsigned(15,6),
        to_unsigned(14,6), to_unsigned(13,6), to_unsigned(12,6), to_unsigned(11,6), to_unsigned(10,6), to_unsigned(10,6),
        to_unsigned(9,6), to_unsigned(8,6), to_unsigned(8,6), to_unsigned(7,6), to_unsigned(6,6), to_unsigned(6,6),
        to_unsigned(5,6), to_unsigned(5,6), to_unsigned(5,6), to_unsigned(4,6), to_unsigned(4,6), to_unsigned(4,6),
        to_unsigned(3,6), to_unsigned(3,6), to_unsigned(3,6), to_unsigned(3,6), to_unsigned(2,6), to_unsigned(2,6),
        to_unsigned(2,6), to_unsigned(2,6), to_unsigned(2,6), to_unsigned(2,6), to_unsigned(1,6), to_unsigned(1,6),
        to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6),
        to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6), to_unsigned(1,6), to_unsigned(0,6),
        to_unsigned(0,6), to_unsigned(0,6), to_unsigned(0,6), to_unsigned(0,6), to_unsigned(0,6), to_unsigned(0,6),
        to_unsigned(0,6), to_unsigned(0,6), to_unsigned(0,6), to_unsigned(0,6));
    constant C_LOG2P : t_log2p := (
        to_signed(-320,10), to_signed(-219,10), to_signed(-171,10), to_signed(-140,10), to_signed(-117,10), to_signed(-99,10),
        to_signed(-83,10), to_signed(-70,10), to_signed(-58,10), to_signed(-48,10), to_signed(-39,10), to_signed(-30,10),
        to_signed(-23,10), to_signed(-16,10), to_signed(-9,10), to_signed(-3,10), to_signed(3,10), to_signed(8,10),
        to_signed(13,10), to_signed(18,10), to_signed(23,10), to_signed(27,10), to_signed(31,10), to_signed(35,10),
        to_signed(39,10), to_signed(43,10), to_signed(47,10), to_signed(50,10), to_signed(53,10), to_signed(56,10),
        to_signed(60,10), to_signed(63,10));
    constant C_LARM : t_larm := (
        to_signed(-512,10), to_signed(-170,10), to_signed(-106,10), to_signed(-68,10), to_signed(-42,10), to_signed(-21,10),
        to_signed(-4,10), to_signed(10,10), to_signed(22,10));
    constant C_RB0 : t_rom256 := (
        x"51F0", x"5DE7", x"69DE", x"75D5", x"81CC", x"8DC4", x"99BB", x"A5B2",
        x"B2A9", x"BEA0", x"CA98", x"D08E", x"C984", x"C379", x"BD6F", x"B764",
        x"B15A", x"AB4F", x"A545", x"9F3A", x"9830", x"9225", x"9221", x"941F",
        x"961D", x"991C", x"9B1A", x"9D18", x"A016", x"A215", x"A413", x"A711",
        x"A910", x"9D18", x"9121", x"852A", x"7933", x"6D3C", x"6144", x"554D",
        x"4956", x"3D5F", x"3168", x"2B71", x"317B", x"3786", x"3D90", x"439B",
        x"49A5", x"4FB0", x"56BA", x"5CC5", x"62CF", x"68DA", x"69DF", x"66E0",
        x"64E2", x"61E4", x"5FE5", x"5DE7", x"5AE9", x"58EB", x"56EC", x"53EE",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_RB1 : t_rom256 := (
        x"5A00", x"5300", x"4C00", x"4500", x"3E00", x"3700", x"3000", x"2900",
        x"2200", x"1B00", x"1400", x"1100", x"1400", x"1800", x"1B00", x"1F00",
        x"2300", x"2600", x"2A00", x"2D00", x"3100", x"3400", x"3C00", x"4700",
        x"5100", x"5C00", x"6600", x"7100", x"7B00", x"8600", x"9000", x"9B00",
        x"A500", x"AC00", x"B300", x"BA00", x"C100", x"C800", x"CF00", x"D600",
        x"DD00", x"E400", x"EB00", x"EE00", x"EB00", x"E700", x"E400", x"E000",
        x"DD00", x"D900", x"D600", x"D200", x"CF00", x"CB00", x"C300", x"B800",
        x"AE00", x"A300", x"9900", x"8E00", x"8400", x"7900", x"6F00", x"6400",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");


    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_cx_k, s_cy_k : signed(10 downto 0) := (others => '0');
    signal s_per_k  : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_warp_k : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_arm_k  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_duty_k : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_spd_k  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_flat_t : std_logic := '1';
    signal s_shape  : std_logic_vector(1 downto 0) := "00";   -- S8 & S7: 00 circle 01 square 10 flower 11 hexagon
    signal s_pal    : std_logic_vector(1 downto 0) := "00";   -- S10 & S9
    signal s_inv    : std_logic := '0';

    signal s_cx, s_cy   : signed(13 downto 0) := (others => '0');
    signal s_cxs, s_cys : signed(21 downto 0) := (others => '0');   -- glide accum Q8
    signal s_lfs   : signed(17 downto 0) := to_signed(-30000, 18);  -- log2 freq glide, Q12
    signal s_pgs   : unsigned(14 downto 0) := to_unsigned(64 * 64, 15);  -- warp p glide, Q12
    signal s_vgs   : signed(16 downto 0) := (others => '0');        -- speed glide, Q6
    signal s_lf    : signed(11 downto 0) := to_signed(-469, 12);    -- log2 cycles/px, Q6
    signal s_m     : unsigned(8 downto 0) := to_unsigned(256, 9);   -- freq mantissa Q8
    signal s_kc    : integer range -16 to 0 := -11;                 -- Li - 4 (phase shift base)
    signal s_p     : unsigned(6 downto 0) := to_unsigned(64, 7);    -- warp exponent Q6
    signal s_a0    : signed(11 downto 0) := (others => '0');        -- log2 radial grad base, Q6
    signal s_arms  : signed(4 downto 0) := (others => '0');
    signal s_armv  : std_logic := '0';                              -- arms /= 0
    signal s_larm  : signed(11 downto 0) := (others => '0');        -- log2 angular grad base
    signal s_D     : unsigned(17 downto 0) := to_unsigned(131072, 18);   -- duty, Q18
    signal s_anim  : unsigned(20 downto 0) := (others => '0');      -- travel phase, Q18
    signal s_animd : unsigned(20 downto 0) := (others => '0');      -- travel - D/2
    signal s_u, s_av : unsigned(9 downto 0) := (others => '0');
    signal s_vneg, s_vzero : std_logic := '1';
    signal s_ls    : unsigned(9 downto 0) := (others => '0');
    signal s_spd   : unsigned(16 downto 0) := (others => '0');      -- |speed| Q18 / frame
    signal s_ssh   : unsigned(3 downto 0) := (others => '0');
    -- sequencer intermediates (one op per state)
    signal s_tg, s_gd, s_gdd : signed(21 downto 0) := (others => '0');
    signal s_gs, s_gm : std_logic := '0';
    signal s_lt  : signed(17 downto 0) := (others => '0');
    signal s_ptg : unsigned(14 downto 0) := to_unsigned(64 * 64, 15);
    signal s_vtg : signed(16 downto 0) := (others => '0');

    -- one shared EXP8 ROM read for the sequencer
    signal s_e8a   : unsigned(5 downto 0) := (others => '0');
    signal s_e8q   : unsigned(8 downto 0) := to_unsigned(256, 9);

    -- vblank sequencer + shared multiplier (centre resolve)
    signal s_seq   : unsigned(5 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    -- serial shift-add multiplier (k * W), 12 cycles; vblank has thousands
    signal sm_go   : std_logic := '0';
    signal sm_k    : signed(10 downto 0) := (others => '0');
    signal sm_w    : unsigned(11 downto 0) := (others => '0');
    signal sm_a    : signed(23 downto 0) := (others => '0');
    signal sm_b    : unsigned(11 downto 0) := (others => '0');
    signal sm_acc  : signed(24 downto 0) := (others => '0');
    signal sm_n    : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- raster measurement + generated avid
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wl    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    -- resolution scaling: field-line step in x-pixel units (Q.8) and the
    -- mode class (00 1080 / 01 720p / 10 SD), decoded from the timing ID;
    -- s_lacc counts field lines in those units (its integer part = s_H)
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    -- log2(s) per mode: s_lref = LREF - log2(s) (Q14) turns the pixel log
    -- distance into the 1080-line one; s_ls6 = log2(s) (Q6) converts the
    -- radial gradient back to cycles per pixel
    signal s_lref  : signed(19 downto 0) := to_signed(C_LREF, 20);
    signal s_ls6   : unsigned(6 downto 0) := (others => '0');
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q, s_hs_r     : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc    : unsigned(3 downto 0) := "1000";
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');

    -- coordinate accumulators
    signal s_dx, s_dyc : signed(14 downto 0) := (others => '0');
    signal s_dyb      : signed(22 downto 0) := (others => '0');   -- next line's dy, Q.8
    signal s_gav_p    : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- EBR delay lines (one write, one registered read each)
    ----------------------------------------------------------------------
    type t_dl22 is array(0 to 255) of std_logic_vector(21 downto 0);
    type t_dl20 is array(0 to 255) of std_logic_vector(19 downto 0);
    type t_dl18 is array(0 to 255) of std_logic_vector(17 downto 0);
    type t_dl12 is array(0 to 255) of std_logic_vector(11 downto 0);
    type t_dl4  is array(0 to 255) of std_logic_vector(3 downto 0);
    signal dlh  : t_dl22;      -- log2 hi + quadrant flags
    signal dll  : t_dl20;      -- log2 d - LREF
    signal dlc  : t_dl18;      -- spiral term
    signal dld  : t_dl12;      -- angular-gradient log
    signal dls  : t_dl4;       -- syncs
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dlh_q : std_logic_vector(21 downto 0) := (others => '0');
    signal dll_q : std_logic_vector(19 downto 0) := (others => '0');
    signal dlc_q : std_logic_vector(17 downto 0) := (others => '0');
    signal dld_q : std_logic_vector(11 downto 0) := (others => '0');
    signal dls_q, dls_q2 : std_logic_vector(3 downto 0) := "0110";

    ----------------------------------------------------------------------
    -- pixel pipeline (stage N = N registers after s_dx)
    ----------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| -> hi/lo -> log2 hi, log2 lo (Q14, EBR PWL tables)
    signal s1_a, s1_b : unsigned(12 downto 0) := (others => '0');
    signal s1_dxs, s1_dys : std_logic := '0';
    signal s2_hi, s2_lo : unsigned(12 downto 0) := (others => '0');
    signal s2_sw, s2_dxs, s2_dys, s2_lz : std_logic := '0';
    signal s3_hi, s3_lo : unsigned(12 downto 0) := (others => '0');
    signal s3_mh, s3_ml : unsigned(3 downto 0) := (others => '0');
    signal s4_ih, s4_il : unsigned(7 downto 0) := (others => '0');
    signal s4_th, s4_tl, s5_th, s5_tl, s6_th, s6_tl : unsigned(3 downto 0) := (others => '0');
    signal s4_mh, s4_ml, s5_mh, s5_ml, s6_mh, s6_ml, s7_mh, s7_ml : unsigned(3 downto 0) := (others => '0');
    signal lvh_q, lsh_q, lvl_q, lsl_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s6_lvh, s6_lsh, s6_lvl, s6_lsl : std_logic_vector(15 downto 0) := (others => '0');
    signal s7_lph, s7_lpl : unsigned(10 downto 0) := (others => '0');
    signal s7_lvh, s7_lvl : unsigned(13 downto 0) := (others => '0');
    signal s8_lh, s8_ll : unsigned(17 downto 0) := (others => '0');
    -- quadrant info rides alongside (S2..S15)
    type t_q is array(3 to 8) of std_logic_vector(3 downto 0);    -- sw, dxs, dys, lz
    signal qsr : t_q := (others => (others => '0'));

    -- S9..S15: u = log2(hi/lo) -> g(u) = 0.5 log2(1+4^-u), atan(2^-u)
    signal s9_ua  : unsigned(17 downto 0) := (others => '0');
    signal s10_ia : unsigned(7 downto 0) := (others => '0');
    signal s10_tu, s11_tu, s12_tu : unsigned(6 downto 0) := (others => '0');
    signal ug_q, ugs_q, ua_q, uas_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_g, s12_gs, s12_a, s12_as : std_logic_vector(15 downto 0) := (others => '0');
    signal s13_gp  : signed(21 downto 0) := (others => '0');
    signal s13_ap  : signed(21 downto 0) := (others => '0');
    signal s13_g, s13_a : unsigned(15 downto 0) := (others => '0');
    signal s14_g   : unsigned(13 downto 0) := (others => '0');
    signal s14_a   : unsigned(15 downto 0) := (others => '0');
    signal s15_lr  : unsigned(17 downto 0) := (others => '0');   -- log2 r, Q14
    signal s15_ld  : unsigned(17 downto 0) := (others => '0');   -- log2 of the shape base
    signal s15_th  : unsigned(17 downto 0) := (others => '0');   -- polar angle, 2^18/turn

    -- S16..S22: shape table (hex / flower) + spiral term + angular gradient
    signal s16_t6  : unsigned(17 downto 0) := (others => '0');
    signal s16_ld  : signed(19 downto 0) := (others => '0');    -- log2 d - LREF
    signal s16_sp  : unsigned(17 downto 0) := (others => '0');
    signal s16_b   : signed(11 downto 0) := (others => '0');
    signal s17_sa  : unsigned(8 downto 0) := (others => '0');
    signal s17_st, s18_st, s19_st : unsigned(5 downto 0) := (others => '0');
    signal shv_q, shs_q, s19_shv, s19_shs : std_logic_vector(15 downto 0) := (others => '0');
    signal s20_sp  : signed(16 downto 0) := (others => '0');
    signal s20_v   : signed(15 downto 0) := (others => '0');
    signal s21_sv  : signed(15 downto 0) := (others => '0');

    -- S22..S25: warp
    signal s22_vlu : signed(20 downto 0) := (others => '0');
    signal s23_ph  : signed(25 downto 0) := (others => '0');
    signal s23_pl  : signed(24 downto 0) := (others => '0');
    signal s23_vlu : signed(20 downto 0) := (others => '0');
    signal s24_pe  : signed(21 downto 0) := (others => '0');
    signal s24_vlu : signed(20 downto 0) := (others => '0');
    signal s25_e   : signed(22 downto 0) := (others => '0');
    signal s25_ga  : signed(14 downto 0) := (others => '0');   -- (p-1)(log2 d - LREF), Q6
    signal s25_a1  : signed(11 downto 0) := (others => '0');

    -- S26..S33: exp2 mantissa (EBR PWL) -> * freq mantissa -> one shift = phase
    signal s26_xa  : unsigned(7 downto 0) := (others => '0');
    signal s26_t, s27_t, s28_t : unsigned(5 downto 0) := (others => '0');
    signal s26_k, s27_k, s28_k, s29_k, s30_k, s31_k, s32_k : integer range -17 to 7 := 0;
    signal s26_a   : signed(11 downto 0) := (others => '0');
    signal exm_q, exs_q, s28_m, s28_s : std_logic_vector(15 downto 0) := (others => '0');
    signal s29_ep  : unsigned(13 downto 0) := (others => '0');
    signal s29_m   : unsigned(15 downto 0) := (others => '0');
    signal s30_mt  : unsigned(15 downto 0) := (others => '0');
    signal s31_ph  : unsigned(16 downto 0) := (others => '0');
    signal s31_pl  : unsigned(16 downto 0) := (others => '0');
    signal s32_pr  : unsigned(24 downto 0) := (others => '0');
    signal s33_phr : unsigned(20 downto 0) := (others => '0');

    -- gradient combine (S27..S35):  Lg = max(a,b) + 0.5 log2(1+2^-2|a-b|)
    signal s27_dl  : signed(12 downto 0) := (others => '0');
    signal s27_mx  : signed(11 downto 0) := (others => '0');
    signal s28_ad  : unsigned(7 downto 0) := (others => '0');
    signal s28_mx  : signed(11 downto 0) := (others => '0');
    signal s29_h   : unsigned(5 downto 0) := (others => '0');
    signal s29_mx  : signed(11 downto 0) := (others => '0');
    signal s30_lg  : signed(11 downto 0) := (others => '0');
    signal s31_lg  : signed(11 downto 0) := (others => '0');
    signal s31_lwp : std_logic := '0';                                -- lg + 128 > 0
    signal s32_lw  : signed(11 downto 0) := (others => '0');
    signal s33_h   : std_logic := '0';                                -- half-octave step
    signal s33_iw  : integer range -16 to 0 := -16;
    signal s33_sat : std_logic := '0';
    signal s33_ri, s34_ri, s35_ri, s36_ri : integer range 0 to 15 := 0;
    signal s33_rh, s34_rh, s35_rh, s36_rh : std_logic := '0';
    signal s34_w   : unsigned(15 downto 0) := (others => '1');
    signal s34_q   : unsigned(20 downto 0) := (others => '0');
    signal s35_phi : unsigned(15 downto 0) := (others => '0');
    signal s35_wh  : unsigned(14 downto 0) := (others => '0');
    signal s_D16   : unsigned(15 downto 0) := to_unsigned(32768, 16);

    -- S36..S41: one-pixel box filter -> coverage 0..256
    signal s36_lo, s36_hi : unsigned(15 downto 0) := (others => '0');
    signal s37_k   : std_logic := '0';
    signal s37_full, s38_full, s39_full, s40_full : std_logic := '0';   -- window wholly inside the ink band
    signal s37_mh, s37_ml : unsigned(15 downto 0) := (others => '0');
    signal s37_ri, s38_ri : integer range 0 to 15 := 0;
    signal s37_rh, s38_rh, s39_rh : std_logic := '0';
    signal s38_n   : unsigned(15 downto 0) := (others => '0');
    signal s39_nsh : unsigned(9 downto 0) := (others => '0');
    signal s40_cp  : unsigned(10 downto 0) := (others => '0');
    signal s41_cov : unsigned(8 downto 0) := (others => '0');
    type t_hue is array(35 to 40) of unsigned(5 downto 0);
    signal huesr   : t_hue := (others => (others => '0'));
    signal rb0_q, rb1_q : std_logic_vector(15 downto 0) := (others => '0');

    -- S42..S45: palette + lerp + output
    signal s42_cov : unsigned(8 downto 0) := (others => '0');
    signal s42_dy  : signed(10 downto 0) := (others => '0');
    signal s42_ua, s42_va : unsigned(9 downto 0) := (others => '0');
    signal s43_py  : signed(20 downto 0) := (others => '0');
    signal s43_u, s43_v : unsigned(9 downto 0) := (others => '0');
    signal s44_y, s44_u, s44_v : unsigned(9 downto 0) := (others => '0');
    -- per-frame palette: paper Y/U/V, ink - paper luma, ink U/V
    signal s_pyb, s_pub, s_pvb, s_pua, s_pva : unsigned(9 downto 0) := (others => '0');
    signal s_pdy : signed(10 downto 0) := (others => '0');
    signal o_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_u, o_v : unsigned(9 downto 0) := C_MID;

    function f_sat(x : unsigned; w : integer) return unsigned is
    begin
        if x > to_unsigned(2**w - 1, x'length) then
            return to_unsigned(2**w - 1, w);
        end if;
        return resize(x, w);
    end function;

    -- Exact signed product a * m, radix-4 Booth on m: ceil(len(m)/2)
    -- partials of {0, +-a, +-2a} (negatives as one's complement, the +1s
    -- gathered in one correction word in a spare slot), balanced adds.
    -- Checked in Python against a*m for every operand width pair used here.
    function bmul(a : signed; m : signed) return signed is
        constant NA  : natural := a'length;
        constant NML : natural := m'length;
        constant NM  : natural := NML + (NML mod 2);
        constant NP  : natural := NA + NML;
        constant NPP : natural := NM / 2;
        type t_pp is array (0 to 7) of signed(NP - 1 downto 0);
        variable va   : signed(NA - 1 downto 0);
        variable vm   : signed(NM - 1 downto 0);
        variable me   : std_logic_vector(NM downto 0);
        variable a1, a2, mag, pp, corr : signed(NP - 1 downto 0);
        variable d    : std_logic_vector(2 downto 0);
        variable p    : t_pp;
    begin
        va := a;
        vm := resize(m, NM);
        me := std_logic_vector(vm) & '0';
        a1 := resize(va, NP);
        a2 := shift_left(a1, 1);
        corr := (others => '0');
        for i in 0 to 7 loop
            p(i) := (others => '0');
        end loop;
        for i in 0 to NPP - 1 loop
            d := me(2 * i + 2 downto 2 * i);
            case d is
                when "001" | "010" | "101" | "110" => mag := a1;
                when "011" | "100"                 => mag := a2;
                when others                        => mag := (others => '0');
            end case;
            if d(2) = '1' and d /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        p(NPP) := corr;
        return ((p(0) + p(1)) + (p(2) + p(3))) + ((p(4) + p(5)) + (p(6) + p(7)));
    end function;

    -- Exact unsigned product a * m (width len(a) + len(m)) on the signed Booth.
    function bmul_u(a : unsigned; m : unsigned) return unsigned is
        constant NP : natural := a'length + m'length;
        variable vp : signed(NP + 1 downto 0);
    begin
        vp := bmul(signed('0' & a), signed('0' & m));
        return unsigned(vp(NP - 1 downto 0));
    end function;



begin

    ------------------------------------------------------------------------
    -- raster measurement + GENERATED AVID.  The encoders get no data-enable,
    -- so they pair Cb/Cr by counting from hsync while the core's 4:2:2 packer
    -- restarts at every avid rise: an analog avid that wanders a clock swaps
    -- U/V for that line (thin coloured lines).  So the program makes its own
    -- avid: start 2 or 3 clocks after the source's first avid rise, choosing
    -- whichever keeps the hsync->start PARITY at the source's usual value (a
    -- saturating vote), and run a fixed field-latched width.  (cubist v0.3.3)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;

            -- per-mode steps (the timing ID is static while a mode runs).
            -- s_ystp = virtual 1080-line step per FIELD line / x step, i.e.
            -- the line pitch in x-pixel units (interlace doubles it).
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ystp <= to_unsigned(569, 10); s_mcls <= "10";  -- 480i
                when 4          => s_ystp <= to_unsigned(288, 10); s_mcls <= "10";  -- 480p
                when 8          => s_ystp <= to_unsigned(480, 10); s_mcls <= "10";  -- 576i
                when 12         => s_ystp <= to_unsigned(240, 10); s_mcls <= "10";  -- 576p
                when 5 | 6 | 14 => s_ystp <= to_unsigned(256, 10); s_mcls <= "01";  -- 720p
                when 1 | 2 | 10 => s_ystp <= to_unsigned(512, 10); s_mcls <= "00";  -- 1080i
                when others     => s_ystp <= to_unsigned(256, 10); s_mcls <= "00";  -- 1080p
            end case;
            -- log2(s): SD 1 octave, 720p log2(1.5) = 0.585 (Q14 9585, Q6 37)
            case s_mcls is
                when "10"   => s_lref <= to_signed(C_LREF - 16384, 20); s_ls6 <= to_unsigned(64, 7);
                when "01"   => s_lref <= to_signed(C_LREF - 9585, 20);  s_ls6 <= to_unsigned(37, 7);
                when others => s_lref <= to_signed(C_LREF, 20);         s_ls6 <= (others => '0');
            end case;

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                -- hysteresis: analog line lengths wander +/-1 px
                if s_xcnt > 16 and (s_xcnt > s_W + 2 or s_xcnt + 2 < s_W) then
                    s_W <= s_xcnt;
                end if;
                s_xcnt <= (others => '0');
                s_lacc <= s_lacc + s_ystp;
            end if;
            if s_vs_pulse = '1' then
                -- field height in x-pixel units (1080p: the line count)
                if s_lacc(19 downto 8) > 8 then
                    s_H <= s_lacc(19 downto 8);
                end if;
                s_lacc <= (others => '0');
            end if;

            if s_hs_r = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if s_avid_r = '1' and g_arm = '1' then
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
                if g_cnt + 1 >= s_Wl then g_avid <= '0'; end if;
                g_cnt <= g_cnt + 1;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.  The frame tick is the first
    -- vsync edge after active video (analog vsync serrates).
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_kx, v_ky : signed(10 downto 0);
        variable v_ct   : signed(24 downto 0);
        variable v_targ : signed(21 downto 0);
        variable v_gd   : signed(21 downto 0);
        variable v_wa   : integer;
        variable v_lt   : signed(17 downto 0);
        variable v_pt   : unsigned(14 downto 0);
        variable v_vt   : signed(16 downto 0);
        variable v_arm  : signed(10 downto 0);
        variable v_a    : integer range 0 to 8;
        variable v_v    : signed(10 downto 0);
        variable v_av   : unsigned(9 downto 0);
        variable v_pl   : unsigned(6 downto 0);
        variable v_sh   : integer;
        variable v_sp   : unsigned(24 downto 0);
        variable v_gc, v_gdl : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            sm_go <= '0';
            s_e8q <= C_EXP8(to_integer(s_e8a));
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';

                v_kx := signed('0' & registers_in(0)) - to_signed(512, 11);
                v_ky := signed('0' & registers_in(1)) - to_signed(512, 11);
                if abs(v_kx) < C_DB then v_kx := (others => '0'); end if;
                if abs(v_ky) < C_DB then v_ky := (others => '0'); end if;
                s_cx_k <= v_kx;
                s_cy_k <= v_ky;
                s_per_k  <= unsigned(registers_in(2));
                s_warp_k <= unsigned(registers_in(3));
                v_wa := to_integer(signed('0' & registers_in(3))) - 512;
                if v_wa < C_DB and v_wa > -C_DB then s_flat_t <= '1'; else s_flat_t <= '0'; end if;
                s_arm_k  <= unsigned(registers_in(4));
                s_duty_k <= unsigned(registers_in(5));
                s_spd_k  <= unsigned(registers_in(7));
                s_shape  <= registers_in(6)(1 downto 0);   -- S8 Six-fold, S7 Form
                s_pal    <= registers_in(6)(3 downto 2);   -- S10 Paper, S9 Ink
                s_inv    <= registers_in(6)(4);            -- S11 Invert
                s_Wl     <= s_W;

                s_seq <= to_unsigned(52, 6);

            elsif s_ssh /= 0 then                     -- speed = mantissa << octave, 1 bit per clock
                s_spd <= shift_left(s_spd, 1);
                s_ssh <= s_ssh - 1;
            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                if s_seq(5) = '1' then v_gc := s_cxs; else v_gc := s_cys; end if;   -- 35..37 vs 19..21
                case to_integer(s_seq) is

                    -- centre X: cx = W/2 + (K1-512)*W/256, glided
                    -- centre: target = W/2 + (K1-512)*W/256 (serial multiply), then
                    -- glide: ease, never slower than 1 px/frame, land exactly.
                    -- One op per state (vblank cones are timed at the pixel clock).
                    when 52 => sm_k <= s_cx_k; sm_w <= s_W; sm_go <= '1';
                    when 38 =>
                        s_tg <= shift_left(resize(signed('0' & s_W(11 downto 1)), 22), 8) + resize(sm_acc, 22);
                        sm_k <= s_cy_k; sm_w <= s_H; sm_go <= '1';
                    -- centre glides share one datapath: x at 37..35, y at 21..19
                    -- (snapping to the target is c + (tg - c) = tg exactly)
                    when 37 | 21 => s_gd <= s_tg - v_gc;
                    when 36 | 20 =>
                        if s_gd <= 256 and s_gd >= -256 then s_gs <= '1'; else s_gs <= '0'; end if;
                        if s_gd < 8192 and s_gd > -8192 then s_gm <= '1'; else s_gm <= '0'; end if;
                        s_gdd <= shift_right(s_gd, C_GL);
                    when 35 | 19 =>
                        if    s_gs = '1' then v_gdl := s_gd;
                        elsif s_gm = '1' then
                            if s_gd(21) = '0' then v_gdl := to_signed(256, 22); else v_gdl := to_signed(-256, 22); end if;
                        else  v_gdl := s_gdd; end if;
                        if s_seq(5) = '1' then s_cxs <= v_gc + v_gdl;
                        else                   s_cys <= v_gc + v_gdl; end if;
                    when 22 => s_tg <= shift_left(resize(signed('0' & s_H(11 downto 1)), 22), 8) + resize(sm_acc, 22);

                    -- PERIOD: log2 cycles/px = -11 + K3*9/1024 octaves (Q12 glide)
                    when 34 => s_lt <= to_signed(-45056, 18) + signed(resize(s_per_k * to_unsigned(36, 6), 18));
                    when 33 => s_lfs <= s_lfs + shift_right(s_lt - s_lfs, C_GL);
                    -- WARP exponent p = K4/512 (Q6), glided; detent = exactly flat
                    when 32 =>
                        if s_flat_t = '1' then s_ptg <= to_unsigned(64 * 64, 15);
                        elsif s_warp_k < 8 then s_ptg <= to_unsigned(64, 15);
                        else s_ptg <= shift_left(resize(shift_right(s_warp_k, 3), 15), 6); end if;
                    when 31 => s_pgs <= unsigned(signed(s_pgs) + shift_right(signed(s_ptg) - signed(s_pgs), C_GL));
                    -- SPEED (P12, bipolar) glide, Q6
                    when 30 => s_vtg <= shift_left(resize(signed('0' & s_spd_k) - 512, 17), 6);
                    when 29 => s_vgs <= s_vgs + shift_right(s_vtg - s_vgs, 3);

                    when 14 =>
                        s_lf <= resize(shift_right(s_lfs, 6), 12);
                        -- SPIRAL: integer arms round((K5-512)/64), -8..+8
                        v_arm := signed('0' & s_arm_k) - to_signed(512 - 32, 11);
                        s_arms <= resize(shift_right(v_arm, 6), 5);
                    when 13 =>
                        if s_arms /= 0 then s_armv <= '1'; else s_armv <= '0'; end if;
                        v_a := to_integer(abs(s_arms));
                        s_larm <= resize(C_LARM(v_a), 12);
                    -- freq = m/256 * 2^Li ; phase shift base kc = Li - 4
                    when 12 =>
                        s_e8a <= unsigned(s_lf(5 downto 0));
                        v_sh := to_integer(shift_right(s_lf, 6)) - 4;
                        if v_sh < -16 then v_sh := -16; elsif v_sh > 0 then v_sh := 0; end if;
                        s_kc <= v_sh;
                    when 11 =>
                        v_pl := resize(shift_right(s_pgs, 6), 7);
                        if v_pl = 0 then v_pl := to_unsigned(1, 7); end if;
                        s_p <= v_pl;
                    when 10 =>
                        s_m  <= s_e8q;
                        s_a0 <= s_lf + resize(C_LOG2P(to_integer(s_p(6 downto 2))), 12);

                    -- SPEED decode: |v| < 24 still; else 2^-12 .. 2^-2 cycle/frame
                    when 9 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        s_vneg <= v_v(10);
                        s_av <= unsigned(resize(abs(v_v), 10));
                    when 8 =>
                        -- RESOLUTION: radial gradient per pixel = per 1080-line
                        -- unit + log2(s)
                        s_a0 <= s_a0 + signed(resize(s_ls6, 12));
                        if s_av < 24 then s_vzero <= '1'; s_u <= (others => '0');
                        else              s_vzero <= '0'; s_u <= s_av - 24; end if;
                    when 7 =>
                        s_ls <= s_u + shift_right(s_u, 2) + shift_right(s_u, 4);
                    when 6 =>
                        s_e8a <= s_ls(5 downto 0);
                    when 4 =>
                        if s_vzero = '1' then s_spd <= (others => '0');
                        else                  s_spd <= resize(shift_right(s_e8q, 2), 17); end if;
                        s_ssh <= s_ls(9 downto 6);
                    when 3 =>
                        if s_vneg = '1' then s_anim <= s_anim - resize(s_spd, 21);
                        else                 s_anim <= s_anim + resize(s_spd, 21); end if;
                    when 5 =>
                        s_D <= shift_left(resize(to_unsigned(13, 9)
                               + resize(shift_right(s_duty_k(9 downto 2) * to_unsigned(230, 8), 8), 9), 18), 10);
                    when 2 =>
                        s_animd <= s_anim - resize(shift_right(s_D, 1), 21);
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);

                    -- PALETTE (ink over paper), BT.601 limited, U/V swapped
                    when 1 =>
                        case s_pal is
                            when "00" =>                              -- B/W
                                s_pyb <= to_unsigned(0, 10);   s_pub <= C_MID; s_pvb <= C_MID;
                                s_pdy <= to_signed(1023, 11);  s_pua <= C_MID; s_pva <= C_MID;
                            when "01" =>                              -- rainbow ink (per pixel) on black
                                s_pyb <= to_unsigned(0, 10);   s_pub <= C_MID; s_pvb <= C_MID;
                            when "10" =>                              -- duo: white on blue
                                s_pyb <= to_unsigned(164, 10); s_pub <= to_unsigned(439, 10); s_pvb <= to_unsigned(960, 10);
                                s_pdy <= to_signed(940 - 164, 11); s_pua <= C_MID; s_pva <= C_MID;
                            when others =>                            -- complementary: red on turquoise
                                s_pyb <= to_unsigned(663, 10); s_pub <= to_unsigned(235, 10); s_pvb <= to_unsigned(579, 10);
                                s_pdy <= to_signed(326 - 663, 11); s_pua <= to_unsigned(960, 10); s_pva <= to_unsigned(361, 10);
                        end case;

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    p_smul : process(clk)
    begin
        if rising_edge(clk) then
            if sm_go = '1' then
                sm_a   <= resize(sm_k, 24);
                sm_b   <= sm_w;
                sm_acc <= (others => '0');
                sm_n   <= to_unsigned(12, 4);
            elsif sm_n /= 0 then
                if sm_b(0) = '1' then sm_acc <= sm_acc + resize(sm_a, 25); end if;
                sm_a <= shift_left(sm_a, 1);
                sm_b <= shift_right(sm_b, 1);
                sm_n <= sm_n - 1;
            end if;
        end if;
    end process p_smul;

    ------------------------------------------------------------------------
    -- coordinate accumulators on the GENERATED avid.  dx loads -cx at each
    -- line start and steps per pixel; dy seeds at the first line of the frame
    -- and steps per line by s_ystp (Q.8 x-pixel units: 1080p exactly 1, so
    -- the integer part is the old counter; others rounded by the +0.5 seed).
    -- (field_n is tied high on hardware, so there is no bottom-field seed.)
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            s_gav_p <= g_avid;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif g_avid = '1' and s_gav_p = '0' then
                if s_newframe = '1' then
                    v_dy0 := shift_left(resize(-s_cy, 23), 8);
                    v_dy0(7) := '1';                                   -- +0.5
                    s_dyc <= v_dy0(22 downto 8);
                    s_dyb <= v_dy0 + signed(resize(s_ystp, 23));
                    s_newframe <= '0';
                else
                    s_dyc <= s_dyb(22 downto 8);
                    s_dyb <= s_dyb + signed(resize(s_ystp, 23));
                end if;
                s_dx <= resize(-s_cx, 15);
            elsif g_avid = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- EBR delay lines.  Write address = free-running counter; each read runs
    -- X behind it, then a fabric register (EBR clk-to-q never feeds logic).
    ------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dlh(to_integer(s_wa)) <= qsr(8) & std_logic_vector(s8_lh);
            dlh_q  <= dlh(to_integer(s_wa - C_XH));
            dll(to_integer(s_wa)) <= std_logic_vector(s16_ld);
            dll_q  <= dll(to_integer(s_wa - C_XL));
            dlc(to_integer(s_wa)) <= std_logic_vector(s16_sp);
            dlc_q  <= dlc(to_integer(s_wa - C_XC));
            dld(to_integer(s_wa)) <= std_logic_vector(s16_b);
            dld_q  <= dld(to_integer(s_wa - C_XD));
            dls(to_integer(s_wa)) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            dls_q  <= dls(to_integer(s_wa - (ST_OUT - 1)));
            dls_q2 <= dls_q;
        end if;
    end process p_dl;

    -- table ROMs (EBR): every address is a register, every read is registered
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lvh_q <= C_LOGV(to_integer(s4_ih));
            lsh_q <= C_LOGS(to_integer(s4_ih));
            lvl_q <= C_LOGV(to_integer(s4_il));
            lsl_q <= C_LOGS(to_integer(s4_il));
            ug_q  <= C_UG(to_integer(s10_ia));
            ugs_q <= C_UGS(to_integer(s10_ia));
            ua_q  <= C_UA(to_integer(s10_ia));
            uas_q <= C_UAS(to_integer(s10_ia));
            shv_q <= C_SHV(to_integer(s17_sa));
            shs_q <= C_SHS(to_integer(s17_sa));
            exm_q <= C_EXM(to_integer(s26_xa));
            exs_q <= C_EXS(to_integer(s26_xa));
            rb0_q <= C_RB0(to_integer(huesr(40)));
            rb1_q <= C_RB1(to_integer(huesr(40)));
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| (13-bit), hi = max, lo = min, and their log2 (Q14):
    -- priority-encode, normalise, 256-segment PWL from EBR.
    ------------------------------------------------------------------------
    p_logs : process(clk)
        variable va, vb : unsigned(14 downto 0);
        variable vh, vl : unsigned(12 downto 0);
        variable mh, ml : integer range 0 to 12;
    begin
        if rising_edge(clk) then
            -- S1
            va := unsigned(abs(s_dx));
            vb := unsigned(abs(s_dyc));
            s1_a <= f_sat(va, 13);
            s1_b <= f_sat(vb, 13);
            s1_dxs <= s_dx(14);
            s1_dys <= s_dyc(14);
            -- S2
            if s1_a >= s1_b then s2_hi <= s1_a; s2_lo <= s1_b; s2_sw <= '0';
            else                 s2_hi <= s1_b; s2_lo <= s1_a; s2_sw <= '1'; end if;
            if s1_a = 0 or s1_b = 0 then s2_lz <= '1'; else s2_lz <= '0'; end if;
            s2_dxs <= s1_dxs;
            s2_dys <= s1_dys;
            -- S3: msb (hi >= 1 always: the exact centre reads as 1)
            vh := s2_hi; if vh = 0 then vh := to_unsigned(1, 13); end if;
            vl := s2_lo; if vl = 0 then vl := to_unsigned(1, 13); end if;
            mh := 0; ml := 0;
            for i in 0 to 12 loop
                if vh(i) = '1' then mh := i; end if;
                if vl(i) = '1' then ml := i; end if;
            end loop;
            s3_hi <= vh; s3_lo <= vl;
            s3_mh <= to_unsigned(mh, 4); s3_ml <= to_unsigned(ml, 4);
            -- S4: normalise -> 8-bit segment index + 4-bit fraction
            vh := shift_left(s3_hi, 12 - to_integer(s3_mh));
            vl := shift_left(s3_lo, 12 - to_integer(s3_ml));
            s4_ih <= vh(11 downto 4); s4_th <= vh(3 downto 0);
            s4_il <= vl(11 downto 4); s4_tl <= vl(3 downto 0);
            s4_mh <= s3_mh; s4_ml <= s3_ml;
            -- S5, S6 (EBR read, fabric register)
            s5_th <= s4_th; s5_tl <= s4_tl; s5_mh <= s4_mh; s5_ml <= s4_ml;
            s6_th <= s5_th; s6_tl <= s5_tl; s6_mh <= s5_mh; s6_ml <= s5_ml;
            s6_lvh <= lvh_q; s6_lsh <= lsh_q; s6_lvl <= lvl_q; s6_lsl <= lsl_q;
            -- S7
            s7_lph <= resize(unsigned(s6_lsh(6 downto 0)) * s6_th, 11);
            s7_lpl <= resize(unsigned(s6_lsl(6 downto 0)) * s6_tl, 11);
            s7_lvh <= unsigned(s6_lvh(13 downto 0));
            s7_lvl <= unsigned(s6_lvl(13 downto 0));
            s7_mh <= s6_mh; s7_ml <= s6_ml;
            -- S8: log2 = msb + PWL(mantissa), Q14
            s8_lh <= shift_left(resize(s7_mh, 18), 14) + resize(s7_lvh, 18) + resize(shift_right(s7_lph, 4), 18);
            s8_ll <= shift_left(resize(s7_ml, 18), 14) + resize(s7_lvl, 18) + resize(shift_right(s7_lpl, 4), 18);

            -- quadrant flags ride along
            qsr(3) <= s2_sw & s2_dxs & s2_dys & s2_lz;
            for i in 4 to 8 loop qsr(i) <= qsr(i - 1); end loop;
        end if;
    end process p_logs;

    ------------------------------------------------------------------------
    -- S9..S15: u = log2(hi/lo) >= 0 ;  log2 r = log2 hi + 0.5 log2(1 + 4^-u)
    -- and the octant angle atan(2^-u) -- both from EBR tables (1/16 octave
    -- steps, interpolated).  No CORDIC: this is exact to ~1e-5 and cheaper.
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu   : unsigned(17 downto 0);
        variable vmag : unsigned(17 downto 0);
        variable q    : std_logic_vector(3 downto 0);
        variable vlh  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S9
            vu := s8_lh - s8_ll;
            -- lo = 0 (on an axis): u -> the table end (g ~ 0, angle ~ 0).
            -- (hi/lo <= 2^13, so u never reaches 16 octaves otherwise.)
            if qsr(8)(0) = '1' then vu := to_unsigned(16 * 16384 - 1, 18); end if;
            s9_ua <= vu;
            -- S10: 1/16-octave segment + 10-bit fraction
            s10_ia <= s9_ua(17 downto 10);
            s10_tu <= s9_ua(9 downto 3);
            -- S11, S12 (EBR read, fabric register)
            s11_tu <= s10_tu;
            s12_tu <= s11_tu;
            s12_g <= ug_q; s12_gs <= ugs_q; s12_a <= ua_q; s12_as <= uas_q;
            -- S13: interpolation products (slopes are negative)
            s13_gp <= resize(bmul(signed(s12_gs(10 downto 0)), signed(resize(s12_tu, 8))), 22);
            s13_ap <= resize(bmul(signed(s12_as(10 downto 0)), signed(resize(s12_tu, 8))), 22);
            s13_g  <= unsigned(s12_g);
            s13_a  <= unsigned(s12_a);
            -- S14
            s14_g <= resize(unsigned(signed(resize(s13_g, 17)) + resize(shift_right(s13_gp, 7), 17)), 14);
            s14_a <= resize(unsigned(signed(resize(s13_a, 17)) + resize(shift_right(s13_ap, 7), 17)), 16);
            -- S15: log2 r ; shape base (square = log2 max(|x|,|y|), exact) ;
            -- full polar angle from the octant angle + swap + signs
            vlh := unsigned(dlh_q(17 downto 0));
            s15_lr <= vlh + resize(s14_g, 18);
            if s_shape = "01" then s15_ld <= vlh;
            else                   s15_ld <= vlh + resize(s14_g, 18); end if;
            q := dlh_q(21 downto 18);
            if q(3) = '1' then vmag := to_unsigned(65536, 18) - resize(s14_a, 18);
            else               vmag := resize(s14_a, 18); end if;
            if    q(2) = '0' and q(1) = '0' then s15_th <= vmag;
            elsif q(2) = '1' and q(1) = '0' then s15_th <= to_unsigned(131072, 18) - vmag;
            elsif q(2) = '1' and q(1) = '1' then s15_th <= to_unsigned(131072, 18) + vmag;
            else                                 s15_th <= to_unsigned(0, 18) - vmag; end if;
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S16..S21: shape (hexagon / flower = r * S(theta), applied as + log2 S
    -- from an EBR table on the 6-fold angle), spiral term, angular gradient.
    ------------------------------------------------------------------------
    p_shape : process(clk)
        variable vsp : signed(23 downto 0);
        variable vv  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S16
            s16_t6 <= shift_left(s15_th, 2) + shift_left(s15_th, 1);          -- 6 theta mod turn
            -- log2 d - LREF in 1080-line units: s_lref = LREF - log2(s)
            s16_ld <= signed(resize(s15_ld, 20)) - s_lref;
            vsp := bmul(signed(resize(s15_th, 19)), s_arms);
            s16_sp <= unsigned(vsp(17 downto 0));                              -- arms*theta mod 1 cycle
            -- angular gradient log2(arms/(2 pi r)) = larm - log2 r  (Q6)
            s16_b <= s_larm - signed(resize(shift_right(s15_lr, 8), 12));
            -- S17: fold 6 theta to [0, pi] ; table half = flower
            if s16_t6(17) = '1' then vv := to_unsigned(0, 18) - s16_t6; else vv := s16_t6; end if;
            if vv(17) = '1' then vv := to_unsigned(131071, 18); end if;
            if s_shape = "11" then s17_sa <= '0' & vv(16 downto 9);     -- hexagon half
            else                   s17_sa <= '1' & vv(16 downto 9); end if;  -- flower half
            s17_st <= vv(8 downto 3);
            -- S18, S19 (EBR read, fabric register)
            s18_st <= s17_st;
            s19_st <= s18_st;
            s19_shv <= shv_q; s19_shs <= shs_q;
            -- S20
            s20_sp <= resize(bmul(signed(s19_shs(15 downto 8)), signed(resize(s19_st, 7))), 17);
            s20_v  <= signed(s19_shv);
            -- S21
            s21_sv <= s20_v + resize(shift_right(s20_sp, 6), 16);
        end if;
    end process p_shape;

    ------------------------------------------------------------------------
    -- S22..S25: warp  pe = p * (log2 d - LREF)  ;  radial gradient term
    ------------------------------------------------------------------------
    p_warp : process(clk)
    begin
        if rising_edge(clk) then
            -- S22
            if s_shape(1) = '1' then s22_vlu <= resize(signed(dll_q), 21) + resize(s21_sv, 21);
            else                     s22_vlu <= resize(signed(dll_q), 21); end if;
            -- S23
            s23_ph  <= bmul(s22_vlu, signed(resize(s_p(6 downto 3), 5)));
            s23_pl  <= bmul(s22_vlu, signed(resize(s_p(2 downto 0), 4)));
            s23_vlu <= s22_vlu;
            -- S24
            s24_pe  <= resize(shift_right(shift_left(resize(s23_ph, 28), 3) + resize(s23_pl, 28), 6), 22);
            s24_vlu <= s23_vlu;
            -- S25: exponent ; (p-1)*(log2 d - LREF) for the gradient
            s25_e  <= resize(s24_pe, 23) + C_LREF;
            s25_ga <= resize(shift_right(s24_pe, 8), 15) - resize(shift_right(s24_vlu, 8), 15);
            s25_a1 <= s_a0;
        end if;
    end process p_warp;

    ------------------------------------------------------------------------
    -- S26..S33: r' = 2^e (mantissa from EBR PWL) ; phase = r' * f folded into
    -- ONE multiply (mantissa x freq mantissa) and ONE shift by (Ie + Li - 4).
    ------------------------------------------------------------------------
    p_exp : process(clk)
        variable vk : integer;
        variable vp : unsigned(31 downto 0);
        variable vw : unsigned(41 downto 0);
    begin
        if rising_edge(clk) then
            -- S26
            s26_xa <= unsigned(s25_e(13 downto 6));
            s26_t  <= unsigned(s25_e(5 downto 0));
            vk := to_integer(shift_right(s25_e, 14)) + s_kc;
            if vk < -17 then vk := -17; elsif vk > 7 then vk := 7; end if;
            s26_k <= vk;
            s26_a <= s25_a1 + resize(s25_ga, 12);
            -- S27, S28 (EBR read, fabric register)
            s27_t <= s26_t; s27_k <= s26_k;
            s28_t <= s27_t; s28_k <= s27_k;
            s28_m <= exm_q; s28_s <= exs_q;
            -- S29
            s29_ep <= resize(unsigned(s28_s(7 downto 0)) * s28_t, 14);
            s29_m  <= unsigned(s28_m);
            s29_k  <= s28_k;
            -- S30
            s30_mt <= s29_m + resize(shift_right(s29_ep, 6), 16);
            s30_k  <= s29_k;
            -- S31: mantissa (Q14) x freq mantissa (Q8), split on the wide operand
            s31_ph <= bmul_u(s30_mt(15 downto 8), s_m);
            s31_pl <= bmul_u(s30_mt(7 downto 0), s_m);
            s31_k  <= s30_k;
            -- S32
            s32_pr <= shift_left(resize(s31_ph, 25), 8) + resize(s31_pl, 25);
            s32_k  <= s31_k;
            -- S33: phase (Q18, 3 integer bits) = product * 2^k
            -- one left shift by k+17 and a 21-bit window (= << k for k >= 0,
            -- >> -k for k < 0); k = -17 (the clamp) still gives zero
            vw := shift_left(resize(s32_pr, 42), s32_k + 17);
            if s32_k <= -17 then
                s33_phr <= (others => '0');
            else
                s33_phr <= vw(37 downto 17);
            end if;
        end if;
    end process p_exp;

    ------------------------------------------------------------------------
    -- gradient combine:  a = log2|radial grad| = Lf + log2 p + (p-1)(log2 d -
    -- LREF) [+ flower slope] ;  b = log2|angular grad| (delay line D) ;
    -- Lg = max + 0.5 log2(1 + 2^-2|a-b|).  Filter width w = 2^Lg cycles/px,
    -- widened above 1/4 cycle/px so it reaches a full cycle (duty grey) at
    -- the Nyquist limit.
    ------------------------------------------------------------------------
    p_grad : process(clk)
        variable vb  : signed(11 downto 0);
        variable vlg : signed(11 downto 0);
        variable vlw : signed(11 downto 0);
        variable viw : integer;
        variable vsh : integer;
        variable vw  : unsigned(15 downto 0);
        variable vm  : unsigned(6 downto 0);
        variable vrr : signed(12 downto 0);
        variable vph : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S27
            vb := signed(dld_q(11 downto 0));
            s27_dl <= resize(s26_a, 13) - resize(vb, 13);
            if s_armv = '1' and vb > s26_a then s27_mx <= vb; else s27_mx <= s26_a; end if;
            -- S28
            if s_armv = '0' or abs(s27_dl) > 255 then s28_ad <= (others => '1');
            else s28_ad <= unsigned(resize(abs(s27_dl), 8)); end if;
            s28_mx <= s27_mx;
            -- S29
            s29_h  <= C_HT(to_integer(s28_ad(7 downto 2)));
            s29_mx <= s28_mx;
            -- S30
            vlg := s29_mx + signed(resize(s29_h, 12));
            if vlg > 255 then vlg := to_signed(255, 12); end if;
            s30_lg <= vlg;
            -- S31
            -- (lg + (lg + 128) is formed as 2 lg + 128: written as a sum of two
            -- registers, yosys merges their identical low 7 bits and the adder
            -- gets the SAME net on both carry inputs, which livelocks router1)
            s31_lg  <= s30_lg;
            vlw := s30_lg + 128;
            if vlw > 0 then s31_lwp <= '1'; else s31_lwp <= '0'; end if;
            -- S32
            if s31_lwp = '1' then s32_lw <= shift_left(s31_lg, 1) + 128; else s32_lw <= s31_lg; end if;
            -- S33: clamp to 16 octaves (rings wider than 2^16 px need no edges)
            vlw := s32_lw;
            if vlw < -1023 then vlw := to_signed(-1023, 12); end if;
            -- quantise UP to a half octave: w in {1, sqrt2} * 2^Iw (edges 1..1.4 px)
            vlw := vlw + 31;
            vlw(4 downto 0) := (others => '0');
            if vlw >= 0 then s33_sat <= '1'; else s33_sat <= '0'; end if;
            s33_h <= vlw(5);
            viw := to_integer(shift_right(vlw, 6));
            if viw > 0 then viw := 0; end if;
            s33_iw <= viw;
            -- reciprocal 2^-log2 w = 2^Ri * {1, sqrt2}
            if vlw >= 0 then vrr := (others => '0'); else vrr := -resize(vlw, 13); end if;
            s33_ri <= to_integer(shift_right(vrr, 6)) mod 16;
            s33_rh <= vrr(5);
            -- S34: filter width w (Q16) ; travel
            if s33_sat = '1' then
                vw := (others => '1');
            else
                vsh := s33_iw + 10;                       -- w (Q16) = {64, 91} * 2^(Iw+10)
                if s33_h = '1' then vm := to_unsigned(91, 7); else vm := to_unsigned(64, 7); end if;
                if vsh >= 0 then vw := resize(shift_left(resize(vm, 16), vsh), 16);
                else             vw := resize(shift_right(resize(vm, 16), -vsh), 16); end if;
                if vw = 0 then vw := to_unsigned(1, 16); end if;
            end if;
            s34_w  <= vw;
            s34_ri <= s33_ri; s34_rh <= s33_rh;
            s34_q  <= s33_phr - s_animd;
            -- S35: + spiral (whole cycles per turn: seamless)
            vph := s34_q(17 downto 0) + unsigned(dlc_q);
            s35_phi <= vph(17 downto 2);
            s35_wh  <= s34_w(15 downto 1);
            s_D16   <= s_D(17 downto 2);
            s35_ri  <= s34_ri; s35_rh <= s34_rh;
            huesr(35) <= s34_q(20 downto 15);
            for i in 36 to 40 loop huesr(i) <= huesr(i - 1); end loop;
        end if;
    end process p_grad;

    ------------------------------------------------------------------------
    -- S36..S41: one-pixel box filter of the band square-wave.
    --   band A = frac in [0,D) ; P(x) = floor(x)*D + min(frac x, D)
    --   coverage = (P(phi+w/2) - P(phi-w/2)) / w   (divide = * 2^-log2 w)
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vr  : signed(12 downto 0);
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S36
            s36_lo <= s35_phi - resize(s35_wh, 16);
            s36_hi <= s35_phi + resize(s35_wh, 16);
            s36_ri <= s35_ri; s36_rh <= s35_rh;
            -- S37
            if s36_hi < s36_lo then s37_k <= '1'; else s37_k <= '0'; end if;
            if s36_hi >= s36_lo and s36_hi < s_D16 then s37_full <= '1'; else s37_full <= '0'; end if;
            if s36_hi < s_D16 then s37_mh <= s36_hi; else s37_mh <= s_D16; end if;
            if s36_lo < s_D16 then s37_ml <= s36_lo; else s37_ml <= s_D16; end if;
            s37_ri <= s36_ri;
            s37_rh <= s36_rh;
            -- S38
            vn := signed(resize(s37_mh, 18)) - signed(resize(s37_ml, 18));
            if s37_k = '1' then vn := vn + signed(resize(s_D16, 18)); end if;
            if vn < 0 then vn := (others => '0'); end if;
            s38_n   <= unsigned(vn(15 downto 0));
            s38_ri  <= s37_ri;
            s38_full <= s37_full;
            s38_rh  <= s37_rh;
            -- S39: (N << Ri) >> 7  (N <= w, so N << Ri < 2^17: a slice, no clamp)
            vs := shift_left(resize(s38_n, 25), s38_ri);
            s39_nsh <= vs(16 downto 7);
            s39_rh  <= s38_rh;
            s39_full <= s38_full;
            -- S40
            -- / 2, or * sqrt2/2 = .70703 (shift-adds) on a half-octave step
            if s39_rh = '1' then
                s40_cp <= resize(shift_right(s39_nsh, 1) + shift_right(s39_nsh, 3)
                                 + shift_right(s39_nsh, 4) + shift_right(s39_nsh, 6), 11);
            else
                s40_cp <= resize(shift_right(s39_nsh, 1), 11);
            end if;
            s40_full <= s39_full;
            -- S41: coverage 0..256 ; INVERT swaps ink and paper
            vc := resize(s40_cp, 10);
            if vc > 256 or s40_full = '1' then vc := to_unsigned(256, 10); end if;
            if s_inv = '1' then s41_cov <= resize(256 - vc, 9);
            else                s41_cov <= resize(vc, 9); end if;
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- S42..S46: palette (ink over paper) + lerp.  Luma lerps at full
    -- coverage; chroma switches at the band edge (4:2:2 chroma is half-res).
    -- BT.601 limited, U/V SWAPPED for the hardware.
    --   S9 Ink  : White / Colour      S10 Paper: Black / Colour
    --   white/black = B/W ; colour/black = Rainbow ; white/colour = Duo
    --   (white on blue) ; colour/colour = Complementary (red on turquoise)
    ------------------------------------------------------------------------
    p_col : process(clk)
        variable vy : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- S42: ink (rainbow per pixel, else per frame)
            s42_cov <= s41_cov;
            if s_pal = "01" then
                s42_dy <= signed(resize(unsigned(rb0_q(15 downto 8)) & "00", 11));
                s42_ua <= unsigned(rb0_q(7 downto 0)) & "00";
                s42_va <= unsigned(rb1_q(15 downto 8)) & "00";
            else
                s42_dy <= s_pdy; s42_ua <= s_pua; s42_va <= s_pva;
            end if;
            -- S43: luma lerp product ; chroma switches at the band edge
            s43_py <= bmul(s42_dy, signed(resize(s42_cov, 10)));
            if s42_cov(8 downto 7) /= "00" then s43_u <= s42_ua; s43_v <= s42_va;
            else                                s43_u <= s_pub;  s43_v <= s_pvb; end if;
            -- S44
            vy := signed(resize(s_pyb, 11)) + resize(shift_right(s43_py, 8), 11);
            s44_y <= unsigned(vy(9 downto 0));
            s44_u <= s43_u;
            s44_v <= s43_v;
            -- S45: output, gated to neutral outside the (generated, aligned) avid
            if dls_q(3) = '0' then
                o_y <= to_unsigned(64, 10); o_u <= C_MID; o_v <= C_MID;
            else
                o_y <= s44_y; o_u <= s44_u; o_v <= s44_v;
            end if;
        end if;
    end process p_col;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= dls_q2(3);
    data_out.hsync_n <= dls_q2(2);
    data_out.vsync_n <= dls_q2(1);
    data_out.field_n <= dls_q2(0);

end architecture hypnos;
