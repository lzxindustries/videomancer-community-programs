-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Screenprint: catalogue combo #77 (seed 20260922) -- halftone dot screens +
-- multi-source mixing (add, screen, multiply, difference, hard light,
-- overlay).  v0.2.0 = finalizing pass (see SUMMARY.md for the review and
-- what changed from v0.1.1).
-- v0.2.4 (2026-10-17) = resolution scaling: the screen cell is in virtual
-- 1080-line units (per-pixel steps x 1 / 1.5 / 2, per-line steps x the
-- field-line step from the timing ID), so 720p and SD show the 1080p screen
-- scaled to the screen height with the same angles (SD = a 1440-wide 4:3
-- window); the Register offset is 60 virtual units at SD too.  On SD the
-- cell is floored at 2 field lines (P12's bottom ~20% at 480i), where a
-- smaller lattice only beats against the line pitch.  1080p is
-- bit-identical.
--
--   K1 Angle    screen angle 0..90 degrees, glided (Spin on: spin speed)
--   K2 Mix      12..100% of the blend
--   K3 Dot      Circle / Diamond / Square / Line
--   K4 Blend    Add / Screen / Multiply / Difference / Hard Light / Overlay
--               (Add is signed: ink darkens, paper lightens)
--   K5 Ink      Black / Red / Orange / Yellow / Green / Cyan / Blue / Magenta
--               (the ink's own luma goes into the blend)
--   K6 Gain     halftone contrast: how far the dots open with brightness
--
--   S7  Register  Tight / Loose: the colour layer printed 60 virtual units
--                 right (W/32 px on 16:9; ~W/24 on SD)
--                 (the left edge holds the line's own edge colour, no seam)
--   S8  Invert    the halftone inverted (ink where the paper was)
--   S9  Spin      the screen turns about the centre; K1 sets the speed
--   S10 Colour    Picture: picture colour, coloured ink tints its dots /
--                 Ink: a one-colour print (ink colour on neutral paper)
--   S11 Rosette   a second screen 30 degrees round (ink where either inks)
--   P12 Size      screen cell 4..160 px at 1080 (virtual units: it scales
--                 with the screen height), exponential, zoomed about the
--                 picture centre; glided
--
-- Screen = accumulator-pair projection per screen: u += cos/cell per pixel,
-- v -= sin/cell per pixel, the row starts advance by sin/cell and cos/cell
-- per active line, all mod one cell (20-bit, the top 8 bits are the
-- position in the cell).  A vblank sequencer (sine ROM, cell ROM, one
-- serial multiplier) sets the steps and centres the lattice on the
-- measured picture centre.  Dot distance per shape in cell units, compared
-- with a gain-scaled luma; two 10x10 products give every blend mode.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (21) in all modes;
-- output conditioning ([1 2 1] on Y/U/V + luma scaled to white ~777: the
-- HD HDMI bar fix from damier v0.4.4 -- halftone dots are full-swing pixel
-- edges that move with the picture); per-field glide + hysteresis on the
-- geometry knobs; labelled rotaries honour the firmware's label-INDEX
-- default until the knob first reads a position; vsync work on the edge,
-- once per field (saw-active guard); no_rw_check on the colour ring (it
-- never reads the address it writes).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture screenprint of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (21 + 3 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 21;

    type t_rom is array (0 to 1023) of unsigned(15 downto 0);
    type t_ink is array (0 to 7) of unsigned(9 downto 0);
    -- GEN BEGIN (gen_tables.py)
    -- quarter-wave sine: 16384 * sin((i + 0.5) / 1024 * pi / 2)
    constant C_SIN : t_rom := (
        x"000D", x"0026", x"003F", x"0058", x"0071", x"008A", x"00A3", x"00BC", x"00D6", x"00EF", x"0108", x"0121",
        x"013A", x"0153", x"016C", x"0186", x"019F", x"01B8", x"01D1", x"01EA", x"0203", x"021C", x"0235", x"024E",
        x"0268", x"0281", x"029A", x"02B3", x"02CC", x"02E5", x"02FE", x"0317", x"0330", x"034A", x"0363", x"037C",
        x"0395", x"03AE", x"03C7", x"03E0", x"03F9", x"0412", x"042B", x"0444", x"045E", x"0477", x"0490", x"04A9",
        x"04C2", x"04DB", x"04F4", x"050D", x"0526", x"053F", x"0558", x"0571", x"058A", x"05A3", x"05BC", x"05D5",
        x"05EE", x"0607", x"0620", x"0639", x"0652", x"066B", x"0684", x"069D", x"06B6", x"06CF", x"06E8", x"0701",
        x"071A", x"0733", x"074C", x"0765", x"077E", x"0797", x"07B0", x"07C9", x"07E2", x"07FB", x"0814", x"082D",
        x"0846", x"085F", x"0878", x"0891", x"08A9", x"08C2", x"08DB", x"08F4", x"090D", x"0926", x"093F", x"0958",
        x"0970", x"0989", x"09A2", x"09BB", x"09D4", x"09ED", x"0A06", x"0A1E", x"0A37", x"0A50", x"0A69", x"0A82",
        x"0A9A", x"0AB3", x"0ACC", x"0AE5", x"0AFD", x"0B16", x"0B2F", x"0B48", x"0B60", x"0B79", x"0B92", x"0BAB",
        x"0BC3", x"0BDC", x"0BF5", x"0C0D", x"0C26", x"0C3F", x"0C57", x"0C70", x"0C89", x"0CA1", x"0CBA", x"0CD3",
        x"0CEB", x"0D04", x"0D1C", x"0D35", x"0D4E", x"0D66", x"0D7F", x"0D97", x"0DB0", x"0DC8", x"0DE1", x"0DF9",
        x"0E12", x"0E2B", x"0E43", x"0E5C", x"0E74", x"0E8C", x"0EA5", x"0EBD", x"0ED6", x"0EEE", x"0F07", x"0F1F",
        x"0F38", x"0F50", x"0F68", x"0F81", x"0F99", x"0FB2", x"0FCA", x"0FE2", x"0FFB", x"1013", x"102B", x"1044",
        x"105C", x"1074", x"108C", x"10A5", x"10BD", x"10D5", x"10ED", x"1106", x"111E", x"1136", x"114E", x"1167",
        x"117F", x"1197", x"11AF", x"11C7", x"11DF", x"11F7", x"1210", x"1228", x"1240", x"1258", x"1270", x"1288",
        x"12A0", x"12B8", x"12D0", x"12E8", x"1300", x"1318", x"1330", x"1348", x"1360", x"1378", x"1390", x"13A8",
        x"13C0", x"13D8", x"13F0", x"1407", x"141F", x"1437", x"144F", x"1467", x"147F", x"1496", x"14AE", x"14C6",
        x"14DE", x"14F6", x"150D", x"1525", x"153D", x"1554", x"156C", x"1584", x"159B", x"15B3", x"15CB", x"15E2",
        x"15FA", x"1612", x"1629", x"1641", x"1658", x"1670", x"1687", x"169F", x"16B6", x"16CE", x"16E5", x"16FD",
        x"1714", x"172C", x"1743", x"175B", x"1772", x"1789", x"17A1", x"17B8", x"17CF", x"17E7", x"17FE", x"1815",
        x"182D", x"1844", x"185B", x"1872", x"1889", x"18A1", x"18B8", x"18CF", x"18E6", x"18FD", x"1914", x"192C",
        x"1943", x"195A", x"1971", x"1988", x"199F", x"19B6", x"19CD", x"19E4", x"19FB", x"1A12", x"1A29", x"1A40",
        x"1A57", x"1A6E", x"1A84", x"1A9B", x"1AB2", x"1AC9", x"1AE0", x"1AF7", x"1B0D", x"1B24", x"1B3B", x"1B52",
        x"1B68", x"1B7F", x"1B96", x"1BAC", x"1BC3", x"1BDA", x"1BF0", x"1C07", x"1C1E", x"1C34", x"1C4B", x"1C61",
        x"1C78", x"1C8E", x"1CA5", x"1CBB", x"1CD2", x"1CE8", x"1CFF", x"1D15", x"1D2B", x"1D42", x"1D58", x"1D6E",
        x"1D85", x"1D9B", x"1DB1", x"1DC7", x"1DDE", x"1DF4", x"1E0A", x"1E20", x"1E36", x"1E4D", x"1E63", x"1E79",
        x"1E8F", x"1EA5", x"1EBB", x"1ED1", x"1EE7", x"1EFD", x"1F13", x"1F29", x"1F3F", x"1F55", x"1F6B", x"1F81",
        x"1F97", x"1FAC", x"1FC2", x"1FD8", x"1FEE", x"2004", x"2019", x"202F", x"2045", x"205B", x"2070", x"2086",
        x"209B", x"20B1", x"20C7", x"20DC", x"20F2", x"2107", x"211D", x"2132", x"2148", x"215D", x"2173", x"2188",
        x"219E", x"21B3", x"21C8", x"21DE", x"21F3", x"2208", x"221E", x"2233", x"2248", x"225D", x"2272", x"2288",
        x"229D", x"22B2", x"22C7", x"22DC", x"22F1", x"2306", x"231B", x"2330", x"2345", x"235A", x"236F", x"2384",
        x"2399", x"23AE", x"23C3", x"23D7", x"23EC", x"2401", x"2416", x"242B", x"243F", x"2454", x"2469", x"247D",
        x"2492", x"24A7", x"24BB", x"24D0", x"24E4", x"24F9", x"250D", x"2522", x"2536", x"254B", x"255F", x"2574",
        x"2588", x"259C", x"25B1", x"25C5", x"25D9", x"25ED", x"2602", x"2616", x"262A", x"263E", x"2652", x"2666",
        x"267B", x"268F", x"26A3", x"26B7", x"26CB", x"26DF", x"26F3", x"2707", x"271A", x"272E", x"2742", x"2756",
        x"276A", x"277E", x"2791", x"27A5", x"27B9", x"27CD", x"27E0", x"27F4", x"2808", x"281B", x"282F", x"2842",
        x"2856", x"2869", x"287D", x"2890", x"28A4", x"28B7", x"28CA", x"28DE", x"28F1", x"2904", x"2918", x"292B",
        x"293E", x"2951", x"2965", x"2978", x"298B", x"299E", x"29B1", x"29C4", x"29D7", x"29EA", x"29FD", x"2A10",
        x"2A23", x"2A36", x"2A49", x"2A5C", x"2A6E", x"2A81", x"2A94", x"2AA7", x"2AB9", x"2ACC", x"2ADF", x"2AF2",
        x"2B04", x"2B17", x"2B29", x"2B3C", x"2B4E", x"2B61", x"2B73", x"2B86", x"2B98", x"2BAB", x"2BBD", x"2BCF",
        x"2BE2", x"2BF4", x"2C06", x"2C18", x"2C2B", x"2C3D", x"2C4F", x"2C61", x"2C73", x"2C85", x"2C97", x"2CA9",
        x"2CBB", x"2CCD", x"2CDF", x"2CF1", x"2D03", x"2D15", x"2D27", x"2D38", x"2D4A", x"2D5C", x"2D6E", x"2D7F",
        x"2D91", x"2DA3", x"2DB4", x"2DC6", x"2DD7", x"2DE9", x"2DFA", x"2E0C", x"2E1D", x"2E2F", x"2E40", x"2E51",
        x"2E63", x"2E74", x"2E85", x"2E97", x"2EA8", x"2EB9", x"2ECA", x"2EDB", x"2EEC", x"2EFD", x"2F0E", x"2F20",
        x"2F30", x"2F41", x"2F52", x"2F63", x"2F74", x"2F85", x"2F96", x"2FA7", x"2FB7", x"2FC8", x"2FD9", x"2FEA",
        x"2FFA", x"300B", x"301B", x"302C", x"303C", x"304D", x"305D", x"306E", x"307E", x"308F", x"309F", x"30AF",
        x"30C0", x"30D0", x"30E0", x"30F0", x"3101", x"3111", x"3121", x"3131", x"3141", x"3151", x"3161", x"3171",
        x"3181", x"3191", x"31A1", x"31B1", x"31C0", x"31D0", x"31E0", x"31F0", x"31FF", x"320F", x"321F", x"322E",
        x"323E", x"324E", x"325D", x"326D", x"327C", x"328B", x"329B", x"32AA", x"32BA", x"32C9", x"32D8", x"32E7",
        x"32F7", x"3306", x"3315", x"3324", x"3333", x"3342", x"3351", x"3360", x"336F", x"337E", x"338D", x"339C",
        x"33AB", x"33BA", x"33C8", x"33D7", x"33E6", x"33F5", x"3403", x"3412", x"3420", x"342F", x"343E", x"344C",
        x"345B", x"3469", x"3477", x"3486", x"3494", x"34A2", x"34B1", x"34BF", x"34CD", x"34DB", x"34EA", x"34F8",
        x"3506", x"3514", x"3522", x"3530", x"353E", x"354C", x"355A", x"3567", x"3575", x"3583", x"3591", x"359F",
        x"35AC", x"35BA", x"35C8", x"35D5", x"35E3", x"35F0", x"35FE", x"360B", x"3619", x"3626", x"3634", x"3641",
        x"364E", x"365C", x"3669", x"3676", x"3683", x"3690", x"369D", x"36AB", x"36B8", x"36C5", x"36D2", x"36DF",
        x"36EB", x"36F8", x"3705", x"3712", x"371F", x"372C", x"3738", x"3745", x"3752", x"375E", x"376B", x"3777",
        x"3784", x"3790", x"379D", x"37A9", x"37B6", x"37C2", x"37CE", x"37DB", x"37E7", x"37F3", x"37FF", x"380B",
        x"3818", x"3824", x"3830", x"383C", x"3848", x"3854", x"3860", x"386B", x"3877", x"3883", x"388F", x"389B",
        x"38A6", x"38B2", x"38BE", x"38C9", x"38D5", x"38E0", x"38EC", x"38F7", x"3903", x"390E", x"391A", x"3925",
        x"3930", x"393B", x"3947", x"3952", x"395D", x"3968", x"3973", x"397E", x"3989", x"3994", x"399F", x"39AA",
        x"39B5", x"39C0", x"39CB", x"39D6", x"39E0", x"39EB", x"39F6", x"3A00", x"3A0B", x"3A16", x"3A20", x"3A2B",
        x"3A35", x"3A3F", x"3A4A", x"3A54", x"3A5F", x"3A69", x"3A73", x"3A7D", x"3A88", x"3A92", x"3A9C", x"3AA6",
        x"3AB0", x"3ABA", x"3AC4", x"3ACE", x"3AD8", x"3AE2", x"3AEB", x"3AF5", x"3AFF", x"3B09", x"3B12", x"3B1C",
        x"3B26", x"3B2F", x"3B39", x"3B42", x"3B4C", x"3B55", x"3B5F", x"3B68", x"3B71", x"3B7B", x"3B84", x"3B8D",
        x"3B96", x"3B9F", x"3BA9", x"3BB2", x"3BBB", x"3BC4", x"3BCD", x"3BD6", x"3BDE", x"3BE7", x"3BF0", x"3BF9",
        x"3C02", x"3C0A", x"3C13", x"3C1C", x"3C24", x"3C2D", x"3C36", x"3C3E", x"3C46", x"3C4F", x"3C57", x"3C60",
        x"3C68", x"3C70", x"3C79", x"3C81", x"3C89", x"3C91", x"3C99", x"3CA1", x"3CA9", x"3CB1", x"3CB9", x"3CC1",
        x"3CC9", x"3CD1", x"3CD9", x"3CE0", x"3CE8", x"3CF0", x"3CF8", x"3CFF", x"3D07", x"3D0E", x"3D16", x"3D1D",
        x"3D25", x"3D2C", x"3D34", x"3D3B", x"3D42", x"3D49", x"3D51", x"3D58", x"3D5F", x"3D66", x"3D6D", x"3D74",
        x"3D7B", x"3D82", x"3D89", x"3D90", x"3D97", x"3D9E", x"3DA4", x"3DAB", x"3DB2", x"3DB9", x"3DBF", x"3DC6",
        x"3DCC", x"3DD3", x"3DD9", x"3DE0", x"3DE6", x"3DED", x"3DF3", x"3DF9", x"3DFF", x"3E06", x"3E0C", x"3E12",
        x"3E18", x"3E1E", x"3E24", x"3E2A", x"3E30", x"3E36", x"3E3C", x"3E42", x"3E48", x"3E4D", x"3E53", x"3E59",
        x"3E5E", x"3E64", x"3E6A", x"3E6F", x"3E75", x"3E7A", x"3E80", x"3E85", x"3E8A", x"3E90", x"3E95", x"3E9A",
        x"3E9F", x"3EA5", x"3EAA", x"3EAF", x"3EB4", x"3EB9", x"3EBE", x"3EC3", x"3EC8", x"3ECC", x"3ED1", x"3ED6",
        x"3EDB", x"3EE0", x"3EE4", x"3EE9", x"3EED", x"3EF2", x"3EF7", x"3EFB", x"3F00", x"3F04", x"3F08", x"3F0D",
        x"3F11", x"3F15", x"3F19", x"3F1E", x"3F22", x"3F26", x"3F2A", x"3F2E", x"3F32", x"3F36", x"3F3A", x"3F3E",
        x"3F42", x"3F45", x"3F49", x"3F4D", x"3F51", x"3F54", x"3F58", x"3F5B", x"3F5F", x"3F62", x"3F66", x"3F69",
        x"3F6D", x"3F70", x"3F73", x"3F77", x"3F7A", x"3F7D", x"3F80", x"3F83", x"3F86", x"3F89", x"3F8C", x"3F8F",
        x"3F92", x"3F95", x"3F98", x"3F9B", x"3F9E", x"3FA0", x"3FA3", x"3FA6", x"3FA8", x"3FAB", x"3FAD", x"3FB0",
        x"3FB2", x"3FB5", x"3FB7", x"3FB9", x"3FBC", x"3FBE", x"3FC0", x"3FC3", x"3FC5", x"3FC7", x"3FC9", x"3FCB",
        x"3FCD", x"3FCF", x"3FD1", x"3FD3", x"3FD5", x"3FD6", x"3FD8", x"3FDA", x"3FDC", x"3FDD", x"3FDF", x"3FE0",
        x"3FE2", x"3FE3", x"3FE5", x"3FE6", x"3FE8", x"3FE9", x"3FEA", x"3FEC", x"3FED", x"3FEE", x"3FEF", x"3FF0",
        x"3FF1", x"3FF2", x"3FF3", x"3FF4", x"3FF5", x"3FF6", x"3FF7", x"3FF8", x"3FF9", x"3FF9", x"3FFA", x"3FFB",
        x"3FFB", x"3FFC", x"3FFC", x"3FFD", x"3FFD", x"3FFE", x"3FFE", x"3FFF", x"3FFF", x"3FFF", x"3FFF", x"4000",
        x"4000", x"4000", x"4000", x"4000"
    );
    -- cell-units per pixel / 8: 2^20 / cell / 8, cell = 4 * 40^(i / 1023) px
    constant C_INV : t_rom := (
        x"8000", x"7F8A", x"7F15", x"7E9F", x"7E2B", x"7DB6", x"7D43", x"7CCF", x"7C5C", x"7BEA", x"7B77", x"7B06",
        x"7A94", x"7A23", x"79B3", x"7943", x"78D3", x"7864", x"77F5", x"7786", x"7718", x"76AA", x"763D", x"75D0",
        x"7563", x"74F7", x"748B", x"7420", x"73B5", x"734A", x"72E0", x"7276", x"720D", x"71A4", x"713B", x"70D3",
        x"706B", x"7003", x"6F9C", x"6F35", x"6ECF", x"6E69", x"6E03", x"6D9D", x"6D38", x"6CD4", x"6C70", x"6C0C",
        x"6BA8", x"6B45", x"6AE2", x"6A80", x"6A1D", x"69BC", x"695A", x"68F9", x"6898", x"6838", x"67D8", x"6778",
        x"6719", x"66BA", x"665B", x"65FD", x"659F", x"6541", x"64E4", x"6487", x"642A", x"63CE", x"6372", x"6317",
        x"62BB", x"6260", x"6206", x"61AB", x"6151", x"60F8", x"609E", x"6045", x"5FED", x"5F94", x"5F3C", x"5EE4",
        x"5E8D", x"5E36", x"5DDF", x"5D88", x"5D32", x"5CDC", x"5C87", x"5C32", x"5BDD", x"5B88", x"5B34", x"5AE0",
        x"5A8C", x"5A38", x"59E5", x"5992", x"5940", x"58EE", x"589C", x"584A", x"57F9", x"57A8", x"5757", x"5706",
        x"56B6", x"5666", x"5617", x"55C7", x"5578", x"552A", x"54DB", x"548D", x"543F", x"53F1", x"53A4", x"5357",
        x"530A", x"52BE", x"5271", x"5225", x"51DA", x"518E", x"5143", x"50F8", x"50AE", x"5063", x"5019", x"4FCF",
        x"4F86", x"4F3D", x"4EF4", x"4EAB", x"4E62", x"4E1A", x"4DD2", x"4D8A", x"4D43", x"4CFC", x"4CB5", x"4C6E",
        x"4C28", x"4BE2", x"4B9C", x"4B56", x"4B11", x"4ACB", x"4A86", x"4A42", x"49FD", x"49B9", x"4975", x"4932",
        x"48EE", x"48AB", x"4868", x"4825", x"47E3", x"47A1", x"475F", x"471D", x"46DB", x"469A", x"4659", x"4618",
        x"45D7", x"4597", x"4557", x"4517", x"44D7", x"4498", x"4459", x"441A", x"43DB", x"439D", x"435E", x"4320",
        x"42E2", x"42A5", x"4267", x"422A", x"41ED", x"41B0", x"4174", x"4138", x"40FB", x"40C0", x"4084", x"4048",
        x"400D", x"3FD2", x"3F97", x"3F5D", x"3F22", x"3EE8", x"3EAE", x"3E75", x"3E3B", x"3E02", x"3DC8", x"3D90",
        x"3D57", x"3D1E", x"3CE6", x"3CAE", x"3C76", x"3C3E", x"3C07", x"3BCF", x"3B98", x"3B61", x"3B2B", x"3AF4",
        x"3ABE", x"3A88", x"3A52", x"3A1C", x"39E6", x"39B1", x"397C", x"3947", x"3912", x"38DE", x"38A9", x"3875",
        x"3841", x"380D", x"37DA", x"37A6", x"3773", x"3740", x"370D", x"36DA", x"36A7", x"3675", x"3643", x"3611",
        x"35DF", x"35AD", x"357C", x"354B", x"351A", x"34E9", x"34B8", x"3487", x"3457", x"3427", x"33F7", x"33C7",
        x"3397", x"3368", x"3338", x"3309", x"32DA", x"32AB", x"327C", x"324E", x"3220", x"31F1", x"31C3", x"3195",
        x"3168", x"313A", x"310D", x"30E0", x"30B3", x"3086", x"3059", x"302D", x"3000", x"2FD4", x"2FA8", x"2F7C",
        x"2F50", x"2F25", x"2EF9", x"2ECE", x"2EA3", x"2E78", x"2E4D", x"2E22", x"2DF8", x"2DCD", x"2DA3", x"2D79",
        x"2D4F", x"2D26", x"2CFC", x"2CD2", x"2CA9", x"2C80", x"2C57", x"2C2E", x"2C05", x"2BDD", x"2BB4", x"2B8C",
        x"2B64", x"2B3C", x"2B14", x"2AED", x"2AC5", x"2A9E", x"2A76", x"2A4F", x"2A28", x"2A01", x"29DB", x"29B4",
        x"298E", x"2967", x"2941", x"291B", x"28F5", x"28D0", x"28AA", x"2884", x"285F", x"283A", x"2815", x"27F0",
        x"27CB", x"27A6", x"2782", x"275E", x"2739", x"2715", x"26F1", x"26CD", x"26A9", x"2686", x"2662", x"263F",
        x"261C", x"25F9", x"25D6", x"25B3", x"2590", x"256D", x"254B", x"2529", x"2506", x"24E4", x"24C2", x"24A0",
        x"247F", x"245D", x"243B", x"241A", x"23F9", x"23D8", x"23B7", x"2396", x"2375", x"2354", x"2334", x"2313",
        x"22F3", x"22D3", x"22B3", x"2293", x"2273", x"2253", x"2233", x"2214", x"21F5", x"21D5", x"21B6", x"2197",
        x"2178", x"2159", x"213A", x"211C", x"20FD", x"20DF", x"20C1", x"20A2", x"2084", x"2066", x"2049", x"202B",
        x"200D", x"1FF0", x"1FD2", x"1FB5", x"1F98", x"1F7B", x"1F5E", x"1F41", x"1F24", x"1F07", x"1EEB", x"1ECE",
        x"1EB2", x"1E95", x"1E79", x"1E5D", x"1E41", x"1E25", x"1E0A", x"1DEE", x"1DD2", x"1DB7", x"1D9B", x"1D80",
        x"1D65", x"1D4A", x"1D2F", x"1D14", x"1CF9", x"1CDF", x"1CC4", x"1CA9", x"1C8F", x"1C75", x"1C5A", x"1C40",
        x"1C26", x"1C0C", x"1BF3", x"1BD9", x"1BBF", x"1BA6", x"1B8C", x"1B73", x"1B59", x"1B40", x"1B27", x"1B0E",
        x"1AF5", x"1ADC", x"1AC4", x"1AAB", x"1A92", x"1A7A", x"1A61", x"1A49", x"1A31", x"1A19", x"1A01", x"19E9",
        x"19D1", x"19B9", x"19A1", x"198A", x"1972", x"195B", x"1943", x"192C", x"1915", x"18FE", x"18E7", x"18D0",
        x"18B9", x"18A2", x"188C", x"1875", x"185E", x"1848", x"1832", x"181B", x"1805", x"17EF", x"17D9", x"17C3",
        x"17AD", x"1797", x"1781", x"176C", x"1756", x"1741", x"172B", x"1716", x"1701", x"16EB", x"16D6", x"16C1",
        x"16AC", x"1697", x"1683", x"166E", x"1659", x"1645", x"1630", x"161C", x"1607", x"15F3", x"15DF", x"15CB",
        x"15B7", x"15A2", x"158F", x"157B", x"1567", x"1553", x"1540", x"152C", x"1518", x"1505", x"14F2", x"14DE",
        x"14CB", x"14B8", x"14A5", x"1492", x"147F", x"146C", x"1459", x"1446", x"1434", x"1421", x"140F", x"13FC",
        x"13EA", x"13D7", x"13C5", x"13B3", x"13A1", x"138F", x"137D", x"136B", x"1359", x"1347", x"1335", x"1323",
        x"1312", x"1300", x"12EF", x"12DD", x"12CC", x"12BB", x"12A9", x"1298", x"1287", x"1276", x"1265", x"1254",
        x"1243", x"1232", x"1221", x"1211", x"1200", x"11F0", x"11DF", x"11CF", x"11BE", x"11AE", x"119D", x"118D",
        x"117D", x"116D", x"115D", x"114D", x"113D", x"112D", x"111D", x"110D", x"10FE", x"10EE", x"10DF", x"10CF",
        x"10BF", x"10B0", x"10A1", x"1091", x"1082", x"1073", x"1064", x"1055", x"1046", x"1037", x"1028", x"1019",
        x"100A", x"0FFB", x"0FEC", x"0FDE", x"0FCF", x"0FC1", x"0FB2", x"0FA4", x"0F95", x"0F87", x"0F78", x"0F6A",
        x"0F5C", x"0F4E", x"0F40", x"0F32", x"0F24", x"0F16", x"0F08", x"0EFA", x"0EEC", x"0EDE", x"0ED1", x"0EC3",
        x"0EB6", x"0EA8", x"0E9A", x"0E8D", x"0E80", x"0E72", x"0E65", x"0E58", x"0E4A", x"0E3D", x"0E30", x"0E23",
        x"0E16", x"0E09", x"0DFC", x"0DEF", x"0DE2", x"0DD6", x"0DC9", x"0DBC", x"0DB0", x"0DA3", x"0D96", x"0D8A",
        x"0D7D", x"0D71", x"0D65", x"0D58", x"0D4C", x"0D40", x"0D33", x"0D27", x"0D1B", x"0D0F", x"0D03", x"0CF7",
        x"0CEB", x"0CDF", x"0CD3", x"0CC8", x"0CBC", x"0CB0", x"0CA4", x"0C99", x"0C8D", x"0C81", x"0C76", x"0C6A",
        x"0C5F", x"0C54", x"0C48", x"0C3D", x"0C32", x"0C26", x"0C1B", x"0C10", x"0C05", x"0BFA", x"0BEF", x"0BE4",
        x"0BD9", x"0BCE", x"0BC3", x"0BB8", x"0BAE", x"0BA3", x"0B98", x"0B8D", x"0B83", x"0B78", x"0B6E", x"0B63",
        x"0B58", x"0B4E", x"0B44", x"0B39", x"0B2F", x"0B25", x"0B1A", x"0B10", x"0B06", x"0AFC", x"0AF2", x"0AE8",
        x"0ADD", x"0AD3", x"0ACA", x"0AC0", x"0AB6", x"0AAC", x"0AA2", x"0A98", x"0A8E", x"0A85", x"0A7B", x"0A71",
        x"0A68", x"0A5E", x"0A55", x"0A4B", x"0A42", x"0A38", x"0A2F", x"0A25", x"0A1C", x"0A13", x"0A09", x"0A00",
        x"09F7", x"09EE", x"09E5", x"09DB", x"09D2", x"09C9", x"09C0", x"09B7", x"09AE", x"09A5", x"099D", x"0994",
        x"098B", x"0982", x"0979", x"0971", x"0968", x"095F", x"0957", x"094E", x"0945", x"093D", x"0934", x"092C",
        x"0923", x"091B", x"0913", x"090A", x"0902", x"08FA", x"08F1", x"08E9", x"08E1", x"08D9", x"08D1", x"08C8",
        x"08C0", x"08B8", x"08B0", x"08A8", x"08A0", x"0898", x"0890", x"0889", x"0881", x"0879", x"0871", x"0869",
        x"0861", x"085A", x"0852", x"084A", x"0843", x"083B", x"0834", x"082C", x"0824", x"081D", x"0815", x"080E",
        x"0807", x"07FF", x"07F8", x"07F1", x"07E9", x"07E2", x"07DB", x"07D3", x"07CC", x"07C5", x"07BE", x"07B7",
        x"07B0", x"07A9", x"07A1", x"079A", x"0793", x"078C", x"0785", x"077F", x"0778", x"0771", x"076A", x"0763",
        x"075C", x"0755", x"074F", x"0748", x"0741", x"073B", x"0734", x"072D", x"0727", x"0720", x"071A", x"0713",
        x"070C", x"0706", x"0700", x"06F9", x"06F3", x"06EC", x"06E6", x"06E0", x"06D9", x"06D3", x"06CD", x"06C6",
        x"06C0", x"06BA", x"06B4", x"06AD", x"06A7", x"06A1", x"069B", x"0695", x"068F", x"0689", x"0683", x"067D",
        x"0677", x"0671", x"066B", x"0665", x"065F", x"0659", x"0653", x"064E", x"0648", x"0642", x"063C", x"0637",
        x"0631", x"062B", x"0625", x"0620", x"061A", x"0614", x"060F", x"0609", x"0604", x"05FE", x"05F9", x"05F3",
        x"05EE", x"05E8", x"05E3", x"05DD", x"05D8", x"05D3", x"05CD", x"05C8", x"05C3", x"05BD", x"05B8", x"05B3",
        x"05AD", x"05A8", x"05A3", x"059E", x"0599", x"0593", x"058E", x"0589", x"0584", x"057F", x"057A", x"0575",
        x"0570", x"056B", x"0566", x"0561", x"055C", x"0557", x"0552", x"054D", x"0548", x"0543", x"053F", x"053A",
        x"0535", x"0530", x"052B", x"0527", x"0522", x"051D", x"0518", x"0514", x"050F", x"050A", x"0506", x"0501",
        x"04FC", x"04F8", x"04F3", x"04EF", x"04EA", x"04E6", x"04E1", x"04DD", x"04D8", x"04D4", x"04CF", x"04CB",
        x"04C6", x"04C2", x"04BE", x"04B9", x"04B5", x"04B1", x"04AC", x"04A8", x"04A4", x"049F", x"049B", x"0497",
        x"0493", x"048E", x"048A", x"0486", x"0482", x"047E", x"047A", x"0475", x"0471", x"046D", x"0469", x"0465",
        x"0461", x"045D", x"0459", x"0455", x"0451", x"044D", x"0449", x"0445", x"0441", x"043D", x"0439", x"0435",
        x"0432", x"042E", x"042A", x"0426", x"0422", x"041E", x"041B", x"0417", x"0413", x"040F", x"040C", x"0408",
        x"0404", x"0400", x"03FD", x"03F9", x"03F5", x"03F2", x"03EE", x"03EB", x"03E7", x"03E3", x"03E0", x"03DC",
        x"03D9", x"03D5", x"03D2", x"03CE", x"03CA", x"03C7", x"03C4", x"03C0", x"03BD", x"03B9", x"03B6", x"03B2",
        x"03AF", x"03AC", x"03A8", x"03A5", x"03A1", x"039E", x"039B", x"0397", x"0394", x"0391", x"038E", x"038A",
        x"0387", x"0384", x"0380", x"037D", x"037A", x"0377", x"0374", x"0370", x"036D", x"036A", x"0367", x"0364",
        x"0361", x"035E", x"035B", x"0357", x"0354", x"0351", x"034E", x"034B", x"0348", x"0345", x"0342", x"033F",
        x"033C", x"0339", x"0336", x"0333"
    );
    -- Black / Red / Orange / Yellow / Green / Cyan / Blue / Magenta: BT.601 10-bit, U/V swapped for the
    -- hardware (the scope-verified phosphor rainbow values where they exist)
    constant C_INK_A  : t_ink := (to_unsigned(0, 10), to_unsigned(264, 10), to_unsigned(520, 10), to_unsigned(776, 10), to_unsigned(516, 10), to_unsigned(616, 10), to_unsigned(100, 10), to_unsigned(360, 10));
    constant C_INK_AN : t_ink := (to_unsigned(0, 10), to_unsigned(308, 10), to_unsigned(607, 10), to_unsigned(906, 10), to_unsigned(603, 10), to_unsigned(719, 10), to_unsigned(117, 10), to_unsigned(420, 10));
    constant C_INK_U  : t_ink := (to_unsigned(512, 10), to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10), to_unsigned(64, 10), to_unsigned(440, 10), to_unsigned(888, 10));
    constant C_INK_V  : t_ink := (to_unsigned(512, 10), to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned(64, 10), to_unsigned(216, 10), to_unsigned(664, 10), to_unsigned(960, 10), to_unsigned(808, 10));
    -- GEN END

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    -- (16h + l)^2 in parts: h^2 & l^2 & h*l (three 4-bit products)
    function f_sqp(a : unsigned(7 downto 0)) return unsigned is
        variable h, l : unsigned(3 downto 0);
    begin
        h := a(7 downto 4); l := a(3 downto 0);
        return (h * h) & (l * l) & (h * l);
    end function;

    -- recombine: 256 h^2 + l^2 + 32 h l
    function f_sqs(p : unsigned(23 downto 0)) return unsigned is
    begin
        return p(23 downto 8) + shift_left(resize(p(7 downto 0), 16), 5);
    end function;

    -- fold a cell coordinate (top 8 bits of a phase) to its distance from
    -- the cell centre: odd 1..255, symmetric about the centre
    function f_fold(f : unsigned(7 downto 0)) return unsigned is
    begin
        if f(7) = '1' then
            return f(6 downto 0) & '1';
        else
            return (not f(6 downto 0)) & '1';
        end if;
    end function;

    -- glide: g += (target*16 - g)/8, at least 1 toward the target
    function f_glide(g : unsigned(13 downto 0); t : unsigned(9 downto 0)) return unsigned is
        variable d : signed(15 downto 0);
        variable s : signed(15 downto 0);
    begin
        d := signed(resize(unsigned'(t & "0000"), 16)) - signed(resize(g, 16));
        s := signed(resize(g, 16)) + shift_right(d, 3);
        if d > 0 then
            s := s + 1;
        end if;
        return unsigned(s(13 downto 0));
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
    -- smallest P12 Size index (C_INV) per mode: a cell at least 2 field
    -- lines tall (480i 8.9, 576i 7.5, 480p 4.5 virtual units).  Below
    -- that the lattice beats against the line pitch (an honest 4.5-unit
    -- cell is one 480i line: horizontal ink / paper bands); 1080p and 720p
    -- never reach the floor.
    function f_sizfloor(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0      => return to_unsigned(222, 10);
            when 8      => return to_unsigned(175, 10);
            when 4      => return to_unsigned(33, 10);
            when others => return to_unsigned(0, 10);
        end case;
    end function;
    -- a step magnitude x the x step (half units: 2 = x1, 3 = x1.5, 4 = x2)
    function f_xscale(v : unsigned(19 downto 0); xs : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(xs) is
            when 3 => return v + shift_right(v, 1);
            when 4 => return shift_left(v, 1);
            when others => return v;
        end case;
    end function;

    -- |a - b| > 2 (pot-noise hysteresis)
    function f_moved(a, b : unsigned(9 downto 0)) return boolean is
        variable d : signed(10 downto 0);
    begin
        d := signed('0' & a) - signed('0' & b);
        return d > 2 or d < -2;
    end function;

    signal s_k_ang, s_k_mix, s_k_dot, s_k_blend, s_k_ink, s_k_gain : unsigned(9 downto 0);
    signal s_sw_reg, s_sw_inv, s_sw_spin, s_sw_inkc, s_sw_ros : std_logic;
    signal s_p_size : unsigned(9 downto 0);

    -- input, measurement
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_rx : unsigned(11 downto 0) := (others => '0');
    signal s_width : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_line : unsigned(10 downto 0) := (others => '0');
    signal s_height : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_w_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_h_h : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_saw : std_logic := '0';
    -- Resolution scaling steps (decoded every clock from the timing ID)
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsd : std_logic := '0';
    signal s_sfl, s_sizi : unsigned(9 downto 0) := (others => '0');   -- size floor, floored size index

    -- per-field control state
    signal s_ang_h, s_siz_h, s_gain_h : unsigned(9 downto 0) := (others => '0');
    signal s_ang_g, s_siz_g : unsigned(13 downto 0) := (others => '0');
    signal s_th : unsigned(19 downto 0) := (others => '0');        -- 12.8, 4096 a turn
    signal s_rate : unsigned(12 downto 0) := to_unsigned(64, 13);
    signal s_dot : unsigned(1 downto 0) := "00";
    signal s_mode : unsigned(2 downto 0) := "010";
    signal s_inkz : unsigned(2 downto 0) := "000";
    signal s_live_dot, s_live_mode, s_live_ink : std_logic := '0';
    signal s_k6 : unsigned(12 downto 0) := (others => '0');        -- K4 * 6
    signal s_gain : unsigned(7 downto 0) := (others => '0');
    signal s_mix : unsigned(8 downto 0) := (others => '0');        -- 32..256
    signal s_delta : unsigned(7 downto 0) := (others => '0');
    signal s_ink_a, s_ink_an, s_ink_ac, s_ink_u, s_ink_v : unsigned(9 downto 0) := (others => '0');
    signal s_ink_lo : std_logic := '1';
    signal s_ink_ou, s_ink_ov : signed(10 downto 0) := (others => '0');

    -- vblank sequencer + serial multiplier
    signal s_sq : unsigned(4 downto 0) := (others => '0');
    signal s_scr : std_logic := '0';
    signal s_t1, s_tc : unsigned(11 downto 0) := (others => '0');
    signal s_wc : unsigned(10 downto 0) := (others => '0');
    signal s_hc : unsigned(9 downto 0) := (others => '0');
    signal s_sa : unsigned(9 downto 0) := (others => '0');
    signal s_sneg_a, s_sneg : std_logic := '0';
    signal s_sin_q : unsigned(15 downto 0) := (others => '0');
    signal s_sin_r : unsigned(15 downto 0) := (others => '0');
    signal s_inv_q : unsigned(15 downto 0) := (others => '0');
    signal s_inv_r : unsigned(15 downto 0) := (others => '0');
    signal s_smag : unsigned(14 downto 0) := (others => '0');
    signal s_sng : std_logic := '0';
    signal s_ct, s_st, s_u0t, s_v0t : unsigned(19 downto 0) := (others => '0');
    signal s_cy, s_sy : unsigned(19 downto 0) := (others => '0');       -- per-line steps
    signal s_invx, s_invy : unsigned(19 downto 0) := (others => '0');   -- 1/cell x the x / y step
    signal s_ma, s_mb : unsigned(19 downto 0) := (others => '0');
    signal s_macc : unsigned(39 downto 0) := (others => '0');
    signal s_mcnt : unsigned(4 downto 0) := (others => '0');

    -- screen DDAs (mod one cell, 20 bits)
    signal s_c1, s_s1, s_c2, s_s2 : unsigned(19 downto 0) := (others => '0');
    signal s_c1y, s_s1y, s_c2y, s_s2y : unsigned(19 downto 0) := (others => '0');
    signal s_ur1, s_vr1, s_ur2, s_vr2 : unsigned(19 downto 0) := (others => '0');
    signal s_u1, s_v1, s_u2, s_v2 : unsigned(19 downto 0) := (others => '0');

    -- pixel pipeline
    signal s_fu1, s_fv1, s_fu2, s_fv2 : unsigned(7 downto 0) := (others => '0');     -- 2
    signal s_pu1, s_pv1, s_pu2, s_pv2 : unsigned(23 downto 0) := (others => '0');    -- 3
    signal s_au1, s_av1, s_au2, s_av2 : unsigned(7 downto 0) := (others => '0');     -- 3
    signal s_yg3 : unsigned(17 downto 0) := (others => '0');
    signal s_qu1, s_qv1, s_qu2, s_qv2 : unsigned(15 downto 0) := (others => '0');    -- 4
    signal s_bu1, s_bv1, s_bu2, s_bv2 : unsigned(7 downto 0) := (others => '0');     -- 4
    signal s_yg4, s_yg5, s_yg6 : unsigned(10 downto 0) := (others => '0');
    signal s_dc1, s_dd1, s_ds1, s_dl1 : unsigned(9 downto 0) := (others => '0');     -- 5
    signal s_dc2, s_dd2, s_ds2, s_dl2 : unsigned(9 downto 0) := (others => '0');
    signal s_dn1, s_dn2 : unsigned(9 downto 0) := (others => '0');                    -- 6
    signal s_ink7 : std_logic := '0';                                                 -- 7
    signal s_a8, s_an8, s_ac8, s_b8, s_bc8 : unsigned(9 downto 0) := (others => '0'); -- 8
    signal s_alo8 : std_logic := '0';
    signal s_p9, s_q9 : unsigned(19 downto 0) := (others => '0');                     -- 9
    signal s_sum9 : unsigned(10 downto 0) := (others => '0');
    signal s_dif9 : signed(10 downto 0) := (others => '0');
    signal s_alo9, s_blo9 : std_logic := '0';
    signal s_c_add10, s_c_scr10, s_c_mul10, s_c_dif10, s_c_lo10, s_c_hi10 : signed(11 downto 0) := (others => '0');
    signal s_alo10, s_blo10 : std_logic := '0';
    signal s_sel11 : signed(11 downto 0) := (others => '0');                          -- 11
    signal s_t12 : unsigned(9 downto 0) := (others => '0');                           -- 12
    signal s_dy13, s_du13, s_dv13 : signed(10 downto 0) := (others => '0');           -- 13
    signal s_py14, s_pu14, s_pv14 : signed(20 downto 0) := (others => '0');           -- 14
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');             -- 15
    signal s_y16, s_y17, s_y18 : unsigned(9 downto 0) := (others => '0');             -- 16..18
    signal s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_g1y, s_g2y, s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');             -- 20
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');             -- 21

    -- colour ring (Register): written at 16, read delta back
    type t_ring is array (0 to 255) of unsigned(9 downto 0);
    signal r_ring_u, r_ring_v : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_ring_u, r_ring_v : signal is true;
    signal s_wp, s_ra : unsigned(7 downto 0) := (others => '0');
    -- read-back shift for the pixel entering stage 16: 0 for the first
    -- C_EDGE pixels, then +1 a pixel up to delta, so the line's left edge
    -- holds pixel C_EDGE's colour instead of showing a colourless strip
    constant C_EDGE : integer := 12;
    signal s_sh : unsigned(7 downto 0) := (others => '0');
    signal s_ex : unsigned(3 downto 0) := (others => '0');
    signal s_rdu, s_rdv : unsigned(9 downto 0) := (others => '0');

    type t_c10 is array (2 to 14) of unsigned(9 downto 0);
    signal s_bd, s_ud, s_vd : t_c10 := (others => (others => '0'));
    type t_inkd is array (8 to 12) of std_logic;
    signal s_inkd : t_inkd := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_ang   <= unsigned(registers_in(0));
    s_k_mix   <= unsigned(registers_in(1));
    s_k_dot   <= unsigned(registers_in(2));
    s_k_blend <= unsigned(registers_in(3));
    s_k_ink   <= unsigned(registers_in(4));
    s_k_gain  <= unsigned(registers_in(5));
    s_sw_reg  <= registers_in(6)(0);
    s_sw_inv  <= registers_in(6)(1);
    s_sw_spin <= registers_in(6)(2);
    s_sw_inkc <= registers_in(6)(3);
    s_sw_ros  <= registers_in(6)(4);
    s_p_size  <= unsigned(registers_in(7));

    ----------------------------------------------------------------------
    -- controls, measurement, per-field state
    ----------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_vedge : boolean;
        variable v_dw : signed(12 downto 0);
        variable v_dh : signed(11 downto 0);
        variable v_z : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            v_vedge := data_in.vsync_n = '0' and s_prev_vsync_n = '1';

            -- width = active pixels on the line just ended; height = active
            -- lines so far (captured in the increment branch, not at vsync)
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw <= '1';
            else
                s_rx <= (others => '0');
            end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_width <= s_rx;
                s_line <= s_line + 1;
                s_height <= s_line + 1;
            end if;
            if v_vedge then
                s_line <= (others => '0');
            end if;

            -- once per field (the saw-active guard: analog vsync serrates)
            if v_vedge and s_saw = '1' then
                s_saw <= '0';
                v_dw := signed(resize(s_width, 13)) - signed(resize(s_w_h, 13));
                if v_dw > 3 or v_dw < -3 then s_w_h <= s_width; end if;
                v_dh := signed(resize(s_height, 12)) - signed(resize(s_h_h, 12));
                if v_dh > 3 or v_dh < -3 then s_h_h <= s_height; end if;

                if f_moved(s_k_ang, s_ang_h) then s_ang_h <= s_k_ang; end if;
                if f_moved(s_p_size, s_siz_h) then s_siz_h <= s_p_size; end if;
                if f_moved(s_k_gain, s_gain_h) then s_gain_h <= s_k_gain; end if;
                s_ang_g <= f_glide(s_ang_g, s_ang_h);
                s_siz_g <= f_glide(s_siz_g, s_siz_h);
                if s_sw_spin = '1' then
                    s_th <= s_th + resize(s_rate, 20);
                else
                    s_th <= "00" & s_ang_g & "0000";
                end if;

                -- zones change only 12+ counts inside the new zone
                if s_k_dot >= 4 then s_live_dot <= '1'; end if;
                if s_live_dot = '0' and s_k_dot < 4 then
                    s_dot <= s_k_dot(1 downto 0);
                elsif s_k_dot(7 downto 0) >= 12 and s_k_dot(7 downto 0) <= 243 then
                    s_dot <= s_k_dot(9 downto 8);
                end if;
                if s_k_blend >= 6 then s_live_mode <= '1'; end if;
                if s_live_mode = '0' and s_k_blend < 6 then
                    s_mode <= s_k_blend(2 downto 0);
                elsif s_k6(9 downto 0) >= 64 and s_k6(9 downto 0) <= 959 then
                    s_mode <= s_k6(12 downto 10);
                end if;
                if s_k_ink >= 8 then s_live_ink <= '1'; end if;
                if s_live_ink = '0' and s_k_ink < 8 then
                    s_inkz <= s_k_ink(2 downto 0);
                elsif s_k_ink(6 downto 0) >= 12 and s_k_ink(6 downto 0) <= 115 then
                    s_inkz <= s_k_ink(9 downto 7);
                end if;
            end if;

            -- free-running per-frame constants
            s_k6 <= shift_left(resize(s_k_blend, 13), 2) + shift_left(resize(s_k_blend, 13), 1);
            s_gain <= s_gain_h(9 downto 2);
            s_mix <= to_unsigned(32, 9) + resize(s_k_mix(9 downto 2), 9) - resize(s_k_mix(9 downto 5), 9);
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            if s_xs = 4 then s_xsd <= '1'; else s_xsd <= '0'; end if;
            s_sfl <= f_sizfloor(registers_in(8)(3 downto 0));
            -- Register offset W/32 (60 px at 1080, 40 at 720p); SD's 720 px
            -- span 1440 virtual units, so W x 11/256 (~W/24, 29 px = 58)
            if s_sw_reg = '1' then
                if s_xsd = '1' then
                    s_delta <= resize(s_w_h(11 downto 5), 8) + resize(s_w_h(11 downto 7), 8) + resize(s_w_h(11 downto 8), 8);
                else
                    s_delta <= resize(s_w_h(11 downto 5), 8);
                end if;
            else
                s_delta <= (others => '0');
            end if;
            v_z := to_integer(s_inkz);
            s_ink_a <= C_INK_A(v_z);
            s_ink_an <= C_INK_AN(v_z);
            s_ink_u <= C_INK_U(v_z);
            s_ink_v <= C_INK_V(v_z);
            s_ink_ac <= to_unsigned(1023, 10) - s_ink_an;
            if s_ink_an < 512 then s_ink_lo <= '1'; else s_ink_lo <= '0'; end if;
            s_ink_ou <= shift_right(signed(resize(s_ink_u, 11)) - 512, 1);
            s_ink_ov <= shift_right(signed(resize(s_ink_v, 11)) - 512, 1);

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ----------------------------------------------------------------------
    -- vblank sequencer: per screen k (angle t, t + 30 degrees)
    --   c = cos(t)/cell, s = sin(t)/cell  (cell units per pixel, 2^20 a cell)
    --   u0 = 1/2 - (wc*c + hc*s),  v0 = 1/2 + wc*s - hc*c
    -- so the picture centre sits in the middle of a cell.  One small op per
    -- state; every product on the serial multiplier.  Restarts on every
    -- vsync edge (idempotent) and finishes in ~330 clocks, inside vblank.
    ----------------------------------------------------------------------
    p_seq : process(clk)
        variable v_sum : unsigned(20 downto 0);
        variable v_p, v_pn : unsigned(19 downto 0);
        variable v_go : boolean;
    begin
        if rising_edge(clk) then
            -- ROMs: registered read (absorbed into the EBR), then a fabric copy
            s_sin_q <= C_SIN(to_integer(s_sa));
            s_sin_r <= s_sin_q;
            -- (the size index floored at 2 field lines a cell: SD only)
            if s_siz_g(13 downto 4) < s_sfl then s_sizi <= s_sfl; else s_sizi <= s_siz_g(13 downto 4); end if;
            s_inv_q <= C_INV(to_integer(s_sizi));
            s_inv_r <= s_inv_q;
            s_invx <= f_xscale(resize(unsigned'(s_inv_r & "000"), 20), s_xs);
            s_sneg <= s_sneg_a;

            -- serial multiplier: macc = ma * mb after 20 steps
            if s_mcnt /= 0 then
                if s_mb(0) = '1' then
                    v_sum := resize(s_macc(39 downto 20), 21) + resize(s_ma, 21);
                else
                    v_sum := resize(s_macc(39 downto 20), 21);
                end if;
                s_macc <= v_sum & s_macc(19 downto 1);
                s_mb <= '0' & s_mb(19 downto 1);
                s_mcnt <= s_mcnt - 1;
            end if;

            v_go := s_mcnt = 0;
            v_p := s_macc(33 downto 14);
            v_pn := 0 - v_p;     -- one shared negator for the four steps
            case to_integer(s_sq) is
                when 0 => null;
                when 1 =>
                    s_t1 <= s_th(19 downto 8);
                    s_wc <= s_w_h(11 downto 1);
                    s_hc <= s_h_h(10 downto 1);
                    s_scr <= '0';
                    s_sq <= to_unsigned(2, 5);
                when 2 =>
                    if s_scr = '1' then
                        s_tc <= s_t1 + 341;           -- +30 degrees
                    else
                        s_tc <= s_t1;
                    end if;
                    s_sq <= to_unsigned(3, 5);
                when 3 =>                              -- sin(tc)
                    if s_tc(10) = '1' then s_sa <= not s_tc(9 downto 0); else s_sa <= s_tc(9 downto 0); end if;
                    s_sneg_a <= s_tc(11);
                    s_sq <= to_unsigned(4, 5);
                when 4 | 5 | 6 | 9 =>
                    s_sq <= s_sq + 1;
                when 7 =>
                    s_smag <= s_sin_r(14 downto 0);
                    s_sng <= s_sneg;
                    s_sq <= to_unsigned(8, 5);
                when 8 =>                              -- cos(tc) = sin(tc + 1024)
                    if s_tc(10) = '0' then s_sa <= not s_tc(9 downto 0); else s_sa <= s_tc(9 downto 0); end if;
                    s_sneg_a <= s_tc(11) xor s_tc(10);
                    s_sq <= to_unsigned(9, 5);
                -- Resolution scaling: the cell is in virtual units, so the
                -- per-pixel steps are |c|, |s| = |cos|, |sin| x inv_x (1/cell
                -- x the x step 1 / 1.5 / 2) and the per-line steps |cos|,
                -- |sin| x inv_y (1/cell x the y step per field line, Q.8).
                -- At 1080p inv_x = inv_y = 1/cell: the old single pair.
                when 10 =>                             -- inv_y = inv * ys
                    s_ma <= resize(unsigned'(s_inv_r & "000"), 20);
                    s_mb <= resize(s_ys, 20);
                    s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                    s_sq <= to_unsigned(11, 5);
                when 11 =>
                    if v_go then
                        s_invy <= s_macc(27 downto 8);
                        s_ma <= s_invx;                              -- |cos| * inv_x
                        s_mb <= resize(s_sin_r(14 downto 0), 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(12, 5);
                    end if;
                when 12 =>
                    if v_go then
                        if s_sneg = '1' then s_ct <= v_pn; else s_ct <= v_p; end if;
                        s_ma <= s_invx;                              -- |sin| * inv_x
                        s_mb <= resize(s_smag, 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(13, 5);
                    end if;
                when 13 =>
                    if v_go then
                        if s_sng = '1' then s_st <= v_pn; else s_st <= v_p; end if;
                        s_ma <= s_invy;                              -- |cos| * inv_y
                        s_mb <= resize(s_sin_r(14 downto 0), 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(14, 5);
                    end if;
                when 14 =>
                    if v_go then
                        if s_sneg = '1' then s_cy <= v_pn; else s_cy <= v_p; end if;
                        s_ma <= s_invy;                              -- |sin| * inv_y
                        s_mb <= resize(s_smag, 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(15, 5);
                    end if;
                when 15 =>
                    if v_go then
                        if s_sng = '1' then s_sy <= v_pn; else s_sy <= v_p; end if;
                        s_ma <= s_ct;                                -- wc * c (x)
                        s_mb <= resize(s_wc, 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(16, 5);
                    end if;
                when 16 =>
                    if v_go then
                        s_u0t <= x"80000" - s_macc(19 downto 0);
                        s_ma <= s_sy;                                -- hc * s (y)
                        s_mb <= resize(s_hc, 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(17, 5);
                    end if;
                when 17 =>
                    if v_go then
                        s_u0t <= s_u0t - s_macc(19 downto 0);
                        s_ma <= s_st;                                -- wc * s (x)
                        s_mb <= resize(s_wc, 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(18, 5);
                    end if;
                when 18 =>
                    if v_go then
                        s_v0t <= x"80000" + s_macc(19 downto 0);
                        s_ma <= s_cy;                                -- hc * c (y)
                        s_mb <= resize(s_hc, 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(19, 5);
                    end if;
                when 19 =>
                    if v_go then
                        s_v0t <= s_v0t - s_macc(19 downto 0);
                        s_sq <= to_unsigned(20, 5);
                    end if;
                when 20 =>                             -- next screen, or spin rate
                    if s_scr = '0' then
                        s_scr <= '1';
                        s_sq <= to_unsigned(2, 5);
                    else
                        s_ma <= resize(s_ang_g(13 downto 4), 20);
                        s_mb <= resize(s_ang_g(13 downto 4), 20);
                        s_macc <= (others => '0'); s_mcnt <= to_unsigned(20, 5);
                        s_sq <= to_unsigned(21, 5);
                    end if;
                when 21 =>
                    if v_go then
                        s_rate <= to_unsigned(64, 13) + resize(s_macc(19 downto 8), 13);
                        s_sq <= (others => '0');
                    end if;
                when others =>
                    s_sq <= (others => '0');
            end case;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_sq <= to_unsigned(1, 5);
            end if;
        end if;
    end process p_seq;

    ----------------------------------------------------------------------
    -- screen DDAs, stage 1 (aligned with s_in_*)
    ----------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            -- results land in vblank: steps and row starts per screen
            -- (per-pixel steps c/s, per-line steps cy/sy: virtual units)
            if s_sq = 20 and s_scr = '0' then
                s_c1 <= s_ct; s_s1 <= s_st; s_c1y <= s_cy; s_s1y <= s_sy;
                s_ur1 <= s_u0t; s_vr1 <= s_v0t;
            elsif s_prev_avid = '1' and data_in.avid = '0' then
                s_ur1 <= s_ur1 + s_s1y; s_vr1 <= s_vr1 + s_c1y;
            end if;
            if s_sq = 20 and s_scr = '1' then
                s_c2 <= s_ct; s_s2 <= s_st; s_c2y <= s_cy; s_s2y <= s_sy;
                s_ur2 <= s_u0t; s_vr2 <= s_v0t;
            elsif s_prev_avid = '1' and data_in.avid = '0' then
                s_ur2 <= s_ur2 + s_s2y; s_vr2 <= s_vr2 + s_c2y;
            end if;
            if data_in.avid = '1' then
                if s_prev_avid = '0' then
                    s_u1 <= s_ur1; s_v1 <= s_vr1; s_u2 <= s_ur2; s_v2 <= s_vr2;
                else
                    s_u1 <= s_u1 + s_c1; s_v1 <= s_v1 - s_s1;
                    s_u2 <= s_u2 + s_c2; s_v2 <= s_v2 - s_s2;
                end if;
            end if;
        end if;
    end process p_dda;

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cs : unsigned(14 downto 0);
        variable v_ds : unsigned(7 downto 0);
        variable v_ink : std_logic;
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_y : signed(11 downto 0);
        variable v_wu, v_wv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: distance from the cell centre on each axis; black-relative luma
            s_fu1 <= f_fold(s_u1(19 downto 12)); s_fv1 <= f_fold(s_v1(19 downto 12));
            s_fu2 <= f_fold(s_u2(19 downto 12)); s_fv2 <= f_fold(s_v2(19 downto 12));
            if s_in_y < 64 then
                s_bd(2) <= (others => '0');
            elsif s_in_y > 940 then
                s_bd(2) <= to_unsigned(876, 10);
            else
                s_bd(2) <= s_in_y - 64;
            end if;
            s_ud(2) <= s_in_u; s_vd(2) <= s_in_v;
            -- 3: square parts; gain product
            s_pu1 <= f_sqp(s_fu1); s_pv1 <= f_sqp(s_fv1);
            s_pu2 <= f_sqp(s_fu2); s_pv2 <= f_sqp(s_fv2);
            s_au1 <= s_fu1; s_av1 <= s_fv1; s_au2 <= s_fu2; s_av2 <= s_fv2;
            s_yg3 <= s_bd(2) * s_gain;
            -- 4: squares
            s_qu1 <= f_sqs(s_pu1); s_qv1 <= f_sqs(s_pv1);
            s_qu2 <= f_sqs(s_pu2); s_qv2 <= f_sqs(s_pv2);
            s_bu1 <= s_au1; s_bv1 <= s_av1; s_bu2 <= s_au2; s_bv2 <= s_av2;
            s_yg4 <= s_yg3(17 downto 7);
            -- 5: every shape's distance, normalised to 0..~1020
            -- (the sums skip the operands' shared low bits: a square's bit 1
            -- is always 0 and the folded distances are all odd, so yosys
            -- merges those FFs and an adder fed the SAME net on both inputs
            -- livelocks the stock router1.  bit 1 = 0 means the low 2 bits
            -- never carry; two equal bit 0s add as one carry-in)
            v_cs := resize(s_qu1(15 downto 2), 15) + resize(s_qv1(15 downto 2), 15);
            s_dc1 <= v_cs(14 downto 5);
            v_ds := resize(s_bu1(7 downto 1), 8) + resize(s_bv1(7 downto 1), 8) + resize(s_bu1(0 downto 0), 8);
            s_dd1 <= v_ds & "00";
            if s_bu1 > s_bv1 then s_ds1 <= s_bu1 & "00"; else s_ds1 <= s_bv1 & "00"; end if;
            s_dl1 <= s_bv1 & "00";
            v_cs := resize(s_qu2(15 downto 2), 15) + resize(s_qv2(15 downto 2), 15);
            s_dc2 <= v_cs(14 downto 5);
            v_ds := resize(s_bu2(7 downto 1), 8) + resize(s_bv2(7 downto 1), 8) + resize(s_bu2(0 downto 0), 8);
            s_dd2 <= v_ds & "00";
            if s_bu2 > s_bv2 then s_ds2 <= s_bu2 & "00"; else s_ds2 <= s_bv2 & "00"; end if;
            s_dl2 <= s_bv2 & "00";
            s_yg5 <= s_yg4;
            -- 6: the shape
            case to_integer(s_dot) is
                when 0 => s_dn1 <= s_dc1; s_dn2 <= s_dc2;
                when 1 => s_dn1 <= s_dd1; s_dn2 <= s_dd2;
                when 2 => s_dn1 <= s_ds1; s_dn2 <= s_ds2;
                when others => s_dn1 <= s_dl1; s_dn2 <= s_dl2;
            end case;
            s_yg6 <= s_yg5;
            -- 7: ink where the luma is below the distance (dark -> big dots)
            v_ink := '0';
            if s_yg6 < s_dn1 then v_ink := '1'; end if;
            if s_sw_ros = '1' and s_yg6 < s_dn2 then v_ink := '1'; end if;
            s_ink7 <= v_ink xor s_sw_inv;
            -- 8: operands: halftone a (ink or paper, black-relative and on a
            -- 0..1023 scale) and picture b
            if s_ink7 = '1' then
                s_a8 <= s_ink_a; s_an8 <= s_ink_an; s_ac8 <= s_ink_ac; s_alo8 <= s_ink_lo;
            else
                s_a8 <= to_unsigned(876, 10); s_an8 <= to_unsigned(1023, 10);
                s_ac8 <= (others => '0'); s_alo8 <= '0';
            end if;
            s_b8 <= s_bd(7);
            s_bc8 <= to_unsigned(876, 10) - s_bd(7);
            -- 9: products
            s_p9 <= s_an8 * s_b8;
            s_q9 <= s_ac8 * s_bc8;
            s_sum9 <= resize(s_a8, 11) + resize(s_b8, 11);
            s_dif9 <= signed(resize(s_a8, 11)) - signed(resize(s_b8, 11));
            s_alo9 <= s_alo8;
            if s_b8 < 438 then s_blo9 <= '1'; else s_blo9 <= '0'; end if;
            -- 10: candidates
            s_c_add10 <= signed(resize(s_sum9, 12)) - 438;
            s_c_scr10 <= signed(resize(s_sum9, 12)) - signed(resize(s_p9(19 downto 10), 12));
            s_c_mul10 <= signed(resize(s_p9(19 downto 10), 12));
            if s_dif9 < 0 then s_c_dif10 <= -resize(s_dif9, 12); else s_c_dif10 <= resize(s_dif9, 12); end if;
            s_c_lo10 <= signed(resize(s_p9(19 downto 9), 12));
            s_c_hi10 <= to_signed(876, 12) - signed(resize(s_q9(19 downto 9), 12));
            s_alo10 <= s_alo9; s_blo10 <= s_blo9;
            -- 11: select
            case to_integer(s_mode) is
                when 0 => s_sel11 <= s_c_add10;
                when 1 => s_sel11 <= s_c_scr10;
                when 2 => s_sel11 <= s_c_mul10;
                when 3 => s_sel11 <= s_c_dif10;
                when 4 => if s_alo10 = '1' then s_sel11 <= s_c_lo10; else s_sel11 <= s_c_hi10; end if;
                when others => if s_blo10 = '1' then s_sel11 <= s_c_lo10; else s_sel11 <= s_c_hi10; end if;
            end case;
            -- 12: clamp to the video range
            if s_sel11 < 0 then
                s_t12 <= (others => '0');
            elsif s_sel11 > 876 then
                s_t12 <= to_unsigned(876, 10);
            else
                s_t12 <= unsigned(s_sel11(9 downto 0));
            end if;
            -- 13: differences toward the print (luma) and its colour
            s_dy13 <= signed(resize(s_t12, 11)) - signed(resize(s_bd(12), 11));
            if s_inkd(12) = '1' then
                if s_sw_inkc = '1' then
                    s_du13 <= signed(resize(s_ink_u, 11)) - signed(resize(s_ud(12), 11));
                    s_dv13 <= signed(resize(s_ink_v, 11)) - signed(resize(s_vd(12), 11));
                else
                    s_du13 <= s_ink_ou; s_dv13 <= s_ink_ov;
                end if;
            else
                if s_sw_inkc = '1' then
                    s_du13 <= 512 - signed(resize(s_ud(12), 11));
                    s_dv13 <= 512 - signed(resize(s_vd(12), 11));
                else
                    s_du13 <= (others => '0'); s_dv13 <= (others => '0');
                end if;
            end if;
            -- 14: mix products
            s_py14 <= s_dy13 * signed(resize(s_mix, 10));
            s_pu14 <= s_du13 * signed(resize(s_mix, 10));
            s_pv14 <= s_dv13 * signed(resize(s_mix, 10));
            -- 15: apply (luma stays inside 64..940 by construction)
            v_y := signed(resize(s_bd(14), 12)) + resize(shift_right(s_py14, 8), 12) + 64;
            s_y15 <= unsigned(v_y(9 downto 0));
            s_u15 <= f_clamp10(signed(resize(s_ud(14), 13)) + resize(shift_right(s_pu14, 8), 13));
            s_v15 <= f_clamp10(signed(resize(s_vd(14), 13)) + resize(shift_right(s_pv14, 8), 13));
            -- 16: neutral outside avid (the smoothing's edge taps read
            -- black); colour into the Register ring
            if s_avid_sr(14) = '1' then
                s_y16 <= s_y15; v_wu := s_u15; v_wv := s_v15;
            else
                s_y16 <= to_unsigned(64, 10);
                v_wu := to_unsigned(512, 10); v_wv := to_unsigned(512, 10);
            end if;
            r_ring_u(to_integer(s_wp)) <= v_wu;
            r_ring_v(to_integer(s_wp)) <= v_wv;
            s_wp <= s_wp + 1;
            s_ra <= s_wp - s_sh;
            if s_avid_sr(13) = '1' and s_avid_sr(14) = '0' then     -- next pixel starts a line
                s_ex <= (others => '0'); s_sh <= (others => '0');
            elsif s_ex < C_EDGE then
                s_ex <= s_ex + 1;
            elsif s_sh < s_delta then
                s_sh <= s_sh + 1;
            elsif s_sh > s_delta then
                s_sh <= s_delta;
            end if;
            -- 17: ring read (delta = 0: the pixel written last clock)
            s_rdu <= r_ring_u(to_integer(s_ra));
            s_rdv <= r_ring_v(to_integer(s_ra));
            s_y17 <= s_y16;
            -- 18: fabric copy of the EBR data
            s_u18 <= s_rdu; s_v18 <= s_rdv; s_y18 <= s_y17;
            -- 19..20: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y18; s_g2y <= s_g1y;
            s_g1u <= s_u18; s_g2u <= s_g1u;
            s_g1v <= s_v18; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y18, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u18, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v18, 12);
            s_y20 <= v_sy(11 downto 2); s_u20 <= v_su(11 downto 2); s_v20 <= v_sv(11 downto 2);
            -- 21: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            v_d := s_y20 - 64;
            if s_avid_sr(19) = '1' then
                s_y21 <= s_y20 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;

            for i in 3 to 14 loop s_bd(i) <= s_bd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            s_inkd(8) <= s_ink7;
            for i in 9 to 12 loop s_inkd(i) <= s_inkd(i - 1); end loop;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y21);
    s_io_align_in.u       <= std_logic_vector(s_u21);
    s_io_align_in.v       <= std_logic_vector(s_v21);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (21 -> 24).
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

end architecture screenprint;
