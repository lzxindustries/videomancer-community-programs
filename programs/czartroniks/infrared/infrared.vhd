-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Infrared v1.0.0: catalogue combo #10 (seed 20260912) -- perspective
-- keystone / corner-pin + false colour (thermal-camera palettes).
--
--   K1 Keystone  bipolar; rows scale linearly with height about the centre
--   K2 Skew      bipolar parallelogram shear, up to +-half the width
--   K3 Zoom      overall horizontal scale 0.25x .. 4x (log, 256 steps)
--   K4 Palette   Thermal / Ironbow / Plasma / Rainbow / Ice / Night Vision /
--                White Hot / X-Ray (8 zones, 256 entries each)
--   K5 Range     luma gain into the palette, 0 .. 4x (unity at 25%)
--   K6 Offset    luma bias into the palette, +-one whole ramp
--
--   S7  Rows     palette offset drifts with the row (heat rises down the frame)
--   S8  Iso      black isotherm lines every eighth of the ramp
--   S9  Cycle    off: P12 is a heat position (clamped ramp);
--                on:  P12 is the palette cycling speed (wrapped ramp)
--   S10 Luma     palette replaces luma too / only chroma comes from the palette
--   S11 Ramp     normal / inverted brightness-to-palette mapping
--   P12 Heat     Cycle off: pushes the whole scene up the ramp (glided);
--                Cycle on: cycling speed (squared curve, 0.5 s a lap at 100%)
--
-- Keystone: kinescope's address engine (u = cx + (x-cx)*k_row + shift_row).
-- False colour: idx = clamp-or-wrap(+-(y-64)*range + offset + row + heat) into
-- an 8 x 256 entry palette ROM (EBR; Y8 Cr8 | Cb8, stored U/V-swapped).  The
-- palette supplies all output chroma, so only luma is line-buffered.
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, field-latched
-- width; damier/aurora) also drives the keystone DDA; one latency (16 clocks)
-- in all modes; outputs gated to neutral outside the generated avid and black
-- outside the keystoned source; output conditioning ([1 2 1] on Y/U/V + luma
-- scaled to white ~775: the HD HDMI bar fix from damier v0.4.4) because the
-- cycling wrap drags a full-swing edge across the picture every frame.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture infrared of program_top is
    -- data_in -> data_out latency is 16 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;

    constant C_LAT : integer := 16;

    -- BEGIN GENERATED TABLES (tools/gen_palettes.py)
    -- K4 zones: Thermal, Ironbow, Plasma, Rainbow, Ice, Night Vision, White Hot, X-Ray
    type t_pal16 is array (0 to 2047) of std_logic_vector(15 downto 0);
    type t_pal8  is array (0 to 2047) of std_logic_vector(7 downto 0);
    constant C_PAL_YC : t_pal16 := (
        x"1080", x"1080", x"1080", x"1080", x"1180", x"1180", x"1180", x"1180", x"1280", x"1280", x"1280", x"1280", x"1381", x"1381", x"1381", x"1481",
        x"1481", x"1581", x"1581", x"1581", x"1681", x"1681", x"1681", x"1681", x"1781", x"1781", x"1781", x"1781", x"1882", x"1882", x"1882", x"1882",
        x"1982", x"1982", x"1982", x"1A82", x"1A82", x"1B82", x"1B82", x"1B82", x"1C83", x"1C83", x"1C83", x"1D84", x"1D84", x"1E85", x"1E85", x"1F86",
        x"2087", x"2188", x"2188", x"228A", x"238A", x"248C", x"248C", x"258E", x"268E", x"2790", x"2790", x"2892", x"2992", x"2A94", x"2A94", x"2B95",
        x"2C96", x"2D97", x"2D98", x"2E98", x"2F9A", x"309A", x"309C", x"319C", x"329E", x"339E", x"33A0", x"34A0", x"35A2", x"36A2", x"36A3", x"37A4",
        x"38A5", x"39A6", x"39A6", x"3AA8", x"3BA8", x"3CAA", x"3CAB", x"3CAC", x"3DAD", x"3DAE", x"3DAF", x"3DB0", x"3EB1", x"3EB2", x"3EB2", x"3EB4",
        x"3FB4", x"3FB6", x"3FB6", x"3FB8", x"40B8", x"40BA", x"40BA", x"40BC", x"41BC", x"41BE", x"41BF", x"41C0", x"42C1", x"42C2", x"42C3", x"42C4",
        x"43C5", x"43C6", x"43C7", x"43C8", x"44C9", x"44CA", x"44CC", x"44CC", x"45CE", x"45CE", x"45CF", x"45D0", x"46D1", x"46D2", x"46D2", x"47D3",
        x"48D4", x"49D5", x"4AD5", x"4BD5", x"4DD4", x"4ED4", x"50D4", x"51D4", x"52D3", x"53D3", x"55D3", x"56D3", x"57D2", x"58D2", x"5AD2", x"5BD2",
        x"5DD2", x"5ED2", x"60D2", x"61D2", x"62D2", x"63D2", x"65D2", x"66D2", x"67D1", x"68D1", x"6AD1", x"6BD1", x"6CD0", x"6DD0", x"6FD0", x"70D0",
        x"72D0", x"73D0", x"75D0", x"76D0", x"77CF", x"78CF", x"7ACF", x"7BCF", x"7CCE", x"7DCE", x"7FCE", x"81CC", x"82CC", x"84CA", x"86C9", x"87C8",
        x"89C7", x"8AC6", x"8CC4", x"8DC4", x"8FC2", x"90C2", x"92C0", x"94BF", x"95BE", x"97BD", x"99BC", x"9ABA", x"9CBA", x"9DB8", x"9FB7", x"A0B6",
        x"A2B5", x"A3B4", x"A5B2", x"A6B2", x"A8B0", x"A9B0", x"ABAE", x"ACAD", x"AEAC", x"AFAB", x"B1AA", x"B2A8", x"B4A8", x"B5A6", x"B7A5", x"B9A4",
        x"BAA3", x"BCA2", x"BEA0", x"BFA0", x"C09E", x"C19E", x"C29D", x"C49C", x"C49B", x"C69A", x"C69A", x"C899", x"C898", x"CA97", x"CA97", x"CC96",
        x"CC95", x"CE94", x"CE94", x"D093", x"D092", x"D291", x"D291", x"D490", x"D490", x"D68F", x"D78E", x"D88E", x"D98C", x"DA8C", x"DB8B", x"DC8A",
        x"DD8A", x"DE89", x"DE89", x"E088", x"E087", x"E286", x"E386", x"E484", x"E584", x"E682", x"E882", x"E881", x"EA81", x"EA80", x"EB80", x"EB80",
        x"117F", x"127F", x"127F", x"137F", x"147F", x"147F", x"157E", x"157E", x"167E", x"167E", x"177E", x"187E", x"187E", x"197E", x"197E", x"1A7E",
        x"1A7D", x"1B7D", x"1C7D", x"1C7D", x"1D7D", x"1D7D", x"1E7D", x"1E7D", x"1F7D", x"207C", x"207D", x"217E", x"217E", x"227F", x"2380", x"2381",
        x"2482", x"2483", x"2583", x"2684", x"2685", x"2786", x"2787", x"2887", x"2988", x"2989", x"2A8A", x"2A8B", x"2B8B", x"2C8C", x"2C8D", x"2D8E",
        x"2E8F", x"2E8F", x"2F90", x"2F91", x"3092", x"3193", x"3193", x"3294", x"3295", x"3396", x"3497", x"3498", x"3598", x"3599", x"369A", x"379B",
        x"379C", x"389D", x"399E", x"399E", x"3A9F", x"3BA0", x"3BA1", x"3CA2", x"3DA3", x"3EA4", x"3EA5", x"3FA6", x"40A7", x"40A8", x"41A9", x"42AA",
        x"42AB", x"43AC", x"44AD", x"45AE", x"45AF", x"46B0", x"47B1", x"47B2", x"48B3", x"49B4", x"49B5", x"4AB6", x"4BB7", x"4BB8", x"4CB9", x"4DBA",
        x"4EBB", x"4EBC", x"4FBC", x"50BD", x"50BE", x"51BF", x"52C0", x"52C1", x"53C2", x"54C3", x"55C4", x"55C5", x"56C5", x"57C6", x"57C6", x"58C6",
        x"59C6", x"5AC7", x"5AC7", x"5BC7", x"5CC7", x"5DC8", x"5DC8", x"5EC8", x"5FC8", x"60C9", x"60C9", x"61C9", x"62C9", x"62CA", x"63CA", x"64CA",
        x"65CA", x"65CB", x"66CB", x"67CB", x"68CB", x"68CC", x"69CC", x"6ACC", x"6BCC", x"6BCD", x"6CCD", x"6DCD", x"6ECD", x"6ECE", x"6FCE", x"70CE",
        x"70CE", x"71CF", x"72CF", x"73CF", x"73CF", x"74CF", x"76CE", x"77CE", x"78CD", x"79CD", x"7ACC", x"7BCC", x"7CCB", x"7DCA", x"7ECA", x"7FC9",
        x"80C9", x"81C8", x"82C8", x"83C7", x"84C7", x"85C6", x"86C6", x"87C5", x"88C5", x"89C4", x"8AC4", x"8BC3", x"8CC3", x"8DC2", x"8EC1", x"8FC1",
        x"90C0", x"91C0", x"92BF", x"93BF", x"94BE", x"96BE", x"97BD", x"98BD", x"99BC", x"9ABB", x"9BBA", x"9CB9", x"9DB9", x"9FB8", x"A0B7", x"A1B6",
        x"A2B5", x"A3B4", x"A5B3", x"A6B3", x"A7B2", x"A8B1", x"A9B0", x"AAAF", x"ACAE", x"ADAD", x"AEAC", x"AFAC", x"B0AB", x"B2AA", x"B3A9", x"B4A8",
        x"B5A7", x"B6A6", x"B7A6", x"B9A5", x"BAA4", x"BBA3", x"BCA2", x"BDA1", x"BFA0", x"C0A0", x"C19F", x"C29E", x"C39D", x"C49C", x"C59B", x"C79B",
        x"C89A", x"C999", x"CA98", x"CB97", x"CC97", x"CD96", x"CE95", x"CF94", x"D093", x"D293", x"D392", x"D491", x"D590", x"D68F", x"D78F", x"D88E",
        x"D98D", x"DA8C", x"DB8B", x"DD8B", x"DE8A", x"DF89", x"E088", x"E187", x"E287", x"E386", x"E485", x"E584", x"E683", x"E883", x"E982", x"EA81",
        x"2379", x"2379", x"2379", x"247A", x"247B", x"257C", x"257C", x"267D", x"267E", x"277F", x"277F", x"2880", x"2881", x"2982", x"2982", x"2A83",
        x"2A84", x"2B85", x"2B85", x"2C86", x"2C87", x"2D88", x"2D88", x"2E89", x"2E8A", x"2F8B", x"2F8C", x"308C", x"308E", x"318E", x"318F", x"3290",
        x"3291", x"3392", x"3392", x"3493", x"3494", x"3595", x"3595", x"3596", x"3697", x"3698", x"3698", x"3799", x"389A", x"399B", x"399B", x"3A9C",
        x"3A9D", x"3B9E", x"3B9E", x"3C9F", x"3CA0", x"3DA1", x"3DA1", x"3DA2", x"3EA3", x"3EA4", x"3EA4", x"3FA6", x"3FA6", x"40A8", x"40A8", x"41A9",
        x"42A9", x"43AA", x"43AA", x"44AA", x"45AB", x"46AB", x"46AB", x"47AB", x"48AC", x"49AC", x"49AC", x"4AAC", x"4BAC", x"4CAC", x"4CAC", x"4DAC",
        x"4EAD", x"4FAD", x"50AD", x"50AD", x"52AE", x"52AE", x"53AE", x"54AE", x"55AF", x"56AF", x"56AF", x"57AF", x"58B0", x"59B0", x"59B0", x"5AB0",
        x"5BB0", x"5CB0", x"5CB0", x"5DB0", x"5EB1", x"5FB1", x"5FB1", x"60B1", x"61B2", x"62B2", x"62B2", x"64B2", x"64B3", x"66B3", x"66B3", x"67B3",
        x"68B4", x"69B4", x"69B4", x"6AB4", x"6BB5", x"6CB5", x"6CB5", x"6DB5", x"6EB5", x"6FB5", x"6FB5", x"70B5", x"71B6", x"72B6", x"72B6", x"73B6",
        x"74B6", x"75B6", x"75B6", x"76B6", x"77B6", x"78B6", x"78B6", x"79B6", x"7AB6", x"7BB6", x"7BB6", x"7CB6", x"7DB6", x"7EB6", x"7EB6", x"7FB6",
        x"7FB5", x"80B5", x"80B5", x"81B5", x"82B5", x"83B5", x"83B5", x"84B5", x"85B4", x"86B4", x"86B4", x"87B4", x"88B4", x"89B4", x"89B4", x"8AB4",
        x"8BB4", x"8CB4", x"8CB4", x"8DB4", x"8EB4", x"8FB4", x"8FB4", x"90B4", x"91B3", x"92B3", x"92B3", x"93B3", x"93B3", x"94B3", x"94B3", x"95B3",
        x"96B2", x"97B2", x"97B2", x"98B2", x"99B2", x"9AB2", x"9AB2", x"9BB2", x"9CB2", x"9DB2", x"9DB2", x"9EB2", x"9FB1", x"A0B1", x"A0B1", x"A1B0",
        x"A2B0", x"A3AF", x"A3AF", x"A4AE", x"A5AE", x"A6AD", x"A6AD", x"A7AC", x"A8AC", x"A9AB", x"A9AB", x"AAAA", x"ABA9", x"ACA8", x"ACA8", x"ADA7",
        x"ADA7", x"AEA6", x"AEA6", x"AFA5", x"B0A4", x"B1A3", x"B1A3", x"B2A2", x"B3A2", x"B4A1", x"B4A1", x"B5A0", x"B69F", x"B79E", x"B79E", x"B89D",
        x"B99D", x"BA9C", x"BA9C", x"BB9B", x"BC9B", x"BD9A", x"BD9A", x"BE99", x"BF98", x"C097", x"C097", x"C196", x"C296", x"C395", x"C395", x"C494",
        x"C594", x"C693", x"C693", x"C792", x"C791", x"C890", x"C890", x"C98F", x"CA8F", x"CB8E", x"CB8E", x"CC8D", x"CD8C", x"CE8B", x"CE8B", x"CE8B",
        x"1080", x"1180", x"1281", x"1281", x"1381", x"1482", x"1582", x"1583", x"1683", x"1783", x"1884", x"1984", x"1984", x"1A85", x"1B85", x"1C86",
        x"1C86", x"1D86", x"1E87", x"1F87", x"2087", x"2088", x"2188", x"2289", x"2389", x"2389", x"2489", x"2488", x"2587", x"2586", x"2585", x"2584",
        x"2683", x"2682", x"2681", x"2780", x"277F", x"277E", x"287D", x"287C", x"287B", x"287A", x"2979", x"2978", x"2977", x"2A76", x"2A75", x"2A74",
        x"2A73", x"2B72", x"2B71", x"2B70", x"2C6F", x"2C6E", x"2C6D", x"2D6C", x"2D6B", x"2F69", x"3168", x"3466", x"3664", x"3962", x"3B61", x"3D5F",
        x"405D", x"425C", x"445A", x"4758", x"4956", x"4C55", x"4E53", x"5051", x"534F", x"554E", x"574C", x"5A4A", x"5C48", x"5F47", x"6145", x"6343",
        x"6642", x"6840", x"6A3E", x"6D3C", x"6F3B", x"7239", x"7437", x"7635", x"7934", x"7B32", x"7D30", x"802E", x"822D", x"822D", x"822D", x"822D",
        x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"822D", x"832C", x"832C", x"832C", x"832C",
        x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C", x"832C",
        x"842D", x"8630", x"8832", x"8A35", x"8B37", x"8D3A", x"8F3C", x"913F", x"9241", x"9444", x"9646", x"9849", x"994B", x"9B4E", x"9D50", x"9F53",
        x"A056", x"A258", x"A45B", x"A65D", x"A760", x"A962", x"AB65", x"AD67", x"AE6A", x"B06C", x"B26F", x"B371", x"B574", x"B776", x"B979", x"BA7B",
        x"BC7E", x"BE81", x"C083", x"C186", x"C188", x"BF89", x"BE8B", x"BC8D", x"BA8F", x"B990", x"B792", x"B694", x"B496", x"B398", x"B199", x"B09B",
        x"AE9D", x"AD9F", x"ABA0", x"AAA2", x"A8A4", x"A7A6", x"A5A8", x"A4A9", x"A2AB", x"A1AD", x"9FAF", x"9DB1", x"9CB2", x"9AB4", x"99B6", x"97B8",
        x"96B9", x"94BB", x"93BD", x"91BF", x"90C1", x"8EC2", x"8DC4", x"8BC6", x"8AC7", x"88C8", x"86C9", x"85CB", x"83CC", x"81CD", x"80CE", x"7ECF",
        x"7CD1", x"7BD2", x"79D3", x"77D4", x"76D5", x"74D7", x"73D8", x"71D9", x"6FDA", x"6EDB", x"6CDD", x"6ADE", x"69DF", x"67E0", x"65E1", x"64E3",
        x"62E4", x"61E5", x"5FE6", x"5DE7", x"5CE9", x"5AEA", x"5CE8", x"61E4", x"67E0", x"6CDD", x"72D9", x"77D5", x"7CD1", x"82CD", x"87C9", x"8DC5",
        x"92C1", x"97BD", x"9DB9", x"A2B5", x"A8B1", x"ADAD", x"B3A9", x"B8A5", x"BDA1", x"C39D", x"C899", x"CE95", x"D391", x"D88E", x"DE8A", x"E386",
        x"1080", x"1080", x"1080", x"107F", x"117F", x"117E", x"117E", x"127E", x"127E", x"137E", x"137E", x"137D", x"147D", x"147C", x"147C", x"147C",
        x"157B", x"157B", x"157B", x"167B", x"167A", x"177A", x"177A", x"187A", x"1879", x"1979", x"1979", x"1979", x"1A78", x"1A78", x"1A78", x"1B78",
        x"1B77", x"1C77", x"1C77", x"1C77", x"1D76", x"1D76", x"1D76", x"1E76", x"1E75", x"1F75", x"1F75", x"1F75", x"2074", x"2074", x"2074", x"2173",
        x"2173", x"2272", x"2272", x"2372", x"2371", x"2471", x"2471", x"2670", x"266F", x"286E", x"286E", x"296D", x"2A6D", x"2B6C", x"2C6C", x"2C6B",
        x"2E6A", x"2E69", x"2F69", x"3068", x"3167", x"3266", x"3266", x"3465", x"3464", x"3663", x"3663", x"3862", x"3862", x"3A61", x"3A61", x"3C60",
        x"3C5F", x"3E5E", x"3E5E", x"3F5D", x"405C", x"415B", x"425B", x"425A", x"445A", x"4459", x"4659", x"4658", x"4857", x"4856", x"4956", x"4A55",
        x"4B55", x"4C54", x"4C54", x"4E53", x"4E52", x"5051", x"5151", x"5250", x"5350", x"544F", x"564F", x"584F", x"594E", x"5B4E", x"5D4E", x"5E4D",
        x"5F4D", x"604C", x"624C", x"644C", x"654B", x"674B", x"694B", x"6A4A", x"6C4A", x"6D49", x"6F49", x"7049", x"7248", x"7348", x"7548", x"7647",
        x"7747", x"7846", x"7A46", x"7C46", x"7D45", x"7F45", x"8145", x"8245", x"8444", x"8544", x"8744", x"8843", x"8A43", x"8B42", x"8D42", x"8E42",
        x"9041", x"9141", x"9341", x"9440", x"9640", x"973F", x"993F", x"9A3F", x"9C3F", x"9D3F", x"9E3F", x"A040", x"A041", x"A242", x"A342", x"A444",
        x"A544", x"A646", x"A846", x"A948", x"AA48", x"AB4A", x"AC4A", x"AE4B", x"AE4C", x"B04D", x"B14E", x"B24E", x"B350", x"B450", x"B651", x"B752",
        x"B853", x"B954", x"BA54", x"BC56", x"BC56", x"BE58", x"BF58", x"C05A", x"C15A", x"C25C", x"C45C", x"C55D", x"C65E", x"C75F", x"C860", x"CA60",
        x"CA62", x"CC62", x"CD63", x"CE64", x"CF65", x"D066", x"D266", x"D368", x"D468", x"D56A", x"D66A", x"D76B", x"D76B", x"D86C", x"D86C", x"D96D",
        x"D96D", x"DA6E", x"DA6E", x"DA6E", x"DB6F", x"DB6F", x"DB6F", x"DC70", x"DC70", x"DD71", x"DD71", x"DE71", x"DE72", x"DF72", x"DF72", x"DF73",
        x"E073", x"E074", x"E074", x"E075", x"E175", x"E176", x"E176", x"E276", x"E277", x"E377", x"E377", x"E478", x"E478", x"E579", x"E579", x"E57A",
        x"E67A", x"E67B", x"E67B", x"E67B", x"E77C", x"E77C", x"E77C", x"E87D", x"E87D", x"E97E", x"E97E", x"EA7F", x"EA7F", x"EB80", x"EB80", x"EB80",
        x"1080", x"1080", x"117F", x"117F", x"127F", x"127F", x"127E", x"137E", x"137E", x"147E", x"147D", x"147D", x"157D", x"157D", x"167C", x"167C",
        x"167C", x"177C", x"177B", x"187B", x"187B", x"187A", x"197A", x"197A", x"1A7A", x"1A79", x"1A79", x"1B79", x"1B79", x"1C78", x"1C78", x"1C78",
        x"1D78", x"1D77", x"1E77", x"1E77", x"1F77", x"1F76", x"1F76", x"2076", x"2075", x"2175", x"2175", x"2175", x"2274", x"2274", x"2374", x"2374",
        x"2373", x"2473", x"2473", x"2573", x"2572", x"2572", x"2672", x"2672", x"2771", x"2771", x"2771", x"2871", x"2870", x"2970", x"2970", x"296F",
        x"2A6F", x"2A6F", x"2B6F", x"2B6E", x"2B6E", x"2C6E", x"2C6E", x"2D6D", x"2D6D", x"2D6D", x"2E6D", x"2E6C", x"2F6C", x"2F6C", x"2F6C", x"306B",
        x"306B", x"316B", x"316A", x"316A", x"326A", x"326A", x"3369", x"3369", x"3369", x"3469", x"3468", x"3568", x"3568", x"3568", x"3667", x"3667",
        x"3767", x"3767", x"3866", x"3866", x"3866", x"3965", x"3965", x"3A65", x"3B64", x"3C64", x"3D64", x"3E63", x"3F63", x"4062", x"4162", x"4262",
        x"4361", x"4461", x"4560", x"4660", x"4760", x"485F", x"495F", x"4A5F", x"4B5E", x"4C5E", x"4D5D", x"4E5D", x"4F5D", x"505C", x"515C", x"525B",
        x"535B", x"545B", x"555A", x"565A", x"5759", x"5859", x"5859", x"5958", x"5A58", x"5B57", x"5C57", x"5D57", x"5E56", x"5F56", x"6055", x"6155",
        x"6255", x"6354", x"6454", x"6554", x"6653", x"6753", x"6852", x"6952", x"6A52", x"6B51", x"6C51", x"6D50", x"6E50", x"6F50", x"704F", x"714F",
        x"724E", x"734E", x"744E", x"754D", x"764D", x"774C", x"784C", x"794C", x"7A4B", x"7B4B", x"7C4A", x"7D4A", x"7E4A", x"7F49", x"8049", x"8149",
        x"8248", x"8348", x"8447", x"8547", x"8647", x"8746", x"8846", x"8945", x"8A45", x"8A45", x"8B44", x"8C44", x"8D43", x"8E43", x"8F43", x"9042",
        x"9143", x"9343", x"9444", x"9545", x"9646", x"9747", x"9947", x"9A48", x"9B49", x"9C4A", x"9D4A", x"9F4B", x"A04C", x"A14D", x"A24D", x"A34E",
        x"A44F", x"A650", x"A750", x"A851", x"A952", x"AA53", x"AC54", x"AD54", x"AE55", x"AF56", x"B057", x"B257", x"B358", x"B459", x"B55A", x"B65A",
        x"B75B", x"B95C", x"BA5D", x"BB5D", x"BC5E", x"BD5F", x"BF60", x"C061", x"C161", x"C262", x"C363", x"C464", x"C664", x"C765", x"C866", x"C967",
        x"CA67", x"CC68", x"CD69", x"CE6A", x"CF6A", x"D06B", x"D26C", x"D36D", x"D46E", x"D56E", x"D66F", x"D770", x"D971", x"DA71", x"DB72", x"DC73",
        x"1080", x"1180", x"1280", x"1380", x"1380", x"1480", x"1580", x"1680", x"1780", x"1880", x"1980", x"1980", x"1A80", x"1B80", x"1C80", x"1D80",
        x"1E80", x"1F80", x"1F80", x"2080", x"2180", x"2280", x"2380", x"2480", x"2580", x"2580", x"2680", x"2780", x"2880", x"2980", x"2A80", x"2B80",
        x"2B80", x"2C80", x"2D80", x"2E80", x"2F80", x"3080", x"3180", x"3180", x"3280", x"3380", x"3480", x"3580", x"3680", x"3780", x"3880", x"3880",
        x"3980", x"3A80", x"3B80", x"3C80", x"3D80", x"3E80", x"3E80", x"3F80", x"4080", x"4180", x"4280", x"4380", x"4480", x"4480", x"4580", x"4680",
        x"4780", x"4880", x"4980", x"4A80", x"4A80", x"4B80", x"4C80", x"4D80", x"4E80", x"4F80", x"5080", x"5080", x"5180", x"5280", x"5380", x"5480",
        x"5580", x"5680", x"5680", x"5780", x"5880", x"5980", x"5A80", x"5B80", x"5C80", x"5C80", x"5D80", x"5E80", x"5F80", x"6080", x"6180", x"6280",
        x"6280", x"6380", x"6480", x"6580", x"6680", x"6780", x"6880", x"6880", x"6980", x"6A80", x"6B80", x"6C80", x"6D80", x"6E80", x"6E80", x"6F80",
        x"7080", x"7180", x"7280", x"7380", x"7480", x"7480", x"7580", x"7680", x"7780", x"7880", x"7980", x"7A80", x"7A80", x"7B80", x"7C80", x"7D80",
        x"7E80", x"7F80", x"8080", x"8180", x"8180", x"8280", x"8380", x"8480", x"8580", x"8680", x"8780", x"8780", x"8880", x"8980", x"8A80", x"8B80",
        x"8C80", x"8D80", x"8D80", x"8E80", x"8F80", x"9080", x"9180", x"9280", x"9380", x"9380", x"9480", x"9580", x"9680", x"9780", x"9880", x"9980",
        x"9980", x"9A80", x"9B80", x"9C80", x"9D80", x"9E80", x"9F80", x"9F80", x"A080", x"A180", x"A280", x"A380", x"A480", x"A580", x"A580", x"A680",
        x"A780", x"A880", x"A980", x"AA80", x"AB80", x"AB80", x"AC80", x"AD80", x"AE80", x"AF80", x"B080", x"B180", x"B180", x"B280", x"B380", x"B480",
        x"B580", x"B680", x"B780", x"B780", x"B880", x"B980", x"BA80", x"BB80", x"BC80", x"BD80", x"BD80", x"BE80", x"BF80", x"C080", x"C180", x"C280",
        x"C380", x"C380", x"C480", x"C580", x"C680", x"C780", x"C880", x"C980", x"CA80", x"CA80", x"CB80", x"CC80", x"CD80", x"CE80", x"CF80", x"D080",
        x"D080", x"D180", x"D280", x"D380", x"D480", x"D580", x"D680", x"D680", x"D780", x"D880", x"D980", x"DA80", x"DB80", x"DC80", x"DC80", x"DD80",
        x"DE80", x"DF80", x"E080", x"E180", x"E280", x"E280", x"E380", x"E480", x"E580", x"E680", x"E780", x"E880", x"E880", x"E980", x"EA80", x"EB80",
        x"DD7A", x"DD7A", x"DC7A", x"DB7A", x"DA7A", x"D97A", x"D77A", x"D67A", x"D479", x"D379", x"D179", x"D079", x"CF79", x"CE79", x"CC79", x"CB79",
        x"C978", x"C878", x"C678", x"C578", x"C478", x"C378", x"C178", x"C078", x"BF77", x"BE77", x"BC77", x"BB77", x"BA77", x"B977", x"B777", x"B677",
        x"B476", x"B376", x"B176", x"B076", x"AF76", x"AE76", x"AC76", x"AB76", x"A975", x"A875", x"A675", x"A575", x"A475", x"A375", x"A175", x"A075",
        x"9F74", x"9E74", x"9C74", x"9B74", x"9974", x"9874", x"9674", x"9574", x"9473", x"9373", x"9173", x"9073", x"8F73", x"8E73", x"8C73", x"8B73",
        x"8A73", x"8973", x"8773", x"8673", x"8573", x"8473", x"8273", x"8173", x"8073", x"7F73", x"7D73", x"7C73", x"7B73", x"7A73", x"7873", x"7873",
        x"7673", x"7673", x"7473", x"7373", x"7274", x"7174", x"6F74", x"6E74", x"6D74", x"6C74", x"6A74", x"6974", x"6874", x"6774", x"6674", x"6474",
        x"6475", x"6275", x"6175", x"6075", x"5F75", x"5E75", x"5C75", x"5B75", x"5A76", x"5976", x"5776", x"5676", x"5576", x"5476", x"5276", x"5276",
        x"5076", x"5076", x"4E76", x"4D76", x"4C77", x"4B77", x"4977", x"4877", x"4777", x"4677", x"4477", x"4377", x"4277", x"4177", x"4077", x"3F77",
        x"3E77", x"3D77", x"3D77", x"3C77", x"3B78", x"3A78", x"3A78", x"3978", x"3978", x"3878", x"3878", x"3778", x"3778", x"3678", x"3678", x"3578",
        x"3578", x"3478", x"3478", x"3378", x"3379", x"3279", x"3279", x"3179", x"307A", x"2F7A", x"2F7A", x"2E7A", x"2E7A", x"2D7A", x"2D7A", x"2C7A",
        x"2C7B", x"2B7B", x"2B7B", x"2A7B", x"2A7B", x"297B", x"297B", x"287B", x"287B", x"277B", x"277B", x"267B", x"267C", x"257C", x"257C", x"247C",
        x"237C", x"227C", x"227C", x"217C", x"217C", x"207C", x"207C", x"1F7C", x"1F7D", x"1E7D", x"1E7D", x"1D7D", x"1D7D", x"1C7D", x"1C7D", x"1C7D",
        x"1B7E", x"1B7E", x"1B7E", x"1B7E", x"1A7E", x"1A7E", x"1A7E", x"1A7E", x"197E", x"197E", x"197E", x"197E", x"197E", x"197E", x"197E", x"197E",
        x"187E", x"187E", x"187E", x"187E", x"177E", x"177E", x"177E", x"177E", x"167E", x"167E", x"167E", x"167E", x"157F", x"157F", x"157F", x"157F",
        x"147F", x"147F", x"147F", x"147F", x"147F", x"147F", x"147F", x"147F", x"137F", x"137F", x"137F", x"137F", x"137F", x"137F", x"137F", x"137F",
        x"127F", x"127F", x"127F", x"127F", x"117F", x"117F", x"117F", x"117F", x"117F", x"117F", x"117F", x"117F", x"1080", x"1080", x"1080", x"1080"
    );
    constant C_PAL_B : t_pal8 := (
        x"80", x"80", x"80", x"81", x"82", x"83", x"83", x"84", x"85", x"86", x"86", x"87", x"88", x"89", x"89", x"8A",
        x"8B", x"8C", x"8C", x"8D", x"8E", x"8F", x"8F", x"90", x"91", x"92", x"92", x"93", x"94", x"95", x"95", x"96",
        x"97", x"98", x"98", x"99", x"9A", x"9B", x"9B", x"9C", x"9D", x"9E", x"9E", x"9F", x"9F", x"A0", x"A0", x"A1",
        x"A1", x"A2", x"A2", x"A2", x"A2", x"A2", x"A2", x"A3", x"A3", x"A4", x"A4", x"A4", x"A5", x"A5", x"A5", x"A5",
        x"A6", x"A6", x"A6", x"A6", x"A7", x"A7", x"A7", x"A7", x"A8", x"A8", x"A8", x"A8", x"A9", x"A9", x"A9", x"A9",
        x"AA", x"AA", x"AA", x"AA", x"AB", x"AB", x"AA", x"A9", x"A8", x"A7", x"A5", x"A4", x"A3", x"A2", x"A0", x"9F",
        x"9D", x"9C", x"9A", x"99", x"98", x"97", x"95", x"94", x"93", x"92", x"90", x"8F", x"8D", x"8C", x"8A", x"89",
        x"88", x"87", x"85", x"84", x"83", x"82", x"80", x"7F", x"7D", x"7C", x"7A", x"79", x"78", x"77", x"75", x"74",
        x"73", x"72", x"70", x"6F", x"6E", x"6D", x"6B", x"6A", x"69", x"68", x"66", x"65", x"64", x"63", x"61", x"60",
        x"5F", x"5E", x"5C", x"5B", x"5A", x"59", x"57", x"56", x"55", x"54", x"52", x"51", x"50", x"4F", x"4E", x"4C",
        x"4C", x"4A", x"49", x"48", x"47", x"46", x"44", x"43", x"42", x"41", x"40", x"3E", x"3E", x"3C", x"3C", x"3A",
        x"3A", x"38", x"38", x"37", x"36", x"35", x"34", x"34", x"32", x"32", x"31", x"30", x"2F", x"2E", x"2E", x"2C",
        x"2C", x"2A", x"2A", x"28", x"28", x"26", x"26", x"25", x"24", x"23", x"22", x"22", x"20", x"20", x"1F", x"1E",
        x"1D", x"1C", x"1C", x"1D", x"1E", x"1F", x"20", x"22", x"25", x"27", x"29", x"2C", x"2E", x"31", x"33", x"36",
        x"38", x"3B", x"3D", x"3F", x"42", x"44", x"46", x"49", x"4B", x"4E", x"50", x"53", x"55", x"58", x"5A", x"5C",
        x"5F", x"61", x"63", x"66", x"68", x"6B", x"6D", x"70", x"72", x"75", x"77", x"7A", x"7C", x"7F", x"80", x"80",
        x"86", x"87", x"88", x"8A", x"8B", x"8D", x"8E", x"8F", x"91", x"92", x"94", x"95", x"96", x"98", x"99", x"9B",
        x"9C", x"9D", x"9F", x"A0", x"A2", x"A3", x"A4", x"A6", x"A7", x"A9", x"AA", x"AA", x"AB", x"AB", x"AC", x"AC",
        x"AC", x"AD", x"AD", x"AE", x"AE", x"AF", x"AF", x"B0", x"B0", x"B1", x"B1", x"B2", x"B2", x"B3", x"B3", x"B4",
        x"B4", x"B5", x"B5", x"B5", x"B6", x"B6", x"B7", x"B7", x"B8", x"B8", x"B9", x"B9", x"BA", x"BA", x"BB", x"BB",
        x"BB", x"BB", x"BA", x"B9", x"B9", x"B8", x"B7", x"B7", x"B6", x"B5", x"B4", x"B4", x"B3", x"B2", x"B2", x"B1",
        x"B0", x"B0", x"AF", x"AE", x"AE", x"AD", x"AC", x"AB", x"AB", x"AA", x"A9", x"A9", x"A8", x"A7", x"A7", x"A6",
        x"A5", x"A5", x"A4", x"A3", x"A2", x"A2", x"A1", x"A0", x"A0", x"9F", x"9E", x"9E", x"9C", x"9B", x"99", x"98",
        x"96", x"95", x"93", x"92", x"90", x"8F", x"8D", x"8C", x"8A", x"89", x"87", x"86", x"84", x"83", x"81", x"7F",
        x"7E", x"7C", x"7B", x"79", x"78", x"76", x"75", x"73", x"72", x"70", x"6F", x"6D", x"6C", x"6A", x"69", x"67",
        x"66", x"64", x"63", x"61", x"60", x"5F", x"5D", x"5C", x"5B", x"59", x"58", x"57", x"55", x"54", x"53", x"51",
        x"50", x"4F", x"4E", x"4C", x"4B", x"4A", x"48", x"47", x"46", x"44", x"43", x"42", x"41", x"3F", x"3E", x"3D",
        x"3B", x"3A", x"39", x"37", x"36", x"35", x"34", x"32", x"31", x"31", x"31", x"31", x"31", x"31", x"30", x"30",
        x"30", x"30", x"30", x"30", x"30", x"2F", x"2F", x"2F", x"2F", x"2F", x"2F", x"2E", x"2E", x"2E", x"2E", x"2E",
        x"2E", x"2E", x"2D", x"2D", x"2D", x"2D", x"2D", x"2D", x"2C", x"2C", x"2C", x"2C", x"2E", x"30", x"32", x"34",
        x"36", x"39", x"3B", x"3D", x"3F", x"41", x"43", x"46", x"48", x"4A", x"4C", x"4E", x"51", x"53", x"55", x"57",
        x"59", x"5C", x"5E", x"60", x"62", x"64", x"67", x"69", x"6B", x"6D", x"6F", x"72", x"74", x"76", x"78", x"7A",
        x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7",
        x"B7", x"B7", x"B7", x"B7", x"B6", x"B6", x"B6", x"B6", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7", x"B7",
        x"B7", x"B7", x"B7", x"B7", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6",
        x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6", x"B6",
        x"B5", x"B5", x"B5", x"B4", x"B3", x"B2", x"B2", x"B0", x"B0", x"AE", x"AE", x"AD", x"AC", x"AB", x"AB", x"AA",
        x"A9", x"A8", x"A8", x"A6", x"A6", x"A4", x"A4", x"A3", x"A2", x"A1", x"A1", x"A0", x"9F", x"9E", x"9E", x"9C",
        x"9C", x"9A", x"9A", x"99", x"98", x"97", x"97", x"96", x"95", x"94", x"94", x"92", x"92", x"90", x"90", x"8F",
        x"8E", x"8D", x"8D", x"8C", x"8B", x"8A", x"8A", x"88", x"88", x"86", x"86", x"85", x"84", x"83", x"83", x"82",
        x"81", x"80", x"80", x"7F", x"7E", x"7D", x"7C", x"7C", x"7A", x"7A", x"79", x"78", x"77", x"76", x"76", x"74",
        x"74", x"72", x"72", x"71", x"70", x"6F", x"6E", x"6E", x"6C", x"6C", x"6B", x"6A", x"69", x"68", x"68", x"66",
        x"66", x"64", x"64", x"63", x"62", x"61", x"61", x"60", x"5F", x"5E", x"5E", x"5C", x"5C", x"5A", x"5A", x"59",
        x"58", x"57", x"56", x"56", x"54", x"54", x"53", x"52", x"51", x"50", x"50", x"4F", x"4E", x"4D", x"4D", x"4C",
        x"4B", x"4A", x"4A", x"49", x"48", x"47", x"47", x"46", x"46", x"45", x"45", x"44", x"43", x"42", x"42", x"41",
        x"40", x"3F", x"3F", x"3E", x"3E", x"3D", x"3D", x"3C", x"3B", x"3A", x"3A", x"39", x"38", x"37", x"37", x"36",
        x"35", x"34", x"34", x"33", x"33", x"32", x"32", x"31", x"30", x"2F", x"2F", x"2E", x"2E", x"2D", x"2D", x"2C",
        x"2B", x"2A", x"2A", x"29", x"29", x"28", x"28", x"27", x"26", x"25", x"25", x"24", x"23", x"22", x"22", x"22",
        x"80", x"82", x"83", x"85", x"86", x"88", x"89", x"8B", x"8C", x"8E", x"8F", x"91", x"92", x"94", x"95", x"97",
        x"99", x"9A", x"9C", x"9D", x"9F", x"A0", x"A2", x"A3", x"A5", x"A6", x"A8", x"AA", x"AC", x"AE", x"B0", x"B2",
        x"B3", x"B5", x"B7", x"B9", x"BB", x"BD", x"BF", x"C1", x"C3", x"C4", x"C6", x"C8", x"CA", x"CC", x"CE", x"D0",
        x"D2", x"D4", x"D5", x"D7", x"D9", x"DB", x"DD", x"DF", x"E1", x"E0", x"DF", x"DE", x"DC", x"DB", x"DA", x"D9",
        x"D8", x"D6", x"D5", x"D4", x"D3", x"D2", x"D0", x"CF", x"CE", x"CD", x"CC", x"CA", x"C9", x"C8", x"C7", x"C6",
        x"C5", x"C3", x"C2", x"C1", x"C0", x"BF", x"BD", x"BC", x"BB", x"BA", x"B9", x"B7", x"B6", x"B3", x"B1", x"AE",
        x"AC", x"A9", x"A7", x"A4", x"A2", x"9F", x"9D", x"9A", x"98", x"95", x"93", x"90", x"8E", x"8B", x"89", x"86",
        x"84", x"81", x"7F", x"7C", x"7A", x"77", x"75", x"72", x"70", x"6D", x"6B", x"68", x"66", x"63", x"61", x"5E",
        x"5C", x"5A", x"58", x"56", x"54", x"53", x"51", x"4F", x"4D", x"4B", x"49", x"47", x"45", x"43", x"42", x"40",
        x"3E", x"3C", x"3A", x"38", x"36", x"34", x"32", x"31", x"2F", x"2D", x"2B", x"29", x"27", x"25", x"23", x"21",
        x"1F", x"1E", x"1C", x"1A", x"1A", x"1B", x"1C", x"1D", x"1E", x"1F", x"1F", x"20", x"21", x"22", x"23", x"24",
        x"25", x"26", x"26", x"27", x"28", x"29", x"2A", x"2B", x"2C", x"2D", x"2E", x"2E", x"2F", x"30", x"31", x"32",
        x"33", x"34", x"35", x"35", x"36", x"37", x"38", x"39", x"3A", x"3B", x"3C", x"3E", x"3F", x"40", x"41", x"42",
        x"43", x"44", x"46", x"47", x"48", x"49", x"4A", x"4B", x"4D", x"4E", x"4F", x"50", x"51", x"52", x"53", x"55",
        x"56", x"57", x"58", x"59", x"5A", x"5C", x"5D", x"5E", x"5F", x"61", x"62", x"63", x"65", x"66", x"67", x"69",
        x"6A", x"6B", x"6D", x"6E", x"6F", x"71", x"72", x"73", x"75", x"76", x"77", x"79", x"7A", x"7B", x"7D", x"7E",
        x"80", x"80", x"80", x"81", x"81", x"82", x"82", x"83", x"84", x"85", x"85", x"86", x"86", x"87", x"88", x"88",
        x"8A", x"8A", x"8B", x"8C", x"8C", x"8D", x"8D", x"8E", x"8F", x"90", x"90", x"91", x"91", x"92", x"92", x"93",
        x"94", x"95", x"95", x"96", x"97", x"98", x"98", x"99", x"99", x"9A", x"9A", x"9B", x"9C", x"9D", x"9D", x"9E",
        x"9F", x"A0", x"A0", x"A1", x"A1", x"A2", x"A2", x"A3", x"A3", x"A4", x"A4", x"A5", x"A6", x"A7", x"A7", x"A8",
        x"A8", x"A9", x"A9", x"AA", x"AA", x"AB", x"AB", x"AC", x"AC", x"AD", x"AD", x"AE", x"AE", x"AF", x"AF", x"B0",
        x"B1", x"B2", x"B2", x"B3", x"B3", x"B4", x"B4", x"B5", x"B5", x"B6", x"B6", x"B7", x"B7", x"B8", x"B8", x"B9",
        x"BA", x"BB", x"BB", x"BC", x"BC", x"BD", x"BD", x"BD", x"BC", x"BC", x"BC", x"BC", x"BB", x"BB", x"BB", x"BB",
        x"BA", x"BA", x"BA", x"B9", x"B9", x"B8", x"B8", x"B8", x"B7", x"B7", x"B7", x"B6", x"B6", x"B5", x"B5", x"B5",
        x"B5", x"B5", x"B5", x"B4", x"B4", x"B3", x"B3", x"B2", x"B2", x"B1", x"B1", x"B1", x"B0", x"B0", x"B0", x"B0",
        x"AF", x"AF", x"AF", x"AF", x"AE", x"AE", x"AE", x"AD", x"AD", x"AC", x"AC", x"AB", x"AA", x"A9", x"A9", x"A8",
        x"A8", x"A7", x"A7", x"A6", x"A5", x"A4", x"A4", x"A3", x"A2", x"A1", x"A1", x"A0", x"9F", x"9E", x"9E", x"9D",
        x"9D", x"9C", x"9C", x"9B", x"9A", x"99", x"99", x"98", x"98", x"97", x"97", x"96", x"95", x"94", x"94", x"93",
        x"92", x"91", x"91", x"90", x"90", x"8F", x"8F", x"8E", x"8D", x"8C", x"8C", x"8B", x"8B", x"8A", x"8A", x"8A",
        x"89", x"89", x"89", x"89", x"88", x"88", x"88", x"88", x"87", x"87", x"87", x"87", x"87", x"87", x"87", x"87",
        x"86", x"86", x"86", x"86", x"85", x"85", x"85", x"85", x"84", x"84", x"84", x"84", x"83", x"83", x"83", x"83",
        x"82", x"82", x"82", x"82", x"82", x"82", x"82", x"82", x"81", x"81", x"81", x"81", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"7F", x"7F", x"7F", x"7F", x"7F", x"7F", x"7E", x"7E", x"7E", x"7E", x"7E", x"7E", x"7D",
        x"7D", x"7D", x"7D", x"7D", x"7D", x"7C", x"7C", x"7C", x"7C", x"7C", x"7C", x"7B", x"7B", x"7B", x"7B", x"7B",
        x"7B", x"7A", x"7A", x"7A", x"7A", x"7A", x"7A", x"79", x"79", x"79", x"79", x"79", x"78", x"78", x"78", x"78",
        x"78", x"78", x"77", x"77", x"77", x"77", x"77", x"77", x"76", x"76", x"76", x"76", x"76", x"76", x"75", x"75",
        x"75", x"75", x"75", x"75", x"74", x"74", x"74", x"74", x"74", x"74", x"73", x"73", x"73", x"73", x"73", x"73",
        x"72", x"72", x"72", x"72", x"72", x"71", x"71", x"71", x"71", x"71", x"71", x"70", x"70", x"70", x"70", x"70",
        x"70", x"6F", x"6F", x"6F", x"6F", x"6F", x"6F", x"6E", x"6E", x"6E", x"6D", x"6D", x"6D", x"6D", x"6C", x"6C",
        x"6C", x"6B", x"6B", x"6B", x"6B", x"6A", x"6A", x"6A", x"6A", x"69", x"69", x"69", x"68", x"68", x"68", x"68",
        x"67", x"67", x"67", x"66", x"66", x"66", x"66", x"65", x"65", x"65", x"64", x"64", x"64", x"64", x"63", x"63",
        x"63", x"62", x"62", x"62", x"62", x"61", x"61", x"61", x"60", x"60", x"60", x"60", x"5F", x"5F", x"5F", x"5E",
        x"5E", x"5E", x"5E", x"5D", x"5D", x"5D", x"5D", x"5C", x"5C", x"5C", x"5B", x"5B", x"5B", x"5B", x"5A", x"5A",
        x"5A", x"59", x"59", x"59", x"59", x"58", x"58", x"58", x"57", x"57", x"57", x"57", x"56", x"56", x"56", x"55",
        x"56", x"56", x"57", x"57", x"57", x"58", x"58", x"59", x"59", x"59", x"5A", x"5A", x"5B", x"5B", x"5B", x"5C",
        x"5C", x"5D", x"5D", x"5D", x"5E", x"5E", x"5F", x"5F", x"5F", x"60", x"60", x"61", x"61", x"61", x"62", x"62",
        x"63", x"63", x"64", x"64", x"64", x"65", x"65", x"66", x"66", x"66", x"67", x"67", x"68", x"68", x"68", x"69",
        x"69", x"6A", x"6A", x"6A", x"6B", x"6B", x"6C", x"6C", x"6C", x"6D", x"6D", x"6E", x"6E", x"6E", x"6F", x"6F",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80", x"80",
        x"88", x"88", x"88", x"88", x"89", x"89", x"89", x"89", x"8A", x"8A", x"8A", x"8B", x"8B", x"8C", x"8C", x"8C",
        x"8D", x"8D", x"8D", x"8E", x"8E", x"8F", x"8F", x"8F", x"90", x"90", x"90", x"90", x"91", x"91", x"91", x"92",
        x"92", x"93", x"93", x"93", x"94", x"94", x"94", x"94", x"95", x"95", x"95", x"96", x"96", x"97", x"97", x"97",
        x"98", x"98", x"98", x"99", x"99", x"9A", x"9A", x"9A", x"9B", x"9B", x"9B", x"9B", x"9C", x"9C", x"9C", x"9C",
        x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D", x"9D",
        x"9D", x"9D", x"9D", x"9D", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C",
        x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C",
        x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9C", x"9B", x"9B", x"9B", x"9B",
        x"9B", x"9B", x"9B", x"9B", x"9A", x"9A", x"9A", x"9A", x"99", x"99", x"99", x"99", x"98", x"98", x"98", x"98",
        x"97", x"97", x"97", x"97", x"96", x"96", x"96", x"96", x"95", x"95", x"95", x"95", x"95", x"95", x"95", x"95",
        x"94", x"94", x"94", x"94", x"93", x"93", x"93", x"93", x"92", x"92", x"92", x"92", x"91", x"91", x"91", x"91",
        x"90", x"90", x"90", x"90", x"8F", x"8F", x"8F", x"8F", x"8E", x"8E", x"8E", x"8E", x"8D", x"8D", x"8D", x"8D",
        x"8C", x"8C", x"8C", x"8C", x"8B", x"8B", x"8B", x"8B", x"8B", x"8B", x"8B", x"8A", x"8A", x"89", x"89", x"89",
        x"89", x"89", x"89", x"89", x"88", x"88", x"88", x"88", x"87", x"87", x"87", x"87", x"86", x"86", x"86", x"86",
        x"86", x"86", x"86", x"85", x"85", x"84", x"84", x"84", x"84", x"84", x"84", x"84", x"83", x"83", x"83", x"83",
        x"82", x"82", x"82", x"82", x"81", x"81", x"81", x"81", x"81", x"81", x"81", x"81", x"80", x"80", x"80", x"80"
    );
    -- 1024 * 2^(i/64): one octave of zoom steps
    type t_zoom_rom is array (0 to 63) of unsigned(10 downto 0);
    constant C_ZOOM_ROM : t_zoom_rom := (
        to_unsigned(1024, 11), to_unsigned(1035, 11), to_unsigned(1046, 11), to_unsigned(1058, 11), to_unsigned(1069, 11), to_unsigned(1081, 11), to_unsigned(1093, 11), to_unsigned(1105, 11),
        to_unsigned(1117, 11), to_unsigned(1129, 11), to_unsigned(1141, 11), to_unsigned(1154, 11), to_unsigned(1166, 11), to_unsigned(1179, 11), to_unsigned(1192, 11), to_unsigned(1205, 11),
        to_unsigned(1218, 11), to_unsigned(1231, 11), to_unsigned(1244, 11), to_unsigned(1258, 11), to_unsigned(1272, 11), to_unsigned(1286, 11), to_unsigned(1300, 11), to_unsigned(1314, 11),
        to_unsigned(1328, 11), to_unsigned(1342, 11), to_unsigned(1357, 11), to_unsigned(1372, 11), to_unsigned(1387, 11), to_unsigned(1402, 11), to_unsigned(1417, 11), to_unsigned(1433, 11),
        to_unsigned(1448, 11), to_unsigned(1464, 11), to_unsigned(1480, 11), to_unsigned(1496, 11), to_unsigned(1512, 11), to_unsigned(1529, 11), to_unsigned(1545, 11), to_unsigned(1562, 11),
        to_unsigned(1579, 11), to_unsigned(1596, 11), to_unsigned(1614, 11), to_unsigned(1631, 11), to_unsigned(1649, 11), to_unsigned(1667, 11), to_unsigned(1685, 11), to_unsigned(1704, 11),
        to_unsigned(1722, 11), to_unsigned(1741, 11), to_unsigned(1760, 11), to_unsigned(1779, 11), to_unsigned(1798, 11), to_unsigned(1818, 11), to_unsigned(1838, 11), to_unsigned(1858, 11),
        to_unsigned(1878, 11), to_unsigned(1898, 11), to_unsigned(1919, 11), to_unsigned(1940, 11), to_unsigned(1961, 11), to_unsigned(1983, 11), to_unsigned(2004, 11), to_unsigned(2026, 11)
    );
    -- END GENERATED TABLES

    --------------------------------------------------------------------------
    -- Line buffers (luma only) and palette ROM ports
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_pyc_q : std_logic_vector(15 downto 0) := x"1080";
    signal s_pb_q  : std_logic_vector(7 downto 0) := x"80";
    signal s_pal_addr : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_key, s_k_skew, s_k_zoom, s_k_pal, s_k_range, s_k_off : unsigned(9 downto 0);
    signal s_sw_rows, s_sw_iso, s_sw_cycle, s_sw_luma, s_sw_inv : std_logic;
    signal s_p_heat : unsigned(9 downto 0);
    signal s_vstep   : unsigned(3 downto 0) := "1111";
    signal s_div_rem : unsigned(11 downto 0) := (others => '0');
    signal s_div_q   : unsigned(20 downto 0) := (others => '0');
    signal s_div_i   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_h   : unsigned(13 downto 0) := to_unsigned(3883, 14);
    signal s_key     : signed(10 downto 0) := (others => '0');
    signal s_skk     : signed(10 downto 0) := (others => '0');   -- skew knob, +-512
    signal s_swh     : signed(16 downto 0) := (others => '0');
    signal s_swl     : signed(17 downto 0) := (others => '0');
    signal s_skew    : signed(10 downto 0) := (others => '0');   -- +-half width
    signal s_zrom    : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_zoct    : unsigned(1 downto 0) := "10";
    signal s_zoom    : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_zone    : unsigned(2 downto 0) := (others => '0');
    signal s_pal_live : std_logic := '0';
    signal s_range   : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_offs    : signed(10 downto 0) := (others => '0');
    signal s_rows, s_isoon, s_wrap, s_lumao, s_inv : std_logic := '0';
    signal s_hpos    : unsigned(13 downto 0) := (others => '0');  -- glided P12, 10.4
    signal s_phase   : unsigned(18 downto 0) := (others => '0');  -- cycle phase, 10.9
    signal s_spd     : unsigned(13 downto 0) := (others => '0');  -- (P12 >> 3)^2
    signal s_heat    : unsigned(9 downto 0) := (others => '0');
    signal s_ch      : signed(12 downto 0) := (others => '0');
    signal s_cframe  : signed(12 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    --------------------------------------------------------------------------
    -- Tracking
    --------------------------------------------------------------------------
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line       : unsigned(10 downto 0) := (others => '0');
    signal s_height     : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_half_w     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    --------------------------------------------------------------------------
    -- Generated avid: starts 3 or 4 clocks after the source's first avid
    -- rise, whichever keeps the hsync->start parity at its voted usual value;
    -- ends after the field-latched width (damier / aurora, HW-confirmed)
    --------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_go : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(10 downto 0) := (others => '0');
    signal g_wm1 : unsigned(10 downto 0) := to_unsigned(479, 11);

    --------------------------------------------------------------------------
    -- Per-line sequencer (keystone row + palette row constant)
    --------------------------------------------------------------------------
    signal s_lstep   : unsigned(3 downto 0) := "1111";
    signal s_tph, s_tpl : unsigned(17 downto 0) := (others => '0');
    signal s_tp      : unsigned(24 downto 0) := (others => '0');
    signal s_t       : unsigned(10 downto 0) := (others => '0');
    signal s_tc      : signed(10 downto 0) := (others => '0');
    signal s_kth, s_skh : signed(16 downto 0) := (others => '0');
    signal s_ktl, s_skl : signed(16 downto 0) := (others => '0');
    signal s_kt, s_sk : signed(21 downto 0) := (others => '0');
    signal s_f1      : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_shift   : signed(10 downto 0) := (others => '0');
    signal s_k1h, s_k1l : unsigned(18 downto 0) := (others => '0');
    signal s_k1p     : unsigned(24 downto 0) := (others => '0');
    signal s_k1q     : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_p1h     : unsigned(17 downto 0) := (others => '0');
    signal s_p1l     : unsigned(16 downto 0) := (others => '0');
    signal s_p1      : unsigned(23 downto 0) := (others => '0');
    signal s_ia1, s_ib1, s_init1 : signed(24 downto 0) := (others => '0');
    signal s_row_off : unsigned(9 downto 0) := (others => '0');
    signal s_cline   : signed(12 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    signal s_acc1 : signed(24 downto 0) := (others => '0');
    signal s_u2   : signed(13 downto 0) := (others => '0');
    signal s_addr_y : unsigned(10 downto 0) := (others => '0');
    signal s_bgy, s_a_bgy, s_a_rbank : std_logic := '0';
    signal s_wet_y : unsigned(9 downto 0) := (others => '0');
    signal s_ym6   : signed(10 downto 0) := (others => '0');
    signal s_gp7   : signed(19 downto 0) := (others => '0');
    signal s_g8    : signed(13 downto 0) := (others => '0');
    signal s_idx9  : unsigned(9 downto 0) := (others => '0');
    signal s_clp9, s_iso10 : std_logic := '0';
    type t_y is array (6 to 12) of unsigned(9 downto 0);
    signal s_yd : t_y := (others => (others => '0'));
    type t_bg is array (5 to 12) of std_logic;
    signal s_bgd : t_bg := (others => '1');
    type t_iso is array (11 to 12) of std_logic;
    signal s_isod : t_iso := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_y13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    --------------------------------------------------------------------------
    -- Sync delay (source syncs; generated avid)
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    s_k_key    <= unsigned(registers_in(0));
    s_k_skew   <= unsigned(registers_in(1));
    s_k_zoom   <= unsigned(registers_in(2));
    s_k_pal    <= unsigned(registers_in(3));
    s_k_range  <= unsigned(registers_in(4));
    s_k_off    <= unsigned(registers_in(5));
    s_sw_rows  <= registers_in(6)(0);
    s_sw_iso   <= registers_in(6)(1);
    s_sw_cycle <= registers_in(6)(2);
    s_sw_luma  <= registers_in(6)(3);
    s_sw_inv   <= registers_in(6)(4);
    s_p_heat   <= unsigned(registers_in(7));

    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_pyc_q <= C_PAL_YC(to_integer(s_pal_addr));
            s_pb_q  <= C_PAL_B(to_integer(s_pal_addr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Generated avid
    --------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            -- registered: g_sel never changes within two clocks of g_rsr(0)
            if (g_sel = '0' and g_rsr(0) = '1') or (g_sel = '1' and g_rsr(1) = '1') then
                g_go <= '1';
            else
                g_go <= '0';
            end if;
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

            if g_go = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;

    --------------------------------------------------------------------------
    -- Stage 1 + sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_rem : unsigned(12 downto 0);
        variable v_nb  : unsigned(0 downto 0);
        variable v_ktq : signed(13 downto 0);
        variable v_f   : signed(13 downto 0);
        variable v_k1  : unsigned(14 downto 0);
        variable v_hd, v_hn : signed(14 downto 0);
        variable v_sw  : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
            else
                s_rx <= (others => '0');
            end if;
            if s_in_avid = '1' then
                s_wr_x <= s_wr_x + 1;
            else
                s_wr_x <= (others => '0');
            end if;
            s_we <= data_in.avid;

            -- keystone DDA, stepped over the generated avid
            if g_avid = '1' then
                s_acc1 <= s_acc1 + signed(resize(s_k1q, 25));
            else
                s_acc1 <= s_init1;
            end if;

            ------------------------------------------------------------------
            -- Per-line sequencer (keystone row, palette row constant)
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    s_tph <= s_line * s_inv_h(13 downto 7);
                    s_tpl <= s_line * s_inv_h(6 downto 0);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_tp <= shift_left(resize(s_tph, 25), 7) + resize(s_tpl, 25);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_tp(24 downto 20) /= 0 then
                        s_t <= to_unsigned(1024, 11);
                    else
                        s_t <= s_tp(20 downto 10);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_tc <= to_signed(512, 11) - signed(s_t);
                    if s_rows = '1' then
                        s_row_off <= s_t(10 downto 1);
                    else
                        s_row_off <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_kth <= s_key * s_tc(10 downto 5);
                    s_ktl <= s_key * signed('0' & s_tc(4 downto 0));
                    s_skh <= s_skew * s_tc(10 downto 5);
                    s_skl <= s_skew * signed('0' & s_tc(4 downto 0));
                    s_cline <= s_cframe + signed(resize(s_row_off, 13));
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_kt <= shift_left(resize(s_kth, 22), 5) + resize(s_ktl, 22);
                    s_sk <= shift_left(resize(s_skh, 22), 5) + resize(s_skl, 22);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    v_ktq := resize(shift_right(s_kt, 8), 14);
                    v_f := to_signed(1024, 14) + v_ktq;
                    if v_f < 128 then v_f := to_signed(128, 14); end if;
                    s_f1 <= unsigned(v_f(11 downto 0));
                    s_shift <= resize(shift_right(s_sk, 9), 11);
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_k1h <= s_zoom * s_f1(11 downto 6);
                    s_k1l <= s_zoom * s_f1(5 downto 0);
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_k1p <= shift_left(resize(s_k1h, 25), 6) + resize(s_k1l, 25);
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    v_k1 := s_k1p(24 downto 10);
                    if v_k1 < 64 then s_k1q <= to_unsigned(64, 13);
                    elsif v_k1 > 8191 then s_k1q <= to_unsigned(8191, 13);
                    else s_k1q <= v_k1(12 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_p1h <= s_half_w * s_k1q(12 downto 6);
                    s_p1l <= s_half_w * s_k1q(5 downto 0);
                    s_lstep <= s_lstep + 1;
                when 11 =>
                    s_p1 <= shift_left(resize(s_p1h, 24), 6) + resize(s_p1l, 24);
                    s_ib1 <= shift_left(resize(s_shift, 25), 10) - signed(resize(s_k1q, 25));
                    s_lstep <= s_lstep + 1;
                when 12 =>
                    s_ia1 <= signed(shift_left(resize(s_half_w, 25), 10)) - signed(resize(s_p1, 25));
                    s_lstep <= s_lstep + 1;
                when 13 =>
                    s_init1 <= s_ia1 + s_ib1;
                    s_lstep <= s_lstep + 1;
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Vblank sequencer
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_half_w <= resize(s_line_width(10 downto 1), 11);
                    g_wm1    <= s_line_width - 1;
                    s_key    <= signed(resize(s_k_key, 11)) - to_signed(512, 11);
                    s_skk    <= signed(resize(s_k_skew, 11)) - to_signed(512, 11);
                    s_zrom   <= C_ZOOM_ROM(to_integer(s_k_zoom(7 downto 2)));
                    s_zoct   <= s_k_zoom(9 downto 8);
                    -- the firmware loads a labelled default as the label INDEX;
                    -- honour it only until the knob first reads a real
                    -- position, so a pot resting at the bottom can't jitter
                    -- through all eight palettes
                    if s_k_pal >= 8 then
                        s_pal_live <= '1';
                    end if;
                    if s_pal_live = '0' and s_k_pal < 8 then
                        s_zone <= s_k_pal(2 downto 0);
                    else
                        s_zone <= s_k_pal(9 downto 7);
                    end if;
                    s_range  <= s_k_range(9 downto 2);
                    s_offs   <= shift_left(signed(resize(s_k_off, 11)) - to_signed(512, 11), 1);
                    s_rows   <= s_sw_rows;
                    s_isoon  <= s_sw_iso;
                    s_wrap   <= s_sw_cycle;
                    s_lumao  <= s_sw_luma;
                    s_inv    <= s_sw_inv;
                    -- P12 glide (ease by diff/8) for the position; the cycle
                    -- phase follows the position while Cycle is off, so
                    -- switching it on starts from the colours on screen
                    v_hd := signed(shift_left(resize(s_p_heat, 15), 4)) - signed(resize(s_hpos, 15));
                    v_hn := signed(resize(s_hpos, 15)) + shift_right(v_hd, 3);
                    s_hpos <= unsigned(v_hn(13 downto 0));
                    if s_sw_cycle = '1' then
                        s_phase <= s_phase + resize(s_spd, 19);
                    else
                        s_phase <= s_heat & "000000000";
                    end if;
                    s_div_rem <= (others => '0');
                    s_div_q   <= (others => '0');
                    s_div_i   <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if s_div_i = 0 then v_nb := "1"; else v_nb := "0"; end if;
                    v_rem := s_div_rem & v_nb(0);
                    if v_rem >= resize(s_height, 13) then
                        s_div_rem <= resize(v_rem - resize(s_height, 13), 12);
                        s_div_q   <= s_div_q(19 downto 0) & '1';
                    else
                        s_div_rem <= v_rem(11 downto 0);
                        s_div_q   <= s_div_q(19 downto 0) & '0';
                    end if;
                    if s_div_i = 20 then
                        s_vstep <= s_vstep + 1;
                    end if;
                    s_div_i <= s_div_i + 1;
                when 2 =>
                    if s_height < 16 then
                        s_inv_h <= (others => '0');
                    else
                        s_inv_h <= s_div_q(13 downto 0);
                    end if;
                    -- skew scaled to the measured half width
                    s_swh <= s_skk * signed('0' & s_half_w(10 downto 6));
                    s_swl <= s_skk * signed('0' & s_half_w(5 downto 0));
                    -- zoom = one octave of steps, shifted by the octave
                    case s_zoct is
                        when "00"   => s_zoom <= resize(s_zrom(10 downto 2), 13);
                        when "01"   => s_zoom <= resize(s_zrom(10 downto 1), 13);
                        when "10"   => s_zoom <= resize(s_zrom, 13);
                        when others => s_zoom <= resize(s_zrom, 12) & '0';
                    end case;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    v_sw := shift_left(resize(s_swh, 22), 6) + resize(s_swl, 22);
                    s_skew <= resize(shift_right(v_sw, 9), 11);
                    s_spd  <= s_p_heat(9 downto 3) * s_p_heat(9 downto 3);
                    if s_wrap = '1' then
                        s_heat <= s_phase(18 downto 9);
                    else
                        -- position: three quarters of the ramp at 100%
                        s_heat <= ('0' & s_hpos(13 downto 5)) + ("00" & s_hpos(13 downto 6));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_ch <= resize(s_offs, 13) + signed(resize(s_heat, 13));
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    -- Ramp = Invert mirrors the luma term (1023 - g), so
                    -- Offset, Rows and Heat still push the scene hotter
                    if s_inv = '1' then
                        s_cframe <= s_ch + to_signed(1023, 13);
                    else
                        s_cframe <= s_ch;
                    end if;
                    s_vstep <= "1111";
                when others =>
                    null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line   <= s_line + 1;
                s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_gav_sr(0)   <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_gav_sr(i)   <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain 2..3
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_u2 <= s_acc1(23 downto 10);
            v_w := signed(resize(s_line_width, 14));
            if s_u2 < 0 or s_u2 >= v_w then s_bgy <= '1'; else s_bgy <= '0'; end if;
            s_addr_y <= unsigned(s_u2(10 downto 0));
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (read at stage 4)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr_y));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr_y));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 4..16: fetch, false colour, compose, conditioning, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_g  : signed(13 downto 0);
        variable v_i  : signed(13 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 4 context
            s_a_bgy <= s_bgy; s_a_rbank <= s_rbank;
            -- stage 5: bank select
            if s_a_bgy = '1' then
                s_wet_y <= to_unsigned(64, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
            else
                s_wet_y <= unsigned(s_rdY0);
            end if;
            s_bgd(5) <= s_a_bgy;
            -- stage 6: luma minus black
            s_ym6 <= signed('0' & s_wet_y) - 64;
            s_yd(6) <= s_wet_y;
            -- stage 7: range product (11 x 9)
            s_gp7 <= s_ym6 * signed('0' & s_range);
            -- stage 8: luma term (1024 = one ramp), negated for Ramp = Invert
            v_g := resize(shift_right(s_gp7, 6), 14);
            if s_inv = '1' then
                s_g8 <= -v_g;
            else
                s_g8 <= v_g;
            end if;
            -- stage 9: + the line's constant (offset + heat + row), clamp
            -- (Cycle off) or wrap (Cycle on) into 0..1023
            v_i := s_g8 + resize(s_cline, 14);
            if s_wrap = '1' then
                s_idx9 <= unsigned(v_i(9 downto 0));
                s_clp9 <= '0';
            elsif v_i < 0 then
                s_idx9 <= (others => '0');
                s_clp9 <= '1';
            elsif v_i > 1023 then
                s_idx9 <= to_unsigned(1023, 10);
                s_clp9 <= '1';
            else
                s_idx9 <= unsigned(v_i(9 downto 0));
                s_clp9 <= '0';
            end if;
            -- stage 10: palette address; isotherm every 128 (never on a
            -- clamped pixel, or Iso blacks out whole shadow areas)
            s_pal_addr <= s_zone & s_idx9(9 downto 2);
            if s_isoon = '1' and s_clp9 = '0' and s_idx9(6 downto 3) = "0000" then
                s_iso10 <= '1';
            else
                s_iso10 <= '0';
            end if;
            -- stage 11: ROM read lands (p_roms)
            s_isod(11) <= s_iso10;
            -- stage 12: fabric copy (the Cr slot goes out on U: stored swapped)
            s_y12 <= unsigned(s_pyc_q(15 downto 8)) & "00";
            s_u12 <= unsigned(s_pyc_q(7 downto 0)) & "00";
            s_v12 <= unsigned(s_pb_q) & "00";
            s_isod(12) <= s_isod(11);
            -- stage 13: compose; black outside the keystoned source, on
            -- isotherms and outside the generated avid (the smoothing's edges)
            if s_bgd(12) = '1' or s_isod(12) = '1' or s_gav_sr(11) = '0' then
                s_y13 <= to_unsigned(64, 10);
                s_u13 <= to_unsigned(512, 10);
                s_v13 <= to_unsigned(512, 10);
            else
                if s_lumao = '0' then
                    s_y13 <= s_y12;
                elsif s_yd(12) < 64 then
                    s_y13 <= to_unsigned(64, 10);
                else
                    s_y13 <= s_yd(12);
                end if;
                s_u13 <= s_u12;
                s_v13 <= s_v12;
            end if;
            -- stages 14..15: horizontal [1 2 1]/4 on the pixel one back (g1
            -- the centre, g2 and stage 13 its neighbours)
            s_g1y <= s_y13;  s_g2y <= s_g1y;
            s_g1u <= s_u13;  s_g2u <= s_g1u;
            s_g1v <= s_v13;  s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y13, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u13, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v13, 12);
            s_y15 <= v_sy(11 downto 2);
            s_u15 <= v_su(11 downto 2);
            s_v15 <= v_sv(11 downto 2);
            -- stage 16: luma scaled toward black, 64 + 13/16 (y - 64) (white
            -- ~775, under the drawn-luma cap); neutral outside the avid
            v_d  := s_y15 - 64;
            v_yo := s_y15 - shift_right(v_d, 3) - shift_right(v_d, 4);
            if s_gav_sr(14) = '1' then
                s_y16 <= v_yo;
                s_u16 <= s_u15;
                s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10);
                s_u16 <= to_unsigned(512, 10);
                s_v16 <= to_unsigned(512, 10);
            end if;
            -- chains
            for i in 7 to 12 loop
                s_yd(i) <= s_yd(i - 1);
            end loop;
            for i in 6 to 12 loop
                s_bgd(i) <= s_bgd(i - 1);
            end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture infrared;
