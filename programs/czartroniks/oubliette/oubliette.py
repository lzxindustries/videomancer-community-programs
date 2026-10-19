#!/usr/bin/env python3
# oubliette.py - Build hook for the Oubliette Videomancer program.
# Copyright (C) 2026  czartroniks
# SPDX-License-Identifier: GPL-3.0-only
#
# Generates oubliette_tiles_pkg.vhd: a 16x16 1-bpp dungeon-tile mask ROM plus a
# per-tile full-colour palette (foreground + background YUV triples).  No third
# party dependencies (no Pillow) - tile art is hand-drawn ASCII below.
#
# Colour: BT.601 YUV stored U/V-SWAPPED for the Videomancer hardware convention
# (stored U = Cr, stored V = Cb), matching ziffern / matrix_rain.  Each tile mask
# bit selects fg (bit=1) or bg (bit=0); both are full colour, so the dungeon is
# rendered in colour with zero runtime multiplies (palette is a lookup).

import os
import sys

CELL_W = 16
CELL_H = 16


# Studio-range output: Y = 64 + y*876 (black 64, white 940), chroma 512 +/- 448.
# Full-scale luma clips RGB and kills chroma on hardware, so luma above Y_KNEE
# is compressed toward Y_CAP.  The chroma is scaled by the same factor, which
# just dims that colour (RGB * k) and so keeps it inside the gamut.
Y_KNEE = 640.0
Y_CAP = 768.0


def yuv_swapped(r, g, b):
    """RGB(0..255) -> (Y, U, V) 10-bit studio range, U/V swapped for the HW
    convention (stored U = Cr, stored V = Cb)."""
    y = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
    cb = 0.564 * (b / 255.0 - y)          # -0.5 .. 0.5
    cr = 0.713 * (r / 255.0 - y)          # -0.5 .. 0.5
    yy = 64.0 + 876.0 * y
    k = 1.0
    if yy > Y_KNEE:
        yk = Y_KNEE + (yy - Y_KNEE) * (Y_CAP - Y_KNEE) / (940.0 - Y_KNEE)
        k = (yk - 64.0) / (yy - 64.0)
        yy = yk
    uu = 512.0 + 896.0 * cr * k           # stored U = Cr
    vv = 512.0 + 896.0 * cb * k           # stored V = Cb

    def clamp(v):
        return max(64, min(960, int(round(v))))
    return clamp(yy), clamp(uu), clamp(vv)


# ----------------------------------------------------------------------------
# Tile table.  Each entry: (name, fg_rgb, bg_rgb, 16x16 art).  '#' = fg bit (1),
# anything else = bg bit (0).  Index = tile id used by the VHDL map.
# ----------------------------------------------------------------------------
# Detailed 16x16 tile masks ('#' = foreground colour, else background).
FLOOR = [   # bevelled flagstone with speckle
    "################",
    "#..............#",
    "#....#.....#...#",
    "#..............#",
    "#.......#......#",
    "#..#..........#.",
    "#..............#",
    "#.........#....#",
    "#....#.........#",
    "#..............#",
    "#........#.....#",
    "#..#..........#.",
    "#..............#",
    "#......#....#..#",
    "#..............#",
    "################",
]

WALL = [    # rough dark bedrock (the overall rock the dungeon is cut from)
    "#####.####.#####",
    "###.####.#####.#",
    "#.####.####.####",
    "####.######.####",
    "##.####.####.###",
    "#####.####.#####",
    "###.####.###.###",
    "#.####.#####.###",
    "####.####.######",
    "##.#####.####.##",
    "#####.###.######",
    "###.####.####.##",
    "#.####.#####.###",
    "####.####.###.##",
    "##.####.####.###",
    "#####.####.#####",
]

WALL2 = [   # cut-stone brick - the distinct rock that outlines rooms
    "################",
    "##.#####.#####.#",
    "##.#####.#####.#",
    "################",
    "#####.#####.####",
    "#####.#####.####",
    "################",
    "##.#####.#####.#",
    "##.#####.#####.#",
    "################",
    "#####.#####.####",
    "#####.#####.####",
    "################",
    "##.#####.#####.#",
    "##.#####.#####.#",
    "################",
]

DOOR = [
    "................",
    ".##############.",
    ".#............#.",
    ".#.##########.#.",
    ".#.#........#.#.",
    ".#.#........#.#.",
    ".#.#...##...#.#.",
    ".#.#..####..#.#.",
    ".#.#...##...#.#.",
    ".#.#........#.#.",
    ".#.#........#.#.",
    ".#.##########.#.",
    ".#............#.",
    ".##############.",
    "................",
    "................",
]

STAIRS = [
    "................",
    "..############..",
    "..#..........#..",
    "..#.########.#..",
    "..#.#......#.#..",
    "..#.#.####.#.#..",
    "..#.#.#..#.#.#..",
    "..#.#.#..#.#.#..",
    "..#.#.####.#.#..",
    "..#.#......#.#..",
    "..#.########.#..",
    "..#..........#..",
    "..############..",
    "................",
    "................",
    "................",
]

WATER = [
    "................",
    ".##...###...##..",
    "#..##.....##..##",
    "................",
    "..###...###...##",
    "##...##.....##..",
    "................",
    ".##...###...##..",
    "#..##.....##..##",
    "................",
    "..###...###...##",
    "##...##.....##..",
    "................",
    ".##...###...##..",
    "#..##.....##..##",
    "................",
]

RUBBLE = [
    "................",
    "...##.....###...",
    "..####...#####..",
    "..####...#####..",
    ".###.......###..",
    "........##......",
    "..##...####...##",
    ".####..####..###",
    ".####........###",
    "........##......",
    "...##..####..##.",
    "..####.####.####",
    "..####......####",
    "................",
    ".....##...##....",
    "....####.####...",
]

CHEST = [
    "................",
    "................",
    "..##########....",
    ".#..........#...",
    "#............#..",
    "#.##########.#..",
    "#.#........#.#..",
    "#.############..",
    "#.#...##...#.#..",
    "#.#..####..#.#..",
    "#.#...##...#.#..",
    "#.############..",
    "#............#..",
    ".#..........#...",
    "..##########....",
    "................",
]

WEAPON = [
    "..............#.",
    ".............###",
    "............###.",
    "...........###..",
    "..........###...",
    ".........###....",
    "........###.....",
    ".......###......",
    "......###.......",
    ".....###........",
    ".#######........",
    ".###.###........",
    "...###..........",
    "...###..........",
    "....#...........",
    "................",
]

GRASS = [
    "................",
    "..#....#....#...",
    ".###..###..###..",
    "..#....#....#...",
    "................",
    ".....#....#.....",
    "....###..###....",
    ".....#....#.....",
    "................",
    "..#....#....#...",
    ".###..###..###..",
    "..#....#....#...",
    "................",
    ".....#....#.....",
    "....###..###....",
    "................",
]

SAND = [
    "..#...#....#...#",
    "....#....#....#.",
    ".#....#....#....",
    "...#....#....#.#",
    "#....#....#....#",
    "..#....#....#...",
    "....#....#....#.",
    ".#....#....#....",
    "...#....#....#.#",
    "#....#....#....#",
    "..#....#....#...",
    "....#....#....#.",
    ".#....#....#....",
    "...#....#....#.#",
    "#....#....#....#",
    "..#....#....#...",
]

CRACKED = [
    "################",
    "#....#.........#",
    "#....#.........#",
    "#....#....###..#",
    "#....######..#.#",
    "#.........#..#.#",
    "#........#....##",
    "#.......#......#",
    "#####..#.......#",
    "#...#.#........#",
    "#....#.....##..#",
    "#....#....#..#.#",
    "#....#...#....##",
    "#....#..#......#",
    "#....#.........#",
    "################",
]

PILLAR = [
    "....######......",
    "...########.....",
    "..##########....",
    "..###....###....",
    "..###....###....",
    "..##########....",
    "...########.....",
    "..##########....",
    "..###....###....",
    "..###....###....",
    "..###....###....",
    "..###....###....",
    "..##########....",
    "...########.....",
    "..##########....",
    "....######......",
]

BONES = [
    "................",
    "....######......",
    "...#......#.....",
    "..#.##..##.#....",
    "..#.##..##.#....",
    "..#....#...#....",
    "..#..####..#....",
    "...#......#.....",
    "....##..##......",
    ".....####.......",
    "##............##",
    ".####......####.",
    "...####..####...",
    ".....######.....",
    "##............##",
    ".####......####.",
]

MOSS = [
    ".##..##..##..##.",
    "####.####.#####.",
    ".##..##..##..##.",
    "................",
    "##..##..##..##..",
    "####.####.####..",
    "##..##..##..##..",
    "................",
    ".##..##..##..##.",
    "####.####.#####.",
    ".##..##..##..##.",
    "................",
    "##..##..##..##..",
    "####.####.####..",
    "##..##..##..##..",
    "................",
]

LAVA = [
    "..##....###....#",
    ".####..#####..##",
    "..##....###....#",
    "................",
    "###...##....###.",
    "####.####..#####",
    "###...##....###.",
    "................",
    "..###....##.....",
    ".#####..####....",
    "..###....##.....",
    "................",
    "###....###...###",
    "####..#####.####",
    "###....###...###",
    "................",
]

# (name, fg_rgb, bg_rgb, art).  Tile id = index; keep 0..5 stable (used by VHDL).
TILES = [
    ("FLOOR",   (112, 100, 82), (60, 52, 42),   FLOOR),    # 0
    ("WALL",    (74, 66, 60),   (34, 30, 28),    WALL),    # 1 - dark bedrock fill
    ("DOOR",    (170, 108, 44), (64, 42, 24),    DOOR),    # 2
    ("STAIRS",  (235, 225, 130),(44, 40, 30),    STAIRS),  # 3
    ("WATER",   (96, 158, 228), (30, 62, 116),   WATER),   # 4
    ("RUBBLE",  (128, 118, 104),(52, 46, 40),    RUBBLE),  # 5
    ("WALL2",   (166, 158, 138),(74, 82, 62),    WALL2),   # 6 - dungeon outline rock
    ("CHEST",   (240, 198, 74), (120, 72, 30),   CHEST),   # 7 - treasure chest
    ("WEAPON",  (222, 228, 238),(90, 82, 72),    WEAPON),  # 8 - sword on ground
    ("GRASS",   (96, 184, 84),  (54, 74, 46),    GRASS),   # 9
    ("SAND",    (214, 194, 132),(150, 130, 86),  SAND),    # 10
    ("CRACKED", (112, 102, 90), (58, 52, 46),    CRACKED), # 11
    ("PILLAR",  (184, 176, 162),(58, 54, 48),    PILLAR),  # 12 - solid column
    ("BONES",   (228, 222, 206),(56, 50, 44),    BONES),   # 13
    ("MOSS",    (78, 156, 86),  (48, 64, 42),    MOSS),    # 14
    ("LAVA",    (255, 150, 44), (140, 42, 12),   LAVA),    # 15
]


def art_to_rows(art):
    """art (list of strings) -> exactly CELL_H ints, MSB = leftmost pixel.
    Robust to short/long art: missing rows are blank, extra rows/cols ignored."""
    rows = []
    for r in range(CELL_H):
        line = art[r] if r < len(art) else ""
        v = 0
        for i, c in enumerate(line[:CELL_W]):
            if c == "#":
                v |= 1 << (CELL_W - 1 - i)
        rows.append(v & 0xFFFF)
    return rows


# ----------------------------------------------------------------------------
# Sprite table (entity layer).  1-bpp masks; bit=1 -> the sprite's single fg
# colour, bit=0 -> transparent (the tile beneath shows through).  16x16.
# ----------------------------------------------------------------------------
PLAYER_WAR = [
    "......####......",
    ".....#....#.....",  # helmet
    "......####...#..",
    "......##....##..",  # sword hilt
    ".....######.##..",
    "....#..##..#.#..",
    "...#...##...##..",
    "...#...##...##..",
    ".......##....#..",
    "......####......",
    ".....#.##.#.....",
    ".....#.##.#.....",
    ".......##.......",
    "......#..#......",
    ".....##..##.....",
    "................",
]

PLAYER_MAG = [
    ".......#........",
    "......###.......",  # pointed hat
    ".....#####......",
    "....#######.....",
    "......####......",  # face
    "......#..#......",
    "......####...#..",
    ".....######..#..",
    "....#..##..#.#..",
    ".......##....#..",
    "......####...#..",  # staff
    ".....#.##.#..#..",
    ".......##.......",
    "......#..#......",
    ".....##..##.....",
    "................",
]

PLAYER_ROG = [
    "................",
    "......####......",  # hood
    ".....######.....",
    ".....#....#.....",
    ".....#.##.#.....",
    "......####......",
    "..#...####...#..",  # daggers out
    "..#..######..#..",
    "..#.#..##..#.#..",
    ".....#.##.#.....",
    ".......##.......",
    "......####......",
    ".....#....#.....",
    ".....#....#.....",
    "....##....##....",
    "................",
]

PLAYER_CLR = [
    "......####......",  # halo
    ".....#....#.....",
    "......####......",
    "......#..#......",  # face
    "......####...#..",
    ".....######..#..",
    "....#..##..#.#..",
    "...#...##...#+..",
    ".......##....#..",
    "......####...#..",  # staff
    ".....#.##.#..#..",
    ".....#.##.#.....",
    ".......##.......",
    "......#..#......",
    ".....##..##.....",
    "................",
]

RAT = [
    "................",
    "................",
    "...........###..",
    ".........##...#.",
    "..####...#....#.",
    ".#....###.####..",
    ".#.............#",
    ".#...#...#....#.",
    "..###.###.####..",
    ".....#...#......",
    "....##...##.....",
    "................",
    "................",
    "................",
    "................",
    "................",
]

SKELETON = [
    "......####......",
    ".....#....#.....",
    ".....#.##.#.....",
    ".....#....#.....",
    "......####......",
    ".......##.......",
    "...##########...",
    "..#..#.##.#..#..",
    "..#...####...#..",
    "......####......",
    ".....#.##.#.....",
    ".....#.##.#.....",
    ".....#.##.#.....",
    "....##....##....",
    "................",
    "................",
]

SLIME = [
    "................",
    "................",
    ".....######.....",
    "...##########...",
    "..############..",
    ".####.##.##.###.",
    ".##############.",
    ".##.########.##.",
    ".##############.",
    ".##############.",
    "..############..",
    "...##########...",
    ".###.#....#.###.",
    "................",
    "................",
    "................",
]

BAT = [
    "................",
    "................",
    ".#....####....#.",
    ".##..######..##.",
    ".###.######.###.",
    ".##############.",
    ".#.##.####.##.#.",
    "....#.####.#....",
    ".....#.##.#.....",
    "......####......",
    "................",
    "................",
    "................",
    "................",
    "................",
    "................",
]

ORC = [
    "......####......",
    ".....#....#.....",
    ".....#.##.#.....",
    ".....######.....",
    "......####......",
    "...##########...",
    "..#.#.####.#.#..",
    "..#.#.####.#.#..",
    "....########....",
    ".....#....#.....",
    "....##....##....",
    "....#......#....",
    "...##......##...",
    "...#........#...",
    "................",
    "................",
]

WRAITH = [
    ".......##.......",
    "......####......",
    ".....#.##.#.....",
    ".....#....#.....",
    "....##.##.##....",
    "....#......#....",
    "...#..####..#...",
    "...#.######.#...",
    "...#.######.#...",
    "...#.######.#...",
    "....#.####.#....",
    "....#.#.#.#.....",
    ".....#.#.#......",
    "......#.#.......",
    "................",
    "................",
]

CHEST = [
    "................",
    "................",
    "...##########...",
    "..#..........#..",
    "..#.########.#..",
    "..############..",
    "..#..######..#..",
    "..#.#......#.#..",
    "..#.#.####.#.#..",
    "..#.#......#.#..",
    "..############..",
    "..#..........#..",
    "...##########...",
    "................",
    "................",
    "................",
]

DRAGON = [  # depth-7 boss (monster type 6)
    "#..............#",
    "##...#....#...##",
    "###..##..##..###",
    "####.######.####",
    "####.#.##.#.####",
    "#####.####.#####",
    ".####.####.####.",
    "..###.#..#.###..",
    "......####......",
    ".....######.....",
    "....###..###....",
    "....##.##.##....",
    "...##..##..##...",
    "...#...##...#...",
    "..##........##..",
    "................",
]

# (name, fg_rgb, art).  Order matters: monster type t -> sprite C_S_MON_BASE + t.
SPRITES = [
    ("PLAYER_WAR", (210, 70, 55),  PLAYER_WAR),   # 0 - warrior, red
    ("PLAYER_MAG", (90, 120, 235), PLAYER_MAG),   # 1 - mage, blue
    ("PLAYER_ROG", (90, 195, 100), PLAYER_ROG),   # 2 - rogue, green
    ("PLAYER_CLR", (235, 215, 120),PLAYER_CLR),   # 3 - cleric, gold
    ("RAT",        (160, 115, 75), RAT),          # 4 - monster type 0
    ("SKELETON",   (220, 215, 200),SKELETON),     # 5 - monster type 1
    ("SLIME",      (110, 210, 90), SLIME),        # 6 - monster type 2
    ("BAT",        (160, 95, 205), BAT),          # 7 - monster type 3
    ("ORC",        (95, 160, 75),  ORC),          # 8 - monster type 4
    ("WRAITH",     (175, 215, 225),WRAITH),       # 9 - monster type 5
    ("DRAGON",     (230, 60, 200), DRAGON),       # 10 - monster type 6 (boss)
]


def write_sprites(here):
    out_path = os.path.join(here, "oubliette_sprites_pkg.vhd")
    n = len(SPRITES)

    L = []
    L.append("-- Copyright (C) 2026 czartroniks")
    L.append("-- SPDX-License-Identifier: GPL-3.0-only")
    L.append("--")
    L.append("-- This file is part of Videomancer Community Programs.")
    L.append("-- See LICENSE file in the repository root for full license text.")
    L.append("--")
    L.append("-- AUTO-GENERATED by oubliette.py - do not edit by hand.")
    L.append(f"-- {n} sprites x {CELL_H} rows x {CELL_W} bits.  MSB = leftmost pixel.")
    L.append("-- 1-bpp masks; bit=1 -> fg colour, bit=0 -> transparent.")
    L.append("-- fg palette stored U/V-swapped for the Videomancer hardware convention.")
    L.append("library ieee;")
    L.append("use ieee.std_logic_1164.all;")
    L.append("")
    L.append("package oubliette_sprites_pkg is")
    L.append("")
    L.append(f"    constant C_NSPR : integer := {n};")
    L.append(f"    constant C_SPR_W : integer := {CELL_W};")
    L.append(f"    constant C_SPR_H : integer := {CELL_H};")
    L.append("")
    for idx, (name, _fg, _art) in enumerate(SPRITES):
        L.append(f"    constant C_S_{name:<10} : integer := {idx};")
    L.append("")
    L.append("    type t_spr_rom is array (0 to C_NSPR - 1, 0 to C_SPR_H - 1)")
    L.append("        of std_logic_vector(C_SPR_W - 1 downto 0);")
    L.append("    type t_spal is array (0 to C_NSPR - 1) of integer range 0 to 1023;")
    L.append("")
    L.append("    constant C_SPRITES : t_spr_rom := (")
    last = n - 1
    for idx, (name, _fg, art) in enumerate(SPRITES):
        rows = art_to_rows(art)
        row_strs = ", ".join(f'x"{r:04X}"' for r in rows)
        sep = "," if idx < last else " "
        L.append(f"        {idx:2d} => ({row_strs}){sep}  -- {name}")
    L.append("    );")
    L.append("")

    sy, su, sv = [], [], []
    for (_n, fg, _a) in SPRITES:
        y, u, v = yuv_swapped(*fg)
        sy.append(y); su.append(u); sv.append(v)
    L.append("    constant C_SPR_Y : t_spal := (" + ", ".join(str(v) for v in sy) + ");")
    L.append("    constant C_SPR_U : t_spal := (" + ", ".join(str(v) for v in su) + ");")
    L.append("    constant C_SPR_V : t_spal := (" + ", ".join(str(v) for v in sv) + ");")
    L.append("")
    L.append("end package oubliette_sprites_pkg;")
    L.append("")

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(L))
    print(f"Wrote {out_path} ({n} sprites, {CELL_W}x{CELL_H} 1bpp + palette)")


def write_tables(here):
    """Lookup ROMs used by the HUD (and, from M3c, combat).  Divide-free: the
    BCD ROM turns a 0..1023 value into 3 decimal digits with a single read."""
    out_path = os.path.join(here, "oubliette_tables_pkg.vhd")
    L = []
    L.append("-- Copyright (C) 2026 czartroniks")
    L.append("-- SPDX-License-Identifier: GPL-3.0-only")
    L.append("--")
    L.append("-- This file is part of Videomancer Community Programs.")
    L.append("-- See LICENSE file in the repository root for full license text.")
    L.append("--")
    L.append("-- AUTO-GENERATED by oubliette.py - do not edit by hand.")
    L.append("library ieee;")
    L.append("use ieee.std_logic_1164.all;")
    L.append("")
    L.append("package oubliette_tables_pkg is")
    L.append("")
    L.append("    -- value 0..1023 -> packed BCD: bits 11:8 hundreds, 7:4 tens, 3:0 ones.")
    L.append("    type t_bcd is array (0 to 1023) of std_logic_vector(11 downto 0);")
    L.append("    constant C_BCD : t_bcd := (")
    for v in range(1024):
        h = (v // 100) % 10
        t = (v // 10) % 10
        o = v % 10
        packed = (h << 8) | (t << 4) | o
        sep = "," if v < 1023 else " "
        L.append(f'        {v} => x"{packed:03X}"{sep}')
    L.append("    );")
    L.append("")
    L.append("end package oubliette_tables_pkg;")
    L.append("")
    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(L))
    print(f"Wrote {out_path} (BCD digit ROM, 1024 entries)")


# ----------------------------------------------------------------------------
# HUD text.  Every line the HUD can show is a layout string; {token} marks a
# field (digits or a name) the HUD sequencer writes into a small field RAM
# each frame.  Layout ROM entry = glyph (0..51) or 64 + field slot.
# ----------------------------------------------------------------------------
FONT_CHARS = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,!?-+:'/()<>=*"
HUD_COLS = 48          # columns per HUD line (32 px glyphs: 1536 px)
HUD_STRIDE = 64        # layout ROM row stride (power of two: addr = id & col)
HUD_LAYOUTS = 32

# Numeric fields, in the HUD sequencer's order (the VHDL value mux follows
# the generated F_* indices): (token, width in digits).
NUM_FIELDS = [
    ("hp", 2), ("mhp", 2), ("mp", 2), ("mmp", 2), ("gold", 3), ("lv", 2),
    ("d", 1), ("atk", 2), ("def", 2), ("xp", 3), ("xpn", 3), ("pot", 1),
    ("ehp", 2), ("dlt", 2), ("tkn", 2), ("xpg", 2),
]
# Name fields (glyph strings), written after the numbers, contiguous slots.
NAME_FIELDS = [("cls", 3), ("mon", 3), ("act", 3), ("res", 4)]
NAME_BASE = 40

LAYOUTS = [
    ("L_BAR",      "HP {hp}/{mhp}  MP {mp}/{mmp}  POT {pot}  GOLD {gold}  LV {lv}  D {d}"),
    ("L_STATS",    "{cls}  ATK {atk}  DEF {def}  XP {xp}/{xpn}"),
    ("L_CREATE",   "CHOOSE CLASS: {cls}   ATK {atk} DEF {def}   S7 = START"),
    ("L_GEN",      "DESCENDING..."),
    ("L_FIGHT",    "FIGHT {mon} HP {ehp}  > {act}  {res} {dlt}  TOOK {tkn}"),
    ("L_DIED",     "YOU DIED ON DEPTH {d}   S7 = TRY AGAIN"),
    ("L_WIN",      "VICTORY! DRAGON SLAIN  GOLD {gold}  S7 = AGAIN"),
    ("M_SWORD",    "FOUND A SWORD   ATK +2"),
    ("M_GOLD",     "FOUND GOLD"),
    ("M_GOLDPOT",  "FOUND GOLD AND A POTION"),
    ("M_DRANK",    "DRANK A POTION   HP +{dlt}"),
    ("M_NOPOT",    "NO POTIONS LEFT"),
    ("M_FULL",     "HP IS ALREADY FULL"),
    ("M_LEVEL",    "LEVEL UP!   NOW LEVEL {lv}"),
    ("M_SLAIN",    "{mon} SLAIN   XP +{xpg}"),
    ("M_SLAINPOT", "{mon} SLAIN   XP +{xpg}   IT DROPPED A POTION"),
    ("M_FLED",     "YOU FLED"),
    ("M_DEPTH",    "DEPTH {d}   FIND THE STAIRS DOWN"),
    ("M_DRAGON",   "DEPTH 7   THE DRAGON AWAITS"),
]

# 3/4-letter names (glyph strings) the HUD copies into the name fields.  They
# live in the layout ROM's last four rows (4-char slots: column = index*4 + k,
# a bit concatenation), copied into the field RAM by the HUD sequencer.
CLASS_NAMES = ["WAR", "MAG", "ROG", "CLR"]
MON_NAMES = ["RAT", "SKE", "SLM", "BAT", "ORC", "WRA", "DRG", "???"]
ACT_NAMES = ["ATK", "DEF", "CST", "ITM", "FLE"]      # 2 = class cast name
CAST_NAMES = ["BSH", "FIR", "STB", "HEL"]           # per class
NAME_ROW0 = 28   # layout ROM rows 28..31 hold the name tables
RES_WORDS = [  # combat result word, index = R_* in the VHDL
    ("R_NONE", "    "), ("R_HIT", "HIT "), ("R_BASH", "BASH"), ("R_FIRE", "FIRE"),
    ("R_STAB", "STAB"), ("R_CRIT", "CRIT"), ("R_HEAL", "HEAL"), ("R_BLCK", "BLCK"),
    ("R_NOMP", "NOMP"), ("R_NOPOT", "NONE"), ("R_FAIL", "FAIL"),
]


def glyph(c):
    if c not in FONT_CHARS:
        raise ValueError(f"HUD text: no glyph for {c!r}")
    return FONT_CHARS.index(c)


def write_hud(here):
    out_path = os.path.join(here, "oubliette_hud_pkg.vhd")
    slots = {}
    k = 0
    for tok, w in NUM_FIELDS:
        slots[tok] = (k, w)
        k += w
    assert k <= NAME_BASE, "numeric fields overflow into name slots"
    k = NAME_BASE
    for tok, w in NAME_FIELDS:
        slots[tok] = (k, w)
        k += w
    assert k <= 64

    rom = [0] * (HUD_LAYOUTS * HUD_STRIDE)
    assert len(LAYOUTS) <= NAME_ROW0, "layouts overflow into the name rows"
    for li, (name, text) in enumerate(LAYOUTS):
        codes = []
        i = 0
        while i < len(text):
            if text[i] == "{":
                j = text.index("}", i)
                base, w = slots[text[i + 1:j]]
                codes += [64 + base + n for n in range(w)]
                i = j + 1
            else:
                codes.append(glyph(text[i]))
                i += 1
        if len(codes) > HUD_COLS:
            raise ValueError(f"layout {name} is {len(codes)} columns (max {HUD_COLS})")
        for c, v in enumerate(codes):
            rom[li * HUD_STRIDE + c] = v
    # title logo (class select): top-line columns 48..56 of L_CREATE, drawn
    # 2x size in the logo band (the HUD only displays columns 0..47)
    li_create = [n for n, _t in LAYOUTS].index("L_CREATE")
    for k, c in enumerate("OUBLIETTE"):
        rom[li_create * HUD_STRIDE + 48 + k] = glyph(c)
    name_rows = [
        CLASS_NAMES,                      # row NAME_ROW0 + 0
        MON_NAMES,                        # + 1
        ACT_NAMES + CAST_NAMES,           # + 2 (cast names from ACT_CAST0)
        [w for _n, w in RES_WORDS],       # + 3
    ]
    for r, names in enumerate(name_rows):
        assert len(names) * 4 <= HUD_STRIDE
        for i, nm in enumerate(names):
            for k, c in enumerate(nm.ljust(4)[:4]):
                rom[(NAME_ROW0 + r) * HUD_STRIDE + i * 4 + k] = glyph(c)

    L = []
    L.append("-- Copyright (C) 2026 czartroniks")
    L.append("-- SPDX-License-Identifier: GPL-3.0-only")
    L.append("--")
    L.append("-- This file is part of Videomancer Community Programs.")
    L.append("-- See LICENSE file in the repository root for full license text.")
    L.append("--")
    L.append("-- AUTO-GENERATED by oubliette.py - do not edit by hand.")
    L.append("-- HUD layout ROM (layout id x 64 columns): glyph 0..51, or 64 + field slot.")
    L.append("library ieee;")
    L.append("use ieee.std_logic_1164.all;")
    L.append("")
    L.append("package oubliette_hud_pkg is")
    L.append("")
    L.append(f"    constant C_HUDN     : integer := {HUD_COLS};")
    L.append(f"    constant C_G_SP     : integer := {glyph(' ')};")
    L.append(f"    constant C_G_D0     : integer := {glyph('0')};")
    L.append("")
    for li, (name, _t) in enumerate(LAYOUTS):
        L.append(f"    constant {name:<10} : integer := {li};")
    L.append("")
    L.append(f"    constant C_NFLD : integer := {len(NUM_FIELDS)};")
    for fi, (tok, _w) in enumerate(NUM_FIELDS):
        L.append(f"    constant F_{tok.upper():<4} : integer := {fi};")
    L.append("    type t_fld is array (0 to C_NFLD - 1) of integer range 0 to 63;")
    L.append("    constant C_FLD_BASE : t_fld := (" +
             ", ".join(str(slots[t][0]) for t, _w in NUM_FIELDS) + ");")
    L.append("    constant C_FLD_W    : t_fld := (" +
             ", ".join(str(w) for _t, w in NUM_FIELDS) + ");")
    L.append(f"    constant C_NAME_BASE : integer := {NAME_BASE};")
    L.append(f"    constant C_NNAME     : integer := {sum(w for _t, w in NAME_FIELDS)};")
    L.append("")
    for ri, (rn, _w) in enumerate(RES_WORDS):
        L.append(f"    constant {rn:<8} : integer := {ri};")
    L.append("")

    L.append(f"    constant C_NROW_CLS : integer := {NAME_ROW0};")
    L.append(f"    constant C_NROW_MON : integer := {NAME_ROW0 + 1};")
    L.append(f"    constant C_NROW_ACT : integer := {NAME_ROW0 + 2};")
    L.append(f"    constant C_NROW_RES : integer := {NAME_ROW0 + 3};")
    L.append(f"    constant C_ACT_CAST0 : integer := {len(ACT_NAMES)};")
    L.append("")
    L.append(f"    type t_hudlay is array (0 to {HUD_LAYOUTS * HUD_STRIDE - 1}) of std_logic_vector(6 downto 0);")
    L.append("    constant C_HUDLAY : t_hudlay := (")
    body = []
    for a, v in enumerate(rom):
        body.append(f'"{v:07b}"')
    for r in range(0, len(body), 16):
        sep = "," if r + 16 < len(body) else ""
        L.append("        " + ", ".join(body[r:r + 16]) + sep)
    L.append("    );")
    L.append("")
    L.append("end package oubliette_hud_pkg;")
    L.append("")
    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(L))
    print(f"Wrote {out_path} ({len(LAYOUTS)} layouts x {HUD_COLS} cols)")


# ----------------------------------------------------------------------------
# Palette ROM (one EBR pair): every colour the select stage can pick, by a
# 7-bit address: 0-15 tile fg, 16-31 tile bg, 32-47 sprites, 48-63 fixed
# colours; +64 = the same colour dimmed (remembered cells under the fog:
# y/2 + 32, chroma/4 + 384 - what the S7 arithmetic used to do).
# ----------------------------------------------------------------------------
FIXED = [  # (name, address 48..63, (Y, U, V)) - studio range, U/V swapped
    ("INK",    48, (737, 512, 512)),
    ("HUDBAR", 49, (96, 512, 512)),
    ("VOID",   50, (64, 512, 512)),
    ("TGT",    51, (705, 607, 195)),
    ("FLASH",  52, (753, 512, 512)),
    ("HURT",   53, (365, 856, 388)),
    ("LOWHP",  54, (215, 702, 444)),
    ("HPBG",   55, (211, 512, 512)),
    ("BLANK",  56, (64, 512, 512)),
    ("HPBARG", 60, (556, 303, 367)),
    ("HPBARA", 61, (651, 671, 232)),
    ("HPBARR", 62, (388, 840, 385)),
]


def write_palette(here):
    out_path = os.path.join(here, "oubliette_pal_pkg.vhd")
    pal = [(64, 512, 512)] * 64
    for t, (_n, fg, bg, _a) in enumerate(TILES):
        pal[t] = yuv_swapped(*fg)
        pal[16 + t] = yuv_swapped(*bg)
    for e, (_n, fg, _a) in enumerate(SPRITES):
        pal[32 + e] = yuv_swapped(*fg)
    for _n, a, yuv in FIXED:
        pal[a] = yuv

    def dim(c):
        y, u, v = c
        return ((y >> 1) + 32, (u >> 2) + 384, (v >> 2) + 384)
    full = pal + [dim(c) for c in pal]

    L = []
    L.append("-- Copyright (C) 2026 czartroniks")
    L.append("-- SPDX-License-Identifier: GPL-3.0-only")
    L.append("--")
    L.append("-- This file is part of Videomancer Community Programs.")
    L.append("-- See LICENSE file in the repository root for full license text.")
    L.append("--")
    L.append("-- AUTO-GENERATED by oubliette.py - do not edit by hand.")
    L.append("-- Palette ROM: Y & U & V (10 bits each, U/V swapped).  0-15 tile fg,")
    L.append("-- 16-31 tile bg, 32-47 sprites, 48-63 fixed colours; +64 dimmed.")
    L.append("library ieee;")
    L.append("use ieee.std_logic_1164.all;")
    L.append("")
    L.append("package oubliette_pal_pkg is")
    L.append("")
    for n, a, _c in FIXED:
        L.append(f"    constant C_PA_{n:<6} : integer := {a};")
    L.append("    type t_palrom is array (0 to 127) of std_logic_vector(29 downto 0);")
    L.append("    constant C_PALROM : t_palrom := (")
    for i, (y, u, v) in enumerate(full):
        sep = "," if i < 127 else ""
        L.append(f'        "{y:010b}{u:010b}{v:010b}"{sep}')
    L.append("    );")
    L.append("")
    L.append("end package oubliette_pal_pkg;")
    L.append("")
    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(L))
    print(f"Wrote {out_path} (128-entry palette ROM)")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, "oubliette_tiles_pkg.vhd")
    n = len(TILES)

    L = []
    L.append("-- Copyright (C) 2026 czartroniks")
    L.append("-- SPDX-License-Identifier: GPL-3.0-only")
    L.append("--")
    L.append("-- This file is part of Videomancer Community Programs.")
    L.append("-- See LICENSE file in the repository root for full license text.")
    L.append("--")
    L.append("-- AUTO-GENERATED by oubliette.py - do not edit by hand.")
    L.append(f"-- {n} tiles x {CELL_H} rows x {CELL_W} bits.  MSB = leftmost pixel.")
    L.append("-- fg/bg palette stored U/V-swapped for the Videomancer hardware convention.")
    L.append("library ieee;")
    L.append("use ieee.std_logic_1164.all;")
    L.append("")
    L.append("package oubliette_tiles_pkg is")
    L.append("")
    L.append(f"    constant C_NTILES : integer := {n};")
    L.append(f"    constant C_TILE_W : integer := {CELL_W};")
    L.append(f"    constant C_TILE_H : integer := {CELL_H};")
    L.append("")
    # tile ids as named constants
    for idx, (name, _fg, _bg, _art) in enumerate(TILES):
        L.append(f"    constant C_T_{name:<6} : integer := {idx};")
    L.append("")
    L.append("    type t_tile_rom is array (0 to C_NTILES - 1, 0 to C_TILE_H - 1)")
    L.append("        of std_logic_vector(C_TILE_W - 1 downto 0);")
    L.append("")
    L.append("    type t_pal is array (0 to C_NTILES - 1) of integer range 0 to 1023;")
    L.append("")
    L.append("    constant C_TILES : t_tile_rom := (")
    last = n - 1
    for idx, (name, _fg, _bg, art) in enumerate(TILES):
        rows = art_to_rows(art)
        row_strs = ", ".join(f'x"{r:04X}"' for r in rows)
        sep = "," if idx < last else " "
        L.append(f"        {idx:2d} => ({row_strs}){sep}  -- {name}")
    L.append("    );")
    L.append("")

    # palette arrays
    def pal_line(name, vals):
        body = ", ".join(str(v) for v in vals)
        return f"    constant {name} : t_pal := ({body});"

    fy, fu, fv, by, bu, bv = [], [], [], [], [], []
    for (_n, fg, bg, _a) in TILES:
        y, u, v = yuv_swapped(*fg)
        fy.append(y); fu.append(u); fv.append(v)
        y, u, v = yuv_swapped(*bg)
        by.append(y); bu.append(u); bv.append(v)

    L.append(pal_line("C_TILE_FG_Y", fy))
    L.append(pal_line("C_TILE_FG_U", fu))
    L.append(pal_line("C_TILE_FG_V", fv))
    L.append(pal_line("C_TILE_BG_Y", by))
    L.append(pal_line("C_TILE_BG_U", bu))
    L.append(pal_line("C_TILE_BG_V", bv))
    L.append("")
    L.append("end package oubliette_tiles_pkg;")
    L.append("")

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(L))

    print(f"Wrote {out_path} ({n} tiles, {CELL_W}x{CELL_H} 1bpp + palette)")

    write_sprites(here)
    write_tables(here)
    write_hud(here)
    write_palette(here)
    return 0


if __name__ == "__main__":
    sys.exit(main())
