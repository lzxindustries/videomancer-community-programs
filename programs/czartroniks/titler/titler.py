#!/usr/bin/env python3
# Copyright (C) 2025
# SPDX-License-Identifier: GPL-3.0-only
"""
Titler - Build Hook

Generates titler_font_pkg.vhd from a visually-defined 8x8 bitmap font.
The character set is 64 glyphs (6-bit index), packed with the visible
glyph in the upper-left so that 5x7 chars leave a 1px gap on the right
and bottom for inter-character spacing.

Each character row is stored as an 8-bit std_logic_vector. The font ROM
is a 2D array indexed by [char_index][row]. MSB of each row is the
leftmost pixel.
"""

import os
import sys

# Each glyph is exactly 8 lines of 8 chars. 'X' = pixel on, '.' = off.
# Glyphs are 5px wide / 7px tall with right/bottom padding for spacing.
GLYPHS = {
    " ": (
        "........",
        "........",
        "........",
        "........",
        "........",
        "........",
        "........",
        "........",
    ),
    "A": (
        ".XXX....",
        "X...X...",
        "X...X...",
        "XXXXX...",
        "X...X...",
        "X...X...",
        "X...X...",
        "........",
    ),
    "B": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "........",
    ),
    "C": (
        ".XXX....",
        "X...X...",
        "X.......",
        "X.......",
        "X.......",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "D": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "XXXX....",
        "........",
    ),
    "E": (
        "XXXXX...",
        "X.......",
        "X.......",
        "XXXX....",
        "X.......",
        "X.......",
        "XXXXX...",
        "........",
    ),
    "F": (
        "XXXXX...",
        "X.......",
        "X.......",
        "XXXX....",
        "X.......",
        "X.......",
        "X.......",
        "........",
    ),
    "G": (
        ".XXX....",
        "X...X...",
        "X.......",
        "X.XXX...",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "H": (
        "X...X...",
        "X...X...",
        "X...X...",
        "XXXXX...",
        "X...X...",
        "X...X...",
        "X...X...",
        "........",
    ),
    "I": (
        "XXXXX...",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "XXXXX...",
        "........",
    ),
    "J": (
        "..XXX...",
        "....X...",
        "....X...",
        "....X...",
        "....X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "K": (
        "X...X...",
        "X..X....",
        "X.X.....",
        "XX......",
        "X.X.....",
        "X..X....",
        "X...X...",
        "........",
    ),
    "L": (
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "XXXXX...",
        "........",
    ),
    "M": (
        "X...X...",
        "XX.XX...",
        "X.X.X...",
        "X.X.X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "........",
    ),
    "N": (
        "X...X...",
        "XX..X...",
        "XX..X...",
        "X.X.X...",
        "X..XX...",
        "X..XX...",
        "X...X...",
        "........",
    ),
    "O": (
        ".XXX....",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "P": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "X.......",
        "X.......",
        "X.......",
        "........",
    ),
    "Q": (
        ".XXX....",
        "X...X...",
        "X...X...",
        "X...X...",
        "X.X.X...",
        "X..X....",
        ".XX.X...",
        "........",
    ),
    "R": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "X.X.....",
        "X..X....",
        "X...X...",
        "........",
    ),
    "S": (
        ".XXXX...",
        "X.......",
        "X.......",
        ".XXX....",
        "....X...",
        "....X...",
        "XXXX....",
        "........",
    ),
    "T": (
        "XXXXX...",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "........",
    ),
    "U": (
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "V": (
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        ".X.X....",
        "..X.....",
        "........",
    ),
    "W": (
        "X...X...",
        "X...X...",
        "X...X...",
        "X.X.X...",
        "X.X.X...",
        "XX.XX...",
        "X...X...",
        "........",
    ),
    "X": (
        "X...X...",
        "X...X...",
        ".X.X....",
        "..X.....",
        ".X.X....",
        "X...X...",
        "X...X...",
        "........",
    ),
    "Y": (
        "X...X...",
        "X...X...",
        ".X.X....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "........",
    ),
    "Z": (
        "XXXXX...",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        "X.......",
        "XXXXX...",
        "........",
    ),
    "0": (
        ".XXX....",
        "X...X...",
        "X..XX...",
        "X.X.X...",
        "XX..X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "1": (
        "..X.....",
        ".XX.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        ".XXX....",
        "........",
    ),
    "2": (
        ".XXX....",
        "X...X...",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        "XXXXX...",
        "........",
    ),
    "3": (
        "XXXX....",
        "....X...",
        "....X...",
        ".XXX....",
        "....X...",
        "....X...",
        "XXXX....",
        "........",
    ),
    "4": (
        "...X....",
        "..XX....",
        ".X.X....",
        "X..X....",
        "XXXXX...",
        "...X....",
        "...X....",
        "........",
    ),
    "5": (
        "XXXXX...",
        "X.......",
        "XXXX....",
        "....X...",
        "....X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "6": (
        ".XXX....",
        "X...X...",
        "X.......",
        "XXXX....",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "7": (
        "XXXXX...",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        ".X......",
        ".X......",
        "........",
    ),
    "8": (
        ".XXX....",
        "X...X...",
        "X...X...",
        ".XXX....",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "9": (
        ".XXX....",
        "X...X...",
        "X...X...",
        ".XXXX...",
        "....X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    ".": (
        "........",
        "........",
        "........",
        "........",
        "........",
        ".XX.....",
        ".XX.....",
        "........",
    ),
    ",": (
        "........",
        "........",
        "........",
        "........",
        ".XX.....",
        ".XX.....",
        ".X......",
        "........",
    ),
    "!": (
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "........",
        "..X.....",
        "........",
    ),
    "?": (
        ".XXX....",
        "X...X...",
        "....X...",
        "...X....",
        "..X.....",
        "........",
        "..X.....",
        "........",
    ),
    "-": (
        "........",
        "........",
        "........",
        "XXXXX...",
        "........",
        "........",
        "........",
        "........",
    ),
    "+": (
        "........",
        "..X.....",
        "..X.....",
        "XXXXX...",
        "..X.....",
        "..X.....",
        "........",
        "........",
    ),
    ":": (
        "........",
        ".XX.....",
        ".XX.....",
        "........",
        ".XX.....",
        ".XX.....",
        "........",
        "........",
    ),
    "'": (
        "..X.....",
        "..X.....",
        "..X.....",
        "........",
        "........",
        "........",
        "........",
        "........",
    ),
    "/": (
        "........",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        "X.......",
        "........",
        "........",
    ),
    "*": (
        "........",
        "X.X.X...",
        ".XXX....",
        "XXXXX...",
        ".XXX....",
        "X.X.X...",
        "........",
        "........",
    ),
    "<": (
        "...X....",
        "..X.....",
        ".X......",
        "X.......",
        ".X......",
        "..X.....",
        "...X....",
        "........",
    ),
    ">": (
        ".X......",
        "..X.....",
        "...X....",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        "........",
    ),
    "=": (
        "........",
        "........",
        "XXXXX...",
        "........",
        "XXXXX...",
        "........",
        "........",
        "........",
    ),
    "(": (
        "...X....",
        "..X.....",
        ".X......",
        ".X......",
        ".X......",
        "..X.....",
        "...X....",
        "........",
    ),
    ")": (
        ".X......",
        "..X.....",
        "...X....",
        "...X....",
        "...X....",
        "..X.....",
        ".X......",
        "........",
    ),
}


# Second font: a "Block Bold" variant. Filled valleys/peaks on the diagonal
# letters (V, M, W, N), octagonal O with chamfered corners, tapered D,
# shorter (3-col) middle bars on E/F, full-base "1" with flag, Z with
# crossbar, and arcade-style 0 (slashed) / 4 (hooked). Letters not explicitly
# listed here fall back to the default glyph.
SCIFI_GLYPHS = {
    "A": (
        "..X.....",
        ".XXX....",
        "X.X.X...",
        "XXXXX...",
        "X...X...",
        "X...X...",
        "X...X...",
        "........",
    ),
    "B": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXX.....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "........",
    ),
    "C": (
        ".XXXX...",
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        ".XXXX...",
        "........",
    ),
    "D": (
        "XXX.....",
        "X..X....",
        "X...X...",
        "X...X...",
        "X...X...",
        "X..X....",
        "XXX.....",
        "........",
    ),
    "E": (
        "XXXXX...",
        "X.......",
        "X.......",
        "XXX.....",
        "X.......",
        "X.......",
        "XXXXX...",
        "........",
    ),
    "F": (
        "XXXXX...",
        "X.......",
        "X.......",
        "XXX.....",
        "X.......",
        "X.......",
        "X.......",
        "........",
    ),
    "G": (
        ".XXXX...",
        "X.......",
        "X.......",
        "X.XXX...",
        "X...X...",
        "X...X...",
        ".XXXX...",
        "........",
    ),
    "H": (
        "X...X...",
        "X...X...",
        "X...X...",
        "XXXXX...",
        "X...X...",
        "X...X...",
        "X...X...",
        "........",
    ),
    "I": (
        "XXXXX...",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "XXXXX...",
        "........",
    ),
    "J": (
        "XXXXX...",
        "....X...",
        "....X...",
        "....X...",
        "....X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "K": (
        "X...X...",
        "X..X....",
        "X.X.....",
        "XX......",
        "X.X.....",
        "X..X....",
        "X...X...",
        "........",
    ),
    "L": (
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "X.......",
        "XXXXX...",
        "........",
    ),
    "M": (
        "X...X...",
        "XX.XX...",
        "XXXXX...",
        "X.X.X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "........",
    ),
    "N": (
        "X...X...",
        "XX..X...",
        "XXX.X...",
        "X.X.X...",
        "X.XXX...",
        "X..XX...",
        "X...X...",
        "........",
    ),
    "O": (
        ".XXX....",
        "XX.XX...",
        "X...X...",
        "X...X...",
        "X...X...",
        "XX.XX...",
        ".XXX....",
        "........",
    ),
    "P": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "X.......",
        "X.......",
        "X.......",
        "........",
    ),
    "Q": (
        ".XXX....",
        "X...X...",
        "X...X...",
        "X...X...",
        "X.X.X...",
        "X..X....",
        ".XX.X...",
        "........",
    ),
    "R": (
        "XXXX....",
        "X...X...",
        "X...X...",
        "XXXX....",
        "X.X.....",
        "X..X....",
        "X...X...",
        "........",
    ),
    "S": (
        ".XXXX...",
        "X.......",
        "X.......",
        ".XXX....",
        "....X...",
        "....X...",
        "XXXX....",
        "........",
    ),
    "T": (
        "XXXXX...",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "........",
    ),
    "U": (
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "V": (
        "X...X...",
        "X...X...",
        "X...X...",
        "X...X...",
        ".X.X....",
        ".XXX....",
        "..X.....",
        "........",
    ),
    "W": (
        "X...X...",
        "X...X...",
        "X...X...",
        "X.X.X...",
        "XXXXX...",
        "XX.XX...",
        "X...X...",
        "........",
    ),
    "X": (
        "X...X...",
        "X...X...",
        ".X.X....",
        "..X.....",
        ".X.X....",
        "X...X...",
        "X...X...",
        "........",
    ),
    "Y": (
        "X...X...",
        "X...X...",
        ".X.X....",
        "..X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "........",
    ),
    "Z": (
        "XXXXX...",
        "....X...",
        "...X....",
        ".XXXX...",
        ".X......",
        "X.......",
        "XXXXX...",
        "........",
    ),
    "0": (
        ".XXX....",
        "X...X...",
        "X..XX...",
        "X.X.X...",
        "XX..X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "1": (
        "..X.....",
        ".XX.....",
        "X.X.....",
        "..X.....",
        "..X.....",
        "..X.....",
        "XXXXX...",
        "........",
    ),
    "2": (
        ".XXX....",
        "X...X...",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        "XXXXX...",
        "........",
    ),
    "3": (
        "XXXX....",
        "....X...",
        "....X...",
        ".XXX....",
        "....X...",
        "....X...",
        "XXXX....",
        "........",
    ),
    "4": (
        ".XXXX...",
        ".X..X...",
        ".X..X...",
        ".XXXX...",
        "....X...",
        "....X...",
        "....X...",
        "........",
    ),
    "5": (
        "XXXXX...",
        "X.......",
        "XXXX....",
        "....X...",
        "....X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "6": (
        ".XXX....",
        "X.......",
        "X.......",
        "XXXX....",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "7": (
        "XXXXX...",
        "....X...",
        "...X....",
        "..X.....",
        ".X......",
        ".X......",
        ".X......",
        "........",
    ),
    "8": (
        ".XXX....",
        "X...X...",
        "X...X...",
        ".XXX....",
        "X...X...",
        "X...X...",
        ".XXX....",
        "........",
    ),
    "9": (
        ".XXX....",
        "X...X...",
        "X...X...",
        ".XXXX...",
        "....X...",
        "....X...",
        ".XXX....",
        "........",
    ),
}

# Fixed character order: index = position in this string. Must be 64 entries.
CHARSET = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,!?-+:'/()<>=*"
# Pad with spaces to exactly 64 entries.
CHARSET = CHARSET + " " * (64 - len(CHARSET))
assert len(CHARSET) == 64, f"charset must be 64 entries, got {len(CHARSET)}"


def glyph_to_rows(glyph):
    """Convert an 8-line ASCII glyph to a list of 8 ints (one per row).

    The glyph data is shifted right by 1 column in the 8-pixel-wide cell so
    that column 0 is always blank. This leaves a 1px gap on the left side of
    every glyph, matching the existing 1px gap on the right side. The renderer
    relies on column 0 (and column 7) being blank so the editing cursor box
    can be drawn at those positions without overlapping any letter pixels.
    """
    assert len(glyph) == 8, "glyph must have 8 rows"
    rows = []
    for line in glyph:
        assert len(line) == 8, f"row must be 8 chars, got: {line!r}"
        v = 0
        for i, c in enumerate(line):
            if c == "X":
                v |= 1 << (7 - i)  # MSB = leftmost pixel
        rows.append(v >> 1)  # left-pad: glyph occupies cols 1..5, col 0 blank
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, "titler_font_pkg.vhd")

    print("Generating titler font ROM...")

    # The ROM is laid out as 128 entries: the first 64 are the default font
    # (font 0), the next 64 are the Sci-Fi font (font 1). Hardware addresses
    # the entry as (font_sel * 64 + char_idx), so the font picker is just one
    # extra bit at the top of the address.
    fonts = [
        ("default", GLYPHS),
        ("sci-fi",  SCIFI_GLYPHS),
    ]
    total_count = 64 * len(fonts)

    lines = []
    lines.append("-- Copyright (C) 2026 czartroniks")
    lines.append("-- SPDX-License-Identifier: GPL-3.0-only")
    lines.append("--")
    lines.append("-- This file is part of Videomancer Community Programs.")
    lines.append("-- See LICENSE file in the repository root for full license text.")
    lines.append("--")
    lines.append("-- AUTO-GENERATED by titler.py - do not edit by hand.")
    lines.append(f"-- {total_count} glyphs, one 64-bit word each (8 rows x 8 bits).")
    lines.append("-- Row r occupies bits (63-8*r downto 56-8*r); MSB of each row is the")
    lines.append("-- leftmost pixel.  Fetching the WHOLE glyph in one read is what makes the")
    lines.append("-- 3x3 neighbourhood (drop shadow / outline / emboss) free in the pixel path.")
    lines.append("-- Layout: entries 0..63 = default font, 64..127 = sci-fi font.")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("")
    lines.append("package titler_font_pkg is")
    lines.append("")
    lines.append(f"    constant C_FONT_CHAR_COUNT : integer := {total_count};")
    lines.append("    constant C_FONT_GLYPH_W    : integer := 8;")
    lines.append("    constant C_FONT_GLYPH_H    : integer := 8;")
    lines.append("")
    lines.append("    type t_font_rom is array (0 to C_FONT_CHAR_COUNT - 1)")
    lines.append("        of std_logic_vector(63 downto 0);")
    lines.append("")
    lines.append("    constant C_FONT_ROM : t_font_rom := (")

    last_idx = total_count - 1
    for font_idx, (font_name, glyph_dict) in enumerate(fonts):
        lines.append(f"        -- font {font_idx}: {font_name}")
        for sub_idx, ch in enumerate(CHARSET):
            # Sci-fi font falls back to the default glyph for any char not
            # explicitly defined (punctuation etc.).
            glyph = glyph_dict.get(ch) or GLYPHS.get(ch, GLYPHS[" "])
            rows = glyph_to_rows(glyph)
            word = "".join(f"{r:02X}" for r in rows)
            comment = ch if ch != " " else "SPC"
            abs_idx = font_idx * 64 + sub_idx
            sep = "," if abs_idx < last_idx else " "
            lines.append(f'        {abs_idx:3d} => x"{word}"{sep}  -- {comment}')
    lines.append("    );")
    lines.append("")
    lines.append("end package titler_font_pkg;")
    lines.append("")

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(lines))

    print(f"Wrote {out_path} ({total_count} glyphs across {len(fonts)} fonts)")


if __name__ == "__main__":
    main()
