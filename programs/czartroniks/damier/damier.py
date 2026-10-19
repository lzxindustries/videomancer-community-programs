#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
DAMIER - build hook.

Generates damier_pkg.vhd:

  C_DPAL     256 x 32  palette ROM, index {scheme(4), zone(2), parity, ink};
                       word = Y(10) & U(10) & V(10) & "00", generated colour
                       is stored U/V-SWAPPED (U carries Cr) for this hardware.
  C_SIN      512 x 16  quarter-wave sine (Q15) for the engine's SIN unit
  C_UCODE    512 x 32  microcode: frame sequencer + table builder
                       (damier_ucode.py, assembled by damier_uasm.py)

Palettes are authored as sRGB triples; each scheme has four colour zones of
(ground on even cells, ground on odd cells, ink on even, ink on odd).
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# ----------------------------------------------------------------------------
# palettes
# ----------------------------------------------------------------------------
K = (12, 12, 14)
WT = (238, 236, 228)


def pn(a, b):
    """positive/negative: the ink on each cell is the other cell's ground"""
    return (a, b, b, a)


SCHEMES = [
    ("Op",        [pn(K, WT)] * 4),
    ("Riley",     [pn((20, 20, 24), (225, 222, 214)), pn((96, 96, 104), (225, 222, 214)),
                   pn((20, 20, 24), (160, 158, 154)), pn((60, 60, 70), (200, 196, 190))]),
    ("Vega",      [((40, 20, 90), (230, 60, 50), (250, 200, 40), (30, 120, 200)),
                   ((230, 60, 50), (40, 20, 90), (40, 170, 110), (250, 200, 40)),
                   ((30, 120, 200), (250, 200, 40), (230, 60, 50), (40, 20, 90)),
                   ((250, 200, 40), (30, 120, 200), (40, 20, 90), (230, 60, 50))]),
    ("Kiss",      [pn((22, 18, 12), (214, 170, 70)), pn((236, 230, 214), (22, 18, 12)),
                   pn((214, 170, 70), (150, 150, 160)), pn((22, 18, 12), (236, 230, 214))]),
    ("Bauhaus",   [((235, 230, 215), (20, 20, 20), (210, 40, 35), (240, 190, 30)),
                   ((210, 40, 35), (235, 230, 215), (25, 70, 160), (20, 20, 20)),
                   ((25, 70, 160), (240, 190, 30), (235, 230, 215), (210, 40, 35)),
                   ((240, 190, 30), (20, 20, 20), (235, 230, 215), (25, 70, 160))]),
    ("Memphis",   [((250, 120, 170), (30, 30, 40), (40, 190, 180), (250, 220, 60)),
                   ((40, 190, 180), (250, 230, 220), (250, 120, 170), (30, 30, 40)),
                   ((250, 220, 60), (150, 110, 220), (30, 30, 40), (250, 120, 170)),
                   ((30, 30, 40), (250, 220, 60), (40, 190, 180), (250, 230, 220))]),
    ("Neon",      [((8, 8, 16), (240, 30, 170), (30, 230, 240), (8, 8, 16)),
                   ((8, 8, 16), (30, 230, 240), (180, 255, 40), (8, 8, 16)),
                   ((8, 8, 16), (180, 255, 40), (255, 120, 20), (8, 8, 16)),
                   ((8, 8, 16), (255, 120, 20), (240, 30, 170), (8, 8, 16))]),
    ("Pastel",    [((90, 170, 150), (252, 236, 220), (240, 150, 130), (150, 130, 210)),
                   ((240, 150, 130), (252, 236, 220), (150, 130, 210), (90, 170, 150)),
                   ((150, 130, 210), (252, 236, 220), (90, 170, 150), (240, 150, 130)),
                   ((252, 236, 220), (90, 170, 150), (150, 130, 210), (240, 150, 130))]),
    ("Seventies", [((200, 90, 30), (90, 50, 30), (230, 180, 60), (240, 225, 190)),
                   ((90, 50, 30), (230, 180, 60), (240, 225, 190), (200, 90, 30)),
                   ((230, 180, 60), (240, 225, 190), (200, 90, 30), (90, 50, 30)),
                   ((240, 225, 190), (200, 90, 30), (90, 50, 30), (230, 180, 60))]),
    ("Mondrian",  [((240, 238, 230), (20, 20, 20), (215, 35, 30), (240, 238, 230)),
                   ((240, 238, 230), (20, 20, 20), (20, 60, 170), (240, 238, 230)),
                   ((240, 238, 230), (20, 20, 20), (245, 205, 30), (240, 238, 230)),
                   ((240, 238, 230), (215, 35, 30), (20, 20, 20), (20, 60, 170))]),
    ("Delft",     [pn((20, 50, 140), (240, 240, 235)), pn((240, 240, 235), (40, 90, 190)),
                   pn((20, 50, 140), (150, 180, 230)), pn((10, 25, 80), (240, 240, 235))]),
    ("Ruby",      [pn((150, 10, 25), (20, 12, 14)), pn((240, 230, 225), (150, 10, 25)),
                   pn((20, 12, 14), (240, 230, 225)), pn((220, 40, 60), (20, 12, 14))]),
    ("Lagoon",    [pn((10, 40, 70), (40, 200, 190)), pn((240, 245, 240), (10, 40, 70)),
                   pn((40, 200, 190), (240, 245, 240)), pn((20, 110, 160), (240, 245, 240))]),
    ("Sunset",    [((90, 20, 110), (250, 120, 40), (250, 210, 60), (220, 40, 120)),
                   ((220, 40, 120), (90, 20, 110), (250, 120, 40), (250, 210, 60)),
                   ((250, 120, 40), (250, 210, 60), (220, 40, 120), (90, 20, 110)),
                   ((250, 210, 60), (220, 40, 120), (90, 20, 110), (250, 120, 40))]),
    ("Acid",      [pn((10, 10, 10), (190, 255, 30)), pn((255, 40, 160), (10, 10, 10)),
                   pn((190, 255, 30), (255, 40, 160)), pn((245, 245, 245), (10, 10, 10))]),
    ("Prism",     [pn((220, 40, 40), (20, 20, 25)), pn((240, 200, 30), (20, 20, 25)),
                   pn((40, 180, 90), (20, 20, 25)), pn((50, 90, 220), (20, 20, 25))]),
]
assert len(SCHEMES) == 16


def yuv10(rgb):
    """sRGB triple -> legal-range 10-bit BT.601 (Y, Cb, Cr)."""
    r, g, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b
    Y = int(round(64 + 876 * y))
    Cb = int(round(512 + 896 * cb))
    Cr = int(round(512 + 896 * cr))
    clip = lambda v, lo, hi: max(lo, min(hi, v))
    return clip(Y, 64, 940), clip(Cb, 64, 960), clip(Cr, 64, 960)


def pal_words():
    words = []
    for _name, zones in SCHEMES:
        for zone in range(4):
            bge, bgo, ine, ino = zones[zone]
            # index bits: c (parity) then ink
            for col in (bge, ine, bgo, ino):
                Y, Cb, Cr = yuv10(col)
                # hardware swaps U/V: U register carries Cr
                words.append((Y << 22) | (Cr << 12) | (Cb << 2))
    assert len(words) == 256
    return words


# ----------------------------------------------------------------------------
# engine ROM
# ----------------------------------------------------------------------------
def sine_words():
    """quarter-wave sine, 512 entries, Q15 (2048 steps per turn)"""
    return [int(round(32767 * math.sin(math.pi / 2 * i / 512))) for i in range(512)]


def emit():
    pal = pal_words()
    eng = sine_words()
    import damier_ucode
    from damier_uasm import emit_vhdl
    ucode = damier_ucode.build().words()

    def arr16(name, words, depth):
        lines = [f"    constant {name} : t_rom16_{depth} := ("]
        body = []
        for i in range(0, depth, 8):
            body.append("        " + ", ".join(f'x"{w & 0xFFFF:04X}"' for w in words[i:i + 8]))
        lines.append(",\n".join(body))
        lines.append("    );")
        return "\n".join(lines)

    def arr(name, words, depth):
        lines = [f"    constant {name} : t_rom32_{depth} := ("]
        body = []
        for i in range(0, depth, 4):
            body.append("        " + ", ".join(f'x"{w:08X}"' for w in words[i:i + 4]))
        lines.append(",\n".join(body))
        lines.append("    );")
        return "\n".join(lines)

    names = ", ".join(n for n, _ in SCHEMES)
    txt = f"""-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- damier_pkg.vhd -- GENERATED by damier.py; edit the generator, not this file.
--
-- C_DPAL: 16 schemes x 4 zones x (ground even, ink even, ground odd, ink odd),
--         Y & U & V & "00", U/V-SWAPPED for this hardware (U carries Cr).
--         Schemes: {names}
-- C_SIN : quarter-wave sine, 512 entries Q15, for the engine's SIN unit
-- C_UCODE: the frame sequencer + table builder microcode (damier_ucode.py)

library ieee;
use ieee.std_logic_1164.all;

package damier_pkg is
    type t_rom32_256  is array(0 to 255)  of std_logic_vector(31 downto 0);
    type t_rom16_512  is array(0 to 511)  of std_logic_vector(15 downto 0);
{arr("C_DPAL", pal, 256)}
{arr16("C_SIN", eng, 512)}
{emit_vhdl(ucode, "C_UCODE", 512)}
end package damier_pkg;
"""
    with open(os.path.join(HERE, "damier_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
