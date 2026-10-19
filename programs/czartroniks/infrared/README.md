# Infrared

Keystone/corner-pin meets thermal-camera false colour: 8 palettes (thermal, ironbow, plasma, rainbow, ice, night vision, white hot, x-ray) with range, offset, isotherms, row drift and a Heat slider that pushes the scene hotter or cycles the palette.

**P12 (Heat):** Cycle off: pushes the whole scene up the palette (glided; 3/4 of the ramp at 100% = white-hot with orange shadows). Cycle on: palette cycling speed on a squared curve (~8 s a lap at 25%, ~0.5 s at 100%), seamless.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Keystone | -100–100% | Keystone: top and bottom widths differ |
| K2 | Skew | -100–100% | Slants the picture into a parallelogram, up to half the width |
| K3 | Zoom | -100–100% | Horizontal zoom (0.25x to 4x, fine steps) |
| K4 | Palette | Thermal / Ironbow / Plasma / Rainbow / Ice / Night Vision / White Hot / X-Ray | Thermal-camera palette, 8 choices |
| K5 | Range | 0–400% | Contrast into the palette; 100% = unity |
| K6 | Offset | -100–100% | Shifts which colours brightness maps to, by up to one whole ramp |
| S7 | Rows | Flat / Drift | Off: flat; On: palette heats up down the screen |
| S8 | Iso | Off / Lines | Off: normal; On: black isotherm lines every eighth of the ramp |
| S9 | Cycle | Off / On | Off: Heat pushes the scene hotter; On: Heat sets the palette cycling speed |
| S10 | Luma | Palette / Original | Off: palette sets brightness too; On: keep original brightness |
| S11 | Ramp | Normal / Invert | Off: normal; On: brightness-to-palette mapping reversed |
| P12 | Heat | 0–100% | Heat push (Cycle off) or cycling speed (Cycle on) |

## Presets

Default, Heat Map, Cold Wall, X-Ray Tilt, Ironbow Burn, Night Scope, Rainbow Spin.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
