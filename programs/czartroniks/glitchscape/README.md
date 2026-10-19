# Glitchscape

A pixel-sort databender: bright parts of the source become a mass of vertical jewel-tone streaks bending around the image on a flat sky (teal or a matching colour per palette), with blown-white hot zones, white contour lines, a torn top edge, and coloured bars that pour down the frame.

**P12 (Glitch Intensity):** Master meltdown, fully continuous: bar bend, ripple, row wander, edge tear, fringe width, bloom, hot-zone reach, drip length (about 120 lines to the full screen) and grain together, from lightly streaked source to full corruption.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Mass Key | 0–100% | Brightness level above which the coloured glitch mass appears |
| K2 | Contrast | 0–100% | Contrast: crushes the dark end of the source before keying |
| K3 | Saturation | 0–400% | Colour intensity of the streaks, smooth (100% at a quarter turn) |
| K4 | Palette | Jewel / Neon / Vaporwave / Inferno / Toxic / Glacier / Candy / Mono | Streak palette; each one brings its own sky colour |
| K5 | Line Spacing | 0–100% | Streak bar spacing, 64 px wide to 4 px tight at 1080p, scaled to the screen height in other modes (smooth) |
| K6 | Filaments | 0–100% | Number of white contour lines tracing the image's brightness |
| S7 | Channel Sep | Off / On | On: colour fringing split, 4-7 px at 1080p (width follows the slider) |
| S8 | Hot Zones | Off / On | On: blown-white cores and bloom at the brightest spots |
| S9 | Drips | Off / On | On: coloured bars pour down the frame (vertical pixel sort) |
| S10 | Compress | Off / On | On: JPEG-style macroblocks and posterize |
| S11 | Sort Axis | Vertical / Horizontal | Streaks run vertically or horizontally |
| P12 | Glitch Intensity | 0–100% | Overall glitch amount: lightly streaked to full meltdown |

## Presets

Teal Meltdown, Clean Streaks, Neon Overload, Vaporwave Drift.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
