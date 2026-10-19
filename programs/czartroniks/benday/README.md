# Benday

Halftone print (dot, line, diamond or square screens) torn by hashed bands that shift, recolour or print the picture negative.

**P12 (Density):** moshed fraction of bands: 1/16 at 0, every band at 100% (default 30%)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Cell | 4 px / 8 px / 16 px / 32 px | halftone cell 4 / 8 / 16 / 32 px (default 16) |
| K2 | Dot Gain | 0–200% | dot weight; at 100% black prints solid and white prints clean paper |
| K3 | Screen | Dot / Line / Diamond / Square | screen shape: dot / line / diamond / square |
| K4 | Height | 8 ln / 16 ln / 32 ln / 64 ln | band height 8 / 16 / 32 / 64 lines |
| K5 | Shift | 0–100% | sideways tear of a moshed band: ±1020 px on HD-width lines, ±510 on SD |
| K6 | Rate | 0–100% | re-roll rate, from every ~2 s to every frame |
| S7 | Ink | Dark / Light | Dark: black dots on white paper. Light: white dots on black (both positive) |
| S8 | Colour | Mono / Colour | Mono / the bright side (paper, or the light ink) takes the cell colour at 2× saturation |
| S9 | Mosh | Picture / Screen | Picture: bands tear the picture. Screen: bands shift the screen phase and print the next screen pattern |
| S10 | Band FX | Colour / Negative | Colour: bands carry 32-px held, U/V-swapped, 2× cell colour. Negative: bands print negative |
| S11 | Band Fill | Halftone / Flat | Halftone / Flat cells (macroblocks) in the bands |
| P12 | Density | 0–100% | moshed fraction of bands: 1/16 at 0, every band at 100% (default 30%) |

## Presets

Default, Newsprint, Torn Comic, Line Screen, Pop Dots.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
