# Snowglobe

Pixelated cells (lens or screen space) through a sphere or tunnel lens, with per-cell or fine snow and lost-tracking lines.

**P12 (Squeeze):** horizontal scale, 50% (2× stretch) to 200% (two picture widths across), glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Cell | 1 px / 2 px / 4 px / 8 px / 16 px / 32 px / 64 px / 128 px | cell width 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 px, deadbanded |
| K2 | Rows | 1 line / 2 lines / 4 lines / 8 lines / 16 lines / 32 lines / 64 lines / 128 lines | cell height 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 lines, deadbanded |
| K3 | Snow | 0–100% | snow amount. Above about 60% the picture also fades toward grey (25% left at 100%), like a lost channel |
| K4 | Tracking | 0–100% | fraction of lines that lose tracking (torn, darkened, desaturated) |
| K5 | Bulge | -100–100% | lens strength, bipolar (pinch .. bulge), glided |
| K6 | Radius | 3–140% | lens radius, 3–140% of the half-height, glided |
| S7 | Cells | Lens / Screen | cells in lens (source) space / in screen space |
| S8 | Snow | Fine / Cells | Fine (per pixel) / Cells (one value per picture cell, in both S7 modes) |
| S9 | Lens | Sphere / Tunnel | Sphere / Tunnel |
| S10 | Edges | Black / Mirror | Black / Mirror (once, about a 12-px inset, black beyond) |
| S11 | Snow | Mono / Colour | Mono: grey snow / Colour: coloured snow |
| P12 | Squeeze | 50–200% | horizontal scale, 50% (2× stretch) to 200% (two picture widths across), glided |

## Presets

Default, Chunky Storm, Mosaic Tunnel, Bad Reception, Lost Channel.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
