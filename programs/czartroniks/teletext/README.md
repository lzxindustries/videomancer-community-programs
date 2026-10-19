# Teletext

Video redrawn as 8x8 ASCII or block-dither glyphs per cell, with datamosh bars and per-line shear tearing the glyph raster.

**P12 (Reveal):** crossfade from the moshed, sheared video to the glyph rendering

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Bars | 0–100% | fraction of glyph bands displaced (datamosh), re-rolled every frame |
| K2 | Distance | 0–1020px | band displacement, up to ±1020 px; wraps inside the picture |
| K3 | Cell | 8 px / 16 px / 32 px / 64 px | glyph cell 8 / 16 / 32 / 64 px (default 8) |
| K4 | Contrast | 0–400% | luma → glyph density; 100% maps white to the densest glyph, up to 400% |
| K5 | Shear | 0–30px | per-line sideways shear, ±30 px |
| K6 | Rate | 0–100% | Wave: wave speed. Tear: fraction of lines torn |
| S7 | Ink | White / Black | white glyphs on black / black glyphs on white |
| S8 | Colour | Mono / Cell | mono glyphs / glyphs take the cell's colour |
| S9 | Mosh | Video / Cells | bands displace the video / bands displace whole glyph cells |
| S10 | Shear | Wave / Tear | Wave (triangle) / Tear (random per line) |
| S11 | Glyphs | ASCII / Blocks | ASCII ramp / block-dither set |
| P12 | Reveal | 0–100% | crossfade from the moshed, sheared video to the glyph rendering |

## Presets

Default, Green Screen, Torn Blocks, Paper Print.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 60 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
