# Trapquant

Straight-sided keystone (top/bottom width, slide, tilt, mirror tiles) posterized with centred dither; bands ride the trapezoid.

**P12 (Tilt):** ±100% width: up narrows the top and widens the bottom (floor 25%), glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Top | 25–225% | width of the picture at the top, 25..225% (the display shows the real width), glided |
| K2 | Bottom | 25–225% | width at the bottom, 25..225%, glided |
| K3 | Slide | -50–50% | horizontal slide, ±½ of the measured width, in source space (wide rows move further: the trapezoid shears), glided |
| K4 | Luma | 1–8 b | luma depth 1..8 bits; levels always include black and white |
| K5 | Chroma | 1–8 b | chroma depth 1..8 bits; centred, so greys stay grey |
| K6 | Dither | 0–100% | dither amount, 0..100% of a step, centred (no brightness shift) |
| S7 | Edges | Fill / Mirror | Fill (the row's own edge pixel) / Mirror (mirrored tiles at any width) |
| S8 | Order | Source / Screen | dither cells keyed to the source (they stretch with the keystone) / to the screen (they stay put) |
| S9 | Ramp | Off / On | quantizer thresholds shift with each row's scale; bands slide as the trapezoid moves |
| S10 | Dither | 2x2 / Lines | 2x2 Bayer (2-px cells) / alternate lines |
| S11 | Chroma | Poster / Keep | posterized / kept |
| P12 | Tilt | -100–100% | ±100% width: up narrows the top and widens the bottom (floor 25%), glided |

## Presets

Default, Road Ahead, Screen Bands, Soft Lean.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
