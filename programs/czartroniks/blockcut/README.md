# Blockcut

Sobel edge drawing and picture cut into hashed blocks that shift, hold, invert or borrow colour; density on the slider.

**P12 (Density):** fraction of blocks moshed, ~5% .. ~98%; half as many again are colour-only

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Gain | 0–100% | edge gain (Sobel edge strength) |
| K2 | Thresh | 0–100% | edges below this are dropped |
| K3 | Blend | 0–100% | edges on black (0) .. edges over the cut-up picture (100) |
| K4 | Block | 8–64 px | block size 8..64 px, continuous |
| K5 | Shift | 0–100% | moshed-block displacement, ±48..±1008 px HD (±24..±504 SD), wraps inside the picture |
| K6 | Rate | 0–100% | re-roll every ~2 s .. every field, exponential |
| S7 | Edges | White / Tinted | white / tinted (the picture's colour at double saturation) |
| S8 | Blocks | Plain / Negative | plain / moshed blocks go negative (luma and chroma inverted) |
| S9 | Hash | Block / Band | per block / per band (whole rows of blocks share a shift) |
| S10 | Mosh | Shift / Hold | shift / hold (block repeats its first column) |
| S11 | Ink | Light / Dark | light edges on black / dark edges on white |
| P12 | Density | 0–100% | fraction of blocks moshed, ~5% .. ~98%; half as many again are colour-only |

## Presets

Default, Shattered, Smeared Ink, Bar Code.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
