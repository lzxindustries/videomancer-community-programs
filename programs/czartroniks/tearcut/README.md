# Tearcut

Edge-detected line drawing torn per band or waved, with moshed blocks, inked over paper or the untorn picture.

**P12 (Blend):** edges on paper (0) .. edges over the untorn picture (100)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Gain | 0–100% | edge gain |
| K2 | Thresh | 0–100% | edges below this are dropped |
| K3 | Block | 8 px / 16 px / 32 px / 64 px | block size 8 / 16 / 32 / 64 px |
| K4 | Density | 5–98% | fraction of blocks moshed, ~5% .. ~98% |
| K5 | Tear | 8–518 px | tear amplitude, ±8 .. ±518 px |
| K6 | Bands | 2 ln / 4 ln / 8 ln / 16 ln | tear band height 2 / 4 / 8 / 16 lines (Random) or wave period 64 / 128 / 256 / 512 lines (Wave) |
| S7 | Edges | White / Tinted | white / tinted (the picture's colour at double saturation) |
| S8 | Tear | Random / Wave | random per band / triangle wave down the picture |
| S9 | Blocks | Plain / Negative | plain / moshed blocks go negative (luma and chroma inverted) |
| S10 | Mosh | Shift / Hold | shift / hold (block repeats its first column) |
| S11 | Ink | Light / Dark | light edges on dark / dark edges on light |
| P12 | Blend | 0–100% | edges on paper (0) .. edges over the untorn picture (100) |

## Presets

Default, Torn Ink, Wavy Neon, Held Blocks.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
