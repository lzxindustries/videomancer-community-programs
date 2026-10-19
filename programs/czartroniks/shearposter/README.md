# Shearposter

Posterized, dithered pixel cells torn by a rolling zigzag or random tears; cells lean or slide, colour can split away.

**P12 (Shear):** amount, glided, squared: width/64 at 0 (30 px HD), ±255 px at ~50%, width/2 (±958 px HD) at 100%

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Cell | 1 px / 2 px / 4 px / 8 px / 16 px / 32 px / 64 px / 128 px | cell width 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 px, deadbanded |
| K2 | Rows | 1 line / 2 lines / 4 lines / 8 lines / 16 lines / 32 lines / 64 lines / 128 lines | cell height 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 lines, deadbanded |
| K3 | Dither | Off / 2 px / 4 px / 8 px / 16 px / 32 px / 64 px / Cell | 2×2 ordered dither: Off / 2 / 4 / 8 / 16 / 32 / 64 px / Cell (one threshold per cell; it moves with the shear) |
| K4 | Rate | 0–100% | Zigzag: waves down the frame, squared. 0 = the whole frame sways as one; the default gives 38 waves on a 1080 frame; the top is an 8-line zigzag. Tear: share of lines torn, squared, 1.6–100%. Glided |
| K5 | Luma | 1 bit / 2 bits / 3 bits / 4 bits / 5 bits / 6 bits / 7 bits / 8 bits | luma levels 1–8 bits |
| K6 | Chroma | 1 bit / 2 bits / 3 bits / 4 bits / 5 bits / 6 bits / 7 bits / 8 bits | chroma levels 1–8 bits (1 bit = −/grey/+ per channel) |
| S7 | Shear | Zigzag / Tear | Zigzag (a triangle wave rolling down the frame, so straight shear segments) / Tear (a random offset per line, new every field) |
| S8 | Shift | Free / Cells | Free / Cells (offsets rounded to whole cells: the cell grid stays on the screen and the picture steps a cell at a time) |
| S9 | Cells | Lean / Slide | Lean (every line takes its own offset, so cells lean) / Slide (one offset per cell row, so rows slide as rigid strips) |
| S10 | Edges | Fill / Mirror | Fill (the line's own edge pixel) / Mirror |
| S11 | Colour | Together / Split | Together / Split (chroma shears the opposite way, so colour tears away from the picture) |
| P12 | Shear | 0–100% | amount, glided, squared: width/64 at 0 (30 px HD), ±255 px at ~50%, width/2 (±958 px HD) at 100% |

## Presets

Default, Torn Mosaic, Sliding Cells, Two Bit Wave, Misprint, Shredder.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
