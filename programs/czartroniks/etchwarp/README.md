# Etchwarp

Edge lines over a noise-warped, pixelated picture; the noise can move the cells or the content, and the lines can trace the blocks.

**P12 (Flow):** Adds noise displacement amount and scroll speed together

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Threshold | 0–100% | Edge sensitivity |
| K2 | Ink | 0–100% | Strength of the edge lines |
| K3 | Noise | 0–100% | How much the wobbly noise pushes the picture |
| K4 | Speed | 0–100% | How fast the noise pattern drifts |
| K5 | Cells | 0–100% | Pixel block width (1 to 64 px at 1080p; the same size relative to the screen height at 720p and SD) |
| K6 | Rows | Square / Wide / Wider / Strip | Pixel block shape: square to wide strips |
| S7 | Lines | Only / Over | Off: edges only on black; On: edges over the picture |
| S8 | Noise Moves | Content / Cells | Off: noise moves the content; On: noise moves the block grid |
| S9 | Pixelate | All / Luma | Off: pixelate everything; On: pixelate brightness only |
| S10 | Line Colour | White / Source | Off: white edge lines; On: edges take picture colour |
| S11 | Edges From | Smooth / Blocks | Off: edges from smooth picture; On: from pixelated one |
| P12 | Flow | 0–100% | Noise flows faster and stronger |

## Presets

Default, Block Etch, Boiling Grid, Sharp Colour.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
