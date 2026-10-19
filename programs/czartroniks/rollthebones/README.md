# Roll the Bones

Video re-rendered as a lit tabletop of six-sided dice: each cell is a die whose pip count follows the picture, so the field reads as a halftone portrait. The dice have domed tops, glossy corners and recessed pips under a movable light, and the slider spills the grid into a rotated, shadow-casting pile.

**P12 (Tumble):** 0% = a clean aligned grid; up the slider the dice shrink to open gaps of table, rotate by a per-die random angle, jitter off centre and cast drop shadows, ending in a rolled, spilled pile.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Zoom | 18–150px | Die size |
| K2 | Palette | 1–8 | Ink: Casino White, Blood Red, Onyx, Ivory, Jade, Gold, Rainbow or Spectrum |
| K3 | Hue | 0–360deg | Rotates the colour wheel (sets saturation under Ink=Video) |
| K4 | Light | 0–360deg | Direction of the light on the dice |
| K5 | Gloss | 0–100% | Matte to hard glossy highlights |
| K6 | Contrast | 0–100% | How strongly the picture sets the pip counts |
| S7 | Source | Luma / Chroma | Pip count follows brightness or colour saturation |
| S8 | Luma Size | Off / On | Bright areas get bigger dice, dark areas smaller |
| S9 | Table | Felt / Video | Green felt or dimmed video between the dice |
| S10 | Ink | Painted / Video | Die bodies in the palette colour or tinted with the picture |
| S11 | Bypass | Off / On | Passes the input video straight through |
| P12 | Tumble | 0–100% | Neat grid to a rolled, rotated, shadowed pile |

## Presets

Casino Table, Spilled Bones, Onyx Grid, Rainbow Dust, Video Dice.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
