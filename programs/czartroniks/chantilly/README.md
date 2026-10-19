# Chantilly

Video re-rendered as black Chantilly lace: cream dots on concentric arcs in fans, rosettes and pebble rows (never on a grid), cut by a scalloped black cordonnet and inked along the picture's edges. Dot size follows the picture, so it reads as a lace halftone.

**P12 (Scale):** Zooms the lace over about 1.75 octaves either way of the default: up = bigger lace and fatter dots, down = a fine dense veil.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Fill | 0–100% | Minimum dot size |
| K2 | Contrast | 0–100% | How strongly brightness grows the dots |
| K3 | Flow | -100–100% | Scroll and spin the lace; centre = still |
| K4 | Ink | 0–100% | Black ink strokes along the picture's edges (bottom = none) |
| K5 | Border | 0–100% | Thickness of the scalloped black cordonnet |
| K6 | Twist | -100–100% | Spirals the dot rings (phyllotaxis shear) |
| S7 | Tint | Mono / Ivory | Pure white dots or warm ivory |
| S8 | Ground | Black / White | Cream lace on black, or black net on white |
| S9 | Colour | Off / On | Dots carry the picture's own colour |
| S10 | Weave | Doily / Pebble | Fans and rosettes, or a calmer pebble set |
| S11 | Ghost | Off / On | Dim video shows through the ground |
| P12 | Scale | 0–100% | Lace size, from fine veil to bold doily |

## Presets

Black Lace, White Lace, Fine Mesh, Bold Doily, Video Lace.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
