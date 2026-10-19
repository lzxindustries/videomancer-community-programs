# Stencil

Cell-averaged mosaic that eats into the picture by luma or saturation: soft stencil edges, grout, tint wash, negative reveal.

**P12 (Stencil):** key threshold, glided. 0% = only the deepest shadows are mosaic; 100% = all but the brightest highlights

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Cells | 0–100% | cell width 2 to 256 px, in 8 octave steps (deadbanded) |
| K2 | Aspect | Square / Wide / Wider / Strip | cell height = width / 1, 2, 4 or 8 (Square, Wide, Wider, Strip) |
| K3 | Grout | 0–100% | dark lines between cells, 1/16 of a cell thick (min 1 px), floored at black |
| K4 | Tint | 0–100% | 0–3% off. Above that, mosaic cells are washed toward a hue that goes once round the colour wheel |
| K5 | Soft | 0–100% | key edge, from hard to very soft, over 5 octaves |
| K6 | Source | 0–100% | key from luma (0) to saturation \|U\|+\|V\| (100), blended |
| S7 | Mask | Normal / Invert | Normal: the mosaic eats the shadows first. Invert: it eats the highlights first |
| S8 | Key From | Original / Mosaic | key from the original, or from the mosaic (cell by cell) |
| S9 | Reveal | Original / Negative | the uncovered picture is the original, or its negative |
| S10 | Luma | Mosaic / Sharp | Mosaic, or Sharp (cells keep the original luma; only colour is averaged) |
| S11 | Grout | All / Masked | grid everywhere, or only on the mosaic |
| P12 | Stencil | 0–100% | key threshold, glided. 0% = only the deepest shadows are mosaic; 100% = all but the brightest highlights |

## Presets

Tiled Shadows, Bright Tiles, Colour Key, Blocky Mask, Stained Glass, Strip Scan.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
