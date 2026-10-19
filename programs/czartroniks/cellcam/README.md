# Cellcam

Live pixelation zoom: cells outlined by the luma steps around them (or raw edges), with held, live or smeared chroma.

**P12 (Cell):** cell size 2..129 px, quadratic (10 px at 25%, 34 at 50%, 74 at 75%). The grid is centred on the screen, so the slider zooms the pixelation about the centre

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Crawl | 0–100% | NTSC dot crawl on the edges. Past 50% it spreads over the whole picture (0 = clean) |
| K2 | Shape | -100–100% | cell height as a ratio of its width, exponential: 1/4 (flat) at −100, square at 0, ~4× (tall) at +100 |
| K3 | Gain | 0–100% | edge gain, exponential: 1/8x at 0, 1/2x at 25%, 2x at 50%, 8x at 75%, 31x at 100% |
| K4 | Width | 1–8px | outline width 1..8 px on the left side of each cell, 1..8 lines on the top. Raw: edges thickened Width px right and Width lines down |
| K5 | Hold | 1–16x | chroma held across 1..16 cells (1 = each cell its own colour) |
| K6 | Smear | 0–100% | chroma lag 0..127 px; past 25% the chroma also bleeds toward each new block (8 / 16 / 32 px at 25 / 50 / 75%, doubled at HD) |
| S7 | Edges | Over / Only | Over: drawn over the cells / Only: edges alone on plain neutral paper |
| S8 | Source | Cells / Raw | Cells: steps between cells (left and top outlines) / Raw: gradient of the live picture, \|dx\| + \|dy\| |
| S9 | Chroma | Cells / Live | Cells: colour from the stored cell row / Live: colour latched from the current line, so it flows down inside the held luma blocks |
| S10 | Ink | Black / White | black / white |
| S11 | Tint | Colour / Mono | edges keep the cell colour / edges mono |
| P12 | Cell | 0–100% | cell size 2..129 px, quadratic (10 px at 25%, 34 at 50%, 74 at 75%). The grid is centred on the screen, so the slider zooms the pixelation about the centre |

## Presets

Default, Comic Cells, White Wire, Raw Crawl, Live Blocks, Thick Sketch.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
