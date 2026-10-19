# Solarcell

Colour-averaged mosaic cells, solarized before or after averaging; staircase threshold per row, contour lines, grid, vivid.

**P12 (Fold):** the fold threshold: 0% folds the highlights only, 100% folds the whole picture

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Level | 15–80% | fold brightness: the bands top out at 15–80% of white (default 58%) |
| K2 | Gain | 100–1600% | fold depth ×1 → ×16 |
| K3 | Rows | 1–64ln | cell height 1–64 lines |
| K4 | Cell | 4–131px | cell width 4–131 px; each cell takes its exact mean colour |
| K5 | Mix | 25–100% | mosaic over the picture, 25–100% |
| K6 | Offset | 0–100% | threshold step per cell row (a staircase) |
| S7 | Chroma | Keep / Fold | Keep: the fold keeps the colour / Fold: it inverts the colour too |
| S8 | Order | Solar-Mos / Mos-Solar | Solar-Mos: solarize, then mosaic / Mos-Solar: mosaic, then solarize |
| S9 | Colour | Average / Vivid | Average / Vivid (cell colours at double saturation) |
| S10 | Grid | Off / On | black cell borders |
| S11 | Iso | Off / On | contour lines in negative ink: black over light, light over dark, with the complementary hue |
| P12 | Fold | 0–100% | the fold threshold: 0% folds the highlights only, 100% folds the whole picture |

## Presets

Default, Staircase, Iso Mosaic, Fine Fold, Stained Glass.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
