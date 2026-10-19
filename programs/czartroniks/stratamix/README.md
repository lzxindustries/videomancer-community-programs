# Stratamix

Luma is depth: the picture shifts by its own brightness and is blended with the still one; posterized strata, contours, tints.

**P12 (Strata):** depth and mix rise together (glided)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Depth | -100–100% | parallax gain, bipolar, glided |
| K2 | Pivot | 0–100% | luma level that does not move |
| K3 | Levels | 2 / 4 / 8 / 16 / 32 / 64 | posterize levels: 2 / 4 / 8 / 16 / 32 / 64 (sixths of the travel; default 8) |
| K4 | Tint | 0–100% | chroma tint per luma band |
| K5 | Mode | Add / Screen / Multiply / Difference / Overlay | Add / Screen / Multiply / Difference / Overlay (fifths of the travel, as the display shows) |
| K6 | Mix | 0–100% | how far the blend replaces the displaced layer |
| S7 | Order | Before / After | posterize before the shift (terraces) / after the mix |
| S8 | Layers | Same / Alternate | same / alternate levels shift opposite ways |
| S9 | Chroma | Shifted / Still | colour from the displaced layer / from the still picture |
| S10 | Key | All / Above | blend everywhere / only above the pivot |
| S11 | Contour | Plain / Lines | plain / black contour bands at every level boundary |
| P12 | Strata | 0–100% | depth and mix rise together (glided) |

## Presets

Default, Terraces, Colour Ghost, Relief Map.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 32 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
