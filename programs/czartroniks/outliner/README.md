# Outliner

Edge lines on one side of a seam or in strips, a negative or bit-plane on the other; the slider sweeps the seam.

**P12 (Sweep):** seam swings on a sine about K3; swing up to ± the full width, ~1.7 s a cycle at 100%

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Threshold | 0–100% | edge threshold, curved (fine steps at the low end) |
| K2 | Kernel | Roberts / Sobel / Wide / DoG | Roberts 2×2 / Sobel 3×3 / Wide (8-px span, lines thickened 3 rows down) / DoG (5- vs 17-px boxes + vertical Laplacian) |
| K3 | Seam | 0–100% | split position, a fraction of the measured width |
| K4 | Panes | Split / 2 strips / 4 strips / 8 strips | Split / 2 / 4 / 8 strips alternating edge side and other side |
| K5 | Plane | Negative / Bit 8 / Bit 7 / Bit 6 / Bit 5 / Bit 4 / Bit 3 / Bit 2 | Negative / Bit 8 … Bit 2 |
| K6 | Mix | 0–100% | how far the invert side goes to its negative or plane (default 100%) |
| S7 | Edges | On Black / Over | edge side: white lines on black / lines over the picture |
| S8 | Invert Side | Edge Side / Other Side | the edge side / the other side is inverted |
| S9 | Lines | One Side / Both | lines on the edge side only / on both sides |
| S10 | Invert | Area / Edges Only | the whole area / only edge pixels (lines take the inverted picture) |
| S11 | Line Colour | White / Source | white lines / lines with the source chroma |
| P12 | Sweep | 0–100% | seam swings on a sine about K3; swing up to ± the full width, ~1.7 s a cycle at 100% |

## Presets

Default, Blueprint, Strip Negative, Bit Sweep.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
