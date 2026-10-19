# Edge Lab

Video redrawn as edge lines in two styles, Fine (thin connected Canny tracing) or Chalk (thick even outlines), with four colour modes and a live Sensitivity slider.

**P12 (Sensitivity):** Live edge threshold: up finds more edges; 100% dissolves the picture into dense edge grain.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Thickness | 1 px / 2 px / 3 px / 4 px / 5 px | Edge line thickness, 1–5 px at 1080p (scaled to the same screen proportion at 720p and SD) |
| K2 | Mix | 0–100% | Crossfade between the original picture and the edge drawing |
| K3 | Brightness | -100–100% | Brightens or darkens the picture before edges are found |
| K4 | Contrast | 0–400% | Contrast before edge detection; 25% = unchanged |
| K5 | Posterize | Off / 128 / 64 / 32 / 16 / 8 / 4 / 2 | Posterizes the picture before detection (fewer levels = bolder contours) |
| K6 | Color | White / Compass / Source / Heat | Line colour: white, by edge direction (compass), the source colour, or heat by strength |
| S7 | Smooth | Off / On | On: softens the picture first so fewer tiny edges appear |
| S8 | Invert | Off / On | On: negative, draws the not-edges and inverts the colour mode |
| S9 | Style | Fine / Chalk | Fine: thin connected tracing; Chalk: thick, even chalky outlines |
| S10 | Overlay | Off / On | On: edges drawn over the live picture instead of black |
| S11 | Output | Binary / Gradient | Binary: solid lines; Gradient: edge strength as shading |
| P12 | Sensitivity | 0–100% | Sensitivity: up = more edges; 100% dissolves the picture into dense edge grain |

## Presets

Contour Map, Ghost Contours, Hairline Sketch.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
