# Rutt Etra

A classic Rutt/Etra scan processor: the picture is redrawn as horizontal scanlines pushed up by brightness, with hidden-line terrain occlusion. The slider raises the relief.

**P12 (Relief):** Glided depth of the luma displacement, from flat scanlines to deep terrain.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scan Lines | 0–100% | Number of scan lines (clockwise = denser) |
| K2 | Thickness/Glow | 1–8 | Line thickness; from 3 up, a glow halo fades in around the lines |
| K3 | Line Color | White / Green / Amber / Cyan / Magenta / Video / Fire / Terrain | Line colour: white, green, amber, cyan, magenta, ice, fire, terrain |
| K4 | Contrast | 0–200% | Input contrast feeding line height and shading |
| K5 | Roll | -100–100% | Scrolls the line grid up or down; centre = still |
| K6 | Smoothing | 0–100% | Horizontal smoothing of the line shapes |
| S7 | Peaks | Bright / Dark | Bright: bright areas rise; Dark: dark areas rise |
| S8 | Hidden Line | Off / On | Off: see-through lines; On: nearer lines hide ones behind |
| S9 | Shading | Banded / Luma | Banded: two-level shading; Luma: shading follows brightness |
| S10 | Roll Motion | Run / Freeze | Run: Roll keeps moving; Freeze: holds the grid where it is |
| S11 | Input | Positive / Negative | Positive: normal input; Negative: inverted input |
| P12 | Relief | 0–100% | Relief: how high brightness lifts the lines |

## Presets

Green Phosphor, Topographic, Flyover, Deep Cut.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
