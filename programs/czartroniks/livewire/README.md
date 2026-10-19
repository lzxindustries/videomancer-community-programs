# Livewire

Edges as a living material: the picture's outlines become neon tubes, a crackling Kirlian corona, an oil-slick spectral swirl, painted cel shading or bleeding wet ink. The slider emerges the material from clean video up to a frame made entirely of edges.

**P12 (Emerge):** The signature sweep: clean video, then coarse edges over video, then the interior fades to the mode's ground, full line weight, and finally overdrive where the whole frame becomes edge material.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Mode | Neon / Kirlian / Spectral / Cel / Ink Bleed | Which edge material (shows 0-4 on the device) |
| K2 | Scale | 0–100% | Fine to coarse edge detection |
| K3 | Thresh | 0–100% | Edge sensitivity |
| K4 | Material | 0–100% | Line weight and how far the material spreads |
| K5 | Character | 0–100% | Per-mode: halo hue, swirl speed, filament scale or cel levels |
| K6 | Colour | 0–100% | Palette and hue offset |
| S7 | Invert | Normal / Inverted | True video negative |
| S8 | Interior | Material / Video | Fill between edges with the material ground or the video |
| S9 | Polarity | Both / One Side | Light both sides of an edge, or only one |
| S10 | Rainbow | Off / On | Spectral rainbow overlay (in Spectral mode: flat palette) |
| S11 | Persist | Halo / Trails | Glow halo, or motion trails that hang |
| P12 | Emerge | 0–100% | Clean video to full edge-material overdrive |

## Presets

Neon Sign, Cel Shade, Oil Slick, Comet Trails.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
