# Gumball

Video pixelated into a hex-packed wall of glossy 3D spheres with movable lighting; squash presses the balls flat against each other into a domed mosaic.

**P12 (Relief):** Flat unlit hex mosaic at 0, rising to deep-shadow wet-gloss spheres (shading depth and specular together).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Ball Size | 12–128px | Size of the balls |
| K2 | Light Angle | 0–360deg | Direction the light comes from |
| K3 | Specular | 0–100% | Brightness of the shiny highlight |
| K4 | Shine | 0–100% | Highlight tightness: broad sheen to pinpoint gloss |
| K5 | Squash | 0–100% | Balls separated, touching, or squashed together |
| K6 | Candy | 75–270% | Colour saturation of the balls |
| S7 | Backdrop | Black / Video | Black or dimmed video behind the balls |
| S8 | Luma Size | Off / On | On: bright areas make bigger balls, dark smaller |
| S9 | Packing | Hex / Square | Hex offset rows or straight square grid |
| S10 | Chrome | Off / On | On: removes colour, chrome ball-bearings |
| S11 | Bypass | Off / On | On: shows the raw input unchanged |
| P12 | Relief | 0–100% | Flat mosaic tiles up to deep shiny 3D balls |

## Presets

Glass Beads, Gumball Machine, Ball Bearings, Pressed Mosaic.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
