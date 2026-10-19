# Porthole

A round or square porthole over the picture: a lens bulges or pinches it, noise ripples it, Drift swoops it in a figure-eight.

**P12 (Drift):** figure-eight swing; swing (to half the screen) and speed grow with the slider; 100% = edge-to-edge swoop every ~2 s; glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Curve | -100–100% | lens strength, bipolar: CCW bulges (fisheye), CW pinches; fitted to the window's rows |
| K2 | Zoom | -100–100% | 0.25×–4×, log, 256 steps; CW magnifies |
| K3 | Noise | 0–100% | value-noise displacement amount (64×32 lattice) |
| K4 | Speed | 0–100% | lattice scroll speed, continuous, 0–16 lines a frame |
| K5 | Size | 10–100% | window width 10–~98% of the picture; height follows the screen aspect |
| K6 | Position | 0–100% | window centre column |
| S7 | Outside | Original / Warped | original / warped picture outside the window |
| S8 | Noise | Window / Everywhere | window only / everywhere |
| S9 | Frame | None / Black | none / black rim (thickness follows the window) |
| S10 | Shape | Rect / Round | Rect / Round porthole (diameter = Size, aspect-corrected; exact ring rim) |
| S11 | Window | Lens / Fit | Lens: magnifies what is behind it / Fit: the whole picture squeezed in |
| P12 | Drift | 0–100% | figure-eight swing; swing (to half the screen) and speed grow with the slider; 100% = edge-to-edge swoop every ~2 s; glided |

## Presets

Default, Ship Window, Funhouse, Wide Lens, Magnifier, Swoop.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
