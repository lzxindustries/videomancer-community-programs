# Aurora

A big, smooth rainbow wave travelling across the screen. Swell grows it from a ripple to crests at the screen edges, Noise makes the path and thickness organic, and Mirror folds it into a seamless reflection, like a rainbow in water.

**P12 (Swell):** Wave amplitude, from a gentle ~3% ripple up to crests touching the top and bottom of the screen. With Height at 100% the whole screen becomes a waving rainbow wall.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Height | 0–100% | Band thickness, from a hairline to the whole screen |
| K2 | Waves | 0–100% | Wavelength: 0.5 to 5 waves across the screen |
| K3 | Speed | -100–100 | Wave travel; centre = still, positive = rightward |
| K4 | Noise | 0–100% | Organic wobble of the path and thickness; 0 = pure sine |
| K5 | Flow | -100–100 | Rainbow drift; centre = stopped |
| K6 | Spectrum | 50–400% | How many rainbow cycles across the band (or screen) |
| S7 | Color | Gradient / Solid | Continuous spectrum, or 7 pure colour stripes |
| S8 | Backdrop | Black / Video | Wave over black or over the live picture |
| S9 | Fill | Rainbow / Video | Band brightness from the rainbow, or from the live picture |
| S10 | Rainbow | Across / Along | Spectrum runs across the band, or along the screen width |
| S11 | Mirror | Off / On | Top half becomes a seamless reflection of the bottom |
| P12 | Swell | 0–100% | Wave amplitude: gentle ripple to crests at the screen edges |

## Presets

First Light, Tidal Wall, Braid, Over the Air.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
