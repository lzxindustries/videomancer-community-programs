# Sidewinder

The picture snakes left and right down the screen as a rolling triangle or sine wave. The slider takes it from a gentle sway to the picture coiling round a tube; Twist bunches the bends down the screen, Split fans the colour out like a prism, and edges mirror or wrap.

**P12 (Amount):** Bend amplitude as a fraction of the screen width: a gentle 3% sway at 0, up to half the width at 100%, where Wrap coils the picture round a tube.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Period | 0–100% | Vertical wavelength of the bends (glided) |
| K2 | Rate | -100–100% | Roll speed and direction; centre = frozen |
| K3 | Ripple | 0–100% | 3x harmonic on top of the main wave, up to 1/8 of the width |
| K4 | Twist | -100–100% | Wavelength changes down the screen about the centre row; centre = even |
| K5 | Split | 0–100% | Chroma dispersion: U bends further than the picture, V less |
| K6 | Mix | 0–100% | Dry/wet between the straight and the warped picture |
| S7 | Shape | Triangle / Sine | Zigzag or smooth bends |
| S8 | Swell | Off / On | Slow automatic breathing of Amount (~6 s) |
| S9 | Steps | Smooth / Stepped | Offsets snap to 16 px (at 1080p; scaled in other modes): staircase band shred |
| S10 | Edge | Mirror / Wrap | Off-edge reads reflect, or the screen becomes a tube |
| S11 | Bypass | Off / On | Straight picture at the same latency |
| P12 | Amount | 0–100% | Bend amplitude: a gentle 3% sway up to half the width |

## Presets

Gentle Sway, Heat Haze, Seasick, Band Shredder, Slow Serpent, Prism, Rolling Road, Coil.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
