# Inferno

Pixelated two-layer noise fire: chunky blocks in a 32-step fire palette, rolling smoke, a blue/violet flame mode, and a Luma Mod mode that sets every edge in the video on fire.

**P12 (Intensify):** 1x-5x boost on fuel height, rise speed and turbulence together - from a calm flame to a roaring blaze.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Fuel | 0–100% | Flame height and raggedness |
| K2 | Pixels | 0–100% | Block size, from fine (about 8 px) to chunky (about 70 px at 1080p) |
| K3 | Smoke | 0–100% | Amount and height of smoke above flames |
| K4 | Turblnce | 0–100% | Turbulence: how much the flames twist |
| K5 | Depth/Edge | 0–100% | Luma Mod off: depth of the second fire layer; Luma Mod on: how sensitive edge detection is |
| K6 | Rise | 0–100% | Speed flames rise |
| S7 | Ember | Off / On | On: random hot ember pops in the flames |
| S8 | Luma Mod | Off / On | On: fire pours off bright objects' edges |
| S9 | Blue Flame | Off / On | On: blue gas flames with violet background |
| S10 | Overlay | Off / On | On: fire drawn over the incoming video |
| S11 | Bypass | Off / On | On: shows the raw input unchanged |
| P12 | Intensify | 1–5x | Intensify: bigger, wilder, faster fire |

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
