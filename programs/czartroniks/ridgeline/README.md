# Ridgeline

Drifting sine-noise displacement with Roberts or gradient edges inked over the warped picture or on paper; edges can push.

**P12 (Rows):** noise frequency down the lines, squared, glided. 0 = pure column waves; 35% ≈ 4 cycles a frame; 100% = 34 cycles a frame, a 32-line ripple that shreds the picture (16-line with two octaves)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Amount | 0–100% | displacement, squared: 0..±511 px at 1080p (±766 with two octaves; the same size relative to the screen height at 720p and SD) |
| K2 | Scale | 0–100% | noise frequency across x, squared, glided: 0..7.5 cycles across a 1080p width (the waves keep their size relative to the screen height in every mode) |
| K3 | Speed | 0–100% | drift speed |
| K4 | Gain | 0–100% | edge gain: how strongly the edges ink |
| K5 | Edges | Roberts / Horizontal / Vertical / Sobel | edge detector: Roberts / Horizontal / Vertical / Sobel, with hysteresis at the zone edges |
| K6 | Ink | 0–100% | ink brightness, black (0%) to white (100%), continuous. Lines blend toward the ink by edge strength |
| S7 | Ground | Picture / Paper | Picture: ink over the displaced picture / Paper: ink on plain paper, white under dark ink and black under light ink (switches at 50%, with hysteresis). On paper the lines carry the picture's colour |
| S8 | Order | Warp-Edge / Edge-Warp | Warp-Edge: edges between the displaced line above and the input line (the ridges) / Edge-Warp: the input's own edges, displaced with the picture |
| S9 | Chroma | Follow / Own | chroma follows luma / on its own noise phase |
| S10 | Push | Off / On | the input gradient adds to the read offset |
| S11 | Octaves | One / Two | one / two octaves of noise |
| P12 | Rows | 0–100% | noise frequency down the lines, squared, glided. 0 = pure column waves; 35% ≈ 4 cycles a frame; 100% = 34 cycles a frame, a 32-line ripple that shreds the picture (16-line with two octaves) |

## Presets

Default, White Ridges, Pushed Lines, Colour Ripple.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
