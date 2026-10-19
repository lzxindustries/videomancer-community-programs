# Revenant

'Ghost in the Signal': redraws a live portrait as swarms of electric contour strokes that hug the brightness, with blown-out icy eyes ringed by icy strokes, chroma fringe and a one-way drifting scroll. Invert turns the figure into a black silhouette cut from a field of lines, and the Signal Decay fader frays it into a possessed ghost.

**P12 (Signal Decay):** Master decay, fully continuous and glided: warp bend (with S7), ripple, row jitter, glitch walk and rate, chroma fringe (3-15 px at HD), grain and stroke bloom together, from a clean stable portrait to a fully possessed, fraying ghost.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Luma Key | 0–100% | Brightness where strokes begin (high = sparse) |
| K2 | Line Density | 0–100% | Stroke spacing, 48 px to 4.9 px, continuous |
| K3 | Contrast | 0–100% | Black-point crush before keying |
| K4 | Hue Rotate | Violet / Cyan / Amber / Green | Colour scheme (violet, cyan, amber, green) |
| K5 | Drift Speed | 0–100% | One-way scroll speed of the strokes (0 = frozen) |
| K6 | Eye Glow | 0–100% | 0 = off; up = brightest highlights blow out to a solid icy fill with an icy-stroke halo |
| S7 | Warp | Off / On | Off: gentle luma bend; On: deep bend that rides P12 |
| S8 | Chroma Sep | Off / On | On: U/V colour fringe, 3-15 px at HD (rides P12) |
| S9 | Motion | Smooth / Glitch | Smooth drift, or adds glitch frame tears |
| S10 | Invert | Off / On | On: strokes fill the dark; the figure becomes a black silhouette |
| S11 | Bypass | Off / On | On: passes the input through unchanged |
| P12 | Signal Decay | 0–100% | Master decay: bend, ripple, jitter, fringe, grain, bloom, glitch rate |

## Presets

Clean Portrait, Possessed, Cyan Wireframe.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
