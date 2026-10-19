# Shearline

Recursive line feedback: each scanline redraws the line above, sheared and fanned, so video streaks into diagonal comet trails that can spray from a vanishing point and age through the hues.

**P12 (Shear):** Trail angle: 0 = straight-down drips, through 45 degrees, to ~15 px/line flat streaks. Half the travel sits below 3 px/line, in 1/16 px/line steps (measured in 1080p lines; 720p and SD draw the same angle).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Fan | -100–100% | Trails spray outward from a vanishing point (CW) or converge into a tunnel (CCW); noon = parallel |
| K2 | Trail | 0–100% | Trail length, exponential: ~4 to 1000+ lines at 1080p (the same share of the screen at 720p and SD) |
| K3 | Slope Mod | 0–100% | Random per-line jitter added to the shear |
| K4 | Spectrum | 0–100% | Trails cycle through the hues as they age; grey trails pick up colour too |
| K5 | Mix | 0–100% | Mix between original and processed picture |
| S7 | Direction | Right / Left | Streaks slant right or left |
| S8 | Blend | Smear / Comet | Smear: the picture dissolves into diagonal blur; Comet: only highlights streak, the picture stays sharp |
| S11 | Bypass | Off / On | On: passes video through unchanged |
| P12 | Shear | 0–100% | Trail angle, from straight-down drips to flat streaks |

## Presets

Drip, Comet, Warp Speed, Rainbow Tail, Tunnel, Screamer, Left Sweep.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
