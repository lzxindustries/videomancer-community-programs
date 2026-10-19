# Feedback

A digital camera-pointed-at-the-monitor loop: last frame's output is read back through a zoom/rotate/drift transform with smooth bilinear echoes, live video is keyed, mixed, added or lightened over it, and every generation can shift hue, sharpen, invert or fold. Tunnels, spirals, mandalas and rainbow trails.

**P12 (Feedback):** Loop gain. Low = one or two quick echoes; around 75% the trails become near-permanent (fine control near unity); above that the loop overdrives and bright areas fold back into rings instead of burning to white.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Zoom | -100–100% | Echoes shrink into a tunnel (left) or expand outward (right); centre = still |
| K2 | Rotate | -100–100% | Spin per generation, up to ~84 deg; ~60-90 deg gives mandala symmetry |
| K3 | Hue | -100–100% | Hue rotation per generation, plus colour bloomed from brightness (mono goes rainbow) |
| K4 | Drift | -100–100% | Echoes rise (right) or fall (left) |
| K5 | Live | 0–100% | How much live video enters: key level, or mix/add amount |
| K6 | Blend | Key / Mix / Add / Lighten | How live video is laid over the echoes |
| S7 | Edges | Black / Wrap | Outside the frame is black, or the frame tiles |
| S8 | Mirror | Off / On | Folds the echoes about the centre line (kaleidoscope) |
| S9 | Pixels | Smooth / Blocks | Bilinear-smooth echoes, or deliberately blocky (Fairlight look) |
| S10 | Negative | Off / On | Each generation inverts: echoes alternate light and dark |
| S11 | Crisp | Off / On | Contrast x1.5 per generation: echoes sharpen into hard-edged shapes |
| P12 | Feedback | 0–100% | Loop gain: short echoes to near-permanent at ~75%, then overdrive that folds into rings |

## Presets

Tunnel, Mandala, Smoke Trails, Negative Rings, Overdrive, Fairlight.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
