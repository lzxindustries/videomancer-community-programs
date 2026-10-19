# Slit Scan

The picture cut into clean bands of lines (or vertical strips), each slid sideways as if it were a different moment of a moving shot: a simulated camera sways or drifts and every band lags it by its own delay, on top of a ramp, fold, random, wave or zigzag staircase.

**P12 (Offset):** Staircase amount: how far apart the bands slide, from 1/64 of the width to a full screen width (quadratic). The simulated motion keeps rippling on top at any setting.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Bands | 2 / 3 / 4 / 6 / 8 / 12 / 16 / 24 / 32 / 48 / 64 / 96 / 128 | Number of bands (or strips) |
| K2 | Pattern | Ramp / Fold / Random / Wave / Zigzag | Staircase shape across the bands |
| K3 | Motion | 0–100% | Size of the simulated camera sway or drift |
| K4 | Span | 0–100% | How far back in time the last band lags (~0.07 to 4 s) |
| K5 | Speed | 0–100% | Sway/drift rate, from ~34 s to ~1/4 s per cycle |
| K6 | Split | 0–100% | Chroma dispersion: U slides further than Y, V less |
| S7 | Axis | Horizontal / Vertical | Bands of lines, or vertical strips |
| S8 | Edges | Wrap / Mirror | Picture slid off one side wraps around, or mirrors back |
| S9 | Gaps | Off / On | Thin black lines between the blocks |
| S10 | Motion | Sway / Drift | Sine sway, or eased random drift |
| S11 | Flip | Off / On | Reverses the band order (staircase and lag) |
| P12 | Offset | 0–100% | Staircase amount: 1/64 of the width up to a full screen width |

## Presets

Staircase, Venetian, Shredded, Strip Wave, Drift Lag, Chroma Steps.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 4 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
