# Noisetone

Value-noise warp over a sweep of tone curves: the noise can also shift the exposure; Storm drives warp and contrast together.

**P12 (Storm):** displacement (+⅛ of the width, linear) and contrast (+2×) climb together

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scale | 0–100% | noise cell size 4–512 px, log, continuous (64 steps an octave); the field zooms about the screen centre. Display 0–100% |
| K2 | Amount | 0–100% | displacement, square law, up to ±⅛ of the width (±240 px in HD) |
| K3 | Drift | -100–100% | vertical scroll, two-way: right of centre the field rises, left it sinks, up to 16 lines a frame |
| K4 | Curve | 0–7 | continuous sweep through 8 curves, neighbours blended: gamma 0.5, 0.7, 1.5, 2.2, inverse S, soft S, hard S, crush (display 0.0–7.0; default 5.0 = soft S) |
| K5 | Contrast | 25–425% | gain about the pivot before the curve, 25–420% |
| K6 | Pivot | 0–100% | grey level the contrast pivots about; it lands on mid grey, so it also sets the exposure |
| S7 | Patchy | Flat / Patchy | one exposure everywhere / the noise field shifts the pivot, so the exposure varies in patches |
| S8 | Displace | All / Luma | all channels / luma only |
| S9 | Stretch | Tall / Wide | cells 4× tall / 4× wide |
| S10 | Field | Smooth / Blocky | smooth value noise / hard cells |
| S11 | Chroma | Flat / Contrast | saturation flat / follows the contrast gain |
| P12 | Storm | 0–100% | displacement (+⅛ of the width, linear) and contrast (+2×) climb together |

## Presets

Default, Heat Shimmer, Patchy Print, Colour Slip.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 32 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
