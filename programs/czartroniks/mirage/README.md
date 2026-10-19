# Mirage

Solarization meets luma displacement: contour bands can drive the warp, mask it, or be drawn on top of it.

**P12 (Bands):** Adds displacement proportional to how much the solarize changed each pixel

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Threshold | 0–100% | Solarize threshold |
| K2 | Gain | 0–100% | Solarize fold strength; high values make bands |
| K3 | Lift | -100–100% | Brightness of the solarized zone |
| K4 | Amount | -100–100% | How far brightness pushes the picture sideways (both ways) |
| K5 | Smooth | 0–100% | Smooths the sideways push |
| K6 | Pivot | 0–100% | Brightness level that doesn't move |
| S7 | Map | Original / Solar | Off: push driven by original; On: by solarized picture |
| S8 | Order | Solar 1st / Warp 1st | Off: solarize then warp; On: warp then solarize |
| S9 | Warp | All / Region | Off: warp everywhere; On: only in solarized areas |
| S10 | Fold | Luma / Chroma | Off: solarize brightness only; On: colour inverts too |
| S11 | Edges | Fill / Wrap | Off: edge colour fills off-screen; On: wraps around |
| P12 | Bands | 0–100% | Push grows stronger in bright solarized bands |

## Presets

Default, Heat Haze, Torn Contours, Bands Only.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.
- The sideways push follows the screen height, so the warp looks the same at SD, 720p and 1080p.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
