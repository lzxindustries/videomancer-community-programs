# Globeswap

Sphere or tunnel lens with Y/U/V read from split columns, channel swaps and negatives, and a Sabattier fold gated by region.

**P12 (Globe):** grows the lens (toward sphere) and its radius, spreads the channels and drives the fold, up to 16× at any Fold setting. Glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Lens | -100–100% | lens strength, bipolar (tunnel .. flat .. sphere), glided. About 75% wraps the picture exactly onto the disc |
| K2 | Size | 3–146% | lens radius, 3–146% of the half-height, glided |
| K3 | Swap | None / U<>V / Y>U / U>Y / Y<>V / Rotate / -Y / -YUV | channel mode: None / U<>V / Y>U / U>Y / Y<>V / Rotate / -Y / -YUV, with hysteresis at the zone edges. Default U<>V |
| K4 | Offset | -256–256px | per-channel column offset, ±256 px at HD (±128 in SD), glided |
| K5 | Solar | 0–100% | solarization: 0% = off, 100% = the most solarized (everything above mid-grey folds). Default 64% |
| K6 | Fold | 0–100% | fold gain 0.125× .. 7.5×, log, over the whole knob |
| S7 | Fringe | Even / Rim | Even / Rim (split grows toward the lens rim) |
| S8 | Swap | All / Solar | All / Solar (swap only inside the solar region) |
| S9 | Split | Chroma / Luma | Chroma (U and V pulled apart) / Luma (luma against both chroma) |
| S10 | Lens | Row / Disc | Row (whole width) / Disc (lens inside the disc, dry picture outside) |
| S11 | Fold | Luma / Colour | Luma / Colour (the solar region also inverts its chroma) |
| P12 | Globe | 0–100% | grows the lens (toward sphere) and its radius, spreads the channels and drives the fold, up to 16× at any Fold setting. Glided |

## Presets

Default, Prism Globe, Solar Tunnel, Torn Channels.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
