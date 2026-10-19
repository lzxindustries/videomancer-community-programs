# Tonecurve

Eight blended tone curves with overdrive, bled and lagging chroma, and a wobbling inset; route each to the main or the inset.

**P12 (Amount):** curve strength 25–200%: 0.25× .. 2× the curve's deviation from the original. 100% (the exact curve) sits at 43% of the travel; above it the curve overdrives

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Curve | 0–7 | continuous sweep through 8 curves, neighbours blended: gamma 0.5, 0.7, 1.5, 2.2, inverse S, soft S, hard S, crush (display 0.0–7.0; default 5.0 = soft S) |
| K2 | Contrast | 25–425% | gain about mid grey before the curve, 0.25–4.2× |
| K3 | Size | 1/2 / 1/4 / 1/8 | inset 1/2, 1/4, 1/8 (default 1/4) |
| K4 | Warp | 0–100% | inset wobble, 0..±255 source px |
| K5 | Bleed | 0–7 | chroma IIR smear 2ⁿ px, n = 0..7 |
| K6 | Lag | 0–100% | chroma behind luma, square law: 0–254 px in HD, 0–127 px in SD |
| S7 | Curve In | Main / Inset | the curve goes to the main picture / the inset |
| S8 | Place | Corner / Centre | inset in the corner / centred |
| S9 | Bleed In | Main / Inset | the chroma bleed goes to the main picture / the inset |
| S10 | Inset | Picture / Negative | picture / negative (luma and chroma, before the curve) |
| S11 | Lag In | Main / Inset | the chroma lag goes to the main picture / the inset |
| P12 | Amount | 25–200% | curve strength 25–200%: 0.25× .. 2× the curve's deviation from the original. 100% (the exact curve) sits at 43% of the travel; above it the curve overdrives |

## Presets

Default, Hard Press, Soft Monitor, Crushed.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
