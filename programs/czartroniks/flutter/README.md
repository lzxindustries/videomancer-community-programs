# Flutter

Sine wobble where each posterized luma band slides with its own phase or amplitude; 2-D ripple on the slider, chroma fringes.

**P12 (Twist):** horizontal phase step: 0 = pure line wobble, up = 2-D ripple (257 px a cycle at 100%). Glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Wobble | 0–255px | displacement amplitude, 0–255 px |
| K2 | Waves | 0–100% | vertical wavelength: line-phase step, 0 (the whole frame sways together) to 64 lines a cycle. Glided |
| K3 | Speed | -100–100% | bipolar, squared, ±2.3% centre deadband. One cycle in ~4 s at ~30% from centre, ~3.5 cycles a second at the ends |
| K4 | Luma | 1–8b | luma levels, 1–8 bits. Also the bands: each posterized level slides as one band |
| K5 | Chroma | 1–8b | chroma levels, 1–8 bits (1 bit = −/grey/+ per channel) |
| K6 | Spread | 0–100% | how far apart the bands are: up to (N−1)/N of a cycle (Phase), or a gain of 0.125–1.12 (Amp) |
| S7 | Bands | Phase / Amp | Phase: band k of N is k/N of a cycle × Spread ahead / Amp: band gain 0.125 + level × Spread (dark bands calm, bright bands swing) |
| S8 | Wave | Smooth / Snap | Smooth: sine / Snap: a flat-topped wave, so lines jump between two offsets |
| S9 | Chroma | Together / Quarter | Together / Quarter: chroma reads a quarter wave apart (fringes) |
| S10 | Edges | Fill / Mirror | Fill: reads off the line take its own edge pixel / Mirror |
| S11 | Dither | Off / On | Off / On: a 2×2 ordered dither, applied before the wobble so it travels with the picture |
| P12 | Twist | 0–100% | horizontal phase step: 0 = pure line wobble, up = 2-D ripple (257 px a cycle at 100%). Glided |

## Presets

Default, Band Slide, Ripple Fringe, Amp Steps.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
