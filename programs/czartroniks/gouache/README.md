# Gouache

Dithered posterize with hue rotation, rainbow hue steps per luma band or hue sector, and sat crush/boost before or after.

**P12 (Spin):** hue spin speed, bipolar, squared. Still at the centre (small deadband); the ends ≈ 0.55 s per turn

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Luma | 1–8b | luma levels, 1–8 bits |
| K2 | Chroma | 1–8b | chroma levels, 1–8 bits |
| K3 | Dither | 0–100% | dither before quantizing, 0–100% of a step, centred on the threshold (default 25%) |
| K4 | Hue | 0–360deg | base hue rotation, 0–360° |
| K5 | Sat | 0–400% | saturation 0–400%, 100% = unity (default 150%) |
| K6 | Spread | 0–100% | extra hue rotation per band: rainbow bands (default 40%) |
| S7 | Order | Post-Rot / Rot-Post | Post-Rot: posterize, then rotate. Rot-Post: rotate, then posterize |
| S8 | Sat | Before / After | gain Before the quantizer (crush/boost the bands) / After |
| S9 | Bands | Luma / Hue | Spread keyed by Luma band / by Hue sector (8 × 45°) |
| S10 | Dither | Bayer / Noise | Bayer 4×4 / Noise |
| S11 | Invert | Off / Alt | alternate luma bands go negative |
| P12 | Spin | -100–100% | hue spin speed, bipolar, squared. Still at the centre (small deadband); the ends ≈ 0.55 s per turn |

## Presets

Default, Rainbow Bands, Crushed Spin, Op Art.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
