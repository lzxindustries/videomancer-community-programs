# Vhsmix

The posterized picture blended onto the original (add/screen/multiply/difference/overlay) under a noisy head-switch band.

**P12 (Static):** noise over the whole picture, squared: fine grain low on the slider, snow (±half the luma range) at 100%

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Luma | 1 bit / 2 bits / 3 bits / 4 bits / 5 bits / 6 bits / 7 bits / 8 bits | posterize luma, 1..8 bits. Black and white are always levels |
| K2 | Chroma | 1 bit / 2 bits / 3 bits / 4 bits / 5 bits / 6 bits / 7 bits / 8 bits | posterize chroma, 1..8 bits. Centred: greys stay grey |
| K3 | Band | 1–50% | head-switch band height, 1..50% of the measured picture height (default 4.8% = 52 HD lines) |
| K4 | Noise | 5–100% | noise in the band, 5..100% (100% = ±half the luma range) |
| K5 | Blend | Add / Screen / Multiply / Difference / Overlay | Add / Screen / Multiply / Difference / Overlay (fifths of the travel) |
| K6 | Mix | 0–100% | blend amount: 0 = the original, 100% = the blend |
| S7 | Ghost | Off / Ghost | off / the posterized layer is read 1/16 of the width behind (120 px HD, 45 px SD), a tape ghost trailing right |
| S8 | Band | Normal / Negative | normal / the band is the negative of the posterized layer |
| S9 | Roll | Fixed / Roll | band fixed at the bottom / rolling up and wrapping round (~5.7 s a cycle in every mode) |
| S10 | Noise | Luma / Colour | luma only / chroma too (independent U and V noise, held 4 px: colour streaks) |
| S11 | Chroma | Posterize / Blend | Posterize: the posterized chroma mixed on (K2 at 8 bits = the original chroma) / Blend: chroma goes through the blend mode too |
| P12 | Static | 0–100% | noise over the whole picture, squared: fine grain low on the slider, snow (±half the luma range) at 100% |

## Presets

Default, Difference, Static Screen, Overlay Roll, Ghost Relief, Hue Twist, Colour Ghost.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
