# Hueflutter

Luma-band wobble of a hue-spun, saturation-scaled picture, posterized with a 2x2 dither; hue can ride the wobble phase.

**P12 (Spin):** hue spin speed, bipolar, squared. Still at the centre (small deadband); the ends ≈ 0.55 s a turn

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Wobble | 0–255 px | displacement amplitude, 0–255 px (default ≈ 112 px) |
| K2 | Waves | 0–100% | vertical wavelength: line-phase step. Glided |
| K3 | Hue | 0–360° | base hue rotation, 0–360°. 0 = unrotated |
| K4 | Sat | 0–400% | saturation 0–400%, 100% = unity |
| K5 | Luma | 1–8 b | luma levels, 1–8 bits |
| K6 | Chroma | 1–8 b | chroma levels, 1–8 bits (1 bit = −/grey/+ per channel) |
| S7 | Bands | Phase / Amp | Phase: band k of 8 is k/16 of a cycle ahead / Amp: band k swings (k+1)/8 of K1 (dark bands calm, bright bands swing) |
| S8 | Order | Dither In / Dither Out | Dither In: chroma posterized before the hue rotation (the colour lattice spins), dither pattern travels with the picture / Dither Out: posterized after it (a fixed palette the colours snap between), dither pattern fixed to the screen |
| S9 | Chroma | With / Quarter | With / Quarter: chroma reads a quarter wave apart (fringes) |
| S10 | Hue | Static / Ripple | Static / Ripple: the hue follows the wobble phase (rainbow ripples) |
| S11 | Quant | Plain / Dither | Plain / Dither: 2×2 ordered dither (cells 2 px wide) |
| P12 | Spin | -100–100% | hue spin speed, bipolar, squared. Still at the centre (small deadband); the ends ≈ 0.55 s a turn |

## Presets

Default, Rainbow Jelly, Grey Static, Posterwave.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 32 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
