# Negspin

Spinning hue rotation with per-band spread and sat gain; a luma or sat bit-plane can flip the hue; luma and chroma negatives.

**P12 (Spin):** hue spin speed, bipolar, squared. Still at the centre (small deadband); the ends ≈ 0.55 s a turn

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Hue | 0–360deg | base hue rotation, 0–360°. 0 = unrotated |
| K2 | Sat | 0–400% | saturation 0–400%, 100% = unity |
| K3 | Spread | 0–100% | extra hue rotation per luma band (8 bands), up to ~315° on the brightest band |
| K4 | Plane | Negative / Bit 8 / Bit 7 / Bit 6 / Bit 5 / Bit 4 / Bit 3 / Bit 2 | Negative / Bit 8 … Bit 2. The plane picture: the negative, or one bit-plane as black/white |
| K5 | Mix | 0–100% | plane picture mixed over the luma, 0–100% (100% = the plane alone) |
| K6 | Chroma | Off / 3 Levels / 5 Levels / 9 Levels / 17 Levels / 33 Levels / 65 Levels / 129 Levels | chroma bit-planes: Off / 3 / 5 / 9 / 17 / 33 / 65 / 129 levels per channel (default 9) |
| S7 | Luma | Normal / Negative | Normal / Negative (luma inverted) |
| S8 | Chroma | Normal / Negative | Normal / Negative (hue + 180°). S7 + S8 together = a full colour negative |
| S9 | Flip | Off / On | the plane bit flips the hue 180°. At Plane = Negative it uses the source's top bit (the bright half, or strongly saturated areas) |
| S10 | Source | Luma / Sat | what the plane bits and the Negative picture come from: Luma / Sat |
| S11 | Order | Before / After | chroma bit-planes Before the rotation (the colour lattice spins with the hue) / After (a fixed palette the spinning colours snap between) |
| P12 | Spin | -100–100% | hue spin speed, bipolar, squared. Still at the centre (small deadband); the ends ≈ 0.55 s a turn |

## Presets

Default, Plane Flip, Solar Negative, Crushed Chroma.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
