# Dropout

Hashed datamosh bars held for N frames (smear, chroma dropout) under a rolling head-switch band with skew and snow.

**P12 (Bands):** band height, exponential: 8 / 16 / 32 / 64 / 128 lines at 0 / 25 / 50 / 75 / 100% (default 30% = 18 lines)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Bars | 0–100% | fraction of bands displaced; 100% displaces every band |
| K2 | Distance | 0–100% | bar shift, 0..100% of half the measured line width (±960 px at HD, ±360 at SD); each displaced bar moves 25..100% of it, hashed sign |
| K3 | Hold | 1–64fr | frames a displacement pattern is kept, 1..64 (P-frame repeat) |
| K4 | Band | 8–256ln | head-switch band height, 8..256 lines (8-line floor) |
| K5 | Noise | 0–100% | Snow: speckle density (Noise², full static at 100%). Lines: per-line luma offset up to ±half the luma range |
| K6 | Skew | 0–100% | rows in the band shift progressively right (up to 32 px per row) |
| S7 | Roll | Fixed / Roll | band fixed at the bottom / rolling up, wrapping round (~5.7 s a cycle in every mode) |
| S8 | Dropout | Off / On | displaced bars lose their chroma |
| S9 | Smear | Shift / Smear | plain shift / displaced bars hold 16-px runs |
| S10 | Burst | Off / On | bands touching the head-switch band (±32 lines) are always displaced |
| S11 | Noise | Snow / Lines | Snow: 2-px grey speckles / Lines: per-line noise lines |
| P12 | Bands | 0–100% | band height, exponential: 8 / 16 / 32 / 64 / 128 lines at 0 / 25 / 50 / 75 / 100% (default 30% = 18 lines) |

## Presets

Default, P-Frame Smear, Bad Tracking, Fat Bars.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
