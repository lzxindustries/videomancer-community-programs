# Oracle

A divinatory eye opening in a wall of static between two spectra: chirped rainbow bars with absorption lines above, a bevelled window of grain, streaks, sparks and rings in the middle, an opening mandorla eye with a white rim and rainbow corona, and swaying horizon bands (or a perspective floor reflecting the eye) below. Tear rips whole rows sideways.

**P12 (Vision):** The eye opens from a glowing hairline slit to an almond to a full circle; the corona, rings, sparks, bar ripple and band sway all swell with it.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Size | 0–100% | Eye height (the eye can outgrow the window at max) |
| K2 | Gaze | 0–100% | The eye wanders on a slow Lissajous; 0 = fixed at centre |
| K3 | Aura | 0–100% | Rainbow corona width and ring density |
| K4 | Static | 0–100% | Fine 1-px grain up to long horizontal streaks |
| K5 | Flow | 0–100% | One speed for every drift; 0 = still (the static still boils) |
| K6 | Spectrum | 0–100% | Density of the top bars and bottom bands |
| S7 | Tear | Off / On | Random bursts of rows torn sideways, plus jagged seams |
| S8 | Eye | Lens / Iris | Bands bowed through a lens, or iris rings with a pupil |
| S9 | Moire | Off / On | Sliding line screens over the bars and bands |
| S10 | Horizon | Flat / Deep | Flat swaying bands, or a perspective floor with haze and the eye reflected |
| S11 | Palette | Spectrum / Opal | Full spectrum, or a cool green-to-violet fold |
| P12 | Vision | 0–100% | The eye opens: hairline slit -> almond -> full circle |

## Presets

Oracle, Slit of Light, Full Vision, Iris, Opal Moire.

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- `processing_delay_clks` is declared 0 (allowed for synthesis programs); the sync latency through the program is 61 clocks.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
