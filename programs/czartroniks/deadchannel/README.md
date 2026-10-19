# Dead Channel

A dying analog signal chain in one program: random glitch dropout bands that corrupt the picture, tape hiss and chroma drift, beam misconvergence, phosphor glow and a CRT grille, with the slider pulling the picture into a tape skew.

**P12 (Skew):** Bipolar bounded tape pull: a decaying bend at the top of the frame (up to a quarter of the width, wrapping around) plus a second pull re-armed under each dropout band; centre straight.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Dropout | 0–100% | Random glitch-band events: how often, how big, how long; subtle until ~50%, 0 = none |
| K2 | Hiss | 0–100% | Amount of tape noise |
| K3 | Drift | 0–100% | Scrolling colour wobble depth and rate |
| K4 | Converge | -100–100% | Colour misconvergence; centre aligned, ends spread |
| K5 | Glow | 0–100% | Glow trail behind bright objects |
| K6 | Tube | 0–100% | Depth of CRT grille and dark corners |
| S7 | Grille | Scan / Triad | Scan lines (1 in 3 at HD) or RGB triad grille |
| S8 | Cast | Cool / Sick | Colour cast: cool or sickly |
| S9 | Chroma | Rich / Faded | Rich or faded colour |
| S10 | Band | Tear / Flash | Band glitches the video, or glitches a bleached light bar |
| S11 | Invert | Off / On | Off: normal; On: full video negative (luma and colour) |
| P12 | Skew | -100–100% | Tape skew bend at the top; centre straight |

## Presets

Default, Dead Air, Good Tube, Head Switch.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
