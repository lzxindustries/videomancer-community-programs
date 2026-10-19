# Cleave

Splits the picture into darks and lights: darks become serrated thermal-rainbow bands, lights become TV static, with a black outline tracing the border. The slider flips which world is which, through a solarized midpoint.

**P12 (Flip):** Rotates the dark/light class wheel: mid-travel is a solarized state, full travel swaps darks and lights exactly.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Band Size | 0–100% | Size of the thermal bands in dark areas |
| K2 | Wave | 0–100% | Wave: grows the bands into big chevron swells |
| K3 | Drift | -100–100% | Band drift direction and speed; centre stopped |
| K4 | Static Size | Fine / Medium / Coarse / Chunky | Grain size of the TV static: fine, medium, coarse or chunky |
| K5 | Boil Rate | 0–100% | S11 Boil: how fast the static dissolves and reforms; S11 Still: scrubs the grain by hand |
| K6 | Fringe | 0–100% | Fringe: width of the transition edge between regions |
| S7 | Patchwork | Off / On | Off: normal; On: patches of double-frequency bands |
| S8 | Contrast | Normal / Boost | Normal or boosted contrast |
| S9 | Outline | Thin / Thick | Thin or thick outline between dark and light |
| S10 | Bands | Horizontal / Vertical | Bands run horizontal or vertical |
| S11 | Grain | Still / Boil | Still: static holds (K5 scrubs it); Boil: static keeps dissolving at the K5 rate |
| P12 | Flip | 0–100% | Flips which areas get bands and which get static |

## Presets

Thermal Scan, Solar Split, Still Negative, Vertical Rain.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
