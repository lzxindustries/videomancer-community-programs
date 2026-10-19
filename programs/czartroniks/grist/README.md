# Grist

Film grain to mezzotint: layered grain, block mottle and pores are mixed with the video and pushed through one 'develop' comparator, so the slider takes the picture from subtle film grain to a hard stipple engraving.

**P12 (Develop):** Comparator gain: additive film grain at 0, rising to a hard black/white mezzotint where the picture is re-rendered as grain density.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Fine | 0–100% | Amount of fine per-pixel grain |
| K2 | Coarse | 0–100% | Amount of coarse blotchy grain |
| K3 | Block | 8–64px | Size of the coarse grain blotches |
| K4 | Response | 0–100% | Grain everywhere vs mostly in mid-tones |
| K5 | Pits | -100–100% | Pits: centre = none; one way digs dark pits, the other raises light blobs |
| K6 | Exposure | 0–100% | Exposure: darker or lighter print |
| S7 | Animate | Off / On | Off: frozen texture; On: moving film grain |
| S8 | Soft | Off / On | On: softer, blurred grain with smooth blocks and blob-shaped pits |
| S9 | Mono | Off / On | Off: keeps video colour; On: black and white |
| S10 | Etch | Off / On | On: posterizes into flat bands with dithered edges |
| S11 | Invert | Off / On | On: negative image |
| P12 | Develop | 0–100% | Light film grain up to hard black-and-white mezzotint |

## Presets

Film Stock, Mezzotint, Concrete, Corroded, Star Dust, Etched Print.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
