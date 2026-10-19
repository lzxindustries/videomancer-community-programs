# Pipedream

A woven Truchet network of glossy 3D pipes: every cell holds two quarter-circle bends or an over/under crossing, so every pipe is a continuous loop. Rings, chrome or gloss, light pulses circulating round the loops, and the input's luma swelling or thinning the pipes so the picture is drawn by the weave.

**P12 (Image):** Bipolar luma drive of pipe thickness (S11 On): right of centre the pipes start thin and swell where the picture is bright, merging at 100%; left of centre they start fat and thin where it is bright; centre = plain Fatness. With Image off, a slow diagonal swell wave drives it instead.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scale | 32–158px | Cell size: how big the pipe network is |
| K2 | Weave | 0–100% | Pure looping Truchet to an all-crossings basket weave (over/under) |
| K3 | Fatness | 0–100% | Tube thickness, 12–40% of the half-cell |
| K4 | Flow | 0–100% | Liquid-light pulses circulating along each loop |
| K5 | Hue | 0–360deg | Base pipe colour |
| K6 | Spread | 0–100% | One solid colour to a full rainbow across the screen |
| S7 | Pattern | Random / Rings | Random tiles (no lone rings), or rings that become chain mail with crossings |
| S8 | Finish | Gloss / Chrome | Glossy plastic or banded chrome |
| S9 | Morph | Static / Reshuffle | Fixed network, or tiles slowly reshuffling |
| S10 | Light | Fixed / Spin | Upper-left light, or a spinning highlight |
| S11 | Image | Off / On | Thickness driven by the input's luma (On) or a slow swell wave (Off) |
| P12 | Image | -100–100% | Bipolar: right = bright areas swell the pipes, left = bright areas thin them; centre = plain Fatness |

## Presets

Truchet, Basket Weave, Chain Mail, Portrait, Negative, Neon Circuit.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
