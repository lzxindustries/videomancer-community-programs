# Afterimage

Video frozen into a coarse 1-bit dithered frame store that feeds back on itself: camera trails, zoom/drift/spin tunnels, diffusion and ordered fades in 8 retro ink/paper palettes.

**P12 (Persist):** How long trails survive, 25-100%: low = the image refreshes quickly; high = trails and tunnels build and hang; 100% = ink never clears (light painting; with Freeze, a frame hold).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Feedback | Hold / Zoom In / Zoom Out / Left / Right / Up / Down / Spin | Feedback motion: hold, zoom in/out, drift L/R/U/D, or spin |
| K2 | Grid | 256x128 / 128x64 / 64x32 / 32x16 | Block grid size, from fine 256x128 to chunky 32x16 (4:3 SD uses 3/4 of the columns, 480i's finest is 180x120, so blocks keep their shape) |
| K3 | Exposure | -100–100% | Exposure: brightens or darkens the image before 1-bit conversion |
| K4 | Dither | Bayer 8x8 / Bayer 4x4 / Threshold / Noise | Dither style: Bayer 8x8, Bayer 4x4, hard threshold, or boiling noise |
| K5 | Palette | Mono / Paper / Game Boy / Phosphor / Amber / Blueprint / Ember / Neon | Picks one of 8 ink/paper colour pairs |
| K6 | Diffuse | 0–100% | Diffuse: how much trails randomly scatter and spread |
| S7 | Invert | Off / On | Off: normal; On: swaps ink and paper |
| S8 | Fade | Grain / Bayer | Trails dissolve as random grain or in an ordered Bayer pattern |
| S9 | Mirror | Off / On | Off: normal; On: mirrors the feedback |
| S10 | Ink | Solid / Video | Ink is a solid colour or shows the live video |
| S11 | Input | Live / Freeze | Live: takes in new video; Freeze: stops new input, trails continue |
| P12 | Persist | 25–100% | Persistence: how long trails survive before dissolving |

## Presets

Camera Trails, Tunnel, Game Boy Smear, Ordered Fade.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
