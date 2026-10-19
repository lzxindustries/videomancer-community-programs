# Impasto

Edge-keeping brush smoothing with stroke relief, warped by drifting value noise; noise can steer the brush instead.

**P12 (Amount):** Warp: displacement up to ±255 px. Steer: share of the picture flipped, none at 0 up to half at 100

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scale | 4–512 px | noise cell size 4–512 px, octave steps |
| K2 | Drift | 0–100% | noise field scrolls vertically |
| K3 | Stroke | 1–8x | noise cells stretched horizontally ×1–×8 |
| K4 | Brush | 0–100% | window 4–64 px each side, exponential (0% 4, 50% 16, 75% 33, 100% 64); the vertical brush drags about half that many lines |
| K5 | Edge | 0–100% | 0 = plain box blur of both windows … 100 = hard nearest-mean pick; also the step that stops the vertical brush |
| K6 | Relief | 0–100% | brush-stroke relief (left mean − right mean), up to 2× |
| S7 | Noise | Warp / Steer | Warp: noise displaces the picture / Steer: noise flips the pick |
| S8 | Field | Smooth / Blocky | smooth value noise / hard cells |
| S9 | Pick | Nearest / Farthest | nearest mean / farthest mean (haloed oil) |
| S10 | Relief | Left / Right | light from the left / right |
| S11 | Knife | Off / On | luma posterized to 8 steps |
| P12 | Amount | 0–100% | Warp: displacement up to ±255 px. Steer: share of the picture flipped, none at 0 up to half at 100 |

## Presets

Default, Wet Canvas, Palette Knife, Halo Oil.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
