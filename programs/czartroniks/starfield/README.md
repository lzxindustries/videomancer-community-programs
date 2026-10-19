# Starfield

A deep-space parallax starfield: eight layers of soft glowing stars in four shapes that twinkle, fade in and out, and shimmer through eight palettes. Near stars are big and sparse, far ones pinpoint dust, and the slider pushes from a slow drift to a hyperspace rush. Optional drifting nebula, Luma Mode (video brightness sets star density and size) and a Video overlay that puts the stars over your picture.

**P12 (Speed):** Scroll speed on a quadratic curve: a slow drift at 0 (it never stops, so Direction and Depth stay live), about 10 px/frame at half, a ~40 px/frame hyperspace rush at full. The top half also switches to the Lively mood: deeper, faster twinkle, faster churn and shimmer.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Layers | 1 / 2 / 3 / 4 / 5 / 6 / 7 / 8 | Number of star layers, 1 to 8 (the nearest two carry the big stars) |
| K2 | Density | 0–100% | How many stars (the luma gain in Luma Mode) |
| K3 | Brightness | 0–100% | Star and nebula brightness |
| K4 | Palette | Silver / Candle / Twilight / Neon / Ember / Honey / Rainbow / B&W | Star colour set; each shimmers slowly around its hues |
| K5 | Glow | 0–100% | Star glow size, blending smoothly between sizes |
| K6 | Depth | 1 / 2 / 3 / 4 / 5 / 6 / 7 / Max | Parallax spread: how much slower the far layers move |
| S7 | Direction | Left / Right | Stars scroll left or right |
| S8 | Video | Off / On | Stars drawn over the input picture (opaque cores, see-through glow) |
| S9 | Luma Mode | Off / On | Video brightness sets star density and size: dark = few pinpoints, bright = many big glows |
| S10 | Background | Off / Deep Space | Off: black; Deep Space: drifting nebula and gradient |
| S11 | Gate | Off / On | Luma Mode: sharper brightness-to-density map; otherwise thins to the bold stars |
| P12 | Speed | 0–100% | Scroll speed, slow drift to hyperspace; top half = livelier twinkle |

## Presets

Night Sky, Deep Field, Hyperspace, Nebula, Crystal Night, Deep Drift.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
