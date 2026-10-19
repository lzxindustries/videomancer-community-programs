# Warpfield

A radial starfield: 6 zoom shells stream stars out of (or into) a vanishing point that can wander, growing through four glow levels as they approach, in 8 shimmer palettes. Optional Deep Space nebula, video brightness driving star density and size, and stars laid over the live video.

**P12 (Warp):** Flow speed on a quadratic curve: ~30 s per cycle at 10%, ~1 s at 50%, hyperspace streaks at full. Above 50% the stars also get livelier.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Layers | 1 / 2 / 3 / 4 / 5 / 6 | How many zoom shells of stars are active |
| K2 | Density | 0–100% | Star density (in Luma Mode, how strongly brightness adds stars) |
| K3 | Brightness | 0–100% | Star brightness |
| K4 | Palette | Silver / Candle / Twilight / Neon / Ember / Honey / Rainbow / B&W | One of 8 star colour sets |
| K5 | Size | 0–100% | Bigger, glowier stars |
| K6 | Drift | 0–100% | How far the vanishing point wanders (0 = locked centre) |
| S7 | Direction | Toward / Away | Stars fly toward you or recede into the distance |
| S8 | Video | Off / On | Stars laid over the live video |
| S9 | Luma Mode | Off / On | Video brightness sets star density and adds a little size |
| S10 | Background | Off / Deep Space | Black, or a faint nebula and gradient |
| S11 | Gate | Off / On | Luma Mode: steeper brightness-to-density; otherwise only the bold stars |
| P12 | Warp | 0–100% | Flow speed, from slow cruise to hyperspace |

## Presets

Cruise, Hyperspace, Star Drift, Nebula Dive, Retreat, Tunnel Weave.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
