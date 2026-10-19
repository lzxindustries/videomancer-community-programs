# Ziffern

A Kraftwerk-style wall of green pixel digits 1-9 in four depth planes; the slider flies the numbers toward you in perspective at different speeds, digits flicker to new values, with glow, alien font, and luma-driven zoom.

**P12 (Zoom):** Flies the number field from far/dense (100%) to near/big (0%) in perspective, planes arriving at different speeds, with a smooth glide.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Color | 0–360deg | Digit colour |
| K2 | Change Rate | 0–100% | How often digits change; zero freezes them |
| K3 | Number Set | 0–100 | Picks a different random arrangement of digits |
| K4 | Size | 0–100% | Digit size |
| K5 | Spacing | 0–100% | Gap between digits |
| K6 | Glow | 0–100% | Digit brightness plus glowing halo |
| S7 | Luma Mod | Off / On | On: bright areas pop digits forward, dark stay far |
| S8 | Background | Black / Tint | Background: black or dark tint |
| S9 | Video | Off / On | On: shows video in the black areas |
| S10 | Font | Plain / Alien | Font: plain or alien |
| S11 | Invert | Off / On | On: inverts the picture |
| P12 | Zoom | 0–100% | Zoom: digits rush toward or away from you |

## Presets

Kraftwerk Green, Slow Drift, Full Throw, Amber Terminal, Luma Pop, Burn.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
