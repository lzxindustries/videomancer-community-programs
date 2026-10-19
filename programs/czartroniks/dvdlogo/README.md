# DVD Logo

The bouncing DVD logo screensaver: up to four logos drift and bounce, changing colour on every hit, with a palette strobe when one lands exactly in a corner.

**P12 (Speed):** Logo travel speed, exponential, with smooth sub-pixel motion.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Angle | 0–100% | Bounce angle of the logo's travel |
| K2 | Size | 50–300% | Logo size, 50–300% |
| K3 | Count | 1 / 2 / 3 / 4 | How many logos bounce around (1–4) |
| K4 | Clr Step | 1 / 2 / 3 / 4 / 5 / 6 / 7 / Random | How far the colour steps on each bounce (1–7, or random) |
| K5 | Tint | 1–8 | Tint colour of the logo |
| K6 | Bright | 25–100% | Logo brightness |
| S7 | Color | Mono / Palette | Mono: one tint; Palette: colour changes on every bounce |
| S8 | Matte | Off / On | On: the logo shows the live video through it |
| S9 | BG | Video / Black | Background: live video or black |
| S10 | Pause | Off / On | On: freezes the logos in place |
| S11 | Overlap | Bounce / Mix | Bounce: logos bounce off each other; Mix: they pass through and blend |
| P12 | Speed | 0–100% | Travel speed, exponential with smooth sub-pixel motion |

## Presets

Classic DVD, Fast Chaos, Lone Drifter, Glass Mix, Jumbo.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
