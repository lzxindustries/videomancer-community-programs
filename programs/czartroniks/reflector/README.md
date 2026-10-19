# Reflector

Two-mirror kaleidoscope meets bloom: panes multiply as the axis moves, the source slides, glow follows or ignores the folds.

**P12 (Sweep):** the axis swings on a sine around K1; depth (to ±half the width) and speed (22 s to 4 s a cycle) grow together

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Axis | 0–100% | first mirror position, 0–100% of the width. The mirrors keep bouncing, so below 50% the panes multiply (to 16) |
| K2 | Folds | None / One / Two / Three | None / One / Two / Three: pane width = Axis, /2, /4 |
| K3 | Slide | -100–100% | the source scrolls under the mirrors, bipolar; squared speed curve with a centre detent (to 15 px a frame at 1080p, scaled with the screen height) |
| K4 | Glow | 0–100% | bloom gain |
| K5 | Threshold | 0–100% | luma above which the blur blooms, black to ~white |
| K6 | Radius | 0–100% | blur width 7–255 px at 1080p, exponential (the same size relative to the screen height at 720p and SD) |
| S7 | Glow | Mirrored / Straight | glow fetched mirrored with the picture / unmirrored (the original picture's highlights) |
| S8 | Panes | Plain / Negative | mirrored panes shown as negative |
| S9 | Tint | Off / Warm | warm glow |
| S10 | Seams | Off / Glow | soft light lines along the mirror seams and the slide's wrap; lit even at Glow 0 |
| S11 | Focus | Sharp / Soft | base picture replaced by its blur |
| P12 | Sweep | 0–100% | the axis swings on a sine around K1; depth (to ±half the width) and speed (22 s to 4 s a cycle) grow together |

## Presets

Default, Cathedral, Drifting Glass, Negative Space.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
