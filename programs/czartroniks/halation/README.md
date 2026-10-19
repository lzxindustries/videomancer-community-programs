# Halation

Bloom meets magnetic wobble: a wide glow (7–255 px) that can wobble on its own, a real screen blend, and a red halation halo on the slider that widens, then splits into twin ghosts.

**P12 (Halo):** Red halation from the un-wobbled glow: rises as a wider glow (full gain by 25%), then splits into twin red ghosts that spread to 1/8 of the width at 100%. Works at any Wobble.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Glow | 0–100% | Brightness of the glow |
| K2 | Threshold | 0–100% | Only areas brighter than this glow (black to ~white) |
| K3 | Radius | 0–100% | Glow blur width, 7 to 255 px at 1080p (the same size relative to the screen height at 720p and SD) |
| K4 | Wobble | 0–100% | How far lines wobble sideways |
| K5 | Rate | -100–100% | Wobble roll speed and direction (centre = still) |
| K6 | Period | 0–100% | Vertical size of the wobble waves |
| S7 | Warp | Both / Glow | Off: wobble moves picture and glow; On: glow only |
| S8 | Tint | Off / Warm | Off: neutral glow; On: warm-tinted glow |
| S9 | Energy | Off / On | Off: normal; On: brighter rows wobble more |
| S10 | Blend | Add / Screen | Off: glow added; On: glow screened (softer, never darkens) |
| S11 | Focus | Sharp / Soft | Off: sharp picture; On: whole picture blurred |
| P12 | Halo | 0–100% | Red halation: widens, then splits into twin ghosts |

## Presets

Default, Warm Bloom, Loose Glow, Dream.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
