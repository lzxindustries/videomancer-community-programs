# Cubist

A glossy Rubik's cube product shot that turns its own layers at random, with live-video stickers or backdrop, auto-spin on each axis and a turn-speed slider.

**P12 (Turn Speed):** How fast the random layer turns happen, from slow, eased quarter-turns to rapid-fire.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Yaw | 0–360deg | Turns the whole cube left/right (yaw angle) |
| K2 | Pitch | 0–360deg | Tilts the cube forward/back (pitch angle) |
| K3 | Roll | 0–360deg | Rolls the cube clockwise/anticlockwise |
| K5 | Video | Off / White / Yellow / Red / Orange / Green / Blue / All | Video stickers: off, one sticker colour, or all stickers show the live video |
| K6 | Size | 50–90% | Cube size on screen, 50–90% |
| S7 | Run | Solved / On | Solved: cube holds still and solved; On: layers turn at random |
| S8 | Auto Yaw | Off / On | On: cube spins continuously around yaw |
| S9 | Auto Pitch | Off / On | On: cube spins continuously around pitch |
| S10 | Auto Roll | Off / On | On: cube spins continuously around roll |
| S11 | Background | Grey / Video | Backdrop: grey sweep or the live video |
| P12 | Turn Speed | 0–100% | How fast the layers turn, from slow to rapid-fire |

## Presets

Hero, Corner.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
