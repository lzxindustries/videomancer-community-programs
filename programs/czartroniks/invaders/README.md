# Invaders

A playable Videomancer-themed Space Invaders: a purple, white-eyed wizard slides along the bottom on the slider, trailing rainbow only while he moves, and flips S7 to fire at a marching wave of CRT-TV aliens whose screens play scrolling waveforms. Shield chunks that get shot, bombed or crunched by the descending aliens break off and fall away. A bomb hit plays a ~2.4 s death inside a giant TV (no lives); clearing the wave sends him zooming off-screen.

**P12 (Ship X):** Slides the wizard ship left and right across the bottom of the screen; his rainbow trail stretches with speed.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Bomb Rate | 0–100% | Frames between alien bomb drops, slow to a constant rain (4 bombs) |
| K2 | Trail | 0–100% | Rainbow trail length at full speed (0 = off); it only shows while the wizard moves |
| K3 | Hue | 0–100% | Sprite colour wheel (0 = white); rotates the cellophane palette |
| K4 | Alien Spd | 0–100% | Formation march speed (it also speeds up with every kill) |
| K5 | Bomb Drop Spd | 0–100% | How fast alien bombs fall |
| K6 | Shot Spd | 0–100% | How fast the wand-shots fly |
| S7 | Fire | A / B | Any flip fires a wand-shot (up to 4 in flight) |
| S8 | BG | Video / Black | Game over the live picture or over black |
| S9 | Overlay | Solid / Cellophane | One sprite colour, or the arcade overlay: red top strip, green shield band |
| S10 | Pause | Run / Pause | Freezes the game, including the death and win animations |
| S11 | Reset | Run / Reset | Restarts the wave with fresh shields |
| P12 | Ship X | 0–100% | Slides the wizard along the bottom |

## Presets

Arcade, Onslaught, Video Raid.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
