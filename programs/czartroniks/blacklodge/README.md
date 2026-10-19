# Blacklodge

The Twin Peaks Red Room: a crimson velvet curtain with rounded folds billowing on slow travelling waves, over a cream-and-brown chevron floor that recedes in forced perspective and can walk toward or away from you. Flicker adds Lynch-style light bursts; Key puts your video on the floor.

**P12 (Horizon):** Raises and lowers the curtain line over the floor: top = curtain raised, bottom = curtain dropped.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Folds | 0–100% | Curtain fold density |
| K2 | Depth | 0–100% | How dark the shadows between folds are |
| K3 | Width | 0–100% | Chevron stripe width at the horizon (4-36 px at 1080p, scaled with the screen; smooth) |
| K4 | Apex | 0–100% | Horizontal position of the vanishing point |
| K5 | Walk | -100–100% | Floor flow speed; centre still, right = toward the viewer |
| K6 | ViewHt | 0–100% | Perspective growth of the floor (smooth) |
| S7 | Sway | Off / On | Curtain frozen in place, or billowing |
| S8 | Chevrons | Rays / Rows | Stripes radiate from the vanishing point, or zigzag bands across the floor |
| S9 | Brown | Off / On | Dark floor stripes near-black or saturated brown |
| S10 | Flicker | Off / On | Random light-flicker bursts |
| S11 | Key | Off / On | Input video replaces the floor |
| P12 | Horizon | 0–100% | Curtain line height (top = curtain raised) |

## Presets

Red Room, Deep Velvet, Alcove, Stage Lit, Vertical Weave, Signal Floor.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
