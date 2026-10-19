# Hallway

One to four panes of mirror folds around a sliding strip, each seen through a barrel lens with chromatic fringing.

**P12 (Depth):** segments narrow continuously, up to 16x (8 px floor); glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Panes | 1–4 | 1 / 2 / 3 / 4 panes |
| K2 | Wipe | 0–100% | pane boundaries slide across the screen, wrapping; glided |
| K3 | Barrel | -100–100% | lens strength, bipolar: pincushion (top/bottom rows magnified to 8x) .. barrel (minified to 1/8); glided |
| K4 | Fringe | 0–100% | chromatic spread, 3 .. 50% of the lens scale (U wider, V narrower) |
| K5 | Segment | 0–100% | mirror segment width, 64 .. 1024 px at 1080p, continuous and exponential (knob up = wider); the same fraction of the screen height in every mode |
| K6 | Slide | -100–100% | strip slide speed, bipolar, squared, with a stop at the centre, up to ~7 px a field |
| S7 | Mirror | Off / On | odd panes mirrored |
| S8 | Lens | Screen / Pane | Screen: every pane looks at the screen centre (copies) / Pane: each pane looks at its own centre |
| S9 | Fold | Mirror / Repeat | Mirror (triangle) / Repeat (sawtooth), same segment width |
| S10 | Edges | Black / Mirror | sources off the line: black / mirrored back in |
| S11 | Fringe | U wide / V wide | U wide: U spreads wider, V narrower / V wide: the other way round |
| P12 | Depth | 0–100% | segments narrow continuously, up to 16x (8 px floor); glided |

## Presets

Default, Four Lenses, Deep Prism, Tiled Pinch.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
