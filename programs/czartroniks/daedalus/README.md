# Daedalus

Redraws the picture as a right-angle labyrinth: one continuous greek-key line that snakes, weaves over and under itself, thickens in the shadows, and drapes over the form of the image.

**P12 (Relief):** Warps the maze's coordinate space by luma so the whole labyrinth bows and drapes over the image's form.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Ink | 0–100% | Ink: how much of the maze is dark line |
| K2 | Contrast | 0–100% | Contrast of the image driving line thickness |
| K3 | Pitch | 0–100% | Maze cell size, from large to tiny |
| K4 | Angle | 0–360deg | Rotates the maze to any angle |
| K5 | Tangle | 0–100% | Tangle: from ordered runs to knotted crossings |
| K6 | Scheme | Parchment / Gallery / Night / Blueprint / Cyanotype / Copper | Colour scheme: parchment, gallery, night, blueprint, cyanotype, copper |
| S7 | Blackout | Off / On | Off: normal; On: darkest areas go solid black |
| S8 | Invert | Off / On | Off: normal; On: swaps ink and paper |
| S9 | Tint | Off / On | Off: scheme colour; On: lines take the video's colours |
| S10 | Width | Fixed / Modulated | Fixed: even lines; Modulated: lines thicken in shadows |
| S11 | Drift | Off / On | Off: still; On: maze slowly drifts |
| P12 | Relief | 0–100% | Relief: bends the maze over the image's shapes |

## Presets

Blackwork, Weave, Circuit, Labyrinth, Chasing, Shatter.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
