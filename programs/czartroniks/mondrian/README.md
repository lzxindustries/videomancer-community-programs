# Mondrian

Generates a De Stijl composition: a white canvas carved into rectangles by black grid lines, with red/yellow/blue (and optional black) blocks, merged rectangles, and a Luma Mod that colours blocks only where the video is bright.

**P12 (Density):** How finely the grid cells subdivide, from a plain grid of cells up to about 8 pieces per cell.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Grid Scale | 32–256px | Size of the base grid cells |
| K2 | Composition |  | Picks a different composition (layout, sub-pieces and colours) |
| K3 | Luma Level | 0–100% | Luma Mod line: blocks take colour only where the video is brighter than this |
| K4 | Split Jitter | Centered / Slight / Medium / Wild | How off-centre the splits fall |
| K5 | Colour Amount | 0–100% | How many blocks get red/yellow/blue |
| K6 | Line Width | 2–32px | Thickness of the black grid lines |
| S7 | Grid Lines | Off / On | On: black grid lines drawn |
| S8 | Black Blocks | Off / On | On: some coloured blocks become black |
| S9 | Grid | Square / Not Square | Not Square: cells merge into bigger rectangles |
| S10 | Video Fill | Off / On | On: coloured blocks show the live video |
| S11 | Luma Mod | Off / On | On: white canvas everywhere; blocks take colour only where the video is brighter than K3 |
| P12 | Density | 0–100% | Density: from a plain grid to many small pieces |

## Presets

Classic, Bold Primaries, Big Rectangles, Luma Mask, Video Windows.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
