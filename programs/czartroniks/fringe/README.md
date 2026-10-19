# Fringe

A moire instrument: two black-and-white line textures (circles, spokes, grids, hex, brick...) laid over each other and combined by boolean logic, with scale and phase played by hand so interference fringes bloom and sweep.

**P12 (Sweep):** Slides layer B against layer A (counter-rotates it for Spokes), raking the moiré fringes across the screen; glided.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Texture A | Circles / Squares / Spokes / Wavy H / Wavy V / Diagonal / Anti-Diag / Grid / Hex / Checker | Pattern shape of layer A (circles, squares, spokes, grids...) |
| K2 | Scale A | 0–100% | Line spacing of layer A, wide to very fine |
| K3 | Phase A | -100–100% | Slides layer A sideways (rotates it for spokes) |
| K4 | Texture B | Circles / Squares / Spokes / Wavy H / Wavy V / Diagonal / Anti-Diag / Grid / Hex / Checker | Pattern shape of layer B |
| K5 | Scale B | 0–100% | Line spacing of layer B |
| K6 | Phase B | -100–100% | Slides layer B sideways (rotates it for spokes) |
| S7 | Meet | Over / Meet | With S8, picks how layers combine: over, meet, fringe, cut |
| S8 | Fringe | Solid / Fringe | With S7: Fringe on gives pure interference (xor) or cut |
| S9 | Weight | Bold / Fine | Bold: thick lines; Fine: thin lines, sparse jewels |
| S10 | Ground | Black / White | Switches ground between black and white (inverts picture) |
| S11 | Colour | Mono / Colour | Mono: black and white; Colour: each layer its own ink colour |
| P12 | Sweep | -100–100% | Slides one pattern against the other, raking the moiré fringes across the screen (glided) |

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
