# Orbifold

An iterated ring fractal: rosette and flower-of-life rings in 8 shapes nested across up to 6 scales, with a seamless endless zoom, an orbit-and-pulse animation that ripples down the scales, and an optional video halftone that fattens the rings where the picture is bright.

**P12 (Animate):** Each scale orbits around its cell while the ring radius pulses, rippling down through the levels. 100% = half-cell swing, rings collapse and swell.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Shape | Circle / Square / Diamond / Octagon / Star / Sparkle / Cross / Mixed | Ring shape at every level |
| K2 | Radius |  | Ring radius within each cell |
| K3 | Thickness |  | Ring line thickness |
| K4 | Depth | 1 / 2 / 3 / 4 / 5 / 6 | How many nested scales are drawn |
| K5 | Scale | 0–100% | Cell size, over 3 octaves (up = bigger) |
| K6 | Zoom | -100–100% | Endless zoom drift; centre still, right zooms in |
| S7 | Centres | Corners / Middles | Rings centred on cell corners (fish scales) or middles (rosettes) |
| S8 | Fill | Rings / Discs | Outline rings or filled discs |
| S9 | Video | Off / Weight | Input brightness fattens the rings (halftone) |
| S10 | Overlap | Stack / XOR | Overlapping levels stack or cancel |
| S11 | Palette | Rainbow / Phosphor | Rainbow or single-phosphor colouring |
| P12 | Animate | 0–100% | Orbit and radius pulse rippling down the scales |

## Presets

Rosette Wall, Infinite Zoom, Polka Bloom, Phosphor Lace, Halftone, Tile Works, Star Garden, Climax.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
