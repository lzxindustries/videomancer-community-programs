# Fray

Every edge in the picture launches a filament that streams off to the right along its scanline - dashes, morse, comets, beads, sparks - so the image frays into individual pixel threads instead of an even smear.

**P12 (Length):** Maximum filament length, from short stubs to full-screen-width streaks.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Pattern | Dash / Morse / Comet / Beads / Sparks / Bits / Stutter / Fray | Filament pattern: dash, morse, comet, beads, sparks, bits, stutter, fray |
| K2 | Thresh | 0–100% | Edge sensitivity: how strong an edge must be to launch filaments |
| K3 | Colour | Hold / Glow / Negative / White / Rainbow / Polarity / Heat / Tint | Filament colour: edge colour, glow, negative, white, rainbow, polarity, heat, tint |
| K4 | Period | 0–100% | Pattern repeat length along each filament, tiny to wide |
| K5 | Bundle | 0–100% | Groups neighbouring lines so filaments bundle into sheaves |
| K6 | Density | 0–100% | Fraction of edges that actually fire filaments |
| S7 | Ground | Black / Video | Off: filaments on black; On: filaments over the video |
| S8 | Motion | Frozen / Boil | Off: frozen filaments; On: filaments re-roll and boil |
| S9 | Trigger | Both / Falling | Off: both edge directions fire; On: only bright-to-dark edges |
| S10 | Retrig | Restart / Hold | Off: filaments restart at each edge; On: run through edges, longer |
| S11 | Negative | Off / On | Off: normal; On: inverts the whole picture |
| P12 | Length | 0–100% | Maximum filament length, short dashes to full-width streaks |

## Presets

Pixel Stretch, Data Burst, Ember Trails, Etching.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
