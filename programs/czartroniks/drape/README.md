# Drape

Captures one scanline at the split and drapes it down the screen like fabric: a perspective fan or folding pleats, with lean, travelling sway and a fade to black.

**P12 (Spread):** Fan strength: from a straight hang to the drape rushing into its vanishing point.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Split | 0–100% | Split: which scanline is captured; rows above it stay untouched |
| K2 | Stretch | 0–100% | Stretch: where below the split the fan begins |
| K3 | Fade | 0–100% | Fade: length of the fade to black; 100% = no fade |
| K4 | Twist | 0–100% | Twist: leans the drape left or right; centre = straight |
| K5 | Onset | 0–100% | Onset: 0 = fan starts with a sharp kink, up = eases in gradually |
| K6 | Sway | 0–100% | Sway: travelling wave that grows with depth; bottom of travel is still |
| S7 | Blur | Off / On | On: softens the captured line (blur widens as it fans) |
| S8 | Shape | Fan / Pleat | Fan: columns converge to a vanishing point; Pleat: folds back and forth |
| S9 | Freeze | Off / On | On: holds the captured line so Split can move without recapturing |
| S11 | Bypass | Off / On | On: passes the input through unchanged |
| P12 | Spread | 0–100% | Spread: fan strength; 100% runs the drape into its vanishing point |

## Presets

Default, Long Drape, Short Tail, Top Clip, Horizon, Pleats, Wind, Tilt.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
