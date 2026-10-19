# Damier

Op-art checkerboards: a clean checkerboard redrawn square by square from a vocabulary of shapes in 16 layouts and 16 palettes, folded by three mirror switches into bilateral, quad or 8-fold kaleidoscope symmetry, bent by travelling waves, and filled on the slider with concentric rings in each square's own shape.

**P12 (Resonance):** Fills every square with concentric rings in its layout's own shape (square rings on the plain checker, circles on Vega, arcs on Truchet). At 0 the picture is untouched; rings appear at the cell corners and multiply inward, tightening to ~17 px apart at 100%. The rings stand still and change only with the slider; their spacing is set in pixels, so the look holds at any Scale.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Layout | Classic / Vega / Albers / Harlequin / Pinwheel / Chevron / Truchet / Weave / Bullseye / Fractal / Quartered / Mosaic / Rings / Zebra / Robe / Shimmer | Arrangement of the squares: plain checker, Vega dots, nested squares, diamonds, pinwheels, chevrons, Truchet arcs, weave, bullseyes, fractal sub-checkers and more |
| K2 | Scale | 0–100% | Square size, 8 to 256 px (exponential) |
| K3 | Wave | 0–100% | Wave amplitude; a second wave crosses in past 50% |
| K4 | Angle | 0–360deg | Lattice rotation; the waves ride the lattice axes |
| K5 | Speed | 0–100% | Everything that moves: waves, drift, spin, size pulses |
| K6 | Palette | Op / Riley / Vega / Kiss / Bauhaus / Memphis / Neon / Pastel / Seventies / Mondrian / Delft / Ruby / Lagoon / Sunset / Acid / Prism | 16 colour schemes |
| S7 | Mirror | Off / On | Mirror the picture left-right (any fold also spins the lattice) |
| S8 | Flip | Off / On | Mirror top-bottom |
| S9 | Kaleido | Off / On | Fold the diagonal too: 8-fold kaleidoscope |
| S10 | Shape | Sine / Zigzag | Wave shape: smooth sine or zigzag chevrons |
| S11 | Motion | Bend / Squeeze | Waves displace across their travel (Bend) or along it (Squeeze) |
| P12 | Resonance | 0–100% | Concentric rings in each square's own shape, from none to ~17 px apart |

## Presets

Vega Dots, Riley Current, Squares Move, Kaleidoscope, Targets, Kiss Robe, Truchet Flow, Chevron Pop.

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- Input-to-output latency: 40 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
