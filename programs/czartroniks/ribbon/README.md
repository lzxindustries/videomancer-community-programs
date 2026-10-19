# Ribbon

Full-screen glossy candy ribbons: diagonal palette stripes that rotate, flow and bend on a ripple, rounded and lit by a specular gloss. They can be mirrored into chevrons, woven into tartan, twisted into ropes or outlined, and the slider bends them until they fold over.

**P12 (Warp):** Bend depth, squared: from straight stripes up to ~180 px of bend per axis, where the ribbons fold over themselves.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Angle | 0–180deg | Stripe rotation |
| K2 | Flow | -100–100% | Stripes march and ripples travel; centre = still |
| K3 | Scale | 0–100% | Ribbon size, over 7 octaves |
| K4 | Palette | 0–100% | Crossfades through 7 candy colour ramps |
| K5 | Ripple | 0–100% | Bend wavelength, from very long to tight |
| K6 | Gloss | 0–100% | Rounding, specular highlight and seam emboss |
| S7 | Chevron | Off / On | Mirror into arrowheads about the screen centre |
| S8 | Plaid | Off / On | Over/under basket weave with a second set at 90 degrees |
| S9 | Wave | Smooth / Zigzag | Ripple shape: sine or triangle |
| S10 | Rope | Off / On | Mirrored ramp: symmetric twisted ropes |
| S11 | Outline | Off / On | Dark seams between colour stops |
| P12 | Warp | 0–100% | Bend depth, up to a fold-over climax |

## Presets

Rainbow, Bakeshop, Bunting, Tartan, Licorice Rope, Flat Bands, Sugar Storm.

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
