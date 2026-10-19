# Hypnos

Razor-sharp 1960s op-art rings (circle, square, hexagon or flower) with an off-screen centre, Vertigo spiral, warp and colour palettes, every edge anti-aliased.

**P12 (Speed):** Bipolar ring travel: centre stopped, push one way for rings flowing outward, the other for inward.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Center X | -100–100% | Moves the rings' centre left/right, up to two screens off-frame |
| K2 | Center Y | -100–100% | Moves the rings' centre up/down, up to two screens off-frame |
| K3 | Period | 0–100% | Ring spacing, from fine to wide bands |
| K4 | Warp | -100–100% | Warps the ring spacing: bunched toward the centre or toward the edges; centre = even |
| K5 | Spiral | -8–8x | Vertigo spiral: number of spiral arms, either direction; 0 = plain rings |
| K6 | Duty | 5–95% | Duty: how thick the ink bands are versus the gaps |
| S7 | Form | Round / Angular | Round or angular rings (with S8: circle, square, hexagon, flower) |
| S8 | Six-fold | Off / On | On: six-fold shapes (hexagon or flower) |
| S9 | Ink | White / Colour | Ink: white or coloured bands |
| S10 | Paper | Black / Colour | Paper: black or coloured background |
| S11 | Invert | Normal / Invert | Invert: swaps ink and paper |
| P12 | Speed | -100–100% | Bipolar ring travel: centre stopped, push to flow outward or inward |

## Presets

Op-Art Target, Vertigo, Riley Stripes, Rainbow Tunnel, Square Spiral, Hex Hive, Flower Bloom, Warp Speed.

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- `processing_delay_clks` is declared 0 (allowed for synthesis programs); the sync latency through the program is 46 clocks.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
