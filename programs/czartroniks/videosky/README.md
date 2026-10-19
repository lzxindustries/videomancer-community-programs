# VideoSky

The picture engraved as a copper line-screen: parallel rules that thicken in the shadows and, with the slider, bow around the image like a banknote or 'Unknown Pleasures' relief.

**P12 (Relief):** Adds the picture's brightness into the line phase so the rules bend around the image, like raised engraving.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Ink | 0–100% | Ink amount: lighter or darker engraving |
| K2 | Contrast | 0–100% | Engraving contrast |
| K3 | Pitch | 0–100% | Line spacing, wide to very fine |
| K4 | Angle | 0–360deg | Line angle, full 360 degrees |
| K5 | Waver | 0–100% | Lines wander like a shaky hand |
| K6 | Scheme | Parchment / Gallery / Night / Blueprint / Cyanotype / Copper | Colour scheme: parchment, gallery, night, blueprint, cyanotype, copper |
| S7 | Cross | Off / On | On: adds crosshatch lines in dark areas |
| S8 | Invert | Off / On | On: inverts light and dark |
| S9 | Tint | Off / On | On: paper and lines take the picture's own colours |
| S10 | Width | Fixed / Modulated | Fixed: constant line weight; Modulated: weight follows brightness |
| S11 | Drift | Off / On | On: the rules crawl, one period every 32 frames |
| P12 | Relief | 0–100% | Relief: brightness bends the lines like topography |

## Presets

Penrose Sky, Bank Note, Copper Plate, Cyanotype, Woodcut, Topography.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
