# Hatchglass

Kaleidoscope panes redrawn as tonal crosshatch; the slider sweeps a luma or saturation key from picture to full engraving

**P12 (Engrave):** the key level sweeps the picture into hatch: none at 0%, everything at 100%, whatever Soft and Invert are

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Ink | 0–100% | stroke colour: black ink at 0%; at 100% the picture's own colour (half its brightness above black, 1.5x its saturation) |
| K2 | Slide | -100–100% | the mirror lattice slides sideways, bipolar; squared speed curve with a centre detent (to 15 px a frame) |
| K3 | Weight | 9–69% | stroke weight in the darkest tones, 9–69% of the pitch |
| K4 | Pitch | 0–100% | stroke spacing 4–48 px (32 exponential steps) |
| K5 | Depth | 0–100% | the panes multiply: 2 at 0% (a mirror at the screen centre) to 128 at 100%; exponential, glided |
| K6 | Soft | 0–100% | key edge softness, 16 half-octave steps |
| S7 | Key | Luma / Sat | luma / saturation |
| S8 | Invert | Off / On | Engrave takes the darks (or greys) first instead of the lights |
| S9 | Space | Screen / Glass | Screen / Glass. Glass mirrors the strokes and the key with the panes (chevrons at every seam, a symmetric key); Screen keeps both on the real picture, so the subject's silhouette cuts through |
| S10 | Paper | White / Over | hatch on white paper / hatch over the mirrored picture |
| S11 | Fold | Mirror / Repeat | mirror (alternate panes reversed) / repeat (tiles) |
| P12 | Engrave | 0–100% | the key level sweeps the picture into hatch: none at 0%, everything at 100%, whatever Soft and Invert are |

## Presets

Default, Etched Mirrors, Dark Keyed, Sat Tiles, Leaded Glass, Reeded Glass, Colour Pencil.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
