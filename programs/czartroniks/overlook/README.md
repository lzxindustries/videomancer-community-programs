# Overlook

The Overlook Hotel carpet from The Shining (David Hicks' 'Hexagon'), drawn exactly from vector geometry: orange serpentines and floating red hexagons. You can zoom it, drift it, tilt it into a fogged corridor floor, mirror it into a valley, or turn the hexagons into a video halftone.

**P12 (Zoom):** Exponential, continuous zoom over 6 octaves, centred on the screen and glided so it never jumps.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Drift X | -100–100% | Sideways drift; centre still |
| K2 | Drift Y | -100–100% | Forward/back drift (walk the corridor); centre still |
| K3 | Perspective | 0–100% | Flat carpet to fogged corridor floor |
| K4 | Weight | 0–100% | Serpentine stroke width |
| K5 | Hex Size | 60–140% | Red hexagon size (halftone gain under S9) |
| K6 | Scheme | Classic / Gold Room / Lobby Blue / Deep Night / Mono Amber | One of 5 colour schemes |
| S7 | Mirror | Off / On | Rows mirror about the centre; with Perspective, a carpet valley |
| S8 | Weave | Off / On | Woven pile grain on the carpet |
| S9 | Halftone | Off / On | Input brightness sets each red hexagon's size |
| S10 | Inverse | Off / On | Swaps ground and serpentine colours |
| S11 | Night | Off / On | Dims the carpet; the reds stay bright |
| P12 | Zoom | 0–100% | Smooth 6-octave zoom into the carpet |

## Presets

Carpet, Corridor, Gold Room, Lobby Drift, Room 237, Valley, Halftone, Wall to Wall.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
