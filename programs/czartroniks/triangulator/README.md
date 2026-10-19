# Triangulator

Triangle pixelation: the video is rebuilt from a lattice of flat-coloured upright and upside-down triangles, each filled with an 8-px average of the picture. The slider sweeps from a fine triangle mosaic to a few giant shards; grid lines, a luma lift/gate, a wireframe mesh and four colour modes shape the look.

**P12 (Triangle Size):** Smooth 8-256 px triangle size (with hysteresis so pot noise can't shake the lattice): from a fine triangle mosaic that still reads as the picture to a few giant abstract shards.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Grid Colour | Black / White / Red / Orange / Yellow / Green / Blue / Violet | Grid and wireframe line colour |
| K2 | Contrast | 0–200% | Fill contrast; 50% = unchanged |
| K3 | Luma Level | 0–100% | Gate off: lifts darker triangles to this level; gate on: hides triangles below it |
| K4 | Colorize | Original / Duotone / Posterize / Hue Spin | Colour mode |
| K5 | Colorize Param | 0–100% | Per mode: saturation (50% = unchanged), duotone pair, palette shift (top = grey), or hue angle |
| K6 | Line Width | Thin / Medium / Bold / Heavy | Grid line thickness, 1-4 px |
| S7 | Grid Lines | Off / On | Triangle edges in the K1 colour |
| S8 | Luma Gate | Off / On | Show only triangles at or above the K3 level |
| S9 | Fill Mode | Faceted / Flat | Faceted: each half-cell its own colour; Flat: one colour per triangle |
| S10 | Wireframe | Off / On | Edges only; with the gate off they take the live video colour |
| S11 | Bypass | Off / On | Clean input passthrough |
| P12 | Triangle Size | 8–256px | Triangle size, from a fine mosaic to a few giant shards |

## Presets

Fine Triangles, Wireframe, Duotone Pop, Posterized.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
