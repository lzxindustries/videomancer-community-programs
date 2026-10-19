# Vitrail

A strip of the picture mirrored into a kaleidoscope, posterized into flat glass colours, with a bloom that can wash to white.

**P12 (Glow):** bloom gain 0.25x .. 4x, square law; the top end floods the bright areas to white

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Strip | 0–100% | width of the mirrored source strip, 1/32 .. 1/2 of the picture, square-law knob; the top is a two-pane bookmatch |
| K2 | Axis | 0–100% | where the strip is taken from; the strip always stays inside the picture (12 px in from each edge) |
| K3 | Thresh | 0–100% | bloom threshold, black .. near white |
| K4 | Radius | 0–100% | bloom box width 4 .. 128 px, exponential knob |
| K5 | Levels | 2–16 | luma levels 2 .. 16; black and white are always two of them |
| K6 | Chroma | 0–100% | 0 = grey; low = a few bold glass colours (3 levels per axis, saturation boosted); high = near-original colour |
| S7 | Wash | Colour / White | Colour: the glow is luma only; White: it also bleaches chroma toward neutral |
| S8 | Mirror | Kaleido / Tile | Kaleido (alternate panes mirrored) / Tile (repeated) |
| S9 | Order | Pal Last / Pal First | Pal Last: palette after the bloom (the glow posterizes into bands); Pal First: palette before (smooth glow over flat poster) |
| S10 | Motion | Fixed / Sweep | Fixed / Sweep: the strip ping-pongs across the picture, width/512 px per frame, content flowing through fixed mirrors |
| S11 | Bloom | Lights / Shadows | Lights glow / Shadows bloom dark |
| P12 | Glow | 0–100% | bloom gain 0.25x .. 4x, square law; the top end floods the bright areas to white |

## Presets

Default, Rose Window, Whiteout, Inkwell, Bookmatch.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
