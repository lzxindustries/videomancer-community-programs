# Clacker

Video re-rendered as a mechanical flip-disc wall: discs or square cards swing and overshoot as each cell's value changes, in colour or as greyscale-by-tilt.

**P12 (Clack):** Swing character: 0 = instant snap, higher = long lazy overshooting tumbles; top band makes the whole wall tumble forever.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Disc Size | 34–162px | Size of the flip discs |
| K2 | Levels | 2–16 | Number of colour/tilt steps each disc can show |
| K3 | Fill | 50–100% | Disc size within its cell: gapped to overlapping |
| K4 | Refresh | 0–100% | How often rows update, from slow wave to every frame |
| K5 | Tilt | 0–100% | Resting lean of every disc, from flat to steeply angled |
| K6 | Chroma | 75–273% | Colour intensity of the discs |
| S7 | Shape | Disc / Square | Round discs or square cards |
| S8 | Mode | Colour / Luma Mod | Colour: discs show colour; Luma Mod: brightness drives tilt |
| S9 | Backing | Black / Video | Behind the panel: black or dimmed video |
| S10 | Grid | Off / On | Off: none; On: dark lines between cells |
| S11 | Bypass | Off / On | Off: effect; On: original video |
| P12 | Clack | 0–100% | Flip swing: instant snap up to long overshooting tumbles |

## Presets

Departures, Flip-Dot Sign, Slow Wave, Split-Flap.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
