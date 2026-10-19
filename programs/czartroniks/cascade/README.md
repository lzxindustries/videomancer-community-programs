# Cascade

Concentric circles that push each other: move the centre and the rings cascade outward one by one, never overlapping, then catch up or stay dragged. Gradient shading from hard to smooth, a band-width balance, a moiré mode, and a Video mode where the rings refract the incoming picture.

**P12 (Zoom/Wave):** In Rings mode, zooms the ring spacing logarithmically up to 8x; in Video mode, sets how deeply the rings refract the picture (up to ~53 px).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Center X | -100–100% | Horizontal position of the ring centre (can go a little past the edge) |
| K2 | Center Y | -100–100% | Vertical position of the ring centre |
| K3 | Period | 0–100% | Ring spacing, 4 to 2048 px |
| K4 | Gradient | 0–100% | Ring shading: hard, 4 steps, 8 steps or smooth |
| K5 | Width | -100–100% | Band balance: thin white lines, classic, thin black lines |
| S7 | Glide | Off / On | Smooth the centre's movement |
| S8 | Cascade | Uniform / Cascade | Rings move together, or push each other outward one by one |
| S9 | Moire | Off / On | XOR with a half-phase copy: doubled rings and moiré arcs |
| S10 | Catch-Up | Stay / Home | Dragged rings stay offset, or settle back to centre |
| S11 | Output | Rings / Video | Draw the rings, or use them to refract the video |
| P12 | Zoom/Wave | 0–100% | Rings: spacing zoom up to 8x. Video: refraction depth |

## Presets

Cascade, Stay Dragged, Fine Rings, Ripple Glass, Velvet Rings.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
