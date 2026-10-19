# Bishop

A faceted wireframe human head drawn in single-colour lines on black or over your video. The mouth opens, the eyes blink, five expressions blend into each other, a noise field can wobble every vertex, and the slider grows the head to fill the screen.

**P12 (Head Size):** Grows the whole head from its natural size to about 95% of the screen height.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Mouth | 0–100% | Mouth closed to open |
| K2 | Line Weight | 1 px / 3 px / 5 px / 7 px | Wireframe line thickness |
| K3 | Palette | Green / Amber / Cyan / Red / White / Magenta / Yellow / Blue | Line colour |
| K4 | Expression | 0–100% | Blends Neutral, Happy, Sad, Angry and Surprised |
| K5 | Noise | 0–100% | Vertex wobble; it also gets faster as it grows |
| K6 | Position | -100–100% | Pans the head left and right |
| S7 | Fill | Outline / Solid | Lines only, or eyes and mouth filled |
| S8 | Eyes | Closed / Open | Eyes closed or open |
| S9 | Noise | Smooth / Streaks | Smooth wobble or streaky noise |
| S10 | Invert | Off / On | Black lines on a coloured field (black ink over video) |
| S11 | Background | Black / Video | Head over black or over the live picture |
| P12 | Head Size | 0–100% | Natural size up to ~95% of the screen height |

## Presets

Neutral, Happy, Angry, Surprised, Solid Fill, Static, Big Head, Video Ink.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
