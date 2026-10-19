# Taffy

Drag the slider down the picture and the lines it touches stretch sideways out from the centre, then slowly snap back behind it - like pulling taffy through the image.

**P12 (Position):** Picks the scanline being pulled; lines it passes stretch and then relax behind it, so moving the slider draws a moving bulge/wake.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Pull | 0–100% | How fast a touched line stretches out |
| K2 | Relax | 0–100% | How fast stretched lines relax back |
| K3 | Height | 1–255ln | Thickness of the stretch band |
| K4 | Centre | 0–511px | Width of the untouched centre strip |
| K5 | Depth | 0–100% | How much lines magnify at full stretch |
| K6 | Soft | 0–100% | Softens the band's top and bottom edges |
| S7 | Polarity | Stretch / Squeeze | Stretch outward or Squeeze inward |
| S8 | Hold | Off / On | On: freezes all current stretches in place |
| S9 | Steps | Smooth / Stepped | Smooth: gradual stretch; Stepped: four fixed levels |
| S10 | Bypass | Off / On | On: passes video through unchanged |
| S11 | Mode | Sustain / One Shot | Sustain: stretch while slider stays; One Shot: stretch once |
| P12 | Position | 0–100% | Which line down the picture gets stretched |

## Presets

Scanner, Long Trail, Single Line, Soft Lens, Terraces, Wake, Pinch.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 4 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
