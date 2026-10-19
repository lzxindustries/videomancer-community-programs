# Panemix

Wiping split-screen panes (repeat or mirror) blended with the picture, false-coloured through 8 cycling palettes.

**P12 (Heat):** palette cycle speed, squared: still at 0, ~0.5 s a lap at 100%

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Panes | 1 / 2 / 3 / 4 | 1 / 2 / 3 / 4 vertical panes (quarters; default 2) |
| K2 | Wipe | 0–100% | panes, and what they show, slide across the screen (0–100% of the width, wraps) |
| K3 | Blend | Add / Screen / Multiply / Difference / Overlay | Add / Screen / Multiply / Difference / Overlay of panes (A) over the picture (B), luma; fifths with hysteresis |
| K4 | Mix | 0–100% | 0–100% of the blend mixed onto the picture |
| K5 | Palette | Thermal / Ironbow / Plasma / Rainbow / Ice / Night Vision / White Hot / X-Ray | Thermal, Ironbow, Plasma, Rainbow, Ice, Night Vision, White Hot, X-Ray (eighths, hysteresis) |
| K6 | Range | 0–200% | gain into the palette 0–200%; 100% = black..top of the blend over one ramp; above, the ramp wraps |
| S7 | Style | Repeat / Mirror | Repeat: centre crop in every pane (filmstrip) / Mirror: odd panes mirrored in place; 1 pane: whole picture flipped |
| S8 | Luma | Palette / Keep | palette luma / the blend's own luma under palette colour |
| S9 | Index | Luma / Diff | palette by the mixed luma / by \|A − B\| |
| S10 | Invert | Off / On | palette reversed |
| S11 | Iso | Off / On | true-black isotherm lines at each eighth of the ramp, width follows Range, none on flat/clipped pixels |
| P12 | Heat | 0–100% | palette cycle speed, squared: still at 0, ~0.5 s a lap at 100% |

## Presets

Default, Mirror Heat, Quad Plasma, X-ray Strip, Rainbow Film, Night Flip.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 32 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
