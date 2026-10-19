# Foldither

Mirror / repeat folds, Bayer-dithered and quantized in source space (the pattern folds too) or in screen space.

**P12 (Panes):** pane width ÷ 1 / 2 / 4 / 8

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Segment | 1024 px / 512 px / 256 px / 128 px / 64 px | pane width 1024 / 512 / 256 / 128 / 64 px at 1080p; the same fraction of the screen height in every mode |
| K2 | Slide | -100–100% | the panes slide across the picture, bipolar. Squared, with a centre detent. Top ~7.75 px a frame |
| K3 | Skew | -100–100% | per-line lean of the panes, bipolar. Squared, with a centre detent. 45° at ~35% from centre, top ~7.75 px a line |
| K4 | Bits | 1 bit / 2 bits / 3 bits / 4 bits / 5 bits / 6 bits | luma levels 1–6 bits. Black and white are always levels |
| K5 | Dither | 0–100% | dither amount, 0–100% of a step |
| K6 | Matrix | 2x2 / 4x4 / 8x8 / Diagonal | 2x2 / 4x4 / 8x8 Bayer / Diagonal (45° line screen) |
| S7 | Fold | Mirror / Repeat | Mirror / Repeat (tiles are two segments wide) |
| S8 | Pattern | Source / Screen | Source: the dither pattern folds with the picture / Screen: it stays put |
| S9 | Chroma | Clean / Dithered | Clean / Dithered (quantized like luma, centred on grey) |
| S10 | Alt Neg | Off / On | alternate panes (tiles in Repeat) negative |
| S11 | Alt Bits | Off / On | alternate panes 2 bits coarser (2 bits finer at 1–2 bits) |
| P12 | Panes | x1 / x2 / x4 / x8 | pane width ÷ 1 / 2 / 4 / 8 |

## Presets

Default, Tiled Gameboy, Skewed Mirror, Alt Crush.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
