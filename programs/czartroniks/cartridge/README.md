# Cartridge

Cel shading over 8 retro palettes, Gameboy to Pico-8: ink outlines, luma bands, Bayer dither, inking slider.

**P12 (Inking):** takes the threshold down to ¼ and adds up to 3 weight steps

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Edge | 0–100% | ink threshold (lower = more ink); the whole travel is live |
| K2 | Levels | Off / 10 bands / 8 bands / 6 bands / 5 bands / 4 bands / 3 bands / 2 bands | luma bands: Off / 10 / 8 / 6 / 5 / 4 / 3 / 2, spread over black..white |
| K3 | Width | 1 px / 2 px / 4 px / 8 px | ink weight 1 / 2 / 4 / 8 px → lines 2 / 4 / 8 / 16 px, in both directions |
| K4 | Palette | Gameboy / Virtual Boy / CGA / ZX Spectrum / Apple II / C64 / EGA / Pico-8 | Gameboy, Virtual Boy, CGA, ZX Spectrum, Apple II, C64, EGA, Pico-8 |
| K5 | Dither | 0–100% | 4×4 Bayer dither before the quantizer (2-px cells in HD) |
| K6 | Range | 0–400% | luma gain into the bands and palette, 0–400% (100% = unity) |
| S7 | Ink | Black / Tinted | Black / Tinted (darkest palette luma carrying the fill's hue) |
| S8 | Order | Bands First / Palette First | Bands First (bands, then palette) / Palette First (palette, then its luma banded) |
| S9 | Edges | Ink / Original | Ink / Original colour |
| S10 | Luma | Palette / Cel | from the palette / from the cel bands (palette hue only) |
| S11 | Invert | Off / On | luma inverted (black ↔ white) before the bands and palette |
| P12 | Inking | 0–100% | takes the threshold down to ¼ and adds up to 3 weight steps |

## Presets

Default, Handheld, Comic Strip, Colour Ink, Red Goggles, Fantasy.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 36 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
