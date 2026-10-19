# Posterduo

Dithered posterize and a two-ink duotone: banded or smooth inks, tritone midtones, band swap, inks snapped to a 9-colour set.

**P12 (Balance):** the ink split sweeps through the picture: 0% high-key (mostly light ink), 50% split at mid-grey, 100% low-key (mostly shadow ink). Glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Luma | 1–8b | posterize depth, 1–8 bits. Black and white are always levels |
| K2 | Dither | 0–100% | 2×2 ordered dither, 0–100% (±⅜ of a step at full). Default 30% |
| K3 | Pitch | 2 px / 4 px / 8 px / 16 px | dither cell 2 / 4 / 8 / 16 px |
| K4 | Shadow | 0–7 | shadow ink, 8 hues 45° apart |
| K5 | Light | 0–7 | highlight ink, 8 hues |
| K6 | Contrast | 0–400% | tone-ramp contrast, 0–400% (100% = the ramp spans black..white) |
| S7 | Order | Post-Tone / Tone-Post | Post-Tone: inks from the posterized luma (one ink mix per band) / Tone-Post: inks from the smooth luma (a smooth ink ramp under the luma bands) |
| S8 | Tritone | Off / On | midtones fade to neutral grey |
| S9 | Luma | Keep / Flat | Keep (posterized luma) / Flat (luma = the tone ramp; posterized and dithered in Tone-Post) |
| S10 | Bands | Same / Swap | alternate posterize bands swap the two inks |
| S11 | Chroma | Smooth / Poster | Smooth / Poster: each chroma channel snapped to three levels (nine colours, grey included), dithered by K2. Works in both orders |
| P12 | Balance | 0–100% | the ink split sweeps through the picture: 0% high-key (mostly light ink), 50% split at mid-grey, 100% low-key (mostly shadow ink). Glided |

## Presets

Default, Band Swap, Dithered Inks, Poster Inks.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
