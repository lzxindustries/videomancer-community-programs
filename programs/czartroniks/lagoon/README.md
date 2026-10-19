# Lagoon

Puts the picture underwater: ripple refraction, a rolling swell, dancing caustic highlights that sway with the waves, and a blue-green grade. The slider dives from dry to deep.

**P12 (Depth):** Master dry to wet: blends from untouched video to the full underwater treatment.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Ripple | 0–95px | Fine ripple wobble of the picture |
| K2 | Waves | 0–255px | Big sideways wave sway of the picture |
| K3 | Speed | -100–100% | Wave speed and direction; centre stops |
| K4 | Scale | 0–100% | Size of ripples, waves and light patterns |
| K5 | Tint | 0–100% | How strongly colours are tinted toward sea colour |
| K6 | Caustics | 0–100% | Amount of dancing light caustics |
| S7 | Hue | Aqua / Abyss | Aqua or deep abyss tint colour |
| S8 | Glitter | Off / On | On: twinkling white glitter on the light |
| S9 | Murk | Off / On | On: picture darkens toward the bottom |
| S10 | Sea | Calm / Storm | Calm or stormy, bigger faster waves |
| S11 | Prism | Off / On | Chromatic refraction: colour fringes split to either side of the image and sway with the waves |
| P12 | Depth | 0–100% | Depth: dry video to full underwater look |

## Presets

Swimming Pool, Open Ocean, The Abyss, Storm Surge, Gentle Stream, Puddle Shimmer.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
