# Pixel Beach

A side-on 8-bit beach driven by real shallow-water physics: waves roll in, steepen, crash into foam and drain back, and a wash sheet comes down over the sand, leaving wet sand that dries where it touched. Sky, sun or moon, gulls, twinkling stars after sunset, and a red crab that wanders the sand and runs from the wash.

**P12 (Sun):** Rides the sun (or the moon, with S10 Night) up and down the sky; in Day the palette follows it through golden and sunset, the sky fades to dark and the stars come out.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Rhythm | 0–100% | Tempo of the waves and surges |
| K2 | Tide | 0–100% | Water level, moves the shoreline |
| K3 | Foam | 0–100% | Amount of white foam |
| K4 | Chop | 0–100% | Extra small choppy waves |
| K5 | Swell | 0–100% | Wave size and energy |
| K6 | Time | 0–100% | Physics speed: 1-4 physics ticks per field |
| S7 | Gulls | Off / On | On: flapping gulls in the sky |
| S8 | Pixels | 16px / 8px | Big 16px or small 8px pixels |
| S9 | Storm | Off / On | On: stormier, bigger waves |
| S10 | Sky | Day / Night | Day or night sky |
| S11 | Crab | Off / On | On: a red crab wanders the sand and runs from the wash; Off: it burrows into a fading hole |
| P12 | Sun | 0–100% | Sun or moon height; sunset colours, then stars |

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- `processing_delay_clks` is declared 0 (allowed for synthesis programs); the sync latency through the program is 5 clocks.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
