# Bajaweave

Two swaying line oscillators woven together: vertical stripes whose sway travels up or down the screen, layered over and under at every crossing, in a fully saturated complementary colour pair, with optional outlines and a drop shadow.

**P12 (Speed):** Bipolar flow speed: centre = frozen, ends = about 23 lines per field each way, eased, same vertical speed for both waves.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Osc1 Bars | 5–125 | Number of oscillator 1 bars across the screen |
| K2 | Osc1 Wave | 0–100% | Oscillator 1 sway wavelength (about 1 to 17 cycles per screen) |
| K3 | Color | 0–360deg | Oscillator 1 hue; oscillator 2 is always its complement |
| K4 | Osc2 Bars | 5–125 | Number of oscillator 2 bars across the screen |
| K5 | Osc2 Wave | 0–100% | Oscillator 2 sway wavelength |
| K6 | Sway | 0–100% | Sway amount, from straight lines to about 4x default |
| S7 | Layering | Woven / Stacked | Over/under at every crossing, or oscillator 1 on top |
| S8 | Edges | Off / On | Dark outlines plus a highlight on the outer bends |
| S9 | Depth | Flat / Shadow | The upper ribbon drops a shadow onto the one below |
| S10 | Flow | Opposite / Same | Waves flow against each other, or the same way with a slow drift |
| S11 | Shape | Sine / Triangle | Smooth sine or pointed triangle sway (glides between) |
| P12 | Speed | -100–100% | Flow speed and direction; centre frozen |

## Presets

Default Weave, Slow Drift, Frantic Stripes.

## Technical notes

- Program type: synthesis; core `yuv444_30b`.
- `processing_delay_clks` is declared 0 (allowed for synthesis programs); the sync latency through the program is 19 clocks.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
