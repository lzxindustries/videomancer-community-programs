# Intaglio

The picture redrawn as a copperplate engraving: hatch strokes that follow the form, stroke width carrying the tone, crosshatched shadows, contour ink and clean paper. Styles run from stipple through engraving to woodcut, with a scratchboard invert and 8 ink/paper palettes.

**P12 (Bite):** Acid depth. 0% = contours only; up to 60% walks the tonal ladder in (hatch, cross, third direction); 60-100% over-bites: strokes swell until shadows go solid, crosshatch floods the midtones, foul-bite pits spread over the paper.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Pitch | 0–100% | Stroke spacing, 5.7 to 89 px |
| K2 | Angle | 0–180deg | Rotates the plate; 0 = strokes run along the contours |
| K3 | Ink | 0–100% | Stroke width gain and contour weight |
| K4 | Palette | Copper / Sepia / Sanguine / Prussian / Banknote / Iron Gall / Gold / Blueprint | One of 8 ink/paper pairs |
| K5 | Hand | 0–100% | Hand tremor: per-line wobble in the strokes |
| K6 | Style | 0–100% | Stipple, engraving or woodcut |
| S7 | Invert | Paper / Scratchboard | Dark strokes on paper, or white strokes on inked ground |
| S8 | Wash | Off / On | Half-saturated video colour printed under the strokes |
| S9 | Contour | Off / On | Ink lines along edges |
| S10 | Cross | 90 deg / 45 deg | Angle of the crosshatch against the main strokes |
| S11 | Drift | Locked / Live | Locked plate, or strokes that crawl and boil |
| P12 | Bite | 0–100% | Acid depth: contours, tonal ladder, then over-bitten climax |

## Presets

Copperplate, Scratchboard, Woodcut Poster, Mezzotint.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
