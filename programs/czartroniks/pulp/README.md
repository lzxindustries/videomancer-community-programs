# Pulp

Turns video into a 1965 newspaper/comic print: rotated CMYK halftone rosettes, dot gain, misregistration, posterized comic colour, ink linework, random comic panel layouts, speech bubbles with grawlix, and a two-colour spot-red press. The Press slider runs from clean video to newsprint to pop art.

**P12 (Press):** Master macro: clean video, then newsprint halftone, then pop-art (screen size, ink, posterize and gain together).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Screen | 8–96px | Halftone dot cell size (knob up = bigger dots) |
| K2 | Shape | 0–100 | Dot shape (line, diamond, round, ellipse, square) |
| K3 | Ink | 0–100% | Ink density, lighter to heavier |
| K4 | Paper | 0–100% | Paper: bright to aged yellowed pulp |
| K5 | Linework | 0–100% | Thickness of comic outlines |
| K6 | Register | 0–100% | Colour plate misalignment |
| S7 | Color | B&W / CMYK | Black-and-white or four-colour CMYK print |
| S8 | Posterize | Off / On | On: flat comic colour at any Press |
| S9 | Panels | Off / On | On: comic panel frames |
| S10 | Bubbles | Off / On | On: speech bubbles appear |
| S11 | Spot | Off / Red | Two-colour press: black + spot red (with B&W: one red ink) |
| P12 | Press | 0–100% | Clean video to newsprint to pop art |

## Presets

Daily News, Sunday Funnies, Morgue Copy, Etching Desk, Pop Art, Misprint Comic.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
