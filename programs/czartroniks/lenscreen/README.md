# Lenscreen

Sphere or tunnel lens under a halftone dot screen (flat or warped with the lens) with dot-crawl and rainbow-comb crosstalk.

**P12 (Squeeze):** horizontal scale, 50% (2× stretch) to 200% (two picture widths across), glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Bulge | -100–100% | lens strength, bipolar (pinch .. bulge), glided |
| K2 | Radius | 3–140% | lens radius, 3–140% of the half-height, glided |
| K3 | Cell | 4 px / 8 px / 16 px / 32 px | halftone cell 4 / 8 / 16 / 32 px, deadbanded |
| K4 | Dot | Circle / Diamond / Square / Line | circle / diamond / square / line, deadbanded |
| K5 | Cross-Y | 0–100% | chroma leaking into luma (dot crawl): a U/V subcarrier, period 8 px at HD, 4 px in SD, that crawls up a line per field |
| K6 | Cross-C | 0–100% | fine luma detail leaking into chroma, on every pixel |
| S7 | Lens | Sphere / Tunnel | Sphere / Tunnel |
| S8 | Screen | Flat / Lensed | Flat (cells on the screen grid) / Lensed (cells warped with the lens) |
| S9 | Ink | Mono / Colour | Mono (black ink) / Colour (ink takes the picture's colour) |
| S10 | Cross-Y | Dots / Print | Dots (before the screen: rattles the dots) / Print (after: flickers the print) |
| S11 | Fringe | Off / On | cross-colour V flips on alternate rows (rainbow comb) |
| P12 | Squeeze | 50–200% | horizontal scale, 50% (2× stretch) to 200% (two picture widths across), glided |

## Presets

Default, Fisheye Print, Tunnel Comb, Flat Screen.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
