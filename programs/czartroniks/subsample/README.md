# Subsample

Chroma held in strips or squares, bled and dot-crawling like a bad composite feed, cut into hashed blocks that shift or hold.

**P12 (Density):** fraction of blocks moshed, ~5% .. ~98%

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Block | 8–64 px | block size 8..64 px, continuous |
| K2 | Shift | 0–100% | moshed-block displacement, ±48..±1012 px HD (±24..±506 SD), wraps inside the picture |
| K3 | Rate | 0–100% | re-roll every ~2 s .. every field, exponential |
| K4 | Hold | 1–64 px | chroma sampled every 1..64 px, every integer size |
| K5 | Bleed | 0–7 | chroma IIR smear, 0 (off) .. 7 (~128 px tail); live in every switch setting |
| K6 | Crawl | 0–100% | dot-crawl depth: luma dots where the output has colour (triangle wave, ~4 px SD / ~11 px HD) |
| S7 | Blocks | Shift / Hold | Shift / Hold (a moshed block repeats its first column) |
| S8 | Chroma | After / Before | After the mosh (hold grid fixed to the screen) / Before (blocks carry their hold boxes and smear) |
| S9 | Crawl | Checker / Bars | Checker: dots alternate line to line and crawl / Bars: vertical bars that march sideways |
| S10 | Moshed | Colour / Grey | moshed blocks keep their colour / go grey |
| S11 | Hold Shape | Line / Square | Line: chroma held along the line (4:1:1) / Square: the held sample also repeats down Hold lines (4:2:0 and worse: Hold × Hold colour squares) |
| P12 | Density | 0–100% | fraction of blocks moshed, ~5% .. ~98% |

## Presets

Default, VHS Dub, Broken Codec, Dot Crawl.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
