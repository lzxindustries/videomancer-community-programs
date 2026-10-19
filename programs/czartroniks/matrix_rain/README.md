# Matrix Rain

Classic green digital rain: three falling drops per column of half-width katakana and digits, with fading tails, a bright head glyph and random mutation. The input picture can size the glyphs and hold the tails, over black or live video.

**P12 (Image):** How strongly the input picture shapes the rain: glyphs grow up to 2x and tails linger up to 3.8x under bright areas, and shrink under dark ones. 0% = plain rain.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Fall Speed | 0–100% | How fast the drops fall |
| K2 | Density | 0–100% | How many drops are falling (shorter gaps between them) |
| K3 | Tail Length | 0–100% | Length of each drop's fading tail |
| K4 | Mutation | 0–100% | How often glyphs change at random |
| K5 | Colour | 0–100% | Rain colour: green at 0%, round the hue wheel, white at the top |
| K6 | Glyph Size | 16–128px | Size of the glyph cells |
| S7 | Head Glow | Off / On | Bright leading glyph on each drop |
| S8 | Mirror | Off / On | Mirrors the glyphs |
| S9 | Rise | Off / On | Rain falls down or rises up |
| S10 | Background | Black / Video | Rain over black or over the live video |
| S11 | Glitch | Off / On | Occasional torn bands of rows or columns |
| P12 | Image | 0–100% | Picture's hold on the rain: bigger glyphs, longer tails where bright |

## Presets

Classic Green, Heavy Downpour, Sparse Drift, Toxic Lime, Matrix Vision, Rain Over Video, Rising Code, Glitch Signal.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.

The katakana glyphs in `unifont_jp.hex` are a subset of GNU Unifont 15.1.05
(http://unifoundry.com/unifont/), Copyright Roman Czyborra, Paul Hardy, et al.,
dual-licensed GPL-2.0-or-later with the GNU
font embedding exception, and SIL OFL 1.1.
