# CGA

Re-renders video as a real IBM CGA screen: the three Mode 4 palettes, 640-dot mono, and composite artifact colour on a native 320×200 block grid, with dither, scanlines and snow.

**P12 (Dither):** Adds ordered dither across the block grid, from flat CGA colour to heavily dithered shading.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Mode | Grn/Red / Cyn/Mag / Cyn/Red / Mono 640 / Composite | Picks the CGA mode: three 4-colour palettes, 640-dot mono, or composite artifact colour |
| K2 | H Size | 2x / 4x / CGA / 8x / 16x | Horizontal block size, 2x up to 16x (CGA = native 320-wide grid at 1080p); sizes scale with the screen resolution |
| K3 | V Size | 2x / 4x / CGA / 8x / 16x | Vertical block size, 2x up to 16x (CGA = native 200-line grid at 1080p); sizes scale with the screen resolution |
| K4 | Contrast | 0–100% | Picture contrast before colour matching; 50% leaves it unchanged |
| K5 | Color | 0–100% | Nearest: source colour strength before matching; Luma/Composite: palette saturation, grey to true CGA colours |
| K6 | Ink | Black / Blue / Green / Cyan / Red / Magenta / Brown / Gray | Background / colour-0 ink (the real CGA background register); mono foreground |
| S7 | Stability | Off / On | On: anti-flicker, holds colour choices steady on still or noisy video |
| S8 | Intensity | Low / High | Low or high intensity palette (dim vs bright CGA colours) |
| S9 | Scanlines | Off / On | On: dark gaps between block rows, like a CRT |
| S10 | Color Mode | Nearest / Luma | Nearest: match each block to the closest colour; Luma: assign colours by brightness |
| S11 | Snow | Off / On | On: CGA "snow", random bright specks like the real card's memory glitch |
| P12 | Dither | 0–100% | Ordered dither across the blocks: 0 = flat colours, up = more dithered shading |

## Presets

Mode 4 Hi, Blue Screen, Composite, Green Mono, Snowstorm.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
