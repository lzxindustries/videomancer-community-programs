# Candy Hills

A Tiny-Wings-style side-scrolling landscape: three parallax layers of striped candy hills with sunlit crests, round bushes and a graded pastel sky. The slider grows the hills from flat plains to ridges sweeping off the screen.

**P12 (Hill Amplitude):** Grows the hills from flat striped plains to ridges that sweep off the top and bottom of the screen, alternating all-hill and all-sky views. Glided, with a noise deadband.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scroll Speed | 0–100% | Continuous scroll speed, 1/8x to ~12x; it never stops |
| K2 | Fan Amount | 0–100% | Stripes widen over the crests and tighten in the valleys (up to ~28–82°) |
| K3 | Sky Tint | 0–100% | Sky hue: rose, lavender, periwinkle, sky blue, mint, lime-cream, warm cream |
| K4 | Palette | Green/Magenta / Mint/Coral / Sunset / Mono / Bubblegum / Lemon/Berry / Aqua/Tangerine / Lavender/Lime | Stripe colour pair |
| K5 | Stripe Width | 0–100% | Stripe width, 66 px up to 1024 px in 64 steps |
| K6 | Bush Density | 0–100% | How many bush groups sit on the crests; 0 = none |
| S7 | 3rd Layer | Off / On | Off removes the middle hill layer and its bushes |
| S8 | Distance Haze | Off / On | Back layers fade toward a pale haze |
| S9 | Crest Line | Off / On | Dark outline along each crest |
| S10 | Video Sky | Off / On | The input video shows through the sky |
| S11 | Reverse | Off / On | Scroll direction |
| P12 | Hill Amplitude | 0–100% | Flat plains up to ridges sweeping off the screen |

## Presets

Candy Country, Mint & Coral, Sunset Ridge, Flat Two-Layer.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
