# POLKA

Retro 1950s polka dots over video or a palette background: 8 diner-style palettes, 8 layouts (grid, brick, honeycomb, clusters, tumble, scatter, halftone fade), portholes into the video, and a Swell fader that grows the dots until they kiss, merge and invert.

**P12 (Swell):** Grows every dot until they kiss, merge into a lattice and finally invert (background becomes the dots).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Dot Size | 0–100% | Size of the dots |
| K2 | Spacing | 0–100% | Spacing between dots |
| K3 | Layout | Grid / Brick / Honeycomb / Diamond / Cluster / Tumble / Scatter / Fade | Dot arrangement (grid, brick, honeycomb, scatter...) |
| K4 | Palette | Diner Cherry / Seafoam Coral / Atomic Teal / Butter Turq / Blush Charcoal / Mint Chocolate / Midnight Fiesta / Linen Ink | Retro colour combination |
| K5 | Tint | 0–100% | Rotates all colours (speed in Cycle mode) |
| K6 | Color Mode | Solid / Duotone / Rows / Confetti / Cycle | How dots pick colours (solid, checker, rows, confetti, cycle) |
| S7 | Swap | Normal / Inverted | Normal or swapped background/dot colours |
| S8 | Backdrop | Palette / Video | Background from palette or input video |
| S9 | Fill | Paint / Porthole | Paint: solid dots / Porthole: video inside dots |
| S10 | Edges | Hard / Soft | Hard or soft dot edges |
| S11 | Motion | Still / Sway | Still or gently swaying dots |
| P12 | Swell | 0–100% | Dots grow, merge, then invert |

## Presets

Diner Classic, Soda Fountain, Atomic Lounge, Thrown Confetti, Porthole Blush, Ink Reverse, Halftone Fade.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
