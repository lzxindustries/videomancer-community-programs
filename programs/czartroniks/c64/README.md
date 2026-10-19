# C64

Re-draws video as a Commodore 64 screen: 16 Colodore colours on a native 320x200 block grid, multicolour fat pixels, checkerboard dither, a real-proportioned border with tape-loading stripes, and a READY. prompt.

**P12 (Dither):** Ordered 4x4 Bayer dither per block, from flat colour to heavy checkerboard game-art.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Size | 2x / 4x / C64 / 8x / 16x | Block size; C64 = native 320x200 grid at 1080p; sizes scale with the screen resolution |
| K2 | Border | 0–100% | Border size as a fraction of the screen; 0 = none |
| K3 | Border Col | Black / White / Red / Cyan / Purple / Green / Blue / Yellow / Orange / Brown / Lt Red / Dk Grey / Md Grey / Lt Green / Lt Blue / Lt Grey | Border colour, also the READY. text colour |
| K4 | Scanlines | 0–100% | Dims the last line of every block row; 0 = off |
| K5 | Color | 0–100% | Chroma gain before matching: greys only to 5x |
| K6 | Contrast | 0–100% | Contrast around mid-grey; 50% = unchanged |
| S7 | Stability | Off / On | Anti-flicker: hysteresis grids hold block colours steady |
| S8 | Multicolor | Off / On | Double-wide pixels (160x200 at native size) |
| S9 | Color Mode | Nearest / Ramp | All 16 colours, or brightness mapped onto an 8-step C64 ramp |
| S10 | Loading | Off / On | Tape-loading stripes flicker in the border |
| S11 | Ready | Off / On | READY. prompt with blinking cursor |
| P12 | Dither | 0–100% | Ordered checkerboard dither per block, flat to heavy |

## Presets

Boot Screen, Game Art, Tape Loading, Demo Ramp, Big Pixels, Hi-Res.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
