# Vhold

Rolling V-hold bar with jello; rows squeeze into tiled strips, hardest just below the bar, glowing as the beam concentrates.

**P12 (Pull):** extra squeeze just below the bar, decaying with Reach; glided, with a small floor (≈1.5x at 0%)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Roll | -100–100% | roll speed, bipolar; a small deadband at the centre = locked. ~5.7 s a cycle at the default, 0.55 s at the ends, the same in every mode |
| K2 | Bar | 1–26% | bar height 0.8 .. 26% of the screen (default 2.9% = 31 lines at HD) |
| K3 | Jello | 0–100% | sine lean, up to 1/8 of the width (±240 px HD); 1.5 sine cycles a screen in every mode |
| K4 | Collapse | 1–9x | base squeeze 1x .. 9x, glided |
| K5 | Reach | 0–100% | how far below the bar the pull and the tear reach: 1/32 .. 8 screen heights (quadratic; default ≈ 1.9 heights) |
| K6 | Beam | 0–100% | squeezed rows brighten; stretched rows dim |
| S7 | Bar | Black / Show | Black / Show (the bar's rows show the peak-pull picture) |
| S8 | Centre | Screen / Jello | Screen: strips stay centred, jello pans the content / Jello: the collapse centre wobbles, so whole strips swing |
| S9 | Edges | Black / Mirror | Black / Mirror |
| S10 | Tear | Off / On | random per-line tear of up to ±1/8 of the width, strongest just below the bar and fading with Reach |
| S11 | Pull | Squeeze / Stretch | Squeeze / Stretch; Stretch zooms by the same factor that Squeeze squeezes |
| P12 | Pull | 0–100% | extra squeeze just below the bar, decaying with Reach; glided, with a small floor (≈1.5x at 0%) |

## Presets

Default, Dying Set, Jello Stretch, Torn Below.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
