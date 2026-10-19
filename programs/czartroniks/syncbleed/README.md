# Syncbleed

Chroma bleeds, lags and dot-crawls like a bad composite feed while rows collapse toward a seam under a rolling wave.

**P12 (Loss):** the squeeze multiplies x1 .. x64, exponentially; bleed +0..3 octaves, crawl climbs. 100% collapses rows to a strip a few pixels wide (about 14 px at HD at the default K4)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Bleed | 0–7 | chroma smear (IIR) in octave steps: off, 2 .. 128 px; Local and P12 can push it to 256 px |
| K2 | Lag | 0–63 px | chroma lags luma by 0 .. 63 source px |
| K3 | Crawl | 0–100% | dot crawl: subcarrier dots, 188 cycles across the line (~10 px pitch at HD, 3.8 px at SD), phased by hue, strongest on colour, a faint floor on grey |
| K4 | Collapse | 1–9x | base squeeze toward the seam, 1x .. 9x, glided |
| K5 | Seam | -100–100% | collapse centre, bipolar (centre = screen centre) |
| K6 | Wave | 0–100% | the squeeze breathes down the picture (1.58 sine cycles a screen in every mode, ~3.6 s a cycle), glided |
| S7 | Bleed | Before / After | Before the collapse (smear squeezed too) / After (screen-space smear) |
| S8 | Length | Fixed / Local | Fixed / Local: two bleed steps per octave of squeeze, so squeezed rows smear longer and stretched rows sharpen |
| S9 | Chroma | Same / Fringe | Same / Fringe (chroma squeezed 1.5x more than luma) |
| S10 | Crawl | Locked / Drift | Locked (still dots) / Drift (the subcarrier slips 270° a field, so the dots crawl as on NTSC) |
| S11 | Smear | Chroma / Luma too | chroma only / luma too |
| P12 | Loss | 0–100% | the squeeze multiplies x1 .. x64, exponentially; bleed +0..3 octaves, crawl climbs. 100% collapses rows to a strip a few pixels wide (about 14 px at HD at the default K4) |

## Presets

Default, Bad Cable, Breathing, Total Loss.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
