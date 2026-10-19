# Prism

Rainbow edge trails: every edge in the video becomes a dispersion point spraying ROYGBIV bands to the right. It adds blowout contrast, mono colour schemes with fades, and a luma key to lay the video over or under the rainbows.

**P12 (Band W):** Width of each colour band in the trails, 1–128 px at 1080p (scaled to the same fraction of screen height at 720p and SD), with fine control at the narrow end (squared response).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Col/Fade | 0–100 | Rainbow: rotates band colours; mono: fade to black/white |
| K2 | Edge Thr | 0–100% | How strong an edge must be to start a trail |
| K3 | Noise | 1–16px | Ignores short noisy edges |
| K4 | Poster | 0–100% | Posterizes the picture before edge detection |
| K5 | Contrast | 100–900% | Contrast boost before edge detection |
| K6 | Scheme | Rainbow / Red / Orange / Yellow / Green / Blue / Indigo / Violet | Rainbow or single-colour scheme |
| S7 | Video Key | Off / On | On: input video shows where trails end |
| S8 | Key Pos | Under / Over | Video under or over the colour bands |
| S9 | Smooth | Off / On | On: smooths picture for cleaner outlines |
| S10 | Curve En | Off / On | On: bands change width along the trail |
| S11 | Reverse | Off / On | On: rainbow order reversed |
| P12 | Band W | 0–100% | Width of each colour band; fine control for thin bands, sweeping up to huge sheets of colour |

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
