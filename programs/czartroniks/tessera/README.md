# Tessera

Up to 64 equilateral triangles rain down the screen, each tumbling end-over-end like a falling card, fluttering side to side and slowly spinning, in four depth layers (big and fast in front, a small slow swarm behind). K5 sets the count, the slider scales them, and a switch turns them into two-sided cards with video on the front.

**P12 (Scale):** Scales every triangle from 0.25x to 1.5x of its count's natural size (0.75/sqrt(count) of the screen height, capped at 75%) on an even sweep: fine confetti up to big overlapping cards. SD modes top out at 1.1x (line-time budget).

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Outline Color | 0–100% | Outline colour, white -> rainbow -> black (back faces use its toned-down version) |
| K2 | Spin | 0–100% | In-plane spin rate; 0 = none |
| K3 | BG Color | 0–100% | Background colour (toned-down palette) |
| K4 | Fall Speed | 0–100% | How fast triangles fall (far layers slower) |
| K5 | Count | 0–100% | Number of triangles, 1-64 (quadratic curve) |
| K6 | Tumble Speed | 0–100% | How fast triangles tumble end over end |
| S10 | Depth | Flat / Layers | Flat: one size and speed; Layers: 4 depth layers, big and fast in front, small and slow behind |
| S11 | Video | Off / On | Off: outlines only; On: video on front faces, toned-down outline colour on the backs |
| P12 | Scale | 0–100% | Scales every triangle 0.25x-1.5x of its count's natural size |

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
