# Crush

Averaged mosaic squeezed toward a drifting line with sync-loss tearing; tiles stay square or squeeze with the picture.

**P12 (Collapse):** the picture squeezes toward the line, ×1 to ×64, exponential: each sixth of the travel doubles it. 100% = a ~30 px sliver on HD

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Line | 0–100% | where the picture collapses to, 0–100% of the width (in both S8 modes) |
| K2 | Sync | 0–100% | sync loss: random per-line jumps up to ±¼ of the width, the same size on screen at any Collapse |
| K3 | Roll | 0–100% | the collapse line drifts right on its own and wraps; speed (0–4 px/frame) |
| K4 | Cells | 2 px / 4 px / 8 px / 16 px / 32 px / 64 px / 128 px / 256 px | cell width 2, 4, 8, 16, 32, 64, 128, 256 px (deadbanded) |
| K5 | Aspect | Square / Wide / Wider / Strip | cell height = width / 1, 2, 4, 8: Square, Wide, Wider, Strip |
| K6 | Grout | 0–100% | grid lines along the cells' real edges, 1/8 of a cell thick (1–8 px / lines); none on cells under 4 px |
| S7 | Tiles | Square / Squeeze | Square: screen-fixed cells over the squeezed picture. Squeeze: the cells squeeze with the picture |
| S8 | Collapse | Both Sides / Right Side | Both Sides squeeze toward the line. Right Side: the left of the line stays put and the right side squeezes into it (at K1 = 0 the whole picture squeezes to the left edge) |
| S9 | Loss | Lines / Rows | Lines: every line jumps. Rows: whole cell rows jump together (bands of 8+ lines) |
| S10 | Beyond | Black / Stretch | black outside the picture / the first and last cells stretched |
| S11 | Grout | Black / White | black / white |
| P12 | Collapse | 0–100% | the picture squeezes toward the line, ×1 to ×64, exponential: each sixth of the travel doubles it. 100% = a ~30 px sliver on HD |

## Presets

Default, Tile Crush, Drain Left, Wide Mosaic.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
