# Parallax

Luma becomes depth: bright and dark shift sideways as the camera sways; chroma reads at its own offsets, channels swap.

**P12 (Position):** camera position, bipolar (centre = flat when Sway is 0)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Depth | 0–100% | parallax gain; floored at 1/8, so Position always moves the picture |
| K2 | Sway | 0–100% | the camera sways on its own: 0 = still and centred, then 68 s → 0.27 s per cycle (the speed doubles every 1/8 of the knob; the bottom 1/8 fades the sway in). Default 37.5% = 8.5 s |
| K3 | Focus | 0–100% | luma level that does not move, from black to white |
| K4 | Swap | Off / U<>V / Y>U / U>Y / Y<>V / Rotate / Rotate Back / Complement | channel mode: Off / U<>V / Y>U / U>Y / Y<>V / Rotate / Rotate Back / Complement |
| K5 | Offset | -128–128 px | both chroma channels read ±128 px away (colour ghost); px at 1080p, scaled to the screen in other modes |
| K6 | Split | 0–127 px | U and V pulled apart in opposite directions, 0 … 127 px at 1080p (scaled to the screen in other modes) |
| S7 | Depth | Luma / Sat | from luma / from saturation |
| S8 | Near | Bright / Dark | bright is near / dark is near |
| S9 | Chroma | Moves / Stays | moves with the luma / stays put |
| S10 | Sway | Glide / Wiggle | Glide (sine) / Wiggle (snaps between two viewpoints: a wigglegram) |
| S11 | Edges | Fill / Wrap | Fill: reads past the edge take the edge pixel / Wrap: they wrap round |
| P12 | Position | -100–100% | camera position, bipolar (centre = flat when Sway is 0) |

## Presets

Default, Relief Map, Colour Ghost, Rotated.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.
- Displacements follow the screen height: the px figures are 1080p pixels (two thirds as many at 720p, half at SD), so the parallax looks the same in every mode.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
