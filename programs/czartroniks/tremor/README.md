# Tremor

Value-noise warp, sawtooth scanline shear and a two-tone palette: noise warps or grains, bands lean and flip tones.

**P12 (Amount):** glided, squared curve: ±5 px warp or faint grain at 0, ±¼ of the width or heavy grain (±half the luma range) at 100%. Above 75% the quake comes in: slabs of lines (a new slab on 1 line in 16, random offsets, new every frame) torn sideways, up to ±⅛ of the width

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scale | 0–100% | noise cell size 4–512 px, log, 64 steps per octave; zooms about the screen centre |
| K2 | Drift | -100–100% | two-way: centre = still, right = the field rises, left = it sinks, up to 8 px a frame. The field boils as it drifts: each row of lattice points also slides sideways at its own random speed (up to the drift speed) |
| K3 | Skew | 0–100% | scanline shear slope; rows lean about the middle of each band, up to ±511 px |
| K4 | Period | 2–257 ln | shear band height 2–257 lines (always visible through S9) |
| K5 | Hue | 0–360° | highlight tone hue |
| K6 | Spread | 0–180° | hue angle from the highlight tone to the shadow tone, 0–178° |
| S7 | Noise | Warp / Grain | Warp: noise displaces the picture horizontally / Grain: noise is added to luma |
| S8 | Shear | Saw / Tri | sawtooth / triangle |
| S9 | Bands | Shade / Flip | Shade: every other band's tone key is lifted a little, so the bands always show / Flip: every other band swaps the two tones |
| S10 | Field | Smooth / Blocky | smooth value noise / hard cells |
| S11 | Tone | Luma / Noise | duotone keyed from luma / from the noise field |
| P12 | Amount | 0–100% | glided, squared curve: ±5 px warp or faint grain at 0, ±¼ of the width or heavy grain (±half the luma range) at 100%. Above 75% the quake comes in: slabs of lines (a new slab on 1 line in 16, random offsets, new every frame) torn sideways, up to ±⅛ of the width |

## Presets

Default, Heat Haze, Venetian, Noise Print.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 32 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
