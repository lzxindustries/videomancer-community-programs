# Screenprint

Halftone screens at any size and angle, inked in colour and blended into the picture. P12 grows the dots to pop art.

**P12 (Size):** the headline. The screen cell grows from 4 px to 160 px (exponential, glided), zoomed about the picture centre. At 100% about 12 giant dots span an HD line

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Angle | 0–90° | screen angle, continuous 0–90° (glided). With Spin on, it sets the spin speed instead |
| K2 | Mix | 12–100% | 12–100% of the blend |
| K3 | Dot | Circle / Diamond / Square / Line | dot shape: circle / diamond / square / line |
| K4 | Blend | Add / Screen / Multiply / Difference / Hard Light / Overlay | Add / Screen / Multiply / Difference / Hard Light / Overlay, in six equal zones. Add is signed: ink darkens, paper lightens |
| K5 | Ink | Black / Red / Orange / Yellow / Green / Cyan / Blue / Magenta | Black / Red / Orange / Yellow / Green / Cyan / Blue / Magenta. The ink's own brightness goes into the blend, and its colour tints its dots |
| K6 | Gain | 0–100% | halftone contrast: how far the dots open with brightness |
| S7 | Register | Tight / Loose | Tight / Loose: the colour layer is printed 1/32 of the width to the right (60 px on HD) |
| S8 | Invert | Off / On | the halftone inverted (ink where the paper was) |
| S9 | Spin | Off / On | the screen turns about the picture centre. K1 sets the speed: ~4.5 min a turn at 0, ~16 s at 50%, ~4 s at 100% |
| S10 | Colour | Picture / Ink | Picture: the picture's colour, and coloured ink tints its dots / Ink: a one-colour print (ink colour on neutral paper) |
| S11 | Rosette | Off / On | a second screen 30° round. Ink prints where either screen inks |
| P12 | Size | 0–100% | the headline. The screen cell grows from 4 px to 160 px (exponential, glided), zoomed about the picture centre. At 100% about 12 giant dots span an HD line |

## Presets

Newsprint, Pop Print, Screen Burn, Line Overlay, Off Register, Pop Art, Spin Rosette.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
