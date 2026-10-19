# Iris

A perspective spiral dot tunnel shaped like an eye: rings of round anti-aliased dots crowd toward an off-centre pupil you steer with K1/K2. The slider splits red/green/blue copies of the dots into misregistered fringes, up to a clean RGB triad, and S8 can put the incoming video in place of the black or white ground.

**P12 (Split):** Fans the R/G/B dot layers apart from the pupil outward (square-law weighted). Below 3% the dots are pure white; from about 70% the rim reaches a clean RGB triad, and at 100% the outer half of the iris is triad.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Pupil X | -100–100% | Moves the pupil left/right |
| K2 | Pupil Y | -100–100% | Moves the pupil up/down |
| K3 | Pupil Size | 0–100% | Pupil size; rings go dark one at a time from the centre |
| K4 | Arms | 6–96 | Number of spiral arms of dots |
| K5 | Rings | 3–48 | Number of rings of dots |
| K6 | Twist | -100–100% | Twists the arms into spirals, either direction |
| S7 | Fill | Dots / Mesh | Dots: round dots; Mesh: connected mesh |
| S8 | Backdrop | Solid / Video | Solid: black (Light) or white paper (Ink) ground; Video: the incoming picture replaces the ground |
| S9 | Mode | Light / Ink | Light: bright dots on dark; Ink: dark on light |
| S10 | Motion | Still / Flow | Still: static; Flow: dots stream outward |
| S11 | Iris | Framed / Full Bleed | Framed: iris circle; Full Bleed: fills screen |
| P12 | Split | 0–100% | Splits colours apart into red/green/blue copies |

## Presets

Reference, Sunburst, Misregister, Vortex, Honeycomb, Window, Triad.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
