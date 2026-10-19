# Pyre

Fire that is simulated, not drawn: a 256x64 temperature grid in EBR re-simulated every field, with buoyancy, entrainment and curl-noise turbulence, so tongues stretch, pinch off and dissolve like a fluid and drift in random gusts. Cool parcels linger as smoke. Video can be the fuel, so bright areas burn.

**P12 (Blaze):** Blends the fuel from the K1 setting up to 100%, so the slider alone takes a small flame to a full-screen blaze. Speed and turbulence stay on their knobs.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Fuel | 0–100% | How much fire: burner level, flame pockets and cooling |
| K2 | Width | 0–100% | Squeezes the flames into dense, narrow tongues |
| K3 | Smoke | 0–100% | How long cool parcels linger as smoke, and how bright |
| K4 | Turblnce | 0–100% | Curl-noise swirl strength |
| K5 | Plume | 0–100% | Buoyancy: hot parcels rise faster and pull flames inward |
| K6 | Rise | 0–100% | Base upward speed of the fire |
| S7 | Ember | Off / On | Random hot embers |
| S8 | Luma Mod | Off / On | Bright video areas become burners (burning silhouettes) |
| S9 | Blue Flame | Off / On | Gas-flame colours (yellow to blue, red to cyan) |
| S10 | Overlay | Off / On | Fire drawn over the video, smoke at 50% |
| S11 | Bypass | Off / On | Passes the input video straight through |
| P12 | Blaze | 0–100% | Takes the fire from the Fuel setting to a full-screen blaze |

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
