# Oubliette

A full-colour top-down turn-based roguelike: pick a class, explore fogged dungeon floors with pillars, pools and lava, fight with class spells and potions, and slay the dragon on depth 7. One flip of switch 7 is one turn; the slider aims your heading.

**P12 (Heading):** 8-way heading for steps and attacks: the top of the travel is N, and pushing up from the bottom turns clockwise NE, E, SE, S, SW, W, NW, N. A reticle marks the target cell.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Class | Warrior / Mage / Rogue / Cleric | Hero class, picked on the title screen (its stats are shown) |
| K2 | Seed | 0–100 | Dungeon seed (with the depth) |
| K3 | Action | Attack / Defend / Cast / Item / Flee | Combat action for the next round (Cast = the class spell) |
| K4 | Zoom | Far / Wide / Normal / Close | Cell size 16 / 32 / 64 / 128 px |
| K5 | Torch | 0–100 | Light radius 2-8 cells; the top of the travel = no fog |
| K6 | Palette | Colour / Game Boy / Amber / CGA / C64 | Palette theme; Colour tints each depth |
| S7 | Step/Act | A / B | Each flip: one turn (step, attack, start) |
| S8 | Map | Off / On | Whole-level map view (explored areas) |
| S9 | Potion | A / B | Each flip drinks a potion |
| S10 | Peaceful | Off / On | Monsters never move or attack |
| S11 | New Game | Run / New | Each flip restarts at the title screen |
| P12 | Heading | 0–100 | 8-way heading; the top of the travel is N |

## Presets

Warrior, Mage, Rogue, Cleric.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
