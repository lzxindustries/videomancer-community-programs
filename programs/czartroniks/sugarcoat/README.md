# Sugarcoat

Video re-made as candy: every pixel is classed by brightness or hue and redrawn in one of 16 candy textures (chocolate, cookies & cream, choc-chip cookie, cane, mint, icing, taffy, Battenberg, allsorts, waffle, jelly beans, gingerbread, liquorice, nonpareils and more), with gloss, outlines and shading. 64 themed maps put dark textures in shadows and hue-matched candies on colours.

**P12 (Strength):** How much variety the map uses: in Luma mode it opens up from 2 to 4 to 8 textures across the brightness range; in Chroma mode it sweeps the saturation floor, so more of the picture turns into hue-matched candy.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Theme | 0–63 | Which candy goes where: 8 themes (Classic, Bakery, Liquorice...) with swap and tone variants |
| K2 | Clean | 0–100% | Removes specks so candy areas form clean patches |
| K3 | Scale | 8 px / 16 px / 32 px / 64 px | Texture size |
| K4 | Contrast | 0–100% | Contrast of the picture before it is sorted into candies (50% = unchanged) |
| K5 | Level | -100–100% | Brightness offset before sorting; centre = unchanged |
| K6 | Gloss | 0–100% | Shiny highlights from the picture's shape |
| S7 | Outline | Off / On | Dark outlines between candy areas |
| S8 | Shade | Flat / Shaded | Flat candy, or shaded by the picture's brightness |
| S9 | Bank | A / B | Second set of 8 themes |
| S10 | Sprinkles | Off / On | Adds sprinkles |
| S11 | Map | Chroma / Luma | Choose candies by colour or by brightness |
| P12 | Strength | 0–100% | How much variety: more textures (Luma) or more colour candies (Chroma) |

## Presets

Candy Shop, Sweet Shelf, Big Bites, Bakeshop, Cel Candy.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 32 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
