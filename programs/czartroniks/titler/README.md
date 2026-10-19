# Titler

An 80s/90s-style video character generator: two lines of 16 characters over your video, with eight classic edge/panel styles, editable live from the front panel, and the slider scaling from a caption to giant hero letters.

**P12 (Crawl):** Bipolar news-ticker crawl: centre = stopped, push one way to scroll the text left, the other way to scroll right; wraps around the screen.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | H Position | 0–100% | Text position left to right |
| K2 | V Position | 0–100% | Text position top to bottom |
| K3 | Style | Plain / Drop Shadow / Black Edge / Hollow / Emboss / Box / Half Box / Reverse | Text style: plain, shadow, edge, hollow, emboss, boxes, reverse |
| K4 | Color/Erase | White / Red / Orange / Yellow / Green / Cyan / Blue / Magenta | Text colour; in Edit Mode, erases/scrolls text |
| K5 | Size/Cursor | 0–100% | Edit Mode off: text size, caption to hero; Edit Mode on: moves the edit cursor |
| K6 | Letter | 0–63 | Picks the letter written at the cursor |
| S7 | Edit Mode | Off / On | On: enables cursor and letter editing |
| S8 | Background | Video / Black | Background: live video or solid black |
| S9 | Matte | Off / On | On: letters show the video through them |
| S10 | Font | Default / Sci-Fi | Font: default blocky or Sci-Fi |
| S11 | Justify | Left / Center | Lines aligned Left or Centred |
| P12 | Crawl | -100–100% | Scrolls the text sideways like a ticker; centre stops it |

## Presets

Lower Third, News Bar, Hero Card, Video Fill, Sci-Fi Emboss, Ticker.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 8 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
