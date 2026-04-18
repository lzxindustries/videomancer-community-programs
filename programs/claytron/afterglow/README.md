# Afterglow

A horizontal video delay effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Afterglow is a per-line horizontal pixel delay that produces a VHS-style smear.
Each scanline is written into a line buffer and read back at a shifted address, so content appears displaced from where it was captured.
The shift for a given line is built from a uniform base offset (Delay), an optional per-line wave (Shape) that drifts across frames (Speed), and a random jitter term (Noise).
Independent offsets for the U and V chroma planes (Chroma-U / Chroma-V) split the colour from the luma for fringing, and a feedback mode mixes the buffer's own output back into its input on write to leave geometrically-decaying echo trails behind moving content.

## Controls

### Knobs


| Knob | Control  | Function |
|------|----------|----------|
|  1   | Delay    | Uniform horizontal shift applied to every line (centre = no shift; CCW = left, CW = right) |
|  2   | Shape    | Depth and frequency of the per-line wave added on top of Delay |
|  3   | Speed    | Rate at which the wave pattern drifts vertically from frame to frame |
|  4   | Noise    | Amount of LFSR-based random jitter added to the per-line offset (±32 px at full) |
|  5   | Chroma-U | Extra horizontal offset applied to the U (Cb) plane only (centre = aligned with Y) |
|  6   | Chroma-V | Extra horizontal offset applied to the V (Cr) plane only (centre = aligned with Y) |


### Switches


| Switch | Description | Off                                | On                                    |
|--------|-------------|------------------------------------|---------------------------------------|
|  7     | Channel     | Smear Y only; U/V pass through dry | Smear Y+U+V (chroma follows the shift)|
|  8     | W.Inv       | Wave adds to Delay                 | Wave subtracts from Delay             |
|  9     | Feedbk      | Write live input each line         | Write 50/50 mix of live input and buffer output |
|  10    | Noise mode  | Per-line jitter (whole line shivers) | Per-pixel jitter (grain / static)  |
|  11    | Bypass      | Effect active                      | Input passed through, delay-matched   |


### Slider


| Control | Function |
|---------|----------|
| Blend   | Wet/dry crossfade between the original input and the smeared output (0% = dry, 100% = wet) |


## Delay, Shape, and Speed

The three time-domain knobs work as a chain.

**Delay** is the backbone: a uniform shift that slides every scanline identically.
With Shape and Noise at zero, Delay alone produces a clean, constant horizontal smear where the entire frame appears offset from its original position.

**Shape** modulates the offset per line, turning the uniform shift into a wave that varies down the frame.
At 0% every line is shifted by the Delay amount exactly; as Shape rises, successive lines get progressively more offset variation, producing a sinuous pattern that resembles VHS head-alignment error.
Shape drives both the depth and the frequency of the wave, so higher values mean bigger deflections and more cycles per frame.

**Speed** advances the wave phase each frame so the Shape pattern scrolls vertically over time.
At 0% the pattern is static; at full Speed the wave drifts quickly frame to frame.
Speed has no audible effect while Shape is at zero — there is no wave to drift.

## Noise Mode

The Noise knob adds a small random offset to the per-line shift, driven by a free-running 16-bit LFSR.
The Noise mode switch selects how often that source is sampled.

In **Line** mode, the LFSR is latched once per horizontal sync pulse and held for the whole line.
Every pixel on a given line shares the same random offset, so entire scanlines shiver as a unit — the character of failing VHS tracking.

In **Pixel** mode, the LFSR is sampled every clock, so each pixel picks up an independent random offset.
At low knob settings this looks like subtle grain; at high settings it breaks the smear up into speckle and static.

## Feedback

With Feedbk **On**, the data written into the line buffer is a 50/50 average of the live input and the buffer's own read output from that pixel position.
The live signal keeps entering the buffer (so Delay, Shape, and Noise continue to shape the effect), but previously-shifted content feeds back into successive passes, producing a geometric echo trail behind moving content.

The Channel switch gates which planes participate.
In **Y** mode, only luma feeds back — U and V are written live each line.
In **Y+U+V** mode, all three channels feed back, so the echo trails carry colour.

## Chroma Offsets

Chroma-U and Chroma-V apply an extra horizontal offset to the U and V planes respectively, on top of whatever Delay/Shape/Noise shift Y is receiving.
With both knobs centred the three planes stay aligned; offsetting them produces colour fringing along smear edges.

Both knobs are only active when the Channel switch is set to **Y+U+V**.
In **Y** mode, U and V are passed through unshifted regardless of the chroma knob positions.

## Wave Invert

W.Inv flips the sign of the Shape contribution without affecting Delay, Noise, or Chroma offsets.
With Delay centred, flipping W.Inv mirrors the wave pattern horizontally.
With Delay off centre, it lets the Shape peaks push either with or against the uniform shift.

## Bypass

Bypass passes the input video directly to the output through a 6-clock delay line that matches the processing pipeline latency, so sync alignment is preserved whether the effect is active or not.

## Typical Use

1. Push the **Blend** fader to 80%.
2. Turn **Delay** off centre to hear the basic horizontal shift.
3. Nudge **Shape** upward to introduce a per-line wave on top.
4. Raise **Speed** so the wave scrolls.
5. Add a touch of **Noise** for tracking jitter, and choose **Line** or **Pixel** Noise mode to taste.
6. Toggle **Channel** to **Y+U+V** and dial **Chroma-U** / **Chroma-V** off centre for colour fringing.
7. Flip **Feedbk** on to leave echo trails behind moving content.

## Technical Notes

- **Colour space:** YUV 4:4:4, 10-bit per channel
- **Pipeline latency:** 6 clock cycles (2 for the line buffer read, 4 for the wet/dry interpolator)
- **Line buffer:** Three dual-bank BRAMs (Y/U/V), 2048 entries each — one line written while the other is read at the shifted address
- **Edge handling:** Out-of-range offsets clamp to the edge pixel instead of wrapping into uninitialised BRAM entries
- **Noise source:** Free-running 16-bit LFSR, lower 10 bits used as the random offset
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 claytron
GPL-3.0-only — see [LICENSE](LICENSE)
