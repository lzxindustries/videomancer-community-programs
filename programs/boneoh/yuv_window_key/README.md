# YUV Window Key

A per-channel YUV window keying effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Window keying isolates a range of pixel values within a defined lower–upper threshold window. For each channel (Y, U, V) a Low and High knob define the window; pixels whose values fall within that range are considered "in-window." Switch 1 selects between Normal and Matte mode, which determines how the three per-channel gate results are combined and what appears in the output. Because the keying operates directly in YUV space (no colour conversion), the thresholds work on luma (Y) and the two chroma-difference channels (U = Cr, V = Cb) as they arrive from the hardware.

## Controls

### Knobs

| Knob | Operation | Description                                                                   |
|------|-----------|-------------------------------------------------------------------------------|
|  1   | Y Low     | Lower threshold for the Y (luma) channel (0% = black, 100% = white)          |
|  2   | U Low     | Lower threshold for the U (Cr, red-difference) channel                        |
|  3   | V Low     | Lower threshold for the V (Cb, blue-difference) channel                       |
|  4   | Y High    | Upper threshold for the Y channel                                             |
|  5   | U High    | Upper threshold for the U channel                                             |
|  6   | V High    | Upper threshold for the V channel                                             |

U and V neutral (no colour) is at 50% (512). Values below 50% are negative chroma; values above 50% are positive chroma.

### Switches

| S1 | Operation | Description                                                                           |
|----|-----------|---------------------------------------------------------------------------------------|
|  0 | Normal    | Per-channel independent gating; S2/S3/S4 have no effect                               |
|  1 | Matte     | Three channel gates OR-combined into a single matte; S2/S3/S4 select the output value |

| S2 | S3 | S4 | Operation    | Description                                                                |
|----|----|----|--------------|----------------------------------------------------------------------------|
|  0 |  0 |  0 | Logical OR   | White (Y=1023, U=V=512) if any channel is in-window, else black            |
|  0 |  0 |  1 | Bitwise OR   | Y = OR of masked channel values (original if gate passed, else 0); U=V=512 |
|  0 |  1 |  0 | Logical AND  | White if all masked values are non-zero, else black (default)              |
|  0 |  1 |  1 | Bitwise AND  | Y = AND of masked channel values (original if gate passed, else 0); U=V=512 |
|  1 |  0 |  0 | Luma         | Y = Y channel value; U=V=512 (neutral chroma)                              |
|  1 |  0 |  1 | LFSR         | Y = frame-locked noise; U=V=512                                            |
|  1 |  1 |  0 | PRNG         | Y = free-running noise; U=V=512                                            |
|  1 |  1 |  1 | Passthrough  | Original Y, U, V (colour)                                                  |

| S5 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Normal    | Full-range knob sensitivity                            |
|  1 | Fine      | Half-sensitivity for precise threshold adjustment      |

### Slider

| Slider | Operation    | Description                                                                 |
|--------|--------------|-----------------------------------------------------------------------------|
|  12    | Global Blend | Wet/dry blend between the original and keyed signal (0% = original, 100% = keyed) |

## Key Inversion

When a channel's Low knob is set **above** its High knob, the window inverts automatically for that channel: pixels outside the normal (High, Low) gap pass, and pixels inside are blocked. Each channel inverts independently — no extra switch is needed.

## Normal Mode

With S1 in Normal mode, each channel gates independently. Y outputs its original value if in-window, else 0. U outputs its original value if in-window, else 512 (neutral chroma). V outputs its original value if in-window, else 512. S2, S3, and S4 have no effect in Normal mode.

## Matte Mode

With S1 in Matte mode, the three per-channel gates are OR-combined into a single matte gate: if any channel is in-window, the pixel passes. Pixels where no channel is in-window output black (Y=0, U=512, V=512).

For pixels that pass the matte gate, S2/S3/S4 select the output value. The computation uses *masked values* — a channel's original value if its gate passed, or 0 if it did not. All non-Passthrough modes output neutral chroma (U=V=512):

**Logical OR** — Y=1023 (white luma) for any pixel that passed the OR gate.

**Bitwise OR** — Y = bitwise OR of the three masked channel values, producing a greyscale luma result.

**Logical AND** — Y=1023 if all three masked values are non-zero (i.e. all channels passed their individual gate), else Y=0.

**Bitwise AND** — Y = bitwise AND of the three masked channel values, producing a greyscale luma result.

**Luma** — Y = the original Y (luma) value of the pixel, passed directly as the output luma.

**LFSR** — Y = a 10-bit frame-locked noise value (Fibonacci LFSR, polynomial x¹⁰ + x⁷ + 1, reseeded each frame).

**PRNG** — Y = free-running noise using the same polynomial as LFSR but never reseeded. The noise pattern shifts by the number of active pixels each frame, producing a different phase every frame.

**Passthrough** — the original Y, U, V pixel is output directly. The window checks are still performed, but the output is always the original colour.

## Fine Mode

With Fine **On**, the current knob positions are latched as reference values at the moment of switching. Each knob then controls its threshold as `(knob + 7 × reference) / 8`, giving one-eighth the normal sensitivity and allowing very precise adjustment anywhere in the 0–1023 range. Switching back to Fine Off restores full-range control immediately.

## Typical Use

1. Set a matte mode (Logical AND, S2=0 S3=1 S4=0, is the default).
2. Switch to Matte mode (S1=1) and adjust each channel's Low/High knobs until the target region passes the gate.
3. Switch to Normal mode (S1=0) to confirm per-channel gating, then back to Matte for the combined output.
4. Use the Global Blend slider to fade between the original and keyed result.
5. Use Fine mode for precise threshold adjustment once the range is roughly set.

## Comparison with RGB Window Key

| Feature              | YUV Window Key               | RGB Window Key                      |
|----------------------|------------------------------|-------------------------------------|
| Colour space         | YUV direct                   | RGB (full BT.601 YUV↔RGB conv.)     |
| Pipeline latency     | 7 clocks                     | 14 clocks                           |
| BRAM usage           | 0 block RAMs                 | 11 block RAMs                       |
| Mode 100 matte       | Y value (luma passthrough)   | BT.601 computed luma                |
| Best for             | Luma/chroma ranges, fast     | Isolating specific RGB colours      |

Use YUV Window Key when your threshold decisions map naturally onto luma or chroma values (e.g., isolating a bright sky, removing a specific hue cast). Use RGB Window Key when the region of interest is best defined in terms of red, green, and blue levels.

## Technical Notes

- **Colour space:** Direct YUV (no colour conversion)
- **Pipeline latency:** 7 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 0 (no lookup tables required)
- **UV convention:** U = Cr (red-difference), V = Cb (blue-difference) — Videomancer hardware convention; thresholds apply to the raw hardware values
- **Neutral chroma:** U = V = 512 (50%)

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
