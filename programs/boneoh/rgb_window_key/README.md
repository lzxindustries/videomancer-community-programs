# RGB Window Key

A per-channel RGB window keying effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Window keying isolates a range of pixel values within a defined lower–upper threshold window. For each channel (R, G, B) a Low and High knob define the window; pixels whose values fall within that range are considered "in-window." Switch 1 selects between Normal and Matte mode, which determines how the three per-channel gate results are combined and what appears in the output. Because the keying operates in RGB space (with full BT.601 YUV↔RGB conversion via BRAM LUTs), the thresholds work directly on red, green, and blue values.

## Controls

### Knobs

| Knob | Operation  | Description                                                       |
|------|------------|-------------------------------------------------------------------|
|  1   | R Low      | Lower threshold for the R (red) channel (0% = black, 100% = white) |
|  2   | G Low      | Lower threshold for the G (green) channel                         |
|  3   | B Low      | Lower threshold for the B (blue) channel                          |
|  4   | R High     | Upper threshold for the R channel                                 |
|  5   | G High     | Upper threshold for the G channel                                 |
|  6   | B High     | Upper threshold for the B channel                                 |

### Switches

| S1 | Operation | Description                                                                           |
|----|-----------|---------------------------------------------------------------------------------------|
|  0 | Normal    | Per-channel independent gating; S2/S3/S4 have no effect                               |
|  1 | Matte     | Three channel gates OR-combined into a single matte; S2/S3/S4 select the output value |

| S2 | S3 | S4 | Operation    | Description                                                                |
|----|----|----|--------------|----------------------------------------------------------------------------|
|  0 |  0 |  0 | Logical OR   | White (1023) if any channel is in-window, else black                       |
|  0 |  0 |  1 | Bitwise OR   | OR of masked channel values (original value if gate passed, else 0)        |
|  0 |  1 |  0 | Logical AND  | White (1023) if all masked values are non-zero, else black (default)       |
|  0 |  1 |  1 | Bitwise AND  | AND of masked channel values (original value if gate passed, else 0)       |
|  1 |  0 |  0 | Luma         | BT.601 luma of the pixel (greyscale)                                       |
|  1 |  0 |  1 | LFSR         | Frame-locked noise (greyscale)                                             |
|  1 |  1 |  0 | PRNG         | Free-running noise (greyscale)                                             |
|  1 |  1 |  1 | Passthrough  | Original pixel R, G, B (colour)                                            |

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

With S1 in Normal mode, each channel gates independently. A channel outputs its original value if the pixel is in-window; otherwise it outputs black (0). S2, S3, and S4 have no effect in Normal mode.

## Matte Mode

With S1 in Matte mode, the three per-channel gates are OR-combined into a single matte gate: if any channel is in-window, the pixel passes. Pixels where no channel is in-window output black.

For pixels that pass the matte gate, S2/S3/S4 select the output value. The computation uses *masked values* — a channel's original value if its gate passed, or 0 if it did not:

**Logical OR** — white (1023) for any pixel that passed the OR gate.

**Bitwise OR** — bitwise OR of the three masked channel values, producing a greyscale result.

**Logical AND** — white (1023) if all three masked values are non-zero (i.e. all channels passed their individual gate), else black.

**Bitwise AND** — bitwise AND of the three masked channel values, producing a greyscale result.

**Luma** — BT.601 luma (`Y = (77R + 150G + 29B) >> 8`) computed from the full unmasked pixel, output as greyscale. Coefficients sum to 256, giving exact 1023 at R=G=B=1023.

**LFSR** — a 10-bit frame-locked noise value (Fibonacci LFSR, polynomial x¹⁰ + x⁷ + 1, reseeded each frame), output as greyscale.

**PRNG** — the same polynomial as LFSR but never reseeded. The noise pattern shifts by the number of active pixels each frame, producing a different phase every frame.

**Passthrough** — the original pixel R, G, B is output directly on all three channels. The window checks are still performed, but the output is always the original colour.

## Fine Mode

With Fine **On**, the current knob positions are latched as reference values at the moment of switching. Each knob then controls its threshold as `(knob + 7 × reference) / 8`, giving one-eighth the normal sensitivity and allowing very precise adjustment anywhere in the 0–1023 range. Switching back to Fine Off restores full-range control immediately.

## Typical Use

1. Set a matte mode (Logical AND, S2=0 S3=1 S4=0, is the default).
2. Switch to Matte mode (S1=1) and adjust each channel's Low/High knobs until the target region passes the gate.
3. Switch to Normal mode (S1=0) to confirm per-channel gating, then back to Matte for the combined output.
4. Use the Global Blend slider to fade between the original and keyed result.
5. Use Fine mode for precise threshold adjustment once the range is roughly set.

## Technical Notes

- **Colour space:** Operates on RGB (full BT.601 YUV↔RGB conversion via BRAM LUTs)
- **Pipeline latency:** 14 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 11 block RAMs (same BT.601 LUT tables as RGB Bit Crush / RGB Bit Rotator)
- **Key inversion:** Handled inside the `window_check` function — no extra hardware

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
