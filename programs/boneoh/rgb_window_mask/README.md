# RGB Window Mask

A per-channel RGB window masking effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Window masking isolates a range of pixel values within a defined lower–upper threshold window. For each channel (R, G, B) a Low and High knob define the window; pixels whose values fall within that range are considered "in-window." Because the masking operates in RGB space (with full BT.601 YUV↔RGB conversion via BRAM LUTs), the thresholds work directly on red, green, and blue values.

Switch 1 selects between two modes:

- **Normal (S1 Off):** Each channel is gated independently. A channel passing its window outputs the original pixel value on that channel; a failing channel outputs black (0). S2, S3, and S4 are ignored. This is the primary use mode.
- **Matte (S1 On):** A combined greyscale matte is computed from S2/S3/S4 and displayed on all three channels (R=G=B → true monochrome after downstream YUV conversion). Use this mode to preview and tune the matte before switching to Normal.

## Controls

### Knobs

| Knob | Operation | Description                                                         |
|------|-----------|---------------------------------------------------------------------|
|  1   | R Low     | Lower threshold for the R (red) channel (0% = black, 100% = white) |
|  2   | G Low     | Lower threshold for the G (green) channel                           |
|  3   | B Low     | Lower threshold for the B (blue) channel                            |
|  4   | R High    | Upper threshold for the R channel                                   |
|  5   | G High    | Upper threshold for the G channel                                   |
|  6   | B High    | Upper threshold for the B channel                                   |

### Switches

| S1 | Operation | Description                                                                             |
|----|-----------|-----------------------------------------------------------------------------------------|
|  0 | Normal    | Per-channel independent gating; S2/S3/S4 have no effect                                 |
|  1 | Matte     | Matte computed from S2/S3/S4 and shown as greyscale on all channels                     |

| S2 | S3 | S4 | Operation    | Description                                                                |
|----|----|----|--------------|----------------------------------------------------------------------------|
|  0 |  0 |  0 | Logical OR   | White (1023) if any channel is in-window, else black                       |
|  0 |  0 |  1 | Bitwise OR   | OR of masked channel values (original value if gate passed, else 0)        |
|  0 |  1 |  0 | Logical AND  | White (1023) if all channels are in-window, else black (default)           |
|  0 |  1 |  1 | Bitwise AND  | AND of masked channel values (original value if gate passed, else 0)       |
|  1 |  0 |  0 | Luma         | BT.601 luma of the pixel (greyscale)                                       |
|  1 |  0 |  1 | LFSR         | Frame-locked noise (greyscale)                                             |
|  1 |  1 |  0 | PRNG         | Free-running noise (greyscale)                                             |
|  1 |  1 |  1 | Passthrough  | Original pixel R, G, B (colour; only active in Matte mode)                 |

| S5 | Operation | Description                                         |
|----|-----------|-----------------------------------------------------|
|  0 | Normal    | Full-range knob sensitivity                         |
|  1 | Fine      | 1/8 sensitivity for precise threshold adjustment    |

### Slider

| Slider | Operation    | Description                                                                       |
|--------|--------------|-----------------------------------------------------------------------------------|
|  12    | Global Blend | Wet/dry blend between the original and masked signal (0% = original, 100% = masked) |

## Window Inversion

When a channel's Low knob is set **above** its High knob, the window inverts automatically for that channel: pixels outside the normal (Low, High) range pass, and pixels inside are blocked. Each channel inverts independently — no extra switch is needed.

## Normal Mode (S1 Off)

Each channel gates independently of the others. A channel whose pixel value falls within its Low–High window outputs the original pixel value on that channel; a channel outside its window outputs black (0). S2, S3, and S4 have no effect in Normal mode.

This means the R, G, and B outputs can be mixed — for example, a pixel where only R is in-window will output the original R value but 0 for G and B.

## Matte Mode (S1 On)

The three per-channel window results are combined by S2/S3/S4 into a single greyscale matte, output identically on all three channels (R=G=B). The downstream YUV conversion produces a true monochrome signal. Use this mode to visualise exactly what the mask is doing before switching to Normal mode.

**Logical OR** — white (1023) for any pixel where at least one channel is in-window.

**Bitwise OR** — bitwise OR of the three masked channel values (a channel contributes its original value if in-window, else 0), producing a greyscale result.

**Logical AND** — white (1023) only if all three channels are simultaneously in-window, else black.

**Bitwise AND** — bitwise AND of the three masked channel values, producing a greyscale result.

**Luma** — BT.601 luma (`Y = (77R + 150G + 29B) >> 8`) computed from the full unmasked pixel, gated by logical AND.

**LFSR** — a 10-bit frame-locked noise value (Fibonacci LFSR, polynomial x¹⁰ + x⁷ + 1, reseeded each frame), gated by logical OR.

**PRNG** — the same polynomial as LFSR but never reseeded. The noise pattern shifts by the number of active pixels each frame, producing a different phase every frame. Gated by logical OR.

**Passthrough** — the original pixel R, G, B is output directly on all three channels. Window checks are still performed but the output is always the original colour.

## Fine Mode

With Fine **On**, the current knob positions are latched as reference values at the moment of switching. Each knob then controls its threshold as `(knob + 7 × reference) / 8`, giving one-eighth the normal sensitivity and allowing very precise adjustment anywhere in the 0–1023 range. Switching back to Fine Off restores full-range control immediately.

## Typical Use

1. Switch to Matte mode (S1=1) with Logical AND (S2=0 S3=1 S4=0, the default).
2. Adjust each channel's Low/High knobs until the region of interest appears white in the matte.
3. Switch to Normal mode (S1=0) — each knob pair now independently gates its named colour channel.
4. Use Fine mode for precise threshold adjustment once the range is roughly set.
5. Use the Global Blend slider to fade between the original and masked result.

## Technical Notes

- **Colour space:** Operates on RGB (full BT.601 YUV↔RGB conversion via BRAM LUTs)
- **Pipeline latency:** 14 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 11 block RAMs (same BT.601 LUT tables as RGB Bit Crush / RGB Bit Rotator)
- **Window inversion:** Handled inside the `window_check` function — no extra hardware

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
