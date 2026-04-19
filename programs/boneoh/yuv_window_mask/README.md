# YUV Window Mask

A per-channel YUV window masking effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Window masking isolates a range of pixel values within a defined lower–upper threshold window. For each channel (Y, U, V) a Low and High knob define the window; pixels whose values fall within that range are considered "in-window." Because the masking operates directly in YUV space (no colour conversion), the thresholds work on luma (Y) and the two chroma-difference channels (U = Cr, V = Cb) as they arrive from the hardware.

Switch 1 selects between two modes:

- **Normal (S1 Off):** Each channel is gated independently. A channel passing its window outputs the original pixel value on that channel; a failing channel outputs 0 (Y) or 512 (U/V — neutral chroma). S2, S3, and S4 are ignored.
- **Matte (S1 On):** A combined greyscale matte is computed from S2/S3/S4 and displayed as Y=matte with U=V=512 (neutral chroma, true monochrome).

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

| S1 | Operation | Description                                                                             |
|----|-----------|-----------------------------------------------------------------------------------------|
|  0 | Normal    | Per-channel independent gating; S2/S3/S4 have no effect                                 |
|  1 | Matte     | Matte computed from S2/S3/S4 and shown as Y=matte, U=V=512 (monochrome)                 |

| S2 | S3 | S4 | Operation    | Description                                                                 |
|----|----|----|--------------|-----------------------------------------------------------------------------|
|  0 |  0 |  0 | Logical OR   | White (Y=1023, U=V=512) if any channel is in-window, else black             |
|  0 |  0 |  1 | Bitwise OR   | Y = OR of masked channel values (original if gate passed, else 0); U=V=512  |
|  0 |  1 |  0 | Logical AND  | White if all channels are in-window, else black (default)                   |
|  0 |  1 |  1 | Bitwise AND  | Y = AND of masked channel values (original if gate passed, else 0); U=V=512 |
|  1 |  0 |  0 | Luma         | Y = Y channel value; U=V=512 (neutral chroma)                               |
|  1 |  0 |  1 | LFSR         | Y = frame-locked noise (OR gated); U=V=512                                  |
|  1 |  1 |  0 | PRNG         | Y = free-running noise (OR gated); U=V=512                                  |
|  1 |  1 |  1 | Passthrough  | Original Y, U, V (colour; only active in Matte mode)                        |

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

Each channel gates independently of the others. A channel whose pixel value falls within its Low–High window outputs the original pixel value on that channel; a channel outside its window outputs 0 for Y or 512 for U/V (neutral chroma). S2, S3, and S4 have no effect in Normal mode.

This means the Y, U, and V outputs can be mixed — for example, a pixel where only Y is in-window will output the original Y value but neutral chroma (U=V=512).

## Matte Mode (S1 On)

The three per-channel window results are combined by S2/S3/S4 into a single matte value, output as Y=matte with U=V=512 (neutral chroma).

**Logical OR** — Y=1023 (white) for any pixel where at least one channel is in-window.

**Bitwise OR** — Y = bitwise OR of the three masked channel values (a channel contributes its original value if in-window, else 0), producing a greyscale luma result.

**Logical AND** — Y=1023 only if all three channels are simultaneously in-window, else Y=0.

**Bitwise AND** — Y = bitwise AND of the three masked channel values, producing a greyscale luma result.

**Luma** — Y = the original Y (luma) value of the pixel, passed directly as the output luma. Gated by logical OR.

**LFSR** — Y = a 10-bit frame-locked noise value (Fibonacci LFSR, polynomial x¹⁰ + x⁷ + 1, reseeded each frame). Gated by logical OR — noise appears only where at least one channel is in-window.

**PRNG** — Y = free-running noise using the same polynomial as LFSR but never reseeded. The noise pattern shifts by the number of active pixels each frame, producing a different phase every frame. Gated by logical OR.

**Passthrough** — the original Y, U, V pixel is output directly. Window checks are still performed but the output is always the original colour.

## Using with a Keyer

The Window Mask output can be used as input to an external keyer, such as the LZX FKG3 module. This can be done using either Matte (greyscale) or Normal (colour) modes.

The simplest approach uses a greyscale Matte. Set the FKG3 Key Source switch (S2) to External Key, then patch the monochrome Matte output to FKG3 input J5. J5 is the external key source for the Red channel and is normalled to the Green and Blue channels, so a single patch cable keys all three. This lets the Window Mask define the key shape while the FKG3 handles the mix and fill. Typical use would set the FKG3 S3 switch (mode) to the Luma position.

More complex keying can be performed using the Normal output. Set the FKG3 Key Source switch (S2) to External Key, then patch the Red, Green, and Blue outputs to FKG3 inputs J5, J9, and J13. These are the external key sources for the three colour channels. This lets the Window Mask define the key shape separately for each colour channel while the FKG3 handles the mix and fill. Experiment with the FKG3 S3 switch to find which mode gives the best results.

## Fine Mode

With Fine **On**, the current knob positions are latched as reference values at the moment of switching. Each knob then controls its threshold as `(knob + 7 × reference) / 8`, giving one-eighth the normal sensitivity and allowing very precise adjustment anywhere in the 0–1023 range. Switching back to Fine Off restores full-range control immediately.

## Typical Use

1. Switch to Matte mode (S1=1) with Logical OR (S2=0 S3=1 S4=0, the default).
2. Adjust each channel's Low/High knobs until the region of interest appears white in the matte.
3. Switch to Normal mode (S1=0) — each knob pair now independently gates its named YUV channel.
4. Use Fine mode for precise threshold adjustment once the range is roughly set.
5. Use the Global Blend slider to fade between the original and masked result.

## Comparison with RGB Window Mask

| Feature              | YUV Window Mask              | RGB Window Mask                     |
|----------------------|------------------------------|-------------------------------------|
| Colour space         | YUV direct                   | RGB (full BT.601 YUV↔RGB conv.)     |
| Pipeline latency     | 7 clocks                     | 14 clocks                           |
| BRAM usage           | 0 block RAMs                 | 11 block RAMs                       |
| Mode 100 matte       | Y value (luma passthrough)   | BT.601 computed luma                |
| Best for             | Luma/chroma ranges, fast     | Isolating specific RGB colours      |

Use YUV Window Mask when your threshold decisions map naturally onto luma or chroma values (e.g., isolating a bright sky, removing a specific hue cast). Use RGB Window Mask when the region of interest is best defined in terms of red, green, and blue levels.

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
