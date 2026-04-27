# RGB Band Filter

A per-channel RGB multi-mode filter and matte generation effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Dual Function

This program operates in two complementary modes:

### Primary: Multi-Mode Filtering
Per-channel frequency-domain filtering using four filter types applied independently to R, G, and B channels based on knob settings:
- **Low Pass Filter** — attenuates frequencies above the threshold
- **High Pass Filter** — attenuates frequencies below the threshold
- **Band Pass Filter** — isolates frequencies within a defined lower–upper band
- **Notch Filter** — rejects frequencies within a defined band while preserving adjacent regions

### Secondary: Matte Generation
Window-based matte generation that isolates a range of pixel values within a defined lower–upper threshold window. For each channel (R, G, B) a Low and High knob define the window; pixels whose values fall within that range are considered "in-window." The three per-channel results are then combined into a matte according to the selected Matte Mode, and that matte controls what appears in the output.

Because the processing operates in RGB space (with full BT.601 YUV↔RGB conversion via BRAM LUTs), the thresholds work directly on red, green, and blue values. The same knob architecture serves both filtering and matte generation applications—switch between modes using the Matte Mode selector and Show Matte control.

## Controls

### Knobs

All knobs control per-channel window/threshold parameters, interpreted differently depending on filter type and matte mode:

| Knob | Control    | Function |
|----- |------------|----------|
|  1   | Red Low    | Lower threshold/frequency edge for the R channel (0% = black, 100% = white); filter type determines interpretation |
|  2   | Green Low  | Lower threshold/frequency edge for the G channel |
|  3   | Blue Low   | Lower threshold/frequency edge for the B channel |
|  4   | Red High   | Upper threshold/frequency edge for the R channel |
|  5   | Green High | Upper threshold/frequency edge for the G channel |
|  6   | Blue High  | Upper threshold/frequency edge for the B channel |

### Switches

| Switch | Description  | Off                                 | On                          |
|--------|--------------|-------------------------------------|-----------------------------|
|  S1    |  Show Matte  | Filtered output (original or black) | Greyscale matte preview     |
|  S2    |  Matte Bit 2 | 0 (MSB of matte mode word)          | 1                           |
|  S3    |  Matte Bit 1 | 0 (middle bit)                      | 1                           |
|  S4    |  Matte Bit 0 | 0 (LSB of matte mode word)          | 1                           |
|  S5    |  Fine        | Normal (full-range knobs)           | Fine (1/8-sensitivity)      |

### Matte Modes

Matte Bit 2 (S2), Matte Bit 1 (S3), and Matte Bit 0 (S4) form a 3-bit word (S2 = MSB, S4 = LSB) that selects one of eight matte modes:

  S2    S3    S4   Operation      Description 
 ----  ----  ---- -------------- ------------- 

  0     0     0    Logical OR     Matte = white (1023) if **any** channel is in-window, else black
  
  0     0     1    Bitwise OR     Matte = bitwise OR of in-window channel values; failing channels
                                  contribute 0

  0     1     0    Logical AND    Matte = white (1023) if **all** channels are in-window, else black  

  0     1     1    Bitwise AND    Matte = bitwise AND of in-window channel values; failing channels
                                  contribute 0

  1     0     0    Luma           Matte = BT.601 luma of the full pixel, gated by logical AND  

  1     0     1    LFSR           Matte = frame-locked noise value, gated by logical OR  

  1     1     0    PRNG           Matte = free-running noise value, gated by logical OR  

  1     1     1    Passthrough    Original pixel output on all channels — matte generation disabled  


### Slider

| Control      | Function |
|--------------|----------|
| Global Blend | Wet/dry blend between the original and filtered signal (0% = original, 100% = filtered) |

## Matte Inversion

When a channel's Low knob is set **above** its High knob, the window inverts automatically for that channel: pixels outside the normal (High, Low) gap pass, and pixels inside are blocked. Each channel inverts independently — no extra switch is needed.

## Show Matte

With Show Matte **On**, the computed matte value is output as a greyscale signal on all three channels (R = G = B). Because the three channels are equal, the downstream YUV conversion produces a pure luma signal with neutral chroma — a true monochrome output. Use this to dial in your thresholds visually, then switch Show Matte Off for the actual filtered output.

With Show Matte **Off**, the matte is used as a binary gate: pixels where the matte is greater than zero pass through as the original RGB pixel; pixels where the matte is zero are replaced with black.

The Passthrough mode (S2=1, S3=1, S4=1) always outputs the original pixel on all channels regardless of the Show Matte setting.

## Matte Mode Details

**Logical OR / AND** — produce a pure black-or-white matte. OR passes if any channel is in-window; AND requires all three channels to be in-window simultaneously.

**Bitwise OR / AND** — produce a greyscale matte from the channel values themselves. Failing channels contribute 0 to the bitwise operation, so only in-window values appear.

**Luma** — computes BT.601 luma (`Y = (77R + 150G + 29B) >> 8`) from the full unmasked pixel. The luma value is output as the matte only if all three channels are in-window (logical AND gate); otherwise matte = 0. Coefficients sum to 256, giving exact 1023 at R=G=B=1023.

**LFSR** — the matte is a 10-bit frame-locked noise value (Fibonacci LFSR, polynomial x¹⁰ + x⁷ + 1, reseeded each frame). The noise value is gated by logical OR: it appears only for pixels where at least one channel is in-window.

**PRNG** — the same polynomial as LFSR but never reseeded. The noise pattern shifts by the number of active pixels each frame, producing a different phase every frame. Also gated by logical OR.

**Passthrough** — the original pixel is output directly on all three channels. The window checks and Show Matte setting are both ignored.

## Fine Mode

With Fine **On**, the current knob positions are latched as reference values at the moment of switching. Each knob then controls its threshold as `(knob + 7 × reference) / 8`, giving one-eighth the normal sensitivity and allowing very precise adjustment anywhere in the 0–1023 range. Switching back to Fine Off restores full-range control immediately.

## Using the Matte with FKG3

The matte output can be patched directly into the LZX FKG3 keyer module as the key source. Enable Show Matte (S1 On) to output the matte as a greyscale signal, then patch the output to FKG3 input J5. Set the FKG3 Key Source switch (S2) to External Key — J5 is normalled to the Green and Blue key inputs, so a single cable drives all three channels. Set FKG3 S3 (mode) to Luma for clean black/white matte operation.

For colour-separated matte generation, patch R, G, and B outputs to FKG3 inputs J5, J9, and J13 respectively with Show Matte Off. This uses the per-channel filter gating directly as independent key sources for each colour channel on the FKG3, allowing the Band Filter's window thresholds to control the FKG3 mix separately per colour.

## Typical Use

1. Set a matte mode (Logical AND, S2=0 S3=1 S4=0, is the default).
2. Turn Show Matte On and adjust each channel's Low/High knobs until the target region shows as white in the preview.
3. Turn Show Matte Off to switch to the filtered output.
4. Use the Global Blend slider to fade between the original and filtered result.
5. Use Fine mode for precise threshold adjustment once the range is roughly set.

## Technical Notes

- **Colour space:** Operates on RGB (full BT.601 YUV↔RGB conversion via BRAM LUTs)
- **Pipeline latency:** 14 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 11 block RAMs (BT.601 colour conversion LUTs; 31 / 32 total including SDK)
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six variants meet 74.25 MHz (worst case 81.81 MHz)
- **LC utilisation:** 4473–4492 of 7680 (~58%)
- **Package size:** 355,254 bytes (rev_b, unsigned)
- **Last built:** 2026-04-27
- **Matte inversion:** Handled inside the `window_check` function — no extra hardware

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
