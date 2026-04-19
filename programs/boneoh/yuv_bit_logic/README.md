# YUV Bit Logic

A per-channel bitwise logic video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

The main processing applies a bitwise logic operation between each channel and its mask. The operation is selected using Switches 2, 3, and 4 as a 3-bit encoded value. Six deterministic operations are available (AND, OR, XOR, NAND, NOR, NXOR), plus two random modes driven by onboard LFSRs. In random modes the mask knobs gate which bit planes the random value can reach.

Switch 1 has a dual role: for deterministic operations (AND–NXOR) it inverts all three masks before applying them; for the LFSR mode it selects whether the LFSR is reseeded from the PRNG at each vsync or runs freely from power-on.

Because this operates directly in YUV space — with no colour conversion — the artistic character differs from the RGB version. Logic on the Y channel scrambles luminance bit planes, while logic on U and V scrambles the chroma axes independently. This also makes the build extremely fast with very high Fmax.

## Notes

This and others of my rgb_bit_* and yuv_bit_* programs have a similar user interface, designed like a simple audio mixer.
These are the Bit Crush, Bit Logic, and Bit Rotator programs.

There are three vertical channels for either Red, Green, and Blue or Y, U, and V.

Knobs 1, 2, and 3 control the effect processing.
Knobs 4, 5, and 6 control the amount of wet/dry effect passed to the master bus.

The slider controls the global wet/dry blend to the master output.

This gives a lot of control, from mild to wild!

Once you understand one, the rest are easy.

## Controls

### Knobs

| Knob | Operation | Description                                                                      |
|------|-----------|----------------------------------------------------------------------------------|
|  1   | Y Mask    | 10-bit mask for the Y (luminance) channel (0 = no bits affected, 1023 = all bits) |
|  2   | U Mask    | 10-bit mask for the U (blue-yellow chroma) channel                               |
|  3   | V Mask    | 10-bit mask for the V (red-cyan chroma) channel                                  |
|  4   | Y Blend   | Wet/dry blend for Y (0% = original, 100% = fully processed)                      |
|  5   | U Blend   | Wet/dry blend for U                                                              |
|  6   | V Blend   | Wet/dry blend for V                                                              |

### Switches

| S1 | Operation    | Description                                                                          |
|----|--------------|--------------------------------------------------------------------------------------|
|  0 | Normal/Free  | Ops 0–5: normal mask applied; Op 6 (LFSR): reseed from PRNG at vsync                |
|  1 | Invert/Sync  | Ops 0–5: all masks bitwise-inverted before applying; Op 6 (LFSR): free-run from power-on; Op 7: no effect |

| S2 | S3 | S4 | Operation | Description                          |
|----|----|----|-----------|--------------------------------------|
|  0 |  0 |  0 | AND       | channel AND mask                     |
|  0 |  0 |  1 | OR        | channel OR mask                      |
|  0 |  1 |  0 | XOR       | channel XOR mask                     |
|  0 |  1 |  1 | NAND      | NOT (channel AND mask)               |
|  1 |  0 |  0 | NOR       | NOT (channel OR mask)                |
|  1 |  0 |  1 | NXOR      | NOT (channel XOR mask)               |
|  1 |  1 |  0 | LFSR      | channel XOR (10-bit LFSR AND mask)   |
|  1 |  1 |  1 | PRNG      | channel XOR (16-bit LFSR AND mask)   |

| S5 | Operation | Description                                   |
|----|-----------|-----------------------------------------------|
|  0 | Process   | Apply effect                                  |
|  1 | Bypass    | Pass the input signal through unprocessed     |

### Slider

| Slider | Operation    | Description                                                                 |
|--------|--------------|-----------------------------------------------------------------------------|
|  12    | Global Blend | Overall wet/dry blend after per-channel blending (0% = original, 100% = processed) |

## Technical Notes

- **Colour space:** Operates directly on YUV444 — no colour conversion
- **Pipeline latency:** 10 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **HD timing:** All six variants meet 74.25 MHz with significant headroom
- **LC utilisation:** Low (no BRAM colour conversion tables)

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
