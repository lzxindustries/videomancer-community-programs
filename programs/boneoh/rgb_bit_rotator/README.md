# RGB Bit Rotator

A per-channel bit rotation video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Performs circular bit rotation (Left/Right) independently on the Red, Green, and Blue channels of the video signal. The input YUV signal is converted to RGB, the rotation is applied, and the result is converted back to YUV. Per-channel and global wet/dry blend controls allow the effect to be mixed with the original signal.

A dark suppress (high-pass) threshold is set by switches S2/S3/S4. Channels at or below the threshold are held at black rather than being rotated — this prevents the characteristic repeating pattern that appears in near-black regions when small pixel values have their low-order bits rotated into high-significance positions. Values above the threshold are rotated at full 10-bit quality with no bit reduction. With all three switches off the full range is rotated.

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

| Knob | Operation      | Description                                                                   |
|------|----------------|-------------------------------------------------------------------------------|
|  1   | Red Rotation   | Rotation amount for the Red channel (0–10 bits; 0 and 10 = no effect)        |
|  2   | Green Rotation | Rotation amount for the Green channel                                         |
|  3   | Blue Rotation  | Rotation amount for the Blue channel                                          |
|  4   | Red Blend      | Wet/dry blend for Red (0% = original, 100% = fully rotated)                  |
|  5   | Green Blend    | Wet/dry blend for Green                                                       |
|  6   | Blue Blend     | Wet/dry blend for Blue                                                        |

### Switches

| S1 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Left      | Rotate bits left — applies to all channels             |
|  1 | Right     | Rotate bits right — applies to all channels            |

| S2 | S3 | S4 | Suppresses | Description                                    |
|----|----|----|------------|------------------------------------------------|
|  0 |  0 |  0 | nothing    | All pass — full range rotated                  |
|  0 |  0 |  1 | 0–3        | Slight — removes noise floor only              |
|  0 |  1 |  0 | 0–15       | Mild                                           |
|  0 |  1 |  1 | 0–31       | Moderate                                       |
|  1 |  0 |  0 | 0–63       | Strong                                         |
|  1 |  0 |  1 | 0–127      | Heavy                                          |
|  1 |  1 |  0 | 0–255      | Very heavy                                     |
|  1 |  1 |  1 | 0–511      | Extreme — only values above 50% rotate         |

| S5 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Process   | Apply effect                                           |
|  1 | Bypass    | Pass the input signal through unprocessed              |

### Slider

| Slider | Operation    | Description                                                                 |
|--------|--------------|-----------------------------------------------------------------------------|
|  12    | Global Blend | Overall wet/dry blend after per-channel blending (0% = original, 100% = processed) |

## Technical Notes

- **Colour space:** Input YUV444 → RGB → process → RGB → YUV444 output
- **Colour conversion:** BT.601 full-range coefficients implemented as 11 pre-computed BRAM lookup tables (31 of 32 iCE40 HX4K BRAM blocks)
- **Pipeline latency:** 14 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six HD variants meet 74.25 MHz (worst case ~76.22 MHz)
- **LC utilisation:** 6083–6089 of 7680 (~79%)

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
