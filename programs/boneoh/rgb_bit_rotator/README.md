# RGB Bit Rotator

A per-channel bit rotation video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

The main processing allows for bit rotation of each colour channel using knobs 1, 2, and 3.
The shifts are performed using a wrap-around and can be set independently for the Red, Green, and Blue channels.
The knobs for rotation will not perform a shift when rotated either fully clockwise or fully counter-clockwise.

Performs circular bit rotation (Left/Right) independently on the Red, Green, and Blue channels of the video signal.
The input YUV signal is converted to RGB, the rotation is applied, and the result is converted back to YUV.
Reducing the bit depth before rotation creates lo-fi posterisation effects.
Per-channel and global wet/dry blend controls allow the effect to be mixed with the original signal.

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

| S2 | S3 | S4 | Operation | Description               |
|----|----|----|-----------|---------------------------|
|  0 |  0 |  0 | 10-bit    | Full quality (no masking) |
|  0 |  0 |  1 |  8-bit    | Mild crunch               |
|  0 |  1 |  0 |  6-bit    | Lo-fi                     |
|  0 |  1 |  1 |  5-bit    | Medium lo-fi              |
|  1 |  0 |  0 |  4-bit    | Heavy crunch              |
|  1 |  0 |  1 |  3-bit    | Very aggressive           |
|  1 |  1 |  0 |  2-bit    | Extreme                   |
|  1 |  1 |  1 |  1-bit    | Pure posterise            |

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
- **HD timing:** All six variants meet 74.25 MHz (worst case ~81 MHz)
- **LC utilisation:** ~25% (routing-friendly)

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
