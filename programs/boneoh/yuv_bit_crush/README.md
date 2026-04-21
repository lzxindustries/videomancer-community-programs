# YUV Bit Crush

A per-channel bit-depth reduction video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Bit crushing quantises each channel to a coarser set of values, producing a stepped, posterised look. This version operates directly in YUV space — no colour conversion — so luminance and chroma are crushed independently.

All three channels (Y, U, V) are crushed using their respective knobs, each selecting one of eight quantisation step sizes. Y is always truncated (floor). U and V support optional round-to-nearest switching and optional RPDF dither from the onboard LFSRs, which breaks up the hard quantisation edges at the cost of added grain.

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

| Knob | Operation | Description                                                                                       |
|------|-----------|---------------------------------------------------------------------------------------------------|
|  1   | Y Crush   | Quantisation step for Y (luminance) — always truncated (knob minimum = step 2, maximum = step 96) |
|  2   | U Crush   | Quantisation step for the U (blue-yellow chroma) channel                                          |
|  3   | V Crush   | Quantisation step for the V (red-cyan chroma) channel                                             |
|  4   | Y Blend   | Wet/dry blend for Y (0% = original, 100% = fully crushed)                                         |
|  5   | U Blend   | Wet/dry blend for U                                                                               |
|  6   | V Blend   | Wet/dry blend for V                                                                               |

### Switches

| S1 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Normal    | Normal output                                          |
|  1 | Invert    | Bitwise-NOT all three channels after processing        |

| S2 | Operation | Description                                                          |
|----|-----------|----------------------------------------------------------------------|
|  0 | Off       | No dither                                                            |
|  1 | Dither    | Add RPDF pseudorandom noise before crushing U and V (Y unaffected)   |

| S3 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Truncate  | Floor to lower quantisation step (U channel)           |
|  1 | Round     | Round to nearest quantisation step (U channel)         |

| S4 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Truncate  | Floor to lower quantisation step (V channel)           |
|  1 | Round     | Round to nearest quantisation step (V channel)         |

| S5 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Process   | Apply effect                                           |
|  1 | Bypass    | Pass the input signal through unprocessed              |

**Note:** Round takes priority over Dither. When both U Round and Dither are on, rounding is applied (no dither) for the U channel.

### Slider

| Slider | Operation    | Description                                                                 |
|--------|--------------|-----------------------------------------------------------------------------|
|  12    | Global Blend | Overall wet/dry blend after per-channel blending (0% = original, 100% = processed) |

## Crush Step Reference

The knob is divided into 8 equal bands of 128 counts each (step_idx = knob / 128):

| Knob range | Step size | Output levels | Character                    |
|------------|-----------|---------------|------------------------------|
| 0–127      | 2         | 512           | very subtle (knob minimum)   |
| 128–255    | 4         | 256           | subtle                       |
| 256–383    | 8         | 128           | mild                         |
| 384–511    | 16        | 64            | noticeable posterisation     |
| 512–639    | 32        | 32            | strong posterisation         |
| 640–767    | 48        | ~21           | heavy                        |
| 768–895    | 64        | 16            | very heavy                   |
| 896–1023   | 96        | ~11           | extreme (knob maximum)       |

## Technical Notes

- **Colour space:** Operates directly on YUV444 — no colour conversion
- **Pipeline latency:** 10 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 0 block RAMs (no colour conversion)
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six HD variants meet 74.25 MHz (worst case ~76 MHz)
- **LC utilisation:** 4814–4857 of 7680 (~63%)
- **Dither sources:** U uses lfsr16[9:0]; V uses lfsr10 (both free-running)
- **Rounding:** Round-to-nearest adds step/2 before quantising with overflow saturation to the largest representable multiple of the step
- **Steps 48 and 96:** Non-power-of-2; implemented via shift-and-reciprocal, no extra BRAMs required
- **Dither gating:** For steps 48 and 96, dither is masked to 31 and 63 respectively (largest 2^k−1 below the step)

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
