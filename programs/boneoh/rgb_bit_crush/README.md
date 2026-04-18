# RGB Bit Crush

A per-channel bit-depth reduction video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

Bit crushing quantises each colour channel to a coarser set of values, producing a posterised, stepped look — the digital equivalent of running video through a low-resolution DAC. Because this operates in RGB space (with full YUV↔RGB colour conversion), the effect is applied independently to red, green, and blue, giving rich colour fringing and cross-channel contrast that differs from the YUV version.

The three column knobs set the crush amount per channel, selecting one of eight quantisation step sizes. Blend knobs and the global slider let you fade between the crushed and original image at each stage. Per-channel rounding switches choose between floor (truncate) and round-to-nearest, which changes whether the crushed levels sit at the bottom or centre of each quantisation step. The Invert switch bitwise-NOTs all three channels after crushing, producing a negative-crushed image.


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

| Knob | Operation    | Description                                                                      |
|------|--------------|----------------------------------------------------------------------------------|
|  1   | Red Crush    | Quantisation step for the R channel (knob minimum = step 2, maximum = step 96)  |
|  2   | Green Crush  | Quantisation step for the G channel                                              |
|  3   | Blue Crush   | Quantisation step for the B channel                                              |
|  4   | Red Blend    | Wet/dry blend for R (0% = original, 100% = fully crushed)                        |
|  5   | Green Blend  | Wet/dry blend for G                                                              |
|  6   | Blue Blend   | Wet/dry blend for B                                                              |

### Switches

| S1 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Normal    | Normal output                                          |
|  1 | Invert    | Bitwise-NOT all channels after crushing                |

| S2 | Operation  | Description                                            |
|----|------------|--------------------------------------------------------|
|  0 | Truncate   | Floor to lower quantisation step (Red channel)         |
|  1 | Round      | Round to nearest quantisation step (Red channel)       |

| S3 | Operation  | Description                                            |
|----|------------|--------------------------------------------------------|
|  0 | Truncate   | Floor to lower quantisation step (Green channel)       |
|  1 | Round      | Round to nearest quantisation step (Green channel)     |

| S4 | Operation  | Description                                            |
|----|------------|--------------------------------------------------------|
|  0 | Truncate   | Floor to lower quantisation step (Blue channel)        |
|  1 | Round      | Round to nearest quantisation step (Blue channel)      |

| S5 | Operation | Description                                            |
|----|-----------|--------------------------------------------------------|
|  0 | Process   | Apply effect                                           |
|  1 | Bypass    | Pass the input signal through unprocessed              |

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

- **Colour space:** Operates on RGB (full BT.601 YUV↔RGB conversion via BRAM LUTs)
- **Pipeline latency:** 16 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 11 block RAMs (same LUT tables as RGB Bit Rotator)
- **Rounding:** Round-to-nearest adds step/2 before quantising with overflow saturation to the largest representable multiple of the step
- **Steps 48 and 96:** Non-power-of-2; implemented via shift-and-reciprocal (floor(x/48)=floor((x>>4)×171>>9)), no extra BRAMs required

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
