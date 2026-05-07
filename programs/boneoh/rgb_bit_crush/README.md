# RGB Bit Crush

A per-channel bit-depth reduction video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

This effect program is designed with a layout that resembles a simple audio mixer.
There are three vertical channels, one each for Red, Green, and Blue.

Knobs 1, 2, 3 control the 10-bit mask that determines the amount of bit crush.
Knobs 4, 5, 6 control the wet/dry amount of video that is sent to the master output bus.
The Slider is the master wet/dry fade.


Bit crushing quantises each colour channel to a coarser set of values, producing a posterised, stepped look — the digital equivalent of running video through a low-resolution DAC. Because this operates in RGB space (with full YUV↔RGB colour conversion), the effect is applied independently to red, green, and blue, giving rich colour fringing and cross-channel contrast that differs from the YUV version.

The three column knobs set the crush amount per channel, selecting one of eight quantisation step sizes. Blend knobs and the global slider let you fade between the crushed and original image at each stage. Per-channel rounding switches choose between floor (truncate) and round-to-nearest, which changes whether the crushed levels sit at the bottom or centre of each quantisation step. The Invert switch bitwise-NOTs all three channels after crushing, producing a negative-crushed image.

## Controls

### Knobs

| Knob  |  Control     | Function
|-------| ------------ |----------
|  1    | Red Crush    | Quantization step for the R channel (knob minimum = step 2, maximum = step 96)
|  2    | Green Crush  | Quantization step for the G channel
|  3    | Blue Crush   | Quantization step for the B channel
|  4    | Red Blend    | Wet/dry blend for R (0% = original, 100% = fully crushed)
|  5    | Green Blend  | Wet/dry blend for G
|  6    | Blue Blend   | Wet/dry blend for B

### Switches

  Switch    Operation
 --------- -----------

  S1		Invert        0   = normal output; 1  = bitwise-NOT all channels after crushing

  S2		Red Round     0   = truncate (floor to lower quantization step); 1  = round to nearest

  S3		Green Round   0   = truncate; 1  = round to nearest

  S4		Blue Round    0   = truncate; 1  = round to nearest

  S5		Bypass        Pass the input signal through unprocessed


### Slider

| Control      | Function
|--------------|----------
| Global Blend | Overall wet/dry blend after per-channel blending (0% = original, 100% = processed)

## Crush Step Reference

The knob is divided into 8 equal bands of 128 counts each (step_idx = knob / 128):

| Knob range | Step size | Output levels | Character
|------------|-----------|---------------|----------
| 0–127      | 2         | 512           | very subtle (knob minimum)
| 128–255    | 4         | 256           | subtle
| 256–383    | 8         | 128           | mild
| 384–511    | 16        | 64            | noticeable posterisation
| 512–639    | 32        | 32            | strong posterisation
| 640–767    | 48        | ~21           | heavy
| 768–895    | 64        | 16            | very heavy
| 896–1023   | 96        | ~11           | extreme (knob maximum)

## Technical Notes

- **Colour space:** Operates on RGB (full BT.601 YUV↔RGB conversion via BRAM LUTs)
- **Pipeline latency:** 16 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six variants meet 74.25 MHz (worst case 76.23 MHz)
- **LC utilisation:** 5945–5963 of 7680 (~78%)
- **Package size:** 422,964 bytes (rev_b, unsigned)
- **Last built:** 2026-04-25 12:55 MDT
- **BRAM usage:** 11 block RAMs (same LUT tables as RGB Bit Rotator)
- **Rounding:** Round-to-nearest adds step/2 before quantising with overflow saturation to the largest representable multiple of the step
- **Steps 48 and 96:** Non-power-of-2; implemented via shift-and-reciprocal (floor(x/48)=floor((x>>4)×171>>9)), no extra BRAMs required

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
