# YUV Bit Crush

A per-channel bit-depth reduction video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

This effect program is designed with a layout that resembles a simple audio mixer.
There are three vertical channels, one each for Red, Green, and Blue.

Knobs 1, 2, 3 control the 10-bit mask that determines the amount of bit crush.
Knobs 4, 5, 6 control the wet/dry amount of video that is sent to the master output bus.
The Slider is the master wet/dry fade.

Bit crushing quantises each channel to a coarser set of values, producing a stepped, posterised look. This version operates directly in YUV space — no colour conversion — so luminance and chroma are crushed independently.

All three channels (Y, U, V) are crushed using their respective knobs, each selecting one of eight quantisation step sizes. Y is always truncated (floor). U and V support optional round-to-nearest switching and optional RPDF dither from the onboard LFSRs, which breaks up the hard quantisation edges at the cost of added grain.

## Controls

### Knobs

|  Knob  | Function | Description
|--------|----------| -------------------
|    1   | Y Crush  | Quantisation step for Y (luminance) — always truncated (knob min = step 2, max = step 96)
|    2   | U Crush  | Quantisation step for the U (blue-yellow chroma) channel
|    3   | V Crush  | Quantisation step for the V (red-cyan chroma) channel
|    4   | Y Blend  | Wet/dry blend for Y (0% = original, 100% = fully crushed)
|    5   | U Blend  | Wet/dry blend for U (0% = original, 100% = fully crushed)
|    6   | V Blend  | Wet/dry blend for V

### Switches

  Switch    Description
 --------- ------------

  S1		Invert    0   = normal output; 1  = bitwise-NOT all three channels after processing

  S2		Dither    0   = no dither; 1  = add RPDF pseudorandom noise before crushing U and V (Y unaffected)

  S3		U Round   0   = truncate (floor to lower quantization step); 1  = round to nearest

  S4		V Round   0   = truncate; 1  = round to nearest

  S5		Bypass    Pass the input signal through unprocessed


**Note:** Round takes priority over Dither. When both U Round and Dither are On, rounding is applied (no dither) for the U channel.

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

- **Colour space:** Operates directly on YUV444 — no colour conversion
- **Pipeline latency:** 10 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six variants meet 74.25 MHz (worst case 74.66 MHz)
- **LC utilisation:** 4836–4851 of 7680 (~63%)
- **Package size:** 310,436 bytes (rev_b, unsigned)
- **Last built:** 2026-04-25 12:55 MDT
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
