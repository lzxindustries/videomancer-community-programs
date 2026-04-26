# YUV Bit Logic

A per-channel bitwise logic video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

This effect program is designed with a layout that resembles a simple audio mixer.
There are three vertical channels, one each for Y (luminance), U (blue-yellow chroma), and V (red-cyan chroma).

Knobs 1, 2, 3 control the 10-bit mask that determines which bit planes are affected by the logic operation.
Knobs 4, 5, 6 control the wet/dry amount of video that is sent to the master output bus.
The Slider is the master wet/dry fade.

The main processing applies a bitwise logic operation between each channel and its mask. The operation is selected using Switches 2, 3, and 4 as a 3-bit encoded value. Six deterministic operations are available (AND, OR, XOR, NAND, NOR, NXOR), plus two random modes driven by onboard LFSRs. In random modes the mask knobs gate which bit planes the random value can reach.

Switch 1 has a dual role: for deterministic operations (AND–NXOR) it inverts all three masks before applying them; for the LFSR mode it selects whether the LFSR is reseeded from the PRNG at each vsync or runs freely from power-on.

Because this operates directly in YUV space — with no colour conversion — the artistic character differs from the RGB version. Logic on the Y channel scrambles luminance bit planes, while logic on U and V scrambles the chroma axes independently. This also makes the build extremely fast with very high Fmax.

## Controls

### Knobs

| Control  | Function
|----------|----------
| Y Mask   | 10-bit mask for the Y (luminance) channel (0 = no bits affected, 1023 = all bits)
| U Mask   | 10-bit mask for the U (blue-yellow chroma) channel
| V Mask   | 10-bit mask for the V (red-cyan chroma) channel
| Y Blend  | Wet/dry blend for Y (0% = original, 100% = fully processed)
| U Blend  | Wet/dry blend for U
| V Blend  | Wet/dry blend for V

### Switches

| Switch      | Function    | Description
|-------------|------------ | -------------------
|   S1        | Invert/Seed | Ops 0–5: Off = normal mask, On = invert all masks before applying; Op 6 (LFSR): Off = reseed from PRNG at vsync (sync), On = free-run; Op 7 (PRNG): no effect
|   S2        | Op S2       | Operator select bit 2 (MSB)
|   S3        | Op S3       | Operator select bit 1
|   S4        | Op S4       | Operator select bit 0 (LSB)
|   S5        | Bypass      | Pass the input signal through unprocessed

**Operator encoding (S2 S3 S4):**

  S2     S3     S4    Operation   Description
 -----  -----  ----- ----------- ------------

  0      0      0     AND         channel AND mask

  0      0      1     OR          channel OR mask

  0      1      0     XOR         channel XOR mask

  0      1      1     NAND        NOT (channel AND mask)

  1      0      0     NOR         NOT (channel OR mask)

  1      0      1     NXOR        NOT (channel XOR mask)

  1      1      0     LFSR        channel XOR (10-bit LFSR AND mask)

  1      1      1     PRNG        channel XOR (16-bit LFSR AND mask)


### Slider

| Control      | Function
|--------------|----------
| Global Blend | Overall wet/dry blend after per-channel blending (0% = original, 100% = processed)

## Technical Notes

- **Colour space:** Operates directly on YUV444 — no colour conversion
- **Pipeline latency:** 10 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six variants meet 74.25 MHz (worst case 75.61 MHz)
- **LC utilisation:** 4815–4854 of 7680 (~63%)
- **Package size:** 312,464 bytes (rev_b, unsigned)
- **Last built:** 2026-04-25 12:55 MDT

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
