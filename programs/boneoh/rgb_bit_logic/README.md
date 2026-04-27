# RGB Bit Logic

A per-channel bitwise logic video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

This effect program is designed with a layout that resembles a simple audio mixer.
There are three vertical channels, one each for Red, Green, and Blue.

Knobs 1, 2, 3 control the 10-bit mask that determines which bit planes are affected by the logic operation.
Knobs 4, 5, 6 control the wet/dry amount of video that is sent to the master output bus.
The Slider is the master wet/dry fade.

The main processing applies a bitwise logic operation between each colour channel and its mask. The operation is selected using Switches 2, 3, and 4 as a 3-bit encoded value. Six deterministic operations are available (AND, OR, XOR, NAND, NOR, NXOR), plus two random modes driven by onboard LFSRs. In random modes the mask knobs gate which bit planes the random value can reach.

Switch 1 has a dual role: for deterministic operations (AND–NXOR) it inverts all three masks before applying them; for the LFSR mode it selects whether the LFSR is reseeded from the PRNG at each vsync or runs freely from power-on.

The input YUV signal is converted to RGB, the logic operation is applied, and the result is converted back to YUV. Per-channel and global wet/dry blend controls allow the effect to be mixed with the original signal.

## Controls

### Knobs

| Knob  |  Control     | Function
|-------| ------------ |----------
|   1   |  Red Mask    | 10-bit mask for the Red channel (0 = no bits affected, 1023 = all bits)
|   2   |  Green Mask  | 10-bit mask for the Green channel
|   3   |  Blue Mask   | 10-bit mask for the Blue channel
|   4   |  Red Blend   | Wet/dry blend for Red (0% = original, 100% = fully processed)
|   5   |  Green Blend | Wet/dry blend for Green
|   6   |  Blue Blend  | Wet/dry blend for Blue

### Switches

| Switch      | Function     | Description
|-------------|----------    | ------------ 
|   S1		  | Invert/Seed  | Ops 0–5: Off = normal mask, On = invert all masks before applying; Op 6 (LFSR): Off = reseed from PRNG at vsync (sync), On = free-run; Op 7 (PRNG): no effect
|   S2 		  | Op S2        | Operator select bit 2 (MSB)
|   S3        | Op S3        | Operator select bit 1
|   S4        | Op S4        | Operator select bit 0 (LSB)
|   S5        | Bypass       | Pass the input signal through unprocessed

**Operator encoding (S2 S3 S4):**

  S2    S3    S4    Operation   Description
 ----- ----- ----- ----------- -------------

  0     0     0     AND         pixel AND mask

  0     0     1     OR          pixel OR mask

  0     1     0     XOR         pixel XOR mask

  0     1     1     NAND        NOT (pixel AND mask)

  1     0     0     NOR         NOT (pixel OR mask)

  1     0     1     NXOR        NOT (pixel XOR mask)

  1     1     0     LFSR        pixel XOR (10-bit LFSR AND mask)

  1     1     1     PRNG        pixel XOR (16-bit LFSR AND mask)


### Slider

| Control      | Function
|--------------|----------
| Global Blend | Overall wet/dry blend after per-channel blending (0% = original, 100% = processed)

## Technical Notes

- **Colour space:** Input YUV444 → RGB → process → RGB → YUV444 output
- **Colour conversion:** BT.601 full-range coefficients implemented as 11 pre-computed BRAM lookup tables (31 of 32 ice40 hx4k BRAM blocks)
- **Pipeline latency:** 16 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six variants meet 74.25 MHz (worst case 80.66 MHz)
- **LC utilisation:** 5721–5733 of 7680 (~75%)
- **Package size:** 413,278 bytes (rev_b, unsigned)
- **Last built:** 2026-04-27

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
