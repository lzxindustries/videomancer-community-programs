# RGB Bit Rotator

A per-channel bit rotation video effect for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

This is a flexible and powerful effect that provides a lot of creative control.

This effect program is designed with a layout that resembles a simple audio mixer.
There are three vertical channels, one each for Red, Green, and Blue. 

Knobs 1, 2, 3 control the number of bits used in the processing.
Knobs 4, 5, 6 control the wet/dry amount of video that is sent to the master output bus.
The Slider is the master wet/dry fade.

The main processing allows for bit rotation of each color channel using knobs 1, 2, and 3.
The shifts are performed using a wrap-around and can be set independently for the Red, Green, and Blue channels.
The knobs for rotation will not perform a shift when rotated either fully clockwise or fully counter clockwise.

The knobs 4, 5, and 6 perform a blend or wet/dry mix with no blend at the fully counter clockwise position.

Switches 8, 9, and 10 control the processing of bit depth. See the table below in the Switches section.

Performs circular bit rotation (Left/Right) independently on the Red, Green, and Blue channels of the video signal. 
The input YUV signal is converted to RGB, the rotation is applied, and the result is converted back to YUV. 
Reducing the bit depth before rotation creates lo-fi posterisation effects. 
Per-channel and global wet/dry blend controls allow the effect to be mixed with the original signal.

## Controls

### Knobs

| Control        | Function 
|----------------|--------- 
| Red Rotation   | Rotation amount for the Red channel (0–10 bits; 0 and 10 = no effect; default 5)  
| Green Rotation | Rotation amount for the Green channel  
| Blue Rotation  | Rotation amount for the Blue channel  
| Red Blend      | Wet/dry blend for Red (0% = original, 100% = fully rotated)  
| Green Blend    | Wet/dry blend for Green  
| Blue Blend     | Wet/dry blend for Blue  

### Switches

| Switch    | Function   |  Description
|-----------|------------|--------------
|   S1      | Direction  | Left (rotate left) or Right (rotate right) — applies to all channels 
|   S2      | Depth S2   | Bit depth selector, bit 0 
|   S3      | Depth S3   | Bit depth selector, bit 1 
|   S4      | Depth S4   | Bit depth selector, bit 2 
|   S5      | Bypass     | Pass the input signal through unprocessed 

**Bit depth combinations (S2 S3 S4):**

  S2     S3    S4    Depth     Description 
 -----  ----- ----- --------  -------------

  0      0      0     10-bit   Full quality (no masking) 
  
  0      0      1      8-bit   Mild crunch 
  
  0      1      0      6-bit   Lo-fi 
  
  0      1      1      5-bit   Medium lo-fi 
  
  1      0      0      4-bit   Heavy crunch 
  
  1      0      1      3-bit   Very aggressive 
   
  1      1      0      2-bit   Extreme 
  
  1      1      1      1-bit   Pure posterise 

### Slider

| Control      | Function
|--------------|---------
| Global Blend | Overall wet/dry blend after per-channel blending 
					(0% = original, 100% = processed)

## Technical Notes

- **Colour space:** Input YUV444 → RGB → process → RGB → YUV444 output
- **Colour conversion:** BT.601 full-range coefficients implemented as 11 pre-computed BRAM lookup tables (31 of 32 ice40 hx4k BRAM blocks)
- **Pipeline latency:** 14 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six variants meet 74.25 MHz (worst case 77.44 MHz)
- **LC utilisation:** 6019–6062 of 7680 (~79%)
- **Package size:** 424,387 bytes (rev_b, unsigned)
- **Last built:** 2026-04-25 12:55 MDT

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
