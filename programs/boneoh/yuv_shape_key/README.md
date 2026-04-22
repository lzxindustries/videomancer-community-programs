# YUV Shape Key

A shape-based keyer operating directly in YUV space for the [LZX Industries Videomancer](https://lzxindustries.net/videomancer) FPGA video processor.

## Effect

The Shape Key places a geometric mask over the video frame. Pixels inside the shape pass the original image; pixels outside output black (Y=0, U=V=512 neutral chroma). A global blend slider mixes the keyed output with the original image at any ratio.

Four shapes are available: Rectangle, Ellipse, Diamond, and Cross. Centre position, width, height, and a global scale factor are each knob-controlled. The Inflate/Pinch knob continuously morphs the shape between L1 (diamond-like pinch) and L-inf (rectangle-like inflate), with a midpoint that approximates an ellipse — all without a square root. Dither fill modes stochastically soften the boundary.

Because the keyer operates directly in YUV space (no colour conversion), it is low-latency and uses no block RAMs.

## Controls

### Knobs

| Knob | Operation     | Description                                                                    |
|------|---------------|--------------------------------------------------------------------------------|
|  1   | Centre X      | Horizontal centre of the shape (0% = left edge, 100% = right edge)            |
|  2   | Centre Y      | Vertical centre of the shape (0% = top, 100% = bottom)                        |
|  3   | Width         | Horizontal half-width of the shape (scaled by Knob 5)                         |
|  4   | Height        | Vertical half-height of the shape (scaled by Knob 5)                          |
|  5   | Scale         | Global scale multiplier applied to both width and height                       |
|  6   | Inflate/Pinch | Morphs between L1 (pinch/diamond) and L-inf (inflate/rectangle) norm          |

Knob 6 ranges:

| Knob 6 range | Norm    | Character                               |
|--------------|---------|-----------------------------------------|
| 0–33%        | L1      | Diamond tendency / maximum pinch        |
| 33–67%       | Mid     | Ellipse-like (blend of L1 and L-inf)    |
| 67–100%      | L-inf   | Rectangle tendency / maximum inflate    |

Knob 6 is ignored for the Cross shape. Default positions: Rectangle = high, Ellipse = mid, Diamond = low.

### Switches

| S1 | Operation | Description                                               |
|----|-----------|-----------------------------------------------------------|
|  0 | Normal    | Inside pixels pass; outside pixels output black           |
|  1 | Invert    | Swap inside and outside (outside passes, inside = black)  |

| S2 | S3 | Shape       | Description                                                       |
|----|----|-------------|-------------------------------------------------------------------|
|  0 |  0 | Rectangle   | Axis-aligned bounding box (L-inf norm); Knob 6 = high            |
|  0 |  1 | Ellipse     | Elliptical region (L2 approximation); Knob 6 = mid               |
|  1 |  0 | Diamond     | Rotated rectangle (L1 norm); Knob 6 = low                        |
|  1 |  1 | Cross       | Horizontal + vertical arm union; Knob 6 ignored                   |

| S4 | S5 | Fill         | Description                                                      |
|----|----|--------------|------------------------------------------------------------------|
|  0 |  0 | Solid        | Binary inside/outside edge                                       |
|  0 |  1 | Slight dither | Stochastic boundary — noise scaled ÷4                          |
|  1 |  0 | More dither  | Stochastic boundary — noise scaled ÷2                           |
|  1 |  1 | Max dither   | Stochastic boundary — full LFSR noise                           |

Dither is not applied to the Cross shape; it always uses solid fill.

### Slider

| Slider | Operation    | Description                                                                         |
|--------|--------------|-------------------------------------------------------------------------------------|
|  12    | Global Blend | Wet/dry blend between original and keyed signal (0% = original, 100% = keyed)      |

## Shapes in Detail

**Rectangle** — pixels pass if they fall within the axis-aligned bounding box defined by width and height. Set Knob 6 high (L-inf norm).

**Ellipse** — an elliptical region computed as a norm blend between L1 and L-inf, avoiding a square root. Set Knob 6 to mid. Accuracy improves visually over a true L2 for most use cases.

**Diamond** — a rotated 45° rectangle (L1 norm). Set Knob 6 low. The diamond points touch the width and height extents.

**Cross** — the union of a horizontal arm (width × height/2) and a vertical arm (height/2 × width). Knob 6 has no effect. Dither fill modes are not available; the cross is always solid.

## Inflate/Pinch Morphing

Knob 6 continuously interpolates between the L1 norm (diamond pinch) and the L-inf norm (rectangle inflate) via a midpoint blend that approximates an ellipse. This works for all shapes except Cross:

- Turning Knob 6 fully low on a Rectangle pinches the corners inward toward a diamond.
- Turning Knob 6 fully high on a Diamond inflates the sides outward toward a rectangle.
- The midpoint (Ellipse default) gives an ellipse-like curve at all Knob 3/4 settings.

## Dither Fill Modes

Dither modes soften the shape boundary by adding LFSR noise to the threshold comparison. A pixel is considered inside if its norm value is less than the (threshold + scaled noise) sum. The noise amount is controlled by S4/S5:

- **Slight** — noise ÷ 4: a narrow band of stochastic pixels at the edge
- **More** — noise ÷ 2: a wider transitional band
- **Max** — full noise: the broadest feathering effect

Dither is driven by a free-running 16-bit LFSR; the pattern shifts every frame, producing animated grain rather than a static texture.

## Typical Use

1. Select a shape with S2/S3 and set S1 to Normal.
2. Adjust Knobs 1 and 2 to position the shape centre over the region of interest.
3. Adjust Knobs 3 and 4 for the desired width and height; use Knob 5 to scale both proportionally.
4. Use Knob 6 to fine-tune the shape character (pinch corners, inflate sides).
5. Enable a dither fill mode (S4/S5) to soften the boundary if desired.
6. Use the Global Blend slider to fade between the original and keyed result.
7. Engage Invert (S1=1) to key the outside of the shape instead of the inside.

## Using with a Keyer

The Shape Key output can be fed to an external keyer such as the LZX FKG3. Because the outside regions are hard black (Y=0, U=V=512), the output can be used directly as a luma key source.

Set the FKG3 Key Source switch (S2) to External Key and patch the Shape Key output to FKG3 input J5. J5 is the external key source for the Red channel and is normalled to the Green and Blue channels, so a single patch cable keys all three. Set the FKG3 S3 switch (mode) to the Luma position for clean black/white keying.

## Technical Notes

- **Colour space:** Direct YUV (no colour conversion)
- **Pipeline latency:** 11 clock cycles
- **FPGA:** Lattice iCE40 HX4K (tq144) on Videomancer rev_b
- **BRAM usage:** 0 block RAMs
- **IOs:** 107 / 256
- **PLLs:** 0 / 2 (HD targets), 1 / 2 (SD targets)
- **HD timing:** All six HD variants meet 74.25 MHz (worst case ~76.71 MHz)
- **LC utilisation:** 4589–4611 of 7680 (~60%)
- **Norm computation:** L1/L-inf blend via 8×8 multiplies (no sqrt required)
- **Dither source:** Free-running 16-bit LFSR (lfsr16), lower 10 bits used

## Hardware Requirements

- LZX Industries Videomancer (rev_b)
- Firmware: v1.0.0-rc.12 or later

## License

Copyright (C) 2026 Pete Appleby  
GPL-3.0-only — see [LICENSE](../LICENSE)
