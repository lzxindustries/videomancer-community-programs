# CHROMA KEY

Chroma Key is a threshold keyer for Videomancer that replaces regions of the
input video with a solid fill color based on a selectable key channel. It
supports luma keying, chroma keying, and saturation keying with adjustable
softness, key inversion, and a dry/wet mix fader.

## OVERVIEW

Chroma Key examines a single channel of the incoming video — luma (Y), blue
chroma (U), red chroma (V), or UV saturation — and compares it against an
adjustable threshold. Pixels whose key channel value is near (or below) the
threshold are replaced by a solid fill color; pixels far from the threshold pass
through unchanged. The transition between keyed and non-keyed regions is
controlled by a Softness parameter that creates smooth, graduated edges.

Two keying modes are available:

- **Near** — keys pixels whose channel value is close to the threshold in either
  direction (distance = |value − threshold|). Useful for isolating a specific
  tonal or color range.
- **Below** — keys pixels whose channel value is at or below the threshold
  (distance = max(value − threshold, 0)). Useful for luma keying dark regions
  or gating low-saturation areas.

A dedicated Key View mode displays the computed matte as a grayscale image,
making it easy to fine-tune threshold and softness before committing to a
composite.

> **NOTE**
> Chroma Key uses zero block RAM. All processing is computed inline with no
> lookup tables or line buffers.

## QUICK START

1. Set **Key Channel** (Knob 1) to the channel you want to key on. Start with
   **Y** for luma keying or **U** for blue-screen keying.
2. Turn **Threshold** (Knob 2) to set the target value. For a luma key on a dark
   background, set it low (~10–20%).
3. Adjust **Softness** (Knob 3) to taste. Lower values produce a hard matte cut;
   higher values create a soft, graduated edge.
4. Toggle **Key View** (Switch 8) to **On** to see the matte. White areas pass
   through; black areas are keyed (replaced by fill). Use this to dial in your
   threshold and softness, then toggle it back off.
5. Push the **Mix** fader (Fader 12) to full to see the keyed result.

## PARAMETERS

### Knob 1 — Key Channel

| | |
|---|---|
| Values | Y, U, V, Sat |
| Default | Y |

Key Channel selects which signal component is compared against the threshold.

- **Y** — luminance. Use for luma keying (dark or bright regions).
- **U** — blue-difference chroma (Cb). Centred at 512. Use for blue-screen
  keying, or wherever the target region is distinguishable on the blue-yellow
  axis.
- **V** — red-difference chroma (Cr). Centred at 512. Use for green-screen
  keying (low V values indicate green), or wherever the target is
  distinguishable on the red-cyan axis.
- **Sat** — UV saturation, computed as Manhattan distance from UV neutral
  (|U−512| + |V−512|). Use to key based on overall color intensity regardless
  of hue.

### Knob 2 — Threshold

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 50.0% |

Threshold sets the target value against which the key channel is compared. In
**Near** mode, pixels within a certain distance of this value are keyed. In
**Below** mode, pixels at or below this value are keyed. The raw hardware value
maps directly to the 10-bit channel range (0–1023).

> **TIP**
> When keying on U or V, remember that 50% (512) is the neutral midpoint. Set
> the threshold below 50% to target blue (U) or green (V), or above 50% to
> target yellow (U) or red/magenta (V).

### Knob 3 — Softness

| | |
|---|---|
| Values | 1, 2, 3, 4, 5, 6, 7, 8 |
| Default | 3 |

Softness controls the width of the transition zone between keyed and non-keyed
regions. At step 1 (hardest), only pixels very close to the threshold are keyed,
producing a sharp matte edge. At step 8 (softest), the transition extends over a
wide tonal range, creating a gentle blend. Internally, softness controls a bit
shift applied to the distance value: step 1 amplifies the distance by 128×
(hard cut), step 8 passes it through unscaled (gradual ramp).

### Knob 4 — Fill Y

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 0.0% |

Fill Y sets the luminance of the replacement color that appears in keyed regions.
At 0%, keyed areas are black. At 100%, keyed areas are full white. Only active
when Fill Source (Switch 9) is set to **Color**.

### Knob 5 — Fill U

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 50.0% |

Fill U sets the blue-difference chroma of the replacement color. 50% is neutral
(no blue or yellow tint). Values below 50% tint keyed areas toward yellow;
values above 50% tint toward blue. Only active when Fill Source is set to
**Color**.

### Knob 6 — Fill V

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 50.0% |

Fill V sets the red-difference chroma of the replacement color. 50% is neutral
(no red or cyan tint). Values below 50% tint keyed areas toward green/cyan;
values above 50% tint toward red/magenta. Only active when Fill Source is set to
**Color**.

### Switch 7 — Key Invert

| | |
|---|---|
| Off | Normal key polarity |
| On | Inverted key polarity |
| Default | Off |

Key Invert flips the matte. With the switch Off, pixels near (or below) the
threshold are replaced by the fill color, and distant pixels pass through. With
the switch On, the opposite occurs: distant pixels are filled while near pixels
pass through. Use this to keep the keyed region instead of removing it.

### Switch 8 — Key View

| | |
|---|---|
| Off | Normal composite output |
| On | Matte displayed as grayscale |
| Default | Off |

Key View replaces the output image with the computed matte signal. White (1023)
means the original pixel passes through; black (0) means the pixel is fully
replaced by fill. Use this to visualize and fine-tune the key matte before
compositing.

### Switch 9 — Fill Source

| | |
|---|---|
| Color | Fill color from Knobs 4–6 |
| Black | Fill is black (Y=0, U=512, V=512) |
| Default | Black |

Fill Source selects where the replacement color comes from. When set to
**Black**, keyed regions are replaced with neutral black regardless of the Fill
Y/U/V knob positions. When set to **Color**, the fill is defined by Knobs 4–6,
allowing any arbitrary solid color.

### Switch 10 — Key Mode

| | |
|---|---|
| Near | Key pixels near the threshold (both directions) |
| Below | Key pixels at or below the threshold |
| Default | Near |

Key Mode changes how distance from the threshold is computed.

- **Near** — absolute distance: |value − threshold|. Pixels on both sides of
  the threshold within the softness range are keyed. Good for isolating a
  specific tone or hue.
- **Below** — one-sided: max(value − threshold, 0). Only pixels above the
  threshold produce a non-zero distance; pixels at or below the threshold are
  fully keyed. Good for luma keying dark backgrounds or cutting everything
  below a saturation floor.

### Switch 11 — Bypass

| | |
|---|---|
| Off | Effect active |
| On | Input passed through unchanged |
| Default | Off |

Bypass passes the input video directly to the output with no processing. The
signal is delayed by the same number of clocks as the processing pipeline (9) to
maintain sync alignment.

### Fader 12 — Mix

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 100.0% |

Mix crossfades between the dry (unprocessed) and wet (keyed) signal. At 0%, the
output is the unprocessed original. At 100%, the output is the fully keyed
composite. Intermediate values blend the two, allowing subtle integration of the
key effect. The mix is applied by scaling the matte before it reaches the
interpolators, so at 0% the matte is forced to all-pass regardless of threshold
and softness settings.

## SIGNAL FLOW

The processing pipeline runs at 9 clocks total (5 inline stages + 4 for the
output interpolators).

```
data_in ──┬──────────────────────────────────────────────────────┐
           │                                                      │
           ▼                                                      │
   ┌──────────────┐                                               │
   │   Stage 0    │  Key channel select (Y/U/V/Sat)              │
   │   1 clock    │  Fill source select (Color/Black)            │
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 1    │  Distance computation                        │
   │   1 clock    │  Near: |key − threshold|                     │
   │              │  Below: max(key − threshold, 0)              │
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 2    │  Softness shift + clamp + key invert         │
   │   1 clock    │  Distance → raw matte (0=fill, 1023=pass)   │
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 3    │  Matte × Mix multiply (20-bit product)       │
   │   1 clock    │                                               │
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 4    │  Extract top 10 bits → final matte           │
   │   1 clock    │                                               │
   └──────┬───────┘                                               │
          │                 ┌─────────┐                           │
          ├──── matte ─────▶│         │◀── fill (Y/U/V) ────┐    │
          │                 │ interp  │                      │    │
          │                 │  ×3     │◀── dry tap ────────────┤
          │                 │ 4 clks  │       (5-clock delay)  │  │
          │                 └────┬────┘                         │  │
          │                      │                              │  │
          │                      ▼                              │  │
          │               keyed Y/U/V                           │  │
          │                      │                              │  │
          │                      ▼                              │  │
          │              ┌──────────────┐                       │  │
          │              │  Output Mux  │◀── bypass ──────────────┘
          │              │              │     (9-clock delay)
          │              └──────┬───────┘
          │                     │
          │                     ▼
          │                 data_out
          │
          └──── sync signals (hsync_n, vsync_n, field_n, avid)
                  delayed 9 clocks via shift register
```

### Signal Flow Notes

1. **Dry tap alignment** — The original Y/U/V data is delayed by 5 clocks (the
   inline processing depth) so that it arrives at the interpolator inputs at
   the same time as the matte signal. The interpolator then blends between
   fill and dry over its own 4-clock latency.

2. **Matte convention** — A matte value of 0 means the pixel is fully keyed
   (fill color shows). A matte value of 1023 means the pixel passes through
   unchanged. The interpolator computes: `output = fill + (dry − fill) ×
   matte / 1023`.

3. **Mix is pre-applied** — The Mix fader scales the matte before
   interpolation (Stage 3–4). At Mix = 0%, the matte becomes all-zeros
   regardless of keying, so the interpolator outputs pure dry signal. This is
   more efficient than running a separate dry/wet interpolator stage.

## EXERCISES

### Exercise 1: Luma Key on Black Background

#### Learning Outcomes

Isolate a subject against a dark background and replace it with a solid color.

#### Video Source

Footage with a subject lit against a dark (near-black) background.

#### Steps

1. Set **Key Channel** (Knob 1) to **Y**.
2. Set **Key Mode** (Switch 10) to **Below**.
3. Turn **Threshold** (Knob 2) low, around 15–20%. This keys everything darker
   than the threshold.
4. Set **Softness** (Knob 3) to **3** or **4** to soften the edge.
5. Toggle **Key View** (Switch 8) **On**. Verify that the background is black
   (keyed) and the subject is white (passing through). Adjust Threshold and
   Softness until the matte is clean.
6. Toggle **Key View** back **Off**.
7. Set **Fill Source** (Switch 9) to **Color** and dial in a fill color with
   Knobs 4–6.
8. Push **Mix** (Fader 12) to 100%.

#### Settings

| | |
|---|---|
| Key Channel | Y |
| Threshold | 15% |
| Softness | 3 |
| Fill Y | 0% |
| Fill U | 50% |
| Fill V | 50% |
| Key Invert | Off |
| Key View | Off |
| Fill Source | Color |
| Key Mode | Below |
| Bypass | Off |
| Mix | 100% |

### Exercise 2: Green-Screen Chroma Key

#### Learning Outcomes

Remove a green background using the V (red-difference) chroma channel.

#### Video Source

Footage shot against a green screen or any strongly green-coloured background.

#### Steps

1. Set **Key Channel** (Knob 1) to **V**. In YCbCr, green has a low V (Cr)
   value.
2. Set **Key Mode** (Switch 10) to **Below**.
3. Turn **Threshold** (Knob 2) to around 40–45%. Green regions have V values
   well below the 512 midpoint.
4. Set **Softness** (Knob 3) to **4** or **5** for smooth edges.
5. Toggle **Key View** (Switch 8) **On** and adjust until the green area is
   cleanly black in the matte.
6. Toggle **Key View** **Off**.
7. Set **Fill Source** (Switch 9) to **Color** and choose a replacement color,
   or leave it on **Black**.
8. Push **Mix** (Fader 12) to 100%.

#### Settings

| | |
|---|---|
| Key Channel | V |
| Threshold | 42% |
| Softness | 4 |
| Fill Y | 0% |
| Fill U | 50% |
| Fill V | 50% |
| Key Invert | Off |
| Key View | Off |
| Fill Source | Black |
| Key Mode | Below |
| Bypass | Off |
| Mix | 100% |

### Exercise 3: Saturation Matte

#### Learning Outcomes

Use the UV saturation channel to isolate colourful regions from desaturated ones.

#### Video Source

Footage with a mix of colourful and neutral/grey areas.

#### Steps

1. Set **Key Channel** (Knob 1) to **Sat**.
2. Set **Key Mode** (Switch 10) to **Below**.
3. Turn **Threshold** (Knob 2) to around 20–30% to key everything below that
   saturation level.
4. Set **Softness** (Knob 3) to **3**.
5. Toggle **Key Invert** (Switch 7) **On** — now colourful regions are replaced
   and desaturated regions pass through (or vice versa, depending on your
   intent).
6. Toggle **Key View** (Switch 8) to inspect the matte and fine-tune.
7. Set fill colour and Mix as desired.

#### Settings

| | |
|---|---|
| Key Channel | Sat |
| Threshold | 25% |
| Softness | 3 |
| Fill Y | 0% |
| Fill U | 50% |
| Fill V | 50% |
| Key Invert | On |
| Key View | Off |
| Fill Source | Black |
| Key Mode | Below |
| Bypass | Off |
| Mix | 100% |

## GLOSSARY

- **Chroma Key** — A compositing technique that removes a specific colour from
  an image to reveal a different background or fill.
- **Dry** — The unprocessed input signal, before any keying is applied.
- **Fill** — The replacement signal that appears in keyed (removed) regions.
- **Interpolator** — A linear blending unit that crossfades between two values
  based on a third control value (the matte).
- **Key Channel** — The signal component (Y, U, V, or Saturation) examined to
  determine which pixels are keyed.
- **Luma Key** — Keying based on brightness (the Y channel).
- **Manhattan Distance** — The sum of absolute differences along each axis;
  used here to compute UV saturation as |U−512| + |V−512|.
- **Matte** — A per-pixel control signal that determines the blend ratio
  between fill and dry. 0 = fully keyed (fill), 1023 = fully passed (dry).
- **Softness** — The width of the transition zone between fully keyed and
  fully passed regions.
- **Threshold** — The target value on the key channel around which (or below
  which) pixels are keyed.
- **Wet** — The fully processed (keyed) output signal.
