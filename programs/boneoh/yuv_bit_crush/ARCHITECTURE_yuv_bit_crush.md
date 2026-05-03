# YUV Bit Crush — Architecture

This document describes the signal flow and pipeline architecture of `yuv_bit_crush.vhd` for developers. Signal names match the VHDL exactly.

---

## Pipeline Overview

Total latency: **10 clock cycles** at 74.25 MHz.

```
T+0  data_in.y/u/v ───────────────────────────────── bypass delay (10 clocks) ──► data_out (bypass)
     registers_in[0..7]                               sync delay  (10 clocks) ──► data_out.hsync_n/vsync_n/field_n
         │
         ▼
    ┌─────────┐  1 clk
    │ Stage 0a│  control decode
    │         │  crush amount decode + pre-register lmask/roundoff
    └────┬────┘
         │  s_crush_y/u/v_r, s_round_u/v_r, s_dither_r, s_invert_r
         │  s_lmask_u/v_r, s_roundoff_u/v_r
         │  s_y_d1, s_u_d1, s_v_d1, s_avid_d1
         │  s_lfsr_reset
         ▼
    ┌─────────┐  1 clk
    │ Stage 0b│  apply_crush(Y/U/V) + optional invert
    └────┬────┘
         │  s_processed_y/u/v, s_processed_valid
         │  s_orig_y/u/v_d2  (dry input for per-channel blend)
         │
    ┌────┴──────────────────────────────────────────────────────┐
    │  [WET]                                    [DRY]           │
    │  s_processed_y/u/v (T+2)       s_orig_y/u/v_d2 (T+2)    │
    │  s_blend_y/u/v (combinational)                            │
    ▼                                           ▼              │
    ┌──────────────────────────────────────────────────────┐   │
    │  Stage 1: 3x interpolator_u (per-channel blend)      │   │
    │  interp_y / interp_u / interp_v  (4 clocks)          │   │
    └──────────────────────┬───────────────────────────────┘   │
                           │  s_blended_y/u/v (T+6)            │
                           │                                    │
    ┌──────────────────────┼────────────────────────────────────┘
    │  [WET]               │              [DRY]
    │  s_blended_y/u/v (T+6)    s_y/u/v_for_global (T+6)
    │  s_global_blend (combinational)      │
    │                                      │ p_global_dry_delay
    ▼                                      ▼ (6-clock shift from T+0)
    ┌──────────────────────────────────────────────────────┐
    │  Stage 2: 3x interpolator_u (global blend)  4 clocks │
    │  interp_global_y / interp_global_u / interp_global_v │
    └──────────────────────┬───────────────────────────────┘
                           │  s_global_y/u/v (T+10)
                           ▼
              data_out.y/u/v (process path), data_out.avid
```

**Bypass path** (S5 On): `s_y/u/v_delayed` (10-clock shift) → `data_out.y/u/v` directly.

---

## Stage Detail

### Stage 0a — Control Decode (T+0 → T+1)
**Process:** `p_control_decode`

**Inputs (T+0):**
- `data_in.y`, `data_in.u`, `data_in.v`, `data_in.avid`, `data_in.vsync_n`
- `registers_in(0..2)` — Y/U/V crush amounts (knobs 1–3)
- `registers_in(3..5)` — Y/U/V channel blends (knobs 4–6, combinational)
- `registers_in(6)(0)` — Invert (S1)
- `registers_in(6)(1)` — Dither (S2)
- `registers_in(6)(2)` — U Round (S3)
- `registers_in(6)(3)` — V Round (S4)
- `registers_in(6)(4)` — Bypass (S5, combinational)

**Outputs (T+1):**

| Signal | Description |
|--------|-------------|
| `s_crush_y_r`, `s_crush_u_r`, `s_crush_v_r` | Step index 0–7 (`knob / 128`) |
| `s_lmask_u_r`, `s_lmask_v_r` | Pre-registered step-lower masks for U/V |
| `s_roundoff_u_r`, `s_roundoff_v_r` | Pre-registered round offsets (step/2) for U/V |
| `s_round_u_r`, `s_round_v_r` | `registers_in(6)(2)/(3)` — round-to-nearest per channel |
| `s_dither_r` | `registers_in(6)(1)` — RPDF dither enable |
| `s_invert_r` | `registers_in(6)(0)` — bitwise-NOT all channels |
| `s_y_d1`, `s_u_d1`, `s_v_d1`, `s_avid_d1` | Registered pixel data |
| `s_lfsr_reset` | One-clock pulse at vsync falling edge (unconditional reseed) |

**Knob → step index mapping (`knob_to_crush`):**
```
step_idx = knob / 128   (integer divide — 8 equal bands of 128 counts)
```

**Step index → step size:**

| `step_idx` | Step size | Max quantised value |
|-----------|-----------|-------------------|
| 0 | 8 | 1016 |
| 1 | 16 | 1008 |
| 2 | 32 | 992 |
| 3 | 48 | 1008 |
| 4 | 64 | 960 |
| 5 | 96 | 960 |
| 6 | 128 | 896 |
| 7 | 256 | 768 |

**Pre-registered helper values** — `crush_to_lmask` and `crush_to_roundoff` are evaluated in Stage 0a so their case decode trees do not appear on Stage 0b's critical path:

| `step_idx` | `lmask` (largest 2^k−1 < step) | `roundoff` (step/2) |
|-----------|-------------------------------|-------------------|
| 0 (step 8) | 7 | 4 |
| 1 (step 16) | 15 | 8 |
| 2 (step 32) | 31 | 16 |
| 3 (step 48) | 31 | 24 |
| 4 (step 64) | 63 | 32 |
| 5 (step 96) | 63 | 48 |
| 6 (step 128) | 127 | 64 |
| 7 (step 256) | 255 | 128 |

---

### Stage 0b — Bit Crush (T+1 → T+2)
**Process:** `p_bit_crush`

**Inputs (Stage 0a registered outputs, T+1):**
`s_crush_y/u/v_r`, `s_round_u/v_r`, `s_dither_r`, `s_invert_r`, `s_lmask_u/v_r`, `s_roundoff_u/v_r`, `s_y_d1/u_d1/v_d1`, `s_avid_d1`, `s_lfsr16_out`, `s_lfsr10_out`

**Outputs (T+2):**

| Signal | Description |
|--------|-------------|
| `s_processed_y/u/v` | Crushed (and optionally inverted) pixel |
| `s_processed_valid` | `s_avid_d1` |
| `s_orig_y/u/v_d2` | `s_y/u/v_d1` re-registered — dry input for Stage 1 |

**`apply_crush` function logic:**

```
1. Select offset:
   do_round = '1' → offset = roundoff (step/2)
   do_dither = '1' (and do_round = '0') → offset = dither AND lmask
   otherwise → offset = 0

2. v_pix = pixel + offset  (11-bit, max 1023+255=1278)

3. Quantise:
   Power-of-2 steps (8,16,32,64,128,256):
     overflow check: if v_pix[10]='1' → return saturated max
     else → shift right to remove lower bits → return with lower bits zeroed
   Non-power-of-2 steps (48, 96):
     case statement LUT on v_pix[10:4] (step 48) or v_pix[10:5] (step 96)
     returns pre-computed multiple directly — no multiply inferred
```

**Y vs U/V treatment:**
- **Y**: always `apply_crush` with `do_round='0'`, `do_dither='0'` — pure truncation, no dither
- **U**: `apply_crush` with `s_round_u_r`, `s_dither_r`, `s_lmask_u_r`, `s_roundoff_u_r`; dither source = `lfsr16_out[9:0]`
- **V**: `apply_crush` with `s_round_v_r`, `s_dither_r`, `s_lmask_v_r`, `s_roundoff_v_r`; dither source = `lfsr10_out`

**Invert** (applied after crush, before output):
```
if s_invert_r = '1': s_processed_y/u/v = NOT v_y/u/v
else:                 s_processed_y/u/v = v_y/u/v
```

---

### LFSR / PRNG Dither Sources (free-running, every clock)

**Modules:** `u_lfsr16` (SDK), `u_lfsr10` (SDK)

```
vsync_n ──► p_control_decode ──► s_lfsr_reset (1-clock pulse, unconditional at vsync) ──► u_lfsr10.reset
u_lfsr16 ─────────────────────────────────────────────────────────────────────────────► u_lfsr10.seed[9:1] & '1'
u_lfsr16 ──► s_lfsr16_out[9:0] ──► Stage 0b (U dither source)
u_lfsr10 ──► s_lfsr10_out[9:0] ──► Stage 0b (V dither source)
```

- `u_lfsr16` — free-runs always, period 65535. U dither source.
- `u_lfsr10` — polynomial x¹⁰ + x⁷ + 1, period 1023. Reseeds from `lfsr16_out[9:0]` at every vsync falling edge (unconditional). Seed bit 0 forced `'1'`. V dither source.
- Different generators for U and V ensure uncorrelated dither patterns on the two chroma channels.

**Dither gate:** Inside `apply_crush`, the LFSR output is masked with `lmask` before adding: `offset = dither AND lmask`. This limits dither amplitude to at most `lmask` (the largest 2^k−1 < step), preventing over-quantisation.

---

### Stage 1 — Per-Channel Wet/Dry Blend (T+2 → T+6)
**Instances:** `interp_y`, `interp_u`, `interp_v` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_orig_y/u/v_d2` | 2 | Original pixel, 2-clock delayed |
| `b` (wet) | `s_processed_y/u/v` | 2 | Crushed result |
| `t` | `s_blend_y/u/v` | — | Per-channel blend (combinational, knobs 4–6) |
| `enable` | `s_processed_valid` | — | Holds output stable during blanking |

Each channel has an independent blend knob — Y, U, V can be mixed at different ratios.

---

### Global Dry Delay Line (T+0 → T+6)
**Process:** `p_global_dry_delay`

```
data_in.y/u/v ──► [6-clock shift] ──► s_y/u/v_for_global (T+6)
```
Aligned with `s_blended_y/u/v` (T+6) for the Stage 2 interpolator `a` (dry) input.

---

### Stage 2 — Global Wet/Dry Blend (T+6 → T+10)
**Instances:** `interp_global_y`, `interp_global_u`, `interp_global_v` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_y/u/v_for_global` | 6 | Original pixel, 6-clock delayed |
| `b` (wet) | `s_blended_y/u/v` | 6 | Per-channel blended result |
| `t` | `s_global_blend` | — | Global blend factor (combinational, slider) |
| `enable` | `s_blended_y/u/v_valid` | — | Holds output stable during blanking |

---

### Bypass / Sync Delay Line (T+0 → T+10)
**Process:** `p_bypass_delay`

```
data_in.hsync_n/vsync_n/field_n ──► [10-clock shift] ──► data_out.hsync_n/vsync_n/field_n (always)
data_in.y/u/v                   ──► [10-clock shift] ──► s_y/u/v_delayed  (bypass path only)
```

**Output mux (combinational):**
```
s_bypass_enable = '0' (Process): data_out.y/u/v = s_global_y/u/v
s_bypass_enable = '1' (Bypass):  data_out.y/u/v = s_y/u/v_delayed
data_out.avid always = s_global_y_valid
```

---

## Signal Timing Summary

| Signal | T+ | Source |
|--------|----|--------|
| `data_in.y/u/v` | 0 | input port |
| `s_crush_y/u/v_r` | 1 | `p_control_decode` |
| `s_lmask_u/v_r`, `s_roundoff_u/v_r` | 1 | `p_control_decode` |
| `s_round_u/v_r`, `s_dither_r`, `s_invert_r` | 1 | `p_control_decode` |
| `s_y_d1`, `s_u_d1`, `s_v_d1` | 1 | `p_control_decode` |
| `s_lfsr_reset` | 1 | `p_control_decode` |
| `s_processed_y/u/v` | 2 | `p_bit_crush` |
| `s_orig_y/u/v_d2` | 2 | `p_bit_crush` |
| `s_blended_y/u/v` | 6 | `interp_y/u/v` |
| `s_y/u/v_for_global` | 6 | `p_global_dry_delay` |
| `s_global_y/u/v` | 10 | `interp_global_y/u/v` |
| `data_out.hsync_n/vsync_n/field_n` | 10 | `p_bypass_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | Y crush amount | Knob 1 |
| `registers_in(1)` | U crush amount | Knob 2 |
| `registers_in(2)` | V crush amount | Knob 3 |
| `registers_in(3)` | Y channel blend | Knob 4 |
| `registers_in(4)` | U channel blend | Knob 5 |
| `registers_in(5)` | V channel blend | Knob 6 |
| `registers_in(6)(0)` | Invert | S1 |
| `registers_in(6)(1)` | Dither | S2 |
| `registers_in(6)(2)` | U Round | S3 |
| `registers_in(6)(3)` | V Round | S4 |
| `registers_in(6)(4)` | Bypass | S5 |
| `registers_in(7)` | Global blend | Slider |

---

## Key Design Decisions

**Pre-registered lmask and roundoff** — `crush_to_lmask` and `crush_to_roundoff` are evaluated in Stage 0a and their results registered as `s_lmask_u/v_r` and `s_roundoff_u/v_r`. This removes two case decode trees from Stage 0b's critical path. Y always truncates so it needs neither; Y crush is still registered (`s_crush_y_r`) so Stage 0b sees only FF inputs.

**Non-power-of-2 steps as LUT** — steps 48 and 96 cannot be implemented as simple bit shifts. The `apply_crush` function handles them as case statements indexed by the shifted pixel value (`v_pix[10:4]` for step 48, `v_pix[10:5]` for step 96). These synthesise as shallow LUT trees (~3–4 levels) rather than carry-chain dividers.

**Separate dither sources for U and V** — `u_lfsr16` drives U dither and `u_lfsr10` drives V dither. Using independent generators ensures the two chroma channels receive uncorrelated noise, preventing dither patterns from visually correlating between U and V.

**Dither masking** — dither amplitude is gated to `lmask` (largest 2^k−1 < step) inside `apply_crush`. This prevents dither from pushing a sample past the next quantisation boundary, which would cause systematic bias rather than the intended noise shaping.

**Round vs dither exclusivity** — round takes priority over dither inside `apply_crush` via the offset selection logic. This is intentional: rounding to nearest is a deterministic operation while dither is stochastic; mixing them would undermine both. The user selects one mode per channel via the per-channel Round switches, and a global Dither switch.

**Y truncation only** — Y (luma) always uses pure truncation. Rounding and dithering luma would introduce perceptible brightness shifts with some quantisation levels, which is undesirable for the typical use case of this program. Chroma channels (U, V) benefit more from dither since chroma quantisation banding is more visually objectionable.
