# YUV Bit Rotator — Architecture

This document describes the signal flow and pipeline architecture of `yuv_bit_rotator.vhd` for developers. Signal names match the VHDL exactly.

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
    │         │  raw_to_shift + get_bit_depth + depth_to_mask
    └────┬────┘
         │  s_shift_y/u/v_r  (integer 0–10)
         │  s_depth_r        (integer 1–10)
         │  s_mask_r         (10-bit AND mask)
         │  s_direction_r    (ROL=0 / ROR=1)
         │  s_y_d1, s_u_d1, s_v_d1, s_avid_d1
         ▼
    ┌─────────┐  1 clk
    │ Stage 0b│  rol10 or ror10 per channel
    │         │  AND s_mask_r (bit depth gate)
    └────┬────┘
         │  s_rotated_y/u/v, s_rotated_valid
         │  s_orig_y/u/v_d2  (dry input for per-channel blend)
         │
    ┌────┴──────────────────────────────────────────────────────┐
    │  [WET]                                    [DRY]           │
    │  s_rotated_y/u/v (T+2)         s_orig_y/u/v_d2 (T+2)    │
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
**Process:** `p_decode_stage`

**Inputs (T+0):**
- `data_in.y`, `data_in.u`, `data_in.v`, `data_in.avid`
- `registers_in(0..2)` — Y/U/V rotation amounts (knobs 1–3)
- `registers_in(6)(0)` — Direction: 0=ROL, 1=ROR (S1)
- `registers_in(6)(1..3)` — Bit depth S1/S2/S3 (switches 2–4)
- `registers_in(6)(4)` — Bypass (S5, combinational)

**Outputs (T+1):**

| Signal | Description |
|--------|-------------|
| `s_shift_y_r`, `s_shift_u_r`, `s_shift_v_r` | Shift amount 0–10 per channel |
| `s_depth_r` | Active bit depth 1–10 |
| `s_mask_r` | 10-bit AND mask for bit depth gate |
| `s_direction_r` | Registered direction flag (0=ROL, 1=ROR) |
| `s_y_d1`, `s_u_d1`, `s_v_d1`, `s_avid_d1` | Registered pixel data |

**`raw_to_shift` — knob to shift amount (0–10):**

| Knob range | Shift |
|-----------|-------|
| 0–51 | 0 |
| 52–153 | 1 |
| 154–255 | 2 |
| 256–358 | 3 |
| 359–460 | 4 |
| 461–562 | 5 |
| 563–665 | 6 |
| 666–767 | 7 |
| 768–869 | 8 |
| 870–971 | 9 |
| 972–1023 | 10 |

Shift 10 is equivalent to shift 0 for a 10-bit rotation (full cycle).

**`get_bit_depth` — switch encoding (S3:S2:S1 = `reg(6)(3):reg(6)(2):reg(6)(1)`):**

| S3 | S2 | S1 | Depth |
|----|----|----|-------|
| 0 | 0 | 0 | 10-bit |
| 0 | 0 | 1 | 8-bit |
| 0 | 1 | 0 | 6-bit |
| 0 | 1 | 1 | 5-bit |
| 1 | 0 | 0 | 4-bit |
| 1 | 0 | 1 | 3-bit |
| 1 | 1 | 0 | 2-bit |
| 1 | 1 | 1 | 1-bit |

S3 (S4 on front panel, `reg(6)(3)`) is the MSB — flipping S4 has the biggest depth effect. S1 (S2 on front panel, `reg(6)(1)`) is the LSB.

**`depth_to_mask` — bit depth to 10-bit AND mask:**

| Depth | Mask (hex) | Effect |
|-------|-----------|--------|
| 10 | `0x3FF` | Pass all bits |
| 8 | `0x3FC` | Zero bits 1:0 |
| 6 | `0x3F0` | Zero bits 3:0 |
| 5 | `0x3E0` | Zero bits 4:0 |
| 4 | `0x3C0` | Zero bits 5:0 |
| 3 | `0x380` | Zero bits 6:0 |
| 2 | `0x300` | Zero bits 7:0 |
| 1 | `0x200` | Zero bits 8:0 |

**Timing note:** `raw_to_shift` (carry chains — comparisons on 10-bit values) and `depth_to_mask` (case decode) land here in Stage 0a, isolating them from Stage 0b's critical path.

---

### Stage 0b — Bit Rotation (T+1 → T+2)
**Process:** `p_rotation_stage`

**Inputs (Stage 0a registered outputs, T+1):**
`s_shift_y/u/v_r`, `s_mask_r`, `s_direction_r`, `s_y_d1/u_d1/v_d1`, `s_avid_d1`

**Outputs (T+2):**

| Signal | Description |
|--------|-------------|
| `s_rotated_y/u/v` | Rotated and masked pixel per channel |
| `s_rotated_valid` | Always `'1'` (registered from constant) |
| `s_orig_y/u/v_d2` | `s_y/u/v_d1` re-registered — dry input for Stage 1 |

**Operation:**
```
if s_direction_r = '0':
    s_rotated_y = rol10(s_y_d1, s_shift_y_r) AND s_mask_r
    s_rotated_u = rol10(s_u_d1, s_shift_u_r) AND s_mask_r
    s_rotated_v = rol10(s_v_d1, s_shift_v_r) AND s_mask_r
else:
    s_rotated_y = ror10(s_y_d1, s_shift_y_r) AND s_mask_r
    s_rotated_u = ror10(s_u_d1, s_shift_u_r) AND s_mask_r
    s_rotated_v = ror10(s_v_d1, s_shift_v_r) AND s_mask_r
```

**`rol10` / `ror10` implementation:**
- Both are pure combinational bit reordering (case on shift amount mod 10)
- No arithmetic — each case is a fixed concatenation of bit slices
- `ror10(v, n)` = `rol10(v, 10 - n)` — ROR reuses ROL
- Synthesises as a mux tree (LUT-only, no carry chains)

**AND mask application** — the bit depth mask is applied after rotation. Lower bits of the rotated value that fall outside the active bit depth are zeroed. This means rotation wraps within the full 10-bit space but only the upper `depth` bits carry through to the output.

**Timing note:** Critical path is `registered_data → bit mux (rol/ror case) → AND registered_mask → register`. No carry chains — pure LUT logic.

---

### Stage 1 — Per-Channel Wet/Dry Blend (T+2 → T+6)
**Instances:** `interp_y`, `interp_u`, `interp_v` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_orig_y/u/v_d2` | 2 | Original pixel, 2-clock delayed |
| `b` (wet) | `s_rotated_y/u/v` | 2 | Rotated and masked result |
| `t` | `s_blend_y/u/v` | — | Per-channel blend (combinational, knobs 4–6) |
| `enable` | `s_rotated_valid` | — | Always `'1'` |

Each channel has an independent blend knob — Y, U, V rotation amounts and blend ratios can be set independently.

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
| `t` | `s_global_blend` | — | Global blend (combinational, slider) |
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
| `s_shift_y/u/v_r` | 1 | `p_decode_stage` |
| `s_depth_r`, `s_mask_r` | 1 | `p_decode_stage` |
| `s_direction_r` | 1 | `p_decode_stage` |
| `s_y_d1`, `s_u_d1`, `s_v_d1` | 1 | `p_decode_stage` |
| `s_rotated_y/u/v` | 2 | `p_rotation_stage` |
| `s_orig_y/u/v_d2` | 2 | `p_rotation_stage` |
| `s_blended_y/u/v` | 6 | `interp_y/u/v` |
| `s_y/u/v_for_global` | 6 | `p_global_dry_delay` |
| `s_global_y/u/v` | 10 | `interp_global_y/u/v` |
| `data_out.hsync_n/vsync_n/field_n` | 10 | `p_bypass_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | Y rotation amount | Knob 1 |
| `registers_in(1)` | U rotation amount | Knob 2 |
| `registers_in(2)` | V rotation amount | Knob 3 |
| `registers_in(3)` | Y channel blend | Knob 4 |
| `registers_in(4)` | U channel blend | Knob 5 |
| `registers_in(5)` | V channel blend | Knob 6 |
| `registers_in(6)(0)` | Direction (0=ROL, 1=ROR) | S1 |
| `registers_in(6)(1)` | Bit depth LSB (S1) | S2 |
| `registers_in(6)(2)` | Bit depth mid (S2) | S3 |
| `registers_in(6)(3)` | Bit depth MSB (S3) | S4 |
| `registers_in(6)(4)` | Bypass | S5 |
| `registers_in(7)` | Global blend | Slider |

---

## Key Design Decisions

**Pre-registered shift amounts** — `raw_to_shift` involves a chain of 10-bit comparisons (carry chains). Registering `s_shift_y/u/v_r` in Stage 0a removes this from Stage 0b's critical path. Stage 0b then sees only registered integer inputs feeding a pure mux tree.

**Pre-registered depth mask** — `depth_to_mask` is a case statement evaluated in Stage 0a and registered as `s_mask_r`. Stage 0b applies it as a simple AND with a registered operand — no combinational case decode on the critical path.

**ROL/ROR as bit reordering** — `rol10` and `ror10` are implemented as `case` statements on the shift amount, each returning a fixed concatenation of bit slices. No arithmetic is involved; synthesis maps each case directly to a mux tree. This is significantly cheaper than a barrel shifter for a 10-bit value with discrete shift positions.

**Shared depth mask** — the same `s_mask_r` is applied to all three channels (Y, U, V) after rotation. Bit depth is a global parameter, not per-channel. The mask zeroes the lower bits that fall outside the active bit depth after rotation.

**`s_rotated_valid` always `'1'`** — unlike other programs where `enable` on the interpolators tracks `avid`, the rotator drives `s_rotated_valid <= '1'` unconditionally. The rotation result is valid every clock; blanking pixels will rotate but the interpolator output during blanking is unused (sync is from the delay line). This simplifies the control logic at the cost of the interpolators running on blanking pixels.

**Bit depth switch ordering** — S4 (front panel) = `reg(6)(3)` = MSB of the 3-bit depth word. Flipping S4 produces the largest change in bit depth (e.g. 10-bit → 4-bit). This is the documented convention from CLAUDE.md: highest-impact switch = MSB. The TOML and README reflect this ordering.
