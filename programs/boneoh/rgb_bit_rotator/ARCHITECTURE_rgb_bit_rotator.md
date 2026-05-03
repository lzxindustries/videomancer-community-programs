# RGB Bit Rotator — Architecture

This document describes the signal flow and pipeline architecture of `rgb_bit_rotator.vhd` for developers. Signal names match the VHDL exactly.

Note: the VHD header comments label Stage 0b as T+2 while the actual pipeline is T+3 because Stage 0a-ii was added later. The signal timing summary below reflects the actual code.

---

## Pipeline Overview

Total latency: **17 clock cycles** (`C_PROCESSING_DELAY_CLKS = 17`) at 74.25 MHz.

```
T+0  data_in.y/u/v ──────────────────────────── bypass delay (17 clocks) ──► data_out (bypass)
     registers_in[0..7]                          sync delay  (17 clocks) ──► data_out.hsync_n/vsync_n/field_n
         │
         ▼
    ┌──────────┐  1 clk    YUV→RGB (1/3): BRAM lookup
    │ Stage 0a │  4 channel offsets
    │          │  control decode: to_eff_shift + depth_to_mask
    └────┬─────┘
         │  s_yr_r/gu/gv/b_off, s_yr_y/avid/u_raw/v_raw
         │  s_eff_shift_r/g/b  (direction folded in)
         │  s_mask_r            (bit depth AND mask)
         ▼
    ┌──────────┐  1 clk    YUV→RGB (2/3): G pre-sum
    │Stage 0a-ii  s_yr_gu_off + s_yr_gv_off
    └────┬─────┘
         │  s_yr_g_presum, s_yr_r/b_off_d, s_yr_y/avid/u_raw/v_raw_d
         ▼
    ┌──────────┐  1 clk    YUV→RGB (3/3): accumulate + clamp
    │ Stage 0b │  Y + offset per channel → s_rgb_r/g/b
    └────┬─────┘
         │  s_rgb_r/g/b, s_rgb_valid
         ▼
    ┌──────────┐  1 clk    ROL rotation (10-way mux, direction pre-folded)
    │ Stage 1a │  rol10(channel, s_eff_shift_*)
    └────┬─────┘
         │  s_rotated_pre_r/g/b, s_rotated_pre_valid
         ▼
    ┌──────────┐  1 clk    bit depth masking
    │ Stage 1b │  s_rotated_pre AND s_mask_r
    └────┬─────┘
         │  s_rotated_r/g/b, s_rotated_valid
         │  [dry: s_r/g/b_for_blend (T+5) from p_global_dry_delay index 1]
         │
    ┌────┴──────────────────────────────────────────────────────┐
    │  [WET]                                    [DRY]           │
    │  s_rotated_r/g/b (T+5)         s_r/g/b_for_blend (T+5)  │
    │  s_blend_r/g/b (combinational)                            │
    ▼                                           ▼              │
    ┌──────────────────────────────────────────────────────┐   │
    │  Stage 2: 3x interpolator_u (per-channel blend)      │   │
    │  interp_r / interp_g / interp_b  (4 clocks)          │   │
    └──────────────────────┬───────────────────────────────┘   │
                           │  s_blended_r/g/b (T+9)            │
                           │                                    │
    ┌──────────────────────┼────────────────────────────────────┘
    │  [WET]               │              [DRY]
    │  s_blended_r/g/b (T+9)    s_r/g/b_for_global (T+9)
    │  s_global_blend (combinational)      │
    │                                      │ p_global_dry_delay
    ▼                                      ▼ (6-clock tap from s_rgb at T+3)
    ┌──────────────────────────────────────────────────────┐
    │  Stage 3: 3x interpolator_u (global blend)  4 clocks │
    │  interp_global_r / interp_global_g / interp_global_b │
    └──────────────────────┬───────────────────────────────┘
                           │  s_global_r/g/b (T+13)
                           ▼
    ┌──────────┐  1 clk    RGB→YUV (1/4): BRAM lookup (7 tables)
    │Stage 4a-i│
    └────┬─────┘
         │  s_4a_ry_r/g/b, s_4a_ru_r/g, s_4a_rv_g/b, s_4a_r/b_d
         ▼
    ┌──────────┐  1 clk    RGB→YUV (2/4): partial pair sums
    │Stage 4a-ii
    └────┬─────┘
         │  s_4b_ry_rg/b, s_4b_ru_rg, s_4b_rv_gb, s_4b_r/b_d
         ▼
    ┌──────────┐  1 clk    RGB→YUV (3/4): channel sums + wire shifts
    │Stage 4a-iii
    └────┬─────┘
         │  s_ry_y/u/v_sum
         ▼
    ┌──────────┐  1 clk    RGB→YUV (4/4): clamp + UV offset + convention swap
    │ Stage 4b │
    └────┬─────┘
         │  s_yuv_out_y/u/v, s_yuv_out_valid
         ▼
    data_out.y/u/v, data_out.avid
```

**Bypass path** (S5 On): `s_y/u/v_delayed` (17-clock shift) → `data_out.y/u/v` directly.

---

## BT.601 Colour Conversion

Both paths use BRAM tables from `rgb_yuv_tables_pkg.vhd` (11 BRAMs, 31/32 blocks used). Identical structure to `rgb_bit_logic` — see that document for UV convention and BRAM arithmetic isolation rules.

---

## Stage Detail

### Stage 0a — YUV→RGB BRAM Lookup + Control Decode (T+0 → T+1)
**Process:** `p_yuv_rgb_lut`

**BRAM lookups (U/V swapped for convention):** identical to other RGB programs — 4 tables producing `s_yr_r/gu/gv/b_off`.

**Control decode (parallel with BRAM, no added latency):**

| Signal | Computation | Description |
|--------|-------------|-------------|
| `s_eff_shift_r/g/b` | `to_eff_shift(raw_to_shift(knob), direction)` | Effective ROL shift 0–9 |
| `s_mask_r` | `depth_to_mask(get_bit_depth(S1,S2,S3))` | 10-bit AND mask |

**`to_eff_shift` — direction fold:**
```
direction = '0' (ROL): eff_shift = raw_shift mod 10
direction = '1' (ROR): eff_shift = (10 - raw_shift) mod 10
```
This converts ROR into an equivalent ROL, so Stage 1a needs only a 10-way ROL mux instead of a 20-way ROL/ROR mux — saves ~1 LUT level on the critical path.

**`raw_to_shift` and `depth_to_mask`:** identical to `yuv_bit_rotator` — see that document for the full tables.

**Timing note:** `raw_to_shift` comparisons (carry chains) and `depth_to_mask` (case decode) both land here in Stage 0a, isolated from Stages 1a and 1b.

---

### Stage 0a-ii — G-channel Pre-Sum (T+1 → T+2)
**Process:** `p_yuv_rgb_presum` — identical to other RGB programs.

---

### Stage 0b — YUV→RGB Accumulate and Clamp (T+2 → T+3)
**Process:** `p_yuv_rgb_acc` — identical to other RGB programs. Outputs `s_rgb_r/g/b`, `s_rgb_valid`.

---

### Stage 1a — Bit Rotation (T+3 → T+4)
**Process:** `p_rotation_stage`

**Inputs (Stage 0a registered, T+1; Stage 0b outputs, T+3):**
`s_rgb_r/g/b`, `s_rgb_valid`, `s_eff_shift_r/g/b`

**Outputs (T+4):** `s_rotated_pre_r/g/b`, `s_rotated_pre_valid`

**Operation:**
```
s_rotated_pre_r = rol10(s_rgb_r, s_eff_shift_r)
s_rotated_pre_g = rol10(s_rgb_g, s_eff_shift_g)
s_rotated_pre_b = rol10(s_rgb_b, s_eff_shift_b)
```

`rol10` is a pure bit reordering case statement (same as `yuv_bit_rotator`) — no arithmetic, no carry chains. ROR has already been converted to equivalent ROL via `to_eff_shift` in Stage 0a.

**Timing note:** Critical path is `registered_data → 10-way mux (rol10) → register`. Pure LUT, no carry chains.

---

### Stage 1b — Bit Depth Masking (T+4 → T+5)
**Process:** `p_mask_stage`

**Inputs (Stage 1a outputs, T+4):** `s_rotated_pre_r/g/b`, `s_rotated_pre_valid`, `s_mask_r`

**Outputs (T+5):** `s_rotated_r/g/b`, `s_rotated_valid`

**Operation:**
```
s_rotated_r = s_rotated_pre_r AND s_mask_r
s_rotated_g = s_rotated_pre_g AND s_mask_r
s_rotated_b = s_rotated_pre_b AND s_mask_r
```

**Why split from Stage 1a:** Combining rotation and masking in one stage creates a path `data → rol mux → AND → register`. Splitting keeps each stage to a single operation. At 74.25 MHz this split was required to meet HD timing.

**Shared mask:** `s_mask_r` is applied to all three channels — bit depth is a global setting, not per-channel.

---

### Dual Dry Delay Line (T+3 → T+5 and T+9)
**Process:** `p_global_dry_delay`

A 6-element shift register (`C_PRE_GLOBAL_DELAY_CLKS = 6`) tapped at two points:

```
s_rgb_r/g/b (T+3) ──► [6-clock shift]
                            ├─ index 1 (T+5) ──► s_r/g/b_for_blend  (per-channel blend dry)
                            └─ index 5 (T+9) ──► s_r/g/b_for_global (global blend dry)
```

Note: index 1 (not 0) is used for the per-channel blend dry tap, because Stage 1a+1b adds 2 clocks (T+3→T+5), while the other RGB programs only add 1 clock (Stage 1 only, T+3→T+4). The tap index compensates for the extra masking stage.

---

### Stage 2 — Per-Channel Wet/Dry Blend (T+5 → T+9)
**Instances:** `interp_r`, `interp_g`, `interp_b` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_r/g/b_for_blend` | 5 | Original RGB, 2-clock tap |
| `b` (wet) | `s_rotated_r/g/b` | 5 | Rotated and masked result |
| `t` | `s_blend_r/g/b` | — | Per-channel blend (combinational, knobs 4–6) |
| `enable` | `s_rotated_valid` | — | Holds output stable during blanking |

---

### Stage 3 — Global Wet/Dry Blend (T+9 → T+13)
**Instances:** `interp_global_r`, `interp_global_g`, `interp_global_b` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_r/g/b_for_global` | 9 | Original RGB, 6-clock tap |
| `b` (wet) | `s_blended_r/g/b` | 9 | Per-channel blended result |
| `t` | `s_global_blend` | — | Global blend (combinational, slider) |
| `enable` | `s_blended_r/g/b_valid` | — | Holds output stable during blanking |

---

### Stages 4a-i → 4b — RGB→YUV Conversion (T+13 → T+17)

Identical structure to `rgb_bit_logic` Stages 4a-i → 4b, shifted 1 clock later due to the extra Stage 1b:

| Stage | Process | T+ | Operation |
|-------|---------|-----|-----------|
| 4a-i | `p_rgb_yuv_lut` | 14 | 7 BRAM reads + R/B passthrough |
| 4a-ii | `p_rgb_yuv_presum` | 15 | Pair sums |
| 4a-iii | `p_rgb_yuv_sum` | 16 | Channel sums + wire shifts |
| 4b | `p_rgb_yuv_acc` | 17 | clamp + convention swap |

---

### Bypass / Sync Delay Line (T+0 → T+17)
**Process:** `p_bypass_delay` — 17-clock shift register for sync signals and bypass pixel data.

---

## Signal Timing Summary

| Signal | T+ | Source |
|--------|----|--------|
| `data_in.y/u/v` | 0 | input port |
| `s_yr_r/gu/gv/b_off` | 1 | `p_yuv_rgb_lut` (BRAM) |
| `s_eff_shift_r/g/b`, `s_mask_r` | 1 | `p_yuv_rgb_lut` |
| `s_yr_g_presum`, `s_yr_r/b_off_d` | 2 | `p_yuv_rgb_presum` |
| `s_rgb_r/g/b`, `s_rgb_valid` | 3 | `p_yuv_rgb_acc` |
| `s_rotated_pre_r/g/b` | 4 | `p_rotation_stage` |
| `s_rotated_r/g/b` | 5 | `p_mask_stage` |
| `s_r/g/b_for_blend` | 5 | `p_global_dry_delay` (index 1) |
| `s_blended_r/g/b` | 9 | `interp_r/g/b` |
| `s_r/g/b_for_global` | 9 | `p_global_dry_delay` (index 5) |
| `s_global_r/g/b` | 13 | `interp_global_r/g/b` |
| `s_4a_ry_r/g/b`, etc. | 14 | `p_rgb_yuv_lut` |
| `s_4b_ry_rg`, etc. | 15 | `p_rgb_yuv_presum` |
| `s_ry_y/u/v_sum` | 16 | `p_rgb_yuv_sum` |
| `s_yuv_out_y/u/v` | 17 | `p_rgb_yuv_acc` |
| `data_out.hsync_n/vsync_n/field_n` | 17 | `p_bypass_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | R rotation amount | Knob 1 |
| `registers_in(1)` | G rotation amount | Knob 2 |
| `registers_in(2)` | B rotation amount | Knob 3 |
| `registers_in(3)` | R channel blend | Knob 4 |
| `registers_in(4)` | G channel blend | Knob 5 |
| `registers_in(5)` | B channel blend | Knob 6 |
| `registers_in(6)(0)` | Direction (0=ROL, 1=ROR) | S1 |
| `registers_in(6)(1)` | Bit depth LSB | S2 |
| `registers_in(6)(2)` | Bit depth mid | S3 |
| `registers_in(6)(3)` | Bit depth MSB | S4 |
| `registers_in(6)(4)` | Bypass | S5 |
| `registers_in(7)` | Global blend | Slider |

**Bit depth encoding** — same as `yuv_bit_rotator`. S4 (MSB) has the largest effect.

---

## Key Design Decisions

**Direction folded into shift (`to_eff_shift`)** — ROR by k positions is equivalent to ROL by (10−k) positions. Pre-computing the effective ROL shift in Stage 0a means Stage 1a needs only a 10-way ROL case statement rather than a 20-way ROL/ROR one. This saves approximately 1 LUT level on the Stage 1a critical path — significant when every level counts toward meeting 74.25 MHz.

**Rotation and masking split into two stages (1a and 1b)** — in the YUV version these are combined into a single stage (`data → rol/ror → AND mask → register`). In the RGB version, the colour conversion overhead means there are more stages in total, but the device is more congested (31/32 BRAMs used), so routing pressure is higher. Splitting the rotation and masking gives the router a cleaner signal at the stage boundary and was necessary to meet HD timing.

**Shared depth mask across channels** — `s_mask_r` is applied to all three channels in Stage 1b. Unlike the per-channel approach of `yuv_bit_rotator` (where each channel could theoretically have its own mask), bit depth is a global setting here. This saves registers and simplifies Stage 1b to three independent AND operations.

**Dry tap at index 1 (not 0)** — `s_r/g/b_for_blend` is taken from `p_global_dry_delay` at index 1 rather than index 0, because Stage 1a+1b adds 2 clocks instead of 1. The global dry tap at index 5 aligns with `s_blended` at T+9 (4 clocks after T+5). Both taps are from the same 6-clock shift register starting from `s_rgb_r/g/b` at T+3.

**`C_PROCESSING_DELAY_CLKS = 17`** — one more than the other 16-clock RGB programs (`rgb_bit_logic`, `rgb_bit_crush`) due to the extra Stage 1b masking clock. The bypass and sync delay lines are both 17 clocks.
