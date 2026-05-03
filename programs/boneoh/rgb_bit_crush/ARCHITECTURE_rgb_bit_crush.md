# RGB Bit Crush — Architecture

This document describes the signal flow and pipeline architecture of `rgb_bit_crush.vhd` for developers. Signal names match the VHDL exactly.

---

## Pipeline Overview

Total latency: **16 clock cycles** at 74.25 MHz.

```
T+0  data_in.y/u/v ──────────────────────────── bypass delay (16 clocks) ──► data_out (bypass)
     registers_in[0..7]                          sync delay  (16 clocks) ──► data_out.hsync_n/vsync_n/field_n
         │
         ▼
    ┌──────────┐  1 clk    YUV→RGB (1/3): BRAM lookup
    │ Stage 0a │  4 channel offsets + control decode (crush/round/invert)
    └────┬─────┘
         │  s_yr_r/gu/gv/b_off, s_yr_y/avid/u_raw/v_raw
         │  s_crush_r/g/b, s_round_r/g/b, s_invert
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
    ┌──────────┐  1 clk    bit crush operation
    │ Stage 1  │  apply_crush(R/G/B) + optional invert
    └────┬─────┘
         │  s_processed_r/g/b, s_processed_valid
         │  [dry: s_r/g/b_for_blend (T+4) tapped from p_global_dry_delay index 0]
         │
    ┌────┴──────────────────────────────────────────────────────┐
    │  [WET]                                    [DRY]           │
    │  s_processed_r/g/b (T+4)       s_r/g/b_for_blend (T+4)  │
    │  s_blend_r/g/b (combinational)                            │
    ▼                                           ▼              │
    ┌──────────────────────────────────────────────────────┐   │
    │  Stage 2: 3x interpolator_u (per-channel blend)      │   │
    │  interp_r / interp_g / interp_b  (4 clocks)          │   │
    └──────────────────────┬───────────────────────────────┘   │
                           │  s_blended_r/g/b (T+8)            │
                           │                                    │
    ┌──────────────────────┼────────────────────────────────────┘
    │  [WET]               │              [DRY]
    │  s_blended_r/g/b (T+8)    s_r/g/b_for_global (T+8)
    │  s_global_blend (combinational)      │
    │                                      │ p_global_dry_delay
    ▼                                      ▼ (5-clock tap from s_rgb at T+3)
    ┌──────────────────────────────────────────────────────┐
    │  Stage 3: 3x interpolator_u (global blend)  4 clocks │
    │  interp_global_r / interp_global_g / interp_global_b │
    └──────────────────────┬───────────────────────────────┘
                           │  s_global_r/g/b (T+12)
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

**Bypass path** (S5 On): `s_y/u/v_delayed` (16-clock shift) → `data_out.y/u/v` directly.

---

## BT.601 Colour Conversion

Both paths use BRAM tables from `rgb_yuv_tables_pkg.vhd` (11 BRAMs, 31/32 blocks used). Identical structure to `rgb_bit_logic` — see that document for full detail on the UV convention swap and BRAM arithmetic isolation rules.

---

## Stage Detail

### Stage 0a — YUV→RGB BRAM Lookup + Control Decode (T+0 → T+1)
**Process:** `p_yuv_rgb_lut`

**BRAM lookups (U/V swapped for convention):** identical to `rgb_bit_logic` Stage 0a — 4 tables producing `s_yr_r/gu/gv/b_off`.

**Control decode (parallel with BRAM, no added latency):**

| Signal | Source | Description |
|--------|--------|-------------|
| `s_crush_r/g/b` | `knob_to_crush(registers_in(0..2))` | Step index 0–7 per channel |
| `s_invert` | `registers_in(6)(0)` | `'1'` = On/Invert output |
| `s_round_r/g/b` | `registers_in(6)(1..3)` | `'1'` = On/Round per channel |

**Switch polarity note:** This program uses `Off='1', On='0'` in hardware but the register decode normalises to `'1'`=active in software — `s_invert='1'` means invert is active, `s_round_r='1'` means round is active. The TOML comment `Off='1', On='0'` reflects the raw hardware pull-up polarity; the VHD decode inverts this so the rest of the code uses the conventional `'1'`=On sense.

**`knob_to_crush` mapping:**
```
step_idx = knob / 128   (8 equal bands, integer divide)
```

| `step_idx` | Step size | Round offset | Max output |
|-----------|-----------|-------------|-----------|
| 0 | 8 | 4 | 1016 |
| 1 | 16 | 8 | 1008 |
| 2 | 32 | 16 | 992 |
| 3 | 48 | 24 | 1008 |
| 4 | 64 | 32 | 960 |
| 5 | 96 | 48 | 960 |
| 6 | 128 | 64 | 896 |
| 7 | 256 | 128 | 768 |

---

### Stage 0a-ii — G-channel Pre-Sum (T+1 → T+2)
**Process:** `p_yuv_rgb_presum`

`s_yr_g_presum = s_yr_gu_off + s_yr_gv_off` — identical to `rgb_bit_logic`.

---

### Stage 0b — YUV→RGB Accumulate and Clamp (T+2 → T+3)
**Process:** `p_yuv_rgb_acc`

Identical to `rgb_bit_logic`. Outputs `s_rgb_r/g/b`, `s_rgb_valid`.

---

### Stage 1 — Bit Crush Operation (T+3 → T+4)
**Process:** `p_bit_crush`

**Inputs (Stage 0a/0b registered outputs, T+3):**
`s_rgb_r/g/b`, `s_rgb_valid`, `s_crush_r/g/b`, `s_round_r/g/b`, `s_invert`

**Outputs (T+4):** `s_processed_r/g/b`, `s_processed_valid`

**`apply_crush(pixel, step_idx, do_round)` function:**

```
1. Select offset:
   do_round = '1' → offset = step/2  (round to nearest)
   do_round = '0' → offset = 0       (truncate/floor)

2. v_pix = pixel + offset  (11-bit, max 1023+128=1151)

3. Quantise:
   Power-of-2 steps (8,16,32,64,128,256):
     if v_pix[10]='1' → saturate to largest step multiple ≤ 1023
     else → shift right to zero lower bits
   Non-power-of-2 steps (48, 96):
     case on v_pix[10:4] (step 48) or v_pix[10:5] (step 96)
     returns pre-computed multiple directly — no multiply inferred
```

Unlike `yuv_bit_crush`, this version folds the offset directly into `apply_crush` rather than pre-registering `lmask` and `roundoff`. This is simpler but puts the round offset add inside Stage 1 rather than Stage 0a. At 16 clocks total latency, the timing budget is more relaxed.

**Invert (applied after crush):**
```
if s_invert = '1': s_processed_r/g/b = NOT v_r/g/b
else:              s_processed_r/g/b = v_r/g/b
```

---

### Dual Dry Delay Line (T+3 → T+4 and T+8)
**Process:** `p_global_dry_delay`

Identical structure to `rgb_bit_logic` — single 5-element shift register tapped at two points:

```
s_rgb_r/g/b (T+3) ──► [5-clock shift]
                            ├─ index 0 (T+4) ──► s_r/g/b_for_blend  (per-channel dry)
                            └─ index 4 (T+8) ──► s_r/g/b_for_global (global dry)
```

---

### Stage 2 — Per-Channel Wet/Dry Blend (T+4 → T+8)
**Instances:** `interp_r`, `interp_g`, `interp_b` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_r/g/b_for_blend` | 4 | Original RGB, 1-clock tap |
| `b` (wet) | `s_processed_r/g/b` | 4 | Crushed result |
| `t` | `s_blend_r/g/b` | — | Per-channel blend (combinational, knobs 4–6) |
| `enable` | `s_processed_valid` | — | Holds output stable during blanking |

---

### Stage 3 — Global Wet/Dry Blend (T+8 → T+12)
**Instances:** `interp_global_r`, `interp_global_g`, `interp_global_b` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_r/g/b_for_global` | 8 | Original RGB, 5-clock tap |
| `b` (wet) | `s_blended_r/g/b` | 8 | Per-channel blended result |
| `t` | `s_global_blend` | — | Global blend (combinational, slider) |
| `enable` | `s_blended_r/g/b_valid` | — | Holds output stable during blanking |

---

### Stages 4a-i → 4b — RGB→YUV Conversion (T+12 → T+16)

Identical to `rgb_bit_logic` Stages 4a-i → 4b. See that document for full detail.

| Stage | Process | T+ | Operation |
|-------|---------|-----|-----------|
| 4a-i | `p_rgb_yuv_lut` | 13 | 7 BRAM reads + R/B passthrough |
| 4a-ii | `p_rgb_yuv_presum` | 14 | Pair sums |
| 4a-iii | `p_rgb_yuv_sum` | 15 | Channel sums + wire shifts |
| 4b | `p_rgb_yuv_acc` | 16 | clamp + convention swap |

---

### Bypass / Sync Delay Line (T+0 → T+16)
**Process:** `p_bypass_delay` — identical to `rgb_bit_logic`.

---

## Signal Timing Summary

| Signal | T+ | Source |
|--------|----|--------|
| `data_in.y/u/v` | 0 | input port |
| `s_yr_r/gu/gv/b_off` | 1 | `p_yuv_rgb_lut` (BRAM) |
| `s_crush_r/g/b`, `s_round_r/g/b`, `s_invert` | 1 | `p_yuv_rgb_lut` |
| `s_yr_g_presum`, `s_yr_r/b_off_d` | 2 | `p_yuv_rgb_presum` |
| `s_rgb_r/g/b`, `s_rgb_valid` | 3 | `p_yuv_rgb_acc` |
| `s_processed_r/g/b` | 4 | `p_bit_crush` |
| `s_r/g/b_for_blend` | 4 | `p_global_dry_delay` (index 0) |
| `s_blended_r/g/b` | 8 | `interp_r/g/b` |
| `s_r/g/b_for_global` | 8 | `p_global_dry_delay` (index 4) |
| `s_global_r/g/b` | 12 | `interp_global_r/g/b` |
| `s_4a_ry_r/g/b`, etc. | 13 | `p_rgb_yuv_lut` |
| `s_4b_ry_rg`, etc. | 14 | `p_rgb_yuv_presum` |
| `s_ry_y/u/v_sum` | 15 | `p_rgb_yuv_sum` |
| `s_yuv_out_y/u/v` | 16 | `p_rgb_yuv_acc` |
| `data_out.hsync_n/vsync_n/field_n` | 16 | `p_bypass_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | R crush amount | Knob 1 |
| `registers_in(1)` | G crush amount | Knob 2 |
| `registers_in(2)` | B crush amount | Knob 3 |
| `registers_in(3)` | R channel blend | Knob 4 |
| `registers_in(4)` | G channel blend | Knob 5 |
| `registers_in(5)` | B channel blend | Knob 6 |
| `registers_in(6)(0)` | Invert (On='1') | S1 |
| `registers_in(6)(1)` | R Round (On='1') | S2 |
| `registers_in(6)(2)` | G Round (On='1') | S3 |
| `registers_in(6)(3)` | B Round (On='1') | S4 |
| `registers_in(6)(4)` | Bypass | S5 |
| `registers_in(7)` | Global blend | Slider |

---

## Comparison with YUV Bit Crush

`rgb_bit_crush` and `yuv_bit_crush` implement the same crushing algorithm but differ in several ways:

| Aspect | `rgb_bit_crush` | `yuv_bit_crush` |
|--------|----------------|----------------|
| Colour space | RGB (with YUV↔RGB conversion) | YUV direct |
| Pipeline latency | 16 clocks | 10 clocks |
| BRAM usage | 31/32 | 0/32 |
| Round granularity | Per-channel R/G/B | Per-channel Y/U/V |
| Dither | Not available | Available (LFSR/PRNG on U/V) |
| `apply_crush` design | Offset folded inline | lmask/roundoff pre-registered in Stage 0a |
| Invert | Global (all 3 channels) | Global (all 3 channels) |

The key architectural difference: `rgb_bit_crush` folds the round offset directly into `apply_crush` (simpler, offset add is in Stage 1), while `yuv_bit_crush` pre-registers `lmask` and `roundoff` in Stage 0a to keep Stage 0b's critical path clean. The RGB version can afford the simpler approach because its 16-clock pipeline has a more relaxed timing budget than the YUV version's 10-clock budget.

## Key Design Decisions

**Inline offset vs pre-registered** — the round offset (`step/2`) is computed inside `apply_crush` in Stage 1, not pre-registered in Stage 0a as in `yuv_bit_crush`. This works at 16 clocks because the overall timing margin is greater. For tighter pipelines, pre-registering intermediate values is necessary.

**No dither** — `rgb_bit_crush` omits RPDF dither (available in `yuv_bit_crush`). RGB bit crushing is typically used for a coarser, more deliberate effect where dither would be counterproductive, and the absence of LFSR modules saves a small amount of routing pressure on an already-dense design (31/32 BRAMs used).

**Per-channel round switches** — R, G, and B each have an independent Round switch (S2, S3, S4). This allows, for example, rounding R and B while truncating G, giving subtle colour-cast control over the quantisation character of each channel independently.

**Colour conversion overhead** — the 4-stage YUV→RGB and 4-stage RGB→YUV paths add 8 clock cycles of latency vs the YUV version (10 vs 16 clocks). The 11 BRAMs used for the conversion tables consume 97% of available block RAM, leaving only 1/32 spare.
