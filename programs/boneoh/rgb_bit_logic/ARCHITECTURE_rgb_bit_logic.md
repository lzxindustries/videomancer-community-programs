# RGB Bit Logic — Architecture

This document describes the signal flow and pipeline architecture of `rgb_bit_logic.vhd` for developers. Signal names match the VHDL exactly.

---

## Pipeline Overview

Total latency: **16 clock cycles** at 74.25 MHz.

```
T+0  data_in.y/u/v ──────────────────────────── bypass delay (16 clocks) ──► data_out (bypass)
     registers_in[0..7]                          sync delay  (16 clocks) ──► data_out.hsync_n/vsync_n/field_n
         │
         ├──────────────────────────────────────────── SDK LFSR/PRNG modules (free-running)
         │
         ▼
    ┌──────────┐  1 clk    YUV→RGB (1/3): BRAM lookup
    │ Stage 0a │  4 channel offsets + control decode + LFSR reset logic
    └────┬─────┘
         │  s_yr_r/gu/gv/b_off, s_yr_y/avid/u_raw/v_raw
         │  s_mask_r/g/b, s_operator, s_invert_mask, s_lfsr_reset
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
    ┌──────────┐  1 clk    bit logic operation
    │ Stage 1  │  AND/OR/XOR/NAND/NOR/NXOR/LFSR/PRNG per channel
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

Both conversion paths use BRAM tables from `rgb_yuv_tables_pkg.vhd` (11 BRAMs, 31/32 blocks used). Tables store pre-divided partial values to eliminate multipliers. The Videomancer UV convention (`u=Cr, v=Cb`) is swapped at both input and output — see `rgb_band_filter` ARCHITECTURE.md for the full explanation.

---

## Stage Detail

### Stage 0a — YUV→RGB BRAM Lookup + Control Decode (T+0 → T+1)
**Process:** `p_yuv_rgb_lut`

**BRAM lookups (U/V swapped for convention):**

| Signal | Table | Index | Description |
|--------|-------|-------|-------------|
| `s_yr_r_off` | `s_lut_yr_r` | `data_in.u` (=Cr) | R chroma offset |
| `s_yr_gu_off` | `s_lut_yr_gu` | `data_in.v` (=Cb) | G U-component offset |
| `s_yr_gv_off` | `s_lut_yr_gv` | `data_in.u` (=Cr) | G V-component offset |
| `s_yr_b_off` | `s_lut_yr_b` | `data_in.v` (=Cb) | B chroma offset |

**Control decode (parallel with BRAM, no added latency):**

| Signal | Source | Description |
|--------|--------|-------------|
| `s_mask_r/g/b` | `registers_in(0..2)` | 10-bit per-channel masks |
| `s_operator` | `{reg(6)(1),(2),(3)}` | Operator S2=MSB, S4=LSB |
| `s_invert_mask` | `registers_in(6)(0)` | Invert/Seed (S1) |
| `s_vsync_n_prev` | `data_in.vsync_n` | Previous vsync for edge detect |
| `s_lfsr_reset` | edge detect + mode check | One-clock pulse for lfsr10 reseed |

**LFSR reset condition (same as `yuv_bit_logic`):**
```
s_lfsr_reset = '1' when:
  vsync falling edge AND op="110" (LFSR) AND reg(6)(0)='1' (S1=Off = vsync-reseed)
```

---

### Stage 0a-ii — G-channel Pre-Sum (T+1 → T+2)
**Process:** `p_yuv_rgb_presum`

`s_yr_g_presum = s_yr_gu_off + s_yr_gv_off`

Balances Stage 0b to one addition per channel. Required for BRAM inference — arithmetic cannot be mixed with BRAM read expressions.

---

### Stage 0b — YUV→RGB Accumulate and Clamp (T+2 → T+3)
**Process:** `p_yuv_rgb_acc`

```
R = clamp10(Y + s_yr_r_off_d)
G = clamp10(Y + s_yr_g_presum)
B = clamp10(Y + s_yr_b_off_d)
```
Blanking passthrough: when `avid='0'`, raw YUV passed as-is.

**Outputs:** `s_rgb_r`, `s_rgb_g`, `s_rgb_b`, `s_rgb_valid`

---

### LFSR / PRNG Modules (free-running, every clock)

**Modules:** `u_lfsr16` (SDK), `u_lfsr10` (SDK)

```
s_lfsr_reset ──────────────────────────────────────────► u_lfsr10.reset
u_lfsr16 ──────────────────────────────────────────────► u_lfsr10.seed[9:1] & '1'
u_lfsr16 ──► s_lfsr16_out[9:0] ──► Stage 1 (PRNG: pixel XOR rand AND mask)
u_lfsr10 ──► s_lfsr10_out[9:0] ──► Stage 1 (LFSR: pixel XOR rand AND mask)
```

- `u_lfsr16` — free-runs always, period 65535. PRNG source (`"111"`).
- `u_lfsr10` — polynomial x¹⁰ + x⁷ + 1, period 1023. Conditionally reseeds at vsync when op=LFSR and S1=Off.

**S1 dual behaviour:** Same as `yuv_bit_logic` — for ops 0–5 S1 inverts masks; for op 6 (LFSR) S1 selects vsync-seed vs free-run; for op 7 (PRNG) S1 has no effect.

---

### Stage 1 — Bit Logic Operation (T+3 → T+4)
**Process:** `p_bit_logic`

**Inputs (Stage 0b + Stage 0a registered outputs, T+3):**
`s_rgb_r/g/b`, `s_rgb_valid`, `s_mask_r/g/b`, `s_operator`, `s_invert_mask`, `s_lfsr10_out`, `s_lfsr16_out`

**Outputs (T+4):** `s_processed_r/g/b`, `s_processed_valid`

**Operator case (`case s_operator`):**

| `s_operator` | Operation | Formula |
|--------------|-----------|---------|
| `"000"` | AND | `pixel AND mask` |
| `"001"` | OR | `pixel OR mask` |
| `"010"` | XOR | `pixel XOR mask` |
| `"011"` | NAND | `NOT(pixel AND mask)` |
| `"100"` | NOR | `NOT(pixel OR mask)` |
| `"101"` | NXOR | `NOT(pixel XOR mask)` |
| `"110"` | LFSR | `pixel XOR (lfsr10_out AND mask)` |
| `"111"` | PRNG | `pixel XOR (lfsr16_out[9:0] AND mask)` |

For ops 0–5: mask is optionally inverted when `s_invert_mask='1'`.
Applied per-channel independently using `apply_logic()` for ops 0–5, inline XOR for ops 6–7.

---

### Dual Dry Delay Line (T+3 → T+4 and T+8)
**Process:** `p_global_dry_delay`

A single 5-element shift register (`C_PRE_GLOBAL_DELAY_CLKS = 5`) tapped at two points, serving both blend stages from one process:

```
s_rgb_r/g/b (T+3) ──► [5-clock shift register]
                            │
                            ├─ index 0 (T+4) ──► s_r/g/b_for_blend  (per-channel blend dry)
                            └─ index 4 (T+8) ──► s_r/g/b_for_global (global blend dry)
```

This eliminates two separate delay processes and keeps both dry taps from the same registered source.

---

### Stage 2 — Per-Channel Wet/Dry Blend (T+4 → T+8)
**Instances:** `interp_r`, `interp_g`, `interp_b` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_r/g/b_for_blend` | 4 | Original RGB, 1-clock tap |
| `b` (wet) | `s_processed_r/g/b` | 4 | Bit logic result |
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

Identical structure to `rgb_band_filter` Stage 3a-i → 3b. See that document for full detail. Summary:

| Stage | Process | T+ | Operation |
|-------|---------|-----|-----------|
| 4a-i | `p_rgb_yuv_lut` | 13 | 7 BRAM reads (ry_r/g/b, ru_r/g, rv_g/b) + R/B passthrough |
| 4a-ii | `p_rgb_yuv_presum` | 14 | Pair sums: ry_rg, ru_rg, rv_gb |
| 4a-iii | `p_rgb_yuv_sum` | 15 | Channel sums + wire shifts (R>>1, B>>1) |
| 4b | `p_rgb_yuv_acc` | 16 | clamp10/clamp10_uv + Cr/Cb convention swap |

---

### Bypass / Sync Delay Line (T+0 → T+16)
**Process:** `p_bypass_delay`

```
data_in.hsync_n/vsync_n/field_n ──► [16-clock shift] ──► data_out.hsync_n/vsync_n/field_n
data_in.y/u/v                   ──► [16-clock shift] ──► s_y/u/v_delayed (bypass only)
```

**Output mux (combinational):**
```
s_bypass_enable = '0': data_out.y/u/v = s_yuv_out_y/u/v
s_bypass_enable = '1': data_out.y/u/v = s_y/u/v_delayed
data_out.avid always = s_yuv_out_valid
```

---

## Signal Timing Summary

| Signal | T+ | Source |
|--------|----|--------|
| `data_in.y/u/v` | 0 | input port |
| `s_yr_r/gu/gv/b_off` | 1 | `p_yuv_rgb_lut` (BRAM) |
| `s_mask_r/g/b`, `s_operator`, `s_invert_mask` | 1 | `p_yuv_rgb_lut` |
| `s_lfsr_reset` | 1 | `p_yuv_rgb_lut` |
| `s_yr_g_presum`, `s_yr_r/b_off_d` | 2 | `p_yuv_rgb_presum` |
| `s_rgb_r/g/b`, `s_rgb_valid` | 3 | `p_yuv_rgb_acc` |
| `s_processed_r/g/b` | 4 | `p_bit_logic` |
| `s_r/g/b_for_blend` | 4 | `p_global_dry_delay` (index 0) |
| `s_blended_r/g/b` | 8 | `interp_r/g/b` |
| `s_r/g/b_for_global` | 8 | `p_global_dry_delay` (index 4) |
| `s_global_r/g/b` | 12 | `interp_global_r/g/b` |
| `s_4a_ry_r/g/b`, `s_4a_ru_r/g`, `s_4a_rv_g/b` | 13 | `p_rgb_yuv_lut` |
| `s_4b_ry_rg`, `s_4b_ru_rg`, `s_4b_rv_gb` | 14 | `p_rgb_yuv_presum` |
| `s_ry_y/u/v_sum` | 15 | `p_rgb_yuv_sum` |
| `s_yuv_out_y/u/v` | 16 | `p_rgb_yuv_acc` |
| `data_out.hsync_n/vsync_n/field_n` | 16 | `p_bypass_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | R bit mask | Knob 1 |
| `registers_in(1)` | G bit mask | Knob 2 |
| `registers_in(2)` | B bit mask | Knob 3 |
| `registers_in(3)` | R channel blend | Knob 4 |
| `registers_in(4)` | G channel blend | Knob 5 |
| `registers_in(5)` | B channel blend | Knob 6 |
| `registers_in(6)(0)` | Invert/Seed (S1) | S1 |
| `registers_in(6)(1)` | Op Bit 2 (MSB) | S2 |
| `registers_in(6)(2)` | Op Bit 1 | S3 |
| `registers_in(6)(3)` | Op Bit 0 (LSB) | S4 |
| `registers_in(6)(4)` | Bypass | S5 |
| `registers_in(7)` | Global blend | Slider |

**Operator encoding** (`s_operator` = `{S2, S3, S4}`):

| `s_operator` | Operation |
|--------------|-----------|
| `"000"` | AND |
| `"001"` | OR |
| `"010"` | XOR |
| `"011"` | NAND |
| `"100"` | NOR |
| `"101"` | NXOR |
| `"110"` | LFSR |
| `"111"` | PRNG |

---

## Key Design Decisions

**Shared dual-tap delay line** — `p_global_dry_delay` uses a single 5-clock shift register tapped at index 0 (T+4, per-channel blend dry) and index 4 (T+8, global blend dry). This eliminates the two separate `s_orig_*_d1/d2` registers used in the YUV bit logic program and keeps both taps from the same source register.

**Control decode co-located with BRAM lookup** — masks, operator, invert flag, and LFSR reset are all registered in Stage 0a alongside the BRAM reads. Since BRAM reads are fast (no carry chains), the control decode carry chains (if any) complete within the same budget. This avoids a separate Stage 0a for controls.

**No luma multiply stage** — unlike `rgb_band_filter`, this program has no Stage 0c. The bit logic operation in Stage 1 only requires the converted RGB values, not BT.601 luma. The three pipeline stages after BRAM lookup (0a-ii, 0b, Stage 1) are sufficient without the extra multiply isolation stage.

**SDK LFSR modules** — unlike `rgb_band_filter` which uses hand-rolled shift registers, this program uses `lfsr16` and `lfsr10` SDK modules. The LFSR reseed condition checks both op=LFSR and S1 polarity, exactly matching `yuv_bit_logic`.

**Colour conversion is symmetric** — the YUV→RGB path (Stages 0a, 0a-ii, 0b) and the RGB→YUV path (Stages 4a-i, 4a-ii, 4a-iii, 4b) use identical table structures and arithmetic. The RGB→YUV path has one extra stage (4a-iii for channel sums) because it combines more partial products.
