# YUV Bit Logic — Architecture

This document describes the signal flow and pipeline architecture of `yuv_bit_logic.vhd` for developers. Signal names match the VHDL exactly.

---

## Pipeline Overview

Total latency: **10 clock cycles** at 74.25 MHz.

```
T+0  data_in.y/u/v ───────────────────────────────────── bypass delay (10 clocks) ──► data_out (bypass)
     registers_in[0..7]                                   sync delay  (10 clocks) ──► data_out.hsync_n/vsync_n/field_n
         │
         ▼
    ┌─────────┐  1 clk
    │ Stage 0a│  control decode
    │         │  LFSR reset logic
    └────┬────┘
         │  s_mask_y/u/v_r, s_operator_r, s_invert_mask_r
         │  s_y_d1, s_u_d1, s_v_d1, s_avid_d1
         │  s_lfsr_reset
         ▼
    ┌─────────┐  1 clk
    │ Stage 0b│  bit logic operation (AND/OR/XOR/NAND/NOR/NXOR/LFSR/PRNG)
    └────┬────┘
         │  s_processed_y/u/v, s_processed_valid
         │  s_orig_y/u/v_d2   (2-clock delayed originals, for per-channel blend dry)
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

**Bypass path** (S5 On): `s_y/u/v_delayed` (10-clock shift) → `data_out.y/u/v` directly, bypassing all processing.

---

## Stage Detail

### Stage 0a — Control Decode (T+0 → T+1)
**Process:** `p_control_decode`

**Inputs (T+0):**
- `data_in.y`, `data_in.u`, `data_in.v`, `data_in.avid`, `data_in.vsync_n`
- `registers_in(0..2)` — Y/U/V bit masks (knobs 1–3)
- `registers_in(6)(0)` — Invert/Seed (S1)
- `registers_in(6)(1..3)` — Operator S2/S3/S4
- `registers_in(6)(4)` — Bypass (S5)

**Outputs (T+1):**

| Signal | Description |
|--------|-------------|
| `s_mask_y_r`, `s_mask_u_r`, `s_mask_v_r` | Registered 10-bit masks (knobs 1–3) |
| `s_operator_r` | `{reg(6)(1), reg(6)(2), reg(6)(3)}` — S2=MSB, S4=LSB |
| `s_invert_mask_r` | `registers_in(6)(0)` — invert masks for ops 0–5 |
| `s_y_d1`, `s_u_d1`, `s_v_d1`, `s_avid_d1` | Registered pixel data |
| `s_vsync_n_prev` | Previous vsync_n (for edge detect) |
| `s_lfsr_reset` | One-clock pulse: fires at vsync falling edge when op=LFSR and S1=Off |

**LFSR reset condition:**
```
s_lfsr_reset = '1' when:
  vsync falling edge (vsync_n='0' AND s_vsync_n_prev='1')
  AND operator = LFSR ("110": reg(6)(1)='1', reg(6)(2)='1', reg(6)(3)='0')
  AND S1 = Off (reg(6)(0)='1' — Off='1' means vsync-reseed mode)
```

Note: S1 polarity for LFSR mode: Off (='1') = reseed at vsync; On (='0') = free-run.

---

### Stage 0b — Bit Logic Operation (T+1 → T+2)
**Process:** `p_bit_logic`

**Inputs (Stage 0a registered outputs, T+1):**
`s_mask_y/u/v_r`, `s_operator_r`, `s_invert_mask_r`, `s_y_d1/u_d1/v_d1`, `s_avid_d1`, `s_lfsr10_out`, `s_lfsr16_out`

**Outputs (T+2):**

| Signal | Description |
|--------|-------------|
| `s_processed_y/u/v` | Result of bit logic operation |
| `s_processed_valid` | `s_avid_d1` (mirrors avid, 1 clock delayed) |
| `s_orig_y/u/v_d2` | `s_y/u/v_d1` re-registered — dry input for per-channel blend |

**Operator case (`case s_operator_r`):**

| `s_operator_r` | Operation | Formula |
|----------------|-----------|---------|
| `"000"` | AND | `pixel AND mask` |
| `"001"` | OR | `pixel OR mask` |
| `"010"` | XOR | `pixel XOR mask` |
| `"011"` | NAND | `NOT(pixel AND mask)` |
| `"100"` | NOR | `NOT(pixel OR mask)` |
| `"101"` | NXOR | `NOT(pixel XOR mask)` |
| `"110"` | LFSR | `pixel XOR (lfsr10_out AND mask)` |
| `"111"` | PRNG | `pixel XOR (lfsr16_out[9:0] AND mask)` |

For ops 0–5, mask is optionally inverted: `if s_invert_mask_r='1' then mask := NOT mask`.

Applied per-channel independently using `apply_logic()` function (ops 0–5) or inline XOR (ops 6–7).

---

### LFSR / PRNG Noise Generators (free-running, every clock)

**Modules:** `u_lfsr16` (SDK), `u_lfsr10` (SDK)

```
s_lfsr_reset ──────────────────────────────────────────► u_lfsr10.reset
u_lfsr16 ──────────────────────────────────────────────► u_lfsr10.seed[9:1] & '1'
u_lfsr16 ──► s_lfsr16_out[9:0] ──► Stage 0b (PRNG: pixel XOR rand AND mask)
u_lfsr10 ──► s_lfsr10_out[9:0] ──► Stage 0b (LFSR: pixel XOR rand AND mask)
```

- `u_lfsr16` — free-runs always, period 65535, never reseeded. PRNG source (`"111"`).
- `u_lfsr10` — polynomial x¹⁰ + x⁷ + 1, period 1023. Conditionally reseeds from `lfsr16_out[9:0]` on vsync falling edge when op=LFSR and S1=Off. Seed bit 0 forced `'1'` to prevent all-zeros lockup.

**S1 dual behaviour:**
- Ops `"000"`–`"101"`: S1 Off = normal mask, S1 On = invert all masks before applying
- Op `"110"` (LFSR): S1 Off = reseed at vsync (frame-locked), S1 On = free-run
- Op `"111"` (PRNG): S1 has no effect (PRNG always free-runs via lfsr16)

---

### Stage 1 — Per-Channel Wet/Dry Blend (T+2 → T+6)
**Instances:** `interp_y`, `interp_u`, `interp_v` (SDK `interpolator_u`, 4 clocks)

`result = a + (b - a) × t / 2^10`

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_orig_y/u/v_d2` | 2 | Original pixel, 2-clock delayed |
| `b` (wet) | `s_processed_y/u/v` | 2 | Bit logic result |
| `t` | `s_blend_y/u/v` | — | Per-channel blend factor (combinational from knobs 4–6) |
| `enable` | `s_processed_valid` | — | Holds output stable during blanking |

Each channel has an **independent** blend knob — Y, U, and V can be mixed at different ratios simultaneously.

---

### Global Dry Delay Line (T+0 → T+6)
**Process:** `p_global_dry_delay`

`data_in.y/u/v` shifted through a 6-element array (`C_PRE_GLOBAL_DELAY_CLKS = 6`):
```
data_in.y/u/v ──► [6-clock shift] ──► s_y/u/v_for_global (T+6)
```
Aligned with `s_blended_y/u/v` (T+6) for the global blend interpolator `a` (dry) input.

---

### Stage 2 — Global Wet/Dry Blend (T+6 → T+10)
**Instances:** `interp_global_y`, `interp_global_u`, `interp_global_v` (SDK `interpolator_u`, 4 clocks)

`result = a + (b - a) × t / 2^10`

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_y/u/v_for_global` | 6 | Original pixel, 6-clock delayed |
| `b` (wet) | `s_blended_y/u/v` | 6 | Per-channel blended result |
| `t` | `s_global_blend` | — | Global blend factor (combinational from slider) |
| `enable` | `s_blended_y/u/v_valid` | — | Holds output stable during blanking |

Note: `s_global_blend` feeds the global interpolators combinationally (no extra register). The three instances share the same `t` input; fan-out is manageable here since the global blend is the only user.

---

### Bypass / Sync Delay Line (T+0 → T+10)
**Process:** `p_bypass_delay`

A single 10-element shift register handles both sync signals and bypass pixel data:

```
data_in.hsync_n ──► [10-clock shift] ──► data_out.hsync_n (always)
data_in.vsync_n ──► [10-clock shift] ──► data_out.vsync_n (always)
data_in.field_n ──► [10-clock shift] ──► data_out.field_n (always)
data_in.y       ──► [10-clock shift] ──► s_y_delayed  ──► data_out.y  (bypass only)
data_in.u       ──► [10-clock shift] ──► s_u_delayed  ──► data_out.u  (bypass only)
data_in.v       ──► [10-clock shift] ──► s_v_delayed  ──► data_out.v  (bypass only)
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
| `s_mask_y/u/v_r` | 1 | `p_control_decode` |
| `s_operator_r`, `s_invert_mask_r` | 1 | `p_control_decode` |
| `s_y_d1`, `s_u_d1`, `s_v_d1` | 1 | `p_control_decode` |
| `s_lfsr_reset` | 1 | `p_control_decode` |
| `s_processed_y/u/v` | 2 | `p_bit_logic` |
| `s_orig_y/u/v_d2` | 2 | `p_bit_logic` |
| `s_blended_y/u/v` | 6 | `interp_y/u/v` |
| `s_y/u/v_for_global` | 6 | `p_global_dry_delay` |
| `s_global_y/u/v` | 10 | `interp_global_y/u/v` |
| `data_out.hsync_n/vsync_n/field_n` | 10 | `p_bypass_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | Y bit mask | Knob 1 |
| `registers_in(1)` | U bit mask | Knob 2 |
| `registers_in(2)` | V bit mask | Knob 3 |
| `registers_in(3)` | Y channel blend | Knob 4 |
| `registers_in(4)` | U channel blend | Knob 5 |
| `registers_in(5)` | V channel blend | Knob 6 |
| `registers_in(6)(0)` | Invert/Seed (S1) | S1 |
| `registers_in(6)(1)` | Op Bit 2 (MSB) | S2 |
| `registers_in(6)(2)` | Op Bit 1 | S3 |
| `registers_in(6)(3)` | Op Bit 0 (LSB) | S4 |
| `registers_in(6)(4)` | Bypass | S5 |
| `registers_in(7)` | Global blend | Slider |

**Operator encoding** (`s_operator_r` = `{S2, S3, S4}`):

| `s_operator_r` | S2 | S3 | S4 | Operation |
|----------------|----|----|----|-----------|
| `"000"` | 0 | 0 | 0 | AND |
| `"001"` | 0 | 0 | 1 | OR |
| `"010"` | 0 | 1 | 0 | XOR |
| `"011"` | 0 | 1 | 1 | NAND |
| `"100"` | 1 | 0 | 0 | NOR |
| `"101"` | 1 | 0 | 1 | NXOR |
| `"110"` | 1 | 1 | 0 | LFSR |
| `"111"` | 1 | 1 | 1 | PRNG |

---

## Key Design Decisions

**Two blend stages** — per-channel blend (Stage 1) plus global blend (Stage 2) gives independent control of how much each channel is affected before the overall wet/dry mix. The per-channel knobs set the artistic balance between Y, U, and V; the slider then scales the whole result.

**Dry tap for per-channel blend** — `s_orig_y/u/v_d2` is computed by re-registering `s_y/u/v_d1` inside `p_bit_logic`. This puts the dry and wet inputs both at T+2 for the per-channel interpolators without a separate delay process.

**Global blend fan-out** — `s_global_blend` feeds the three global interpolators combinationally (no extra register, unlike `yuv_band_filter`). This is safe here because the three interpolators are the only consumers and there is no history of timing failures on this fan-out path.

**LFSR reset gating** — `s_lfsr_reset` is registered in Stage 0a (not generated combinationally) to eliminate glitches on `registers_in` during vsync. The mode check (op=LFSR, S1=Off) is also done here in the same registered process.

**S1 dual role** — S1 serves two completely different functions depending on the operator. For ops 0–5 it inverts masks; for op 6 (LFSR) it selects vsync-seed vs free-run. This is documented in the TOML and README but is easy to miss when reading the VHD — the `p_control_decode` process handles both behaviours from the same `registers_in(6)(0)` bit.

**Bypass path** — `p_bypass_delay` shifts both sync signals and pixel data through the same 10-clock array. The output mux is purely combinational. `data_out.avid` always comes from `s_global_y_valid` regardless of bypass state, which means avid timing is always pipeline-derived.
