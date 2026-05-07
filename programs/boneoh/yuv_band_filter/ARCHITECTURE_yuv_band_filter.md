# YUV Band Filter — Architecture

This document describes the signal flow and pipeline architecture of `yuv_band_filter.vhd` for developers. Signal names match the VHDL exactly.

---

## Pipeline Overview

Total latency: **9 clock cycles** at 74.25 MHz.

```
T+0  data_in.y/u/v ──── sync delay (9 clocks) ──► data_out.hsync_n/vsync_n/field_n
     registers_in[0..7]
         │
         ▼
    ┌─────────┐  1 clk
    │ Stage 0a│  fine_knob arithmetic (carry chains)
    │         │  control decode + pixel register
    └────┬────┘
         │  s_y_d1, s_u_d1, s_v_d1, s_avid_d1
         │  s_low_y/u/v, s_high_y/u/v
         │  s_show_matte, s_matte_mode
         ▼
    ┌─────────┐  1 clk
    │ Stage 0b│  9x window comparisons (carry chains)
    │         │  LFSR/PRNG noise register
    └────┬────┘
         │  s_pb_y/u/v_ge_low, s_pb_y/u/v_le_high, s_pb_lo_le_hi_y/u/v
         │  s_lfsr_d1, s_prng_d1
         │  s_show_matte_d1, s_matte_mode_d1
         ▼
    ┌─────────┐  1 clk
    │ Stage 1a│  window flag combination (pure LUT, no carry chains)
    │         │  masked value computation
    └────┬────┘
         │  s_wf_in_y/u/v, s_wf_in_any, s_wf_in_all
         │  s_wf_y_m, s_wf_u_m, s_wf_v_m
         │  s_wf_y, s_wf_u, s_wf_v
         │  s_wf_lfsr, s_wf_prng
         │  s_wf_show_matte, s_wf_matte_mode, s_wf_avid
         ▼
    ┌─────────┐  1 clk
    │ Stage 1b│  matte computation + output mux
    └────┬────┘
         │  s_processed_y/u/v, s_processed_valid
         │
    ┌────┴──────────────────────────────────────────────────┐
    │  [WET]                              [DRY]              │
    │  s_processed_y/u/v (T+4)  s_y/u/v_for_global (T+4)   │
    │                            s_global_blend_r   (T+4)   │◄── p_global_dry_delay
    │         │                           │                  │    (Stage 2r, 1 clk)
    ▼         ▼                           ▼                  │
    ┌──────────────────────────────────────────────────────┐ │
    │  Stage 2: 3x interpolator_u  (4 clocks)              │ │
    │  interp_global_y / interp_global_u / interp_global_v │ │
    └────────────────────────┬─────────────────────────────┘ │
                             │  s_global_y/u/v               │
                             ▼
                        data_out.y/u/v, data_out.avid
```

---

## Stage Detail

### Stage 0a — Fine Mode Arithmetic + Control Decode (T+0 → T+1)
**Process:** `p_control_decode`

**Inputs (T+0):**
- `data_in.y`, `data_in.u`, `data_in.v`, `data_in.avid`
- `registers_in(0..5)` — Y/U/V Low/High thresholds (knobs 1–6)
- `registers_in(6)(0)` — Show Matte (S1)
- `registers_in(6)(1..3)` — Matte mode bits S2/S3/S4
- `registers_in(6)(4)` — Fine (S5)
- `registers_in(7)` — Global blend → `s_global_blend` (combinational)

**Outputs (T+1):**

| Signal | Description |
|--------|-------------|
| `s_y_d1`, `s_u_d1`, `s_v_d1`, `s_avid_d1` | Registered pixel data |
| `s_low_y/u/v`, `s_high_y/u/v` | Computed thresholds (fine_knob or direct) |
| `s_show_matte` | `registers_in(6)(0)` registered |
| `s_matte_mode` | `{reg(6)(1), reg(6)(2), reg(6)(3)}` — S2=MSB, S4=LSB |
| `s_fine`, `s_in_ref(0..5)` | Fine mode state; refs latched on Normal→Fine edge |

**Key logic:**
- `fine_knob(raw, ref)` = `(raw + 7×ref) / 8` via shift arithmetic — no multiplier inferred
- Pixel registered here to isolate raw port fan-out from Stage 0b carry chains

**Timing note:** All `fine_knob` carry chains land here, isolated from Stage 0b.

---

### Stage 0b — Window Comparison Pre-compute (T+1 → T+2)
**Process:** `p_data_register`

**Inputs (Stage 0a registered outputs, T+1):**
`s_y_d1/u_d1/v_d1`, `s_low_*/s_high_*`, `s_show_matte`, `s_matte_mode`, `s_lfsr10_out`, `s_lfsr16_out`

**Outputs (T+2):**

| Signal | Comparison |
|--------|-----------|
| `s_pb_y_ge_low` | `s_y_d1 >= s_low_y` |
| `s_pb_y_le_high` | `s_y_d1 <= s_high_y` |
| `s_pb_lo_le_hi_y` | `s_low_y <= s_high_y` (normal vs inverted window) |
| (same pattern for U and V) | 6 more bits |
| `s_lfsr_d1` | Registered `lfsr10_out[9:0]` (frame-locked noise) |
| `s_prng_d1` | Registered `lfsr16_out[9:0]` (free-running noise) |
| `s_show_matte_d1`, `s_matte_mode_d1` | Controls delayed 1 clock |

**Timing note:** 9 independent carry chains. Stage 0a isolation prevents chaining with fine_knob logic.

---

### LFSR / PRNG Noise Generators (free-running, every clock)

**Modules:** `u_lfsr16` (SDK), `u_lfsr10` (SDK) **— Process:** `p_lfsr_reseed`

```
vsync_n ──► p_lfsr_reseed ──► s_lfsr_reset (1-clock pulse) ──► u_lfsr10.reset
u_lfsr16 ────────────────────────────────────────────────────► u_lfsr10.seed[9:1] & '1'
u_lfsr16 ──► s_lfsr16_out[9:0] ─────────────────────────────► Stage 0b → s_prng_d1
u_lfsr10 ──► s_lfsr10_out[9:0] ─────────────────────────────► Stage 0b → s_lfsr_d1
```

- `u_lfsr16` — free-runs always, period 65535, never reseeded. PRNG source.
- `u_lfsr10` — polynomial x¹⁰ + x⁷ + 1, period 1023. Reseeds from `lfsr16_out[9:0]` at every vsync falling edge (unconditional). Seed bit 0 forced `'1'` to prevent all-zeros lockup.

---

### Stage 1a — Window Flag Register (T+2 → T+3)
**Process:** `p_window_check`

**Inputs (Stage 0b registered outputs, T+2):**
all `s_pb_*`, `s_y_d1/u_d1/v_d1`, `s_lfsr_d1`, `s_prng_d1`, `s_show_matte_d1`, `s_matte_mode_d1`, `s_avid_d1`

**Outputs (T+3):**

| Signal | Description |
|--------|-------------|
| `s_wf_in_y`, `s_wf_in_u`, `s_wf_in_v` | Per-channel window flags |
| `s_wf_in_any` | `s_wf_in_y OR s_wf_in_u OR s_wf_in_v` |
| `s_wf_in_all` | `s_wf_in_y AND s_wf_in_u AND s_wf_in_v` |
| `s_wf_y_m`, `s_wf_u_m`, `s_wf_v_m` | Channel value if in-window, else 0 |
| `s_wf_y`, `s_wf_u`, `s_wf_v` | Raw pixel passthrough |
| `s_wf_lfsr`, `s_wf_prng` | Registered noise values |
| `s_wf_show_matte`, `s_wf_matte_mode`, `s_wf_avid` | Control passthrough |

**Key logic (pure LUT — no carry chains):**
```
s_pb_lo_le_hi = '1' (normal):   in_window = ge_low AND le_high
s_pb_lo_le_hi = '0' (inverted): in_window = ge_low OR  le_high
```

**Timing note:** No carry chains. All outputs registered so Stage 1b sees only FF outputs.

---

### Stage 1b — Matte Computation + Output Mux (T+3 → T+4)
**Process:** `p_window_key`

**Inputs:** all `s_wf_*` signals (Stage 1a registered outputs, T+3)

**Outputs (T+4):** `s_processed_y`, `s_processed_u`, `s_processed_v`, `s_processed_valid`

**Matte computation (`case s_wf_matte_mode`):**

| `s_wf_matte_mode` | Mode | Gate | Matte value |
|-------------------|------|------|-------------|
| `"000"` | Logical OR | `s_wf_in_any` | 1023 or 0 |
| `"001"` | Bitwise OR | — | `s_wf_y_m OR s_wf_u_m OR s_wf_v_m` |
| `"010"` | Logical AND | `s_wf_in_all` | 1023 or 0 |
| `"011"` | Bitwise AND | — | `s_wf_y_m AND s_wf_u_m AND s_wf_v_m` |
| `"100"` | Luma | `s_wf_in_all` | `s_wf_y` |
| `"101"` | LFSR | `s_wf_in_any` | `s_wf_lfsr` |
| `"110"` | PRNG | `s_wf_in_any` | `s_wf_prng` |
| `"111"` | Passthrough | — | (bypass — see output mux) |

**Output mux:**
```
avid = '0'         → s_wf_y / s_wf_u / s_wf_v        (blanking passthrough)
mode = "111"       → s_wf_y / s_wf_u / s_wf_v        (passthrough)
show_matte = '1'   → v_matte / 512 / 512              (greyscale matte, neutral chroma)
show_matte = '0':
  v_matte > 0      → s_wf_y / s_wf_u / s_wf_v        (original pixel)
  v_matte = 0      → 0 / 512 / 512                    (black)
```

**Timing note:** Pure mux/logic — no carry chains.

---

### Stage 2r — Dry Delay + Blend Factor Register (T+3 → T+4)
**Process:** `p_global_dry_delay`

Two things registered in the same clock cycle:

1. `s_wf_y/u/v` (T+3) → `s_y/u/v_for_global` (T+4) — aligns dry pixel with `s_processed_y/u/v`
2. `s_global_blend` (combinational) → `s_global_blend_r` (T+4) — registered blend factor

**Why this register exists:** Without it `s_global_blend` fans out from `registers_in(7)` directly to all three interpolator `t` inputs across the device. The registered copy lets the placer locate each interpolator's `t` register locally, eliminating ~2.4 ns routing wires that caused HD HDMI to fail at 71.82 MHz (passes at 78+ MHz with this register).

---

### Stage 2 — Global Wet/Dry Blend (T+4 → T+9)
**Instances:** `interp_global_y`, `interp_global_u`, `interp_global_v` (SDK `interpolator_u`, 4 clocks)

`result = a + (b - a) × t / 2^10`

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` | `s_y/u/v_for_global` | 4 | Dry (original) pixel |
| `b` | `s_processed_y/u/v` | 4 | Wet (processed) pixel |
| `t` | `s_global_blend_r` | 4 | Blend factor 0–1023 |
| `enable` | `s_processed_valid` | — | Holds output stable during blanking |

---

### Sync Delay Line (T+0 → T+9)
**Process:** `p_sync_delay`

Three 9-element `std_logic` shift registers (length = `C_PROCESSING_DELAY_CLKS = 9`):
```
data_in.hsync_n ──► [9] ──► data_out.hsync_n
data_in.vsync_n ──► [9] ──► data_out.vsync_n
data_in.field_n ──► [9] ──► data_out.field_n
```

---

## Signal Timing Summary

| Signal | T+ | Source |
|--------|----|--------|
| `data_in.y/u/v` | 0 | input port |
| `s_y_d1`, `s_u_d1`, `s_v_d1` | 1 | `p_control_decode` |
| `s_low_*/s_high_*` | 1 | `p_control_decode` |
| `s_show_matte`, `s_matte_mode` | 1 | `p_control_decode` |
| `s_pb_*` (9 comparison bits) | 2 | `p_data_register` |
| `s_lfsr_d1`, `s_prng_d1` | 2 | `p_data_register` |
| `s_show_matte_d1`, `s_matte_mode_d1` | 2 | `p_data_register` |
| `s_wf_in_y/u/v`, `s_wf_in_any/all` | 3 | `p_window_check` |
| `s_wf_y_m/u_m/v_m` | 3 | `p_window_check` |
| `s_wf_y/u/v`, `s_wf_lfsr`, `s_wf_prng` | 3 | `p_window_check` |
| `s_processed_y/u/v` | 4 | `p_window_key` |
| `s_y/u/v_for_global` | 4 | `p_global_dry_delay` |
| `s_global_blend_r` | 4 | `p_global_dry_delay` |
| `s_global_y/u/v` | 9 | `interp_global_*` |
| `data_out.hsync_n/vsync_n/field_n` | 9 | `p_sync_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | Y Low threshold | Knob 1 |
| `registers_in(1)` | U Low threshold | Knob 2 |
| `registers_in(2)` | V Low threshold | Knob 3 |
| `registers_in(3)` | Y High threshold | Knob 4 |
| `registers_in(4)` | U High threshold | Knob 5 |
| `registers_in(5)` | V High threshold | Knob 6 |
| `registers_in(6)(0)` | Show Matte | S1 |
| `registers_in(6)(1)` | Matte Bit 2 (MSB) | S2 |
| `registers_in(6)(2)` | Matte Bit 1 | S3 |
| `registers_in(6)(3)` | Matte Bit 0 (LSB) | S4 |
| `registers_in(6)(4)` | Fine mode | S5 |
| `registers_in(7)` | Global blend | Slider |

---

## Key Design Decisions

**Carry chain isolation** — `fine_knob` arithmetic (Stage 0a) and window comparisons (Stage 0b) are in separate stages. Merging them chains carry logic and fails timing at 74.25 MHz.

**Fan-out isolation** — `data_in.y/u/v` are registered in Stage 0a before Stage 0b comparisons. Raw input port signals have high fan-out; registering first prevents them from loading carry chain inputs.

**Stage 1a pure LUT** — pre-registering all 9 comparison bits in Stage 0b means Stage 1a contains only AND/OR of 1-bit flags. No carry chains; 1-clock budget achievable.

**`s_global_blend_r`** — extra register breaks fan-out from `registers_in(7)` to all three interpolator `t` inputs. Without it the placer scatters the instances and a ~2.4 ns cross-device routing wire appears on the critical path.

**SDK LFSR modules** — `lfsr16` and `lfsr10` used in preference to hand-rolled shift registers. `lfsr10` reseeds unconditionally at vsync — no mode check needed since noise outputs are consumed only in matte modes `"101"` and `"110"`.

**Dry tap at Stage 1a** — `s_y/u/v_for_global` tapped from `s_wf_y/u/v` (T+3) and registered once in `p_global_dry_delay` to T+4, matching `s_processed_y/u/v`. One process handles both the dry pixel delay and blend factor register, keeping alignment logic together.
