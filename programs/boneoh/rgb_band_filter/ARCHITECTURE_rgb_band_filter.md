# RGB Band Filter — Architecture

This document describes the signal flow and pipeline architecture of `rgb_band_filter.vhd` for developers. Signal names match the VHDL exactly.

Note: the VHDL architecture is named `rgb_window_key` (legacy name); the program is shipped as `rgb_band_filter`.

---

## Pipeline Overview

Total latency: **14 clock cycles** at 74.25 MHz.

```
T+0  data_in.y/u/v ──────────────────────────────── sync delay (14 clocks) ──► data_out.hsync_n/vsync_n/field_n
     registers_in[0..7]
         │
         ├──────────────────────────────────────────────────────────────────────── LFSR/PRNG (free-running)
         │
         ▼
    ┌──────────┐  1 clk    YUV→RGB conversion (part 1 of 3)
    │ Stage 0a │  BRAM lookup: 4 channel offsets
    │          │  control decode + fine_knob thresholds
    └────┬─────┘
         │  s_yr_r/gu/gv/b_off, s_yr_y, s_yr_avid, s_yr_u/v_raw
         │  s_low_r/g/b, s_high_r/g/b, s_show_matte, s_matte_mode
         ▼
    ┌──────────┐  1 clk    YUV→RGB conversion (part 2 of 3)
    │Stage 0a-ii  G pre-sum: s_yr_gu_off + s_yr_gv_off
    └────┬─────┘
         │  s_yr_g_presum, s_yr_r/b_off_d, s_yr_y_d, s_yr_avid_d, s_yr_u/v_raw_d
         ▼
    ┌──────────┐  1 clk    YUV→RGB conversion (part 3 of 3)
    │ Stage 0b │  accumulate: Y + offset per channel, clamp to 0–1023
    └────┬─────┘
         │  s_rgb_r, s_rgb_g, s_rgb_b, s_rgb_valid
         ▼
    ┌──────────┐  1 clk    BT.601 luma partial products (3x multiply, carry chains)
    │ Stage 0c │  s_rgb_r×77, s_rgb_g×150, s_rgb_b×29
    │          │  propagate pixel + controls
    └────┬─────┘
         │  s_0c_luma_r/g/b_prod, s_0c_rgb_r/g/b, s_0c_rgb_valid
         │  s_0c_show_matte, s_0c_matte_mode, s_0c_lfsr, s_0c_prng
         ▼
    ┌──────────┐  1 clk    window checks (3x carry-chain comparisons)
    │ Stage 1a │  luma sum (add 3 partial products → s_wf_luma)
    │          │  masked values, flag reductions
    └────┬─────┘
         │  s_wf_in_r/g/b, s_wf_in_any/all
         │  s_wf_r/g/b_m, s_wf_luma
         │  s_wf_rgb_r/g/b, s_wf_rgb_valid
         │  s_wf_show_matte, s_wf_matte_mode, s_wf_lfsr, s_wf_prng
         ▼
    ┌──────────┐  1 clk    matte computation + output mux
    │ Stage 1b │  (pure mux — all inputs registered)
    └────┬─────┘
         │  s_processed_r/g/b, s_processed_valid
         │
    ┌────┴──────────────────────────────────────────────────┐
    │  [WET]                              [DRY]              │
    │  s_processed_r/g/b (T+6)  s_r/g/b_for_global (T+6)   │◄── p_global_dry_delay
    │  s_global_blend (combinational)     │                  │    (1-clock tap from
    ▼                                     ▼                  │     s_wf_rgb_* at T+5)
    ┌──────────────────────────────────────────────────────┐ │
    │  Stage 2: 3x interpolator_u (global blend)  4 clocks │ │
    │  interp_global_r / interp_global_g / interp_global_b │ │
    └──────────────────────┬───────────────────────────────┘ │
                           │  s_global_r/g/b (T+10)
                           ▼
    ┌──────────┐  1 clk    RGB→YUV conversion (part 1 of 4)
    │Stage 3a-i│  BRAM lookup: 7 partial values
    └────┬─────┘
         │  s_3a_ry_r/g/b, s_3a_ru_r/g, s_3a_rv_g/b, s_3a_r/b_d
         ▼
    ┌──────────┐  1 clk    RGB→YUV conversion (part 2 of 4)
    │Stage 3a-ii  partial sums: pair additions
    └────┬─────┘
         │  s_3b_ry_rg, s_3b_ry_b, s_3b_ru_rg, s_3b_rv_gb, s_3b_r/b_d
         ▼
    ┌──────────┐  1 clk    RGB→YUV conversion (part 3 of 4)
    │Stage 3a-iii  channel sums: Y/U/V final additions + wire shifts
    └────┬─────┘
         │  s_ry_y/u/v_sum
         ▼
    ┌──────────┐  1 clk    RGB→YUV conversion (part 4 of 4)
    │ Stage 3b │  clamp + UV offset + convention swap
    └────┬─────┘
         │  s_yuv_out_y/u/v, s_yuv_out_valid
         ▼
    data_out.y/u/v, data_out.avid
```

---

## BT.601 Colour Conversion Overview

The program converts YUV→RGB at the input, processes in RGB space, then converts RGB→YUV at the output. Both paths use pre-computed BRAM tables from `rgb_yuv_tables_pkg.vhd` (11 BRAMs total). The arithmetic avoids multipliers by pre-dividing all table values by 1024 — additions and shifts only.

**UV convention swap:** `data_in.u` = Cr (red-difference) and `data_in.v` = Cb, which is the reverse of standard BT.601 (U=Cb, V=Cr). The BRAM tables are built for the standard convention, so `data_in.u` and `data_in.v` are swapped when indexing the lookup tables. The swap is reversed at the output.

---

## Stage Detail

### Stage 0a — YUV→RGB BRAM Lookup + Control Decode (T+0 → T+1)
**Process:** `p_yuv_rgb_lut`

**BRAM lookups (4 tables, U/V swapped for convention):**

| Signal | Table | Index | Description |
|--------|-------|-------|-------------|
| `s_yr_r_off` | `s_lut_yr_r` | `data_in.u` (=Cr) | R channel offset from chroma |
| `s_yr_gu_off` | `s_lut_yr_gu` | `data_in.v` (=Cb) | G channel U-component offset |
| `s_yr_gv_off` | `s_lut_yr_gv` | `data_in.u` (=Cr) | G channel V-component offset |
| `s_yr_b_off` | `s_lut_yr_b` | `data_in.v` (=Cb) | B channel offset from chroma |

Also registered: `s_yr_y`, `s_yr_avid`, `s_yr_u_raw`, `s_yr_v_raw` (raw inputs delayed for blanking passthrough).

**Control decode (parallel with BRAM lookup):**
- `s_show_matte` — `registers_in(6)(0)` (S1)
- `s_matte_mode` — `{reg(6)(1), reg(6)(2), reg(6)(3)}` (S2=MSB, S4=LSB)
- `s_fine`, `s_in_ref(0..5)` — fine mode state; refs latched on Normal→Fine edge
- `s_low_r/g/b`, `s_high_r/g/b` — thresholds (fine_knob or direct from knobs 1–6)

**Timing note:** BRAM reads are fast (no carry chains). `fine_knob` carry chains land here in parallel. Both complete in 1 clock.

---

### Stage 0a-ii — G-channel Pre-Sum (T+1 → T+2)
**Process:** `p_yuv_rgb_presum`

**Purpose:** G requires two chroma offsets (gu + gv). Summing them here keeps Stage 0b to a single addition per channel, preventing G from having a longer carry chain than R or B.

| Output | Computation |
|--------|-------------|
| `s_yr_g_presum` | `s_yr_gu_off + s_yr_gv_off` |
| `s_yr_r_off_d` | `s_yr_r_off` delayed 1 clock |
| `s_yr_b_off_d` | `s_yr_b_off` delayed 1 clock |
| `s_yr_y_d`, `s_yr_avid_d`, `s_yr_u/v_raw_d` | delayed for Stage 0b alignment |

**Timing note:** Adding arithmetic inside the BRAM read process breaks BRAM inference. This separate stage keeps BRAM reads clean and arithmetic in the next clock.

---

### Stage 0b — YUV→RGB Accumulate and Clamp (T+2 → T+3)
**Process:** `p_yuv_rgb_acc`

**Computation (active video only):**
```
R = clamp10(Y + s_yr_r_off_d)
G = clamp10(Y + s_yr_g_presum)
B = clamp10(Y + s_yr_b_off_d)
```

**Blanking passthrough:** When `avid='0'`, raw YUV is passed through as-is (`R=Y, G=U, B=V`) to preserve sync structure.

**Outputs:** `s_rgb_r`, `s_rgb_g`, `s_rgb_b`, `s_rgb_valid`

---

### Stage 0c — Luma Partial Products (T+3 → T+4)
**Process:** `p_luma_mult`

**Purpose:** Pre-register the three BT.601 luma multiplications so their carry chains do not chain into Stage 1a's window comparison carry chains.

| Signal | Computation | Width |
|--------|-------------|-------|
| `s_0c_luma_r_prod` | `s_rgb_r × 77` | 17-bit |
| `s_0c_luma_g_prod` | `s_rgb_g × 150` | 18-bit |
| `s_0c_luma_b_prod` | `s_rgb_b × 29` | 15-bit |

Also propagates: `s_0c_rgb_r/g/b`, `s_0c_rgb_valid`, `s_0c_show_matte`, `s_0c_matte_mode`, `s_0c_lfsr`, `s_0c_prng`.

**Timing note:** Three independent multiplier carry chains land here. Stage 1a sees only registered products — no multiplication on its critical path.

---

### LFSR / PRNG Noise Generators (hand-rolled, free-running)

**Processes:** `p_lfsr`, `p_prng`

Note: `rgb_band_filter` uses hand-rolled 10-bit shift registers rather than the SDK `lfsr`/`lfsr16` modules used by the YUV programs.

```
vsync_n falling edge ──► p_lfsr: s_lfsr reseeds from s_prng
hsync_n falling edge ──► p_prng: s_prng reseeds from s_lfsr
```

- `s_lfsr` — 10-bit Fibonacci LFSR, polynomial x¹⁰ + x⁷ + 1, period 1023. Reseeds from `s_prng` at each vsync falling edge, giving a different pattern per frame.
- `s_prng` — same polynomial. Reseeds from `s_lfsr` at each hsync falling edge, giving per-line variation.
- Both outputs are XOR'd with `s_wf_luma` in Stage 1b (matte modes `"101"` and `"110"`), making the noise content-dependent.

---

### Stage 1a — Window Checks + Luma Sum (T+4 → T+5)
**Process:** `p_window_check`

**Inputs (Stage 0c registered outputs, T+4):**
`s_0c_rgb_r/g/b`, `s_0c_luma_r/g/b_prod`, `s_0c_rgb_valid`, `s_0c_show_matte`, `s_0c_matte_mode`, `s_0c_lfsr`, `s_0c_prng`, and `s_low_r/g/b`, `s_high_r/g/b` (from Stage 0a)

**Window comparisons (`window_check` function, 3x carry chains):**
```
Normal   (low <= high): in_window = (pixel >= low) AND (pixel <= high)
Inverted (low >  high): in_window = (pixel >= low) OR  (pixel <= high)
```

**Luma sum:**
```
v_luma_wide = resize(luma_r_prod, 18) + resize(luma_g_prod, 18) + resize(luma_b_prod, 18)
s_wf_luma = v_luma_wide[17:8]   (divide by 256 → BT.601 10-bit luma)
```
Max sum: 1023×77 + 1023×150 + 1023×29 = 261,888 < 2¹⁸ = 262,144 ✓

**Outputs (T+5):**

| Signal | Description |
|--------|-------------|
| `s_wf_in_r/g/b` | Per-channel window flags |
| `s_wf_in_any` | `in_r OR in_g OR in_b` |
| `s_wf_in_all` | `in_r AND in_g AND in_b` |
| `s_wf_r/g/b_m` | Masked values (channel if in-window, else 0) |
| `s_wf_luma` | BT.601 luma (10-bit) |
| `s_wf_rgb_r/g/b`, `s_wf_rgb_valid` | Raw RGB passthrough |
| `s_wf_show_matte`, `s_wf_matte_mode` | Control passthrough |
| `s_wf_lfsr`, `s_wf_prng` | Noise passthrough |

**Timing note:** 3 window comparison carry chains + 2 luma addition carry chains. Stage 0c isolation prevents luma multiply chains from chaining here.

---

### Stage 1b — Window Key Operation (T+5 → T+6)
**Process:** `p_window_key`

**Inputs:** all `s_wf_*` signals (Stage 1a registered outputs, T+5)

**Outputs (T+6):** `s_processed_r/g/b`, `s_processed_valid`

**Matte computation (`case s_wf_matte_mode`):**

| `s_wf_matte_mode` | Mode | Gate | Matte value |
|-------------------|------|------|-------------|
| `"000"` | Logical OR | `s_wf_in_any` | 1023 or 0 |
| `"001"` | Bitwise OR | — | `s_wf_r_m OR s_wf_g_m OR s_wf_b_m` |
| `"010"` | Logical AND | `s_wf_in_all` | 1023 or 0 |
| `"011"` | Bitwise AND | — | `s_wf_r_m AND s_wf_g_m AND s_wf_b_m` |
| `"100"` | Luma | `s_wf_in_all` | `s_wf_luma` |
| `"101"` | LFSR | `s_wf_in_any` | `s_wf_lfsr XOR s_wf_luma` |
| `"110"` | PRNG | `s_wf_in_any` | `s_wf_prng XOR s_wf_luma` |
| `"111"` | Passthrough | — | (bypass) |

**Output mux:**
```
mode = "111"       → s_wf_rgb_r/g/b         (passthrough)
show_matte = '1'   → v_matte / v_matte / v_matte  (greyscale: R=G=B → true mono in YUV)
show_matte = '0':
  v_matte > 0      → s_wf_rgb_r/g/b         (original pixel)
  v_matte = 0      → 0 / 0 / 0              (black)
```

Note: Show Matte On sets R=G=B=matte so the downstream RGB→YUV conversion produces a true monochrome signal.

**Timing note:** Pure mux/logic — no carry chains.

---

### Global Dry Delay (T+5 → T+6)
**Process:** `p_global_dry_delay`

```
s_wf_rgb_r/g/b (T+5) ──► s_r/g/b_for_global (T+6)
```

The pixel was already propagated through Stage 0c (T+3→T+4) and Stage 1a (T+4→T+5) via `s_0c_rgb_*` and `s_wf_rgb_*`. This single final register aligns the dry path with `s_processed_r/g/b` at T+6.

---

### Stage 2 — Global Wet/Dry Blend (T+6 → T+10)
**Instances:** `interp_global_r`, `interp_global_g`, `interp_global_b` (SDK `interpolator_u`, 4 clocks)

| Port | Signal | T+ | Description |
|------|--------|----|-------------|
| `a` (dry) | `s_r/g/b_for_global` | 6 | Original RGB pixel |
| `b` (wet) | `s_processed_r/g/b` | 6 | Matte-processed RGB |
| `t` | `s_global_blend` | — | Global blend factor (combinational, slider) |
| `enable` | `s_processed_valid` | — | Holds output stable during blanking |

---

### Stage 3a-i — RGB→YUV BRAM Lookup (T+10 → T+11)
**Process:** `p_rgb_yuv_lut`

7 BRAM tables, indexed by the blended RGB values:

| Signal | Table | Index | Range | Description |
|--------|-------|-------|-------|-------------|
| `s_3a_ry_r` | `s_lut_ry_r` | R | 0..299 | Y ← R contribution |
| `s_3a_ry_g` | `s_lut_ry_g` | G | 0..601 | Y ← G contribution |
| `s_3a_ry_b` | `s_lut_ry_b` | B | 0..117 | Y ← B contribution |
| `s_3a_ru_r` | `s_lut_ru_r` | R | −173..0 | Cb ← R contribution |
| `s_3a_ru_g` | `s_lut_ru_g` | G | −339..0 | Cb ← G contribution |
| `s_3a_rv_g` | `s_lut_rv_g` | G | −429..0 | Cr ← G contribution |
| `s_3a_rv_b` | `s_lut_rv_b` | B | −83..0 | Cr ← B contribution |

Also: `s_3a_r_d`, `s_3a_b_d` (R and B delayed for wire-shift in Stage 3a-iii).

Note: `ru_b` (Cb←B = B/2) and `rv_r` (Cr←R = R/2) are implemented as wire shifts in Stage 3a-iii, not BRAM tables.

---

### Stage 3a-ii — RGB→YUV Partial Sums (T+11 → T+12)
**Process:** `p_rgb_yuv_presum`

Pairs BRAM results are summed, one clock after the BRAM reads (required to preserve BRAM inference — arithmetic inside a BRAM read process prevents synthesis from mapping to block RAM):

| Signal | Computation |
|--------|-------------|
| `s_3b_ry_rg` | `s_3a_ry_r + s_3a_ry_g` |
| `s_3b_ry_b` | `s_3a_ry_b` (pass-through) |
| `s_3b_ru_rg` | `s_3a_ru_r + s_3a_ru_g` |
| `s_3b_rv_gb` | `s_3a_rv_g + s_3a_rv_b` |
| `s_3b_r_d`, `s_3b_b_d` | R and B delayed for wire shifts |

---

### Stage 3a-iii — RGB→YUV Channel Sums (T+12 → T+13)
**Process:** `p_rgb_yuv_sum`

Final channel sums with wire-shift terms for the two missing BRAM entries:

```
Y_sum  = s_3b_ry_rg + s_3b_ry_b
Cb_sum = s_3b_ru_rg + (B >> 1)   (unsigned, wire shift — ru_b = B/2)
Cr_sum = (R >> 1) + s_3b_rv_gb   (unsigned, wire shift — rv_r = R/2)
```

---

### Stage 3b — RGB→YUV Clamp + Convention Swap (T+13 → T+14)
**Process:** `p_rgb_yuv_acc`

```
s_yuv_out_y = clamp10(Y_sum)
s_yuv_out_u = clamp10_uv(Cr_sum)   ← Cr → data_out.u  (SDK convention: u=Cr)
s_yuv_out_v = clamp10_uv(Cb_sum)   ← Cb → data_out.v  (SDK convention: v=Cb)
```

`clamp10_uv(v)` = `clamp(v + 512, 0, 1023)` — adds the UV offset (512 = neutral chroma) with the bounds folded as compile-time constants to avoid an adder on the comparator critical path.

**Convention swap:** Cb and Cr are swapped on assignment to `data_out.u/v` to restore the Videomancer SDK convention (`u=Cr, v=Cb`).

---

### Sync Delay Line (T+0 → T+14)
**Process:** `p_sync_delay`

```
data_in.hsync_n ──► [14-clock shift] ──► data_out.hsync_n
data_in.vsync_n ──► [14-clock shift] ──► data_out.vsync_n
data_in.field_n ──► [14-clock shift] ──► data_out.field_n
```

---

## Signal Timing Summary

| Signal | T+ | Source |
|--------|----|--------|
| `data_in.y/u/v` | 0 | input port |
| `s_yr_r/gu/gv/b_off`, `s_yr_y/avid/u_raw/v_raw` | 1 | `p_yuv_rgb_lut` |
| `s_low_r/g/b`, `s_high_r/g/b` | 1 | `p_yuv_rgb_lut` |
| `s_show_matte`, `s_matte_mode` | 1 | `p_yuv_rgb_lut` |
| `s_yr_g_presum`, `s_yr_r/b_off_d` | 2 | `p_yuv_rgb_presum` |
| `s_rgb_r/g/b`, `s_rgb_valid` | 3 | `p_yuv_rgb_acc` |
| `s_0c_luma_r/g/b_prod` | 4 | `p_luma_mult` |
| `s_0c_rgb_r/g/b`, `s_0c_show_matte`, `s_0c_matte_mode` | 4 | `p_luma_mult` |
| `s_wf_in_r/g/b`, `s_wf_in_any/all` | 5 | `p_window_check` |
| `s_wf_r/g/b_m`, `s_wf_luma` | 5 | `p_window_check` |
| `s_wf_rgb_r/g/b`, `s_wf_show_matte`, `s_wf_matte_mode` | 5 | `p_window_check` |
| `s_processed_r/g/b` | 6 | `p_window_key` |
| `s_r/g/b_for_global` | 6 | `p_global_dry_delay` |
| `s_global_r/g/b` | 10 | `interp_global_r/g/b` |
| `s_3a_ry_r/g/b`, `s_3a_ru_r/g`, `s_3a_rv_g/b` | 11 | `p_rgb_yuv_lut` |
| `s_3b_ry_rg`, `s_3b_ru_rg`, `s_3b_rv_gb` | 12 | `p_rgb_yuv_presum` |
| `s_ry_y/u/v_sum` | 13 | `p_rgb_yuv_sum` |
| `s_yuv_out_y/u/v` | 14 | `p_rgb_yuv_acc` |
| `data_out.hsync_n/vsync_n/field_n` | 14 | `p_sync_delay` |

---

## Register Map

| Register | Signal | Control |
|----------|--------|---------|
| `registers_in(0)` | R Low threshold | Knob 1 |
| `registers_in(1)` | G Low threshold | Knob 2 |
| `registers_in(2)` | B Low threshold | Knob 3 |
| `registers_in(3)` | R High threshold | Knob 4 |
| `registers_in(4)` | G High threshold | Knob 5 |
| `registers_in(5)` | B High threshold | Knob 6 |
| `registers_in(6)(0)` | Show Matte | S1 |
| `registers_in(6)(1)` | Matte Bit 2 (MSB) | S2 |
| `registers_in(6)(2)` | Matte Bit 1 | S3 |
| `registers_in(6)(3)` | Matte Bit 0 (LSB) | S4 |
| `registers_in(6)(4)` | Fine mode | S5 |
| `registers_in(7)` | Global blend | Slider |

---

## Key Design Decisions

**G pre-sum stage (0a-ii)** — G requires two chroma contributions from the YUV→RGB tables (gu and gv), while R and B each need only one. Without the pre-sum, Stage 0b would have two chained additions for G but one for R and B, creating an unbalanced critical path. The pre-sum stage balances all three channels to a single addition each in Stage 0b.

**Stage 0c — luma multiply isolation** — BT.601 luma requires R×77 + G×150 + B×29. Registering the three partial products in Stage 0c prevents the multiply carry chains from chaining into Stage 1a's window comparison carry chains. Without this stage, Stage 1a would have both multiply and compare carry chains, almost certainly failing timing.

**BRAM arithmetic isolation** — arithmetic added inside a BRAM read process prevents Yosys from inferring block RAM (the read address and output must be directly connected). All seven RGB→YUV BRAM reads in Stage 3a-i are clean; the additions happen in Stages 3a-ii and 3a-iii one and two clocks later. The same principle applies to the four YUV→RGB reads in Stage 0a.

**Wire shifts for ru_b and rv_r** — Cb←B and Cr←R both equal channel/2, which is a wire shift (no BRAM, no arithmetic carry chain). These are implemented inline in Stage 3a-iii as `'0' & channel(9 downto 1)`.

**Hand-rolled LFSR/PRNG** — unlike the YUV bit logic and band filter programs which use SDK `lfsr`/`lfsr16` modules, this program uses hand-rolled 10-bit shift registers with cross-seeding (LFSR seeds from PRNG at vsync; PRNG seeds from LFSR at hsync). The matte noise output is XOR'd with BT.601 luma (`s_wf_luma XOR noise`) rather than using raw noise, making the pattern content-dependent.

**`clamp10_uv` constant folding** — the UV offset (512) is folded into the comparator bounds as compile-time constants, so the clamping comparison evaluates directly on the sum without an adder in the critical path. Only the non-saturating branch adds the offset.
