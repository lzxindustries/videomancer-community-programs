# Program Improvement Scan
**Date:** 2026-04-20
**Scope:** All 8 programs under `programs/boneoh/`
**Method:** Review against SDK docs, program-categories.md, ai-program-generation-guide checklist, and VHD/TOML cross-reference.

---

## Common Issues (All 8 Programs)

These apply to every program and should be fixed in bulk:

1. **TOML: `category` → `categories`** — All programs use `category = "Video Processing"`, which is both the wrong field name (must be `categories`) and an invalid value ("Video Processing" does not exist in the 37-category list). Each program needs `categories = [...]` with values from `docs/program-categories.md`.
2. **TOML: Missing `program_type`** — Required field. All 8 are processing programs; add `program_type = "processing"` to each.
3. **TOML: No `[[preset]]` entries** — Optional but standard for community programs. Other authors include presets; adds user value.
4. **VHDL: `use work.all;`** — Present in all 8 VHD files. Redundant — all needed packages are already imported explicitly on the following lines.

---

## rgb_bit_crush

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Color", "Glitch"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Quality)**
- The TOML header comment block describes a completely different crush algorithm (knob → crush_n, 2–8 bits zeroed) from what's actually implemented in the VHD (knob → step_idx, steps: 8/16/32/48/64/96/128/256). Stale from an earlier design iteration.
- `display_min_value = 2` / `display_max_value = 8` on the three crush knobs doesn't reflect the actual step size range — display values are misleading.
- No `[[preset]]` entries (see common issue #3).

**VHDL (Minor)**
- `use work.all;` redundant (see common issue #4).
- Register map header has a polarity contradiction: says "hardware polarity: Off='1', On='0'" and "bit 0: Invert (0=On, 1=Off/normal)", but the process code uses `'1'` as the active/On state. One description is wrong.

---

## rgb_bit_rotator

**TOML (Bug)**
- `program_name = "RGB Bit Rotato"` — truncated, missing the final 'r'. Should be `"RGB Bit Rotator"`.

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Color", "Glitch"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Significant)**
- **Pipeline latency mismatch:** The file header and README both state "14 clock cycles", but `C_PROCESSING_DELAY_CLKS = 17` (with its own inline comment: "+1 for Stage 4a-ii pre-sum, +1 for Stage 0a-ii pre-sum"). The sync/bypass delay is actually 17 clocks. Stage 1 was split into Stage 1a (rotation) + Stage 1b (masking), Stage 4 gained a 4a-iii pre-sum stage, and Stage 0a-ii is fully registered — the header was never updated to reflect these additions.
- **Stage T+n annotation inconsistency:** Stage 1a process comment says "Input T+2, output T+3" but reads from `s_rgb_r` which is the Stage 0b T+3 output — the T values in process comments and signal declarations do not agree with each other throughout the YUV→RGB and RGB→YUV sections.
- **avid documentation contradiction:** Header (lines 85–88) states `data_out.avid` is driven from the 14-clock delay line, "NOT from `s_yuv_out_valid`". The actual output assignment (line 1511) uses `s_yuv_out_valid`. One of the two is wrong.

**VHDL (Minor)**
- `use work.all;` redundant (see common issue #4).

---

## rgb_bit_logic

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Color", "Glitch"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Minor — documentation only)**
- Signal declaration labels for Stage 4a-i say "(T+12)" but the process comment and file header both correctly state output T+13. Same label error on Stage 4b: signal section says "(T+14)", actual output is T+16. These label comments do not affect synthesis; the process-level T+n annotations and `C_PROCESSING_DELAY_CLKS = 16` are correct.
- `use work.all;` redundant (see common issue #4).

**Deferred investigation**
- LFSR flashing bug: in vsync-reseed mode (op=LFSR, switch 1 Off) the 10-bit LFSR reseeds from `lfsr16[9:0]` every vsync falling edge. If the seed drawn at that instant produces a high-value output on the very first pixel of the frame, it may manifest as a single-frame flash. Root cause not yet confirmed; likely requires VHDL Image Tester or hardware observation to isolate.

---

## rgb_window_mask

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Mask", "Color"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Convention deviation)**
- No Bypass parameter: `toggle_switch_11` is used for "Fine" mode (half-sensitivity), leaving no hardware bypass. All other programs reserve the last switch for Bypass per Videomancer convention. Users have no way to hard-bypass the effect from the panel.

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Naming)**
- Architecture is declared and closed as `rgb_window_key` but the file, program name, and all other identifiers use `rgb_window_mask`. The mismatch is harmless in synthesis but is confusing and inconsistent with the convention where the architecture name matches the program name.

**VHDL (Design)**
- No bypass path exists in the signal or output logic. Every other program has a bypass mux (`data_out <= s_delayed` when bypass switch is set). If a bypass switch were added to the TOML, the VHD would also need a bypass delay line and output mux.
- Inline LFSR implementation (a hand-written 10-bit Fibonacci shift register in `p_lfsr`) rather than the SDK's `entity work.lfsr` module. Functional, but inconsistent with rgb_bit_logic and yuv_bit_logic which use the SDK component.

**VHDL (Minor)**
- `use work.all;` redundant (see common issue #4).

**README note**
- LFSR and PRNG matte modes are already marked experimental — root cause of any visual roughness is the same vsync-reseed pattern noted in rgb_bit_logic.

---

## yuv_bit_crush

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Color", "Glitch"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Inaccuracy)**
- `description` says "YUV bit crushing with **Y brightness trim**, dither, and per-channel round/blend" — no Y brightness trim exists in the program. The feature does not appear in the register map, TOML parameters, or VHD logic. Description should be corrected.
- TOML header comment block describes the old crush algorithm (`crush_n = 2 + (knob * 7) / 1024`, zeroing N LSBs) which doesn't match the VHD implementation (`step_idx = knob / 128`, step sizes 8/16/32/48/64/96/128/256). Same stale-comment issue as rgb_bit_crush.
- `display_min_value = 2` / `display_max_value = 8` on the three crush knobs reflects the old N-bits model; actual step sizes range from 8 to 256 — display values are misleading.

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Minor)**
- Comment at line 365 reads "Pre-registers crush amounts, **brightness offset**, and mode flags…" — no brightness offset signal or logic exists in the file. Stale fragment from an earlier design iteration.
- `use work.all;` redundant (see common issue #4).

---

## yuv_bit_logic

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Color", "Glitch"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Minor)**
- `use work.all;` redundant (see common issue #4).

**Deferred investigation**
- Same LFSR vsync-reseed flashing concern as rgb_bit_logic — the two programs share the same LFSR architecture (SDK `lfsr` + `lfsr16` modules, vsync-triggered reseed in op=LFSR mode). If the root cause is identified and fixed in rgb_bit_logic, the same fix should be applied here.

*Otherwise the cleanest program in the set — pipeline documentation, architecture naming, bypass path, signal T-values, and register map comments are all internally consistent.*

---

## yuv_bit_rotator

**TOML (Bug)**
- `program_name = "YUV Bit Rotato"` — truncated, missing the final 'r'. Should be `"YUV Bit Rotator"`. Same defect as rgb_bit_rotator.

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Color", "Glitch"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Minor)**
- `use work.all;` redundant (see common issue #4).

*Otherwise clean — pipeline constant (10 clocks), header timing, signal section T-values, bypass path, avid assignment, and architecture name are all internally consistent. The simpler YUV-native structure avoided the Stage 1a/1b split and extra Stage 4 pre-sum stages that caused documentation drift in rgb_bit_rotator.*

---

## yuv_window_mask

**TOML (Required fixes)**
- `category = "Video Processing"` → `categories = ["Mask", "Color"]` (see common issue #1)
- Missing `program_type = "processing"` (see common issue #2)

**TOML (Convention deviation)**
- No Bypass parameter: `toggle_switch_11` is "Fine" mode, same as rgb_window_mask. No hardware bypass available from the panel.

**TOML (Quality)**
- No `[[preset]]` entries (see common issue #3).

**VHDL (Naming)**
- Architecture is declared and closed as `yuv_window_key` but the file, program name, and all identifiers use `yuv_window_mask`. Same naming mismatch as rgb_window_mask — the two programs share this inconsistency.

**VHDL (Design)**
- No bypass path or bypass mux in the output logic — by design, consistent with rgb_window_mask. The sync delay loop only delays hsync_n/vsync_n/field_n (not Y/U/V), which is correct given the absence of a bypass path.
- Inline LFSR implementation (hand-written `p_lfsr` process, same x^10 + x^7 + 1 polynomial and "0101010101" vsync seed as rgb_window_mask) rather than the SDK's `entity work.lfsr` module.

**VHDL (Minor)**
- `use work.all;` redundant (see common issue #4).

**README note**
- LFSR and PRNG matte modes are marked experimental — same root cause as rgb_window_mask.
