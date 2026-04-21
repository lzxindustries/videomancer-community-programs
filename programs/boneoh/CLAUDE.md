# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## Session Protocol

When Pete says "quit," pause and check whether any lessons learned from the session belong in CLAUDE.md. Summarize the gist and ask before updating. If nothing is worth recording, say so.

---

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.


## Context and rules for working on Pete Appleby's Videomancer FPGA programs.

---

## Standing Rule

Only ever edit files under `programs/boneoh/`. Never modify SDK files, shared library files, or other authors' programs.

---

## Hardware and Platform

- **Device:** Lattice iCE40 HX4K (tq144) on LZX Industries Videomancer rev_b
- **Clock:** 74.25 MHz hard requirement — all six HD build targets must pass
- **Pipeline:** YUV444, 10 bits per channel (0–1023)
- **UV neutral chroma:** U = V = 512 (mid-scale = no colour)
- **Videomancer UV convention:** `data_in.u` = Cr (red-difference), `data_in.v` = Cb — this is swapped from the standard convention; thresholds apply directly to the raw hardware values

---

## iCE40 Timing Rules

These were learned the hard way across multiple timing failures.

- The hard target is 74.25 MHz. Even 0.27 MHz short fails the build.
- **Carry chains** (10-bit comparisons, multiplications, adders) belong in their own pipeline stage. Pure LUT logic goes in the following stage.
- **4-input OR fits in 1 LUT4. 5-input does not.** A gate variable combined with a 4-input OR becomes a 5-input function requiring 2 LUT4s — 1 extra LUT level. Solution: remove the gate from that stage and apply it in the next stage on a registered signal (1 LUT, fresh clock cycle).
- **Fan-out kills timing.** Adding more loads to signals that already feed carry chains or multipliers increases routing delay. The router's wire budget is often the real bottleneck, not combinational LUT depth.
- **Moving logic earlier can make timing worse.** If the target stage already has heavy computation (e.g. multiplications), adding more fan-out there creates a new bottleneck. Measure first, assume nothing.
- **Pre-compute in the stage before you need the result.** The consuming stage should see only registered signals — no expressions in the critical path.
- **Never add a separate dry-delay process.** Take the bypass tap from `p_bypass_delay` at index `C_PRE_GLOBAL_DELAY_CLKS - 1`. A redundant delay process adds FFs and routing pressure.

---

## Pipeline Stage Conventions

| Stage   | Contents                                                                 |
|---------|--------------------------------------------------------------------------|
| Stage 0 | Data registration, control decode, carry-chain pre-computes (comparisons, multiplications) |
| Stage 1a | Pure LUT logic on registered inputs: window flags, masked values, noise OR pre-computes |
| Stage 1b | Mux/output only — all inputs are registered signals; no expressions     |
| Stage 2  | Global wet/dry blend (interpolator_u × 3), 4 clocks                    |

For RGB programs there are additional colour conversion stages (0a, 0a-ii, 0b, 0c) before Stage 1a.

---

## Design Process

- **Pseudocode first.** Agree on pseudocode and iterate until correct before touching any VHD. This catches logic errors before they become timing problems.
- Write pseudocode in plain text files in the program directory for reference.
- Build and test each program before moving to the next.

---

## Window Mask Design Decisions

These decisions were reached through pseudocode review and confirmed in testing.

**Normal mode (S1 Off):**
- Per-channel independent gating; S2/S3/S4 are ignored
- Failing Y → 0; failing U/V → 512 (neutral chroma — not 0)
- RGB version: failing channels → 0

**Matte mode (S1 On):**
- Output as greyscale: Y = matte value, U = V = 512 (YUV version); R = G = B = matte (RGB version)
- Luma, LFSR, and PRNG modes use **OR gate** (`s_wf_in_any`), not AND
- Logical OR and Bitwise OR modes also use `s_wf_in_any`
- Logical AND and Bitwise AND modes use `s_wf_in_all`

**LFSR/PRNG texture:**
- Noise OR `shift_right(channel, 4)` — limits pixel contribution to max 63/1023
- Noise dominates brightness; pixel adds subtle low-bit texture
- Gate applied in Stage 1b on registered `s_wf_in_any` (keeps Stage 1a at 2 LUT levels after carry chains)
- YUV version uses Y channel only for texture (saves timing, appropriate for YUV space)
- Both LFSR and PRNG modes are marked **experimental**

---

## LFSR / PRNG Notes

- Polynomial: x¹⁰ + x⁷ + 1, 10-bit Fibonacci LFSR
- **LFSR:** reseeds to `"0101010101"` on vsync falling edge → same noise pattern every frame (frozen static)
- **PRNG:** never reseeded — free-running from power-on; pattern shifts each frame
- `lfsr.vhd` in the SDK uses a **Fibonacci XOR right-shift architecture** — lockup state is **all-zeros**, not all-ones
- Bit-0 forcing on seed load (`seed(9:1) & '1'`) correctly prevents the all-zeros lockup state
- The `lfsr16.vhd` module handles the all-zeros case internally by substituting `x"ACE1"` when a zero seed is loaded

---

## Known Deferred Issues

- **LFSR flashing bug** — RGB Bit Logic and YUV Bit Logic both exhibit occasional single-frame flashes of the original (unprocessed) image, approximately once per second. Root cause is unconfirmed — the earlier diagnosis (Galois LFSR all-ones lockup) was incorrect; the SDK LFSR is Fibonacci XOR and the all-zeros state is already protected. Actual cause may be related to the vsync-triggered reseed producing a high-value output on the first pixel of a frame. Investigation deferred.

---

## Shared VHDL Package Files

RGB programs share a common set of YUV↔RGB conversion tables via `programs/boneoh/rgb_yuv_tables_pkg.vhd`. This file is **not** in any program directory — it lives one level up at `boneoh/` and is symlinked into each RGB program directory:

```
programs/boneoh/rgb_bit_rotator/rgb_yuv_tables_pkg.vhd -> ../rgb_yuv_tables_pkg.vhd
```

**How discovery works:** The SDK build system globs `*.vhd` in the program directory and follows symlinks, so the package is found and compiled automatically. Because the package filename sorts before the main architecture file alphabetically, and the build system always moves the `architecture ... of program_top` file to last, the package compiles in the correct order with no extra configuration.

**To add a new shared package:** create the `.vhd` file at `programs/boneoh/`, then `ln -sf ../filename.vhd` in each program directory that needs it.

---

## LFSR/PRNG Texture Design

**Noise as modulator = pleasing. Noise as output = harsh static.**

- **Bit Logic** (correct): `pixel XOR (noise AND mask_knob)` — noise randomly flips low-order bits of the pixel. The image content is the carrier; the noise scintillates on top. With a moderate mask, color and structure are preserved.

- **Window Mask** (original, wrong): `noise OR (channel >> 4)` — the full-range LFSR output (0–1023) was used directly as matte brightness. Since a 10-bit LFSR has a flat distribution across the full range, this produced TV static. The channel contribution (`>> 4`, max 63/1023) was invisible against it.

- **Window Mask** (fixed): `channel OR (noise >> 4)` — channel content occupies the full brightness range; noise contributes at most 63/1023 in the low bits. The in-window image is clearly visible with a scintillating low-bit texture.

**Rule:** whenever LFSR/PRNG output feeds a brightness or matte value, shift the noise down (`>> 4` or more) so it adds texture to the content rather than replacing it.

---

## Toggle Switch Patterns

**Switch naming layers** — there are three naming layers that must all agree:

| Layer | Example |
|-------|---------|
| Hardware switch number | S1, S2, S3 … (physical front-panel left-to-right) |
| ABI parameter_id | `toggle_switch_7`, `toggle_switch_8` … |
| TOML `name_label` / program label | "Direction", "Depth S1", "Depth S2" … |

S1 = `toggle_switch_7` (bit 0 of register 6), S2 = `toggle_switch_8` (bit 1), S3 = `toggle_switch_9` (bit 2), S4 = `toggle_switch_10` (bit 3), S5 = `toggle_switch_11` (bit 4 — Bypass by convention).

**`get_bit_depth` concatenation order** — when encoding N switches into a case selector, put the **highest-impact switch first (MSB)**:

```vhdl
-- Correct: s1 is MSB → flipping S1 has the biggest effect
case std_logic_vector'(s1 & s2 & s3) is
```

Reversing this (`s3 & s2 & s1`) makes the last switch the MSB. With the last switch off (the common case), only the low half of the case table is reachable — the milder bit depths — so the effect appears broken. This bug was found in both bit rotator programs: S2 and S3 appeared ignored whenever S4 was off, because S4 (mapped to `s3`) was the MSB and its `0` value confined the selector to depths 10/8/6/5 (too subtle to notice on rotating content).

**TOML comment tables** — when documenting switch combinations in a TOML comment, always write the table in the same order as the `get_bit_depth` case statement. If the code uses `s1 & s2 & s3`, the table rows should be in `s1 s2 s3` binary order (000=10bit, 001=8bit, … 111=1bit). A mismatch between the code and the TOML comment is how this bug went undetected.

---

## TOML Required Fields

Every program TOML **must** include these fields or the firmware will reject the package:

- `categories` — array of one or more valid category names from `docs/program-categories.md` (up to 8). Field name is plural; `category` (singular) is wrong. `"Video Processing"` is not a valid category name.
- `program_type` — either `"processing"` (transforms input video) or `"synthesis"` (generates video from scratch)

Common category choices for this project's programs: `"Color"`, `"Glitch"`, `"Mask"`.

---

## Architecture Naming Convention

The VHDL architecture name must match the program file name:

```vhdl
-- Correct:
architecture rgb_window_mask of program_top is
...
end architecture rgb_window_mask;

-- Wrong (causes confusion, found in window mask programs):
architecture rgb_window_key of program_top is
```

---

## Program List

| Program            | Colour space | Notes                                    |
|--------------------|--------------|------------------------------------------|
| rgb_bit_crush      | RGB          |                                          |
| rgb_bit_rotator    | RGB          |                                          |
| rgb_bit_logic      | RGB          | LFSR flashing bug deferred               |
| rgb_window_mask    | RGB          | no bypass switch                         |
| yuv_bit_crush      | YUV          |                                          |
| yuv_bit_rotator    | YUV          |                                          |
| yuv_bit_logic      | YUV          | LFSR flashing bug deferred               |
| yuv_window_mask    | YUV          | no bypass switch                         |
| yuv_shape_key      | YUV          | no bypass switch; cross shape solid-only in v1 |

---

*Maintained by Pete Appleby. Last updated during development of rgb_window_mask and yuv_window_mask.*
4/19/2026 added general rules from https://github.com/forrestchang/andrej-karpathy-skills/blob/main/CLAUDE.md
4/19/2026 README Technical Notes sections for all 8 programs verified and updated against clean build results (Clean and Build.2026.04.19.txt). Pipeline latencies, BRAM usage, IOs, PLLs, and HD worst-case timing figures are current as of this date.
4/20/2026 Full improvement scan completed across all 8 programs (see program-improvement-scan.md). LFSR lockup-state diagnosis corrected (Fibonacci XOR, not Galois; all-zeros is the lockup state, not all-ones). TOML required fields rule added. Architecture naming convention added. Program list completed (yuv_bit_crush and yuv_bit_rotator were missing).
4/20/2026 yuv_shape_key initial implementation written (yuv_shape_key.vhd + yuv_shape_key.toml). 10-clock pipeline; inline pixel counter; 4 shapes; Knob 6 3-mux norm; dither via norm-space bias; lfsr16 free-running. Pending: build and test.
4/21/2026 Fixed get_bit_depth concatenation order bug in both bit rotators (s3&s2&s1 → s1&s2&s3). Added Toggle Switch Patterns section to CLAUDE.md. Consolidated YUV<->RGB LUT tables from all 4 RGB programs into shared rgb_yuv_tables_pkg.vhd (symlinked into each program directory); ~747 lines removed per program. Fixed Window Mask LFSR/PRNG texture (noise OR channel>>4 → channel OR noise>>4); removed LFSR/PRNG experimental flag from both window masks. Added Shared VHDL Package Files and LFSR/PRNG Texture Design sections to CLAUDE.md.