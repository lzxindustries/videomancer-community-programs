# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

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
- **LFSR/PRNG texture improvement** — Window Mask LFSR and PRNG modes produce functional but visually rough noise. Texture quality improvement is planned but deferred; both modes are currently marked experimental.

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
| rgb_window_mask    | RGB          | LFSR/PRNG experimental; no bypass switch |
| yuv_bit_crush      | YUV          |                                          |
| yuv_bit_rotator    | YUV          |                                          |
| yuv_bit_logic      | YUV          | LFSR flashing bug deferred               |
| yuv_window_mask    | YUV          | LFSR/PRNG experimental; no bypass switch |
| yuv_shape_key      | YUV          | no bypass switch; cross shape solid-only in v1 |

---

*Maintained by Pete Appleby. Last updated during development of rgb_window_mask and yuv_window_mask.*
4/19/2026 added general rules from https://github.com/forrestchang/andrej-karpathy-skills/blob/main/CLAUDE.md
4/19/2026 README Technical Notes sections for all 8 programs verified and updated against clean build results (Clean and Build.2026.04.19.txt). Pipeline latencies, BRAM usage, IOs, PLLs, and HD worst-case timing figures are current as of this date.
4/20/2026 Full improvement scan completed across all 8 programs (see program-improvement-scan.md). LFSR lockup-state diagnosis corrected (Fibonacci XOR, not Galois; all-zeros is the lockup state, not all-ones). TOML required fields rule added. Architecture naming convention added. Program list completed (yuv_bit_crush and yuv_bit_rotator were missing).
4/20/2026 yuv_shape_key initial implementation written (yuv_shape_key.vhd + yuv_shape_key.toml). 10-clock pipeline; inline pixel counter; 4 shapes; Knob 6 3-mux norm; dither via norm-space bias; lfsr16 free-running. Pending: build and test.