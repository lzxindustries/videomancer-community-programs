# CLAUDE.md — boneoh Videomancer Programs

Context and rules for working on Pete Appleby's Videomancer FPGA programs.

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
- `lfsr.vhd` in the SDK uses a right-shift Galois architecture; lockup state is **all-ones**, not all-zeros
- Bit-0 forcing on seed load does not protect against the all-ones lockup state in the Galois variant

---

## Known Deferred Issues

- **LFSR flashing bug** — RGB Bit Logic and YUV Bit Logic both exhibit occasional single-frame flashes of the original (unprocessed) image, approximately once per second. Suspected cause: Galois LFSR in `lfsr.vhd` hitting the all-ones lockup state. Investigation deferred.
- **LFSR/PRNG texture improvement** — Window Mask LFSR and PRNG modes produce functional but visually rough noise. Texture quality improvement is planned but deferred; both modes are currently marked experimental.

---

## Program List

| Program            | Colour space | Notes                                    |
|--------------------|--------------|------------------------------------------|
| rgb_bit_crush      | RGB          |                                          |
| rgb_bit_rotator    | RGB          |                                          |
| rgb_bit_logic      | RGB          | LFSR flashing bug deferred               |
| yuv_bit_logic      | YUV          | LFSR flashing bug deferred               |
| rgb_window_mask    | RGB          | LFSR/PRNG experimental                   |
| yuv_window_mask    | YUV          | LFSR/PRNG experimental                   |

---

*Maintained by Pete Appleby. Last updated during development of rgb_window_mask and yuv_window_mask.*
