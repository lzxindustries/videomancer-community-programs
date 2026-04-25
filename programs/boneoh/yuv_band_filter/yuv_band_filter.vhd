-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   YUV Window Key
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Dual-function processor: per-channel YUV multi-mode filter + window keying.
--
--   PRIMARY FUNCTION: Multi-Mode Filtering
--   Per-channel frequency-domain filtering in YUV space using four filter types
--   controlled independently by knob settings:
--     - Low Pass:  attenuates frequencies above threshold
--     - High Pass: attenuates frequencies below threshold
--     - Band Pass: isolates frequencies between Low/High thresholds
--     - Notch:     rejects frequencies between Low/High thresholds
--
--   SECONDARY FUNCTION: Window Keying / Matte Processing
--   Per-channel window keying operating directly in YUV space.  For each channel
--   (Y, U, V) a lower and upper threshold knob define a window.  When the lower
--   threshold exceeds the upper threshold the window inverts automatically for that
--   channel — no extra switch required.
--
--   Matte Mode (S2/S3/S4 as a 3-bit word, S2=MSB, S4=LSB):
--     000  Logical OR    - white (1023) if any channel in-window, else black
--     001  Bitwise OR    - OR of channel values; failing channels contribute 0
--     010  Logical AND   - white (1023) if all channels in-window, else black
--     011  Bitwise AND   - AND of channel values; failing channels contribute 0
--     100  Luma          - Y channel value passed directly, gated by logical AND
--     101  LFSR synced   - frame-locked noise value, gated by logical OR
--     110  PRNG          - free-running noise value, gated by logical OR
--     111  Passthrough   - original Y/U/V pixel (no keying)
--
--   Show Matte (S1):
--     On  — outputs the computed matte as Y; U and V are forced to 512 (neutral
--           chroma) for a true monochrome signal suitable for feeding other devices.
--     Off — matte gates original pixel: matte>0 passes Y/U/V through; matte=0
--           outputs black (Y=0, U=512, V=512).
--
--   No colour space conversion is performed; processing is entirely in YUV.
--
-- Architecture:
--   Stage 0   - Control decode + data register + comparisons  (1 clock) -> T+1
--   Stage 1a  - Window flag register (LUT-only)               (1 clock) -> T+2
--   Stage 1b  - Matte computation + output                    (1 clock) -> T+3
--   Stage 2   - Global Blend (3x interpolator_u)             (4 clocks) -> T+7
--
-- Videomancer UV Convention Note:
--   data_in.u / data_out.u = Cr (red-difference);  data_in.v / data_out.v = Cb.
--   Thresholds for U and V are applied directly to whatever value the hardware
--   presents — no internal remapping is performed.
--
-- Register Map:
--   Register  0: Y channel low threshold  (0-1023)   rotary_potentiometer_1
--   Register  1: U channel low threshold  (0-1023)   rotary_potentiometer_2
--   Register  2: V channel low threshold  (0-1023)   rotary_potentiometer_3
--   Register  3: Y channel high threshold (0-1023)   rotary_potentiometer_4
--   Register  4: U channel high threshold (0-1023)   rotary_potentiometer_5
--   Register  5: V channel high threshold (0-1023)   rotary_potentiometer_6
--   Register  6: Packed toggle bits (Off='0', On='1'):
--     bit 0: Show Matte (0=Off/gate pixel, 1=On/show matte)            toggle_switch_7
--     bit 3: Matte Bit2 MSB of 3-bit matte mode {S2,S3,S4}             toggle_switch_8
--     bit 2: Matte Bit1 middle bit of matte mode word                  toggle_switch_9
--     bit 1: Matte Bit0 LSB of matte mode word                         toggle_switch_10
--     bit 4: Fine       (0=Normal full-range, 1=Fine 1/8-sensitivity)   toggle_switch_11
--   Register  7: Global blend (0=dry, 1023=wet)                    linear_potentiometer_12
--
--   Fine mode: on the Normal->Fine transition, the current knob positions are
--   latched as reference values.  Each knob then controls its threshold as a
--   weighted blend of the current position and the locked reference, giving 1/8
--   the sensitivity for precise adjustment anywhere in the 0-1023 range.
--     ctrl_fine = (in + 7 * in_ref) / 8
--   Switching back to Normal restores full-range control immediately.
--
-- Timing:
--   Total pipeline latency: 7 clock cycles.
--   Sync delay line is 7 clocks.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture yuv_window_key of program_top is

    constant C_PROCESSING_DELAY_CLKS : integer := 7;
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 2;   -- 2-clock registered dry tap (Stage 0 + Stage 1a)

    --------------------------------------------------------------------------------
    -- Functions
    --------------------------------------------------------------------------------

    -- fine_knob: 1/8-sensitivity knob computation for Fine mode.
    --   Returns (raw + 7*ref) / 8, keeping the result centred on the locked
    --   reference and giving 1/8 the travel sensitivity of the full range.
    --   Implemented as (raw + 8*ref - ref) / 8 using shift arithmetic (no
    --   multiplier inferred).  Intermediate raw+8*ref fits in 14 bits (max
    --   9207); final result is at most (1023 + 7*1023)/8 = 1023, always in range.
    function fine_knob(
        raw : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        ref : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return unsigned is
        variable v_sum : unsigned(13 downto 0);  -- 14-bit to hold raw + 8*ref
    begin
        v_sum := ("0000" & unsigned(raw))
               + ("0" & ref & "000")   -- ref * 8 (shift left 3)
               - ("0000" & ref);       -- subtract ref → raw + 7*ref
        return v_sum(12 downto 3);     -- divide by 8
    end function;

    -- window_check: returns '1' if pixel is within the [lower, upper] window.
    --   Normal (lower <= upper): pass if lower <= pixel <= upper.
    --   Inverted (lower > upper): pass if pixel >= lower OR pixel <= upper.
    --   Crossing the knobs inverts the key naturally, per channel.
    function window_check(
        pixel : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        lower : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        upper : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return std_logic is
    begin
        if lower <= upper then
            if pixel >= lower and pixel <= upper then
                return '1';
            else
                return '0';
            end if;
        else
            -- Inverted window: pass values outside the (upper, lower) gap
            if pixel >= lower or pixel <= upper then
                return '1';
            else
                return '0';
            end if;
        end if;
    end function;

    --------------------------------------------------------------------------------
    -- Control Signals
    --------------------------------------------------------------------------------
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- Fine mode: registered S5 for edge detection; locked knob reference values
    signal s_fine           : std_logic := '0';
    type t_knob_refs is array (0 to 5) of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_in_ref         : t_knob_refs := (others => (others => '0'));

    -- Window key controls (registered in Stage 0 alongside data)
    signal s_low_y          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_low_u          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_low_v          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_high_y         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_high_u         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_high_v         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_show_matte     : std_logic := '0';  -- '1' = On/Show Matte
    signal s_matte_mode     : std_logic_vector(2 downto 0) := "010";  -- default: Logical AND
    -- LFSR / PRNG noise generators for matte modes 101 and 110
    signal s_vsync_n_d      : std_logic := '1';  -- registered vsync_n for edge detect
    signal s_hsync_n_d      : std_logic := '1';  -- registered hsync_n for edge detect
    signal s_lfsr           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := "0101010101";
    signal s_prng           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := "0011001100";

    --------------------------------------------------------------------------------
    -- Stage 0: Control Decode + Data Register Outputs (T+1)
    --------------------------------------------------------------------------------
    signal s_y_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_u_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_v_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_avid_d1        : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 0: Pre-computed LFSR/PRNG XOR Y (T+1)
    -- Computed alongside data_in registration to keep Stage 1a critical path
    -- free of XOR logic.
    --------------------------------------------------------------------------------
    signal s_lfsr_xor_y_d1  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_prng_xor_y_d1  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 0: Window Comparison Pre-compute Outputs (T+1)
    -- Carry chains (10-bit comparisons) are computed in p_control_decode alongside
    -- data registration so Stage 1a is pure LUT logic (no carry chains).
    -- Compares data_in against the PREVIOUS clock's threshold values (s_low_y etc.
    -- retain their registered values inside the process that drives them).
    -- Threshold lag is 1 clock — imperceptible at knob-turn speeds.
    --------------------------------------------------------------------------------
    signal s_pb_y_ge_low    : std_logic := '0';  -- data_in.y >= s_low_y[T-1]
    signal s_pb_y_le_high   : std_logic := '0';  -- data_in.y <= s_high_y[T-1]
    signal s_pb_lo_le_hi_y  : std_logic := '0';  -- s_low_y[T-1] <= s_high_y[T-1] (normal window)
    signal s_pb_u_ge_low    : std_logic := '0';
    signal s_pb_u_le_high   : std_logic := '0';
    signal s_pb_lo_le_hi_u  : std_logic := '0';
    signal s_pb_v_ge_low    : std_logic := '0';
    signal s_pb_v_le_high   : std_logic := '0';
    signal s_pb_lo_le_hi_v  : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 1a: Window Flag Register Outputs (T+2)
    -- Pure LUT logic: combines pre-registered comparison bits from Stage 0.
    -- No carry chains — Stage 0 holds all comparison carry logic.
    --------------------------------------------------------------------------------
    signal s_wf_in_y        : std_logic := '0';
    signal s_wf_in_u        : std_logic := '0';
    signal s_wf_in_v        : std_logic := '0';
    signal s_wf_in_any      : std_logic := '0';
    signal s_wf_in_all      : std_logic := '0';
    signal s_wf_y_m         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_u_m         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_v_m         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_y           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_u           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_v           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_avid        : std_logic := '0';
    signal s_wf_show_matte  : std_logic := '0';
    signal s_wf_matte_mode  : std_logic_vector(2 downto 0) := "010";
    signal s_wf_lfsr_xor_y  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wf_prng_xor_y  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 1b: Window Key Outputs (T+3)
    --------------------------------------------------------------------------------
    signal s_processed_y    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_u    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_v    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Blend Outputs (T+7)
    -- Dry input: original data delayed to T+3 (2 clocks after Stage 0)
    --------------------------------------------------------------------------------
    signal s_y_for_global   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_for_global   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_for_global   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    signal s_global_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_y_valid : std_logic;
    signal s_global_u_valid : std_logic;
    signal s_global_v_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Sync Delay Line (7 clocks — aligns hsync/vsync/field with pipeline output)
    --------------------------------------------------------------------------------
    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;

begin

    --------------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------------
    s_global_blend  <= unsigned(registers_in(7));  -- linear_potentiometer_12

    --------------------------------------------------------------------------------
    -- Stage 0: Control Decode + Data Register
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Registers all threshold knobs, switches, and the data_in Y/U/V samples
    -- in one clock so that controls and data arrive at Stage 1a together.
    -- Also pre-computes the 9 window comparison bits (carry chains) and the
    -- LFSR/PRNG XOR Y values so that Stage 1a is pure LUT logic.
    -- Hardware switch polarity: Off='0', On='1' (direct bit value).
    --
    -- Note on comparison timing: s_low_y / s_high_y etc. are written in this same
    -- process, so reads of those signals inside the process see their PREVIOUS
    -- registered values (VHDL signal semantics).  The comparison therefore uses the
    -- threshold from one clock ago, giving a 1-clock lag that is invisible at
    -- knob-turn speeds.  data_in.y/u/v are compared directly (no extra pipeline hop).
    --------------------------------------------------------------------------------
    p_control_decode : process(clk)
    begin
        if rising_edge(clk) then
            -- Data
            s_y_d1    <= data_in.y;
            s_u_d1    <= data_in.u;
            s_v_d1    <= data_in.v;
            s_avid_d1 <= data_in.avid;
            -- Pre-compute LFSR/PRNG XOR Y so Stage 1a has no XOR carry logic.
            s_lfsr_xor_y_d1 <= unsigned(s_lfsr) xor unsigned(data_in.y);
            s_prng_xor_y_d1 <= unsigned(s_prng) xor unsigned(data_in.y);
            -- Pre-compute per-channel window comparison bits (carry chains here,
            -- not in Stage 1a).  Reads current s_low_*/s_high_* (previous clock's
            -- registered values) vs data_in port inputs.
            if unsigned(data_in.y) >= s_low_y  then s_pb_y_ge_low   <= '1'; else s_pb_y_ge_low   <= '0'; end if;
            if unsigned(data_in.y) <= s_high_y then s_pb_y_le_high  <= '1'; else s_pb_y_le_high  <= '0'; end if;
            if s_low_y             <= s_high_y  then s_pb_lo_le_hi_y <= '1'; else s_pb_lo_le_hi_y <= '0'; end if;
            if unsigned(data_in.u) >= s_low_u  then s_pb_u_ge_low   <= '1'; else s_pb_u_ge_low   <= '0'; end if;
            if unsigned(data_in.u) <= s_high_u then s_pb_u_le_high  <= '1'; else s_pb_u_le_high  <= '0'; end if;
            if s_low_u             <= s_high_u  then s_pb_lo_le_hi_u <= '1'; else s_pb_lo_le_hi_u <= '0'; end if;
            if unsigned(data_in.v) >= s_low_v  then s_pb_v_ge_low   <= '1'; else s_pb_v_ge_low   <= '0'; end if;
            if unsigned(data_in.v) <= s_high_v then s_pb_v_le_high  <= '1'; else s_pb_v_le_high  <= '0'; end if;
            if s_low_v             <= s_high_v  then s_pb_lo_le_hi_v <= '1'; else s_pb_lo_le_hi_v <= '0'; end if;
            -- Switches — Off='0'/On='1'
            s_show_matte <= registers_in(6)(0);          -- '1' = On/Show Matte
            -- S2=bit3=MSB, S3=bit2, S4=bit1=LSB → concatenate to preserve {S2,S3,S4} order
            s_matte_mode <= registers_in(6)(3) & registers_in(6)(2) & registers_in(6)(1);
            -- Fine mode: register S5; latch reference values on Normal->Fine transition
            s_fine <= registers_in(6)(4);
            if s_fine = '0' and registers_in(6)(4) = '1' then
                s_in_ref(0) <= unsigned(registers_in(0));
                s_in_ref(1) <= unsigned(registers_in(1));
                s_in_ref(2) <= unsigned(registers_in(2));
                s_in_ref(3) <= unsigned(registers_in(3));
                s_in_ref(4) <= unsigned(registers_in(4));
                s_in_ref(5) <= unsigned(registers_in(5));
            end if;
            -- Compute threshold values: 1/8-sensitivity in Fine mode, full range Normal
            if registers_in(6)(4) = '1' then
                s_low_y  <= fine_knob(registers_in(0), s_in_ref(0));
                s_low_u  <= fine_knob(registers_in(1), s_in_ref(1));
                s_low_v  <= fine_knob(registers_in(2), s_in_ref(2));
                s_high_y <= fine_knob(registers_in(3), s_in_ref(3));
                s_high_u <= fine_knob(registers_in(4), s_in_ref(4));
                s_high_v <= fine_knob(registers_in(5), s_in_ref(5));
            else
                s_low_y  <= unsigned(registers_in(0));
                s_low_u  <= unsigned(registers_in(1));
                s_low_v  <= unsigned(registers_in(2));
                s_high_y <= unsigned(registers_in(3));
                s_high_u <= unsigned(registers_in(4));
                s_high_v <= unsigned(registers_in(5));
            end if;
        end if;
    end process p_control_decode;

    --------------------------------------------------------------------------------
    -- LFSR Noise Generator (frame-synced)
    -- 10-bit Fibonacci LFSR, polynomial x^10 + x^7 + 1 (primitive, period 1023).
    -- Reseeds from the current PRNG state on the falling edge of vsync_n so that
    -- each frame produces a different base pattern.
    -- Output is XOR'd with the Y pixel value in p_window_key to make the noise
    -- content-dependent (each luma level yields a different texture).
    -- Used by matte mode 101; gated by logical OR in p_window_key.
    --------------------------------------------------------------------------------
    p_lfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            s_vsync_n_d <= data_in.vsync_n;
            if s_vsync_n_d = '1' and data_in.vsync_n = '0' then
                -- Reseed from PRNG so each frame starts with a different pattern
                s_lfsr <= s_prng;
            else
                v_fb   := s_lfsr(9) xor s_lfsr(6);
                s_lfsr <= s_lfsr(8 downto 0) & v_fb;
            end if;
        end if;
    end process p_lfsr;

    --------------------------------------------------------------------------------
    -- PRNG Noise Generator (line-seeded)
    -- Same polynomial as the LFSR.  Reseeds from the current LFSR state on each
    -- falling edge of hsync_n so that every line starts with a different sequence.
    -- Because the LFSR itself varies each frame, the PRNG pattern is unique across
    -- both lines and frames.  Output is XOR'd with Y in p_window_key.
    -- Used by matte mode 110; gated by logical OR in p_window_key.
    --------------------------------------------------------------------------------
    p_prng : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            s_hsync_n_d <= data_in.hsync_n;
            if s_hsync_n_d = '1' and data_in.hsync_n = '0' then
                -- Reseed from LFSR at each line start for per-line variation
                s_prng <= s_lfsr;
            else
                v_fb   := s_prng(9) xor s_prng(6);
                s_prng <= s_prng(8 downto 0) & v_fb;
            end if;
        end if;
    end process p_prng;

    --------------------------------------------------------------------------------
    -- Stage 0b: Window Comparison Pre-compute
    --------------------------------------------------------------------------------
    -- Stage 1a: Window Check Flag Register
    -- Latency: 1 clock. Input T+1, output T+2.
    -- Combines the pre-registered comparison bits from Stage 0 using pure LUT
    -- logic (no carry chains) and registers all results — per-channel flags,
    -- OR/AND combinations, masked values, raw data, and controls — so that
    -- Stage 1b (p_window_key) sees only registered signals.
    --------------------------------------------------------------------------------
    p_window_check : process(clk)
        variable v_in_y, v_in_u, v_in_v : std_logic;
    begin
        if rising_edge(clk) then
            -- Combine pre-registered comparison bits (pure LUT logic, no carry chains)
            if s_pb_lo_le_hi_y = '1' then
                v_in_y := s_pb_y_ge_low and s_pb_y_le_high;
            else
                v_in_y := s_pb_y_ge_low or  s_pb_y_le_high;
            end if;
            if s_pb_lo_le_hi_u = '1' then
                v_in_u := s_pb_u_ge_low and s_pb_u_le_high;
            else
                v_in_u := s_pb_u_ge_low or  s_pb_u_le_high;
            end if;
            if s_pb_lo_le_hi_v = '1' then
                v_in_v := s_pb_v_ge_low and s_pb_v_le_high;
            else
                v_in_v := s_pb_v_ge_low or  s_pb_v_le_high;
            end if;
            -- Register per-channel flags and reductions
            s_wf_in_y   <= v_in_y;
            s_wf_in_u   <= v_in_u;
            s_wf_in_v   <= v_in_v;
            s_wf_in_any <= v_in_y or  v_in_u or  v_in_v;
            s_wf_in_all <= v_in_y and v_in_u and v_in_v;
            -- Register masked channel values (failing channel = 0)
            if v_in_y = '1' then s_wf_y_m <= unsigned(s_y_d1); else s_wf_y_m <= (others => '0'); end if;
            if v_in_u = '1' then s_wf_u_m <= unsigned(s_u_d1); else s_wf_u_m <= (others => '0'); end if;
            if v_in_v = '1' then s_wf_v_m <= unsigned(s_v_d1); else s_wf_v_m <= (others => '0'); end if;
            -- Delay raw data and control signals to keep them aligned with flags
            s_wf_y          <= s_y_d1;
            s_wf_u          <= s_u_d1;
            s_wf_v          <= s_v_d1;
            s_wf_avid       <= s_avid_d1;
            s_wf_show_matte <= s_show_matte;
            s_wf_matte_mode <= s_matte_mode;
            s_wf_lfsr_xor_y <= s_lfsr_xor_y_d1;
            s_wf_prng_xor_y <= s_prng_xor_y_d1;
        end if;
    end process p_window_check;

    --------------------------------------------------------------------------------
    -- Stage 1b: Matte Computation + Output
    -- Latency: 1 clock. Input T+2 (registered flags), output T+3.
    -- All inputs are registered values — no comparisons, just mux/logic.
    --
    -- Matte mode (s_wf_matte_mode = {S2, S3, S4}, S2=MSB):
    --   "000" Logical OR  : 1023 if any flag set, else 0
    --   "001" Bitwise OR  : OR of masked channel values
    --   "010" Logical AND : 1023 if all flags set, else 0
    --   "011" Bitwise AND : AND of masked channel values
    --   "100" Luma        : Y value passed, gated by AND
    --   "101" LFSR synced : frame-seeded noise XOR Y value, gated by OR
    --   "110" PRNG        : line-seeded noise XOR Y value, gated by OR
    --   "111" Passthrough : original Y/U/V, no keying
    --
    -- Show Matte On  : matte → Y; U=V=512 (neutral chroma, true monochrome)
    -- Show Matte Off : matte>0 passes original Y/U/V; matte=0 → black (Y=0,U=V=512)
    -- Blanking (avid='0'): raw data always passes to preserve sync structure.
    --------------------------------------------------------------------------------
    p_window_key : process(clk)
        variable v_matte : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if s_wf_avid = '0' then
                -- Blanking: pass raw data to preserve sync structure
                s_processed_y <= unsigned(s_wf_y);
                s_processed_u <= unsigned(s_wf_u);
                s_processed_v <= unsigned(s_wf_v);
            else
                -- Compute matte value: all inputs are registered flags/values
                case s_wf_matte_mode is
                    when "000" =>  -- Logical OR: B/W, any channel in-window passes
                        if s_wf_in_any = '1' then
                            v_matte := to_unsigned(1023, C_VIDEO_DATA_WIDTH);
                        else
                            v_matte := (others => '0');
                        end if;
                    when "001" =>  -- Bitwise OR: OR of masked values
                        v_matte := s_wf_y_m or s_wf_u_m or s_wf_v_m;
                    when "010" =>  -- Logical AND: B/W, all channels in-window pass
                        if s_wf_in_all = '1' then
                            v_matte := to_unsigned(1023, C_VIDEO_DATA_WIDTH);
                        else
                            v_matte := (others => '0');
                        end if;
                    when "011" =>  -- Bitwise AND: AND of masked values
                        v_matte := s_wf_y_m and s_wf_u_m and s_wf_v_m;
                    when "100" =>  -- Luma: Y value, gated by AND
                        if s_wf_in_all = '1' then
                            v_matte := unsigned(s_wf_y);
                        else
                            v_matte := (others => '0');
                        end if;
                    when "101" =>  -- LFSR synced: frame-seeded noise XOR Y, gated by OR
                        if s_wf_in_any = '1' then
                            v_matte := s_wf_lfsr_xor_y;
                        else
                            v_matte := (others => '0');
                        end if;
                    when "110" =>  -- PRNG: line-seeded noise XOR Y, gated by OR
                        if s_wf_in_any = '1' then
                            v_matte := s_wf_prng_xor_y;
                        else
                            v_matte := (others => '0');
                        end if;
                    when others =>  -- "111" Passthrough
                        v_matte := (others => '0');  -- unused; handled below
                end case;

                -- Drive outputs
                if s_wf_matte_mode = "111" then
                    -- Passthrough: output original Y/U/V
                    s_processed_y <= unsigned(s_wf_y);
                    s_processed_u <= unsigned(s_wf_u);
                    s_processed_v <= unsigned(s_wf_v);
                elsif s_wf_show_matte = '1' then
                    -- Show Matte: Y = matte; U/V = neutral chroma for true mono
                    s_processed_y <= v_matte;
                    s_processed_u <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
                    s_processed_v <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
                else
                    -- Normal: gate original Y/U/V with matte
                    if v_matte /= 0 then
                        s_processed_y <= unsigned(s_wf_y);
                        s_processed_u <= unsigned(s_wf_u);
                        s_processed_v <= unsigned(s_wf_v);
                    else
                        s_processed_y <= (others => '0');
                        s_processed_u <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
                        s_processed_v <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
                    end if;
                end if;
            end if;
            s_processed_valid <= s_wf_avid;
        end if;
    end process p_window_key;

    --------------------------------------------------------------------------------
    -- Delay Line: YUV dry input for global blend.
    -- 1-clock registered delay from Stage 1a: T+2+1=T+3, aligned with s_processed.
    --------------------------------------------------------------------------------
    p_global_dry_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_for_global <= unsigned(s_wf_y);
            s_u_for_global <= unsigned(s_wf_u);
            s_v_for_global <= unsigned(s_wf_v);
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+3, output T+7.
    --------------------------------------------------------------------------------
    interp_global_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_y_for_global, b=>s_processed_y, t=>s_global_blend,
                 result=>s_global_y, valid=>s_global_y_valid);

    interp_global_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_u_for_global, b=>s_processed_u, t=>s_global_blend,
                 result=>s_global_u, valid=>s_global_u_valid);

    interp_global_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_v_for_global, b=>s_processed_v, t=>s_global_blend,
                 result=>s_global_v, valid=>s_global_v_valid);

    --------------------------------------------------------------------------------
    -- Sync Delay Line (7 clocks — aligns hsync/vsync/field with pipeline output)
    --------------------------------------------------------------------------------
    p_sync_delay : process(clk)
        type t_sync_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
    begin
        if rising_edge(clk) then
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_hsync_n_delayed <= v_hsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_vsync_n_delayed <= v_vsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_field_n_delayed <= v_field_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_sync_delay;

    --------------------------------------------------------------------------------
    -- Output
    --------------------------------------------------------------------------------
    data_out.y       <= std_logic_vector(s_global_y);
    data_out.u       <= std_logic_vector(s_global_u);
    data_out.v       <= std_logic_vector(s_global_v);
    data_out.avid    <= s_global_y_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end architecture yuv_window_key;
