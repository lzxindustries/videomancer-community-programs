-- TomTom - Video delay / per-line horizontal smear for Videomancer
-- Copyright (C) 2026 claytron
-- License: GNU General Public License v3.0
--
-- Program Name:
--   TomTom
--
-- Author:
--   claytron
--
-- Overview:
--   Per-line horizontal pixel delay producing a VHS-style smear/shear effect.
--   Each horizontal line is offset by a variable number of pixels determined
--   by a per-line pattern (linear ramp, sine, or sawtooth), a drift speed,
--   and an LFSR noise component.
--
-- Architecture (planned stages):
--   Stage 0 : Control decode — extract and scale register values
--   Stage 1 : LFSR noise generator — lfsr16 free-runs; output sampled once
--             per line on hsync falling edge into s_noise_sample
--   Stage 2 : Per-line offset calculator — line counter + frame_phase_accumulator
--             feed a 2-clock multiply pipeline:
--               2a: scale line by shape, centre wave/delay/noise into signed inputs
--               2b: multiply + sum → s_line_offset (signed pixel shift for this line)
--   Stage 3 : Line pixel buffer (BRAM) — circular buffer, one line wide
--   Stage 4 : Read delayed pixel from buffer at offset address
--   Stage 5 : Channel mux — apply delay to Y only or Y+U+V
--   Stage 6 : Wet/dry blend via interpolator_u
--   Stage 7 : Bypass mux + sync delay to match total pipeline latency
--
-- Register Map:
--   rotary_potentiometer_1  : Delay depth (0=full left, 512=centre/off, 1023=full right)
--   rotary_potentiometer_2  : Shape (0=linear ramp, 512=sine, 1023=sawtooth)
--   rotary_potentiometer_3  : Speed (0=static, 1023=fast drift)
--   rotary_potentiometer_4  : Noise (0=clean, 1023=full VHS chaos)
--   toggle_switch_7         : Channel select (0=Y only, 1=Y+U+V)
--   toggle_switch_11        : Bypass (0=process, 1=bypass)
--   linear_potentiometer_12 : Blend wet/dry (0=dry, 1023=wet)
--
-- Timing:
--   Total pipeline latency: TBD (will be updated as stages are implemented)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tomtom of program_top is

    ---------------------------------------------------------------------------
    -- Pipeline latency constant
    -- Update this as stages are added so sync delay stays correct.
    ---------------------------------------------------------------------------
    constant C_LATENCY_CLKS : integer := 1;  -- passthrough for now

    ---------------------------------------------------------------------------
    -- Stage 0: Control signals (decoded from registers_in)
    ---------------------------------------------------------------------------
    signal s_delay  : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);
    signal s_shape  : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);
    signal s_speed  : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);
    signal s_noise  : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);
    signal s_ch_all : std_logic;  -- '0' = Y only, '1' = Y+U+V
    signal s_bypass : std_logic;  -- '1' = bypass
    signal s_blend  : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 1: LFSR noise generator signals
    ---------------------------------------------------------------------------
    -- Raw 16-bit pseudo-random output, advances every clock.
    signal s_lfsr16_out   : std_logic_vector(15 downto 0);
    -- Previous hsync state for falling-edge detection (also used by Stage 2).
    signal s_hsync_n_prev : std_logic := '1';
    -- Per-line noise sample: lower 10 bits of LFSR latched at each hsync
    -- falling edge. Stable for the entire line so every pixel on a given
    -- line receives the same noise contribution.
    signal s_noise_sample : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 2: Per-line offset calculator signals
    ---------------------------------------------------------------------------
    -- Frame-rate phase accumulator output: wraps at 2^16 once per (2^16/speed)
    -- frames, giving a smooth cross-frame drift.
    signal s_frame_phase   : unsigned(15 downto 0);

    -- Vertical line counter: increments each hsync, resets each vsync.
    signal s_vsync_n_prev  : std_logic             := '1';
    signal s_v_count       : unsigned(11 downto 0) := (others => '0');

    -- Stage 2a outputs (1 clk): signed, centred inputs for the multiply stage.
    signal s2a_wave_signed  : signed(10 downto 0) := (others => '0');
    signal s2a_delay_signed : signed(10 downto 0) := (others => '0');
    signal s2a_noise_signed : signed(10 downto 0) := (others => '0');
    signal s2a_noise_scale  : signed(10 downto 0) := (others => '0');

    -- Stage 2b output (2 clks total): signed horizontal pixel offset for
    -- the current line.  Consumed by Stage 3 (line buffer address calc).
    --
    -- shape_contrib = (wave × delay) >> 9  → up to ±512 px
    -- noise_contrib = (noise × scale) >> 14 → up to  ±32 px
    signal s_line_offset    : signed(10 downto 0) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Stage 0: Control decode
    -- Extract register values into named signals for readability.
    ---------------------------------------------------------------------------
    p_control_decode : process(clk)
    begin
        if rising_edge(clk) then
            s_delay  <= unsigned(registers_in(0));
            s_shape  <= unsigned(registers_in(1));
            s_speed  <= unsigned(registers_in(2));
            s_noise  <= unsigned(registers_in(3));
            s_ch_all <= registers_in(6)(0);  -- toggle_switch_7 bit
            s_bypass <= registers_in(6)(4);  -- toggle_switch_11 bit
            s_blend  <= unsigned(registers_in(11));
        end if;
    end process p_control_decode;

    ---------------------------------------------------------------------------
    -- Passthrough (temporary — stages will replace this)
    ---------------------------------------------------------------------------
    p_passthrough : process(clk)
    begin
        if rising_edge(clk) then
            data_out <= data_in;
        end if;
    end process p_passthrough;

    ---------------------------------------------------------------------------
    -- Stage 1a: LFSR16 — free-running pseudo-random source
    -- Advances one step every clock. Seed fed back to itself so it never
    -- reloads; the all-zeros lockup guard inside lfsr16 keeps it alive.
    ---------------------------------------------------------------------------
    u_lfsr16 : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => s_lfsr16_out,  -- feedback; load is never pulsed
            load   => '0',
            q      => s_lfsr16_out
        );

    ---------------------------------------------------------------------------
    -- Stage 1b: Per-line noise sample
    -- On the falling edge of hsync_n (start of sync pulse), latch the lower
    -- 10 bits of the LFSR into s_noise_sample. Held constant for the whole
    -- line — every pixel on a given line shares the same random offset,
    -- producing VHS-style per-line horizontal jitter.
    -- Stage 2 multiplies s_noise_sample by the s_noise knob to scale magnitude.
    ---------------------------------------------------------------------------
    p_noise_sample : process(clk)
    begin
        if rising_edge(clk) then
            s_hsync_n_prev <= data_in.hsync_n;
            if data_in.hsync_n = '0' and s_hsync_n_prev = '1' then
                s_noise_sample <= unsigned(s_lfsr16_out(C_PARAMETER_DATA_WIDTH - 1 downto 0));
            end if;
        end if;
    end process p_noise_sample;

    ---------------------------------------------------------------------------
    -- Stage 2: frame phase accumulator (speed-driven cross-frame drift)
    -- Advances the 16-bit phase register by s_speed on each vsync falling
    -- edge.  Upper 10 bits are mixed into the wave address in Stage 2a so
    -- the smear pattern scrolls vertically at a rate set by the Speed knob.
    ---------------------------------------------------------------------------
    u_frame_phase : entity work.frame_phase_accumulator
        generic map (
            G_PHASE_WIDTH => 16,
            G_SPEED_WIDTH => C_PARAMETER_DATA_WIDTH
        )
        port map (
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => s_speed,
            phase   => s_frame_phase
        );

    ---------------------------------------------------------------------------
    -- Stage 2: vertical line counter
    -- Increments s_v_count on each hsync falling edge; resets on vsync.
    -- s_hsync_n_prev is already driven by p_noise_sample — only read here.
    ---------------------------------------------------------------------------
    p_stage2_counters : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_n_prev <= data_in.vsync_n;
            if data_in.vsync_n = '0' and s_vsync_n_prev = '1' then
                s_v_count <= (others => '0');
            elsif data_in.hsync_n = '0' and s_hsync_n_prev = '1' then
                s_v_count <= s_v_count + 1;
            end if;
        end if;
    end process p_stage2_counters;

    ---------------------------------------------------------------------------
    -- Stage 2a: derive signed, centred inputs for the multiply stage (1 clk)
    --
    -- Wave phase: line position scaled by Shape knob (1×–5× cycles per frame)
    -- plus upper 10 bits of frame phase for drift.  Result wraps at 1024.
    --
    -- All three values are centred to −512..+511 so the multiply in Stage 2b
    -- produces a bipolar result with zero at knob centre.
    --
    --   s_shape = 0   → shape_scale = 256 → 1 sawtooth ramp per frame
    --   s_shape = 1023 → shape_scale = 1279 → ~5 ramps per frame
    ---------------------------------------------------------------------------
    p_stage2a : process(clk)
        variable v_shape_scale : unsigned(10 downto 0);
        variable v_scaled_line : unsigned(21 downto 0);
        variable v_wave_phase  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_shape_scale := resize(s_shape, 11) + 256;
            v_scaled_line := resize(s_v_count(9 downto 0), 11) * v_shape_scale;
            -- Mix in upper 10 bits of frame phase for vertical drift.
            v_wave_phase  := v_scaled_line(17 downto 8) + s_frame_phase(15 downto 6);

            -- Centre wave: 0..1023 → −512..+511
            s2a_wave_signed  <= signed(resize(v_wave_phase, 11)) - 512;
            -- Centre delay: 0..1023 → −512..+511
            -- (knob at 512 = no smear; CW/CCW = direction and depth)
            s2a_delay_signed <= signed(resize(s_delay, 11)) - 512;
            -- Centre noise sample: 0..1023 → −512..+511 (bipolar per-line jitter)
            s2a_noise_signed <= signed(resize(s_noise_sample, 11)) - 512;
            -- Noise knob as positive scale (0 = no jitter, 1023 = full scale)
            s2a_noise_scale  <= signed('0' & s_noise);
        end if;
    end process p_stage2a;

    ---------------------------------------------------------------------------
    -- Stage 2b: multiply and sum into s_line_offset (1 clk)
    --
    -- Two independent DSP multiplies, results shifted and added:
    --
    --   shape_contrib = (wave_signed × delay_signed) >> 9
    --     Arithmetic shift right 9 on the 22-bit product.
    --     Range: ±(512 × 512) >> 9 = ±512 pixels at full knob deflection.
    --
    --   noise_contrib = (noise_signed × noise_scale) >> 14
    --     Range: ±(512 × 1023) >> 14 ≈ ±32 pixels at full noise knob.
    --     Intentionally small — VHS jitter is subtle per line.
    --
    -- s_line_offset is the final signed pixel shift for every pixel on this
    -- line.  It is read by Stage 3 when computing the BRAM read address.
    ---------------------------------------------------------------------------
    p_stage2b : process(clk)
        variable v_shape_prod : signed(21 downto 0);
        variable v_noise_prod : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            v_shape_prod := s2a_wave_signed * s2a_delay_signed;
            v_noise_prod := s2a_noise_signed * s2a_noise_scale;
            s_line_offset <= resize(signed(v_shape_prod(21 downto 9)), 11)
                           + resize(signed(v_noise_prod(21 downto 14)), 11);
        end if;
    end process p_stage2b;

    -- TODO Stage 3 : Line pixel buffer (BRAM circular buffer)
    -- TODO Stage 4 : Read delayed pixel at computed offset address
    -- TODO Stage 5 : Channel mux (Y only vs Y+U+V)
    -- TODO Stage 6 : Wet/dry blend (interpolator_u)
    -- TODO Stage 7 : Bypass mux + sync delay (C_LATENCY_CLKS)

end architecture tomtom;
