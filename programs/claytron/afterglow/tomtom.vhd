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
--   Stage 2 : Per-line offset calculator — combines shape + speed + noise
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
    -- Control signals (decoded from registers_in)
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
    -- Previous hsync state for falling-edge detection.
    signal s_hsync_n_prev : std_logic := '1';
    -- Per-line noise sample: lower 10 bits of LFSR latched at each hsync
    -- falling edge. Stable for the entire line so every pixel on a given
    -- line receives the same noise contribution.
    signal s_noise_sample : unsigned(C_PARAMETER_DATA_WIDTH - 1 downto 0);

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
    -- 10 bits of the LFSR into s_noise_sample. This value is held constant
    -- for the whole line, so every pixel on that line shares the same random
    -- horizontal offset — exactly the VHS line-jitter character we want.
    -- Later stages multiply s_noise_sample by the s_noise knob to scale how
    -- much jitter is applied.
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

    -- TODO Stage 2 : Per-line offset calculator (shape + speed + noise)
    -- TODO Stage 3 : Line pixel buffer (BRAM circular buffer)
    -- TODO Stage 4 : Read delayed pixel at computed offset address
    -- TODO Stage 5 : Channel mux (Y only vs Y+U+V)
    -- TODO Stage 6 : Wet/dry blend (interpolator_u)
    -- TODO Stage 7 : Bypass mux + sync delay (C_LATENCY_CLKS)

end architecture tomtom;
