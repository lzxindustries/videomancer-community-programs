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
--   Stage 3 : Line pixel buffer — 3× dual-bank BRAM (Y/U/V); write current
--             line at wr_addr, read previous line at wr_addr+s_line_offset;
--             2-clock read latency
--   Stage 4 : Channel mux — 2-clock dry delay to match line buffer latency;
--             combinatorial mux: Y always smeared, U/V smeared (Y+U+V mode)
--             or dry (Y-only mode)
--   Stage 5 : Wet/dry blend — 3× interpolator_u (a=dry, b=wet, t=s_blend);
--             4-clock latency; valid output used as data_out.avid
--   Stage 6 : Bypass mux + sync delay (C_LATENCY_CLKS = 6)
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
--   Total pipeline latency: 6 clock cycles
--     2 clocks : video_line_buffer read latency (Stage 3)
--     4 clocks : interpolator_u blend (Stage 5)
--   Sync delay line (Stage 6) is C_LATENCY_CLKS = 6 deep to match.

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
    -- Total pipeline latency: 2 (line buffer) + 4 (interpolator_u) = 6 clocks.
    -- Stage 6 sync delay must match this value exactly.
    constant C_LATENCY_CLKS : integer := 6;
    -- Stage 3 line buffer depth: 2^11 = 2048 entries per bank.
    -- Covers HD active width (1920) with room to spare for offset wrap.
    constant C_LINE_DEPTH   : integer := 11;

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

    ---------------------------------------------------------------------------
    -- Stage 3: Line pixel buffer signals
    -- Three video_line_buffer instances (Y, U, V).  Each is a dual-bank BRAM:
    -- while one bank is written with the current line, the other is read to
    -- supply the previous line's pixels at a horizontally shifted address.
    -- Read latency: 2 clocks from i_rd_addr to o_data.
    ---------------------------------------------------------------------------
    -- Avid edge detection (for avid-gated write counter).
    signal s_avid_prev  : std_logic                              := '0';
    -- Bank select: '0' on vsync, toggled each hsync.
    signal s_lb_ab      : std_logic                              := '0';
    -- Write address: 0-indexed from first active pixel, advances during avid.
    signal s_lb_wr_addr : unsigned(C_LINE_DEPTH - 1 downto 0)   := (others => '0');
    -- Read address: wr_addr + s_line_offset, wraps naturally at 2048.
    signal s_lb_rd_addr : unsigned(C_LINE_DEPTH - 1 downto 0);
    -- Line buffer outputs: previous line's Y/U/V at the shifted read position.
    signal s_lb_y_out   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_lb_u_out   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_lb_v_out   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 4: Channel mux signals
    -- A 2-clock delay pipeline aligns data_in with the line buffer read
    -- latency so the dry U/V (Y-only mode) and the dry blend input (Stage 5)
    -- are in phase with the wet line-buffer outputs.
    ---------------------------------------------------------------------------
    -- 2-clock delay pipeline for Y/U/V and avid.
    -- avid delayed to match wet-path latency so interpolator valid tracks avid.
    signal s_dry_y_d1   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_dry_u_d1   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_dry_v_d1   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_avid_d1    : std_logic := '0';
    signal s_dry_y_d2   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_dry_u_d2   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_dry_v_d2   : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_avid_d2    : std_logic := '0';
    -- Mux outputs (combinatorial — no added latency).
    -- Y always takes the smeared line-buffer value.
    -- U/V are smeared (line buffer) in Y+U+V mode, or passed through dry
    -- in Y-only mode — giving luma-smear-only with live colour.
    signal s4_y         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s4_u         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s4_v         : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 5: Wet/dry blend signals
    -- Three interpolator_u instances (Y, U, V).
    -- result = a + (b - a) * t   where a=dry, b=wet, t=s_blend
    -- Latency: 4 clocks.  valid output tracks enable (s_avid_d2) through the
    -- pipeline — used as data_out.avid in Stage 6.
    ---------------------------------------------------------------------------
    signal s5_y_result  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s5_y_valid   : std_logic;
    signal s5_u_result  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s5_u_valid   : std_logic;
    signal s5_v_result  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s5_v_valid   : std_logic;

    ---------------------------------------------------------------------------
    -- Stage 6: Bypass delay line signals
    -- Sync and data signals from data_in shifted by C_LATENCY_CLKS clocks so
    -- they arrive at data_out at the same time as the processed result.
    -- Sync signals are always taken from this delay line.
    -- Y/U/V data is muxed: processed when s_bypass='0', delayed when '1'.
    ---------------------------------------------------------------------------
    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;
    signal s_y_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

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

    ---------------------------------------------------------------------------
    -- Stage 3: Line pixel buffer control
    -- Pixel counter (s_lb_wr_addr): resets to 0 on the first active pixel
    -- of each line (avid rising edge), increments each subsequent avid cycle.
    -- This mirrors the kintsugi pattern and ensures pixel 0 always maps to
    -- address 0 regardless of horizontal blanking duration.
    --
    -- Bank select (s_lb_ab): cleared on vsync, toggled on each hsync so the
    -- write and read banks swap every line.
    --
    -- s_hsync_n_prev and s_vsync_n_prev are driven by Stages 1/2 — read only.
    ---------------------------------------------------------------------------
    p_stage3_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_prev <= data_in.avid;

            -- Write address: 0 at first active pixel, then increment
            if data_in.avid = '1' then
                if s_avid_prev = '0' then
                    s_lb_wr_addr <= (others => '0');
                else
                    s_lb_wr_addr <= s_lb_wr_addr + 1;
                end if;
            end if;

            -- Bank toggle: reset on vsync, flip on each hsync
            if data_in.vsync_n = '0' and s_vsync_n_prev = '1' then
                s_lb_ab <= '0';
            elsif data_in.hsync_n = '0' and s_hsync_n_prev = '1' then
                s_lb_ab <= not s_lb_ab;
            end if;
        end if;
    end process p_stage3_ctrl;

    ---------------------------------------------------------------------------
    -- Stage 3: Read address — current pixel position plus signed line offset.
    -- Natural 11-bit wrap (mod 2048) means extreme offsets produce edge-wrap
    -- artefacts rather than hard clipping — a pleasing VHS character.
    ---------------------------------------------------------------------------
    s_lb_rd_addr <= unsigned(
        (signed('0' & s_lb_wr_addr) + resize(s_line_offset, 12))(10 downto 0)
    );

    ---------------------------------------------------------------------------
    -- Stage 3: Line buffer instances — one per channel (Y, U, V).
    -- Each dual-bank BRAM writes the current line while reading the previous
    -- line at the offset address. Read latency: 2 clocks.
    ---------------------------------------------------------------------------
    lb_y : entity work.video_line_buffer
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_DEPTH => C_LINE_DEPTH)
        port map (
            clk       => clk,
            i_ab      => s_lb_ab,
            i_wr_addr => s_lb_wr_addr,
            i_rd_addr => s_lb_rd_addr,
            i_data    => data_in.y,
            o_data    => s_lb_y_out
        );

    lb_u : entity work.video_line_buffer
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_DEPTH => C_LINE_DEPTH)
        port map (
            clk       => clk,
            i_ab      => s_lb_ab,
            i_wr_addr => s_lb_wr_addr,
            i_rd_addr => s_lb_rd_addr,
            i_data    => data_in.u,
            o_data    => s_lb_u_out
        );

    lb_v : entity work.video_line_buffer
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_DEPTH => C_LINE_DEPTH)
        port map (
            clk       => clk,
            i_ab      => s_lb_ab,
            i_wr_addr => s_lb_wr_addr,
            i_rd_addr => s_lb_rd_addr,
            i_data    => data_in.v,
            o_data    => s_lb_v_out
        );

    ---------------------------------------------------------------------------
    -- Stage 4: Dry delay pipeline
    -- Delays data_in Y/U/V by exactly 2 clocks to match the line buffer read
    -- latency.  s_dry_*_d2 is used in two places:
    --   • As the dry U/V in Y-only channel mode (below).
    --   • As the 'a' (dry) input to each interpolator_u in Stage 5.
    ---------------------------------------------------------------------------
    p_stage4_dry_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_dry_y_d1 <= data_in.y;
            s_dry_u_d1 <= data_in.u;
            s_dry_v_d1 <= data_in.v;
            s_avid_d1  <= data_in.avid;
            s_dry_y_d2 <= s_dry_y_d1;
            s_dry_u_d2 <= s_dry_u_d1;
            s_dry_v_d2 <= s_dry_v_d1;
            s_avid_d2  <= s_avid_d1;
        end if;
    end process p_stage4_dry_delay;

    ---------------------------------------------------------------------------
    -- Stage 4: Channel mux (combinatorial)
    -- Y always comes from the line buffer — the smear is always on luma.
    -- U/V follow the channel toggle:
    --   s_ch_all = '0' (Y)      → U/V from current line (s_dry_*_d2)
    --   s_ch_all = '1' (Y+U+V) → U/V from line buffer (s_lb_*_out)
    -- No pipeline register here; Stage 5's interpolator input register
    -- absorbs the combinatorial path.
    ---------------------------------------------------------------------------
    s4_y <= s_lb_y_out;
    s4_u <= s_lb_u_out when s_ch_all = '1' else s_dry_u_d2;
    s4_v <= s_lb_v_out when s_ch_all = '1' else s_dry_v_d2;

    ---------------------------------------------------------------------------
    -- Stage 5: Wet/dry blend via interpolator_u (4-clock latency each)
    --
    -- For each channel:
    --   a = dry  = s_dry_*_d2   (current-line pixel, 2-clock delayed)
    --   b = wet  = s4_*          (channel-mux output: smeared line-buffer data)
    --   t = s_blend              (slider, 0 = full dry, 1023 = full wet)
    --
    -- enable = s_avid_d2 so that the valid output tracks avid through the
    -- 4-clock pipeline, arriving as a correctly-timed avid for Stage 6.
    --
    -- In Y-only channel mode s4_u = s4_v = s_dry_*_d2, so a = b and the
    -- blend has no effect on chroma — regardless of the Blend knob position.
    ---------------------------------------------------------------------------
    interp_y : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s_avid_d2,
            a      => unsigned(s_dry_y_d2),
            b      => unsigned(s4_y),
            t      => s_blend,
            result => s5_y_result,
            valid  => s5_y_valid
        );

    interp_u : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s_avid_d2,
            a      => unsigned(s_dry_u_d2),
            b      => unsigned(s4_u),
            t      => s_blend,
            result => s5_u_result,
            valid  => s5_u_valid
        );

    interp_v : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s_avid_d2,
            a      => unsigned(s_dry_v_d2),
            b      => unsigned(s4_v),
            t      => s_blend,
            result => s5_v_result,
            valid  => s5_v_valid
        );

    ---------------------------------------------------------------------------
    -- Stage 6: Bypass delay line
    -- Shifts hsync_n, vsync_n, field_n and Y/U/V through a C_LATENCY_CLKS-deep
    -- shift register so the bypass path arrives at data_out in phase with the
    -- processed path.  Uses the same variable-prepend pattern as yuv_bit_logic.
    ---------------------------------------------------------------------------
    p_stage6_delay : process(clk)
        type t_sync_delay is array (0 to C_LATENCY_CLKS - 1) of std_logic;
        type t_data_delay is array (0 to C_LATENCY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_hsync : t_sync_delay := (others => '1');
        variable v_vsync : t_sync_delay := (others => '1');
        variable v_field : t_sync_delay := (others => '1');
        variable v_y     : t_data_delay := (others => (others => '0'));
        variable v_u     : t_data_delay := (others => (others => '0'));
        variable v_v     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync := data_in.hsync_n & v_hsync(0 to C_LATENCY_CLKS - 2);
            v_vsync := data_in.vsync_n & v_vsync(0 to C_LATENCY_CLKS - 2);
            v_field := data_in.field_n & v_field(0 to C_LATENCY_CLKS - 2);
            v_y     := data_in.y       & v_y(0 to C_LATENCY_CLKS - 2);
            v_u     := data_in.u       & v_u(0 to C_LATENCY_CLKS - 2);
            v_v     := data_in.v       & v_v(0 to C_LATENCY_CLKS - 2);
            s_hsync_n_delayed <= v_hsync(C_LATENCY_CLKS - 1);
            s_vsync_n_delayed <= v_vsync(C_LATENCY_CLKS - 1);
            s_field_n_delayed <= v_field(C_LATENCY_CLKS - 1);
            s_y_delayed       <= v_y(C_LATENCY_CLKS - 1);
            s_u_delayed       <= v_u(C_LATENCY_CLKS - 1);
            s_v_delayed       <= v_v(C_LATENCY_CLKS - 1);
        end if;
    end process p_stage6_delay;

    ---------------------------------------------------------------------------
    -- Stage 6: Output mux and assignment
    -- Y/U/V: processed result when s_bypass='0', delayed input when '1'.
    -- avid: always s5_y_valid — the interpolator propagates avid timing
    --       naturally through its 4-clock pipeline regardless of bypass.
    -- Sync: always the C_LATENCY_CLKS-delayed versions.
    ---------------------------------------------------------------------------
    data_out.y <= std_logic_vector(s5_y_result) when s_bypass = '0' else s_y_delayed;
    data_out.u <= std_logic_vector(s5_u_result) when s_bypass = '0' else s_u_delayed;
    data_out.v <= std_logic_vector(s5_v_result) when s_bypass = '0' else s_v_delayed;
    data_out.avid    <= s5_y_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end architecture tomtom;
