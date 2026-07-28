-- Copyright (C) 2026 Jacob Roufa
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Program Name:
--   Digital Delay
--
-- Author:
--   Jacob Roufa
--
-- Overview:
--   Advanced feedback video processor providing frame strobe, threshold gating,
--   multi-echo delay with Fibonacci/Exponential spacing, independent Y/U/V channel
--   delays, and blend control. The design uses a pipelined architecture with a
--   total latency of 13 clock cycles, ensuring proper timing alignment between
--   the video data and sync signals.
--
-- Architecture:
--   Frame Strobe Stage:
--     - Temporal posterization: holds entire frames to reduce frame rate
--     - 0-127 frames hold time (~2.1 seconds max @ 60Hz)
--     - Creates stop-motion, stutter, or freeze-frame effects
--     - 1 clock latency
--
--   Threshold Gate Stage:
--     - Selective feedback based on pixel luminance (Y channel)
--     - Bidirectional: -100% to +100% (center = 0% = unity/disabled)
--     - Negative values: pass only shadows (Y <= threshold)
--     - Positive values: pass only highlights (Y >= threshold)
--     - Gates circular buffer writes for selective persistence
--     - 1 clock latency
--
--   Circular Buffer:
--     - 512-pixel delay per channel (Y, U, V independent)
--     - Block RAM inference with synchronous reads
--     - Supports parallel multi-echo reads from single buffer
--     - No additional RAM needed for multi-echo operation
--
--   Multi-Echo Processing:
--     - Echo Address Calculation: Fibonacci or Exponential spacing
--       * Fibonacci: 1/8, 2/8, 5/8, 8/8 (golden ratio, organic echoes)
--       * Exponential: 1/8, 2/8, 4/8, 8/8 (powers of 2, reverb-like)
--       * All ratios use constant power-of-2 divisions (optimized to bit shifts)
--       * Pipelined: 2 clock latency (amount calculation + address calculation)
--     - Echo Density Control: Progressive repeat activation (0-100%)
--       * 0-25%: 1 echo active (shortest delay, sparse)
--       * 25-50%: 2 echoes averaged (moderate density)
--       * 50-75%: 3 echoes averaged (dense repeats)
--       * 75-100%: 4 echoes averaged (maximum reverb density)
--       * 1 clock latency for echo count calculation
--     - Synchronous RAM Read: 1 clock latency
--     - Echo Mixing: Weighted averaging based on active echo count
--       * Division by 2, 4: exact (shift right)
--       * Division by 3: approximated as ÷4 for timing (75% amplitude)
--       * 1 clock latency
--
--   Blend/Interpolation Stage:
--     - Three interpolator_u instances for Y, U, V channels
--     - Blends current pixels with multi-echo delayed output
--     - 0% = current only, 100% = delayed only
--     - 4 clock latency per channel
--
--   Bypass Path:
--     - Optional bypass mode to pass through unprocessed video
--     - 9-clock delay line compensates for processing pipeline before interpolator
--     - Total bypass latency: 13 clocks (matches processing path)
--
-- Submodules:
--   interpolator_u: Unsigned linear interpolator
--     - Performs linear interpolation: result = a + (b - a) * t
--     - Used for blend/decay effects where t is the effect amount
--     - 4 clock pipeline stages
--
-- Register Map:
--   Compatible with Videomancer ABI 1.x
--   Register 0: Strobe amount
--     Bits [9:3]: 0-127 frames (7-bit range, ~2.1 sec @ 60Hz)
--   Register 1: Threshold level (0-1023)
--     Display: -100% to +100%, where 512 = 0% (unity/disabled)
--     < 512: pass shadows, > 512: pass highlights
--   Register 2: Echo density (0-1023)
--     Display: 0-100%, controls active echo count (1-4 echoes)
--   Register 3: Y channel delay
--     Bits [9:1]: 0-511 pixels (9-bit range)
--   Register 4: U channel delay
--     Bits [9:1]: 0-511 pixels (9-bit range)
--   Register 5: V channel delay
--     Bits [9:1]: 0-511 pixels (9-bit range)
--   Register 6: Control flags
--     Bit 0: Enable strobe
--     Bit 1: Enable delay (consolidated Y/U/V enable)
--     Bit 2: Multi-echo enable (0=single echo, 1=multi-echo mode)
--     Bit 3: Echo spacing (0=Fibonacci, 1=Exponential)
--     Bit 4: Bypass enable (1=bypass all processing)
--   Register 7: Effect amount/blend (0-1023)
--     Display: 0-100%, decay/feedback amount
--     0 = current pixels only, 1023 = delayed pixels only
--
-- Timing:
--   Total pipeline latency: 13 clocks
--   - Frame strobe: 1 clock
--   - Threshold gate + buffer write: 1 clock
--   - Echo amount calculation: 1 clock (pipelined)
--   - Echo address calculation: 1 clock (pipelined)
--   - Synchronous RAM read: 1 clock
--   - Active echo count: 1 clock
--   - Echo mixing: 1 clock
--   - Interpolator: 4 clocks
--   - Output mux: combinatorial (0 clocks)
--   All sync signals are delayed to match video data path
--
-- Resource Usage (ICE40 HX4K):
--   - Block RAM: 24 of 32 blocks (75%) for 512-pixel × 3 channel buffers
--   - Logic Cells: 3957 of 7680 LCs (52%)
--   - I/O Pins: 107 of 256 (42%)
--   - PLLs: 0-1 of 2 (depending on video mode)
--   - Timing: Fmax = 69-84 MHz (meets 74.25 MHz HD and 27 MHz SD requirements)
--   - Optimizations:
--     * No variable divisions (constant power-of-2 only)
--     * Parallel echo reads from single buffer
--     * Pipelined address calculations for timing closure
--     * Division by 3 approximated as ÷4 (shift) for timing
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture digital_delay of program_top is
    -- AUTO_IO_ALIGN_PAD declarations
    type t_io_align_stream is record
        y       : std_logic_vector(10-1 downto 0);
        u       : std_logic_vector(10-1 downto 0);
        v       : std_logic_vector(10-1 downto 0);
        avid    : std_logic;
        hsync_n : std_logic;
        vsync_n : std_logic;
        field_n : std_logic;
    end record;
    signal s_io_align_in : t_io_align_stream;
    signal s_io_pad_0 : t_io_align_stream;
    signal s_io_pad_1 : t_io_align_stream;
    signal s_io_pad_2 : t_io_align_stream;

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    -- Circular buffer size per channel (Y, U, V)
    constant C_PIXEL_DELAY_SIZE : integer := 512;
    -- Maximum frame strobe count (7-bit range, ~2.1 sec @ 60Hz)
    constant C_MAX_FRAME_STROBE : integer := 127;
    -- Processing latency before interpolator (for bypass delay matching)
    -- 1 (strobe) + 1 (threshold) + 2 (echo addressing) + 1 (RAM read) + 1 (echo count) + 1 (mix) + 2 (margin) = 9 clocks
    constant C_BASE_LATENCY     : integer := 9;

    --------------------------------------------------------------------------------
    -- Control Signals (from registers)
    --------------------------------------------------------------------------------
    signal s_bypass_enable   : std_logic;
    signal s_strobe_enable   : std_logic;
    signal s_delay_enable    : std_logic;  -- Consolidated Y/U/V enable
    signal s_multi_enable    : std_logic;  -- Multi-echo enable
    signal s_multi_shape     : std_logic;  -- Echo spacing (Fib/Exp)
    signal s_effect_amount   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_strobe_frames   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_threshold_level : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);  -- Threshold gate
    signal s_multitap_param  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);  -- Echo density parameter
    signal s_y_delay_pixels  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_delay_pixels  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_delay_pixels  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_strobe_amount   : integer range 0 to C_MAX_FRAME_STROBE;
    signal s_y_delay_amount  : integer range 0 to C_PIXEL_DELAY_SIZE;
    signal s_u_delay_amount  : integer range 0 to C_PIXEL_DELAY_SIZE;
    signal s_v_delay_amount  : integer range 0 to C_PIXEL_DELAY_SIZE;
    signal s_tap_count       : integer range 1 to 4 := 1;  -- Active echo count (legacy signal)
    signal s_tap_decay       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);  -- Echo density parameter

    -- Frame strobe effect signals
    -- Holds entire frames for temporal posterization effect
    signal s_frame_counter   : integer range 0 to C_MAX_FRAME_STROBE := 0;
    signal s_frame_update    : std_logic := '1';  -- Signal to capture new frame
    signal s_prev_vsync      : std_logic := '1';
    signal s_strobe_y, s_strobe_u, s_strobe_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_strobe_avid : std_logic;

    -- Pixel delay buffers (circular buffers using block RAM)
    -- Indexed 0 to C_PIXEL_DELAY_SIZE-1 for circular buffer addressing
    -- Synthesis tool will infer block RAM from these arrays with synchronous access
    type t_pixel_delay is array (0 to C_PIXEL_DELAY_SIZE - 1) of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_pixel_buf_y : t_pixel_delay := (others => (others => '0'));
    signal s_pixel_buf_u : t_pixel_delay := (others => (others => '0'));
    signal s_pixel_buf_v : t_pixel_delay := (others => (others => '0'));

    -- Circular buffer write pointer (increments each pixel)
    signal s_write_ptr : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;

    -- Read address calculation (registered for synchronous RAM access)
    signal s_read_addr_y : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_read_addr_u : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_read_addr_v : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;

    -- Multi-echo read addresses (4 echoes per channel)
    signal s_tap1_addr_y, s_tap1_addr_u, s_tap1_addr_v : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_tap2_addr_y, s_tap2_addr_u, s_tap2_addr_v : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_tap3_addr_y, s_tap3_addr_u, s_tap3_addr_v : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_tap4_addr_y, s_tap4_addr_u, s_tap4_addr_v : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;

    -- Multi-echo delay amounts (calculated based on spacing mode)
    signal s_tap1_amount, s_tap2_amount, s_tap3_amount, s_tap4_amount : integer range 0 to C_PIXEL_DELAY_SIZE := 0;

    -- Pipeline registers for echo amounts (break critical path for timing)
    signal s_tap1_amount_r, s_tap2_amount_r, s_tap3_amount_r, s_tap4_amount_r : integer range 0 to C_PIXEL_DELAY_SIZE := 0;

    -- Pipeline stages
    signal s_delayed_y, s_current_y : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_delayed_u, s_current_u : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_delayed_v, s_current_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_delayed_avid : std_logic;

    -- Multi-echo values (after synchronous reads)
    signal s_tap1_y, s_tap1_u, s_tap1_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_tap2_y, s_tap2_u, s_tap2_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_tap3_y, s_tap3_u, s_tap3_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_tap4_y, s_tap4_u, s_tap4_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- Pipeline register for active echo count (breaks timing critical path)
    signal s_active_taps : integer range 1 to 4 := 4;

    -- Mixed echo outputs (weighted sum of active echoes)
    signal s_mixed_y, s_mixed_u, s_mixed_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_mixed_avid : std_logic;

    -- Blend outputs
    signal s_blend_y, s_blend_u, s_blend_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_valid : std_logic;

    -- Bypass delay line
    signal s_hsync_delayed, s_vsync_delayed, s_field_delayed : std_logic;
    signal s_y_bypassed, s_u_bypassed, s_v_bypassed : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

begin
    --------------------------------------------------------------------------------
    -- Register mapping (parameters assigned in TOML order)
    --------------------------------------------------------------------------------
    s_strobe_frames   <= unsigned(registers_in(0));  -- Register 0: Frame strobe amount
    s_threshold_level <= unsigned(registers_in(1));  -- Register 1: Threshold level
    s_multitap_param  <= unsigned(registers_in(2));  -- Register 2: Echo density parameter
    s_y_delay_pixels  <= unsigned(registers_in(3));  -- Register 3: Y channel delay
    s_u_delay_pixels  <= unsigned(registers_in(4));  -- Register 4: U channel delay
    s_v_delay_pixels  <= unsigned(registers_in(5));  -- Register 5: V channel delay
    s_strobe_enable   <= registers_in(6)(0);         -- Bit 0: Enable frame strobe
    s_delay_enable    <= registers_in(6)(1);         -- Bit 1: Enable delay (consolidated Y/U/V)
    s_multi_enable    <= registers_in(6)(2);         -- Bit 2: Multi-echo enable
    s_multi_shape     <= registers_in(6)(3);         -- Bit 3: Echo spacing (Fib/Exp)
    s_bypass_enable   <= registers_in(6)(4);         -- Bit 4: Bypass all processing
    s_effect_amount   <= unsigned(registers_in(7));  -- Register 7: Decay/blend amount

    -- Map 10-bit register values to control ranges using bit slicing (no arithmetic)
    -- Strobe: bits [9:3] → 0-127 frames (7 bits, ~2.1 sec @ 60Hz)
    -- Threshold: full 10-bit value → 0-1023 (display: -100% to +100%, center=512=0%)
    -- Echo density: full 10-bit value → controls active echo count (1-4 echoes progressively)
    -- Y/U/V delays: bits [9:1] → 0-511 pixels each (9 bits)
    -- All use bit slicing for timing-safe implementation
    s_strobe_amount  <= to_integer(s_strobe_frames(9 downto 3));
    s_y_delay_amount <= to_integer(s_y_delay_pixels(9 downto 1));
    s_u_delay_amount <= to_integer(s_u_delay_pixels(9 downto 1));
    s_v_delay_amount <= to_integer(s_v_delay_pixels(9 downto 1));

    -- Echo density parameter interpretation
    -- Full 10-bit value controls active echo count (progressive activation)
    -- Decoded in p_active_taps process: <256=1 echo, <512=2 echoes, <768=3 echoes, >=768=4 echoes
    s_tap_count <= to_integer(s_multitap_param(9 downto 8)) + 1 when (s_multi_enable = '0' and s_multi_shape = '1') else 1;
    s_tap_decay <= s_multitap_param;

    --------------------------------------------------------------------------------
    -- Frame Strobe Effect
    --
    -- Purpose: Creates temporal posterization by holding entire video frames
    -- Operation:
    --   - Counts frame boundaries using vsync_n signal
    --   - Amount = 0: updates every frame (pass-through, no strobe)
    --   - Amount > 0: holds frame for N frames, then captures next
    --   - Effect: reduces effective frame rate (e.g., 60fps → 30fps, 20fps, etc.)
    --
    -- Visual Result:
    --   - 0 frames: no effect (pass-through)
    --   - Low values (1-5 frames): subtle judder/stutter
    --   - Medium values (10-30 frames): stop-motion animation effect
    --   - High values (60-127 frames): freeze-frame that updates periodically
    --
    -- Processing Order: Runs BEFORE pixel delay to reduce processing load
    --------------------------------------------------------------------------------
    p_frame_strobe : process(clk)
    begin
        if rising_edge(clk) then
            -- Track vsync falling edge to detect new frame start
            s_prev_vsync <= data_in.vsync_n;

            -- Detect frame boundary (vsync falling edge: '1' → '0')
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                if s_strobe_enable = '1' and s_strobe_amount > 0 then
                    -- Frame strobe enabled with amount > 0: count frames and determine update
                    if s_frame_counter >= s_strobe_amount then
                        s_frame_counter <= 0;
                        s_frame_update <= '1';  -- Capture this frame
                    else
                        s_frame_counter <= s_frame_counter + 1;
                        s_frame_update <= '0';  -- Hold previous frame
                    end if;
                else
                    -- Frame strobe disabled or amount = 0: always update (pass through)
                    s_frame_counter <= 0;
                    s_frame_update <= '1';
                end if;
            end if;

            -- Pixel capture: update or hold based on frame_update flag
            if data_in.avid = '1' then
                if s_frame_update = '1' then
                    -- Capture new pixels from this frame
                    s_strobe_y <= unsigned(data_in.y);
                    s_strobe_u <= unsigned(data_in.u);
                    s_strobe_v <= unsigned(data_in.v);
                else
                    -- Hold pixels from previous captured frame
                    -- (s_strobe_y/u/v retain their values)
                end if;
            end if;

            -- Pass through timing signals
            s_strobe_avid <= data_in.avid;
        end if;
    end process p_frame_strobe;

    --------------------------------------------------------------------------------
    -- Pixel Delay Effect with Full Y/U/V Independent Control
    --
    -- Purpose: Creates horizontal trailing/echo by delaying pixel data
    -- Operation:
    --   - Maintains circular buffers in block RAM (Y, U, V channels)
    --   - Uses write pointer that increments each pixel
    --   - Calculates read addresses: (write_ptr - delay_amount) mod buffer_size
    --   - Y channel: independent enable + delay amount (0-511 pixels)
    --   - U channel: independent enable + delay amount (0-511 pixels)
    --   - V channel: independent enable + delay amount (0-511 pixels)
    --   - Synchronous RAM reads for proper block RAM inference
    --   - When disabled or delay = 0: passes through current pixel
    --   - Feeds both current and delayed pixels to blend stage
    --
    -- Visual Result:
    --   - Creates horizontal displacement/echo effect per channel
    --   - At 720p (1280px wide): 511px = 40% screen width (dramatic echoes)
    --   - At 480i (720px wide): 511px = 71% screen width (extreme displacement)
    --   - Full independent control enables complex chromatic aberration effects
    --   - Can delay Y, U, V by different amounts for creative color separation
    --
    -- Processing Order: Runs AFTER frame strobe
    --   - Benefit: If strobe holds frames, delay processes fewer unique pixels
    --   - Example: 30fps strobe @ 60Hz = 50% less pixel processing
    --
    -- Resource Usage: 512 pixels × 30 bits/pixel = 15,360 bits block RAM (~12 blocks)
    --------------------------------------------------------------------------------
    p_pixel_delay_write : process(clk)
        variable v_write_enable : std_logic;
    begin
        if rising_edge(clk) then
            if s_strobe_avid = '1' then
                -- Threshold gate: determine if pixel should be written to buffer
                -- Center point (512) = unity/disabled (all pass)
                -- < 512: Pass only pixels below threshold (shadow feedback)
                -- > 512: Pass only pixels above threshold (highlight feedback)
                if s_threshold_level < 512 then
                    -- Below-threshold mode: pass dark pixels
                    v_write_enable := '1' when s_strobe_y <= s_threshold_level else '0';
                elsif s_threshold_level > 512 then
                    -- Above-threshold mode: pass bright pixels
                    v_write_enable := '1' when s_strobe_y >= s_threshold_level else '0';
                else
                    -- Unity mode (512): all pixels pass
                    v_write_enable := '1';
                end if;

                -- Write current pixel to circular buffer at write pointer (gated by threshold)
                if v_write_enable = '1' then
                    s_pixel_buf_y(s_write_ptr) <= s_strobe_y;
                    s_pixel_buf_u(s_write_ptr) <= s_strobe_u;
                    s_pixel_buf_v(s_write_ptr) <= s_strobe_v;
                end if;

                -- Increment write pointer (circular) regardless of threshold
                if s_write_ptr = C_PIXEL_DELAY_SIZE - 1 then
                    s_write_ptr <= 0;
                else
                    s_write_ptr <= s_write_ptr + 1;
                end if;

                -- Calculate read addresses for each channel (circular buffer math)
                -- read_addr = (write_ptr - delay_amount + buffer_size) mod buffer_size
                -- Adding buffer_size prevents negative results before modulo
                if s_delay_enable = '1' and s_y_delay_amount > 0 then
                    s_read_addr_y <= (s_write_ptr - s_y_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_read_addr_y <= s_write_ptr;  -- Read current position (no delay)
                end if;

                if s_delay_enable = '1' and s_u_delay_amount > 0 then
                    s_read_addr_u <= (s_write_ptr - s_u_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_read_addr_u <= s_write_ptr;  -- Read current position (no delay)
                end if;

                if s_delay_enable = '1' and s_v_delay_amount > 0 then
                    s_read_addr_v <= (s_write_ptr - s_v_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_read_addr_v <= s_write_ptr;  -- Read current position (no delay)
                end if;

                -- Multi-echo delay address calculations
                -- Calculate echo delay amounts based on spacing mode (Fibonacci/Exponential only)
                -- Mode decode: s_multi_enable (0=OFF, 1=ON), s_multi_shape (0=Fibonacci, 1=Exponential)
                --   Echo OFF: Single echo (all echoes at base delay)
                --   Echo ON + Fibonacci: 1/8, 2/8, 5/8, 8/8 spacing
                --   Echo ON + Exponential: 1/8, 2/8, 4/8, 8/8 spacing
                if s_multi_enable = '0' then
                    -- Multi-echo OFF: All echoes at base delay (single echo behavior)
                    s_tap1_amount <= s_y_delay_amount;
                    s_tap2_amount <= s_y_delay_amount;
                    s_tap3_amount <= s_y_delay_amount;
                    s_tap4_amount <= s_y_delay_amount;
                elsif s_multi_shape = '0' then
                    -- Fibonacci: 1/8, 2/8, 5/8, 8/8 (synthesis tool optimizes to bit shifts)
                    s_tap1_amount <= s_y_delay_amount / 8;                  -- 1/8
                    s_tap2_amount <= s_y_delay_amount / 4;                  -- 2/8
                    s_tap3_amount <= s_y_delay_amount / 2 + s_y_delay_amount / 8;  -- 5/8 = 4/8 + 1/8
                    s_tap4_amount <= s_y_delay_amount;                      -- 8/8 (full base)
                else
                    -- Exponential: 1/8, 2/8, 4/8, 8/8 (synthesis tool optimizes to bit shifts)
                    s_tap1_amount <= s_y_delay_amount / 8;   -- 1/8 base
                    s_tap2_amount <= s_y_delay_amount / 4;   -- 2/8 base
                    s_tap3_amount <= s_y_delay_amount / 2;   -- 4/8 base
                    s_tap4_amount <= s_y_delay_amount;       -- 8/8 full base
                end if;

                -- Pipeline register: break critical path for timing closure
                s_tap1_amount_r <= s_tap1_amount;
                s_tap2_amount_r <= s_tap2_amount;
                s_tap3_amount_r <= s_tap3_amount;
                s_tap4_amount_r <= s_tap4_amount;

                -- Calculate tap read addresses for all channels
                -- Y channel taps (using registered amounts)
                if s_delay_enable = '1' and s_tap1_amount_r > 0 then
                    s_tap1_addr_y <= (s_write_ptr - s_tap1_amount_r + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap1_addr_y <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap2_amount_r > 0 then
                    s_tap2_addr_y <= (s_write_ptr - s_tap2_amount_r + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap2_addr_y <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap3_amount_r > 0 then
                    s_tap3_addr_y <= (s_write_ptr - s_tap3_amount_r + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap3_addr_y <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap4_amount_r > 0 then
                    s_tap4_addr_y <= (s_write_ptr - s_tap4_amount_r + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap4_addr_y <= s_write_ptr;
                end if;

                -- U channel taps (use registered tap amounts scaled from U delay)
                if s_delay_enable = '1' and s_tap1_amount_r > 0 then
                    s_tap1_addr_u <= (s_write_ptr - (s_u_delay_amount / 8) + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap1_addr_u <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap2_amount_r > 0 then
                    s_tap2_addr_u <= (s_write_ptr - (s_u_delay_amount / 4) + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap2_addr_u <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap3_amount_r > 0 then
                    s_tap3_addr_u <= (s_write_ptr - (s_u_delay_amount / 2) + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap3_addr_u <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap4_amount_r > 0 then
                    s_tap4_addr_u <= (s_write_ptr - s_u_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap4_addr_u <= s_write_ptr;
                end if;

                -- V channel taps (use registered tap amounts scaled from V delay)
                if s_delay_enable = '1' and s_tap1_amount_r > 0 then
                    s_tap1_addr_v <= (s_write_ptr - (s_v_delay_amount / 8) + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap1_addr_v <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap2_amount_r > 0 then
                    s_tap2_addr_v <= (s_write_ptr - (s_v_delay_amount / 4) + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap2_addr_v <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap3_amount_r > 0 then
                    s_tap3_addr_v <= (s_write_ptr - (s_v_delay_amount / 2) + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap3_addr_v <= s_write_ptr;
                end if;
                if s_delay_enable = '1' and s_tap4_amount_r > 0 then
                    s_tap4_addr_v <= (s_write_ptr - s_v_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_tap4_addr_v <= s_write_ptr;
                end if;

                -- Store current strobed pixel for blending
                s_current_y <= s_strobe_y;
                s_current_u <= s_strobe_u;
                s_current_v <= s_strobe_v;
            end if;

            s_delayed_avid <= s_strobe_avid;
        end if;
    end process p_pixel_delay_write;

    -- Synchronous RAM read (separate process for proper block RAM inference)
    -- Read addresses were calculated in previous clock cycle
    p_pixel_delay_read : process(clk)
    begin
        if rising_edge(clk) then
            if s_strobe_avid = '1' then
                -- Synchronous read from block RAM buffers (base delays)
                s_delayed_y <= s_pixel_buf_y(s_read_addr_y);
                s_delayed_u <= s_pixel_buf_u(s_read_addr_u);
                s_delayed_v <= s_pixel_buf_v(s_read_addr_v);

                -- Synchronous read for multi-tap values (parallel reads)
                s_tap1_y <= s_pixel_buf_y(s_tap1_addr_y);
                s_tap1_u <= s_pixel_buf_u(s_tap1_addr_u);
                s_tap1_v <= s_pixel_buf_v(s_tap1_addr_v);

                s_tap2_y <= s_pixel_buf_y(s_tap2_addr_y);
                s_tap2_u <= s_pixel_buf_u(s_tap2_addr_u);
                s_tap2_v <= s_pixel_buf_v(s_tap2_addr_v);

                s_tap3_y <= s_pixel_buf_y(s_tap3_addr_y);
                s_tap3_u <= s_pixel_buf_u(s_tap3_addr_u);
                s_tap3_v <= s_pixel_buf_v(s_tap3_addr_v);

                s_tap4_y <= s_pixel_buf_y(s_tap4_addr_y);
                s_tap4_u <= s_pixel_buf_u(s_tap4_addr_u);
                s_tap4_v <= s_pixel_buf_v(s_tap4_addr_v);
            end if;
        end if;
    end process p_pixel_delay_read;

    --------------------------------------------------------------------------------
    -- Active Echo Count Calculation (Pipeline Stage for Timing)
    --
    -- Purpose: Pre-calculate active echo count based on density parameter
    -- Operation: Register the echo count decision to break critical path
    --------------------------------------------------------------------------------
    p_active_taps : process(clk)
    begin
        if rising_edge(clk) then
            -- Decode echo count from density parameter (0-1023)
            if s_tap_decay < 256 then
                s_active_taps <= 1;  -- 0-25%: sparse (1 echo)
            elsif s_tap_decay < 512 then
                s_active_taps <= 2;  -- 25-50%: moderate (2 echoes)
            elsif s_tap_decay < 768 then
                s_active_taps <= 3;  -- 50-75%: dense (3 echoes)
            else
                s_active_taps <= 4;  -- 75-100%: maximum (4 echoes)
            end if;
        end if;
    end process p_active_taps;

    --------------------------------------------------------------------------------
    -- Multi-Echo Mixing Process
    --
    -- Purpose: Combines multiple delay echoes with density-controlled averaging
    -- Operation:
    --   - Uses registered active echo count (pipelined for timing)
    --   - Progressively adds more echoes as density increases
    --   - Equal averaging (no weighting) for timing closure at 74.25 MHz
    --
    -- Modes:
    --   - Multi-Echo OFF: Pass base delayed pixel through
    --   - Multi-Echo ON: 1-4 echoes based on registered density parameter
    --------------------------------------------------------------------------------
    p_tap_mixing : process(clk)
        variable v_sum_y, v_sum_u, v_sum_v : unsigned(11 downto 0);  -- 10-bit echo + 2 bits for 4 additions
    begin
        if rising_edge(clk) then
            if s_strobe_avid = '1' then
                -- Use registered echo count (calculated in previous cycle)
                if s_multi_enable = '0' then
                    -- Multi-echo OFF: Use only base delayed pixel (single echo)
                    s_mixed_y <= s_tap4_y;
                    s_mixed_u <= s_tap4_u;
                    s_mixed_v <= s_tap4_v;
                else
                    -- Multi-echo ON: Use registered active echo count for mixing
                    -- Echo count was calculated in previous cycle (pipelined for timing)
                    case s_active_taps is
                        when 1 =>
                            -- Sparse: Use only first echo (shortest delay)
                            s_mixed_y <= s_tap1_y;
                            s_mixed_u <= s_tap1_u;
                            s_mixed_v <= s_tap1_v;

                        when 2 =>
                            -- Moderate: Average 2 echoes
                            v_sum_y := resize(s_tap1_y, 12) + resize(s_tap2_y, 12);
                            s_mixed_y <= resize(shift_right(v_sum_y, 1), C_VIDEO_DATA_WIDTH);  -- ÷2
                            v_sum_u := resize(s_tap1_u, 12) + resize(s_tap2_u, 12);
                            s_mixed_u <= resize(shift_right(v_sum_u, 1), C_VIDEO_DATA_WIDTH);
                            v_sum_v := resize(s_tap1_v, 12) + resize(s_tap2_v, 12);
                            s_mixed_v <= resize(shift_right(v_sum_v, 1), C_VIDEO_DATA_WIDTH);

                        when 3 =>
                            -- Dense: Sum 3 echoes, approximate ÷3 with ÷4 (shift) for timing
                            v_sum_y := resize(s_tap1_y, 12) + resize(s_tap2_y, 12) + resize(s_tap3_y, 12);
                            s_mixed_y <= resize(shift_right(v_sum_y, 2), C_VIDEO_DATA_WIDTH);  -- ÷4 (75% amplitude)
                            v_sum_u := resize(s_tap1_u, 12) + resize(s_tap2_u, 12) + resize(s_tap3_u, 12);
                            s_mixed_u <= resize(shift_right(v_sum_u, 2), C_VIDEO_DATA_WIDTH);
                            v_sum_v := resize(s_tap1_v, 12) + resize(s_tap2_v, 12) + resize(s_tap3_v, 12);
                            s_mixed_v <= resize(shift_right(v_sum_v, 2), C_VIDEO_DATA_WIDTH);

                        when 4 =>
                            -- Maximum: Average all 4 echoes (full reverb density)
                            v_sum_y := resize(s_tap1_y, 12) + resize(s_tap2_y, 12) +
                                       resize(s_tap3_y, 12) + resize(s_tap4_y, 12);
                            s_mixed_y <= resize(shift_right(v_sum_y, 2), C_VIDEO_DATA_WIDTH);  -- ÷4
                            v_sum_u := resize(s_tap1_u, 12) + resize(s_tap2_u, 12) +
                                       resize(s_tap3_u, 12) + resize(s_tap4_u, 12);
                            s_mixed_u <= resize(shift_right(v_sum_u, 2), C_VIDEO_DATA_WIDTH);
                            v_sum_v := resize(s_tap1_v, 12) + resize(s_tap2_v, 12) +
                                       resize(s_tap3_v, 12) + resize(s_tap4_v, 12);
                            s_mixed_v <= resize(shift_right(v_sum_v, 2), C_VIDEO_DATA_WIDTH);
                    end case;
                end if;

                s_mixed_avid <= s_strobe_avid;
            end if;
        end if;
    end process p_tap_mixing;

    --------------------------------------------------------------------------------
    -- Decay/Blend Effect (using hardware interpolators)
    --
    -- Purpose: Blends current pixels with delayed pixels for feedback effect
    -- Operation:
    --   - Interpolates between current pixel (a) and delayed pixel (b)
    --   - Effect amount (t) controls blend ratio: lerp(a, b, t) = a + (b-a)*t
    --   - Separate interpolators for Y, U, V channels (color-correct blending)
    --
    -- Control Range:
    --   - Effect amount = 0%: output = current pixel only (no echo/delay visible)
    --   - Effect amount = 50%: output = equal mix of current and delayed
    --   - Effect amount = 100%: output = delayed pixel only (full echo effect)
    --
    -- Visual Results (effect combinations):
    --   - Decay alone: subtle to strong feedback loop
    --   - Decay + Strobe: stuttering ghost images with temporal posterization
    --   - Decay + Y Delay: brightness echo trails
    --   - Decay + U/V Delay: color-separated echoes (chromatic aberration)
    --   - All effects: complex spatio-temporal effects with per-channel control
    --
    -- Creative Examples (Y/U/V delay combinations):
    --   - Y=0, U=0, V=0: no delay (pure decay/feedback effect)
    --   - Y=511, U=511, V=511: synchronized echo (all channels delayed equally)
    --   - Y=511, U=300, V=100: progressive color separation (V leads U leads Y)
    --   - Y=0, U=511, V=511: color echo only (brightness stays current)
    --   - Y=150, U=0, V=450: unique V channel lag (red-green trails)
    --   - Y=300, U=100, V=511: asymmetric color separation (blue leads, red lags)
    --   - Full independent per-channel control enables unique glitch aesthetics
    --------------------------------------------------------------------------------
    interpolator_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_mixed_avid,
            a      => s_current_y,
            b      => s_mixed_y,  -- Use mixed tap output instead of single delayed
            t      => s_effect_amount,
            result => s_blend_y,
            valid  => s_blend_valid
        );

    interpolator_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_mixed_avid,
            a      => s_current_u,
            b      => s_mixed_u,  -- Use mixed tap output instead of single delayed
            t      => s_effect_amount,
            result => s_blend_u,
            valid  => open
        );

    interpolator_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_mixed_avid,
            a      => s_current_v,
            b      => s_mixed_v,  -- Use mixed tap output instead of single delayed
            t      => s_effect_amount,
            result => s_blend_v,
            valid  => open
        );

    --------------------------------------------------------------------------------
    -- Bypass delay line (matches processing latency)
    --------------------------------------------------------------------------------
    p_bypass_delay : process(clk)
        type t_sync_delay is array (0 to C_BASE_LATENCY - 1) of std_logic;
        type t_data_delay is array (0 to C_BASE_LATENCY - 1) of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

        variable v_hsync : t_sync_delay := (others => '1');
        variable v_vsync : t_sync_delay := (others => '1');
        variable v_field : t_sync_delay := (others => '1');
        variable v_y     : t_data_delay := (others => (others => '0'));
        variable v_u     : t_data_delay := (others => (others => '0'));
        variable v_v     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Shift delay lines
            v_hsync := data_in.hsync_n & v_hsync(0 to C_BASE_LATENCY - 2);
            v_vsync := data_in.vsync_n & v_vsync(0 to C_BASE_LATENCY - 2);
            v_field := data_in.field_n & v_field(0 to C_BASE_LATENCY - 2);
            v_y     := data_in.y       & v_y(0 to C_BASE_LATENCY - 2);
            v_u     := data_in.u       & v_u(0 to C_BASE_LATENCY - 2);
            v_v     := data_in.v       & v_v(0 to C_BASE_LATENCY - 2);

            -- Output delayed signals
            s_hsync_delayed <= v_hsync(C_BASE_LATENCY - 1);
            s_vsync_delayed <= v_vsync(C_BASE_LATENCY - 1);
            s_field_delayed <= v_field(C_BASE_LATENCY - 1);
            s_y_bypassed    <= v_y(C_BASE_LATENCY - 1);
            s_u_bypassed    <= v_u(C_BASE_LATENCY - 1);
            s_v_bypassed    <= v_v(C_BASE_LATENCY - 1);
        end if;
    end process p_bypass_delay;

    --------------------------------------------------------------------------------
    -- Output multiplexing
    --------------------------------------------------------------------------------
    s_io_align_in.y <= std_logic_vector(s_blend_y) when s_bypass_enable = '0' else s_y_bypassed;
    s_io_align_in.u <= std_logic_vector(s_blend_u) when s_bypass_enable = '0' else s_u_bypassed;
    s_io_align_in.v <= std_logic_vector(s_blend_v) when s_bypass_enable = '0' else s_v_bypassed;

    s_io_align_in.avid <= s_blend_valid;
    s_io_align_in.hsync_n <= s_hsync_delayed;
    s_io_align_in.vsync_n <= s_vsync_delayed;
    s_io_align_in.field_n <= s_field_delayed;

    -- AUTO_IO_ALIGN_PAD
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_pad_0 <= s_io_align_in;
            s_io_pad_1 <= s_io_pad_0;
            s_io_pad_2 <= s_io_pad_1;
        end if;
    end process p_io_align_pad;

    data_out.y      <= s_io_pad_2.y;
    data_out.u      <= s_io_pad_2.u;
    data_out.v      <= s_io_pad_2.v;
    data_out.avid   <= s_io_pad_2.avid;
    data_out.hsync_n<= s_io_pad_2.hsync_n;
    data_out.vsync_n<= s_io_pad_2.vsync_n;
    data_out.field_n<= s_io_pad_2.field_n;

end architecture digital_delay;
