-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   YUV Bit Rotator
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Per-channel bit rotation effect operating directly in YUV space.
--   No colour conversion is performed. Bit rotation is applied to the
--   Y (luminance), U (Cb chroma), and V (Cr chroma) channels independently,
--   then blended back with the original signal per-channel and globally.
--
--   Because this operates in YUV space, the artistic character differs from
--   the RGB version:
--     Y rotation: scrambles brightness/luminance
--     U rotation: scrambles blue-yellow chroma axis
--     V rotation: scrambles red-cyan chroma axis
--
-- Architecture:
--   Stage 0a - Control Decode (1 clock):
--     - Register decoded shift amounts for Y, U, V channels
--     - Register dark suppress threshold (get_threshold LUT → FF)
--     - Pre-compute and register chroma neutral bounds (512 ± threshold, carry chains)
--     - Register direction flag
--     - Delay data_in by 1 clock for Stage 0b
--     - Critical path: registers_in -> comparisons (raw_to_shift) -> register
--
--   Stage 0b - Rotation + Dark Suppress (1 clock):
--     - Apply ROL or ROR using pre-registered shift amounts (pure bit-reorder mux)
--     - Compute per-channel above-threshold flags (independent carry chains):
--       Y: high-pass  (value > threshold → pass; else → 0)
--       U/V: notch    (|value − 512| > threshold → pass; else → 512 neutral chroma)
--     - Critical path: registered data/threshold → carry chain → LUT mux → register
--
--   Stage 1 - Per-Channel Blend (4 clocks, 3x interpolator_u parallel):
--     - Blends original YUV (dry) with rotated YUV (wet) per channel
--
--   Stage 2 - Global Blend (4 clocks, 3x interpolator_u parallel):
--     - Blends original YUV (dry, delayed) with per-channel blended output
--
--   Bypass / Output avid:
--     - 10-clock delay line matches full processing pipeline
--     - toggle_switch_11 routes delayed YUV input directly to output
--     - data_out.avid is driven from s_global_y_valid (end of interpolator chain)
--
-- Register Map:
--   Compatible with Videomancer ABI 1.x
--   Register  0: Y rotation amount (0-1023, maps to 0-10 bits)  rotary_potentiometer_1
--   Register  1: U rotation amount (0-1023, maps to 0-10 bits)  rotary_potentiometer_2
--   Register  2: V rotation amount (0-1023, maps to 0-10 bits)  rotary_potentiometer_3
--   Register  3: Y channel blend   (0=fully dry, 1023=fully wet) rotary_potentiometer_4
--   Register  4: U channel blend   (0=fully dry, 1023=fully wet) rotary_potentiometer_5
--   Register  5: V channel blend   (0=fully dry, 1023=fully wet) rotary_potentiometer_6
--   Register  6: Packed toggle bits (one bit per switch):
--     bit 0: Direction     (0=ROL, 1=ROR)          toggle_switch_7
--     bit 1: Cutoff S1     (0=Off, 1=On)           toggle_switch_8
--     bit 2: Cutoff S2     (0=Off, 1=On)           toggle_switch_9
--     bit 3: Cutoff S3     (0=Off, 1=On)           toggle_switch_10
--     bit 4: Bypass enable (0=Process, 1=Bypass)   toggle_switch_11
--   Register  7: Global blend  (0=fully dry, 1023=fully wet) linear_potentiometer_12
--
-- Dark Suppress Threshold (S1:S2:S3, S1=Cutoff S1=MSB, highest-impact):
--   Y  channel: high-pass  — values at or below threshold → 0 (black)
--   UV channels: notch     — values within ±threshold of 512 → 512 (neutral chroma)
--   000=all pass  001=±3   010=±15   011=±31
--   100=±63       101=±127 110=±255  111=±511
-- Cutoff S1 (hardware S2) is MSB: biggest suppression jump alone.
-- At 111: only Y > 511 or U/V more than 50% from neutral are rotated.
--
-- Timing:
--   Total pipeline latency: 10 clock cycles
--     Stage 0a (control decode):       1 clock  -> T+1
--     Stage 0b (mask + rotate):        1 clock  -> T+2
--     Stage 1  (per-channel blend):    4 clocks -> T+6
--     Stage 2  (global blend):         4 clocks -> T+10
--
--   Pre-global delay: 6 clocks (T+0 + 6 = T+6, aligned with Stage 1 output)
--   Bypass delay:    10 clocks (matches full pipeline)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture yuv_bit_rotator of program_top is

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    -- Total pipeline latency: 1 + 1 + 4 + 4 = 10 clocks
    constant C_PROCESSING_DELAY_CLKS : integer := 10;
    constant C_CHROMA_NEUTRAL        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := to_unsigned(512, C_VIDEO_DATA_WIDTH);

    -- Delay for global blend "dry" YUV input.
    -- Original YUV valid at T+0. Stage 1 output valid at T+6.
    -- Delay original by 6 clocks: T+0+6 = T+6. Aligned.
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 6;

    --------------------------------------------------------------------------------
    -- Functions
    --------------------------------------------------------------------------------

    -- Convert raw 10-bit register value (0-1023) to shift amount (0-10).
    function raw_to_shift(raw : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0))
        return integer is
        variable v : integer;
    begin
        v := to_integer(raw);
        if    v <  52 then return 0;
        elsif v < 154 then return 1;
        elsif v < 256 then return 2;
        elsif v < 359 then return 3;
        elsif v < 461 then return 4;
        elsif v < 563 then return 5;
        elsif v < 666 then return 6;
        elsif v < 768 then return 7;
        elsif v < 870 then return 8;
        elsif v < 972 then return 9;
        else               return 10;
        end if;
    end function;

    -- Rotate left (ROL) within a 10-bit value.
    function rol10(value : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                   n     : integer)
        return unsigned is
        variable m : integer;
    begin
        m := n mod 10;
        case m is
            when 0  => return value;
            when 1  => return value(8 downto 0) & value(9);
            when 2  => return value(7 downto 0) & value(9 downto 8);
            when 3  => return value(6 downto 0) & value(9 downto 7);
            when 4  => return value(5 downto 0) & value(9 downto 6);
            when 5  => return value(4 downto 0) & value(9 downto 5);
            when 6  => return value(3 downto 0) & value(9 downto 4);
            when 7  => return value(2 downto 0) & value(9 downto 3);
            when 8  => return value(1 downto 0) & value(9 downto 2);
            when 9  => return value(0)           & value(9 downto 1);
            when others => return value;
        end case;
    end function;

    -- Rotate right (ROR) within a 10-bit value.
    function ror10(value : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                   n     : integer)
        return unsigned is
        variable m : integer;
    begin
        m := n mod 10;
        if m = 0 then
            return value;
        else
            return rol10(value, 10 - m);
        end if;
    end function;

    -- Dark suppress threshold: values at or below threshold (Y) or within
    -- ±threshold of neutral 512 (U/V) are gated rather than rotated.
    -- Cutoff S1 (hardware S2) is MSB — highest-impact switch.
    -- 000 = all pass; each step doubles the suppressed range.
    function get_threshold(s1, s2, s3 : std_logic)
        return unsigned is
    begin
        case std_logic_vector'(s1 & s2 & s3) is
            when "000"  => return to_unsigned(0,   C_VIDEO_DATA_WIDTH);  -- all pass
            when "001"  => return to_unsigned(3,   C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–3;   UV: ±3 of neutral
            when "010"  => return to_unsigned(15,  C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–15;  UV: ±15
            when "011"  => return to_unsigned(31,  C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–31;  UV: ±31
            when "100"  => return to_unsigned(63,  C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–63;  UV: ±63
            when "101"  => return to_unsigned(127, C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–127; UV: ±127
            when "110"  => return to_unsigned(255, C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–255; UV: ±255
            when "111"  => return to_unsigned(511, C_VIDEO_DATA_WIDTH);  -- Y: suppress 0–511; UV: ±511
            when others => return to_unsigned(0,   C_VIDEO_DATA_WIDTH);
        end case;
    end function;

    --------------------------------------------------------------------------------
    -- Control Signals
    --------------------------------------------------------------------------------
    signal s_bypass_enable  : std_logic;
    signal s_direction      : std_logic;
    signal s_cutoff_s1      : std_logic;
    signal s_cutoff_s2      : std_logic;
    signal s_cutoff_s3      : std_logic;
    signal s_blend_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode Signals (T+1)
    -- Registered decoded controls and 1-cycle delayed data.
    --------------------------------------------------------------------------------
    signal s_shift_y_r      : integer range 0 to 10 := 0;
    signal s_shift_u_r      : integer range 0 to 10 := 0;
    signal s_shift_v_r      : integer range 0 to 10 := 0;
    signal s_threshold      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_neutral_plus   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := to_unsigned(512, C_VIDEO_DATA_WIDTH);
    signal s_neutral_minus  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := to_unsigned(512, C_VIDEO_DATA_WIDTH);
    signal s_direction_r    : std_logic := '0';
    signal s_y_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_u_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_v_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_avid_d1        : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 0b: Rotation Signals (T+2)
    --------------------------------------------------------------------------------
    signal s_rotated_y      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_u      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_v      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_valid  : std_logic;

    -- 2-cycle delayed originals for per-channel blend dry input (aligned with T+2)
    signal s_orig_y_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_u_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_v_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Blend Signals (T+6)
    --------------------------------------------------------------------------------
    signal s_blended_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_y_valid : std_logic;
    signal s_blended_u_valid : std_logic;
    signal s_blended_v_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Blend Signals (T+10)
    --------------------------------------------------------------------------------
    signal s_global_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_y_valid  : std_logic;
    signal s_global_u_valid  : std_logic;
    signal s_global_v_valid  : std_logic;

    -- Original YUV delayed 6 clocks from T+0 to align with s_blended at T+6
    signal s_y_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_for_global    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Bypass Path Delay Line Outputs (10 clocks)
    --------------------------------------------------------------------------------
    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;
    signal s_y_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

begin

    --------------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------------
    s_blend_y       <= unsigned(registers_in(3));
    s_blend_u       <= unsigned(registers_in(4));
    s_blend_v       <= unsigned(registers_in(5));
    s_direction     <= registers_in(6)(0);   -- toggle_switch_7: 0=ROL, 1=ROR
    s_cutoff_s1     <= registers_in(6)(1);   -- toggle_switch_8: 0=Off, 1=On
    s_cutoff_s2     <= registers_in(6)(2);   -- toggle_switch_9: 0=Off, 1=On
    s_cutoff_s3     <= registers_in(6)(3);   -- toggle_switch_10: 0=Off, 1=On
    s_bypass_enable <= registers_in(6)(4);   -- toggle_switch_11: 0=Process, 1=Bypass
    s_global_blend  <= unsigned(registers_in(7));

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Registers decoded shift amounts and bit depth so that Stage 0b sees them
    -- as registered inputs, removing them from the Stage 0b critical path.
    -- Also delays data_in by 1 clock to align with the registered controls.
    --------------------------------------------------------------------------------
    p_decode_stage : process(clk)
        variable v_thresh : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Decode and register shift amounts for each channel
            s_shift_y_r   <= raw_to_shift(unsigned(registers_in(0)));
            s_shift_u_r   <= raw_to_shift(unsigned(registers_in(1)));
            s_shift_v_r   <= raw_to_shift(unsigned(registers_in(2)));

            -- Compute dark suppress threshold; pre-compute chroma neutral bounds.
            -- s_neutral_plus/minus are registered here (carry chains in Stage 0a)
            -- so Stage 0b comparisons start from registered FFs (one carry chain each).
            v_thresh        := get_threshold(s_cutoff_s1, s_cutoff_s2, s_cutoff_s3);
            s_threshold     <= v_thresh;
            s_neutral_plus  <= C_CHROMA_NEUTRAL + v_thresh;  -- upper chroma bound (512 + threshold)
            s_neutral_minus <= C_CHROMA_NEUTRAL - v_thresh;  -- lower chroma bound (512 - threshold)

            -- Register direction flag
            s_direction_r <= s_direction;

            -- Delay data by 1 clock to align with registered controls
            s_y_d1    <= data_in.y;
            s_u_d1    <= data_in.u;
            s_v_d1    <= data_in.v;
            s_avid_d1 <= data_in.avid;
        end if;
    end process p_decode_stage;

    --------------------------------------------------------------------------------
    -- Stage 0b: Bit Depth Masking and Bit Rotation
    -- Latency: 1 clock. Input T+1, output T+2.
    -- All control inputs (shift amounts, depth, direction) are pre-registered,
    -- so the critical path is only: data -> bit mux -> AND mask -> register.
    --------------------------------------------------------------------------------
    p_rotation_stage : process(clk)
        variable v_rot_y   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_rot_u   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_rot_v   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_above_y : std_logic;
        variable v_above_u : std_logic;
        variable v_above_v : std_logic;
    begin
        if rising_edge(clk) then
            -- Compute rotation result (pure bit-reorder mux, no carry chain)
            if s_direction_r = '0' then
                v_rot_y := rol10(unsigned(s_y_d1), s_shift_y_r);
                v_rot_u := rol10(unsigned(s_u_d1), s_shift_u_r);
                v_rot_v := rol10(unsigned(s_v_d1), s_shift_v_r);
            else
                v_rot_y := ror10(unsigned(s_y_d1), s_shift_y_r);
                v_rot_u := ror10(unsigned(s_u_d1), s_shift_u_r);
                v_rot_v := ror10(unsigned(s_v_d1), s_shift_v_r);
            end if;

            -- Dark suppress flags (registered FF inputs → independent carry chains)
            -- Y: high-pass — pass values above threshold; suppress near-black → 0
            if unsigned(s_y_d1) > s_threshold then
                v_above_y := '1';
            else
                v_above_y := '0';
            end if;
            -- U/V: notch — pass values outside ±threshold of neutral 512
            --   s_neutral_plus/minus pre-registered in Stage 0a; one carry chain each
            if unsigned(s_u_d1) > s_neutral_plus or unsigned(s_u_d1) < s_neutral_minus then
                v_above_u := '1';
            else
                v_above_u := '0';
            end if;
            if unsigned(s_v_d1) > s_neutral_plus or unsigned(s_v_d1) < s_neutral_minus then
                v_above_v := '1';
            else
                v_above_v := '0';
            end if;

            -- Apply dark suppress gate (LUT mux: flag selects rotated vs suppressed value)
            -- Suppressed Y → 0 (black); suppressed U/V → 512 (neutral chroma, no colour cast)
            s_rotated_y <= v_rot_y when v_above_y = '1' else (others => '0');
            s_rotated_u <= v_rot_u when v_above_u = '1' else C_CHROMA_NEUTRAL;
            s_rotated_v <= v_rot_v when v_above_v = '1' else C_CHROMA_NEUTRAL;

            -- 2-cycle delayed originals (T+1 data registered to T+2) for blend dry input
            s_orig_y_d2 <= unsigned(s_y_d1);
            s_orig_u_d2 <= unsigned(s_u_d1);
            s_orig_v_d2 <= unsigned(s_v_d1);

            s_rotated_valid <= '1';
        end if;
    end process p_rotation_stage;

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Wet/Dry Blend
    -- Latency: 4 clocks. Input T+2, output T+6.
    -- a = original YUV (dry, 2-cycle delayed), b = rotated YUV (wet)
    --------------------------------------------------------------------------------
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_orig_y_d2, b=>s_rotated_y, t=>s_blend_y,
                 result=>s_blended_y, valid=>s_blended_y_valid);

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_orig_u_d2, b=>s_rotated_u, t=>s_blend_u,
                 result=>s_blended_u, valid=>s_blended_u_valid);

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_orig_v_d2, b=>s_rotated_v, t=>s_blend_v,
                 result=>s_blended_v, valid=>s_blended_v_valid);

    --------------------------------------------------------------------------------
    -- Delay Line: Original YUV for Global Blend Dry Input
    -- Delays data_in.y/u/v by 6 clocks: T+0+6 = T+6. Aligned with s_blended at T+6.
    --------------------------------------------------------------------------------
    p_global_dry_delay : process(clk)
        type t_data_delay is array (0 to C_PRE_GLOBAL_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_delay : t_data_delay := (others => (others => '0'));
        variable v_u_delay : t_data_delay := (others => (others => '0'));
        variable v_v_delay : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_y_delay := unsigned(data_in.y) & v_y_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_u_delay := unsigned(data_in.u) & v_u_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_v_delay := unsigned(data_in.v) & v_v_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            s_y_for_global <= v_y_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_u_for_global <= v_u_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_v_for_global <= v_v_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+6, output T+10.
    -- a = original YUV (dry, delayed to T+6), b = per-channel blended YUV
    --------------------------------------------------------------------------------
    interp_global_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_y_valid,
                 a=>s_y_for_global, b=>s_blended_y, t=>s_global_blend,
                 result=>s_global_y, valid=>s_global_y_valid);

    interp_global_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_u_valid,
                 a=>s_u_for_global, b=>s_blended_u, t=>s_global_blend,
                 result=>s_global_u, valid=>s_global_u_valid);

    interp_global_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_blended_v_valid,
                 a=>s_v_for_global, b=>s_blended_v, t=>s_global_blend,
                 result=>s_global_v, valid=>s_global_v_valid);

    --------------------------------------------------------------------------------
    -- Bypass Path Delay Line (10 clocks)
    --------------------------------------------------------------------------------
    p_bypass_delay : process(clk)
        type t_sync_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
        variable v_y_delay     : t_data_delay := (others => (others => '0'));
        variable v_u_delay     : t_data_delay := (others => (others => '0'));
        variable v_v_delay     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_y_delay     := data_in.y       & v_y_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_delay     := data_in.u       & v_u_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_delay     := data_in.v       & v_v_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_hsync_n_delayed <= v_hsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_vsync_n_delayed <= v_vsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_field_n_delayed <= v_field_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_y_delayed       <= v_y_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_u_delayed       <= v_u_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_v_delayed       <= v_v_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_bypass_delay;

    --------------------------------------------------------------------------------
    -- Output Multiplexing
    --------------------------------------------------------------------------------
    data_out.y <= std_logic_vector(s_global_y) when s_bypass_enable = '0'
                  else s_y_delayed;
    data_out.u <= std_logic_vector(s_global_u) when s_bypass_enable = '0'
                  else s_u_delayed;
    data_out.v <= std_logic_vector(s_global_v) when s_bypass_enable = '0'
                  else s_v_delayed;

    data_out.avid    <= s_global_y_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end architecture yuv_bit_rotator;
