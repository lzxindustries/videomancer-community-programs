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
--     - Fold direction into effective ROL shift via to_eff_shift() — eliminates
--       the direction mux from Stage 0b's critical path
--     - Register decoded bit depth mask
--     - Delay data_in by 1 clock for Stage 0b
--     - Critical path: registers_in -> comparisons (raw_to_shift) -> register
--
--   Stage 0b - Input Clamp + Bit Rotation (1 clock):
--     - Clamp near-neutral Y/U/V values before rotation to prevent artifacts
--       (Y < 32 → 0; |U-512| < 32 → 512; |V-512| < 32 → 512)
--     - Apply ROL using pre-registered effective shift amounts (single 10-way mux)
--     - Critical path: registered data -> 10-way ROL mux -> register
--
--   Stage 0c - Bit Depth Masking (1 clock):
--     - AND rotated values with pre-registered depth mask
--     - Critical path: registered_pre -> AND registered_mask -> register
--
--   Stage 1 - Per-Channel Blend (4 clocks, 3x interpolator_u parallel):
--     - Blends original YUV (dry) with rotated YUV (wet) per channel
--
--   Stage 2 - Global Blend (4 clocks, 3x interpolator_u parallel):
--     - Blends original YUV (dry, delayed) with per-channel blended output
--
--   Bypass / Output avid:
--     - 11-clock delay line matches full processing pipeline
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
--     bit 1: Bit depth S1  (0=Off, 1=On)           toggle_switch_8
--     bit 2: Bit depth S2  (0=Off, 1=On)           toggle_switch_9
--     bit 3: Bit depth S3  (0=Off, 1=On)           toggle_switch_10
--     bit 4: Bypass enable (0=Process, 1=Bypass)   toggle_switch_11
--   Register  7: Global blend  (0=fully dry, 1023=fully wet) linear_potentiometer_12
--
-- Bit Depth Mode (S3:S2:S1 -> active bit depth):
--   000=10bit  001=8bit  010=6bit  011=5bit
--   100=4bit   101=3bit  110=2bit  111=1bit
--
-- Timing:
--   Total pipeline latency: 11 clock cycles
--     Stage 0a (control decode):       1 clock  -> T+1
--     Stage 0b (clamp + rotate):       1 clock  -> T+2
--     Stage 0c (depth mask):           1 clock  -> T+3
--     Stage 1  (per-channel blend):    4 clocks -> T+7
--     Stage 2  (global blend):         4 clocks -> T+11
--
--   Pre-global delay: 7 clocks (T+0 + 7 = T+7, aligned with Stage 1 output)
--   Bypass delay:    11 clocks (matches full pipeline)
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

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    -- Total pipeline latency: 1 + 1 + 1 + 4 + 4 = 11 clocks
    constant C_PROCESSING_DELAY_CLKS : integer := 11;

    -- Radius of the U/V neutral bypass zone around 512.
    -- If the original U or V value falls within
    -- [512 - C_UV_CLAMP_RADIUS, 512 + C_UV_CLAMP_RADIUS), the rotation result
    -- is discarded and 512 (neutral chroma) is written directly to the output.
    -- This bypass is necessary because rotating 512 (0b1000000000) by any
    -- non-zero shift produces a non-neutral value (e.g. rol10(512,1) = 1),
    -- so pre-clamping to 512 and then rotating does not help.
    -- Increase C_UV_CLAMP_RADIUS if background areas still show a colour cast;
    -- decrease to preserve intentional subtle colour in dark regions.
    constant C_UV_CLAMP_RADIUS : integer := 32;

    -- Delay for global blend "dry" YUV input.
    -- Original YUV valid at T+0. Stage 1 output valid at T+7.
    -- Delay original by 7 clocks: T+0+7 = T+7. Aligned.
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 7;

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

    -- Apply bit depth mask: zero the lower (10 - depth) bits.
    function apply_mask(value : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                        depth : integer)
        return unsigned is
    begin
        case depth is
            when 10    => return value;
            when 8     => return value and to_unsigned(16#3FC#, C_VIDEO_DATA_WIDTH);
            when 6     => return value and to_unsigned(16#3F0#, C_VIDEO_DATA_WIDTH);
            when 5     => return value and to_unsigned(16#3E0#, C_VIDEO_DATA_WIDTH);
            when 4     => return value and to_unsigned(16#3C0#, C_VIDEO_DATA_WIDTH);
            when 3     => return value and to_unsigned(16#380#, C_VIDEO_DATA_WIDTH);
            when 2     => return value and to_unsigned(16#300#, C_VIDEO_DATA_WIDTH);
            when 1     => return value and to_unsigned(16#200#, C_VIDEO_DATA_WIDTH);
            when others => return value;
        end case;
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

    -- Converts a raw shift amount (0-10) and direction flag into an effective
    -- ROL-only shift (0-9).  Since ror10(x,k) = rol10(x, 10-k), folding
    -- direction into the shift amount in Stage 0a lets Stage 0b use a single
    -- 10-way ROL mux instead of a 20-way ROL/ROR mux, saving ~1 LUT level on
    -- the critical path.
    function to_eff_shift(shift : integer; direction : std_logic)
        return integer is
        variable m : integer;
    begin
        m := shift mod 10;          -- normalise to 0-9 (handles the "return 10" edge case)
        if direction = '0' then     -- ROL: use shift directly
            return m;
        else                        -- ROR k = ROL (10-k); ROR 0 = ROL 0
            return (10 - m) mod 10;
        end if;
    end function;

    -- Decode active bit depth from three switch bits (S3:S2:S1).
    function get_bit_depth(s1, s2, s3 : std_logic) return integer is
    begin
        case std_logic_vector'(s3 & s2 & s1) is
            when "000"  => return 10;
            when "001"  => return 8;
            when "010"  => return 6;
            when "011"  => return 5;
            when "100"  => return 4;
            when "101"  => return 3;
            when "110"  => return 2;
            when "111"  => return 1;
            when others => return 10;
        end case;
    end function;

    -- Returns the 10-bit AND mask corresponding to a bit depth.
    -- Pre-registering this in Stage 0a removes the case statement from Stage 0b's
    -- critical path, leaving only: data -> rol/ror mux -> AND registered_mask -> reg.
    function depth_to_mask(depth : integer) return unsigned is
    begin
        case depth is
            when 8      => return to_unsigned(16#3FC#, C_VIDEO_DATA_WIDTH);
            when 6      => return to_unsigned(16#3F0#, C_VIDEO_DATA_WIDTH);
            when 5      => return to_unsigned(16#3E0#, C_VIDEO_DATA_WIDTH);
            when 4      => return to_unsigned(16#3C0#, C_VIDEO_DATA_WIDTH);
            when 3      => return to_unsigned(16#380#, C_VIDEO_DATA_WIDTH);
            when 2      => return to_unsigned(16#300#, C_VIDEO_DATA_WIDTH);
            when 1      => return to_unsigned(16#200#, C_VIDEO_DATA_WIDTH);
            when others => return to_unsigned(16#3FF#, C_VIDEO_DATA_WIDTH); -- 10-bit: pass through
        end case;
    end function;

    --------------------------------------------------------------------------------
    -- Control Signals
    --------------------------------------------------------------------------------
    signal s_bypass_enable  : std_logic;
    signal s_direction      : std_logic;
    signal s_bit_depth_s1   : std_logic;
    signal s_bit_depth_s2   : std_logic;
    signal s_bit_depth_s3   : std_logic;
    signal s_blend_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode Signals (T+1)
    -- Registered decoded controls and 1-cycle delayed data.
    -- Direction is folded into the shift via to_eff_shift(), so Stage 0b uses a
    -- single 10-way ROL mux instead of a 20-way ROL/ROR mux.
    --------------------------------------------------------------------------------
    signal s_eff_shift_y_r  : integer range 0 to 9 := 0;
    signal s_eff_shift_u_r  : integer range 0 to 9 := 0;
    signal s_eff_shift_v_r  : integer range 0 to 9 := 0;
    signal s_mask_r         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_y_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_u_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_v_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0)
                                          := (others => '0');
    signal s_avid_d1        : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 0b: Clamp + Rotation Signals (T+2)
    -- Rotation only — mask applied in Stage 0c.
    --------------------------------------------------------------------------------
    signal s_rotated_pre_y      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_pre_u      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_pre_v      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_pre_valid  : std_logic;
    -- Unclamped data forwarded T+1→T+2 for the dry path
    signal s_y_d2               : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_d2               : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_d2               : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 0c: Bit Depth Masking Signals (T+3)
    --------------------------------------------------------------------------------
    signal s_rotated_y      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_u      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_v      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_rotated_valid  : std_logic;

    -- 3-cycle delayed originals for per-channel blend dry input (aligned with T+3)
    signal s_orig_y_d3      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_u_d3      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_v_d3      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

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
    s_bit_depth_s1  <= registers_in(6)(1);   -- toggle_switch_8: 0=Off, 1=On
    s_bit_depth_s2  <= registers_in(6)(2);   -- toggle_switch_9: 0=Off, 1=On
    s_bit_depth_s3  <= registers_in(6)(3);   -- toggle_switch_10: 0=Off, 1=On
    s_bypass_enable <= registers_in(6)(4);   -- toggle_switch_11: 0=Process, 1=Bypass
    s_global_blend  <= unsigned(registers_in(7));

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Direction is folded into the shift via to_eff_shift(), eliminating the
    -- direction mux from Stage 0b's critical path.  The depth mask is also
    -- pre-registered so Stage 0c has only a single AND gate on its critical path.
    -- Also delays data_in by 1 clock to align with the registered controls.
    --------------------------------------------------------------------------------
    p_decode_stage : process(clk)
    begin
        if rising_edge(clk) then
            -- Decode shift and fold in direction for a single 10-way ROL mux in Stage 0b
            s_eff_shift_y_r <= to_eff_shift(raw_to_shift(unsigned(registers_in(0))),
                                             registers_in(6)(0));
            s_eff_shift_u_r <= to_eff_shift(raw_to_shift(unsigned(registers_in(1))),
                                             registers_in(6)(0));
            s_eff_shift_v_r <= to_eff_shift(raw_to_shift(unsigned(registers_in(2))),
                                             registers_in(6)(0));

            -- Pre-register bit depth mask
            s_mask_r      <= depth_to_mask(get_bit_depth(s_bit_depth_s1, s_bit_depth_s2, s_bit_depth_s3));

            -- Delay data by 1 clock to align with registered controls
            s_y_d1    <= data_in.y;
            s_u_d1    <= data_in.u;
            s_v_d1    <= data_in.v;
            s_avid_d1 <= data_in.avid;
        end if;
    end process p_decode_stage;

    --------------------------------------------------------------------------------
    -- Stage 0b: Y Input Clamp + Bit Rotation
    -- Latency: 1 clock. Input T+1, output T+2.
    -- Y clamp: if Y < 32, set Y = 0 before rotation. rol10(0, n) = 0 for any n,
    -- so clamping to 0 safely prevents near-black luma LSBs from being rotated
    -- into high bit positions. This does NOT apply to U/V: rol10(512, n) != 512,
    -- so clamping U/V to 512 here would still produce wrong values after rotation.
    -- U/V neutral bypass is handled in Stage 0c after the rotation is complete.
    -- Critical path: data -> 10-way ROL mux -> register.
    --------------------------------------------------------------------------------
    p_rotation_stage : process(clk)
        variable v_y : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_y := unsigned(s_y_d1);

            -- Y input clamp: suppress near-black luma before rotation
            if to_integer(v_y) < 32 then
                v_y := (others => '0');
            end if;

            s_rotated_pre_y <= rol10(v_y, s_eff_shift_y_r);
            -- U/V rotated without pre-clamping; bypass applied in Stage 0c
            s_rotated_pre_u <= rol10(unsigned(s_u_d1), s_eff_shift_u_r);
            s_rotated_pre_v <= rol10(unsigned(s_v_d1), s_eff_shift_v_r);

            -- Original data forwarded unclamped for dry blend path and Stage 0c bypass check
            s_y_d2 <= unsigned(s_y_d1);
            s_u_d2 <= unsigned(s_u_d1);
            s_v_d2 <= unsigned(s_v_d1);

            s_rotated_pre_valid <= '1';
        end if;
    end process p_rotation_stage;

    --------------------------------------------------------------------------------
    -- Stage 0c: Bit Depth Masking + U/V Neutral Bypass
    -- Latency: 1 clock. Input T+2, output T+3.
    -- Y: AND with pre-registered depth mask.
    -- U/V: if the original value (s_u_d2 / s_v_d2) was within C_UV_CLAMP_RADIUS
    -- of neutral (512), output 512 directly — discarding the rotation result.
    -- This is necessary because rotating 512 (0b1000000000) by any non-zero
    -- shift gives a non-neutral value (e.g. rol10(512,1) = 1), so the rotation
    -- must be bypassed rather than pre-clamped before it.
    -- s_u_d2/s_v_d2 are registered T+2 inputs; comparisons vs compile-time
    -- constants are fast (~2 gate levels), keeping the critical path short.
    --------------------------------------------------------------------------------
    p_mask_stage : process(clk)
    begin
        if rising_edge(clk) then
            s_rotated_y <= s_rotated_pre_y and s_mask_r;

            -- U bypass: near-neutral original → output 512; otherwise mask rotated result
            if (to_integer(s_u_d2) >= 512 - C_UV_CLAMP_RADIUS) and
               (to_integer(s_u_d2) <  512 + C_UV_CLAMP_RADIUS) then
                s_rotated_u <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
            else
                s_rotated_u <= s_rotated_pre_u and s_mask_r;
            end if;

            -- V bypass: same logic for V channel
            if (to_integer(s_v_d2) >= 512 - C_UV_CLAMP_RADIUS) and
               (to_integer(s_v_d2) <  512 + C_UV_CLAMP_RADIUS) then
                s_rotated_v <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
            else
                s_rotated_v <= s_rotated_pre_v and s_mask_r;
            end if;

            -- 3-cycle delayed originals (T+2 → T+3) for per-channel blend dry input
            s_orig_y_d3 <= s_y_d2;
            s_orig_u_d3 <= s_u_d2;
            s_orig_v_d3 <= s_v_d2;

            s_rotated_valid <= s_rotated_pre_valid;
        end if;
    end process p_mask_stage;

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Wet/Dry Blend
    -- Latency: 4 clocks. Input T+3, output T+7.
    -- a = original YUV (dry, 3-cycle delayed), b = masked+rotated YUV (wet)
    --------------------------------------------------------------------------------
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_orig_y_d3, b=>s_rotated_y, t=>s_blend_y,
                 result=>s_blended_y, valid=>s_blended_y_valid);

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_orig_u_d3, b=>s_rotated_u, t=>s_blend_u,
                 result=>s_blended_u, valid=>s_blended_u_valid);

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_rotated_valid,
                 a=>s_orig_v_d3, b=>s_rotated_v, t=>s_blend_v,
                 result=>s_blended_v, valid=>s_blended_v_valid);

    --------------------------------------------------------------------------------
    -- Delay Line: Original YUV for Global Blend Dry Input
    -- Delays data_in.y/u/v by 7 clocks: T+0+7 = T+7. Aligned with s_blended at T+7.
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
    -- Latency: 4 clocks. Input T+7, output T+11.
    -- a = original YUV (dry, delayed to T+7), b = per-channel blended YUV
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
    s_io_align_in.y <= std_logic_vector(s_global_y) when s_bypass_enable = '0'
                  else s_y_delayed;
    s_io_align_in.u <= std_logic_vector(s_global_u) when s_bypass_enable = '0'
                  else s_u_delayed;
    s_io_align_in.v <= std_logic_vector(s_global_v) when s_bypass_enable = '0'
                  else s_v_delayed;

    s_io_align_in.avid <= s_global_y_valid;
    s_io_align_in.hsync_n <= s_hsync_n_delayed;
    s_io_align_in.vsync_n <= s_vsync_n_delayed;
    s_io_align_in.field_n <= s_field_n_delayed;

    -- AUTO_IO_ALIGN_PAD
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_pad_0 <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out.y      <= s_io_pad_0.y;
    data_out.u      <= s_io_pad_0.u;
    data_out.v      <= s_io_pad_0.v;
    data_out.avid   <= s_io_pad_0.avid;
    data_out.hsync_n<= s_io_pad_0.hsync_n;
    data_out.vsync_n<= s_io_pad_0.vsync_n;
    data_out.field_n<= s_io_pad_0.field_n;

end architecture yuv_bit_rotator;
