-- Videomancer Community Programs
-- Copyright (C) 2026 Adam Pflanzer
-- File: chromakey.vhd - Chroma Key Program for Videomancer
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-sdk
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
-- Program Name:        Chroma Key
-- Author:              Adam Pflanzer
-- Overview:
--   Threshold keyer with selectable key channel (Y, U, V, or UV saturation),
--   adjustable threshold and softness, solid color fill, key inversion, matte
--   view, and dry/wet mix. Useful for luma keying, chroma keying, and
--   generating mattes from video content.
--
-- Resources:
--   0 BRAM, ~300-500 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + channel select):  1 clock  -> T+1
--   Stage 1 (distance computation):             1 clock  -> T+2
--   Stage 2 (softness shift + clamp + invert):  1 clock  -> T+3
--   Stage 3 (matte * mix multiply):             1 clock  -> T+4
--   Stage 4 (extract scaled matte):             1 clock  -> T+5
--   interpolator_u x3 (key composite):          4 clocks -> T+9
--   Total: 9 clocks
--
-- Submodules:
--   interpolator_u: linear blend, 4 clocks
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Key Channel (steps_4: Y/U/V/Sat)
--   Pot 2  (registers_in(1)):   Threshold (key target value)
--   Pot 3  (registers_in(2)):   Softness (steps_8: 0=hard, 7=soft)
--   Pot 4  (registers_in(3)):   Fill Y
--   Pot 5  (registers_in(4)):   Fill U
--   Pot 6  (registers_in(5)):   Fill V
--   Tog 7  (registers_in(6)(0)): Key Invert
--   Tog 8  (registers_in(6)(1)): Key View (show matte)
--   Tog 9  (registers_in(6)(2)): Fill Source (Color / Black)
--   Tog 10 (registers_in(6)(3)): Key Mode (Near / Below)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 5 (inline stages)
--   C_SYNC_DELAY_CLKS       = 9 (total, including trailing interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture chromakey of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 5;
    constant C_SYNC_DELAY_CLKS       : integer := 9; -- 5 + 4 (interpolator)

    ---------------------------------------------------------------------------
    -- Control signals (from registers)
    ---------------------------------------------------------------------------
    signal s_key_channel_sel : std_logic_vector(1 downto 0);
    signal s_key_target      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_softness_steps  : unsigned(2 downto 0);
    signal s_fill_y          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_fill_u          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_fill_v          : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_key_invert      : std_logic;
    signal s_key_view        : std_logic;
    signal s_fill_black      : std_logic;
    signal s_key_mode_below  : std_logic;
    signal s_bypass_enable   : std_logic;
    signal s_mix_amount      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 0 outputs: input register + channel select
    ---------------------------------------------------------------------------
    signal s_key_value       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_valid_0         : std_logic;

    ---------------------------------------------------------------------------
    -- Stage 1 outputs: distance computation
    ---------------------------------------------------------------------------
    signal s_distance        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_valid_1         : std_logic;

    ---------------------------------------------------------------------------
    -- Stage 2 outputs: softness shift + clamp + invert
    ---------------------------------------------------------------------------
    signal s_matte_raw       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_valid_2         : std_logic;

    ---------------------------------------------------------------------------
    -- Stage 3 outputs: matte * mix multiply
    ---------------------------------------------------------------------------
    signal s_matte_product   : unsigned(C_VIDEO_DATA_WIDTH * 2 - 1 downto 0);
    signal s_valid_3         : std_logic;

    ---------------------------------------------------------------------------
    -- Stage 4 outputs: extract scaled matte
    ---------------------------------------------------------------------------
    signal s_matte_final     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_valid_4         : std_logic;

    ---------------------------------------------------------------------------
    -- Fill values (after source selection, registered in stage 0)
    ---------------------------------------------------------------------------
    signal s_fill_y_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_fill_u_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_fill_v_r        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Interpolator outputs (key composite)
    ---------------------------------------------------------------------------
    signal s_keyed_y         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_keyed_y_valid   : std_logic;
    signal s_keyed_u         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_keyed_u_valid   : std_logic;
    signal s_keyed_v         : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_keyed_v_valid   : std_logic;

    ---------------------------------------------------------------------------
    -- Bypass / sync delay path
    ---------------------------------------------------------------------------
    signal s_y_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_delayed       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Dry tap: input delayed by C_PROCESSING_DELAY_CLKS for interpolator input
    ---------------------------------------------------------------------------
    signal s_dry_y           : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_dry_u           : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_dry_v           : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

begin
    ---------------------------------------------------------------------------
    -- Register mapping (concurrent)
    ---------------------------------------------------------------------------
    s_key_channel_sel <= registers_in(0)(9 downto 8);
    s_key_target      <= unsigned(registers_in(1));
    s_softness_steps  <= unsigned(registers_in(2)(9 downto 7));
    s_fill_y          <= unsigned(registers_in(3));
    s_fill_u          <= unsigned(registers_in(4));
    s_fill_v          <= unsigned(registers_in(5));
    s_key_invert      <= registers_in(6)(0);
    s_key_view        <= registers_in(6)(1);
    s_fill_black      <= registers_in(6)(2);
    s_key_mode_below  <= registers_in(6)(3);
    s_bypass_enable   <= registers_in(6)(4);
    s_mix_amount      <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- Stage 0: Input register + key channel select
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage0 : process(clk)
        variable v_du  : integer range -512 to 512;
        variable v_dv  : integer range -512 to 512;
        variable v_sat : integer range 0 to 1024;
    begin
        if rising_edge(clk) then
            -- Compute saturation (Manhattan distance from UV neutral)
            v_du := to_integer(unsigned(data_in.u)) - 512;
            v_dv := to_integer(unsigned(data_in.v)) - 512;
            if v_du < 0 then v_du := -v_du; end if;
            if v_dv < 0 then v_dv := -v_dv; end if;
            v_sat := v_du + v_dv;
            if v_sat > 1023 then v_sat := 1023; end if;

            -- Channel select mux
            case s_key_channel_sel is
                when "00"   => s_key_value <= unsigned(data_in.y);
                when "01"   => s_key_value <= unsigned(data_in.u);
                when "10"   => s_key_value <= unsigned(data_in.v);
                when others => s_key_value <= to_unsigned(v_sat, C_VIDEO_DATA_WIDTH);
            end case;

            -- Register fill values with source select
            if s_fill_black = '1' then
                s_fill_y_r <= to_unsigned(0, C_VIDEO_DATA_WIDTH);
                s_fill_u_r <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
                s_fill_v_r <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
            else
                s_fill_y_r <= s_fill_y;
                s_fill_u_r <= s_fill_u;
                s_fill_v_r <= s_fill_v;
            end if;

            s_valid_0 <= data_in.avid;
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: Distance computation
    -- Latency: 1 clock
    -- Near mode: distance = |key_value - target|
    -- Below mode: distance = max(key_value - target, 0)
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_diff : signed(C_VIDEO_DATA_WIDTH downto 0); -- 11-bit signed
    begin
        if rising_edge(clk) then
            v_diff := signed(resize(s_key_value, C_VIDEO_DATA_WIDTH + 1))
                    - signed(resize(s_key_target, C_VIDEO_DATA_WIDTH + 1));

            if s_key_mode_below = '0' then
                -- Near mode: absolute distance from target
                if v_diff < 0 then
                    s_distance <= unsigned(resize(-v_diff, C_VIDEO_DATA_WIDTH));
                else
                    s_distance <= unsigned(resize(v_diff, C_VIDEO_DATA_WIDTH));
                end if;
            else
                -- Below mode: 0 when below target (keyed), positive when above
                if v_diff <= 0 then
                    s_distance <= to_unsigned(0, C_VIDEO_DATA_WIDTH);
                else
                    s_distance <= unsigned(resize(v_diff, C_VIDEO_DATA_WIDTH));
                end if;
            end if;

            s_valid_1 <= s_valid_0;
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 2: Softness shift + clamp + invert -> raw matte
    -- Latency: 1 clock
    -- softness_steps 0 = hardest (shift 7), 7 = softest (shift 0)
    -- Matte: 0 = keyed (fill shows), 1023 = not keyed (original shows)
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_shifted : unsigned(16 downto 0); -- enough for 10-bit << 7
        variable v_clamped : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Apply softness shift (amplify distance to fill 10-bit range)
            case to_integer(s_softness_steps) is
                when 0      => v_shifted := shift_left(resize(s_distance, 17), 7);
                when 1      => v_shifted := shift_left(resize(s_distance, 17), 6);
                when 2      => v_shifted := shift_left(resize(s_distance, 17), 5);
                when 3      => v_shifted := shift_left(resize(s_distance, 17), 4);
                when 4      => v_shifted := shift_left(resize(s_distance, 17), 3);
                when 5      => v_shifted := shift_left(resize(s_distance, 17), 2);
                when 6      => v_shifted := shift_left(resize(s_distance, 17), 1);
                when others => v_shifted := resize(s_distance, 17);
            end case;

            -- Clamp to 10-bit range
            if v_shifted > 1023 then
                v_clamped := to_unsigned(1023, C_VIDEO_DATA_WIDTH);
            else
                v_clamped := v_shifted(C_VIDEO_DATA_WIDTH - 1 downto 0);
            end if;

            -- Apply key invert
            if s_key_invert = '1' then
                s_matte_raw <= to_unsigned(1023, C_VIDEO_DATA_WIDTH) - v_clamped;
            else
                s_matte_raw <= v_clamped;
            end if;

            s_valid_2 <= s_valid_1;
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 3: Matte * Mix multiply
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
    begin
        if rising_edge(clk) then
            s_matte_product <= s_matte_raw * s_mix_amount;
            s_valid_3 <= s_valid_2;
        end if;
    end process p_stage3;

    ---------------------------------------------------------------------------
    -- Stage 4: Extract top 10 bits of 20-bit product -> final matte
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage4 : process(clk)
    begin
        if rising_edge(clk) then
            s_matte_final <= s_matte_product(C_VIDEO_DATA_WIDTH * 2 - 1 downto C_VIDEO_DATA_WIDTH);
            s_valid_4 <= s_valid_3;
        end if;
    end process p_stage4;

    ---------------------------------------------------------------------------
    -- Interpolators: Key composite
    -- Latency: 4 clocks each (parallel)
    -- t=0 -> result=a (fill), t=1023 -> result=b (dry original)
    ---------------------------------------------------------------------------
    u_interp_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_valid_4,
            a      => s_fill_y_r,
            b      => s_dry_y,
            t      => s_matte_final,
            result => s_keyed_y,
            valid  => s_keyed_y_valid
        );

    u_interp_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_valid_4,
            a      => s_fill_u_r,
            b      => s_dry_u,
            t      => s_matte_final,
            result => s_keyed_u,
            valid  => s_keyed_u_valid
        );

    u_interp_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_valid_4,
            a      => s_fill_v_r,
            b      => s_dry_v,
            t      => s_matte_final,
            result => s_keyed_v,
            valid  => s_keyed_v_valid
        );

    ---------------------------------------------------------------------------
    -- Delay lines: sync signals (full pipeline depth) and dry tap + bypass
    ---------------------------------------------------------------------------
    p_delay : process(clk)
        -- Sync delay (C_SYNC_DELAY_CLKS = 9)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        -- Bypass data delay (C_SYNC_DELAY_CLKS = 9)
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        -- Dry tap delay (C_PROCESSING_DELAY_CLKS = 5, aligned with matte)
        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Sync delay shift registers
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);

            -- Bypass data delay (full pipeline depth)
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);

            -- Dry tap delay (aligned with end of inline stages, for interpolator input)
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Output delayed signals
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            s_y_delayed <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_u_delayed <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_v_delayed <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    ---------------------------------------------------------------------------
    -- Output mux: bypass / key view / composite
    ---------------------------------------------------------------------------
    data_out.y <= s_y_delayed when s_bypass_enable = '1' else
                  std_logic_vector(s_matte_final) when s_key_view = '1' else
                  std_logic_vector(s_keyed_y);

    data_out.u <= s_u_delayed when s_bypass_enable = '1' else
                  std_logic_vector(to_unsigned(512, C_VIDEO_DATA_WIDTH)) when s_key_view = '1' else
                  std_logic_vector(s_keyed_u);

    data_out.v <= s_v_delayed when s_bypass_enable = '1' else
                  std_logic_vector(to_unsigned(512, C_VIDEO_DATA_WIDTH)) when s_key_view = '1' else
                  std_logic_vector(s_keyed_v);

end chromakey;
