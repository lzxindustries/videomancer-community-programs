-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   YUV Bit Logic
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Per-channel bitwise logic effect operating directly in YUV space.
--   No colour conversion is performed. Six deterministic operations
--   (AND/OR/XOR/NAND/NOR/NXOR) plus two random modes (LFSR and PRNG) are
--   selectable via a 3-switch encoded operator.  Each channel has an
--   independent 10-bit mask; in random modes the mask gates which bit planes
--   the generated value can reach.
--
--   Because this operates in YUV space, the artistic character differs from
--   the RGB version:
--     Y logic: affects brightness/luminance bit planes
--     U logic: affects blue-yellow chroma axis bit planes
--     V logic: affects red-cyan chroma axis bit planes
--
-- Architecture:
--   Stage 0a    - Control Decode                          (1 clock) -> T+1
--   Stage 0b    - Bit Logic Operation                     (1 clock) -> T+2
--   Stage 1     - Per-Channel Blend (3x interpolator_u)  (4 clocks) -> T+6
--   Stage 2     - Global Blend      (3x interpolator_u)  (4 clocks) -> T+10
--
-- Register Map:
--   Register  0: Y bit mask  (0-1023, all 10 bits)       rotary_potentiometer_1
--   Register  1: U bit mask  (0-1023, all 10 bits)       rotary_potentiometer_2
--   Register  2: V bit mask  (0-1023, all 10 bits)       rotary_potentiometer_3
--   Register  3: Y channel blend (0=dry, 1023=wet)       rotary_potentiometer_4
--   Register  4: U channel blend (0=dry, 1023=wet)       rotary_potentiometer_5
--   Register  5: V channel blend (0=dry, 1023=wet)       rotary_potentiometer_6
--   Register  6: Packed toggle bits:
--     bit 0: Invert/Seed   (ops 0-5: 0=normal, 1=invert masks;
--                           op  6:   0=vsync-reseed LFSR, 1=free-run;
--                           op  7:   no effect)           toggle_switch_7
--     bit 3: Op S2 MSB     (0=Off, 1=On)                 toggle_switch_8
--     bit 2: Op S3         (0=Off, 1=On)                 toggle_switch_9
--     bit 1: Op S4 LSB     (0=Off, 1=On)                 toggle_switch_10
--     bit 4: Bypass enable (0=Process, 1=Bypass)         toggle_switch_11
--   Register  7: Global blend (0=dry, 1023=wet)          linear_potentiometer_12
--
--   Operator encoding (bits 3 downto 1 of register 6):
--     "000"=AND  "001"=OR   "010"=XOR  "011"=NAND
--     "100"=NOR  "101"=NXOR "110"=LFSR "111"=PRNG
--
-- Timing:
--   Total pipeline latency: 10 clock cycles.
--   Bypass delay line and sync delays are all 10 clocks.
--     Stage 0a (control decode):       1 clock  -> T+1
--     Stage 0b (bit logic):            1 clock  -> T+2
--     Stage 1  (per-channel blend):    4 clocks -> T+6
--     Stage 2  (global blend):         4 clocks -> T+10
--
--   Pre-global delay: 6 clocks (data_in at T+0, delayed to T+6 for global dry)
--   Per-channel dry:  2 clocks (data_in.y/u/v delayed to T+2, aligned with Stage 0b output)
--
-- LFSR/PRNG:
--   lfsr16 free-runs continuously (period 2^16-1 = 65535).
--   10-bit lfsr runs continuously; in vsync-seed mode (switch 1 off, op=LFSR)
--   it is reseeded from lfsr16[9:0] on the falling edge of vsync_n.
--   Seed bit 0 is forced high to prevent all-zeros lockup.
--   Random mask = lfsr_out AND channel_mask_knob (XOR applied to channel).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture yuv_bit_logic of program_top is

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    -- Total pipeline latency: 1 + 1 + 4 + 4 = 10 clocks
    constant C_PROCESSING_DELAY_CLKS : integer := 10;

    -- Delay for global blend "dry" YUV input.
    -- Original YUV valid at T+0. Stage 1 output valid at T+6.
    -- Delay original by 6 clocks: T+0+6 = T+6. Aligned.
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 6;

    --------------------------------------------------------------------------------
    -- Functions
    --------------------------------------------------------------------------------

    -- Apply bitwise logic operation between a pixel channel and a mask.
    function apply_logic(pixel : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                         mask  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
                         op    : std_logic_vector(2 downto 0))
        return unsigned is
    begin
        case op is
            when "000"  => return pixel and mask;
            when "001"  => return pixel or  mask;
            when "010"  => return pixel xor mask;
            when "011"  => return not (pixel and mask);
            when "100"  => return not (pixel or  mask);
            when "101"  => return not (pixel xor mask);
            when others => return pixel;
        end case;
    end function;

    --------------------------------------------------------------------------------
    -- Control Signals (combinational from registers_in)
    --------------------------------------------------------------------------------
    signal s_bypass_enable  : std_logic;
    signal s_blend_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_blend   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- LFSR signals
    signal s_lfsr10_out     : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_lfsr16_out     : std_logic_vector(15 downto 0);
    signal s_vsync_n_prev   : std_logic := '1';
    signal s_lfsr_reset     : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode Outputs (T+1)
    -- Registered decoded controls and 1-cycle delayed data.
    --------------------------------------------------------------------------------
    signal s_mask_y_r       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_mask_u_r       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_mask_v_r       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '1');
    signal s_operator_r     : std_logic_vector(2 downto 0) := "000";
    signal s_invert_mask_r  : std_logic := '0';
    signal s_y_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_u_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_v_d1           : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_avid_d1        : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 0b: Bit Logic Outputs (T+2)
    --------------------------------------------------------------------------------
    signal s_processed_y     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_u     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_v     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_processed_valid : std_logic;

    -- 2-cycle delayed originals for per-channel blend dry input (aligned with T+2)
    signal s_orig_y_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_u_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_orig_v_d2      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Blend Outputs (T+6)
    --------------------------------------------------------------------------------
    signal s_blended_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blended_y_valid : std_logic;
    signal s_blended_u_valid : std_logic;
    signal s_blended_v_valid : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 2: Global Blend Outputs (T+10)
    --------------------------------------------------------------------------------
    signal s_global_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_y_valid  : std_logic;
    signal s_global_u_valid  : std_logic;
    signal s_global_v_valid  : std_logic;

    -- Original YUV delayed 6 clocks: T+0+6 = T+6, aligned with s_blended (global dry)
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
    s_bypass_enable <= registers_in(6)(4);
    s_global_blend  <= unsigned(registers_in(7));


    --------------------------------------------------------------------------------
    -- LFSR Modules
    -- lfsr16 free-runs at all times (period 65535).
    -- lfsr10 free-runs and is optionally reseeded from lfsr16 at vsync.
    -- Seed bit 0 is forced high to prevent all-zeros lockup.
    --------------------------------------------------------------------------------
    u_lfsr16 : entity work.lfsr16
        port map (clk    => clk,
                  enable => '1',
                  seed   => s_lfsr16_out,  -- feedback; load never pulses
                  load   => '0',
                  q      => s_lfsr16_out);

    u_lfsr10 : entity work.lfsr
        generic map (G_DATA_WIDTH => C_VIDEO_DATA_WIDTH)
        port map (clk      => clk,
                  reset    => s_lfsr_reset,
                  enable   => '1',
                  seed     => s_lfsr16_out(C_VIDEO_DATA_WIDTH - 1 downto 1) & '1',  -- bit 0 forced high: prevents zero-seed lockup
                  poly     => "1001000000",
                  lfsr_out => s_lfsr10_out);

    --------------------------------------------------------------------------------
    -- Stage 0a: Control Decode
    -- Latency: 1 clock. Input T+0, output T+1.
    -- Pre-registers mask, operator, and invert flag so Stage 0b sees only
    -- registered inputs on its critical path. Also delays data_in by 1 clock.
    --------------------------------------------------------------------------------
    p_control_decode : process(clk)
    begin
        if rising_edge(clk) then
            s_mask_y_r      <= unsigned(registers_in(0));
            s_mask_u_r      <= unsigned(registers_in(1));
            s_mask_v_r      <= unsigned(registers_in(2));
            s_operator_r    <= registers_in(6)(3) & registers_in(6)(2) & registers_in(6)(1); -- {S2=MSB, S3, S4=LSB}
            s_invert_mask_r <= registers_in(6)(0);
            s_vsync_n_prev  <= data_in.vsync_n;
            s_y_d1          <= data_in.y;
            s_u_d1          <= data_in.u;
            s_v_d1          <= data_in.v;
            s_avid_d1       <= data_in.avid;
            -- LFSR reset: registered here to eliminate combinational glitches on
            -- registers_in during vsync. Output is a flip-flop; fires one clock
            -- after the vsync falling edge when op=LFSR and switch is on (sync mode).
            if (data_in.vsync_n = '0' and s_vsync_n_prev = '1')
                    and (registers_in(6)(3) = '1' and registers_in(6)(2) = '1' and registers_in(6)(1) = '0')
                    and (registers_in(6)(0) = '0') then   -- 0=off=vsync-reseed, 1=on=free-run
                s_lfsr_reset <= '1';
            else
                s_lfsr_reset <= '0';
            end if;
        end if;
    end process p_control_decode;

    --------------------------------------------------------------------------------
    -- Stage 0b: Bit Logic Operation
    -- Latency: 1 clock. Input T+1, output T+2.
    -- All control inputs are pre-registered; critical path is only:
    --   registered_data -> bit_ops -> register.
    -- The interpolators downstream use enable=s_processed_valid so they hold
    -- outputs stable during blanking. s_processed_valid mirrors s_avid_d1.
    --------------------------------------------------------------------------------
    p_bit_logic : process(clk)
        variable v_mask_y, v_mask_u, v_mask_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_rand                        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            case s_operator_r is
                when "110" =>  -- LFSR: XOR with gated 10-bit LFSR output
                    v_rand := unsigned(s_lfsr10_out);
                    s_processed_y <= unsigned(s_y_d1) xor (v_rand and s_mask_y_r);
                    s_processed_u <= unsigned(s_u_d1) xor (v_rand and s_mask_u_r);
                    s_processed_v <= unsigned(s_v_d1) xor (v_rand and s_mask_v_r);

                when "111" =>  -- PRNG: XOR with gated lower 10 bits of lfsr16
                    v_rand := unsigned(s_lfsr16_out(C_VIDEO_DATA_WIDTH - 1 downto 0));
                    s_processed_y <= unsigned(s_y_d1) xor (v_rand and s_mask_y_r);
                    s_processed_u <= unsigned(s_u_d1) xor (v_rand and s_mask_u_r);
                    s_processed_v <= unsigned(s_v_d1) xor (v_rand and s_mask_v_r);

                when others =>
                    if s_invert_mask_r = '0' then
                        v_mask_y := s_mask_y_r;
                        v_mask_u := s_mask_u_r;
                        v_mask_v := s_mask_v_r;
                    else
                        v_mask_y := not s_mask_y_r;
                        v_mask_u := not s_mask_u_r;
                        v_mask_v := not s_mask_v_r;
                    end if;
                    s_processed_y <= apply_logic(unsigned(s_y_d1), v_mask_y, s_operator_r);
                    s_processed_u <= apply_logic(unsigned(s_u_d1), v_mask_u, s_operator_r);
                    s_processed_v <= apply_logic(unsigned(s_v_d1), v_mask_v, s_operator_r);
            end case;

            -- 2-cycle delayed originals for per-channel blend dry input
            s_orig_y_d2 <= unsigned(s_y_d1);
            s_orig_u_d2 <= unsigned(s_u_d1);
            s_orig_v_d2 <= unsigned(s_v_d1);

            s_processed_valid <= s_avid_d1;
        end if;
    end process p_bit_logic;

    --------------------------------------------------------------------------------
    -- Stage 1: Per-Channel Wet/Dry Blend
    -- Latency: 4 clocks. Input T+2, output T+6.
    -- a = original YUV (dry, 2-cycle delayed), b = processed YUV (wet)
    --------------------------------------------------------------------------------
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_orig_y_d2, b=>s_processed_y, t=>s_blend_y,
                 result=>s_blended_y, valid=>s_blended_y_valid);

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_orig_u_d2, b=>s_processed_u, t=>s_blend_u,
                 result=>s_blended_u, valid=>s_blended_u_valid);

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH=>C_VIDEO_DATA_WIDTH, G_FRAC_BITS=>C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN=>0, G_OUTPUT_MAX=>1023)
        port map(clk=>clk, enable=>s_processed_valid,
                 a=>s_orig_v_d2, b=>s_processed_v, t=>s_blend_v,
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

end architecture yuv_bit_logic;
