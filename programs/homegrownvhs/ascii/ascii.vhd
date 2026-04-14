-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: ascii.vhd - ASCII Art Program for Videomancer
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
-- Program Name:        ASCII Art
-- Author:              Adam Pflanzer
-- Overview:
--   Converts input video into ASCII-art style rendering. Divides the frame
--   into NxN character blocks. Each block samples the input luma at its center
--   pixel and maps the brightness to one of 10 density-sorted glyphs from a
--   hardcoded 8x8 font ROM. Output renders the glyph pixels with user-
--   selectable foreground/background colors and optional color preservation
--   mode where the video chroma tints each character cell.
--
--   Zero BRAM -- font data is stored as constant LUT arrays. Block center
--   luma is sampled on-the-fly (single pixel per block).
--
-- Resources:
--   0 BRAM, ~2000 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + counters + block tracking): 1 clock  -> T+1
--   Stage 1 (threshold adjust + register):                  1 clock  -> T+2
--   Stage 1a (contrast multiply from registered luma):      1 clock  -> T+3
--   Stage 1b (glyph index mapping from registered luma):    1 clock  -> T+4
--   Stage 2 (font ROM lookup):                             1 clock  -> T+5
--   Stage 3 (glyph decision register):                     1 clock  -> T+6
--   Stage 3b (color mode + brightness multiply):            1 clock  -> T+7
--   interpolator_u x3 (wet/dry mix):                        4 clocks -> T+11
--   Total: 11 clocks
--
-- Submodules:
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Block Size (steps_4: 4/8/12/16 px)
--   Pot 2  (registers_in(1)):   Contrast (sharpens luma->glyph mapping)
--   Pot 3  (registers_in(2)):   FG Hue (foreground color angle)
--   Pot 4  (registers_in(3)):   BG Hue (background color angle)
--   Pot 5  (registers_in(4)):   Brightness (output Y level)
--   Pot 6  (registers_in(5)):   Threshold (luma cutoff for glyph selection)
--   Tog 7  (registers_in(6)(0)): Color Mode (Mono / Video Tint)
--   Tog 8  (registers_in(6)(1)): Invert (Normal / Inverted glyphs)
--   Tog 9  (registers_in(6)(2)): Grid (Off / On)
--   Tog 10 (registers_in(6)(3)): Font (Normal / Bold)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 7 (inline stages)
--   C_SYNC_DELAY_CLKS       = 11 (7 + 4 interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture ascii of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 7;
    constant C_SYNC_DELAY_CLKS       : integer := 11;  -- 7 + 4 (interpolator)

    -- Number of density-sorted glyphs
    constant C_NUM_GLYPHS : integer := 10;

    ---------------------------------------------------------------------------
    -- Font ROM: 10 glyphs sorted by visual density, 8 rows x 8 columns each
    -- Glyph 0 = space (darkest), Glyph 9 = '@' (brightest/densest)
    -- Each row is 8 bits, MSB = leftmost pixel
    -- Density order: ' ' '.' ':' '-' '=' '+' '*' '#' '@' 'M'
    ---------------------------------------------------------------------------
    type t_font_row is array (0 to 7) of unsigned(7 downto 0);
    type t_font_rom is array (0 to C_NUM_GLYPHS - 1) of t_font_row;

    constant C_FONT : t_font_rom := (
        -- Glyph 0: ' ' (space - empty)
        (x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00"),
        -- Glyph 1: '.' (dot)
        (x"00", x"00", x"00", x"00", x"00", x"00", x"18", x"18"),
        -- Glyph 2: ':' (colon)
        (x"00", x"18", x"18", x"00", x"00", x"18", x"18", x"00"),
        -- Glyph 3: '-' (dash)
        (x"00", x"00", x"00", x"7E", x"00", x"00", x"00", x"00"),
        -- Glyph 4: '=' (equals)
        (x"00", x"00", x"7E", x"00", x"7E", x"00", x"00", x"00"),
        -- Glyph 5: '+' (plus)
        (x"00", x"18", x"18", x"7E", x"18", x"18", x"00", x"00"),
        -- Glyph 6: '*' (asterisk)
        (x"00", x"66", x"3C", x"FF", x"3C", x"66", x"00", x"00"),
        -- Glyph 7: '#' (hash)
        (x"24", x"24", x"7E", x"24", x"7E", x"24", x"24", x"00"),
        -- Glyph 8: '@' (at sign)
        (x"3C", x"42", x"5A", x"56", x"5A", x"40", x"3C", x"00"),
        -- Glyph 9: 'M' (capital M - densest)
        (x"C3", x"E7", x"FF", x"DB", x"C3", x"C3", x"C3", x"00")
    );

    -- Bold variant: thicken each glyph by OR-ing with itself shifted right
    type t_bold_rom is array (0 to C_NUM_GLYPHS - 1) of t_font_row;
    constant C_FONT_BOLD : t_bold_rom := (
        (x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00"),
        (x"00", x"00", x"00", x"00", x"00", x"00", x"3C", x"3C"),
        (x"00", x"3C", x"3C", x"00", x"00", x"3C", x"3C", x"00"),
        (x"00", x"00", x"00", x"FF", x"00", x"00", x"00", x"00"),
        (x"00", x"00", x"FF", x"00", x"FF", x"00", x"00", x"00"),
        (x"00", x"3C", x"3C", x"FF", x"3C", x"3C", x"00", x"00"),
        (x"00", x"FF", x"7E", x"FF", x"7E", x"FF", x"00", x"00"),
        (x"36", x"7E", x"FF", x"7E", x"FF", x"7E", x"36", x"00"),
        (x"7E", x"E7", x"FF", x"FF", x"FF", x"E0", x"7E", x"00"),
        (x"E7", x"FF", x"FF", x"FF", x"E7", x"E7", x"E7", x"00")
    );

    ---------------------------------------------------------------------------
    -- Immediate controls (bypass + mix need instant response)
    ---------------------------------------------------------------------------
    signal s_bypass     : std_logic;             -- Toggle 11: bypass
    signal s_mix        : unsigned(9 downto 0);  -- Fader: dry/wet

    ---------------------------------------------------------------------------
    -- Vsync-latched controls (prevents mid-frame tearing / banding)
    ---------------------------------------------------------------------------
    signal r_block_sel  : unsigned(1 downto 0)  := "01";
    signal r_contrast   : unsigned(9 downto 0)  := to_unsigned(512, 10);
    signal r_fg_hue     : unsigned(9 downto 0)  := to_unsigned(512, 10);
    signal r_bg_hue     : unsigned(9 downto 0)  := (others => '0');
    signal r_brightness : unsigned(9 downto 0)  := to_unsigned(512, 10);
    signal r_threshold  : unsigned(9 downto 0)  := (others => '0');
    signal r_color_mode : std_logic             := '0';
    signal r_invert     : std_logic             := '0';
    signal r_grid       : std_logic             := '0';
    signal r_bold       : std_logic             := '0';

    ---------------------------------------------------------------------------
    -- Block size parameters
    ---------------------------------------------------------------------------
    signal s_block_size  : unsigned(4 downto 0) := to_unsigned(8, 5);  -- 4,8,12,16
    signal s_block_mask  : unsigned(3 downto 0) := "0111";             -- block_size-1

    ---------------------------------------------------------------------------
    -- Pixel / line counters
    ---------------------------------------------------------------------------
    signal s_pixel_count  : unsigned(10 downto 0) := (others => '0');
    signal s_line_count   : unsigned(9 downto 0)  := (others => '0');
    signal s_prev_avid    : std_logic := '0';
    signal s_prev_vsync_n : std_logic := '1';

    ---------------------------------------------------------------------------
    -- Block-local coordinates (0 to block_size-1)
    ---------------------------------------------------------------------------
    signal s_local_x : unsigned(3 downto 0) := (others => '0');
    signal s_local_y : unsigned(3 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Sampled luma per block (captured at center pixel)
    ---------------------------------------------------------------------------
    signal s_block_luma  : unsigned(9 downto 0) := (others => '0');
    signal s_block_u     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_block_v     : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- Stage 0 outputs
    ---------------------------------------------------------------------------
    signal s0_y       : unsigned(9 downto 0) := (others => '0');
    signal s0_u       : unsigned(9 downto 0) := (others => '0');
    signal s0_v       : unsigned(9 downto 0) := (others => '0');
    signal s0_local_x : unsigned(2 downto 0) := (others => '0');
    signal s0_local_y : unsigned(2 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 1 outputs (threshold adjust, registered)
    ---------------------------------------------------------------------------
    signal s1_thresh_luma : unsigned(9 downto 0) := (others => '0');
    signal s1_thresh_local_x : unsigned(2 downto 0) := (others => '0');
    signal s1_thresh_local_y : unsigned(2 downto 0) := (others => '0');
    signal s1_thresh_blk_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_thresh_blk_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_thresh_is_edge : std_logic := '0';
    signal s1_thresh_px_y : unsigned(9 downto 0) := (others => '0');
    signal s1_thresh_px_u : unsigned(9 downto 0) := (others => '0');
    signal s1_thresh_px_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 1a intermediate outputs (contrast multiply registered)
    ---------------------------------------------------------------------------
    signal s1a_luma_cont : unsigned(9 downto 0) := (others => '0');
    signal s1a_local_x   : unsigned(2 downto 0) := (others => '0');
    signal s1a_local_y   : unsigned(2 downto 0) := (others => '0');
    signal s1a_blk_u     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1a_blk_v     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1a_is_edge   : std_logic := '0';
    signal s1a_px_y      : unsigned(9 downto 0) := (others => '0');
    signal s1a_px_u      : unsigned(9 downto 0) := (others => '0');
    signal s1a_px_v      : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 1 outputs (glyph index mapped)
    ---------------------------------------------------------------------------
    signal s1_glyph_idx : unsigned(3 downto 0) := (others => '0');
    signal s1_local_x   : unsigned(2 downto 0) := (others => '0');
    signal s1_local_y   : unsigned(2 downto 0) := (others => '0');
    signal s1_blk_u     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_blk_v     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_is_edge   : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 2 outputs
    ---------------------------------------------------------------------------
    signal s2_font_pixel : std_logic := '0';
    signal s2_blk_u      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2_blk_v      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2_is_edge    : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Per-pixel video pipeline (for video-through mode)
    ---------------------------------------------------------------------------
    signal s1_px_y : unsigned(9 downto 0) := (others => '0');
    signal s1_px_u : unsigned(9 downto 0) := (others => '0');
    signal s1_px_v : unsigned(9 downto 0) := (others => '0');
    signal s2_px_y : unsigned(9 downto 0) := (others => '0');
    signal s2_px_u : unsigned(9 downto 0) := (others => '0');
    signal s2_px_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 3 outputs (glyph decision register)
    ---------------------------------------------------------------------------
    signal s3_is_glyph : std_logic := '0';
    signal s3_px_y     : unsigned(9 downto 0) := (others => '0');
    signal s3_px_u     : unsigned(9 downto 0) := (others => '0');
    signal s3_px_v     : unsigned(9 downto 0) := (others => '0');
    signal s3_blk_u    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s3_blk_v    : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- Stage 3b outputs (wet)
    ---------------------------------------------------------------------------
    signal s3b_y : unsigned(9 downto 0) := (others => '0');
    signal s3b_u : unsigned(9 downto 0) := (others => '0');
    signal s3b_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Dry tap / bypass / valid
    ---------------------------------------------------------------------------
    signal s_dry_y : unsigned(9 downto 0);
    signal s_dry_u : unsigned(9 downto 0);
    signal s_dry_v : unsigned(9 downto 0);

    signal s_bypass_y : std_logic_vector(9 downto 0);
    signal s_bypass_u : std_logic_vector(9 downto 0);
    signal s_bypass_v : std_logic_vector(9 downto 0);

    signal s_interp_valid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Delayed sync (intermediate signals for all-concurrent data_out)
    ---------------------------------------------------------------------------
    signal s_sync_hsync_n : std_logic := '1';
    signal s_sync_vsync_n : std_logic := '1';
    signal s_sync_field_n : std_logic := '1';
    signal s_sync_avid    : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Interpolator outputs
    ---------------------------------------------------------------------------
    signal s_mix_y       : unsigned(9 downto 0);
    signal s_mix_u       : unsigned(9 downto 0);
    signal s_mix_v       : unsigned(9 downto 0);
    signal s_mix_y_valid : std_logic;

begin
    ---------------------------------------------------------------------------
    -- Register mapping (bypass + mix are immediate, rest latched on vsync)
    ---------------------------------------------------------------------------
    s_bypass <= registers_in(6)(4);
    s_mix    <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- Block size decode (combinational)
    -- sel 0: 4x4,  sel 1: 8x8,  sel 2: 12x12,  sel 3: 16x16
    ---------------------------------------------------------------------------
    process(r_block_sel)
    begin
        case r_block_sel is
            when "00"   => s_block_size <= to_unsigned(4, 5);
                           s_block_mask <= "0011";
            when "01"   => s_block_size <= to_unsigned(8, 5);
                           s_block_mask <= "0111";
            when "10"   => s_block_size <= to_unsigned(12, 5);
                           s_block_mask <= "1011";
            when "11"   => s_block_size <= to_unsigned(16, 5);
                           s_block_mask <= "1111";
            when others => s_block_size <= to_unsigned(8, 5);
                           s_block_mask <= "0111";
        end case;
    end process;

    ---------------------------------------------------------------------------
    -- Stage 0: Register input, pixel/line counters, block-local coords,
    --          sample luma at block center pixel
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage0 : process(clk)
        variable v_at_center_x : std_logic;
        variable v_at_center_y : std_logic;
        variable v_half_block  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s0_y <= unsigned(data_in.y);
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);

            s_prev_avid    <= data_in.avid;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Pixel counter + local X
            if data_in.avid = '1' then
                s_pixel_count <= s_pixel_count + 1;
                if s_local_x >= s_block_size(3 downto 0) - 1 then
                    s_local_x <= (others => '0');
                else
                    s_local_x <= s_local_x + 1;
                end if;
            end if;

            -- End of active line: reset pixel counter, advance line
            if data_in.avid = '0' and s_prev_avid = '1' then
                s_pixel_count <= (others => '0');
                s_local_x     <= (others => '0');
                if s_local_y >= s_block_size(3 downto 0) - 1 then
                    s_local_y <= (others => '0');
                else
                    s_local_y <= s_local_y + 1;
                end if;
                s_line_count <= s_line_count + 1;
            end if;

            -- Start of new frame + latch parameters
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line_count <= (others => '0');
                s_local_y    <= (others => '0');
                -- Latch parameters on vsync (prevents mid-frame tearing)
                r_block_sel  <= unsigned(registers_in(0)(9 downto 8));
                r_contrast   <= unsigned(registers_in(1));
                r_fg_hue     <= unsigned(registers_in(2));
                r_bg_hue     <= unsigned(registers_in(3));
                r_brightness <= unsigned(registers_in(4));
                r_threshold  <= unsigned(registers_in(5));
                r_color_mode <= registers_in(6)(0);
                r_invert     <= registers_in(6)(1);
                r_grid       <= registers_in(6)(2);
                r_bold       <= registers_in(6)(3);
            end if;

            -- Half block size for center detection
            v_half_block := '0' & s_block_size(3 downto 1);

            -- Detect center pixel of block
            v_at_center_x := '1' when s_local_x = v_half_block else '0';
            v_at_center_y := '1' when s_local_y = v_half_block else '0';

            -- Sample luma and chroma at block center
            -- Sample luma and chroma at block center X on every line
            -- (sampling only at center_y caused banding: s_block_luma was stale
            -- on all non-center scanlines within a block row)
            if v_at_center_x = '1' and data_in.avid = '1' then
                s_block_luma <= unsigned(data_in.y);
                s_block_u    <= unsigned(data_in.u);
                s_block_v    <= unsigned(data_in.v);
            end if;

            -- Pass local coords to next stage (clamped to 0-7 for font lookup)
            -- For block sizes > 8, we tile the 8x8 font within the block
            case r_block_sel is
                when "00" =>  -- 4px: use top 3 bits of 0-3 range, pad
                    s0_local_x <= s_local_x(2 downto 0);
                    s0_local_y <= s_local_y(2 downto 0);
                when "01" =>  -- 8px: direct mapping
                    s0_local_x <= s_local_x(2 downto 0);
                    s0_local_y <= s_local_y(2 downto 0);
                when "10" =>  -- 12px: wrap at 8 (truncate)
                    if s_local_x >= 8 then
                        s0_local_x <= s_local_x(2 downto 0);
                    else
                        s0_local_x <= s_local_x(2 downto 0);
                    end if;
                    if s_local_y >= 8 then
                        s0_local_y <= s_local_y(2 downto 0);
                    else
                        s0_local_y <= s_local_y(2 downto 0);
                    end if;
                when others =>  -- 16px: use bits 3:1 (every other pixel)
                    s0_local_x <= s_local_x(3 downto 1);
                    s0_local_y <= s_local_y(3 downto 1);
            end case;
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: Threshold adjust + register
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_luma_adj : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s1_thresh_local_x <= s0_local_x;
            s1_thresh_local_y <= s0_local_y;
            s1_thresh_blk_u   <= s_block_u;
            s1_thresh_blk_v   <= s_block_v;
            s1_thresh_px_y    <= s0_y;
            s1_thresh_px_u    <= s0_u;
            s1_thresh_px_v    <= s0_v;

            -- Apply threshold: shift luma so threshold becomes black point
            if s_block_luma > r_threshold then
                v_luma_adj := s_block_luma - r_threshold;
            else
                v_luma_adj := (others => '0');
            end if;
            s1_thresh_luma <= v_luma_adj;

            -- Grid edge detection: first or last pixel of block
            if s_local_x = 0 or s_local_y = 0 then
                s1_thresh_is_edge <= '1';
            else
                s1_thresh_is_edge <= '0';
            end if;
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 1a: Contrast multiply from registered threshold luma
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1a : process(clk)
        variable v_contrast_prod : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s1a_local_x <= s1_thresh_local_x;
            s1a_local_y <= s1_thresh_local_y;
            s1a_blk_u   <= s1_thresh_blk_u;
            s1a_blk_v   <= s1_thresh_blk_v;
            s1a_px_y    <= s1_thresh_px_y;
            s1a_px_u    <= s1_thresh_px_u;
            s1a_px_v    <= s1_thresh_px_v;
            s1a_is_edge <= s1_thresh_is_edge;

            -- Apply contrast: multiply registered threshold-adjusted luma
            -- contrast=512: 1x, contrast=1023: ~2x, contrast=0: black
            v_contrast_prod := s1_thresh_luma * r_contrast;
            if v_contrast_prod(19 downto 9) > 1023 then
                s1a_luma_cont <= to_unsigned(1023, 10);
            else
                s1a_luma_cont <= v_contrast_prod(18 downto 9);
            end if;
        end if;
    end process p_stage1a;

    ---------------------------------------------------------------------------
    -- Stage 1b: Glyph index mapping from registered contrast product
    -- Uses fast multiply-by-5 instead of 10-way comparison cascade
    -- index = (luma * 5) >> 9, giving 0-9
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1b : process(clk)
        variable v_idx_prod : unsigned(12 downto 0);
        variable v_index    : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- Pipeline through from Stage 1
            s1_local_x <= s1a_local_x;
            s1_local_y <= s1a_local_y;
            s1_blk_u   <= s1a_blk_u;
            s1_blk_v   <= s1a_blk_v;
            s1_px_y    <= s1a_px_y;
            s1_px_u    <= s1a_px_u;
            s1_px_v    <= s1a_px_v;
            s1_is_edge <= s1a_is_edge;

            -- Map 10-bit luma to glyph index 0-9
            -- index = (luma * 5) >> 9 (luma*5 = luma*4 + luma)
            v_idx_prod := ('0' & s1a_luma_cont & "00")
                        + ("000" & s1a_luma_cont);
            v_index := v_idx_prod(12 downto 9);
            if v_index > 9 then
                v_index := to_unsigned(9, 4);
            end if;

            -- Invert: flip glyph mapping
            if r_invert = '1' then
                v_index := to_unsigned(9, 4) - v_index;
            end if;

            s1_glyph_idx <= v_index;
        end if;
    end process p_stage1b;

    ---------------------------------------------------------------------------
    -- Stage 2: Font ROM lookup — read pixel from glyph bitmap
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_row_data : unsigned(7 downto 0);
        variable v_pixel    : std_logic;
        variable v_gidx     : integer range 0 to 9;
        variable v_row      : integer range 0 to 7;
        variable v_col      : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s2_blk_u   <= s1_blk_u;
            s2_blk_v   <= s1_blk_v;
            s2_is_edge <= s1_is_edge;
            s2_px_y    <= s1_px_y;
            s2_px_u    <= s1_px_u;
            s2_px_v    <= s1_px_v;

            v_gidx := to_integer(s1_glyph_idx);
            if v_gidx > 9 then
                v_gidx := 9;
            end if;
            v_row := to_integer(s1_local_y);
            v_col := to_integer(s1_local_x);

            -- Select font (normal or bold)
            if r_bold = '1' then
                v_row_data := C_FONT_BOLD(v_gidx)(v_row);
            else
                v_row_data := C_FONT(v_gidx)(v_row);
            end if;

            -- Extract pixel (MSB = leftmost column)
            v_pixel := v_row_data(7 - v_col);

            s2_font_pixel <= v_pixel;
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 3: Register glyph decision (grid overlay + font pixel)
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_is_glyph : std_logic;
    begin
        if rising_edge(clk) then
            -- Grid overlay: if grid is on and at edge, draw grid line
            if r_grid = '1' and s2_is_edge = '1' then
                v_is_glyph := '1';  -- grid lines render as foreground
            else
                v_is_glyph := s2_font_pixel;
            end if;

            s3_is_glyph <= v_is_glyph;
            s3_px_y     <= s2_px_y;
            s3_px_u     <= s2_px_u;
            s3_px_v     <= s2_px_v;
            s3_blk_u    <= s2_blk_u;
            s3_blk_v    <= s2_blk_v;
        end if;
    end process p_stage3;

    ---------------------------------------------------------------------------
    -- Stage 3b: Apply FG/BG colors, brightness, color mode
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3b : process(clk)
        variable v_fg_y    : unsigned(9 downto 0);
        variable v_fg_u    : unsigned(9 downto 0);
        variable v_fg_v    : unsigned(9 downto 0);
        variable v_bg_y    : unsigned(9 downto 0);
        variable v_bg_u    : unsigned(9 downto 0);
        variable v_bg_v    : unsigned(9 downto 0);
        variable v_bright  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            if r_color_mode = '1' then
                -- Video-through mode: glyph shapes modulate original video
                -- FG: video at brightness level, BG: video at bg_hue level
                if s3_is_glyph = '1' then
                    v_bright := s3_px_y * r_brightness;
                    s3b_y <= v_bright(19 downto 10);
                    s3b_u <= s3_px_u;
                    s3b_v <= s3_px_v;
                else
                    v_bright := s3_px_y * r_bg_hue;
                    s3b_y <= v_bright(19 downto 10);
                    s3b_u <= s3_px_u;
                    s3b_v <= s3_px_v;
                end if;
            else
                -- Classic mono mode: flat FG/BG colors
                v_fg_y := r_brightness;
                if r_fg_hue < 64 then
                    v_fg_u := to_unsigned(512, 10);
                    v_fg_v := to_unsigned(512, 10);
                else
                    v_fg_u := r_fg_hue;
                    v_fg_v := to_unsigned(1023, 10) - r_fg_hue;
                end if;

                v_bg_y := (others => '0');
                if r_bg_hue < 64 then
                    v_bg_u := to_unsigned(512, 10);
                    v_bg_v := to_unsigned(512, 10);
                else
                    v_bg_y := to_unsigned(128, 10);
                    v_bg_u := r_bg_hue;
                    v_bg_v := to_unsigned(1023, 10) - r_bg_hue;
                end if;

                if s3_is_glyph = '1' then
                    s3b_y <= v_fg_y;
                    s3b_u <= v_fg_u;
                    s3b_v <= v_fg_v;
                else
                    s3b_y <= v_bg_y;
                    s3b_u <= v_bg_u;
                    s3b_v <= v_bg_v;
                end if;
            end if;
        end if;
    end process p_stage3b;

    ---------------------------------------------------------------------------
    -- Valid pipeline for interpolator enable
    ---------------------------------------------------------------------------
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_interp_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    ---------------------------------------------------------------------------
    -- Interpolators: wet/dry mix (3 channels)
    ---------------------------------------------------------------------------
    u_interp_y : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s_interp_valid,
            a      => s_dry_y,
            b      => s3b_y,
            t      => s_mix,
            result => s_mix_y,
            valid  => s_mix_y_valid
        );

    u_interp_u : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s_interp_valid,
            a      => s_dry_u,
            b      => s3b_u,
            t      => s_mix,
            result => s_mix_u,
            valid  => open
        );

    u_interp_v : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s_interp_valid,
            a      => s_dry_v,
            b      => s3b_v,
            t      => s_mix,
            result => s_mix_v,
            valid  => open
        );

    ---------------------------------------------------------------------------
    -- Delay lines: sync signals, dry tap, bypass data
    ---------------------------------------------------------------------------
    p_delay : process(clk)
        -- Sync delay (full pipeline = 8)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        -- Bypass data delay (full pipeline = 8)
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        -- Dry tap delay (processing depth = 4)
        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Sync signal shift registers
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);

            -- Bypass data shift registers
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);

            -- Dry tap shift registers
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Outputs (to intermediate signals — data_out driven by concurrent)
            s_sync_hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            s_sync_vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            s_sync_field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            s_sync_avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            s_bypass_y <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_u <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_v <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    ---------------------------------------------------------------------------
    -- Output mux: all data_out fields driven by concurrent assignments
    ---------------------------------------------------------------------------
    data_out.hsync_n <= s_sync_hsync_n;
    data_out.vsync_n <= s_sync_vsync_n;
    data_out.field_n <= s_sync_field_n;
    data_out.avid    <= s_sync_avid;

    data_out.y <= s_bypass_y when s_bypass = '1' else
                  std_logic_vector(s_mix_y);

    data_out.u <= s_bypass_u when s_bypass = '1' else
                  std_logic_vector(s_mix_u);

    data_out.v <= s_bypass_v when s_bypass = '1' else
                  std_logic_vector(s_mix_v);

end architecture ascii;
