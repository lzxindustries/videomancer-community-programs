-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2026 Adam Pflanzer
-- File: tetris.vhd - Classic Tetris game with analog knob controls
-- License: GNU General Public License v3.0
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

-- Program Name:        Tetris
-- Author:              Adam Pflanzer
-- Overview:            Classic block-stacking game overlaid on input video.
--                      Knob 1 selects column (0-9), Knob 2 selects rotation
--                      (4 orientations).  Pieces fall at gravity speed
--                      controlled by Knob 3 and auto-increasing with level.
--                      Toggle 7 triggers hard drop, Toggle 10 restarts.
-- Resources:
--   0 BRAM, ~2000 LUTs (estimated)
-- Pipeline:
--   video_timing_generator:             ~2 clocks
--   pixel_counter:                       1 clock
--   Stage 1 (coord + cell position):     1 clock  -> T+1
--   Stage 2 (grid/piece lookup):         1 clock  -> T+2
--   Stage 3 (color assignment):          1 clock  -> T+3
--   Stage 4 (score/next overlay):        1 clock  -> T+4
--   Stage 5 (final color mux):           1 clock  -> T+5
--   interpolator_u (wet/dry mix):        4 clocks -> T+9
--   IO alignment:                        2 clocks -> T+11
--   Total data path: 7 render + 4 interp + 2 IO = 13 clocks
-- Submodules:
--   video_timing_generator: edge detection from video sync
--   pixel_counter: active-region coordinate counter
--   lfsr16: pseudo-random piece generation
--   interpolator_u (x3): wet/dry mix for Y, U, V
-- Parameters:
--   Pot 1  (registers_in(0)):  Column (piece horizontal position 0-9)
--   Pot 2  (registers_in(1)):  Rotate (piece orientation, 4 steps)
--   Pot 3  (registers_in(2)):  Speed (gravity drop rate)
--   Pot 4  (registers_in(3)):  Background (grid/border brightness)
--   Pot 5  (registers_in(4)):  Brightness (piece brightness)
--   Pot 6  (registers_in(5)):  Ghost (ghost piece brightness)
--   Tog 7  (registers_in(6)(0)): Hard Drop (edge-triggered)
--   Tog 8  (registers_in(6)(1)): Grid lines on/off
--   Tog 9  (registers_in(6)(2)): Next piece preview on/off
--   Tog 10 (registers_in(6)(3)): Restart (edge-triggered)
--   Tog 11 (registers_in(6)(4)): Pause
--   Fader  (registers_in(7)):   Mix (dry/wet blend)
-- Timing:
--   C_DELAY          = 7 (dry signal pipeline to match wet path)
--   Sync pipe depth   = 11 (indices 0-10)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture tetris of program_top is

    -- ====================================================================
    -- Constants
    -- ====================================================================

    constant C_DATA_WIDTH : integer := 10;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_MAX_VAL    : unsigned(9 downto 0) := to_unsigned(1023, 10);

    -- Pipeline: 11 render + 4 interp + 2 IO = 17 clocks
    constant C_DELAY : integer := 11;

    -- Field geometry
    constant C_FIELD_COLS : integer := 10;
    constant C_FIELD_ROWS : integer := 20;

    -- ----------------------------------------------------------------
    -- Piece bitmap ROM: 7 types x 4 rotations = 28 entries (16 bits)
    -- Bit mapping: bit(15 - row*4 - col) = cell at (row, col) in 4x4 grid
    -- Types: 0=I, 1=O, 2=T, 3=S, 4=Z, 5=L, 6=J
    -- ----------------------------------------------------------------
    type t_piece_rom is array(0 to 27) of std_logic_vector(15 downto 0);
    constant C_PIECES : t_piece_rom := (
        -- Type 0 (I): rotations 0-3
        x"0F00", x"2222", x"00F0", x"4444",
        -- Type 1 (O): rotations 0-3
        x"6600", x"6600", x"6600", x"6600",
        -- Type 2 (T): rotations 0-3
        x"4E00", x"4640", x"0E40", x"4C40",
        -- Type 3 (S): rotations 0-3
        x"6C00", x"8C40", x"06C0", x"4620",
        -- Type 4 (Z): rotations 0-3
        x"C600", x"2640", x"0C60", x"4C80",
        -- Type 5 (L): rotations 0-3
        x"2E00", x"4460", x"0E80", x"6220",
        -- Type 6 (J): rotations 0-3
        x"8E00", x"6440", x"0E20", x"44C0"
    );

    -- ----------------------------------------------------------------
    -- Color LUTs: piece type (1-7) -> Y, U, V
    -- Index 0 = empty (black), 1=I(cyan), 2=O(yellow), 3=T(magenta),
    -- 4=S(green), 5=Z(red), 6=L(orange), 7=J(blue)
    -- ----------------------------------------------------------------
    type t_color_lut is array(0 to 7) of unsigned(9 downto 0);
    constant C_PIECE_Y : t_color_lut := (
        to_unsigned(0, 10),    -- 0: empty
        to_unsigned(800, 10),  -- 1: I
        to_unsigned(900, 10),  -- 2: O
        to_unsigned(550, 10),  -- 3: T
        to_unsigned(700, 10),  -- 4: S
        to_unsigned(500, 10),  -- 5: Z
        to_unsigned(750, 10),  -- 6: L
        to_unsigned(450, 10)   -- 7: J
    );
    constant C_PIECE_U : t_color_lut := (
        to_unsigned(512, 10),  -- 0: empty
        to_unsigned(750, 10),  -- 1: I cyan
        to_unsigned(200, 10),  -- 2: O yellow
        to_unsigned(700, 10),  -- 3: T magenta
        to_unsigned(250, 10),  -- 4: S green
        to_unsigned(380, 10),  -- 5: Z red
        to_unsigned(300, 10),  -- 6: L orange
        to_unsigned(800, 10)   -- 7: J blue
    );
    constant C_PIECE_V : t_color_lut := (
        to_unsigned(512, 10),  -- 0: empty
        to_unsigned(300, 10),  -- 1: I cyan
        to_unsigned(650, 10),  -- 2: O yellow
        to_unsigned(720, 10),  -- 3: T magenta
        to_unsigned(350, 10),  -- 4: S green
        to_unsigned(830, 10),  -- 5: Z red
        to_unsigned(750, 10),  -- 6: L orange
        to_unsigned(350, 10)   -- 7: J blue
    );

    -- ----------------------------------------------------------------
    -- Score font: 5x8 bitmapped digits 0-9 (80 entries of 5-bit vectors)
    -- ----------------------------------------------------------------
    type t_font_rom is array(0 to 79) of std_logic_vector(4 downto 0);
    constant C_FONT : t_font_rom := (
        "01110","10001","10011","10101","11001","10001","01110","00000",
        "00100","01100","00100","00100","00100","00100","01110","00000",
        "01110","10001","00001","00110","01000","10000","11111","00000",
        "01110","10001","00001","00110","00001","10001","01110","00000",
        "00010","00110","01010","10010","11111","00010","00010","00000",
        "11111","10000","11110","00001","00001","10001","01110","00000",
        "01110","10001","10000","11110","10001","10001","01110","00000",
        "11111","00001","00010","00100","01000","01000","01000","00000",
        "01110","10001","10001","01110","10001","10001","01110","00000",
        "01110","10001","10001","01111","00001","10001","01110","00000"
    );
    constant C_DIGIT_W     : integer := 5;
    constant C_DIGIT_H     : integer := 8;
    constant C_DIGIT_SCALE : integer := 2;

    -- ====================================================================
    -- Game FSM States
    -- ====================================================================
    type t_game_state is (
        GS_IDLE,
        GS_INPUT,
        GS_GRAVITY,
        GS_MOVE_SETUP,
        GS_CHECK_READ,
        GS_CHECK_ITER,
        GS_ROT_RESULT,
        GS_MOVE_RESULT,
        GS_DROP_SETUP,
        GS_DROP_RESULT,
        GS_LOCK_ITER,
        GS_LOCK_WRITE,
        GS_LINE_SCAN,
        GS_LINE_READ,
        GS_LINE_WRITE,
        GS_GHOST_SETUP,
        GS_GHOST_RESULT,
        GS_SPAWN,
        GS_SPAWN_CHECK,
        GS_GAMEOVER
    );

    -- ====================================================================
    -- Grid Types
    -- ====================================================================
    type t_occ_grid   is array(0 to C_FIELD_ROWS - 1) of
                         std_logic_vector(C_FIELD_COLS - 1 downto 0);

    -- ====================================================================
    -- Signals
    -- ====================================================================

    -- Timing & measurement
    signal s_timing          : t_video_timing_port;
    signal s_h_count         : unsigned(11 downto 0);
    signal s_v_count         : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_h_active        : integer range 0 to 4095 := 720;
    signal s_v_active        : integer range 0 to 4095 := 480;

    -- Cell size & field geometry
    signal s_cell_shift   : integer range 0 to 5  := 4;  -- vertical cell size shift (power of 2)
    signal s_cell_w_shift : integer range 0 to 5  := 4;  -- horizontal cell size shift (power of 2)
    signal s_cell_w_u     : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_field_w       : integer range 0 to 4095 := 160;
    signal s_field_h       : integer range 0 to 4095 := 320;
    signal s_field_x_off   : integer range 0 to 4095 := 280;
    signal s_field_y_off   : integer range 0 to 4095 := 80;
    signal s_next_x_off    : integer range 0 to 4095 := 0;
    signal s_next_y_off    : integer range 0 to 4095 := 0;
    signal s_score_x       : integer range 0 to 4095 := 0;
    signal s_score_y_pos   : integer range 0 to 4095 := 0;
    signal s_digit_h_scale : integer range 1 to 8     := 2;
    signal s_cell_size_u   : unsigned(5 downto 0) := to_unsigned(16, 6);

    -- LFSR
    signal s_lfsr_q : std_logic_vector(15 downto 0);

    -- Control registers
    signal s_col_reg     : unsigned(9 downto 0);
    signal s_rot_reg     : unsigned(9 downto 0);
    signal s_speed_reg   : unsigned(9 downto 0);
    signal s_bg_reg      : unsigned(9 downto 0);
    signal s_bright_reg  : unsigned(9 downto 0);
    signal s_ghost_reg   : unsigned(9 downto 0);
    signal s_grid_en     : std_logic;
    signal s_next_en     : std_logic;
    signal s_pause_en    : std_logic := '0';
    signal s_mix_reg     : unsigned(9 downto 0);

    -- Toggle edge detection (sticky flags, cleared by FSM)
    signal s_tog7_prev       : std_logic := '0';
    signal s_tog10_prev      : std_logic := '0';
    signal s_hard_drop_flag  : std_logic := '0';
    signal s_restart_flag    : std_logic := '0';
    signal s_clear_hard_drop : std_logic := '0';
    signal s_clear_restart   : std_logic := '0';

    -- Game state
    signal s_state       : t_game_state := GS_IDLE;
    signal s_game_over   : std_logic := '0';
    signal s_needs_spawn : std_logic := '1';
    signal s_flash_ctr   : unsigned(5 downto 0) := (others => '0');

    -- Score (BCD digits, no division needed)
    signal s_score_ones     : integer range 0 to 9 := 0;
    signal s_score_tens     : integer range 0 to 9 := 0;
    signal s_score_hundreds : integer range 0 to 9 := 0;

    -- Grid storage (occupancy only; locked cells rendered as white)
    signal s_grid_occ   : t_occ_grid   := (others => (others => '0'));

    -- Current falling piece
    signal s_cur_type   : integer range 0 to 6 := 0;
    signal s_cur_rot    : unsigned(1 downto 0) := "00";
    signal s_cur_x      : integer range -3 to 12 := 3;
    signal s_cur_y      : integer range -3 to 23 := 0;
    signal s_cur_bitmap : std_logic_vector(15 downto 0) := (others => '0');

    -- Next piece
    signal s_next_type : integer range 0 to 6 := 0;

    -- Ghost piece
    signal s_ghost_y         : integer range -3 to 23 := 0;
    signal s_ghost_y_display : integer range -3 to 23 := 0;
    signal s_ghost_valid     : std_logic := '0';

    -- Physics FSM intermediates
    signal s_req_col    : integer range -3 to 12 := 3;
    signal s_req_rot    : unsigned(1 downto 0) := "00";
    signal s_req_bitmap : std_logic_vector(15 downto 0) := (others => '0');

    -- Collision check (shared sub-FSM)
    signal s_chk_x       : integer range -3 to 12 := 0;
    signal s_chk_y       : integer range -3 to 23 := 0;
    signal s_chk_bitmap  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_chk_idx     : unsigned(3 downto 0) := (others => '0');
    signal s_chk_coll    : std_logic := '0';
    signal s_chk_grid_row : std_logic_vector(C_FIELD_COLS - 1 downto 0) := (others => '0');
    signal s_chk_row_oob  : std_logic := '0';
    signal s_after_check : t_game_state := GS_IDLE;

    -- Gravity
    signal s_gravity_ctr    : unsigned(7 downto 0) := (others => '0');
    signal s_gravity_thresh : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_gravity_tick   : std_logic := '0';
    signal s_hard_drop_active : std_logic := '0';

    -- Lock / line clear
    signal s_lock_idx   : unsigned(3 downto 0) := (others => '0');
    signal s_scan_row   : integer range -1 to 19 := 19;
    signal s_shift_row  : integer range 0 to 19 := 0;

    -- Pre-registered grid write (breaks deep grid write MUX path)
    signal s_lock_wr_en  : std_logic := '0';
    signal s_lock_wr_row : integer range 0 to 19 := 0;
    signal s_lock_wr_col : integer range 0 to 9 := 0;
    signal s_line_src_data  : std_logic_vector(C_FIELD_COLS - 1 downto 0) := (others => '0');

    -- Ghost search
    signal s_ghost_test_y : integer range -3 to 23 := 0;

    -- ----------------------------------------------------------------
    -- Rendering pipeline
    -- ----------------------------------------------------------------

    -- Stage 1
    signal s_stg1_in_field  : std_logic := '0';
    signal s_stg1_cell_col  : integer range 0 to 15 := 0;
    signal s_stg1_cell_row  : integer range 0 to 31 := 0;
    signal s_stg1_cell_px   : unsigned(5 downto 0) := (others => '0');
    signal s_stg1_cell_py   : unsigned(5 downto 0) := (others => '0');
    signal s_stg1_hx        : unsigned(11 downto 0) := (others => '0');
    signal s_stg1_vy        : unsigned(11 downto 0) := (others => '0');

    -- Stage 1.5 (grid row pre-read)
    signal s_stg1p5_in_field  : std_logic := '0';
    signal s_stg1p5_cell_col  : integer range 0 to 15 := 0;
    signal s_stg1p5_cell_row  : integer range 0 to 31 := 0;
    signal s_stg1p5_cell_px   : unsigned(5 downto 0) := (others => '0');
    signal s_stg1p5_cell_py   : unsigned(5 downto 0) := (others => '0');
    signal s_stg1p5_hx        : unsigned(11 downto 0) := (others => '0');
    signal s_stg1p5_vy        : unsigned(11 downto 0) := (others => '0');
    signal s_stg1p5_grid_row  : std_logic_vector(C_FIELD_COLS - 1 downto 0) := (others => '0');

    -- Stage 2
    signal s_stg2_on_grid    : std_logic := '0';
    signal s_stg2_grid_type  : integer range 0 to 7 := 0;
    signal s_stg2_on_piece   : std_logic := '0';
    signal s_stg2_on_ghost   : std_logic := '0';
    signal s_stg2_piece_type : integer range 0 to 7 := 0;
    signal s_stg2_grid_line  : std_logic := '0';
    signal s_stg2_in_field   : std_logic := '0';
    signal s_stg2_in_next    : std_logic := '0';
    signal s_stg2_next_type  : integer range 0 to 7 := 0;
    signal s_stg2_hx         : unsigned(11 downto 0) := (others => '0');
    signal s_stg2_vy         : unsigned(11 downto 0) := (others => '0');

    -- Stage 2.5 (color LUT pre-fetch, breaks multiply from LUT path)
    signal s_stg2p5_y_base    : unsigned(9 downto 0) := (others => '0');
    signal s_stg2p5_u         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_stg2p5_v         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_stg2p5_bright    : unsigned(9 downto 0) := (others => '0');
    signal s_stg2p5_need_mult : std_logic := '0';
    signal s_stg2p5_on_field  : std_logic := '0';
    signal s_stg2p5_hx        : unsigned(11 downto 0) := (others => '0');
    signal s_stg2p5_vy        : unsigned(11 downto 0) := (others => '0');

    -- Stage 3
    signal s_stg3_y         : unsigned(9 downto 0) := (others => '0');
    signal s_stg3_u         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg3_v         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg3_on_field  : std_logic := '0';
    signal s_stg3_hx        : unsigned(11 downto 0) := (others => '0');
    signal s_stg3_vy        : unsigned(11 downto 0) := (others => '0');

    -- Stage 4a (score coordinate decode)
    signal s_stg4a_y         : unsigned(9 downto 0) := (others => '0');
    signal s_stg4a_u         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg4a_v         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg4a_in_y      : std_logic := '0';
    signal s_stg4a_font_r    : integer range 0 to 7 := 0;
    signal s_stg4a_in_d0     : std_logic := '0';
    signal s_stg4a_in_d1     : std_logic := '0';
    signal s_stg4a_in_d2     : std_logic := '0';
    signal s_stg4a_fc0       : integer range 0 to 7 := 0;
    signal s_stg4a_fc1       : integer range 0 to 7 := 0;
    signal s_stg4a_fc2       : integer range 0 to 7 := 0;
    signal s_stg4a_addr0     : integer range 0 to 79 := 0;
    signal s_stg4a_addr1     : integer range 0 to 79 := 0;
    signal s_stg4a_addr2     : integer range 0 to 79 := 0;

    -- Stage 4b (font ROM lookup)
    signal s_stg4b_y         : unsigned(9 downto 0) := (others => '0');
    signal s_stg4b_u         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg4b_v         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg4b_in_d0     : std_logic := '0';
    signal s_stg4b_in_d1     : std_logic := '0';
    signal s_stg4b_in_d2     : std_logic := '0';
    signal s_stg4b_fc0       : integer range 0 to 7 := 0;
    signal s_stg4b_fc1       : integer range 0 to 7 := 0;
    signal s_stg4b_fc2       : integer range 0 to 7 := 0;
    signal s_stg4b_frow0     : std_logic_vector(4 downto 0) := (others => '0');
    signal s_stg4b_frow1     : std_logic_vector(4 downto 0) := (others => '0');
    signal s_stg4b_frow2     : std_logic_vector(4 downto 0) := (others => '0');

    -- Stage 4c (bit select + merge)
    signal s_stg4_y         : unsigned(9 downto 0) := (others => '0');
    signal s_stg4_u         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg4_v         : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Stage 5 (wet output)
    signal s_out_y : unsigned(9 downto 0) := (others => '0');
    signal s_out_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_out_v : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Sync / data delay
    type t_sync_pipe is array(0 to 14) of std_logic_vector(3 downto 0);
    signal s_sync_pipe : t_sync_pipe := (others => (others => '0'));
    type t_data_delay is array(0 to C_DELAY - 1) of std_logic_vector(9 downto 0);
    signal s_y_delay : t_data_delay := (others => (others => '0'));
    signal s_u_delay : t_data_delay := (others => (others => '0'));
    signal s_v_delay : t_data_delay := (others => (others => '0'));

    -- Mix results
    signal s_mix_y_result : unsigned(9 downto 0);
    signal s_mix_u_result : unsigned(9 downto 0);
    signal s_mix_v_result : unsigned(9 downto 0);
    signal s_mix_y_valid  : std_logic;
    signal s_mix_u_valid  : std_logic;
    signal s_mix_v_valid  : std_logic;

    -- IO alignment
    signal s_io_0 : t_video_stream_yuv444_30b;
    signal s_io_1 : t_video_stream_yuv444_30b;

begin

    -- ====================================================================
    -- Timing Generator & Pixel Counter
    -- ====================================================================

    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    pixel_counter_inst : entity work.pixel_counter
        port map (
            clk     => clk,
            timing  => s_timing,
            h_count => s_h_count,
            v_count => s_v_count
        );

    lfsr_inst : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"ACE1",
            load   => '0',
            q      => s_lfsr_q
        );

    -- ====================================================================
    -- Resolution Measurement
    -- ====================================================================

    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then
                    s_measured_h <= s_h_pixel_counter;
                end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then
                    s_measured_v <= s_v_line_counter;
                end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Geometry Derivation
    -- ====================================================================

    p_geometry : process(clk)
        variable v_cell   : integer range 1 to 63;
        variable v_cell_w : integer range 1 to 63;
        variable v_fw     : integer range 0 to 4095;
        variable v_fh     : integer range 0 to 4095;
    begin
        if rising_edge(clk) then
            s_h_active  <= to_integer(s_measured_h);
            s_v_active  <= to_integer(s_measured_v);

            -- Vertical cell size: power of 2 based on v_active
            if s_v_active < 300 then
                v_cell := 8;  s_cell_shift <= 3;
            elsif s_v_active < 600 then
                v_cell := 16; s_cell_shift <= 4;
            else
                v_cell := 32; s_cell_shift <= 5;
            end if;
            s_cell_size_u <= to_unsigned(v_cell, 6);

            -- Horizontal cell width: double for interlaced modes
            -- In interlaced video each field has half the lines, so cells
            -- appear twice as tall on the display.  Widening the cell by 2x
            -- compensates and keeps grid squares visually square.
            if s_timing.is_interlaced = '1' then
                v_cell_w := v_cell * 2;
                s_cell_w_shift <= s_cell_shift + 1;
            else
                v_cell_w := v_cell;
                s_cell_w_shift <= s_cell_shift;
            end if;
            s_cell_w_u <= to_unsigned(v_cell_w, 6);

            v_fw := v_cell_w * C_FIELD_COLS;
            v_fh := v_cell * C_FIELD_ROWS;
            s_field_w <= v_fw;
            s_field_h <= v_fh;
            s_field_x_off <= (s_h_active - v_fw) / 2;
            s_field_y_off <= (s_v_active - v_fh) / 2;

            -- Next piece box: right of field + 2 cell gap
            s_next_x_off  <= (s_h_active + v_fw) / 2 + v_cell_w * 2;
            s_next_y_off  <= (s_v_active - v_fh) / 2 + v_cell;

            -- Score: below next piece box
            s_score_x     <= (s_h_active + v_fw) / 2 + v_cell_w * 2;
            s_score_y_pos <= (s_v_active - v_fh) / 2 + v_cell * 7;

            -- Digit horizontal scale: double when interlaced to match
            -- the 2x vertical stretch
            if s_timing.is_interlaced = '1' then
                s_digit_h_scale <= C_DIGIT_SCALE * 2;
            else
                s_digit_h_scale <= C_DIGIT_SCALE;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Control Register Mapping
    -- ====================================================================

    s_col_reg    <= unsigned(registers_in(0)(9 downto 0));
    s_rot_reg    <= unsigned(registers_in(1)(9 downto 0));
    s_speed_reg  <= unsigned(registers_in(2)(9 downto 0));
    s_bg_reg     <= unsigned(registers_in(3)(9 downto 0));
    s_bright_reg <= unsigned(registers_in(4)(9 downto 0));
    s_ghost_reg  <= unsigned(registers_in(5)(9 downto 0));
    s_grid_en    <= registers_in(6)(1);
    s_next_en    <= registers_in(6)(2);
    s_pause_en   <= registers_in(6)(4);
    s_mix_reg    <= unsigned(registers_in(7)(9 downto 0));

    -- ====================================================================
    -- Toggle Edge Detection
    -- ====================================================================

    p_edges : process(clk)
    begin
        if rising_edge(clk) then
            s_tog7_prev  <= registers_in(6)(0);
            s_tog10_prev <= registers_in(6)(3);
            -- Set sticky flag on any edge; clear when FSM consumes it
            if s_clear_hard_drop = '1' then
                s_hard_drop_flag <= '0';
            elsif (registers_in(6)(0) xor s_tog7_prev) = '1' then
                s_hard_drop_flag <= '1';
            end if;
            if s_clear_restart = '1' then
                s_restart_flag <= '0';
            elsif (registers_in(6)(3) xor s_tog10_prev) = '1' then
                s_restart_flag <= '1';
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Game Physics FSM
    -- ====================================================================

    p_physics : process(clk)
        variable v_piece_row : integer range 0 to 3;
        variable v_piece_col : integer range 0 to 3;
        variable v_bit_idx   : integer range 0 to 15;
        variable v_chk_row   : integer range -3 to 23;
        variable v_chk_col   : integer range -3 to 15;
        variable v_lock_row  : integer range -3 to 23;
        variable v_lock_col  : integer range -3 to 15;
        variable v_rom_idx   : integer range 0 to 27;
        variable v_col_prod  : unsigned(13 downto 0);
        variable v_lfsr_val  : integer range 0 to 7;
        variable v_knob_thresh : unsigned(7 downto 0);
        variable v_level     : integer range 0 to 20;
        variable v_level_thresh : integer range 0 to 63;
    begin
        if rising_edge(clk) then

            -- Default: do not clear sticky flags
            s_clear_hard_drop <= '0';
            s_clear_restart   <= '0';

            case s_state is

            -- ============================================================
            -- IDLE: wait for vsync
            -- ============================================================
            when GS_IDLE =>
                if s_timing.vsync_start = '1' then
                    -- Game-over flash counter
                    if s_game_over = '1' then
                        s_flash_ctr <= s_flash_ctr + 1;
                    end if;
                    -- Pause: skip to idle each frame, freezing game
                    if s_pause_en = '0' then
                        s_state <= GS_INPUT;
                    end if;
                end if;

            -- ============================================================
            -- INPUT: read controls, detect restart/spawn
            -- ============================================================
            when GS_INPUT =>
                -- Column: val * 12 / 1024 - 2  (range -2..9)
                v_col_prod := s_col_reg * to_unsigned(12, 4);
                s_req_col <= to_integer(v_col_prod(13 downto 10)) - 2;

                -- Rotation: top 2 bits
                s_req_rot <= s_rot_reg(9 downto 8);

                -- Speed -> gravity threshold (combine knob + level)
                if s_speed_reg(9 downto 4) > 58 then
                    v_knob_thresh := to_unsigned(2, 8);
                else
                    v_knob_thresh := to_unsigned(60, 8) -
                                     resize(s_speed_reg(9 downto 4), 8);
                end if;

                -- Level = lines / 8 (shift right 3)
                v_level := to_integer(to_unsigned(s_score_hundreds, 4) &
                           to_unsigned(s_score_tens, 4)) / 2;
                if v_level > 12 then
                    v_level := 12;
                end if;
                v_level_thresh := 55 - v_level * 4;
                if v_level_thresh < 2 then
                    v_level_thresh := 2;
                end if;

                -- Use faster of knob vs level
                if to_integer(v_knob_thresh) < v_level_thresh then
                    s_gravity_thresh <= v_knob_thresh;
                else
                    s_gravity_thresh <= to_unsigned(v_level_thresh, 8);
                end if;

                -- Restart
                if s_restart_flag = '1' then
                    s_clear_restart <= '1';
                    for r in 0 to C_FIELD_ROWS - 1 loop
                        s_grid_occ(r)   <= (others => '0');
                    end loop;
                    s_score_ones     <= 0;
                    s_score_tens     <= 0;
                    s_score_hundreds <= 0;
                    s_game_over      <= '0';
                    s_needs_spawn    <= '0';
                    s_gravity_ctr    <= (others => '0');
                    s_state          <= GS_SPAWN;
                elsif s_needs_spawn = '1' then
                    s_needs_spawn <= '0';
                    s_gravity_ctr <= (others => '0');
                    s_state       <= GS_SPAWN;
                elsif s_game_over = '1' then
                    s_state <= GS_IDLE;
                else
                    -- Look up requested bitmap
                    v_rom_idx    := s_cur_type * 4 + to_integer(s_req_rot);
                    s_req_bitmap <= C_PIECES(v_rom_idx);
                    s_state      <= GS_GRAVITY;
                end if;

            -- ============================================================
            -- GRAVITY: tick gravity timer
            -- ============================================================
            when GS_GRAVITY =>
                s_gravity_tick     <= '0';
                s_hard_drop_active <= '0';

                if s_hard_drop_flag = '1' then
                    s_clear_hard_drop  <= '1';
                    s_hard_drop_active <= '1';
                    s_gravity_tick     <= '1';
                elsif s_gravity_ctr >= s_gravity_thresh then
                    s_gravity_ctr  <= (others => '0');
                    s_gravity_tick <= '1';
                else
                    s_gravity_ctr <= s_gravity_ctr + 1;
                end if;

                s_state <= GS_MOVE_SETUP;

            -- ============================================================
            -- MOVE_SETUP: start collision check for requested position
            -- ============================================================
            when GS_MOVE_SETUP =>
                -- Check rotation at current column first
                s_chk_x       <= s_cur_x;
                s_chk_y       <= s_cur_y;
                s_chk_bitmap  <= s_req_bitmap;
                s_chk_idx     <= (others => '0');
                s_chk_coll    <= '0';
                s_after_check <= GS_ROT_RESULT;
                s_state       <= GS_CHECK_READ;

            -- ============================================================
            -- CHECK_READ: pre-read grid row (breaks 20:1 MUX path)
            -- ============================================================
            when GS_CHECK_READ =>
                v_piece_row := to_integer(s_chk_idx(3 downto 2));
                v_chk_row   := s_chk_y + v_piece_row;
                if v_chk_row >= 0 and v_chk_row < C_FIELD_ROWS then
                    s_chk_grid_row <= s_grid_occ(v_chk_row);
                    s_chk_row_oob  <= '0';
                else
                    s_chk_grid_row <= (others => '0');
                    s_chk_row_oob  <= '1';
                end if;
                s_state <= GS_CHECK_ITER;

            -- ============================================================
            -- CHECK_ITER: collision check using pre-read grid row
            -- ============================================================
            when GS_CHECK_ITER =>
                v_piece_row := to_integer(s_chk_idx(3 downto 2));
                v_piece_col := to_integer(s_chk_idx(1 downto 0));
                v_bit_idx   := 15 - v_piece_row * 4 - v_piece_col;

                if s_chk_bitmap(v_bit_idx) = '1' then
                    v_chk_col := s_chk_x + v_piece_col;

                    if s_chk_row_oob = '1' or
                       v_chk_col < 0 or v_chk_col >= C_FIELD_COLS then
                        s_chk_coll <= '1';
                    else
                        if s_chk_grid_row(v_chk_col) = '1' then
                            s_chk_coll <= '1';
                        end if;
                    end if;
                end if;

                if s_chk_idx = 15 then
                    s_state <= s_after_check;
                else
                    s_chk_idx <= s_chk_idx + 1;
                    s_state   <= GS_CHECK_READ;
                end if;

            -- ============================================================
            -- ROT_RESULT: apply rotation if valid, then check column
            -- ============================================================
            when GS_ROT_RESULT =>
                if s_chk_coll = '0' then
                    -- Rotation fits: apply and use new bitmap for column check
                    s_cur_rot    <= s_req_rot;
                    s_cur_bitmap <= s_req_bitmap;
                    s_chk_bitmap <= s_req_bitmap;
                else
                    -- Rotation blocked: check column with current bitmap
                    s_chk_bitmap <= s_cur_bitmap;
                end if;
                s_chk_x       <= s_req_col;
                s_chk_y       <= s_cur_y;
                s_chk_idx     <= (others => '0');
                s_chk_coll    <= '0';
                s_after_check <= GS_MOVE_RESULT;
                s_state       <= GS_CHECK_READ;

            -- ============================================================
            -- MOVE_RESULT: apply or reject column move
            -- ============================================================
            when GS_MOVE_RESULT =>
                if s_chk_coll = '0' then
                    s_cur_x <= s_req_col;
                end if;

                if s_gravity_tick = '1' or s_hard_drop_active = '1' then
                    s_state <= GS_DROP_SETUP;
                else
                    s_state <= GS_GHOST_SETUP;
                end if;

            -- ============================================================
            -- DROP_SETUP: check if piece can descend by 1
            -- ============================================================
            when GS_DROP_SETUP =>
                s_chk_x       <= s_cur_x;
                s_chk_y       <= s_cur_y + 1;
                s_chk_bitmap  <= s_cur_bitmap;
                s_chk_idx     <= (others => '0');
                s_chk_coll    <= '0';
                s_after_check <= GS_DROP_RESULT;
                s_state       <= GS_CHECK_READ;

            -- ============================================================
            -- DROP_RESULT: descend or lock
            -- ============================================================
            when GS_DROP_RESULT =>
                if s_chk_coll = '0' then
                    s_cur_y <= s_cur_y + 1;
                    if s_hard_drop_active = '1' then
                        s_state <= GS_DROP_SETUP;
                    else
                        s_state <= GS_GHOST_SETUP;
                    end if;
                else
                    s_lock_idx <= (others => '0');
                    s_state    <= GS_LOCK_ITER;
                end if;

            -- ============================================================
            -- LOCK_ITER: decode piece cell address (16 iterations)
            -- ============================================================
            when GS_LOCK_ITER =>
                v_piece_row := to_integer(s_lock_idx(3 downto 2));
                v_piece_col := to_integer(s_lock_idx(1 downto 0));
                v_bit_idx   := 15 - v_piece_row * 4 - v_piece_col;

                v_lock_col := s_cur_x + v_piece_col;
                v_lock_row := s_cur_y + v_piece_row;

                if s_cur_bitmap(v_bit_idx) = '1' and
                   v_lock_row >= 0 and v_lock_row < C_FIELD_ROWS and
                   v_lock_col >= 0 and v_lock_col < C_FIELD_COLS then
                    s_lock_wr_en  <= '1';
                    s_lock_wr_row <= v_lock_row;
                    s_lock_wr_col <= v_lock_col;
                else
                    s_lock_wr_en <= '0';
                end if;
                s_state <= GS_LOCK_WRITE;

            -- ============================================================
            -- LOCK_WRITE: apply grid write from registered address
            -- ============================================================
            when GS_LOCK_WRITE =>
                if s_lock_wr_en = '1' then
                    s_grid_occ(s_lock_wr_row)(s_lock_wr_col) <= '1';
                end if;
                s_lock_wr_en <= '0';

                if s_lock_idx = 15 then
                    s_scan_row <= C_FIELD_ROWS - 1;
                    s_state    <= GS_LINE_SCAN;
                else
                    s_lock_idx <= s_lock_idx + 1;
                    s_state    <= GS_LOCK_ITER;
                end if;

            -- ============================================================
            -- LINE_SCAN: check rows for completed lines
            -- ============================================================
            when GS_LINE_SCAN =>
                if s_scan_row < 0 then
                    s_state <= GS_SPAWN;
                else
                    if s_grid_occ(s_scan_row) = "1111111111" then
                        s_shift_row <= s_scan_row;
                        -- Increment BCD score
                        if s_score_ones < 9 then
                            s_score_ones <= s_score_ones + 1;
                        else
                            s_score_ones <= 0;
                            if s_score_tens < 9 then
                                s_score_tens <= s_score_tens + 1;
                            else
                                s_score_tens <= 0;
                                if s_score_hundreds < 9 then
                                    s_score_hundreds <= s_score_hundreds + 1;
                                end if;
                            end if;
                        end if;
                        s_state <= GS_LINE_READ;
                    else
                        s_scan_row <= s_scan_row - 1;
                    end if;
                end if;

            -- ============================================================
            -- LINE_READ: pre-read source row from grid
            -- ============================================================
            when GS_LINE_READ =>
                if s_shift_row > 0 then
                    s_line_src_data <= s_grid_occ(s_shift_row - 1);
                    s_state <= GS_LINE_WRITE;
                else
                    s_grid_occ(0) <= (others => '0');
                    -- Re-check same scan_row (new row shifted in)
                    s_state <= GS_LINE_SCAN;
                end if;

            -- ============================================================
            -- LINE_WRITE: write pre-read data to destination row
            -- ============================================================
            when GS_LINE_WRITE =>
                s_grid_occ(s_shift_row) <= s_line_src_data;
                s_shift_row <= s_shift_row - 1;
                s_state <= GS_LINE_READ;

            -- ============================================================
            -- GHOST_SETUP: find where piece would land
            -- ============================================================
            when GS_GHOST_SETUP =>
                s_ghost_valid  <= '0';
                s_ghost_test_y <= s_cur_y + 1;
                s_chk_x        <= s_cur_x;
                s_chk_y        <= s_cur_y + 1;
                s_chk_bitmap   <= s_cur_bitmap;
                s_chk_idx      <= (others => '0');
                s_chk_coll     <= '0';
                s_after_check  <= GS_GHOST_RESULT;
                s_state        <= GS_CHECK_READ;

            -- ============================================================
            -- GHOST_RESULT: advance ghost downward or finalize
            -- ============================================================
            when GS_GHOST_RESULT =>
                if s_chk_coll = '0' then
                    s_ghost_test_y <= s_ghost_test_y + 1;
                    s_chk_y        <= s_ghost_test_y + 1;
                    s_chk_idx      <= (others => '0');
                    s_chk_coll     <= '0';
                    s_after_check  <= GS_GHOST_RESULT;
                    s_state        <= GS_CHECK_READ;
                else
                    s_ghost_y         <= s_ghost_test_y - 1;
                    s_ghost_y_display <= s_ghost_test_y - 1;
                    s_ghost_valid     <= '1';
                    s_state           <= GS_IDLE;
                end if;

            -- ============================================================
            -- SPAWN: generate new piece
            -- ============================================================
            when GS_SPAWN =>
                -- Current piece = old next piece
                s_cur_type <= s_next_type;
                s_cur_rot  <= "00";
                s_cur_x    <= 3;
                s_cur_y    <= 0;
                v_rom_idx  := s_next_type * 4;
                s_cur_bitmap <= C_PIECES(v_rom_idx);

                -- Suppress ghost until search completes for new piece
                s_ghost_y         <= 0;
                s_ghost_y_display <= 0;
                s_ghost_valid     <= '0';

                -- New next piece from LFSR
                v_lfsr_val := to_integer(unsigned(s_lfsr_q(2 downto 0)));
                if v_lfsr_val > 6 then
                    v_lfsr_val := 0;
                end if;
                s_next_type <= v_lfsr_val;

                -- Check spawn collision
                s_chk_x       <= 3;
                s_chk_y       <= 0;
                s_chk_bitmap  <= C_PIECES(s_next_type * 4);
                s_chk_idx     <= (others => '0');
                s_chk_coll    <= '0';
                s_after_check <= GS_SPAWN_CHECK;
                s_state       <= GS_CHECK_READ;

            -- ============================================================
            -- SPAWN_CHECK: game over if spawn collides
            -- ============================================================
            when GS_SPAWN_CHECK =>
                if s_chk_coll = '1' then
                    s_game_over <= '1';
                    s_flash_ctr <= (others => '0');
                    s_state     <= GS_GAMEOVER;
                else
                    -- Start ghost search for the newly spawned piece
                    s_ghost_test_y <= 1;
                    s_chk_x        <= 3;
                    s_chk_y        <= 1;
                    s_chk_bitmap   <= s_cur_bitmap;
                    s_chk_idx      <= (others => '0');
                    s_chk_coll     <= '0';
                    s_after_check  <= GS_GHOST_RESULT;
                    s_state        <= GS_CHECK_READ;
                end if;

            -- ============================================================
            -- GAMEOVER: return to idle (wait for restart)
            -- ============================================================
            when GS_GAMEOVER =>
                s_state <= GS_IDLE;

            end case;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 1: Coordinate & Cell Position
    -- ====================================================================

    p_stage1 : process(clk)
        variable v_fx : integer range -4095 to 4095;
        variable v_fy : integer range -4095 to 4095;
    begin
        if rising_edge(clk) then
            s_stg1_hx <= s_h_count;
            s_stg1_vy <= s_v_count;

            v_fx := to_integer(s_h_count) - s_field_x_off;
            v_fy := to_integer(s_v_count) - s_field_y_off;

            if v_fx >= 0 and v_fx < s_field_w and
               v_fy >= 0 and v_fy < s_field_h then
                s_stg1_in_field <= '1';
                -- Column: divide by horizontal cell width (power of 2)
                case s_cell_w_shift is
                    when 3 =>
                        s_stg1_cell_col <= v_fx / 8;
                        s_stg1_cell_px  <= to_unsigned(v_fx mod 8, 6);
                    when 4 =>
                        s_stg1_cell_col <= v_fx / 16;
                        s_stg1_cell_px  <= to_unsigned(v_fx mod 16, 6);
                    when others =>
                        s_stg1_cell_col <= v_fx / 32;
                        s_stg1_cell_px  <= to_unsigned(v_fx mod 32, 6);
                end case;
                -- Row: divide by cell size
                case s_cell_shift is
                    when 3 =>
                        s_stg1_cell_row <= v_fy / 8;
                        s_stg1_cell_py  <= to_unsigned(v_fy mod 8, 6);
                    when 4 =>
                        s_stg1_cell_row <= v_fy / 16;
                        s_stg1_cell_py  <= to_unsigned(v_fy mod 16, 6);
                    when others =>
                        s_stg1_cell_row <= v_fy / 32;
                        s_stg1_cell_py  <= to_unsigned(v_fy mod 32, 6);
                end case;
            else
                s_stg1_in_field <= '0';
                s_stg1_cell_col <= 0;
                s_stg1_cell_row <= 0;
                s_stg1_cell_px  <= (others => '0');
                s_stg1_cell_py  <= (others => '0');
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 1.5: Grid Row Pre-Read
    -- ====================================================================

    p_stage1p5 : process(clk)
    begin
        if rising_edge(clk) then
            s_stg1p5_in_field <= s_stg1_in_field;
            s_stg1p5_cell_col <= s_stg1_cell_col;
            s_stg1p5_cell_row <= s_stg1_cell_row;
            s_stg1p5_cell_px  <= s_stg1_cell_px;
            s_stg1p5_cell_py  <= s_stg1_cell_py;
            s_stg1p5_hx       <= s_stg1_hx;
            s_stg1p5_vy       <= s_stg1_vy;
            -- Pre-read grid row (breaks the deep 20:1 MUX path)
            if s_stg1_cell_row < C_FIELD_ROWS then
                s_stg1p5_grid_row <= s_grid_occ(s_stg1_cell_row);
            else
                s_stg1p5_grid_row <= (others => '0');
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 2: Grid / Piece / Ghost Lookup
    -- ====================================================================

    p_stage2 : process(clk)
        variable v_dx      : integer range -15 to 15;
        variable v_dy      : integer range -23 to 23;
        variable v_gdy     : integer range -23 to 23;
        variable v_bit     : integer range 0 to 15;
        variable v_col     : integer range 0 to 9;
        variable v_nfx     : integer range -4095 to 4095;
        variable v_nfy     : integer range -4095 to 4095;
        variable v_nc      : integer range 0 to 3;
        variable v_nr      : integer range 0 to 3;
        variable v_nbit    : integer range 0 to 15;
        variable v_cell_sz : integer range 1 to 63;
    begin
        if rising_edge(clk) then
            s_stg2_in_field   <= s_stg1p5_in_field;
            s_stg2_hx         <= s_stg1p5_hx;
            s_stg2_vy         <= s_stg1p5_vy;
            s_stg2_on_grid    <= '0';
            s_stg2_grid_type  <= 0;
            s_stg2_on_piece   <= '0';
            s_stg2_on_ghost   <= '0';
            s_stg2_piece_type <= 0;
            s_stg2_grid_line  <= '0';
            s_stg2_in_next    <= '0';
            s_stg2_next_type  <= 0;

            -- Main field (uses pre-read grid row from Stage 1.5)
            if s_stg1p5_in_field = '1' then
                -- Clamp column for safe bit access
                if s_stg1p5_cell_col < C_FIELD_COLS then
                    v_col := s_stg1p5_cell_col;
                else
                    v_col := C_FIELD_COLS - 1;
                end if;

                -- Grid cell (occupied = white) — uses pre-read row
                if s_stg1p5_grid_row(v_col) = '1' then
                    s_stg2_on_grid   <= '1';
                end if;

                -- Falling piece
                v_dx := v_col - s_cur_x;
                v_dy := s_stg1p5_cell_row - s_cur_y;
                if v_dx >= 0 and v_dx < 4 and v_dy >= 0 and v_dy < 4 then
                    v_bit := 15 - v_dy * 4 - v_dx;
                    if s_cur_bitmap(v_bit) = '1' then
                        s_stg2_on_piece   <= '1';
                        s_stg2_piece_type <= s_cur_type + 1;
                    end if;
                end if;

                -- Ghost piece
                v_gdy := s_stg1p5_cell_row - s_ghost_y_display;
                if v_dx >= 0 and v_dx < 4 and v_gdy >= 0 and v_gdy < 4 then
                    v_bit := 15 - v_gdy * 4 - v_dx;
                    if s_cur_bitmap(v_bit) = '1' and
                       s_ghost_valid = '1' and
                       s_ghost_y_display /= s_cur_y then
                        s_stg2_on_ghost   <= '1';
                        s_stg2_piece_type <= s_cur_type + 1;
                    end if;
                end if;

                -- Grid line (cell boundaries + field border)
                if s_grid_en = '1' then
                    if s_stg1p5_cell_px = 0 or s_stg1p5_cell_py = 0 then
                        s_stg2_grid_line <= '1';
                    end if;
                    -- Right edge of field
                    if s_stg1p5_cell_col = C_FIELD_COLS - 1 and
                       s_stg1p5_cell_px = s_cell_w_u - 1 then
                        s_stg2_grid_line <= '1';
                    end if;
                    -- Bottom edge of field
                    if s_stg1p5_cell_row = C_FIELD_ROWS - 1 and
                       s_stg1p5_cell_py = s_cell_size_u - 1 then
                        s_stg2_grid_line <= '1';
                    end if;
                end if;
            end if;

            -- Next piece preview (to right of field)
            v_cell_sz := to_integer(s_cell_size_u);
            v_nfx := to_integer(s_stg1p5_hx) - s_next_x_off;
            v_nfy := to_integer(s_stg1p5_vy) - s_next_y_off;
            if v_nfx >= 0 and v_nfx < to_integer(s_cell_w_u) * 4 and
               v_nfy >= 0 and v_nfy < v_cell_sz * 4 and
               s_next_en = '1' then
                s_stg2_in_next <= '1';
                -- Column: divide by horizontal cell width
                case s_cell_w_shift is
                    when 3 =>  v_nc := v_nfx / 8;
                    when 4 =>  v_nc := v_nfx / 16;
                    when others => v_nc := v_nfx / 32;
                end case;
                -- Row: divide by vertical cell size
                case s_cell_shift is
                    when 3 =>  v_nr := v_nfy / 8;
                    when 4 =>  v_nr := v_nfy / 16;
                    when others => v_nr := v_nfy / 32;
                end case;
                v_nbit := 15 - v_nr * 4 - v_nc;
                if C_PIECES(s_next_type * 4)(v_nbit) = '1' then
                    s_stg2_next_type <= s_next_type + 1;
                end if;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 2.5: Color LUT Pre-Fetch
    -- ====================================================================

    p_stage2p5 : process(clk)
        variable v_type : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_stg2p5_hx       <= s_stg2_hx;
            s_stg2p5_vy       <= s_stg2_vy;
            s_stg2p5_on_field <= s_stg2_in_field;
            s_stg2p5_need_mult <= '1';

            if s_stg2_in_field = '1' then
                if s_stg2_on_piece = '1' then
                    v_type := s_stg2_piece_type;
                    s_stg2p5_y_base <= C_PIECE_Y(v_type);
                    s_stg2p5_u      <= C_PIECE_U(v_type);
                    s_stg2p5_v      <= C_PIECE_V(v_type);
                    s_stg2p5_bright <= s_bright_reg;
                elsif s_stg2_on_grid = '1' then
                    s_stg2p5_y_base <= to_unsigned(900, 10);
                    s_stg2p5_u      <= C_CHROMA_MID;
                    s_stg2p5_v      <= C_CHROMA_MID;
                    s_stg2p5_bright <= s_bright_reg;
                elsif s_stg2_on_ghost = '1' then
                    v_type := s_stg2_piece_type;
                    s_stg2p5_y_base <= C_PIECE_Y(v_type);
                    s_stg2p5_u      <= C_PIECE_U(v_type);
                    s_stg2p5_v      <= C_PIECE_V(v_type);
                    s_stg2p5_bright <= s_ghost_reg;
                elsif s_stg2_grid_line = '1' then
                    s_stg2p5_y_base    <= s_bg_reg;
                    s_stg2p5_u         <= C_CHROMA_MID;
                    s_stg2p5_v         <= C_CHROMA_MID;
                    s_stg2p5_need_mult <= '0';
                else
                    s_stg2p5_y_base    <= (others => '0');
                    s_stg2p5_u         <= C_CHROMA_MID;
                    s_stg2p5_v         <= C_CHROMA_MID;
                    s_stg2p5_need_mult <= '0';
                end if;
            elsif s_stg2_in_next = '1' and s_stg2_next_type > 0 then
                v_type := s_stg2_next_type;
                s_stg2p5_y_base <= C_PIECE_Y(v_type);
                s_stg2p5_u      <= C_PIECE_U(v_type);
                s_stg2p5_v      <= C_PIECE_V(v_type);
                s_stg2p5_bright <= s_bright_reg;
                s_stg2p5_on_field <= '1';
            else
                s_stg2p5_y_base    <= resize(shift_right(s_bg_reg, 3), 10);
                s_stg2p5_u         <= C_CHROMA_MID;
                s_stg2p5_v         <= C_CHROMA_MID;
                s_stg2p5_on_field  <= '0';
                s_stg2p5_need_mult <= '0';
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 3: Brightness Multiply
    -- ====================================================================

    p_stage3 : process(clk)
        variable v_prod : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_stg3_hx       <= s_stg2p5_hx;
            s_stg3_vy       <= s_stg2p5_vy;
            s_stg3_on_field <= s_stg2p5_on_field;
            s_stg3_u        <= s_stg2p5_u;
            s_stg3_v        <= s_stg2p5_v;

            if s_stg2p5_need_mult = '1' then
                v_prod   := s_stg2p5_y_base * s_stg2p5_bright;
                s_stg3_y <= v_prod(19 downto 10);
            else
                s_stg3_y <= s_stg2p5_y_base;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 4a: Score Coordinate Decode
    -- ====================================================================

    p_stage4a : process(clk)
        variable v_sx : integer range -4095 to 4095;
        variable v_sy : integer range -4095 to 4095;
        variable v_dx : integer range -4095 to 4095;
    begin
        if rising_edge(clk) then
            s_stg4a_y <= s_stg3_y;
            s_stg4a_u <= s_stg3_u;
            s_stg4a_v <= s_stg3_v;
            s_stg4a_in_y  <= '0';
            s_stg4a_in_d0 <= '0';
            s_stg4a_in_d1 <= '0';
            s_stg4a_in_d2 <= '0';

            v_sx := to_integer(s_stg3_hx) - s_score_x;
            v_sy := to_integer(s_stg3_vy) - s_score_y_pos;

            if v_sy >= 0 and v_sy < C_DIGIT_H * C_DIGIT_SCALE then
                s_stg4a_in_y   <= '1';
                s_stg4a_font_r <= v_sy / C_DIGIT_SCALE;

                -- Digit 0 (hundreds)
                v_dx := v_sx;
                if v_dx >= 0 and v_dx < C_DIGIT_W * s_digit_h_scale then
                    s_stg4a_in_d0 <= '1';
                    s_stg4a_fc0   <= v_dx / s_digit_h_scale;
                end if;

                -- Digit 1 (tens)
                v_dx := v_sx - (C_DIGIT_W + 1) * s_digit_h_scale;
                if v_dx >= 0 and v_dx < C_DIGIT_W * s_digit_h_scale then
                    s_stg4a_in_d1 <= '1';
                    s_stg4a_fc1   <= v_dx / s_digit_h_scale;
                end if;

                -- Digit 2 (ones)
                v_dx := v_sx - 2 * (C_DIGIT_W + 1) * s_digit_h_scale;
                if v_dx >= 0 and v_dx < C_DIGIT_W * s_digit_h_scale then
                    s_stg4a_in_d2 <= '1';
                    s_stg4a_fc2   <= v_dx / s_digit_h_scale;
                end if;

                -- Pre-compute font ROM addresses
                s_stg4a_addr0 <= s_score_hundreds * 8 + v_sy / C_DIGIT_SCALE;
                s_stg4a_addr1 <= s_score_tens     * 8 + v_sy / C_DIGIT_SCALE;
                s_stg4a_addr2 <= s_score_ones     * 8 + v_sy / C_DIGIT_SCALE;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 4b: Font ROM Lookup
    -- ====================================================================

    p_stage4b : process(clk)
    begin
        if rising_edge(clk) then
            s_stg4b_y    <= s_stg4a_y;
            s_stg4b_u    <= s_stg4a_u;
            s_stg4b_v    <= s_stg4a_v;
            s_stg4b_in_d0 <= s_stg4a_in_y and s_stg4a_in_d0;
            s_stg4b_in_d1 <= s_stg4a_in_y and s_stg4a_in_d1;
            s_stg4b_in_d2 <= s_stg4a_in_y and s_stg4a_in_d2;
            s_stg4b_fc0   <= s_stg4a_fc0;
            s_stg4b_fc1   <= s_stg4a_fc1;
            s_stg4b_fc2   <= s_stg4a_fc2;
            -- Pre-read font ROM rows (breaks 80-entry ROM MUX from bit select)
            s_stg4b_frow0 <= C_FONT(s_stg4a_addr0);
            s_stg4b_frow1 <= C_FONT(s_stg4a_addr1);
            s_stg4b_frow2 <= C_FONT(s_stg4a_addr2);
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 4c: Font Bit Select + Score Merge
    -- ====================================================================

    p_stage4c : process(clk)
        variable v_on_score : std_logic;
    begin
        if rising_edge(clk) then
            s_stg4_y <= s_stg4b_y;
            s_stg4_u <= s_stg4b_u;
            s_stg4_v <= s_stg4b_v;

            v_on_score := '0';
            if s_stg4b_in_d0 = '1' and s_stg4b_frow0(4 - s_stg4b_fc0) = '1' then
                v_on_score := '1';
            end if;
            if s_stg4b_in_d1 = '1' and s_stg4b_frow1(4 - s_stg4b_fc1) = '1' then
                v_on_score := '1';
            end if;
            if s_stg4b_in_d2 = '1' and s_stg4b_frow2(4 - s_stg4b_fc2) = '1' then
                v_on_score := '1';
            end if;

            if v_on_score = '1' then
                s_stg4_y <= s_bright_reg;
                s_stg4_u <= C_CHROMA_MID;
                s_stg4_v <= C_CHROMA_MID;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Rendering Pipeline — Stage 5: Final Color Mux + Game Over Flash
    -- ====================================================================

    p_stage5 : process(clk)
    begin
        if rising_edge(clk) then
            if s_game_over = '1' and s_flash_ctr(3) = '1' then
                -- Flash: invert brightness
                s_out_y <= C_MAX_VAL - s_stg4_y;
                s_out_u <= s_stg4_u;
                s_out_v <= s_stg4_v;
            else
                s_out_y <= s_stg4_y;
                s_out_u <= s_stg4_u;
                s_out_v <= s_stg4_v;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Sync & Data Delay Pipelines
    -- ====================================================================

    p_delay_pipes : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_pipe(0) <= data_in.field_n & data_in.avid &
                              data_in.vsync_n & data_in.hsync_n;
            for i in 1 to 14 loop
                s_sync_pipe(i) <= s_sync_pipe(i - 1);
            end loop;

            s_y_delay(0) <= data_in.y;
            s_u_delay(0) <= data_in.u;
            s_v_delay(0) <= data_in.v;
            for i in 1 to C_DELAY - 1 loop
                s_y_delay(i) <= s_y_delay(i - 1);
                s_u_delay(i) <= s_u_delay(i - 1);
                s_v_delay(i) <= s_v_delay(i - 1);
            end loop;
        end if;
    end process;

    -- ====================================================================
    -- Interpolators (wet/dry mix)
    -- ====================================================================

    mix_y_inst : entity work.interpolator_u
        generic map (G_WIDTH => 10, G_FRAC_BITS => 10,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => '1',
                  a => unsigned(s_y_delay(C_DELAY - 1)), b => s_out_y,
                  t => s_mix_reg,
                  result => s_mix_y_result, valid => s_mix_y_valid);

    mix_u_inst : entity work.interpolator_u
        generic map (G_WIDTH => 10, G_FRAC_BITS => 10,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => '1',
                  a => unsigned(s_u_delay(C_DELAY - 1)), b => s_out_u,
                  t => s_mix_reg,
                  result => s_mix_u_result, valid => s_mix_u_valid);

    mix_v_inst : entity work.interpolator_u
        generic map (G_WIDTH => 10, G_FRAC_BITS => 10,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => '1',
                  a => unsigned(s_v_delay(C_DELAY - 1)), b => s_out_v,
                  t => s_mix_reg,
                  result => s_mix_v_result, valid => s_mix_v_valid);

    -- ====================================================================
    -- IO Alignment (2 stages)
    -- ====================================================================

    p_io_align : process(clk)
    begin
        if rising_edge(clk) then
            s_io_0.y       <= std_logic_vector(s_mix_y_result);
            s_io_0.u       <= std_logic_vector(s_mix_u_result);
            s_io_0.v       <= std_logic_vector(s_mix_v_result);
            s_io_0.hsync_n <= s_sync_pipe(14)(0);
            s_io_0.vsync_n <= s_sync_pipe(14)(1);
            s_io_0.avid    <= s_mix_y_valid and s_mix_u_valid and s_mix_v_valid;
            s_io_0.field_n <= s_sync_pipe(14)(3);
            s_io_1 <= s_io_0;
        end if;
    end process;

    -- ====================================================================
    -- Output Assignment
    -- ====================================================================

    data_out.y       <= s_io_1.y;
    data_out.u       <= s_io_1.u;
    data_out.v       <= s_io_1.v;
    data_out.hsync_n <= s_io_1.hsync_n;
    data_out.vsync_n <= s_io_1.vsync_n;
    data_out.avid    <= s_io_1.avid;
    data_out.field_n <= s_io_1.field_n;

end architecture tetris;
