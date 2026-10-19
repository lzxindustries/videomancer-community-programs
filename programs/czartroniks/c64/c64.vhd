-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Videomancer C64  v2.0.2
-- Commodore 64 display re-render
--
-- Re-draws the picture on a block grid in the C64's 16 colours (Colodore),
-- with the native 320x200 grid of the 1080-line raster, multicolour
-- (double-wide) pixels, per-block ordered dither on P12, a C64 border sized
-- as a fraction of the screen, tape-loading border stripes, a READY. prompt
-- and block-row scanlines.
--
-- Pixelation (the CGA program's HW-proven scheme):
--   1. DOWNSAMPLE: each cell on the FIRST line of its cell row averages its
--      first 2^k pixels (4 of 6 / 8 of 12 for the native sizes).
--   2. STORE: palette-match the average, write the 4-bit index into a
--      per-column BRAM.
--   3. UPSCALE: every output pixel reads its column's entry -> solid blocks.
--      (A row's first line reads before that row's write, so the grid sits
--      one line low; the field's first line is blanked.)
--
-- Anti-flicker ("Stability", S7): a ping-pong pair of BRAM grids keeps 8
-- bits per 32x16-pixel tile (60x68 at 1080p), used two ways:
--   * Dither off -> DECISION hysteresis {pending, current}: margin-on-
--     challenger + two-field debounce (cga v1.3, HW-validated).
--   * Dither on  -> VALUE hysteresis: the tile stores luma bits 9:2; a cell's
--     undithered luma snaps to it within +-20 codes and the dither is added
--     after the snap (cga v2.0, HW-validated).
-- Any decision-side control change re-seeds the grids (two plain fields).
--
-- Resolution scaling (v2.0.2): block sizes are virtual 1080-line units.
-- The cell counters step per pixel / per field line from the video timing
-- ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p,
-- and at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame
-- line, so every K1 detent -- the native C64 6x5 one too -- keeps its size
-- relative to the screen height and stays square on a 4:3 screen
-- (non-integer sizes alternate pixel widths); the READY. glyphs (one block
-- per glyph pixel) scale with them.  A field line can cross several block
-- rows at SD; the row wrap runs one row per blanking clock.  Stability
-- tiles are 32x16 virtual units, Loading stripes change ~1 virtual line in
-- 16, and the border stays a fraction of the measured raster.
--
-- Pipeline: match/decision path T1..T3b (writes land 11 clocks after the
-- pixel, for later lines); display path buf -> 2 fabric regs -> palette
-- lookup -> scanline multiply -> output.  Latency 7 clocks in every mode.
--
-- License: GPL-3.0
-- Author: Claude (Anthropic)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture c64 of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (7 + 1 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

  -- ==========================================================================
  -- Constants
  -- ==========================================================================

  constant C_LATENCY  : integer := 7;
  constant C_MAX_COLS : integer := 1024;  -- cells >= 2 px: <= 960 per HD line

  type t_pal_entry is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;
  type t_pal16 is array(0 to 15) of t_pal_entry;

  -- Colodore C64 palette, BT.601 re-levelled to black 64 / white 768 (full-
  -- scale luma clips chroma on hardware), chroma at nominal amplitude.
  -- Stored (y, u=Cr, v=Cb): U/V swapped for Videomancer, so raw input is
  -- matched directly (as cga).
  constant C_PAL : t_pal16 := (
    (y => to_unsigned(  64, 10), u => to_unsigned( 512, 10), v => to_unsigned( 512, 10)),  --  0 black
    (y => to_unsigned( 768, 10), u => to_unsigned( 512, 10), v => to_unsigned( 512, 10)),  --  1 white
    (y => to_unsigned( 271, 10), u => to_unsigned( 648, 10), v => to_unsigned( 475, 10)),  --  2 red
    (y => to_unsigned( 557, 10), u => to_unsigned( 357, 10), v => to_unsigned( 554, 10)),  --  3 cyan
    (y => to_unsigned( 326, 10), u => to_unsigned( 630, 10), v => to_unsigned( 623, 10)),  --  4 purple
    (y => to_unsigned( 438, 10), u => to_unsigned( 388, 10), v => to_unsigned( 396, 10)),  --  5 green
    (y => to_unsigned( 222, 10), u => to_unsigned( 484, 10), v => to_unsigned( 706, 10)),  --  6 blue
    (y => to_unsigned( 686, 10), u => to_unsigned( 542, 10), v => to_unsigned( 289, 10)),  --  7 yellow
    (y => to_unsigned( 324, 10), u => to_unsigned( 632, 10), v => to_unsigned( 407, 10)),  --  8 orange
    (y => to_unsigned( 225, 10), u => to_unsigned( 579, 10), v => to_unsigned( 396, 10)),  --  9 brown
    (y => to_unsigned( 436, 10), u => to_unsigned( 665, 10), v => to_unsigned( 469, 10)),  -- 10 lt red
    (y => to_unsigned( 268, 10), u => to_unsigned( 512, 10), v => to_unsigned( 512, 10)),  -- 11 dk grey
    (y => to_unsigned( 404, 10), u => to_unsigned( 512, 10), v => to_unsigned( 512, 10)),  -- 12 md grey
    (y => to_unsigned( 667, 10), u => to_unsigned( 388, 10), v => to_unsigned( 394, 10)),  -- 13 lt green
    (y => to_unsigned( 407, 10), u => to_unsigned( 481, 10), v => to_unsigned( 732, 10)),  -- 14 lt blue
    (y => to_unsigned( 555, 10), u => to_unsigned( 512, 10), v => to_unsigned( 512, 10))   -- 15 lt grey
  );

  -- ==========================================================================
  -- Manhattan distance on top 5 bits (32 levels per channel).
  -- (4-bit coarse matching was modelled in cga v1.1 as a flicker placebo.)
  -- ==========================================================================
  function f_dist(
    py, pu, pv : unsigned(9 downto 0);
    cy, cu, cv : unsigned(9 downto 0)
  ) return unsigned is
    variable dy, du, dv : unsigned(4 downto 0);
  begin
    if py(9 downto 5) > cy(9 downto 5) then
      dy := py(9 downto 5) - cy(9 downto 5);
    else
      dy := cy(9 downto 5) - py(9 downto 5);
    end if;
    if pu(9 downto 5) > cu(9 downto 5) then
      du := pu(9 downto 5) - cu(9 downto 5);
    else
      du := cu(9 downto 5) - pu(9 downto 5);
    end if;
    if pv(9 downto 5) > cv(9 downto 5) then
      dv := pv(9 downto 5) - cv(9 downto 5);
    else
      dv := cv(9 downto 5) - pv(9 downto 5);
    end if;
    return resize(dy, 7) + resize(du, 7) + resize(dv, 7);
  end function;

  -- ==========================================================================
  -- Ramp mode (S9): 8 luma zones over the legal 64..940 range, each mapped
  -- to one C64 colour, darkest to lightest: black, blue, purple, lt blue,
  -- lt red, lt grey, yellow, white.  C_RAMP_INV maps a stored index back to
  -- its zone for the hysteresis judgement (non-ramp colours -> zone 0; the
  -- grids re-seed on a mode change, so they never hold one in Ramp mode).
  -- ==========================================================================
  type t_idx8 is array(0 to 7) of integer range 0 to 15;
  constant C_RAMP : t_idx8 := (0, 6, 4, 14, 10, 15, 7, 1);
  type t_zone16 is array(0 to 15) of integer range 0 to 7;
  constant C_RAMP_INV : t_zone16 := (0, 7, 0, 0, 2, 0, 1, 6, 0, 0, 4, 0, 0, 0, 3, 5);
  type t_bound8 is array(0 to 7) of unsigned(9 downto 0);
  constant C_ZLO : t_bound8 := (to_unsigned(  0, 10), to_unsigned(174, 10),
                                to_unsigned(284, 10), to_unsigned(393, 10),
                                to_unsigned(503, 10), to_unsigned(612, 10),
                                to_unsigned(722, 10), to_unsigned(831, 10));
  constant C_ZHI : t_bound8 := (to_unsigned(173, 10), to_unsigned(283, 10),
                                to_unsigned(392, 10), to_unsigned(502, 10),
                                to_unsigned(611, 10), to_unsigned(721, 10),
                                to_unsigned(830, 10), to_unsigned(1023, 10));

  -- 4x4 Bayer matrix, index = row(1:0) & col(1:0)
  type t_bayer is array(0 to 15) of integer range 0 to 15;
  constant C_BAYER : t_bayer := ( 0,  8,  2, 10,
                                 12,  4, 14,  6,
                                  3, 11,  1,  9,
                                 15,  7, 13,  5);

  -- Resolution scaling: per-mode steps from the video timing ID
  -- (registers_in(8)).  x in half virtual units per pixel: 1080 2 (1.0),
  -- 720p 3 (1.5), SD 4 (2.0: 720 px span 1440 units, a 4:3 window of the
  -- 1080-line canvas).  y in Q4.8 virtual units per FIELD line (1080 /
  -- active lines, doubled for interlaced modes).
  function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
  begin
    case to_integer(unsigned(t)) is
      when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
      when 5 | 6 | 14     => return to_unsigned(3, 3);
      when others         => return to_unsigned(2, 3);
    end case;
  end function;
  function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
  begin
    case to_integer(unsigned(t)) is
      when 0          => return to_unsigned(1138, 12);  -- 480i: 2 x 1080/486
      when 4          => return to_unsigned(576, 12);   -- 480p: 1080/480
      when 8          => return to_unsigned(960, 12);   -- 576i: 2 x 1080/576
      when 12         => return to_unsigned(480, 12);   -- 576p: 1080/576
      when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p: 1.5
      when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i: 2.0
      when others     => return to_unsigned(256, 12);   -- 1080p: 1.0
    end case;
  end function;

  -- READY. prompt glyphs from the C64 character ROM: R E A D Y . cursor blank
  type t_glyph_rows is array(0 to 63) of std_logic_vector(7 downto 0);
  constant C_FONT : t_glyph_rows := (
    x"7C", x"66", x"66", x"7C", x"78", x"6C", x"66", x"00",   -- R
    x"7E", x"60", x"60", x"78", x"60", x"60", x"7E", x"00",   -- E
    x"18", x"3C", x"66", x"7E", x"66", x"66", x"66", x"00",   -- A
    x"78", x"6C", x"66", x"66", x"66", x"6C", x"78", x"00",   -- D
    x"66", x"66", x"66", x"3C", x"18", x"18", x"18", x"00",   -- Y
    x"00", x"00", x"00", x"00", x"00", x"18", x"18", x"00",   -- .
    x"FF", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF",   -- cursor
    x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00"    -- blank
  );

  -- ==========================================================================
  -- Signals
  -- ==========================================================================

  -- Sync handling
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';
  signal s_in           : t_video_stream_yuv444_30b;

  type t_sr_logic is array(0 to C_LATENCY - 1) of std_logic;
  signal s_avid_sr  : t_sr_logic := (others => '0');
  signal s_hsync_sr : t_sr_logic := (others => '1');
  signal s_vsync_sr : t_sr_logic := (others => '1');
  signal s_field_sr : t_sr_logic := (others => '0');

  -- Controls (registered; decoded off the SPI register net)
  signal s_scode      : unsigned(2 downto 0) := to_unsigned(2, 3);  -- 2x 4x C64 8x 16x
  signal s_bcol       : unsigned(3 downto 0) := to_unsigned(14, 4);
  signal s_scan4      : unsigned(3 downto 0) := (others => '0');
  signal s_stab_on    : std_logic := '1';
  signal s_multi      : std_logic := '0';
  signal s_ramp_on    : std_logic := '0';
  signal s_load_on    : std_logic := '0';
  signal s_ready_on   : std_logic := '0';
  signal s_contrast   : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_color      : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_damp       : unsigned(7 downto 0) := (others => '0');
  signal s_dith_on    : std_logic := '0';
  signal s_k2         : unsigned(7 downto 0) := (others => '0');

  -- Per-frame arithmetic constants
  signal s_c64     : signed(7 downto 0) := (others => '0');       -- 64 - c7
  signal s_cbias_a : signed(17 downto 0) := (others => '0');
  signal s_cbias   : signed(17 downto 0) := (others => '0');      -- 502*(64-c7)
  signal s_c7      : unsigned(6 downto 0) := to_unsigned(64, 7);
  signal s_cg      : unsigned(7 downto 0) := to_unsigned(32, 8);  -- chroma gain, 32 = 1x
  signal s_qbias   : signed(18 downto 0) := (others => '0');      -- 512*(32-cg)

  -- Raster measurement -> native block size, border geometry
  signal s_width   : unsigned(10 downto 0) := to_unsigned(1920, 11);  -- per line
  signal s_width_f : unsigned(10 downto 0) := to_unsigned(1920, 11);  -- per field
  signal s_height  : unsigned(10 downto 0) := to_unsigned(1080, 11);
  signal s_k8      : unsigned(7 downto 0) := (others => '0');
  signal s_k13     : unsigned(8 downto 0) := (others => '0');
  signal s_bx_p    : unsigned(15 downto 0) := (others => '0');
  signal s_by_p    : unsigned(16 downto 0) := (others => '0');
  signal s_bx      : unsigned(10 downto 0) := (others => '0');
  signal s_by      : unsigned(10 downto 0) := (others => '0');
  signal s_xr      : unsigned(10 downto 0) := to_unsigned(1920, 11);
  signal s_yb      : unsigned(10 downto 0) := to_unsigned(1080, 11);

  -- Cell sizing (registered)
  signal s_h_base  : unsigned(5 downto 0) := to_unsigned(6, 6);
  signal s_h_size  : unsigned(5 downto 0) := to_unsigned(6, 6);
  signal s_v_size  : unsigned(4 downto 0) := to_unsigned(5, 5);
  signal s_h_shift : unsigned(2 downto 0) := to_unsigned(2, 3);
  signal s_acc_lim : unsigned(5 downto 0) := to_unsigned(4, 6);

  -- Cell tracking
  signal s_cell_x   : unsigned(5 downto 0)  := (others => '0');  -- pixel # in cell
  -- Resolution scaling (v2.0.2): block sizes are in virtual 1080-line
  -- units, stepped per pixel (x, half units: 2/3/4 = 1.0/1.5/2.0) and per
  -- field line (y, Q4.8) from the video timing ID.  1.0 / 1.0 at 1080p.
  signal s_xs       : unsigned(2 downto 0)  := to_unsigned(2, 3);
  signal s_ys       : unsigned(11 downto 0) := to_unsigned(256, 12);
  signal s_ys3      : unsigned(13 downto 0) := (others => '0');
  signal s_hgrain   : std_logic := '0';  -- blocks < 1.5 px wide
  signal s_vgrain   : std_logic := '0';  -- block rows < 1.5 lines tall
  signal s_vhalf    : std_logic := '0';  -- block rows < 2 lines tall
  signal s_cell_xh  : unsigned(6 downto 0)  := (others => '0');  -- virtual x in cell, half units
  signal s_hth      : unsigned(6 downto 0)  := to_unsigned(10, 7); -- 2 * h_size - x step
  signal s_cell_orgv : unsigned(10 downto 0) := (others => '0'); -- cell origin, virtual units
  signal s_vy       : unsigned(19 downto 0) := (others => '0');  -- virtual y, Q.8
  signal s_vyl      : unsigned(13 downto 0) := (others => '0');  -- virtual y in cell row, Q.8
  signal s_row_new  : std_logic := '1';                          -- first line of a cell row
  signal s_cell_col : unsigned(10 downto 0) := (others => '0');
  signal s_cell_row : unsigned(7 downto 0)  := (others => '0');
  signal s_line_act : std_logic := '0';
  signal s_act_row  : unsigned(10 downto 0) := (others => '0');
  signal s_px       : unsigned(10 downto 0) := (others => '0');
  signal s_saw_act  : std_logic := '0';
  signal s_fcount   : unsigned(6 downto 0) := (others => '0');

  -- READY. prompt: cell coordinates inside the border
  signal s_icol      : unsigned(6 downto 0) := (others => '0');
  signal s_irow      : unsigned(4 downto 0) := (others => '0');
  signal s_col_inner : std_logic := '0';
  signal s_row_inner : std_logic := '0';

  -- Tape-loading stripes: per-line hash -> border colour
  signal s_lh1, s_lh2, s_lh3 : unsigned(23 downto 0) := (others => '0');
  signal s_load_idx : unsigned(3 downto 0) := to_unsigned(14, 4);

  signal s_acc_y, s_acc_u, s_acc_v : unsigned(13 downto 0) := (others => '0');
  signal s_use_avg : std_logic := '0';

  -- Bootstrap / re-seed (two plain-match fields)
  signal s_boot     : unsigned(1 downto 0) := (others => '0');
  signal s_sig      : std_logic_vector(7 downto 0) := (others => '0');
  signal s_sig_snap : std_logic_vector(7 downto 0) := (others => '0');
  signal s_con_snap : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_col_snap : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_sig_chg, s_con_chg, s_col_chg : std_logic := '0';
  signal s_vsnap    : std_logic := '0';   -- value-hysteresis active
  signal s_dhyst    : std_logic := '0';   -- decision-hysteresis active

  -- Per-column buffer (palette index)
  type t_pidx_buf is array(0 to C_MAX_COLS - 1) of unsigned(3 downto 0);
  signal s_pidx_buf : t_pidx_buf := (others => "0000");
  signal s_buf_we    : std_logic := '0';
  signal s_buf_waddr : unsigned(9 downto 0) := (others => '0');
  signal s_buf_wdata : unsigned(3 downto 0) := (others => '0');
  signal s_buf_raddr : unsigned(9 downto 0) := (others => '0');
  signal s_buf_rdata : unsigned(3 downto 0) := (others => '0');
  signal s_bufq, s_bufq2 : unsigned(3 downto 0) := (others => '0');

  -- Hysteresis grid (32x16-px tiles, 60x68 at 1080p), ping-pong banks
  constant C_GRID_SIZE : integer := 4096;
  type t_hyst_grid is array(0 to C_GRID_SIZE - 1) of unsigned(7 downto 0);
  signal s_grid_a : t_hyst_grid := (others => x"00");
  signal s_grid_b : t_hyst_grid := (others => x"00");
  signal s_fparity    : std_logic := '0';
  signal s_grid_raddr : unsigned(11 downto 0) := (others => '0');
  signal s_grid_waddr : unsigned(11 downto 0) := (others => '0');
  signal s_grid_we    : std_logic := '0';
  signal s_grid_wdata : unsigned(7 downto 0) := (others => '0');
  signal s_grid_rd_a, s_grid_rd_b : unsigned(7 downto 0) := (others => '0');

  -- Match pipeline
  signal s_trow_d1  : unsigned(6 downto 0) := (others => '0');
  signal s_tcol_d1  : unsigned(5 downto 0) := (others => '0');
  signal s_bay_d1   : unsigned(3 downto 0) := (others => '0');
  signal s_match_y, s_match_u, s_match_v : unsigned(9 downto 0) := (others => '0');
  signal s_dith2, s_dith3, s_dith4 : signed(10 downto 0) := (others => '0');
  signal s_yprod    : unsigned(16 downto 0) := (others => '0');
  signal s_uprod, s_vprod : unsigned(17 downto 0) := (others => '0');
  signal s_ycon, s_ucon4, s_vcon4 : unsigned(9 downto 0) := (others => '0');
  signal s_gprev4, s_gprev5, s_gprev6, s_gprev7, s_gprev8 : unsigned(7 downto 0) := (others => '0');
  signal s_d11      : signed(10 downto 0) := (others => '0');
  signal s_q5       : unsigned(7 downto 0) := (others => '0');
  signal s_snap6    : std_logic := '0';
  signal s_ya5, s_ya6, s_yb6 : signed(11 downto 0) := (others => '0');
  signal s_lnew6, s_lnew7, s_lnew8, s_lnew9, s_lnew10 : unsigned(7 downto 0) := (others => '0');
  signal s_uc5, s_vc5, s_uc6, s_vc6, s_uc7, s_vc7 : unsigned(9 downto 0) := (others => '0');
  signal s_ym7, s_ym8 : unsigned(9 downto 0) := (others => '0');

  type t_dist_arr is array(0 to 15) of unsigned(6 downto 0);
  signal s_dist     : t_dist_arr := (others => (others => '0'));
  signal s_luma_idx8, s_luma_idx9 : unsigned(3 downto 0) := (others => '0');
  signal s_pz8      : integer range 0 to 7 := 0;

  -- T3a: 16 -> 4 group winners + incumbent distances
  signal s_gmin_a, s_gmin_b, s_gmin_c, s_gmin_d : unsigned(6 downto 0) := (others => '0');
  signal s_gidx_a, s_gidx_b, s_gidx_c, s_gidx_d : unsigned(3 downto 0) := (others => '0');
  signal s_pdist_n9, s_pdist_l9 : unsigned(6 downto 0) := (others => '0');
  signal s_gprev9   : unsigned(7 downto 0) := (others => '0');

  -- T3a2: challenger
  signal s_chal      : unsigned(3 downto 0) := (others => '0');
  signal s_chal_dist : unsigned(6 downto 0) := (others => '0');
  signal s_prev_dist : unsigned(6 downto 0) := (others => '0');
  signal s_prev10, s_pend10 : unsigned(3 downto 0) := (others => '0');

  -- write-side delay chains (index = stage)
  -- (stage 1 is driven by p_t1, so it gets its own signals: one driver
  -- per signal, or GHDL resolves the arrays to 'X')
  type t_col_arr  is array(2 to 10) of unsigned(9 downto 0);
  type t_gad_arr  is array(3 to 10) of unsigned(11 downto 0);
  signal s_we1   : std_logic := '0';
  signal s_wcol1 : unsigned(9 downto 0) := (others => '0');
  signal s_we    : std_logic_vector(2 to 10) := (others => '0');
  signal s_wcol  : t_col_arr := (others => (others => '0'));
  signal s_gaddr : t_gad_arr := (others => (others => '0'));

  -- Display
  signal s_bord1, s_bord2, s_bord3 : std_logic := '0';
  signal s_tbyte1 : std_logic_vector(7 downto 0) := (others => '0');
  signal s_tcol1  : unsigned(2 downto 0) := (others => '0');
  signal s_ton1, s_txt2, s_txt3 : std_logic := '0';
  signal s_disp_y, s_disp_u, s_disp_v : unsigned(9 downto 0) := (others => '0');
  signal s_sf4    : unsigned(4 downto 0) := to_unsigned(16, 5);   -- scanline factor, 16 = 1x
  signal s_pyd    : unsigned(14 downto 0) := (others => '0');
  signal s_pud, s_pvd : signed(16 downto 0) := (others => '0');
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

begin

  -- ==========================================================================
  -- Controls: decode, per-frame constants, border geometry, change flags.
  -- ==========================================================================
  p_ctl : process(clk)
    variable v_r0, v_r1, v_r2, v_r3, v_r7 : unsigned(9 downto 0);
    variable v_c     : unsigned(9 downto 0);
    variable v_cg    : unsigned(7 downto 0);
    variable v_v     : unsigned(4 downto 0);
  begin
    if rising_edge(clk) then
      v_r0 := unsigned(registers_in(0));
      v_r1 := unsigned(registers_in(1));
      v_r2 := unsigned(registers_in(2));
      v_r3 := unsigned(registers_in(3));
      v_r7 := unsigned(registers_in(7));

      -- K1 Size: 5 labels.  The firmware loads a labelled default as the
      -- raw label INDEX (0..4), not a 0..1023 position: treat v < 5 as one.
      if    v_r0 < 5   then s_scode <= v_r0(2 downto 0);
      elsif v_r0 < 205 then s_scode <= "000";
      elsif v_r0 < 410 then s_scode <= "001";
      elsif v_r0 < 615 then s_scode <= "010";
      elsif v_r0 < 820 then s_scode <= "011";
      else                  s_scode <= "100";
      end if;

      -- K3 Border Col: 16 labels (same default-index rule)
      if v_r2 < 16 then s_bcol <= v_r2(3 downto 0);
      else              s_bcol <= v_r2(9 downto 6);
      end if;

      s_k2       <= v_r1(9 downto 2);
      s_scan4    <= v_r3(9 downto 6);
      s_contrast <= unsigned(registers_in(5));
      s_color    <= unsigned(registers_in(4));

      s_stab_on  <= registers_in(6)(0);
      s_multi    <= registers_in(6)(1);
      s_ramp_on  <= registers_in(6)(2);
      s_load_on  <= registers_in(6)(3);
      s_ready_on <= registers_in(6)(4);

      -- P12 Dither (deadband at the bottom so pot noise can't flip modes)
      s_damp <= v_r7(9 downto 2);
      if v_r7 >= 32 then s_dith_on <= '1'; else s_dith_on <= '0'; end if;

      -- Resolution scaling: per-mode steps (constant within a mode).
      s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
      s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
      s_ys3 <= resize(s_ys, 14) + shift_left(resize(s_ys, 14), 1);

      -- Block size in virtual units; the native C64 block is 320x200 of
      -- the 1080p raster (6x5), the same size relative to the screen
      -- height in every mode.  Multicolour doubles the width (160x200 fat
      -- pixels).
      case s_scode is
        when "000"  => s_h_base <= to_unsigned(2, 6);  v_v := to_unsigned(2, 5);
        when "001"  => s_h_base <= to_unsigned(4, 6);  v_v := to_unsigned(4, 5);
        when "011"  => s_h_base <= to_unsigned(8, 6);  v_v := to_unsigned(8, 5);
        when "100"  => s_h_base <= to_unsigned(16, 6); v_v := to_unsigned(16, 5);
        when others => s_h_base <= to_unsigned(6, 6);  v_v := to_unsigned(5, 5);
      end case;
      s_v_size <= v_v;
      if s_multi = '1' then s_h_size <= s_h_base(4 downto 0) & '0';
      else                  s_h_size <= s_h_base;
      end if;
      -- Average the largest power of 2 (max 16) that fits in the narrowest
      -- cell of this mode: n pixels fit when 2*size >= n * x step (cells
      -- alternate widths when the size isn't a whole number of pixels).
      if    (s_h_size & '0') >= shift_left(resize(s_xs, 7), 4) then
        s_acc_lim <= to_unsigned(16, 6); s_h_shift <= "100";
      elsif (s_h_size & '0') >= shift_left(resize(s_xs, 7), 3) then
        s_acc_lim <= to_unsigned(8, 6);  s_h_shift <= "011";
      elsif (s_h_size & '0') >= shift_left(resize(s_xs, 7), 2) then
        s_acc_lim <= to_unsigned(4, 6);  s_h_shift <= "010";
      elsif (s_h_size & '0') >= shift_left(resize(s_xs, 7), 1) then
        s_acc_lim <= to_unsigned(2, 6);  s_h_shift <= "001";
      else
        s_acc_lim <= to_unsigned(1, 6);  s_h_shift <= "000";
      end if;
      s_hth <= (s_h_size & '0') - s_xs;
      -- Thin-block classes (real size = virtual size / step): Bayer grain
      -- and scanline style follow the block's size in pixels / lines.
      -- Never at 1080p (blocks >= 2 px), so 1080p is unchanged.
      if resize(s_h_size & "00", 9) < resize(s_xs, 9) + shift_left(resize(s_xs, 9), 1) then
        s_hgrain <= '1';
      else
        s_hgrain <= '0';
      end if;
      if (s_v_size & "000000000") < s_ys3 then s_vgrain <= '1'; else s_vgrain <= '0'; end if;
      if (s_v_size & "0000000") < s_ys then s_vhalf <= '1'; else s_vhalf <= '0'; end if;

      -- Contrast pivots at mid-grey 502; 50% = unity.  ycon =
      -- (y*c7 + 502*(64-c7)) >> 6, bias built one add per stage.
      s_c7      <= s_contrast(9 downto 3);
      s_c64     <= to_signed(64, 8) - signed('0' & s_c7);
      s_cbias_a <= shift_left(resize(s_c64, 18), 9) - shift_left(resize(s_c64, 18), 3);
      s_cbias   <= s_cbias_a - shift_left(resize(s_c64, 18), 1);

      -- Color (K5): input chroma gain before matching, 0..1x below 50%,
      -- 1..5x above.
      v_c := s_color;
      if v_c(9) = '0' then v_cg := "000" & v_c(8 downto 4);
      else                 v_cg := to_unsigned(32, 8) + ('0' & v_c(8 downto 2));
      end if;
      s_cg    <= v_cg;
      s_qbias <= shift_left(to_signed(32, 19) - signed(resize(s_cg, 19)), 9);

      -- Border: fractions of the measured raster.  k8/1024 of the width
      -- per side (max 25%), 1.6x that of the height (the C64's 32/384 vs
      -- 36/272 proportions; max 40%).  Loading forces at least the
      -- authentic C64 border (k8 = 85) so its stripes always show.
      if s_load_on = '1' and s_k2 < 85 then s_k8 <= to_unsigned(85, 8);
      else                                  s_k8 <= s_k2;
      end if;
      s_k13  <= resize(shift_right(resize(s_k8, 12) + shift_left(resize(s_k8, 12), 2)
                                   + shift_left(resize(s_k8, 12), 3), 3), 9);
      s_bx_p <= s_width_f(10 downto 3) * s_k8;
      s_by_p <= s_height(10 downto 3) * s_k13;
      s_bx   <= resize(s_bx_p(15 downto 7), 11);
      s_by   <= resize(s_by_p(16 downto 7), 11);
      s_xr   <= s_width_f - s_bx;
      s_yb   <= s_height - s_by;

      -- Decision-side signature: any change re-seeds the grids.
      s_sig <= std_logic_vector(s_scode) & s_multi & s_ramp_on & s_dith_on
               & s_stab_on & '0';
      if s_sig /= s_sig_snap then s_sig_chg <= '1'; else s_sig_chg <= '0'; end if;
      if (s_contrast > s_con_snap and s_contrast - s_con_snap > 16)
         or (s_con_snap > s_contrast and s_con_snap - s_contrast > 16) then
        s_con_chg <= '1';
      else
        s_con_chg <= '0';
      end if;
      if (s_color > s_col_snap and s_color - s_col_snap > 16)
         or (s_col_snap > s_color and s_col_snap - s_color > 16) then
        s_col_chg <= '1';
      else
        s_col_chg <= '0';
      end if;

      -- Both modes wait out a pending control change: the grids switch
      -- meaning with the dither/stability setting, and until the re-seed at
      -- the next field they hold the other mode's data.
      if s_stab_on = '1' and s_boot = "11" and s_sig_chg = '0' then
        s_vsnap <= s_dith_on;
        s_dhyst <= not s_dith_on;
      else
        s_vsnap <= '0';
        s_dhyst <= '0';
      end if;
    end if;
  end process p_ctl;

  -- ==========================================================================
  -- T1: input register, cell tracking, accumulator, raster measurement.
  -- ==========================================================================
  p_t1 : process(clk)
    variable v_at_cell_end : boolean;
    variable v_bc, v_br : unsigned(1 downto 0);
    variable v_lk : unsigned(23 downto 0);
  begin
    if rising_edge(clk) then
      s_avid_sr(0)  <= data_in.avid;
      s_hsync_sr(0) <= data_in.hsync_n;
      s_vsync_sr(0) <= data_in.vsync_n;
      s_field_sr(0) <= data_in.field_n;
      for i in 1 to C_LATENCY - 1 loop
        s_avid_sr(i)  <= s_avid_sr(i - 1);
        s_hsync_sr(i) <= s_hsync_sr(i - 1);
        s_vsync_sr(i) <= s_vsync_sr(i - 1);
        s_field_sr(i) <= s_field_sr(i - 1);
      end loop;
      s_in <= data_in;

      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;

      v_at_cell_end := false;

      if data_in.avid = '1' then
        s_line_act <= '1';
        s_saw_act  <= '1';
        s_px       <= s_px + 1;
        -- Virtual x within the cell (half units): this pixel ends its cell
        -- when the next one starts at or past the cell size.  At 1080p the
        -- step is 1 unit and this is the old "cell_x = size - 1" test.
        if s_cell_xh >= s_hth then              -- xh + step >= 2 * size
          s_cell_x    <= (others => '0');
          s_cell_xh   <= s_cell_xh - s_hth;
          v_at_cell_end := true;
          s_cell_col  <= s_cell_col + 1;
          s_cell_orgv <= s_cell_orgv + s_h_size;
          -- READY. columns count whole cells that start inside the border
          if s_col_inner = '1' and s_icol(6) = '0' then
            s_icol <= s_icol + 1;
          end if;
          if s_px + 1 >= s_bx then s_col_inner <= '1'; else s_col_inner <= '0'; end if;
        else
          s_cell_x  <= s_cell_x + 1;
          s_cell_xh <= s_cell_xh + s_xs;
        end if;

        -- Average the first acc_lim (power of 2) pixels of the cell
        if s_row_new = '1' then
          if s_cell_x = 0 then
            s_acc_y <= resize(unsigned(data_in.y), 14);
            s_acc_u <= resize(unsigned(data_in.u), 14);
            s_acc_v <= resize(unsigned(data_in.v), 14);
          elsif s_cell_x < s_acc_lim then
            s_acc_y <= s_acc_y + resize(unsigned(data_in.y), 14);
            s_acc_u <= s_acc_u + resize(unsigned(data_in.u), 14);
            s_acc_v <= s_acc_v + resize(unsigned(data_in.v), 14);
          end if;
        end if;
      end if;

      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_cell_x    <= (others => '0');
        s_cell_xh   <= (others => '0');
        s_cell_orgv <= (others => '0');
        s_cell_col  <= (others => '0');
        s_px        <= (others => '0');
        s_icol      <= (others => '0');
        if s_bx = 0 then s_col_inner <= '1'; else s_col_inner <= '0'; end if;
        -- Active lines only advance the row phase (interlace-safe), by one
        -- virtual y step; the cell-row wrap follows in the blanking below.
        if s_line_act = '1' then
          s_width   <= s_px;
          s_act_row <= s_act_row + 1;
          s_vy      <= s_vy + s_ys;
          s_vyl     <= s_vyl + s_ys;
          s_row_new <= '0';
          -- Loading stripes: a new random colour on ~1 virtual line in 16
          -- (hit chance scaled by the y step: 16/256 at 1080p, as before)
          if s_lh3(23 downto 16) < s_ys(11 downto 4) then
            s_load_idx <= s_lh3(11 downto 8);
          end if;
        end if;
        s_line_act <= '0';
      elsif s_line_act = '0' and data_in.avid = '0'
            and s_vyl >= (s_v_size & "00000000") then
        -- Cell-row wrap, one row per blanking clock: a field line can
        -- cross several block rows at SD (2x is under half a field line).
        -- At 1080p this is the old "cell_y = size - 1" wrap, one clock
        -- later and still in the blanking (act_row is already the next
        -- line's).
        s_vyl      <= s_vyl - (s_v_size & "00000000");
        s_cell_row <= s_cell_row + 1;
        s_row_new  <= '1';
        if s_row_inner = '1' and s_irow(4) = '0' then
          s_irow <= s_irow + 1;
        end if;
        if s_act_row >= s_by then s_row_inner <= '1'; else s_row_inner <= '0'; end if;
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_cell_x    <= (others => '0');
        s_cell_xh   <= (others => '0');
        s_cell_orgv <= (others => '0');
        s_cell_col  <= (others => '0');
        s_cell_row  <= (others => '0');
        s_px        <= (others => '0');
        s_act_row   <= (others => '0');
        s_line_act  <= '0';
        s_irow      <= (others => '0');
        s_row_new   <= '1';
        -- the bottom field starts half a field step (one frame line) down
        -- (inert on hardware, whose core ties field_n high)
        if data_in.field_n = '0' then
          s_vy  <= resize(s_ys(11 downto 1), 20);
          s_vyl <= resize(s_ys(11 downto 1), 14);
        else
          s_vy  <= (others => '0');
          s_vyl <= (others => '0');
        end if;
        if s_by = 0 then s_row_inner <= '1'; else s_row_inner <= '0'; end if;
        -- Once per field, on the first vsync edge after active video
        -- (analog vsync is serrated): bank swap, raster, boot and re-seed.
        -- Change flags are free-running 1-bit registers.
        if s_saw_act = '1' then
          s_fparity <= not s_fparity;
          s_saw_act <= '0';
          s_height  <= s_act_row;
          s_width_f <= s_width;
          s_fcount  <= s_fcount + 1;
          if s_boot /= "11" then
            s_boot <= s_boot + 1;
          end if;
          if s_con_chg = '1' then s_con_snap <= s_contrast; end if;
          if s_col_chg = '1' then s_col_snap <= s_color;    end if;
          if s_sig_chg = '1' then s_sig_snap <= s_sig;      end if;
          if s_con_chg = '1' or s_col_chg = '1' or s_sig_chg = '1' then
            s_boot <= "01";
          end if;
        end if;
      end if;

      -- Loading-stripe hash of (field, line); free-running, settles within
      -- a few clocks of each line start.
      v_lk  := "000000" & s_fcount & s_act_row;
      s_lh1 <= v_lk + shift_left(v_lk, 7);
      s_lh2 <= (s_lh1 xor rotate_left(s_lh1, 13)) + to_unsigned(16#5A3C95#, 24);
      s_lh3 <= s_lh2 + shift_left(s_lh2, 3);

      if v_at_cell_end and s_row_new = '1' and data_in.avid = '1' then
        s_use_avg <= '1';
        s_we1     <= '1';
        s_wcol1   <= s_cell_col(9 downto 0);
        -- 32x16 stability tiles in virtual units (same cells per tile in
        -- every mode; at 1080p the old pixel / active-line tiles)
        s_trow_d1 <= s_vy(18 downto 12);
        s_tcol_d1 <= s_cell_orgv(10 downto 5);
        -- Bayer grain never finer than ~2 px / lines (blocks under 1.5 px
        -- or lines: 2x at 720p, the small sizes at SD; never at 1080p)
        if s_vgrain = '1' then v_br := s_cell_row(2 downto 1);
        else                   v_br := s_cell_row(1 downto 0);
        end if;
        if s_hgrain = '1' then v_bc := s_cell_col(2 downto 1);
        else                   v_bc := s_cell_col(1 downto 0);
        end if;
        s_bay_d1  <= to_unsigned(C_BAYER(to_integer(v_br & v_bc)), 4);
      else
        s_use_avg <= '0';
        s_we1     <= '0';
        s_wcol1   <= (others => '0');
      end if;
    end if;
  end process p_t1;

  -- ==========================================================================
  -- Match / decision path
  -- ==========================================================================
  p_match : process(clk)
    variable v_sb    : signed(5 downto 0);
    variable v_dp    : signed(14 downto 0);
    variable v_ys    : signed(18 downto 0);
    variable v_us, v_vs : signed(19 downto 0);
    variable v_ysel  : signed(11 downto 0);
    variable v_z     : integer range 0 to 7;
    variable v_lo, v_hi : unsigned(6 downto 0);
    variable v_loi, v_hii : unsigned(3 downto 0);
    variable v_blo, v_bhi, v_dy : unsigned(9 downto 0);
    variable v_idx, v_cur, v_pend : unsigned(3 downto 0);
  begin
    if rising_edge(clk) then
      -- ---- T1b: average divide / raw select, grid address, dither offset
      if s_use_avg = '1' then
        s_match_y <= resize(s_acc_y srl to_integer(s_h_shift), 10);
        s_match_u <= resize(s_acc_u srl to_integer(s_h_shift), 10);
        s_match_v <= resize(s_acc_v srl to_integer(s_h_shift), 10);
      else
        s_match_y <= unsigned(s_in.y);
        s_match_u <= unsigned(s_in.u);
        s_match_v <= unsigned(s_in.v);
      end if;
      -- tile = trow*60 + tcol
      s_grid_raddr <= resize(
          shift_left(resize(s_trow_d1, 13), 6)
        - shift_left(resize(s_trow_d1, 13), 2)
        + resize(s_tcol_d1, 13), 12);
      -- Bayer offset: (2b - 15) * amp / 16  ->  +-239 codes at 100%
      v_sb := signed('0' & s_bay_d1 & '0') - to_signed(15, 6);
      if s_dith_on = '1' then
        v_dp := v_sb * signed('0' & s_damp);
      else
        v_dp := (others => '0');
      end if;
      s_dith2 <= resize(shift_right(v_dp, 4), 11);

      -- ---- T2a: contrast and chroma-gain products
      s_yprod <= s_match_y * s_c7;
      s_uprod <= s_match_u * s_cg;
      s_vprod <= s_match_v * s_cg;
      s_dith3 <= s_dith2;

      -- ---- T2c: bias, scale, clamp; last field's grid entry (bank mux)
      v_ys := signed(resize(s_yprod, 19)) + resize(s_cbias, 19);
      v_ys := shift_right(v_ys, 6);
      if    v_ys < 0    then s_ycon <= (others => '0');
      elsif v_ys > 1023 then s_ycon <= to_unsigned(1023, 10);
      else                   s_ycon <= unsigned(v_ys(9 downto 0));
      end if;
      v_us := shift_right(signed(resize(s_uprod, 20)) + resize(s_qbias, 20), 5);
      v_vs := shift_right(signed(resize(s_vprod, 20)) + resize(s_qbias, 20), 5);
      if    v_us < 0    then s_ucon4 <= (others => '0');
      elsif v_us > 1023 then s_ucon4 <= to_unsigned(1023, 10);
      else                   s_ucon4 <= unsigned(v_us(9 downto 0));
      end if;
      if    v_vs < 0    then s_vcon4 <= (others => '0');
      elsif v_vs > 1023 then s_vcon4 <= to_unsigned(1023, 10);
      else                   s_vcon4 <= unsigned(v_vs(9 downto 0));
      end if;
      if s_fparity = '0' then s_gprev4 <= s_grid_rd_a;
      else                    s_gprev4 <= s_grid_rd_b;
      end if;
      s_dith4 <= s_dith3;

      -- ---- T2d: distance to the tile's stored luma (bits 9:2)
      s_d11 <= signed(resize(s_ycon, 11)) - signed(shift_left(resize(s_gprev4, 11), 2));
      s_q5  <= s_ycon(9 downto 2);
      s_ya5 <= signed(resize(s_ycon, 12)) + resize(s_dith4, 12);
      s_uc5 <= s_ucon4;
      s_vc5 <= s_vcon4;
      s_gprev5 <= s_gprev4;

      -- ---- T2e: snap within +-20 (stored value is within 3 of its
      -- source, leaving ~16 codes of noise margin)
      if s_vsnap = '1' and s_d11 >= -20 and s_d11 <= 20 then
        s_snap6 <= '1';
        s_lnew6 <= s_gprev5;
      else
        s_snap6 <= '0';
        s_lnew6 <= s_q5;
      end if;
      s_ya6 <= s_ya5;
      s_yb6 <= s_ya5 - resize(s_d11, 12);
      s_uc6 <= s_uc5;
      s_vc6 <= s_vc5;
      s_gprev6 <= s_gprev5;

      -- ---- T2f: select + clamp the match luma
      if s_snap6 = '1' then v_ysel := s_yb6; else v_ysel := s_ya6; end if;
      if    v_ysel < 0    then s_ym7 <= (others => '0');
      elsif v_ysel > 1023 then s_ym7 <= to_unsigned(1023, 10);
      else                     s_ym7 <= unsigned(v_ysel(9 downto 0));
      end if;
      s_uc7 <= s_uc6;
      s_vc7 <= s_vc6;
      s_lnew7  <= s_lnew6;
      s_gprev7 <= s_gprev6;

      -- ---- T2g: sixteen palette distances + ramp-zone pick
      for i in 0 to 15 loop
        s_dist(i) <= f_dist(s_ym7, s_uc7, s_vc7, C_PAL(i).y, C_PAL(i).u, C_PAL(i).v);
      end loop;
      if    s_ym7 < 174 then v_z := 0;
      elsif s_ym7 < 284 then v_z := 1;
      elsif s_ym7 < 393 then v_z := 2;
      elsif s_ym7 < 503 then v_z := 3;
      elsif s_ym7 < 612 then v_z := 4;
      elsif s_ym7 < 722 then v_z := 5;
      elsif s_ym7 < 831 then v_z := 6;
      else                   v_z := 7;
      end if;
      s_luma_idx8 <= to_unsigned(C_RAMP(v_z), 4);
      s_pz8    <= C_RAMP_INV(to_integer(s_gprev7(3 downto 0)));
      s_ym8    <= s_ym7;
      s_lnew8  <= s_lnew7;
      s_gprev8 <= s_gprev7;

      -- ---- T3a: 16 -> 4 group winners (2-deep trees) + incumbent distances
      if s_dist(1) < s_dist(0) then v_lo := s_dist(1); v_loi := x"1";
      else                          v_lo := s_dist(0); v_loi := x"0"; end if;
      if s_dist(3) < s_dist(2) then v_hi := s_dist(3); v_hii := x"3";
      else                          v_hi := s_dist(2); v_hii := x"2"; end if;
      if v_hi < v_lo then s_gmin_a <= v_hi; s_gidx_a <= v_hii;
      else                s_gmin_a <= v_lo; s_gidx_a <= v_loi; end if;

      if s_dist(5) < s_dist(4) then v_lo := s_dist(5); v_loi := x"5";
      else                          v_lo := s_dist(4); v_loi := x"4"; end if;
      if s_dist(7) < s_dist(6) then v_hi := s_dist(7); v_hii := x"7";
      else                          v_hi := s_dist(6); v_hii := x"6"; end if;
      if v_hi < v_lo then s_gmin_b <= v_hi; s_gidx_b <= v_hii;
      else                s_gmin_b <= v_lo; s_gidx_b <= v_loi; end if;

      if s_dist(9) < s_dist(8) then v_lo := s_dist(9); v_loi := x"9";
      else                          v_lo := s_dist(8); v_loi := x"8"; end if;
      if s_dist(11) < s_dist(10) then v_hi := s_dist(11); v_hii := x"B";
      else                            v_hi := s_dist(10); v_hii := x"A"; end if;
      if v_hi < v_lo then s_gmin_c <= v_hi; s_gidx_c <= v_hii;
      else                s_gmin_c <= v_lo; s_gidx_c <= v_loi; end if;

      if s_dist(13) < s_dist(12) then v_lo := s_dist(13); v_loi := x"D";
      else                            v_lo := s_dist(12); v_loi := x"C"; end if;
      if s_dist(15) < s_dist(14) then v_hi := s_dist(15); v_hii := x"F";
      else                            v_hi := s_dist(14); v_hii := x"E"; end if;
      if v_hi < v_lo then s_gmin_d <= v_hi; s_gidx_d <= v_hii;
      else                s_gmin_d <= v_lo; s_gidx_d <= v_loi; end if;

      -- Nearest: the incumbent's own distance.  Ramp: how far y sits
      -- outside the incumbent's zone, in the same 32-code units.
      s_pdist_n9 <= s_dist(to_integer(s_gprev8(3 downto 0)));
      v_blo := C_ZLO(s_pz8);
      v_bhi := C_ZHI(s_pz8);
      if s_ym8 < v_blo then
        v_dy := v_blo - s_ym8;
      elsif s_ym8 > v_bhi then
        v_dy := s_ym8 - v_bhi;
      else
        v_dy := (others => '0');
      end if;
      s_pdist_l9  <= resize(v_dy(9 downto 5), 7);
      s_luma_idx9 <= s_luma_idx8;
      s_gprev9    <= s_gprev8;
      s_lnew9     <= s_lnew8;

      -- ---- T3a2: challenger (4 -> 1), or the ramp pick
      if s_ramp_on = '1' then
        s_chal      <= s_luma_idx9;
        s_chal_dist <= (others => '0');
        s_prev_dist <= s_pdist_l9;
      else
        if s_gmin_b < s_gmin_a then v_lo := s_gmin_b; v_loi := s_gidx_b;
        else                        v_lo := s_gmin_a; v_loi := s_gidx_a; end if;
        if s_gmin_d < s_gmin_c then v_hi := s_gmin_d; v_hii := s_gidx_d;
        else                        v_hi := s_gmin_c; v_hii := s_gidx_c; end if;
        if v_hi < v_lo then s_chal <= v_hii; s_chal_dist <= v_hi;
        else                s_chal <= v_loi; s_chal_dist <= v_lo; end if;
        s_prev_dist <= s_pdist_n9;
      end if;
      s_prev10 <= s_gprev9(3 downto 0);
      s_pend10 <= s_gprev9(7 downto 4);
      s_lnew10 <= s_lnew9;

      -- ---- T3b: decision (margin 2 + debounce), buffer + grid writes
      if s_dhyst = '0' then
        v_idx := s_chal; v_cur := s_chal; v_pend := s_chal;
      elsif resize(s_chal_dist, 8) + 6 < resize(s_prev_dist, 8) then
        v_idx := s_chal; v_cur := s_chal; v_pend := s_chal;
      elsif resize(s_chal_dist, 8) + 2 < resize(s_prev_dist, 8) then
        if s_chal = s_pend10 then
          v_idx := s_chal;   v_cur := s_chal;   v_pend := s_chal;
        else
          v_idx := s_prev10; v_cur := s_prev10; v_pend := s_chal;
        end if;
      else
        v_idx := s_prev10; v_cur := s_prev10; v_pend := s_prev10;
      end if;

      s_buf_we    <= s_we(10);
      s_buf_waddr <= s_wcol(10);
      s_buf_wdata <= v_idx;

      s_grid_we    <= s_we(10);
      s_grid_waddr <= s_gaddr(10);
      if s_dith_on = '1' then
        s_grid_wdata <= s_lnew10;
      else
        s_grid_wdata <= v_pend & v_cur;
      end if;

      -- write-side delay chains
      s_we(2)   <= s_we1;
      s_wcol(2) <= s_wcol1;
      for i in 3 to 10 loop
        s_we(i)   <= s_we(i - 1);
        s_wcol(i) <= s_wcol(i - 1);
      end loop;
      s_gaddr(3) <= s_grid_raddr;
      for i in 4 to 10 loop
        s_gaddr(i) <= s_gaddr(i - 1);
      end loop;
    end if;
  end process p_match;

  -- ==========================================================================
  -- Per-column BRAM (1W1R).  The write is for a cell that has already ended,
  -- so it never shares an address with the live read.
  -- ==========================================================================
  p_buf : process(clk)
  begin
    if rising_edge(clk) then
      if s_buf_we = '1' then
        s_pidx_buf(to_integer(s_buf_waddr)) <= s_buf_wdata;
      end if;
      s_buf_rdata <= s_pidx_buf(to_integer(s_buf_raddr));
    end if;
  end process p_buf;

  s_buf_raddr <= s_cell_col(9 downto 0);

  -- Hysteresis grid banks: read the bank written LAST field (no collision).
  p_grid_a : process(clk)
  begin
    if rising_edge(clk) then
      if s_grid_we = '1' and s_fparity = '1' then
        s_grid_a(to_integer(s_grid_waddr)) <= s_grid_wdata;
      end if;
      s_grid_rd_a <= s_grid_a(to_integer(s_grid_raddr));
    end if;
  end process p_grid_a;

  p_grid_b : process(clk)
  begin
    if rising_edge(clk) then
      if s_grid_we = '1' and s_fparity = '0' then
        s_grid_b(to_integer(s_grid_waddr)) <= s_grid_wdata;
      end if;
      s_grid_rd_b <= s_grid_b(to_integer(s_grid_raddr));
    end if;
  end process p_grid_b;

  -- ==========================================================================
  -- Display path: buffer read -> 2 fabric registers (border / READY. flags
  -- alongside) -> palette lookup -> scanline multiply -> blanking gate.
  -- ==========================================================================
  p_disp : process(clk)
    variable v_ch   : integer range 0 to 7;
    variable v_idx  : unsigned(3 downto 0);
    variable v_dim  : boolean;
    variable v_su, v_sv : signed(10 downto 0);
    variable v_f    : signed(5 downto 0);
  begin
    if rising_edge(clk) then
      -- stage 1: border test, READY. glyph row
      if s_px < s_bx or s_px >= s_xr or s_act_row < s_by or s_act_row >= s_yb then
        s_bord1 <= '1';
      else
        s_bord1 <= '0';
      end if;
      if s_irow(3) = '0' then
        if s_icol(5 downto 3) < 6 then v_ch := to_integer(s_icol(5 downto 3));
        else                           v_ch := 7;
        end if;
      else
        -- blinking cursor under the R (C64 blink ~ 1/3 s)
        if s_icol(5 downto 3) = 0 and s_fcount(4) = '0' then v_ch := 6;
        else                                                  v_ch := 7;
        end if;
      end if;
      s_tbyte1 <= C_FONT(v_ch * 8 + to_integer(s_irow(2 downto 0)));
      s_tcol1  <= s_icol(2 downto 0);
      if s_ready_on = '1' and s_col_inner = '1' and s_row_inner = '1'
         and s_icol(6) = '0' and s_icol(5 downto 4) /= "11"
         and s_irow(4) = '0' then
        s_ton1 <= '1';
      else
        s_ton1 <= '0';
      end if;

      -- stage 2/3: fabric registers after the EBR
      s_bufq  <= s_buf_rdata;
      s_bufq2 <= s_bufq;
      s_bord2 <= s_bord1;
      s_bord3 <= s_bord2;
      s_txt2  <= s_ton1 and s_tbyte1(7 - to_integer(s_tcol1));
      s_txt3  <= s_txt2;

      -- stage 4: palette lookup.  The prompt and the border take the
      -- Border colour (a C64 boots with matching text and border); Loading
      -- stripes replace the border colour line by line.
      v_idx := s_bufq2;
      if s_bord3 = '1' then
        if s_load_on = '1' then v_idx := s_load_idx; else v_idx := s_bcol; end if;
      elsif s_txt3 = '1' then
        v_idx := s_bcol;
      end if;
      s_disp_y <= C_PAL(to_integer(v_idx)).y;
      s_disp_u <= C_PAL(to_integer(v_idx)).u;
      s_disp_v <= C_PAL(to_integer(v_idx)).v;

      -- Scanlines on the block grid: the bottom line of every block row
      -- (a row's first line is the displayed block's last line -- see
      -- header), or alternate lines where block rows are under 1.5 lines.
      -- The 1-line dim is pixel scale, so where rows are under 2 lines
      -- (small sizes at SD / 720p, never 1080p) it dims half as deep:
      -- light lines instead of dark gaps.  Factor 16 = unchanged.
      if s_vgrain = '1' then
        v_dim := s_act_row(0) = '1';
      else
        v_dim := s_row_new = '1';
      end if;
      if not v_dim then
        s_sf4 <= to_unsigned(16, 5);
      elsif s_vhalf = '1' then
        s_sf4 <= to_unsigned(16, 5) - resize(s_scan4(3 downto 1), 5);
      else
        s_sf4 <= to_unsigned(16, 5) - resize(s_scan4, 5);
      end if;

      -- stage 5: scanline multiply, pivoting on black (64) / neutral chroma
      v_su := signed(resize(s_disp_u, 11)) - to_signed(512, 11);
      v_sv := signed(resize(s_disp_v, 11)) - to_signed(512, 11);
      v_f  := signed('0' & s_sf4);
      s_pyd <= (s_disp_y - 64) * s_sf4;
      s_pud <= v_su * v_f;
      s_pvd <= v_sv * v_f;
    end if;
  end process p_disp;

  p_out : process(clk)
    variable v_cu, v_cv : signed(16 downto 0);
  begin
    if rising_edge(clk) then
      -- Blanking gate (latency-aligned avid) + blank the field's first
      -- line, which shows the previous field's bottom row.
      if s_avid_sr(C_LATENCY - 2) = '0' or s_act_row = 0 then
        s_out_y <= to_unsigned(64, 10);
        s_out_u <= to_unsigned(512, 10);
        s_out_v <= to_unsigned(512, 10);
      else
        v_cu := shift_right(s_pud, 4) + to_signed(512, 17);
        v_cv := shift_right(s_pvd, 4) + to_signed(512, 17);
        s_out_y <= s_pyd(13 downto 4) + 64;
        s_out_u <= unsigned(v_cu(9 downto 0));
        s_out_v <= unsigned(v_cv(9 downto 0));
      end if;
    end if;
  end process p_out;

  -- ==========================================================================
  -- Output
  -- ==========================================================================
  s_io_align_in.y <= std_logic_vector(s_out_y);
  s_io_align_in.u <= std_logic_vector(s_out_u);
  s_io_align_in.v <= std_logic_vector(s_out_v);

  s_io_align_in.hsync_n <= s_hsync_sr(C_LATENCY - 1);
  s_io_align_in.vsync_n <= s_vsync_sr(C_LATENCY - 1);
  s_io_align_in.field_n <= s_field_sr(C_LATENCY - 1);
  s_io_align_in.avid    <= s_avid_sr(C_LATENCY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (7 -> 8).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture c64;
