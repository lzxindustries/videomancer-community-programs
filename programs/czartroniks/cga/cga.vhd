-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Videomancer CGA (Color Graphics Adapter) Emulator  v2.0.9
-- LZX Industries Community FPGA Program
--
-- Re-renders the picture in the IBM CGA's own modes:
--   * Mode 4 four-colour palettes (Green/Red/Brown, Cyan/Magenta/Gray,
--     Cyan/Red/Gray), low/high intensity, colour 0 = selectable Ink
--     (the real mode-4 background register)
--   * Mode 6 640-dot Mono: black + one Ink foreground
--   * Composite: 640-dot mono bits read four at a time as the NTSC
--     artifact-colour nibble -> the famous 16 composite colours
-- at a native 320x200 (640x200) block grid of the 1080-line raster,
-- with per-block ordered dither (P12), block-row scanlines and CGA snow.
--
-- Pixelation: downsample + nearest-neighbour upscale.
--   1. DOWNSAMPLE: each cell on the FIRST line of its cell row averages its
--      first 2^k pixels (all of them for power-of-2 cells; 4 of 6 / 2 of 3
--      for the native sizes -- the look, not the bits).
--   2. STORE: palette-match the average and write the index (or, in
--      Composite, the 4-cell nibble) into a per-column BRAM.
--   3. UPSCALE: every output pixel reads its column's entry -> solid blocks.
--      (The read on a row's first line precedes that row's write, so the
--      block grid sits one line low; the field's first line is blanked.)
--
-- Anti-flicker ("Stability", switch 7): a ping-pong pair of BRAM grids
-- keeps 4 bits per 16x16-pixel tile, used two ways:
--   * Dither off -> DECISION hysteresis {pending, current}: margin-on-
--     challenger + two-field debounce (HW-validated in v1.3).
--   * Dither on  -> VALUE hysteresis: the tile stores luma bits 7:4 (16-code
--     steps, modulo 256 -- the live pixel resolves the wrap); a cell's
--     undithered luma snaps to it while within +-24 codes, and the dither
--     is added AFTER the snap.  (Storing bits 9:6, as v2.0 did, posterised
--     gradients to 64-code steps: thick flat bars on hardware.)  Decision hysteresis on shared tiles
--     would scramble dither patterns (a cell judged against another cell's
--     dither phase); value snapping keeps each cell's pattern and still
--     pins noise.
-- Any decision-side control change re-seeds the grids (two plain fields).
--
-- Resolution scaling (v2.0.9): block sizes are virtual 1080-line units.
-- The cell counters step per pixel / per field line from the video timing
-- ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p,
-- and at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame
-- line, so every K2/K3 detent -- the native CGA 6x5 (3x5) one too -- keeps
-- its size relative to the screen height and stays square on a 4:3 screen
-- (non-integer sizes alternate pixel widths).  A field line can cross
-- several block rows at SD; the row wrap runs one row per blanking clock.
-- Stability tiles are 16x16 virtual units.  Dither grain and the scanline
-- style follow the real block size (1080p unchanged).
--
-- Pipeline: match/decision path T1..T3b (writes land 11 clocks after the
-- pixel, for later lines); display path buf -> T4a -> T4b -> T5, output
-- latency 7 clocks for every mode.
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

architecture cga of program_top is
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
  constant C_MAX_COLS : integer := 2048;  -- 2048x4 = 2 EBR; covers 1920
                                          -- cells/line at H Off.

  type t_pal_entry is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;
  type t_pal16 is array(0 to 15) of t_pal_entry;

  -- The 16 RGBI colours, BT.601 limited range re-levelled to black 64 /
  -- white 768 (full-scale luma clips chroma on hardware), chroma at nominal
  -- amplitude.  Stored (y, u=Cr, v=Cb): U/V swapped for Videomancer.  The
  -- v1 hand-calibrated table was exactly full-range BT.601 of these RGBs.
  constant C_RGBI : t_pal16 := (
    (y => to_unsigned( 64, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  --  0 black
    (y => to_unsigned(118, 10), u => to_unsigned(463, 10), v => to_unsigned(811, 10)),  --  1 blue
    (y => to_unsigned(339, 10), u => to_unsigned(262, 10), v => to_unsigned(314, 10)),  --  2 green
    (y => to_unsigned(393, 10), u => to_unsigned(213, 10), v => to_unsigned(613, 10)),  --  3 cyan
    (y => to_unsigned(204, 10), u => to_unsigned(811, 10), v => to_unsigned(411, 10)),  --  4 red
    (y => to_unsigned(258, 10), u => to_unsigned(762, 10), v => to_unsigned(710, 10)),  --  5 magenta
    (y => to_unsigned(342, 10), u => to_unsigned(686, 10), v => to_unsigned(312, 10)),  --  6 brown
    (y => to_unsigned(533, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  --  7 lt gray
    (y => to_unsigned(299, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  --  8 dk gray
    (y => to_unsigned(352, 10), u => to_unsigned(463, 10), v => to_unsigned(811, 10)),  --  9 lt blue
    (y => to_unsigned(574, 10), u => to_unsigned(262, 10), v => to_unsigned(314, 10)),  -- 10 lt green
    (y => to_unsigned(628, 10), u => to_unsigned(213, 10), v => to_unsigned(613, 10)),  -- 11 lt cyan
    (y => to_unsigned(439, 10), u => to_unsigned(811, 10), v => to_unsigned(411, 10)),  -- 12 lt red
    (y => to_unsigned(493, 10), u => to_unsigned(762, 10), v => to_unsigned(710, 10)),  -- 13 lt magenta
    (y => to_unsigned(714, 10), u => to_unsigned(561, 10), v => to_unsigned(213, 10)),  -- 14 yellow
    (y => to_unsigned(768, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))   -- 15 white
  );

  -- Composite artifact colours of 640x200 mono on NTSC (old-style CGA),
  -- indexed by the 4-dot nibble (first dot = MSB).  Nibble 0 is replaced by
  -- the Ink background at display.  Same re-levelling as C_RGBI.
  constant C_COMPOSITE : t_pal16 := (
    (y => to_unsigned( 64, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  --  0 black
    (y => to_unsigned(258, 10), u => to_unsigned(336, 10), v => to_unsigned(470, 10)),  --  1 dk green
    (y => to_unsigned(199, 10), u => to_unsigned(512, 10), v => to_unsigned(920, 10)),  --  2 blue
    (y => to_unsigned(368, 10), u => to_unsigned(236, 10), v => to_unsigned(799, 10)),  --  3 sky blue
    (y => to_unsigned(217, 10), u => to_unsigned(791, 10), v => to_unsigned(499, 10)),  --  4 crimson
    (y => to_unsigned(390, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  --  5 gray
    (y => to_unsigned(367, 10), u => to_unsigned(829, 10), v => to_unsigned(800, 10)),  --  6 violet
    (y => to_unsigned(535, 10), u => to_unsigned(553, 10), v => to_unsigned(679, 10)),  --  7 lavender
    (y => to_unsigned(250, 10), u => to_unsigned(466, 10), v => to_unsigned(378, 10)),  --  8 olive
    (y => to_unsigned(419, 10), u => to_unsigned(190, 10), v => to_unsigned(257, 10)),  --  9 green
    (y => to_unsigned(390, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  -- 10 gray
    (y => to_unsigned(580, 10), u => to_unsigned(216, 10), v => to_unsigned(512, 10)),  -- 11 aqua
    (y => to_unsigned(419, 10), u => to_unsigned(781, 10), v => to_unsigned(257, 10)),  -- 12 orange
    (y => to_unsigned(588, 10), u => to_unsigned(505, 10), v => to_unsigned(136, 10)),  -- 13 lime
    (y => to_unsigned(539, 10), u => to_unsigned(720, 10), v => to_unsigned(542, 10)),  -- 14 pink
    (y => to_unsigned(768, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))   -- 15 white
  );

  -- ==========================================================================
  -- Manhattan distance on top 5 bits (32 levels per channel).
  -- (Coarsening to 4 bits was modelled in v1.1 as a flicker placebo.)
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
  -- Luma-mode mapping (4-colour palettes): luma zone (0 darkest) -> entry.
  -- Palette 0 sorts {0,1,2,3}; palettes 1 and 2 swap the middle pair
  -- (magenta/red is darker than cyan).  Self-inverse, so it also maps a
  -- stored entry back to its zone.  Entry 0 (the Ink background) is always
  -- zone 0: the darks take the background colour, whatever it is.
  -- ==========================================================================
  function f_luma_map(pal : unsigned(1 downto 0); z : unsigned(1 downto 0))
    return unsigned is
  begin
    if pal /= "00" and (z = "01" or z = "10") then
      return not z;          -- 01 <-> 10
    end if;
    return z;
  end function;

  -- Luma zone bounds over the legal 64..940 range after the mid-pivot
  -- contrast (quarters: 283 / 502 / 721; halves: 502).
  type t_bound6 is array(0 to 5) of unsigned(9 downto 0);
  -- index: 0..3 = 4-zone quarters, 4..5 = 2-zone halves
  constant C_ZLO : t_bound6 := (to_unsigned(0, 10), to_unsigned(283, 10),
                                to_unsigned(502, 10), to_unsigned(721, 10),
                                to_unsigned(0, 10), to_unsigned(502, 10));
  constant C_ZHI : t_bound6 := (to_unsigned(282, 10), to_unsigned(501, 10),
                                to_unsigned(720, 10), to_unsigned(1023, 10),
                                to_unsigned(501, 10), to_unsigned(1023, 10));

  -- 4x4 Bayer matrix, index = row(1:0) & col(1:0)
  type t_bayer is array(0 to 15) of integer range 0 to 15;
  constant C_BAYER : t_bayer := ( 0,  8,  2, 10,
                                 12,  4, 14,  6,
                                  3, 11,  1,  9,
                                 15,  7, 13,  5);

  -- ==========================================================================
  -- Resolution scaling: per-mode steps from the video timing ID
  -- (registers_in(8)).  x in half virtual units per pixel: 1080 2 (1.0),
  -- 720p 3 (1.5), SD 4 (2.0: 720 px span 1440 units, a 4:3 window of the
  -- 1080-line canvas).  y in Q4.8 virtual units per FIELD line (1080 /
  -- active lines, doubled for interlaced modes).
  -- ==========================================================================
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
  signal s_mode       : unsigned(2 downto 0) := to_unsigned(1, 3);  -- 0-2 pal, 3 mono, 4 comp
  signal s_intens     : std_logic := '1';
  signal s_ink3       : unsigned(2 downto 0) := (others => '0');
  signal s_stab_on    : std_logic := '1';
  signal s_scanlines  : std_logic := '0';
  signal s_color_mode : std_logic := '0';  -- switch 10: nearest / luma (comp: phase)
  signal s_snow_on    : std_logic := '0';
  signal s_contrast   : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_color      : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_damp       : unsigned(7 downto 0) := (others => '0');
  signal s_dith_on    : std_logic := '0';
  signal s_hcode      : unsigned(2 downto 0) := to_unsigned(3, 3);
  signal s_vcode      : unsigned(2 downto 0) := to_unsigned(3, 3);
  signal s_luma_on    : std_logic := '0';  -- decisions by luma zone
  signal s_zone2      : std_logic := '0';  -- 2-zone (mono / composite)
  signal s_is_comp    : std_logic := '0';
  signal s_pal2       : unsigned(1 downto 0) := "01";

  -- Per-frame arithmetic constants
  signal s_c64   : signed(7 downto 0) := (others => '0');       -- 64 - c7
  signal s_cbias_a : signed(17 downto 0) := (others => '0');
  signal s_cbias : signed(17 downto 0) := (others => '0');      -- 502*(64-c7)
  signal s_c7    : unsigned(6 downto 0) := to_unsigned(64, 7);
  signal s_cg    : unsigned(7 downto 0) := to_unsigned(32, 8);  -- chroma gain, 32 = 1x
  signal s_qbias : signed(18 downto 0) := (others => '0');      -- 512*(32-cg)
  signal s_osat  : unsigned(7 downto 0) := to_unsigned(128, 8); -- 128 = 1x (max)

  -- Palette entries (decision + display), rebuilt from the controls
  type t_pe4 is array(0 to 3) of t_pal_entry;
  signal s_pe    : t_pe4;
  signal s_bg    : t_pal_entry;   -- composite nibble-0 background

  -- Native CGA block (virtual units): 6x5, 3x5 in the 640-dot modes
  signal s_nat_h   : unsigned(2 downto 0) := to_unsigned(6, 3);

  -- Resolution scaling (v2.0.9): block sizes are in virtual 1080-line units,
  -- stepped per pixel (x, half units: 2/3/4 = 1.0/1.5/2.0) and per field
  -- line (y, Q4.8) from the video timing ID.  1.0 / 1.0 at 1080p.
  signal s_xs      : unsigned(2 downto 0)  := to_unsigned(2, 3);
  signal s_ys      : unsigned(11 downto 0) := to_unsigned(256, 12);
  signal s_ys3, s_ys5 : unsigned(13 downto 0) := (others => '0');
  signal s_hgrain  : std_logic := '0';  -- blocks < 1.5 px wide
  signal s_vgrain  : std_logic := '0';  -- block rows < 1.5 lines tall
  signal s_vq25    : std_logic := '0';  -- block rows < 2.5 lines tall

  -- Cell sizing (registered)
  signal s_h_size  : unsigned(4 downto 0) := to_unsigned(6, 5);
  signal s_v_size  : unsigned(4 downto 0) := to_unsigned(5, 5);
  signal s_h_shift : unsigned(2 downto 0) := to_unsigned(2, 3);
  signal s_acc_lim : unsigned(4 downto 0) := to_unsigned(4, 5);

  -- Cell tracking
  signal s_cell_x   : unsigned(4 downto 0)  := (others => '0');  -- pixel # in cell
  signal s_cell_xh  : unsigned(5 downto 0)  := (others => '0');  -- virtual x in cell, half units
  signal s_hth      : unsigned(5 downto 0)  := to_unsigned(10, 6); -- 2 * h_size - x step
  signal s_cell_orgv : unsigned(10 downto 0) := (others => '0'); -- cell origin, virtual units
  signal s_vy       : unsigned(19 downto 0) := (others => '0');  -- virtual y, Q.8
  signal s_vyl      : unsigned(13 downto 0) := (others => '0');  -- virtual y in cell row, Q.8
  signal s_row_new  : std_logic := '1';                          -- first line of a cell row
  signal s_cell_col : unsigned(10 downto 0) := (others => '0');
  signal s_cell_row : unsigned(7 downto 0)  := (others => '0');
  signal s_line_act : std_logic := '0';
  signal s_act_row  : unsigned(10 downto 0) := (others => '0');
  signal s_saw_act  : std_logic := '0';
  signal s_fcount   : unsigned(6 downto 0) := (others => '0');

  signal s_acc_y, s_acc_u, s_acc_v : unsigned(13 downto 0) := (others => '0');
  signal s_use_avg : std_logic := '0';

  -- Bootstrap / re-seed (two plain-match fields)
  signal s_boot     : unsigned(1 downto 0) := (others => '0');
  signal s_sig      : std_logic_vector(15 downto 0) := (others => '0');
  signal s_sig_snap : std_logic_vector(15 downto 0) := (others => '0');
  signal s_con_snap : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_col_snap : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_sig_chg, s_con_chg, s_col_chg : std_logic := '0';
  signal s_vsnap    : std_logic := '0';   -- value-hysteresis active
  signal s_dhyst    : std_logic := '0';   -- decision-hysteresis active

  -- Per-column buffer (index, or composite nibble)
  type t_pidx_buf is array(0 to C_MAX_COLS - 1) of unsigned(3 downto 0);
  signal s_pidx_buf : t_pidx_buf := (others => "0000");
  signal s_buf_we    : std_logic := '0';
  signal s_buf_waddr : unsigned(10 downto 0) := (others => '0');
  signal s_buf_wdata : unsigned(3 downto 0) := (others => '0');
  signal s_buf_raddr : unsigned(10 downto 0) := (others => '0');
  signal s_buf_rdata : unsigned(3 downto 0) := (others => '0');
  signal s_bufq, s_bufq2 : unsigned(3 downto 0) := (others => '0');
  signal s_nib       : unsigned(2 downto 0) := (others => '0');

  -- Hysteresis grid (16x16-px tiles, 120x68 at 1080p), ping-pong banks
  constant C_GRID_SIZE : integer := 8192;
  type t_hyst_grid is array(0 to C_GRID_SIZE - 1) of unsigned(3 downto 0);
  signal s_grid_a : t_hyst_grid := (others => "0000");
  signal s_grid_b : t_hyst_grid := (others => "0000");
  signal s_fparity    : std_logic := '0';
  signal s_grid_raddr : unsigned(12 downto 0) := (others => '0');
  signal s_grid_waddr : unsigned(12 downto 0) := (others => '0');
  signal s_grid_we    : std_logic := '0';
  signal s_grid_wdata : unsigned(3 downto 0) := (others => '0');
  signal s_grid_rd_a, s_grid_rd_b : unsigned(3 downto 0) := (others => '0');

  -- Match pipeline
  signal s_trow_d1, s_tcol_d1 : unsigned(6 downto 0) := (others => '0');
  signal s_bay_d1   : unsigned(3 downto 0) := (others => '0');
  signal s_match_y, s_match_u, s_match_v : unsigned(9 downto 0) := (others => '0');
  signal s_dith2, s_dith3, s_dith4 : signed(10 downto 0) := (others => '0');
  signal s_yprod    : unsigned(16 downto 0) := (others => '0');
  signal s_uprod, s_vprod : unsigned(17 downto 0) := (others => '0');
  signal s_ycon, s_ucon4, s_vcon4 : unsigned(9 downto 0) := (others => '0');
  signal s_gprev4, s_gprev5, s_gprev6, s_gprev7, s_gprev8 : unsigned(3 downto 0) := (others => '0');
  signal s_d8       : signed(7 downto 0) := (others => '0');
  signal s_q5, s_l5 : unsigned(3 downto 0) := (others => '0');
  signal s_snap6    : std_logic := '0';
  signal s_ya5, s_ya6, s_yb6 : signed(11 downto 0) := (others => '0');
  signal s_lnew6, s_lnew7, s_lnew8, s_lnew9 : unsigned(3 downto 0) := (others => '0');
  signal s_uc5, s_vc5, s_uc6, s_vc6, s_uc7, s_vc7 : unsigned(9 downto 0) := (others => '0');
  signal s_ym7, s_ym8 : unsigned(9 downto 0) := (others => '0');

  type t_dist_arr is array(0 to 3) of unsigned(6 downto 0);
  signal s_dist     : t_dist_arr := (others => (others => '0'));
  signal s_luma_idx : unsigned(1 downto 0) := (others => '0');

  signal s_chal      : unsigned(1 downto 0) := (others => '0');
  signal s_chal_dist : unsigned(6 downto 0) := (others => '0');
  signal s_prev_dist : unsigned(6 downto 0) := (others => '0');
  signal s_prev9, s_pend9 : unsigned(1 downto 0) := (others => '0');

  -- write-side delay chains (index = stage)
  -- (stage 1 is driven by p_t1, so it gets its own signals: one driver
  -- per signal, or GHDL resolves the arrays to 'X')
  type t_col_arr  is array(2 to 9) of unsigned(10 downto 0);
  type t_gad_arr  is array(3 to 9) of unsigned(12 downto 0);
  signal s_we1   : std_logic := '0';
  signal s_wcol1 : unsigned(10 downto 0) := (others => '0');
  signal s_we    : std_logic_vector(2 to 9) := (others => '0');
  signal s_wcol  : t_col_arr := (others => (others => '0'));
  signal s_gaddr : t_gad_arr := (others => (others => '0'));

  -- Snow hash stages
  signal s_sh1, s_sh2, s_sh3 : unsigned(23 downto 0) := (others => '0');

  -- Display
  signal s_disp_y, s_disp_u, s_disp_v : unsigned(9 downto 0) := (others => '0');
  signal s_t4_su_prod, s_t4_sv_prod : signed(20 downto 0) := (others => '0');
  signal s_t4_y     : unsigned(9 downto 0) := (others => '0');
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

begin

  -- ==========================================================================
  -- Controls: decode, palette entries, per-frame constants, change flags.
  -- ==========================================================================
  p_ctl : process(clk)
    variable v_r0, v_r1, v_r2, v_r7 : unsigned(9 downto 0);
    variable v_mode  : unsigned(2 downto 0);
    variable v_ink4  : unsigned(3 downto 0);
    variable v_fg4   : unsigned(3 downto 0);
    variable v_pal   : unsigned(1 downto 0);
    variable v_i1, v_i2, v_i3 : integer range 0 to 15;
    variable v_luma  : std_logic;
    variable v_c     : unsigned(9 downto 0);
    variable v_cg    : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      v_r0 := unsigned(registers_in(0));
      v_r1 := unsigned(registers_in(1));
      v_r2 := unsigned(registers_in(2));
      v_r7 := unsigned(registers_in(7));

      -- Labelled defaults: at program load the firmware writes the
      -- initial_value_label's INDEX (0..N-1) into the register, not a
      -- 0..1023 position (HW-measured 2026-10-02), so a value below the
      -- label count is taken as the label index itself.
      -- K1 Mode: 5 labels
      if    v_r0 < 5   then v_mode := v_r0(2 downto 0);
      elsif v_r0 < 205 then v_mode := "000";
      elsif v_r0 < 410 then v_mode := "001";
      elsif v_r0 < 615 then v_mode := "010";
      elsif v_r0 < 820 then v_mode := "011";
      else                  v_mode := "100";
      end if;
      s_mode <= v_mode;

      s_stab_on    <= registers_in(6)(0);
      s_intens     <= registers_in(6)(1);
      s_scanlines  <= registers_in(6)(2);
      s_color_mode <= registers_in(6)(3);
      s_snow_on    <= registers_in(6)(4);
      if unsigned(registers_in(5)) < 8 then                  -- K6: 8 labels
        s_ink3     <= unsigned(registers_in(5)(2 downto 0));
      else
        s_ink3     <= unsigned(registers_in(5)(9 downto 7));
      end if;
      s_contrast   <= unsigned(registers_in(3));
      s_color      <= unsigned(registers_in(4));

      -- P12 Dither (deadband at the bottom so pot noise can't flip modes)
      s_damp <= v_r7(9 downto 2);
      if v_r7 >= 32 then s_dith_on <= '1'; else s_dith_on <= '0'; end if;

      -- K2/K3: 5 labels  2x / 4x / CGA / 8x / 16x.  No Off: 1-px cells with
      -- dither on showed noisy colour bars on HDMI (cause not found).
      if    v_r1 < 5   then s_hcode <= v_r1(2 downto 0) + 1;
      elsif v_r1 < 205 then s_hcode <= "001";
      elsif v_r1 < 410 then s_hcode <= "010";
      elsif v_r1 < 615 then s_hcode <= "011";
      elsif v_r1 < 820 then s_hcode <= "100";
      else                  s_hcode <= "101";
      end if;
      if    v_r2 < 5   then s_vcode <= v_r2(2 downto 0) + 1;
      elsif v_r2 < 205 then s_vcode <= "001";
      elsif v_r2 < 410 then s_vcode <= "010";
      elsif v_r2 < 615 then s_vcode <= "011";
      elsif v_r2 < 820 then s_vcode <= "100";
      else                  s_vcode <= "101";
      end if;

      -- Native CGA block: 320 (or 640) columns x 200 rows of the 1080p
      -- raster, in virtual units (6x5 / 3x5), so it keeps its size relative
      -- to the screen height in every mode.
      if s_is_comp = '1' or s_mode = "011" then s_nat_h <= "011";  -- 3
      else                                      s_nat_h <= "110";  -- 6
      end if;

      -- Resolution scaling: per-mode steps (constant within a mode).
      s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
      s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
      s_ys3 <= resize(s_ys, 14) + shift_left(resize(s_ys, 14), 1);
      s_ys5 <= resize(s_ys, 14) + shift_left(resize(s_ys, 14), 2);

      -- H size (virtual units)
      case s_hcode is
        when "000" | "001" => s_h_size <= to_unsigned(2, 5);
        when "010" => s_h_size <= to_unsigned(4, 5);
        when "100" => s_h_size <= to_unsigned(8, 5);
        when "101" => s_h_size <= to_unsigned(16, 5);
        when others => s_h_size <= resize(s_nat_h, 5);
      end case;
      -- Averaged samples per cell: the largest power of 2 that fits in the
      -- narrowest cell of this mode (cells alternate widths when the size
      -- isn't a whole number of pixels): 1080p h px, 720p h/1.5, SD h/2.
      case s_h_size(4 downto 1) is
        when "0001" =>                                   -- 2 / 3
          if s_xs = 2 or (s_xs = 3 and s_h_size(0) = '1') then
            s_h_shift <= "001";
          else
            s_h_shift <= "000";
          end if;
        when "0010" | "0011" =>                          -- 4 / 6
          if s_xs = 2 or (s_xs = 3 and s_h_size(1) = '1') then
            s_h_shift <= "010";
          else
            s_h_shift <= "001";
          end if;
        when "0100" =>                                   -- 8
          if s_xs = 2 then s_h_shift <= "011"; else s_h_shift <= "010"; end if;
        when others =>                                   -- 16
          if s_xs = 2 then s_h_shift <= "100"; else s_h_shift <= "011"; end if;
      end case;
      s_acc_lim <= shift_left(to_unsigned(1, 5), to_integer(s_h_shift));
      s_hth     <= (s_h_size & '0') - s_xs;

      -- Thin-block classes (real size = virtual size / step): Bayer grain
      -- and scanline style follow the block's size in pixels / lines.
      -- Never at 1080p (blocks >= 2 px), so 1080p is unchanged.
      if resize(s_h_size & "00", 8) < resize(s_xs, 8) + shift_left(resize(s_xs, 8), 1) then
        s_hgrain <= '1';
      else
        s_hgrain <= '0';
      end if;
      if (s_v_size & "000000000") < s_ys3 then s_vgrain <= '1'; else s_vgrain <= '0'; end if;
      if (s_v_size & "000000000") < s_ys5 then s_vq25   <= '1'; else s_vq25   <= '0'; end if;

      case s_vcode is
        when "000" => s_v_size <= to_unsigned(1, 5);
        when "001" => s_v_size <= to_unsigned(2, 5);
        when "010" => s_v_size <= to_unsigned(4, 5);
        when "100" => s_v_size <= to_unsigned(8, 5);
        when "101" => s_v_size <= to_unsigned(16, 5);
        when others => s_v_size <= to_unsigned(5, 5);
      end case;

      -- Mode flags
      if s_mode = "100" then s_is_comp <= '1'; else s_is_comp <= '0'; end if;
      if s_mode(2) = '1' or s_mode = "011" then s_zone2 <= '1'; else s_zone2 <= '0'; end if;
      if s_mode = "100" or s_color_mode = '1' then v_luma := '1'; else v_luma := '0'; end if;
      s_luma_on <= v_luma;
      s_pal2    <= s_mode(1 downto 0);

      -- Palette entries.  Ink = RGBI colour with the intensity switch as I.
      -- Black ink stays black at High intensity (RGBI 8 = dark grey read as
      -- a washed-out black on hardware); the other inks take the I bit.
      if s_ink3 = "000" then v_ink4 := "0000";
      else                   v_ink4 := s_intens & s_ink3;
      end if;
      if s_ink3 = "000" then v_fg4 := s_intens & "111";  -- mono: Black ink -> gray/white
      else                   v_fg4 := v_ink4;
      end if;
      v_pal := s_mode(1 downto 0);
      if s_intens = '0' then
        case v_pal is
          when "00"   => v_i1 := 4;  v_i2 := 2;  v_i3 := 6;    -- red green brown
          when "01"   => v_i1 := 3;  v_i2 := 5;  v_i3 := 7;    -- cyan magenta gray
          when others => v_i1 := 3;  v_i2 := 4;  v_i3 := 7;    -- cyan red gray
        end case;
      else
        case v_pal is
          when "00"   => v_i1 := 12; v_i2 := 10; v_i3 := 14;
          when "01"   => v_i1 := 11; v_i2 := 13; v_i3 := 15;
          when others => v_i1 := 11; v_i2 := 12; v_i3 := 15;
        end case;
      end if;
      if s_mode(2) = '1' or s_mode = "011" then
        -- mono / composite: {black, fg, black, fg}
        s_pe(0) <= C_RGBI(0);
        s_pe(1) <= C_RGBI(to_integer(v_fg4));
        s_pe(2) <= C_RGBI(0);
        s_pe(3) <= C_RGBI(to_integer(v_fg4));
      else
        s_pe(0) <= C_RGBI(to_integer(v_ink4));
        s_pe(1) <= C_RGBI(v_i1);
        s_pe(2) <= C_RGBI(v_i2);
        s_pe(3) <= C_RGBI(v_i3);
      end if;
      s_bg <= C_RGBI(to_integer(v_ink4));

      -- Contrast pivots at mid-grey 502; 50% = unity.  ycon =
      -- (y*c7 + 502*(64-c7)) >> 6, bias built one add per stage.
      s_c7      <= s_contrast(9 downto 3);
      s_c64     <= to_signed(64, 8) - signed('0' & s_c7);
      s_cbias_a <= shift_left(resize(s_c64, 18), 9) - shift_left(resize(s_c64, 18), 3);
      s_cbias   <= s_cbias_a - shift_left(resize(s_c64, 18), 1);

      -- Color (K5).  Nearest decisions: input chroma gain before matching,
      -- 0..1x below 50%, 1..5x above.  Luma decisions: palette saturation.
      v_c := s_color;
      if v_c(9) = '0' then v_cg := "000" & v_c(8 downto 4);
      else                 v_cg := to_unsigned(32, 8) + ('0' & v_c(8 downto 2));
      end if;
      if s_luma_on = '1' then
        -- Palette saturation 0..1x: the CGA colours already sit on the RGB
        -- gamut edge, so anything above 1x only pushed chroma onto the
        -- rails (light cyan Cr -> code 0) = glitch bars on hardware.
        s_cg <= to_unsigned(32, 8);
        if s_color >= 1016 then s_osat <= to_unsigned(128, 8);
        else                    s_osat <= "0" & s_color(9 downto 3);
        end if;
      else
        s_cg   <= v_cg;
        s_osat <= to_unsigned(128, 8);
      end if;
      s_qbias <= shift_left(to_signed(32, 19) - signed(resize(s_cg, 19)), 9);

      -- Decision-side signature: any change re-seeds the grids.
      s_sig <= std_logic_vector(s_mode) & s_intens & std_logic_vector(s_ink3)
               & std_logic_vector(s_hcode) & std_logic_vector(s_vcode)
               & s_color_mode & s_dith_on & s_stab_on;
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
        -- Virtual x within the cell (half units): this pixel ends its cell
        -- when the next one starts at or past the cell size.  At 1080p the
        -- step is 1 unit and this is the old "cell_x = size - 1" test.
        if s_cell_xh >= s_hth then              -- xh + step >= 2 * size
          s_cell_x    <= (others => '0');
          s_cell_xh   <= s_cell_xh - s_hth;
          v_at_cell_end := true;
          s_cell_col  <= s_cell_col + 1;
          s_cell_orgv <= s_cell_orgv + s_h_size;
        else
          s_cell_x    <= s_cell_x + 1;
          s_cell_xh   <= s_cell_xh + s_xs;
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
        -- Active lines only advance the row phase (interlace-safe), by one
        -- virtual y step; the cell-row wrap follows in the blanking below.
        if s_line_act = '1' then
          s_act_row <= s_act_row + 1;
          s_vy      <= s_vy + s_ys;
          s_vyl     <= s_vyl + s_ys;
          s_row_new <= '0';
        end if;
        s_line_act <= '0';
      elsif s_line_act = '0' and data_in.avid = '0'
            and s_vyl >= (s_v_size & "00000000") then
        -- Cell-row wrap, one row per blanking clock: a field line can
        -- cross several block rows at SD (2x is under half a field line).
        -- At 1080p this is the old "cell_y = size - 1" wrap, one clock
        -- later and still in the blanking.
        s_vyl      <= s_vyl - (s_v_size & "00000000");
        s_cell_row <= s_cell_row + 1;
        s_row_new  <= '1';
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_cell_x    <= (others => '0');
        s_cell_xh   <= (others => '0');
        s_cell_orgv <= (others => '0');
        s_cell_col  <= (others => '0');
        s_cell_row  <= (others => '0');
        s_act_row   <= (others => '0');
        s_line_act  <= '0';
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
        -- Once per field, on the first vsync edge after active video
        -- (analog vsync is serrated): bank swap, field height, boot and
        -- re-seed.  Change flags are free-running 1-bit registers.
        if s_saw_act = '1' then
          s_fparity <= not s_fparity;
          s_saw_act <= '0';
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

      if v_at_cell_end and s_row_new = '1' and data_in.avid = '1' then
        s_use_avg <= '1';
        s_we1     <= '1';
        s_wcol1   <= s_cell_col;
        -- 16x16 stability tiles in virtual units (same cells per tile in
        -- every mode; at 1080p the old pixel / active-line tiles)
        s_trow_d1 <= s_vy(18 downto 12);
        s_tcol_d1 <= s_cell_orgv(10 downto 4);
        -- Dither grain never finer than ~2 px: a 1x1-pixel checkerboard
        -- renders as thick glitch bars on hardware, like other programs'
        -- 1-2 px high-contrast patterns.  (Blocks under 1.5 px / lines:
        -- 2x at 720p and SD, never at 1080p.)
        if s_hgrain = '1' then v_bc := s_cell_col(2 downto 1);
        else                   v_bc := s_cell_col(1 downto 0);
        end if;
        if s_vgrain = '1' then v_br := s_cell_row(2 downto 1);
        else                 v_br := s_cell_row(1 downto 0);
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
    variable v_z     : unsigned(1 downto 0);
    variable v_min   : unsigned(6 downto 0);
    variable v_chal  : unsigned(1 downto 0);
    variable v_pz    : integer range 0 to 5;
    variable v_lo, v_hi, v_dy : unsigned(9 downto 0);
    variable v_idx, v_cur, v_pend : unsigned(1 downto 0);
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
      s_grid_raddr <= resize(
          shift_left(resize(s_trow_d1, 14), 7)
        - shift_left(resize(s_trow_d1, 14), 3)
        + resize(s_tcol_d1, 14), 13);
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

      -- ---- T2d: distance to the tile's stored level.  The grid keeps
      -- luma bits 7:4 (16-code steps, modulo 256); the live pixel resolves
      -- the wrap, since noise is a few codes, not 128.  (v2.0 stored bits
      -- 9:6 -- 64-code steps posterised gradients into flat bars.)
      s_d8  <= signed(s_ycon(7 downto 0) - (s_gprev4 & "1000"));
      s_q5  <= s_ycon(7 downto 4);
      s_l5  <= s_gprev4;
      s_ya5 <= signed(resize(s_ycon, 12)) + resize(s_dith4, 12);
      s_uc5 <= s_ucon4;
      s_vc5 <= s_vcon4;
      s_gprev5 <= s_gprev4;

      -- ---- T2e: snap within +-24 (stored centre is within 8 of the value
      -- it was quantised from, leaving 16 codes of noise margin)
      if s_vsnap = '1' and s_d8 >= -24 and s_d8 <= 24 then
        s_snap6 <= '1';
        s_lnew6 <= s_l5;
      else
        s_snap6 <= '0';
        s_lnew6 <= s_q5;
      end if;
      s_ya6 <= s_ya5;
      s_yb6 <= s_ya5 - resize(s_d8, 12);
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

      -- ---- T2g: four palette distances + luma-zone pick
      for i in 0 to 3 loop
        s_dist(i) <= f_dist(s_ym7, s_uc7, s_vc7, s_pe(i).y, s_pe(i).u, s_pe(i).v);
      end loop;
      if s_zone2 = '1' then
        if s_ym7 >= 502 then s_luma_idx <= "01"; else s_luma_idx <= "00"; end if;
      else
        if    s_ym7 < 283 then v_z := "00";
        elsif s_ym7 < 502 then v_z := "01";
        elsif s_ym7 < 721 then v_z := "10";
        else                   v_z := "11";
        end if;
        s_luma_idx <= f_luma_map(s_pal2, v_z);
      end if;
      s_ym8    <= s_ym7;
      s_lnew8  <= s_lnew7;
      s_gprev8 <= s_gprev7;

      -- ---- T3a: challenger + incumbent distance
      v_min := s_dist(0); v_chal := "00";
      if s_dist(1) < v_min then v_min := s_dist(1); v_chal := "01"; end if;
      if s_dist(2) < v_min then v_min := s_dist(2); v_chal := "10"; end if;
      if s_dist(3) < v_min then v_min := s_dist(3); v_chal := "11"; end if;
      if s_luma_on = '1' then
        -- Luma decisions: incumbent distance = how far y sits outside the
        -- stored entry's zone, in the same 32-code units.
        if s_zone2 = '1' then
          v_pz := 4 + to_integer(s_gprev8(0 downto 0));
        else
          v_pz := to_integer(f_luma_map(s_pal2, s_gprev8(1 downto 0)));
        end if;
        v_lo := C_ZLO(v_pz);
        v_hi := C_ZHI(v_pz);
        if s_ym8 < v_lo then
          v_dy := v_lo - s_ym8;
        elsif s_ym8 > v_hi then
          v_dy := s_ym8 - v_hi;
        else
          v_dy := (others => '0');
        end if;
        s_chal      <= s_luma_idx;
        s_chal_dist <= (others => '0');
        s_prev_dist <= resize(v_dy(9 downto 5), 7);
      else
        s_chal      <= v_chal;
        s_chal_dist <= v_min;
        s_prev_dist <= s_dist(to_integer(s_gprev8(1 downto 0)));
      end if;
      s_prev9 <= s_gprev8(1 downto 0);
      s_pend9 <= s_gprev8(3 downto 2);
      s_lnew9 <= s_lnew8;

      -- ---- T3b: decision (margin 2 + debounce), buffer + grid writes
      if s_dhyst = '0' then
        v_idx := s_chal; v_cur := s_chal; v_pend := s_chal;
      elsif resize(s_chal_dist, 8) + 6 < resize(s_prev_dist, 8) then
        v_idx := s_chal; v_cur := s_chal; v_pend := s_chal;
      elsif resize(s_chal_dist, 8) + 2 < resize(s_prev_dist, 8) then
        if s_chal = s_pend9 then
          v_idx := s_chal;  v_cur := s_chal;  v_pend := s_chal;
        else
          v_idx := s_prev9; v_cur := s_prev9; v_pend := s_chal;
        end if;
      else
        v_idx := s_prev9; v_cur := s_prev9; v_pend := s_prev9;
      end if;

      if s_is_comp = '1' then
        -- 4 consecutive dots -> one nibble, written at the group's last dot
        if s_we(9) = '1' then
          s_nib <= s_nib(1 downto 0) & v_idx(0);
        end if;
        if s_we(9) = '1' and s_wcol(9)(1 downto 0) = "11" then
          s_buf_we <= '1';
        else
          s_buf_we <= '0';
        end if;
        s_buf_waddr <= "00" & s_wcol(9)(10 downto 2);
        s_buf_wdata <= s_nib & v_idx(0);
      else
        s_buf_we    <= s_we(9);
        s_buf_waddr <= s_wcol(9);
        s_buf_wdata <= "00" & v_idx;
      end if;

      s_grid_we    <= s_we(9);
      s_grid_waddr <= s_gaddr(9);
      if s_dith_on = '1' then
        s_grid_wdata <= s_lnew9;
      else
        s_grid_wdata <= v_pend & v_cur;
      end if;

      -- write-side delay chains
      s_we(2)   <= s_we1;
      s_wcol(2) <= s_wcol1;
      for i in 3 to 9 loop
        s_we(i)   <= s_we(i - 1);
        s_wcol(i) <= s_wcol(i - 1);
      end loop;
      s_gaddr(3) <= s_grid_raddr;
      for i in 4 to 9 loop
        s_gaddr(i) <= s_gaddr(i - 1);
      end loop;
    end if;
  end process p_match;

  -- ==========================================================================
  -- Per-column BRAM (1W1R)
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

  s_buf_raddr <= "00" & s_cell_col(10 downto 2) when s_is_comp = '1'
                 else s_cell_col;

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
  -- Display path: buffer read -> fabric registers (+ snow hash) -> palette
  -- lookup -> saturation -> scanlines / blanking gate -> output.
  -- ==========================================================================
  p_disp : process(clk)
    variable v_g    : unsigned(8 downto 0);
    variable v_k    : unsigned(23 downto 0);
    variable v_n    : unsigned(3 downto 0);
    variable v_e    : t_pal_entry;
    variable v_su, v_sv : signed(11 downto 0);
    variable v_sat  : signed(8 downto 0);
  begin
    if rising_edge(clk) then
      s_bufq  <= s_buf_rdata;
      s_bufq2 <= s_bufq;

      -- CGA snow: a hashed 1-in-64 of the character-wide groups (4 cells;
      -- 8 dots in the 640 modes) flash a random colour each field.
      if s_zone2 = '1' then v_g := '0' & s_cell_col(10 downto 3);
      else                  v_g := s_cell_col(10 downto 2);
      end if;
      v_k := s_fcount & s_cell_row & v_g;
      s_sh1 <= v_k + shift_left(v_k, 7);
      s_sh2 <= (s_sh1 xor rotate_left(s_sh1, 13)) + to_unsigned(16#5A3C95#, 24);
      s_sh3 <= s_sh2 + shift_left(s_sh2, 3);

      -- T4a: palette lookup (with snow override)
      v_n := s_bufq2;
      if s_snow_on = '1' and s_sh3(23 downto 18) = "000000" then
        v_n := s_sh3(11 downto 8);
        if s_is_comp = '0' and v_n(1 downto 0) = "00" then
          v_n(1 downto 0) := "11";
        end if;
      end if;
      if s_is_comp = '1' then
        if s_color_mode = '1' then
          v_n := v_n(1 downto 0) & v_n(3 downto 2);   -- burst phase flip
        end if;
        if v_n = "0000" then v_e := s_bg;
        else                 v_e := C_COMPOSITE(to_integer(v_n));
        end if;
      else
        v_e := s_pe(to_integer(v_n(1 downto 0)));
      end if;
      s_disp_y <= v_e.y;
      s_disp_u <= v_e.u;
      s_disp_v <= v_e.v;

      -- T4b: saturation multiply (128 = unity)
      v_su  := signed(resize(s_disp_u, 12)) - to_signed(512, 12);
      v_sv  := signed(resize(s_disp_v, 12)) - to_signed(512, 12);
      v_sat := signed('0' & s_osat);
      s_t4_su_prod <= v_su * v_sat;
      s_t4_sv_prod <= v_sv * v_sat;
      s_t4_y       <= s_disp_y;
    end if;
  end process p_disp;

  p_t5 : process(clk)
    variable v_su_out, v_sv_out : signed(11 downto 0);
    variable v_yd : unsigned(9 downto 0);
    variable v_u, v_v : unsigned(9 downto 0);
    variable v_cu, v_cv : signed(10 downto 0);
    variable v_dim : boolean;
  begin
    if rising_edge(clk) then
      v_su_out := s_t4_su_prod(18 downto 7) + to_signed(512, 12);
      v_sv_out := s_t4_sv_prod(18 downto 7) + to_signed(512, 12);
      -- Legal chroma range only (64..960): rail codes overdrive the encoders.
      if    v_su_out < 64  then v_u := to_unsigned(64, 10);
      elsif v_su_out > 960 then v_u := to_unsigned(960, 10);
      else                      v_u := unsigned(v_su_out(9 downto 0));
      end if;
      if    v_sv_out < 64  then v_v := to_unsigned(64, 10);
      elsif v_sv_out > 960 then v_v := to_unsigned(960, 10);
      else                      v_v := unsigned(v_sv_out(9 downto 0));
      end if;

      -- Scanlines on the CGA line grid: the bottom line of every block row
      -- (a row's first line is the displayed block's last line -- see
      -- header), 50% for tall blocks, 25% for rows under 2.5 lines, and
      -- 25% on alternate lines where rows are under 1.5 lines (the 1-line
      -- dim is pixel scale, so short SD rows get the light style instead
      -- of dark gaps).  Dimming pivots on black (64) / neutral chroma (512).
      if s_vgrain = '1' then
        v_dim := s_act_row(0) = '1';
      else
        v_dim := s_row_new = '1';
      end if;
      v_yd := s_t4_y - 64;
      v_cu := signed(resize(v_u, 11)) - to_signed(512, 11);
      v_cv := signed(resize(v_v, 11)) - to_signed(512, 11);
      if s_scanlines = '1' and v_dim then
        if s_vq25 = '1' then
          v_yd := v_yd - (v_yd srl 2);
          v_cu := v_cu - shift_right(v_cu, 2);
          v_cv := v_cv - shift_right(v_cv, 2);
        else
          v_yd := v_yd srl 1;
          v_cu := shift_right(v_cu, 1);
          v_cv := shift_right(v_cv, 1);
        end if;
      end if;

      -- Blanking gate (latency-aligned avid) + blank the field's first
      -- line, which shows the previous field's bottom row.
      if s_avid_sr(C_LATENCY - 2) = '0' or s_act_row = 0 then
        s_out_y <= to_unsigned(64, 10);
        s_out_u <= to_unsigned(512, 10);
        s_out_v <= to_unsigned(512, 10);
      else
        v_cu := v_cu + to_signed(512, 11);
        v_cv := v_cv + to_signed(512, 11);
        s_out_y <= v_yd + 64;
        s_out_u <= unsigned(v_cu(9 downto 0));
        s_out_v <= unsigned(v_cv(9 downto 0));
      end if;
    end if;
  end process p_t5;

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

end architecture cga;
