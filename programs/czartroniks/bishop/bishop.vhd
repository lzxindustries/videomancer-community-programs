-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Bishop v2 — line-buffer wireframe head.
--
-- v1.0.3 (2026-10-17): resolution scaling.  The head is sized in virtual
-- 1080-line units: at 720p and SD it is the 1080p head scaled to the screen
-- height (SD shows a 1440-unit-wide 4:3 window, so the head keeps its shape on
-- a 4:3 screen).  The per-mode factor is folded into the copy engine's P12
-- scale product (one signed 9-bit factor per axis, about HEAD_Y_MID, plus a
-- per-mode row offset and a half-row bias on the bottom field of interlaced
-- SD), so the raster still works in real pixels and field lines.  P12 has
-- the 1080p range in every mode; K6 pan, the stroke half-widths (per axis,
-- rounded, never under 1 px), the noise span floor and the head clear window
-- scale with it.  SD also wipes 8 entries per line instead of 32, and a
-- raster pass that is still stamping when the next hsync arrives now aborts
-- its remaining stamps instead of missing that hsync (a missed hsync skipped
-- a row's DDA step and kinked every edge crossing it).  At 1080p every
-- factor is the identity and the picture is bit-identical to v1.0.2.
--
-- Architecture:
--   v1 instantiated one DDA + per-pixel hit-tester per edge, in parallel.
--   At ~80 LCs/edge × 50 edges that filled most of HX4K just on edge logic.
--
--   v2 uses a SINGLE shared rasterizer that walks edges sequentially and
--   writes stamps into a 1-bit/pixel dual-bank line buffer (inlined here
--   so we can gate writes — the shared video_line_buffer entity writes
--   every cycle, which kept overwriting fresh stamps with zeros).
--
-- Per-scanline rasterizer timeline (running during the line BEFORE the
-- one being prepared):
--   R_CLEAR  2*hw_r cycles           write 0 across head's x bbox of next-line bank
--                                    (edge update fires once per cycle for first
--                                     N_EDGES cycles, in parallel with clears)
--   R_WIPE   32 cycles                 rolling clear of the rest of the buffer
--                                    (per-bank pointer: each bank fully
--                                     wiped every 128 lines)
--   R_STAMP  5 + (|slope| + 2*ext + 1) cycles per ACTIVE edge
--                                    write 1 across the edge's row sweep,
--                                    clamped to its own [xlo, xhi]
--   R_IDLE                           wait for next hsync (writes gated off)
--
-- Worst-case HD budget (256 edges, K2 = 7 px, P12 = full size):
--   CLEAR 696 + WIPE 32 + STAMP ~1220  ≈ 1950 of the 2200 cycles in an HD line.
-- (v1.0.3) SD (858 cycles/line) now draws the head scaled to 243 field
-- lines; R_CLEAR's 268-cycle edge walk plus ~7 cycles per active edge is a
-- larger share of the short line, so the heaviest settings (eyes or mouth
-- closed with thick lines) can still overrun a few rows -- they now lose
-- stamps on those rows instead of a whole row.
--
-- Register map:
--   registers_in(0) = K1 Mouth      (analog close amount)
--   registers_in(1) = K2 Thickness  (top 2 bits → PAD ∈ {0,1,2,3} = 1/3/5/7 px)
--   registers_in(2) = K3 Palette    (top 3 bits = palette index)
--   registers_in(3) = K4 Expression (blend across Neutral..Surprised)
--   registers_in(4) = K5 Noise       (top 5 bits = spread; speed rises with it)
--   registers_in(5) = K6 Position    (bipolar horizontal pan, +-511 px at 1080)
--   registers_in(8) = video timing ID (resolution scaling, v1.0.3)
--   registers_in(6) = Switches      (bit0 = T7 Fill, bit1 = T8 Eyes open,
--                                    bit2 = T9 noise streaks, bit3 = T10 Invert,
--                                    bit4 = T11 BG-Video)
--   registers_in(7) = P12 Head Size (0 = mesh size, 1023 = ~95% of screen height)
--
-- License: GPL-3.0
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.bishop_mesh_pkg.all;
use work.all;

architecture bishop of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (5 + 3 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    -- ---------------------------------------------------------------
    -- Geometry / rasterizer parameters
    -- ---------------------------------------------------------------
    -- Base half-width of the head's x bounding box at scale 1 (max|x| in the
    -- mesh is ~253 px).  The CLEAR phase and the stamp clamp both use the
    -- per-frame `hw_r`, which is this value grown by the P12 size factor.
    constant HEAD_HW_BASE : natural := 258;
    -- Absolute ceiling on hw_r (used to size counters / the clamp range).
    -- 258 * (1 + 127/256) = 386.
    constant HEAD_HW_MAX  : natural := 386;
    -- Mesh y-coordinate of the head's vertical centre ((ymin+ymax)/2 over the
    -- whole mesh = (-362+415)/2).  P12 scaling is applied about this row so
    -- the head grows symmetrically instead of dropping off the bottom.
    constant HEAD_Y_MID   : integer := 26;
    -- Cap stamps per edge so a single near-horizontal high-slope edge
    -- can't blow the per-scanline cycle budget.  Edges needing more than
    -- this render only the first portion of their sweep.
    constant MAX_STAMP_M1 : natural := 191;
    -- Cap on the per-side thickness extension so a morph/noise-flattened
    -- edge can't demand a 700-px sweep.
    constant MAX_EXT      : natural := 64;

    -- Inlined line buffer is 1 BRAM cycle + 1 output flop = 2 cycles.
    -- Plus palette-mul flop = 3.  Match the bypass pipe accordingly.
    -- 3 stages of line-buffer/eor alignment + 1 for the chroma-pair
    -- lookahead + 1 for the registered output stage (see out_proc).
    constant LATENCY : natural := 5;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLACK      : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- Luma is stored PRE-SCALED (the old per-pixel `pal_y * bright` multiply
    -- is gone — P12 is the head-size slider now).  Values are the previous
    -- palette at the old default fader (x0.879), capped at 768: full-scale Y
    -- clips RGB and kills chroma on hardware.  T10 "Bright" picks these;
    -- "Normal" takes 3/4 of them (a shift-subtract at vsync).
    type t_pal_lut is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal_lut := (
        to_unsigned( 633, 10), to_unsigned( 668, 10),
        to_unsigned( 765, 10), to_unsigned( 281, 10),
        to_unsigned( 768, 10), to_unsigned( 475, 10),
        to_unsigned( 768, 10), to_unsigned( 246, 10));
    constant C_PAL_U : t_pal_lut := (
        to_unsigned( 220, 10), to_unsigned( 260, 10),
        to_unsigned( 720, 10), to_unsigned( 400, 10),
        to_unsigned( 512, 10), to_unsigned( 800, 10),
        to_unsigned( 130, 10), to_unsigned( 920, 10));
    constant C_PAL_V : t_pal_lut := (
        to_unsigned( 220, 10), to_unsigned( 720, 10),
        to_unsigned( 220, 10), to_unsigned( 880, 10),
        to_unsigned( 512, 10), to_unsigned( 880, 10),
        to_unsigned( 580, 10), to_unsigned( 380, 10));

    -- ---------------------------------------------------------------
    -- Frame / line timing
    -- ---------------------------------------------------------------
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal hsync_falling_r  : std_logic := '0';
    signal vsync_falling_r  : std_logic := '0';
    -- Saw-active guard: analog vsync serrates (several falling edges per
    -- field).  Only the first edge after a field that carried avid counts;
    -- the later ones would re-zero pixel_y (a one-line jitter), re-latch
    -- max_y_r from a counter that has just been reset (v_centre -> 0, head
    -- thrown off the top of the screen) and step the noise phase again.
    signal saw_active_r     : std_logic := '0';
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    -- ACTIVE line count of the previous frame (lines that carried avid), as
    -- opposed to max_y_r's total-including-blanking count.  P12's size ceiling
    -- is a fraction of the picture, so it has to key off the picture.
    signal avid_line_r  : std_logic := '0';
    signal aline_cnt_r  : unsigned(11 downto 0) := (others => '0');
    signal act_h_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal h_centre_r   : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal v_centre_r   : unsigned(11 downto 0) := to_unsigned(540, 12);
    -- Registered scanline target row (pixel_y - v_centre + 1).  Pipelined
    -- out of the Stage-1 active-update compares: pixel_y/v_centre_r are
    -- stable across the whole R_CLEAR edge walk, so the 1-cycle delay is
    -- transparent, and it removes the 13-bit subtract from the long
    -- subtract-then-compare carry chain that limited HD HDMI timing.
    signal v_y_target_r : signed(12 downto 0) := (others => '0');

    -- ---------------------------------------------------------------
    -- Per-frame latched user controls (all updated on vsync_falling_r)
    -- ---------------------------------------------------------------
    signal pal_y_r    : unsigned(9 downto 0) := C_PAL_Y(0);
    signal pal_yf_r   : unsigned(9 downto 0) := C_PAL_Y(0);  -- 3/4 luma: the Invert field
    signal pal_u_r    : unsigned(9 downto 0) := C_PAL_U(0);
    signal pal_v_r    : unsigned(9 downto 0) := C_PAL_V(0);
    signal bg_video_r : std_logic := '0';
    signal invert_r   : std_logic := '0';   -- T10: black lines on a palette-colour field
    -- K6 Position: bipolar horizontal pan in pixels, applied to the measured
    -- line centre once per cycle (h_orig_r) so the hsync cone stays light.
    signal pan_r      : signed(9 downto 0)    := (others => '0');
    signal h_orig_r   : unsigned(11 downto 0) := to_unsigned(960, 12);
    -- K2 Thickness — half-width of the stroke in px.  Rendered perpendicular
    -- thickness is 2*pad+1 (1 / 3 / 5 / 7 px) for EVERY edge orientation.
    -- (v1.0.3) One half-width per axis: padx_r in pixels (across), pady_r in
    -- field lines (down).  Both equal the K2 index at 1080p; other modes take
    -- the 1080p stroke scaled to the mode and rounded to the nearest odd
    -- width (min 1 px) -- see padx_pre_r / pady_pre_r.
    signal padx_r     : unsigned(1 downto 0) := "01";
    signal pady_r     : unsigned(1 downto 0) := "01";
    signal padx_pre_r : unsigned(1 downto 0) := "01";
    signal pady_pre_r : unsigned(1 downto 0) := "01";
    -- Free-running label decodes for K2/K3 (kept out of the vsync cone).
    -- Labelled defaults: at program load the firmware writes the
    -- initial_value_label's INDEX (0..N-1) into the register, not a 0..1023
    -- position (HW-measured 2026-10-02), so a value below the label count is
    -- taken as the label index itself.
    signal k2_idx_r   : unsigned(1 downto 0) := "01";
    signal k3_idx_r   : unsigned(2 downto 0) := (others => '0');
    -- S7 / S8 / T10 / T11.
    signal fill_on_r    : std_logic := '0';   -- T7: solid-fill eyes/mouth
    signal open_eyes_r  : std_logic := '0';
    -- ---------------------------------------------------------------
    -- P12 HEAD SIZE.  The whole mesh is scaled by (1 + m/256) about
    -- HEAD_Y_MID, applied in the vblank copy engine so the per-pixel path is
    -- untouched.  m spans 0..m_max, where m_max is derived from the RUNTIME
    -- measured frame height so full slider = ~95% of screen height:
    --   m_max = H/4 + H/16 + H/512 - 256, clamped to [0,127]
    -- (H is the ACTIVE line count; the +H/512 term compensates the knob's
    -- 63/64 top step).  At 1080 that is 83, so full slider gives m = 81 ->
    -- 1.316x -> the 777-row mesh becomes 1023 rows = 94.7% of the picture.
    -- Modes too short to hold the mesh at 1x (SD/PAL/720p) get m_max = 0.
    -- ---------------------------------------------------------------
    signal size_k_r   : unsigned(5 downto 0) := (others => '0');  -- P12 top 6 bits
    signal size_mmax_r: unsigned(6 downto 0) := (others => '0');
    signal size_m_r   : unsigned(6 downto 0) := (others => '0');  -- 0..127
    -- Per-frame head half-width: HEAD_HW_BASE grown by the size factor.  Used
    -- for BOTH the CLEAR window and the stamp clamp, so no stamp can ever land
    -- outside the region that gets cleared.  It DECAYS rather than snapping
    -- down, so shrinking the head can't strand set pixels outside the new
    -- (narrower) clear window.
    signal hw_r       : unsigned(9 downto 0) := to_unsigned(HEAD_HW_BASE, 10);
    signal hw_tgt_r   : unsigned(9 downto 0) := to_unsigned(HEAD_HW_BASE, 10);
    signal clear_m1_r : unsigned(10 downto 0)
                      := to_unsigned(2 * HEAD_HW_BASE - 1, 11);
    -- +-hw_r pre-formed as signed clamp limits for CP_XEXT2.
    signal hw_lo_r    : signed(12 downto 0) := to_signed(-HEAD_HW_BASE, 13);
    signal hw_hi_r    : signed(12 downto 0) := to_signed(HEAD_HW_BASE - 1, 13);
    signal sz_step    : integer range 0 to 14 := 14;

    -- ---------------------------------------------------------------
    -- RESOLUTION SCALING (v1.0.3).  Mode class from the video timing ID:
    --   0 1080p  1 1080i  2 720p  3 480i  4 480p  5 576i  6 576p
    -- Each mode shows the 1080p head at the same fraction of the screen
    -- height (virtual 1080-line units; SD x steps 2.0, i.e. a 1440-wide
    -- 4:3 window).  The copy engine already scales the mesh by
    -- (1 + m/256) for P12; that factor becomes k*(1 + m/256) per axis, with
    -- k = 1 at 1080p (bit-identical), so the raster keeps working in real
    -- pixels / field lines.  sc_mx_r / sc_my_r are 256*(S - 1) for the two
    -- axes (signed, -199..127), formed once per frame in size_proc by a
    -- serial shift-add multiply of (256 + m) by the mode's k in Q8
    -- (md_kxq_r / md_kyq_r).
    -- ---------------------------------------------------------------
    signal md_cls_r   : unsigned(2 downto 0) := "000";
    signal md_ident_r : std_logic := '1';   -- 1080p: every factor is 1
    signal md_sd_r    : std_logic := '0';   -- SD: short wipe
    signal md_kxq_r   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal md_kyq_r   : unsigned(8 downto 0) := to_unsigned(256, 9);
    -- y offset in Q8 rows, added inside the y scale product's floor:
    -- 256*HEAD_Y_MID*(ky - 1) (the head scales about HEAD_Y_MID, but its
    -- centre must land at HEAD_Y_MID*ky below the screen centre) + 128 (round).
    -- The bottom field of an interlaced mode sits half a field line lower
    -- (md_yoffb_r = md_yoff_r - 128).  Both 0 at 1080p.
    signal md_yoff_r  : signed(13 downto 0) := (others => '0');
    signal md_yoffb_r : signed(13 downto 0) := (others => '0');
    signal yoff_r     : signed(13 downto 0) := (others => '0');
    signal field_bot_r: std_logic := '0';
    -- NOISE_SPAN_FLOOR (8 rows at 1080) in this mode's rows.
    signal md_nfl_r   : unsigned(3 downto 0) := to_unsigned(8, 4);
    -- K6 pan scaled to the mode (free-running), latched into pan_r at vsync.
    signal pan_pre_r  : signed(9 downto 0) := (others => '0');
    -- size_proc's serial multiply: (256 + m) * kq, MSB-first.
    signal sz_a_r     : unsigned(9 downto 0) := (others => '0');
    signal sz_kx_r    : unsigned(8 downto 0) := (others => '0');
    signal sz_ky_r    : unsigned(8 downto 0) := (others => '0');
    signal sz_accx_r  : unsigned(17 downto 0) := (others => '0');
    signal sz_accy_r  : unsigned(17 downto 0) := (others => '0');
    signal sz_cls_r   : unsigned(2 downto 0) := "000";  -- class hw_r was formed for
    signal sc_mx_r    : signed(8 downto 0) := (others => '0');
    signal sc_my_r    : signed(8 downto 0) := (others => '0');
    -- Raster overrun guard: an hsync that arrives while the pass is still
    -- stamping is remembered here and ends the pass (see R_STAMP_SCAN).
    signal hs_pend_r  : std_logic := '0';

    -- R_CLEAR length: the clear window 2*hw, but never shorter than the
    -- edge walk (C_NUM_EDGES issues + the 5-stage update pipe; same N + 12
    -- rule as build_svg_mesh.py).  Only small (SD) heads reach the floor.
    function f_clear_m1(h : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(10 downto 0);
    begin
        v := shift_left(resize(h, 11), 1) - to_unsigned(1, 11);
        if v < to_unsigned(C_NUM_EDGES + 11, 11) then
            v := to_unsigned(C_NUM_EDGES + 11, 11);
        end if;
        return v;
    end function;

    -- ---------------------------------------------------------------
    -- Mutable edge state.  current_x lives in a small BRAM (1R1W,
    -- 1-cycle read latency) so we don't pay LCs + wide muxes for an
    -- N_EDGES register file.
    --
    -- There is NO per-edge `active` flag any more.  R_CLEAR visits every
    -- edge on every scanline, so "active" is exactly
    -- (ymin_eff <= y_target <= ymax_eff) and can be derived from the mesh
    -- BRAM read that Stage 0b already performs.  Dropping the 256-bit
    -- vector removes a 256:1 read mux, a 256-wide write decoder and 256
    -- flops from the render path.
    -- ---------------------------------------------------------------
    -- Q11.7 fixed-point: bits 17..7 are the integer pixel position
    -- (signed, ±1024 range — covers the head at full P12 size), bits 6..0 are 1/128-px
    -- fractional.  Slope rounding error is ~1/256 per row, so a 50-row
    -- edge drifts ~0.2 px — small enough that edges visibly meet at
    -- shared vertices.  Same 16-bit storage as Q12.4 so current_x_ram
    -- still fits a single EBR.
    type t_cur_x_ram is array (0 to 255) of std_logic_vector(17 downto 0);
    signal current_x_ram : t_cur_x_ram := (others => (others => '0'));
    signal cx_rd_addr  : unsigned(7 downto 0) := (others => '0');
    signal cx_rd_data  : std_logic_vector(17 downto 0);
    -- SECOND, fabric copy of the current_x read.  yosys absorbs cx_rd_data's
    -- register into the EBR, so every consumer starts 2.15 ns of clk-to-q plus
    -- column routing behind — and the DDA's 18-bit accumulate hung straight
    -- off it.  cx_rd_data_f costs one more pipeline stage in the R_CLEAR edge
    -- walk (upd_*_r3) and is steady for the whole stamp loop, so R_STAMP uses
    -- it too.
    signal cx_rd_data_f : std_logic_vector(17 downto 0) := (others => '0');
    signal cx_wr_addr  : unsigned(7 downto 0) := (others => '0');
    signal cx_wr_data  : std_logic_vector(17 downto 0) := (others => '0');
    signal cx_wr_en    : std_logic := '0';

    -- ---------------------------------------------------------------
    -- Rasterizer FSM
    -- ---------------------------------------------------------------
    -- Active-edge list: during R_CLEAR the edge-update appends the index of
    -- every edge active on this scanline to act_list_ram.  R_STAMP then
    -- iterates ONLY that list (R_STAMP_SCAN issues the list read, SCAN2
    -- consumes it and enters the stamp pipeline).  This replaces the old
    -- inline scan that indexed the full active vector every cycle (a wide
    -- N:1 mux that was the HD critical path) and walked all N edges — now
    -- the stamp phase is O(active-this-row) with no wide mux.
    type t_raster_state is (R_IDLE, R_CLEAR, R_WIPE, R_STAMP_SCAN, R_STAMP_SCAN2,
                             R_STAMP_PRELOAD, R_STAMP_PRELOAD2, R_STAMP_PRELOAD3,
                             R_STAMP_PRELOAD4, R_STAMP, R_STAMP_DRAIN);
    signal raster_state    : t_raster_state := R_IDLE;
    signal raster_cycle    : unsigned(10 downto 0) := (others => '0');
    -- Rolling wipe pointers: R_WIPE clears C_WIPE_LEN entries per line at
    -- the pointer of the bank being prepared, so each bank's WHOLE 2048
    -- entries are zeroed every 64 of its lines (128 lines).  The per-line
    -- clear only covers the head's own x-range, so anything stamped
    -- elsewhere before the mesh/geometry settled at power-up (or by a pan)
    -- would otherwise sit there for ever as thin vertical lines.
    -- ONE pointer for both banks does NOT work: the banks alternate per
    -- line and 2048/32 = 64 chunks is even, so bank A only ever saw the
    -- even chunks and bank B the odd ones -- half of each bank never wiped
    -- (HW: lines present or not depending on where load garbage landed).
    constant C_WIPE_LEN    : natural := 32;
    signal wipe_ptr_a_r    : unsigned(10 downto 0) := (others => '0');
    signal wipe_ptr_b_r    : unsigned(10 downto 0) := (others => '0');
    -- Active-edge list storage + pointers (single-driver: all in raster_proc).
    type t_act_list_ram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal act_list_ram : t_act_list_ram := (others => (others => '0'));
    signal al_wr_addr   : unsigned(7 downto 0) := (others => '0');  -- build ptr (R_CLEAR)
    signal al_rd_addr   : unsigned(7 downto 0) := (others => '0');  -- iterate ptr (R_STAMP)
    signal al_rd_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal al_count     : unsigned(7 downto 0) := (others => '0');  -- # active this line
    signal al_wr_en     : std_logic := '0';
    signal al_wr_data   : std_logic_vector(7 downto 0) := (others => '0');
    -- Wide enough for max |slope| + 2*THICK.
    signal stamp_sub       : unsigned(7 downto 0)  := (others => '0');

    -- Per-edge values held across the edge's STAMP iterations.  Latched
    -- once at edge transition.  cur_x doesn't need a "held" register —
    -- cx_rd_addr stays parked on the edge's index for the duration of
    -- its stamps, so cx_rd_data is steady (and registered, so it has
    -- the same per-cycle cost as a flop).
    -- bnd_held is now latched from the mesh BRAM in compute_counts_a
    -- (PRELOAD2), i.e. AFTER R_STAMP_PRELOAD has drained the previous edge's
    -- pending stamp — so the drain naturally uses the previous edge's routing
    -- and the old bnd_held_d1 shadow register is gone.
    signal bnd_held       : std_logic := '0';   -- 1=route stamps to boundary buffer
    signal bside_held     : std_logic := '0';   -- 1=this crossing ENTERS the fill
    -- Pre-formed in compute_counts_a2, consumed in compute_counts_b.
    signal srel_pre_r     : signed(10 downto 0) := (others => '0');
    signal cx_h_r         : signed(13 downto 0) := (others => '0');
    -- Absolute screen x of the edge's TRUE crossing (h_centre + cur_x).  The
    -- fill crossing is written from this in R_STAMP_PRELOAD4, independently of
    -- the thickness sweep: riding on the sweep meant a near-flat edge whose
    -- sweep hit the MAX_STAMP_M1 cap could silently lose its crossing, leaving
    -- the scanline's enter/leave pairs unbalanced.
    signal cx_abs_r       : signed(13 downto 0) := (others => '0');
    signal count_m1_held  : unsigned(7 downto 0) := (others => '0');
    -- Per-edge x-extent for THIS edge, read from the x-extent BRAM that the
    -- copy engine fills each frame.  It is the edge's ACTUAL (scaled, morphed,
    -- noised) span padded by pad_r and then clipped to the head bbox +-hw_r,
    -- so a single pair of compares in R_STAMP does three jobs:
    --   * stops the thickness sweep of a near-horizontal edge overshooting
    --     past its endpoint (the stray spurs around the eyes),
    --   * squares off the +-pad_r row extension into a proper line cap,
    --   * keeps every write inside the region R_CLEAR wipes.
    signal xlo_held       : signed(12 downto 0) := (others => '0');
    signal xhi_held       : signed(12 downto 0) := (others => '0');

    -- Stamp pipeline registers (Stage A → Stage B).  Splits the
    -- (cur_x + start_rel + sub) + h_centre add chain into two cycles.
    -- Stage A registers the absolute stamp x + valid, Stage B writes
    -- and writes the LB.  Adds 1 cycle of latency, drained in
    -- R_STAMP_DRAIN.
    signal stamp_abs_r    : signed(13 downto 0) := (others => '0');
    signal stamp_valid_r  : std_logic := '0';
    -- Everything in the stamp address that is CONSTANT across an edge's sweep
    -- — h_centre, the DDA's cur_x, and start_rel — is pre-added once per edge
    -- in compute_counts_b, and the x-extent clamp is pre-offset to absolute
    -- screen coordinates with it.  R_STAMP is then a single two-input add plus
    -- two compares, and Stage B is a bare register move: this is the hottest
    -- path in the design (once per stamped pixel).
    signal stamp_base_r   : signed(13 downto 0) := (others => '0');
    signal xlo_abs_r      : signed(13 downto 0) := (others => '0');
    signal xhi_abs_r      : signed(13 downto 0) := (others => '0');
    -- EOR-vs-thickness decoupling.  A boundary edge writes its full
    -- thick sweep to the DETAIL buffer (visible) but only a single 1px
    -- crossing mark to the BOUNDARY buffer (which the EOR fill walker
    -- reads).  center_sub_held is the stamp_sub offset at which the
    -- sweep pixel lands exactly on cur_x (the edge's true crossing);
    -- stamp_is_center_r flags that pixel through the Stage A->B pipe.
    -- Without this, thick/adjacent boundary stamps merge into one run
    -- and corrupt the EOR parity (the "5 sections" banding + the mouth
    -- stub were both this).
    -- compute_counts is split across two preload cycles to shorten its
    -- combinational depth (it was the HD critical path).  Stage a
    -- (PRELOAD2) registers the slope's sign / magnitude and the per-side
    -- thickness extension; stage b (PRELOAD3) does the add + cap + center.
    signal s_int_held  : signed(11 downto 0) := (others => '0');
    signal abs_s_held  : unsigned(7 downto 0) := (others => '0');
    signal ext_held    : unsigned(7 downto 0) := (others => '0');  -- per-side px
    signal flat_held   : std_logic := '0';   -- mesh ymin == ymax
    signal sneg_held   : std_logic := '0';   -- slope is negative

    -- h_centre and bbox-edge constants pre-added once per scanline to
    -- avoid recomputing the (h_centre - hw_r) sum every cycle
    -- of R_CLEAR.
    signal clear_base_r   : unsigned(10 downto 0) := (others => '0');
    signal h_centre_s     : signed(13 downto 0)  := to_signed(960, 14);

    -- R_CLEAR edge-update pipeline stage.  Stage 0 (combinational from
    -- raster_cycle) looks up mesh constants and the current_x value;
    -- those are registered into upd_*_r.  Stage 1 (one cycle later)
    -- uses upd_*_r to decide y_min / y_max / advance and writes back to
    -- current_x / active.  Breaks the big "array read → ALU → array
    -- write" combinational chain that was the dominant critical path.
    signal upd_valid_r  : std_logic := '0';
    signal upd_idx_r    : unsigned(7 downto 0)  := (others => '0');
    signal upd_ymin_r   : signed(12 downto 0)   := (others => '0');
    signal upd_ymax_r   : signed(12 downto 0)   := (others => '0');
    -- ymin==ymax on the RAW mesh values (upd_ymin_r/upd_ymax_r are already
    -- widened by +-pad_r, so they can no longer be compared for it).
    signal upd_horiz_r  : std_logic := '0';
    signal upd_xtop_r   : signed(17 downto 0)   := (others => '0');  -- Q11.7
    signal upd_slope_r  : signed(15 downto 0)   := (others => '0');  -- Q9.7
    -- One more pipeline stage so Stage 1 fires the cycle AFTER the
    -- BRAM read commits.  Without this, Stage 1's cx_rd_data would be
    -- ram[OLD edge] (the read issued one cycle earlier than the latch
    -- in upd_*_r).
    signal upd_valid_r2  : std_logic := '0';
    signal upd_idx_r2    : unsigned(7 downto 0)  := (others => '0');
    -- EFFECTIVE activation window = [ymin - pad_r, ymax + pad_r].  EVERY edge
    -- is extended, not just horizontals: on a horizontal edge (cur_x parked)
    -- the extra rows are what makes the bar 2*pad+1 px thick; on a sloped edge
    -- they are the line's square cap, squared off in x by [xlo, xhi].
    signal upd_ymin_r2   : signed(12 downto 0)   := (others => '0');
    signal upd_xtop_r2   : signed(17 downto 0)   := (others => '0');  -- Q11.7
    signal upd_slope_r2  : signed(15 downto 0)   := (others => '0');  -- Q9.7
    -- 1 = this edge is exactly horizontal (ymin==ymax): a single-scanline
    -- line whose `slope` field holds its WIDTH, not a per-row slope.  Such
    -- edges must NOT accumulate slope into cur_x.
    signal upd_is_horiz_r2 : std_logic := '0';
    -- Scanline-vs-edge compare flags, computed at the r->r2 stage (vs the
    -- effective bounds) and registered, so Stage 1 uses 1-bit flags instead
    -- of the 13-bit v_y_target compare (which was the HD critical path).
    -- Valid a stage early because v_y_target_r is constant across R_CLEAR.
    signal upd_is_ymin_r2   : std_logic := '0';   -- v_y_target_r = effective ymin
    signal upd_active_r2    : std_logic := '0';   -- ymin_eff <= y <= ymax_eff
    -- Third stage, aligned with cx_rd_data_f.
    signal upd_valid_r3     : std_logic := '0';
    signal upd_idx_r3       : unsigned(7 downto 0) := (others => '0');
    signal upd_xtop_r3      : signed(17 downto 0)  := (others => '0');
    signal upd_slope_r3     : signed(17 downto 0)  := (others => '0');
    signal upd_is_horiz_r3  : std_logic := '0';
    signal upd_is_ymin_r3   : std_logic := '0';
    signal upd_active_r3    : std_logic := '0';

    -- ---------------------------------------------------------------
    -- Active mesh BRAM (the BRAM-prefetch architecture).
    --
    -- The per-edge mesh-read mux (5:1 expr x N-entry LUT-ROM x 3-way
    -- variant) on the render critical path is what was capping HD
    -- timing.  Instead, the vblank copy engine (cp_proc) walks edges
    -- 0..N-1 once per frame and resolves (expr, group, om, oe) into a
    -- single value per (slot, field), written here.  The render then
    -- reads one BRAM word per slot — no LUT-ROM, no expr mux, no
    -- variant select on the per-pixel critical path.  4 EBRs.
    -- ---------------------------------------------------------------
    -- act_ymin word: bit 13 carries the edge's BOUNDARY flag (EOR fill), so
    -- the render no longer needs a 256:1 combinational read of C_EDGE_BND.
    type t_act_y_ram is array (0 to 255) of std_logic_vector(14 downto 0);
    type t_act_x_ram is array (0 to 255) of std_logic_vector(15 downto 0);
    -- x_top needs 18 bits: at full P12 size the head reaches +-333 px, which
    -- a Q9.7 (+-255.99 px) field silently clips -- the silhouette stops
    -- growing while the horizontal bars keep going, so they shoot out past
    -- the face.  Q11.7 keeps the 1/128 px slope precision the DDA needs.
    type t_act_xt_ram is array (0 to 255) of std_logic_vector(17 downto 0);
    signal act_ymin_ram  : t_act_y_ram := (others => (others => '0'));
    signal act_ymax_ram  : t_act_y_ram := (others => (others => '0'));
    signal act_xtop_ram  : t_act_xt_ram := (others => (others => '0'));
    signal act_slope_ram : t_act_x_ram := (others => (others => '0'));

    signal act_rd_addr   : unsigned(7 downto 0) := (others => '0');
    signal act_ymin_rd   : std_logic_vector(14 downto 0);
    signal act_ymax_rd   : std_logic_vector(14 downto 0);
    signal act_xtop_rd   : std_logic_vector(17 downto 0);
    signal act_slope_rd  : std_logic_vector(15 downto 0);

    -- Per-edge x-extent, written by the copy engine each frame (it used to be
    -- a pair of STATIC ROMs of the neutral geometry, which had to be disabled
    -- whenever a morph or noise moved an edge — and with the clamp disabled a
    -- near-horizontal edge's thickness sweep ran clean off its endpoints.
    -- That is the "thin lines around the eyes when changing parameters" bug).
    -- Same two EBRs as the ROMs they replace.
    type t_xext_ram is array (0 to 255) of std_logic_vector(12 downto 0);
    signal act_xlo_ram   : t_xext_ram := (others => (others => '0'));
    signal act_xhi_ram   : t_xext_ram := (others => (others => '0'));
    signal act_xlo_rd    : std_logic_vector(12 downto 0);
    signal act_xhi_rd    : std_logic_vector(12 downto 0);

    signal act_wr_en     : std_logic := '0';
    signal act_wr_addr   : unsigned(7 downto 0) := (others => '0');
    signal act_ymin_wr   : std_logic_vector(14 downto 0) := (others => '0');
    signal act_ymax_wr   : std_logic_vector(14 downto 0) := (others => '0');
    signal act_xtop_wr   : std_logic_vector(17 downto 0) := (others => '0');
    signal act_slope_wr  : std_logic_vector(15 downto 0) := (others => '0');
    signal act_xlo_wr    : std_logic_vector(12 downto 0) := (others => '0');
    signal act_xhi_wr    : std_logic_vector(12 downto 0) := (others => '0');

    -- ---------------------------------------------------------------
    -- Copy engine — SEQUENTIAL per-edge FSM.  Once per frame in vblank it
    -- walks edges 0..N-1, reads the (static) mesh, optionally jitters both
    -- endpoints (K6 noise) and recomputes the slope with a restoring DIVIDER
    -- (shallow add/compare per cycle — no multiply, so it meets the HD clock),
    -- then writes the active mesh BRAM.  ~22 cycles/edge fits the vblank
    -- window easily.  Only the 'open' mesh variant is read (the static dense
    -- mesh has no per-expression variants), which also trims the LUT-ROM mux.
    -- ---------------------------------------------------------------
    -- Split into many shallow states (free in vblank) so no single cycle has a
    -- deep combinational path — keeps the noise engine off the HD critical path.
    type t_cp_state is (CP_IDLE, CP_LOAD, CP_LOAD2, CP_HASH, CP_PHASE, CP_TRI,
                        CP_MR0, CP_MR1, CP_MR2, CP_MLOAD, CP_MSEL, CP_MDO,
                        CP_MACC,
                        CP_OFF, CP_SCALE, CP_SORT, CP_FLOOR, CP_SPAN, CP_DX,
                        CP_DIV, CP_XEXT, CP_XEXT2, CP_XT, CP_WR);
    signal cp_state : t_cp_state := CP_IDLE;
    signal cp_addr  : unsigned(8 downto 0) := (others => '0');
    -- The FSM's branch conditions pre-registered as 1-bit flags.  Feeding
    -- cp_state's next-state logic from a 9-bit compare (cp_addr), a 4-bit one
    -- (m_cnt) and a 5-bit one (d_cnt) made that cone — which fans out to every
    -- register in the copy engine — the HD Analog critical path.
    signal cp_done_r : std_logic := '0';   -- cp_addr has reached C_NUM_EDGES
    signal m_at11_r  : std_logic := '0';   -- m_cnt = 11 (last morph product)
    signal m_at15_r  : std_logic := '0';   -- m_cnt = 15 (last scale product)
    signal d_last_r  : std_logic := '0';   -- divider on its final step

    -- K6 NOISE: coherent 2D point wander.  K6 = spread (level 0..7), K5 =
    -- speed.  Each vertex's (dx,dy) is hashed from its (x,y) so a shared point
    -- moves identically and the wireframe stays joined.
    -- spread is a 5-bit LINEAR amplitude (0..31): offset = (tri*spread)>>1 in
    -- Q9.7, so the response is smooth (32 steps) and tops out near +-31px (what
    -- the old coarse level-4 / knob-50% gave) at full knob.
    signal noise_spread_r : unsigned(4 downto 0)  := (others => '0');
    signal noise_speed_r  : unsigned(4 downto 0)  := (others => '0');
    signal noise_t_r      : unsigned(15 downto 0) := (others => '0');
    signal noise_on       : std_logic;
    -- P9 "Noise": '1' = allow the thin-horizontal-streak texture (edges may
    -- collapse flat); '0' = enforce a min vertical span so they don't streak.
    signal noise_streaks_r : std_logic := '0';
    constant NOISE_SPAN_FLOOR : integer := 8;
    -- Latched triangle-wave samples (per endpoint, x/y phases) -> CP_AMP.
    signal t_trxt, t_tryt, t_trxb, t_tryb : signed(9 downto 0) := (others => '0');

    -- Triangle wave: period 1024, amplitude -256..+255.  Low 10 phase bits
    -- (plain truncation; resize() on signed would corrupt the wrap).
    function tri10(ph : signed(16 downto 0)) return signed is
        variable p  : unsigned(9 downto 0);
        variable up : unsigned(8 downto 0);
    begin
        p := unsigned(ph(9 downto 0));
        if p(9) = '0' then up := p(8 downto 0);
        else               up := to_unsigned(511, 9) - p(8 downto 0);
        end if;
        return resize(signed('0' & up), 10) - to_signed(256, 10);
    end function;

    -- Per-edge working registers (one edge in flight at a time).
    signal w_xtop, w_xbot, w_slope : signed(15 downto 0) := (others => '0');
    signal w_ymin, w_ymax          : signed(12 downto 0) := (others => '0');
    signal w_phxt, w_phyt, w_phxb, w_phyb : signed(16 downto 0) := (others => '0');
    signal o_dxt, o_dxb : signed(17 downto 0) := (others => '0');  -- x offsets
    signal o_dyt, o_dyb : signed(13 downto 0) := (others => '0');  -- y offsets
    signal w_nxt, w_nxb : signed(17 downto 0) := (others => '0');
    signal w_nyt, w_nyb : signed(13 downto 0) := (others => '0');
    signal w_xa, w_xb   : signed(17 downto 0) := (others => '0');
    signal w_ya, w_yb   : signed(13 downto 0) := (others => '0');
    -- 1 = this edge is horizontal in the NEUTRAL mesh (ymin==ymax).  A morph or
    -- noise nudge can tilt it off-horizontal, which would drop its is_horiz
    -- Y-thicken/width-fill and render it as a thin diagonal stroke; CP_FLOOR
    -- collapses such edges back to a flat row so they stay thick bars.
    signal horiz_r      : std_logic := '0';
    -- This edge's boundary (EOR fill) flag, carried through the copy engine so
    -- it can be packed into the mesh BRAM's ymin word.
    signal bnd_wr_r     : std_logic := '0';
    signal bside_wr_r   : std_logic := '0';
    signal w_dxsign     : std_logic := '0';
    -- Restoring divider: quot = |dx| / span (slope magnitude, Q9.7).
    signal d_rem      : unsigned(8 downto 0)  := (others => '0');
    signal d_quot     : unsigned(17 downto 0) := (others => '0');
    signal d_dividend : unsigned(17 downto 0) := (others => '0');
    signal d_divisor  : unsigned(7 downto 0)  := (others => '0');
    signal d_cnt      : integer range 0 to 17 := 0;
    -- Slope actually written to the mesh BRAM this frame, and the cur_x load
    -- value backed up by pad_r rows of it (see CP_XEXT / CP_XT).
    signal slope_sel_r : signed(15 downto 0) := (others => '0');
    signal xtop_ext_r  : signed(17 downto 0) := (others => '0');
    signal scale_on    : std_logic;

    -- ---------------------------------------------------------------
    -- STATIC MESH ROMs.  These used to be read COMBINATIONALLY out of the
    -- integer constant arrays in CP_LOAD, which made cp_addr drive five
    -- 256-entry LUT mux trees.  That fanout — not LC count — was what pinned
    -- the HD critical path: nextpnr had to scatter the copy engine across the
    -- die, and the routed paths ran ~12 ns of wire behind ~4 ns of logic.
    -- Reading them as REGISTERED std_logic_vector arrays maps them into 5
    -- EBRs instead, replacing all that fanout with five address ports.
    -- Costs one extra copy-engine state (CP_LOAD2) for the read latency.
    -- Bits 13/14 of the ymin word carry the edge's boundary flag and, for a
    -- boundary edge, which SIDE of the filled region it is (see C_EDGE_BSIDE).
    -- ---------------------------------------------------------------
    type t_rom14 is array (0 to 255) of std_logic_vector(14 downto 0);
    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);
    function init_ymin return t_rom14 is
        variable r : t_rom14 := (others => (others => '0'));
    begin
        for i in 0 to C_NUM_EDGES - 1 loop
            r(i) := std_logic_vector(to_signed(C_EDGE_Y_MIN(i), 15));
            if C_EDGE_BND(i) = 1 then r(i)(13) := '1'; else r(i)(13) := '0'; end if;
            if C_EDGE_BSIDE(i) = 1 then r(i)(14) := '1'; else r(i)(14) := '0'; end if;
        end loop;
        return r;
    end function;
    function init_rom14(src : t_int_array) return t_rom14 is
        variable r : t_rom14 := (others => (others => '0'));
    begin
        for i in 0 to C_NUM_EDGES - 1 loop
            r(i) := std_logic_vector(to_signed(src(i), 15));
        end loop;
        return r;
    end function;
    function init_rom16(src : t_int_array) return t_rom16 is
        variable r : t_rom16 := (others => (others => '0'));
    begin
        for i in 0 to C_NUM_EDGES - 1 loop
            r(i) := std_logic_vector(to_signed(src(i), 16));
        end loop;
        return r;
    end function;
    constant C_ROM_YMIN  : t_rom14 := init_ymin;
    constant C_ROM_YMAX  : t_rom14 := init_rom14(C_EDGE_Y_MAX);
    constant C_ROM_XTOP  : t_rom16 := init_rom16(C_EDGE_X_TOP);
    constant C_ROM_SLOPE : t_rom16 := init_rom16(C_EDGE_SLOPE);
    constant C_ROM_XBOT  : t_rom16 := init_rom16(C_EDGE_X_BOT);
    signal rom_ymin_r  : std_logic_vector(14 downto 0);
    signal rom_ymax_r  : std_logic_vector(14 downto 0);
    signal rom_xtop_r  : std_logic_vector(15 downto 0);
    signal rom_slope_r : std_logic_vector(15 downto 0);
    signal rom_xbot_r  : std_logic_vector(15 downto 0);

    -- ---------------------------------------------------------------
    -- MORPHS: mouth open/close (K1 analog), eye blink (P8), expression (K4
    -- blend).  Per-edge endpoint px deltas live in 12 packed ROMs (top/bot for
    -- mouth, eye, and 4 expressions).  In the cp_proc the selected deltas are
    -- scaled (mouth by K1, expression by the K4 blend t) and added to the
    -- endpoints at CP_OFF alongside the noise offset, so the existing
    -- sort->divider->slope pipeline recomputes the slope for free.  All in
    -- vblank; the per-pixel render path is untouched.
    -- ---------------------------------------------------------------
    signal mouth_frac_r : unsigned(4 downto 0) := (others => '0');  -- K1 CLOSE amount (0=open)
    signal expr_base_r  : unsigned(1 downto 0) := (others => '0');  -- K4 lower expr A (0..3)
    signal expr_frac_r  : unsigned(4 downto 0) := (others => '0');  -- K4 blend t (0..31)
    signal morph_active : std_logic;

    -- SPARSE morph storage: per-VERTEX delta ROMs (one per morph, indexed by a
    -- vertex slot 0..NSLOT-1; slot 0 = static = zero) + an edge->slot map.  A
    -- delta is a property of a vertex, so storing per moving-vertex (not per
    -- edge-endpoint) drops 12 edge ROMs to 6 vertex ROMs + 1 map (12->7 BRAM).
    type t_vrom is array (0 to C_MORPH_NSLOT - 1) of std_logic_vector(15 downto 0);
    function init_vrom(src : t_int_array) return t_vrom is
        variable r : t_vrom;
    begin
        for i in 0 to C_MORPH_NSLOT - 1 loop
            r(i) := std_logic_vector(to_unsigned(src(i) mod 65536, 16));
        end loop;
        return r;
    end function;
    constant C_VROM_M : t_vrom := init_vrom(C_VROM_MOUTH);
    constant C_VROM_E : t_vrom := init_vrom(C_VROM_EYE);
    constant C_VROM_H : t_vrom := init_vrom(C_VROM_HAPPY);
    constant C_VROM_S : t_vrom := init_vrom(C_VROM_SAD);
    constant C_VROM_A : t_vrom := init_vrom(C_VROM_ANGRY);
    constant C_VROM_U : t_vrom := init_vrom(C_VROM_SUR);

    type t_emap is array (0 to 255) of std_logic_vector(15 downto 0);
    function init_emap(src : t_int_array) return t_emap is
        variable r : t_emap := (others => (others => '0'));
    begin
        for i in 0 to C_NUM_EDGES - 1 loop
            r(i) := std_logic_vector(to_unsigned(src(i), 16));
        end loop;
        return r;
    end function;
    constant C_EMAP_ROM : t_emap := init_emap(C_MORPH_EMAP);

    -- emap read at cp_addr; the 6 vrom reads share one address (vrom_addr), so
    -- top and bottom endpoints are read in successive cycles (CP_MR0..MR2).
    signal emap_r    : std_logic_vector(15 downto 0);
    signal vrom_addr : unsigned(5 downto 0) := (others => '0');
    signal vr_m, vr_e, vr_h, vr_s, vr_a, vr_u : std_logic_vector(15 downto 0);
    -- latched top/bottom-endpoint deltas for the current edge (fed to CP_MLOAD).
    signal mrd_mt, mrd_mb, mrd_et, mrd_eb : std_logic_vector(15 downto 0);
    signal mrd_ht, mrd_hb, mrd_st, mrd_sb : std_logic_vector(15 downto 0);
    signal mrd_at, mrd_ab, mrd_ut, mrd_ub : std_logic_vector(15 downto 0);

    -- Per-edge morph working registers (px deltas), held for the multiply loop.
    signal m_mxt, m_myt, m_mxb, m_myb : signed(7 downto 0) := (others => '0'); -- mouth
    signal m_dxt, m_dyt, m_dxb, m_dyb : signed(8 downto 0) := (others => '0'); -- exprB - exprA
    -- one shared multiplier sequenced over 8 products (operand select split
    -- into CP_MSEL to keep the mux off the multiply path); results accumulate
    -- straight into mt_* so no per-product intermediate registers are kept.
    -- 0..3 noise, 4..7 expr, 8..11 mouth, 12..15 P12 head scale.
    signal m_cnt   : integer range 0 to 15 := 0;
    signal mul_a_r : signed(10 downto 0) := (others => '0');
    -- 9 bits since v1.0.3: the per-mode scale factors (sc_mx_r / sc_my_r)
    -- reach -199.
    signal mul_b_r : signed(8 downto 0)  := (others => '0');
    -- The product gets a cycle of its own (CP_MDO) before anything consumes
    -- it (CP_MACC).  Folding multiply + shift + accumulate into one state made
    -- the 11x8 array feed a 20-bit carry chain and was the HD critical path.
    signal prod_r  : signed(19 downto 0) := (others => '0');
    -- ONE-HOT destination for prod_r, latched in CP_MSEL alongside the
    -- operands.  Decoding the destination from m_cnt inside CP_MACC put an
    -- 8-LUT chain (shared with cp_state's own next-state cone) in front of
    -- every accumulator's clock enable — the HD critical path.  One-hot makes
    -- each enable 2 LUTs deep and independent of the others.
    --   0 o_dxt  1 o_dyt  2 o_dxb  3 o_dyb
    --   4 mt_xt  5 mt_yt  6 mt_xb  7 mt_yb
    --   8 sc_xt  9 sc_xb 10 sc_yt 11 sc_yb
    signal mdst_r  : std_logic_vector(11 downto 0) := (others => '0');
    -- running total morph delta per endpoint (px): exprA + eye init, then the
    -- expr-blend and mouth products accumulate in CP_MDO.
    signal mt_xt, mt_yt, mt_xb, mt_yb : signed(10 downto 0) := (others => '0');
    -- P12 scale accumulators: (coord * size_m) >> 8, added back to the coord
    -- in CP_SCALE.  x is Q9.7 so it takes two products (integer part shifted
    -- 7 up, fraction direct) to stay inside the shared 11x8 multiplier.
    signal sc_xt, sc_xb : signed(18 downto 0) := (others => '0');
    signal sc_yt, sc_yb : signed(13 downto 0) := (others => '0');

    -- Render-side Stage 0 pre signals.  The active-mesh BRAM read has
    -- effective 2-cycle latency from when Stage 0a sets act_rd_addr
    -- (register input) to when act_*_rd presents the data (register
    -- output): 1 cycle for the address to be visible, 1 cycle for the
    -- BRAM process to commit ram[addr] -> data register.  So control
    -- signals (valid/idx/active/bnd) need to ride a 2-stage shift
    -- (_pre -> _pre2) before Stage 0b reads them together with the
    -- act_*_rd data.  cx_rd_addr is also driven from _pre2 so
    -- cx_rd_data lines up the same way.
    signal upd_valid_pre  : std_logic := '0';
    signal upd_idx_pre    : unsigned(7 downto 0) := (others => '0');
    signal upd_active_pre : std_logic := '0';
    signal upd_bnd_pre    : std_logic := '0';
    signal upd_valid_pre2  : std_logic := '0';
    signal upd_idx_pre2    : unsigned(7 downto 0) := (others => '0');
    signal upd_active_pre2 : std_logic := '0';
    signal upd_bnd_pre2    : std_logic := '0';

    -- Line-buffer write ports.  Two physical buffers (each ping-pong):
    -- lb_bnd_* receives only boundary-edge stamps and is what the EOR
    -- walker reads.  lb_det_* receives detail-edge stamps (brows,
    -- centerlines) which should be visible but must NOT toggle EOR
    -- parity — otherwise open polylines like the brow arches would
    -- create black holes in the grid fill underneath them.
    signal lb_wr_en_bin_r  : std_logic := '0';
    signal lb_wr_en_bout_r : std_logic := '0';
    signal lb_wr_en_det_r : std_logic := '0';
    signal lb_wr_addr_r   : unsigned(10 downto 0) := (others => '0');
    signal lb_wr_data_r   : std_logic := '0';
    signal lb_bnd_data_r  : std_logic := '0';
    signal lb_ab_r        : std_logic := '0';

    -- Inlined dual-bank line buffers (6 EBRs: enter × 2, leave × 2, det × 2).
    -- The fill's ENTER and LEAVE crossings live in SEPARATE memories, not two
    -- bits of one word: at a local-minimum vertex both land on the same pixel,
    -- and a single RAM write REPLACES the word rather than OR-ing into it, so
    -- one of the pair would be lost — whichever edge happened to be stamped
    -- last, which is just edge-index order.  Two memories, two write enables,
    -- both crossings recorded.
    type t_lb is array (0 to 2047) of std_logic;
    signal lb_bin_a  : t_lb := (others => '0');
    signal lb_bin_b  : t_lb := (others => '0');
    signal lb_bout_a : t_lb := (others => '0');
    signal lb_bout_b : t_lb := (others => '0');
    signal lb_det_a : t_lb := (others => '0');
    signal lb_det_b : t_lb := (others => '0');
    signal lb_rd_addr     : unsigned(10 downto 0);
    signal lb_bin_rd_a_r  : std_logic := '0';
    signal lb_bin_rd_b_r  : std_logic := '0';
    signal lb_bout_rd_a_r : std_logic := '0';
    signal lb_bout_rd_b_r : std_logic := '0';
    signal lb_det_rd_a_r  : std_logic := '0';
    signal lb_det_rd_b_r  : std_logic := '0';
    signal lb_ab_d1       : std_logic := '0';  -- aligns mux with BRAM read latency
    signal lb_bnd_rd_data : std_logic_vector(1 downto 0);  -- drives the fill walker
    signal lb_any_rd_data : std_logic;  -- boundary OR detail — drives display

    -- ---------------------------------------------------------------
    -- Post-line-buffer pipeline registers
    -- ---------------------------------------------------------------
    signal edge_hit_r : std_logic := '0';
    signal pix_y_r    : unsigned(9 downto 0) := (others => '0');
    signal pix_yf_r   : unsigned(9 downto 0) := (others => '0');
    -- Output stage (out_proc)
    signal show_pix    : std_logic;
    signal show_d1_r   : std_logic := '0';
    signal show_d2_r   : std_logic := '0';
    signal out_phase_r : std_logic := '0';   -- parity of the pixel in pipe(LATENCY-2)
    signal out_y_r     : unsigned(9 downto 0) := C_BLACK;
    signal out_u_r     : unsigned(9 downto 0) := C_CHROMA_MID;
    signal out_v_r     : unsigned(9 downto 0) := C_CHROMA_MID;
    signal pix_u_r    : unsigned(9 downto 0) := C_CHROMA_MID;
    signal pix_v_r    : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Even-odd polygon fill state.  Walks the LB left-to-right per
    -- scanline; flips on every 0->1 transition, which is the parity
    -- rule for "am I inside the polygon".  When the wireframe contains
    -- nested closed shapes (silhouette, eyes, brows, mouth) the rule
    -- naturally fills the silhouette interior MINUS the feature
    -- interiors — no per-region bbox needed.
    signal eor_inside_r   : std_logic := '0';
    signal eor_inside_d_r : std_logic := '0';  -- aligned with edge_hit_r
    -- Rightmost fill crossing written on the row, per bank.  A scanline can
    -- only run away to the right if its LAST crossing was an "enter" — an
    -- unbalanced pair, which a morph can still produce by moving a shared lip
    -- vertex so the two edges meeting there no longer share a row.  Gating the
    -- fill past the last crossing removes every such streak by construction,
    -- and is a no-op on a correct row (whose last crossing is always a
    -- "leave", after which the walker is already outside).
    signal bnd_max_a_r    : unsigned(10 downto 0) := (others => '0');
    signal bnd_max_b_r    : unsigned(10 downto 0) := (others => '0');
    signal bnd_max_disp   : unsigned(10 downto 0);
    signal past_max_r     : std_logic := '0';
    -- Saturating count of consecutive '1' pixels in the LB walk.  Used
    -- to recognise abnormally long solid runs (e.g. the mouth
    -- horizontal centerline, 124 px in one row) as non-crossings — we
    -- apply a cancelling toggle on the trailing 1->0 transition so the
    -- run doesn't break even-odd parity for the rest of the scanline.
    -- Normal edge crossings are 3-4 px and never trip the threshold.

begin

    -- ---------------------------------------------------------------
    -- Input pipe + timing tracking
    -- ---------------------------------------------------------------
    timing_proc : process(clk)
        variable v_pk : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if unsigned(registers_in(1)) < 4 then
                k2_idx_r <= unsigned(registers_in(1)(1 downto 0));
            else
                k2_idx_r <= unsigned(registers_in(1)(9 downto 8));
            end if;
            if unsigned(registers_in(2)) < 8 then
                k3_idx_r <= unsigned(registers_in(2)(2 downto 0));
            else
                k3_idx_r <= unsigned(registers_in(2)(9 downto 7));
            end if;

            -- ---- Resolution scaling: per-mode constants (free-running,
            -- quasi-static, kept out of the vsync cone) ----
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 1 | 2 | 10 => md_cls_r <= "001";   -- 1080i
                when 5 | 6 | 14 => md_cls_r <= "010";   -- 720p
                when 0          => md_cls_r <= "011";   -- 480i
                when 4          => md_cls_r <= "100";   -- 480p
                when 8          => md_cls_r <= "101";   -- 576i
                when 12         => md_cls_r <= "110";   -- 576p
                when others     => md_cls_r <= "000";   -- 1080p
            end case;
            -- k in Q8 per axis (x: 1080 lines 1.0, 720p 2/3, SD 1/2; y: 1080
            -- over this mode's FIELD lines), the y offset (see md_yoff_r) and
            -- the noise span floor (8 rows at 1080, scaled, min 2).
            case md_cls_r is
                when "001" =>   -- 1080i: y 1/2
                    md_kxq_r <= to_unsigned(256, 9); md_kyq_r <= to_unsigned(128, 9);
                    md_yoff_r <= to_signed(-3200, 14); md_nfl_r <= to_unsigned(4, 4);
                when "010" =>   -- 720p: 2/3
                    md_kxq_r <= to_unsigned(171, 9); md_kyq_r <= to_unsigned(171, 9);
                    md_yoff_r <= to_signed(-2091, 14); md_nfl_r <= to_unsigned(5, 4);
                when "011" =>   -- 480i: 486 / 2160 per field line
                    md_kxq_r <= to_unsigned(128, 9); md_kyq_r <= to_unsigned(58, 9);
                    md_yoff_r <= to_signed(-5030, 14); md_nfl_r <= to_unsigned(2, 4);
                when "100" =>   -- 480p: 480 / 1080
                    md_kxq_r <= to_unsigned(128, 9); md_kyq_r <= to_unsigned(114, 9);
                    md_yoff_r <= to_signed(-3570, 14); md_nfl_r <= to_unsigned(4, 4);
                when "101" =>   -- 576i: 576 / 2160 per field line
                    md_kxq_r <= to_unsigned(128, 9); md_kyq_r <= to_unsigned(68, 9);
                    md_yoff_r <= to_signed(-4753, 14); md_nfl_r <= to_unsigned(2, 4);
                when "110" =>   -- 576p: 576 / 1080
                    md_kxq_r <= to_unsigned(128, 9); md_kyq_r <= to_unsigned(137, 9);
                    md_yoff_r <= to_signed(-2978, 14); md_nfl_r <= to_unsigned(4, 4);
                when others =>  -- 1080p: identity
                    md_kxq_r <= to_unsigned(256, 9); md_kyq_r <= to_unsigned(256, 9);
                    md_yoff_r <= to_signed(0, 14);     md_nfl_r <= to_unsigned(8, 4);
            end case;
            md_yoffb_r <= md_yoff_r - to_signed(128, 14);
            if md_cls_r = "000" then md_ident_r <= '1'; else md_ident_r <= '0'; end if;
            if md_cls_r(2) = '1' or md_cls_r = "011" then md_sd_r <= '1';
            else md_sd_r <= '0'; end if;

            -- K2 stroke half-widths per axis.  The 1080p stroke 2w+1 scaled to
            -- the mode and rounded to the nearest odd width (min 1 px):
            --   x: 1080 lines w; 720p 0,1,1,2; SD 0,0,1,1
            --   y: 1080p w; 1080i 0,0,1,1; 720p 0,1,1,2; 480i 0,0,0,0;
            --      480p/576p 0,0,1,1; 576i 0,0,0,1
            case md_cls_r is
                when "000" =>
                    padx_pre_r <= k2_idx_r;  pady_pre_r <= k2_idx_r;
                when "001" =>
                    padx_pre_r <= k2_idx_r;  pady_pre_r <= '0' & k2_idx_r(1);
                when "010" =>
                    padx_pre_r <= (k2_idx_r(1) and k2_idx_r(0)) & (k2_idx_r(1) xor k2_idx_r(0));
                    pady_pre_r <= (k2_idx_r(1) and k2_idx_r(0)) & (k2_idx_r(1) xor k2_idx_r(0));
                when "011" =>
                    padx_pre_r <= '0' & k2_idx_r(1);  pady_pre_r <= "00";
                when "101" =>
                    padx_pre_r <= '0' & k2_idx_r(1);
                    pady_pre_r <= '0' & (k2_idx_r(1) and k2_idx_r(0));
                when others =>
                    padx_pre_r <= '0' & k2_idx_r(1);  pady_pre_r <= '0' & k2_idx_r(1);
            end case;

            -- K6 = POSITION: offset-binary knob -> two's complement pan (flip
            -- the MSB), -512..+511 px at 1080, scaled by the mode's x step.
            v_pk := signed(registers_in(5)(9 downto 0) xor "1000000000");
            case md_cls_r is
                when "000" | "001" => pan_pre_r <= v_pk;
                when "010"         => pan_pre_r <= shift_right(v_pk, 1)
                                                 + shift_right(v_pk, 3)
                                                 + shift_right(v_pk, 5);
                when others        => pan_pre_r <= shift_right(v_pk, 1);
            end case;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                hsync_falling_r <= '1';
            else
                hsync_falling_r <= '0';
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' and saw_active_r = '1' then
                vsync_falling_r <= '1';
            else
                vsync_falling_r <= '0';
            end if;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
                avid_line_r <= '1';
                saw_active_r <= '1';
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_x > to_unsigned(0, 12) then
                    max_x_r    <= pixel_x;
                    h_centre_r <= '0' & pixel_x(11 downto 1);
                end if;
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
                if avid_line_r = '1' then
                    aline_cnt_r <= aline_cnt_r + 1;
                end if;
                avid_line_r <= '0';
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' and saw_active_r = '1' then
                saw_active_r <= '0';
                if pixel_y > to_unsigned(0, 12) then
                    max_y_r    <= pixel_y;
                    v_centre_r <= '0' & pixel_y(11 downto 1);
                end if;
                pixel_y <= (others => '0');
                if aline_cnt_r > to_unsigned(0, 12) then
                    act_h_r <= aline_cnt_r;
                end if;
                aline_cnt_r <= (others => '0');

                -- K3 Palette.  Luma is pre-scaled in the LUT (capped at 768);
                -- the lines take it as-is, the Invert field takes 3/4 of it
                -- (one shift-subtract here instead of a per-pixel multiply).
                pal_y_r  <= C_PAL_Y(to_integer(k3_idx_r));
                pal_yf_r <= C_PAL_Y(to_integer(k3_idx_r))
                          - shift_right(C_PAL_Y(to_integer(k3_idx_r)), 2);
                pal_u_r <= C_PAL_U(to_integer(k3_idx_r));
                pal_v_r <= C_PAL_V(to_integer(k3_idx_r));
                -- T7-T11 switches
                bg_video_r   <= registers_in(6)(4);  -- T11
                invert_r     <= registers_in(6)(3);  -- T10
                noise_streaks_r <= registers_in(6)(2);  -- T9 "Noise": allow streaks
                fill_on_r <= registers_in(6)(0);  -- T7: 1 = solid-fill features
                open_eyes_r  <= registers_in(6)(1);  -- T8: eyes  open(1)/closed(0)
                -- K2 Thickness: top 2 bits → pad 0..3 → stroke 1/3/5/7 px
                -- (per axis, scaled to the mode; = k2_idx_r at 1080p)
                padx_r     <= padx_pre_r;
                pady_r     <= pady_pre_r;
                -- bottom field of an interlaced mode (SD only: HD inputs tie
                -- field_n high) is drawn half a field line lower
                field_bot_r <= not data_in.field_n;
                -- ---- P12 head size ----
                -- Only the raw knob is latched here; the height-derived
                -- ceiling and the scale factor itself are formed in the copy
                -- engine (CP_SIZE), keeping every multiply out of the vsync
                -- decision cone.
                size_k_r <= unsigned(registers_in(7)(9 downto 4));
                -- ---- Morph controls ----
                -- K1 = MOUTH (analog): 0% closed .. 100% open.  Current mesh is
                -- fully open (= neutral), so the close fraction = 31 - K1top5.
                mouth_frac_r <= to_unsigned(31, 5) - unsigned(registers_in(0)(9 downto 5));
                -- K4 = EXPRESSION blend across 5 (Neutral..Surprised): the knob
                -- spans positions 0..4; base = floor, frac t = fractional part.
                expr_base_r <= unsigned(registers_in(3)(9 downto 8));      -- 0..3 (A)
                expr_frac_r <= unsigned(registers_in(3)(7 downto 3));      -- 0..31 (t)
                -- P8 (open_eyes_r, registers_in(6)(1)) gates the eye-close morph.
                -- K5 = NOISE: spread (amplitude 0..31) from the top 5 bits,
                -- and the frame-phase speed rises with it (8..23) so one knob
                -- is never gated by another.  noise_t_r advances once per
                -- frame by the speed, so the per-vertex Lissajous phase
                -- animates.
                noise_spread_r <= unsigned(registers_in(4)(9 downto 5));
                noise_speed_r  <= to_unsigned(8, 5) + ('0' & unsigned(registers_in(4)(9 downto 6)));
                noise_t_r      <= noise_t_r + resize(noise_speed_r, 16);
                -- K6 = POSITION: -512..+511 px about the measured centre at
                -- 1080, scaled to the mode (pan_pre_r).
                pan_r <= pan_pre_r;
            end if;
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- Rasterizer FSM — one shared engine, time-multiplexed over edges.
    -- Writes go to bank selected by lb_ab_r and routed to bnd vs det
    -- buffer by lb_wr_en_bnd_r / lb_wr_en_det_r (set from bnd_held).
    -- ---------------------------------------------------------------
    raster_proc : process(clk)
        variable v_stamp_abs   : signed(13 downto 0);  -- absolute screen x

        -- Inline helper: kick off the per-edge BRAM reads (current_x and
        -- active mesh slope) plus latch the simple flag/group state.
        -- count_m1_held / start_rel_held are NOT computed here — they
        -- depend on the slope, which arrives in act_slope_rd one cycle
        -- later.  R_STAMP_PRELOAD's wait cycle does the math from the
        -- registered act_slope_rd value, which keeps the heavy LUT-ROM
        -- mux off the per-cycle critical path entirely.
        procedure latch_held(idx : integer) is
        begin
            cx_rd_addr  <= to_unsigned(idx, 8);
            act_rd_addr <= to_unsigned(idx, 8);
            -- bnd is NOT resolved here any more: it rides bit 13 of the mesh
            -- BRAM's ymin word and is latched in compute_counts_a, which
            -- removes a 256:1 combinational ROM read from this cycle.
        end procedure;

        -- Compute count_m1_held / start_rel_held from a registered slope
        -- value (act_slope_rd, arriving 1 cycle after the BRAM read was
        -- kicked off by latch_held).  Stamp range covers the row-to-row
        -- slope sweep + K2 px padding on each side, except when |slope|
        -- already exceeds 2*K2 — then the K2 padding would just extend
        -- horizontals past their endpoints, so drop it.
        -- compute_counts stage a (PRELOAD2): decode the slope into its
        -- sign / magnitude / "needs K2 padding" form and register them.
        -- Also resolves the horizontal-edge EOR exclusion here (parallel
        -- to the magnitude path).  Splitting the original one-cycle
        -- compute into a+b shortened the HD critical path.
        procedure compute_counts_a is
            variable s_fp   : signed(15 downto 0);
            variable s_int  : signed(11 downto 0);
            variable abs_s  : unsigned(7 downto 0);
            variable v_bymin : signed(12 downto 0);
            variable v_bymax : signed(12 downto 0);
        begin
            s_fp   := signed(act_slope_rd);
            s_int  := resize(s_fp(15 downto 7), 12);
            if s_int >= to_signed(0, 12) then
                abs_s := to_unsigned(to_integer(s_int), 8);
                sneg_held <= '0';
            else
                abs_s := to_unsigned(-to_integer(s_int), 8);
                sneg_held <= '1';
            end if;
            s_int_held <= s_int;
            abs_s_held <= abs_s;
            v_bymin := signed(act_ymin_rd(12 downto 0));
            v_bymax := signed(act_ymax_rd(12 downto 0));

            if act_ymax_rd(12 downto 0) = act_ymin_rd(12 downto 0) then
                flat_held <= '1';
            else
                flat_held <= '0';
            end if;

            -- EOR boundary rules (detail/line drawing is unaffected; this only
            -- gates the 1px boundary-buffer crossing the fill walker reads):
            --  * exactly-horizontal edge -> no crossing (lids/baselines),
            --  * half-open span [ymin, ymax): no crossing on the BOTTOM row, so
            --    a shared vertex counts once and a local-max vertex (both edges
            --    ending here, e.g. the mouth's cupid's-bow valley) counts zero.
            --    Without this the parity goes odd on those rows and the fill
            --    leaks right to the screen edge.
            --  * the +-pad_r cap rows outside [ymin, ymax) are not crossings
            --    either — the window the raster activates is wider than the
            --    edge's true span.
            if act_ymax_rd(12 downto 0) = act_ymin_rd(12 downto 0)
               or v_y_target_r >= v_bymax
               or v_y_target_r <  v_bymin then
                bnd_held <= '0';
            else
                bnd_held <= act_ymin_rd(13);
            end if;
            bside_held <= act_ymin_rd(14);

            -- Latch this edge's x-extent (aligned with act_slope_rd) for the
            -- stamp clamp.  The copy engine wrote it from the edge's ACTUAL
            -- (scaled / morphed / noised) endpoints padded by pad_r and clipped
            -- to +-hw_r, so it is valid in every configuration — it squares off
            -- the cap rows, stops a near-horizontal sweep overshooting its
            -- endpoint, and keeps writes inside the cleared window all at once.
            xlo_held <= signed(act_xlo_rd);
            xhi_held <= signed(act_xhi_rd);
        end procedure;

        -- compute_counts stage a2 (PRELOAD3): the per-side thickness
        -- extension.
        --
        -- ---- UNIFORM STROKE WIDTH ----
        -- A horizontal run of N px on a line of integer slope `a` covers a
        -- perpendicular thickness of only N/sqrt(1+a^2), so a FIXED pad draws
        -- near-vertical edges 2*pad+1 px thick and near-horizontal ones barely
        -- 1 px.  That mismatch — plus the old three-way branch that switched
        -- behaviour at |a| = 2*thick and again at |a| = 31 — is what made
        -- lines flip thickness as a morph nudged an edge's slope.  Scaling the
        -- extension with the slope instead:
        --     a = 0, or ymin==ymax  ->  ext = pad   (there the +-pad row
        --                                            extension does the
        --                                            thickening, since cur_x
        --                                            is parked)
        --     otherwise             ->  ext = pad*a + pad/2
        -- gives (a + 2*ext + 1)/sqrt(1+a^2) ~= 2*pad+1 at every slope.
        --
        -- Its own state because abs_s comes off the mesh EBR (2.15 ns
        -- clk-to-q + column routing), and stacking this shift-add-carry chain
        -- behind that read was the HD critical path.
        procedure compute_counts_a2 is
            variable ext : unsigned(10 downto 0);
            variable v_2a : unsigned(10 downto 0);
        begin
            -- (v1.0.3) Per-axis form: ext = pady*a + padx/2 (sloped) or padx
            -- (a = 0 / flat).  With padx = pady = pad (1080p) it is exactly
            -- the old pad*a + pad/2.  padx/2 is 0 or 1, OR-ed into the LSB of
            -- the doubled slope so no extra adder sits on this path.
            v_2a := shift_left(resize(abs_s_held, 11), 1);
            v_2a(0) := padx_r(1);
            if abs_s_held = 0 or flat_held = '1' then
                ext := resize(padx_r, 11);
            else
                case pady_r is
                    when "00"   => ext := resize(padx_r(1 downto 1), 11);
                    when "01"   => ext := resize(abs_s_held, 11)
                                        + resize(padx_r(1 downto 1), 11);
                    when "10"   => ext := v_2a;
                    when others => ext := v_2a + resize(abs_s_held, 11);
                end case;
            end if;
            if ext > to_unsigned(MAX_EXT, 11) then
                ext_held <= to_unsigned(MAX_EXT, 8);
            else
                ext_held <= ext(7 downto 0);
            end if;

            -- Everything in stage b that does NOT depend on ext is pre-formed
            -- here, in parallel with ext, so stage b is four INDEPENDENT
            -- single-operation subtracts off registers instead of a chain.
            -- (cx_rd_data_f first becomes valid this cycle.)
            if sneg_held = '1' then
                srel_pre_r <= resize(s_int_held, 11);
                cx_h_r     <= h_centre_s
                            + resize(signed(cx_rd_data_f(17 downto 7)), 14)
                            + resize(s_int_held, 14);
            else
                srel_pre_r <= (others => '0');
                cx_h_r     <= h_centre_s
                            + resize(signed(cx_rd_data_f(17 downto 7)), 14);
            end if;
            cx_abs_r  <= h_centre_s
                       + resize(signed(cx_rd_data_f(17 downto 7)), 14);
            xlo_abs_r <= h_centre_s + resize(xlo_held, 14);
            xhi_abs_r <= h_centre_s + resize(xhi_held, 14);
        end procedure;

        -- compute_counts stage b (PRELOAD4): add the K2 padding, cap the
        -- count, derive start_rel and the center offset — all from the
        -- registered stage-a values, so this cycle is just an add + a
        -- compare.
        -- compute_counts stage b (PRELOAD3): turn the registered slope
        -- magnitude + per-side extension into the sweep's length, its start
        -- offset from cur_x, and which sweep pixel is the true crossing.  The
        -- sweep always covers [cur_x - ext, cur_x + a + ext] (positive slope)
        -- or [cur_x + s - ext, cur_x + ext] (negative), i.e. it is centred on
        -- the edge, so the same rule serves vertical, diagonal and horizontal
        -- edges with no branch on orientation.
        procedure compute_counts_b is
            variable count_raw   : unsigned(9 downto 0);
            variable v_count_m1  : unsigned(7 downto 0);
        begin
            -- Four independent ops off registered stage-a2 values.
            count_raw := resize(abs_s_held, 10)
                       + shift_left(resize(ext_held, 10), 1);
            if count_raw > to_unsigned(MAX_STAMP_M1, 10) then
                v_count_m1 := to_unsigned(MAX_STAMP_M1, 8);
            else
                v_count_m1 := count_raw(7 downto 0);
            end if;
            count_m1_held <= v_count_m1;
            -- start_rel = srel_pre - ext, and stamp_base folds it together
            -- with h_centre and cur_x (both constant across this edge's
            -- sweep), so R_STAMP is one add.
            stamp_base_r  <= cx_h_r - signed(resize(ext_held, 14));
        end procedure;
    begin
        if rising_edge(clk) then
            -- Default each cycle: no write to either buffer
            lb_wr_en_bin_r  <= '0';
            lb_wr_en_bout_r <= '0';
            lb_wr_en_det_r  <= '0';

            v_y_target_r <= signed(resize(pixel_y, 13))
                        - signed(resize(v_centre_r, 13))
                        + to_signed(1, 13);

            -- Pre-add h_centre + pan once per cycle (2 cycles stale, fine
            -- because h_centre_r only updates on hsync and pan_r on vsync).
            h_orig_r     <= h_centre_r + unsigned(resize(pan_r, 12));
            clear_base_r <= resize(h_orig_r, 11) - resize(hw_r, 11);
            h_centre_s   <= signed(resize(h_orig_r, 14));

            -- Stage 0a': 1-cycle delay of pre signals so they reach
            -- Stage 0b at the same cycle that act_*_rd presents the
            -- BRAM-read data for the same edge.
            upd_valid_pre2  <= upd_valid_pre;
            upd_idx_pre2    <= upd_idx_pre;

            -- Stage 0b: BRAM read data lands here, aligned with the
            -- 2-stage-delayed control signals.  cx_rd_addr is driven
            -- from upd_idx_pre2 so cx_rd_data also lines up with
            -- upd_*_r2 in Stage 1 — but ONLY during R_CLEAR, since
            -- during R_STAMP latch_held owns cx_rd_addr (parked on the
            -- edge being stamped for the whole stamp loop).
            upd_valid_r  <= upd_valid_pre2;
            upd_idx_r    <= upd_idx_pre2;
            if raster_state = R_CLEAR then
                cx_rd_addr <= upd_idx_pre2;
            end if;
            -- ymin/ymax are latched ALREADY WIDENED by +-pad_r: doing the
            -- add here (a stage with nothing else in it) keeps the r->r2
            -- window test down to two plain register compares.
            upd_ymin_r   <= signed(act_ymin_rd(12 downto 0))
                          - signed(resize(pady_r, 13));
            upd_ymax_r   <= signed(act_ymax_rd(12 downto 0))
                          + signed(resize(pady_r, 13));
            if act_ymin_rd(12 downto 0) = act_ymax_rd(12 downto 0) then
                upd_horiz_r <= '1';
            else
                upd_horiz_r <= '0';
            end if;
            upd_xtop_r   <= signed(act_xtop_rd);
            upd_slope_r  <= signed(act_slope_rd);

            -- Pipeline shift: upd_*_r2 = upd_*_r delayed 1 cycle so
            -- Stage 1's use of upd_*_r2 aligns with cx_rd_data (which
            -- has BRAM's 1-cycle read latency).
            --
            -- EVERY edge's activation window is widened by ±pad_r.  On a
            -- horizontal edge (cur_x parked, `slope` = width) the extra rows
            -- ARE the stroke thickness; on a sloped edge they are its square
            -- cap, trimmed back in x by [xlo_held, xhi_held].  Applying the
            -- same rule to both removes the old orientation branch here.
            --
            -- "active" is derived rather than stored: R_CLEAR visits every
            -- edge on every line, so active == (ymin_eff <= y <= ymax_eff).
            -- That retires the 256-bit active vector, its 256:1 read mux and
            -- its 256-wide write decoder.
            upd_valid_r2  <= upd_valid_r;
            upd_idx_r2    <= upd_idx_r;
            upd_is_horiz_r2 <= upd_horiz_r;
            upd_ymin_r2 <= upd_ymin_r;
            if v_y_target_r = upd_ymin_r then
                upd_is_ymin_r2 <= '1'; else upd_is_ymin_r2 <= '0'; end if;
            if v_y_target_r >= upd_ymin_r and v_y_target_r <= upd_ymax_r then
                upd_active_r2 <= '1'; else upd_active_r2 <= '0'; end if;
            upd_xtop_r2   <= upd_xtop_r;
            upd_slope_r2  <= upd_slope_r;

            -- r2 -> r3, aligned with the fabric copy of cx_rd_data.  The
            -- slope is pre-extended to 18 bits here so Stage 1 is a bare add.
            upd_valid_r3    <= upd_valid_r2;
            upd_idx_r3      <= upd_idx_r2;
            upd_xtop_r3     <= upd_xtop_r2;
            upd_slope_r3    <= resize(upd_slope_r2, 18);
            upd_is_horiz_r3 <= upd_is_horiz_r2;
            upd_is_ymin_r3  <= upd_is_ymin_r2;
            upd_active_r3   <= upd_active_r2;

            -- Overrun guard (v1.0.3): an hsync that lands while a pass is
            -- still running is remembered; R_STAMP_SCAN then drops the rest of
            -- the active list and R_STAMP_DRAIN starts the next pass.  Before
            -- this the hsync was simply missed: the bank swap slipped a line
            -- and the next row's R_CLEAR walk (its DDA step and the ymin
            -- reload of edges starting there) never ran, kinking every edge
            -- through that row.  A pass that ends in time never sets it.
            if hsync_falling_r = '1' and raster_state /= R_IDLE
               and raster_state /= R_STAMP_DRAIN then
                hs_pend_r <= '1';
            end if;

            case raster_state is

                when R_IDLE =>
                    if hsync_falling_r = '1' then
                        lb_ab_r       <= not lb_ab_r;
                        raster_state  <= R_CLEAR;
                        raster_cycle  <= (others => '0');
                        al_wr_addr    <= (others => '0');  -- start a fresh active list
                        if lb_ab_r = '1' then bnd_max_a_r <= (others => '0');
                        else                  bnd_max_b_r <= (others => '0'); end if;
                    end if;

                when R_CLEAR =>
                    -- Write 0 across the head's bbox of BOTH buffers.
                    -- Grid fill is computed on the DISPLAY side via an
                    -- even-odd scan of the boundary line buffer.
                    lb_wr_en_bin_r  <= '1';
                    lb_wr_en_bout_r <= '1';
                    lb_wr_en_det_r  <= '1';
                    lb_wr_addr_r    <= clear_base_r + raster_cycle;
                    lb_wr_data_r    <= '0';
                    lb_bnd_data_r   <= '0';

                    -- Stage 0a: issue active mesh BRAM read for this
                    -- slot, register the slot's control bits.  Mesh
                    -- data lands in upd_*_r on the next cycle (in
                    -- Stage 0b above), 1 cycle BRAM read latency
                    -- aligned with the BRAM read for current_x and the
                    -- pipeline shift to upd_*_r2.
                    if raster_cycle < to_unsigned(C_NUM_EDGES, 11) then
                        act_rd_addr    <= raster_cycle(7 downto 0);
                        upd_valid_pre  <= '1';
                        upd_idx_pre    <= raster_cycle(7 downto 0);
                    else
                        upd_valid_pre <= '0';
                    end if;

                    if raster_cycle = clear_m1_r then
                        raster_state    <= R_WIPE;
                        raster_cycle    <= (others => '0');
                    else
                        raster_cycle <= raster_cycle + 1;
                    end if;

                when R_WIPE =>
                    -- Rolling clear of C_WIPE_LEN entries outside the head
                    -- window (before the stamps, so it can never erase them).
                    lb_wr_en_bin_r  <= '1';
                    lb_wr_en_bout_r <= '1';
                    lb_wr_en_det_r  <= '1';
                    lb_wr_data_r    <= '0';
                    lb_bnd_data_r   <= '0';
                    if lb_ab_r = '0' then
                        lb_wr_addr_r  <= wipe_ptr_a_r;
                        wipe_ptr_a_r  <= wipe_ptr_a_r + 1;
                    else
                        lb_wr_addr_r  <= wipe_ptr_b_r;
                        wipe_ptr_b_r  <= wipe_ptr_b_r + 1;
                    end if;
                    upd_valid_pre   <= '0';
                    -- (v1.0.3) 8 entries per line at SD, whose 858-cycle line
                    -- has no room for 32 (each bank is still fully wiped every
                    -- 512 lines, under two fields); 32 elsewhere, as before.
                    if raster_cycle(2 downto 0) = "111"
                       and (md_sd_r = '1' or raster_cycle(4 downto 3) = "11") then
                        -- Active list is fully built; iterate it.  al_count
                        -- snapshots the build pointer; al_rd_addr starts the
                        -- read so al_rd_data = list[0] arrives in SCAN2.
                        raster_state    <= R_STAMP_SCAN;
                        raster_cycle    <= (others => '0');
                        al_count        <= al_wr_addr;
                        al_rd_addr      <= (others => '0');
                        stamp_sub       <= (others => '0');
                    else
                        raster_cycle <= raster_cycle + 1;
                    end if;

                when R_STAMP_SCAN =>
                    -- Issue the list read for al_rd_addr (al_rd_data lands
                    -- next cycle); drain when the list is exhausted, or when
                    -- the next line's hsync has already come (overrun).
                    if al_rd_addr >= al_count or hs_pend_r = '1' then
                        raster_state <= R_STAMP_DRAIN;
                    else
                        raster_state <= R_STAMP_SCAN2;
                    end if;

                when R_STAMP_SCAN2 =>
                    -- al_rd_data holds this active edge's index; latch it
                    -- (issues its mesh/cur_x reads, settling during PRELOAD)
                    -- and enter the stamp pipeline.
                    latch_held(to_integer(unsigned(al_rd_data)));
                    raster_state <= R_STAMP_PRELOAD;

                when R_STAMP_PRELOAD =>
                    -- First wait cycle.  Drain any pending stamp from
                    -- Stage B so the last stamp of the previous edge
                    -- still lands.  bnd_held still holds the PREVIOUS
                    -- edge's routing here (it is only re-latched in
                    -- compute_counts_a, one state later), so no shadow
                    -- register is needed.  BRAM reads for cx_rd_data and
                    -- act_slope_rd complete during this cycle; their
                    -- outputs become visible to PRELOAD2 next cycle.
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(stamp_abs_r(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_det_r <= '1';
                    end if;
                    stamp_valid_r <= '0';
                    raster_state  <= R_STAMP_PRELOAD2;

                when R_STAMP_PRELOAD2 =>
                    -- act_slope_rd / act_ymin_rd / act_ymax_rd are valid
                    -- now; decode slope sign/magnitude/padding (stage a).
                    compute_counts_a;
                    raster_state <= R_STAMP_PRELOAD3;

                when R_STAMP_PRELOAD3 =>
                    -- Slope magnitude -> per-side thickness extension.
                    compute_counts_a2;
                    raster_state <= R_STAMP_PRELOAD4;

                when R_STAMP_PRELOAD4 =>
                    -- Finish the sweep parameters, and — in the same otherwise
                    -- idle write slot — emit this edge's single fill crossing
                    -- at its true x.  Decoupled from the sweep entirely, so
                    -- thickness, the stamp cap and the sweep clamp can never
                    -- drop it and unbalance the scanline's enter/leave pairs.
                    compute_counts_b;
                    if bnd_held = '1'
                       and cx_abs_r >= xlo_abs_r and cx_abs_r <= xhi_abs_r then
                        lb_wr_addr_r  <= unsigned(cx_abs_r(10 downto 0));
                        lb_bnd_data_r <= '1';
                        if bside_held = '1' then
                            lb_wr_en_bin_r  <= '1';   -- enters the fill
                        else
                            lb_wr_en_bout_r <= '1';   -- leaves it
                        end if;
                        if lb_ab_r = '0' then
                            if unsigned(cx_abs_r(10 downto 0)) > bnd_max_a_r then
                                bnd_max_a_r <= unsigned(cx_abs_r(10 downto 0));
                            end if;
                        else
                            if unsigned(cx_abs_r(10 downto 0)) > bnd_max_b_r then
                                bnd_max_b_r <= unsigned(cx_abs_r(10 downto 0));
                            end if;
                        end if;
                    end if;
                    raster_state <= R_STAMP;

                when R_STAMP =>
                    -- Stage A: one add.  ONE pair of compares does everything
                    -- else: the copy engine already folded the head bounding
                    -- box (+-hw_r, the region R_CLEAR wipes) into this edge's
                    -- own padded x-extent, so the two HEAD_HALF_W compares
                    -- that used to sit here are gone; and every list entry is
                    -- active by construction, so the active test is too.
                    v_stamp_abs := stamp_base_r + signed(resize(stamp_sub, 14));
                    stamp_abs_r <= v_stamp_abs;
                    if v_stamp_abs >= xlo_abs_r and v_stamp_abs <= xhi_abs_r then
                        stamp_valid_r <= '1';
                    else
                        stamp_valid_r <= '0';
                    end if;
                    -- Stage B: drain previous cycle's registered values
                    -- into the LB.  Every sweep pixel goes to the DETAIL
                    -- buffer (visible thickness).  The BOUNDARY buffer
                    -- (read by the EOR fill) gets only the single center
                    -- pixel, and only for boundary edges — so thickness
                    -- never corrupts the fill parity.
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(stamp_abs_r(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_det_r <= '1';
                    end if;

                    if stamp_sub = count_m1_held then
                        -- Advance to the next list entry; R_STAMP_SCAN reads
                        -- it (or drains when the list is exhausted).
                        stamp_sub <= (others => '0');
                        al_rd_addr <= al_rd_addr + 1;
                        raster_state <= R_STAMP_SCAN;
                    else
                        stamp_sub <= stamp_sub + 1;
                    end if;

                when R_STAMP_DRAIN =>
                    -- One extra cycle to flush the final stamp_abs_r
                    -- through Stage B.
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(stamp_abs_r(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_det_r <= '1';
                    end if;
                    stamp_valid_r <= '0';
                    raster_state <= R_IDLE;

                    -- If hsync hits before STAMP completes, restart anyway —
                    -- a partial bank is worse than a fresh one.
                    if hsync_falling_r = '1' or hs_pend_r = '1' then
                        hs_pend_r     <= '0';
                        lb_ab_r       <= not lb_ab_r;
                        raster_state  <= R_CLEAR;
                        raster_cycle  <= (others => '0');
                        al_wr_addr    <= (others => '0');  -- fresh active list
                        if lb_ab_r = '1' then bnd_max_a_r <= (others => '0');
                        else                  bnd_max_b_r <= (others => '0'); end if;
                    end if;

            end case;

            -- Stage 1 of R_CLEAR's edge-update pipeline.  Uses the
            -- DOUBLE-registered upd_*_r2 so cx_rd_data (1-cycle delayed
            -- from cx_rd_addr) lines up with the right edge.
            cx_wr_en <= '0';
            al_wr_en <= '0';
            if upd_valid_r3 = '1' then
                if upd_is_ymin_r3 = '1' then
                    -- x_top is already Q9.7 in the mesh package and the copy
                    -- engine pre-backed it up by pad_r*slope, so the ±pad_r
                    -- cap rows extrapolate the line correctly.
                    cx_wr_en   <= '1';
                    cx_wr_addr <= upd_idx_r3;
                    cx_wr_data <= std_logic_vector(resize(upd_xtop_r3, 18));
                elsif upd_active_r3 = '1' and upd_is_horiz_r3 = '0' then
                    -- Q9.7 add: per-row fractional accumulation.  Skipped
                    -- for horizontal edges — their slope field holds the
                    -- line WIDTH, so cur_x must stay parked on the left
                    -- end across all rows of the widened window.
                    cx_wr_en   <= '1';
                    cx_wr_addr <= upd_idx_r3;
                    cx_wr_data <= std_logic_vector(
                                    signed(cx_rd_data_f) + upd_slope_r3);
                end if;
                -- Append to the active list iff this scanline is inside the
                -- edge's (pad-widened) activation window.
                if upd_active_r3 = '1' then
                    al_wr_en   <= '1';
                    al_wr_data <= std_logic_vector(upd_idx_r3);
                end if;
            end if;

            -- Active-list BRAM: append on al_wr_en (advancing the build
            -- pointer), plus a continuous read so al_rd_data trails
            -- al_rd_addr by one cycle for the R_STAMP_SCAN/SCAN2 iterator.
            if al_wr_en = '1' then
                act_list_ram(to_integer(al_wr_addr)) <= al_wr_data;
                al_wr_addr <= al_wr_addr + 1;
            end if;
            al_rd_data <= act_list_ram(to_integer(al_rd_addr));
        end if;
    end process;

    -- Noise active whenever K6 spread is non-zero.
    noise_on <= '1' when noise_spread_r /= "00000" else '0';

    -- P12 head size active (disables the bit-exact static-slope bypass, since
    -- a horizontal edge's "slope" field is a WIDTH and does scale).
    -- (v1.0.3) any non-identity scale: P12, or a non-1080p mode.
    scale_on <= '1' when sc_mx_r /= 0 or sc_my_r /= 0 or yoff_r /= 0 else '0';

    -- Morph active when the mouth is not fully open, eyes are closed, or an
    -- expression other than pure neutral is selected.  Frame-constant; used to
    -- gate the slope recompute in CP_WR.
    morph_active <= '1' when mouth_frac_r /= "00000" or open_eyes_r = '0'
                         or expr_base_r /= "00" or expr_frac_r /= "00000"
                    else '0';

    -- ---------------------------------------------------------------
    -- current_x BRAM — replaces what was an N_EDGES-entry register
    -- file.  iCE40 EBR in 256×16 mode (we use 128 entries × 12 bits).
    -- 1-cycle read latency, lined up with the rest of Stage 0's
    -- registered outputs.
    -- ---------------------------------------------------------------
    cur_x_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if cx_wr_en = '1' then
                current_x_ram(to_integer(cx_wr_addr)) <= cx_wr_data;
            end if;
            cx_rd_data   <= current_x_ram(to_integer(cx_rd_addr));
            cx_rd_data_f <= cx_rd_data;
        end if;
    end process;

    -- Active mesh BRAMs (1W1R, 1-cycle read latency).  cp_proc writes
    -- once per slot during vblank; render Stage 0 reads continuously.
    act_ymin_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_ymin_ram(to_integer(act_wr_addr)) <= act_ymin_wr;
            end if;
            act_ymin_rd <= act_ymin_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_ymax_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_ymax_ram(to_integer(act_wr_addr)) <= act_ymax_wr;
            end if;
            act_ymax_rd <= act_ymax_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_xtop_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_xtop_ram(to_integer(act_wr_addr)) <= act_xtop_wr;
            end if;
            act_xtop_rd <= act_xtop_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_slope_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_slope_ram(to_integer(act_wr_addr)) <= act_slope_wr;
            end if;
            act_slope_rd <= act_slope_ram(to_integer(act_rd_addr));
        end if;
    end process;

    -- x-extent BRAMs, written by the copy engine in lockstep with the mesh
    -- BRAMs above and read at the same address / latency, so act_xlo_rd /
    -- act_xhi_rd line up with act_slope_rd in compute_counts_a.
    act_xlo_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_xlo_ram(to_integer(act_wr_addr)) <= act_xlo_wr;
            end if;
            act_xlo_rd <= act_xlo_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_xhi_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_xhi_ram(to_integer(act_wr_addr)) <= act_xhi_wr;
            end if;
            act_xhi_rd <= act_xhi_ram(to_integer(act_rd_addr));
        end if;
    end process;

    -- Morph-delta ROMs read at cp_addr (registered, 1-cycle latency).  cp_addr
    -- is stable across an edge's cp_proc states, so the reads are valid well
    -- before CP_MLOAD consumes them.
    -- emap read tracks the current edge (cp_addr); the 6 vrom reads track
    -- vrom_addr, which cp_proc points at the top then the bottom vertex slot.
    -- Static mesh ROMs (5 EBRs), read at cp_addr with 1 cycle of latency.
    mesh_rom_proc : process(clk)
    begin
        if rising_edge(clk) then
            rom_ymin_r  <= C_ROM_YMIN(to_integer(cp_addr(7 downto 0)));
            rom_ymax_r  <= C_ROM_YMAX(to_integer(cp_addr(7 downto 0)));
            rom_xtop_r  <= C_ROM_XTOP(to_integer(cp_addr(7 downto 0)));
            rom_slope_r <= C_ROM_SLOPE(to_integer(cp_addr(7 downto 0)));
            rom_xbot_r  <= C_ROM_XBOT(to_integer(cp_addr(7 downto 0)));
        end if;
    end process;

    morph_rom_proc : process(clk)
        variable va : integer range 0 to C_MORPH_NSLOT - 1;
    begin
        if rising_edge(clk) then
            emap_r <= C_EMAP_ROM(to_integer(cp_addr(7 downto 0)));
            va := to_integer(vrom_addr);
            vr_m <= C_VROM_M(va);  vr_e <= C_VROM_E(va);
            vr_h <= C_VROM_H(va);  vr_s <= C_VROM_S(va);
            vr_a <= C_VROM_A(va);  vr_u <= C_VROM_U(va);
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- P12 head size — its own sequencer, deliberately NOT folded into
    -- the copy engine's state machine.  Sharing cp_state pulled size_k_r and
    -- hw_r into the same next-state cone as the multiply accumulators, and
    -- nextpnr's HD critical path ran straight through the tangle.  Fourteen
    -- cycles after vsync is still far ahead of the first edge that needs
    -- sc_mx_r (~47 cycles) or hw_r (~85).
    --
    -- Full slider must put the 777-row mesh at ~95% of the ACTIVE picture, so
    -- the ceiling keys off act_h_r (lines that carried avid), NOT max_y_r
    -- (which includes vertical blanking and differs by mode).  1080 -> 83;
    -- the knob's top step is 63/64 of that = 81 -> 1.316x -> 1023 rows =
    -- 94.7%.  (v1.0.3) Every other mode is drawn in virtual 1080-line units,
    -- so it takes the same 83 and the same 1.0..1.316x range.
    --
    -- The scale factors: S = k*(1 + m/256) per axis, carried as
    -- 256*(S - 1) = (256 + m)*kq/256 - 256, kq = 256*k.  At 1080p kq = 256,
    -- so sc_mx_r = sc_my_r = m and hw = 258 + m, exactly as before.  The
    -- head half-width hw = 258 + m at 1080 becomes (256 + m)*kx + 2 (the
    -- mesh reaches |x| ~253, so the spare still covers padx + 1 px of DDA
    -- slack in every mode).
    -- ---------------------------------------------------------------
    size_proc : process(clk)
        variable v_px : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if vsync_falling_r = '1' then
                sz_step <= 0;
            elsif sz_step < 14 then
                sz_step <= sz_step + 1;
                case sz_step is
                    when 0 =>
                        if md_ident_r = '0' then
                            size_mmax_r <= to_unsigned(83, 7);
                        elsif act_h_r > to_unsigned(819, 12) then
                            size_mmax_r <= resize(shift_right(act_h_r, 2)
                                                + shift_right(act_h_r, 4)
                                                + shift_right(act_h_r, 9)
                                                - to_unsigned(256, 12), 7);
                        else
                            size_mmax_r <= (others => '0');
                        end if;
                        if field_bot_r = '1' then yoff_r <= md_yoffb_r;
                        else                      yoff_r <= md_yoff_r; end if;
                    when 1 =>
                        size_m_r <= resize(shift_right(size_k_r * size_mmax_r, 6), 7);
                    when 2 =>
                        -- arm the serial multiply: (256 + m) * kq, MSB first
                        sz_a_r    <= to_unsigned(256, 10) + resize(size_m_r, 10);
                        sz_kx_r   <= md_kxq_r;
                        sz_ky_r   <= md_kyq_r;
                        sz_accx_r <= (others => '0');
                        sz_accy_r <= (others => '0');
                    when 3 to 11 =>
                        -- one bit of kq per step (shift-add; never a multiplier)
                        if sz_kx_r(8) = '1' then
                            sz_accx_r <= shift_left(sz_accx_r, 1) + resize(sz_a_r, 18);
                        else
                            sz_accx_r <= shift_left(sz_accx_r, 1);
                        end if;
                        if sz_ky_r(8) = '1' then
                            sz_accy_r <= shift_left(sz_accy_r, 1) + resize(sz_a_r, 18);
                        else
                            sz_accy_r <= shift_left(sz_accy_r, 1);
                        end if;
                        sz_kx_r <= shift_left(sz_kx_r, 1);
                        sz_ky_r <= shift_left(sz_ky_r, 1);
                    when 12 =>
                        -- 256*(S - 1) per axis, and the head half-width
                        v_px    := sz_accx_r(17 downto 8);
                        sc_mx_r <= resize(signed('0' & v_px) - to_signed(256, 11), 9);
                        sc_my_r <= resize(signed('0' & sz_accy_r(17 downto 8))
                                          - to_signed(256, 11), 9);
                        hw_tgt_r <= v_px + to_unsigned(2, 10);
                    when others =>
                        -- hw_r rises instantly but DECAYS on the way down
                        -- (2 px/frame): the clear window must never shrink
                        -- below a region that still holds set pixels from a
                        -- wider frame, or they are stranded on screen.  A
                        -- video-mode change snaps it (the old geometry is
                        -- meaningless there, and the wipe clears the rest).
                        sz_cls_r <= md_cls_r;
                        if hw_tgt_r >= hw_r or md_cls_r /= sz_cls_r then
                            hw_r       <= hw_tgt_r;
                            clear_m1_r <= f_clear_m1(hw_tgt_r);
                            hw_lo_r    <= -signed(resize(hw_tgt_r, 13));
                            hw_hi_r    <= signed(resize(hw_tgt_r, 13))
                                          - to_signed(1, 13);
                        else
                            hw_r       <= hw_r - 2;
                            clear_m1_r <= f_clear_m1(hw_r - 2);
                            hw_lo_r    <= -signed(resize(hw_r - 2, 13));
                            hw_hi_r    <= signed(resize(hw_r - 2, 13))
                                          - to_signed(1, 13);
                        end if;
                end case;
            end if;
        end if;
    end process;

    -- Copy engine.  Walks edges 0..N_EDGES-1 in vblank, resolving the
    -- (expr, group, om, oe) variant select once per slot and writing
    -- to the active mesh BRAMs.  3-stage pipeline so the heavy
    -- LUT-ROM mux is isolated from the variant case mux which is
    -- isolated from the BRAM write — each stage shallow.
    cp_proc : process(clk)
        variable ixt, iyt, ixb, iyb : signed(13 downto 0);
        variable hxt, hyt, hxb, hyb : signed(16 downto 0);
        variable tt   : signed(16 downto 0);
        variable dx_v       : signed(18 downto 0);
        variable span_v     : signed(13 downto 0);
        variable spc        : integer range 1 to 255;
        variable newrem     : unsigned(8 downto 0);
        variable v_sl       : signed(18 downto 0);
        variable nslope     : signed(15 downto 0);
        variable xa18       : signed(17 downto 0);
        variable idx        : integer range 0 to 255;
        variable v_mid      : signed(13 downto 0);
        -- x-extent
        variable v_xa_i, v_xb_i, v_lo, v_hi : signed(12 downto 0);
        -- morph
        variable va_t, va_b, vb_t, vb_b : std_logic_vector(15 downto 0);
        variable v_py : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            act_wr_en <= '0';   -- default: no write this cycle

            case cp_state is
                when CP_IDLE =>
                    if vsync_falling_r = '1' then
                        cp_addr   <= (others => '0');
                        cp_done_r <= '0';
                        cp_state  <= CP_LOAD;
                    end if;

                -- Read the (static, single-variant) mesh for this edge.
                when CP_LOAD =>
                    -- The mesh ROM read for cp_addr is in flight this cycle.
                    if cp_done_r = '1' then
                        cp_state <= CP_IDLE;
                    else
                        cp_state <= CP_LOAD2;
                    end if;

                when CP_LOAD2 =>
                    w_ymin  <= signed(rom_ymin_r(12 downto 0));
                    w_ymax  <= signed(rom_ymax_r(12 downto 0));
                    w_xtop  <= signed(rom_xtop_r);
                    w_slope <= signed(rom_slope_r);
                    w_xbot  <= signed(rom_xbot_r);
                    bnd_wr_r   <= rom_ymin_r(13);
                    bside_wr_r <= rom_ymin_r(14);
                    if rom_ymin_r(12 downto 0) = rom_ymax_r(12 downto 0) then
                        horiz_r <= '1';
                    else
                        horiz_r <= '0';
                    end if;
                    cp_state <= CP_HASH;

                -- CP_HASH: per-endpoint coordinate hash (ix = x>>7 px).
                -- hash_x = ix*5 + iy*3 ; hash_y = iy*5 - ix*3 (distinct per
                -- axis -> each point traces a small Lissajous orbit).  Stored
                -- in w_ph*; the frame phase is added in the next state so this
                -- cycle's add chain stays shallow.
                when CP_HASH =>
                    ixt := resize(shift_right(w_xtop, 7), 14);  iyt := resize(w_ymin, 14);
                    ixb := resize(shift_right(w_xbot, 7), 14);  iyb := resize(w_ymax, 14);
                    hxt := resize((shift_left(ixt,2)+ixt) + (shift_left(iyt,1)+iyt), 17);
                    hyt := resize((shift_left(iyt,2)+iyt) - (shift_left(ixt,1)+ixt), 17);
                    hxb := resize((shift_left(ixb,2)+ixb) + (shift_left(iyb,1)+iyb), 17);
                    hyb := resize((shift_left(iyb,2)+iyb) - (shift_left(ixb,1)+ixb), 17);
                    w_phxt <= hxt;  w_phyt <= hyt;  w_phxb <= hxb;  w_phyb <= hyb;
                    cp_state <= CP_PHASE;

                -- CP_PHASE: add the per-frame phase (animation).
                when CP_PHASE =>
                    tt := signed(resize(noise_t_r, 17));
                    w_phxt <= resize(w_phxt + tt, 17);  w_phyt <= resize(w_phyt + tt, 17);
                    w_phxb <= resize(w_phxb + tt, 17);  w_phyb <= resize(w_phyb + tt, 17);
                    cp_state <= CP_TRI;

                -- CP_TRI: sample the triangle waves (kept shallow; the noise
                -- spread multiply is folded into the shared multiply loop).
                -- Also point vrom_addr at this edge's TOP vertex slot (emap_r is
                -- valid by now — emap tracks cp_addr, stable since CP_LOAD).
                when CP_TRI =>
                    t_trxt <= tri10(w_phxt);  t_tryt <= tri10(w_phyt);
                    t_trxb <= tri10(w_phxb);  t_tryb <= tri10(w_phyb);
                    vrom_addr <= unsigned(emap_r(13 downto 8));   -- top slot
                    cp_state  <= CP_MR0;

                -- CP_MR0..MR2: dependent ROM reads have 2-cycle latency (set
                -- addr -> +1 addr visible -> +1 data out).  MR0 switches the
                -- shared vrom_addr to the BOTTOM slot; MR1 latches the TOP-vertex
                -- deltas (now valid from the addr set in CP_TRI); MR2 latches the
                -- BOTTOM-vertex deltas.  mrd_*t/*b then feed CP_MLOAD as before.
                when CP_MR0 =>
                    vrom_addr <= unsigned(emap_r(5 downto 0));    -- bottom slot
                    cp_state  <= CP_MR1;
                when CP_MR1 =>
                    mrd_mt <= vr_m;  mrd_et <= vr_e;  mrd_ht <= vr_h;
                    mrd_st <= vr_s;  mrd_at <= vr_a;  mrd_ut <= vr_u;
                    cp_state <= CP_MR2;
                when CP_MR2 =>
                    mrd_mb <= vr_m;  mrd_eb <= vr_e;  mrd_hb <= vr_h;
                    mrd_sb <= vr_s;  mrd_ab <= vr_a;  mrd_ub <= vr_u;
                    cp_state <= CP_MLOAD;

                -- CP_MLOAD: unpack the per-edge morph deltas.  Mouth + eye are
                -- direct (eye gated by P8 closed); the two K4-adjacent expression
                -- deltas A and (B-A) are selected from the 4 expr ROMs by the
                -- blend base index.  All px (8-bit signed), packed (dy<<8|dx).
                when CP_MLOAD =>
                    m_mxt <= signed(mrd_mt(7 downto 0));  m_myt <= signed(mrd_mt(15 downto 8));
                    m_mxb <= signed(mrd_mb(7 downto 0));  m_myb <= signed(mrd_mb(15 downto 8));
                    case expr_base_r is
                        when "00"   => va_t := (others=>'0'); va_b := (others=>'0');
                                       vb_t := mrd_ht; vb_b := mrd_hb;
                        when "01"   => va_t := mrd_ht; va_b := mrd_hb; vb_t := mrd_st; vb_b := mrd_sb;
                        when "10"   => va_t := mrd_st; va_b := mrd_sb; vb_t := mrd_at; vb_b := mrd_ab;
                        when others => va_t := mrd_at; va_b := mrd_ab; vb_t := mrd_ut; vb_b := mrd_ub;
                    end case;
                    m_dxt <= resize(signed(vb_t(7 downto 0)),9)  - resize(signed(va_t(7 downto 0)),9);
                    m_dyt <= resize(signed(vb_t(15 downto 8)),9) - resize(signed(va_t(15 downto 8)),9);
                    m_dxb <= resize(signed(vb_b(7 downto 0)),9)  - resize(signed(va_b(7 downto 0)),9);
                    m_dyb <= resize(signed(vb_b(15 downto 8)),9) - resize(signed(va_b(15 downto 8)),9);
                    -- init the running total = exprA + eye (eye gated by P8 closed)
                    if open_eyes_r = '0' then
                        mt_xt <= resize(signed(va_t(7 downto 0)),11)  + resize(signed(mrd_et(7 downto 0)),11);
                        mt_yt <= resize(signed(va_t(15 downto 8)),11) + resize(signed(mrd_et(15 downto 8)),11);
                        mt_xb <= resize(signed(va_b(7 downto 0)),11)  + resize(signed(mrd_eb(7 downto 0)),11);
                        mt_yb <= resize(signed(va_b(15 downto 8)),11) + resize(signed(mrd_eb(15 downto 8)),11);
                    else
                        mt_xt <= resize(signed(va_t(7 downto 0)),11);
                        mt_yt <= resize(signed(va_t(15 downto 8)),11);
                        mt_xb <= resize(signed(va_b(7 downto 0)),11);
                        mt_yb <= resize(signed(va_b(15 downto 8)),11);
                    end if;
                    m_cnt    <= 0;
                    m_at11_r <= '0';
                    m_at15_r <= '0';
                    cp_state <= CP_MSEL;

                -- CP_MSEL: select this product's operands (mux only -> reg, so
                -- the mux stays off the multiply path).  One shared multiplier
                -- serves all three engines: 0..3 = noise tri*spread, 4..7 = expr
                -- blend t*(B-A), 8..11 = mouth frac*mouthΔ.
                when CP_MSEL =>
                    case m_cnt is
                        when 0  => mul_a_r <= resize(t_trxt,11);  when 1  => mul_a_r <= resize(t_tryt,11);
                        when 2  => mul_a_r <= resize(t_trxb,11);  when 3  => mul_a_r <= resize(t_tryb,11);
                        when 4  => mul_a_r <= resize(m_dxt,11);   when 5  => mul_a_r <= resize(m_dyt,11);
                        when 6  => mul_a_r <= resize(m_dxb,11);   when 7  => mul_a_r <= resize(m_dyb,11);
                        when 8  => mul_a_r <= resize(m_mxt,11);   when 9  => mul_a_r <= resize(m_myt,11);
                        when 10 => mul_a_r <= resize(m_mxb,11);   when 11 => mul_a_r <= resize(m_myb,11);
                        -- P12 scale.  x is Q11.7; only its INTEGER part is
                        -- multiplied — dropping the 1/128 fraction costs at
                        -- most half a pixel and, being a function of the
                        -- vertex's own x, it shifts both edges meeting at a
                        -- vertex identically, so joints stay closed.  y is
                        -- whole rows, biased to HEAD_Y_MID so the head grows
                        -- about its own centre instead of sliding off the
                        -- bottom of the picture.
                        when 12 => mul_a_r <= w_nxt(17 downto 7);
                        when 13 => mul_a_r <= w_nxb(17 downto 7);
                        when 14 => mul_a_r <= resize(w_nyt, 11) - to_signed(HEAD_Y_MID, 11);
                        when others => mul_a_r <= resize(w_nyb, 11) - to_signed(HEAD_Y_MID, 11);
                    end case;
                    -- (v1.0.3) 12/13 take the x scale, 14/15 the y scale
                    -- (both = size_m_r at 1080p).
                    if    m_cnt <= 3  then mul_b_r <= signed(resize(noise_spread_r, 9));
                    elsif m_cnt <= 7  then mul_b_r <= signed(resize(expr_frac_r, 9));
                    elsif m_cnt <= 11 then mul_b_r <= signed(resize(mouth_frac_r, 9));
                    elsif m_cnt <= 13 then mul_b_r <= sc_mx_r;
                    else                   mul_b_r <= sc_my_r;
                    end if;
                    -- expr (4..7) and mouth (8..11) share the mt_* accumulator.
                    mdst_r <= (others => '0');
                    if m_cnt < 4 then
                        mdst_r(m_cnt) <= '1';
                    elsif m_cnt < 12 then
                        mdst_r(4 + (m_cnt mod 4)) <= '1';
                    else
                        mdst_r(m_cnt - 4) <= '1';
                    end if;
                    cp_state <= CP_MDO;

                -- CP_MDO: the multiply, and NOTHING else (see prod_r).
                when CP_MDO =>
                    prod_r <= mul_a_r * mul_b_r;
                    cp_state <= CP_MACC;

                -- CP_MACC: route the product by m_cnt.  Noise (0..3) -> the
                -- offset o_d* (direct); morph (4..11) -> accumulate into the
                -- running total mt_*; scale (12..15) -> the sc_* deltas, which
                -- CP_SCALE adds back to the endpoint.
                when CP_MACC =>
                    -- y scale: the per-mode row offset (and the bottom-field
                    -- half row) goes in BEFORE the floor (0 at 1080p).
                    v_py := prod_r + resize(yoff_r, 20);
                    if mdst_r(0)  = '1' then o_dxt <= resize(shift_right(prod_r, 1), 18); end if;
                    if mdst_r(1)  = '1' then o_dyt <= resize(shift_right(prod_r, 8), 14); end if;
                    if mdst_r(2)  = '1' then o_dxb <= resize(shift_right(prod_r, 1), 18); end if;
                    if mdst_r(3)  = '1' then o_dyb <= resize(shift_right(prod_r, 8), 14); end if;
                    if mdst_r(4)  = '1' then mt_xt <= mt_xt + resize(shift_right(prod_r, 5), 11); end if;
                    if mdst_r(5)  = '1' then mt_yt <= mt_yt + resize(shift_right(prod_r, 5), 11); end if;
                    if mdst_r(6)  = '1' then mt_xb <= mt_xb + resize(shift_right(prod_r, 5), 11); end if;
                    if mdst_r(7)  = '1' then mt_yb <= mt_yb + resize(shift_right(prod_r, 5), 11); end if;
                    -- x: (hi*m)<<7 >>8 == (hi*m)>>1.  y: (dy*m)>>8.
                    if mdst_r(8)  = '1' then sc_xt <= resize(shift_right(prod_r, 1), 19); end if;
                    if mdst_r(9)  = '1' then sc_xb <= resize(shift_right(prod_r, 1), 19); end if;
                    if mdst_r(10) = '1' then sc_yt <= resize(shift_right(v_py, 8), 14); end if;
                    if mdst_r(11) = '1' then sc_yb <= resize(shift_right(v_py, 8), 14); end if;
                    if m_at11_r = '1' then
                        m_cnt <= 12;  m_at11_r <= '0';  cp_state <= CP_OFF;
                    elsif m_at15_r = '1' then
                        cp_state <= CP_SCALE;
                    else
                        m_cnt <= m_cnt + 1;
                        if m_cnt = 10 then m_at11_r <= '1'; end if;
                        if m_cnt = 14 then m_at15_r <= '1'; end if;
                        cp_state <= CP_MSEL;
                    end if;

                -- CP_OFF: move the endpoints by the noise offset AND the morph
                -- delta (x px<<7 to Q9.7, y px direct).  The P12 scale products
                -- (m_cnt 12..17) run next, over these moved endpoints, so the
                -- morph and noise amplitudes grow with the head.
                when CP_OFF =>
                    w_nxt <= resize(w_xtop,18) + o_dxt + shift_left(resize(mt_xt,18),7);
                    w_nyt <= resize(w_ymin,14) + o_dyt + resize(mt_yt,14);
                    w_nxb <= resize(w_xbot,18) + o_dxb + shift_left(resize(mt_xb,18),7);
                    w_nyb <= resize(w_ymax,14) + o_dyb + resize(mt_yb,14);
                    cp_state <= CP_MSEL;

                -- CP_SCALE: endpoint = endpoint + (endpoint * m)/256, i.e. the
                -- head scaled by (1 + m/256) about HEAD_Y_MID.  dx and dy scale
                -- together so the slope the divider recomputes below is
                -- unchanged — only the size is.
                when CP_SCALE =>
                    w_nxt <= w_nxt + resize(sc_xt, 18);
                    w_nxb <= w_nxb + resize(sc_xb, 18);
                    w_nyt <= w_nyt + resize(sc_yt, 14);
                    w_nyb <= w_nyb + resize(sc_yb, 14);
                    cp_state <= CP_SORT;

                -- CP_SORT: re-order endpoints by y (jitter may flip them).
                -- Just the compare + mux; the subtracts that used to chain off
                -- this in the same cycle were the HD critical path, so they now
                -- get their own (registered) states below.
                when CP_SORT =>
                    if w_nyt <= w_nyb then
                        w_ya <= w_nyt;  w_xa <= w_nxt;  w_yb <= w_nyb;  w_xb <= w_nxb;
                    else
                        w_ya <= w_nyb;  w_xa <= w_nxb;  w_yb <= w_nyt;  w_xb <= w_nxt;
                    end if;
                    cp_state <= CP_FLOOR;

                -- CP_FLOOR: when P9 "Noise" is OFF (Smooth), enforce a min
                -- vertical span (extend ymax) so an edge can't collapse flat
                -- and draw a wide horizontal streak — it renders as a gentle
                -- shallow diagonal instead.  Its own state to keep the compare
                -- off the divisor-setup critical path.
                when CP_FLOOR =>
                    if horiz_r = '1' and morph_active = '1' then
                        -- A neutral-horizontal edge tilted by a morph: snap both
                        -- ends to the mid row so it keeps ymin==ymax (is_horiz =>
                        -- Y-thicken + width-fill) and renders as a thick bar that
                        -- moved, not a thin diagonal.  span 0 -> slope = width.
                        v_mid := resize(shift_right(resize(w_ya, 15) + resize(w_yb, 15), 1), 14);
                        w_ya <= v_mid;  w_yb <= v_mid;
                    elsif noise_on = '1' and noise_streaks_r = '0'
                            and (w_yb - w_ya) < signed(resize(md_nfl_r, 14)) then
                        -- (v1.0.3) the floor is NOISE_SPAN_FLOOR rows at
                        -- 1080, in this mode's rows (md_nfl_r)
                        w_yb <= w_ya + signed(resize(md_nfl_r, 14));
                    end if;
                    cp_state <= CP_SPAN;

                -- CP_SPAN: span = yb-ya, clamped to [1,255] -> divisor.
                when CP_SPAN =>
                    span_v := w_yb - w_ya;
                    if    span_v <= 0  then spc := 1;
                    elsif span_v > 255 then spc := 255;
                    else  spc := to_integer(span_v); end if;
                    d_divisor <= to_unsigned(spc, 8);
                    cp_state  <= CP_DX;

                -- CP_DX: dx = xb-xa -> sign + |dx| (dividend); arm the divider.
                when CP_DX =>
                    dx_v := resize(w_xb, 19) - resize(w_xa, 19);
                    if dx_v < 0 then w_dxsign <= '1'; d_dividend <= unsigned(resize(-dx_v, 18));
                    else             w_dxsign <= '0'; d_dividend <= unsigned(resize( dx_v, 18));
                    end if;
                    d_rem    <= (others => '0');
                    d_quot   <= (others => '0');
                    d_cnt    <= 17;
                    d_last_r <= '0';
                    cp_state <= CP_DIV;

                -- Restoring division, one bit per cycle (shallow: a 9-bit
                -- compare + subtract).  18 steps -> |slope| in Q9.7.
                when CP_DIV =>
                    newrem := d_rem(7 downto 0) & d_dividend(17);
                    if newrem >= ('0' & d_divisor) then
                        d_rem  <= resize(newrem - ('0' & d_divisor), 9);
                        d_quot <= d_quot(16 downto 0) & '1';
                    else
                        d_rem  <= newrem;
                        d_quot <= d_quot(16 downto 0) & '0';
                    end if;
                    d_dividend <= d_dividend(16 downto 0) & '0';
                    if d_last_r = '1' then
                        cp_state <= CP_XEXT;
                    else
                        d_cnt <= d_cnt - 1;
                        if d_cnt = 1 then d_last_r <= '1'; end if;
                    end if;

                -- CP_XEXT: commit the slope (sign + saturate, or the bit-exact
                -- static value when nothing has moved the edge) and compute the
                -- x-extent the raster clamps its sweep to.  The extent is the
                -- edge's own x span (+/- 1 px of DDA slack) grown by pad_r for
                -- the stroke's cap, then clipped to the +-hw_r window R_CLEAR
                -- wipes — which is why the raster needs no separate bbox test.
                when CP_XEXT =>
                    if w_dxsign = '1' then v_sl := -signed('0' & d_quot);
                    else                   v_sl :=  signed('0' & d_quot);
                    end if;
                    if    v_sl > 32767  then nslope := to_signed(32767, 16);
                    elsif v_sl < -32768 then nslope := to_signed(-32768, 16);
                    else  nslope := resize(v_sl, 16); end if;
                    -- The static slope is only bit-exact while the mesh is
                    -- untouched.  A horizontal edge's "slope" field is its
                    -- WIDTH, which DOES scale with P12, so size must disable
                    -- the bypass alongside noise and morphs.
                    if noise_on = '1' or morph_active = '1' or scale_on = '1' then
                        slope_sel_r <= nslope;
                    else
                        slope_sel_r <= w_slope;
                    end if;

                    -- Sort the endpoints in x (one compare).  The pad and
                    -- the +-hw_r clamp are a separate state: chaining sort ->
                    -- pad -> clamp put four 13-bit carry chains in one cycle
                    -- and that became the HD critical path.
                    v_xa_i := resize(shift_right(w_xa, 7), 13);
                    v_xb_i := resize(shift_right(w_xb, 7), 13);
                    if v_xa_i <= v_xb_i then
                        act_xlo_wr <= std_logic_vector(v_xa_i);
                        act_xhi_wr <= std_logic_vector(v_xb_i);
                    else
                        act_xlo_wr <= std_logic_vector(v_xb_i);
                        act_xhi_wr <= std_logic_vector(v_xa_i);
                    end if;
                    cp_state <= CP_XEXT2;

                -- CP_XEXT2: grow the span by pad_r (+1 px of DDA slack) for
                -- the stroke's cap, then clip to the +-hw_r window R_CLEAR
                -- wipes, so the raster needs no separate bbox test.
                when CP_XEXT2 =>
                    v_lo := signed(act_xlo_wr) - signed(resize(padx_r, 13))
                            - to_signed(1, 13);
                    v_hi := signed(act_xhi_wr) + signed(resize(padx_r, 13))
                            + to_signed(1, 13);
                    if v_lo < hw_lo_r then v_lo := hw_lo_r; end if;
                    if v_hi > hw_hi_r then v_hi := hw_hi_r; end if;
                    act_xlo_wr <= std_logic_vector(v_lo);
                    act_xhi_wr <= std_logic_vector(v_hi);
                    cp_state <= CP_XT;

                -- CP_XT: back cur_x's load value up by pad_r rows of slope, so
                -- that when the raster activates the edge pad_r rows early (the
                -- cap) the DDA is already on the line's extrapolation.
                -- Horizontal edges park cur_x, so they are left alone.
                when CP_XT =>
                    xa18 := resize(w_xa, 18);
                    -- Skip the back-up for horizontals (cur_x is parked) and
                    -- for very shallow edges (|slope| > 32 px/row): pad*slope
                    -- would overflow the 16-bit field, and their cap rows are
                    -- clipped hard by [xlo, xhi] anyway.
                    if horiz_r = '1' or w_ya = w_yb
                       or slope_sel_r > to_signed(4096, 16)
                       or slope_sel_r < to_signed(-4096, 16) then
                        xtop_ext_r <= xa18;
                    else
                        case pady_r is
                            when "00"   => xtop_ext_r <= xa18;
                            when "01"   => xtop_ext_r <= xa18
                                       - resize(slope_sel_r, 18);
                            when "10"   => xtop_ext_r <= xa18
                                       - shift_left(resize(slope_sel_r, 18), 1);
                            when others => xtop_ext_r <= xa18
                                       - shift_left(resize(slope_sel_r, 18), 1)
                                       - resize(slope_sel_r, 18);
                        end case;
                    end if;
                    cp_state <= CP_WR;

                -- Write the active mesh BRAM.  bnd rides bit 13 of the ymin
                -- word so the render never has to look it up in a ROM.
                when CP_WR =>
                    act_wr_en   <= '1';
                    act_wr_addr <= cp_addr(7 downto 0);
                    act_ymin_wr <= bside_wr_r & bnd_wr_r
                                   & std_logic_vector(resize(w_ya, 13));
                    act_ymax_wr <= "00" & std_logic_vector(resize(w_yb, 13));
                    act_xtop_wr <= std_logic_vector(xtop_ext_r);
                    act_slope_wr <= std_logic_vector(slope_sel_r);
                    cp_addr  <= cp_addr + 1;
                    if cp_addr + 1 >= to_unsigned(C_NUM_EDGES, 9) then
                        cp_done_r <= '1';
                    end if;
                    cp_state <= CP_LOAD;
            end case;
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- Inlined dual-bank 1-bit line buffers (4 iCE40 EBRs total).
    -- bnd buffer carries boundary stamps + clears; the EOR walker
    -- reads it.  det buffer carries detail stamps + clears; the
    -- display ORs in det so brows/centerlines are visible without
    -- breaking EOR parity.
    -- ---------------------------------------------------------------
    lb_rd_addr <= pixel_x(10 downto 0);

    lb_bin_a_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_bin_r = '1' and lb_ab_r = '0' then
                lb_bin_a(to_integer(lb_wr_addr_r)) <= lb_bnd_data_r;
            end if;
            lb_bin_rd_a_r <= lb_bin_a(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_bin_b_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_bin_r = '1' and lb_ab_r = '1' then
                lb_bin_b(to_integer(lb_wr_addr_r)) <= lb_bnd_data_r;
            end if;
            lb_bin_rd_b_r <= lb_bin_b(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_bout_a_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_bout_r = '1' and lb_ab_r = '0' then
                lb_bout_a(to_integer(lb_wr_addr_r)) <= lb_bnd_data_r;
            end if;
            lb_bout_rd_a_r <= lb_bout_a(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_bout_b_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_bout_r = '1' and lb_ab_r = '1' then
                lb_bout_b(to_integer(lb_wr_addr_r)) <= lb_bnd_data_r;
            end if;
            lb_bout_rd_b_r <= lb_bout_b(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_det_a_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_det_r = '1' and lb_ab_r = '0' then
                lb_det_a(to_integer(lb_wr_addr_r)) <= lb_wr_data_r;
            end if;
            lb_det_rd_a_r <= lb_det_a(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_det_b_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_det_r = '1' and lb_ab_r = '1' then
                lb_det_b(to_integer(lb_wr_addr_r)) <= lb_wr_data_r;
            end if;
            lb_det_rd_b_r <= lb_det_b(to_integer(lb_rd_addr));
        end if;
    end process;

    -- ab_d1 captures the bank that was active when the read was issued
    lb_ab_d1_proc : process(clk)
    begin
        if rising_edge(clk) then
            lb_ab_d1 <= lb_ab_r;
        end if;
    end process;

    -- ab='0' means writing A → display reads B (and vice-versa)
    bnd_max_disp   <= bnd_max_a_r when lb_ab_d1 = '1' else bnd_max_b_r;
    lb_bnd_rd_data <= (lb_bout_rd_a_r & lb_bin_rd_a_r) when lb_ab_d1 = '1'
                      else (lb_bout_rd_b_r & lb_bin_rd_b_r);
    lb_any_rd_data <= (lb_bin_rd_a_r or lb_bout_rd_a_r or lb_det_rd_a_r)
                      when lb_ab_d1 = '1'
                      else (lb_bin_rd_b_r or lb_bout_rd_b_r or lb_det_rd_b_r);

    -- ---------------------------------------------------------------
    -- Post-line-buffer pipeline
    -- ---------------------------------------------------------------
    -- Luma is pre-scaled at vsync, so the old per-pixel pal_y * brightness
    -- multiply is gone from the render path entirely.
    pix_proc : process(clk)
    begin
        if rising_edge(clk) then
            edge_hit_r <= lb_any_rd_data;
            pix_y_r  <= pal_y_r;
            pix_yf_r <= pal_yf_r;
            pix_u_r <= pal_u_r;
            pix_v_r <= pal_v_r;
        end if;
    end process;

    -- Even-odd fill walker.  Each boundary edge writes exactly one 1px crossing
    -- per row (the centre stamp; horizontal edges and the half-open bottom row
    -- are excluded), so walking left-to-right and flipping "inside" on every set
    -- pixel is a clean even-odd scan — no rising-edge or run-cancel needed.
    eor_proc : process(clk)
    begin
        if rising_edge(clk) then
            -- Past the row's rightmost crossing? (registered, so it carries the
            -- same 1-cycle delay as eor_inside_d_r).
            if pixel_x > resize(bnd_max_disp, 12) then
                past_max_r <= '1';
            else
                past_max_r <= '0';
            end if;

            if hsync_falling_r = '1' then
                eor_inside_r   <= '0';
                eor_inside_d_r <= '0';
            else
                -- SET/RESET, not toggle.  Two crossings landing on the same
                -- pixel (a local-minimum vertex, which a morph can create
                -- anywhere on the lip loop) collapse to one set bit in the
                -- buffer; a toggle then has odd parity and floods the rest of
                -- the scanline.  An explicit enter/leave pair is idempotent,
                -- so the collapse costs at most a one-pixel sliver.  "Leave"
                -- wins a tie, which always errs towards NOT filling.
                if lb_bnd_rd_data(1) = '1' then
                    eor_inside_r <= '0';
                elsif lb_bnd_rd_data(0) = '1' then
                    eor_inside_r <= '1';
                end if;
                -- Delay the "inside" flag by one cycle so it lines up with
                -- edge_hit_r at the output mux, and drop it past the row's last
                -- crossing (past_max_r carries the same 1-cycle delay).
                eor_inside_d_r <= eor_inside_r and not past_max_r;
            end if;
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- Output stage
    -- ---------------------------------------------------------------
    -- The core's 444->422 packer takes U from the EVEN avid-relative pixel
    -- of each pair and V from the ODD one.  An isolated 1-px stroke pixel
    -- therefore reaches the encoder with only ONE of its chroma axes and
    -- its hue swings to that axis (blue strokes read red/yellow on
    -- hardware at 1 px; thicker lines fill whole pairs and hide it).  Fix:
    -- both pixels of a pair carry the PAIR-AVERAGED chroma (alpha = hits
    -- in the pair, 0/1/2), so whichever axis the packer samples the pair's
    -- chroma vector is exact.  Luma stays per-pixel (4:4:4).  The even
    -- pixel needs one pixel of lookahead, so the sync/video pipe is one
    -- stage deeper than the show path, plus one for the output register.
    --
    -- Alignment: show_pix (from edge_hit_r / eor_inside_d_r) goes with
    -- pipe(LATENCY-3); show_d1_r with pipe(LATENCY-2), the pixel this
    -- stage computes; the result registers out alongside pipe(LATENCY-1).
    -- out_phase_r mirrors the packer's phase counter on pipe(LATENCY-2).
    --
    -- Blanking gate: outside avid the output is neutral (y 64, u/v 512)
    -- in every mode -- the encoder takes its colour reference from
    -- blanking.  Black inside the picture is 64, not 0.  The palette is
    -- authored in BT.601; the hardware wants Cr in U and Cb in V, so the
    -- drawn colour is swapped here (and only here -- the video passthrough
    -- is a dry path).  T10 Invert: the lines go black and the black
    -- background becomes the palette colour at 3/4 luma (over Video it is
    -- just black ink).
    show_pix <= edge_hit_r or (fill_on_r and eor_inside_d_r);

    out_proc : process(clk)
        variable v_alpha  : unsigned(1 downto 0);
        variable v_line_y, v_line_u, v_line_v : unsigned(9 downto 0);
        variable v_bg_y,   v_bg_u,   v_bg_v   : unsigned(9 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_sum_u, v_sum_v : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            show_d1_r <= show_pix;
            show_d2_r <= show_d1_r;

            -- Packer phase mirror: 0 at the first active pixel, toggles per
            -- active pixel, held at 0 through blanking.
            if pipe(LATENCY - 2).avid = '1' then
                out_phase_r <= not out_phase_r;
            else
                out_phase_r <= '0';
            end if;

            -- Hits in this pixel's 422 pair.
            if out_phase_r = '0' then
                v_alpha := ('0' & show_d1_r) + ('0' & show_pix);    -- even: me + next
            else
                v_alpha := ('0' & show_d2_r) + ('0' & show_d1_r);   -- odd: prev + me
            end if;

            -- Line and background colours for this frame's switches.
            if invert_r = '1' then
                v_line_y := C_BLACK;  v_line_u := C_CHROMA_MID;  v_line_v := C_CHROMA_MID;
                if bg_video_r = '1' then
                    v_bg_y := unsigned(pipe(LATENCY - 2).y);
                    v_bg_u := unsigned(pipe(LATENCY - 2).u);
                    v_bg_v := unsigned(pipe(LATENCY - 2).v);
                else
                    v_bg_y := pix_yf_r;  v_bg_u := pix_v_r;  v_bg_v := pix_u_r;
                end if;
            else
                v_line_y := pix_y_r;  v_line_u := pix_v_r;  v_line_v := pix_u_r;
                if bg_video_r = '1' then
                    v_bg_y := unsigned(pipe(LATENCY - 2).y);
                    v_bg_u := unsigned(pipe(LATENCY - 2).u);
                    v_bg_v := unsigned(pipe(LATENCY - 2).v);
                else
                    v_bg_y := C_BLACK;  v_bg_u := C_CHROMA_MID;  v_bg_v := C_CHROMA_MID;
                end if;
            end if;

            -- Luma per pixel; chroma by pair occupancy.
            if show_d1_r = '1' then v_y := v_line_y; else v_y := v_bg_y; end if;
            v_sum_u := resize(v_bg_u, 11) + resize(v_line_u, 11);
            v_sum_v := resize(v_bg_v, 11) + resize(v_line_v, 11);
            case v_alpha is
                when "00"   => v_u := v_bg_u;               v_v := v_bg_v;
                when "10"   => v_u := v_line_u;             v_v := v_line_v;
                when others => v_u := v_sum_u(10 downto 1); v_v := v_sum_v(10 downto 1);
            end case;

            if pipe(LATENCY - 2).avid = '0' then
                out_y_r <= C_BLACK;  out_u_r <= C_CHROMA_MID;  out_v_r <= C_CHROMA_MID;
            else
                out_y_r <= v_y;  out_u_r <= v_u;  out_v_r <= v_v;
            end if;
        end if;
    end process;

    s_io_align_in.hsync_n <= pipe(LATENCY - 1).hsync_n;
    s_io_align_in.vsync_n <= pipe(LATENCY - 1).vsync_n;
    s_io_align_in.field_n <= pipe(LATENCY - 1).field_n;
    s_io_align_in.avid    <= pipe(LATENCY - 1).avid;
    s_io_align_in.y <= std_logic_vector(out_y_r);
    s_io_align_in.u <= std_logic_vector(out_u_r);
    s_io_align_in.v <= std_logic_vector(out_v_r);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (5 -> 8).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 2 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(2);

end architecture bishop;
