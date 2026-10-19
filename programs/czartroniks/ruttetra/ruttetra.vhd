-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ruttetra.vhd - Classic Rutt-Etra scan processor for Videomancer
--
-- Incoming video is re-rendered as a field of horizontal scanlines whose
-- vertical position is displaced by the picture's luma (the classic
-- Rutt/Etra "Z-displacement" look): bright material rises out of a sagging
-- dark floor, drawn as glowing lines on black, with optional hidden-line
-- terrain occlusion (each nearer line drops an opaque black skirt that
-- hides the lines behind it).
--
-- Engine
--   Every Nth active row is a "sampled" row.  As it streams in, its luma
--   (after contrast + horizontal smoothing) is converted to a displacement
--   (sag, in rows, applied DOWNWARD from the source row) and a 4-bit
--   brightness, and written column-by-column (columns decimated x2) into
--   one of 8 rotating BRAM banks (1024 x 12 each; 12-bit word packs
--   disp[7:0] & bright[3:0], mapping to 3x 1024x4 EBR per bank, 24 EBR
--   total).  A displaced line lands on rows BELOW its source row, so at
--   display time the 8 banks hold every sampled row whose line could pass
--   through the current pixel: 8 parallel candidate tests (age_k - disp_k
--   against the stroke window) find the hits, and a recency-priority
--   resolve picks the nearest (lowest source row) - which is also the
--   hidden-line rule.  Max displacement = 8 x spacing - 1 (capped 255), so
--   denser line fields have proportionally shallower relief.
--
--   Because displacement is one-sided (down), "Peaks Bright" inverts luma
--   into the sag term: bright stays near its source row, dark sags away.
--
--   Connected strokes: each display pixel draws the segment from the
--   previous half-column's height to its own (midpoint subdivision of the
--   x2-decimated columns), so steep relief stays a continuous line; the
--   connecting segments are dimmed by their vertical extent (a real beam
--   sweeping fast deposits less light).  Relief is normalised to the
--   retrievable window (8 x spacing), so the slider sweeps the same range at
--   every line density.  The smoothing IIR's group delay (2^s - 1 px) is
--   cancelled by reading the stored rows 2^s px ahead (clamped at the
--   measured line end) - free, since those rows are already complete.
--
-- Controls
--   Pot 1  (registers_in(0))    : Scan Lines - line density (CW = denser)
--   Pot 2  (registers_in(1))    : Thickness  - stroke height 1..8 rows;
--                                 a graded glow halo fades in from T=3
--   Pot 3  (registers_in(2))    : Line Color - White/Green/Amber/Cyan/
--                                 Magenta/Video/Fire/Terrain (luma tint)
--   Pot 4  (registers_in(3))    : Contrast   - input gain into disp+shade
--   Pot 5  (registers_in(4))    : Roll       - bipolar line-grid scroll
--                                 (centre still; CW toward, CCW away)
--   Pot 6  (registers_in(5))    : Smoothing  - horizontal IIR on the
--                                 profile (raw nervous edges -> flowing)
--   Tog 7  (registers_in(6)(0)) : Peaks      - Bright rises / Dark rises
--   Tog 8  (registers_in(6)(1)) : Hidden Line- terrain skirts on/off
--   Tog 9  (registers_in(6)(2)) : Shading    - Banded (2-level) / Luma
--   Tog 10 (registers_in(6)(3)) : Roll       - Run / Freeze (holds the
--                                 grid where it is; Run resumes from there)
--   Tog 11 (registers_in(6)(4)) : Input      - Positive / Negative
--   Fader  (registers_in(7))    : RELIEF     - displacement depth (glided)
--
-- Latency: 15 clocks, identical in every mode.  Interlace: the terrain is
-- rebuilt per field (banks invalidated at vsync, idempotent under
-- serration); ages advance on active lines only.
--
-- Resolution scaling (v0.2.4): all vertical geometry -- line spacing (8..71),
-- bank ages, displacement (<= 255), stroke thickness and roll -- is counted in
-- VIRTUAL 1080-line rows.  Each active field row advances the ages by the
-- virtual rows it spans (y step Q4.8 from the timing ID, x2 for interlace:
-- 1 at 1080p, 1-2 at 720p, 4-5 at 480i), so SD/720p show the same number of
-- scan lines and the same relief as 1080p.  The stroke window is at least
-- ceil(y step) rows thick so a thin line still lands on a field row.  1080p
-- is bit-identical to v0.2.3.  Horizontal work stays pixel scale (the
-- picture itself sets the profile; smoothing is a filter kernel).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;


architecture ruttetra of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (15 + 1 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 15;      -- display pipeline registers
    constant NB    : integer := 8;       -- sampled-row banks
    constant C_CDLY : integer := 10;     -- input-chroma delay (Video colour)

    attribute keep : boolean;

    ----------------------------------------------------------------------
    -- Latched controls (at vsync, quilt-style)
    ----------------------------------------------------------------------
    signal s_dspace : unsigned(6 downto 0) := to_unsigned(39, 7);  -- 8..71
    signal s_dmax   : unsigned(7 downto 0) := to_unsigned(255, 8);
    -- relief normaliser: min(spacing, 32) (32 = unity; window = 8 x spacing)
    signal s_rmul   : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_thick  : unsigned(3 downto 0) := to_unsigned(2, 4);   -- 1..8
    -- stroke-window thickness: max(T, ceil(y step)) so a 1-unit stroke still
    -- lands on a field row when rows are several virtual rows apart (SD)
    signal s_thkw   : unsigned(3 downto 0) := to_unsigned(2, 4);
    -- thin strokes integrate less light: small shading lift at T=1..3
    signal s_bboost : unsigned(1 downto 0) := "10";
    signal s_zone   : unsigned(2 downto 0) := (others => '0');
    -- K3 label decode, free-running (kept out of the per-frame latch).
    -- Labelled default: at program load the firmware writes the
    -- initial_value_label's INDEX (0..7), not a 0..1023 position, so a value
    -- below 8 is the label index itself.
    signal s_k3_idx : unsigned(2 downto 0) := (others => '0');
    signal s_gain8  : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_smooth : unsigned(2 downto 0) := "010";
    -- smoothing lag compensation: read 2^s px ahead (IIR group delay
    -- 2^s - 1, plus the half-pixel of the connected-stroke segment)
    signal s_lagp1  : unsigned(7 downto 0) := to_unsigned(4, 8);
    signal s_scale9 : unsigned(8 downto 0) := to_unsigned(8, 9);   -- glided
    signal s_scale_eff : unsigned(8 downto 0) := to_unsigned(8, 9);
    signal s_peakdark : std_logic := '0';
    signal s_hlr      : std_logic := '1';
    signal s_shade    : std_logic := '1';
    -- K2 glow: halo on from T=3; inner-half shift 2 (T3-4) / 1 (T5-8),
    -- outer half one step dimmer; s_half = ceil(T/2)
    signal s_glow     : std_logic := '0';
    signal s_gsh      : unsigned(2 downto 0) := "010";
    signal s_half     : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal s_rfreeze  : std_logic := '0';
    signal s_neg      : std_logic := '0';

    -- K5 Roll: line-grid phase in units of one line spacing (wraps free)
    signal s_rdir   : std_logic := '0';
    signal s_ru     : unsigned(4 downto 0) := (others => '0');
    signal s_rspd   : unsigned(13 downto 0) := (others => '0');
    signal s_rphase : unsigned(15 downto 0) := (others => '0');
    signal s_roff   : unsigned(6 downto 0) := (others => '0');
    signal s_fade4  : unsigned(3 downto 0) := (others => '0');
    signal s_firstrow : std_logic := '1';
    signal s_rowtop   : std_logic := '0';

    ----------------------------------------------------------------------
    -- Position + row bookkeeping
    ----------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    -- display read position = pixel_x + 2^s, held at the line's last column
    signal px_rd        : unsigned(11 downto 0) := (others => '0');
    signal s_wline      : unsigned(9 downto 0) := to_unsigned(959, 10);
    signal s_wlast      : unsigned(9 downto 0) := to_unsigned(959, 10);
    signal s_row_armed  : std_logic := '1';
    signal s_saw_frame  : std_logic := '0';
    -- Resolution scaling (v0.2.4): rows are counted in virtual 1080-line
    -- units.  s_ys = y step per FIELD row (Q4.8, x2 for interlace, from the
    -- timing ID); s_yf its fractional accumulator over active rows; s_ka the
    -- virtual rows the NEXT active row advances (1 at 1080p); s_kc =
    -- ceil(y step).  s_spacing = virtual rows to the next sampled row,
    -- biased +1 (first test after subtracting this row's advance).
    signal s_ys         : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_yf         : unsigned(7 downto 0) := (others => '0');
    signal s_ka         : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal s_kc         : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal s_spacing    : signed(8 downto 0) := (others => '0');
    -- the row-start decision, precomputed every clock (s_spacing and s_ka
    -- only change at a row start / vsync, rows are >= 858 clocks apart):
    -- keeps the subtract off the row-start -> age-enable path
    signal s_samp_nx    : std_logic := '0';
    signal s_sp_nx      : signed(8 downto 0) := (others => '0');
    signal s_sampled    : std_logic := '0';
    signal s_head       : unsigned(2 downto 0) := (others => '0');

    type t_b10 is array (0 to NB - 1) of unsigned(9 downto 0);
    type t_b8  is array (0 to NB - 1) of unsigned(7 downto 0);
    type t_b4  is array (0 to NB - 1) of unsigned(3 downto 0);
    type t_b3  is array (0 to NB - 1) of unsigned(2 downto 0);
    type t_b2  is array (0 to NB - 1) of unsigned(1 downto 0);
    type t_b11s is array (0 to NB - 1) of signed(10 downto 0);

    signal s_age   : t_b10 := (others => (others => '0'));
    signal s_valid : std_logic_vector(NB - 1 downto 0) := (others => '0');
    -- valid AND age /= 0 (the bank being written this row is masked: iCE40
    -- EBR read-during-write returns undefined data)
    signal s_live  : std_logic_vector(NB - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Write path (profile capture on sampled rows)
    ----------------------------------------------------------------------
    signal w0_y    : unsigned(9 downto 0) := (others => '0');
    signal w1_prod : unsigned(17 downto 0) := (others => '0');
    signal w2_acc  : unsigned(13 downto 0) := (others => '0');
    signal w3_prod : unsigned(14 downto 0) := (others => '0');
    signal w3_brt  : unsigned(3 downto 0) := (others => '0');
    signal w4_data : std_logic_vector(11 downto 0) := (others => '0');
    signal w4_addr : unsigned(9 downto 0) := (others => '0');
    signal w4_we   : std_logic := '0';

    type t_wapipe is array (0 to 3) of unsigned(9 downto 0);
    signal w_addr : t_wapipe := (others => (others => '0'));
    signal w_av   : std_logic_vector(3 downto 0) := (others => '0');
    signal w_par  : std_logic_vector(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Read-side position flags (aligned with the bank read pipeline)
    ----------------------------------------------------------------------
    signal r_ph    : std_logic_vector(3 downto 1) := (others => '0');
    signal r_first : std_logic_vector(3 downto 1) := (others => '0');

    ----------------------------------------------------------------------
    -- Bank read data (A2 fabric registers, after the EBR output register)
    ----------------------------------------------------------------------
    signal a2_disp : t_b8 := (others => (others => '0'));
    signal a2_brt  : t_b4 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- Connected-stroke segment (B1..B3)
    ----------------------------------------------------------------------
    signal p_d1    : t_b8 := (others => (others => '0'));  -- previous pixel
    signal s_pc    : t_b8 := (others => (others => '0'));  -- previous column
    signal b1_prev : t_b8 := (others => (others => '0'));
    signal b1_d    : t_b8 := (others => (others => '0'));
    signal b1_ph   : std_logic := '0';
    signal b1_brt  : t_b4 := (others => (others => '0'));
    signal b2_ea   : t_b8 := (others => (others => '0'));
    signal b2_eb   : t_b8 := (others => (others => '0'));
    signal b2_brt  : t_b4 := (others => (others => '0'));
    signal b3_lo   : t_b8 := (others => (others => '0'));
    signal b3_hi   : t_b8 := (others => (others => '0'));
    signal b3_span : t_b8 := (others => (others => '0'));
    signal b3_brt  : t_b4 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- Display pipeline
    ----------------------------------------------------------------------
    signal a3_slo  : t_b11s := (others => (others => '0'));
    signal a3_shi  : t_b11s := (others => (others => '0'));
    signal a3_dim  : t_b2 := (others => (others => '0'));
    signal a3_brt  : t_b4 := (others => (others => '0'));

    signal a4_hit  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_cov  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_sh   : t_b3 := (others => (others => '0'));  -- dim shift 0..4
    signal a4_brt  : t_b4 := (others => (others => '0'));

    signal a5_rhit : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_rcov : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_sh   : t_b3 := (others => (others => '0'));
    signal a5_brt  : t_b4 := (others => (others => '0'));

    signal a6_hidx   : unsigned(2 downto 0) := (others => '0');
    signal a6_cidx   : unsigned(2 downto 0) := (others => '0');
    signal a6_anyhit : std_logic := '0';
    signal a6_anycov : std_logic := '0';
    signal a6_sh     : t_b3 := (others => (others => '0'));
    signal a6_brt    : t_b4 := (others => (others => '0'));

    signal a7_line  : std_logic := '0';
    signal a7_sh    : unsigned(2 downto 0) := (others => '0');
    signal a7_brt   : unsigned(3 downto 0) := (others => '0');
    signal a7_cu    : signed(9 downto 0) := (others => '0');
    signal a7_cv    : signed(9 downto 0) := (others => '0');

    signal a8_yo    : unsigned(9 downto 0) := (others => '0');
    signal a8_uo    : signed(9 downto 0) := (others => '0');
    signal a8_vo    : signed(9 downto 0) := (others => '0');
    signal a8_bf1   : unsigned(4 downto 0) := (others => '0');
    signal a8_line  : std_logic := '0';

    signal a9_ym    : unsigned(14 downto 0) := (others => '0');
    signal a9_um    : signed(15 downto 0) := (others => '0');
    signal a9_vm    : signed(15 downto 0) := (others => '0');
    signal a9_line  : std_logic := '0';

    signal a10_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a10_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a10_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal a11_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a11_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a11_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Sync delay line (tap 13 gates blanking, tap 14 drives sync out) and
    -- the input-chroma delay feeding the Video line colour at A7
    ----------------------------------------------------------------------
    type t_dly is record
        avid    : std_logic;
        hsync_n : std_logic;
        vsync_n : std_logic;
        field_n : std_logic;
    end record;
    type t_dly_arr is array (0 to C_LAT - 1) of t_dly;
    signal s_dly : t_dly_arr := (others =>
        (avid => '0', hsync_n => '1', vsync_n => '1', field_n => '1'));

    type t_cdly is array (0 to C_CDLY - 1) of std_logic_vector(9 downto 0);
    signal s_cu : t_cdly := (others => (others => '0'));
    signal s_cv : t_cdly := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- Line palettes.  10-bit YUV with U/V pre-swapped for the hardware
    -- encoder (U slot carries Cr): Y stored as offset-from-black (yo, so
    -- 64 + yo*16/16 <= 768, respecting the drawn-luma cap), chroma as
    -- signed offsets from 512.  Zone 5 (Video) takes the source chroma
    -- instead, which is already in hardware order (dry path: no swap).
    ----------------------------------------------------------------------
    type t_pal10 is array (0 to 7) of unsigned(9 downto 0);
    type t_pal10s is array (0 to 7) of signed(9 downto 0);
    -- zones 0..6: White, Green, Amber, Cyan, Magenta, Video, Fire (7=Terrain)
    constant C_ZY : t_pal10 := (to_unsigned(704, 10), to_unsigned(587, 10),
        to_unsigned(647, 10), to_unsigned(614, 10), to_unsigned(516, 10),
        to_unsigned(620, 10), to_unsigned(400, 10), to_unsigned(704, 10));
    constant C_ZU : t_pal10s := (to_signed(0, 10), to_signed(-300, 10),
        to_signed(166, 10), to_signed(-448, 10), to_signed(335, 10),
        to_signed(0, 10), to_signed(347, 10), to_signed(0, 10));
    constant C_ZV : t_pal10s := (to_signed(0, 10), to_signed(-237, 10),
        to_signed(-373, 10), to_signed(151, 10), to_signed(208, 10),
        to_signed(0, 10), to_signed(-181, 10), to_signed(0, 10));
    -- Terrain tint, indexed by the line's luma band: 0 = least sag (peak,
    -- white-hot) .. 7 = deepest sag (valley, dim blue)
    constant C_TY : t_pal10 := (to_unsigned(704, 10), to_unsigned(585, 10),
        to_unsigned(682, 10), to_unsigned(604, 10), to_unsigned(468, 10),
        to_unsigned(414, 10), to_unsigned(271, 10), to_unsigned(111, 10));
    constant C_TU : t_pal10s := (to_signed(30, 10), to_signed(212, 10),
        to_signed(77, 10), to_signed(-121, 10), to_signed(-277, 10),
        to_signed(-238, 10), to_signed(-133, 10), to_signed(-81, 10));
    constant C_TV : t_pal10s := (to_signed(-52, 10), to_signed(-262, 10),
        to_signed(-343, 10), to_signed(-297, 10), to_signed(-118, 10),
        to_signed(165, 10), to_signed(299, 10), to_signed(239, 10));

begin

    ----------------------------------------------------------------------
    -- Position counters, per-row bank bookkeeping, vsync parameter latch
    ----------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_samp    : std_logic;
        variable v_newhead : unsigned(2 downto 0);
        variable v_inv     : unsigned(9 downto 0);
        variable v_d7      : unsigned(6 downto 0);
        variable v_dm      : unsigned(7 downto 0);
        variable v_targ    : unsigned(9 downto 0);
        variable v_diff    : signed(10 downto 0);
        variable v_next    : signed(10 downto 0);
        variable v_ro      : unsigned(8 downto 0);
        variable v_se      : unsigned(14 downto 0);
        variable v_rp      : unsigned(14 downto 0);
        variable v_sp      : signed(8 downto 0);
        variable v_ag      : unsigned(10 downto 0);
        variable v_tw      : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            if unsigned(registers_in(2)) < 8 then
                s_k3_idx <= unsigned(registers_in(2)(2 downto 0));
            else
                s_k3_idx <= unsigned(registers_in(2)(9 downto 7));
            end if;
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            -- y step (free-running from the timing ID) and the next active
            -- row's virtual-row advance (registered ahead: rows are >= 858
            -- clocks apart, so it is always settled at row start)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ys <= to_unsigned(1138, 12); s_kc <= to_unsigned(5, 4); -- 480i
                when 4          => s_ys <= to_unsigned(576, 12);  s_kc <= to_unsigned(3, 4); -- 480p
                when 8          => s_ys <= to_unsigned(960, 12);  s_kc <= to_unsigned(4, 4); -- 576i
                when 12         => s_ys <= to_unsigned(480, 12);  s_kc <= to_unsigned(2, 4); -- 576p
                when 5 | 6 | 14 => s_ys <= to_unsigned(384, 12);  s_kc <= to_unsigned(2, 4); -- 720p
                when 1 | 2 | 10 => s_ys <= to_unsigned(512, 12);  s_kc <= to_unsigned(2, 4); -- 1080i
                when others     => s_ys <= to_unsigned(256, 12);  s_kc <= to_unsigned(1, 4); -- 1080p
            end case;
            v_ag := resize(s_yf, 11) + resize(s_ys, 11);
            s_ka <= v_ag(10 downto 8);
            v_sp := s_spacing - signed(resize(s_ka, 9));
            if v_sp <= 0 then
                s_samp_nx <= '1';
                s_sp_nx   <= v_sp + signed(resize(s_dspace, 9));
            else
                s_samp_nx <= '0';
                s_sp_nx   <= v_sp;
            end if;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then
                pixel_x     <= (others => '0');
                px_rd       <= resize(s_lagp1, 12);
                s_row_armed <= '1';
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
                if px_rd(10 downto 1) < s_wlast then
                    px_rd <= px_rd + 1;
                end if;
                s_wline <= pixel_x(10 downto 1);   -- ends at the last column
            end if;

            if data_in.avid = '1' then s_saw_frame <= '1'; end if;

            -- Quasi-static products (inputs change only at vsync)
            v_se := s_scale9 * s_rmul;
            s_scale_eff <= v_se(13 downto 5);
            s_rspd <= resize(s_ru * s_ru, 10) & "0000";
            v_rp := s_rphase(15 downto 8) * s_dspace;
            s_roff <= v_rp(14 downto 8);

            -- Row-start bookkeeping: fires on the FIRST active pixel of each
            -- row.  Everything decided here is first consumed >= 3 pipeline
            -- stages later, so the one-cycle registration lag is safe.
            if data_in.avid = '1' and s_row_armed = '1' then
                s_row_armed <= '0';
                -- virtual rows: this row advanced s_ka units (1 at 1080p,
                -- where this is the old 'spacing = 0' test and reload D - 1)
                v_ag := resize(s_yf, 11) + resize(s_ys, 11);
                s_yf <= v_ag(7 downto 0);
                v_samp    := s_samp_nx;
                s_spacing <= s_sp_nx;
                s_sampled <= v_samp;
                if v_samp = '1' then
                    v_newhead := s_head + 1;
                    s_rowtop   <= s_firstrow;
                    s_firstrow <= '0';
                else
                    v_newhead := s_head;
                end if;
                s_head <= v_newhead;
                for k in 0 to NB - 1 loop
                    if v_samp = '1' and k = to_integer(v_newhead) then
                        s_age(k)   <= (others => '0');
                        s_valid(k) <= '1';
                        s_live(k)  <= '0';
                    elsif s_valid(k) = '1' then
                        v_ag := resize(s_age(k), 11) + resize(s_ka, 11);
                        if v_ag(10) = '1' then
                            s_age(k) <= to_unsigned(1023, 10);
                        else
                            s_age(k) <= v_ag(9 downto 0);
                        end if;
                        s_live(k) <= '1';
                    end if;
                end loop;
            end if;

            -- Field/frame start.  Analog vsync serrates (several edges per
            -- field): every action here is idempotent, and the relief glide
            -- and roll step are guarded to one step per field by s_saw_frame.
            if v_v_edge = '1' then
                s_valid    <= (others => '0');
                s_live     <= (others => '0');
                -- roll: first sampled row = phase x spacing; the field's top
                -- line fades in with the phase so lines never pop in
                -- (+ the first row's advance: the row-start test subtracts it)
                s_spacing  <= signed(resize(s_roff, 9)) + signed(resize(s_ys(11 downto 8), 9));
                s_yf       <= (others => '0');
                s_fade4    <= s_rphase(15 downto 12);
                s_firstrow <= '1';
                s_wlast    <= s_wline;

                -- K1 Scan Lines: CW = denser.  Spacing 8..71 rows.
                v_inv := 1023 - unsigned(registers_in(0));
                v_d7  := to_unsigned(8, 7) + v_inv(9 downto 4);
                s_dspace <= v_d7;
                -- Max displacement: a bank lives 8 sampled rows, so a line
                -- stays retrievable while disp < 8*spacing (storage caps 255).
                -- Relief is normalised to that window: x min(spacing,32)/32.
                if v_d7 >= 32 then
                    s_dmax <= to_unsigned(255, 8);
                    s_rmul <= to_unsigned(32, 6);
                else
                    v_dm := shift_left(resize(v_d7, 8), 3) - 1;
                    s_dmax <= v_dm;
                    s_rmul <= resize(v_d7, 6);
                end if;

                v_tw := to_unsigned(1, 4) + resize(unsigned(registers_in(1)(9 downto 7)), 4);
                s_thick  <= v_tw;
                if v_tw < s_kc then
                    v_tw := s_kc;
                end if;
                s_thkw <= v_tw;
                s_half <= shift_right(v_tw + 1, 1);   -- ceil(Tw / 2)
                case registers_in(1)(9 downto 7) is
                    when "000"  => s_bboost <= "11";   -- T=1: +3
                    when "001"  => s_bboost <= "10";   -- T=2: +2
                    when "010"  => s_bboost <= "01";   -- T=3: +1
                    when others => s_bboost <= "00";
                end case;
                s_zone   <= s_k3_idx;
                s_gain8  <= unsigned(registers_in(3)(9 downto 2));
                s_smooth <= unsigned(registers_in(5)(9 downto 7));
                case registers_in(5)(9 downto 7) is
                    when "000"  => s_lagp1 <= to_unsigned(1, 8);
                    when "001"  => s_lagp1 <= to_unsigned(2, 8);
                    when "010"  => s_lagp1 <= to_unsigned(4, 8);
                    when "011"  => s_lagp1 <= to_unsigned(8, 8);
                    when "100"  => s_lagp1 <= to_unsigned(16, 8);
                    when "101"  => s_lagp1 <= to_unsigned(32, 8);
                    when "110"  => s_lagp1 <= to_unsigned(64, 8);
                    when others => s_lagp1 <= to_unsigned(128, 8);
                end case;

                -- K5 Roll: bipolar about centre, quadratic speed, up to
                -- ~0.23 line spacings per field (no wagon-wheel aliasing)
                if registers_in(4)(9) = '1' then
                    s_rdir <= '0';
                    v_ro := unsigned(registers_in(4)(8 downto 0));
                else
                    s_rdir <= '1';
                    v_ro := not unsigned(registers_in(4)(8 downto 0));
                end if;
                s_ru <= v_ro(8 downto 4);

                s_peakdark <= registers_in(6)(0);
                s_hlr      <= registers_in(6)(1);
                s_shade    <= registers_in(6)(2);
                s_rfreeze  <= registers_in(6)(3);
                case registers_in(1)(9 downto 7) is
                    when "000" | "001" =>
                        s_glow <= '0'; s_gsh <= "010";
                    when "010" | "011" =>
                        s_glow <= '1'; s_gsh <= "010";
                    when others =>
                        s_glow <= '1'; s_gsh <= "001";
                end case;
                s_neg      <= registers_in(6)(4);

                -- P12 Relief -> displacement scale, glided (ease by diff/4
                -- with a +/-1 deadband) so pot ADC noise cannot make the
                -- whole line field breathe.
                if s_saw_frame = '1' then
                    s_saw_frame <= '0';
                    if s_rfreeze = '1' then
                        null;
                    elsif s_rdir = '0' then
                        s_rphase <= s_rphase + s_rspd;
                    else
                        s_rphase <= s_rphase - s_rspd;
                    end if;
                    v_targ := to_unsigned(8, 10) + resize(unsigned(registers_in(7)(9 downto 2)), 10);
                    v_diff := signed(resize(v_targ, 11)) - signed(resize(s_scale9, 11));
                    v_next := signed(resize(s_scale9, 11));
                    if v_diff > 3 then
                        v_next := v_next + shift_right(v_diff, 2);
                    elsif v_diff > 1 then
                        v_next := v_next + 1;
                    elsif v_diff < -3 then
                        v_next := v_next + shift_right(v_diff, 2);
                    elsif v_diff < -1 then
                        v_next := v_next - 1;
                    end if;
                    s_scale9 <= unsigned(v_next(8 downto 0));
                end if;
            end if;
        end if;
    end process p_position;

    ----------------------------------------------------------------------
    -- Write path: luma -> (negative) -> contrast gain -> smoothing IIR ->
    -- displacement + brightness, written into bank[head] on sampled rows
    -- (even columns only; the store is column-decimated x2).
    ----------------------------------------------------------------------
    p_write : process(clk)
        variable v_arg  : unsigned(9 downto 0);
        variable v_l11  : unsigned(10 downto 0);
        variable v_lum  : unsigned(9 downto 0);
        variable v_diff : signed(14 downto 0);
        variable v_step : signed(14 downto 0);
        variable v_nacc : signed(14 downto 0);
        variable v_d6   : unsigned(5 downto 0);
        variable v_bm   : unsigned(7 downto 0);
        variable v_dr   : unsigned(9 downto 0);
        variable v_disp : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- W0: input register + S11 negative
            if s_neg = '1' then
                w0_y <= to_unsigned(1023, 10) - unsigned(data_in.y);
            else
                w0_y <= unsigned(data_in.y);
            end if;
            w_addr(0) <= pixel_x(10 downto 1);
            w_av(0)   <= data_in.avid;
            w_par(0)  <= pixel_x(0);

            -- W1: contrast gain (unity at knob centre: x gain8/128, 0..2x)
            if w0_y > 64 then
                v_arg := w0_y - 64;
            else
                v_arg := (others => '0');
            end if;
            w1_prod   <= v_arg * s_gain8;
            w_addr(1) <= w_addr(0);
            w_av(1)   <= w_av(0);
            w_par(1)  <= w_par(0);

            -- W2: clamp + horizontal smoothing IIR (14-bit accumulator with
            -- 4 fractional bits; reset to the first sample of each row)
            v_l11 := w1_prod(17 downto 7);
            if v_l11 > 1023 then
                v_lum := to_unsigned(1023, 10);
            else
                v_lum := v_l11(9 downto 0);
            end if;
            if w_av(1) = '1' then
                if w_av(2) = '0' then
                    w2_acc <= v_lum & "0000";
                else
                    v_diff := signed(resize(v_lum & "0000", 15)) - signed(resize(w2_acc, 15));
                    case to_integer(s_smooth) is
                        when 0      => v_step := v_diff;
                        when 1      => v_step := shift_right(v_diff, 1);
                        when 2      => v_step := shift_right(v_diff, 2);
                        when 3      => v_step := shift_right(v_diff, 3);
                        when 4      => v_step := shift_right(v_diff, 4);
                        when 5      => v_step := shift_right(v_diff, 5);
                        when 6      => v_step := shift_right(v_diff, 6);
                        when others => v_step := shift_right(v_diff, 7);
                    end case;
                    v_nacc := signed(resize(w2_acc, 15)) + v_step;
                    w2_acc <= unsigned(v_nacc(13 downto 0));
                end if;
            end if;
            w_addr(2) <= w_addr(1);
            w_av(2)   <= w_av(1);
            w_par(2)  <= w_par(1);

            -- W3: sag term x (window-normalised) relief scale, own stage;
            -- the field's top (rolling) line has its brightness faded
            if s_peakdark = '1' then
                v_d6 := w2_acc(13 downto 8);
            else
                v_d6 := 63 - w2_acc(13 downto 8);
            end if;
            w3_prod <= v_d6 * s_scale_eff;
            if s_rowtop = '1' then
                v_bm := w2_acc(13 downto 10) * s_fade4;
                w3_brt <= v_bm(7 downto 4);
            else
                w3_brt <= w2_acc(13 downto 10);
            end if;
            w_addr(3) <= w_addr(2);
            w_av(3)   <= w_av(2);
            w_par(3)  <= w_par(2);

            -- W4: displacement clamp (min 1 so a line never lands on its own
            -- write row) + pack.  Write on even columns of sampled rows.
            v_dr := w3_prod(14 downto 5);
            if v_dr > resize(s_dmax, 10) then
                v_disp := s_dmax;
            elsif v_dr = 0 then
                v_disp := to_unsigned(1, 8);
            else
                v_disp := v_dr(7 downto 0);
            end if;
            w4_data <= std_logic_vector(v_disp) & std_logic_vector(w3_brt);
            w4_addr <= w_addr(3);
            if w_av(3) = '1' and s_sampled = '1' and w_par(3) = '0' then
                w4_we <= '1';
            else
                w4_we <= '0';
            end if;
        end if;
    end process p_write;

    ----------------------------------------------------------------------
    -- 8 rotating profile banks: canonical 1W1R, 1024 x 12 each.
    -- Per-bank registered read address (kept) so the placer can sit each
    -- copy beside its own EBR group instead of routing one high-fanout net.
    ----------------------------------------------------------------------
    gen_banks : for k in 0 to NB - 1 generate
        type t_ram is array (0 to 1023) of std_logic_vector(11 downto 0);
        signal ram    : t_ram := (others => (others => '0'));
        signal rdaddr : unsigned(9 downto 0) := (others => '0');
        signal rdata  : std_logic_vector(11 downto 0) := (others => '0');
        attribute keep of rdaddr : signal is true;
    begin
        p_bank : process(clk)
        begin
            if rising_edge(clk) then
                rdaddr <= px_rd(10 downto 1);
                if w4_we = '1' and to_integer(s_head) = k then
                    ram(to_integer(w4_addr)) <= w4_data;
                end if;
                rdata <= ram(to_integer(rdaddr));
                -- second (fabric) register after the EBR-absorbed one
                a2_disp(k) <= unsigned(rdata(11 downto 4));
                a2_brt(k)  <= unsigned(rdata(3 downto 0));
            end if;
        end process p_bank;
    end generate gen_banks;

    ----------------------------------------------------------------------
    -- Display pipeline: connected-stroke segment -> candidate tests ->
    -- recency-priority resolve -> palette + shading -> compose -> blanking
    ----------------------------------------------------------------------
    p_disp : process(clk)
        variable v_t    : signed(10 downto 0);
        variable v_2t   : signed(10 downto 0);
        variable v_nt   : signed(10 downto 0);
        variable v_t4   : unsigned(7 downto 0);
        variable v_core : boolean;
        variable v_halo : boolean;
        variable v_hin  : boolean;
        variable v_nh   : signed(10 downto 0);
        variable v_th   : signed(10 downto 0);
        variable v_cov  : boolean;
        variable v_sh   : unsigned(2 downto 0);
        variable v_m9   : unsigned(8 downto 0);
        variable v_ab   : signed(8 downto 0);
        variable v_ba   : signed(8 downto 0);
        variable v_idx  : integer range 0 to 7;
        variable v_hidx : integer range 0 to 7;
        variable v_cidx : integer range 0 to 7;
        variable v_anyh : std_logic;
        variable v_anyc : std_logic;
        variable v_line : std_logic;
        variable v_bank : integer range 0 to 7;
        variable v_cs   : signed(9 downto 0);
        variable v_alt  : unsigned(2 downto 0);
        variable v_bf   : unsigned(3 downto 0);
        variable v_bf5  : unsigned(4 downto 0);
        variable v_yi   : unsigned(10 downto 0);
        variable v_ui   : integer range -2048 to 2047;
        variable v_vi   : integer range -2048 to 2047;
    begin
        if rising_edge(clk) then
            -- R1..R3: read-position flags travel beside the bank reads
            r_ph(1)    <= px_rd(0);
            r_first(1) <= data_in.avid and s_row_armed;
            r_ph(3 downto 2)    <= r_ph(2 downto 1);
            r_first(3 downto 2) <= r_first(2 downto 1);

            -- B1: previous-column height.  A new column (phase 0) takes the
            -- previous pixel's value; the row's first pixel has none (flat).
            for k in 0 to NB - 1 loop
                p_d1(k) <= a2_disp(k);
                if r_first(3) = '1' then
                    b1_prev(k) <= a2_disp(k);
                    s_pc(k)    <= a2_disp(k);
                elsif r_ph(3) = '0' then
                    b1_prev(k) <= p_d1(k);
                    s_pc(k)    <= p_d1(k);
                else
                    b1_prev(k) <= s_pc(k);
                end if;
                b1_d(k)   <= a2_disp(k);
                b1_brt(k) <= a2_brt(k);
            end loop;
            b1_ph <= r_ph(3);

            -- B2: midpoint subdivision: phase 0 draws prev -> mid, phase 1
            -- draws mid -> this column
            for k in 0 to NB - 1 loop
                v_m9 := resize(b1_prev(k), 9) + resize(b1_d(k), 9);
                if b1_ph = '0' then
                    b2_ea(k) <= b1_prev(k);
                    b2_eb(k) <= v_m9(8 downto 1);
                else
                    b2_ea(k) <= v_m9(8 downto 1);
                    b2_eb(k) <= b1_d(k);
                end if;
                b2_brt(k) <= b1_brt(k);
            end loop;

            -- B3: segment extent (lo = top, hi = bottom, span = hi - lo)
            for k in 0 to NB - 1 loop
                v_ab := signed(resize(b2_ea(k), 9)) - signed(resize(b2_eb(k), 9));
                v_ba := signed(resize(b2_eb(k), 9)) - signed(resize(b2_ea(k), 9));
                if v_ab(8) = '1' then
                    b3_lo(k)   <= b2_ea(k);
                    b3_hi(k)   <= b2_eb(k);
                    b3_span(k) <= unsigned(v_ba(7 downto 0));
                else
                    b3_lo(k)   <= b2_eb(k);
                    b3_hi(k)   <= b2_ea(k);
                    b3_span(k) <= unsigned(v_ab(7 downto 0));
                end if;
                b3_brt(k) <= b2_brt(k);
            end loop;

            -- A3: candidate offsets against both segment ends; beam-speed
            -- dimming from the segment's vertical extent (>T: 1/2, >4T: 1/4)
            v_t4 := shift_left(resize(s_thick, 8), 2);
            for k in 0 to NB - 1 loop
                a3_slo(k) <= signed(resize(s_age(k), 11)) - signed(resize(b3_lo(k), 11));
                a3_shi(k) <= signed(resize(s_age(k), 11)) - signed(resize(b3_hi(k), 11));
                if b3_span(k) > v_t4 then
                    a3_dim(k) <= "10";
                elsif b3_span(k) > resize(s_thick, 8) then
                    a3_dim(k) <= "01";
                else
                    a3_dim(k) <= "00";
                end if;
                a3_brt(k) <= b3_brt(k);
            end loop;

            -- A4: classify each candidate against the stroke window
            v_t  := signed(resize(s_thkw, 11));
            v_2t := signed(resize(shift_left(resize(s_thkw, 5), 1), 11));
            v_nt := -v_t;
            v_nh := -signed(resize(s_half, 11));
            v_th := v_t + signed(resize(s_half, 11));
            for k in 0 to NB - 1 loop
                v_cov  := s_live(k) = '1' and a3_slo(k) >= 0;
                v_core := v_cov and a3_shi(k) < v_t;
                v_halo := s_glow = '1' and s_live(k) = '1' and
                          ((a3_slo(k) >= v_nt and a3_slo(k) < 0) or
                           (a3_shi(k) >= v_t and a3_shi(k) < v_2t));
                if v_core or v_halo then a4_hit(k) <= '1'; else a4_hit(k) <= '0'; end if;
                if v_cov then a4_cov(k) <= '1'; else a4_cov(k) <= '0'; end if;
                -- total dim shift: beam-speed 0..2, plus the glow level for
                -- halo pixels (inner half of the halo / outer half +1)
                v_hin := a3_slo(k) >= v_nh and a3_shi(k) < v_th;
                v_sh := resize(a3_dim(k), 3);
                if v_halo and not v_core then
                    if v_hin then
                        v_sh := v_sh + s_gsh;
                    else
                        v_sh := v_sh + s_gsh + 1;
                    end if;
                end if;
                a4_sh(k)  <= v_sh;
                a4_brt(k) <= a3_brt(k);
            end loop;

            -- A5: rotate hit/cover vectors so index 0 = most recent bank
            -- (lowest source row = nearest = hidden-line priority)
            for i in 0 to NB - 1 loop
                v_idx := to_integer(s_head - to_unsigned(i, 3));
                a5_rhit(i) <= a4_hit(v_idx);
                a5_rcov(i) <= a4_cov(v_idx);
            end loop;
            a5_sh  <= a4_sh;
            a5_brt <= a4_brt;

            -- A6: priority encode (lowest rotated index wins)
            v_hidx := 7; v_anyh := '0';
            v_cidx := 7; v_anyc := '0';
            for i in NB - 1 downto 0 loop
                if a5_rhit(i) = '1' then v_hidx := i; v_anyh := '1'; end if;
                if a5_rcov(i) = '1' then v_cidx := i; v_anyc := '1'; end if;
            end loop;
            a6_hidx   <= to_unsigned(v_hidx, 3);
            a6_cidx   <= to_unsigned(v_cidx, 3);
            a6_anyhit <= v_anyh;
            a6_anycov <= v_anyc;
            a6_sh     <= a5_sh;
            a6_brt    <= a5_brt;

            -- A7: draw decision + winner payload mux.  A hit draws unless a
            -- strictly nearer line's skirt covers this pixel (hidden-line
            -- mode only); skirts and empty space are both black.
            if a6_anyhit = '1' and (s_hlr = '0' or a6_anycov = '0' or a6_hidx <= a6_cidx) then
                v_line := '1';
            else
                v_line := '0';
            end if;
            a7_line <= v_line;
            v_bank := to_integer(s_head - a6_hidx);
            a7_brt <= a6_brt(v_bank);
            a7_sh  <= a6_sh(v_bank);
            -- A7 (Video lane): source chroma at this display pixel, x2
            -- saturation, clamped to +/-448
            v_cs := signed(s_cu(C_CDLY - 1) xor "1000000000");
            if v_cs > 224 then
                a7_cu <= to_signed(448, 10);
            elsif v_cs < -224 then
                a7_cu <= to_signed(-448, 10);
            else
                a7_cu <= shift_left(v_cs, 1);
            end if;
            v_cs := signed(s_cv(C_CDLY - 1) xor "1000000000");
            if v_cs > 224 then
                a7_cv <= to_signed(448, 10);
            elsif v_cs < -224 then
                a7_cv <= to_signed(-448, 10);
            else
                a7_cv <= shift_left(v_cs, 1);
            end if;

            -- A8: palette + shading factor
            v_alt := a7_brt(3 downto 1) xor (2 downto 0 => not s_peakdark);
            if s_zone = 7 then
                a8_yo <= C_TY(to_integer(v_alt));
                a8_uo <= C_TU(to_integer(v_alt));
                a8_vo <= C_TV(to_integer(v_alt));
            elsif s_zone = 5 then
                a8_yo <= C_ZY(5);
                a8_uo <= a7_cu;
                a8_vo <= a7_cv;
            else
                a8_yo <= C_ZY(to_integer(s_zone));
                a8_uo <= C_ZU(to_integer(s_zone));
                a8_vo <= C_ZV(to_integer(s_zone));
            end if;
            if s_shade = '1' then
                v_bf := a7_brt;
            elsif a7_brt >= 8 then
                v_bf := to_unsigned(15, 4);
            else
                v_bf := to_unsigned(4, 4);
            end if;
            case to_integer(a7_sh) is
                when 0      => null;
                when 1      => v_bf := shift_right(v_bf, 1);
                when 2      => v_bf := shift_right(v_bf, 2);
                when 3      => v_bf := shift_right(v_bf, 3);
                when others => v_bf := (others => '0');
            end case;
            -- +1 (full scale at 16) plus the thin-stroke lift, held at the
            -- 768 drawn-luma cap (bf1 = 16)
            v_bf5 := resize(v_bf, 5) + 1 + resize(s_bboost, 5);
            if v_bf5 > 16 then
                v_bf5 := to_unsigned(16, 5);
            end if;
            a8_bf1  <= v_bf5;
            a8_line <= a7_line;

            -- A9: shading multiplies (each gets the stage to itself)
            a9_ym   <= a8_yo * a8_bf1;
            a9_um   <= a8_uo * signed(resize(a8_bf1, 6));
            a9_vm   <= a8_vo * signed(resize(a8_bf1, 6));
            a9_line <= a8_line;

            -- A10: compose (line over black).  Line luma tops out at
            -- 64 + 704 = 768 by construction (drawn-luma cap); chroma at
            -- 512 +/- 448.
            if a9_line = '1' then
                v_yi := to_unsigned(64, 11) + resize(a9_ym(14 downto 4), 11);
                a10_y <= v_yi(9 downto 0);
                v_ui := 512 + to_integer(shift_right(a9_um, 4));
                v_vi := 512 + to_integer(shift_right(a9_vm, 4));
                a10_u <= to_unsigned(v_ui, 10);
                a10_v <= to_unsigned(v_vi, 10);
            else
                a10_y <= to_unsigned(64, 10);
                a10_u <= to_unsigned(512, 10);
                a10_v <= to_unsigned(512, 10);
            end if;

            -- A11: blanking gate (one stage before the output register is
            -- this register: outputs are neutral whenever aligned avid = 0)
            if s_dly(C_LAT - 2).avid = '1' then
                a11_y <= a10_y;
                a11_u <= a10_u;
                a11_v <= a10_v;
            else
                a11_y <= to_unsigned(64, 10);
                a11_u <= to_unsigned(512, 10);
                a11_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_disp;

    ----------------------------------------------------------------------
    -- Sync delay line + input-chroma delay
    ----------------------------------------------------------------------
    p_dly : process(clk)
    begin
        if rising_edge(clk) then
            s_dly(0).avid    <= data_in.avid;
            s_dly(0).hsync_n <= data_in.hsync_n;
            s_dly(0).vsync_n <= data_in.vsync_n;
            s_dly(0).field_n <= data_in.field_n;
            for i in 1 to C_LAT - 1 loop
                s_dly(i) <= s_dly(i - 1);
            end loop;
            s_cu(0) <= data_in.u;
            s_cv(0) <= data_in.v;
            for i in 1 to C_CDLY - 1 loop
                s_cu(i) <= s_cu(i - 1);
                s_cv(i) <= s_cv(i - 1);
            end loop;
        end if;
    end process p_dly;

    ----------------------------------------------------------------------
    -- Outputs
    ----------------------------------------------------------------------
    s_io_align_in.y <= std_logic_vector(a11_y);
    s_io_align_in.u <= std_logic_vector(a11_u);
    s_io_align_in.v <= std_logic_vector(a11_v);

    s_io_align_in.avid    <= s_dly(C_LAT - 1).avid;
    s_io_align_in.hsync_n <= s_dly(C_LAT - 1).hsync_n;
    s_io_align_in.vsync_n <= s_dly(C_LAT - 1).vsync_n;
    s_io_align_in.field_n <= s_dly(C_LAT - 1).field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (15 -> 16).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end ruttetra;
