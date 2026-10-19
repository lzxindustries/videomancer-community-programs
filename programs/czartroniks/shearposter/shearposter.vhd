-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Shearposter v0.2.0: catalogue combo #110 (seed 20260925) -- pixelation +
-- scanline shear / horizontal tearing + posterization.  The switches and the
-- slider decide how they combine.
--
--   K1 Cell     cell width 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 px (labelled)
--   K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 lines (labelled)
--   K3 Dither   2x2 ordered dither: Off / 2 / 4 / 8 / 16 / 32 / 64 px / Cell
--               (Cell = one threshold per cell, carried with the shear)
--   K4 Rate     Zigzag: waves down the frame, squared (0 = the whole frame
--               sways as one, top = an 8-line zigzag) / Tear: share of lines
--               torn, squared (1%..100%).  Glided
--   K5 Luma     luma levels 1..8 bits (labelled)
--   K6 Chroma   chroma levels 1..8 bits (labelled)
--
--   S7  Shear   Zigzag (a triangle wave rolling down the frame: straight shear
--               segments) / Tear (a random offset per line, every field)
--   S8  Shift   Free / Cells (offsets rounded to whole cells: the cell grid
--               stays on the screen and the picture steps a cell at a time)
--   S9  Cells   Lean (every line takes its own offset: cells lean) / Slide
--               (one offset per cell row: rows slide as rigid strips)
--   S10 Edges   Fill (the line's own edge pixel) / Mirror
--   S11 Colour  Together / Split (chroma shears the opposite way)
--   P12 Shear   amount, glided, squared: width/64 at 0 .. width/2 at 100%
--
-- v0.2.2 (2026-10-17): resolution scaling.  The read-address chain (x, the
-- shear, the edges, the cell grid) runs in virtual 1080-line units: x steps
-- 1.0 / 1.5 / 2.0 per pixel at 1080 / 720p / SD (SD = a 1440-wide 4:3
-- window), from the video timing ID; the cell centre is turned back into a
-- line-buffer pixel at the end (720p floor((2V+1)/3), SD V/2; 4 clocks), so
-- K1 cells keep their 1080 size relative to the screen height.  K2 Rows
-- count virtual lines (Q8 accumulator, 2 x 1080/486 per 480i field line)
-- and the zigzag steps step x (1080/active lines) per line, so SD cells and
-- waves are no longer twice as tall.  The shear amount is sized from 1920
-- units on the 4:3 SD window (sizes follow the screen height); the 12-px
-- edge inset stays 12 pixels.  The ordered dither stays at pixel scale.
-- 1080p is bit-identical; latency 20 -> 24.
--
-- Rows: the first line of each cell row captures into one bank of a 2-bank
-- line buffer; the row is shown from its next line on, so the picture moves
-- down 1 line at every Rows setting.  Each output line reads the captured
-- line at x + shear; the read is edge-filled or mirrored about a C_EDGE inset,
-- then held to the centre of its cell.  Posterize after the read so the
-- dither stays on the screen grid: luma normalized to the video range and
-- bit-replicated (black and white are always levels), chroma a centred
-- mid-tread quantizer (greys stay grey).
--
-- The shear for a line is worked out in hblank by a per-line serial
-- shift-add multiply (wave x amplitude); the amplitude, the zigzag step and
-- the tear share by a free-running serial multiplier from the glided P12/K4.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (22) in all modes; banks,
-- offsets and dither row latched per line at the avid rise; U/V swapped on
-- rows whose hsync->avid parity differs from their capture row's (every
-- row replays a stored line -- drape v1.3); output conditioning ([1 2 1] on
-- Y/U/V + luma scaled to white ~777, the damier v0.4.4 HD HDMI bar fix:
-- posterized, dithered steps that tear every field are the trigger profile);
-- no_rw_check on every RAM (a bank is never read while it is written; the
-- write bank's read data is discarded).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture shearposter of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (22 + 2 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT   : integer := 22;
    constant C_EDGE  : integer := 12;     -- capture blanking columns kept out of every read
    constant C_SPEED : integer := 768;    -- zigzag roll per field (16-bit phase): ~1.4 s a cycle at 60 Hz

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    -- a bank is never read while it is written (the other bank is shown)
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    -- Resolution scaling (mondrian v1.0.2): per-mode steps from the timing ID.
    -- x step in half virtual units per pixel: 1080 2 (1.0), 720p 3 (1.5),
    -- SD 4 (2.0: 720 px span 1440 units, a 4:3 window of the 1080 canvas).
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y step in Q4.8 virtual units per FIELD line (1080 / active lines,
    -- doubled for interlaced modes).
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

    type t_m7 is array (0 to 7) of unsigned(6 downto 0);
    constant C_MASK : t_m7 := ("0000000", "0000001", "0000011", "0000111", "0001111", "0011111", "0111111", "1111111");
    constant C_HALF : t_m7 := ("0000000", "0000001", "0000010", "0000100", "0001000", "0010000", "0100000", "1000000");

    -- labelled 8-step knob: the firmware loads a labelled default as its
    -- label INDEX (0..7); otherwise the zone, changing only 12 counts inside
    -- a new zone so a pot resting on a boundary can't flicker
    function f_zone(v : unsigned(9 downto 0); cur : unsigned(2 downto 0)) return unsigned is
        variable z  : unsigned(2 downto 0);
        variable lo : unsigned(6 downto 0);
    begin
        if v < 8 then
            return v(2 downto 0);
        end if;
        z := v(9 downto 7); lo := v(6 downto 0);
        if (lo >= 12 or z = 0) and (lo <= 115 or z = 7) then
            return z;
        end if;
        return cur;
    end function;

    -- top L = z+1 bits of t, replicated down to 10 bits
    function f_rep10(t : unsigned(9 downto 0); z : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(z) is
            when 0 => r := (others => t(9));
            when 1 => r := t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8);
            when 2 => r := t(9 downto 7) & t(9 downto 7) & t(9 downto 7) & t(9);
            when 3 => r := t(9 downto 6) & t(9 downto 6) & t(9 downto 8);
            when 4 => r := t(9 downto 5) & t(9 downto 5);
            when 5 => r := t(9 downto 4) & t(9 downto 6);
            when 6 => r := t(9 downto 3) & t(9 downto 7);
            when others => r := t(9 downto 2) & t(9 downto 8);
        end case;
        return r;
    end function;

    -- centred chroma: keep the masked level, clamp to 64..960
    function f_cq(t : unsigned(10 downto 0); m : unsigned(9 downto 0)) return unsigned is
        variable q : unsigned(9 downto 0);
    begin
        q := t(9 downto 0) and m;
        if t(10) = '1' or q > 960 then
            q := to_unsigned(960, 10);
        elsif q < 64 then
            q := to_unsigned(64, 10);
        end if;
        return q;
    end function;

    signal s_k_cell, s_k_rows, s_k_dith, s_k_rate, s_k_lbits, s_k_cbits : unsigned(9 downto 0);
    signal s_sw_tear, s_sw_snap, s_sw_slide, s_sw_mirror, s_sw_split : std_logic;
    signal s_p_shear : unsigned(9 downto 0);

    -- knob zones (live) and their per-field copies
    signal s_zcell : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_zrows : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_zdith : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal s_zl, s_zc : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_fcell, s_frows, s_fdith : unsigned(2 downto 0) := (others => '0');
    signal s_cmask, s_chalf, s_rmask : unsigned(6 downto 0) := (others => '0');
    signal s_mask11, s_half11 : unsigned(10 downto 0) := (others => '0');
    signal s_gp, s_gk : unsigned(13 downto 0) := (others => '0');     -- glided P12 / K4 (10.4)
    -- quantizer constants (step = 512 >> zone)
    signal s_lstep, s_cstep : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_lqm, s_cqm : unsigned(9 downto 0) := (others => '1');
    signal s_ld0, s_ld1, s_ld2, s_ld3 : signed(10 downto 0) := (others => '0');    -- luma dither, (2b-3)/8 step
    signal s_cd0, s_cd1, s_cd2, s_cd3 : unsigned(9 downto 0) := (others => '0');  -- chroma round + dither, (2b+1)/8 step
    signal s_cqh : unsigned(9 downto 0) := (others => '0');                       -- chroma round, no dither

    -- frame-constant serial multiplier: P12^2 -> amplitude, K4^2 -> step / tear share
    signal s_mph : unsigned(1 downto 0) := (others => '0');
    signal s_mit : unsigned(3 downto 0) := (others => '0');
    signal s_mA  : unsigned(23 downto 0) := (others => '0');
    signal s_mB  : unsigned(11 downto 0) := (others => '0');
    signal s_mP  : unsigned(23 downto 0) := (others => '0');
    signal s_sq  : unsigned(11 downto 0) := (others => '0');
    signal s_range, s_floor : unsigned(10 downto 0) := (others => '0');
    signal s_amp  : unsigned(10 downto 0) := to_unsigned(30, 11);    -- shear amplitude, px
    signal s_step : unsigned(12 downto 0) := (others => '0');        -- zigzag step per line (16-bit phase)
    signal s_prob : unsigned(7 downto 0) := (others => '0');         -- tear share /256

    -- per-line shear sequencer (runs in hblank, from the avid fall)
    signal s_fph, s_tph : unsigned(15 downto 0) := (others => '0');
    signal s_rnd  : unsigned(31 downto 0) := x"2545F491";
    signal s_ls   : unsigned(3 downto 0) := (others => '1');         -- 15 = idle
    signal s_w    : signed(10 downto 0) := (others => '0');
    signal s_wneg : std_logic := '0';
    signal s_lA, s_lP : unsigned(20 downto 0) := (others => '0');
    signal s_lB   : unsigned(9 downto 0) := (others => '0');
    signal s_offr, s_offs : signed(11 downto 0) := (others => '0');
    signal s_offn, s_offcn : signed(11 downto 0) := (others => '0');  -- next line
    signal s_offl, s_offcl : signed(11 downto 0) := (others => '0');  -- this line (luma / chroma)

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);   -- virtual units
    -- resolution scaling: steps (free-running from the timing ID), virtual x
    -- counters, the edge inset in virtual units, the width that sizes the shear
    signal s_stx  : unsigned(2 downto 0) := to_unsigned(2, 3);        -- Q.1 per pixel
    signal s_sty  : unsigned(11 downto 0) := to_unsigned(256, 12);    -- Q4.8 per field line
    signal s_msd, s_m720 : std_logic := '0';
    signal s_vrx, s_vwx : unsigned(12 downto 0) := (others => '0');   -- Q.1
    signal s_cedge : unsigned(10 downto 0) := to_unsigned(C_EDGE, 11);
    signal s_ce13, s_nce13 : signed(12 downto 0) := to_signed(C_EDGE, 13);
    signal s_wsz  : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_ya   : unsigned(15 downto 0) := (others => '0');         -- line position in its cell row, Q8.8
    signal s_rowst : std_logic := '1';                                -- the line starts a cell row
    signal s_stepv : unsigned(15 downto 0) := (others => '0');        -- zigzag step per field line
    signal s_saw_active : std_logic := '0';
    -- rows: capture flag and banks of this line, bank of the last capture
    signal s_cap, s_wbank, s_rbank, s_lastcap, s_prevcap : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_cpar0, s_cpar1, s_swap : std_logic := '0';
    signal s_dline : unsigned(7 downto 0) := (others => '0');

    -- edge bounds (registered) so the compares run on the source value in
    -- parallel with the mirror negate / subtract
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    signal s_w2m1, s_we13, s_wl13, s_nwe13, s_rce13, s_rwe13 : signed(12 downto 0) := (others => '0');

    -- x by stage (s_xs(k) = x of the stage-k pixel; stage 1 is s_wr_x)
    type t_x is array (2 to 10) of unsigned(10 downto 0);
    signal s_xs : t_x := (others => (others => '0'));
    signal s_a2, s_c2 : signed(12 downto 0) := (others => '0');
    signal s_e3, s_ec3, s_h4, s_hc4 : unsigned(10 downto 0) := (others => '0');
    signal s_vY5, s_vC5, s_vY6, s_vC6, s_vY7, s_vC7, s_vY8, s_vC8 : unsigned(10 downto 0) := (others => '0');
    signal s_pY6, s_pC6 : unsigned(11 downto 0) := (others => '0');   -- V + 1
    signal s_tY7, s_tC7 : unsigned(13 downto 0) := (others => '0');   -- x 5
    signal s_tY8, s_tC8 : unsigned(17 downto 0) := (others => '0');   -- x 17
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');
    type t_cb is array (5 to 10) of std_logic;
    signal s_cb : t_cb := (others => '0');
    -- fetch + posterize
    signal s_y7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_db7, s_db8, s_db9, s_db10 : unsigned(1 downto 0) := (others => '0');
    signal s_don7, s_don8, s_don9, s_don10 : std_logic := '0';
    signal s_a8 : unsigned(9 downto 0) := (others => '0');
    signal s_cu8, s_cv8 : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal s_n19 : unsigned(10 downto 0) := (others => '0');
    signal s_n29 : unsigned(6 downto 0) := (others => '0');
    signal s_yn10, s_t11, s_rep12, s_d113 : unsigned(9 downto 0) := (others => '0');
    signal s_d213 : unsigned(4 downto 0) := (others => '0');
    signal s_yq14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_cd is array (9 to 14) of unsigned(9 downto 0);
    signal s_ucd, s_vcd : t_cd := (others => to_unsigned(512, 10));
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_y15, s_g1y, s_g2y, s_y17, s_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15, s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u17, s_v17, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');               -- s_avid_sr(k) = avid of the stage-(k+1) pixel

begin

    s_k_cell  <= unsigned(registers_in(0));
    s_k_rows  <= unsigned(registers_in(1));
    s_k_dith  <= unsigned(registers_in(2));
    s_k_rate  <= unsigned(registers_in(3));
    s_k_lbits <= unsigned(registers_in(4));
    s_k_cbits <= unsigned(registers_in(5));
    s_sw_tear   <= registers_in(6)(0);
    s_sw_snap   <= registers_in(6)(1);
    s_sw_slide  <= registers_in(6)(2);
    s_sw_mirror <= registers_in(6)(3);
    s_sw_split  <= registers_in(6)(4);
    s_p_shear <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_tg, v_cur, v_dg : signed(14 downto 0);
        variable v_s8 : unsigned(10 downto 0);
        variable v_f  : unsigned(15 downto 0);
        variable v_ph : unsigned(10 downto 0);
        variable v_t  : unsigned(9 downto 0);
        variable v_o  : signed(11 downto 0);
        variable v_cap : std_logic;
        variable v_r  : unsigned(31 downto 0);
        variable v_nw : signed(10 downto 0);
        variable v_ya : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            -- resolution scaling: per-mode steps, free-running
            s_stx <= f_vstep_x(registers_in(8)(3 downto 0));
            s_sty <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_msd <= '1'; s_m720 <= '0'; s_cedge <= to_unsigned(2 * C_EDGE, 11);
                when 5 | 6 | 14     => s_msd <= '0'; s_m720 <= '1'; s_cedge <= to_unsigned(C_EDGE + C_EDGE / 2, 11);
                when others         => s_msd <= '0'; s_m720 <= '0'; s_cedge <= to_unsigned(C_EDGE, 11);
            end case;
            s_ce13  <= signed(resize(s_cedge, 13));
            s_nce13 <= -signed(resize(s_cedge, 13));
            if s_msd = '1' then s_wsz <= to_unsigned(1920, 11); else s_wsz <= s_line_width; end if;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1; s_vrx <= s_vrx + s_stx; s_saw_active <= '1';
            else
                s_rx <= (others => '0'); s_vrx <= (others => '0');
            end if;
            if s_avid_sr(0) = '1' then
                s_wr_x <= s_wr_x + 1; s_vwx <= s_vwx + s_stx;
            else
                s_wr_x <= (others => '0'); s_vwx <= (others => '0');
            end if;
            v_r := s_rnd xor shift_left(s_rnd, 13);     -- xorshift32: all bits new every clock
            v_r := v_r xor shift_right(v_r, 17);
            s_rnd <= v_r xor shift_left(v_r, 5);

            -- knob zones
            s_zcell <= f_zone(s_k_cell, s_zcell);
            s_zrows <= f_zone(s_k_rows, s_zrows);
            s_zdith <= f_zone(s_k_dith, s_zdith);
            s_zl    <= f_zone(s_k_lbits, s_zl);
            s_zc    <= f_zone(s_k_cbits, s_zc);
            -- per-field cell constants
            s_cmask <= C_MASK(to_integer(s_fcell));
            s_chalf <= C_HALF(to_integer(s_fcell));
            s_rmask <= C_MASK(to_integer(s_frows));
            s_mask11 <= resize(s_cmask, 11);
            s_half11 <= resize(s_chalf, 11);
            -- quantizer constants
            s_lstep <= shift_right(to_unsigned(512, 11), to_integer(s_zl));
            s_cstep <= shift_right(to_unsigned(512, 11), to_integer(s_zc));
            s_lqm <= not (s_lstep(9 downto 0) - 1);
            s_cqm <= not (s_cstep(9 downto 0) - 1);
            v_s8 := shift_right(s_lstep, 3);
            s_ld0 <= -signed(shift_right(s_lstep, 2) + v_s8); s_ld1 <= -signed(v_s8);
            s_ld2 <= signed(v_s8); s_ld3 <= signed(shift_right(s_lstep, 2) + v_s8);
            v_s8 := shift_right(s_cstep, 3);
            s_cd0 <= resize(v_s8, 10);
            s_cd1 <= resize(shift_right(s_cstep, 2) + v_s8, 10);
            s_cd2 <= resize(shift_right(s_cstep, 1) + v_s8, 10);
            s_cd3 <= resize(s_cstep - v_s8, 10);
            s_cqh <= resize(shift_right(s_cstep, 1), 10);
            -- shear range: width/64 .. width/2 (of 1920 units on the 4:3 SD window)
            s_floor <= resize(s_wsz(10 downto 6), 11);
            s_range <= resize(s_wsz(10 downto 1), 11) - resize(s_wsz(10 downto 6), 11);
            -- edge bounds
            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;
            s_w_edge <= s_line_width - s_cedge - 1;
            s_we13  <= signed(resize(s_w_edge, 13));
            s_wl13  <= signed(resize(s_line_width, 13));
            s_nwe13 <= -signed(resize(s_w_edge, 13));
            s_rce13 <= s_w2m1 - s_ce13;
            s_rwe13 <= signed(resize(s_line_width, 13)) + s_ce13;

            -- per-line shear: wave (or tear) x amplitude, one op per state
            case to_integer(s_ls) is
                when 0 =>
                    if s_sw_tear = '1' then
                        if s_rnd(31 downto 24) < s_prob or s_rnd(31 downto 26) = 0 then
                            s_w <= resize(signed(s_rnd(9 downto 0)), 11);
                        else
                            s_w <= (others => '0');
                        end if;
                    else
                        v_ph := s_tph(15 downto 5);
                        if v_ph(10) = '1' then v_t := not v_ph(9 downto 0); else v_t := v_ph(9 downto 0); end if;
                        s_w <= signed('0' & v_t) - 512;
                    end if;
                    s_ls <= s_ls + 1;
                when 1 =>
                    s_wneg <= s_w(10);
                    v_nw := -s_w;
                    if s_w(10) = '1' then s_lB <= unsigned(v_nw(9 downto 0));
                    else s_lB <= unsigned(s_w(9 downto 0)); end if;
                    s_lA <= resize(s_amp, 21);
                    s_lP <= (others => '0');
                    s_ls <= s_ls + 1;
                when 2 to 11 =>
                    if s_lB(0) = '1' then s_lP <= s_lP + s_lA; end if;
                    s_lA <= shift_left(s_lA, 1);
                    s_lB <= shift_right(s_lB, 1);
                    s_ls <= s_ls + 1;
                when 12 =>
                    v_o := signed(s_lP(20 downto 9));
                    if s_wneg = '1' then s_offr <= -v_o; else s_offr <= v_o; end if;
                    s_ls <= s_ls + 1;
                when 13 =>
                    -- Cells: round to the nearest whole cell
                    if s_sw_snap = '1' then
                        s_offs <= signed(unsigned(s_offr + signed(resize(s_chalf, 12))) and not resize(s_cmask, 12));
                    else
                        s_offs <= s_offr;
                    end if;
                    s_ls <= s_ls + 1;
                when 14 =>
                    s_offn <= s_offs;
                    if s_sw_split = '1' then s_offcn <= -s_offs; else s_offcn <= s_offs; end if;
                    s_ls <= s_ls + 1;
                when others => null;
            end case;

            -- line start: banks, capture, parity swap, offsets, dither row
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                v_cap := s_rowst;                     -- the line starts a cell row
                s_cap   <= v_cap;
                s_rbank <= s_lastcap;                 -- the last completed capture
                s_wbank <= not s_lastcap;
                if s_lastcap = '1' then s_swap <= s_hpar xor s_cpar1; else s_swap <= s_hpar xor s_cpar0; end if;
                if v_cap = '1' then
                    if s_lastcap = '1' then s_cpar0 <= s_hpar; else s_cpar1 <= s_hpar; end if;
                end if;
                -- Slide: a new offset only where a new cell row starts
                if s_sw_slide = '0' or s_prevcap = '1' then
                    s_offl <= s_offn; s_offcl <= s_offcn;
                end if;
                s_dline <= s_line(7 downto 0);
            end if;
            -- line end: next line's phase, start its shear
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_vrx(11 downto 1); s_line <= s_line + 1;
                -- cell rows in virtual lines (Q8; 2^rows units, so the wrap is a mask)
                v_ya := resize(s_ya, 17) + s_sty;
                s_ya <= (v_ya(15 downto 8) and ('0' & s_rmask)) & v_ya(7 downto 0);
                if (v_ya(16 downto 8) and not ("00" & s_rmask)) /= 0 then s_rowst <= '1'; else s_rowst <= '0'; end if;
                if s_cap = '1' then s_lastcap <= s_wbank; end if;
                s_prevcap <= s_cap;
                s_tph <= s_tph + s_stepv;
                s_ls <= (others => '0');
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
            end if;
            -- frame start (idempotent: vsync may serrate): line 0 captures
            -- into bank 0 and starts a cell row; the field's own phase
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_ya <= (others => '0'); s_rowst <= '1';
                s_lastcap <= '1'; s_prevcap <= '1';
                v_f := s_fph;
                if s_saw_active = '1' then
                    v_f := s_fph + C_SPEED;
                    -- glide P12 by 1/4 and K4 by 1/8 of the gap per field
                    v_tg := signed(resize(unsigned'(s_p_shear & "0000"), 15)); v_cur := signed(resize(s_gp, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 2);
                    s_gp <= unsigned(v_cur(13 downto 0));
                    v_tg := signed(resize(unsigned'(s_k_rate & "0000"), 15)); v_cur := signed(resize(s_gk, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 3);
                    s_gk <= unsigned(v_cur(13 downto 0));
                    s_fcell <= s_zcell; s_frows <= s_zrows; s_fdith <= s_zdith;
                end if;
                s_fph <= v_f; s_tph <= v_f;
                s_ls <= (others => '0');
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- frame constants, free-running: sq = P12^2, amp = floor + range*sq,
    -- step / tear share = K4^2 (the inputs only move at vsync)
    p_fmul : process(clk)
    begin
        if rising_edge(clk) then
            if s_mit = 0 then
                case to_integer(s_mph) is
                    when 0 => s_mA <= resize(s_gp(13 downto 2), 24); s_mB <= s_gp(13 downto 2);
                    when 1 => s_mA <= resize(s_range, 24);           s_mB <= s_sq;
                    when 2 => s_mA <= resize(s_gk(13 downto 2), 24); s_mB <= s_gk(13 downto 2);
                    when others => s_mA <= resize(s_step, 24);       s_mB <= s_sty;
                end case;
                s_mP <= (others => '0');
                s_mit <= s_mit + 1;
            elsif s_mit <= 12 then
                if s_mB(0) = '1' then s_mP <= s_mP + s_mA; end if;
                s_mA <= shift_left(s_mA, 1);
                s_mB <= shift_right(s_mB, 1);
                s_mit <= s_mit + 1;
            else
                case to_integer(s_mph) is
                    when 0 => s_sq <= s_mP(23 downto 12); s_mph <= "01";
                    when 1 => s_amp <= s_floor + s_mP(22 downto 12); s_mph <= "10";
                    when 2 => s_step <= s_mP(23 downto 11); s_prob <= s_mP(23 downto 16); s_mph <= "11";
                    -- zigzag step per field line: step x (1080 / active lines)
                    when others => s_stepv <= s_mP(23 downto 8); s_mph <= "00";
                end case;
                s_mit <= (others => '0');
            end if;
        end if;
    end process p_fmul;

    -- read address: 2 x + shear, 3 edge fill / mirror, 4 cell centre, 5 inset
    -- clamp (all virtual), 6..9 to a pixel
    p_addr : process(clk)
        variable v_cy, v_cc : unsigned(18 downto 0);
        procedure edge(signal src : in signed(12 downto 0); signal addr : out unsigned(10 downto 0)) is
            variable n, r : signed(12 downto 0);
        begin
            n := -src; r := s_w2m1 - src;
            if s_sw_mirror = '1' and src < 0 then
                if src > s_nce13 then addr <= s_cedge;
                elsif src < s_nwe13 then addr <= s_w_edge;
                else addr <= unsigned(n(10 downto 0)); end if;
            elsif s_sw_mirror = '1' and src >= s_wl13 then
                if src > s_rce13 then addr <= s_cedge;
                elsif src < s_rwe13 then addr <= s_w_edge;
                else addr <= unsigned(r(10 downto 0)); end if;
            elsif src < s_ce13 then addr <= s_cedge;
            elsif src > s_we13 then addr <= s_w_edge;
            else addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
        procedure inset(signal h : in unsigned(10 downto 0); signal addr : out unsigned(10 downto 0)) is
        begin
            if h < s_cedge then addr <= s_cedge;
            elsif h > s_w_edge then addr <= s_w_edge;
            else addr <= h; end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_xs(2) <= s_wr_x;
            for i in 3 to 10 loop s_xs(i) <= s_xs(i - 1); end loop;
            -- 2 (virtual x)
            s_a2 <= signed(resize(s_vwx(11 downto 1), 13)) + resize(s_offl, 13);
            s_c2 <= signed(resize(s_vwx(11 downto 1), 13)) + resize(s_offcl, 13);
            -- 3
            edge(s_a2, s_e3);
            edge(s_c2, s_ec3);
            -- 4: hold to the centre of the cell
            s_h4  <= (s_e3 and not s_mask11) or s_half11;
            s_hc4 <= (s_ec3 and not s_mask11) or s_half11;
            -- 5
            inset(s_h4, s_vY5);
            inset(s_hc4, s_vC5);
            s_cb(5) <= s_h4(to_integer(s_fcell));     -- cell-index parity (Cell dither)
            for i in 6 to 10 loop s_cb(i) <= s_cb(i - 1); end loop;
            -- 6..9: virtual x -> line-buffer pixel (the one under the unit's
            -- centre): 1080 V, SD V/2, 720p floor((2V+1)/3) =
            -- ((V+1) x 21845) >> 15, as x5, x17, x257
            s_pY6 <= resize(s_vY5, 12) + 1;                 s_pC6 <= resize(s_vC5, 12) + 1;
            s_vY6 <= s_vY5;                                 s_vC6 <= s_vC5;
            s_tY7 <= resize(s_pY6, 14) + shift_left(resize(s_pY6, 14), 2);
            s_tC7 <= resize(s_pC6, 14) + shift_left(resize(s_pC6, 14), 2);
            s_vY7 <= s_vY6;                                 s_vC7 <= s_vC6;
            s_tY8 <= resize(s_tY7, 18) + shift_left(resize(s_tY7, 18), 4);
            s_tC8 <= resize(s_tC7, 18) + shift_left(resize(s_tC7, 18), 4);
            s_vY8 <= s_vY7;                                 s_vC8 <= s_vC7;
            if s_m720 = '1' then
                v_cy := resize(s_tY8, 19) + resize(s_tY8(17 downto 8), 19);
                v_cc := resize(s_tC8, 19) + resize(s_tC8(17 downto 8), 19);
                s_addrY <= v_cy(17 downto 7); s_addrC <= v_cc(17 downto 7);
            elsif s_msd = '1' then
                s_addrY <= '0' & s_vY8(10 downto 1); s_addrC <= '0' & s_vC8(10 downto 1);
            else
                s_addrY <= s_vY8; s_addrC <= s_vC8;
            end if;
        end if;
    end process p_addr;

    -- line buffers: written on capture lines at stage 1, read at 10
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_avid_sr(0) = '1' and s_cap = '1' and s_wbank = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_avid_sr(0) = '1' and s_cap = '1' and s_wbank = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(0) = '1' and s_cap = '1' and s_wbank = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(0) = '1' and s_cap = '1' and s_wbank = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(0) = '1' and s_cap = '1' and s_wbank = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(0) = '1' and s_cap = '1' and s_wbank = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_xb, v_yb : std_logic;
        variable v_j  : integer range 0 to 7;
        variable v_dc : unsigned(9 downto 0);
        variable v_n  : unsigned(10 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_dy : signed(10 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 7: fetch the last capture's bank; U/V swapped where this row's
            --    parity differs from the capture row's.  Dither cell index.
            if s_rbank = '1' then
                v_y := unsigned(s_rdY1); v_u := unsigned(s_rdU1) & "00"; v_v := unsigned(s_rdV1) & "00";
            else
                v_y := unsigned(s_rdY0); v_u := unsigned(s_rdU0) & "00"; v_v := unsigned(s_rdV0) & "00";
            end if;
            s_y7 <= v_y;
            if s_swap = '1' then s_u7 <= v_v; s_v7 <= v_u; else s_u7 <= v_u; s_v7 <= v_v; end if;
            v_j := to_integer(s_fdith);
            if v_j = 7 then
                v_xb := s_cb(10); v_yb := s_rbank;       -- Cell: cell column / cell row parity
            else
                v_xb := s_xs(10)(v_j); v_yb := s_dline(v_j);
            end if;
            s_db7 <= (v_xb xor v_yb) & v_yb;            -- 2x2 Bayer index
            if v_j = 0 then s_don7 <= '0'; else s_don7 <= '1'; end if;
            -- 8: luma - black; chroma + rounding / dither offset
            s_db8 <= s_db7; s_don8 <= s_don7;
            if s_y7 < 64 then s_a8 <= (others => '0'); else s_a8 <= s_y7 - 64; end if;
            if s_don7 = '1' then
                case to_integer(s_db7) is
                    when 0 => v_dc := s_cd0;
                    when 1 => v_dc := s_cd1;
                    when 2 => v_dc := s_cd2;
                    when others => v_dc := s_cd3;
                end case;
            else
                v_dc := s_cqh;
            end if;
            s_cu8 <= resize(s_u7, 11) + resize(v_dc, 11);
            s_cv8 <= resize(s_v7, 11) + resize(v_dc, 11);
            -- 9: luma * 1.168 (= 1023/876) in two halves; chroma level
            s_db9 <= s_db8; s_don9 <= s_don8;
            s_n19 <= resize(s_a8, 11) + resize(shift_right(s_a8, 3), 11);
            s_n29 <= resize(shift_right(s_a8, 5), 7) + resize(shift_right(s_a8, 7), 7) + resize(shift_right(s_a8, 8), 7);
            s_ucd(9) <= f_cq(s_cu8, s_cqm);
            s_vcd(9) <= f_cq(s_cv8, s_cqm);
            for i in 10 to 14 loop s_ucd(i) <= s_ucd(i - 1); s_vcd(i) <= s_vcd(i - 1); end loop;
            -- 10: normalized luma 0..1023
            s_db10 <= s_db9; s_don10 <= s_don9;
            v_n := s_n19 + resize(s_n29, 11);
            if v_n(10) = '1' then s_yn10 <= (others => '1'); else s_yn10 <= v_n(9 downto 0); end if;
            -- 11: dither (signed, mean 0)
            if s_don10 = '1' then
                case to_integer(s_db10) is
                    when 0 => v_dy := s_ld0;
                    when 1 => v_dy := s_ld1;
                    when 2 => v_dy := s_ld2;
                    when others => v_dy := s_ld3;
                end case;
            else
                v_dy := (others => '0');
            end if;
            v_t := signed(resize(s_yn10, 12)) + resize(v_dy, 12);
            if v_t < 0 then s_t11 <= (others => '0');
            elsif v_t > 1023 then s_t11 <= (others => '1');
            else s_t11 <= unsigned(v_t(9 downto 0)); end if;
            -- 12: keep L bits, replicate (level k of N -> k * 1023 / (N-1))
            s_rep12 <= f_rep10(s_t11, s_zl);
            -- 13, 14: back to the video range, 64 + rep * 0.855
            s_d113 <= s_rep12 - shift_right(s_rep12, 3);
            s_d213 <= resize(shift_right(s_rep12, 6), 5) + resize(shift_right(s_rep12, 8), 5);
            s_yq14 <= 64 + s_d113 - resize(s_d213, 10);
            -- 15: neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(17) = '1' then
                s_y15 <= s_yq14; s_u15 <= s_ucd(14); s_v15 <= s_vcd(14);
            else
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            end if;
            -- 16, 17: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y15; s_g2y <= s_g1y;
            s_g1u <= s_u15; s_g2u <= s_g1u;
            s_g1v <= s_v15; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y15, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u15, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v15, 12);
            s_y17 <= v_sy(11 downto 2); s_u17 <= v_su(11 downto 2); s_v17 <= v_sv(11 downto 2);
            -- 18: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            v_d := s_y17 - 64;
            if s_avid_sr(20) = '1' then
                s_y18 <= s_y17 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y18);
    s_io_align_in.u       <= std_logic_vector(s_u18);
    s_io_align_in.v       <= std_logic_vector(s_v18);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (22 -> 24).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 1 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(1);

end architecture shearposter;
