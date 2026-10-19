-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Foldither: catalogue combo #117 (seed 20260926) -- ordered dithering
-- (Bayer) + mirror / kaleidoscope.  Three knobs each; the switches and the
-- slider decide how they combine.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The fold runs in virtual
-- 1080-line units (mondrian's scheme: x step 1 / 1.5 / 2 units a pixel at
-- 1080 / 720p / SD from the timing ID), so a Segment is the same fraction
-- of the screen HEIGHT in every mode (SD: a 1440-unit 4:3 window) and Skew
-- keeps its angle (lean per field line x the virtual y step).  The source
-- column is the fold result / x step, tracked by a per-pixel counter that
-- restarts at each fold period (no per-pixel divide); its line-start value
-- is a pipelined /3 (720p) or shift.  1080p bit-identical.  The Bayer
-- dither stays at pixel scale.
--
--   K1 Segment  pane width 1024 / 512 / 256 / 128 / 64 px (1080p pixels;
--               the same fraction of the screen height in every mode)
--   K2 Slide    the panes slide across the picture, bipolar, squared
--               (centre detent; top ~7.5 px a frame)
--   K3 Skew     per-line lean of the panes, bipolar, squared (centre
--               detent; 45 degrees at ~35%, top ~7.5 px a line)
--   K4 Bits     luma levels 1..6 bits (black and white are always levels)
--   K5 Dither   dither amount, 0..100% of a step
--   K6 Matrix   2x2 / 4x4 / 8x8 Bayer / diagonal line screen
--
--   S7  Fold     mirror (triangle) / repeat (tiles of two segments)
--   S8  Pattern  dither pattern in source space (it folds with the
--                picture) / in screen space (it stays put)
--   S9  Chroma   clean / quantized and dithered like luma (centred levels)
--   S10 Alt Neg  alternate panes negative
--   S11 Alt Bits alternate panes 2 bits coarser (2 finer at 1-2 bits)
--   P12 Panes    pane width / 1, 2, 4, 8
--
-- Fold: v0.1 nested up to four halving folds, but a halving fold of a
-- triangle wave is the same triangle wave at the smaller width
-- (tri(tri(a, W), W/2) = tri(a, W/2); repeat: (a mod 2W) mod W = a mod W),
-- so the cascade is ONE fold at the final width W = 2^(10 - K1 - P12):
-- mirror t = |(a mod 2W) - W|, src = cx - W/2 + t; repeat t = a mod 2W,
-- src = cx - W + t.  Reads that leave the line reflect back inside it
-- (inset C_EDGE columns, the capture's black edge).
--
-- Quantizer: one, after the fetch.  The line buffer holds the raw picture;
-- Source mode takes the dither cell from the SOURCE column and line (the
-- fetch address, line - 1), Screen mode from the screen pixel -- the same
-- pattern v0.1 made with a second quantizer on the write port.  Luma is
-- normalized to 0..1023, scaled by (N-1)/N, offset half a step plus the
-- centred dither ((2t + 1 - 64)/128 of Dither x step), cut to L bits and
-- bit-replicated back to 64..940: mean-preserving ordered dither, black
-- and white always levels.  Chroma is a centred mid-tread quantizer
-- (levels 512 +- k*step, clamped 64..960) so greys stay grey.
--
-- Stages are numbered from the input register (stage 1 = s_in_*, s_wr_x;
-- s_avid_sr(k) is the avid of the stage-(k+1) pixel); output at 20.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (20) in all modes; write
-- bank, line number and pane offset latched per line at the avid rise
-- (the pipeline tail runs past hsync on SD/PAL front porches); U/V swapped
-- on rows whose hsync->avid parity differs from the row above (each row
-- replays the row above -- drape v1.3); centre from a field-latched width
-- (analog line lengths wander +-1 px); output conditioning: luma scaled to
-- white ~775 and [1 2 1] on U/V (damier v0.4.4's HD HDMI bar fix minus the
-- luma smoothing, which would erase a 1-px dither); no_rw_check on every
-- RAM (a bank is only read while the other is written; the write bank's
-- read data is discarded).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture foldither of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LAT  : integer := 20;
    constant C_EDGE : integer := 12;

    type t_b8 is array (0 to 63) of integer range 0 to 63;
    constant C_B8 : t_b8 := (0, 32, 8, 40, 2, 34, 10, 42, 48, 16, 56, 24, 50, 18, 58, 26, 12, 44, 4, 36, 14, 46, 6, 38, 60, 28, 52, 20, 62, 30, 54, 22, 3, 35, 11, 43, 1, 33, 9, 41, 51, 19, 59, 27, 49, 17, 57, 25, 15, 47, 7, 39, 13, 45, 5, 37, 63, 31, 55, 23, 61, 29, 53, 21);
    type t_b4 is array (0 to 15) of integer range 0 to 15;
    constant C_B4 : t_b4 := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);

    -- dither threshold 0..63 (the cell's level is (t + 0.5)/64)
    function f_thr(mat : unsigned(1 downto 0); bx, by : unsigned(2 downto 0)) return unsigned is
        variable v_t  : unsigned(5 downto 0);
        variable v_i4 : unsigned(3 downto 0);
        variable v_i8 : unsigned(5 downto 0);
        variable v_d  : unsigned(2 downto 0);
    begin
        v_i4 := by(1 downto 0) & bx(1 downto 0);
        v_i8 := by & bx;
        v_d  := bx + by;
        case to_integer(mat) is
            when 0 =>
                case to_integer(unsigned'(by(0) & bx(0))) is
                    when 0 => v_t := "001000"; when 1 => v_t := "101000"; when 2 => v_t := "111000"; when others => v_t := "011000";
                end case;
            when 1 => v_t := to_unsigned(C_B4(to_integer(v_i4)), 4) & "10";
            when 2 => v_t := to_unsigned(C_B8(to_integer(v_i8)), 6);
            when others => v_t := v_d & "100";                          -- 45-degree line screen
        end case;
        return v_t;
    end function;

    -- label zone of a knob: n even zones, the firmware's label split
    function f_zone(v : unsigned(10 downto 0); n : integer) return integer is
        variable z : integer range 0 to 7;
    begin
        z := 0;
        for k in 1 to 7 loop
            if k < n then
                if v >= (k * 1024 + n - 1) / n then z := k; end if;
            end if;
        end loop;
        return z;
    end function;

    -- top L = z+1 bits of t, replicated down to 10 bits (level k of N -> k*1023/(N-1))
    function f_rep10(t : unsigned(9 downto 0); z : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(z) is
            when 0 => r := (others => t(9));
            when 1 => r := t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8);
            when 2 => r := t(9 downto 7) & t(9 downto 7) & t(9 downto 7) & t(9);
            when 3 => r := t(9 downto 6) & t(9 downto 6) & t(9 downto 8);
            when 4 => r := t(9 downto 5) & t(9 downto 5);
            when others => r := t(9 downto 4) & t(9 downto 6);
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

    -- per-depth quantizer constants (L = 1..6 bits, step = 1024 >> L)
    function f_dsc(d8 : unsigned(7 downto 0); l : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(l) is                                         -- step * d8 / 256
            when 1 => r := "0" & d8 & "0";
            when 2 => r := "00" & d8;
            when 3 => r := "000" & d8(7 downto 1);
            when 4 => r := "0000" & d8(7 downto 2);
            when 5 => r := "00000" & d8(7 downto 3);
            when others => r := "000000" & d8(7 downto 4);
        end case;
        return r;
    end function;
    function f_half(l : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        r := shift_right(to_unsigned(512, 10), to_integer(l));
        return r(8 downto 0);
    end function;
    function f_cmask(l : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(10 downto 0);
    begin
        r := shift_right(to_unsigned(1024, 11), to_integer(l)) - 1;
        return not r(9 downto 0);
    end function;

    -- Resolution scaling (mondrian's scheme): x step in half virtual units
    -- per pixel -- 1080 2 (1.0), 720p 3 (1.5), SD 4 (2.0: 720 px span 1440
    -- units, a 4:3 window of the 1080 canvas).
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- floor(2^k / 3): a power-of-2 virtual length / 1.5 (720p), k = 2..11
    function f_d3(k : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(10 downto 0);
    begin
        case to_integer(k) is
            when 2 => r := to_unsigned(1, 11);   when 3 => r := to_unsigned(2, 11);
            when 4 => r := to_unsigned(5, 11);   when 5 => r := to_unsigned(10, 11);
            when 6 => r := to_unsigned(21, 11);  when 7 => r := to_unsigned(42, 11);
            when 8 => r := to_unsigned(85, 11);  when 9 => r := to_unsigned(170, 11);
            when 10 => r := to_unsigned(341, 11); when others => r := to_unsigned(682, 11);
        end case;
        return r;
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    signal s_k_seg, s_k_slide, s_k_skew, s_k_bits, s_k_dith, s_k_mat : unsigned(9 downto 0);
    signal s_sw_repeat, s_sw_screen, s_sw_chroma, s_sw_altneg, s_sw_altbits : std_logic;
    signal s_p_panes : unsigned(9 downto 0);

    -- labelled knobs: 0 K1 Segment, 1 K4 Bits, 2 K6 Matrix, 3 P12 Panes
    type t_int4 is array (0 to 3) of integer;
    constant C_NLBL : t_int4 := (5, 6, 4, 4);
    type t_u10x4 is array (0 to 3) of unsigned(9 downto 0);
    type t_u11x4 is array (0 to 3) of unsigned(10 downto 0);
    type t_u3x4  is array (0 to 3) of unsigned(2 downto 0);
    signal s_kv, s_kv2 : t_u10x4 := (others => (others => '0'));
    signal s_klo, s_khi : t_u11x4 := (others => (others => '0'));
    signal s_kix : std_logic_vector(0 to 3) := (others => '0');
    signal s_z : t_u3x4 := (to_unsigned(1, 3), to_unsigned(2, 3), to_unsigned(2, 3), to_unsigned(1, 3));

    -- geometry (per frame / field)
    signal s_e      : unsigned(3 downto 0) := to_unsigned(8, 4);         -- log2 pane width, 3..10
    signal s_w      : unsigned(11 downto 0) := to_unsigned(256, 12);     -- pane width W
    signal s_pbit   : unsigned(11 downto 0) := to_unsigned(256, 12);     -- pane parity bit (W, or 2W for tiles)
    signal s_boff   : unsigned(10 downto 0) := to_unsigned(128, 11);     -- W/2 (mirror) or W (repeat)
    signal s_wf     : unsigned(10 downto 0) := to_unsigned(1920, 11);    -- field-latched line width
    signal s_cx     : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_bsrc   : signed(12 downto 0) := (others => '0');            -- cx - boff
    signal s_hi, s_hi2, s_lo2mhi, s_hi2mlo : signed(12 downto 0) := (others => '0');
    -- slide / skew (12.11 px, wrap at 4096 px -- a multiple of every 2W)
    signal s_sd, s_kd : unsigned(8 downto 0) := (others => '0');
    signal s_sneg, s_kneg, s_sneg2, s_kneg2 : std_logic := '0';
    signal s_sp, s_kp : unsigned(14 downto 0) := (others => '0');
    signal s_sstep, s_kstep : unsigned(22 downto 0) := (others => '0');
    signal s_slide, s_shrow : unsigned(22 downto 0) := (others => '0');
    -- quantizer constants (L bits, alternate panes Lb bits)
    signal s_l, s_lb : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_dsc, s_dscb : unsigned(9 downto 0) := (others => '0');
    signal s_half, s_halfb : unsigned(8 downto 0) := (others => '0');
    signal s_cmask, s_cmaskb : unsigned(9 downto 0) := (others => '1');
    signal s_mat : unsigned(1 downto 0) := "10";
    signal s_saw_active : std_logic := '0';

    -- Resolution scaling: x step in half virtual units per pixel (1080 2,
    -- 720p 3, SD 4), decoded every clock from the timing ID; fold geometry in
    -- virtual units (half units for the per-pixel phase), source columns in
    -- real pixels (virtual / x step).
    signal s_tid    : std_logic_vector(3 downto 0) := "0011";
    signal s_xsh    : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_cxv    : unsigned(11 downto 0) := to_unsigned(960, 12);    -- virtual centre
    signal s_maskh  : unsigned(11 downto 0) := to_unsigned(1023, 12);   -- 4W - 1 (half units)
    signal s_pbith  : unsigned(12 downto 0) := to_unsigned(512, 13);    -- 2 * pbit
    signal s_wr     : unsigned(10 downto 0) := to_unsigned(256, 11);    -- W / x step (real px)
    signal s_boffr  : unsigned(10 downto 0) := to_unsigned(128, 11);    -- boff / x step
    signal s_kadd, s_kmlo : signed(12 downto 0) := (others => '0');     -- src = kadd + m / kmlo - m
    -- next line's phase (half units) and its source counter start m / x step
    signal s_bnh    : unsigned(12 downto 0) := (others => '0');
    signal s_mh0    : unsigned(11 downto 0) := (others => '0');
    signal s_n13    : unsigned(12 downto 0) := (others => '0');
    signal s_n5     : unsigned(14 downto 0) := (others => '0');
    signal s_n273   : unsigned(22 downto 0) := (others => '0');
    signal s_q3     : unsigned(10 downto 0) := (others => '0');
    signal s_mr0, s_mr0l : unsigned(10 downto 0) := (others => '0');
    -- skew in virtual units per field line: kp * y step (two shift-add stages)
    signal s_kpa, s_kpb : signed(18 downto 0) := (others => '0');
    signal s_kpv    : unsigned(17 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    -- per line, latched at the avid rise
    signal s_lw : std_logic := '0';                                      -- bank this line writes
    signal s_lnum, s_lm1 : unsigned(2 downto 0) := (others => '0');     -- screen / source dither row
    signal s_base_h : unsigned(12 downto 0) := (others => '0');          -- 2 * (pane offset - cx), virtual
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';

    -- address path
    type t_x3 is array (2 to 6) of unsigned(2 downto 0);
    signal s_xs : t_x3 := (others => (others => '0'));                  -- screen x(2:0) by stage
    signal s_par : std_logic_vector(3 to 16) := (others => '0');         -- alternate pane by stage
    signal s_ah  : unsigned(12 downto 0) := (others => '0');            -- fold phase, virtual half units
    signal s_mr3 : unsigned(10 downto 0) := (others => '0');            -- (phase mod 2W) / x step, real px
    signal s_srca4, s_srcb4 : signed(12 downto 0) := (others => '0');
    signal s_sel4 : std_logic := '0';
    signal s_src5 : signed(12 downto 0) := (others => '0');
    signal s_addr6 : unsigned(10 downto 0) := (others => '0');
    -- quantizer path
    signal s_thr7  : unsigned(5 downto 0) := (others => '0');
    signal s_dsc7  : unsigned(9 downto 0) := (others => '0');
    signal s_half7, s_half8 : unsigned(8 downto 0) := (others => '0');
    signal s_cm7, s_cm8, s_cm9, s_cm10 : unsigned(9 downto 0) := (others => '1');
    signal s_l7, s_l8, s_l9, s_l10, s_l11, s_l12, s_l13 : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_y8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u8, s_v8, s_u9, s_v9, s_u10, s_v10, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d8  : signed(17 downto 0) := (others => '0');
    signal s_ya9 : unsigned(9 downto 0) := (others => '0');
    signal s_hd9, s_hd10, s_hd11, s_hd12 : unsigned(8 downto 0) := (others => '0');
    signal s_n1_10 : unsigned(10 downto 0) := (others => '0');
    signal s_n2_10 : unsigned(6 downto 0) := (others => '0');
    signal s_cu10, s_cv10 : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal s_cuq11, s_cvq11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yn11, s_ys12 : unsigned(9 downto 0) := (others => '0');
    signal s_t13, s_rep14 : unsigned(9 downto 0) := (others => '0');
    signal s_e1_15 : unsigned(9 downto 0) := (others => '0');
    signal s_e2_15 : unsigned(4 downto 0) := (others => '0');
    signal s_yq16, s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_c is array (12 to 17) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_c := (others => to_unsigned(512, 10));         -- output chroma by stage
    -- output conditioning: [1 2 1] window on U/V, g1 the centre
    signal s_g1y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');                 -- s_avid_sr(k) = avid of the stage-(k+1) pixel

begin

    s_k_seg   <= unsigned(registers_in(0));
    s_k_slide <= unsigned(registers_in(1));
    s_k_skew  <= unsigned(registers_in(2));
    s_k_bits  <= unsigned(registers_in(3));
    s_k_dith  <= unsigned(registers_in(4));
    s_k_mat   <= unsigned(registers_in(5));
    s_sw_repeat  <= registers_in(6)(0);
    s_sw_screen  <= registers_in(6)(1);
    s_sw_chroma  <= registers_in(6)(2);
    s_sw_altneg  <= registers_in(6)(3);
    s_sw_altbits <= registers_in(6)(4);
    s_p_panes <= unsigned(registers_in(7));

    s_kv(0) <= s_k_seg; s_kv(1) <= s_k_bits; s_kv(2) <= s_k_mat; s_kv(3) <= s_p_panes;

    p_ctrl : process(clk)
        variable v_v  : unsigned(9 downto 0);
        variable v_d  : unsigned(9 downto 0);
        variable v_zl, v_zh : integer range 0 to 7;
        variable v_w  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_avid_sr(0) = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- labelled knobs: a value below the label count is the firmware's
            -- power-on label index; otherwise the even label zone, changed only
            -- once the knob is 12 counts inside it
            for i in 0 to 3 loop
                v_v := s_kv(i);
                s_kv2(i) <= v_v;
                if v_v < C_NLBL(i) then s_kix(i) <= '1'; else s_kix(i) <= '0'; end if;
                if v_v < 12 then s_klo(i) <= (others => '0'); else s_klo(i) <= resize(v_v, 11) - 12; end if;
                s_khi(i) <= resize(v_v, 11) + 12;
                v_zl := f_zone(s_klo(i), C_NLBL(i));
                v_zh := f_zone(s_khi(i), C_NLBL(i));
                if s_kix(i) = '1' then
                    s_z(i) <= s_kv2(i)(2 downto 0);
                elsif v_zl = v_zh then
                    s_z(i) <= to_unsigned(v_zl, 3);
                end if;
            end loop;

            -- geometry: W = 2^(10 - K1 - P12) = 1024 .. 8 virtual units (1080p
            -- px); the fold runs in virtual units, sources are real px
            s_tid <= registers_in(8)(3 downto 0);
            s_xsh <= f_vstep_x(s_tid);
            s_e <= to_unsigned(10, 4) - resize(s_z(0), 4) - resize(s_z(3), 4);
            v_w := shift_left(to_unsigned(1, 12), to_integer(s_e));
            s_w <= v_w;
            s_maskh <= shift_left(v_w, 2) - 1;
            if s_sw_repeat = '1' then
                s_pbit <= shift_left(v_w, 1); s_boff <= v_w(10 downto 0);
            else
                s_pbit <= v_w; s_boff <= v_w(11 downto 1);
            end if;
            s_pbith <= shift_left(resize(s_pbit, 13), 1);
            -- W and boff in real px: / 1 (1080), / 1.5 (720p), / 2 (SD)
            case to_integer(s_xsh) is
                when 3 =>
                    s_wr <= f_d3(s_e + 1);
                    if s_sw_repeat = '1' then s_boffr <= f_d3(s_e + 1); else s_boffr <= f_d3(s_e); end if;
                when 4 =>
                    s_wr <= s_w(11 downto 1);
                    s_boffr <= '0' & s_boff(10 downto 1);
                when others =>
                    s_wr <= s_w(10 downto 0);
                    s_boffr <= s_boff;
            end case;
            s_cx <= '0' & s_wf(10 downto 1);
            -- virtual centre: (real width * x step) / 2
            case to_integer(s_xsh) is
                when 3 => s_cxv <= resize(s_wf(10 downto 1), 12) + resize(s_wf(10 downto 2), 12);
                when 4 => s_cxv <= resize(s_wf, 12);
                when others => s_cxv <= resize(s_wf(10 downto 1), 12);
            end case;
            s_bsrc <= signed(resize(s_cx, 13)) - signed(resize(s_boffr, 13));
            -- source = kadd + m (repeat; mirror past the turn) / kmlo - m
            if s_sw_repeat = '1' then
                s_kadd <= s_bsrc;
            else
                s_kadd <= s_bsrc - signed(resize(s_wr, 13));
            end if;
            s_kmlo <= s_bsrc + signed(resize(s_wr, 13));
            -- next line's fold phase (half units) and the source counter's
            -- start (phase mod 2W) / x step: shifts, or floor((m+1)*1365/4096)
            -- = floor(m/3) exactly for m < 4096 at 720p.  Quasi-static (shrow
            -- moves at the avid fall), latched at the avid rise.
            s_bnh  <= (s_shrow(22 downto 11) - s_cxv) & '0';
            s_mh0  <= s_bnh(11 downto 0) and s_maskh;
            s_n13  <= resize(s_mh0, 13) + 1;
            s_n5   <= resize(s_n13, 15) + shift_left(resize(s_n13, 15), 2);
            s_n273 <= shift_left(resize(s_n5, 23), 8) + shift_left(resize(s_n5, 23), 4);
            s_q3   <= resize(shift_right(s_n273 + resize(s_n5, 23), 12), 11);
            case to_integer(s_xsh) is
                when 3 => s_mr0 <= s_q3;
                when 4 => s_mr0 <= resize(s_mh0(11 downto 2), 11);
                when others => s_mr0 <= s_mh0(11 downto 1);
            end case;
            s_hi <= signed(resize(s_wf, 13)) - (C_EDGE + 1);                -- last column inside the inset
            s_hi2 <= shift_left(s_hi, 1);
            s_lo2mhi <= to_signed(2 * C_EDGE, 13) - s_hi;                   -- below: the reflection passes hi
            s_hi2mlo <= shift_left(s_hi, 1) - C_EDGE;                       -- above: the reflection passes lo

            -- K2 Slide / K3 Skew: |k - 512| less a 16-count detent, then
            -- d * (d/16 + 1) in 1/2048 px (squared; top 496*32/2048 = 7.75 px)
            if s_k_slide >= 512 then v_d := s_k_slide - 512; else v_d := 512 - s_k_slide; end if;
            if v_d > 16 then s_sd <= resize(v_d - 16, 9); else s_sd <= (others => '0'); end if;
            if s_k_slide < 512 then s_sneg <= '1'; else s_sneg <= '0'; end if;
            if s_k_skew >= 512 then v_d := s_k_skew - 512; else v_d := 512 - s_k_skew; end if;
            if v_d > 16 then s_kd <= resize(v_d - 16, 9); else s_kd <= (others => '0'); end if;
            if s_k_skew < 512 then s_kneg <= '1'; else s_kneg <= '0'; end if;
            s_sp <= s_sd * (resize(s_sd(8 downto 4), 6) + 1);
            s_kp <= s_kd * (resize(s_kd(8 downto 4), 6) + 1);
            -- Skew in virtual units per FIELD line: kp * y step (1080 / active
            -- lines, x2 interlaced), as two shift-add pairs
            case to_integer(unsigned(s_tid)) is
                when 0 =>                                                   -- 480i 4.445
                    s_kpa <= signed(resize(shift_left(resize(s_kp, 18), 2), 19)) + signed(resize(shift_right(s_kp, 1), 19));
                    s_kpb <= signed(resize(shift_right(s_kp, 7), 19)) - signed(resize(shift_right(s_kp, 4), 19));
                when 8 =>                                                   -- 576i 3.75
                    s_kpa <= signed(resize(shift_left(resize(s_kp, 18), 2), 19)) - signed(resize(shift_right(s_kp, 2), 19));
                    s_kpb <= (others => '0');
                when 4 =>                                                   -- 480p 2.25
                    s_kpa <= signed(resize(shift_left(resize(s_kp, 18), 1), 19)) + signed(resize(shift_right(s_kp, 2), 19));
                    s_kpb <= (others => '0');
                when 12 =>                                                  -- 576p 1.875
                    s_kpa <= signed(resize(shift_left(resize(s_kp, 18), 1), 19)) - signed(resize(shift_right(s_kp, 3), 19));
                    s_kpb <= (others => '0');
                when 5 | 6 | 14 =>                                          -- 720p 1.5
                    s_kpa <= signed(resize(s_kp, 19)) + signed(resize(shift_right(s_kp, 1), 19));
                    s_kpb <= (others => '0');
                when 1 | 2 | 10 =>                                          -- 1080i 2
                    s_kpa <= signed(resize(shift_left(resize(s_kp, 18), 1), 19));
                    s_kpb <= (others => '0');
                when others =>                                              -- 1080p 1
                    s_kpa <= signed(resize(s_kp, 19));
                    s_kpb <= (others => '0');
            end case;
            s_kpv <= unsigned(resize(s_kpa + s_kpb, 18));
            s_sneg2 <= s_sneg; s_kneg2 <= s_kneg;
            if s_sneg2 = '1' then s_sstep <= to_unsigned(0, 23) - resize(s_sp, 23); else s_sstep <= resize(s_sp, 23); end if;
            if s_kneg2 = '1' then s_kstep <= to_unsigned(0, 23) - resize(s_kpv, 23); else s_kstep <= resize(s_kpv, 23); end if;

            -- quantizer constants: L = Bits, Lb = 2 coarser (2 finer at 1-2 bits)
            s_l <= s_z(1) + 1;
            if s_l >= 3 then s_lb <= s_l - 2; else s_lb <= s_l + 2; end if;
            s_dsc  <= f_dsc(s_k_dith(9 downto 2), s_l);
            s_dscb <= f_dsc(s_k_dith(9 downto 2), s_lb);
            s_half <= f_half(s_l);   s_halfb <= f_half(s_lb);
            s_cmask <= f_cmask(s_l); s_cmaskb <= f_cmask(s_lb);
            s_mat <= s_z(2)(1 downto 0);

            -- line events
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                -- this line writes bank s_lw and reads the other (the line above)
                s_lw <= not s_lw;
                s_lnum <= s_line(2 downto 0);
                s_lm1 <= s_line(2 downto 0) - 1;
                s_base_h <= s_bnh;
                s_mr0l   <= s_mr0;
                s_line_par <= s_hpar;
                if s_lw = '0' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_lw = '1' then s_rowswap <= s_line_par xor s_cap_par0;
            else s_rowswap <= s_line_par xor s_cap_par1; end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_shrow <= s_shrow + s_kstep;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                if s_saw_active = '1' then
                    s_slide <= s_slide + s_sstep;
                    s_wf <= s_line_width;
                end if;
                s_shrow <= s_slide;
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

    -- address path, stages 2..6 (stage 1 = s_wr_x)
    p_addr : process(clk)
        variable v_rl, v_rh : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_xs(2) <= s_wr_x(2 downto 0);
            for i in 3 to 6 loop s_xs(i) <= s_xs(i - 1); end loop;
            for i in 4 to 16 loop s_par(i) <= s_par(i - 1); end loop;
            -- 2: virtual position relative to the centre plus slide / skew, in
            --    half units (mod 4096 units): x step per pixel from the line's
            --    start phase
            if s_avid_sr(0) = '1' and s_avid_sr(1) = '0' then
                s_ah <= s_base_h;
            else
                s_ah <= s_ah + s_xsh;
            end if;
            -- 3: pane (or tile) parity; m = (phase mod 2W) / x step in real px,
            --    a counter that restarts where the phase wraps (1080p: m = the
            --    phase itself)
            if (s_ah and s_pbith) /= 0 then s_par(3) <= '1'; else s_par(3) <= '0'; end if;
            if s_avid_sr(1) = '1' and s_avid_sr(2) = '0' then
                s_mr3 <= s_mr0l;
            elsif (s_ah(11 downto 2) and s_maskh(11 downto 2)) = 0 and s_ah(1 downto 0) < s_xsh then
                s_mr3 <= (others => '0');
            else
                s_mr3 <= s_mr3 + 1;
            end if;
            -- 4: source column, both arms (mirror: |m - W| about the turn;
            --    repeat: m); 5: pick
            s_srca4 <= s_kadd + signed(resize(s_mr3, 13));
            s_srcb4 <= s_kmlo - signed(resize(s_mr3, 13));
            s_sel4  <= s_sw_repeat or s_par(3);
            if s_sel4 = '1' then s_src5 <= s_srca4; else s_src5 <= s_srcb4; end if;
            -- 6: off the line -> reflect inside the inset (clamp an overshoot);
            --    every compare is on src against a registered bound
            v_rl := to_signed(2 * C_EDGE, 13) - s_src5;
            v_rh := s_hi2 - s_src5;
            if s_src5 < C_EDGE then
                if s_src5 < s_lo2mhi then s_addr6 <= unsigned(s_hi(10 downto 0));
                else s_addr6 <= unsigned(v_rl(10 downto 0)); end if;
            elsif s_src5 > s_hi then
                if s_src5 > s_hi2mlo then s_addr6 <= to_unsigned(C_EDGE, 11);
                else s_addr6 <= unsigned(v_rh(10 downto 0)); end if;
            else
                s_addr6 <= unsigned(s_src5(10 downto 0));
            end if;
        end if;
    end process p_addr;

    -- line buffers: written at stage 2 from the input register, read at 7
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr6));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr6));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr6(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr6(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr6(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr6(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_bx, v_by : unsigned(2 downto 0);
        variable v_t  : unsigned(5 downto 0);
        variable v_tc : signed(6 downto 0);
        variable v_h  : signed(10 downto 0);
        variable v_n  : unsigned(10 downto 0);
        variable v_s  : unsigned(9 downto 0);
        variable v_q  : unsigned(10 downto 0);
        variable v_cu, v_cv : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 7 (RAM read): dither cell from the source column and line (S8
            --   Source) or the screen pixel (Screen); alternate panes take the
            --   Alt Bits constants
            if s_sw_screen = '1' then v_bx := s_xs(6); v_by := s_lnum;
            else v_bx := s_addr6(2 downto 0); v_by := s_lm1; end if;
            s_thr7 <= f_thr(s_mat, v_bx, v_by);
            if s_sw_altbits = '1' and s_par(6) = '1' then
                s_dsc7 <= s_dscb; s_half7 <= s_halfb; s_cm7 <= s_cmaskb; s_l7 <= s_lb;
            else
                s_dsc7 <= s_dsc;  s_half7 <= s_half;  s_cm7 <= s_cmask;  s_l7 <= s_l;
            end if;
            -- 8: fetch the bank written by the line above; swap U/V when its
            --    hsync->avid parity differs from this line's.  Centred dither
            --    product: (2t + 1 - 64) * dsc (2(t - 32) + 1: flip t's top bit)
            if s_lw = '0' then
                s_y8 <= unsigned(s_rdY1);
                if s_rowswap = '1' then s_u8 <= unsigned(s_rdV1) & "00"; s_v8 <= unsigned(s_rdU1) & "00";
                else s_u8 <= unsigned(s_rdU1) & "00"; s_v8 <= unsigned(s_rdV1) & "00"; end if;
            else
                s_y8 <= unsigned(s_rdY0);
                if s_rowswap = '1' then s_u8 <= unsigned(s_rdV0) & "00"; s_v8 <= unsigned(s_rdU0) & "00";
                else s_u8 <= unsigned(s_rdU0) & "00"; s_v8 <= unsigned(s_rdV0) & "00"; end if;
            end if;
            v_t := s_thr7 xor "100000";
            v_tc := signed(v_t & '1');
            s_d8 <= v_tc * signed(resize(s_dsc7, 11));
            s_half8 <= s_half7; s_cm8 <= s_cm7; s_l8 <= s_l7;
            -- 9: luma above black; half a step + the dither (always 0..2*half)
            if s_y8 < 64 then s_ya9 <= (others => '0'); else s_ya9 <= s_y8 - 64; end if;
            v_h := signed(resize(s_half8, 11)) + resize(shift_right(s_d8, 7), 11);
            s_hd9 <= unsigned(v_h(8 downto 0));
            s_u9 <= s_u8; s_v9 <= s_v8; s_cm9 <= s_cm8; s_l9 <= s_l8;
            -- 10: luma * 1.168 (= 1023/876) in two halves; chroma + offset
            s_n1_10 <= resize(s_ya9, 11) + resize(shift_right(s_ya9, 3), 11);
            s_n2_10 <= resize(shift_right(s_ya9, 5), 7) + resize(shift_right(s_ya9, 7), 7) + resize(shift_right(s_ya9, 8), 7);
            s_cu10 <= resize(s_u9, 11) + resize(s_hd9, 11);
            s_cv10 <= resize(s_v9, 11) + resize(s_hd9, 11);
            s_u10 <= s_u9; s_v10 <= s_v9; s_hd10 <= s_hd9; s_cm10 <= s_cm9; s_l10 <= s_l9;
            -- 11: normalized luma 0..1023; chroma level
            v_n := s_n1_10 + resize(s_n2_10, 11);
            if v_n(10) = '1' then s_yn11 <= (others => '1'); else s_yn11 <= v_n(9 downto 0); end if;
            s_cuq11 <= f_cq(s_cu10, s_cm10);
            s_cvq11 <= f_cq(s_cv10, s_cm10);
            s_u11 <= s_u10; s_v11 <= s_v10; s_hd11 <= s_hd10; s_l11 <= s_l10;
            -- 12: scale by (N-1)/N so N levels span the range evenly; S9 chroma
            case to_integer(s_l11) is
                when 1 => v_s := shift_right(s_yn11, 1);
                when 2 => v_s := shift_right(s_yn11, 2);
                when 3 => v_s := shift_right(s_yn11, 3);
                when 4 => v_s := shift_right(s_yn11, 4);
                when 5 => v_s := shift_right(s_yn11, 5);
                when others => v_s := shift_right(s_yn11, 6);
            end case;
            s_ys12 <= s_yn11 - v_s;
            if s_sw_chroma = '1' then s_cu(12) <= s_cuq11; s_cv(12) <= s_cvq11;
            else s_cu(12) <= s_u11; s_cv(12) <= s_v11; end if;
            s_hd12 <= s_hd11; s_l12 <= s_l11;
            for i in 13 to 17 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            -- 13: + half step + dither
            v_q := resize(s_ys12, 11) + resize(s_hd12, 11);
            if v_q(10) = '1' then s_t13 <= (others => '1'); else s_t13 <= v_q(9 downto 0); end if;
            s_l13 <= s_l12;
            -- 14: keep L bits, replicate
            s_rep14 <= f_rep10(s_t13, s_l13 - 1);
            -- 15, 16: back to the video range, 64 + rep * 0.855 (64..939)
            s_e1_15 <= s_rep14 - shift_right(s_rep14, 3);
            s_e2_15 <= resize(shift_right(s_rep14, 6), 5) + resize(shift_right(s_rep14, 8), 5);
            s_yq16 <= 64 + s_e1_15 - resize(s_e2_15, 10);
            -- 17: alternate panes negative about the video range (64 <-> 942);
            --     neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(15) = '0' then
                s_y17 <= to_unsigned(64, 10); s_cu(17) <= to_unsigned(512, 10); s_cv(17) <= to_unsigned(512, 10);
            elsif s_sw_altneg = '1' and s_par(16) = '1' then
                s_y17 <= 1006 - s_yq16;
            else
                s_y17 <= s_yq16;
            end if;
            -- 18..19: horizontal [1 2 1]/4 on U/V (g1 the centre); luma rides along
            s_g1y <= s_y17;
            s_g1u <= s_cu(17); s_g2u <= s_g1u;
            s_g1v <= s_cv(17); s_g2v <= s_g1v;
            v_cu := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_cu(17), 12);
            v_cv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_cv(17), 12);
            s_y19 <= s_g1y; s_u19 <= v_cu(11 downto 2); s_v19 <= v_cv(11 downto 2);
            -- 20: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 775); gate
            v_d := s_y19 - 64;
            if s_avid_sr(18) = '1' then
                s_y20 <= s_y19 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture foldither;
