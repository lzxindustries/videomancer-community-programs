-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Trapquant: catalogue combo #120 (seed 20260926) -- perspective keystone /
-- corner-pin + posterization (bit-depth reduction per channel).  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Top      width of the picture at the top (25..225%, glided)
--   K2 Bottom   width of the picture at the bottom (25..225%, glided)
--   K3 Slide    horizontal slide, bipolar, +-1/2 of the width (glided)
--   K4 Luma     posterize luma 1..8 bits
--   K5 Chroma   posterize chroma 1..8 bits
--   K6 Dither   dither amount (0..100% of a step, centred)
--
--   S7  Edges   outside the picture: edge fill / mirrored (tiles forever)
--   S8  Order   dither keyed to the source (cells stretch with the
--               keystone) / to the screen (cells stay put)
--   S9  Ramp    the quantizer thresholds shift with each row's scale:
--               bands slide along the trapezoid as its shape moves
--   S10 Dither  2x2 Bayer / alternate lines
--   S11 Chroma  posterized / kept
--   P12 Tilt    rotates the trapezoid: top wide .. bottom wide (glided)
--
-- Keystone: each row's width is interpolated linearly top -> bottom (so
-- the sides are straight), and a per-line restoring divider turns it into
-- the Q12 source step 1/width.  A DDA walks the source line one row late
-- out of a dual-bank line buffer.  Fill clamps the source x into the
-- line's C_EDGE inset (the fill is the line's own edge pixel); Mirror runs
-- the DDA modulo 2W' (W' = the inset width) and reflects the upper half,
-- so the picture tiles mirrored at any width.
--
-- Posterize (read side, one quantizer for both orders -- a point-sampled
-- gather commutes with a per-pixel quantizer, so only the dither cell
-- coordinates differ): luma normalized ((y-64)*1.168), dithered, top L
-- bits kept and bit-replicated (levels span black..white), scaled back
-- by 0.695 (64 + 13/16 of the 0.855 range: white ~775, the HD HDMI bar
-- guard's luma scale folded in).  Chroma is a centred mid-tread quantizer
-- (levels at 512 +- k*step, clamped 64..960), so greys stay grey.  Dither
-- offsets are centred ((2b-3)/8 step, lines +-1/4) and cells are 2 px
-- wide so the [1 2 1] guard does not flatten them.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (16) in all modes; banks
-- latched per line at the avid rise; U/V swapped on rows whose
-- hsync->avid parity differs from the stored row's (each row replays the
-- row above -- drape v1.3); output conditioning ([1 2 1] on Y/U/V, luma
-- white ~775 -- damier v0.4.4: dithered posterize steps are full-swing
-- pixel edges that move with the keystone; at 1-2 bit chroma a 9-tap
-- [1 2 2 2 2 2 2 2 1]/16 replaces the chroma [1 2 1] -- v0.2.0 still barred
-- there); no_rw_check on every RAM (a
-- bank is only read while the other is written; the write bank's read
-- data is discarded); width latched once a field; per-frame and per-line
-- maths in hblank/vblank sequencers, one op per state.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture trapquant of program_top is
    -- data_in -> data_out latency is 16 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;

    constant C_LAT  : integer := 16;
    constant C_EDGE : integer := 12;

    -- line buffers: Y full rate, U/V every other pixel at 8 bits
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_top, s_k_bot, s_k_slide, s_k_lbits, s_k_cbits, s_k_dith : unsigned(9 downto 0);
    signal s_p_tilt : unsigned(9 downto 0);
    -- switches, registered once (1-bit flags into the pixel path)
    signal s_mir, s_scr, s_rmp, s_lns, s_ckp : std_logic := '0';

    -- bit depths: zone = bits - 1, following the 1..8 readout (round(7k/1023)),
    -- hysteretic: a new zone is taken only 12 counts inside it
    signal s_k7l, s_k7c : unsigned(12 downto 0) := (others => '0');
    signal s_zl : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_zc : unsigned(2 downto 0) := to_unsigned(2, 3);

    -- per frame (vblank sequencer)
    signal s_saw_active : std_logic := '0';
    signal s_vstep  : unsigned(5 downto 0) := (others => '1');
    signal s_dq     : unsigned(20 downto 0) := (others => '0');
    signal s_dr     : unsigned(11 downto 0) := (others => '0');
    signal s_dbit   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_h  : unsigned(12 downto 0) := to_unsigned(970, 13);    -- 2^20 / height
    -- glided knobs, 10.4 fixed point
    signal s_g_top  : unsigned(13 downto 0) := to_unsigned(484 * 16, 14);
    signal s_g_bot  : unsigned(13 downto 0) := to_unsigned(289 * 16, 14);
    signal s_g_sld  : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal s_g_tlt  : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal s_fw     : unsigned(10 downto 0) := to_unsigned(480, 11);    -- field-latched width
    signal s_cx     : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_hi_e   : unsigned(10 downto 0) := to_unsigned(467, 11);    -- fw - 1 - C_EDGE
    signal s_wp     : unsigned(10 downto 0) := to_unsigned(456, 11);    -- W' = fw - 2 C_EDGE
    signal s_rref   : unsigned(11 downto 0) := to_unsigned(923, 12);    -- 2W' - 1 + C_EDGE
    signal s_2wp    : signed(26 downto 0) := (others => '0');           -- 2W' << 12
    signal s_zlf : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_zcf : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_lnsf   : std_logic := '0';
    signal s_ls : unsigned(9 downto 0) := to_unsigned(64, 10);           -- step = 512 >> zone
    signal s_cs : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal s_lmask, s_cmask : unsigned(9 downto 0) := (others => '1');
    signal s_w1, s_w2 : unsigned(14 downto 0) := to_unsigned(4096, 15); -- knob widths, Q12
    signal s_tq     : signed(15 downto 0) := (others => '0');           -- tilt, Q12 (+-1.0)
    signal s_top, s_bot : unsigned(14 downto 0) := to_unsigned(4096, 15);
    signal s_dtb    : signed(15 downto 0) := (others => '0');
    signal s_dsc    : signed(21 downto 0) := (others => '0');           -- per-line width step, Q24
    signal s_slq    : signed(26 downto 0) := (others => '0');           -- slide, Q12 px
    signal s_cxsl   : signed(26 downto 0) := (others => '0');           -- cx << 12 + slide
    signal s_amt    : unsigned(7 downto 0) := (others => '0');
    -- shared per-frame multiplier: operands registered one state ahead,
    -- product free-running and split on B; valid three states after set
    signal s_ma     : signed(15 downto 0) := (others => '0');
    signal s_mb     : unsigned(12 downto 0) := (others => '0');
    signal s_mph    : signed(23 downto 0) := (others => '0');
    signal s_mpl    : signed(22 downto 0) := (others => '0');
    signal s_mp     : signed(29 downto 0) := (others => '0');
    signal s_el1, s_el2, s_el3, s_ec1, s_ec2, s_ec3 : unsigned(9 downto 0) := (others => '0');
    -- dither tables by Bayer cell b: luma signed offsets, chroma round + offset
    signal s_dl0, s_dl1, s_dl2, s_dl3 : signed(10 downto 0) := (others => '0');
    signal s_dc0, s_dc1, s_dc2, s_dc3 : unsigned(9 downto 0) := to_unsigned(32, 10);

    -- per line (hblank sequencer, started one clock after the avid fall,
    -- the vsync edge, or the end of the frame sequence)
    signal s_lgo    : std_logic := '0';
    signal s_lst    : unsigned(4 downto 0) := (others => '1');
    signal s_acc_s  : signed(28 downto 0) := (others => '0');           -- width offset, Q24
    signal s_wrow   : unsigned(18 downto 0) := to_unsigned(65536, 19);  -- row width, Q16
    signal s_r      : unsigned(18 downto 0) := (others => '0');
    signal s_q      : unsigned(14 downto 0) := (others => '0');
    signal s_step   : unsigned(14 downto 0) := to_unsigned(4096, 15);   -- source px per screen px, Q12
    signal s_cxs_h  : unsigned(17 downto 0) := (others => '0');
    signal s_cxs_l  : unsigned(16 downto 0) := (others => '0');
    signal s_cxs    : unsigned(24 downto 0) := (others => '0');
    signal s_s0     : signed(26 downto 0) := (others => '0');
    signal s_src0   : signed(26 downto 0) := (others => '0');           -- DDA preload
    signal s_stepm  : signed(26 downto 0) := (others => '0');           -- step - 2W' (mirror wrap)
    signal s_phi    : signed(9 downto 0) := (others => '0');            -- Ramp threshold phase

    signal s_rx : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_wr_x : unsigned(10 downto 0) := (others => '0');          -- x of the s_in pixel (stage 1)
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_lw, s_lpar : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';

    -- read side
    signal s_acc  : signed(26 downto 0) := (others => '0');            -- 2: source x (Q12)
    signal s_addr : unsigned(10 downto 0) := to_unsigned(C_EDGE, 11);  -- 3
    signal s_xb2, s_xb3, s_xb4, s_ab4 : std_logic := '0';
    signal s_b5, s_b6 : unsigned(1 downto 0) := (others => '0');
    signal s_wet_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_a6  : unsigned(9 downto 0) := (others => '0');
    signal s_cu6, s_cv6 : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal s_n17 : unsigned(10 downto 0) := (others => '0');
    signal s_n27 : unsigned(6 downto 0) := (others => '0');
    signal s_dz7, s_dz8 : signed(10 downto 0) := (others => '0');
    signal s_yn8 : unsigned(9 downto 0) := (others => '0');
    signal s_t9, s_rep10 : unsigned(9 downto 0) := (others => '0');
    signal s_p111 : unsigned(9 downto 0) := (others => '0');
    signal s_p211 : unsigned(6 downto 0) := (others => '0');
    signal s_y12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- chroma delay line, s_cu(k) = chroma of the stage-k pixel (neutral outside avid)
    type t_c is array (7 to 15) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_c := (others => to_unsigned(512, 10));
    signal s_y13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning: luma [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- chroma: [1 2 1] (3+ bits, Keep) or [1 2 2 2 2 2 2 2 1]/16 (1-2 bits):
    -- running 8-sums of s_cu(8..15), plus the previous sum
    signal s_heavy : std_logic := '0';
    signal s_ru, s_ru1, s_rv, s_rv1 : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_yu13, s_yv13 : unsigned(13 downto 0) := to_unsigned(8192, 14);
    signal s_tu13, s_tv13 : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal s_u14, s_v14, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');               -- s_avid_sr(k) = avid of the stage-(k+1) pixel

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

    -- one glide step: cur + (knob*16 - cur) / 4
    function f_glide(cur : unsigned(13 downto 0); k : unsigned(9 downto 0)) return unsigned is
        variable v_t, v_c : signed(14 downto 0);
    begin
        v_t := signed(resize(k & "0000", 15));
        v_c := signed(resize(cur, 15));
        v_c := v_c + shift_right(v_t - v_c, 2);
        return unsigned(v_c(13 downto 0));
    end function;

begin

    s_k_top   <= unsigned(registers_in(0));
    s_k_bot   <= unsigned(registers_in(1));
    s_k_slide <= unsigned(registers_in(2));
    s_k_lbits <= unsigned(registers_in(3));
    s_k_cbits <= unsigned(registers_in(4));
    s_k_dith  <= unsigned(registers_in(5));
    s_p_tilt  <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_r   : unsigned(12 downto 0);
        variable v_lo, v_hi : unsigned(12 downto 0);
        variable v_w   : signed(20 downto 0);
        variable v_dv  : unsigned(18 downto 0);
        variable v_t   : signed(15 downto 0);
        variable v_f   : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_avid_sr(0) = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            s_mir <= registers_in(6)(0);
            s_scr <= registers_in(6)(1);
            s_rmp <= registers_in(6)(2);
            s_lns <= registers_in(6)(3);
            s_ckp <= registers_in(6)(4);
            s_amt <= s_k_dith(9 downto 2);

            -- K4 / K5 zones: round(7k/1023), changed only 12 counts inside the new zone
            s_k7l <= (s_k_lbits & "000") - resize(s_k_lbits, 13);
            s_k7c <= (s_k_cbits & "000") - resize(s_k_cbits, 13);
            v_lo := s_k7l + 428; v_hi := s_k7l + 596;
            if v_lo(12 downto 10) = v_hi(12 downto 10) then s_zl <= v_lo(12 downto 10); end if;
            v_lo := s_k7c + 428; v_hi := s_k7c + 596;
            if v_lo(12 downto 10) = v_hi(12 downto 10) then s_zc <= v_lo(12 downto 10); end if;

            -- free-running per-frame product (operands registered one state ahead)
            s_mph <= s_ma * signed('0' & s_mb(12 downto 6));
            s_mpl <= s_ma * signed('0' & s_mb(5 downto 0));
            s_mp  <= shift_left(resize(s_mph, 30), 6) + resize(s_mpl, 30);
            -- free-running per-line product cx * step, split on step
            s_cxs_h <= s_cx * s_step(14 downto 7);
            s_cxs_l <= s_cx * s_step(6 downto 0);
            s_cxs   <= shift_left(resize(s_cxs_h, 25), 7) + resize(s_cxs_l, 25);

            s_lgo <= '0';

            -- per frame: inverse height (2^20 / height, 21-step restoring
            -- divider), glides, the row-width endpoints, slide, dither tables
            case to_integer(s_vstep) is
                when 0 =>
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(20, 5);
                    s_fw <= s_line_width;
                    s_zlf <= s_zl; s_zcf <= s_zc; s_lnsf <= s_lns;
                    s_vstep <= s_vstep + 1;
                when 1 to 21 =>
                    -- numerator 2^20: bit 20 is 1, all lower bits 0
                    if s_dbit = 20 then v_r := (s_dr(11 downto 0) & '1'); else v_r := (s_dr(11 downto 0) & '0'); end if;
                    if v_r >= resize(s_height, 13) then
                        s_dr <= resize(v_r - resize(s_height, 13), 12);
                        s_dq <= s_dq(19 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0);
                        s_dq <= s_dq(19 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    s_vstep <= s_vstep + 1;
                when 22 =>
                    s_inv_h <= s_dq(12 downto 0);
                    s_g_top <= f_glide(s_g_top, s_k_top);
                    s_g_bot <= f_glide(s_g_bot, s_k_bot);
                    s_g_sld <= f_glide(s_g_sld, s_k_slide);
                    s_g_tlt <= f_glide(s_g_tlt, s_p_tilt);
                    s_cx   <= s_fw(10 downto 1);
                    s_hi_e <= s_fw - (C_EDGE + 1);
                    s_wp   <= s_fw - 2 * C_EDGE;
                    s_ls   <= shift_right(to_unsigned(512, 10), to_integer(s_zlf));
                    s_cs   <= shift_right(to_unsigned(512, 10), to_integer(s_zcf));
                    s_vstep <= s_vstep + 1;
                when 23 =>
                    -- width = 0.25 + k/512 (Q12: 1024 + 8k); tilt = (p - 512)/512
                    s_w1 <= 1024 + resize(s_g_top(13 downto 1), 15);
                    s_w2 <= 1024 + resize(s_g_bot(13 downto 1), 15);
                    s_tq <= shift_right(signed(resize(s_g_tlt, 16)) - 8192, 1);
                    s_rref <= (s_wp & '0') + (C_EDGE - 1);
                    s_2wp <= shift_left(resize(signed('0' & s_wp), 27), 13);
                    s_lmask <= not (s_ls - 1);
                    s_cmask <= not (s_cs - 1);
                    s_vstep <= s_vstep + 1;
                when 24 =>
                    -- P12 up narrows the top and widens the bottom; floor 25%
                    v_t := signed(resize(s_w1, 16)) - s_tq;
                    if v_t < 1024 then s_top <= to_unsigned(1024, 15); else s_top <= unsigned(v_t(14 downto 0)); end if;
                    v_t := signed(resize(s_w2, 16)) + s_tq;
                    if v_t < 1024 then s_bot <= to_unsigned(1024, 15); else s_bot <= unsigned(v_t(14 downto 0)); end if;
                    s_vstep <= s_vstep + 1;
                when 25 =>
                    s_dtb <= signed(resize(s_bot, 16)) - signed(resize(s_top, 16));
                    s_vstep <= s_vstep + 1;
                when 26 => s_ma <= s_dtb; s_mb <= s_inv_h; s_vstep <= s_vstep + 1;
                when 27 =>
                    s_ma <= signed(resize(s_g_sld, 16)) - 8192;               -- (k - 512) * 16
                    s_mb <= resize(s_fw, 13);
                    s_vstep <= s_vstep + 1;
                when 28 => s_ma <= signed(resize(s_amt, 16)); s_mb <= resize(s_ls, 13); s_vstep <= s_vstep + 1;
                when 29 =>
                    s_dsc <= resize(shift_right(s_mp, 8), 22);                 -- dtb * 2^12 / height, Q24
                    s_mb <= resize(s_ls & '0', 13);
                    s_vstep <= s_vstep + 1;
                when 30 =>
                    s_slq <= resize(shift_right(s_mp, 2), 27);                 -- (k-512)/1024 * width, Q12
                    s_mb <= resize(s_ls & '0', 13) + resize(s_ls, 13);
                    s_vstep <= s_vstep + 1;
                when 31 =>
                    s_el1 <= unsigned(s_mp(20 downto 11));                     -- amt * step / 8
                    s_cxsl <= shift_left(resize(signed('0' & s_cx), 27), 12) + s_slq;
                    s_mb <= resize(s_cs, 13);
                    s_vstep <= s_vstep + 1;
                when 32 => s_el2 <= unsigned(s_mp(20 downto 11)); s_mb <= resize(s_cs & '0', 13); s_vstep <= s_vstep + 1;
                when 33 =>
                    s_el3 <= unsigned(s_mp(20 downto 11));
                    s_mb <= resize(s_cs & '0', 13) + resize(s_cs, 13);
                    s_vstep <= s_vstep + 1;
                when 34 => s_ec1 <= unsigned(s_mp(20 downto 11)); s_vstep <= s_vstep + 1;
                when 35 => s_ec2 <= unsigned(s_mp(20 downto 11)); s_vstep <= s_vstep + 1;
                when 36 => s_ec3 <= unsigned(s_mp(20 downto 11)); s_vstep <= s_vstep + 1;
                when 37 =>
                    -- Bayer b = 0..3 -> (2b-3)/8 step; Lines -> -+1/4 step by line
                    if s_lnsf = '1' then
                        s_dl0 <= -signed(resize(s_el2, 11)); s_dl1 <= signed(resize(s_el2, 11));
                        s_dl2 <= -signed(resize(s_el2, 11)); s_dl3 <= signed(resize(s_el2, 11));
                        s_dc0 <= shift_right(s_cs, 1) - s_ec2; s_dc1 <= shift_right(s_cs, 1) + s_ec2;
                        s_dc2 <= shift_right(s_cs, 1) - s_ec2; s_dc3 <= shift_right(s_cs, 1) + s_ec2;
                    else
                        s_dl0 <= -signed(resize(s_el3, 11)); s_dl1 <= -signed(resize(s_el1, 11));
                        s_dl2 <= signed(resize(s_el1, 11));  s_dl3 <= signed(resize(s_el3, 11));
                        s_dc0 <= shift_right(s_cs, 1) - s_ec3; s_dc1 <= shift_right(s_cs, 1) - s_ec1;
                        s_dc2 <= shift_right(s_cs, 1) + s_ec1; s_dc3 <= shift_right(s_cs, 1) + s_ec3;
                    end if;
                    s_lgo <= '1';                                              -- redo the line maths with the new frame
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- per line: row width (linear top -> bottom), step = 2^28 / width
            -- (15-step restoring divider), DDA preload, Ramp phase
            case to_integer(s_lst) is
                when 0 =>
                    v_w := signed(resize(s_top & "0000", 21)) + resize(shift_right(s_acc_s, 8), 21);
                    s_wrow <= unsigned(v_w(18 downto 0));
                    s_lst <= s_lst + 1;
                when 1 =>
                    -- the quotient is < 2^15 (width >= 0.25), so bits 28..15 of
                    -- 2^28 leave the remainder 2^13
                    s_r <= to_unsigned(8192, 19); s_q <= (others => '0');
                    s_lst <= s_lst + 1;
                when 2 to 16 =>
                    v_dv := s_r(17 downto 0) & '0';
                    if v_dv >= s_wrow then
                        s_r <= v_dv - s_wrow; s_q <= s_q(13 downto 0) & '1';
                    else
                        s_r <= v_dv; s_q <= s_q(13 downto 0) & '0';
                    end if;
                    s_lst <= s_lst + 1;
                when 17 => s_step <= s_q; s_lst <= s_lst + 1;
                when 18 => s_lst <= s_lst + 1;                                 -- cx * step settles
                when 19 =>
                    s_s0 <= s_cxsl - signed(resize(s_step, 27));               -- one step early: the DDA adds it first
                    s_lst <= s_lst + 1;
                when 20 =>
                    -- src0 = (cx + slide - cx * step) - step, Q12
                    s_s0 <= s_s0 - signed(resize(s_cxs, 27));
                    s_stepm <= signed(resize(s_step, 27)) - s_2wp;
                    -- Ramp: the threshold phase cycles four times per unit of scale
                    v_f := signed('0' & s_step(9 downto 0)) - 512;
                    if s_rmp = '0' then
                        s_phi <= (others => '0');
                    else
                        s_phi <= resize(shift_right(v_f, to_integer(s_zlf) + 1), 10);
                    end if;
                    s_lst <= s_lst + 1;
                when 21 =>
                    if s_mir = '1' then s_s0 <= s_s0 - C_EDGE * 4096; end if;
                    s_lst <= s_lst + 1;
                when 22 | 23 =>
                    -- mirror: fold into [0, 2W')
                    if s_mir = '1' and s_s0 < 0 then s_s0 <= s_s0 + s_2wp; end if;
                    s_lst <= s_lst + 1;
                when 24 =>
                    if s_mir = '1' and s_s0 >= s_2wp then s_s0 <= s_s0 - s_2wp; end if;
                    s_lst <= s_lst + 1;
                when 25 =>
                    s_src0 <= s_s0;
                    s_lst <= (others => '1');
                when others => null;
            end case;

            -- line events
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                -- this line writes bank s_wpar and reads the other (the line above)
                s_lw <= s_wpar;
                s_lpar <= s_line(0);
                s_line_par <= s_hpar;
                if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_lw = '1' then s_rowswap <= s_line_par xor s_cap_par0;
            else s_rowswap <= s_line_par xor s_cap_par1; end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                s_acc_s <= s_acc_s + resize(s_dsc, 29);
                s_lgo <= '1';
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0'; s_acc_s <= (others => '0');
                s_lgo <= '1';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
                s_saw_active <= '0';
            end if;
            if s_lgo = '1' then s_lst <= (others => '0'); end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_addr : process(clk)
        variable v_a, v_b : signed(26 downto 0);
        variable v_si : signed(14 downto 0);
        variable v_u  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: DDA (preloaded during blanking).  Mirror runs modulo 2W'.
            if s_avid_sr(0) = '0' then
                s_acc <= s_src0;
            else
                v_a := s_acc + signed(resize(s_step, 27));
                v_b := s_acc + s_stepm;
                if s_mir = '1' and v_b >= 0 then s_acc <= v_b; else s_acc <= v_a; end if;
            end if;
            s_xb2 <= s_wr_x(1);
            -- 3: address.  Fill: clamp into the C_EDGE inset.  Mirror: reflect
            --    the upper half of [0, 2W'), offset into the inset.
            v_si := s_acc(26 downto 12);
            v_u  := unsigned(s_acc(23 downto 12));
            if s_mir = '1' then
                if v_u >= s_wp then s_addr <= resize(s_rref - v_u, 11);
                else s_addr <= resize(v_u + C_EDGE, 11); end if;
            elsif v_si < C_EDGE then s_addr <= to_unsigned(C_EDGE, 11);
            elsif v_si > signed(resize(s_hi_e, 15)) then s_addr <= s_hi_e;
            else s_addr <= unsigned(v_si(10 downto 0)); end if;
            s_xb3 <= s_xb2;
            -- 4: (RAM read) dither cell x: 2-px cells of the source or the screen
            s_xb4 <= s_xb3;
            s_ab4 <= s_addr(1);
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_bx : std_logic;
        variable v_dc : unsigned(9 downto 0);
        variable v_dl : signed(10 downto 0);
        variable v_n  : unsigned(10 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_blank : std_logic;
    begin
        if rising_edge(clk) then
            -- 5: fetch the bank written by the line above; swap U/V when its
            --    hsync->avid parity differs from this line's.  Dither cell
            --    b = Bayer index of (2-px x cell, line parity).
            if s_lw = '0' then
                s_wet_y <= unsigned(s_rdY1);
                if s_rowswap = '1' then s_wet_u <= unsigned(s_rdV1) & "00"; s_wet_v <= unsigned(s_rdU1) & "00";
                else s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00"; end if;
            else
                s_wet_y <= unsigned(s_rdY0);
                if s_rowswap = '1' then s_wet_u <= unsigned(s_rdV0) & "00"; s_wet_v <= unsigned(s_rdU0) & "00";
                else s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            end if;
            if s_scr = '1' then v_bx := s_xb4; else v_bx := s_ab4; end if;
            s_b5 <= (v_bx xor s_lpar) & s_lpar;
            -- 6: luma - black; chroma + rounding/dither offset (or kept)
            s_b6 <= s_b5;
            if s_wet_y < 64 then s_a6 <= (others => '0'); else s_a6 <= s_wet_y - 64; end if;
            case to_integer(s_b5) is
                when 0 => v_dc := s_dc0;
                when 1 => v_dc := s_dc1;
                when 2 => v_dc := s_dc2;
                when others => v_dc := s_dc3;
            end case;
            if s_ckp = '1' then v_dc := (others => '0'); end if;
            s_cu6 <= resize(s_wet_u, 11) + resize(v_dc, 11);
            s_cv6 <= resize(s_wet_v, 11) + resize(v_dc, 11);
            -- 7: luma * 1.168 (= 1023/876) in two halves; chroma level;
            --    luma dither + Ramp phase
            s_n17 <= resize(s_a6, 11) + resize(shift_right(s_a6, 3), 11);
            s_n27 <= resize(shift_right(s_a6, 5), 7) + resize(shift_right(s_a6, 7), 7) + resize(shift_right(s_a6, 8), 7);
            if s_avid_sr(5) = '0' then
                s_cu(7) <= to_unsigned(512, 10); s_cv(7) <= to_unsigned(512, 10);
            elsif s_ckp = '1' then
                s_cu(7) <= s_cu6(9 downto 0); s_cv(7) <= s_cv6(9 downto 0);
            else
                s_cu(7) <= f_cq(s_cu6, s_cmask); s_cv(7) <= f_cq(s_cv6, s_cmask);
            end if;
            case to_integer(s_b6) is
                when 0 => v_dl := s_dl0;
                when 1 => v_dl := s_dl1;
                when 2 => v_dl := s_dl2;
                when others => v_dl := s_dl3;
            end case;
            s_dz7 <= v_dl + resize(s_phi, 11);
            -- 8: normalized luma 0..1023
            v_n := s_n17 + resize(s_n27, 11);
            if v_n(10) = '1' then s_yn8 <= (others => '1'); else s_yn8 <= v_n(9 downto 0); end if;
            s_dz8 <= s_dz7;
            -- 9: dithered (signed, mean 0)
            v_t := signed(resize(s_yn8, 12)) + resize(s_dz8, 12);
            if v_t < 0 then s_t9 <= (others => '0');
            elsif v_t > 1023 then s_t9 <= (others => '1');
            else s_t9 <= unsigned(v_t(9 downto 0)); end if;
            -- 10: keep L bits, replicate (level k of N -> k * 1023 / (N-1))
            s_rep10 <= f_rep10(s_t9, s_zlf);
            -- 11, 12: back to video, 64 + rep * 0.695 (white ~775)
            s_p111 <= shift_right(s_rep10, 1) + shift_right(s_rep10, 3);
            s_p211 <= resize(shift_right(s_rep10, 4), 7) + resize(shift_right(s_rep10, 7), 7);
            s_y12 <= 64 + s_p111 + resize(s_p211, 10);
            for i in 8 to 15 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            -- 13: luma neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(11) = '1' then s_y13 <= s_y12; else s_y13 <= to_unsigned(64, 10); end if;
            -- 14..15: horizontal [1 2 1]/4 on Y (g1 the centre)
            s_g1y <= s_y13; s_g2y <= s_g1y;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y13, 12);
            s_y15 <= v_sy(11 downto 2);
            -- chroma smoothing, centred on the pixel at stage 12 (out at 13).
            -- 1-2 bit chroma jumps 64 <-> 512 <-> 960 and the dither flips it
            -- every 2 px: after [1 2 1] that is ~224 codes a pixel, which still
            -- bars on HD HDMI (v0.2.0).  The 9-tap caps a full-swing step at
            -- 56/px and nulls the 2-px dither cells.  Running sums: R = sum of
            -- s_cu(8..15), resynced to 8 x 512 whenever those 8 are all blank.
            v_blank := '1';
            for i in 6 to 13 loop
                if s_avid_sr(i) = '1' then v_blank := '0'; end if;
            end loop;
            if v_blank = '1' then
                s_ru <= to_unsigned(4096, 13); s_rv <= to_unsigned(4096, 13);
            else
                s_ru <= s_ru + resize(s_cu(7), 13) - resize(s_cu(15), 13);
                s_rv <= s_rv + resize(s_cv(7), 13) - resize(s_cv(15), 13);
            end if;
            s_ru1 <= s_ru; s_rv1 <= s_rv;
            s_yu13 <= resize(s_ru, 14) + resize(s_ru1, 14);
            s_yv13 <= resize(s_rv, 14) + resize(s_rv1, 14);
            s_tu13 <= resize(s_cu(11), 12) + shift_left(resize(s_cu(12), 12), 1) + resize(s_cu(13), 12);
            s_tv13 <= resize(s_cv(11), 12) + shift_left(resize(s_cv(12), 12), 1) + resize(s_cv(13), 12);
            if s_zcf <= 1 and s_ckp = '0' then s_heavy <= '1'; else s_heavy <= '0'; end if;
            -- 14, 15
            if s_heavy = '1' then
                s_u14 <= s_yu13(13 downto 4); s_v14 <= s_yv13(13 downto 4);
            else
                s_u14 <= s_tu13(11 downto 2); s_v14 <= s_tv13(11 downto 2);
            end if;
            s_u15 <= s_u14; s_v15 <= s_v14;
            -- 16: gate
            if s_avid_sr(14) = '1' then
                s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture trapquant;
