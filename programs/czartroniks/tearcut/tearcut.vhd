-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Tearcut: catalogue combo #115 (seed 20260926) -- datamosh / block
-- displacement + scanline shear / horizontal tearing + edge detect.  Two
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Gain     edge gain
--   K2 Thresh   edges below this are dropped
--   K3 Block    block size 8 .. 64 px
--   K4 Density  fraction of blocks moshed, ~5% .. ~98%
--   K5 Tear     tear amplitude, +-8 .. +-518 px
--   K6 Bands    tear band height 2 / 4 / 8 / 16 lines (Random) or the
--               wave period 64 / 128 / 256 / 512 lines (Wave)
--
--   S7  Edges   white / tinted (picture chroma at double saturation)
--   S8  Tear    random per band / triangle wave down the picture
--   S9  Blocks  plain / moshed blocks go negative
--   S10 Mosh    shift / hold (block repeats its first column)
--   S11 Ink     light edges on dark / dark edges on light
--   P12 Blend   edges on paper (0) .. edges over the picture (100)
--
-- Edge = |gx| + |gy| from the input and the Y line buffer, written as an
-- 8-bit edge line and read back one line later at the block-displaced,
-- torn address; the picture itself (dry ring) is never torn, so the edges
-- slide over it.  Background = paper .. picture (Blend), then the edge
-- strength keys the ink over it.  No edges in the C_EDGE side columns or on
-- a field's first line.  Latency 19 clocks, all modes, blanking-gated.
--
-- v0.2.0: blend keeps the edges at 100%, mosh key = block row & column with
-- a per-roll seed after the first round (v0.1 aliased at 8 px in HD and
-- every re-roll was a diagonal copy), wave period at 16 ln, negative
-- 64 <-> 940 with mirrored chroma, tinted ink luma 768, Density / Tear
-- floors, no_rw_check on every RAM.
--
-- v0.2.3 (2026-10-17): K3 Block / K6 Bands honour the firmware's labelled
-- default, which arrives as the label INDEX (K6 "8 ln" loaded as 2 and gave
-- 2 ln): a value below 4 is the label itself.
-- Resolution scaling: block columns/rows, tear bands and the tear wave
-- count virtual 1080-line units, stepped per pixel / per field line from
-- the video timing ID (registers_in(8)): 1.0 at 1080p
-- (identical to before), 1.5 at 720p, and at SD 2.0 across (a 1440-wide 4:3
-- window) and 1080/486 per frame line (interlaced fields step two frame
-- lines; v0.2 counted field lines, so SD blocks and bands were twice as
-- tall).  Blocks, bands and the wave are the same fraction of the screen
-- height in every mode and blocks stay square on a 4:3 screen.  The mosh
-- and tear displacements are in the same units (2/3 of the pixel offsets
-- at 720p, half at SD); Hold reads the first pixel of the virtual block.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tearcut of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 19;
    constant C_EDGE : integer := 12;
    constant C_RATE : integer := 180;   -- re-roll every C_RATE + 1 fields (~3 s)

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;
    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;
    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

    -- Resolution scaling: per-mode steps from the video timing ID.
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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_e is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 31) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbE0, lbE1 : t_lb_e := (others => x"00");
    signal ring : t_ring := (others => x"04080200");
    -- collision-free: Y reads x while writing x - 1, E banks alternate by
    -- line (a write-bank collision only hits the discarded read), ring w - 8
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbE0, lbE1, ring : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdE0, s_rdE1 : std_logic_vector(7 downto 0) := x"00";
    signal s_ring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_gain, s_k_thr, s_k_block, s_k_dens, s_k_tear, s_k_bands : unsigned(9 downto 0);
    signal s_sw_tint, s_sw_wave, s_sw_neg, s_sw_hold, s_sw_dark : std_logic;
    signal s_p_blend : unsigned(9 downto 0);
    signal s_lk_g, s_lk_d, s_lk_t : unsigned(9 downto 0) := (others => '0');
    signal s_lw     : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_hi     : unsigned(10 downto 0) := to_unsigned(708, 11);
    signal s_tear   : unsigned(8 downto 0) := (others => '0');
    signal s_bsh : unsigned(2 downto 0) := (others => '0');
    signal s_tkey : unsigned(15 downto 0) := (others => '0');
    signal s_th1 : unsigned(15 downto 0) := (others => '0');
    signal s_tri : unsigned(8 downto 0) := (others => '0');
    signal s_tp : signed(18 downto 0) := (others => '0');
    signal s_tear_row : signed(10 downto 0) := (others => '0');
    signal s_tstep : unsigned(2 downto 0) := "111";
    signal s_thr, s_blend, s_shift, s_dens : unsigned(7 downto 0) := (others => '0');
    signal s_gain   : unsigned(8 downto 0) := (others => '0');
    signal s_bs     : unsigned(2 downto 0) := to_unsigned(4, 3);       -- block shift 3..6
    signal s_bmask  : unsigned(10 downto 0) := (others => '0');
    signal s_roll, s_sd, s_seed : unsigned(15 downto 0) := (others => '0');
    signal s_rate_cnt : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    -- resolution scaling: steps (free-running from the timing ID), virtual
    -- x (Q.1, pixel on data_in) and field-line y (Q.8), per-mode mosh
    -- amount / clamp and tear-amount parts
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_vxa    : unsigned(11 downto 0) := (others => '0');
    signal s_vya    : unsigned(19 downto 0) := (others => '0');
    signal s_isfirst : std_logic := '1';     -- pixel on data_in starts a block
    signal s_omax, s_omaxn : signed(12 downto 0) := to_signed(511, 13);
    signal s_tsa, s_tsb : unsigned(8 downto 0) := (others => '0');

    -- tracking
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_rowkey : unsigned(7 downto 0) := (others => '0');
    signal s_top    : std_logic := '1';

    -- edge side (stages 1..8)
    signal s_in_y   : unsigned(9 downto 0) := (others => '0');
    signal s_y2     : unsigned(9 downto 0) := (others => '0');
    signal s_gx2    : signed(10 downto 0) := (others => '0');
    signal s_prev2  : unsigned(9 downto 0) := (others => '0');
    signal s_r_rbank : std_logic := '0';
    signal s_gx3, s_gy3 : signed(10 downto 0) := (others => '0');
    signal s_ax4, s_ay4 : unsigned(9 downto 0) := (others => '0');
    signal s_sum5   : unsigned(10 downto 0) := (others => '0');
    signal s_pe6    : unsigned(18 downto 0) := (others => '0');
    signal s_e7     : signed(14 downto 0) := (others => '0');
    signal s_side7  : std_logic := '0';
    signal s_e8     : unsigned(7 downto 0) := (others => '0');
    type t_wx is array (1 to 7) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 7) := (others => '0');
    signal s_wx8 : unsigned(10 downto 0) := (others => '0');
    signal s_wp8, s_we8 : std_logic := '0';

    -- address / hash chain (stages 1..8)
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_vx1    : unsigned(10 downto 0) := (others => '0');
    signal s_fd     : std_logic_vector(1 to 5) := (others => '1');
    signal s_bx6    : unsigned(10 downto 0) := (others => '0');   -- block's first pixel
    signal s_chi6, s_clo6 : std_logic := '0';                    -- offset clamps
    signal s_key2   : unsigned(15 downto 0) := (others => '0');
    signal s_h3, s_h4, s_h5 : unsigned(15 downto 0) := (others => '0');
    signal s_mosh6  : std_logic := '0';
    signal s_po6    : signed(17 downto 0) := (others => '0');
    signal s_u7     : signed(12 downto 0) := (others => '0');
    signal s_addr8  : unsigned(10 downto 0) := (others => '0');
    signal s_bg8    : std_logic := '0';
    type t_xd is array (2 to 6) of unsigned(10 downto 0);
    signal s_xd     : t_xd := (others => (others => '0'));
    signal s_moshd  : std_logic_vector(7 to 16) := (others => '0');

    -- display side (stages 9..19)
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_ed10   : unsigned(7 downto 0) := (others => '0');
    signal s_dy10, s_du10, s_dv10 : unsigned(9 downto 0) := (others => '0');
    signal s_pap11, s_iny11, s_inu11, s_inv11 : unsigned(9 downto 0) := (others => '0');
    signal s_dy11, s_du11, s_dv11 : signed(10 downto 0) := (others => '0');
    signal s_a11, s_a12, s_a13, s_a14 : unsigned(7 downto 0) := (others => '0');
    signal s_py12, s_pu12, s_pv12 : signed(19 downto 0) := (others => '0');
    signal s_pap12, s_iny12, s_inu12, s_inv12 : unsigned(9 downto 0) := (others => '0');
    signal s_by13, s_bu13, s_bv13 : unsigned(9 downto 0) := (others => '0');
    signal s_iny13, s_inu13, s_inv13 : unsigned(9 downto 0) := (others => '0');
    signal s_iy14, s_iu14, s_iv14 : signed(10 downto 0) := (others => '0');
    signal s_by14, s_bu14, s_bv14 : unsigned(9 downto 0) := (others => '0');
    signal s_qy15, s_qu15, s_qv15 : signed(19 downto 0) := (others => '0');
    signal s_by15, s_bu15, s_bv15 : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_gain  <= unsigned(registers_in(0));
    s_k_thr   <= unsigned(registers_in(1));
    s_k_block <= unsigned(registers_in(2));
    s_k_dens  <= unsigned(registers_in(3));
    s_k_tear  <= unsigned(registers_in(4));
    s_k_bands <= unsigned(registers_in(5));
    s_sw_tint <= registers_in(6)(0);
    s_sw_wave <= registers_in(6)(1);
    s_sw_neg  <= registers_in(6)(2);
    s_sw_hold <= registers_in(6)(3);
    s_sw_dark <= registers_in(6)(4);
    s_p_blend <= unsigned(registers_in(7));

    -- dry ring: data_in delayed 9 (q at stage 9, registered at stage 10)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - 8));
            ring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control, tracking, edge side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_a : signed(10 downto 0);
        variable v_nx : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            -- virtual x of the pixel on data_in, and whether it starts a block
            -- (its block index differs from the previous pixel's; at 1080p:
            -- x is a multiple of the block size)
            if data_in.avid = '1' then
                v_nx := s_vxa + s_xs;
                s_vxa <= v_nx;
                if ((v_nx(11 downto 1) xor s_vxa(11 downto 1)) and not s_bmask) /= 0 then
                    s_isfirst <= '1';
                else
                    s_isfirst <= '0';
                end if;
            else
                s_vxa <= (others => '0'); s_isfirst <= '1';
            end if;
            s_in_y <= unsigned(data_in.y);
            s_in_avid <= data_in.avid;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- stage 2: horizontal gradient, previous-line pixel
            s_y2  <= s_in_y;
            s_gx2 <= signed('0' & s_in_y) - signed('0' & s_y2);
            s_r_rbank <= s_rbank;
            if s_r_rbank = '1' then s_prev2 <= unsigned(s_rdY1); else s_prev2 <= unsigned(s_rdY0); end if;
            -- stage 3: vertical gradient
            s_gy3 <= signed('0' & s_y2) - signed('0' & s_prev2);
            s_gx3 <= s_gx2;
            -- stage 4: abs (negate at full width, then slice)
            if s_gx3 < 0 then v_a := -s_gx3; else v_a := s_gx3; end if;
            s_ax4 <= unsigned(v_a(9 downto 0));
            if s_gy3 < 0 then v_a := -s_gy3; else v_a := s_gy3; end if;
            s_ay4 <= unsigned(v_a(9 downto 0));
            -- stage 5: sum
            s_sum5 <= resize(s_ax4, 11) + resize(s_ay4, 11);
            -- stage 6: gain
            s_pe6 <= s_sum5(10 downto 1) * s_gain;
            -- stage 7: threshold; the capture's black side columns and a
            -- field's first line draw no edge
            s_e7 <= signed(resize(s_pe6(18 downto 5), 15)) - signed(resize(s_thr, 15));
            if s_wx(6) < C_EDGE or s_wx(6) >= s_hi or s_top = '1' then s_side7 <= '1'; else s_side7 <= '0'; end if;
            -- stage 8: 8-bit edge
            if s_e7 < 0 or s_side7 = '1' then s_e8 <= (others => '0');
            elsif s_e7 > 255 then s_e8 <= x"FF";
            else s_e8 <= unsigned(s_e7(7 downto 0)); end if;
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid;
            for i in 2 to 7 loop s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1); end loop;
            s_wx8 <= s_wx(7); s_wp8 <= s_wp(7); s_we8 <= s_wc(7);

            -- per-frame decode (one small op per state)
            case to_integer(s_vstep) is
                when 0 =>
                    s_thr   <= s_k_thr(9 downto 2);
                    s_blend <= s_p_blend(9 downto 2);
                    -- labelled defaults: the firmware loads the label INDEX
                    if s_k_block(9 downto 2) = 0 then s_bs <= resize(s_k_block(1 downto 0), 3) + 3;
                    else s_bs <= resize(s_k_block(9 downto 8), 3) + 3; end if;
                    -- mosh amount and clamp per mode (+-511 px at 1080p)
                    if s_xs = 4 then
                        s_shift <= to_unsigned(100, 8); s_omax <= to_signed(255, 13); s_omaxn <= to_signed(-255, 13);
                    elsif s_xs = 3 then
                        s_shift <= to_unsigned(133, 8); s_omax <= to_signed(340, 13); s_omaxn <= to_signed(-340, 13);
                    else
                        s_shift <= to_unsigned(200, 8); s_omax <= to_signed(511, 13); s_omaxn <= to_signed(-511, 13);
                    end if;
                    if s_k_bands(9 downto 2) = 0 then s_bsh <= resize(s_k_bands(1 downto 0), 3) + 1;
                    else s_bsh <= resize(s_k_bands(9 downto 8), 3) + 1; end if;
                    s_lk_g <= s_k_gain; s_lk_d <= s_k_dens; s_lk_t <= s_k_tear;
                    s_lw    <= s_line_width;
                    if s_rate_cnt >= C_RATE then
                        s_rate_cnt <= (others => '0');
                        s_roll <= s_roll + 1013;
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    for i in 0 to 10 loop
                        if i < to_integer(s_bs) then s_bmask(i) <= '1'; else s_bmask(i) <= '0'; end if;
                    end loop;
                    -- gain x 1.156 keeps v0.1's ink levels under the 876-step key
                    s_gain  <= resize(s_lk_g(9 downto 2), 9) + resize(s_lk_g(9 downto 5), 9);
                    s_dens  <= s_lk_d(9 downto 2) - resize(s_lk_d(9 downto 6), 8);
                    s_tear  <= resize(s_lk_t(9 downto 2), 9) + 4;
                    s_hi    <= s_lw - C_EDGE;
                    s_sd    <= f_round1(s_roll xor x"5A3C");
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_gain  <= s_gain + resize(s_lk_g(9 downto 7), 9);
                    s_dens  <= s_dens + 12;
                    s_sd    <= f_round2(s_sd);
                    -- tear amount x 2/3 (720p) parts
                    s_tsa   <= ('0' & s_tear(8 downto 1)) + ("000" & s_tear(8 downto 3));
                    s_tsb   <= ("00000" & s_tear(8 downto 5)) + ("0000000" & s_tear(8 downto 7));
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_seed  <= f_round3(s_sd);
                    -- tear amount in pixels: 2/3 at 720p, half at SD
                    if s_xs = 3 then s_tear <= s_tsa + s_tsb;
                    elsif s_xs = 4 then s_tear <= '0' & s_tear(8 downto 1); end if;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_vya <= s_vya + s_ys;
            end if;
            -- bands, wave and block rows key on the virtual line
            case to_integer(s_tstep) is
                when 0 =>
                    s_tkey <= x"2C9F" + s_roll + shift_left(resize(shift_right(s_vya(18 downto 8), to_integer(s_bsh)), 16), 5);
                    -- triangle wave: period 2^(bsh+5) lines
                    case to_integer(s_bsh) is
                        when 1      => s_tri <= shift_left(s_vya(18 downto 8), 3)(8 downto 0);
                        when 2      => s_tri <= shift_left(s_vya(18 downto 8), 2)(8 downto 0);
                        when 3      => s_tri <= shift_left(s_vya(18 downto 8), 1)(8 downto 0);
                        when others => s_tri <= s_vya(16 downto 8);
                    end case;
                    s_tstep <= s_tstep + 1;
                when 1 =>
                    s_th1 <= f_round3(f_round1(s_tkey));
                    if s_tri(8) = '1' then s_tri <= 511 - s_tri; end if;
                    s_tstep <= s_tstep + 1;
                when 2 =>
                    if s_sw_wave = '1' then
                        s_tp <= (signed(resize(s_tri, 9)) - 128) * signed(resize(s_tear, 10));
                    else
                        s_tp <= (signed(resize(s_th1(15 downto 8), 9)) - 128) * signed(resize(s_tear, 10));
                    end if;
                    s_tstep <= s_tstep + 1;
                when 3 =>
                    s_tear_row <= s_tp(16 downto 6);
                    s_tstep <= "111";
                when others => null;
            end case;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
                s_rowkey <= shift_right(s_vya(18 downto 8), to_integer(s_bs))(7 downto 0);
                if s_line = 0 then s_top <= '1'; else s_top <= '0'; end if;
                s_tstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                -- a bottom field starts half a field step (one frame line) down
                if data_in.field_n = '0' then
                    s_vya <= resize(s_ys(11 downto 1), 20);
                else
                    s_vya <= (others => '0');
                end if;
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
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

    --------------------------------------------------------------------------
    -- Block hash + display address (stages 1..8)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_x : unsigned(10 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx; s_vx1 <= s_vxa(11 downto 1);
            s_fd(1) <= s_isfirst;
            for i in 2 to 5 loop s_fd(i) <= s_fd(i - 1); end loop;
            -- stage 2: key = block row & block column (no adder: v0.1's
            -- row * 128 + column aliased at 8-px blocks in HD), virtual units
            s_key2 <= s_rowkey & shift_right(s_vx1, to_integer(s_bs))(7 downto 0);
            s_xd(2) <= s_x1;
            for i in 3 to 6 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- stage 6: the block's first pixel (Hold); at 1080p x and not bmask
            if s_fd(5) = '1' then s_bx6 <= s_xd(5); end if;
            -- stages 3..5: hash, the roll seed enters after the first round
            s_h3 <= f_round1(s_key2);
            s_h4 <= f_round2(s_h3 xor s_seed);
            s_h5 <= f_round3(s_h4);
            -- stage 6: mosh decision, offset product
            if s_h5(15 downto 8) < s_dens then s_mosh6 <= '1'; else s_mosh6 <= '0'; end if;
            s_po6 <= (signed('0' & s_h5(7 downto 0)) - to_signed(128, 9)) * signed('0' & s_shift);
            -- the offset clamps (|h - 128| >= 82: exactly where (h - 128) * 200 / 32
            -- leaves +-511 at 1080p), with a per-mode limit
            if s_h5(7 downto 0) >= 210 then s_chi6 <= '1'; else s_chi6 <= '0'; end if;
            if s_h5(7 downto 0) <= 46 then s_clo6 <= '1'; else s_clo6 <= '0'; end if;
            -- stage 7: displaced x (hold = block start)
            if s_mosh6 = '1' then
                if s_chi6 = '1' then v_o := s_omax;
                elsif s_clo6 = '1' then v_o := s_omaxn;
                else v_o := resize(shift_right(s_po6, 5), 13); end if;
                if s_sw_hold = '1' then v_x := s_bx6; else v_x := s_xd(6); end if;
                s_u7 <= signed(resize(v_x, 13)) + v_o + resize(s_tear_row, 13);
            else
                s_u7 <= signed(resize(s_xd(6), 13)) + resize(s_tear_row, 13);
            end if;
            s_moshd(7) <= s_mosh6;
            for i in 8 to 16 loop s_moshd(i) <= s_moshd(i - 1); end loop;
            -- stage 8: range
            if s_u7 < 0 or s_u7 >= signed(resize(s_line_width, 13)) then s_bg8 <= '1'; else s_bg8 <= '0'; end if;
            s_addr8 <= unsigned(s_u7(10 downto 0));
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers: Y (written stage 1, read at rx -> data 1),
    -- E (written stage 8, read at stage 8 -> data 9)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_rx));
            if s_in_avid = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_rx));
            if s_in_avid = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbE0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdE0 <= lbE0(to_integer(s_addr8));
            if s_we8 = '1' and s_wp8 = '0' then lbE0(to_integer(s_wx8)) <= std_logic_vector(s_e8); end if;
        end if;
    end process;
    p_lbE1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdE1 <= lbE1(to_integer(s_addr8));
            if s_we8 = '1' and s_wp8 = '1' then lbE1(to_integer(s_wx8)) <= std_logic_vector(s_e8); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Display pipeline (stages 9..19)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y, v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 9 ctx / 10 fetch + dry (both pixel-aligned at stage 10)
            s_a_bg <= s_bg8; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then s_ed10 <= (others => '0');
            elsif s_a_rbank = '1' then s_ed10 <= unsigned(s_rdE1);
            else s_ed10 <= unsigned(s_rdE0); end if;
            s_dy10 <= unsigned(s_ring_q(29 downto 20)); s_du10 <= unsigned(s_ring_q(19 downto 10)); s_dv10 <= unsigned(s_ring_q(9 downto 0));
            -- stage 11: picture - paper, ink, key
            if s_sw_dark = '1' then
                s_pap11 <= to_unsigned(940, 10);
                s_dy11  <= signed('0' & s_dy10) - to_signed(940, 11);
                if s_sw_tint = '1' then s_iny11 <= to_unsigned(192, 10); else s_iny11 <= to_unsigned(64, 10); end if;
            else
                s_pap11 <= to_unsigned(64, 10);
                s_dy11  <= signed('0' & s_dy10) - to_signed(64, 11);
                if s_sw_tint = '1' then s_iny11 <= to_unsigned(768, 10); else s_iny11 <= to_unsigned(940, 10); end if;
            end if;
            s_du11 <= signed('0' & s_du10) - to_signed(512, 11);
            s_dv11 <= signed('0' & s_dv10) - to_signed(512, 11);
            -- tinted ink = picture chroma at double saturation, clamped
            if s_sw_tint = '0' then
                s_inu11 <= to_unsigned(512, 10); s_inv11 <= to_unsigned(512, 10);
            else
                case s_du10(9 downto 8) is
                    when "00"   => s_inu11 <= (others => '0');
                    when "11"   => s_inu11 <= (others => '1');
                    when others => s_inu11 <= s_du10(9) & s_du10(7 downto 0) & '0';
                end case;
                case s_dv10(9 downto 8) is
                    when "00"   => s_inv11 <= (others => '0');
                    when "11"   => s_inv11 <= (others => '1');
                    when others => s_inv11 <= s_dv10(9) & s_dv10(7 downto 0) & '0';
                end case;
            end if;
            s_a11 <= s_ed10;
            -- stage 12: background products (paper .. picture by Blend)
            s_py12 <= s_dy11 * signed('0' & s_blend);
            s_pu12 <= s_du11 * signed('0' & s_blend);
            s_pv12 <= s_dv11 * signed('0' & s_blend);
            s_pap12 <= s_pap11; s_iny12 <= s_iny11; s_inu12 <= s_inu11; s_inv12 <= s_inv11;
            s_a12 <= s_a11;
            -- stage 13: background (lies between paper and picture: slice)
            v_y := signed(resize(s_pap12, 12)) + resize(shift_right(s_py12, 8), 12);
            v_u := to_signed(512, 12) + resize(shift_right(s_pu12, 8), 12);
            v_v := to_signed(512, 12) + resize(shift_right(s_pv12, 8), 12);
            s_by13 <= unsigned(v_y(9 downto 0)); s_bu13 <= unsigned(v_u(9 downto 0)); s_bv13 <= unsigned(v_v(9 downto 0));
            s_iny13 <= s_iny12; s_inu13 <= s_inu12; s_inv13 <= s_inv12; s_a13 <= s_a12;
            -- stage 14: ink - background
            s_iy14 <= signed('0' & s_iny13) - signed('0' & s_by13);
            s_iu14 <= signed('0' & s_inu13) - signed('0' & s_bu13);
            s_iv14 <= signed('0' & s_inv13) - signed('0' & s_bv13);
            s_by14 <= s_by13; s_bu14 <= s_bu13; s_bv14 <= s_bv13; s_a14 <= s_a13;
            -- stage 15: key products
            s_qy15 <= s_iy14 * signed('0' & s_a14);
            s_qu15 <= s_iu14 * signed('0' & s_a14);
            s_qv15 <= s_iv14 * signed('0' & s_a14);
            s_by15 <= s_by14; s_bu15 <= s_bu14; s_bv15 <= s_bv14;
            -- stage 16: ink over background (stays between the two, no clamp)
            v_y := signed(resize(s_by15, 12)) + resize(shift_right(s_qy15, 8), 12);
            v_u := signed(resize(s_bu15, 12)) + resize(shift_right(s_qu15, 8), 12);
            v_v := signed(resize(s_bv15, 12)) + resize(shift_right(s_qv15, 8), 12);
            s_y16 <= unsigned(v_y(9 downto 0)); s_u16 <= unsigned(v_u(9 downto 0)); s_v16 <= unsigned(v_v(9 downto 0));
            -- stage 17: moshed blocks negative (64 <-> 940, chroma mirrored about 512)
            if s_sw_neg = '1' and s_moshd(16) = '1' then
                if s_y16 > 1004 then s_y17 <= (others => '0');
                else s_y17 <= to_unsigned(1004, 10) - s_y16; end if;
                if s_u16 = 0 then s_u17 <= (others => '1'); else s_u17 <= unsigned(-signed(s_u16)); end if;
                if s_v16 = 0 then s_v17 <= (others => '1'); else s_v17 <= unsigned(-signed(s_v16)); end if;
            else
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            end if;
            -- stage 18: blanking gate
            if s_avid_sr(16) = '1' then
                s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            -- stage 19: output
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y19);
    s_io_align_in.u       <= std_logic_vector(s_u19);
    s_io_align_in.v       <= std_logic_vector(s_v19);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (19 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture tearcut;
