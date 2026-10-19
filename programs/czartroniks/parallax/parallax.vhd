-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Parallax: catalogue combo #38 (seed 20260918) -- depth-from-luma parallax
-- + colour channel swap / offset.  Three knobs each; the switches and the
-- slider decide how the two combine.
--
--   K1 Depth    parallax gain (luma above / below focus shifts sideways);
--               floored at 1/8 so Position always moves the picture
--   K2 Sway     the camera sways on its own: 0 = still and centred, then
--               ~70 s .. 0.27 s per cycle (speed doubles every 1/8 of the
--               knob; the bottom 1/8 also fades the sway in)
--   K3 Focus    luma level that does not move (spans black .. white)
--   K4 Swap     channel mode (labelled): Off, U<>V, Y>U, U>Y, Y<>V, Rotate,
--               Rotate Back, Complement
--   K5 Offset   chroma read offset, both channels together (+-128 px at
--               1080p)
--   K6 Split    U and V pulled apart, opposite ways (0 .. 127 px at 1080p)
--
--   S7  Depth   from luma / from saturation
--   S8  Near    bright is near / dark is near
--   S9  Chroma  displaced with luma / stays put (colour ghosts)
--   S10 Sway    Glide (sine) / Wiggle (snaps between two viewpoints, a
--               wigglegram)
--   S11 Edges   edge fill / wrap around
--   P12 Position camera position, bipolar (centre = flat when Sway is 0)
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Depth = the current pixel's own luma (or saturation), source = the line
-- above read at x + (depth - focus) * gain; U and V have their own read
-- addresses (free chromatic offset).  Rows replay the line above, so U/V
-- are swapped on rows whose hsync->avid parity differs from the row they
-- were captured on (drape v1.3).  Output conditioning ([1 2 1] on Y/U/V +
-- luma scaled to white ~777) is the HD HDMI bar fix from damier v0.4.4 --
-- the depth tears are full-swing pixel edges and the sway moves them.
-- Latency 15 clocks, all modes, blanking-gated.
--
-- v0.2.2 (2026-10-19): resolution scaling.  Every displacement is in
-- pixels, so it now follows the timing ID's x step (1080: 1.0 virtual unit
-- a pixel, 720p 1.5, SD 2.0): the depth gain (Depth x Position/Sway), the
-- Offset and the Split are scaled x1 / x2/3 / x1/2 (720p: (1/2 + 1/8) x
-- 17/16 = 0.664, unsigned adds, no shared sign bit), and the gain and
-- displacement clamps scale with them (511 / 341 / 255).  The same parallax
-- relative to the screen height in every mode.  The edge clamp and Wrap
-- stay on the real measured line.  1080p is bit-identical.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture parallax of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (15 + 1 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 15;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    -- Both banks are read every clock but only the bank NOT being written
    -- is used, so a read that collides with the write is always discarded.
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

    -- controls / per-frame
    signal s_k_depth, s_k_sway, s_k_focus, s_k_swap, s_k_offset, s_k_split : unsigned(9 downto 0);
    signal s_sw_sat, s_sw_darknear, s_sw_cstay, s_sw_wiggle, s_sw_wrap : std_logic;
    signal s_p_pos : unsigned(9 downto 0);
    signal s_live4  : std_logic := '0';   -- K4 has read a real position (label-index guard)
    signal s_mode   : unsigned(2 downto 0) := (others => '0');
    signal s_f1     : unsigned(9 downto 0) := (others => '0');
    signal s_focus  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_dg1    : unsigned(7 downto 0) := (others => '0');
    signal s_dg     : unsigned(8 downto 0) := to_unsigned(128, 9);   -- depth gain 32 .. 256
    signal s_inc    : unsigned(12 downto 0) := (others => '0');      -- sway phase step x2
    signal s_amp    : unsigned(7 downto 0) := (others => '0');       -- sway fade-in 0 .. 128
    signal s_coff, s_split : signed(9 downto 0) := (others => '0');
    signal s_coffu, s_coffv : signed(9 downto 0) := (others => '0');
    signal s_phase  : unsigned(15 downto 0) := (others => '0');
    signal s_sqh, s_sql : unsigned(14 downto 0) := (others => '0');
    signal s_sq     : unsigned(19 downto 0) := (others => '0');      -- x * (1023 - x)
    signal s_wav    : signed(9 downto 0) := (others => '0');         -- sway wave +-255
    signal s_swh, s_swl : signed(14 downto 0) := (others => '0');
    signal s_swa    : signed(18 downto 0) := (others => '0');
    signal s_pos    : signed(10 downto 0) := (others => '0');
    signal s_tot    : signed(10 downto 0) := (others => '0');        -- position + sway
    signal s_gph, s_gpl : signed(16 downto 0) := (others => '0');
    signal s_gp     : signed(20 downto 0) := (others => '0');
    signal s_gc     : signed(9 downto 0) := (others => '0');
    signal s_g      : signed(9 downto 0) := (others => '0');
    signal s_vstep  : unsigned(3 downto 0) := "1111";
    -- v0.2.2 resolution scaling: timing class and the x2/3 (720p) products
    signal s_sd, s_720 : std_logic := '0';
    signal s_dg_h, s_dg_t, s_dgk : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal s_ko_h, s_ko_t : unsigned(7 downto 0) := (others => '0');
    signal s_sp_h, s_sp_t : unsigned(6 downto 0) := (others => '0');

    -- tracking
    signal s_rx, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';

    -- address chain
    signal s_qy, s_qu, s_qv : unsigned(9 downto 0) := (others => '0');
    signal s_x1, s_x2, s_x3, s_x4, s_x4b, s_x5 : unsigned(10 downto 0) := (others => '0');
    signal s_dep2   : unsigned(9 downto 0) := (others => '0');
    signal s_d3     : signed(11 downto 0) := (others => '0');
    signal s_ph4    : signed(16 downto 0) := (others => '0');   -- d * g(9:5)
    signal s_pl4    : signed(17 downto 0) := (others => '0');   -- d * g(4:0)
    signal s_pd4    : signed(21 downto 0) := (others => '0');
    signal s_off5   : signed(10 downto 0) := (others => '0');
    signal s_uy6, s_uu6, s_uv6 : signed(12 downto 0) := (others => '0');
    signal s_ay7, s_au7, s_av7 : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal s_a_rbank : std_logic := '0';
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y13, s_y14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    -- per-line clamp bounds (registered) so the address compares run on the
    -- source value in parallel with the wrap add / subtract
    signal s_we13, s_wl13, s_wlo13, s_rwe13, s_rhi13 : signed(12 downto 0) := (others => '0');

    -- symmetric clamp; the limit is picked by the timing class (SD / 720p /
    -- 1080) from three constants, so the compares stay constant compares
    function f_sclamp3(v : signed; sd, c720 : std_logic;
                       l_hd, l_720, l_sd : integer; w : integer) return signed is
        variable v_lim : integer;
    begin
        if sd = '1' then
            v_lim := l_sd;
        elsif c720 = '1' then
            v_lim := l_720;
        else
            v_lim := l_hd;
        end if;
        if v > v_lim then
            return to_signed(v_lim, w);
        elsif v < -v_lim then
            return to_signed(-v_lim, w);
        else
            return resize(v, w);
        end if;
    end function;

begin

    s_k_depth  <= unsigned(registers_in(0));
    s_k_sway   <= unsigned(registers_in(1));
    s_k_focus  <= unsigned(registers_in(2));
    s_k_swap   <= unsigned(registers_in(3));
    s_k_offset <= unsigned(registers_in(4));
    s_k_split  <= unsigned(registers_in(5));
    s_sw_sat      <= registers_in(6)(0);
    s_sw_darknear <= registers_in(6)(1);
    s_sw_cstay    <= registers_in(6)(2);
    s_sw_wiggle   <= registers_in(6)(3);
    s_sw_wrap     <= registers_in(6)(4);
    s_p_pos    <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Control
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_m6 : unsigned(5 downto 0);
        variable v_c  : signed(9 downto 0);
        variable v_w  : unsigned(7 downto 0);
        variable v_g  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- resolution scaling: the displacements follow the screen height.
            -- Timing class (static while a mode runs): SD x1/2, 720p x2/3.
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_sd <= '1'; s_720 <= '0';
                when 5 | 6 | 14     => s_sd <= '0'; s_720 <= '1';
                when others         => s_sd <= '0'; s_720 <= '0';
            end case;
            -- 720p: (x/2 + x/8) x 17/16 = 0.664 x, unsigned (the two operands
            -- of each add are different bits: no router1 self-add)
            s_dg_h <= shift_right(s_dg, 1) + shift_right(s_dg, 3);
            s_dg_t <= s_dg_h + shift_right(s_dg_h, 4);
            if s_sd = '1' then s_dgk <= shift_right(s_dg, 1);
            elsif s_720 = '1' then s_dgk <= s_dg_t;
            else s_dgk <= s_dg; end if;
            s_ko_h <= shift_right(s_k_offset(9 downto 2), 1) + shift_right(s_k_offset(9 downto 2), 3);
            s_ko_t <= s_ko_h + shift_right(s_ko_h, 4);
            s_sp_h <= shift_right(s_k_split(9 downto 3), 1) + shift_right(s_k_split(9 downto 3), 3);
            s_sp_t <= s_sp_h + shift_right(s_sp_h, 4);

            -- per-frame decode: one small op per state
            case to_integer(s_vstep) is
                when 0 =>
                    -- sway step (32 + m) << e, halved when applied: 16 .. 4032
                    v_m6 := '1' & s_k_sway(6 downto 2);
                    s_inc <= shift_left(resize(v_m6, 13), to_integer(s_k_sway(9 downto 7)));
                    if s_k_sway(9 downto 7) /= "000" then s_amp <= to_unsigned(128, 8);
                    else s_amp <= '0' & s_k_sway(6 downto 0); end if;
                    s_f1  <= s_k_focus - shift_right(s_k_focus, 3);
                    s_dg1 <= s_k_depth(9 downto 2) - s_k_depth(9 downto 5);
                    s_pos <= signed('0' & s_p_pos) - to_signed(512, 11);
                    -- Offset and Split in this mode's pixels (centre 128 -> 64 / 85)
                    if s_sd = '1' then
                        v_c := signed(resize(shift_right(s_k_offset(9 downto 2), 1), 10));
                        s_coff  <= v_c - to_signed(64, 10);
                        s_split <= signed(resize(shift_right(s_k_split(9 downto 3), 1), 10));
                    elsif s_720 = '1' then
                        v_c := signed(resize(s_ko_t, 10));
                        s_coff  <= v_c - to_signed(85, 10);
                        s_split <= signed(resize(s_sp_t, 10));
                    else
                        v_c := signed(resize(s_k_offset(9 downto 2), 10));
                        s_coff  <= v_c - to_signed(128, 10);
                        s_split <= signed(resize(s_k_split(9 downto 3), 10));
                    end if;
                    if s_k_swap >= 8 then s_live4 <= '1'; end if;
                    if s_live4 = '0' and s_k_swap < 8 then s_mode <= s_k_swap(2 downto 0);
                    else s_mode <= s_k_swap(9 downto 7); end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_phase <= s_phase + resize(s_inc(12 downto 1), 16);
                    -- focus: luma 64 .. 959, or saturation 0 .. 511
                    if s_sw_sat = '1' then s_focus <= '0' & s_k_focus(9 downto 1);
                    else s_focus <= s_f1 + 64; end if;
                    s_dg <= resize(s_dg1, 9) + 32;
                    s_coffu <= s_coff + s_split;
                    s_coffv <= s_coff - s_split;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- parabolic sine over each half cycle: x * (1023 - x), split on x
                    s_sqh <= s_phase(14 downto 10) * (not s_phase(14 downto 5));
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_sql <= s_phase(9 downto 5) * (not s_phase(14 downto 5));
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_sq <= shift_left(resize(s_sqh, 20), 5) + resize(s_sql, 20);
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    -- Glide: sine 0 .. 255; Wiggle: held at the peak (square wave)
                    if s_sw_wiggle = '1' then v_w := to_unsigned(255, 8); else v_w := s_sq(17 downto 10); end if;
                    if s_phase(15) = '1' then s_wav <= -signed(resize(v_w, 10));
                    else s_wav <= signed(resize(v_w, 10)); end if;
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_swh <= s_wav * signed('0' & s_amp(7 downto 4));
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_swl <= s_wav * signed('0' & s_amp(3 downto 0));
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    s_swa <= shift_left(resize(s_swh, 19), 4) + resize(s_swl, 19);
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    -- position + sway (+-127)
                    s_tot <= s_pos + resize(shift_right(s_swa, 8), 11);
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    s_gph <= s_tot * signed('0' & s_dgk(8 downto 4));
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    s_gpl <= resize(s_tot * signed('0' & s_dgk(3 downto 0)), 17);
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    s_gp <= shift_left(resize(s_gph, 21), 4) + resize(s_gpl, 21);
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    -- gain = (position + sway) * depth (Q8), clamped (it used to wrap);
                    -- the clamp follows the mode's scale: 511 / 341 / 255
                    v_g := s_gp(20 downto 8);
                    s_gc <= f_sclamp3(v_g, s_sd, s_720, 511, 341, 255, 10);
                    s_vstep <= s_vstep + 1;
                when 14 =>
                    if s_sw_darknear = '1' then s_g <= -s_gc; else s_g <= s_gc; end if;
                    s_vstep <= "1111";
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13  <= signed(resize(s_w_edge, 13));                              -- w_edge
            s_wl13  <= signed(resize(s_line_width, 13));                          -- width
            s_wlo13 <= to_signed(C_EDGE, 13) - signed(resize(s_line_width, 13)); -- wrapped-left: w < C_EDGE <=> u < C_EDGE-width
            s_rwe13 <= signed(resize(s_line_width, 13)) + C_EDGE;                 -- wrapped-right: w < C_EDGE <=> u < width+C_EDGE
            s_rhi13 <= signed(resize(s_line_width, 13)) + signed(resize(s_w_edge, 13)); -- wrapped-right: w > w_edge <=> u > width+w_edge
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            -- U/V parity: rows show the line above, so swap where its
            -- hsync->avid parity differs from this row's
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_line_par <= s_hpar;
                if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_rbank = '1' then s_rowswap <= s_line_par xor s_cap_par1;
            else s_rowswap <= s_line_par xor s_cap_par0; end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
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
    -- Address chain (stages 1..7, plus 4b)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_au, v_av : unsigned(8 downto 0);
        variable v_w : signed(12 downto 0);
        procedure wrapit(signal a : out unsigned(10 downto 0); u : signed(12 downto 0)) is
            variable wl, wr : signed(12 downto 0);
        begin
            -- every compare is on u against a registered bound, in parallel
            -- with the wrap add / subtract; the mux is the only serial step
            wl := u + s_wl13; wr := u - s_wl13;
            if s_sw_wrap = '1' and u < 0 then
                if u < s_wlo13 then a <= to_unsigned(C_EDGE, 11);
                elsif u > -(C_EDGE + 1) then a <= s_w_edge;
                else a <= unsigned(wl(10 downto 0)); end if;
            elsif s_sw_wrap = '1' and u >= s_wl13 then
                if u < s_rwe13 then a <= to_unsigned(C_EDGE, 11);
                elsif u > s_rhi13 then a <= s_w_edge;
                else a <= unsigned(wr(10 downto 0)); end if;
            elsif u < C_EDGE then a <= to_unsigned(C_EDGE, 11);
            elsif u > s_we13 then a <= s_w_edge;
            else a <= unsigned(u(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            -- stage 1: dry pixel (stage 2)
            s_x1 <= s_rx;
            s_qy <= s_in_y; s_qu <= s_in_u; s_qv <= s_in_v;
            -- stage 2: depth source -- luma, or saturation |u-512| + |v-512|
            -- (abs by bit inversion: 1 code low below neutral, no carry)
            s_x2 <= s_x1;
            if s_qu(9) = '1' then v_au := s_qu(8 downto 0); else v_au := not s_qu(8 downto 0); end if;
            if s_qv(9) = '1' then v_av := s_qv(8 downto 0); else v_av := not s_qv(8 downto 0); end if;
            if s_sw_sat = '1' then s_dep2 <= resize(v_au, 10) + resize(v_av, 10);
            else s_dep2 <= s_qy; end if;
            -- stage 3: depth difference
            s_x3 <= s_x2;
            s_d3 <= signed(resize(s_dep2, 12)) - signed(resize(s_focus, 12));
            -- stage 4: displacement product, split on the gain (12x10 in one
            -- stage was the HD critical path)
            s_x4 <= s_x3;
            s_ph4 <= s_d3 * s_g(9 downto 5);
            s_pl4 <= s_d3 * signed('0' & s_g(4 downto 0));
            -- stage 4b: recombine
            s_x4b <= s_x4;
            s_pd4 <= shift_left(resize(s_ph4, 22), 5) + resize(s_pl4, 22);
            -- stage 5: offset (+-511 px at 1080p, 341 at 720p, 255 at SD)
            s_x5 <= s_x4b;
            v_w := resize(shift_right(s_pd4, 8), 13);
            s_off5 <= f_sclamp3(v_w, s_sd, s_720, 511, 341, 255, 11);
            -- stage 6: per-channel source x
            s_uy6 <= signed(resize(s_x5, 13)) + resize(s_off5, 13);
            if s_sw_cstay = '1' then
                s_uu6 <= signed(resize(s_x5, 13)) + resize(s_coffu, 13);
                s_uv6 <= signed(resize(s_x5, 13)) + resize(s_coffv, 13);
            else
                s_uu6 <= signed(resize(s_x5, 13)) + resize(s_off5, 13) + resize(s_coffu, 13);
                s_uv6 <= signed(resize(s_x5, 13)) + resize(s_off5, 13) + resize(s_coffv, 13);
            end if;
            -- stage 7: wrap / background per channel
            wrapit(s_ay7, s_uy6);
            wrapit(s_au7, s_uu6);
            wrapit(s_av7, s_uv6);
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 1, read stage 7 -> data 8), per-channel addresses
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ay7));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ay7));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_au7(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_au7(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_av7(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_av7(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 8..14, data one clock behind the stage number)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 8 ctx / 9 fetch + per-row U/V parity fix
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then v_y := unsigned(s_rdY1); v_u := unsigned(s_rdU1) & "00"; v_v := unsigned(s_rdV1) & "00";
            else v_y := unsigned(s_rdY0); v_u := unsigned(s_rdU0) & "00"; v_v := unsigned(s_rdV0) & "00"; end if;
            s_y9 <= v_y;
            if s_rowswap = '1' then s_u9 <= v_v; s_v9 <= v_u; else s_u9 <= v_u; s_v9 <= v_v; end if;
            -- stage 10: channel swap
            case to_integer(s_mode) is
                when 0 => s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;             -- Off
                when 1 => s_y10 <= s_y9; s_u10 <= s_v9; s_v10 <= s_u9;             -- U<>V
                when 2 => s_y10 <= s_y9; s_u10 <= s_y9; s_v10 <= s_v9;             -- Y>U
                when 3 => s_y10 <= s_u9; s_u10 <= s_u9; s_v10 <= s_v9;             -- U>Y
                when 4 => s_y10 <= s_v9; s_u10 <= s_u9; s_v10 <= s_y9;             -- Y<>V
                when 5 => s_y10 <= s_u9; s_u10 <= s_v9; s_v10 <= s_y9;             -- Rotate
                when 6 => s_y10 <= s_v9; s_u10 <= s_y9; s_v10 <= s_u9;             -- Rotate Back
                when others => s_y10 <= s_y9; s_u10 <= not s_u9; s_v10 <= not s_v9; -- Complement
            end case;
            -- stage 11: luma held in 64 .. 940; neutral outside avid (so the
            -- smoothing's edge taps read black)
            if s_avid_sr(10) = '1' then
                if s_y10(9 downto 6) = "0000" then s_y11 <= to_unsigned(64, 10);
                elsif s_y10 > 940 then s_y11 <= to_unsigned(940, 10);
                else s_y11 <= s_y10; end if;
                s_u11 <= s_u10; s_v11 <= s_v10;
            else
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            end if;
            -- stages 12..13: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y11; s_g2y <= s_g1y;
            s_g1u <= s_u11; s_g2u <= s_g1u;
            s_g1v <= s_v11; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y11, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u11, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v11, 12);
            s_y13 <= v_sy(11 downto 2); s_u13 <= v_su(11 downto 2); s_v13 <= v_sv(11 downto 2);
            -- stage 14: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            v_d := s_y13 - 64;
            if s_avid_sr(13) = '1' then
                s_y14 <= s_y13 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y14);
    s_io_align_in.u       <= std_logic_vector(s_u14);
    s_io_align_in.v       <= std_logic_vector(s_v14);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (15 -> 16).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture parallax;
