-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Etchwarp: catalogue combo #12 (seed 20260914) -- edge detect + Perlin /
-- value-noise displacement + pixelation.  Two knobs each; the switches and
-- the slider decide how they combine.
--
--   K1 Threshold edge threshold
--   K2 Ink       edge line strength (mixed over the picture)
--   K3 Noise     value-noise displacement amount (64x32 lattice, scrolling;
--                1080-line px, scaled per mode)
--   K4 Speed     lattice scroll speed
--   K5 Cells     pixel cell width 1 .. 64 px (1080-line px, scaled per mode)
--   K6 Rows      cell height = width / 1, 2, 4, 8
--
--   S7  Lines    edges drawn over the picture / edges only on black
--   S8  Order    noise moves the cell grid / noise moves the content
--   S9  Luma     pixelate luma only (chroma stays sharp)
--   S10 Colour   edge lines take the source chroma / are white
--   S11 Blocks   edges detected on the pixelated picture / on the smooth one
--   P12 Flow     extra noise scroll speed and amount together
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Line buffer read a line late; noise lattice engine (roulette) drives the
-- read address, cells hold it; edges from the fetched luma.  Latency 14
-- clocks, all modes, blanking-gated.
--
-- v0.1.6 (2026-10-17): resolution scaling.  The lattice, the cells and the
-- displacement live on a virtual 1080-line canvas, stepped per mode from the
-- video timing ID (registers_in(8)): x in half units per pixel (1080: 2,
-- 720p: 3, SD: 4 -- 720 px span 1440 units), y in Q.8 units per FIELD line
-- (1.5 at 720p, 1080/486 x 2 per 480i field line).  The noise lattice is
-- 64 x 32 virtual units (the DDA runs in Q7 with the step x the x step, and
-- a 720p lattice crossing between pixels reloads with its fractional part);
-- cell width / height count virtual units (wrap by subtracting the size);
-- the displacement amplitude is x 2/3 at 720p and x 1/2 at SD.  So the warp
-- and the blocks keep their size relative to the screen height, and SD's
-- cells stay square on the 4:3 screen.  1080p is unchanged (every step is
-- exactly 1.0).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture etchwarp of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (14 + 2 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 14;
    constant C_SEED_NOISE : unsigned(15 downto 0) := x"3C6E";

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
    -- Resolution scaling: virtual 1080-line steps from the timing ID.  x step
    -- in half units per pixel (1080: 2, 720p: 3, SD: 4), y step Q4.8 per
    -- FIELD line (1080 / active lines, doubled for interlace).
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
    -- (x step) * d for x steps 2 / 3 / 4
    function f_xmul(d : signed(10 downto 0); xs : unsigned(2 downto 0)) return signed is
    begin
        case xs is
            when "011"  => return resize(d, 13) + shift_left(resize(d, 13), 1);
            when "100"  => return shift_left(resize(d, 13), 2);
            when others => return shift_left(resize(d, 13), 1);
        end case;
    end function;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_thr, s_k_ink, s_k_noise, s_k_speed, s_k_cells, s_k_rows : unsigned(9 downto 0);
    -- K6 Rows label decode, registered.  Labelled defaults: at program load
    -- the firmware writes the initial_value_label's INDEX (0..3) into the
    -- register, not a 0..1023 position, so a value below 4 is the index.
    signal s_rows_idx : unsigned(1 downto 0) := (others => '0');
    signal s_sw_over, s_sw_order, s_sw_lumaonly, s_sw_colour, s_sw_blocks : std_logic;
    signal s_p_flow : unsigned(9 downto 0);
    signal s_thr    : unsigned(9 downto 0) := (others => '0');
    signal s_ink    : unsigned(7 downto 0) := (others => '0');
    signal s_amp_eff : unsigned(9 downto 0) := (others => '0');
    signal s_amp_pre : unsigned(19 downto 0) := (others => '0');
    signal s_ampin   : unsigned(9 downto 0) := (others => '0');
    signal s_pixn   : unsigned(6 downto 0) := to_unsigned(1, 7);
    signal s_rows   : unsigned(6 downto 0) := to_unsigned(1, 7);
    signal s_n_pre  : unsigned(19 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    -- resolution scaling (static per mode, decoded every clock)
    signal s_vxs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_vys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_amp_raw : unsigned(9 downto 0) := (others => '0');
    signal s_a168, s_a3r : unsigned(17 downto 0) := (others => '0');
    -- virtual x (Q.1) for the lattice: crossing flag + fractional remainder
    -- of the next pixel (720p crossings land between pixels)
    signal s_xa     : unsigned(12 downto 0) := (others => '0');
    signal s_xc     : std_logic := '0';
    signal s_xr     : unsigned(1 downto 0) := (others => '0');
    -- lattice differences and x-step scaled DDA steps (Q7)
    signal s_dCB, s_dBA   : signed(10 downto 0) := (others => '0');
    signal s_dCBx, s_dBAx : signed(12 downto 0) := (others => '0');
    -- virtual line (Q.8) for the lattice rows
    signal s_lya    : unsigned(19 downto 0) := (others => '0');
    -- cell counters in virtual units: x Q.1, y Q.8; lim = size - step (>= 0)
    signal s_hloc   : unsigned(7 downto 0) := (others => '0');
    signal s_hlim   : unsigned(7 downto 0) := (others => '0');
    signal s_hnew   : std_logic := '1';
    signal s_vloc   : unsigned(15 downto 0) := (others => '0');
    signal s_vlim   : unsigned(15 downto 0) := (others => '0');
    signal s_vnew   : std_logic := '1';

    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_oy     : unsigned(15 downto 0) := (others => '0');
    signal s_capture : std_logic := '1';
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    signal s_lstep  : unsigned(1 downto 0) := "11";
    signal s_ncy, s_ncy1 : unsigned(8 downto 0) := (others => '0');
    signal s_nfy    : unsigned(4 downto 0) := (others => '0');

    signal s_nstep  : unsigned(3 downto 0) := "1111";
    signal s_ct     : unsigned(6 downto 0) := (others => '0');
    signal s_nrem   : unsigned(1 downto 0) := (others => '0');
    signal s_nshift : std_logic := '0';
    signal s_ha, s_hb : unsigned(15 downto 0) := (others => '0');
    signal s_h0     : unsigned(7 downto 0) := (others => '0');
    signal s_hd     : signed(8 downto 0) := (others => '0');
    signal s_hp     : signed(14 downto 0) := (others => '0');
    signal s_hl     : signed(9 downto 0) := (others => '0');
    signal s_hlc    : signed(8 downto 0) := (others => '0');
    signal s_hq     : signed(19 downto 0) := (others => '0');
    signal s_vA, s_vB, s_vC : signed(9 downto 0) := (others => '0');
    signal s_nacc   : signed(16 downto 0) := (others => '0');   -- Q7
    signal s_nstp   : signed(12 downto 0) := (others => '0');   -- per pixel, Q7

    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_n2     : signed(9 downto 0) := (others => '0');
    signal s_x2     : unsigned(10 downto 0) := (others => '0');
    signal s_xh3    : unsigned(10 downto 0) := (others => '0');
    signal s_u3     : signed(13 downto 0) := (others => '0');
    signal s_n3     : signed(9 downto 0) := (others => '0');
    signal s_u4, s_uh : signed(13 downto 0) := (others => '0');
    signal s_uc4    : signed(13 downto 0) := (others => '0');
    signal s_addr_y, s_addr_c : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_yp1, s_yp2 : unsigned(9 downto 0) := (others => '0');
    signal s_d8     : signed(10 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_edge9  : std_logic := '0';
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_e10    : signed(10 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_edge10 : std_logic := '0';
    signal s_pe11   : signed(19 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_edge11 : std_logic := '0';
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    -- smooth-luma reference for edge detection (dry ring, aligned to stage 7)
    type t_ring is array (0 to 31) of std_logic_vector(9 downto 0);
    signal ringY : t_ring := (others => "0001000000");
    signal s_ring_qy : std_logic_vector(9 downto 0) := (others => '0');
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');
    signal s_dp1, s_dp2, s_dry7 : unsigned(9 downto 0) := (others => '0');

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
    function f_edge_addr(v : signed; we : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > signed(resize(we, v'length)) then
            return we;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

begin

    s_k_thr   <= unsigned(registers_in(0));
    s_k_ink   <= unsigned(registers_in(1));
    s_k_noise <= unsigned(registers_in(2));
    s_k_speed <= unsigned(registers_in(3));
    s_k_cells <= unsigned(registers_in(4));
    s_k_rows  <= unsigned(registers_in(5));
    s_sw_over     <= registers_in(6)(0);
    s_sw_order    <= registers_in(6)(1);
    s_sw_lumaonly <= registers_in(6)(2);
    s_sw_colour   <= registers_in(6)(3);
    s_sw_blocks   <= registers_in(6)(4);
    s_p_flow  <= unsigned(registers_in(7));

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w  <= s_ring_w + 1;
            s_ring_qy <= ringY(to_integer(s_ring_w - 6));
            ringY(to_integer(s_ring_w)) <= data_in.y;
        end if;
    end process;

    p_ctrl : process(clk)
        variable v_key  : unsigned(15 downto 0);
        variable v_ysum : unsigned(15 downto 0);
        variable v_v    : signed(9 downto 0);
        variable v_a    : unsigned(10 downto 0);
        variable v_xl   : unsigned(7 downto 0);
        variable v_rm   : signed(12 downto 0);
        variable v_hl   : signed(9 downto 0);
        variable v_vl   : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            if s_k_rows < 4 then
                s_rows_idx <= s_k_rows(1 downto 0);
            else
                s_rows_idx <= s_k_rows(9 downto 8);
            end if;
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid and s_capture;
            s_x1 <= s_rx;
            s_thr <= s_k_thr;
            s_ink <= s_k_ink(9 downto 2);

            -- resolution scaling: mode steps (static per mode)
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_vys <= f_vstep_y(registers_in(8)(3 downto 0));
            -- cell limits (size - step, floored at 0: a cell no wider than a
            -- step makes every pixel / line its own cell)
            v_hl := signed(resize(s_pixn & '0', 10)) - signed(resize(s_vxs, 10));
            if v_hl(9) = '1' then s_hlim <= (others => '0'); else s_hlim <= unsigned(v_hl(7 downto 0)); end if;
            v_vl := signed(resize(s_rows & x"00", 17)) - signed(resize(s_vys, 17));
            if v_vl(16) = '1' then s_vlim <= (others => '0'); else s_vlim <= unsigned(v_vl(15 downto 0)); end if;
            -- lattice differences, scaled by the x step (settle in 2 clocks;
            -- the lattice values change >= 10 clocks before they are used)
            s_dCB  <= resize(s_vC, 11) - resize(s_vB, 11);
            s_dBA  <= resize(s_vB, 11) - resize(s_vA, 11);
            s_dCBx <= f_xmul(s_dCB, s_vxs);
            s_dBAx <= f_xmul(s_dBA, s_vxs);
            -- virtual x of the next pixel: a lattice column every 64 units
            if data_in.avid = '1' then
                v_xl := resize(s_xa(6 downto 0), 8) + resize(s_vxs, 8);
                s_xa <= s_xa + resize(s_vxs, 13);
                s_xc <= v_xl(7);
                s_xr <= v_xl(1 downto 0);
            else
                s_xa <= (others => '0'); s_xc <= '0'; s_xr <= "00";
            end if;
            -- noise DDA (roulette), Q7 per virtual unit; a crossing between
            -- pixels (720p) reloads with its remainder (0..1 unit) of the step
            case s_xr is
                when "01"   => v_rm := resize(s_dCB, 13);
                when "10"   => v_rm := shift_left(resize(s_dCB, 13), 1);
                when others => v_rm := (others => '0');
            end case;
            if data_in.avid = '1' then
                if s_xc = '1' then
                    s_nacc <= shift_left(resize(s_vB, 17), 7) + resize(v_rm, 17);
                    s_nstp <= s_dCBx;
                    s_vA <= s_vB; s_vB <= s_vC;
                    s_nrem <= "01"; s_nshift <= '0'; s_nstep <= (others => '0');
                else
                    s_nacc <= s_nacc + resize(s_nstp, 17);
                end if;
            else
                s_nstp <= s_dBAx;
                s_nacc <= shift_left(resize(s_vA, 17), 7) - resize(s_dBAx, 17);
            end if;

            case to_integer(s_nstep) is
                when 0 =>
                    v_key := s_ncy & s_ct; s_ha <= C_SEED_NOISE + v_key;
                    v_key := s_ncy1 & s_ct; s_hb <= C_SEED_NOISE + v_key;
                    s_nstep <= s_nstep + 1;
                when 1 => s_ha <= f_round1(s_ha); s_hb <= f_round1(s_hb); s_nstep <= s_nstep + 1;
                when 2 => s_ha <= f_round2(s_ha); s_hb <= f_round2(s_hb); s_nstep <= s_nstep + 1;
                when 3 => s_ha <= f_round3(s_ha); s_hb <= f_round3(s_hb); s_nstep <= s_nstep + 1;
                when 4 =>
                    s_h0 <= s_ha(15 downto 8);
                    s_hd <= signed('0' & s_hb(15 downto 8)) - signed('0' & s_ha(15 downto 8));
                    s_nstep <= s_nstep + 1;
                when 5 => s_hp <= s_hd * signed('0' & s_nfy); s_nstep <= s_nstep + 1;
                when 6 => s_hl <= signed(resize(s_h0, 10)) + resize(shift_right(s_hp, 5), 10); s_nstep <= s_nstep + 1;
                when 7 => s_hlc <= resize(s_hl - to_signed(128, 10), 9); s_nstep <= s_nstep + 1;
                when 8 => s_hq <= s_hlc * signed('0' & s_amp_eff); s_nstep <= s_nstep + 1;
                when 9 =>
                    v_v := resize(shift_right(s_hq, 9), 10);
                    if s_nshift = '1' then s_vA <= s_vB; s_vB <= s_vC; end if;
                    s_vC <= v_v;
                    s_ct <= s_ct + 1;
                    if s_nrem > 1 then s_nrem <= s_nrem - 1; s_nstep <= (others => '0');
                    else s_nrem <= (others => '0'); s_nstep <= "1111"; end if;
                when others => null;
            end case;

            case to_integer(s_lstep) is
                when 0 =>
                    v_ysum := resize(s_lya(19 downto 8), 16) + s_oy;
                    s_ncy  <= v_ysum(13 downto 5);
                    s_ncy1 <= v_ysum(13 downto 5) + 1;
                    s_nfy  <= v_ysum(4 downto 0);
                    s_ct <= (others => '0'); s_nrem <= "11"; s_nshift <= '1'; s_nstep <= (others => '0');
                    s_lstep <= "11";
                when others => null;
            end case;

            case to_integer(s_vstep) is
                when 0 =>
                    v_a := resize(s_k_noise, 11) + resize(s_p_flow(9 downto 1), 11);
                    if v_a > 1023 then v_a := to_unsigned(1023, 11); end if;
                    s_ampin   <= v_a(9 downto 0);
                    s_n_pre   <= s_k_cells * s_k_cells;
                    s_oy <= s_oy + resize(s_k_speed(9 downto 6), 16) + resize(s_p_flow(9 downto 6), 16);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_amp_pre <= s_ampin * s_ampin;
                    s_pixn <= to_unsigned(1, 7) + resize(s_n_pre(19 downto 14), 7);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_amp_raw <= s_amp_pre(19 downto 10);
                    case to_integer(s_rows_idx) is
                        when 0 => s_rows <= s_pixn;
                        when 1 => s_rows <= '0' & s_pixn(6 downto 1);
                        when 2 => s_rows <= "00" & s_pixn(6 downto 2);
                        when others => s_rows <= "000" & s_pixn(6 downto 3);
                    end case;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    -- displacement in 1080-line px -> real px: 720p x 2/3
                    -- ((a*171 + 128) >> 8), SD x 1/2
                    s_a168 <= shift_left(resize(s_amp_raw, 18), 7) + shift_left(resize(s_amp_raw, 18), 5) + shift_left(resize(s_amp_raw, 18), 3);
                    s_a3r  <= shift_left(resize(s_amp_raw, 18), 1) + resize(s_amp_raw, 18) + 128;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    v_a := resize(s_amp_raw, 11) + 1;
                    case s_vxs is
                        when "011"  => s_amp_eff <= resize(shift_right(s_a168 + s_a3r, 8), 10);
                        when "100"  => s_amp_eff <= v_a(10 downto 1);
                        when others => s_amp_eff <= s_amp_raw;
                    end case;
                    s_vstep <= "111";
                when others =>
                    if s_rows = 0 then s_rows <= to_unsigned(1, 7); end if;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_lya <= s_lya + resize(s_vys, 20);
                -- cell rows in virtual lines (Q.8)
                if s_vloc >= s_vlim then s_vloc <= s_vloc - s_vlim; s_vnew <= '1';
                else s_vloc <= s_vloc + resize(s_vys, 16); s_vnew <= '0'; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_vnew = '1' then s_capture <= '1'; s_wpar <= not s_wpar; s_rbank <= s_wpar;
                else s_capture <= '0'; s_rbank <= s_wpar; end if;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_lya <= (others => '0'); s_vloc <= (others => '0'); s_vnew <= '1'; s_wpar <= '0';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                end if;
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

    p_addr : process(clk)
        variable v_w : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 2: noise value, column
            s_n2 <= s_nacc(16 downto 7);
            s_x2 <= s_x1;
            -- stage 3: cell counter and the two orders: grid moved by noise, or content
            -- (cell width in virtual units, Q.1: wrap by subtracting size - step)
            if s_avid_sr(1) = '0' then s_hloc <= (others => '0'); s_hnew <= '1';
            elsif s_hloc >= s_hlim then s_hloc <= s_hloc - s_hlim; s_hnew <= '1';
            else s_hloc <= s_hloc + resize(s_vxs, 8); s_hnew <= '0'; end if;
            s_u3 <= signed(resize(s_x2, 14)) + resize(s_n2, 14);
            s_n3 <= s_n2;
            s_xh3 <= s_x2;
            -- stage 4: hold (order 0: hold the displaced address -> grid steady, content moves;
            --                 order 1: hold the column, add noise after -> cells move)
            if s_sw_order = '0' then
                if s_hnew = '1' then s_u4 <= s_u3; s_uh <= s_u3; else s_u4 <= s_uh; end if;
            else
                if s_hnew = '1' then s_uh <= signed(resize(s_xh3, 14)); s_u4 <= signed(resize(s_xh3, 14)) + resize(s_n3, 14);
                else s_u4 <= s_uh + resize(s_n3, 14); end if;
            end if;
            s_uc4 <= s_u3;                      -- unpixelated address for chroma when luma-only
            -- stage 5: range
            v_w := signed(resize(s_line_width, 14));
            s_addr_y <= f_edge_addr(s_u4, s_w_edge);
            if s_sw_lumaonly = '1' then s_addr_c <= f_edge_addr(s_uc4, s_w_edge); else s_addr_c <= f_edge_addr(s_u4, s_w_edge); end if;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr_y));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr_y));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_src : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 6 ctx / 7 fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); else s_wet_y <= unsigned(s_rdY0); end if;
            if s_a_rbank = '1' then s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            s_dry7 <= unsigned(s_ring_qy);
            -- stage 8: 2-px gradient on the chosen luma (pixelated fetch or smooth dry)
            if s_sw_blocks = '1' then v_src := s_wet_y; else v_src := s_dry7; end if;
            s_yp1 <= v_src; s_yp2 <= s_yp1;
            s_d8 <= signed('0' & v_src) - signed('0' & s_yp2);
            s_y8 <= s_wet_y; s_u8 <= s_wet_u; s_v8 <= s_wet_v;
            -- stage 9: edge flag
            if s_d8 > signed('0' & s_thr) or s_d8 < -signed('0' & s_thr) then s_edge9 <= '1'; else s_edge9 <= '0'; end if;
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;
            -- stage 10: line target difference
            if s_edge9 = '1' then
                s_e10 <= to_signed(940, 11) - signed('0' & s_y9);
            else
                s_e10 <= (others => '0');
            end if;
            s_edge10 <= s_edge9;
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            -- stage 11: ink product
            s_pe11 <= s_e10 * signed('0' & s_ink);
            s_edge11 <= s_edge10;
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            -- stage 12: apply (over) or edges-only
            if s_sw_over = '1' then
                s_y12 <= f_clamp10(signed(resize(s_y11, 12)) + resize(shift_right(s_pe11, 8), 12));
                s_u12 <= s_u11; s_v12 <= s_v11;
            elsif s_edge11 = '1' then
                s_y12 <= f_clamp10(to_signed(64, 12) + resize(shift_right(s_pe11, 8), 12) + resize(shift_right(s_pe11, 8), 12));
                if s_sw_colour = '1' then s_u12 <= s_u11; s_v12 <= s_v11; else s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10); end if;
            else
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            if s_avid_sr(12) = '1' then
                s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
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
    -- latency changes (14 -> 16).
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

end architecture etchwarp;
