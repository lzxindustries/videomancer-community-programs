-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Lenscreen v0.2.0: catalogue combo #106 (seed 20260925) -- composite
-- luma/chroma crosstalk + halftone + lens (sphere / tunnel).  Two knobs
-- each; the switches and the slider decide how they combine.
--
--   K1 Bulge    lens strength, bipolar (pinch .. bulge), glided
--   K2 Radius   lens radius, 3 .. 140% of the half-height, glided
--   K3 Cell     halftone cell 4 / 8 / 16 / 32 px
--   K4 Dot      dot shape: circle / diamond / square / line
--   K5 Cross-Y  chroma leaking into luma (dot crawl) strength
--   K6 Cross-C  fine luma detail leaking into chroma strength
--
--   S7  Lens    sphere / tunnel
--   S8  Screen  dots on the screen grid / dots warped with the lens
--   S9  Ink     black dots on white / dots take the picture colour
--   S10 Cross-Y rattles the dots (before the screen) / flickers the print
--   S11 Fringe  cross-colour phase alternates per line (rainbow comb)
--   P12 Squeeze horizontal scale, 50% (2x stretch) .. 200% (two widths), glided
--
-- Each row shows the previous line from a dual-bank line buffer through a
-- per-row lens DDA (24 bits, source address saturated, mirrored once about
-- a 12-px inset, black beyond).  Replayed rows swap U/V where their
-- hsync->avid parity differs from the capture row's (drape v1.3).  The
-- halftone screen is a cell-local distance test against the luma, scaled
-- so black inks the whole cell and white clears it.  Cross-luma rides a
-- 4-phase U/V subcarrier (period 8 px HD / 4 px SD) that turns 90 degrees
-- a line and a field, so it crawls; cross-colour (the luma high-pass) lands
-- on every pixel, paper and ink.  Output: ink 64, paper 768, [1 2 1] on
-- Y/U/V (the HD HDMI bar guard), blanking-gated.  Latency 20 clocks in all
-- modes.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The halftone cells and the lens
-- run in virtual 1080-line units stepped from the video timing ID
-- (registers_in(8)): x 1.0 / 1.5 / 2.0 units a pixel (1080 / 720p / SD),
-- y 1080 / active lines a field line (interlace included).  A cell is the
-- same fraction of the screen height in every mode and square on a 4:3 SD
-- screen (it was 2:1 tall and ~4x coarser in SD, 1.5x at 720p); the lens
-- band's centre, radius and minimum radius are virtual too.  The dot-crawl
-- carrier and the Fringe comb stay at pixel / line scale.  1080p is
-- bit-identical (every step is 1.0 there).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lenscreen of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LAT   : integer := 20;
    constant C_EDGE  : integer := 12;     -- capture blanking columns kept out of the mirror
    constant C_INK   : integer := 64;     -- ink luma (black)
    constant C_PAPER : integer := 768;    -- paper luma (white, under the bar-guard ceiling)
    constant C_YMAX  : integer := 775;    -- luma ceiling (HD HDMI bar guard)
    constant C_BLK   : integer := 72;     -- luma at or below this inks the whole cell

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

    type t_tab is array (0 to 63) of integer range 0 to 2047;
    constant C_SPH : t_tab := (256, 256, 256, 256, 257, 257, 257, 258, 258, 259, 259, 260, 261, 261, 262, 263, 264, 266, 267, 268, 269, 271, 273, 274, 276, 278, 280, 282, 285, 287, 290, 293, 296, 299, 302, 306, 310, 314, 318, 323, 328, 333, 339, 346, 353, 360, 368, 377, 387, 398, 410, 424, 439, 457, 477, 501, 529, 563, 606, 661, 736, 846, 1032, 1454);
    constant C_TUN : t_tab := (256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 244, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219, 217, 214, 212, 209, 206, 203, 200, 197, 193, 190, 186, 182, 178, 174, 169, 165, 160, 155, 149, 143, 137, 131, 124, 116, 108, 99, 89, 77, 63, 45);

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    -- the bank being written is never the one shown (its read data is dropped)
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

    -- per-frame controls (vblank sequencer)
    signal s_d3, s_d4   : signed(10 downto 0) := (others => '0');
    signal s_k3h, s_k4h : unsigned(9 downto 0) := (others => '0');
    signal s_k3m, s_k4m : std_logic := '0';  -- knob has left its label index
    signal s_eb, s_er, s_ez : signed(13 downto 0) := (others => '0');
    signal s_gb  : unsigned(12 downto 0) := to_unsigned(6400, 13);   -- glided K1 x 8
    signal s_gr  : unsigned(12 downto 0) := to_unsigned(4800, 13);   -- glided K2 x 8
    signal s_gz  : unsigned(12 downto 0) := to_unsigned(2720, 13);   -- glided P12 x 8
    signal s_tun, s_lensed, s_colour, s_after, s_fringe : std_logic := '0';
    signal s_kcy, s_kcc : unsigned(7 downto 0) := (others => '0');
    signal s_ci     : unsigned(1 downto 0) := "01";
    signal s_di     : unsigned(1 downto 0) := "00";
    signal s_cmask  : unsigned(5 downto 0) := "000111";
    signal s_half   : unsigned(5 downto 0) := "000100";
    signal s_sh     : unsigned(3 downto 0) := "0101";
    signal s_dot    : unsigned(1 downto 0) := "00";
    signal s_kb     : signed(8 downto 0) := (others => '0');
    signal s_zoom   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_rm     : unsigned(9 downto 0) := to_unsigned(428, 10);
    signal s_wf     : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_hh     : unsigned(9 downto 0) := to_unsigned(135, 10);
    signal s_hd     : std_logic := '0';
    signal s_rph, s_rpl : unsigned(14 downto 0) := (others => '0');
    signal s_rsum   : unsigned(19 downto 0) := (others => '0');
    signal s_rad    : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_lo_b, s_hi_b, s_2hi, s_lo_lim, s_hi_lim : signed(12 downto 0) := (others => '0');
    signal s_vstep  : unsigned(4 downto 0) := (others => '1');
    signal s_dq     : unsigned(16 downto 0) := (others => '0');
    signal s_dr     : unsigned(11 downto 0) := (others => '0');
    signal s_dbit   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_r  : unsigned(11 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_fcnt   : unsigned(1 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';
    -- per line (lens sequencer after hsync)
    signal s_lstep  : unsigned(3 downto 0) := (others => '1');
    signal s_ady    : unsigned(10 downto 0) := (others => '0');
    signal s_beyond : std_logic := '0';
    signal s_tph, s_tpl : unsigned(16 downto 0) := (others => '0');
    signal s_tp     : unsigned(22 downto 0) := (others => '0');
    signal s_t6     : unsigned(5 downto 0) := (others => '0');
    signal s_f      : unsigned(10 downto 0) := (others => '0');
    signal s_fm1    : signed(11 downto 0) := (others => '0');
    signal s_fk     : signed(20 downto 0) := (others => '0');
    signal s_srel   : signed(11 downto 0) := (others => '0');
    signal s_sz     : signed(21 downto 0) := (others => '0');
    signal s_srow   : signed(12 downto 0) := to_signed(256, 13);
    signal s_cxh, s_cxl : unsigned(16 downto 0) := (others => '0');
    signal s_cxs    : unsigned(23 downto 0) := (others => '0');
    signal s_base   : signed(23 downto 0) := (others => '0');
    signal s_src0   : signed(23 downto 0) := (others => '0');
    -- line capture / read side (switched at the avid edges)
    signal s_wpar, s_wb, s_rb_nx, s_rbank : std_logic := '0';
    signal s_l0_nx, s_line0, s_hpar, s_swap : std_logic := '0';
    signal s_cpar   : std_logic_vector(1 downto 0) := "00";
    signal s_prow   : unsigned(10 downto 0) := (others => '0');
    signal s_qb     : unsigned(1 downto 0) := (others => '0');
    signal s_cyl    : signed(6 downto 0) := (others => '0');
    signal s_syl    : unsigned(11 downto 0) := (others => '0');
    signal s_ayl    : unsigned(5 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_we : std_logic := '0';
    -- resolution scaling: steps (free-running from the timing ID), the
    -- line's virtual y (Q.8) and the field's virtual height
    signal s_xs   : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_vya  : unsigned(19 downto 0) := (others => '0');
    signal s_vht  : unsigned(10 downto 0) := to_unsigned(270, 11);

    -- pixel path (E<n> = registered n clocks after data_in)
    type t_x is array (1 to 7) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    signal s_vx7 : unsigned(6 downto 0) := (others => '0');   -- virtual x of s_x(7), Q.1 mod 64
    type t_b is array (1 to 9) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc  : signed(23 downto 0) := (others => '0');
    signal s_src4 : signed(12 downto 0) := (others => '0');
    signal s_src5 : signed(12 downto 0) := (others => '0');
    type t_src is array (5 to 8) of unsigned(5 downto 0);
    signal s_srcd : t_src := (others => (others => '0'));
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg, s_a_bg : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_q8   : unsigned(1 downto 0) := (others => '0');
    signal s_qs9, s_qs10 : std_logic := '0';
    signal s_ywp1, s_ywp2 : unsigned(9 downto 0) := (others => '0');
    signal s_cx9  : signed(6 downto 0) := (others => '0');
    signal s_cc9, s_ydf9 : signed(10 downto 0) := (others => '0');
    signal s_sx10 : unsigned(11 downto 0) := (others => '0');
    signal s_ax10 : unsigned(5 downto 0) := (others => '0');
    signal s_pcy10, s_pcc10 : signed(19 downto 0) := (others => '0');
    signal s_d11  : unsigned(12 downto 0) := (others => '0');
    signal s_dly11, s_dlc11 : signed(11 downto 0) := (others => '0');
    signal s_ycx12 : unsigned(9 downto 0) := (others => '0');
    type t_dn is array (12 to 14) of unsigned(10 downto 0);
    signal s_dn : t_dn := (others => (others => '0'));
    type t_dl is array (12 to 15) of signed(11 downto 0);
    signal s_dly, s_dlc : t_dl := (others => (others => '0'));
    signal s_yl13 : unsigned(9 downto 0) := (others => '0');
    signal s_yg14 : unsigned(10 downto 0) := (others => '0');
    signal s_ink15 : std_logic := '0';
    signal s_y16  : signed(12 downto 0) := to_signed(64, 13);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_lk16 : signed(11 downto 0) := (others => '0');
    signal s_fneg16 : std_logic := '0';
    signal s_y17  : signed(12 downto 0) := to_signed(64, 13);
    signal s_u17, s_v17 : signed(12 downto 0) := to_signed(512, 13);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_g1y, s_g2y, s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (9 to 15) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    p_ctrl : process(clk)
        variable v_r  : unsigned(12 downto 0);
        variable v_s  : signed(13 downto 0);
        variable v_g  : signed(13 downto 0);
        variable v_kb : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_wb <= s_wpar;
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            -- hsync->avid parity (drape): clock parity since hsync, per line
            s_hpar <= not s_hpar;
            -- this row's cell-local y: squared and absolute (set at the avid
            -- rise, settled long before the row's pixels arrive)
            s_syl <= unsigned(resize(s_cyl * s_cyl, 12));
            if s_cyl < 0 then s_ayl <= unsigned(resize(-s_cyl, 6)); else s_ayl <= unsigned(resize(s_cyl, 6)); end if;

            -- per-frame sequencer, one small op per state, in vblank
            case to_integer(s_vstep) is
                when 0 =>
                    s_d3 <= signed(resize(unsigned(registers_in(2)), 11)) - signed(resize(s_k3h, 11));
                    s_d4 <= signed(resize(unsigned(registers_in(3)), 11)) - signed(resize(s_k4h, 11));
                    if unsigned(registers_in(2)(9 downto 2)) /= 0 then s_k3m <= '1'; end if;
                    if unsigned(registers_in(3)(9 downto 2)) /= 0 then s_k4m <= '1'; end if;
                    s_eb <= signed(shift_left(resize(unsigned(registers_in(0)), 14), 3)) - signed(resize(s_gb, 14));
                    s_er <= signed(shift_left(resize(unsigned(registers_in(1)), 14), 3)) - signed(resize(s_gr, 14));
                    s_ez <= signed(shift_left(resize(unsigned(registers_in(7)), 14), 3)) - signed(resize(s_gz, 14));
                    s_tun    <= registers_in(6)(0);
                    s_lensed <= registers_in(6)(1);
                    s_colour <= registers_in(6)(2);
                    s_after  <= registers_in(6)(3);
                    s_fringe <= registers_in(6)(4);
                    s_kcy <= unsigned(registers_in(4)(9 downto 2));
                    s_kcc <= unsigned(registers_in(5)(9 downto 2));
                    s_wf <= s_line_width;
                    s_hh <= s_vht(10 downto 1);
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_vht(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- deadband the stepped knobs (no flicker at a step edge)
                    if s_d3 > 12 or s_d3 < -12 then s_k3h <= unsigned(registers_in(2)); end if;
                    if s_d4 > 12 or s_d4 < -12 then s_k4h <= unsigned(registers_in(3)); end if;
                    -- glides: ease by diff/8 a frame
                    v_g := signed('0' & s_gb) + shift_right(s_eb, 3); s_gb <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_gr) + shift_right(s_er, 3); s_gr <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_gz) + shift_right(s_ez, 3); s_gz <= unsigned(v_g(12 downto 0));
                    s_hd <= s_wf(10);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- labelled knobs: until a knob first moves, the firmware
                    -- holds its label index (0..3) there, not a position
                    if s_k3m = '0' and unsigned(registers_in(2)(9 downto 2)) = 0 then
                        s_ci <= unsigned(registers_in(2)(1 downto 0));
                    else
                        s_ci <= s_k3h(9 downto 8);
                    end if;
                    if s_k4m = '0' and unsigned(registers_in(3)(9 downto 2)) = 0 then
                        s_di <= unsigned(registers_in(3)(1 downto 0));
                    else
                        s_di <= s_k4h(9 downto 8);
                    end if;
                    v_kb := signed(resize(s_gb(12 downto 3), 11)) - 512;
                    s_kb <= resize(shift_right(v_kb, 1), 9);
                    s_zoom <= to_unsigned(128, 9) + resize(s_gz(12 downto 5), 9) + resize(s_gz(12 downto 6), 9);
                    -- radius multiplier 16 .. 717 (/512 of the half-height)
                    s_rm <= to_unsigned(16, 10) + resize(s_gr(12 downto 4), 10) + resize(s_gr(12 downto 6), 10) + resize(s_gr(12 downto 7), 10);
                    s_lo_b <= to_signed(C_EDGE, 13);
                    s_hi_b <= signed(resize(s_wf, 13)) - (C_EDGE + 1);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    case to_integer(s_ci) is
                        when 0 => s_cmask <= "000011"; s_half <= "000010";
                        when 1 => s_cmask <= "000111"; s_half <= "000100";
                        when 2 => s_cmask <= "001111"; s_half <= "001000";
                        when others => s_cmask <= "011111"; s_half <= "010000";
                    end case;
                    -- distance << sh spans 0..1024: circle r^2 (max 2h^2),
                    -- diamond |x|+|y| (2h), square / line (h)
                    case to_integer(s_di) is
                        when 0 => s_sh <= to_unsigned(7, 4) - (s_ci & '0');
                        when 1 => s_sh <= to_unsigned(8, 4) - s_ci;
                        when others => s_sh <= to_unsigned(9, 4) - s_ci;
                    end case;
                    s_dot <= s_di;
                    s_rph <= s_hh * s_rm(9 downto 5);
                    s_rpl <= s_hh * s_rm(4 downto 0);
                    s_2hi <= shift_left(s_hi_b, 1);
                    s_lo_lim <= to_signed(2 * C_EDGE, 13) - s_hi_b;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_rsum <= shift_left(resize(s_rph, 20), 5) + resize(s_rpl, 20);
                    s_hi_lim <= s_2hi - C_EDGE;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    if s_rsum(19 downto 13) = 0 then
                        s_rad <= to_unsigned(16, 11);
                    else
                        s_rad <= s_rsum(19 downto 9);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(16, 5);
                    s_vstep <= s_vstep + 1;
                when 7 to 23 =>
                    -- restoring divide: inv_r = 65536 / rad
                    if s_dbit = 16 then v_r := (s_dr & '1'); else v_r := (s_dr & '0'); end if;
                    if v_r >= resize(s_rad, 13) then
                        s_dr <= resize(v_r - resize(s_rad, 13), 12);
                        s_dq <= s_dq(15 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0);
                        s_dq <= s_dq(15 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    s_vstep <= s_vstep + 1;
                when 24 =>
                    if s_dq(16 downto 12) /= 0 then s_inv_r <= (others => '1'); else s_inv_r <= s_dq(11 downto 0); end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- per-line lens: the row's source step and DDA start (STA times
            -- these at the pixel clock, so the wide products are split hi/lo)
            case to_integer(s_lstep) is
                when 0 =>
                    if s_vya(18 downto 8) >= s_cy then s_ady <= s_vya(18 downto 8) - s_cy; else s_ady <= s_cy - s_vya(18 downto 8); end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_tph <= s_ady * s_inv_r(11 downto 6);
                    s_tpl <= s_ady * s_inv_r(5 downto 0);
                    if s_ady >= s_rad then s_beyond <= '1'; else s_beyond <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_tp <= shift_left(resize(s_tph, 23), 6) + resize(s_tpl, 23);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_beyond = '1' then s_t6 <= (others => '0'); else s_t6 <= s_tp(15 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_tun = '1' then s_f <= to_unsigned(C_TUN(to_integer(s_t6)), 11); else s_f <= to_unsigned(C_SPH(to_integer(s_t6)), 11); end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_beyond = '1' then s_fm1 <= (others => '0'); else s_fm1 <= signed(resize(s_f, 12)) - 256; end if;
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_fk <= s_fm1 * s_kb;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    -- 14 bits before the clamp: the lens term must not fold
                    v_s := to_signed(256, 14) + resize(shift_right(s_fk, 7), 14);
                    if v_s < 32 then
                        s_srel <= to_signed(32, 12);
                    elsif v_s > 2047 then
                        s_srel <= to_signed(2047, 12);
                    else
                        s_srel <= v_s(11 downto 0);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_sz <= s_srel * signed('0' & s_zoom);
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    -- row step 1/16 .. 16 px a pixel (13 bits: no wrap to magnify)
                    v_s := resize(shift_right(s_sz, 8), 14);
                    if v_s < 16 then
                        s_srow <= to_signed(16, 13);
                    elsif v_s > 4095 then
                        s_srow <= to_signed(4095, 13);
                    else
                        s_srow <= v_s(12 downto 0);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    -- srow is 16..4095 (positive): cx x srow as two 11x6 products
                    s_cxh <= s_cx * unsigned(s_srow(11 downto 6));
                    s_cxl <= s_cx * unsigned(s_srow(5 downto 0));
                    s_base <= shift_left(resize(signed('0' & s_cx), 24), 8) - resize(s_srow, 24);
                    s_lstep <= s_lstep + 1;
                when 11 =>
                    s_cxs <= shift_left(resize(s_cxh, 24), 6) + resize(s_cxl, 24);
                    s_lstep <= s_lstep + 1;
                when 12 =>
                    s_src0 <= s_base - signed(s_cxs);
                    s_lstep <= (others => '1');
                when others => null;
            end case;

            -- end of a line: the next line writes the other bank and shows
            -- this one
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                s_vya <= s_vya + s_ys;
                s_vht <= resize(shift_right(s_vya + s_ys, 8), 11);
                s_rb_nx <= s_wpar;
                s_wpar  <= not s_wpar;
                s_l0_nx <= '0';
            end if;
            -- start of a line: the read side and the row switch here (the
            -- previous line's pixels have left the pipe)
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_rbank <= s_rb_nx;
                if s_rb_nx = '1' then s_swap <= s_hpar xor s_cpar(1); else s_swap <= s_hpar xor s_cpar(0); end if;
                if s_wpar = '1' then s_cpar(1) <= s_hpar; else s_cpar(0) <= s_hpar; end if;
                s_line0 <= s_l0_nx;
                s_prow  <= s_line;
                s_qb    <= s_line(1 downto 0) + s_fcnt;
                s_cyl   <= signed(resize(s_vya(13 downto 8) and s_cmask, 7)) - signed(resize(s_half, 7));
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0');
                s_hpar  <= '0';
            end if;
            -- frame start (idempotent: vsync may serrate): line 0 writes
            -- bank 0 and shows black
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line  <= (others => '0');
                s_vya   <= (others => '0');
                s_wpar  <= '0';
                s_rb_nx <= '1';
                s_l0_nx <= '1';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                    s_fcnt  <= s_fcnt + 1;
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
        variable v_s13 : signed(12 downto 0);
        variable v_s7  : unsigned(6 downto 0);
        variable v_c, v_ml, v_mh : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- E2: pixel x
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 7 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 9 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- virtual x beside s_x(7) (Q.1, kept mod 64 units: the cells
            -- use the low 6 bits)
            if s_avd(7) = '1' then s_vx7 <= s_vx7 + s_xs; else s_vx7 <= (others => '0'); end if;
            -- E3: DDA (1/256 px)
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 24); end if;
            -- E4: source x, saturated to +-4096 (squeezed rows never wrap
            -- back onto the line)
            if s_acc(23 downto 20) = "0000" or s_acc(23 downto 20) = "1111" then
                v_s13 := s_acc(20 downto 8);
            elsif s_acc(23) = '1' then
                v_s13 := to_signed(-4096, 13);
            else
                v_s13 := to_signed(4095, 13);
            end if;
            s_src4 <= v_s13;
            -- E5
            s_src5 <= s_src4;
            -- source x in virtual units, mod 64 (x 1 / 1.5 / 2)
            case to_integer(s_xs) is
                when 3 =>
                    v_s7 := unsigned(s_src4(6 downto 0)) + (unsigned(s_src4(5 downto 0)) & '0');
                    s_srcd(5) <= v_s7(6 downto 1);
                when 4 =>
                    s_srcd(5) <= unsigned(s_src4(4 downto 0)) & '0';
                when others =>
                    s_srcd(5) <= unsigned(s_src4(5 downto 0));
            end case;
            for i in 6 to 8 loop s_srcd(i) <= s_srcd(i - 1); end loop;
            -- E6: edges, mirrored once about the inset (bounds per frame;
            -- compares run beside the subtracts), black beyond
            v_c  := s_src5;
            v_ml := to_signed(2 * C_EDGE, 13) - v_c;
            v_mh := s_2hi - v_c;
            if s_avd(4) = '0' then
                s_bg <= '1'; s_addr <= (others => '0');
            elsif v_c < s_lo_b then
                if v_c >= s_lo_lim then s_bg <= '0'; s_addr <= unsigned(v_ml(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            elsif v_c > s_hi_b then
                if v_c <= s_hi_lim then s_bg <= '0'; s_addr <= unsigned(v_mh(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            else
                s_bg <= '0'; s_addr <= unsigned(v_c(10 downto 0));
            end if;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wb = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wb = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wb = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wb = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wb = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wb = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_q : unsigned(1 downto 0);
        variable v_base : signed(12 downto 0);
        variable v_ys, v_us, v_vs : signed(12 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- E7 (rd) / E8: fetch; black outside the line and on the frame's
            -- first line; U/V swapped on rows whose parity differs from the
            -- capture row's
            s_a_bg <= s_bg;
            if s_rbank = '1' then
                v_y := unsigned(s_rdY1); v_u := unsigned(s_rdU1) & "00"; v_v := unsigned(s_rdV1) & "00";
            else
                v_y := unsigned(s_rdY0); v_u := unsigned(s_rdU0) & "00"; v_v := unsigned(s_rdV0) & "00";
            end if;
            if s_a_bg = '1' or s_line0 = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_swap = '1' then
                s_wet_y <= v_y; s_wet_u <= v_v; s_wet_v <= v_u;
            else
                s_wet_y <= v_y; s_wet_u <= v_u; s_wet_v <= v_v;
            end if;
            -- E8: subcarrier quarter: 2 px a quarter at HD, 1 px in SD; +90
            -- degrees a line and a field (crawls up a line a field)
            if s_hd = '1' then v_q := s_x(6)(2 downto 1); else v_q := s_x(6)(1 downto 0); end if;
            s_q8 <= v_q + s_qb;
            -- E9: cell-local x, the carrier's chroma (U, V, -U, -V), luma high-pass
            s_ywp1 <= s_wet_y; s_ywp2 <= s_ywp1;
            if s_lensed = '1' then
                s_cx9 <= signed(resize(s_srcd(8) and s_cmask, 7)) - signed(resize(s_half, 7));
            else
                s_cx9 <= signed(resize(s_vx7(6 downto 1) and s_cmask, 7)) - signed(resize(s_half, 7));
            end if;
            if s_q8(0) = '1' then
                s_cc9 <= signed(resize(s_wet_v, 11)) - 512;
            else
                s_cc9 <= signed(resize(s_wet_u, 11)) - 512;
            end if;
            s_qs9 <= s_q8(1);
            if s_avd(9) = '1' then   -- the pixel two back is on the line
                s_ydf9 <= signed(resize(s_wet_y, 11)) - signed(resize(s_ywp2, 11));
            else
                s_ydf9 <= (others => '0');
            end if;
            s_yd(9) <= s_wet_y; s_ud(9) <= s_wet_u; s_vd(9) <= s_wet_v;
            for i in 10 to 15 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- E10: x square / abs, crosstalk products
            s_sx10 <= unsigned(resize(s_cx9 * s_cx9, 12));
            if s_cx9 < 0 then s_ax10 <= unsigned(resize(-s_cx9, 6)); else s_ax10 <= unsigned(resize(s_cx9, 6)); end if;
            s_pcy10 <= s_cc9 * signed(resize(s_kcy, 9));
            s_pcc10 <= s_ydf9 * signed(resize(s_kcc, 9));
            s_qs10 <= s_qs9;
            -- E11: distance measure (row terms per line), crosstalk terms
            case to_integer(s_dot) is
                when 0 => s_d11 <= resize(s_sx10, 13) + resize(s_syl, 13);
                when 1 => s_d11 <= resize(s_ax10, 13) + resize(s_ayl, 13);
                when 2 => if s_ax10 > s_ayl then s_d11 <= resize(s_ax10, 13); else s_d11 <= resize(s_ayl, 13); end if;
                when others => s_d11 <= resize(s_ayl, 13);
            end case;
            if s_qs10 = '1' then s_dly11 <= -resize(shift_right(s_pcy10, 7), 12); else s_dly11 <= resize(shift_right(s_pcy10, 7), 12); end if;
            s_dlc11 <= resize(shift_right(s_pcc10, 7), 12);
            -- E12: cross-luma before the screen (Dots); distance into 0..1024
            if s_after = '0' then s_ycx12 <= f_clamp10(signed(resize(s_yd(11), 13)) + resize(s_dly11, 13)); else s_ycx12 <= s_yd(11); end if;
            s_dn(12) <= shift_left(resize(s_d11, 11), to_integer(s_sh));
            s_dn(13) <= s_dn(12); s_dn(14) <= s_dn(13);
            s_dly(12) <= s_dly11; s_dlc(12) <= s_dlc11;
            for i in 13 to 15 loop s_dly(i) <= s_dly(i - 1); s_dlc(i) <= s_dlc(i - 1); end loop;
            -- E13: black-relative luma (the bottom 8 codes ink solid)
            if s_ycx12 <= C_BLK then s_yl13 <= (others => '0'); else s_yl13 <= s_ycx12 - C_BLK; end if;
            -- E14: x 1.25, so white (~895 and up) clears the whole cell
            s_yg14 <= resize(s_yl13, 11) + resize(s_yl13(9 downto 2), 11);
            -- E15: ink decision (dark luma -> more ink)
            if s_yg14 <= s_dn(14) then s_ink15 <= '1'; else s_ink15 <= '0'; end if;
            -- E16: compose: paper white, ink black or the picture's colour;
            -- cross-luma after the screen (Print) flickers the print
            if s_ink15 = '1' then v_base := to_signed(C_INK, 13); else v_base := to_signed(C_PAPER, 13); end if;
            if s_after = '1' then s_y16 <= v_base + resize(s_dly(15), 13); else s_y16 <= v_base; end if;
            if s_colour = '1' and s_ink15 = '1' then
                s_u16 <= s_ud(15); s_v16 <= s_vd(15);
            else
                s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
            s_lk16 <= s_dlc(15);
            s_fneg16 <= s_fringe and s_prow(0);
            -- E17: cross-colour onto every pixel (paper and ink, Mono and
            -- Colour); Fringe flips its V on odd rows
            s_y17 <= s_y16;
            s_u17 <= signed(resize(s_u16, 13)) + resize(s_lk16, 13);
            if s_fneg16 = '1' then
                s_v17 <= signed(resize(s_v16, 13)) - resize(s_lk16, 13);
            else
                s_v17 <= signed(resize(s_v16, 13)) + resize(s_lk16, 13);
            end if;
            -- E18: clamp (luma 64..775, chroma 64..960); neutral outside the
            -- avid (the smoothing's edges)
            if s_y17 < 64 then v_ys := to_signed(64, 13); elsif s_y17 > C_YMAX then v_ys := to_signed(C_YMAX, 13); else v_ys := s_y17; end if;
            if s_u17 < 64 then v_us := to_signed(64, 13); elsif s_u17 > 960 then v_us := to_signed(960, 13); else v_us := s_u17; end if;
            if s_v17 < 64 then v_vs := to_signed(64, 13); elsif s_v17 > 960 then v_vs := to_signed(960, 13); else v_vs := s_v17; end if;
            if s_avid_sr(16) = '1' then
                s_y18 <= unsigned(v_ys(9 downto 0)); s_u18 <= unsigned(v_us(9 downto 0)); s_v18 <= unsigned(v_vs(9 downto 0));
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            -- E19..E20: horizontal [1 2 1]/4 on Y/U/V (g1 the centre; the HD
            -- HDMI bar guard), gated
            s_g1y <= s_y18; s_g2y <= s_g1y;
            s_g1u <= s_u18; s_g2u <= s_g1u;
            s_g1v <= s_v18; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y18, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u18, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v18, 12);
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y20 <= v_sy(11 downto 2); s_u20 <= v_su(11 downto 2); s_v20 <= v_sv(11 downto 2);
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

end architecture lenscreen;
