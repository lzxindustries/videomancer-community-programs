-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Snowglobe v0.2.0: catalogue combo #68 (seed 20260922) -- pixelation + RF
-- noise / snow / VHS tracking + polar warp (sphere / tunnel).  Two knobs
-- each; the switches and the slider decide how they combine.
--
--   K1 Cell     cell width 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 px
--   K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 lines
--   K3 Snow     snow amount; above 50% the snow keeps growing and the
--               picture fades into it (100% = a lost channel)
--   K4 Tracking fraction of lines that lose tracking (torn and dark)
--   K5 Bulge    lens strength, bipolar (pinch .. bulge)
--   K6 Radius   lens radius, 3 .. 140% of the half-height
--
--   S7  Cells   cells in lens (source) space / in screen space
--   S8  Snow    per pixel / held per cell (the picture's own cells)
--   S9  Lens    sphere / tunnel
--   S10 Edges   outside the line: black / mirrored once (12-px inset)
--   S11 Snow    mono / coloured
--   P12 Squeeze horizontal scale, 50% (2x stretch) .. 200% (two widths)
--
-- Rows: the first line of each cell row captures into one bank while the
-- rows below show the other, so the picture sits 1 line low and the frame's
-- first line is black.  Replayed rows swap U/V where their hsync->avid
-- parity differs from the capture row's (drape v1.3).  The lens DDA runs at
-- 24 bits and the source address saturates, so squeezed rows never wrap.
-- Snow = a two-round shift-add hash of (column, row, per-frame seed), where
-- column/row are the pixel or its cell.  Output: [1 2 1] on Y/U/V + luma
-- scaled to white ~775 (the HD HDMI bar fix), blanking-gated.  Latency 20
-- clocks in all modes.
--
-- v0.2.2 (2026-10-17): resolution scaling.  Cells, rows, the lens and the
-- tracking tears run in virtual 1080-line units stepped from the video
-- timing ID (registers_in(8)): x 1.0 / 1.5 / 2.0 units a pixel (1080 / 720p
-- / SD), y 1080 / active lines a field line (interlace included).  Cell
-- and row sizes are fractions of the screen height in every mode (square
-- cells stay square on a 4:3 SD screen; SD cells were 2:1 tall and ~4x
-- coarser, 720p 1.5x).  Screen cells hold at virtual-x crossings; lens
-- cells quantize the source column to the virtual grid (720p: 2C/3-px
-- cells via two per-frame thresholds, no divide).  Fine snow and the lost
-- lines stay at pixel / line scale.  1080p is bit-identical.  Also: the
-- Snow amplitude is written without a self-add (router1 livelock), same
-- value.  Build: router1, seed 3.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture snowglobe of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LAT  : integer := 20;
    constant C_EDGE : integer := 12;     -- capture blanking columns kept out of the mirror
    constant C_GREY : integer := 448;    -- the level a lost picture fades to

    -- octave index 0..7 (labelled knob: the firmware loads the label index)
    function f_oct(v : unsigned(9 downto 0)) return integer is
    begin
        if v < 8 then
            return to_integer(v(2 downto 0));
        else
            return to_integer(v(9 downto 7));
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
    -- 720p lens cells: a C-unit cell is 2C/3 px.  Within each 2C-px block
    -- (3 cells) the cells start at 0, ceil(2C/3) and ceil(4C/3).
    type t_thr is array (0 to 7) of integer range 0 to 255;
    constant C_T1 : t_thr := (1, 2, 3, 6, 11, 22, 43, 86);
    constant C_T2 : t_thr := (2, 3, 6, 11, 22, 43, 86, 171);

    type t_mask is array (0 to 7) of unsigned(6 downto 0);
    constant C_MASK : t_mask := ("0000000", "0000001", "0000011", "0000111", "0001111", "0011111", "0111111", "1111111");

    type t_tab is array (0 to 63) of integer range 0 to 2047;
    constant C_SPH : t_tab := (256, 256, 256, 256, 257, 257, 257, 258, 258, 259, 259, 260, 261, 261, 262, 263, 264, 266, 267, 268, 269, 271, 273, 274, 276, 278, 280, 282, 285, 287, 290, 293, 296, 299, 302, 306, 310, 314, 318, 323, 328, 333, 339, 346, 353, 360, 368, 377, 387, 398, 410, 424, 439, 457, 477, 501, 529, 563, 606, 661, 736, 846, 1032, 1454);
    constant C_TUN : t_tab := (256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 244, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219, 217, 214, 212, 209, 206, 203, 200, 197, 193, 190, 186, 182, 178, 174, 169, 165, 160, 155, 149, 143, 137, 131, 124, 116, 108, 99, 89, 77, 63, 45);

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    -- a bank is never read while it is written (the other bank is shown)
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

    -- per-frame controls
    signal s_d1, s_d2   : signed(10 downto 0) := (others => '0');
    signal s_k1h        : unsigned(9 downto 0) := to_unsigned(448, 10);
    signal s_k2h        : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal s_eb, s_er, s_ez : signed(13 downto 0) := (others => '0');
    signal s_gb  : unsigned(12 downto 0) := to_unsigned(6400, 13);   -- glided K5 x 8
    signal s_gr  : unsigned(12 downto 0) := to_unsigned(4800, 13);   -- glided K6 x 8
    signal s_gz  : unsigned(12 downto 0) := to_unsigned(2720, 13);   -- glided P12 x 8
    signal s_scr, s_csn, s_tun, s_mir, s_cols : std_logic := '0';
    signal s_cmask, s_rm1 : unsigned(6 downto 0) := (others => '0');
    signal s_k1i    : unsigned(2 downto 0) := (others => '0');
    -- lens cells in source px: mask, and the 720p in-block cell starts
    -- (256 = never)
    signal s_qm     : unsigned(7 downto 0) := (others => '0');
    signal s_qt1, s_qt2 : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_kb     : signed(8 downto 0) := (others => '0');
    signal s_zoom   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_rm     : unsigned(9 downto 0) := to_unsigned(428, 10);
    signal s_wf     : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_hh     : unsigned(9 downto 0) := to_unsigned(135, 10);
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
    signal s_seed   : unsigned(31 downto 0) := (others => '0');
    signal s_rng    : unsigned(31 downto 0) := x"2545F491";
    signal s_saw_active : std_logic := '0';
    -- snow / tracking (instant)
    signal s_amp    : unsigned(8 downto 0) := (others => '0');
    signal s_kx     : signed(10 downto 0) := (others => '0');
    signal s_fw     : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_track  : unsigned(7 downto 0) := (others => '0');
    -- per line (lens sequencer after hsync; switched in at the next avid rise)
    signal s_lstep  : unsigned(3 downto 0) := (others => '1');
    signal s_ady    : unsigned(10 downto 0) := (others => '0');
    signal s_beyond : std_logic := '0';
    signal s_tp     : unsigned(22 downto 0) := (others => '0');
    signal s_t6     : unsigned(5 downto 0) := (others => '0');
    signal s_f      : unsigned(10 downto 0) := (others => '0');
    signal s_fm1    : signed(11 downto 0) := (others => '0');
    signal s_fk     : signed(20 downto 0) := (others => '0');
    signal s_srel   : signed(11 downto 0) := (others => '0');
    signal s_sz     : signed(21 downto 0) := (others => '0');
    signal s_srow   : signed(12 downto 0) := to_signed(256, 13);
    signal s_cxs    : signed(24 downto 0) := (others => '0');
    signal s_ctr    : signed(11 downto 0) := (others => '0');
    signal s_base   : signed(23 downto 0) := (others => '0');
    signal s_src0   : signed(23 downto 0) := (others => '0');
    signal s_tear   : signed(10 downto 0) := (others => '0');
    signal s_lost_nx, s_lost : std_logic := '0';
    signal s_lnz_nx, s_lnz   : unsigned(7 downto 0) := (others => '0');
    -- row capture
    signal s_capture, s_wpar, s_rb_nx, s_rbank : std_logic := '0';
    signal s_l0_nx, s_line0, s_hpar, s_swap : std_logic := '0';
    signal s_cpar   : std_logic_vector(1 downto 0) := "00";
    signal s_ccnt, s_rid_nx, s_hrow : unsigned(10 downto 0) := (others => '0');
    signal s_we, s_wb : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    -- resolution scaling: steps (free-running from the timing ID), the
    -- line's virtual y (Q.8) and the field's virtual height
    signal s_xs   : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_vya  : unsigned(19 downto 0) := (others => '0');
    signal s_vht  : unsigned(10 downto 0) := to_unsigned(270, 11);

    -- pixel path (E<n> = registered n clocks after data_in)
    type t_x is array (1 to 3) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    signal s_vx4  : unsigned(12 downto 0) := (others => '0');   -- virtual x of s_x(3), Q.1
    signal s_cr4  : std_logic := '1';                            -- s_x(3) starts a screen cell
    signal s_off4 : unsigned(7 downto 0) := (others => '0');     -- 720p lens-cell start in its block
    type t_b is array (1 to 4) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc  : signed(23 downto 0) := (others => '0');
    signal s_src4 : signed(12 downto 0) := (others => '0');
    signal s_src5 : signed(12 downto 0) := (others => '0');
    signal s_hcl  : unsigned(15 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg, s_a_bg : std_logic := '0';
    signal s_hk0, s_hk1, s_h1, s_h2, s_h3, s_h4, s_h5, s_h6, s_h7 : unsigned(31 downto 0) := (others => '0');
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_dy9, s_du9, s_dv9 : signed(10 downto 0) := (others => '0');
    signal s_py10, s_pu10, s_pv10 : signed(20 downto 0) := (others => '0');
    type t_f is array (11 to 14) of signed(11 downto 0);
    signal s_yf, s_uf, s_vf : t_f := (others => (others => '0'));
    signal s_ny14, s_nu14, s_nv14 : signed(17 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : signed(11 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : signed(11 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_g1y, s_g2y, s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    p_ctrl : process(clk)
        variable v_r   : unsigned(12 downto 0);
        variable v_s   : signed(13 downto 0);
        variable v_g   : signed(13 downto 0);
        variable v_k3  : unsigned(9 downto 0);
        variable v_t   : signed(10 downto 0);
        variable v_kb  : signed(10 downto 0);
        variable v_x   : unsigned(31 downto 0);
        variable v_ny  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid and s_capture;
            s_wb <= s_wpar;
            -- xorshift32 (per-line decisions, per-frame snow seed)
            v_x := s_rng xor shift_left(s_rng, 13);
            v_x := v_x xor shift_right(v_x, 17);
            s_rng <= v_x xor shift_left(v_x, 5);
            -- hsync->avid parity (drape): clock parity since hsync, per line
            s_hpar <= not s_hpar;
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- snow: amplitude k/4, plus (k-512)/4 above half (up to +-382);
            -- above 62.5% the picture fades toward grey (to 25% at 100%)
            v_k3 := unsigned(registers_in(2));
            if v_k3(9) = '1' then
                -- = k(9:2) + k(8:2) with bit 9 set, written without a self-add
                -- (a carry with both inputs on one net livelocks router1)
                s_amp <= to_unsigned(128, 9) + shift_left(resize(v_k3(8 downto 2), 9), 1);
            else
                s_amp <= resize(v_k3(9 downto 2), 9);
            end if;
            s_kx <= signed(resize(v_k3, 11)) - 640;
            if s_kx > 0 then
                s_fw <= to_unsigned(256, 9) - resize(unsigned(s_kx(9 downto 1)), 9);
            else
                s_fw <= to_unsigned(256, 9);
            end if;
            s_track <= unsigned(registers_in(3)(9 downto 2));

            -- per-frame sequencer, one small op per state, in vblank
            case to_integer(s_vstep) is
                when 0 =>
                    s_d1 <= signed('0' & unsigned(registers_in(0))) - signed('0' & s_k1h);
                    s_d2 <= signed('0' & unsigned(registers_in(1))) - signed('0' & s_k2h);
                    s_eb <= signed(shift_left(resize(unsigned(registers_in(4)), 14), 3)) - signed(resize(s_gb, 14));
                    s_er <= signed(shift_left(resize(unsigned(registers_in(5)), 14), 3)) - signed(resize(s_gr, 14));
                    s_ez <= signed(shift_left(resize(unsigned(registers_in(7)), 14), 3)) - signed(resize(s_gz, 14));
                    s_scr  <= registers_in(6)(0);
                    s_csn  <= registers_in(6)(1);
                    s_tun  <= registers_in(6)(2);
                    s_mir  <= registers_in(6)(3);
                    s_cols <= registers_in(6)(4);
                    s_wf <= s_line_width;
                    s_hh <= s_vht(10 downto 1);
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_vht(10 downto 1);
                    s_seed <= s_rng;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- deadband the stepped knobs (no flicker at a step edge)
                    if s_d1 > 12 or s_d1 < -12 then s_k1h <= unsigned(registers_in(0)); end if;
                    if s_d2 > 12 or s_d2 < -12 then s_k2h <= unsigned(registers_in(1)); end if;
                    -- glides: ease by diff/8 a frame
                    v_g := signed('0' & s_gb) + shift_right(s_eb, 3); s_gb <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_gr) + shift_right(s_er, 3); s_gr <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_gz) + shift_right(s_ez, 3); s_gz <= unsigned(v_g(12 downto 0));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_cmask <= C_MASK(f_oct(s_k1h));
                    s_k1i   <= to_unsigned(f_oct(s_k1h), 3);
                    s_rm1   <= C_MASK(f_oct(s_k2h));
                    v_kb := signed(resize(s_gb(12 downto 3), 11)) - 512;
                    s_kb <= resize(shift_right(v_kb, 1), 9);
                    s_zoom <= to_unsigned(128, 9) + resize(s_gz(12 downto 5), 9) + resize(s_gz(12 downto 6), 9);
                    -- radius multiplier 16 .. 717 (/512 of the half-height)
                    s_rm <= to_unsigned(16, 10) + resize(s_gr(12 downto 4), 10) + resize(s_gr(12 downto 6), 10) + resize(s_gr(12 downto 7), 10);
                    if s_mir = '1' then
                        s_lo_b <= to_signed(C_EDGE, 13);
                        s_hi_b <= signed(resize(s_wf, 13)) - (C_EDGE + 1);
                    else
                        s_lo_b <= (others => '0');
                        s_hi_b <= signed(resize(s_wf, 13)) - 1;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    -- lens cells in source px: C (1080), C/2 (SD), 2C/3 (720p)
                    case to_integer(s_xs) is
                        when 3 =>
                            s_qm  <= s_cmask & '1';
                            s_qt1 <= to_unsigned(C_T1(to_integer(s_k1i)), 9);
                            s_qt2 <= to_unsigned(C_T2(to_integer(s_k1i)), 9);
                        when 4 =>
                            s_qm  <= "00" & s_cmask(6 downto 1);
                            s_qt1 <= to_unsigned(256, 9); s_qt2 <= to_unsigned(256, 9);
                        when others =>
                            s_qm  <= '0' & s_cmask;
                            s_qt1 <= to_unsigned(256, 9); s_qt2 <= to_unsigned(256, 9);
                    end case;
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

            -- per-line lens: the row's source step and DDA start
            case to_integer(s_lstep) is
                when 0 =>
                    if s_vya(18 downto 8) >= s_cy then s_ady <= s_vya(18 downto 8) - s_cy; else s_ady <= s_cy - s_vya(18 downto 8); end if;
                    if s_rng(7 downto 0) < s_track then
                        s_lost_nx <= '1';
                        -- +-512 virtual units (scaled to px at step 1)
                        v_t := signed(resize(s_rng(17 downto 8), 11)) - 512;
                        s_tear <= v_t;
                    else
                        s_lost_nx <= '0';
                        s_tear <= (others => '0');
                    end if;
                    s_lnz_nx <= s_rng(31 downto 24);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    -- tear in px: x 1 (1080), ~2/3 (720p), 1/2 (SD)
                    case to_integer(s_xs) is
                        when 3 => s_tear <= shift_right(s_tear, 1) + shift_right(s_tear, 3) + shift_right(s_tear, 5);
                        when 4 => s_tear <= shift_right(s_tear, 1);
                        when others => null;
                    end case;
                    s_tp <= s_ady * s_inv_r;
                    if s_ady >= s_rad then s_beyond <= '1'; else s_beyond <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_beyond = '1' then s_t6 <= (others => '0'); else s_t6 <= s_tp(15 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_tun = '1' then s_f <= to_unsigned(C_TUN(to_integer(s_t6)), 11); else s_f <= to_unsigned(C_SPH(to_integer(s_t6)), 11); end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_beyond = '1' then s_fm1 <= (others => '0'); else s_fm1 <= signed(resize(s_f, 12)) - 256; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_fk <= s_fm1 * s_kb;
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    v_s := to_signed(256, 14) + resize(shift_right(s_fk, 7), 14);
                    if v_s < 32 then
                        s_srel <= to_signed(32, 12);
                    elsif v_s > 2047 then
                        s_srel <= to_signed(2047, 12);
                    else
                        s_srel <= v_s(11 downto 0);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_sz <= s_srel * signed('0' & s_zoom);
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    v_s := resize(shift_right(s_sz, 8), 14);
                    if v_s < 16 then
                        s_srow <= to_signed(16, 13);
                    elsif v_s > 4095 then
                        s_srow <= to_signed(4095, 13);
                    else
                        s_srow <= v_s(12 downto 0);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    s_cxs <= signed('0' & s_cx) * s_srow;
                    s_ctr <= signed(resize(s_cx, 12)) + resize(s_tear, 12);
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_base <= shift_left(resize(s_ctr, 24), 8) - resize(s_srow, 24);
                    s_lstep <= s_lstep + 1;
                when 11 =>
                    s_src0 <= s_base - resize(s_cxs, 24);
                    s_lstep <= (others => '1');
                when others => null;
            end case;

            -- end of a line: the next line's row slot, capture and read bank
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                v_ny := s_vya + s_ys;
                s_vya <= v_ny;
                s_vht <= v_ny(18 downto 8);
                s_rb_nx  <= s_wpar;           -- the last completed capture
                s_rid_nx <= s_ccnt;
                -- the next line starts a row when its virtual y crosses a
                -- multiple of the row height (bits 7..k: see the notes)
                if ((v_ny(15 downto 8) xor s_vya(15 downto 8)) and not ('0' & s_rm1)) /= 0 then
                    s_capture <= '1';
                    s_wpar    <= not s_wpar;
                    s_ccnt    <= s_ccnt + 1;
                else
                    s_capture <= '0';
                end if;
                s_l0_nx <= '0';
            end if;
            -- start of a line: the read side switches here (the previous
            -- line's pixels have left the pipe)
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_rbank <= s_rb_nx;
                if s_rb_nx = '1' then s_swap <= s_hpar xor s_cpar(1); else s_swap <= s_hpar xor s_cpar(0); end if;
                if s_capture = '1' then
                    if s_wpar = '1' then s_cpar(1) <= s_hpar; else s_cpar(0) <= s_hpar; end if;
                end if;
                s_line0 <= s_l0_nx;
                s_lost  <= s_lost_nx;
                s_lnz   <= s_lnz_nx;
                if s_csn = '1' then s_hrow <= s_rid_nx; else s_hrow <= s_line; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0');
                s_hpar  <= '0';
            end if;
            -- frame start (idempotent: vsync may serrate): line 0 captures
            -- into bank 0 and shows black
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line    <= (others => '0');
                s_vya     <= (others => '0');
                s_capture <= '1';
                s_wpar    <= '0';
                s_rb_nx   <= '1';
                s_l0_nx   <= '1';
                s_ccnt    <= (others => '0');
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

    p_addr : process(clk)
        variable v_s13 : signed(12 downto 0);
        variable v_c, v_ml, v_mh : signed(12 downto 0);
        variable v_col : unsigned(15 downto 0);
        variable v_vx  : unsigned(12 downto 0);
        variable v_r   : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- E2: pixel x
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 3 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 4 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- E4: virtual x beside s_x(3), and whether that pixel starts a
            -- screen cell (its virtual x crossed a cell edge; the line's
            -- first pixel always does).  Bits 7..k decide any crossing.
            if s_avd(3) = '1' then
                v_vx := s_vx4 + s_xs;
                s_vx4 <= v_vx;
                if ((v_vx(8 downto 1) xor s_vx4(8 downto 1)) and not ('0' & s_cmask)) /= 0 then s_cr4 <= '1'; else s_cr4 <= '0'; end if;
            else
                s_vx4 <= (others => '0');
                s_cr4 <= '1';
            end if;            -- E3: DDA (1/256 px)
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 24); end if;
            -- E4: source x, saturated to +-4096 (squeezed rows never wrap
            -- back onto the line); cells in source space
            if s_acc(23 downto 20) = "0000" or s_acc(23 downto 20) = "1111" then
                v_s13 := s_acc(20 downto 8);
            elsif s_acc(23) = '1' then
                v_s13 := to_signed(-4096, 13);
            else
                v_s13 := to_signed(4095, 13);
            end if;
            if s_scr = '1' then
                s_src4 <= v_s13;
            else
                s_src4 <= v_s13 and not signed(resize(s_qm, 13));
            end if;
            s_hcl <= unsigned(s_acc(23 downto 8)) and not resize(s_qm, 16);
            -- 720p: the cell's start inside its 2C-px block (0 elsewhere)
            v_r := '0' & (unsigned(s_acc(15 downto 8)) and s_qm);
            if v_r >= s_qt2 then
                s_off4 <= s_qt2(7 downto 0);
            elsif v_r >= s_qt1 then
                s_off4 <= s_qt1(7 downto 0);
            else
                s_off4 <= (others => '0');
            end if;
            -- E5: cells in screen space = hold the address at cell boundaries;
            -- snow hash key = (row, column) of the pixel or of its cell
            if s_scr = '1' then
                if s_cr4 = '1' then s_src5 <= s_src4; end if;
            else
                s_src5 <= s_src4 or signed(resize(s_off4, 13));
            end if;
            if s_csn = '0' then
                v_col := resize(s_x(3), 16);
            elsif s_scr = '1' then
                v_col := resize(s_vx4(12 downto 1) and not resize(s_cmask, 12), 16);
            else
                v_col := s_hcl or resize(s_off4, 16);
            end if;
            s_hk0 <= "00000" & s_hrow & v_col;
            -- E6: edges (bounds per frame; compares run beside the subtracts)
            v_c  := s_src5;
            v_ml := to_signed(2 * C_EDGE, 13) - v_c;
            v_mh := s_2hi - v_c;
            if s_avd(4) = '0' then
                s_bg <= '1'; s_addr <= (others => '0');
            elsif v_c < s_lo_b then
                if s_mir = '1' and v_c >= s_lo_lim then s_bg <= '0'; s_addr <= unsigned(v_ml(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            elsif v_c > s_hi_b then
                if s_mir = '1' and v_c <= s_hi_lim then s_bg <= '0'; s_addr <= unsigned(v_mh(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            else
                s_bg <= '0'; s_addr <= unsigned(v_c(10 downto 0));
            end if;
            -- E6..E13: snow hash (key + per-frame seed, then two shift-add rounds)
            s_hk1 <= s_hk0 + s_seed;
            s_h1 <= s_hk1 + shift_left(s_hk1, 10);
            s_h2 <= s_h1 xor shift_right(s_h1, 6);
            s_h3 <= s_h2 + shift_left(s_h2, 10);
            s_h4 <= s_h3 xor shift_right(s_h3, 6);
            s_h5 <= s_h4 + shift_left(s_h4, 3);
            s_h6 <= s_h5 xor shift_right(s_h5, 11);
            s_h7 <= s_h6 + shift_left(s_h6, 15);
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
        variable v_ys, v_us, v_vs : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- E7 (rd) / E8: fetch; black outside the line and on line 0;
            -- U/V swapped on rows whose parity differs from the capture row's
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
            -- E9..E11: fade toward grey (unity below 62.5% Snow)
            s_dy9 <= signed(resize(s_wet_y, 11)) - C_GREY;
            s_du9 <= signed(resize(s_wet_u, 11)) - 512;
            s_dv9 <= signed(resize(s_wet_v, 11)) - 512;
            s_py10 <= s_dy9 * signed('0' & s_fw);
            s_pu10 <= s_du9 * signed('0' & s_fw);
            s_pv10 <= s_dv9 * signed('0' & s_fw);
            s_yf(11) <= C_GREY + resize(shift_right(s_py10, 8), 12);
            s_uf(11) <= 512 + resize(shift_right(s_pu10, 8), 12);
            s_vf(11) <= 512 + resize(shift_right(s_pv10, 8), 12);
            for i in 12 to 14 loop
                s_yf(i) <= s_yf(i - 1); s_uf(i) <= s_uf(i - 1); s_vf(i) <= s_vf(i - 1);
            end loop;
            -- E14: noise (-128..127) x amplitude
            s_ny14 <= signed(s_h7(31 downto 24) xor x"80") * signed('0' & s_amp);
            s_nu14 <= signed(s_h7(23 downto 16) xor x"80") * signed('0' & s_amp);
            s_nv14 <= signed(s_h7(15 downto 8) xor x"80") * signed('0' & s_amp);
            -- E15: add
            s_y15 <= s_yf(14) + resize(shift_right(s_ny14, 7), 12);
            if s_cols = '1' then
                s_u15 <= s_uf(14) + resize(shift_right(s_nu14, 7), 12);
                s_v15 <= s_vf(14) + resize(shift_right(s_nv14, 7), 12);
            else
                s_u15 <= s_uf(14); s_v15 <= s_vf(14);
            end if;
            -- E16: lost tracking: luma halved + a line level, chroma / 4
            if s_lost = '1' then
                s_y16 <= shift_right(s_y15, 1) + signed(resize(s_lnz, 12));
                s_u16 <= shift_right(s_u15, 2) + 384;
                s_v16 <= shift_right(s_v15, 2) + 384;
            else
                s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            end if;
            -- E17: clamp (luma 64..940, chroma 64..960); neutral outside the
            -- avid (the smoothing's edges)
            if s_y16 < 64 then v_ys := to_signed(64, 12); elsif s_y16 > 940 then v_ys := to_signed(940, 12); else v_ys := s_y16; end if;
            if s_u16 < 64 then v_us := to_signed(64, 12); elsif s_u16 > 960 then v_us := to_signed(960, 12); else v_us := s_u16; end if;
            if s_v16 < 64 then v_vs := to_signed(64, 12); elsif s_v16 > 960 then v_vs := to_signed(960, 12); else v_vs := s_v16; end if;
            if s_avid_sr(15) = '1' then
                s_y17 <= unsigned(v_ys(9 downto 0)); s_u17 <= unsigned(v_us(9 downto 0)); s_v17 <= unsigned(v_vs(9 downto 0));
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- E18..E19: horizontal [1 2 1]/4 (g1 the centre)
            s_g1y <= s_y17; s_g2y <= s_g1y;
            s_g1u <= s_u17; s_g2u <= s_g1u;
            s_g1v <= s_v17; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y17, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u17, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v17, 12);
            s_y19 <= v_sy(11 downto 2);
            s_u19 <= v_su(11 downto 2);
            s_v19 <= v_sv(11 downto 2);
            -- E20: luma toward black, 64 + 13/16 (y - 64) (white ~775); gate
            v_d  := s_y19 - 64;
            v_yo := s_y19 - shift_right(v_d, 3) - shift_right(v_d, 4);
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y20 <= v_yo; s_u20 <= s_u19; s_v20 <= s_v19;
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

end architecture snowglobe;
