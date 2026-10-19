-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Globeswap v0.2.0: catalogue combo #134 (seed 20260927) -- polar warp
-- (spherize / tunnel) + colour channel swap / offset + solarization
-- (Sabattier).  Two knobs each; the switches and the slider decide how they
-- combine.
--
--   K1 Lens     lens strength, bipolar (tunnel .. flat .. sphere); about 75%
--               wraps the picture exactly onto the disc
--   K2 Size     lens radius, 3 .. 146% of the half-height
--   K3 Swap     channel mode: None, U<>V, Y>U, U>Y, Y<>V, Rotate, -Y, -YUV
--               (labelled, with hysteresis at the zone edges)
--   K4 Offset   per-channel column offset, bipolar (+-256 px chromatic split
--               at HD, half that in SD)
--   K5 Solar    solarization: 0% = off, 100% = everything above mid-grey
--               folds (threshold 960 .. 386)
--   K6 Fold     fold gain 0.125x .. 7.5x (log, whole knob); >1x repeats into
--               bands
--
--   S7  Fringe  even split across the picture / split grows toward the lens rim
--   S8  Swap    channel swap everywhere / only inside the solar region
--   S9  Split   U and V pulled apart, luma still / luma pulled against both chroma
--   S10 Lens    row lens across the whole width / lens only inside the disc (the dry picture outside)
--   S11 Fold    solar region folds luma only / also inverts its chroma
--   P12 Globe   climax: grows the lens (sphere) and its radius, spreads the
--               channels, drives the fold (up to 16x)
--
-- Lens, Size, Offset and Globe are latched per frame and glided (ease 1/8 a
-- frame), so pot noise can't shimmer the squeezed rows.
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.  The DDA runs at 24 bits and the
-- source column saturates, so squeezed rows never wrap.
-- Per-row lens scale from a 64-entry sphere / tunnel table (per-line
-- sequencer), DDA source column plus a per-channel offset, three edge-clamped
-- addresses into a dual-bank line buffer (Y, U, V read from different
-- columns); a 32-deep dry ring supplies the unlensed pixel outside the disc;
-- solar_fold on the fetched pixel, then the swap.  Rows whose hsync->avid
-- parity differs from the stored line's swap U/V (addresses and values,
-- drape v1.3).  Output: luma 64..940, [1 2 1] on Y/U/V + luma scaled to white
-- ~775 (the HD HDMI bar fix), blanking-gated.  Latency 20 clocks, all modes.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The lens runs in virtual
-- 1080-line units stepped from the video timing ID (registers_in(8)): y
-- 1080 / active lines a field line (interlace included), x 1.0 / 1.5 / 2.0
-- units a pixel (1080 / 720p / SD).  The radius is a fraction of the
-- virtual height and the Disc half-width converts it back to pixels
-- (x 1 / 2/3 / 1/2), so the disc is round on a 4:3 SD screen (it was 2.2x
-- taller than wide) and at 720p.  The channel Offset is x 2/3 at 720p (it
-- was the HD size there) and x 1/2 in SD as before.  1080p is
-- bit-identical.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture globeswap of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LAT : integer := 20;

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
    -- circle half-width per ring, Q8: 256 * sqrt(1 - (t/64)^2); also the
    -- tunnel scale (the same curve)
    constant C_CIRC : t_tab := (256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 244, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219, 217, 214, 212, 209, 206, 203, 200, 197, 193, 190, 186, 182, 178, 174, 169, 165, 160, 155, 149, 143, 137, 131, 124, 116, 108, 99, 89, 77, 63, 45);

    -- fold gain, Q8: 256 * 2^((i - 32) * 3/32) -- K6 alone spans 0..63
    -- (0.125x .. 7.5x), P12 pushes on to 16x
    type t_gain_rom is array (0 to 127) of integer range 0 to 4095;
    constant C_GAIN_ROM : t_gain_rom := (
        32, 34, 36, 39, 41, 44, 47, 50,
        54, 57, 61, 65, 70, 74, 79, 85,
        91, 97, 103, 110, 117, 125, 134, 143,
        152, 162, 173, 185, 197, 211, 225, 240,
        256, 273, 292, 311, 332, 354, 378, 403,
        431, 459, 490, 523, 558, 596, 636, 679,
        724, 773, 825, 880, 939, 1002, 1069, 1141,
        1218, 1300, 1387, 1480, 1579, 1685, 1798, 1919,
        2048, 2186, 2332, 2489, 2656, 2834, 3025, 3228,
        3444, 3676, 3922, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 31) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal ring : t_ring := (others => x"04080200");
    -- a bank is never used while it is written, the ring never reads the
    -- slot it writes
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;
    attribute no_rw_check of ring : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_ring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');

    signal s_sw_fringe, s_sw_swapreg, s_sw_ysplit, s_sw_disc, s_sw_foldc : std_logic;

    -- per-frame control decode (vblank sequencer)
    signal s_gl     : unsigned(12 downto 0) := to_unsigned(6400, 13);   -- glided K1 x 8
    signal s_gs     : unsigned(12 downto 0) := to_unsigned(4800, 13);   -- glided K2 x 8
    signal s_go     : unsigned(12 downto 0) := to_unsigned(5120, 13);   -- glided K4 x 8
    signal s_gp     : unsigned(12 downto 0) := (others => '0');         -- glided P12 x 8
    signal s_el, s_es, s_eo, s_ep : signed(13 downto 0) := (others => '0');
    signal s_k5l    : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_k6l    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k3seen : std_logic := '0';
    signal s_kb     : signed(8 downto 0) := (others => '0');
    signal s_p12h   : unsigned(8 downto 0) := (others => '0');
    signal s_p12q   : unsigned(7 downto 0) := (others => '0');
    signal s_kbe    : signed(10 downto 0) := (others => '0');
    signal s_ksph   : std_logic := '1';
    signal s_kabs   : unsigned(9 downto 0) := (others => '0');
    signal s_rmk1, s_rmk2, s_rmp1, s_rmp2 : unsigned(10 downto 0) := (others => '0');
    signal s_rmk, s_rmp : unsigned(10 downto 0) := (others => '0');
    signal s_rm     : unsigned(10 downto 0) := to_unsigned(446, 11);
    signal s_rph    : unsigned(14 downto 0) := (others => '0');
    signal s_rpl    : unsigned(15 downto 0) := (others => '0');
    signal s_radw   : unsigned(20 downto 0) := (others => '0');
    signal s_rad_l  : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_hh     : unsigned(9 downto 0) := to_unsigned(135, 10);
    -- resolution scaling: steps (free-running from the timing ID), the
    -- line's virtual y (Q.8), the field's virtual height, and the radius
    -- in pixels for the disc half-width
    signal s_vxs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_vys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_vya    : unsigned(19 downto 0) := (others => '0');
    signal s_vht    : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_rx1    : unsigned(18 downto 0) := (others => '0');
    signal s_rx2    : unsigned(14 downto 0) := (others => '0');
    signal s_rad_x  : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_of1    : signed(18 downto 0) := (others => '0');
    signal s_of2    : signed(14 downto 0) := (others => '0');
    signal s_mode   : unsigned(2 downto 0) := "001";
    signal s_off    : signed(9 downto 0) := (others => '0');
    signal s_offe   : signed(10 downto 0) := (others => '0');
    signal s_offs   : signed(10 downto 0) := (others => '0');
    signal s_thr1   : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_thr    : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_base   : signed(11 downto 0) := to_signed(88, 12);
    signal s_gsum   : unsigned(6 downto 0) := to_unsigned(32, 7);
    signal s_gain_q : unsigned(11 downto 0) := to_unsigned(256, 12);   -- ROM out (absorbed into the EBR)
    signal s_gain   : unsigned(11 downto 0) := to_unsigned(256, 12);   -- fabric copy for the fold
    signal s_vstep  : unsigned(5 downto 0) := (others => '1');
    signal s_dq     : unsigned(16 downto 0) := (others => '0');
    signal s_dr     : unsigned(11 downto 0) := (others => '0');
    signal s_dbit   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_r  : unsigned(11 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';
    -- per line
    signal s_lstep  : unsigned(3 downto 0) := (others => '1');
    signal s_ady    : unsigned(10 downto 0) := (others => '0');
    signal s_beyond : std_logic := '0';
    signal s_tph, s_tpl : unsigned(16 downto 0) := (others => '0');
    signal s_tp     : unsigned(22 downto 0) := (others => '0');
    signal s_hwph   : unsigned(14 downto 0) := (others => '0');
    signal s_hwpl   : unsigned(15 downto 0) := (others => '0');
    signal s_fkh, s_fkl : signed(17 downto 0) := (others => '0');
    signal s_cxsh   : signed(17 downto 0) := (others => '0');
    signal s_cxsl   : signed(18 downto 0) := (others => '0');
    signal s_offph, s_offpl : signed(16 downto 0) := (others => '0');
    signal s_t6     : unsigned(5 downto 0) := (others => '0');
    signal s_f      : unsigned(10 downto 0) := (others => '0');
    signal s_circ   : unsigned(8 downto 0) := (others => '0');
    signal s_fm1    : signed(11 downto 0) := (others => '0');
    signal s_hwp    : unsigned(19 downto 0) := (others => '0');
    signal s_fk     : signed(22 downto 0) := (others => '0');
    signal s_srel   : signed(11 downto 0) := to_signed(256, 12);
    signal s_hw     : unsigned(10 downto 0) := (others => '0');
    signal s_srow   : signed(11 downto 0) := to_signed(256, 12);
    signal s_dl, s_dr_b : signed(11 downto 0) := (others => '0');
    signal s_dev    : unsigned(11 downto 0) := (others => '0');
    signal s_gfac   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_cxs    : signed(23 downto 0) := (others => '0');
    signal s_src0   : signed(23 downto 0) := (others => '0');
    signal s_offp   : signed(21 downto 0) := (others => '0');
    signal s_offrow : signed(9 downto 0) := (others => '0');
    -- hsync->avid parity: this line's vs the stored (previous) line's
    signal s_hpar, s_lpar, s_swap : std_logic := '0';
    signal s_offu   : signed(9 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

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

    type t_x is array (1 to 5) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 5) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc  : signed(23 downto 0) := (others => '0');
    signal s_src4 : signed(11 downto 0) := (others => '0');
    signal s_uy5, s_uu5, s_uv5 : signed(12 downto 0) := (others => '0');
    signal s_ge5, s_lt5 : std_logic := '0';
    signal s_ay6, s_au6, s_av6 : unsigned(10 downto 0) := (others => '0');
    signal s_disc6, s_disc7, s_disc8 : std_logic := '0';
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_ys, s_us, s_vs, s_ysol : unsigned(9 downto 0) := (others => '0');
    signal s_regs : std_logic := '0';
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

    s_sw_fringe  <= registers_in(6)(0);
    s_sw_swapreg <= registers_in(6)(1);
    s_sw_ysplit  <= registers_in(6)(2);
    s_sw_disc    <= registers_in(6)(3);
    s_sw_foldc   <= registers_in(6)(4);

    -- dry ring: data_in delayed 8 (q aligned with the fetched pixel)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - 7));
            ring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    -- solarization on the fetched pixel (stages 9 -> 16)
    solar : entity work.solar_fold
        port map (clk => clk, y_in => s_y9, u_in => s_u9, v_in => s_v9,
                  thr => s_thr, gain => s_gain, base => s_base, fold_c => s_sw_foldc,
                  enable => '1', y_out => s_ys, u_out => s_us, v_out => s_vs,
                  ysol => s_ysol, region => s_regs);

    p_ctrl : process(clk)
        variable v_r   : unsigned(12 downto 0);
        variable v_s   : signed(11 downto 0);
        variable v_o   : signed(13 downto 0);
        variable v_g   : signed(13 downto 0);
        variable v_k3  : unsigned(9 downto 0);
        variable v_c   : unsigned(8 downto 0);
        variable v_rad : unsigned(11 downto 0);
        variable v_sw  : std_logic;
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            -- hsync->avid parity (drape): clock parity since hsync, per line
            s_hpar <= not s_hpar;
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_vys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- per-frame sequencer, one small op per state, in vblank
            case to_integer(s_vstep) is
                when 0 =>
                    s_el <= signed(shift_left(resize(unsigned(registers_in(0)), 14), 3)) - signed(resize(s_gl, 14));
                    s_es <= signed(shift_left(resize(unsigned(registers_in(1)), 14), 3)) - signed(resize(s_gs, 14));
                    s_eo <= signed(shift_left(resize(unsigned(registers_in(3)), 14), 3)) - signed(resize(s_go, 14));
                    s_ep <= signed(shift_left(resize(unsigned(registers_in(7)), 14), 3)) - signed(resize(s_gp, 14));
                    s_k5l <= unsigned(registers_in(4));
                    s_k6l <= unsigned(registers_in(5));
                    -- K3: a label index right after load (the firmware writes
                    -- the default's index), then 8 zones with a 8-count
                    -- hysteresis band at each inner edge
                    v_k3 := unsigned(registers_in(2));
                    if s_k3seen = '0' and v_k3 < 8 then
                        s_mode <= v_k3(2 downto 0);
                    elsif (v_k3(6 downto 0) >= 8 or v_k3(9 downto 7) = 0) and
                          (v_k3(6 downto 0) < 120 or v_k3(9 downto 7) = 7) then
                        s_mode <= v_k3(9 downto 7);
                    end if;
                    if v_k3 >= 8 then s_k3seen <= '1'; end if;
                    s_hh <= s_vht(10 downto 1);
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_vht(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- glides: ease by diff/8 a frame
                    v_g := signed('0' & s_gl) + shift_right(s_el, 3); s_gl <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_gs) + shift_right(s_es, 3); s_gs <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_go) + shift_right(s_eo, 3); s_go <= unsigned(v_g(12 downto 0));
                    v_g := signed('0' & s_gp) + shift_right(s_ep, 3); s_gp <= unsigned(v_g(12 downto 0));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_kb <= resize(shift_right(signed(resize(s_gl(12 downto 3), 11)) - 512, 1), 9);
                    s_p12h <= s_gp(12 downto 4);
                    s_p12q <= s_gp(12 downto 5);
                    -- radius in 1/512 of the half-height: 15 + 0.72 K2 + 0.47 P12
                    s_rmk1 <= resize(s_gs(12 downto 4), 11) + resize(s_gs(12 downto 6), 11);
                    s_rmk2 <= resize(s_gs(12 downto 7), 11) + resize(s_gs(12 downto 8), 11);
                    s_rmp1 <= resize(s_gp(12 downto 5), 11) + resize(s_gp(12 downto 6), 11);
                    s_rmp2 <= resize(s_gp(12 downto 7), 11) + resize(s_gp(12 downto 8), 11);
                    s_off <= resize(shift_right(signed(resize(s_go(12 downto 3), 11)) - 512, 1), 10);
                    -- threshold 960 - 0.5625 K5: 0% = off, 100% = 386
                    s_thr1 <= to_unsigned(960, 10) - resize(s_k5l(9 downto 1), 10);
                    s_gsum <= resize(s_k6l(9 downto 4), 7) + resize(s_gp(12 downto 9), 7);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_kbe <= resize(s_kb, 11) + signed(resize(s_p12h, 11));
                    s_rmk <= s_rmk1 + s_rmk2;
                    s_rmp <= s_rmp1 + s_rmp2 + 15;
                    s_offe <= resize(s_off, 11) + signed(resize(s_p12q, 11));
                    s_thr <= s_thr1 - resize(s_k5l(9 downto 4), 10);
                    s_gain_q <= to_unsigned(C_GAIN_ROM(to_integer(s_gsum)), 12);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    if s_kbe >= 0 then s_ksph <= '1'; s_kabs <= unsigned(s_kbe(9 downto 0));
                    else s_ksph <= '0'; s_kabs <= unsigned(resize(-s_kbe, 10)); end if;
                    s_rm <= s_rmk + s_rmp;
                    -- the split in px (x 171/256 at 720p, below): offe x 160, x 11
                    s_of1 <= shift_left(resize(s_offe, 19), 7) + shift_left(resize(s_offe, 19), 5);
                    s_of2 <= shift_left(resize(s_offe, 15), 3) + shift_left(resize(s_offe, 15), 1) + resize(s_offe, 15);
                    s_base <= signed(resize(s_thr, 12)) - 512;
                    s_gain <= s_gain_q;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    -- split x 1 (1080), x 2/3 (720p), x 1/2 (SD; the line is
                    -- 1440 virtual units)
                    case to_integer(s_vxs) is
                        when 3 => s_offs <= resize(shift_right(s_of1 + resize(s_of2, 19), 8), 11);
                        when 4 => s_offs <= shift_right(s_offe, 1);
                        when others => s_offs <= s_offe;
                    end case;
                    s_rph <= s_hh * s_rm(10 downto 6);
                    s_rpl <= s_hh * s_rm(5 downto 0);
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_radw <= shift_left(resize(s_rph, 21), 6) + resize(s_rpl, 21);
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    v_rad := s_radw(20 downto 9);
                    if v_rad > 2047 then s_rad_l <= (others => '1');
                    elsif v_rad < 16 then s_rad_l <= to_unsigned(16, 11);
                    else s_rad_l <= v_rad(10 downto 0); end if;
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(16, 5);
                    s_vstep <= s_vstep + 1;
                when 8 to 24 =>
                    -- 2^16 / radius, restoring, one bit a state
                    if s_dbit = 16 then v_r := (s_dr & '1'); else v_r := (s_dr & '0'); end if;
                    if v_r >= resize(s_rad_l, 13) then
                        s_dr <= resize(v_r - resize(s_rad_l, 13), 12);
                        s_dq <= s_dq(15 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0);
                        s_dq <= s_dq(15 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    -- beside the divide: the radius in px for the disc
                    -- half-width (x 1 / 171/256 / 1/2): rad x 160, x 11
                    if s_vstep = 8 then
                        s_rx1 <= shift_left(resize(s_rad_l, 19), 7) + shift_left(resize(s_rad_l, 19), 5);
                        s_rx2 <= shift_left(resize(s_rad_l, 15), 3) + shift_left(resize(s_rad_l, 15), 1) + resize(s_rad_l, 15);
                    end if;
                    if s_vstep = 9 then
                        case to_integer(s_vxs) is
                            when 3 => s_rad_x <= resize(shift_right(s_rx1 + resize(s_rx2, 19), 8), 11);
                            when 4 => s_rad_x <= shift_right(s_rad_l, 1);
                            when others => s_rad_x <= s_rad_l;
                        end case;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 25 =>
                    if s_dq(16 downto 12) /= 0 then s_inv_r <= (others => '1'); else s_inv_r <= s_dq(11 downto 0); end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- per-line sequencer: every multiply is split hi/lo over two
            -- steps (the line has 100+ blanking clocks to spare)
            case to_integer(s_lstep) is
                when 0 =>
                    if s_vya(18 downto 8) >= s_cy then s_ady <= s_vya(18 downto 8) - s_cy; else s_ady <= s_cy - s_vya(18 downto 8); end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_tph <= s_ady * s_inv_r(11 downto 6);
                    s_tpl <= s_ady * s_inv_r(5 downto 0);
                    if s_ady >= s_rad_l then s_beyond <= '1'; else s_beyond <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_tp <= shift_left(resize(s_tph, 23), 6) + resize(s_tpl, 23);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_beyond = '1' then s_t6 <= (others => '0'); else s_t6 <= s_tp(15 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    v_c := to_unsigned(C_CIRC(to_integer(s_t6)), 9);
                    if s_ksph = '1' then s_f <= to_unsigned(C_SPH(to_integer(s_t6)), 11); else s_f <= resize(v_c, 11); end if;
                    if s_beyond = '1' then s_circ <= (others => '0'); else s_circ <= v_c; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_beyond = '1' then s_fm1 <= (others => '0'); else s_fm1 <= signed(resize(s_f, 12)) - 256; end if;
                    s_hwph <= s_rad_x * s_circ(8 downto 5);
                    s_hwpl <= s_rad_x * s_circ(4 downto 0);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_fkh <= s_fm1 * signed('0' & s_kabs(9 downto 5));
                    s_fkl <= s_fm1 * signed('0' & s_kabs(4 downto 0));
                    s_hwp <= shift_left(resize(s_hwph, 20), 5) + resize(s_hwpl, 20);
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_fk <= shift_left(resize(s_fkh, 23), 5) + resize(s_fkl, 23);
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    v_s := to_signed(256, 12) + resize(shift_right(s_fk, 7), 12);
                    if s_fk >= 229376 then v_s := to_signed(2047, 12); elsif v_s < 32 then v_s := to_signed(32, 12); end if;
                    s_srel <= v_s;
                    s_hw <= s_hwp(18 downto 8);
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    s_srow <= s_srel;
                    s_dl <= signed(resize(s_cx, 12)) - signed(resize(s_hw, 12));
                    s_dr_b <= signed(resize(s_cx, 12)) + signed(resize(s_hw, 12));
                    -- lens deviation of this row, |scale - 1|
                    if s_srel >= 256 then s_dev <= unsigned(s_srel - 256); else s_dev <= unsigned(to_signed(256, 12) - s_srel); end if;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_cxsh <= s_srow * signed('0' & s_cx(10 downto 6));
                    s_cxsl <= s_srow * signed('0' & s_cx(5 downto 0));
                    if s_sw_fringe = '0' then s_gfac <= to_unsigned(256, 10);
                    elsif s_dev > 767 then s_gfac <= to_unsigned(1023, 10);
                    else s_gfac <= to_unsigned(256, 10) + s_dev(9 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 11 =>
                    s_cxs <= shift_left(resize(s_cxsh, 24), 6) + resize(s_cxsl, 24);
                    s_offph <= s_offs * signed('0' & s_gfac(9 downto 5));
                    s_offpl <= s_offs * signed('0' & s_gfac(4 downto 0));
                    s_lstep <= s_lstep + 1;
                when 12 =>
                    -- DDA start (Q8, 24 bits): cx - (cx + 1) * scale
                    s_src0 <= shift_left(signed(resize(s_cx, 24)), 8) - s_cxs - resize(s_srow, 24);
                    s_offp <= shift_left(resize(s_offph, 22), 5) + resize(s_offpl, 22);
                    s_lstep <= s_lstep + 1;
                when 13 =>
                    v_o := resize(shift_right(s_offp, 8), 14);
                    if v_o > 511 then v_o := to_signed(511, 14); elsif v_o < -511 then v_o := to_signed(-511, 14); end if;
                    s_offrow <= resize(v_o, 10);
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                s_vya <= s_vya + s_vys;
                s_vht <= resize(shift_right(s_vya + s_vys, 8), 11);
            end if;
            -- start of a line: U/V swapped (addresses and values) when this
            -- line's parity differs from the stored line's
            if s_prev_avid = '0' and data_in.avid = '1' then
                v_sw := s_hpar xor s_lpar;
                s_swap <= v_sw;
                s_lpar <= s_hpar;
                if v_sw = '1' and s_sw_ysplit = '0' then s_offu <= -s_offrow; else s_offu <= s_offrow; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0');
                s_hpar <= '0';
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_vya <= (others => '0'); s_wpar <= '0';
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
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 5 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: DDA
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 24); end if;
            -- 4: source x, saturated to +-2047 (the edge clamp fills beyond)
            if s_acc(23 downto 19) = "00000" or s_acc(23 downto 19) = "11111" then
                s_src4 <= s_acc(19 downto 8);
            elsif s_acc(23) = '0' then
                s_src4 <= to_signed(2047, 12);
            else
                s_src4 <= to_signed(-2048, 12);
            end if;
            -- 5: per-channel source columns; disc bounds test on the screen column
            if s_sw_ysplit = '1' then s_uy5 <= resize(s_src4, 13) - resize(s_offu, 13); else s_uy5 <= resize(s_src4, 13); end if;
            s_uu5 <= resize(s_src4, 13) + resize(s_offu, 13);
            if s_sw_ysplit = '1' then s_uv5 <= resize(s_src4, 13) + resize(s_offu, 13); else s_uv5 <= resize(s_src4, 13) - resize(s_offu, 13); end if;
            if signed(resize(s_x(3), 12)) >= s_dl then s_ge5 <= '1'; else s_ge5 <= '0'; end if;
            if signed(resize(s_x(3), 12)) < s_dr_b then s_lt5 <= '1'; else s_lt5 <= '0'; end if;
            -- 6: edge clamp per channel, disc flag
            s_ay6 <= f_edge_addr(s_uy5, s_w_edge);
            s_au6 <= f_edge_addr(s_uu5, s_w_edge);
            s_av6 <= f_edge_addr(s_uv5, s_w_edge);
            s_disc6 <= s_ge5 and s_lt5;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ay6));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ay6));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_au6(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_au6(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_av6(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_av6(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_outside : std_logic;
        variable v_m : unsigned(2 downto 0);
        variable v_u, v_v : unsigned(9 downto 0);
        variable v_y17, v_u17, v_v17 : unsigned(9 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 7 (rd) / 8: fetch; U/V swapped on rows whose parity differs from
            -- the stored line's (the addresses were swapped at stage 5)
            s_a_rbank <= s_rbank; s_disc7 <= s_disc6; s_disc8 <= s_disc7;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); v_u := unsigned(s_rdU1) & "00"; v_v := unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); v_u := unsigned(s_rdU0) & "00"; v_v := unsigned(s_rdV0) & "00";
            end if;
            if s_swap = '1' then s_wet_u <= v_v; s_wet_v <= v_u; else s_wet_u <= v_u; s_wet_v <= v_v; end if;
            -- 9: lensed / dry pick (dry outside the disc in Disc mode) -> solar_fold (9 -> 16)
            v_outside := s_sw_disc and not s_disc8;
            if v_outside = '1' then
                s_y9 <= unsigned(s_ring_q(29 downto 20)); s_u9 <= unsigned(s_ring_q(19 downto 10)); s_v9 <= unsigned(s_ring_q(9 downto 0));
            else
                s_y9 <= s_wet_y; s_u9 <= s_wet_u; s_v9 <= s_wet_v;
            end if;
            -- 17: channel swap (everywhere, or only inside the solar region);
            -- luma 64..940; neutral outside the avid (the smoothing's edges)
            if s_sw_swapreg = '1' and s_regs = '0' then v_m := "000"; else v_m := s_mode; end if;
            case to_integer(v_m) is
                when 0 => v_y17 := s_ys; v_u17 := s_us; v_v17 := s_vs;
                when 1 => v_y17 := s_ys; v_u17 := s_vs; v_v17 := s_us;
                when 2 => v_y17 := s_ys; v_u17 := s_ys; v_v17 := s_vs;
                when 3 => v_y17 := s_us; v_u17 := s_us; v_v17 := s_vs;
                when 4 => v_y17 := s_vs; v_u17 := s_us; v_v17 := s_ys;
                when 5 => v_y17 := s_us; v_u17 := s_vs; v_v17 := s_ys;
                when 6 => v_y17 := not s_ys; v_u17 := s_us; v_v17 := s_vs;
                when others => v_y17 := not s_ys; v_u17 := not s_us; v_v17 := not s_vs;
            end case;
            if s_avid_sr(15) = '1' then
                if v_y17 < 64 then s_y17 <= to_unsigned(64, 10);
                elsif v_y17 > 940 then s_y17 <= to_unsigned(940, 10);
                else s_y17 <= v_y17; end if;
                s_u17 <= v_u17; s_v17 <= v_v17;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- 18..19: horizontal [1 2 1]/4 (g1 the centre)
            s_g1y <= s_y17; s_g2y <= s_g1y;
            s_g1u <= s_u17; s_g2u <= s_g1u;
            s_g1v <= s_v17; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y17, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u17, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v17, 12);
            s_y19 <= v_sy(11 downto 2);
            s_u19 <= v_su(11 downto 2);
            s_v19 <= v_sv(11 downto 2);
            -- 20: luma toward black, 64 + 13/16 (y - 64) (white ~775); gate
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

end architecture globeswap;
