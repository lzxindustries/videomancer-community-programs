-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Pressroom: catalogue combo #40 (seed 20260919) -- ASCII / dither-art /
-- halftone + head-switch noise band + chroma / luma / difference key.  Two
-- knobs each; the switches and the slider decide how the three combine.
-- v0.2.0 = finalizing pass (see SUMMARY.md for the review and what changed
-- from v0.1.0).
-- v0.2.2 (2026-10-17) = resolution scaling: cells and the band tear run in
-- virtual 1080-line units (x steps 1 / 1.5 / 2 per pixel, y steps per field
-- line from the timing ID), so 720p and SD show the 1080p print scaled to
-- the screen height (SD = a 1440-wide 4:3 window; cell rows there are
-- whole lines, 4 units a field line, so the dots stay regular and ~11%
-- tall; on 480i/576i the 4-px cell is floored to 8, as it is under one
-- field line); the band's 4-line floor scales too.  1080p is bit-identical.
--
--   K1 Cell      halftone cell 4 / 8 / 16 / 32 px
--   K2 Screen    Dot / Line / Diamond / Square
--   K3 Band Size head-switch band height: 4 lines .. half the frame
--   K4 Noise     band noise, tear and break-up (never fully off)
--   K5 Level     key threshold (luma 72..903, saturation 8..519)
--   K6 Soft      key edge dithered by noise
--
--   S7  Key      from luma / from saturation
--   S8  Halftone inside the key / outside it (the rest is a colour cell
--                mosaic)
--   S9  Ink      dark ink on white paper / light ink on black paper (both
--                positive: dots grow with darkness / with brightness)
--   S10 Band     at the bottom / rolling up the frame (wraps round)
--   S11 Band FX  noisy picture / the whole band prints as a broken screen
--   P12 Ink      how much ink, glided: x0.28 at 0 (a light print, dots in
--                the shadows only), x1.17 at 50% (black prints solid, white
--                clean), x4 at 100% (the ink floods; only highlights open)
--
-- Cell average = running box sum (EBR ring, zero outside active video)
-- into a two-bank line buffer; address a holds the mean of pixels
-- a-N+1..a, a cell reads its last pixel.  One line per cell row is
-- captured (the last line of the row above; line 0 for the top row).
-- The halftone threshold comes from a 4 x 16 x 16 screen ROM (EBR) indexed
-- by the position inside the cell.  The key is taken per pixel from the
-- dry input and picks halftone or the plain cell colour.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (17) in all modes; U/V
-- swapped on rows whose hsync->avid parity differs from their capture
-- row's (drape v1.3: a stored line replayed on other rows); output
-- conditioning ([1 2 1] on Y/U/V + luma scaled to white ~777: the HD HDMI
-- bar fix from damier v0.4.4 -- halftone dots are full-swing pixel edges
-- and the band moves them every frame); labelled rotaries honour the
-- firmware's label-INDEX default until the knob first reads a position;
-- no_rw_check on every RAM (none reads the address it writes).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture pressroom of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (17 + 3 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 17;

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
    -- cell-row y step in Q4.8 virtual units per FIELD line: 1080 / active
    -- lines (doubled for interlaced modes), except that SD is snapped to a
    -- power of 2 (4.0 interlaced, 2.0 progressive; exact 4.44 / 3.75 /
    -- 2.25 / 1.875), so every cell row is a whole number of lines and the
    -- dots stay regular (4.44 put 1.8 lines in an 8-unit cell and the rows
    -- beat into streaks); SD cells are ~11% taller than exact
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8      => return to_unsigned(1024, 12);  -- 480i / 576i: 4.0
            when 4 | 12     => return to_unsigned(512, 12);   -- 480p / 576p: 2.0
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p: 1.5
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i: 2.0
            when others     => return to_unsigned(256, 12);   -- 1080p: 1.0
        end case;
    end function;
    -- the band's 4-line floor in field lines (4 / y step, at least 1)
    function f_bfloor(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8           => return to_unsigned(1, 3);
            when 1 | 2 | 10 | 4 | 12 => return to_unsigned(2, 3);
            when 5 | 6 | 14      => return to_unsigned(3, 3);
            when others          => return to_unsigned(4, 3);
        end case;
    end function;
    -- floor(r / 3), r = 0..63 (720p: real pixels left in a cell from the
    -- half units left)
    type t_div3 is array (0 to 63) of unsigned(4 downto 0);
    function f_mk_div3 return t_div3 is
        variable t : t_div3;
    begin
        for i in 0 to 63 loop
            t(i) := to_unsigned(i / 3, 5);
        end loop;
        return t;
    end function;
    constant C_DIV3 : t_div3 := f_mk_div3;

    type t_screen is array (0 to 1023) of integer range 0 to 255;
    constant C_SCREEN : t_screen := (
        16, 31, 45, 58, 68, 77, 83, 86, 86, 83, 77, 68, 58, 45, 31, 16,
        31, 48, 63, 77, 89, 98, 105, 108, 108, 105, 98, 89, 77, 63, 48, 31,
        45, 63, 80, 95, 108, 119, 127, 131, 131, 127, 119, 108, 95, 80, 63, 45,
        58, 77, 95, 112, 127, 139, 148, 153, 153, 148, 139, 127, 112, 95, 77, 58,
        68, 89, 108, 127, 143, 158, 169, 175, 175, 169, 158, 143, 127, 108, 89, 68,
        77, 98, 119, 139, 158, 175, 189, 198, 198, 189, 175, 158, 139, 119, 98, 77,
        83, 105, 127, 148, 169, 189, 207, 219, 219, 207, 189, 169, 148, 127, 105, 83,
        86, 108, 131, 153, 175, 198, 219, 239, 239, 219, 198, 175, 153, 131, 108, 86,
        86, 108, 131, 153, 175, 198, 219, 239, 239, 219, 198, 175, 153, 131, 108, 86,
        83, 105, 127, 148, 169, 189, 207, 219, 219, 207, 189, 169, 148, 127, 105, 83,
        77, 98, 119, 139, 158, 175, 189, 198, 198, 189, 175, 158, 139, 119, 98, 77,
        68, 89, 108, 127, 143, 158, 169, 175, 175, 169, 158, 143, 127, 108, 89, 68,
        58, 77, 95, 112, 127, 139, 148, 153, 153, 148, 139, 127, 112, 95, 77, 58,
        45, 63, 80, 95, 108, 119, 127, 131, 131, 127, 119, 108, 95, 80, 63, 45,
        31, 48, 63, 77, 89, 98, 105, 108, 108, 105, 98, 89, 77, 63, 48, 31,
        16, 31, 45, 58, 68, 77, 83, 86, 86, 83, 77, 68, 58, 45, 31, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
        80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
        112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112,
        143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143,
        175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175,
        207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207,
        239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239,
        239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239,
        207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207,
        175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175,
        143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143,
        112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112,
        80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
        48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 32, 48, 64, 80, 96, 112, 128, 128, 112, 96, 80, 64, 48, 32, 16,
        32, 48, 64, 80, 96, 112, 128, 143, 143, 128, 112, 96, 80, 64, 48, 32,
        48, 64, 80, 96, 112, 128, 143, 159, 159, 143, 128, 112, 96, 80, 64, 48,
        64, 80, 96, 112, 128, 143, 159, 175, 175, 159, 143, 128, 112, 96, 80, 64,
        80, 96, 112, 128, 143, 159, 175, 191, 191, 175, 159, 143, 128, 112, 96, 80,
        96, 112, 128, 143, 159, 175, 191, 207, 207, 191, 175, 159, 143, 128, 112, 96,
        112, 128, 143, 159, 175, 191, 207, 223, 223, 207, 191, 175, 159, 143, 128, 112,
        128, 143, 159, 175, 191, 207, 223, 239, 239, 223, 207, 191, 175, 159, 143, 128,
        128, 143, 159, 175, 191, 207, 223, 239, 239, 223, 207, 191, 175, 159, 143, 128,
        112, 128, 143, 159, 175, 191, 207, 223, 223, 207, 191, 175, 159, 143, 128, 112,
        96, 112, 128, 143, 159, 175, 191, 207, 207, 191, 175, 159, 143, 128, 112, 96,
        80, 96, 112, 128, 143, 159, 175, 191, 191, 175, 159, 143, 128, 112, 96, 80,
        64, 80, 96, 112, 128, 143, 159, 175, 175, 159, 143, 128, 112, 96, 80, 64,
        48, 64, 80, 96, 112, 128, 143, 159, 159, 143, 128, 112, 96, 80, 64, 48,
        32, 48, 64, 80, 96, 112, 128, 143, 143, 128, 112, 96, 80, 64, 48, 32,
        16, 32, 48, 64, 80, 96, 112, 128, 128, 112, 96, 80, 64, 48, 32, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 16,
        16, 48, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 48, 16,
        16, 48, 80, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 143, 143, 143, 143, 143, 143, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 175, 175, 175, 175, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 207, 207, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 239, 239, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 239, 239, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 207, 207, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 175, 175, 175, 175, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 143, 143, 143, 143, 143, 143, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 80, 48, 16,
        16, 48, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 48, 16,
        16, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_bring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal bring : t_bring := (others => (others => '0'));
    -- no RAM ever reads the address it writes: the ring reads N-1 back,
    -- and a line-buffer bank is read only while the other bank is written
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1, bring : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_bring_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bring_w : unsigned(5 downto 0) := (others => '0');
    signal s_scr_addr : unsigned(9 downto 0) := (others => '0');
    signal s_scr_q, s_scr_q2 : unsigned(7 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_cell, s_k_screen, s_k_band, s_k_noise, s_k_level, s_k_soft : unsigned(9 downto 0);
    signal s_sw_sat, s_sw_outside, s_sw_light, s_sw_roll, s_sw_break : std_logic;
    signal s_p_ink  : unsigned(9 downto 0);
    signal s_live1, s_live2 : std_logic := '0';
    signal s_nsh    : unsigned(2 downto 0) := to_unsigned(3, 3);       -- cell shift 2..5 (virtual)
    signal s_nshr   : unsigned(2 downto 0) := to_unsigned(3, 3);       -- box-window shift (real px)
    signal s_mask   : unsigned(10 downto 0) := to_unsigned(7, 11);     -- N - 1 (virtual)
    signal s_maskh  : unsigned(5 downto 0) := to_unsigned(15, 6);      -- 2N - 1 (half units)
    signal s_ndel   : unsigned(5 downto 0) := to_unsigned(7, 6);       -- window - 1 (real px)

    -- Resolution scaling: x in half virtual units (only the low 6 bits are
    -- ever needed: cells are powers of 2), y in Q.8 virtual units per field
    -- line (at 1080p the integer parts equal the old counters)
    signal s_xs     : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_bfl    : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_x720, s_xsd, s_sdi : std_logic := '0';   -- 720p, SD, interlaced SD
    signal s_xa, s_xa0 : unsigned(5 downto 0) := (others => '0');
    signal s_ya, s_yan : unsigned(19 downto 0) := (others => '0');
    signal s_rowend : std_logic := '0';
    signal s_ngt, s_ngh : unsigned(7 downto 0) := to_unsigned(24, 8);  -- tear gain (x 2/3 at 720p)
    signal s_off_h  : unsigned(5 downto 0) := (others => '0');        -- tear in half units, mod 64
    signal s_shape  : unsigned(1 downto 0) := (others => '0');
    signal s_ksat, s_outside, s_light : std_logic := '0';
    signal s_ng     : unsigned(7 downto 0) := to_unsigned(24, 8);      -- band noise 24..248
    signal s_soft   : unsigned(7 downto 0) := (others => '0');
    signal s_lv0, s_lvt : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_level, s_slevel : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_klev   : unsigned(9 downto 0) := to_unsigned(512, 10);    -- threshold of the selected key
    signal s_pd     : signed(14 downto 0) := (others => '0');
    signal s_pg     : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);  -- glided P12, 4 frac bits
    signal s_pe     : unsigned(10 downto 0) := to_unsigned(556, 11);
    signal s_gk     : unsigned(1 downto 0) := "10";                    -- ink octave: x 2^(gk-2)
    signal s_gf     : unsigned(7 downto 0) := to_unsigned(44, 8);      -- ink mantissa: (256 + gf)/256
    signal s_bk8    : unsigned(7 downto 0) := (others => '0');
    signal s_bph, s_bpl : unsigned(14 downto 0) := (others => '0');
    signal s_bs     : unsigned(18 downto 0) := (others => '0');
    signal s_bandh  : unsigned(10 downto 0) := to_unsigned(16, 11);
    signal s_bpos   : unsigned(14 downto 0) := (others => '0');        -- rolling band top, 1/16 lines
    signal s_bdec   : signed(15 downto 0) := (others => '0');
    signal s_bover  : std_logic := '0';
    signal s_btop   : unsigned(10 downto 0) := to_unsigned(254, 11);
    signal s_bbot, s_bwrap : unsigned(11 downto 0) := (others => '0');
    signal s_vstep  : unsigned(3 downto 0) := "1111";
    signal s_rnd    : unsigned(31 downto 0) := x"2545F491";

    -- tracking / per-line
    signal s_rx, s_line : unsigned(10 downto 0) := (others => '0');
    signal s_height : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_wm1    : unsigned(10 downto 0) := to_unsigned(719, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_capture, s_wpar, s_rbank : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_band_row, s_brk_row, s_nz_row : std_logic := '0';
    signal s_jp     : signed(18 downto 0) := (others => '0');
    signal s_off_row : signed(10 downto 0) := (others => '0');
    signal s_lo_row : signed(12 downto 0) := (others => '0');
    signal s_hi_row : signed(12 downto 0) := to_signed(720, 13);
    signal s_ny     : unsigned(3 downto 0) := (others => '0');
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';

    -- input side
    signal s_accy, s_accu, s_accv : unsigned(15 downto 0) := (others => '0');
    signal s_ay3, s_au3, s_av3 : unsigned(9 downto 0) := (others => '0');
    signal s_wx1, s_wx3 : unsigned(10 downto 0) := (others => '0');
    signal s_wp1, s_wc1, s_wp3, s_we3 : std_logic := '0';

    -- read side; the suffix is the offset (pixel p's x is in s_rx at -1,
    -- s_xt0 at 0, its output at 16)
    signal s_in_y   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_in_u, s_in_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_xt0    : signed(12 downto 0) := (others => '0');   -- torn real x
    signal s_ta0    : unsigned(5 downto 0) := (others => '0');
    signal s_xt1, s_rem1 : unsigned(10 downto 0) := (others => '0');
    signal s_d1     : unsigned(4 downto 0) := (others => '0');
    signal s_bg0, s_bg1, s_bg2 : std_logic := '0';
    signal s_xn1    : unsigned(5 downto 0) := (others => '0');
    signal s_dy1, s_dy2 : unsigned(9 downto 0) := (others => '0');
    signal s_au1, s_av1 : unsigned(9 downto 0) := (others => '0');
    signal s_addr2  : unsigned(10 downto 0) := (others => '0');
    signal s_nx2    : unsigned(3 downto 0) := (others => '0');
    signal s_sat2   : unsigned(10 downto 0) := (others => '0');
    signal s_nz2    : signed(17 downto 0) := (others => '0');
    signal s_kv3    : signed(12 downto 0) := (others => '0');
    signal s_kd     : std_logic_vector(4 to 10) := (others => '0');     -- key
    signal s_bfd    : std_logic_vector(1 to 10) := (others => '0');     -- Break band row
    signal s_nfd    : std_logic_vector(1 to 11) := (others => '0');     -- Noise band row
    signal s_a_bg3, s_a_rbank3 : std_logic := '0';
    signal s_wet_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d5     : signed(10 downto 0) := (others => '0');
    type t_c is array (5 to 10) of unsigned(9 downto 0);
    signal s_yf     : t_c := (others => to_unsigned(64, 10));
    signal s_uc, s_vc : t_c := (others => to_unsigned(512, 10));
    signal s_thr5, s_thr6 : unsigned(7 downto 0) := (others => '0');
    signal s_d6, s_d7 : unsigned(7 downto 0) := (others => '0');
    signal s_nb6    : signed(17 downto 0) := (others => '0');
    signal s_pm7    : unsigned(15 downto 0) := (others => '0');
    signal s_thr7   : signed(10 downto 0) := (others => '0');
    signal s_s8     : unsigned(16 downto 0) := (others => '0');
    signal s_ithr8, s_ithr9 : unsigned(7 downto 0) := (others => '0');
    signal s_lum9   : unsigned(7 downto 0) := (others => '0');
    signal s_ink10  : std_logic := '0';
    signal s_nz10   : signed(17 downto 0) := (others => '0');
    signal s_n11    : signed(10 downto 0) := (others => '0');
    signal s_y11    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ys12   : signed(11 downto 0) := to_signed(64, 12);
    signal s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y13    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_cell   <= unsigned(registers_in(0));
    s_k_screen <= unsigned(registers_in(1));
    s_k_band   <= unsigned(registers_in(2));
    s_k_noise  <= unsigned(registers_in(3));
    s_k_level  <= unsigned(registers_in(4));
    s_k_soft   <= unsigned(registers_in(5));
    s_sw_sat     <= registers_in(6)(0);
    s_sw_outside <= registers_in(6)(1);
    s_sw_light   <= registers_in(6)(2);
    s_sw_roll    <= registers_in(6)(3);
    s_sw_break   <= registers_in(6)(4);
    s_p_ink    <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_scr_q  <= to_unsigned(C_SCREEN(to_integer(s_scr_addr)), 8);
            s_scr_q2 <= s_scr_q;
        end if;
    end process;

    -- zeros outside active video, so the running sums never carry the
    -- blanking level (v0.1 subtracted N blanking samples from every sum:
    -- luma sat 64 low and chroma wrapped through code 0 = green)
    p_ring : process(clk)
        variable v_w : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                v_w := "00" & data_in.y & data_in.u & data_in.v;
            else
                v_w := (others => '0');
            end if;
            s_bring_w <= s_bring_w + 1;
            s_bring_q <= bring(to_integer(s_bring_w - s_ndel));
            bring(to_integer(s_bring_w)) <= v_w;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_i  : unsigned(1 downto 0);
        variable v_m  : unsigned(10 downto 0);
        variable v_e  : unsigned(9 downto 0);
        variable v_pg : signed(14 downto 0);
        variable v_o  : signed(10 downto 0);
        variable v_r  : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;

            -- xorshift32, one step a clock (v0.1's 1-bit LFSR made each
            -- 8-bit sample twice the last one: streaky, correlated noise)
            v_r := s_rnd xor shift_left(s_rnd, 13);
            v_r := v_r xor shift_right(v_r, 17);
            v_r := v_r xor shift_left(v_r, 5);
            s_rnd <= v_r;

            -- running sums (window N ending at data_in)
            v_oy := unsigned(s_bring_q(29 downto 20)); v_ou := unsigned(s_bring_q(19 downto 10)); v_ov := unsigned(s_bring_q(9 downto 0));
            if data_in.avid = '0' then
                s_accy <= (others => '0'); s_accu <= (others => '0'); s_accv <= (others => '0');
            else
                s_accy <= s_accy + resize(unsigned(data_in.y), 16) - resize(v_oy, 16);
                s_accu <= s_accu + resize(unsigned(data_in.u), 16) - resize(v_ou, 16);
                s_accv <= s_accv + resize(unsigned(data_in.v), 16) - resize(v_ov, 16);
            end if;
            -- averages; written at the pixel the window ends on
            s_ay3 <= resize(shift_right(s_accy, to_integer(s_nshr)), 10);
            s_au3 <= resize(shift_right(s_accu, to_integer(s_nshr)), 10);
            s_av3 <= resize(shift_right(s_accv, to_integer(s_nshr)), 10);

            -- Resolution scaling: steps decoded every clock (the timing ID
            -- is static while a mode runs); x in half units (only mod 64 is
            -- needed).  Pixels and lines sample their left / top edge, as
            -- 1080p always did: a 2-sample cell reads screen rows 0 and 8
            -- (a gap line and the dot), where centre samples (4, 12) both
            -- hit the dot and printed solid stripes.
            s_xs  <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys  <= f_vstep_y(registers_in(8)(3 downto 0));
            s_bfl <= f_bfloor(registers_in(8)(3 downto 0));
            if s_xs = 3 then s_x720 <= '1'; else s_x720 <= '0'; end if;
            if s_xs = 4 then s_xsd <= '1'; else s_xsd <= '0'; end if;
            if registers_in(8)(3 downto 0) = "0000" or registers_in(8)(3 downto 0) = "1000" then
                s_sdi <= '1';
            else
                s_sdi <= '0';
            end if;
            if data_in.avid = '1' then
                s_xa <= s_xa + s_xs;
            else
                s_xa <= (others => '0');
            end if;
            s_yan <= s_ya + s_ys;
            -- this line is the last of its cell row: the next line's cell
            -- row differs (always so when a cell is shorter than a line)
            if ((s_yan(18 downto 8) xor s_ya(18 downto 8)) and not s_mask) /= 0 then
                s_rowend <= '1';
            else
                s_rowend <= '0';
            end if;
            s_wx1 <= s_rx; s_wp1 <= s_wpar; s_wc1 <= data_in.avid and s_capture;
            s_wx3 <= s_wx1; s_wp3 <= s_wp1; s_we3 <= s_wc1;

            -- per-line: band flags, tear, screen row
            case to_integer(s_lstep) is
                when 0 =>
                    if (s_line >= s_btop and resize(s_line, 12) < s_bbot) or resize(s_line, 12) < s_bwrap then
                        s_band_row <= '1';
                    else
                        s_band_row <= '0';
                    end if;
                    -- screen row from the virtual line
                    case to_integer(s_nsh) is
                        when 2 => s_ny <= s_ya(9 downto 8) & "00";
                        when 3 => s_ny <= s_ya(10 downto 8) & "0";
                        when 4 => s_ny <= s_ya(11 downto 8);
                        when others => s_ny <= s_ya(12 downto 9);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_jp <= signed(s_rnd(31 downto 22)) * signed('0' & s_ngt);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    -- tear: up to +-ng/256 x 512 virtual units (1080: 512
                    -- px; 720p: gain x 2/3, 341 px; SD: / 512, 256 px)
                    if s_band_row = '0' then
                        v_o := (others => '0');
                    elsif s_xsd = '0' then
                        v_o := resize(shift_right(s_jp, 8), 11);
                    else
                        v_o := resize(shift_right(s_jp, 9), 11);
                    end if;
                    s_off_row <= v_o;
                    s_brk_row <= s_band_row and s_sw_break;
                    s_nz_row  <= s_band_row and not s_sw_break;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    -- the torn read stays on the line for x in [lo, hi)
                    s_lo_row <= -resize(s_off_row, 13);
                    s_hi_row <= signed(resize(s_line_width, 13)) - resize(s_off_row, 13);
                    -- the tear in half virtual units (x step x offset), mod
                    -- 64: all the cell maths needs
                    case to_integer(s_xs) is
                        when 3 => s_off_h <= unsigned(s_off_row(5 downto 0)) + unsigned(s_off_row(4 downto 0) & '0');
                        when 4 => s_off_h <= unsigned(s_off_row(3 downto 0) & "00");
                        when others => s_off_h <= unsigned(s_off_row(4 downto 0) & '0');
                    end case;
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    -- the firmware loads a labelled default as the label
                    -- INDEX; honour 0..3 as an index only until the knob
                    -- first reads a real position
                    if s_k_cell >= 4 then s_live1 <= '1'; end if;
                    if s_live1 = '0' and s_k_cell < 4 then v_i := s_k_cell(1 downto 0); else v_i := s_k_cell(9 downto 8); end if;
                    -- interlaced SD: cells at least 8 units (2 field lines);
                    -- a 4-unit cell is under one line and every line read
                    -- the same screen row (the halftone vanished)
                    if s_sdi = '1' and v_i = 0 then v_i := "01"; end if;
                    s_nsh <= resize(v_i, 3) + 2;
                    -- box window: the whole cell at 1080 and SD (N/2 px),
                    -- 3/4 of it at 720p (N/2 px of 2N/3)
                    if s_xs = 2 then s_nshr <= resize(v_i, 3) + 2; else s_nshr <= resize(v_i, 3) + 1; end if;
                    if s_k_screen >= 4 then s_live2 <= '1'; end if;
                    if s_live2 = '0' and s_k_screen < 4 then v_i := s_k_screen(1 downto 0); else v_i := s_k_screen(9 downto 8); end if;
                    s_shape <= v_i;
                    s_ksat <= s_sw_sat; s_outside <= s_sw_outside; s_light <= s_sw_light;
                    s_bk8  <= s_k_band(9 downto 2);
                    -- noise 24 + 7/8 K4: the band never goes quiet
                    s_ng   <= to_unsigned(24, 8) + s_k_noise(9 downto 2) - resize(s_k_noise(9 downto 5), 8);
                    s_lv0  <= s_k_level;
                    s_soft <= s_k_soft(9 downto 2);
                    s_pd   <= signed(shift_left(resize(s_p_ink, 15), 4)) - signed(resize(s_pg, 15));
                    -- the rolling band climbs height/32 sixteenths of a line
                    -- a frame (one pass in ~8.5 s in every mode)
                    s_bdec <= signed(resize(s_bpos, 16)) - signed(resize(s_height(10 downto 5), 16));
                    if s_bpos(14 downto 4) >= s_height then s_bover <= '1'; else s_bover <= '0'; end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    for i in 0 to 10 loop
                        if i < to_integer(s_nsh) then v_m(i) := '1'; else v_m(i) := '0'; end if;
                    end loop;
                    s_mask   <= v_m;
                    s_maskh  <= v_m(4 downto 0) & '1';
                    for i in 0 to 5 loop
                        if i < to_integer(s_nshr) then s_ndel(i) <= '1'; else s_ndel(i) <= '0'; end if;
                    end loop;
                    -- tear gain x 171/256 (~2/3) at 720p, so the tear spans
                    -- the same virtual width as at 1080p
                    s_ngh <= shift_right(s_ng, 1) + shift_right(s_ng, 3);
                    s_lvt <= s_lv0 - resize(s_lv0(9 downto 3), 10);
                    -- P12 glide, 1/8 per frame
                    v_pg := signed('0' & s_pg) + shift_right(s_pd, 3);
                    s_pg <= unsigned(v_pg(13 downto 0));
                    s_bph <= s_height * s_bk8(7 downto 4);
                    -- v0.1 wrapped below 0 back to 0 and parked the band at
                    -- line 2046 (Rolling showed no band); wrap to the bottom
                    if s_sw_roll = '1' then
                        if s_bover = '1' then
                            s_bpos <= (others => '0');
                        elsif s_bdec < 0 then
                            s_bpos <= unsigned(s_bdec(14 downto 0)) + shift_left(resize(s_height, 15), 4);
                        else
                            s_bpos <= unsigned(s_bdec(14 downto 0));
                        end if;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- luma threshold 72 + 13/16 K5 (72..903: v0.1's top and
                    -- bottom of the knob sat outside the picture's range);
                    -- saturation 8 + K5/2
                    s_level  <= to_unsigned(72, 10) + s_lvt - resize(s_lv0(9 downto 4), 10);
                    s_slevel <= to_unsigned(8, 10) + resize(s_lv0(9 downto 1), 10);
                    s_pe  <= resize(s_pg(13 downto 4), 11) + 44;
                    s_bpl <= s_height * s_bk8(3 downto 0);
                    if s_x720 = '1' then
                        s_ngt <= s_ngh + shift_right(s_ng, 5) + shift_right(s_ng, 7);
                    else
                        s_ngt <= s_ng;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    if s_ksat = '1' then s_klev <= s_slevel; else s_klev <= s_level; end if;
                    -- ink gain (256 + gf)/256 x 2^(gk-2) from P12 + 44, so
                    -- 50% is x1.17 (solid black covers every threshold)
                    if s_pe(10) = '1' then v_e := (others => '1'); else v_e := s_pe(9 downto 0); end if;
                    s_gk <= v_e(9 downto 8); s_gf <= v_e(7 downto 0);
                    s_bs <= shift_left(resize(s_bph, 19), 4) + resize(s_bpl, 19);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    -- band 4 + height x K3/2048 lines (4 .. half the frame)
                    -- (the 4-line floor is 4 lines at 1080: s_bfl field lines)
                    s_bandh <= resize(s_bs(18 downto 9), 11) + s_bfl;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    if s_sw_roll = '1' then s_btop <= s_bpos(14 downto 4); else s_btop <= s_height - s_bandh; end if;
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_bbot <= resize(s_btop, 12) + resize(s_bandh, 12);
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    -- a rolling band running off the bottom wraps to the top
                    if s_bbot > resize(s_height, 12) then
                        s_bwrap <= s_bbot - resize(s_height, 12);
                    else
                        s_bwrap <= (others => '0');
                    end if;
                    s_vstep <= "1111";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_wm1 <= s_rx - 1; s_line <= s_line + 1; s_height <= s_line + 1;
                s_ya <= s_yan;
            end if;
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_line_par <= s_hpar;
                if s_capture = '1' then
                    if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
                end if;
            end if;
            if s_rbank = '1' then
                s_rowswap <= s_line_par xor s_cap_par1;
            else
                s_rowswap <= s_line_par xor s_cap_par0;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                -- capture the last line of each cell row (shown on the
                -- next row) and line 0 (shown on the rest of the top row)
                if s_rowend = '1' or s_line = 0 then
                    s_capture <= '1'; s_wpar <= not s_wpar; s_rbank <= s_wpar;
                else
                    s_capture <= '0'; s_rbank <= s_wpar;
                end if;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                s_ya <= (others => '0');
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
    -- Read address, screen index and key (offsets 0..4)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_r : unsigned(5 downto 0);
        variable v_k : std_logic;
    begin
        if rising_edge(clk) then
            -- 0: band tear (real px) and the torn pixel's position in half
            -- virtual units; off-line test against the per-line range
            -- (parallel to the add); the dry pixel the key is taken from
            -- (two FFs deep)
            s_xt0 <= signed(resize(s_rx, 13)) + resize(s_off_row, 13);
            s_ta0 <= s_xa + s_off_h;
            if signed(resize(s_rx, 13)) < s_lo_row or signed(resize(s_rx, 13)) >= s_hi_row then
                s_bg0 <= '1';
            else
                s_bg0 <= '0';
            end if;
            s_xa0 <= s_xa;
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            -- 1: real pixels left in the torn pixel's cell (half units
            -- left / x step; 1080: (not x) and mask, so x + d = x or mask);
            -- room left on the line; dry luma and |chroma|
            v_r := (not s_ta0) and s_maskh;
            if s_xsd = '1' then
                s_d1 <= resize(v_r(5 downto 2), 5);
            elsif s_x720 = '1' then
                s_d1 <= C_DIV3(to_integer(v_r));
            else
                s_d1 <= v_r(5 downto 1);
            end if;
            s_xt1 <= unsigned(s_xt0(10 downto 0));
            s_rem1 <= s_wm1 - unsigned(s_xt0(10 downto 0));
            s_bg1 <= s_bg0;
            s_xn1 <= s_xa0;
            s_dy1 <= s_in_y;
            if s_in_u(9) = '1' then s_au1 <= '0' & s_in_u(8 downto 0); else s_au1 <= to_unsigned(512, 10) - s_in_u; end if;
            if s_in_v(9) = '1' then s_av1 <= '0' & s_in_v(8 downto 0); else s_av1 <= to_unsigned(512, 10) - s_in_v; end if;
            s_bfd(1) <= s_brk_row; for i in 2 to 10 loop s_bfd(i) <= s_bfd(i - 1); end loop;
            s_nfd(1) <= s_nz_row;  for i in 2 to 11 loop s_nfd(i) <= s_nfd(i - 1); end loop;
            -- 2: the cell's last pixel (its window ends there); a partial
            -- last cell reads the line's last pixel.  Off-line reads are
            -- discarded at the fetch (s_a_bg3).  Screen column from the
            -- virtual x.
            if s_d1 > s_rem1 then s_addr2 <= s_wm1; else s_addr2 <= s_xt1 + s_d1; end if;
            s_bg2 <= s_bg1;
            case to_integer(s_nsh) is
                when 2 => s_nx2 <= s_xn1(2 downto 1) & "00";
                when 3 => s_nx2 <= s_xn1(3 downto 1) & "0";
                when 4 => s_nx2 <= s_xn1(4 downto 1);
                when others => s_nx2 <= s_xn1(5 downto 2);
            end case;
            s_sat2 <= resize(s_au1, 11) + resize(s_av1, 11);
            s_dy2  <= s_dy1;
            s_nz2  <= (signed('0' & s_rnd(7 downto 0)) - to_signed(128, 9)) * signed('0' & s_soft);
            -- 3: key value: luma + soft/64, or saturation + soft/128
            if s_ksat = '1' then
                s_kv3 <= signed(resize(s_sat2, 13)) + resize(shift_right(s_nz2, 7), 13);
            else
                s_kv3 <= signed(resize(s_dy2, 13)) + resize(shift_right(s_nz2, 6), 13);
            end if;
            -- 4: key (S8 flips it); 5..10 carry it to the compose stage
            if s_kv3 > signed(resize(s_klev, 13)) then v_k := '1'; else v_k := '0'; end if;
            s_kd(4) <= v_k xor s_outside;
            for i in 5 to 10 loop s_kd(i) <= s_kd(i - 1); end loop;
        end if;
    end process p_addr;

    s_scr_addr <= s_shape & s_ny & s_nx2;

    --------------------------------------------------------------------------
    -- Line buffers (written at the window's end pixel; read at 2 -> data 3)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr2));
            if s_we3 = '1' and s_wp3 = '0' then lbY0(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr2));
            if s_we3 = '1' and s_wp3 = '1' then lbY1(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr2(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbU0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr2(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbU1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr2(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbV0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr2(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbV1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (offsets 3..16)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t  : unsigned(9 downto 0);
        variable v_b  : std_logic;
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 3 ctx / 4 fetch
            s_a_bg3 <= s_bg2; s_a_rbank3 <= s_rbank;
            if s_a_bg3 = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank3 = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 5: ink amount = darkness (940 - y) for dark ink, brightness
            -- (y - 64) for light ink; flat cell colour for the mosaic
            if s_light = '1' then
                s_d5 <= signed(resize(s_wet_y, 11)) - to_signed(64, 11);
            else
                s_d5 <= to_signed(940, 11) - signed(resize(s_wet_y, 11));
            end if;
            if s_wet_y < 64 then
                s_yf(5) <= to_unsigned(64, 10);
            elsif s_wet_y > 940 then
                s_yf(5) <= to_unsigned(940, 10);
            else
                s_yf(5) <= s_wet_y;
            end if;
            s_uc(5) <= s_wet_u; s_vc(5) <= s_wet_v;
            for i in 6 to 10 loop
                s_yf(i) <= s_yf(i - 1); s_uc(i) <= s_uc(i - 1); s_vc(i) <= s_vc(i - 1);
            end loop;
            s_thr5 <= s_scr_q2;
            -- 6: amount clamped to 0..876, 8 bits (0..219); break noise
            if s_d5 < 0 then
                s_d6 <= (others => '0');
            elsif s_d5 > 876 then
                s_d6 <= to_unsigned(219, 8);
            else
                s_d6 <= unsigned(s_d5(9 downto 2));
            end if;
            s_thr6 <= s_thr5;
            s_nb6 <= (signed('0' & s_rnd(23 downto 16)) - to_signed(128, 9)) * signed('0' & s_ng);
            -- 7: ink-gain mantissa product; Break bands scatter the screen
            -- thresholds by up to +-248 (fully random at Noise 100%)
            s_pm7 <= s_d6 * s_gf;
            s_d7  <= s_d6;
            if s_bfd(6) = '1' then
                s_thr7 <= signed(resize(s_thr6, 11)) + resize(shift_right(s_nb6, 7), 11);
            else
                s_thr7 <= signed(resize(s_thr6, 11));
            end if;
            -- 8: amount x (256 + gf); threshold clamped and mirrored (ink
            -- where amount > 255 - thr)
            s_s8 <= shift_left(resize(s_d7, 17), 8) + resize(s_pm7, 17);
            if s_thr7 < 0 then
                s_ithr8 <= (others => '1');
            elsif s_thr7 > 255 then
                s_ithr8 <= (others => '0');
            else
                s_ithr8 <= not unsigned(s_thr7(7 downto 0));
            end if;
            -- 9: x 2^(gk-2) / 256, saturated to 8 bits
            case to_integer(s_gk) is
                when 0 => v_t := resize(s_s8(16 downto 10), 10);
                when 1 => v_t := resize(s_s8(16 downto 9), 10);
                when 2 => v_t := resize(s_s8(16 downto 8), 10);
                when others => v_t := s_s8(16 downto 7);
            end case;
            if v_t(9 downto 8) /= "00" then s_lum9 <= (others => '1'); else s_lum9 <= v_t(7 downto 0); end if;
            s_ithr9 <= s_ithr8;
            -- 10: ink decision; band luma noise
            if s_lum9 > s_ithr9 then s_ink10 <= '1'; else s_ink10 <= '0'; end if;
            s_nz10 <= (signed('0' & s_rnd(15 downto 8)) - to_signed(128, 9)) * signed('0' & s_ng);
            -- 11: compose: keyed pixels (and every pixel of a Break band)
            -- print the halftone, the rest show the plain cell colour.
            -- Ink is black (dark) or white (light); paper the other one.
            if (s_kd(10) or s_bfd(10)) = '1' then
                v_b := not (s_ink10 xor s_light);
                if v_b = '1' then s_y11 <= to_unsigned(940, 10); else s_y11 <= to_unsigned(64, 10); end if;
                s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            else
                s_y11 <= s_yf(10); s_u11 <= s_uc(10); s_v11 <= s_vc(10);
            end if;
            s_n11 <= resize(shift_right(s_nz10, 7), 11);
            -- 12: band noise, zero-mean (+-248 at Noise 100%; v0.1 was
            -- -128..+127 at most and a flat -128 at Noise 0)
            if s_nfd(11) = '1' then
                s_ys12 <= signed(resize(s_y11, 12)) + resize(s_n11, 12);
            else
                s_ys12 <= signed(resize(s_y11, 12));
            end if;
            s_u12 <= s_u11; s_v12 <= s_v11;
            -- 13: clamp 64..940; per-row U/V parity fix; neutral outside
            -- avid (so the smoothing's edge taps read black)
            if s_avid_sr(12) = '0' then
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            else
                if s_ys12 < 64 then
                    s_y13 <= to_unsigned(64, 10);
                elsif s_ys12 > 940 then
                    s_y13 <= to_unsigned(940, 10);
                else
                    s_y13 <= unsigned(s_ys12(9 downto 0));
                end if;
                if s_rowswap = '1' then
                    s_u13 <= s_v12; s_v13 <= s_u12;
                else
                    s_u13 <= s_u12; s_v13 <= s_v12;
                end if;
            end if;
            -- 14..15: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y13; s_g2y <= s_g1y;
            s_g1u <= s_u13; s_g2u <= s_g1u;
            s_g1v <= s_v13; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y13, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u13, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v13, 12);
            s_y15 <= v_sy(11 downto 2); s_u15 <= v_su(11 downto 2); s_v15 <= v_sv(11 downto 2);
            -- 16: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777);
            -- gate
            v_d := s_y15 - 64;
            if s_avid_sr(15) = '1' then
                s_y16 <= s_y15 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u16 <= s_u15; s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y16);
    s_io_align_in.u       <= std_logic_vector(s_u16);
    s_io_align_in.v       <= std_logic_vector(s_v16);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (17 -> 20).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 2 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(2);

end architecture pressroom;
