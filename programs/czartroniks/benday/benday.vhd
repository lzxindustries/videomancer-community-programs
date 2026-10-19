-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Benday: catalogue combo #31 (seed 20260917) -- chromatic glitch / datamosh
-- bars + ASCII / dither-art / halftone.  Three knobs each; the switches
-- and the slider decide how the two combine.  v0.2.0 = finalizing pass
-- (see SUMMARY.md for the review and what changed from v0.1.0).
-- v0.2.2 (2026-10-17) = resolution scaling: cells, mosh bands, the tear and
-- the 32-px colour blocks run in virtual 1080-line units (x steps 1 / 1.5 /
-- 2 per pixel, y steps per field line from the timing ID), so 720p and SD
-- show the 1080p print scaled to the screen height (SD = a 1440-wide 4:3
-- window; cell rows there are whole lines, 4 units a field line, so the
-- dots stay regular and ~11% tall; on 480i/576i the 4-px cell is floored
-- to 8, as it is under one field line).  1080p is bit-identical.
--
--   K1 Cell      halftone cell 4 / 8 / 16 / 32 px
--   K2 Dot Gain  how heavily the dots print (100%: black prints solid,
--                white prints clean paper)
--   K3 Screen    Dot / Line / Diamond / Square
--   K4 Height    mosh band height 8 / 16 / 32 / 64 lines
--   K5 Shift     sideways tear of a moshed band (to +-1020 virtual units:
--                1020 px at 1080, 680 at 720p, 510 on SD)
--   K6 Rate      re-roll rate (every ~2 s .. every frame)
--
--   S7  Ink       dark ink on white paper / light ink on black paper (both
--                 positive: dots grow with darkness / with brightness)
--   S8  Colour    mono print / the bright side takes the cell colour (x2 sat)
--   S9  Mosh      bands tear the picture / bands tear the screen (phase
--                 shift + the next screen pattern)
--   S10 Band FX   bands carry the 32-px held, U/V-swapped cell colour /
--                 bands print negative
--   S11 Band Fill bands halftoned / bands flat cells (macroblocks)
--   P12 Density   fraction of bands moshed: 1/16 at 0 .. every band at 100%
--
-- Cell average = running box sum (EBR ring, zero outside active video)
-- into a two-bank line buffer; address a holds the mean of pixels
-- a-N+1..a, a cell reads its last pixel.  One line per cell row is
-- captured (the last line of the row above; line 0 for the top row).
-- The halftone threshold comes from a 4 x 16 x 16 screen ROM (EBR) indexed
-- by the position inside the cell.  Bands are hashed per band and
-- re-rolled on a frame counter.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (15) in all modes; U/V
-- swapped on rows whose hsync->avid parity differs from their capture
-- row's (drape v1.3: a stored line replayed on other rows); output
-- conditioning ([1 2 1] on Y/U/V + luma scaled to white ~777: the HD HDMI
-- bar fix from damier v0.4.4 -- halftone dots are full-swing pixel edges
-- and the bands move them every re-roll); labelled rotaries honour the
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

architecture benday of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (15 + 1 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 15;

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

    -- chroma x2 about neutral, clamped (the drawn colour; dots dilute it)
    function f_boost(c : unsigned(9 downto 0)) return unsigned is
        variable t : signed(11 downto 0);
    begin
        t := signed(shift_left(resize(c, 12), 1)) - to_signed(512, 12);
        if t(11) = '1' then
            return to_unsigned(0, 10);
        elsif t(10) = '1' then
            return to_unsigned(1023, 10);
        else
            return unsigned(t(9 downto 0));
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
    -- cell-row y step (Q.8 per field line): the true step, except that the
    -- SD and 1080i steps are snapped to a power of 2 (4.0 interlaced SD,
    -- 2.0 progressive SD), so every cell row is a whole number of lines and
    -- the dots stay regular (4.44 per line put 1.8 lines in an 8-unit cell
    -- and the rows beat into streaks); SD cells are ~11% taller than exact
    function f_cstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8      => return to_unsigned(1024, 12);  -- 480i / 576i: 4.0
            when 4 | 12     => return to_unsigned(512, 12);   -- 480p / 576p: 2.0
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p: 1.5
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i: 2.0
            when others     => return to_unsigned(256, 12);   -- 1080p: 1.0
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
    signal s_k_cell, s_k_ink, s_k_screen, s_k_height, s_k_shift, s_k_rate : unsigned(9 downto 0);
    signal s_sw_light, s_sw_colour, s_sw_moshscreen, s_sw_neg, s_sw_flat : std_logic;
    signal s_p_density : unsigned(9 downto 0);
    signal s_live1, s_live3, s_live4 : std_logic := '0';
    signal s_nsh    : unsigned(2 downto 0) := to_unsigned(4, 3);       -- cell shift 2..5 (virtual)
    signal s_nshr   : unsigned(2 downto 0) := to_unsigned(4, 3);       -- box-window shift (real px)
    signal s_mask   : unsigned(10 downto 0) := to_unsigned(15, 11);    -- N - 1 (virtual)
    signal s_maskh  : unsigned(5 downto 0) := to_unsigned(31, 6);      -- 2N - 1 (half units)
    signal s_ndel   : unsigned(5 downto 0) := to_unsigned(15, 6);      -- window - 1 (real px)

    -- Resolution scaling: x in half virtual units (only the low 6 bits are
    -- ever needed: cells and the 32-unit colour blocks are powers of 2),
    -- y in Q.8 virtual units per field line (at 1080p the integer parts
    -- equal the old counters)
    signal s_xs     : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_x720, s_xsd, s_sdi : std_logic := '0';   -- 720p, SD, interlaced SD
    signal s_xa, s_xa1 : unsigned(5 downto 0) := (others => '0');
    signal s_ycs    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ya     : unsigned(19 downto 0) := (others => '0');   -- true (bands)
    signal s_yc, s_ycn : unsigned(19 downto 0) := (others => '0');   -- cell rows
    signal s_rowend : std_logic := '0';
    signal s_sh_v, s_sh_t, s_sh23 : unsigned(7 downto 0) := (others => '0');
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_gaine  : unsigned(8 downto 0) := (others => '0');         -- gain x 1.125
    signal s_shape  : unsigned(1 downto 0) := (others => '0');
    signal s_bsh    : unsigned(2 downto 0) := to_unsigned(4, 3);       -- band shift 3..6
    signal s_shift  : unsigned(7 downto 0) := (others => '0');
    signal s_dens   : unsigned(8 downto 0) := to_unsigned(16, 9);      -- 16..256
    signal s_roll   : unsigned(15 downto 0) := (others => '0');
    signal s_rate_cnt : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(1 downto 0) := "11";
    signal s_offp_h, s_offs_h : unsigned(5 downto 0) := (others => '0');   -- tears in half units, mod 64

    -- tracking / per-line
    signal s_rx, s_line : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_wm1    : unsigned(10 downto 0) := to_unsigned(719, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_capture, s_wpar, s_rbank : std_logic := '0';
    signal s_lstep  : unsigned(3 downto 0) := "1111";
    signal s_hb     : unsigned(15 downto 0) := (others => '0');
    signal s_band_row : std_logic := '0';
    signal s_prod   : signed(17 downto 0) := (others => '0');
    signal s_offp_row, s_offs_row : signed(10 downto 0) := (others => '0');
    signal s_lo_row : signed(12 downto 0) := (others => '0');
    signal s_hi_row : signed(12 downto 0) := to_signed(720, 13);
    signal s_ny     : unsigned(3 downto 0) := (others => '0');
    signal s_shape_row : unsigned(1 downto 0) := (others => '0');
    signal s_fxneg_row, s_fxcol_row, s_flat_row : std_logic := '0';
    signal s_colour_row, s_light_row : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';

    -- input side
    signal s_accy, s_accu, s_accv : unsigned(15 downto 0) := (others => '0');
    signal s_ay3, s_au3, s_av3 : unsigned(9 downto 0) := (others => '0');
    signal s_wx1, s_wx3 : unsigned(10 downto 0) := (others => '0');
    signal s_wp1, s_wc1, s_wp3, s_we3 : std_logic := '0';

    -- read side (offsets: pixel p is in s_rx at -1 and s_xt1 at 0, its output at 14)
    signal s_xt1    : signed(12 downto 0) := (others => '0');   -- torn real x
    signal s_ta1, s_sa1, s_sa2 : unsigned(5 downto 0) := (others => '0');
    signal s_xt2, s_rem2 : unsigned(10 downto 0) := (others => '0');
    signal s_d2     : unsigned(4 downto 0) := (others => '0');
    signal s_bg1, s_bg2, s_bg3 : std_logic := '0';
    signal s_addr3  : unsigned(10 downto 0) := (others => '0');
    signal s_nx3    : unsigned(3 downto 0) := (others => '0');
    signal s_xd     : std_logic_vector(1 to 8) := (others => '0');   -- x mod 32 = 0
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yk5    : unsigned(9 downto 0) := (others => '0');
    type t_c is array (5 to 9) of unsigned(9 downto 0);
    signal s_yf, s_uc, s_vc, s_ub, s_vb : t_c := (others => to_unsigned(512, 10));
    signal s_ph6    : unsigned(12 downto 0) := (others => '0');   -- gain hi 5 bits
    signal s_pl6    : unsigned(11 downto 0) := (others => '0');   -- gain lo 4 bits
    signal s_pg7    : unsigned(16 downto 0) := (others => '0');
    signal s_thr5, s_thr6, s_thr7, s_thr8 : unsigned(7 downto 0) := (others => '0');
    signal s_lum8   : unsigned(7 downto 0) := (others => '0');
    signal s_ink9   : std_logic := '0';
    signal s_yn9    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_hu, s_hv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y10, s_y11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u10, s_v10, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
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

begin

    s_k_cell   <= unsigned(registers_in(0));
    s_k_ink    <= unsigned(registers_in(1));
    s_k_screen <= unsigned(registers_in(2));
    s_k_height <= unsigned(registers_in(3));
    s_k_shift  <= unsigned(registers_in(4));
    s_k_rate   <= unsigned(registers_in(5));
    s_sw_light      <= registers_in(6)(0);
    s_sw_colour     <= registers_in(6)(1);
    s_sw_moshscreen <= registers_in(6)(2);
    s_sw_neg        <= registers_in(6)(3);
    s_sw_flat       <= registers_in(6)(4);
    s_p_density <= unsigned(registers_in(7));

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
        variable v_k  : unsigned(15 downto 0);
        variable v_i  : unsigned(1 downto 0);
        variable v_o  : signed(10 downto 0);
        variable v_m  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;

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
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_ycs <= f_cstep_y(registers_in(8)(3 downto 0));
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
            s_ycn <= s_yc + s_ycs;
            -- this line is the last of its cell row: the next line's cell
            -- row differs (always so when a cell is shorter than a line)
            if ((s_ycn(18 downto 8) xor s_yc(18 downto 8)) and not s_mask) /= 0 then
                s_rowend <= '1';
            else
                s_rowend <= '0';
            end if;
            -- K5 Shift x 2/3 for 720p (x 171/256), so the tear spans the
            -- same virtual width as at 1080p
            s_sh_v <= s_k_shift(9 downto 2);
            s_sh_t <= shift_right(s_sh_v, 1) + shift_right(s_sh_v, 3);
            s_sh23 <= s_sh_t + shift_right(s_sh_v, 5) + shift_right(s_sh_v, 7);
            s_wx1 <= s_rx; s_wp1 <= s_wpar; s_wc1 <= data_in.avid and s_capture;
            s_wx3 <= s_wx1; s_wp3 <= s_wp1; s_we3 <= s_wc1;

            -- per-line: band hash, tear, screen, row flags
            case to_integer(s_lstep) is
                when 0 =>
                    -- band and screen row from the virtual line
                    v_k := s_roll + resize(shift_right(s_ya(18 downto 8), to_integer(s_bsh)), 16);
                    s_hb <= x"3D71" + v_k;
                    case to_integer(s_nsh) is
                        when 2 => s_ny <= s_yc(9 downto 8) & "00";
                        when 3 => s_ny <= s_yc(10 downto 8) & "0";
                        when 4 => s_ny <= s_yc(11 downto 8);
                        when others => s_ny <= s_yc(12 downto 9);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 1 => s_hb <= f_round1(s_hb); s_lstep <= s_lstep + 1;
                when 2 => s_hb <= f_round2(s_hb); s_lstep <= s_lstep + 1;
                when 3 => s_hb <= f_round3(s_hb); s_lstep <= s_lstep + 1;
                when 4 =>
                    if resize(s_hb(15 downto 8), 9) < s_dens then s_band_row <= '1'; else s_band_row <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_prod <= (signed('0' & s_hb(7 downto 0)) - to_signed(128, 9)) * signed('0' & s_shift);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    -- tear = (hash - 128) * Shift / 32: +-1020 virtual units
                    -- (1080: +-1020 px; 720p: Shift x 2/3, +-680 px; SD:
                    -- / 64, +-510 px of 720)
                    if s_band_row = '0' then
                        v_o := (others => '0');
                    elsif s_xsd = '0' then
                        v_o := resize(shift_right(s_prod, 5), 11);
                    else
                        v_o := resize(shift_right(s_prod, 6), 11);
                    end if;
                    if s_sw_moshscreen = '0' then
                        s_offp_row <= v_o; s_offs_row <= (others => '0');
                    else
                        s_offp_row <= (others => '0'); s_offs_row <= v_o;
                    end if;
                    -- Mosh = Screen: torn bands also print the next screen
                    if s_band_row = '1' and s_sw_moshscreen = '1' then
                        s_shape_row <= s_shape + 1;
                    else
                        s_shape_row <= s_shape;
                    end if;
                    s_fxneg_row  <= s_band_row and s_sw_neg;
                    s_fxcol_row  <= s_band_row and not s_sw_neg;
                    s_flat_row   <= s_band_row and s_sw_flat;
                    s_colour_row <= s_sw_colour;
                    s_light_row  <= s_sw_light;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    -- the shifted read stays on the line for x in [lo, hi)
                    s_lo_row <= -resize(s_offp_row, 13);
                    s_hi_row <= signed(resize(s_line_width, 13)) - resize(s_offp_row, 13);
                    -- the tears in half virtual units (x step x offset),
                    -- mod 64: all the cell maths needs
                    case to_integer(s_xs) is
                        when 3 =>
                            s_offp_h <= unsigned(s_offp_row(5 downto 0)) + unsigned(s_offp_row(4 downto 0) & '0');
                            s_offs_h <= unsigned(s_offs_row(5 downto 0)) + unsigned(s_offs_row(4 downto 0) & '0');
                        when 4 =>
                            s_offp_h <= unsigned(s_offp_row(3 downto 0) & "00");
                            s_offs_h <= unsigned(s_offs_row(3 downto 0) & "00");
                        when others =>
                            s_offp_h <= unsigned(s_offp_row(4 downto 0) & '0');
                            s_offs_h <= unsigned(s_offs_row(4 downto 0) & '0');
                    end case;
                    s_lstep <= "1111";
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
                    if s_k_screen >= 4 then s_live3 <= '1'; end if;
                    if s_live3 = '0' and s_k_screen < 4 then v_i := s_k_screen(1 downto 0); else v_i := s_k_screen(9 downto 8); end if;
                    s_shape <= v_i;
                    if s_k_height >= 4 then s_live4 <= '1'; end if;
                    if s_live4 = '0' and s_k_height < 4 then v_i := s_k_height(1 downto 0); else v_i := s_k_height(9 downto 8); end if;
                    s_bsh <= resize(v_i, 3) + 3;
                    s_gain  <= s_k_ink(9 downto 2);
                    if s_x720 = '1' then s_shift <= s_sh23; else s_shift <= s_k_shift(9 downto 2); end if;
                    -- density 16 + P12 x 15/64 (1/16 of bands .. all 256/256)
                    s_dens  <= resize(s_p_density(9 downto 2), 9) - resize(s_p_density(9 downto 6), 9);
                    if s_rate_cnt >= resize(not s_k_rate(9 downto 3), 8) then
                        s_rate_cnt <= (others => '0');
                        s_roll <= s_roll + 977;
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
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
                    -- x 1.125: at 100% white (876 above black) maps past the
                    -- screen's top threshold and prints clean paper
                    s_gaine <= resize(s_gain, 9) + resize(s_gain(7 downto 3), 9);
                    s_dens  <= s_dens + 16;
                    s_vstep <= "11";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_wm1 <= s_rx - 1; s_line <= s_line + 1;
                s_ya <= s_ya + s_ys; s_yc <= s_ycn;
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
                s_ya <= (others => '0'); s_yc <= (others => '0');
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
    -- Read address + screen index (offsets 0..2)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_r : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: picture tear (real px) and its / the screen phase's
            -- position in half virtual units; off-line test against the
            -- per-line range (parallel to the add)
            s_xt1 <= signed(resize(s_rx, 13)) + resize(s_offp_row, 13);
            s_ta1 <= s_xa + s_offp_h;
            s_sa1 <= s_xa + s_offs_h;   -- only low bits index the screen
            if signed(resize(s_rx, 13)) < s_lo_row or signed(resize(s_rx, 13)) >= s_hi_row then
                s_bg1 <= '1';
            else
                s_bg1 <= '0';
            end if;
            s_xa1 <= s_xa;
            -- 1: real pixels left in the torn pixel's cell (half units
            -- left / x step; 1080: (not x) and mask, so x + d = x or mask);
            -- room left on the line.  32-unit colour block starts.
            v_r := (not s_ta1) and s_maskh;
            if s_xsd = '1' then
                s_d2 <= resize(v_r(5 downto 2), 5);
            elsif s_x720 = '1' then
                s_d2 <= C_DIV3(to_integer(v_r));
            else
                s_d2 <= v_r(5 downto 1);
            end if;
            s_xt2 <= unsigned(s_xt1(10 downto 0));
            s_rem2 <= s_wm1 - unsigned(s_xt1(10 downto 0));
            s_bg2 <= s_bg1;
            s_sa2 <= s_sa1;
            if s_xa1 < s_xs then s_xd(1) <= '1'; else s_xd(1) <= '0'; end if;
            for i in 2 to 8 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- 2: the cell's last pixel (its window ends there); a partial
            -- last cell reads the line's last pixel.  Off-line reads are
            -- discarded at the fetch (s_a_bg).
            if s_d2 > s_rem2 then s_addr3 <= s_wm1; else s_addr3 <= s_xt2 + s_d2; end if;
            s_bg3 <= s_bg2;
            case to_integer(s_nsh) is
                when 2 => s_nx3 <= s_sa2(2 downto 1) & "00";
                when 3 => s_nx3 <= s_sa2(3 downto 1) & "0";
                when 4 => s_nx3 <= s_sa2(4 downto 1);
                when others => s_nx3 <= s_sa2(5 downto 2);
            end case;
        end if;
    end process p_addr;

    s_scr_addr <= s_shape_row & s_ny & s_nx3;

    --------------------------------------------------------------------------
    -- Line buffers (written at the window's end pixel; read at 2 -> data 3)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr3));
            if s_we3 = '1' and s_wp3 = '0' then lbY0(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr3));
            if s_we3 = '1' and s_wp3 = '1' then lbY1(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbU0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbU1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbV0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbV1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (offsets 3..14)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l  : unsigned(7 downto 0);
        variable v_b  : std_logic;
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 3 ctx / 4 fetch
            s_a_bg <= s_bg3; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 5: luma above black; flat luma clamped to 64..940; boosted chroma
            if s_wet_y < 64 then s_yk5 <= (others => '0'); else s_yk5 <= s_wet_y - 64; end if;
            if s_wet_y < 64 then
                s_yf(5) <= to_unsigned(64, 10);
            elsif s_wet_y > 940 then
                s_yf(5) <= to_unsigned(940, 10);
            else
                s_yf(5) <= s_wet_y;
            end if;
            s_uc(5) <= s_wet_u; s_vc(5) <= s_wet_v;
            s_ub(5) <= f_boost(s_wet_u); s_vb(5) <= f_boost(s_wet_v);
            for i in 6 to 9 loop
                s_yf(i) <= s_yf(i - 1); s_uc(i) <= s_uc(i - 1); s_vc(i) <= s_vc(i - 1);
                s_ub(i) <= s_ub(i - 1); s_vb(i) <= s_vb(i - 1);
            end loop;
            -- 6..7: dot gain, the 9-bit gain split hi/lo (one LUT multiply
            -- per stage was the HD critical path)
            s_ph6 <= s_yk5(9 downto 2) * s_gaine(8 downto 4);
            s_pl6 <= s_yk5(9 downto 2) * s_gaine(3 downto 0);
            s_pg7 <= shift_left(resize(s_ph6, 17), 4) + resize(s_pl6, 17);
            -- 8: 8-bit level (clamped), mirrored for light ink; threshold
            -- (ROM address at 2, q 3, q2 4, thr 5..8)
            if s_pg7(16 downto 15) /= "00" then v_l := x"FF"; else v_l := s_pg7(14 downto 7); end if;
            if s_light_row = '1' then v_l := not v_l; end if;
            s_lum8 <= v_l;
            s_thr5 <= s_scr_q2; s_thr6 <= s_thr5; s_thr7 <= s_thr6; s_thr8 <= s_thr7;
            -- 9: ink decision; 32-px chroma hold (U/V swapped) for Colour bands
            if s_lum8 < s_thr8 then s_ink9 <= '1'; else s_ink9 <= '0'; end if;
            if s_xd(8) = '1' then s_hu <= s_vb(8); s_hv <= s_ub(8); end if;
            s_yn9 <= to_unsigned(1004, 10) - s_yf(8);
            -- 10: compose (light ink: the ink is the bright side; Negative
            -- bands flip the sides; colour always rides the bright side)
            if s_flat_row = '1' then
                if s_fxneg_row = '1' then
                    s_y10 <= s_yn9; s_u10 <= not s_uc(9); s_v10 <= not s_vc(9);
                else
                    s_y10 <= s_yf(9); s_u10 <= s_hu; s_v10 <= s_hv;
                end if;
            else
                v_b := not (s_ink9 xor s_light_row) xor s_fxneg_row;
                if v_b = '1' then
                    s_y10 <= to_unsigned(940, 10);
                    if s_fxcol_row = '1' then
                        s_u10 <= s_hu; s_v10 <= s_hv;
                    elsif s_colour_row = '1' then
                        s_u10 <= s_ub(9); s_v10 <= s_vb(9);
                    else
                        s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
                    end if;
                else
                    s_y10 <= to_unsigned(64, 10);
                    s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
                end if;
            end if;
            -- 11: per-row U/V parity fix; neutral outside avid (so the
            -- smoothing's edge taps read black)
            if s_avid_sr(10) = '0' then
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            elsif s_rowswap = '1' then
                s_y11 <= s_y10; s_u11 <= s_v10; s_v11 <= s_u10;
            else
                s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            end if;
            -- 12..13: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y11; s_g2y <= s_g1y;
            s_g1u <= s_u11; s_g2u <= s_g1u;
            s_g1v <= s_v11; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y11, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u11, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v11, 12);
            s_y13 <= v_sy(11 downto 2); s_u13 <= v_su(11 downto 2); s_v13 <= v_sv(11 downto 2);
            -- 14: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777);
            -- gate
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

end architecture benday;
