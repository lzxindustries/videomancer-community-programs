-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Stencil v0.2.0: catalogue combo #2 (seed 20260912) -- a cell-averaged
-- mosaic keyed against the original by luma / saturation.  P12 is the
-- stencil: the mosaic eats into the picture as the slider rises.
--
-- v0.2.2 (community, 2026-10-17): the labelled K2 Aspect default loads as
-- its label INDEX (firmware): K2 follows directly for 15 frames past the
-- deadband, and values below 4 decode as the index.
--
-- v0.2.3 (2026-10-17): resolution scaling.  Cells and rows are measured in
-- virtual 1080-line units: x steps 1.0 / 1.5 / 2.0 per pixel at 1080 / 720p
-- / SD (SD = a 1440-wide 4:3 window) and y steps 1080 / active lines per
-- field line (2 x 1080/486 at 480i), from the video timing ID, so cells keep
-- their 1080 size relative to the screen height and stay square on 4:3.
-- The line buffers are indexed by CELL: every capture pixel writes its
-- cell's entry, so the entry ends up holding the average that ends at the
-- cell's last pixel (the line's last, for the part cell), exactly what
-- x | mask read before; any cell width in pixels needs no divide.  The box
-- average spans 2^(n-1) pixels at 720p / SD (the whole cell at SD, 3/4 of
-- it at 720p).  Rows count virtual lines (Q8 accumulator).  Grout stays
-- 1/16 of the cell but is at least 1 px / 1 line on screen; cells / rows
-- under 2 px / 2 lines on screen (which can't show a grid) get every other
-- pixel / every (R/band)-th line instead, the 1080 coverage at pixel scale.
-- 1080p bit-identical.
--
--   K1 Cells    cell width 2 .. 256 px, eight octave steps (deadbanded)
--   K2 Aspect   cell height = width / 1, 2, 4, 8
--   K3 Grout    dark lines between cells, 1/16 of a cell thick (min 1 px)
--   K4 Tint     0-3% off; above, the mosaic cells are washed toward a hue
--               that walks once round the colour wheel
--   K5 Soft     key edge, hard (+-16 codes) .. very soft (+-512), 5 octaves
--   K6 Source   key from luma (0) .. saturation |U|+|V| (100), blended
--
--   S7  Mask    Normal: the mosaic eats the shadows first / Invert: highlights
--   S8  Key     key from the original / from the mosaic (cell by cell)
--   S9  Reveal  the uncovered picture is the original / its negative
--   S10 Luma    Sharp: mosaic cells keep the original luma (colour only)
--   S11 Grout   grid everywhere / only on the mosaic
--   P12 Stencil the key threshold, glided: 0% = only the deepest shadows are
--               mosaic, 100% = all but the brightest highlights
--
-- Horizontal cell average = an exact N-sample running box sum (EBR ring read
-- N-1 back; the line's first pixel stands in for the samples before it), so
-- address a of the line buffer holds the average of pixels a-N+1 .. a.  The
-- display reads the cell's LAST pixel (x | mask): the cell's own average.
-- Rows: the last line of each cell row captures the next row's averages into
-- one bank while the other is displayed; the frame's first line shows the
-- original.  Replayed rows swap U/V where their hsync->avid parity differs
-- from the capture row's (drape v1.3).  Dry video comes from two EBR rings,
-- tapped where the key and the blend need it (shallow raw video).  Output:
-- [1 2 1] on Y/U/V + luma scaled to white ~775 (the HD HDMI bar fix),
-- blanking-gated.  Latency 24 clocks in all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture stencil of program_top is
    -- data_in -> data_out latency is 24 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;

    constant C_LAT : integer := 24;
    constant C_DK  : integer := 4;     -- key dry ring: lands as stage 6
    constant C_DB  : integer := 14;    -- blend dry ring: lands as stage 16
    constant C_BLANK : std_logic_vector(31 downto 0) := "00" & "0001000000" & "1000000000" & "1000000000";

    -- K5 soft: key gain 8 * 2^(-5k/63) = G / 2^S, packed G(6..0) & (S-3)(2..0)
    type t_soft is array (0 to 63) of std_logic_vector(9 downto 0);
    constant C_SOFT : t_soft := (
        "1000000000", "1111001001", "1110011001", "1101101001", "1100111001", "1100001001", "1011100001", "1010111001",
        "1010010001", "1001110001", "1001010001", "1000110001", "1000010001", "1111101010", "1110111010", "1110000010",
        "1101010010", "1100100010", "1011111010", "1011010010", "1010101010", "1010001010", "1001100010", "1001000010",
        "1000100010", "1000001010", "1111010011", "1110100011", "1101110011", "1101000011", "1100010011", "1011101011",
        "1011000011", "1010011011", "1001111011", "1001011011", "1000111011", "1000011011", "1111111100", "1111000100",
        "1110001100", "1101011100", "1100110100", "1100000100", "1011011100", "1010110100", "1010010100", "1001101100",
        "1001001100", "1000101100", "1000001100", "1111100101", "1110101101", "1101111101", "1101001101", "1100011101",
        "1011110101", "1011001101", "1010100101", "1010000101", "1001011101", "1000111101", "1000100101", "1000000101"
    );
    -- K4 tint wheel: 200 * sin(2 pi i / 64)
    type t_sin is array (0 to 63) of integer range -200 to 200;
    constant C_SIN : t_sin := (
           0,   20,   39,   58,   77,   94,  111,  127,
         141,  155,  166,  176,  185,  191,  196,  199,
         200,  199,  196,  191,  185,  176,  166,  155,
         141,  127,  111,   94,   77,   58,   39,   20,
           0,  -20,  -39,  -58,  -77,  -94, -111, -127,
        -141, -155, -166, -176, -185, -191, -196, -199,
        -200, -199, -196, -191, -185, -176, -166, -155,
        -141, -127, -111,  -94,  -77,  -58,  -39,  -20
    );

    -- Resolution scaling (mondrian v1.0.2): per-mode steps from the timing ID.
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
    -- cell index: virtual x >> n, n = 1..8 (fixed slices)
    function f_shr(x : unsigned(10 downto 0); n : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(n) is
            when 1 => r := x(10 downto 1);
            when 2 => r := resize(x(10 downto 2), 10);
            when 3 => r := resize(x(10 downto 3), 10);
            when 4 => r := resize(x(10 downto 4), 10);
            when 5 => r := resize(x(10 downto 5), 10);
            when 6 => r := resize(x(10 downto 6), 10);
            when 7 => r := resize(x(10 downto 7), 10);
            when others => r := resize(x(10 downto 8), 10);
        end case;
        return r;
    end function;

    -- x * 2^n and a / 2^n for n = 1..8 (fixed slices, no barrel)
    function f_mul(x : unsigned(9 downto 0); n : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(17 downto 0);
    begin
        r := (others => '0');
        case to_integer(n) is
            when 1 => r(10 downto 1) := x;
            when 2 => r(11 downto 2) := x;
            when 3 => r(12 downto 3) := x;
            when 4 => r(13 downto 4) := x;
            when 5 => r(14 downto 5) := x;
            when 6 => r(15 downto 6) := x;
            when 7 => r(16 downto 7) := x;
            when others => r(17 downto 8) := x;
        end case;
        return r;
    end function;
    function f_avg(a : unsigned(17 downto 0); n : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(n) is
            when 1 => return a(10 downto 1);
            when 2 => return a(11 downto 2);
            when 3 => return a(12 downto 3);
            when 4 => return a(13 downto 4);
            when 5 => return a(14 downto 5);
            when 6 => return a(15 downto 6);
            when 7 => return a(16 downto 7);
            when others => return a(17 downto 8);
        end case;
    end function;
    -- thermometer: bit i set when i + off < n
    function f_therm(n : unsigned(3 downto 0); w : integer; off : integer) return unsigned is
        variable r : unsigned(w - 1 downto 0);
    begin
        for i in 0 to w - 1 loop
            if i + off < to_integer(n) then r(i) := '1'; else r(i) := '0'; end if;
        end loop;
        return r;
    end function;
    -- luma minus the grout, floored at black
    function f_grout(y : unsigned(9 downto 0); g : unsigned(9 downto 0)) return unsigned is
        variable v : signed(11 downto 0);
    begin
        v := signed(resize(y, 12)) - signed(resize(g, 12));
        if v < 64 then
            return to_unsigned(64, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Memories (no_rw_check: rings never read their write address; a line
    -- buffer's read data is discarded whenever its bank is being written)
    --------------------------------------------------------------------------
    type t_bring is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_ring is array (0 to 31) of std_logic_vector(31 downto 0);
    type t_lb_y is array (0 to 1023) of std_logic_vector(9 downto 0);   -- by cell
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal bring : t_bring := (others => C_BLANK);
    signal ringk, ringb : t_ring := (others => C_BLANK);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    attribute no_rw_check : boolean;
    attribute no_rw_check of bring, ringk, ringb, lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;
    signal s_bq, s_kq, s_bdq : std_logic_vector(31 downto 0) := C_BLANK;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := "0001000000";
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rw : unsigned(7 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-frame controls
    --------------------------------------------------------------------------
    signal s_vstep  : unsigned(4 downto 0) := (others => '1');
    signal s_saw_active : std_logic := '0';
    signal s_k1h    : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal s_k2h    : unsigned(9 downto 0) := (others => '0');
    signal s_kwarm  : unsigned(3 downto 0) := (others => '0');   -- K2 follows directly for 15 frames after load
    signal s_k4h    : unsigned(9 downto 0) := (others => '0');
    signal s_d1, s_d2, s_d4 : signed(10 downto 0) := (others => '0');
    signal s_soft_q : std_logic_vector(9 downto 0) := C_SOFT(18);
    signal s_sin_a  : unsigned(5 downto 0) := (others => '0');
    signal s_sin_q  : signed(8 downto 0) := (others => '0');
    signal s_nsh    : unsigned(3 downto 0) := to_unsigned(4, 4);   -- cell = 2^nsh, 1..8
    signal s_ash    : unsigned(1 downto 0) := (others => '0');
    signal s_msh    : unsigned(3 downto 0) := to_unsigned(4, 4);   -- rows = 2^msh
    signal s_mask   : unsigned(10 downto 0) := to_unsigned(15, 11);       -- box window - 1
    signal s_nmask  : unsigned(10 downto 0) := not to_unsigned(15, 11);
    signal s_cmaskh : unsigned(8 downto 0) := to_unsigned(31, 9);         -- cell, virtual half units - 1
    signal s_cm     : unsigned(8 downto 0) := to_unsigned(15, 9);         -- cell, virtual units - 1
    signal s_gtx    : unsigned(8 downto 0) := to_unsigned(2, 9);          -- grout columns, half units
    signal s_gty    : unsigned(12 downto 0) := to_unsigned(256, 13);      -- grout lines, Q.8
    signal s_nb     : unsigned(3 downto 0) := to_unsigned(4, 4);          -- box = 2^nb pixels
    signal s_r2ok   : std_logic := '1';                                   -- rows are 2+ lines on screen
    signal s_cfine  : std_logic := '0';                                   -- cells are under 2 px on screen
    signal s_pm     : unsigned(3 downto 0) := (others => '0');            -- line-grout period - 1
    signal s_lcnt   : unsigned(3 downto 0) := (others => '0');            -- screen lines since vsync, mod 16
    signal s_xpar   : std_logic := '0';                                   -- read-side pixel parity
    -- resolution scaling: steps decoded free-running from the timing ID
    signal s_stx    : unsigned(2 downto 0) := to_unsigned(2, 3);          -- Q.1 per pixel
    signal s_sty    : unsigned(11 downto 0) := to_unsigned(256, 12);      -- Q4.8 per field line
    signal s_msd, s_m720 : std_logic := '0';
    signal s_vxw    : unsigned(12 downto 0) := (others => '0');           -- write-side virtual x, Q.1
    signal s_vx4    : unsigned(12 downto 0) := (others => '0');           -- read-side virtual x, Q.1
    signal s_lwv    : unsigned(10 downto 0) := to_unsigned(479, 11);      -- last pixel's virtual x
    signal s_lastci : unsigned(9 downto 0) := (others => '1');            -- its cell
    signal s_ya     : unsigned(16 downto 0) := (others => '1');           -- line position in its row, Q9.8
    signal s_capchk : std_logic := '0';
    signal s_rm1    : unsigned(8 downto 0) := to_unsigned(15, 9);
    signal s_grow_en : std_logic := '1';
    signal s_G      : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal s_S      : unsigned(3 downto 0) := to_unsigned(5, 4);
    signal s_wl0    : unsigned(7 downto 0) := (others => '0');
    signal s_wl, s_ws : unsigned(8 downto 0) := (others => '0');
    signal s_pd     : signed(14 downto 0) := (others => '0');
    signal s_pg     : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);   -- glided P12, 4 frac bits
    signal s_t0     : unsigned(10 downto 0) := (others => '0');
    signal s_T      : unsigned(10 downto 0) := to_unsigned(528, 11);
    signal s_cu, s_cv : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_tint_on : std_logic := '0';
    signal s_grout  : unsigned(9 downto 0) := (others => '0');
    signal s_inv, s_keymos, s_neg, s_sharp, s_groutin : std_logic := '0';

    --------------------------------------------------------------------------
    -- Line tracking, box sums, write port
    --------------------------------------------------------------------------
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_rx       : unsigned(10 downto 0) := (others => '0');
    signal s_capture  : std_logic := '0';
    signal s_wpar     : std_logic := '0';
    signal s_rb_nx, s_rbank : std_logic := '0';
    signal s_l0_nx, s_line0 : std_logic := '0';
    signal s_grow     : std_logic := '0';
    signal s_hpar     : std_logic := '0';
    signal s_cpar     : std_logic_vector(1 downto 0) := "00";
    signal s_swap     : std_logic := '0';
    signal s_xdy, s_xdu, s_xdv : unsigned(9 downto 0) := (others => '0');
    signal s_x0y, s_x0u, s_x0v : unsigned(9 downto 0) := (others => '0');
    signal s_q2       : std_logic_vector(29 downto 0) := (others => '0');
    signal s_cond     : std_logic := '0';
    signal s_dy, s_du, s_dv : signed(10 downto 0) := (others => '0');
    signal s_xny, s_xnu, s_xnv : unsigned(17 downto 0) := (others => '0');
    signal s_first    : std_logic := '0';
    signal s_accy, s_accu, s_accv : unsigned(17 downto 0) := (others => '0');
    signal s_avy, s_avu, s_avv : unsigned(9 downto 0) := (others => '0');
    type t_x is array (1 to 4) of unsigned(9 downto 0);
    signal s_wx : t_x := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 4) := (others => '0');

    --------------------------------------------------------------------------
    -- Display / key / blend pipeline (names carry the stage they hold)
    --------------------------------------------------------------------------
    signal s_addr     : unsigned(9 downto 0) := (others => '0');
    signal s_gf4      : std_logic := '0';
    type t_b is array (5 to 17) of std_logic;
    signal s_gf       : t_b := (others => '0');
    signal s_wy6, s_wu6, s_wv6 : unsigned(9 downto 0) := (others => '0');
    signal s_dky, s_dku, s_dkv : unsigned(9 downto 0) := (others => '0');
    type t_c is array (7 to 16) of unsigned(9 downto 0);
    signal s_my, s_mu, s_mv : t_c := (others => (others => '0'));
    signal s_ky7, s_ku7, s_kv7 : unsigned(9 downto 0) := (others => '0');
    signal s_vy8      : unsigned(9 downto 0) := (others => '0');
    signal s_du8, s_dv8 : signed(10 downto 0) := (others => '0');
    signal s_vy9, s_au9, s_av9 : unsigned(9 downto 0) := (others => '0');
    signal s_vy10     : unsigned(9 downto 0) := (others => '0');
    signal s_sat10    : unsigned(10 downto 0) := (others => '0');
    signal s_pl11     : unsigned(18 downto 0) := (others => '0');
    signal s_ps11     : unsigned(19 downto 0) := (others => '0');
    signal s_sum12    : unsigned(20 downto 0) := (others => '0');
    signal s_val13    : unsigned(9 downto 0) := (others => '0');
    signal s_dif14    : signed(11 downto 0) := (others => '0');
    signal s_ap15     : signed(19 downto 0) := (others => '0');
    signal s_as16     : signed(19 downto 0) := (others => '0');
    signal s_dqy, s_dqu, s_dqv : unsigned(9 downto 0) := (others => '0');
    signal s_al17, s_al18 : unsigned(7 downto 0) := (others => '0');
    signal s_ry17, s_ru17, s_rv17 : unsigned(9 downto 0) := (others => '0');
    signal s_my17, s_mu17, s_mv17 : unsigned(9 downto 0) := (others => '0');
    signal s_ry18, s_ru18, s_rv18 : unsigned(9 downto 0) := (others => '0');
    signal s_my18, s_mu18, s_mv18 : unsigned(9 downto 0) := (others => '0');
    signal s_ey19, s_eu19, s_ev19 : signed(10 downto 0) := (others => '0');
    signal s_my19, s_mu19, s_mv19 : unsigned(9 downto 0) := (others => '0');
    signal s_a19      : unsigned(8 downto 0) := (others => '0');
    signal s_py20, s_pu20, s_pv20 : signed(20 downto 0) := (others => '0');
    signal s_my20, s_mu20, s_mv20 : unsigned(9 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_g1y, s_g1u, s_g1v, s_g2y, s_g2u, s_g2v : unsigned(9 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := (others => '0');
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Rings: box-sum delay (256 deep, read N-1 back) and two dry taps
    --------------------------------------------------------------------------
    p_bring : process(clk)
    begin
        if rising_edge(clk) then
            s_rw <= s_rw + 1;
            s_bq <= bring(to_integer(s_rw - s_mask(7 downto 0)));
            bring(to_integer(s_rw)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;
    p_ringk : process(clk)
    begin
        if rising_edge(clk) then
            s_kq <= ringk(to_integer(s_rw(4 downto 0) - C_DK));
            ringk(to_integer(s_rw(4 downto 0))) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;
    p_ringb : process(clk)
    begin
        if rising_edge(clk) then
            s_bdq <= ringb(to_integer(s_rw(4 downto 0) - C_DB));
            ringb(to_integer(s_rw(4 downto 0))) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Tracking, row capture, box sums, per-frame sequencer
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_ys : unsigned(16 downto 0);
        variable v_pg : signed(14 downto 0);
        variable v_t  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            -- resolution scaling: per-mode steps, free-running
            s_stx <= f_vstep_x(registers_in(8)(3 downto 0));
            s_sty <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_msd <= '1'; s_m720 <= '0';
                when 5 | 6 | 14     => s_msd <= '0'; s_m720 <= '1';
                when others         => s_msd <= '0'; s_m720 <= '0';
            end case;
            if data_in.avid = '1' then
                s_rx  <= s_rx + 1;
                s_vxw <= s_vxw + s_stx;
                s_saw_active <= '1';
            else
                s_rx  <= (others => '0');
                s_vxw <= (others => '0');
            end if;
            s_lastci <= f_shr(s_lwv, s_nsh);

            -- hsync->avid parity (drape): clock parity since hsync, per line
            s_hpar <= not s_hpar;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
            end if;

            -- end of a line: the next line's position in its row (virtual
            -- lines, Q8; rows are 2^msh units, so the wrap is a mask) and, one
            -- clock later, whether it ends its row (then it captures)
            s_capchk <= '0';
            if s_capchk = '1' then
                v_ys := s_ya + s_sty;
                if (v_ys(16 downto 8) and not s_rm1) /= 0 then
                    s_capture <= '1';
                    s_wpar    <= not s_wpar;
                else
                    s_capture <= '0';
                end if;
            end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                v_t := s_vxw - s_stx;
                s_lwv <= v_t(11 downto 1);    -- the last pixel, virtual
                v_ys := s_ya + s_sty;
                if s_line0 = '1' then
                    s_ya <= (others => '0');
                else
                    s_ya <= (v_ys(16 downto 8) and s_rm1) & v_ys(7 downto 0);
                end if;
                s_rb_nx  <= s_wpar;           -- the last completed capture
                s_capchk <= '1';
                s_l0_nx  <= '0';
                s_lcnt   <= s_lcnt + 1;
            end if;
            -- start of a line: the read side switches here (in-flight pixels
            -- of the previous line are long gone)
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_rbank <= s_rb_nx;
                if s_rb_nx = '1' then
                    s_swap <= s_hpar xor s_cpar(1);
                else
                    s_swap <= s_hpar xor s_cpar(0);
                end if;
                if s_capture = '1' then
                    if s_wpar = '1' then s_cpar(1) <= s_hpar; else s_cpar(0) <= s_hpar; end if;
                end if;
                s_line0 <= s_l0_nx;
                -- rows under 2 screen lines can't show a band: their grout
                -- becomes every (R/band)-th screen line (the same coverage)
                if s_grow_en = '1' and ((s_r2ok = '1' and s_ya < resize(s_gty, 17)) or
                                        (s_r2ok = '0' and (s_lcnt and s_pm) = 0)) then
                    s_grow <= '1';
                else
                    s_grow <= '0';
                end if;
            end if;
            -- frame start (idempotent: vsync may serrate): the first line
            -- captures into bank 0 and shows the original
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_ya      <= (others => '1');
                s_lcnt    <= (others => '0');
                s_capchk  <= '0';
                s_capture <= '1';
                s_wpar    <= '0';
                s_rb_nx   <= '1';
                s_l0_nx   <= '1';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            -- box path E0: raw sample, ring sample N back or the line's first
            -- pixel (edge extension), first-pixel latch
            s_xdy <= unsigned(data_in.y); s_xdu <= unsigned(data_in.u); s_xdv <= unsigned(data_in.v);
            s_q2  <= s_bq(29 downto 0);
            if (s_rx and s_nmask) /= 0 then s_cond <= '1'; else s_cond <= '0'; end if;   -- box window (pixels)
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_x0y <= unsigned(data_in.y); s_x0u <= unsigned(data_in.u); s_x0v <= unsigned(data_in.v);
            end if;
            -- E1: difference, first-pixel preload (x * N)
            if s_cond = '1' then
                s_dy <= signed('0' & s_xdy) - signed('0' & unsigned(s_q2(29 downto 20)));
                s_du <= signed('0' & s_xdu) - signed('0' & unsigned(s_q2(19 downto 10)));
                s_dv <= signed('0' & s_xdv) - signed('0' & unsigned(s_q2(9 downto 0)));
            else
                s_dy <= signed('0' & s_xdy) - signed('0' & s_x0y);
                s_du <= signed('0' & s_xdu) - signed('0' & s_x0u);
                s_dv <= signed('0' & s_xdv) - signed('0' & s_x0v);
            end if;
            s_xny <= f_mul(s_xdy, s_nb); s_xnu <= f_mul(s_xdu, s_nb); s_xnv <= f_mul(s_xdv, s_nb);
            s_first <= s_avid_sr(0) and not s_avid_sr(1);
            -- E2: running sums = exact N-sample window ending at this pixel
            if s_first = '1' then
                s_accy <= s_xny; s_accu <= s_xnu; s_accv <= s_xnv;
            else
                s_accy <= unsigned(signed(s_accy) + s_dy);
                s_accu <= unsigned(signed(s_accu) + s_du);
                s_accv <= unsigned(signed(s_accv) + s_dv);
            end if;
            -- E3: average
            s_avy <= f_avg(s_accy, s_nb); s_avu <= f_avg(s_accu, s_nb); s_avv <= f_avg(s_accv, s_nb);
            -- write port: cell index / bank / enable, 4 clocks behind; every
            -- pixel writes, so a cell's entry ends on its last pixel's average
            s_wx(1) <= f_shr(s_vxw(11 downto 1), s_nsh); s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid and s_capture;
            for i in 2 to 4 loop
                s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1);
            end loop;

            -- per-frame decode, one small op per state, in vblank
            s_sin_q  <= to_signed(C_SIN(to_integer(s_sin_a)), 9);
            case to_integer(s_vstep) is
                when 0 =>
                    s_d1 <= signed('0' & unsigned(registers_in(0))) - signed('0' & s_k1h);
                    s_d2 <= signed('0' & unsigned(registers_in(1))) - signed('0' & s_k2h);
                    s_d4 <= signed('0' & unsigned(registers_in(3))) - signed('0' & s_k4h);
                    s_inv     <= registers_in(6)(0);
                    s_keymos  <= registers_in(6)(1);
                    s_neg     <= registers_in(6)(2);
                    s_sharp   <= registers_in(6)(3);
                    s_groutin <= registers_in(6)(4);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- deadband the stepped knobs (no flicker at a step edge)
                    if s_d1 > 12 or s_d1 < -12 then s_k1h <= unsigned(registers_in(0)); end if;
                    -- (K2's labelled default arrives as its index 0..3, inside
                    -- the deadband, so K2 follows directly for 15 frames)
                    if s_kwarm /= 15 or s_d2 > 12 or s_d2 < -12 then s_k2h <= unsigned(registers_in(1)); end if;
                    if s_kwarm /= 15 then s_kwarm <= s_kwarm + 1; end if;
                    if s_d4 > 12 or s_d4 < -12 then s_k4h <= unsigned(registers_in(3)); end if;
                    s_soft_q <= C_SOFT(to_integer(unsigned(registers_in(4)(9 downto 4))));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_nsh <= resize(s_k1h(9 downto 7), 4) + 1;
                    -- labelled default: the firmware loads the label INDEX
                    if s_k2h(9 downto 2) = 0 then s_ash <= s_k2h(1 downto 0);
                    else s_ash <= s_k2h(9 downto 8); end if;
                    s_G <= unsigned(s_soft_q(9 downto 3));
                    s_S <= resize(unsigned(s_soft_q(2 downto 0)), 4) + 3;
                    s_sin_a <= s_k4h(9 downto 4) + 16;       -- cos
                    if s_k4h(9 downto 5) /= 0 then s_tint_on <= '1'; else s_tint_on <= '0'; end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    if s_nsh > resize(s_ash, 4) then
                        s_msh <= s_nsh - resize(s_ash, 4);
                    else
                        s_msh <= (others => '0');
                    end if;
                    -- box average: the cell's pixels at SD (N/2), 3/4 of them
                    -- at 720p (N/2 of 2N/3); at least 2
                    if (s_msd = '1' or s_m720 = '1') and s_nsh > 1 then
                        s_nb <= s_nsh - 1;
                    else
                        s_nb <= s_nsh;
                    end if;
                    s_sin_a <= s_k4h(9 downto 4);             -- sin
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    -- tint target, halved: u' = u/2 + (512 + 400 cos)/2
                    s_cu <= unsigned(to_signed(256, 10) + resize(s_sin_q, 10));
                    s_mask   <= f_therm(s_nb, 11, 0);
                    s_cm     <= f_therm(s_nsh, 9, 0);
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_cv <= unsigned(to_signed(256, 10) + resize(s_sin_q, 10));
                    s_nmask  <= not s_mask;
                    s_rm1    <= f_therm(s_msh, 9, 0);
                    s_cmaskh <= s_cm(7 downto 0) & '1';
                    -- grout columns: 1/16 of the cell (min 1 unit), in half
                    -- units, at least one pixel on screen; none on cells
                    -- narrower than 2 px on screen
                    if s_nsh >= 4 then
                        v_t := shift_right(resize(s_cm, 13) + 1, 3);
                    else
                        v_t := to_unsigned(2, 13);
                    end if;
                    if resize(s_cm, 13) + 1 < resize(s_stx, 13) then
                        s_gtx <= (others => '0');             -- every other pixel instead
                    elsif v_t < resize(s_stx, 13) then
                        s_gtx <= resize(s_stx, 9);
                    else
                        s_gtx <= v_t(8 downto 0);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_wl0 <= to_unsigned(255, 8) - unsigned(registers_in(5)(9 downto 2));
                    s_ws  <= resize(unsigned(registers_in(5)(9 downto 2)), 9)
                             + resize(unsigned(registers_in(5)(9 downto 3)), 9);   -- sat x1.5
                    -- grout lines: 1/16 of the row (min 1 unit), at least one
                    -- line on screen; none on rows under 2 lines on screen
                    if s_msh >= 4 then
                        v_t := shift_left(resize(s_rm1, 13) + 1, 4);
                    else
                        v_t := to_unsigned(256, 13);
                    end if;
                    if v_t < resize(s_sty, 13) then
                        s_gty <= resize(s_sty, 13);
                    else
                        s_gty <= v_t;
                    end if;
                    if shift_left(resize(s_rm1, 17) + 1, 8) >= shift_left(resize(s_sty, 17), 1) then
                        s_r2ok <= '1';
                    else
                        s_r2ok <= '0';
                    end if;
                    if resize(s_cm, 13) + 1 < resize(s_stx, 13) then s_cfine <= '1'; else s_cfine <= '0'; end if;
                    -- line grout period R / band = min(R, 16) lines
                    if s_rm1 > 15 then s_pm <= (others => '1'); else s_pm <= s_rm1(3 downto 0); end if;
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_wl <= resize(s_wl0, 9) + resize(s_wl0(7 downto 3), 9);         -- luma x1.125
                    if s_msh /= 0 then s_grow_en <= '1'; else s_grow_en <= '0'; end if;
                    s_pd <= signed(shift_left(resize(unsigned(registers_in(7)), 15), 4)) - signed(resize(s_pg, 15));
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    -- P12 glide, 1/8 per frame
                    v_pg := signed('0' & s_pg) + shift_right(s_pd, 3);
                    s_pg <= unsigned(v_pg(13 downto 0));
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    s_t0 <= resize(s_pg(13 downto 4), 11) + 48;
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    -- threshold 48 .. 1007 in key units
                    s_T <= s_t0 - resize(s_pg(13 downto 8), 11);
                    s_grout <= unsigned(registers_in(2));
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain 3..4: the pixel's cell; grout flag
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_a : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_avid_sr(2) = '0' then
                s_vx4 <= (others => '0'); s_xpar <= '0';
            else
                s_vx4 <= s_vx4 + s_stx;   s_xpar <= not s_xpar;
            end if;
            -- the pixel's cell (never past the last line's last cell)
            v_a := f_shr(s_vx4(11 downto 1), s_nsh);
            if v_a > s_lastci then
                s_addr <= s_lastci;
            else
                s_addr <= v_a;
            end if;
            -- grout: the cell's first columns (virtual half units)
            if (s_vx4(8 downto 0) and s_cmaskh) < s_gtx or (s_cfine = '1' and s_xpar = '0') or s_grow = '1' then
                s_gf4 <= '1';
            else
                s_gf4 <= '0';
            end if;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (averages written 4 clocks behind, read at stage 5)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_wc(4) = '1' and s_wp(4) = '0' then lbY0(to_integer(s_wx(4))) <= std_logic_vector(s_avy); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_wc(4) = '1' and s_wp(4) = '1' then lbY1(to_integer(s_wx(4))) <= std_logic_vector(s_avy); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr));
            if s_wc(4) = '1' and s_wp(4) = '0' then lbU0(to_integer(s_wx(4))) <= std_logic_vector(s_avu(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr));
            if s_wc(4) = '1' and s_wp(4) = '1' then lbU1(to_integer(s_wx(4))) <= std_logic_vector(s_avu(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr));
            if s_wc(4) = '1' and s_wp(4) = '0' then lbV0(to_integer(s_wx(4))) <= std_logic_vector(s_avv(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr));
            if s_wc(4) = '1' and s_wp(4) = '1' then lbV1(to_integer(s_wx(4))) <= std_logic_vector(s_avv(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 6..24: fetch, key, blend, conditioning, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u, v_v : unsigned(9 downto 0);
        variable v_s : unsigned(12 downto 0);
        variable v_a : signed(19 downto 0);
        variable v_y, v_c1, v_c2 : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_gf(5) <= s_gf4;
            for i in 6 to 17 loop s_gf(i) <= s_gf(i - 1); end loop;
            -- stage 6: bank select; U/V swapped on rows whose hsync->avid
            -- parity differs from the capture row's; key dry tap
            if s_rbank = '1' then
                s_wy6 <= unsigned(s_rdY1); v_u := unsigned(s_rdU1) & "00"; v_v := unsigned(s_rdV1) & "00";
            else
                s_wy6 <= unsigned(s_rdY0); v_u := unsigned(s_rdU0) & "00"; v_v := unsigned(s_rdV0) & "00";
            end if;
            if s_swap = '1' then
                s_wu6 <= v_v; s_wv6 <= v_u;
            else
                s_wu6 <= v_u; s_wv6 <= v_v;
            end if;
            s_dky <= unsigned(s_kq(29 downto 20)); s_dku <= unsigned(s_kq(19 downto 10)); s_dkv <= unsigned(s_kq(9 downto 0));
            -- stage 7: tint wash on the mosaic; key source (untinted)
            s_my(7) <= s_wy6;
            if s_tint_on = '1' then
                s_mu(7) <= ('0' & s_wu6(9 downto 1)) + s_cu;
                s_mv(7) <= ('0' & s_wv6(9 downto 1)) + s_cv;
            else
                s_mu(7) <= s_wu6;
                s_mv(7) <= s_wv6;
            end if;
            for i in 8 to 16 loop
                s_my(i) <= s_my(i - 1); s_mu(i) <= s_mu(i - 1); s_mv(i) <= s_mv(i - 1);
            end loop;
            if s_keymos = '1' then
                s_ky7 <= s_wy6; s_ku7 <= s_wu6; s_kv7 <= s_wv6;
            else
                s_ky7 <= s_dky; s_ku7 <= s_dku; s_kv7 <= s_dkv;
            end if;
            -- stage 8: luma above black, chroma centred
            if s_ky7 < 64 then s_vy8 <= (others => '0'); else s_vy8 <= s_ky7 - 64; end if;
            s_du8 <= signed('0' & s_ku7) - 512;
            s_dv8 <= signed('0' & s_kv7) - 512;
            -- stage 9: |U|, |V| (ones' complement)
            s_vy9 <= s_vy8;
            if s_du8(10) = '1' then s_au9 <= unsigned(not s_du8(9 downto 0)); else s_au9 <= unsigned(s_du8(9 downto 0)); end if;
            if s_dv8(10) = '1' then s_av9 <= unsigned(not s_dv8(9 downto 0)); else s_av9 <= unsigned(s_dv8(9 downto 0)); end if;
            -- stage 10: saturation
            s_vy10  <= s_vy9;
            s_sat10 <= resize(s_au9, 11) + resize(s_av9, 11);
            -- stage 11: luma / saturation weights (K6)
            s_pl11 <= s_vy10 * s_wl;
            s_ps11 <= s_sat10 * s_ws;
            -- stage 12
            s_sum12 <= resize(s_pl11, 21) + resize(s_ps11, 21);
            -- stage 13: key value 0..1023, complemented for Invert
            v_s := s_sum12(20 downto 8);
            if v_s(12 downto 10) /= 0 then
                s_val13 <= (others => not s_inv);
            elsif s_inv = '1' then
                s_val13 <= not v_s(9 downto 0);
            else
                s_val13 <= v_s(9 downto 0);
            end if;
            -- stage 14: distance from the P12 threshold
            s_dif14 <= signed(resize(s_val13, 12)) - signed(resize(s_T, 12));
            -- stage 15: soft gain mantissa
            s_ap15 <= s_dif14 * signed('0' & s_G);
            -- stage 16: soft gain exponent; blend dry tap
            case to_integer(s_S) is
                when 3 => s_as16 <= shift_right(s_ap15, 3);
                when 4 => s_as16 <= shift_right(s_ap15, 4);
                when 5 => s_as16 <= shift_right(s_ap15, 5);
                when 6 => s_as16 <= shift_right(s_ap15, 6);
                when 7 => s_as16 <= shift_right(s_ap15, 7);
                when others => s_as16 <= shift_right(s_ap15, 8);
            end case;
            s_dqy <= unsigned(s_bdq(29 downto 20)); s_dqu <= unsigned(s_bdq(19 downto 10)); s_dqv <= unsigned(s_bdq(9 downto 0));
            -- stage 17: alpha (255 = original), reveal, mosaic luma
            v_a := s_as16 + 128;
            if s_line0 = '1' then
                s_al17 <= (others => '1');
            elsif v_a(19) = '1' then
                s_al17 <= (others => '0');
            elsif v_a(18 downto 8) /= 0 then
                s_al17 <= (others => '1');
            else
                s_al17 <= unsigned(v_a(7 downto 0));
            end if;
            if s_neg = '1' then
                if s_dqy >= 940 then s_ry17 <= to_unsigned(64, 10); else s_ry17 <= 1004 - s_dqy; end if;
                s_ru17 <= not s_dqu; s_rv17 <= not s_dqv;
            else
                s_ry17 <= s_dqy; s_ru17 <= s_dqu; s_rv17 <= s_dqv;
            end if;
            if s_sharp = '1' then s_my17 <= s_dqy; else s_my17 <= s_my(16); end if;
            s_mu17 <= s_mu(16); s_mv17 <= s_mv(16);
            -- stage 18: grout on the mosaic (and on the reveal unless Masked)
            if s_gf(17) = '1' then
                s_my18 <= f_grout(s_my17, s_grout);
                if s_groutin = '0' then s_ry18 <= f_grout(s_ry17, s_grout); else s_ry18 <= s_ry17; end if;
            else
                s_my18 <= s_my17;
                s_ry18 <= s_ry17;
            end if;
            s_mu18 <= s_mu17; s_mv18 <= s_mv17;
            s_ru18 <= s_ru17; s_rv18 <= s_rv17;
            s_al18 <= s_al17;
            -- stage 19: differences (reveal - mosaic), alpha 0..256
            s_ey19 <= signed('0' & s_ry18) - signed('0' & s_my18);
            s_eu19 <= signed('0' & s_ru18) - signed('0' & s_mu18);
            s_ev19 <= signed('0' & s_rv18) - signed('0' & s_mv18);
            s_my19 <= s_my18; s_mu19 <= s_mu18; s_mv19 <= s_mv18;
            s_a19  <= resize(s_al18, 9) + resize(s_al18(7 downto 7), 9);
            -- stage 20: products
            s_py20 <= s_ey19 * signed('0' & s_a19);
            s_pu20 <= s_eu19 * signed('0' & s_a19);
            s_pv20 <= s_ev19 * signed('0' & s_a19);
            s_my20 <= s_my19; s_mu20 <= s_mu19; s_mv20 <= s_mv19;
            -- stage 21: blend; neutral outside the avid (the smoothing's edges)
            v_y  := signed(resize(s_my20, 12)) + resize(shift_right(s_py20, 8), 12);
            v_c1 := signed(resize(s_mu20, 12)) + resize(shift_right(s_pu20, 8), 12);
            v_c2 := signed(resize(s_mv20, 12)) + resize(shift_right(s_pv20, 8), 12);
            if s_avid_sr(19) = '0' then
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            else
                if v_y < 64 then s_y21 <= to_unsigned(64, 10);
                elsif v_y > 1023 then s_y21 <= to_unsigned(1023, 10);
                else s_y21 <= unsigned(v_y(9 downto 0)); end if;
                if v_c1 < 0 then s_u21 <= (others => '0');
                elsif v_c1 > 1023 then s_u21 <= (others => '1');
                else s_u21 <= unsigned(v_c1(9 downto 0)); end if;
                if v_c2 < 0 then s_v21 <= (others => '0');
                elsif v_c2 > 1023 then s_v21 <= (others => '1');
                else s_v21 <= unsigned(v_c2(9 downto 0)); end if;
            end if;
            -- stages 22..23: horizontal [1 2 1]/4 (g1 the centre)
            s_g1y <= s_y21; s_g2y <= s_g1y;
            s_g1u <= s_u21; s_g2u <= s_g1u;
            s_g1v <= s_v21; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y21, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u21, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v21, 12);
            s_y23 <= v_sy(11 downto 2);
            s_u23 <= v_su(11 downto 2);
            s_v23 <= v_sv(11 downto 2);
            -- stage 24: luma toward black, 64 + 13/16 (y - 64) (white ~775);
            -- blanking gate
            v_d  := s_y23 - 64;
            v_yo := s_y23 - shift_right(v_d, 3) - shift_right(v_d, 4);
            if s_avid_sr(22) = '1' then
                s_y24 <= v_yo; s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture stencil;
