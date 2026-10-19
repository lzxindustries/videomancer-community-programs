-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Solarcell: catalogue combo #81 (seed 20260923) -- solarization + mosaic
-- with per-cell colour averaging.  Three knobs each; the switches and the
-- slider decide how they combine.
--
--   K1 Level     fold brightness: the bands top out at 15..80% of white
--   K2 Gain      fold depth 1x .. 16x
--   K3 Rows      cell height 1 .. 64 lines
--   K4 Cell      cell width 4 .. 131 px
--   K5 Mix       mosaic over the picture, 25 .. 100%
--   K6 Offset    threshold step per cell row (staircase; reflects at the ends)
--
--   S7  Chroma   fold keeps the colour / inverts it
--   S8  Order    solarize, then mosaic / mosaic, then solarize
--   S9  Colour   cell colours as averaged / Vivid (double saturation)
--   S10 Grid     black borders along the cells (thicker on big cells)
--   S11 Iso      contour lines where the picture's fold peaks, in negative
--                ink (black over light, light over dark, complementary hue)
--   P12 Fold     the threshold: from the highlights (0%) down to black
--                (100%, the whole picture folds)
--
-- v0.3.2 (2026-10-17): resolution scaling.  The column and row DDAs run in
-- virtual 1080-line units: x steps 1.0 / 1.5 / 2.0 per pixel at 1080 / 720p
-- / SD (SD = a 1440-wide 4:3 window), y steps 1080 / active lines per field
-- line (2 x 1080/486 at 480i), from the video timing ID, so K4 Cell and K3
-- Rows keep their 1080 sizes relative to the screen height (no 2x tall SD
-- cells).  The cell mean still divides by the cell's real pixel count.
-- Rows smaller than a line are stepped over (the K6 staircase still steps
-- once per row).  The Grid is at least 1 px / 1 line; rows under 2 lines on
-- screen (no room for a band) get every N-th line instead, the 1080
-- coverage.  Iso stays at pixel scale.
-- 1080p is bit-identical.
--
-- v0.3.0 (Tier 2).  One cell RAM: each cell row is captured on the line
-- before it (line 0 captures its own row and shows the plain picture), so
-- the mosaic sits on the picture.  Cells are any width (column DDA); a
-- cell's colour is its exact mean (sum x 1/n from a 256-entry table, n = the
-- cell's pixel count, so the line's last part cell is exact too).  The
-- threshold is a phase that reflects inside the luma range (64..939).  Iso =
-- a change of {region, period} against the left pixel or the line above
-- (2 px / 2 lines).  U/V swapped on rows whose hsync->avid parity differs
-- from the capture line's.  HD HDMI bar guard: [1 2 1] on Y/U/V and a
-- highlight knee (white ~770).  Latency 27 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture solarcell of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (27 + 1 = 28 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 28;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 27;
    constant C_NEUTRAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(64, 10)) & std_logic_vector(to_unsigned(512, 10)) &
        std_logic_vector(to_unsigned(512, 10)) & "00";

    -- 1/n for the exact cell mean: {0, e = floor(log2 n), R = round(2^(e+9) / n)}
    type t_recip is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_RECIP : t_recip := (
        x"0200", x"0200", x"0600", x"0555", x"0A00", x"099A", x"0955", x"0925",
        x"0E00", x"0DC7", x"0D9A", x"0D74", x"0D55", x"0D3B", x"0D25", x"0D11",
        x"1200", x"11E2", x"11C7", x"11AF", x"119A", x"1186", x"1174", x"1164",
        x"1155", x"1148", x"113B", x"112F", x"1125", x"111A", x"1111", x"1108",
        x"1600", x"15F0", x"15E2", x"15D4", x"15C7", x"15BB", x"15AF", x"15A4",
        x"159A", x"1590", x"1586", x"157D", x"1574", x"156C", x"1564", x"155D",
        x"1555", x"154E", x"1548", x"1541", x"153B", x"1535", x"152F", x"152A",
        x"1525", x"151F", x"151A", x"1516", x"1511", x"150D", x"1508", x"1504",
        x"1A00", x"19F8", x"19F0", x"19E9", x"19E2", x"19DB", x"19D4", x"19CE",
        x"19C7", x"19C1", x"19BB", x"19B5", x"19AF", x"19AA", x"19A4", x"199F",
        x"199A", x"1995", x"1990", x"198B", x"1986", x"1982", x"197D", x"1979",
        x"1974", x"1970", x"196C", x"1968", x"1964", x"1960", x"195D", x"1959",
        x"1955", x"1952", x"194E", x"194B", x"1948", x"1944", x"1941", x"193E",
        x"193B", x"1938", x"1935", x"1932", x"192F", x"192D", x"192A", x"1927",
        x"1925", x"1922", x"191F", x"191D", x"191A", x"1918", x"1916", x"1913",
        x"1911", x"190F", x"190D", x"190A", x"1908", x"1906", x"1904", x"1902",
        x"1E00", x"1DFC", x"1DF8", x"1DF4", x"1DF0", x"1DED", x"1DE9", x"1DE5",
        x"1DE2", x"1DDE", x"1DDB", x"1DD7", x"1DD4", x"1DD1", x"1DCE", x"1DCA",
        x"1DC7", x"1DC4", x"1DC1", x"1DBE", x"1DBB", x"1DB8", x"1DB5", x"1DB2",
        x"1DAF", x"1DAC", x"1DAA", x"1DA7", x"1DA4", x"1DA1", x"1D9F", x"1D9C",
        x"1D9A", x"1D97", x"1D95", x"1D92", x"1D90", x"1D8D", x"1D8B", x"1D88",
        x"1D86", x"1D84", x"1D82", x"1D7F", x"1D7D", x"1D7B", x"1D79", x"1D76",
        x"1D74", x"1D72", x"1D70", x"1D6E", x"1D6C", x"1D6A", x"1D68", x"1D66",
        x"1D64", x"1D62", x"1D60", x"1D5E", x"1D5D", x"1D5B", x"1D59", x"1D57",
        x"1D55", x"1D54", x"1D52", x"1D50", x"1D4E", x"1D4D", x"1D4B", x"1D49",
        x"1D48", x"1D46", x"1D44", x"1D43", x"1D41", x"1D40", x"1D3E", x"1D3D",
        x"1D3B", x"1D3A", x"1D38", x"1D37", x"1D35", x"1D34", x"1D32", x"1D31",
        x"1D2F", x"1D2E", x"1D2D", x"1D2B", x"1D2A", x"1D29", x"1D27", x"1D26",
        x"1D25", x"1D23", x"1D22", x"1D21", x"1D1F", x"1D1E", x"1D1D", x"1D1C",
        x"1D1A", x"1D19", x"1D18", x"1D17", x"1D16", x"1D15", x"1D13", x"1D12",
        x"1D11", x"1D10", x"1D0F", x"1D0E", x"1D0D", x"1D0B", x"1D0A", x"1D09",
        x"1D08", x"1D07", x"1D06", x"1D05", x"1D04", x"1D03", x"1D02", x"1D01"
    );

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

    -- Vivid: 512 + 2 (c - 512), clamped to 0..1023 (bit slices, no adder)
    function f_vivid(c : unsigned(9 downto 0)) return unsigned is
    begin
        case c(9 downto 8) is
            when "00" => return to_unsigned(0, 10);
            when "11" => return to_unsigned(1023, 10);
            when others => return c(9) & c(7 downto 0) & '0';
        end case;
    end function;

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

    -- cell RAM, one bank: {Y, U, V, capture-line chroma parity, 0}
    type t_cell is array (0 to 511) of std_logic_vector(31 downto 0);
    signal s_cell : t_cell := (others => C_NEUTRAL);
    signal s_crd  : std_logic_vector(31 downto 0) := (others => '0');
    -- iso line buffer: {in region, period(1:0), contour} of the line above
    type t_lb is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal s_lb   : t_lb := (others => (others => '0'));
    signal s_lbrd : std_logic_vector(3 downto 0) := (others => '0');
    -- both RAMs read an address 3+ clocks ahead of its write: no collisions
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_cell : signal is true;
    attribute no_rw_check of s_lb : signal is true;

    -- controls (registered locally)
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_kp : unsigned(9 downto 0) := (others => '0');
    signal s_sw_chroma, s_sw_vivid, s_sw_grid, s_sw_iso : std_logic := '0';
    signal s_en_a, s_en_b : std_logic := '0';

    -- per frame
    signal s_saw   : std_logic := '0';
    signal s_fstep : unsigned(2 downto 0) := "111";
    signal s_kpg   : unsigned(13 downto 0) := (others => '0');            -- P12 glide, 10.4
    signal s_dkp   : signed(14 downto 0) := (others => '0');
    signal s_h1, s_h3, s_h4    : unsigned(9 downto 0) := (others => '0');  -- hysteresis holds
    signal s_dh1, s_dh3, s_dh4 : signed(10 downto 0) := (others => '0');
    signal s_wm1  : unsigned(7 downto 0) := to_unsigned(7, 8);        -- cell width - 1
    signal s_nm1  : unsigned(5 downto 0) := to_unsigned(3, 6);        -- cell height - 1
    signal s_tx   : unsigned(2 downto 0) := "001";                    -- grid px
    signal s_ty   : unsigned(2 downto 0) := "001";                    -- grid lines
    signal s_ga   : unsigned(12 downto 0) := (others => '0');
    signal s_gain : unsigned(11 downto 0) := to_unsigned(1456, 12);   -- Q8
    signal s_la   : unsigned(16 downto 0) := (others => '0');
    signal s_level : unsigned(6 downto 0) := to_unsigned(64, 7);      -- Q6
    signal s_pa   : unsigned(14 downto 0) := (others => '0');
    signal s_pph  : unsigned(9 downto 0) := to_unsigned(400, 10);     -- threshold phase from P12
    signal s_ma, s_mb : unsigned(10 downto 0) := (others => '0');
    signal s_mix  : unsigned(8 downto 0) := to_unsigned(256, 9);      -- /256
    signal s_offr : unsigned(7 downto 0) := (others => '0');
    signal s_base : signed(11 downto 0) := to_signed(64, 12);
    -- resolution scaling: steps (free-running from the timing ID) and the
    -- per-frame virtual geometry
    signal s_stx  : unsigned(2 downto 0) := to_unsigned(2, 3);        -- Q.1 per pixel
    signal s_sty  : unsigned(11 downto 0) := to_unsigned(256, 12);    -- Q4.8 per field line
    signal s_w2   : unsigned(8 downto 0) := to_unsigned(16, 9);       -- cell width, half units
    signal s_w2ms : unsigned(8 downto 0) := to_unsigned(14, 9);       -- ... minus a pixel step
    signal s_gtx  : unsigned(3 downto 0) := to_unsigned(2, 4);        -- grid columns, half units
    signal s_n256 : unsigned(14 downto 0) := to_unsigned(1024, 15);   -- row height, Q.8
    signal s_nlast : unsigned(14 downto 0) := to_unsigned(768, 15);   -- a row's last line starts here
    signal s_gty  : unsigned(14 downto 0) := to_unsigned(256, 15);    -- grid lines, Q.8 (0 = none)
    signal s_gfine : std_logic := '0';                                -- rows under 2 lines: grid every N-th line
    signal s_lmod : unsigned(5 downto 0) := (others => '0');          -- screen line mod N

    -- per line
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line  : unsigned(10 downto 0) := (others => '0');
    signal s_rcy   : unsigned(14 downto 0) := (others => '0');        -- line position in its row, Q.8 virtual
    signal s_rwrap : std_logic := '0';
    signal s_stair : unsigned(14 downto 0) := (others => '0');        -- K6 staircase, 11.4 phase
    signal s_lstep : unsigned(2 downto 0) := "111";
    signal s_ss    : signed(12 downto 0) := (others => '0');
    signal s_tri : unsigned(9 downto 0) := (others => '0');
    signal s_ta  : unsigned(17 downto 0) := (others => '0');
    signal s_thr_next, s_thr : unsigned(9 downto 0) := to_unsigned(406, 10);
    signal s_capl, s_l0, s_rgrid : std_logic := '0';
    signal s_hpar, s_lpar : std_logic := '0';

    -- input + input-side solar (out at d9)
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_ya, s_ua, s_va : unsigned(9 downto 0);
    signal s_rega : std_logic;
    signal s_pera : unsigned(1 downto 0);

    -- write side: column DDA (d8), flags (d9), sums (d10), 1/n (d11, d12),
    -- products (d13, d14), cell word (d15)
    signal s_wcc : unsigned(8 downto 0) := (others => '0');           -- position in the cell, half units
    signal s_wpc : unsigned(7 downto 0) := (others => '0');           -- pixels before this one in the cell
    signal s_wci : unsigned(8 downto 0) := (others => '0');
    signal s_wx  : unsigned(10 downto 0) := (others => '0');
    signal s_wst9, s_wlast9 : std_logic := '0';
    signal s_wn9, s_wn10 : unsigned(7 downto 0) := (others => '0');   -- pixels so far in the cell
    signal s_wci9 : unsigned(8 downto 0) := (others => '0');
    signal s_wx9  : unsigned(10 downto 0) := (others => '0');
    signal s_sy, s_su, s_sv : unsigned(17 downto 0) := (others => '0');   -- running sums (d10)
    signal s_ty11, s_tu11, s_tv11, s_ty12, s_tu12, s_tv12 : unsigned(17 downto 0) := (others => '0');
    signal s_rom11 : std_logic_vector(15 downto 0) := (others => '0');
    signal s_r12  : unsigned(9 downto 0) := (others => '0');
    type t_e is array (12 to 14) of std_logic_vector(2 downto 0);   -- slv: unsigned & literal stays unambiguous
    signal s_e : t_e := (others => (others => '0'));
    signal s_hy13, s_hu13, s_hv13, s_ly13, s_lu13, s_lv13 : unsigned(22 downto 0) := (others => '0');
    signal s_py14, s_pu14, s_pv14 : unsigned(27 downto 0) := (others => '0');
    type t_wen is array (10 to 15) of std_logic;
    signal s_wen : t_wen := (others => '0');
    type t_wa is array (10 to 15) of std_logic_vector(8 downto 0);
    signal s_wa : t_wa := (others => (others => '0'));
    signal s_wd15 : std_logic_vector(31 downto 0) := C_NEUTRAL;

    -- read side: column DDA (d7), RAM (d8), fabric (d9), select (d10), mix (d11..d14)
    signal s_rcc : unsigned(8 downto 0) := (others => '0');           -- position in the cell, half units
    signal s_rci : unsigned(8 downto 0) := (others => '0');
    signal s_my9, s_mu9, s_mv9 : unsigned(9 downto 0) := (others => '0');
    signal s_mp9 : std_logic := '0';
    signal s_my10, s_mu10, s_mv10, s_dy10, s_du10, s_dv10 : unsigned(9 downto 0) := (others => '0');
    signal s_ey11, s_eu11, s_ev11 : signed(10 downto 0) := (others => '0');
    signal s_dy11, s_du11, s_dv11 : unsigned(9 downto 0) := (others => '0');
    signal s_hy12, s_hu12, s_hv12 : signed(15 downto 0) := (others => '0');   -- diff * mix(8:5)
    signal s_ly12, s_lu12, s_lv12 : signed(16 downto 0) := (others => '0');   -- diff * mix(4:0)
    signal s_dy12, s_du12, s_dv12 : unsigned(9 downto 0) := (others => '0');
    signal s_xy13, s_xu13, s_xv13 : signed(20 downto 0) := (others => '0');
    signal s_dy13, s_du13, s_dv13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    -- output-side solar (out at d22)
    signal s_yb, s_ub, s_vb : unsigned(9 downto 0);

    -- grid column flag (d8..d22); iso contour (d10..d22)
    type t_gx is array (8 to 22) of std_logic;
    signal s_gx : t_gx := (others => '0');
    signal s_k10, s_k11, s_k12 : std_logic_vector(2 downto 0) := (others => '0');
    signal s_x10, s_x11 : unsigned(10 downto 0) := (others => '0');
    signal s_ab11 : std_logic_vector(3 downto 0) := (others => '0');
    signal s_c12, s_c13, s_ac12 : std_logic := '0';
    signal s_lbwe12 : std_logic := '0';
    signal s_lbwa12 : unsigned(10 downto 0) := (others => '0');
    signal s_lbwd12 : std_logic_vector(3 downto 0) := (others => '0');
    type t_iso is array (13 to 22) of std_logic;
    signal s_iso : t_iso := (others => '0');

    -- output: overlay (d23), clamp + neutral (d24), [1 2 1] (d26), knee + gate (d27)
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := (others => '0');
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := (others => '0');
    signal s_g1y, s_g1u, s_g1v, s_g2y, s_g2u, s_g2v : unsigned(9 downto 0) := (others => '0');
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := (others => '0');
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');   -- s_avid_sr(k) = avid at d(k+1)

begin

    p_ctrl : process(clk)
        variable v_t : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            s_stx <= f_vstep_x(registers_in(8)(3 downto 0));
            s_sty <= f_vstep_y(registers_in(8)(3 downto 0));
            s_k1 <= unsigned(registers_in(0)); s_k2 <= unsigned(registers_in(1));
            s_k3 <= unsigned(registers_in(2)); s_k4 <= unsigned(registers_in(3));
            s_k5 <= unsigned(registers_in(4)); s_k6 <= unsigned(registers_in(5));
            s_kp <= unsigned(registers_in(7));
            s_sw_chroma <= registers_in(6)(0);
            s_en_a <= not registers_in(6)(1); s_en_b <= registers_in(6)(1);
            s_sw_vivid  <= registers_in(6)(2);
            s_sw_grid   <= registers_in(6)(3);
            s_sw_iso    <= registers_in(6)(4);

            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;

            -- chroma phase: hsync->avid parity (drape v1.3)
            s_hpar <= not s_hpar;

            -- per frame, one op per state: P12 glide, hysteresis (K1, K3,
            -- K4), geometry, gain, level, threshold phase, mix
            case to_integer(s_fstep) is
                when 0 =>
                    s_dkp <= signed('0' & s_kp & "0000") - signed('0' & s_kpg);
                    s_dh1 <= signed('0' & s_k1) - signed('0' & s_h1);
                    s_dh3 <= signed('0' & s_k3) - signed('0' & s_h3);
                    s_dh4 <= signed('0' & s_k4) - signed('0' & s_h4);
                when 1 =>
                    v_t := signed('0' & s_kpg) + shift_right(s_dkp, 2);
                    s_kpg <= unsigned(v_t(13 downto 0));
                    if s_dh1 > 3 or s_dh1 < -3 then s_h1 <= s_k1; end if;
                    if s_dh3 > 3 or s_dh3 < -3 then s_h3 <= s_k3; end if;
                    if s_dh4 > 3 or s_dh4 < -3 then s_h4 <= s_k4; end if;
                when 2 =>
                    s_wm1 <= to_unsigned(3, 8) + resize(s_h4(9 downto 3), 8);   -- W = 4 + K4/8
                    s_nm1 <= s_h3(9 downto 4);                                -- N = 1 + K3/16
                    s_ga  <= to_unsigned(256, 13) + resize(s_k2 & "00", 13);
                    s_la  <= resize(s_h1 & "000000", 17) + resize(s_h1 & "000", 17);   -- 72 K1
                    s_pa  <= resize(s_kpg(13 downto 4) & "0000", 15) + resize(s_kpg(13 downto 4) & "000", 15);   -- 24 P
                    s_ma  <= resize(s_k5(9 downto 2), 11) + 1;
                    s_offr <= s_k6(9 downto 2);
                when 3 =>
                    s_gain <= resize(s_ga - resize(s_k2(9 downto 2), 13), 12);   -- 256 + 3.75 K2: 1x .. 16x
                    s_level <= resize(s_la(16 downto 10), 7) + 16;                -- 16 .. 87 (/64)
                    s_pa <= s_pa + resize(s_kpg(13 downto 4), 15);                -- 25 P
                    s_mb <= s_ma + shift_left(s_ma, 1);
                    s_tx <= resize(s_wm1(7 downto 5), 3) + 1;
                    if s_nm1 < 3 then s_ty <= "000"; else s_ty <= resize(s_nm1(5 downto 4), 3) + 1; end if;
                when 4 =>
                    s_pph <= to_unsigned(800, 10) - s_pa(14 downto 5);             -- 800 - 25 P / 32: 800 .. 1
                    s_mix <= resize(s_mb(10 downto 2), 9) + 64;                    -- 64 .. 256
                    -- virtual geometry: cell width in half units, row height Q.8
                    s_w2   <= shift_left(resize(s_wm1, 9) + 1, 1);
                    s_n256 <= shift_left(resize(s_nm1, 15) + 1, 8);
                    if shift_left(resize(s_tx, 4), 1) < resize(s_stx, 4) then
                        s_gtx <= resize(s_stx, 4);                                 -- at least 1 px
                    else
                        s_gtx <= shift_left(resize(s_tx, 4), 1);
                    end if;
                when 5 =>
                    s_w2ms <= s_w2 - s_stx;
                    if s_n256 > resize(s_sty, 15) then
                        s_nlast <= s_n256 - s_sty;
                    else
                        s_nlast <= (others => '0');                                 -- rows under a line
                    end if;
                    -- grid lines: at least 1 line; rows under 2 lines on screen
                    -- can't show a band, so every N-th screen line (the same
                    -- coverage, ty = 1 there)
                    s_gfine <= '0';
                    if s_ty = 0 then
                        s_gty <= (others => '0');
                    elsif s_n256 < shift_left(resize(s_sty, 15), 1) then
                        s_gty <= (others => '0'); s_gfine <= '1';
                    elsif shift_left(resize(s_ty, 15), 8) < resize(s_sty, 15) then
                        s_gty <= resize(s_sty, 15);
                    else
                        s_gty <= shift_left(resize(s_ty, 15), 8);
                    end if;
                when others => null;
            end case;
            if s_fstep /= 7 then s_fstep <= s_fstep + 1; end if;

            -- per line (from hsync, idempotent): threshold phase = P12 phase
            -- + staircase, reflected into 0..1023, then mapped onto the luma
            -- range 64 + 0.855 * phase
            case to_integer(s_lstep) is
                when 0 =>
                    s_ss <= signed(resize(s_pph, 13)) + signed(resize(s_stair(14 downto 4), 13));
                when 1 =>
                    if s_ss(10) = '1' then s_tri <= not unsigned(s_ss(9 downto 0));
                    else s_tri <= unsigned(s_ss(9 downto 0)); end if;
                when 2 => s_ta <= resize(s_tri & "00000000", 18) - resize(s_tri & "00000", 18);  -- 219 tri
                when 3 => s_ta <= s_ta - resize(s_tri & "00", 18);
                when 4 => s_ta <= s_ta - resize(s_tri, 18);
                when 5 => s_thr_next <= s_ta(17 downto 8) + 64;
                when others => null;
            end case;
            if s_lstep /= 7 then s_lstep <= s_lstep + 1; end if;

            -- end of an active line: advance the cell-row DDA (virtual lines,
            -- Q8); then, a clock at a time, step over every row boundary it
            -- passed (rows under a line can pass several) with the staircase
            if s_rwrap = '1' then
                if s_rcy >= s_n256 then
                    s_rcy <= s_rcy - s_n256; s_stair <= s_stair + s_offr;
                else
                    s_rwrap <= '0';
                end if;
            end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line <= s_line + 1;
                s_rcy <= s_rcy + s_sty;
                if s_lmod >= s_nm1 then s_lmod <= (others => '0'); else s_lmod <= s_lmod + 1; end if;
                s_rwrap <= '1';
            end if;
            -- start of an active line: latch the line's threshold and flags
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_saw <= '1';
                s_thr <= s_thr_next;
                s_lpar <= s_hpar;
                if s_line = 0 then s_l0 <= '1'; else s_l0 <= '0'; end if;
                -- capture the next cell row on its row's last line (line 0: its own)
                if s_rcy >= s_nlast or s_line = 0 then s_capl <= '1'; else s_capl <= '0'; end if;
                if s_rcy < s_gty or (s_gfine = '1' and s_lmod = 0) then s_rgrid <= '1'; else s_rgrid <= '0'; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0'); s_hpar <= '0';
            end if;
            -- resets are idempotent (serrated vsync); the frame sequencer
            -- start is guarded by saw-active
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_rcy <= (others => '0'); s_rwrap <= '0'; s_stair <= (others => '0');
                s_lmod <= (others => '0');
                if s_saw = '1' then s_fstep <= (others => '0'); s_saw <= '0'; end if;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- input-side solar: d1 -> d9 (region and period always reported: iso)
    solar_a : entity work.solar_fold
        port map (clk => clk, y_in => s_in_y, u_in => s_in_u, v_in => s_in_v,
                  thr => s_thr, gain => s_gain, level => s_level, base => s_base,
                  fold_c => s_sw_chroma, enable => s_en_a,
                  y_out => s_ya, u_out => s_ua, v_out => s_va,
                  region => s_rega, period => s_pera);

    -- write side: exact cell means of the d9 picture
    p_wr : process(clk)
        variable v_sy, v_su, v_sv : unsigned(17 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- d8: column DDA (driven by the d8 avid), virtual half units
            if s_avid_sr(7) = '1' then
                if s_wcc >= s_w2ms then
                    s_wcc <= s_wcc + s_stx - s_w2; s_wci <= s_wci + 1; s_wpc <= (others => '0');
                else
                    s_wcc <= s_wcc + s_stx; s_wpc <= s_wpc + 1;
                end if;
                s_wx <= s_wx + 1;
            else
                s_wcc <= (others => '0'); s_wci <= (others => '0'); s_wx <= (others => '0');
                s_wpc <= (others => '0');
            end if;
            -- d9: cell start / last flags, pixel count
            if s_wcc < s_stx then s_wst9 <= '1'; else s_wst9 <= '0'; end if;
            if s_wcc >= s_w2ms then s_wlast9 <= '1'; else s_wlast9 <= '0'; end if;
            s_wn9 <= s_wpc + 1;
            s_wci9 <= s_wci; s_wx9 <= s_wx;
            -- d10: running cell sums
            if s_wst9 = '1' then
                v_sy := resize(s_ya, 18); v_su := resize(s_ua, 18); v_sv := resize(s_va, 18);
            else
                v_sy := s_sy + s_ya; v_su := s_su + s_ua; v_sv := s_sv + s_va;
            end if;
            s_sy <= v_sy; s_su <= v_su; s_sv <= v_sv;
            s_wn10 <= s_wn9;
            -- write at the cell's last pixel, or at the line's last (part) cell
            s_wen(10) <= s_capl and s_avid_sr(8) and (s_wlast9 or not s_avid_sr(7));
            s_wa(10) <= std_logic_vector(s_wci9);
            for i in 11 to 15 loop s_wen(i) <= s_wen(i - 1); s_wa(i) <= s_wa(i - 1); end loop;
            -- d11: 1/n table (may land in an EBR: re-registered at d12)
            s_rom11 <= C_RECIP(to_integer(s_wn10));
            s_ty11 <= s_sy; s_tu11 <= s_su; s_tv11 <= s_sv;
            -- d12
            s_r12 <= unsigned(s_rom11(9 downto 0)); s_e(12) <= s_rom11(12 downto 10);
            s_ty12 <= s_ty11; s_tu12 <= s_tu11; s_tv12 <= s_tv11;
            s_e(13) <= s_e(12); s_e(14) <= s_e(13);
            -- d13: sum * 1/n, the reciprocal split hi/lo
            s_hy13 <= s_ty12 * s_r12(9 downto 5); s_ly13 <= s_ty12 * s_r12(4 downto 0);
            s_hu13 <= s_tu12 * s_r12(9 downto 5); s_lu13 <= s_tu12 * s_r12(4 downto 0);
            s_hv13 <= s_tv12 * s_r12(9 downto 5); s_lv13 <= s_tv12 * s_r12(4 downto 0);
            -- d14
            s_py14 <= shift_left(resize(s_hy13, 28), 5) + resize(s_ly13, 28);
            s_pu14 <= shift_left(resize(s_hu13, 28), 5) + resize(s_lu13, 28);
            s_pv14 <= shift_left(resize(s_hv13, 28), 5) + resize(s_lv13, 28);
            -- d15: mean = product >> (e + 9)
            case to_integer(unsigned(s_e(14))) is
                when 0 => v_y := s_py14(18 downto 9);  v_u := s_pu14(18 downto 9);  v_v := s_pv14(18 downto 9);
                when 1 => v_y := s_py14(19 downto 10); v_u := s_pu14(19 downto 10); v_v := s_pv14(19 downto 10);
                when 2 => v_y := s_py14(20 downto 11); v_u := s_pu14(20 downto 11); v_v := s_pv14(20 downto 11);
                when 3 => v_y := s_py14(21 downto 12); v_u := s_pu14(21 downto 12); v_v := s_pv14(21 downto 12);
                when 4 => v_y := s_py14(22 downto 13); v_u := s_pu14(22 downto 13); v_v := s_pv14(22 downto 13);
                when 5 => v_y := s_py14(23 downto 14); v_u := s_pu14(23 downto 14); v_v := s_pv14(23 downto 14);
                when 6 => v_y := s_py14(24 downto 15); v_u := s_pu14(24 downto 15); v_v := s_pv14(24 downto 15);
                when others => v_y := s_py14(25 downto 16); v_u := s_pu14(25 downto 16); v_v := s_pv14(25 downto 16);
            end case;
            s_wd15 <= std_logic_vector(v_y) & std_logic_vector(v_u) & std_logic_vector(v_v) & s_lpar & '0';
        end if;
    end process p_wr;

    -- cell RAM: written at d15 (address 8 px behind the read), read at d7
    p_cell : process(clk)
    begin
        if rising_edge(clk) then
            if s_wen(15) = '1' then s_cell(to_integer(unsigned(s_wa(15)))) <= s_wd15; end if;
            s_crd <= s_cell(to_integer(s_rci));
        end if;
    end process p_cell;

    -- iso line buffer: read at d9, written at d12 (3 px behind)
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            if s_lbwe12 = '1' then s_lb(to_integer(s_lbwa12)) <= s_lbwd12; end if;
            s_lbrd <= s_lb(to_integer(s_wx9));
        end if;
    end process p_lb;

    p_rd : process(clk)
        variable v_c : std_logic;
        variable v_u, v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- d7: column DDA (driven by the d7 avid) = read address
            if s_avid_sr(6) = '1' then
                if s_rcc >= s_w2ms then
                    s_rcc <= s_rcc + s_stx - s_w2; s_rci <= s_rci + 1;
                else
                    s_rcc <= s_rcc + s_stx;
                end if;
            else
                s_rcc <= (others => '0'); s_rci <= (others => '0');
            end if;
            -- d8: grid column (the cell's first tx units, at least 1 px)
            if s_rcc < resize(s_gtx, 9) then s_gx(8) <= '1'; else s_gx(8) <= '0'; end if;
            for i in 9 to 22 loop s_gx(i) <= s_gx(i - 1); end loop;
            -- d9: RAM out -> fabric
            s_my9 <= unsigned(s_crd(31 downto 22)); s_mu9 <= unsigned(s_crd(21 downto 12));
            s_mv9 <= unsigned(s_crd(11 downto 2));  s_mp9 <= s_crd(1);
            -- d10: the mosaic, U/V swapped where the capture line's chroma
            -- phase differs from this line's, Vivid; line 0 shows the plain picture
            s_dy10 <= s_ya; s_du10 <= s_ua; s_dv10 <= s_va;
            if s_mp9 /= s_lpar then v_u := s_mv9; v_v := s_mu9; else v_u := s_mu9; v_v := s_mv9; end if;
            if s_sw_vivid = '1' then v_u := f_vivid(v_u); v_v := f_vivid(v_v); end if;
            if s_l0 = '1' then
                s_my10 <= s_ya; s_mu10 <= s_ua; s_mv10 <= s_va;
            else
                s_my10 <= s_my9; s_mu10 <= v_u; s_mv10 <= v_v;
            end if;
            -- d11: differences toward the mosaic
            s_ey11 <= signed(resize(s_my10, 11)) - signed(resize(s_dy10, 11));
            s_eu11 <= signed(resize(s_mu10, 11)) - signed(resize(s_du10, 11));
            s_ev11 <= signed(resize(s_mv10, 11)) - signed(resize(s_dv10, 11));
            s_dy11 <= s_dy10; s_du11 <= s_du10; s_dv11 <= s_dv10;
            -- d12: diff * mix, the mix split hi/lo
            s_hy12 <= s_ey11 * signed('0' & s_mix(8 downto 5));
            s_hu12 <= s_eu11 * signed('0' & s_mix(8 downto 5));
            s_hv12 <= s_ev11 * signed('0' & s_mix(8 downto 5));
            s_ly12 <= s_ey11 * signed('0' & s_mix(4 downto 0));
            s_lu12 <= s_eu11 * signed('0' & s_mix(4 downto 0));
            s_lv12 <= s_ev11 * signed('0' & s_mix(4 downto 0));
            s_dy12 <= s_dy11; s_du12 <= s_du11; s_dv12 <= s_dv11;
            -- d13
            s_xy13 <= shift_left(resize(s_hy12, 21), 5) + resize(s_ly12, 21);
            s_xu13 <= shift_left(resize(s_hu12, 21), 5) + resize(s_lu12, 21);
            s_xv13 <= shift_left(resize(s_hv12, 21), 5) + resize(s_lv12, 21);
            s_dy13 <= s_dy12; s_du13 <= s_du12; s_dv13 <= s_dv12;
            -- d14
            s_y14 <= f_clamp10(signed(resize(s_dy13, 14)) + resize(shift_right(s_xy13, 8), 14));
            s_u14 <= f_clamp10(signed(resize(s_du13, 14)) + resize(shift_right(s_xu13, 8), 14));
            s_v14 <= f_clamp10(signed(resize(s_dv13, 14)) + resize(shift_right(s_xv13, 8), 14));

            -- iso: key = {in region, period mod 4} of the input-side fold (d9)
            if s_rega = '1' then s_k10 <= '1' & std_logic_vector(s_pera); else s_k10 <= "000"; end if;
            s_k11 <= s_k10; s_k12 <= s_k11;
            s_x10 <= s_wx9; s_x11 <= s_x10;
            s_ab11 <= s_lbrd;
            -- d12: contour = key differs from the left pixel or the line above
            v_c := '0';
            if s_k11 /= s_k12 and s_avid_sr(11) = '1' then v_c := '1'; end if;
            if s_k11 /= s_ab11(3 downto 1) and s_l0 = '0' then v_c := '1'; end if;
            s_c12 <= v_c; s_ac12 <= s_ab11(0);
            s_lbwe12 <= s_avid_sr(10); s_lbwa12 <= s_x11; s_lbwd12 <= s_k11 & v_c;
            -- d13: 2 px wide, 2 lines tall
            s_c13 <= s_c12;
            s_iso(13) <= s_sw_iso and (s_c12 or s_c13 or s_ac12);
            for i in 14 to 22 loop s_iso(i) <= s_iso(i - 1); end loop;
        end if;
    end process p_rd;

    -- output-side solar: d14 -> d22
    solar_b : entity work.solar_fold
        port map (clk => clk, y_in => s_y14, u_in => s_u14, v_in => s_v14,
                  thr => s_thr, gain => s_gain, level => s_level, base => s_base,
                  fold_c => s_sw_chroma, enable => s_en_b,
                  y_out => s_yb, u_out => s_ub, v_out => s_vb,
                  region => open, period => open);

    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- d23: grid black; iso in negative ink (black over light, light
            -- over dark, complementary hue)
            if s_sw_grid = '1' and (s_gx(22) = '1' or s_rgrid = '1') then
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            elsif s_iso(22) = '1' then
                if s_yb >= 502 then s_y23 <= to_unsigned(64, 10); else s_y23 <= to_unsigned(768, 10); end if;
                s_u23 <= not s_ub; s_v23 <= not s_vb;
            else
                s_y23 <= s_yb; s_u23 <= s_ub; s_v23 <= s_vb;
            end if;
            -- d24: luma 64..940; neutral outside the avid (the smoothing's edges)
            if s_avid_sr(22) = '1' then
                if s_y23 < 64 then s_y24 <= to_unsigned(64, 10);
                elsif s_y23 > 940 then s_y24 <= to_unsigned(940, 10);
                else s_y24 <= s_y23; end if;
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
            -- d25..d26: horizontal [1 2 1]/4 on Y/U/V (HD HDMI bar guard)
            s_g1y <= s_y24; s_g2y <= s_g1y;
            s_g1u <= s_u24; s_g2u <= s_g1u;
            s_g1v <= s_v24; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y24, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u24, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v24, 12);
            s_y26 <= v_sy(11 downto 2); s_u26 <= v_su(11 downto 2); s_v26 <= v_sv(11 downto 2);
            -- d27: highlight knee 600 + (y - 600)/2 (white ~770); gate
            if s_avid_sr(C_LAT - 2) = '1' then
                if s_y26 > 600 then
                    s_y27 <= to_unsigned(300, 10) + shift_right(s_y26, 1);
                else
                    s_y27 <= s_y26;
                end if;
                s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    s_io_align_in.y       <= std_logic_vector(s_y27);
    s_io_align_in.u       <= std_logic_vector(s_u27);
    s_io_align_in.v       <= std_logic_vector(s_v27);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (27 -> 28).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture solarcell;
