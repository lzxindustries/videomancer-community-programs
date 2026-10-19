-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- sugarcoat.vhd
--
-- v1.2.2 (2026-10-17): resolution scaling.  Every screen-space coordinate --
-- the centred projections (stripes, bands, grooves, scallops), the cell
-- engine (K3 cells, chips, beans, dots, checks, waffle grid) and the garnish
-- cells -- runs in virtual 1080-line units, stepped per pixel / per field
-- line from the video timing ID (registers_in(8)): 1.0 at 1080p
-- (bit-identical), 1.5 at 720p, and at SD 2.0 across (a 1440-wide 4:3
-- window) and 1080/486 per frame line.  Stripes and cells keep their size
-- relative to the screen height and stay square on a 4:3 screen (at SD they
-- were 2x wide and, on field lines, 4.4x tall).  The centre is measured in
-- virtual units.  Thin features keep at least one sample per step: the
-- chocolate grooves widen to 2 units when the x step exceeds one unit at
-- the 8 px scale, and the garnish rows widen to 5 units where a field line
-- spans more than 2.  The gloss / outline gradient and the Clean run length
-- are pixel-scale filters and stay in pixels.
--
-- SUGARCOAT -- video re-rendered as candy: a 1:1 COLOUR -> CANDY-TEXTURE map.
--
-- Each pixel is classified into one of 16 CLASSES: 0-7 = luma band (Luma
-- map, and neutral pixels in the Chroma map), 8-15 = hue octant (Chroma map).
-- A THEME table maps classes to a bank of 16 flat screen-space candy
-- materials: every theme's luma ramp is sorted dark -> bright by the
-- texture's own brightness (dark dense textures land in shadows, bright ones
-- in highlights) and its hue map follows colour (red -> cane, amber ->
-- cookie / gingerbread / waffle, green -> mint, pink -> icing / Battenberg,
-- cool hues -> the multi-colour textures).  K1 = 8 themes x pair-swap x
-- tone-shift, S9 Bank B = 8 more themes.  NO generated geometry, nothing
-- self-animates -- the picture alone decides where each candy appears.
--
-- Texture bank:  0 dark chocolate bar   1 cookies & cream   2 choc-chip cookie
--                3 candy cane           4 mint stripes      5 piped icing
--                6 rainbow taffy        7 frosting+confetti 8 Battenberg
--                9 liquorice allsorts  10 waffle cone      11 jelly beans
--               12 gingerbread         13 liquorice twist  14 nonpareils
--               15 white chocolate
--
-- Pipeline (streaming, C_LATENCY = 32, EBR: gloss line RAM, class line RAM, theme ROM):
--   Classify: E1 in (U/V swap) -> G1..G3 CONTRAST/LEVEL proc-amp -> E2 |du|,
--             |dv|, sat, octant -> E3 class -> CL CLEAN (run-length speck
--             filter with look-ahead + "matches the line above" fast accept,
--             class line RAM) -> variable-tap SR (fixed total delay) ->
--             MAP (class adjust -> theme ROM -> reg) -> PH1/PH2 -> CO -> OUT.
--   Geometry: three static projections of (dx,dy) (both diagonals, wavy
--             horizontal) + a plain horizontal one -> stripe / band bits; the
--             cell engine (chips, dots, beans, checks, grid) runs on the
--             free-running pixel/line counters.  All screen-space: the
--             pattern stage is sampled ~28 px ahead of the class stream,
--             which is just a constant texture offset (invisible).
--   Gloss:    line RAM -> 2D luma-gradient emboss -> specular sheen g_add,
--             plus the OUTLINE edge flag; both delayed to the compose stage.
--   Output:   gloss + shade add -> clamp -> blanking-gated register.
-- No multiplies anywhere (HX4K has no DSP): shifts, compares and slices.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture sugarcoat of program_top is
    -- data_in -> data_out latency is 32 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 32;

    constant C_LATENCY : integer := 32;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- Folded TRIANGLE wave (10-bit phase, 1024 = one cycle, output +/-504),
    -- used by the icing / gingerbread scallop projection.
    function f_tri(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then v_i := a(7 downto 2);
        else               v_i := not a(7 downto 2); end if;
        v_t := signed(resize(v_i, 7) & "000");        -- v_i * 8 (0..504)
        if a(9) = '1' then return -v_t; else return v_t; end if;
    end function;

    -- cheap 16-bit hash for cell / garnish placement (xor-rotate-add).
    -- NOTE: slices y(9 downto 4) -- pass the y cell PRE-SHIFTED into 9:4.
    function f_hash(x, y : unsigned(9 downto 0);
                    seed : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := (x & y(9 downto 4)) xor seed;
        h := h xor rotate_left(h, 7);
        h := h + rotate_left(h, 3);
        h := h xor (x"00" & h(15 downto 8));
        return h;
    end function;

    ----------------------------------------------------------------------
    -- fixed RAINBOW ramp (standard BT.601, 10-bit): red, orange, yellow,
    -- green, cyan, blue, violet, magenta.  Taffy stripes + confetti/jimmies.
    ----------------------------------------------------------------------
    type t_rb is array (0 to 7) of unsigned(9 downto 0);
    constant C_RB_Y : t_rb := (to_unsigned(374,10), to_unsigned(588,10), to_unsigned(786,10), to_unsigned(535,10),
                               to_unsigned(586,10), to_unsigned(352,10), to_unsigned(362,10), to_unsigned(486,10));
    constant C_RB_U : t_rb := (to_unsigned(403,10), to_unsigned(276,10), to_unsigned(201,10), to_unsigned(367,10),
                               to_unsigned(621,10), to_unsigned(779,10), to_unsigned(753,10), to_unsigned(606,10));
    constant C_RB_V : t_rb := (to_unsigned(836,10), to_unsigned(730,10), to_unsigned(581,10), to_unsigned(302,10),
                               to_unsigned(242,10), to_unsigned(415,10), to_unsigned(599,10), to_unsigned(767,10));

    constant C_WHITE_Y : unsigned(9 downto 0) := to_unsigned(1000, 10);

    ----------------------------------------------------------------------
    -- THEME ROM: 16 themes x 16 classes -> texture.  Per theme: entries 0-7
    -- = luma ramp (dark -> bright, sorted by texture brightness), 8-15 = hue
    -- map (green, teal, cyan, blue, red, amber, pink, violet).  Textures:
    -- 0 choc 1 cream 2 cookie 3 cane 4 mint 5 icing 6 taffy 7 frosting
    -- 8 Battenberg 9 allsorts A waffle B beans C gingerbread D twist
    -- E nonpareils F white choc.  Address = bank & theme(2:0) & class(3:0).
    ----------------------------------------------------------------------
    type t_tmap is array (0 to 255) of std_logic_vector(3 downto 0);
    constant C_TMAP : t_tmap := (
        x"0", x"1", x"2", x"6", x"3", x"4", x"5", x"7", x"4", x"1", x"7", x"6", x"3", x"2", x"5", x"0",  -- Classic
        x"0", x"E", x"2", x"C", x"A", x"8", x"F", x"7", x"4", x"F", x"7", x"B", x"3", x"C", x"8", x"0",  -- Bakery
        x"D", x"E", x"9", x"C", x"A", x"8", x"F", x"5", x"4", x"D", x"B", x"E", x"9", x"A", x"5", x"0",  -- Liquorice
        x"D", x"1", x"9", x"6", x"3", x"B", x"4", x"7", x"4", x"E", x"7", x"6", x"B", x"2", x"8", x"9",  -- Sweet Shop
        x"0", x"D", x"E", x"1", x"2", x"A", x"F", x"5", x"A", x"E", x"F", x"0", x"C", x"2", x"1", x"D",  -- Cocoa
        x"0", x"D", x"9", x"6", x"3", x"F", x"4", x"7", x"4", x"D", x"F", x"6", x"3", x"9", x"8", x"0",  -- Stripes
        x"1", x"9", x"C", x"3", x"8", x"B", x"4", x"5", x"4", x"F", x"7", x"B", x"3", x"8", x"5", x"6",  -- Pastel
        x"E", x"1", x"2", x"A", x"8", x"B", x"5", x"7", x"B", x"1", x"7", x"E", x"2", x"A", x"8", x"0",  -- Bits
        x"D", x"E", x"9", x"C", x"3", x"8", x"4", x"7", x"4", x"E", x"B", x"6", x"9", x"C", x"8", x"D",  -- Classic B
        x"0", x"1", x"C", x"A", x"3", x"F", x"5", x"7", x"F", x"1", x"7", x"A", x"C", x"2", x"5", x"0",  -- Bakery B
        x"0", x"D", x"E", x"1", x"9", x"2", x"6", x"C", x"E", x"D", x"1", x"0", x"9", x"C", x"2", x"A",  -- Dark
        x"2", x"6", x"3", x"8", x"B", x"F", x"5", x"7", x"4", x"B", x"7", x"F", x"3", x"8", x"5", x"6",  -- Bright
        x"0", x"D", x"1", x"A", x"3", x"F", x"5", x"7", x"1", x"D", x"F", x"E", x"3", x"A", x"9", x"0",  -- Mono
        x"E", x"9", x"6", x"C", x"B", x"8", x"5", x"7", x"B", x"E", x"6", x"7", x"9", x"8", x"5", x"0",  -- Multi
        x"0", x"E", x"1", x"2", x"C", x"A", x"8", x"F", x"C", x"E", x"F", x"0", x"9", x"A", x"1", x"D",  -- Bakery Dark
        x"0", x"D", x"9", x"6", x"3", x"8", x"F", x"4", x"4", x"0", x"6", x"D", x"3", x"9", x"8", x"F"  -- Stripes B
    );

    ----------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
    ----------------------------------------------------------------------
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

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_k1  : unsigned(9 downto 0) := (others => '0');
    signal s_k2  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_k3  : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal s_k4  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k5  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k6  : unsigned(9 downto 0) := to_unsigned(640, 10);

    signal s_outline : std_logic := '0';                       -- S7
    signal s_shade   : std_logic := '0';                       -- S8
    signal s_bank    : std_logic := '0';                       -- S9  bank A/B
    signal s_sprk_on : std_logic := '1';                       -- S10 sprinkles
    signal s_lumamap : std_logic := '0';                       -- S11 map mode

    -- K1 THEME: theme (with S9 bank), pair-swap and tone-shift variants
    signal s_theme : unsigned(2 downto 0) := (others => '0');
    signal s_swap  : std_logic := '0';
    signal s_tone  : std_logic := '0';
    -- K2 CLEAN: minimum run length (px) a class must hold to be accepted
    signal s_ncl : integer range 1 to 16 := 2;
    signal s_tap : integer range 1 to 16 := 15;               -- 17 - s_ncl
    -- K4 CONTRAST gain step (4 = unity) and K5 LEVEL offset (luma only)
    signal s_gain  : integer range 0 to 7 := 4;
    signal s_level : signed(9 downto 0) := (others => '0');
    -- P12 STRENGTH: chroma saturation floor / luma ramp quantiser
    signal s_sthr   : unsigned(9 downto 0) := to_unsigned(80, 10);
    signal s_ymask  : unsigned(2 downto 0) := "111";
    -- K3 SCALE: 0..3 = 8/16/32/64 px cells (1 reproduces the approved sizes)
    signal s_scl    : integer range 0 to 3 := 1;
    constant C_DGB  : integer := 4;    -- diagonal stripe pitch bit (pre-scaled)
    -- K6 GLOSS
    signal s_gsh    : integer range 0 to 5 := 3;
    signal s_gspec  : unsigned(8 downto 0) := to_unsigned(220, 9);
    signal s_sprk   : unsigned(7 downto 0) := to_unsigned(48, 8);

    ----------------------------------------------------------------------
    -- raster measurement + centre
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt       : unsigned(11 downto 0) := (others => '0');
    -- Resolution scaling: virtual 1080-line units.  x accumulators are Q.1
    -- (steps 2/3/4 = 1.0/1.5/2.0 per pixel), y accumulators Q.8 per FIELD
    -- line (already doubled for interlace).  s_Wv / s_Hv are the measured
    -- raster in virtual units (1080p: the pixel / line counts).
    signal s_xs         : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys         : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xva        : unsigned(12 downto 0) := (others => '0');
    signal s_lva        : unsigned(19 downto 0) := (others => '0');
    signal s_Wv         : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Hv         : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_thy        : std_logic := '0';   -- a field line spans > 2 units
    signal s_gw2        : std_logic := '0';   -- grooves 2 units wide
    signal s_cx, s_cy   : signed(13 downto 0) := (others => '0');
    signal s_seq        : unsigned(4 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- coordinate accumulators (dx / xrun keep running through blanking so
    -- the pattern stage, sampled ahead of the class stream, stays continuous)
    ----------------------------------------------------------------------
    signal s_avid_p   : std_logic := '0';
    signal s_newframe : std_logic := '1';
    signal s_dxa      : signed(14 downto 0) := (others => '0');   -- Q.1
    signal s_dyca     : signed(21 downto 0) := (others => '0');   -- Q.8
    signal s_dyba     : signed(21 downto 0) := (others => '0');   -- Q.8
    signal s_xruna    : unsigned(12 downto 0) := (others => '0');  -- Q.1
    signal s_xrun     : unsigned(11 downto 0) := (others => '0');
    signal s_lrun     : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- colour path
    ----------------------------------------------------------------------
    signal e1_y, e1_u, e1_v : unsigned(9 downto 0) := (others => '0');
    -- G1..G3 proc-amp (CONTRAST / LEVEL)
    signal g1_ya, g1_yb, g1_ua, g1_ub, g1_va, g1_vb : signed(12 downto 0) := (others => '0');
    signal g2_y, g2_u, g2_v : signed(12 downto 0) := (others => '0');
    signal ec_y, ec_u, ec_v : unsigned(9 downto 0) := (others => '0');
    -- E2 / E3 classifier
    signal e2_sat : unsigned(9 downto 0) := (others => '0');
    signal e2_dus, e2_dvs : std_logic := '0';
    signal e2_cmp : std_logic := '0';
    signal e2_yb  : unsigned(2 downto 0) := (others => '0');
    signal e3_cls : unsigned(3 downto 0) := (others => '0');  -- 0-7 luma, 8-15 hue
    -- CL clean filter
    signal cl_cand : unsigned(3 downto 0) := (others => '0');
    signal cl_run  : unsigned(4 downto 0) := (others => '0');
    signal cl_hold : unsigned(3 downto 0) := (others => '0');
    signal cl_acc  : unsigned(3 downto 0) := (others => '0');
    type t_acc is array (0 to 16) of unsigned(3 downto 0);
    signal acc_sr  : t_acc := (others => (others => '0'));
    signal cl_out  : unsigned(3 downto 0) := (others => '0');   -- depth 25
    -- class line RAM (previous line's clean class), 2048 x 4
    type t_cram is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal cram    : t_cram;
    signal cram_q  : std_logic_vector(3 downto 0) := (others => '0');
    signal cu0     : unsigned(3 downto 0) := (others => '0');
    type t_cu is array (0 to 3) of unsigned(3 downto 0);
    signal cu_sr   : t_cu := (others => (others => '0'));
    signal cw_x    : unsigned(10 downto 0) := (others => '0');
    -- MAP + texture pipeline
    signal t_cls   : unsigned(3 downto 0) := (others => '0');   -- depth 26
    signal tm_q    : std_logic_vector(3 downto 0) := (others => '0'); -- 27 (ROM out)
    signal t4_tex  : unsigned(3 downto 0) := (others => '0');   -- depth 28
    signal ph_tex  : unsigned(3 downto 0) := (others => '0');   -- depth 29

    ----------------------------------------------------------------------
    -- geometry / pattern path
    ----------------------------------------------------------------------
    signal e1_dx, e1_dy : signed(13 downto 0) := (others => '0');
    signal w_proj, w_proj2, w_proj3, w_projh : unsigned(13 downto 0) := (others => '0');
    signal grv_f, ic_f : std_logic := '0';
    -- cell engine coords (scale-selected, registered ahead of the hash)
    signal cl_ix, cl_iy : unsigned(3 downto 0) := (others => '0');
    signal cl_hx  : unsigned(9 downto 0) := (others => '0');
    signal cl_hy6 : unsigned(5 downto 0) := (others => '0');
    signal cl_chk : std_logic := '0';
    -- pattern bits: MP (1) -> PH1 (2)
    signal dg1, dg2     : std_logic := '0';        -- diagonal A stripe
    signal mg1, mg2     : std_logic := '0';        -- diagonal B stripe
    signal ic1, ic2     : std_logic := '0';        -- scallop piping
    signal dot1, dot2   : std_logic := '0';        -- cell dot
    signal grv1, grv2   : std_logic := '0';        -- groove line
    signal chip1, chip2 : std_logic := '0';        -- cookie chip
    signal bean1, bean2 : std_logic := '0';        -- jelly bean
    signal chk1, chk2   : std_logic := '0';        -- checker
    signal wfl1, wfl2   : std_logic := '0';        -- waffle grid line
    signal hb1, hb2     : unsigned(1 downto 0) := (others => '0');  -- h-band
    signal cc1, cc2     : unsigned(1 downto 0) := (others => '0');  -- cell colour
    signal mp_rb        : unsigned(2 downto 0) := (others => '0');  -- rainbow stripe
    -- garnish decision (MP) -> PH1
    signal g_type : unsigned(1 downto 0) := (others => '0');
    signal g_cv   : unsigned(1 downto 0) := (others => '0');
    signal g_d1, g_d2 : std_logic := '0';
    signal ph_gt  : unsigned(1 downto 0) := (others => '0');
    -- PH1 outputs
    signal ph_ry, ph_ru, ph_rv : unsigned(9 downto 0) := (others => '0');
    signal ph_gadd : signed(10 downto 0) := (others => '0');
    signal ph_sh   : signed(10 downto 0) := (others => '0');
    signal ph_ol   : std_logic := '0';
    -- PH2 outputs
    signal ph_ry2, ph_ru2, ph_rv2 : unsigned(9 downto 0) := (others => '0');
    signal co_alt_y, co_alt_u, co_alt_v : unsigned(9 downto 0) := (others => '0');
    signal co_useRing : std_logic := '0';
    signal co_gy      : signed(10 downto 0) := (others => '0');
    -- CO + output
    signal wcy, wcu, wcv : unsigned(9 downto 0) := (others => '0');
    signal o_y, o_u, o_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- GLOSS: line RAM luma heightfield -> emboss -> sheen; OUTLINE flag
    ----------------------------------------------------------------------
    type t_lram is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal lram   : t_lram;
    signal lb_rd  : std_logic_vector(9 downto 0) := (others => '0');
    signal lb_wa  : unsigned(10 downto 0) := (others => '0');
    signal lb_wd  : std_logic_vector(9 downto 0) := (others => '0');
    signal lb_we  : std_logic := '0';
    signal gpl, gpl_d : unsigned(9 downto 0) := (others => '0');
    signal ggx, ggy : signed(10 downto 0) := (others => '0');
    signal glit     : signed(11 downto 0) := (others => '0');
    signal spec_hi, spec_hi2 : std_logic := '0';
    signal g_sh     : signed(13 downto 0) := (others => '0');
    signal g_add    : signed(10 downto 0) := (others => '0');
    type t_gsr is array (0 to 22) of signed(10 downto 0);
    signal gadd_sr : t_gsr := (others => (others => '0'));
    signal ol_sr   : std_logic_vector(0 to 25) := (others => '0');

    -- source-luma delay (Shade tap at 27)
    type t_sr is array (0 to 27) of std_logic_vector(9 downto 0);
    signal s_y_sr : t_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement (interlace-aware; own snapshotted counters)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            -- per-mode virtual steps (the timing ID is static while a mode
            -- runs, so free-running registers are enough)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            if s_ys > 512 then s_thy <= '1'; else s_thy <= '0'; end if;

            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
                s_xva  <= s_xva + s_xs;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_Wv <= s_xva(12 downto 1); end if;
                s_xcnt <= (others => '0');
                s_xva  <= (others => '0');
                s_lcnt <= s_lcnt + 1;
                s_lva  <= s_lva + s_ys;
            end if;

            if s_vs_pulse = '1' then
                -- height in virtual rows (the interlace factor is in s_ys;
                -- detecting it from field_n is dead on hardware, which ties
                -- field_n high).  Only on a vsync that followed real active
                -- lines (analog vsync serrates).
                if s_lcnt > 8 then
                    s_Hv <= s_lva(19 downto 8);
                end if;
                s_lcnt <= (others => '0');
                s_lva  <= (others => '0');
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one small op per state)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_c  : integer;
        variable v_n  : integer range 1 to 16;
        variable v_l  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                s_p12 <= unsigned(registers_in(7));
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_outline <= registers_in(6)(0);          -- S7
                s_shade   <= registers_in(6)(1);          -- S8
                s_bank    <= registers_in(6)(2);          -- S9
                s_sprk_on <= registers_in(6)(3);          -- S10
                s_lumamap <= registers_in(6)(4);          -- S11
                s_cx <= signed(resize(s_Wv(11 downto 1), 14));
                s_cy <= signed(resize(s_Hv(11 downto 1), 14));
                s_seq <= to_unsigned(7, 5);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 7 =>
                        -- GLOSS (K6) as a shift level; specular threshold
                        v_c := to_integer(s_k6(9 downto 7));
                        if v_c > 5 then v_c := 5; end if;
                        s_gsh   <= v_c;
                        s_gspec <= to_unsigned(300, 9) - resize(s_k6(9 downto 3), 9);
                    when 6 =>
                        -- SPRINKLES (S10): fixed "buried" density or none
                        if s_sprk_on = '1' then s_sprk <= to_unsigned(48, 8);
                        else                    s_sprk <= (others => '0'); end if;
                    when 5 =>
                        -- CLEAN (K2): run length 1 (raw) .. 16 px; the SR tap
                        -- keeps the total class delay fixed at 24 (see CL).
                        v_n := 1 + to_integer(s_k2(9 downto 6));
                        s_ncl <= v_n;
                        s_tap <= 17 - v_n;
                    when 4 =>
                        -- CONTRAST (K4) step, LEVEL (K5) bipolar luma offset
                        -- with a small deadband so centre is exact unity.
                        s_gain <= to_integer(s_k4(9 downto 7));
                        v_l := signed(resize(s_k5, 11)) - 512;
                        if v_l > -8 and v_l < 8 then
                            s_level <= (others => '0');
                        else
                            s_level <= resize(shift_right(v_l, 1), 10);
                        end if;
                    when 3 =>
                        -- THEME (K1): 8 themes, pair-swap, tone-shift
                        s_theme <= s_k1(9 downto 7);
                        s_swap  <= s_k1(6);
                        s_tone  <= s_k1(5);
                    when 2 =>
                        -- SCALE (K3): labelled param -- a value below the
                        -- label count is the raw label INDEX (firmware
                        -- loads the default that way).
                        if s_k3 < 4 then
                            s_scl <= to_integer(s_k3(1 downto 0));
                        else
                            s_scl <= to_integer(s_k3(9 downto 8));
                        end if;
                    when 1 =>
                        -- grooves: 2 units wide where a pixel steps more
                        -- than one unit at the 8 px scale (720p / SD)
                        if s_scl = 0 and s_xs /= 2 then s_gw2 <= '1';
                        else                            s_gw2 <= '0'; end if;
                        -- STRENGTH (P12): chroma saturation floor 263 -> 8;
                        -- luma ramp quantiser 2 / 4 / 8 varieties.
                        s_sthr <= to_unsigned(8 + 255 - to_integer(s_p12(9 downto 2)), 10);
                        if    s_p12 < 341 then s_ymask <= "100";
                        elsif s_p12 < 682 then s_ymask <= "110";
                        else                   s_ymask <= "111";
                        end if;
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- coordinate accumulators: dx = x - cx (per pixel, free-running through
    -- blanking), dy = y - cy (per active line, interlace-safe); xrun/lrun are
    -- the free-running pixel / latched line counters for the cell engine.
    ------------------------------------------------------------------------
    -- (All in virtual units: dx Q.1 per pixel, dy Q.8 per active field line;
    -- at 1080p the integer parts are the old pixel / line counts.)
    p_acc : process(clk)
        variable v_dy0 : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
                s_dxa <= s_dxa + signed(resize(s_xs, 15));
            elsif data_in.avid = '1' and s_avid_p = '0' then
                if s_newframe = '1' then
                    -- the bottom field starts half a field step down (inert
                    -- on hardware, which ties field_n high)
                    v_dy0 := shift_left(resize(-s_cy, 22), 8);
                    if data_in.field_n = '0' then
                        v_dy0 := v_dy0 + signed(resize(s_ys(11 downto 1), 22));
                    end if;
                    s_dyca <= v_dy0;
                    s_dyba <= v_dy0 + signed(resize(s_ys, 22));
                    s_newframe <= '0';
                else
                    s_dyca <= s_dyba;
                    s_dyba <= s_dyba + signed(resize(s_ys, 22));
                end if;
                s_dxa  <= shift_left(resize(-s_cx, 15), 1);
                s_lrun <= s_lva(19 downto 8);
            else
                s_dxa <= s_dxa + signed(resize(s_xs, 15));
            end if;
            -- xrun = xcnt - 1 during the line, keeps counting after it ends
            if data_in.avid = '1' then s_xruna <= s_xva;
            else                       s_xruna <= s_xruna + s_xs; end if;
        end if;
    end process p_acc;

    s_xrun <= s_xruna(12 downto 1);

    ------------------------------------------------------------------------
    -- COLOUR PATH: proc-amp -> classify -> clean -> map -> texture -> out
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d  : signed(12 downto 0);
        variable v_dua, v_dva : unsigned(9 downto 0);
        variable v_dus, v_dvs : std_logic;
        variable v_oct : unsigned(3 downto 0);
        variable v_acc : boolean;
        variable v_cl  : unsigned(3 downto 0);
        variable v_idx : integer range 0 to 7;
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_hc2 : unsigned(15 downto 0);
        variable v_ax4, v_ay4 : unsigned(3 downto 0);
        variable v_cx, v_cy, v_aya, v_axa : signed(5 downto 0);
        variable v_hc : unsigned(15 downto 0);
        variable v_sx3, v_sy3 : unsigned(2 downto 0);
        variable v_indot : boolean;
        variable v_hrow, v_grow : boolean;
    begin
        if rising_edge(clk) then
            -- source-luma delay line (Shade tap)
            s_y_sr(0) <= data_in.y;
            for i in 1 to 27 loop
                s_y_sr(i) <= s_y_sr(i - 1);
            end loop;

            -- E1 (depth 1): input latch.  HARDWARE U/V SWAP (HW-confirmed
            -- 2026-07-27): the u wire carries Cr and v carries Cb, so swap
            -- here; everything internal is standard BT.601 and the output
            -- swaps back at CO.
            e1_y  <= unsigned(data_in.y);
            e1_u  <= unsigned(data_in.v);
            e1_v  <= unsigned(data_in.u);
            e1_dx <= s_dxa(14 downto 1);
            e1_dy <= s_dyca(21 downto 8);

            -- G1 (2): CONTRAST gain as two shifted terms of the centred value
            -- (0.375 0.5 0.625 0.75 1 1.5 2 3; step 4 = unity).
            v_d := signed(resize(e1_y, 13)) - 512;
            case s_gain is
                when 0 => g1_ya <= shift_right(v_d, 2); g1_yb <= shift_right(v_d, 3);
                when 1 => g1_ya <= shift_right(v_d, 1); g1_yb <= (others => '0');
                when 2 => g1_ya <= shift_right(v_d, 1); g1_yb <= shift_right(v_d, 3);
                when 3 => g1_ya <= shift_right(v_d, 1); g1_yb <= shift_right(v_d, 2);
                when 4 => g1_ya <= v_d;                 g1_yb <= (others => '0');
                when 5 => g1_ya <= v_d;                 g1_yb <= shift_right(v_d, 1);
                when 6 => g1_ya <= shift_left(v_d, 1);  g1_yb <= (others => '0');
                when others => g1_ya <= shift_left(v_d, 1); g1_yb <= v_d;
            end case;
            v_d := signed(resize(e1_u, 13)) - 512;
            case s_gain is
                when 0 => g1_ua <= shift_right(v_d, 2); g1_ub <= shift_right(v_d, 3);
                when 1 => g1_ua <= shift_right(v_d, 1); g1_ub <= (others => '0');
                when 2 => g1_ua <= shift_right(v_d, 1); g1_ub <= shift_right(v_d, 3);
                when 3 => g1_ua <= shift_right(v_d, 1); g1_ub <= shift_right(v_d, 2);
                when 4 => g1_ua <= v_d;                 g1_ub <= (others => '0');
                when 5 => g1_ua <= v_d;                 g1_ub <= shift_right(v_d, 1);
                when 6 => g1_ua <= shift_left(v_d, 1);  g1_ub <= (others => '0');
                when others => g1_ua <= shift_left(v_d, 1); g1_ub <= v_d;
            end case;
            v_d := signed(resize(e1_v, 13)) - 512;
            case s_gain is
                when 0 => g1_va <= shift_right(v_d, 2); g1_vb <= shift_right(v_d, 3);
                when 1 => g1_va <= shift_right(v_d, 1); g1_vb <= (others => '0');
                when 2 => g1_va <= shift_right(v_d, 1); g1_vb <= shift_right(v_d, 3);
                when 3 => g1_va <= shift_right(v_d, 1); g1_vb <= shift_right(v_d, 2);
                when 4 => g1_va <= v_d;                 g1_vb <= (others => '0');
                when 5 => g1_va <= v_d;                 g1_vb <= shift_right(v_d, 1);
                when 6 => g1_va <= shift_left(v_d, 1);  g1_vb <= (others => '0');
                when others => g1_va <= shift_left(v_d, 1); g1_vb <= v_d;
            end case;

            -- G2 (3): sum the terms
            g2_y <= g1_ya + g1_yb;
            g2_u <= g1_ua + g1_ub;
            g2_v <= g1_va + g1_vb;

            -- G3 (4): re-centre, LEVEL offset on luma, clamp
            ec_y <= f_cu10(resize(g2_y, 14) + 512 + resize(s_level, 14));
            ec_u <= f_cu10(resize(g2_u, 14) + 512);
            ec_v <= f_cu10(resize(g2_v, 14) + 512);

            -- E2 (5): chroma split -- |du|, |dv|, saturation, octant terms
            if ec_u >= 512 then v_dua := resize(ec_u - 512, 10); v_dus := '1';
            else                v_dua := resize(512 - ec_u, 10); v_dus := '0'; end if;
            if ec_v >= 512 then v_dva := resize(ec_v - 512, 10); v_dvs := '1';
            else                v_dva := resize(512 - ec_v, 10); v_dvs := '0'; end if;
            e2_sat <= v_dua + v_dva;
            e2_dus <= v_dus;
            e2_dvs <= v_dvs;
            if v_dua > v_dva then e2_cmp <= '1'; else e2_cmp <= '0'; end if;
            e2_yb  <= ec_y(9 downto 7);

            -- E3 (6): CLASSIFY.  Class 0-7 = luma band (Luma map, or a neutral
            -- pixel in the Chroma map -- so grey reads the same in both
            -- modes); class 8-15 = hue octant (Cr high, Cb high, |Cb| dominant:
            -- green, teal, cyan, blue, red, amber, pink, violet).
            if s_lumamap = '1' then
                e3_cls <= '0' & (e2_yb and s_ymask);
            elsif e2_sat < s_sthr then
                e3_cls <= '0' & e2_yb;
            else
                e3_cls <= '1' & e2_dvs & e2_dus & e2_cmp;
            end if;

            -- CL (7): CLEAN.  A class is ACCEPTED when its run has reached
            -- s_ncl pixels, or it matches the clean class directly above
            -- (cu_sr(3), from the class line RAM).  Otherwise the last
            -- accepted class holds -- specks narrower than s_ncl vanish.
            -- The accepted stream is read back s_ncl-1 pixels LATE through
            -- the variable tap below, so genuine edges do not shift right.
            if s_avid_sr(5) = '0' then                  -- aligned with e3_cls
                cl_run  <= (others => '0');
                cl_cand <= e3_cls;
            elsif e3_cls = cl_cand then
                if cl_run /= 31 then cl_run <= cl_run + 1; end if;
            else
                cl_cand <= e3_cls;
                cl_run  <= to_unsigned(1, 5);
            end if;
            v_acc := (e3_cls = cl_cand and to_integer(cl_run) >= s_ncl - 1)
                     or s_ncl = 1 or e3_cls = cu_sr(3);
            if v_acc then
                cl_acc  <= e3_cls;
                cl_hold <= e3_cls;
            else
                cl_acc  <= cl_hold;
            end if;
            -- fixed-total-delay SR: tap 17 - s_ncl -> cl_out is pixel t-25
            acc_sr(0) <= cl_acc;
            for i in 1 to 16 loop
                acc_sr(i) <= acc_sr(i - 1);
            end loop;
            cl_out <= acc_sr(s_tap);

            -- class line RAM: write the clean class behind the output
            -- counter; read every clock at the live pixel index, EBR output
            -- register -> fabric register -> align to E3 (4 more).
            if s_avid_sr(24) = '1' then
                cram(to_integer(cw_x)) <= std_logic_vector(cl_out);
                cw_x <= cw_x + 1;
            else
                cw_x <= (others => '0');
            end if;
            cram_q <= cram(to_integer(s_xcnt(10 downto 0)));
            cu0    <= unsigned(cram_q);
            cu_sr(0) <= cu0;
            for i in 1 to 3 loop
                cu_sr(i) <= cu_sr(i - 1);
            end loop;

            -- MAP (26): K1 variants on the class -- TONE shifts the luma ramp
            -- one step brighter (hues untouched), SWAP exchanges adjacent
            -- pairs (neighbouring luma bands / neighbouring hues), both keep
            -- the value / hue ordering.  Then (27) the theme ROM, (28) reg.
            v_cl := cl_out;
            if s_tone = '1' and v_cl(3) = '0' and v_cl(2 downto 0) /= 7 then
                v_cl := v_cl + 1;
            end if;
            if s_swap = '1' then v_cl(0) := not v_cl(0); end if;
            t_cls  <= v_cl;
            tm_q   <= C_TMAP(to_integer(unsigned'(s_bank & s_theme & t_cls)));
            t4_tex <= unsigned(tm_q);

            -- =============== PATTERN STAGE (screen-space, free-running) =====
            -- MP: stripe / band bits from the pre-scaled projections
            dg1   <= w_proj(C_DGB);                    -- diagonal A
            mg1   <= w_proj2(C_DGB);                   -- diagonal B
            mp_rb <= w_projh(6 downto 4);              -- rainbow stripe (32 px)
            hb1   <= w_projh(5 downto 4);              -- allsorts band
            grv1  <= grv_f;
            ic1   <= ic_f;
            chk1  <= cl_chk;
            if cl_ix < 2 or cl_iy < 2 then wfl1 <= '1'; else wfl1 <= '0'; end if;
            -- cell engine: hash per cell, in-cell folded distances
            v_hc2 := f_hash(cl_hx, cl_hy6 & "0000", x"C3B1");
            cc1 <= v_hc2(12 downto 11);
            if cl_ix(3) = '1' then v_ax4 := unsigned('0' & cl_ix(2 downto 0));
            else                   v_ax4 := unsigned('0' & not cl_ix(2 downto 0)); end if;
            if cl_iy(3) = '1' then v_ay4 := unsigned('0' & cl_iy(2 downto 0));
            else                   v_ay4 := unsigned('0' & not cl_iy(2 downto 0)); end if;
            v_cx := signed(resize(cl_ix, 6)) - 8;
            v_cy := signed(resize(cl_iy, 6)) - 8;
            if v_hc2(9) = '1' then v_cx := -v_cx; end if;      -- point left/right
            if v_hc2(10) = '1' then v_cy := v_cy + 1;          -- sit high/low
            else                    v_cy := v_cy - 1; end if;
            if v_cy < 0 then v_aya := -v_cy; else v_aya := v_cy; end if;
            if v_cx < 0 then v_axa := -v_cx; else v_axa := v_cx; end if;
            -- CHOCOLATE CHIP: big sideways teardrop (~14 x 11 px of a 16 cell)
            if v_hc2(7 downto 0) < 72 and v_cx > -6 and v_aya < 6
               and (v_cx + shift_left(v_aya, 1)) < 11
               and (v_cx + v_aya) < 9 then
                chip1 <= '1';
            else
                chip1 <= '0';
            end if;
            -- JELLY BEAN: rounded capsule (~13 x 9 px)
            if v_hc2(7 downto 0) < 160 and v_axa < 7 and v_aya < 5
               and (v_axa + v_aya) < 9 then
                bean1 <= '1';
            else
                bean1 <= '0';
            end if;
            -- cream / nonpareil dot: regular grid, one dot per cell
            if (v_ax4 + v_ay4) < 5 then dot1 <= '1'; else dot1 <= '0'; end if;

            -- GARNISH decision (8 px cells, fixed physical size -- K3 does
            -- not scale it; virtual units like everything else).  Where a
            -- field line spans more than 2 units (SD) the horizontal-jimmie
            -- and glint rows widen to 5 units so every cell still lands one
            -- line on them.
            v_hc := f_hash(resize(s_xrun(9 downto 3), 10),
                           s_lrun(8 downto 3) & "0000", x"5A3C");
            v_sx3 := s_xrun(2 downto 0);
            v_sy3 := s_lrun(2 downto 0);
            if s_thy = '1' then
                v_hrow := (v_sy3 >= 2 and v_sy3 <= 6);
                v_grow := (v_sy3 >= 3);
            else
                v_hrow := (v_sy3 >= 3 and v_sy3 <= 4);
                v_grow := (v_sy3 = 3);
            end if;
            if v_hc(12) = '0' then
                v_indot := (v_hrow and v_sx3 >= 1 and v_sx3 <= 6);
            else
                v_indot := (v_sx3 >= 3 and v_sx3 <= 4 and v_sy3 >= 1 and v_sy3 <= 6);
            end if;
            g_cv <= v_hc(14 downto 13);
            if s_sprk /= 0 and v_hc(7 downto 0) < s_sprk and v_indot then
                g_type <= "01";                -- jimmie
            elsif s_sprk /= 0 and v_hc(15 downto 9) < ('0' & s_sprk(7 downto 2))
                  and v_hc(3 downto 0) > 13
                  and v_sx3(2 downto 1) = "01" and v_grow then
                g_type <= "10";                -- sugar-dust glint (2x1 px)
            else
                g_type <= "00";
            end if;
            if s_sprk /= 0 and v_hc(7 downto 0) < (s_sprk(5 downto 0) & "00")
               and v_indot then
                g_d1 <= '1';                   -- dense frosting confetti
            else
                g_d1 <= '0';
            end if;

            -- PH1 (29): ONE rainbow-ROM read per pixel -- taffy stripe index
            -- for the taffy texture, hash colour for confetti / jimmies.
            ph_tex <= t4_tex;
            if t4_tex = 6 then v_idx := to_integer(mp_rb);
            else               v_idx := to_integer(g_cv & '1'); end if;
            ph_ry <= C_RB_Y(v_idx);
            ph_ru <= C_RB_U(v_idx);
            ph_rv <= C_RB_V(v_idx);
            ph_gadd <= gadd_sr(22);
            ph_ol   <= ol_sr(25);
            if s_shade = '1' then              -- SHADE: (y - 512) / 2
                ph_sh <= resize(shift_right(signed(resize(unsigned(s_y_sr(27)), 11)) - 512, 1), 11);
            else
                ph_sh <= (others => '0');
            end if;
            ph_gt <= g_type;
            dg2 <= dg1;  mg2 <= mg1;  ic2 <= ic1;  dot2 <= dot1;  grv2 <= grv1;
            chip2 <= chip1;  bean2 <= bean1;  chk2 <= chk1;  wfl2 <= wfl1;
            hb2 <= hb1;  cc2 <= cc1;  g_d2 <= g_d1;

            -- PH2 (30): TEXTURE BANK -- constant-colour selects only.
            co_useRing <= '0';
            co_gy      <= ph_gadd + ph_sh;
            ph_ry2 <= ph_ry; ph_ru2 <= ph_ru; ph_rv2 <= ph_rv;
            case to_integer(ph_tex) is
                when 0 =>          -- DARK CHOCOLATE bar: cocoa + grooves
                    if grv2 = '1' then co_alt_y <= to_unsigned(105, 10);
                    else               co_alt_y <= to_unsigned(185, 10); end if;
                    co_alt_u <= to_unsigned(462, 10); co_alt_v <= to_unsigned(566, 10);
                when 1 =>          -- COOKIES & CREAM: cream dots on black
                    if dot2 = '1' then
                        co_alt_y <= to_unsigned(965, 10); co_alt_u <= to_unsigned(506, 10); co_alt_v <= to_unsigned(522, 10);
                    else
                        co_alt_y <= to_unsigned(85, 10);  co_alt_u <= to_unsigned(502, 10); co_alt_v <= to_unsigned(526, 10);
                    end if;
                when 2 =>          -- CHOC-CHIP COOKIE: tan dough + big chips
                    if chip2 = '1' then
                        co_alt_y <= to_unsigned(170, 10); co_alt_u <= to_unsigned(462, 10); co_alt_v <= to_unsigned(570, 10);
                    else
                        co_alt_y <= to_unsigned(600, 10); co_alt_u <= to_unsigned(400, 10); co_alt_v <= to_unsigned(600, 10);
                    end if;
                when 3 =>          -- CANDY CANE: red / white diagonal A
                    if dg2 = '1' then
                        co_alt_y <= to_unsigned(360, 10); co_alt_u <= to_unsigned(425, 10); co_alt_v <= to_unsigned(865, 10);
                    else
                        co_alt_y <= C_WHITE_Y; co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                    end if;
                when 4 =>          -- MINT: spearmint / white diagonal B
                    if mg2 = '1' then
                        co_alt_y <= to_unsigned(690, 10); co_alt_u <= to_unsigned(420, 10); co_alt_v <= to_unsigned(350, 10);
                    else
                        co_alt_y <= C_WHITE_Y; co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                    end if;
                when 5 =>          -- PIPED ICING: pink scallop piping on white
                    if ic2 = '1' then
                        co_alt_y <= to_unsigned(720, 10); co_alt_u <= to_unsigned(560, 10); co_alt_v <= to_unsigned(650, 10);
                    else
                        co_alt_y <= to_unsigned(980, 10); co_alt_u <= to_unsigned(508, 10); co_alt_v <= to_unsigned(518, 10);
                    end if;
                when 6 =>          -- RAINBOW TAFFY: horizontal rainbow stripes
                    co_useRing <= '1';
                    co_alt_y <= C_WHITE_Y; co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                when 7 =>          -- FROSTING: white + dense rainbow confetti
                    co_alt_y <= to_unsigned(1010, 10); co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                    if g_d2 = '1' then co_useRing <= '1'; end if;
                when 8 =>          -- BATTENBERG: pink / yellow checks
                    if chk2 = '1' then
                        co_alt_y <= to_unsigned(699, 10); co_alt_u <= to_unsigned(499, 10); co_alt_v <= to_unsigned(675, 10);
                    else
                        co_alt_y <= to_unsigned(800, 10); co_alt_u <= to_unsigned(321, 10); co_alt_v <= to_unsigned(577, 10);
                    end if;
                when 9 =>          -- LIQUORICE ALLSORTS: black / pink / black / yellow bands
                    case hb2 is
                        when "01"   => co_alt_y <= to_unsigned(635, 10); co_alt_u <= to_unsigned(511, 10); co_alt_v <= to_unsigned(703, 10);
                        when "11"   => co_alt_y <= to_unsigned(788, 10); co_alt_u <= to_unsigned(246, 10); co_alt_v <= to_unsigned(610, 10);
                        when others => co_alt_y <= to_unsigned(80, 10);  co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                    end case;
                when 10 =>         -- WAFFLE CONE: tan with brown grid lines
                    if wfl2 = '1' then
                        co_alt_y <= to_unsigned(320, 10); co_alt_u <= to_unsigned(415, 10); co_alt_v <= to_unsigned(613, 10);
                    else
                        co_alt_y <= to_unsigned(625, 10); co_alt_u <= to_unsigned(340, 10); co_alt_v <= to_unsigned(646, 10);
                    end if;
                when 11 =>         -- JELLY BEANS: coloured beans on white
                    if bean2 = '1' then
                        case cc2 is
                            when "00"   => co_alt_y <= to_unsigned(392, 10); co_alt_u <= to_unsigned(416, 10); co_alt_v <= to_unsigned(798, 10);
                            when "01"   => co_alt_y <= to_unsigned(527, 10); co_alt_u <= to_unsigned(392, 10); co_alt_v <= to_unsigned(334, 10);
                            when "10"   => co_alt_y <= to_unsigned(765, 10); co_alt_u <= to_unsigned(233, 10); co_alt_v <= to_unsigned(593, 10);
                            when others => co_alt_y <= to_unsigned(391, 10); co_alt_u <= to_unsigned(696, 10); co_alt_v <= to_unsigned(608, 10);
                        end case;
                    else
                        co_alt_y <= C_WHITE_Y; co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                    end if;
                when 12 =>         -- GINGERBREAD: brown with white icing scallops
                    if ic2 = '1' then
                        co_alt_y <= to_unsigned(980, 10); co_alt_u <= to_unsigned(508, 10); co_alt_v <= to_unsigned(518, 10);
                    else
                        co_alt_y <= to_unsigned(403, 10); co_alt_u <= to_unsigned(377, 10); co_alt_v <= to_unsigned(648, 10);
                    end if;
                when 13 =>         -- LIQUORICE TWIST: black / grey diagonal B
                    if mg2 = '1' then
                        co_alt_y <= to_unsigned(70, 10);  co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                    else
                        co_alt_y <= to_unsigned(329, 10); co_alt_u <= to_unsigned(521, 10); co_alt_v <= to_unsigned(511, 10);
                    end if;
                when 14 =>         -- NONPAREILS: coloured dots on dark chocolate
                    if dot2 = '1' then
                        case cc2 is
                            when "00"   => co_alt_y <= to_unsigned(392, 10); co_alt_u <= to_unsigned(416, 10); co_alt_v <= to_unsigned(798, 10);
                            when "01"   => co_alt_y <= to_unsigned(527, 10); co_alt_u <= to_unsigned(392, 10); co_alt_v <= to_unsigned(334, 10);
                            when "10"   => co_alt_y <= to_unsigned(765, 10); co_alt_u <= to_unsigned(233, 10); co_alt_v <= to_unsigned(593, 10);
                            when others => co_alt_y <= to_unsigned(391, 10); co_alt_u <= to_unsigned(696, 10); co_alt_v <= to_unsigned(608, 10);
                        end case;
                    else
                        co_alt_y <= to_unsigned(150, 10); co_alt_u <= to_unsigned(462, 10); co_alt_v <= to_unsigned(566, 10);
                    end if;
                when others =>     -- WHITE CHOCOLATE: cream bar with grooves
                    if grv2 = '1' then co_alt_y <= to_unsigned(640, 10);
                    else               co_alt_y <= to_unsigned(800, 10); end if;
                    co_alt_u <= to_unsigned(437, 10); co_alt_v <= to_unsigned(545, 10);
            end case;

            -- GARNISH override: jimmies only on the smooth bright surfaces
            -- (icing 5, frosting 7); sugar dust everywhere.
            if ph_gt = "01" and (ph_tex = 5 or ph_tex = 7) then
                co_useRing <= '1';
                co_gy <= to_signed(120, 11);
            elsif ph_gt = "10" then
                co_alt_y <= C_WHITE_Y; co_alt_u <= to_unsigned(512, 10); co_alt_v <= to_unsigned(512, 10);
                co_useRing <= '0'; co_gy <= (others => '0');
            end if;
            -- OUTLINE (S7): dark cocoa contour over everything
            if s_outline = '1' and ph_ol = '1' then
                co_alt_y <= to_unsigned(90, 10);
                co_alt_u <= to_unsigned(478, 10); co_alt_v <= to_unsigned(545, 10);
                co_useRing <= '0'; co_gy <= (others => '0');
            end if;

            -- CO (31): 2:1 mux + gloss/shade add + clamp; U/V swap back out
            if co_useRing = '1' then
                v_by := ph_ry2; v_bu := ph_ru2; v_bv := ph_rv2;
            else
                v_by := co_alt_y; v_bu := co_alt_u; v_bv := co_alt_v;
            end if;
            wcy <= f_cu10(signed(resize(v_by, 12)) + resize(co_gy, 12));
            wcu <= v_bv;
            wcv <= v_bu;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- GEOMETRY: static projections of (dx,dy) -> stripe/band/groove bits,
    -- and the SCALE-selected cell coordinates for the cell engine.
    ------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_pj, v_pj2, v_pj3 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- projections: both 45-deg diagonals, a wavy horizontal (icing
            -- scallops) and a plain horizontal (rainbow / allsorts bands).
            -- SCALE applied ONCE here as a pre-shift; consumers keep fixed bits.
            v_pj  := resize(e1_dx, 15) + resize(e1_dy, 15);
            v_pj2 := resize(e1_dx, 15) - resize(e1_dy, 15);
            v_pj3 := resize(e1_dy, 15)
                     + resize(shift_right(f_tri(unsigned(std_logic_vector(e1_dx(9 downto 0)))), 3), 15);
            w_proj  <= shift_right(unsigned(std_logic_vector(v_pj(13 downto 0))), s_scl);
            w_proj2 <= shift_right(unsigned(std_logic_vector(v_pj2(13 downto 0))), s_scl);
            w_proj3 <= shift_right(unsigned(std_logic_vector(v_pj3(13 downto 0))), s_scl);
            w_projh <= shift_right(unsigned(std_logic_vector(e1_dy(13 downto 0))), s_scl);

            -- chocolate-bar grooves: thin lines on both diagonals
            -- (2 units wide when a pixel steps more than one, see s_gw2)
            if (w_proj(4 downto 1) = "0000" and (w_proj(0) = '0' or s_gw2 = '1'))
               or (w_proj2(4 downto 1) = "0000" and (w_proj2(0) = '0' or s_gw2 = '1')) then
                grv_f <= '1';
            else grv_f <= '0'; end if;
            -- scallop piping: thin wavy bands
            if w_proj3(3 downto 2) = "11" then ic_f <= '1'; else ic_f <= '0'; end if;

            -- cell coordinates: 8/16/32/64 px cells, in-cell position
            -- normalised to 0..15; cell index for the hash (y into bits 9:4)
            case s_scl is
                when 0 =>
                    cl_ix <= s_xrun(2 downto 0) & '0'; cl_iy <= s_lrun(2 downto 0) & '0';
                    cl_hx <= resize(s_xrun(9 downto 3), 10); cl_hy6 <= resize(s_lrun(8 downto 3), 6);
                    cl_chk <= s_xrun(4) xor s_lrun(4);
                when 1 =>
                    cl_ix <= s_xrun(3 downto 0); cl_iy <= s_lrun(3 downto 0);
                    cl_hx <= resize(s_xrun(10 downto 4), 10); cl_hy6 <= resize(s_lrun(9 downto 4), 6);
                    cl_chk <= s_xrun(5) xor s_lrun(5);
                when 2 =>
                    cl_ix <= s_xrun(4 downto 1); cl_iy <= s_lrun(4 downto 1);
                    cl_hx <= resize(s_xrun(10 downto 5), 10); cl_hy6 <= resize(s_lrun(10 downto 5), 6);
                    cl_chk <= s_xrun(6) xor s_lrun(6);
                when others =>
                    cl_ix <= s_xrun(5 downto 2); cl_iy <= s_lrun(5 downto 2);
                    cl_hx <= resize(s_xrun(10 downto 6), 10); cl_hy6 <= resize(s_lrun(10 downto 6), 6);
                    cl_chk <= s_xrun(7) xor s_lrun(7);
            end case;
        end if;
    end process p_geo;

    ------------------------------------------------------------------------
    -- GLOSS: line-RAM luma heightfield -> emboss -> specular sheen (g_add)
    -- + OUTLINE edge flag.  Both delayed to the compose stage.
    ------------------------------------------------------------------------
    p_gloss : process(clk)
        variable v_a : signed(13 downto 0);
        variable v_lit : signed(11 downto 0);
        variable v_mag : signed(11 downto 0);
        variable v_sh : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- L0: read the previous line at x; this line's pixel is written
            -- one clock later at x-1 (never the address being read).
            lb_rd <= lram(to_integer(s_xcnt(10 downto 0)));
            lb_wa <= s_xcnt(10 downto 0);
            lb_wd <= data_in.y;
            lb_we <= data_in.avid;
            if lb_we = '1' then
                lram(to_integer(lb_wa)) <= lb_wd;
            end if;
            gpl   <= unsigned(data_in.y);
            gpl_d <= gpl;

            -- L1: gradients
            ggx <= signed('0' & gpl) - signed('0' & gpl_d);
            ggy <= signed('0' & gpl) - signed('0' & lb_rd);

            -- L2: lit = gx+gy (light upper-left); specular decision; clamp
            v_lit := resize(ggx, 12) + resize(ggy, 12);
            if v_lit > signed('0' & s_gspec) then spec_hi <= '1';
            else                                  spec_hi <= '0'; end if;
            if    v_lit >  255 then glit <= to_signed(255, 12);
            elsif v_lit < -255 then glit <= to_signed(-255, 12);
            else                    glit <= v_lit; end if;
            -- OUTLINE flag (parallel): |gx| + |gy| above a fixed threshold
            if ggx < 0 then v_mag := resize(-ggx, 12); else v_mag := resize(ggx, 12); end if;
            if ggy < 0 then v_mag := v_mag - resize(ggy, 12); else v_mag := v_mag + resize(ggy, 12); end if;
            if v_mag > 96 then ol_sr(0) <= '1'; else ol_sr(0) <= '0'; end if;
            for i in 1 to 25 loop
                ol_sr(i) <= ol_sr(i - 1);
            end loop;

            -- L3: sheen shift only
            if s_gsh = 0 then
                v_sh := (others => '0');
            else
                v_sh := shift_left(resize(glit, 14), s_gsh - 1);
            end if;
            g_sh     <= v_sh;
            spec_hi2 <= spec_hi;

            -- L4: blown-crescent add + clamp
            v_a := g_sh;
            if spec_hi2 = '1' then v_a := v_a + 360; end if;
            if v_a > 511 then v_a := to_signed(511, 14);
            elsif v_a < -300 then v_a := to_signed(-300, 14); end if;
            g_add <= resize(v_a, 11);

            gadd_sr(0) <= g_add;
            for i in 1 to 22 loop
                gadd_sr(i) <= gadd_sr(i - 1);
            end loop;
        end if;
    end process p_gloss;

    ------------------------------------------------------------------------
    -- OUTPUT: blanking gate (neutral outside active video) + register.
    ------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_LATENCY - 2) = '1' then
                o_y <= wcy; o_u <= wcu; o_v <= wcv;
            else
                o_y <= to_unsigned(64, 10);
                o_u <= to_unsigned(512, 10);
                o_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture sugarcoat;
