-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Edge Lab: video redrawn as edge lines.
--
-- Two line styles (S9): Fine = Canny (thin, connected single-pixel
-- tracing) and Chalk = morphological gradient (thick, even bands), fed by
-- the image-cleanup chain you want in front of a detector (brightness,
-- contrast, posterize, optional Gaussian smoothing), with edge thickening,
-- four colour modes, a not-edges negative, overlay and gradient views.
--
-- Styles (S9; both normalized so a luma step of h reads about h, so one
-- Sensitivity setting suits either):
--   Fine    Canny: Sobel + gradient-direction quantize + non-maximum
--           suppression + double threshold + causal hysteresis (weak edges
--           survive next to an accepted left/upper edge).
--   Chalk   morphological gradient: 3x3 max - 3x3 min (dilate minus
--           erode).  Thick, even, chalky outlines on both sides of every
--           edge.
-- (v1.0-1.2 carried nine detectors; Roberts/Prewitt/Sobel/Scharr/Kirsch
-- looked alike once normalized and Laplace/LoG were just noisier, so v1.3
-- keeps the two distinct looks and gives K6 to colour.)
--
-- Controls:
--   K1 Thickness  dilate the edge map: 1 / 2 / 3 / 4 / 5 px squares (1080p
--                 pixels; v1.3.2 scales the square per mode so the stroke
--                 is about the same fraction of screen height: 720p
--                 1/2/2/3/4, SD 1/1/2/2/3 wide x 1 field line)
--   K2 Mix        dry/wet crossfade
--   K3 Brightness pre-offset, +-512
--   K4 Contrast   pre-gain 0..4x around mid-grey (unity at 25%)
--   K5 Posterize  quantize luma to 128..2 levels before detection
--   K6 Color      White   white lines (Invert: black)
--                 Compass hue by gradient direction (16-sector Sobel angle
--                         on a 360-degree wheel; Invert: complementary)
--                 Source  the input's own colour, saturation x2 (Invert:
--                         complementary chroma)
--                 Heat    by edge strength, dark red -> orange -> yellow ->
--                         white (Invert: reversed ramp)
--   S7 Smooth     3x3 Gaussian pre-blur feeding the detector
--   S8 Invert     negative: not-edges, and the colour mode's inverse
--   S9 Style      Fine / Chalk
--   S10 Overlay   draw edges on top of the live picture instead of black
--   S11 Output    Binary / Gradient (edge strength as grey or colour; Fine
--                 shows the post-NMS magnitude -- thinned ridges)
--   P12 Sensitivity  live threshold, glided: up = more edges; 100% dissolves
--                 the picture into dense edge grain (Canny low = high/2)
--
-- Video levels: black 64, white 940, gradient 64..924, blanking neutral.
--
-- Structure (all luma; chroma is neutral except overlay/mix/colour):
--   S1-S4   preprocess: (y-512)*gain>>4 + bright, clamp, posterize -> 8 bit
--   S5      4 chained line buffers -> 5x5 raw window + per-column sums
--   S6-S8   [1 2 1]^2 blurred 3x3; Smooth mux -> effective 3-row column
--   S9-S14  Sobel / Canny direction and the Morph max-min tree, one
--           registered op-level per stage (S13-S14 are pure delays that
--           keep the v1.2 latency)
--   S15     normalize to 0..255, style mux, edge blank
--   S16-S17 magnitude line buffers -> 3x3 mag window, NMS (Fine only,
--           passthrough for Chalk so both styles share one latency)
--   S18     threshold + hysteresis (accept-bit line buffer)
--   S19     thickness dilate (4-line packed bit buffer, 5x5 window,
--           separable: per-mode width x height)
--   S20     compose views; then interpolator_u dry/wet mix (4 clocks)
--
-- Compass hue and Heat strength run on their own small Sobel over window
-- rows 2..4 (one line below the displayed edge row, which is invisible for
-- a colour) so they need no line buffer; S7-S11 compute the 16-sector angle
-- and a 4-bit strength, then a valid-gated shift chain lines them up with
-- the dilated output column.
--
-- All line buffers are canonical 1W1R single-process BRAMs, chained by
-- rewriting each read value one buffer down (inferno lesson).  The dry-video
-- delay for overlay/mix/Source colour is a 32-deep EBR ring (2 EBR) instead
-- of a 24-stage 30-bit shift register (~700 LCs saved).  ~30 EBR total.
--
-- The edge layer is 4 lines above the input row (window centering); the
-- dry ring is tapped 5 columns late so overlay/mix line up horizontally.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture edgelab of program_top is
    -- data_in -> data_out latency is 24 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;

    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_TOTAL_LATENCY : integer := 24;                  -- 20 wet + 4 mix

    ----------------------------------------------------------------------
    -- Helpers
    ----------------------------------------------------------------------
    function f_c255(v : unsigned) return unsigned is
    begin
        if v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    function f_absu(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(resize(-v, v'length));
        else
            return unsigned(v);
        end if;
    end function;

    -- Colour palettes, stored as offsets from black: Y-64 & (U-512) &
    -- (V-512), U/V-swapped BT.601 for hardware.
    -- Compass: 16 hues (HSV s=0.8 v=0.88); index 8 apart = complementary.
    function f_pal(i : unsigned(3 downto 0)) return std_logic_vector is
        variable y, u, v : integer;
    begin
        case to_integer(i) is
            when  0 => y := 339; u :=  315; v := -106;
            when  1 => y := 474; u :=  216; v := -185;
            when  2 => y := 610; u :=  117; v := -263;
            when  3 => y := 678; u :=   12; v := -302;
            when  4 => y := 608; u := -106; v := -262;
            when  5 => y := 539; u := -225; v := -222;
            when  6 => y := 534; u := -277; v := -130;
            when  7 => y := 560; u := -296; v :=  -12;
            when  8 => y := 586; u := -315; v :=  106;
            when  9 => y := 451; u := -216; v :=  185;
            when 10 => y := 315; u := -117; v :=  263;
            when 11 => y := 248; u :=  -12; v :=  302;
            when 12 => y := 317; u :=  106; v :=  262;
            when 13 => y := 386; u :=  225; v :=  222;
            when 14 => y := 391; u :=  277; v :=  130;
            when others => y := 365; u :=  296; v :=   12;
        end case;
        return std_logic_vector(to_unsigned(y, 10))
             & std_logic_vector(to_signed(u, 10))
             & std_logic_vector(to_signed(v, 10));
    end function;

    -- Heat: 16 steps by edge strength, dark red -> orange -> yellow -> white.
    function f_heat(i : unsigned(3 downto 0)) return std_logic_vector is
        variable y, u, v : integer;
    begin
        case to_integer(i) is
            when  0 => y :=  78; u :=  133; v :=  -45;
            when  1 => y := 107; u :=  183; v :=  -62;
            when  2 => y := 137; u :=  234; v :=  -79;
            when  3 => y := 166; u :=  284; v :=  -96;
            when  4 => y := 232; u :=  308; v := -134;
            when  5 => y := 319; u :=  316; v := -184;
            when  6 => y := 388; u :=  292; v := -224;
            when  7 => y := 445; u :=  250; v := -257;
            when  8 => y := 503; u :=  208; v := -290;
            when  9 => y := 561; u :=  166; v := -324;
            when 10 => y := 623; u :=  121; v := -337;
            when 11 => y := 696; u :=   68; v := -301;
            when 12 => y := 734; u :=   40; v := -246;
            when 13 => y := 749; u :=   29; v := -177;
            when 14 => y := 764; u :=   18; v := -109;
            when others => y := 823; u :=    7; v :=  -43;
        end case;
        return std_logic_vector(to_unsigned(y, 10))
             & std_logic_vector(to_signed(u, 10))
             & std_logic_vector(to_signed(v, 10));
    end function;

    ----------------------------------------------------------------------
    -- Line buffers (BRAM)
    ----------------------------------------------------------------------
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_lb2 is array (0 to 2047) of std_logic_vector(1 downto 0);
    type t_lb4 is array (0 to 2047) of std_logic_vector(3 downto 0);
    type t_lb1 is array (0 to 2047) of std_logic_vector(0 downto 0);

    signal lb1, lb2, lb3, lb4 : t_lb8 := (others => (others => '0'));  -- luma
    signal mb1, mb2           : t_lb8 := (others => (others => '0'));  -- mag
    signal db1                : t_lb2 := (others => (others => '0'));  -- dir
    signal ab1                : t_lb1 := (others => (others => '0'));  -- accept
    signal dd1                : t_lb4 := (others => (others => '0'));  -- dilate

    signal q1, q2, q3, q4 : std_logic_vector(7 downto 0) := (others => '0');
    signal mq1, mq2       : std_logic_vector(7 downto 0) := (others => '0');
    signal dq             : std_logic_vector(1 downto 0) := (others => '0');
    signal aq             : std_logic_vector(0 downto 0) := (others => '0');
    signal ddq            : std_logic_vector(3 downto 0) := (others => '0');

    -- Dry video delay ring (overlay + mix + Source colour): 32 x 30 EBR
    type t_ring is array (0 to 31) of std_logic_vector(29 downto 0);
    signal vring : t_ring := (others => (others => '0'));
    signal vr_wp : unsigned(4 downto 0) := (others => '0');
    signal vr_qa, vr_qb, vr_qc : std_logic_vector(29 downto 0) := (others => '0');
    signal vr_q  : std_logic_vector(29 downto 0) := (others => '0');
    signal vr_q2 : std_logic_vector(29 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Valid chain / counters
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 20) := (others => '0');

    signal lrx, lwx : unsigned(10 downto 0) := (others => '0');  -- luma rd/wr
    signal mrx, mwx : unsigned(10 downto 0) := (others => '0');  -- mag rd/wr
    signal abr, abw : unsigned(10 downto 0) := (others => '0');  -- accept
    signal drx, dwx : unsigned(10 downto 0) := (others => '0');  -- dilate
    signal cbx      : unsigned(10 downto 0) := (others => '0');  -- blank @S15
    signal gbx      : unsigned(10 downto 0) := (others => '0');  -- guard @S18
    signal obx      : unsigned(10 downto 0) := (others => '0');  -- guard @S19

    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_aline : unsigned(10 downto 0) := (others => '0');  -- 1080 rows
    signal s_seen  : std_logic := '0';
    signal s_fseen : std_logic := '0';   -- saw active video this field

    ----------------------------------------------------------------------
    -- Frame-latched controls
    ----------------------------------------------------------------------
    signal s_chalk  : std_logic := '0';                       -- S9 Style
    signal s_cmode  : unsigned(1 downto 0) := (others => '0'); -- K6 Color
    signal s_thr    : unsigned(7 downto 0) := (others => '0');
    signal s_thrlo  : unsigned(7 downto 0) := (others => '0');
    signal s_base   : signed(11 downto 0) := to_signed(512, 12);  -- 512+bright
    signal s_glo    : unsigned(2 downto 0) := "000";
    signal s_ghi    : unsigned(2 downto 0) := "010";
    signal s_pmask  : std_logic_vector(9 downto 0) := (others => '1');
    signal s_thick  : unsigned(2 downto 0) := (others => '0');
    signal s_smooth : std_logic := '0';
    signal s_inv    : std_logic := '0';
    signal s_col    : std_logic := '0';   -- any colour mode but White
    signal s_ovl    : std_logic := '0';
    signal s_gview  : std_logic := '0';

    -- P12 Sensitivity glide (10.3 fixed point, eased once per field)
    signal s_p12   : unsigned(9 downto 0) := (others => '0');
    signal s_gacc  : unsigned(12 downto 0) := to_unsigned(768 * 8, 13);
    signal s_gstep : signed(13 downto 0) := (others => '0');
    signal s_sv    : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S1-S4 preprocess
    ----------------------------------------------------------------------
    signal a1_c        : signed(11 downto 0) := (others => '0');
    signal a2_lo, a2_hi : signed(15 downto 0) := (others => '0');
    signal a3_y        : unsigned(9 downto 0) := (others => '0');
    signal a4_y8       : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S5 window + column sums
    ----------------------------------------------------------------------
    type t_row5 is array (0 to 4) of unsigned(7 downto 0);
    type t_win  is array (0 to 4) of t_row5;
    signal win : t_win := (others => (others => (others => '0')));

    signal a5_ga, a5_gb, a5_gc : unsigned(9 downto 0) := (others => '0');

    -- Compass hue + Heat strength: Sobel over raw window rows 2..4
    signal a5_hdy, hd1, hd2 : signed(8 downto 0) := (others => '0');
    signal h_gx, h_gy       : signed(10 downto 0) := (others => '0');
    signal h_ax, h_ay       : unsigned(10 downto 0) := (others => '0');
    signal h_a, h_b         : unsigned(10 downto 0) := (others => '0');
    signal h_sx1, h_sy1, h_sx2, h_sy2, h_sx3, h_sy3 : std_logic := '0';
    signal h_sw2, h_sw3     : std_logic := '0';
    signal h_na, h_nd       : std_logic := '0';
    signal h_ht             : unsigned(3 downto 0) := (others => '0');
    -- hsr entries: heat (7..4) & hue sector (3..0)
    type t_hsr is array (0 to 11) of unsigned(7 downto 0);
    signal hsr : t_hsr := (others => (others => '0'));
    signal src_dy           : unsigned(9 downto 0) := (others => '0');
    signal src_du, src_dv   : signed(9 downto 0) := (others => '0');
    signal pal_dy           : unsigned(9 downto 0) := (others => '0');
    signal pal_du, pal_dv   : signed(9 downto 0) := (others => '0');
    signal pf_y, pf_u, pf_v : unsigned(9 downto 0) := (others => '0');
    signal pr_y             : unsigned(9 downto 0) := (others => '0');
    signal pr_u, pr_v       : signed(9 downto 0) := (others => '0');
    signal g_y              : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S6-S8 blur / e-columns
    ----------------------------------------------------------------------
    signal g1c, g2c   : unsigned(9 downto 0) := (others => '0');
    signal t1c, t2c   : unsigned(9 downto 0) := (others => '0');
    signal b1c, b2c   : unsigned(9 downto 0) := (others => '0');

    signal a7_bt, a7_bm, a7_bb : unsigned(7 downto 0) := (others => '0');
    signal a8_e0, a8_e1, a8_e2 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S9-S14 Sobel / Canny direction / Morph
    ----------------------------------------------------------------------
    signal a9_ss    : unsigned(9 downto 0) := (others => '0');
    signal a9_dy    : signed(8 downto 0) := (others => '0');
    signal a9_e1    : unsigned(7 downto 0) := (others => '0');
    signal ss1, ss2 : unsigned(9 downto 0) := (others => '0');
    signal dy1, dy2 : signed(8 downto 0) := (others => '0');

    signal a10_gxs, a10_gys : signed(11 downto 0) := (others => '0');
    -- Morph: column max/min of the effective 3 rows, then 3-column tree
    signal a9_m02, a9_n02   : unsigned(7 downto 0) := (others => '0');
    signal a10_cmx, a10_cmn : unsigned(7 downto 0) := (others => '0');
    signal cmx1, cmx2, cmn1, cmn2 : unsigned(7 downto 0) := (others => '0');
    signal a11_pmx, a11_pmn, a11_c2x, a11_c2n : unsigned(7 downto 0) := (others => '0');
    signal a12_mx, a12_mn   : unsigned(7 downto 0) := (others => '0');
    signal a13_mor, mor_d   : unsigned(7 downto 0) := (others => '0');

    signal a11_ms  : unsigned(11 downto 0) := (others => '0');
    signal a11_ax, a11_ay : unsigned(11 downto 0) := (others => '0');
    signal a11_ds  : std_logic := '0';
    signal ms_d1, ms_d2, ms_d3 : unsigned(11 downto 0) := (others => '0');

    signal a12_dir : unsigned(1 downto 0) := (others => '0');
    signal dir_d1, dir_d2 : unsigned(1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S15-S19 mag select, NMS, threshold, dilate
    ----------------------------------------------------------------------
    signal a15_mag : unsigned(7 downto 0) := (others => '0');
    signal a15_dir : unsigned(1 downto 0) := (others => '0');
    signal mq1r, mq2r : unsigned(7 downto 0) := (others => '0');

    type t_m3 is array (0 to 2) of unsigned(7 downto 0);
    signal mr0, mr1, mr2 : t_m3 := (others => (others => '0'));
    signal dird1, dird2, dird3 : unsigned(1 downto 0) := (others => '0');

    signal a17_mag : unsigned(7 downto 0) := (others => '0');
    signal a18_mag : unsigned(7 downto 0) := (others => '0');
    signal a18_bit : std_logic := '0';
    signal leftacc : std_logic := '0';
    signal aw1, aw2 : std_logic := '0';

    signal ddqr : std_logic_vector(3 downto 0) := (others => '0');
    signal v1, v2, v3, v4 : std_logic := '0';  -- lines -1..-4, column 0
    -- Separable dilate (v1.3.2): vcr(c) = column c of the window ORed over
    -- the enabled rows; s_cen / s_ren = enabled columns / rows (col 2 and
    -- row 1 = line-1 are the 1-px centre and always on).
    signal vcr : std_logic_vector(1 to 4) := (others => '0');
    signal s_cen : std_logic_vector(0 to 4) := "00100";
    signal s_ren : std_logic_vector(0 to 4) := "01000";
    signal a19_bit : std_logic := '0';
    signal mg_0, mg_1 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S20 compose
    ----------------------------------------------------------------------
    signal a20_y, a20_u, a20_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Sync delay + mix
    ----------------------------------------------------------------------
    type t_bit_shift is array (0 to C_TOTAL_LATENCY - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

    -- K2 Mix, one registered copy per interpolator: a single shared t
    -- register fans out to all three multipliers and its first routing
    -- hop set Fmax (~74.8 MHz HD); keep stops opt_merge re-merging them
    signal s_mix_ty, s_mix_tu, s_mix_tv : unsigned(C_DW - 1 downto 0)
        := (others => '0');
    attribute keep : boolean;
    attribute keep of s_mix_ty : signal is true;
    attribute keep of s_mix_tu : signal is true;
    attribute keep of s_mix_tv : signal is true;
    signal s_interp_y_result, s_interp_u_result, s_interp_v_result
        : unsigned(C_DW - 1 downto 0);
    signal s_interp_y_valid, s_interp_u_valid, s_interp_v_valid : std_logic;

begin

    p_mix_t : process(clk)
    begin
        if rising_edge(clk) then
            s_mix_ty <= unsigned(registers_in(1));   -- K2 Mix
            s_mix_tu <= unsigned(registers_in(1));
            s_mix_tv <= unsigned(registers_in(1));
        end if;
    end process p_mix_t;

    ----------------------------------------------------------------------
    -- Main pipeline
    ----------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sum   : signed(19 downto 0);
        variable v_v     : signed(12 downto 0);
        variable v_wc    : t_row5;
        variable v_e02   : unsigned(8 downto 0);
        variable v_p10   : unsigned(9 downto 0);
        variable v_m     : unsigned(7 downto 0);
        variable v_na, v_nb : unsigned(7 downto 0);
        variable v_keep  : std_logic;
        variable v_str, v_wk, v_acc : std_logic;
        variable v_bit   : std_logic;
        variable v_vc0   : std_logic;
        variable v_dw, v_dh : integer range 1 to 5;
        variable v_g6    : unsigned(5 downto 0);
        variable v_k1    : unsigned(9 downto 0);
        variable v_p3    : unsigned(2 downto 0);
        variable v_hf    : unsigned(2 downto 0);
        variable v_hq    : unsigned(4 downto 0);
        variable v_hs    : unsigned(4 downto 0);
        variable v_b5, v_b3 : unsigned(13 downto 0);
        variable v_gm    : unsigned(7 downto 0);
        variable v_gl    : unsigned(10 downto 0);
        variable v_pal   : std_logic_vector(29 downto 0);
        variable v_ga    : signed(13 downto 0);
        variable v_sy    : unsigned(9 downto 0);
        variable v_su, v_sv : signed(9 downto 0);
        variable v_ca, v_cb : signed(10 downto 0);
        variable v_cd    : signed(11 downto 0);
        variable v_inv4  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- valid chain
            av(1) <= data_in.avid;
            for i in 2 to 20 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen  <= '1';
                s_fseen <= '1';
            end if;

            ------------------------------------------------------------------
            -- S1: center luma for contrast (pivot mid-grey)
            ------------------------------------------------------------------
            if data_in.avid = '1' then
                a1_c <= signed(resize(unsigned(data_in.y), 12)) - 512;
            end if;

            ------------------------------------------------------------------
            -- S2: contrast gain split into two 12x3 partial products
            ------------------------------------------------------------------
            if av(1) = '1' then
                a2_lo <= a1_c * signed(resize(s_glo, 4));
                a2_hi <= a1_c * signed(resize(s_ghi, 4));
            end if;

            ------------------------------------------------------------------
            -- S3: combine, add base (512 + brightness), clamp
            ------------------------------------------------------------------
            if av(2) = '1' then
                v_sum := resize(a2_lo, 20) + shift_left(resize(a2_hi, 20), 3);
                v_v   := resize(s_base, 13) + resize(shift_right(v_sum, 4), 13);
                if v_v < 0 then
                    a3_y <= (others => '0');
                elsif v_v > 1023 then
                    a3_y <= (others => '1');
                else
                    a3_y <= unsigned(v_v(9 downto 0));
                end if;
            end if;

            ------------------------------------------------------------------
            -- S4: posterize, reduce to 8 bit
            ------------------------------------------------------------------
            if av(3) = '1' then
                v_p10 := a3_y and unsigned(s_pmask);
                a4_y8 <= v_p10(9 downto 2);
                lrx   <= lrx + 1;   -- luma reads were presented this cycle
            end if;

            ------------------------------------------------------------------
            -- S5: 5x5 window shift, buffer rotate writes happen in the BRAM
            -- processes; per-column vertical sums.
            -- Rows: w0 = current line ... w4 = 4 lines up (top of window).
            ------------------------------------------------------------------
            if av(4) = '1' then
                v_wc(0) := a4_y8;
                v_wc(1) := unsigned(q1);
                v_wc(2) := unsigned(q2);
                v_wc(3) := unsigned(q3);
                v_wc(4) := unsigned(q4);
                for r in 0 to 4 loop
                    win(r)(0) <= v_wc(r);
                    for c in 1 to 4 loop
                        win(r)(c) <= win(r)(c - 1);
                    end loop;
                end loop;

                -- [1 2 1] vertical sums for blurred rows 1..3
                a5_ga <= resize(v_wc(0), 10) + resize(v_wc(2), 10)
                         + shift_left(resize(v_wc(1), 10), 1);
                a5_gb <= resize(v_wc(1), 10) + resize(v_wc(3), 10)
                         + shift_left(resize(v_wc(2), 10), 1);
                a5_gc <= resize(v_wc(2), 10) + resize(v_wc(4), 10)
                         + shift_left(resize(v_wc(3), 10), 1);
                -- colour Sobel column term: row 2 (lower) - row 4 (upper)
                a5_hdy <= signed(resize(v_wc(2), 9)) - signed(resize(v_wc(4), 9));

                lwx <= lwx + 1;
            end if;

            ------------------------------------------------------------------
            -- S6: start the column delay chains
            ------------------------------------------------------------------
            if av(5) = '1' then
                g1c <= a5_gb;  g2c <= g1c;
                t1c <= a5_ga;  t2c <= t1c;
                b1c <= a5_gc;  b2c <= b1c;
                hd1 <= a5_hdy; hd2 <= hd1;
            end if;

            ------------------------------------------------------------------
            -- S7: 3x3 Gaussian blur (horizontal [1 2 1] over the vertical
            -- sums, /16); colour Sobel
            ------------------------------------------------------------------
            if av(6) = '1' then
                a7_bt <= resize(shift_right(resize(t2c, 12)
                         + shift_left(resize(t1c, 12), 1)
                         + resize(a5_ga, 12), 4), 8);
                a7_bm <= resize(shift_right(resize(g2c, 12)
                         + shift_left(resize(g1c, 12), 1)
                         + resize(a5_gb, 12), 4), 8);
                a7_bb <= resize(shift_right(resize(b2c, 12)
                         + shift_left(resize(b1c, 12), 1)
                         + resize(a5_gc, 12), 4), 8);
                -- colour Sobel, centered on this stage's column (b1c):
                -- gx = right - left, gy = lower - upper ([1 2 1] across)
                h_gx <= signed(resize(a5_gc, 11)) - signed(resize(b2c, 11));
                h_gy <= resize(a5_hdy, 11) + shift_left(resize(hd1, 11), 1)
                        + resize(hd2, 11);
            end if;

            ------------------------------------------------------------------
            -- S8: Smooth mux -> effective 3-row column
            -- (e0 = upper row, e1 = center, e2 = lower row)
            ------------------------------------------------------------------
            if av(7) = '1' then
                -- colour: magnitudes + signs
                h_ax  <= f_absu(h_gx);
                h_ay  <= f_absu(h_gy);
                h_sx1 <= h_gx(10);
                h_sy1 <= h_gy(10);
                if s_smooth = '1' then
                    a8_e0 <= a7_bt;
                    a8_e1 <= a7_bm;
                    a8_e2 <= a7_bb;
                else
                    a8_e0 <= win(1)(2);
                    a8_e1 <= win(2)(2);
                    a8_e2 <= win(3)(2);
                end if;
            end if;

            ------------------------------------------------------------------
            -- S9: Sobel column sums + histories
            ------------------------------------------------------------------
            if av(8) = '1' then
                v_e02 := resize(a8_e0, 9) + a8_e2;
                a9_ss <= resize(v_e02, 10) + shift_left(resize(a8_e1, 10), 1);
                a9_dy <= signed(resize(a8_e0, 9)) - signed(resize(a8_e2, 9));
                a9_e1 <= a8_e1;
                ss1 <= a9_ss;  ss2 <= ss1;
                dy1 <= a9_dy;  dy2 <= dy1;
                -- Morph: max/min of the outer pair (one shared compare)
                if a8_e0 > a8_e2 then
                    a9_m02 <= a8_e0;  a9_n02 <= a8_e2;
                else
                    a9_m02 <= a8_e2;  a9_n02 <= a8_e0;
                end if;
                -- colour: fold to the first octant
                if h_ay > h_ax then
                    h_sw2 <= '1';  h_a <= h_ay;  h_b <= h_ax;
                else
                    h_sw2 <= '0';  h_a <= h_ax;  h_b <= h_ay;
                end if;
                h_sx2 <= h_sx1;
                h_sy2 <= h_sy1;
            end if;

            ------------------------------------------------------------------
            -- S10: Sobel gradients (center = middle column of the
            -- histories); Morph column max/min; colour sector tests
            ------------------------------------------------------------------
            if av(9) = '1' then
                a10_gxs <= signed(resize(a9_ss, 12)) - signed(resize(ss2, 12));
                a10_gys <= resize(a9_dy, 12) + shift_left(resize(dy1, 12), 1)
                           + resize(dy2, 12);
                -- Morph: finish this column's max/min with the center row
                if a9_e1 > a9_m02 then
                    a10_cmx <= a9_e1;
                else
                    a10_cmx <= a9_m02;
                end if;
                if a9_e1 < a9_n02 then
                    a10_cmn <= a9_e1;
                else
                    a10_cmn <= a9_n02;
                end if;
                -- compass: sector boundaries at tan 11.25 (~1/5) and
                -- tan 33.75 (~2/3) inside the folded octant
                v_b5 := shift_left(resize(h_b, 14), 2) + resize(h_b, 14);
                v_b3 := shift_left(resize(h_b, 14), 1) + resize(h_b, 14);
                if v_b5 < resize(h_a, 14) then
                    h_na <= '1';
                else
                    h_na <= '0';
                end if;
                if v_b3 > shift_left(resize(h_a, 14), 1) then
                    h_nd <= '1';
                else
                    h_nd <= '0';
                end if;
                -- heat: strength = max(|gx|,|gy|) / 32, saturating at 15
                -- (a full-scale luma step is ~1020)
                if h_a(10 downto 9) /= "00" then
                    h_ht <= (others => '1');
                else
                    h_ht <= h_a(8 downto 5);
                end if;
                h_sw3 <= h_sw2;
                h_sx3 <= h_sx2;
                h_sy3 <= h_sy2;
            end if;

            ------------------------------------------------------------------
            -- S11: |gx|+|gy|, Canny |gx| |gy| for direction, Morph tree
            -- level 1, colour sector
            ------------------------------------------------------------------
            if av(10) = '1' then
                a11_ms  <= resize(f_absu(a10_gxs), 12) + f_absu(a10_gys);
                a11_ax  <= f_absu(a10_gxs);
                a11_ay  <= f_absu(a10_gys);
                a11_ds  <= a10_gxs(11) xor a10_gys(11);
                -- Morph: 3-column max/min tree, level 1
                if a10_cmx > cmx1 then
                    a11_pmx <= a10_cmx;
                else
                    a11_pmx <= cmx1;
                end if;
                if a10_cmn < cmn1 then
                    a11_pmn <= a10_cmn;
                else
                    a11_pmn <= cmn1;
                end if;
                a11_c2x <= cmx2;
                a11_c2n <= cmn2;
                cmx1 <= a10_cmx;  cmx2 <= cmx1;
                cmn1 <= a10_cmn;  cmn2 <= cmn1;
                -- compass: 16-sector angle, 0 = +x, counting toward +y
                -- (screen down); f = 0 near axis, 1 mid, 2 near diagonal
                if h_na = '1' then
                    v_hf := "000";
                elsif h_nd = '1' then
                    v_hf := "010";
                else
                    v_hf := "001";
                end if;
                if h_sw3 = '1' then
                    v_hq := to_unsigned(4, 5) - resize(v_hf, 5);
                else
                    v_hq := resize(v_hf, 5);
                end if;
                if h_sx3 = '0' and h_sy3 = '0' then
                    v_hs := v_hq;
                elsif h_sx3 = '1' and h_sy3 = '0' then
                    v_hs := to_unsigned(8, 5) - v_hq;
                elsif h_sx3 = '1' then
                    v_hs := to_unsigned(8, 5) + v_hq;
                else
                    v_hs := to_unsigned(16, 5) - v_hq;
                end if;
                hsr(0) <= h_ht & v_hs(3 downto 0);
                for i in 1 to 11 loop
                    hsr(i) <= hsr(i - 1);
                end loop;
            end if;

            ------------------------------------------------------------------
            -- S12: Canny direction quantize; Morph level 2
            ------------------------------------------------------------------
            if av(11) = '1' then
                if a11_ay > shift_left(resize(a11_ax, 13), 1) then
                    a12_dir <= "10";                        -- vertical grad
                elsif a11_ax > shift_left(resize(a11_ay, 13), 1) then
                    a12_dir <= "00";                        -- horizontal grad
                elsif a11_ds = '1' then
                    a12_dir <= "11";                        -- gx,gy opposite
                else
                    a12_dir <= "01";
                end if;
                ms_d1 <= a11_ms;
                if a11_pmx > a11_c2x then
                    a12_mx <= a11_pmx;
                else
                    a12_mx <= a11_c2x;
                end if;
                if a11_pmn < a11_c2n then
                    a12_mn <= a11_pmn;
                else
                    a12_mn <= a11_c2n;
                end if;
            end if;

            ------------------------------------------------------------------
            -- S13: Morph max - min; delays
            ------------------------------------------------------------------
            if av(12) = '1' then
                dir_d1  <= a12_dir;
                ms_d2   <= ms_d1;
                a13_mor <= a12_mx - a12_mn;
            end if;

            ------------------------------------------------------------------
            -- S14: delays (keeps the v1.2 latency)
            ------------------------------------------------------------------
            if av(13) = '1' then
                dir_d2 <= dir_d1;
                ms_d3  <= ms_d2;
                mor_d  <= a13_mor;
                mrx <= mrx + 1;   -- mag-window reads presented this cycle
            end if;

            ------------------------------------------------------------------
            -- S15: normalize to ~0..255, style mux, blanking
            ------------------------------------------------------------------
            if av(14) = '1' then
                if s_chalk = '1' then
                    v_m := mor_d;
                else
                    v_m := f_c255(shift_right(ms_d3, 2));
                end if;
                -- blank warm-up lines (window straddles previous frame) and
                -- left columns (shift chains carry the previous line's tail)
                if s_aline < 6 or cbx < 12 then
                    v_m := (others => '0');
                end if;
                a15_mag <= v_m;
                a15_dir <= dir_d2;
                mq1r <= unsigned(mq1);
                mq2r <= unsigned(mq2);
                dird1 <= unsigned(dq);
                cbx  <= cbx + 1;
            end if;

            ------------------------------------------------------------------
            -- S16: 3x3 magnitude window (rows: mr0 = current line = below,
            -- mr1 = center line, mr2 = up; col 0 = right)
            ------------------------------------------------------------------
            if av(15) = '1' then
                mr0(0) <= a15_mag;  mr0(1) <= mr0(0);  mr0(2) <= mr0(1);
                mr1(0) <= mq1r;     mr1(1) <= mr1(0);  mr1(2) <= mr1(1);
                mr2(0) <= mq2r;     mr2(1) <= mr2(0);  mr2(2) <= mr2(1);
                dird2 <= dird1;
                dird3 <= dird2;
                mwx <= mwx + 1;
            end if;

            ------------------------------------------------------------------
            -- S17: non-maximum suppression (Fine), passthrough for Chalk
            ------------------------------------------------------------------
            if av(16) = '1' then
                case to_integer(dird3) is
                    when 0 =>      -- horizontal gradient: left/right
                        v_na := mr1(0);  v_nb := mr1(2);
                    when 2 =>      -- vertical gradient: up/down
                        v_na := mr2(1);  v_nb := mr0(1);
                    -- gx = right-left, gy = lower-upper, so same signs
                    -- point down-right (v1.0 had the diagonals swapped)
                    when 3 =>      -- gx,gy opposite signs: UR / DL
                        v_na := mr2(0);  v_nb := mr0(2);
                    when others => -- same signs: DR / UL
                        v_na := mr0(0);  v_nb := mr2(2);
                end case;
                v_keep := '0';
                if mr1(1) >= v_na and mr1(1) > v_nb then
                    v_keep := '1';
                end if;
                if s_chalk = '0' and v_keep = '0' then
                    a17_mag <= (others => '0');
                else
                    a17_mag <= mr1(1);
                end if;
            end if;

            ------------------------------------------------------------------
            -- S18: threshold; Fine double threshold + causal hysteresis
            ------------------------------------------------------------------
            if av(17) = '1' then
                v_str := '0';
                v_wk  := '0';
                if a17_mag > s_thr then
                    v_str := '1';
                end if;
                if a17_mag > s_thrlo then
                    v_wk := '1';
                end if;
                if s_chalk = '0' then
                    v_acc := v_str or (v_wk and (leftacc or aq(0) or aw1 or aw2));
                else
                    v_acc := v_str;
                end if;
                if gbx < 16 then
                    v_acc := '0';
                end if;
                a18_bit <= v_acc;
                leftacc <= v_acc;
                -- gradient view shows strength only where the threshold
                -- passes, so Sensitivity stays live in every view
                if v_str = '1' then
                    a18_mag <= a17_mag;
                else
                    a18_mag <= (others => '0');
                end if;
                abw <= abw + 1;
                gbx <= gbx + 1;
            end if;
            if av(16) = '1' then
                aw1 <= aq(0);
                aw2 <= aw1;
                drx <= drx + 1;
            end if;
            if av(15) = '1' then
                abr <= abr + 1;  -- accept reads lead the write by two columns
            end if;

            ------------------------------------------------------------------
            -- S19: thickness dilate over a 5x5 bit window
            -- rows: 0 = current line (a18_bit), 1 = line-1 (1 px center),
            -- 2..4 = lines -2..-4; column c = c pixels back (2 = center).
            -- Separable: each column is ORed over the enabled rows as it
            -- enters (v_vc0), then shifted along vcr, and the enabled columns
            -- are ORed -- the same row x column squares as v1.3.1's per-size
            -- trees (1080p bit-identical), with independent width / height
            -- so the stroke can follow the mode (see s_cen / s_ren).
            ------------------------------------------------------------------
            if av(17) = '1' then
                ddqr <= ddq;
            end if;
            if av(18) = '1' then
                v1 <= ddqr(3);
                v2 <= ddqr(2);
                v3 <= ddqr(1);
                v4 <= ddqr(0);
                v_vc0 := (a18_bit and s_ren(0)) or v1
                         or (v2 and s_ren(2)) or (v3 and s_ren(3))
                         or (v4 and s_ren(4));
                vcr(1) <= v_vc0;
                for i in 2 to 4 loop
                    vcr(i) <= vcr(i - 1);
                end loop;
                v_bit := (v_vc0 and s_cen(0)) or (vcr(1) and s_cen(1))
                         or vcr(2) or (vcr(3) and s_cen(3))
                         or (vcr(4) and s_cen(4));
                -- left guard, and a top guard: the edge layer trails the
                -- input by up to 7 rows (window + NMS + 5x5 dilate), so the
                -- first lines would show the previous frame's bottom rows
                -- still held in the mag / accept / dilate line buffers
                if obx < 20 or s_aline < 6 then
                    v_bit := '0';
                end if;
                a19_bit <= v_bit;
                if s_aline < 6 then
                    mg_0 <= (others => '0');
                else
                    mg_0 <= a18_mag;
                end if;
                mg_1 <= mg_0;
                dwx <= dwx + 1;
                obx <= obx + 1;
                -- gradient level (inverted = not-edges); grey ramp 64..924
                -- and the edge colour scaled by its top 3 bits (8 steps
                -- of shift-add: three soft multipliers cost 570 LUTs)
                if s_inv = '1' then
                    v_gm := not mg_1;
                else
                    v_gm := mg_1;
                end if;
                v_gl := shift_left(resize(v_gm, 11), 2)
                        - resize(shift_right(v_gm, 1), 11)
                        - resize(shift_right(v_gm, 3), 11) + 64;
                g_y  <= v_gl(9 downto 0);
                v_sy := (others => '0');
                v_su := (others => '0');
                v_sv := (others => '0');
                if v_gm(7) = '1' then
                    v_sy := shift_right(pal_dy, 1);
                    v_su := shift_right(pal_du, 1);
                    v_sv := shift_right(pal_dv, 1);
                end if;
                if v_gm(6) = '1' then
                    v_sy := v_sy + shift_right(pal_dy, 2);
                    v_su := v_su + shift_right(pal_du, 2);
                    v_sv := v_sv + shift_right(pal_dv, 2);
                end if;
                if v_gm(5) = '1' then
                    v_sy := v_sy + shift_right(pal_dy, 3);
                    v_su := v_su + shift_right(pal_du, 3);
                    v_sv := v_sv + shift_right(pal_dv, 3);
                end if;
                pr_y <= v_sy;
                pr_u <= v_su;
                pr_v <= v_sv;
                pf_y <= pal_dy + 64;
                pf_u <= unsigned(pal_du) + 512;
                pf_v <= unsigned(pal_dv) + 512;
            end if;

            ------------------------------------------------------------------
            -- S17 (side): Source colour from the dry ring, for the column
            -- composed at S20: Y lifted to 400 + y/2, chroma x2 clamped to
            -- +-448; Invert takes the complementary chroma.
            ------------------------------------------------------------------
            src_dy <= resize(shift_right(unsigned(vr_qa(29 downto 20)), 1), 10) + 336;
            if s_inv = '1' then
                v_ca := to_signed(512, 11);
                v_cb := signed(resize(unsigned(vr_qa(19 downto 10)), 11));
            else
                v_ca := signed(resize(unsigned(vr_qa(19 downto 10)), 11));
                v_cb := to_signed(512, 11);
            end if;
            v_cd := shift_left(resize(v_ca - v_cb, 12), 1);
            if v_cd > 448 then
                src_du <= to_signed(448, 10);
            elsif v_cd < -448 then
                src_du <= to_signed(-448, 10);
            else
                src_du <= v_cd(9 downto 0);
            end if;
            if s_inv = '1' then
                v_ca := to_signed(512, 11);
                v_cb := signed(resize(unsigned(vr_qa(9 downto 0)), 11));
            else
                v_ca := signed(resize(unsigned(vr_qa(9 downto 0)), 11));
                v_cb := to_signed(512, 11);
            end if;
            v_cd := shift_left(resize(v_ca - v_cb, 12), 1);
            if v_cd > 448 then
                src_dv <= to_signed(448, 10);
            elsif v_cd < -448 then
                src_dv <= to_signed(-448, 10);
            else
                src_dv <= v_cd(9 downto 0);
            end if;

            ------------------------------------------------------------------
            -- S18 (side): edge colour for the column composed at S20;
            -- hsr(11) is the hue/heat 5 columns behind this token (= the
            -- dilated output column).  Invert: complementary hue, reversed
            -- heat ramp.
            ------------------------------------------------------------------
            if av(17) = '1' then
                v_inv4 := (others => s_inv);
                case to_integer(s_cmode) is
                    when 3 =>
                        v_pal := f_heat(hsr(11)(7 downto 4) xor v_inv4);
                    when 2 =>
                        v_pal := std_logic_vector(src_dy)
                               & std_logic_vector(src_du)
                               & std_logic_vector(src_dv);
                    when others =>
                        v_inv4 := (3 => s_inv, others => '0');
                        v_pal := f_pal(hsr(11)(3 downto 0) xor v_inv4);
                end case;
                pal_dy <= unsigned(v_pal(29 downto 20));
                pal_du <= signed(v_pal(19 downto 10));
                pal_dv <= signed(v_pal(9 downto 0));
            end if;

            ------------------------------------------------------------------
            -- S20: compose the wet pixel; neutral black outside active video
            ------------------------------------------------------------------
            if av(19) = '1' then
                v_bit := a19_bit xor s_inv;
                if s_gview = '1' then                    -- Gradient view
                    if s_col = '1' then
                        a20_y <= pr_y + 64;
                        a20_u <= unsigned(pr_u) + 512;
                        a20_v <= unsigned(pr_v) + 512;
                    else
                        a20_y <= g_y;
                        a20_u <= to_unsigned(512, C_DW);
                        a20_v <= to_unsigned(512, C_DW);
                    end if;
                elsif a19_bit = '1' and s_col = '1' then -- coloured edge
                    a20_y <= pf_y;
                    a20_u <= pf_u;
                    a20_v <= pf_v;
                elsif s_ovl = '1' and a19_bit = '0' then -- overlay ground
                    a20_y <= unsigned(vr_q(29 downto 20));
                    a20_u <= unsigned(vr_q(19 downto 10));
                    a20_v <= unsigned(vr_q(9 downto 0));
                else                                     -- white / black
                    if (s_ovl = '0' and v_bit = '1')
                       or (s_ovl = '1' and s_inv = '0') then
                        a20_y <= to_unsigned(940, C_DW);
                    else
                        a20_y <= to_unsigned(64, C_DW);
                    end if;
                    a20_u <= to_unsigned(512, C_DW);
                    a20_v <= to_unsigned(512, C_DW);
                end if;
            else
                a20_y <= to_unsigned(64, C_DW);
                a20_u <= to_unsigned(512, C_DW);
                a20_v <= to_unsigned(512, C_DW);
            end if;

            ------------------------------------------------------------------
            -- New line / new frame
            ------------------------------------------------------------------
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                mrx <= (others => '0');
                mwx <= (others => '0');
                abr <= (others => '0');
                abw <= (others => '0');
                drx <= (others => '0');
                dwx <= (others => '0');
                cbx <= (others => '0');
                gbx <= (others => '0');
                obx <= (others => '0');
                leftacc <= '0';
                if s_seen = '1' then
                    s_seen  <= '0';
                    s_aline <= s_aline + 1;
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_aline <= (others => '0');
                s_seen  <= '0';

                -- latch controls once per frame; K1 Thickness has 5 zones
                -- (1024/5), K5 Posterize 8, K6 Color 4.
                -- Labelled defaults: at program load the firmware writes the
                -- initial_value_label's INDEX (0..N-1) into the register, not
                -- a 0..1023 position (HW-measured 2026-10-02), so a value
                -- below the label count is taken as the label index itself.
                v_k1 := unsigned(registers_in(5));
                if v_k1 < 4 then
                    s_cmode <= v_k1(1 downto 0);
                else
                    s_cmode <= v_k1(9 downto 8);
                end if;
                -- P12 glide: ease 1/4 of the way once per field (the
                -- saw-active guard keeps serrated vsync to one step)
                if s_fseen = '1' then
                    s_fseen <= '0';
                    v_ga    := signed(resize(s_gacc, 14)) + s_gstep;
                    s_gacc  <= unsigned(v_ga(12 downto 0));
                end if;
                s_base <= to_signed(512, 12)
                          + (signed(resize(unsigned(registers_in(2)), 12)) - 512);
                v_g6  := unsigned(registers_in(3)(9 downto 4));
                s_glo <= v_g6(2 downto 0);
                s_ghi <= v_g6(5 downto 3);
                v_k1 := unsigned(registers_in(4));
                if v_k1 < 8 then
                    v_p3 := v_k1(2 downto 0);
                else
                    v_p3 := v_k1(9 downto 7);
                end if;
                case to_integer(v_p3) is
                    when 0      => s_pmask <= "1111111111";
                    when 1      => s_pmask <= "1111111000";
                    when 2      => s_pmask <= "1111110000";
                    when 3      => s_pmask <= "1111100000";
                    when 4      => s_pmask <= "1111000000";
                    when 5      => s_pmask <= "1110000000";
                    when 6      => s_pmask <= "1100000000";
                    when others => s_pmask <= "1000000000";
                end case;
                v_k1 := unsigned(registers_in(0));
                if v_k1 < 5 then
                    s_thick <= v_k1(2 downto 0);
                elsif v_k1 < 205 then s_thick <= to_unsigned(0, 3);
                elsif v_k1 < 410 then s_thick <= to_unsigned(1, 3);
                elsif v_k1 < 614 then s_thick <= to_unsigned(2, 3);
                elsif v_k1 < 819 then s_thick <= to_unsigned(3, 3);
                else                  s_thick <= to_unsigned(4, 3);
                end if;
                s_smooth <= registers_in(6)(0);
                s_inv    <= registers_in(6)(1);
                s_chalk  <= registers_in(6)(2);
                s_ovl    <= registers_in(6)(3);
                s_gview  <= registers_in(6)(4);
            end if;

            ------------------------------------------------------------------
            -- Resolution scaling (v1.3.2): K1 Thickness is a line weight in
            -- 1080-line pixels t = 1..5.  The dilate square's width / height
            -- in real pixels / field lines follow the mode's x / y step s so
            -- the stroke is about the same fraction of screen height
            -- everywhere: n = floor(t/s + 0.75), i.e. rounded with a bold
            -- bias (up from a quarter), 1-px floor.  The bias is deliberate:
            -- the pixel-scale detector finds fewer edges in a lower-res
            -- picture, and plain rounding left 720p at half the 1080p line
            -- coverage (2 px -> 1 px).  Steps: 720p 1.5, SD x 2.0, field-line
            -- y 1080i 2.0, 576p 1.875, 480p 2.25, 576i 3.75, 480i 4.44.
            -- 1080p keeps t x t.  Free-running decode of the timing ID
            -- (static while a mode runs) and the frame-latched s_thick.
            ------------------------------------------------------------------
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 =>          -- SD: x step 2.0 -> 1 1 2 2 3
                    case to_integer(s_thick) is
                        when 0 | 1  => v_dw := 1;
                        when 2 | 3  => v_dw := 2;
                        when others => v_dw := 3;
                    end case;
                when 5 | 6 | 14 =>              -- 720p: 1.5      -> 1 2 2 3 4
                    case to_integer(s_thick) is
                        when 0      => v_dw := 1;
                        when 1 | 2  => v_dw := 2;
                        when 3      => v_dw := 3;
                        when others => v_dw := 4;
                    end case;
                when others =>                  -- 1080p / 1080i: 1.0
                    case to_integer(s_thick) is
                        when 0      => v_dw := 1;
                        when 1      => v_dw := 2;
                        when 2      => v_dw := 3;
                        when 3      => v_dw := 4;
                        when others => v_dw := 5;
                    end case;
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 =>                       -- 480i: 4.44     -> 1 1 1 1 1
                    v_dh := 1;
                when 8 =>                       -- 576i: 3.75     -> 1 1 1 1 2
                    case to_integer(s_thick) is
                        when 0 | 1 | 2 | 3 => v_dh := 1;
                        when others        => v_dh := 2;
                    end case;
                when 4 =>                       -- 480p: 2.25     -> 1 1 2 2 2
                    case to_integer(s_thick) is
                        when 0 | 1  => v_dh := 1;
                        when others => v_dh := 2;
                    end case;
                when 1 | 2 | 10 | 12 =>         -- 1080i 2.0, 576p 1.875
                    case to_integer(s_thick) is  --              -> 1 1 2 2 3
                        when 0 | 1  => v_dh := 1;
                        when 2 | 3  => v_dh := 2;
                        when others => v_dh := 3;
                    end case;
                when 5 | 6 | 14 =>              -- 720p: 1.5      -> 1 2 2 3 4
                    case to_integer(s_thick) is
                        when 0      => v_dh := 1;
                        when 1 | 2  => v_dh := 2;
                        when 3      => v_dh := 3;
                        when others => v_dh := 4;
                    end case;
                when others =>                  -- 1080p: 1.0
                    case to_integer(s_thick) is
                        when 0      => v_dh := 1;
                        when 1      => v_dh := 2;
                        when 2      => v_dh := 3;
                        when 3      => v_dh := 4;
                        when others => v_dh := 5;
                    end case;
            end case;
            -- columns: 1 px {2}, 2 {1,2}, 3 {1..3}, 4 {0..3}, 5 {0..4};
            -- rows:    1 px {1}, 2 {0,1}, 3 {0..2}, 4 {0..3}, 5 {0..4}
            s_cen <= "00100";
            s_ren <= "01000";
            if v_dw >= 4 then s_cen(0) <= '1'; end if;
            if v_dw >= 2 then s_cen(1) <= '1'; end if;
            if v_dw >= 3 then s_cen(3) <= '1'; end if;
            if v_dw >= 5 then s_cen(4) <= '1'; end if;
            if v_dh >= 2 then s_ren(0) <= '1'; end if;
            if v_dh >= 3 then s_ren(2) <= '1'; end if;
            if v_dh >= 4 then s_ren(3) <= '1'; end if;
            if v_dh >= 5 then s_ren(4) <= '1'; end if;

            -- any colour mode but White (free-running, out of the vsync cone)
            if s_cmode /= 0 then
                s_col <= '1';
            else
                s_col <= '0';
            end if;

            ------------------------------------------------------------------
            -- Sensitivity -> threshold, free-running and registered (out of
            -- the vsync cone): thr = 61 - s*15/64, so 0% still shows the
            -- strongest edges and 100% (thr 1) floods the frame.  (v1.1's
            -- 240..1 range went black below ~80%: 0..100% was rescaled to
            -- cover the old 75..100%.)
            ------------------------------------------------------------------
            s_p12   <= unsigned(registers_in(7));
            s_gstep <= shift_right(signed(shift_left(resize(s_p12, 14), 3))
                                   - signed(resize(s_gacc, 14)), 2);
            s_sv    <= shift_right(s_gacc(12 downto 5), 2)
                       - shift_right(s_gacc(12 downto 5), 6);
            s_thr   <= to_unsigned(61, 8) - s_sv;
            s_thrlo <= '0' & s_thr(7 downto 1);

            ------------------------------------------------------------------
            -- Sync delay chain
            ------------------------------------------------------------------
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_pix;

    ----------------------------------------------------------------------
    -- Luma line buffers: read at lrx (one column ahead of the write),
    -- rotate rows by rewriting each read value one buffer down.
    ----------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(4) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(a4_y8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(4) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    p_lb3 : process(clk)
    begin
        if rising_edge(clk) then
            q3 <= lb3(to_integer(lrx));
            if av(4) = '1' then
                lb3(to_integer(lwx)) <= q2;
            end if;
        end if;
    end process p_lb3;

    p_lb4 : process(clk)
    begin
        if rising_edge(clk) then
            q4 <= lb4(to_integer(lrx));
            if av(4) = '1' then
                lb4(to_integer(lwx)) <= q3;
            end if;
        end if;
    end process p_lb4;

    ----------------------------------------------------------------------
    -- Magnitude + direction line buffers (NMS window)
    ----------------------------------------------------------------------
    p_mb1 : process(clk)
    begin
        if rising_edge(clk) then
            mq1 <= mb1(to_integer(mrx));
            if av(15) = '1' then
                mb1(to_integer(mwx)) <= std_logic_vector(a15_mag);
            end if;
        end if;
    end process p_mb1;

    p_mb2 : process(clk)
    begin
        if rising_edge(clk) then
            mq2 <= mb2(to_integer(mrx));
            if av(15) = '1' then
                mb2(to_integer(mwx)) <= std_logic_vector(mq1r);
            end if;
        end if;
    end process p_mb2;

    p_db1 : process(clk)
    begin
        if rising_edge(clk) then
            dq <= db1(to_integer(mrx));
            if av(15) = '1' then
                db1(to_integer(mwx)) <= std_logic_vector(a15_dir);
            end if;
        end if;
    end process p_db1;

    ----------------------------------------------------------------------
    -- Hysteresis accept-bit line buffer
    ----------------------------------------------------------------------
    p_ab1 : process(clk)
    begin
        if rising_edge(clk) then
            aq <= ab1(to_integer(abr));
            if av(17) = '1' then
                ab1(to_integer(abw)) <= (0 => a18_bit);
            end if;
        end if;
    end process p_ab1;

    ----------------------------------------------------------------------
    -- Dilate bit buffer: 4 lines packed (bit 3 = line-1 ... bit 0 = line-4)
    ----------------------------------------------------------------------
    p_dd1 : process(clk)
    begin
        if rising_edge(clk) then
            ddq <= dd1(to_integer(drx));
            if av(18) = '1' then
                dd1(to_integer(dwx)) <= a18_bit & v1 & v2 & v3;
            end if;
        end if;
    end process p_dd1;

    ----------------------------------------------------------------------
    -- Dry video delay ring: read 20 back, then 3 registers, so vr_q is 24
    -- clocks at compose and vr_q2 25 at the mix dry leg (5 columns behind
    -- the wet token = the dilated edge column); vr_qa feeds the Source
    -- colour 3 stages ahead of compose.
    ----------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            vring(to_integer(vr_wp)) <= data_in.y & data_in.u & data_in.v;
            vr_qa <= vring(to_integer(vr_wp - 20));
            vr_qb <= vr_qa;
            vr_qc <= vr_qb;
            vr_q  <= vr_qc;
            vr_q2 <= vr_q;
            vr_wp <= vr_wp + 1;
        end if;
    end process p_ring;

    ----------------------------------------------------------------------
    -- Dry/wet mix
    ----------------------------------------------------------------------
    interp_y_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(vr_q2(29 downto 20)),
            b      => a20_y,
            t      => s_mix_ty,
            result => s_interp_y_result,
            valid  => s_interp_y_valid
        );

    interp_u_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(vr_q2(19 downto 10)),
            b      => a20_u,
            t      => s_mix_tu,
            result => s_interp_u_result,
            valid  => s_interp_u_valid
        );

    interp_v_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(vr_q2(9 downto 0)),
            b      => a20_v,
            t      => s_mix_tv,
            result => s_interp_v_result,
            valid  => s_interp_v_valid
        );

    ----------------------------------------------------------------------
    -- Output
    ----------------------------------------------------------------------
    data_out.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    data_out.y <= std_logic_vector(s_interp_y_result);
    data_out.u <= std_logic_vector(s_interp_u_result);
    data_out.v <= std_logic_vector(s_interp_v_result);

end architecture edgelab;
