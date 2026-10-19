-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- prism.vhd
--
-- Edge detection with rainbow band-trail to the right of each edge: every
-- detected edge becomes a dispersion point, splitting into an ordered
-- ROYGBIV spectrum. (Formerly "phosphor", after the CRT-phosphor edge
-- simulator this program grew out of.)
--
-- At each detected edge the output resets to RED; subsequent pixels on the
-- same scanline step through ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET,
-- then black, each band occupying `band_width` pixels (P12 slider; 1080-line
-- pixels, scaled to the mode -- see Scaling).
-- Pipeline resets at hsync so trails never cross lines.
--
-- Edge detect : 3x3 Sobel on preprocessed luma (the Edge Lab detector).
--               Four chained line buffers give a 5-row window; the Sobel
--               rows are raw, or 3x3 Gaussian-blurred (S9 Smooth, per
--               Edge Lab).
--               Magnitude |gx| + |gy| against the knob 2 threshold.
-- Solidify    : the thresholded edge bit is OR'd with the same column of
--               the previous two lines (two 1-bit line buffers), so a line
--               whose magnitude dips just under threshold still fires where
--               its neighbours detected the contour -- no gaps in the sheet.
--               The trigger is then HELD for the whole solid run, pinning
--               the band machine at red: the outline renders as a solid red
--               region as thick as the detected edge, rainbow trailing off
--               its right side.
-- Preprocess  : contrast/blowout around a sliding pivot (K5), then
--               posterize (K4, 128..2 luma levels, Edge Lab mapping), then
--               8-bit reduce.
-- Color path  : 3-bit band index → 8-entry registered YUV palette (7 bands
--               + black), built by a background engine from K6 scheme / K1
--               phase-or-fade / S11 reverse. Output gated to neutral black
--               outside active video (blanking contract).
-- Scaling     : (v1.1.2) band widths are in 1080-line pixels: 720p and SD
--               draw 2/3 and 1/2 as many real pixels per band (1-px
--               floor), so a trail is the same fraction of screen height in
--               every mode. The detector (Sobel, solidify, K3 min-run)
--               stays at pixel scale. 1080 output unchanged.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture prism of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (13 + 3 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_SYNC_DELAY : integer := 13;

    -- 8-entry rainbow palette in 10-bit YUV, BT.601 limited range, with U/V
    -- swapped for Videomancer hardware (SDK convention). The C_PAL_U constant
    -- holds what BT.601 calls Cr, and C_PAL_V holds Cb.
    -- Index 7 is black (past violet).
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(328, 10),  -- Red    (255,   0,   0)
        to_unsigned(584, 10),  -- Orange (255, 128,   0)
        to_unsigned(840, 10),  -- Yellow (255, 255,   0)
        to_unsigned(580, 10),  -- Green  (  0, 255,   0)
        to_unsigned(164, 10),  -- Blue   (  0,   0, 255)
        to_unsigned(192, 10),  -- Indigo ( 75,   0, 130)
        to_unsigned(349, 10),  -- Violet (180,   0, 255)
        to_unsigned( 64, 10)); -- Black (BT.601 black level, not 0)
    constant C_PAL_U : t_pal := (
        to_unsigned(960, 10),  -- Red
        to_unsigned(772, 10),  -- Orange
        to_unsigned(584, 10),  -- Yellow
        to_unsigned(136, 10),  -- Green
        to_unsigned(440, 10),  -- Blue
        to_unsigned(608, 10),  -- Indigo
        to_unsigned(756, 10),  -- Violet
        to_unsigned(512, 10)); -- Black
    constant C_PAL_V : t_pal := (
        to_unsigned(360, 10),  -- Red
        to_unsigned(212, 10),  -- Orange
        to_unsigned( 64, 10),  -- Yellow
        to_unsigned(216, 10),  -- Green
        to_unsigned(960, 10),  -- Blue
        to_unsigned(696, 10),  -- Indigo
        to_unsigned(852, 10),  -- Violet
        to_unsigned(512, 10)); -- Black

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

    -- Parameters
    signal s_knob_width      : unsigned(9 downto 0);
    signal s_knob_noise      : unsigned(9 downto 0);
    signal s_knob_contrast   : unsigned(9 downto 0);
    signal s_poster_mask     : std_logic_vector(9 downto 0) := (others => '1');
    signal s_scheme          : unsigned(2 downto 0);  -- K6: 0 rainbow, 1..7 mono fade
    signal s_sw_video_key    : std_logic;
    signal s_sw_curve_en     : std_logic;
    signal s_sw_smooth       : std_logic := '0';  -- field-latched in p_knobs
    signal s_sw_key_over     : std_logic;  -- S10: keyed video above the rainbow
    signal s_knob_color      : unsigned(9 downto 0);  -- K1: colour phase, animatable
    signal s_sw_reverse      : std_logic;             -- S11: violet-first / fade-up
    signal s_min_run_m1      : unsigned(4 downto 0);  -- min run length - 1 (0..15)
    signal s_contrast_gain   : unsigned(10 downto 0) := to_unsigned(128, 11);  -- 128..1151 (1.0x..~9.0x in 7-bit fixed)
    signal s_pivot           : unsigned(9 downto 0)  := to_unsigned(512, 10);  -- contrast pivot: 512 -> 64 as K5 rises
    signal s_thr             : unsigned(7 downto 0);  -- Sobel magnitude threshold

    -- Valid chain: av(i) = data_in.avid delayed i clocks
    signal av : std_logic_vector(1 to 12) := (others => '0');

    -- S1-S3 contrast/blowout: Y_out = pivot + (Y_in - pivot) * gain / 128,
    -- then posterize + reduce to 8 bit. Split so the 11x12 multiply has a
    -- stage to itself (EdgeLab structure): S1 center, S2 multiply, S3
    -- shift/recenter/clamp/posterize.
    signal a1_c    : signed(10 downto 0) := (others => '0');
    signal a2_prod : signed(22 downto 0) := (others => '0');
    signal a3_y8   : unsigned(7 downto 0) := (others => '0');

    -- Luma line buffers (BRAM): 4 chained lines for the 5-row window that
    -- feeds the 3x3 Gaussian blur (and the raw Sobel rows when smooth is
    -- off). Canonical 1W1R single-process BRAMs; read addr leads write by
    -- one column.
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_lb1 is array (0 to 2047) of std_logic_vector(0 downto 0);
    signal lb1, lb2, lb3, lb4 : t_lb8 := (others => (others => '0'));
    signal q1, q2, q3, q4     : std_logic_vector(7 downto 0) := (others => '0');
    signal lrx, lwx           : unsigned(10 downto 0) := (others => '0');

    -- S4 registered copies of the BRAM read data. The line-buffer Tco plus
    -- routing was the head of the critical path (14.2 ns through the median
    -- comparators); landing the reads in plain FFs first decouples it.
    signal qr1, qr2, qr3, qr4 : unsigned(7 downto 0) := (others => '0');
    signal y8d                : unsigned(7 downto 0) := (others => '0');

    -- S5 vertical [1 2 1] sums for the 3 blur rows (rows n-1, n-2, n-3)
    -- plus 2-column histories for the horizontal blur pass, and 2-column
    -- delayed raw taps so raw mode aligns with the blur center column.
    signal ga, ga1, ga2 : unsigned(9 downto 0) := (others => '0');  -- bottom (n-1)
    signal gb, gb1, gb2 : unsigned(9 downto 0) := (others => '0');  -- mid    (n-2)
    signal gc, gc1, gc2 : unsigned(9 downto 0) := (others => '0');  -- top    (n-3)
    signal q1d, q2d, q3d : unsigned(7 downto 0) := (others => '0');
    signal q1e, q2e, q3e : unsigned(7 downto 0) := (others => '0');

    -- S5 effective Sobel rows (blurred or raw; e0 = top, e2 = bottom)
    signal e0, e1, e2 : unsigned(7 downto 0) := (others => '0');

    -- S6 Sobel column sums + histories
    signal ss, ss1, ss2 : unsigned(9 downto 0) := (others => '0');  -- e0+2e1+e2
    signal dy, dy1, dy2 : signed(8 downto 0) := (others => '0');    -- e0-e2

    -- S7-S9 gradients, magnitude, normalized+guarded magnitude
    signal s_gx, s_gy : signed(11 downto 0) := (others => '0');
    signal s_mag      : unsigned(11 downto 0) := (others => '0');
    signal s_mag8     : unsigned(7 downto 0) := (others => '0');

    -- S10 thresholded edge bit + vertical-solidify bit line buffers
    signal s_eb       : std_logic := '0';
    signal elb1, elb2 : t_lb1 := (others => (others => '0'));
    signal eq1, eq2   : std_logic_vector(0 downto 0) := (others => '0');
    signal ebr, ebw   : unsigned(10 downto 0) := (others => '0');

    -- Field-boundary latch pulse: fires on the hsync edge of lines with no
    -- active video (vertical blanking). Preprocess parameters only update
    -- here, so a whole field is always processed with ONE set of values --
    -- a mid-frame knob change otherwise bakes rows with old and new gain
    -- into the line buffers, and the Sobel reads the uniform step between
    -- them as a full-width horizontal edge (flashing glitch bar).
    signal s_ctl_latch : std_logic := '0';

    -- Blanking guards: left columns (shift/history chains crossing hblank)
    -- and field warm-up lines (window straddling vblank)
    signal cbx     : unsigned(10 downto 0) := (others => '0');
    signal s_aline : unsigned(9 downto 0) := (others => '0');
    signal s_seen  : std_logic := '0';
    signal s_prev_hsync_n_p : std_logic := '1';

    -- S11 solidify + min-run
    signal s_edge_bit  : std_logic := '0';
    signal s_run_count : unsigned(4 downto 0) := (others => '0');

    -- Rainbow band state
    signal s_prev_hsync_n : std_logic := '1';
    signal s_band_pixel : unsigned(6 downto 0) := (others => '0');  -- 0..band_width-1
    signal s_band_idx   : unsigned(2 downto 0) := "111";            -- 7 = black at line start
    -- Colour remap table: band_idx -> palette index, with the K1 cycle
    -- phase and S11 reverse folded in. Registered off the pixel path so
    -- the output mux stays shallow; index 7 always maps to 7 (black/key).
    type t_cmap is array (0 to 7) of unsigned(2 downto 0);
    signal s_cmap      : t_cmap := ("000", "001", "010", "011", "100", "101", "110", "111");
    signal s_base_sel  : unsigned(2 downto 0) := (others => '0');   -- mono-scheme base colour
    signal s_key_video : std_logic;                                 -- pass source video this pixel

    -- Registered display palette (built by p_pal) + engine pipeline regs.
    -- Mono schemes lerp each band from the base colour toward black or
    -- white: weight w(i) = depth * C_FADE(i) / 256, K1 sets depth+target.
    signal s_pal_y : t_pal := C_PAL_Y;
    signal s_pal_u : t_pal := C_PAL_U;
    signal s_pal_v : t_pal := C_PAL_V;
    signal s_pcnt, s_pcnt_d1, s_pcnt_d2, s_pcnt_d3, s_pcnt_d4
        : unsigned(2 downto 0) := (others => '0');
    signal s_pf   : unsigned(8 downto 0) := (others => '0');  -- per-band fade coeff
    signal s_pby, s_pby_d1, s_pby_d2 : unsigned(9 downto 0) := (others => '0');
    signal s_pbu, s_pbu_d1, s_pbu_d2 : signed(10 downto 0) := (others => '0');
    signal s_pbv, s_pbv_d1, s_pbv_d2 : signed(10 downto 0) := (others => '0');
    signal s_pb7_a, s_pb7_b, s_pb7_c : std_logic := '0';  -- band-7 pipe
    signal s_ptw_a, s_ptw_b, s_ptw_c : std_logic := '0';  -- to-white pipe
    signal s_pwp  : unsigned(17 downto 0) := (others => '0');  -- depth * coeff
    signal s_pw         : unsigned(8 downto 0) := (others => '0');  -- weight 0..256
    signal s_ptgt       : unsigned(9 downto 0) := (others => '0');  -- target * w / 256
    signal s_pwc  : unsigned(8 downto 0) := to_unsigned(256, 9);    -- 256 - weight
    signal s_ply  : unsigned(18 downto 0) := (others => '0');       -- y * (256-w)
    signal s_plu, s_plv : signed(20 downto 0) := (others => '0');
    -- K1-derived fade depth (0..256) and direction (registered in p_cmap)
    signal s_pdepth : unsigned(8 downto 0) := (others => '0');
    signal s_ptw    : std_logic := '0';

    -- Fade ramp i/7 in /256 fixed point: band 0 always full base colour
    type t_fade is array (0 to 6) of unsigned(8 downto 0);
    constant C_FADE : t_fade := (
        to_unsigned(  0, 9), to_unsigned( 37, 9), to_unsigned( 73, 9),
        to_unsigned(110, 9), to_unsigned(146, 9), to_unsigned(183, 9),
        to_unsigned(219, 9));

    -- Per-band effective widths MINUS ONE (curve-adjusted terminal counts),
    -- indexed directly by s_band_idx so the value is never stale when the
    -- band advances and the compare needs no subtract.
    type t_sizes is array (0 to 7) of unsigned(7 downto 0);
    signal s_sizes_m1 : t_sizes := (others => to_unsigned(127, 8));

    -- Curve coefficient ROM: width(i) = max(1, W * c(i) / 256), a linear
    -- ramp (i+1)/7 from thin red to full-width violet. Real coefficients
    -- instead of bit-shifts so the growth is smooth.
    type t_crow is array (0 to 6) of unsigned(8 downto 0);
    constant C_CURVE : t_crow := (
        to_unsigned( 37, 9), to_unsigned( 73, 9), to_unsigned(110, 9),
        to_unsigned(146, 9), to_unsigned(183, 9), to_unsigned(219, 9),
        to_unsigned(256, 9));

    -- Curve engine: one 8x9 multiply cycles through the bands, one per
    -- clock (full table refresh every 8 clocks -- instant for a knob)
    -- Band width from P12 with a SQUARED response: 1 + knob^2 / 8192 gives
    -- 1..128 px, putting the fine 1-16 px trails on the first ~35% of the
    -- slider (linear gave 1-8 px only 6%). Must be 8 bits: 127+1 in 7 bits
    -- wraps to 0 and the band machine sticks on red forever.
    signal r_k       : unsigned(9 downto 0) := (others => '0');
    signal r_ksq     : unsigned(19 downto 0) := (others => '0');
    signal r_bw      : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_ccnt    : unsigned(2 downto 0) := (others => '0');
    signal s_ccnt_d1 : unsigned(2 downto 0) := (others => '0');
    signal s_ccnt_d2 : unsigned(2 downto 0) := (others => '0');
    signal s_coef    : unsigned(8 downto 0) := (others => '0');
    signal s_cprod   : unsigned(16 downto 0) := (others => '0');
    signal s_cprod_r : unsigned(16 downto 0) := (others => '0');
    signal s_ccnt_d3 : unsigned(2 downto 0) := (others => '0');

    -- Resolution scaling (v1.1.2): P12 sets the band width in 1080-line
    -- pixels; 720p and SD draw it at 2/3 and 1/2 as many real pixels
    -- (x step 1.5 / 2.0 virtual units per pixel), so a trail is the same
    -- fraction of screen height in every mode. The scale is folded into
    -- the curve engine's coefficient (plain width = coefficient 256 at
    -- 1080, 171 at 720p, 128 at SD) and rounded outside 1080; the 1-px
    -- floor is the engine's existing clamp. 1080 is unchanged (coefficient
    -- 256, no rounding term). Class from the timing ID, free-running:
    -- 0 = 1080 (p and i), 1 = 720p, 2 = SD.
    signal s_xcls    : unsigned(1 downto 0) := (others => '0');
    type t_cscale is array (0 to 7) of unsigned(8 downto 0);
    -- (i+1)/7 * 256 * 2/3 and * 1/2, rounded; index 7 = plain width
    constant C_CURVE720 : t_cscale := (
        to_unsigned( 24, 9), to_unsigned( 49, 9), to_unsigned( 73, 9),
        to_unsigned( 98, 9), to_unsigned(122, 9), to_unsigned(146, 9),
        to_unsigned(171, 9), to_unsigned(171, 9));
    constant C_CURVESD : t_cscale := (
        to_unsigned( 18, 9), to_unsigned( 37, 9), to_unsigned( 55, 9),
        to_unsigned( 73, 9), to_unsigned( 91, 9), to_unsigned(110, 9),
        to_unsigned(128, 9), to_unsigned(128, 9));

    -- Delayed incoming Y/U/V for video key-through (needs full SYNC_DELAY alignment)
    type t_yuv_delay is array (0 to C_SYNC_DELAY - 1) of std_logic_vector(9 downto 0);
    signal s_y_in_sr : t_yuv_delay := (others => (others => '0'));
    signal s_u_in_sr : t_yuv_delay := (others => (others => '0'));
    signal s_v_in_sr : t_yuv_delay := (others => (others => '0'));

    -- Luma-key matte: PREPROCESSED luma >= 50% (a3_y8 MSB), delayed from
    -- pipeline index 3 to output alignment. Where the matte is high the
    -- source rides on top of the rainbow -- and because the matte comes
    -- from the post-blowout signal, K5 directly hardens the key cutout.
    -- Stateless per-pixel decision: none of the parity problems of the
    -- earlier within-edges experiments.
    signal s_matte_sr : std_logic_vector(C_SYNC_DELAY - 4 downto 0) := (others => '0');

    -- Sync shift register
    signal s_avid_sr    : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');

begin

    -- Band W lives on the P12 slider (registers_in(7)). Rotary 1 is the
    -- rainbow colour-cycle phase: turning it rotates the colours through
    -- the bands, animating the trail.
    s_knob_width    <= unsigned(registers_in(7));
    s_knob_color    <= unsigned(registers_in(0));
    s_knob_noise    <= unsigned(registers_in(2));
    s_knob_contrast <= unsigned(registers_in(4));
    -- Toggle switches 7..11 arrive as bits 0..4 of registers_in(6)
    -- (s_sw_smooth is latched at field boundaries in p_knobs -- it feeds
    -- the detector window like the preprocess knobs do)
    s_sw_video_key  <= registers_in(6)(0);
    s_sw_key_over   <= registers_in(6)(1);  -- S8: Key Pos, next to S7's key
    s_sw_curve_en   <= registers_in(6)(3);  -- S10: Curve En
    s_sw_reverse    <= registers_in(6)(4);

    -- Band width (1..128 px, squared slider response) is computed in
    -- p_curve.
    -- Min run length: knob top 4 bits + 1 gives 1..16 pixels; stored minus 1
    -- so the run-count compare needs no subtract
    s_min_run_m1 <= resize(unsigned(s_knob_noise(9 downto 6)), 5);
    -- Sobel threshold: knob top 8 bits cover the full 0..255 magnitude range
    s_thr <= unsigned(registers_in(1)(9 downto 2));
    -- Contrast gain: 7-bit fixed-point (128 = 1.0x). 128 + knob = 128..1151,
    -- i.e. 1.0x..~9.0x. The pivot slides from mid-grey (512) down to BT.601
    -- black level (64) as the knob rises -- knob * 7/16 subtracted -- so the
    -- low range is classic centered contrast while the top blows EVERYTHING
    -- above ~12% luma out to white and crushes the black-level noise floor
    -- to black: a video-level-aware binarizer (the "3x value multiplier to
    -- all white" technique, minus the amplified dark noise).
    -- REGISTERED: both values are knob-derived and quasi-static; computing
    -- them combinationally ahead of the S1 subtract+multiply cost ~30 MHz.
    -- Colour scheme: knob top 3 bits -- zone 0 = rainbow, zones 1..7 = all
    -- bands in that single colour (red..violet), fading along the trail
    s_scheme <= unsigned(registers_in(5)(9 downto 7));
    -- (posterize mask is latched at field boundaries in p_knobs)

    ----------------------------------------------------------------------------
    -- Edge pipeline (all luma), Edge Lab Sobel with optional Gaussian smooth
    -- and a vertical edge-bit solidify:
    --   S1  contrast stage A: (Y - 512) * gain
    --   S2  contrast stage B: shift back, recenter, clamp
    --   S3  posterize, reduce to 8 bit; present line-buffer read address
    --   S4  register BRAM reads into FFs; line-buffer rotate writes
    --   S5  vertical [1 2 1] sums for rows n-1/n-2/n-3 + column histories;
    --       delayed raw taps
    --   S6  horizontal [1 2 1], /16 -> blurred rows; smooth mux -> e-rows
    --   S7  Sobel column sums ss / dy + two-column histories
    --   S8  gx = ss - ss2 (horizontal), gy = dy + 2*dy1 + dy2 (vertical)
    --   S9  magnitude |gx| + |gy|
    --   S10 normalize to 0..255, blank warm-up lines / left columns
    --   S11 threshold -> edge bit; present bit-buffer read address
    --   S12 solidify (OR with previous 2 lines) + min-run → trigger
    ----------------------------------------------------------------------------
    p_knobs : process(clk)
    begin
        if rising_edge(clk) then
            -- Detector-window parameters only change at field boundaries
            -- (s_ctl_latch) so a field is never processed with mixed
            -- settings -- mid-frame changes painted a full-width fake
            -- horizontal edge near the top of the screen.
            if s_ctl_latch = '1' then
                s_contrast_gain <= resize(s_knob_contrast, 11) + to_unsigned(128, 11);
                s_pivot <= to_unsigned(512, 10)
                           - (shift_right(s_knob_contrast, 1)
                              - shift_right(s_knob_contrast, 4));
                case to_integer(unsigned(registers_in(3)(9 downto 7))) is
                    when 0      => s_poster_mask <= "1111111111";
                    when 1      => s_poster_mask <= "1111111000";
                    when 2      => s_poster_mask <= "1111110000";
                    when 3      => s_poster_mask <= "1111100000";
                    when 4      => s_poster_mask <= "1111000000";
                    when 5      => s_poster_mask <= "1110000000";
                    when 6      => s_poster_mask <= "1100000000";
                    when others => s_poster_mask <= "1000000000";
                end case;
                s_sw_smooth <= registers_in(6)(2);
            end if;
        end if;
    end process p_knobs;

    p_pix : process(clk)
        variable v_sum   : signed(14 downto 0);
        variable v_p10   : unsigned(9 downto 0);
        variable v_m8    : unsigned(7 downto 0);
        variable v_solid : std_logic;
    begin
        if rising_edge(clk) then
            s_prev_hsync_n_p <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 12 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S1: difference from the (knob-tracking, registered) pivot
            if data_in.avid = '1' then
                a1_c <= signed(resize(unsigned(data_in.y), 11))
                        - signed(resize(s_pivot, 11));
            end if;

            -- S2: multiply by gain (the 11x12 multiply gets this stage to
            -- itself -- with any logic ahead of it the HD configs miss)
            if av(1) = '1' then
                a2_prod <= a1_c * signed('0' & std_logic_vector(s_contrast_gain));
            end if;

            -- S3: shift back, recenter on the pivot, clamp, posterize,
            -- reduce to 8 bit; luma reads presented this cycle
            if av(2) = '1' then
                v_sum := resize(shift_right(a2_prod, 7), 15)
                         + signed(resize(s_pivot, 15));
                if v_sum < 0 then
                    v_p10 := (others => '0');
                elsif v_sum > 1023 then
                    v_p10 := (others => '1');
                else
                    v_p10 := unsigned(std_logic_vector(v_sum(9 downto 0)));
                end if;
                v_p10 := v_p10 and unsigned(s_poster_mask);
                a3_y8 <= v_p10(9 downto 2);
                lrx   <= lrx + 1;
            end if;

            -- S4: land the BRAM reads in plain FFs (window rows: current =
            -- a3_y8 -> y8d, q1 = 1 line up ... q4 = 4 up); buffer rotate
            -- writes happen in the BRAM processes
            if av(3) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                qr3 <= unsigned(q3);
                qr4 <= unsigned(q4);
                y8d <= a3_y8;
                lwx <= lwx + 1;
            end if;

            -- S5: vertical [1 2 1] sums for the 3 blur rows, two-column
            -- histories for the horizontal pass, delayed raw taps
            if av(4) = '1' then
                ga <= resize(y8d, 10)
                      + shift_left(resize(qr1, 10), 1)
                      + resize(qr2, 10);
                gb <= resize(qr1, 10)
                      + shift_left(resize(qr2, 10), 1)
                      + resize(qr3, 10);
                gc <= resize(qr2, 10)
                      + shift_left(resize(qr3, 10), 1)
                      + resize(qr4, 10);
                ga1 <= ga;  ga2 <= ga1;
                gb1 <= gb;  gb2 <= gb1;
                gc1 <= gc;  gc2 <= gc1;
                q1d <= qr1;  q1e <= q1d;
                q2d <= qr2;  q2e <= q2d;
                q3d <= qr3;  q3e <= q3d;
            end if;

            -- S6: horizontal [1 2 1] over the vertical sums, /16 -> blurred
            -- rows; smooth mux picks blurred (S9) or raw rows -- both
            -- centered on the same column, so latency is identical.
            if av(5) = '1' then
                if s_sw_smooth = '1' then
                    e2 <= resize(shift_right(resize(ga2, 12)
                              + shift_left(resize(ga1, 12), 1)
                              + resize(ga, 12), 4), 8);
                    e1 <= resize(shift_right(resize(gb2, 12)
                              + shift_left(resize(gb1, 12), 1)
                              + resize(gb, 12), 4), 8);
                    e0 <= resize(shift_right(resize(gc2, 12)
                              + shift_left(resize(gc1, 12), 1)
                              + resize(gc, 12), 4), 8);
                else
                    e2 <= q1e;
                    e1 <= q2e;
                    e0 <= q3e;
                end if;
            end if;

            -- S7: Sobel column sums (e0 = top row, e2 = bottom) and
            -- two-column histories
            if av(6) = '1' then
                ss <= resize(e0, 10) + shift_left(resize(e1, 10), 1)
                      + resize(e2, 10);
                dy <= signed(resize(e0, 9)) - signed(resize(e2, 9));
                ss1 <= ss;  ss2 <= ss1;
                dy1 <= dy;  dy2 <= dy1;
            end if;

            -- S8: gradients
            if av(7) = '1' then
                s_gx <= signed(resize(ss, 12)) - signed(resize(ss2, 12));
                s_gy <= resize(dy, 12) + shift_left(resize(dy1, 12), 1)
                        + resize(dy2, 12);
            end if;

            -- S9: magnitude
            if av(8) = '1' then
                s_mag <= resize(f_absu(s_gx), 12) + resize(f_absu(s_gy), 12);
            end if;

            -- S10: normalize to 0..255; blank warm-up lines (window
            -- straddles the previous field) and left columns (histories
            -- carry the previous line's tail)
            if av(9) = '1' then
                v_m8 := f_c255(shift_right(s_mag, 3));
                if s_aline < 5 or cbx < 10 then
                    v_m8 := (others => '0');
                end if;
                s_mag8 <= v_m8;
                cbx <= cbx + 1;
            end if;

            -- S11: threshold; bit-buffer reads presented this cycle
            if av(10) = '1' then
                if s_mag8 > s_thr then
                    s_eb <= '1';
                else
                    s_eb <= '0';
                end if;
                ebr <= ebr + 1;
            end if;

            -- S12: vertical solidify -- OR with the same column of the
            -- previous two lines -- then min-run. Once a run qualifies
            -- (s_min_run consecutive solid pixels) the edge bit is HELD
            -- high for the rest of the run, pinning the band machine at
            -- red: the outline is drawn solid, exactly as thick as the
            -- detected edge region, and the rainbow starts at its trailing
            -- side. The aline guard keeps the previous field's buffered
            -- bits from triggering on the first lines.
            if av(11) = '1' then
                v_solid := s_eb or eq1(0) or eq2(0);
                if s_aline < 5 then
                    v_solid := '0';
                end if;
                if v_solid = '1' then
                    if s_run_count >= s_min_run_m1 then
                        s_edge_bit <= '1';
                    else
                        s_edge_bit <= '0';
                    end if;
                    if s_run_count < 31 then
                        s_run_count <= s_run_count + 1;
                    end if;
                else
                    s_run_count <= (others => '0');
                    s_edge_bit <= '0';
                end if;
                ebw <= ebw + 1;
            else
                s_edge_bit <= '0';
            end if;

            -- New line: reset counters; count active lines per field.
            -- A line period with no active video is vertical blanking, so
            -- the active-line counter resets THERE -- not on the vsync
            -- edge, whose position relative to the last active lines varies
            -- by source/decoder. (An early vsync parked the warm-up guard's
            -- 5-line black band inside the picture near the bottom on
            -- hardware.) Saturate rather than wrap so an unexpectedly long
            -- field can never re-trigger the guard mid-picture.
            if data_in.hsync_n = '0' and s_prev_hsync_n_p = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                ebr <= (others => '0');
                ebw <= (others => '0');
                cbx <= (others => '0');
                s_run_count <= (others => '0');
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                else
                    s_aline <= (others => '0');
                end if;
            end if;

            -- Field-boundary latch pulse: hsync edge of a blank line
            if data_in.hsync_n = '0' and s_prev_hsync_n_p = '1'
               and s_seen = '0' then
                s_ctl_latch <= '1';
            else
                s_ctl_latch <= '0';
            end if;
        end if;
    end process p_pix;

    ----------------------------------------------------------------------------
    -- Luma line buffers: read at lrx (one column ahead of the write), rotate
    -- rows by rewriting each read value one buffer down.
    ----------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(3) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(a3_y8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(3) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    p_lb3 : process(clk)
    begin
        if rising_edge(clk) then
            q3 <= lb3(to_integer(lrx));
            if av(3) = '1' then
                lb3(to_integer(lwx)) <= q2;
            end if;
        end if;
    end process p_lb3;

    p_lb4 : process(clk)
    begin
        if rising_edge(clk) then
            q4 <= lb4(to_integer(lrx));
            if av(3) = '1' then
                lb4(to_integer(lwx)) <= q3;
            end if;
        end if;
    end process p_lb4;

    ----------------------------------------------------------------------------
    -- Solidify bit line buffers: elb1 holds the previous line's edge bits,
    -- elb2 the line before that. Same read-leads-write chaining as the luma
    -- buffers.
    ----------------------------------------------------------------------------
    p_elb1 : process(clk)
    begin
        if rising_edge(clk) then
            eq1 <= elb1(to_integer(ebr));
            if av(11) = '1' then
                elb1(to_integer(ebw)) <= (0 => s_eb);
            end if;
        end if;
    end process p_elb1;

    p_elb2 : process(clk)
    begin
        if rising_edge(clk) then
            eq2 <= elb2(to_integer(ebr));
            if av(11) = '1' then
                elb2(to_integer(ebw)) <= eq1;
            end if;
        end if;
    end process p_elb2;

    ----------------------------------------------------------------------------
    -- Curve: per-band effective widths. With curve enabled (S10) the
    -- widths grow linearly toward violet:
    --   width(i) = max(1, band_width * C_CURVE(i) / 256)
    -- so red is the thinnest band and violet gets the full width. One 8x9
    -- multiply is time-multiplexed across the bands (one per clock; band 7
    -- and curve-off get the plain width) -- the table refreshes every 8
    -- clocks, instant for a knob, and p_band still indexes it by band_idx
    -- with no stale-size lag.
    ----------------------------------------------------------------------------
    p_curve : process(clk)
        variable v_w : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- resolution class from the timing ID (static while a mode runs)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_xcls <= "10";  -- SD: x step 2.0
                when 5 | 6 | 14     => s_xcls <= "01";  -- 720p: x step 1.5
                when others         => s_xcls <= "00";  -- 1080p/i: 1.0
            end case;

            -- band width: register knob, square alone, scale + 1
            r_k   <= s_knob_width;
            r_ksq <= r_k * r_k;
            r_bw  <= resize(r_ksq(19 downto 13), 8) + to_unsigned(1, 8);

            -- stage 0: fetch the coefficient (ROM mux gets its own register
            -- -- sharing a stage with the multiply cost ~20 MHz). Curve off
            -- and band 7 take the plain width (coefficient = mode scale).
            if s_sw_curve_en = '0' or s_ccnt = 7 then
                case s_xcls is
                    when "01"   => s_coef <= C_CURVE720(7);
                    when "10"   => s_coef <= C_CURVESD(7);
                    when others => s_coef <= to_unsigned(256, 9);
                end case;
            else
                case s_xcls is
                    when "01"   => s_coef <= C_CURVE720(to_integer(s_ccnt));
                    when "10"   => s_coef <= C_CURVESD(to_integer(s_ccnt));
                    when others => s_coef <= C_CURVE(to_integer(s_ccnt));
                end case;
            end if;
            s_ccnt_d1 <= s_ccnt;
            s_ccnt    <= s_ccnt + 1;

            -- stage 1: the multiply, alone in its stage
            s_cprod   <= r_bw * s_coef;
            s_ccnt_d2 <= s_ccnt_d1;

            -- stage 2: round to nearest outside 1080 (1080 truncates, as
            -- before)
            if s_xcls = "00" then
                s_cprod_r <= s_cprod;
            else
                s_cprod_r <= s_cprod + to_unsigned(128, 17);
            end if;
            s_ccnt_d3 <= s_ccnt_d2;

            -- stage 3: normalize, clamp to >= 1, store minus one
            v_w := resize(shift_right(s_cprod_r, 8), 9);
            if v_w = 0 then
                v_w := to_unsigned(1, 9);
            end if;
            s_sizes_m1(to_integer(s_ccnt_d3)) <= resize(v_w - 1, 8);
        end if;
    end process p_curve;

    ----------------------------------------------------------------------------
    -- Rainbow band state machine:
    --   edge        → reset to red (band 0, pixel 0)
    --   within band → increment band_pixel
    --   end of band → advance band_idx (saturates at 7 = black)
    --   hsync edge  → back to black (idx 7)
    --
    -- With curve enabled (S10), band widths follow the linear curve in
    -- s_sizes_m1: thin red first, each colour wider toward violet.
    -- The >= compare (not =) means a band can never overshoot its terminal
    -- count and run away to the 7-bit counter wrap.
    ----------------------------------------------------------------------------
    p_band : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_band_idx   <= "111";
                s_band_pixel <= (others => '0');
            elsif av(12) = '1' then
                if s_edge_bit = '1' then
                    s_band_idx   <= (others => '0');
                    s_band_pixel <= (others => '0');
                elsif resize(s_band_pixel, 8) >= s_sizes_m1(to_integer(s_band_idx)) then
                    s_band_pixel <= (others => '0');
                    if s_band_idx /= "111" then
                        s_band_idx <= s_band_idx + 1;
                    end if;
                else
                    s_band_pixel <= s_band_pixel + 1;
                end if;
            end if;
        end if;
    end process p_band;

    ----------------------------------------------------------------------------
    -- Video-key delay: incoming Y/U/V delayed by full SYNC_DELAY so it lines
    -- up with data_out. Used when switch 7 is on and the band index falls
    -- past violet — the "black" region is replaced with pass-through video.
    ----------------------------------------------------------------------------
    p_video_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_in_sr(0) <= data_in.y;
            s_u_in_sr(0) <= data_in.u;
            s_v_in_sr(0) <= data_in.v;
            for i in 1 to C_SYNC_DELAY - 1 loop
                s_y_in_sr(i) <= s_y_in_sr(i - 1);
                s_u_in_sr(i) <= s_u_in_sr(i - 1);
                s_v_in_sr(i) <= s_v_in_sr(i - 1);
            end loop;
            s_matte_sr(0) <= a3_y8(7);
            for i in 1 to C_SYNC_DELAY - 4 loop
                s_matte_sr(i) <= s_matte_sr(i - 1);
            end loop;
        end if;
    end process p_video_delay;

    ----------------------------------------------------------------------------
    -- Sync delay: 13 stages — contrast (2) + posterize (1) + BRAM-read
    -- register (1) + column sums (1) + blur/mux (1) + Sobel sums (1)
    -- + gradients (1) + magnitude (1) + normalize (1) + threshold (1) +
    -- solidify/min-run (1) + band (1); palette lookup is combinational on
    -- band_idx.
    ----------------------------------------------------------------------------
    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (C_SYNC_DELAY - 2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.field_n;
        end if;
    end process p_sync_delay;

    ----------------------------------------------------------------------------
    -- Colour remap table: fold the K1 cycle phase (0..6, knob*7/1024 so a
    -- full turn is one full rotation with no dead zone) and the S11 reverse
    -- into a registered band->palette map. Turning K1 rotates the colours
    -- through the bands, animating the trail; geometry (band widths, curve,
    -- video key) is untouched.
    ----------------------------------------------------------------------------
    p_cmap : process(clk)
        variable v_k7 : unsigned(12 downto 0);
        variable v_s  : integer range 0 to 13;
        variable v_b  : integer range -1 to 13;
    begin
        if rising_edge(clk) then
            v_k7 := shift_left(resize(s_knob_color, 13), 3)
                    - resize(s_knob_color, 13);              -- knob * 7
            for i in 0 to 6 loop
                if s_sw_reverse = '1' then
                    v_s := (6 - i) + to_integer(v_k7(12 downto 10));
                else
                    v_s := i + to_integer(v_k7(12 downto 10));
                end if;
                if v_s > 6 then
                    v_s := v_s - 7;
                end if;
                s_cmap(i) <= to_unsigned(v_s, 3);
            end loop;
            s_cmap(7) <= "111";  -- black/key region never remaps

            -- Mono-scheme base colour comes straight from K6 (no K1 phase
            -- -- in mono schemes K1 is the fade control instead)
            if s_scheme /= 0 then
                v_b := to_integer(s_scheme) - 1;
                s_base_sel <= to_unsigned(v_b, 3);
            else
                s_base_sel <= (others => '0');
            end if;

            -- Mono fade from K1: centre = flat solid colour, turning down
            -- fades the trail to black, turning up bleaches it to white
            if s_knob_color >= 512 then
                s_ptw    <= '1';
                s_pdepth <= resize(shift_right(s_knob_color - to_unsigned(512, 10), 1), 9);
            else
                s_ptw    <= '0';
                s_pdepth <= resize(shift_right(to_unsigned(512, 10) - s_knob_color, 1), 9);
            end if;
        end if;
    end process p_cmap;

    ----------------------------------------------------------------------------
    -- Palette engine: builds the registered display palette one band per
    -- clock (5-stage: fetch / depth-multiply / weight / lerp-multiply /
    -- store; table refresh every 8 clocks). Scheme 0 (Rainbow) reads the
    -- classic palette through s_cmap unfaded. Schemes 1..7 render every
    -- band in the K6 base colour, lerped along the trail toward black or
    -- white by K1: weight w(i) = depth * C_FADE(i)/256, chroma relaxes to
    -- neutral, Y heads to 0 or full white. S11 reverse flips the fade
    -- direction. Band 7 is forced w = 256 toward black in every scheme,
    -- preserving the video-key region. Each multiply is alone in its stage.
    ----------------------------------------------------------------------------
    p_pal : process(clk)
        variable v_sel : unsigned(2 downto 0);
        variable v_f   : integer range 0 to 6;
        variable v_w   : unsigned(8 downto 0);
        variable v_y   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage A: fetch base colour, fade coefficient, flags
            if s_scheme = 0 then
                v_sel := s_cmap(to_integer(s_pcnt));
            else
                v_sel := s_base_sel;
            end if;
            s_pby <= C_PAL_Y(to_integer(v_sel));
            s_pbu <= signed(resize(C_PAL_U(to_integer(v_sel)), 11))
                     - to_signed(512, 11);
            s_pbv <= signed(resize(C_PAL_V(to_integer(v_sel)), 11))
                     - to_signed(512, 11);
            if s_pcnt = 7 or s_scheme = 0 then
                s_pf <= (others => '0');      -- rainbow / band 7: no ramp
            else
                if s_sw_reverse = '1' then
                    v_f := 6 - to_integer(s_pcnt);
                else
                    v_f := to_integer(s_pcnt);
                end if;
                s_pf <= C_FADE(v_f);
            end if;
            if s_pcnt = 7 then
                s_pb7_a <= '1';
            else
                s_pb7_a <= '0';
            end if;
            s_ptw_a   <= s_ptw;
            s_pcnt_d1 <= s_pcnt;
            s_pcnt    <= s_pcnt + 1;

            -- stage B: depth x coefficient multiply, alone
            s_pwp    <= s_pdepth * s_pf;
            s_pby_d1 <= s_pby;  s_pbu_d1 <= s_pbu;  s_pbv_d1 <= s_pbv;
            s_pb7_b  <= s_pb7_a;  s_ptw_b <= s_ptw_a;
            s_pcnt_d2 <= s_pcnt_d1;

            -- stage C: lerp weight and its complement (band 7 = full black)
            if s_pb7_b = '1' then
                v_w := to_unsigned(256, 9);
            else
                v_w := resize(shift_right(s_pwp, 8), 9);
            end if;
            s_pw     <= v_w;
            s_pwc    <= to_unsigned(256, 9) - v_w;
            s_pby_d2 <= s_pby_d1;  s_pbu_d2 <= s_pbu_d1;  s_pbv_d2 <= s_pbv_d1;
            s_pb7_c  <= s_pb7_b;  s_ptw_c <= s_ptw_b;
            s_pcnt_d3 <= s_pcnt_d2;

            -- stage D: lerp multiplies, alone
            s_ply <= s_pby_d2 * s_pwc;
            s_plu <= s_pbu_d2 * signed('0' & std_logic_vector(s_pwc));
            s_plv <= s_pbv_d2 * signed('0' & std_logic_vector(s_pwc));
            -- lerp target term target*w/256: white 960 (3.75w), black 64
            -- (w/4) -- limited-range levels, never super-black/white
            if s_ptw_c = '1' and s_pb7_c = '0' then
                s_ptgt <= resize(shift_left(resize(s_pw, 11), 2)
                                 - resize(shift_right(s_pw, 2), 11), 10);
            else
                s_ptgt <= resize(shift_right(s_pw, 2), 10);
            end if;
            s_pcnt_d4 <= s_pcnt_d3;

            -- stage E: recombine with the target term, clamp, recenter
            -- chroma, store
            v_y := resize(shift_right(s_ply, 8), 12) + resize(s_ptgt, 12);
            if v_y > 1023 then
                v_y := to_unsigned(1023, 12);
            end if;
            s_pal_y(to_integer(s_pcnt_d4)) <= v_y(9 downto 0);
            s_pal_u(to_integer(s_pcnt_d4)) <= unsigned(resize(
                shift_right(s_plu, 8) + to_signed(512, 21), 10));
            s_pal_v(to_integer(s_pcnt_d4)) <= unsigned(resize(
                shift_right(s_plv, 8) + to_signed(512, 21), 10));
        end if;
    end process p_pal;

    -- Video key: source video fills the past-violet black region. With
    -- Key Pos = Over (S8), it ALSO rides on top of the colour bands
    -- wherever the luma-key matte is high (preprocessed luma >= 50%) --
    -- trails stream out from behind the keyed image instead of covering
    -- it. K5 blowout hardens the matte.
    s_key_video <= '1' when s_sw_video_key = '1'
                        and (s_band_idx = "111"
                             or (s_sw_key_over = '1'
                                 and s_matte_sr(C_SYNC_DELAY - 4) = '1'))
                   else '0';

    -- Blanking gate: neutral black (Y 64, U=V 512) whenever the aligned
    -- avid is low. Without it the last band colour (or keyed video) runs
    -- through hblank and corrupts the encoder's colour reference.
    s_io_align_in.y <= std_logic_vector(to_unsigned(64, 10))
                      when s_avid_sr(C_SYNC_DELAY - 1) = '0'
                  else s_y_in_sr(C_SYNC_DELAY - 1)
                      when s_key_video = '1'
                  else std_logic_vector(s_pal_y(to_integer(s_band_idx)));
    s_io_align_in.u <= std_logic_vector(to_unsigned(512, 10))
                      when s_avid_sr(C_SYNC_DELAY - 1) = '0'
                  else s_u_in_sr(C_SYNC_DELAY - 1)
                      when s_key_video = '1'
                  else std_logic_vector(s_pal_u(to_integer(s_band_idx)));
    s_io_align_in.v <= std_logic_vector(to_unsigned(512, 10))
                      when s_avid_sr(C_SYNC_DELAY - 1) = '0'
                  else s_v_in_sr(C_SYNC_DELAY - 1)
                      when s_key_video = '1'
                  else std_logic_vector(s_pal_v(to_integer(s_band_idx)));
    s_io_align_in.avid    <= s_avid_sr   (C_SYNC_DELAY - 1);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_SYNC_DELAY - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_SYNC_DELAY - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_SYNC_DELAY - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (13 -> 16).
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

end prism;
