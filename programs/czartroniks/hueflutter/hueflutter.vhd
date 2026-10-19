-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Hueflutter: catalogue combo #111 (seed 20260925) -- hue rotation /
-- saturation + magnetic warp (wobble) + ordered dithering.  Two knobs each;
-- the switches and the slider decide how they combine.
--
--   K1 Wobble   displacement amplitude (0..255 px at 1080p; scaled per mode)
--   K2 Waves    vertical wavelength (phase step per line; glided)
--   K3 Hue      base hue rotation 0..360 degrees (0 = unrotated)
--   K4 Sat      saturation 0..400% (100% = unity)
--   K5 Luma     luma levels 1..8 bits
--   K6 Chroma   chroma levels 1..8 bits
--
--   S7  Bands   the 8 luma bands differ in phase (band k is k/16 of a
--               cycle ahead) / in amplitude (band k swings (k+1)/8 of K1)
--   S8  Order   Dither In: chroma posterized before the hue rotation (the
--               colour lattice spins), dither pattern travels with the
--               picture / Dither Out: posterized after it (a fixed palette),
--               dither pattern fixed to the screen
--   S9  Chroma  chroma wobbles with luma / a quarter wave apart
--   S10 Hue     static / hue follows the wobble phase (rainbow ripples)
--   S11 Quant   plain quantizer / 2x2 dithered quantizer
--   P12 Spin    hue rotation speed, bipolar, squared (centre = still)
--
-- Posterize: luma is normalized to the video range ((y-64)*1.168), cut to
-- L bits, bit-replicated and scaled back (*0.855 + 64), so black 64 and
-- white ~940 are always levels.  Chroma is a centred mid-tread quantizer
-- (levels at 512 +- k*step, clamped 64..960), so greys stay grey at every
-- depth.  Dither is centred (no brightness shift).  Dither cells are 2 px
-- wide so the pattern survives the output [1 2 1].
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (29) in all modes; read
-- and write banks latched per line at the avid rise; U/V swapped on rows
-- whose hsync->avid parity differs from the previous row's (each row
-- replays the row above -- drape v1.3); output conditioning (highlight knee
-- 600 + (y-600)/2, white ~770, then [1 2 1] on Y/U/V: the HD HDMI bar fix
-- -- low-bit posterized edges are near-full-swing and move every frame);
-- no_rw_check on every RAM (a bank is only ever read while the other one is
-- written; the write bank's read data is discarded).
--
-- v0.2.0: Hue 0 was a -90 degree rotation (the ROM holds -cos; read a
-- quarter turn early); the chroma posterizer had no neutral level (greys
-- went pink: U = V = 576 at the default 3 bits); the luma posterizer lifted
-- black (1 bit = two greys); dither was 0..3/4 step (brightened the
-- picture); the output dither cell came from the input-side counters (it
-- froze over the last ~22 px of each line); Amp bands used a third of K1;
-- Spin squared with a deadband; Waves 10-bit + glided; depth hysteresis.
--
-- v0.2.3 (2026-10-17): resolution scaling.  The wobble lives on a virtual
-- 1080-line canvas, stepped per mode from the video timing ID
-- (registers_in(8)): K1's amplitude is 1080-line pixels (x 2/3 at 720p,
-- x 1/2 at SD, where 720 px span 1440 units) and the Waves phase step per
-- field line is scaled by the virtual line step (1.5 at 720p, 1080/486 x 2
-- per 480i field line; a serial multiplier, once per frame), so the wobble
-- has the same size and the same number of waves relative to the screen
-- height in every mode.  The 2 px dither cells stay at pixel scale.  1080p
-- is unchanged (every step is exactly 1.0).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture hueflutter of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (29 + 3 = 32 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 32;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 29;

    -- the ROM holds -cos: C(a) = -cos(a), so sin(a) = C(a + 64) and
    -- cos(a) = C(a + 128)
    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    signal s_k_amp, s_k_waves, s_k_hue, s_k_sat, s_k_lbits, s_k_cbits : unsigned(9 downto 0);
    signal s_sw_bandamp, s_sw_wobfirst, s_sw_cquarter, s_sw_huewob, s_sw_dither : std_logic;
    signal s_p_spin : unsigned(9 downto 0);

    -- Resolution scaling: virtual 1080-line steps from the timing ID.  x step
    -- in half units per pixel (1080: 2, 720p: 3, SD: 4), y step Q4.8 per
    -- FIELD line (1080 / active lines, doubled for interlace).
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
    signal s_vxs : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_vys : unsigned(11 downto 0) := to_unsigned(256, 12);
    -- K1 in 1080-line px -> real px: x 2/3 at 720p ((a*171 + 128) >> 8), x 1/2 at SD
    signal s_amp_k  : unsigned(7 downto 0) := (others => '0');
    signal s_a168, s_a3r : unsigned(15 downto 0) := (others => '0');
    -- Waves line step = (glided Waves * y step) >> 10, on a free-running
    -- 16-clock serial multiplier (both operands change only at vsync)
    signal s_sm_cnt : unsigned(3 downto 0) := (others => '0');
    signal s_sm_a   : unsigned(13 downto 0) := (others => '0');
    signal s_sm_b   : unsigned(11 downto 0) := (others => '0');
    signal s_sm_p   : unsigned(25 downto 0) := (others => '0');
    signal s_wstep  : unsigned(15 downto 0) := (others => '0');

    -- per-frame controls
    signal s_amp : unsigned(7 downto 0) := (others => '0');
    signal s_hue, s_hp : unsigned(7 downto 0) := (others => '0');
    signal s_gain, s_gain_a : unsigned(8 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_gain_a : signal is true;
    signal s_gw : unsigned(13 downto 0) := (others => '0');            -- glided Waves (10.4)
    -- spin: squared speed curve, free-running (only the vsync add reads it)
    signal s_spin_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_spin_m   : unsigned(8 downto 0) := (others => '0');
    signal s_spin_sqh : unsigned(12 downto 0) := (others => '0');
    signal s_spin_sql : unsigned(13 downto 0) := (others => '0');
    signal s_spin_mag : unsigned(10 downto 0) := (others => '0');
    signal s_spin_n1, s_spin_n2, s_spin_n3 : std_logic := '0';
    signal s_spin_inc : signed(15 downto 0) := (others => '0');
    -- bit depths: zone = bits - 1, hysteretic across the zone boundaries
    signal s_zl, s_zc : unsigned(2 downto 0) := to_unsigned(2, 3);
    -- frames since load; until it reaches 15 the K5/K6 zones follow the knob
    -- directly (no hysteresis), so a saved default near a zone edge loads in
    -- its own zone instead of staying at the power-up zone
    signal s_zwarm : unsigned(3 downto 0) := (others => '0');
    signal s_lstep, s_cstep : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_cmask : unsigned(9 downto 0) := (others => '1');
    signal s_ld0, s_ld1, s_ld2, s_ld3 : signed(10 downto 0) := (others => '0');    -- luma dither, (2b-3)/8 step
    signal s_cd0, s_cd1, s_cd2, s_cd3 : unsigned(9 downto 0) := (others => '0');  -- chroma round + dither, (2b+1)/8 step
    signal s_chalf : unsigned(9 downto 0) := (others => '0');
    -- chroma quantizer constants for the write side (Dither In) and the
    -- output (Dither Out); the other side only rounds / clamps
    signal s_wco0, s_wco1, s_wco2, s_wco3 : unsigned(9 downto 0) := to_unsigned(2, 10);
    signal s_oco0, s_oco1, s_oco2, s_oco3 : unsigned(9 downto 0) := (others => '0');
    signal s_wmask, s_omask : unsigned(9 downto 0) := (others => '1');
    signal s_ldith : std_logic := '0';
    signal s_fphase, s_lphase : unsigned(17 downto 0) := (others => '0');
    signal s_hphase : unsigned(15 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_rx : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_wr_x : unsigned(10 downto 0) := (others => '0');          -- x of the s_in pixel (stage 1)
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_lw : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    -- output-side dither coordinates (x of the stage-14 pixel, its line parity)
    signal s_ox : unsigned(1 downto 0) := (others => '0');
    signal s_oline : std_logic := '0';

    -- x by stage (s_xs(k) = x of the stage-k pixel; stage 1 is s_wr_x)
    type t_x is array (2 to 8) of unsigned(10 downto 0);
    signal s_xs : t_x := (others => (others => '0'));
    -- write path: chroma round / dither + mid-tread quantize (Dither In) or
    -- round to the 8-bit store (Dither Out)
    signal s_cu2, s_cv2 : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal s_cuq3, s_cvq3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- band / address path
    signal s_band2 : unsigned(2 downto 0) := (others => '0');
    signal s_lph2, s_lph3 : unsigned(7 downto 0) := (others => '0');
    signal s_off3 : unsigned(7 downto 0) := (others => '0');
    signal s_ba3, s_ba4 : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_ph4 : unsigned(7 downto 0) := (others => '0');
    signal s_sa5, s_ca5 : unsigned(7 downto 0) := (others => '0');
    type t_ph is array (5 to 12) of unsigned(7 downto 0);
    signal s_phd : t_ph := (others => (others => '0'));
    signal s_ampe5 : unsigned(11 downto 0) := (others => '0');
    signal s_ampe6, s_ampe7a, s_ampe7b : unsigned(7 downto 0) := (others => '0');   -- one copy per product
    attribute keep of s_ampe7a, s_ampe7b : signal is true;
    signal s_sin6, s_cos6, s_sin7, s_cos7 : signed(9 downto 0) := (others => '0');
    signal s_ps8h, s_ps8l, s_pc8h, s_pc8l : signed(14 downto 0) := (others => '0');  -- sine * amp hi / lo nibble
    signal s_ay9, s_ac9 : signed(12 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');
    -- read side
    signal s_wet_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_sxb is array (11 to 14) of std_logic;
    signal s_sxb : t_sxb := (others => '0');                          -- source x bit 1 (Dither In cell)
    -- hue rotation
    signal s_uc13, s_vc13, s_ucd14, s_vcd14, s_ucd15, s_vcd15 : signed(10 downto 0) := (others => '0');
    signal s_th13, s_ha14, s_hc14 : unsigned(7 downto 0) := (others => '0');
    signal s_hs15, s_hcs15 : signed(9 downto 0) := (others => '0');
    -- stage-16 operand copies, one per multiplier (keep: yosys would re-merge them)
    signal s_hs16u, s_hs16v, s_hcs16u, s_hcs16v : signed(9 downto 0) := (others => '0');
    signal s_ua16, s_ub16, s_va16, s_vb16 : signed(10 downto 0) := (others => '0');
    attribute keep of s_hs16u, s_hs16v, s_hcs16u, s_hcs16v, s_ua16, s_ub16, s_va16, s_vb16 : signal is true;
    signal s_uc17h, s_us17h, s_vc17h, s_vs17h : signed(15 downto 0) := (others => '0');  -- chroma * trig(9..5)
    signal s_uc17l, s_us17l, s_vc17l, s_vs17l : signed(16 downto 0) := (others => '0');  -- chroma * trig(4..0)
    signal s_uc18, s_us18, s_vc18, s_vs18 : signed(20 downto 0) := (others => '0');
    signal s_ur19, s_vr19 : signed(21 downto 0) := (others => '0');
    signal s_u20, s_v20 : signed(10 downto 0) := (others => '0');
    signal s_ugh21, s_vgh21 : signed(15 downto 0) := (others => '0');
    signal s_ugl21, s_vgl21 : signed(16 downto 0) := (others => '0');
    signal s_ug22, s_vg22 : signed(12 downto 0) := (others => '0');
    signal s_uq23, s_vq23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_cu24, s_cv24 : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_cell is array (16 to 23) of unsigned(1 downto 0);
    signal s_scp : t_cell := (others => (others => '0'));            -- screen dither cell by stage
    signal s_lcell15, s_scell15 : unsigned(1 downto 0) := (others => '0');
    -- luma posterize: normalize -> dither -> keep L bits -> replicate -> scale back
    signal s_a13 : unsigned(9 downto 0) := (others => '0');
    signal s_n1_14 : unsigned(10 downto 0) := (others => '0');
    signal s_n2_14 : unsigned(6 downto 0) := (others => '0');
    signal s_yn15, s_t16, s_rep17 : unsigned(9 downto 0) := (others => '0');
    signal s_d1_18 : unsigned(9 downto 0) := (others => '0');
    signal s_d2_18 : unsigned(4 downto 0) := (others => '0');
    signal s_yq19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_y is array (20 to 25) of unsigned(9 downto 0);
    signal s_yd : t_y := (others => to_unsigned(64, 10));
    -- output conditioning: neutral gate, [1 2 1] window (g1 the centre), gate
    signal s_y26, s_g1y, s_g2y, s_y28, s_yo : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26, s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u28, s_v28, s_uo, s_vo : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');               -- s_avid_sr(k) = avid of the stage-(k+1) pixel
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    signal s_we13 : signed(12 downto 0) := (others => '0');            -- the same, registered, signed

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    -- top L = z+1 bits of t, replicated down to 10 bits
    function f_rep10(t : unsigned(9 downto 0); z : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(z) is
            when 0 => r := (others => t(9));
            when 1 => r := t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8);
            when 2 => r := t(9 downto 7) & t(9 downto 7) & t(9 downto 7) & t(9);
            when 3 => r := t(9 downto 6) & t(9 downto 6) & t(9 downto 8);
            when 4 => r := t(9 downto 5) & t(9 downto 5);
            when 5 => r := t(9 downto 4) & t(9 downto 6);
            when 6 => r := t(9 downto 3) & t(9 downto 7);
            when others => r := t(9 downto 2) & t(9 downto 8);
        end case;
        return r;
    end function;

    -- centred chroma: keep the masked level, clamp to 64..960
    function f_cq(t : unsigned(10 downto 0); m : unsigned(9 downto 0)) return unsigned is
        variable q : unsigned(9 downto 0);
    begin
        q := t(9 downto 0) and m;
        if t(10) = '1' or q > 960 then
            q := to_unsigned(960, 10);
        elsif q < 64 then
            q := to_unsigned(64, 10);
        end if;
        return q;
    end function;

begin

    s_k_amp    <= unsigned(registers_in(0));
    s_k_waves  <= unsigned(registers_in(1));
    s_k_hue    <= unsigned(registers_in(2));
    s_k_sat    <= unsigned(registers_in(3));
    s_k_lbits  <= unsigned(registers_in(4));
    s_k_cbits  <= unsigned(registers_in(5));
    s_sw_bandamp  <= registers_in(6)(0);
    s_sw_wobfirst <= registers_in(6)(1);
    s_sw_cquarter <= registers_in(6)(2);
    s_sw_huewob   <= registers_in(6)(3);
    s_sw_dither   <= registers_in(6)(4);
    s_p_spin <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_z  : unsigned(2 downto 0);
        variable v_lo : unsigned(6 downto 0);
        variable v_tg, v_cur, v_dg : signed(14 downto 0);
        variable v_s8, v_s38 : unsigned(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_sq : unsigned(17 downto 0);
        variable v_am : unsigned(15 downto 0);
        variable v_ah : unsigned(8 downto 0);
        variable v_sm : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_avid_sr(0) = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            -- output-side dither coordinates
            if s_avid_sr(13) = '1' then s_ox <= s_ox + 1; else s_ox <= (others => '0'); end if;
            if s_avid_sr(13) = '0' and s_avid_sr(14) = '1' then s_oline <= not s_oline; end if;

            -- resolution scaling: mode steps (static per mode); K1 to real
            -- px (720p x 2/3, SD x 1/2)
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_vys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_amp_k <= s_k_amp(9 downto 2);
            s_a168 <= shift_left(resize(s_amp_k, 16), 7) + shift_left(resize(s_amp_k, 16), 5) + shift_left(resize(s_amp_k, 16), 3);
            s_a3r  <= shift_left(resize(s_amp_k, 16), 1) + resize(s_amp_k, 16) + 128;
            v_am := s_a168 + s_a3r;
            case s_vxs is
                when "011"  => s_amp <= v_am(15 downto 8);
                when "100"  => v_ah := resize(s_amp_k, 9) + 1; s_amp <= v_ah(8 downto 1);
                when others => s_amp <= s_amp_k;
            end case;
            -- Waves line step = (s_gw * y step) >> 10 (1080p: s_gw >> 2 as
            -- before): load at 0, 12 shift-add steps, latch at 13
            s_sm_cnt <= s_sm_cnt + 1;
            if s_sm_cnt = 0 then
                s_sm_a <= s_gw; s_sm_b <= s_vys; s_sm_p <= (others => '0');
            elsif s_sm_cnt <= 12 then
                if s_sm_b(0) = '1' then v_sm := ('0' & s_sm_p(25 downto 12)) + ('0' & s_sm_a);
                else v_sm := '0' & s_sm_p(25 downto 12); end if;
                s_sm_p <= v_sm & s_sm_p(11 downto 1);
                s_sm_b <= '0' & s_sm_b(11 downto 1);
            elsif s_sm_cnt = 13 then
                s_wstep <= s_sm_p(25 downto 10);
            end if;
            s_hue <= s_k_hue(9 downto 2);
            s_hp  <= s_hue + s_hphase(15 downto 8);
            s_gain <= s_k_sat(9 downto 1); s_gain_a <= s_gain;

            -- spin speed = sign * (|P12 - centre| - 8)^2 / 128: deadband 8,
            -- top 1976 = 7.7 hue steps a field (v0.1 topped out at 1.0)
            s_spin_r <= s_p_spin;
            if s_spin_r(9) = '1' then v_m := '0' & s_spin_r(8 downto 0); else v_m := '0' & not s_spin_r(8 downto 0); end if;
            v_d := signed('0' & v_m) - 8;
            if v_d < 0 then s_spin_m <= (others => '0'); else s_spin_m <= unsigned(v_d(8 downto 0)); end if;
            s_spin_n1 <= not s_spin_r(9);
            s_spin_sqh <= s_spin_m * s_spin_m(8 downto 5);
            s_spin_sql <= s_spin_m * s_spin_m(4 downto 0);
            s_spin_n2 <= s_spin_n1;
            v_sq := shift_left(resize(s_spin_sqh, 18), 5) + resize(s_spin_sql, 18);
            s_spin_mag <= v_sq(17 downto 7);
            s_spin_n3 <= s_spin_n2;
            if s_spin_n3 = '1' then s_spin_inc <= -signed(resize(s_spin_mag, 16));
            else s_spin_inc <= signed(resize(s_spin_mag, 16)); end if;

            -- K5 / K6 bit-depth zones: change only 12 counts inside the new zone
            v_z := s_k_lbits(9 downto 7); v_lo := s_k_lbits(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_zl <= v_z; end if;
            v_z := s_k_cbits(9 downto 7); v_lo := s_k_cbits(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_zc <= v_z; end if;
            -- quantizer constants (step = 512 >> zone)
            s_lstep <= shift_right(to_unsigned(512, 11), to_integer(s_zl));
            s_cstep <= shift_right(to_unsigned(512, 11), to_integer(s_zc));
            s_cmask <= not (s_cstep(9 downto 0) - 1);
            v_s8 := shift_right(s_lstep, 3); v_s38 := shift_right(s_lstep, 2) + shift_right(s_lstep, 3);
            s_ld0 <= -signed(v_s38); s_ld1 <= -signed(v_s8);
            s_ld2 <= signed(v_s8);   s_ld3 <= signed(v_s38);
            v_s8 := shift_right(s_cstep, 3);
            s_cd0 <= resize(v_s8, 10);
            s_cd1 <= resize(shift_right(s_cstep, 2) + v_s8, 10);
            s_cd2 <= resize(shift_right(s_cstep, 1) + v_s8, 10);
            s_cd3 <= resize(s_cstep - v_s8, 10);
            s_chalf <= resize(shift_right(s_cstep, 1), 10);
            -- S8: Dither In posterizes chroma on the write side (before the
            -- rotation) and only clamps at the output; Dither Out stores it
            -- rounded to 8 bits and posterizes after the rotation
            if s_sw_wobfirst = '0' then
                s_wmask <= s_cmask;
                if s_sw_dither = '1' then
                    s_wco0 <= s_cd0; s_wco1 <= s_cd1; s_wco2 <= s_cd2; s_wco3 <= s_cd3;
                else
                    s_wco0 <= s_chalf; s_wco1 <= s_chalf; s_wco2 <= s_chalf; s_wco3 <= s_chalf;
                end if;
                s_omask <= (others => '1');
                s_oco0 <= (others => '0'); s_oco1 <= (others => '0'); s_oco2 <= (others => '0'); s_oco3 <= (others => '0');
            else
                s_wmask <= "1111111100";
                s_wco0 <= to_unsigned(2, 10); s_wco1 <= to_unsigned(2, 10); s_wco2 <= to_unsigned(2, 10); s_wco3 <= to_unsigned(2, 10);
                s_omask <= s_cmask;
                if s_sw_dither = '1' then
                    s_oco0 <= s_cd0; s_oco1 <= s_cd1; s_oco2 <= s_cd2; s_oco3 <= s_cd3;
                else
                    s_oco0 <= s_chalf; s_oco1 <= s_chalf; s_oco2 <= s_chalf; s_oco3 <= s_chalf;
                end if;
            end if;
            s_ldith <= s_sw_dither;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13 <= signed(resize(s_w_edge, 13));

            -- line events
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                -- this line writes bank s_wpar and reads the other (the line above)
                s_lw <= s_wpar;
                s_line_par <= s_hpar;
                if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_lw = '1' then s_rowswap <= s_line_par xor s_cap_par0;
            else s_rowswap <= s_line_par xor s_cap_par1; end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_lphase <= s_lphase + resize(s_wstep, 18);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0'; s_oline <= '0';
                if s_saw_active = '1' then
                    if s_zwarm /= 15 then s_zwarm <= s_zwarm + 1; end if;
                    s_fphase <= s_fphase + 1600;                     -- v0.1's 400 at 16 bits
                    s_hphase <= s_hphase + unsigned(s_spin_inc);
                    -- glide Waves by 1/8 of the gap per frame
                    v_tg := signed(resize(unsigned'(s_k_waves & "0000"), 15)); v_cur := signed(resize(s_gw, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 3);
                    s_gw <= unsigned(v_cur(13 downto 0));
                end if;
                s_lphase <= s_fphase;
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

    -- write path: luma stored as is at stage 1; chroma rounded / posterized,
    -- stored at stage 3
    p_wr : process(clk)
        variable v_b  : unsigned(1 downto 0);
        variable v_dc : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_xs(2) <= s_wr_x;
            for i in 3 to 8 loop s_xs(i) <= s_xs(i - 1); end loop;
            -- 2: 2x2 Bayer cell (2 px wide) from x and line parity; offset
            v_b := (s_wr_x(1) xor s_line(0)) & s_line(0);
            case to_integer(v_b) is
                when 0 => v_dc := s_wco0;
                when 1 => v_dc := s_wco1;
                when 2 => v_dc := s_wco2;
                when others => v_dc := s_wco3;
            end case;
            s_cu2 <= resize(s_in_u, 11) + resize(v_dc, 11);
            s_cv2 <= resize(s_in_v, 11) + resize(v_dc, 11);
            -- 3: level
            s_cuq3 <= f_cq(s_cu2, s_wmask);
            s_cvq3 <= f_cq(s_cv2, s_wmask);
        end if;
    end process p_wr;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin6 <= to_signed(C_SINE(to_integer(s_sa5)), 10);
            s_cos6 <= to_signed(C_SINE(to_integer(s_ca5)), 10);
            s_sin7 <= s_sin6; s_cos7 <= s_cos6;
            s_hs15 <= to_signed(C_SINE(to_integer(s_ha14)), 10);
            s_hcs15 <= to_signed(C_SINE(to_integer(s_hc14)), 10);
            s_hs16u <= s_hs15; s_hs16v <= s_hs15; s_hcs16u <= s_hcs15; s_hcs16v <= s_hcs15;
        end if;
    end process;

    p_addr : process(clk)
        procedure edge(signal src : in signed(12 downto 0); signal addr : out unsigned(10 downto 0)) is
        begin
            if src < C_EDGE then addr <= to_unsigned(C_EDGE, 11);
            elsif src > s_we13 then addr <= s_w_edge;
            else addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            -- 2: band from the input luma; the line's phase, held for the line
            s_band2 <= s_in_y(9 downto 7);
            if s_avid_sr(0) = '0' then s_lph2 <= s_lphase(17 downto 10); end if;
            -- 3: Phase: band k is k/16 of a cycle ahead / Amp: gain (k+1)/8
            s_lph3 <= s_lph2;
            if s_sw_bandamp = '1' then
                s_off3 <= (others => '0');
                s_ba3 <= resize(s_band2, 4) + 1;
            else
                s_off3 <= '0' & s_band2 & "0000";
                s_ba3 <= to_unsigned(8, 4);
            end if;
            -- 4: phase
            s_ph4 <= s_lph3 + s_off3;
            s_ba4 <= s_ba3;
            -- 5: ROM addresses; amplitude * band gain (x8)
            s_sa5 <= s_ph4;
            if s_sw_cquarter = '1' then s_ca5 <= s_ph4 + 64; else s_ca5 <= s_ph4; end if;
            s_phd(5) <= s_ph4;
            for i in 6 to 12 loop s_phd(i) <= s_phd(i - 1); end loop;
            s_ampe5 <= s_amp * s_ba4;
            -- 6, 7 (ROM out 6, fabric 7)
            s_ampe6 <= s_ampe5(10 downto 3);                          -- 0..255
            s_ampe7a <= s_ampe6; s_ampe7b <= s_ampe6;
            -- 8: products, amplitude split into nibbles
            s_ps8h <= s_sin7 * signed('0' & s_ampe7a(7 downto 4)); s_ps8l <= s_sin7 * signed('0' & s_ampe7a(3 downto 0));
            s_pc8h <= s_cos7 * signed('0' & s_ampe7b(7 downto 4)); s_pc8l <= s_cos7 * signed('0' & s_ampe7b(3 downto 0));
            -- 9: addresses (half products recombined here: two short adds)
            s_ay9 <= signed(resize(s_xs(8), 13)) + resize(shift_right(shift_left(resize(s_ps8h, 19), 4) + resize(s_ps8l, 19), 8), 13);
            s_ac9 <= signed(resize(s_xs(8), 13)) + resize(shift_right(shift_left(resize(s_pc8h, 19), 4) + resize(s_pc8l, 19), 8), 13);
            -- 10: edges
            edge(s_ay9, s_addrY);
            edge(s_ac9, s_addrC);
        end if;
    end process p_addr;

    -- line buffers: luma written at stage 1, chroma at 3; read at 11
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_avid_sr(0) = '1' and s_lw = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_avid_sr(0) = '1' and s_lw = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(2) = '1' and s_lw = '0' then lbU0(to_integer(s_xs(3)(10 downto 1))) <= std_logic_vector(s_cuq3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(2) = '1' and s_lw = '1' then lbU1(to_integer(s_xs(3)(10 downto 1))) <= std_logic_vector(s_cuq3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(2) = '1' and s_lw = '0' then lbV0(to_integer(s_xs(3)(10 downto 1))) <= std_logic_vector(s_cvq3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrC(10 downto 1)));
            if s_avid_sr(2) = '1' and s_lw = '1' then lbV1(to_integer(s_xs(3)(10 downto 1))) <= std_logic_vector(s_cvq3(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_n  : unsigned(10 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_dy : signed(10 downto 0);
        variable v_dc : unsigned(9 downto 0);
        variable v_x  : std_logic;
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 12: fetch the bank written by the line above; swap U/V when its
            --     hsync->avid parity differs from this line's
            if s_lw = '0' then
                s_wet_y <= unsigned(s_rdY1);
                if s_rowswap = '1' then s_wet_u <= unsigned(s_rdV1) & "00"; s_wet_v <= unsigned(s_rdU1) & "00";
                else s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00"; end if;
            else
                s_wet_y <= unsigned(s_rdY0);
                if s_rowswap = '1' then s_wet_u <= unsigned(s_rdV0) & "00"; s_wet_v <= unsigned(s_rdU0) & "00";
                else s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            end if;
            s_sxb(11) <= s_addrY(1);
            for i in 12 to 14 loop s_sxb(i) <= s_sxb(i - 1); end loop;

            -- 13: centred chroma; hue angle (follows the wobble phase when
            --     asked); luma - black
            s_uc13 <= signed(resize(s_wet_u, 11)) - 512;
            s_vc13 <= signed(resize(s_wet_v, 11)) - 512;
            if s_sw_huewob = '1' then s_th13 <= s_hp + s_phd(12); else s_th13 <= s_hp; end if;
            if s_wet_y < 64 then s_a13 <= (others => '0'); else s_a13 <= s_wet_y - 64; end if;
            -- 14: ROM addresses, sin = C(a + 64), cos = C(a + 128);
            --     luma * 1.168 (= 1023/876) in two halves
            s_ha14 <= s_th13 + 64;
            s_hc14 <= s_th13 xor x"80";
            s_ucd14 <= s_uc13; s_vcd14 <= s_vc13;
            s_n1_14 <= resize(s_a13, 11) + resize(shift_right(s_a13, 3), 11);
            s_n2_14 <= resize(shift_right(s_a13, 5), 7) + resize(shift_right(s_a13, 7), 7) + resize(shift_right(s_a13, 8), 7);
            -- 15 (ROM out): normalized luma 0..1023; dither cells of this pixel
            --     -- luma on the screen (Dither Out) or on the source column
            --     (Dither In: the pattern travels with the picture)
            s_ucd15 <= s_ucd14; s_vcd15 <= s_vcd14;
            v_n := s_n1_14 + resize(s_n2_14, 11);
            if v_n(10) = '1' then s_yn15 <= (others => '1'); else s_yn15 <= v_n(9 downto 0); end if;
            if s_sw_wobfirst = '1' then v_x := s_ox(1); else v_x := s_sxb(14); end if;
            s_lcell15 <= (v_x xor s_oline) & s_oline;
            s_scell15 <= (s_ox(1) xor s_oline) & s_oline;
            -- 16 (fabric copies): luma dither (signed, mean 0)
            s_ua16 <= s_ucd15; s_ub16 <= s_ucd15; s_va16 <= s_vcd15; s_vb16 <= s_vcd15;
            if s_ldith = '1' then
                case to_integer(s_lcell15) is
                    when 0 => v_dy := s_ld0;
                    when 1 => v_dy := s_ld1;
                    when 2 => v_dy := s_ld2;
                    when others => v_dy := s_ld3;
                end case;
            else
                v_dy := (others => '0');
            end if;
            v_t := signed(resize(s_yn15, 12)) + resize(v_dy, 12);
            if v_t < 0 then s_t16 <= (others => '0');
            elsif v_t > 1023 then s_t16 <= (others => '1');
            else s_t16 <= unsigned(v_t(9 downto 0)); end if;
            s_scp(16) <= s_scell15;
            for i in 17 to 23 loop s_scp(i) <= s_scp(i - 1); end loop;
            -- 17: half products (trig operand split hi/lo); keep L bits,
            --     replicate (level k of N -> k * 1023 / (N-1))
            s_uc17h <= s_ua16 * s_hcs16u(9 downto 5); s_uc17l <= s_ua16 * signed('0' & s_hcs16u(4 downto 0));
            s_us17h <= s_ub16 * s_hs16u(9 downto 5);  s_us17l <= s_ub16 * signed('0' & s_hs16u(4 downto 0));
            s_vc17h <= s_va16 * s_hcs16v(9 downto 5); s_vc17l <= s_va16 * signed('0' & s_hcs16v(4 downto 0));
            s_vs17h <= s_vb16 * s_hs16v(9 downto 5);  s_vs17l <= s_vb16 * signed('0' & s_hs16v(4 downto 0));
            s_rep17 <= f_rep10(s_t16, s_zl);
            -- 18: recombine; luma back toward the video range
            s_uc18 <= shift_left(resize(s_uc17h, 21), 5) + resize(s_uc17l, 21);
            s_us18 <= shift_left(resize(s_us17h, 21), 5) + resize(s_us17l, 21);
            s_vc18 <= shift_left(resize(s_vc17h, 21), 5) + resize(s_vc17l, 21);
            s_vs18 <= shift_left(resize(s_vs17h, 21), 5) + resize(s_vs17l, 21);
            s_d1_18 <= s_rep17 - shift_right(s_rep17, 3);
            s_d2_18 <= resize(shift_right(s_rep17, 6), 5) + resize(shift_right(s_rep17, 8), 5);
            -- 19: rotate, U' = U cos - V sin, V' = U sin + V cos;
            --     luma 64 + rep * 0.855
            s_ur19 <= resize(s_uc18, 22) - resize(s_vs18, 22);
            s_vr19 <= resize(s_us18, 22) + resize(s_vc18, 22);
            s_yq19 <= 64 + s_d1_18 - resize(s_d2_18, 10);
            -- 20: clamp; luma highlight knee 600 + (y - 600)/2 (white ~770)
            if shift_right(s_ur19, 8) > 511 then s_u20 <= to_signed(511, 11); elsif shift_right(s_ur19, 8) < -511 then s_u20 <= to_signed(-511, 11); else s_u20 <= resize(shift_right(s_ur19, 8), 11); end if;
            if shift_right(s_vr19, 8) > 511 then s_v20 <= to_signed(511, 11); elsif shift_right(s_vr19, 8) < -511 then s_v20 <= to_signed(-511, 11); else s_v20 <= resize(shift_right(s_vr19, 8), 11); end if;
            if s_yq19 > 600 then s_yd(20) <= 300 + shift_right(s_yq19, 1); else s_yd(20) <= s_yq19; end if;
            for i in 21 to 25 loop s_yd(i) <= s_yd(i - 1); end loop;
            -- 21: gain split
            s_ugh21 <= s_u20 * signed(resize(s_gain_a(8 downto 5), 5));
            s_ugl21 <= s_u20 * signed(resize(s_gain_a(4 downto 0), 6));
            s_vgh21 <= s_v20 * signed(resize(s_gain_a(8 downto 5), 5));
            s_vgl21 <= s_v20 * signed(resize(s_gain_a(4 downto 0), 6));
            -- 22: recombine the gain halves (Q7 -> centred 13-bit chroma)
            s_ug22 <= resize(shift_right(shift_left(resize(s_ugh21, 22), 5) + resize(s_ugl21, 22), 7), 13);
            s_vg22 <= resize(shift_right(shift_left(resize(s_vgh21, 22), 5) + resize(s_vgl21, 22), 7), 13);
            -- 23: recentre + clamp
            s_uq23 <= f_clamp10(s_ug22 + 512);
            s_vq23 <= f_clamp10(s_vg22 + 512);
            -- 24: round / dither offset (screen cell)
            case to_integer(s_scp(23)) is
                when 0 => v_dc := s_oco0;
                when 1 => v_dc := s_oco1;
                when 2 => v_dc := s_oco2;
                when others => v_dc := s_oco3;
            end case;
            s_cu24 <= resize(s_uq23, 11) + resize(v_dc, 11);
            s_cv24 <= resize(s_vq23, 11) + resize(v_dc, 11);
            -- 25: posterize (Dither Out) or just clamp 64..960 (Dither In)
            s_u25 <= f_cq(s_cu24, s_omask);
            s_v25 <= f_cq(s_cv24, s_omask);
            -- 26: neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(24) = '1' then
                s_y26 <= s_yd(25); s_u26 <= s_u25; s_v26 <= s_v25;
            else
                s_y26 <= to_unsigned(64, 10); s_u26 <= to_unsigned(512, 10); s_v26 <= to_unsigned(512, 10);
            end if;
            -- 27..28: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y26; s_g2y <= s_g1y;
            s_g1u <= s_u26; s_g2u <= s_g1u;
            s_g1v <= s_v26; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y26, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u26, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v26, 12);
            s_y28 <= v_sy(11 downto 2); s_u28 <= v_su(11 downto 2); s_v28 <= v_sv(11 downto 2);
            -- 29: gate
            if s_avid_sr(27) = '1' then
                s_yo <= s_y28; s_uo <= s_u28; s_vo <= s_v28;
            else
                s_yo <= to_unsigned(64, 10); s_uo <= to_unsigned(512, 10); s_vo <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_yo);
    s_io_align_in.u       <= std_logic_vector(s_uo);
    s_io_align_in.v       <= std_logic_vector(s_vo);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (29 -> 32).
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

end architecture hueflutter;
