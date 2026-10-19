-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Flutter: catalogue combo #60 (seed 20260921) -- magnetic warp (wobble)
-- + posterization (bit-depth reduction per channel).  Three knobs each;
-- the switches and the slider decide how they combine.
--
--   K1 Wobble   displacement amplitude (+-255 px at 1080p; scaled per mode)
--   K2 Waves    vertical wavelength (phase step per line; glided)
--   K3 Speed    wobble speed, bipolar, squared (centre deadband)
--   K4 Luma     luma levels 1..8 bits -- also the wobble bands: each
--               posterized level slides as one band
--   K5 Chroma   chroma levels 1..8 bits
--   K6 Spread   per-band phase (or amplitude) spread
--
--   S7  Bands   bands differ in phase / in amplitude
--   S8  Wave    smooth sine / snapped (flat-topped) wave
--   S9  Chroma  chroma wobbles with luma / a quarter wave apart (fringes)
--   S10 Edges   outside the line: edge fill / mirrored
--   S11 Dither  plain quantizer / 2x2 dithered quantizer
--   P12 Twist   horizontal phase term (glided): 0 = pure line wobble,
--               up = 2-D ripple
--
-- Posterize: on the write side (a pixel shift commutes with a per-pixel
-- quantizer, so v0.1's posterize-after-wobble order looked identical and
-- S8 was dead without dither).  Luma levels span the video range
-- (normalize (y-64)*1.168, keep L bits, bit-replicate, scale back by
-- 0.855: black 64 and white ~940 are always levels).  Chroma is a centred
-- mid-tread quantizer (levels at 512 +- k*step, clamped 64..960), so
-- greys stay grey at every depth.  The band of a pixel is its posterized
-- luma level: level/N of a cycle (Phase) or the bit-replicated level
-- (Amp), times Spread.
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (20) in all modes; read
-- and write banks latched per line at the avid rise (the pipeline tail
-- runs past hsync on SD/PAL front porches); U/V swapped on rows whose
-- hsync->avid parity differs from the previous row's (each row replays
-- the row above -- drape v1.3); output conditioning ([1 2 1] on Y/U/V +
-- luma scaled to white ~777: the HD HDMI bar fix from damier v0.4.4 --
-- posterized steps are near-full-swing pixel edges that move every
-- frame); no_rw_check on every RAM (a bank is only ever read while the
-- other one is written; the write bank's read data is discarded).
--
-- v0.2.3 (2026-10-17): resolution scaling.  The wobble lives on a virtual
-- 1080-line canvas, stepped per mode from the video timing ID
-- (registers_in(8)): K1's amplitude is 1080-line pixels (x 2/3 at 720p,
-- x 1/2 at SD, where 720 px span 1440 units), the Waves phase step per
-- field line is scaled by the virtual line step (1.5 at 720p, 1080/486 x 2
-- per 480i field line; a serial multiplier, once per frame) and the Twist
-- step per pixel by the virtual x step, so the ripple has the same size and
-- the same number of waves relative to the screen height in every mode.
-- 1080p is unchanged (every step is exactly 1.0).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture flutter of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LAT : integer := 20;

    -- wave ROM: 0..255 the sine (-cos, index 0 = -255), 256..511 the same
    -- wave snapped flat (255 tanh(4s)/tanh(4)) for S8
    type t_wave_rom is array (0 to 511) of integer range -256 to 255;
    constant C_WAVE : t_wave_rom := (
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
        -250, -251, -252, -253, -254, -254, -255, -255,
        -255, -255, -255, -255, -255, -255, -255, -255,
        -255, -255, -255, -255, -255, -255, -255, -255,
        -255, -255, -255, -255, -255, -255, -255, -255,
        -255, -254, -254, -254, -254, -254, -254, -254,
        -253, -253, -253, -252, -252, -251, -251, -250,
        -249, -248, -247, -245, -244, -242, -239, -236,
        -232, -228, -223, -217, -210, -201, -191, -180,
        -167, -152, -135, -116,  -95,  -73,  -49,  -25,
           0,   25,   49,   73,   95,  116,  135,  152,
         167,  180,  191,  201,  210,  217,  223,  228,
         232,  236,  239,  242,  244,  245,  247,  248,
         249,  250,  251,  251,  252,  252,  253,  253,
         253,  254,  254,  254,  254,  254,  254,  254,
         255,  255,  255,  255,  255,  255,  255,  255,
         255,  255,  255,  255,  255,  255,  255,  255,
         255,  255,  255,  255,  255,  255,  255,  255,
         255,  255,  255,  255,  255,  255,  255,  255,
         255,  255,  255,  255,  255,  255,  255,  255,
         255,  255,  255,  255,  255,  255,  255,  255,
         255,  254,  254,  254,  254,  254,  254,  254,
         253,  253,  253,  252,  252,  251,  251,  250,
         249,  248,  247,  245,  244,  242,  239,  236,
         232,  228,  223,  217,  210,  201,  191,  180,
         167,  152,  135,  116,   95,   73,   49,   25,
           0,  -25,  -49,  -73,  -95, -116, -135, -152,
        -167, -180, -191, -201, -210, -217, -223, -228,
        -232, -236, -239, -242, -244, -245, -247, -248,
        -249, -250, -251, -251, -252, -252, -253, -253,
        -253, -254, -254, -254, -254, -254, -254, -254,
        -255, -255, -255, -255, -255, -255, -255, -255,
        -255, -255, -255, -255, -255, -255, -255, -255,
        -255, -255, -255, -255, -255, -255, -255, -255
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

    signal s_k_amp, s_k_waves, s_k_speed, s_k_lbits, s_k_cbits, s_k_spread : unsigned(9 downto 0);
    signal s_sw_bandamp, s_sw_snap, s_sw_cquarter, s_sw_mirror, s_sw_dither : std_logic;
    signal s_p_twist : unsigned(9 downto 0);

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
    -- Twist phase step per pixel = (glided Twist * x step) >> 5
    signal s_tstep  : unsigned(11 downto 0) := (others => '0');

    -- per-frame controls
    signal s_amp    : unsigned(7 downto 0) := (others => '0');
    signal s_spread : unsigned(7 downto 0) := (others => '0');
    signal s_gw, s_gt : unsigned(13 downto 0) := (others => '0');      -- glided Waves / Twist (10.4)
    signal s_spd_d  : unsigned(8 downto 0) := (others => '0');
    signal s_spd_p  : unsigned(13 downto 0) := (others => '0');
    signal s_spd_neg, s_spd_neg2 : std_logic := '0';
    -- bit depths: zone = bits - 1, hysteretic across the zone boundaries
    signal s_zl, s_zc : unsigned(2 downto 0) := to_unsigned(2, 3);
    -- frames since load; until it reaches 15 the K4/K5 zones follow the knob
    -- directly (no hysteresis), so a saved default near a zone edge loads in
    -- its own zone instead of staying at the power-up zone
    signal s_zwarm : unsigned(3 downto 0) := (others => '0');
    signal s_lstep, s_cstep : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_lmask, s_cmask : unsigned(9 downto 0) := (others => '1');
    signal s_ld0, s_ld1, s_ld2, s_ld3 : signed(10 downto 0) := (others => '0');    -- luma dither, (2b-3)/8 step
    signal s_cd0, s_cd1, s_cd2, s_cd3 : unsigned(9 downto 0) := (others => '0');  -- chroma round + dither, (2b+1)/8 step
    signal s_chalf : unsigned(9 downto 0) := (others => '0');
    signal s_fphase, s_lphase : unsigned(17 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_rx : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_wr_x : unsigned(10 downto 0) := (others => '0');          -- x of the s_in pixel (stage 1)
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_lw : std_logic := '0';
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    signal s_w2m1 : signed(12 downto 0) := (others => '0');

    -- x by stage (s_xs(k) = x of the stage-k pixel; stage 1 is s_wr_x)
    type t_x is array (2 to 12) of unsigned(10 downto 0);
    signal s_xs : t_x := (others => (others => '0'));
    -- write path: luma normalize -> dither -> keep L bits -> replicate -> scale back
    signal s_db2, s_db3, s_db4 : unsigned(1 downto 0) := (others => '0');
    signal s_a2  : unsigned(9 downto 0) := (others => '0');
    signal s_n13 : unsigned(10 downto 0) := (others => '0');
    signal s_n23 : unsigned(6 downto 0) := (others => '0');
    signal s_yn4 : unsigned(9 downto 0) := (others => '0');
    signal s_t5, s_rep6 : unsigned(9 downto 0) := (others => '0');
    signal s_d17 : unsigned(9 downto 0) := (others => '0');
    signal s_d27 : unsigned(4 downto 0) := (others => '0');
    signal s_yq8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_cu2, s_cv2 : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal s_cuq3, s_cvq3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- band / address path
    signal s_lvl5 : unsigned(7 downto 0) := (others => '0');
    signal s_bh6, s_bl6 : unsigned(11 downto 0) := (others => '0');
    signal s_off7 : unsigned(7 downto 0) := (others => '0');
    signal s_xph7 : unsigned(17 downto 0) := (others => '0');
    signal s_ph8 : unsigned(7 downto 0) := (others => '0');
    signal s_ba8 : unsigned(8 downto 0) := (others => '0');
    signal s_sa9, s_ca9 : unsigned(8 downto 0) := (others => '0');
    signal s_ampe9, s_ampe10, s_ampe11 : unsigned(8 downto 0) := (others => '0');
    signal s_sin10, s_cos10, s_sin11, s_cos11 : signed(9 downto 0) := (others => '0');
    signal s_ps12, s_pc12 : signed(19 downto 0) := (others => '0');
    signal s_ay13, s_ac13 : signed(12 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');
    -- read side
    signal s_wet_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    -- per-line clamp bounds (registered) so the address compares run on the
    -- source value in parallel with the mirror negate / subtract
    signal s_we13, s_wl13, s_nwe13, s_rce13, s_rwe13 : signed(12 downto 0) := (others => '0');

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
    s_k_speed  <= unsigned(registers_in(2));
    s_k_lbits  <= unsigned(registers_in(3));
    s_k_cbits  <= unsigned(registers_in(4));
    s_k_spread <= unsigned(registers_in(5));
    s_sw_bandamp  <= registers_in(6)(0);
    s_sw_snap     <= registers_in(6)(1);
    s_sw_cquarter <= registers_in(6)(2);
    s_sw_mirror   <= registers_in(6)(3);
    s_sw_dither   <= registers_in(6)(4);
    s_p_twist <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_d  : unsigned(9 downto 0);
        variable v_z  : unsigned(2 downto 0);
        variable v_lo : unsigned(6 downto 0);
        variable v_tg, v_cur, v_dg : signed(14 downto 0);
        variable v_s8, v_s38 : unsigned(10 downto 0);
        variable v_am : unsigned(15 downto 0);
        variable v_ah : unsigned(8 downto 0);
        variable v_ts : unsigned(16 downto 0);
        variable v_sm : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_avid_sr(0) = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- resolution scaling: mode steps (static per mode); K1 to real
            -- px (720p x 2/3, SD x 1/2); Twist step x the x step
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
            case s_vxs is
                when "011"  => v_ts := shift_left(resize(s_gt, 17), 1) + resize(s_gt, 17);
                when "100"  => v_ts := shift_left(resize(s_gt, 17), 2);
                when others => v_ts := shift_left(resize(s_gt, 17), 1);
            end case;
            s_tstep <= v_ts(16 downto 5);
            s_spread <= s_k_spread(9 downto 2);

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

            -- K3 Speed: |k-512| less a 24-count deadband, then d*(d/16+1)
            -- (squared: the old full speed is reached at ~30%, the top is ~15x it)
            if s_k_speed >= 512 then v_d := s_k_speed - 512; else v_d := 512 - s_k_speed; end if;
            if v_d > 24 then s_spd_d <= resize(v_d - 24, 9); else s_spd_d <= (others => '0'); end if;
            if s_k_speed < 512 then s_spd_neg <= '1'; else s_spd_neg <= '0'; end if;
            s_spd_p <= s_spd_d * (resize(s_spd_d(8 downto 4), 5) + 1);
            s_spd_neg2 <= s_spd_neg;

            -- K4 / K5 bit-depth zones: change only 12 counts inside the new zone
            v_z := s_k_lbits(9 downto 7); v_lo := s_k_lbits(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_zl <= v_z; end if;
            v_z := s_k_cbits(9 downto 7); v_lo := s_k_cbits(6 downto 0);
            if s_zwarm /= 15 or ((v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7)) then s_zc <= v_z; end if;
            -- quantizer constants (step = 512 >> zone)
            s_lstep <= shift_right(to_unsigned(512, 11), to_integer(s_zl));
            s_cstep <= shift_right(to_unsigned(512, 11), to_integer(s_zc));
            s_lmask <= not (s_lstep(9 downto 0) - 1);
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

            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;
            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13  <= signed(resize(s_w_edge, 13));                  -- w_edge
            s_wl13  <= signed(resize(s_line_width, 13));              -- width
            s_nwe13 <= -signed(resize(s_w_edge, 13));                 -- mirrored-left: n > w_edge  <=> src < -w_edge
            s_rce13 <= s_w2m1 - C_EDGE;                               -- mirrored-right: r < C_EDGE <=> src > 2w-1-C_EDGE
            s_rwe13 <= signed(resize(s_line_width, 13)) + C_EDGE;     -- mirrored-right: r > w_edge <=> src < w+C_EDGE

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
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    if s_zwarm /= 15 then s_zwarm <= s_zwarm + 1; end if;
                    if s_spd_neg2 = '1' then s_fphase <= s_fphase - resize(s_spd_p, 18);
                    else s_fphase <= s_fphase + resize(s_spd_p, 18); end if;
                    -- glide Waves and Twist by 1/8 of the gap per frame
                    v_tg := signed(resize(unsigned'(s_k_waves & "0000"), 15)); v_cur := signed(resize(s_gw, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 3);
                    s_gw <= unsigned(v_cur(13 downto 0));
                    v_tg := signed(resize(unsigned'(s_p_twist & "0000"), 15)); v_cur := signed(resize(s_gt, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 3);
                    s_gt <= unsigned(v_cur(13 downto 0));
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

    -- write path: posterize the input, stage 2..8 (chroma writes at 3)
    p_wr : process(clk)
        variable v_b  : unsigned(1 downto 0);
        variable v_dc : unsigned(9 downto 0);
        variable v_n  : unsigned(10 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_dy : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_xs(2) <= s_wr_x;
            for i in 3 to 12 loop s_xs(i) <= s_xs(i - 1); end loop;
            -- 2: dither cell (2x2 Bayer from x and line parity); luma - black;
            --    chroma + rounding/dither offset
            v_b := (s_wr_x(0) xor s_line(0)) & s_line(0);
            s_db2 <= v_b;
            if s_in_y < 64 then s_a2 <= (others => '0'); else s_a2 <= s_in_y - 64; end if;
            if s_sw_dither = '1' then
                case to_integer(v_b) is
                    when 0 => v_dc := s_cd0;
                    when 1 => v_dc := s_cd1;
                    when 2 => v_dc := s_cd2;
                    when others => v_dc := s_cd3;
                end case;
            else
                v_dc := s_chalf;
            end if;
            s_cu2 <= resize(s_in_u, 11) + resize(v_dc, 11);
            s_cv2 <= resize(s_in_v, 11) + resize(v_dc, 11);
            -- 3: luma * 1.168 (= 1023/876) in two halves; chroma level
            s_db3 <= s_db2;
            s_n13 <= resize(s_a2, 11) + resize(shift_right(s_a2, 3), 11);
            s_n23 <= resize(shift_right(s_a2, 5), 7) + resize(shift_right(s_a2, 7), 7) + resize(shift_right(s_a2, 8), 7);
            s_cuq3 <= f_cq(s_cu2, s_cmask);
            s_cvq3 <= f_cq(s_cv2, s_cmask);
            -- 4: normalized luma 0..1023
            s_db4 <= s_db3;
            v_n := s_n13 + resize(s_n23, 11);
            if v_n(10) = '1' then s_yn4 <= (others => '1'); else s_yn4 <= v_n(9 downto 0); end if;
            -- 5: dither (signed, mean 0)
            if s_sw_dither = '1' then
                case to_integer(s_db4) is
                    when 0 => v_dy := s_ld0;
                    when 1 => v_dy := s_ld1;
                    when 2 => v_dy := s_ld2;
                    when others => v_dy := s_ld3;
                end case;
            else
                v_dy := (others => '0');
            end if;
            v_t := signed(resize(s_yn4, 12)) + resize(v_dy, 12);
            if v_t < 0 then s_t5 <= (others => '0');
            elsif v_t > 1023 then s_t5 <= (others => '1');
            else s_t5 <= unsigned(v_t(9 downto 0)); end if;
            -- 6: keep L bits, replicate (level k of N -> k * 1023 / (N-1))
            s_rep6 <= f_rep10(s_t5, s_zl);
            -- 7, 8: back to the video range, 64 + rep * 0.855
            s_d17 <= s_rep6 - shift_right(s_rep6, 3);
            s_d27 <= resize(shift_right(s_rep6, 6), 5) + resize(shift_right(s_rep6, 8), 5);
            s_yq8 <= 64 + s_d17 - resize(s_d27, 10);
        end if;
    end process p_wr;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin10 <= to_signed(C_WAVE(to_integer(s_sa9)), 10);
            s_cos10 <= to_signed(C_WAVE(to_integer(s_ca9)), 10);
            s_sin11 <= s_sin10; s_cos11 <= s_cos10;
        end if;
    end process;

    p_addr : process(clk)
        variable v_o : unsigned(15 downto 0);
        variable v_a : unsigned(16 downto 0);
        procedure edge(signal src : in signed(12 downto 0); signal addr : out unsigned(10 downto 0)) is
            variable n, r : signed(12 downto 0);
        begin
            -- every compare is on src against a registered bound, in parallel
            -- with the negate / subtract; the mux is the only serial step
            n := -src; r := s_w2m1 - src;
            if s_sw_mirror = '1' and src < 0 then
                if src > -C_EDGE then addr <= to_unsigned(C_EDGE, 11);
                elsif src < s_nwe13 then addr <= s_w_edge;
                else addr <= unsigned(n(10 downto 0)); end if;
            elsif s_sw_mirror = '1' and src >= s_wl13 then
                if src > s_rce13 then addr <= to_unsigned(C_EDGE, 11);
                elsif src < s_rwe13 then addr <= s_w_edge;
                else addr <= unsigned(r(10 downto 0)); end if;
            elsif src < C_EDGE then addr <= to_unsigned(C_EDGE, 11);
            elsif src > s_we13 then addr <= s_w_edge;
            else addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            -- 5: band level from the posterized luma -- k/N of a cycle (Phase)
            --    or the replicated level, full scale at the top band (Amp)
            if s_sw_bandamp = '1' then
                s_lvl5 <= f_rep10(s_yn4, s_zl)(9 downto 2);
            else
                s_lvl5 <= s_yn4(9 downto 2) and s_lmask(9 downto 2);
            end if;
            -- 6, 7: level * Spread / 256, operand split hi/lo
            s_bh6 <= s_lvl5(7 downto 4) * s_spread;
            s_bl6 <= s_lvl5(3 downto 0) * s_spread;
            v_o := shift_left(resize(s_bh6, 16), 4) + resize(s_bl6, 16);
            s_off7 <= v_o(15 downto 8);
            -- 7: horizontal phase DDA (Twist), reloaded with the line phase
            if s_avid_sr(5) = '0' then s_xph7 <= s_lphase; else s_xph7 <= s_xph7 + resize(s_tstep, 18); end if;
            -- 8: phase and band amplitude (Q8 gain)
            if s_sw_bandamp = '1' then
                s_ph8 <= s_xph7(17 downto 10);
                s_ba8 <= resize(s_off7, 9) + 32;                    -- 32..287
            else
                s_ph8 <= s_xph7(17 downto 10) + s_off7;
                s_ba8 <= to_unsigned(256, 9);
            end if;
            -- 9: ROM addresses (S8 picks the snapped half); amplitude
            s_sa9 <= s_sw_snap & s_ph8;
            if s_sw_cquarter = '1' then s_ca9 <= s_sw_snap & (s_ph8 + 64); else s_ca9 <= s_sw_snap & s_ph8; end if;
            v_a := s_amp * s_ba8;
            s_ampe9 <= v_a(16 downto 8);                             -- 0..286
            -- 10, 11 (ROM out 10, fabric 11)
            s_ampe10 <= s_ampe9; s_ampe11 <= s_ampe10;
            -- 12: products
            s_ps12 <= s_sin11 * signed(resize(s_ampe11, 10));
            s_pc12 <= s_cos11 * signed(resize(s_ampe11, 10));
            -- 13: addresses
            s_ay13 <= signed(resize(s_xs(12), 13)) + resize(shift_right(s_ps12, 8), 13);
            s_ac13 <= signed(resize(s_xs(12), 13)) + resize(shift_right(s_pc12, 8), 13);
            -- 14: edges
            edge(s_ay13, s_addrY);
            edge(s_ac13, s_addrC);
        end if;
    end process p_addr;

    -- line buffers: luma written at stage 9 (stage-8 data), chroma at 4;
    -- read at 15
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_avid_sr(7) = '1' and s_lw = '0' then lbY0(to_integer(s_xs(8))) <= std_logic_vector(s_yq8); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_avid_sr(7) = '1' and s_lw = '1' then lbY1(to_integer(s_xs(8))) <= std_logic_vector(s_yq8); end if;
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
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 16: fetch the bank written by the line above; swap U/V when its
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
            -- 17: neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(15) = '1' then
                s_y17 <= s_wet_y; s_u17 <= s_wet_u; s_v17 <= s_wet_v;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- 18..19: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y17; s_g2y <= s_g1y;
            s_g1u <= s_u17; s_g2u <= s_g1u;
            s_g1v <= s_v17; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y17, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u17, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v17, 12);
            s_y19 <= v_sy(11 downto 2); s_u19 <= v_su(11 downto 2); s_v19 <= v_sv(11 downto 2);
            -- 20: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            v_d := s_y19 - 64;
            if s_avid_sr(18) = '1' then
                s_y20 <= s_y19 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u20 <= s_u19; s_v20 <= s_v19;
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

end architecture flutter;
