-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Sidewinder: per-scanline horizontal wave displacement.
--
-- Incoming video is written into a dual-bank line buffer and read back one
-- line later at a horizontally shifted address. The shift is a triangle (or
-- sine) wave of the line number, so the picture snakes left and right as it
-- goes down the screen; the wave phase advances every field so the bends
-- roll vertically.
--
--   offset(a)  = wave(phase(a)) * Amount + wave(3 * phase(a)) * Ripple
--   phase(a+1) = phase(a) + freq(a),  phase(0) = frame phase
--   freq(a)    = freq(Period) + (a - H/2) * Twist      (a = active line)
--   Y(x) = prev.Y(x + offset)
--   U(x) = prev.U(x + offset * (1 + Split))
--   V(x) = prev.V(x + offset * (1 - Split))
--
-- Controls:
--   P12 Amount  headline gesture: amplitude as a fraction of the measured
--               width, quadratic, from a 3% floor (so every other control
--               always shows) up to +-W/2 -- in Wrap the picture coils
--               round a tube
--   K1 Period   vertical wavelength, freq = K1^2 >> 8 per line (glided:
--               the phase at the bottom line is ~H * freq, so one LSB of
--               pot noise would otherwise shake the bottom of the screen)
--   K2 Rate     roll speed, bidirectional, centre detent = frozen,
--               quadratic away from centre
--   K3 Ripple   3x harmonic, quadratic, 0 .. +-W/8, independent of Amount
--   K4 Twist    bidirectional, centre detent: the wavelength changes down
--               the screen about the centre row; right bunches the bends
--               towards the bottom, left towards the top (glided)
--   K5 Split    chroma dispersion: U bends further than Y, V less
--               (0 .. U at 2x / V at 0x), quadratic
--   K6 Mix      dry/wet
--   S7 Shape    Triangle / Sine (sine table is -cos so shapes stay aligned)
--   S8 Swell    slow automatic triangle LFO on Amount (~6 s)
--   S9 Steps    quantize the offset to 16 px (16 virtual units) -> staircase
--   S10 Edge    Mirror / Wrap, both over the inset columns [12, W-13] so
--               the source's black blanking columns never reach the picture
--   S11 Bypass  mix forced dry (same latency)
--
-- All wave math runs in two step sequencers: once per field in vertical
-- blanking (knob curves, glides, width-relative scaling; one shared
-- registered multiplier) and once per line in horizontal blanking, kicked
-- at the active-video fall (phase/freq advance, wave lookups, products,
-- chroma split, edge folding; a second shared multiplier). The phase
-- advances on ACTIVE lines only and the frame anchor is the vsync edge
-- guarded by a saw-active flag, so serrated analog vsync and interlace
-- cannot change the roll speed or the field-to-field phase.
--
-- Per pixel there is no arithmetic beyond three run-along read pointers
-- (Y, U, V). Each is initialised once per line to its folded start column
-- and then steps +-1, wrapping (R -> L) or bouncing (hold the edge column
-- one pixel, turn) at the inset bounds. The pointers ARE the RAM read
-- addresses -- no comparator sits in front of RADDR.
--
-- Buffer = inlined dual-bank ping-pong BRAM: each active line WRITES its
-- own parity bank and READS the other (the line above); parity flips at the
-- active-video fall, on the same edge as the line's last write. Pixel k is
-- written at address k. Y 2048 deep; U/V 2:1 decimated (1024): 22 EBR plus
-- the sine ROM.
--
-- Pipeline (data_in -> data_out, 7 clocks):
--   E0     input registers; bank reads at the pointers; pointers step
--   E1     bank select -> s_wet_*
--   E2     blanking gate: dry and wet forced to 64/512/512 outside avid
--   E3-E6  dry/wet mix (split-multiply lerp, 4 clocks) -> data_out
--
-- v1.1.2 (2026-10-18) resolution scaling: the wave runs in virtual
-- 1080-line units from the video timing ID (registers_in(8)), 1080p output
-- unchanged bit for bit.  The line sequencer's phase/freq advance (step 0)
-- repeats once per VIRTUAL line the field line covers (720p 1-2, 480i 4-5),
-- so Period (a vertical wavelength) and Twist give the same bends per
-- screen height at every resolution, and the Twist pivot is the virtual
-- centre row.  S9 Steps quantizes to 16 virtual units (720p ~10.5 px,
-- SD 8 px).  Amount and Ripple were already fractions of the width.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture sidewinder of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (7 + 1 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_AW            : integer := 11;                  -- Y depth 2048
    constant C_TOTAL_LATENCY : integer := 7;

    -- Swell LFO speed: added to a 16-bit phase every field (~6 s per cycle).
    constant C_SWELL_RATE    : integer := 192;

    -- Inset bounds: source columns < C_EDGE or > W-1-C_EDGE never shown.
    constant C_EDGE          : integer := 12;
    -- Active line the field width is measured on (away from half lines).
    constant C_WIDTH_LINE    : integer := 8;
    -- Centre detent half-width on the bipolar knobs.
    constant C_DETENT        : integer := 8;
    -- Amount floor (of 1023): ~3% of the width at slider 0.
    constant C_AMP_FLOOR     : integer := 32;

    constant C_Y_BLACK       : unsigned(C_DW - 1 downto 0) := to_unsigned(64, C_DW);
    constant C_C_ZERO        : unsigned(C_DW - 1 downto 0) := to_unsigned(512, C_DW);

    -- Sequencer idle states (the last step of each sequence parks there).
    constant C_L_IDLE        : unsigned(5 downto 0) := "111111";
    constant C_L_LAST        : integer := 33;
    constant C_V_IDLE        : unsigned(4 downto 0) := "11111";
    constant C_V_LAST        : integer := 18;

    --------------------------------------------------------------------------
    -- Sine ROM: 256 x signed, value = round(-255*cos(2*pi*i/256)) so index 0
    -- is the wave minimum, matching the triangle fold below.
    --------------------------------------------------------------------------
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

    -- Triangle fold of a 16-bit phase to -256..255 (min at phase 0).
    -- Resolution scaling: Q4.8 virtual lines per FIELD line (the pilot's,
    -- mondrian, table; doubled for interlace).
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

    function f_tri(phase : unsigned(15 downto 0)) return signed is
        variable v_ramp : signed(9 downto 0);
    begin
        v_ramp := signed(resize(phase(14 downto 6), 10));  -- 0..511
        if phase(15) = '0' then
            return v_ramp - to_signed(256, 10);            -- -256 .. 255
        else
            return to_signed(255, 10) - v_ramp;            --  255 .. -256
        end if;
    end function f_tri;

    -- One pixel of a run-along read pointer. Up: at R wrap to L (Wrap) or
    -- hold R and turn (Mirror). Down (Mirror only): at L hold and turn.
    -- Returns dir & ptr.
    function f_bounce(ptr    : unsigned(C_AW - 1 downto 0);
                      dir    : std_logic;
                      r      : unsigned(C_AW - 1 downto 0);
                      mirror : std_logic) return unsigned is
        variable v : unsigned(C_AW downto 0);
    begin
        if dir = '0' then
            if ptr = r then
                if mirror = '1' then
                    v := '1' & ptr;
                else
                    v := '0' & to_unsigned(C_EDGE, C_AW);
                end if;
            else
                v := '0' & (ptr + 1);
            end if;
        elsif ptr = to_unsigned(C_EDGE, C_AW) then
            v := '0' & ptr;
        else
            v := '1' & (ptr - 1);
        end if;
        return v;
    end function f_bounce;

    -- |a - b| with the sign of (a - b) in the MSB.
    function f_sdiff(a, b : unsigned(C_DW - 1 downto 0)) return unsigned is
        variable v_d, v_nd : signed(C_DW downto 0);
        variable v_r       : unsigned(C_DW downto 0);
    begin
        v_d  := signed(resize(a, C_DW + 1)) - signed(resize(b, C_DW + 1));
        v_nd := signed(resize(b, C_DW + 1)) - signed(resize(a, C_DW + 1));
        if v_d(C_DW) = '1' then
            v_r := '1' & unsigned(v_nd(C_DW - 1 downto 0));
        else
            v_r := '0' & unsigned(v_d(C_DW - 1 downto 0));
        end if;
        return v_r;
    end function f_sdiff;

    -- b +/- (pm >> 10). pm <= w * |a - b| with w < 1024, so the result
    -- always lies between b and a: no clamp needed.
    function f_step(b : unsigned(C_DW - 1 downto 0); pm : unsigned(19 downto 0);
                    neg : std_logic) return unsigned is
    begin
        if neg = '1' then
            return b - pm(19 downto 10);
        else
            return b + pm(19 downto 10);
        end if;
    end function f_step;

    --------------------------------------------------------------------------
    -- Inlined dual-bank line buffers (initialised to black / neutral chroma)
    --------------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(C_DW - 1 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(C_DW - 1 downto 0);

    signal lbY0, lbY1 : t_lb_y  := (others => std_logic_vector(C_Y_BLACK));
    signal lbU0, lbU1 : t_lb_uv := (others => std_logic_vector(C_C_ZERO));
    signal lbV0, lbV1 : t_lb_uv := (others => std_logic_vector(C_C_ZERO));

    -- A bank is only displayed on the lines it is not written, so a
    -- read/write collision is harmless: skip yosys's read-before-write
    -- emulation (needs the init values on the read-data registers below).
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;

    signal s_rdY0, s_rdY1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Write side / line and field tracking
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_in_avid      : std_logic := '0';
    signal s_wr_x         : unsigned(C_AW - 1 downto 0) := (others => '0');
    signal s_par          : std_logic := '0';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_saw_active   : std_logic := '0';
    signal s_aline        : unsigned(10 downto 0) := (others => '0');
    signal s_height_m     : unsigned(10 downto 0) := to_unsigned(1080, 11);  -- virtual lines

    -- Resolution scaling (v1.1.2): s_ys = Q4.8 virtual lines per FIELD line
    -- (1080p 1.0, 720p 1.5, 480i 4.44 ...), s_cls 0 = 1080 lines, 1 = 720p,
    -- 2 = SD; both decoded free-running from the timing ID.  s_yfr carries
    -- the virtual-line fraction down the field, s_rep = virtual lines the
    -- next line advances (step-0 repeats), s_alv the virtual line count.
    signal s_ys           : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_cls          : unsigned(1 downto 0)  := (others => '0');
    signal s_yfr          : unsigned(7 downto 0)  := (others => '0');
    signal s_rep          : unsigned(2 downto 0)  := to_unsigned(1, 3);
    signal s_alv          : unsigned(19 downto 0) := (others => '0');
    signal s_q3, s_qk, s_q21 : signed(13 downto 0) := (others => '0');  -- 720p Steps
    signal s_width_m      : unsigned(C_AW - 1 downto 0) := to_unsigned(1920, C_AW);

    --------------------------------------------------------------------------
    -- Per-field constants (latched by the frame sequencer)
    --------------------------------------------------------------------------
    signal s_W        : unsigned(C_AW - 1 downto 0) := to_unsigned(1920, C_AW);
    signal s_H        : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_R        : unsigned(C_AW - 1 downto 0) := to_unsigned(1920 - 1 - C_EDGE, C_AW);
    signal s_S        : unsigned(C_AW - 1 downto 0) := to_unsigned(1920 - 2 * C_EDGE, C_AW);
    signal s_mod      : unsigned(C_AW downto 0) := to_unsigned(1920 - 2 * C_EDGE, C_AW + 1);
    signal s_C1       : unsigned(C_AW downto 0) := to_unsigned(2 * (1920 - 2 * C_EDGE) + C_EDGE - 1, C_AW + 1);
    signal s_mirror_f : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-field step sequencer (vertical blanking)
    --------------------------------------------------------------------------
    signal s_vstep       : unsigned(4 downto 0) := C_V_IDLE;
    signal s_fm_a, s_fm_b : unsigned(10 downto 0) := (others => '0');
    signal s_fm_pl       : unsigned(16 downto 0) := (others => '0');
    signal s_fm_ph       : unsigned(15 downto 0) := (others => '0');
    signal s_fm_p        : unsigned(21 downto 0) := (others => '0');
    signal s_g_per       : unsigned(15 downto 0) := to_unsigned(340 * 64, 16);
    signal s_g_tw        : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_gdiff       : signed(16 downto 0) := (others => '0');
    signal s_d_rate      : signed(10 downto 0) := (others => '0');
    signal s_absd_rate   : unsigned(9 downto 0) := (others => '0');
    signal s_rate        : signed(13 downto 0) := (others => '0');
    signal s_d_tw        : signed(10 downto 0) := (others => '0');
    signal s_absd_tw     : unsigned(9 downto 0) := (others => '0');
    signal s_tw_neg      : std_logic := '0';
    signal s_tw_mag      : unsigned(10 downto 0) := (others => '0');
    signal s_twist       : unsigned(23 downto 0) := (others => '0');
    signal s_freq0       : unsigned(23 downto 0) := (others => '0');
    signal s_pivot       : unsigned(21 downto 0) := (others => '0');
    signal s_freq_top    : unsigned(23 downto 0) := (others => '0');
    signal s_swell_phase : unsigned(15 downto 0) := (others => '0');
    signal s_swell_tri   : unsigned(9 downto 0) := (others => '0');
    signal s_aq          : unsigned(9 downto 0) := (others => '0');
    signal s_amp_frac    : unsigned(9 downto 0) := (others => '0');
    signal s_eff         : unsigned(9 downto 0) := (others => '0');
    signal s_rip_q       : unsigned(9 downto 0) := (others => '0');
    signal s_split_q     : unsigned(9 downto 0) := (others => '0');
    signal s_amp_px      : unsigned(10 downto 0) := (others => '0');
    signal s_rip_px      : unsigned(7 downto 0) := (others => '0');
    signal s_frame_phase : unsigned(23 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-line step sequencer (horizontal blanking). Phase and freq are
    -- 16.8 fixed point; the phase's top 16 bits are the wave phase.
    --------------------------------------------------------------------------
    signal s_lstep      : unsigned(5 downto 0) := C_L_IDLE;
    signal s_phase      : unsigned(23 downto 0) := (others => '0');
    signal s_freq       : unsigned(23 downto 0) := (others => '0');
    signal s_phase3     : unsigned(15 downto 0) := (others => '0');
    signal s_rom_addr   : unsigned(7 downto 0) := (others => '0');
    signal s_rom_q      : signed(9 downto 0) := (others => '0');
    signal s_sin_main   : signed(9 downto 0) := (others => '0');
    signal s_tri_main   : signed(9 downto 0) := (others => '0');
    signal s_sin_rip    : signed(9 downto 0) := (others => '0');
    signal s_tri_rip    : signed(9 downto 0) := (others => '0');
    signal s_wave_main  : signed(9 downto 0) := (others => '0');
    signal s_wave_rip   : signed(9 downto 0) := (others => '0');
    signal s_lm_a, s_lm_b : unsigned(10 downto 0) := (others => '0');
    signal s_lm_pl      : unsigned(16 downto 0) := (others => '0');
    signal s_lm_ph      : unsigned(15 downto 0) := (others => '0');
    signal s_lm_p       : unsigned(21 downto 0) := (others => '0');
    signal s_neg_main   : std_logic := '0';
    signal s_neg_rip    : std_logic := '0';
    signal s_neg_off    : std_logic := '0';
    signal s_off_m      : signed(12 downto 0) := (others => '0');
    signal s_off_r      : signed(12 downto 0) := (others => '0');
    signal s_off        : signed(12 downto 0) := (others => '0');
    signal s_dsplit     : signed(12 downto 0) := (others => '0');
    signal s_off_u      : signed(13 downto 0) := (others => '0');
    signal s_off_v      : signed(13 downto 0) := (others => '0');
    signal s_red        : signed(13 downto 0) := (others => '0');
    signal s_ge         : std_logic := '0';

    -- Run-along read pointers (= RAM read addresses) and directions
    signal s_ptr_y, s_ptr_u, s_ptr_v : unsigned(C_AW - 1 downto 0) := to_unsigned(C_EDGE, C_AW);
    signal s_dir_y, s_dir_u, s_dir_v : std_logic := '0';

    --------------------------------------------------------------------------
    -- Control (decoded from registers)
    --------------------------------------------------------------------------
    signal s_period_k : unsigned(C_DW - 1 downto 0);
    signal s_rate_k   : unsigned(C_DW - 1 downto 0);
    signal s_ripple_k : unsigned(C_DW - 1 downto 0);
    signal s_twist_k  : unsigned(C_DW - 1 downto 0);
    signal s_split_k  : unsigned(C_DW - 1 downto 0);
    signal s_mix_k    : unsigned(C_DW - 1 downto 0);
    signal s_amount   : unsigned(C_DW - 1 downto 0);
    signal s_shape    : std_logic;
    signal s_swell    : std_logic;
    signal s_steps    : std_logic;
    signal s_wrap     : std_logic;
    signal s_bypass   : std_logic;
    signal s_mix_w    : unsigned(C_DW - 1 downto 0) := (others => '0');  -- 1023 - Mix

    --------------------------------------------------------------------------
    -- Pixel path
    --------------------------------------------------------------------------
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_g_dry_y : unsigned(C_DW - 1 downto 0) := C_Y_BLACK;
    signal s_g_dry_u : unsigned(C_DW - 1 downto 0) := C_C_ZERO;
    signal s_g_dry_v : unsigned(C_DW - 1 downto 0) := C_C_ZERO;
    signal s_g_wet_y : unsigned(C_DW - 1 downto 0) := C_Y_BLACK;
    signal s_g_wet_u : unsigned(C_DW - 1 downto 0) := C_C_ZERO;
    signal s_g_wet_v : unsigned(C_DW - 1 downto 0) := C_C_ZERO;

    --------------------------------------------------------------------------
    -- Dry / sync delay shift registers
    --------------------------------------------------------------------------
    type t_data_shift is array (0 to C_TOTAL_LATENCY - 1)
        of std_logic_vector(C_DW - 1 downto 0);
    type t_bit_shift  is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_y_sr     : t_data_shift := (others => (others => '0'));
    signal s_u_sr     : t_data_shift := (others => (others => '0'));
    signal s_v_sr     : t_data_shift := (others => (others => '0'));
    signal s_hsync_sr : t_bit_shift  := (others => '1');
    signal s_vsync_sr : t_bit_shift  := (others => '1');
    signal s_field_sr : t_bit_shift  := (others => '1');
    signal s_avid_sr  : t_bit_shift  := (others => '0');

    --------------------------------------------------------------------------
    -- Dry/wet mix pipeline (E3-E6)
    --------------------------------------------------------------------------
    type t_yuv is record
        y, u, v : unsigned(C_DW - 1 downto 0);
    end record;
    constant C_YUV_BLACK : t_yuv := (y => C_Y_BLACK, u => C_C_ZERO, v => C_C_ZERO);

    signal s_mm10           : t_yuv := C_YUV_BLACK;              -- |dry - wet|
    signal s_mn10, s_mn11, s_mn12 : std_logic_vector(2 downto 0) := (others => '0');
    signal s_mwet10, s_mwet11, s_mwet12 : t_yuv := C_YUV_BLACK;
    signal s_mdry10, s_mdry11, s_mdry12 : t_yuv := C_YUV_BLACK;
    signal s_mpa11_y, s_mpb11_y : unsigned(14 downto 0) := (others => '0');
    signal s_mpa11_u, s_mpb11_u : unsigned(14 downto 0) := (others => '0');
    signal s_mpa11_v, s_mpb11_v : unsigned(14 downto 0) := (others => '0');
    signal s_mpm12_y, s_mpm12_u, s_mpm12_v : unsigned(19 downto 0) := (others => '0');
    signal s_mix13          : t_yuv := C_YUV_BLACK;

begin

    --------------------------------------------------------------------------
    -- Control decode
    --------------------------------------------------------------------------
    s_period_k <= unsigned(registers_in(0));
    s_rate_k   <= unsigned(registers_in(1));
    s_ripple_k <= unsigned(registers_in(2));
    s_twist_k  <= unsigned(registers_in(3));
    s_split_k  <= unsigned(registers_in(4));
    s_mix_k    <= unsigned(registers_in(5));
    s_shape    <= registers_in(6)(0);
    s_swell    <= registers_in(6)(1);
    s_steps    <= registers_in(6)(2);
    s_wrap     <= registers_in(6)(3);
    s_bypass   <= registers_in(6)(4);
    s_amount   <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Sine ROM: one registered read port shared by both lookups (the line
    -- sequencer re-registers s_rom_q in fabric before using it).
    --------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_rom_q <= to_signed(C_SINE(to_integer(s_rom_addr)), 10);
        end if;
    end process p_rom;

    --------------------------------------------------------------------------
    -- E0: input registers, write column, read pointers, line/field events,
    -- and both step sequencers. Single driver for every pointer and phase.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_n      : unsigned(C_AW downto 0);
        variable v_ph     : unsigned(15 downto 0);
        variable v_abs    : signed(10 downto 0);
        variable v_abs13  : signed(12 downto 0);
        variable v_off    : signed(12 downto 0);
        variable v_hi8    : signed(12 downto 0);
        variable v_hi10   : signed(12 downto 0);
        variable v_add    : signed(13 downto 0);
        variable v_sub    : signed(13 downto 0);
        variable v_subs   : signed(13 downto 0);
        variable v_fold   : signed(13 downto 0);
        variable v_lin    : signed(13 downto 0);
        variable v_g      : signed(16 downto 0);
        variable v_mag    : unsigned(13 downto 0);
        variable v_rate24 : signed(23 downto 0);
        variable v_yn     : unsigned(11 downto 0);
        variable v_av     : unsigned(19 downto 0);
        variable v_k      : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y    <= data_in.y;
            s_in_u    <= data_in.u;
            s_in_v    <= data_in.v;
            s_in_avid <= data_in.avid;

            s_prev_vsync_n <= data_in.vsync_n;
            if data_in.avid = '1' then
                s_saw_active <= '1';
            end if;

            s_mix_w <= not s_mix_k;

            -- resolution-scaling steps (timing ID static while a mode runs)
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_cls <= "10";
                when 5 | 6 | 14     => s_cls <= "01";
                when others         => s_cls <= "00";
            end case;

            -- Write column: pixel k of the line lands at address k.
            if s_in_avid = '1' then
                s_wr_x <= s_wr_x + 1;
            else
                s_wr_x <= (others => '0');
            end if;

            -- Run-along read pointers: one step per active pixel.
            if data_in.avid = '1' then
                v_n := f_bounce(s_ptr_y, s_dir_y, s_R, s_mirror_f);
                s_ptr_y <= v_n(C_AW - 1 downto 0);
                s_dir_y <= v_n(C_AW);
                v_n := f_bounce(s_ptr_u, s_dir_u, s_R, s_mirror_f);
                s_ptr_u <= v_n(C_AW - 1 downto 0);
                s_dir_u <= v_n(C_AW);
                v_n := f_bounce(s_ptr_v, s_dir_v, s_R, s_mirror_f);
                s_ptr_v <= v_n(C_AW - 1 downto 0);
                s_dir_v <= v_n(C_AW);
            end if;

            -- Shared multipliers, operand b split 6/5 hi/lo (an unsplit
            -- 11x11 in one clock was the HD HDMI critical path): operands
            -- registered by the sequencers, product readable THREE steps
            -- after the operands are set.
            s_lm_pl <= s_lm_a * s_lm_b(5 downto 0);
            s_lm_ph <= s_lm_a * s_lm_b(10 downto 6);
            s_lm_p  <= shift_left(resize(s_lm_ph, 22), 6) + resize(s_lm_pl, 22);
            s_fm_pl <= s_fm_a * s_fm_b(5 downto 0);
            s_fm_ph <= s_fm_a * s_fm_b(10 downto 6);
            s_fm_p  <= shift_left(resize(s_fm_ph, 22), 6) + resize(s_fm_pl, 22);

            -- Edge-fold arithmetic, hoisted so all three channels share
            -- one instance: s_red holds (start column - L), reduced into
            -- [0, mod) with mod = S (Wrap) or 2S (Mirror).
            v_add  := s_red + signed(resize(s_mod, 14));
            v_sub  := s_red - signed(resize(s_mod, 14));
            v_subs := s_red - signed(resize(s_S, 14));
            v_fold := signed(resize(s_C1, 14)) - s_red;     -- 2S + L - 1 - red
            v_lin  := s_red + to_signed(C_EDGE, 14);         -- L + red
            v_hi8  := signed(resize(s_lm_p(21 downto 8), 13));
            v_hi10 := signed(resize(s_lm_p(21 downto 10), 13));

            ------------------------------------------------------------------
            -- Per-line sequencer: one registered step per clock, kicked at
            -- the active-video fall (step 0) or by the frame sequencer for
            -- the first line (step 1). 34 steps, well inside SD's 138-clock
            -- horizontal blanking.
            ------------------------------------------------------------------
            if s_lstep /= C_L_IDLE then
                s_lstep <= s_lstep + 1;
            end if;

            case to_integer(s_lstep) is
                when 0 =>       -- advance to this line: once per VIRTUAL
                                -- line it covers (s_rep, 1 at 1080p)
                    s_phase <= s_phase + s_freq;
                    s_freq  <= s_freq + s_twist;
                    if s_rep > 1 then
                        s_rep   <= s_rep - 1;
                        s_lstep <= (others => '0');
                    end if;
                when 1 =>       -- main lookup address, 3x phase, triangle
                    v_ph       := s_phase(23 downto 8);
                    s_rom_addr <= v_ph(15 downto 8);
                    s_phase3   <= v_ph + shift_left(v_ph, 1);
                    s_tri_main <= f_tri(v_ph);
                when 2 =>       -- ripple lookup address, triangle
                    s_rom_addr <= s_phase3(15 downto 8);
                    s_tri_rip  <= f_tri(s_phase3);
                when 3 =>
                    s_sin_main <= s_rom_q;
                when 4 =>
                    s_sin_rip <= s_rom_q;
                    if s_shape = '1' then
                        s_wave_main <= s_sin_main;
                    else
                        s_wave_main <= s_tri_main;
                    end if;
                when 5 =>       -- main product: |wave| * amp_px
                    if s_shape = '1' then
                        s_wave_rip <= s_sin_rip;
                    else
                        s_wave_rip <= s_tri_rip;
                    end if;
                    v_abs      := abs(resize(s_wave_main, 11));
                    s_lm_a     <= unsigned(v_abs);
                    s_lm_b     <= s_amp_px;
                    s_neg_main <= s_wave_main(9);
                when 6 =>       -- ripple product: |wave3| * rip_px
                    v_abs     := abs(resize(s_wave_rip, 11));
                    s_lm_a    <= unsigned(v_abs);
                    s_lm_b    <= resize(s_rip_px, 11);
                    s_neg_rip <= s_wave_rip(9);
                when 8 =>       -- main product ready (set in step 5)
                    if s_neg_main = '1' then
                        s_off_m <= -v_hi8;
                    else
                        s_off_m <= v_hi8;
                    end if;
                when 9 =>       -- ripple product ready (set in step 6)
                    if s_neg_rip = '1' then
                        s_off_r <= -v_hi8;
                    else
                        s_off_r <= v_hi8;
                    end if;
                when 10 =>      -- combine, optional quantize to 16 virtual
                                -- units: 1080p 16 px, SD 8 px; 720p (10.67
                                -- px, not a power of two) in steps 11-13
                    v_off := s_off_m + s_off_r;
                    if s_steps = '1' then
                        case s_cls is
                            when "00"   => v_off(3 downto 0) := (others => '0');
                            when "10"   => v_off(2 downto 0) := (others => '0');
                            when others => null;
                        end case;
                    end if;
                    s_off <= v_off;
                when 11 =>      -- split product: |offset| * split
                    v_abs13   := abs(s_off);
                    s_lm_a    <= unsigned(v_abs13(10 downto 0));
                    s_lm_b    <= resize(s_split_q, 11);
                    s_neg_off <= s_off(12);
                    s_q3      <= resize(s_off, 14) + shift_left(resize(s_off, 14), 1);
                when 12 =>      -- 720p Steps: k = floor(off * 3 / 32)
                    v_k   := shift_right(s_q3, 5);
                    s_qk  <= v_k;
                    s_q21 <= shift_left(v_k, 4) + shift_left(v_k, 2);
                when 13 =>      -- 720p Steps: off = k * 10.5 (split keeps the
                                -- unquantized |offset|: a sub-step difference)
                    if s_steps = '1' and s_cls = "01" then
                        s_off <= resize(shift_right(s_q21 + s_qk, 1), 13);
                    end if;
                when 14 =>
                    if s_neg_off = '1' then
                        s_dsplit <= -v_hi10;
                    else
                        s_dsplit <= v_hi10;
                    end if;
                when 15 =>
                    s_off_u <= resize(s_off, 14) + resize(s_dsplit, 14);
                    s_off_v <= resize(s_off, 14) - resize(s_dsplit, 14);
                -- Fold each channel's start column into [L, R]: load,
                -- three reduce steps, mirror-half test, pointer load.
                when 16 =>
                    s_red <= resize(s_off, 14) - to_signed(C_EDGE, 14);
                when 22 =>
                    s_red <= s_off_u - to_signed(C_EDGE, 14);
                when 28 =>
                    s_red <= s_off_v - to_signed(C_EDGE, 14);
                when 17 | 18 | 19 | 23 | 24 | 25 | 29 | 30 | 31 =>
                    if s_red(13) = '1' then
                        s_red <= v_add;
                    elsif v_sub(13) = '0' then
                        s_red <= v_sub;
                    end if;
                when 20 | 26 | 32 =>
                    s_ge <= s_mirror_f and not v_subs(13);
                when 21 =>
                    if s_ge = '1' then
                        s_ptr_y <= unsigned(v_fold(C_AW - 1 downto 0));
                        s_dir_y <= '1';
                    else
                        s_ptr_y <= unsigned(v_lin(C_AW - 1 downto 0));
                        s_dir_y <= '0';
                    end if;
                when 27 =>
                    if s_ge = '1' then
                        s_ptr_u <= unsigned(v_fold(C_AW - 1 downto 0));
                        s_dir_u <= '1';
                    else
                        s_ptr_u <= unsigned(v_lin(C_AW - 1 downto 0));
                        s_dir_u <= '0';
                    end if;
                when C_L_LAST =>
                    if s_ge = '1' then
                        s_ptr_v <= unsigned(v_fold(C_AW - 1 downto 0));
                        s_dir_v <= '1';
                    else
                        s_ptr_v <= unsigned(v_lin(C_AW - 1 downto 0));
                        s_dir_v <= '0';
                    end if;
                    s_lstep <= C_L_IDLE;
                when others =>  -- 7 (multiplier wait) and idle
                    null;
            end case;

            ------------------------------------------------------------------
            -- Per-field sequencer (vertical blanking): knob curves, glides,
            -- width-relative scaling on one shared multiplier, then the
            -- frame phase, then kick the line sequencer for the first line.
            ------------------------------------------------------------------
            if s_vstep /= C_V_IDLE then
                s_vstep <= s_vstep + 1;
            end if;

            case to_integer(s_vstep) is
                when 0 =>
                    s_d_rate      <= signed('0' & s_rate_k) - to_signed(512, 11);
                    s_swell_phase <= s_swell_phase + to_unsigned(C_SWELL_RATE, 16);
                    s_W           <= s_width_m;
                    s_H           <= s_height_m;   -- virtual lines (~1080)
                    s_mirror_f    <= not s_wrap;
                    s_gdiff       <= signed('0' & s_period_k & "000000") - signed('0' & s_g_per);
                when 1 =>
                    v_g     := signed('0' & s_g_per) + shift_right(s_gdiff, 3);
                    s_g_per <= unsigned(v_g(15 downto 0));
                    v_abs   := abs(s_d_rate);
                    s_absd_rate <= unsigned(v_abs(9 downto 0));
                    if s_swell_phase(15) = '0' then
                        s_swell_tri <= s_swell_phase(14 downto 5);
                    else
                        s_swell_tri <= to_unsigned(1023, 10) - s_swell_phase(14 downto 5);
                    end if;
                    s_R <= s_W - to_unsigned(C_EDGE + 1, C_AW);
                    s_S <= s_W - to_unsigned(2 * C_EDGE, C_AW);
                when 2 =>       -- mul: Rate^2
                    s_gdiff <= signed('0' & s_twist_k & "000000") - signed('0' & s_g_tw);
                    s_fm_a  <= resize(s_absd_rate, 11);
                    s_fm_b  <= resize(s_absd_rate, 11);
                    if s_mirror_f = '1' then
                        s_mod <= shift_left(resize(s_S, C_AW + 1), 1);
                    else
                        s_mod <= resize(s_S, C_AW + 1);
                    end if;
                    s_C1 <= shift_left(resize(s_S, C_AW + 1), 1) + to_unsigned(C_EDGE - 1, C_AW + 1);
                when 3 =>       -- mul: Period^2 (10.1)
                    v_g    := signed('0' & s_g_tw) + shift_right(s_gdiff, 3);
                    s_g_tw <= unsigned(v_g(15 downto 0));
                    s_fm_a <= s_g_per(15 downto 5);
                    s_fm_b <= s_g_per(15 downto 5);
                when 4 =>       -- mul: Amount^2
                    s_d_tw <= signed('0' & s_g_tw(15 downto 6)) - to_signed(512, 11);
                    s_fm_a <= resize(s_amount, 11);
                    s_fm_b <= resize(s_amount, 11);
                when 5 =>       -- rate = sign * (|K2-512|^2 >> 6), detent; mul: Ripple^2
                    v_mag := s_fm_p(19 downto 6);
                    if s_absd_rate < to_unsigned(C_DETENT, 10) then
                        s_rate <= (others => '0');
                    elsif s_d_rate(10) = '1' then
                        s_rate <= -signed(v_mag);
                    else
                        s_rate <= signed(v_mag);
                    end if;
                    v_abs     := abs(s_d_tw);
                    s_absd_tw <= unsigned(v_abs(9 downto 0));
                    s_tw_neg  <= s_d_tw(10);
                    s_fm_a    <= resize(s_ripple_k, 11);
                    s_fm_b    <= resize(s_ripple_k, 11);
                when 6 =>       -- freq0 = Period^2 >> 8, in 16.8; mul: Twist^2
                    s_freq0 <= resize(s_fm_p(21 downto 2), 24);
                    s_fm_a  <= resize(s_absd_tw, 11);
                    s_fm_b  <= resize(s_absd_tw, 11);
                when 7 =>       -- mul: Split^2
                    s_aq   <= s_fm_p(19 downto 10);           -- Amount^2
                    s_fm_a <= resize(s_split_k, 11);
                    s_fm_b <= resize(s_split_k, 11);
                when 8 =>
                    s_amp_frac <= to_unsigned(C_AMP_FLOOR, 10) + s_aq - shift_right(s_aq, 5);
                    s_rip_q    <= s_fm_p(19 downto 10);       -- Ripple^2
                when 9 =>       -- twist magnitude (1/256 freq units per line); mul: swell
                    if s_absd_tw < to_unsigned(C_DETENT, 10) then
                        s_tw_mag <= (others => '0');
                    else
                        s_tw_mag <= s_fm_p(18 downto 8);
                    end if;
                    s_fm_a <= resize(s_amp_frac, 11);
                    s_fm_b <= resize(s_swell_tri, 11);
                when 10 =>      -- mul: Ripple px
                    s_split_q <= s_fm_p(19 downto 10);        -- Split^2
                    s_fm_a    <= resize(s_rip_q, 11);
                    s_fm_b    <= s_W;
                when 11 =>      -- mul: twist pivot
                    s_fm_a <= resize(s_H(10 downto 1), 11);   -- H / 2
                    s_fm_b <= s_tw_mag;
                when 12 =>
                    if s_swell = '1' then
                        s_eff <= s_fm_p(19 downto 10);
                    else
                        s_eff <= s_amp_frac;
                    end if;
                when 13 =>      -- mul: Amount px
                    s_rip_px <= s_fm_p(20 downto 13);         -- rip * W / 8
                    s_fm_a   <= resize(s_eff, 11);
                    s_fm_b   <= s_W;
                when 14 =>
                    s_pivot <= s_fm_p;                        -- H/2 * twist
                when 15 =>      -- freq at the top line = freq0 - H/2 * twist
                    if s_tw_neg = '1' then
                        s_freq_top <= s_freq0 + resize(s_pivot, 24);
                        s_twist    <= to_unsigned(0, 24) - resize(s_tw_mag, 24);
                    else
                        s_freq_top <= s_freq0 - resize(s_pivot, 24);
                        s_twist    <= resize(s_tw_mag, 24);
                    end if;
                when 16 =>
                    s_amp_px <= s_fm_p(21 downto 11);         -- eff * W / 2
                when 17 =>
                    v_rate24      := shift_left(resize(s_rate, 24), 8);
                    s_frame_phase <= s_frame_phase + unsigned(v_rate24);
                when C_V_LAST =>
                    s_phase <= s_frame_phase;
                    s_freq  <= s_freq_top;
                    s_lstep <= to_unsigned(1, 6);
                    s_vstep <= C_V_IDLE;
                when others =>
                    null;
            end case;

            -- Active-video fall = end of a line (same edge as its last
            -- write, which still uses the old parity): flip parity, count
            -- the line, capture height (every line) and width (one line),
            -- kick the line sequencer for the next line.
            if s_in_avid = '1' and data_in.avid = '0' then
                s_par      <= not s_par;
                s_aline    <= s_aline + 1;
                -- virtual lines: the count (height for the Twist pivot) and
                -- the step-0 repeats for the next line (1 at 1080p)
                v_av       := s_alv + s_ys;
                s_alv      <= v_av;
                s_height_m <= v_av(18 downto 8);
                v_yn       := resize(s_yfr, 12) + s_ys;
                s_rep      <= v_yn(10 downto 8);
                s_yfr      <= v_yn(7 downto 0);
                if s_aline = to_unsigned(C_WIDTH_LINE, 11) then
                    s_width_m <= s_wr_x + 1;
                end if;
                s_lstep <= (others => '0');
            end if;

            -- New field: vsync falling edge, guarded so serrated analog
            -- vsync fires once per field.
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw_active = '1' then
                s_saw_active <= '0';
                s_aline      <= (others => '0');
                s_alv        <= (others => '0');
                s_yfr        <= (others => '0');
                s_vstep      <= (others => '0');
            end if;

            -- Dry / sync shift registers
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank (single write port). Write = the
    -- E0-registered pixel into this line's parity bank; read = the
    -- registered pointer, registered out (valid with s_y_sr(0)).
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ptr_y));
            if s_in_avid = '1' and s_par = '0' then
                lbY0(to_integer(s_wr_x)) <= s_in_y;
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ptr_y));
            if s_in_avid = '1' and s_par = '1' then
                lbY1(to_integer(s_wr_x)) <= s_in_y;
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_ptr_u(C_AW - 1 downto 1)));
            if s_in_avid = '1' and s_par = '0' then
                lbU0(to_integer(s_wr_x(C_AW - 1 downto 1))) <= s_in_u;
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_ptr_u(C_AW - 1 downto 1)));
            if s_in_avid = '1' and s_par = '1' then
                lbU1(to_integer(s_wr_x(C_AW - 1 downto 1))) <= s_in_u;
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_ptr_v(C_AW - 1 downto 1)));
            if s_in_avid = '1' and s_par = '0' then
                lbV0(to_integer(s_wr_x(C_AW - 1 downto 1))) <= s_in_v;
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_ptr_v(C_AW - 1 downto 1)));
            if s_in_avid = '1' and s_par = '1' then
                lbV1(to_integer(s_wr_x(C_AW - 1 downto 1))) <= s_in_v;
            end if;
        end if;
    end process p_lbV1;

    --------------------------------------------------------------------------
    -- E1: bank select (the bank NOT being written = the line above).
    --------------------------------------------------------------------------
    p_wet : process(clk)
    begin
        if rising_edge(clk) then
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                s_wet_u <= unsigned(s_rdU1);
                s_wet_v <= unsigned(s_rdV1);
            else
                s_wet_y <= unsigned(s_rdY0);
                s_wet_u <= unsigned(s_rdU0);
                s_wet_v <= unsigned(s_rdV0);
            end if;
        end if;
    end process p_wet;

    -- Read-bank flag captured with the bank reads (E0).
    p_rbank : process(clk)
    begin
        if rising_edge(clk) then
            s_a_rbank <= not s_par;
        end if;
    end process p_rbank;

    --------------------------------------------------------------------------
    -- E2: blanking gate. Outside active video both mix inputs are black /
    -- neutral chroma, so the mixed output is too (bypass included).
    --------------------------------------------------------------------------
    p_gate : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(1) = '1' then
                s_g_dry_y <= unsigned(s_y_sr(1));
                s_g_dry_u <= unsigned(s_u_sr(1));
                s_g_dry_v <= unsigned(s_v_sr(1));
                s_g_wet_y <= s_wet_y;
                s_g_wet_u <= s_wet_u;
                s_g_wet_v <= s_wet_v;
            else
                s_g_dry_y <= C_Y_BLACK;
                s_g_dry_u <= C_C_ZERO;
                s_g_dry_v <= C_C_ZERO;
                s_g_wet_y <= C_Y_BLACK;
                s_g_wet_u <= C_C_ZERO;
                s_g_wet_v <= C_C_ZERO;
            end if;
        end if;
    end process p_gate;

    --------------------------------------------------------------------------
    -- E3-E6: dry/wet mix, out = wet +/- (|dry - wet| * (1023 - Mix)) >> 10,
    -- multiply split 10x5 hi/lo (shearline's shape: the SDK interpolator's
    -- unsplit 11x11 multiply was the HD critical path here too). Mix 100%
    -- is exactly wet; Bypass selects the gated dry at the same latency.
    --------------------------------------------------------------------------
    p_m10 : process(clk)
        variable v_dy, v_du, v_dv : unsigned(C_DW downto 0);
    begin
        if rising_edge(clk) then
            v_dy := f_sdiff(s_g_dry_y, s_g_wet_y);
            v_du := f_sdiff(s_g_dry_u, s_g_wet_u);
            v_dv := f_sdiff(s_g_dry_v, s_g_wet_v);
            s_mm10   <= (y => v_dy(C_DW - 1 downto 0), u => v_du(C_DW - 1 downto 0),
                         v => v_dv(C_DW - 1 downto 0));
            s_mn10   <= v_dv(C_DW) & v_du(C_DW) & v_dy(C_DW);
            s_mwet10 <= (y => s_g_wet_y, u => s_g_wet_u, v => s_g_wet_v);
            s_mdry10 <= (y => s_g_dry_y, u => s_g_dry_u, v => s_g_dry_v);
        end if;
    end process p_m10;

    p_m11 : process(clk)
    begin
        if rising_edge(clk) then
            s_mpa11_y <= s_mix_w * s_mm10.y(4 downto 0);
            s_mpb11_y <= s_mix_w * s_mm10.y(9 downto 5);
            s_mpa11_u <= s_mix_w * s_mm10.u(4 downto 0);
            s_mpb11_u <= s_mix_w * s_mm10.u(9 downto 5);
            s_mpa11_v <= s_mix_w * s_mm10.v(4 downto 0);
            s_mpb11_v <= s_mix_w * s_mm10.v(9 downto 5);
            s_mwet11  <= s_mwet10;
            s_mdry11  <= s_mdry10;
            s_mn11    <= s_mn10;
        end if;
    end process p_m11;

    p_m12 : process(clk)
    begin
        if rising_edge(clk) then
            s_mpm12_y <= shift_left(resize(s_mpb11_y, 20), 5) + resize(s_mpa11_y, 20);
            s_mpm12_u <= shift_left(resize(s_mpb11_u, 20), 5) + resize(s_mpa11_u, 20);
            s_mpm12_v <= shift_left(resize(s_mpb11_v, 20), 5) + resize(s_mpa11_v, 20);
            s_mwet12  <= s_mwet11;
            s_mdry12  <= s_mdry11;
            s_mn12    <= s_mn11;
        end if;
    end process p_m12;

    p_m13 : process(clk)
    begin
        if rising_edge(clk) then
            if s_bypass = '1' then
                s_mix13 <= s_mdry12;
            else
                s_mix13.y <= f_step(s_mwet12.y, s_mpm12_y, s_mn12(0));
                s_mix13.u <= f_step(s_mwet12.u, s_mpm12_u, s_mn12(1));
                s_mix13.v <= f_step(s_mwet12.v, s_mpm12_v, s_mn12(2));
            end if;
        end if;
    end process p_m13;

    --------------------------------------------------------------------------
    -- Output (latency 7 for syncs and video in every mode).
    --------------------------------------------------------------------------
    s_io_align_in.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    s_io_align_in.y <= std_logic_vector(s_mix13.y);
    s_io_align_in.u <= std_logic_vector(s_mix13.u);
    s_io_align_in.v <= std_logic_vector(s_mix13.v);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (7 -> 8).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture sidewinder;
