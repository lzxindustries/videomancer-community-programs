-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Dead Channel: a whole dying analog chain in one program.
--
-- Three cicero programs are folded into one signal path, in the order the
-- artifacts really happen:
--
--   TAPE  (from cicero/vhstrack) - wrap-around per-row horizontal skew,
--         random dropout bands that corrupt the picture (slips, block
--         pixelation, posterize, U/V swap, white dropout comets), tape hiss
--         and vertical chroma drift.
--   BEAM  (from cicero/chromaticab) - U/V read the line buffer from opposite
--         X positions, the split growing toward the screen edges.
--   TUBE  (from cicero/crt) - scanline or RGB-triad grille, corner vignette
--         and phosphor glow (horizontal IIR bleed on brights).  Darkening
--         desaturates with it, so dark corners and scanline gaps are black.
--
-- Differences from the three sources, deliberately:
--   * the skew is a wrapped DDA, not a clamped offset, so the picture folds
--     around the frame instead of smearing into the edge pixel at max;
--   * every control is live at every position of every other control (the
--     sources latched several knobs they never used);
--   * blanking gate, runtime-measured width/height, serration-guarded frame
--     anchor and interlace-correct per-line accumulators.
--
-- Register map:
--   registers_in(0) = Dropout  (rotary 1 - random glitch-band events: how
--                               often, how big, how long, how fast they
--                               scroll; 0 = no band at all)
--   registers_in(1) = Hiss     (rotary 2 - tape noise on luma)
--   registers_in(2) = Drift    (rotary 3 - scrolling chroma wobble, depth
--                               and rate together)
--   registers_in(3) = Converge (rotary 4, BIPOLAR - beam misconvergence,
--                               centre = converged, ends = spread / squeeze)
--   registers_in(4) = Glow     (rotary 5 - phosphor bleed trail)
--   registers_in(5) = Tube     (rotary 6 - grille depth + vignette, floored
--                               so the Grille switch always reads)
--   registers_in(6) = switches:
--     b0  Grille  (0 = Scan lines / 1 = RGB Triad)
--     b1  Cast    (0 = Cool       / 1 = Sick)
--     b2  Chroma  (0 = Rich       / 1 = Faded)
--     b3  Band    (0 = Tear - glitched video / 1 = Flash - glitched,
--                  bleached light bar)
--     b4  Invert  (0 = Off        / 1 = On - full video negative, luma
--                                  about 64..940 and chroma about 512)
--   registers_in(7) = Skew (slider, KEY, BIPOLAR - bounded tape pull: a flag
--                            at the top of the frame plus a head-switch tear,
--                            centre = straight, ends = a quarter-width bend)
--
-- Chroma note: this is a processing program, so the dry path is identity and
-- no U/V swap is applied.  The generated Cast tints are authored in HARDWARE
-- sense (data_out.u carries Cr / red-ness, data_out.v carries Cb / blue-ness).
--
-- BRAM: Y line buffer 10b x 2048 dual-bank (10 EBR) + U and V line buffers
-- 8b x 1024 dual-bank (4 EBR each) = 18 EBR.  Chroma is stored at half
-- horizontal resolution and 8 bits (steps of 4 codes, neutral 128 -> 512
-- exactly) purely to stay off the 30/32 EBR congestion cliff.
--
-- Pipeline depth (both the video data and the sync tap): 14 clocks, the same
-- in every mode.
--
-- v1.5.2 (2026-10-18) resolution scaling: the geometry runs in virtual
-- 1080-line units from the video timing ID (registers_in(8)), so SD and 720p
-- show the 1080p picture scaled (1080p output unchanged bit for bit):
--   * band position, size and roll, the skew decay and the drift wavelength
--     count virtual lines (480i 4.44 per field line, 720p 1.5);
--   * block pixelation 4..64 px is a SIZE in virtual units (720p 2/3, SD 1/2:
--     SD shifts the address mask, 720p holds the read address where 3 x the
--     source address crosses a multiple of 2 x size, so the blocks still sit
--     on the source and curve with the skew); comet length and rate follow;
--   * the vignette is measured on the screen-normalised raster (x 1.5 at
--     720p, 2.625 at SD, y in virtual lines), so it darkens the same edges
--     and corners at every resolution;
--   * the horizontal displacements that wrap at the frame width (skew pull,
--     per-line slip, chroma drift) are fractions of the width: 720p 2/3,
--     SD 3/8 (were 1/2 and 1/4);
--   * interlace comes from the timing ID: field_n is tied high on hardware,
--     so the old field_n-toggle detector never fired (SD skew decay and
--     vignette were 2x tall).
-- Scanlines, the RGB triad, hiss and the glow IIR stay at pixel scale.
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture deadchannel of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (14 + 2 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    -- 13 pipe taps; the output register makes the sync delay 14 clocks,
    -- identical in every mode.
    constant LATENCY : natural := 13;

    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ========================================================================
    -- Input registers / timing
    -- ========================================================================
    signal vs_r, hs_r, av_r                : std_logic := '0';
    signal prev_hsync_n                    : std_logic := '1';
    signal prev_vsync_n                    : std_logic := '1';
    signal prev_avid                       : std_logic := '0';

    signal px        : unsigned(11 downto 0) := (others => '0');
    signal aline     : unsigned(11 downto 0) := (others => '0');
    signal act_w     : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal saw_act   : std_logic := '0';  -- active video seen since anchor

    -- Resolution scaling (v1.5.2).  Steps decoded free-running from the
    -- timing ID (static while a mode runs).  s_xs: Q.1 virtual units per
    -- pixel (1080 2, 720p 3, SD 4); s_ys: Q4.8 virtual lines per FIELD line;
    -- s_vxs: Q.3 screen-normalised x step for the vignette (1080 8, 720p 12,
    -- SD 21 = 2.625); s_cls: 0 = 1080 lines, 1 = 720p, 2 = SD.
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_vxs    : unsigned(4 downto 0)  := to_unsigned(8, 5);
    signal s_cls    : unsigned(1 downto 0)  := (others => '0');
    signal alv      : unsigned(19 downto 0) := (others => '0'); -- virtual line Q12.8
    signal act_hv   : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal pxv      : unsigned(14 downto 0) := (others => '0'); -- vignette x Q12.3
    signal act_wv   : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal cxv_r    : unsigned(11 downto 0) := to_unsigned(960, 12);
    -- per-line decay sequencer: one exponential step per VIRTUAL line
    signal dcnt     : unsigned(2 downto 0) := (others => '0');
    signal d_first  : std_logic := '0';
    signal rp_pend  : std_logic := '0';
    signal rearm_r  : std_logic := '0';
    signal jog_r    : signed(11 downto 0) := (others => '0');
    signal jog_s    : signed(11 downto 0) := (others => '0');
    signal drift_sa : unsigned(6 downto 0) := (others => '0');
    -- 720p block pixelation: read-address hold.  A block is 2B halves of
    -- the source address x 3 (= x 1.5 virtual units), so its edges sit on
    -- the source like the 1080p mask's and follow the skew the same way.
    signal wrapx_r  : unsigned(11 downto 0) := (others => '0');
    signal hold_ln  : std_logic := '0';
    signal bmsk_ln  : unsigned(6 downto 0) := (others => '0'); -- 2B - 1
    signal r0_r     : unsigned(6 downto 0) := (others => '0'); -- 3 x row_eff, low bits

    -- ========================================================================
    -- Parameters (decoded free-running from the SPI registers every clock)
    -- ========================================================================
    signal hiss_r   : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal grille_r : std_logic := '0';
    signal cast_r   : std_logic := '0';
    signal faded_r  : std_logic := '1';
    signal bmode_r  : std_logic := '0';
    signal inv_r    : std_logic := '0';

    -- Derived, per frame
    signal conv_s    : signed(6 downto 0)  := (others => '0'); -- misconvergence
    signal drift_s   : unsigned(6 downto 0):= (others => '0');
    signal tube6     : unsigned(6 downto 0):= (others => '0'); -- vignette gain
    signal scan_amt  : unsigned(9 downto 0):= (others => '0'); -- grille depth
    signal triad_amt : unsigned(7 downto 0):= (others => '0');
    signal glow6     : unsigned(5 downto 0):= (others => '0');
    signal cx_r      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal cyv_r     : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- ========================================================================
    -- Tape transport state
    -- ========================================================================
    -- Skew profile.  NOT a cumulative shear: a cumulative DDA is unbounded by
    -- construction (even 1 px/line is a full screen width over 1080 lines and
    -- shreds the picture into diagonal slabs).  Instead the row offset is a
    -- BOUNDED position function - a decaying "flag" pull anchored at the top of
    -- the frame plus a second decaying pull anchored at the head-switch band -
    -- so the extremes of the slider bend the picture instead of destroying it.
    -- The whole slider -> amplitude -> decay-rate derivation runs FREE-RUNNING,
    -- one small op per cycle, and the frame anchor only samples the results.
    -- Doing it inside the anchor cone cost 13 MHz: an SPI read, a deadband, a
    -- variable shift and an abs all landed in one pixel-clock path.
    signal conv_d_r : signed(11 downto 0) := (others => '0');
    signal drop_r   : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal band_mul : unsigned(9 downto 0) := (others => '0');
    signal skew_d_r : signed(11 downto 0) := (others => '0');
    signal skew_z_r : signed(11 downto 0) := (others => '0');
    signal top_amp  : signed(11 downto 0) := (others => '0');  -- px, +-w/4
    -- Decay is EXPONENTIAL (bend - bend>>k), one subtract per line.  The
    -- linear "subtract a rate, clamp at zero" form chains two compares and an
    -- add into one cone and cost 6 MHz; the exponential form also happens to
    -- be the shape real tape flagging has.  The step is applied once per
    -- VIRTUAL line (v1.5.2), so the pull spans the same fraction of the
    -- frame in every mode.
    signal bend_seed: signed(15 downto 0) := (others => '0'); -- top_amp * 16
    signal bend2_sd : signed(15 downto 0) := (others => '0'); -- top_amp * 8
    signal row_pre  : signed(13 downto 0) := (others => '0'); -- offset pre-wrap
    signal bend1    : signed(15 downto 0) := (others => '0');  -- 1/16 px
    signal bend2    : signed(15 downto 0) := (others => '0');  -- 1/16 px
    signal row_eff  : unsigned(11 downto 0) := (others => '0'); -- + tear jog
    -- Band membership used to be |aline - band_pos| < half, which chains a
    -- subtract, an abs and two compares into ONE pixel-clock cone: 19 ns, the
    -- critical path of the whole design.  The four edges are registered once
    -- per frame instead, leaving a plain register-to-register compare per line.
    -- band_pos is signed and travels a little past both ends of the raster so
    -- the tear rolls off one edge and back on at the other.
    signal band_pos : signed(13 downto 0) := (others => '0');
    signal band_hlf : unsigned(11 downto 0) := to_unsigned(24, 12);

    -- Dropout scheduler (v1.4).  The band is an EVENT, not a smooth roll:
    -- on a random frame it appears at a random height and size, holds for
    -- 1..8 frames while scrolling, then vanishes until the next hit.  Every
    -- random draw is formed free-running and the frame anchor only samples
    -- it.  K1 sets the hit rate, the size, the hold and the scroll speed.
    signal glfsr     : unsigned(22 downto 0) := to_unsigned(5167321, 23);
    signal band_on   : std_logic := '0';
    signal band_hold : unsigned(2 downto 0) := (others => '0');
    signal band_up   : std_logic := '0';
    signal roll_amt  : signed(13 downto 0) := (others => '0');
    signal r_pos_mul : unsigned(19 downto 0) := (others => '0');
    signal r_pos     : unsigned(11 downto 0) := (others => '0');
    signal r_hlf_mul : unsigned(13 downto 0) := (others => '0');
    signal r_hlf     : unsigned(11 downto 0) := (others => '0');
    signal r_hit     : std_logic := '0';
    signal r_hold    : unsigned(2 downto 0) := (others => '0');
    -- K1 response curves (v1.5): exponential, so the lower half stays subtle
    -- and the top end is frequent and large.
    signal hit_thr   : unsigned(15 downto 0) := (others => '0');
    signal sz8       : unsigned(7 downto 0) := (others => '0');
    signal sz_mul    : unsigned(19 downto 0) := (others => '0');
    signal jsh       : unsigned(1 downto 0) := (others => '0');

    -- Per-line glitch recipe, drawn at hsync for band lines only.  Outside
    -- the band every field is its identity value, so the pixel path applies
    -- them unconditionally.
    signal blkm_ln  : unsigned(11 downto 0) := (others => '0'); -- addr block mask
    signal crush_ln : unsigned(9 downto 0) := (others => '1');  -- luma AND mask
    signal swap_ln  : std_logic := '0';
    signal linv_ln  : std_logic := '0';
    signal stat_ln  : std_logic := '0';
    signal bt1, bb1 : signed(13 downto 0) := (others => '0');
    signal bt2, bb2 : signed(13 downto 0) := (others => '0');
    signal aline_s  : signed(13 downto 0) := (others => '0');
    signal in_band  : std_logic := '0';
    signal near_bnd : std_logic := '0';
    signal in_band_p: std_logic := '0';
    signal dyd_r    : signed(12 downto 0) := (others => '0');
    signal dy_r     : unsigned(11 downto 0) := (others => '0');
    signal dy_h     : unsigned(11 downto 0) := (others => '0');
    signal dy_a     : unsigned(11 downto 0) := (others => '0'); -- aspect-scaled
    signal scan_cnt : unsigned(1 downto 0) := (others => '0');  -- mod-3 line
    signal scan_par : std_logic := '0';
    -- Per-line flags for the pixel path, latched at hsync so the last pixels
    -- of a line (still in the pipe at active-video fall) keep their own line.
    signal scan_ln  : std_logic := '0';
    signal band_ln  : std_logic := '0';

    -- Chroma drift: free-running sine * depth, sampled once per line.
    signal drift_ph  : unsigned(13 downto 0) := (others => '0');
    signal sin_r     : signed(8 downto 0)  := (others => '0');
    signal drift_mul : signed(15 downto 0) := (others => '0');
    signal drift_off : signed(7 downto 0)  := (others => '0');

    signal lfsr_r    : unsigned(15 downto 0) := to_unsigned(38271, 16);

    -- ========================================================================
    -- Line buffers
    -- ========================================================================
    signal lb_ab      : std_logic := '0';
    signal lb_wr_y    : unsigned(10 downto 0);
    signal lb_wr_c    : unsigned(9 downto 0);
    signal lb_rd_y    : unsigned(10 downto 0);
    signal lb_rd_u    : unsigned(9 downto 0);
    signal lb_rd_v    : unsigned(9 downto 0);
    signal lb_y_out   : std_logic_vector(9 downto 0);
    signal lb_u_out   : std_logic_vector(7 downto 0);
    signal lb_v_out   : std_logic_vector(7 downto 0);
    signal c_in_u     : std_logic_vector(7 downto 0);
    signal c_in_v     : std_logic_vector(7 downto 0);

    -- 64-entry quarter-wave sine LUT, 0..127
    type t_quad is array (0 to 63) of unsigned(7 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 8), to_unsigned(  3, 8), to_unsigned(  6, 8), to_unsigned( 10, 8),
        to_unsigned( 13, 8), to_unsigned( 16, 8), to_unsigned( 19, 8), to_unsigned( 22, 8),
        to_unsigned( 25, 8), to_unsigned( 28, 8), to_unsigned( 31, 8), to_unsigned( 35, 8),
        to_unsigned( 38, 8), to_unsigned( 41, 8), to_unsigned( 44, 8), to_unsigned( 47, 8),
        to_unsigned( 49, 8), to_unsigned( 52, 8), to_unsigned( 55, 8), to_unsigned( 58, 8),
        to_unsigned( 61, 8), to_unsigned( 63, 8), to_unsigned( 66, 8), to_unsigned( 69, 8),
        to_unsigned( 71, 8), to_unsigned( 74, 8), to_unsigned( 76, 8), to_unsigned( 79, 8),
        to_unsigned( 81, 8), to_unsigned( 83, 8), to_unsigned( 86, 8), to_unsigned( 88, 8),
        to_unsigned( 90, 8), to_unsigned( 92, 8), to_unsigned( 94, 8), to_unsigned( 96, 8),
        to_unsigned( 98, 8), to_unsigned(100, 8), to_unsigned(102, 8), to_unsigned(104, 8),
        to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(109, 8), to_unsigned(110, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(116, 8),
        to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8),
        to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(125, 8), to_unsigned(126, 8),
        to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(127, 8), to_unsigned(127, 8));

    -- ========================================================================
    -- Pixel pipeline (depth in comments; pipe(N-1) is the depth-N sync tap)
    -- ========================================================================
    signal c1_x    : unsigned(11 downto 0);
    signal c1_dx   : signed(11 downto 0);
    signal c1_dxv  : signed(11 downto 0);   -- screen-normalised (vignette)
    signal c1_lf   : unsigned(15 downto 0);

    signal c2_nb   : std_logic;             -- 720p hold: line start
    signal c2_r    : unsigned(6 downto 0) := (others => '0'); -- 3 x address mod 2B
    signal c2_base : unsigned(12 downto 0);
    signal c2_cmul : signed(16 downto 0);
    signal c2_adx  : unsigned(11 downto 0);
    signal c2_x    : unsigned(11 downto 0);
    signal c2_lf   : unsigned(15 downto 0);

    signal c3_base : unsigned(11 downto 0);
    signal c3_coff : signed(8 downto 0);
    signal c3_mx   : unsigned(11 downto 0);
    signal c3_mn   : unsigned(11 downto 0);
    signal c3_x    : unsigned(11 downto 0);
    signal c3_lf   : unsigned(15 downto 0);

    signal c4_ya   : unsigned(10 downto 0);
    signal c4_ua   : unsigned(10 downto 0);
    signal c4_va   : unsigned(10 downto 0);
    signal c4_dist : unsigned(12 downto 0);
    signal c4_x    : unsigned(11 downto 0);
    signal c4_lf   : unsigned(15 downto 0);

    signal c5_vmul : unsigned(16 downto 0);
    signal c5_x    : unsigned(11 downto 0);
    signal c5_lf   : unsigned(15 downto 0);

    signal c6_y    : unsigned(9 downto 0);
    signal c6_u    : unsigned(9 downto 0);
    signal c6_v    : unsigned(9 downto 0);
    signal c6_vamt : unsigned(9 downto 0);
    signal c6_x    : unsigned(11 downto 0);
    signal c6_lf   : unsigned(15 downto 0);

    signal c7_y    : unsigned(9 downto 0);
    signal c7_u    : unsigned(9 downto 0);
    signal c7_v    : unsigned(9 downto 0);
    signal c7_vamt : unsigned(9 downto 0);
    signal c7_x    : unsigned(11 downto 0);
    signal c7_yf   : unsigned(9 downto 0);
    signal c7_lf   : unsigned(15 downto 0);
    signal c7_ct   : std_logic;             -- comet trigger (rate per mode)

    signal g_y     : unsigned(9 downto 0);
    signal g_u     : unsigned(9 downto 0);
    signal g_v     : unsigned(9 downto 0);
    signal g_vamt  : unsigned(9 downto 0);
    signal g_x     : unsigned(11 downto 0);
    -- comet length counter, Q.1 (twice the old streak); steps s_xs per pixel
    signal streak2 : unsigned(7 downto 0) := (others => '0');

    signal bleed_r : unsigned(9 downto 0) := (others => '0');
    signal c8_y    : unsigned(9 downto 0);
    signal c8_u    : unsigned(9 downto 0);
    signal c8_v    : unsigned(9 downto 0);
    signal c8_vamt : unsigned(9 downto 0);
    signal c8_x    : unsigned(11 downto 0);

    signal c9_gmul : unsigned(15 downto 0);
    signal c9_y    : unsigned(9 downto 0);
    signal c9_u    : unsigned(9 downto 0);
    signal c9_v    : unsigned(9 downto 0);
    signal c9_vamt : unsigned(9 downto 0);
    signal c9_x    : unsigned(11 downto 0);

    signal c10_y   : unsigned(9 downto 0);
    signal c10_u   : unsigned(9 downto 0);
    signal c10_v   : unsigned(9 downto 0);
    signal c10_dim : unsigned(10 downto 0);
    signal c10_x   : unsigned(11 downto 0);

    signal c11_y   : unsigned(9 downto 0);
    signal c11_u   : unsigned(9 downto 0);
    signal c11_v   : unsigned(9 downto 0);
    signal c11_dsh : unsigned(2 downto 0);  -- chroma fade shift, 0..4

    signal c12_y   : unsigned(9 downto 0);
    signal c12_u   : unsigned(9 downto 0);
    signal c12_v   : unsigned(9 downto 0);

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);
    signal s_io : t_video_stream_yuv444_30b;

    -- ========================================================================
    -- Helpers
    -- ========================================================================
    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        end if;
        -- v is proven in [0,1023] by the tests above; take the low 10 bits.
        -- resize(signed, 10) would keep the SIGN plus only 9 magnitude bits,
        -- folding every value >= 512 (512 -> 0 = saturated green).
        return resize(unsigned(std_logic_vector(v)), 10);
    end function;

    -- Luma clamp to the legal 64..940 range: full-scale Y clips all three RGB
    -- channels on hardware and kills the chroma with them.
    function lim_y(v : signed) return unsigned is
    begin
        if v < 64 then
            return to_unsigned(64, 10);
        elsif v > 940 then
            return to_unsigned(940, 10);
        end if;
        return resize(unsigned(std_logic_vector(v)), 10);
    end function;

    -- Resolution scaling: x step in half virtual units per pixel (1080 2,
    -- 720p 3, SD 4) and y step in Q4.8 virtual lines per FIELD line
    -- (doubled for interlace) - the pilot's (mondrian) tables.
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

    -- Fraction-of-width scale for displacements that wrap at the frame
    -- width: 1080 x1, 720p x0.656 (~2/3), SD x0.375 (720/1920).
    function f_fw(x : signed(11 downto 0); c : unsigned(1 downto 0)) return signed is
    begin
        case c is
            when "00"   => return x;
            when "01"   => return shift_right(x, 1) + shift_right(x, 3) +
                                  shift_right(x, 5);
            when others => return shift_right(x, 2) + shift_right(x, 3);
        end case;
    end function;

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q   : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(7 downto 0);
    begin
        v_q   := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        if v_q(0) = '0' then
            v_mag := C_SIN_QUAD(v_idx);
        else
            v_mag := C_SIN_QUAD(63 - v_idx);
        end if;
        if v_q(1) = '0' then
            return signed(resize(v_mag, 9));
        else
            return -signed(resize(v_mag, 9));
        end if;
    end function;

begin

    -- ========================================================================
    -- Line buffers: full-rate 10-bit Y, half-rate 8-bit U and V.
    -- ========================================================================
    c_in_u <= data_in.u(9 downto 2);
    c_in_v <= data_in.v(9 downto 2);

    lb_y_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(clk => clk, i_ab => lb_ab, i_wr_addr => lb_wr_y,
                 i_rd_addr => lb_rd_y, i_data => data_in.y, o_data => lb_y_out);

    lb_u_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 8, G_DEPTH => 10)
        port map(clk => clk, i_ab => lb_ab, i_wr_addr => lb_wr_c,
                 i_rd_addr => lb_rd_u, i_data => c_in_u, o_data => lb_u_out);

    lb_v_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 8, G_DEPTH => 10)
        port map(clk => clk, i_ab => lb_ab, i_wr_addr => lb_wr_c,
                 i_rd_addr => lb_rd_v, i_data => c_in_v, o_data => lb_v_out);

    lb_wr_y <= px(10 downto 0);
    lb_wr_c <= px(10 downto 1);
    lb_rd_y <= c4_ya;
    lb_rd_u <= c4_ua(10 downto 1);
    lb_rd_v <= c4_va(10 downto 1);

    -- ========================================================================
    -- Timing, measurement, per-frame and per-line state
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_b1n    : signed(15 downto 0);
        variable v_b2n    : signed(15 downto 0);
        variable v_jraw   : signed(11 downto 0);
        variable v_hlf    : signed(13 downto 0);
        variable v_gtap   : std_logic;
        variable v_jog    : signed(11 downto 0);
        variable v_eff    : signed(13 downto 0);
        variable v_ang    : unsigned(9 downto 0);
        variable v_tap    : std_logic;
        variable v_nl     : unsigned(11 downto 0);
        variable v_m      : unsigned(6 downto 0);
        variable v_lim    : unsigned(7 downto 0);
        variable v_dsa    : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            vs_r <= data_in.vsync_n;
            hs_r <= data_in.hsync_n;
            av_r <= data_in.avid;
            prev_hsync_n <= hs_r;
            prev_vsync_n <= vs_r;
            prev_avid    <= av_r;

            v_h_edge := '0';
            if hs_r = '0' and prev_hsync_n = '1' then
                v_h_edge := '1';
            end if;

            -- Free-running noise
            v_tap  := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;
            -- Second, independent generator for band / glitch decisions
            -- (x^23 + x^18 + 1, maximal length).
            v_gtap := glfsr(22) xor glfsr(17);
            glfsr  <= glfsr(21 downto 0) & v_gtap;

            -- Free-running chroma-drift sine and its depth multiply.  Both are
            -- registered every cycle and only SAMPLED at line start, so the
            -- multiply never sits in the per-line decision cone.
            -- (virtual lines: the wobble's wavelength is a fraction of the
            -- screen height in every mode; depth is a fraction of the width)
            v_ang     := alv(17 downto 8) + drift_ph(13 downto 4);
            sin_r     <= quad_sin(v_ang);
            drift_mul <= resize(sin_r * signed(resize(drift_sa, 8)), 16);

            -- Resolution-scaling steps, decoded free-running from the timing
            -- ID (static while a mode runs; 1080p = all identity).
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 =>
                    s_cls <= "10";
                    s_vxs <= to_unsigned(21, 5);
                when 5 | 6 | 14 =>
                    s_cls <= "01";
                    s_vxs <= to_unsigned(12, 5);
                when others =>
                    s_cls <= "00";
                    s_vxs <= to_unsigned(8, 5);
            end case;

            -- ===============================================================
            -- PARAMETER DECODE - free-running, every clock.
            --
            -- These were latched once per frame at the video anchor.  On
            -- hardware that anchor stopped re-firing and every knob and switch
            -- went dead while the picture still rendered, so nothing here is
            -- allowed to depend on a frame event any more: the registers are
            -- read continuously and pipelined one op per stage.  It is also
            -- what the house rule asks for - decode SPI registers into
            -- free-running values, never inside the vsync decision cone.
            -- ===============================================================
            hiss_r    <= unsigned(registers_in(1));
            grille_r  <= registers_in(6)(0);
            cast_r    <= registers_in(6)(1);
            faded_r   <= registers_in(6)(2);
            bmode_r   <= registers_in(6)(3);
            inv_r     <= registers_in(6)(4);
            drop_r    <= unsigned(registers_in(0));
            drift_s   <= unsigned(registers_in(2)(9 downto 3));
            -- drift depth as a fraction of the width (rate stays drift_s)
            v_dsa     := f_fw(signed(resize(drift_s, 12)), s_cls);
            drift_sa  <= unsigned(std_logic_vector(v_dsa(6 downto 0)));
            glow6     <= unsigned(registers_in(4)(9 downto 4));
            tube6     <= resize(unsigned(registers_in(5)(9 downto 4)), 7) +
                         to_unsigned(4, 7);
            scan_amt  <= resize(unsigned(registers_in(5)(9 downto 2)), 10) +
                         to_unsigned(24, 10);
            triad_amt <= resize(unsigned(registers_in(5)(9 downto 3)), 8) +
                         to_unsigned(12, 8);

            -- Misconvergence: bipolar with a dead band, split over two stages
            conv_d_r <= signed(resize(unsigned(registers_in(3)), 12)) -
                        to_signed(512, 12);
            if conv_d_r < 40 and conv_d_r > -40 then
                conv_s <= (others => '0');
            else
                conv_s <= resize(shift_right(conv_d_r, 3), 7);
            end if;

            -- Dropout: half-height of the head-switch band as a fraction of
            -- the raster.  Zero means NO band - the bar has to be defeatable.
            -- K1 response, exponential.  Per-frame hit chance (x/65536)
            -- = 2^(12k - 12): ~1 event per 30 s at 10%, ~1/s at 50%, ~7/s at
            -- 75%, every frame at 100%; none in the bottom 3%.  Band
            -- half-height = 0.12 * 10^(2k - 2) of the raster: a few lines
            -- up to 50% (~26-line band), ~190 lines on HD at 100%.
            case to_integer(drop_r(9 downto 5)) is
                when  0 => hit_thr <= to_unsigned(    0, 16);
                when  1 => hit_thr <= to_unsigned(   21, 16);
                when  2 => hit_thr <= to_unsigned(   27, 16);
                when  3 => hit_thr <= to_unsigned(   36, 16);
                when  4 => hit_thr <= to_unsigned(   47, 16);
                when  5 => hit_thr <= to_unsigned(   61, 16);
                when  6 => hit_thr <= to_unsigned(   80, 16);
                when  7 => hit_thr <= to_unsigned(  105, 16);
                when  8 => hit_thr <= to_unsigned(  137, 16);
                when  9 => hit_thr <= to_unsigned(  179, 16);
                when 10 => hit_thr <= to_unsigned(  234, 16);
                when 11 => hit_thr <= to_unsigned(  306, 16);
                when 12 => hit_thr <= to_unsigned(  400, 16);
                when 13 => hit_thr <= to_unsigned(  524, 16);
                when 14 => hit_thr <= to_unsigned(  685, 16);
                when 15 => hit_thr <= to_unsigned(  895, 16);
                when 16 => hit_thr <= to_unsigned( 1171, 16);
                when 17 => hit_thr <= to_unsigned( 1531, 16);
                when 18 => hit_thr <= to_unsigned( 2003, 16);
                when 19 => hit_thr <= to_unsigned( 2619, 16);
                when 20 => hit_thr <= to_unsigned( 3425, 16);
                when 21 => hit_thr <= to_unsigned( 4479, 16);
                when 22 => hit_thr <= to_unsigned( 5858, 16);
                when 23 => hit_thr <= to_unsigned( 7661, 16);
                when 24 => hit_thr <= to_unsigned(10018, 16);
                when 25 => hit_thr <= to_unsigned(13101, 16);
                when 26 => hit_thr <= to_unsigned(17133, 16);
                when 27 => hit_thr <= to_unsigned(22406, 16);
                when 28 => hit_thr <= to_unsigned(29302, 16);
                when 29 => hit_thr <= to_unsigned(38320, 16);
                when 30 => hit_thr <= to_unsigned(50113, 16);
                when others => hit_thr <= to_unsigned(65535, 16);
            end case;
            case to_integer(drop_r(9 downto 6)) is
                when  0 => sz8 <= to_unsigned(  2, 8);
                when  1 => sz8 <= to_unsigned(  3, 8);
                when  2 => sz8 <= to_unsigned(  5, 8);
                when  3 => sz8 <= to_unsigned(  6, 8);
                when  4 => sz8 <= to_unsigned(  8, 8);
                when  5 => sz8 <= to_unsigned( 11, 8);
                when  6 => sz8 <= to_unsigned( 16, 8);
                when  7 => sz8 <= to_unsigned( 21, 8);
                when  8 => sz8 <= to_unsigned( 29, 8);
                when  9 => sz8 <= to_unsigned( 39, 8);
                when 10 => sz8 <= to_unsigned( 53, 8);
                when 11 => sz8 <= to_unsigned( 72, 8);
                when 12 => sz8 <= to_unsigned( 98, 8);
                when 13 => sz8 <= to_unsigned(133, 8);
                when 14 => sz8 <= to_unsigned(181, 8);
                when others => sz8 <= to_unsigned(246, 8);
            end case;
            sz_mul   <= act_hv * sz8;
            band_mul <= resize(sz_mul(19 downto 11), 10);

            -- Per-line slip shift: a knob term, so the slip is +-32 px below
            -- 25%, +-64 below 50% (HD; jog_s scales it to the raster width).
            if drop_r(9 downto 8) = "00" then
                jsh <= to_unsigned(2, 2);
            elsif drop_r(9) = '0' then
                jsh <= to_unsigned(1, 2);
            else
                jsh <= to_unsigned(0, 2);
            end if;
            jog_s <= f_fw(jog_r, s_cls);

            -- Dropout event draws, free-running, sampled by the anchor:
            --   position  anywhere in the raster (rand8 * height / 256)
            --   half-size 1/2 .. 15/16 of the K1 maximum, never below 2 lines
            --   hit       per-frame chance from the exponential table above
            --   hold      1..2 frames below half travel, 1..4 to 3/4, 1..8 above
            --   scroll    2..32 lines per frame, direction drawn per event
            r_pos_mul <= lfsr_r(15 downto 8) * act_hv;
            r_pos     <= r_pos_mul(19 downto 8);
            r_hlf_mul <= band_mul * ('1' & glfsr(14 downto 12));
            -- (virtual lines; at SD a band is at least 3 units of half
            -- height so its 480i field-line samples, 4.44 units apart,
            -- never step over it)
            if s_cls = "10" and r_hlf_mul(13 downto 4) = 0 then
                r_hlf <= to_unsigned(3, 12);
            else
                r_hlf <= resize(r_hlf_mul(13 downto 4), 12) + to_unsigned(2, 12);
            end if;
            if glfsr(22 downto 7) < hit_thr then
                r_hit <= '1';
            else
                r_hit <= '0';
            end if;
            if drop_r(9) = '0' then
                r_hold <= "00" & glfsr(0);
            elsif drop_r(8) = '0' then
                r_hold <= '0' & glfsr(1 downto 0);
            else
                r_hold <= glfsr(2 downto 0);
            end if;
            roll_amt <= signed(resize(drop_r(9 downto 6) & '0', 14)) +
                        to_signed(2, 14);

            -- Free-running skew derivation (one op per stage, never in a
            -- decision cone).  Settles in five clocks - instant next to a frame.
            skew_d_r <= signed(resize(unsigned(registers_in(7)), 12)) -
                        to_signed(512, 12);
            if skew_d_r < 40 and skew_d_r > -40 then
                skew_z_r <= (others => '0');
            else
                skew_z_r <= skew_d_r;
            end if;
            -- amplitude: a quarter of the WIDTH in every raster
            top_amp   <= f_fw(skew_z_r, s_cls);
            bend_seed <= shift_left(resize(top_amp, 16), 4);
            bend2_sd  <= shift_left(resize(top_amp, 16), 3);

            -- Free-running band edges and line geometry.  Everything here is
            -- register-in / register-out and settles long before active video.
            -- The band lives in virtual lines (position, size and roll).
            aline_s <= signed(resize(alv(19 downto 8), 14));
            v_hlf   := signed(resize(band_hlf, 14));
            bt1 <= band_pos - v_hlf;
            bb1 <= band_pos + v_hlf;
            bt2 <= band_pos - shift_left(v_hlf, 1);
            bb2 <= band_pos + shift_left(v_hlf, 1);
            if aline_s >= bt1 and aline_s < bb1 and band_on = '1' then
                in_band <= '1';
            else
                in_band <= '0';
            end if;
            if aline_s >= bt2 and aline_s < bb2 and band_on = '1' then
                near_bnd <= '1';
            else
                near_bnd <= '0';
            end if;
            -- Scanline grille: at 1080 lines a 1-px on/off alternation reads
            -- as flat dimming, so the HD raster darkens one line in three
            -- (2 on / 1 off).  720p and SD keep the classic alternation.
            -- (pixel-scale texture: stays per real line)
            if s_cls = "00" then
                if scan_cnt = 2 then
                    scan_par <= '1';
                else
                    scan_par <= '0';
                end if;
            else
                scan_par <= aline(0);
            end if;

            -- Vertical distance from centre for the vignette, scaled by the
            -- 16:9 width/height (x1.75) so the falloff follows the screen
            -- shape.  v1.5.2: measured in virtual lines against the
            -- screen-normalised x (pxv), so every raster - interlaced or
            -- not - gets the 1080p vignette fitted to its own screen.
            dyd_r <= signed(resize(alv(19 downto 8), 13)) - signed(resize(cyv_r, 13));
            if dyd_r < 0 then
                dy_r <= unsigned(std_logic_vector(resize(-dyd_r, 12)));
            else
                dy_r <= unsigned(std_logic_vector(resize(dyd_r, 12)));
            end if;
            dy_h <= shift_right(dy_r, 1) + shift_right(dy_r, 2);
            dy_a <= dy_r + dy_h;

            -- 720p block hold: the skewed read address wraps to 0 at this x,
            -- and starts the line at address row_eff (phase 3 x row_eff)
            wrapx_r <= act_w - row_eff;
            r0_r    <= row_eff(6 downto 0) + (row_eff(5 downto 0) & '0');

            -- ---------------------------------------------------------------
            -- Horizontal counter + measured active width
            -- ---------------------------------------------------------------
            if v_h_edge = '1' then
                px      <= (others => '0');
                pxv     <= (others => '0');
                lb_ab   <= not lb_ab;
                scan_ln <= scan_par;
                band_ln <= in_band;
                -- Glitch recipe for this line (identity outside the band):
                -- block pixelation 4..64 px on 5/8 of lines, luma posterize
                -- on half, U/V swap on half, a negative line 1 in 8 and a
                -- pure static line 1 in 8.
                if in_band = '1' then
                    -- Block size in virtual units (4..64 at 1080p): SD
                    -- halves the address mask; 720p (2/3, not a power of
                    -- two) holds the read address for 2*size halves of a
                    -- 3-halves-per-pixel run counter instead.
                    case to_integer(glfsr(2 downto 0)) is
                        when 3      => v_m := to_unsigned(3, 7);
                                       v_lim := to_unsigned(8, 8);
                        when 4      => v_m := to_unsigned(7, 7);
                                       v_lim := to_unsigned(16, 8);
                        when 5      => v_m := to_unsigned(15, 7);
                                       v_lim := to_unsigned(32, 8);
                        when 6      => v_m := to_unsigned(31, 7);
                                       v_lim := to_unsigned(64, 8);
                        when 7      => v_m := to_unsigned(63, 7);
                                       v_lim := to_unsigned(128, 8);
                        when others => v_m := (others => '0');
                                       v_lim := (others => '0');
                    end case;
                    bmsk_ln <= resize(v_lim - 1, 7);
                    case s_cls is
                        when "00" =>
                            blkm_ln <= resize(v_m, 12);
                            hold_ln <= '0';
                        when "01" =>
                            blkm_ln <= (others => '0');
                            if v_m = 0 then
                                hold_ln <= '0';
                            else
                                hold_ln <= '1';
                            end if;
                        when others =>
                            blkm_ln <= resize(shift_right(v_m, 1), 12);
                            hold_ln <= '0';
                    end case;
                    case glfsr(4 downto 3) is
                        when "10"   => crush_ln <= "1110000000";
                        when "11"   => crush_ln <= "1100000000";
                        when others => crush_ln <= (others => '1');
                    end case;
                    swap_ln <= glfsr(5);
                    if glfsr(8 downto 6) = "000" then
                        linv_ln <= '1';
                    else
                        linv_ln <= '0';
                    end if;
                    if glfsr(11 downto 9) = "000" then
                        stat_ln <= '1';
                    else
                        stat_ln <= '0';
                    end if;
                else
                    blkm_ln  <= (others => '0');
                    hold_ln  <= '0';
                    crush_ln <= (others => '1');
                    swap_ln  <= '0';
                    linv_ln  <= '0';
                    stat_ln  <= '0';
                end if;
            elsif av_r = '1' then
                px  <= px + 1;
                pxv <= pxv + s_vxs;
            end if;

            -- ---------------------------------------------------------------
            -- End of an active line: advance every per-line accumulator here
            -- (active lines only, so both interlaced fields stay in step).
            -- ---------------------------------------------------------------
            if prev_avid = '1' and av_r = '0' then
                if px > 100 then
                    act_w  <= px;
                    act_wv <= pxv(14 downto 3);
                end if;
                aline   <= aline + 1;
                alv     <= alv + s_ys;
                if scan_cnt = 2 then
                    scan_cnt <= (others => '0');
                else
                    scan_cnt <= scan_cnt + 1;
                end if;

                -- Both pulls decay once per ACTIVE line, so the two
                -- interlaced fields stay in step.
                -- Time constants: the frame flag spans roughly the top
                -- quarter of the picture and the head-switch tear about a
                -- sixteenth of it, in every raster: v1.5.2 steps the decay
                -- once per VIRTUAL line this field line covers (1080p 1,
                -- 720p 1-2, 480i 4-5), on the clocks after the line ends.
                v_nl  := resize(alv(7 downto 0), 12) + s_ys;
                dcnt  <= v_nl(10 downto 8);
                d_first <= '1';

                -- Raw tear jog for the next line, from the registered flags
                if in_band = '1' then
                    v_jraw := signed(resize(lfsr_r(7 downto 0), 12)) -
                              to_signed(128, 12);
                elsif near_bnd = '1' then
                    v_jraw := signed(resize(lfsr_r(11 downto 7), 12)) -
                              to_signed(16, 12);
                else
                    v_jraw := (others => '0');
                end if;

                -- The head-switch pull re-arms every time the band's top edge
                -- is crossed, then decays over ~64 lines below it.
                in_band_p <= in_band;
                if in_band = '1' and in_band_p = '0' then
                    rearm_r <= '1';
                else
                    rearm_r <= '0';
                end if;

                -- Row offset for the next line, formed after the decay steps
                -- (a whole line of slack) so hsync only has to wrap it.  The
                -- tear jog keys off the REGISTERED band flags - one line of
                -- lag, invisible.  jog_s scales it to the raster width.
                case to_integer(jsh) is
                    when 0      => v_jog := v_jraw;
                    when 1      => v_jog := shift_right(v_jraw, 1);
                    when others => v_jog := shift_right(v_jraw, 2);
                end case;
                jog_r <= v_jog;

                drift_off <= resize(shift_right(drift_mul, 9), 8);
            elsif dcnt /= 0 then
                -- one exponential decay step per virtual line; the first
                -- step re-arms the head-switch pull instead of decaying it
                v_b1n := bend1 - shift_right(bend1, 7);
                v_b2n := bend2 - shift_right(bend2, 5);
                if d_first = '1' and rearm_r = '1' then
                    v_b2n := bend2_sd;
                end if;
                bend1   <= v_b1n;
                bend2   <= v_b2n;
                d_first <= '0';
                dcnt    <= dcnt - 1;
                if dcnt = 1 then
                    rp_pend <= '1';
                end if;
            elsif rp_pend = '1' then
                rp_pend <= '0';
                row_pre <= resize(shift_right(bend1 + bend2, 4), 14) +
                           resize(jog_s, 14);
            end if;

            -- ---------------------------------------------------------------
            -- Tear jog folded into the row pointer, one line ahead of use.
            -- ---------------------------------------------------------------
            if v_h_edge = '1' then
                v_eff := row_pre;
                if v_eff >= signed(resize(act_w, 14)) then
                    v_eff := v_eff - signed(resize(act_w, 14));
                elsif v_eff < 0 then
                    v_eff := v_eff + signed(resize(act_w, 14));
                end if;
                if v_eff < 0 or v_eff >= signed(resize(act_w, 14)) then
                    row_eff <= (others => '0');
                else
                    row_eff <= unsigned(std_logic_vector(v_eff(11 downto 0)));
                end if;
            end if;

            -- ---------------------------------------------------------------
            -- Frame anchor: the first vsync EDGE after active video.
            --
            -- v1.1/v1.2 used the vsync LEVEL ("clear while vsync_n is low,
            -- fire on the next active pixel"), with the clear taking priority.
            -- If the hardware vsync reads low across active video that anchor
            -- never fires - and that matches the v1.1 failure exactly: every
            -- latched knob dead, aline free-running to its 4096 wrap so the
            -- tear band flashed and scrolled.  An edge fires once per frame
            -- whichever way the pulse sits, the saw-active guard makes
            -- serrated analog vsync fire once, and the line-count ceiling
            -- (no format has 1200 active lines) is now OUTSIDE any vsync
            -- condition so it backs up a vsync that never toggles at all.
            -- ---------------------------------------------------------------
            if saw_act = '1' and
               ((vs_r = '0' and prev_vsync_n = '1') or aline >= 1200) then
                saw_act <= '0';

                -- heights in virtual lines (~1080 in every mode)
                if aline > 50 then
                    act_hv <= alv(19 downto 8);
                    cyv_r  <= shift_right(alv(19 downto 8), 1);
                end if;
                aline    <= (others => '0');
                alv      <= (others => '0');
                scan_cnt <= (others => '0');
                cx_r     <= shift_right(act_w, 1);
                cxv_r    <= shift_right(act_wv, 1);

                -- Dropout events: a live band scrolls and counts down its
                -- hold; an idle one waits for a random hit, then jumps in at
                -- a random height and size.
                if band_on = '1' then
                    if band_hold = 0 then
                        band_on <= '0';
                    else
                        band_hold <= band_hold - 1;
                    end if;
                    if band_up = '1' then
                        band_pos <= band_pos - roll_amt;
                    else
                        band_pos <= band_pos + roll_amt;
                    end if;
                elsif r_hit = '1' then
                    band_on   <= '1';
                    band_pos  <= signed(resize(r_pos, 14));
                    band_hlf  <= r_hlf;
                    band_hold <= r_hold;
                    band_up   <= glfsr(5);
                end if;

                drift_ph <= drift_ph + resize(drift_s, 14) + to_unsigned(24, 14);

                -- The frame flag re-arms at the top of every frame; the
                -- amplitude and raster class are already resolved.
                bend1 <= bend_seed;

            elsif av_r = '1' and saw_act = '0' then
                saw_act  <= '1';
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx    : signed(11 downto 0);
        variable v_sum   : unsigned(12 downto 0);
        variable v_ca    : signed(13 downto 0);
        variable v_w13   : signed(13 downto 0);
        variable v_noise : signed(11 downto 0);
        variable v_yn    : signed(12 downto 0);
        variable v_bl    : unsigned(12 downto 0);
        variable v_g     : signed(12 downto 0);
        variable v_dim   : unsigned(10 downto 0);
        variable v_cu    : signed(11 downto 0);
        variable v_uin   : unsigned(9 downto 0);
        variable v_vin   : unsigned(9 downto 0);
        variable v_ds    : unsigned(2 downto 0);
        variable v_gy    : unsigned(9 downto 0);
        variable v_gu    : unsigned(9 downto 0);
        variable v_gv    : unsigned(9 downto 0);
        variable v_gt    : unsigned(9 downto 0);
        variable v_cv    : signed(11 downto 0);
        variable v_wb    : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- ---- depth 1: sample position, distance from centre, noise -----
            c1_x  <= px;
            c1_lf <= lfsr_r;
            v_dx  := signed(resize(px, 12)) - signed(resize(cx_r, 12));
            c1_dx <= v_dx;
            -- vignette x on the screen-normalised raster (= c1_dx at 1080p)
            c1_dxv <= signed(resize(pxv(14 downto 3), 12)) -
                      signed(resize(cxv_r, 12));

            -- ---- depth 2: skew sum, misconvergence multiply, vignette dist --
            c2_base <= resize(row_eff, 13) + resize(c1_x, 13);
            c2_cmul <= resize(c1_dx(11 downto 2) * conv_s, 17);
            -- 720p hold phase: 3 x (source address) mod 2B for this pixel
            if c1_x = 0 then
                c2_nb <= '1';
                c2_r  <= r0_r and bmsk_ln;
            elsif c1_x = wrapx_r then
                c2_nb <= '0';
                c2_r  <= (others => '0');
            else
                c2_nb <= '0';
                c2_r  <= (c2_r + 3) and bmsk_ln;
            end if;
            -- Vignette distance is max + min/2 (a near-circle, aspect-scaled
            -- through dy_a) instead of |dx|+|dy|, which drew a diamond.
            if c1_dxv < 0 then
                c2_adx <= unsigned(std_logic_vector(-c1_dxv));
            else
                c2_adx <= unsigned(std_logic_vector(c1_dxv));
            end if;
            c2_x  <= c1_x;
            c2_lf <= c1_lf;

            -- ---- depth 3: wrap the skewed X, form the chroma offset --------
            -- Wrap, then the band's block pixelation (read-address hold).
            -- 720p band lines hold the address, taking a new one where
            -- 3 x address crosses a multiple of 2B (c2_r < 3) and at the
            -- line start; elsewhere blkm_ln masks it (SD: mask halved).
            if c2_base >= resize(act_w, 13) then
                v_wb := resize(c2_base - resize(act_w, 13), 12);
            else
                v_wb := resize(c2_base, 12);
            end if;
            if hold_ln = '1' then
                if c2_nb = '1' or c2_r < 3 then
                    c3_base <= v_wb;
                end if;
            else
                c3_base <= v_wb and not blkm_ln;
            end if;
            c3_coff <= resize(shift_right(c2_cmul, 8), 9) + resize(drift_off, 9);
            if c2_adx >= dy_a then
                c3_mx <= c2_adx;
                c3_mn <= dy_a;
            else
                c3_mx <= dy_a;
                c3_mn <= c2_adx;
            end if;
            c3_x    <= c2_x;
            c3_lf   <= c2_lf;

            -- ---- depth 4: three read addresses ----------------------------
            c4_ya <= c3_base(10 downto 0);
            v_w13 := signed(resize(act_w, 14));
            v_ca  := signed(resize(c3_base, 14)) + resize(c3_coff, 14);
            if v_ca >= v_w13 then
                v_ca := v_ca - v_w13;
            elsif v_ca < 0 then
                v_ca := v_ca + v_w13;
            end if;
            c4_ua <= unsigned(std_logic_vector(v_ca(10 downto 0)));
            v_ca  := signed(resize(c3_base, 14)) - resize(c3_coff, 14);
            if v_ca >= v_w13 then
                v_ca := v_ca - v_w13;
            elsif v_ca < 0 then
                v_ca := v_ca + v_w13;
            end if;
            c4_va <= unsigned(std_logic_vector(v_ca(10 downto 0)));
            c4_dist <= resize(c3_mx, 13) + resize(shift_right(c3_mn, 1), 13);
            c4_x  <= c3_x;
            c4_lf <= c3_lf;

            -- ---- depth 5: line buffer read in flight ----------------------
            c5_vmul <= resize(c4_dist(12 downto 3) * tube6, 17);
            c5_x    <= c4_x;
            c5_lf   <= c4_lf;

            -- ---- depth 6: capture the line buffer outputs ------------------
            c6_y    <= unsigned(lb_y_out);
            c6_u    <= shift_left(resize(unsigned(lb_u_out), 10), 2);
            c6_v    <= shift_left(resize(unsigned(lb_v_out), 10), 2);
            if c5_vmul(16 downto 4) > to_unsigned(700, 13) then
                c6_vamt <= to_unsigned(700, 10);
            else
                c6_vamt <= c5_vmul(13 downto 4);
            end if;
            c6_x    <= c5_x;
            c6_lf   <= c5_lf;

            -- ---- depth 7: tape hiss -------------------------------------
            v_noise := signed(resize(c6_lf(8 downto 0), 12)) - to_signed(256, 12);
            case to_integer(hiss_r(9 downto 7)) is
                when 0      => v_noise := to_signed(0, 12);
                when 1      => v_noise := resize(shift_right(v_noise, 4), 12);
                when 2      => v_noise := resize(shift_right(v_noise, 3), 12);
                when 3      => v_noise := resize(shift_right(v_noise, 2), 12);
                when 4 | 5  => v_noise := resize(shift_right(v_noise, 1), 12);
                when others => null;
            end case;
            v_yn := signed(resize(c6_y, 13)) + resize(v_noise, 13);

            c7_y  <= sat10(v_yn);
            -- Flash variant (bleached light bar), formed in parallel
            c7_yf <= sat10(signed(resize(c6_y, 13)) + to_signed(360, 13) +
                           resize(shift_right(v_noise, 1), 13));
            c7_u    <= c6_u;
            c7_v    <= c6_v;
            c7_vamt <= c6_vamt;
            c7_x    <= c6_x;
            c7_lf   <= c6_lf;
            -- comet trigger: ~1 per 1024 px at 1080p, 1.5/1024 at 720p and
            -- 1/512 at SD, so the comets cover the same share of the band
            -- with their length scaled to the raster
            c7_ct <= '0';
            if c6_lf(15 downto 6) = "1111111111" then
                c7_ct <= '1';
            end if;
            if s_cls = "01" and c6_lf(15 downto 5) = "11111111100" then
                c7_ct <= '1';
            end if;
            if s_cls = "10" and c6_lf(15 downto 7) = "111111111" then
                c7_ct <= '1';
            end if;

            -- ---- depth 8: dropout band glitches ---------------------------
            -- The band corrupts the picture instead of replacing it: the
            -- per-line recipe (block hold, posterize, U/V swap, negative,
            -- static) plus white tape-dropout comets, ~1 per 1024 px, 8..71
            -- px long, bright at the head and fading along the tail.  Flash
            -- runs the same glitches on a bleached, colourless bar.
            v_gy := c7_y;
            v_gu := c7_u;
            v_gv := c7_v;
            if band_ln = '1' and bmode_r = '1' then
                v_gy := c7_yf;
                v_gu := C_MID;
                v_gv := C_MID;
            end if;
            if swap_ln = '1' then
                v_gt := v_gu;
                v_gu := v_gv;
                v_gv := v_gt;
            end if;
            v_gy := v_gy and crush_ln;
            if linv_ln = '1' then
                v_gy := not v_gy;
            end if;
            -- (streak2 = 2 x the 1080p count, stepping s_xs halves per pixel:
            -- comets are 8..71 px at 1080p, 2/3 at 720p, 1/2 at SD)
            if streak2(7 downto 1) /= 0 then
                if streak2 > s_xs then
                    streak2 <= streak2 - s_xs;
                else
                    streak2 <= (others => '0');
                end if;
            elsif band_ln = '1' and c7_ct = '1' then
                streak2 <= (resize(c7_lf(5 downto 0), 7) + to_unsigned(8, 7)) & '0';
            end if;
            if stat_ln = '1' and bmode_r = '0' then
                g_y <= resize(c7_lf(10 downto 2), 10) + to_unsigned(128, 10);
                g_u <= C_MID;
                g_v <= C_MID;
            elsif streak2(7 downto 1) /= 0 then
                g_y <= to_unsigned(640, 10) + shift_left(resize(streak2(7 downto 1), 10), 2);
                g_u <= C_MID;
                g_v <= C_MID;
            else
                g_y <= v_gy;
                g_u <= v_gu;
                g_v <= v_gv;
            end if;
            g_vamt <= c7_vamt;
            g_x    <= c7_x;

            -- ---- depth 9: phosphor bleed (one-pole IIR along the line) -----
            v_bl := resize(bleed_r, 13) - resize(shift_right(bleed_r, 2), 13) +
                    resize(shift_right(g_y, 2), 13);
            if g_x = 0 then
                bleed_r <= g_y;
            else
                bleed_r <= v_bl(9 downto 0);
            end if;
            c8_y    <= g_y;
            c8_u    <= g_u;
            c8_v    <= g_v;
            c8_vamt <= g_vamt;
            c8_x    <= g_x;

            -- ---- depth 10: glow multiply -----------------------------------
            c9_gmul <= resize(bleed_r * glow6, 16);
            c9_y    <= c8_y;
            c9_u    <= c8_u;
            c9_v    <= c8_v;
            c9_vamt <= c8_vamt;
            c9_x    <= c8_x;

            -- ---- depth 11: add the glow, gather the tube darkening --------
            v_g := signed(resize(c9_y, 13)) +
                   signed(resize(c9_gmul(15 downto 8), 13));
            c10_y <= sat10(v_g);
            c10_u <= c9_u;
            c10_v <= c9_v;
            if grille_r = '0' and scan_ln = '1' then
                v_dim := resize(c9_vamt, 11) + resize(scan_amt, 11);
            else
                v_dim := resize(c9_vamt, 11);
            end if;
            c10_dim <= v_dim;
            c10_x   <= c9_x;

            -- ---- depth 12: apply the darkening, RGB triad tint ------------
            c11_y <= lim_y(signed(resize(c10_y, 13)) -
                           signed(resize(c10_dim, 13)));
            -- Chroma follows the darkening: vignette corners and scanline gaps
            -- go dark, not dark-and-coloured.  One halving per 128 codes of
            -- dimming (max 3), one more for Faded.
            if c10_dim(10 downto 7) >= 3 then
                v_ds := to_unsigned(3, 3);
            else
                v_ds := resize(c10_dim(8 downto 7), 3);
            end if;
            if faded_r = '1' then
                v_ds := v_ds + 1;
            end if;
            c11_dsh <= v_ds;
            if grille_r = '1' then
                -- 4-column phosphor stripe: red, green, blue, gap.  Hardware
                -- sense: u carries Cr (red), v carries Cb (blue).
                case to_integer(c10_x(1 downto 0)) is
                    when 0 =>
                        c11_u <= sat10(signed(resize(c10_u, 12)) +
                                       signed(resize(triad_amt, 12)));
                        c11_v <= sat10(signed(resize(c10_v, 12)) -
                                       signed(resize(triad_amt, 12)));
                    when 1 =>
                        c11_u <= sat10(signed(resize(c10_u, 12)) -
                                       signed(resize(triad_amt, 12)));
                        c11_v <= sat10(signed(resize(c10_v, 12)) -
                                       signed(resize(triad_amt, 12)));
                    when 2 =>
                        c11_u <= sat10(signed(resize(c10_u, 12)) -
                                       signed(resize(triad_amt, 12)));
                        c11_v <= sat10(signed(resize(c10_v, 12)) +
                                       signed(resize(triad_amt, 12)));
                    when others =>
                        c11_u <= c10_u;
                        c11_v <= c10_v;
                end case;
            else
                c11_u <= c10_u;
                c11_v <= c10_v;
            end if;

            -- ---- depth 13: chroma cast, fade, luma invert -----------------
            -- Invert is a true video negative: chroma flips too.  1023 - c is
            -- a plain bitwise NOT, so it costs no adder.
            if inv_r = '1' then
                v_uin := not c11_u;
                v_vin := not c11_v;
            else
                v_uin := c11_u;
                v_vin := c11_v;
            end if;
            v_cu := signed(resize(v_uin, 12)) - to_signed(512, 12);
            v_cv := signed(resize(v_vin, 12)) - to_signed(512, 12);
            case to_integer(c11_dsh) is
                when 0 => null;
                when 1 =>
                    v_cu := shift_right(v_cu, 1);
                    v_cv := shift_right(v_cv, 1);
                when 2 =>
                    v_cu := shift_right(v_cu, 2);
                    v_cv := shift_right(v_cv, 2);
                when 3 =>
                    v_cu := shift_right(v_cu, 3);
                    v_cv := shift_right(v_cv, 3);
                when others =>
                    v_cu := shift_right(v_cu, 4);
                    v_cv := shift_right(v_cv, 4);
            end case;
            if cast_r = '0' then
                -- Cool: toward the blue axis (Cb up, Cr down)
                v_cu := v_cu - to_signed(40, 12);
                v_cv := v_cv + to_signed(72, 12);
            else
                -- Sick: toward sallow green (both below neutral)
                v_cu := v_cu - to_signed(56, 12);
                v_cv := v_cv - to_signed(64, 12);
            end if;
            c12_u <= sat10(v_cu + to_signed(512, 12));
            c12_v <= sat10(v_cv + to_signed(512, 12));
            -- Luma negative about the legal range: 64 <-> 940.
            if inv_r = '1' then
                c12_y <= to_unsigned(1004, 10) - c11_y;
            else
                c12_y <= c11_y;
            end if;

            -- ---- depth 14: blanking gate + sync tap (output register) -----
            if pipe(LATENCY - 1).avid = '0' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(c12_y);
                s_io.u <= std_logic_vector(c12_u);
                s_io.v <= std_logic_vector(c12_v);
            end if;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    s_io_align_in.y       <= s_io.y;
    s_io_align_in.u       <= s_io.u;
    s_io_align_in.v       <= s_io.v;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.field_n <= s_io.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (14 -> 16).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 1 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(1);

end architecture deadchannel;
