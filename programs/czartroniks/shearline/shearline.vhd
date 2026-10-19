-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Shearline: recursive line-feedback smear.
--
-- Each scanline re-draws the previous OUTPUT line, read at a sheared and
-- fanned column, blends the live line into it and writes the result back for
-- the next line. Because every line re-draws the already-moved line above
-- (recursive feedback, not a one-shot skew), bright detail streaks into
-- diagonal comet trails sliding down-screen.
--
-- Per active pixel x on the current line:
--   rd     = x -/+ S_n - D(x)                     column read in the line above
--   prev   = buffer[rd]                           (already Spectrum-rotated)
--   ref    = live (Smear)  |  black/neutral (Comet)
--   res    = ref + rtz( t * (prev - ref) / 1024 ) t = Trail weight
--   wet    = res  (Smear)  |  brighter of live / res (Comet)
--   buffer[x] = rotate(wet);   out = mix(live, wet, Mix)
--
-- Shear (P12): S_n is the integer carry of a per-line Bresenham accumulator
--   of a 4.4 fixed-point slope (px per 1080p line; 4.8 per field line after
--   resolution scaling), so fractional slopes (1/16 px/line) draw steep
--   trails as regular staircases. Slope Mod adds per-line random jitter.
-- Fan (K1): D(x) = k*(x - c), a per-pixel 7.14 fixed-point DDA about c = half
--   the measured width: each line re-draws the one above zoomed about the
--   centre, + sprays trails outward, - converges them. c*k comes free from a
--   second accumulator latched at the centre pixel. A golden-ratio per-line
--   dither on D's start keeps small fans moving (no rounding dead zone).
-- Spectrum (K4): a Minsky circle step on the stored chroma at write-back
--   (u -= a*v; v += a*u'), area-preserving so it never runs away, plus a
--   luma-scaled seed along +U so grey trails cycle through the hues too.
-- Rounding: the blend is round-toward-zero (t * |diff|, sign applied after),
--   so the recursion settles exactly on its target -- floor() biased long
--   trails toward green/black. The rotation rounds to nearest.
--
-- Buffer: dual-bank (ping-pong by line parity) EBR, one pair per channel.
--   Reads use the live parity (the line above); each pixel's write parity is
--   captured at read time and carried to the write, so trailing writes land
--   in the right bank with no boundary special-casing (see project memory:
--   lb_write_addr_timing). The read bank is never the write bank during
--   active video, so all six banks carry no_rw_check.
--   Y is full 2048-deep; U/V are 2:1 decimated (1024 deep, odd pixel wins).
--   The chroma read column is rd-1+b, b toggling on every odd-shift line, so
--   the chroma trail drifts at the luma rate instead of rounding to even.
-- Field start: the first active line of every field is seeded from live
--   video, so trails never wrap from the bottom of the frame to the top.
-- Off-line reads (rd < 0 or rd >= measured width) pass live video.
--
-- Resolution scaling (v1.1.2): every per-line quantity is defined per 1080p
--   line and converted to per FIELD line from the timing ID (virtual
--   1080-line units; ys = 1080p lines per field line, xs = units per pixel),
--   so trail angle, length, fan spray and hue cycling match 1080p as
--   fractions of the screen height at 720p and SD:
--     Shear and Slope Mod  x ys/xs  (px per field line; 720p 1.0, 480i 2.22)
--     Trail (1024 - t)     x ys     (Q.8, fraction carried line to line)
--     Fan k, Spectrum a    x ys     (zoom / rotation per field line)
--   One serial shift-add multiplier forms the five products. At 1080p both
--   steps are 1.0 and the output is bit-identical to v1.1.1.
--
-- Pipeline (data_in -> data_out = 14 clocks in every mode):
--   E1  input regs, counters, line/field bookkeeping
--   E2  read columns (registered -> EBR address)
--   E3  EBR read data, off-line flag
--   E4  bank select
--   E5  ref, |prev - ref|, sign, t
--   E6  t * |diff|, operand split hi/lo
--   E7  combine partial products
--   E8  res = ref +/- q
--   E9  Comet select -> wet / dry
--   E10-E13 Mix: wet + rtz((dry - wet) * (1023 - Mix)), split multiply
--           (exactly wet at 100%; Bypass selects dry at the same latency)
--   E14 blanking gate -> data_out
--   Write-back: E10-E15 Spectrum rotation, EBR write from the E15 registers.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture shearline of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (14 + 2 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_Y_AW          : integer := 11;                  -- 2048 (full)
    constant C_UV_AW         : integer := 10;                  -- 1024 (2:1 dec)
    constant C_RW            : integer := 13;                  -- signed read column
    constant C_FW            : integer := 21;                  -- fan accumulators, 7.14
    constant C_TOTAL_LATENCY : integer := 14;

    constant C_BLACK  : unsigned(C_DW - 1 downto 0) := to_unsigned(64, C_DW);
    constant C_NEUT   : unsigned(C_DW - 1 downto 0) := to_unsigned(512, C_DW);
    constant C_GOLDEN : unsigned(13 downto 0)       := to_unsigned(10125, 14);  -- 0.618 * 2^14

    type t_yuv is record
        y : unsigned(C_DW - 1 downto 0);
        u : unsigned(C_DW - 1 downto 0);
        v : unsigned(C_DW - 1 downto 0);
    end record;
    constant C_YUV_BLACK : t_yuv := (y => C_BLACK, u => C_NEUT, v => C_NEUT);

    -- Write-back context carried with every pixel from E2 to the EBR write.
    type t_ctx is record
        px  : unsigned(C_Y_AW - 1 downto 0);  -- write column
        par : std_logic;                       -- write bank
        we  : std_logic;                       -- active pixel
    end record;
    constant C_CTX_0 : t_ctx := (px => (others => '0'), par => '0', we => '0');

    type t_live_pipe is array (2 to 9) of t_yuv;
    type t_ctx_pipe  is array (2 to 15) of t_ctx;

    -- |a - b| in the low C_DW bits, sign (a < b) in the top bit.
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
    end function;

    -- ref +/- (pm >> 10). pm <= t*|prev-ref| with t < 1024, so the result
    -- always lies between ref and prev: no clamp needed.
    function f_step(b : unsigned(C_DW - 1 downto 0); pm : unsigned(19 downto 0);
                    neg : std_logic) return unsigned is
    begin
        if neg = '1' then
            return b - pm(19 downto 10);
        else
            return b + pm(19 downto 10);
        end if;
    end function;

    -- Resolution scaling: virtual 1080-line units per FIELD line, Q4.8
    -- (1080 / active lines, doubled for interlaced modes).
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
    -- Slope conversion, Q.8: y step / x step (virtual units per pixel: 1.0
    -- at 1080, 1.5 at 720p, 2.0 at SD). A slope of S px per 1080p line is
    -- S * r px per field line.
    function f_rstep(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(569, 10);   -- 480i: 4.445 / 2
            when 4          => return to_unsigned(288, 10);   -- 480p: 2.25 / 2
            when 8          => return to_unsigned(480, 10);   -- 576i: 3.75 / 2
            when 12         => return to_unsigned(240, 10);   -- 576p: 1.875 / 2
            when 1 | 2 | 10 => return to_unsigned(512, 10);   -- 1080i: 2 / 1
            when others     => return to_unsigned(256, 10);   -- 1080p, 720p
        end case;
    end function;

    -- Clamp to signed 10-bit chroma (-512..511).
    function f_clamp_c(x : signed) return signed is
    begin
        if x < -512 then
            return to_signed(-512, C_DW);
        elsif x > 511 then
            return to_signed(511, C_DW);
        else
            return resize(x, C_DW);
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Dual-bank line buffers (feedback store). Y black / chroma neutral at
    -- power-up: code 0 chroma is saturated green.
    --------------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(C_DW - 1 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(C_DW - 1 downto 0);

    signal lbY0, lbY1 : t_lb_y  := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_uv := (others => "1000000000");
    signal lbV0, lbV1 : t_lb_uv := (others => "1000000000");

    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    signal s_rdY0, s_rdY1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Control (registered decode; quasi-static)
    --------------------------------------------------------------------------
    signal s_kq       : signed(9 downto 0)          := (others => '0');  -- Fan, 2^-14 px/px
    signal s_g        : unsigned(7 downto 0)        := (others => '0');  -- 1024 - t (1080p)
    signal s_sh_off   : unsigned(7 downto 0)        := (others => '0');
    signal s_sh_inc   : unsigned(7 downto 0)        := (others => '0');
    signal s_shear    : unsigned(7 downto 0)        := (others => '0');  -- 4.4 px/line
    signal s_slope    : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_a_raw    : unsigned(7 downto 0)        := (others => '0');  -- Spectrum knob
    signal s_spec_a   : signed(11 downto 0)         := (others => '0');  -- 2^-10 rad/line
    signal s_seed_sel : unsigned(3 downto 0)        := (others => '0');

    -- Resolution scaling: every per-line quantity is defined per 1080p line
    -- and converted to per FIELD line by one serial shift-add multiplier
    -- (quasi-static, round-robin over five products). At 1080p both steps
    -- are 1.0 and every product equals its old value bit for bit.
    signal s_rq       : unsigned(9 downto 0)  := to_unsigned(256, 10);   -- Q.8 ys/xs
    signal s_ysq      : unsigned(11 downto 0) := to_unsigned(256, 12);   -- Q.8 ys
    signal s_mstep    : unsigned(3 downto 0)  := (others => '0');
    signal s_mslot    : unsigned(2 downto 0)  := (others => '0');
    signal s_ma       : unsigned(9 downto 0)  := (others => '0');
    signal s_mb       : unsigned(10 downto 0) := (others => '0');
    signal s_macc     : unsigned(20 downto 0) := (others => '0');
    signal s_mneg     : std_logic := '0';
    signal s_shear_e  : unsigned(13 downto 0) := (others => '0');  -- 4.8 px / field line
    signal s_slope_e  : unsigned(11 downto 0) := (others => '0');  -- Slope Mod * r
    signal s_g_e      : unsigned(18 downto 0) := (others => '0');  -- (1024 - t) * ys, Q.8
    signal s_k_e      : signed(12 downto 0)   := (others => '0');  -- Fan * ys
    signal s_a_e      : unsigned(10 downto 0) := (others => '0');  -- Spectrum * ys
    signal s_dir      : std_logic := '0';
    signal s_comet    : std_logic := '0';
    signal s_mix_w    : unsigned(C_DW - 1 downto 0) := (others => '0');  -- 1023 - Mix
    signal s_bypass   : std_logic := '0';

    --------------------------------------------------------------------------
    -- E1: input registers, counters, line / field bookkeeping
    --------------------------------------------------------------------------
    signal s_in           : t_yuv := C_YUV_BLACK;
    signal s_in_avid      : std_logic := '0';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';

    signal s_px     : unsigned(C_Y_AW - 1 downto 0) := (others => '0');  -- write column
    signal s_rp     : signed(C_RW - 1 downto 0)     := (others => '0');  -- x -/+ S
    signal s_rpu    : signed(C_RW - 1 downto 0)     := (others => '0');  -- x -/+ S - 1 + b
    signal s_dacc   : signed(C_FW - 1 downto 0)     := (others => '0');  -- k*(x - c) + g
    signal s_kacc   : signed(C_FW - 1 downto 0)     := (others => '0');  -- k*x
    signal s_kc     : signed(C_FW - 1 downto 0)     := (others => '0');  -- k*c
    signal s_par    : std_logic := '0';

    signal s_first    : std_logic := '1';  -- next active line is a field's first
    signal s_seedline : std_logic := '0';  -- this line is seeded from live
    signal s_afall    : std_logic := '0';
    signal s_saw_act  : std_logic := '0';
    signal s_wmax     : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_width    : unsigned(C_Y_AW - 1 downto 0) := to_unsigned(1920, C_Y_AW);
    signal s_half     : unsigned(C_Y_AW - 1 downto 0);

    signal s_phi      : unsigned(7 downto 0)  := (others => '0');  -- shear phase
    signal s_gold     : unsigned(13 downto 0) := (others => '0');  -- fan dither
    signal s_cbit     : std_logic := '0';                          -- chroma rounding toggle
    signal s_shcur    : unsigned(6 downto 0)  := (others => '0');  -- S for the next line
    signal s_tacc     : unsigned(7 downto 0)  := (others => '0');  -- trail-decay phase
    signal s_gcur     : unsigned(11 downto 0) := (others => '0');  -- 1024 - t, next line
    signal s_tline    : unsigned(C_DW - 1 downto 0) := (others => '0');  -- t, this line

    -- line-rate precompute (settles within the front porch)
    signal s_lfsr_q    : std_logic_vector(15 downto 0);
    signal s_line_rand : unsigned(4 downto 0)        := (others => '0');
    signal s_jit_p     : unsigned(16 downto 0)       := (others => '0');
    signal s_sum       : unsigned(14 downto 0)       := (others => '0');
    signal s_shnext    : unsigned(6 downto 0)        := (others => '0');
    signal s_gsum      : unsigned(8 downto 0)        := (others => '0');
    signal s_gnext     : unsigned(11 downto 0)       := (others => '0');
    signal s_tnext     : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_rp_load   : signed(C_RW - 1 downto 0)   := (others => '0');
    signal s_rpu_load  : signed(C_RW - 1 downto 0)   := (others => '0');
    signal s_dload     : signed(C_FW - 1 downto 0)   := (others => '0');

    --------------------------------------------------------------------------
    -- Blend pipeline
    --------------------------------------------------------------------------
    signal s_live : t_live_pipe := (others => C_YUV_BLACK);
    signal s_ctx  : t_ctx_pipe  := (others => C_CTX_0);

    signal s_rd_y, s_rd_uv : signed(C_RW - 1 downto 0) := (others => '0');   -- E2
    signal s_ra_uv         : unsigned(C_UV_AW - 1 downto 0);
    signal s_seed2, s_seed3 : std_logic := '0';
    signal s_bg3           : std_logic := '0';                               -- E3
    signal s_p4            : t_yuv := C_YUV_BLACK;                           -- E4
    signal s_tz4           : std_logic := '0';

    signal s_m5, s_b5      : t_yuv := C_YUV_BLACK;                           -- E5
    signal s_n5            : std_logic_vector(2 downto 0) := (others => '0');
    signal s_t5            : unsigned(C_DW - 1 downto 0) := (others => '0');

    signal s_pa6_y, s_pa6_u, s_pa6_v : unsigned(14 downto 0) := (others => '0');  -- E6
    signal s_pb6_y, s_pb6_u, s_pb6_v : unsigned(14 downto 0) := (others => '0');
    signal s_b6            : t_yuv := C_YUV_BLACK;
    signal s_n6            : std_logic_vector(2 downto 0) := (others => '0');

    signal s_pm7_y, s_pm7_u, s_pm7_v : unsigned(19 downto 0) := (others => '0');  -- E7
    signal s_b7            : t_yuv := C_YUV_BLACK;
    signal s_n7            : std_logic_vector(2 downto 0) := (others => '0');

    signal s_r8            : t_yuv := C_YUV_BLACK;                           -- E8
    signal s_wet9, s_dry9  : t_yuv := C_YUV_BLACK;                           -- E9

    --------------------------------------------------------------------------
    -- Spectrum write-back pipeline (E10-E15)
    --------------------------------------------------------------------------
    signal s_p1_10 : signed(19 downto 0)         := (others => '0');
    signal s_yb10  : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_cu10, s_cv10 : signed(C_DW - 1 downto 0) := (others => '0');
    signal s_y10   : unsigned(C_DW - 1 downto 0) := (others => '0');

    signal s_p1r11 : signed(11 downto 0)         := (others => '0');
    signal s_cus11 : signed(C_DW downto 0)       := (others => '0');
    signal s_cv11  : signed(C_DW - 1 downto 0)   := (others => '0');
    signal s_y11   : unsigned(C_DW - 1 downto 0) := (others => '0');

    signal s_u12, s_cv12  : signed(C_DW - 1 downto 0) := (others => '0');
    signal s_y12   : unsigned(C_DW - 1 downto 0) := (others => '0');

    signal s_p2_13 : signed(19 downto 0)         := (others => '0');
    signal s_u13, s_cv13  : signed(C_DW - 1 downto 0) := (others => '0');
    signal s_y13   : unsigned(C_DW - 1 downto 0) := (others => '0');

    signal s_p2r14 : signed(11 downto 0)         := (others => '0');
    signal s_u14, s_cv14  : signed(C_DW - 1 downto 0) := (others => '0');
    signal s_y14   : unsigned(C_DW - 1 downto 0) := (others => '0');

    signal s_wy15, s_wu15, s_wv15 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay shift registers (length = C_TOTAL_LATENCY)
    -- index 0 = 1 clock delayed, index N = (N+1) clocks delayed
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

    --------------------------------------------------------------------------
    -- Dry/wet mix pipeline (E10-E13) and output register
    --------------------------------------------------------------------------
    signal s_mm10, s_mwet10, s_mdry10 : t_yuv := C_YUV_BLACK;
    signal s_mn10  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_mpa11_y, s_mpa11_u, s_mpa11_v : unsigned(14 downto 0) := (others => '0');
    signal s_mpb11_y, s_mpb11_u, s_mpb11_v : unsigned(14 downto 0) := (others => '0');
    signal s_mwet11, s_mdry11 : t_yuv := C_YUV_BLACK;
    signal s_mn11  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_mpm12_y, s_mpm12_u, s_mpm12_v : unsigned(19 downto 0) := (others => '0');
    signal s_mwet12, s_mdry12 : t_yuv := C_YUV_BLACK;
    signal s_mn12  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_mix13 : t_yuv := C_YUV_BLACK;

    signal s_out : t_yuv := C_YUV_BLACK;

begin

    --------------------------------------------------------------------------
    -- Control decode (registered; two stages where a shift feeds an add)
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : signed(10 downto 0);
        variable v_p : unsigned(9 downto 0);
        variable v_e : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- K1 Fan: bipolar about noon with a +/-16 deadband, so the knob's
            -- noise can't wobble an "off" fan.
            v_k := signed(resize(unsigned(registers_in(0)), 11)) - 512;
            if v_k(10 downto 4) = "0000000" or v_k(10 downto 4) = "1111111" then
                s_kq <= (others => '0');
            else
                s_kq <= v_k(9 downto 0);
            end if;

            -- K2 Trail: exponential. t = 1024 - (('1' & ~m) >> e), e = top 3
            -- bits: trail length doubles every 1/8 turn, 4 lines -> 1000+.
            -- (t itself is formed per line from s_g * ys, below.)
            s_g <= shift_right('1' & not unsigned(registers_in(1)(6 downto 0)),
                               to_integer(unsigned(registers_in(1)(9 downto 7))));

            -- P12 Shear: piecewise slope (4.4 px/line), half the travel below
            -- 3 px/line where the steep angles live.
            v_p := unsigned(registers_in(7));
            case v_p(9 downto 8) is
                when "00"   => s_sh_off <= to_unsigned(0, 8);   s_sh_inc <= resize(v_p(7 downto 4), 8);
                when "01"   => s_sh_off <= to_unsigned(16, 8);  s_sh_inc <= resize(v_p(7 downto 3), 8);
                when "10"   => s_sh_off <= to_unsigned(48, 8);  s_sh_inc <= resize(v_p(7 downto 2), 8);
                when others => s_sh_off <= to_unsigned(112, 8); s_sh_inc <= resize(v_p(7 downto 1), 8);
            end case;
            s_shear <= s_sh_off + s_sh_inc;

            -- K4 Spectrum: rotation step a (0..0.25 rad per 1080p line; times
            -- ys per field line) and a luma seed of ~a/4096 per line (shift
            -- by a's octave) so grey trails tint.
            s_a_raw  <= unsigned(registers_in(3)(9 downto 2));
            v_e      := s_a_e;
            s_spec_a <= signed(resize(v_e, 12));
            if v_e(10) = '1' then   s_seed_sel <= to_unsigned(8, 4);
            elsif v_e(9) = '1' then s_seed_sel <= to_unsigned(7, 4);
            elsif v_e(8) = '1' then s_seed_sel <= to_unsigned(6, 4);
            elsif v_e(7) = '1' then s_seed_sel <= to_unsigned(5, 4);
            elsif v_e(6) = '1' then s_seed_sel <= to_unsigned(4, 4);
            elsif v_e(5) = '1' then s_seed_sel <= to_unsigned(3, 4);
            elsif v_e(4) = '1' then s_seed_sel <= to_unsigned(2, 4);
            elsif v_e(3) = '1' then s_seed_sel <= to_unsigned(1, 4);
            else                    s_seed_sel <= to_unsigned(0, 4);
            end if;

            s_slope <= unsigned(registers_in(2));
            s_dir   <= registers_in(6)(0);
            s_comet <= registers_in(6)(1);
            s_bypass <= registers_in(6)(4);
            s_mix_w  <= not unsigned(registers_in(4));
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Resolution scaling. The steps decode free-running from the timing ID
    -- (static while a mode runs). One serial shift-add multiplier (MSB
    -- first, 11 steps) cycles through the five per-line products:
    --   0 Shear * r -> 4.8 px/field line   1 Slope Mod * r
    --   2 (1024 - t) * ys, Q.8 (the fraction is dithered per line)
    --   3 |Fan| * ys (sign re-applied)     4 Spectrum * ys
    -- r = ys / xs converts a slope in px per 1080p line to px per field line;
    -- ys alone converts a per-line rate (decay, zoom, rotation).
    --------------------------------------------------------------------------
    p_scale : process(clk)
        variable v_km : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_rq  <= f_rstep(registers_in(8)(3 downto 0));
            s_ysq <= f_vstep_y(registers_in(8)(3 downto 0));

            s_mstep <= s_mstep + 1;
            if s_mstep = 0 then
                s_macc <= (others => '0');
                v_km   := abs(resize(s_kq, 11));
                case s_mslot is
                    when "000" =>
                        s_ma <= resize(s_shear, 10);
                        s_mb <= resize(s_rq, 11);
                    when "001" =>
                        s_ma <= s_slope;
                        s_mb <= resize(s_rq, 11);
                    when "010" =>
                        s_ma <= resize(s_g, 10);
                        s_mb <= s_ysq(10 downto 0);
                    when "011" =>
                        s_ma   <= unsigned(v_km(9 downto 0));
                        s_mb   <= s_ysq(10 downto 0);
                        s_mneg <= s_kq(9);
                    when others =>
                        s_ma <= resize(s_a_raw, 10);
                        s_mb <= s_ysq(10 downto 0);
                end case;
            elsif s_mstep <= 11 then
                if s_mb(10) = '1' then
                    s_macc <= shift_left(s_macc, 1) + resize(s_ma, 21);
                else
                    s_macc <= shift_left(s_macc, 1);
                end if;
                s_mb <= shift_left(s_mb, 1);
            elsif s_mstep = 12 then
                case s_mslot is
                    when "000" => s_shear_e <= s_macc(17 downto 4);
                    when "001" => s_slope_e <= s_macc(19 downto 8);
                    when "010" => s_g_e     <= s_macc(18 downto 0);
                    when "011" =>
                        if s_mneg = '1' then
                            s_k_e <= -signed('0' & s_macc(19 downto 8));
                        else
                            s_k_e <= signed('0' & s_macc(19 downto 8));
                        end if;
                    when others => s_a_e <= s_macc(18 downto 8);
                end case;
                if s_mslot = 4 then
                    s_mslot <= (others => '0');
                else
                    s_mslot <= s_mslot + 1;
                end if;
            end if;
        end if;
    end process p_scale;

    s_half <= '0' & s_width(C_Y_AW - 1 downto 1);

    --------------------------------------------------------------------------
    -- E1: input register, per-pixel counters, line and field bookkeeping.
    -- Counters advance on s_in_avid, so each holds the value for the pixel
    -- currently in s_in.
    --------------------------------------------------------------------------
    p_input : process(clk)
        variable v_hs_edge, v_vs_edge, v_av_rise, v_av_fall : boolean;
    begin
        if rising_edge(clk) then
            s_in.y    <= unsigned(data_in.y);
            s_in.u    <= unsigned(data_in.u);
            s_in.v    <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            v_hs_edge := data_in.hsync_n = '0' and s_prev_hsync_n = '1';
            v_vs_edge := data_in.vsync_n = '0' and s_prev_vsync_n = '1';
            v_av_rise := data_in.avid = '1' and s_in_avid = '0';
            v_av_fall := data_in.avid = '0' and s_in_avid = '1';

            -- hsync = new line: reload the read pointers and the fan DDA
            -- (values precomputed during the previous line), flip the bank.
            if v_hs_edge then
                s_px        <= (others => '0');
                s_rp        <= s_rp_load;
                s_rpu       <= s_rpu_load;
                s_dacc      <= s_dload;
                s_kacc      <= (others => '0');
                s_par       <= not s_par;
                s_line_rand <= unsigned(s_lfsr_q(4 downto 0));
                s_tline     <= s_tnext;
            elsif s_in_avid = '1' then
                s_px   <= s_px + 1;
                s_rp   <= s_rp + 1;
                s_rpu  <= s_rpu + 1;
                s_dacc <= s_dacc + resize(s_k_e, C_FW);
                s_kacc <= s_kacc + resize(s_k_e, C_FW);
                if s_px = s_half then
                    s_kc <= s_kacc;                  -- k*c for the next line
                end if;
            end if;

            if v_av_rise then
                s_seedline <= s_first;
            end if;

            -- End of an active line: advance the per-line DDAs (active lines
            -- only, so both interlace fields step identically).
            s_afall <= '0';
            if v_av_fall then
                s_afall <= '1';
                if s_seedline = '1' then
                    s_first <= '0';
                end if;
                s_phi   <= s_sum(7 downto 0);
                s_shcur <= s_shnext;
                if s_shnext(0) = '1' then
                    s_cbit <= not s_cbit;
                end if;
                s_gold  <= s_gold + C_GOLDEN;
                s_tacc  <= s_gsum(7 downto 0);
                s_gcur  <= s_gnext;
            end if;

            -- Line width: field maximum, one clock after the fall (s_px = W).
            if s_afall = '1' then
                s_saw_act <= '1';
                if s_px > s_wmax then
                    s_wmax <= s_px;
                end if;
            end if;

            -- vsync = new field. Resets are idempotent under serration; the
            -- width copy is guarded by saw-active so later edges don't zero it.
            if v_vs_edge then
                s_par   <= '0';
                s_first <= '1';
                s_phi   <= (others => '0');
                s_gold  <= (others => '0');
                s_tacc  <= (others => '0');
                if s_saw_act = '1' then
                    s_width   <= s_wmax;
                    s_wmax    <= (others => '0');
                    s_saw_act <= '0';
                end if;
            end if;

            -- Line-rate precompute for the next line's loads. Each term is one
            -- op from registers; all settle long before the next hsync.
            -- Shear: 4.8 Bresenham (the 1080p slope's fraction has 4 bits, so
            -- at 1080p the low 4 phase bits stay 0 and S_n is unchanged).
            s_sum    <= resize(s_shear_e, 15) + resize(s_phi, 15);
            s_jit_p  <= s_slope_e * s_line_rand;
            s_shnext <= resize(s_sum(14 downto 8), 7) + resize(s_jit_p(16 downto 10), 7);
            -- Trail: g = (1024 - t) * ys per field line; its Q.8 fraction
            -- is carried line to line so the mean decay is exact (1080p: the
            -- fraction is 0 and t = 1024 - g). Latched into s_tline at hsync.
            s_gsum  <= resize(s_g_e(7 downto 0), 9) + resize(s_tacc, 9);
            s_gnext <= resize(s_g_e(18 downto 8), 12) + resize(s_gsum(8 downto 8), 12);
            if s_gcur(11 downto 10) /= "00" then
                s_tnext <= (others => '0');          -- shorter than one line
            else
                s_tnext <= resize(to_unsigned(1024, 11) - resize(s_gcur, 11), C_DW);
            end if;
            if s_dir = '0' then
                s_rp_load <= -signed(resize(s_shcur, C_RW));  -- Right: read from the left
            else
                s_rp_load <= signed(resize(s_shcur, C_RW));   -- Left: read from the right
            end if;
            if s_cbit = '1' then
                s_rpu_load <= s_rp_load;
            else
                s_rpu_load <= s_rp_load - 1;
            end if;
            s_dload <= signed(resize(s_gold, C_FW)) - s_kc;

            -- Sync shift registers
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
    end process p_input;

    --------------------------------------------------------------------------
    -- Live video and write context, carried alongside the blend.
    --------------------------------------------------------------------------
    p_carry : process(clk)
    begin
        if rising_edge(clk) then
            s_live(2) <= s_in;
            for i in 3 to 9 loop
                s_live(i) <= s_live(i - 1);
            end loop;
            s_ctx(2) <= (px => s_px, par => s_par, we => s_in_avid);
            for i in 3 to 15 loop
                s_ctx(i) <= s_ctx(i - 1);
            end loop;
        end if;
    end process p_carry;

    --------------------------------------------------------------------------
    -- E2: read columns. Y reads rd; chroma reads pair (rd - 1 + b) >> 1.
    --------------------------------------------------------------------------
    p_e2 : process(clk)
        variable v_dint : signed(C_RW - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_dint  := resize(s_dacc(C_FW - 1 downto 14), C_RW);
            s_rd_y  <= s_rp - v_dint;
            s_rd_uv <= s_rpu - v_dint;
            s_seed2 <= s_seedline;
        end if;
    end process p_e2;

    s_ra_uv <= (others => '0') when s_rd_uv(C_RW - 1) = '1'
               else unsigned(s_rd_uv(C_UV_AW downto 1));

    --------------------------------------------------------------------------
    -- E3: EBR data arrives (bank processes); flag off-line columns.
    --------------------------------------------------------------------------
    p_e3 : process(clk)
    begin
        if rising_edge(clk) then
            if s_rd_y < 0 or s_rd_y >= signed(resize(s_width, C_RW)) then
                s_bg3 <= '1';
            else
                s_bg3 <= '0';
            end if;
            s_seed3 <= s_seed2;
        end if;
    end process p_e3;

    --------------------------------------------------------------------------
    -- E4: select the bank the line above wrote (the one this pixel doesn't).
    --------------------------------------------------------------------------
    p_e4 : process(clk)
    begin
        if rising_edge(clk) then
            if s_ctx(3).par = '1' then
                s_p4 <= (y => unsigned(s_rdY0), u => unsigned(s_rdU0), v => unsigned(s_rdV0));
            else
                s_p4 <= (y => unsigned(s_rdY1), u => unsigned(s_rdU1), v => unsigned(s_rdV1));
            end if;
            s_tz4 <= s_bg3 or s_seed3;
        end if;
    end process p_e4;

    --------------------------------------------------------------------------
    -- E5: reference (live for Smear, black for Comet), |prev - ref| and its
    -- sign, blend weight (0 off-line and on a field's first line).
    --------------------------------------------------------------------------
    p_e5 : process(clk)
        variable v_ref : t_yuv;
        variable v_dy, v_du, v_dv : unsigned(C_DW downto 0);
    begin
        if rising_edge(clk) then
            if s_comet = '1' then
                v_ref := C_YUV_BLACK;
            else
                v_ref := s_live(4);
            end if;
            v_dy := f_sdiff(s_p4.y, v_ref.y);
            v_du := f_sdiff(s_p4.u, v_ref.u);
            v_dv := f_sdiff(s_p4.v, v_ref.v);
            s_m5 <= (y => v_dy(C_DW - 1 downto 0), u => v_du(C_DW - 1 downto 0),
                     v => v_dv(C_DW - 1 downto 0));
            s_n5 <= v_dv(C_DW) & v_du(C_DW) & v_dy(C_DW);
            s_b5 <= v_ref;
            if s_tz4 = '1' then
                s_t5 <= (others => '0');
            else
                s_t5 <= s_tline;
            end if;
        end if;
    end process p_e5;

    --------------------------------------------------------------------------
    -- E6: t * |diff|, operands split into 5-bit halves (two 10x5 per channel).
    --------------------------------------------------------------------------
    p_e6 : process(clk)
    begin
        if rising_edge(clk) then
            s_pa6_y <= s_t5 * s_m5.y(4 downto 0);
            s_pb6_y <= s_t5 * s_m5.y(9 downto 5);
            s_pa6_u <= s_t5 * s_m5.u(4 downto 0);
            s_pb6_u <= s_t5 * s_m5.u(9 downto 5);
            s_pa6_v <= s_t5 * s_m5.v(4 downto 0);
            s_pb6_v <= s_t5 * s_m5.v(9 downto 5);
            s_b6 <= s_b5;
            s_n6 <= s_n5;
        end if;
    end process p_e6;

    -- E7: combine the partial products.
    p_e7 : process(clk)
    begin
        if rising_edge(clk) then
            s_pm7_y <= shift_left(resize(s_pb6_y, 20), 5) + resize(s_pa6_y, 20);
            s_pm7_u <= shift_left(resize(s_pb6_u, 20), 5) + resize(s_pa6_u, 20);
            s_pm7_v <= shift_left(resize(s_pb6_v, 20), 5) + resize(s_pa6_v, 20);
            s_b7 <= s_b6;
            s_n7 <= s_n6;
        end if;
    end process p_e7;

    -- E8: res = ref +/- floor(t*|diff| / 1024)  (round toward zero).
    p_e8 : process(clk)
    begin
        if rising_edge(clk) then
            s_r8.y <= f_step(s_b7.y, s_pm7_y, s_n7(0));
            s_r8.u <= f_step(s_b7.u, s_pm7_u, s_n7(1));
            s_r8.v <= f_step(s_b7.v, s_pm7_v, s_n7(2));
        end if;
    end process p_e8;

    -- E9: Comet keeps whichever of live / faded trail is brighter.
    p_e9 : process(clk)
    begin
        if rising_edge(clk) then
            if s_comet = '1' and s_live(8).y >= s_r8.y then
                s_wet9 <= s_live(8);
            else
                s_wet9 <= s_r8;
            end if;
            s_dry9 <= s_live(8);
        end if;
    end process p_e9;

    --------------------------------------------------------------------------
    -- Spectrum write-back (E10-E15). Minsky step on signed chroma:
    --   u' = clamp(u + seed - round(a*v/1024));  v' = clamp(v + round(a*u'/1024))
    -- Multiplies take the top 8 chroma bits (so /256); identity at a = 0.
    --------------------------------------------------------------------------
    p_w1 : process(clk)
        variable v_cv : signed(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_cv    := signed(s_wet9.v xor C_NEUT);
            s_p1_10 <= s_spec_a * v_cv(C_DW - 1 downto 2);
            s_cu10  <= signed(s_wet9.u xor C_NEUT);
            s_cv10  <= v_cv;
            if s_wet9.y > C_BLACK then
                s_yb10 <= s_wet9.y - C_BLACK;
            else
                s_yb10 <= (others => '0');
            end if;
            s_y10 <= s_wet9.y;
        end if;
    end process p_w1;

    p_w2 : process(clk)
        variable v_rb   : signed(1 downto 0);
        variable v_seed : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_rb    := '0' & s_p1_10(7);
            s_p1r11 <= s_p1_10(19 downto 8) + v_rb;
            case to_integer(s_seed_sel) is
                when 8      => v_seed := s_yb10(9 downto 2);
                when 7      => v_seed := resize(s_yb10(9 downto 3), 8);
                when 6      => v_seed := resize(s_yb10(9 downto 4), 8);
                when 5      => v_seed := resize(s_yb10(9 downto 5), 8);
                when 4      => v_seed := resize(s_yb10(9 downto 6), 8);
                when 3      => v_seed := resize(s_yb10(9 downto 7), 8);
                when 2      => v_seed := resize(s_yb10(9 downto 8), 8);
                when 1      => v_seed := resize(s_yb10(9 downto 9), 8);
                when others => v_seed := (others => '0');
            end case;
            s_cus11 <= resize(s_cu10, C_DW + 1) + signed(resize(v_seed, C_DW + 1));
            s_cv11  <= s_cv10;
            s_y11   <= s_y10;
        end if;
    end process p_w2;

    p_w3 : process(clk)
    begin
        if rising_edge(clk) then
            s_u12  <= f_clamp_c(resize(s_cus11, C_DW + 2) - resize(s_p1r11, C_DW + 2));
            s_cv12 <= s_cv11;
            s_y12  <= s_y11;
        end if;
    end process p_w3;

    p_w4 : process(clk)
    begin
        if rising_edge(clk) then
            s_p2_13 <= s_spec_a * s_u12(C_DW - 1 downto 2);
            s_u13   <= s_u12;
            s_cv13  <= s_cv12;
            s_y13   <= s_y12;
        end if;
    end process p_w4;

    p_w5 : process(clk)
        variable v_rb : signed(1 downto 0);
    begin
        if rising_edge(clk) then
            v_rb    := '0' & s_p2_13(7);
            s_p2r14 <= s_p2_13(19 downto 8) + v_rb;
            s_u14   <= s_u13;
            s_cv14  <= s_cv13;
            s_y14   <= s_y13;
        end if;
    end process p_w5;

    p_w6 : process(clk)
    begin
        if rising_edge(clk) then
            s_wv15 <= std_logic_vector(unsigned(
                          f_clamp_c(resize(s_cv14, C_DW + 2) + resize(s_p2r14, C_DW + 2)))
                      xor C_NEUT);
            s_wu15 <= std_logic_vector(unsigned(s_u14) xor C_NEUT);
            s_wy15 <= std_logic_vector(s_y14);
        end if;
    end process p_w6;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank, one write port each (canonical 1W1R:
    -- a second addressed write makes yosys fall back to flip-flops). Each
    -- reads unconditionally at the registered E2 column and writes the
    -- carried result for the pixels whose parity matches the bank.
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(unsigned(s_rd_y(C_Y_AW - 1 downto 0))));
            if s_ctx(15).we = '1' and s_ctx(15).par = '0' then
                lbY0(to_integer(s_ctx(15).px)) <= s_wy15;
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(unsigned(s_rd_y(C_Y_AW - 1 downto 0))));
            if s_ctx(15).we = '1' and s_ctx(15).par = '1' then
                lbY1(to_integer(s_ctx(15).px)) <= s_wy15;
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_ra_uv));
            if s_ctx(15).we = '1' and s_ctx(15).par = '0' then
                lbU0(to_integer(s_ctx(15).px(C_Y_AW - 1 downto 1))) <= s_wu15;
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_ra_uv));
            if s_ctx(15).we = '1' and s_ctx(15).par = '1' then
                lbU1(to_integer(s_ctx(15).px(C_Y_AW - 1 downto 1))) <= s_wu15;
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_ra_uv));
            if s_ctx(15).we = '1' and s_ctx(15).par = '0' then
                lbV0(to_integer(s_ctx(15).px(C_Y_AW - 1 downto 1))) <= s_wv15;
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_ra_uv));
            if s_ctx(15).we = '1' and s_ctx(15).par = '1' then
                lbV1(to_integer(s_ctx(15).px(C_Y_AW - 1 downto 1))) <= s_wv15;
            end if;
        end if;
    end process p_lbV1;

    --------------------------------------------------------------------------
    -- Dry/wet mix (E10-E13): out = wet +/- rtz(|dry - wet| * (1023 - Mix)),
    -- the same split-multiply shape as the blend (the SDK interpolator's
    -- unsplit 11x11 multiply was the HD critical path). Mix 100% is exactly
    -- wet; Bypass selects dry at the same latency.
    --------------------------------------------------------------------------
    p_m10 : process(clk)
        variable v_dy, v_du, v_dv : unsigned(C_DW downto 0);
    begin
        if rising_edge(clk) then
            v_dy := f_sdiff(s_dry9.y, s_wet9.y);
            v_du := f_sdiff(s_dry9.u, s_wet9.u);
            v_dv := f_sdiff(s_dry9.v, s_wet9.v);
            s_mm10 <= (y => v_dy(C_DW - 1 downto 0), u => v_du(C_DW - 1 downto 0),
                       v => v_dv(C_DW - 1 downto 0));
            s_mn10   <= v_dv(C_DW) & v_du(C_DW) & v_dy(C_DW);
            s_mwet10 <= s_wet9;
            s_mdry10 <= s_dry9;
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
            s_mwet11 <= s_mwet10;
            s_mdry11 <= s_mdry10;
            s_mn11   <= s_mn10;
        end if;
    end process p_m11;

    p_m12 : process(clk)
    begin
        if rising_edge(clk) then
            s_mpm12_y <= shift_left(resize(s_mpb11_y, 20), 5) + resize(s_mpa11_y, 20);
            s_mpm12_u <= shift_left(resize(s_mpb11_u, 20), 5) + resize(s_mpa11_u, 20);
            s_mpm12_v <= shift_left(resize(s_mpb11_v, 20), 5) + resize(s_mpa11_v, 20);
            s_mwet12 <= s_mwet11;
            s_mdry12 <= s_mdry11;
            s_mn12   <= s_mn11;
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
    -- LFSR noise source (free-running) for Slope Mod
    --------------------------------------------------------------------------
    lfsr_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr_q
        );

    --------------------------------------------------------------------------
    -- E14: blanking gate -> output register. Neutral whenever the
    -- latency-aligned avid is low.
    --------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_TOTAL_LATENCY - 2) = '1' then
                s_out <= s_mix13;
            else
                s_out <= C_YUV_BLACK;
            end if;
        end if;
    end process p_out;

    s_io_align_in.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);
    s_io_align_in.y       <= std_logic_vector(s_out.y);
    s_io_align_in.u       <= std_logic_vector(s_out.u);
    s_io_align_in.v       <= std_logic_vector(s_out.v);

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

end architecture shearline;
