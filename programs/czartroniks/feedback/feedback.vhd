-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- feedback.vhd  (v0.1.4 -- "Feedback": a digital camera-at-the-monitor loop)
--
-- A video feedback exploration.  Videomancer has no frame buffer, so the
-- loop runs through a coarse store: last frame's OUTPUT, averaged into a
-- 128x64 luma grid (4 bit) and a 64x32 chroma grid (U4 V4), two banks
-- ping-ponged by field parity.  Every output pixel reads the old store back
-- through a per-pixel affine transform (zoom / rotate / drift about the
-- screen centre, optional mirror fold, black or wrapped edges) with
-- BILINEAR interpolation, so the echoes are smooth at full resolution, and
-- the live picture is composited over them (luma key / mix / add /
-- lighten).  What goes back into the store passes through one
-- "generation" of camera-loop processing: gain with a wave-fold above
-- unity, hue rotation (+ colour bloom from luma), optional crisp contrast
-- and video negative.  Stochastic (LFSR-dithered) rounding keeps slow
-- decays and small rotations alive at 4 bits.
--
-- Controls:
--   K1 Zoom    (bipolar: left = tunnel / shrink inward, right = expand)
--   K2 Rotate  (bipolar spin per generation, up to ~84 deg)
--   K3 Hue     (bipolar hue rotation per generation; also blooms colour
--               from luma so mono sources go rainbow)
--   K4 Drift   (bipolar: right = rise, left = fall)
--   K5 Live    (how much live video enters: key level / mix / add amount)
--   K6 Blend   Key / Mix / Add / Lighten
--   S7 Edges   Black / Wrap      S8 Mirror     S9 Pixels Smooth / Blocks
--   S10 Negative                 S11 Crisp (contrast x1.5 per generation)
--   P12 Feedback: loop gain 0.44 .. 1.0 (at 75% travel) .. 1.5 overdrive,
--       which folds instead of clipping (rings, not whiteout).
--
-- Store organisation: Y sub-bank (c(0), r(0)) holds {bank, r>>1, c>>1}
-- (4 x 4096 x 4), chroma sub-bank (cc(0), rc(0)) holds {bank, rc>>1,
-- cc>>1} (4 x 1024 x 8).  Any 2x2 neighbourhood has one cell in each
-- sub-bank, so one read per sub-bank per clock gives all four bilinear
-- taps.  Each array: one registered read, one write statement.
--
-- Alignment: the read DDA runs C_LOOK pixels AHEAD of the input raster
-- (idle value = pixel C_LOOK during blanking), so the 11-stage read
-- pipeline lands on the mixer together with live video that is only 3
-- registers deep.  The write side samples the composite at k=8 with its
-- own cell DDA; the IIR's group delay is cancelled by a per-frame trim in
-- the read offsets (exact at 1920 wide), the weight floor by +1/32 cell.
--
-- 25 EBR (24 store + 1 sine ROM).  Per-frame math in vblank on one serial
-- multiplier, one serial divider and one accumulator.
--
-- v0.1.3 (2026-10-17) resolution scaling; 1080p output bit-identical:
--   * Rotation stays a rotation on non-square field lines: the cross terms
--     carry rho = field-line height / pixel width from the timing ID
--     (480i 2.22, 576i 1.875, 1080i 2, 480p 1.125, 576p 0.94; 1 at
--     1080p / 720p): dudy = -b kx rho, dvdx = b ky / rho (two more serial
--     multiplies in the vblank sequence).
--   * SD: 720 / 128 = 5.625 px cells, so the cell-end captures land 7/16 px
--     (luma) / 3/8 px (chroma) late on average; the read offsets trim it,
--     otherwise the echoes creep left ~0.44 px every field.
--   * 720p: the write IIR alternates shift 3 / 2 per pixel (was 3, the
--     1080p value on 2/3-size cells, so echoes softened faster), with its
--     own group-delay trims.
--
-- v0.1.4 (2026-10-18) fit for the stock SDK SD Standalone; output bit-identical
-- to v0.1.3 in every mode:
--   * no_rw_check on the 8 store RAMs (read bank = not par, write bank = par:
--     they never meet), so yosys drops its read-before-write emulation.
--   * One-adder forms: the sequencer accumulator (load = add to a masked acc,
--     sub = add of not op + 1), the divider (one compare on the selected divisor),
--     bip_mag (one subtract), the mixer rounding ((p >> 5) + p(4) carry-in).
--   * Booth mixer products (3 partials); 13-bit vertical bilinear products.
--   * Sequencer step-ready flag kept as a register (sq_rdy), so the step
--     decode is pc + 1 bit instead of pc + two 5-bit zero tests (HD timing).
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture feedback of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (10 + 2 = 12 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 12;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_PIPE : natural := 9;     -- sync taps pipe(0..8); output latency 10
    constant C_LOOK : natural := 8;     -- read DDA lookahead (11 read stages - 3);
                                        -- p_seq steps 18-20 / 28-30 hard-code 8.5

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- sin(i * 90deg / 256) * 16384, i = 0..255 (cos(t) read as sin(255 - t)).
    -- Integer elements: an array-of-unsigned type here would make every
    -- "unsigned & literal" in the file ambiguous.
    type t_sinrom is array (0 to 255) of natural range 0 to 16384;
    constant SINROM : t_sinrom := (
        0, 101, 201, 302, 402, 503, 603, 704, 804, 904, 1005, 1105, 1205, 1306, 1406, 1506,
        1606, 1706, 1806, 1906, 2006, 2105, 2205, 2305, 2404, 2503, 2603, 2702, 2801, 2900, 2999, 3098,
        3196, 3295, 3393, 3492, 3590, 3688, 3786, 3883, 3981, 4078, 4176, 4273, 4370, 4467, 4563, 4660,
        4756, 4852, 4948, 5044, 5139, 5235, 5330, 5425, 5520, 5614, 5708, 5803, 5897, 5990, 6084, 6177,
        6270, 6363, 6455, 6547, 6639, 6731, 6823, 6914, 7005, 7096, 7186, 7276, 7366, 7456, 7545, 7635,
        7723, 7812, 7900, 7988, 8076, 8163, 8250, 8337, 8423, 8509, 8595, 8680, 8765, 8850, 8935, 9019,
        9102, 9186, 9269, 9352, 9434, 9516, 9598, 9679, 9760, 9841, 9921, 10001, 10080, 10159, 10238, 10316,
        10394, 10471, 10549, 10625, 10702, 10778, 10853, 10928, 11003, 11077, 11151, 11224, 11297, 11370, 11442, 11514,
        11585, 11656, 11727, 11797, 11866, 11935, 12004, 12072, 12140, 12207, 12274, 12340, 12406, 12472, 12537, 12601,
        12665, 12729, 12792, 12854, 12916, 12978, 13039, 13100, 13160, 13219, 13279, 13337, 13395, 13453, 13510, 13567,
        13623, 13678, 13733, 13788, 13842, 13896, 13949, 14001, 14053, 14104, 14155, 14206, 14256, 14305, 14354, 14402,
        14449, 14497, 14543, 14589, 14635, 14680, 14724, 14768, 14811, 14854, 14896, 14937, 14978, 15019, 15059, 15098,
        15137, 15175, 15213, 15250, 15286, 15322, 15357, 15392, 15426, 15460, 15493, 15525, 15557, 15588, 15619, 15649,
        15679, 15707, 15736, 15763, 15791, 15817, 15843, 15868, 15893, 15917, 15941, 15964, 15986, 16008, 16029, 16049,
        16069, 16088, 16107, 16125, 16143, 16160, 16176, 16192, 16207, 16221, 16235, 16248, 16261, 16273, 16284, 16295,
        16305, 16315, 16324, 16332, 16340, 16347, 16353, 16359, 16364, 16369, 16373, 16376, 16379, 16381, 16383, 16384
    );

    -- bipolar knob: |k - 512| minus a 16-step centre deadband (0..496).
    -- |k - 512| = x + c with x = k(8..0) (k >= 512) or not k(8..0) (= 511 - k)
    -- and c = not k(9), so the result is max(0, x - (16 - c)): one subtract
    -- (16 - c = 16 or 15), floored on its sign.
    function bip_mag(k : unsigned(9 downto 0)) return unsigned is
        variable x   : unsigned(8 downto 0);
        variable sub : unsigned(10 downto 0);
        variable t   : unsigned(10 downto 0);
    begin
        x := k(8 downto 0) xor (8 downto 0 => not k(9));
        sub := "000000" & k(9) & (not k(9)) & (not k(9)) & (not k(9)) & (not k(9));
        t := ("00" & x) - sub;
        if t(10) = '1' then
            return to_unsigned(0, 9);
        else
            return t(8 downto 0);
        end if;
    end function;

    -- 9 x 10 signed product, radix-4 Booth: 5 partial products of
    -- {0, +-b, +-2b} (negatives as one's complement, the +1s gathered into
    -- one constant-free correction word), balanced adds
    function booth9x10(a : signed(8 downto 0); b : signed(9 downto 0)) return signed is
        variable ae   : std_logic_vector(10 downto 0);
        variable d    : std_logic_vector(2 downto 0);
        variable mag  : signed(18 downto 0);
        variable pp   : signed(18 downto 0);
        variable b1   : signed(18 downto 0);
        variable b2   : signed(18 downto 0);
        variable corr : signed(18 downto 0);
        type t_pp is array (0 to 4) of signed(18 downto 0);
        variable p    : t_pp;
    begin
        ae := std_logic_vector(resize(a, 10)) & '0';
        b1 := resize(b, 19);
        b2 := shift_left(b1, 1);
        corr := (others => '0');
        for i in 0 to 4 loop
            d := ae(2 * i + 2 downto 2 * i);
            case d is
                when "001" | "010" | "101" | "110" => mag := b1;
                when "011" | "100"                 => mag := b2;
                when others                        => mag := (others => '0');
            end case;
            if d(2) = '1' and d /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        return ((p(0) + p(1)) + (p(2) + p(3))) + (p(4) + corr);
    end function;

    -- signed d times a 4-bit weight: AND-gated partials, balanced adds
    function smul4(d : signed; w : unsigned(3 downto 0); n : natural) return signed is
        variable dd, p0, p1, p2, p3 : signed(n - 1 downto 0);
    begin
        dd := resize(d, n);
        p0 := (others => '0'); p1 := (others => '0');
        p2 := (others => '0'); p3 := (others => '0');
        if w(0) = '1' then p0 := dd; end if;
        if w(1) = '1' then p1 := shift_left(dd, 1); end if;
        if w(2) = '1' then p2 := shift_left(dd, 2); end if;
        if w(3) = '1' then p3 := shift_left(dd, 3); end if;
        return (p0 + p1) + (p2 + p3);
    end function;

    -- signed d (11 bit) times a 5-bit coefficient (0..31), 17-bit result:
    -- radix-4 Booth on 0 & w & 0, 3 partials of {0, +-d, +-2d} (negatives as
    -- one's complement, the +1s gathered into one correction word)
    function smul5(d : signed(10 downto 0); w : unsigned(4 downto 0)) return signed is
        variable we   : std_logic_vector(6 downto 0);
        variable dg   : std_logic_vector(2 downto 0);
        variable mag  : signed(16 downto 0);
        variable pp   : signed(16 downto 0);
        variable b1   : signed(16 downto 0);
        variable b2   : signed(16 downto 0);
        variable corr : signed(16 downto 0);
        type t_pp is array (0 to 2) of signed(16 downto 0);
        variable p    : t_pp;
    begin
        we := '0' & std_logic_vector(w) & '0';
        b1 := resize(d, 17);
        b2 := shift_left(b1, 1);
        corr := (others => '0');
        for i in 0 to 2 loop
            dg := we(2 * i + 2 downto 2 * i);
            case dg is
                when "001" | "010" | "101" | "110" => mag := b1;
                when "011" | "100"                 => mag := b2;
                when others                        => mag := (others => '0');
            end case;
            if dg(2) = '1' and dg /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        return (p(0) + p(1)) + (p(2) + corr);
    end function;

    --------------------------------------------------------------------------
    -- Control decode (free-running).
    --------------------------------------------------------------------------
    signal k_mag1, k_mag2, k_mag3, k_mag4 : unsigned(8 downto 0) := (others => '0');
    signal k_neg1, k_neg2, k_neg3, k_neg4 : std_logic := '0';
    signal k_t    : unsigned(9 downto 0) := (others => '0');   -- 768 - P12
    signal k_over : unsigned(7 downto 0) := (others => '0');   -- P12 - 768
    signal k_lo   : std_logic := '1';
    signal k_m5   : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal k_thr  : signed(11 downto 0) := to_signed(512, 12);
    signal k_mode : unsigned(1 downto 0) := (others => '0');
    signal sw_wrap, sw_mir, sw_blk, sw_neg, sw_crisp : std_logic := '0';
    -- Rotation aspect (v0.1.3): rho = height of one field line / width of
    -- one pixel on the screen (virtual y step / x step from the timing ID),
    -- Q14.  1.0 at 1080p and 720p (square pixels), so 1080p is unchanged.
    signal c_rx : unsigned(17 downto 0) := to_unsigned(16384, 18);   -- rho
    signal c_ry : unsigned(17 downto 0) := to_unsigned(16384, 18);   -- 1 / rho
    signal c_sd : std_logic := '0';   -- SD timing (720 wide): capture-phase trim
    signal c_720 : std_logic := '0';  -- 720p timing: alternating IIR shift

    --------------------------------------------------------------------------
    -- Input raster, measurement, frame anchor.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to C_PIPE - 1);

    signal prev_hsync_n, prev_vsync_n, prev_avid : std_logic := '1';
    signal lfsr : unsigned(15 downto 0) := x"ACE1";
    signal px   : unsigned(11 downto 0) := (others => '0');
    signal py   : unsigned(10 downto 0) := (others => '0');
    signal line_act, frame_act, frame_armed : std_logic := '0';
    signal w_meas : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal h_meas : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal g_h    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_k3   : std_logic := '1';       -- IIR shift 3 (wide) / 2 (narrow)
    signal g_alt  : std_logic := '0';       -- 720p: IIR shift alternates 3 / 2
    signal par    : std_logic := '0';
    signal anchor : std_logic := '0';
    signal hpar, line_par : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame sequencer (vblank).
    --------------------------------------------------------------------------
    signal pc      : unsigned(5 downto 0) := (others => '0');
    signal ma, mb  : unsigned(17 downto 0) := (others => '0');
    signal macc    : unsigned(35 downto 0) := (others => '0');
    signal mcnt    : unsigned(4 downto 0) := (others => '0');
    signal dsel    : std_logic := '0';                         -- 0: 2^25/W, 1: 2^24/H
    signal drem    : unsigned(12 downto 0) := (others => '0');
    signal dcnt    : unsigned(4 downto 0) := (others => '0');
    signal rom_a   : unsigned(7 downto 0) := (others => '0');
    signal rom_q, rom_f : unsigned(14 downto 0) := (others => '0');
    signal acc     : signed(29 downto 0) := (others => '0');
    signal alu_op  : signed(29 downto 0) := (others => '0');   -- registered operand
    signal alu_ctl : unsigned(1 downto 0) := (others => '0');
    signal sq_go   : std_logic := '0';
    -- step-ready flag, = (no step last clock) and mcnt = 0 and dcnt = 0,
    -- kept as a register so the step decode is pc + 1 bit
    signal sq_rdy  : std_logic := '1';

    signal s_s    : unsigned(14 downto 0) := to_unsigned(16384, 15);
    signal th, ph : unsigned(7 downto 0) := (others => '0');
    signal a_q, b_q : unsigned(17 downto 0) := (others => '0');
    signal kx, ky : unsigned(17 downto 0) := (others => '0');

    -- per-frame values used by the pixel paths (written in vblank)
    signal commit : std_logic := '0';
    signal c_dudx, c_dudy, c_dvdx, c_dvdy : signed(19 downto 0) := (others => '0');
    signal c_u0, c_v0 : signed(27 downto 0) := (others => '0');
    signal c_cy, c_cc : signed(19 downto 0) := (others => '0');
    signal c_g  : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal c_hc : signed(9 downto 0) := to_signed(256, 10);
    signal c_hs : signed(9 downto 0) := (others => '0');
    signal c_hue : std_logic := '0';

    --------------------------------------------------------------------------
    -- Read pipeline R0..R11 (signal prefixes keep their pre-merge names).
    --------------------------------------------------------------------------
    signal u_line, v_line, u_acc, v_acc : signed(27 downto 0) := (others => '0');
    signal r1_um, r1_v : signed(19 downto 0) := (others => '0');
    signal r2_uy, r2_uc, r2_vy, r2_vc : signed(19 downto 0) := (others => '0');

    signal ya00, ya10, ya01, ya11 : unsigned(11 downto 0) := (others => '0');
    signal ca00, ca10, ca01, ca11 : unsigned(9 downto 0) := (others => '0');
    -- side band: 27 Ly 26 Ty 25..22 fu 21..18 fv 17 Lv 16 Rv 15 Tv 14 Bv
    --            13 cLx 12 cTy 11..8 fuc 7..4 fvc 3 cLv 2 cRv 1 cTv 0 cBv
    signal sb3, sb4 : std_logic_vector(27 downto 0) := (others => '0');

    type t_yram is array (0 to 4095) of std_logic_vector(3 downto 0);
    type t_cram is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal yr00, yr10, yr01, yr11 : t_yram := (others => (others => '0'));
    signal cr00, cr10, cr01, cr11 : t_cram := (others => (others => '0'));
    signal q00, q10, q01, q11 : std_logic_vector(3 downto 0) := (others => '0');
    signal e00, e10, e01, e11 : std_logic_vector(7 downto 0) := (others => '0');
    -- Read bank (not par) and write bank (par) differ whenever a write can
    -- happen (par flips at the vsync anchor, writes end ~15 clocks after each
    -- line's active video), so read and write never meet on one address:
    -- no read-before-write emulation needed.
    attribute no_rw_check : boolean;
    attribute no_rw_check of yr00, yr10, yr01, yr11 : signal is true;
    attribute no_rw_check of cr00, cr10, cr01, cr11 : signal is true;

    -- R6 reordered taps (Y unsigned 0..15 in 5-bit signed, chroma -8..7)
    signal r6_ytl, r6_ytr, r6_ybl, r6_ybr : signed(4 downto 0) := (others => '0');
    signal r6_utl, r6_utr, r6_ubl, r6_ubr : signed(4 downto 0) := (others => '0');
    signal r6_vtl, r6_vtr, r6_vbl, r6_vbr : signed(4 downto 0) := (others => '0');
    signal r6_wx, r6_wy, r6_wxc, r6_wyc : unsigned(3 downto 0) := (others => '0');
    signal r6_nx, r6_ny, r6_nxc, r6_nyc : std_logic := '0';
    -- R7 differences (Blocks: nearest tap, zero differences)
    signal r7_ytl, r7_ybl, r7_dyt, r7_dyb : signed(4 downto 0) := (others => '0');
    signal r7_utl, r7_ubl, r7_dut, r7_dub : signed(4 downto 0) := (others => '0');
    signal r7_vtl, r7_vbl, r7_dvt, r7_dvb : signed(4 downto 0) := (others => '0');
    signal r7_wx, r7_wy, r7_wxc, r7_wyc : unsigned(3 downto 0) := (others => '0');
    -- R8 horizontal products
    signal r8_ytl, r8_ybl, r8_utl, r8_ubl, r8_vtl, r8_vbl : signed(4 downto 0) := (others => '0');
    signal r8_pyt, r8_pyb, r8_put, r8_pub, r8_pvt, r8_pvb : signed(9 downto 0) := (others => '0');
    signal r8_wy, r8_wyc : unsigned(3 downto 0) := (others => '0');
    -- R10 top row (x16) and vertical difference
    signal r10_yt, r10_ut, r10_vt : signed(9 downto 0) := (others => '0');
    signal r10_dy, r10_du, r10_dv : signed(10 downto 0) := (others => '0');
    signal r10_wy, r10_wyc : unsigned(3 downto 0) := (others => '0');
    -- R11 vertical products
    signal r11_yt, r11_ut, r11_vt : signed(9 downto 0) := (others => '0');
    signal r11_py, r11_pu, r11_pv : signed(14 downto 0) := (others => '0');
    -- R12 interpolated (x256)
    signal r12_y, r12_u, r12_v : signed(14 downto 0) := (others => '0');
    -- R11 feedback pixel
    signal fb_y, fb_u, fb_v : unsigned(9 downto 0) := C_BLKY;

    --------------------------------------------------------------------------
    -- Mixer / output.
    --------------------------------------------------------------------------
    signal k1_d : signed(11 downto 0) := (others => '0');
    signal k2_a : unsigned(4 downto 0) := (others => '0');
    signal x1_dy, x1_du, x1_dv : signed(10 downto 0) := (others => '0');
    signal x1_c : unsigned(4 downto 0) := (others => '0');
    signal x1_fy, x1_fu, x1_fv : unsigned(9 downto 0) := C_BLKY;
    signal x1_lt : std_logic := '0';
    signal x2_py, x2_pu, x2_pv : signed(16 downto 0) := (others => '0');
    signal x2_fy, x2_fu, x2_fv : unsigned(9 downto 0) := C_BLKY;
    signal x2_lt : std_logic := '0';
    signal x3_ay, x3_au, x3_av : signed(12 downto 0) := (others => '0');
    signal x4_ay, x4_au, x4_av : unsigned(9 downto 0) := C_BLKY;
    signal x5_y, x5_u, x5_v : unsigned(9 downto 0) := C_BLKY;
    signal x5_bl : std_logic := '1';
    signal d6_y, d6_u, d6_v, d7_y : unsigned(9 downto 0) := C_BLKY;
    signal d6_bl : std_logic := '1';
    signal s_io : t_video_stream_yuv444_30b;

    --------------------------------------------------------------------------
    -- Write side: cell DDA + IIR on the composite (k=8), pulse pipelines.
    --------------------------------------------------------------------------
    signal w_prev : std_logic := '0';
    signal w_ks   : std_logic := '1';     -- IIR shift select for this pixel
    signal w_x    : unsigned(12 downto 0) := (others => '0');
    signal w_c    : unsigned(6 downto 0) := (others => '0');
    signal w_yi, w_ui, w_vi : unsigned(13 downto 0) := (others => '0');
    signal w_ey, w_eu, w_ev : signed(14 downto 0) := (others => '0');
    signal w_yacc : unsigned(11 downto 0) := (others => '0');
    signal w_r    : unsigned(5 downto 0) := (others => '0');
    signal w_lsn, w_csn : std_logic := '0';          -- next-line flags
    signal w_lsl, w_csl, w_par : std_logic := '0';   -- latched for this line
    signal w_rl   : unsigned(5 downto 0) := (others => '0');

    -- luma chain
    signal l1_vy, l1_vc : std_logic := '0';
    signal l1_c : unsigned(6 downto 0) := (others => '0');
    signal l1_r : unsigned(5 downto 0) := (others => '0');
    signal l1_y : unsigned(7 downto 0) := (others => '0');
    signal l1_u, l1_v : signed(8 downto 0) := (others => '0');
    signal l2_vy : std_logic := '0';
    signal l2_c : unsigned(6 downto 0) := (others => '0');
    signal l2_r : unsigned(5 downto 0) := (others => '0');
    -- luma gain table: entry y = (y * c_g + 512) >> 10.  c_g only changes in
    -- the vblank sequence, so the table is refilled after each commit (one
    -- add per entry, 256 clocks) instead of a per-pixel 8x11 multiply.  It
    -- starts as the identity, which is the power-up gain (c_g = 1024).
    type t_gtab is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_gid return t_gtab is
        variable v : t_gtab;
    begin
        for i in 0 to 255 loop
            v(i) := std_logic_vector(to_unsigned(i, 16));
        end loop;
        return v;
    end function;
    signal g_tab : t_gtab := f_gid;
    attribute no_rw_check of g_tab : signal is true;
    signal g_rd  : std_logic_vector(15 downto 0) := (others => '0');
    signal g_we  : std_logic := '0';
    signal g_wa  : unsigned(7 downto 0) := (others => '0');
    signal g_wd  : unsigned(9 downto 0) := (others => '0');
    signal g_run : std_logic := '0';
    signal g_idx : unsigned(7 downto 0) := (others => '0');
    signal g_acc : unsigned(19 downto 0) := (others => '0');
    signal l3_vy : std_logic := '0';
    signal l3_c : unsigned(6 downto 0) := (others => '0');
    signal l3_r : unsigned(5 downto 0) := (others => '0');
    signal l3_y : unsigned(9 downto 0) := (others => '0');
    signal l4_vy : std_logic := '0';
    signal l4_c : unsigned(6 downto 0) := (others => '0');
    signal l4_r : unsigned(5 downto 0) := (others => '0');
    signal l4_y : signed(11 downto 0) := (others => '0');
    signal l5_vy : std_logic := '0';
    signal l5_c : unsigned(6 downto 0) := (others => '0');
    signal l5_r : unsigned(5 downto 0) := (others => '0');
    signal l5_y : signed(11 downto 0) := (others => '0');
    signal l6_vy : std_logic := '0';
    signal l6_c : unsigned(6 downto 0) := (others => '0');
    signal l6_r : unsigned(5 downto 0) := (others => '0');
    signal l6_y : unsigned(7 downto 0) := (others => '0');
    signal wy_v, wy_px, wy_py : std_logic := '0';
    signal wy_a : unsigned(11 downto 0) := (others => '0');
    signal wy_d : std_logic_vector(3 downto 0) := (others => '0');

    -- chroma chain: one multiplier, four products over phases 1..6
    signal cs_ph : unsigned(2 downto 0) := (others => '0');
    signal cs_u, cs_v : signed(8 downto 0) := (others => '0');
    signal cs_inj : unsigned(4 downto 0) := (others => '0');
    signal cs_c : unsigned(6 downto 0) := (others => '0');
    signal cs_r : unsigned(5 downto 0) := (others => '0');
    signal cm_a : signed(8 downto 0) := (others => '0');
    signal cm_b : signed(9 downto 0) := (others => '0');
    signal cm_p : signed(18 downto 0) := (others => '0');
    signal acc_u, acc_v : signed(19 downto 0) := (others => '0');
    signal c8_v : std_logic := '0';
    signal c8_c : unsigned(6 downto 0) := (others => '0');
    signal c8_r : unsigned(5 downto 0) := (others => '0');
    signal c8_u, c8_v2 : signed(11 downto 0) := (others => '0');
    signal c9_v : std_logic := '0';
    signal c9_c : unsigned(6 downto 0) := (others => '0');
    signal c9_r : unsigned(5 downto 0) := (others => '0');
    signal c9_u, c9_v2 : signed(7 downto 0) := (others => '0');
    signal wc_v, wc_px, wc_py : std_logic := '0';
    signal wc_a : unsigned(9 downto 0) := (others => '0');
    signal wc_d : std_logic_vector(7 downto 0) := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0);
        variable rsw : std_logic_vector(4 downto 0);
    begin
        if rising_edge(clk) then
            rk1  := unsigned(registers_in(0)(9 downto 0));
            rk2  := unsigned(registers_in(1)(9 downto 0));
            rk3  := unsigned(registers_in(2)(9 downto 0));
            rk4  := unsigned(registers_in(3)(9 downto 0));
            rk5  := unsigned(registers_in(4)(9 downto 0));
            rk6  := unsigned(registers_in(5)(9 downto 0));
            rsw  := registers_in(6)(4 downto 0);
            rp12 := unsigned(registers_in(7)(9 downto 0));

            k_mag1 <= bip_mag(rk1); k_neg1 <= not rk1(9);
            k_mag2 <= bip_mag(rk2); k_neg2 <= not rk2(9);
            k_mag3 <= bip_mag(rk3); k_neg3 <= not rk3(9);
            k_mag4 <= bip_mag(rk4); k_neg4 <= not rk4(9);

            -- P12: below 768 the gain is 1 - t^2/2^20 (fine near unity),
            -- above it 1 + 2*(P12-768)/1024 (overdrive, folds)
            if rp12 < 768 then
                k_lo   <= '1';
                k_t    <= to_unsigned(768, 10) - rp12;
                k_over <= (others => '0');
            else
                k_lo   <= '0';
                k_t    <= (others => '0');
                k_over <= resize(rp12 - 768, 8);
            end if;

            -- K5 Live: mix/add amount 1..31 of 32, key threshold 1000..104
            if rk5(9 downto 5) = 0 then k_m5 <= to_unsigned(1, 5); else k_m5 <= rk5(9 downto 5); end if;
            k_thr <= to_signed(1000, 12) - signed(resize(rk5, 12)) + signed(resize(rk5(9 downto 3), 12));

            -- K6 Blend (labelled: firmware may load the label INDEX 0..3)
            if rk6 < 4 then
                k_mode <= rk6(1 downto 0);
            else
                k_mode <= rk6(9 downto 8);
            end if;

            sw_wrap  <= rsw(0);   -- S7  Edges Black / Wrap
            sw_mir   <= rsw(1);   -- S8  Mirror
            sw_blk   <= rsw(2);   -- S9  Pixels Smooth / Blocks
            sw_neg   <= rsw(3);   -- S10 Negative
            sw_crisp <= rsw(4);   -- S11 Crisp

            -- rho = (virtual y step per field line) / (x step per pixel):
            -- SD is a 4:3 screen 1440 virtual units wide, interlaced field
            -- lines are two frame lines tall
            c_sd <= not (registers_in(8)(1) or registers_in(8)(0));   -- ids 0/4/8/12
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 5 | 6 | 14 => c_720 <= '1';
                when others     => c_720 <= '0';
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0      => c_rx <= to_unsigned(36409, 18); c_ry <= to_unsigned(7373, 18);    -- 480i 2.222
                when 4      => c_rx <= to_unsigned(18432, 18); c_ry <= to_unsigned(14564, 18);   -- 480p 1.125
                when 8      => c_rx <= to_unsigned(30720, 18); c_ry <= to_unsigned(8738, 18);    -- 576i 1.875
                when 12     => c_rx <= to_unsigned(15360, 18); c_ry <= to_unsigned(17476, 18);   -- 576p 0.9375
                when 1 | 2 | 10 => c_rx <= to_unsigned(32768, 18); c_ry <= to_unsigned(8192, 18); -- 1080i 2.0
                when others => c_rx <= to_unsigned(16384, 18); c_ry <= to_unsigned(16384, 18);   -- 1080p, 720p
            end case;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Input raster: measurement, hsync->avid parity, frame anchor.
    --------------------------------------------------------------------------
    p_input : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- encoder pairs Cb/Cr by counting from hsync: record each line's
            -- hsync->avid offset parity (drape pattern)
            hpar <= not hpar;
            if v_h_edge = '1' then hpar <= '0'; end if;
            if data_in.avid = '1' and prev_avid = '0' then line_par <= hpar; end if;

            if v_h_edge = '1' then
                if px > 32 then w_meas <= px; end if;
                px <= (others => '0');
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    h_meas   <= resize(py, 12) + 1;
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                line_act    <= '1';
                px <= px + 1;
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                end if;
            end if;

            anchor <= '0';
            if v_v_edge = '1' and frame_armed = '1' then
                -- field anchor (serration-guarded): latch geometry, flip banks
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                anchor      <= '1';
                par         <= not par;
                g_w <= w_meas;
                g_h <= h_meas;
                if w_meas >= 1024 then g_k3 <= '1'; else g_k3 <= '0'; end if;
                g_alt <= c_720;
            end if;
        end if;
    end process p_input;

    --------------------------------------------------------------------------
    -- Per-frame sequencer: a program counter over one serial multiplier,
    -- one serial divider and one accumulator (one add/sub per step).
    -- Products chain straight from macc into the next operand or the
    -- accumulator, so almost nothing is held between steps.
    --   s  = 1 -/+ zoom^2       a = s cos(th)   b = s sin(th)
    --   dudx = a kx  dudy = -b kx rho  dvdx = b ky / rho  dvdy = a ky
    --   (rho = field-line height / pixel width on screen: 1 at 1080p/720p)
    --   u0 = 64 - 64a - dudy H/2 + dudy/2 + (C_LOOK + 1/2) dudx      (Q18)
    --   v0 = 32 - 32a - dvdx W/2 + dvdy/2 + (C_LOOK + 1/2) dvdx + drift
    -- (u0, v0 = pixel C_LOOK's centre on line 0's centre.)  The sine ROM
    -- is addressed one step ahead; each multiply/divide covers its 3-clock
    -- latency.  Steps run every other clock: the accumulator operand is
    -- registered in the step and applied in the following idle clock, so
    -- the 30-bit add starts from registers (this cone is quasi-static but
    -- STA times it at the pixel clock).  ~1300 clocks after the anchor.
    --------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_rem : unsigned(12 downto 0);
        variable v_dv  : unsigned(12 downto 0);
        variable v_qb  : std_logic;
        variable v_nb  : std_logic;
        variable v_aop : unsigned(1 downto 0);   -- 01 load, 10 add, 11 sub
        variable v_op  : signed(29 downto 0);
        variable v_mul : std_logic;
        variable v_ma, v_mb : unsigned(17 downto 0);
        variable v_div : std_logic;
        variable v_ds  : std_logic;
        variable v_acc, v_opx : signed(29 downto 0);
    begin
        if rising_edge(clk) then
            commit <= '0';

            -- serial multiplier: macc = ma * mb, MSB-first, 18 steps
            if mcnt /= 0 then
                if mb(17) = '1' then
                    macc <= (macc(34 downto 0) & '0') + resize(ma, 36);
                else
                    macc <= macc(34 downto 0) & '0';
                end if;
                mb   <= mb(16 downto 0) & '0';
                mcnt <= mcnt - 1;
            end if;

            -- serial restoring divider: kx = 2^25 / W or ky = 2^24 / H, 26 steps;
            -- the numerator is a single 1 bit, generated from the count
            if dcnt /= 0 then
                if (dsel = '0' and dcnt = 26) or (dsel = '1' and dcnt = 25) then
                    v_nb := '1';
                else
                    v_nb := '0';
                end if;
                v_rem := drem(11 downto 0) & v_nb;
                if dsel = '0' then v_dv := resize(g_w, 13); else v_dv := resize(g_h, 13); end if;
                if v_rem >= v_dv then
                    drem <= v_rem - v_dv; v_qb := '1';
                else
                    drem <= v_rem; v_qb := '0';
                end if;
                if dsel = '0' then kx <= kx(16 downto 0) & v_qb;
                else               ky <= ky(16 downto 0) & v_qb; end if;
                dcnt <= dcnt - 1;
            end if;

            -- sine ROM: address -> EBR -> fabric
            rom_q <= to_unsigned(SINROM(to_integer(rom_a)), 15);
            rom_f <= rom_q;

            v_aop := "00"; v_op := (others => '0');
            v_mul := '0'; v_ma := (others => '0'); v_mb := (others => '0');
            v_div := '0'; v_ds := '0';

            if anchor = '1' then sq_go <= '1'; end if;

            -- sq_rdy: a step runs every other clock at most (the clock after a
            -- step is the ALU's idle clock), once the multiplier and divider
            -- are done.  Next value = no step now and both counters reach 0.
            sq_rdy <= (not sq_rdy) and (not (mcnt(4) or mcnt(3) or mcnt(2) or mcnt(1)))
                                   and (not (dcnt(4) or dcnt(3) or dcnt(2) or dcnt(1)));
            if sq_rdy = '1' then
                pc <= pc + 1;
                case to_integer(pc) is
                    when 0 =>
                        if sq_go = '0' then pc <= (others => '0'); else sq_go <= '0'; end if;
                    when 1 =>
                        v_mul := '1'; v_ma := resize(k_mag1, 18); v_mb := resize(k_mag1, 18);
                    when 2 =>
                        -- K1 left (neg) = tunnel: read further out, s > 1
                        if k_neg1 = '1' then
                            s_s <= to_unsigned(16384, 15) + resize(macc(17 downto 6), 15);
                        else
                            s_s <= to_unsigned(16384, 15) - resize(macc(17 downto 6), 15);
                        end if;
                        v_mul := '1'; v_ma := resize(k_mag2, 18); v_mb := resize(k_mag2, 18);
                    when 3 =>
                        th <= macc(17 downto 10);                 -- 0..240 x (90/256 deg)
                        rom_a <= not macc(17 downto 10);          -- cos(th) ~ sin(255 - th)
                        v_mul := '1'; v_ma := resize(k_mag3, 18); v_mb := resize(k_mag3, 18);
                    when 4 =>
                        ph <= macc(17 downto 10);
                        v_mul := '1'; v_ma := resize(s_s, 18); v_mb := resize(rom_f, 18);
                    when 5 =>
                        a_q <= macc(31 downto 14);
                        rom_a <= th;
                        v_mul := '1'; v_ma := resize(k_t, 18); v_mb := resize(k_t, 18);
                    when 6 =>
                        if k_lo = '1' then
                            c_g <= to_unsigned(1024, 11) - resize(macc(19 downto 10), 11);
                        else
                            c_g <= to_unsigned(1024, 11) + shift_left(resize(k_over, 11), 1);
                        end if;
                        v_mul := '1'; v_ma := resize(s_s, 18); v_mb := resize(rom_f, 18);
                    when 7 =>
                        b_q <= macc(31 downto 14);
                        rom_a <= not ph;
                        v_div := '1'; v_ds := '0';                -- kx = 2^25 / W (Q18, 128 across)
                    when 8 =>
                        v_mul := '1'; v_ma := resize(c_g, 18); v_mb := resize(rom_f, 18);
                    when 9 =>
                        c_hc <= signed(resize(macc(24 downto 16), 10));   -- Q8, <= 383
                        rom_a <= ph;
                        v_div := '1'; v_ds := '1';                -- ky = 2^24 / H (64 rows)
                    when 10 =>
                        v_mul := '1'; v_ma := resize(c_g, 18); v_mb := resize(rom_f, 18);
                        v_aop := "01"; v_op := (others => '0');
                    when 11 =>
                        v_aop := '1' & k_neg3; v_op := signed(resize(macc(24 downto 16), 30));
                        v_mul := '1'; v_ma := a_q; v_mb := kx;
                    when 12 =>
                        c_hs <= acc(9 downto 0);
                        c_dudx <= signed(resize(macc(31 downto 14), 20));
                        v_aop := "01"; v_op := (others => '0');
                        v_mul := '1'; v_ma := b_q; v_mb := kx;
                    when 13 =>
                        -- b kx times rho (Q14): non-square field lines (v0.1.3)
                        v_mul := '1'; v_ma := macc(31 downto 14); v_mb := c_rx;
                    when 14 =>
                        -- dudy = -b kx rho (b >= 0 when K2 is right of centre)
                        v_aop := '1' & (not k_neg2); v_op := signed(resize(macc(31 downto 14), 30));
                        v_mul := '1'; v_ma := macc(31 downto 14); v_mb := resize(g_h(11 downto 1), 18);
                    when 15 =>
                        c_dudy <= acc(19 downto 0);
                        v_aop := "01"; v_op := to_signed(2**24, 30);
                    -- u0
                    when 16 => v_aop := "11"; v_op := signed(shift_left(resize(a_q, 30), 10));
                    when 17 => v_aop := '1' & k_neg2; v_op := signed(resize(macc(27 downto 0), 30));
                    -- (C_LOOK + 1/2) dudx = 8.5 dudx (steps 18-20 encode C_LOOK = 8)
                    when 18 => v_aop := "10"; v_op := shift_left(resize(c_dudx, 30), 3);
                    when 19 => null;
                    when 20 => v_aop := "10"; v_op := shift_right(resize(c_dudx, 30), 1);
                    when 21 => v_aop := "10"; v_op := shift_right(resize(c_dudy, 30), 1);
                    when 22 =>
                        c_u0 <= acc(27 downto 0);
                        v_aop := "01"; v_op := (others => '0');
                        v_mul := '1'; v_ma := b_q; v_mb := ky;
                    when 23 =>
                        -- b ky over rho (Q14)
                        v_mul := '1'; v_ma := macc(31 downto 14); v_mb := c_ry;
                    when 24 =>
                        -- dvdx = b ky / rho
                        v_aop := '1' & k_neg2; v_op := signed(resize(macc(31 downto 14), 30));
                        v_mul := '1'; v_ma := macc(31 downto 14); v_mb := resize(g_w(11 downto 1), 18);
                    when 25 =>
                        c_dvdx <= acc(19 downto 0);
                        v_aop := "01"; v_op := to_signed(2**23, 30);
                    -- v0
                    when 26 => v_aop := "11"; v_op := signed(shift_left(resize(a_q, 30), 9));
                    when 27 => v_aop := '1' & (not k_neg2); v_op := signed(resize(macc(27 downto 0), 30));
                    -- (C_LOOK + 1/2) dvdx = 8.5 dvdx
                    when 28 => v_aop := "10"; v_op := shift_left(resize(c_dvdx, 30), 3);
                    when 29 => null;
                    when 30 =>
                        v_aop := "10"; v_op := shift_right(resize(c_dvdx, 30), 1);
                        v_mul := '1'; v_ma := a_q; v_mb := ky;
                    when 31 =>
                        c_dvdy <= signed(resize(macc(31 downto 14), 20));
                        v_aop := "10"; v_op := signed(resize(macc(31 downto 15), 30));   -- + dvdy/2
                        v_mul := '1'; v_ma := resize(k_mag4, 18); v_mb := resize(k_mag4, 18);
                    when 32 =>
                        -- drift (Q18 cells, <= 0.94/gen); K4 right = rise
                        v_aop := '1' & k_neg4; v_op := signed(resize(macc(17 downto 0), 30));
                    when 33 =>
                        c_v0 <= acc(27 downto 0);
                        -- luma read offset: -0.5 cell + IIR trim (2^k - 0.5) kx - 2^17,
                        -- +1/32 cell weight rounding, +2^7 for the Q18 -> Q10 slice.
                        -- SD (v0.1.3): 720 / 128 = 5.625 px cells, so the cell-end
                        -- capture lands on average 7/16 px late (0 at 1920 / 1280):
                        -- -7/16 px = -7/16 kx (2^25/720) or echoes creep left
                        -- 720p (v0.1.3): the IIR alternates shift 3 / 2 per pixel
                        -- (a 1080p pixel's decay over 1.5 px); its group delay at
                        -- the (even) capture pixels is 4.73 px = 3.5 kx + 32269
                        v_aop := "01";
                        if c_sd = '1' then    v_op := to_signed(-2**18 + 2**13 + 2**7 - 20389, 30);
                        elsif g_alt = '1' then v_op := to_signed(-2**18 + 2**13 + 2**7 + 32269, 30);
                        else                  v_op := to_signed(-2**18 + 2**13 + 2**7, 30); end if;
                    when 34 =>
                        v_aop := "10";
                        if g_k3 = '1' and g_alt = '0' then v_op := signed(shift_left(resize(kx, 30), 3));
                        else               v_op := signed(shift_left(resize(kx, 30), 2)); end if;
                    when 35 => v_aop := "11"; v_op := signed(resize(kx(17 downto 1), 30));
                    when 36 =>
                        c_cy <= acc(27 downto 8);
                    when 37 =>
                        -- chroma: -1 Y cell + IIR trim (2^(k+1) - 0.5) kx - 2^18, +1/16;
                        -- SD: chroma captures (even cells) are 3/8 px late on average
                        -- 720p: chroma alternates shift 4 / 3, delay 10.07 px = 7.5 kx + 67239
                        v_aop := "01";
                        if c_sd = '1' then    v_op := to_signed(-2**19 + 2**14 + 2**7 - 17476, 30);
                        elsif g_alt = '1' then v_op := to_signed(-2**19 + 2**14 + 2**7 + 67239, 30);
                        else                  v_op := to_signed(-2**19 + 2**14 + 2**7, 30); end if;
                    when 38 =>
                        v_aop := "10";
                        if g_k3 = '1' and g_alt = '0' then v_op := signed(shift_left(resize(kx, 30), 4));
                        else               v_op := signed(shift_left(resize(kx, 30), 3)); end if;
                    when 39 => v_aop := "11"; v_op := signed(resize(kx(17 downto 1), 30));
                    when 40 =>
                        c_cc <= acc(27 downto 8);
                        if k_mag3 /= 0 then c_hue <= '1'; else c_hue <= '0'; end if;
                        commit <= '1';
                        pc <= (others => '0');
                    when others =>
                        pc <= (others => '0');
                end case;
            end if;

            if v_mul = '1' then
                ma <= v_ma; mb <= v_mb;
                macc <= (others => '0'); mcnt <= to_unsigned(18, 5);
            end if;
            if v_div = '1' then
                dsel <= v_ds;
                drem <= (others => '0');
                if v_ds = '0' then kx <= (others => '0'); else ky <= (others => '0'); end if;
                dcnt <= to_unsigned(26, 5);
            end if;
            -- ALU on one carry chain: load = add to a masked accumulator,
            -- subtract = add of the one's complement with carry-in 1
            if alu_ctl /= "00" then
                if alu_ctl = "01" then v_acc := (others => '0'); else v_acc := acc; end if;
                if alu_ctl = "11" then v_opx := not alu_op; else v_opx := alu_op; end if;
                acc <= v_acc + v_opx + signed'("0" & (alu_ctl(1) and alu_ctl(0)));
            end if;
            alu_ctl <= v_aop;
            alu_op  <= v_op;
        end if;
    end process p_seq;

    --------------------------------------------------------------------------
    -- R0: read DDA.  Holds pixel C_LOOK's coordinate while avid is low and
    -- steps once per input pixel, so after edge t of a line it holds pixel
    -- t + C_LOOK.  Line starts step by (dudy, dvdy) at each active-video fall.
    --------------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if commit = '1' then
                u_line <= c_u0;
                v_line <= c_v0;
            elsif data_in.avid = '0' and prev_avid = '1' then
                u_line <= u_line + resize(c_dudy, 28);
                v_line <= v_line + resize(c_dvdy, 28);
            end if;
            if data_in.avid = '1' then
                u_acc <= u_acc + resize(c_dudx, 28);
                v_acc <= v_acc + resize(c_dvdx, 28);
            else
                u_acc <= u_line;
                v_acc <= v_line;
            end if;
        end if;
    end process p_dda;

    --------------------------------------------------------------------------
    -- R1..R11: mirror, offsets, addresses, EBR, bilinear, convert.
    --------------------------------------------------------------------------
    p_read : process(clk)
        variable v_u20 : signed(19 downto 0);
        variable v_iu, v_iv, v_iu1, v_iv1 : signed(9 downto 0);
        variable v_iuc, v_ivc, v_iuc1, v_ivc1 : signed(8 downto 0);
        variable v_colE, v_colO : unsigned(5 downto 0);
        variable v_rowE, v_rowO : unsigned(4 downto 0);
        variable v_ccolE, v_ccolO : unsigned(4 downto 0);
        variable v_crowE, v_crowO : unsigned(3 downto 0);
        variable v_rb : std_logic;
        variable v_lv, v_rv, v_tv, v_bv : std_logic;
        variable v_clv, v_crv, v_ctv, v_cbv : std_logic;
        variable v_ytl, v_ytr, v_ybl, v_ybr : std_logic_vector(3 downto 0);
        variable v_ctl, v_ctr, v_cbl, v_cbr : std_logic_vector(7 downto 0);
        variable v_lx, v_ty : std_logic;
        variable v_n : signed(4 downto 0);
        variable v_t10, v_b10 : signed(9 downto 0);
        variable v_yo : unsigned(10 downto 0);
        variable v_uo, v_vo : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            ---------------------------------------------------------------
            -- R1: Q18 -> Q10, mirror fold about the centre (u >= 64 -> 128 - u)
            ---------------------------------------------------------------
            v_u20 := u_acc(27 downto 8);
            if sw_mir = '1' and v_u20 >= to_signed(65536, 20) then
                r1_um <= to_signed(131072, 20) - v_u20;
            else
                r1_um <= v_u20;
            end if;
            r1_v <= v_acc(27 downto 8);

            ---------------------------------------------------------------
            -- R2: offsets (luma / chroma index coordinates, Y-cell units)
            ---------------------------------------------------------------
            r2_uy <= r1_um + c_cy;
            r2_uc <= r1_um + c_cc;
            r2_vy <= r1_v - to_signed(480, 20);    -- -0.5 cell + 1/32
            r2_vc <= r1_v - to_signed(960, 20);    -- -1 cell + 1/16

            ---------------------------------------------------------------
            -- R3: integer / fraction, sub-bank addresses, edge flags
            ---------------------------------------------------------------
            v_iu  := r2_uy(19 downto 10);
            v_iv  := r2_vy(19 downto 10);
            v_iuc := r2_uc(19 downto 11);
            v_ivc := r2_vc(19 downto 11);
            v_iu1 := v_iu + 1;  v_iv1 := v_iv + 1;
            v_iuc1 := v_iuc + 1; v_ivc1 := v_ivc + 1;

            v_colO := unsigned(v_iu(6 downto 1));
            if v_iu(0) = '1' then v_colE := v_colO + 1; else v_colE := v_colO; end if;
            v_rowO := unsigned(v_iv(5 downto 1));
            if v_iv(0) = '1' then v_rowE := v_rowO + 1; else v_rowE := v_rowO; end if;
            v_ccolO := unsigned(v_iuc(5 downto 1));
            if v_iuc(0) = '1' then v_ccolE := v_ccolO + 1; else v_ccolE := v_ccolO; end if;
            v_crowO := unsigned(v_ivc(4 downto 1));
            if v_ivc(0) = '1' then v_crowE := v_crowO + 1; else v_crowE := v_crowO; end if;

            v_rb := not par;
            ya00 <= v_rb & v_rowE & v_colE;
            ya10 <= v_rb & v_rowE & v_colO;
            ya01 <= v_rb & v_rowO & v_colE;
            ya11 <= v_rb & v_rowO & v_colO;
            ca00 <= v_rb & v_crowE & v_ccolE;
            ca10 <= v_rb & v_crowE & v_ccolO;
            ca01 <= v_rb & v_crowO & v_ccolE;
            ca11 <= v_rb & v_crowO & v_ccolO;

            if sw_wrap = '1' then
                v_lv := '1'; v_rv := '1'; v_tv := '1'; v_bv := '1';
                v_clv := '1'; v_crv := '1'; v_ctv := '1'; v_cbv := '1';
            else
                if v_iu(9 downto 7) = "000" then v_lv := '1'; else v_lv := '0'; end if;
                if v_iu1(9 downto 7) = "000" then v_rv := '1'; else v_rv := '0'; end if;
                if v_iv(9 downto 6) = "0000" then v_tv := '1'; else v_tv := '0'; end if;
                if v_iv1(9 downto 6) = "0000" then v_bv := '1'; else v_bv := '0'; end if;
                if v_iuc(8 downto 6) = "000" then v_clv := '1'; else v_clv := '0'; end if;
                if v_iuc1(8 downto 6) = "000" then v_crv := '1'; else v_crv := '0'; end if;
                if v_ivc(8 downto 5) = "0000" then v_ctv := '1'; else v_ctv := '0'; end if;
                if v_ivc1(8 downto 5) = "0000" then v_cbv := '1'; else v_cbv := '0'; end if;
            end if;
            sb3 <= v_iu(0) & v_iv(0) & std_logic_vector(r2_uy(9 downto 6)) & std_logic_vector(r2_vy(9 downto 6))
                 & v_lv & v_rv & v_tv & v_bv
                 & v_iuc(0) & v_ivc(0) & std_logic_vector(r2_uc(10 downto 7)) & std_logic_vector(r2_vc(10 downto 7))
                 & v_clv & v_crv & v_ctv & v_cbv;

            -- R4 = EBR read (p_ram).  R5 below is muxes only, so the EBR
            -- outputs feed it without a fabric re-register.
            sb4 <= sb3;

            ---------------------------------------------------------------
            -- R5: reorder taps (left = sub-bank iu(0), top = iv(0)),
            -- zero out-of-range taps, weights
            ---------------------------------------------------------------
            v_lx := sb4(27); v_ty := sb4(26);
            if v_lx = '0' and v_ty = '0' then
                v_ytl := q00; v_ytr := q10; v_ybl := q01; v_ybr := q11;
            elsif v_lx = '1' and v_ty = '0' then
                v_ytl := q10; v_ytr := q00; v_ybl := q11; v_ybr := q01;
            elsif v_lx = '0' and v_ty = '1' then
                v_ytl := q01; v_ytr := q11; v_ybl := q00; v_ybr := q10;
            else
                v_ytl := q11; v_ytr := q01; v_ybl := q10; v_ybr := q00;
            end if;
            if (sb4(17) and sb4(15)) = '1' then r6_ytl <= signed('0' & v_ytl); else r6_ytl <= (others => '0'); end if;
            if (sb4(16) and sb4(15)) = '1' then r6_ytr <= signed('0' & v_ytr); else r6_ytr <= (others => '0'); end if;
            if (sb4(17) and sb4(14)) = '1' then r6_ybl <= signed('0' & v_ybl); else r6_ybl <= (others => '0'); end if;
            if (sb4(16) and sb4(14)) = '1' then r6_ybr <= signed('0' & v_ybr); else r6_ybr <= (others => '0'); end if;

            v_lx := sb4(13); v_ty := sb4(12);
            if v_lx = '0' and v_ty = '0' then
                v_ctl := e00; v_ctr := e10; v_cbl := e01; v_cbr := e11;
            elsif v_lx = '1' and v_ty = '0' then
                v_ctl := e10; v_ctr := e00; v_cbl := e11; v_cbr := e01;
            elsif v_lx = '0' and v_ty = '1' then
                v_ctl := e01; v_ctr := e11; v_cbl := e00; v_cbr := e10;
            else
                v_ctl := e11; v_ctr := e01; v_cbl := e10; v_cbr := e00;
            end if;
            if (sb4(3) and sb4(1)) = '1' then
                r6_utl <= resize(signed(v_ctl(7 downto 4)), 5); r6_vtl <= resize(signed(v_ctl(3 downto 0)), 5);
            else
                r6_utl <= (others => '0'); r6_vtl <= (others => '0');
            end if;
            if (sb4(2) and sb4(1)) = '1' then
                r6_utr <= resize(signed(v_ctr(7 downto 4)), 5); r6_vtr <= resize(signed(v_ctr(3 downto 0)), 5);
            else
                r6_utr <= (others => '0'); r6_vtr <= (others => '0');
            end if;
            if (sb4(3) and sb4(0)) = '1' then
                r6_ubl <= resize(signed(v_cbl(7 downto 4)), 5); r6_vbl <= resize(signed(v_cbl(3 downto 0)), 5);
            else
                r6_ubl <= (others => '0'); r6_vbl <= (others => '0');
            end if;
            if (sb4(2) and sb4(0)) = '1' then
                r6_ubr <= resize(signed(v_cbr(7 downto 4)), 5); r6_vbr <= resize(signed(v_cbr(3 downto 0)), 5);
            else
                r6_ubr <= (others => '0'); r6_vbr <= (others => '0');
            end if;
            r6_wx  <= unsigned(sb4(25 downto 22));
            r6_wy  <= unsigned(sb4(21 downto 18));
            r6_wxc <= unsigned(sb4(11 downto 8));
            r6_wyc <= unsigned(sb4(7 downto 4));
            r6_nx <= sb4(25); r6_ny <= sb4(21); r6_nxc <= sb4(11); r6_nyc <= sb4(7);

            ---------------------------------------------------------------
            -- R6: horizontal differences; Blocks = nearest tap, flat
            ---------------------------------------------------------------
            if sw_blk = '1' then
                if r6_ny = '0' then
                    if r6_nx = '0' then v_n := r6_ytl; else v_n := r6_ytr; end if;
                else
                    if r6_nx = '0' then v_n := r6_ybl; else v_n := r6_ybr; end if;
                end if;
                r7_ytl <= v_n; r7_ybl <= v_n; r7_dyt <= (others => '0'); r7_dyb <= (others => '0');
                if r6_nyc = '0' then
                    if r6_nxc = '0' then v_n := r6_utl; else v_n := r6_utr; end if;
                else
                    if r6_nxc = '0' then v_n := r6_ubl; else v_n := r6_ubr; end if;
                end if;
                r7_utl <= v_n; r7_ubl <= v_n; r7_dut <= (others => '0'); r7_dub <= (others => '0');
                if r6_nyc = '0' then
                    if r6_nxc = '0' then v_n := r6_vtl; else v_n := r6_vtr; end if;
                else
                    if r6_nxc = '0' then v_n := r6_vbl; else v_n := r6_vbr; end if;
                end if;
                r7_vtl <= v_n; r7_vbl <= v_n; r7_dvt <= (others => '0'); r7_dvb <= (others => '0');
            else
                r7_ytl <= r6_ytl; r7_ybl <= r6_ybl;
                r7_dyt <= r6_ytr - r6_ytl; r7_dyb <= r6_ybr - r6_ybl;
                r7_utl <= r6_utl; r7_ubl <= r6_ubl;
                r7_dut <= r6_utr - r6_utl; r7_dub <= r6_ubr - r6_ubl;
                r7_vtl <= r6_vtl; r7_vbl <= r6_vbl;
                r7_dvt <= r6_vtr - r6_vtl; r7_dvb <= r6_vbr - r6_vbl;
            end if;
            r7_wx <= r6_wx; r7_wy <= r6_wy; r7_wxc <= r6_wxc; r7_wyc <= r6_wyc;

            ---------------------------------------------------------------
            -- R7: x weight products
            ---------------------------------------------------------------
            r8_ytl <= r7_ytl; r8_ybl <= r7_ybl;
            r8_utl <= r7_utl; r8_ubl <= r7_ubl;
            r8_vtl <= r7_vtl; r8_vbl <= r7_vbl;
            r8_pyt <= smul4(r7_dyt, r7_wx, 10);  r8_pyb <= smul4(r7_dyb, r7_wx, 10);
            r8_put <= smul4(r7_dut, r7_wxc, 10); r8_pub <= smul4(r7_dub, r7_wxc, 10);
            r8_pvt <= smul4(r7_dvt, r7_wxc, 10); r8_pvb <= smul4(r7_dvb, r7_wxc, 10);
            r8_wy <= r7_wy; r8_wyc <= r7_wyc;

            ---------------------------------------------------------------
            -- R8: top / bottom rows (x16) and their difference
            ---------------------------------------------------------------
            v_t10 := shift_left(resize(r8_ytl, 10), 4) + r8_pyt;
            v_b10 := shift_left(resize(r8_ybl, 10), 4) + r8_pyb;
            r10_yt <= v_t10; r10_dy <= resize(v_b10, 11) - resize(v_t10, 11);
            v_t10 := shift_left(resize(r8_utl, 10), 4) + r8_put;
            v_b10 := shift_left(resize(r8_ubl, 10), 4) + r8_pub;
            r10_ut <= v_t10; r10_du <= resize(v_b10, 11) - resize(v_t10, 11);
            v_t10 := shift_left(resize(r8_vtl, 10), 4) + r8_pvt;
            v_b10 := shift_left(resize(r8_vbl, 10), 4) + r8_pvb;
            r10_vt <= v_t10; r10_dv <= resize(v_b10, 11) - resize(v_t10, 11);
            r10_wy <= r8_wy; r10_wyc <= r8_wyc;

            ---------------------------------------------------------------
            -- R9: y weight products
            ---------------------------------------------------------------
            r11_yt <= r10_yt; r11_ut <= r10_ut; r11_vt <= r10_vt;
            -- |dy| <= 240 (rows are convex blends of 0..240 / -128..112), w <= 15:
            -- the products fit 13 bits
            r11_py <= resize(smul4(r10_dy, r10_wy, 13), 15);
            r11_pu <= resize(smul4(r10_du, r10_wyc, 13), 15);
            r11_pv <= resize(smul4(r10_dv, r10_wyc, 13), 15);

            ---------------------------------------------------------------
            -- R10: interpolated value x256
            ---------------------------------------------------------------
            r12_y <= shift_left(resize(r11_yt, 15), 4) + r11_py;
            r12_u <= shift_left(resize(r11_ut, 15), 4) + r11_pu;
            r12_v <= shift_left(resize(r11_vt, 15), 4) + r11_pv;

            ---------------------------------------------------------------
            -- R11: to 10-bit video; level L -> 64 + 64 L (Y), 512 + 64 L (C).
            -- Chroma is stored canonical; swap on rows whose hsync->avid
            -- parity is odd.
            ---------------------------------------------------------------
            if r12_y < 0 then
                v_yo := to_unsigned(64, 11);
            else
                v_yo := resize(unsigned(r12_y(13 downto 2)), 11) + 64;
            end if;
            if v_yo > 960 then v_yo := to_unsigned(960, 11); end if;
            fb_y <= v_yo(9 downto 0);
            v_uo := resize(r12_u(14 downto 2), 12) + 512;
            v_vo := resize(r12_v(14 downto 2), 12) + 512;
            if line_par = '1' then
                fb_u <= unsigned(v_vo(9 downto 0)); fb_v <= unsigned(v_uo(9 downto 0));
            else
                fb_u <= unsigned(v_uo(9 downto 0)); fb_v <= unsigned(v_vo(9 downto 0));
            end if;
        end if;
    end process p_read;

    --------------------------------------------------------------------------
    -- Store: 8 arrays, each one registered read + one write statement.
    --------------------------------------------------------------------------
    --------------------------------------------------------------------------
    -- Luma gain table: refilled after each commit (entry y = acc >> 10 with
    -- acc = 512 + y * c_g, one add per clock); read by the luma chain at l2.
    --------------------------------------------------------------------------
    p_gtab : process(clk)
    begin
        if rising_edge(clk) then
            g_we <= '0';
            if commit = '1' then
                g_run <= '1';
                g_idx <= (others => '0');
                g_acc <= to_unsigned(512, 20);
            elsif g_run = '1' then
                g_we  <= '1';
                g_wa  <= g_idx;
                g_wd  <= g_acc(19 downto 10);
                g_acc <= g_acc + resize(c_g, 20);
                g_idx <= g_idx + 1;
                if g_idx = 255 then
                    g_run <= '0';
                end if;
            end if;
            if g_we = '1' then
                g_tab(to_integer(g_wa)) <= "000000" & std_logic_vector(g_wd);
            end if;
            g_rd <= g_tab(to_integer(l1_y));
        end if;
    end process p_gtab;

    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            q00 <= yr00(to_integer(ya00));
            q10 <= yr10(to_integer(ya10));
            q01 <= yr01(to_integer(ya01));
            q11 <= yr11(to_integer(ya11));
            e00 <= cr00(to_integer(ca00));
            e10 <= cr10(to_integer(ca10));
            e01 <= cr01(to_integer(ca01));
            e11 <= cr11(to_integer(ca11));

            if wy_v = '1' and wy_px = '0' and wy_py = '0' then yr00(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '1' and wy_py = '0' then yr10(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '0' and wy_py = '1' then yr01(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '1' and wy_py = '1' then yr11(to_integer(wy_a)) <= wy_d; end if;
            if wc_v = '1' and wc_px = '0' and wc_py = '0' then cr00(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '1' and wc_py = '0' then cr10(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '0' and wc_py = '1' then cr01(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '1' and wc_py = '1' then cr11(to_integer(wc_a)) <= wc_d; end if;
        end if;
    end process p_ram;

    --------------------------------------------------------------------------
    -- Mixer and output.  Live video is only 3 registers deep at X1.
    --   Key / Mix : out = fb + (live - fb) * c/32   (c = soft key / K5)
    --   Add       : out = fb + (live - black) * c/32
    --   Lighten   : out = black + (live - black) * c/32 where live is
    --               brighter than fb, else fb
    -- All four are base + addend through one adder per channel.
    --------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_q : signed(12 downto 0);
        variable v_c : std_logic;
        variable v_ay, v_au, v_av : signed(12 downto 0);
        variable v_lb : std_logic;
        variable v_lz : std_logic;
        variable v_ysum : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to C_PIPE - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- K1/K2: soft luma key alpha 0..31 over a 128-code ramp (k=2, 3)
            k1_d <= signed(resize(unsigned(pipe(0).y), 12)) - k_thr;
            v_d := shift_right(k1_d, 2);
            if v_d < 0 then
                k2_a <= (others => '0');
            elsif v_d > 31 then
                k2_a <= to_unsigned(31, 5);
            else
                k2_a <= unsigned(v_d(4 downto 0));
            end if;

            -- X1 (k=4): operands
            if k_mode(1) = '0' then
                x1_dy <= signed(resize(unsigned(pipe(2).y), 11)) - signed(resize(fb_y, 11));
                x1_du <= signed(resize(unsigned(pipe(2).u), 11)) - signed(resize(fb_u, 11));
                x1_dv <= signed(resize(unsigned(pipe(2).v), 11)) - signed(resize(fb_v, 11));
            else
                x1_dy <= signed(resize(unsigned(pipe(2).y), 11)) - 64;
                x1_du <= signed(resize(unsigned(pipe(2).u), 11)) - 512;
                x1_dv <= signed(resize(unsigned(pipe(2).v), 11)) - 512;
            end if;
            if k_mode = "00" then x1_c <= k2_a; else x1_c <= k_m5; end if;
            if unsigned(pipe(2).y) > fb_y then x1_lt <= '1'; else x1_lt <= '0'; end if;
            x1_fy <= fb_y; x1_fu <= fb_u; x1_fv <= fb_v;

            -- X2 (k=5): products
            x2_py <= smul5(x1_dy, x1_c);
            x2_pu <= smul5(x1_du, x1_c);
            x2_pv <= smul5(x1_dv, x1_c);
            x2_fy <= x1_fy; x2_fu <= x1_fu; x2_fv <= x1_fv;
            x2_lt <= x1_lt;

            -- X3 (k=6): base + rounded addend (the mixer is inside the loop:
            -- a floor here would pump a drift every generation).
            -- floor((p + 16) / 32) = (p >> 5) + p(4): one adder, carry-in.
            if k_mode = "11" and x2_lt = '1' then v_lb := '1'; else v_lb := '0'; end if;
            if k_mode = "11" and x2_lt = '0' then v_lz := '1'; else v_lz := '0'; end if;
            v_q := resize(shift_right(x2_py, 5), 13); v_c := x2_py(4);
            if v_lz = '1' then v_q := (others => '0'); v_c := '0'; end if;
            if v_lb = '1' then x3_ay <= to_signed(64, 13) + v_q + signed'("0" & v_c);
            else               x3_ay <= signed(resize(x2_fy, 13)) + v_q + signed'("0" & v_c); end if;
            v_q := resize(shift_right(x2_pu, 5), 13); v_c := x2_pu(4);
            if v_lz = '1' then v_q := (others => '0'); v_c := '0'; end if;
            if v_lb = '1' then x3_au <= to_signed(512, 13) + v_q + signed'("0" & v_c);
            else               x3_au <= signed(resize(x2_fu, 13)) + v_q + signed'("0" & v_c); end if;
            v_q := resize(shift_right(x2_pv, 5), 13); v_c := x2_pv(4);
            if v_lz = '1' then v_q := (others => '0'); v_c := '0'; end if;
            if v_lb = '1' then x3_av <= to_signed(512, 13) + v_q + signed'("0" & v_c);
            else               x3_av <= signed(resize(x2_fv, 13)) + v_q + signed'("0" & v_c); end if;

            -- X4 (k=7): clamp
            v_ay := x3_ay; v_au := x3_au; v_av := x3_av;
            if v_ay < 64 then v_ay := to_signed(64, 13); elsif v_ay > 1019 then v_ay := to_signed(1019, 13); end if;
            if v_au < 64 then v_au := to_signed(64, 13); elsif v_au > 960 then v_au := to_signed(960, 13); end if;
            if v_av < 64 then v_av := to_signed(64, 13); elsif v_av > 960 then v_av := to_signed(960, 13); end if;
            x4_ay <= unsigned(v_ay(9 downto 0)); x4_au <= unsigned(v_au(9 downto 0)); x4_av <= unsigned(v_av(9 downto 0));

            -- X5 (k=8): blanking gate (avid of pipe(6) is this pixel)
            if pipe(6).avid = '0' then
                x5_y <= C_BLKY; x5_u <= C_MID; x5_v <= C_MID; x5_bl <= '1';
            else
                x5_y <= x4_ay; x5_u <= x4_au; x5_v <= x4_av; x5_bl <= '0';
            end if;

            -- k=9: centre tap; k=10: luma [1 2 1]/4, re-gated on blanking
            d6_y <= x5_y; d6_u <= x5_u; d6_v <= x5_v; d6_bl <= x5_bl;
            d7_y <= d6_y;
            if d6_bl = '1' then
                s_io.y <= std_logic_vector(C_BLKY);
            else
                v_ysum := resize(x5_y, 12) + (resize(d6_y, 12) sll 1) + resize(d7_y, 12) + 2;
                s_io.y <= std_logic_vector(v_ysum(11 downto 2));
            end if;
            s_io.u <= std_logic_vector(d6_u);
            s_io.v <= std_logic_vector(d6_v);
            s_io.hsync_n <= pipe(C_PIPE - 1).hsync_n;
            s_io.vsync_n <= pipe(C_PIPE - 1).vsync_n;
            s_io.avid    <= pipe(C_PIPE - 1).avid;
            s_io.field_n <= pipe(C_PIPE - 1).field_n;
        end if;
    end process p_mix;

    s_io_align_in.y       <= s_io.y;
    s_io_align_in.u       <= s_io.u;
    s_io_align_in.v       <= s_io.v;
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.field_n <= s_io.field_n;

    --------------------------------------------------------------------------
    -- Write side.  The composite at k=8 (x5, avid = pipe(7)) is averaged by
    -- a one-pole IIR along the line (k = 3 or 2 for luma, +1 for chroma;
    -- 4 fraction bits, rounded) and captured at each cell end.  Luma rows
    -- are sampled on the line holding the row's middle, chroma rows on the
    -- line crossing from an even into an odd luma row (chroma row centre).
    -- Luma and chroma are captured on different lines, so the two chains
    -- never write together.  Dither = free-running LFSR bits.
    --------------------------------------------------------------------------
    p_write : process(clk)
        variable v_av : std_logic;
        variable v_x  : unsigned(12 downto 0);
        variable v_y  : unsigned(12 downto 0);
        variable v_ty, v_tu, v_tv : signed(14 downto 0);
        variable v_ny, v_nu, v_nv : signed(14 downto 0);
        variable v_pulse : std_logic;
        variable v_w15 : unsigned(14 downto 0);
        variable v_q : unsigned(8 downto 0);
        variable v_t : signed(11 downto 0);
        variable v_s9 : signed(9 downto 0);
        variable v_u8, v_v8 : signed(7 downto 0);
        variable v_cu, v_cv : unsigned(9 downto 0);
        variable v_cr : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            v_av := pipe(7).avid;
            v_pulse := '0';

            -- IIR, split so each clock is one carry chain:
            --   e   <= x*16/2^k - acc/2^k     acc <= acc + e
            -- = acc' = acc + a x[n-1] - a acc[n-1] (a = 2^-k): same DC gain and
            -- the same DC group delay (8 px from the capture edge, = the plain
            -- one-pole read before the pixel), real stable poles for k = 2..4.
            -- Luma k, chroma k+1.  720p (g_alt): k alternates 3 / 2 per pixel
            -- from 3 on pixel 1, so its decay per pixel is close to 1080p's
            -- over 1.5 px (the store cells are 10 px instead of 15).
            if w_ks = '1' then
                v_ty := signed(resize(shift_left(resize(x5_y, 14), 1), 15)) - signed(resize(shift_right(w_yi, 3), 15));
                v_tu := signed(resize(x5_u, 15)) - signed(resize(shift_right(w_ui, 4), 15));
                v_tv := signed(resize(x5_v, 15)) - signed(resize(shift_right(w_vi, 4), 15));
            else
                v_ty := signed(resize(shift_left(resize(x5_y, 14), 2), 15)) - signed(resize(shift_right(w_yi, 2), 15));
                v_tu := signed(resize(shift_left(resize(x5_u, 14), 1), 15)) - signed(resize(shift_right(w_ui, 3), 15));
                v_tv := signed(resize(shift_left(resize(x5_v, 14), 1), 15)) - signed(resize(shift_right(w_vi, 3), 15));
            end if;
            v_ny := signed(resize(w_yi, 15)) + w_ey;
            v_nu := signed(resize(w_ui, 15)) + w_eu;
            v_nv := signed(resize(w_vi, 15)) + w_ev;

            if v_av = '1' and w_prev = '0' then
                -- first pixel of the line
                w_x  <= (others => '0');
                w_c  <= (others => '0');
                w_yi <= shift_left(resize(x5_y, 14), 4);
                w_ui <= shift_left(resize(x5_u, 14), 4);
                w_vi <= shift_left(resize(x5_v, 14), 4);
                w_ey <= (others => '0'); w_eu <= (others => '0'); w_ev <= (others => '0');
                w_lsl <= w_lsn;
                w_csl <= w_csn;
                w_ks  <= g_k3;
                w_rl  <= w_r;
                w_par <= line_par;
            elsif v_av = '1' then
                v_x := w_x + 128;
                if v_x >= resize(g_w, 13) then
                    w_x <= v_x - resize(g_w, 13);
                    if w_c /= 127 then
                        v_pulse := '1';
                        w_c <= w_c + 1;
                    end if;
                else
                    w_x <= v_x;
                end if;
                w_yi <= unsigned(v_ny(13 downto 0));
                w_ui <= unsigned(v_nu(13 downto 0));
                w_vi <= unsigned(v_nv(13 downto 0));
                w_ey <= v_ty; w_eu <= v_tu; w_ev <= v_tv;
                if g_alt = '1' then w_ks <= not w_ks; else w_ks <= g_k3; end if;
            elsif w_prev = '1' then
                -- line end: last cell, row DDA
                v_pulse := '1';
                v_y := resize(w_yacc, 13) + 64;
                if v_y >= resize(g_h, 13) then
                    w_yacc <= resize(v_y - resize(g_h, 13), 12);
                    if w_r /= 63 then w_r <= w_r + 1; end if;
                else
                    w_yacc <= v_y(11 downto 0);
                end if;
            end if;
            w_prev <= v_av;

            if anchor = '1' then
                w_yacc <= (others => '0');
                w_r    <= (others => '0');
            end if;

            -- next-line flags from the row DDA state
            if resize(w_yacc, 13) <= resize(g_h(11 downto 1), 13)
               and resize(w_yacc, 13) + 64 > resize(g_h(11 downto 1), 13) then
                w_lsn <= '1';
            else
                w_lsn <= '0';
            end if;
            if resize(w_yacc, 13) + 64 >= resize(g_h, 13) and w_r(0) = '0' then
                w_csn <= '1';
            else
                w_csn <= '0';
            end if;

            -----------------------------------------------------------------
            -- L1 (on the cell-end pulse): luma in 4-code units above black
            -- (0..239), chroma canonical signed in 4-code units
            -----------------------------------------------------------------
            if v_pulse = '1' then
                l1_vy <= w_lsl;
                l1_vc <= w_csl and w_c(0);
            else
                l1_vy <= '0';
                l1_vc <= '0';
            end if;
            l1_c <= w_c; l1_r <= w_rl;
            v_w15 := shift_right(resize(w_yi, 15) + 32, 6);
            v_q := v_w15(8 downto 0);
            if v_q < 16 then l1_y <= (others => '0');
            elsif v_q > 255 then l1_y <= (others => '1');
            else l1_y <= resize(v_q - 16, 8); end if;
            v_w15 := shift_right(resize(w_ui, 15) + 32, 6);
            v_cu := v_w15(9 downto 0);
            v_w15 := shift_right(resize(w_vi, 15) + 32, 6);
            v_cv := v_w15(9 downto 0);
            if w_par = '1' then
                l1_u <= resize(signed(v_cv) - 128, 9);
                l1_v <= resize(signed(v_cu) - 128, 9);
            else
                l1_u <= resize(signed(v_cu) - 128, 9);
                l1_v <= resize(signed(v_cv) - 128, 9);
            end if;

            -----------------------------------------------------------------
            -- Luma chain: gain, crisp, fold, negative, stochastic rounding
            -----------------------------------------------------------------
            l2_vy <= l1_vy; l2_c <= l1_c; l2_r <= l1_r;
            -- (gain from the table: g_rd is the l2 stage)

            l3_vy <= l2_vy; l3_c <= l2_c; l3_r <= l2_r;
            l3_y <= unsigned(g_rd(9 downto 0));

            l4_vy <= l3_vy; l4_c <= l3_c; l4_r <= l3_r;
            if sw_crisp = '1' then
                v_t := signed(resize(l3_y, 12)) - 120;
                l4_y <= signed(resize(l3_y, 12)) + shift_right(v_t + 1, 1);
            else
                l4_y <= signed(resize(l3_y, 12));
            end if;

            -- wave-fold into 0..239 (overdrive turns into rings, not white)
            l5_vy <= l4_vy; l5_c <= l4_c; l5_r <= l4_r;
            if l4_y > 239 then
                l5_y <= to_signed(478, 12) - l4_y;
            elsif l4_y < 0 then
                l5_y <= -l4_y;
            else
                l5_y <= l4_y;
            end if;

            l6_vy <= l5_vy; l6_c <= l5_c; l6_r <= l5_r;
            if l5_y < 0 then v_q := (others => '0');
            elsif l5_y > 239 then v_q := to_unsigned(239, 9);
            else v_q := unsigned(l5_y(8 downto 0)); end if;
            if sw_neg = '1' then
                l6_y <= resize(to_unsigned(239, 9) - v_q, 8);
            else
                l6_y <= v_q(7 downto 0);
            end if;

            wy_v <= l6_vy;
            v_q := resize(l6_y, 9) + resize(lfsr(3 downto 0), 9);
            wy_d  <= std_logic_vector(v_q(7 downto 4));
            wy_px <= l6_c(0); wy_py <= l6_r(0);
            wy_a  <= par & l6_r(5 downto 1) & l6_c(6 downto 1);

            -----------------------------------------------------------------
            -- Chroma chain: hue rotation U' = U hc - V hs, V' = U hs + V hc
            -- on one multiplier over phases 1..6 (chroma cells are >= 10
            -- clocks apart), then bloom, clamp, negative, rounding.
            -----------------------------------------------------------------
            cm_p <= booth9x10(cm_a, cm_b);
            c8_v <= '0';
            case cs_ph is
                when "000" =>
                    if l1_vc = '1' then
                        cs_u <= l1_u; cs_v <= l1_v; cs_c <= l1_c; cs_r <= l1_r;
                        if c_hue = '1' then cs_inj <= l1_y(7 downto 3); else cs_inj <= (others => '0'); end if;
                        cs_ph <= "001";
                    end if;
                when "001" =>
                    cm_a <= cs_u; cm_b <= c_hc; cs_ph <= "010";
                when "010" =>
                    cm_a <= cs_v; cm_b <= c_hs; cs_ph <= "011";
                when "011" =>
                    cm_a <= cs_u; cm_b <= c_hs; acc_u <= resize(cm_p, 20); cs_ph <= "100";
                when "100" =>
                    cm_a <= cs_v; cm_b <= c_hc; acc_u <= acc_u - resize(cm_p, 20); cs_ph <= "101";
                when "101" =>
                    acc_v <= resize(cm_p, 20); cs_ph <= "110";
                when "110" =>
                    acc_v <= acc_v + resize(cm_p, 20); cs_ph <= "111";
                when others =>
                    c8_v <= '1';
                    c8_c <= cs_c; c8_r <= cs_r;
                    v_cr := shift_right(acc_u + 128, 8);
                    c8_u <= resize(v_cr, 12) + signed(resize(cs_inj, 12));
                    v_cr := shift_right(acc_v + 128, 8);
                    c8_v2 <= resize(v_cr, 12);
                    cs_ph <= "000";
            end case;

            c9_v <= c8_v; c9_c <= c8_c; c9_r <= c8_r;
            if c8_u > 112 then v_u8 := to_signed(112, 8);
            elsif c8_u < -112 then v_u8 := to_signed(-112, 8);
            else v_u8 := c8_u(7 downto 0); end if;
            if c8_v2 > 112 then v_v8 := to_signed(112, 8);
            elsif c8_v2 < -112 then v_v8 := to_signed(-112, 8);
            else v_v8 := c8_v2(7 downto 0); end if;
            if sw_neg = '1' then
                c9_u <= -v_u8; c9_v2 <= -v_v8;
            else
                c9_u <= v_u8; c9_v2 <= v_v8;
            end if;

            wc_v <= c9_v;
            v_s9 := resize(c9_u, 10) + signed(resize(lfsr(11 downto 8), 10));
            v_u8 := v_s9(7 downto 0);
            v_s9 := resize(c9_v2, 10) + signed(resize(lfsr(15 downto 12), 10));
            v_v8 := v_s9(7 downto 0);
            wc_d  <= std_logic_vector(v_u8(7 downto 4)) & std_logic_vector(v_v8(7 downto 4));
            wc_px <= c9_c(1); wc_py <= c9_r(1);
            wc_a  <= par & c9_r(5 downto 2) & c9_c(6 downto 2);
        end if;
    end process p_write;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (10 -> 12).
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

end architecture feedback;
