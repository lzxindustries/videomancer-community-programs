-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
-- File: fray.vhd - edge-launched scanline filaments
-- License: GNU General Public License v3.0
--
-- FRAY v0.4.2 -- every scanline frays to the right of its edges.
--
-- v0.4.5 (2026-10-17): resolution scaling.  Geometry runs in virtual
-- 1080-line units stepped from the video timing ID (registers_in(8)):
-- 1.0 at 1080p (identical to before), 1.5 at 720p, 2.0 across / 1080/486
-- per frame line at SD (a 1440-wide 4:3 window).  The K4 dash period
-- (phase step x the x step), the 16-unit hash column clumps, the K5 bundle
-- bands and the Fray weave phase (virtual frame line), and the run length
-- and tail taper (counted down in virtual units) keep their proportions at
-- every resolution (the period floors at 2 px, so K4's bottom ~30% sits
-- at that floor at SD).  Interlace now comes from the timing ID (field_n is
-- tied high on hardware, so the old field_n-toggle detector never fired).
-- The Sobel kernel, hold-colour tap and Sparks specks stay pixel-scale.
--
-- Where prism grows one even rainbow band off every edge, FRAY launches an
-- individual FILAMENT: a 1-px-tall run whose length, firing decision and
-- internal seed all come from a spatial hash of (line group, column group,
-- seed).  Neighbouring lines therefore get different filaments (ragged,
-- glitchy) unless K5 Bundle widens the line group, at which point runs
-- bundle into coherent sheaves.  Inside a run a 16-bit phase accumulator
-- (K4 Period) drives eight pattern engines, and eight colour engines decide
-- what the filament is made of -- including the video colour sampled at the
-- edge (pixel-stretch), boosted or inverted live video, and palettes.
--
-- The edges themselves are never drawn; they are only where filaments
-- begin.  Streaming pipeline: filaments can only extend to the RIGHT.
--
-- Pipeline (stage k signals are registered under av(k-1)):
--   1  luma 8-bit; line-buffer read presented; dry-video ring write
--   2  BRAM reads landed in FFs
--   3  Sobel column sums + two-column histories
--   4  gx / gy
--   5  magnitude, gx sign (falling = bright->dark going right)
--   6  normalize, warm-up guards
--   7  threshold + polarity gate -> edge bit.  Hash chain (run one column
--      ahead so the length multiply gets two stages) lands len / fire / run
--      seed here; dry video ring read lands here (vr)
--   8  run state machine (launch / count / phase)
--   9  pattern -> ink + intensity + palette index; tail taper level;
--      glow/negative video
--  10  intensity = min(pattern, taper); Heat palette index from it
--  11  base colour select + ground select
--  12  base - ground (signed)
--  13  x intensity, hi/lo partial products (multiplies alone)
--  14  partial sum
--  15  ground + product
--  16  negative, blanking gate
--  17  chroma [1 2 1] history (luma waits)
--  18  chroma low-pass, blanking gate, output register
--
-- EBR: 2 x 2048x8 luma line buffers (8) + 256x32 dry-video ring (2) = 10.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture fray of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (18 + 2 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 18;
    constant C_SEED : unsigned(15 downto 0) := x"3C5A";

    ----------------------------------------------------------------------------
    -- Palette ROM, 10-bit YUV with U/V already swapped for the hardware
    -- (the scope-verified prism table; U holds BT.601 Cr, V holds Cb).
    --   idx(4..3) = 0 : rainbow R O Y G B I V + white     (Rainbow, Tint)
    --   idx(4..3) = 1 : heat  black..deep red..yellow..white (Heat)
    --   idx(4..3) = 2 : polarity  cyan (rising) / red (falling)
    ----------------------------------------------------------------------------
    type t_pal is array (0 to 31) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(328,10), to_unsigned(584,10), to_unsigned(840,10), to_unsigned(580,10),
        to_unsigned(164,10), to_unsigned(192,10), to_unsigned(349,10), to_unsigned(768,10),
        to_unsigned( 64,10), to_unsigned(195,10), to_unsigned(328,10), to_unsigned(454,10),
        to_unsigned(584,10), to_unsigned(712,10), to_unsigned(840,10), to_unsigned(768,10),
        to_unsigned(678,10), to_unsigned(328,10), to_unsigned(678,10), to_unsigned(328,10),
        to_unsigned(678,10), to_unsigned(328,10), to_unsigned(678,10), to_unsigned(328,10),
        to_unsigned(768,10), to_unsigned(768,10), to_unsigned(768,10), to_unsigned(768,10),
        to_unsigned(768,10), to_unsigned(768,10), to_unsigned(768,10), to_unsigned(768,10));
    constant C_PAL_U : t_pal := (
        to_unsigned(960,10), to_unsigned(772,10), to_unsigned(584,10), to_unsigned(136,10),
        to_unsigned(440,10), to_unsigned(608,10), to_unsigned(756,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(736,10), to_unsigned(960,10), to_unsigned(866,10),
        to_unsigned(772,10), to_unsigned(679,10), to_unsigned(584,10), to_unsigned(512,10),
        to_unsigned( 64,10), to_unsigned(960,10), to_unsigned( 64,10), to_unsigned(960,10),
        to_unsigned( 64,10), to_unsigned(960,10), to_unsigned( 64,10), to_unsigned(960,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10));
    constant C_PAL_V : t_pal := (
        to_unsigned(360,10), to_unsigned(212,10), to_unsigned( 64,10), to_unsigned(216,10),
        to_unsigned(960,10), to_unsigned(696,10), to_unsigned(852,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(436,10), to_unsigned(360,10), to_unsigned(287,10),
        to_unsigned(212,10), to_unsigned(138,10), to_unsigned( 64,10), to_unsigned(512,10),
        to_unsigned(663,10), to_unsigned(360,10), to_unsigned(663,10), to_unsigned(360,10),
        to_unsigned(663,10), to_unsigned(360,10), to_unsigned(663,10), to_unsigned(360,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10));

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

    -- Pot deadband: follow the raw value only on a move of 2+ LSB (or at
    -- either end of travel), so ADC noise never reaches the glide
    -- (one subtract; |d| <= 1 iff d(10..1) is all-0 or all-1)
    function f_dead(raw, tgt : unsigned(9 downto 0)) return std_logic is
        variable v_d : unsigned(10 downto 0);
    begin
        v_d := resize(raw, 11) - resize(tgt, 11);
        if raw = 0 or raw = 1023 then
            return '1';
        elsif v_d(10 downto 1) = 0 or v_d(10 downto 1) = 1023 then
            return '0';
        else
            return '1';
        end if;
    end function;

    -- Glide (10.3 fixed point): ease 1/8 of the way per field, no snap
    function f_glide(tgt : unsigned(9 downto 0); g : unsigned(12 downto 0))
        return unsigned is
        variable v_t, v_g, v_d : signed(14 downto 0);
    begin
        v_t := signed(shift_left(resize(tgt, 15), 3));
        v_g := signed(resize(g, 15));
        v_d := v_t - v_g;
        v_d := v_g + shift_right(v_d, 3);
        return unsigned(v_d(12 downto 0));   -- slice, not resize (sign fold)
    end function;

    -- Tail taper: full level until the last T = 2^k pixels of the run, then
    -- a linear ramp down (remaining * 256 / T).  k is fixed per run at launch.
    function f_taper(rm : unsigned(11 downto 0); k : unsigned(3 downto 0))
        return unsigned is
        variable v_t : unsigned(8 downto 0);
    begin
        v_t := to_unsigned(256, 9);
        for i in 0 to 8 loop
            if to_integer(k) = i then
                if shift_right(rm, i) = 0 then
                    v_t := shift_left(resize(rm(7 downto 0), 9), 8 - i);
                end if;
            end if;
        end loop;
        return v_t;
    end function;

    -- Resolution scaling: per-mode steps from the video timing ID.
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

    -- Controls (registered, quasi-static)
    signal s_pat     : unsigned(2 downto 0) := (others => '0');  -- K1 pattern
    signal s_thr     : unsigned(7 downto 0) := (others => '0');  -- K2 threshold
    signal s_thr_r   : unsigned(7 downto 0) := (others => '0');  -- K2 raw, registered
    signal s_col     : unsigned(2 downto 0) := (others => '0');  -- K3 colour
    signal s_step    : unsigned(15 downto 0) := (others => '0'); -- K4 phase step
    signal s_step_x  : unsigned(16 downto 0) := (others => '0'); -- step x x step
    signal s_step_v  : unsigned(15 downto 0) := (others => '0'); -- per pixel, 2-px floor
    signal s_csh     : unsigned(2 downto 0) := (others => '0');  -- K5 bundle shift
    signal s_dens    : unsigned(7 downto 0) := (others => '0');  -- K6 density
    signal s_gnd_vid : std_logic := '0';   -- S7  ground = video
    signal s_boil    : std_logic := '0';   -- S8  reseed hash every 4th field
    signal s_fall    : std_logic := '0';   -- S9  falling edges only
    signal s_hold    : std_logic := '0';   -- S10 hold run through edges
    signal s_neg     : std_logic := '0';   -- S11 negative output
    signal s_k4_r, s_p12_r   : unsigned(9 downto 0) := (others => '0');  -- raw, registered
    signal s_k4_mv, s_p12_mv : std_logic := '0';   -- pot moved past the deadband
    signal s_k4_t    : unsigned(9 downto 0) := (others => '0');  -- K4 deadbanded
    signal s_k4_g    : unsigned(12 downto 0) := (others => '0'); -- K4 glided 10.3
    signal s_k4_a, s_k4_b, s_k4_sc : unsigned(15 downto 0) := (others => '0'); -- g * 2.797
    signal s_p12_t   : unsigned(9 downto 0) := (others => '0');  -- P12 deadbanded
    signal s_p12_g   : unsigned(12 downto 0) := (others => '0'); -- P12 glided 10.3
    signal s_lm_prod : unsigned(20 downto 0) := (others => '0');
    signal s_len_max : unsigned(11 downto 0) := to_unsigned(8, 12);

    -- Valid chain: av(i) = data_in.avid delayed i clocks
    signal av : std_logic_vector(1 to C_LAT - 1) := (others => '0');

    -- Line / column bookkeeping
    signal s_prev_hs  : std_logic := '1';
    signal s_hs_edge  : std_logic := '0';   -- registered hsync falling edge
    signal s_avid_d   : std_logic := '0';
    signal s_seen     : std_logic := '0';
    signal s_aline    : unsigned(10 downto 0) := (others => '0');
    signal s_fl       : unsigned(11 downto 0) := (others => '0'); -- frame line key
    signal s_nav      : unsigned(11 downto 0) := (others => '0'); -- clocks without avid
    signal s_nav11_d  : std_logic := '0';
    signal s_fev      : std_logic := '0';   -- once-per-field event pulse
    -- Resolution scaling (virtual 1080-line units, latched per field)
    signal s_tid      : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xs       : unsigned(2 downto 0)  := to_unsigned(2, 3);    -- Q.1 per pixel
    signal s_ys       : unsigned(11 downto 0) := to_unsigned(256, 12); -- Q4.8 per field line
    signal s_ya       : unsigned(18 downto 0) := (others => '0');      -- virtual line, Q.8
    signal s_wv       : unsigned(10 downto 0) := to_unsigned(720, 11); -- virtual width
    signal s_lg       : unsigned(11 downto 0) := (others => '0'); -- line group
    signal cx0        : unsigned(10 downto 0) := (others => '0'); -- column of data_in
    signal xa1        : unsigned(11 downto 0) := to_unsigned(2, 12); -- virtual x of cx0 + 1, Q.1 (hash look-ahead)
    signal s_width    : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_fcnt     : unsigned(1 downto 0) := (others => '0');
    signal s_seed     : unsigned(15 downto 0) := C_SEED;
    signal s_lfsr     : std_logic_vector(15 downto 0);
    signal s_lfsr_act : std_logic_vector(15 downto 0) := x"A5C3";

    -- Sobel path
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb1, lb2   : t_lb8 := (others => (others => '0'));
    signal q1, q2     : std_logic_vector(7 downto 0) := (others => '0');
    signal lrx, lwx   : unsigned(10 downto 0) := (others => '0');
    signal y8, y8d    : unsigned(7 downto 0) := (others => '0');
    signal qr1, qr2   : unsigned(7 downto 0) := (others => '0');
    signal ss, ss1, ss2 : unsigned(9 downto 0) := (others => '0');
    signal dy, dy1, dy2 : signed(8 downto 0) := (others => '0');
    signal s_gx, s_gy : signed(11 downto 0) := (others => '0');
    signal s_mag      : unsigned(11 downto 0) := (others => '0');
    signal s_mag8     : unsigned(7 downto 0) := (others => '0');
    signal fall5, fall6, fall7 : std_logic := '0';
    signal cbx        : unsigned(10 downto 0) := (others => '0');
    signal e7         : std_logic := '0';

    -- Hash chain (stages 1..7)
    signal h1_a   : unsigned(17 downto 0) := (others => '0');
    signal h2_b   : unsigned(17 downto 0) := (others => '0');
    signal h2_c1  : unsigned(23 downto 0) := (others => '0');
    signal h3_c   : unsigned(23 downto 0) := (others => '0');
    signal h4_d   : unsigned(23 downto 0) := (others => '0');
    signal h4_e1  : unsigned(23 downto 0) := (others => '0');
    signal h5_e   : unsigned(23 downto 0) := (others => '0');
    signal h6_f   : unsigned(23 downto 0) := (others => '0');
    signal lp_lo6, lp_hi6 : unsigned(15 downto 0) := (others => '0');
    signal fire6  : std_logic := '0';
    signal rh6    : unsigned(15 downto 0) := (others => '0');
    signal lp7    : unsigned(19 downto 0) := (others => '0');
    signal fire7  : std_logic := '0';
    signal rhash7 : unsigned(15 downto 0) := (others => '0');

    -- Dry-video ring (256 x 32 EBR) and video chain
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ring   : t_ring := (others => (others => '0'));
    signal wp     : unsigned(7 downto 0) := (others => '0');
    signal rq     : std_logic_vector(31 downto 0) := (others => '0');
    signal vr7, vr8, vr9, vr10 : std_logic_vector(29 downto 0) := (others => '0');

    -- Run state (stage 8)
    signal r_act  : std_logic := '0';
    signal r_rem  : unsigned(12 downto 0) := (others => '0');  -- virtual units, Q.1
    signal r_ph   : unsigned(15 downto 0) := (others => '0');
    signal r_seg  : unsigned(2 downto 0) := (others => '0');
    signal r_sl   : unsigned(15 downto 0) := (others => '0');
    signal r_hold : std_logic_vector(29 downto 0) := (others => '0');
    signal r_pol  : std_logic := '0';
    signal r_tint : unsigned(2 downto 0) := (others => '0');
    signal r_wrap : std_logic := '0';   -- phase wrapped on the previous pixel
    signal r_tk   : unsigned(3 downto 0) := (others => '0');  -- taper length log2

    -- Pattern stage (9)
    signal p_ink  : std_logic := '0';
    signal p_int  : unsigned(8 downto 0) := (others => '0');
    signal p_pidx : unsigned(4 downto 0) := (others => '0');
    signal hold9  : std_logic_vector(29 downto 0) := (others => '0');
    signal glow9  : std_logic_vector(29 downto 0) := (others => '0');
    signal neg9   : std_logic_vector(29 downto 0) := (others => '0');
    signal p_tap  : unsigned(8 downto 0) := (others => '0');

    -- Taper stage (10)
    signal t_ink  : std_logic := '0';
    signal t_int  : unsigned(8 downto 0) := (others => '0');
    signal t_pidx : unsigned(4 downto 0) := (others => '0');
    signal hold_t, glow_t, neg_t : std_logic_vector(29 downto 0) := (others => '0');

    -- Colour stages (10..13)
    signal b_y, b_u, b_v : unsigned(9 downto 0) := (others => '0');
    signal g10, g11, g12, g13 : std_logic_vector(29 downto 0) := (others => '0');
    signal ink10, ink11, ink12, ink13 : std_logic := '0';
    signal int10         : unsigned(8 downto 0) := (others => '0');
    -- one intensity copy per multiplier: a single int11 fanning out to all
    -- three lerp multipliers was the HD critical path (9 ns of routing)
    signal int11_y, int11_u, int11_v : unsigned(8 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of int11_y : signal is true;
    attribute keep of int11_u : signal is true;
    attribute keep of int11_v : signal is true;
    signal d_y, d_u, d_v : signed(10 downto 0) := (others => '0');
    signal pl_y, pl_u, pl_v : signed(16 downto 0) := (others => '0');  -- d * int(4..0)
    signal ph_y, ph_u, ph_v : signed(15 downto 0) := (others => '0');  -- d * int(8..5)
    signal pr_y, pr_u, pr_v : signed(20 downto 0) := (others => '0');
    signal o_y, o_u, o_v : unsigned(9 downto 0) := (others => '0');

    -- Output registers
    signal g_y, g_u, g_v : std_logic_vector(9 downto 0) := (others => '0');   -- stage 16
    signal h_y, h_u, h_v : std_logic_vector(9 downto 0) := (others => '0');   -- stage 17
    signal h2_u, h2_v    : std_logic_vector(9 downto 0) := (others => '0');
    signal s_y_out, s_u_out, s_v_out : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_sr    : std_logic_vector(C_LAT - 1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(C_LAT - 1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(C_LAT - 1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(C_LAT - 1 downto 0) := (others => '1');

begin

    ----------------------------------------------------------------------------
    -- Controls: decoded free-running, every value registered
    ----------------------------------------------------------------------------
    p_ctl : process(clk)
    begin
        if rising_edge(clk) then
            -- Labelled defaults (K1 Pattern, K3 Colour): at program load the
            -- firmware writes the initial_value_label's INDEX (0..7) into the
            -- register, not a 0..1023 position, so a value below 8 is the
            -- label index itself.
            if unsigned(registers_in(0)) < 8 then
                s_pat <= unsigned(registers_in(0)(2 downto 0));
            else
                s_pat <= unsigned(registers_in(0)(9 downto 7));
            end if;
            -- K2: full travel = 0 .. 5/8 of the old range (HW: above ~60%
            -- nothing fired any more); thr = raw8/2 + raw8/8, max 158
            s_thr_r <= unsigned(registers_in(1)(9 downto 2));
            s_thr   <= shift_right(s_thr_r, 1) + shift_right(s_thr_r, 3);
            if unsigned(registers_in(2)) < 8 then
                s_col <= unsigned(registers_in(2)(2 downto 0));
            else
                s_col <= unsigned(registers_in(2)(9 downto 7));
            end if;
            -- K4 / P12 accumulate along a run (a 1-LSB wobble moves the far
            -- end of a long filament by half a period): deadband, glide,
            -- and latch once per field so a change never tears mid-frame
            -- (quasi-static: register, decide, then load -- 3 clocks of lag)
            s_k4_r   <= unsigned(registers_in(3));
            s_p12_r  <= unsigned(registers_in(7));
            s_k4_mv  <= f_dead(s_k4_r, s_k4_t);
            s_p12_mv <= f_dead(s_p12_r, s_p12_t);
            if s_k4_mv = '1' then
                s_k4_t <= s_k4_r;
            end if;
            if s_p12_mv = '1' then
                s_p12_t <= s_p12_r;
            end if;
            s_csh   <= unsigned(registers_in(4)(9 downto 7));
            s_dens  <= unsigned(registers_in(5)(9 downto 2));
            s_gnd_vid <= registers_in(6)(0);
            s_boil    <= registers_in(6)(1);
            s_fall    <= registers_in(6)(2);
            s_hold    <= registers_in(6)(3);
            s_neg     <= registers_in(6)(4);
            -- max filament length = slider fraction of the measured width + 8,
            -- in virtual units (SD 1440, 720p/1080p 1920 wide)
            s_tid <= registers_in(8)(3 downto 0);
            case to_integer(s_xs) is
                when 4      => s_wv <= s_width(9 downto 0) & '0';
                when 3      => s_wv <= s_width + shift_right(s_width, 1);
                when others => s_wv <= s_width;
            end case;
            s_lm_prod <= s_p12_g(12 downto 3) * s_wv;
            -- phase step per pixel = per virtual unit x the x step, so the
            -- pattern period is the same fraction of the screen in every mode;
            -- floored at a 2-px period (Nyquist: K4's 2.8-px bottom end would
            -- be 1.4 px at SD and alias backwards).  Never reached at 1080p.
            case to_integer(s_xs) is
                when 4      => s_step_x <= s_step & '0';
                when 3      => s_step_x <= ('0' & s_step) + shift_right(s_step, 1);
                when others => s_step_x <= '0' & s_step;
            end case;
            if s_step_x(16 downto 15) /= "00" then
                s_step_v <= x"7FFF";
            else
                s_step_v <= s_step_x(15 downto 0);
            end if;
            -- K4 travel = the old 30..100% (HW: below ~30% the 2-px dash
            -- pitch is all aliasing): step = 23200 - g*2.797, built as two
            -- registered partial sums (2 + 1/2) and (1/4 + 1/32 + 1/64)
            s_k4_a  <= shift_left(resize(s_k4_g, 16), 1) + shift_right(resize(s_k4_g, 16), 1);
            s_k4_b  <= shift_right(resize(s_k4_g, 16), 2) + shift_right(resize(s_k4_g, 16), 5)
                       + shift_right(resize(s_k4_g, 16), 6);
            s_k4_sc <= s_k4_a + s_k4_b;
            if s_fev = '1' then
                s_k4_g    <= f_glide(s_k4_t, s_k4_g);
                s_p12_g   <= f_glide(s_p12_t, s_p12_g);
                -- period: knob up = longer period. step 23200 -> ~310
                -- (2.8 .. ~210 px); glide fraction bits give sub-LSB steps
                s_step    <= to_unsigned(23200, 16) - s_k4_sc;
                s_len_max <= resize(s_lm_prod(20 downto 10), 12) + to_unsigned(8, 12);
            end if;
        end if;
    end process p_ctl;

    ----------------------------------------------------------------------------
    -- Pixel pipeline: Sobel edge bit, line/column bookkeeping, seed
    ----------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_m8 : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hs <= data_in.hsync_n;
            s_avid_d  <= data_in.avid;
            if data_in.hsync_n = '0' and s_prev_hs = '1' then
                s_hs_edge <= '1';
            else
                s_hs_edge <= '0';
            end if;

            av(1) <= data_in.avid;
            for i in 2 to C_LAT - 1 loop
                av(i) <= av(i - 1);
            end loop;

            -- Frame-line key = the virtual 1080-line row (the active line
            -- itself at 1080p; f_vstep_y already doubles interlaced fields).
            -- field_n is tied high on hardware, so interlace is known only
            -- from the timing ID.
            s_fl <= '0' & s_ya(18 downto 8);
            -- line group for the hash (barrel by K5, quasi-static)
            s_lg <= shift_right(s_fl, to_integer(s_csh));

            -- S1: 8-bit luma, present line-buffer read, column of data_in
            if data_in.avid = '1' then
                y8  <= unsigned(data_in.y(9 downto 2));
                lrx <= lrx + 1;
                cx0 <= cx0 + 1;
                xa1 <= xa1 + s_xs;
                s_seen <= '1';
                s_lfsr_act <= s_lfsr;
            end if;
            -- measured active width at the fall of avid
            if s_avid_d = '1' and data_in.avid = '0' then
                s_width <= cx0;
            end if;

            -- S2: land BRAM reads (qr1 = 1 line up, qr2 = 2 lines up)
            if av(1) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                y8d <= y8;
                lwx <= lwx + 1;
            end if;

            -- S3: Sobel column sums (top = qr2, mid = qr1, bottom = y8d)
            if av(2) = '1' then
                ss  <= resize(qr2, 10) + shift_left(resize(qr1, 10), 1) + resize(y8d, 10);
                dy  <= signed(resize(qr2, 9)) - signed(resize(y8d, 9));
                ss1 <= ss;  ss2 <= ss1;
                dy1 <= dy;  dy2 <= dy1;
            end if;

            -- S4: gradients
            if av(3) = '1' then
                s_gx <= signed(resize(ss, 12)) - signed(resize(ss2, 12));
                s_gy <= resize(dy, 12) + shift_left(resize(dy1, 12), 1) + resize(dy2, 12);
            end if;

            -- S5: magnitude + horizontal polarity
            if av(4) = '1' then
                s_mag <= resize(f_absu(s_gx), 12) + resize(f_absu(s_gy), 12);
                fall5 <= s_gx(11);          -- gx < 0 : luma falls to the right
            end if;

            -- S6: normalize, guards (window straddles previous field / hblank)
            -- mag>>1: a vertical step of D codes reads 2*D, so the K2 sweep
            -- covers luma steps 0..50% (>>3 put everything above ~12% of
            -- the knob past any real edge -- a black screen at defaults)
            if av(5) = '1' then
                v_m8 := f_c255(shift_right(s_mag, 1));
                if s_aline < 3 or cbx < 4 then
                    v_m8 := (others => '0');
                end if;
                s_mag8 <= v_m8;
                fall6  <= fall5;
                cbx    <= cbx + 1;
            end if;

            -- S7: threshold + polarity gate
            if av(6) = '1' then
                if s_mag8 > s_thr and (s_fall = '0' or fall6 = '1') then
                    e7 <= '1';
                else
                    e7 <= '0';
                end if;
                fall7 <= fall6;
            else
                e7 <= '0';
            end if;

            -- New line: reset column state; count active lines
            if data_in.hsync_n = '0' and s_prev_hs = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                cbx <= (others => '0');
                cx0 <= (others => '0');
                xa1 <= resize(s_xs, 12);
                if s_seen = '1' then
                    s_seen <= '0';
                    s_ya   <= s_ya + s_ys;
                    if s_aline /= 2047 then
                        s_aline <= s_aline + 1;
                    end if;
                end if;
            end if;

            -- Once-per-field event: avid absent for 2048 clocks (longer
            -- than any hblank, shorter than any vblank).  Keyed on avid
            -- alone, so neither vsync serration nor a doubled hsync edge
            -- can fire it mid-picture.
            if data_in.avid = '1' then
                s_nav <= (others => '0');
            elsif s_nav(11) = '0' then
                s_nav <= s_nav + 1;
            end if;
            s_nav11_d <= s_nav(11);
            s_fev     <= s_nav(11) and not s_nav11_d;
            if s_fev = '1' then
                s_aline <= (others => '0');
                s_ya    <= (others => '0');
                s_xs    <= f_vstep_x(s_tid);
                s_ys    <= f_vstep_y(s_tid);
                s_fcnt  <= s_fcnt + 1;
                if s_boil = '1' and s_fcnt = 0 then
                    s_seed <= unsigned(s_lfsr_act);
                end if;
            end if;
            if s_boil = '0' then
                s_seed <= C_SEED;
            end if;
        end if;
    end process p_pix;

    -- Luma line buffers: read at lrx (one column ahead of the write), rotate
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(1) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(y8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(1) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    ----------------------------------------------------------------------------
    -- Dry-video ring: written every clock, read 5 back, then a fabric FF.
    -- vr7 is the source pixel aligned with stage 7; vr8/vr9 trail it.
    ----------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rq <= ring(to_integer(wp - 5));
            ring(to_integer(wp)) <= "00" & data_in.y & data_in.u & data_in.v;
            wp <= wp + 1;
        end if;
    end process p_ring;

    p_vchain : process(clk)
    begin
        if rising_edge(clk) then
            vr7 <= rq(29 downto 0);
            vr8 <= vr7;
            vr9 <= vr8;
            vr10 <= vr9;
        end if;
    end process p_vchain;

    ----------------------------------------------------------------------------
    -- Spatial hash, one op per stage (spot-checked: neighbour / seed
    -- correlation of every used field < 0.006):
    --   a = (line_group & col_group) + seed
    --   b = a ^ a>>5 ; c = b * (1 + 2^5 + 2^11)
    --   d = c ^ c>>7 ; e = d * (1 + 2^9 + 2^17)
    --   f = e ^ e>>13
    -- f(23..16) -> length, f(19..12) -> fire gate, f(15..0) -> run seed
    ----------------------------------------------------------------------------
    p_hash : process(clk)
        variable v_b : unsigned(17 downto 0);
        variable v_d : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            -- H1 (stage 0): computed one column AHEAD from xa1 (cx0 + 1),
            -- every clock, so the chain gains a stage for the multiply
            -- columns grouped by 16 virtual units (16 px at 1080p, 8 at SD)
            h1_a <= (s_lg(10 downto 0) & xa1(11 downto 5)) + resize(s_seed, 18);
            -- H2 (stage 1)
            if data_in.avid = '1' then
                v_b   := h1_a xor shift_right(h1_a, 5);
                h2_b  <= v_b;
                h2_c1 <= resize(v_b, 24) + shift_left(resize(v_b, 24), 5);
            end if;
            -- H3 (stage 2)
            if av(1) = '1' then
                h3_c <= h2_c1 + shift_left(resize(h2_b, 24), 11);
            end if;
            -- H4 (stage 3)
            if av(2) = '1' then
                v_d   := h3_c xor shift_right(h3_c, 7);
                h4_d  <= v_d;
                h4_e1 <= v_d + shift_left(v_d, 9);
            end if;
            -- H5 (stage 4)
            if av(3) = '1' then
                h5_e <= h4_e1 + shift_left(h4_d, 17);
            end if;
            -- H6 (stage 5)
            if av(4) = '1' then
                h6_f <= h5_e xor shift_right(h5_e, 13);
            end if;
            -- H7a (stage 6): length partial products (4x12 each, alone);
            -- fire compare; run seed
            if av(5) = '1' then
                lp_lo6 <= h6_f(19 downto 16) * s_len_max;
                lp_hi6 <= h6_f(23 downto 20) * s_len_max;
                if h6_f(19 downto 12) <= s_dens then
                    fire6 <= '1';
                else
                    fire6 <= '0';
                end if;
                rh6 <= h6_f(15 downto 0);
            end if;
            -- H7b (stage 7): len = hash8 * len_max (>>8 taken by the consumer)
            if av(6) = '1' then
                lp7    <= resize(lp_lo6, 20) + shift_left(resize(lp_hi6, 20), 4);
                fire7  <= fire6;
                rhash7 <= rh6;
            end if;
        end if;
    end process p_hash;

    ----------------------------------------------------------------------------
    -- Run state machine (stage 8): one filament per scanline at a time.
    -- Launch on a fired edge (always in Restart mode, only when idle in
    -- Hold mode); count down the hashed length; advance the pattern phase;
    -- step the run's own seed at each phase wrap (Morse: xorshift,
    -- Bits: rotate); Stutter re-samples the held video colour each wrap.
    ----------------------------------------------------------------------------
    p_run : process(clk)
        variable v_launch : std_logic;
        variable v_sum    : unsigned(16 downto 0);
        variable v_len    : unsigned(11 downto 0);
        variable v_x      : unsigned(15 downto 0);
        variable v_tk     : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            if s_hs_edge = '1' then
                r_act  <= '0';
                r_rem  <= (others => '0');
                r_wrap <= '0';
            elsif av(7) = '1' then
                v_launch := e7 and fire7 and (s_hold nand r_act);
                v_sum    := ('0' & r_ph) + ('0' & s_step_v);
                v_len    := lp7(19 downto 8);
                if v_len < 2 then
                    v_len := to_unsigned(2, 12);
                end if;
                -- taper over the last T = 2^(msb(len)-1) pixels (a quarter
                -- to a half of the run), capped at 256
                if lp7(19 downto 17) /= 0 then    v_tk := to_unsigned(8, 4);
                elsif lp7(16) = '1' then          v_tk := to_unsigned(7, 4);
                elsif lp7(15) = '1' then          v_tk := to_unsigned(6, 4);
                elsif lp7(14) = '1' then          v_tk := to_unsigned(5, 4);
                elsif lp7(13) = '1' then          v_tk := to_unsigned(4, 4);
                elsif lp7(12) = '1' then          v_tk := to_unsigned(3, 4);
                elsif lp7(11) = '1' then          v_tk := to_unsigned(2, 4);
                elsif lp7(10) = '1' then          v_tk := to_unsigned(1, 4);
                else                              v_tk := to_unsigned(0, 4);
                end if;

                if v_launch = '1' then
                    r_act  <= '1';
                    r_rem  <= v_len & '0';
                    r_seg  <= (others => '0');
                    r_sl   <= rhash7;
                    -- Hold colour: over video, the object side (left, so a
                    -- falling edge streams its bright side into the dark);
                    -- over black, always the BRIGHT side, else every rising
                    -- edge launches an invisible dark filament
                    if fall7 = '0' and s_gnd_vid = '0' then
                        r_hold <= vr7;             -- right of the edge
                    else
                        r_hold <= vr9;             -- left of the edge
                    end if;
                    r_pol  <= fall7;
                    r_tint <= rhash7(10 downto 8);
                    r_tk   <= v_tk;
                    r_wrap <= '0';
                    if s_pat = 7 then
                        r_ph <= s_fl(3 downto 0) & x"000";   -- Fray: per-line phase
                    else
                        r_ph <= (others => '0');
                    end if;
                elsif r_act = '1' then
                    -- count down in virtual units (one x step per pixel)
                    if r_rem(12 downto 3) = 0 and r_rem(2 downto 0) <= s_xs then
                        r_act <= '0';
                        r_rem <= (others => '0');
                    else
                        r_rem <= r_rem - s_xs;
                    end if;
                    r_ph   <= v_sum(15 downto 0);
                    r_wrap <= v_sum(16);
                    -- act on the wrap one pixel late: keeps the adder's
                    -- carry-out off the enables of the 30-bit hold register
                    if r_wrap = '1' then
                        if r_seg = 6 then
                            r_seg <= (others => '0');
                        else
                            r_seg <= r_seg + 1;
                        end if;
                        case to_integer(s_pat) is
                            when 1 =>          -- Morse: xorshift16
                                v_x := r_sl xor shift_left(r_sl, 7);
                                v_x := v_x xor shift_right(v_x, 9);
                                r_sl <= v_x xor shift_left(v_x, 8);
                            when 5 =>          -- Bits: rotate the code
                                r_sl <= r_sl(0) & r_sl(15 downto 1);
                            when 6 =>          -- Stutter: re-sample video
                                r_hold <= vr9;
                            when others =>
                                null;
                        end case;
                    end if;
                else
                    r_wrap <= '0';
                end if;
            end if;
        end if;
    end process p_run;

    ----------------------------------------------------------------------------
    -- Pattern stage (9): ink + intensity (0..256) + palette index.
    -- Also glow / negative versions of the live video for the colour mux.
    ----------------------------------------------------------------------------
    p_pattern : process(clk)
        variable v_ink  : std_logic;
        variable v_int  : unsigned(8 downto 0);
        variable v_y    : unsigned(10 downto 0);
        variable v_c    : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_ink := '1';
            v_int := to_unsigned(256, 9);
            case to_integer(s_pat) is
                when 0 =>                          -- Dash
                    v_ink := not r_ph(15);
                when 1 =>                          -- Morse: random duty per period
                    if r_ph(15 downto 13) < r_sl(7 downto 5) then
                        v_ink := '1';
                    else
                        v_ink := '0';
                    end if;
                when 2 =>                          -- Comet: sawtooth heads
                    v_int := to_unsigned(256, 9) - resize(r_ph(15 downto 8), 9);
                when 3 =>                          -- Beads: triangle
                    if r_ph(15) = '0' then
                        v_int := resize(r_ph(14 downto 7), 9);
                    else
                        v_int := to_unsigned(255, 9) - resize(r_ph(14 downto 7), 9);
                    end if;
                when 4 =>                          -- Sparks: dim thread + specks
                    if s_lfsr(1 downto 0) = "00" and r_ph(15) = '0' then
                        v_int := to_unsigned(256, 9);
                    else
                        v_int := to_unsigned(40, 9);
                    end if;
                when 5 =>                          -- Bits: the run's binary code
                    v_ink := r_sl(0);
                when 6 =>                          -- Stutter: blocks with 1/16 gap
                    if r_ph(15 downto 12) = 0 then
                        v_ink := '0';
                    end if;
                when others =>                     -- Fray: dashes, phase per line
                    v_ink := not r_ph(15);
            end case;
            if r_act = '0' then
                v_ink := '0';
            end if;

            case to_integer(s_col) is
                when 4      => p_pidx <= "00" & r_seg;       -- Rainbow
                when 5      => p_pidx <= "1000" & r_pol;     -- Polarity
                when others => p_pidx <= "00" & r_tint;      -- Tint (Heat: stage 10)
            end case;

            p_ink <= v_ink;
            p_int <= v_int;
            p_tap <= f_taper(r_rem(12 downto 1), r_tk);
            hold9 <= r_hold;

            -- Glow: luma +192 (cap 768: full-scale luma kills chroma on
            -- the hardware), chroma x2 about 512 (clamp 64..960)
            v_y := resize(unsigned(vr8(29 downto 20)), 11) + to_unsigned(192, 11);
            if v_y > 768 then
                v_y := to_unsigned(768, 11);
            end if;
            glow9(29 downto 20) <= std_logic_vector(v_y(9 downto 0));
            v_c := shift_left(signed(resize(unsigned(vr8(19 downto 10)), 12)) - to_signed(512, 12), 1);
            if v_c > 448 then
                v_c := to_signed(448, 12);
            elsif v_c < -448 then
                v_c := to_signed(-448, 12);
            end if;
            glow9(19 downto 10) <= std_logic_vector(resize(v_c + to_signed(512, 12), 10));
            v_c := shift_left(signed(resize(unsigned(vr8(9 downto 0)), 12)) - to_signed(512, 12), 1);
            if v_c > 448 then
                v_c := to_signed(448, 12);
            elsif v_c < -448 then
                v_c := to_signed(-448, 12);
            end if;
            glow9(9 downto 0) <= std_logic_vector(resize(v_c + to_signed(512, 12), 10));

            -- Negative
            neg9(29 downto 20) <= std_logic_vector(to_unsigned(1023, 10) - unsigned(vr8(29 downto 20)));
            neg9(19 downto 10) <= std_logic_vector(to_unsigned(1023, 10) - unsigned(vr8(19 downto 10)));
            neg9(9 downto 0)   <= std_logic_vector(to_unsigned(1023, 10) - unsigned(vr8(9 downto 0)));
        end if;
    end process p_pattern;

    ----------------------------------------------------------------------------
    -- Taper stage (10): every filament fades out over its tail.  Heat takes
    -- its palette index from the combined level, so it runs black -> red ->
    -- yellow -> white along every pattern (not just the graded ones).
    ----------------------------------------------------------------------------
    p_taper_stage : process(clk)
        variable v_c : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if p_tap < p_int then
                v_c := p_tap;
            else
                v_c := p_int;
            end if;
            if s_col = 6 then
                if v_c(8) = '1' then
                    t_pidx <= "01111";
                else
                    t_pidx <= "01" & v_c(7 downto 5);
                end if;
                t_int <= to_unsigned(256, 9);      -- Heat: colour carries the fade
            else
                t_pidx <= p_pidx;
                t_int  <= v_c;
            end if;
            t_ink  <= p_ink;
            hold_t <= hold9;
            glow_t <= glow9;
            neg_t  <= neg9;
        end if;
    end process p_taper_stage;

    ----------------------------------------------------------------------------
    -- Colour stages 11..14: out = ground + (base - ground) * int / 256
    ----------------------------------------------------------------------------
    p_colour : process(clk)
        variable v_sy, v_su, v_sv : signed(12 downto 0);
        variable v_ny : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S11: base colour + ground select
            case to_integer(s_col) is
                when 0 =>                                  -- Hold
                    b_y <= unsigned(hold_t(29 downto 20));
                    b_u <= unsigned(hold_t(19 downto 10));
                    b_v <= unsigned(hold_t(9 downto 0));
                when 1 =>                                  -- Glow
                    b_y <= unsigned(glow_t(29 downto 20));
                    b_u <= unsigned(glow_t(19 downto 10));
                    b_v <= unsigned(glow_t(9 downto 0));
                when 2 =>                                  -- Negative
                    b_y <= unsigned(neg_t(29 downto 20));
                    b_u <= unsigned(neg_t(19 downto 10));
                    b_v <= unsigned(neg_t(9 downto 0));
                when 3 =>                                  -- White
                    b_y <= to_unsigned(768, 10);
                    b_u <= to_unsigned(512, 10);
                    b_v <= to_unsigned(512, 10);
                when others =>                             -- palettes
                    b_y <= C_PAL_Y(to_integer(t_pidx));
                    b_u <= C_PAL_U(to_integer(t_pidx));
                    b_v <= C_PAL_V(to_integer(t_pidx));
            end case;
            if s_gnd_vid = '1' then
                g10 <= vr10;
            else
                g10 <= std_logic_vector(to_unsigned(64, 10))
                       & std_logic_vector(to_unsigned(512, 10))
                       & std_logic_vector(to_unsigned(512, 10));
            end if;
            ink10 <= t_ink;
            int10 <= t_int;

            -- S12: difference
            d_y <= signed(resize(b_y, 11)) - signed(resize(unsigned(g10(29 downto 20)), 11));
            d_u <= signed(resize(b_u, 11)) - signed(resize(unsigned(g10(19 downto 10)), 11));
            d_v <= signed(resize(b_v, 11)) - signed(resize(unsigned(g10(9 downto 0)), 11));
            g11   <= g10;
            ink11 <= ink10;
            int11_y <= int10;
            int11_u <= int10;
            int11_v <= int10;

            -- S13: partial products, each multiply alone (11x6 / 11x5)
            pl_y <= d_y * signed('0' & std_logic_vector(int11_y(4 downto 0)));
            pl_u <= d_u * signed('0' & std_logic_vector(int11_u(4 downto 0)));
            pl_v <= d_v * signed('0' & std_logic_vector(int11_v(4 downto 0)));
            ph_y <= d_y * signed('0' & std_logic_vector(int11_y(8 downto 5)));
            ph_u <= d_u * signed('0' & std_logic_vector(int11_u(8 downto 5)));
            ph_v <= d_v * signed('0' & std_logic_vector(int11_v(8 downto 5)));
            g12   <= g11;
            ink12 <= ink11;

            -- S14: combine partials
            pr_y <= resize(pl_y, 21) + shift_left(resize(ph_y, 21), 5);
            pr_u <= resize(pl_u, 21) + shift_left(resize(ph_u, 21), 5);
            pr_v <= resize(pl_v, 21) + shift_left(resize(ph_v, 21), 5);
            g13   <= g12;
            ink13 <= ink12;

            -- S15: compose (result always lies between ground and base)
            v_sy := signed(resize(unsigned(g13(29 downto 20)), 13)) + pr_y(20 downto 8);
            v_su := signed(resize(unsigned(g13(19 downto 10)), 13)) + pr_u(20 downto 8);
            v_sv := signed(resize(unsigned(g13(9 downto 0)), 13))   + pr_v(20 downto 8);
            if ink13 = '1' then
                o_y <= unsigned(v_sy(9 downto 0));
                o_u <= unsigned(v_su(9 downto 0));
                o_v <= unsigned(v_sv(9 downto 0));
            else
                o_y <= unsigned(g13(29 downto 20));
                o_u <= unsigned(g13(19 downto 10));
                o_v <= unsigned(g13(9 downto 0));
            end if;

            -- S16: negative + blanking gate.  Negative
            -- luma maps 64 <-> 768 (black ground -> white at the drawn-luma
            -- cap, never superwhite); clamp tests run on o_y in parallel
            -- with the subtract.
            v_ny := to_unsigned(832, 10) - o_y;
            if o_y > 768 then
                v_ny := to_unsigned(64, 10);
            elsif o_y < 64 then
                v_ny := to_unsigned(768, 10);
            end if;
            if av(15) = '0' then
                g_y <= std_logic_vector(to_unsigned(64, 10));
                g_u <= std_logic_vector(to_unsigned(512, 10));
                g_v <= std_logic_vector(to_unsigned(512, 10));
            elsif s_neg = '1' then
                g_y <= std_logic_vector(v_ny);
                g_u <= std_logic_vector(to_unsigned(1023, 10) - o_u);
                g_v <= std_logic_vector(to_unsigned(1023, 10) - o_v);
            else
                g_y <= std_logic_vector(o_y);
                g_u <= std_logic_vector(o_u);
                g_v <= std_logic_vector(o_v);
            end if;
        end if;
    end process p_colour;

    ----------------------------------------------------------------------------
    -- Stages 17-18: horizontal [1 2 1]/4 low-pass on CHROMA only.  The core
    -- sends 4:2:2 to the encoders (U from one pixel, V from the next), so
    -- chroma alternating at the pixel rate -- 1-px dashes at short Period --
    -- pairs U and V from different colours: a wrong hue that flips every
    -- time the pattern slips against the pixel pairs = solid colour-changing
    -- bars running to the right.  The filter turns a colour/black alternation
    -- into steady half-strength colour; luma stays pixel-sharp.  Re-gated at
    -- the output so the blanking interval stays exactly neutral.
    ----------------------------------------------------------------------------
    p_chroma_lp : process(clk)
        variable v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            h_y  <= g_y;
            h_u  <= g_u;   h2_u <= h_u;
            h_v  <= g_v;   h2_v <= h_v;
            v_su := resize(unsigned(h2_u), 12) + shift_left(resize(unsigned(h_u), 12), 1)
                    + resize(unsigned(g_u), 12);
            v_sv := resize(unsigned(h2_v), 12) + shift_left(resize(unsigned(h_v), 12), 1)
                    + resize(unsigned(g_v), 12);
            if av(17) = '0' then
                s_y_out <= std_logic_vector(to_unsigned(64, 10));
                s_u_out <= std_logic_vector(to_unsigned(512, 10));
                s_v_out <= std_logic_vector(to_unsigned(512, 10));
            else
                s_y_out <= h_y;                    -- centre tap
                s_u_out <= std_logic_vector(v_su(11 downto 2));
                s_v_out <= std_logic_vector(v_sv(11 downto 2));
            end if;
        end if;
    end process p_chroma_lp;

    ----------------------------------------------------------------------------
    -- Sync delay: C_LAT stages, identical in every mode
    ----------------------------------------------------------------------------
    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (C_LAT - 2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(C_LAT - 2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(C_LAT - 2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(C_LAT - 2 downto 0) & data_in.field_n;
        end if;
    end process p_sync_delay;

    s_io_align_in.y       <= s_y_out;
    s_io_align_in.u       <= s_u_out;
    s_io_align_in.v       <= s_v_out;
    s_io_align_in.avid    <= s_avid_sr   (C_LAT - 1);
    s_io_align_in.hsync_n <= s_hsync_n_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_n_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_n_sr(C_LAT - 1);

    lfsr_inst : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr
        );

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (18 -> 20).
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

end architecture fray;
