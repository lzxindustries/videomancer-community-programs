-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Subsample: catalogue combo #37 (seed 20260918) -- datamosh / block
-- displacement + chroma subsampling artifacts (4:1:1, 4:2:0, composite).
-- Three knobs each; the switches and the slider decide how the two combine.
--
--   K1 Block    block size 8 .. 64 px (continuous)
--   K2 Shift    block displacement (floor: moshed blocks always move; wraps
--               inside the picture)
--   K3 Rate     re-roll rate, every ~2 s .. every field (exponential)
--   K4 Hold     chroma sampled every 1 .. 64 px
--   K5 Bleed    chroma IIR smear length (0 = off)
--   K6 Crawl    composite dot-crawl amplitude (luma dots where there is colour)
--
--   S7  Blocks  shift / hold (block repeats its first column)
--   S8  Chroma  subsampled after the mosh / before it (blocks carry the artefacts)
--   S9  Crawl   crawling checker dots / marching bars
--   S10 Moshed  blocks keep chroma / lose chroma (grey blocks)
--   S11 Hold Shape  chroma held along the line / in squares (the held
--               sample also repeats down Hold lines: 4:2:0 and worse)
--   P12 Density fraction of blocks moshed (~5% .. ~98%)
--
-- Two-bank line buffers.  Luma ping-pongs every line (shown one line late).
-- Chroma ping-pongs on capture lines only: every line, or in Square the
-- first line of each Hold-tall group (frame lines, so squares stay square
-- under interlace), shown on the rest of that group.  U/V
-- are swapped on rows whose hsync->avid parity differs from their chroma
-- capture row's (drape v1.3: a stored line replayed on other rows).
-- Per-block hash on a row & column key, re-roll seed entering after the
-- first round; displaced reads wrap inside the C_EDGE inset.  Chroma hold
-- (column counter) and IIR bleed run pre-buffer (Before) or post-fetch
-- (After).  The dot crawl is a triangle wave, period ~4 px SD / ~11 px HD,
-- weighted by the output saturation, half a period apart on alternate lines
-- (Checker) and a quarter period further on every field.
-- Latency 22 clocks, all modes, blanking-gated.  no_rw_check on every RAM:
-- a bank's read data is only used while the other bank is written.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The block grid and the chroma
-- hold strips / squares count virtual 1080-line units, stepped per pixel /
-- per field line from the video timing ID (registers_in(8)): 1.0 at 1080p
-- (identical to before), 1.5 at 720p, and at SD 2.0 across (a 1440-wide
-- 4:3 window) and 1080/486 per frame line (interlaced fields step two frame
-- lines, as the old row counters did).  Blocks and held-chroma squares are
-- the same fraction of the screen height in every mode and stay square on
-- a 4:3 screen; a hold shorter than one pixel / field line samples every
-- pixel / line.  The displacement is in the same units: the HD shift switch
-- (width bit 10) used to give 720p the 1080p pixel offsets, so the 720p
-- shift is now 2/3 of them (SD already used half).  The bleed IIR and the
-- dot crawl stay at pixel scale.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture subsample of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (22 + 2 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 22;
    constant C_EDGE : integer := 12;

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;
    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;
    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

    -- re-roll period - 1 in fields: round(128 * 2^(-7i/31)) - 1
    type t_period is array (0 to 31) of unsigned(6 downto 0);
    constant C_PERIOD : t_period := (
        to_unsigned(127, 7), to_unsigned(108, 7), to_unsigned(93, 7), to_unsigned(79, 7),
        to_unsigned(67, 7),  to_unsigned(58, 7),  to_unsigned(49, 7), to_unsigned(42, 7),
        to_unsigned(36, 7),  to_unsigned(30, 7),  to_unsigned(26, 7), to_unsigned(22, 7),
        to_unsigned(19, 7),  to_unsigned(16, 7),  to_unsigned(13, 7), to_unsigned(11, 7),
        to_unsigned(9, 7),   to_unsigned(8, 7),   to_unsigned(7, 7),  to_unsigned(6, 7),
        to_unsigned(5, 7),   to_unsigned(4, 7),   to_unsigned(3, 7),  to_unsigned(2, 7),
        to_unsigned(2, 7),   to_unsigned(2, 7),   to_unsigned(1, 7),  to_unsigned(1, 7),
        to_unsigned(1, 7),   to_unsigned(0, 7),   to_unsigned(0, 7),  to_unsigned(0, 7));

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

    -- line buffers, two banks each (chroma at pair rate, 8 bits)
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- controls
    signal s_k_block, s_k_shift, s_k_rate, s_k_hold, s_k_bleed, s_k_crawl : unsigned(9 downto 0);
    signal s_sw_hold, s_sw_pre, s_sw_bars, s_sw_grey, s_sw_sq : std_logic;
    signal s_p_density : unsigned(9 downto 0);

    -- per-frame (vblank sequencer)
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_lk_1, s_lk_2, s_lk_5, s_lk_p : unsigned(9 downto 0) := (others => '0');
    signal s_lw     : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_shift, s_dens, s_cg : unsigned(7 downto 0) := (others => '0');
    signal s_bsz    : unsigned(6 downto 0) := to_unsigned(8, 7);      -- block size 8..64
    signal s_hnm1   : unsigned(5 downto 0) := (others => '0');        -- hold period - 1 (0..63)
    signal s_vnm1   : unsigned(5 downto 0) := to_unsigned(1, 6);      -- square group lines - 1 (>= 1)
    signal s_vn     : unsigned(6 downto 0) := to_unsigned(2, 7);
    -- bleed shift, one copy per IIR, each loaded in its own sequencer state so
    -- yosys cannot merge them (one shared net was the HD critical path)
    signal s_bn_pu, s_bn_pv, s_bn_qu, s_bn_qv : unsigned(2 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_bn_pu, s_bn_pv, s_bn_qu, s_bn_qv : signal is true;
    signal s_period, s_rate_cnt : unsigned(6 downto 0) := (others => '0');
    signal s_roll_due : std_logic := '0';
    signal s_roll, s_sd, s_seed : unsigned(15 downto 0) := (others => '0');
    signal s_hi     : unsigned(10 downto 0) := to_unsigned(708, 11);  -- width - C_EDGE
    signal s_span   : unsigned(10 downto 0) := to_unsigned(696, 11);  -- width - 2 C_EDGE
    signal s_hd     : std_logic := '0';
    signal s_sha, s_shb : unsigned(7 downto 0) := (others => '0');   -- 720p shift x 2/3 parts
    signal s_vall   : std_logic := '0';   -- square group <= one field-line step: every line captures
    -- resolution steps (free-running from the timing ID)
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_step   : unsigned(7 downto 0) := to_unsigned(64, 8);     -- crawl phase per pixel
    signal s_fph    : unsigned(7 downto 0) := (others => '0');        -- crawl phase per field

    -- tracking
    signal s_rx, s_line : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_ilace, s_fld, s_fld_prev, s_fsrc, s_ftog : std_logic := '0';
    -- banks: luma swaps every line, chroma on capture lines
    signal s_wpar, s_rbank, s_cwpar, s_crbank : std_logic := '0';
    signal s_ccap, s_vfirst : std_logic := '1';
    signal s_vca, s_vcn, s_vcw : unsigned(14 downto 0) := (others => '0');   -- Q.8 virtual
    -- hsync->avid parity of this line and of each chroma bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    -- crawl phase: line start, running (aligned to stage 18)
    signal s_lpar   : std_logic := '0';
    signal s_lph, s_ph : unsigned(7 downto 0) := (others => '0');

    -- block grid: column DDA (stage 0), row DDA (per line); hold column
    -- counter -- virtual units, columns Q.1, rows Q.8
    signal s_cca    : unsigned(7 downto 0) := (others => '0');
    signal s_bc     : unsigned(7 downto 0) := (others => '0');
    signal s_isfirst : std_logic := '1';
    signal s_rca, s_rcn, s_rcw : unsigned(14 downto 0) := (others => '0');
    signal s_br, s_krow : unsigned(7 downto 0) := (others => '0');
    signal s_hca    : unsigned(7 downto 0) := (others => '0');
    signal s_hfirst : std_logic := '1';

    -- input side (Before: hold stage 1, bleed stage 2; written stage 2)
    signal s_y1, s_y2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_hu1, s_hv1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pu, s_pv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_wx1, s_wx2 : unsigned(10 downto 0) := (others => '0');
    signal s_wc1, s_wc2, s_wp1, s_wp2 : std_logic := '0';
    signal s_cwe1, s_cwe2, s_cwp1, s_cwp2 : std_logic := '0';

    -- address / hash chain (stages 1..12)
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_bc1    : unsigned(7 downto 0) := (others => '0');
    signal s_key2   : unsigned(15 downto 0) := (others => '0');
    signal s_h3, s_h4, s_h5 : unsigned(15 downto 0) := (others => '0');
    signal s_mosh6  : std_logic := '0';
    signal s_po6    : signed(16 downto 0) := (others => '0');
    signal s_d7     : signed(12 downto 0) := (others => '0');
    signal s_u8, s_dp8, s_dm8 : signed(12 downto 0) := (others => '0');
    signal s_u9, s_up9, s_um9 : signed(12 downto 0) := (others => '0');
    signal s_lt9, s_ge9 : std_logic := '0';
    signal s_w10, s_a11, s_a12 : unsigned(10 downto 0) := (others => '0');
    type t_xd is array (2 to 10) of unsigned(10 downto 0);
    signal s_xd     : t_xd := (others => (others => '0'));
    signal s_moshd  : std_logic_vector(7 to 16) := (others => '0');
    signal s_firstd : std_logic_vector(1 to 13) := (others => '1');
    signal s_hfd    : std_logic_vector(1 to 14) := (others => '1');

    -- pixel path (stages 14..22)
    signal s_fy14   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_fu14, s_fv14 : unsigned(7 downto 0) := x"80";
    signal s_y15, s_y16, s_y17, s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_hu15, s_hv15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_qu, s_qv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_u17, s_v17, s_u18, s_v18, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u20, s_v20, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_du17, s_dv17 : signed(10 downto 0) := (others => '0');
    signal s_grey17 : std_logic := '0';
    signal s_sat18  : unsigned(10 downto 0) := (others => '0');
    signal s_amp19  : unsigned(16 downto 0) := (others => '0');
    signal s_tri19  : signed(6 downto 0) := (others => '0');
    signal s_pc20   : signed(18 downto 0) := (others => '0');
    signal s_vy21   : signed(12 downto 0) := to_signed(64, 13);
    signal s_y22    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_block <= unsigned(registers_in(0));
    s_k_shift <= unsigned(registers_in(1));
    s_k_rate  <= unsigned(registers_in(2));
    s_k_hold  <= unsigned(registers_in(3));
    s_k_bleed <= unsigned(registers_in(4));
    s_k_crawl <= unsigned(registers_in(5));
    s_sw_hold <= registers_in(6)(0);
    s_sw_pre  <= registers_in(6)(1);
    s_sw_bars <= registers_in(6)(2);
    s_sw_grey <= registers_in(6)(3);
    s_sw_sq   <= registers_in(6)(4);
    s_p_density <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Control, tracking, block grid, input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_nu, v_nv : integer range 0 to 7;
        variable v_nc, v_nh : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; s_fld <= data_in.field_n;
            else s_rx <= (others => '0'); end if;

            -- block column DDA and chroma hold counter: state for the pixel now
            -- on data_in (virtual half units; at 1080p the step is 2 and these
            -- are the old cc = bsz - 1 / hc = hold - 1 wraps; a 1-unit hold is
            -- shorter than a 720p / SD pixel, so every pixel samples)
            if data_in.avid = '1' then
                v_nc := s_cca + s_xs;
                if v_nc(7 downto 1) >= s_bsz then
                    s_cca <= v_nc - (s_bsz & '0'); s_bc <= s_bc + 1; s_isfirst <= '1';
                else
                    s_cca <= v_nc; s_isfirst <= '0';
                end if;
                v_nh := s_hca + s_xs;
                if s_hnm1 = 0 then
                    s_hca <= (others => '0'); s_hfirst <= '1';
                elsif v_nh(7 downto 1) > s_hnm1 then
                    s_hca <= v_nh - (('0' & s_hnm1 & '0') + 2); s_hfirst <= '1';
                else
                    s_hca <= v_nh; s_hfirst <= '0';
                end if;
            else
                s_cca <= (others => '0'); s_bc <= (others => '0'); s_isfirst <= '1';
                s_hca <= (others => '0'); s_hfirst <= '1';
            end if;
            -- row DDA candidates (block rows, square chroma groups; change once
            -- per line); the Q.8 field-line step already counts frame lines
            -- under interlace
            s_rcn <= s_rca + s_ys;
            s_vcn <= s_vca + s_ys;
            s_rcw <= s_rcn - (s_bsz & x"00");
            s_vcw <= s_vcn - (s_vn & x"00");
            -- this line is the first of its square group in this field: the
            -- counter wrapped on the way to it (or the field just started)
            if s_vca < s_ys then s_vfirst <= '1'; else s_vfirst <= '0'; end if;

            -- stage 1: chroma hold (Before: sample at the start of each hold period)
            s_y1 <= unsigned(data_in.y);
            if s_sw_pre = '0' or s_hfirst = '1' then
                s_hu1 <= unsigned(data_in.u); s_hv1 <= unsigned(data_in.v);
            end if;
            s_wx1 <= s_rx; s_wc1 <= data_in.avid; s_wp1 <= s_wpar;
            s_cwe1 <= s_ccap; s_cwp1 <= s_cwpar;
            -- stage 2: bleed IIR (Q6) when Before; write data
            v_nu := to_integer(s_bn_pu); v_nv := to_integer(s_bn_pv);
            if s_wc1 = '0' or s_sw_pre = '0' then
                s_pu <= s_hu1 & "000000"; s_pv <= s_hv1 & "000000";
            else
                s_pu <= s_pu - shift_right(s_pu, v_nu) + shift_right(s_hu1 & "000000", v_nu);
                s_pv <= s_pv - shift_right(s_pv, v_nv) + shift_right(s_hv1 & "000000", v_nv);
            end if;
            s_y2 <= s_y1; s_wx2 <= s_wx1; s_wc2 <= s_wc1; s_wp2 <= s_wp1;
            s_cwe2 <= s_cwe1; s_cwp2 <= s_cwp1;

            -- per-frame sequencer (one small op per state)
            case to_integer(s_vstep) is
                when 0 =>
                    s_lk_1 <= s_k_block; s_lk_2 <= s_k_shift; s_lk_p <= s_p_density;
                    s_lw   <= s_line_width;
                    s_hnm1 <= s_k_hold(9 downto 4);
                    s_lk_5  <= s_k_bleed;
                    s_bn_pu <= s_k_bleed(9 downto 7);
                    s_cg   <= s_k_crawl(9 downto 2);
                    s_fph  <= s_fph + 64;
                    if s_rate_cnt >= s_period then
                        s_rate_cnt <= (others => '0'); s_roll_due <= '1';
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
                    s_period <= C_PERIOD(to_integer(s_k_rate(9 downto 5)));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_bn_pv <= s_lk_5(9 downto 7);
                    s_dens  <= s_lk_p(9 downto 2) - resize(s_lk_p(9 downto 6), 8);
                    s_shift <= s_lk_2(9 downto 2) - resize(s_lk_2(9 downto 6), 8);
                    s_bsz   <= resize(s_lk_1(9 downto 4), 7) - resize(s_lk_1(9 downto 7), 7);
                    s_hi    <= s_lw - C_EDGE;
                    if s_hnm1 = 0 then s_vnm1 <= to_unsigned(1, 6); else s_vnm1 <= s_hnm1; end if;
                    -- interlace: re-roll only ahead of the top field
                    if s_roll_due = '1' and (s_ilace = '0' or (s_fsrc = '1' and s_fld = '0') or (s_fsrc = '0' and s_ftog = '0')) then
                        s_roll <= s_roll + 1; s_roll_due <= '0';
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_bn_qu <= s_lk_5(9 downto 7);
                    s_dens  <= s_dens + 12;
                    s_shift <= s_shift + 12;
                    s_bsz   <= s_bsz + 8;
                    s_span  <= s_lw - 2 * C_EDGE;
                    s_vn    <= resize(s_vnm1, 7) + 1;
                    if s_lw(10) = '1' then s_step <= to_unsigned(24, 8); else s_step <= to_unsigned(64, 8); end if;
                    s_sd    <= f_round1(s_roll xor x"5A3C");
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_bn_qv <= s_lk_5(9 downto 7);
                    -- pixel offsets: HD class (1080 and 720) shifts 1/32 of
                    -- the product, SD 1/64; 720p scales the amount by 2/3
                    if s_xs = 4 then s_hd <= '0'; else s_hd <= '1'; end if;
                    s_sha   <= ('0' & s_shift(7 downto 1)) + ("000" & s_shift(7 downto 3));
                    s_shb   <= ("00000" & s_shift(7 downto 5)) + ("0000000" & s_shift(7 downto 7));
                    -- a square group no taller than one field-line step
                    if (s_vn & x"00") <= resize(s_ys, 15) then s_vall <= '1'; else s_vall <= '0'; end if;
                    s_sd    <= f_round2(s_sd);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_seed  <= f_round3(s_sd);
                    if s_xs = 3 then s_shift <= s_sha + s_shb; end if;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- hsync->avid parity (drape v1.3)
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_line_par <= s_hpar;
                if s_ccap = '1' then
                    if s_cwpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
                end if;
            end if;
            if s_crbank = '1' then
                s_rowswap <= s_line_par xor s_cap_par1;
            else
                s_rowswap <= s_line_par xor s_cap_par0;
            end if;

            -- line events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                if s_rcn(14 downto 8) >= s_bsz then s_rca <= s_rcw; s_br <= s_br + 1; else s_rca <= s_rcn; end if;
                if s_vall = '1' then s_vca <= (others => '0');
                elsif s_vcn(14 downto 8) >= s_vn then s_vca <= s_vcw;
                else s_vca <= s_vcn; end if;
                s_lpar <= not s_lpar;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_krow <= s_br;
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
                -- chroma capture: every line (Line), or a group's first line in
                -- this field (Square).  A capture line shows the previous
                -- capture, the rest of the group its own first line -- with
                -- luma one line late too, each group's colour lines up with it.
                if s_sw_sq = '0' or s_vfirst = '1' then
                    s_ccap <= '1'; s_cwpar <= not s_cwpar; s_crbank <= s_cwpar;
                else
                    s_ccap <= '0'; s_crbank <= s_cwpar;
                end if;
                -- crawl: alternate lines half a period apart (Checker)
                s_lph <= (s_fph(7) xor (s_lpar and not s_sw_bars)) & s_fph(6 downto 0);
            end if;
            -- frame events: resets are idempotent; toggles need the saw-active guard
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_br <= (others => '0');
                s_lpar <= '0';
                -- a bottom field (frame lines 1, 3, ..) follows a top field:
                -- it starts half a field step (one frame line) down
                if (s_ilace and s_fsrc and s_fld) = '1' then
                    s_rca <= resize(s_ys(11 downto 1), 15);
                    s_vca <= resize(s_ys(11 downto 1), 15);
                else
                    s_rca <= (others => '0');
                    s_vca <= (others => '0');
                end if;
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                    if s_line < 600 then s_ilace <= '1'; else s_ilace <= '0'; end if;
                    s_fld_prev <= s_fld;
                    s_fsrc <= s_fld xor s_fld_prev;
                    s_ftog <= not s_ftog;
                end if;
                s_saw_active <= '0';
            end if;

            -- crawl phase, running at the stage-18 pixel
            if s_avid_sr(17) = '1' then s_ph <= s_ph + s_step; else s_ph <= s_lph; end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Block hash + displaced, wrapped read address (stages 1..12)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_d : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx; s_bc1 <= s_bc;
            s_firstd(1) <= s_isfirst;
            for i in 2 to 13 loop s_firstd(i) <= s_firstd(i - 1); end loop;
            s_hfd(1) <= s_hfirst;
            for i in 2 to 14 loop s_hfd(i) <= s_hfd(i - 1); end loop;
            -- stage 2: key = block row & block column
            s_key2 <= s_krow & s_bc1;
            s_xd(2) <= s_x1;
            for i in 3 to 10 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- stages 3..5: hash, the roll seed enters after the first round
            s_h3 <= f_round1(s_key2);
            s_h4 <= f_round2(s_h3 xor s_seed);
            s_h5 <= f_round3(s_h4);
            -- stage 6: moshed decision, offset product ((h - 128) * shift)
            if s_h5(15 downto 8) < s_dens then s_mosh6 <= '1'; else s_mosh6 <= '0'; end if;
            s_po6 <= signed((not s_h5(7)) & s_h5(6 downto 0)) * signed('0' & s_shift);
            -- stage 7: offset (+-1012 px HD, +-506 SD)
            if s_hd = '1' then v_d := resize(s_po6(16 downto 5), 13);
            else v_d := resize(s_po6(16 downto 6), 13); end if;
            s_d7 <= v_d;
            s_moshd(7) <= s_mosh6;
            for i in 8 to 16 loop s_moshd(i) <= s_moshd(i - 1); end loop;
            -- stage 8: displaced column; wrap candidates' offsets
            s_u8  <= signed(resize(s_xd(7), 13)) + s_d7;
            s_dp8 <= s_d7 + signed(resize(s_span, 13));
            s_dm8 <= s_d7 - signed(resize(s_span, 13));
            -- stage 9: wrap candidates, range tests
            s_u9  <= s_u8;
            s_up9 <= signed(resize(s_xd(8), 13)) + s_dp8;
            s_um9 <= signed(resize(s_xd(8), 13)) + s_dm8;
            if s_u8 < C_EDGE then s_lt9 <= '1'; else s_lt9 <= '0'; end if;
            if s_u8 >= signed(resize(s_hi, 13)) then s_ge9 <= '1'; else s_ge9 <= '0'; end if;
            -- stage 10: wrapped displaced column
            if s_lt9 = '1' then s_w10 <= unsigned(s_up9(10 downto 0));
            elsif s_ge9 = '1' then s_w10 <= unsigned(s_um9(10 downto 0));
            else s_w10 <= unsigned(s_u9(10 downto 0)); end if;
            -- stage 11: read address (moshed blocks displaced)
            if s_moshd(10) = '1' then s_a11 <= s_w10; else s_a11 <= s_xd(10); end if;
            -- stage 12
            s_a12 <= s_a11;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 2, read stage 12 -> data stage 13)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_a12));
            if s_wc2 = '1' and s_wp2 = '0' then lbY0(to_integer(s_wx2)) <= std_logic_vector(s_y2); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_a12));
            if s_wc2 = '1' and s_wp2 = '1' then lbY1(to_integer(s_wx2)) <= std_logic_vector(s_y2); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_a12(10 downto 1)));
            if s_wc2 = '1' and s_cwe2 = '1' and s_cwp2 = '0' then lbU0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pu(15 downto 8)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_a12(10 downto 1)));
            if s_wc2 = '1' and s_cwe2 = '1' and s_cwp2 = '1' then lbU1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pu(15 downto 8)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_a12(10 downto 1)));
            if s_wc2 = '1' and s_cwe2 = '1' and s_cwp2 = '0' then lbV0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pv(15 downto 8)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_a12(10 downto 1)));
            if s_wc2 = '1' and s_cwe2 = '1' and s_cwp2 = '1' then lbV1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pv(15 downto 8)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 14..22): fetch, post hold, post bleed, crawl, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_nu, v_nv : integer range 0 to 7;
        variable v_a : signed(10 downto 0);
        variable v_au, v_av : unsigned(9 downto 0);
        variable v_t, v_tri : unsigned(6 downto 0);
        variable v_y : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 14: fetch; S7 Hold keeps a moshed block's first read
            if not (s_sw_hold = '1' and s_moshd(13) = '1' and s_firstd(13) = '0') then
                if s_rbank = '1' then s_fy14 <= unsigned(s_rdY1); else s_fy14 <= unsigned(s_rdY0); end if;
                if s_crbank = '1' then
                    s_fu14 <= unsigned(s_rdU1); s_fv14 <= unsigned(s_rdV1);
                else
                    s_fu14 <= unsigned(s_rdU0); s_fv14 <= unsigned(s_rdV0);
                end if;
            end if;
            -- stage 15: post hold (After), on the output column
            if s_sw_pre = '1' or s_hfd(14) = '1' then
                s_hu15 <= s_fu14 & "00"; s_hv15 <= s_fv14 & "00";
            end if;
            s_y15 <= s_fy14;
            -- stage 16: post bleed (After)
            v_nu := to_integer(s_bn_qu); v_nv := to_integer(s_bn_qv);
            if s_avid_sr(14) = '0' or s_sw_pre = '1' then
                s_qu <= s_hu15 & "000000"; s_qv <= s_hv15 & "000000";
            else
                s_qu <= s_qu - shift_right(s_qu, v_nu) + shift_right(s_hu15 & "000000", v_nu);
                s_qv <= s_qv - shift_right(s_qv, v_nv) + shift_right(s_hv15 & "000000", v_nv);
            end if;
            s_y16 <= s_y15;
            -- stage 17: chroma out (grey in moshed blocks), centred copies
            s_grey17 <= s_sw_grey and s_moshd(16);
            if s_sw_grey = '1' and s_moshd(16) = '1' then
                s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_u17 <= s_qu(15 downto 6); s_v17 <= s_qv(15 downto 6);
            end if;
            s_du17 <= signed('0' & s_qu(15 downto 6)) - to_signed(512, 11);
            s_dv17 <= signed('0' & s_qv(15 downto 6)) - to_signed(512, 11);
            s_y17 <= s_y16;
            -- stage 18: saturation |du| + |dv| of the output (|d| reaches 512: slice)
            if s_du17 < 0 then v_a := -s_du17; else v_a := s_du17; end if;
            v_au := unsigned(v_a(9 downto 0));
            if s_dv17 < 0 then v_a := -s_dv17; else v_a := s_dv17; end if;
            v_av := unsigned(v_a(9 downto 0));
            if s_grey17 = '1' then s_sat18 <= (others => '0');
            else s_sat18 <= resize(v_au, 11) + resize(v_av, 11); end if;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            -- stage 19: crawl amplitude; triangle wave -64..63 at this pixel's phase
            s_amp19 <= s_sat18(10 downto 2) * s_cg;
            if s_ph(7) = '1' then v_t := not s_ph(6 downto 0); else v_t := s_ph(6 downto 0); end if;
            v_tri := (not v_t(6)) & v_t(5 downto 0);
            s_tri19 <= signed(v_tri);
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            -- stage 20: crawl = amplitude x wave
            s_pc20 <= signed('0' & s_amp19(16 downto 6)) * s_tri19;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            -- stage 21: luma + crawl
            v_y := signed(resize(s_y20, 13)) + resize(shift_right(s_pc20, 6), 13);
            s_vy21 <= v_y;
            s_u21 <= s_u20; s_v21 <= s_v20;
            -- stage 22: clamp to 64..940, per-row U/V parity fix, blanking gate
            if s_avid_sr(20) = '1' then
                if s_vy21 < 64 then s_y22 <= to_unsigned(64, 10);
                elsif s_vy21 > 940 then s_y22 <= to_unsigned(940, 10);
                else s_y22 <= unsigned(s_vy21(9 downto 0)); end if;
                if s_rowswap = '1' then
                    s_u22 <= s_v21; s_v22 <= s_u21;
                else
                    s_u22 <= s_u21; s_v22 <= s_v21;
                end if;
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y22);
    s_io_align_in.u       <= std_logic_vector(s_u22);
    s_io_align_in.v       <= std_logic_vector(s_v22);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (22 -> 24).
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

end architecture subsample;
