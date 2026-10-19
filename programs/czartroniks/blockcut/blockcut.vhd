-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Blockcut: catalogue combo #32 (seed 20260917) -- edge detect + datamosh /
-- block displacement.  Three knobs each; the switches and the slider
-- decide how the two combine.
--
--   K1 Gain     edge gain
--   K2 Thresh   edges below this are dropped
--   K3 Blend    edges on black (0) .. edges over the cut-up picture (100)
--   K4 Block    block size 8 .. 64 px (continuous)
--   K5 Shift    block displacement amount (floor: moshed blocks always move)
--   K6 Rate     re-roll rate, every ~2 s .. every field (exponential)
--
--   S7  Edges   white / tinted with the picture's colour
--   S8  Blocks  plain / moshed blocks go negative (luma and chroma)
--   S9  Hash    per block / per band (whole rows of blocks share a shift)
--   S10 Mosh    shift / hold (block repeats its first column)
--   S11 Ink     light edges on dark / dark edges on light
--   P12 Density fraction of blocks moshed (~5% .. ~98%); half as many again
--               keep their luma and take their colour from the displaced spot
--
-- v0.3.0: 3x3 Sobel edges (|Gx| + |Gy|) / 4 from the input and a single-bank
-- two-line Y buffer ({y[n-1], y[n-2]} in one 20-bit word), written as an
-- 8-bit edge line at the window centre (x - 1).  The picture (Y + pair-rate
-- 8-bit U/V) has its own single-bank line buffer.  Each block's hash byte
-- picks moshed (b < Density: luma, edges and chroma displaced) or colour-only
-- (Density <= b < 1.5 Density: chroma displaced, luma and edges in place);
-- luma/edges and chroma have separate read addresses, both wrapped inside the
-- C_EDGE inset.  Background = paper .. picture (Blend), then the edge
-- strength keys the ink over it.
--
-- Read/write collision freedom (no_rw_check on every RAM): the displaced
-- reads are issued at stage 12, the edge line is written at stage 10 (one
-- column ahead: centre x - 1) and the picture at stage 2 (ten ahead).
-- Offsets and the wrap span are multiples of 4, so a read never meets a
-- write address, including the pair-rate chroma (10 = 2 mod 4).  Hold
-- latches the block's first read instead of re-addressing.
-- Latency 23 clocks, all modes, blanking-gated.
--
-- v0.3.2 (2026-10-17): resolution scaling.  The block grid counts virtual
-- 1080-line units, stepped per pixel / per field line from the video timing
-- ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p, and
-- at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame line
-- (interlaced fields step two frame lines, as the old row counter did).  A
-- block is the same fraction of the screen height in every mode and stays
-- square on a 4:3 screen.  The displacement is in the same units: the HD
-- shift switch (width bit 10) used to give 720p the 1080p pixel offsets, so
-- the 720p shift is now 2/3 of them (SD already used half).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture blockcut of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (23 + 1 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 23;
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

    -- line buffers, one bank each
    type t_lb20 is array (0 to 2047) of std_logic_vector(19 downto 0);
    type t_lb10 is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb8  is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_lc8  is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbP : t_lb20 := (others => x"10040");         -- {y[n-1], y[n-2]} (Sobel rows)
    signal lbY : t_lb10 := (others => "0001000000");    -- picture Y
    signal lbU : t_lc8  := (others => x"80");           -- picture U, pair rate
    signal lbV : t_lc8  := (others => x"80");           -- picture V, pair rate
    signal lbE : t_lb8  := (others => x"00");           -- edge strength
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbP, lbY, lbU, lbV, lbE : signal is true;
    signal s_rdP : std_logic_vector(19 downto 0) := (others => '0');
    signal s_rdY : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU, s_rdV : std_logic_vector(7 downto 0) := (others => '0');
    signal s_rdE : std_logic_vector(7 downto 0) := (others => '0');

    -- controls
    signal s_k_gain, s_k_thr, s_k_blend, s_k_block, s_k_shift, s_k_rate : unsigned(9 downto 0);
    signal s_sw_tint, s_sw_neg, s_sw_band, s_sw_hold, s_sw_dark : std_logic;
    signal s_p_density : unsigned(9 downto 0);

    -- per-frame (vblank sequencer)
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_lk_g, s_lk_4, s_lk_5, s_lk_p : unsigned(9 downto 0) := (others => '0');
    signal s_lw     : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_gain   : unsigned(8 downto 0) := (others => '0');
    signal s_thr, s_blend, s_shift, s_dens, s_dens2 : unsigned(7 downto 0) := (others => '0');
    signal s_d2t    : unsigned(8 downto 0) := (others => '0');
    signal s_bsz    : unsigned(6 downto 0) := to_unsigned(8, 7);     -- block size 8..64
    signal s_period : unsigned(6 downto 0) := (others => '0');
    signal s_rate_cnt : unsigned(6 downto 0) := (others => '0');
    signal s_roll_due : std_logic := '0';
    signal s_roll, s_sd, s_seed : unsigned(15 downto 0) := (others => '0');
    signal s_hi     : unsigned(10 downto 0) := to_unsigned(708, 11);  -- width - C_EDGE
    signal s_span   : unsigned(10 downto 0) := to_unsigned(696, 11);  -- wrap span, multiple of 4
    signal s_hd     : std_logic := '0';
    signal s_sha, s_shb : unsigned(7 downto 0) := (others => '0');   -- 720p shift x 2/3 parts
    -- resolution steps (free-running from the timing ID)
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);

    -- tracking
    signal s_rx, s_line : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_ilace, s_fld, s_fld_prev, s_fsrc, s_ftog : std_logic := '0';
    signal s_top    : std_logic := '0';

    -- block grid: column DDA (stage 0), row DDA (per line), in virtual
    -- units: column Q.1, row Q.8
    signal s_cca    : unsigned(7 downto 0) := (others => '0');
    signal s_bc     : unsigned(7 downto 0) := (others => '0');
    signal s_isfirst : std_logic := '1';
    signal s_rca, s_rcn, s_rcw : unsigned(14 downto 0) := (others => '0');
    signal s_br, s_krow : unsigned(7 downto 0) := (others => '0');

    -- edge side, Sobel (stages 1..10)
    signal s_in_y   : unsigned(9 downto 0) := (others => '0');
    signal s_in_u, s_in_v : unsigned(7 downto 0) := x"80";
    signal s_y2, s_r1_2 : unsigned(9 downto 0) := (others => '0');
    signal s_s02    : unsigned(10 downto 0) := (others => '0');
    signal s_dc2, s_dc3, s_dA, s_dB : signed(10 downto 0) := (others => '0');
    signal s_c3, s_cA, s_cB : unsigned(11 downto 0) := (others => '0');
    signal s_gx4    : signed(12 downto 0) := (others => '0');
    signal s_gya4   : signed(11 downto 0) := (others => '0');
    signal s_dmid4  : signed(10 downto 0) := (others => '0');
    signal s_gx5, s_gy5 : signed(12 downto 0) := (others => '0');
    signal s_ax6, s_ay6 : unsigned(11 downto 0) := (others => '0');
    signal s_sum7   : unsigned(12 downto 0) := (others => '0');
    signal s_pe8    : unsigned(18 downto 0) := (others => '0');
    signal s_e9     : signed(14 downto 0) := (others => '0');
    signal s_side9  : std_logic := '0';
    signal s_e10    : unsigned(7 downto 0) := (others => '0');
    signal s_ewa10  : unsigned(10 downto 0) := (others => '0');
    signal s_we10   : std_logic := '0';
    type t_wx is array (1 to 9) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wc : std_logic_vector(1 to 9) := (others => '0');

    -- picture write (stage 2)
    signal s_pu2, s_pv2 : unsigned(7 downto 0) := x"80";

    -- address / hash chain (stages 1..12)
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_bc1    : unsigned(7 downto 0) := (others => '0');
    signal s_key2   : unsigned(15 downto 0) := (others => '0');
    signal s_h3, s_h4, s_h5 : unsigned(15 downto 0) := (others => '0');
    signal s_mosh6, s_cmosh6 : std_logic := '0';
    signal s_po6    : signed(16 downto 0) := (others => '0');
    signal s_d7     : signed(12 downto 0) := (others => '0');
    signal s_u8, s_dp8, s_dm8 : signed(12 downto 0) := (others => '0');
    signal s_u9, s_up9, s_um9 : signed(12 downto 0) := (others => '0');
    signal s_lt9, s_ge9 : std_logic := '0';
    signal s_w10    : unsigned(10 downto 0) := (others => '0');
    signal s_aY11, s_aC11, s_aY12, s_aC12 : unsigned(10 downto 0) := (others => '0');
    type t_xd is array (2 to 10) of unsigned(10 downto 0);
    signal s_xd     : t_xd := (others => (others => '0'));
    signal s_moshd  : std_logic_vector(7 to 20) := (others => '0');
    signal s_cmoshd : std_logic_vector(7 to 10) := (others => '0');
    signal s_firstd : std_logic_vector(1 to 13) := (others => '1');

    -- display side (stages 14..23)
    signal s_fy14   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_fu14, s_fv14 : unsigned(7 downto 0) := x"80";
    signal s_fe14   : unsigned(7 downto 0) := (others => '0');
    signal s_dy15, s_du15, s_dv15 : signed(10 downto 0) := (others => '0');
    signal s_iny15, s_inu15, s_inv15 : unsigned(9 downto 0) := (others => '0');
    signal s_pap15  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_a15    : unsigned(7 downto 0) := (others => '0');
    signal s_py16, s_pu16, s_pv16 : signed(19 downto 0) := (others => '0');
    signal s_iny16, s_inu16, s_inv16 : unsigned(9 downto 0) := (others => '0');
    signal s_pap16  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_a16    : unsigned(7 downto 0) := (others => '0');
    signal s_by17, s_bu17, s_bv17 : unsigned(9 downto 0) := (others => '0');
    signal s_iny17, s_inu17, s_inv17 : unsigned(9 downto 0) := (others => '0');
    signal s_a17    : unsigned(7 downto 0) := (others => '0');
    signal s_iy18, s_iu18, s_iv18 : signed(10 downto 0) := (others => '0');
    signal s_by18, s_bu18, s_bv18 : unsigned(9 downto 0) := (others => '0');
    signal s_a18    : unsigned(7 downto 0) := (others => '0');
    signal s_qy19, s_qu19, s_qv19 : signed(19 downto 0) := (others => '0');
    signal s_by19, s_bu19, s_bv19 : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_gain  <= unsigned(registers_in(0));
    s_k_thr   <= unsigned(registers_in(1));
    s_k_blend <= unsigned(registers_in(2));
    s_k_block <= unsigned(registers_in(3));
    s_k_shift <= unsigned(registers_in(4));
    s_k_rate  <= unsigned(registers_in(5));
    s_sw_tint <= registers_in(6)(0);
    s_sw_neg  <= registers_in(6)(1);
    s_sw_band <= registers_in(6)(2);
    s_sw_hold <= registers_in(6)(3);
    s_sw_dark <= registers_in(6)(4);
    s_p_density <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Control, tracking, block grid, edge side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_ag : signed(12 downto 0);
        variable v_nc : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; s_fld <= data_in.field_n;
            else s_rx <= (others => '0'); end if;
            s_in_y <= unsigned(data_in.y);
            s_in_u <= unsigned(data_in.u(9 downto 2)); s_in_v <= unsigned(data_in.v(9 downto 2));
            -- the 3x3 window needs two lines of this field above it
            if s_line < 2 then s_top <= '1'; else s_top <= '0'; end if;

            -- block column DDA: state for the pixel now on data_in
            -- (virtual half units; at 1080p the step is 2 and this is the old
            -- cc = bsz - 1 wrap)
            if data_in.avid = '1' then
                v_nc := s_cca + s_xs;
                if v_nc(7 downto 1) >= s_bsz then
                    s_cca <= v_nc - (s_bsz & '0'); s_bc <= s_bc + 1; s_isfirst <= '1';
                else
                    s_cca <= v_nc; s_isfirst <= '0';
                end if;
            else
                s_cca <= (others => '0'); s_bc <= (others => '0'); s_isfirst <= '1';
            end if;
            -- block row DDA candidates (rc changes once per line); the Q.8
            -- field-line step already counts frame lines under interlace
            s_rcn <= s_rca + s_ys;
            s_rcw <= s_rcn - (s_bsz & x"00");

            -- Sobel, rows r0 = y[n-2], r1 = y[n-1], r2 = y[n] (stage 1)
            -- stage 2: outer-row sum and difference per column
            s_y2   <= s_in_y;
            s_r1_2 <= unsigned(s_rdP(19 downto 10));
            s_s02  <= resize(unsigned(s_rdP(9 downto 0)), 11) + resize(s_in_y, 11);
            s_dc2  <= signed('0' & s_in_y) - signed('0' & s_rdP(9 downto 0));
            -- stage 3: column sum r0 + 2 r1 + r2; column history x-1, x-2
            s_c3  <= resize(s_s02, 12) + shift_left(resize(s_r1_2, 12), 1);
            s_dc3 <= s_dc2;
            s_cA <= s_c3; s_cB <= s_cA;
            s_dA <= s_dc3; s_dB <= s_dA;
            -- stage 4: Gx = c[x] - c[x-2]; Gy outer taps (centre x-1)
            s_gx4   <= signed('0' & s_c3) - signed('0' & s_cB);
            s_gya4  <= resize(s_dc3, 12) + resize(s_dB, 12);
            s_dmid4 <= s_dA;
            -- stage 5: Gy = d[x] + 2 d[x-1] + d[x-2]
            s_gx5 <= s_gx4;
            s_gy5 <= resize(s_gya4, 13) + shift_left(resize(s_dmid4, 13), 1);
            -- stage 6: abs (|G| reaches 4092: slice, never resize)
            if s_gx5 < 0 then v_ag := -s_gx5; else v_ag := s_gx5; end if;
            s_ax6 <= unsigned(v_ag(11 downto 0));
            if s_gy5 < 0 then v_ag := -s_gy5; else v_ag := s_gy5; end if;
            s_ay6 <= unsigned(v_ag(11 downto 0));
            -- stage 7: sum
            s_sum7 <= resize(s_ax6, 13) + resize(s_ay6, 13);
            -- stage 8: gain on (sum / 4) / 2 -- same calibration as v0.2's 2-tap
            s_pe8 <= s_sum7(12 downto 3) * s_gain;
            -- stage 9: threshold; the capture's black side columns and the top
            -- two lines of a field draw no edge
            s_e9 <= signed(resize(s_pe8(18 downto 5), 15)) - signed(resize(s_thr, 15));
            if s_wx(8) < C_EDGE or s_wx(8) >= s_hi or s_top = '1' then s_side9 <= '1'; else s_side9 <= '0'; end if;
            -- stage 10: 8-bit edge, written at the window centre
            if s_e9 < 0 or s_side9 = '1' then s_e10 <= (others => '0');
            elsif s_e9 > 255 then s_e10 <= x"FF";
            else s_e10 <= unsigned(s_e9(7 downto 0)); end if;
            s_ewa10 <= s_wx(9) - 1; s_we10 <= s_wc(9);
            s_wx(1) <= s_rx; s_wc(1) <= data_in.avid;
            for i in 2 to 9 loop s_wx(i) <= s_wx(i - 1); s_wc(i) <= s_wc(i - 1); end loop;

            -- picture write data, stage 2
            s_pu2 <= s_in_u; s_pv2 <= s_in_v;

            -- per-frame sequencer (one small op per state)
            case to_integer(s_vstep) is
                when 0 =>
                    s_thr   <= s_k_thr(9 downto 2);
                    s_blend <= s_k_blend(9 downto 2);
                    s_lk_g <= s_k_gain; s_lk_4 <= s_k_block; s_lk_5 <= s_k_shift; s_lk_p <= s_p_density;
                    s_lw <= s_line_width;
                    if s_rate_cnt >= s_period then
                        s_rate_cnt <= (others => '0'); s_roll_due <= '1';
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
                    s_period <= C_PERIOD(to_integer(s_k_rate(9 downto 5)));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_gain  <= resize(s_lk_g(9 downto 2), 9) + resize(s_lk_g(9 downto 5), 9);
                    s_dens  <= s_lk_p(9 downto 2) - resize(s_lk_p(9 downto 6), 8);
                    s_shift <= s_lk_5(9 downto 2) - resize(s_lk_5(9 downto 6), 8);
                    s_bsz   <= resize(s_lk_4(9 downto 4), 7) - resize(s_lk_4(9 downto 7), 7);
                    s_hi    <= s_lw - C_EDGE;
                    -- interlace: re-roll only ahead of the top field
                    if s_roll_due = '1' and (s_ilace = '0' or (s_fsrc = '1' and s_fld = '0') or (s_fsrc = '0' and s_ftog = '0')) then
                        s_roll <= s_roll + 1; s_roll_due <= '0';
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_gain  <= s_gain + resize(s_lk_g(9 downto 7), 9);
                    s_dens  <= s_dens + 12;
                    s_shift <= s_shift + 12;
                    s_bsz   <= s_bsz + 8;
                    s_span  <= s_lw - 2 * C_EDGE;
                    s_sd    <= f_round1(s_roll xor x"5A3C");
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_span  <= s_span(10 downto 2) & "00";
                    -- pixel offsets: HD class (1080 and 720) shifts 1/32 of
                    -- the product, SD 1/64; 720p scales the amount by 2/3
                    if s_xs = 4 then s_hd <= '0'; else s_hd <= '1'; end if;
                    s_sha   <= ('0' & s_shift(7 downto 1)) + ("000" & s_shift(7 downto 3));
                    s_shb   <= ("00000" & s_shift(7 downto 5)) + ("0000000" & s_shift(7 downto 7));
                    s_sd    <= f_round2(s_sd);
                    s_d2t   <= resize(s_dens, 9) + resize(s_dens(7 downto 1), 9);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_seed  <= f_round3(s_sd);
                    -- colour-only band: Density .. 1.5 Density
                    if s_d2t(8) = '1' then s_dens2 <= x"FF"; else s_dens2 <= s_d2t(7 downto 0); end if;
                    if s_xs = 3 then s_shift <= s_sha + s_shb; end if;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- line events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                if s_rcn(14 downto 8) >= s_bsz then s_rca <= s_rcw; s_br <= s_br + 1; else s_rca <= s_rcn; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_krow <= s_br;
            end if;
            -- frame events: resets are idempotent; toggles need the saw-active guard
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_br <= (others => '0');
                -- a bottom field (frame lines 1, 3, ..) follows a top field:
                -- it starts half a field step (one frame line) down
                if (s_ilace and s_fsrc and s_fld) = '1' then
                    s_rca <= resize(s_ys(11 downto 1), 15);
                else
                    s_rca <= (others => '0');
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

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Block hash + displaced, wrapped read addresses (stages 1..12)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_d : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx; s_bc1 <= s_bc;
            s_firstd(1) <= s_isfirst;
            for i in 2 to 13 loop s_firstd(i) <= s_firstd(i - 1); end loop;
            -- stage 2: key = block row & block column (row only per band)
            if s_sw_band = '1' then
                s_key2 <= s_krow & x"00";
            else
                s_key2 <= s_krow & s_bc1;
            end if;
            s_xd(2) <= s_x1;
            for i in 3 to 10 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- stages 3..5: hash, the roll seed enters after the first round
            s_h3 <= f_round1(s_key2);
            s_h4 <= f_round2(s_h3 xor s_seed);
            s_h5 <= f_round3(s_h4);
            -- stage 6: moshed / colour-only decision, offset product ((h - 128) * shift)
            if s_h5(15 downto 8) < s_dens then s_mosh6 <= '1'; else s_mosh6 <= '0'; end if;
            if s_h5(15 downto 8) < s_dens2 then s_cmosh6 <= '1'; else s_cmosh6 <= '0'; end if;
            s_po6 <= signed((not s_h5(7)) & s_h5(6 downto 0)) * signed('0' & s_shift);
            -- stage 7: offset, a multiple of 4 (+-1008 px HD, +-504 SD)
            if s_hd = '1' then v_d := resize(s_po6(16 downto 5), 13);
            else v_d := resize(s_po6(16 downto 6), 13); end if;
            s_d7 <= v_d(12 downto 2) & "00";
            s_moshd(7) <= s_mosh6;
            for i in 8 to 20 loop s_moshd(i) <= s_moshd(i - 1); end loop;
            s_cmoshd(7) <= s_cmosh6;
            for i in 8 to 10 loop s_cmoshd(i) <= s_cmoshd(i - 1); end loop;
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
            -- stage 11: luma/edge address (moshed) and chroma address (moshed or colour-only)
            if s_moshd(10) = '1' then s_aY11 <= s_w10; else s_aY11 <= s_xd(10); end if;
            if s_cmoshd(10) = '1' then s_aC11 <= s_w10; else s_aC11 <= s_xd(10); end if;
            -- stage 12: read addresses
            s_aY12 <= s_aY11; s_aC12 <= s_aC11;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (one write, one read each)
    --   P: read at rx (stage 0), written stage 2   -> two lines above x
    --   Y/U/V: written stage 2, read stage 12      -> data stage 13
    --   E: written stage 10 (centre x - 1), read stage 12 -> data stage 13
    --------------------------------------------------------------------------
    p_lbP : process(clk)
    begin
        if rising_edge(clk) then
            s_rdP <= lbP(to_integer(s_rx));
            -- {y[n], y[n-1]} from fabric registers (pipelined read-modify-write)
            if s_wc(2) = '1' then
                lbP(to_integer(s_wx(2))) <= std_logic_vector(s_y2) & std_logic_vector(s_r1_2);
            end if;
        end if;
    end process;
    p_lbY : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY <= lbY(to_integer(s_aY12));
            if s_wc(2) = '1' then lbY(to_integer(s_wx(2))) <= std_logic_vector(s_y2); end if;
        end if;
    end process;
    p_lbU : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU <= lbU(to_integer(s_aC12(10 downto 1)));
            if s_wc(2) = '1' then lbU(to_integer(s_wx(2)(10 downto 1))) <= std_logic_vector(s_pu2); end if;
        end if;
    end process;
    p_lbV : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV <= lbV(to_integer(s_aC12(10 downto 1)));
            if s_wc(2) = '1' then lbV(to_integer(s_wx(2)(10 downto 1))) <= std_logic_vector(s_pv2); end if;
        end if;
    end process;
    p_lbE : process(clk)
    begin
        if rising_edge(clk) then
            s_rdE <= lbE(to_integer(s_aY12));
            if s_we10 = '1' then lbE(to_integer(s_ewa10)) <= std_logic_vector(s_e10); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Display pipeline (stages 14..23)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y, v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 14: fetch; Hold keeps the block's first read
            if not (s_sw_hold = '1' and s_moshd(13) = '1' and s_firstd(13) = '0') then
                s_fy14 <= unsigned(s_rdY); s_fu14 <= unsigned(s_rdU); s_fv14 <= unsigned(s_rdV);
                s_fe14 <= unsigned(s_rdE);
            end if;
            -- stage 15: picture - paper, ink, key
            if s_sw_dark = '1' then
                s_pap15 <= to_unsigned(940, 10);
                s_dy15  <= signed('0' & s_fy14) - to_signed(940, 11);
                if s_sw_tint = '1' then s_iny15 <= to_unsigned(192, 10); else s_iny15 <= to_unsigned(64, 10); end if;
            else
                s_pap15 <= to_unsigned(64, 10);
                s_dy15  <= signed('0' & s_fy14) - to_signed(64, 11);
                if s_sw_tint = '1' then s_iny15 <= to_unsigned(768, 10); else s_iny15 <= to_unsigned(940, 10); end if;
            end if;
            s_du15 <= signed(shift_left(resize(s_fu14, 11), 2)) - to_signed(512, 11);
            s_dv15 <= signed(shift_left(resize(s_fv14, 11), 2)) - to_signed(512, 11);
            -- tinted ink = picture chroma at double saturation, clamped
            if s_sw_tint = '0' then
                s_inu15 <= to_unsigned(512, 10); s_inv15 <= to_unsigned(512, 10);
            else
                case s_fu14(7 downto 6) is
                    when "00"   => s_inu15 <= (others => '0');
                    when "11"   => s_inu15 <= (others => '1');
                    when others => s_inu15 <= s_fu14(7) & s_fu14(5 downto 0) & "000";
                end case;
                case s_fv14(7 downto 6) is
                    when "00"   => s_inv15 <= (others => '0');
                    when "11"   => s_inv15 <= (others => '1');
                    when others => s_inv15 <= s_fv14(7) & s_fv14(5 downto 0) & "000";
                end case;
            end if;
            s_a15 <= s_fe14;
            -- stage 16: background products (paper .. picture by Blend)
            s_py16 <= s_dy15 * signed('0' & s_blend);
            s_pu16 <= s_du15 * signed('0' & s_blend);
            s_pv16 <= s_dv15 * signed('0' & s_blend);
            s_iny16 <= s_iny15; s_inu16 <= s_inu15; s_inv16 <= s_inv15;
            s_pap16 <= s_pap15; s_a16 <= s_a15;
            -- stage 17: background
            -- (results lie between paper and picture: slice, never resize)
            v_y := signed(resize(s_pap16, 12)) + resize(shift_right(s_py16, 8), 12);
            v_u := to_signed(512, 12) + resize(shift_right(s_pu16, 8), 12);
            v_v := to_signed(512, 12) + resize(shift_right(s_pv16, 8), 12);
            s_by17 <= unsigned(v_y(9 downto 0)); s_bu17 <= unsigned(v_u(9 downto 0)); s_bv17 <= unsigned(v_v(9 downto 0));
            s_iny17 <= s_iny16; s_inu17 <= s_inu16; s_inv17 <= s_inv16; s_a17 <= s_a16;
            -- stage 18: ink - background
            s_iy18 <= signed('0' & s_iny17) - signed('0' & s_by17);
            s_iu18 <= signed('0' & s_inu17) - signed('0' & s_bu17);
            s_iv18 <= signed('0' & s_inv17) - signed('0' & s_bv17);
            s_by18 <= s_by17; s_bu18 <= s_bu17; s_bv18 <= s_bv17; s_a18 <= s_a17;
            -- stage 19: key products
            s_qy19 <= s_iy18 * signed('0' & s_a18);
            s_qu19 <= s_iu18 * signed('0' & s_a18);
            s_qv19 <= s_iv18 * signed('0' & s_a18);
            s_by19 <= s_by18; s_bu19 <= s_bu18; s_bv19 <= s_bv18;
            -- stage 20: ink over background (stays between the two, no clamp)
            v_y := signed(resize(s_by19, 12)) + resize(shift_right(s_qy19, 8), 12);
            v_u := signed(resize(s_bu19, 12)) + resize(shift_right(s_qu19, 8), 12);
            v_v := signed(resize(s_bv19, 12)) + resize(shift_right(s_qv19, 8), 12);
            s_y20 <= unsigned(v_y(9 downto 0)); s_u20 <= unsigned(v_u(9 downto 0)); s_v20 <= unsigned(v_v(9 downto 0));
            -- stage 21: moshed blocks negative (64 <-> 940, chroma mirrored about 512)
            if s_sw_neg = '1' and s_moshd(20) = '1' then
                if s_y20 > 1004 then s_y21 <= (others => '0');
                else s_y21 <= to_unsigned(1004, 10) - s_y20; end if;
                if s_u20 = 0 then s_u21 <= (others => '1'); else s_u21 <= unsigned(-signed(s_u20)); end if;
                if s_v20 = 0 then s_v21 <= (others => '1'); else s_v21 <= unsigned(-signed(s_v20)); end if;
            else
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            end if;
            -- stage 22: blanking gate
            if s_avid_sr(20) = '1' then
                s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
            -- stage 23: output
            s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y23);
    s_io_align_in.u       <= std_logic_vector(s_u23);
    s_io_align_in.v       <= std_logic_vector(s_v23);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (23 -> 24).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture blockcut;
