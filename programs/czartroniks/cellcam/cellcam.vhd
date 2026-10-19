-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Cellcam: catalogue combo #86 (seed 20260923) -- edge detect + chroma
-- subsampling artifacts + pixelation.  Two knobs each; the switches and the
-- slider decide how they combine.
--
--   K1 Crawl    NTSC dot crawl on the edges; past 50% it spreads over the
--               whole picture (0 = clean)
--   K2 Shape    cell height relative to its width: 1/4 (flat) .. square
--               (centre) .. ~4x (tall)
--   K3 Gain     edge gain, exponential 1/8x..31x (2x at 50%)
--   K4 Width    outline width 1..8 px (left side) / lines (top side); Raw:
--               edges thickened Width px right and Width lines down
--   K5 Hold     chroma held across 1..16 cells
--   K6 Smear    chroma lag 0..127 px; past 25% the chroma also bleeds
--               (eases toward each new block over 8/16/32 px, x2 at HD)
--
--   S7  Edges   drawn over the cells / edges only, on plain paper
--   S8  Source  steps between cells / gradient of the live picture
--   S9  Chroma  from the stored cell row / live from the current line
--   S10 Ink     edges black / edges white
--   S11 Tint    edges keep the cell colour / edges mono
--   P12 Cell    cell size 2..129 px, quadratic; the grid is centred on the
--               screen, so the slider zooms the pixelation about the centre
--
-- Block-held line buffer: the first line of each cell row is written into
-- the row's bank (banks by row parity) and every line of the row reads it
-- back at its cell's first column, the first line included (it reads
-- columns already written).  The other bank holds the row above, which
-- gives the top outline.  Chroma is read at its own hold-block column (or
-- latched live there); stored rows whose hsync->avid parity differs from
-- the current line's get U/V swapped (drape v1.3).  A vblank sequencer
-- sizes the grid once per field from P12 / Shape / Hold (deadbanded) and
-- centres it: start phases from (width/2) mod W and (height/2) mod R.
-- Output: luma 64..940, highlight knee (white -> 770), [1 2 1] on Y/U/V
-- (HD HDMI bars).  Latency 23 clocks, all modes, blanking-gated.
--
-- v0.3.2 (2026-10-17) resolution scaling: P12's cell size, the Shape
-- height, K4's outline/raw widths and K6's lag are sizes on the 1080-line
-- canvas, converted to pixels (x 2/3 at 720p, 1/2 at SD) and field lines
-- (1080 / active lines, halved on interlace) from the timing ID, so cells
-- keep their size relative to the screen height and stay square on 4:3 SD
-- (cells >= 2 px, outlines >= 1 px / 1 line).  The bleed's HD doubling is
-- now 1080-only (720p takes the SD lengths, the nearer power of 2), and
-- interlace comes from the timing ID instead of the field height.  The
-- grid centring is unchanged (real pixels / field lines).  1080p unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cellcam of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (23 + 1 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 23;

    -- [1 2 1] / 4 of three 10-bit taps
    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
    end function;

    function f_absd(a, b : unsigned(7 downto 0)) return unsigned is
    begin
        if a >= b then return a - b; else return b - a; end if;
    end function;

    -- cell rows (8-bit luma, 8-bit chroma at half rate) in two banks, a
    -- 1-line luma delay for the raw gradient, a 1-line raw-edge hold state
    -- {level 5, lines left 3}, a 256-entry chroma ring for the lag
    type t_lb_y is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lbY0, lbY1, lbP : t_lb_y := (others => x"10");
    signal lbD : t_lb_y := (others => x"00");
    signal lbU0, lbU1, lbV0, lbV1 : t_lb_c := (others => x"80");
    signal ringC : t_ring := (others => x"8080");
    -- no RAM reads the address its write port writes that clock: the cell
    -- banks read <= x while x+2 is written, the delay line reads x while
    -- x-1 is written, the edge hold reads x while x-3 is written, the ring
    -- reads 1+lag behind its write pointer
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0  : signal is true;
    attribute no_rw_check of lbY1  : signal is true;
    attribute no_rw_check of lbP   : signal is true;
    attribute no_rw_check of lbD   : signal is true;
    attribute no_rw_check of lbU0  : signal is true;
    attribute no_rw_check of lbU1  : signal is true;
    attribute no_rw_check of lbV0  : signal is true;
    attribute no_rw_check of lbV1  : signal is true;
    attribute no_rw_check of ringC : signal is true;
    signal s_rdY0, s_rdY1, s_rdP : std_logic_vector(7 downto 0) := x"10";
    signal s_rdD : std_logic_vector(7 downto 0) := x"00";
    signal s_rdU0, s_rdU1, s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdC : std_logic_vector(15 downto 0) := x"8080";

    -- controls
    signal s_sw_only, s_sw_raw, s_sw_live, s_sw_white, s_sw_mono : std_logic;
    signal s_p12r, s_k2r, s_k5r : unsigned(9 downto 0) := (others => '0');
    signal s_p12h, s_k2h, s_k5h : unsigned(9 downto 0) := (others => '0');   -- deadbanded
    signal s_w7 : unsigned(12 downto 0) := (others => '0');
    signal s_wq : unsigned(2 downto 0) := (others => '0');      -- outline width - 1
    signal s_w : unsigned(3 downto 0) := to_unsigned(1, 4);     -- outline width
    signal s_h15 : unsigned(13 downto 0) := (others => '0');
    signal s_gexp : unsigned(2 downto 0) := (others => '0');
    signal s_gman : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_lag : unsigned(6 downto 0) := (others => '0');
    signal s_bk0, s_bk : unsigned(2 downto 0) := (others => '0');   -- bleed shift (0 = none)
    signal s_crawl, s_cbase : unsigned(7 downto 0) := (others => '0');

    -- the cell grid for this field (committed by the sequencer in vblank)
    signal s_wm1 : unsigned(7 downto 0) := to_unsigned(7, 8);   -- cell width - 1
    signal s_c0 : unsigned(7 downto 0) := (others => '0');      -- column phase at x = 0
    signal s_hm1 : unsigned(3 downto 0) := (others => '0');     -- hold cells - 1
    signal s_h0 : unsigned(3 downto 0) := (others => '0');      -- hold phase at x = 0
    signal s_rm1 : unsigned(7 downto 0) := to_unsigned(7, 8);   -- cell rows - 1

    -- per-field sequencer
    signal s_sq : unsigned(3 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of s_sq : signal is "none";
    signal s_sk : unsigned(7 downto 0) := (others => '0');
    signal s_sksq : unsigned(15 downto 0) := (others => '0');
    signal s_se : unsigned(1 downto 0) := (others => '0');
    signal s_sm : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_wd, s_ht, s_m : unsigned(10 downto 0) := (others => '0');
    signal s_fw, s_fwm1, s_fr, s_c0n, s_r0n : unsigned(7 downto 0) := (others => '0');
    signal s_rprod : unsigned(12 downto 0) := (others => '0');
    signal s_rr : unsigned(9 downto 0) := (others => '0');
    signal s_x0z : std_logic := '0';

    -- Resolution scaling (v0.3.2): Q10 virtual-unit -> pixel (kx) and
    -- -> field-line (ky) factors from the timing ID; sizes below in px/lines
    signal s_kx, s_ky : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_x1080 : std_logic := '1';                           -- 1080p/1080i class
    signal s_fwv : unsigned(7 downto 0) := (others => '0');     -- cell width, virtual units
    signal s_pw : unsigned(18 downto 0) := (others => '0');     -- W_v * kx
    signal s_pr : unsigned(20 downto 0) := (others => '0');     -- R_v * ky
    -- products split on the factor (low 6 / high 5 bits), recombined a clock later
    signal s_pwl : unsigned(13 downto 0) := (others => '0');
    signal s_pwh : unsigned(12 downto 0) := (others => '0');
    signal s_prl : unsigned(15 downto 0) := (others => '0');
    signal s_prh : unsigned(14 downto 0) := (others => '0');
    signal s_wpl, s_wll : unsigned(9 downto 0) := (others => '0');
    signal s_wph, s_wlh : unsigned(8 downto 0) := (others => '0');
    signal s_lagl : unsigned(12 downto 0) := (others => '0');
    signal s_lagh : unsigned(11 downto 0) := (others => '0');
    signal s_wpp, s_wlp : unsigned(14 downto 0) := (others => '0');
    signal s_wpx, s_wln : unsigned(3 downto 0) := to_unsigned(1, 4);   -- outline width px / lines
    signal s_wqx, s_wqy : unsigned(2 downto 0) := (others => '0');     -- ... minus 1
    signal s_lagp : unsigned(17 downto 0) := (others => '0');
    signal s_lagx : unsigned(6 downto 0) := (others => '0');    -- lag in px
    signal s_qm, s_qm1, s_h0n, s_hm1n : unsigned(3 downto 0) := (others => '0');

    -- input side
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_vsync_n, s_prev_hsync_n, s_prev_avid : std_logic := '1';
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_line_width, s_lcnt, s_height : unsigned(10 downto 0) := (others => '0');
    signal s_hd : std_logic := '0';
    signal s_we : std_logic := '0';
    signal s_cc : unsigned(7 downto 0) := (others => '0');      -- column within the cell
    signal s_hc : unsigned(3 downto 0) := (others => '0');      -- cell within the hold block
    signal s_fc : std_logic := '1';                             -- first cell of the line
    -- line state (changes at line end / field commit) and its copy for the
    -- pixel path (taken at avid rise, so a line's tail never sees the next's)
    signal s_rc : unsigned(7 downto 0) := (others => '0');
    signal s_blk0, s_top, s_first : std_logic := '1';
    signal s_wbank, s_lph : std_logic := '0';
    signal s_saw_active : std_logic := '0';
    signal s_fph : unsigned(1 downto 0) := (others => '0');
    signal s_hpar, s_bpar : std_logic := '0';
    signal s_r_bank, s_r_band, s_r_top, s_r_first, s_r_lph, s_r_swap : std_logic := '0';

    -- pixel path (suffix = stage; s_in_* is stage 1)
    signal s_cx2, s_hx2, s_ra3, s_ca3 : unsigned(10 downto 0) := (others => '0');
    signal s_cs2, s_nb2, s_fc2, s_avd2, s_fp2 : std_logic := '0';
    signal s_cs3, s_nb3, s_fc3, s_avd3 : std_logic := '0';
    signal s_cs4, s_nb4, s_fc4, s_avd4 : std_logic := '0';
    signal s_cs5, s_nb5, s_fc5 : std_logic := '0';
    signal s_lu2, s_lv2, s_lu3, s_lv3, s_lu4, s_lv4 : unsigned(7 downto 0) := x"80";
    signal s_yr2, s_yrm, s_lp2, s_gx3, s_gy3 : unsigned(7 downto 0) := (others => '0');
    signal s_gs4, s_pk5, s_raw6, s_e7 : unsigned(8 downto 0) := (others => '0');
    signal s_pc5 : unsigned(2 downto 0) := (others => '0');
    signal s_cy5, s_oy5 : unsigned(7 downto 0) := x"10";
    signal s_cu5, s_cv5 : unsigned(7 downto 0) := x"80";
    signal s_prev, s_hst6, s_vst6, s_y6 : unsigned(7 downto 0) := (others => '0');
    signal s_nb6 : std_logic := '0';
    signal s_ub6, s_vb6 : unsigned(15 downto 0) := x"8000";     -- chroma, 10.6 fixed point
    signal s_y7, s_y8, s_y9 : unsigned(7 downto 0) := (others => '0');
    signal s_u7, s_v7, s_u8, s_v8, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ep8 : unsigned(13 downto 0) := (others => '0');
    signal s_es9 : unsigned(16 downto 0) := (others => '0');
    signal s_x8, s_x9, s_x10, s_x11 : unsigned(10 downto 0) := (others => '0');
    signal s_ev10, s_ev11, s_ev12 : unsigned(9 downto 0) := (others => '0');
    signal s_vh10 : unsigned(7 downto 0) := (others => '0');
    signal s_vs11 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_wt11 : unsigned(4 downto 0) := (others => '0');
    signal s_ud10, s_vd10, s_ud11, s_vd11 : signed(10 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10, s_y11, s_u11, s_v11, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_ys12 : signed(11 downto 0) := (others => '0');
    signal s_um12, s_vm12 : signed(16 downto 0) := (others => '0');
    signal s_hi12 : std_logic := '0';
    signal s_y13, s_u13, s_v13, s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    signal s_ca13 : unsigned(10 downto 0) := (others => '0');
    signal s_cp14 : unsigned(18 downto 0) := (others => '0');
    signal s_dx14 : unsigned(2 downto 0) := (others => '0');
    signal s_y15 : unsigned(9 downto 0) := (others => '0');
    signal s_u15, s_v15 : signed(11 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_wp, s_rp : unsigned(7 downto 0) := (others => '0');
    signal s_y17, s_y18, s_y19 : unsigned(9 downto 0) := (others => '0');
    signal s_lc19 : unsigned(15 downto 0) := x"8080";
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    signal s_yk20 : signed(10 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_ya, s_yb, s_ua, s_ub, s_va, s_vb : unsigned(9 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_sw_only  <= registers_in(6)(0);
    s_sw_raw   <= registers_in(6)(1);
    s_sw_live  <= registers_in(6)(2);
    s_sw_white <= registers_in(6)(3);
    s_sw_mono  <= registers_in(6)(4);

    p_ctrl : process(clk)
        variable v_q : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- knob decode (quasi-static)
            s_crawl <= unsigned(registers_in(0)(9 downto 2));
            if s_crawl(7) = '1' then s_cbase <= s_crawl(6 downto 0) & '0'; else s_cbase <= (others => '0'); end if;
            s_gexp <= unsigned(registers_in(2)(9 downto 7));
            s_gman <= resize(unsigned(registers_in(2)(6 downto 3)), 5) + 16;
            s_w7 <= shift_left(resize(unsigned(registers_in(3)), 13), 3)
                    - resize(unsigned(registers_in(3)), 13) + 512;            -- K4 * 7 + 1/2
            s_wq <= s_w7(12 downto 10);
            s_w <= resize(s_w7(12 downto 10), 4) + 1;                         -- 1..8
            s_lag <= unsigned(registers_in(5)(9 downto 3));
            case to_integer(unsigned(registers_in(5)(9 downto 8))) is
                when 0 => s_bk0 <= "000";
                when 1 => s_bk0 <= "011";
                when 2 => s_bk0 <= "100";
                when others => s_bk0 <= "101";
            end case;
            if s_x1080 = '1' and s_bk0 /= 0 then s_bk <= s_bk0 + 1; else s_bk <= s_bk0; end if;
            -- Resolution scaling: per-mode factors (the timing ID is static
            -- while a mode runs), then the widths and lag in px / lines
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_kx <= to_unsigned(512, 11); s_ky <= to_unsigned(230, 11);   -- 480i
                when 4          => s_kx <= to_unsigned(512, 11); s_ky <= to_unsigned(455, 11);   -- 480p
                when 8          => s_kx <= to_unsigned(512, 11); s_ky <= to_unsigned(273, 11);   -- 576i
                when 12         => s_kx <= to_unsigned(512, 11); s_ky <= to_unsigned(546, 11);   -- 576p
                when 5 | 6 | 14 => s_kx <= to_unsigned(683, 11); s_ky <= to_unsigned(683, 11);   -- 720p
                when 1 | 2 | 10 => s_kx <= to_unsigned(1024, 11); s_ky <= to_unsigned(512, 11);  -- 1080i
                when others     => s_kx <= to_unsigned(1024, 11); s_ky <= to_unsigned(1024, 11); -- 1080p
            end case;
            s_x1080 <= s_kx(10);
            s_wpl <= s_w * s_kx(5 downto 0);  s_wph <= s_w * s_kx(10 downto 6);
            s_wll <= s_w * s_ky(5 downto 0);  s_wlh <= s_w * s_ky(10 downto 6);
            s_wpp <= shift_left(resize(s_wph, 15), 6) + resize(s_wpl, 15);
            s_wlp <= shift_left(resize(s_wlh, 15), 6) + resize(s_wll, 15);
            if s_wpp(14 downto 10) + ("0000" & s_wpp(9)) = 0 then s_wpx <= to_unsigned(1, 4);
            else s_wpx <= s_wpp(13 downto 10) + ("000" & s_wpp(9)); end if;
            if s_wlp(14 downto 10) + ("0000" & s_wlp(9)) = 0 then s_wln <= to_unsigned(1, 4);
            else s_wln <= s_wlp(13 downto 10) + ("000" & s_wlp(9)); end if;
            s_wqx <= resize(s_wpx - 1, 3);
            s_wqy <= resize(s_wln - 1, 3);
            s_lagl <= s_lag * s_kx(5 downto 0);  s_lagh <= s_lag * s_kx(10 downto 6);
            s_lagp <= shift_left(resize(s_lagh, 18), 6) + resize(s_lagl, 18);
            s_lagx <= s_lagp(16 downto 10) + ("000000" & s_lagp(9));
            -- grid knobs: +-4 LSB deadband, so pot noise never jitters the grid
            s_p12r <= unsigned(registers_in(7)); s_k2r <= unsigned(registers_in(1)); s_k5r <= unsigned(registers_in(4));
            if resize(s_p12r, 11) > resize(s_p12h, 11) + 4 or resize(s_p12r, 11) + 4 < resize(s_p12h, 11) then s_p12h <= s_p12r; end if;
            if resize(s_k2r, 11) > resize(s_k2h, 11) + 4 or resize(s_k2r, 11) + 4 < resize(s_k2h, 11) then s_k2h <= s_k2r; end if;
            if resize(s_k5r, 11) > resize(s_k5h, 11) + 4 or resize(s_k5r, 11) + 4 < resize(s_k5h, 11) then s_k5h <= s_k5r; end if;
            s_h15 <= shift_left(resize(s_k5h, 14), 4) - resize(s_k5h, 14) + 512;  -- K5 * 15 + 1/2

            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid and s_blk0;

            -- cell and hold-block counters for the pixel entering stage 1;
            -- each line starts at the field's centring phases
            if s_in_avid = '1' and data_in.avid = '1' then
                if s_cc >= s_wm1 then
                    s_cc <= (others => '0'); s_fc <= '0';
                    if s_hc >= s_hm1 then s_hc <= (others => '0'); else s_hc <= s_hc + 1; end if;
                else
                    s_cc <= s_cc + 1;
                end if;
            else
                s_cc <= s_c0; s_hc <= s_h0; s_fc <= '1';
            end if;

            -- hsync->avid parity of this line
            s_hpar <= not s_hpar;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then s_hpar <= '0'; end if;

            -- line end: width, height, row counter, banks, crawl line phase
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_lcnt <= s_lcnt + 1; s_height <= s_lcnt + 1;
                if s_rc >= s_rm1 then
                    s_rc <= (others => '0'); s_blk0 <= '1'; s_wbank <= not s_wbank; s_top <= '0';
                else
                    s_rc <= s_rc + 1; s_blk0 <= '0';
                end if;
                s_first <= '0'; s_lph <= not s_lph;
            end if;
            -- line start: freeze the row state for this line's pixels
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_r_bank <= s_wbank; s_r_top <= s_top; s_r_first <= s_first; s_r_lph <= s_lph;
                if s_rc < s_wln then s_r_band <= '1'; else s_r_band <= '0'; end if;
                if s_blk0 = '1' then
                    s_bpar <= s_hpar; s_r_swap <= '0';
                else
                    s_r_swap <= s_hpar xor s_bpar;
                end if;
            end if;
            s_hd <= s_line_width(10);

            -- per field (first vsync edge after active video; serrations
            -- ignored): size and centre the grid, one op per state, then
            -- commit it and reset the field's line state
            case to_integer(s_sq) is
                when 1 =>
                    s_sksq <= s_sk * s_sk;
                    s_sq <= s_sq + 1;
                when 2 =>
                    s_fwv <= resize(s_sksq(15 downto 9), 8) + 2;               -- 2..129 virtual units
                    s_sq <= s_sq + 1;
                when 3 =>
                    s_rprod <= s_fwv * s_sm;
                    s_pwl <= s_fwv * s_kx(5 downto 0);                         -- width in px, Q10,
                    s_pwh <= s_fwv * s_kx(10 downto 6);                        -- split on kx
                    s_sq <= s_sq + 1;
                when 4 =>
                    case to_integer(s_se) is                                   -- R = W * (16+m) * 2^e / 64
                        when 0 => s_rr <= resize(s_rprod(12 downto 6), 10);
                        when 1 => s_rr <= resize(s_rprod(12 downto 5), 10);
                        when 2 => s_rr <= resize(s_rprod(12 downto 4), 10);
                        when others => s_rr <= resize(s_rprod(12 downto 3), 10);
                    end case;
                    s_pw <= shift_left(resize(s_pwh, 19), 6) + resize(s_pwl, 19);
                    s_sq <= s_sq + 1;
                when 5 =>
                    if s_pw(18 downto 10) + ("00000000" & s_pw(9)) < 2 then
                        s_fw <= to_unsigned(2, 8);                             -- cells >= 2 px
                    else
                        s_fw <= s_pw(17 downto 10) + ("0000000" & s_pw(9));
                    end if;
                    s_prl <= s_rr * s_ky(5 downto 0);                          -- rows in field lines,
                    s_prh <= s_rr * s_ky(10 downto 6);                         -- Q10, split on ky
                    s_sq <= s_sq + 1;
                when 6 =>
                    s_fwm1 <= s_fw - 1;
                    s_pr <= shift_left(resize(s_prh, 21), 6) + resize(s_prl, 21);
                    s_sq <= s_sq + 1;
                when 7 =>
                    if s_pr(20 downto 10) + ("0000000000" & s_pr(9)) = 0 then s_fr <= to_unsigned(1, 8);
                    elsif s_pr(20 downto 10) + ("0000000000" & s_pr(9)) > 255 then s_fr <= to_unsigned(255, 8);
                    else s_fr <= s_pr(17 downto 10) + ("0000000" & s_pr(9)); end if;
                    s_sq <= s_sq + 1;
                when 8 =>
                    s_m <= shift_right(s_wd, 1); s_qm <= (others => '0');
                    s_sq <= s_sq + 1;
                when 9 =>                                                      -- (width/2) mod W, cells mod H
                    if s_m >= s_fw then
                        s_m <= s_m - s_fw;
                        if s_qm >= s_hm1n then s_qm <= (others => '0'); else s_qm <= s_qm + 1; end if;
                    else
                        s_sq <= s_sq + 1;
                    end if;
                when 10 =>
                    if s_m = 0 then
                        s_c0n <= (others => '0'); s_x0z <= '1';
                    else
                        s_c0n <= s_fw - s_m(7 downto 0); s_x0z <= '0';
                    end if;
                    if s_qm >= s_hm1n then s_qm1 <= (others => '0'); else s_qm1 <= s_qm + 1; end if;
                    s_sq <= s_sq + 1;
                when 11 =>
                    if s_x0z = '1' then v_q := s_qm; else v_q := s_qm1; end if;
                    if v_q = 0 then s_h0n <= (others => '0'); else s_h0n <= s_hm1n + 1 - v_q; end if;
                    s_sq <= s_sq + 1;
                when 12 =>
                    s_m <= shift_right(s_ht, 1);
                    s_sq <= s_sq + 1;
                when 13 =>                                                     -- (height/2) mod R
                    if s_m >= s_fr then s_m <= s_m - s_fr; else s_sq <= s_sq + 1; end if;
                when 14 =>
                    if s_m = 0 then s_r0n <= (others => '0'); else s_r0n <= s_fr - s_m(7 downto 0); end if;
                    s_sq <= s_sq + 1;
                when 15 =>
                    s_wm1 <= s_fwm1; s_c0 <= s_c0n; s_hm1 <= s_hm1n; s_h0 <= s_h0n; s_rm1 <= s_fr - 1;
                    s_rc <= s_r0n; s_blk0 <= '1'; s_wbank <= '0'; s_top <= '1'; s_first <= '1'; s_lph <= '0';
                    s_sq <= (others => '0');
                when others => null;
            end case;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw_active = '1' then
                s_sk <= s_p12h(9 downto 2); s_wd <= s_line_width; s_ht <= s_height;
                s_hm1n <= s_h15(13 downto 10);
                s_se <= s_k2h(9 downto 8); s_sm <= resize(s_k2h(7 downto 4), 5) + 16;
                s_lcnt <= (others => '0'); s_fph <= s_fph + 1; s_saw_active <= '0';
                s_sq <= to_unsigned(1, 4);
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ra3));
            if s_we = '1' and s_wbank = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y(9 downto 2)); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ra3));
            if s_we = '1' and s_wbank = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y(9 downto 2)); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_ca3(10 downto 1)));
            if s_we = '1' and s_wbank = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_ca3(10 downto 1)));
            if s_we = '1' and s_wbank = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_ca3(10 downto 1)));
            if s_we = '1' and s_wbank = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_ca3(10 downto 1)));
            if s_we = '1' and s_wbank = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    -- previous line, read at stage 0 (column x) while stage 1 writes x-1
    p_lbP : process(clk)
    begin
        if rising_edge(clk) then
            s_rdP <= lbP(to_integer(s_rx));
            if s_in_avid = '1' then lbP(to_integer(s_wr_x)) <= std_logic_vector(s_in_y(9 downto 2)); end if;
        end if;
    end process;
    -- raw-edge hold from the line above: read at stage 8, written back at
    -- stage 11 (same column, 3 clocks later)
    p_lbD : process(clk)
    begin
        if rising_edge(clk) then
            s_rdD <= lbD(to_integer(s_x8));
            if s_avid_sr(10) = '1' then lbD(to_integer(s_x11)) <= s_vs11; end if;
        end if;
    end process;
    -- chroma lag ring: written at stage 16, read 1 + lag back
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_wp <= s_wp + 1;
            s_rp <= s_wp - resize(s_lagx, 8);
            s_rdC <= ringC(to_integer(s_rp));
            ringC(to_integer(s_wp)) <= std_logic_vector(s_u16(9 downto 2)) & std_logic_vector(s_v16(9 downto 2));
        end if;
    end process;

    p_pix : process(clk)
        variable v_f1 : std_logic;
        variable v_h, v_v : unsigned(7 downto 0);
        variable v_d, v_s : signed(16 downto 0);
        variable v_c : signed(10 downto 0);
        variable v_ph : unsigned(1 downto 0);
        variable v_a : signed(11 downto 0);
        variable v_lv : unsigned(4 downto 0);
        variable v_ln : unsigned(2 downto 0);
        variable v_hd, v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: cell / hold-block first columns (the line's first pixel
            -- starts a partial cell), live chroma latch, raw-gradient taps
            v_f1 := s_in_avid and not s_avd2;
            if s_cc = 0 or v_f1 = '1' then s_cx2 <= s_wr_x; s_cs2 <= '1'; else s_cs2 <= '0'; end if;
            if (s_cc = 0 and s_hc = 0) or v_f1 = '1' then
                s_hx2 <= s_wr_x; s_lu2 <= s_in_u(9 downto 2); s_lv2 <= s_in_v(9 downto 2);
            end if;
            if s_cc < s_wpx then s_nb2 <= '1'; else s_nb2 <= '0'; end if;
            s_fc2 <= s_fc; s_avd2 <= s_in_avid; s_fp2 <= v_f1;
            s_yr2 <= s_in_y(9 downto 2); s_yrm <= s_yr2;
            s_lp2 <= unsigned(s_rdP);

            -- 3: read addresses; |dx|, |dy| (none on a line's first pixel or
            -- a field's first line)
            s_ra3 <= s_cx2; s_ca3 <= s_hx2;
            s_cs3 <= s_cs2; s_nb3 <= s_nb2; s_fc3 <= s_fc2; s_avd3 <= s_avd2;
            s_lu3 <= s_lu2; s_lv3 <= s_lv2;
            if s_fp2 = '1' then s_gx3 <= (others => '0'); else s_gx3 <= f_absd(s_yr2, s_yrm); end if;
            if s_r_first = '1' then s_gy3 <= (others => '0'); else s_gy3 <= f_absd(s_yr2, s_lp2); end if;

            -- 4: (RAM data) raw gradient
            s_cs4 <= s_cs3; s_nb4 <= s_nb3; s_fc4 <= s_fc3; s_avd4 <= s_avd3;
            s_lu4 <= s_lu3; s_lv4 <= s_lv3;
            s_gs4 <= resize(s_gx3, 9) + resize(s_gy3, 9);

            -- 5: this row's cell and the row above; chroma from the stored
            -- row (U/V swapped on rows whose sync parity differs) or live
            if s_avd4 = '0' then
                s_cy5 <= x"10"; s_oy5 <= x"10";
            elsif s_r_bank = '1' then
                s_cy5 <= unsigned(s_rdY1); s_oy5 <= unsigned(s_rdY0);
            else
                s_cy5 <= unsigned(s_rdY0); s_oy5 <= unsigned(s_rdY1);
            end if;
            if s_avd4 = '0' then
                s_cu5 <= x"80"; s_cv5 <= x"80";
            elsif s_sw_live = '1' then
                s_cu5 <= s_lu4; s_cv5 <= s_lv4;
            elsif s_r_bank = '1' then
                if s_r_swap = '1' then s_cu5 <= unsigned(s_rdV1); s_cv5 <= unsigned(s_rdU1);
                else s_cu5 <= unsigned(s_rdU1); s_cv5 <= unsigned(s_rdV1); end if;
            else
                if s_r_swap = '1' then s_cu5 <= unsigned(s_rdV0); s_cv5 <= unsigned(s_rdU0);
                else s_cu5 <= unsigned(s_rdU0); s_cv5 <= unsigned(s_rdV0); end if;
            end if;
            s_cs5 <= s_cs4; s_nb5 <= s_nb4; s_fc5 <= s_fc4;
            -- raw edge widened to Width px (peak held Width-1 px)
            if s_avd4 = '0' then
                s_pk5 <= (others => '0'); s_pc5 <= (others => '0');
            elsif s_gs4 >= s_pk5 then
                s_pk5 <= s_gs4; s_pc5 <= s_wqx;
            elsif s_pc5 /= 0 then
                s_pc5 <= s_pc5 - 1;
            else
                s_pk5 <= s_gs4;
            end if;

            -- 6: steps across the cell's left boundary (latched at its first
            -- column; none for the line's first cell) and its top boundary
            if s_cs5 = '1' then
                s_prev <= s_cy5;
                if s_fc5 = '1' then s_hst6 <= (others => '0'); else s_hst6 <= f_absd(s_cy5, s_prev); end if;
            end if;
            s_vst6 <= f_absd(s_cy5, s_oy5);
            s_y6 <= s_cy5; s_nb6 <= s_nb5; s_raw6 <= s_pk5;
            -- chroma bleed: one-pole ease toward each block's colour, 10.6
            -- fixed point (shift 0 = hard hold)
            v_d := signed(shift_left(resize(s_cu5, 17), 8)) - signed(resize(s_ub6, 17));
            case to_integer(s_bk) is
                when 3 => v_d := shift_right(v_d, 3);
                when 4 => v_d := shift_right(v_d, 4);
                when 5 => v_d := shift_right(v_d, 5);
                when 6 => v_d := shift_right(v_d, 6);
                when others => null;
            end case;
            v_s := signed(resize(s_ub6, 17)) + v_d;
            s_ub6 <= unsigned(v_s(15 downto 0));
            v_d := signed(shift_left(resize(s_cv5, 17), 8)) - signed(resize(s_vb6, 17));
            case to_integer(s_bk) is
                when 3 => v_d := shift_right(v_d, 3);
                when 4 => v_d := shift_right(v_d, 4);
                when 5 => v_d := shift_right(v_d, 5);
                when 6 => v_d := shift_right(v_d, 6);
                when others => null;
            end case;
            v_s := signed(resize(s_vb6, 17)) + v_d;
            s_vb6 <= unsigned(v_s(15 downto 0));

            -- 7: edge value: Cells = step of a boundary within Width of this
            -- pixel (left or top, the stronger), Raw = widened gradient
            if s_nb6 = '1' then v_h := s_hst6; else v_h := (others => '0'); end if;
            if s_r_band = '1' and s_r_top = '0' then v_v := s_vst6; else v_v := (others => '0'); end if;
            if s_sw_raw = '1' then
                s_e7 <= s_raw6;
            elsif v_h >= v_v then
                s_e7 <= resize(v_h, 9);
            else
                s_e7 <= resize(v_v, 9);
            end if;
            s_y7 <= s_y6; s_u7 <= s_ub6(15 downto 6); s_v7 <= s_vb6(15 downto 6);

            -- 8-9: gain = (16 + m) * 2^e / 128; column of the edge-hold RAM
            s_ep8 <= s_e7 * s_gman;
            s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            if s_avid_sr(6) = '0' then s_x8 <= (others => '1'); else s_x8 <= s_x8 + 1; end if;
            case to_integer(s_gexp) is
                when 0 => s_es9 <= resize(s_ep8(13 downto 5), 17);
                when 1 => s_es9 <= resize(s_ep8(13 downto 4), 17);
                when 2 => s_es9 <= resize(s_ep8(13 downto 3), 17);
                when 3 => s_es9 <= resize(s_ep8(13 downto 2), 17);
                when 4 => s_es9 <= resize(s_ep8(13 downto 1), 17);
                when 5 => s_es9 <= resize(s_ep8, 17);
                when 6 => s_es9 <= shift_left(resize(s_ep8, 17), 1);
                when others => s_es9 <= shift_left(resize(s_ep8, 17), 2);
            end case;
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8; s_x9 <= s_x8;

            -- 10: edge 0..876; (RAM data) hold state of the line above
            if s_es9 > 876 then s_ev10 <= to_unsigned(876, 10); else s_ev10 <= s_es9(9 downto 0); end if;
            s_vh10 <= unsigned(s_rdD);
            s_ud10 <= signed(resize(s_u9, 11)) - 512; s_vd10 <= signed(resize(s_v9, 11)) - 512;
            s_y10 <= s_y9 & "00"; s_u10 <= s_u9; s_v10 <= s_v9; s_x10 <= s_x9;

            -- 11: Raw edges held Width lines down: the line above's level
            -- while it has lines left; restart on a stronger edge (or none
            -- held); colour weight for Only (sixteenths, full at 512)
            v_lv := s_vh10(7 downto 3); v_ln := s_vh10(2 downto 0);
            if v_ln /= 0 and s_sw_raw = '1' and s_r_first = '0' then v_hd := v_lv & "00000"; else v_hd := (others => '0'); end if;
            if s_ev10 >= v_hd then v_m := s_ev10; else v_m := v_hd; end if;
            s_ev11 <= v_m;
            if v_m(9) = '1' then s_wt11 <= to_unsigned(16, 5); else s_wt11 <= resize(v_m(8 downto 5), 5); end if;
            if v_ln = 0 or s_r_first = '1' or s_ev10(9 downto 5) >= v_lv then
                s_vs11 <= std_logic_vector(s_ev10(9 downto 5)) & std_logic_vector(s_wqy);
            else
                s_vs11 <= std_logic_vector(v_lv) & std_logic_vector(v_ln - 1);
            end if;
            s_ud11 <= s_ud10; s_vd11 <= s_vd10;
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10; s_x11 <= s_x10;

            -- 12: compose luma; Only-mode colour product
            if s_sw_only = '1' then
                if s_sw_white = '1' then s_ys12 <= signed(resize(s_ev11, 12)) + 64;
                else s_ys12 <= 940 - signed(resize(s_ev11, 12)); end if;
            else
                if s_sw_white = '1' then s_ys12 <= signed(resize(s_y11, 12)) + signed(resize(s_ev11, 12));
                else s_ys12 <= signed(resize(s_y11, 12)) - signed(resize(s_ev11, 12)); end if;
            end if;
            s_um12 <= s_ud11 * signed(resize(s_wt11, 6));
            s_vm12 <= s_vd11 * signed(resize(s_wt11, 6));
            if s_ev11 > 64 then s_hi12 <= '1'; else s_hi12 <= '0'; end if;
            s_ev12 <= s_ev11; s_u12 <= s_u11; s_v12 <= s_v11;

            -- 13: legal luma; chroma: Only = cell colour scaled by the edge on
            -- neutral paper, Mono = neutral edges
            if s_ys12 < 64 then
                s_y13 <= to_unsigned(64, 10);
            elsif s_ys12 > 940 then
                s_y13 <= to_unsigned(940, 10);
            else
                s_y13 <= unsigned(s_ys12(9 downto 0));
            end if;
            if s_sw_only = '1' then
                if s_sw_mono = '1' then
                    s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
                else
                    v_c := s_um12(14 downto 4) + 512; s_u13 <= unsigned(v_c(9 downto 0));
                    v_c := s_vm12(14 downto 4) + 512; s_v13 <= unsigned(v_c(9 downto 0));
                end if;
            elsif s_sw_mono = '1' and s_hi12 = '1' then
                s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            else
                s_u13 <= s_u12; s_v13 <= s_v12;
            end if;
            s_ca13 <= resize(s_ev12, 11) + resize(s_cbase, 11);

            -- 14: crawl amplitude (edge + base) * Crawl
            s_cp14 <= s_ca13 * s_crawl;
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            if s_avid_sr(12) = '0' then s_dx14 <= (others => '1'); else s_dx14 <= s_dx14 + 1; end if;

            -- 15: dot crawl: U+ V+ U- V- every 4 px (SD) / 8 px (HD), inverted
            -- every line, a quarter step every field
            if s_hd = '1' then v_ph := s_dx14(2 downto 1); else v_ph := s_dx14(1 downto 0); end if;
            if s_r_lph = '1' then v_ph := v_ph + 2; end if;
            v_ph := v_ph + s_fph;
            if s_avid_sr(13) = '1' then v_a := signed(resize(s_cp14(18 downto 9), 12)); else v_a := (others => '0'); end if;
            case to_integer(v_ph) is
                when 0 => s_u15 <= signed(resize(s_u14, 12)) + v_a; s_v15 <= signed(resize(s_v14, 12));
                when 1 => s_u15 <= signed(resize(s_u14, 12)); s_v15 <= signed(resize(s_v14, 12)) + v_a;
                when 2 => s_u15 <= signed(resize(s_u14, 12)) - v_a; s_v15 <= signed(resize(s_v14, 12));
                when others => s_u15 <= signed(resize(s_u14, 12)); s_v15 <= signed(resize(s_v14, 12)) - v_a;
            end case;
            s_y15 <= s_y14;

            -- 16: legal chroma (into the lag ring)
            if s_u15 < 0 then s_u16 <= (others => '0');
            elsif s_u15 > 1023 then s_u16 <= (others => '1');
            else s_u16 <= unsigned(s_u15(9 downto 0)); end if;
            if s_v15 < 0 then s_v16 <= (others => '0');
            elsif s_v15 > 1023 then s_v16 <= (others => '1');
            else s_v16 <= unsigned(s_v15(9 downto 0)); end if;
            s_y16 <= s_y15;

            -- 17-19: luma waits out the ring (read 1 + lag behind)
            s_y17 <= s_y16; s_y18 <= s_y17; s_y19 <= s_y18;
            s_lc19 <= unsigned(s_rdC);

            -- 20: merge; knee prep
            s_y20 <= s_y19; s_yk20 <= signed(resize(s_y19, 11)) - 600;
            s_u20 <= s_lc19(15 downto 8) & "00"; s_v20 <= s_lc19(7 downto 0) & "00";

            -- 21: highlight knee (940 -> 770), neutral outside avid so the
            -- smoothing's edge taps read black
            if s_avid_sr(19) = '0' then
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            else
                if s_yk20 > 0 then
                    s_y21 <= to_unsigned(600, 10) + unsigned(s_yk20(9 downto 1));
                else
                    s_y21 <= s_y20;
                end if;
                s_u21 <= s_u20; s_v21 <= s_v20;
            end if;

            -- 22-23: [1 2 1] on Y/U/V (output centre = s_ya), blanking gate
            s_ya <= s_y21; s_yb <= s_ya;
            s_ua <= s_u21; s_ub <= s_ua;
            s_va <= s_v21; s_vb <= s_va;
            if s_avid_sr(21) = '1' then
                s_y23 <= f_121(s_yb, s_ya, s_y21);
                s_u23 <= f_121(s_ub, s_ua, s_u21);
                s_v23 <= f_121(s_vb, s_va, s_v21);
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
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

end architecture cellcam;
