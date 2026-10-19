-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Crush v0.2.0: catalogue combo #42 (seed 20260919) -- sync loss / horizontal
-- collapse + mosaic with per-cell colour averaging.  Three knobs each; the
-- switches and the slider decide how the two combine.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The read-address chain (screen x,
-- sync jumps, Square hold, collapse, Squeeze hold, the line, the edges) runs
-- in virtual 1080-line units: x steps 1.0 / 1.5 / 2.0 per pixel at 1080 /
-- 720p / SD (SD = a 1440-wide 4:3 window), from the video timing ID.  The
-- virtual address is turned back into a line-buffer pixel at the end (720p:
-- floor((2V+1)/3), SD: V/2), 4 clocks, so cells keep their 1080 size
-- relative to the screen height.  The box average spans 2^(n-1) pixels at
-- 720p / SD (the whole cell at SD, 3/4 of it at 720p).  Rows count virtual
-- lines (Q8 accumulator, 2 x 1080/486 per 480i field line), so SD cells
-- are no longer twice as tall.  Grout follows: vertical bands are at least
-- one line; rows under 2 lines on screen get every R-th line instead (the
-- 1080 coverage); horizontal grout is 1/8 of the cell, at least 1 px.  Sync jumps are sized from a 1920-unit width at SD
-- (sizes follow the screen height).  1080p is bit-identical; latency 20 -> 24.
--
--   K1 Line     where the picture collapses to (0 .. 100 % of the width)
--   K2 Sync     sync loss: random per-line jumps, up to +-1/4 of the width,
--               the same size on screen at any Collapse
--   K3 Roll     the collapse line drifts sideways on its own; speed
--   K4 Cells    cell width 2 .. 256 px, eight octave steps (deadbanded)
--   K5 Aspect   cell height = width / 1, 2, 4, 8
--   K6 Grout    grid lines along the cells' real edges, 1/8 of a cell thick
--               (1 .. 8 px / lines); none on cells under 4 px
--
--   S7  Tiles   Square: screen-fixed cells over the squeezed picture /
--               Squeeze: the cells squeeze with the picture
--   S8  Collapse Both Sides squeeze toward the line / Right Side: the left of
--               the line stays put, the right side squeezes into it
--   S9  Loss    every line jumps / whole cell rows jump (bands >= 8 lines)
--   S10 Beyond  black outside the picture / the edge cells stretched
--   S11 Grout   black / white
--   P12 Collapse the picture squeezes toward the line, x1 .. x64,
--               exponential: every sixth of the travel doubles it
--
-- Cell average = an exact N-sample running box sum (EBR ring read N-1 back;
-- the line's first pixel stands in for the samples before it), so line
-- buffer address a holds the average of pixels a-N+1 .. a and a cell reads
-- its own average at its last pixel (x | mask).  Rows: the last line of each
-- cell row captures the next row's averages into one bank while the other
-- is displayed; the frame's first line is black.  Replayed rows swap U/V
-- where their hsync->avid parity differs from the capture row's (drape v1.3).
-- Read address: screen x - sync offset, cell-held (Square), collapsed about
-- the line (dm x mantissa, << octave), cell-held again (Squeeze); outside
-- the line = black or the first / last cell.  Grout = the read address
-- changed within the last t pixels, or the row's first t lines.  Output:
-- [1 2 1] on Y/U/V + luma scaled to white ~775 (the HD HDMI bar fix),
-- blanking-gated.  Latency 23 clocks in all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture crush of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (23 + 1 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT   : integer := 23;
    constant C_BLANK : std_logic_vector(31 downto 0) := "00" & "0001000000" & "1000000000" & "1000000000";

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;
    -- luma: floored at black (the luma scale below assumes y >= 64)
    function f_clampy(v : signed) return unsigned is
    begin
        if v < 64 then
            return to_unsigned(64, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;
    -- x * 2^n and a / 2^n for n = 1..8 (fixed slices, no barrel)
    function f_mul(x : unsigned(9 downto 0); n : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(17 downto 0);
    begin
        r := (others => '0');
        case to_integer(n) is
            when 1 => r(10 downto 1) := x;
            when 2 => r(11 downto 2) := x;
            when 3 => r(12 downto 3) := x;
            when 4 => r(13 downto 4) := x;
            when 5 => r(14 downto 5) := x;
            when 6 => r(15 downto 6) := x;
            when 7 => r(16 downto 7) := x;
            when others => r(17 downto 8) := x;
        end case;
        return r;
    end function;
    function f_avg(a : unsigned(17 downto 0); n : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(n) is
            when 1 => return a(10 downto 1);
            when 2 => return a(11 downto 2);
            when 3 => return a(12 downto 3);
            when 4 => return a(13 downto 4);
            when 5 => return a(14 downto 5);
            when 6 => return a(15 downto 6);
            when 7 => return a(16 downto 7);
            when others => return a(17 downto 8);
        end case;
    end function;
    -- thermometer: bit i set when i < n
    function f_therm(n : integer; w : integer) return unsigned is
        variable r : unsigned(w - 1 downto 0);
    begin
        for i in 0 to w - 1 loop
            if i < n then r(i) := '1'; else r(i) := '0'; end if;
        end loop;
        return r;
    end function;

    -- Resolution scaling (mondrian v1.0.2): per-mode steps from the timing ID.
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

    --------------------------------------------------------------------------
    -- Memories (no_rw_check: the ring never reads its write address; a line
    -- buffer's read data is discarded whenever its bank is being written)
    --------------------------------------------------------------------------
    type t_bring is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal bring : t_bring := (others => C_BLANK);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    attribute no_rw_check : boolean;
    attribute no_rw_check of bring, lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := "0001000000";
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- per-frame controls (latched in vblank)
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_k4h, s_k5h : unsigned(9 downto 0) := (others => '0');
    signal s_d4, s_d5   : signed(10 downto 0) := (others => '0');
    signal s_k4_live    : std_logic := '0';
    signal s_sq, s_os, s_bands, s_stretch, s_white : std_logic := '0';
    signal s_n      : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_a      : unsigned(1 downto 0) := (others => '0');
    signal s_mask   : unsigned(7 downto 0) := x"0F";
    signal s_nmask  : unsigned(10 downto 0) := "11111110000";
    signal s_wf     : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_wm1    : unsigned(10 downto 0) := to_unsigned(1919, 11);
    signal s_lastcs : unsigned(10 downto 0) := to_unsigned(1904, 11);
    signal s_e6     : unsigned(12 downto 0) := (others => '0');
    signal s_oct    : unsigned(2 downto 0) := (others => '0');
    signal s_frac   : unsigned(7 downto 0) := (others => '0');
    signal s_clp    : unsigned(20 downto 0) := (others => '0');
    signal s_cdrift : unsigned(14 downto 0) := (others => '0');
    signal s_clpre  : unsigned(11 downto 0) := (others => '0');
    signal s_cline  : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_rr, s_ll : signed(5 downto 0) := to_signed(4, 6);   -- log2 rows, log2 screen cell width
    signal s_rm1    : unsigned(7 downto 0) := x"0F";
    signal s_tmask  : unsigned(7 downto 0) := x"03";
    signal s_gh_en  : std_logic := '1';
    signal s_sgp    : unsigned(18 downto 0) := (others => '0');
    signal s_sg     : unsigned(10 downto 0) := (others => '0');
    signal s_grout  : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(3 downto 0) := (others => '1');
    signal s_lfsr   : unsigned(15 downto 0) := x"6E27";

    -- resolution scaling: steps decoded free-running from the timing ID
    -- (static while a mode runs); box octave, row grout band, sizing width
    signal s_stx    : unsigned(2 downto 0) := to_unsigned(2, 3);      -- Q.1 per pixel
    signal s_sty    : unsigned(11 downto 0) := to_unsigned(256, 12);  -- Q4.8 per field line
    signal s_msd, s_m720 : std_logic := '0';
    signal s_vxa    : unsigned(12 downto 0) := (others => '0');       -- virtual screen x, Q.1
    signal s_nb     : unsigned(3 downto 0) := to_unsigned(4, 4);      -- box: 2^nb pixels
    signal s_maskb  : unsigned(7 downto 0) := x"0F";
    signal s_nmaskb : unsigned(10 downto 0) := "11111110000";
    signal s_wsz    : unsigned(10 downto 0) := to_unsigned(1920, 11); -- width that sizes the jumps
    signal s_gth    : unsigned(11 downto 0) := to_unsigned(256, 12);  -- row grout band, Q4.8
    signal s_tv     : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal s_r2ok   : std_logic := '1';                               -- rows are 2+ lines on screen
    signal s_lcnt   : unsigned(2 downto 0) := (others => '0');        -- screen lines since vsync, mod 8

    -- per-line sync offset
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_bandv  : unsigned(11 downto 0) := to_unsigned(1792, 12); -- lines since a re-roll, Q4.8, sat. 7.0
    signal s_hl     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_hs     : signed(10 downto 0) := (others => '0');
    signal s_plo    : signed(17 downto 0) := (others => '0');
    signal s_phi    : signed(16 downto 0) := (others => '0');
    signal s_jp     : signed(22 downto 0) := (others => '0');
    signal s_off_row : signed(10 downto 0) := (others => '0');

    -- tracking / rows
    signal s_rx, s_line_width : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_prev_avid, s_saw_active : std_logic := '0';
    signal s_ya     : unsigned(16 downto 0) := (others => '1');       -- line position in its row, Q9.8 virtual
    signal s_rowst  : std_logic := '0';                               -- the line starts a cell row
    signal s_capchk : std_logic := '0';
    signal s_capture, s_wpar, s_rb_nx, s_rbank : std_logic := '0';
    signal s_l0_nx, s_line0 : std_logic := '0';
    signal s_hpar, s_swap : std_logic := '0';
    signal s_cpar   : std_logic_vector(1 downto 0) := "00";
    signal s_growl  : std_logic := '0';

    -- box path
    signal s_rw     : unsigned(7 downto 0) := (others => '0');
    signal s_bq     : std_logic_vector(31 downto 0) := C_BLANK;
    signal s_q2     : std_logic_vector(29 downto 0) := C_BLANK(29 downto 0);
    signal s_xdy, s_xdu, s_xdv : unsigned(9 downto 0) := (others => '0');
    signal s_x0y, s_x0u, s_x0v : unsigned(9 downto 0) := (others => '0');
    signal s_cond, s_first : std_logic := '0';
    signal s_dy, s_du, s_dv : signed(10 downto 0) := (others => '0');
    signal s_xny, s_xnu, s_xnv : unsigned(17 downto 0) := (others => '0');
    signal s_accy, s_accu, s_accv : unsigned(17 downto 0) := (others => '0');
    signal s_avy, s_avu, s_avv : unsigned(9 downto 0) := (others => '0');
    type t_wx is array (1 to 4) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 4) := (others => '0');

    -- address chain (k = clocks after the input pixel)
    signal s_ax0    : unsigned(10 downto 0) := (others => '0');
    signal s_xs1    : signed(12 downto 0) := (others => '0');
    signal s_xh2    : signed(12 downto 0) := (others => '0');
    signal s_xge2   : std_logic := '0';
    signal s_xc3, s_xc4, s_xc5, s_xc6, s_xc7 : signed(12 downto 0) := (others => '0');
    signal s_dm4    : signed(13 downto 0) := (others => '0');
    signal s_pl5, s_ph5 : signed(18 downto 0) := (others => '0');
    signal s_d85    : signed(22 downto 0) := (others => '0');
    signal s_os5, s_os6, s_os7 : std_logic := '0';
    signal s_pr6    : signed(22 downto 0) := (others => '0');
    signal s_of7    : signed(19 downto 0) := (others => '0');
    signal s_am8    : signed(19 downto 0) := (others => '0');
    signal s_lt9, s_ge9, s_pc9 : std_logic := '0';
    signal s_amh9   : unsigned(10 downto 0) := (others => '0');
    signal s_addr10, s_addrp : unsigned(10 downto 0) := (others => '0');
    signal s_bg10, s_bg11, s_bg12 : std_logic := '0';
    signal s_chg11  : std_logic := '0';
    -- virtual address -> line-buffer pixel (k11..k14): 720p floor((2V+1)/3)
    -- = ((V+1) x 5 x 17 x 257) >> 15, SD V/2, 1080 V
    signal s_cv11, s_cv12, s_cv13 : unsigned(10 downto 0) := (others => '0');
    signal s_cp11   : unsigned(11 downto 0) := (others => '0');
    signal s_ct12   : unsigned(13 downto 0) := (others => '0');
    signal s_ct13   : unsigned(17 downto 0) := (others => '0');
    signal s_ra14   : unsigned(10 downto 0) := (others => '0');
    signal s_chg12, s_chg13, s_chg14, s_chg15 : std_logic := '0';
    signal s_bgd12, s_bgd13, s_bgd14, s_bg15 : std_logic := '0';
    signal s_gh     : unsigned(6 downto 0) := (others => '0');
    signal s_gvt12  : std_logic := '0';

    -- pixel path
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_ey13, s_eu13, s_ev13 : signed(10 downto 0) := (others => '0');
    signal s_gc13   : unsigned(7 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_py14, s_pu14, s_pv14 : signed(19 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_g1y, s_g1u, s_g1v : unsigned(9 downto 0) := (others => '0');
    signal s_g2y, s_g2u, s_g2v : unsigned(9 downto 0) := (others => '0');
    signal s_ys, s_us, s_vs : unsigned(9 downto 0) := (others => '0');
    signal s_yo, s_uo, s_vo : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Box-sum ring: 256 deep, read N-1 back
    --------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rw <= s_rw + 1;
            s_bq <= bring(to_integer(s_rw - s_maskb));
            bring(to_integer(s_rw)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Tracking, rows, per-line sync offset, per-frame decode, box sums
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_ys : unsigned(16 downto 0);
        variable v_n  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            -- resolution scaling: per-mode steps, free-running
            s_stx <= f_vstep_x(registers_in(8)(3 downto 0));
            s_sty <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_msd <= '1'; s_m720 <= '0';
                when 5 | 6 | 14     => s_msd <= '0'; s_m720 <= '1';
                when others         => s_msd <= '0'; s_m720 <= '0';
            end case;
            if data_in.avid = '1' then
                s_rx  <= s_rx + 1;
                s_vxa <= s_vxa + s_stx;
                s_saw_active <= '1';
            else
                s_rx  <= (others => '0');
                s_vxa <= (others => '0');
            end if;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            -- per-line sync offset, one small op per step: (rand - 512) x the
            -- per-frame gain, split hi/lo; Rows mode re-rolls on a row's first
            -- line once 8+ (virtual) lines have passed
            case to_integer(s_lstep) is
                when 0 =>
                    if s_bands = '0' or (s_rowst = '1' and s_bandv >= 1792) then
                        s_hl <= s_lfsr(9 downto 0);
                        s_bandv <= (others => '0');
                    elsif s_bandv < 1792 then
                        if s_bandv + s_sty >= 1792 then
                            s_bandv <= to_unsigned(1792, 12);
                        else
                            s_bandv <= s_bandv + s_sty;
                        end if;
                    end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_hs <= signed('0' & s_hl) - to_signed(512, 11);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_plo <= s_hs * signed('0' & s_sg(5 downto 0));
                    s_phi <= s_hs * signed('0' & s_sg(10 downto 6));
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_jp <= shift_left(resize(s_phi, 23), 6) + resize(s_plo, 23);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_off_row <= resize(shift_right(s_jp, 11), 11);
                    s_lstep <= "111";
                when others => null;
            end case;

            -- hsync->avid parity (drape): clock parity since hsync
            s_hpar <= not s_hpar;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar  <= '0';
                s_lstep <= (others => '0');
            end if;

            -- end of a line: the next line's position in its row (virtual
            -- lines, Q8; rows are 2^rr units, so the wrap is a mask) and,
            -- one clock later, whether it ends its row (then it captures)
            s_capchk <= '0';
            if s_capchk = '1' then
                v_ys := s_ya + s_sty;
                if (v_ys(16 downto 8) and not resize(s_rm1, 9)) /= 0 then
                    s_capture <= '1';
                    s_wpar    <= not s_wpar;
                else
                    s_capture <= '0';
                end if;
            end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_vxa(11 downto 1);   -- virtual width
                v_ys := s_ya + s_sty;
                if s_line0 = '1' then
                    s_ya <= (others => '0');
                    s_rowst <= '1';
                else
                    s_ya <= (v_ys(16 downto 8) and resize(s_rm1, 9)) & v_ys(7 downto 0);
                    if (v_ys(16 downto 8) and not resize(s_rm1, 9)) /= 0 then
                        s_rowst <= '1';
                    else
                        s_rowst <= '0';
                    end if;
                end if;
                s_rb_nx  <= s_wpar;           -- the last completed capture
                s_capchk <= '1';
                s_l0_nx  <= '0';
                s_lcnt   <= s_lcnt + 1;
            end if;
            -- start of a line: the read side switches here (in-flight pixels
            -- of the previous line are long gone)
            if s_prev_avid = '0' and data_in.avid = '1' then
                s_rbank <= s_rb_nx;
                if s_rb_nx = '1' then
                    s_swap <= s_hpar xor s_cpar(1);
                else
                    s_swap <= s_hpar xor s_cpar(0);
                end if;
                if s_capture = '1' then
                    if s_wpar = '1' then s_cpar(1) <= s_hpar; else s_cpar(0) <= s_hpar; end if;
                end if;
                s_line0 <= s_l0_nx;
                -- rows under 2 screen lines can't show a band: their grout
                -- becomes every R-th screen line (the same coverage)
                if s_gh_en = '1' and ((s_r2ok = '1' and s_ya < s_gth) or
                                      (s_r2ok = '0' and (s_lcnt and s_rm1(2 downto 0)) = 0)) then
                    s_growl <= '1';
                else
                    s_growl <= '0';
                end if;
            end if;

            -- per-frame decode, one small op per state, in vblank
            case to_integer(s_vstep) is
                when 0 =>
                    s_k1  <= unsigned(registers_in(0));
                    s_k2  <= unsigned(registers_in(1));
                    s_k3  <= unsigned(registers_in(2));
                    s_k4  <= unsigned(registers_in(3));
                    s_k5  <= unsigned(registers_in(4));
                    s_k6  <= unsigned(registers_in(5));
                    s_p12 <= unsigned(registers_in(7));
                    s_sq      <= registers_in(6)(0);
                    s_os      <= registers_in(6)(1);
                    s_bands   <= registers_in(6)(2);
                    s_stretch <= registers_in(6)(3);
                    s_white   <= registers_in(6)(4);
                    s_wf <= s_line_width;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_d4 <= signed('0' & s_k4) - signed('0' & s_k4h);
                    s_d5 <= signed('0' & s_k5) - signed('0' & s_k5h);
                    s_cdrift <= s_cdrift + resize(s_k3(9 downto 4), 15);
                    s_clp <= s_k1 * s_wf;
                    s_wm1 <= s_wf - 1;
                    -- jumps are a size: sized from 1920 units on the 4:3 SD window
                    if s_msd = '1' then s_wsz <= to_unsigned(1920, 11); else s_wsz <= s_wf; end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- deadband the stepped knobs (no flicker at a step edge)
                    if s_d4 > 12 or s_d4 < -12 then s_k4h <= s_k4; end if;
                    if s_d5 > 12 or s_d5 < -12 then s_k5h <= s_k5; end if;
                    if s_k4 >= 8 then s_k4_live <= '1'; end if;
                    -- P12 x 6 = octave . fraction of the squeeze
                    s_e6 <= shift_left(resize(s_p12, 13), 2) + shift_left(resize(s_p12, 13), 1);
                    if s_cdrift(14 downto 4) >= s_wf then
                        s_cdrift <= s_cdrift - shift_left(resize(s_wf, 15), 4);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    -- the firmware loads a labelled default as the label INDEX;
                    -- honour it only until the knob first reads a real position
                    if s_k4_live = '0' and s_k4 < 8 then
                        v_n := resize(s_k4(2 downto 0), 4) + 1;
                    else
                        v_n := resize(s_k4h(9 downto 7), 4) + 1;
                    end if;
                    s_n <= v_n;
                    s_a <= s_k5h(9 downto 8);
                    s_oct  <= s_e6(12 downto 10);
                    s_frac <= s_e6(9 downto 2);
                    s_clpre <= resize(s_clp(20 downto 10), 12) + resize(s_cdrift(14 downto 4), 12);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_mask <= f_therm(to_integer(s_n), 8);
                    s_rr   <= signed(resize(s_n, 6)) - signed(resize(s_a, 6));
                    -- box average: the cell's pixels at SD (N/2), 3/4 of them at
                    -- 720p (N/2 of 2N/3); at least 2
                    if (s_msd = '1' or s_m720 = '1') and s_n > 1 then
                        s_nb <= s_n - 1;
                    else
                        s_nb <= s_n;
                    end if;
                    -- screen cell width: N, or N / squeeze with Squeeze tiles
                    if s_sq = '1' then
                        s_ll <= signed(resize(s_n, 6)) - signed(resize(s_oct, 6)) - signed(resize(s_frac(7 downto 7), 6));
                    else
                        s_ll <= signed(resize(s_n, 6));
                    end if;
                    if s_clpre >= resize(s_wf, 12) then
                        s_cline <= resize(s_clpre - resize(s_wf, 12), 11);
                    else
                        s_cline <= resize(s_clpre, 11);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_nmask  <= not resize(s_mask, 11);
                    s_lastcs <= s_wm1 and not resize(s_mask, 11);
                    if s_rr <= 0 then
                        s_rm1 <= (others => '0');
                    else
                        s_rm1 <= f_therm(to_integer(s_rr), 8);
                    end if;
                    s_maskb  <= f_therm(to_integer(s_nb), 8);
                    -- grout 1/8 of a cell (1..8 virtual units, at least 1 px on
                    -- screen: 720p 1/1/1/3/5 px, SD 1/1/1/2/4 px), none on cells
                    -- under 4 units
                    if s_ll < 2 then s_tmask <= x"00";
                    elsif s_ll < 4 then s_tmask <= x"01";
                    elsif s_ll = 4 then
                        if s_msd = '1' or s_m720 = '1' then s_tmask <= x"01"; else s_tmask <= x"03"; end if;
                    elsif s_ll = 5 then
                        if s_msd = '1' then s_tmask <= x"03"; elsif s_m720 = '1' then s_tmask <= x"07"; else s_tmask <= x"0F"; end if;
                    else
                        if s_msd = '1' then s_tmask <= x"0F"; elsif s_m720 = '1' then s_tmask <= x"1F"; else s_tmask <= x"FF"; end if;
                    end if;
                    -- row grout: the row's first 1/1/2/4/8 virtual lines
                    if s_rr < 2 then s_gh_en <= '0'; s_tv <= to_unsigned(1, 4);
                    elsif s_rr < 4 then s_gh_en <= '1'; s_tv <= to_unsigned(1, 4);
                    elsif s_rr = 4 then s_gh_en <= '1'; s_tv <= to_unsigned(2, 4);
                    elsif s_rr = 5 then s_gh_en <= '1'; s_tv <= to_unsigned(4, 4);
                    else s_gh_en <= '1'; s_tv <= to_unsigned(8, 4); end if;
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    -- sync gain: K2 x width (full knob ~ +-1/4 of the width)
                    s_sgp <= s_k2(9 downto 2) * s_wsz;
                    s_grout <= s_k6(9 downto 2);
                    s_nmaskb <= not resize(s_maskb, 11);
                    -- rows of under 2 screen lines get line grout instead of a band
                    if shift_left(resize(s_rm1, 17) + 1, 8) >= shift_left(resize(s_sty, 17), 1) then
                        s_r2ok <= '1';
                    else
                        s_r2ok <= '0';
                    end if;
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_sg <= s_sgp(18 downto 8);
                    -- the band is at least one line (half a field step each side)
                    if shift_left(resize(s_tv, 12), 8) >= s_sty then
                        s_gth <= shift_left(resize(s_tv, 12), 8);
                    else
                        s_gth <= s_sty;
                    end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- frame start (idempotent: vsync may serrate): the first line
            -- captures into bank 0 and is shown black
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_ya      <= (others => '1');
                s_lcnt    <= (others => '0');
                s_rowst   <= '0';
                s_capchk  <= '0';
                s_capture <= '1';
                s_wpar    <= '0';
                s_rb_nx   <= '1';
                s_l0_nx   <= '1';
                s_bandv   <= to_unsigned(1792, 12);
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            -- box path k0: raw sample, ring sample N back, first-pixel latch
            s_xdy <= unsigned(data_in.y); s_xdu <= unsigned(data_in.u); s_xdv <= unsigned(data_in.v);
            s_q2  <= s_bq(29 downto 0);
            if (s_rx and s_nmaskb) /= 0 then s_cond <= '1'; else s_cond <= '0'; end if;
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_x0y <= unsigned(data_in.y); s_x0u <= unsigned(data_in.u); s_x0v <= unsigned(data_in.v);
            end if;
            -- k1: difference (the line's first pixel stands in before it), N x
            if s_cond = '1' then
                s_dy <= signed('0' & s_xdy) - signed('0' & unsigned(s_q2(29 downto 20)));
                s_du <= signed('0' & s_xdu) - signed('0' & unsigned(s_q2(19 downto 10)));
                s_dv <= signed('0' & s_xdv) - signed('0' & unsigned(s_q2(9 downto 0)));
            else
                s_dy <= signed('0' & s_xdy) - signed('0' & s_x0y);
                s_du <= signed('0' & s_xdu) - signed('0' & s_x0u);
                s_dv <= signed('0' & s_xdv) - signed('0' & s_x0v);
            end if;
            s_xny <= f_mul(s_xdy, s_nb); s_xnu <= f_mul(s_xdu, s_nb); s_xnv <= f_mul(s_xdv, s_nb);
            s_first <= s_avid_sr(0) and not s_avid_sr(1);
            -- k2: running sums = exact N-sample window ending at this pixel
            if s_first = '1' then
                s_accy <= s_xny; s_accu <= s_xnu; s_accv <= s_xnv;
            else
                s_accy <= unsigned(signed(s_accy) + s_dy);
                s_accu <= unsigned(signed(s_accu) + s_du);
                s_accv <= unsigned(signed(s_accv) + s_dv);
            end if;
            -- k3: average
            s_avy <= f_avg(s_accy, s_nb); s_avu <= f_avg(s_accu, s_nb); s_avv <= f_avg(s_accv, s_nb);
            -- write port: pixel index / bank / enable, 4 clocks behind
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid and s_capture;
            for i in 2 to 4 loop
                s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1);
            end loop;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Read address (k0..k10): sync shift, Square hold, collapse, Squeeze hold
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_c : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            -- k0: screen x (virtual units)
            s_ax0 <= s_vxa(11 downto 1);
            -- k1: sync loss shifts the screen
            s_xs1 <= signed(resize(s_ax0, 13)) - resize(s_off_row, 13);
            -- k2: Square tiles hold at the screen cell's last pixel
            if s_sq = '0' then
                s_xh2 <= s_xs1 or signed(resize(s_mask, 13));
            else
                s_xh2 <= s_xs1;
            end if;
            if s_xs1 >= signed(resize(s_wf, 13)) then s_xge2 <= '1'; else s_xge2 <= '0'; end if;
            -- k3: a part cell at the right edge reads the line's last pixel
            if s_xh2 >= signed(resize(s_wf, 13)) and s_xge2 = '0' then
                s_xc3 <= signed(resize(s_wm1, 13));
            else
                s_xc3 <= s_xh2;
            end if;
            -- k4: about the collapse line
            s_dm4 <= resize(s_xc3, 14) - signed(resize(s_cline, 14));
            s_xc4 <= s_xc3;
            -- k5: dm x mantissa (1 + frac/256), split in nibbles
            s_pl5 <= s_dm4 * signed('0' & s_frac(3 downto 0));
            s_ph5 <= s_dm4 * signed('0' & s_frac(7 downto 4));
            s_d85 <= shift_left(resize(s_dm4, 23), 8);
            s_os5 <= s_os and s_dm4(13);
            s_xc5 <= s_xc4;
            -- k6
            s_pr6 <= s_d85 + shift_left(resize(s_ph5, 23), 4) + resize(s_pl5, 23);
            s_os6 <= s_os5; s_xc6 <= s_xc5;
            -- k7: x 2^octave, back to pixels
            case to_integer(s_oct) is
                when 0 => s_of7 <= resize(shift_right(s_pr6, 8), 20);
                when 1 => s_of7 <= resize(shift_right(s_pr6, 7), 20);
                when 2 => s_of7 <= resize(shift_right(s_pr6, 6), 20);
                when 3 => s_of7 <= resize(shift_right(s_pr6, 5), 20);
                when 4 => s_of7 <= resize(shift_right(s_pr6, 4), 20);
                when others => s_of7 <= resize(shift_right(s_pr6, 3), 20);
            end case;
            s_os7 <= s_os6; s_xc7 <= s_xc6;
            -- k8: collapsed x (Right Side: the left of the line stays put)
            if s_os7 = '1' then
                s_am8 <= resize(s_xc7, 20);
            else
                s_am8 <= s_of7 + signed(resize(s_cline, 20));
            end if;
            -- k9: range; Squeeze tiles hold after the collapse
            s_lt9 <= s_am8(19);
            if s_am8 >= signed(resize(s_wf, 20)) then s_ge9 <= '1'; else s_ge9 <= '0'; end if;
            if s_sq = '1' then
                s_amh9 <= unsigned(s_am8(10 downto 0)) or resize(s_mask, 11);
                if (unsigned(s_am8(10 downto 0)) and s_nmask) = s_lastcs then s_pc9 <= '1'; else s_pc9 <= '0'; end if;
            else
                s_amh9 <= unsigned(s_am8(10 downto 0));
                s_pc9 <= '0';
            end if;
            -- k10: line-buffer address; outside the picture: black, or the
            -- first / last cell stretched
            if s_lt9 = '1' then
                if s_stretch = '1' then s_addr10 <= resize(s_mask, 11); else s_addr10 <= (others => '1'); end if;
                s_bg10 <= not s_stretch;
            elsif s_ge9 = '1' then
                if s_stretch = '1' then s_addr10 <= s_wm1; else s_addr10 <= (others => '1'); end if;
                s_bg10 <= not s_stretch;
            elsif s_pc9 = '1' then
                s_addr10 <= s_wm1;
                s_bg10 <= '0';
            else
                s_addr10 <= s_amh9;
                s_bg10 <= '0';
            end if;
            -- k11: grout edge = the read address changed
            s_addrp <= s_addr10;
            if s_addr10 /= s_addrp then s_chg11 <= '1'; else s_chg11 <= '0'; end if;
            s_bg11 <= s_bg10;
            -- k11..k14: virtual address -> line-buffer pixel (the last pixel
            -- whose span starts inside virtual unit V): 1080 V, SD V/2, 720p
            -- floor((2V+1)/3) = ((V+1) x 21845) >> 15, as x5, x17, x257
            s_cv11 <= s_addr10;
            s_cp11 <= resize(s_addr10, 12) + 1;
            s_cv12 <= s_cv11;
            s_ct12 <= resize(s_cp11, 14) + shift_left(resize(s_cp11, 14), 2);
            s_cv13 <= s_cv12;
            s_ct13 <= resize(s_ct12, 18) + shift_left(resize(s_ct12, 18), 4);
            if s_m720 = '1' then
                v_c := resize(s_ct13, 19) + resize(s_ct13(17 downto 8), 19);
                s_ra14 <= v_c(17 downto 7);
            elsif s_msd = '1' then
                s_ra14 <= '0' & s_cv13(10 downto 1);
            else
                s_ra14 <= s_cv13;
            end if;
            s_chg12 <= s_chg11; s_chg13 <= s_chg12; s_chg14 <= s_chg13; s_chg15 <= s_chg14;
            s_bgd12 <= s_bg11;  s_bgd13 <= s_bgd12; s_bgd14 <= s_bgd13; s_bg15  <= s_bgd14;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (averages written k4, read at k14 -> data k15)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ra14));
            if s_wc(4) = '1' and s_wp(4) = '0' then lbY0(to_integer(s_wx(4))) <= std_logic_vector(s_avy); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ra14));
            if s_wc(4) = '1' and s_wp(4) = '1' then lbY1(to_integer(s_wx(4))) <= std_logic_vector(s_avy); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_ra14(10 downto 1)));
            if s_wc(4) = '1' and s_wp(4) = '0' then lbU0(to_integer(s_wx(4)(10 downto 1))) <= std_logic_vector(s_avu(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_ra14(10 downto 1)));
            if s_wc(4) = '1' and s_wp(4) = '1' then lbU1(to_integer(s_wx(4)(10 downto 1))) <= std_logic_vector(s_avu(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_ra14(10 downto 1)));
            if s_wc(4) = '1' and s_wp(4) = '0' then lbV0(to_integer(s_wx(4)(10 downto 1))) <= std_logic_vector(s_avv(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_ra14(10 downto 1)));
            if s_wc(4) = '1' and s_wp(4) = '1' then lbV1(to_integer(s_wx(4)(10 downto 1))) <= std_logic_vector(s_avv(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (k16..k22; the names keep their pre-v0.2.2 stage numbers,
    -- 4 lower): bank, grout, output conditioning, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_h  : unsigned(7 downto 0);
        variable v_c1, v_c2 : std_logic_vector(7 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- k12: bank; U/V swapped on rows whose hsync->avid parity differs
            -- from the capture row's; black outside / on the frame's line 0
            if s_rbank = '1' then
                v_c1 := s_rdU1; v_c2 := s_rdV1;
            else
                v_c1 := s_rdU0; v_c2 := s_rdV0;
            end if;
            if s_bg15 = '1' or s_line0 = '1' then
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            else
                if s_rbank = '1' then s_y12 <= unsigned(s_rdY1); else s_y12 <= unsigned(s_rdY0); end if;
                if s_swap = '1' then
                    s_u12 <= unsigned(v_c2) & "00"; s_v12 <= unsigned(v_c1) & "00";
                else
                    s_u12 <= unsigned(v_c1) & "00"; s_v12 <= unsigned(v_c2) & "00";
                end if;
            end if;
            v_h := s_gh & s_chg15;
            s_gh <= v_h(6 downto 0);
            if (v_h and s_tmask) /= 0 then s_gvt12 <= '1'; else s_gvt12 <= '0'; end if;
            s_bg12 <= s_bg15 or s_line0;
            -- k13: grout differences
            if s_white = '1' then
                s_ey13 <= to_signed(940, 11) - signed('0' & s_y12);
            else
                s_ey13 <= to_signed(64, 11) - signed('0' & s_y12);
            end if;
            s_eu13 <= to_signed(512, 11) - signed('0' & s_u12);
            s_ev13 <= to_signed(512, 11) - signed('0' & s_v12);
            if (s_gvt12 = '1' or s_growl = '1') and s_bg12 = '0' then
                s_gc13 <= s_grout;
            else
                s_gc13 <= (others => '0');
            end if;
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            -- k14: products
            s_py14 <= s_ey13 * signed('0' & s_gc13);
            s_pu14 <= s_eu13 * signed('0' & s_gc13);
            s_pv14 <= s_ev13 * signed('0' & s_gc13);
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            -- k15: apply; neutral outside the avid (the smoothing's edges)
            if s_avid_sr(18) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= f_clampy(signed(resize(s_y14, 13)) + resize(shift_right(s_py14, 8), 13));
                s_u15 <= f_clamp10(signed(resize(s_u14, 13)) + resize(shift_right(s_pu14, 8), 13));
                s_v15 <= f_clamp10(signed(resize(s_v14, 13)) + resize(shift_right(s_pv14, 8), 13));
            end if;
            -- k16..17: horizontal [1 2 1]/4 (g1 the centre)
            s_g1y <= s_y15; s_g2y <= s_g1y;
            s_g1u <= s_u15; s_g2u <= s_g1u;
            s_g1v <= s_v15; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y15, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u15, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v15, 12);
            s_ys <= v_sy(11 downto 2);
            s_us <= v_su(11 downto 2);
            s_vs <= v_sv(11 downto 2);
            -- k18: luma toward black, 64 + 13/16 (y - 64) (white ~775); gate
            v_d  := s_ys - 64;
            v_yo := s_ys - shift_right(v_d, 3) - shift_right(v_d, 4);
            if s_avid_sr(21) = '1' then
                s_yo <= v_yo; s_uo <= s_us; s_vo <= s_vs;
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
    -- latency changes (23 -> 24).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture crush;
