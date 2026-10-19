-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- slitscan.vhd  (v3.0.0 -- "Slit Scan": clean blocks of lines, offset in
-- simulated time)
--
-- With a frame buffer, slit-scan gives every band of lines its own moment
-- in time, so anything moving comes out as a staircase of offset blocks.
-- Videomancer has no frame buffer, so the time is simulated: a virtual
-- camera position X(t) (sway or drift) is kept as a 256-frame history of
-- one number per frame, and band k shows the LIVE picture shifted by how
-- far that camera moved since k's moment, X(now) - X(now - d_k), plus a
-- pattern staircase (ramp, fold, random, wave, zigzag) scaled by P12.  The
-- bands ripple like a real slit-scan of a moving shot, and the picture
-- stays at full resolution with sharp band edges.
--
-- Horizontal: bands of lines, each slid sideways.  Vertical: strips of
-- columns, each showing the picture slid sideways (a vertical offset would
-- need many stored lines).
--
-- Line buffer = sidewinder's: dual-bank ping-pong by line parity, each
-- active line writes its own bank and reads the line above, Y 2048 deep,
-- U/V 2:1 decimated (22 EBR); run-along read pointers (Y, U, V) step +-1
-- per pixel, wrapping or bouncing at the inset bounds [12, W-13] so the
-- source's black blanking columns never show.
--
-- Per frame, in vertical blanking, a step sequencer works out every band's
-- offset (pattern x amount + motion lag, chroma split), folds each
-- channel's start column into [L, R] and writes {dir, pointer} into three
-- 256-entry tables (one per channel).  Horizontal: each line loads its
-- band's entry in horizontal blanking.  Vertical: the line loads strip 0,
-- and at each strip boundary (a pixel DDA, N per pixel against W) the
-- pointers load the next strip's entry, prefetched during the strip.  So
-- the pixel path is pointer steps and loads only, no arithmetic.
--
-- Controls:
--   K1 Bands     2 .. 128 bands / strips
--   K2 Pattern   Ramp, Fold, Random, Wave, Zigzag (the staircase shape)
--   K3 Motion    size of the virtual camera move (small floor)
--   K4 Span      how far back in time the last band reaches (~0.1 .. 4 s)
--   K5 Speed     how fast the camera sways / drifts
--   K6 Split     chroma dispersion: U slides further than Y, V less
--   S7 Axis      Horizontal bands / Vertical strips
--   S8 Edges     Wrap / Mirror
--   S9 Gaps      thin black lines between the blocks
--   S10 Motion   Sway (sine) / Drift (eased random targets)
--   S11 Flip     reverse the band order (staircase and lag)
--   P12 Offset   staircase amount, W/64 .. W
--
-- Pipeline (data_in -> data_out, 3 clocks, every mode):
--   E0  input registers; bank reads at the pointers; pointers step / load
--   E1  bank select
--   E2  blanking gate and gaps -> data_out
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture slitscan of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (3 + 1 = 4 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 4;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_AW            : integer := 11;                  -- Y depth 2048
    constant C_LAT           : integer := 3;
    constant C_EDGE          : integer := 12;                  -- inset bound L
    constant C_WIDTH_LINE    : integer := 8;

    constant C_Y_BLACK       : unsigned(C_DW - 1 downto 0) := to_unsigned(64, C_DW);
    constant C_C_ZERO        : unsigned(C_DW - 1 downto 0) := to_unsigned(512, C_DW);

    -- Sine ROM: 256 x signed, round(-255*cos(2*pi*i/256)) (sidewinder's).
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

    -- K1 label index -> band count
    function f_nbands(i : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 0      => return to_unsigned(2, 8);
            when 1      => return to_unsigned(3, 8);
            when 2      => return to_unsigned(4, 8);
            when 3      => return to_unsigned(6, 8);
            when 4      => return to_unsigned(8, 8);
            when 5      => return to_unsigned(12, 8);
            when 6      => return to_unsigned(16, 8);
            when 7      => return to_unsigned(24, 8);
            when 8      => return to_unsigned(32, 8);
            when 9      => return to_unsigned(48, 8);
            when 10     => return to_unsigned(64, 8);
            when 11     => return to_unsigned(96, 8);
            when others => return to_unsigned(128, 8);
        end case;
    end function f_nbands;

    -- One pixel of a run-along read pointer (sidewinder). Up: at R wrap to
    -- L (Wrap) or hold R and turn (Mirror). Down (Mirror only): at L hold
    -- and turn. Returns dir & ptr.
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

    --------------------------------------------------------------------------
    -- Dual-bank line buffers (initialised to black / neutral chroma)
    --------------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(C_DW - 1 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(C_DW - 1 downto 0);

    signal lbY0, lbY1 : t_lb_y  := (others => std_logic_vector(C_Y_BLACK));
    signal lbU0, lbU1 : t_lb_uv := (others => std_logic_vector(C_C_ZERO));
    signal lbV0, lbV1 : t_lb_uv := (others => std_logic_vector(C_C_ZERO));

    -- A bank is only displayed on the lines it is not written, the tables
    -- are written in vertical blanking and read in active lines, the
    -- history read never meets its write: no RAM reads the address it
    -- writes in the same clock.
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;

    signal s_rdY0, s_rdY1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    -- per-band pointer tables {dir, ptr} and the camera history
    type t_tab  is array (0 to 255) of std_logic_vector(15 downto 0);
    signal tabY, tabU, tabV : t_tab := (others => (others => '0'));
    signal hist : t_tab := (others => (others => '0'));
    attribute no_rw_check of tabY, tabU, tabV, hist : signal is true;
    signal s_ta    : unsigned(7 downto 0) := (others => '0');      -- table read address
    signal s_tqY, s_tqU, s_tqV : std_logic_vector(15 downto 0) := (others => '0');
    signal s_tfY, s_tfU, s_tfV : std_logic_vector(11 downto 0) := (others => '0');  -- fabric copies
    signal s_tw    : std_logic := '0';                             -- table write strobe
    signal s_twch  : unsigned(1 downto 0) := (others => '0');      -- 0 Y, 1 U, 2 V
    signal s_twa   : unsigned(7 downto 0) := (others => '0');
    signal s_twd   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ha    : unsigned(7 downto 0) := (others => '0');      -- history read address
    signal s_hq    : std_logic_vector(15 downto 0) := (others => '0');
    signal s_hw    : std_logic := '0';
    signal s_hwa   : unsigned(7 downto 0) := (others => '0');
    signal s_hwd   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rom_addr : unsigned(7 downto 0) := (others => '0');
    signal s_rom_q : signed(9 downto 0) := (others => '0');

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
    signal s_height_m     : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_width_m      : unsigned(C_AW - 1 downto 0) := to_unsigned(1920, C_AW);
    signal s_lfsr         : unsigned(15 downto 0) := x"ACE1";

    --------------------------------------------------------------------------
    -- Controls (free-running decode) and per-frame constants
    --------------------------------------------------------------------------
    signal k_bidx  : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal k_pat   : unsigned(2 downto 0) := (others => '0');
    signal k_amt, k_mot, k_span, k_spd, k_split : unsigned(9 downto 0) := (others => '0');
    signal k_axis, k_mirror, k_gap, k_drift, k_flip : std_logic := '0';

    signal s_W     : unsigned(C_AW - 1 downto 0) := to_unsigned(1920, C_AW);
    signal s_H     : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_R     : unsigned(C_AW - 1 downto 0) := to_unsigned(1920 - 1 - C_EDGE, C_AW);
    signal s_S     : unsigned(C_AW - 1 downto 0) := to_unsigned(1920 - 2 * C_EDGE, C_AW);
    signal s_mod   : unsigned(C_AW downto 0) := to_unsigned(1920 - 2 * C_EDGE, C_AW + 1);
    signal s_C1    : unsigned(C_AW downto 0) := to_unsigned(2 * (1920 - 2 * C_EDGE) + C_EDGE - 1, C_AW + 1);
    signal c_N     : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal c_pat   : unsigned(2 downto 0) := (others => '0');
    signal c_axis, c_mirror, c_gap, c_drift, c_flip : std_logic := '0';
    signal c_G     : unsigned(3 downto 0) := to_unsigned(2, 4);    -- gap size
    signal c_A     : unsigned(10 downto 0) := to_unsigned(400, 11); -- amount, px
    signal c_M     : unsigned(10 downto 0) := to_unsigned(80, 11);  -- motion, px
    signal c_split : unsigned(9 downto 0) := (others => '0');       -- Q10
    signal c_rate  : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal c_span  : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal c_rq    : unsigned(16 downto 0) := to_unsigned(8192, 17); -- 2^16 / N
    signal c_lag   : unsigned(17 downto 0) := (others => '0');       -- frames/band, Q10
    signal c_q     : unsigned(C_AW - 1 downto 0) := (others => '0'); -- W div N
    signal c_q1    : unsigned(C_AW - 1 downto 0) := (others => '0'); -- W div N + 1
    signal c_r     : unsigned(7 downto 0) := (others => '0');        -- W mod N
    signal c_gfl   : unsigned(9 downto 0) := (others => '0');        -- (N-1) * golden
    signal c_nw    : signed(12 downto 0) := to_signed(8 - 1920, 13);  -- N - W
    signal c_nh    : signed(12 downto 0) := to_signed(8 - 1080, 13);  -- N - H

    --------------------------------------------------------------------------
    -- Per-frame sequencer: one shared split multiplier (product readable
    -- three steps after the operands are set), one serial divider.
    --------------------------------------------------------------------------
    signal s_fs    : unsigned(6 downto 0) := (others => '1');
    signal s_ma, s_mb : unsigned(10 downto 0) := (others => '0');
    signal s_mpl   : unsigned(16 downto 0) := (others => '0');
    signal s_mph   : unsigned(15 downto 0) := (others => '0');
    signal s_mp    : unsigned(21 downto 0) := (others => '0');
    signal s_dnum  : unsigned(16 downto 0) := (others => '0');
    signal s_dden  : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal s_drem  : unsigned(7 downto 0) := (others => '0');
    signal s_dq    : unsigned(16 downto 0) := (others => '0');
    signal s_dcnt  : unsigned(4 downto 0) := (others => '0');
    signal s_aq16, s_mq16 : unsigned(10 downto 0) := (others => '0');
    signal s_mreq  : std_logic := '0';
    signal s_msel  : unsigned(3 downto 0) := (others => '0');
    signal s_absr, s_lfb : unsigned(10 downto 0) := (others => '0');

    -- virtual camera
    signal s_phi   : unsigned(15 downto 0) := (others => '0');
    signal s_newt  : std_logic := '0';
    signal s_X     : signed(12 downto 0) := (others => '0');
    signal s_T     : signed(12 downto 0) := (others => '0');
    signal s_hidx     : unsigned(7 downto 0) := (others => '0');       -- newest history index
    signal s_romf  : signed(9 downto 0) := (others => '0');
    signal s_sneg  : std_logic := '0';

    -- band loop
    signal s_k     : unsigned(7 downto 0) := (others => '0');
    signal s_rr    : unsigned(15 downto 0) := (others => '0');       -- ramp, Q16
    signal s_g     : unsigned(9 downto 0) := (others => '0');        -- golden sequence
    signal s_dacc  : unsigned(17 downto 0) := (others => '0');       -- lag, Q10 frames
    signal s_x0    : unsigned(C_AW - 1 downto 0) := (others => '0'); -- strip start column
    signal s_xa    : unsigned(7 downto 0) := (others => '0');
    signal s_kpar  : std_logic := '0';
    signal s_lastk : std_logic := '0';                               -- k = N - 1
    signal c_nm1   : unsigned(7 downto 0) := to_unsigned(7, 8);
    signal s_p     : signed(10 downto 0) := (others => '0');
    signal s_offm  : signed(12 downto 0) := (others => '0');
    signal s_offp  : signed(12 downto 0) := (others => '0');
    signal s_off   : signed(12 downto 0) := (others => '0');
    signal s_offu, s_offv : signed(13 downto 0) := (others => '0');
    signal s_dsp   : signed(12 downto 0) := (others => '0');
    signal s_pneg, s_oneg, s_tneg : std_logic := '0';
    signal s_pa, s_oa : unsigned(10 downto 0) := (others => '0');
    signal s_fd, s_fa : signed(11 downto 0) := (others => '0');
    signal s_xs   : unsigned(8 downto 0) := (others => '0');
    signal s_xd   : signed(9 downto 0) := (others => '0');
    signal s_red   : signed(13 downto 0) := (others => '0');
    signal s_ge    : std_logic := '0';
    signal s_hqf   : signed(12 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Line-start sequencer and the pixel-side pointers
    --------------------------------------------------------------------------
    signal s_ls     : unsigned(3 downto 0) := (others => '1');
    signal s_first  : std_logic := '0';
    signal s_band   : unsigned(7 downto 0) := (others => '0');
    signal s_bacc   : unsigned(11 downto 0) := (others => '0');
    signal s_bsum   : unsigned(11 downto 0) := (others => '0');       -- bacc + N
    signal s_bdif   : signed(12 downto 0) := (others => '0');         -- bacc + N - H
    signal s_lib    : unsigned(3 downto 0) := (others => '0');       -- lines in band
    signal s_gapl   : std_logic := '0';                              -- this line is a gap

    signal s_ptr_y, s_ptr_u, s_ptr_v : unsigned(C_AW - 1 downto 0) := to_unsigned(C_EDGE, C_AW);
    signal s_dir_y, s_dir_u, s_dir_v : std_logic := '0';
    signal s_nx_y, s_nx_u, s_nx_v    : std_logic_vector(11 downto 0) := (others => '0');
    signal s_strip  : unsigned(7 downto 0) := (others => '0');
    signal s_se     : signed(12 downto 0) := (others => '0');       -- strip DDA excess
    signal s_gcnt   : unsigned(3 downto 0) := (others => '0');
    signal s_pf     : unsigned(1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel path
    --------------------------------------------------------------------------
    signal s_a_rbank : std_logic := '0';
    signal s_e0_gap, s_e1_gap : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_out_y : unsigned(C_DW - 1 downto 0) := C_Y_BLACK;
    signal s_out_u : unsigned(C_DW - 1 downto 0) := C_C_ZERO;
    signal s_out_v : unsigned(C_DW - 1 downto 0) := C_C_ZERO;

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Control decode (labelled knobs: the firmware may load the label
    -- INDEX, so values below the label count are taken as the index)
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_m : unsigned(13 downto 0);
        variable v_rsw : std_logic_vector(4 downto 0);
    begin
        if rising_edge(clk) then
            v_k := unsigned(registers_in(0)(9 downto 0));
            if v_k < 13 then
                k_bidx <= v_k(3 downto 0);
            else
                v_m := shift_left(resize(v_k, 14), 3) + shift_left(resize(v_k, 14), 2) + resize(v_k, 14);
                k_bidx <= v_m(13 downto 10);                    -- K1 * 13 / 1024
            end if;
            v_k := unsigned(registers_in(1)(9 downto 0));
            if v_k < 5 then
                k_pat <= v_k(2 downto 0);
            else
                v_m := shift_left(resize(v_k, 14), 2) + resize(v_k, 14);
                k_pat <= v_m(12 downto 10);                     -- K2 * 5 / 1024
            end if;
            k_mot   <= unsigned(registers_in(2)(9 downto 0));
            k_span  <= unsigned(registers_in(3)(9 downto 0));
            k_spd   <= unsigned(registers_in(4)(9 downto 0));
            k_split <= unsigned(registers_in(5)(9 downto 0));
            v_rsw   := registers_in(6)(4 downto 0);
            k_axis   <= v_rsw(0);    -- S7  Horizontal / Vertical
            k_mirror <= v_rsw(1);    -- S8  Wrap / Mirror
            k_gap    <= v_rsw(2);    -- S9  Gaps
            k_drift  <= v_rsw(3);    -- S10 Sway / Drift
            k_flip   <= v_rsw(4);    -- S11 Flip
            k_amt   <= unsigned(registers_in(7)(9 downto 0));
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- ROM, tables and history: one registered read, one write each
    --------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_rom_q <= to_signed(C_SINE(to_integer(s_rom_addr)), 10);
        end if;
    end process p_rom;

    p_tabY : process(clk)
    begin
        if rising_edge(clk) then
            s_tqY <= tabY(to_integer(s_ta));
            if s_tw = '1' and s_twch = "00" then tabY(to_integer(s_twa)) <= s_twd; end if;
        end if;
    end process p_tabY;

    p_tabU : process(clk)
    begin
        if rising_edge(clk) then
            s_tqU <= tabU(to_integer(s_ta));
            if s_tw = '1' and s_twch = "01" then tabU(to_integer(s_twa)) <= s_twd; end if;
        end if;
    end process p_tabU;

    p_tabV : process(clk)
    begin
        if rising_edge(clk) then
            s_tqV <= tabV(to_integer(s_ta));
            if s_tw = '1' and s_twch = "10" then tabV(to_integer(s_twa)) <= s_twd; end if;
        end if;
    end process p_tabV;

    p_hist : process(clk)
    begin
        if rising_edge(clk) then
            s_hq <= hist(to_integer(s_ha));
            if s_hw = '1' then hist(to_integer(s_hwa)) <= s_hwd; end if;
        end if;
    end process p_hist;

    --------------------------------------------------------------------------
    -- Per-frame sequencer (vertical blanking), kicked by the guarded vsync
    -- edge.  Frame steps 0..28 set up the constants and the camera; band
    -- steps 32..74 run once per band; step 75 starts line 0.  43 clocks
    -- per band: 128 bands = 5.5k clocks (NTSC has ~13k after vsync).
    -- One small op per step.  Multiplier loads are requested one step
    -- ahead through a registered strobe + select code (no state decode in
    -- front of the operand registers); a product is readable three steps
    -- after its load.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_v    : unsigned(9 downto 0);
        variable v_p    : signed(10 downto 0);
        variable v_d    : unsigned(9 downto 0);
        variable v_rem  : unsigned(8 downto 0);
        variable v_nb   : std_logic;
        variable v_add, v_sub, v_subs, v_fold, v_lin : signed(13 downto 0);
        variable v_s13  : signed(13 downto 0);
        variable v_ph   : unsigned(16 downto 0);
        variable v_dx   : signed(13 downto 0);
        variable v_ent  : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            s_tw <= '0';
            s_hw <= '0';
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            -- requested multiplier load
            s_mreq <= '0';
            if s_mreq = '1' then
                case to_integer(s_msel) is
                    when 0      => s_ma <= resize(k_amt, 11);   s_mb <= resize(k_amt, 11);
                    when 1      => s_ma <= resize(k_mot, 11);   s_mb <= resize(k_mot, 11);
                    when 2      => s_ma <= resize(k_spd, 11);   s_mb <= resize(k_spd, 11);
                    when 3      => s_ma <= resize(k_split, 11); s_mb <= resize(k_split, 11);
                    when 4      => s_ma <= resize(k_span, 11);  s_mb <= resize(k_span, 11);
                    when 5      => s_ma <= s_aq16;              s_mb <= s_W;
                    when 6      => s_ma <= s_mq16;              s_mb <= s_W;
                    when 7      => s_ma <= s_absr;              s_mb <= c_M;
                    when 8      => s_ma <= s_lfb;               s_mb <= c_M;
                    when 9      => s_ma <= resize(c_span, 11);  s_mb <= c_rq(16 downto 6);
                    when 10     => s_ma <= resize(c_nm1, 11);   s_mb <= to_unsigned(633, 11);
                    when 11     => s_ma <= s_pa;                s_mb <= c_A;
                    when others => s_ma <= s_oa;                s_mb <= resize(c_split, 11);
                end case;
            end if;

            -- shared multiplier, operand b split 6/5 (sidewinder)
            s_mpl <= s_ma * s_mb(5 downto 0);
            s_mph <= s_ma * s_mb(10 downto 6);
            s_mp  <= shift_left(resize(s_mph, 22), 6) + resize(s_mpl, 22);

            -- serial restoring divider: dq = dnum / dden, 17 steps MSB first
            if s_dcnt /= 0 then
                v_nb  := s_dnum(16);
                s_dnum <= s_dnum(15 downto 0) & '0';
                v_rem := s_drem & v_nb;
                if v_rem >= resize(s_dden, 9) then
                    s_drem <= resize(v_rem - resize(s_dden, 9), 8);
                    s_dq   <= s_dq(15 downto 0) & '1';
                else
                    s_drem <= v_rem(7 downto 0);
                    s_dq   <= s_dq(15 downto 0) & '0';
                end if;
                s_dcnt <= s_dcnt - 1;
            end if;

            -- edge fold, shared by the three channels (sidewinder): s_red
            -- = start column - L, reduced into [0, mod)
            v_add  := s_red + signed(resize(s_mod, 14));
            v_sub  := s_red - signed(resize(s_mod, 14));
            v_subs := s_red - signed(resize(s_S, 14));
            v_fold := signed(resize(s_C1, 14)) - s_red;     -- 2S + L - 1 - red
            v_lin  := s_red + to_signed(C_EDGE, 14);         -- L + red

            if s_fs /= "1111111" then
                s_fs <= s_fs + 1;
            end if;

            case to_integer(s_fs) is
                ----------------------------------------------------------------
                -- frame set-up
                ----------------------------------------------------------------
                when 0 =>
                    c_N      <= f_nbands(k_bidx);
                    c_pat    <= k_pat;
                    c_axis   <= k_axis;
                    c_mirror <= k_mirror;
                    c_gap    <= k_gap;
                    c_drift  <= k_drift;
                    c_flip   <= k_flip;
                    s_W      <= s_width_m;
                    s_H      <= s_height_m;
                    v_ph     := resize(s_phi, 17) + resize(c_rate, 17);
                    s_phi    <= v_ph(15 downto 0);
                    s_newt   <= v_ph(16);
                    s_mreq <= '1'; s_msel <= to_unsigned(0, 4);   -- Offset^2 -> 4
                when 1 =>
                    s_R <= s_W - to_unsigned(C_EDGE + 1, C_AW);
                    s_S <= s_W - to_unsigned(2 * C_EDGE, C_AW);
                    c_nw <= signed(resize(c_N, 13)) - signed(resize(s_W, 13));
                    c_nm1 <= c_N - 1;
                    c_nh <= signed(resize(c_N, 13)) - signed(resize(s_H, 13));
                    -- rq = 2^16 / N
                    s_dnum <= '1' & x"0000"; s_dden <= c_N;
                    s_drem <= (others => '0'); s_dq <= (others => '0');
                    s_dcnt <= to_unsigned(17, 5);
                    s_mreq <= '1'; s_msel <= to_unsigned(1, 4);   -- Motion^2 -> 5
                when 2 =>
                    if c_mirror = '1' then
                        s_mod <= shift_left(resize(s_S, C_AW + 1), 1);
                    else
                        s_mod <= resize(s_S, C_AW + 1);
                    end if;
                    s_C1 <= shift_left(resize(s_S, C_AW + 1), 1) + to_unsigned(C_EDGE - 1, C_AW + 1);
                    -- gap: ~1/512 of the dimension the blocks are cut across
                    if c_axis = '1' then v_d := resize(s_W(10 downto 9), 10);
                    else                 v_d := resize(s_H(10 downto 9), 10); end if;
                    if v_d = 0 then c_G <= to_unsigned(1, 4); else c_G <= v_d(3 downto 0); end if;
                    s_mreq <= '1'; s_msel <= to_unsigned(2, 4);   -- Speed^2 -> 6
                when 3 =>
                    s_mreq <= '1'; s_msel <= to_unsigned(3, 4);   -- Split^2 -> 7
                when 4 =>
                    s_aq16 <= resize(s_mp(19 downto 10), 11) + 16;
                    s_mreq <= '1'; s_msel <= to_unsigned(4, 4);   -- Span^2 -> 8
                when 5 =>
                    s_mq16 <= resize(s_mp(19 downto 10), 11) + 16;
                    s_mreq <= '1'; s_msel <= to_unsigned(5, 4);   -- A -> 9
                when 6 =>
                    c_rate <= to_unsigned(32, 13) + resize(s_mp(19 downto 8), 13);
                    s_mreq <= '1'; s_msel <= to_unsigned(6, 4);   -- M -> 10
                when 7 =>
                    c_split <= s_mp(19 downto 10);
                when 8 =>       -- Span^2: 4 .. 255 frames
                    v_d := s_mp(19 downto 10);
                    if v_d(9 downto 2) > 251 then
                        c_span <= to_unsigned(255, 8);
                    else
                        c_span <= v_d(9 downto 2) + 4;
                    end if;
                when 9 =>       -- A = (Offset^2 + 16) W / 1024: W/64 .. W
                    c_A <= s_mp(20 downto 10);
                when 10 =>      -- M = (Motion^2 + 16) W / 4096: W/256 .. W/4
                    c_M <= resize(s_mp(21 downto 12), 11);
                    s_rom_addr <= s_phi(15 downto 8);
                when 12 =>
                    s_romf <= s_rom_q;
                when 13 =>      -- |sin|, and a random drift target magnitude
                    if s_romf < 0 then
                        s_absr <= unsigned(resize(-s_romf, 11)); s_sneg <= '1';
                    else
                        s_absr <= unsigned(resize(s_romf, 11));  s_sneg <= '0';
                    end if;
                    s_lfb  <= resize(s_lfsr(7 downto 0), 11);
                    s_tneg <= s_lfsr(8);
                    s_mreq <= '1'; s_msel <= to_unsigned(7, 4);   -- sway -> 17
                when 14 =>
                    s_mreq <= '1'; s_msel <= to_unsigned(8, 4);   -- drift -> 18
                when 17 =>
                    if c_drift = '0' then
                        if s_sneg = '1' then
                            s_X <= -signed(resize(s_mp(20 downto 8), 13));
                        else
                            s_X <= signed(resize(s_mp(20 downto 8), 13));
                        end if;
                    end if;
                when 18 =>      -- new target each time the phase wraps
                    if s_newt = '1' then
                        if s_tneg = '1' then
                            s_T <= -signed(resize(s_mp(20 downto 8), 13));
                        else
                            s_T <= signed(resize(s_mp(20 downto 8), 13));
                        end if;
                    end if;
                when 19 =>      -- drift: ease 1/16 of the way per frame
                    if c_drift = '1' then
                        v_dx := resize(s_T, 14) - resize(s_X, 14);
                        s_X <= s_X + resize(shift_right(v_dx, 4), 13);
                    end if;
                when 20 =>      -- record this frame's camera position
                    s_hidx <= s_hidx + 1;
                    s_hw   <= '1';
                    s_hwa  <= s_hidx + 1;
                    s_hwd  <= std_logic_vector(resize(s_X, 16));
                when 21 =>      -- wait for rq; then W / N for the strip starts
                    if s_dcnt = 0 then
                        c_rq <= s_dq;
                        s_dnum <= resize(s_W, 17); s_dden <= c_N;
                        s_drem <= (others => '0'); s_dq <= (others => '0');
                        s_dcnt <= to_unsigned(17, 5);
                        s_mreq <= '1'; s_msel <= to_unsigned(9, 4);   -- lag -> 25
                    else
                        s_fs <= s_fs;
                    end if;
                when 22 =>
                    s_mreq <= '1'; s_msel <= to_unsigned(10, 4);  -- (N-1) golden -> 26
                when 25 =>      -- lag per band = span / N frames (Q10)
                    c_lag <= s_mp(17 downto 0);
                when 26 =>
                    c_gfl <= s_mp(9 downto 0);
                when 27 =>
                    if s_dcnt = 0 then
                        c_q  <= s_dq(C_AW - 1 downto 0);
                        c_r  <= s_drem;
                    else
                        s_fs <= s_fs;
                    end if;
                when 28 =>      -- band loop initial state
                    c_q1 <= c_q + 1;
                    s_k  <= (others => '0');
                    s_x0 <= (others => '0');
                    s_xa <= c_nm1;
                    if c_flip = '1' then
                        s_rr   <= to_unsigned(0, 16) - c_rq(15 downto 0);
                        s_g    <= c_gfl;
                        s_dacc <= shift_left(resize(c_span, 18), 10) - c_lag;
                        s_kpar <= not c_N(0);
                    else
                        s_rr   <= (others => '0');
                        s_g    <= (others => '0');
                        s_dacc <= (others => '0');
                        s_kpar <= '0';
                    end if;
                    s_fs <= to_unsigned(32, 7);

                ----------------------------------------------------------------
                -- band loop
                ----------------------------------------------------------------
                when 32 =>      -- wave lookup, history lookup (h - d), fold
                    s_rom_addr <= s_rr(15 downto 8);
                    if s_k = c_nm1 then s_lastk <= '1'; else s_lastk <= '0'; end if;
                    s_ha <= s_hidx - s_dacc(17 downto 10);
                    s_fd <= signed(shift_left(resize(s_rr(15 downto 6), 12), 1)) - 1023;
                when 33 =>
                    if s_fd < 0 then s_fa <= -s_fd; else s_fa <= s_fd; end if;
                when 34 =>
                    s_romf <= s_rom_q;
                    s_hqf  <= signed(s_hq(12 downto 0));
                when 35 =>      -- pattern -512..511 (Fold 0..1023); motion lag
                    v_v := s_rr(15 downto 6);
                    case to_integer(c_pat) is
                        when 0 =>       -- Ramp
                            v_p := signed(resize(v_v, 11)) - 512;
                        when 1 =>       -- Fold: edges far, centre home
                            v_p := s_fa(10 downto 0);
                        when 2 =>       -- Random (golden-ratio scatter)
                            v_p := signed(resize(s_g, 11)) - 512;
                        when 3 =>       -- Wave
                            v_p := shift_left(resize(s_romf, 11), 1);
                        when others =>  -- Zigzag
                            if s_kpar = '1' then v_p := to_signed(511, 11);
                            else v_p := to_signed(-512, 11); end if;
                    end case;
                    s_p <= v_p;
                    s_offm <= s_X - s_hqf;
                when 36 =>      -- |p| -> mul by A
                    if s_p < 0 then
                        s_pa <= unsigned(-s_p); s_pneg <= '1';
                    else
                        s_pa <= unsigned(s_p);  s_pneg <= '0';
                    end if;
                    s_mreq <= '1'; s_msel <= to_unsigned(11, 4);  -- -> 40
                when 40 =>
                    if s_pneg = '1' then
                        s_offp <= -signed(resize(s_mp(20 downto 10), 13));
                    else
                        s_offp <= signed(resize(s_mp(20 downto 10), 13));
                    end if;
                when 41 =>      -- offset, clamped to +-2047
                    v_s13 := resize(s_offp, 14) + resize(s_offm, 14);
                    if v_s13 > 2047 then
                        s_off <= to_signed(2047, 13);
                    elsif v_s13 < -2047 then
                        s_off <= to_signed(-2047, 13);
                    else
                        s_off <= v_s13(12 downto 0);
                    end if;
                when 42 =>      -- |off| -> mul by split
                    if s_off < 0 then
                        v_s13 := -resize(s_off, 14); s_oneg <= '1';
                    else
                        v_s13 := resize(s_off, 14);  s_oneg <= '0';
                    end if;
                    s_oa <= unsigned(v_s13(10 downto 0));
                    s_mreq <= '1'; s_msel <= to_unsigned(12, 4);  -- -> 46
                when 46 =>
                    if s_oneg = '1' then
                        s_dsp <= -signed(resize(s_mp(20 downto 10), 13));
                    else
                        s_dsp <= signed(resize(s_mp(20 downto 10), 13));
                    end if;
                    s_xs <= resize(s_xa, 9) + resize(c_r, 9);
                when 47 =>
                    s_offu <= resize(s_off, 14) + resize(s_dsp, 14);
                    s_offv <= resize(s_off, 14) - resize(s_dsp, 14);
                    s_xd <= signed('0' & s_xs) - signed(resize(c_N, 10));
                -- Fold each channel's start column into [L, R]: load, six
                -- reduce steps, mirror-half test, table write.
                when 48 =>
                    if c_axis = '1' then
                        s_red <= signed(resize(s_x0, 14)) + resize(s_off, 14);
                    else
                        s_red <= resize(s_off, 14);
                    end if;
                when 57 =>
                    if c_axis = '1' then
                        s_red <= signed(resize(s_x0, 14)) + s_offu;
                    else
                        s_red <= s_offu;
                    end if;
                when 66 =>
                    if c_axis = '1' then
                        s_red <= signed(resize(s_x0, 14)) + s_offv;
                    else
                        s_red <= s_offv;
                    end if;
                when 49 | 50 | 51 | 52 | 53 | 54 | 58 | 59 | 60 | 61 | 62 | 63
                   | 67 | 68 | 69 | 70 | 71 | 72 =>
                    if s_red(13) = '1' then
                        s_red <= v_add;
                    elsif v_sub(13) = '0' then
                        s_red <= v_sub;
                    end if;
                when 55 | 64 | 73 =>
                    s_ge <= c_mirror and not v_subs(13);
                when 56 | 65 | 74 =>
                    if s_ge = '1' then
                        v_ent := "0000" & '1' & std_logic_vector(v_fold(C_AW - 1 downto 0));
                    else
                        v_ent := "0000" & '0' & std_logic_vector(v_lin(C_AW - 1 downto 0));
                    end if;
                    s_tw  <= '1';
                    s_twa <= s_k;
                    s_twd <= v_ent;
                    if s_fs = 56 then
                        s_twch <= "00";
                    elsif s_fs = 65 then
                        s_twch <= "01";
                    else
                        s_twch <= "10";
                        -- next band
                        s_k <= s_k + 1;
                        if c_flip = '1' then
                            s_rr   <= s_rr - c_rq(15 downto 0);
                            s_g    <= s_g - 633;
                            s_dacc <= s_dacc - c_lag;
                        else
                            s_rr   <= s_rr + c_rq(15 downto 0);
                            s_g    <= s_g + 633;
                            s_dacc <= s_dacc + c_lag;
                        end if;
                        s_kpar <= not s_kpar;
                        -- next strip start = x0 + W div N (+1 as the
                        -- remainder accumulates): ceil(k W / N)
                        if s_xd(9) = '0' then
                            s_xa <= unsigned(s_xd(7 downto 0));
                            s_x0 <= s_x0 + c_q1;
                        else
                            s_xa <= s_xs(7 downto 0);
                            s_x0 <= s_x0 + c_q;
                        end if;
                        if s_lastk = '1' then
                            s_fs <= to_unsigned(75, 7);
                        else
                            s_fs <= to_unsigned(32, 7);
                        end if;
                    end if;
                when 75 =>      -- tables done: p_line starts line 0
                    s_fs <= (others => '1');
                when others =>
                    null;
            end case;

            -- New field: vsync falling edge, guarded so serrated analog
            -- vsync fires once per field.
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw_active = '1' then
                s_fs <= (others => '0');
            end if;
        end if;
    end process p_frame;

    --------------------------------------------------------------------------
    -- E0: input registers, write column, line events, line-start sequencer,
    -- pointer steps / strip loads.
    --------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n   : unsigned(C_AW downto 0);
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

            -- Write column: pixel k of the line lands at address k.
            if s_in_avid = '1' then
                s_wr_x <= s_wr_x + 1;
            else
                s_wr_x <= (others => '0');
            end if;

            -- table read data goes through a fabric register before any
            -- pointer load; a strip prefetch lands three clocks after its
            -- address (strips are >= 5 px)
            s_tfY <= s_tqY(11 downto 0);
            s_tfU <= s_tqU(11 downto 0);
            s_tfV <= s_tqV(11 downto 0);
            if s_pf /= 0 then s_pf <= s_pf - 1; end if;
            if s_pf = 1 then
                s_nx_y <= s_tfY;
                s_nx_u <= s_tfU;
                s_nx_v <= s_tfV;
            end if;

            -- Pointers: one step per active pixel; in Vertical mode the
            -- pixel after a strip boundary loads the next strip's entry.
            -- Strip DDA kept as the excess e = (p N mod W) + N - W, so
            -- "the next pixel starts a strip" is e's sign bit, straight
            -- from a register, and the update is one add of N - W or N.
            if data_in.avid = '1' then
                if s_se(12) = '0' then
                    s_se <= s_se + c_nw;
                else
                    s_se <= s_se + signed(resize(c_N, 13));
                end if;
                if c_axis = '1' and s_se(12) = '0' then
                    s_strip <= s_strip + 1;
                    s_gcnt  <= (others => '0');
                    s_ptr_y <= unsigned(s_nx_y(10 downto 0)); s_dir_y <= s_nx_y(11);
                    s_ptr_u <= unsigned(s_nx_u(10 downto 0)); s_dir_u <= s_nx_u(11);
                    s_ptr_v <= unsigned(s_nx_v(10 downto 0)); s_dir_v <= s_nx_v(11);
                    s_ta    <= s_strip + 2;
                    s_pf    <= "11";
                else
                    if s_gcnt /= 15 then s_gcnt <= s_gcnt + 1; end if;
                    v_n := f_bounce(s_ptr_y, s_dir_y, s_R, c_mirror);
                    s_ptr_y <= v_n(C_AW - 1 downto 0); s_dir_y <= v_n(C_AW);
                    v_n := f_bounce(s_ptr_u, s_dir_u, s_R, c_mirror);
                    s_ptr_u <= v_n(C_AW - 1 downto 0); s_dir_u <= v_n(C_AW);
                    v_n := f_bounce(s_ptr_v, s_dir_v, s_R, c_mirror);
                    s_ptr_v <= v_n(C_AW - 1 downto 0); s_dir_v <= v_n(C_AW);
                end if;
            end if;

            -- gap flag for the pixel read this clock
            if c_gap = '1' and ((c_axis = '1' and s_strip /= 0 and s_gcnt < c_G)
                                or (c_axis = '0' and s_gapl = '1')) then
                s_e0_gap <= '1';
            else
                s_e0_gap <= '0';
            end if;

            ------------------------------------------------------------------
            -- Line-start sequencer (horizontal blanking): band DDA, then the
            -- line's first entry (its band, or strip 0) into the pointers,
            -- then strip 1 into the prefetch registers.
            ------------------------------------------------------------------
            if s_ls /= "1111" then
                s_ls <= s_ls + 1;
            end if;
            case to_integer(s_ls) is
                when 0 =>       -- band DDA, part 1: both candidates
                    s_bsum <= s_bacc + resize(c_N, 12);
                    s_bdif <= signed(resize(s_bacc, 13)) + c_nh;
                when 1 =>       -- part 2: pick by the sign of the excess
                    if s_first = '1' then
                        s_band <= (others => '0');
                        s_bacc <= (others => '0');
                        s_lib  <= (others => '0');
                        s_ta   <= (others => '0');
                    elsif s_bdif(12) = '0' then
                        s_bacc <= unsigned(s_bdif(11 downto 0));
                        s_band <= s_band + 1;
                        s_lib  <= (others => '0');
                        s_ta   <= s_band + 1;
                    else
                        s_bacc <= s_bsum;
                        if s_lib /= 15 then s_lib <= s_lib + 1; end if;
                        s_ta   <= s_band;
                    end if;
                    s_first <= '0';
                when 2 =>
                    if c_axis = '1' then s_ta <= (others => '0'); end if;
                    if s_band /= 0 and s_lib < c_G then s_gapl <= '1'; else s_gapl <= '0'; end if;
                when 5 =>       -- entry (address set in step 1/2) via s_tf
                    s_ptr_y <= unsigned(s_tfY(10 downto 0)); s_dir_y <= s_tfY(11);
                    s_ptr_u <= unsigned(s_tfU(10 downto 0)); s_dir_u <= s_tfU(11);
                    s_ptr_v <= unsigned(s_tfV(10 downto 0)); s_dir_v <= s_tfV(11);
                    s_ta <= to_unsigned(1, 8);
                when 8 =>
                    s_nx_y <= s_tfY;
                    s_nx_u <= s_tfU;
                    s_nx_v <= s_tfV;
                    s_strip <= (others => '0');
                    s_se    <= c_nw;
                    s_gcnt  <= (others => '0');
                    s_ls <= "1111";
                when others =>
                    null;
            end case;

            -- Active-video fall: flip parity, count the line, capture height
            -- and width, start the next line.
            if s_in_avid = '1' and data_in.avid = '0' then
                s_par      <= not s_par;
                s_aline    <= s_aline + 1;
                s_height_m <= s_aline + 1;
                if s_aline = to_unsigned(C_WIDTH_LINE, 11) then
                    s_width_m <= s_wr_x + 1;
                end if;
                s_ls <= (others => '0');
            end if;
            -- Frame tables ready: start line 0.
            if s_fs = 75 then
                s_first <= '1';
                s_ls <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw_active = '1' then
                s_saw_active <= '0';
                s_aline      <= (others => '0');
            end if;

            -- Sync shift registers
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_line;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank (single write port). Write = the
    -- E0-registered pixel into this line's parity bank; read = the pointer,
    -- registered out.
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
    -- E2: blanking gate and gaps.
    --------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            s_a_rbank <= not s_par;
            s_e1_gap  <= s_e0_gap;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                s_wet_u <= unsigned(s_rdU1);
                s_wet_v <= unsigned(s_rdV1);
            else
                s_wet_y <= unsigned(s_rdY0);
                s_wet_u <= unsigned(s_rdU0);
                s_wet_v <= unsigned(s_rdV0);
            end if;

            if s_avid_sr(1) = '1' and s_e1_gap = '0' then
                s_out_y <= s_wet_y;
                s_out_u <= s_wet_u;
                s_out_v <= s_wet_v;
            else
                s_out_y <= C_Y_BLACK;
                s_out_u <= C_C_ZERO;
                s_out_v <= C_C_ZERO;
            end if;
        end if;
    end process p_out;

    --------------------------------------------------------------------------
    -- Output (latency 3 for syncs and video in every mode).
    --------------------------------------------------------------------------
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    s_io_align_in.y <= std_logic_vector(s_out_y);
    s_io_align_in.u <= std_logic_vector(s_out_u);
    s_io_align_in.v <= std_logic_vector(s_out_v);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (3 -> 4).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture slitscan;
