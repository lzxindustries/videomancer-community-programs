-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Stratamix: catalogue combo #132 (seed 20260927) -- depth-from-luma
-- parallax + posterization + multi-source mixing (add, screen, multiply,
-- difference, overlay).  Two knobs each; the switches and the slider decide
-- how they combine.
--
--   K1 Depth    parallax gain, bipolar (luma above the pivot shifts one way, below it the other)
--   K2 Pivot    luma level that does not move
--   K3 Levels   posterize luma to 2 .. 64 levels (labelled, sixths of the travel)
--   K4 Tint     chroma tint per luma band (the strata take alternating hues)
--   K5 Mode     add / screen / multiply / difference / overlay (fifths of the travel)
--   K6 Mix      how far the blend replaces the displaced layer
--
--   S7  Order   posterize before the shift (stepped depth: terraces) / after the mix
--   S8  Layers  all strata shift the same way / alternate levels shift opposite ways
--   S9  Chroma  colour from the displaced layer / from the still picture (colour ghosts)
--   S10 Key     blend everywhere / only above the pivot (the background stays put)
--   S11 Contour plain / black contour lines at every level boundary
--   P12 Strata  depth and mix rise together: at 100 % the picture separates into stepped terraces
--
-- Operand A = the line above read at x + (luma - pivot) * depth (edge-filled),
-- operand B = the still picture from a dry EBR ring.  Both are black-relative
-- and clamped to white (W = 876); two 10x10 products scaled by 1/W give
-- multiply, screen and overlay; floor quantizers with registered mask / half
-- run before the shift or after the mix.  Chroma is fetched late (delayed
-- read address / a second, deeper ring) so input video stays shallow; the
-- replayed line's U/V swap where its hsync->avid parity differs from this
-- row's, and the tint swaps on rows whose parity is not the usual one
-- (drape v1.3).  Output: highlight knee (940 -> 770) + [1 2 1] on Y/U/V
-- (HD HDMI bar guard).  Latency 29 clocks, all modes, blanking-gated.
--
-- v0.2.2 (2026-10-18): resolution scaling.  The depth offset is in pixels, so
-- the gain follows the timing ID's x step (1080 x1, 720p x2/3, SD x1/2) and
-- so does the reach clamp (511 / 341 / 255 px): the same shift relative to
-- the screen height in every mode.  v0.2.0 grouped 720p with 1080p (1.5x
-- the shift).  1080p is bit-identical.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture stratamix of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (29 + 3 = 32 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 32;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 29;
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    -- white, black-relative (940 - 64)
    constant C_W    : integer := 876;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    function f_edge_addr(v : signed; we : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > signed(resize(we, v'length)) then
            return we;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

    -- floor quantizer: keep the masked bits, sit on the step's half
    function f_quant(y : unsigned(9 downto 0); m : unsigned(9 downto 0); h : unsigned(9 downto 0)) return unsigned is
    begin
        return (y and m) or h;
    end function;

    -- black-relative luma clamped to 0 .. W
    function f_brel(y : unsigned(9 downto 0)) return unsigned is
    begin
        if y < 64 then return to_unsigned(0, 10);
        elsif y > 940 then return to_unsigned(C_W, 10);
        else return y - 64; end if;
    end function;

    -- [1 2 1] / 4 of three 10-bit taps
    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
    end function;

    -- labelled knobs: the firmware divides the travel evenly across the labels
    function f_zone5(v : unsigned(9 downto 0)) return unsigned is
    begin
        if v >= 820 then return to_unsigned(4, 3);
        elsif v >= 615 then return to_unsigned(3, 3);
        elsif v >= 410 then return to_unsigned(2, 3);
        elsif v >= 205 then return to_unsigned(1, 3);
        else return to_unsigned(0, 3);
        end if;
    end function;

    function f_zone6(v : unsigned(9 downto 0)) return unsigned is
    begin
        if v >= 854 then return to_unsigned(5, 3);
        elsif v >= 683 then return to_unsigned(4, 3);
        elsif v >= 512 then return to_unsigned(3, 3);
        elsif v >= 342 then return to_unsigned(2, 3);
        elsif v >= 171 then return to_unsigned(1, 3);
        else return to_unsigned(0, 3);
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Line buffers + dry rings
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ring_y is array (0 to 31) of std_logic_vector(9 downto 0);
    type t_ring_c is array (0 to 31) of std_logic_vector(19 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal ringY : t_ring_y := (others => "0001000000");
    signal ringC : t_ring_c := (others => x"80200");
    -- every bank is read only while the other is written; the rings read
    -- 8 / 23 behind the write pointer
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1, ringY, ringC : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_ringY_q : std_logic_vector(9 downto 0) := "0001000000";
    signal s_ringC_q : std_logic_vector(19 downto 0) := x"80200";
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_depth, s_k_pivot, s_k_levels, s_k_tint, s_k_mode, s_k_mix : unsigned(9 downto 0);
    signal s_sw_after, s_sw_alt, s_sw_bchroma, s_sw_key, s_sw_cont : std_logic;
    signal s_p_strata : unsigned(9 downto 0);

    -- labelled knobs: zone with +-10 hysteresis; label index until first live read
    signal s_mo_m, s_mo_p, s_lv_m, s_lv_p : unsigned(9 downto 0) := (others => '0');
    signal s_zmo_m, s_zmo_p, s_zlv_m, s_zlv_p : unsigned(2 downto 0) := (others => '0');
    signal s_mo_live, s_lv_live : std_logic := '0';
    signal s_levels : unsigned(2 downto 0) := to_unsigned(2, 3);

    -- glides (Q10.2, a quarter of the way per field)
    signal s_kd, s_kp : unsigned(9 downto 0) := (others => '0');
    signal s_dep, s_p12 : unsigned(11 downto 0) := (others => '0');
    signal s_ddep, s_dp12 : signed(12 downto 0) := (others => '0');

    signal s_gk, s_g_e : signed(10 downto 0) := (others => '0');
    -- v0.2.2 resolution scaling: the offsets are pixels, so the gain and the
    -- reach follow the timing ID's x step (1080: 1.0 virtual unit a pixel,
    -- 720p 1.5, SD 2.0): the same shift relative to the screen height
    signal s_sd     : std_logic := '0';
    signal s_720    : std_logic := '0';
    signal s_ga     : signed(10 downto 0) := (others => '0');
    signal s_limp : signed(12 downto 0) := to_signed(511, 13);
    signal s_limn : signed(12 downto 0) := to_signed(-511, 13);
    signal s_pivot  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_mask   : unsigned(9 downto 0) := "1110000000";
    signal s_half   : unsigned(9 downto 0) := "0001000000";
    signal s_stepm  : unsigned(9 downto 0) := "0001111111";
    signal s_pbit   : unsigned(9 downto 0) := "0010000000";
    signal s_cw     : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_tint   : unsigned(7 downto 0) := (others => '0');
    signal s_mode   : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_mix, s_mix_e : unsigned(7 downto 0) := (others => '0');
    signal s_p      : unsigned(9 downto 0) := (others => '0');
    signal s_mpx    : unsigned(17 downto 0) := (others => '0');
    signal s_vstep  : unsigned(3 downto 0) := "1111";

    --------------------------------------------------------------------------
    -- Tracking / write path
    --------------------------------------------------------------------------
    signal s_rx, s_wr_x, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';
    -- chroma phase: hsync->avid parity of this line, the stored line, and the usual one
    signal s_hpar, s_line_par, s_cap_par, s_swap, s_tswap : std_logic := '0';
    signal s_pvote  : unsigned(3 downto 0) := "1000";
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_wu2, s_wv2 : unsigned(9 downto 0) := (others => '0');
    signal s_wx2    : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_wpar2 : std_logic := '0';

    --------------------------------------------------------------------------
    -- Address chain (stages 1..8); chroma address carried to stage 23
    --------------------------------------------------------------------------
    type t_x is array (1 to 6) of unsigned(10 downto 0);
    signal s_x      : t_x := (others => (others => '0'));
    signal s_dq2    : unsigned(9 downto 0) := (others => '0');
    signal s_par2, s_key2, s_par3 : std_logic := '0';
    signal s_d3, s_d4 : signed(10 downto 0) := (others => '0');
    signal s_g4     : signed(10 downto 0) := (others => '0');
    signal s_pd5    : signed(21 downto 0) := (others => '0');
    signal s_off6   : signed(10 downto 0) := (others => '0');
    signal s_u7     : signed(12 downto 0) := (others => '0');
    signal s_addr8  : unsigned(10 downto 0) := (others => '0');
    signal s_keyd   : std_logic_vector(3 to 19) := (others => '0');
    type t_cad is array (9 to 23) of unsigned(9 downto 0);
    signal s_cad    : t_cad := (others => (others => '0'));
    signal s_cbk, s_csw : std_logic_vector(9 to 24) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel path (stages 10..29): operands, blend, mix toward A, posterize,
    -- tint, knee, [1 2 1], gate
    --------------------------------------------------------------------------
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y10, s_by10 : unsigned(9 downto 0) := (others => '0');
    signal s_a11, s_bq11 : unsigned(9 downto 0) := (others => '0');
    signal s_a12, s_b12, s_na12 : unsigned(9 downto 0) := (others => '0');
    signal s_a13, s_b13, s_na13, s_nb13 : unsigned(9 downto 0) := (others => '0');
    signal s_sum13, s_sum14, s_sum15, s_sum16 : unsigned(10 downto 0) := (others => '0');
    signal s_dif13, s_dif14 : signed(10 downto 0) := (others => '0');
    signal s_cdif15, s_cdif16 : unsigned(9 downto 0) := (others => '0');
    signal s_p14, s_q14 : unsigned(19 downto 0) := (others => '0');
    signal s_a14    : unsigned(9 downto 0) := (others => '0');
    signal s_alo15, s_alo16 : std_logic := '0';
    signal s_ps15, s_pt15, s_qs15, s_qt15 : unsigned(10 downto 0) := (others => '0');
    signal s_pn16, s_qn16 : unsigned(10 downto 0) := (others => '0');
    signal s_c_add17, s_c_scr17, s_c_mul17, s_c_dif17, s_c_ovl17 : signed(12 downto 0) := (others => '0');
    signal s_sel18  : signed(12 downto 0) := (others => '0');
    signal s_t19    : unsigned(9 downto 0) := (others => '0');
    signal s_dy20   : signed(10 downto 0) := (others => '0');
    signal s_mx20   : unsigned(7 downto 0) := (others => '0');
    signal s_base20, s_base21 : unsigned(9 downto 0) := (others => '0');
    signal s_py21   : signed(19 downto 0) := (others => '0');
    signal s_ys22   : signed(12 downto 0) := (others => '0');
    signal s_y23, s_yq24, s_y25, s_y26 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_tu25, s_tv25 : signed(8 downto 0) := (others => '0');
    signal s_su26, s_sv26 : signed(11 downto 0) := to_signed(512, 12);
    signal s_yk26   : signed(10 downto 0) := (others => '0');
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ua, s_ub, s_va, s_vb : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29    : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_ad is array (12 to 19) of unsigned(9 downto 0);
    signal s_ad     : t_ad := (others => (others => '0'));
    signal s_cont   : std_logic_vector(11 to 26) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_depth  <= unsigned(registers_in(0));
    s_k_pivot  <= unsigned(registers_in(1));
    s_k_levels <= unsigned(registers_in(2));
    s_k_tint   <= unsigned(registers_in(3));
    s_k_mode   <= unsigned(registers_in(4));
    s_k_mix    <= unsigned(registers_in(5));
    s_sw_after   <= registers_in(6)(0);
    s_sw_alt     <= registers_in(6)(1);
    s_sw_bchroma <= registers_in(6)(2);
    s_sw_key     <= registers_in(6)(3);
    s_sw_cont    <= registers_in(6)(4);
    s_p_strata <= unsigned(registers_in(7));

    -- dry rings: luma delayed 9 (q at stage 9), chroma delayed 24 (q at stage 24)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ringY_q <= ringY(to_integer(s_ring_w - 8));
            ringY(to_integer(s_ring_w)) <= data_in.y;
        end if;
    end process;
    p_ringc : process(clk)
    begin
        if rising_edge(clk) then
            s_ringC_q <= ringC(to_integer(s_ring_w - 23));
            ringC(to_integer(s_ring_w)) <= data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_kl : integer range 4 to 9;
        variable v_g  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_w_edge <= s_line_width - (C_EDGE + 1);

            -- labelled knobs (K5 Mode fifths, K3 Levels sixths): a zone takes
            -- when the knob +-10 agree.  The firmware loads a labelled default
            -- as the label INDEX: honour it until the knob first reads a real
            -- position.
            if s_k_mode >= 10 then s_mo_m <= s_k_mode - 10; else s_mo_m <= (others => '0'); end if;
            if s_k_mode <= 1013 then s_mo_p <= s_k_mode + 10; else s_mo_p <= (others => '1'); end if;
            s_zmo_m <= f_zone5(s_mo_m); s_zmo_p <= f_zone5(s_mo_p);
            if s_k_mode >= 5 then s_mo_live <= '1'; end if;
            if s_mo_live = '0' and s_k_mode < 5 then
                s_mode <= s_k_mode(2 downto 0);
            elsif s_zmo_m = s_zmo_p then
                s_mode <= s_zmo_m;
            end if;
            if s_k_levels >= 10 then s_lv_m <= s_k_levels - 10; else s_lv_m <= (others => '0'); end if;
            if s_k_levels <= 1013 then s_lv_p <= s_k_levels + 10; else s_lv_p <= (others => '1'); end if;
            s_zlv_m <= f_zone6(s_lv_m); s_zlv_p <= f_zone6(s_lv_p);
            if s_k_levels >= 6 then s_lv_live <= '1'; end if;
            if s_lv_live = '0' and s_k_levels < 6 then
                s_levels <= s_k_levels(2 downto 0);
            elsif s_zlv_m = s_zlv_p then
                s_levels <= s_zlv_m;
            end if;

            -- timing-ID class (static while a mode runs) and the reach clamp
            -- in this mode's pixels: 511 at 1080, 341 at 720p, 255 at SD
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 =>
                    s_sd <= '1'; s_720 <= '0';
                    s_limp <= to_signed(255, 13); s_limn <= to_signed(-255, 13);
                when 5 | 6 | 14 =>
                    s_sd <= '0'; s_720 <= '1';
                    s_limp <= to_signed(341, 13); s_limn <= to_signed(-341, 13);
                when others =>
                    s_sd <= '0'; s_720 <= '0';
                    s_limp <= to_signed(511, 13); s_limn <= to_signed(-511, 13);
            end case;

            -- chroma phase (drape v1.3): parity of the hsync->avid offset
            s_hpar <= not s_hpar;
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_line_par <= s_hpar;
                if s_hpar = '1' and s_pvote /= 15 then s_pvote <= s_pvote + 1;
                elsif s_hpar = '0' and s_pvote /= 0 then s_pvote <= s_pvote - 1; end if;
            end if;
            s_swap  <= s_line_par xor s_cap_par;    -- replayed line vs this row
            s_tswap <= s_line_par xor s_pvote(3);   -- this row vs the usual phase

            -- per-field decode (one op per state; the one product has its
            -- operands registered a state ahead)
            case to_integer(s_vstep) is
                when 0 =>
                    s_kd    <= s_k_depth;
                    s_kp    <= s_p_strata;
                    s_pivot <= s_k_pivot;
                    s_tint  <= s_k_tint(9 downto 2);
                    s_mix   <= s_k_mix(9 downto 2);
                    case to_integer(s_levels) is
                        when 0 => v_kl := 9;
                        when 1 => v_kl := 8;
                        when 2 => v_kl := 7;
                        when 3 => v_kl := 6;
                        when 4 => v_kl := 5;
                        when others => v_kl := 4;
                    end case;
                    for i in 0 to 9 loop
                        if i >= v_kl then s_mask(i) <= '1'; else s_mask(i) <= '0'; end if;
                        if i = v_kl - 1 then s_half(i) <= '1'; else s_half(i) <= '0'; end if;
                        if i < v_kl then s_stepm(i) <= '1'; else s_stepm(i) <= '0'; end if;
                        if i = v_kl then s_pbit(i) <= '1'; else s_pbit(i) <= '0'; end if;
                    end loop;
                    case v_kl is
                        when 9 => s_cw <= to_unsigned(64, 7);
                        when 8 => s_cw <= to_unsigned(32, 7);
                        when 7 => s_cw <= to_unsigned(16, 7);
                        when 6 => s_cw <= to_unsigned(8, 7);
                        when others => s_cw <= to_unsigned(4, 7);
                    end case;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_ddep <= signed(shift_left(resize(s_kd, 13), 2)) - signed(resize(s_dep, 13));
                    s_dp12 <= signed(shift_left(resize(s_kp, 13), 2)) - signed(resize(s_p12, 13));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    v_g := signed(resize(s_dep, 13)) + shift_right(s_ddep, 2); s_dep <= unsigned(v_g(11 downto 0));
                    v_g := signed(resize(s_p12, 13)) + shift_right(s_dp12, 2); s_p12 <= unsigned(v_g(11 downto 0));
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_gk <= signed(resize(s_dep(11 downto 2), 11)) - to_signed(512, 11);
                    s_p  <= s_p12(11 downto 2);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_mpx <= (not s_mix) * s_p;                                         -- (255 - mix) * p
                    if s_gk >= 0 then
                        s_g_e <= s_gk + signed(resize(s_p(9 downto 1), 11));
                    else
                        s_g_e <= s_gk - signed(resize(s_p(9 downto 1), 11));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_mix_e <= s_mix + s_mpx(17 downto 10);
                    -- pixels per virtual unit: SD half the 1080 gain, 720p
                    -- two thirds: (1/2 + 1/8) x 17/16 = 0.664 over states
                    -- 5..8.  Each add takes one operand from the s_ga copy:
                    -- x + (x >> k) on one signed register feeds the sign bit
                    -- to both carry inputs (the router1 self-add hang)
                    if s_sd = '1' then
                        s_g_e <= shift_right(s_g_e, 1);
                    end if;
                    s_ga <= shift_right(s_g_e, 1);
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    if s_720 = '1' then s_g_e <= s_ga + shift_right(s_g_e, 3); end if;
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_ga <= shift_right(s_g_e, 4);
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    if s_720 = '1' then s_g_e <= s_g_e + s_ga; end if;
                    s_vstep <= "1111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
                s_cap_par <= s_line_par;   -- parity of the line now in the read bank
                s_hpar <= '0';
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
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
    -- Write path (stage 2) + address chain (stages 1..8, chroma to 23)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1
            s_x(1) <= s_rx;
            for i in 2 to 6 loop s_x(i) <= s_x(i - 1); end loop;
            -- 2: depth luma (posterized before the shift), write data, step parity, key
            if s_sw_after = '0' then s_dq2 <= f_quant(s_in_y, s_mask, s_half); else s_dq2 <= s_in_y; end if;
            s_wu2 <= s_in_u; s_wv2 <= s_in_v;
            s_wx2 <= s_wr_x; s_we2 <= s_in_avid; s_wpar2 <= s_wpar;
            if (s_in_y and s_pbit) /= 0 then s_par2 <= '1'; else s_par2 <= '0'; end if;
            if s_in_y >= s_pivot then s_key2 <= '1'; else s_key2 <= '0'; end if;
            -- 3: depth difference
            s_d3 <= signed(resize(s_dq2, 11)) - signed(resize(s_pivot, 11));
            s_par3 <= s_par2;
            s_keyd(3) <= s_key2;
            for i in 4 to 19 loop s_keyd(i) <= s_keyd(i - 1); end loop;
            -- 4: gain (alternate levels reversed), registered ahead of the product
            if s_sw_alt = '1' and s_par3 = '1' then s_g4 <= -s_g_e; else s_g4 <= s_g_e; end if;
            s_d4 <= s_d3;
            -- 5: displacement product (11 x 11)
            s_pd5 <= s_d4 * s_g4;
            -- 6: offset (+-511 px at 1080, the same reach scaled at 720p / SD)
            v_w := resize(shift_right(s_pd5, 9), 13);
            if v_w > s_limp then v_w := s_limp; elsif v_w < s_limn then v_w := s_limn; end if;
            s_off6 <= resize(v_w, 11);
            -- 7: source column
            s_u7 <= signed(resize(s_x(6), 13)) + resize(s_off6, 13);
            -- 8: edge fill
            s_addr8 <= f_edge_addr(s_u7, s_w_edge);
            -- 9..23: chroma read address, bank and swap (the tail of an SD
            -- line reads past hsync, so the bank travels with the address)
            s_cad(9) <= s_addr8(10 downto 1);
            for i in 10 to 23 loop s_cad(i) <= s_cad(i - 1); end loop;
            s_cbk(9) <= s_rbank; s_csw(9) <= s_swap;
            for i in 10 to 24 loop s_cbk(i) <= s_cbk(i - 1); s_csw(i) <= s_csw(i - 1); end loop;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 2; luma read stage 8 -> data 9, chroma read
    -- stage 23 -> data 24)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr8));
            if s_we2 = '1' and s_wpar2 = '0' then lbY0(to_integer(s_wx2)) <= std_logic_vector(s_dq2); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr8));
            if s_we2 = '1' and s_wpar2 = '1' then lbY1(to_integer(s_wx2)) <= std_logic_vector(s_dq2); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_cad(23)));
            if s_we2 = '1' and s_wpar2 = '0' then lbU0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wu2(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_cad(23)));
            if s_we2 = '1' and s_wpar2 = '1' then lbU1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wu2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_cad(23)));
            if s_we2 = '1' and s_wpar2 = '0' then lbV0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wv2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_cad(23)));
            if s_we2 = '1' and s_wpar2 = '1' then lbV1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wv2(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 10..29)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_tu, v_tv : signed(8 downto 0);
        variable v_nd : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 9 (rd) / 10: operand A (displaced, a line late) and B (still, dry ring)
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then s_wet_y10 <= unsigned(s_rdY1); else s_wet_y10 <= unsigned(s_rdY0); end if;
            s_by10 <= unsigned(s_ringY_q);
            -- 11: black-relative A, posterized B, contour flag (still picture)
            s_a11 <= f_brel(s_wet_y10);
            if s_sw_after = '0' then s_bq11 <= f_quant(s_by10, s_mask, s_half); else s_bq11 <= s_by10; end if;
            if s_sw_cont = '1' and (s_by10 and s_stepm) < resize(s_cw, 10) then s_cont(11) <= '1'; else s_cont(11) <= '0'; end if;
            for i in 12 to 26 loop s_cont(i) <= s_cont(i - 1); end loop;
            -- 12: black-relative B, inverted A
            s_b12 <= f_brel(s_bq11);
            s_a12 <= s_a11;
            s_na12 <= to_unsigned(C_W, 10) - s_a11;
            s_ad(12) <= s_a11;
            for i in 13 to 19 loop s_ad(i) <= s_ad(i - 1); end loop;
            -- 13: inverted B, sum, difference
            s_nb13 <= to_unsigned(C_W, 10) - s_b12;
            s_a13 <= s_a12; s_b13 <= s_b12; s_na13 <= s_na12;
            s_sum13 <= resize(s_a12, 11) + resize(s_b12, 11);
            s_dif13 <= signed(resize(s_a12, 11)) - signed(resize(s_b12, 11));
            -- 14: products (10 x 10)
            s_p14 <= s_a13 * s_b13;
            s_q14 <= s_na13 * s_nb13;
            s_sum14 <= s_sum13; s_dif14 <= s_dif13; s_a14 <= s_a13;
            -- 15-16: products / W  (1/1024 * (1 + 1/8 + 1/32 + 1/64) = 1/874)
            s_ps15 <= resize(s_p14(19 downto 10), 11) + resize(s_p14(19 downto 13), 11);
            s_pt15 <= resize(s_p14(19 downto 15), 11) + resize(s_p14(19 downto 16), 11);
            s_qs15 <= resize(s_q14(19 downto 10), 11) + resize(s_q14(19 downto 13), 11);
            s_qt15 <= resize(s_q14(19 downto 15), 11) + resize(s_q14(19 downto 16), 11);
            if s_dif14 < 0 then v_nd := -s_dif14; else v_nd := s_dif14; end if;
            s_cdif15 <= unsigned(v_nd(9 downto 0));
            if s_a14 < C_W / 2 then s_alo15 <= '1'; else s_alo15 <= '0'; end if;
            s_sum15 <= s_sum14;
            s_pn16 <= s_ps15 + s_pt15;
            s_qn16 <= s_qs15 + s_qt15;
            s_cdif16 <= s_cdif15; s_alo16 <= s_alo15; s_sum16 <= s_sum15;
            -- 17: candidates
            s_c_add17 <= signed(resize(s_sum16, 13));
            s_c_scr17 <= signed(resize(s_sum16, 13)) - signed(resize(s_pn16, 13));
            s_c_mul17 <= signed(resize(s_pn16, 13));
            s_c_dif17 <= signed(resize(s_cdif16, 13));
            if s_alo16 = '1' then
                s_c_ovl17 <= signed(shift_left(resize(s_pn16, 13), 1));
            else
                s_c_ovl17 <= to_signed(C_W, 13) - signed(shift_left(resize(s_qn16, 13), 1));
            end if;
            -- 18: select
            case to_integer(s_mode) is
                when 0 => s_sel18 <= s_c_add17;
                when 1 => s_sel18 <= s_c_scr17;
                when 2 => s_sel18 <= s_c_mul17;
                when 3 => s_sel18 <= s_c_dif17;
                when others => s_sel18 <= s_c_ovl17;
            end case;
            -- 19: clamp to 0 .. W
            if s_sel18 < 0 then s_t19 <= (others => '0');
            elsif s_sel18 > C_W then s_t19 <= to_unsigned(C_W, 10);
            else s_t19 <= unsigned(s_sel18(9 downto 0)); end if;
            -- 20: difference toward the blend, keyed mix amount, base
            s_dy20 <= signed(resize(s_t19, 11)) - signed(resize(s_ad(19), 11));
            if s_sw_key = '0' or s_keyd(19) = '1' then s_mx20 <= s_mix_e; else s_mx20 <= (others => '0'); end if;
            s_base20 <= s_ad(19) + 64;
            -- 21: mix product (11 x 9)
            s_py21 <= s_dy20 * signed('0' & s_mx20);
            s_base21 <= s_base20;
            -- 22: mixed luma on the displaced layer
            s_ys22 <= signed(resize(s_base21, 13)) + resize(shift_right(s_py21, 8), 13);
            -- 23: legal luma
            if s_ys22 < 64 then s_y23 <= to_unsigned(64, 10);
            elsif s_ys22 > 940 then s_y23 <= to_unsigned(940, 10);
            else s_y23 <= unsigned(s_ys22(9 downto 0)); end if;
            -- 24: posterize after the mix
            if s_sw_after = '1' then s_yq24 <= f_quant(s_y23, s_mask, s_half); else s_yq24 <= s_y23; end if;
            -- 25: contour ink, band tint (swapped on rows off the usual
            -- phase), chroma source (the replayed line swapped where its
            -- phase differs from this row's)
            if s_cont(24) = '1' then s_y25 <= to_unsigned(64, 10); else s_y25 <= s_yq24; end if;
            if s_yq24(8) = '1' then v_tu := signed('0' & s_tint); else v_tu := -signed('0' & s_tint); end if;
            if s_yq24(9) = '1' then v_tv := signed('0' & s_tint); else v_tv := -signed('0' & s_tint); end if;
            if s_tswap = '1' then s_tu25 <= v_tv; s_tv25 <= v_tu; else s_tu25 <= v_tu; s_tv25 <= v_tv; end if;
            if s_sw_bchroma = '1' then
                s_u25 <= unsigned(s_ringC_q(19 downto 10)); s_v25 <= unsigned(s_ringC_q(9 downto 0));
            elsif s_cbk(24) = '1' then
                if s_csw(24) = '1' then s_u25 <= unsigned(s_rdV1) & "00"; s_v25 <= unsigned(s_rdU1) & "00";
                else s_u25 <= unsigned(s_rdU1) & "00"; s_v25 <= unsigned(s_rdV1) & "00"; end if;
            else
                if s_csw(24) = '1' then s_u25 <= unsigned(s_rdV0) & "00"; s_v25 <= unsigned(s_rdU0) & "00";
                else s_u25 <= unsigned(s_rdU0) & "00"; s_v25 <= unsigned(s_rdV0) & "00"; end if;
            end if;
            -- 26: tint add, knee distance
            s_su26 <= signed(resize(s_u25, 12)) + resize(s_tu25, 12);
            s_sv26 <= signed(resize(s_v25, 12)) + resize(s_tv25, 12);
            s_yk26 <= signed(resize(s_y25, 11)) - 600;
            s_y26 <= s_y25;
            -- 27: highlight knee (940 -> 770), black contour ink, neutral
            -- outside avid so the smoothing's edge taps read black
            if s_avid_sr(25) = '0' then
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            else
                if s_yk26 > 0 then s_y27 <= to_unsigned(600, 10) + unsigned(s_yk26(9 downto 1));
                else s_y27 <= s_y26; end if;
                if s_cont(26) = '1' then
                    s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
                else
                    s_u27 <= f_clamp10(s_su26); s_v27 <= f_clamp10(s_sv26);
                end if;
            end if;
            -- 28-29: [1 2 1] on Y/U/V (HD HDMI bar guard; output centre = s_ya), gate
            s_ya <= s_y27; s_yb <= s_ya;
            s_ua <= s_u27; s_ub <= s_ua;
            s_va <= s_v27; s_vb <= s_va;
            if s_avid_sr(27) = '1' then
                s_y29 <= f_121(s_yb, s_ya, s_y27);
                s_u29 <= f_121(s_ub, s_ua, s_u27);
                s_v29 <= f_121(s_vb, s_va, s_v27);
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y29);
    s_io_align_in.u       <= std_logic_vector(s_u29);
    s_io_align_in.v       <= std_logic_vector(s_v29);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (29 -> 32).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to 2 loop
                s_io_align_pad(i) <= s_io_align_pad(i - 1);
            end loop;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(2);

end architecture stratamix;
