-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Vhsmix v0.3.0: catalogue combo #87 (seed 20260923) -- posterization +
-- head-switch noise band + multi-source mixing (add, screen, multiply,
-- difference, overlay).  Two knobs each; the switches and the slider decide
-- how they combine.
--
--   K1 Luma     posterize luma, 1..8 bits (black and white are always levels)
--   K2 Chroma   posterize chroma, 1..8 bits (centred: greys stay grey)
--   K3 Band     head-switch band height, 1..50% of the picture height
--   K4 Noise    noise in the band, 5..100% (100% = +-half the luma range)
--   K5 Blend    Add / Screen / Multiply / Difference / Overlay
--   K6 Mix      blend amount (the blend mixed onto the original)
--
--   S7  Ghost   off / the posterized layer is read 1/16 of the width
--               behind (a tape ghost trailing right), so every blend mixes
--               two different pictures (v0.1's Order did nothing in the
--               four symmetric blends)
--   S8  Band    normal / the band is the negative of the posterized layer
--   S9  Roll    band fixed at the bottom / rolling up, wrapping round
--               (~5.7 s a cycle in every mode)
--   S10 Noise   noise on luma / on chroma too (independent U and V noise
--               held 4 px: VHS colour streaks)
--   S11 Chroma  Posterize: the posterized chroma mixed on (K2 at 8 bits =
--               the original chroma) / Blend: chroma goes through the blend
--               mode too (signed U/V vector about neutral):
--                 Add        A + B: saturation x2, hard clip
--                 Screen     |A| + |B| - |A||B|/512: soft saturation boost
--                 Multiply   complex A * B / 256: hue angles add (without the
--                            ghost every hue doubles; greys stay grey)
--                 Difference (A - B) x 2: colour contours / colour emboss
--                 Overlay    follows the luma: Multiply colour where the
--                            posterized luma is dark, Screen where light
--   P12 Static  noise over the whole picture, squared (100% = snow, +-half
--               the luma range); the band is never quieter than the rest
--
-- Luma: normalized to n = 0..1023 (black 64 -> 0, white 940 -> 1023) at
-- the input; posterize = keep L bits and bit-replicate, so black and white
-- are always levels; the blends work on n, so multiply / screen / overlay
-- pivot on the real white (1023 - x is just not x, so no subtract feeds a
-- multiply); the output is 64 + n * 0.695 (white ~775).
-- Chroma: centred mid-tread quantizer (levels at 512 +- k * step, clamped
-- 64..960); the ghost ring carries it with the posterized luma.  The colour
-- products use the top 8 bits of each operand (one 8x8 multiply a stage).
-- Band: height = H * (1..50%) by a serial multiply per field, top placed
-- modulo the measured field height; per-line flags computed after hsync and
-- committed at the avid rise (the 22-clock pipeline tail runs past hsync on
-- SD front porches).  Noise: xorshift32, every bit new every clock.
--
-- HARDWARE CONTRACTS: blanking gate; one latency (22) in all modes;
-- output conditioning ([1 2 1] on Y/U/V + luma scaled to white ~775: the
-- HD HDMI bar fix from damier v0.4.4 -- posterized edges and full-swing
-- noise in motion are the bar trigger); labelled knobs honour the
-- firmware's label-index default until the knob first moves; no_rw_check
-- on the ghost ring (read address = write - offset, offset >= 2; 32-bit
-- words, 2 EBR).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vhsmix of program_top is
    -- IO-align pad: 2 register stages after the program's own output so the
    -- data_in -> data_out latency (22 + 2 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 22;

    -- keep the top z+1 bits of t and bit-replicate them (level k of N ->
    -- k * 1023 / (N - 1): black and white are always levels)
    function f_rep10(t : unsigned(9 downto 0); z : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case to_integer(z) is
            when 0 => r := (others => t(9));
            when 1 => r := t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8) & t(9 downto 8);
            when 2 => r := t(9 downto 7) & t(9 downto 7) & t(9 downto 7) & t(9);
            when 3 => r := t(9 downto 6) & t(9 downto 6) & t(9 downto 8);
            when 4 => r := t(9 downto 5) & t(9 downto 5);
            when 5 => r := t(9 downto 4) & t(9 downto 6);
            when 6 => r := t(9 downto 3) & t(9 downto 7);
            when others => r := t(9 downto 2) & t(9 downto 8);
        end case;
        return r;
    end function;

    -- centred chroma: keep the masked level, clamp to 64..960
    function f_cq(t : unsigned(10 downto 0); m : unsigned(9 downto 0)) return unsigned is
        variable q : unsigned(9 downto 0);
    begin
        q := t(9 downto 0) and m;
        if t(10) = '1' or q > 960 then
            q := to_unsigned(960, 10);
        elsif q < 64 then
            q := to_unsigned(64, 10);
        end if;
        return q;
    end function;

    -- [1 2 1] / 4 of three 10-bit taps
    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
    end function;

    -- K5: fifths of the travel (the firmware divides labels evenly)
    function f_zone5(v : unsigned(9 downto 0)) return unsigned is
    begin
        if v >= 820 then return to_unsigned(4, 3);
        elsif v >= 615 then return to_unsigned(3, 3);
        elsif v >= 410 then return to_unsigned(2, 3);
        elsif v >= 205 then return to_unsigned(1, 3);
        else return to_unsigned(0, 3);
        end if;
    end function;

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

    function f_clampc(v : signed) return unsigned is
    begin
        if v < 64 then
            return to_unsigned(64, 10);
        elsif v > 960 then
            return to_unsigned(960, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    -- controls
    signal s_k_lbits, s_k_cbits, s_k_band, s_k_noise, s_k_blend, s_k_mix, s_p_static : unsigned(9 downto 0) := (others => '0');
    signal s_sw_ghost, s_sw_bandneg, s_sw_roll, s_sw_cnoise, s_sw_cblend : std_logic := '0';
    signal s_zl, s_zc : unsigned(2 downto 0) := "010";
    signal s_mode : unsigned(2 downto 0) := "011";
    signal s_l_live, s_c_live, s_m_live : std_logic := '0';
    signal s_bm, s_bp : unsigned(9 downto 0) := (others => '0');
    signal s_zbm, s_zbp : unsigned(2 downto 0) := (others => '0');
    signal s_cstep : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_cmask : unsigned(9 downto 0) := "1110000000";
    signal s_chalf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_m1, s_m2, s_mk : unsigned(11 downto 0) := (others => '0');
    signal s_nb1 : unsigned(8 downto 0) := (others => '0');
    signal s_nb2 : unsigned(6 downto 0) := (others => '0');
    signal s_nb, s_ns, s_nmax : unsigned(8 downto 0) := (others => '0');
    signal s_psq : unsigned(15 downto 0) := (others => '0');
    signal s_mix9 : unsigned(8 downto 0) := (others => '0');

    -- counters, field / line sequencers
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_rx, s_line_width : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_line, s_height : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_hf : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_wf : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_vstep : unsigned(2 downto 0) := "111";
    signal s_mbit : unsigned(3 downto 0) := (others => '0');
    signal s_msh : unsigned(11 downto 0) := (others => '0');
    signal s_acc : unsigned(22 downto 0) := (others => '0');
    signal s_bandh : unsigned(10 downto 0) := to_unsigned(52, 11);
    signal s_rollacc : unsigned(14 downto 0) := (others => '0');  -- sixteenths of a line
    signal s_rstep : unsigned(6 downto 0) := (others => '0');
    signal s_rt : signed(16 downto 0) := (others => '0');
    signal s_t1 : signed(12 downto 0) := (others => '0');
    signal s_top : unsigned(10 downto 0) := to_unsigned(1028, 11);
    signal s_off : unsigned(6 downto 0) := to_unsigned(120, 7);
    signal s_off1 : unsigned(7 downto 0) := to_unsigned(119, 8);
    signal s_lstep : unsigned(1 downto 0) := "11";
    signal s_lidle : std_logic := '1';
    signal s_d : signed(11 downto 0) := (others => '0');
    signal s_inb : std_logic := '0';
    -- row values: pending (hblank) and committed (avid rise)
    signal s_neg_p, s_neg_row : std_logic := '0';
    signal s_namp_p, s_namp_row : unsigned(8 downto 0) := (others => '0');
    signal s_mix_p, s_mix_row : unsigned(8 downto 0) := to_unsigned(256, 9);

    -- noise
    signal s_rng : unsigned(31 downto 0) := x"2545F491";
    signal s_pny : unsigned(16 downto 0) := (others => '0');
    signal s_pnu, s_pnv : unsigned(15 downto 0) := (others => '0');
    signal s_nh1 : unsigned(8 downto 0) := (others => '0');
    signal s_ncnt : unsigned(1 downto 0) := (others => '0');
    signal s_ny : signed(10 downto 0) := (others => '0');
    signal s_nu, s_nv : signed(9 downto 0) := (others => '0');

    -- ghost ring: valid + posterized Y, U, V, read 'off' pixels behind
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal s_ring : t_ring;
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_ring : signal is true;
    signal s_wptr, s_rptr : unsigned(7 downto 0) := (others => '0');
    signal s_rd6 : std_logic_vector(31 downto 0) := (others => '0');

    -- pixel path, luma (stage n = n clocks after data_in)
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_a2 : unsigned(9 downto 0) := (others => '0');
    signal s_n13 : unsigned(10 downto 0) := (others => '0');
    signal s_n23 : unsigned(6 downto 0) := (others => '0');
    signal s_n4, s_n5, s_n6, s_n7 : unsigned(9 downto 0) := (others => '0');
    signal s_post5, s_post6, s_post7 : unsigned(9 downto 0) := (others => '0');
    signal s_gh7 : std_logic_vector(30 downto 0) := (others => '0');
    signal s_a8, s_b8, s_a9, s_b9, s_a10, s_b10 : unsigned(9 downto 0) := (others => '0');
    signal s_plo9, s_phi9, s_qlo9, s_qhi9 : unsigned(14 downto 0) := (others => '0');
    signal s_sum9 : unsigned(10 downto 0) := (others => '0');
    signal s_dif9 : signed(10 downto 0) := (others => '0');
    signal s_alo9, s_alo10 : std_logic := '0';
    signal s_p10, s_q10 : unsigned(19 downto 0) := (others => '0');
    signal s_sum10 : unsigned(10 downto 0) := (others => '0');
    signal s_ad10 : unsigned(9 downto 0) := (others => '0');
    signal s_cadd11, s_cscr11, s_cmul11, s_cdif11, s_covl11, s_cneg11 : unsigned(9 downto 0) := (others => '0');
    signal s_b11, s_t12, s_b12, s_b13, s_b14, s_b15 : unsigned(9 downto 0) := (others => '0');
    signal s_d13 : signed(10 downto 0) := (others => '0');
    signal s_dlo14 : signed(16 downto 0) := (others => '0');
    signal s_dhi14 : signed(15 downto 0) := (others => '0');
    signal s_pd15 : signed(20 downto 0) := (others => '0');
    signal s_n16, s_n18 : unsigned(9 downto 0) := (others => '0');
    signal s_y17 : signed(11 downto 0) := (others => '0');
    signal s_s119 : unsigned(9 downto 0) := (others => '0');
    signal s_s219 : unsigned(6 downto 0) := (others => '0');
    signal s_y20, s_ya, s_yb, s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- pixel path, chroma (A = posterized or its ghost, B = original; signed
    -- about 512 from stage 8)
    signal s_cu2, s_cv2 : unsigned(10 downto 0) := (others => '0');
    type t_cq is array (3 to 7) of unsigned(9 downto 0);
    signal s_uq, s_vq : t_cq := (others => to_unsigned(512, 10));
    type t_craw is array (2 to 15) of unsigned(9 downto 0);
    signal s_ur, s_vr : t_craw := (others => to_unsigned(512, 10));
    signal s_pu8, s_pv8, s_qu8, s_qv8, s_pu9, s_pv9, s_pu10, s_pv10 : signed(9 downto 0) := (others => '0');
    signal s_puqu9, s_pvqv9, s_puqv9, s_pvqu9 : signed(15 downto 0) := (others => '0');
    signal s_su9, s_sv9, s_du9, s_dv9 : signed(10 downto 0) := (others => '0');
    signal s_qun9, s_qvn9 : std_logic := '0';
    signal s_caddu10, s_caddv10 : signed(10 downto 0) := (others => '0');
    signal s_cscru10, s_cscrv10, s_cdifu10, s_cdifv10 : signed(11 downto 0) := (others => '0');
    signal s_cmulu10, s_cmulv10 : signed(12 downto 0) := (others => '0');
    signal s_tcu11, s_tcv11, s_tcu12, s_tcv12, s_dcu13, s_dcv13 : signed(12 downto 0) := (others => '0');
    signal s_culo14, s_cvlo14 : signed(18 downto 0) := (others => '0');
    signal s_cuhi14, s_cvhi14 : signed(17 downto 0) := (others => '0');
    signal s_cpu15, s_cpv15 : signed(22 downto 0) := (others => '0');
    signal s_cu16, s_cv16 : signed(12 downto 0) := (others => '0');
    signal s_u17, s_v17 : signed(12 downto 0) := (others => '0');
    signal s_u18, s_v18, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u20, s_ua, s_ub, s_u22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_v20, s_va, s_vb, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_sw_ghost   <= registers_in(6)(0);
    s_sw_bandneg <= registers_in(6)(1);
    s_sw_roll    <= registers_in(6)(2);
    s_sw_cnoise  <= registers_in(6)(3);
    s_sw_cblend  <= registers_in(6)(4);

    ---------------------------------------------------------------------------
    -- control decode, counters, sequencers, noise
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_z  : unsigned(2 downto 0);
        variable v_lo : unsigned(6 downto 0);
        variable v_k8 : unsigned(7 downto 0);
        variable v_r  : unsigned(31 downto 0);
        variable v_t  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_k_lbits  <= unsigned(registers_in(0));
            s_k_cbits  <= unsigned(registers_in(1));
            s_k_band   <= unsigned(registers_in(2));
            s_k_noise  <= unsigned(registers_in(3));
            s_k_blend  <= unsigned(registers_in(4));
            s_k_mix    <= unsigned(registers_in(5));
            s_p_static <= unsigned(registers_in(7));

            -- K1 / K2 bit depth: eighths; a new zone takes only 12 counts
            -- inside it (no flicker from pot noise at a boundary).  The
            -- firmware loads a labelled default as the label INDEX: honour
            -- it until the knob first reads a real position.
            v_z := s_k_lbits(9 downto 7); v_lo := s_k_lbits(6 downto 0);
            if s_k_lbits >= 8 then s_l_live <= '1'; end if;
            if s_l_live = '0' and s_k_lbits < 8 then
                s_zl <= s_k_lbits(2 downto 0);
            elsif (v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7) then
                s_zl <= v_z;
            end if;
            v_z := s_k_cbits(9 downto 7); v_lo := s_k_cbits(6 downto 0);
            if s_k_cbits >= 8 then s_c_live <= '1'; end if;
            if s_c_live = '0' and s_k_cbits < 8 then
                s_zc <= s_k_cbits(2 downto 0);
            elsif (v_lo >= 12 or v_z = 0) and (v_lo <= 115 or v_z = 7) then
                s_zc <= v_z;
            end if;
            -- chroma quantizer (step = 512 >> zone)
            s_cstep <= shift_right(to_unsigned(512, 11), to_integer(s_zc));
            s_cmask <= not (s_cstep(9 downto 0) - 1);
            s_chalf <= resize(shift_right(s_cstep, 1), 10);

            -- K5 blend: fifths, changes only when K5 +-10 agree
            if s_k_blend >= 10 then s_bm <= s_k_blend - 10; else s_bm <= (others => '0'); end if;
            if s_k_blend <= 1013 then s_bp <= s_k_blend + 10; else s_bp <= (others => '1'); end if;
            s_zbm <= f_zone5(s_bm); s_zbp <= f_zone5(s_bp);
            if s_k_blend >= 5 then s_m_live <= '1'; end if;
            if s_m_live = '0' and s_k_blend < 5 then
                s_mode <= s_k_blend(2 downto 0);
            elsif s_zbm = s_zbp then
                s_mode <= s_zbm;
            end if;

            -- K3 band fraction m / 4096 = 1 .. 50%: m = 41 + 2k - k/32 - k/128
            s_m1 <= to_unsigned(41, 12) + resize(s_k_band & '0', 12);
            s_m2 <= resize(s_k_band(9 downto 5), 12) + resize(s_k_band(9 downto 7), 12);
            s_mk <= s_m1 - s_m2;
            -- K4 band noise 24 + k * 1.875 (5 .. 100% of 502)
            v_k8 := s_k_noise(9 downto 2);
            s_nb1 <= to_unsigned(24, 9) + resize(v_k8, 9) + resize(v_k8(7 downto 1), 9);
            s_nb2 <= resize(v_k8(7 downto 2), 7) + resize(v_k8(7 downto 3), 7);
            s_nb <= s_nb1 + resize(s_nb2, 9);
            -- P12 static, squared: 0 .. 508
            s_psq <= s_p_static(9 downto 2) * s_p_static(9 downto 2);
            s_ns <= s_psq(15 downto 7);
            if s_nb > s_ns then s_nmax <= s_nb; else s_nmax <= s_ns; end if;
            -- K6 mix 0..256 (100% exact)
            s_mix9 <= resize(s_k_mix(9 downto 2), 9) + ("00000000" & s_k_mix(9));

            -- xorshift32: every bit new every clock
            v_r := s_rng;
            v_r := v_r xor shift_left(v_r, 13);
            v_r := v_r xor shift_right(v_r, 17);
            v_r := v_r xor shift_left(v_r, 5);
            s_rng <= v_r;
            -- noise: +-amp on luma, +-amp/2 on U and V (held 4 px)
            s_pny <= s_namp_row * s_rng(7 downto 0);
            s_pnu <= s_namp_row(8 downto 1) * s_rng(15 downto 8);
            s_pnv <= s_namp_row(8 downto 1) * s_rng(23 downto 16);
            s_nh1 <= s_namp_row;
            s_ny <= signed(resize(s_pny(16 downto 7), 11)) - signed(resize(s_nh1, 11));
            s_ncnt <= s_ncnt + 1;
            if s_ncnt = 0 then
                if s_sw_cnoise = '1' then
                    s_nu <= signed(resize(s_pnu(15 downto 7), 10)) - signed(resize(s_nh1(8 downto 1), 10));
                    s_nv <= signed(resize(s_pnv(15 downto 7), 10)) - signed(resize(s_nh1(8 downto 1), 10));
                else
                    s_nu <= (others => '0'); s_nv <= (others => '0');
                end if;
            end if;

            -- ghost ring pointers: read = write - off (registered one ahead)
            s_wptr <= s_wptr + 1;
            s_rptr <= s_wptr - s_off1;

            -- per field (one op per state)
            case to_integer(s_vstep) is
                when 0 =>
                    s_hf <= s_height; s_wf <= s_line_width; s_msh <= s_mk;
                    if s_sw_roll = '1' then s_rollacc <= s_rollacc + s_rstep; else s_rollacc <= (others => '0'); end if;
                    s_acc <= (others => '0'); s_mbit <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_rt <= signed(resize(s_rollacc, 17)) - signed(resize(s_hf & "0000", 17));
                    -- roll 3/1024 of the height a field (sixteenths of a line)
                    s_rstep <= resize(s_hf(10 downto 5), 7) + resize(s_hf(10 downto 6), 7);
                    -- ghost offset: 1/16 of the width
                    if s_wf(10 downto 5) = 0 then s_off <= to_unsigned(2, 7); else s_off <= s_wf(10 downto 4); end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- band = H * m / 4096, MSB first, one shift-add a clock
                    if s_msh(11) = '1' then
                        s_acc <= shift_left(s_acc, 1) + resize(s_hf, 23);
                    else
                        s_acc <= shift_left(s_acc, 1);
                    end if;
                    s_msh <= shift_left(s_msh, 1);
                    s_mbit <= s_mbit + 1;
                    if s_mbit = 11 then s_vstep <= s_vstep + 1; end if;
                when 3 =>
                    if s_rt >= 0 then s_rollacc <= unsigned(s_rt(14 downto 0)); end if;
                    if s_acc(22 downto 12) = 0 then s_bandh <= to_unsigned(1, 11); else s_bandh <= s_acc(22 downto 12); end if;
                    s_off1 <= resize(s_off, 8) - 1;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_t1 <= signed(resize(s_hf, 13)) - signed(resize(s_bandh, 13));
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_t1 <= s_t1 - signed(resize(s_rollacc(14 downto 4), 13));
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    -- band top = H - band - roll, wrapped into [0, H)
                    v_t := s_t1 + signed(resize(s_hf, 13));
                    if s_t1 < 0 then s_top <= unsigned(v_t(10 downto 0)); else s_top <= unsigned(s_t1(10 downto 0)); end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- per line, in hblank: (line - top) mod H < band
            if s_lidle = '0' then
                case to_integer(s_lstep) is
                    when 0 =>
                        s_d <= signed(resize(s_line, 12)) - signed(resize(s_top, 12));
                    when 1 =>
                        if s_d < 0 then s_d <= s_d + signed(resize(s_hf, 12)); end if;
                    when 2 =>
                        if s_d < signed(resize(s_bandh, 12)) then s_inb <= '1'; else s_inb <= '0'; end if;
                    when others =>
                        s_neg_p <= s_inb and s_sw_bandneg;
                        if s_inb = '1' then s_namp_p <= s_nmax; else s_namp_p <= s_ns; end if;
                        if s_inb = '1' and s_sw_bandneg = '1' then
                            s_mix_p <= to_unsigned(256, 9);
                        else
                            s_mix_p <= s_mix9;
                        end if;
                        s_lidle <= '1';
                end case;
                s_lstep <= s_lstep + 1;
            end if;

            -- line / field counting
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            -- row values commit at the avid rise (the previous line has left
            -- the pipeline; the new one has not reached any row consumer)
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_neg_row <= s_neg_p;
                s_namp_row <= s_namp_p; s_mix_row <= s_mix_p;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0'); s_lidle <= '0';
            end if;
            -- resets are idempotent (serrated vsync); the field sequencer
            -- start is guarded by saw-active
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
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

    ---------------------------------------------------------------------------
    -- ghost ring (write every clock; reads never hit the write address)
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring(to_integer(s_wptr)) <= '0' & s_avid_sr(4) & std_logic_vector(s_post5)
                                          & std_logic_vector(s_uq(5)) & std_logic_vector(s_vq(5));
            s_rd6 <= s_ring(to_integer(s_rptr));
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s : unsigned(10 downto 0);
        variable v_d : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);

            -- luma 2..4: n = (y - 64) * 1.168, 0..1023
            if s_in_y < 64 then s_a2 <= (others => '0'); else s_a2 <= s_in_y - 64; end if;
            s_n13 <= resize(s_a2, 11) + resize(shift_right(s_a2, 3), 11);
            s_n23 <= resize(shift_right(s_a2, 5), 7) + resize(shift_right(s_a2, 7), 7) + resize(shift_right(s_a2, 8), 7);
            v_s := s_n13 + resize(s_n23, 11);
            if v_s(10) = '1' then s_n4 <= (others => '1'); else s_n4 <= v_s(9 downto 0); end if;
            -- 5: posterize (ring write of s_post5 happens in p_ring)
            s_post5 <= f_rep10(s_n4, s_zl); s_n5 <= s_n4;
            -- 6: ring read data arrives (s_rd6); 7: fabric register
            s_post6 <= s_post5; s_n6 <= s_n5;
            s_gh7 <= s_rd6(30 downto 0); s_post7 <= s_post6; s_n7 <= s_n6;
            -- 8: blend operands a = posterized (or its ghost), b = original
            if s_sw_ghost = '1' and s_gh7(30) = '1' then s_a8 <= unsigned(s_gh7(29 downto 20)); else s_a8 <= s_post7; end if;
            s_b8 <= s_n7;
            -- 9: products split hi/lo; 1023 - x = not x
            s_plo9 <= s_a8 * s_b8(4 downto 0);
            s_phi9 <= s_a8 * s_b8(9 downto 5);
            s_qlo9 <= (not s_a8) * (not s_b8(4 downto 0));
            s_qhi9 <= (not s_a8) * (not s_b8(9 downto 5));
            s_sum9 <= resize(s_a8, 11) + resize(s_b8, 11);
            s_dif9 <= signed(resize(s_a8, 11)) - signed(resize(s_b8, 11));
            s_alo9 <= not s_a8(9);
            s_a9 <= s_a8; s_b9 <= s_b8;
            -- 10
            s_p10 <= shift_left(resize(s_phi9, 20), 5) + resize(s_plo9, 20);
            s_q10 <= shift_left(resize(s_qhi9, 20), 5) + resize(s_qlo9, 20);
            s_sum10 <= s_sum9;
            v_d := -s_dif9;   -- |a - b| reaches 1023: negate at full width, then slice
            if s_dif9 < 0 then s_ad10 <= unsigned(v_d(9 downto 0)); else s_ad10 <= unsigned(s_dif9(9 downto 0)); end if;
            s_alo10 <= s_alo9; s_a10 <= s_a9; s_b10 <= s_b9;
            -- 11: every blend
            if s_sum10(10) = '1' then s_cadd11 <= (others => '1'); else s_cadd11 <= s_sum10(9 downto 0); end if;
            s_cscr11 <= not s_q10(19 downto 10);
            s_cmul11 <= s_p10(19 downto 10);
            s_cdif11 <= s_ad10;
            if s_alo10 = '1' then s_covl11 <= s_p10(18 downto 9); else s_covl11 <= not s_q10(18 downto 9); end if;
            s_cneg11 <= not s_a10;
            s_b11 <= s_b10;
            -- 12: pick (the negative band row takes the inverted layer)
            if s_neg_row = '1' then
                s_t12 <= s_cneg11;
            else
                case to_integer(s_mode) is
                    when 0 => s_t12 <= s_cadd11;
                    when 1 => s_t12 <= s_cscr11;
                    when 2 => s_t12 <= s_cmul11;
                    when 3 => s_t12 <= s_cdif11;
                    when others => s_t12 <= s_covl11;
                end case;
            end if;
            s_b12 <= s_b11;
            -- 13..16: mix the blend onto the original (row mix: 256 in the negative band)
            s_d13 <= signed(resize(s_t12, 11)) - signed(resize(s_b12, 11));
            s_b13 <= s_b12;
            s_dlo14 <= s_d13 * signed('0' & s_mix_row(4 downto 0));
            s_dhi14 <= s_d13 * signed('0' & s_mix_row(8 downto 5));
            s_b14 <= s_b13;
            s_pd15 <= shift_left(resize(s_dhi14, 21), 5) + resize(s_dlo14, 21);
            s_b15 <= s_b14;
            s_n16 <= f_clamp10(signed(resize(s_b15, 13)) + resize(shift_right(s_pd15, 8), 13));
            -- 17, 18: noise, clamp
            s_y17 <= signed(resize(s_n16, 12)) + resize(s_ny, 12);
            s_n18 <= f_clamp10(s_y17);
            -- 19, 20: 64 + n * 0.695 (normalize back + bar-fix scale: white ~775);
            -- neutral outside avid so the smoothing's edge taps read black
            s_s119 <= shift_right(s_n18, 1) + shift_right(s_n18, 3);
            s_s219 <= resize(shift_right(s_n18, 4), 7) + resize(shift_right(s_n18, 7), 7);
            if s_avid_sr(18) = '1' then
                s_y20 <= to_unsigned(64, 10) + s_s119 + resize(s_s219, 10);
            else
                s_y20 <= to_unsigned(64, 10);
            end if;

            -- chroma 2..3: posterize; raw chroma carried for the mix
            s_cu2 <= resize(s_in_u, 11) + resize(s_chalf, 11);
            s_cv2 <= resize(s_in_v, 11) + resize(s_chalf, 11);
            s_ur(2) <= s_in_u; s_vr(2) <= s_in_v;
            for i in 3 to 15 loop s_ur(i) <= s_ur(i - 1); s_vr(i) <= s_vr(i - 1); end loop;
            s_uq(3) <= f_cq(s_cu2, s_cmask); s_vq(3) <= f_cq(s_cv2, s_cmask);
            for i in 4 to 7 loop s_uq(i) <= s_uq(i - 1); s_vq(i) <= s_vq(i - 1); end loop;
            -- 8: signed operands (code - 512 = flip the top bit); the chroma
            -- ghost only in Blend
            if s_sw_ghost = '1' and s_sw_cblend = '1' and s_gh7(30) = '1' then
                s_pu8 <= signed((not s_gh7(19)) & s_gh7(18 downto 10));
                s_pv8 <= signed((not s_gh7(9)) & s_gh7(8 downto 0));
            else
                s_pu8 <= signed((not s_uq(7)(9)) & s_uq(7)(8 downto 0));
                s_pv8 <= signed((not s_vq(7)(9)) & s_vq(7)(8 downto 0));
            end if;
            s_qu8 <= signed((not s_ur(7)(9)) & s_ur(7)(8 downto 0));
            s_qv8 <= signed((not s_vr(7)(9)) & s_vr(7)(8 downto 0));
            -- 9: colour products on the top 8 bits (units of 16), sums
            s_puqu9 <= s_pu8(9 downto 2) * s_qu8(9 downto 2);
            s_pvqv9 <= s_pv8(9 downto 2) * s_qv8(9 downto 2);
            s_puqv9 <= s_pu8(9 downto 2) * s_qv8(9 downto 2);
            s_pvqu9 <= s_pv8(9 downto 2) * s_qu8(9 downto 2);
            s_su9 <= resize(s_pu8, 11) + resize(s_qu8, 11);
            s_sv9 <= resize(s_pv8, 11) + resize(s_qv8, 11);
            s_du9 <= resize(s_pu8, 11) - resize(s_qu8, 11);
            s_dv9 <= resize(s_pv8, 11) - resize(s_qv8, 11);
            s_qun9 <= s_qu8(9); s_qvn9 <= s_qv8(9);
            s_pu9 <= s_pu8; s_pv9 <= s_pv8;
            -- 10: every colour blend
            s_caddu10 <= s_su9; s_caddv10 <= s_sv9;
            -- screen: A + B - A|B|/512 (|A|+|B|-|A||B|/512 when the signs agree)
            if s_qun9 = '1' then
                s_cscru10 <= resize(s_su9, 12) + resize(shift_right(s_puqu9, 5), 12);
            else
                s_cscru10 <= resize(s_su9, 12) - resize(shift_right(s_puqu9, 5), 12);
            end if;
            if s_qvn9 = '1' then
                s_cscrv10 <= resize(s_sv9, 12) + resize(shift_right(s_pvqv9, 5), 12);
            else
                s_cscrv10 <= resize(s_sv9, 12) - resize(shift_right(s_pvqv9, 5), 12);
            end if;
            -- multiply: complex A * B / 256
            s_cmulu10 <= resize(shift_right(resize(s_puqu9, 17) - resize(s_pvqv9, 17), 4), 13);
            s_cmulv10 <= resize(shift_right(resize(s_puqv9, 17) + resize(s_pvqu9, 17), 4), 13);
            s_cdifu10 <= shift_left(resize(s_du9, 12), 1);
            s_cdifv10 <= shift_left(resize(s_dv9, 12), 1);
            s_pu10 <= s_pu9; s_pv10 <= s_pv9;
            -- 11: pick (negative band: the inverted posterized layer)
            if s_neg_row = '1' then
                s_tcu11 <= -resize(s_pu10, 13); s_tcv11 <= -resize(s_pv10, 13);
            elsif s_sw_cblend = '0' then
                s_tcu11 <= resize(s_pu10, 13); s_tcv11 <= resize(s_pv10, 13);
            else
                case to_integer(s_mode) is
                    when 0 => s_tcu11 <= resize(s_caddu10, 13); s_tcv11 <= resize(s_caddv10, 13);
                    when 1 => s_tcu11 <= resize(s_cscru10, 13); s_tcv11 <= resize(s_cscrv10, 13);
                    when 2 => s_tcu11 <= s_cmulu10; s_tcv11 <= s_cmulv10;
                    when 3 => s_tcu11 <= resize(s_cdifu10, 13); s_tcv11 <= resize(s_cdifv10, 13);
                    when others =>
                        if s_alo10 = '1' then
                            s_tcu11 <= s_cmulu10; s_tcv11 <= s_cmulv10;
                        else
                            s_tcu11 <= resize(s_cscru10, 13); s_tcv11 <= resize(s_cscrv10, 13);
                        end if;
                end case;
            end if;
            -- 12: back to codes about 512
            s_tcu12 <= s_tcu11 + 512; s_tcv12 <= s_tcv11 + 512;
            -- 13..16: mix onto the original chroma (same row mix as luma)
            s_dcu13 <= s_tcu12 - signed(resize(s_ur(12), 13));
            s_dcv13 <= s_tcv12 - signed(resize(s_vr(12), 13));
            s_culo14 <= s_dcu13 * signed('0' & s_mix_row(4 downto 0));
            s_cuhi14 <= s_dcu13 * signed('0' & s_mix_row(8 downto 5));
            s_cvlo14 <= s_dcv13 * signed('0' & s_mix_row(4 downto 0));
            s_cvhi14 <= s_dcv13 * signed('0' & s_mix_row(8 downto 5));
            s_cpu15 <= shift_left(resize(s_cuhi14, 23), 5) + resize(s_culo14, 23);
            s_cpv15 <= shift_left(resize(s_cvhi14, 23), 5) + resize(s_cvlo14, 23);
            s_cu16 <= signed(resize(s_ur(15), 13)) + resize(shift_right(s_cpu15, 8), 13);
            s_cv16 <= signed(resize(s_vr(15), 13)) + resize(shift_right(s_cpv15, 8), 13);
            -- 17, 18: noise, legal chroma
            s_u17 <= s_cu16 + resize(s_nu, 13);
            s_v17 <= s_cv16 + resize(s_nv, 13);
            s_u18 <= f_clampc(s_u17); s_v18 <= f_clampc(s_v17);
            s_u19 <= s_u18; s_v19 <= s_v18;
            if s_avid_sr(18) = '1' then
                s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;

            -- 21, 22: [1 2 1] on Y/U/V (output centre = s_ya), blanking gate
            s_ya <= s_y20; s_yb <= s_ya;
            s_ua <= s_u20; s_ub <= s_ua;
            s_va <= s_v20; s_vb <= s_va;
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y22 <= f_121(s_yb, s_ya, s_y20);
                s_u22 <= f_121(s_ub, s_ua, s_u20);
                s_v22 <= f_121(s_vb, s_va, s_v20);
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

end architecture vhsmix;
