-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Dropout: catalogue combo #50 (seed 20260920) -- chromatic glitch /
-- datamosh bars + head-switch noise band.  Three knobs each; the switches
-- and the slider decide how they combine.
--
--   K1 Bars     fraction of bands displaced (datamosh); 100% = every band
--   K2 Distance bar shift, up to +-half the line width; each displaced bar
--               moves 25..100% of it, hashed sign
--   K3 Hold     frames a displacement pattern is kept (1..64, P-frame repeat)
--   K4 Band     head-switch band height (8..256 lines)
--   K5 Noise    snow density (squared) / line-noise amplitude in the band
--   K6 Skew     head-switch flagging: rows in the band shift progressively
--
--   S7  Roll    band fixed at the bottom / rolling upward (wraps round)
--   S8  Dropout displaced bars lose their chroma
--   S9  Smear   displaced bars hold 16-px runs (blocky smear) / plain shift
--   S10 Burst   bands touching the head-switch band are always displaced
--   S11 Noise   snow (2-px grey speckles) / per-line noise lines
--   P12 Bands   band height 8 .. 128 lines, exponential (16 at 25%)
--
-- v0.2.0.  Line buffer read a line late; U/V swapped on rows whose
-- hsync->avid parity differs from the stored row's (drape v1.3).  Per-field
-- FSM: seed every Hold frames (one field parity under interlace), roll
-- 3/1024 of the height per field (H/32 + H/64 sixteenths of a line; ~5.7 s
-- a cycle at 60 Hz in every mode), head-switch band + burst zone placed
-- modulo the field height,
-- D = Distance * width / 512 on a serial shift-add.  Per-line FSM (one op
-- per state): band hash from a 256x16 ROM keyed by band xor seed, offset
-- = +-D*m/256 (m = 64..255), skew and line-noise products split hi/lo.
-- Output: luma clamp 64..940, highlight knee (600 + (y - 600)/2, whites
-- ~770) and [1 2 1] on Y/U/V (the HD HDMI bar fix).  Latency 13 clocks, all
-- modes, blanking-gated.
--
-- v0.2.2 (2026-10-17) resolution scaling: P12's band height, K4's
-- head-switch band and the burst margin are sizes on the 1080-line canvas,
-- converted to whole field lines from the timing ID (x 1080 / active lines,
-- halved on interlace: 480i bands are 0.225x as many field lines), so the
-- bars keep their height relative to the screen at 720p and SD.  Skew is a
-- slope in canvas units (x ys * kx px per field line), and the Smear runs
-- are 16 canvas units (16 px at 1080, ~10.7 at 720p, 8 at SD).  Distance
-- and Roll were already fractions of the screen; snow and line noise stay
-- at pixel scale.  Under interlace a new pattern starts only every other
-- field (a field toggle, since field_n is tied high on hardware).
-- 1080p unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture dropout of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (13 + 3 = 16 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT : integer := 13;

    -- per-band hash: 256 random words (Python random.Random(20261012))
    type t_hrom is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_HROM : t_hrom := (
        x"6818", x"0205", x"CA00", x"0CE9", x"005D", x"F897", x"0B0B", x"8FD7",
        x"D11D", x"C2B0", x"87F6", x"AB36", x"1191", x"F13E", x"38FA", x"64D1",
        x"0CDB", x"2BB9", x"3ADC", x"8E69", x"E37D", x"E194", x"8F4C", x"3BA5",
        x"131E", x"9B78", x"8BF0", x"E7BE", x"A7E9", x"50D6", x"6FFC", x"1D57",
        x"BC57", x"8A24", x"20B5", x"154B", x"E097", x"57C4", x"D9FC", x"46BF",
        x"9849", x"933D", x"B55D", x"A732", x"7370", x"9EA6", x"1003", x"4A86",
        x"B3D4", x"BB8B", x"1849", x"1682", x"6411", x"D489", x"752F", x"8C80",
        x"DE31", x"F3AD", x"5636", x"1261", x"4ADD", x"9CD3", x"08AD", x"0172",
        x"4AD9", x"BD12", x"2064", x"67C5", x"523B", x"91BC", x"8749", x"2755",
        x"691D", x"77A6", x"B529", x"E6A5", x"130A", x"4B3B", x"D4D3", x"806F",
        x"0A55", x"E236", x"668F", x"852E", x"F02C", x"415D", x"B1F2", x"F9D7",
        x"140B", x"39AC", x"510F", x"3182", x"D316", x"CA27", x"0F8F", x"1B3E",
        x"5CC2", x"6E6F", x"878D", x"0A73", x"B436", x"7A60", x"2F5E", x"57F6",
        x"DE53", x"E5A9", x"6983", x"2B21", x"B966", x"7BF1", x"7BA6", x"D4B6",
        x"30DF", x"9AB8", x"14C6", x"6263", x"163E", x"9432", x"8296", x"173D",
        x"0701", x"5233", x"DB91", x"2A1D", x"FB30", x"3B14", x"2429", x"43CB",
        x"7896", x"7423", x"4188", x"0616", x"F410", x"BF24", x"2709", x"38E9",
        x"D0CF", x"6CB5", x"07FF", x"7644", x"944D", x"033B", x"3B03", x"C0FA",
        x"BAD7", x"C7FA", x"1D6A", x"CC9C", x"0EFF", x"2A99", x"0421", x"22EE",
        x"F52F", x"32E9", x"C111", x"D1B4", x"C48F", x"683B", x"E69D", x"2E17",
        x"7F34", x"021C", x"EA76", x"32EE", x"9DDA", x"8CCA", x"97AF", x"4F7A",
        x"2C45", x"BC25", x"AA10", x"3A0D", x"9B45", x"B699", x"D19C", x"D640",
        x"8669", x"6D32", x"40A7", x"5E02", x"E292", x"F6FB", x"72DC", x"25FE",
        x"FA4D", x"8F9C", x"57FF", x"0052", x"18E1", x"3317", x"CAB7", x"040E",
        x"940D", x"356D", x"CC70", x"6FD0", x"AEC7", x"8B44", x"C55D", x"7344",
        x"D00C", x"293C", x"5315", x"9273", x"BF7D", x"68CB", x"E777", x"758B",
        x"E62F", x"7349", x"AC21", x"9433", x"DBA6", x"5127", x"6642", x"F5FE",
        x"096E", x"57FE", x"DC12", x"212C", x"AD8F", x"1942", x"E799", x"6EC1",
        x"2B95", x"D93A", x"3B79", x"D968", x"77D0", x"050C", x"BE70", x"FE5D",
        x"1DCC", x"808A", x"6695", x"155A", x"C17A", x"6689", x"31D2", x"1932",
        x"17B0", x"3F3A", x"9A09", x"0135", x"2B84", x"EDFC", x"1140", x"94E7",
        x"2116", x"D7ED", x"9A9F", x"754A", x"590F", x"9EE8", x"7E24", x"D3A5");

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    -- the pixel path only uses the bank that is not being written (the write
    -- bank's read data is discarded), so no used read hits the write address
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1 : signal is true;
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- controls (registered copies, then decoded)
    signal s_k_bars, s_k_dist, s_k_hold, s_k_band, s_k_noise, s_k_skew, s_k_pb : unsigned(9 downto 0) := (others => '0');
    signal s_sw_roll, s_sw_drop, s_sw_smear, s_sw_burst, s_sw_lines : std_logic;
    signal s_bars, s_dist, s_noise, s_skew : unsigned(7 downto 0) := (others => '0');
    signal s_bars_all : std_logic := '0';
    signal s_hold : unsigned(5 downto 0) := (others => '0');
    signal s_bhk : unsigned(8 downto 0) := to_unsigned(40, 9);    -- K4: 8..256 lines
    signal s_nbk : unsigned(6 downto 0) := to_unsigned(16, 7);    -- P12: 8..127 lines
    signal s_nsq_lo, s_nsq_hi : unsigned(11 downto 0) := (others => '0');
    signal s_pspk : unsigned(15 downto 0) := (others => '0');     -- snow density, Noise^2

    -- Resolution scaling (v0.2.2): Q10 canvas -> field-line factor, skew
    -- slope factor (Q8 ys * kx), burst margins, canvas x step (Q.1)
    signal s_ky : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_fsk : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_m31, s_m63 : unsigned(6 downto 0) := (others => '0');
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilace, s_ftog : std_logic := '0';
    signal s_nbp : unsigned(17 downto 0) := (others => '0');
    signal s_bhp : unsigned(19 downto 0) := (others => '0');
    signal s_nbl : unsigned(6 downto 0) := to_unsigned(16, 7);    -- P12 band, field lines
    signal s_bhl : unsigned(8 downto 0) := to_unsigned(40, 9);    -- K4 band, field lines
    signal s_skp2 : unsigned(17 downto 0) := (others => '0');
    signal s_skw : unsigned(9 downto 0) := (others => '0');       -- skew per field line
    signal s_vcur : unsigned(5 downto 0) := (others => '0');      -- smear run x, Q.1 mod 32 units
    signal s_vprev5 : std_logic := '0';
    signal s_rs : unsigned(10 downto 0) := (others => '0');       -- smear run start

    -- free-running noise
    signal s_rng : unsigned(31 downto 0) := x"2545F491";
    signal s_spk_tog, s_spk_hit : std_logic := '0';
    signal s_spk_pre, s_spk_val : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- per field
    signal s_vstep : unsigned(3 downto 0) := (others => '1');
    signal s_saw_active : std_logic := '0';
    signal s_fld_cur, s_fld_prev : std_logic := '1';
    signal s_hcount : unsigned(5 downto 0) := (others => '0');
    signal s_seed : unsigned(15 downto 0) := x"5A5A";
    signal s_hf, s_wf : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_nf, s_nfm1 : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_bhf, s_bhc : unsigned(8 downto 0) := to_unsigned(40, 9);
    signal s_dsh : unsigned(7 downto 0) := (others => '0');
    signal s_dbit : unsigned(2 downto 0) := (others => '0');
    signal s_dacc : unsigned(18 downto 0) := (others => '0');
    signal s_D : unsigned(9 downto 0) := (others => '0');
    signal s_rollacc : unsigned(14 downto 0) := (others => '0');  -- sixteenths of a line
    signal s_rstep : unsigned(6 downto 0) := (others => '0');
    signal s_rt : signed(16 downto 0) := (others => '0');
    signal s_t1 : signed(12 downto 0) := (others => '0');
    signal s_hs_top, s_near_a : unsigned(10 downto 0) := (others => '0');
    signal s_nlen : unsigned(9 downto 0) := (others => '0');

    -- input side, counters
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid, s_we : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_brow : unsigned(6 downto 0) := (others => '0');
    signal s_band : unsigned(7 downto 0) := (others => '0');
    signal s_wpar : std_logic := '0';
    signal s_rbank : std_logic := '1';
    signal s_hpar, s_line_par, s_cap_par, s_swap : std_logic := '0';

    -- per line
    signal s_lstep : unsigned(3 downto 0) := (others => '1');
    signal s_dhs, s_dnr : signed(11 downto 0) := (others => '0');
    signal s_bst : std_logic := '0';
    signal s_hidx : unsigned(7 downto 0) := (others => '0');
    signal s_hq, s_h : std_logic_vector(15 downto 0) := (others => '0');
    signal s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_inhs, s_near, s_moshed, s_sgn : std_logic := '0';
    signal s_hsrow : unsigned(7 downto 0) := (others => '0');
    signal s_m : unsigned(7 downto 0) := (others => '0');
    signal s_sp_lo, s_sp_hi : unsigned(11 downto 0) := (others => '0');
    signal s_sp_x : unsigned(9 downto 0) := (others => '0');
    signal s_sp : unsigned(17 downto 0) := (others => '0');
    signal s_skp : unsigned(14 downto 0) := (others => '0');
    signal s_lns : signed(8 downto 0) := (others => '0');
    signal s_ln_lo, s_ln_hi : signed(13 downto 0) := (others => '0');
    signal s_ln : signed(15 downto 0) := (others => '0');
    signal s_pm_lo, s_pm_hi : unsigned(13 downto 0) := (others => '0');
    signal s_mag : unsigned(17 downto 0) := (others => '0');
    signal s_moff : signed(11 downto 0) := (others => '0');
    signal s_lnoise : signed(9 downto 0) := (others => '0');
    signal s_vo : signed(16 downto 0) := (others => '0');
    signal s_offc : signed(11 downto 0) := (others => '0');
    -- row constants, committed in hblank
    signal s_off_row : signed(11 downto 0) := (others => '0');
    signal s_mosh_row, s_hs_row : std_logic := '0';
    signal s_lnoise_row : signed(9 downto 0) := (others => '0');

    -- pixel path
    signal s_x1, s_xh2 : unsigned(10 downto 0) := (others => '0');
    signal s_u3 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg, s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_wet_u, s_wet_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y8 : signed(11 downto 0) := to_signed(64, 12);
    signal s_u8, s_v8, s_u9, s_v9, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y9, s_y10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_yk10 : signed(10 downto 0) := (others => '0');
    signal s_y11, s_ya, s_yb, s_y13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_ua, s_ub, s_u13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_v11, s_va, s_vb, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

    -- [1 2 1] / 4 of three 10-bit taps
    function f_121(a, b, c : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(a, 12) + shift_left(resize(b, 12), 1) + resize(c, 12);
        return v(11 downto 2);
    end function;

begin

    s_sw_roll  <= registers_in(6)(0);
    s_sw_drop  <= registers_in(6)(1);
    s_sw_smear <= registers_in(6)(2);
    s_sw_burst <= registers_in(6)(3);
    s_sw_lines <= registers_in(6)(4);

    ---------------------------------------------------------------------------
    -- control decode + free-running noise
    ---------------------------------------------------------------------------
    p_dec : process(clk)
        variable v_r : unsigned(31 downto 0);
        variable v_p : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_k_bars  <= unsigned(registers_in(0));
            s_k_dist  <= unsigned(registers_in(1));
            s_k_hold  <= unsigned(registers_in(2));
            s_k_band  <= unsigned(registers_in(3));
            s_k_noise <= unsigned(registers_in(4));
            s_k_skew  <= unsigned(registers_in(5));
            s_k_pb    <= unsigned(registers_in(7));

            s_bars <= s_k_bars(9 downto 2);
            if s_k_bars(9 downto 2) = 255 then s_bars_all <= '1'; else s_bars_all <= '0'; end if;
            s_dist  <= s_k_dist(9 downto 2);
            s_hold  <= s_k_hold(9 downto 4);
            s_noise <= s_k_noise(9 downto 2);
            s_skew  <= s_k_skew(9 downto 2);
            -- Band 8 + 248 * K4 / 1023
            s_bhk <= to_unsigned(8, 9) + resize(s_k_band(9 downto 2), 9) - resize(s_k_band(9 downto 7), 9);
            -- Bands: (256 + frac) * 2^oct / 32 -> 8, 16, 32, 64 at 0/25/50/75%, 127 at 100%
            v_p := '1' & s_k_pb(7 downto 0);
            case to_integer(s_k_pb(9 downto 8)) is
                when 0 => s_nbk <= resize(v_p(8 downto 5), 7);
                when 1 => s_nbk <= resize(v_p(8 downto 4), 7);
                when 2 => s_nbk <= resize(v_p(8 downto 3), 7);
                when others => s_nbk <= v_p(8 downto 2);
            end case;
            -- Resolution scaling: per-mode factors (static while a mode runs)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 =>          -- 480i
                    s_ky <= to_unsigned(230, 11); s_fsk <= to_unsigned(569, 10); s_xs <= "100"; s_ilace <= '1';
                    s_m31 <= to_unsigned(6, 7);  s_m63 <= to_unsigned(13, 7);
                when 4 =>          -- 480p
                    s_ky <= to_unsigned(455, 11); s_fsk <= to_unsigned(288, 10); s_xs <= "100"; s_ilace <= '0';
                    s_m31 <= to_unsigned(13, 7); s_m63 <= to_unsigned(27, 7);
                when 8 =>          -- 576i
                    s_ky <= to_unsigned(273, 11); s_fsk <= to_unsigned(480, 10); s_xs <= "100"; s_ilace <= '1';
                    s_m31 <= to_unsigned(8, 7);  s_m63 <= to_unsigned(17, 7);
                when 12 =>         -- 576p
                    s_ky <= to_unsigned(546, 11); s_fsk <= to_unsigned(240, 10); s_xs <= "100"; s_ilace <= '0';
                    s_m31 <= to_unsigned(16, 7); s_m63 <= to_unsigned(33, 7);
                when 5 | 6 | 14 => -- 720p
                    s_ky <= to_unsigned(683, 11); s_fsk <= to_unsigned(256, 10); s_xs <= "011"; s_ilace <= '0';
                    s_m31 <= to_unsigned(20, 7); s_m63 <= to_unsigned(41, 7);
                when 1 | 2 | 10 => -- 1080i
                    s_ky <= to_unsigned(512, 11); s_fsk <= to_unsigned(512, 10); s_xs <= "010"; s_ilace <= '1';
                    s_m31 <= to_unsigned(15, 7); s_m63 <= to_unsigned(31, 7);
                when others =>     -- 1080p
                    s_ky <= to_unsigned(1024, 11); s_fsk <= to_unsigned(256, 10); s_xs <= "010"; s_ilace <= '0';
                    s_m31 <= to_unsigned(31, 7); s_m63 <= to_unsigned(63, 7);
            end case;
            -- band heights in field lines (>= 1), skew slope per field line
            s_nbp <= s_nbk * s_ky;
            s_bhp <= s_bhk * s_ky;
            if s_nbp(17 downto 10) + ("0000000" & s_nbp(9)) = 0 then s_nbl <= to_unsigned(1, 7);
            else s_nbl <= s_nbp(16 downto 10) + ("000000" & s_nbp(9)); end if;
            if s_bhp(19 downto 10) + ("000000000" & s_bhp(9)) = 0 then s_bhl <= to_unsigned(1, 9);
            else s_bhl <= s_bhp(18 downto 10) + ("00000000" & s_bhp(9)); end if;
            s_skp2 <= s_skew * s_fsk;
            s_skw <= s_skp2(17 downto 8);

            -- snow density = Noise^2 (operand split hi/lo)
            s_nsq_lo <= s_noise * s_noise(3 downto 0);
            s_nsq_hi <= s_noise * s_noise(7 downto 4);
            s_pspk <= shift_left(resize(s_nsq_hi, 16), 4) + resize(s_nsq_lo, 16);

            -- xorshift32: every bit refreshed every clock (the old 16-bit
            -- LFSR moved one bit a clock, so neighbours were correlated and
            -- the snow repeated every ~30 HD lines)
            v_r := s_rng;
            v_r := v_r xor shift_left(v_r, 13);
            v_r := v_r xor shift_right(v_r, 17);
            v_r := v_r xor shift_left(v_r, 5);
            s_rng <= v_r;

            -- 2-px snow speckles: grey 64..702, density Noise^2
            s_spk_pre <= to_unsigned(64, 10) + resize(s_rng(24 downto 16), 10) + resize(s_rng(31 downto 25), 10);
            s_spk_tog <= not s_spk_tog;
            if s_spk_tog = '1' then
                if s_rng(15 downto 0) < s_pspk then s_spk_hit <= '1'; else s_spk_hit <= '0'; end if;
                s_spk_val <= s_spk_pre;
            end if;
        end if;
    end process p_dec;

    ---------------------------------------------------------------------------
    -- counters, per-field and per-line sequencers
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_x1 <= s_rx - 1;

            -- chroma phase: hsync->avid parity of this line vs the stored line
            s_hpar <= not s_hpar;
            if data_in.avid = '1' and s_prev_avid = '0' then s_line_par <= s_hpar; end if;
            s_swap <= s_line_par xor s_cap_par;

            -- per field (one op per state)
            case to_integer(s_vstep) is
                when 0 =>
                    s_nf <= s_nbl; s_bhf <= s_bhl; s_hf <= s_height; s_wf <= s_line_width;
                    s_dsh <= s_dist;
                    -- new pattern every Hold frames; under interlace only on
                    -- the top field, so both fields of a frame share it
                    if s_hcount >= s_hold and (s_fld_cur = '1' or s_fld_cur = s_fld_prev) and (s_ilace = '0' or s_ftog = '1') then
                        s_hcount <= (others => '0'); s_seed <= s_rng(31 downto 16);
                    elsif s_hcount /= 63 then
                        s_hcount <= s_hcount + 1;
                    end if;
                    if s_sw_roll = '1' then s_rollacc <= s_rollacc + s_rstep; else s_rollacc <= (others => '0'); end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_nfm1 <= s_nf - 1;
                    s_rt <= signed(resize(s_rollacc, 17)) - signed(resize(s_hf & "0000", 17));
                    s_rstep <= resize(s_hf(10 downto 5), 7) + resize(s_hf(10 downto 6), 7);
                    if resize(s_bhf, 11) > s_hf then s_bhc <= s_hf(8 downto 0); else s_bhc <= s_bhf; end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    if s_rt >= 0 then s_rollacc <= unsigned(s_rt(14 downto 0)); end if;
                    s_t1 <= signed(resize(s_hf, 13)) - signed(resize(s_bhc, 13));
                    s_nlen <= resize(s_bhc, 10) + resize(s_nf, 10);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_t1 <= s_t1 - signed(resize(s_rollacc(14 downto 4), 13));
                    s_nlen <= s_nlen + resize(s_m63, 10);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    -- band top = H - band - roll, wrapped into [0, H)
                    v_t := s_t1 + signed(resize(s_hf, 13));
                    if s_t1 < 0 then s_hs_top <= unsigned(v_t(10 downto 0)); else s_hs_top <= unsigned(s_t1(10 downto 0)); end if;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_t1 <= signed(resize(s_hs_top, 13)) - signed(resize(s_nf, 13));
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    s_t1 <= s_t1 - signed(resize(s_m31, 13));
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    -- burst zone: band starts that overlap [top - 32, top + band + 32)
                    v_t := s_t1 + signed(resize(s_hf, 13));
                    if s_t1 < 0 then s_near_a <= unsigned(v_t(10 downto 0)); else s_near_a <= unsigned(s_t1(10 downto 0)); end if;
                    s_dacc <= (others => '0'); s_dbit <= "111";
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    -- D = Distance * width (MSB first, one shift-add a clock)
                    if s_dsh(7) = '1' then
                        s_dacc <= shift_left(s_dacc, 1) + resize(s_wf, 19);
                    else
                        s_dacc <= shift_left(s_dacc, 1);
                    end if;
                    s_dsh <= shift_left(s_dsh, 1);
                    s_dbit <= s_dbit - 1;
                    if s_dbit = 0 then s_vstep <= s_vstep + 1; end if;
                when 9 =>
                    s_D <= s_dacc(18 downto 9);   -- up to half the width
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            -- per line (one op per state); row values commit in state 10
            case to_integer(s_lstep) is
                when 0 =>
                    s_dhs <= signed(resize(s_line, 12)) - signed(resize(s_hs_top, 12));
                    s_dnr <= signed(resize(s_line, 12)) - signed(resize(s_near_a, 12));
                    if s_brow = 0 then s_bst <= '1'; else s_bst <= '0'; end if;
                    s_hidx <= s_band xor s_seed(7 downto 0);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    if s_dhs < 0 then s_dhs <= s_dhs + signed(resize(s_hf, 12)); end if;
                    if s_dnr < 0 then s_dnr <= s_dnr + signed(resize(s_hf, 12)); end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_dhs < signed(resize(s_bhc, 12)) then s_inhs <= '1'; else s_inhs <= '0'; end if;
                    if s_dnr < signed(resize(s_nlen, 12)) then s_near <= '1'; else s_near <= '0'; end if;
                    s_hsrow <= unsigned(s_dhs(7 downto 0));
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_h2 <= unsigned(s_h) xor s_seed;
                    s_sp_lo <= s_hsrow * s_skw(3 downto 0);
                    s_sp_hi <= s_hsrow * s_skw(7 downto 4);
                    s_sp_x  <= s_hsrow * s_skw(9 downto 8);
                    s_lns <= signed(resize(s_rng(23 downto 16), 9)) - 128;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_bst = '1' then
                        if s_bars_all = '1' or s_h2(15 downto 8) < s_bars or (s_sw_burst = '1' and s_near = '1') then
                            s_moshed <= '1';
                        else
                            s_moshed <= '0';
                        end if;
                        -- shift fraction m/256: 25..50% twice as often as 50..100%
                        if s_h2(7 downto 6) = "00" then s_m <= "01" & s_h2(5 downto 0); else s_m <= s_h2(7 downto 0); end if;
                        s_sgn <= s_h2(8);
                    end if;
                    s_sp <= shift_left(resize(s_sp_x, 18), 8) + shift_left(resize(s_sp_hi, 18), 4) + resize(s_sp_lo, 18);
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_pm_lo <= s_D * s_m(3 downto 0);
                    s_pm_hi <= s_D * s_m(7 downto 4);
                    s_ln_lo <= s_lns * signed('0' & s_noise(3 downto 0));
                    s_ln_hi <= s_lns * signed('0' & s_noise(7 downto 4));
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_mag <= shift_left(resize(s_pm_hi, 18), 4) + resize(s_pm_lo, 18);
                    s_ln <= shift_left(resize(s_ln_hi, 16), 4) + resize(s_ln_lo, 16);
                    if s_inhs = '1' then s_skp <= s_sp(17 downto 3); else s_skp <= (others => '0'); end if;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    if s_moshed = '0' then
                        s_moff <= (others => '0');
                    elsif s_sgn = '1' then
                        s_moff <= -signed("00" & s_mag(17 downto 8));
                    else
                        s_moff <= signed("00" & s_mag(17 downto 8));
                    end if;
                    if s_sw_lines = '1' and s_inhs = '1' then s_lnoise <= s_ln(15 downto 6); else s_lnoise <= (others => '0'); end if;
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_vo <= resize(s_moff, 17) + signed(resize(s_skp, 17));
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    if s_vo > 2047 then
                        s_offc <= to_signed(2047, 12);
                    elsif s_vo < -2047 then
                        s_offc <= to_signed(-2047, 12);
                    else
                        s_offc <= s_vo(11 downto 0);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_off_row <= s_offc; s_mosh_row <= s_moshed; s_hs_row <= s_inhs;
                    s_lnoise_row <= s_lnoise;
                    s_lstep <= (others => '1');
                when others => null;
            end case;

            -- line end: count lines and band rows (the next line's values)
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                if s_brow >= s_nfm1 then
                    s_brow <= (others => '0'); s_band <= s_band + 1;
                else
                    s_brow <= s_brow + 1;
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_cap_par <= s_line_par;   -- parity of the line now in the read bank
                s_hpar <= '0';
            end if;
            -- resets are idempotent (serrated vsync); the sequencer start and
            -- field sample are guarded by saw-active
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_brow <= (others => '0'); s_band <= (others => '0');
                s_wpar <= '0'; s_rbank <= '1';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                    s_fld_prev <= s_fld_cur; s_fld_cur <= data_in.field_n;
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

    p_hrom : process(clk)
    begin
        if rising_edge(clk) then
            s_hq <= C_HROM(to_integer(s_hidx));
            s_h <= s_hq;
        end if;
    end process p_hrom;

    ---------------------------------------------------------------------------
    -- read address (stage n = n clocks after data_in)
    ---------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(12 downto 0);
        variable v_rs : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: smear hold: runs of 16 canvas units (a run starts where the
            -- canvas x of s_x1 crosses a multiple of 16; 16 px at 1080)
            if s_x1 = "11111111111" then
                s_vcur <= (others => '0');
            else
                s_vcur <= s_vcur + s_xs;
            end if;
            s_vprev5 <= s_vcur(5);
            if s_x1 = 0 or s_vcur(5) /= s_vprev5 then v_rs := s_x1; else v_rs := s_rs; end if;
            s_rs <= v_rs;
            if s_sw_smear = '1' and s_mosh_row = '1' then s_xh2 <= v_rs; else s_xh2 <= s_x1; end if;
            -- 4
            s_u3 <= signed(resize(s_xh2, 13)) + resize(s_off_row, 13);
            -- 5
            v_w := signed(resize(s_line_width, 13));
            if s_u3 < 0 or s_u3 >= v_w then s_bg <= '1'; else s_bg <= '0'; end if;
            s_addr <= unsigned(s_u3(10 downto 0));
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- pixel path: 6 RAM data, 7 fetch, 8 noise / dropout / U-V swap,
    -- 9 clamp, 10-11 knee + neutral gate, 12-13 [1 2 1] + output gate
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 7
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;

            -- 8: band noise (row constants change only in hblank)
            v_y := signed(resize(s_wet_y, 12));
            if s_hs_row = '1' and s_sw_lines = '1' then
                s_y8 <= v_y + resize(s_lnoise_row, 12);
            elsif s_hs_row = '1' and s_spk_hit = '1' then
                s_y8 <= signed(resize(s_spk_val, 12));
            else
                s_y8 <= v_y;
            end if;
            if (s_hs_row = '1' and s_sw_lines = '0' and s_spk_hit = '1') or (s_sw_drop = '1' and s_mosh_row = '1') then
                s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
            elsif s_swap = '1' then
                s_u8 <= s_wet_v; s_v8 <= s_wet_u;
            else
                s_u8 <= s_wet_u; s_v8 <= s_wet_v;
            end if;

            -- 9: legal luma
            if s_y8 < 64 then
                s_y9 <= to_unsigned(64, 10);
            elsif s_y8 > 940 then
                s_y9 <= to_unsigned(940, 10);
            else
                s_y9 <= unsigned(s_y8(9 downto 0));
            end if;
            s_u9 <= s_u8; s_v9 <= s_v8;

            -- 10-11: highlight knee (940 -> 770), neutral outside avid so the
            -- smoothing's edge taps read black
            s_yk10 <= signed(resize(s_y9, 11)) - 600;
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            if s_avid_sr(9) = '0' then
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            else
                if s_yk10 > 0 then
                    s_y11 <= to_unsigned(600, 10) + unsigned(s_yk10(9 downto 1));
                else
                    s_y11 <= s_y10;
                end if;
                s_u11 <= s_u10; s_v11 <= s_v10;
            end if;

            -- 12-13: [1 2 1] on Y/U/V (output centre = s_ya), blanking gate
            s_ya <= s_y11; s_yb <= s_ya;
            s_ua <= s_u11; s_ub <= s_ua;
            s_va <= s_v11; s_vb <= s_va;
            if s_avid_sr(11) = '1' then
                s_y13 <= f_121(s_yb, s_ya, s_y11);
                s_u13 <= f_121(s_ub, s_ua, s_u11);
                s_v13 <= f_121(s_vb, s_va, s_v11);
            else
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y13);
    s_io_align_in.u       <= std_logic_vector(s_u13);
    s_io_align_in.v       <= std_logic_vector(s_v13);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (13 -> 16).
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

end architecture dropout;
