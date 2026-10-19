-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- grist.vhd
--
-- v0.3.2 (2026-10-17): resolution scaling.  The K3 block octave (8..64) is
-- structure, so it is now hashed on virtual 1080-line coordinates stepped
-- per pixel / per field line from the video timing ID (registers_in(8)):
-- 1.0 at 1080p (bit-identical), 1.5 at 720p, and at SD 2.0 across (a
-- 1440-wide 4:3 window) and 1080/486 per frame line, so blocks keep their
-- size relative to the screen height and stay square on a 4:3 screen (they
-- were 2x tall at SD, where field lines counted as rows).  The per-pixel
-- tooth, the 4 px mottle, the Soft 4x4 box and the pores are film grain --
-- a pixel-level texture -- and stay at pixel scale.  Interlace is now read
-- from the timing ID (the hardware ties field_n high, so the field_n
-- detector never fired): the pixel grain hashes frame rows at 480i / 576i /
-- 1080i and the 4 px mottle is square there instead of 2:1 tall.  Animate
-- still reseeds every second field.
--
-- GRIST -- the grain from MORSEL's cookie dough, promoted to the whole
-- instrument.  Nothing else survived the fork: no cells, no chips, no dough.
--
-- The engine is morsel's surface noise: a 16-bit xor-rotate-add hash of the
-- live pixel counters giving a per-pixel fine tooth, the same hash on
-- shifted coordinates giving block-scale mottle, and a sparse threshold test
-- giving pores.  It is a STATIC function of (x, y) -- a material texture,
-- not TV snow -- unless S7 folds a scrambled frame counter into the seeds,
-- which turns the same texture into living film grain (reseeded every
-- second field/frame, ~30/25 Hz, so it boils like film rather than snow).
--
-- The whole program is one comparator:
--
--     out = clamp( 512 + (Y - thr + N) * gain )
--
-- where N is the octave-mixed grain field and gain rides P12 (DEVELOP).
-- At gain = 1 the multiply is transparent and the program is additive film
-- grain; as gain rises the video's tones slam toward black and white with
-- the grain as dither, and the picture re-renders itself as a MEZZOTINT --
-- pure stipple density.  For density to TRACK the picture the dither has to
-- span the whole luma range, so DEVELOP also widens the grain: the octave
-- gains are renormalised per frame (gain' = gain * A / 16, A ramping from
-- 16 at 0% to 7040 / span at 100%) so the climax dither spans +-440 whatever
-- K1/K2 say.  At the climax K1/K2 stop meaning "amount" and become the
-- octave MIX -- fine spray vs clumped aquatint.
--
-- Panel:
--   K1 FINE     per-pixel tooth amount (climax: share of fine spray)
--   K2 COARSE   block-mottle amount (4 px mid octave + K3-sized octave)
--   K3 BLOCK    coarse octave cell size, 8/16/32/64 px at 1080p (follows
--               the screen height in every mode)
--   K4 RESPONSE 0 = uniform grain, up = film-stock mid-tone weighting
--               (shadows and highlights print clean)
--   K5 PITS     bipolar: left = dark pits, centre = none, right = bright
--               sparkle (density grows away from the centre)
--   K6 EXPOSURE comparator midpoint: darker or lighter print
--   P12 DEVELOP additive grain -> hard mezzotint (the climax)
--   S7 ANIMATE  frozen etching <-> living grain
--   S8 SOFT     pixel stipple <-> soft grain.  Soft box-filters the fine
--               and 4 px octaves over 4x4 px (four hash rows + horizontal
--               delay taps -- no line buffer, so it stays field-coherent
--               under interlace), bilinear-interpolates the K3 block octave,
--               and draws pits as blobs from a third box-filtered field.  The
--               climax prints organic ink-spatter instead of a pixel dither,
--               and the 4x4 box also nulls the hash's row-alternation
--               component (the woven 'checkerboard' of the pixel grain).
--   S9 MONO     B/W print vs the video's own chroma.  In colour, the upper
--               half of DEVELOP drains chroma from the dark dots and lowers
--               the white ceiling a little, so the climax is coloured
--               stipple on black rather than tinted mud
--   S10 ETCH    7-level posterize after the comparator (boiling dithered
--               contours between flat plates)
--   S11 INVERT  negative print
--
-- Pipeline (streaming, C_LATENCY = 20, identical in every mode and in
-- both S8 positions -- the pixel-grain values ride a delay line to meet the
-- soft path):
--   N0    coordinates: frame rows y..y+3, 4 px and block cells, block
--         fractions (block shifts registered BEFORE the hash)
--   N1    packs (+ the region fold on the four fine rows), block cell + 1
--   N2-3  nine hashes: 4 fine rows / 4 px / 4 block corners (mix1, mix2)
--   N4-7  soft: 4-row column sums, then 4-tap horizontal sums (0..240);
--         bilinear: corner deltas, x lerp, y delta
--   N8-9  soft: re-centre to the pixel nibble's std and clamp, blob pores;
--         bilinear: y lerp
--   N10   S8 mux -> octave values + pore flag
--   N11   octave gains (small multiplies by the renormalised gains)
--   N12   octave sum + pore push
--   N13   tonal-response multiply, weight from the luma delay line
--   N14   D = (Y - 512) + grain + exposure bias
--   N15   the develop multiply D * m (gain = m << e, normalised at vblank)
--   N16   barrel shift << e (registered on its own -- fused with the clamp
--         it was the v0.1 critical path)
--   N17   >> 4, clamp to the legal print range, offset; dark-dot flag
--   N18   etch plates; chroma drain on dark dots
--   N19   invert / offset, blanking gate, output
-- No BRAM, no ROM.  Every per-frame product runs on one serial shift-add
-- multiplier and one serial divider in the vblank sequencer.
--
-- Hardware contracts: output luma stays inside 64..940 (no sub-black, no
-- full-scale white); y/u/v are gated to 64/512/512 whenever the
-- latency-aligned avid is low; per-field work (frame counter, interlace
-- detection) is guarded by a saw-active flag because analog vsync serrates.
--
-- Interlace: the pixel-grain hash row is the FRAME row -- (line << 1) |
-- field, with the offset on field_n = '0' (bottom field carries the odd rows;
-- seeding the opposite way was ribbon's crawling-comb hardware bug).
-- Progressive modes use the line counter directly.  (v0.3.2: interlace is
-- read from the timing ID -- the hardware ties field_n high, so the old
-- field_n detector never fired and the 4 px mottle printed 2:1 tall at SD /
-- 1080i.  The K3 block octave uses virtual rows instead.)
--
-- Chroma is passthrough (or 512 in MONO), so the hardware U/V swap is
-- irrelevant here -- both channels are treated identically.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture grist of program_top is
    -- data_in -> data_out latency is 20 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    constant C_LATENCY : integer := 20;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    -- 16-bit xor-rotate-add hash, bit-identical to morsel's.  The rotate is
    -- load-bearing: without it (a xor b) depends only on x xor y and every
    -- cell on a diagonal shares a hash (see the vhdl-hash-pack trap).
    function f_pack(x, y : unsigned(11 downto 0);
                    seed : unsigned(15 downto 0)) return unsigned is
        variable a, b : unsigned(15 downto 0);
    begin
        a := x(9 downto 0) & y(5 downto 0);
        b := y(9 downto 0) & x(5 downto 0);
        return (a xor rotate_left(b, 5)) xor seed;
    end function;

    -- f_pack only sees x(9:0) / y(9:0), so on a 1920x1080 raster the fine
    -- octave repeated exactly at x+1024 and y+1024.  The high coordinate
    -- bits pick a per-region constant that is ADDED to the pack (an xor
    -- would only xor-translate the texture -- the pack is GF(2)-linear).
    -- Region 0 keeps morsel's exact texture.  Checked numerically: max
    -- cross-correlation between regions 0.012 (was 1.000).
    function f_region(r : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(r) is
            when 0      => return x"0000";
            when 1      => return x"9E37";
            when 2      => return x"3C6E";
            when 3      => return x"DAA5";
            when 4      => return x"78DC";
            when 5      => return x"1713";
            when 6      => return x"B54A";
            when others => return x"5381";
        end case;
    end function;

    function f_mix1(a : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := a;
        h := h xor rotate_left(h, 7);
        h := h + rotate_left(h, 3);
        return h;
    end function;

    function f_mix2(a : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := a;
        h := h xor rotate_left(h, 11);
        h := h + rotate_left(h, 5);
        return h;
    end function;

    ----------------------------------------------------------------------
    -- raster measurement
    ----------------------------------------------------------------------
    signal s_xcnt : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt : unsigned(10 downto 0) := (others => '0');
    signal s_avid_q      : std_logic := '0';
    signal s_prev_vsync  : std_logic := '1';
    signal s_vs_pulse    : std_logic := '0';
    signal s_saw_act     : std_logic := '0';
    signal s_fodd        : std_logic := '0';
    signal s_ilace       : std_logic := '0';
    signal s_aframe : unsigned(15 downto 0) := (others => '0');

    -- Resolution scaling: virtual 1080-line coordinates for the K3 block
    -- octave.  x is Q.1 (steps 2/3/4 = 1.0/1.5/2.0 per pixel), y Q.8 per
    -- FIELD line (already doubled for interlace).  At 1080p the integer
    -- parts equal s_xcnt / s_lcnt.
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    signal s_xs  : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xva : unsigned(12 downto 0) := (others => '0');
    signal s_yva : unsigned(19 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_anim, s_spark, s_soft, s_mono, s_etch, s_inv : std_logic := '0';
    signal s_seq : unsigned(5 downto 0) := (others => '0');

    signal s_gf, s_gc : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_msc  : integer range 3 to 6 := 4;         -- coarse block shift
    signal s_tone : unsigned(3 downto 0) := (others => '0');
    signal s_pthr : unsigned(3 downto 0) := (others => '0');
    signal s_sthr : unsigned(7 downto 0) := (others => '0');  -- soft pores
    signal s_bias : signed(9 downto 0) := (others => '0');
    signal s_p8   : unsigned(7 downto 0) := to_unsigned(75, 8);
    signal s_g10  : unsigned(10 downto 0) := to_unsigned(16, 11);
    signal s_m    : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_e    : integer range 0 to 5 := 0;

    -- dither renormalisation: span S = 4 gf + 12 gc, A_full = 7040 / S
    signal s_sum1 : unsigned(7 downto 0) := (others => '0');
    signal s_span : unsigned(7 downto 0) := (others => '0');
    signal s_rem  : unsigned(12 downto 0) := (others => '0');
    signal s_dvs  : unsigned(15 downto 0) := (others => '0');
    signal s_q    : unsigned(8 downto 0) := (others => '0');
    signal s_mula : unsigned(9 downto 0) := (others => '0');
    signal s_mulb : unsigned(7 downto 0) := (others => '0');
    signal s_acc  : unsigned(17 downto 0) := (others => '0');
    signal s_aeff : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_gfe, s_gce : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal s_pm   : unsigned(9 downto 0) := to_unsigned(144, 10);

    -- colour-mode climax: dark-dot chroma drain + white ceiling
    signal s_cd   : unsigned(7 downto 0) := (others => '0');
    signal s_cs   : integer range 0 to 4 := 0;
    signal s_hi   : signed(10 downto 0) := to_signed(428, 11);
    signal s_ehi  : unsigned(9 downto 0) := to_unsigned(876, 10);

    signal s_fs1, s_fs2 : unsigned(15 downto 0) := (others => '0');
    signal s_sa : unsigned(15 downto 0) := x"1234";
    signal s_sb : unsigned(15 downto 0) := x"77A1";
    signal s_sc : unsigned(15 downto 0) := x"C65D";

    ----------------------------------------------------------------------
    -- grain pipeline (N0..N10)
    ----------------------------------------------------------------------
    signal c0_x, c0_y0, c0_y1, c0_y2, c0_y3 : unsigned(11 downto 0) := (others => '0');
    signal c0_xb, c0_yb, c0_xc, c0_yc : unsigned(11 downto 0) := (others => '0');
    signal c0_fx, c0_fy : unsigned(3 downto 0) := (others => '0');

    signal c1_pa0, c1_pa1, c1_pa2, c1_pa3, c1_pb : unsigned(15 downto 0) := (others => '0');
    signal c1_p00, c1_p10, c1_p01, c1_p11 : unsigned(15 downto 0) := (others => '0');
    signal c2_a0, c2_a1, c2_a2, c2_a3, c2_b : unsigned(15 downto 0) := (others => '0');
    signal c2_c00, c2_c10, c2_c01, c2_c11 : unsigned(15 downto 0) := (others => '0');
    signal c3_a0, c3_a1, c3_a2, c3_a3, c3_b : unsigned(15 downto 0) := (others => '0');
    signal c3_c00, c3_c10, c3_c01, c3_c11 : unsigned(15 downto 0) := (others => '0');

    -- block fractions ride along to the lerps (index = stage)
    type t_f4 is array (1 to 8) of unsigned(3 downto 0);
    signal s_fx_sr, s_fy_sr : t_f4 := (others => (others => '0'));

    -- soft: column sums (lo / mid / pore nibbles) and their x taps
    signal c4_l01, c4_l23, c4_m01, c4_m23, c4_p01, c4_p23 : unsigned(4 downto 0) := (others => '0');
    signal c5_lc, c5_mc, c5_pc : unsigned(5 downto 0) := (others => '0');
    signal s_lc1, s_lc2, s_lc3 : unsigned(5 downto 0) := (others => '0');
    signal s_mc1, s_mc2, s_mc3 : unsigned(5 downto 0) := (others => '0');
    signal s_pc1, s_pc2, s_pc3 : unsigned(5 downto 0) := (others => '0');
    signal c6_l01, c6_l23, c6_m01, c6_m23, c6_p01, c6_p23 : unsigned(6 downto 0) := (others => '0');
    signal c7_ls, c7_ms, c7_ps : unsigned(7 downto 0) := (others => '0');
    signal c8_ln, c8_mn : signed(4 downto 0) := (others => '0');
    signal c8_sp : std_logic := '0';
    signal c9_ln, c9_mn : signed(4 downto 0) := (others => '0');
    signal c9_sp : std_logic := '0';

    -- bilinear block octave (corner values re-centred to -16..15)
    signal c4_ba, c4_bc, c4_dab, c4_dcd : signed(5 downto 0) := (others => '0');
    signal c5_ba, c5_bc : signed(5 downto 0) := (others => '0');
    signal c5_mt, c5_mb : signed(10 downto 0) := (others => '0');
    signal c6_top, c6_bot : signed(10 downto 0) := (others => '0');
    signal c7_top, c7_dv : signed(10 downto 0) := (others => '0');
    signal c8_top : signed(10 downto 0) := (others => '0');
    signal c8_mv  : signed(15 downto 0) := (others => '0');
    signal c9_val : signed(15 downto 0) := (others => '0');

    -- pixel-grain values (lo nibble, 4 px nibble, block 5 bits, pore
    -- nibble) delayed N4..N9 to meet the soft path
    type t_px is array (4 to 9) of std_logic_vector(16 downto 0);
    signal s_px_sr : t_px := (others => (others => '0'));

    signal c10_n0, c10_n1 : signed(4 downto 0) := (others => '0');
    signal c10_n2 : signed(5 downto 0) := (others => '0');
    signal c10_pore : std_logic := '0';

    ----------------------------------------------------------------------
    -- comparator pipeline (N11..N19)
    ----------------------------------------------------------------------
    signal c11_g0, c11_g1, c11_g2 : signed(9 downto 0) := (others => '0');
    signal c11_pore : std_logic := '0';
    signal c12_n  : signed(11 downto 0) := (others => '0');
    signal t1_w5 : unsigned(4 downto 0) := (others => '0');
    signal t2_w  : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal c13_m : signed(11 downto 0) := (others => '0');
    signal c14_d : signed(12 downto 0) := (others => '0');
    signal c15_p : signed(19 downto 0) := (others => '0');
    signal c16_v : signed(24 downto 0) := (others => '0');
    signal c17_e : unsigned(9 downto 0) := (others => '0');
    signal c17_dark : std_logic := '0';
    signal c18_vy : unsigned(9 downto 0) := (others => '0');
    signal c18_cu, c18_cv : signed(10 downto 0) := (others => '0');
    signal c19_y, c19_u, c19_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- video + sync delay
    ----------------------------------------------------------------------
    type t_sr is array (0 to 17) of std_logic_vector(9 downto 0);
    signal s_y_sr, s_u_sr, s_v_sr : t_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            if data_in.avid = '1' then
                s_saw_act <= '1';
            end if;

            -- per-mode virtual steps (the timing ID is static while a mode
            -- runs, so a free-running register is enough)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            -- interlace from the timing ID (480i, 576i, 1080i): field_n
            -- alternation never reaches a program on hardware
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => s_ilace <= '1';
                when others             => s_ilace <= '0';
            end case;

            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
                s_xva  <= s_xva + s_xs;
            elsif s_avid_q = '1' then
                s_xcnt <= (others => '0');
                s_xva  <= (others => '0');
                s_lcnt <= s_lcnt + 1;
                s_yva  <= s_yva + s_ys;
            end if;

            if s_vs_pulse = '1' then
                -- reset is idempotent: safe on every serration edge
                s_lcnt <= (others => '0');
                -- virtual row: the bottom field starts half a field step
                -- down (inert on hardware, which ties field_n high)
                if data_in.field_n = '0' then
                    s_yva <= resize(s_ys(11 downto 1), 20);
                else
                    s_yva <= (others => '0');
                end if;
                -- everything that counts or compares field-to-field runs
                -- once per field: analog vsync serrates (several edges)
                if s_saw_act = '1' then
                    s_saw_act <= '0';
                    s_fodd    <= not s_fodd;
                    -- grain reseeds every second field (~30/25 Hz), so
                    -- under interlace both fields of a frame print the same
                    -- texture.  (v0.3.2: on the field toggle alone -- the
                    -- hardware ties field_n high, so a field_n-keyed reseed
                    -- would fire every field.)
                    if s_fodd = '1' then
                        s_aframe <= s_aframe + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer
    -- (everything with a ladder in it lives here -- a vblank cone is timed
    -- at the pixel clock like everything else; one small op per state)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_p10 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_p12 <= unsigned(registers_in(7));
                s_anim  <= registers_in(6)(0);            -- S7
                s_soft  <= registers_in(6)(1);            -- S8
                s_mono  <= registers_in(6)(2);            -- S9
                s_etch  <= registers_in(6)(3);            -- S10
                s_inv   <= registers_in(6)(4);            -- S11
                s_seq <= to_unsigned(63, 6);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 63 =>
                        s_gf <= s_k1(9 downto 6);
                        s_gc <= s_k2(9 downto 6);
                        -- K3 BLOCK: coarse octave 8/16/32/64 px
                        if    s_k3 < 256 then s_msc <= 3;
                        elsif s_k3 < 512 then s_msc <= 4;
                        elsif s_k3 < 768 then s_msc <= 5;
                        else                  s_msc <= 6; end if;
                    when 62 =>
                        s_tone <= s_k4(9 downto 6);
                        -- K5 PITS, bipolar: |K5 - 512| >> 5 is a slice
                        -- (or its complement below the centre) -- no adder
                        s_spark <= s_k5(9);
                        if s_k5(9) = '1' then
                            s_pthr <= s_k5(8 downto 5);
                        else
                            s_pthr <= not s_k5(8 downto 5);
                        end if;
                        -- K6 EXPOSURE: +-256 around the 512 midpoint
                        s_bias <= resize(
                            shift_right(signed(resize(s_k6, 11)) - 512, 1), 10);
                    when 61 =>
                        -- DEADBAND: the fader ADC dithers an LSB or two per
                        -- frame, and at high gain the comparator would make
                        -- that visible as frame-rate shimmer (morsel lesson)
                        if s_p12(9 downto 2) > s_p8 + 1
                           or s_p12(9 downto 2) + 1 < s_p8 then
                            s_p8 <= s_p12(9 downto 2);
                        end if;
                    when 60 =>
                        -- P12 DEVELOP -> linear gain, slow first half then
                        -- steep: 16..144 over the bottom, 144..1160 over the
                        -- top (gain/16 = 1x .. ~72x)
                        v_p10 := shift_left(resize(s_p8, 11), 2);
                        if v_p10 < 512 then
                            s_g10 <= to_unsigned(16, 11)
                                     + resize(v_p10(10 downto 2), 11);
                        else
                            s_g10 <= to_unsigned(144, 11)
                                     + shift_left(v_p10 - 512, 1);
                        end if;
                    when 59 =>
                        -- normalise gain to m << e so the pixel path only
                        -- ever multiplies by the 6-bit mantissa
                        if    s_g10 < 64 then
                            s_e <= 0; s_m <= s_g10(5 downto 0);
                        elsif s_g10 < 128 then
                            s_e <= 1; s_m <= s_g10(6 downto 1);
                        elsif s_g10 < 256 then
                            s_e <= 2; s_m <= s_g10(7 downto 2);
                        elsif s_g10 < 512 then
                            s_e <= 3; s_m <= s_g10(8 downto 3);
                        elsif s_g10 < 1024 then
                            s_e <= 4; s_m <= s_g10(9 downto 4);
                        else
                            s_e <= 5; s_m <= s_g10(10 downto 5);
                        end if;
                        s_sum1 <= shift_left(resize(s_gf, 8), 2) + shift_left(resize(s_gc, 8), 2);
                    when 58 =>
                        -- grain span: |g0| <= 4 gf, |g1| <= 4 gc, |g2| <= 8 gc
                        s_span <= s_sum1 + shift_left(resize(s_gc, 8), 3);
                    when 57 =>
                        -- A_full (x16) = 440 * 16 / span, restoring divide
                        s_rem <= to_unsigned(7040, 13);
                        s_dvs <= s_span & x"00";
                        s_q   <= (others => '0');
                    when 48 to 56 =>
                        if resize(s_rem, 16) >= s_dvs then
                            s_rem <= s_rem - s_dvs(12 downto 0);
                            s_q   <= s_q(7 downto 0) & '1';
                        else
                            s_q   <= s_q(7 downto 0) & '0';
                        end if;
                        s_dvs <= shift_right(s_dvs, 1);
                    when 47 =>
                        -- tiny spans would need A > 511: cap it there
                        if s_span < 14 then
                            s_mula <= to_unsigned(495, 10);
                        else
                            s_mula <= resize(s_q, 10) - 16;
                        end if;
                        s_mulb <= s_p8;
                        s_acc  <= (others => '0');
                    when 39 to 46 | 33 to 36 | 28 to 31 =>
                        -- shared serial shift-add multiplier, MSB first
                        if s_mulb(7) = '1' then
                            s_acc <= shift_left(s_acc, 1) + resize(s_mula, 18);
                        else
                            s_acc <= shift_left(s_acc, 1);
                        end if;
                        s_mulb <= shift_left(s_mulb, 1);
                    when 38 =>
                        -- A_eff = 16 + (A_full - 16) * develop
                        s_aeff <= to_unsigned(16, 10) + s_acc(17 downto 8);
                    when 37 =>
                        s_mula <= s_aeff;
                        s_mulb <= s_gf & "0000";
                        s_acc  <= (others => '0');
                    when 32 =>
                        -- renormalised gains: span * A_eff / 16 <= 440, so
                        -- gf * A_eff >> 4 <= 110
                        s_gfe  <= s_acc(11 downto 4);
                        s_mulb <= s_gc & "0000";
                        s_acc  <= (others => '0');
                    when 27 =>
                        s_gce <= s_acc(11 downto 4);
                    when 26 =>
                        -- pore push 144 -> 576 with develop, so pits stay
                        -- pits once the dither has widened
                        s_pm <= to_unsigned(144, 10) + shift_left(resize(s_p8, 10), 1);
                    when 25 =>
                        s_pm <= s_pm - resize(s_p8(7 downto 2), 10);
                    when 24 =>
                        s_pm <= s_pm - resize(s_p8(7 downto 4), 10);
                    when 23 =>
                        -- colour climax over the upper half of DEVELOP
                        if s_mono = '1' or s_p8(7) = '0' then
                            s_cd <= (others => '0');
                        else
                            s_cd <= '0' & s_p8(6 downto 0);
                        end if;
                        if    s_p8 < 128 then s_cs <= 0;
                        elsif s_p8 < 160 then s_cs <= 1;
                        elsif s_p8 < 192 then s_cs <= 2;
                        elsif s_p8 < 224 then s_cs <= 3;
                        else                  s_cs <= 4; end if;
                    when 22 =>
                        -- white ceiling 940 -> 813 in colour at the climax
                        -- (full-scale white clips the dot's colour away)
                        s_hi  <= to_signed(428, 11) - signed(resize(s_cd, 11));
                        s_ehi <= to_unsigned(876, 10) - resize(s_cd, 10);
                    when 18 =>
                        -- soft pores: threshold on the box-summed pore
                        -- field (mean 120, std ~18.4) giving density
                        -- pthr/16, from normal quantiles
                        case to_integer(s_pthr) is
                            when 0  => s_sthr <= to_unsigned(0, 8);
                            when 1  => s_sthr <= to_unsigned(92, 8);
                            when 2  => s_sthr <= to_unsigned(99, 8);
                            when 3  => s_sthr <= to_unsigned(104, 8);
                            when 4  => s_sthr <= to_unsigned(108, 8);
                            when 5  => s_sthr <= to_unsigned(111, 8);
                            when 6  => s_sthr <= to_unsigned(114, 8);
                            when 7  => s_sthr <= to_unsigned(117, 8);
                            when 8  => s_sthr <= to_unsigned(120, 8);
                            when 9  => s_sthr <= to_unsigned(123, 8);
                            when 10 => s_sthr <= to_unsigned(126, 8);
                            when 11 => s_sthr <= to_unsigned(129, 8);
                            when 12 => s_sthr <= to_unsigned(132, 8);
                            when 13 => s_sthr <= to_unsigned(136, 8);
                            when 14 => s_sthr <= to_unsigned(141, 8);
                            when others => s_sthr <= to_unsigned(148, 8);
                        end case;
                    when 21 =>
                        s_fs1 <= f_mix1(s_aframe);
                    when 20 =>
                        s_fs2 <= f_mix2(s_fs1);
                    when 19 =>
                        if s_anim = '1' then
                            s_sa <= x"1234" xor s_fs2;
                            s_sb <= x"77A1" xor s_fs2;
                            s_sc <= x"C65D" xor s_fs2;
                        else
                            s_sa <= x"1234";
                            s_sb <= x"77A1";
                            s_sc <= x"C65D";
                        end if;
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- GRAIN FIELD (N0..N10): hash octaves, soft filters, pores, S8 mux
    ------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_y12 : unsigned(11 downto 0);
        variable v_xv, v_yv : unsigned(11 downto 0);
        variable v_fx, v_fy : unsigned(3 downto 0);
        variable v_t   : signed(8 downto 0);
        variable v_b   : signed(15 downto 0);
        variable v_px  : std_logic_vector(16 downto 0);
    begin
        if rising_edge(clk) then
            -- N0: coordinates.  Interlace: hash the FRAME row -- offset on
            -- field_n = '0' (bottom field carries the odd rows).  Rows
            -- y+1..y+3 are frame rows too, so the soft box is computed from
            -- the hash function itself and stays coherent across fields.
            if s_ilace = '1' then
                v_y12 := shift_left(resize(s_lcnt, 12), 1);
                if data_in.field_n = '0' then
                    v_y12(0) := '1';
                end if;
            else
                v_y12 := resize(s_lcnt, 12);
            end if;
            c0_x  <= s_xcnt;
            c0_y0 <= v_y12;
            c0_y1 <= v_y12 + 1;
            c0_y2 <= v_y12 + 2;
            c0_y3 <= v_y12 + 3;
            c0_xb <= shift_right(s_xcnt, 2);
            c0_yb <= shift_right(v_y12, 2);
            -- K3 block octave on VIRTUAL 1080-line coordinates (structure,
            -- not grain): at 1080p v_xv = s_xcnt and v_yv = the line count
            v_xv := s_xva(12 downto 1);
            v_yv := s_yva(19 downto 8);
            c0_xc <= shift_right(v_xv, s_msc);
            c0_yc <= shift_right(v_yv, s_msc);
            -- block fraction as a 4-bit weight (8 px cells: 3 bits << 1)
            case s_msc is
                when 3 =>
                    v_fx := v_xv(2 downto 0) & '0';
                    v_fy := v_yv(2 downto 0) & '0';
                when 4 =>
                    v_fx := v_xv(3 downto 0);
                    v_fy := v_yv(3 downto 0);
                when 5 =>
                    v_fx := v_xv(4 downto 1);
                    v_fy := v_yv(4 downto 1);
                when others =>
                    v_fx := v_xv(5 downto 2);
                    v_fy := v_yv(5 downto 2);
            end case;
            c0_fx <= v_fx;
            c0_fy <= v_fy;

            -- N1: packs.  The four fine rows take the region fold.
            c1_pa0 <= f_pack(c0_x, c0_y0, s_sa) + f_region(c0_x(11 downto 10) & c0_y0(10));
            c1_pa1 <= f_pack(c0_x, c0_y1, s_sa) + f_region(c0_x(11 downto 10) & c0_y1(10));
            c1_pa2 <= f_pack(c0_x, c0_y2, s_sa) + f_region(c0_x(11 downto 10) & c0_y2(10));
            c1_pa3 <= f_pack(c0_x, c0_y3, s_sa) + f_region(c0_x(11 downto 10) & c0_y3(10));
            c1_pb  <= f_pack(c0_xb, c0_yb, s_sb);
            c1_p00 <= f_pack(c0_xc,     c0_yc,     s_sc);
            c1_p10 <= f_pack(c0_xc + 1, c0_yc,     s_sc);
            c1_p01 <= f_pack(c0_xc,     c0_yc + 1, s_sc);
            c1_p11 <= f_pack(c0_xc + 1, c0_yc + 1, s_sc);

            -- N2-3: the nine hashes
            c2_a0 <= f_mix1(c1_pa0);  c3_a0 <= f_mix2(c2_a0);
            c2_a1 <= f_mix1(c1_pa1);  c3_a1 <= f_mix2(c2_a1);
            c2_a2 <= f_mix1(c1_pa2);  c3_a2 <= f_mix2(c2_a2);
            c2_a3 <= f_mix1(c1_pa3);  c3_a3 <= f_mix2(c2_a3);
            c2_b  <= f_mix1(c1_pb);   c3_b  <= f_mix2(c2_b);
            c2_c00 <= f_mix1(c1_p00); c3_c00 <= f_mix2(c2_c00);
            c2_c10 <= f_mix1(c1_p10); c3_c10 <= f_mix2(c2_c10);
            c2_c01 <= f_mix1(c1_p01); c3_c01 <= f_mix2(c2_c01);
            c2_c11 <= f_mix1(c1_p11); c3_c11 <= f_mix2(c2_c11);

            s_fx_sr(1) <= c0_fx;
            s_fy_sr(1) <= c0_fy;
            for i in 2 to 8 loop
                s_fx_sr(i) <= s_fx_sr(i - 1);
                s_fy_sr(i) <= s_fy_sr(i - 1);
            end loop;

            -- N4: soft column pairs; bilinear corner deltas; pixel values
            c4_l01 <= resize(c3_a0(3 downto 0), 5)   + resize(c3_a1(3 downto 0), 5);
            c4_l23 <= resize(c3_a2(3 downto 0), 5)   + resize(c3_a3(3 downto 0), 5);
            c4_m01 <= resize(c3_a0(7 downto 4), 5)   + resize(c3_a1(7 downto 4), 5);
            c4_m23 <= resize(c3_a2(7 downto 4), 5)   + resize(c3_a3(7 downto 4), 5);
            c4_p01 <= resize(c3_a0(15 downto 12), 5) + resize(c3_a1(15 downto 12), 5);
            c4_p23 <= resize(c3_a2(15 downto 12), 5) + resize(c3_a3(15 downto 12), 5);
            c4_ba  <= signed(resize(c3_c00(4 downto 0), 6)) - 16;
            c4_bc  <= signed(resize(c3_c01(4 downto 0), 6)) - 16;
            c4_dab <= signed(resize(c3_c10(4 downto 0), 6))
                      - signed(resize(c3_c00(4 downto 0), 6));
            c4_dcd <= signed(resize(c3_c11(4 downto 0), 6))
                      - signed(resize(c3_c01(4 downto 0), 6));
            v_px := std_logic_vector(c3_a0(3 downto 0)) & std_logic_vector(c3_b(7 downto 4))
                    & std_logic_vector(c3_c00(4 downto 0)) & std_logic_vector(c3_a0(15 downto 12));
            s_px_sr(4) <= v_px;
            for i in 5 to 9 loop
                s_px_sr(i) <= s_px_sr(i - 1);
            end loop;

            -- N5: column sums; x lerp products
            c5_lc <= resize(c4_l01, 6) + resize(c4_l23, 6);
            c5_mc <= resize(c4_m01, 6) + resize(c4_m23, 6);
            c5_pc <= resize(c4_p01, 6) + resize(c4_p23, 6);
            c5_mt <= resize(c4_dab * signed('0' & s_fx_sr(4)), 11);
            c5_mb <= resize(c4_dcd * signed('0' & s_fx_sr(4)), 11);
            c5_ba <= c4_ba;
            c5_bc <= c4_bc;

            -- N6: horizontal 4-tap pairs over the column-sum delay line;
            -- x lerps
            s_lc1 <= c5_lc;  s_lc2 <= s_lc1;  s_lc3 <= s_lc2;
            s_mc1 <= c5_mc;  s_mc2 <= s_mc1;  s_mc3 <= s_mc2;
            s_pc1 <= c5_pc;  s_pc2 <= s_pc1;  s_pc3 <= s_pc2;
            c6_l01 <= resize(c5_lc, 7) + resize(s_lc1, 7);
            c6_l23 <= resize(s_lc2, 7) + resize(s_lc3, 7);
            c6_m01 <= resize(c5_mc, 7) + resize(s_mc1, 7);
            c6_m23 <= resize(s_mc2, 7) + resize(s_mc3, 7);
            c6_p01 <= resize(c5_pc, 7) + resize(s_pc1, 7);
            c6_p23 <= resize(s_pc2, 7) + resize(s_pc3, 7);
            c6_top <= shift_left(resize(c5_ba, 11), 4) + c5_mt;
            c6_bot <= shift_left(resize(c5_bc, 11), 4) + c5_mb;

            -- N7: 4x4 box sums (0..240, mean 120, std ~18.4); y delta
            c7_ls <= resize(c6_l01, 8) + resize(c6_l23, 8);
            c7_ms <= resize(c6_m01, 8) + resize(c6_m23, 8);
            c7_ps <= resize(c6_p01, 8) + resize(c6_p23, 8);
            c7_top <= c6_top;
            c7_dv  <= c6_bot - c6_top;

            -- N8: re-centre to the pixel nibble's std (>> 2) and clamp to
            -- its -8..7 range so the climax renormalisation still holds;
            -- blob pores from the independent pore-nibble field; y lerp
            v_t := shift_right(signed(resize(c7_ls, 9)) - 120, 2);
            if    v_t < -8 then c8_ln <= to_signed(-8, 5);
            elsif v_t >  7 then c8_ln <= to_signed(7, 5);
            else                c8_ln <= resize(v_t, 5); end if;
            v_t := shift_right(signed(resize(c7_ms, 9)) - 120, 2);
            if    v_t < -8 then c8_mn <= to_signed(-8, 5);
            elsif v_t >  7 then c8_mn <= to_signed(7, 5);
            else                c8_mn <= resize(v_t, 5); end if;
            if c7_ps < s_sthr then c8_sp <= '1';
            else                   c8_sp <= '0'; end if;
            c8_top <= c7_top;
            c8_mv  <= resize(c7_dv * signed('0' & s_fy_sr(7)), 16);

            -- N9: y lerp (value x 256)
            c9_val <= shift_left(resize(c8_top, 16), 4) + c8_mv;
            c9_ln <= c8_ln;
            c9_mn <= c8_mn;
            c9_sp <= c8_sp;

            -- N10: the S8 mux.  The bilinear block value gets x1.5 back
            -- (interpolation costs it a third of its spread) and the
            -- block octave's -16..15 range.
            if s_soft = '1' then
                c10_n0 <= c9_ln;
                c10_n1 <= c9_mn;
                v_b := shift_right(c9_val + shift_right(c9_val, 1), 8);
                if    v_b < -16 then c10_n2 <= to_signed(-16, 6);
                elsif v_b >  15 then c10_n2 <= to_signed(15, 6);
                else                 c10_n2 <= resize(v_b, 6); end if;
                c10_pore <= c9_sp;
            else
                v_px := s_px_sr(9);
                c10_n0 <= signed(resize(unsigned(v_px(16 downto 13)), 5)) - 8;
                c10_n1 <= signed(resize(unsigned(v_px(12 downto 9)), 5)) - 8;
                c10_n2 <= signed(resize(unsigned(v_px(8 downto 4)), 6)) - 16;
                if unsigned(v_px(3 downto 0)) < s_pthr then c10_pore <= '1';
                else                                        c10_pore <= '0'; end if;
            end if;

            -- N11: octave gains (small multiplies)
            c11_g0 <= resize(shift_right(c10_n0 * signed('0' & s_gfe), 1), 10);
            c11_g1 <= resize(shift_right(c10_n1 * signed('0' & s_gce), 1), 10);
            c11_g2 <= resize(shift_right(c10_n2 * signed('0' & s_gce), 1), 10);
            c11_pore <= c10_pore;

            -- N12: octave sum (<= +-440) + pore push (<= 576)
            if c11_pore = '1' then
                if s_spark = '1' then
                    c12_n <= resize(c11_g0, 12) + resize(c11_g1, 12) + resize(c11_g2, 12)
                             + signed(resize(s_pm, 12));
                else
                    c12_n <= resize(c11_g0, 12) + resize(c11_g1, 12) + resize(c11_g2, 12)
                             - signed(resize(s_pm, 12));
                end if;
            else
                c12_n <= resize(c11_g0, 12) + resize(c11_g1, 12) + resize(c11_g2, 12);
            end if;
        end if;
    end process p_noise;

    ------------------------------------------------------------------------
    -- tonal-response weight, from the luma delay line (film stock puts its
    -- grain in the mid-tones; K4 fades between uniform and that)
    ------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_y  : unsigned(9 downto 0);
        variable v_mn : unsigned(9 downto 0);
        variable v_d  : signed(5 downto 0);
    begin
        if rising_edge(clk) then
            v_y := unsigned(s_y_sr(10));
            v_mn := v_y;
            if v_mn > (1023 - v_y) then v_mn := 1023 - v_y; end if;
            t1_w5 <= v_mn(9 downto 5);                     -- 0..15

            v_d := signed(resize(t1_w5, 6)) - 16;          -- -16..-1
            t2_w <= unsigned(resize(
                to_signed(16, 6)
                + shift_right(v_d * signed(resize(s_tone, 5)), 4),
                5));                                       -- 1..16
        end if;
    end process p_tone;

    ------------------------------------------------------------------------
    -- THE COMPARATOR (N13..N19): weight the grain, bias, develop, clamp
    ------------------------------------------------------------------------
    p_core : process(clk)
        variable v21 : signed(20 downto 0);
        variable vs  : signed(10 downto 0);
        variable vc  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- N13: grain through the response weight
            c13_m <= resize(shift_right(c12_n * signed('0' & t2_w), 4), 12);

            -- N14: distance from the print midpoint
            c14_d <= (signed(resize(unsigned(s_y_sr(13)), 13)) - 512)
                     + resize(c13_m, 13) + resize(s_bias, 13);

            -- N15: the develop multiply (mantissa only; << e comes next)
            c15_p <= resize(c14_d * signed('0' & s_m), 20);

            -- N16: barrel shift, on its own stage
            c16_v <= shift_left(resize(c15_p, 25), s_e);

            -- N17: unscale and clamp to the legal print range.  c17_e is
            -- luma - 64 (0..876); the in-range offset is a 10-bit add that
            -- runs beside the compares rather than after them.
            v21 := resize(shift_right(c16_v, 4), 21);
            if v21 < -448 then
                c17_e <= (others => '0');
            elsif v21 > s_hi then
                c17_e <= s_ehi;
            else
                c17_e <= unsigned(v21(9 downto 0)) + 448;
            end if;
            c17_dark <= c16_v(24);

            -- N18: etch plates (7 levels, 64..940); chroma drain on dark dots
            if s_etch = '1' then
                case to_integer(c17_e(9 downto 7)) is
                    when 0      => c18_vy <= to_unsigned(0, 10);
                    when 1      => c18_vy <= to_unsigned(146, 10);
                    when 2      => c18_vy <= to_unsigned(292, 10);
                    when 3      => c18_vy <= to_unsigned(438, 10);
                    when 4      => c18_vy <= to_unsigned(584, 10);
                    when 5      => c18_vy <= to_unsigned(730, 10);
                    when others => c18_vy <= to_unsigned(876, 10);
                end case;
            else
                c18_vy <= c17_e;
            end if;

            vs := signed(resize(unsigned(s_u_sr(17)), 11)) - 512;
            if s_mono = '1' or (c17_dark = '1' and s_cs = 4) then
                c18_cu <= (others => '0');
            elsif c17_dark = '1' then
                c18_cu <= shift_right(vs, s_cs);
            else
                c18_cu <= vs;
            end if;
            vs := signed(resize(unsigned(s_v_sr(17)), 11)) - 512;
            if s_mono = '1' or (c17_dark = '1' and s_cs = 4) then
                c18_cv <= (others => '0');
            elsif c17_dark = '1' then
                c18_cv <= shift_right(vs, s_cs);
            else
                c18_cv <= vs;
            end if;

            -- N19: invert / offset back to 64..940, blanking gate
            if s_avid_sr(C_LATENCY - 2) = '0' then
                c19_y <= to_unsigned(64, 10);
                c19_u <= to_unsigned(512, 10);
                c19_v <= to_unsigned(512, 10);
            else
                if s_inv = '1' then
                    c19_y <= to_unsigned(940, 10) - c18_vy;
                    vc := 512 - resize(c18_cu, 12);
                else
                    c19_y <= to_unsigned(64, 10) + c18_vy;
                    vc := 512 + resize(c18_cu, 12);
                end if;
                if vc > 1023 then c19_u <= to_unsigned(1023, 10);
                else              c19_u <= unsigned(vc(9 downto 0)); end if;
                if s_inv = '1' then
                    vc := 512 - resize(c18_cv, 12);
                else
                    vc := 512 + resize(c18_cv, 12);
                end if;
                if vc > 1023 then c19_v <= to_unsigned(1023, 10);
                else              c19_v <= unsigned(vc(9 downto 0)); end if;
            end if;
        end if;
    end process p_core;

    ------------------------------------------------------------------------
    -- video delay (chroma is symmetric here, so no U/V swap needed)
    ------------------------------------------------------------------------
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.u;
            s_v_sr(0) <= data_in.v;
            for i in 1 to 17 loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;
        end if;
    end process p_delay;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(c19_y);
    data_out.u       <= std_logic_vector(c19_u);
    data_out.v       <= std_logic_vector(c19_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture grist;
