-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Impasto: catalogue combo #23 (seed 20260916) -- Perlin / simplex
-- displacement + oil paint / Kuwahara smoothing.  Three knobs each; the
-- switches and the slider decide how the two combine.
--
--   K1 Scale    noise cell size 4 .. 512 px (octave steps)
--   K2 Drift    noise field scrolls vertically
--   K3 Stroke   noise cells stretched horizontally x1 .. x8
--   K4 Brush    window 4 .. 64 px each side (exponential, 64 sizes); the
--               vertical brush drags about N/2 lines
--   K5 Edge     0 = plain box blur .. 100 = hard nearest-mean pick; also
--               where the vertical brush stops at an edge
--   K6 Relief   brush-stroke relief (difference of the two window means)
--
--   S7  Noise   warps the picture / steers which window is picked
--   S8  Field   smooth value noise / hard cells
--   S9  Pick    nearest mean / farthest mean (haloed oil)
--   S10 Relief  light from the left / from the right
--   S11 Knife   plain / luma posterized to 8 steps
--   P12 Amount  Warp: displacement amplitude, up to +-255 px.
--               Steer: share of the picture whose pick the noise flips,
--               none at 0 up to half at 100.
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Kuwahara-lite, in colour: running box sums of Y, U and V (EBR rings) give
-- the means of the N pixels left and right of a centre pixel taken C_DLY px
-- behind the input; 1/N is an 8-bit mantissa + shift.  The window whose luma
-- mean is nearer the centre wins, and the chroma takes that window too.
-- Chroma runs ONE channel at pixel rate: U on odd columns, V on even ones
-- (the line buffer keeps chroma per column pair anyway), signed (neutral 0).
-- A window that would reach past the C_EDGE inset is replaced by the other.
-- Vertical brush: a per-column IIR (state RAM, read-modify-write one line
-- apart) drags the brushed picture down by ~N/2 lines, with stochastic
-- rounding; a luma or chroma step bigger than the Edge threshold restarts it,
-- so it fills regions instead of smearing across their edges.  Relief and
-- Knife come after it.  The result goes through a delayed write port into ONE
-- line-buffer bank (the write trails the read; a same-address read takes the
-- written data) and is read back displaced by 2-D value noise.
-- Latency 21 clocks, blanking-gated.
--
-- v0.2.0: relief wrap, luma clamp 64..940, seamless noise rows, edge-aware
-- windows, continuous Brush, P12 live in Steer, steer aligned, no_rw_check.
-- v0.3.0: colour brush, vertical brush, single-bank line buffer, latency
-- 30 -> 21 (the old fetch delay was pure padding).
-- v0.3.2: resolution scaling.  The noise lattice runs on virtual 1080-line
-- coordinates (x step per pixel and y step per field line from the video
-- timing ID in registers_in(8)), the warp displacement is scaled by 1/x step
-- and the brush window / vertical drag by the x / y step (index offsets into
-- the exponential Brush table), so cells, warp and strokes keep their size
-- relative to the screen height at SD and 720p.  1080p is unchanged.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture impasto of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (21 + 3 = 24 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_LAT  : integer := 21;
    constant C_SEED : unsigned(15 downto 0) := x"91E3";
    -- Centre pixel delay (dring read-back).  The left window [C-N, C-1] is the
    -- running sum read C_DLY-1 back (lring); the right window [C+1, C+N] is
    -- read C_DLY-2-N back (rring), which must stay >= 1: C_DLY >= N + 3.
    -- Input-side stage k (stage 3 = ring reads land) holds the centre pixel
    -- p(t - (k-3) - C_DLY); its avid is s_ad(k) (carried in the dring).
    constant C_DLY  : integer := 67;
    -- the noise field is computed 13 px ahead of the input column, so the
    -- steer bit for a pixel is ready when that pixel enters the dring
    constant C_NLEAD : integer := 13;

    function f_clamp_y(v : signed) return unsigned is
    begin
        if v < 64 then
            return to_unsigned(64, 10);
        elsif v > 940 then
            return to_unsigned(940, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
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
    -- Brush table index offsets (the table is 16x over 63 steps, so 15.75
    -- steps an octave): horizontal by the x step, vertical by the y step
    -- (round(63/4 * log2(step))).
    function f_boff_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(16, 6);  -- x2.0
            when 5 | 6 | 14     => return to_unsigned(9, 6);   -- x1.5
            when others         => return to_unsigned(0, 6);
        end case;
    end function;
    function f_boff_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(34, 6);  -- 480i x4.44
            when 4          => return to_unsigned(18, 6);  -- 480p x2.25
            when 8          => return to_unsigned(30, 6);  -- 576i x3.75
            when 12         => return to_unsigned(14, 6);  -- 576p x1.875
            when 5 | 6 | 14 => return to_unsigned(9, 6);   -- 720p x1.5
            when 1 | 2 | 10 => return to_unsigned(16, 6);  -- 1080i x2.0
            when others     => return to_unsigned(0, 6);
        end case;
    end function;

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

    -- K4 Brush (top 6 bits) -> window N = round(4 * 16^(i/63)), 4..64;
    -- 1/N ~ rm / 2^(9 + rsh) (rm 128..255, floored so a mean never exceeds
    -- full scale); vertical IIR coefficient a = (2 + vb) / 2^(vs + 1) ~ 2/N.
    -- Packed N(21:15) & rm(14:7) & rsh(6:4) & vs(3:1) & vb(0).
    function f_brush(i : unsigned(5 downto 0)) return std_logic_vector is
    begin
        case to_integer(i) is
            when  0 => return "0000100100000000000010";   -- N  4, 1/N ~ 128/2^9, vertical a 0.5000 (~2.0 lines)
            when  1 => return "0000100100000000000010";   -- N  4, 1/N ~ 128/2^9, vertical a 0.5000 (~2.0 lines)
            when  2 => return "0000100100000000000010";   -- N  4, 1/N ~ 128/2^9, vertical a 0.5000 (~2.0 lines)
            when  3 => return "0000101110011000010101";   -- N  5, 1/N ~ 204/2^10, vertical a 0.3750 (~2.7 lines)
            when  4 => return "0000101110011000010101";   -- N  5, 1/N ~ 204/2^10, vertical a 0.3750 (~2.7 lines)
            when  5 => return "0000101110011000010101";   -- N  5, 1/N ~ 204/2^10, vertical a 0.3750 (~2.7 lines)
            when  6 => return "0000101110011000010101";   -- N  5, 1/N ~ 204/2^10, vertical a 0.3750 (~2.7 lines)
            when  7 => return "0000101110011000010101";   -- N  5, 1/N ~ 204/2^10, vertical a 0.3750 (~2.7 lines)
            when  8 => return "0000110101010100010101";   -- N  6, 1/N ~ 170/2^10, vertical a 0.3750 (~2.7 lines)
            when  9 => return "0000110101010100010101";   -- N  6, 1/N ~ 170/2^10, vertical a 0.3750 (~2.7 lines)
            when 10 => return "0000110101010100010101";   -- N  6, 1/N ~ 170/2^10, vertical a 0.3750 (~2.7 lines)
            when 11 => return "0000110101010100010101";   -- N  6, 1/N ~ 170/2^10, vertical a 0.3750 (~2.7 lines)
            when 12 => return "0000111100100100010100";   -- N  7, 1/N ~ 146/2^10, vertical a 0.2500 (~4.0 lines)
            when 13 => return "0000111100100100010100";   -- N  7, 1/N ~ 146/2^10, vertical a 0.2500 (~4.0 lines)
            when 14 => return "0000111100100100010100";   -- N  7, 1/N ~ 146/2^10, vertical a 0.2500 (~4.0 lines)
            when 15 => return "0001000100000000010100";   -- N  8, 1/N ~ 128/2^10, vertical a 0.2500 (~4.0 lines)
            when 16 => return "0001000100000000010100";   -- N  8, 1/N ~ 128/2^10, vertical a 0.2500 (~4.0 lines)
            when 17 => return "0001000100000000010100";   -- N  8, 1/N ~ 128/2^10, vertical a 0.2500 (~4.0 lines)
            when 18 => return "0001001111000110100100";   -- N  9, 1/N ~ 227/2^11, vertical a 0.2500 (~4.0 lines)
            when 19 => return "0001001111000110100100";   -- N  9, 1/N ~ 227/2^11, vertical a 0.2500 (~4.0 lines)
            when 20 => return "0001010110011000100111";   -- N 10, 1/N ~ 204/2^11, vertical a 0.1875 (~5.3 lines)
            when 21 => return "0001010110011000100111";   -- N 10, 1/N ~ 204/2^11, vertical a 0.1875 (~5.3 lines)
            when 22 => return "0001011101110100100111";   -- N 11, 1/N ~ 186/2^11, vertical a 0.1875 (~5.3 lines)
            when 23 => return "0001011101110100100111";   -- N 11, 1/N ~ 186/2^11, vertical a 0.1875 (~5.3 lines)
            when 24 => return "0001100101010100100111";   -- N 12, 1/N ~ 170/2^11, vertical a 0.1875 (~5.3 lines)
            when 25 => return "0001100101010100100111";   -- N 12, 1/N ~ 170/2^11, vertical a 0.1875 (~5.3 lines)
            when 26 => return "0001101100111010100111";   -- N 13, 1/N ~ 157/2^11, vertical a 0.1875 (~5.3 lines)
            when 27 => return "0001101100111010100111";   -- N 13, 1/N ~ 157/2^11, vertical a 0.1875 (~5.3 lines)
            when 28 => return "0001110100100100100110";   -- N 14, 1/N ~ 146/2^11, vertical a 0.1250 (~8.0 lines)
            when 29 => return "0001110100100100100110";   -- N 14, 1/N ~ 146/2^11, vertical a 0.1250 (~8.0 lines)
            when 30 => return "0001111100010000100110";   -- N 15, 1/N ~ 136/2^11, vertical a 0.1250 (~8.0 lines)
            when 31 => return "0010000100000000100110";   -- N 16, 1/N ~ 128/2^11, vertical a 0.1250 (~8.0 lines)
            when 32 => return "0010000100000000100110";   -- N 16, 1/N ~ 128/2^11, vertical a 0.1250 (~8.0 lines)
            when 33 => return "0010001111100000110110";   -- N 17, 1/N ~ 240/2^12, vertical a 0.1250 (~8.0 lines)
            when 34 => return "0010010111000110110110";   -- N 18, 1/N ~ 227/2^12, vertical a 0.1250 (~8.0 lines)
            when 35 => return "0010011110101110111001";   -- N 19, 1/N ~ 215/2^12, vertical a 0.0938 (~10.7 lines)
            when 36 => return "0010100110011000111001";   -- N 20, 1/N ~ 204/2^12, vertical a 0.0938 (~10.7 lines)
            when 37 => return "0010100110011000111001";   -- N 20, 1/N ~ 204/2^12, vertical a 0.0938 (~10.7 lines)
            when 38 => return "0010101110000110111001";   -- N 21, 1/N ~ 195/2^12, vertical a 0.0938 (~10.7 lines)
            when 39 => return "0010110101110100111001";   -- N 22, 1/N ~ 186/2^12, vertical a 0.0938 (~10.7 lines)
            when 40 => return "0010111101100100111001";   -- N 23, 1/N ~ 178/2^12, vertical a 0.0938 (~10.7 lines)
            when 41 => return "0011000101010100111001";   -- N 24, 1/N ~ 170/2^12, vertical a 0.0938 (~10.7 lines)
            when 42 => return "0011001101000110111001";   -- N 25, 1/N ~ 163/2^12, vertical a 0.0938 (~10.7 lines)
            when 43 => return "0011011100101110111000";   -- N 27, 1/N ~ 151/2^12, vertical a 0.0625 (~16.0 lines)
            when 44 => return "0011100100100100111000";   -- N 28, 1/N ~ 146/2^12, vertical a 0.0625 (~16.0 lines)
            when 45 => return "0011101100011010111000";   -- N 29, 1/N ~ 141/2^12, vertical a 0.0625 (~16.0 lines)
            when 46 => return "0011110100010000111000";   -- N 30, 1/N ~ 136/2^12, vertical a 0.0625 (~16.0 lines)
            when 47 => return "0100000100000000111000";   -- N 32, 1/N ~ 128/2^12, vertical a 0.0625 (~16.0 lines)
            when 48 => return "0100001111110001001000";   -- N 33, 1/N ~ 248/2^13, vertical a 0.0625 (~16.0 lines)
            when 49 => return "0100011111010101001000";   -- N 35, 1/N ~ 234/2^13, vertical a 0.0625 (~16.0 lines)
            when 50 => return "0100100111000111001000";   -- N 36, 1/N ~ 227/2^13, vertical a 0.0625 (~16.0 lines)
            when 51 => return "0100110110101111001011";   -- N 38, 1/N ~ 215/2^13, vertical a 0.0469 (~21.3 lines)
            when 52 => return "0100111110100101001011";   -- N 39, 1/N ~ 210/2^13, vertical a 0.0469 (~21.3 lines)
            when 53 => return "0101001110001111001011";   -- N 41, 1/N ~ 199/2^13, vertical a 0.0469 (~21.3 lines)
            when 54 => return "0101011101111101001011";   -- N 43, 1/N ~ 190/2^13, vertical a 0.0469 (~21.3 lines)
            when 55 => return "0101101101101101001011";   -- N 45, 1/N ~ 182/2^13, vertical a 0.0469 (~21.3 lines)
            when 56 => return "0101111101011101001011";   -- N 47, 1/N ~ 174/2^13, vertical a 0.0469 (~21.3 lines)
            when 57 => return "0110001101001111001011";   -- N 49, 1/N ~ 167/2^13, vertical a 0.0469 (~21.3 lines)
            when 58 => return "0110011101000001001011";   -- N 51, 1/N ~ 160/2^13, vertical a 0.0469 (~21.3 lines)
            when 59 => return "0110110100101111001010";   -- N 54, 1/N ~ 151/2^13, vertical a 0.0312 (~32.0 lines)
            when 60 => return "0111000100100101001010";   -- N 56, 1/N ~ 146/2^13, vertical a 0.0312 (~32.0 lines)
            when 61 => return "0111011100010101001010";   -- N 59, 1/N ~ 138/2^13, vertical a 0.0312 (~32.0 lines)
            when 62 => return "0111101100001101001010";   -- N 61, 1/N ~ 134/2^13, vertical a 0.0312 (~32.0 lines)
            when 63 => return "1000000100000001001010";   -- N 64, 1/N ~ 128/2^13, vertical a 0.0312 (~32.0 lines)
            when others => return "0000100100000000000010";
        end case;
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(15 downto 0);
    type t_ring16 is array (0 to 127) of std_logic_vector(15 downto 0);
    type t_ring32 is array (0 to 127) of std_logic_vector(31 downto 0);
    signal lbY : t_lb_y := (others => "0001000000");      -- one bank, see p_lbY
    type t_st_c is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lbC : t_lb_c := (others => x"8080");           -- U8 (odd col) & V8 (even col before), offset binary
    signal stY : t_lb_y := (others => "0001000000");      -- vertical brush state
    signal stC : t_st_c := (others => x"00");             -- U on odd columns, V on even, signed
    signal dring : t_ring16 := (others => x"0040");       -- steer & avid & Y, read C_DLY back
    signal bring : t_ring32 := (others => x"00000000");   -- Y10 U8 V8, read N-1 back (box sums)
    signal rring : t_ring32 := (others => x"00000000");   -- sums >> rsh, read C_DLY-2-N back (right)
    signal lring : t_ring32 := (others => x"00000000");   -- sums >> rsh, read C_DLY-1 back (left)
    -- Collision-free RAMs: the rings read >= 1 behind the write, the state RAMs
    -- read 6 columns ahead of the write.  The line buffer CAN collide (a read
    -- displaced ~66 px left lands on the column being written): p_pix
    -- substitutes the written data for that read.
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY : signal is true;
    attribute no_rw_check of lbC : signal is true;
    attribute no_rw_check of stY : signal is true;
    attribute no_rw_check of stC : signal is true;
    attribute no_rw_check of dring : signal is true;
    attribute no_rw_check of bring : signal is true;
    attribute no_rw_check of rring : signal is true;
    attribute no_rw_check of lring : signal is true;
    signal s_rdY : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdC : std_logic_vector(15 downto 0) := x"8080";
    signal s_stq_y : std_logic_vector(9 downto 0) := (others => '0');
    signal s_stq_c : std_logic_vector(7 downto 0) := x"00";
    signal s_dring_q : std_logic_vector(15 downto 0) := x"0040";
    signal s_bring_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_rring_q, s_lring_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ring_w : unsigned(6 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_scale, s_k_drift, s_k_stroke, s_k_brush, s_k_edge, s_k_relief : unsigned(9 downto 0);
    signal s_sw_steer, s_sw_blocky, s_sw_far, s_sw_right, s_sw_knife : std_logic;
    signal s_p_amount : unsigned(9 downto 0);
    signal s_k      : unsigned(3 downto 0) := to_unsigned(5, 4);       -- y cell shift 2..9
    signal s_kx     : unsigned(3 downto 0) := to_unsigned(5, 4);       -- x cell shift = k + stroke, max 10
    signal s_ymask  : unsigned(9 downto 0) := (others => '1');         -- rows per 4096-line wrap - 1
    signal s_amp    : unsigned(7 downto 0) := (others => '0');
    signal s_sthr   : unsigned(7 downto 0) := (others => '0');         -- Steer coverage threshold
    signal s_n      : unsigned(6 downto 0) := to_unsigned(16, 7);      -- brush window N, 4..64
    signal s_rm     : unsigned(7 downto 0) := to_unsigned(128, 8);     -- 1/N mantissa
    -- per-multiplier copies of the mantissa (one per channel and side)
    signal s_rmyl, s_rmyr, s_rmcl, s_rmcr : unsigned(7 downto 0) := to_unsigned(128, 8);
    attribute keep : boolean;
    attribute keep of s_rmyl : signal is true;
    attribute keep of s_rmyr : signal is true;
    attribute keep of s_rmcl : signal is true;
    attribute keep of s_rmcr : signal is true;
    signal s_rsh    : unsigned(2 downto 0) := to_unsigned(2, 3);       -- 1/N shift - 9
    signal s_vs     : unsigned(2 downto 0) := to_unsigned(3, 3);       -- vertical coefficient shift
    signal s_vb     : std_logic := '0';                                -- ... and its x1.5 bit
    signal s_vk     : unsigned(2 downto 0) := to_unsigned(4, 3);       -- vs + 1: 2..6
    signal s_vmask  : unsigned(5 downto 0) := to_unsigned(15, 6);      -- 2^vk - 1 (stochastic rounding)
    signal s_tinv   : unsigned(7 downto 0) := (others => '0');         -- 255 - Edge
    signal s_vt3    : unsigned(9 downto 0) := (others => '0');
    signal s_vt     : unsigned(9 downto 0) := to_unsigned(200, 10);    -- vertical restart step, luma
    signal s_vtc    : unsigned(7 downto 0) := to_unsigned(50, 8);      -- ... chroma (8-bit units)
    signal s_vtp, s_vtn : signed(11 downto 0) := (others => '0');      -- +-T
    signal s_vtcp, s_vtcn : signed(9 downto 0) := (others => '0');     -- +-Tc
    signal s_ndel, s_rdel : unsigned(6 downto 0) := (others => '0');
    signal s_lthr, s_rthr : unsigned(10 downto 0) := (others => '0');  -- edge-window column bounds
    signal s_edge   : unsigned(7 downto 0) := (others => '0');
    signal s_relief : unsigned(7 downto 0) := (others => '0');
    signal s_nphase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    -- resolution scaling (free-running from the timing ID)
    signal s_xs     : unsigned(2 downto 0) := to_unsigned(2, 3);       -- x step, Q.1
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);   -- y step per field line, Q.8
    signal s_xlead  : unsigned(11 downto 0) := to_unsigned(26, 12);    -- C_NLEAD px in Q.1 virtual units
    signal s_bih, s_biv : unsigned(5 downto 0) := (others => '0');     -- Brush table index, x / y
    signal s_ampr   : unsigned(7 downto 0) := (others => '0');         -- warp amplitude, virtual px
    signal s_amp_a, s_amp_b : unsigned(7 downto 0) := (others => '0');
    signal s_vxa    : unsigned(11 downto 0) := to_unsigned(26, 12);    -- noise column, Q.1 virtual
    signal s_lya    : unsigned(18 downto 0) := (others => '0');        -- field line, Q.8 virtual

    -- tracking / per-line
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_rxn    : unsigned(10 downto 0) := to_unsigned(C_NLEAD, 11); -- noise column (virtual) = s_rx + C_NLEAD
    signal s_x15    : unsigned(10 downto 0) := (others => '0');          -- s_rx, 15 clocks late
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_brun   : unsigned(11 downto 0) := (others => '0');       -- blank run (frame start detect)
    signal s_vfirst : std_logic := '1';                                -- first line: vertical brush restarts
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_ly     : unsigned(11 downto 0) := (others => '0');
    signal s_cy     : unsigned(9 downto 0) := (others => '0');
    signal s_cyn    : unsigned(9 downto 0) := (others => '0');
    signal s_fy     : unsigned(6 downto 0) := (others => '0');
    signal s_rk0, s_rk0p, s_rk1, s_rk1p : unsigned(15 downto 0) := (others => '0');
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";

    -- input side: box sums, windows, pick
    signal s_y1     : unsigned(9 downto 0) := (others => '0');
    signal s_u1, s_v1 : signed(7 downto 0) := (others => '0');
    signal s_acc    : unsigned(15 downto 0) := (others => '0');
    signal s_accu, s_accv : signed(13 downto 0) := (others => '0');
    signal s_ad     : std_logic_vector(4 to 16) := (others => '0');   -- avid of the stage-k centre
    signal s_st4, s_st5, s_st6 : std_logic := '0';                     -- steer bit of the stage-k centre
    signal s_par4   : std_logic := '1';                                -- stage-4 column odd: chroma = U
    signal s_col5   : unsigned(10 downto 0) := (others => '1');       -- column, stage 5 (flags at 6)
    signal s_col9   : unsigned(10 downto 0) := (others => '1');       -- column, stage 9 (state lands at 11)
    signal s_col15  : unsigned(10 downto 0) := (others => '1');       -- column, stage 15 (state write)
    signal s_syl4, s_syr4 : unsigned(11 downto 0) := (others => '0');
    signal s_scl4, s_scr4 : signed(9 downto 0) := (others => '0');
    signal s_cy4, s_cy5, s_cy6 : unsigned(9 downto 0) := (others => '0');
    signal s_pylh5, s_pyll5, s_pyrh5, s_pyrl5 : unsigned(15 downto 0) := (others => '0');
    signal s_pclh5, s_pcll5, s_pcrh5, s_pcrl5 : signed(14 downto 0) := (others => '0');
    signal s_wl6, s_wr6 : unsigned(9 downto 0) := (others => '0');
    signal s_cl6, s_cr6 : signed(7 downto 0) := (others => '0');
    signal s_lok6, s_rok6 : std_logic := '0';
    signal s_dl7, s_dr7 : unsigned(9 downto 0) := (others => '0');
    signal s_wl7, s_wr7 : unsigned(9 downto 0) := (others => '0');
    signal s_cl7, s_cr7 : signed(7 downto 0) := (others => '0');
    signal s_rl7    : signed(10 downto 0) := (others => '0');
    signal s_flip7  : std_logic := '0';
    signal s_pick8, s_avg8 : unsigned(9 downto 0) := (others => '0');
    signal s_pc8, s_ac8 : signed(7 downto 0) := (others => '0');
    signal s_pr8h, s_pr8l : signed(15 downto 0) := (others => '0');   -- relief product, hi / lo nibble of Relief
    signal s_e9     : signed(10 downto 0) := (others => '0');
    signal s_ec9    : signed(8 downto 0) := (others => '0');
    signal s_avg9   : unsigned(9 downto 0) := (others => '0');
    signal s_ac9    : signed(7 downto 0) := (others => '0');
    signal s_pe10   : signed(19 downto 0) := (others => '0');
    signal s_pec10  : signed(13 downto 0) := (others => '0');
    signal s_avg10  : unsigned(9 downto 0) := (others => '0');
    signal s_ac10   : signed(7 downto 0) := (others => '0');
    -- vertical brush (stages 11..15), then relief / clamp / knife (16..17)
    signal s_hy11   : unsigned(9 downto 0) := (others => '0');
    signal s_hc11   : signed(7 downto 0) := (others => '0');
    signal s_sy11   : unsigned(9 downto 0) := (others => '0');
    signal s_stc11  : signed(7 downto 0) := (others => '0');
    signal s_dy12, s_dy13 : signed(10 downto 0) := (others => '0');
    signal s_dc12, s_dc13 : signed(8 downto 0) := (others => '0');
    type t_sy is array (12 to 14) of unsigned(9 downto 0);
    type t_sc8 is array (12 to 14) of signed(7 downto 0);
    signal s_syd : t_sy := (others => (others => '0'));
    signal s_stcd : t_sc8 := (others => (others => '0'));
    signal s_rst13  : std_logic := '0';
    signal s_qy13   : signed(12 downto 0) := (others => '0');
    signal s_qc13   : signed(10 downto 0) := (others => '0');
    signal s_vy14   : signed(10 downto 0) := (others => '0');
    signal s_vcm14  : signed(8 downto 0) := (others => '0');
    signal s_ny15   : unsigned(9 downto 0) := (others => '0');
    signal s_nc15, s_nc16 : signed(7 downto 0) := (others => '0');
    type t_rf is array (9 to 15) of signed(11 downto 0);               -- relief, +-2038: 12 bits
    signal s_rf     : t_rf := (others => (others => '0'));
    signal s_t16    : signed(12 downto 0) := (others => '0');
    signal s_wy17   : unsigned(9 downto 0) := (others => '0');
    signal s_wc17, s_wcp17 : unsigned(7 downto 0) := x"80";              -- this column's chroma, the one before
    signal s_wcnt   : unsigned(10 downto 0) := (others => '0');
    signal s_we15   : std_logic := '0';
    signal s_we17   : std_logic := '0';
    signal s_steer  : std_logic := '0';

    -- noise pipeline (mirrors tremor)
    signal s_cx1    : unsigned(9 downto 0) := (others => '0');
    signal s_fx1    : unsigned(6 downto 0) := (others => '0');
    signal s_ka, s_kb, s_kc, s_kd : unsigned(15 downto 0) := (others => '0');
    signal s_ha3, s_hb3, s_hc3, s_hd3 : unsigned(15 downto 0) := (others => '0');
    signal s_ha4, s_hb4, s_hc4, s_hd4 : unsigned(15 downto 0) := (others => '0');
    signal s_ha5, s_hb5, s_hc5, s_hd5 : unsigned(15 downto 0) := (others => '0');
    signal s_a6, s_b6, s_c6, s_d6 : unsigned(7 downto 0) := (others => '0');
    signal s_a7, s_c7 : unsigned(7 downto 0) := (others => '0');
    signal s_dab7, s_dcd7 : signed(8 downto 0) := (others => '0');
    signal s_a8, s_c8 : unsigned(7 downto 0) := (others => '0');
    signal s_pab8, s_pcd8 : signed(16 downto 0) := (others => '0');
    signal s_top9, s_bot9 : unsigned(7 downto 0) := (others => '0');
    signal s_top10  : unsigned(7 downto 0) := (others => '0');
    signal s_dtb10  : signed(8 downto 0) := (others => '0');
    signal s_top11  : unsigned(7 downto 0) := (others => '0');
    signal s_ptb11  : signed(16 downto 0) := (others => '0');
    signal s_n12    : unsigned(7 downto 0) := (others => '0');
    signal s_nc13   : signed(8 downto 0) := (others => '0');
    signal s_d14    : signed(17 downto 0) := (others => '0');
    signal s_off15  : signed(9 downto 0) := (others => '0');
    type t_fx is array (2 to 7) of unsigned(6 downto 0);
    signal s_fxd    : t_fx := (others => (others => '0'));
    signal s_u16    : signed(12 downto 0) := (others => '0');
    signal s_addr17 : unsigned(10 downto 0) := (others => '0');
    -- fetch side: same-address bypass, then the gate
    signal s_byy, s_byc : std_logic := '0';
    signal s_byd_y  : unsigned(9 downto 0) := (others => '0');
    signal s_byd_u, s_byd_v : unsigned(7 downto 0) := x"80";
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
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

begin

    s_k_scale  <= unsigned(registers_in(0));
    s_k_drift  <= unsigned(registers_in(1));
    s_k_stroke <= unsigned(registers_in(2));
    s_k_brush  <= unsigned(registers_in(3));
    s_k_edge   <= unsigned(registers_in(4));
    s_k_relief <= unsigned(registers_in(5));
    s_sw_steer  <= registers_in(6)(0);
    s_sw_blocky <= registers_in(6)(1);
    s_sw_far    <= registers_in(6)(2);
    s_sw_right  <= registers_in(6)(3);
    s_sw_knife  <= registers_in(6)(4);
    s_p_amount <= unsigned(registers_in(7));
    -- virtual noise coordinates: integer parts of the accumulators
    s_rxn  <= s_vxa(11 downto 1);
    s_line <= s_lya(18 downto 8);

    --------------------------------------------------------------------------
    -- Rings.  After clock t: s_acc* = S(t-1), the N-sums ending at input pixel
    -- p(t-1); s_dring_q = p(t-C_DLY) with its avid and steer bit; s_lring_q =
    -- S(t-C_DLY-1); s_rring_q = S(t-C_DLY+N).  Carried the same number of
    -- stages, the centre C has its left window S(C-1) and right window S(C+N)
    -- beside it.  The sums go in pre-shifted by rsh (so the 1/N products are
    -- 12 x 8 for Y and 10 x 8 for U / V at every N):
    -- SY >> rsh (12) & SU >> rsh (10, signed) & SV >> rsh (10, signed).
    --------------------------------------------------------------------------
    p_rings : process(clk)
        variable v_w : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            dring(to_integer(s_ring_w)) <= "0000" & s_steer & data_in.avid & data_in.y;
            s_dring_q <= dring(to_integer(s_ring_w - C_DLY));
            bring(to_integer(s_ring_w)) <= "000000" & std_logic_vector(s_y1) & std_logic_vector(s_u1) & std_logic_vector(s_v1);
            s_bring_q <= bring(to_integer(s_ring_w - s_ndel));
            case to_integer(s_rsh) is
                when 0 =>      v_w := std_logic_vector(s_acc(11 downto 0)) & std_logic_vector(s_accu(9 downto 0)) & std_logic_vector(s_accv(9 downto 0));
                when 1 =>      v_w := std_logic_vector(s_acc(12 downto 1)) & std_logic_vector(s_accu(10 downto 1)) & std_logic_vector(s_accv(10 downto 1));
                when 2 =>      v_w := std_logic_vector(s_acc(13 downto 2)) & std_logic_vector(s_accu(11 downto 2)) & std_logic_vector(s_accv(11 downto 2));
                when 3 =>      v_w := std_logic_vector(s_acc(14 downto 3)) & std_logic_vector(s_accu(12 downto 3)) & std_logic_vector(s_accv(12 downto 3));
                when others => v_w := std_logic_vector(s_acc(15 downto 4)) & std_logic_vector(s_accu(13 downto 4)) & std_logic_vector(s_accv(13 downto 4));
            end case;
            rring(to_integer(s_ring_w)) <= v_w;
            s_rring_q <= rring(to_integer(s_ring_w - s_rdel));
            lring(to_integer(s_ring_w)) <= v_w;
            s_lring_q <= lring(to_integer(s_ring_w - (C_DLY - 1)));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : integer range 2 to 10;
        variable v_b : std_logic_vector(21 downto 0);
        variable v_dl, v_dr : unsigned(9 downto 0);
        variable v_m20 : unsigned(19 downto 0);
        variable v_m19 : signed(18 downto 0);
        variable v_y : unsigned(9 downto 0);
        variable v_near, v_sel, v_par : std_logic;
        variable v_e : signed(10 downto 0);
        variable v_ec : signed(7 downto 0);
        variable v_my : signed(12 downto 0);
        variable v_mc : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1; s_vxa <= s_vxa + s_xs; s_saw_active <= '1';
            else
                s_rx <= (others => '0'); s_vxa <= s_xlead;
            end if;
            -- resolution scaling: steps, the noise lead in virtual units, the
            -- Brush indices and the warp amplitude, all free-running
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            case to_integer(s_xs) is
                when 4      => s_xlead <= to_unsigned(4 * C_NLEAD, 12);
                when 3      => s_xlead <= to_unsigned(3 * C_NLEAD, 12);
                when others => s_xlead <= to_unsigned(2 * C_NLEAD, 12);
            end case;
            if s_k_brush(9 downto 4) > f_boff_x(registers_in(8)(3 downto 0)) then
                s_bih <= s_k_brush(9 downto 4) - f_boff_x(registers_in(8)(3 downto 0));
            else
                s_bih <= (others => '0');
            end if;
            if s_k_brush(9 downto 4) > f_boff_y(registers_in(8)(3 downto 0)) then
                s_biv <= s_k_brush(9 downto 4) - f_boff_y(registers_in(8)(3 downto 0));
            else
                s_biv <= (others => '0');
            end if;
            -- warp px = virtual px x 2 / x step (720p: 1/2+1/8+1/32+1/128)
            case to_integer(s_xs) is
                when 4 =>
                    s_amp_a <= shift_right(s_ampr, 1); s_amp_b <= (others => '0');
                when 3 =>
                    s_amp_a <= shift_right(s_ampr, 1) + shift_right(s_ampr, 3);
                    s_amp_b <= shift_right(s_ampr, 5) + shift_right(s_ampr, 7);
                when others =>
                    s_amp_a <= s_ampr; s_amp_b <= (others => '0');
            end case;
            s_amp <= s_amp_a + s_amp_b;
            if s_avid_sr(14) = '1' then s_x15 <= s_x15 + 1; else s_x15 <= (others => '0'); end if;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(14) xor s_lfsr(12) xor s_lfsr(3));

            -- stage 1: Y, signed U8 / V8, zero through blanking (a window reaching
            -- past a line end is never used -- see the edge flags at stage 6)
            if data_in.avid = '1' then
                s_y1 <= unsigned(data_in.y);
                s_u1 <= signed((not data_in.u(9)) & data_in.u(8 downto 2));
                s_v1 <= signed((not data_in.v(9)) & data_in.v(8 downto 2));
            else
                s_y1 <= (others => '0'); s_u1 <= (others => '0'); s_v1 <= (others => '0');
            end if;
            -- stage 2: running sums over N (window ending at the stage-1 pixel),
            -- re-zeroed through blanking while the ring refills with zeros
            if s_rx = 0 then
                s_acc <= (others => '0'); s_accu <= (others => '0'); s_accv <= (others => '0');
            else
                s_acc  <= s_acc + resize(s_y1, 16) - resize(unsigned(s_bring_q(25 downto 16)), 16);
                s_accu <= s_accu + resize(s_u1, 14) - resize(signed(s_bring_q(15 downto 8)), 14);
                s_accv <= s_accv + resize(s_v1, 14) - resize(signed(s_bring_q(7 downto 0)), 14);
            end if;
            -- stage 3: ring reads land.  stage 4: luma sums; the chroma channel of
            -- this column (U on odd columns, V on even)
            if s_dring_q(10) = '1' then v_par := not s_par4; else v_par := '1'; end if;
            s_par4 <= v_par;
            s_syl4 <= unsigned(s_lring_q(31 downto 20)); s_syr4 <= unsigned(s_rring_q(31 downto 20));
            if v_par = '1' then
                s_scl4 <= signed(s_lring_q(19 downto 10)); s_scr4 <= signed(s_rring_q(19 downto 10));
            else
                s_scl4 <= signed(s_lring_q(9 downto 0));   s_scr4 <= signed(s_rring_q(9 downto 0));
            end if;
            s_cy4 <= unsigned(s_dring_q(9 downto 0)); s_cy5 <= s_cy4; s_cy6 <= s_cy5;
            s_ad(4) <= s_dring_q(10);
            for i in 5 to 16 loop s_ad(i) <= s_ad(i - 1); end loop;
            s_st4 <= s_dring_q(11); s_st5 <= s_st4; s_st6 <= s_st5;
            -- stage 5: 1/N products, split on the mantissa nibbles
            s_pylh5 <= s_syl4 * s_rmyl(7 downto 4); s_pyll5 <= s_syl4 * s_rmyl(3 downto 0);
            s_pyrh5 <= s_syr4 * s_rmyr(7 downto 4); s_pyrl5 <= s_syr4 * s_rmyr(3 downto 0);
            s_pclh5 <= s_scl4 * signed('0' & s_rmcl(7 downto 4)); s_pcll5 <= s_scl4 * signed('0' & s_rmcl(3 downto 0));
            s_pcrh5 <= s_scr4 * signed('0' & s_rmcr(7 downto 4)); s_pcrl5 <= s_scr4 * signed('0' & s_rmcr(3 downto 0));
            -- stage 6: window means; edge flags (window inside the C_EDGE inset)
            v_m20 := shift_left(resize(s_pylh5, 20), 4) + resize(s_pyll5, 20); s_wl6 <= v_m20(18 downto 9);
            v_m20 := shift_left(resize(s_pyrh5, 20), 4) + resize(s_pyrl5, 20); s_wr6 <= v_m20(18 downto 9);
            v_m19 := shift_left(resize(s_pclh5, 19), 4) + resize(s_pcll5, 19); s_cl6 <= v_m19(16 downto 9);
            v_m19 := shift_left(resize(s_pcrh5, 19), 4) + resize(s_pcrl5, 19); s_cr6 <= v_m19(16 downto 9);
            if s_col5 >= s_lthr then s_lok6 <= '1'; else s_lok6 <= '0'; end if;
            if s_col5 <= s_rthr then s_rok6 <= '1'; else s_rok6 <= '0'; end if;
            -- stage 7: distances; a window off the picture takes the other one's
            -- means (pick, average and relief then all see one-sided smoothing)
            if s_wl6 > s_cy6 then v_dl := s_wl6 - s_cy6; else v_dl := s_cy6 - s_wl6; end if;
            if s_wr6 > s_cy6 then v_dr := s_wr6 - s_cy6; else v_dr := s_cy6 - s_wr6; end if;
            s_dl7 <= v_dl; s_dr7 <= v_dr;
            if s_lok6 = '1' then s_wl7 <= s_wl6; s_cl7 <= s_cl6; else s_wl7 <= s_wr6; s_cl7 <= s_cr6; end if;
            if s_rok6 = '1' then s_wr7 <= s_wr6; s_cr7 <= s_cr6; else s_wr7 <= s_wl6; s_cr7 <= s_cl6; end if;
            if (s_lok6 and s_rok6) = '1' then
                s_rl7 <= signed('0' & s_wl6) - signed('0' & s_wr6);
            else
                s_rl7 <= (others => '0');
            end if;
            s_flip7 <= s_sw_far xor (s_sw_steer and s_st6);
            -- stage 8: pick (the luma decision picks the chroma too), average,
            -- relief product
            if s_dl7 <= s_dr7 then v_near := '1'; else v_near := '0'; end if;
            v_sel := v_near xor s_flip7;
            if v_sel = '1' then s_pick8 <= s_wl7; s_pc8 <= s_cl7; else s_pick8 <= s_wr7; s_pc8 <= s_cr7; end if;
            s_avg8 <= resize(shift_right(resize(s_wl7, 11) + resize(s_wr7, 11), 1), 10);
            s_ac8 <= resize(shift_right(resize(s_cl7, 9) + resize(s_cr7, 9), 1), 8);
            -- (split on the Relief nibbles, sign applied after the product: the negate+mux in
            -- front of an 11x9 multiply was the HD critical path)
            s_pr8h <= s_rl7 * signed('0' & s_relief(7 downto 4));
            s_pr8l <= s_rl7 * signed('0' & s_relief(3 downto 0));
            -- stage 9: pick - avg, relief term (up to +-2038: 12 bits)
            s_e9  <= signed('0' & s_pick8) - signed('0' & s_avg8);
            s_ec9 <= resize(s_pc8, 9) - resize(s_ac8, 9);
            s_avg9 <= s_avg8; s_ac9 <= s_ac8;
            if s_sw_right = '1' then
                s_rf(9) <= -resize(shift_right(shift_left(resize(s_pr8h, 20), 4) + resize(s_pr8l, 20), 7), 12);
            else
                s_rf(9) <= resize(shift_right(shift_left(resize(s_pr8h, 20), 4) + resize(s_pr8l, 20), 7), 12);
            end if;
            for i in 10 to 15 loop s_rf(i) <= s_rf(i - 1); end loop;
            -- stage 10: edge products (chroma on the top Edge nibble)
            s_pe10  <= s_e9 * signed('0' & s_edge);
            s_pec10 <= s_ec9 * signed('0' & s_edge(7 downto 4));
            s_avg10 <= s_avg9; s_ac10 <= s_ac9;
            -- stage 11: brushed colour (between the average and the pick); the
            -- vertical state for this column lands from stY / stC
            v_e  := signed('0' & s_avg10) + resize(shift_right(s_pe10, 8), 11);
            s_hy11 <= unsigned(v_e(9 downto 0));
            s_hc11 <= s_ac10 + resize(shift_right(s_pec10, 4), 8);
            s_sy11 <= unsigned(s_stq_y);
            s_stc11 <= signed(s_stq_c);
            -- stage 12: step from the column state
            s_dy12 <= signed('0' & s_hy11) - signed('0' & s_sy11);
            s_dc12 <= resize(s_hc11, 9) - resize(s_stc11, 9);
            s_syd(12) <= s_sy11; s_stcd(12) <= s_stc11;
            for i in 13 to 14 loop s_syd(i) <= s_syd(i - 1); s_stcd(i) <= s_stcd(i - 1); end loop;
            -- stage 13: restart test (a step past +-T, or the first line of the
            -- frame); step x (2 + vb) + stochastic rounding offset
            if s_dy12 > s_vtp or s_dy12 < s_vtn or s_dc12 > s_vtcp or s_dc12 < s_vtcn or s_vfirst = '1' then
                s_rst13 <= '1';
            else
                s_rst13 <= '0';
            end if;
            if s_vb = '1' then
                v_my := shift_left(resize(s_dy12, 13), 1) + resize(s_dy12, 13);
                v_mc := shift_left(resize(s_dc12, 11), 1) + resize(s_dc12, 11);
            else
                v_my := shift_left(resize(s_dy12, 13), 1);
                v_mc := shift_left(resize(s_dc12, 11), 1);
            end if;
            s_qy13 <= v_my + signed(resize(s_lfsr(5 downto 0) and s_vmask, 13));
            s_qc13 <= v_mc + signed(resize(s_lfsr(13 downto 8) and s_vmask, 11));
            s_dy13 <= s_dy12; s_dc13 <= s_dc12;
            -- stage 14: the move -- the whole step on a restart, else floor(q / 2^vk)
            if s_rst13 = '1' then
                s_vy14 <= s_dy13; s_vcm14 <= s_dc13;
            else
                case to_integer(s_vk) is
                    when 2 =>      s_vy14 <= resize(shift_right(s_qy13, 2), 11); s_vcm14 <= resize(shift_right(s_qc13, 2), 9);
                    when 3 =>      s_vy14 <= resize(shift_right(s_qy13, 3), 11); s_vcm14 <= resize(shift_right(s_qc13, 3), 9);
                    when 4 =>      s_vy14 <= resize(shift_right(s_qy13, 4), 11); s_vcm14 <= resize(shift_right(s_qc13, 4), 9);
                    when 5 =>      s_vy14 <= resize(shift_right(s_qy13, 5), 11); s_vcm14 <= resize(shift_right(s_qc13, 5), 9);
                    when others => s_vy14 <= resize(shift_right(s_qy13, 6), 11); s_vcm14 <= resize(shift_right(s_qc13, 6), 9);
                end case;
            end if;
            -- stage 15: new column state (stays between the old state and the
            -- brushed value, so no clamp)
            v_e := signed('0' & s_syd(14)) + s_vy14;
            s_ny15 <= unsigned(v_e(9 downto 0));
            v_e := resize(s_stcd(14), 11) + resize(s_vcm14, 11); s_nc15 <= v_e(7 downto 0);
            -- stage 16: + relief
            s_t16 <= signed(resize(s_ny15, 13)) + resize(s_rf(15), 13);
            s_nc16 <= s_nc15;
            -- stage 17: clamp 64..940, optional knife (8 steps, the top one 940);
            -- chroma back to offset binary, paired with the column before (an odd
            -- column writes its U with the even column's V)
            v_y := f_clamp_y(s_t16);
            if s_sw_knife = '0' then
                s_wy17 <= v_y;
            elsif v_y(9 downto 7) = "111" then
                s_wy17 <= to_unsigned(940, 10);
            else
                s_wy17 <= (v_y and "1110000000") or "0001000000";
            end if;
            v_ec := s_nc16; s_wc17 <= unsigned((not v_ec(7)) & std_logic_vector(v_ec(6 downto 0)));
            s_wcp17 <= s_wc17;

            -- columns: stage 5 (edge flags at 6), stage 9 (state read, lands at
            -- 11), stage 15 (state write), stage 17 (line-buffer write)
            if s_ad(4) = '1' then s_col5 <= s_col5 + 1; else s_col5 <= (others => '1'); end if;
            if s_ad(8) = '1' then s_col9 <= s_col9 + 1; else s_col9 <= (others => '1'); end if;
            if s_ad(14) = '1' then s_col15 <= s_col15 + 1; else s_col15 <= (others => '1'); end if;
            s_we15 <= s_ad(14);
            s_we17 <= s_ad(16);
            if s_we17 = '1' then s_wcnt <= s_wcnt + 1; else s_wcnt <= (others => '0'); end if;
            -- frame start: avid absent for 2048 clocks (vertical blanking) arms the
            -- restart; it holds through the first line at stage 13 and clears at
            -- that line's end there
            if data_in.avid = '1' then s_brun <= (others => '0'); elsif s_brun(11) = '0' then s_brun <= s_brun + 1; end if;
            if s_brun(11) = '1' then
                s_vfirst <= '1';
            elsif s_ad(12) = '0' and s_ad(13) = '1' then
                s_vfirst <= '0';
            end if;

            -- per-line: row keys (the row after the last one before the 4096-line
            -- wrap is row 0, so a drifting field has no seam)
            v_k := to_integer(s_k);
            case to_integer(s_lstep) is
                when 0 =>
                    s_ly <= resize(s_line, 12) + resize(s_nphase(15 downto 4), 12);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_cy <= resize(shift_right(s_ly, v_k), 10);
                    case v_k is
                        when 2 => s_fy <= s_ly(1 downto 0) & "00000";
                        when 3 => s_fy <= s_ly(2 downto 0) & "0000";
                        when 4 => s_fy <= s_ly(3 downto 0) & "000";
                        when 5 => s_fy <= s_ly(4 downto 0) & "00";
                        when 6 => s_fy <= s_ly(5 downto 0) & "0";
                        when 7 => s_fy <= s_ly(6 downto 0);
                        when 8 => s_fy <= s_ly(7 downto 1);
                        when others => s_fy <= s_ly(8 downto 2);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_rk0 <= C_SEED + shift_left(resize(s_cy, 16), 7);
                    s_cyn <= (s_cy + 1) and s_ymask;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_rk0p <= s_rk0 + 1;
                    s_rk1  <= C_SEED + shift_left(resize(s_cyn, 16), 7);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_rk1p <= s_rk1 + 1;
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_ampr <= s_p_amount(9 downto 2);
                    s_sthr <= resize(s_p_amount(9 downto 3), 8);
                    if s_k_scale(9 downto 7) = 7 then s_k <= to_unsigned(9, 4); else s_k <= resize(s_k_scale(9 downto 7), 4) + 2; end if;
                    -- window from the x-scaled index, vertical drag from the
                    -- y-scaled one (both the plain K4 index at 1080p)
                    v_b := f_brush(s_bih);
                    s_n   <= unsigned(v_b(21 downto 15));
                    s_rm  <= unsigned(v_b(14 downto 7));
                    s_rsh <= unsigned(v_b(6 downto 4));
                    v_b := f_brush(s_biv);
                    s_vs  <= unsigned(v_b(3 downto 1));
                    s_vb  <= v_b(0);
                    s_edge <= s_k_edge(9 downto 2);
                    s_relief <= s_k_relief(9 downto 2);
                    s_nphase <= s_nphase + resize(s_k_drift(9 downto 3), 16);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if to_integer(s_k) + to_integer(s_k_stroke(9 downto 8)) > 10 then
                        s_kx <= to_unsigned(10, 4);
                    else
                        s_kx <= s_k + resize(s_k_stroke(9 downto 8), 4);
                    end if;
                    case to_integer(s_k) is
                        when 2 => s_ymask <= to_unsigned(1023, 10);
                        when 3 => s_ymask <= to_unsigned(511, 10);
                        when 4 => s_ymask <= to_unsigned(255, 10);
                        when 5 => s_ymask <= to_unsigned(127, 10);
                        when 6 => s_ymask <= to_unsigned(63, 10);
                        when 7 => s_ymask <= to_unsigned(31, 10);
                        when 8 => s_ymask <= to_unsigned(15, 10);
                        when others => s_ymask <= to_unsigned(7, 10);
                    end case;
                    s_rmyl <= s_rm; s_rmyr <= s_rm; s_rmcl <= s_rm; s_rmcr <= s_rm;
                    s_ndel <= s_n - 1;
                    s_rdel <= to_unsigned(C_DLY - 2, 7) - s_n;
                    s_lthr <= to_unsigned(C_EDGE, 11) + resize(s_n, 11);
                    s_vk <= s_vs + 1;
                    s_tinv <= 255 - s_edge;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    case to_integer(s_vk) is
                        when 2 => s_vmask <= to_unsigned(3, 6);
                        when 3 => s_vmask <= to_unsigned(7, 6);
                        when 4 => s_vmask <= to_unsigned(15, 6);
                        when 5 => s_vmask <= to_unsigned(31, 6);
                        when others => s_vmask <= to_unsigned(63, 6);
                    end case;
                    -- vertical restart step: 48 + 3 * (255 - Edge) luma codes
                    -- (Edge 0 ~ 813, a plain smear; Edge 100% = 48)
                    s_vt3 <= resize(s_tinv, 10) + shift_left(resize(s_tinv, 10), 1);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_vt <= s_vt3 + 48;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_vtc <= s_vt(9 downto 2);
                    s_vtp <= signed(resize(s_vt, 12));
                    s_vtn <= -signed(resize(s_vt, 12));
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_vtcp <= signed(resize(s_vtc, 10));
                    s_vtcn <= -signed(resize(s_vtc, 10));
                    s_vstep <= "111";
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_rthr <= s_w_edge - resize(s_n, 11);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_lya <= s_lya + s_ys;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_lya <= (others => '0');
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
    -- Vertical brush state: per column, read at stage 9 (lands re-registered
    -- at 11), written back at stage 15 -- one line before it is read again.
    -- Chroma per column: U on odd columns, V on even.
    --------------------------------------------------------------------------
    p_stY : process(clk)
    begin
        if rising_edge(clk) then
            s_stq_y <= stY(to_integer(s_col9));
            if s_we15 = '1' then stY(to_integer(s_col15)) <= std_logic_vector(s_ny15); end if;
        end if;
    end process;
    p_stC : process(clk)
    begin
        if rising_edge(clk) then
            s_stq_c <= stC(to_integer(s_col9));
            if s_we15 = '1' then stC(to_integer(s_col15)) <= std_logic_vector(s_nc15); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Noise + address pipeline (stages 1..17), as in tremor.  The noise is
    -- evaluated at s_rxn = s_rx + C_NLEAD (a 13 px shift of the field), so the
    -- steer bit at stage 13 belongs to the pixel entering the dring.
    --------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_kx : integer range 2 to 10;
        variable v_t, v_b : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_kx := to_integer(s_kx);
            s_cx1 <= resize(shift_right(s_rxn, v_kx), 10);
            case v_kx is
                -- (fractions from the Q.1 accumulator: s_vxa = 2 x s_rxn at
                -- 1080p, so these are the old slices there)
                when 2 => s_fx1 <= s_vxa(2 downto 0) & "0000";
                when 3 => s_fx1 <= s_vxa(3 downto 0) & "000";
                when 4 => s_fx1 <= s_vxa(4 downto 0) & "00";
                when 5 => s_fx1 <= s_vxa(5 downto 0) & "0";
                when 6 => s_fx1 <= s_vxa(6 downto 0);
                when 7 => s_fx1 <= s_vxa(7 downto 1);
                when 8 => s_fx1 <= s_vxa(8 downto 2);
                when 9 => s_fx1 <= s_vxa(9 downto 3);
                when others => s_fx1 <= s_vxa(10 downto 4);
            end case;
            s_ka <= s_rk0  + resize(s_cx1, 16);
            s_kb <= s_rk0p + resize(s_cx1, 16);
            s_kc <= s_rk1  + resize(s_cx1, 16);
            s_kd <= s_rk1p + resize(s_cx1, 16);
            if s_sw_blocky = '1' then s_fxd(2) <= (others => '0'); else s_fxd(2) <= s_fx1; end if;
            for i in 3 to 7 loop s_fxd(i) <= s_fxd(i - 1); end loop;
            s_ha3 <= f_round1(s_ka); s_hb3 <= f_round1(s_kb); s_hc3 <= f_round1(s_kc); s_hd3 <= f_round1(s_kd);
            s_ha4 <= f_round2(s_ha3); s_hb4 <= f_round2(s_hb3); s_hc4 <= f_round2(s_hc3); s_hd4 <= f_round2(s_hd3);
            s_ha5 <= f_round3(s_ha4); s_hb5 <= f_round3(s_hb4); s_hc5 <= f_round3(s_hc4); s_hd5 <= f_round3(s_hd4);
            s_a6 <= s_ha5(15 downto 8); s_b6 <= s_hb5(15 downto 8); s_c6 <= s_hc5(15 downto 8); s_d6 <= s_hd5(15 downto 8);
            s_dab7 <= signed('0' & s_b6) - signed('0' & s_a6);
            s_dcd7 <= signed('0' & s_d6) - signed('0' & s_c6);
            s_a7 <= s_a6; s_c7 <= s_c6;
            s_pab8 <= s_dab7 * signed('0' & s_fxd(7));
            s_pcd8 <= s_dcd7 * signed('0' & s_fxd(7));
            s_a8 <= s_a7; s_c8 <= s_c7;
            v_t := signed(resize(s_a8, 10) & '0') + resize(shift_right(s_pab8, 6), 11);
            v_b := signed(resize(s_c8, 10) & '0') + resize(shift_right(s_pcd8, 6), 11);
            s_top9 <= unsigned(v_t(8 downto 1));
            s_bot9 <= unsigned(v_b(8 downto 1));
            s_dtb10 <= signed('0' & s_bot9) - signed('0' & s_top9);
            s_top10 <= s_top9;
            if s_sw_blocky = '1' then s_ptb11 <= (others => '0'); else s_ptb11 <= s_dtb10 * signed('0' & s_fy); end if;
            s_top11 <= s_top10;
            v_t := signed(resize(s_top11, 10) & '0') + resize(shift_right(s_ptb11, 6), 11);
            s_n12 <= unsigned(v_t(8 downto 1));
            s_nc13 <= signed('0' & s_n12) - to_signed(128, 9);
            s_d14 <= s_nc13 * signed('0' & s_amp);
            if s_sw_steer = '1' then s_off15 <= (others => '0'); else s_off15 <= s_d14(16 downto 7); end if;
            -- Steer: flip the pick where the noise sits below the P12 threshold
            -- (none at 0, half the picture at 100); rides the dring to the pick
            if s_n12 < s_sthr then s_steer <= '1'; else s_steer <= '0'; end if;
            s_u16 <= signed(resize(s_x15, 13)) + resize(s_off15, 13);
            s_addr17 <= f_edge_addr(s_u16, s_w_edge);
        end if;
    end process p_noise;

    --------------------------------------------------------------------------
    -- Line buffer, ONE bank: column c of line L is written at input-side stage
    -- 17 (~C_DLY + 14 clocks after the input passed it) and read at noise stage
    -- 17, so an undisplaced read always precedes its overwrite.  A read
    -- displaced far enough left sees line L instead of L-1 (invisible); the one
    -- that lands on the column being written that clock takes the written data
    -- (p_pix), since the EBR returns undefined data there.
    --------------------------------------------------------------------------
    p_lbY : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY <= lbY(to_integer(s_addr17));
            if s_we17 = '1' then lbY(to_integer(s_wcnt)) <= std_logic_vector(s_wy17); end if;
        end if;
    end process;
    p_lbC : process(clk)
    begin
        if rising_edge(clk) then
            s_rdC <= lbC(to_integer(s_addr17(10 downto 1)));
            if s_we17 = '1' and s_wcnt(0) = '1' then
                lbC(to_integer(s_wcnt(10 downto 1))) <= std_logic_vector(s_wc17) & std_logic_vector(s_wcp17);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch + gate (stages 18..21)
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 18: same-address read-during-write -> take the written data
            if s_we17 = '1' and s_addr17 = s_wcnt then s_byy <= '1'; else s_byy <= '0'; end if;
            if s_we17 = '1' and s_wcnt(0) = '1' and s_addr17(10 downto 1) = s_wcnt(10 downto 1) then
                s_byc <= '1';
            else
                s_byc <= '0';
            end if;
            s_byd_y <= s_wy17; s_byd_u <= s_wc17; s_byd_v <= s_wcp17;
            -- stage 19
            if s_byy = '1' then s_y19 <= s_byd_y; else s_y19 <= unsigned(s_rdY); end if;
            if s_byc = '1' then
                s_u19 <= s_byd_u & "00"; s_v19 <= s_byd_v & "00";
            else
                s_u19 <= unsigned(s_rdC(15 downto 8)) & "00"; s_v19 <= unsigned(s_rdC(7 downto 0)) & "00";
            end if;
            -- stage 20: blanking gate; 21: output register
            if s_avid_sr(C_LAT - 3) = '1' then
                s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
            s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y21);
    s_io_align_in.u       <= std_logic_vector(s_u21);
    s_io_align_in.v       <= std_logic_vector(s_v21);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (21 -> 24).
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

end architecture impasto;
