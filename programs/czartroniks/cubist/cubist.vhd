-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- cubist.vhd
--
-- CUBIST v0.1 -- a photoreal-clean Rubik's cube floating on a soft backdrop.
-- This stage: static SOLVED cube, whole-cube rotation on K1/K2/K3
-- (yaw/pitch/roll, glide-smoothed), auto-zoom framing, satin-black body with
-- rounded cubies, inset rounded stickers, gap AO, silhouette AA with rounded
-- outline corners, gradient+vignette backdrop.  Layer turns, specular rig,
-- MANUAL/AUTO modes and the solver stack on top of this datapath in later
-- stages (see SUMMARY.md).
--
-- ARCHITECTURE (streaming, no line buffer; orthographic camera):
--   GEOMETRY ENGINE (vblank): a small microcoded processor (frame_ucode.py,
--     assembled by uasm.py, checked by uemu.py) glides the three angles,
--     rotates the cube basis, auto-zooms, culls, and for each visible face
--     (<= 3 slots) writes the per-slot descriptors (H projections, light,
--     albedo, chroma) and the face's affine UV map: gradient, field seed and
--     per-line wrap for u and v.  ~9,500 clocks per field.
--   PER-FACE DDAs (p_dda): six accumulators, u and v of every visible face,
--     stepped per active pixel and wrapped once per line.  The owner of a
--     pixel is whichever face's (u, v) lies inside the square -- the faces of
--     a convex solid never overlap on screen -- with a 64-unit padded
--     fallback for crease pixels that independent rounding leaves unclaimed.
--   PIXEL PIPE (C_LATENCY stages): cell split, rounded-rect SDFs for sticker
--     / cubie gaps / face bevel + silhouette (sqrt-free: squared distances by
--     table, linearized near the zero crossing), half-vector tilt/specular
--     rig, analytic AA, gradient + vignette backdrop, alpha compose,
--     blanking gate.
--
-- Q formats: angles 12b/turn; sin/cos Q12; basis vectors Q2.12; cube spans
-- +/-1.5 units (one cubie = 4096); camera at D = 13.0 (Q12).  UV in Q12
-- cubie units, 0..12288 across a face.
--
-- v0.5.6 (2026-10-18): resolution scaling.  Interlace now comes from the
-- timing ID (480i / 576i / 1080i): it was detected from field_n toggling,
-- which core_top ties high, so SD drew the cube in field lines -- twice as
-- tall as wide.  On SD the projected x axes are also scaled by the pixel
-- aspect (x1.111 on 480 lines, x0.9375 on 576; CTL 15), so the cube stays
-- square on a 4:3 screen.  HD is unchanged (offset 0).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture cubist of program_top is
    -- data_in -> data_out latency is 16 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;

    constant C_LATENCY : integer := 16;

    ----------------------------------------------------------------------
    -- geometry constants (Q12 cubie units)
    ----------------------------------------------------------------------
    constant C_FACE  : integer := 12288;         -- 3 cubies
    constant C_RB    : integer := 328;           -- outer edge rounding 0.080
    constant C_RB2   : integer := 13448;         -- C_RB^2 >> 3 (ROM domain)
    constant C_GB    : integer := 331;           -- gap+cubie-round zone 0.081
    constant C_CBW2  : integer := 6328;          -- CBW^2 >> 3 (CBW = 225)
    constant C_HSRS  : integer := 1405;          -- sticker |p| inset (HS-RS)
    constant C_RS    : integer := 348;           -- sticker corner radius
    constant C_RS2   : integer := 15138;         -- C_RS^2 >> 3 (ROM domain)
    constant C_INSET : integer := 98;            -- silhouette inset 0.024
    constant C_CRH   : integer := 32;            -- outline-corner bevel half-leg
    constant C_SEW   : integer := 123;           -- sticker edge bevel 0.030
    constant C_AMB   : integer := 62;            -- ambient, linear Q8
    constant C_SEDEF : integer := 55;            -- sticker rim light deficit
    -- linearization / profile constants
    constant C_RRB   : integer := 25;            -- ~16384/(2*RB)
    constant C_RRS   : integer := 24;            -- ~16384/(2*RS)
    constant C_WBEV  : integer := 331;           -- wbev = s*331 >> 16 (~s/CBW^2)
    constant C_RAO   : integer := 285;           -- ao ramp ~1/230 Q16

    -- camera: D = 13.0 in Q12
    constant C_CAMD  : integer := 53248;

    ----------------------------------------------------------------------
    -- quarter-sine table, Q12: C_SIN(k) = round(4096*sin(k*pi/128))
    ----------------------------------------------------------------------
    -- Quarter-sine, Q12.  Padded to 256x16 so it maps to a block RAM:
    -- at 65x14 yosys leaves it in logic, which cost ~600 cells on a
    -- chip that is over budget on logic and has 23 spare memories.
    type t_sin256 is array (0 to 255) of signed(15 downto 0);
    constant C_SIN : t_sin256 := (
        to_signed(    0,16), to_signed(  101,16), to_signed(  201,16), to_signed(  301,16), to_signed(  401,16), to_signed(  501,16), to_signed(  601,16), to_signed(  700,16),
        to_signed(  799,16), to_signed(  897,16), to_signed(  995,16), to_signed( 1092,16), to_signed( 1189,16), to_signed( 1285,16), to_signed( 1380,16), to_signed( 1474,16),
        to_signed( 1567,16), to_signed( 1660,16), to_signed( 1751,16), to_signed( 1842,16), to_signed( 1931,16), to_signed( 2019,16), to_signed( 2106,16), to_signed( 2191,16),
        to_signed( 2276,16), to_signed( 2359,16), to_signed( 2440,16), to_signed( 2520,16), to_signed( 2598,16), to_signed( 2675,16), to_signed( 2751,16), to_signed( 2824,16),
        to_signed( 2896,16), to_signed( 2967,16), to_signed( 3035,16), to_signed( 3102,16), to_signed( 3166,16), to_signed( 3229,16), to_signed( 3290,16), to_signed( 3349,16),
        to_signed( 3406,16), to_signed( 3461,16), to_signed( 3513,16), to_signed( 3564,16), to_signed( 3612,16), to_signed( 3659,16), to_signed( 3703,16), to_signed( 3745,16),
        to_signed( 3784,16), to_signed( 3822,16), to_signed( 3857,16), to_signed( 3889,16), to_signed( 3920,16), to_signed( 3948,16), to_signed( 3973,16), to_signed( 3996,16),
        to_signed( 4017,16), to_signed( 4036,16), to_signed( 4052,16), to_signed( 4065,16), to_signed( 4076,16), to_signed( 4085,16), to_signed( 4091,16), to_signed( 4095,16),
        to_signed( 4096,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16) );

    function f_clamp10(v : signed) return unsigned is

    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return resize(unsigned(v), 10); end if;
    end function;

    -- (u or v) of one face DDA inside [0, 3 cubies)?  Q8 when e = '0', and
    -- integer units when e = '1' (a steep face, gradient block-floated by 8)
    -- (u or v) of one face DDA inside [0, n cubies)?  Q6 units when e = '0',
    -- whole units when e = '1' (a steep face, block-floated by 6)
    -- (u or v) of one face DDA inside [0, n cubies)?  Q6 units when e = '0',
    -- whole units when e = '1' (a steep face, block-floated by 6).  Written
    -- as zero tests plus a 2-bit compare: as magnitude compares they became
    -- 24 carry chains.
    function f_inside(a : signed(26 downto 0); e : std_logic;
                      n : unsigned(1 downto 0)) return boolean is
    begin
        if a(26) = '1' then return false; end if;
        if e = '0' then return a(25 downto 20) = 0 and unsigned(a(19 downto 18)) < n;
        else            return a(25 downto 14) = 0 and unsigned(a(13 downto 12)) < n; end if;
    end function;

    -- the same test widened by 64 units each side: the fallback owner for a
    -- pixel on a crease that both faces' independently-rounded DDAs reject
    function f_inpad(a : signed(26 downto 0); e : std_logic;
                     n : unsigned(1 downto 0)) return boolean is
    begin
        if e = '0' then
            if a(26) = '1' then return a(25 downto 12) = -1; end if;
            return a(25 downto 20) = 0
                   and (unsigned(a(19 downto 18)) < n
                        or (unsigned(a(19 downto 18)) = n and a(17 downto 12) = 0));
        else
            if a(26) = '1' then return a(25 downto 6) = -1; end if;
            return a(25 downto 14) = 0
                   and (unsigned(a(13 downto 12)) < n
                        or (unsigned(a(13 downto 12)) = n and a(11 downto 6) = 0));
        end if;
    end function;

    -- index of the lowest set bit of a 6-bit vector (0 if none)
    function f_first6(v : std_logic_vector(5 downto 0)) return unsigned is
    begin
        if    v(0) = '1' then return "000";
        elsif v(1) = '1' then return "001";
        elsif v(2) = '1' then return "010";
        elsif v(3) = '1' then return "011";
        elsif v(4) = '1' then return "100";
        else                  return "101"; end if;
    end function;

    -- two or more bits set
    -- (as plain logic: a counted popcount became a carry chain on stage A's
    -- critical path) two in one half, or one in each half
    function f_two6(v : std_logic_vector(5 downto 0)) return boolean is
        variable lo2, hi2, loa, hia : std_logic;
    begin
        lo2 := (v(0) and v(1)) or (v(0) and v(2)) or (v(1) and v(2));
        hi2 := (v(3) and v(4)) or (v(3) and v(5)) or (v(4) and v(5));
        loa := v(0) or v(1) or v(2);
        hia := v(3) or v(4) or v(5);
        return (lo2 or hi2 or (loa and hia)) = '1';
    end function;

    function f_uvsl(a : signed(26 downto 0); e : std_logic) return signed is
    begin
        if e = '1' then return a(15 downto 0);
        else            return a(21 downto 6); end if;
    end function;

    -- Cell split by bit slicing.  One cubie is 4096 units, so the cell is
    -- bits 13..12 and the in-cell coordinate p = low 12 bits - 2048.  The
    -- two distances every zone needs come straight off the low 11 bits:
    --   d = 2048 - |p|  (to the nearest grid line)
    --   q = |p| - HSRS  (past the sticker inset)
    -- each two parallel subtracts and a mux, instead of abs -> subtract ->
    -- clamp in series.  Out of range (the one-pixel ownership lag) clamps
    -- to the face edge, which is what the old saturating chain produced.
    procedure p_cell(x : in signed(15 downto 0); n : in unsigned(1 downto 0);
                     d : out unsigned(11 downto 0); q : out signed(11 downto 0);
                     ng : out std_logic; cell : out unsigned(1 downto 0)) is
        variable m : unsigned(11 downto 0);
    begin
        m := resize(unsigned(x(10 downto 0)), 12);
        if x(15) = '1' then
            d := (others => '0'); q := to_signed(643, 12); ng := '1'; cell := "00";
        elsif unsigned(x(14 downto 12)) >= n then       -- past the far edge
            d := (others => '0'); q := to_signed(643, 12); ng := '0'; cell := n - 1;
        else
            cell := unsigned(x(13 downto 12));
            if x(11) = '1' then
                d := to_unsigned(2048, 12) - m;
                q := signed(m) - to_signed(C_HSRS, 12);
                ng := '0';
            else
                d := m;
                q := to_signed(2048 - C_HSRS, 12) - signed(m);
                ng := '1';
            end if;
        end if;
    end procedure;



    ----------------------------------------------------------------------
    -- sticker colors, 10-bit BT.601 (chroma centred 512), by face id
    -- 0=U white  1=D yellow  2=R red  3=L orange  4=F green  5=B blue
    ----------------------------------------------------------------------
    type t_col6 is array (0 to 5) of unsigned(9 downto 0);
    constant C_STY : t_col6 := (to_unsigned(995,10), to_unsigned(807,10), to_unsigned(332,10),
                                to_unsigned(513,10), to_unsigned(416,10), to_unsigned(276,10));
    constant C_STU : t_col6 := (to_unsigned(512,10), to_unsigned( 91,10), to_unsigned(460,10),
                                to_unsigned(244,10), to_unsigned(496,10), to_unsigned(757,10));
    constant C_STV : t_col6 := (to_unsigned(512,10), to_unsigned(654,10), to_unsigned(811,10),
                                to_unsigned(848,10), to_unsigned(238,10), to_unsigned(330,10));
    constant C_PLY_I : integer := 173;   -- satin black, VIDEO luma of 0.020 linear

    -- half-vector H = normalize(Lkey + V), V = world +z (orthographic), Q8
    constant C_HX : integer := -57;
    constant C_HY : integer :=  81;
    constant C_HZ : integer := 236;

    ----------------------------------------------------------------------
    -- MATERIAL ROMs.  Separate constants per read port: GHDL's synth pass
    -- crashes on two parallel reads of one array-of-arrays constant.
    ----------------------------------------------------------------------
    type t_rom16 is array (0 to 255) of unsigned(15 downto 0);
    type t_rom8  is array (0 to 255) of unsigned(7 downto 0);

    -- bevel tilt profile, indexed by penetration>>1 (164 = full 80 deg):
    -- high byte sin(phi), low byte 1-cos(phi), both Q8.
    constant C_TILTA : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  512,16),
        to_unsigned(  512,16), to_unsigned(  768,16), to_unsigned(  768,16), to_unsigned( 1024,16), to_unsigned( 1024,16), to_unsigned( 1280,16), to_unsigned( 1280,16), to_unsigned( 1536,16),
        to_unsigned( 1792,16), to_unsigned( 2048,16), to_unsigned( 2048,16), to_unsigned( 2304,16), to_unsigned( 2560,16), to_unsigned( 2816,16), to_unsigned( 3072,16), to_unsigned( 3328,16),
        to_unsigned( 3584,16), to_unsigned( 3840,16), to_unsigned( 4096,16), to_unsigned( 4353,16), to_unsigned( 4609,16), to_unsigned( 4865,16), to_unsigned( 5121,16), to_unsigned( 5377,16),
        to_unsigned( 5633,16), to_unsigned( 5889,16), to_unsigned( 6145,16), to_unsigned( 6657,16), to_unsigned( 6913,16), to_unsigned( 7170,16), to_unsigned( 7682,16), to_unsigned( 7938,16),
        to_unsigned( 8194,16), to_unsigned( 8706,16), to_unsigned( 8962,16), to_unsigned( 9219,16), to_unsigned( 9731,16), to_unsigned( 9987,16), to_unsigned(10499,16), to_unsigned(10756,16),
        to_unsigned(11268,16), to_unsigned(11524,16), to_unsigned(12036,16), to_unsigned(12549,16), to_unsigned(12805,16), to_unsigned(13317,16), to_unsigned(13574,16), to_unsigned(14086,16),
        to_unsigned(14598,16), to_unsigned(15111,16), to_unsigned(15367,16), to_unsigned(15880,16), to_unsigned(16392,16), to_unsigned(16905,16), to_unsigned(17161,16), to_unsigned(17674,16),
        to_unsigned(18186,16), to_unsigned(18699,16), to_unsigned(19211,16), to_unsigned(19724,16), to_unsigned(19980,16), to_unsigned(20493,16), to_unsigned(21006,16), to_unsigned(21518,16),
        to_unsigned(22031,16), to_unsigned(22544,16), to_unsigned(23056,16), to_unsigned(23569,16), to_unsigned(24082,16), to_unsigned(24595,16), to_unsigned(25108,16), to_unsigned(25620,16),
        to_unsigned(26133,16), to_unsigned(26646,16), to_unsigned(27159,16), to_unsigned(27672,16), to_unsigned(28185,16), to_unsigned(28698,16), to_unsigned(29467,16), to_unsigned(29980,16),
        to_unsigned(30493,16), to_unsigned(31006,16), to_unsigned(31520,16), to_unsigned(32033,16), to_unsigned(32546,16), to_unsigned(33059,16), to_unsigned(33829,16), to_unsigned(34342,16),
        to_unsigned(34855,16), to_unsigned(35369,16), to_unsigned(35882,16), to_unsigned(36395,16), to_unsigned(36909,16), to_unsigned(37678,16), to_unsigned(38192,16), to_unsigned(38705,16),
        to_unsigned(39219,16), to_unsigned(39733,16), to_unsigned(40246,16), to_unsigned(40760,16), to_unsigned(41530,16), to_unsigned(42044,16), to_unsigned(42557,16), to_unsigned(43071,16),
        to_unsigned(43585,16), to_unsigned(44099,16), to_unsigned(44613,16), to_unsigned(45127,16), to_unsigned(45641,16), to_unsigned(46155,16), to_unsigned(46925,16), to_unsigned(47439,16),
        to_unsigned(47953,16), to_unsigned(48467,16), to_unsigned(48982,16), to_unsigned(49496,16), to_unsigned(50010,16), to_unsigned(50525,16), to_unsigned(50783,16), to_unsigned(51297,16),
        to_unsigned(51812,16), to_unsigned(52326,16), to_unsigned(52841,16), to_unsigned(53355,16), to_unsigned(53870,16), to_unsigned(54129,16), to_unsigned(54643,16), to_unsigned(55158,16),
        to_unsigned(55673,16), to_unsigned(55931,16), to_unsigned(56446,16), to_unsigned(56961,16), to_unsigned(57220,16), to_unsigned(57735,16), to_unsigned(57994,16), to_unsigned(58509,16),
        to_unsigned(58768,16), to_unsigned(59283,16), to_unsigned(59542,16), to_unsigned(60057,16), to_unsigned(60316,16), to_unsigned(60575,16), to_unsigned(61091,16), to_unsigned(61350,16),
        to_unsigned(61609,16), to_unsigned(61868,16), to_unsigned(62128,16), to_unsigned(62387,16), to_unsigned(62646,16), to_unsigned(62906,16), to_unsigned(63165,16), to_unsigned(63425,16),
        to_unsigned(63684,16), to_unsigned(63944,16), to_unsigned(64204,16), to_unsigned(64207,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16) );

    constant C_TILTB : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  512,16),
        to_unsigned(  512,16), to_unsigned(  768,16), to_unsigned(  768,16), to_unsigned( 1024,16), to_unsigned( 1024,16), to_unsigned( 1280,16), to_unsigned( 1280,16), to_unsigned( 1536,16),
        to_unsigned( 1792,16), to_unsigned( 2048,16), to_unsigned( 2048,16), to_unsigned( 2304,16), to_unsigned( 2560,16), to_unsigned( 2816,16), to_unsigned( 3072,16), to_unsigned( 3328,16),
        to_unsigned( 3584,16), to_unsigned( 3840,16), to_unsigned( 4096,16), to_unsigned( 4353,16), to_unsigned( 4609,16), to_unsigned( 4865,16), to_unsigned( 5121,16), to_unsigned( 5377,16),
        to_unsigned( 5633,16), to_unsigned( 5889,16), to_unsigned( 6145,16), to_unsigned( 6657,16), to_unsigned( 6913,16), to_unsigned( 7170,16), to_unsigned( 7682,16), to_unsigned( 7938,16),
        to_unsigned( 8194,16), to_unsigned( 8706,16), to_unsigned( 8962,16), to_unsigned( 9219,16), to_unsigned( 9731,16), to_unsigned( 9987,16), to_unsigned(10499,16), to_unsigned(10756,16),
        to_unsigned(11268,16), to_unsigned(11524,16), to_unsigned(12036,16), to_unsigned(12549,16), to_unsigned(12805,16), to_unsigned(13317,16), to_unsigned(13574,16), to_unsigned(14086,16),
        to_unsigned(14598,16), to_unsigned(15111,16), to_unsigned(15367,16), to_unsigned(15880,16), to_unsigned(16392,16), to_unsigned(16905,16), to_unsigned(17161,16), to_unsigned(17674,16),
        to_unsigned(18186,16), to_unsigned(18699,16), to_unsigned(19211,16), to_unsigned(19724,16), to_unsigned(19980,16), to_unsigned(20493,16), to_unsigned(21006,16), to_unsigned(21518,16),
        to_unsigned(22031,16), to_unsigned(22544,16), to_unsigned(23056,16), to_unsigned(23569,16), to_unsigned(24082,16), to_unsigned(24595,16), to_unsigned(25108,16), to_unsigned(25620,16),
        to_unsigned(26133,16), to_unsigned(26646,16), to_unsigned(27159,16), to_unsigned(27672,16), to_unsigned(28185,16), to_unsigned(28698,16), to_unsigned(29467,16), to_unsigned(29980,16),
        to_unsigned(30493,16), to_unsigned(31006,16), to_unsigned(31520,16), to_unsigned(32033,16), to_unsigned(32546,16), to_unsigned(33059,16), to_unsigned(33829,16), to_unsigned(34342,16),
        to_unsigned(34855,16), to_unsigned(35369,16), to_unsigned(35882,16), to_unsigned(36395,16), to_unsigned(36909,16), to_unsigned(37678,16), to_unsigned(38192,16), to_unsigned(38705,16),
        to_unsigned(39219,16), to_unsigned(39733,16), to_unsigned(40246,16), to_unsigned(40760,16), to_unsigned(41530,16), to_unsigned(42044,16), to_unsigned(42557,16), to_unsigned(43071,16),
        to_unsigned(43585,16), to_unsigned(44099,16), to_unsigned(44613,16), to_unsigned(45127,16), to_unsigned(45641,16), to_unsigned(46155,16), to_unsigned(46925,16), to_unsigned(47439,16),
        to_unsigned(47953,16), to_unsigned(48467,16), to_unsigned(48982,16), to_unsigned(49496,16), to_unsigned(50010,16), to_unsigned(50525,16), to_unsigned(50783,16), to_unsigned(51297,16),
        to_unsigned(51812,16), to_unsigned(52326,16), to_unsigned(52841,16), to_unsigned(53355,16), to_unsigned(53870,16), to_unsigned(54129,16), to_unsigned(54643,16), to_unsigned(55158,16),
        to_unsigned(55673,16), to_unsigned(55931,16), to_unsigned(56446,16), to_unsigned(56961,16), to_unsigned(57220,16), to_unsigned(57735,16), to_unsigned(57994,16), to_unsigned(58509,16),
        to_unsigned(58768,16), to_unsigned(59283,16), to_unsigned(59542,16), to_unsigned(60057,16), to_unsigned(60316,16), to_unsigned(60575,16), to_unsigned(61091,16), to_unsigned(61350,16),
        to_unsigned(61609,16), to_unsigned(61868,16), to_unsigned(62128,16), to_unsigned(62387,16), to_unsigned(62646,16), to_unsigned(62906,16), to_unsigned(63165,16), to_unsigned(63425,16),
        to_unsigned(63684,16), to_unsigned(63944,16), to_unsigned(64204,16), to_unsigned(64207,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16) );

    -- specular: video-luma DELTA over a nominal lit base, so the add can
    -- happen after gamma.  High byte sticker, low byte plastic.
    constant C_SPEC : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    1,16), to_unsigned(    1,16),
        to_unsigned(    1,16), to_unsigned(    1,16), to_unsigned(    1,16), to_unsigned(    1,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16),
        to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  258,16), to_unsigned(  258,16),
        to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  514,16), to_unsigned(  514,16), to_unsigned(  515,16),
        to_unsigned(  515,16), to_unsigned(  515,16), to_unsigned(  515,16), to_unsigned(  515,16), to_unsigned(  516,16), to_unsigned(  772,16), to_unsigned(  772,16), to_unsigned(  772,16),
        to_unsigned(  772,16), to_unsigned(  773,16), to_unsigned(  773,16), to_unsigned( 1029,16), to_unsigned( 1030,16), to_unsigned( 1030,16), to_unsigned( 1030,16), to_unsigned( 1287,16),
        to_unsigned( 1287,16), to_unsigned( 1287,16), to_unsigned( 1544,16), to_unsigned( 1544,16), to_unsigned( 1545,16), to_unsigned( 1801,16), to_unsigned( 1801,16), to_unsigned( 1802,16),
        to_unsigned( 2058,16), to_unsigned( 2059,16), to_unsigned( 2315,16), to_unsigned( 2316,16), to_unsigned( 2317,16), to_unsigned( 2573,16), to_unsigned( 2830,16), to_unsigned( 2830,16),
        to_unsigned( 3087,16), to_unsigned( 3088,16), to_unsigned( 3345,16), to_unsigned( 3601,16), to_unsigned( 3602,16), to_unsigned( 3859,16), to_unsigned( 4116,16), to_unsigned( 4373,16),
        to_unsigned( 4373,16), to_unsigned( 4630,16), to_unsigned( 4887,16), to_unsigned( 5144,16), to_unsigned( 5401,16), to_unsigned( 5658,16), to_unsigned( 5915,16), to_unsigned( 6172,16),
        to_unsigned( 6429,16), to_unsigned( 6687,16), to_unsigned( 7200,16), to_unsigned( 7457,16), to_unsigned( 7970,16), to_unsigned( 8228,16), to_unsigned( 8742,16), to_unsigned( 9256,16),
        to_unsigned(10026,16), to_unsigned(11053,16), to_unsigned(12338,16), to_unsigned(14392,16), to_unsigned(17216,16), to_unsigned(21324,16), to_unsigned(27485,16), to_unsigned(35956,16) );

    -- x*x >> 3 for x = 2*index: the squarers become table reads, trading
    -- logic (which is 154% full) for block RAM (which is 21% full).
    constant C_SQA : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    2,16), to_unsigned(    4,16), to_unsigned(    8,16), to_unsigned(   12,16), to_unsigned(   18,16), to_unsigned(   24,16),
        to_unsigned(   32,16), to_unsigned(   40,16), to_unsigned(   50,16), to_unsigned(   60,16), to_unsigned(   72,16), to_unsigned(   84,16), to_unsigned(   98,16), to_unsigned(  112,16),
        to_unsigned(  128,16), to_unsigned(  144,16), to_unsigned(  162,16), to_unsigned(  180,16), to_unsigned(  200,16), to_unsigned(  220,16), to_unsigned(  242,16), to_unsigned(  264,16),
        to_unsigned(  288,16), to_unsigned(  312,16), to_unsigned(  338,16), to_unsigned(  364,16), to_unsigned(  392,16), to_unsigned(  420,16), to_unsigned(  450,16), to_unsigned(  480,16),
        to_unsigned(  512,16), to_unsigned(  544,16), to_unsigned(  578,16), to_unsigned(  612,16), to_unsigned(  648,16), to_unsigned(  684,16), to_unsigned(  722,16), to_unsigned(  760,16),
        to_unsigned(  800,16), to_unsigned(  840,16), to_unsigned(  882,16), to_unsigned(  924,16), to_unsigned(  968,16), to_unsigned( 1012,16), to_unsigned( 1058,16), to_unsigned( 1104,16),
        to_unsigned( 1152,16), to_unsigned( 1200,16), to_unsigned( 1250,16), to_unsigned( 1300,16), to_unsigned( 1352,16), to_unsigned( 1404,16), to_unsigned( 1458,16), to_unsigned( 1512,16),
        to_unsigned( 1568,16), to_unsigned( 1624,16), to_unsigned( 1682,16), to_unsigned( 1740,16), to_unsigned( 1800,16), to_unsigned( 1860,16), to_unsigned( 1922,16), to_unsigned( 1984,16),
        to_unsigned( 2048,16), to_unsigned( 2112,16), to_unsigned( 2178,16), to_unsigned( 2244,16), to_unsigned( 2312,16), to_unsigned( 2380,16), to_unsigned( 2450,16), to_unsigned( 2520,16),
        to_unsigned( 2592,16), to_unsigned( 2664,16), to_unsigned( 2738,16), to_unsigned( 2812,16), to_unsigned( 2888,16), to_unsigned( 2964,16), to_unsigned( 3042,16), to_unsigned( 3120,16),
        to_unsigned( 3200,16), to_unsigned( 3280,16), to_unsigned( 3362,16), to_unsigned( 3444,16), to_unsigned( 3528,16), to_unsigned( 3612,16), to_unsigned( 3698,16), to_unsigned( 3784,16),
        to_unsigned( 3872,16), to_unsigned( 3960,16), to_unsigned( 4050,16), to_unsigned( 4140,16), to_unsigned( 4232,16), to_unsigned( 4324,16), to_unsigned( 4418,16), to_unsigned( 4512,16),
        to_unsigned( 4608,16), to_unsigned( 4704,16), to_unsigned( 4802,16), to_unsigned( 4900,16), to_unsigned( 5000,16), to_unsigned( 5100,16), to_unsigned( 5202,16), to_unsigned( 5304,16),
        to_unsigned( 5408,16), to_unsigned( 5512,16), to_unsigned( 5618,16), to_unsigned( 5724,16), to_unsigned( 5832,16), to_unsigned( 5940,16), to_unsigned( 6050,16), to_unsigned( 6160,16),
        to_unsigned( 6272,16), to_unsigned( 6384,16), to_unsigned( 6498,16), to_unsigned( 6612,16), to_unsigned( 6728,16), to_unsigned( 6844,16), to_unsigned( 6962,16), to_unsigned( 7080,16),
        to_unsigned( 7200,16), to_unsigned( 7320,16), to_unsigned( 7442,16), to_unsigned( 7564,16), to_unsigned( 7688,16), to_unsigned( 7812,16), to_unsigned( 7938,16), to_unsigned( 8064,16),
        to_unsigned( 8192,16), to_unsigned( 8320,16), to_unsigned( 8450,16), to_unsigned( 8580,16), to_unsigned( 8712,16), to_unsigned( 8844,16), to_unsigned( 8978,16), to_unsigned( 9112,16),
        to_unsigned( 9248,16), to_unsigned( 9384,16), to_unsigned( 9522,16), to_unsigned( 9660,16), to_unsigned( 9800,16), to_unsigned( 9940,16), to_unsigned(10082,16), to_unsigned(10224,16),
        to_unsigned(10368,16), to_unsigned(10512,16), to_unsigned(10658,16), to_unsigned(10804,16), to_unsigned(10952,16), to_unsigned(11100,16), to_unsigned(11250,16), to_unsigned(11400,16),
        to_unsigned(11552,16), to_unsigned(11704,16), to_unsigned(11858,16), to_unsigned(12012,16), to_unsigned(12168,16), to_unsigned(12324,16), to_unsigned(12482,16), to_unsigned(12640,16),
        to_unsigned(12800,16), to_unsigned(12960,16), to_unsigned(13122,16), to_unsigned(13284,16), to_unsigned(13448,16), to_unsigned(13612,16), to_unsigned(13778,16), to_unsigned(13944,16),
        to_unsigned(14112,16), to_unsigned(14280,16), to_unsigned(14450,16), to_unsigned(14620,16), to_unsigned(14792,16), to_unsigned(14964,16), to_unsigned(15138,16), to_unsigned(15312,16),
        to_unsigned(15488,16), to_unsigned(15664,16), to_unsigned(15842,16), to_unsigned(16020,16), to_unsigned(16200,16), to_unsigned(16380,16), to_unsigned(16562,16), to_unsigned(16744,16),
        to_unsigned(16928,16), to_unsigned(17112,16), to_unsigned(17298,16), to_unsigned(17484,16), to_unsigned(17672,16), to_unsigned(17860,16), to_unsigned(18050,16), to_unsigned(18240,16),
        to_unsigned(18432,16), to_unsigned(18624,16), to_unsigned(18818,16), to_unsigned(19012,16), to_unsigned(19208,16), to_unsigned(19404,16), to_unsigned(19602,16), to_unsigned(19800,16),
        to_unsigned(20000,16), to_unsigned(20200,16), to_unsigned(20402,16), to_unsigned(20604,16), to_unsigned(20808,16), to_unsigned(21012,16), to_unsigned(21218,16), to_unsigned(21424,16),
        to_unsigned(21632,16), to_unsigned(21840,16), to_unsigned(22050,16), to_unsigned(22260,16), to_unsigned(22472,16), to_unsigned(22684,16), to_unsigned(22898,16), to_unsigned(23112,16),
        to_unsigned(23328,16), to_unsigned(23544,16), to_unsigned(23762,16), to_unsigned(23980,16), to_unsigned(24200,16), to_unsigned(24420,16), to_unsigned(24642,16), to_unsigned(24864,16),
        to_unsigned(25088,16), to_unsigned(25312,16), to_unsigned(25538,16), to_unsigned(25764,16), to_unsigned(25992,16), to_unsigned(26220,16), to_unsigned(26450,16), to_unsigned(26680,16),
        to_unsigned(26912,16), to_unsigned(27144,16), to_unsigned(27378,16), to_unsigned(27612,16), to_unsigned(27848,16), to_unsigned(28084,16), to_unsigned(28322,16), to_unsigned(28560,16),
        to_unsigned(28800,16), to_unsigned(29040,16), to_unsigned(29282,16), to_unsigned(29524,16), to_unsigned(29768,16), to_unsigned(30012,16), to_unsigned(30258,16), to_unsigned(30504,16),
        to_unsigned(30752,16), to_unsigned(31000,16), to_unsigned(31250,16), to_unsigned(31500,16), to_unsigned(31752,16), to_unsigned(32004,16), to_unsigned(32258,16), to_unsigned(32512,16) );

    constant C_SQB : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    2,16), to_unsigned(    4,16), to_unsigned(    8,16), to_unsigned(   12,16), to_unsigned(   18,16), to_unsigned(   24,16),
        to_unsigned(   32,16), to_unsigned(   40,16), to_unsigned(   50,16), to_unsigned(   60,16), to_unsigned(   72,16), to_unsigned(   84,16), to_unsigned(   98,16), to_unsigned(  112,16),
        to_unsigned(  128,16), to_unsigned(  144,16), to_unsigned(  162,16), to_unsigned(  180,16), to_unsigned(  200,16), to_unsigned(  220,16), to_unsigned(  242,16), to_unsigned(  264,16),
        to_unsigned(  288,16), to_unsigned(  312,16), to_unsigned(  338,16), to_unsigned(  364,16), to_unsigned(  392,16), to_unsigned(  420,16), to_unsigned(  450,16), to_unsigned(  480,16),
        to_unsigned(  512,16), to_unsigned(  544,16), to_unsigned(  578,16), to_unsigned(  612,16), to_unsigned(  648,16), to_unsigned(  684,16), to_unsigned(  722,16), to_unsigned(  760,16),
        to_unsigned(  800,16), to_unsigned(  840,16), to_unsigned(  882,16), to_unsigned(  924,16), to_unsigned(  968,16), to_unsigned( 1012,16), to_unsigned( 1058,16), to_unsigned( 1104,16),
        to_unsigned( 1152,16), to_unsigned( 1200,16), to_unsigned( 1250,16), to_unsigned( 1300,16), to_unsigned( 1352,16), to_unsigned( 1404,16), to_unsigned( 1458,16), to_unsigned( 1512,16),
        to_unsigned( 1568,16), to_unsigned( 1624,16), to_unsigned( 1682,16), to_unsigned( 1740,16), to_unsigned( 1800,16), to_unsigned( 1860,16), to_unsigned( 1922,16), to_unsigned( 1984,16),
        to_unsigned( 2048,16), to_unsigned( 2112,16), to_unsigned( 2178,16), to_unsigned( 2244,16), to_unsigned( 2312,16), to_unsigned( 2380,16), to_unsigned( 2450,16), to_unsigned( 2520,16),
        to_unsigned( 2592,16), to_unsigned( 2664,16), to_unsigned( 2738,16), to_unsigned( 2812,16), to_unsigned( 2888,16), to_unsigned( 2964,16), to_unsigned( 3042,16), to_unsigned( 3120,16),
        to_unsigned( 3200,16), to_unsigned( 3280,16), to_unsigned( 3362,16), to_unsigned( 3444,16), to_unsigned( 3528,16), to_unsigned( 3612,16), to_unsigned( 3698,16), to_unsigned( 3784,16),
        to_unsigned( 3872,16), to_unsigned( 3960,16), to_unsigned( 4050,16), to_unsigned( 4140,16), to_unsigned( 4232,16), to_unsigned( 4324,16), to_unsigned( 4418,16), to_unsigned( 4512,16),
        to_unsigned( 4608,16), to_unsigned( 4704,16), to_unsigned( 4802,16), to_unsigned( 4900,16), to_unsigned( 5000,16), to_unsigned( 5100,16), to_unsigned( 5202,16), to_unsigned( 5304,16),
        to_unsigned( 5408,16), to_unsigned( 5512,16), to_unsigned( 5618,16), to_unsigned( 5724,16), to_unsigned( 5832,16), to_unsigned( 5940,16), to_unsigned( 6050,16), to_unsigned( 6160,16),
        to_unsigned( 6272,16), to_unsigned( 6384,16), to_unsigned( 6498,16), to_unsigned( 6612,16), to_unsigned( 6728,16), to_unsigned( 6844,16), to_unsigned( 6962,16), to_unsigned( 7080,16),
        to_unsigned( 7200,16), to_unsigned( 7320,16), to_unsigned( 7442,16), to_unsigned( 7564,16), to_unsigned( 7688,16), to_unsigned( 7812,16), to_unsigned( 7938,16), to_unsigned( 8064,16),
        to_unsigned( 8192,16), to_unsigned( 8320,16), to_unsigned( 8450,16), to_unsigned( 8580,16), to_unsigned( 8712,16), to_unsigned( 8844,16), to_unsigned( 8978,16), to_unsigned( 9112,16),
        to_unsigned( 9248,16), to_unsigned( 9384,16), to_unsigned( 9522,16), to_unsigned( 9660,16), to_unsigned( 9800,16), to_unsigned( 9940,16), to_unsigned(10082,16), to_unsigned(10224,16),
        to_unsigned(10368,16), to_unsigned(10512,16), to_unsigned(10658,16), to_unsigned(10804,16), to_unsigned(10952,16), to_unsigned(11100,16), to_unsigned(11250,16), to_unsigned(11400,16),
        to_unsigned(11552,16), to_unsigned(11704,16), to_unsigned(11858,16), to_unsigned(12012,16), to_unsigned(12168,16), to_unsigned(12324,16), to_unsigned(12482,16), to_unsigned(12640,16),
        to_unsigned(12800,16), to_unsigned(12960,16), to_unsigned(13122,16), to_unsigned(13284,16), to_unsigned(13448,16), to_unsigned(13612,16), to_unsigned(13778,16), to_unsigned(13944,16),
        to_unsigned(14112,16), to_unsigned(14280,16), to_unsigned(14450,16), to_unsigned(14620,16), to_unsigned(14792,16), to_unsigned(14964,16), to_unsigned(15138,16), to_unsigned(15312,16),
        to_unsigned(15488,16), to_unsigned(15664,16), to_unsigned(15842,16), to_unsigned(16020,16), to_unsigned(16200,16), to_unsigned(16380,16), to_unsigned(16562,16), to_unsigned(16744,16),
        to_unsigned(16928,16), to_unsigned(17112,16), to_unsigned(17298,16), to_unsigned(17484,16), to_unsigned(17672,16), to_unsigned(17860,16), to_unsigned(18050,16), to_unsigned(18240,16),
        to_unsigned(18432,16), to_unsigned(18624,16), to_unsigned(18818,16), to_unsigned(19012,16), to_unsigned(19208,16), to_unsigned(19404,16), to_unsigned(19602,16), to_unsigned(19800,16),
        to_unsigned(20000,16), to_unsigned(20200,16), to_unsigned(20402,16), to_unsigned(20604,16), to_unsigned(20808,16), to_unsigned(21012,16), to_unsigned(21218,16), to_unsigned(21424,16),
        to_unsigned(21632,16), to_unsigned(21840,16), to_unsigned(22050,16), to_unsigned(22260,16), to_unsigned(22472,16), to_unsigned(22684,16), to_unsigned(22898,16), to_unsigned(23112,16),
        to_unsigned(23328,16), to_unsigned(23544,16), to_unsigned(23762,16), to_unsigned(23980,16), to_unsigned(24200,16), to_unsigned(24420,16), to_unsigned(24642,16), to_unsigned(24864,16),
        to_unsigned(25088,16), to_unsigned(25312,16), to_unsigned(25538,16), to_unsigned(25764,16), to_unsigned(25992,16), to_unsigned(26220,16), to_unsigned(26450,16), to_unsigned(26680,16),
        to_unsigned(26912,16), to_unsigned(27144,16), to_unsigned(27378,16), to_unsigned(27612,16), to_unsigned(27848,16), to_unsigned(28084,16), to_unsigned(28322,16), to_unsigned(28560,16),
        to_unsigned(28800,16), to_unsigned(29040,16), to_unsigned(29282,16), to_unsigned(29524,16), to_unsigned(29768,16), to_unsigned(30012,16), to_unsigned(30258,16), to_unsigned(30504,16),
        to_unsigned(30752,16), to_unsigned(31000,16), to_unsigned(31250,16), to_unsigned(31500,16), to_unsigned(31752,16), to_unsigned(32004,16), to_unsigned(32258,16), to_unsigned(32512,16) );


    -- linear Q8 light factor -> video Q8
    constant C_GAM : t_rom8 := (
        to_unsigned(    0,8), to_unsigned(   21,8), to_unsigned(   28,8), to_unsigned(   34,8), to_unsigned(   39,8), to_unsigned(   43,8), to_unsigned(   46,8), to_unsigned(   50,8), to_unsigned(   53,8), to_unsigned(   56,8), to_unsigned(   59,8), to_unsigned(   61,8),
        to_unsigned(   64,8), to_unsigned(   66,8), to_unsigned(   68,8), to_unsigned(   70,8), to_unsigned(   72,8), to_unsigned(   74,8), to_unsigned(   76,8), to_unsigned(   78,8), to_unsigned(   80,8), to_unsigned(   82,8), to_unsigned(   84,8), to_unsigned(   85,8),
        to_unsigned(   87,8), to_unsigned(   89,8), to_unsigned(   90,8), to_unsigned(   92,8), to_unsigned(   93,8), to_unsigned(   95,8), to_unsigned(   96,8), to_unsigned(   98,8), to_unsigned(   99,8), to_unsigned(  101,8), to_unsigned(  102,8), to_unsigned(  103,8),
        to_unsigned(  105,8), to_unsigned(  106,8), to_unsigned(  107,8), to_unsigned(  109,8), to_unsigned(  110,8), to_unsigned(  111,8), to_unsigned(  112,8), to_unsigned(  114,8), to_unsigned(  115,8), to_unsigned(  116,8), to_unsigned(  117,8), to_unsigned(  118,8),
        to_unsigned(  119,8), to_unsigned(  120,8), to_unsigned(  122,8), to_unsigned(  123,8), to_unsigned(  124,8), to_unsigned(  125,8), to_unsigned(  126,8), to_unsigned(  127,8), to_unsigned(  128,8), to_unsigned(  129,8), to_unsigned(  130,8), to_unsigned(  131,8),
        to_unsigned(  132,8), to_unsigned(  133,8), to_unsigned(  134,8), to_unsigned(  135,8), to_unsigned(  136,8), to_unsigned(  137,8), to_unsigned(  138,8), to_unsigned(  139,8), to_unsigned(  140,8), to_unsigned(  141,8), to_unsigned(  142,8), to_unsigned(  143,8),
        to_unsigned(  144,8), to_unsigned(  144,8), to_unsigned(  145,8), to_unsigned(  146,8), to_unsigned(  147,8), to_unsigned(  148,8), to_unsigned(  149,8), to_unsigned(  150,8), to_unsigned(  151,8), to_unsigned(  151,8), to_unsigned(  152,8), to_unsigned(  153,8),
        to_unsigned(  154,8), to_unsigned(  155,8), to_unsigned(  156,8), to_unsigned(  156,8), to_unsigned(  157,8), to_unsigned(  158,8), to_unsigned(  159,8), to_unsigned(  160,8), to_unsigned(  160,8), to_unsigned(  161,8), to_unsigned(  162,8), to_unsigned(  163,8),
        to_unsigned(  164,8), to_unsigned(  164,8), to_unsigned(  165,8), to_unsigned(  166,8), to_unsigned(  167,8), to_unsigned(  167,8), to_unsigned(  168,8), to_unsigned(  169,8), to_unsigned(  170,8), to_unsigned(  170,8), to_unsigned(  171,8), to_unsigned(  172,8),
        to_unsigned(  173,8), to_unsigned(  173,8), to_unsigned(  174,8), to_unsigned(  175,8), to_unsigned(  175,8), to_unsigned(  176,8), to_unsigned(  177,8), to_unsigned(  178,8), to_unsigned(  178,8), to_unsigned(  179,8), to_unsigned(  180,8), to_unsigned(  180,8),
        to_unsigned(  181,8), to_unsigned(  182,8), to_unsigned(  182,8), to_unsigned(  183,8), to_unsigned(  184,8), to_unsigned(  184,8), to_unsigned(  185,8), to_unsigned(  186,8), to_unsigned(  186,8), to_unsigned(  187,8), to_unsigned(  188,8), to_unsigned(  188,8),
        to_unsigned(  189,8), to_unsigned(  190,8), to_unsigned(  190,8), to_unsigned(  191,8), to_unsigned(  192,8), to_unsigned(  192,8), to_unsigned(  193,8), to_unsigned(  194,8), to_unsigned(  194,8), to_unsigned(  195,8), to_unsigned(  195,8), to_unsigned(  196,8),
        to_unsigned(  197,8), to_unsigned(  197,8), to_unsigned(  198,8), to_unsigned(  199,8), to_unsigned(  199,8), to_unsigned(  200,8), to_unsigned(  200,8), to_unsigned(  201,8), to_unsigned(  202,8), to_unsigned(  202,8), to_unsigned(  203,8), to_unsigned(  203,8),
        to_unsigned(  204,8), to_unsigned(  205,8), to_unsigned(  205,8), to_unsigned(  206,8), to_unsigned(  206,8), to_unsigned(  207,8), to_unsigned(  207,8), to_unsigned(  208,8), to_unsigned(  209,8), to_unsigned(  209,8), to_unsigned(  210,8), to_unsigned(  210,8),
        to_unsigned(  211,8), to_unsigned(  212,8), to_unsigned(  212,8), to_unsigned(  213,8), to_unsigned(  213,8), to_unsigned(  214,8), to_unsigned(  214,8), to_unsigned(  215,8), to_unsigned(  215,8), to_unsigned(  216,8), to_unsigned(  217,8), to_unsigned(  217,8),
        to_unsigned(  218,8), to_unsigned(  218,8), to_unsigned(  219,8), to_unsigned(  219,8), to_unsigned(  220,8), to_unsigned(  220,8), to_unsigned(  221,8), to_unsigned(  221,8), to_unsigned(  222,8), to_unsigned(  223,8), to_unsigned(  223,8), to_unsigned(  224,8),
        to_unsigned(  224,8), to_unsigned(  225,8), to_unsigned(  225,8), to_unsigned(  226,8), to_unsigned(  226,8), to_unsigned(  227,8), to_unsigned(  227,8), to_unsigned(  228,8), to_unsigned(  228,8), to_unsigned(  229,8), to_unsigned(  229,8), to_unsigned(  230,8),
        to_unsigned(  230,8), to_unsigned(  231,8), to_unsigned(  231,8), to_unsigned(  232,8), to_unsigned(  232,8), to_unsigned(  233,8), to_unsigned(  233,8), to_unsigned(  234,8), to_unsigned(  234,8), to_unsigned(  235,8), to_unsigned(  235,8), to_unsigned(  236,8),
        to_unsigned(  236,8), to_unsigned(  237,8), to_unsigned(  237,8), to_unsigned(  238,8), to_unsigned(  238,8), to_unsigned(  239,8), to_unsigned(  239,8), to_unsigned(  240,8), to_unsigned(  240,8), to_unsigned(  241,8), to_unsigned(  241,8), to_unsigned(  242,8),
        to_unsigned(  242,8), to_unsigned(  243,8), to_unsigned(  243,8), to_unsigned(  244,8), to_unsigned(  244,8), to_unsigned(  245,8), to_unsigned(  245,8), to_unsigned(  246,8), to_unsigned(  246,8), to_unsigned(  247,8), to_unsigned(  247,8), to_unsigned(  248,8),
        to_unsigned(  248,8), to_unsigned(  249,8), to_unsigned(  249,8), to_unsigned(  249,8), to_unsigned(  250,8), to_unsigned(  250,8), to_unsigned(  251,8), to_unsigned(  251,8), to_unsigned(  252,8), to_unsigned(  252,8), to_unsigned(  253,8), to_unsigned(  253,8),
        to_unsigned(  254,8), to_unsigned(  254,8), to_unsigned(  255,8), to_unsigned(  255,8) );
    ----------------------------------------------------------------------
    -- face tables (single chunk).  Face axes as +/- axis codes 0..5 =
    -- +x,-x,+y,-y,+z,-z.  n = Uv x Vv (matches the python model FACES).
    ----------------------------------------------------------------------
    type t_i6 is array (0 to 5) of integer range 0 to 5;
    constant C_FN : t_i6 := (2, 3, 0, 1, 4, 5);   -- normal axis code
    constant C_FU : t_i6 := (4, 0, 2, 4, 0, 2);   -- U axis code
    constant C_FV : t_i6 := (0, 4, 4, 2, 2, 0);   -- V axis code
    -- cube corner index = sx + 2*sy + 4*sz (sign bits, 1 = +1.5).  Corner
    -- order per face: P0=(u0,v0) P1=(u3,v0) P2=(u3,v3) P3=(u0,v3).
    type t_fc is array (0 to 5, 0 to 3) of integer range 0 to 7;
    constant C_FCORN : t_fc := (
        (2, 6, 7, 3),    -- U (+y): O=(-x,+y,-z), +U=+z, +V=+x
        (0, 1, 5, 4),    -- D (-y): O=(-x,-y,-z), +U=+x, +V=+z
        (1, 3, 7, 5),    -- R (+x): O=(+x,-y,-z), +U=+y, +V=+z
        (0, 4, 6, 2),    -- L (-x): O=(-x,-y,-z), +U=+z, +V=+y
        (4, 5, 7, 6),    -- F (+z): O=(-x,-y,+z), +U=+x, +V=+y
        (1, 0, 2, 3) );  -- B (-z): O=(+x,-y,-z), +U=+y, +V=+x
    -- neighbour face across each edge [u=0, u=3, v=0, v=3] (for sil flags)
    type t_fa is array (0 to 5, 0 to 3) of integer range 0 to 5;
    constant C_FADJ : t_fa := (
        (5, 4, 3, 2),    -- U: -U=-z=B, +U=F, -V=-x=L, +V=R
        (3, 2, 5, 4),    -- D: -U=L, +U=R, -V=B, +V=F
        (1, 0, 5, 4),    -- R: -U=D, +U=U, -V=B, +V=F
        (5, 4, 1, 0),    -- L: -U=B, +U=F, -V=D, +V=U
        (3, 2, 1, 0),    -- F: -U=L, +U=R, -V=D, +V=U
        (1, 0, 3, 2) );  -- B: -U=D, +U=U, -V=L, +V=R

    ----------------------------------------------------------------------
    -- raster measurement / control latch
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_sawact     : std_logic := '0';
    signal s_fstart     : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_avid_r     : std_logic := '0';   -- avid rising strobe
    signal s_avid_f     : std_logic := '0';   -- avid falling strobe
    signal s_hs_q       : std_logic := '0';
    signal s_hs_r       : std_logic := '0';   -- hsync (active low) leading strobe
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_ycnt       : unsigned(10 downto 0) := (others => '0');  -- active line ctr
    signal s_H          : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_ilace      : std_logic := '0';
    signal s_field      : std_logic := '0';   -- latched field_n at vsync
    -- resolution scaling (v0.5.6), decoded from the timing ID every clock:
    -- interlace (core_top ties field_n high, so a field_n toggle never
    -- shows on hardware) and the SD pixel-aspect x correction, Q12 offset
    -- from 1.0 (0 at HD; +455 = x1.111 on 480 lines, -256 = x0.9375 on 576)
    signal s_tid_il     : std_logic := '0';
    signal s_xa         : signed(10 downto 0) := (others => '0');

    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sw   : std_logic_vector(4 downto 0) := "00001";
    signal s_sld  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_estart, s_earm : std_logic := '0';
    signal s_idle : unsigned(12 downto 0) := (others => '0');
    -- line width latched for the field: the DDAs step exactly this many
    -- times per line and the engine's line wrap uses the same value
    signal s_Wl   : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal dd_x   : unsigned(11 downto 0) := (others => '0');
    signal dd_run, dd_wrap : std_logic := '0';
    -- generated avid (see p_measure): parity-locked start, ends with the DDA
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avid_q : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := "1000";

    ----------------------------------------------------------------------
    -- FRAME FSM
    ----------------------------------------------------------------------
    signal fr_st   : unsigned(7 downto 0) := (others => '0');  -- state
    signal fr_done : std_logic := '0';

    -- glide-smoothed angles (12b) + targets
    signal an_yaw, an_pit, an_rol : unsigned(11 downto 0) := (others => '0');

    -- ONE multiplier + ONE divider pair, shared by the frame and line FSMs
    -- (operands muxed on fr_done in p_dmath).  2-cycle multiply latency.
    signal fm_a  : signed(17 downto 0) := (others => '0');
    signal fm_b  : signed(13 downto 0) := (others => '0');
    signal lm_a  : signed(17 downto 0) := (others => '0');
    signal lm_b  : signed(13 downto 0) := (others => '0');
    signal mu_p  : signed(31 downto 0) := (others => '0');
    -- serial multiplier state; mu_idle means 'no multiply outstanding',
    -- which is what every call site waits on before reading mu_p
    signal mu_go_f, mu_go_l : std_logic := '0';
    signal mu_bsy : std_logic := '0';
    signal mu_ng  : std_logic := '0';
    signal mu_sa  : signed(31 downto 0) := (others => '0');
    signal mu_sb  : unsigned(13 downto 0) := (others => '0');
    signal mu_ac  : signed(31 downto 0) := (others => '0');
    signal mu_cnt : unsigned(3 downto 0) := (others => '0');
    signal mu_idle : std_logic := '1';
    -- right-shifting serial multiplier: the addend (sign already folded in)
    -- holds still and the product shifts down past it, so the adder is 20
    -- bits instead of 32 and there is no output negation
    signal mu_pa : signed(19 downto 0) := (others => '0');
    signal mu_ph : signed(21 downto 0) := (others => '0');
    signal mu_bp : std_logic := '0';
    signal mu_pl : unsigned(15 downto 0) := (others => '0');

    signal dv_na : unsigned(33 downto 0) := (others => '0');
    signal dv_da : unsigned(20 downto 0) := (others => '1');
    signal dv_q  : unsigned(33 downto 0) := (others => '0');
    signal dv_ra : unsigned(21 downto 0) := (others => '0');
    signal dv_cnt : unsigned(5 downto 0) := (others => '0');
    signal dv_bsy : std_logic := '0';
    signal fd_sgn : std_logic := '0';
    -- frame-side request (30/17, right-aligned into the 38-bit divider)
    signal frd_ns : unsigned(31 downto 0) := (others => '0');
    signal frd_ds : unsigned(20 downto 0) := (others => '1');
    signal frd_start : std_logic := '0';

    -- sin/cos scratch (Q12) for the three angles
    signal t_sinA, t_cosA : signed(13 downto 0) := (others => '0');
    signal cs_yaw_s, cs_yaw_c : signed(13 downto 0) := (others => '0');
    signal cs_pit_s, cs_pit_c : signed(13 downto 0) := (others => '0');
    signal cs_rol_s, cs_rol_c : signed(13 downto 0) := (others => '0');

    -- FRAM: per-frame vertex/basis scratch EBR (one read + one write port;
    -- addressing replaces every register-array mux the old FSM had).
    -- Map: 0..8 basis | 18+c*8+k corners | 42+k rx | 50+k ry | 58+k sx | 66+k sy
    signal fr_ra, fr_wa : unsigned(6 downto 0) := (others => '0');
    signal fr_rd2 : std_logic_vector(15 downto 0) := (others => '0');
    signal fr_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal fr_we  : std_logic := '0';
    -- shared quarter-sine ROM port (single instance)
    signal t_sk   : unsigned(7 downto 0) := (others => '0');
    signal t_srd  : signed(13 downto 0) := (others => '0');
    signal t_sneg : std_logic := '0';
    signal t_ang  : unsigned(11 downto 0) := (others => '0');
    signal t_s1   : signed(13 downto 0) := (others => '0');
    -- frame datapath scalars
    signal opA, opB : signed(15 downto 0) := (others => '0');
    signal coX, coY, coZ : signed(16 downto 0) := (others => '0');  -- C-O, Q11
    signal uvc0, uvc1, uvc2 : signed(15 downto 0) := (others => '0');
    signal vvc0, vvc1, vvc2 : signed(15 downto 0) := (others => '0');
    signal t_cacc : signed(17 downto 0) := (others => '0');
    signal bz0, bz1, bz2 : signed(15 downto 0) := (others => '0');
    signal t_ex, t_ey : unsigned(16 downto 0) := (others => '0');
    signal t_px0, t_py0 : signed(15 downto 0) := (others => '0');
    signal t_dx1, t_dx2, t_dy1, t_dy2 : signed(12 downto 0) := (others => '0');
    signal t_det : signed(25 downto 0) := (others => '0');
    signal t_sxa, t_sya : signed(15 downto 0) := (others => '0');
    signal t_dxe : signed(17 downto 0) := (others => '0');
    -- per-slot affine UV gradients (Q8 units per screen pixel) + face origin
    type t_sl3s17 is array (0 to 2) of signed(16 downto 0);
    type t_sl3s13 is array (0 to 2) of signed(12 downto 0);
    signal sl_gux, sl_guy, sl_gvx, sl_gvy : t_sl3s17 := (others => (others => '0'));
    signal sl_px0, sl_py0 : t_sl3s13 := (others => (others => '0'));

    signal f_zoom  : unsigned(11 downto 0) := to_unsigned(256, 12);  -- focal, px
    signal f_zsm   : unsigned(15 downto 0) := to_unsigned(256*16, 16); -- smoothed Q4
    signal s_zfirst : std_logic := '1';
    signal s_frans  : std_logic := '0';   -- a frame has started
    signal hmax, wmax : unsigned(14 downto 0) := (others => '0');

    -- per-slot face meta for the pixel path (written by frame FSM)
    type t_sl3u8 is array (0 to 2) of unsigned(7 downto 0);
    type t_sl3u4 is array (0 to 2) of std_logic_vector(3 downto 0);
    type t_sl3u3 is array (0 to 2) of unsigned(2 downto 0);
    signal sl_face : t_sl3u3 := (others => (others => '0'));
    signal sl_sil  : t_sl3u4 := (others => (others => '0'));
    type t_sl3u10 is array (0 to 2) of unsigned(9 downto 0);
    signal sl_cu   : t_sl3u10 := (others => to_unsigned(512, 10));
    signal sl_cv   : t_sl3u10 := (others => to_unsigned(512, 10));
    -- UNLIT sticker albedo (video luma).  The plastic albedo is the same on
    -- every face, so it stays a constant; the light itself now arrives per
    -- pixel through the gamma ROM, because the bevels shade.
    signal sl_ly   : t_sl3u10 := (others => (others => '0'));
    -- flat face light, LINEAR Q8 (ambient + key + fill against the face normal)
    signal sl_lf   : t_sl3u8 := (others => (others => '0'));
    -- half-vector H projected on the face basis, Q8: H.n, H.U, H.V.  With an
    -- orthographic camera both L and V are world constants, so H is too, and
    -- these are just the three whole-cube dot products picked and signed per
    -- face by orthonormality.
    type t_sl3s10 is array (0 to 2) of signed(9 downto 0);
    signal sl_hn, sl_hu, sl_hv : t_sl3s10 := (others => (others => '0'));
    signal s_pxs   : unsigned(7 downto 0) := to_unsigned(64, 8);  -- AA slope Q8
    -- AA slope as mantissa (4..7 encoded 0..3) and shift, so stage H needs
    -- only a shift-add tree
    signal s_pxm   : unsigned(1 downto 0) := (others => '0');
    signal s_pxe   : integer range 0 to 5 := 3;   -- actual shift = 3 + s_pxe
    signal s_nslot : unsigned(2 downto 0) := (others => '0');
    -- per-slot face extent in cubies (1..3), for the stage-A limits
    type t_sl6u2 is array (0 to 5) of unsigned(1 downto 0);
    signal sl_nu, sl_nv : t_sl6u2 := (others => "11");

    -- frame FSM loop counters / scratch
    signal fr_i    : unsigned(3 downto 0) := (others => '0');
    signal fr_j    : unsigned(2 downto 0) := (others => '0');
    signal fr_w    : unsigned(1 downto 0) := (others => '0');
    signal t_s0    : signed(13 downto 0) := (others => '0');
    signal t_zv    : unsigned(16 downto 0) := (others => '1');
    signal t_ac0   : signed(28 downto 0) := (others => '0');
    signal t_ac1   : signed(28 downto 0) := (others => '0');
    signal f_y12   : unsigned(11 downto 0) := (others => '0');
    signal s_fuse  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_hf    : unsigned(12 downto 0) := to_unsigned(480, 13);  -- frame height
    signal s_cx    : unsigned(11 downto 0) := to_unsigned(360, 12);
    signal s_cy    : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal s_fvis  : std_logic_vector(5 downto 0) := (others => '0');
    signal fr_c    : unsigned(3 downto 0) := (others => '0');
    signal t_ymin, t_ymax : signed(15 downto 0) := (others => '0');
    signal t_lacc  : signed(15 downto 0) := (others => '0');
    signal t_lf    : unsigned(7 downto 0) := (others => '0');
    signal fr_t0, fr_t1, fr_t2, fr_t3 : signed(23 downto 0) := (others => '0');
    signal fr_ax, fr_ay, fr_az : signed(23 downto 0) := (others => '0');
    signal fr_bx, fr_by, fr_bz : signed(23 downto 0) := (others => '0');
    signal fr_nx, fr_ny, fr_nz : signed(23 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- GEOMETRY EBR (256x16): frame FSM writes, line FSM reads.
    -- Layout per slot (base = slot*32):
    --   +0  ymin  +1 ymax
    --   +2..+13  four edges x (y0, x0, slope)  (x0 Q11.2, slope Q8 signed)
    --   +14..+22 Ax,Ay,Cu, Bx,By,Cv, Nx,Ny,Cw  (block-float s16)
    --   +23 flags: [11:8] sil, [2:0] face id
    -- word 127: number of valid slots
    ----------------------------------------------------------------------
    type t_gram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal gram : t_gram := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of gram : signal is "block";
    signal g_wa : unsigned(7 downto 0) := (others => '0');
    signal g_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal g_we : std_logic := '0';
    signal g_ra : unsigned(7 downto 0) := (others => '0');
    signal g_rd : std_logic_vector(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- DISPLAY LIST EBR (256x16): line FSM writes, pixel path reads.
    -- 5-word records (x/type, u0, v0, du, dv); bank = target line parity.
    ----------------------------------------------------------------------
    type t_dram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal dram : t_dram := (others => (others => '0'));
    attribute ram_style of dram : signal is "block";
    signal d_wa : unsigned(7 downto 0) := (others => '0');
    signal d_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal d_we : std_logic := '0';
    signal d_ra : unsigned(7 downto 0) := (others => '0');
    signal d_rd : std_logic_vector(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- PER-FACE AFFINE DDAs (replace the line engine + display list)
    -- wram: 0..11 wrap step, 16..27 field seed, 32..43 gx  (index 2*slot+coord)
    ----------------------------------------------------------------------
    type t_acc12 is array (0 to 11) of signed(26 downto 0);
    signal dd_acc : t_acc12 := (others => (others => '0'));
    signal dd_gx  : t_acc12 := (others => (others => '0'));
    signal dd_e   : std_logic_vector(11 downto 0) := (others => '0');
    signal dd_kick : std_logic := '0';
    signal dq_run, dq_seed : std_logic := '0';
    signal dq_v1, dq_v2, dq_k1, dq_k2, dq_st : std_logic := '0';
    signal dq_t : unsigned(4 downto 0) := (others => '0');
    signal dq_a1, dq_a2, dq_a3 : unsigned(3 downto 0) := (others => '0');
    type t_wram is array (0 to 255) of std_logic_vector(31 downto 0);
    signal wram : t_wram := (others => (others => '0'));
    attribute ram_style of wram : signal is "block";
    signal w_wa, w_ra : unsigned(7 downto 0) := (others => '0');
    signal w_wd, w_rd : std_logic_vector(31 downto 0) := (others => '0');
    signal w_we : std_logic := '0';
    signal a_cr, a_cp : std_logic_vector(5 downto 0) := (others => '0');
    signal a_own : unsigned(2 downto 0) := (others => '0');   -- owner slot
    signal a_any, a_bh : std_logic := '0';

    ----------------------------------------------------------------------
    -- per-slot descriptors in block RAM, each read at the stage that uses
    -- it (3 live words of 256).  As register files they cost a flip-flop
    -- per bit per slot plus a 3:1 mux per read, ~300 cells in all.
    --   sd_h1: hu(10) & hn(5:0)    sd_h2: hv(10) & hn(9:6)     (stage E)
    --   sd_lf: ly>>2 (8) & lf (8)                                (stage G)
    --   sd_cv: cv>>2 (8) & cu>>2 (8)                             (stage L)
    ----------------------------------------------------------------------
    type t_sd is array (0 to 255) of std_logic_vector(15 downto 0);
    -- generated by regen_ucode.py: sticker state + turn tables, and sticker
    -- chroma by (light level, colour)
    constant C_TABLE : t_sd := (
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002",
        x"0002", x"0002", x"0002", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003",
        x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", x"0005", x"0005", x"0005",
        x"0005", x"0005", x"0005", x"0005", x"0005", x"0005", x"0000", x"0000", x"0000", x"0002", x"0008", x"0006",
        x"0001", x"0005", x"0007", x"0003", x"0014", x"002F", x"0023", x"002C", x"0017", x"0032", x"0022", x"002B",
        x"001A", x"0035", x"0021", x"002A", x"0009", x"000F", x"0011", x"000B", x"000A", x"000C", x"0010", x"000E",
        x"0012", x"002D", x"001D", x"0026", x"0015", x"0030", x"001C", x"0025", x"0018", x"0033", x"001B", x"0024",
        x"0006", x"002C", x"0011", x"0033", x"0007", x"0029", x"000E", x"0034", x"0008", x"0026", x"000B", x"0035",
        x"0012", x"0014", x"001A", x"0018", x"0013", x"0017", x"0019", x"0015", x"0000", x"002A", x"000F", x"002D",
        x"0001", x"0027", x"000C", x"002E", x"0002", x"0024", x"0009", x"002F", x"001B", x"0021", x"0023", x"001D",
        x"001C", x"001E", x"0022", x"0020", x"0002", x"001D", x"0011", x"001A", x"0005", x"0020", x"0010", x"0019",
        x"0008", x"0023", x"000F", x"0018", x"0024", x"0026", x"002C", x"002A", x"0025", x"0029", x"002B", x"0027",
        x"0000", x"001B", x"000B", x"0014", x"0003", x"001E", x"000A", x"0013", x"0006", x"0021", x"0009", x"0012",
        x"002D", x"0033", x"0035", x"002F", x"002E", x"0030", x"0034", x"0032", x"0000", x"003F", x"0014", x"0021",
        x"002A", x"002F", x"003F", x"0009", x"0012", x"001B", x"0024", x"002D", x"003F", x"0009", x"0012", x"001B",
        x"0024", x"002D", x"0000", x"003F", x"0013", x"001E", x"0027", x"002E", x"0006", x"000B", x"0012", x"003F",
        x"0026", x"0033", x"0000", x"0009", x"003F", x"001B", x"0024", x"002D", x"0000", x"0009", x"003F", x"001B",
        x"0024", x"002D", x"0003", x"000A", x"0012", x"003F", x"0025", x"0030", x"0002", x"000F", x"0018", x"001D",
        x"0024", x"003F", x"0000", x"0009", x"0012", x"001B", x"003F", x"002D", x"0000", x"0009", x"0012", x"001B",
        x"003F", x"002D", x"0001", x"000C", x"0015", x"001C", x"0024", x"003F", x"04E5", x"0953", x"0941", x"0065",
        x"0053", x"04C1", x"0000", x"0000" );
    constant C_CHROMA : t_sd := (
        x"8080", x"963C", x"AF77", x"B555", x"547D", x"62A6", x"8080", x"8080", x"8080", x"963B", x"AF77", x"B654",
        x"537D", x"62A7", x"8080", x"8080", x"8080", x"973A", x"B077", x"B753", x"527D", x"61A8", x"8080", x"8080",
        x"8080", x"9738", x"B177", x"B852", x"517D", x"61A8", x"8080", x"8080", x"8080", x"9737", x"B277", x"B952",
        x"507D", x"60A9", x"8080", x"8080", x"8080", x"9836", x"B376", x"BA51", x"507D", x"60AA", x"8080", x"8080",
        x"8080", x"9835", x"B476", x"BB50", x"4F7D", x"5FAB", x"8080", x"8080", x"8080", x"9933", x"B576", x"BC4F",
        x"4E7D", x"5EAB", x"8080", x"8080", x"8080", x"9932", x"B676", x"BD4F", x"4D7D", x"5EAC", x"8080", x"8080",
        x"8080", x"9931", x"B676", x"BE4E", x"4C7D", x"5DAD", x"8080", x"8080", x"8080", x"9A30", x"B776", x"BF4D",
        x"4B7C", x"5DAD", x"8080", x"8080", x"8080", x"9A2E", x"B876", x"C04C", x"4B7C", x"5CAE", x"8080", x"8080",
        x"8080", x"9B2D", x"B975", x"C14B", x"4A7C", x"5CAF", x"8080", x"8080", x"8080", x"9B2C", x"BA75", x"C24B",
        x"497C", x"5BB0", x"8080", x"8080", x"8080", x"9C2B", x"BB75", x"C34A", x"487C", x"5BB0", x"8080", x"8080",
        x"8080", x"9C29", x"BC75", x"C449", x"477C", x"5AB1", x"8080", x"8080", x"8080", x"9C28", x"BC75", x"C548",
        x"477C", x"5AB2", x"8080", x"8080", x"8080", x"9D27", x"BD75", x"C647", x"467C", x"59B2", x"8080", x"8080",
        x"8080", x"9D26", x"BE74", x"C747", x"457C", x"59B3", x"8080", x"8080", x"8080", x"9E24", x"BF74", x"C846",
        x"447C", x"58B4", x"8080", x"8080", x"8080", x"9E23", x"C074", x"C945", x"437C", x"57B5", x"8080", x"8080",
        x"8080", x"9E22", x"C174", x"CA44", x"437C", x"57B5", x"8080", x"8080", x"8080", x"9F21", x"C274", x"CB44",
        x"427C", x"56B6", x"8080", x"8080", x"8080", x"9F1F", x"C374", x"CC43", x"417C", x"56B7", x"8080", x"8080",
        x"8080", x"A01E", x"C374", x"CD42", x"407C", x"55B7", x"8080", x"8080", x"8080", x"A01D", x"C473", x"CE41",
        x"3F7C", x"55B8", x"8080", x"8080", x"8080", x"A01C", x"C573", x"CF40", x"3F7C", x"54B9", x"8080", x"8080",
        x"8080", x"A11A", x"C673", x"D040", x"3E7C", x"54BA", x"8080", x"8080", x"8080", x"A119", x"C773", x"D13F",
        x"3D7C", x"53BA", x"8080", x"8080", x"8080", x"A218", x"C873", x"D23E", x"3C7C", x"53BB", x"8080", x"8080",
        x"8080", x"A217", x"C973", x"D33D", x"3B7C", x"52BC", x"8080", x"8080", x"8080", x"A316", x"CA73", x"D43D",
        x"3B7C", x"52BD", x"8080", x"8080" );
    signal sd_h1, sd_h2, sd_lf, sd_fd : t_sd := (others => (others => '0'));
    attribute ram_style of sd_h1 : signal is "block";
    attribute ram_style of sd_h2 : signal is "block";
    attribute ram_style of sd_lf : signal is "block";
    attribute ram_style of sd_fd : signal is "block";
    signal sd_wa : unsigned(7 downto 0) := (others => '0');
    signal sd_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal sd_we : std_logic_vector(3 downto 0) := (others => '0');
    signal sd_fdd : std_logic_vector(15 downto 0) := x"FC00";
    -- sticker state + turn tables (engine reads/writes in blanking, the
    -- pixel path reads the state during active video)
    signal tbl : t_sd := C_TABLE;
    attribute ram_style of tbl : signal is "block";
    signal tb_era, tb_wa, px_ta : unsigned(7 downto 0) := (others => '0');
    signal tb_wd, tb_rd : std_logic_vector(15 downto 0) := (others => '0');
    signal tb_we : std_logic := '0';
    signal sd_h1d, sd_h2d, sd_lfd : std_logic_vector(15 downto 0) := (others => '0');
    signal sd_cvd : std_logic_vector(15 downto 0) := x"8080";
    -- K4 sticker sheen (x/4) and K5 video stickers (colour mask)
    signal s_vsel  : unsigned(2 downto 0) := (others => '0');
    signal x14_vid : std_logic := '0';
    signal g_hn : signed(9 downto 0) := (others => '0');
    signal i_ly, j_ly : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel pipe, stages E..O (x<N>_ = register written by stage N)
    ----------------------------------------------------------------------
    signal c_du, c_dv : unsigned(11 downto 0) := (others => '0');
    signal c_base : unsigned(5 downto 0) := (others => '1');
    signal c_gap  : unsigned(2 downto 0) := (others => '0');
    signal c_sil  : std_logic_vector(3 downto 0) := (others => '0');
    signal c_nu, c_nv : unsigned(1 downto 0) := "11";
    signal b_bh, c_bh, e_bh, f_bh, x5_bh, x6_bh, x7_bh, x8_bh : std_logic := '0';
    -- owned only through the 64-unit padding (no strict claimant)
    signal a_po, b_po, c_po : std_logic := '0';
    signal f_col, x5_col, x6_col, x7_col : unsigned(2 downto 0) := (others => '0');
    -- chroma ROM address (light level & colour), carried to stage N
    signal x8_ca, x9_ca, x10_ca, x11_ca, x12_ca, x13_ca : unsigned(2 downto 0) := (others => '0');
    signal c_qu, c_qv : signed(11 downto 0) := (others => '0');
    signal c_ngu, c_ngv : std_logic := '0';
    signal x5_cm1, x5_snu, x5_snv : unsigned(5 downto 0) := (others => '0');
    signal x5_hn : signed(9 downto 0) := (others => '0');
    signal x5_hn8, x5_hu, x5_hv : signed(7 downto 0) := (others => '0');
    signal x5_c19 : signed(19 downto 0) := (others => '0');
    signal x5_tmax : unsigned(7 downto 0) := (others => '0');
    signal x5_dst : signed(10 downto 0) := (others => '0');
    signal x5_dstr : signed(12 downto 0) := (others => '0');
    signal x5_cact, x5_2m, x5_silz, x5_stz, x5_stc, x5_on : std_logic := '0';
    signal x5_sl : unsigned(2 downto 0) := (others => '0');
    signal x6_p0, x6_p1, x6_p2 : signed(14 downto 0) := (others => '0');
    signal x6_dlin : signed(11 downto 0) := (others => '0');
    signal x6_ao, x6_tmax : unsigned(7 downto 0) := (others => '0');
    signal x6_hn : signed(9 downto 0) := (others => '0');
    signal x6_dst : signed(10 downto 0) := (others => '0');
    signal x6_dstr : signed(12 downto 0) := (others => '0');
    signal x6_cact, x6_2m, x6_silz, x6_stz, x6_stc, x6_on : std_logic := '0';
    signal x6_sl : unsigned(2 downto 0) := (others => '0');
    signal x7_del : signed(11 downto 0) := (others => '0');
    signal x7_dsil : signed(13 downto 0) := (others => '0');
    signal x7_ds : signed(10 downto 0) := (others => '0');
    signal x7_occ, x7_sed, x7_ly : unsigned(7 downto 0) := (others => '0');
    signal x7_ao2 : unsigned(6 downto 0) := (others => '0');
    signal x7_hn : signed(9 downto 0) := (others => '0');
    signal x7_lfa, x7_p1, x7_p2 : signed(9 downto 0) := (others => '0');
    signal x7_q : signed(10 downto 0) := (others => '0');
    signal x7_sl : unsigned(2 downto 0) := (others => '0');
    signal x7_on : std_logic := '0';
    -- unlit albedo by (sticker colour, sticker coverage): stage J's
    -- satin-to-sticker blend 173 + (dly(col) * ast) >> 4 as a block-RAM
    -- table instead of a multiply and an add (same values, same stages)
    type t_yrom is array (0 to 255) of unsigned(9 downto 0);
    constant C_YROM : t_yrom := (
        to_unsigned( 173,10), to_unsigned( 224,10), to_unsigned( 275,10), to_unsigned( 327,10), to_unsigned( 378,10), to_unsigned( 429,10), to_unsigned( 481,10), to_unsigned( 532,10),
        to_unsigned( 584,10), to_unsigned( 635,10), to_unsigned( 686,10), to_unsigned( 738,10), to_unsigned( 789,10), to_unsigned( 840,10), to_unsigned( 892,10), to_unsigned( 943,10),
        to_unsigned( 995,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10),
        to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10),
        to_unsigned( 173,10), to_unsigned( 212,10), to_unsigned( 252,10), to_unsigned( 291,10), to_unsigned( 331,10), to_unsigned( 371,10), to_unsigned( 410,10), to_unsigned( 450,10),
        to_unsigned( 490,10), to_unsigned( 529,10), to_unsigned( 569,10), to_unsigned( 608,10), to_unsigned( 648,10), to_unsigned( 688,10), to_unsigned( 727,10), to_unsigned( 767,10),
        to_unsigned( 807,10), to_unsigned( 846,10), to_unsigned( 886,10), to_unsigned( 925,10), to_unsigned( 965,10), to_unsigned(1005,10), to_unsigned(1023,10), to_unsigned(1023,10),
        to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10), to_unsigned(1023,10),
        to_unsigned( 173,10), to_unsigned( 182,10), to_unsigned( 192,10), to_unsigned( 202,10), to_unsigned( 212,10), to_unsigned( 222,10), to_unsigned( 232,10), to_unsigned( 242,10),
        to_unsigned( 252,10), to_unsigned( 262,10), to_unsigned( 272,10), to_unsigned( 282,10), to_unsigned( 292,10), to_unsigned( 302,10), to_unsigned( 312,10), to_unsigned( 322,10),
        to_unsigned( 332,10), to_unsigned( 341,10), to_unsigned( 351,10), to_unsigned( 361,10), to_unsigned( 371,10), to_unsigned( 381,10), to_unsigned( 391,10), to_unsigned( 401,10),
        to_unsigned( 411,10), to_unsigned( 421,10), to_unsigned( 431,10), to_unsigned( 441,10), to_unsigned( 451,10), to_unsigned( 461,10), to_unsigned( 471,10), to_unsigned( 481,10),
        to_unsigned( 173,10), to_unsigned( 194,10), to_unsigned( 215,10), to_unsigned( 236,10), to_unsigned( 258,10), to_unsigned( 279,10), to_unsigned( 300,10), to_unsigned( 321,10),
        to_unsigned( 343,10), to_unsigned( 364,10), to_unsigned( 385,10), to_unsigned( 406,10), to_unsigned( 428,10), to_unsigned( 449,10), to_unsigned( 470,10), to_unsigned( 491,10),
        to_unsigned( 513,10), to_unsigned( 534,10), to_unsigned( 555,10), to_unsigned( 576,10), to_unsigned( 598,10), to_unsigned( 619,10), to_unsigned( 640,10), to_unsigned( 661,10),
        to_unsigned( 683,10), to_unsigned( 704,10), to_unsigned( 725,10), to_unsigned( 746,10), to_unsigned( 768,10), to_unsigned( 789,10), to_unsigned( 810,10), to_unsigned( 831,10),
        to_unsigned( 173,10), to_unsigned( 188,10), to_unsigned( 203,10), to_unsigned( 218,10), to_unsigned( 233,10), to_unsigned( 248,10), to_unsigned( 264,10), to_unsigned( 279,10),
        to_unsigned( 294,10), to_unsigned( 309,10), to_unsigned( 324,10), to_unsigned( 340,10), to_unsigned( 355,10), to_unsigned( 370,10), to_unsigned( 385,10), to_unsigned( 400,10),
        to_unsigned( 416,10), to_unsigned( 431,10), to_unsigned( 446,10), to_unsigned( 461,10), to_unsigned( 476,10), to_unsigned( 491,10), to_unsigned( 507,10), to_unsigned( 522,10),
        to_unsigned( 537,10), to_unsigned( 552,10), to_unsigned( 567,10), to_unsigned( 583,10), to_unsigned( 598,10), to_unsigned( 613,10), to_unsigned( 628,10), to_unsigned( 643,10),
        to_unsigned( 173,10), to_unsigned( 179,10), to_unsigned( 185,10), to_unsigned( 192,10), to_unsigned( 198,10), to_unsigned( 205,10), to_unsigned( 211,10), to_unsigned( 218,10),
        to_unsigned( 224,10), to_unsigned( 230,10), to_unsigned( 237,10), to_unsigned( 243,10), to_unsigned( 250,10), to_unsigned( 256,10), to_unsigned( 263,10), to_unsigned( 269,10),
        to_unsigned( 276,10), to_unsigned( 282,10), to_unsigned( 288,10), to_unsigned( 295,10), to_unsigned( 301,10), to_unsigned( 308,10), to_unsigned( 314,10), to_unsigned( 321,10),
        to_unsigned( 327,10), to_unsigned( 333,10), to_unsigned( 340,10), to_unsigned( 346,10), to_unsigned( 353,10), to_unsigned( 359,10), to_unsigned( 366,10), to_unsigned( 372,10),
        to_unsigned( 173,10), to_unsigned( 179,10), to_unsigned( 185,10), to_unsigned( 192,10), to_unsigned( 198,10), to_unsigned( 205,10), to_unsigned( 211,10), to_unsigned( 218,10),
        to_unsigned( 224,10), to_unsigned( 230,10), to_unsigned( 237,10), to_unsigned( 243,10), to_unsigned( 250,10), to_unsigned( 256,10), to_unsigned( 263,10), to_unsigned( 269,10),
        to_unsigned( 276,10), to_unsigned( 282,10), to_unsigned( 288,10), to_unsigned( 295,10), to_unsigned( 301,10), to_unsigned( 308,10), to_unsigned( 314,10), to_unsigned( 321,10),
        to_unsigned( 327,10), to_unsigned( 333,10), to_unsigned( 340,10), to_unsigned( 346,10), to_unsigned( 353,10), to_unsigned( 359,10), to_unsigned( 366,10), to_unsigned( 372,10),
        to_unsigned( 173,10), to_unsigned( 179,10), to_unsigned( 185,10), to_unsigned( 192,10), to_unsigned( 198,10), to_unsigned( 205,10), to_unsigned( 211,10), to_unsigned( 218,10),
        to_unsigned( 224,10), to_unsigned( 230,10), to_unsigned( 237,10), to_unsigned( 243,10), to_unsigned( 250,10), to_unsigned( 256,10), to_unsigned( 263,10), to_unsigned( 269,10),
        to_unsigned( 276,10), to_unsigned( 282,10), to_unsigned( 288,10), to_unsigned( 295,10), to_unsigned( 301,10), to_unsigned( 308,10), to_unsigned( 314,10), to_unsigned( 321,10),
        to_unsigned( 327,10), to_unsigned( 333,10), to_unsigned( 340,10), to_unsigned( 346,10), to_unsigned( 353,10), to_unsigned( 359,10), to_unsigned( 366,10), to_unsigned( 372,10) );
    signal y_rd : unsigned(9 downto 0) := (others => '0');
    signal x8_ams, x8_amt : signed(16 downto 0) := (others => '0');
    signal x8_dly : signed(10 downto 0) := (others => '0');
    signal x8_sl : unsigned(2 downto 0) := (others => '0');
    signal x8_on : std_logic := '0';
    signal x9_asil, x9_ast : unsigned(4 downto 0) := (others => '0');
    signal x9_dly : signed(10 downto 0) := (others => '0');
    signal x9_sl : unsigned(2 downto 0) := (others => '0');
    signal x10_gam, x10_spl : unsigned(7 downto 0) := (others => '0');
    signal x10_dsp : signed(9 downto 0) := (others => '0');
    signal x10_py : signed(16 downto 0) := (others => '0');
    signal x10_ast, x10_asil : unsigned(4 downto 0) := (others => '0');
    signal x10_ston : std_logic := '0';
    signal x10_sl : unsigned(2 downto 0) := (others => '0');
    signal x11_y : unsigned(9 downto 0) := (others => '0');
    signal x11_ps : signed(15 downto 0) := (others => '0');
    signal x11_gam, x11_spl : unsigned(7 downto 0) := (others => '0');
    signal x11_asil : unsigned(4 downto 0) := (others => '0');
    signal x11_ston : std_logic := '0';
    signal x11_sl : unsigned(2 downto 0) := (others => '0');
    signal x12_pm : unsigned(17 downto 0) := (others => '0');
    signal x12_spec : unsigned(11 downto 0) := (others => '0');
    signal x12_asil : unsigned(4 downto 0) := (others => '0');
    signal x12_ston : std_logic := '0';
    signal x12_sl : unsigned(2 downto 0) := (others => '0');
    signal x13_dy : signed(10 downto 0) := (others => '0');
    signal x13_bg : unsigned(9 downto 0) := (others => '0');
    signal x13_asil : unsigned(4 downto 0) := (others => '0');
    signal x13_ston : std_logic := '0';
    signal x13_sl : unsigned(2 downto 0) := (others => '0');
    signal x14_pn : signed(16 downto 0) := (others => '0');
    signal x14_bg : unsigned(9 downto 0) := (others => '0');
    signal x14_asil : unsigned(4 downto 0) := (others => '0');
    signal x14_ston : std_logic := '0';

    ----------------------------------------------------------------------
    -- LINE FSM
    ----------------------------------------------------------------------
    signal ln_st   : unsigned(5 downto 0) := (others => '0');
    signal ln_busy : std_logic := '0';
    signal ln_y    : unsigned(11 downto 0) := (others => '0');   -- frame-space target y
    signal ln_ynext: unsigned(10 downto 0) := (others => '0');   -- active-line index
    signal ln_bank : std_logic := '0';

    -- line FSM working registers
    signal nslot_l : unsigned(1 downto 0) := (others => '0');
    type t_cy4 is array (0 to 3) of signed(15 downto 0);
    signal cy4 : t_cy4 := (others => (others => '0'));
    signal ln_yc2 : signed(15 downto 0) := (others => '0');
    signal d_wp  : unsigned(7 downto 0) := (others => '0');
    signal ord0, ord1, ord2 : unsigned(1 downto 0) := (others => '0');
    signal ocnt  : unsigned(1 downto 0) := (others => '0');
    signal ln_k  : unsigned(1 downto 0) := (others => '0');
    signal ln_xa, ln_xb : signed(13 downto 0) := (others => '0');
    signal ln_ua, ln_va, ln_ub, ln_vb : signed(15 downto 0) := (others => '0');
    signal ln_du2, ln_dv2 : signed(15 downto 0) := (others => '0');
    signal ln_pxl, ln_py : signed(13 downto 0) := (others => '0');
    signal ln_lg : integer range 6 to 7 := 6;
    signal ln_first2 : std_logic := '0';
    signal ln_w : unsigned(1 downto 0) := (others => '0');
    signal ln_slot : unsigned(1 downto 0) := (others => '0');
    signal ln_dxp, ln_dyp : signed(13 downto 0) := (others => '0');
    signal ln_acc, ln_acc2 : signed(31 downto 0) := (others => '0');
    signal ln_useed, ln_vseed : signed(15 downto 0) := (others => '0');
    -- per-face working set (loaded from geometry EBR)
    signal w_ymin, w_ymax : unsigned(11 downto 0) := (others => '0');
    signal w_ax, w_ay, w_cu : signed(15 downto 0) := (others => '0');
    signal w_bx, w_by, w_cv : signed(15 downto 0) := (others => '0');
    signal w_nx, w_ny, w_cw : signed(15 downto 0) := (others => '0');
    -- span table (3 spans: xl, xr, slot), then x-sorted emission
    type t_sp3 is array (0 to 2) of signed(13 downto 0);
    signal sp_xl, sp_xr : t_sp3 := (others => (others => '0'));
    signal sp_ok : std_logic_vector(2 downto 0) := (others => '0');
    signal ln_i  : unsigned(1 downto 0) := (others => '0');
    signal ln_e  : unsigned(2 downto 0) := (others => '0');
    signal ln_x0, ln_x1 : signed(13 downto 0) := (others => '0');
    signal ln_uw, ln_vw, ln_ww : signed(27 downto 0) := (others => '0');
    signal ln_dua, ln_dva, ln_dwa : signed(27 downto 0) := (others => '0');
    signal ln_un, ln_vn : signed(15 downto 0) := (others => '0');
    signal ln_up, ln_vp : signed(15 downto 0) := (others => '0');
    signal ln_xn : signed(13 downto 0) := (others => '0');
    signal ln_seg : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal ln_edge_y0 : unsigned(11 downto 0) := (others => '0');
    signal ln_edge_x0 : signed(15 downto 0) := (others => '0');
    signal ln_edge_sl : signed(15 downto 0) := (others => '0');
    signal ln_xmin, ln_xmax : signed(15 downto 0) := (others => '0');
    signal ln_t0 : signed(15 downto 0) := (others => '0');
    signal sp_msk : std_logic_vector(2 downto 0) := (others => '0');
    signal ln_ys : signed(15 downto 0) := (others => '0');
    signal ln_have : std_logic := '0';
    signal ln_first : std_logic := '0';

    ----------------------------------------------------------------------
    -- PIXEL PATH
    ----------------------------------------------------------------------
    -- record prefetcher
    signal pf_st  : unsigned(2 downto 0) := (others => '0');
    signal nx_x   : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal nx_u, nx_v : signed(15 downto 0) := (others => '0');
    signal nx_xe : unsigned(11 downto 0) := (others => '0');
    signal nx_sl  : unsigned(1 downto 0) := (others => '0');
    signal nx_end : std_logic := '1';
    signal uacc, vacc : signed(25 downto 0) := (others => '0');
    signal cu_du, cu_dv : signed(25 downto 0) := (others => '0');
    signal cu_xe  : unsigned(11 downto 0) := (others => '0');
    signal cu_sl  : unsigned(1 downto 0) := (others => '0');
    signal cu_on  : std_logic := '0';
    signal pf_rdp : unsigned(7 downto 0) := (others => '0');

    -- pipeline stage registers (see stage comments in p_pix)
    signal b_u, b_v : signed(15 downto 0) := (others => '0');
    signal b_sl : unsigned(2 downto 0) := (others => '0');
    signal b_on : std_logic := '0';
    signal c_pu, c_pv : signed(13 downto 0) := (others => '0');
    signal c_cu, c_cv : unsigned(1 downto 0) := (others => '0');
    signal c_sl : unsigned(2 downto 0) := (others => '0');
    signal c_on : std_logic := '0';
    signal e_mu, e_mv : unsigned(8 downto 0) := (others => '0');
    signal e_gu, e_gv : unsigned(8 downto 0) := (others => '0');
    signal e_qsu, e_qsv : unsigned(8 downto 0) := (others => '0');
    signal e_stz : std_logic := '0';
    -- bevel tilt: ROM index per axis (penetration>>1, face bevel and cubie
    -- gap rounding rescaled onto one profile) plus the outward direction bit,
    -- which is simply the sign of the in-cubie coordinate in BOTH zones.
    signal e_tu, e_tv : unsigned(7 downto 0) := (others => '0');
    signal e_su, e_sv : std_logic := '0';
    signal e_tmax : unsigned(7 downto 0) := (others => '0');
    signal f_tmax, g_tmax, h_tmax : unsigned(7 downto 0) := (others => '0');
    signal f_su, f_sv : std_logic := '0';
    signal tr_du, tr_dv : unsigned(15 downto 0) := (others => '0');
    signal g_p0, g_p1, g_p2 : signed(18 downto 0) := (others => '0');
    signal h_ndh : unsigned(7 downto 0) := (others => '0');
    signal h_dif : signed(10 downto 0) := (others => '0');
    signal gm_a, gm_ap, gm_d : unsigned(7 downto 0) := (others => '0');
    signal sp_a : unsigned(7 downto 0) := (others => '0');
    signal sp_d : unsigned(15 downto 0) := (others => '0');
    signal k_gam : unsigned(7 downto 0) := (others => '0');
    signal k_spec : unsigned(11 downto 0) := (others => '0');
    signal t_dh0, t_dh1, t_dh2 : signed(11 downto 0) := (others => '0');
    signal e_dst : signed(10 downto 0) := (others => '0');
    signal e_dstr : signed(12 downto 0) := (others => '0');
    -- outline-corner chamfer: du+dv at a corner where both nearest lines are
    -- outline and one is a silhouette; s_crh = half the chamfer leg (K4)
    signal e_chs  : signed(13 downto 0) := (others => '0');
    signal e_chz  : std_logic := '0';
    signal e_cact : std_logic := '0';
    signal e_silz : std_logic := '0';
    signal e_stc : std_logic := '0';
    signal e_sl : unsigned(2 downto 0) := (others => '0');
    signal e_on : std_logic := '0';
    signal sq_du, sq_dv : unsigned(15 downto 0) := (others => '0');
    signal f_stz : std_logic := '0';
    signal f_dst : signed(10 downto 0) := (others => '0');
    signal f_dstr : signed(12 downto 0) := (others => '0');
    signal f_cact : std_logic := '0';
    signal f_2m : std_logic := '0';
    signal f_silz : std_logic := '0';
    signal f_stc : std_logic := '0';
    signal f_sl : unsigned(2 downto 0) := (others => '0');
    signal f_on : std_logic := '0';
    signal g_sx : unsigned(18 downto 0) := (others => '0');
    signal g_stz : std_logic := '0';
    signal g_dst : signed(10 downto 0) := (others => '0');
    signal g_dstr : signed(12 downto 0) := (others => '0');
    signal g_cact : std_logic := '0';
    signal g_2m : std_logic := '0';
    signal g_silz : std_logic := '0';
    signal g_stc : std_logic := '0';
    signal g_sl2 : unsigned(1 downto 0) := (others => '0');
    signal g_on : std_logic := '0';
    signal h_dlin : signed(11 downto 0) := (others => '0');
    signal h_stz : std_logic := '0';
    signal h_ao : unsigned(7 downto 0) := (others => '0');
    signal h_dst : signed(10 downto 0) := (others => '0');
    signal h_dstr : signed(12 downto 0) := (others => '0');
    signal h_cact : std_logic := '0';
    signal h_2m : std_logic := '0';
    signal h_silz : std_logic := '0';
    signal h_stc : std_logic := '0';
    signal h_sl : unsigned(1 downto 0) := (others => '0');
    signal h_on : std_logic := '0';
    signal i_dsil : signed(13 downto 0) := (others => '0');
    signal i_ds : signed(10 downto 0) := (others => '0');
    signal i_sl : unsigned(1 downto 0) := (others => '0');
    signal i_on : std_logic := '0';
    signal j_asil, j_ast : unsigned(4 downto 0) := (others => '0');
    signal j_sl : unsigned(1 downto 0) := (others => '0');
    signal j_on : std_logic := '0';
    signal k_y : unsigned(9 downto 0) := (others => '0');
    signal k_ston : std_logic := '0';
    signal k_sl2 : unsigned(1 downto 0) := (others => '0');
    signal k_asil : unsigned(4 downto 0) := (others => '0');
    signal m_y : unsigned(9 downto 0) := (others => '0');
    signal m_ston : std_logic := '0';
    signal m_sl2 : unsigned(1 downto 0) := (others => '0');
    signal m_asil : unsigned(4 downto 0) := (others => '0');
    signal n_y, n_u, n_v : unsigned(9 downto 0) := (others => '0');
    signal pf_consume : std_logic := '0';
    signal s_qx0 : unsigned(21 downto 0) := (others => '0');
    signal bg_vy : unsigned(9 downto 0) := (others => '0');
    signal bg_bx : unsigned(11 downto 0) := (others => '0');
    signal bg_bup : std_logic := '0';
    signal bg_wide : std_logic := '0';
    signal bg_vg, bg_v0, bg_st : unsigned(17 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- background accumulators (second differences: no multiplies)
    ----------------------------------------------------------------------
    signal bg_uq   : unsigned(17 downto 0) := to_unsigned(506*256, 18);
    signal bg_vq   : unsigned(17 downto 0) := to_unsigned(516*256, 18);
    signal bg_u    : unsigned(9 downto 0) := to_unsigned(506, 10);
    signal bg_v    : unsigned(9 downto 0) := to_unsigned(516, 10);
    -- delayed to meet pixel pipe stage M
    type t_bg_sr is array (0 to 13) of unsigned(9 downto 0);
    signal bgy_sr : t_bg_sr := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- output + sync delay
    ----------------------------------------------------------------------
    signal o_y, o_u, o_v : unsigned(9 downto 0) := (others => '0');
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');


    ------------------------------------------------------------------------
    -- microcoded geometry engine: register file, program counter, decode
    ------------------------------------------------------------------------
    type t_rf is array (0 to 127) of std_logic_vector(31 downto 0);
    signal rf_a, rf_b : t_rf := (others => (others => '0'));
    attribute ram_style of rf_a : signal is "block";
    attribute ram_style of rf_b : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf_a : signal is true;
    attribute no_rw_check of rf_b : signal is true;
    attribute no_rw_check of wram : signal is true;
    attribute no_rw_check of sd_h1 : signal is true;
    attribute no_rw_check of sd_h2 : signal is true;
    attribute no_rw_check of sd_lf : signal is true;
    attribute no_rw_check of sd_fd : signal is true;
    attribute no_rw_check of tbl : signal is true;
    signal u_pc  : unsigned(9 downto 0) := (others => '0');
    signal u_st  : unsigned(2 downto 0) := (others => '0');
    signal u_gwt : unsigned(1 downto 0) := (others => '0');
    signal u_ir, u_rom : std_logic_vector(31 downto 0) := (others => '0');
    signal u_fa  : unsigned(9 downto 0) := (others => '0');
    signal u_jr  : unsigned(9 downto 0) := (others => '0');
    signal u_wlast : std_logic := '0';
    signal u_ra, u_rb, u_wa, u_xa : unsigned(6 downto 0) := (others => '0');
    signal u_rda, u_rdb, u_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal u_xd, u_slv, u_ctlv : signed(31 downto 0) := (others => '0');
    signal u_we, u_slw : std_logic := '0';
    -- '1' at reset: the engine must IDLE until the first s_fstart.  At '0'
    -- it runs the whole program at power-up against default registers and
    -- burns the one-shot s_zfirst snap on them.
    signal u_done : std_logic := '1';
    signal u_slp : unsigned(4 downto 0) := (others => '0');
    signal u_sls : unsigned(2 downto 0) := (others => '0');
    -- serial shifter for SHR/SHL (one bit per cycle, instruction stalls)
    signal u_sh  : signed(31 downto 0) := (others => '0');
    signal u_shc : unsigned(3 downto 0) := (others => '0');
    signal u_shb : std_logic := '0';
    signal u_blt, u_bnz : std_logic := '0';   -- branch conditions (LATCH)
    signal u_rsel : unsigned(3 downto 0) := (others => '0');
    signal u_rwr, u_cin : std_logic := '0';
    signal u_ctlr : signed(31 downto 0) := (others => '0');
    signal u_romr : signed(16 downto 0) := (others => '0');
    signal dv_post, dv_ovf : std_logic := '0';

    type t_ucode is array (0 to 1023) of std_logic_vector(31 downto 0);
    constant C_UCODE : t_ucode := (
        x"9000003D", x"20504008", x"D850A03F", x"2060400E", x"D860C001", x"9800C00A",
        x"7070A000", x"1050A040", x"7050A000", x"9000000F", x"78600040", x"1850C140",
        x"7070A000", x"1850A040", x"7050A000", x"1850A1C0", x"D84040FF", x"5000A100",
        x"58500000", x"2050A008", x"1030E140", x"2060400F", x"D860C001", x"9800C019",
        x"D001C000", x"183000C0", x"D001C000", x"A0004020", x"60004ED0", x"68300000",
        x"9801E024", x"D001C000", x"18200080", x"60004ED0", x"68300000", x"9801E025",
        x"183000C0", x"D001C000", x"08304000", x"78600004", x"A80041AD", x"78600002",
        x"10304180", x"A00041AD", x"18304180", x"20604001", x"1060C180", x"78400004",
        x"18408180", x"78500006", x"1850A180", x"78600006", x"A000A1B6", x"78500000",
        x"D8604001", x"9800C039", x"D001C000", x"08608000", x"0840A000", x"0850C000",
        x"D001C000", x"78000000", x"78100001", x"C990000B", x"C9A0000C", x"C880000A",
        x"CA000000", x"7A300048", x"CA100001", x"7A40002D", x"CA200002", x"7A50001C",
        x"20D32001", x"78900000", x"F0212020", x"F0A12010", x"18304280", x"A0106010",
        x"08306000", x"90000051", x"183000C0", x"78400003", x"A0106114", x"08A04000",
        x"98010056", x"90000057", x"08A04000", x"F8012290", x"28214006", x"F0B12013",
        x"183042C0", x"28306008", x"28306008", x"20306008", x"2030600B", x"10B160C0",
        x"98010062", x"90000063", x"08B04000", x"F80122D3", x"D831A001", x"98006067",
        x"9000006B", x"F0412016", x"F0512023", x"10408140", x"F8012116", x"20D1A001",
        x"10912040", x"78200003", x"A011208A", x"D8232001", x"98004072", x"900000B2",
        x"9803C089", x"C820000D", x"78700005", x"D8304007", x"78400006", x"A810613A",
        x"185067C0", x"9800A081", x"20204003", x"1870E040", x"9800E075", x"1033E040",
        x"78400006", x"A0206101", x"78300000", x"09C06000", x"09F06000", x"C820000D",
        x"21D0400F", x"D9D3A001", x"79B00000", x"79E00001", x"900000C1", x"28234002",
        x"10204680", x"7830019A", x"102040C0", x"11B36080", x"2023600F", x"20204001",
        x"D8204001", x"98004093", x"900000C1", x"78200014", x"50038080", x"58C00000",
        x"78200038", x"10C18080", x"78D00005", x"80218000", x"80318001", x"80418002",
        x"80518003", x"80604000", x"80706000", x"80808000", x"8090A000", x"9803A0A7",
        x"B0006180", x"B00081C0", x"B000A200", x"B0004240", x"900000AB", x"B00041C0",
        x"B0006200", x"B0008240", x"B000A180", x"78200004", x"10C18080", x"18D1A040",
        x"9801A099", x"79E00000", x"79B00000", x"900000BF", x"78200000", x"78300000",
        x"78400009", x"B00040C0", x"10204040", x"18408040", x"980080B5", x"78400009",
        x"10306040", x"78500006", x"A0206175", x"79E00000", x"79B00000", x"7BB00000",
        x"900000CB", x"20236001", x"78E000C4", x"900000DF", x"78401000", x"10408100",
        x"103060C0", x"1BB080C0", x"9803A0CA", x"900000CB", x"1BB00EC0", x"12826580",
        x"129285C0", x"12A2A600", x"0AB76000", x"78800000", x"78900000", x"F0210028",
        x"78E000D4", x"90000001", x"F80120E0", x"F0210028", x"78E000D8", x"900000DF",
        x"F80120E1", x"10810040", x"10912040", x"10912040", x"78200004", x"A0310091",
        x"900000E3", x"78401000", x"28408002", x"10204100", x"90000001", x"7AC01000",
        x"7AD00000", x"7AE00000", x"7AF00000", x"7B001000", x"7B100000", x"7B200000",
        x"7B300000", x"7B401000", x"78900000", x"F061202C", x"F071202D", x"5000C940",
        x"58200008", x"5000E900", x"58300008", x"5000C900", x"58400008", x"5000E940",
        x"58500008", x"182040C0", x"10408140", x"F80120AC", x"F801212D", x"78200003",
        x"10912080", x"78200009", x"A03120AD", x"78900000", x"F061202D", x"F071202E",
        x"5000C8C0", x"58200008", x"5000E880", x"58300008", x"5000C880", x"58400008",
        x"5000E8C0", x"58500008", x"182040C0", x"10408140", x"F80120AD", x"F801212E",
        x"78200003", x"10912080", x"78200009", x"A0412080", x"78900000", x"F061202E",
        x"F071202C", x"5000C840", x"58200008", x"5000E800", x"58300008", x"5000C800",
        x"58400008", x"5000E840", x"58500008", x"182040C0", x"10408140", x"F80120AE",
        x"F801212C", x"78200003", x"10912080", x"78200009", x"A0412093", x"C8200005",
        x"C830000E", x"784013BC", x"28408002", x"10408040", x"50006100", x"58500008",
        x"784018A4", x"1050A100", x"50004140", x"58600008", x"788000FF", x"83510000",
        x"C870000A", x"9800E13E", x"1830CD40", x"A0406038", x"08406000", x"90000139",
        x"184000C0", x"7850000C", x"A040817F", x"20306003", x"1356A0C0", x"9000013F",
        x"0B50C000", x"B0010D40", x"CB600006", x"2B66C006", x"CB700007", x"2B76E006",
        x"783000B7", x"5006A0C0", x"58200008", x"D82040FF", x"78300000", x"08504000",
        x"78600008", x"A050A18F", x"2050A001", x"10306040", x"9000014B", x"78600006",
        x"A050A192", x"10306040", x"78600005", x"A0506196", x"0830C000", x"90000157",
        x"08306000", x"1860C0C0", x"8800C012", x"C8200008", x"C8300009", x"13802080",
        x"184020C0", x"A0504120", x"0B908000", x"90000161", x"0B904000", x"C8D0000F",
        x"5006A340", x"58D00008", x"10D1AD40", x"78900000", x"78800056", x"F0A1202C",
        x"F0B1202D", x"F0C1202E", x"50014340", x"58200000", x"2020400A", x"F8010080",
        x"50016D40", x"58200000", x"2020400A", x"18200080", x"F8010083", x"F8010306",
        x"78400000", x"78303FC7", x"500140C0", x"58200008", x"10408080", x"78300051",
        x"500160C0", x"58200008", x"10408080", x"783000EC", x"500180C0", x"58200008",
        x"10408080", x"F8010109", x"78400000", x"78303F95", x"500140C0", x"58200008",
        x"10408080", x"7830009A", x"500160C0", x"58200008", x"10408080", x"783000B8",
        x"500180C0", x"58200008", x"10408080", x"F801010C", x"78400000", x"7830008D",
        x"500140C0", x"58200008", x"10408080", x"78303F73", x"500160C0", x"58200008",
        x"10408080", x"783000A1", x"500180C0", x"58200008", x"10408080", x"F801010F",
        x"10810040", x"78200003", x"10912080", x"78200009", x"A05120A7", x"78200004",
        x"08338000", x"A86380AB", x"78200002", x"A06380AA", x"18338080", x"900001AB",
        x"10338080", x"27E06001", x"DFF06001", x"10AFC040", x"78200003", x"A06140B1",
        x"78A00000", x"10B14040", x"A06160B4", x"78B00000", x"78C00000", x"10619F80",
        x"F020C056", x"F800C0BC", x"10718280", x"F080E056", x"109182C0", x"F0D12056",
        x"500109C0", x"58200008", x"5001A980", x"58300008", x"102040C0", x"F800E0BC",
        x"5001A9C0", x"58200008", x"50010980", x"58300008", x"182040C0", x"F80120BC",
        x"78200003", x"10C18080", x"78200012", x"A06180B5", x"20278001", x"14E78080",
        x"2027E001", x"1517E080", x"2027A001", x"14F7A080", x"20280001", x"15280080",
        x"2027C001", x"1507C080", x"20282001", x"15382080", x"202AC001", x"168AC080",
        x"202B2001", x"16BB2080", x"202AE001", x"169AE080", x"202B4001", x"16CB4080",
        x"202B0001", x"16AB0080", x"202B6001", x"16DB6080", x"F02FC056", x"20304001",
        x"7850004E", x"1050BF80", x"F800A0C0", x"78500068", x"1050BF80", x"F800A080",
        x"980FE1EF", x"08604000", x"900001F0", x"18600080", x"1546C180", x"980FE1F4",
        x"08606000", x"900001F5", x"186000C0", x"1EE6C180", x"F02FC059", x"20304001",
        x"78500051", x"1050BF80", x"F800A0C0", x"7850006B", x"1050BF80", x"F800A080",
        x"980FE201", x"08604000", x"90000202", x"18600080", x"1556E180", x"980FE206",
        x"08606000", x"90000207", x"186000C0", x"1EF6E180", x"78900000", x"78B00070",
        x"08212000", x"78E0020D", x"90000026", x"20406001", x"D8506001", x"F0608042",
        x"9800A213", x"0860C000", x"90000214", x"18600180", x"78700008", x"78200000",
        x"A880E198", x"78200001", x"F8016080", x"F060805C", x"9800A21D", x"0860C000",
        x"9000021E", x"18600180", x"78700008", x"78200000", x"A880E1A2", x"78200001",
        x"F8016086", x"10B16040", x"10912040", x"78200006", x"A081208A", x"F02FC05C",
        x"980FE22B", x"08304000", x"9000022C", x"18300080", x"7FC00000", x"A08000EF",
        x"7FC00001", x"7FD00000", x"7AC00002", x"7AD00003", x"7AE00004", x"7AF00006",
        x"7B00003F", x"7B100080", x"7B2000FF", x"7B3000B0", x"7B40000C", x"0A0F8000",
        x"7A20003C", x"9804023D", x"9000023E", x"7A200056", x"7BA00070", x"98040241",
        x"90000242", x"7BA00076", x"7A100000", x"10274840", x"F0304000", x"98006247",
        x"9000039E", x"08242000", x"78E0024A", x"90000026", x"22306001", x"12346880",
        x"DB506001", x"22408001", x"12448880", x"DB608001", x"2250A001", x"1254A880",
        x"DB70A001", x"F0348000", x"9806C257", x"0A606000", x"90000258", x"1A6000C0",
        x"F0348003", x"9806C25C", x"0A706000", x"9000025D", x"1A7000C0", x"F034A000",
        x"9806E261", x"0A806000", x"90000262", x"1A8000C0", x"F034A003", x"9806E266",
        x"0A906000", x"90000267", x"1A9000C0", x"F2A44018", x"F2B44019", x"F0346012",
        x"9806A26D", x"08506000", x"9000026E", x"185000C0", x"12A54140", x"F0346015",
        x"9806A273", x"08506000", x"90000274", x"185000C0", x"12B56140", x"F0348012",
        x"9806C279", x"08506000", x"9000027A", x"185000C0", x"1AA54140", x"F0348015",
        x"9806C27F", x"08506000", x"90000280", x"185000C0", x"1AB56140", x"F034A012",
        x"9806E285", x"08506000", x"90000286", x"185000C0", x"1AA54140", x"F034A015",
        x"9806E28B", x"08506000", x"9000028C", x"185000C0", x"1AB56140", x"5004CA40",
        x"58200000", x"5004EA00", x"58300000", x"182040C0", x"78F00000", x"A0A00096",
        x"78F00001", x"18200080", x"23B04008", x"08252000", x"78E0029A", x"9000001B",
        x"08A06000", x"18200A00", x"78E0029E", x"9000001B", x"08B06000", x"182009C0",
        x"78E002A2", x"9000001B", x"08C06000", x"0824C000", x"78E002A6", x"9000001B",
        x"08D06000", x"22A54002", x"22B56002", x"78900003", x"2891200C", x"78800FFF",
        x"10912200", x"A0A14030", x"08214000", x"900002B1", x"18200280", x"A0A16034",
        x"08316000", x"900002B5", x"183002C0", x"A0A040F8", x"08204000", x"900002B9",
        x"08206000", x"A0A120BE", x"08414000", x"08516000", x"88001F53", x"900002C4",
        x"78600020", x"10414180", x"20408006", x"10516180", x"2050A006", x"88003F53",
        x"88009F40", x"18600A80", x"50008180", x"58700000", x"28672004", x"1860CAC0",
        x"5000A180", x"58800000", x"1070E200", x"2070E004", x"8800FF44", x"5000AE00",
        x"58800000", x"C8600003", x"50008180", x"58700000", x"188101C0", x"88011F41",
        x"A0B18019", x"08218000", x"900002DA", x"18200300", x"A0B1A01D", x"0831A000",
        x"900002DE", x"18300340", x"A0B040E1", x"08204000", x"900002E2", x"08206000",
        x"A0B120A7", x"08418000", x"0851A000", x"88001F54", x"900002ED", x"78600020",
        x"10418180", x"20408006", x"1051A180", x"2050A006", x"88003F54", x"88009F42",
        x"18600A80", x"50008180", x"58700000", x"28672004", x"1860CAC0", x"5000A180",
        x"58800000", x"1070E200", x"2070E004", x"8800FF45", x"5000AE00", x"58800000",
        x"C8600003", x"50008180", x"58700000", x"188101C0", x"88011F43", x"50038D00",
        x"58800000", x"50040BC0", x"58200000", x"10810080", x"10810CC0", x"10210840",
        x"80A04000", x"78B00000", x"78C00000", x"782000F8", x"10204840", x"80D04000",
        x"0821A000", x"D8204007", x"10410080", x"80508000", x"1870AC00", x"9800E314",
        x"18714C00", x"9800E31A", x"10474080", x"F0508000", x"9800A31B", x"78500001",
        x"10B16140", x"9000031B", x"78C00001", x"2021A003", x"D8204007", x"10410080",
        x"80508000", x"1870AC00", x"9800E323", x"18714C00", x"9800E329", x"10474080",
        x"F0508000", x"9800A32A", x"78500002", x"10B16140", x"9000032A", x"78C00002",
        x"2021A006", x"D8204007", x"10410080", x"80508000", x"1870AC00", x"9800E332",
        x"18714C00", x"9800E338", x"10474080", x"F0508000", x"9800A339", x"78500004",
        x"10B16140", x"90000339", x"78C00003", x"2021A009", x"D8204007", x"10410080",
        x"80508000", x"1870AC00", x"9800E341", x"18714C00", x"9800E347", x"10474080",
        x"F0508000", x"9800A348", x"78500008", x"10B16140", x"90000348", x"78C00004",
        x"18248880", x"78600003", x"18205F80", x"9800434D", x"10602800", x"1824A880",
        x"78700003", x"18205F80", x"98004352", x"10702800", x"78200000", x"1030C1C0",
        x"A8D06BE5", x"78200001", x"78400001", x"1830E100", x"9800635A", x"90000365",
        x"78200002", x"78400001", x"1830C100", x"9800635F", x"90000365", x"78200003",
        x"78400002", x"1830E100", x"98006364", x"90000365", x"78200004", x"2831400A",
        x"28204007", x"10306080", x"28218004", x"10306080", x"103062C0", x"88007F46",
        x"F0346009", x"9806A370", x"08B06000", x"90000371", x"18B000C0", x"F0348009",
        x"9806C375", x"08C06000", x"90000376", x"18C000C0", x"F034A009", x"9806E37A",
        x"08D06000", x"9000037B", x"18D000C0", x"D82183FF", x"28204006", x"D831603F",
        x"102040C0", x"88005F47", x"D821A3FF", x"28204006", x"20316006", x"D830600F",
        x"102040C0", x"88005F48", x"F034600C", x"9806A38A", x"08B06000", x"9000038B",
        x"18B000C0", x"F034600F", x"9806A38F", x"08C06000", x"90000390", x"18C000C0",
        x"7850003E", x"A8E002D5", x"20316002", x"184160C0", x"1050A100", x"A8E00318",
        x"20318002", x"1050A0C0", x"A0E0AC9B", x"08564000", x"9000039C", x"0850A000",
        x"8800BF49", x"17DFA040", x"12142040", x"A0942BC3", x"1A002800", x"18241F00",
        x"9800423A", x"880FA00F", x"88000019", x"C0000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000" );
    -- Exact signed product a * m, radix-4 Booth on m: ceil(len(m)/2)
    -- partials of {0, +-a, +-2a} (negatives as one's complement, the +1s
    -- gathered in one correction word in a spare slot), balanced adds.
    -- Checked in Python against a*m for every operand width pair used here.
    function bmul(a : signed; m : signed) return signed is
        constant NA  : natural := a'length;
        constant NML : natural := m'length;
        constant NM  : natural := NML + (NML mod 2);
        constant NP  : natural := NA + NML;
        constant NPP : natural := NM / 2;
        type t_pp is array (0 to 7) of signed(NP - 1 downto 0);
        variable va   : signed(NA - 1 downto 0);
        variable vm   : signed(NM - 1 downto 0);
        variable me   : std_logic_vector(NM downto 0);
        variable a1, a2, mag, pp, corr : signed(NP - 1 downto 0);
        variable d    : std_logic_vector(2 downto 0);
        variable p    : t_pp;
    begin
        va := a;
        vm := resize(m, NM);
        me := std_logic_vector(vm) & '0';
        a1 := resize(va, NP);
        a2 := shift_left(a1, 1);
        corr := (others => '0');
        for i in 0 to 7 loop
            p(i) := (others => '0');
        end loop;
        for i in 0 to NPP - 1 loop
            d := me(2 * i + 2 downto 2 * i);
            case d is
                when "001" | "010" | "101" | "110" => mag := a1;
                when "011" | "100"                 => mag := a2;
                when others                        => mag := (others => '0');
            end case;
            if d(2) = '1' and d /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        p(NPP) := corr;
        return ((p(0) + p(1)) + (p(2) + p(3))) + ((p(4) + p(5)) + (p(6) + p(7)));
    end function;

    -- Exact unsigned product a * m (width len(a) + len(m)) on the signed Booth.
    function bmul_u(a : unsigned; m : unsigned) return unsigned is
        constant NP : natural := a'length + m'length;
        variable vp : signed(NP + 1 downto 0);
    begin
        vp := bmul(signed('0' & a), signed('0' & m));
        return unsigned(vp(NP - 1 downto 0));
    end function;


begin

    ------------------------------------------------------------------------
    -- raster measurement (width, active-line count, interlace detect)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_avid_f <= s_avid_q and not data_in.avid;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;   -- falling edge of _n

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            -- CLEAN AVID.  The encoders get no data-enable -- only hsync and
            -- the 4:2:2 bus -- so they pair Cb/Cr by counting from hsync, while
            -- the core's 4:4:4->4:2:2 packer restarts on Cb at every avid
            -- rise and resets at every avid fall.  An analog source's avid
            -- wanders a clock against hsync and can drop out mid-line: either
            -- one swaps U/V for (the rest of) that line -- thin coloured lines
            -- over the stickers, faces flipping hue.  So the program makes its
            -- own avid: it starts 2 or 3 clocks after the source's first avid
            -- rise of the line, whichever keeps the hsync->start distance at
            -- the source's USUAL parity (a saturating vote, so jitter can't
            -- move it), and ends with the DDA's line (dd_wrap), so dropouts
            -- are ignored.
            if s_hs_r = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if s_avid_r = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            g_avid_q <= g_avid;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
            elsif dd_wrap = '1' then
                g_avid <= '0';
            end if;

            -- count active lines on the ACTIVE edge (vsync height-capture trap:
            -- never latch height at vsync from a serrated counter)
            if s_avid_r = '1' then
                s_ycnt <= s_ycnt + 1;
                s_sawact <= '1';
            end if;

            -- The engine starts once active video has been absent for two
            -- line widths -- early in vertical blanking, not at vsync (the
            -- whole blanking, ~21 lines in NTSC instead of ~17).  No hsync
            -- involved: a noisy or doubled hsync edge must never start it in
            -- mid-picture, where it would take the sticker-colour RAM and
            -- reset the face DDAs.  Re-arms only on active video.
            s_estart <= '0';
            if data_in.avid = '1' then
                s_idle <= (others => '0');
                s_earm <= '1';
            else
                if s_idle /= "1111111111111" then s_idle <= s_idle + 1; end if;
                if s_earm = '1' and s_idle = (s_W & '0') then
                    s_estart <= '1';
                    s_earm   <= '0';
                end if;
            end if;
            -- Only the FIRST vsync edge after active video is a real frame
            -- boundary; an analog serrated vsync gives several per field and
            -- would latch a partial line count (and restart the frame FSM).
            s_fstart <= '0';
            if s_vs_pulse = '1' and s_sawact = '1' then
                if s_ycnt > 16 then s_H <= s_ycnt; end if;
                s_ycnt <= (others => '0');
                s_sawact <= '0';
                s_fstart <= '1';
                s_field <= data_in.field_n;
                s_ilace <= s_tid_il;
            end if;
        end if;
    end process p_measure;

    -- timing-ID classes (static while a mode runs)
    p_tid : process(clk)
    begin
        if rising_edge(clk) then
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => s_tid_il <= '1';
                when others             => s_tid_il <= '0';
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0      => s_xa <= to_signed(455, 11);    -- 480i: 1080/486 / 2
                when 4      => s_xa <= to_signed(512, 11);    -- 480p: 1080/480 / 2
                when 8 | 12 => s_xa <= to_signed(-256, 11);   -- 576: 1080/576 / 2
                when others => s_xa <= to_signed(0, 11);
            end case;
        end if;
    end process p_tid;

    ------------------------------------------------------------------------
    -- shared frame multiplier (2-cycle: operands set at N, product at N+2)
    -- and serial restoring divider (dv_q = fd_n / fd_d after fd_cnt cycles)
    ------------------------------------------------------------------------
    ------------------------------------------------------------------------
    -- SHARED FRAME/LINE DATAPATH.  The line FSM only ever runs with
    -- fr_done = '1' and the frame FSM only with fr_done = '0', so both can
    -- drive ONE multiplier array and ONE divider pair through an operand
    -- mux -- half the arithmetic area for free.  Latency is unchanged
    -- (operands registered by the caller at N, product readable at N+2).
    ------------------------------------------------------------------------

    ------------------------------------------------------------------------
    -- FRAME FSM (microcoded).  All per-frame vertex/basis data lives in the
    -- FRAM scratch EBR (one read + one write port), so no state instantiates
    -- register-array muxes; the shared multiplier (fm), divider (fd) and the
    -- single quarter-sine ROM port (t_sk/t_srd) do all the heavy math.
    -- FRAM map: 0..8 basis (vec*3+comp) | 18+c*8+k corners (c=0/1/2=x/y/z)
    --           42+k rx | 50+k ry | 58+k sx | 66+k sy   (k = corner 0..7)
    ------------------------------------------------------------------------
    ------------------------------------------------------------------------
    -- What survives of the old frame FSM: latch the knobs and the raster
    -- measurements at vblank, and clear the first-frame snap once the engine
    -- has consumed it.  Everything else is microcode now.
    ------------------------------------------------------------------------
    p_fsetup : process(clk)
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                if s_ilace = '1' then s_hf <= resize(s_H & '0', 13);
                else                  s_hf <= resize(s_H, 13); end if;
                s_cx <= '0' & s_W(11 downto 1);
            end if;
            if s_estart = '1' then
                fr_done <= '0';
                s_k1 <= unsigned(registers_in(0));
                s_k2 <= unsigned(registers_in(1));
                s_k3 <= unsigned(registers_in(2));
                s_sw <= registers_in(6)(4 downto 0);
                s_sld <= unsigned(registers_in(7));
                s_vsel <= unsigned(registers_in(4)(9 downto 7));
                s_Wl <= s_W;
                s_frans <= '1';
            else
                fr_done <= u_done;
                -- only once a frame has actually STARTED: u_done idles high out
                -- of reset, so an ungated test clears the snap before the engine
                -- has ever run.
                if u_done = '1' and s_frans = '1' then s_zfirst <= '0'; end if;
            end if;
        end if;
    end process p_fsetup;

    ------------------------------------------------------------------------
    -- SLW decoder.  Ports 0..14 are per-slot; 15 and 16 are globals, which
    -- is why the port field is five bits and not four.
    ------------------------------------------------------------------------
    p_slotw : process(clk)
        variable v_s : integer range 0 to 5;
    begin
        if rising_edge(clk) then
            w_we <= '0';
            sd_we <= (others => '0');
            dd_kick <= '0';
            if u_slw = '1' and u_sls < 6 then
                v_s := to_integer(u_sls);
                case to_integer(u_slp) is
                    -- DDA table: wrap step 0.., field seed 16.., gx 32..
                    when  0 => w_wa <= to_unsigned(32 + 2*v_s, 8); w_we <= '1';
                    when  1 => w_wa <= to_unsigned( 0 + 2*v_s, 8); w_we <= '1';
                    when  2 => w_wa <= to_unsigned(33 + 2*v_s, 8); w_we <= '1';
                    when  3 => w_wa <= to_unsigned( 1 + 2*v_s, 8); w_we <= '1';
                    when  4 => w_wa <= to_unsigned(16 + 2*v_s, 8); w_we <= '1';
                    when  5 => w_wa <= to_unsigned(17 + 2*v_s, 8); w_we <= '1';
                    -- face descriptor; the stage-A limits keep a copy of the shape
                    when  6 =>
                        sd_wa <= to_unsigned(v_s, 8); sd_we(3) <= '1';
                        case to_integer(unsigned(u_slv(9 downto 7))) is
                            when 1 => sl_nu(v_s) <= "11"; sl_nv(v_s) <= "01";
                            when 2 => sl_nu(v_s) <= "01"; sl_nv(v_s) <= "11";
                            when 3 => sl_nu(v_s) <= "11"; sl_nv(v_s) <= "10";
                            when 4 => sl_nu(v_s) <= "10"; sl_nv(v_s) <= "11";
                            when others => sl_nu(v_s) <= "11"; sl_nv(v_s) <= "11";
                        end case;
                    when  7 => sd_wa <= to_unsigned(v_s, 8); sd_we(0) <= '1';
                    when  8 => sd_wa <= to_unsigned(v_s, 8); sd_we(1) <= '1';
                    when  9 => sd_wa <= to_unsigned(v_s, 8); sd_we(2) <= '1';
                    when 15 => s_nslot <= unsigned(u_slv(2 downto 0));
                    when 18 => s_pxe <= to_integer(unsigned(u_slv(2 downto 0)));
                    when 19 => dd_e(2*v_s) <= u_slv(0);
                    when 20 => dd_e(2*v_s + 1) <= u_slv(0);
                    when 25 => dd_kick <= '1';
                    when others => null;
                end case;
            end if;
        end if;
    end process p_slotw;

    ------------------------------------------------------------------------
    -- MICROCODED GEOMETRY ENGINE
    --
    -- Replaces ~110 hand-written frame-setup states.  Each of those states
    -- carried its own adders, comparators and address arithmetic, and each
    -- one that touched the multiplier was another input to an 18-bit operand
    -- mux; together that measured 2,666 cells for math that runs once per
    -- frame in blanking with ~99,000 spare clocks.
    --
    -- Here there is ONE ALU, the scratch RAM is the register file (two
    -- copies so an instruction can read both operands in a cycle), and the
    -- program lives in a block RAM.  Three cycles per instruction, which at
    -- ~1,500 dynamic instructions is ~4,500 clocks of the vblank budget.
    ------------------------------------------------------------------------
    p_ueng : process(clk)
        variable v_a, v_b : signed(31 downto 0);
        variable v_r      : signed(31 downto 0);
        variable v_op     : integer range 0 to 31;
        variable v_op2    : integer range 0 to 31;
        variable v_imm    : signed(13 downto 0);
        variable v_stl    : boolean;
        variable v_jmp    : boolean;
        variable v_dr     : unsigned(21 downto 0);
        variable v_ms     : signed(21 downto 0);
        variable v_mx     : signed(21 downto 0);
        variable v_m23    : signed(22 downto 0);
        variable v_s1, v_s2, v_neg : std_logic;
        variable v_pr     : signed(31 downto 0);
        variable v_c33    : signed(32 downto 0);
        variable v_fa, v_fb : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            u_wlast <= u_we;
            -- serial multiply, radix-4 Booth, 8 steps: {mu_ph, mu_pl} =
            -- mu_pa * B for a two's-complement B, two bits a step with the
            -- digit in {0, +-A, +-2A} -- no operand or product negation.
            if mu_bsy = '1' then
                -- ONE adder: the digit picks 0 / A / 2A, a negative digit
                -- inverts it and carries in the +1 (two's complement), so
                -- ph + {0, +-A, +-2A} without four adders and a 5:1 mux
                v_s1  := mu_pl(0) xor mu_bp;
                v_s2  := (mu_pl(1) xor mu_pl(0)) and not (mu_pl(0) xor mu_bp);
                v_neg := mu_pl(1) and not (mu_pl(0) and mu_bp);
                if v_s2 = '1' then
                    v_mx := shift_left(resize(mu_pa, 22), 1);
                elsif v_s1 = '1' then
                    v_mx := resize(mu_pa, 22);
                else
                    v_mx := (others => '0');
                end if;
                if v_neg = '1' then v_mx := not v_mx; end if;
                v_m23 := (mu_ph & '1') + (v_mx & v_neg);
                v_ms := v_m23(22 downto 1);
                mu_ph <= shift_right(v_ms, 2);
                mu_pl <= unsigned(v_ms(1 downto 0)) & mu_pl(15 downto 2);
                mu_bp <= mu_pl(1);
                if mu_cnt = 0 then mu_bsy <= '0';
                else mu_cnt <= mu_cnt - 1; end if;
            end if;

            -- serial restoring divider.  The numerator shifts out of the top
            -- of dv_na as the quotient shifts in at the bottom, so after 34
            -- steps dv_na IS the quotient: one register, not two.
            if dv_bsy = '1' and dv_post = '0' then
                v_dr := dv_ra(20 downto 0) & dv_na(33);
                if v_dr >= resize(dv_da, 22) then
                    dv_ra <= v_dr - resize(dv_da, 22);
                    dv_na <= dv_na(32 downto 0) & '1';
                else
                    dv_ra <= v_dr;
                    dv_na <= dv_na(32 downto 0) & '0';
                end if;
                if dv_cnt = 0 then dv_post <= '1';
                else dv_cnt <= dv_cnt - 1; end if;
            end if;
            if dv_post = '1' then                 -- one step after the last
                if dv_na(33 downto 20) /= 0 then dv_ovf <= '1';
                else dv_ovf <= '0'; end if;
                dv_post <= '0';
                dv_bsy  <= '0';
            end if;

            u_we   <= '0';
            u_slw  <= '0';
            tb_we  <= '0';

            if s_estart = '1' then
                u_pc   <= (others => '0');
                u_st   <= "000";
                u_gwt  <= (others => '0');
                u_done <= '0';
            elsif u_done = '0' then
                case to_integer(u_st) is

                    -- FETCH (after a jump only): the ROM answers next cycle
                    when 0 =>
                        u_st <= "001";

                    -- DECODE: latch the instruction.  The register-file reads
                    -- are addressed straight off the ROM output this cycle
                    -- (see p_rfa/p_rfb), so the operands arrive next cycle.
                    when 1 =>
                        u_ir <= u_rom;
                        u_st <= "010";

                    -- OPERAND LATCH: block-RAM outputs into fabric before any
                    -- arithmetic.  Operand A lands in the shifter register and
                    -- B in the write-data register -- both idle here, since
                    -- the previous result was written two cycles ago.
                    when 2 =>
                        v_op2 := to_integer(unsigned(u_ir(31 downto 27)));
                        -- Forwarding: the previous instruction's register write
                        -- lands on the same edge as this one's read, and an
                        -- iCE40 block RAM returns undefined data for a
                        -- read-during-write -- so a just-written source comes
                        -- from the write-data register instead.
                        if u_wlast = '1' and unsigned(u_ir(19 downto 13)) = u_wa then
                            v_fa := u_wd;
                        else
                            v_fa := u_rda;
                        end if;
                        if u_wlast = '1' and unsigned(u_ir(12 downto 6)) = u_wa then
                            v_fb := u_wd;
                        else
                            v_fb := u_rdb;
                        end if;
                        u_sh  <= signed(v_fa);
                        -- SUB = A + not B + 1 on the one adder
                        if v_op2 = 3 then
                            u_wd <= not v_fb;  u_cin <= '1';
                        else
                            u_wd <= v_fb;      u_cin <= '0';
                        end if;
                        u_ctlr <= u_ctlv;
                        -- result source and write flag, off EXEC's cone
                        case v_op2 is
                            when 1 | 4 | 5    => u_rsel <= x"0"; u_rwr <= '1';
                            when 2 | 3        => u_rsel <= x"1"; u_rwr <= '1';
                            when 27           => u_rsel <= x"2"; u_rwr <= '1';
                            when 15           => u_rsel <= x"3"; u_rwr <= '1';
                            when 11           => u_rsel <= x"4"; u_rwr <= '1';
                            when 13           => u_rsel <= x"5"; u_rwr <= '1';
                            when 14 | 16 => u_rsel <= x"6"; u_rwr <= '1';
                            when 25           => u_rsel <= x"8"; u_rwr <= '1';
                            when 31           => u_rsel <= x"9"; u_rwr <= '1';
                            when others       => u_rsel <= x"0"; u_rwr <= '0';
                        end case;
                        u_shc <= unsigned(u_ir(3 downto 0));
                        u_wa  <= unsigned(u_ir(26 downto 20));
                        -- branch conditions a state early, into flip-flops:
                        -- from EXEC's subtract the compare had to cross the
                        -- die to the program counter beside the ROM
                        if signed(v_fa) < signed(v_fb) then u_blt <= '1';
                        else u_blt <= '0'; end if;
                        if v_fa /= x"00000000" then u_bnz <= '1';
                        else u_bnz <= '0'; end if;
                        u_jr <= unsigned(v_fa(9 downto 0));
                        -- Every unit launches HERE, from the register-file
                        -- outputs, into registers that live beside it.  From
                        -- EXEC, operand A fanned out to the ALU, multiplier,
                        -- divider, three address ports and the slot writer at
                        -- once, and one hop of that net cost 9 ns of routing.
                        case v_op2 is
                            when 10 =>                     -- MUL
                                mu_pa  <= signed(v_fa(19 downto 0));
                                mu_pl  <= unsigned(v_fb(15 downto 0));
                                mu_ph  <= (others => '0');
                                mu_bp  <= '0';
                                mu_cnt <= to_unsigned(7, 4);
                                mu_bsy <= '1';
                            when 12 =>                     -- DIV: A << 0/16
                                -- operands are magnitudes (microcode contract)
                                if u_ir(4) = '1' then
                                    dv_na <= shift_left(resize(unsigned(v_fa), 34), 16);
                                else
                                    dv_na <= resize(unsigned(v_fa), 34);
                                end if;
                                if unsigned(v_fb(20 downto 0)) = 0 then
                                    dv_da <= to_unsigned(1, 21);
                                else
                                    dv_da <= unsigned(v_fb(20 downto 0));
                                end if;
                                dv_ra   <= (others => '0');
                                dv_cnt  <= to_unsigned(33, 6);
                                dv_bsy  <= '1';
                            when 14 => t_sk   <= unsigned(v_fa(7 downto 0));
                            when 16 =>                     -- SRD address
                                tb_era <= unsigned(v_fa(7 downto 0))
                                          + unsigned(u_ir(6 downto 0));
                            when 22 =>                     -- SWR
                                tb_wa <= unsigned(v_fa(7 downto 0));
                                tb_wd <= v_fb(15 downto 0);
                            when 30 =>                     -- LDX address
                                u_ra <= unsigned(v_fa(6 downto 0))
                                        + unsigned(u_ir(6 downto 0));
                            when 31 =>                     -- STX address
                                u_wa <= unsigned(v_fa(6 downto 0))
                                        + unsigned(u_ir(5 downto 0));
                            when 17 =>                     -- SLW operands
                                u_slp <= unsigned(u_ir(4 downto 0));
                                u_slv <= signed(v_fa);
                                u_sls <= unsigned(v_fb(2 downto 0));
                            when others => null;
                        end case;
                        u_st  <= "011";

                    -- same one-cycle read latency on the indexed address
                    when 4 =>
                        u_st <= "101";

                    -- INDEXED second access (LDX).  The ROM has been reading
                    -- pc+1 meanwhile, so the next instruction decodes directly.
                    when 5 =>
                        u_st <= "001";
                        u_pc <= u_pc + 1;
                        u_wa <= unsigned(u_ir(26 downto 20));
                        u_wd <= u_rda;
                        u_we <= '1';

                    -- EXECUTE.  Everything this state reads is a register:
                    -- the operands and branch conditions from the latch, the
                    -- result SOURCE pre-decoded there too, the units' outputs.
                    -- One adder (ADD, and SUB via B inverted at the latch), one
                    -- serial shifter, fixed MRD/DIV shift sets, no negators --
                    -- NEG, MIN/MAX/CLP/ABS and BIT are assembler macros, and
                    -- DIV operands are magnitudes by microcode contract.  The
                    -- ROM reads pc+1 throughout, so a straight-line instruction
                    -- takes 3 clocks; a taken jump refetches (4).
                    when others =>
                        v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                        v_imm := signed(u_ir(13 downto 0));
                        v_a   := u_sh;
                        v_b   := signed(u_wd);
                        v_stl := false;
                        v_jmp := false;

                        -- the value, by the pre-decoded source
                        case to_integer(u_rsel) is
                            when 0 => v_r := u_sh;          -- MOV, SHR/SHL
                            -- one adder: SUB's +1 enters as the carry-in
                            -- (a 33-bit add with the cin in a spare low bit)
                            -- instead of a second, incrementing chain
                            when 1 => v_c33 := (v_a & '1') + (v_b & u_cin);
                                      v_r := v_c33(32 downto 1);
                            when 2 => v_r := v_a and signed(resize(unsigned(u_ir(12 downto 0)), 32));
                            when 3 => v_r := resize(v_imm, 32);
                            when 4 =>                        -- MRD: low 32 bits
                                v_pr := signed(std_logic_vector'(
                                            std_logic_vector(mu_ph(15 downto 0))
                                            & std_logic_vector(mu_pl)));
                                if u_ir(3) = '1' then v_r := shift_right(v_pr, 12);
                                else                  v_r := v_pr; end if;
                            when 5 =>                        -- DRD: clamped magnitude
                                if dv_ovf = '1' then
                                    v_r := to_signed(1048575, 32);
                                else
                                    v_r := signed(resize(dv_na(19 downto 0), 32));
                                end if;
                            when 6 => v_r := resize(u_romr, 32);  -- SIN/GAM/SRD
                            when 9 => v_r := v_b;           -- STX data
                            when others => v_r := u_ctlr;   -- knobs and raster
                        end case;

                        -- control
                        case v_op is
                            when 4 | 5 =>                  -- SHR / SHL
                                -- A is already in u_sh with the count beside it
                                if u_shc /= 0 then
                                    if v_op = 4 then u_sh <= shift_right(u_sh, 1);
                                    else             u_sh <= shift_left(u_sh, 1); end if;
                                    u_shc <= u_shc - 1;
                                    v_stl := true;
                                end if;
                            when 11 => if mu_bsy = '1' then v_stl := true; end if;
                            when 13 => if dv_bsy = '1' then v_stl := true; end if;
                            when 14 | 16 =>                -- SIN / SRD
                                -- the ROM answers on the second cycle; take it
                                -- into fabric before the result mux sees it,
                                -- or the mux gets placed beside the EBR
                                if u_gwt = 0 then
                                    u_gwt <= "01";
                                    v_stl := true;
                                elsif u_gwt = 1 then
                                    if v_op = 14 then u_romr <= resize(t_srd, 17);
                                    else u_romr <= signed('0' & tb_rd); end if;
                                    u_gwt <= "10";
                                    v_stl := true;
                                else
                                    u_gwt <= (others => '0');
                                end if;
                            when 17 => u_slw <= '1';       -- SLW (operands latched)
                            when 22 => tb_we <= '1';       -- SWR (operands latched)
                            when 18 => v_jmp := true;
                                       u_pc <= unsigned(u_ir(9 downto 0));
                            when 19 => if u_bnz = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(9 downto 0));
                                       end if;
                            when 20 => if u_blt = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(23 downto 20))
                                                   & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 21 => if u_blt = '0' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(23 downto 20))
                                                   & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 26 => v_jmp := true;      -- JR
                                       u_pc <= u_jr;
                            when 24 => u_done <= '1';
                            when others => null;           -- MUL/DIV launched
                        end case;

                        if v_stl then
                            u_st <= "011";
                        elsif v_op = 30 then
                            u_st <= "100";                 -- LDX second access
                        elsif v_jmp then
                            u_st <= "000";
                        else
                            u_st <= "001";
                            u_pc <= u_pc + 1;
                        end if;

                        -- The result register loads every EXEC cycle (nothing
                        -- that stalls still needs operand B), so its enable is
                        -- just the state; the strobe carries the conditions.
                        u_wd <= std_logic_vector(resize(v_r, 32));
                        if u_rwr = '1' and not v_stl then
                            u_we <= '1';
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

    ------------------------------------------------------------------------
    -- CTL reads the outside world.  u_ir is latched at DECODE, so selecting
    -- on its immediate field is stable through EXECUTE.
    ------------------------------------------------------------------------
    with to_integer(unsigned(u_ir(3 downto 0))) select u_ctlv <=
        signed(resize(s_k1, 32)) when 0,
        signed(resize(s_k2, 32)) when 1,
        signed(resize(s_k3, 32)) when 2,
        signed(resize(s_Wl, 32)) when 3,
        signed(resize(s_H, 32)) when 4,
        signed(resize(s_hf, 32)) when 5,
        signed(resize(s_cx, 32)) when 6,
        signed(resize(s_cy, 32)) when 7,
        (0 => s_ilace, others => '0')    when 8,
        -- the engine runs in the blanking BEFORE the next field, whose
        -- parity is the opposite of the one just shown
        (0 => not s_field, others => '0') when 9,
        (0 => s_zfirst, others => '0')   when 10,
        signed(resize(unsigned(s_sw), 32))  when 11,
        signed(resize(s_sld, 32))        when 12,
        signed(resize(s_lfsr, 32))       when 13,
        -- K6 size, read live: one CTL a field, so no latch is needed
        signed(resize(unsigned(registers_in(5)), 32)) when 14,
        resize(s_xa, 32)                 when 15,
        (others => '0')                  when others;

    -- free-running random source for the turn picker
    p_lfsr : process(clk)
    begin
        if rising_edge(clk) then
            if s_lfsr(0) = '1' then
                s_lfsr <= ('0' & s_lfsr(15 downto 1)) xor x"B400";
            else
                s_lfsr <= '0' & s_lfsr(15 downto 1);
            end if;
        end if;
    end process p_lfsr;

    ------------------------------------------------------------------------
    -- microcode ROM and the two register-file copies (dual read)
    ------------------------------------------------------------------------
    -- the ROM reads pc+1 during EXEC and LDX's second access, so a
    -- straight-line instruction decodes on the very next clock
    u_fa <= u_pc + 1 when (u_st = "011" or u_st = "101") else u_pc;

    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_fa));
        end if;
    end process p_urom;

    p_rfa : process(clk)
        variable v_ra : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_a(to_integer(u_wa)) <= u_wd; end if;
            -- DECODE reads srcA straight off the ROM; LDX's second access
            -- reads the index it computed
            if u_st = "001" then v_ra := unsigned(u_rom(19 downto 13));
            else                 v_ra := u_ra; end if;
            u_rda <= rf_a(to_integer(v_ra));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_b(to_integer(u_wa)) <= u_wd; end if;
            u_rdb <= rf_b(to_integer(unsigned(u_rom(12 downto 6))));
        end if;
    end process p_rfb;


    ------------------------------------------------------------------------
    -- DDA table (write: engine via SLW, read: DDA step sequencer)
    ------------------------------------------------------------------------
    ------------------------------------------------------------------------
    -- per-slot descriptor RAMs: write from the engine, read by the pixel
    -- pipe with the address one stage ahead of the use (data registered)
    ------------------------------------------------------------------------
    sd_wd <= std_logic_vector(u_slv(15 downto 0));

    p_sdh1 : process(clk)
    begin
        if rising_edge(clk) then
            if sd_we(0) = '1' then sd_h1(to_integer(sd_wa)) <= sd_wd; end if;
            sd_h1d <= sd_h1(to_integer(e_sl));
        end if;
    end process p_sdh1;

    p_sdh2 : process(clk)
    begin
        if rising_edge(clk) then
            if sd_we(1) = '1' then sd_h2(to_integer(sd_wa)) <= sd_wd; end if;
            sd_h2d <= sd_h2(to_integer(e_sl));
        end if;
    end process p_sdh2;

    p_sdlf : process(clk)
    begin
        if rising_edge(clk) then
            if sd_we(2) = '1' then sd_lf(to_integer(sd_wa)) <= sd_wd; end if;
            sd_lfd <= sd_lf(to_integer(x5_sl));
        end if;
    end process p_sdlf;

    -- sticker chroma lit by the face's light level: a ROM, no engine work
    p_sdcv : process(clk)
    begin
        if rising_edge(clk) then
            -- full-saturation sticker chroma: the luma carries all the
            -- shading (a light-stepped chroma read as flicker)
            case to_integer(x13_ca) is
                when 1      => sd_cvd <= x"A316";
                when 2      => sd_cvd <= x"CA73";
                when 3      => sd_cvd <= x"D43D";
                when 4      => sd_cvd <= x"3B7C";
                when 5      => sd_cvd <= x"52BD";
                when others => sd_cvd <= x"8080";
            end case;
        end if;
    end process p_sdcv;

    -- face descriptor, read with the owner slot straight out of stage A
    p_sdfd : process(clk)
    begin
        if rising_edge(clk) then
            if sd_we(3) = '1' then sd_fd(to_integer(sd_wa)) <= sd_wd; end if;
            sd_fdd <= sd_fd(to_integer(a_own));
        end if;
    end process p_sdfd;

    -- sticker state + turn tables: one read port, the engine's in blanking
    -- and the pixel path's (stage C, the sticker under the pixel) otherwise
    p_tbl : process(clk)
        variable v_ta : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if tb_we = '1' then tbl(to_integer(tb_wa)) <= tb_wd; end if;
            if u_done = '0' then v_ta := tb_era;
            else                 v_ta := px_ta; end if;
            tb_rd <= tbl(to_integer(v_ta));
        end if;
    end process p_tbl;

    -- u_slv holds still until the next SLW, several clocks after this write
    w_wd <= std_logic_vector(u_slv);

    p_wram_w : process(clk)
    begin
        if rising_edge(clk) then
            if w_we = '1' then
                wram(to_integer(w_wa)) <= w_wd;
            end if;
        end if;
    end process p_wram_w;

    p_wram_r : process(clk)
    begin
        if rising_edge(clk) then
            w_rd <= wram(to_integer(w_ra));
        end if;
    end process p_wram_r;

    ------------------------------------------------------------------------
    -- MATERIAL ROM ports.  Address combinational, data registered -- one
    -- cycle, the same shape as every other memory here.
    ------------------------------------------------------------------------
    -- Squared-distance tables.  The zone mux lives HERE rather than in the
    -- pixel process: reading the stage-C outputs directly keeps the data
    -- one cycle behind them, exactly where the old multipliers put it.
    p_sqromu : process(clk)
        variable v : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if e_stz = '1' then v := e_qsu;
            else v := e_gu; end if;
            sq_du <= C_SQA(to_integer(v(8 downto 1)));
        end if;
    end process p_sqromu;

    p_sqromv : process(clk)
        variable v : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if e_stz = '1' then v := e_qsv;
            else v := e_gv; end if;
            sq_dv <= C_SQB(to_integer(v(8 downto 1)));
        end if;
    end process p_sqromv;

    p_tromu : process(clk)
    begin
        if rising_edge(clk) then
            tr_du <= C_TILTA(to_integer(e_tu));
        end if;
    end process p_tromu;

    p_tromv : process(clk)
    begin
        if rising_edge(clk) then
            tr_dv <= C_TILTB(to_integer(e_tv));
        end if;
    end process p_tromv;

    p_srom2 : process(clk)
    begin
        if rising_edge(clk) then
            sp_d <= C_SPEC(to_integer(sp_a));
        end if;
    end process p_srom2;

    -- unlit albedo table (stage J read, stage K fabric register)
    p_yrom : process(clk)
    begin
        if rising_edge(clk) then
            y_rd <= C_YROM(to_integer(x9_ca & x9_ast));
        end if;
    end process p_yrom;

    -- gamma: one port for the pixel path, one for the engine
    p_gamrom : process(clk)
    begin
        if rising_edge(clk) then
            gm_d <= C_GAM(to_integer(gm_a));
        end if;
    end process p_gamrom;


    s_cy <= resize(s_hf(12 downto 1), 12);

    gm_a <= gm_ap;

    mu_idle <= not mu_bsy;

    -- the ONE quarter-sine ROM read port
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            t_srd <= resize(C_SIN(to_integer(t_sk)), 14);
        end if;
    end process p_srom;




    ------------------------------------------------------------------------
    -- PER-FACE AFFINE DDAs.  The camera is orthographic, so each visible
    -- face's (u, v) is affine in screen x/y and runs as a plain accumulator:
    -- +gx per active pixel, then once per line (after avid falls) +wrap,
    -- wrap = gy*ystep - W*gx, which lands it on x = 0 of the next line.  All
    -- six run in parallel, and which face owns a pixel is simply whose (u, v)
    -- lies inside the square: the faces of a convex solid never overlap on
    -- screen.  This replaces the line engine, the display list and the
    -- prefetcher -- no spans, no edge tables, no per-line multiplies.
    --
    -- The adder only ever adds gx.  To add a wrap (or the field seed after
    -- the frame reset) the sequencer parks that value in gx for one step and
    -- then restores gx from the table, so the accumulator has no input mux.
    ------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_a : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            -- sequencer: t = 2a + k, k=0 issues a's wrap/seed, k=1 its gx
            -- Exactly s_Wl steps per line, from the first active pixel --
            -- carried on into the front porch if a line comes up short, cut
            -- off if it runs long.  An analog source's line lengths wander
            -- by a pixel; stepping per ACTIVE pixel let that error pile up
            -- down the screen (a wavy cube), and a width measured on one odd
            -- line sheared the whole field.
            dd_wrap <= '0';
            if g_avid = '1' and g_avid_q = '0' then
                dd_x   <= to_unsigned(1, 12);
                dd_run <= '1';
            elsif dd_run = '1' then
                if dd_x >= s_Wl then
                    dd_run  <= '0';
                    dd_wrap <= '1';
                else
                    dd_x <= dd_x + 1;
                end if;
            end if;

            dq_v1 <= '0';
            if dd_wrap = '1' or dd_kick = '1' then
                dq_run  <= '1';
                dq_seed <= dd_kick;
                dq_t    <= (others => '0');
            elsif dq_run = '1' then
                if dq_t(0) = '0' then
                    w_ra <= "000" & dq_seed & dq_t(4 downto 1);
                else
                    w_ra <= "0010" & dq_t(4 downto 1);
                end if;
                dq_v1 <= '1';
                dq_k1 <= dq_t(0);
                dq_a1 <= dq_t(4 downto 1);
                if dq_t = 23 then dq_run <= '0'; end if;
                dq_t <= dq_t + 1;
            end if;
            -- the table answers a cycle after the address
            dq_v2 <= dq_v1;
            dq_k2 <= dq_k1;
            dq_a2 <= dq_a1;
            dq_st <= '0';
            if dq_v2 = '1' then
                v_a := to_integer(dq_a2);
                if v_a < 12 then dd_gx(v_a) <= signed(w_rd(26 downto 0)); end if;
                dq_st <= not dq_k2;
            end if;
            dq_a3 <= dq_a2;

            -- reset when the engine starts (in blanking); it kicks the new
            -- field's seeds in when it finishes
            for a in 0 to 11 loop
                if s_estart = '1' then
                    dd_acc(a) <= (others => '0');
                elsif (g_avid = '1' and g_avid_q = '0')
                      or (dd_run = '1' and dd_x < s_Wl)
                      or (dq_st = '1' and to_integer(dq_a3) = a) then
                    dd_acc(a) <= dd_acc(a) + dd_gx(a);
                end if;
            end loop;
        end if;
    end process p_dda;

    ------------------------------------------------------------------------
    -- background: gradient + horizontal vignette, all accumulators
    ------------------------------------------------------------------------
    -- backdrop: a soft vertical luma sweep over a flat cool grey (the
    -- horizontal vignette and chroma drift went to make room for the turns)
    -- backdrop: a flat cool grey.  The vertical sweep went for timing: a
    -- CONSTANT backdrop luma folds away the silhouette blend's subtract and
    -- the backdrop pipeline registers (~116 LC at 98% full, seed odds 1/8 ->
    -- 3/8; a cheaper sweep saved only 52 LC and nothing in odds).
    p_bg : process(clk)
    begin
        if rising_edge(clk) then
            bg_vy <= to_unsigned(530, 10);
            bg_u <= to_unsigned(508, 10);
            bg_v <= to_unsigned(514, 10);
        end if;
    end process p_bg;

    ------------------------------------------------------------------------
    -- PIXEL PIPE (stages A..L + output reg = C_LATENCY 13)
    ------------------------------------------------------------------------

    -- the sticker under the pixel: base + 3 v cell + u cell (stage C)
    px_ta <= resize(c_base + resize(c_cv & '0', 6) + resize(c_cv, 6)
                    + resize(c_cu, 6), 8);

    p_pix : process(clk)
        variable v_off  : signed(15 downto 0);
        variable v_apu, v_apv : signed(14 downto 0);
        variable v_mu, v_mv : unsigned(8 downto 0);
        variable v_du, v_dv : signed(12 downto 0);
        variable v_outu, v_outv, v_su, v_sv : std_logic;
        variable v_qsu, v_qsv : signed(13 downto 0);
        variable v_gqu, v_gqv : unsigned(8 downto 0);
        variable v_ad   : signed(13 downto 0);
        variable v_am   : signed(16 downto 0);
        variable v_str  : signed(12 downto 0);
        variable v_ch   : signed(13 downto 0);
        variable v_qcu, v_qcv : unsigned(8 downto 0);
        variable v_w    : signed(27 downto 0);
        variable v_w8   : unsigned(25 downto 0);
        variable v_a    : signed(15 downto 0);
        variable v_a5   : unsigned(4 downto 0);
        variable v_bgy  : unsigned(9 downto 0);
        variable v_dy   : signed(11 downto 0);
        variable v_c    : signed(16 downto 0);
        variable v_c19  : signed(19 downto 0);
        variable v_cm1  : unsigned(9 downto 0);
        variable v_hh   : signed(9 downto 0);
        variable v_del  : signed(11 downto 0);
        variable v_nd   : signed(12 downto 0);
        variable v_sed  : unsigned(7 downto 0);
        variable v_dsx  : signed(10 downto 0);
        variable v_ramp : signed(10 downto 0);
        variable v_li   : signed(13 downto 0);
        variable v_dir  : signed(13 downto 0);
        variable v_occ  : signed(13 downto 0);
        variable v_t8   : signed(8 downto 0);
        variable v_sil4 : std_logic_vector(3 downto 0);
        variable v_face : integer range 0 to 5;
        variable v_d12  : unsigned(11 downto 0);
        variable v_q12  : signed(11 downto 0);
        variable v_ng   : std_logic;
        variable v_cl   : unsigned(1 downto 0);
        variable v_c20  : signed(19 downto 0);
        variable v_os   : signed(13 downto 0);
        variable v_my   : unsigned(9 downto 0);
        variable v_ndh  : unsigned(7 downto 0);
        variable v_s6   : integer range 0 to 5;
        variable v_oh   : std_logic_vector(5 downto 0);
        variable v_cr, v_cp, v_f : std_logic_vector(5 downto 0);
        variable v_nu, v_nv : unsigned(1 downto 0);
        variable v_dl   : unsigned(11 downto 0);
        variable v_n1   : std_logic;
        variable v_t17  : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then

            ------------------------------------------------------ stage A
            -- Affine UV straight off the per-face DDAs (p_dda).  Ownership
            -- flags come from the PREVIOUS pixel (registered off the
            -- accumulators) and (u, v) from this one: a one-pixel lag on the
            -- ownership edge only, which lands inside the silhouette inset or
            -- on the crease's own bevel.  Slots are ordered near box first,
            -- so the first owner is also the front one.
            for s in 0 to 5 loop
                v_cr(s) := '0';  v_cp(s) := '0';
                if g_avid = '1' and to_unsigned(s, 3) < s_nslot then
                    if f_inside(dd_acc(2*s), dd_e(2*s), sl_nu(s))
                       and f_inside(dd_acc(2*s + 1), dd_e(2*s + 1), sl_nv(s)) then
                        v_cr(s) := '1';
                    end if;
                    if f_inpad(dd_acc(2*s), dd_e(2*s), sl_nu(s))
                       and f_inpad(dd_acc(2*s + 1), dd_e(2*s + 1), sl_nv(s)) then
                        v_cp(s) := '1';
                    end if;
                end if;
            end loop;
            -- the owner, encoded here so the next stage only muxes: first
            -- strict claimant, else first padded one -- both encoders in
            -- parallel, then one select (a priority chain was 14 LUTs deep)
            if v_cr /= "000000" then a_own <= f_first6(v_cr); a_po <= '0';
            else                     a_own <= f_first6(v_cp); a_po <= '1'; end if;
            if v_cr /= "000000" or v_cp /= "000000" then a_any <= '1';
            else a_any <= '0'; end if;
            -- a second strict claimant lies behind the owner, so the owner's
            -- silhouette must not blend to the backdrop there
            if f_two6(v_cr) then a_bh <= '1'; else a_bh <= '0'; end if;

            if g_avid = '1' and a_any = '1' then
                v_s6 := to_integer(a_own);
                b_u <= f_uvsl(dd_acc(2*v_s6), dd_e(2*v_s6));
                b_v <= f_uvsl(dd_acc(2*v_s6 + 1), dd_e(2*v_s6 + 1));
                b_sl <= a_own;
                b_on <= '1';
                b_bh <= a_bh;
                b_po <= a_po;
            else
                b_on <= '0';
                b_bh <= '0';
                b_po <= '0';
            end if;

            ------------------------------------------------------ stage B
            -- Cell split only (p_cell): every boundary distance the shader
            -- needs -- cubie gap AND face bevel -- is 2048 - |p| off the
            -- in-cubie coordinate, because the face edge is simply the
            -- outermost grid line.
            -- the owner's face descriptor arrived with this stage
            case to_integer(unsigned(sd_fdd(9 downto 7))) is
                when 1 => v_nu := "11"; v_nv := "01";
                when 2 => v_nu := "01"; v_nv := "11";
                when 3 => v_nu := "11"; v_nv := "10";
                when 4 => v_nu := "10"; v_nv := "11";
                when others => v_nu := "11"; v_nv := "11";
            end case;
            p_cell(b_u, v_nu, v_d12, v_q12, v_ng, v_cl);
            c_du <= v_d12;  c_qu <= v_q12;  c_ngu <= v_ng;  c_cu <= v_cl;
            p_cell(b_v, v_nv, v_d12, v_q12, v_ng, v_cl);
            c_dv <= v_d12;  c_qv <= v_q12;  c_ngv <= v_ng;  c_cv <= v_cl;
            c_nu <= v_nu;  c_nv <= v_nv;
            c_base <= unsigned(sd_fdd(15 downto 10));
            c_gap  <= unsigned(sd_fdd(6 downto 4));
            c_sil  <= sd_fdd(3 downto 0);
            c_sl <= b_sl;
            c_on <= b_on;
            c_bh <= b_bh;
            c_po <= b_po;

            ------------------------------------------------------ stage C
            v_sil4 := c_sil;
            v_du := signed(resize(c_du, 13));
            v_dv := signed(resize(c_dv, 13));

            -- Is the nearest line the face outline or a seam?  An outline edge
            -- is a silhouette (rounds away 80 degrees, anti-aliased), a crease
            -- (the rounded corner of the solid, 45 degrees to meet the next
            -- face), or a CUT edge beside the other box -- a layer gap, drawn
            -- exactly like the seams between cubies.
            v_outu := '0';  v_su := '0';
            if c_cu = "00" and c_ngu = '1' and c_gap /= 1 then
                v_outu := '1';  v_su := v_sil4(0);
            elsif c_cu = c_nu - 1 and c_ngu = '0' and c_gap /= 2 then
                v_outu := '1';  v_su := v_sil4(1);
            end if;
            v_outv := '0';  v_sv := '0';
            if c_cv = "00" and c_ngv = '1' and c_gap /= 3 then
                v_outv := '1';  v_sv := v_sil4(2);
            elsif c_cv = c_nv - 1 and c_ngv = '0' and c_gap /= 4 then
                v_outv := '1';  v_sv := v_sil4(3);
            end if;

            if v_outu = '1' and v_du < C_RB then
                v_mu := resize(unsigned(resize(to_signed(C_RB, 13) - v_du, 9)), 9);
            else v_mu := (others => '0'); end if;
            if v_outv = '1' and v_dv < C_RB then
                v_mv := resize(unsigned(resize(to_signed(C_RB, 13) - v_dv, 9)), 9);
            else v_mv := (others => '0'); end if;
            if v_outu = '0' and v_du < C_GB then
                v_gqu := resize(unsigned(resize(to_signed(C_GB, 13) - v_du, 9)), 9);
            else v_gqu := (others => '0'); end if;
            if v_outv = '0' and v_dv < C_GB then
                v_gqv := resize(unsigned(resize(to_signed(C_GB, 13) - v_dv, 9)), 9);
            else v_gqv := (others => '0'); end if;
            e_mu <= v_mu;  e_mv <= v_mv;
            e_gu <= v_gqu; e_gv <= v_gqv;

            -- Bevel tilt index: every rounded zone rescales onto ONE profile
            -- table (>>1 for a silhouette bevel and *0.75 for a gap -> index
            -- 164 = 80 degrees; a crease stops at 45, index ~92, where the
            -- neighbouring face's rounding takes over).  Outward direction is
            -- the sign of p, one bit.
            if v_outu = '1' and v_su = '1' then
                e_tu <= resize(shift_right(v_mu, 1), 8);
            elsif v_outu = '1' then
                e_tu <= resize(shift_right(v_mu, 2) + shift_right(v_mu, 5), 8);
            else
                e_tu <= resize(shift_right(v_gqu, 1)
                               + shift_right(v_gqu, 2), 8);
            end if;
            if v_outv = '1' and v_sv = '1' then
                e_tv <= resize(shift_right(v_mv, 1), 8);
            elsif v_outv = '1' then
                e_tv <= resize(shift_right(v_mv, 2) + shift_right(v_mv, 5), 8);
            else
                e_tv <= resize(shift_right(v_gqv, 1)
                               + shift_right(v_gqv, 2), 8);
            end if;
            e_su <= c_ngu;
            e_sv <= c_ngv;
            if v_du < v_dv then e_tmax <= resize(shift_right(v_gqu, 1)
                                                 + shift_right(v_gqu, 2), 8);
            else                e_tmax <= resize(shift_right(v_gqv, 1)
                                                 + shift_right(v_gqv, 2), 8);
            end if;

            -- straight silhouette distance = distance to the nearest outline
            -- edge that actually is a silhouette
            v_str := to_signed(4095, 13);
            if v_su = '1' then v_str := v_du; end if;
            if v_sv = '1' and v_dv < v_str then v_str := v_dv; end if;
            -- a pixel claimed only through the padding is past its face's
            -- true edge; at a silhouette it shows as a sliver sticking out
            -- where the neighbour's corner is bevelled away, so it goes to
            -- the backdrop.  (Crease holes, the padding's job, are interior.)
            if c_po = '1' and (v_su = '1' or v_sv = '1') then
                e_dstr <= to_signed(-2048, 13);
            else
                e_dstr <= v_str;
            end if;
            e_chs <= resize(v_du, 14) + resize(v_dv, 14);
            e_chz <= v_outu and v_outv and (v_su or v_sv);
            e_cact <= v_su or v_sv;

            -- sticker rounded rect off the clamped corner coordinates
            v_qsu := resize(c_qu, 14);
            v_qsv := resize(c_qv, 14);
            if v_qsu < 0 then v_qcu := (others => '0');
            elsif v_qsu > 511 then v_qcu := to_unsigned(511, 9);
            else v_qcu := resize(unsigned(v_qsu(8 downto 0)), 9); end if;
            if v_qsv < 0 then v_qcv := (others => '0');
            elsif v_qsv > 511 then v_qcv := to_unsigned(511, 9);
            else v_qcv := resize(unsigned(v_qsv(8 downto 0)), 9); end if;
            e_qsu <= v_qcu;
            e_qsv <= v_qcv;
            if v_qcu > v_qcv then
                e_dst <= to_signed(C_RS, 11) - signed(resize(v_qcu, 11));
            else
                e_dst <= to_signed(C_RS, 11) - signed(resize(v_qcv, 11));
            end if;

            -- zone select (bevel / gap / sticker are mutually exclusive)
            if v_mu > 0 or v_mv > 0 then
                e_silz <= '1'; e_stz <= '0';
            elsif v_gqu > 0 or v_gqv > 0 then
                e_silz <= '0'; e_stz <= '0';
            elsif c_base = 63 then                   -- a cut face: bare plastic
                e_silz <= '0'; e_stz <= '0';
            else
                e_silz <= '0'; e_stz <= '1';
            end if;
            e_stc <= '0';
            if v_qsu > 0 and v_qsv > 0 then e_stc <= '1'; end if;
            e_sl <= c_sl;
            e_on <= c_on;
            e_bh <= c_bh;

            ------------------------------------------------------ stage D
            -- squared distance / tilt profile / descriptor block-RAM reads
            -- are in flight (their ports take the stage-C registers)
            f_dst <= e_dst;
            -- a slight 45-degree chamfer on outline corners, small enough
            -- that the faces' in-plane cuts can't visibly disagree: the
            -- line du + dv = K (+2 insets), distance taken at half scale
            v_ch := resize(shift_right(e_chs, 1), 14) - to_signed(C_CRH, 14);
            if e_chz = '1' and v_ch < resize(e_dstr, 14) then
                f_dstr <= resize(v_ch, 13);
            else
                f_dstr <= e_dstr;
            end if;
            f_cact <= e_cact;
            if e_mu > 0 and e_mv > 0 then f_2m <= '1'; else f_2m <= '0'; end if;
            f_silz <= e_silz;
            f_stz <= e_stz;
            f_stc <= e_stc;
            f_sl <= e_sl;
            f_on <= e_on;
            f_su <= e_su;
            f_sv <= e_sv;
            f_tmax <= e_tmax;
            f_col <= unsigned(tb_rd(2 downto 0));   -- sticker colour (px_ta)
            f_bh <= e_bh;

            ------------------------------------------------------ stage E
            -- Block-RAM outputs land here.  Each gets at most one short op so
            -- the multiplies next stage start from fabric registers.
            v_cm1 := resize(tr_du(7 downto 0), 10) + resize(tr_dv(7 downto 0), 10);
            if v_cm1 > 255 then v_cm1 := to_unsigned(255, 10); end if;
            -- the rig multiplies run at 6 x 8 bits (1/16 scale, restored at
            -- stage G): a bevel is a few pixels wide, 64 tilt levels suffice
            x5_cm1 <= v_cm1(7 downto 2);
            x5_snu <= tr_du(15 downto 10);
            x5_snv <= tr_dv(15 downto 10);
            v_hh := signed(std_logic_vector'(sd_h2d(3 downto 0) & sd_h1d(5 downto 0)));
            x5_hn <= v_hh;
            x5_hn8 <= v_hh(9 downto 2);
            v_hh := signed(sd_h1d(15 downto 6));
            if f_su = '1' then v_hh := -v_hh; end if;
            x5_hu <= v_hh(9 downto 2);
            v_hh := signed(sd_h2d(15 downto 6));
            if f_sv = '1' then v_hh := -v_hh; end if;
            x5_hv <= v_hh(9 downto 2);
            -- R^2 - (x^2 + y^2) for whichever rounded corner this zone uses
            if f_stz = '1' then v_c20 := to_signed(C_RS2, 20);
            else                v_c20 := to_signed(C_RB2, 20); end if;
            x5_c19 <= v_c20 - signed(resize(sq_du, 20)) - signed(resize(sq_dv, 20));
            x5_tmax <= f_tmax;
            x5_dst <= f_dst;
            x5_dstr <= f_dstr;
            x5_cact <= f_cact;
            x5_2m <= f_2m;
            x5_silz <= f_silz;
            x5_stz <= f_stz;
            x5_stc <= f_stc;
            x5_sl <= f_sl;
            x5_on <= f_on;
            x5_col <= f_col;
            x5_bh <= f_bh;

            ------------------------------------------------------ stage F
            -- SPECULAR / TILT DOT PRODUCT.  N = cos(phi)*n + sin(phi)*dir, and
            -- the camera is orthographic, so one half-vector serves both the
            -- diffuse tilt and the highlight:
            --   N.H = H.n + [ -(1-cos)*H.n + sin_u*H.U + sin_v*H.V ] / 256
            -- Three 9x10 multiplies -- the whole material rig -- each alone.
            x6_p0 <= resize(bmul(x5_hn8, signed('0' & x5_cm1)), 15);
            x6_p1 <= resize(bmul(x5_hu, signed('0' & x5_snu)), 15);
            x6_p2 <= resize(bmul(x5_hv, signed('0' & x5_snv)), 15);
            -- Linearize the squared distance near its zero crossing: the
            -- 1/(2r) factors are shift-adds, x*25>>14 and x*24>>14.
            -- Only the sticker zone's form is computed: x6_dlin is read
            -- solely as the sticker corner arc (stage G, x6_stz = '1'), so
            -- the bevel/gap form (c >> 6 + c >> 8 + c >> 11) was never seen.
            -- Written without sign extension.  x5_c19 always fits 18 bits
            -- (a 15-bit constant less two 16-bit squares), so yosys makes
            -- bits 19..17 one net, and two sign-extended shifts of it put
            -- that net on both inputs of an adder bit -- the router1
            -- livelock.  With S = c(17): c >> j (18-bit signed) is
            -- c(16..j) - S * 2^(17-j), and both -S * 2^k fold into free
            -- high bits of the narrower field, so it stays one adder and
            -- exact mod 2^12:
            --   (c >> 7) + (c >> 8) = c(16..7) + (S0S & c(16..8))
            v_n1 := x5_c19(17);
            v_t17 := unsigned(x5_c19(16 downto 0));
            v_dl := resize(v_t17(16 downto 7), 12)
                    + (v_n1 & '0' & v_n1 & v_t17(16 downto 8));
            x6_dlin <= signed(v_dl);
            -- crevice AO: x^2+y^2 - CBW^2 = (RB^2 - CBW^2) - c19 in the gaps
            if x5_silz = '1' or x5_stz = '1' or x5_c19 >= C_RB2 - C_CBW2 then
                x6_ao <= (others => '0');
            else
                v_c20 := shift_right(to_signed(C_RB2 - C_CBW2, 20) - x5_c19, 5);
                if v_c20 > 255 then x6_ao <= to_unsigned(255, 8);
                else x6_ao <= unsigned(v_c20(7 downto 0)); end if;
            end if;
            x6_hn <= x5_hn;
            x6_tmax <= x5_tmax;
            x6_dst <= x5_dst;
            x6_dstr <= x5_dstr;
            x6_cact <= x5_cact;
            x6_2m <= x5_2m;
            x6_silz <= x5_silz;
            x6_stz <= x5_stz;
            x6_stc <= x5_stc;
            x6_sl <= x5_sl;
            x6_on <= x5_on;
            x6_col <= x5_col;
            x6_bh <= x5_bh;

            ------------------------------------------------------ stage G
            -- the tilt's signed departure from the flat face value
            x7_del <= resize(shift_right(resize(x6_p1, 16) + resize(x6_p2, 16)
                                         - resize(x6_p0, 16), 4), 12);

            -- CRISP outline corners: the silhouette is the straight edges
            -- only.  Every face runs to the cube's true vertex, so two or
            -- three faces always meet at ONE point (an in-plane corner arc
            -- per face disagreed on steep faces and left spikes); the
            -- rounding reads through the bevel shading, as at the seams.
            x7_dsil <= resize(x6_dstr, 14) - to_signed(C_INSET, 14);
            if x6_stz = '0' then
                x7_ds <= to_signed(-1024, 11);       -- bevel/gap: plastic only
            elsif x6_stc = '1' then
                x7_ds <= resize(x6_dlin, 11);        -- sticker corner arc
            else
                x7_ds <= x6_dst;                     -- sticker straight edge
            end if;


            -- crevice occlusion (the model's 3/4 at full gap depth)
            v_occ := resize(signed('0' & (shift_right(x6_ao, 3)
                                          + shift_right(x6_ao, 5)
                                          + shift_right(x6_ao, 6))), 14);
            -- the tilt-independent halves of stage H's sums, so stage H is one
            -- add, one select and a clamp
            x7_lfa <= signed(resize(unsigned(sd_lfd(7 downto 0)), 10)) - to_signed(C_AMB, 10);
            x7_p1 <= signed(resize(unsigned(sd_lfd(7 downto 0)), 10)) - resize(v_occ, 10);
            x7_p2 <= to_signed(C_AMB, 10) - resize(v_occ, 10);
            x7_q  <= resize(x6_hn, 11) - signed(resize(x6_ao(7 downto 1), 11));
            x7_col <= x6_col;
            x7_bh <= x6_bh;
            x7_sl <= x6_sl;
            x7_on <= x6_on;

            ------------------------------------------------------ stage H
            -- the highlight: N.H less the crevice occlusion, clamped
            --   (H.n - ao/2) + tilt
            v_li := resize(x7_q, 14) + resize(x7_del, 14);
            if v_li < 0 then sp_a <= (others => '0');
            elsif v_li > 255 then sp_a <= to_unsigned(255, 8);
            else sp_a <= unsigned(v_li(7 downto 0)); end if;

            -- LIGHT INDEX, linear Q8.  Only the DIRECTIONAL part of the face
            -- light may be turned away by the bevel: ambient is held out of
            -- the tilt and floors the shading, else every surface tilting
            -- away from the key crushes to black instead of dark grey.
            --   li = AMB + max(lf - AMB + tilt, 0) - occ
            --      = (lf - AMB + tilt >= 0) ? (lf - occ) + tilt : AMB - occ
            v_dir := resize(x7_lfa, 14) + resize(x7_del, 14);
            if v_dir < 0 then v_li := resize(x7_p2, 14);
            else              v_li := resize(x7_p1, 14) + resize(x7_del, 14); end if;
            if v_li < 0 then gm_ap <= (others => '0');
            elsif v_li > 255 then gm_ap <= to_unsigned(255, 8);
            else gm_ap <= unsigned(v_li(7 downto 0)); end if;

            -- Coverage, part 1: scale the distances by the AA slope, a frame
            -- constant rounded to a power of two (within 10% in every video
            -- mode).  The shift comes FIRST, on the narrow distance: under one
            -- coverage LSB, and it keeps the barrel out of the clamp stage.
            v_ad := shift_right(x7_dsil, s_pxe);
            x8_ams <= shift_left(resize(v_ad, 17), 2);
            v_ad := shift_right(resize(x7_ds, 14), s_pxe);
            x8_amt <= shift_left(resize(v_ad, 17), 2);
            x8_ca <= x7_col;
            x8_bh <= x7_bh;
            x8_sl <= x7_sl;
            x8_on <= x7_on;

            ------------------------------------------------------ stage I
            -- Coverage, part 2: the +/-8 ramp and its clamp.
            v_a := resize(shift_right(x8_ams, 3), 16) + 8;
            if x8_on = '0' or v_a < 0 then v_a5 := (others => '0');
            elsif v_a > 16 then v_a5 := to_unsigned(16, 5);
            else v_a5 := resize(unsigned(v_a(4 downto 0)), 5); end if;
            -- over the other box, not the backdrop: a hard edge beats a halo
            if x8_on = '1' and x8_bh = '1' then v_a5 := to_unsigned(16, 5); end if;
            x9_asil <= v_a5;
            x9_ca <= x8_ca;
            v_a := resize(shift_right(x8_amt, 3), 16) + 8;
            if v_a < 0 then v_a5 := (others => '0');
            elsif v_a > 16 then v_a5 := to_unsigned(16, 5);
            else v_a5 := resize(unsigned(v_a(4 downto 0)), 5); end if;
            x9_ast <= v_a5;
            x9_sl <= x8_sl;

            ------------------------------------------------------ stage J
            -- LUMA carries all the shading; CHROMA is a per-face constant hard-
            -- switched at the sticker edge (the encoder subsamples it 4:2:2).
            -- Gamma and specular block-RAM outputs land here.
            x10_gam <= gm_d;
            -- specular: the glossier sticker sheen or the plastic one, switched
            -- where the chroma switches (the blend's multiply went for area)
            x10_spl <= sp_d(7 downto 0);       -- plastic sheen only
            -- unlit albedo: satin plastic blended to the sticker over its AA edge
            -- (C_YROM is read here: p_yrom, address x9_ca & x9_ast)
            if x9_ast >= 8 then x10_ston <= '1'; else x10_ston <= '0'; end if;
            x10_ast <= x9_ast;
            x10_asil <= x9_asil;
            x10_sl <= x9_sl;
            x10_ca <= x9_ca;

            ------------------------------------------------------ stage K
            x11_y <= y_rd;
            x11_gam <= x10_gam;
            x11_spl <= x10_spl;
            x11_asil <= x10_asil;
            x11_ston <= x10_ston;
            x11_sl <= x10_sl;
            x11_ca <= x10_ca;

            ------------------------------------------------------ stage L
            -- albedo * light
            x12_pm <= bmul_u(x11_y, x11_gam);
            -- matte stickers (sheen fixed at 0); the black plastic shines
            if x11_ston = '1' then x12_spec <= (others => '0');
            else x12_spec <= resize(shift_left(resize(x11_spl, 12), 2), 12); end if;
            x12_asil <= x11_asil;
            x12_ston <= x11_ston;
            x12_sl <= x11_sl;
            x12_ca <= x11_ca;

            ------------------------------------------------------ stage M
            -- add the highlight (already in video units), then take the
            -- distance to the backdrop for the silhouette blend
            v_my := f_clamp10(resize(signed('0' & shift_right(x12_pm, 8)), 14)
                              + resize(signed('0' & x12_spec), 14));
            -- S11: the backdrop is the incoming video instead of the sweep
            -- (its luma enters HERE so the silhouette blends into it)
            if s_sw(4) = '1' then v_bgy := unsigned(data_in.y(9 downto 2)) & "00";
            else                  v_bgy := bg_vy; end if;
            x13_dy <= signed(resize(v_my, 11)) - signed(resize(v_bgy, 11));
            x13_bg <= v_bgy;
            x13_asil <= x12_asil;
            x13_ston <= x12_ston;
            x13_sl <= x12_sl;
            x13_ca <= x12_ca;

            ------------------------------------------------------ stage N
            x14_pn <= resize(bmul(x13_dy, signed(resize(x13_asil, 6))), 17);
            x14_bg <= x13_bg;
            x14_asil <= x13_asil;
            x14_ston <= x13_ston;
            if x13_ston = '1' and s_vsel /= 0
               and (s_vsel = 7 or x13_ca = s_vsel - 1) then x14_vid <= '1';
            else x14_vid <= '0'; end if;

            ------------------------------------------------------ stage O
            n_y <= f_clamp10(signed(resize(x14_bg, 12))
                             + resize(shift_right(x14_pn, 4), 12));
            if x14_asil < 8 then
                if s_sw(4) = '1' then          -- video backdrop, pre-swapped
                    n_u <= unsigned(data_in.v(9 downto 2)) & "00";
                    n_v <= unsigned(data_in.u(9 downto 2)) & "00";
                else
                    n_u <= bg_u;
                    n_v <= bg_v;
                end if;
            elsif x14_vid = '1' then
                -- incoming video, straight in (a fixed 16-pixel offset nobody
                -- can see through a sticker window), pre-swapped so the output
                -- swap hands the encoder data_in's own U/V: the dry path is
                -- never swapped
                n_y <= unsigned(data_in.y(9 downto 2)) & "00";
                n_u <= unsigned(data_in.v(9 downto 2)) & "00";
                n_v <= unsigned(data_in.u(9 downto 2)) & "00";
            elsif x14_ston = '1' then
                n_u <= unsigned(sd_cvd(7 downto 0)) & "00";
                n_v <= unsigned(sd_cvd(15 downto 8)) & "00";
            else
                n_u <= to_unsigned(512, 10);       -- black plastic: neutral
                n_v <= to_unsigned(512, 10);
            end if;

            ------------------------------------------------------ output
            if s_avid_sr(C_LATENCY - 2) = '1' then
                o_y <= n_y;
                o_u <= n_v;      -- U/V swapped for hardware
                o_v <= n_u;
            else
                o_y <= to_unsigned(64, 10);
                o_u <= to_unsigned(512, 10);
                o_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= g_avid          & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture cubist;
