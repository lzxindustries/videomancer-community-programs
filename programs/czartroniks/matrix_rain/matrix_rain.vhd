-- matrix_rain.vhd
-- Copyright (C) 2026  czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Matrix Rain - classic "digital rain" generator for Videomancer.
--
-- A black field is divided into square glyph cells (16<<zoom px).  Every
-- column carries THREE independent falling drops.  Each drop advances once per
-- frame at its own speed (re-rolled at every respawn), draws a bright
-- near-white head glyph and a tail that fades smoothly to black over the tail
-- length, then waits a random gap before falling again.  Density sets the gap
-- length and the chance a respawned drop is visible, so it never switches
-- columns on or off mid-fall.  Glyphs are a static random field the rain
-- reveals: the head glyph shimmers every other frame, a few "flicker" cells
-- re-roll fast, the rest re-roll slowly (Mutation sets rate).
--
-- Image (P12) is the strength of the input picture's hold on the rain.  The
-- rain keeps falling everywhere; under bright parts of the picture glyphs grow
-- (up to 2x wide, bold) and tails stretch (up to ~4x, so even sparse rain
-- fills and lingers there); under dark parts glyphs shrink to 0.4x with normal
-- tails.  0% = plain rain.  Background (S10) is black or the input video.
--
-- v1.0.2 (2026-10-18): resolution scaling.  The glyph grid, luma map and
-- glitch bands run in virtual 1080-line units (x step 1/1.5/2 per pixel,
-- y step per field line from the timing ID), so 720p and SD show the 1080p
-- glyph sizes relative to screen height, square on a 4:3 SD screen (SD = a
-- 1440-unit-wide window).  1080p is unchanged.
--
-- v1.0.3 (2026-10-18): fit (stock SDK SD Standalone 7821 -> 7175 LC), same
-- picture bit for bit: no_rw_check on the collision-free RAMs, and the
-- respawn draws share s_rnd (mix16 is linear) and one multiplier.
--
-- Glyph ROM: matrix_rain_glyphs_pkg.vhd (16x16 1bpp), generated at build time
-- by matrix_rain.py from the vendored GNU Unifont subset (half-width katakana
-- U+FF66..FF9D + digits + a few Latin/symbols).
--
-- =====================================================================
-- Register / front-panel map  (registers_in indices, see docs/abi-format.md)
-- ---------------------------------------------------------------------
--   0x00  K1  rotary_potentiometer_1   Fall Speed        (base fall rate)
--   0x01  K2  rotary_potentiometer_2   Density           (respawn gap / visibility)
--   0x02  K3  rotary_potentiometer_3   Tail Length       (visible trail rows)
--   0x03  K4  rotary_potentiometer_4   Mutation Rate     (glyph re-roll speed)
--   0x04  K5  rotary_potentiometer_5   Colour            (0 = green, hue wheel; top step = white)
--   0x05  K6  rotary_potentiometer_6   Glyph Size        (cell = 16<<zoom: 16/32/64/128 px)
--   0x06b0 S7 toggle_switch_7          Head Glow / bloom on/off
--   0x06b1 S8 toggle_switch_8          Mirror glyphs horizontally on/off
--   0x06b2 S9 toggle_switch_9          Rise (drops travel upward)
--   0x06b3 S10 toggle_switch_10        Background: Black / Video
--   0x06b4 S11 toggle_switch_11        Glitch (tearing scrambled bands + mutation bursts)
--   0x07  P12 linear_potentiometer_12  Image (input luma -> glyph size + tail persistence)
-- =====================================================================
--
-- Colour: every lit pixel is a LEVEL L above black, times a per-colour luma
-- gain k: Y = 64 + L*k, chroma = 512 + L*k*c/512, so chroma scales with luma
-- and a dim tail really fades to black.  c and k come from C_PAL (genpal.py):
-- a 64-entry hue wheel (phosphor green at the knob centre) + white, stored
-- U/V-SWAPPED for the Videomancer hardware (stored U = Cr); k keeps the body
-- peak of dim hues (blue, red) from sitting clipped.  Body peaks at L=500,
-- the head is L=704 (Y=768, the drawn-luma cap) with half chroma.
--
-- Timing front end (hardware contracts, see CLAUDE.md):
--   * Generated avid: starts 2-3 clocks after the source's first avid rise of
--     the line at the source's USUAL hsync->avid parity (saturating vote),
--     runs a FIXED per-field pixel count, ignores source avid dropouts --
--     keeps the encoders' Cb/Cr pairing fixed (cubist v0.3.3 pattern).
--   * Frame start = the FIRST vsync edge after active video (serrated analog
--     vsync gives several per field); field height captured from the line
--     count of the generated avid, never latched from a vsync-reset counter.
--   * Output gated to black / neutral chroma whenever the generated avid is
--     low (blanking-interval contract).  One latency for every mode.
--
-- =====================================================================
-- Pipeline (one pixel per clock; M = front-end x/y/avid registers):
--   S0  p_acc      cell coords, offset from cell centre; RAM read addresses
--   --  RAM        3 head BRAMs + drop-info BRAM + luma map, registered read
--   S1  p_s1       align; Rise row flip
--   S2  p_dist     per-drop distance; luma level; glyph hash base
--   S3  p_s34      param RAM read by luma level (scale, stretched tail, fade)
--   S4  p_s34      params into fabric
--   S5  p_lit      per-drop lit / head tests against the stretched tail
--   S6  p_idx      glyph index (mutation) + nearest lit drop
--   S7  p_scale    offsets * 16/scale; fade product
--   S8  p_gcoord   scaled glyph pixel + inside test; fade level
--   S9  p_rom      glyph ROM row read
--   S10 p_pix      glyph bit + glow dilation
--   S11 p_lvl      level select * colour gain
--   S12 p_chr      Y = 64 + L; chroma partial products L*|c|_hi, L*|c|_lo
--   S12b p_chr2    chroma products = hi<<5 + lo
--   S12c p_chr3    chroma offset * mode (full / 3/4 hot rows / half head+glow)
--   S13 p_color    U/V = 512 +- offset (clamped); background; blanking gate
--   --  p_io       output registers           (M -> output = 17 clocks)
-- =====================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.matrix_rain_glyphs_pkg.all;

architecture matrix_rain of program_top is
    -- IO-align pad: 3 register stages after the program's own output so the
    -- data_in -> data_out latency (17 + 3 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 2) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK  : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- Mutation: slow cells re-roll every 2^C_MUT_SLOW / T_inc frames, the
    -- 1-in-8 "flicker" cells every 2^C_MUT_FAST / T_inc frames.
    constant C_MUT_SLOW : integer := 7;
    constant C_MUT_FAST : integer := 4;

    -- Sync passthrough depth: data_in sync at clock t leaves p_io at t+17,
    -- the same as the generated avid / pixel data from the M registers.
    constant C_SYNCD : integer := 14;

    -- Video background delay: output trails M by 17, M trails the source
    -- pixel by 4-5, ring read + fabric copy add 3.
    constant C_VDLY : integer := 18;

    -- Number of drops per column (each has its own head BRAM).
    constant C_ND : integer := 3;

    -- Head level (Y=768 cap) and glow level, in luma units above black.
    constant C_L_HEAD : unsigned(9 downto 0) := to_unsigned(704, 10);
    constant C_L_GLOW : unsigned(9 downto 0) := to_unsigned(520, 10);

    --------------------------------------------------------------------------
    -- Body fade curve, level above black indexed by 16*dist/tail (0..15):
    -- indices 0-1 are the "hot" leading rows (brighter, drawn with 3/4
    -- chroma so they read pale and bright -- saturated green is already at
    -- its ceiling near L=520); then 520*(1-(i/16)^2): holds near full
    -- brightness for the first half of the tail, then sinks into black.
    --------------------------------------------------------------------------
    type t_fade is array (0 to 15) of unsigned(9 downto 0);
    constant C_FADE : t_fade := (
        to_unsigned(620,10), to_unsigned(580,10), to_unsigned(512,10), to_unsigned(502,10),
        to_unsigned(488,10), to_unsigned(469,10), to_unsigned(447,10), to_unsigned(420,10),
        to_unsigned(390,10), to_unsigned(355,10), to_unsigned(317,10), to_unsigned(274,10),
        to_unsigned(228,10), to_unsigned(177,10), to_unsigned(122,10), to_unsigned( 63,10));


    --------------------------------------------------------------------------
    -- Colour palette ROM (one EBR pair, read once per frame's index).
    -- 0..63 hue wheel, 64 = white.
    --------------------------------------------------------------------------
    type t_pal is array (0 to 127) of std_logic_vector(31 downto 0);
    -- generated by genpal.py: [31] U sign, [30:21] |cu|, [20] V sign, [19:10] |cv|, [2:0] k
    constant C_PAL : t_pal := (
        x"54D39403", x"4C93C404", x"4553EC04", x"3ED41404", x"39143405", x"33D45405",
        x"2F347005", x"2AD48806", x"26F49C06", x"23549C06", x"1FF49C07", x"1CF49C07",
        x"1A149C07", x"17749C07", x"14F49C07", x"12B49C07", x"10949C07", x"0C949C07",
        x"08B49C07", x"04D49C07", x"00D49C07", x"83349C07", x"87349C07", x"8B149C07",
        x"8F149C07", x"92549007", x"95D48007", x"99747007", x"9D346007", x"A1345007",
        x"A5743C07", x"A9F42807", x"AEB41407", x"AEB33007", x"AEB25007", x"AEB17807",
        x"AEB0A007", x"AEA03007", x"AEA10007", x"AEA1CC07", x"AEA29007", x"ADC30C07",
        x"ACA39806", x"AB643C06", x"A9E4FC05", x"A805DC05", x"A5E6E804", x"A3683004",
        x"A029D003", x"96898C03", x"8D494C03", x"84690C03", x"0448D003", x"0CA89803",
        x"14A86003", x"1C882804", x"2407F404", x"29E69004", x"2FE52405", x"35E3B805",
        x"3C024805", x"4220D804", x"4850A004", x"4E921804", x"00000007",
        others => x"00000007");
    signal s_palrom : t_pal := C_PAL;
    attribute ram_style : string;
    attribute ram_style of s_palrom : signal is "block";

    --------------------------------------------------------------------------
    -- Helpers
    --------------------------------------------------------------------------
    function mix16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x;
        v := v xor shift_left(v, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        v := v xor shift_right(v, 5);
        return v;
    end function;

    -- shift-only 8-step gain: (sel+1)/8.
    function scale8(val : unsigned(9 downto 0); sel : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case sel is
            when "000" => r := shift_right(val, 3);
            when "001" => r := shift_right(val, 2);
            when "010" => r := shift_right(val, 2) + shift_right(val, 3);
            when "011" => r := shift_right(val, 1);
            when "100" => r := shift_right(val, 1) + shift_right(val, 3);
            when "101" => r := shift_right(val, 1) + shift_right(val, 2);
            when "110" => r := shift_right(val, 1) + shift_right(val, 2) + shift_right(val, 3);
            when others => r := val;
        end case;
        return r;
    end function;

    -- pixel at column p of a 16-bit glyph row, MSB = leftmost (col 0).
    function getbit(rb : std_logic_vector(15 downto 0); p : integer) return std_logic is
    begin
        if p >= 0 and p <= 15 then return rb(15 - p); else return '0'; end if;
    end function;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID.
    --------------------------------------------------------------------------
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

    -- respawn-draw constants mix16(5A3C xor K) (see u_um)
    function f_uk1(d : integer) return unsigned is
    begin
        return mix16(x"5A3C" xor to_unsigned(16#3A71# + d * 16#1D3#, 16));
    end function;
    function f_uk2(d : integer) return unsigned is
    begin
        return mix16(x"5A3C" xor to_unsigned(16#C4B5# + d * 16#2E9#, 16));
    end function;

    -- reverse a 16-bit row (horizontal mirror).
    function revb(rb : std_logic_vector(15 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(15 downto 0);
    begin
        for i in 0 to 15 loop r(i) := rb(15 - i); end loop;
        return r;
    end function;

    --------------------------------------------------------------------------
    -- Knob / switch reads
    --------------------------------------------------------------------------
    signal s_speedknob  : unsigned(9 downto 0);
    signal s_densknob   : unsigned(9 downto 0);
    signal s_tailknob   : unsigned(9 downto 0);
    signal s_mutknob    : unsigned(9 downto 0);
    signal s_colknob    : unsigned(9 downto 0);
    signal s_sizeknob   : unsigned(9 downto 0);
    signal s_lumknob    : unsigned(9 downto 0);
    signal s_glow_en    : std_logic;
    signal s_mirror     : std_logic;
    signal s_rise       : std_logic;
    signal s_bgvid      : std_logic;
    signal s_glitch_sw  : std_logic;

    --------------------------------------------------------------------------
    -- Front end: raster measurement, generated avid, frame start.
    --------------------------------------------------------------------------
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q,   s_hs_r   : std_logic := '0';
    signal s_vs_q,   s_vs_f   : std_logic := '0';
    signal s_xcnt   : unsigned(11 downto 0) := (others => '0');
    signal s_W      : unsigned(11 downto 0) := to_unsigned(1920, 12);   -- longest line so far
    signal s_Wm1    : unsigned(11 downto 0) := to_unsigned(1919, 12);   -- per-field count-1
    signal s_H      : unsigned(11 downto 0) := to_unsigned(1080, 12);   -- active lines
    signal s_sawact : std_logic := '0';
    signal s_fstart : std_logic := '0';
    -- generated avid: hsync phase toggle, arm, parity vote, start select
    signal g_ph, g_arm, g_par, g_sel : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := (others => '0');
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    -- M registers: current pixel (valid while m_avid = '1').  m_xs = m_x plus
    -- the glitch band's horizontal tear (0 outside the band).
    -- Resolution scaling: m_x counts REAL pixels (generated-avid length);
    -- m_xv / m_xs / m_y are the virtual-1080 integer parts of the Q.1 x
    -- accumulators and the Q.8 per-line y accumulator (step 1.0 at 1080p, so
    -- they equal the old counters there).
    signal m_avid : std_logic := '0';
    signal m_x    : unsigned(11 downto 0) := (others => '0');
    signal m_xa   : unsigned(12 downto 0) := (others => '0');
    signal m_xsa  : unsigned(12 downto 0) := (others => '0');
    signal m_ya   : unsigned(19 downto 0) := (others => '0');
    signal m_xv   : unsigned(11 downto 0);
    signal m_xs   : unsigned(11 downto 0);
    signal m_y    : unsigned(11 downto 0);
    signal s_xs   : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_Wv1r : unsigned(11 downto 0) := to_unsigned(1919, 12);   -- s_W - 1
    signal s_Wvr  : unsigned(11 downto 0) := to_unsigned(1919, 12);   -- virtual s_W - 1
    signal s_Wvm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);   -- per-field virtual width - 1
    signal s_xbq  : std_logic_vector(2 downto 0) := (others => '0');  -- m_xv(6..4), last clock
    signal s_ybq  : std_logic_vector(2 downto 0) := (others => '0');  -- m_y(6..4), last line
    signal m_band : std_logic := '0';                  -- in a horizontal glitch band
    signal m_vband : std_logic := '0';                 -- in a vertical glitch band
    signal s_ydy  : unsigned(11 downto 0) := (others => '0');   -- m_y + vertical tear

    --------------------------------------------------------------------------
    -- Per-frame latched parameters (all shift-only math).
    --------------------------------------------------------------------------
    signal s_zoom     : unsigned(1 downto 0)  := "01";                -- cell = 16<<zoom px
    signal s_lz       : unsigned(1 downto 0)  := "01";                -- luma cell = 16<<lz (lz>=1)
    signal s_ncols    : unsigned(7 downto 0)  := to_unsigned(60, 8);   -- columns (<=128)
    signal s_nrows    : unsigned(7 downto 0)  := to_unsigned(34, 8);   -- rows (rounded up)
    signal s_nrm1     : unsigned(7 downto 0)  := to_unsigned(33, 8);   -- rows - 1
    signal s_tail     : unsigned(5 downto 0)  := to_unsigned(12, 6);   -- visible tail rows
    signal s_sparse   : unsigned(7 downto 0)  := to_unsigned(75, 8);   -- 255-density
    signal s_athr     : unsigned(8 downto 0)  := to_unsigned(255, 9);  -- alive threshold
    signal s_gapmax   : unsigned(9 downto 0)  := to_unsigned(74, 10);  -- longest gap (rows)
    signal s_basespd  : unsigned(7 downto 0)  := to_unsigned(48, 8);   -- 1/32 row per frame
    signal s_T        : unsigned(23 downto 0) := (others => '0');      -- global mutation tick
    signal s_fc       : unsigned(8 downto 0)  := (others => '0');      -- frame counter
    signal s_r8       : unsigned(7 downto 0)  := (others => '0');      -- luma->size amount
    signal s_cbase    : unsigned(5 downto 0)  := to_unsigned(32, 6);   -- hue wheel index
    signal s_white    : std_logic := '0';
    signal s_fpar     : std_logic := '0';                              -- frame parity (luma map bank)
    signal s_limit    : signed(10 downto 0)   := to_signed(46, 11);   -- off-bottom row
    signal s_far      : signed(10 downto 0)   := to_signed(-78, 11);  -- pull-in row

    -- Glitch events (S11): a band of glyph rows tears sideways and scrambles
    -- for a few frames, every 0.5-2.5 s; the whole field mutation-bursts.
    signal s_rnd      : unsigned(15 downto 0) := (others => '0');      -- free-running hash
    signal s_gl_ycand : unsigned(11 downto 0) := (others => '0');      -- rnd * H >> 8
    signal s_gl_on    : std_logic := '0';
    signal s_gl_wait  : unsigned(7 downto 0)  := to_unsigned(40, 8);
    signal s_gl_len   : unsigned(4 downto 0)  := (others => '0');
    signal s_gl_y0    : unsigned(11 downto 0) := (others => '0');
    signal s_gl_y1    : unsigned(11 downto 0) := (others => '0');
    signal s_gl_dx    : unsigned(11 downto 0) := (others => '0');
    signal s_gl_vert  : std_logic := '0';                              -- event is a column band
    signal s_gl_xcand : unsigned(11 downto 0) := (others => '0');      -- rnd * W >> 8
    signal s_gl_x0    : unsigned(11 downto 0) := (others => '0');
    signal s_gl_x1    : unsigned(11 downto 0) := (others => '0');
    signal s_gl_dy    : unsigned(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-column drop state.  Three head BRAMs (one per drop, read in
    -- parallel by the renderer) hold each drop's position: 16-bit fixed point,
    -- [15:5] = signed row (negative = waiting above the top), [4:0] fraction.
    -- A fourth BRAM holds per-drop info, 4 bits per drop d at [4d+3:4d]:
    -- bit 3 = alive (visible), bits 2..0 = speed factor j (speed = base*(8+j)/8).
    -- One shared read address (render, or the update walk during vblank) and
    -- one shared write address (update walk only) -- canonical 1W1R each.
    --------------------------------------------------------------------------
    type t_mem is array (0 to 127) of std_logic_vector(15 downto 0);

    -- Power-on state: drops scattered over rows -64..63 so the screen is
    -- already raining, all alive, hashed speed factors.
    function build_head(d : integer) return t_mem is
        variable r : t_mem;
        variable h : unsigned(15 downto 0);
    begin
        for c in 0 to 127 loop
            h := mix16(to_unsigned(c * 5 + d * 1237 + 4660, 16));
            r(c) := std_logic_vector(to_signed(to_integer(h(6 downto 0)) - 64, 11))
                    & std_logic_vector(h(11 downto 7));
        end loop;
        return r;
    end function;
    function build_info return t_mem is
        variable r : t_mem;
        variable h : unsigned(15 downto 0);
    begin
        for c in 0 to 127 loop
            h := mix16(to_unsigned(c * 7 + 2571, 16));
            r(c) := "0000" & '1' & std_logic_vector(h(2 downto 0))
                           & '1' & std_logic_vector(h(5 downto 3))
                           & '1' & std_logic_vector(h(8 downto 6));
        end loop;
        return r;
    end function;

    signal s_hm0 : t_mem := build_head(0);
    signal s_hm1 : t_mem := build_head(1);
    signal s_hm2 : t_mem := build_head(2);
    signal s_im  : t_mem := build_info;
    attribute ram_style of s_hm0 : signal is "block";
    attribute ram_style of s_hm1 : signal is "block";
    attribute ram_style of s_hm2 : signal is "block";
    attribute ram_style of s_im  : signal is "block";

    type t_w16 is array (0 to C_ND - 1) of std_logic_vector(15 downto 0);
    signal s_head_raddr : unsigned(6 downto 0) := (others => '0');
    signal s_hq         : t_w16 := (others => (others => '0'));
    signal s_iq         : std_logic_vector(15 downto 0) := (others => '0');
    signal s_hw_we      : std_logic := '0';
    signal s_hw_waddr   : unsigned(6 downto 0) := (others => '0');
    signal s_hw_wdata   : t_w16 := (others => (others => '0'));
    signal s_iw_wdata   : std_logic_vector(15 downto 0) := (others => '0');

    -- palette: per-frame index -> registered EBR read -> fabric copy
    signal s_pal_raddr  : unsigned(6 downto 0) := to_unsigned(32, 7);
    signal s_pal_q      : std_logic_vector(31 downto 0) := (others => '0');
    signal s_palw       : std_logic_vector(31 downto 0) := C_PAL(32);

    --------------------------------------------------------------------------
    -- Luma map (P12): one 4-bit luma level l per luma cell (the glyph
    -- cell, or 2x2 cells at 16 px), sampled at each cell's centre from the
    -- input, double-buffered by frame parity (write one bank, read the other:
    -- no read/write collision, and a glyph never changes scale part-way down).
    -- Address = bank & row(5:0) & col(5:0).
    --------------------------------------------------------------------------
    type t_lmap is array (0 to 8191) of std_logic_vector(3 downto 0);
    signal s_lmap : t_lmap := (others => "1111");
    attribute ram_style of s_lmap : signal is "block";
    signal s_lm_raddr : unsigned(12 downto 0) := (others => '0');
    signal s_lm_q     : std_logic_vector(3 downto 0) := "1111";
    -- sampler pipeline
    signal s_y1, s_y2, s_y3 : unsigned(9 downto 0) := (others => '0');
    signal s_ysa, s_ysb     : unsigned(10 downto 0) := (others => '0');
    signal s_ysum           : unsigned(11 downto 0) := (others => '0');   -- 4-px luma sum
    signal w1_v, w2_v, w3_v : std_logic := '0';
    signal w1_a, w2_a, w3_a : unsigned(12 downto 0) := (others => '0');
    signal w1_y             : unsigned(11 downto 0) := (others => '0');
    signal w2_l4            : unsigned(3 downto 0) := (others => '0');
    signal w3_q             : std_logic_vector(3 downto 0) := "1111";

    -- Image strength (P12) tables.  For luma level l (0..15) and slider r:
    --   bright strength b = r*l, dark strength d = r*(15-l)
    --   glyph scale  s = 1 + b/256 - 0.6*d/256   (0.4x in black .. 2x wide in white)
    --   tail stretch m = 1 + j/5, j = b>>8 (0..14)  -> up to ~3.8x in white
    -- Rebuilt every frame by p_pseq into a 16-entry param RAM indexed by l.
    -- C_INV16(s16) = round(256 / s16): inverse scale for s = s16/16.
    type t_inv16 is array (0 to 31) of unsigned(6 downto 0);
    function build_inv16 return t_inv16 is
        variable r : t_inv16;
    begin
        for k in 0 to 31 loop
            if k < 6 then r(k) := to_unsigned(43, 7);
            else r(k) := to_unsigned((256 + k / 2) / k, 7); end if;
        end loop;
        return r;
    end function;
    constant C_INV16 : t_inv16 := build_inv16;
    -- tail stretch x16 and its reciprocal x256, by j
    type t_m16 is array (0 to 15) of unsigned(6 downto 0);
    type t_rm  is array (0 to 15) of unsigned(8 downto 0);
    function build_m16 return t_m16 is
        variable r : t_m16;
    begin
        for j in 0 to 15 loop r(j) := to_unsigned((80 + 16 * j + 2) / 5, 7); end loop;  -- 16(1+j/5)
        return r;
    end function;
    function build_rm return t_rm is
        variable r : t_rm;
    begin
        for j in 0 to 15 loop r(j) := to_unsigned((1280 + (5 + j) / 2) / (5 + j), 9); end loop; -- 256/(1+j/5)
        return r;
    end function;
    constant C_M16 : t_m16 := build_m16;
    constant C_RM  : t_rm  := build_rm;
    -- fade scale for the plain tail, x4096: round(4096/tail), tail = 2..33
    type t_kf12 is array (0 to 31) of unsigned(11 downto 0);
    function build_kf12 return t_kf12 is
        variable r : t_kf12;
        variable t : integer;
    begin
        for i in 0 to 31 loop
            t := i + 2;
            r(i) := to_unsigned((4096 + t / 2) / t, 12);
        end loop;
        return r;
    end function;
    constant C_KF12 : t_kf12 := build_kf12;

    -- Param RAM (EBR): word [6:0] = 16/s, [14:7] = stretched tail,
    -- [26:15] = fade scale x4096 / stretched tail.  Written in vblank by the
    -- sequencer, read per pixel by luma level.  Depth declared 256 for EBR.
    type t_prm is array (0 to 255) of std_logic_vector(31 downto 0);
    signal s_prm     : t_prm := (others => x"00000000");
    attribute ram_style of s_prm : signal is "block";
    signal s_prm_q   : std_logic_vector(31 downto 0) := (others => '0');
    type t_pst is (P_IDLE, P_A, P_B, P_C, P_D, P_E);
    signal s_pst     : t_pst := P_IDLE;
    signal s_pi      : unsigned(3 downto 0) := (others => '0');
    signal p_bprod, p_dprod  : unsigned(11 downto 0) := (others => '0');
    signal p_s16     : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal p_j       : unsigned(3 downto 0) := (others => '0');
    signal p_invx    : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal p_m16     : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal p_rm      : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal p_te      : unsigned(7 downto 0) := (others => '0');
    signal p_ke      : unsigned(11 downto 0) := (others => '0');
    signal p_we      : std_logic := '0';
    signal p_wa      : unsigned(3 downto 0) := (others => '0');
    signal p_wd      : std_logic_vector(31 downto 0) := (others => '0');
    signal s_kf12    : unsigned(11 downto 0) := to_unsigned(341, 12);
    signal s_temax   : unsigned(7 downto 0) := to_unsigned(12, 8);

    --------------------------------------------------------------------------
    -- Video background (S10): input delayed in an EBR ring to meet the
    -- output pixel.  Word = y & u & v padded to 32 bits.
    --------------------------------------------------------------------------
    type t_vring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal s_vring : t_vring := (others => (others => '0'));
    attribute ram_style of s_vring : signal is "block";
    -- Collision-free RAMs (see CLAUDE.md "no_rw_check"): skip yosys's
    -- read-before-write emulation.  vring: read pointer trails the write
    -- pointer by C_VDLY.  Drop RAMs: the one colliding read (update walk
    -- write-back) lands in vblank and is never used.  Luma map: write and
    -- read banks differ while video is active.  Param RAM: written in vblank.
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_vring : signal is true;
    attribute no_rw_check of s_hm0   : signal is true;
    attribute no_rw_check of s_hm1   : signal is true;
    attribute no_rw_check of s_hm2   : signal is true;
    attribute no_rw_check of s_im    : signal is true;
    attribute no_rw_check of s_lmap  : signal is true;
    attribute no_rw_check of s_prm   : signal is true;
    signal s_vwa   : unsigned(7 downto 0) := (others => '0');
    signal s_vra   : unsigned(7 downto 0) := to_unsigned(256 - C_VDLY, 8);
    signal s_vq    : std_logic_vector(31 downto 0) := (others => '0');
    signal s_vq2   : std_logic_vector(31 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Glyph ROM flattened into a single-port BRAM (addr = glyph*16 + row).
    --------------------------------------------------------------------------
    constant C_GROM_DEPTH : integer := C_NGLYPH * 16;
    type t_grom is array (0 to C_GROM_DEPTH - 1) of std_logic_vector(15 downto 0);
    function build_grom return t_grom is
        variable r : t_grom;
    begin
        for g in 0 to C_NGLYPH - 1 loop
            for y in 0 to 15 loop
                r(g * 16 + y) := C_GLYPHS(g, y);
            end loop;
        end loop;
        return r;
    end function;
    signal s_grom    : t_grom := build_grom;
    attribute ram_style of s_grom : signal is "block";

    --------------------------------------------------------------------------
    -- Per-frame drop update FSM + global LFSR.
    -- Read latency to s_hq is 3 cycles: s_upd_raddr -> registered raddr mux in
    -- p_acc -> registered BRAM output.  Then one op group per state.
    --------------------------------------------------------------------------
    type t_upd is (U_IDLE, U_REQ, U_W1, U_W2, U_LAT, U_SPD1, U_SPD2, U_ADD, U_WR);
    signal s_ustate : t_upd := U_IDLE;
    attribute fsm_encoding : string;
    attribute fsm_encoding of s_ustate : signal is "none";
    signal s_ucnt   : unsigned(7 downto 0) := (others => '0');
    signal s_busy   : std_logic := '0';
    signal s_upd_raddr : unsigned(6 downto 0) := (others => '0');
    signal s_lfsr   : std_logic_vector(15 downto 0) := x"ACE1";

    type t_s16 is array (0 to C_ND - 1) of signed(15 downto 0);
    type t_u9  is array (0 to C_ND - 1) of unsigned(8 downto 0);
    type t_u10 is array (0 to C_ND - 1) of unsigned(9 downto 0);
    type t_u4  is array (0 to C_ND - 1) of std_logic_vector(3 downto 0);
    signal u_pos   : t_s16 := (others => (others => '0'));   -- latched position
    signal u_info  : t_u4  := (others => (others => '0'));   -- latched alive/j
    signal u_spd   : t_u9  := (others => (others => '0'));   -- speed (partial, then full)
    signal u_gap   : t_u10 := (others => (others => '0'));   -- respawn gap (rows)
    signal u_ninfo : t_u4  := (others => (others => '0'));   -- respawn alive/j
    signal u_nfrac : t_u4  := (others => (others => '0'));   -- respawn fraction bits
    -- Respawn randoms.  mix16 is linear over GF(2), so the per-drop draws
    -- mix16(lfsr xor K) equal mix16(lfsr xor 5A3C) xor mix16(5A3C xor K):
    -- one held copy of s_rnd (taken the clock after U_W1, when it carries
    -- the U_W1 lfsr) xor a per-drop constant.  One shared rnd8 * sparse
    -- multiplier serves the three drops on consecutive clocks.
    signal u_um    : unsigned(15 downto 0) := (others => '0');   -- s_rnd held at U_W2
    signal u_op    : unsigned(7 downto 0)  := (others => '0');   -- multiplier operand
    signal u_p     : unsigned(15 downto 0) := (others => '0');   -- shared rnd8 * sparse

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    type t_d12 is array (0 to C_ND - 1) of signed(11 downto 0);
    type t_u8  is array (0 to C_ND - 1) of unsigned(7 downto 0);
    -- S0
    signal s0_col  : unsigned(6 downto 0) := (others => '0');
    signal s0_row  : unsigned(7 downto 0) := (others => '0');
    signal s0_rx, s0_ry : signed(6 downto 0) := (others => '0');   -- px from cell centre
    signal s0_render, s0_band : std_logic := '0';
    -- S1 (align with the RAM reads)
    signal s1_col  : unsigned(6 downto 0) := (others => '0');
    signal s1_row  : unsigned(7 downto 0) := (others => '0');   -- true row (glyph hash)
    signal s1_drow : unsigned(7 downto 0) := (others => '0');   -- row for drop distance
    signal s1_rx, s1_ry : signed(6 downto 0) := (others => '0');
    signal s1_render, s1_band : std_logic := '0';
    -- S2
    signal s2_dist   : t_d12 := (others => (others => '1'));
    signal s2_alive  : std_logic_vector(C_ND - 1 downto 0) := (others => '0');
    signal s2_l      : unsigned(3 downto 0) := (others => '0');
    signal s2_base   : unsigned(15 downto 0) := (others => '0');
    signal s2_rx, s2_ry : signed(6 downto 0) := (others => '0');
    signal s2_render, s2_band : std_logic := '0';
    -- S3 (param RAM read)
    signal s3_dist   : t_d12 := (others => (others => '1'));
    signal s3_alive  : std_logic_vector(C_ND - 1 downto 0) := (others => '0');
    signal s3_base   : unsigned(15 downto 0) := (others => '0');
    signal s3_rx, s3_ry : signed(6 downto 0) := (others => '0');
    signal s3_render, s3_band : std_logic := '0';
    -- S4 (params in fabric)
    signal s4_dist   : t_d12 := (others => (others => '1'));
    signal s4_alive  : std_logic_vector(C_ND - 1 downto 0) := (others => '0');
    signal s4_base   : unsigned(15 downto 0) := (others => '0');
    signal s4_rx, s4_ry : signed(6 downto 0) := (others => '0');
    signal s4_te     : unsigned(7 downto 0) := (others => '0');
    signal s4_ke     : unsigned(11 downto 0) := (others => '0');
    signal s4_invx, s4_invy : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s4_render, s4_band : std_logic := '0';
    -- S5 (lit tests)
    signal s5_lit    : std_logic_vector(C_ND - 1 downto 0) := (others => '0');
    signal s5_head   : std_logic_vector(C_ND - 1 downto 0) := (others => '0');
    signal s5_dc     : t_u8 := (others => (others => '1'));
    signal s5_base   : unsigned(15 downto 0) := (others => '0');
    signal s5_rx, s5_ry : signed(6 downto 0) := (others => '0');
    signal s5_ke     : unsigned(11 downto 0) := (others => '0');
    signal s5_invx, s5_invy : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s5_render, s5_band : std_logic := '0';
    -- S6 (glyph index, nearest drop)
    signal s6_idx  : integer range 0 to C_NGLYPH - 1 := 0;
    signal s6_dmin : unsigned(7 downto 0) := (others => '1');
    signal s6_lit, s6_head, s6_render, s6_band : std_logic := '0';
    signal s6_rx, s6_ry : signed(6 downto 0) := (others => '0');
    signal s6_ke     : unsigned(11 downto 0) := (others => '0');
    signal s6_invx, s6_invy : unsigned(6 downto 0) := to_unsigned(16, 7);
    -- S7 (products)
    signal s7_idx  : integer range 0 to C_NGLYPH - 1 := 0;
    signal s7_px, s7_py : signed(13 downto 0) := (others => '0');
    signal s7_fprod : unsigned(19 downto 0) := (others => '0');
    signal s7_lit, s7_head, s7_render, s7_band : std_logic := '0';
    -- S8 (glyph coords, fade level)
    signal s8_idx  : integer range 0 to C_NGLYPH - 1 := 0;
    signal s8_gx, s8_gy : unsigned(3 downto 0) := (others => '0');
    signal s8_in   : std_logic := '0';
    signal s8_lvl  : unsigned(9 downto 0) := (others => '0');
    signal s8_lit, s8_head, s8_render, s8_hot : std_logic := '0';
    -- S9 (glyph ROM)
    signal s9_rb_c : std_logic_vector(15 downto 0) := (others => '0');
    signal s9_gx   : unsigned(3 downto 0) := (others => '0');
    signal s9_in, s9_lit, s9_head, s9_render, s9_hot : std_logic := '0';
    signal s9_lvl  : unsigned(9 downto 0) := (others => '0');
    -- S10 (bit / glow)
    signal s10_on, s10_glow, s10_head, s10_render, s10_hot : std_logic := '0';
    signal s10_lvl : unsigned(9 downto 0) := (others => '0');
    -- S11: one copy of the level per consumer (Y adder, U and V multipliers)
    signal s11_l_y   : unsigned(9 downto 0) := (others => '0');
    signal s11_l_u   : unsigned(9 downto 0) := (others => '0');
    signal s11_l_v   : unsigned(9 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s11_l_y : signal is true;
    attribute keep of s11_l_u : signal is true;
    attribute keep of s11_l_v : signal is true;
    -- chroma mode: "00" full, "01" 3/4 (hot rows), "10" half (head / glow)
    signal s11_cm    : std_logic_vector(1 downto 0) := "00";
    signal s11_drawn, s11_render : std_logic := '0';
    -- S12
    signal s12_y      : unsigned(9 downto 0) := C_BLK;
    signal s12_puh, s12_pvh : unsigned(14 downto 0) := (others => '0');  -- L * |c|(9:5)
    signal s12_pul, s12_pvl : unsigned(14 downto 0) := (others => '0');  -- L * |c|(4:0)
    signal s12_cm    : std_logic_vector(1 downto 0) := "00";
    signal s12_drawn, s12_render : std_logic := '0';
    -- S12b
    signal s12b_y      : unsigned(9 downto 0) := C_BLK;
    signal s12b_pu     : unsigned(19 downto 0) := (others => '0');
    signal s12b_pv     : unsigned(19 downto 0) := (others => '0');
    signal s12b_cm   : std_logic_vector(1 downto 0) := "00";
    signal s12b_drawn, s12b_render : std_logic := '0';
    -- S12c: chroma offsets after the mode scale
    signal s12c_y      : unsigned(9 downto 0) := C_BLK;
    signal s12c_du, s12c_dv : unsigned(10 downto 0) := (others => '0');
    signal s12c_drawn, s12c_render : std_logic := '0';
    -- S13
    signal s13_y, s13_u, s13_v : unsigned(9 downto 0) := (others => '0');
    signal s13_avid : std_logic := '0';
    signal s13_sync : std_logic_vector(2 downto 0) := (others => '1');

    -- sync passthrough pipeline: [field vsync hsync]
    type t_syncp is array (0 to C_SYNCD) of std_logic_vector(2 downto 0);
    signal s_syncp : t_syncp := (others => (others => '1'));

    signal s_io : t_video_stream_yuv444_30b;

begin
    -- virtual-1080 integer parts (see the M registers)
    m_xv <= m_xa(12 downto 1);
    m_xs <= m_xsa(12 downto 1);
    m_y  <= m_ya(19 downto 8);

    --------------------------------------------------------------------------
    -- Control reads
    --------------------------------------------------------------------------
    s_speedknob  <= unsigned(registers_in(0));
    s_densknob   <= unsigned(registers_in(1));
    s_tailknob   <= unsigned(registers_in(2));
    s_mutknob    <= unsigned(registers_in(3));
    s_colknob    <= unsigned(registers_in(4));
    s_sizeknob   <= unsigned(registers_in(5));
    s_glow_en    <= registers_in(6)(0);   -- S7
    s_mirror     <= registers_in(6)(1);   -- S8
    s_rise       <= registers_in(6)(2);   -- S9
    s_bgvid      <= registers_in(6)(3);   -- S10
    s_glitch_sw  <= registers_in(6)(4);   -- S11
    s_lumknob    <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Front end.  Width from the source avid; the program's own avid
    -- (parity-locked start, fixed per-field length); pixel x/y; frame start.
    --------------------------------------------------------------------------
    p_front : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;    -- hsync assert
            s_vs_q   <= data_in.vsync_n;
            s_vs_f   <= s_vs_q and not data_in.vsync_n;    -- vsync assert

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > s_W then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            -- Generated avid (see header): start 2 or 3 clocks after the
            -- line's first source avid rise, whichever keeps the hsync->start
            -- parity at the source's usual value.
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

            -- Exactly W pixels per line from the generated start, carried into
            -- the porch if the source line runs short (line-length jitter
            -- must not pile up down the screen).
            -- (virtual accumulators step s_xs / s_ys alongside the real m_x)
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                m_avid <= '1';
                m_x    <= (others => '0');
                m_xa   <= (others => '0');
                if m_band = '1' then m_xsa <= s_gl_dx & '0';
                else                 m_xsa <= (others => '0'); end if;
            elsif m_avid = '1' then
                if m_x = s_Wm1 then
                    m_avid <= '0';
                    m_ya   <= m_ya + s_ys;  -- advance on active lines only
                    s_ybq  <= std_logic_vector(m_y(6 downto 4));
                end if;
                m_x   <= m_x + 1;
                m_xa  <= m_xa + s_xs;
                m_xsa <= m_xsa + s_xs;
            else
                m_x  <= (others => '0');
                m_xa <= (others => '0');
            end if;
            s_xbq <= std_logic_vector(m_xv(6 downto 4));

            -- per-mode steps (timing ID is static while a mode runs)
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            -- virtual width - 1 of the longest line so far (latched per field)
            s_Wv1r <= s_W - 1;
            case to_integer(s_xs) is
                when 3      => s_Wvr <= s_Wv1r + shift_right(s_Wv1r, 1);
                when 4      => s_Wvr <= shift_left(s_Wv1r, 1);
                when others => s_Wvr <= s_Wv1r;
            end case;

            -- glitch band membership: rows per line (m_y settles in blanking),
            -- columns per pixel (one pixel late -- invisible in a glitch)
            if s_gl_on = '1' and s_gl_vert = '0' and m_y >= s_gl_y0 and m_y < s_gl_y1 then
                m_band <= '1';
            else
                m_band <= '0';
            end if;
            if s_gl_on = '1' and s_gl_vert = '1' and m_xv >= s_gl_x0 and m_xv < s_gl_x1 then
                m_vband <= '1';
            else
                m_vband <= '0';
            end if;
            s_ydy <= m_y + s_gl_dy;

            -- Frame start: the first vsync edge after active video only.
            s_fstart <= '0';
            if m_avid = '1' then
                s_sawact <= '1';
            end if;
            if s_vs_f = '1' and s_sawact = '1' then
                s_sawact <= '0';
                s_fstart <= '1';
                if m_y > 16 then s_H <= m_y; end if;   -- virtual (1080)
                -- per-field width = the LONGEST source line of the field, so a
                -- short (dropout) line can never shear the next field
                if s_W > 16 then
                    s_Wm1  <= s_W - 1;
                    s_Wvm1 <= s_Wvr;
                end if;
                s_W   <= (others => '0');
                m_ya  <= (others => '0');
                s_ybq <= (others => '0');
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame parameter latch + glitch event generator.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_z    : integer range 0 to 3;
        variable v_d8   : unsigned(7 downto 0);
        variable v_sp   : unsigned(7 downto 0);
        variable v_pm   : unsigned(15 downto 0);
        variable v_a    : unsigned(8 downto 0);
        variable v_cw   : unsigned(11 downto 0);
        variable v_ch   : unsigned(11 downto 0);
        variable v_nr   : unsigned(7 downto 0);
        variable v_gy   : unsigned(11 downto 0);
        variable v_bh   : unsigned(11 downto 0);
        variable v_gpm  : unsigned(19 downto 0);
        variable v_gpx  : unsigned(19 downto 0);
        variable v_gx0  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- respawn bounds from the latched params (stable long before the
            -- update walk reaches its write state)
            -- (a drop respawns only once its longest stretched tail is gone)
            s_limit <= signed(resize(s_nrows, 11)) + signed(resize(s_temax, 11));
            s_far   <= -signed(resize(s_gapmax, 11)) - 4;
            -- glitch randoms, free-running: hash and a random row position
            s_rnd <= mix16(unsigned(s_lfsr) xor x"5A3C");
            v_gpm := s_rnd(7 downto 0) * s_H;
            s_gl_ycand <= v_gpm(19 downto 8);
            v_gpx := s_rnd(15 downto 8) * s_Wvm1;
            s_gl_xcand <= v_gpx(19 downto 8);

            if s_fstart = '1' then
                -- Glyph size (K6): cell = 16 << zoom px (16/32/64/128).
                v_z := to_integer(s_sizeknob(9 downto 8));
                s_zoom <= to_unsigned(v_z, 2);
                -- luma map cell: the glyph cell, 2x2 cells at 16 px
                if v_z = 0 then s_lz <= "01"; else s_lz <= to_unsigned(v_z, 2); end if;
                s_fpar <= not s_fpar;
                -- columns and rows rounded UP so partial cells still rain
                case v_z is
                    when 0      => v_cw := s_Wvm1 + 16;  v_ch := s_H + 15;
                    when 1      => v_cw := s_Wvm1 + 32;  v_ch := s_H + 31;
                    when 2      => v_cw := s_Wvm1 + 64;  v_ch := s_H + 63;
                    when others => v_cw := s_Wvm1 + 128; v_ch := s_H + 127;
                end case;
                s_ncols <= resize(shift_right(v_cw, 4 + v_z), 8);
                v_nr    := resize(shift_right(v_ch, 4 + v_z), 8);
                s_nrows <= v_nr;
                s_nrm1  <= v_nr - 1;
                s_tail  <= resize(to_unsigned(2, 6) + s_tailknob(9 downto 5), 6);   -- 2..33
                s_kf12  <= C_KF12(to_integer(s_tailknob(9 downto 5)));
                -- 4..42 (1/32 row per frame): 0.125 .. ~1.3 rows/frame base
                s_basespd <= to_unsigned(4, 8) + resize(s_speedknob(9 downto 5), 8)
                             + resize(s_speedknob(9 downto 7), 8);

                -- Density: upper half shortens the respawn gap to zero; lower
                -- half also thins out how many respawned drops are visible.
                v_d8 := s_densknob(9 downto 2);
                v_sp := not v_d8;                                -- 255 - density
                s_sparse <= v_sp;
                v_a := shift_left(resize(v_d8, 9), 1) + 16;
                if v_d8(7) = '1' then v_a := to_unsigned(256, 9); end if;
                s_athr <= v_a;
                -- longest gap = (255 * sparse) >> (6 + zoom) rows
                v_pm := shift_left(resize(v_sp, 16), 8) - resize(v_sp, 16);
                case v_z is
                    when 0      => s_gapmax <= v_pm(15 downto 6);
                    when 1      => s_gapmax <= resize(v_pm(15 downto 7), 10);
                    when 2      => s_gapmax <= resize(v_pm(15 downto 8), 10);
                    when others => s_gapmax <= resize(v_pm(15 downto 9), 10);
                end case;

                -- mutation tick: 1..16 per frame, +64 during a glitch burst.
                if s_gl_on = '1' then
                    s_T <= s_T + (resize(s_mutknob(9 downto 6), 24) + 65);
                else
                    s_T <= s_T + (resize(s_mutknob(9 downto 6), 24) + 1);
                end if;
                s_fc <= s_fc + 1;

                -- Image strength and colour (wheel index; top step = white).
                s_r8    <= s_lumknob(9 downto 2);
                -- knob 0 = phosphor green (wheel 32), then cyan, blue, violet,
                -- red, orange, amber, lime; the top step is white
                s_cbase <= s_colknob(9 downto 4) + 32;
                if s_colknob(9 downto 4) = 63 then
                    s_white <= '1';
                    s_pal_raddr <= to_unsigned(64, 7);
                else
                    s_white <= '0';
                    s_pal_raddr <= '0' & (s_colknob(9 downto 4) + 32);
                end if;

                -- Glitch: wait 30..157 frames, then tear a band of 1..4 glyph
                -- rows for 2..17 frames (new tear offset every frame).
                if s_glitch_sw = '0' then
                    s_gl_on   <= '0';
                    s_gl_wait <= to_unsigned(20, 8);
                elsif s_gl_on = '1' then
                    s_gl_dx <= resize(s_rnd(9 downto 0), 12);
                    s_gl_dy <= resize(s_rnd(10 downto 1), 12);
                    if s_gl_len = 0 then
                        s_gl_on   <= '0';
                        s_gl_wait <= to_unsigned(30, 8) + resize(s_rnd(14 downto 8), 8);
                    else
                        s_gl_len <= s_gl_len - 1;
                    end if;
                elsif s_gl_wait = 0 then
                    s_gl_on   <= '1';
                    s_gl_vert <= s_lfsr(0);                 -- rows or columns, 50/50
                    s_gl_len <= to_unsigned(2, 5) + resize(s_rnd(15 downto 12), 5);
                    s_gl_dx  <= resize(s_rnd(9 downto 0), 12);
                    s_gl_dy  <= resize(s_rnd(10 downto 1), 12);
                    -- band edge snapped to a glyph row / column; 1..4 cells
                    case to_integer(s_zoom) is
                        when 0 =>
                            v_gx0 := s_gl_xcand(11 downto 4) & "0000";
                            v_gy := s_gl_ycand(11 downto 4) & "0000";
                            v_bh := shift_left(resize(s_rnd(11 downto 10), 12) + 1, 4);
                        when 1 =>
                            v_gx0 := s_gl_xcand(11 downto 5) & "00000";
                            v_gy := s_gl_ycand(11 downto 5) & "00000";
                            v_bh := shift_left(resize(s_rnd(11 downto 10), 12) + 1, 5);
                        when 2 =>
                            v_gx0 := s_gl_xcand(11 downto 6) & "000000";
                            v_gy := s_gl_ycand(11 downto 6) & "000000";
                            v_bh := shift_left(resize(s_rnd(11 downto 10), 12) + 1, 6);
                        when others =>
                            v_gx0 := s_gl_xcand(11 downto 7) & "0000000";
                            v_gy := s_gl_ycand(11 downto 7) & "0000000";
                            v_bh := shift_left(resize(s_rnd(11 downto 10), 12) + 1, 7);
                    end case;
                    s_gl_y0 <= v_gy;
                    s_gl_y1 <= v_gy + v_bh;
                    s_gl_x0 <= v_gx0;
                    s_gl_x1 <= v_gx0 + v_bh;
                else
                    s_gl_wait <= s_gl_wait - 1;
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Free-running LFSR (16-bit Fibonacci, taps 16,14,13,11).
    --------------------------------------------------------------------------
    p_lfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            v_fb := s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10);
            s_lfsr <= s_lfsr(14 downto 0) & v_fb;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame drop update, columns 0..ncols-1, all three drops in parallel:
    --   pos += base * (8 + j) / 8
    --   off the bottom (head + tail gone)   -> respawn
    --   waiting further up than the current longest gap (density was raised)
    --                                       -> respawn now (knob responds at once)
    -- Respawn: pos = -(random gap) rows, fresh speed factor j, alive with
    -- probability athr/256.  Runs in vblank (9 clocks/column).
    --------------------------------------------------------------------------
    p_headupd : process(clk)
        variable v_r     : unsigned(15 downto 0);
        variable v_r1    : unsigned(15 downto 0);
        variable v_gap   : unsigned(9 downto 0);
        variable v_b     : unsigned(8 downto 0);
        variable v_j     : std_logic_vector(2 downto 0);
        variable v_new   : signed(15 downto 0);
        variable v_int   : signed(10 downto 0);
        variable v_info  : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            -- shared respawn multiplier: drop 0 / 1 / 2 operand registered in
            -- U_W2 / U_LAT / U_SPD1, product in U_LAT / U_SPD1 / U_SPD2, the
            -- gap slice lands in U_SPD1 / U_SPD2 / U_ADD
            case s_ustate is
                when U_W2   => u_op <= s_rnd(7 downto 0) xor f_uk1(0)(7 downto 0);
                when U_LAT  => u_op <= u_um(7 downto 0)  xor f_uk1(1)(7 downto 0);
                when others => u_op <= u_um(7 downto 0)  xor f_uk1(2)(7 downto 0);
            end case;
            u_p <= u_op * s_sparse;
            case to_integer(s_zoom) is
                when 0      => v_gap := u_p(15 downto 6);
                when 1      => v_gap := resize(u_p(15 downto 7), 10);
                when 2      => v_gap := resize(u_p(15 downto 8), 10);
                when others => v_gap := resize(u_p(15 downto 9), 10);
            end case;

            s_hw_we <= '0';
            case s_ustate is
                when U_IDLE =>
                    if s_fstart = '1' then
                        s_ucnt   <= (others => '0');
                        s_busy   <= '1';
                        s_ustate <= U_REQ;
                    end if;
                when U_REQ =>
                    s_upd_raddr <= s_ucnt(6 downto 0);
                    s_ustate    <= U_W1;
                when U_W1 =>
                    -- (s_rnd takes this clock's lfsr: the respawn draw)
                    s_ustate <= U_W2;
                when U_W2 =>
                    u_um     <= s_rnd;
                    s_ustate <= U_LAT;
                when U_LAT =>
                    -- register the BRAM words before any arithmetic; finish the
                    -- respawn draw (gap, alive, speed factor, frac).
                    for d in 0 to C_ND - 1 loop
                        u_pos(d)  <= signed(s_hq(d));
                        u_info(d) <= s_iq(4 * d + 3 downto 4 * d);
                        v_r1 := u_um xor f_uk1(d);
                        v_r  := u_um xor f_uk2(d);
                        if resize(v_r1(15 downto 8), 9) < s_athr then
                            u_ninfo(d) <= '1' & std_logic_vector(v_r(2 downto 0));
                        else
                            u_ninfo(d) <= '0' & std_logic_vector(v_r(2 downto 0));
                        end if;
                        u_nfrac(d) <= std_logic_vector(v_r(6 downto 3));
                    end loop;
                    s_ustate <= U_SPD1;
                when U_SPD1 =>
                    u_gap(0) <= v_gap;
                    -- speed = base + j0*base/8 + j1*base/4 (+ j2*base/2 next)
                    for d in 0 to C_ND - 1 loop
                        v_b := resize(s_basespd, 9);
                        v_j := u_info(d)(2 downto 0);
                        if v_j(0) = '1' and v_j(1) = '1' then
                            u_spd(d) <= v_b + shift_right(v_b, 3) + shift_right(v_b, 2);
                        elsif v_j(0) = '1' then
                            u_spd(d) <= v_b + shift_right(v_b, 3);
                        elsif v_j(1) = '1' then
                            u_spd(d) <= v_b + shift_right(v_b, 2);
                        else
                            u_spd(d) <= v_b;
                        end if;
                    end loop;
                    s_ustate <= U_SPD2;
                when U_SPD2 =>
                    u_gap(1) <= v_gap;
                    for d in 0 to C_ND - 1 loop
                        if u_info(d)(2) = '1' then
                            u_spd(d) <= u_spd(d) + shift_right(resize(s_basespd, 9), 1);
                        end if;
                    end loop;
                    s_ustate <= U_ADD;
                when U_ADD =>
                    u_gap(2) <= v_gap;
                    for d in 0 to C_ND - 1 loop
                        u_pos(d) <= u_pos(d) + signed(resize(u_spd(d), 16));
                    end loop;
                    s_ustate <= U_WR;
                when U_WR =>
                    v_info  := (others => '0');
                    for d in 0 to C_ND - 1 loop
                        v_new := u_pos(d);
                        v_int := v_new(15 downto 5);
                        if v_int > s_limit or v_int < s_far then
                            v_new := -shift_left(signed(resize(u_gap(d), 16)), 5)
                                     - signed(resize(unsigned(u_nfrac(d)), 16));
                            v_info(4 * d + 3 downto 4 * d) := u_ninfo(d);
                        else
                            v_info(4 * d + 3 downto 4 * d) := u_info(d);
                        end if;
                        s_hw_wdata(d) <= std_logic_vector(v_new);
                    end loop;
                    s_iw_wdata <= v_info;
                    s_hw_waddr <= s_ucnt(6 downto 0);
                    s_hw_we    <= '1';
                    if s_ucnt + 1 >= s_ncols then
                        s_busy   <= '0';
                        s_ustate <= U_IDLE;
                    else
                        s_ucnt   <= s_ucnt + 1;
                        s_ustate <= U_REQ;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Drop-state BRAMs: one write port (update) + one registered read port
    -- (render / update) each, shared addresses.
    --------------------------------------------------------------------------
    p_hm0 : process(clk)
    begin
        if rising_edge(clk) then
            if s_hw_we = '1' then s_hm0(to_integer(s_hw_waddr)) <= s_hw_wdata(0); end if;
            s_hq(0) <= s_hm0(to_integer(s_head_raddr));
        end if;
    end process;
    p_hm1 : process(clk)
    begin
        if rising_edge(clk) then
            if s_hw_we = '1' then s_hm1(to_integer(s_hw_waddr)) <= s_hw_wdata(1); end if;
            s_hq(1) <= s_hm1(to_integer(s_head_raddr));
        end if;
    end process;
    p_hm2 : process(clk)
    begin
        if rising_edge(clk) then
            if s_hw_we = '1' then s_hm2(to_integer(s_hw_waddr)) <= s_hw_wdata(2); end if;
            s_hq(2) <= s_hm2(to_integer(s_head_raddr));
        end if;
    end process;
    p_im : process(clk)
    begin
        if rising_edge(clk) then
            if s_hw_we = '1' then s_im(to_integer(s_hw_waddr)) <= s_iw_wdata; end if;
            s_iq <= s_im(to_integer(s_head_raddr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Palette: the frame's colour word, registered EBR read + fabric copy.
    --------------------------------------------------------------------------
    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_pal_q <= s_palrom(to_integer(s_pal_raddr));
            s_palw  <= s_pal_q;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sync passthrough delay line (aligned with the generated avid).
    --------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syncp(0) <= data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            for k in 1 to C_SYNCD loop
                s_syncp(k) <= s_syncp(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Video background ring: input written every clock, read C_VDLY behind
    -- (the M pixel trails the source pixel by 4-5 clocks, the output trails
    -- M by 17).
    --------------------------------------------------------------------------
    p_vring : process(clk)
    begin
        if rising_edge(clk) then
            s_vring(to_integer(s_vwa)) <= data_in.y & data_in.u & data_in.v & "00";
            s_vwa <= s_vwa + 1;
            s_vra <= s_vra + 1;
            s_vq  <= s_vring(to_integer(s_vra));
            s_vq2 <= s_vq;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Luma map writer.  At the centre pixel of each luma cell, take a 4-pixel
    -- luma average, quantize it to a level l (0..15) and write it to this
    -- frame's bank.  The renderer reads last frame's.
    --------------------------------------------------------------------------
    p_lsample : process(clk)
        variable v_trig : std_logic;
        variable v_a    : unsigned(12 downto 0);
        variable v_avg  : unsigned(9 downto 0);
        variable v_yv   : unsigned(9 downto 0);
        variable v_l    : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            s_y1 <= unsigned(data_in.y);
            s_y2 <= s_y1;
            s_y3 <= s_y2;
            s_ysa  <= resize(unsigned(data_in.y), 11) + resize(s_y1, 11);
            s_ysb  <= resize(s_y2, 11) + resize(s_y3, 11);
            s_ysum <= resize(s_ysa, 12) + resize(s_ysb, 12);

            -- W1: centre pixel of a luma cell (16<<lz px square)
            v_trig := '0';
            case to_integer(s_lz) is
                -- (virtual units: the centre bit RISING on this pixel / line,
                -- since a fractional step can jump over the exact centre)
                when 1 =>
                    if m_xv(4) = '1' and s_xbq(0) = '0' and m_y(4) = '1' and s_ybq(0) = '0' then v_trig := '1'; end if;
                    v_a := s_fpar & m_y(10 downto 5) & m_xv(10 downto 5);
                when 2 =>
                    if m_xv(5) = '1' and s_xbq(1) = '0' and m_y(5) = '1' and s_ybq(1) = '0' then v_trig := '1'; end if;
                    v_a := s_fpar & m_y(11 downto 6) & m_xv(11 downto 6);
                when others =>
                    if m_xv(6) = '1' and s_xbq(2) = '0' and m_y(6) = '1' and s_ybq(2) = '0' then v_trig := '1'; end if;
                    v_a := s_fpar & '0' & m_y(11 downto 7) & '0' & m_xv(11 downto 7);
            end case;
            w1_v <= v_trig and m_avid;
            w1_a <= v_a;
            w1_y <= s_ysum;
            -- W2: luma above black
            v_avg := w1_y(11 downto 2);
            if v_avg > C_BLK then v_yv := v_avg - C_BLK;
            else v_yv := (others => '0'); end if;
            -- 876 -> ~15.4, saturated to 0..15
            v_l := resize(shift_right(v_yv, 6), 5) + resize(shift_right(v_yv, 9), 5);
            if v_l > 15 then w2_l4 <= "1111";
            else w2_l4 <= v_l(3 downto 0); end if;
            w2_v <= w1_v; w2_a <= w1_a;
            -- W3: (stage kept for timing) raw luma level
            w3_q <= std_logic_vector(w2_l4);
            w3_v <= w2_v; w3_a <= w2_a;
            -- W4: write
            if w3_v = '1' then
                s_lmap(to_integer(w3_a)) <= w3_q;
            end if;
            s_lm_q <= s_lmap(to_integer(s_lm_raddr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: pixel/cell coordinates (power-of-two grid => plain bit slicing).
    -- rx/ry = pixel offset from the cell centre (the MSB flip of the in-cell
    -- position).  Drives the drop-RAM read address (mux'd with the update
    -- walk) and the luma-map read address (last frame's bank).
    -- x comes from m_xs, which carries the glitch band's tear offset.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_col : unsigned(6 downto 0);
        variable v_row : unsigned(7 downto 0);
        variable v_rx, v_ry : signed(6 downto 0);
        variable v_y   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- vertical glitch band: rows torn by a random offset
            if m_vband = '1' then v_y := s_ydy; else v_y := m_y; end if;
            case to_integer(s_zoom) is
                when 0 =>
                    v_col := resize(m_xs(11 downto 4), 7);
                    v_row := resize(v_y(11 downto 4), 8);
                    v_rx := resize(signed((not m_xs(3)) & m_xs(2 downto 0)), 7);
                    v_ry := resize(signed((not v_y(3))  & v_y(2 downto 0)), 7);
                when 1 =>
                    v_col := resize(m_xs(11 downto 5), 7);
                    v_row := resize(v_y(11 downto 5), 8);
                    v_rx := resize(signed((not m_xs(4)) & m_xs(3 downto 0)), 7);
                    v_ry := resize(signed((not v_y(4))  & v_y(3 downto 0)), 7);
                when 2 =>
                    v_col := resize(m_xs(11 downto 6), 7);
                    v_row := resize(v_y(11 downto 6), 8);
                    v_rx := resize(signed((not m_xs(5)) & m_xs(4 downto 0)), 7);
                    v_ry := resize(signed((not v_y(5))  & v_y(4 downto 0)), 7);
                when others =>
                    v_col := resize(m_xs(11 downto 7), 7);
                    v_row := resize(v_y(11 downto 7), 8);
                    v_rx := signed((not m_xs(6)) & m_xs(5 downto 0));
                    v_ry := signed((not v_y(6))  & v_y(5 downto 0));
            end case;

            s0_col <= v_col; s0_row <= v_row;
            s0_rx  <= v_rx;  s0_ry  <= v_ry;
            s0_render <= m_avid;
            s0_band   <= m_band or m_vband;

            if s_busy = '1' then
                s_head_raddr <= s_upd_raddr;
            else
                s_head_raddr <= v_col;
            end if;

            case to_integer(s_lz) is
                when 1      => s_lm_raddr <= (not s_fpar) & m_y(10 downto 5) & m_xv(10 downto 5);
                when 2      => s_lm_raddr <= (not s_fpar) & m_y(11 downto 6) & m_xv(11 downto 6);
                when others => s_lm_raddr <= (not s_fpar) & '0' & m_y(11 downto 7) & '0' & m_xv(11 downto 7);
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: align with the RAM outputs.  Rise flips the row used for the drop
    -- distance (glyph field stays put).
    --------------------------------------------------------------------------
    p_s1 : process(clk)
    begin
        if rising_edge(clk) then
            s1_col <= s0_col; s1_row <= s0_row;
            if s_rise = '1' then s1_drow <= s_nrm1 - s0_row;
            else                 s1_drow <= s0_row; end if;
            s1_rx <= s0_rx; s1_ry <= s0_ry;
            s1_render <= s0_render;
            s1_band   <= s0_band;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Image-strength param sequencer: after each frame start, for every luma
    -- level l = 0..15 compute glyph scale, stretched tail and its fade scale
    -- (one op group per state) and write them to the param RAM.
    --------------------------------------------------------------------------
    p_pseq : process(clk)
        variable v_bi, v_di : unsigned(7 downto 0);
        variable v_s        : signed(6 downto 0);
        variable v_tp       : unsigned(12 downto 0);
        variable v_kp       : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            p_we <= '0';
            case s_pst is
                when P_IDLE =>
                    if s_fstart = '1' then
                        s_pi  <= (others => '0');
                        s_pst <= P_A;
                    end if;
                when P_A =>
                    p_bprod <= s_r8 * s_pi;
                    p_dprod <= s_r8 * (not s_pi);
                    s_pst <= P_B;
                when P_B =>
                    v_bi := p_bprod(11 downto 4);
                    v_di := p_dprod(11 downto 4);
                    v_s := to_signed(16, 7) + signed(resize(v_bi(7 downto 4), 7))
                           - signed(resize(v_di(7 downto 5), 7))
                           - signed(resize(v_di(7 downto 6), 7));
                    if v_s < 6 then v_s := to_signed(6, 7); end if;
                    p_s16 <= unsigned(v_s(4 downto 0));
                    p_j   <= p_bprod(11 downto 8);
                    s_pst <= P_C;
                when P_C =>
                    p_invx <= C_INV16(to_integer(p_s16));
                    p_m16  <= C_M16(to_integer(p_j));
                    p_rm   <= C_RM(to_integer(p_j));
                    s_pst  <= P_D;
                when P_D =>
                    v_tp := s_tail * p_m16;
                    p_te <= v_tp(11 downto 4);
                    v_kp := s_kf12 * p_rm;
                    p_ke <= v_kp(19 downto 8);
                    s_pst <= P_E;
                when P_E =>
                    p_we <= '1';
                    p_wa <= s_pi;
                    p_wd <= "00000" & std_logic_vector(p_ke) & std_logic_vector(p_te)
                                    & std_logic_vector(p_invx);
                    if s_pi = 15 then
                        s_temax <= p_te;
                        s_pst   <= P_IDLE;
                    else
                        s_pi  <= s_pi + 1;
                        s_pst <= P_A;
                    end if;
            end case;
        end if;
    end process;

    p_prm : process(clk)
    begin
        if rising_edge(clk) then
            if p_we = '1' then
                s_prm(to_integer(p_wa)) <= p_wd;
            end if;
            s_prm_q <= s_prm(to_integer(s2_l));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2: per drop: distance below its head; luma level lands in fabric.
    --------------------------------------------------------------------------
    p_dist : process(clk)
    begin
        if rising_edge(clk) then
            for d in 0 to C_ND - 1 loop
                s2_dist(d)  <= resize(signed(s_hq(d)(15 downto 5)), 12)
                               - signed(resize(s1_drow, 12));
                s2_alive(d) <= s_iq(4 * d + 3);
            end loop;
            s2_l    <= unsigned(s_lm_q);
            s2_base <= mix16(shift_left(resize(s1_col, 16), 7) + resize(s1_row, 16));
            s2_rx <= s1_rx; s2_ry <= s1_ry;
            s2_render <= s1_render;
            s2_band   <= s1_band;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3 / S4: param RAM read (by luma level), then into fabric.
    --------------------------------------------------------------------------
    p_s34 : process(clk)
        variable v_inv : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- S3
            s3_dist <= s2_dist; s3_alive <= s2_alive; s3_base <= s2_base;
            s3_rx <= s2_rx; s3_ry <= s2_ry;
            s3_render <= s2_render; s3_band <= s2_band;
            -- S4
            v_inv := unsigned(s_prm_q(6 downto 0));
            s4_invx <= v_inv;
            if v_inv > 16 then s4_invy <= v_inv;               -- shrink: both axes
            else               s4_invy <= to_unsigned(16, 7); end if;  -- grow: wider only
            s4_te <= unsigned(s_prm_q(14 downto 7));
            s4_ke <= unsigned(s_prm_q(26 downto 15));
            s4_dist <= s3_dist; s4_alive <= s3_alive; s4_base <= s3_base;
            s4_rx <= s3_rx; s4_ry <= s3_ry;
            s4_render <= s3_render; s4_band <= s3_band;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5: per drop: lit (alive and within this pixel's stretched tail), head
    -- flag, clamped distance for the nearest-drop pick.
    --------------------------------------------------------------------------
    p_lit : process(clk)
    begin
        if rising_edge(clk) then
            for d in 0 to C_ND - 1 loop
                if s4_alive(d) = '1' and s4_dist(d)(11 downto 8) = "0000" and
                   unsigned(s4_dist(d)(7 downto 0)) < s4_te then
                    s5_lit(d) <= '1';
                    s5_dc(d)  <= unsigned(s4_dist(d)(7 downto 0));
                else
                    s5_lit(d) <= '0';
                    s5_dc(d)  <= (others => '1');
                end if;
                if s4_alive(d) = '1' and s4_dist(d) = 0 then
                    s5_head(d) <= '1';
                else
                    s5_head(d) <= '0';
                end if;
            end loop;
            s5_base <= s4_base; s5_ke <= s4_ke;
            s5_invx <= s4_invx; s5_invy <= s4_invy;
            s5_rx <= s4_rx; s5_ry <= s4_ry;
            s5_render <= s4_render; s5_band <= s4_band;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S6: glyph index + nearest lit drop.  Glitch-band cells re-roll every
    -- frame; the head cell every other frame; 1 cell in 8 is a fast "flicker"
    -- cell; the rest re-roll slowly.  Index reduced to 0..C_NGLYPH-1 with a
    -- single conditional subtract.
    --------------------------------------------------------------------------
    p_idx : process(clk)
        variable v_t    : unsigned(14 downto 0);
        variable v_gen  : unsigned(7 downto 0);
        variable v_h    : unsigned(15 downto 0);
        variable v_i    : integer range 0 to 127;
        variable v_head : std_logic;
        variable v_m    : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_head := s5_head(0) or s5_head(1) or s5_head(2);
            v_t := s_T(14 downto 0) + resize(s5_base(7 downto 0), 15);
            if s5_band = '1' then
                v_gen := s_fc(7 downto 0);
            elsif v_head = '1' then
                v_gen := s_fc(7 downto 1) & '1';
            elsif s5_base(10 downto 8) = "000" then
                v_gen := v_t(C_MUT_FAST + 7 downto C_MUT_FAST);
            else
                v_gen := v_t(C_MUT_SLOW + 7 downto C_MUT_SLOW);
            end if;
            v_h := mix16(s5_base xor (v_gen & v_gen) xor x"7E5A");
            v_i := to_integer(v_h(6 downto 0));     -- 0..127
            if v_i >= C_NGLYPH then v_i := v_i - C_NGLYPH; end if;
            s6_idx <= v_i;

            v_m := s5_dc(0);
            if s5_dc(1) < v_m then v_m := s5_dc(1); end if;
            if s5_dc(2) < v_m then v_m := s5_dc(2); end if;
            s6_dmin <= v_m;

            s6_lit  <= s5_lit(0) or s5_lit(1) or s5_lit(2) or s5_band;
            s6_head <= v_head and not s5_band;
            s6_band <= s5_band;
            s6_ke <= s5_ke; s6_invx <= s5_invx; s6_invy <= s5_invy;
            s6_rx <= s5_rx; s6_ry <= s5_ry;
            s6_render <= s5_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S7: scaled glyph-space offsets (rel * 16/s); fade product dist * scale.
    --------------------------------------------------------------------------
    p_scale : process(clk)
    begin
        if rising_edge(clk) then
            s7_px <= resize(s6_rx * signed('0' & s6_invx), 14);
            s7_py <= resize(s6_ry * signed('0' & s6_invy), 14);
            s7_fprod <= s6_dmin * s6_ke;
            s7_idx <= s6_idx;
            s7_lit <= s6_lit; s7_head <= s6_head; s7_render <= s6_render;
            s7_band <= s6_band;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S8: glyph pixel = 8 + offset >> (4 + zoom); inside the glyph box only if
    -- both offsets land in -8..7 (sign-extension test, then flip the MSB).
    -- Fade level = curve(16 * dist / stretched tail) (glitch band: a bright
    -- level that flickers frame to frame).
    --------------------------------------------------------------------------
    p_gcoord : process(clk)
        variable v_ox, v_oy : signed(9 downto 0);
        variable v_bi : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            case to_integer(s_zoom) is
                when 0      => v_ox := s7_px(13 downto 4);
                               v_oy := s7_py(13 downto 4);
                when 1      => v_ox := resize(s7_px(13 downto 5), 10);
                               v_oy := resize(s7_py(13 downto 5), 10);
                when 2      => v_ox := resize(s7_px(13 downto 6), 10);
                               v_oy := resize(s7_py(13 downto 6), 10);
                when others => v_ox := resize(s7_px(13 downto 7), 10);
                               v_oy := resize(s7_py(13 downto 7), 10);
            end case;
            if (v_ox(9 downto 3) = "0000000" or v_ox(9 downto 3) = "1111111") and
               (v_oy(9 downto 3) = "0000000" or v_oy(9 downto 3) = "1111111") then
                s8_in <= '1';
            else
                s8_in <= '0';
            end if;
            s8_gx <= unsigned((not v_ox(3)) & v_ox(2 downto 0));
            s8_gy <= unsigned((not v_oy(3)) & v_oy(2 downto 0));

            if s7_band = '1' then
                v_bi := to_integer(s_fc(1 downto 0));
            elsif s7_fprod(19 downto 12) /= 0 then
                v_bi := 15;
            else
                v_bi := to_integer(s7_fprod(11 downto 8));
            end if;
            s8_lvl <= C_FADE(v_bi);
            if s7_band = '0' and v_bi < 2 then s8_hot <= '1'; else s8_hot <= '0'; end if;
            s8_idx <= s7_idx;
            s8_lit <= s7_lit; s8_head <= s7_head; s8_render <= s7_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S9: glyph ROM read (addr = idx*16 + gy).
    --------------------------------------------------------------------------
    p_rom : process(clk)
        variable v_addr : integer range 0 to C_GROM_DEPTH - 1;
    begin
        if rising_edge(clk) then
            v_addr := s8_idx * 16 + to_integer(s8_gy);
            s9_rb_c <= s_grom(v_addr);
            s9_gx <= s8_gx; s9_in <= s8_in; s9_lvl <= s8_lvl; s9_hot <= s8_hot;
            s9_lit <= s8_lit; s9_head <= s8_head; s9_render <= s8_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S10: glyph bit extract (+ mirror) + 1px horizontal head glow.
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable rbc : std_logic_vector(15 downto 0);
        variable p   : integer range -1 to 16;
        variable v_bit, v_glow : std_logic;
    begin
        if rising_edge(clk) then
            if s_mirror = '1' then rbc := revb(s9_rb_c); else rbc := s9_rb_c; end if;
            p := to_integer(s9_gx);
            v_bit  := getbit(rbc, p) and s9_in;
            v_glow := ( getbit(rbc, p-1) or getbit(rbc, p+1) ) and s9_in
                      and s9_lit and s9_head and (not v_bit) and s_glow_en;
            s10_on   <= v_bit and s9_lit;
            s10_glow <= v_glow;
            s10_head <= s9_head;
            s10_lvl  <= s9_lvl;
            s10_hot  <= s9_hot;
            s10_render <= s9_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S11: level select (head / body * colour gain / glow).
    -- cm = chroma mode (head / glow half, hot rows 3/4); drawn = a rain pixel.
    --------------------------------------------------------------------------
    p_lvl : process(clk)
        variable v_l : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s10_on = '1' and s10_head = '1' then
                v_l := C_L_HEAD; s11_cm <= "10"; s11_drawn <= '1';
            elsif s10_on = '1' then
                v_l := scale8(s10_lvl, unsigned(s_palw(2 downto 0)));
                s11_cm <= '0' & s10_hot; s11_drawn <= '1';
            elsif s10_glow = '1' then
                v_l := C_L_GLOW; s11_cm <= "10"; s11_drawn <= '1';
            else
                v_l := (others => '0'); s11_cm <= "00"; s11_drawn <= '0';
            end if;
            s11_l_y <= v_l; s11_l_u <= v_l; s11_l_v <= v_l;
            s11_render <= s10_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S12: luma above black; chroma partial products (multiplier operand
    -- split hi/lo so each stage is a small multiply or one add).
    --------------------------------------------------------------------------
    p_chr : process(clk)
    begin
        if rising_edge(clk) then
            s12_y   <= C_BLK + s11_l_y;
            s12_puh <= s11_l_u * unsigned(s_palw(30 downto 26));
            s12_pul <= s11_l_u * unsigned(s_palw(25 downto 21));
            s12_pvh <= s11_l_v * unsigned(s_palw(19 downto 15));
            s12_pvl <= s11_l_v * unsigned(s_palw(14 downto 10));
            s12_cm <= s11_cm; s12_drawn <= s11_drawn;
            s12_render <= s11_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S12b: recombine the chroma products.
    --------------------------------------------------------------------------
    p_chr2 : process(clk)
    begin
        if rising_edge(clk) then
            s12b_pu <= shift_left(resize(s12_puh, 20), 5) + resize(s12_pul, 20);
            s12b_pv <= shift_left(resize(s12_pvh, 20), 5) + resize(s12_pvl, 20);
            s12b_y      <= s12_y;
            s12b_cm     <= s12_cm;
            s12b_drawn  <= s12_drawn;
            s12b_render <= s12_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S12c: chroma offset = L*|c|/512, scaled by the chroma mode.
    --------------------------------------------------------------------------
    p_chr3 : process(clk)
    begin
        if rising_edge(clk) then
            case s12b_cm is
                when "10" =>                                   -- half
                    s12c_du <= '0' & s12b_pu(19 downto 10);
                    s12c_dv <= '0' & s12b_pv(19 downto 10);
                when "01" =>                                   -- 3/4
                    s12c_du <= ('0' & s12b_pu(19 downto 10)) + ("00" & s12b_pu(19 downto 11));
                    s12c_dv <= ('0' & s12b_pv(19 downto 10)) + ("00" & s12b_pv(19 downto 11));
                when others =>                                 -- full
                    s12c_du <= s12b_pu(19 downto 9);
                    s12c_dv <= s12b_pv(19 downto 9);
            end case;
            s12c_y      <= s12b_y;
            s12c_drawn  <= s12b_drawn;
            s12c_render <= s12b_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S13: rain chroma = 512 +- offset, clamped; background = black, or the
    -- input video (S10); blanking gate.
    --------------------------------------------------------------------------
    p_color : process(clk)
        variable v_du, v_dv : unsigned(10 downto 0);
        variable v_u, v_v   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_du := s12c_du;
            v_dv := s12c_dv;
            if s_palw(31) = '1' then v_u := 512 - signed(resize(v_du, 13));
            else                     v_u := 512 + signed(resize(v_du, 13)); end if;
            if s_palw(20) = '1' then v_v := 512 - signed(resize(v_dv, 13));
            else                     v_v := 512 + signed(resize(v_dv, 13)); end if;

            if s12c_render = '0' then
                s13_y <= C_BLK; s13_u <= C_MID; s13_v <= C_MID;
            elsif s12c_drawn = '1' then
                s13_y <= s12c_y;
                if v_u < 0 then s13_u <= (others => '0');
                elsif v_u > 1023 then s13_u <= (others => '1');
                else s13_u <= unsigned(v_u(9 downto 0)); end if;
                if v_v < 0 then s13_v <= (others => '0');
                elsif v_v > 1023 then s13_v <= (others => '1');
                else s13_v <= unsigned(v_v(9 downto 0)); end if;
            elsif s_bgvid = '1' then
                s13_y <= unsigned(s_vq2(31 downto 22));
                s13_u <= unsigned(s_vq2(21 downto 12));
                s13_v <= unsigned(s_vq2(11 downto 2));
            else
                s13_y <= C_BLK; s13_u <= C_MID; s13_v <= C_MID;
            end if;
            s13_avid <= s12c_render;
            s13_sync <= s_syncp(C_SYNCD);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output registers.
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(s13_y);
            s_io.u       <= std_logic_vector(s13_u);
            s_io.v       <= std_logic_vector(s13_v);
            s_io.hsync_n <= s13_sync(0);
            s_io.vsync_n <= s13_sync(1);
            s_io.field_n <= s13_sync(2);
            s_io.avid    <= s13_avid;
        end if;
    end process;

    s_io_align_in.y       <= s_io.y;
    s_io_align_in.u       <= s_io.u;
    s_io_align_in.v       <= s_io.v;
    s_io_align_in.hsync_n <= s_io.hsync_n;
    s_io_align_in.vsync_n <= s_io.vsync_n;
    s_io_align_in.avid    <= s_io.avid;
    s_io_align_in.field_n <= s_io.field_n;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (17 -> 20).
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

end architecture matrix_rain;
