-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Teletext v0.2.0: catalogue combo #47 (seed 20260920) -- chromatic glitch /
-- datamosh bars + ASCII / dither-art + scanline shear.  Two knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Bars     fraction of glyph bands displaced (datamosh)
--   K2 Distance mosh displacement (up to +-1020 px, wraps inside the picture)
--   K3 Cell     glyph cell 8 / 16 / 32 / 64 px
--   K4 Contrast density -> glyph mapping (100% = unity: white = densest glyph)
--   K5 Shear    per-line shear amount (+-30 px)
--   K6 Rate     shear wave rate (Wave) / tear probability (Tear)
--
--   S7  Ink     white glyphs on black / black on white
--   S8  Colour  glyphs mono / glyphs take the cell's chroma
--   S9  Mosh    displaces the video / displaces whole glyph cells
--   S10 Shear   triangle wave / random per-line tear
--   S11 Glyphs  ASCII ramp / block-dither set
--   P12 Reveal  video (moshed, sheared) .. glyph rendering
--
-- Line buffer read a line late with the mosh offset.  Each glyph band is
-- sampled on the line directly above it: that line's cell luma means and
-- chroma go to one bank of a two-bank cell RAM, which the band then reads
-- (the grid starts two lines down, so the top band samples the first real
-- line).  Glyph = 8x8 ROM row indexed by 16 density levels.  Shear is
-- applied on the output side through a 64-deep ring, so the glyph raster
-- tears too; each ring word carries its line's glyph row / bank / valid
-- bits, so the glyph lookup always matches the pixel it is drawn on.
--
-- v0.2.0: glyph positions keep counting through blanking (v0.1 drew the
-- right ~33 px of every line as background); the top band no longer shows
-- the previous frame's bottom band; moshed reads wrap inside the C_EDGE
-- inset and sheared edges fill with the line's own edge pixel (v0.1 painted
-- black); Cells mosh wraps on the line's real cell count; the right-edge
-- partial cell is written; a new line's shear is applied when that line
-- starts leaving the ring; Contrast 100% maps white to the densest glyph;
-- 64 px cells; HD HDMI bar conditioning (damier v0.4.4): [1 2 1] on Y/U/V
-- before the gate, and the glyph levels scaled 64 + 13/16 (y - 64) (white
-- ink / paper 776, coloured dark ink 168) -- on the glyphs only, so the
-- video keeps its level at low Reveal; labelled K3 honours the firmware's
-- label-INDEX default until the knob first reads a position; no_rw_check on
-- every RAM (the cell RAM's discarded bank read is the only collision).
-- Latency 59 clocks, all modes, blanking-gated.
--
-- v0.2.2 (2026-10-17): resolution scaling.  The glyph grid counts virtual
-- 1080-line units, stepped per pixel / per field line from the video timing
-- ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5 at 720p, and
-- at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per frame line
-- (v0.2 counted field lines, so SD cells were twice as tall as wide).  Cells
-- are the same fraction of the screen height in every mode and stay square
-- on a 4:3 screen; the glyph row / column come from the virtual position
-- (nearest-neighbour pixel art, so the 8 / 16 px cells lose glyph rows at
-- SD).  Band sample lines and new bands are virtual-band crossings; a cell's
-- luma mean is taken over its first 2^n px (all of them at 1080p and SD).
-- The mosh distance and the shear are in the same units (2/3 of the pixel
-- offsets at 720p, half at SD) and the shear wave's period follows the
-- height.  The ring, line buffers and edge inset stay in real pixels.
-- Where a field line spans >= 1.5 glyph rows (720p 8 px, SD 8 / 16 px, 1080i
-- 8 px) the ink is the average of the sampled row and its neighbour (half
-- ink = the mean of the ink and paper levels): the decimated rows otherwise
-- picked even rows in one band and odd rows in the next, and the dither set
-- striped bright/dark band by band.
-- Timing: the U and V Reveal products take their own copies of the mix
-- (loaded per line), as the shared register's fan-out set the HD path.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture teletext of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (59 + 1 = 60 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 60;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_D    : integer := 33;          -- ring delay at zero shear
    constant C_LAT  : integer := 26 + C_D;    -- output register O25 + 1
    constant C_EDGE : integer := 12;          -- capture's black side columns

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
    -- a real pixel count times the x step: Q.1 virtual units
    function f_xv(x : unsigned; xs : unsigned(2 downto 0); w : natural) return unsigned is
        variable v : unsigned(w - 1 downto 0);
    begin
        v := resize(x, w);
        case to_integer(xs) is
            when 4      => return shift_left(v, 2);
            when 3      => return shift_left(v, 1) + v;
            when others => return shift_left(v, 1);
        end case;
    end function;

    -- ROM word: the glyph row (high byte) and its neighbour row (low byte:
    -- row + 1, row 6 for row 7).  Where a field line spans >= 1.5 glyph rows
    -- (720p / SD small cells, 1080i 8 px) the ink is the 2-row average, so
    -- decimated glyphs keep their density instead of aliasing into stripes.
    type t_glyph is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_GLYPH : t_glyph := (
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0018", x"1818", x"1800", x"0018",
        x"0000", x"0000", x"0000", x"0000", x"0018", x"1818", x"1830", x"3018",
        x"0018", x"1818", x"1800", x"0000", x"0018", x"1818", x"1800", x"0018",
        x"0018", x"1818", x"1800", x"0000", x"0018", x"1818", x"1830", x"3018",
        x"0000", x"0000", x"007e", x"7e00", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0032", x"324c", x"4c00", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"007e", x"7e00", x"007e", x"7e00", x"0000", x"0000", x"0000",
        x"0018", x"1818", x"187e", x"7e7e", x"7e18", x"1818", x"1800", x"0018",
        x"0066", x"663c", x"3cff", x"ff3c", x"3c66", x"6600", x"0000", x"0000",
        x"0000", x"003c", x"3c66", x"6666", x"6666", x"663c", x"3c00", x"003c",
        x"0000", x"0066", x"663c", x"3c18", x"183c", x"3c66", x"6600", x"0066",
        x"2424", x"24ff", x"ff24", x"2424", x"24ff", x"ff24", x"2424", x"2424",
        x"6266", x"660c", x"0c18", x"1830", x"3066", x"6646", x"4600", x"0046",
        x"3c66", x"6666", x"663c", x"3c66", x"6666", x"663c", x"3c00", x"003c",
        x"3c66", x"666e", x"6e6e", x"6e6e", x"6e60", x"603c", x"3c00", x"003c",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"8800", x"0000", x"0000", x"0088", x"8800", x"0000", x"0000", x"0000",
        x"8800", x"0022", x"2200", x"0088", x"8800", x"0022", x"2200", x"0022",
        x"aa00", x"0022", x"2200", x"00aa", x"aa00", x"0022", x"2200", x"0022",
        x"aa00", x"00aa", x"aa00", x"00aa", x"aa00", x"00aa", x"aa00", x"00aa",
        x"aa44", x"44aa", x"aa00", x"00aa", x"aa44", x"44aa", x"aa00", x"00aa",
        x"aa44", x"44aa", x"aa11", x"11aa", x"aa44", x"44aa", x"aa11", x"11aa",
        x"aa55", x"55aa", x"aa11", x"11aa", x"aa55", x"55aa", x"aa11", x"11aa",
        x"aa55", x"55aa", x"aa55", x"55aa", x"aa55", x"55aa", x"aa55", x"55aa",
        x"ee55", x"55aa", x"aa55", x"55ee", x"ee55", x"55aa", x"aa55", x"55aa",
        x"ee55", x"55bb", x"bb55", x"55ee", x"ee55", x"55bb", x"bb55", x"55bb",
        x"ff55", x"55bb", x"bb55", x"55ff", x"ff55", x"55bb", x"bb55", x"55bb",
        x"ff55", x"55ff", x"ff55", x"55ff", x"ff55", x"55ff", x"ff55", x"55ff",
        x"ffdd", x"ddff", x"ff55", x"55ff", x"ffdd", x"ddff", x"ff55", x"55ff",
        x"ffdd", x"ddff", x"ff77", x"77ff", x"ffdd", x"ddff", x"ff77", x"77ff",
        x"ffff", x"ffff", x"ffff", x"ffff", x"ffff", x"ffff", x"ffff", x"ffff"
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    -- ring word: Y10 & U8 & V8 & glyph row 3 & bank & valid & '0'
    type t_ring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal ring : t_ring := (others => x"10202000");
    signal s_rrd : std_logic_vector(31 downto 0) := x"10202000";
    -- cell word: luma mean 10 & U8 & V8 & "000000"
    type t_dens is array (0 to 255) of std_logic_vector(31 downto 0);
    signal dens0, dens1 : t_dens := (others => x"00202000");
    signal s_drd0, s_drd1 : std_logic_vector(31 downto 0) := x"00202000";

    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1, ring, dens0, dens1 : signal is true;

    signal s_k_bars, s_k_dist, s_k_cell, s_k_con, s_k_shear, s_k_rate : unsigned(9 downto 0);
    signal s_sw_black, s_sw_colour, s_sw_cells, s_sw_tear, s_sw_blocks : std_logic;
    signal s_p_reveal : unsigned(9 downto 0);

    signal s_bars, s_dist, s_con, s_rate : unsigned(7 downto 0) := (others => '0');
    signal s_amt    : unsigned(4 downto 0) := (others => '0');
    signal s_live3  : std_logic := '0';
    signal s_ssk    : unsigned(1 downto 0) := "00";       -- K3 decode (live)
    signal s_ss     : unsigned(1 downto 0) := "00";       -- per frame: 8/16/32/64 px
    signal s_cmask  : unsigned(5 downto 0) := "000111";   -- cell px - 1
    signal s_mix    : unsigned(7 downto 0) := (others => '0');
    -- Reveal copies for the U and V products: s_mix fanned out to all three
    -- multipliers and its first hop was half of the HD critical path.  Each
    -- copy loads on a different enable, so yosys cannot merge them.
    signal s_mixu, s_mixv : unsigned(7 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_mix, s_mixu, s_mixv : signal is true;
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_wp     : unsigned(5 downto 0) := (others => '0');
    signal s_tph, s_fph : unsigned(7 downto 0) := (others => '0');

    -- resolution scaling (free-running steps from the timing ID; per-frame
    -- masks; virtual glyph line Q.8; Q.1 virtual x on the input chain)
    signal s_xs     : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_half   : std_logic := '0';                          -- 720p / SD: cell mean over 2^(2+ss) px
    signal s_ka, s_kb, s_da, s_db : unsigned(9 downto 0) := (others => '0');   -- x 2/3 parts
    signal s_k23, s_d23 : unsigned(9 downto 0) := (others => '0');
    signal s_tpa, s_tpb : unsigned(16 downto 0) := (others => '0');
    signal s_tinc   : unsigned(16 downto 0) := to_unsigned(0, 17);   -- wave phase per field line, Q.8
    signal s_tphf   : unsigned(7 downto 0) := (others => '0');       -- wave phase fraction
    signal s_gva, s_gvn : signed(19 downto 0) := to_signed(-512, 20);   -- virtual glyph line, Q.8
    signal s_gv0    : signed(19 downto 0) := to_signed(-512, 20);
    signal s_bandm  : signed(11 downto 0) := to_signed(-8, 12);      -- glyph-line bits >= 3 + ss
    signal s_chm    : unsigned(12 downto 0) := (others => '0');      -- Q.1 x bits >= 4 + ss
    signal s_msh    : unsigned(2 downto 0) := to_unsigned(3, 3);     -- cell mean shift 2..6
    signal s_nsum   : unsigned(6 downto 0) := to_unsigned(8, 7);     -- px summed per cell
    signal s_pcnt   : unsigned(6 downto 0) := (others => '0');
    signal s_xv6, s_xv7, s_xv8 : unsigned(12 downto 0) := (others => '0');
    signal s_bnd78  : std_logic := '0';     -- the S7 pixel starts a cell (the S8 pixel ends one)
    signal s_lwv    : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_dposv  : unsigned(11 downto 0) := (others => '0');
    signal s_off_nxv, s_off_rowv : unsigned(8 downto 0) := to_unsigned(2 * C_D, 9);
    signal s_vxo7   : signed(13 downto 0) := (others => '0');

    -- positions: s_rx counts from each avid rise and keeps counting through
    -- blanking (saturates), so glyph positions past the line end stay true
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '1');
    signal s_tail   : unsigned(6 downto 0) := (others => '1');   -- clocks since avid fell
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid, s_we : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_prev_avid : std_logic := '0';
    signal s_gl : signed(11 downto 0) := to_signed(-2, 12);      -- glyph line = line - 2
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wpar, s_rbank : std_logic := '0';

    -- per-line sequencer
    signal s_lstep  : unsigned(3 downto 0) := x"F";
    signal s_lw     : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_hi1    : unsigned(10 downto 0) := to_unsigned(467, 11);   -- lw - 1 - C_EDGE
    signal s_hi0    : signed(12 downto 0) := to_signed(468, 13);       -- lw - C_EDGE
    signal s_span   : signed(11 downto 0) := to_signed(456, 12);       -- lw - 2 C_EDGE
    signal s_wcm    : unsigned(11 downto 0) := (others => '0');
    signal s_ncells : unsigned(8 downto 0) := to_unsigned(60, 9);
    signal s_samp, s_newband, s_first, s_wbank : std_logic := '0';
    signal s_row_l  : unsigned(2 downto 0) := "000";
    signal s_dbank_l, s_valid_l : std_logic := '0';
    signal s_tri, s_tear : signed(7 downto 0) := (others => '0');
    signal s_shp   : signed(13 downto 0) := (others => '0');
    signal s_off_nx, s_off_row : unsigned(5 downto 0) := to_unsigned(C_D, 6);
    signal s_mp    : signed(18 downto 0) := (others => '0');
    signal s_mraw  : signed(11 downto 0) := (others => '0');   -- band displacement, px
    signal s_mhi, s_mlo : std_logic := '0';
    signal s_dn    : signed(11 downto 0) := (others => '0');   -- in (-span, span)
    signal s_dv    : signed(11 downto 0) := (others => '0');   -- video displacement
    signal s_dpos  : unsigned(10 downto 0) := (others => '0'); -- dn mod lw
    signal s_dvp, s_dvm, s_elo, s_ehi : signed(12 downto 0) := (others => '0');
    signal s_dc    : unsigned(8 downto 0) := (others => '0');  -- cell displacement
    signal s_dcm   : signed(9 downto 0) := (others => '0');    -- dc - ncells

    -- input side (S = clocks after the pixel was registered at S1)
    type t_x is array (2 to 9) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '1'));
    type t_b is array (2 to 9) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_xa2   : unsigned(10 downto 0) := (others => '0');    -- read column (held/filled in blanking)
    signal s_xc3   : unsigned(10 downto 0) := (others => '0');
    signal s_u4, s_up4, s_um4 : signed(12 downto 0) := (others => '0');
    signal s_lt4, s_ge4 : std_logic := '0';
    signal s_addr5 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank, s_a_first : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_csum  : unsigned(15 downto 0) := (others => '0');
    signal s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_dwe9  : std_logic := '0';
    signal s_da9   : unsigned(7 downto 0) := (others => '0');
    signal s_dd9   : std_logic_vector(31 downto 0) := (others => '0');

    -- output side (O = clocks after the pixel's ring read was issued + ...)
    signal s_xo7   : signed(11 downto 0) := (others => '0');
    signal s_ra7   : unsigned(5 downto 0) := (others => '0');
    signal s_cia8  : unsigned(8 downto 0) := (others => '0');
    signal s_cib8  : signed(9 downto 0) := (others => '0');
    signal s_ci9   : unsigned(7 downto 0) := (others => '0');
    signal s_dens11 : unsigned(9 downto 0) := (others => '0');
    signal s_cu11, s_cv11 : unsigned(7 downto 0) := x"80";
    signal s_d12   : unsigned(9 downto 0) := (others => '0');
    signal s_dk13  : unsigned(10 downto 0) := (others => '0');
    signal s_p14   : unsigned(18 downto 0) := (others => '0');
    signal s_romaddr15 : unsigned(7 downto 0) := (others => '0');
    signal s_pat16, s_pat17 : std_logic_vector(15 downto 0) := (others => '0');
    signal s_avg    : std_logic := '0';   -- field line spans >= 1.5 glyph rows: 2-row ink average
    signal s_inky18, s_inku18, s_inkv18 : unsigned(9 downto 0) := (others => '0');
    signal s_dy19, s_du19, s_dv19 : signed(10 downto 0) := (others => '0');
    signal s_py20, s_pu20, s_pv20 : signed(19 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_c10 is array (9 to 20) of unsigned(9 downto 0);
    signal s_wy, s_wu, s_wv : t_c10 := (others => (others => '0'));
    type t_c8 is array (12 to 17) of unsigned(7 downto 0);
    signal s_cu, s_cv : t_c8 := (others => x"80");
    type t_g3 is array (8 to 17) of unsigned(2 downto 0);
    signal s_gcol : t_g3 := (others => (others => '0'));
    type t_bo is array (8 to 17) of std_logic;
    signal s_bgo : t_bo := (others => '1');
    type t_vo is array (9 to 17) of std_logic;
    signal s_val : t_vo := (others => '0');
    type t_r3 is array (9 to 14) of unsigned(2 downto 0);
    signal s_row : t_r3 := (others => (others => '0'));
    signal s_dbank9, s_dbank10 : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_bars  <= unsigned(registers_in(0));
    s_k_dist  <= unsigned(registers_in(1));
    s_k_cell  <= unsigned(registers_in(2));
    s_k_con   <= unsigned(registers_in(3));
    s_k_shear <= unsigned(registers_in(4));
    s_k_rate  <= unsigned(registers_in(5));
    s_sw_black  <= registers_in(6)(0);
    s_sw_colour <= registers_in(6)(1);
    s_sw_cells  <= registers_in(6)(2);
    s_sw_tear   <= registers_in(6)(3);
    s_sw_blocks <= registers_in(6)(4);
    s_p_reveal <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_t : unsigned(6 downto 0);
        variable v_s, v_o : signed(7 downto 0);
        variable v_w : signed(11 downto 0);
        variable v_g, v_gn : signed(11 downto 0);
        variable v_p : unsigned(15 downto 0);
        variable v_y2 : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            if s_xs = 2 then s_half <= '0'; else s_half <= '1'; end if;
            -- next line's virtual glyph line; the field's start (two lines up,
            -- a bottom field half a step further down)
            s_gvn <= s_gva + signed(resize(s_ys, 20));
            v_y2 := signed(shift_left(resize(s_ys, 20), 1));
            if data_in.field_n = '0' then
                s_gv0 <= signed(resize(s_ys(11 downto 1), 20)) - v_y2;
            else
                s_gv0 <= -v_y2;
            end if;
            -- wave phase per field line: (Rate / 32) x ys (Q.8; 1080p: Rate / 32)
            s_tpa <= resize(s_ys, 14) * s_rate(5 downto 3);
            s_tpb <= shift_left(resize(s_ys, 15) * s_rate(7 downto 6), 3);
            s_tinc <= s_tpa + s_tpb;
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_saw_active <= '1'; end if;
            -- position of the S1 pixel + 1; keeps counting past the line end
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_rx <= to_unsigned(1, 11);
            elsif s_rx /= 2047 then
                s_rx <= s_rx + 1;
            end if;
            if data_in.avid = '1' then
                s_tail <= (others => '0');
            elsif s_tail /= 127 then
                s_tail <= s_tail + 1;
            end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_wp <= s_wp + 1;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            s_x(2) <= s_rx - 1; s_avd(2) <= s_in_avid;
            for i in 3 to 9 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;
            -- virtual x (Q.1) beside x(6..8); cell boundary between x(8) and x(7)
            s_xv6 <= f_xv(s_x(5), s_xs, 13);
            s_xv7 <= s_xv6; s_xv8 <= s_xv7;
            if ((s_xv6 xor s_xv7) and s_chm) /= 0 then s_bnd78 <= '1'; else s_bnd78 <= '0'; end if;
            -- read column: the pixel itself; in blanking the last column is
            -- held (right fill) until 64 clocks after the line, then the first
            -- (left fill for the next line); the inset clamp follows
            if s_in_avid = '1' then
                s_xa2 <= s_rx - 1;
            elsif s_tail(6) = '1' then
                s_xa2 <= (others => '0');
            end if;

            s_bars <= s_k_bars(9 downto 2);
            -- Distance and Shear in virtual units: 2/3 of the pixels at 720p,
            -- half at SD
            s_da <= ('0' & s_k_dist(9 downto 1)) + ("000" & s_k_dist(9 downto 3));
            s_db <= ("00000" & s_k_dist(9 downto 5)) + ("0000000" & s_k_dist(9 downto 7));
            s_d23 <= s_da + s_db;
            s_ka <= ('0' & s_k_shear(9 downto 1)) + ("000" & s_k_shear(9 downto 3));
            s_kb <= ("00000" & s_k_shear(9 downto 5)) + ("0000000" & s_k_shear(9 downto 7));
            s_k23 <= s_ka + s_kb;
            case to_integer(s_xs) is
                when 4 =>
                    s_dist <= '0' & s_k_dist(9 downto 3);
                    s_amt  <= '0' & s_k_shear(9 downto 6);
                when 3 =>
                    s_dist <= s_d23(9 downto 2);
                    s_amt  <= s_k23(9 downto 5);
                when others =>
                    s_dist <= s_k_dist(9 downto 2);
                    s_amt  <= s_k_shear(9 downto 5);
            end case;
            -- the firmware loads a labelled default as the label INDEX;
            -- honour 0..3 as an index only until the knob first reads a position
            if s_k_cell >= 4 then s_live3 <= '1'; end if;
            if s_live3 = '0' and s_k_cell < 4 then s_ssk <= s_k_cell(1 downto 0); else s_ssk <= s_k_cell(9 downto 8); end if;
            s_con <= s_k_con(9 downto 2);
            s_rate <= s_k_rate(9 downto 2);
            s_mix <= s_p_reveal(9 downto 2);
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then s_mixu <= s_p_reveal(9 downto 2); end if;
            if s_prev_avid = '1' and data_in.avid = '0' then s_mixv <= s_p_reveal(9 downto 2); end if;

            -- a new line's shear takes effect as its first pixel enters the
            -- ring (the previous line has left it by then: blank > 63)
            if s_avd(7) = '1' and s_avd(8) = '0' then
                s_off_row <= s_off_nx; s_off_rowv <= s_off_nxv;
            end if;

            -- per-line sequencer (one small op per state), starts at hsync
            case to_integer(s_lstep) is
                when 0 =>
                    s_lw <= s_line_width;
                    s_lwv <= f_xv(s_line_width, s_xs, 13)(12 downto 1);
                    -- virtual glyph line G (this line) and Gn (the next): the
                    -- band ends on this line when the next one is in another
                    -- band; a band starts where the previous line ended one
                    -- (1080p: G and cmask = cmask / = 0)
                    v_g := s_gva(19 downto 8); v_gn := s_gvn(19 downto 8);
                    if ((v_g xor v_gn) and s_bandm) /= 0 then s_samp <= '1'; else s_samp <= '0'; end if;
                    s_newband <= s_samp;
                    if s_gl = -2 then s_first <= '1'; else s_first <= '0'; end if;
                    s_valid_l <= not v_g(11);
                    case to_integer(s_ss) is
                        when 0 => s_row_l <= unsigned(v_g(2 downto 0)); s_dbank_l <= v_g(3); s_wbank <= not v_g(3);
                        when 1 => s_row_l <= unsigned(v_g(3 downto 1)); s_dbank_l <= v_g(4); s_wbank <= not v_g(4);
                        when 2 => s_row_l <= unsigned(v_g(4 downto 2)); s_dbank_l <= v_g(5); s_wbank <= not v_g(5);
                        when others => s_row_l <= unsigned(v_g(5 downto 3)); s_dbank_l <= v_g(6); s_wbank <= not v_g(6);
                    end case;
                    if s_tph(7) = '1' then v_t := not s_tph(6 downto 0); else v_t := s_tph(6 downto 0); end if;
                    s_tri <= signed(resize(v_t, 8)) - 64;
                    if s_lfsr(15 downto 8) < s_rate then s_tear <= signed(resize(s_lfsr(6 downto 0), 8)) - 64; else s_tear <= (others => '0'); end if;
                    -- wave phase: Q8.8, steps follow the field-line height
                    v_p := shift_left(resize(s_tph, 16), 8) + resize(s_tphf, 16) + s_tinc(15 downto 0);
                    s_tph <= v_p(15 downto 8); s_tphf <= v_p(7 downto 0);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    if s_sw_tear = '1' then s_shp <= s_tear * signed(resize(s_amt, 6)); else s_shp <= s_tri * signed(resize(s_amt, 6)); end if;
                    s_mp <= (signed(resize(s_lfsr(8 downto 0), 10)) - 256) * signed(resize(s_dist, 9));
                    s_hi1  <= s_lw - (C_EDGE + 1);
                    s_hi0  <= signed(resize(s_lw, 13)) - C_EDGE;
                    s_span <= signed(resize(s_lw, 12)) - 2 * C_EDGE;
                    s_wcm  <= s_lwv + resize(s_cmask, 12);   -- virtual width: cell count
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    v_s := resize(shift_right(s_shp, 6), 8);
                    if v_s > 30 then v_s := to_signed(30, 8); elsif v_s < -30 then v_s := to_signed(-30, 8); end if;
                    -- slice, not resize: synthesis truncates either way, but a
                    -- narrowing signed resize folds 32..63 in GHDL simulation
                    v_o := to_signed(C_D, 8) + v_s;
                    s_off_nx <= unsigned(v_o(5 downto 0));
                    if s_newband = '1' then
                        if s_lfsr(7 downto 0) < s_bars then s_mraw <= resize(shift_right(s_mp, 6), 12); else s_mraw <= (others => '0'); end if;
                    end if;
                    case to_integer(s_ss) is
                        when 0 => s_ncells <= resize(s_wcm(11 downto 3), 9);
                        when 1 => s_ncells <= resize(s_wcm(11 downto 4), 9);
                        when 2 => s_ncells <= resize(s_wcm(11 downto 5), 9);
                        when others => s_ncells <= resize(s_wcm(11 downto 6), 9);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_off_nxv <= f_xv(s_off_nx, s_xs, 9);
                    if s_mraw >= s_span then s_mhi <= '1'; else s_mhi <= '0'; end if;
                    if s_mraw <= -s_span then s_mlo <= '1'; else s_mlo <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_mhi = '1' then s_dn <= s_mraw - s_span;
                    elsif s_mlo = '1' then s_dn <= s_mraw + s_span;
                    else s_dn <= s_mraw; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_sw_cells = '1' then s_dv <= (others => '0'); else s_dv <= s_dn; end if;
                    if s_dn < 0 then
                        v_w := s_dn + signed(resize(s_lw, 12));
                        s_dpos <= unsigned(v_w(10 downto 0));
                    else
                        s_dpos <= unsigned(s_dn(10 downto 0));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_dvp <= resize(s_dv, 13) + resize(s_span, 13);
                    s_dvm <= resize(s_dv, 13) - resize(s_span, 13);
                    s_elo <= to_signed(C_EDGE, 13) - resize(s_dv, 13);
                    s_ehi <= s_hi0 - resize(s_dv, 13);
                    -- Cells displacement in virtual columns
                    s_dposv <= f_xv(s_dpos, s_xs, 13)(12 downto 1);
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    case to_integer(s_ss) is
                        when 0 => s_dc <= resize(s_dposv(11 downto 3), 9);
                        when 1 => s_dc <= resize(s_dposv(11 downto 4), 9);
                        when 2 => s_dc <= resize(s_dposv(11 downto 5), 9);
                        when others => s_dc <= resize(s_dposv(11 downto 6), 9);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_dcm <= signed(resize(s_dc, 10)) - signed(resize(s_ncells, 10));
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_gl <= s_gl + 1; s_gva <= s_gvn;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            -- frame events: resets are idempotent; toggles need the saw-active guard
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_gl <= to_signed(-2, 12); s_wpar <= '0';
                s_gva <= s_gv0;
                s_mraw <= (others => '0');
                s_ss <= s_ssk;
                case to_integer(s_ssk) is
                    when 0 => s_cmask <= "000111"; s_bandm <= to_signed(-8, 12);  s_chm <= to_unsigned(8176, 13);
                    when 1 => s_cmask <= "001111"; s_bandm <= to_signed(-16, 12); s_chm <= to_unsigned(8160, 13);
                    when 2 => s_cmask <= "011111"; s_bandm <= to_signed(-32, 12); s_chm <= to_unsigned(8128, 13);
                    when others => s_cmask <= "111111"; s_bandm <= to_signed(-64, 12); s_chm <= to_unsigned(8064, 13);
                end case;
                -- 2-row glyph average where a field line spans >= 1.5 rows
                if s_ys >= shift_left(to_unsigned(384, 12), to_integer(s_ssk)) then s_avg <= '1'; else s_avg <= '0'; end if;
                -- cell mean: all 8 x 2^ss px at 1080p; at 720p / SD the first
                -- 4 x 2^ss (every px of an SD cell, the 720p cells' common part)
                if s_half = '1' then
                    s_msh <= resize(s_ssk, 3) + 2;
                    s_nsum <= shift_left(to_unsigned(4, 7), to_integer(s_ssk));
                else
                    s_msh <= resize(s_ssk, 3) + 3;
                    s_nsum <= shift_left(to_unsigned(8, 7), to_integer(s_ssk));
                end if;
                if s_saw_active = '1' then s_fph <= s_fph + 3; end if;
                s_tph <= s_fph; s_tphf <= (others => '0');
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

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rrd <= ring(to_integer(s_ra7));
            ring(to_integer(s_wp)) <= std_logic_vector(s_wet_y) & std_logic_vector(s_wet_u(9 downto 2)) & std_logic_vector(s_wet_v(9 downto 2))
                                      & std_logic_vector(s_row_l) & s_dbank_l & s_valid_l & '0';
        end if;
    end process;
    p_dens0 : process(clk)
    begin
        if rising_edge(clk) then
            s_drd0 <= dens0(to_integer(s_ci9));
            if s_dwe9 = '1' and s_wbank = '0' then dens0(to_integer(s_da9)) <= s_dd9; end if;
        end if;
    end process;
    p_dens1 : process(clk)
    begin
        if rising_edge(clk) then
            s_drd1 <= dens1(to_integer(s_ci9));
            if s_dwe9 = '1' and s_wbank = '1' then dens1(to_integer(s_da9)) <= s_dd9; end if;
        end if;
    end process;
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_pat16 <= C_GLYPH(to_integer(s_romaddr15));
            s_pat17 <= s_pat16;
        end if;
    end process;

    p_pix : process(clk)
        variable v_cm : unsigned(9 downto 0);
        variable v_ink, v_half, v_b0, v_b1 : std_logic;
        variable v_g : unsigned(6 downto 0);
        variable v_ci : unsigned(7 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------------
            -- input side: moshed, wrapped, inset-clamped line-buffer read
            ------------------------------------------------------------------
            -- S3: clamp into the inset (the capture's side columns are black)
            if s_xa2 < C_EDGE then
                s_xc3 <= to_unsigned(C_EDGE, 11);
            elsif s_xa2 > s_hi1 then
                s_xc3 <= s_hi1;
            else
                s_xc3 <= s_xa2;
            end if;
            -- S4: displaced column and its two wrap candidates, range tests
            s_u4  <= signed(resize(s_xc3, 13)) + resize(s_dv, 13);
            s_up4 <= signed(resize(s_xc3, 13)) + s_dvp;
            s_um4 <= signed(resize(s_xc3, 13)) + s_dvm;
            if signed(resize(s_xc3, 13)) < s_elo then s_lt4 <= '1'; else s_lt4 <= '0'; end if;
            if signed(resize(s_xc3, 13)) >= s_ehi then s_ge4 <= '1'; else s_ge4 <= '0'; end if;
            -- S5: read address, wrapped inside [C_EDGE, lw - C_EDGE)
            if s_lt4 = '1' then s_addr5 <= unsigned(s_up4(10 downto 0));
            elsif s_ge4 = '1' then s_addr5 <= unsigned(s_um4(10 downto 0));
            else s_addr5 <= unsigned(s_u4(10 downto 0)); end if;
            -- S7: fetch (S6 = RAM out); the frame's first line has no line above
            s_a_rbank <= s_rbank; s_a_first <= s_first;
            if s_a_first = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- S8: cell sum of the S7 pixel (its first nsum px), sampled chroma
            if s_bnd78 = '1' then
                s_csum <= resize(s_wet_y, 16); s_pcnt <= to_unsigned(1, 7);
            elsif s_pcnt < s_nsum then
                s_csum <= s_csum + resize(s_wet_y, 16); s_pcnt <= s_pcnt + 1;
            end if;
            s_u8 <= s_wet_u; s_v8 <= s_wet_v;
            -- S9: cell write on sample lines (the S8 pixel ends its cell, or the line)
            if s_samp = '1' and s_avd(8) = '1' and (s_bnd78 = '1' or s_avd(7) = '0') then s_dwe9 <= '1'; else s_dwe9 <= '0'; end if;
            case to_integer(s_ss) is
                when 0 => s_da9 <= s_xv8(11 downto 4);
                when 1 => s_da9 <= s_xv8(12 downto 5);
                when 2 => s_da9 <= resize(s_xv8(12 downto 6), 8);
                when others => s_da9 <= resize(s_xv8(12 downto 7), 8);
            end case;
            case to_integer(s_msh) is
                when 2 => v_cm := s_csum(11 downto 2);
                when 3 => v_cm := s_csum(12 downto 3);
                when 4 => v_cm := s_csum(13 downto 4);
                when 5 => v_cm := s_csum(14 downto 5);
                when others => v_cm := s_csum(15 downto 6);
            end case;
            s_dd9 <= std_logic_vector(v_cm) & std_logic_vector(s_u8(9 downto 2)) & std_logic_vector(s_v8(9 downto 2)) & "000000";

            ------------------------------------------------------------------
            -- output side: the pixel leaving the ring and its glyph
            ------------------------------------------------------------------
            -- O7: its position (x keeps counting through blanking) and ring address
            s_xo7 <= signed(resize(s_x(7), 12)) - signed(resize(s_off_row, 12));
            s_vxo7 <= signed(resize(s_xv7, 14)) - signed(resize(s_off_rowv, 14));
            s_ra7 <= s_wp - s_off_row;
            -- O8: cell index (+ Cells displacement, two wrap candidates), glyph
            -- column -- from the virtual position (Q.1)
            case to_integer(s_ss) is
                when 0 => v_ci := unsigned(s_vxo7(11 downto 4)); s_gcol(8) <= unsigned(s_vxo7(3 downto 1));
                when 1 => v_ci := unsigned(s_vxo7(12 downto 5)); s_gcol(8) <= unsigned(s_vxo7(4 downto 2));
                when 2 => v_ci := unsigned(s_vxo7(13 downto 6)); s_gcol(8) <= unsigned(s_vxo7(5 downto 3));
                when others => v_ci := resize(unsigned(s_vxo7(13 downto 7)), 8); s_gcol(8) <= unsigned(s_vxo7(6 downto 4));
            end case;
            s_cia8 <= resize(v_ci, 9) + s_dc;
            s_cib8 <= signed(resize(v_ci, 10)) + s_dcm;
            if s_xo7 < 0 or s_xo7 >= signed(resize(s_lw, 12)) then s_bgo(8) <= '1'; else s_bgo(8) <= '0'; end if;
            -- O9: ring out -> fabric; wrapped cell address
            s_wy(9) <= unsigned(s_rrd(31 downto 22)); s_wu(9) <= unsigned(s_rrd(21 downto 14)) & "00"; s_wv(9) <= unsigned(s_rrd(13 downto 6)) & "00";
            s_row(9) <= unsigned(s_rrd(5 downto 3)); s_dbank9 <= s_rrd(2); s_val(9) <= s_rrd(1);
            if s_cib8 < 0 then s_ci9 <= s_cia8(7 downto 0); else s_ci9 <= unsigned(s_cib8(7 downto 0)); end if;
            -- O11: cell RAM out (O10) -> fabric, the band's bank
            s_dbank10 <= s_dbank9;
            if s_dbank10 = '1' then
                s_dens11 <= unsigned(s_drd1(31 downto 22)); s_cu11 <= unsigned(s_drd1(21 downto 14)); s_cv11 <= unsigned(s_drd1(13 downto 6));
            else
                s_dens11 <= unsigned(s_drd0(31 downto 22)); s_cu11 <= unsigned(s_drd0(21 downto 14)); s_cv11 <= unsigned(s_drd0(13 downto 6));
            end if;
            -- O12
            if s_dens11 < 64 then s_d12 <= (others => '0'); else s_d12 <= s_dens11 - 64; end if;
            -- O13: x 1.156 so that white (876) reaches the densest glyph at 100%
            s_dk13 <= resize(s_d12, 11) + resize(s_d12(9 downto 3), 11) + resize(s_d12(9 downto 5), 11);
            -- O14
            s_p14 <= s_dk13 * s_con;
            -- O15: glyph index and row
            v_g := s_p14(18 downto 12);
            if v_g > 15 then v_g := to_unsigned(15, 7); end if;
            s_romaddr15 <= s_sw_blocks & v_g(3 downto 0) & s_row(14);
            -- O18: ink (O16 ROM, O17 fabric)
            v_b0 := s_pat17(15 - to_integer(s_gcol(17)));
            v_b1 := s_pat17(7 - to_integer(s_gcol(17)));
            if s_avg = '1' then
                v_ink := v_b0 and v_b1; v_half := v_b0 xor v_b1;
            else
                v_ink := v_b0; v_half := '0';
            end if;
            if s_bgo(17) = '1' or s_val(17) = '0' then v_ink := '0'; v_half := '0'; end if;
            -- glyph levels pre-scaled 64 + 13/16 (y - 64): white 776 (the bar
            -- fix's luma cut, on the glyphs only, so the video keeps its level);
            -- half ink is the mean of the ink and paper levels
            if v_half = '1' then
                if s_sw_colour = '1' then
                    if s_sw_black = '1' then s_inky18 <= to_unsigned(472, 10); else s_inky18 <= to_unsigned(420, 10); end if;
                    -- (c x 4 + 512) / 2 = (c + 128) x 2
                    s_inku18 <= s_cu(17)(7) & (not s_cu(17)(7)) & s_cu(17)(6 downto 0) & '0';
                    s_inkv18 <= s_cv(17)(7) & (not s_cv(17)(7)) & s_cv(17)(6 downto 0) & '0';
                else
                    s_inky18 <= to_unsigned(420, 10);
                    s_inku18 <= to_unsigned(512, 10); s_inkv18 <= to_unsigned(512, 10);
                end if;
            elsif v_ink = '1' and s_sw_colour = '1' then
                if s_sw_black = '1' then s_inky18 <= to_unsigned(168, 10); else s_inky18 <= to_unsigned(776, 10); end if;
                s_inku18 <= s_cu(17) & "00"; s_inkv18 <= s_cv(17) & "00";
            else
                if (v_ink xor s_sw_black) = '1' then s_inky18 <= to_unsigned(776, 10); else s_inky18 <= to_unsigned(64, 10); end if;
                s_inku18 <= to_unsigned(512, 10); s_inkv18 <= to_unsigned(512, 10);
            end if;
            -- O19
            s_dy19 <= signed(resize(s_inky18, 11)) - signed(resize(s_wy(18), 11));
            s_du19 <= signed(resize(s_inku18, 11)) - signed(resize(s_wu(18), 11));
            s_dv19 <= signed(resize(s_inkv18, 11)) - signed(resize(s_wv(18), 11));
            -- O20
            s_py20 <= s_dy19 * signed(resize(s_mix, 9));
            s_pu20 <= s_du19 * signed(resize(s_mixu, 9));
            s_pv20 <= s_dv19 * signed(resize(s_mixv, 9));
            -- O21
            s_y21 <= f_clamp10(signed(resize(s_wy(20), 13)) + resize(shift_right(s_py20, 8), 13));
            s_u21 <= f_clamp10(signed(resize(s_wu(20), 13)) + resize(shift_right(s_pu20, 8), 13));
            s_v21 <= f_clamp10(signed(resize(s_wv(20), 13)) + resize(shift_right(s_pv20, 8), 13));
            -- O22: neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(C_D + 21) = '1' then
                s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
            -- O23..O24: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y22; s_g2y <= s_g1y;
            s_g1u <= s_u22; s_g2u <= s_g1u;
            s_g1v <= s_v22; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y22, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u22, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v22, 12);
            s_y24 <= v_sy(11 downto 2); s_u24 <= v_su(11 downto 2); s_v24 <= v_sv(11 downto 2);
            -- O25: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;

            -- carry chains
            for i in 10 to 20 loop s_wy(i) <= s_wy(i - 1); s_wu(i) <= s_wu(i - 1); s_wv(i) <= s_wv(i - 1); end loop;
            s_cu(12) <= s_cu11; s_cv(12) <= s_cv11;
            for i in 13 to 17 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            for i in 9 to 17 loop s_gcol(i) <= s_gcol(i - 1); s_bgo(i) <= s_bgo(i - 1); end loop;
            for i in 10 to 17 loop s_val(i) <= s_val(i - 1); end loop;
            for i in 10 to 14 loop s_row(i) <= s_row(i - 1); end loop;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y25);
    s_io_align_in.u       <= std_logic_vector(s_u25);
    s_io_align_in.v       <= std_logic_vector(s_v25);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (59 -> 60).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture teletext;
