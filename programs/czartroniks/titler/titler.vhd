-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Titler: lo-res video character generator in the style of 1980s/90s
-- hardware titlers (Chyron / Video Toaster CG / Videonics TitleMaker).
--
-- Renders two lines of 16 characters each (32 cells total) over the input
-- video.  Knobs place the text block, pick one of eight edge/box "styles",
-- choose the ink colour and edit the message.  The P12 slider is a bipolar
-- news-ticker Crawl: centre holds the text still, above centre it crawls
-- left, below centre right, faster toward either end.  Size lives on K5
-- (outside Edit Mode) and runs from small caption to full-height hero type.
--
-- Everything geometric derives from the RUNTIME-MEASURED active raster, so
-- the position knobs and the Size control have the same full useful travel
-- in every video mode (this was the biggest defect in v1.0: the H/V knobs
-- were hard-coded to a 4092-pixel span, which crammed all of SD's placement
-- into the bottom 20% of the knob).
--
-- v2.2.3: K3 Style / K4 Color label-index fix folded in from community
-- v2.2.2 (a value below 8 is the label index the firmware loads).
--
-- Resolution scaling (v2.2.3): sizes and layout are worked out in virtual
-- 1080-line units (the video timing ID gives the per-mode steps), so the
-- text is the same size relative to the screen HEIGHT in every mode and
-- glyphs keep their shape: SD is a 1440-unit-wide (4:3) window, 720p steps
-- 1.5 units per pixel and line, interlaced modes 2x per field line.  The
-- Size range is the 1080p one (glyph pixel 3..29 units) everywhere, the
-- positions and the crawl are fractions of the virtual width/height, and
-- the accumulators step size_step * x-step per pixel and size_step *
-- y-step per field line.  Before,
-- the size came from meas_w and one step served both axes, so 480i glyphs
-- were ~2x tall and 720p/SD got their own width-based size ranges.  1080p
-- is bit-identical (every step is exactly 1.0 there).
--
-- Pipeline (7 cycles from data_in to data_out):
--   S1: active-region counters (h_act / v_act) and Bresenham position
--       accumulators (hpos_acc / vpos_acc) mapping physical pixels/lines to
--       glyph-pixel indices at any integer scale.  The accumulator walks by
--       step = floor(2^16 / scale) per pixel (or per ACTIVE line), so no
--       multiplier is needed in the pixel path - only a 32-entry step LUT.
--   S2: extract char/line/glyph-x/y from the accumulator integer parts;
--       apply the centring offset; compute in_text / in_box / margin flags.
--   S3: text-buffer read, cursor box and edit underscore.
--   S4: font ROM read - the WHOLE 8x8 glyph as one 64-bit word.
--   S5: select glyph rows y-1, y, y+1 out of that word; palette index.
--   S6: 3x3 dilation and the diagonal shifts -> self / outline / shadow /
--       highlight bits; palette lookup.
--   S7: pixel-class decode, output mux, blanking gate.
-- The crawl adds nothing to this pipeline: it is one more term in the
-- per-field text origin, computed by the vblank sequencer.
--
-- Why the whole glyph is fetched at once: with rows y-1, y and y+1 in hand
-- the entire 3x3 neighbourhood costs a couple of OR gates
--     dil  = row(y-1) or row(y) or row(y+1)
--     dil3 = dil or (dil sll 1) or (dil srl 1)
-- which buys drop shadow, black edge, hollow and emboss for essentially
-- nothing.  Glyphs live in columns 1..5 of an 8-wide cell and rows 0..6 of
-- an 8-tall cell, so a one-glyph-pixel edge always stays inside its own
-- cell HORIZONTALLY.  Vertically it does not: the row above a glyph's top
-- row belongs to the cell above, which holds a different character.  So each
-- text line is given its own dedicated top-margin row (rows 0 and 9 of the
-- 18-row block).  On those rows the pipeline fetches the line's character but
-- forces row(y-1) = row(y) = 0 and row(y+1) = glyph row 0, which is exactly
-- the "one row above the glyph" the dilation needs.  Without it every
-- character loses the top of its outline.
-- At scale 8 an edge is 8 physical pixels thick - the chunky look of the
-- period hardware.
--
-- Resources:
--   - Font ROM: 128 glyphs x 64 bits = 8 kbit (EBR)
--   - Text buffer: 32 entries x 6 bits = 192 register bits
--   - Step LUT: 32 entries x 17 bits (LUT-ROM)
--   - One shared serial shift-add multiplier for the per-frame geometry
--     and the crawl velocity
--
-- Register Map:
--   reg(0)    : H Position    0..1023 -> text centre across the active width
--   reg(1)    : V Position    0..1023 -> text centre down the active height
--   reg(2)    : Style         top 3 bits select one of 8 looks
--   reg(3)    : Colour / Erase   Edit Mode OFF -> top 3 bits pick 1 of 8 inks
--                                Edit Mode ON  -> top 5 bits drag the cursor
--                                                 and blank each cell passed
--   reg(4)    : Size / Cursor    Edit Mode OFF -> screen-relative Size
--                                Edit Mode ON  -> top 5 bits select 1 of 32
--                                                 cells (bit 4 = line)
--   reg(5)    : Letter        top 6 bits = char code 0..63
--   reg(6)(0) : Edit Mode     1 = enable cursor + writes
--   reg(6)(1) : Background    0 = passthrough video, 1 = solid black
--   reg(6)(2) : Matte         1 = glyph pixels reveal the incoming video
--   reg(6)(3) : Font          0 = default, 1 = sci-fi
--   reg(6)(4) : Justify       0 = left, 1 = centre each line
--   reg(7)    : Crawl         P12 slider, bipolar: centre = stopped,
--                             above = crawl left, below = crawl right
--
-- Colour convention: the palette below is authored in standard BT.601 and
-- swapped (U<->V) at the single compose point in S7, because Videomancer
-- hardware wires Cr into the U register.  The passthrough / matte paths are
-- NOT swapped - raw in is already raw out.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.titler_font_pkg.all;

architecture titler of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (7 + 1 = 8 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    --==========================================================================
    -- Constants
    --==========================================================================
    constant C_LATENCY      : integer := 7;
    constant C_DELAY_DEPTH  : integer := C_LATENCY - 1;  -- shift register depth
    constant C_TEXT_COLS    : integer := 16;             -- chars per line
    constant C_TEXT_ROWS    : integer := 2;              -- number of lines
    constant C_TEXT_LEN     : integer := C_TEXT_COLS * C_TEXT_ROWS;  -- 32
    constant C_COL_BIT      : integer := 4;              -- bits to address cols
    constant C_LINE_BIT     : integer := 1;              -- bits to address rows
    constant C_CURSOR_BIT   : integer := C_COL_BIT + C_LINE_BIT;     -- 5

    -- Bresenham position-accumulator format:
    --   bits [C_POS_BITS-1 : C_POS_FRAC]  : 8-bit integer glyph-pixel index
    --   bits [C_POS_FRAC-1 : 0]           : 16-bit fractional remainder
    -- A "past text" sentinel is encoded by forcing the accumulator to all-1s,
    -- which sets bit [C_POS_BITS-1] (the integer-part MSB) and lets us test
    -- "outside the text band" with a single-bit compare.
    constant C_SCALE_MAX    : integer := 32;
    constant C_POS_FRAC     : integer := 16;
    constant C_POS_INT      : integer := 8;
    constant C_POS_BITS     : integer := C_POS_INT + C_POS_FRAC;     -- 24
    constant C_STEP_BITS    : integer := C_POS_FRAC + 1;             -- 17

    -- Style codes (reg(2) top 3 bits).
    constant C_ST_PLAIN     : integer := 0;   -- ink only
    constant C_ST_SHADOW    : integer := 1;   -- ink + black drop shadow
    constant C_ST_EDGE      : integer := 2;   -- ink + black outline
    constant C_ST_HOLLOW    : integer := 3;   -- outline only, interior open
    constant C_ST_EMBOSS    : integer := 4;   -- white NW highlight, black SE
    constant C_ST_BOX       : integer := 5;   -- ink on a solid black panel
    constant C_ST_HALFBOX   : integer := 6;   -- ink on 50%-dimmed video
    constant C_ST_REVERSE   : integer := 7;   -- black text knocked out of ink

    -- Pixel classes handed from S6 to the S7 output mux.
    constant C_CL_BG        : integer := 0;   -- background (video or black)
    constant C_CL_INK       : integer := 1;   -- selected text/cursor colour
    constant C_CL_BLACK     : integer := 2;
    constant C_CL_WHITE     : integer := 3;
    constant C_CL_DIM       : integer := 4;   -- video at 50%
    constant C_CL_VIDEO     : integer := 5;   -- matte: raw video through glyph

    -- Drawn luma is capped at 768: full-scale Y clips every RGB channel on
    -- the encoder and kills chroma.
    constant C_Y_WHITE      : integer := 768;
    constant C_Y_BLACK      : integer := 64;
    constant C_MID          : integer := 512;

    type t_step_lut is array (1 to C_SCALE_MAX)
        of unsigned(C_STEP_BITS - 1 downto 0);

    function f_make_step_lut return t_step_lut is
        variable v : t_step_lut;
    begin
        for i in 1 to C_SCALE_MAX loop
            v(i) := to_unsigned(2 ** C_POS_FRAC / i, C_STEP_BITS);
        end loop;
        return v;
    end function;

    constant C_STEP_LUT : t_step_lut := f_make_step_lut;

    -- Extract one 8-bit glyph row out of the packed 64-bit word.  Written as
    -- an explicit case rather than a computed slice: non-static slice bounds
    -- are a synthesis hazard, and the case maps to a plain 8:1 mux anyway.
    function f_row(g : std_logic_vector(63 downto 0);
                   y : unsigned(2 downto 0)) return std_logic_vector is
    begin
        case to_integer(y) is
            when 0      => return g(63 downto 56);
            when 1      => return g(55 downto 48);
            when 2      => return g(47 downto 40);
            when 3      => return g(39 downto 32);
            when 4      => return g(31 downto 24);
            when 5      => return g(23 downto 16);
            when 6      => return g(15 downto  8);
            when others => return g( 7 downto  0);
        end case;
    end function;

    --==========================================================================
    -- Decoded parameters (concurrent)
    --==========================================================================
    signal s_h_pos       : unsigned(9 downto 0);
    signal s_v_pos       : unsigned(9 downto 0);
    signal s_crawl_raw   : unsigned(9 downto 0);
    signal s_style       : unsigned(2 downto 0) := "000";
    signal s_letter      : unsigned(5 downto 0);
    signal s_edit_mode   : std_logic;
    signal s_bg_black    : std_logic;
    signal s_matte       : std_logic;
    signal s_font_sel    : std_logic;
    signal s_justify     : std_logic;

    -- Colour is latched: only updates when not in edit mode (knob 4 is then
    -- repurposed as the Erase-and-scroll knob).
    signal s_color_sel   : unsigned(2 downto 0) := "000";  -- 8 preset colours

    -- Dual cursor sources: knob 5 (top 5 bits of reg(4)) walks the cursor
    -- non-destructively; knob 4 (top 5 bits of reg(3)) also drags the cursor
    -- but blanks every cell it lands on.  r_cursor is the latched "active
    -- cursor" - it follows whichever source most recently moved.
    signal s_cursor_5p     : unsigned(C_CURSOR_BIT - 1 downto 0);
    signal s_erase_5p      : unsigned(C_CURSOR_BIT - 1 downto 0);
    signal r_cursor        : unsigned(C_CURSOR_BIT - 1 downto 0)
                             := (others => '0');
    signal prev_cursor_5p  : unsigned(C_CURSOR_BIT - 1 downto 0)
                             := (others => '0');
    signal prev_erase_5p   : unsigned(C_CURSOR_BIT - 1 downto 0)
                             := (others => '0');

    -- Downstream consumers see the cursor as (line, col) split.
    signal s_cursor_line : unsigned(C_LINE_BIT - 1 downto 0);
    signal s_cursor_col  : unsigned(C_COL_BIT  - 1 downto 0);

    -- Where that cell is actually DRAWN once centring has shifted the line.
    -- Without this the edit box would sit on the physical slot while the
    -- letter it belongs to had moved right, which reads as a broken cursor.
    signal s_cursor_disp : unsigned(C_COL_BIT - 1 downto 0);

    -- Edge detection for non-destructive letter writes.
    signal prev_letter   : unsigned(5 downto 0) := (others => '0');

    -- Registered write request: the text buffer is written one clock after
    -- the knob edge, from the already-registered r_cursor.  Keeps the edge
    -- compare, the cursor mux and the 32-way cell decode out of one cone.
    signal wr_letter     : std_logic := '0';
    signal wr_erase      : std_logic := '0';
    signal wr_code       : unsigned(5 downto 0) := (others => '0');

    -- Size shares K5 with the cursor.  r_size is the latched size; it only
    -- follows K5 while size_follow is set, which Edit Mode clears and a real
    -- K5 move (outside Edit Mode) sets again - so leaving Edit Mode keeps the
    -- size you had instead of jumping to wherever the cursor was left.
    signal r_size        : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal size_follow   : std_logic := '1';

    --==========================================================================
    -- Raster measurement.  h_act / v_act count ACTIVE pixels and ACTIVE lines
    -- only, so every geometry term below is expressed in active-picture
    -- coordinates and the vertical DDA cannot drift between interlaced
    -- fields.  Widths default to HD until the first frame has been measured.
    --==========================================================================
    signal prev_vsync_n  : std_logic := '1';
    signal prev_avid     : std_logic := '0';
    signal h_act         : unsigned(11 downto 0) := (others => '0');
    signal v_act         : unsigned(11 downto 0) := (others => '0');
    signal meas_w        : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal meas_h        : unsigned(11 downto 0) := to_unsigned(1080, 12);

    -- Serration guard: analog vsync emits several falling edges per field.
    -- Anything that toggles or increments must fire exactly once, gated on
    -- having seen active video since the last time it fired.
    signal saw_active    : std_logic := '0';
    signal frame_counter : unsigned(7 downto 0) := (others => '0');

    --==========================================================================
    -- Per-frame geometry, produced by the vblank sequencer below.
    --==========================================================================
    signal s_size_step   : unsigned(C_STEP_BITS - 1 downto 0)
                           := C_STEP_LUT(8);
    signal s_h_origin    : unsigned(11 downto 0) := (others => '1');
    signal s_v_origin    : unsigned(11 downto 0) := (others => '1');
    signal hpos_init     : unsigned(C_POS_BITS - 1 downto 0) := (others => '1');
    signal vpos_init     : unsigned(C_POS_BITS - 1 downto 0) := (others => '1');

    -- Resolution scaling: per-mode constants from the timing ID (registered,
    -- free-running), the per-axis accumulator steps, and the virtual raster.
    --   s_tx     x step class: 0 = 1.0 (1080), 1 = 1.5 (720p), 2 = 2.0 (SD)
    --   s_ys2    y step per FIELD line / 2, Q.7 (= Q.8 y step / 2)
    --   s_inv_ys field lines per virtual unit, Q12
    --   s_inv_xs pixels per virtual unit, Q12
    signal s_tx          : unsigned(1 downto 0) := "00";
    signal s_t1080p      : std_logic := '1';
    signal s_ys2         : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal s_inv_ys      : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_inv_xs      : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_hstep       : unsigned(C_STEP_BITS - 1 downto 0) := C_STEP_LUT(8);
    -- (no bottom-field offset: core_top ties field_n high on every input,
    -- so both interlaced fields render alike -- SD comes out line-doubled)
    signal s_vstep       : unsigned(C_STEP_BITS - 1 downto 0) := C_STEP_LUT(8);
    signal sq_wv         : unsigned(11 downto 0) := to_unsigned(1920, 12);  -- virtual width
    signal sq_hv         : unsigned(11 downto 0) := to_unsigned(1080, 12);  -- virtual height
    signal sq_lv         : unsigned(11 downto 0) := (others => '0');  -- left edge (>= 0), virtual
    signal sq_hpos       : std_logic := '0';
    signal sq_ov         : unsigned(11 downto 0) := (others => '0');  -- top edge (>= 0), virtual

    -- Centring offsets, one per line (0 when Justify = Left).
    signal s_off0        : unsigned(3 downto 0) := (others => '0');
    signal s_off1        : unsigned(3 downto 0) := (others => '0');
    signal s_used0       : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_used1       : unsigned(4 downto 0) := to_unsigned(16, 5);

    -- Does each line carry any text at all?  The panel styles use these to
    -- size the mask: a blank second line must not drag an empty line's worth
    -- of panel down under the words.  Defaults match the power-on buffer
    -- (line 0 holds the greeting, line 1 is blank).
    signal s_len0_nz     : std_logic := '1';
    signal s_len1_nz     : std_logic := '0';

    --==========================================================================
    -- Vblank sequencer.  Everything here is quasi-static (once per field) but
    -- is still fully STA-timed at the pixel clock, so each state does exactly
    -- one small operation and all five products run on ONE shared serial
    -- shift-add multiplier.  Worst-case run length is ~140 cycles, against
    -- ~38 000 clocks of SD vblank.
    --==========================================================================
    type t_sm_state is (SM_IDLE, SM_RANGE, SM_MUL_S, SM_SCALE,
                        SM_MUL_CX, SM_CX, SM_MUL_CY, SM_CY,
                        SM_MUL_V, SM_V, SM_PERIOD, SM_CRAWL, SM_WRAP,
                        SM_LEFT, SM_LEFT2, SM_LWRAP, SM_ORIGIN,
                        SM_MUL_H, SM_H, SM_MUL_LX, SM_LX, SM_MUL_OY, SM_OY,
                        SM_MUL_VS, SM_VS, SM_V_DIV, SM_SCAN, SM_JUST);
    signal sm_state    : t_sm_state := SM_IDLE;

    -- shared serial multiplier: acc = a * b, one partial product per cycle
    -- (10 iterations for the knob products, 13 for the left-edge preload)
    signal mul_a       : unsigned(23 downto 0) := (others => '0');
    signal mul_b       : unsigned(12 downto 0) := (others => '0');
    signal mul_acc     : unsigned(23 downto 0) := (others => '0');
    signal mul_cnt     : unsigned(3 downto 0)  := (others => '0');

    signal sq_smin     : unsigned(5 downto 0) := to_unsigned(1, 6);
    signal sq_srange   : unsigned(5 downto 0) := to_unsigned(7, 6);
    signal sq_scale    : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal sq_cx       : unsigned(11 downto 0) := (others => '0');
    signal sq_cy       : unsigned(11 downto 0) := (others => '0');
    signal sq_residue  : unsigned(12 downto 0) := (others => '0');
    signal sq_quotient : unsigned(7 downto 0)  := (others => '0');
    signal sq_hneg     : std_logic := '0';
    signal sq_vneg     : std_logic := '0';

    -- Crawl.  crawl_q is the text's offset from its resting position in
    -- 1/16 px (14 integer + 4 fractional bits), kept in [0, P) where
    -- P = meas_w + 2*half_w is the wrap period: the text leaves one side
    -- completely before it re-enters the other.  At hero size half_w reaches
    -- 32*64 = 2048, so P reaches ~6016 px - hence the wide terms here.
    signal crawl_q     : unsigned(17 downto 0) := (others => '0');
    signal sq_ctmp     : signed(18 downto 0)   := (others => '0');
    signal sq_vel      : unsigned(7 downto 0)  := (others => '0');
    signal sq_left     : std_logic := '0';      -- '1' = crawl leftward
    signal sq_period   : unsigned(13 downto 0) := to_unsigned(1920, 14);
    signal sq_l        : signed(14 downto 0)   := (others => '0');

    -- Justify scan state: walks all 32 cells recording the last non-space.
    signal sq_scan_i   : unsigned(5 downto 0) := (others => '0');
    signal sq_last0    : unsigned(4 downto 0) := (others => '0');
    signal sq_last1    : unsigned(4 downto 0) := (others => '0');

    --==========================================================================
    -- Position counters + accumulators (S1)
    --==========================================================================
    signal hpos_acc     : unsigned(C_POS_BITS - 1 downto 0) := (others => '1');
    signal vpos_acc     : unsigned(C_POS_BITS - 1 downto 0) := (others => '1');

    --==========================================================================
    -- Text buffer (32 chars x 6 bits = 2 lines x 16 cols)
    -- Indexing: text_buf(line * 16 + col).
    --==========================================================================
    type t_text_buf is array (0 to C_TEXT_LEN - 1) of unsigned(5 downto 0);
    signal text_buf : t_text_buf := (
        -- Line 0: "LZX VIDEOMANCER!"
        to_unsigned(12, 6),  -- L
        to_unsigned(26, 6),  -- Z
        to_unsigned(24, 6),  -- X
        to_unsigned( 0, 6),  -- (space)
        to_unsigned(22, 6),  -- V
        to_unsigned( 9, 6),  -- I
        to_unsigned( 4, 6),  -- D
        to_unsigned( 5, 6),  -- E
        to_unsigned(15, 6),  -- O
        to_unsigned(13, 6),  -- M
        to_unsigned( 1, 6),  -- A
        to_unsigned(14, 6),  -- N
        to_unsigned( 3, 6),  -- C
        to_unsigned( 5, 6),  -- E
        to_unsigned(18, 6),  -- R
        to_unsigned(39, 6),  -- !
        -- Line 1: blank
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6)
    );

    --==========================================================================
    -- Pipeline stage signals
    --==========================================================================
    -- Stage 2: glyph indices, centring applied, band flags.
    signal s2_char_eff     : unsigned(C_COL_BIT  - 1 downto 0);
    signal s2_char_raw     : unsigned(C_COL_BIT  - 1 downto 0);
    signal s2_blank        : std_logic;
    signal s2_line_idx     : unsigned(C_LINE_BIT - 1 downto 0);
    signal s2_glyph_x      : unsigned(2 downto 0);
    signal s2_glyph_y      : unsigned(2 downto 0);
    signal s2_in_text      : std_logic;
    signal s2_in_box       : std_logic;
    signal s2_top: std_logic;

    -- Stage 3: char code + cursor box + edit-mode underscore guide
    signal s3_char_code    : unsigned(5 downto 0);
    signal s3_glyph_x      : unsigned(2 downto 0);
    signal s3_glyph_y      : unsigned(2 downto 0);
    signal s3_in_text      : std_logic;
    signal s3_in_box       : std_logic;
    signal s3_top: std_logic;
    signal s3_cursor_box   : std_logic;
    signal s3_underscore   : std_logic;

    -- Stage 4: whole-glyph ROM word + pass-through control bits.
    signal s4_glyph        : std_logic_vector(63 downto 0);
    signal s4_glyph_x      : unsigned(2 downto 0);
    signal s4_glyph_y      : unsigned(2 downto 0);
    signal s4_in_text      : std_logic;
    signal s4_in_box       : std_logic;
    signal s4_top: std_logic;
    signal s4_cursor_box   : std_logic;
    signal s4_underscore   : std_logic;

    -- Stage 5: the three glyph rows we need, plus the palette index.
    signal s5_row_m1       : std_logic_vector(7 downto 0);
    signal s5_row_0        : std_logic_vector(7 downto 0);
    signal s5_row_p1       : std_logic_vector(7 downto 0);
    signal s5_glyph_x      : unsigned(2 downto 0);
    signal s5_in_text      : std_logic;
    signal s5_in_box       : std_logic;
    signal s5_top: std_logic;
    signal s5_cursor_box   : std_logic;
    signal s5_underscore   : std_logic;
    signal s5_pal_sel      : unsigned(2 downto 0);

    -- Stage 6: neighbourhood bits + resolved ink colour + dimmed video.
    signal s6_class        : unsigned(2 downto 0);
    signal s6_ink_y        : unsigned(9 downto 0);
    signal s6_ink_u        : unsigned(9 downto 0);
    signal s6_ink_v        : unsigned(9 downto 0);
    signal s6_dim_y        : unsigned(9 downto 0);
    signal s6_dim_u        : unsigned(9 downto 0);
    signal s6_dim_v        : unsigned(9 downto 0);

    --==========================================================================
    -- Bypass delay line (matches pipeline latency)
    --==========================================================================
    type t_pix_delay is array (0 to C_DELAY_DEPTH - 1)
        of std_logic_vector(9 downto 0);
    type t_sync_delay is array (0 to C_DELAY_DEPTH - 1) of std_logic;

    signal bypass_y     : t_pix_delay;
    signal bypass_u     : t_pix_delay;
    signal bypass_v     : t_pix_delay;
    signal bypass_hs    : t_sync_delay := (others => '1');
    signal bypass_vs    : t_sync_delay := (others => '1');
    signal bypass_field : t_sync_delay := (others => '1');
    signal bypass_avid  : t_sync_delay := (others => '0');

    --==========================================================================
    -- Ink palette (combinational).  Standard BT.601 limited range, derived
    -- from sRGB primaries and scaled to 10 bits; the U/V swap that Videomancer
    -- hardware wants happens once, at the S7 compose.  White and yellow are
    -- pulled down to the 768 luma cap.
    --==========================================================================
    signal pal_y : unsigned(9 downto 0);
    signal pal_u : unsigned(9 downto 0);
    signal pal_v : unsigned(9 downto 0);

begin

    --==========================================================================
    -- Register decoding
    --==========================================================================
    s_h_pos       <= unsigned(registers_in(0));
    s_v_pos       <= unsigned(registers_in(1));
    s_cursor_5p   <= unsigned(registers_in(4)(9 downto 5));
    s_erase_5p    <= unsigned(registers_in(3)(9 downto 5));
    s_cursor_line <= r_cursor(C_CURSOR_BIT - 1 downto C_COL_BIT);
    s_cursor_col  <= r_cursor(C_COL_BIT - 1 downto 0);
    s_letter      <= unsigned(registers_in(5)(9 downto 4));  -- top 6 bits
    s_edit_mode   <= registers_in(6)(0);
    s_bg_black    <= registers_in(6)(1);
    s_matte       <= registers_in(6)(2);
    s_font_sel    <= registers_in(6)(3);
    s_justify     <= registers_in(6)(4);
    s_crawl_raw   <= unsigned(registers_in(7));   -- P12 slider

    -- Colour is latched while in edit mode (knob 4 then becomes the eraser).
    -- The drawn cursor column is registered here too: it is quasi-static, and
    -- keeping the add out of S3 leaves the cell compare a plain equality.
    p_color_latch : process(clk)
        variable v_disp : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- labelled defaults: the firmware loads the label INDEX (0..7),
            -- so a value below 8 is the label, not the bottom zone
            if unsigned(registers_in(2)(9 downto 3)) = 0 then
                s_style <= unsigned(registers_in(2)(2 downto 0));
            else
                s_style <= unsigned(registers_in(2)(9 downto 7));
            end if;
            if s_edit_mode = '0' then
                if unsigned(registers_in(3)(9 downto 3)) = 0 then
                    s_color_sel <= unsigned(registers_in(3)(2 downto 0));
                else
                    s_color_sel <= unsigned(registers_in(3)(9 downto 7));
                end if;
            end if;

            -- Size pickup: Edit Mode drops the link to K5; outside Edit Mode
            -- a real K5 move (its top 5 bits change - the same edge that moves
            -- the cursor) picks it up again.  Power-on starts linked, so the
            -- knob and presets work straight away.
            if s_edit_mode = '1' then
                size_follow <= '0';
            elsif s_cursor_5p /= prev_cursor_5p then
                size_follow <= '1';
            end if;
            if s_edit_mode = '0' and size_follow = '1' then
                r_size <= unsigned(registers_in(4));
            end if;

            if s_cursor_line(0) = '0' then
                v_disp := ('0' & s_cursor_col) + ('0' & s_off0);
            else
                v_disp := ('0' & s_cursor_col) + ('0' & s_off1);
            end if;
            if v_disp(4) = '1' then      -- shoved past the last cell: pin it
                s_cursor_disp <= (others => '1');
            else
                s_cursor_disp <= v_disp(3 downto 0);
            end if;
        end if;
    end process p_color_latch;

    --==========================================================================
    -- 8-entry ink palette, addressed by s5_pal_sel (text colour normally, the
    -- cycling cursor hue while the edit box is being drawn).
    --==========================================================================
    with s5_pal_sel select
        pal_y <= to_unsigned(768, 10) when "000",  -- White  (luma-capped)
                 to_unsigned(326, 10) when "001",  -- Red
                 to_unsigned(584, 10) when "010",  -- Orange
                 to_unsigned(772, 10) when "011",  -- Yellow (luma-capped)
                 to_unsigned(578, 10) when "100",  -- Green
                 to_unsigned(678, 10) when "101",  -- Cyan
                 to_unsigned(164, 10) when "110",  -- Blue
                 to_unsigned(426, 10) when "111",  -- Magenta
                 to_unsigned(768, 10) when others;
    with s5_pal_sel select
        pal_u <= to_unsigned(512, 10) when "000",
                 to_unsigned(361, 10) when "001",
                 to_unsigned(212, 10) when "010",
                 to_unsigned(103, 10) when "011",
                 to_unsigned(215, 10) when "100",
                 to_unsigned(663, 10) when "101",
                 to_unsigned(960, 10) when "110",
                 to_unsigned(809, 10) when "111",
                 to_unsigned(512, 10) when others;
    with s5_pal_sel select
        pal_v <= to_unsigned(512, 10) when "000",
                 to_unsigned(960, 10) when "001",
                 to_unsigned(772, 10) when "010",
                 to_unsigned(578, 10) when "011",
                 to_unsigned(137, 10) when "100",
                 to_unsigned( 64, 10) when "101",
                 to_unsigned(440, 10) when "110",
                 to_unsigned(888, 10) when "111",
                 to_unsigned(512, 10) when others;

    --==========================================================================
    -- S1: raster measurement + Bresenham accumulators.
    --
    -- h_act counts active pixels from the start of active video; v_act counts
    -- ACTIVE lines from the top of the field.  Freezing both outside avid
    -- means the text band can only ever fall inside the picture, and gives
    -- the vertical DDA the "advance on active lines only" behaviour that
    -- keeps interlaced fields in step.
    --
    -- Every once-per-field action (frame counter, height capture, sequencer
    -- start) is gated on saw_active so that serrated analog vsync - which
    -- produces several falling edges per field - fires it exactly once.
    --==========================================================================
    p_pipeline_s1 : process(clk)
        variable v_vs_falling : std_logic;
        variable v_avid_start : std_logic;
        variable v_avid_end   : std_logic;
        variable v_h_next     : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_vs_falling := prev_vsync_n and (not data_in.vsync_n);
            v_avid_start := data_in.avid and (not prev_avid);
            v_avid_end   := prev_avid and (not data_in.avid);

            -- Horizontal active counter, and the width measurement taken at
            -- the end of each active line.
            if v_avid_start = '1' then
                h_act    <= (others => '0');
                v_h_next := (others => '0');
            elsif data_in.avid = '1' then
                h_act    <= h_act + 1;
                v_h_next := h_act + 1;
            else
                v_h_next := h_act;
            end if;

            if v_avid_end = '1' and h_act > 0 then
                meas_w <= h_act + 1;   -- h_act ends on the LAST active index
            end if;

            -- Vertical active-line counter.  v_act only advances on active
            -- lines, so it holds its final value all through vblank and can
            -- safely be sampled at vsync (the classic trap is a counter that
            -- zeroes on blanking hsyncs first and reads back 0).
            if v_vs_falling = '1' then
                v_act <= (others => '0');
            elsif v_avid_start = '1' then
                v_act <= v_act + 1;
            end if;

            -- Once-per-field, serration-guarded.
            if data_in.avid = '1' then
                saw_active <= '1';
            elsif v_vs_falling = '1' and saw_active = '1' then
                saw_active    <= '0';
                frame_counter <= frame_counter + 1;
                if v_act > 0 then
                    meas_h <= v_act;
                end if;
            end if;

            -- Horizontal accumulator: reloaded at the start of every active
            -- line, reset to 0 when the scan reaches the text origin, then
            -- stepped once per active pixel until the integer part leaves the
            -- 128-glyph-pixel band.  Frozen during blanking.
            if data_in.avid = '1' then
                if v_avid_start = '1' then
                    -- Origin 0 means the text starts on the very first active
                    -- pixel, which the "next pixel equals origin" test below
                    -- can never see; catch it here.
                    if s_h_origin = 0 then
                        hpos_acc <= (others => '0');
                    else
                        hpos_acc <= hpos_init;
                    end if;
                elsif v_h_next = s_h_origin then
                    hpos_acc <= (others => '0');
                elsif hpos_acc(C_POS_BITS - 1) = '0' then
                    hpos_acc <= hpos_acc + resize(s_hstep, C_POS_BITS);
                end if;
            end if;

            -- Vertical accumulator: reloaded at vsync (idempotent, so
            -- serration is harmless) and stepped once per ACTIVE line.
            -- v_act is compared BEFORE its increment, so it carries the
            -- index of the line that is just starting (0 for the first).
            if v_vs_falling = '1' then
                vpos_acc <= vpos_init;
            elsif v_avid_start = '1' then
                if v_act = s_v_origin then
                    vpos_acc <= (others => '0');
                elsif vpos_acc(C_POS_BITS - 1) = '0' then
                    vpos_acc <= vpos_acc + resize(s_vstep, C_POS_BITS);
                end if;
            end if;
        end if;
    end process p_pipeline_s1;

    --==========================================================================
    -- Timing-ID class (free-running, registered: kept out of the sequencer's
    -- cones).  Virtual raster 1920x1080 (HD) / 1440x1080 (SD, a 4:3 window).
    --==========================================================================
    p_tclass : process(clk)
    begin
        if rising_edge(clk) then
            s_t1080p <= '0';
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 =>            -- 480i: 2 x 1080/486 per field line
                    s_tx <= "10";
                    s_ys2 <= to_unsigned(569, 10);  s_inv_ys <= to_unsigned(921, 13);
                when 4 =>            -- 480p
                    s_tx <= "10";
                    s_ys2 <= to_unsigned(288, 10);  s_inv_ys <= to_unsigned(1820, 13);
                when 8 =>            -- 576i
                    s_tx <= "10";
                    s_ys2 <= to_unsigned(480, 10);  s_inv_ys <= to_unsigned(1092, 13);
                when 12 =>           -- 576p
                    s_tx <= "10";
                    s_ys2 <= to_unsigned(240, 10);  s_inv_ys <= to_unsigned(2185, 13);
                when 5 | 6 | 14 =>   -- 720p: 1.5
                    s_tx <= "01";
                    s_ys2 <= to_unsigned(192, 10);  s_inv_ys <= to_unsigned(2731, 13);
                when 1 | 2 | 10 =>   -- 1080i: 2.0 per field line
                    s_tx <= "00";
                    s_ys2 <= to_unsigned(256, 10);  s_inv_ys <= to_unsigned(2048, 13);
                when others =>       -- 1080p: 1.0
                    s_tx <= "00"; s_t1080p <= '1';
                    s_ys2 <= to_unsigned(128, 10);  s_inv_ys <= to_unsigned(4096, 13);
            end case;
            case s_tx is
                when "00"   => s_inv_xs <= to_unsigned(4096, 13);
                when "01"   => s_inv_xs <= to_unsigned(2731, 13);
                when others => s_inv_xs <= to_unsigned(2048, 13);
            end case;
        end if;
    end process p_tclass;

    --==========================================================================
    -- Vblank sequencer: derives every per-frame geometry term from the
    -- measured raster and the knobs.
    --
    -- All in VIRTUAL units (v2.2.3): W = meas_w * x-step (1920 HD, 1440 SD),
    -- H = 1080 (meas_h at 1080p).  The scale range is the 1080p one in every
    -- mode (chars = 1920 >> 7 = 15), so the Size knob is height-relative.
    --   chars   = 15                   glyph scale at which one 16-char line
    --                                  exactly fills a 1920-unit width
    --   smin    = chars >> 2 = 3       Size at 0%   (small caption type)
    --   smax    = chars * 2 = 30       Size at 100% (hero type, ~8 chars
    --                                  across the screen - the climax)
    --   scale   = smin + (Size * (smax - smin)) >> 10
    --   cx      = (H Pos * W) >> 10    text centre
    --   cy      = (V Pos * H) >> 10
    -- The left/top edges are converted to real pixels / field lines at the
    -- end (x inv_xs, x inv_ys, Q12 -- exact at 1080p); the accumulators step
    -- size_step * x-step per pixel and size_step * y-step per field line.
    --
    -- Crawl (P12, bipolar):
    --   d       = max(0, |P12 - 512| - 24)     +-24 deadband holds it still
    --   v       = (d * d) >> 10                1/16 px per field, 0..231
    --                                          (~14.5 px/field flat out; the
    --                                          square gives fine control near
    --                                          the centre)
    --   P       = meas_w + 2 * half_w          wrap period
    --   crawl   = (crawl -/+ v) mod P          above centre = leftward
    --   L       = cx - half_w + int(crawl)     left edge; if L >= meas_w then
    --                                          L -= P, so L in [-2*half_w,
    --                                          meas_w): the text is fully gone
    --                                          on one side before it returns
    --                                          on the other
    --
    -- A negative left edge (text partly off the left) preloads the
    -- horizontal accumulator with |L| * step - exactly the value it would
    -- have reached walking from the true origin, so the text keeps moving a
    -- pixel at a time across the edge.  (A plain |L| / scale divide drops
    -- the remainder and would make crawling text jump a whole glyph pixel,
    -- up to 32 px, at a time.)  |L| <= 2*half_w = 128*scale, so the product
    -- never exceeds 2^23; exactly 2^23 sets the past-text bit, as it should.
    --
    -- The vertical negative-origin preload is still |origin| / scale by
    -- repeated subtraction (at most 9 iterations; nothing moves vertically).
    --
    -- SM_SCAN then walks the 32 text cells recording the last non-space in
    -- each line, which is what Justify=Centre needs.
    --==========================================================================
    p_sequencer : process(clk)
        variable v_vs_falling : std_logic;
        variable v_half_h     : unsigned(12 downto 0);
        variable v_cy         : unsigned(12 downto 0);
        variable v_dev        : unsigned(8 downto 0);
        variable v_d          : unsigned(8 downto 0);
        variable v_p16        : signed(19 downto 0);
        variable v_sub        : signed(19 downto 0);
        variable v_neg        : signed(14 downto 0);
        variable v_cell       : unsigned(5 downto 0);
        variable v_u0, v_u1   : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            v_vs_falling := prev_vsync_n and (not data_in.vsync_n);

            -- Shared serial shift-add multiplier: one partial product per
            -- cycle.  mul_a may shift past bit 23; that only ever drops bits
            -- of partial products that are never added (every product here
            -- fits 24 bits).
            if mul_cnt /= 0 then
                if mul_b(0) = '1' then
                    mul_acc <= mul_acc + mul_a;
                end if;
                mul_a   <= shift_left(mul_a, 1);
                mul_b   <= '0' & mul_b(12 downto 1);
                mul_cnt <= mul_cnt - 1;
            end if;

            case sm_state is
                when SM_IDLE =>
                    -- Serration-guarded start: saw_active is still '1' on the
                    -- first vsync edge of the field and cleared by S1, so the
                    -- sequencer launches exactly once.
                    if v_vs_falling = '1' and saw_active = '1' then
                        sm_state <= SM_RANGE;
                    end if;

                when SM_RANGE =>
                    -- Scale range: the 1080p one (chars = 15) in every mode,
                    -- so Size is relative to the screen height.  Virtual
                    -- width / height for the layout terms.
                    sq_smin   <= to_unsigned(3, 6);
                    sq_srange <= to_unsigned(27, 6);
                    case s_tx is
                        when "00"   => sq_wv <= meas_w;
                        when "01"   => sq_wv <= meas_w + shift_right(meas_w, 1);
                        when others => sq_wv <= shift_left(meas_w, 1);
                    end case;
                    if s_t1080p = '1' then
                        sq_hv <= meas_h;
                    else
                        sq_hv <= to_unsigned(1080, 12);
                    end if;
                    sm_state  <= SM_MUL_S;

                when SM_MUL_S =>
                    mul_a   <= resize(sq_srange, 24);
                    mul_b   <= resize(r_size, 13);
                    mul_acc <= (others => '0');
                    mul_cnt <= to_unsigned(10, 4);
                    sm_state <= SM_SCALE;

                when SM_SCALE =>
                    if mul_cnt = 0 then
                        sq_scale <= sq_smin
                                    + resize(shift_right(mul_acc, 10), 6);
                        sm_state <= SM_MUL_CX;
                    end if;

                when SM_MUL_CX =>
                    -- sq_scale is valid now; latch the step and the extents.
                    s_size_step <= C_STEP_LUT(to_integer(sq_scale));
                    mul_a    <= resize(sq_wv, 24);
                    mul_b    <= resize(s_h_pos, 13);
                    mul_acc  <= (others => '0');
                    mul_cnt  <= to_unsigned(10, 4);
                    sm_state <= SM_CX;

                when SM_CX =>
                    -- x accumulator step = size_step * x-step
                    case s_tx is
                        when "00"   => s_hstep <= s_size_step;
                        when "01"   => s_hstep <= s_size_step + shift_right(s_size_step, 1);
                        when others => s_hstep <= shift_left(s_size_step, 1);
                    end case;
                    if mul_cnt = 0 then
                        sq_cx    <= resize(shift_right(mul_acc, 10), 12);
                        sm_state <= SM_MUL_CY;
                    end if;

                when SM_MUL_CY =>
                    mul_a    <= resize(sq_hv, 24);
                    mul_b    <= resize(s_v_pos, 13);
                    mul_acc  <= (others => '0');
                    mul_cnt  <= to_unsigned(10, 4);
                    sm_state <= SM_CY;

                when SM_CY =>
                    if mul_cnt = 0 then
                        sq_cy    <= resize(shift_right(mul_acc, 10), 12);
                        sm_state <= SM_MUL_V;
                    end if;

                when SM_MUL_V =>
                    -- |P12 - 512| by ones' complement: bit 9 says which half,
                    -- the low 9 bits (inverted below centre) are the distance.
                    if s_crawl_raw(9) = '1' then
                        v_dev := s_crawl_raw(8 downto 0);
                    else
                        v_dev := not s_crawl_raw(8 downto 0);
                    end if;
                    if v_dev > to_unsigned(24, 9) then
                        v_d := v_dev - to_unsigned(24, 9);
                    else
                        v_d := (others => '0');
                    end if;
                    sq_left  <= s_crawl_raw(9);
                    mul_a    <= resize(v_d, 24);
                    mul_b    <= resize(v_d, 13);
                    mul_acc  <= (others => '0');
                    mul_cnt  <= to_unsigned(10, 4);
                    sm_state <= SM_V;

                when SM_V =>
                    if mul_cnt = 0 then
                        sq_vel   <= mul_acc(17 downto 10);   -- <= 487^2 >> 10
                        sm_state <= SM_PERIOD;
                    end if;

                when SM_PERIOD =>
                    -- P = W + 2*half_w = W + scale*128.
                    sq_period <= resize(sq_wv, 14)
                                 + shift_left(resize(sq_scale, 14), 7);
                    sm_state  <= SM_CRAWL;

                when SM_CRAWL =>
                    if sq_left = '1' then
                        sq_ctmp <= signed(resize(crawl_q, 19))
                                   - signed(resize(sq_vel, 19));
                    else
                        sq_ctmp <= signed(resize(crawl_q, 19))
                                   + signed(resize(sq_vel, 19));
                    end if;
                    sm_state <= SM_WRAP;

                when SM_WRAP =>
                    -- Back into [0, P).  One correction per field: if Size
                    -- shrank P a lot the offset converges over a few fields,
                    -- and SM_ORIGIN hides the text meanwhile if it lands
                    -- off-screen.
                    v_p16 := signed(resize(shift_left(resize(sq_period, 18), 4),
                                           20));
                    v_sub := resize(sq_ctmp, 20) - v_p16;
                    if sq_ctmp(18) = '1' then
                        crawl_q <= unsigned(resize(resize(sq_ctmp, 20) + v_p16,
                                                   18));
                    elsif v_sub(19) = '0' then
                        crawl_q <= unsigned(v_sub(17 downto 0));
                    else
                        crawl_q <= unsigned(sq_ctmp(17 downto 0));
                    end if;
                    sm_state <= SM_LEFT;

                when SM_LEFT =>
                    -- cx - half_w, half_w = scale*64.  Resize BEFORE shifting.
                    sq_l <= signed(resize(sq_cx, 15))
                            - signed(shift_left(resize(sq_scale, 15), 6));
                    sm_state <= SM_LEFT2;

                when SM_LEFT2 =>
                    sq_l <= sq_l + signed(resize(crawl_q(17 downto 4), 15));
                    sm_state <= SM_LWRAP;

                when SM_LWRAP =>
                    if sq_l >= signed(resize(sq_wv, 15)) then
                        sq_l <= sq_l - signed(resize(sq_period, 15));
                    end if;
                    sm_state <= SM_ORIGIN;

                when SM_ORIGIN =>
                    -- Horizontal: the left edge from the crawl.  L >= meas_w
                    -- can only happen while crawl_q is still converging after
                    -- a Size change - hide the text rather than wrap it.
                    -- (virtual units; a visible left edge is converted to
                    -- pixels in SM_LX)
                    sq_hpos <= '0';
                    if sq_l >= signed(resize(sq_wv, 15)) then
                        s_h_origin <= (others => '1');   -- unreachable by h_act
                        hpos_init  <= (others => '1');   -- past-text sentinel
                        sq_hneg    <= '0';
                    elsif sq_l(14) = '0' then
                        sq_lv      <= unsigned(sq_l(11 downto 0));
                        sq_hpos    <= '1';
                        hpos_init  <= (others => '1');
                        sq_hneg    <= '0';
                    else
                        s_h_origin <= (others => '1');
                        sq_hneg    <= '1';
                    end if;

                    -- Vertical: 18 glyph rows tall (two 8-row lines plus a
                    -- top-margin row each), so half_h = scale*9.
                    v_half_h := shift_left(resize(sq_scale, 13), 3)
                                + resize(sq_scale, 13);
                    v_cy     := resize(sq_cy, 13);
                    if v_cy >= v_half_h then
                        sq_ov      <= resize(v_cy - v_half_h, 12);  -- -> SM_OY
                        sq_vneg    <= '0';
                        sq_residue <= (others => '0');   -- divide exits at once
                    else
                        s_v_origin <= (others => '1');
                        sq_vneg    <= '1';
                        sq_residue <= v_half_h - v_cy;
                    end if;
                    sq_quotient <= (others => '0');
                    sm_state    <= SM_MUL_H;

                when SM_MUL_H =>
                    -- (|L| virtual units * size_step = |L| px * x step)
                    if sq_hneg = '1' then
                        v_neg    := -sq_l;               -- 1 .. 2*half_w
                        mul_a    <= resize(s_size_step, 24);
                        mul_b    <= unsigned(v_neg(12 downto 0));
                        mul_acc  <= (others => '0');
                        mul_cnt  <= to_unsigned(13, 4);
                        sm_state <= SM_H;
                    else
                        sm_state <= SM_MUL_LX;
                    end if;

                when SM_H =>
                    if mul_cnt = 0 then
                        hpos_init <= mul_acc;
                        sm_state  <= SM_MUL_LX;
                    end if;

                when SM_MUL_LX =>
                    -- visible left edge: virtual -> pixels (x inv_xs, Q12)
                    if sq_hpos = '1' then
                        mul_a    <= resize(sq_lv, 24);
                        mul_b    <= s_inv_xs;
                        mul_acc  <= (others => '0');
                        mul_cnt  <= to_unsigned(13, 4);
                        sm_state <= SM_LX;
                    else
                        sm_state <= SM_MUL_OY;
                    end if;

                when SM_LX =>
                    if mul_cnt = 0 then
                        s_h_origin <= mul_acc(23 downto 12);
                        sm_state   <= SM_MUL_OY;
                    end if;

                when SM_MUL_OY =>
                    -- visible top edge: virtual -> field lines (x inv_ys, Q12)
                    if sq_vneg = '0' then
                        mul_a    <= resize(sq_ov, 24);
                        mul_b    <= s_inv_ys;
                        mul_acc  <= (others => '0');
                        mul_cnt  <= to_unsigned(13, 4);
                        sm_state <= SM_OY;
                    else
                        sm_state <= SM_MUL_VS;
                    end if;

                when SM_OY =>
                    if mul_cnt = 0 then
                        s_v_origin <= mul_acc(23 downto 12);
                        sm_state   <= SM_MUL_VS;
                    end if;

                when SM_MUL_VS =>
                    -- y accumulator step = size_step * y-step (Q.7 half step)
                    mul_a    <= resize(s_size_step, 24);
                    mul_b    <= resize(s_ys2, 13);
                    mul_acc  <= (others => '0');
                    mul_cnt  <= to_unsigned(10, 4);
                    sm_state <= SM_VS;

                when SM_VS =>
                    if mul_cnt = 0 then
                        s_vstep  <= mul_acc(C_STEP_BITS + 6 downto 7);
                        sm_state <= SM_V_DIV;
                    end if;

                when SM_V_DIV =>
                    if sq_residue < resize(sq_scale, 13) then
                        if sq_vneg = '1' then
                            vpos_init <= shift_left(
                                resize(sq_quotient, C_POS_BITS), C_POS_FRAC);
                        else
                            vpos_init <= (others => '1');
                        end if;
                        sq_scan_i <= (others => '0');
                        sq_last0  <= (others => '1');   -- 31 = "line is empty"
                        sq_last1  <= (others => '1');
                        sm_state  <= SM_SCAN;
                    else
                        sq_residue  <= sq_residue - resize(sq_scale, 13);
                        sq_quotient <= sq_quotient + 1;
                    end if;
                when SM_SCAN =>
                    -- One cell per cycle; record the highest column index that
                    -- is not a space, per line.
                    v_cell := sq_scan_i;
                    if text_buf(to_integer(v_cell(4 downto 0))) /= 0 then
                        if v_cell(4) = '0' then
                            sq_last0 <= resize(v_cell(3 downto 0), 5);
                        else
                            sq_last1 <= resize(v_cell(3 downto 0), 5);
                        end if;
                    end if;
                    if sq_scan_i = to_unsigned(31, 6) then
                        sm_state <= SM_JUST;
                    else
                        sq_scan_i <= sq_scan_i + 1;
                    end if;

                when SM_JUST =>
                    -- used = last + 1 (0 when the line held only spaces);
                    -- offset centres those characters in the 16-cell field.
                    if sq_last0(4) = '1' then
                        v_u0 := (others => '0');
                    else
                        v_u0 := sq_last0 + 1;
                    end if;
                    if sq_last1(4) = '1' then
                        v_u1 := (others => '0');
                    else
                        v_u1 := sq_last1 + 1;
                    end if;

                    -- The true lengths are kept in ALL cases, not just when
                    -- justifying: the panel styles need them regardless, and
                    -- clipping at the true length is a no-op with Justify=Left
                    -- (the cells beyond it hold spaces anyway).
                    s_used0 <= v_u0;
                    s_used1 <= v_u1;
                    if v_u0 = 0 then s_len0_nz <= '0';
                    else             s_len0_nz <= '1'; end if;
                    if v_u1 = 0 then s_len1_nz <= '0';
                    else             s_len1_nz <= '1'; end if;

                    if s_justify = '1' then
                        s_off0 <= resize(shift_right(
                                      to_unsigned(16, 5) - v_u0, 1), 4);
                        s_off1 <= resize(shift_right(
                                      to_unsigned(16, 5) - v_u1, 1), 4);
                    else
                        s_off0 <= (others => '0');
                        s_off1 <= (others => '0');
                    end if;
                    sm_state <= SM_IDLE;
            end case;
        end if;
    end process p_sequencer;

    --==========================================================================
    -- Cursor tracking + text buffer write (Edit Mode)
    --
    -- The active cursor (r_cursor) follows whichever input source last moved:
    --   * knob 5 moves the cursor non-destructively (just position).
    --   * knob 4 moves the cursor AND blanks the cell it lands on, so a sweep
    --     of knob 4 erases a span of cells.
    -- Knob 6 writes its current code into whichever cell the cursor is on.
    -- All three only take effect in Edit Mode; the prev_* edge registers
    -- update unconditionally so entering Edit Mode never fires a spurious
    -- write.
    --
    -- The write itself lands one clock after the edge: the request is
    -- registered alongside r_cursor, and the next cycle decodes r_cursor into
    -- the cell enables.  (Doing it all in one cycle put the edge compare, the
    -- cursor mux and the 32-way decode into the enables of all 192 buffer
    -- flip-flops - the design's critical path, almost all routing.)
    --==========================================================================
    p_cursor_edit : process(clk)
        variable v_new_cursor : unsigned(C_CURSOR_BIT - 1 downto 0);
        variable v_do_erase   : std_logic;
        variable v_flat_idx   : integer range 0 to C_TEXT_LEN - 1;
    begin
        if rising_edge(clk) then
            prev_letter    <= s_letter;
            prev_cursor_5p <= s_cursor_5p;
            prev_erase_5p  <= s_erase_5p;

            v_new_cursor := r_cursor;
            v_do_erase   := '0';
            if s_cursor_5p /= prev_cursor_5p then
                v_new_cursor := s_cursor_5p;
            elsif s_erase_5p /= prev_erase_5p then
                v_new_cursor := s_erase_5p;
                v_do_erase   := '1';
            end if;
            r_cursor <= v_new_cursor;

            -- Stage 1: register the request (r_cursor above is its address).
            wr_letter <= '0';
            wr_erase  <= '0';
            wr_code   <= s_letter;
            if s_edit_mode = '1' then
                if s_letter /= prev_letter then
                    wr_letter <= '1';
                elsif v_do_erase = '1' then
                    wr_erase  <= '1';
                end if;
            end if;

            -- Stage 2: write at the registered cursor.
            v_flat_idx :=
                to_integer(r_cursor(C_CURSOR_BIT - 1 downto C_COL_BIT))
                * C_TEXT_COLS
                + to_integer(r_cursor(C_COL_BIT - 1 downto 0));
            if wr_letter = '1' then
                text_buf(v_flat_idx) <= wr_code;
            elsif wr_erase = '1' then
                text_buf(v_flat_idx) <= to_unsigned(0, 6);
            end if;
        end if;
    end process p_cursor_edit;

    --==========================================================================
    -- S2: glyph indices from the accumulator integer parts, plus the centring
    -- shift.
    --
    -- Horizontal integer part [22:16] addresses the 128 glyph-pixels across
    -- the band: bits[22:19] = char index, bits[18:16] = glyph_x, bit[23] is
    -- the past-text sentinel.
    -- Vertical integer part [23:16] indexes an 18-row block plus one row of
    -- bottom padding for the panel styles:
    --     0       line 0 top margin  (carries line 0's top outline)
    --     1..8    line 0, glyph rows 0..7   (row 7 is the blank gap)
    --     9       line 1 top margin
    --    10..17   line 1, glyph rows 0..7
    --    18       bottom padding, panel only
    --   >=19      past text
    --
    -- Justify=Centre shifts the whole line right by off(line) cells and
    -- blanks everything outside [off, off+used) - a 5-bit subtract and one
    -- compare, done here so S3 stays a plain buffer read.
    --==========================================================================
    p_stage2 : process(clk)
        variable v_h_in      : std_logic;
        variable v_v_int     : unsigned(C_POS_INT - 1 downto 0);
        variable v_in_rows   : std_logic;
        variable v_top       : std_logic;
        variable v_in_box_v  : std_logic;
        variable v_box_lo    : unsigned(C_POS_INT - 1 downto 0);
        variable v_box_hi    : unsigned(C_POS_INT - 1 downto 0);
        variable v_base      : unsigned(C_POS_INT - 1 downto 0);
        variable v_gy        : unsigned(2 downto 0);
        variable v_char_raw  : unsigned(C_COL_BIT - 1 downto 0);
        variable v_line      : std_logic;
        variable v_off       : unsigned(3 downto 0);
        variable v_used      : unsigned(4 downto 0);
        variable v_eff       : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            v_char_raw := hpos_acc(C_POS_FRAC + 6 downto C_POS_FRAC + 3);
            s2_char_raw <= v_char_raw;
            s2_glyph_x  <= hpos_acc(C_POS_FRAC + 2 downto C_POS_FRAC);

            v_v_int   := vpos_acc(C_POS_BITS - 1 downto C_POS_FRAC);
            v_top     := '0';
            v_in_rows := '0';
            v_line    := '0';
            v_base    := to_unsigned(1, C_POS_INT);
            if v_v_int <= to_unsigned(8, C_POS_INT) then
                if v_v_int = 0 then
                    v_top := '1';
                else
                    v_in_rows := '1';
                end if;
            elsif v_v_int <= to_unsigned(17, C_POS_INT) then
                v_line := '1';
                v_base := to_unsigned(10, C_POS_INT);
                if v_v_int = to_unsigned(9, C_POS_INT) then
                    v_top := '1';
                else
                    v_in_rows := '1';
                end if;
            end if;
            -- Garbage on the two margin rows; S5 discards it there.
            v_gy := resize(v_v_int - v_base, 3);

            s2_line_idx(0) <= v_line;
            s2_glyph_y     <= v_gy;

            -- Centring: shift the cell index and blank outside the used span.
            if v_line = '0' then
                v_off  := s_off0;
                v_used := s_used0;
            else
                v_off  := s_off1;
                v_used := s_used1;
            end if;
            v_eff := ('0' & v_char_raw) - ('0' & v_off);
            s2_char_eff <= v_eff(3 downto 0);
            if v_eff(4) = '1' or v_eff >= v_used then
                s2_blank <= '1';
            else
                s2_blank <= '0';
            end if;

            -- h band: bit 23 (integer MSB) clear means we're below 128.
            v_h_in := not hpos_acc(C_POS_BITS - 1);

            -- Panel band.  The mask wraps only the lines that actually
            -- carry text, so clearing line 1 shrinks the panel to a single
            -- bar instead of leaving an empty line's worth hanging below:
            --   both lines   rows 0..18
            --   line 0 only  rows 0..9
            --   line 1 only  rows 9..18
            --   neither      no panel at all
            -- Row 0 doubles as line 0's outline margin; rows 17/18 are the
            -- inter-line gap and the bottom pad, and row 9 plays both parts
            -- for a single-line panel, so the padding stays proportionate
            -- whichever line is in use.
            if s_len0_nz = '1' then
                v_box_lo := to_unsigned(0, C_POS_INT);
            else
                v_box_lo := to_unsigned(9, C_POS_INT);
            end if;
            if s_len1_nz = '1' then
                v_box_hi := to_unsigned(18, C_POS_INT);
            else
                v_box_hi := to_unsigned(9, C_POS_INT);
            end if;
            if (s_len0_nz or s_len1_nz) = '1'
               and v_v_int >= v_box_lo and v_v_int <= v_box_hi then
                v_in_box_v := '1';
            else
                v_in_box_v := '0';
            end if;

            s2_in_text <= v_h_in and (v_in_rows or v_top);
            s2_in_box  <= v_h_in and v_in_box_v;
            s2_top     <= v_h_in and v_top;
        end if;
    end process p_stage2;

    --==========================================================================
    -- S3: text buffer read + cursor box + edit underscore.
    --
    -- The cursor box is a one-glyph-pixel frame around the active cell.  All
    -- four edges land on cells/rows the font guarantees blank:
    --   left  edge = glyph_x 0   (blank gap column)
    --   right edge = glyph_x 7   (blank gap column)
    --   bottom     = glyph_y 7   (inter-line gap row)
    --   top        = the line's own top-margin row
    -- It is drawn at the cell's DISPLAYED column, so with Justify=Centre the
    -- box travels with the letters instead of staying on the physical slot.
    --==========================================================================
    p_stage3 : process(clk)
        variable v_box        : std_logic;
        variable v_underscore : std_logic;
        variable v_flat_idx   : integer range 0 to C_TEXT_LEN - 1;
    begin
        if rising_edge(clk) then
            v_flat_idx := to_integer(s2_line_idx) * C_TEXT_COLS
                          + to_integer(s2_char_eff);
            if s2_blank = '1' then
                s3_char_code <= (others => '0');
            else
                s3_char_code <= text_buf(v_flat_idx);
            end if;
            s3_glyph_x       <= s2_glyph_x;
            s3_glyph_y       <= s2_glyph_y;
            s3_in_text       <= s2_in_text;
            s3_in_box        <= s2_in_box;
            s3_top           <= s2_top;

            v_box := '0';
            if s_edit_mode = '1' and s2_in_text = '1'
               and s2_char_raw = s_cursor_disp
               and s2_line_idx(0) = s_cursor_line(0) then
                if s2_top = '1' then
                    v_box := '1';                     -- full-width top edge
                elsif s2_glyph_x = "000" or s2_glyph_x = "111"
                      or s2_glyph_y = "111" then
                    v_box := '1';                     -- sides and bottom
                end if;
            end if;

            -- Underscore guide: baseline row of every text cell, flashing at
            -- ~2 Hz, restricted to glyph_x 1..5 so the inter-character gaps
            -- stay dark and the slots read as separate.
            if s_edit_mode = '1' and s2_in_text = '1'
               and s2_top = '0' and s2_glyph_y = "111"
               and s2_glyph_x /= "000"
               and s2_glyph_x(2 downto 1) /= "11"
               and frame_counter(4) = '0' then
                v_underscore := '1';
            else
                v_underscore := '0';
            end if;
            s3_underscore <= v_underscore;
            s3_cursor_box <= v_box;
        end if;
    end process p_stage3;

    --==========================================================================
    -- S4: font ROM lookup.  One read returns the entire 8x8 glyph as a 64-bit
    -- word (2 EBR pairs).  Fonts sit back to back, so the font picker is just
    -- the top address bit.
    --==========================================================================
    p_stage4 : process(clk)
        variable v_font_addr : integer range 0 to C_FONT_CHAR_COUNT - 1;
    begin
        if rising_edge(clk) then
            if s_font_sel = '1' then
                v_font_addr := 64 + to_integer(s3_char_code);
            else
                v_font_addr := to_integer(s3_char_code);
            end if;
            s4_glyph         <= C_FONT_ROM(v_font_addr);
            s4_glyph_x       <= s3_glyph_x;
            s4_glyph_y       <= s3_glyph_y;
            s4_in_text       <= s3_in_text;
            s4_in_box        <= s3_in_box;
            s4_top           <= s3_top;
            s4_cursor_box    <= s3_cursor_box;
            s4_underscore    <= s3_underscore;
        end if;
    end process p_stage4;

    --==========================================================================
    -- S5: pick glyph rows y-1, y and y+1 out of the 64-bit word.
    --
    -- Row 7 of every glyph is blank by construction, so the y-1 wrap at y=0
    -- is harmless.  The y+1 wrap at y=7 is NOT (it would pull row 0's pixels
    -- into the inter-line gap), so it is masked explicitly.
    --
    -- On a line's top-margin row the pipeline is conceptually at glyph row -1:
    -- nothing at or above it, and glyph row 0 below.  Forcing that here is
    -- what gives the outline styles their top edge.
    --
    -- The palette index is resolved here too: the cursor box borrows the
    -- cycling hue, everything else uses the latched ink colour.
    --==========================================================================
    p_stage5 : process(clk)
    begin
        if rising_edge(clk) then
            if s4_top = '1' then
                s5_row_m1 <= (others => '0');
                s5_row_0  <= (others => '0');
                s5_row_p1 <= f_row(s4_glyph, "000");
            else
                s5_row_0  <= f_row(s4_glyph, s4_glyph_y);
                s5_row_m1 <= f_row(s4_glyph, s4_glyph_y - 1);
                if s4_glyph_y = "111" then
                    s5_row_p1 <= (others => '0');
                else
                    s5_row_p1 <= f_row(s4_glyph, s4_glyph_y + 1);
                end if;
            end if;

            s5_glyph_x       <= s4_glyph_x;
            s5_in_text       <= s4_in_text;
            s5_in_box        <= s4_in_box;
            s5_top           <= s4_top;
            s5_cursor_box    <= s4_cursor_box;
            s5_underscore    <= s4_underscore;

            if s4_cursor_box = '1' then
                s5_pal_sel <= frame_counter(5 downto 3);
            else
                s5_pal_sel <= s_color_sel;
            end if;
        end if;
    end process p_stage5;

    --==========================================================================
    -- S6: the 3x3 neighbourhood, then the pixel class.
    --
    --   self    glyph pixel is lit
    --   outl    a neighbour is lit but self is not   (black edge / hollow)
    --   shad    the pixel up-and-left is lit         (drop shadow, SE)
    --   hilo    the pixel down-and-right is lit      (emboss highlight, NW)
    --
    -- srl moves toward higher column numbers, i.e. to the RIGHT, so a shadow
    -- cast down-and-right reads the row above shifted right.
    --
    -- The dimmed-video terms for the Half Box style are built here too, from
    -- the bypass tap one stage upstream of the output.
    --==========================================================================
    p_stage6 : process(clk)
        variable v_dil    : std_logic_vector(7 downto 0);
        variable v_dil3   : std_logic_vector(7 downto 0);
        variable v_shrow  : std_logic_vector(7 downto 0);
        variable v_hirow  : std_logic_vector(7 downto 0);
        variable v_bit    : integer range 0 to 7;
        variable v_self   : std_logic;
        variable v_outl   : std_logic;
        variable v_shad   : std_logic;
        variable v_hilo   : std_logic;
        variable v_mask   : std_logic;
        variable v_class  : unsigned(2 downto 0);
        variable v_style  : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_dil   := s5_row_m1 or s5_row_0 or s5_row_p1;
            v_dil3  := v_dil or std_logic_vector(shift_left(unsigned(v_dil), 1))
                             or std_logic_vector(shift_right(unsigned(v_dil), 1));
            v_shrow := std_logic_vector(shift_right(unsigned(s5_row_m1), 1));
            v_hirow := std_logic_vector(shift_left(unsigned(s5_row_p1), 1));

            v_bit := 7 - to_integer(s5_glyph_x);

            -- The margin rows already arrive with their rows forced, so the
            -- band flag is the only gate needed here.
            v_mask := s5_in_text;
            v_self := s5_row_0(v_bit)  and v_mask;
            v_outl := v_dil3(v_bit)    and v_mask and not v_self;
            v_shad := v_shrow(v_bit)   and v_mask and not v_self;
            v_hilo := v_hirow(v_bit)   and v_mask and not v_self;

            v_style := to_integer(s_style);
            v_class := to_unsigned(C_CL_BG, 3);

            -- Panel styles paint the whole text rectangle first; glyph pixels
            -- then land on top of it.
            if s5_in_box = '1' then
                case v_style is
                    when C_ST_BOX     => v_class := to_unsigned(C_CL_BLACK, 3);
                    when C_ST_HALFBOX => v_class := to_unsigned(C_CL_DIM, 3);
                    when C_ST_REVERSE => v_class := to_unsigned(C_CL_INK, 3);
                    when others       => null;
                end case;
            end if;

            case v_style is
                when C_ST_SHADOW =>
                    if v_shad = '1' then
                        v_class := to_unsigned(C_CL_BLACK, 3);
                    end if;
                when C_ST_EDGE =>
                    if v_outl = '1' then
                        v_class := to_unsigned(C_CL_BLACK, 3);
                    end if;
                when C_ST_HOLLOW =>
                    if v_outl = '1' then
                        v_class := to_unsigned(C_CL_INK, 3);
                    end if;
                when C_ST_EMBOSS =>
                    if v_hilo = '1' then
                        v_class := to_unsigned(C_CL_WHITE, 3);
                    elsif v_shad = '1' then
                        v_class := to_unsigned(C_CL_BLACK, 3);
                    end if;
                when others => null;
            end case;

            -- The glyph body itself.  Hollow deliberately leaves it open;
            -- Reverse knocks it out of the panel in black.
            if v_self = '1' then
                if v_style = C_ST_HOLLOW then
                    null;
                elsif v_style = C_ST_REVERSE then
                    v_class := to_unsigned(C_CL_BLACK, 3);
                elsif s_matte = '1' then
                    v_class := to_unsigned(C_CL_VIDEO, 3);
                else
                    v_class := to_unsigned(C_CL_INK, 3);
                end if;
            end if;

            -- Edit-mode overlays sit on top of every style.
            if s5_underscore = '1' then
                v_class := to_unsigned(C_CL_WHITE, 3);
            end if;
            if s5_cursor_box = '1' then
                v_class := to_unsigned(C_CL_INK, 3);
            end if;

            s6_class <= v_class;
            s6_ink_y <= pal_y;
            s6_ink_u <= pal_u;
            s6_ink_v <= pal_v;

            -- 50% toward black / neutral, from the tap one stage early so
            -- it lands aligned with the S7 output register.
            s6_dim_y <= to_unsigned(32, 10)
                        + shift_right(unsigned(bypass_y(C_DELAY_DEPTH - 2)), 1);
            s6_dim_u <= to_unsigned(256, 10)
                        + shift_right(unsigned(bypass_u(C_DELAY_DEPTH - 2)), 1);
            s6_dim_v <= to_unsigned(256, 10)
                        + shift_right(unsigned(bypass_v(C_DELAY_DEPTH - 2)), 1);
        end if;
    end process p_stage6;

    --==========================================================================
    -- S7: output mux and blanking gate.
    --
    -- Generated colour is swapped U<->V exactly here (Videomancer wires Cr
    -- into the U register); passthrough and matte are left alone.
    --
    -- Outside active video every channel is forced neutral.  Without this a
    -- text block placed near the picture edge writes ink into the blanking
    -- interval and destroys the encoder's colour reference across the whole
    -- frame - a failure no simulation can see, because the image tester only
    -- scores the active region.
    --==========================================================================
    p_stage7 : process(clk)
        variable v_class : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_class := to_integer(s6_class);

            if bypass_avid(C_DELAY_DEPTH - 1) = '0' then
                s_io_align_in.y <= std_logic_vector(to_unsigned(C_Y_BLACK, 10));
                s_io_align_in.u <= std_logic_vector(to_unsigned(C_MID, 10));
                s_io_align_in.v <= std_logic_vector(to_unsigned(C_MID, 10));
            else
                case v_class is
                    when C_CL_INK =>
                        s_io_align_in.y <= std_logic_vector(s6_ink_y);
                        s_io_align_in.u <= std_logic_vector(s6_ink_v);  -- swap
                        s_io_align_in.v <= std_logic_vector(s6_ink_u);  -- swap
                    when C_CL_BLACK =>
                        s_io_align_in.y <= std_logic_vector(
                                          to_unsigned(C_Y_BLACK, 10));
                        s_io_align_in.u <= std_logic_vector(to_unsigned(C_MID, 10));
                        s_io_align_in.v <= std_logic_vector(to_unsigned(C_MID, 10));
                    when C_CL_WHITE =>
                        s_io_align_in.y <= std_logic_vector(
                                          to_unsigned(C_Y_WHITE, 10));
                        s_io_align_in.u <= std_logic_vector(to_unsigned(C_MID, 10));
                        s_io_align_in.v <= std_logic_vector(to_unsigned(C_MID, 10));
                    when C_CL_DIM =>
                        s_io_align_in.y <= std_logic_vector(s6_dim_y);
                        s_io_align_in.u <= std_logic_vector(s6_dim_u);
                        s_io_align_in.v <= std_logic_vector(s6_dim_v);
                    when C_CL_VIDEO =>
                        s_io_align_in.y <= bypass_y(C_DELAY_DEPTH - 1);
                        s_io_align_in.u <= bypass_u(C_DELAY_DEPTH - 1);
                        s_io_align_in.v <= bypass_v(C_DELAY_DEPTH - 1);
                    when others =>            -- C_CL_BG
                        if s_bg_black = '1' then
                            s_io_align_in.y <= std_logic_vector(
                                              to_unsigned(C_Y_BLACK, 10));
                            s_io_align_in.u <= std_logic_vector(
                                              to_unsigned(C_MID, 10));
                            s_io_align_in.v <= std_logic_vector(
                                              to_unsigned(C_MID, 10));
                        else
                            s_io_align_in.y <= bypass_y(C_DELAY_DEPTH - 1);
                            s_io_align_in.u <= bypass_u(C_DELAY_DEPTH - 1);
                            s_io_align_in.v <= bypass_v(C_DELAY_DEPTH - 1);
                        end if;
                end case;
            end if;

            s_io_align_in.avid    <= bypass_avid(C_DELAY_DEPTH - 1);
            s_io_align_in.hsync_n <= bypass_hs(C_DELAY_DEPTH - 1);
            s_io_align_in.vsync_n <= bypass_vs(C_DELAY_DEPTH - 1);
            s_io_align_in.field_n <= bypass_field(C_DELAY_DEPTH - 1);
        end if;
    end process p_stage7;

    --==========================================================================
    -- Bypass delay line shift register
    -- Index 0 = newest (just-arrived data_in),
    -- Index DEPTH-1 = oldest (consumed by p_stage7).
    --==========================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            bypass_y     <= data_in.y       & bypass_y(0 to C_DELAY_DEPTH - 2);
            bypass_u     <= data_in.u       & bypass_u(0 to C_DELAY_DEPTH - 2);
            bypass_v     <= data_in.v       & bypass_v(0 to C_DELAY_DEPTH - 2);
            bypass_hs    <= data_in.hsync_n & bypass_hs(0 to C_DELAY_DEPTH - 2);
            bypass_vs    <= data_in.vsync_n & bypass_vs(0 to C_DELAY_DEPTH - 2);
            bypass_field <= data_in.field_n & bypass_field(0 to C_DELAY_DEPTH - 2);
            bypass_avid  <= data_in.avid    & bypass_avid(0 to C_DELAY_DEPTH - 2);
        end if;
    end process p_delay;

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (7 -> 8).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture titler;
