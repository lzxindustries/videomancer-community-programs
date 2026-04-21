-- Copyright (C) 2026 Pete Appleby
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   YUV Shape Key
--
-- Author:
--   Pete Appleby
--
-- Overview:
--   Shape-based keyer operating directly in YUV space (no colour conversion).
--   For each pixel a shape test determines inside/outside. Inside pixels pass
--   the original image; outside pixels output black (Y=0, U=512, V=512).
--   Dither fill modes stochastically dither the boundary. A global blend
--   slider mixes the keyed output with the original image.
--
--   Shapes (S2=MSB, S3=LSB):
--     00  Rectangle  — axis-aligned bounding box (L-inf norm)
--     01  Ellipse    — elliptical region (L2 norm approximation)
--     10  Diamond    — rotated rectangle (L1 norm)
--     11  Cross      — horizontal + vertical arm union (solid fill only in v1)
--
--   Knob 6 (Inflate/Pinch) blends between L1 (pinch) and L-inf (inflate):
--     knob6 < 341  → L1 norm (diamond / max pinch)
--     341–682      → midpoint blend (ellipse-like, no sqrt)
--     > 682        → L-inf norm (rectangle / max inflate)
--   Default positions per shape: Rectangle=high, Ellipse=mid, Diamond=low.
--   Knob 6 is ignored for Cross shape.
--
--   Fill modes (S4=MSB, S5=LSB):
--     00  Solid        — binary inside/outside
--     01  Slight dither — lfsr noise scaled /4, stochastic boundary
--     10  More dither  — lfsr noise scaled /2
--     11  Max dither   — full lfsr noise
--   Dither not applied to Cross shape (always solid).
--
--   Invert (S1): swaps inside and outside.
--
--   Global blend: Slider at 0 = original image (effective bypass).
--
-- Architecture:
--   Stage 0   — Pixel counter; register data_in; decode controls; get
--               h/v_active from resolution_pkg; advance LFSR        (1 clk) → T+1
--   Stage 1   — cx, cy, w, h: four 8×8 multiplications (knob>>2 × res>>4)   (1 clk) → T+2
--   Stage 2   — dx, dy, abs_dx, abs_dy (8-bit >>3 scales); cross arms       (1 clk) → T+3
--   Stage 3   — norm products + threshold: three 8×8 → 16-bit multiplies    (1 clk) → T+4
--   Stage 4   — L1, L-inf, mid; norm mux; shape test; dither comparison    (1 clk) → T+5
--   Stage 5   — invert; matte select; keyed output mux                      (1 clk) → T+6
--   Stage 6   — Global blend (3x interpolator_u)                           (4 clks) → T+10
--
-- Latency: 10 clock cycles.
--
-- Videomancer UV Convention:
--   data_in.u = Cr (red-difference), data_in.v = Cb. Swapped vs standard.
--   Black: Y=0, U=512, V=512 (neutral chroma).
--
-- Register Map:
--   Register  0: Centre X        (0–1023)   rotary_potentiometer_1
--   Register  1: Centre Y        (0–1023)   rotary_potentiometer_2
--   Register  2: Width           (0–1023)   rotary_potentiometer_3
--   Register  3: Height          (0–1023)   rotary_potentiometer_4
--   Register  4: Scale           (0–1023)   rotary_potentiometer_5
--   Register  5: Inflate/Pinch   (0–1023)   rotary_potentiometer_6
--   Register  6: Packed switches (Off='0', On='1'):
--     bit 0: Invert    (1=On/swap inside-outside)         toggle_switch_7
--     bit 1: Shape Hi  (MSB of 2-bit shape code)          toggle_switch_8
--     bit 2: Shape Lo  (LSB of 2-bit shape code)          toggle_switch_9
--     bit 3: Fill Hi   (MSB of 2-bit fill code)           toggle_switch_10
--     bit 4: Fill Lo   (LSB of 2-bit fill code)           toggle_switch_11
--   Register  7: Wet/Dry blend   (0=dry, 1023=wet)        linear_potentiometer_12
--   Register  8: timing_id       (from SDK, read-only)
--
-- Known limitations (v1):
--   - Cross shape: dither fill modes fall back to solid (no norm for cross geometry).
--   - Ellipse in dither modes uses L1/L-inf blend norm, not true L2; visually similar.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.resolution_pkg.all;

architecture yuv_shape_key of program_top is

    -- Total pipeline latency: 1+1+1+1+1+1+4 = 10 clocks
    constant C_PROCESSING_DELAY_CLKS : integer := 10;
    -- Original data delayed this many clocks for global blend dry input.
    -- Processed output valid at T+6; delay raw data by 6 clocks.
    constant C_PRE_GLOBAL_DELAY_CLKS : integer := 6;

    -- Neutral chroma for "black" output (U=512, V=512 in YUV space)
    constant C_CHROMA_ZERO : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) :=
        to_unsigned(512, C_VIDEO_DATA_WIDTH);

    --------------------------------------------------------------------------------
    -- Global blend control
    --------------------------------------------------------------------------------
    signal s_global_blend : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------------
    -- LFSR (free-running — never reseeded)
    -- lfsr16 advances every clock; lower 10 bits used as dither noise.
    --------------------------------------------------------------------------------
    signal s_lfsr_q : std_logic_vector(15 downto 0);

    --------------------------------------------------------------------------------
    -- Pixel counter (inline — avid/vsync_n edge detection)
    -- h_count: 0 at first active pixel of each line, increments during avid.
    -- v_count: resets to 0 at vsync_n falling edge; increments at each line start.
    -- Both outputs are registered and valid at T+1 (Stage 0 output).
    --------------------------------------------------------------------------------
    signal s_h_count    : unsigned(11 downto 0) := (others => '0');
    signal s_v_count    : unsigned(11 downto 0) := (others => '0');
    signal s_avid_prev  : std_logic := '0';
    signal s_vsync_prev : std_logic := '1';

    --------------------------------------------------------------------------------
    -- Stage 0 → Stage 1 (T+1)
    --------------------------------------------------------------------------------
    signal s0_y        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_u        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_v        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_avid     : std_logic := '0';
    -- h/v counts available at T+1 directly from counter update
    -- (counter signals s_h_count / s_v_count already registered)
    signal s0_h_active : unsigned(11 downto 0) := (others => '0');
    signal s0_v_active : unsigned(11 downto 0) := (others => '0');
    signal s0_knob1    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_knob2    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_knob3    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_knob4    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_knob5    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s0_inv      : std_logic := '0';
    signal s0_shape    : std_logic_vector(1 downto 0) := "00";  -- {S2,S3}
    signal s0_fill     : std_logic_vector(1 downto 0) := "00";  -- {S4,S5}
    -- Knob 6 decoded to 2-bit norm selection: "00"=L1, "01"=mid, "10"=L-inf
    signal s0_norm_sel : std_logic_vector(1 downto 0) := "10";
    -- Dither noise: lfsr16[9:0] registered in Stage 0
    signal s0_noise    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 1 → Stage 2 (T+2): cx, cy, w, h computed
    --------------------------------------------------------------------------------
    signal s1_cx       : unsigned(11 downto 0) := (others => '0');
    signal s1_cy       : unsigned(11 downto 0) := (others => '0');
    signal s1_w        : unsigned(10 downto 0) := (others => '0');
    signal s1_h        : unsigned(10 downto 0) := (others => '0');
    -- Pixel data delay
    signal s1_y        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s1_u        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s1_v        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s1_avid     : std_logic := '0';
    signal s1_h_count  : unsigned(11 downto 0) := (others => '0');
    signal s1_v_count  : unsigned(11 downto 0) := (others => '0');
    signal s1_inv      : std_logic := '0';
    signal s1_shape    : std_logic_vector(1 downto 0) := "00";
    signal s1_fill     : std_logic_vector(1 downto 0) := "00";
    signal s1_norm_sel : std_logic_vector(1 downto 0) := "10";
    signal s1_noise    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 2 → Stage 3 (T+3): abs_dx, abs_dy, scaled 8-bit values, cross arms
    -- Threshold is NOT computed here — moved to Stage 3 to ease timing.
    -- 8-bit scaled values (>>3) are used for norm products in Stage 3.
    -- All four quantities (abs_dx8, abs_dy8, w8, h8) are shifted by the same K=3,
    -- so the comparison abs_dx8*h8 <= w8*h8 is equivalent to abs_dx*h <= w*h.
    --------------------------------------------------------------------------------
    signal s2_abs_dx   : unsigned(11 downto 0) := (others => '0');  -- full precision
    signal s2_abs_dy   : unsigned(11 downto 0) := (others => '0');
    signal s2_abs_dx8  : unsigned(7 downto 0)  := (others => '0');  -- abs_dx >> 3
    signal s2_abs_dy8  : unsigned(7 downto 0)  := (others => '0');  -- abs_dy >> 3
    signal s2_w8       : unsigned(7 downto 0)  := (others => '0');  -- w >> 3
    signal s2_h8       : unsigned(7 downto 0)  := (others => '0');  -- h >> 3
    signal s2_arm_h    : std_logic := '0';  -- cross horizontal arm flag
    signal s2_arm_v    : std_logic := '0';  -- cross vertical arm flag
    -- Pixel data delay
    signal s2_y        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s2_u        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s2_v        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s2_avid     : std_logic := '0';
    signal s2_inv      : std_logic := '0';
    signal s2_shape    : std_logic_vector(1 downto 0) := "00";
    signal s2_fill     : std_logic_vector(1 downto 0) := "00";
    signal s2_norm_sel : std_logic_vector(1 downto 0) := "10";
    signal s2_noise    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Stage 3 → Stage 4 (T+4): norm products and threshold (all 8×8 → 16-bit)
    -- Operands are 8-bit (shifted by 3) so multiplies fit at 74.25 MHz.
    -- Three independent 8×8 multiplies: prod_dxh, prod_dyw, threshold.
    --------------------------------------------------------------------------------
    signal s3_prod_dxh : unsigned(15 downto 0) := (others => '0');  -- abs_dx8 * h8
    signal s3_prod_dyw : unsigned(15 downto 0) := (others => '0');  -- abs_dy8 * w8
    signal s3_threshold: unsigned(15 downto 0) := (others => '0');  -- w8 * h8
    -- Pixel data delay
    signal s3_y        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s3_u        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s3_v        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s3_avid     : std_logic := '0';
    signal s3_inv      : std_logic := '0';
    signal s3_shape    : std_logic_vector(1 downto 0) := "00";
    signal s3_fill     : std_logic_vector(1 downto 0) := "00";
    signal s3_norm_sel : std_logic_vector(1 downto 0) := "10";
    signal s3_noise    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s3_arm_h    : std_logic := '0';
    signal s3_arm_v    : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 4 → Stage 5 (T+5): shape test, dither comparison
    --------------------------------------------------------------------------------
    signal s4_in_shape : std_logic := '0';  -- solid shape test result
    signal s4_in_dith  : std_logic := '0';  -- dithered shape test result
    signal s4_shape    : std_logic_vector(1 downto 0) := "00";
    signal s4_fill     : std_logic_vector(1 downto 0) := "00";
    -- Pixel data delay
    signal s4_y        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s4_u        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s4_v        : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s4_avid     : std_logic := '0';
    signal s4_inv      : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 5 → Stage 6 (T+6): processed pixel output
    --------------------------------------------------------------------------------
    signal s5_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s5_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s5_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s5_valid    : std_logic := '0';

    --------------------------------------------------------------------------------
    -- Stage 6: Global Blend outputs (T+10)
    --------------------------------------------------------------------------------
    signal s_global_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_global_y_valid : std_logic;
    signal s_global_u_valid : std_logic;
    signal s_global_v_valid : std_logic;

    -- Global blend dry input (original data delayed to T+6)
    signal s_y_for_global   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_u_for_global   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_v_for_global   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------------
    -- Bypass delay line (sync + data, C_PROCESSING_DELAY_CLKS clocks)
    --------------------------------------------------------------------------------
    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;

begin

    --------------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------------
    s_global_blend <= unsigned(registers_in(7));

    --------------------------------------------------------------------------------
    -- LFSR: free-running 16-bit, lower 10 bits used for dither noise
    --------------------------------------------------------------------------------
    lfsr16_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            seed   => x"ACE1",
            load   => '0',
            q      => s_lfsr_q
        );

    --------------------------------------------------------------------------------
    -- Stage 0: Pixel Counter + Control Decode + Data Register
    -- Latency: 1 clock. Input T+0, output T+1.
    --
    -- Pixel counter: h_count resets to 0 at avid rising edge, increments during
    --   active video. v_count resets to 0 at vsync_n falling edge, increments at
    --   each active line start (avid rising edge). Counter outputs are registered
    --   signals updated each clock, valid alongside s0_y/u/v at T+1.
    --
    -- Control decode: timing_id selects h/v_active via resolution_pkg (pure mux).
    --   Knob 6 decoded to 2-bit norm selector (carry-chain comparisons here,
    --   not in Stage 4 critical path).
    --------------------------------------------------------------------------------
    p_stage0 : process(clk)
        variable v_timing_id : t_video_timing_id;
        variable v_knob6     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Edge detection state
            s_avid_prev  <= data_in.avid;
            s_vsync_prev <= data_in.vsync_n;

            -- H counter: reset to 0 at avid rising edge (first active pixel),
            -- increment during active video.
            if s_avid_prev = '0' and data_in.avid = '1' then
                s_h_count <= (others => '0');
            elsif data_in.avid = '1' then
                s_h_count <= s_h_count + 1;
            end if;

            -- V counter: vsync falling edge resets (highest priority);
            -- avid rising edge increments (new active line).
            if s_vsync_prev = '1' and data_in.vsync_n = '0' then
                s_v_count <= (others => '0');
            elsif s_avid_prev = '0' and data_in.avid = '1' then
                s_v_count <= s_v_count + 1;
            end if;

            -- Data register
            s0_y    <= data_in.y;
            s0_u    <= data_in.u;
            s0_v    <= data_in.v;
            s0_avid <= data_in.avid;

            -- Resolution lookup (pure mux on registered timing_id)
            v_timing_id   := registers_in(8)(3 downto 0);
            s0_h_active   <= get_h_active(v_timing_id);
            s0_v_active   <= get_v_active(v_timing_id);

            -- Knob registers
            s0_knob1 <= unsigned(registers_in(0));
            s0_knob2 <= unsigned(registers_in(1));
            s0_knob3 <= unsigned(registers_in(2));
            s0_knob4 <= unsigned(registers_in(3));
            s0_knob5 <= unsigned(registers_in(4));

            -- Switch decode
            s0_inv   <= registers_in(6)(0);
            s0_shape <= registers_in(6)(1) & registers_in(6)(2);  -- {S2, S3}
            s0_fill  <= registers_in(6)(3) & registers_in(6)(4);  -- {S4, S5}

            -- Knob 6 → norm selector (carry-chain comparisons, done here in Stage 0)
            v_knob6 := unsigned(registers_in(5));
            if v_knob6 < 341 then
                s0_norm_sel <= "00";   -- L1 norm (pinch / diamond tendency)
            elsif v_knob6 < 683 then
                s0_norm_sel <= "01";   -- midpoint blend (ellipse-like)
            else
                s0_norm_sel <= "10";   -- L-inf norm (inflate / rectangle tendency)
            end if;

            -- Dither noise: register lfsr16[9:0] for use downstream
            s0_noise <= unsigned(s_lfsr_q(9 downto 0));
        end if;
    end process p_stage0;

    --------------------------------------------------------------------------------
    -- Stage 1: Centre and Size Computation
    -- Latency: 1 clock. Input T+1, output T+2.
    -- Four 10×12 and 10×10 multiplications on registered control values.
    -- All inputs are registered Stage 0 outputs; no pixel-path carry chains here.
    --
    --   cx = (knob1 * h_active) >> 10   (centre X in pixels, 0..h_active)
    --   cy = (knob2 * v_active) >> 10   (centre Y in pixels, 0..v_active)
    --   w  = (knob3 * knob5) >> 9       (horizontal half-width, 0..~2046)
    --   h  = (knob4 * knob5) >> 9       (vertical half-height, 0..~2046)
    --
    -- knob5=512 gives scale=1x (512/512 = 1.0 after >> 9 correction).
    --------------------------------------------------------------------------------
    p_stage1 : process(clk)
        -- All multiplies use 8-bit operands (inputs right-shifted) to keep
        -- carry-chain multiply depth within 74.25 MHz timing budget.
        -- 8-bit × 8-bit → 16-bit products; results sliced to needed width.
        variable v_k1_8    : unsigned(7 downto 0);   -- knob1 >> 2
        variable v_k2_8    : unsigned(7 downto 0);   -- knob2 >> 2
        variable v_k3_8    : unsigned(7 downto 0);   -- knob3 >> 2
        variable v_k4_8    : unsigned(7 downto 0);   -- knob4 >> 2
        variable v_k5_8    : unsigned(7 downto 0);   -- knob5 >> 2
        variable v_ha_8    : unsigned(7 downto 0);   -- h_active >> 4
        variable v_va_8    : unsigned(7 downto 0);   -- v_active >> 4
        variable v_cx_full : unsigned(15 downto 0);  -- 8×8 product
        variable v_cy_full : unsigned(15 downto 0);
        variable v_w_full  : unsigned(15 downto 0);
        variable v_h_full  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- Reduce to 8-bit operands (lose 2 LSBs of knobs, 4 LSBs of resolution)
            v_k1_8 := s0_knob1(9 downto 2);
            v_k2_8 := s0_knob2(9 downto 2);
            v_k3_8 := s0_knob3(9 downto 2);
            v_k4_8 := s0_knob4(9 downto 2);
            v_k5_8 := s0_knob5(9 downto 2);
            v_ha_8 := s0_h_active(11 downto 4);
            v_va_8 := s0_v_active(11 downto 4);

            -- cx = (knob1 × h_active) >> 10 ≈ (knob1>>2 × h_active>>4) >> 4
            -- Verified: knob=512, h=1920 → 128×120>>4 = 15360>>4 = 960 ✓
            v_cx_full := v_k1_8 * v_ha_8;
            s1_cx <= v_cx_full(15 downto 4);   -- >>4 → 12-bit

            -- cy = (knob2 × v_active) >> 10 ≈ (knob2>>2 × v_active>>4) >> 4
            v_cy_full := v_k2_8 * v_va_8;
            s1_cy <= v_cy_full(15 downto 4);

            -- w = (knob3 × knob5) >> 9 ≈ (knob3>>2 × knob5>>2) >> 5
            -- Verified: knob3=knob5=512 → 128×128>>5 = 16384>>5 = 512 ✓
            v_w_full := v_k3_8 * v_k5_8;
            s1_w <= v_w_full(15 downto 5);     -- >>5 → 11-bit

            -- h = (knob4 × knob5) >> 9 ≈ same
            v_h_full := v_k4_8 * v_k5_8;
            s1_h <= v_h_full(15 downto 5);

            -- Delay pixel data and controls
            s1_y        <= s0_y;
            s1_u        <= s0_u;
            s1_v        <= s0_v;
            s1_avid     <= s0_avid;
            s1_h_count  <= s_h_count;   -- counter is already registered at T+1
            s1_v_count  <= s_v_count;
            s1_inv      <= s0_inv;
            s1_shape    <= s0_shape;
            s1_fill     <= s0_fill;
            s1_norm_sel <= s0_norm_sel;
            s1_noise    <= s0_noise;
        end if;
    end process p_stage1;

    --------------------------------------------------------------------------------
    -- Stage 2: Displacement, Abs Value, Threshold, Cross Arms
    -- Latency: 1 clock. Input T+2, output T+3.
    --
    --   dx = signed(h_count) - signed(cx)   signed displacement from centre
    --   dy = signed(v_count) - signed(cy)
    --   abs_dx, abs_dy: magnitude (12-bit unsigned)
    --   threshold = w * h   (22-bit, used for norm comparison in Stage 4)
    --   arm_h / arm_v: cross shape arm membership flags
    --
    -- threshold (11b×11b) multiply is here alongside abs computation (comparisons).
    -- Cross arm comparisons are pure LUT logic on registered abs values — BUT since
    -- abs_dx/abs_dy are computed in this same stage, they are variables not signals;
    -- the comparisons complete before the register clock edge. Acceptable depth.
    --------------------------------------------------------------------------------
    p_stage2 : process(clk)
        -- Stage 2: displacement, abs value, 8-bit scaled values, cross arm flags.
        -- NO multiplies here — threshold moved to Stage 3 alongside norm products.
        -- 8-bit scaled values: right-shift all four quantities by K=3 so that
        --   (abs_dx8*h8 <= w8*h8) is equivalent to (abs_dx*h <= w*h) up to truncation.
        -- Cross arms use full-precision abs values against registered w/h.
        variable v_dx     : signed(12 downto 0);
        variable v_dy     : signed(12 downto 0);
        variable v_neg_dx : signed(12 downto 0);
        variable v_neg_dy : signed(12 downto 0);
        variable v_abs_dx : unsigned(11 downto 0);
        variable v_abs_dy : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Signed displacement from shape centre
            v_dx := signed(resize(s1_h_count, 13)) - signed(resize(s1_cx, 13));
            v_dy := signed(resize(s1_v_count, 13)) - signed(resize(s1_cy, 13));

            -- Absolute value: negate into an intermediate variable, then slice.
            -- |dx| <= 1920, |dy| <= 1080 — both fit safely in 12 bits.
            if v_dx(12) = '1' then
                v_neg_dx := -v_dx;
                v_abs_dx := unsigned(v_neg_dx(11 downto 0));
            else
                v_abs_dx := unsigned(v_dx(11 downto 0));
            end if;
            if v_dy(12) = '1' then
                v_neg_dy := -v_dy;
                v_abs_dy := unsigned(v_neg_dy(11 downto 0));
            else
                v_abs_dy := unsigned(v_dy(11 downto 0));
            end if;

            s2_abs_dx <= v_abs_dx;
            s2_abs_dy <= v_abs_dy;

            -- 8-bit scaled values (>>3): bit-select only, zero combinatorial delay.
            -- abs_dx max=1920 → >>3 = 240 (fits in 8 bits)
            -- w/h max=2046 → >>3 = 255 (fits in 8 bits)
            s2_abs_dx8 <= v_abs_dx(10 downto 3);
            s2_abs_dy8 <= v_abs_dy(10 downto 3);
            s2_w8      <= s1_w(10 downto 3);
            s2_h8      <= s1_h(10 downto 3);

            -- Cross arm flags: full-precision comparisons (carry chains, not multiplies)
            -- arm_h: horizontal arm — abs_dx <= w AND abs_dy <= h
            -- arm_v: vertical arm  — abs_dx <= h AND abs_dy <= w
            if v_abs_dx <= s1_w and v_abs_dy <= s1_h then
                s2_arm_h <= '1';
            else
                s2_arm_h <= '0';
            end if;
            if v_abs_dx <= s1_h and v_abs_dy <= s1_w then
                s2_arm_v <= '1';
            else
                s2_arm_v <= '0';
            end if;

            -- Delay pixel data and controls
            s2_y        <= s1_y;
            s2_u        <= s1_u;
            s2_v        <= s1_v;
            s2_avid     <= s1_avid;
            s2_inv      <= s1_inv;
            s2_shape    <= s1_shape;
            s2_fill     <= s1_fill;
            s2_norm_sel <= s1_norm_sel;
            s2_noise    <= s1_noise;
        end if;
    end process p_stage2;

    --------------------------------------------------------------------------------
    -- Stage 3: Norm Products + Threshold
    -- Latency: 1 clock. Input T+3, output T+4.
    -- Three independent 8×8 → 16-bit multiplications (carry chains).
    --
    --   prod_dxh  = abs_dx8 * h8   (abs_dx >> 3 × h >> 3)
    --   prod_dyw  = abs_dy8 * w8   (abs_dy >> 3 × w >> 3)
    --   threshold = w8 * h8        (moved here from Stage 2 to share multiply stage)
    --
    -- All four operands shifted by K=3 so comparison prod_dxh <= threshold is
    -- equivalent to abs_dx*h <= w*h. 8×8 products fit comfortably at 74.25 MHz.
    --------------------------------------------------------------------------------
    p_stage3 : process(clk)
    begin
        if rising_edge(clk) then
            -- Three independent 8×8 → 16-bit multiplications (much faster than 12×11).
            -- All four operands shifted by K=3: comparison (prod_dxh <= threshold) is
            -- equivalent to the full-precision (abs_dx*h <= w*h) up to 3-bit truncation.
            s3_prod_dxh <= s2_abs_dx8 * s2_h8;   -- 8×8 = 16-bit
            s3_prod_dyw <= s2_abs_dy8 * s2_w8;   -- 8×8 = 16-bit
            s3_threshold <= s2_w8 * s2_h8;        -- 8×8 = 16-bit

            -- Delay through-signals
            s3_y        <= s2_y;
            s3_u        <= s2_u;
            s3_v        <= s2_v;
            s3_avid     <= s2_avid;
            s3_inv      <= s2_inv;
            s3_shape    <= s2_shape;
            s3_fill     <= s2_fill;
            s3_norm_sel <= s2_norm_sel;
            s3_noise    <= s2_noise;
            s3_arm_h    <= s2_arm_h;
            s3_arm_v    <= s2_arm_v;
        end if;
    end process p_stage3;

    --------------------------------------------------------------------------------
    -- Stage 4: Norm Selection, Shape Test, Dither Comparison
    -- Latency: 1 clock. Input T+4, output T+5.
    --
    -- L1   = prod_dxh + prod_dyw                (diamond norm, 24-bit)
    -- Linf = max(prod_dxh, prod_dyw)            (rectangle norm, 23-bit)
    -- mid  = (L1 + Linf) >> 1                   (ellipse approximation, 23-bit)
    -- norm = mux(L1, mid, Linf) by norm_sel
    --
    -- Solid shape test: norm <= threshold (resize threshold to 24-bit for compare)
    -- Cross shape:      in_shape = arm_h OR arm_v (no norm used)
    --
    -- Dither (non-cross shapes only):
    --   noise_scaled = noise >> fill_shift   (shift: slight=2, more=1, max=0)
    --   pass = norm <= resize(threshold, 24) + noise_scaled
    -- Comparing norm against a widened threshold avoids underflow issues.
    -- Cross shape: dither not supported — always uses solid in_shape.
    --------------------------------------------------------------------------------
    p_stage4 : process(clk)
        -- Products and threshold are 16-bit (from 8×8 multiplies in Stage 3).
        -- L1 = prod_dxh + prod_dyw → 17-bit.  All arithmetic stays ≤ 17 bits.
        variable v_l1          : unsigned(16 downto 0);  -- 17-bit L1 norm
        variable v_linf        : unsigned(15 downto 0);  -- 16-bit L-inf norm
        variable v_mid         : unsigned(16 downto 0);  -- 17-bit midpoint
        variable v_norm        : unsigned(16 downto 0);  -- selected norm
        variable v_thresh17    : unsigned(16 downto 0);  -- threshold padded to 17-bit
        variable v_noise_sc    : unsigned(9 downto 0);   -- scaled noise (10-bit)
        variable v_thresh_dith : unsigned(16 downto 0);  -- dithered threshold
        variable v_solid       : std_logic;
        variable v_dith        : std_logic;
    begin
        if rising_edge(clk) then
            -- L1 = prod_dxh + prod_dyw (16-bit + 16-bit → 17-bit)
            v_l1 := resize(s3_prod_dxh, 17) + resize(s3_prod_dyw, 17);
            -- L-inf = max(prod_dxh, prod_dyw) — 16-bit
            if s3_prod_dxh >= s3_prod_dyw then
                v_linf := s3_prod_dxh;
            else
                v_linf := s3_prod_dyw;
            end if;
            -- Midpoint: (L1 + L-inf) >> 1 — stays 17-bit
            v_mid := shift_right(v_l1 + resize(v_linf, 17), 1);

            -- Norm mux by knob6 range (L1 pinch / mid ellipse / L-inf inflate)
            case s3_norm_sel is
                when "00"   => v_norm := v_l1;
                when "01"   => v_norm := v_mid;
                when others => v_norm := resize(v_linf, 17);
            end case;

            -- Threshold resized to 17-bit for comparison
            v_thresh17 := resize(s3_threshold, 17);

            -- Solid shape test
            case s3_shape is
                when "11"   =>  -- Cross: arm union (no norm needed)
                    v_solid := s3_arm_h or s3_arm_v;
                when others =>
                    if v_norm <= v_thresh17 then v_solid := '1';
                    else                         v_solid := '0'; end if;
            end case;

            -- Dither: noise added to threshold side (avoids underflow).
            -- noise_scaled << threshold so the addition doesn't overflow 17-bit
            -- (max threshold=65025 + max noise=1023 = 66048 < 131072 = 2^17 ✓).
            case s3_fill is
                when "01"   => v_noise_sc := shift_right(s3_noise, 2);  -- slight: /4
                when "10"   => v_noise_sc := shift_right(s3_noise, 1);  -- more:  /2
                when others => v_noise_sc := s3_noise;                   -- max:   full
            end case;
            v_thresh_dith := v_thresh17 + resize(v_noise_sc, 17);
            if s3_shape = "11" then
                v_dith := v_solid;  -- Cross: no dither
            elsif v_norm <= v_thresh_dith then
                v_dith := '1';
            else
                v_dith := '0';
            end if;

            s4_in_shape <= v_solid;
            s4_in_dith  <= v_dith;

            -- Delay through-signals
            s4_y     <= s3_y;
            s4_u     <= s3_u;
            s4_v     <= s3_v;
            s4_avid  <= s3_avid;
            s4_inv   <= s3_inv;
            s4_shape <= s3_shape;
            s4_fill  <= s3_fill;
        end if;
    end process p_stage4;

    --------------------------------------------------------------------------------
    -- Stage 5: Invert, Matte Select, Output Mux
    -- Latency: 1 clock. Input T+5, output T+6.
    -- All inputs are registered — pure LUT logic, no carry chains.
    --
    -- Apply invert (S1): flip in_shape and in_dith if s4_inv='1'.
    -- Select matte: solid modes use in_shape; dither modes use in_dith.
    --   Fill "00" (Solid):         matte = in_shape
    --   Fill "01"/"10"/"11":       matte = in_dith
    -- Output mux:
    --   avid='0' (blanking): pass original data unchanged.
    --   avid='1', matte=1:   inside — pass original pixel
    --   avid='1', matte=0:   outside — output black (Y=0, U=512, V=512)
    --------------------------------------------------------------------------------
    p_stage5 : process(clk)
        variable v_in  : std_logic;
        variable v_use_dith : std_logic;
    begin
        if rising_edge(clk) then
            -- Select which test result to use
            if s4_fill = "00" then
                v_use_dith := '0';
            else
                v_use_dith := '1';
            end if;

            -- Apply invert
            if v_use_dith = '0' then
                v_in := s4_in_shape xor s4_inv;
            else
                v_in := s4_in_dith  xor s4_inv;
            end if;

            -- Output mux
            if s4_avid = '0' then
                -- Blanking: preserve sync structure
                s5_y <= unsigned(s4_y);
                s5_u <= unsigned(s4_u);
                s5_v <= unsigned(s4_v);
            elsif v_in = '1' then
                -- Inside: pass original pixel
                s5_y <= unsigned(s4_y);
                s5_u <= unsigned(s4_u);
                s5_v <= unsigned(s4_v);
            else
                -- Outside: black
                s5_y <= (others => '0');
                s5_u <= C_CHROMA_ZERO;
                s5_v <= C_CHROMA_ZERO;
            end if;

            s5_valid <= s4_avid;
        end if;
    end process p_stage5;

    --------------------------------------------------------------------------------
    -- Global Blend Dry Delay
    -- Delays original data_in by C_PRE_GLOBAL_DELAY_CLKS clocks so it aligns
    -- with the processed output at T+6 for the global blend dry input.
    --------------------------------------------------------------------------------
    p_global_dry_delay : process(clk)
        type t_data_delay is array (0 to C_PRE_GLOBAL_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_delay : t_data_delay := (others => (others => '0'));
        variable v_u_delay : t_data_delay := (others => (others => '0'));
        variable v_v_delay : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_y_delay := unsigned(data_in.y) & v_y_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_u_delay := unsigned(data_in.u) & v_u_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            v_v_delay := unsigned(data_in.v) & v_v_delay(0 to C_PRE_GLOBAL_DELAY_CLKS - 2);
            s_y_for_global <= v_y_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_u_for_global <= v_u_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
            s_v_for_global <= v_v_delay(C_PRE_GLOBAL_DELAY_CLKS - 1);
        end if;
    end process p_global_dry_delay;

    --------------------------------------------------------------------------------
    -- Stage 6: Global Wet/Dry Blend
    -- Latency: 4 clocks. Input T+6, output T+10.
    -- a=original (dry), b=processed (wet), t=global_blend slider.
    --------------------------------------------------------------------------------
    interp_global_y : entity work.interpolator_u
        generic map(G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => s5_valid,
                 a => s_y_for_global, b => s5_y, t => s_global_blend,
                 result => s_global_y, valid => s_global_y_valid);

    interp_global_u : entity work.interpolator_u
        generic map(G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => s5_valid,
                 a => s_u_for_global, b => s5_u, t => s_global_blend,
                 result => s_global_u, valid => s_global_u_valid);

    interp_global_v : entity work.interpolator_u
        generic map(G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => s5_valid,
                 a => s_v_for_global, b => s5_v, t => s_global_blend,
                 result => s_global_v, valid => s_global_v_valid);

    --------------------------------------------------------------------------------
    -- Bypass Delay Line (sync + data, C_PROCESSING_DELAY_CLKS clocks)
    -- Delays hsync_n/vsync_n/field_n to align with pipeline output at T+10.
    -- Also delays y/u/v for the bypass mux (no bypass switch in this program;
    -- bypass path kept for structural consistency but mux is unused).
    --------------------------------------------------------------------------------
    p_bypass_delay : process(clk)
        type t_sync_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
    begin
        if rising_edge(clk) then
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_hsync_n_delayed <= v_hsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_vsync_n_delayed <= v_vsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_field_n_delayed <= v_field_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_bypass_delay;

    --------------------------------------------------------------------------------
    -- Output
    --------------------------------------------------------------------------------
    data_out.y       <= std_logic_vector(s_global_y);
    data_out.u       <= std_logic_vector(s_global_u);
    data_out.v       <= std_logic_vector(s_global_v);
    data_out.avid    <= s_global_y_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end architecture yuv_shape_key;
