-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Ridgeline: catalogue combo #98 (seed 20260924) -- Perlin / simplex
-- displacement + edge detect (Roberts / gradient).  Three knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Amount   noise displacement, squared, 0..+-511 px (+-766 with two
--               octaves; 1080-line px, scaled per mode)
--   K2 Scale    noise frequency across x, squared (glided)
--   K3 Speed    noise drift speed
--   K4 Gain     edge gain
--   K5 Edges    Roberts / Horizontal / Vertical / Sobel (zone hysteresis)
--   K6 Ink      ink luma, black .. white (continuous); the Paper ground is
--               white under dark ink, black under light ink
--
--   S7  Ground  edges inked over the displaced picture / on plain paper
--               (the lines carry the picture's colour)
--   S8  Order   edges of the displaced picture against the input line /
--               edges of the input, displaced with the picture
--   S9  Chroma  chroma follows luma / chroma on its own noise phase
--   S10 Push    edges also push the noise (input gradient adds to the offset)
--   S11 Octaves one / two octaves
--   P12 Rows    noise frequency down the lines, squared (glided): 0 = pure
--               column waves, 100% = a 32-line ripple that shreds the picture
--
-- The luma line buffer holds luma (8 bits) and the input's edge strength
-- (4 bits) per pixel.  Warp-Edge inks the edges between the displaced line
-- above and the input line (the ridges); Edge-Warp inks the stored edges,
-- read through the same displaced address, so they travel with the
-- picture.  The write-side detector's "line above" is the write bank's own
-- contents (2 lines up), read through that bank's otherwise idle port 6
-- clocks ahead of the write.  Ink is a lerp toward the ink luma by the edge
-- alpha; on Paper the chroma is scaled by the same alpha.
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel (no black seam).
--
-- HARDWARE CONTRACTS: blanking gate; one latency (24) in all modes; read
-- and write banks latched per line at the avid rise (the read tail runs
-- past hsync on SD/PAL front porches); U/V swapped on rows whose
-- hsync->avid parity differs from the previous row's (each row replays
-- the row above -- drape v1.3); output conditioning ([1 2 1] on Y/U/V +
-- luma scaled to white ~777: the HD HDMI bar fix from damier v0.4.4 --
-- thin ink lines are near-full-swing edges that move every frame);
-- no_rw_check on every RAM (luma: the write bank is read 6 columns ahead
-- of its write; chroma: the write bank's read data is discarded).
--
-- v0.2.3 (2026-10-17): resolution scaling.  The noise lives on a virtual
-- 1080-line canvas, stepped per mode from the video timing ID
-- (registers_in(8)): K1's amplitude (and Push's offset) is 1080-line pixels
-- (x 2/3 at 720p, x 1/2 at SD, where 720 px span 1440 units); the Scale
-- phase step per pixel is x the virtual x step (1.5 at 720p, 2 at SD) and
-- the Rows phase step per field line x the virtual line step (1.5 at 720p,
-- 1080/486 x 2 per 480i field line; a serial multiplier), so the waves keep
-- their size and count relative to the screen height in every mode.  1080p
-- is unchanged (every step is exactly 1.0).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture ridgeline of program_top is
    -- data_in -> data_out latency is 24 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;

    constant C_LAT  : integer := 24;
    constant C_EDGE : integer := 12;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    -- luma words: luma(9 downto 2) & edge alpha (0..15)
    -- Resolution scaling: virtual 1080-line steps from the timing ID.  x step
    -- in half units per pixel (1080: 2, 720p: 3, SD: 4), y step Q4.8 per
    -- FIELD line (1080 / active lines, doubled for interlace).
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

    type t_lb_y is array (0 to 2047) of std_logic_vector(11 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => x"100");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(11 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0 : signal is true;
    attribute no_rw_check of lbY1 : signal is true;
    attribute no_rw_check of lbU0 : signal is true;
    attribute no_rw_check of lbU1 : signal is true;
    attribute no_rw_check of lbV0 : signal is true;
    attribute no_rw_check of lbV1 : signal is true;

    signal s_k_amp, s_k_scale, s_k_speed, s_k_gain, s_k_mode, s_k_ink : unsigned(9 downto 0);
    signal s_sw_paper, s_sw_edgefirst, s_sw_csep, s_sw_push, s_sw_two : std_logic;
    signal s_p_rows : unsigned(9 downto 0);

    -- per-frame / quasi-static controls
    signal s_amp_p, s_sc_p, s_rw_p : unsigned(16 downto 0) := (others => '0');
    signal s_amp  : unsigned(8 downto 0) := (others => '0');            -- 0..511 px
    signal s_gs, s_gr : unsigned(13 downto 0) := (others => '0');       -- glided Scale / Rows (10.4)
    signal s_sc   : unsigned(15 downto 0) := (others => '0');           -- phase step per pixel (Q24)
    signal s_rw   : unsigned(15 downto 0) := (others => '0');           -- phase step per line / 8 (Q24)
    signal s_gain : unsigned(7 downto 0) := (others => '0');
    signal s_mode : unsigned(1 downto 0) := "00";
    signal s_inky, s_papery : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_paper_blk : std_logic := '0';
    signal s_fph1, s_fph2, s_lph1, s_lph2 : unsigned(23 downto 0) := (others => '0');
    -- resolution scaling (static per mode, decoded every clock)
    signal s_vxs    : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_vys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_amp_r  : unsigned(8 downto 0) := (others => '0');        -- K1 in 1080-line px
    signal s_a168, s_a3r : unsigned(16 downto 0) := (others => '0');
    signal s_scx    : unsigned(16 downto 0) := (others => '0');       -- Scale step x the x step
    -- Rows line step = (s_rw * y step) >> 8 on a free-running 16-clock serial
    -- multiplier (s_rw changes only at vsync)
    signal s_sm_cnt : unsigned(3 downto 0)  := (others => '0');
    signal s_sm_a   : unsigned(15 downto 0) := (others => '0');
    signal s_sm_b   : unsigned(11 downto 0) := (others => '0');
    signal s_sm_p   : unsigned(27 downto 0) := (others => '0');
    signal s_rwx    : unsigned(19 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');     -- s_wr_x = x of the stage-1 pixel
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_lw : std_logic := '0';                             -- s_lw = this line's write bank
    -- hsync->avid parity of this line and of each bank's capture line
    signal s_hpar, s_line_par, s_cap_par0, s_cap_par1, s_rowswap : std_logic := '0';
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);    -- line_width - 1 - C_EDGE
    signal s_we13 : signed(12 downto 0) := (others => '0');

    -- x by stage
    type t_x is array (2 to 9) of unsigned(10 downto 0);
    signal s_xs : t_x := (others => (others => '0'));
    -- input luma by stage
    type t_yc is array (2 to 13) of unsigned(9 downto 0);
    signal s_yc : t_yc := (others => (others => '0'));
    signal s_u2, s_v2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- write side: edges of the input against the write bank (2 lines up)
    signal s_up4, s_up5 : unsigned(7 downto 0) := (others => '0');
    signal s_wh5, s_wv5, s_wr15, s_wr25 : unsigned(7 downto 0) := (others => '0');
    signal s_we6 : unsigned(8 downto 0) := (others => '0');
    signal s_weg7 : unsigned(16 downto 0) := (others => '0');
    signal s_word8 : std_logic_vector(11 downto 0) := x"100";

    -- noise / address path
    signal s_ph1, s_ph2 : unsigned(23 downto 0) := (others => '0');
    signal s_sa1, s_sa2, s_ca1, s_ca2 : unsigned(7 downto 0) := (others => '0');
    signal s_q1, s_q2, s_qc1, s_qc2, s_r1, s_r2, s_rc1, s_rc2 : signed(9 downto 0) := (others => '0');
    signal s_n6, s_nc6 : signed(10 downto 0) := (others => '0');
    signal s_pn7, s_pnc7 : signed(20 downto 0) := (others => '0');
    signal s_gx3 : unsigned(9 downto 0) := (others => '0');
    type t_g is array (4 to 7) of unsigned(9 downto 0);
    signal s_gxd : t_g := (others => (others => '0'));
    signal s_aY8, s_aC8 : signed(12 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');
    signal s_ra0, s_ra1, s_rc10 : unsigned(10 downto 0) := (others => '0');

    -- read side
    signal s_wy12 : unsigned(9 downto 0) := to_unsigned(66, 10);
    signal s_wa12 : unsigned(3 downto 0) := (others => '0');
    signal s_wu12, s_wv12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pv13, s_r1_13, s_r2_13, s_gh13, s_gv13 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_y14, s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13, s_u14, s_v14, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_a13, s_a14, s_a15 : unsigned(7 downto 0) := (others => '0');
    signal s_e14 : unsigned(10 downto 0) := (others => '0');
    signal s_eg15 : unsigned(18 downto 0) := (others => '0');
    signal s_al16 : unsigned(7 downto 0) := (others => '0');            -- ink alpha 0..255
    -- separate luma / chroma alpha registers (the chroma one is 0 on the
    -- Picture ground, so they can't merge): each feeds its own products
    signal s_aly17, s_alc17 : unsigned(7 downto 0) := (others => '0');
    signal s_by16, s_by17, s_by18, s_by19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dy17, s_du17, s_dv17 : signed(10 downto 0) := (others => '0');
    signal s_u17, s_v17, s_u18, s_v18, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_hy18, s_ly18, s_hu18, s_lu18, s_hv18, s_lv18 : signed(15 downto 0) := (others => '0');
    signal s_py19, s_pu19, s_pv19 : signed(20 downto 0) := (others => '0');
    signal s_y20, s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');                 -- s_avid_sr(k) = avid of the stage-(k+1) pixel

    function f_ad8(a, b : unsigned(7 downto 0)) return unsigned is
    begin
        if a >= b then return a - b; else return b - a; end if;
    end function;

    function f_ad10(a, b : unsigned(9 downto 0)) return unsigned is
    begin
        if a >= b then return a - b; else return b - a; end if;
    end function;

begin

    s_k_amp   <= unsigned(registers_in(0));
    s_k_scale <= unsigned(registers_in(1));
    s_k_speed <= unsigned(registers_in(2));
    s_k_gain  <= unsigned(registers_in(3));
    s_k_mode  <= unsigned(registers_in(4));
    s_k_ink   <= unsigned(registers_in(5));
    s_sw_paper     <= registers_in(6)(0);
    s_sw_edgefirst <= registers_in(6)(1);
    s_sw_csep      <= registers_in(6)(2);
    s_sw_push      <= registers_in(6)(3);
    s_sw_two       <= registers_in(6)(4);
    s_p_rows <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_z  : unsigned(1 downto 0);
        variable v_lo : unsigned(7 downto 0);
        variable v_tg, v_cur, v_dg : signed(14 downto 0);
        variable v_sm : unsigned(16 downto 0);
        variable v_ah : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_avid_sr(0) = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- squared curves d*(d/16+1): linear at the bottom, 64x steeper at the top
            s_amp_p <= s_k_amp * (resize(s_k_amp(9 downto 4), 7) + 1);
            s_amp_r <= s_amp_p(15 downto 7);                                -- K1: 0..511 px (1080p)
            -- resolution scaling: mode steps (static per mode); K1 to real px
            -- (720p x 2/3 = (a*171 + 128) >> 8, SD x 1/2); Scale step x the x step
            s_vxs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_vys <= f_vstep_y(registers_in(8)(3 downto 0));
            s_a168 <= shift_left(resize(s_amp_r, 17), 7) + shift_left(resize(s_amp_r, 17), 5) + shift_left(resize(s_amp_r, 17), 3);
            s_a3r  <= shift_left(resize(s_amp_r, 17), 1) + resize(s_amp_r, 17) + 128;
            v_ah := resize(s_amp_r, 10) + 1;
            case s_vxs is
                when "011"  => s_amp <= resize(shift_right(s_a168 + s_a3r, 8), 9);
                               s_scx <= resize(s_sc, 17) + resize(s_sc(15 downto 1), 17);
                when "100"  => s_amp <= v_ah(9 downto 1);
                               s_scx <= s_sc & '0';
                when others => s_amp <= s_amp_r;
                               s_scx <= resize(s_sc, 17);
            end case;
            -- Rows line step = (s_rw * y step) >> 8 (1080p: s_rw): load at 0,
            -- 12 shift-add steps, latch at 13
            s_sm_cnt <= s_sm_cnt + 1;
            if s_sm_cnt = 0 then
                s_sm_a <= s_rw; s_sm_b <= s_vys; s_sm_p <= (others => '0');
            elsif s_sm_cnt <= 12 then
                if s_sm_b(0) = '1' then v_sm := ('0' & s_sm_p(27 downto 12)) + ('0' & s_sm_a);
                else v_sm := '0' & s_sm_p(27 downto 12); end if;
                s_sm_p <= v_sm & s_sm_p(11 downto 1);
                s_sm_b <= '0' & s_sm_b(11 downto 1);
            elsif s_sm_cnt = 13 then
                s_rwx <= s_sm_p(27 downto 8);
            end if;
            s_sc_p  <= s_gs(13 downto 4) * (resize(s_gs(13 downto 8), 7) + 1);
            s_sc    <= s_sc_p(15 downto 0);                                 -- K2: 0..7.5 cycles across 1920 px
            s_rw_p  <= s_gr(13 downto 4) * (resize(s_gr(13 downto 8), 7) + 1);
            s_rw    <= s_rw_p(15 downto 0);                                 -- P12: 0..34 cycles down 1080 lines
            s_gain  <= s_k_gain(9 downto 2);
            -- K5 detector zones: change only 12 counts inside the new zone
            v_z := s_k_mode(9 downto 8); v_lo := s_k_mode(7 downto 0);
            -- labelled default: at program load the firmware writes the
            -- initial_value_label's INDEX (0..3), not a 0..1023 position, so
            -- a value below 4 is the detector index itself
            if s_k_mode < 4 then
                s_mode <= s_k_mode(1 downto 0);
            elsif (v_lo >= 12 or v_z = 0) and (v_lo <= 243 or v_z = 3) then
                s_mode <= v_z;
            end if;
            -- K6 ink luma 64..945; the paper is white under dark ink, black under light
            s_inky <= 64 + s_k_ink - shift_right(s_k_ink, 3) - shift_right(s_k_ink, 6);
            if s_k_ink >= 528 then s_paper_blk <= '1'; elsif s_k_ink < 496 then s_paper_blk <= '0'; end if;
            if s_paper_blk = '1' then s_papery <= to_unsigned(64, 10); else s_papery <= to_unsigned(940, 10); end if;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13 <= signed(resize(s_w_edge, 13));

            -- line events
            s_hpar <= not s_hpar;
            if s_prev_avid = '0' and data_in.avid = '1' then
                -- this line writes bank s_wpar and reads the other (the line above)
                s_lw <= s_wpar;
                s_line_par <= s_hpar;
                if s_wpar = '1' then s_cap_par1 <= s_hpar; else s_cap_par0 <= s_hpar; end if;
            end if;
            if s_lw = '1' then s_rowswap <= s_line_par xor s_cap_par0;
            else s_rowswap <= s_line_par xor s_cap_par1; end if;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_lph1 <= s_lph1 + resize(s_rwx & "000", 24);
                s_lph2 <= s_lph2 - resize(s_rwx & "0000", 24);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_hpar <= '0';
                s_wpar <= not s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_fph1 <= s_fph1 + resize(s_k_speed(9 downto 3) & "0000000000", 24);
                    s_fph2 <= s_fph2 + resize(s_k_speed(9 downto 3) & "00000000000", 24) + 9472;
                    -- glide Scale and Rows by 1/8 of the gap per frame
                    v_tg := signed(resize(unsigned'(s_k_scale & "0000"), 15)); v_cur := signed(resize(s_gs, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 3);
                    s_gs <= unsigned(v_cur(13 downto 0));
                    v_tg := signed(resize(unsigned'(s_p_rows & "0000"), 15)); v_cur := signed(resize(s_gr, 15));
                    v_dg := v_tg - v_cur;
                    v_cur := v_cur + shift_right(v_dg, 3);
                    s_gr <= unsigned(v_cur(13 downto 0));
                end if;
                s_lph1 <= s_fph1; s_lph2 <= s_fph2;
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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_q1 <= to_signed(C_SINE(to_integer(s_sa1)), 10);
            s_q2 <= to_signed(C_SINE(to_integer(s_sa2)), 10);
            s_qc1 <= to_signed(C_SINE(to_integer(s_ca1)), 10);
            s_qc2 <= to_signed(C_SINE(to_integer(s_ca2)), 10);
            s_r1 <= s_q1; s_r2 <= s_q2; s_rc1 <= s_qc1; s_rc2 <= s_qc2;
        end if;
    end process;

    -- write side: the input's edges against the write bank's old line (2 lines
    -- up, read at stage 2 by its idle port), stored with the luma; chroma
    -- written at stage 3, luma at 9 (stage-8 word)
    p_wr : process(clk)
        variable v_e : unsigned(8 downto 0);
        variable v_a : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_yc(2) <= s_in_y;
            for i in 3 to 13 loop s_yc(i) <= s_yc(i - 1); end loop;
            s_u2 <= s_in_u; s_v2 <= s_in_v;
            -- 3: input gradient (Push)
            s_gx3 <= f_ad10(s_yc(2), s_yc(3));
            -- 4: luma 2 lines up
            if s_lw = '1' then s_up4 <= unsigned(s_rdY1(11 downto 4)); else s_up4 <= unsigned(s_rdY0(11 downto 4)); end if;
            s_up5 <= s_up4;
            -- 5: gradients (x = yc(4) / up4, x-1 = yc(5) / up5)
            s_wh5  <= f_ad8(s_yc(4)(9 downto 2), s_yc(5)(9 downto 2));
            s_wv5  <= f_ad8(s_yc(4)(9 downto 2), s_up4);
            s_wr15 <= f_ad8(s_yc(4)(9 downto 2), s_up5);
            s_wr25 <= f_ad8(s_yc(5)(9 downto 2), s_up4);
            -- 6: detector (8-bit units)
            case to_integer(s_mode) is
                when 0 => v_e := resize(s_wr15, 9) + resize(s_wr25, 9);
                when 1 => v_e := resize(s_wh5, 9) + resize(s_wh5, 9);
                when 2 => v_e := resize(s_wv5, 9) + resize(s_wv5, 9);
                when others => v_e := resize(s_wh5, 9) + resize(s_wv5, 9);
            end case;
            s_we6 <= v_e;
            -- 7: gain
            s_weg7 <= s_we6 * s_gain;
            -- 8: 4-bit alpha (the read side's alpha / 16) beside the luma
            if s_weg7(16 downto 13) /= "0000" then v_a := "1111"; else v_a := s_weg7(12 downto 9); end if;
            s_word8 <= std_logic_vector(s_yc(7)(9 downto 2)) & std_logic_vector(v_a);
        end if;
    end process p_wr;

    p_addr : process(clk)
        variable v_p : signed(12 downto 0);
        procedure edge(signal src : in signed(12 downto 0); signal addr : out unsigned(10 downto 0)) is
        begin
            if src < C_EDGE then addr <= to_unsigned(C_EDGE, 11);
            elsif src > s_we13 then addr <= s_w_edge;
            else addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_xs(2) <= s_wr_x;
            for i in 3 to 9 loop s_xs(i) <= s_xs(i - 1); end loop;
            -- 2: phase DDAs, reloaded with the line phases outside avid
            if s_avid_sr(0) = '0' then s_ph1 <= s_lph1; s_ph2 <= s_lph2;
            else s_ph1 <= s_ph1 + resize(s_scx, 24); s_ph2 <= s_ph2 + resize(s_scx & '0', 24); end if;
            -- 3: ROM addresses
            s_sa1 <= s_ph1(23 downto 16); s_sa2 <= s_ph2(23 downto 16);
            if s_sw_csep = '1' then s_ca1 <= s_ph1(23 downto 16) + 64; s_ca2 <= s_ph2(23 downto 16) + 96;
            else s_ca1 <= s_ph1(23 downto 16); s_ca2 <= s_ph2(23 downto 16); end if;
            -- 4 ROM, 5 fabric (p_rom); 6: octaves
            if s_sw_two = '1' then
                s_n6 <= resize(s_r1, 11) + resize(shift_right(s_r2, 1), 11);
                s_nc6 <= resize(s_rc1, 11) + resize(shift_right(s_rc2, 1), 11);
            else
                s_n6 <= resize(s_r1, 11); s_nc6 <= resize(s_rc1, 11);
            end if;
            -- 7: amplitude
            s_pn7 <= s_n6 * signed(resize(s_amp, 10));
            s_pnc7 <= s_nc6 * signed(resize(s_amp, 10));
            -- Push offset in 1080-line px -> real px (720p ~x 0.66, SD x 1/2)
            case s_vxs is
                when "011"  => s_gxd(4) <= ('0' & s_gx3(9 downto 1)) + ("000" & s_gx3(9 downto 3)) + ("00000" & s_gx3(9 downto 5));
                when "100"  => s_gxd(4) <= '0' & s_gx3(9 downto 1);
                when others => s_gxd(4) <= s_gx3;
            end case;
            for i in 5 to 7 loop s_gxd(i) <= s_gxd(i - 1); end loop;
            -- 8: address = x + noise (+ the input gradient)
            if s_sw_push = '1' then v_p := signed(resize(s_gxd(7)(9 downto 2), 13)); else v_p := (others => '0'); end if;
            s_aY8 <= signed(resize(s_xs(7), 13)) + resize(shift_right(s_pn7, 8), 13) + v_p;
            s_aC8 <= signed(resize(s_xs(7), 13)) + resize(shift_right(s_pnc7, 8), 13) + v_p;
            -- 9: edge clamp
            edge(s_aY8, s_addrY);
            edge(s_aC8, s_addrC);
            -- 10: luma read addresses: the read bank takes the displaced address,
            --     the write bank reads its old line at the stage-1 x (see p_wr)
            if s_lw = '1' then s_ra0 <= s_addrY; s_ra1 <= s_wr_x;
            else s_ra0 <= s_wr_x; s_ra1 <= s_addrY; end if;
            s_rc10 <= s_addrC;
        end if;
    end process p_addr;

    -- line buffers: luma written at stage 9 (stage-8 word), chroma at 3;
    -- read at 11 (luma also at 3 from the write bank)
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ra0));
            if s_avid_sr(7) = '1' and s_lw = '0' then lbY0(to_integer(s_xs(8))) <= s_word8; end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ra1));
            if s_avid_sr(7) = '1' and s_lw = '1' then lbY1(to_integer(s_xs(8))) <= s_word8; end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_rc10(10 downto 1)));
            if s_avid_sr(1) = '1' and s_lw = '0' then lbU0(to_integer(s_xs(2)(10 downto 1))) <= std_logic_vector(s_u2(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_rc10(10 downto 1)));
            if s_avid_sr(1) = '1' and s_lw = '1' then lbU1(to_integer(s_xs(2)(10 downto 1))) <= std_logic_vector(s_u2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_rc10(10 downto 1)));
            if s_avid_sr(1) = '1' and s_lw = '0' then lbV0(to_integer(s_xs(2)(10 downto 1))) <= std_logic_vector(s_v2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_rc10(10 downto 1)));
            if s_avid_sr(1) = '1' and s_lw = '1' then lbV1(to_integer(s_xs(2)(10 downto 1))) <= std_logic_vector(s_v2(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_w  : std_logic_vector(11 downto 0);
        variable v_e  : unsigned(10 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 12: fetch the read bank (the line above); swap U/V when its
            --     hsync->avid parity differs from this line's
            if s_lw = '0' then
                v_w := s_rdY1;
                if s_rowswap = '1' then s_wu12 <= unsigned(s_rdV1) & "00"; s_wv12 <= unsigned(s_rdU1) & "00";
                else s_wu12 <= unsigned(s_rdU1) & "00"; s_wv12 <= unsigned(s_rdV1) & "00"; end if;
            else
                v_w := s_rdY0;
                if s_rowswap = '1' then s_wu12 <= unsigned(s_rdV0) & "00"; s_wv12 <= unsigned(s_rdU0) & "00";
                else s_wu12 <= unsigned(s_rdU0) & "00"; s_wv12 <= unsigned(s_rdV0) & "00"; end if;
            end if;
            s_wy12 <= unsigned(v_w(11 downto 4)) & "10";
            s_wa12 <= unsigned(v_w(3 downto 0));
            -- 13: edges of the displaced line above against the input line
            --     (input x = yc(12), x-1 = yc(13); displaced x = wy12, x-1 = pv13)
            s_pv13 <= s_wy12;
            s_r1_13 <= f_ad10(s_yc(12), s_pv13);
            s_r2_13 <= f_ad10(s_yc(13), s_wy12);
            s_gh13  <= f_ad10(s_wy12, s_pv13);
            s_gv13  <= f_ad10(s_yc(12), s_wy12);
            s_y13 <= s_wy12; s_u13 <= s_wu12; s_v13 <= s_wv12;
            s_a13 <= s_wa12 & s_wa12;                                       -- stored alpha x17 (0..255)
            -- 14: detector
            case to_integer(s_mode) is
                when 0 => v_e := resize(s_r1_13, 11) + resize(s_r2_13, 11);
                when 1 => v_e := resize(s_gh13, 11) + resize(s_gh13, 11);
                when 2 => v_e := resize(s_gv13, 11) + resize(s_gv13, 11);
                when others => v_e := resize(s_gh13, 11) + resize(s_gv13, 11);
            end case;
            s_e14 <= v_e;
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13; s_a14 <= s_a13;
            -- 15: gain
            s_eg15 <= s_e14 * s_gain;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14; s_a15 <= s_a14;
            -- 16: ink alpha (Warp-Edge: e * gain / 128, Edge-Warp: the stored
            --     alpha); ground (the displaced picture or plain paper)
            if s_sw_edgefirst = '1' then
                s_al16 <= s_a15;
            elsif s_eg15(18 downto 15) /= "0000" then
                s_al16 <= to_unsigned(255, 8);
            else
                s_al16 <= s_eg15(14 downto 7);
            end if;
            if s_sw_paper = '1' then s_by16 <= s_papery; else s_by16 <= s_y15; end if;
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17: ink - ground; chroma about neutral
            s_dy17 <= signed(resize(s_inky, 11)) - signed(resize(s_by16, 11));
            s_du17 <= signed(resize(s_u16, 11)) - 512;
            s_dv17 <= signed(resize(s_v16, 11)) - 512;
            s_aly17 <= s_al16;
            if s_sw_paper = '1' then s_alc17 <= s_al16; else s_alc17 <= (others => '0'); end if;
            s_by17 <= s_by16; s_u17 <= s_u16; s_v17 <= s_v16;
            -- 18: products, alpha split hi/lo
            s_hy18 <= s_dy17 * signed('0' & s_aly17(7 downto 4));
            s_ly18 <= s_dy17 * signed('0' & s_aly17(3 downto 0));
            s_hu18 <= s_du17 * signed('0' & s_alc17(7 downto 4));
            s_lu18 <= s_du17 * signed('0' & s_alc17(3 downto 0));
            s_hv18 <= s_dv17 * signed('0' & s_alc17(7 downto 4));
            s_lv18 <= s_dv17 * signed('0' & s_alc17(3 downto 0));
            s_by18 <= s_by17; s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19: d * alpha
            s_py19 <= shift_left(resize(s_hy18, 21), 4) + resize(s_ly18, 21);
            s_pu19 <= shift_left(resize(s_hu18, 21), 4) + resize(s_lu18, 21);
            s_pv19 <= shift_left(resize(s_hv18, 21), 4) + resize(s_lv18, 21);
            s_by19 <= s_by18; s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: lerp toward the ink; on paper the chroma follows the alpha
            v_t := signed(resize(s_by19, 12)) + resize(shift_right(s_py19, 8), 12);
            s_y20 <= unsigned(v_t(9 downto 0));
            if s_sw_paper = '1' then
                v_t := 512 + resize(shift_right(s_pu19, 8), 12); s_u20 <= unsigned(v_t(9 downto 0));
                v_t := 512 + resize(shift_right(s_pv19, 8), 12); s_v20 <= unsigned(v_t(9 downto 0));
            else
                s_u20 <= s_u19; s_v20 <= s_v19;
            end if;
            -- 21: neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(19) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            -- 22..23: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y21; s_g2y <= s_g1y;
            s_g1u <= s_u21; s_g2u <= s_g1u;
            s_g1v <= s_v21; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y21, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u21, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v21, 12);
            s_y23 <= v_sy(11 downto 2); s_u23 <= v_su(11 downto 2); s_v23 <= v_sv(11 downto 2);
            -- 24: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            if s_y23 < 64 then v_d := (others => '0'); else v_d := s_y23 - 64; end if;
            if s_avid_sr(22) = '1' then
                s_y24 <= s_y23 - shift_right(v_d, 3) - shift_right(v_d, 4);
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture ridgeline;
