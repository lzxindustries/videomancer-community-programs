-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Taffy: slider-scanned horizontal stretch bands.
--
-- v0.4.2 (2026-10-17): resolution scaling.  Band height, feather and the
-- centre dead zone are measured in virtual 1080-line units, stepped from the
-- video timing ID (registers_in(8)): 1.0 at 1080p (identical to before), 1.5
-- at 720p, and at SD 2.0 across (a 1440-wide 4:3 window) and 1080/486 per
-- frame line.  Each line's distance to the band is scaled from field lines to
-- virtual units in the line sequencer (one blanking-time multiply), and the
-- dead-zone half-width is converted to real pixels once per frame, so a band
-- and dead zone keep the same share of the screen at SD, 720p and 1080p.
-- Interlaced fields share the per-line envelope, so both fields use the same
-- field-line distance (no half-line offset -> no field-to-field flicker in
-- the feather).  The stretch itself is a ratio and already scaled; the 12-px
-- edge clamp stays in real pixels (it hides the source's blanking columns).
--
-- P12 selects a scanline (100% = top, 0% = bottom). Lines inside the selected
-- band grow an envelope toward full effect at the Pull rate; every other line
-- relaxes back toward zero at the Relax rate. Dragging the slider therefore
-- leaves a trail of still-stretched lines slowly snapping back behind it.
--
-- S11 picks how a line answers the band:
--   Sustain   the envelope CHASES the band -- full while the slider is on the
--             line, receding once it leaves. A quick pass either snaps the line
--             to full (fast Pull) or never gets it there (slow Pull), so the
--             stretch itself is rarely something you watch happen.
--   One Shot  the band TRIGGERS the line and lets go. However briefly the
--             slider touched it, the line grows to full and recedes on its own,
--             so a sweep leaves a wake of complete stretches behind it. Re-arms
--             only once the band leaves, so parking the slider fires each line
--             once instead of throbbing.
--
-- The envelope drives a horizontal magnification about the centre column: the
-- picture in the band is pulled outward, away from x = width/2, so the pixels
-- next to the centre smear out to the screen edges. A dead zone of +-Centre px
-- either side of the centre is left untouched -- widen it and the stretch
-- starts further out.
--
--   env(y)      -> chases 255 inside the band / 0 outside (Pull / Relax rates)
--   inv         =  256 - env*Depth        (Stretch:  <256 -> magnify)
--               =  256 + env*Depth*4      (Squeeze:  >256 -> shrink)
--   src_dist(x) =  Centre + (|x-c| - Centre) * inv/256
--   out(x, y)   =  in(c +- src_dist, y - 1)
--
-- Controls:
--   K1 Pull    how fast a line grows to full stretch (quadratic: 1..255 env
--              steps per frame, so the low end crawls for seconds)
--   K2 Relax   how fast it recedes again (same curve)
--   K3 Height  band thickness, 1..255 lines (1080-line units)
--   K4 Centre  half-width of the untouched centre dead zone, 0..511 px
--              (1080p pixels; a 1440-wide virtual window at SD)
--   K5 Depth   magnification at full envelope (up to 64x at the top end)
--   K6 Soft    feathers the band edges over 1..128 lines (1080-line units). In
--              Sustain that is a
--              lens-like bulge instead of a rectangular slab; in One Shot the
--              feather is the trigger's reach, so lines fire before the band
--              proper arrives and the wake runs ahead of the slider
--   S7 Polarity  Stretch (pull outward) / Squeeze (pull inward)
--   S8 Hold      freeze every envelope: paint a shape with the slider, then
--                latch it and let go (also freezes a One Shot mid-pulse)
--   S9 Steps     quantize the envelope to 4 levels -> terraced bands
--   S10 Bypass
--   S11 Mode     Sustain / One Shot (see above)
--   P12 Position the scanned line
--
-- No per-pixel multiply. The read address is a run-along DDA (sidewinder's
-- wrapped-pointer idiom): the accumulator is seeded once per line during
-- horizontal blanking and then adds either 256 (inside the dead zone, 1:1) or
-- inv (outside it) per pixel. The dead-zone test is registered one pixel ahead
-- so the per-pixel path is just mux -> add, never compare -> mux -> add.
--
-- All envelope and geometry math runs in step sequencers during horizontal /
-- vertical blanking, so every multiply has registered inputs and outputs (see
-- ziffern memory: combinational mults feeding latched signals are still
-- STA-timed).
--
-- In Stretch the source always lands between x and the centre, so reads can
-- never leave the line -- only Squeeze runs off the end. Every read is clamped
-- to [C_EDGE, width-1-C_EDGE], so the reveal is the line's own edge colour
-- drawn outward (catalogue-combo edge clamp, HW-preferred over an average
-- matte) and the source's black blanking columns never reach the screen.
--
-- Buffer = inlined dual-bank ping-pong BRAM (from sidewinder): each line
-- WRITES its own parity bank, READS the other (the line above). Y full 2048
-- deep; U/V 2:1 decimated. ~22 EBR, + 5 EBR for the 2048x10 envelope.
--
-- Pipeline (rising-edge clocks, data_in -> data_out):
--   E1  p_ctrl: register inputs, pixel_x / acc counters, parity, line + frame
--       step sequencers
--   E2  p_rdaddr: edge-clamp the accumulator -> registered read addr; write
--       of the live pixel lands
--   E3  bank reads registered; capture rbank
--   E4  p_wet: bank select + blanking gate -> data_out (combinational bypass
--       mux)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture taffy of program_top is
    -- data_in -> data_out latency is 4 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 4;

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_Y_AW          : integer := 11;                  -- 2048 (full)
    constant C_UV_AW         : integer := 10;                  -- 1024 (2:1 dec)
    constant C_TOTAL_LATENCY : integer := 4;

    -- Smallest inv (largest magnification): 256/4 = 64x.
    constant C_INV_MIN       : integer := 4;

    -- Edge clamp. Squeeze pulls the picture in from the borders; a read that
    -- runs off the line used to land on the source's edge column -- a black
    -- blanking column on capture sources -> a black bar. Reads within C_EDGE
    -- columns of either border are redirected to that inset column, so the
    -- reveal is the line's own edge colour and the blanking columns never
    -- show (12 = the catalogue-combo value, wide enough for capture sources).
    constant C_EDGE          : integer := 12;

    --------------------------------------------------------------------------
    -- Resolution scaling: per-mode steps from the video timing ID (the
    -- mondrian pilot's tables).
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

    --------------------------------------------------------------------------
    -- Inlined dual-bank line buffers
    --------------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(C_DW - 1 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(C_DW - 1 downto 0);

    signal lbY0, lbY1 : t_lb_y  := (others => (others => '0'));
    signal lbU0, lbU1 : t_lb_uv := (others => (others => '0'));
    signal lbV0, lbV1 : t_lb_uv := (others => (others => '0'));

    signal s_rdY0, s_rdY1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Envelope memory: per scanline, {state[9:8], level[7:0]}, 1W1R, addressed
    -- by the line sequencer only (read at step 0, written back at step 5).
    --------------------------------------------------------------------------
    type t_env is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal env_mem : t_env := (others => (others => '0'));

    signal s_env_addr : unsigned(10 downto 0) := (others => '0');
    signal s_env_rd   : std_logic_vector(9 downto 0) := (others => '0');
    signal s_env_wd   : std_logic_vector(9 downto 0) := (others => '0');
    signal s_env_we   : std_logic := '0';

    -- One Shot per-line state. Sustain ignores the level rules but still keeps
    -- the state field honest (armed when the band is off the line, spent when
    -- it is on it) so flipping S11 mid-performance never fires every line at
    -- once or strands a line mid-pulse.
    constant C_ST_ARMED : unsigned(1 downto 0) := "00";  -- at rest, will fire
    constant C_ST_ATK   : unsigned(1 downto 0) := "01";  -- growing to full
    constant C_ST_DEC   : unsigned(1 downto 0) := "10";  -- receding to nothing
    constant C_ST_SPENT : unsigned(1 downto 0) := "11";  -- done; re-arms when
                                                         -- the band leaves

    --------------------------------------------------------------------------
    -- Input registers / position tracking (E1)
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_in_avid              : std_logic := '0';

    signal s_pixel_x      : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_par          : std_logic := '0';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_had_avid     : std_logic := '0';

    signal s_line_width   : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_line_y       : unsigned(10 downto 0) := (others => '0');
    signal s_active_h     : unsigned(10 downto 0) := to_unsigned(1080, 11);

    -- Write column: the pixel index of s_in_y (s_pixel_x one clock late), so
    -- pixel k lands at address k and a 2:1 chroma address k>>1 holds k's own
    -- pair.
    signal s_wr_x         : unsigned(C_Y_AW - 1 downto 0) := (others => '0');

    -- Pre-registered right-edge bound (mercurial/redshift lesson: per-pixel
    -- compares must see a registered bound, never `width - const` inline).
    signal s_w_edge  : unsigned(C_Y_AW - 1 downto 0) := (others => '0');

    -- Run-along read accumulator: 14.8 fixed, integer part = s_acc(21 downto 8).
    -- Kept as narrow as the worst case allows (Squeeze at full Depth reaches
    -- about -1.05M .. +1.57M) -- every bit here is a carry cell in the read
    -- address comparator, which is the design's critical path.
    signal s_acc      : signed(21 downto 0) := (others => '0');
    signal s_inz      : std_logic := '0';           -- next pixel is in the dead zone
    signal s_inv_step : signed(12 downto 0) := to_signed(256, 13);

    --------------------------------------------------------------------------
    -- Per-frame step sequencer (vertical blanking)
    --------------------------------------------------------------------------
    signal s_vstep    : unsigned(2 downto 0) := "111";
    signal s_p12d     : unsigned(9 downto 0) := (others => '0');
    signal s_atk_prod : unsigned(19 downto 0) := (others => '0');
    signal s_rel_prod : unsigned(19 downto 0) := (others => '0');
    signal s_tgt_prod : unsigned(20 downto 0) := (others => '0');
    signal s_atk      : unsigned(7 downto 0) := to_unsigned(1, 8);
    signal s_rel      : unsigned(7 downto 0) := to_unsigned(1, 8);
    signal s_target   : unsigned(10 downto 0) := (others => '0');
    signal s_half     : unsigned(6 downto 0) := (others => '0');
    signal s_soft_sh  : unsigned(2 downto 0) := (others => '0');
    signal s_depth    : unsigned(7 downto 0) := (others => '0');
    signal s_c        : unsigned(10 downto 0) := (others => '0');
    signal s_cwc      : unsigned(10 downto 0) := (others => '0');
    signal s_lo       : unsigned(10 downto 0) := (others => '0');
    signal s_hi       : unsigned(10 downto 0) := (others => '0');
    signal s_lo_m1    : signed(11 downto 0) := (others => '0');   -- lo - 1

    -- Resolution scaling (latched per field at vsync): s_xs = virtual x units
    -- per pixel (Q.1), s_ys = virtual lines per field line (Q4.8). s_cz is
    -- the K4 dead-zone half-width converted from virtual units to real px.
    signal s_xs       : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys       : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_cz       : unsigned(8 downto 0)  := (others => '0');

    --------------------------------------------------------------------------
    -- Per-line step sequencer (horizontal blanking)
    --------------------------------------------------------------------------
    signal s_lstep    : unsigned(3 downto 0) := "1111";
    signal s_seq_y    : unsigned(10 downto 0) := (others => '0');
    signal s_dist     : unsigned(10 downto 0) := (others => '0');
    signal s_dist8    : unsigned(7 downto 0)  := (others => '0');  -- field lines, sat 255
    signal s_dist_v   : unsigned(11 downto 0) := (others => '0');  -- virtual lines
    signal s_over     : signed(12 downto 0) := (others => '0');
    signal s_env_r2   : unsigned(7 downto 0) := (others => '0');
    signal s_st_r2    : unsigned(1 downto 0) := C_ST_ARMED;
    signal s_tgt      : unsigned(7 downto 0) := (others => '0');
    signal s_env_new  : unsigned(7 downto 0) := (others => '0');
    signal s_st_new   : unsigned(1 downto 0) := C_ST_ARMED;
    signal s_env_q    : unsigned(7 downto 0) := (others => '0');
    signal s_eff_prod : unsigned(15 downto 0) := (others => '0');
    signal s_inv      : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_lo_prod  : unsigned(22 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Control (decoded from registers)
    --------------------------------------------------------------------------
    signal s_pull_k   : unsigned(C_DW - 1 downto 0);
    signal s_relax_k  : unsigned(C_DW - 1 downto 0);
    signal s_height_k : unsigned(C_DW - 1 downto 0);
    signal s_centre_k : unsigned(C_DW - 1 downto 0);
    signal s_depth_k  : unsigned(C_DW - 1 downto 0);
    signal s_soft_k   : unsigned(C_DW - 1 downto 0);
    signal s_squeeze  : std_logic;
    signal s_hold     : std_logic;
    signal s_steps    : std_logic;
    signal s_bypass   : std_logic;
    signal s_mode     : std_logic;
    signal s_pos      : unsigned(C_DW - 1 downto 0);

    --------------------------------------------------------------------------
    -- Registered read address (E2)
    --------------------------------------------------------------------------
    signal s_rd_addr_y  : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_rd_addr_uv : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_r_rbank    : std_logic := '0';

    --------------------------------------------------------------------------
    -- Stage E3 capture
    --------------------------------------------------------------------------
    signal s_a_rbank : std_logic := '0';

    --------------------------------------------------------------------------
    -- Wet result (E3)
    --------------------------------------------------------------------------
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Dry / sync delay shift registers
    --------------------------------------------------------------------------
    type t_data_shift is array (0 to C_TOTAL_LATENCY - 1)
        of std_logic_vector(C_DW - 1 downto 0);
    type t_bit_shift  is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_y_sr     : t_data_shift := (others => (others => '0'));
    signal s_u_sr     : t_data_shift := (others => (others => '0'));
    signal s_v_sr     : t_data_shift := (others => (others => '0'));
    signal s_hsync_sr : t_bit_shift  := (others => '1');
    signal s_vsync_sr : t_bit_shift  := (others => '1');
    signal s_field_sr : t_bit_shift  := (others => '1');
    signal s_avid_sr  : t_bit_shift  := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Control decode
    --------------------------------------------------------------------------
    s_pull_k   <= unsigned(registers_in(0));
    s_relax_k  <= unsigned(registers_in(1));
    s_height_k <= unsigned(registers_in(2));
    s_centre_k <= unsigned(registers_in(3));
    s_depth_k  <= unsigned(registers_in(4));
    s_soft_k   <= unsigned(registers_in(5));
    s_squeeze  <= registers_in(6)(0);
    s_hold     <= registers_in(6)(1);
    s_steps    <= registers_in(6)(2);
    s_bypass   <= registers_in(6)(3);
    s_mode     <= registers_in(6)(4);
    s_pos      <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- E2: read address from the DDA accumulator, REGISTERED. Stretch keeps the
    -- source between x and the centre, so only Squeeze ever runs off the line,
    -- and those reads clamp to the inset edge columns (the edge colour smears).
    --
    -- This range test is a 14-bit carry chain, and letting it run on into the
    -- line buffer's address pins made one combinational cone from s_acc all the
    -- way to RAM.RADDR (15.2 ns, 12 of it routing -- hd_hdmi stuck at 65 MHz).
    -- Registering here splits it into acc -> compare -> reg and reg -> RAM, and
    -- costs one clock of latency that the dry shift register absorbs.
    --------------------------------------------------------------------------
    p_rdaddr : process(clk)
        variable v_int  : signed(13 downto 0);
        variable v_rd   : unsigned(C_Y_AW - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_int  := s_acc(21 downto 8);
            -- Both bounds are compares against pre-registered values, kept
            -- parallel: neither extends the acc -> RAM.RADDR chain.
            if v_int < to_signed(C_EDGE, 14) then
                v_rd := to_unsigned(C_EDGE, C_Y_AW);
            elsif v_int > signed(resize(s_w_edge, 14)) then
                v_rd := s_w_edge;
            else
                v_rd := unsigned(v_int(C_Y_AW - 1 downto 0));
            end if;
            s_rd_addr_y  <= v_rd;
            s_rd_addr_uv <= v_rd(C_Y_AW - 1 downto 1);  -- >> 1 decimate
            -- read parity travels with the address so the bank select stays
            -- aligned with the data it selects
            s_r_rbank    <= not s_par;
        end if;
    end process p_rdaddr;

    --------------------------------------------------------------------------
    -- E1: input registers, counters, parity, and both step sequencers.
    -- Single driver for s_acc, s_lstep, s_vstep and the envelope port.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_nx    : unsigned(C_Y_AW - 1 downto 0);
        variable v_step  : signed(12 downto 0);
        variable v_diff  : signed(11 downto 0);
        variable v_soft  : unsigned(19 downto 0);
        variable v_a     : unsigned(8 downto 0);
        variable v_b     : unsigned(8 downto 0);
        variable v_eff   : unsigned(7 downto 0);
        variable v_lohi  : signed(21 downto 0);
        variable v_prod  : signed(21 downto 0);
        variable v_atk   : unsigned(7 downto 0);
        variable v_rel   : unsigned(7 downto 0);
        variable v_dp    : unsigned(19 downto 0);
        variable v_czm   : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y    <= unsigned(data_in.y);
            s_in_u    <= unsigned(data_in.u);
            s_in_v    <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            s_env_we <= '0';

            ------------------------------------------------------------------
            -- Per-pixel: the write column counter and the read accumulator
            -- run along together. The dead-zone test is registered one pixel
            -- ahead so the accumulator path is only mux -> add.
            ------------------------------------------------------------------
            if data_in.avid = '1' then
                s_had_avid <= '1';
                s_pixel_x  <= s_pixel_x + 1;

                if s_inz = '1' then
                    v_step := to_signed(256, 13);   -- dead zone: 1:1
                else
                    v_step := s_inv_step;
                end if;
                s_acc <= s_acc + resize(v_step, 22);

                v_nx := s_pixel_x + 1;
                if v_nx >= s_lo and v_nx < s_hi then
                    s_inz <= '1';
                else
                    s_inz <= '0';
                end if;
            end if;

            s_wr_x <= s_pixel_x;

            ------------------------------------------------------------------
            -- Per-line sequencer: one registered step per clock during
            -- horizontal blanking; done long before active video starts.
            -- Updates env(s_seq_y) and seeds the DDA for the next line.
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>       -- issue the envelope read; distance to target
                    s_env_addr <= s_seq_y;
                    v_diff     := signed(resize(s_seq_y, 12))
                                - signed(resize(s_target, 12));
                    s_dist     <= unsigned(resize(abs(v_diff), 11));
                    s_lstep    <= s_lstep + 1;
                when 1 =>       -- saturate: Height <= 127 + Soft <= 128 means
                    -- any distance >= 255 is off the band at every setting
                    -- (and virtual distance >= field distance in every mode)
                    if s_dist(10 downto 8) /= "000" then
                        s_dist8 <= to_unsigned(255, 8);
                    else
                        s_dist8 <= s_dist(7 downto 0);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>       -- field lines -> virtual 1080-line units
                    -- (x1 at 1080p, so the band there is unchanged)
                    v_dp     := s_dist8 * s_ys;
                    s_dist_v <= v_dp(19 downto 8);
                    s_lstep  <= s_lstep + 1;
                when 3 =>       -- how far outside the flat top of the band
                    s_over  <= signed(resize(s_dist_v, 13))
                             - signed(resize(s_half, 13));
                    s_lstep <= s_lstep + 1;
                when 4 =>       -- envelope read lands; shape the band target
                    s_env_r2 <= unsigned(s_env_rd(7 downto 0));
                    s_st_r2  <= unsigned(s_env_rd(9 downto 8));
                    if s_over <= 0 then
                        s_tgt <= to_unsigned(255, 8);
                    else
                        -- Soft = 0 shifts by 8 -> any overshoot kills it dead;
                        -- Soft = 7 shifts by 1 -> fades out over 128 lines.
                        v_soft := shift_left(resize(unsigned(s_over(11 downto 0)), 20),
                                             8 - to_integer(s_soft_sh));
                        if v_soft > to_unsigned(255, 20) then
                            s_tgt <= (others => '0');
                        else
                            s_tgt <= to_unsigned(255, 8) - v_soft(7 downto 0);
                        end if;
                    end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>       -- advance this line's envelope
                    v_a := ('0' & s_env_r2) + ('0' & s_atk);   -- Pull  step up
                    v_b := ('0' & s_env_r2) - ('0' & s_rel);   -- Relax step down
                    if s_mode = '0' then
                        --------------------------------------------------------
                        -- Sustain: chase the band. Holds at full for as long as
                        -- the slider stays, so a line only ever recedes once the
                        -- band has moved off it.
                        --------------------------------------------------------
                        if s_env_r2 < s_tgt then
                            if v_a > ('0' & s_tgt) then
                                s_env_new <= s_tgt;
                            else
                                s_env_new <= v_a(7 downto 0);
                            end if;
                        elsif s_env_r2 > s_tgt then
                            if v_b(8) = '1' or v_b < ('0' & s_tgt) then
                                s_env_new <= s_tgt;
                            else
                                s_env_new <= v_b(7 downto 0);
                            end if;
                        else
                            s_env_new <= s_tgt;
                        end if;
                        -- keep the state field meaningful for a mode flip
                        if s_tgt = 0 then
                            s_st_new <= C_ST_ARMED;
                        else
                            s_st_new <= C_ST_SPENT;
                        end if;
                    else
                        --------------------------------------------------------
                        -- One Shot: the band TRIGGERS the line and then lets go.
                        -- The line grows to full and recedes on its own however
                        -- briefly the slider touched it, so every line gets the
                        -- whole stretch instead of snapping to full and holding.
                        -- It re-arms only once the band has left, so parking the
                        -- slider fires each line once rather than throbbing.
                        --------------------------------------------------------
                        case to_integer(s_st_r2) is
                            when 0 =>       -- ARMED
                                if s_tgt /= 0 then
                                    -- saturate: a line flipped over from
                                    -- Sustain can still be part-stretched
                                    if v_a(8) = '1' then
                                        s_env_new <= to_unsigned(255, 8);
                                    else
                                        s_env_new <= v_a(7 downto 0);
                                    end if;
                                    s_st_new <= C_ST_ATK;
                                else
                                    s_st_new <= C_ST_ARMED;
                                    -- normally already 0; this also walks a line
                                    -- down gracefully if S11 flipped mid-stretch
                                    if v_b(8) = '1' then
                                        s_env_new <= (others => '0');
                                    else
                                        s_env_new <= v_b(7 downto 0);
                                    end if;
                                end if;
                            when 1 =>       -- ATTACK
                                if v_a >= to_unsigned(255, 9) then
                                    s_env_new <= to_unsigned(255, 8);
                                    s_st_new  <= C_ST_DEC;
                                else
                                    s_env_new <= v_a(7 downto 0);
                                    s_st_new  <= C_ST_ATK;
                                end if;
                            when 2 =>       -- DECAY
                                if v_b(8) = '1' or v_b(7 downto 0) = 0 then
                                    s_env_new <= (others => '0');
                                    s_st_new  <= C_ST_SPENT;
                                else
                                    s_env_new <= v_b(7 downto 0);
                                    s_st_new  <= C_ST_DEC;
                                end if;
                            when others =>  -- SPENT
                                s_env_new <= (others => '0');
                                if s_tgt = 0 then
                                    s_st_new <= C_ST_ARMED;
                                else
                                    s_st_new <= C_ST_SPENT;
                                end if;
                        end case;
                    end if;
                    s_lstep <= s_lstep + 1;
                when 6 =>       -- write back (Hold freezes it) + quantize
                    s_env_wd <= std_logic_vector(s_st_new)
                              & std_logic_vector(s_env_new);
                    s_env_we <= not s_hold;
                    if s_steps = '1' then
                        -- replicate the top 2 bits -> 0 / 85 / 170 / 255
                        s_env_q <= s_env_new(7 downto 6) & s_env_new(7 downto 6)
                                 & s_env_new(7 downto 6) & s_env_new(7 downto 6);
                    else
                        s_env_q <= s_env_new;
                    end if;
                    s_lstep <= s_lstep + 1;
                when 7 =>       -- envelope -> effect amount
                    s_eff_prod <= s_env_q * s_depth;
                    s_lstep    <= s_lstep + 1;
                when 8 =>       -- effect amount -> source step
                    v_eff := s_eff_prod(15 downto 8);
                    if s_squeeze = '1' then
                        s_inv <= to_unsigned(256, 12)
                               + resize(shift_left(resize(v_eff, 12), 2), 12);
                    elsif v_eff > to_unsigned(256 - C_INV_MIN, 8) then
                        s_inv <= to_unsigned(C_INV_MIN, 12);
                    else
                        s_inv <= to_unsigned(256, 12) - resize(v_eff, 12);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 9 =>       -- seed term for x = 0
                    s_lo_prod <= s_lo * s_inv;
                    s_lstep   <= s_lstep + 1;
                when 10 =>      -- load the DDA: acc(0) = (lo - lo*inv/256 - 1) << 8
                    -- The -1 is because the accumulator steps on the first
                    -- pixel before its address is read, so output pixel x
                    -- samples src(x+1) - 1: with inv = 256 that is exactly x.
                    v_lohi := shift_left(resize(s_lo_m1, 22), 8);
                    v_prod := signed(resize(s_lo_prod, 22));
                    s_acc      <= v_lohi - v_prod;
                    s_inv_step <= signed(resize(s_inv, 13));
                    if s_lo = 0 then
                        s_inz <= '1';
                    else
                        s_inz <= '0';
                    end if;
                    s_lstep <= s_lstep + 1;
                when others =>  -- idle
                    null;
            end case;

            ------------------------------------------------------------------
            -- Per-frame sequencer (vertical blanking): rate curves, the
            -- selected line, and the centre geometry.
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    -- P12 up (1023) = top line, down (0) = bottom line
                    s_p12d     <= to_unsigned(1023, 10) - s_pos;
                    s_atk_prod <= s_pull_k * s_pull_k;
                    s_rel_prod <= s_relax_k * s_relax_k;
                    s_vstep    <= s_vstep + 1;
                when 1 =>
                    s_tgt_prod <= s_p12d * s_active_h;
                    -- quadratic so the slow end of both rates crawls
                    v_atk := s_atk_prod(19 downto 12);
                    v_rel := s_rel_prod(19 downto 12);
                    if v_atk = 0 then
                        s_atk <= to_unsigned(1, 8);
                    else
                        s_atk <= v_atk;
                    end if;
                    if v_rel = 0 then
                        s_rel <= to_unsigned(1, 8);
                    else
                        s_rel <= v_rel;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_target  <= s_tgt_prod(20 downto 10);
                    s_c       <= resize(s_line_width(C_Y_AW - 1 downto 1), 11);
                    s_half    <= s_height_k(9 downto 3);
                    s_soft_sh <= s_soft_k(9 downto 7);
                    s_depth   <= s_depth_k(9 downto 2);
                    -- dead zone: K4 in virtual units -> real px (x 2/xs)
                    case to_integer(s_xs) is
                        when 4 =>       -- SD: 2 units per pixel
                            s_cz <= '0' & s_centre_k(9 downto 2);
                        when 3 =>       -- 720p: 1.5 units per pixel
                            v_czm := s_centre_k(9 downto 1) * to_unsigned(171, 8);
                            s_cz  <= v_czm(16 downto 8);
                        when others =>  -- 1080: 1:1
                            s_cz <= s_centre_k(9 downto 1);
                    end case;
                    s_vstep   <= s_vstep + 1;
                when 3 =>
                    -- keep the dead zone strictly inside the half-width, else
                    -- lo hits 0 and the whole line goes 1:1
                    if s_c = 0 then
                        s_cwc <= (others => '0');
                    elsif resize(s_cz, 11) >= s_c then
                        s_cwc <= s_c - 1;
                    else
                        s_cwc <= resize(s_cz, 11);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_lo    <= s_c - s_cwc;
                    s_hi    <= s_c + s_cwc;
                    -- right-edge clamp bound, pre-registered for the per-pixel
                    -- compare; degenerate widths just disable the right zone
                    if s_line_width > to_unsigned(2 * C_EDGE + 1, C_Y_AW) then
                        s_w_edge <= s_line_width - (C_EDGE + 1);
                    else
                        s_w_edge <= s_line_width;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    s_lo_m1 <= signed(resize(s_lo, 12)) - 1;
                    s_vstep <= s_vstep + 1;
                when others =>
                    null;
            end case;

            -- hsync falling edge = new line: latch width, reset counters, flip
            -- parity, and kick the line sequencer for the line that just ended
            -- (its envelope drives the next line -- a one-line shift nobody
            -- can see, and it keeps blanking lines out of the envelope).
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_pixel_x /= 0 then
                    s_line_width <= s_pixel_x;
                end if;
                s_pixel_x  <= (others => '0');
                s_par      <= not s_par;
                s_had_avid <= '0';
                if s_had_avid = '1' then
                    s_seq_y  <= s_line_y;
                    s_line_y <= s_line_y + 1;
                    s_lstep  <= (others => '0');
                end if;
            end if;

            -- vsync falling edge = new frame: latch the active height, re-sync
            -- parity, kick the frame sequencer, and park the DDA at identity so
            -- the top line never renders with the bottom line's envelope.
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_line_y /= 0 then
                    s_active_h <= s_line_y;
                end if;
                s_line_y   <= (others => '0');
                s_par      <= '0';
                s_vstep    <= (others => '0');
                -- per-mode scaling steps (idempotent: safe on serrated vsync)
                s_xs       <= f_vstep_x(registers_in(8)(3 downto 0));
                s_ys       <= f_vstep_y(registers_in(8)(3 downto 0));
                s_acc      <= to_signed(-256, 22);   -- identity seed
                s_inv_step <= to_signed(256, 13);
                s_inz      <= '0';
            end if;

            -- Dry / sync shift registers
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Envelope memory port: canonical 1W1R, shared address (the sequencer
    -- reads and writes the same line), pre-muxed write data.
    --------------------------------------------------------------------------
    p_env : process(clk)
    begin
        if rising_edge(clk) then
            s_env_rd <= env_mem(to_integer(s_env_addr));
            if s_env_we = '1' then
                env_mem(to_integer(s_env_addr)) <= s_env_wd;
            end if;
        end if;
    end process p_env;

    --------------------------------------------------------------------------
    -- E3: carry the read-side context one more stage so it lands with the bank
    -- read data. Read uses the live (already flipped) parity -> the line above.
    --------------------------------------------------------------------------
    p_stage_a : process(clk)
    begin
        if rising_edge(clk) then
            s_a_rbank <= s_r_rbank;
        end if;
    end process p_stage_a;

    --------------------------------------------------------------------------
    -- E4: bank select -> wet result, gated to neutral outside active video.
    -- The line buffer is read on every clock, so without the gate the blanking
    -- interval carries picture (edge pixels) where the encoder expects its
    -- neutral colour reference (blanking-interval contract; no sim sees it).
    -- s_avid_sr(C_TOTAL_LATENCY - 2) lands in the same register stage as
    -- data_out.avid = s_avid_sr(C_TOTAL_LATENCY - 1).
    --------------------------------------------------------------------------
    p_wet : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_TOTAL_LATENCY - 2) = '0' then
                s_wet_y <= to_unsigned(64, C_DW);
                s_wet_u <= to_unsigned(512, C_DW);
                s_wet_v <= to_unsigned(512, C_DW);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                s_wet_u <= unsigned(s_rdU1);
                s_wet_v <= unsigned(s_rdV1);
            else
                s_wet_y <= unsigned(s_rdY0);
                s_wet_u <= unsigned(s_rdU0);
                s_wet_v <= unsigned(s_rdV0);
            end if;
        end if;
    end process p_wet;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank (single driver, single write port).
    -- Write = live E1-registered pixel at its own index (s_wr_x) into this
    -- line's parity bank; read = registered E2 address, registered out.
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_rd_addr_y));
            if s_in_avid = '1' and s_par = '0' then
                lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y);
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_rd_addr_y));
            if s_in_avid = '1' and s_par = '1' then
                lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y);
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '0' then
                lbU0(to_integer(s_wr_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_u);
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '1' then
                lbU1(to_integer(s_wr_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_u);
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '0' then
                lbV0(to_integer(s_wr_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_v);
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '1' then
                lbV1(to_integer(s_wr_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_v);
            end if;
        end if;
    end process p_lbV1;

    --------------------------------------------------------------------------
    -- Output mux. env = 0 reproduces the input exactly (inv = 256, pixel x
    -- reads x) apart from the C_EDGE clamp columns, so there is no dry/wet
    -- mix to carry.
    --------------------------------------------------------------------------
    data_out.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    data_out.y <= s_y_sr(C_TOTAL_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_wet_y);
    data_out.u <= s_u_sr(C_TOTAL_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_wet_u);
    data_out.v <= s_v_sr(C_TOTAL_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_wet_v);

end architecture taffy;
