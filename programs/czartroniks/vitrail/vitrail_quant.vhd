-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Vitrail palette quantizer: luma posterized to L levels that land exactly
-- on black (64) and white (~940), chroma rounded per axis to a multiple of
-- 2^k after a saturation gain (mid-tread, so 512 stays neutral -- greys do
-- not pick up a cast), clamped to 512 +-448.  Used twice (before the bloom
-- and after it); en = '0' passes the input through at the same latency.
-- Latency 5 clocks from the input ports (inputs must be registered).
--
--   ml, sl  luma:   q = ((y - 64) * ml) >> 16,  y' = 64 + q * sl
--   cg      chroma: saturation gain, 1/16 units (0 = grey)
--   ck      chroma: rounding step 2^(ck + 4)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vitrail_quant is
    port (
        clk   : in  std_logic;
        en    : in  std_logic;
        ml    : in  unsigned(10 downto 0);
        sl    : in  unsigned(9 downto 0);
        cg    : in  unsigned(5 downto 0);
        ck    : in  unsigned(2 downto 0);
        y_in  : in  unsigned(9 downto 0);
        u_in  : in  unsigned(9 downto 0);
        v_in  : in  unsigned(9 downto 0);
        y_out : out unsigned(9 downto 0);
        u_out : out unsigned(9 downto 0);
        v_out : out unsigned(9 downto 0)
    );
end entity vitrail_quant;

architecture rtl of vitrail_quant is

    type t_d10 is array (1 to 4) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_d10 := (others => (others => '0'));

    -- luma
    signal s_t1  : unsigned(9 downto 0) := (others => '0');
    signal s_ph2, s_pl2 : unsigned(15 downto 0) := (others => '0');
    signal s_s3  : unsigned(20 downto 0) := (others => '0');
    signal s_m4  : unsigned(13 downto 0) := (others => '0');
    -- chroma
    signal s_pu1, s_pv1 : signed(17 downto 0) := (others => '0');
    signal s_au2, s_av2 : signed(13 downto 0) := (others => '0');
    signal s_qu3, s_qv3 : signed(10 downto 0) := (others => '0');
    signal s_qu4, s_qv4 : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := (others => '0');

    -- round down to a multiple of 2^(ck + 4), then clamp to +-448
    function f_cq(a : signed(13 downto 0); k : unsigned(2 downto 0)) return signed is
        variable v : signed(13 downto 0);
    begin
        v := a;
        case to_integer(k) is
            when 0      => v(3 downto 0) := (others => '0');
            when 1      => v(4 downto 0) := (others => '0');
            when 2      => v(5 downto 0) := (others => '0');
            when 3      => v(6 downto 0) := (others => '0');
            when 4      => v(7 downto 0) := (others => '0');
            when others => v(8 downto 0) := (others => '0');
        end case;
        if v > 448 then
            return to_signed(448, 11);
        elsif v < -448 then
            return to_signed(-448, 11);
        else
            return v(10 downto 0);
        end if;
    end function;

    -- half of the rounding step, added before the round-down
    function f_half(k : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(k) is
            when 0      => return to_signed(8, 14);
            when 1      => return to_signed(16, 14);
            when 2      => return to_signed(32, 14);
            when 3      => return to_signed(64, 14);
            when 4      => return to_signed(128, 14);
            when others => return to_signed(256, 14);
        end case;
    end function;

begin

    process(clk)
        variable v_du, v_dv : signed(10 downto 0);
        variable v_cu, v_cv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_yd(1) <= y_in; s_ud(1) <= u_in; s_vd(1) <= v_in;
            for i in 2 to 4 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
            end loop;

            -- 1: luma offset from black, clamped to 0..875; chroma gain
            if y_in < 64 then
                s_t1 <= (others => '0');
            elsif y_in > 939 then
                s_t1 <= to_unsigned(875, 10);
            else
                s_t1 <= y_in - 64;
            end if;
            v_du := signed(resize(u_in, 11)) - 512;
            v_dv := signed(resize(v_in, 11)) - 512;
            s_pu1 <= v_du * signed('0' & cg);
            s_pv1 <= v_dv * signed('0' & cg);

            -- 2: luma level products (split on t); chroma + half step
            s_ph2 <= s_t1(9 downto 5) * ml;
            s_pl2 <= s_t1(4 downto 0) * ml;
            s_au2 <= resize(shift_right(s_pu1, 4), 14) + f_half(ck);
            s_av2 <= resize(shift_right(s_pv1, 4), 14) + f_half(ck);

            -- 3: luma level index; chroma round + clamp
            s_s3  <= shift_left(resize(s_ph2, 21), 5) + resize(s_pl2, 21);
            s_qu3 <= f_cq(s_au2, ck);
            s_qv3 <= f_cq(s_av2, ck);

            -- 4: luma level value; chroma recentre (+512 on a 10-bit
            -- two's-complement value in -448..448 = flip the top bit)
            s_m4  <= s_s3(19 downto 16) * sl;
            v_cu := unsigned(s_qu3(9 downto 0)); v_cu(9) := not v_cu(9);
            v_cv := unsigned(s_qv3(9 downto 0)); v_cv(9) := not v_cv(9);
            s_qu4 <= v_cu;
            s_qv4 <= v_cv;

            -- 5: output (or the delayed input)
            if en = '1' then
                s_y5 <= resize(s_m4, 10) + 64;
                s_u5 <= s_qu4; s_v5 <= s_qv4;
            else
                s_y5 <= s_yd(4); s_u5 <= s_ud(4); s_v5 <= s_vd(4);
            end if;
        end if;
    end process;

    y_out <= s_y5;
    u_out <= s_u5;
    v_out <= s_v5;

end architecture rtl;
