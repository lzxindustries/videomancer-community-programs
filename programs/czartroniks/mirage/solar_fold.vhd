-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- solar_fold: Sabattier fold used by mirage (two instances).
--   y' = base + |((gain * (y - thr)) mod 1024) - 512| inside the region
--   (y > thr); chroma inverted inside the region when fold_c = '1'.
-- Latency 7 clocks.  enable = '0' passes the input through (region still
-- reported).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity solar_fold is
    port (
        clk    : in  std_logic;
        y_in   : in  unsigned(9 downto 0);
        u_in   : in  unsigned(9 downto 0);
        v_in   : in  unsigned(9 downto 0);
        thr    : in  unsigned(9 downto 0);
        gain   : in  unsigned(11 downto 0);   -- Q8
        base   : in  signed(11 downto 0);
        fold_c : in  std_logic;
        enable : in  std_logic;
        y_out  : out unsigned(9 downto 0);
        u_out  : out unsigned(9 downto 0);
        v_out  : out unsigned(9 downto 0);
        ysol   : out unsigned(9 downto 0);    -- folded luma regardless of enable
        region : out std_logic
    );
end entity solar_fold;

architecture rtl of solar_fold is
    type t_y is array (1 to 7) of unsigned(9 downto 0);
    signal s_y, s_u, s_v : t_y := (others => (others => '0'));
    type t_b is array (2 to 7) of std_logic;
    signal s_reg : t_b := (others => '0');
    signal s_d2  : signed(10 downto 0) := (others => '0');
    signal s_xh3 : signed(17 downto 0) := (others => '0');
    signal s_xl3 : signed(17 downto 0) := (others => '0');
    signal s_x4  : signed(23 downto 0) := (others => '0');
    signal s_t5  : unsigned(9 downto 0) := (others => '0');
    signal s_tri6 : unsigned(9 downto 0) := (others => '0');
    signal s_ysol7 : unsigned(9 downto 0) := (others => '0');
begin
    process(clk)
        variable v_s : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_y(1) <= y_in; s_u(1) <= u_in; s_v(1) <= v_in;
            for i in 2 to 7 loop
                s_y(i) <= s_y(i - 1); s_u(i) <= s_u(i - 1); s_v(i) <= s_v(i - 1);
            end loop;
            -- 2
            s_d2 <= signed('0' & s_y(1)) - signed('0' & thr);
            if s_y(1) > thr then s_reg(2) <= '1'; else s_reg(2) <= '0'; end if;
            for i in 3 to 7 loop
                s_reg(i) <= s_reg(i - 1);
            end loop;
            -- 3
            s_xh3 <= s_d2 * signed('0' & gain(11 downto 6));
            s_xl3 <= s_d2 * signed('0' & gain(5 downto 0));
            -- 4
            s_x4 <= shift_left(resize(s_xh3, 24), 6) + resize(s_xl3, 24);
            -- 5
            s_t5 <= unsigned(s_x4(17 downto 8));
            -- 6
            if s_t5 >= 512 then
                s_tri6 <= s_t5 - 512;
            else
                s_tri6 <= to_unsigned(512, 10) - s_t5;
            end if;
            -- 7
            v_s := resize(base, 13) + signed(resize(s_tri6, 13));
            if v_s < 0 then
                s_ysol7 <= (others => '0');
            elsif v_s > 1023 then
                s_ysol7 <= (others => '1');
            else
                s_ysol7 <= unsigned(v_s(9 downto 0));
            end if;
        end if;
    end process;

    ysol   <= s_ysol7;
    region <= s_reg(7);
    y_out  <= s_ysol7 when (enable = '1' and s_reg(7) = '1') else s_y(7);
    u_out  <= (not s_u(7)) when (enable = '1' and fold_c = '1' and s_reg(7) = '1') else s_u(7);
    v_out  <= (not s_v(7)) when (enable = '1' and fold_c = '1' and s_reg(7) = '1') else s_v(7);
end architecture rtl;
