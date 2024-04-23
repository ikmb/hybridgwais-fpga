--
--    Copyright (C) 2018-2023 by Lars Wienbrandt,
--    Institute of Clinical Molecular Biology, Kiel University
--    
--    This file is part of HybridGWAIS-FPGA.
--
--    HybridGWAIS-FPGA is free software: you can redistribute it and/or modify
--    it under the terms of the GNU General Public License as published by
--    the Free Software Foundation, either version 3 of the License, or
--    (at your option) any later version.
--
--    HybridGWAIS-FPGA is distributed in the hope that it will be useful,
--    but WITHOUT ANY WARRANTY; without even the implied warranty of
--    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--    GNU General Public License for more details.
--
--    You should have received a copy of the GNU General Public License
--    along with HybridGWAIS-FPGA. If not, see <https://www.gnu.org/licenses/>.
--
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.ad8k5_2way_ctables_pkg.all;

entity CollectTables is
port (
   stream_clk : in std_logic;
   stream_clk_reset : in std_logic;
   table_read_clk : in std_logic;
   table_read_clk_reset : in std_logic;
   
   -- transport bus / ctables (stream_clk domain)
   slot_occ_in : in std_logic;
   slot_row_done_in : in std_logic;
   slot_round_done_in : in std_logic;
   bus_data_in : in std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0);
   
   -- output (table_read_clk_domain)
   table_ready_out : out std_logic;
   table_read_in : in std_logic;
   table_out : out half_table_t;
   table_ov_out : out std_logic;
   
   -- TODO clean up!
   table_row_done_out : out std_logic;         -- updated with reading a case table
   table_round_done_out : out std_logic; -- updated with reading a ctrl table
   
   -- stall (stream_clk domain)
   stall_out : out std_logic; -- asserted, when there's not enough space to keep 1024 tables
   
   -- DEBUG
   dbg_out : out std_logic_vector(127 downto 0)
);
end CollectTables;

architecture Behavioral of CollectTables is

signal bus_data_in_del : half_table_t;
signal small_round_done : std_logic;
signal row_done : std_logic;

-- buffer that holds all finished tables from one chain
signal out_buffer_we : std_logic := '0';
signal out_buffer_re : std_logic := '0';
signal out_buffer_dout : half_table_t;
signal out_buffer_full : std_logic;
signal out_buffer_empty : std_logic;

signal table_row_done : std_logic;
signal table_small_round_done : std_logic;

-- DEBUG
--signal slot_occ_count : unsigned(31 downto 0) := (others => '0');
signal we_count  : unsigned(47 downto 0) := (others => '0');
--signal cc_rd_count    : unsigned(31 downto 0) := (others => '0');
-- __DEBUG


begin

fifo_i : entity work.FIFO_FWFT_async_2kx145_Wrapper
port map (
   wr_clk => stream_clk,
   wr_rst => stream_clk_reset,
   rd_clk => table_read_clk,
   rd_rst => table_read_clk_reset,
   din(143 downto 0) => bus_data_in_del,
   din(144) => small_round_done,
   din(145) => row_done,
   wr_en => out_buffer_we,
   rd_en => out_buffer_re,
   dout(143 downto 0) => out_buffer_dout,
   dout(144) => table_small_round_done,
   dout(145) => table_row_done,
   full => out_buffer_full,
   empty => out_buffer_empty,
   prog_full => stall_out -- asserted if only 1024 entries left
);


collect_p: process
begin
   wait until rising_edge(stream_clk);
   
   out_buffer_we <= '0';
   
   -- overflow flag
   if stream_clk_reset = '1' then      
      table_ov_out <= '0';
   elsif out_buffer_we = '1' and out_buffer_full = '1' then
      table_ov_out <= '1';
   end if; 
   
   -- fill FIFOs
   if slot_occ_in = '1' then
     bus_data_in_del <= bus_data_in;
     small_round_done <= slot_round_done_in;
     row_done <= slot_row_done_in;
     out_buffer_we <= '1';
   end if;
   
end process collect_p;

table_ready_out <= not out_buffer_empty;
table_out <= out_buffer_dout;
out_buffer_re <= table_read_in;
-- TODO there is still some work to do
table_row_done_out <= table_row_done;           -- updated with reading a case table
table_round_done_out <= table_small_round_done; -- updated with reading a ctrl table

-- DEBUG
--dbg_out(31 downto 0)   <= std_logic_vector(slot_occ_count); 
dbg_out(63 downto 48) <= (others => '0');
dbg_out(47 downto 0)  <= (others => '0');
dbg_out(127 downto 112) <= (others => '0');
dbg_out(111 downto 64)  <= std_logic_vector(we_count);
--dbg_out(127 downto 96) <= std_logic_vector(cc_rd_count);

dbg_p: process
begin
   wait until rising_edge(stream_clk);
   
   if stream_clk_reset = '1' then
--      slot_occ_count <= (others => '0');
      we_count <= (others => '0');
--      cc_rd_count <= (others => '0');
   else
      
--      if slot_occ_in = '1' then
--         slot_occ_count <= slot_occ_count + 1;
--      end if;
      
      
      if out_buffer_we = '1' then
         we_count <= we_count + 1;
      end if;
      
      -- wrong clock domain!!! --> removed.
--      if casetable_read_in = '1' or ctrltable_read_in = '1' then
--         cc_rd_count <= cc_rd_count + 1;
--      end if;
      
   end if;
end process dbg_p;
-- __DEBUG

end Behavioral;

