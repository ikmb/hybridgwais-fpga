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
   slot_casenctrl_in : in std_logic;
   slot_row_done_in : in std_logic;
   slot_round_done_in : in std_logic;
   bus_data_in : in std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0);
   
   -- output (table_read_clk_domain)
   casetable_ready_out : out std_logic;
   casetable_read_in : in std_logic;
   casetable_out : out half_table_t;
   ctrltable_ready_out : out std_logic;
   ctrltable_read_in : in std_logic;
   ctrltable_out : out half_table_t;
   table_ov_out : out std_logic;
   table_row_done_out : out std_logic;         -- updated with reading a case table
   table_round_done_out : out std_logic; -- updated with reading a ctrl table
   
   -- stall (stream_clk domain)
   stall_out : out std_logic; -- asserted, when there's not enough space to keep 1024 tables
   
   -- DEBUG
   dbg_out : out std_logic_vector(127 downto 0)
);
end CollectTables;

architecture Behavioral of CollectTables is

signal casectrl_din : half_table_t;
signal row_done : std_logic;
signal small_round_done : std_logic;

signal case_we : std_logic := '0';
signal case_re : std_logic := '0';
signal case_dout : half_table_t;
signal case_full : std_logic;
signal case_empty : std_logic;

signal ctrl_we : std_logic := '0';
signal ctrl_re : std_logic := '0';
signal ctrl_dout : half_table_t;
signal ctrl_full : std_logic;
signal ctrl_empty : std_logic;

signal table_row_done : std_logic;
signal table_small_round_done : std_logic;

-- DEBUG
--signal slot_occ_count : unsigned(31 downto 0) := (others => '0');
signal case_we_count  : unsigned(47 downto 0) := (others => '0');
signal ctrl_we_count  : unsigned(47 downto 0) := (others => '0');
--signal cc_rd_count    : unsigned(31 downto 0) := (others => '0');
-- __DEBUG


begin

fifo_case_i : entity work.FIFO_FWFT_async_2kx145_Wrapper
port map (
   wr_clk => stream_clk,
   wr_rst => stream_clk_reset,
   rd_clk => table_read_clk,
   rd_rst => table_read_clk_reset,
   din(143 downto 0) => casectrl_din,
   din(144) => row_done,
   wr_en => case_we,
   rd_en => case_re,
   dout(143 downto 0) => case_dout,
   dout(144) => table_row_done,
   full => case_full,
   empty => case_empty,
   prog_full => stall_out -- asserted if only 1024 entries left
);

fifo_ctrl_i : entity work.FIFO_FWFT_async_2kx145_Wrapper
port map (
   wr_clk => stream_clk,
   wr_rst => stream_clk_reset,
   rd_clk => table_read_clk,
   rd_rst => table_read_clk_reset,
   din(143 downto 0) => casectrl_din,
   din(144) => small_round_done, -- assumed to come with controls only!!!
   wr_en => ctrl_we,
   rd_en => ctrl_re,
   dout(143 downto 0) => ctrl_dout,
   dout(144) => table_small_round_done,
   full => ctrl_full,
   empty => ctrl_empty,
   prog_full => open
);


collect_p: process
begin
   wait until rising_edge(stream_clk);
   
   case_we <= '0';
   ctrl_we <= '0';
   
   -- overflow flag
   if stream_clk_reset = '1' then      
      table_ov_out <= '0';
   elsif (case_we = '1' and case_full = '1') or (ctrl_we = '1' and ctrl_full = '1') then
      table_ov_out <= '1';
   end if; 
   
   -- fill FIFOs
   if slot_occ_in = '1' then
     casectrl_din <= bus_data_in;
     row_done <= slot_row_done_in;
     small_round_done <= slot_round_done_in;
     if slot_casenctrl_in = '1' then
        -- cases
        case_we <= '1';
     else
        -- controls
        ctrl_we <= '1';
     end if;
   end if;
   
end process collect_p;

casetable_ready_out <= not case_empty;
ctrltable_ready_out <= not ctrl_empty;
casetable_out <= case_dout;
ctrltable_out <= ctrl_dout;
ctrl_re <= ctrltable_read_in;
case_re <= casetable_read_in;
table_row_done_out <= table_row_done;           -- updated with reading a case table
table_round_done_out <= table_small_round_done; -- updated with reading a ctrl table

-- DEBUG
--dbg_out(31 downto 0)   <= std_logic_vector(slot_occ_count); 
dbg_out(63 downto 48) <= (others => '0');
dbg_out(47 downto 0)  <= std_logic_vector(case_we_count);
dbg_out(127 downto 112) <= (others => '0');
dbg_out(111 downto 64)  <= std_logic_vector(ctrl_we_count);
--dbg_out(127 downto 96) <= std_logic_vector(cc_rd_count);

dbg_p: process
begin
   wait until rising_edge(stream_clk);
   
   if stream_clk_reset = '1' then
--      slot_occ_count <= (others => '0');
      case_we_count <= (others => '0');
      ctrl_we_count <= (others => '0');
--      cc_rd_count <= (others => '0');
   else
      
--      if slot_occ_in = '1' then
--         slot_occ_count <= slot_occ_count + 1;
--      end if;
      
      if case_we = '1' then
         case_we_count <= case_we_count + 1;
      end if;
      
      if ctrl_we = '1' then
         ctrl_we_count <= ctrl_we_count + 1;
      end if;
      
      -- wrong clock domain!!! --> removed.
--      if casetable_read_in = '1' or ctrltable_read_in = '1' then
--         cc_rd_count <= cc_rd_count + 1;
--      end if;
      
   end if;
end process dbg_p;
-- __DEBUG

end Behavioral;

