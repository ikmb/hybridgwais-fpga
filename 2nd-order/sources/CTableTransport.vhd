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

use work.ad8k5_2way_ctables_pkg.all;

entity CTableTransport is
generic (
   PE_ID : natural := 0
);
port (
   stream_clk : in std_logic;
   stream_clk_reset_in : in std_logic;
   
   -- CTable input
   table_busy_in  : in std_logic;
   table_ready_in : in std_logic;
   get_table_out : out std_logic;
   table_casenctrl_in : in std_logic;
   table_round_done_in : in std_logic;
   table_counts_in : in std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0);
   
   -- Bus input
   slot_occ_in : in std_logic;
   slot_casenctrl_in : in std_logic;
   slot_row_done_in : in std_logic;
   slot_round_done_in : in std_logic;
   bus_data_in : in std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0);
   
   -- Bus output
   slot_occ_out : out std_logic;
   slot_casenctrl_out : out std_logic;
   slot_row_done_out : out std_logic;
   slot_round_done_out : out std_logic;
   bus_data_out : out std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0)
);
end CTableTransport;

architecture Behavioral of CTableTransport is

signal get_table : std_logic := '0';
signal slot_occ_del : std_logic := '0';

begin

-- read table if requested or falling edge of slot_occ_in and only if ready
get_table_out <= ((slot_occ_del and not slot_occ_in) or get_table) and table_ready_in;

bus_ctrl_p: process
begin
   wait until rising_edge(stream_clk);
   
   get_table <= '0';
   slot_occ_out <= slot_occ_in;
   slot_occ_del <= slot_occ_in;
   
   if stream_clk_reset_in = '1' then
     slot_occ_out <= '0';
     slot_round_done_out <= '0';
     slot_row_done_out <= '0';
   else    
     if PE_ID = 0 then -- only for first PE
     
        -- always put output from PE on bus as soon as it is ready
        bus_data_out <= table_counts_in;
        slot_casenctrl_out <= table_casenctrl_in;
        slot_round_done_out <= table_round_done_in;
        -- potentionally this is the last table in this row
        -- will be blocked by further PEs if they prepare a table themselves
        slot_row_done_out <= '1'; 
        get_table <= '1';
        if table_ready_in = '1' then
           slot_occ_out <= '1';
        else
           slot_occ_out <= '0';
        end if;
        
     else -- all other PEs
     
        bus_data_out <= bus_data_in;
        slot_casenctrl_out <= slot_casenctrl_in;
        -- the "round done" information of the previous PE is only forwarded if this PE 
        -- does not have a table ready since it would carry the same information again 
        slot_round_done_out <= slot_round_done_in and not table_ready_in;
        -- "row done" indicates the last table from this chain, hence it is forwarded only if
        -- this PE does not prepare a table as well.
        slot_row_done_out <= slot_row_done_in and not table_busy_in;
        
        if slot_occ_del = '1' and slot_occ_in = '0' then
          -- falling edge of slot_occ_in
          get_table           <= '1';
          bus_data_out        <= table_counts_in;
          slot_casenctrl_out  <= table_casenctrl_in;
          slot_round_done_out <= table_round_done_in;
          -- potentionally this is the last table in this row
          -- will be blocked by further PEs if they prepare a table themselves
          slot_row_done_out   <= '1';
          if table_ready_in = '1' then
            slot_occ_out <= '1';
          else
            slot_occ_out <= '0';
          end if;
        end if;
        
     end if; -- PID /= 0     
   end if; -- reset
   
end process bus_ctrl_p;

end Behavioral;
