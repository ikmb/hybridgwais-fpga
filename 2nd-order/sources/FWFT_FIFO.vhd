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

entity FWFT_FIFO is
generic (
	WIDTH : natural := 64;
	DEPTH : natural := 4
);
port (
	sync_reset : in std_logic;
	clk : in std_logic;
	
	d_in : in std_logic_vector(WIDTH-1 downto 0);
	wr_in : in std_logic;
	
	d_out : out std_logic_vector(WIDTH-1 downto 0);
	rd_in : in std_logic;
	
	empty_out : out std_logic;
	full_out : out std_logic
);
end FWFT_FIFO;

architecture Behavioral of FWFT_FIFO is

type data_array_t is array (natural range <>) of std_logic_vector(WIDTH-1 downto 0);
signal data_array : data_array_t(DEPTH-1 downto 0);

signal curr_wr : integer range 0 to DEPTH := 0;

signal empty : std_logic := '1';
signal full : std_logic := '0';

begin

empty <= '1' when curr_wr = 0 else '0';
empty_out <= empty;

full <= '1' when curr_wr = DEPTH else '0';
full_out <= full;

d_out <= data_array(0);

rw_p: process
	variable wr_ptr : integer range 0 to DEPTH;
begin
	wait until rising_edge(clk);
	
	wr_ptr := curr_wr;
	
	if sync_reset = '1' then
		curr_wr <= 0;
	else
		-- increment/decrement current write ptr
		if wr_in = '1' and rd_in = '0' and full = '0' then
		-- write, but no read
			curr_wr <= curr_wr + 1;
		elsif rd_in = '1' and wr_in = '0' and empty = '0' then
		-- read but no write
			curr_wr <= curr_wr - 1;
		end if;
		
		if rd_in = '1' and empty = '0' then
			-- left shift of data array
			data_array(DEPTH-2 downto 0) <= data_array(DEPTH-1 downto 1);
			-- correct write ptr for this cycle
			-- (to prevent data being written to the wrong address
			--  while concurrently writing)
			wr_ptr := curr_wr - 1;
		end if;
		
		if wr_in = '1' and full = '0' then
			data_array(wr_ptr) <= d_in;
		end if;
	end if;
end process rw_p;

end Behavioral;

