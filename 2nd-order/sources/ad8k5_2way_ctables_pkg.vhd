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

package ad8k5_2way_ctables_pkg is

-- number of processing elements for contingency table creation
-- ATTENTION! Increase FIFO sizes and adjust stall limit in CollectTables.vhd if increasing the number of PEs
-- Example: 500 PEs need a stall limit of 1000, i.e. a free space of 1000 tables in each FIFO.
constant NUM_PE_PER_CHAIN : integer := 240; -- 250 is maximum for WTCCC (2000 cases), 8gts/cycle and one table/cycle
constant NUM_CHAINS : integer := 2; -- other changes required in VirtualFIFOArbiter
constant NUM_PE : integer := NUM_CHAINS * NUM_PE_PER_CHAIN;
constant GENOTYPES_PER_CYCLE : integer := 8; -- must be a power of 2 -> when changing, update ReReadableFIFO!
constant LOG_GENOTYPES_PER_CYCLE : integer := 3; -- base-2 logarithm of GENOTYPES_PER_CYCLE

constant CTABLE_ENTRY_WIDTH : integer := 16;
constant ID_WIDTH : integer := 32;
--constant NUM_COUNTERS : integer := 9; -- all counters
constant NUM_COUNTERS_TRANSFERRED : integer := 9; -- the subset of counters which is transferred to the host
constant TRANSPORT_BUS_WIDTH : integer := NUM_COUNTERS_TRANSFERRED * CTABLE_ENTRY_WIDTH; -- complete table on bus 

constant BYTES_PER_PCIEWORD : integer := 32; -- 32 bytes in one PCIe word (256bit)
constant LONGS_PER_PCIEWORD : integer := 4;  -- 4x 64bit (long) in one PCIe word (256bit)
constant LONGS_PER_RAMWORD : integer := 8; -- 8x 64bit (long) in one RAM word (512bit)
constant PCIWORDS_PER_RAMWORD : unsigned(5 downto 0) := "000001"; -- 2 words (means 2 256bit words which is one 512bit RAM word, required for compatibility)

constant MEM_CMD_WR : std_logic_vector(2 downto 0) := "000"; -- without auto-precharge -- "010"; (auto pre-charge)
constant MEM_CMD_RD : std_logic_vector(2 downto 0) := "001"; -- without auto-precharge -- "011"; (auto pre-charge)

constant NOT_BUSY_DELAY : integer := 16777215;

subtype genotype_t is std_logic_vector(1 downto 0);
type genotype_vector is array (natural range <>) of genotype_t;
subtype genotype_block_t is genotype_vector(GENOTYPES_PER_CYCLE-1 downto 0);
type genotype_block_vector is array (natural range <>) of genotype_block_t;
type counts_vector is array (natural range <>) of std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0);
subtype half_table_t is std_logic_vector(NUM_COUNTERS_TRANSFERRED*CTABLE_ENTRY_WIDTH-1 downto 0);
type half_table_vector is array (natural range <>) of half_table_t;
type id_t is array (1 downto 0) of std_logic_vector(ID_WIDTH-1 downto 0);
type id_vector is array (natural range <>) of id_t;

type integer_vector is array (natural range <>) of integer;
type boolean_vector is array (natural range <>) of boolean;

type debug_vector is array (natural range <>) of std_logic_vector(127 downto 0);

end ad8k5_2way_ctables_pkg;

package body ad8k5_2way_ctables_pkg is

 
end ad8k5_2way_ctables_pkg;
