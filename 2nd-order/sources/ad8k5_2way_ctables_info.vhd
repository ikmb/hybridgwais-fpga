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

package ad8k5_2way_ctables_info is

subtype field_t is std_logic_vector(15 downto 0);

constant INFO_APP_ID : std_logic_vector(7 downto 0) := x"02"; -- application identifier for 2-way interactions
constant INFO_VERSION_MAJOR : std_logic_vector(7 downto 0) := x"02";
constant INFO_VERSION_MINOR : std_logic_vector(7 downto 0) := x"02";
constant INFO_VERSION_REVISION : std_logic_vector(7 downto 0) := x"01";
         
constant INFO_STREAM_FREQUENCY : std_logic_vector(31 downto 0) := x"0FDAD680"; -- 266 MHz
         
constant INFO_NUM_CHAINS : field_t := std_logic_vector(to_unsigned(NUM_CHAINS,16));
constant INFO_NUM_PE_PER_CHAIN : field_t := std_logic_vector(to_unsigned(NUM_PE_PER_CHAIN,16));

constant INFO_TABLE_SIZE : field_t := x"002C"; -- in bytes (may include padding)
constant INFO_NUM_TABLE_ENTRIES : field_t := std_logic_vector(to_unsigned(2*NUM_COUNTERS_TRANSFERRED,16));
constant INFO_NUM_TABLE_ENTRY_SIZE : field_t := std_logic_vector(to_unsigned(CTABLE_ENTRY_WIDTH,16)); -- in bits

constant INFO_RESERVED0 : field_t := x"0000";

constant INFO_SNP_ID_SIZE : field_t := std_logic_vector(to_unsigned(ID_WIDTH,16)); -- in bits

constant INFO_SNP_WORD_SIZE : field_t := std_logic_vector(to_unsigned(8*LONGS_PER_RAMWORD,16)); -- word size for SNP raw data in bytes (RAM word size)

constant INFO_MINIMUM_SAMPLES : field_t := std_logic_vector(to_unsigned((NUM_PE_PER_CHAIN+1) * GENOTYPES_PER_CYCLE,16)); -- required minimum number of either cases or controls
constant INFO_MAXIMUM_SAMPLES : field_t := std_logic_vector(to_unsigned(65535,16)); -- maximum number of cases or controls (according to FIFO depths)

constant INFO_LOG_MAX_GENOTYPES : field_t := std_logic_vector(to_unsigned(35,16)); -- log_2 of maximum number of total genotypes supported (correlated to size of used memory)

constant INFO_RESERVED1 : std_logic_vector(7 downto 0) := x"00";


-- should be read as uint16_t, so reverse order in the fields...
constant INFO_DATA : std_logic_vector(247 downto 0) :=
    INFO_RESERVED1 
  & INFO_LOG_MAX_GENOTYPES
  & INFO_MAXIMUM_SAMPLES
  & INFO_MINIMUM_SAMPLES 
  & INFO_SNP_WORD_SIZE
  & INFO_SNP_ID_SIZE
  & INFO_RESERVED0
  & INFO_NUM_TABLE_ENTRY_SIZE
  & INFO_NUM_TABLE_ENTRIES
  & INFO_TABLE_SIZE
  & INFO_NUM_PE_PER_CHAIN
  & INFO_NUM_CHAINS
  & INFO_STREAM_FREQUENCY
  & INFO_VERSION_MINOR & INFO_VERSION_REVISION
  & INFO_APP_ID & INFO_VERSION_MAJOR;

end ad8k5_2way_ctables_info;

package body ad8k5_2way_ctables_info is

end ad8k5_2way_ctables_info;
