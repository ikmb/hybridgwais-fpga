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

-- Counts the input genotype pairs.
-- If casenctrl switches from 1 to 0, half of the table is finished and needs to be fetched.
-- The other half is finished if the new_snp is asserted, and needs to be fetched as well.
entity CTable is
generic (
   PE_ID : natural := 0
);
port (
   stream_clk : in std_logic;
   stream_clk_reset_in : in std_logic;
   
   -- Input: pair stream
   genotypeA_in : in genotype_block_t;
   genotypeB_in : in genotype_block_t;
   casenctrlAB_in : in std_logic; -- '1' case, '0' ctrl
   new_genotypeAB_in : in std_logic;
   snp_doneAB_in : in std_logic;
   round_doneAB_in : in std_logic;
   
   -- Output:
   -- signalizes a table is currently built
   table_busy_out : out std_logic := '0';
   -- signalizes output is ready to read
   table_ready_out : out std_logic := '0';
   -- get next counters (FWFT behaviour)
   get_table_in : in std_logic;
   -- indicates if table is for case (1) or controls (0)
   table_casenctrl_out : out std_logic;
   -- indicates if this is the last table of this round from this PE
   table_round_done_out : out std_logic;
   -- all counters in parts of TRANSPORT_BUS_WIDTH bits
   table_counts_out : out std_logic_vector(TRANSPORT_BUS_WIDTH-1 downto 0)
);
end CTable;

architecture Behavioral of CTable is

subtype genotype_pair_t is std_logic_vector(3 downto 0);
type genotype_pair_vector is array (natural range <>) of genotype_pair_t;
signal genotype_pairs : genotype_pair_vector(GENOTYPES_PER_CYCLE-1 downto 0);
signal casenctrl_old : std_logic := '0';

signal snp_doneAB_del : std_logic := '0';
signal round_doneAB_del : std_logic := '0';

signal table_ready : std_logic := '0'; 
signal save : std_logic_vector(NUM_COUNTERS_TRANSFERRED*CTABLE_ENTRY_WIDTH-1 downto 0) := (others => '0');
signal casenctrl_save : std_logic := '0';

begin

gpairs_g : for I in 0 to GENOTYPES_PER_CYCLE - 1 generate
   genotype_pairs(I) <= genotypeA_in(I) & genotypeB_in(I);
end generate gpairs_g;

table_casenctrl_out <= casenctrl_save;
table_counts_out <= save(TRANSPORT_BUS_WIDTH-1 downto 0);-- when table_ready = '1' else (others => '1');
table_ready_out <= table_ready;

cnt_p: process
  variable n00 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n01 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n02 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n10 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n11 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n12 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n20 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n21 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  variable n22 : unsigned(CTABLE_ENTRY_WIDTH - 1 downto 0) := (others => '0');
  
  variable n00_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n01_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n02_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n10_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n11_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n12_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n20_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n21_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
  variable n22_inc : integer range 0 to GENOTYPES_PER_CYCLE := 0;
      
  -- including missing values
  variable tobefetched : integer range 0 to 1 := 0;
begin
   wait until rising_edge(stream_clk);
   
   snp_doneAB_del <= snp_doneAB_in;
   round_doneAB_del <= round_doneAB_in;

   -- copy counters, reset
   if tobefetched = 0 then
      -- order for readout: as in above declarations
      save <= std_logic_vector(n22) 
            & std_logic_vector(n21) --*
            & std_logic_vector(n20)
            & std_logic_vector(n12) --*
            & std_logic_vector(n11) --*
            & std_logic_vector(n10) --*
            & std_logic_vector(n02)
            & std_logic_vector(n01) --*
            & std_logic_vector(n00);
              
      casenctrl_save <= casenctrl_old;
   end if;
   
   -- fetch table
   if get_table_in = '1' then
      if tobefetched = 1 then -- this was the last one
         table_ready <= '0';
         table_round_done_out <= '0';
         table_busy_out <= '0';
      end if;
--      if tobefetched /= 0 then -- unnecessary since get_table_in is directly coupled with table_ready
        tobefetched := tobefetched - 1;
--      end if;
   end if;
   
   if snp_doneAB_del = '1' or (casenctrl_old = '1' and casenctrlAB_in = '0' and new_genotypeAB_in = '1') then
            
      n00 := (others => '0');
      n01 := (others => '0');
      n02 := (others => '0');
      n10 := (others => '0');
      n11 := (others => '0');
      n12 := (others => '0');
      n20 := (others => '0');
      n21 := (others => '0');
      n22 := (others => '0');
      
      table_ready <= '1';
      table_round_done_out <= round_doneAB_del;
      tobefetched := 1;
      
   end if;
      
   -- counting genotypes
   if new_genotypeAB_in = '1' then
    
     table_busy_out <= '1'; -- indicates that this PE is preparing a table
     
      n00_inc := 0;
      n01_inc := 0;
      n02_inc := 0;
      n10_inc := 0;
      n11_inc := 0;
      n12_inc := 0;
      n20_inc := 0;
      n21_inc := 0;
      n22_inc := 0;
      
      -- choose increment
      for I in 0 to GENOTYPES_PER_CYCLE - 1 loop
         case genotype_pairs(I) is
         when "0000" => n00_inc := n00_inc + 1; -- 000
         when "0001" => n01_inc := n01_inc + 1; -- 001
         when "0010" => n02_inc := n02_inc + 1; -- 002
         when "0100" => n10_inc := n10_inc + 1; -- 010
         when "0101" => n11_inc := n11_inc + 1; -- 011
         when "0110" => n12_inc := n12_inc + 1; -- 012
         when "1000" => n20_inc := n20_inc + 1; -- 020
         when "1001" => n21_inc := n21_inc + 1; -- 021
         when "1010" => n22_inc := n22_inc + 1; -- 022
         when others => -- unrequired or unsupported genotype pair (not counted)
            null;
         end case;
      end loop;
      
      -- increment counters
      n00 := n00 + n00_inc;
      n01 := n01 + n01_inc;
      n02 := n02 + n02_inc;
      n10 := n10 + n10_inc;
      n11 := n11 + n11_inc;
      n12 := n12 + n12_inc;
      n20 := n20 + n20_inc;
      n21 := n21 + n21_inc;
      n22 := n22 + n22_inc;
  
      -- save state of casenctrl
      casenctrl_old <= casenctrlAB_in;
      
   end if;
   
   if stream_clk_reset_in = '1' then
     table_ready <= '0';
     table_round_done_out <= '0';
     table_busy_out <= '0';
     tobefetched := 0;
      n00 := (others => '0');
      n01 := (others => '0');
      n02 := (others => '0');
      n10 := (others => '0');
      n11 := (others => '0');
      n12 := (others => '0');
      n20 := (others => '0');
      n21 := (others => '0');
      n22 := (others => '0');
   end if;
      
end process cnt_p;

end Behavioral;

