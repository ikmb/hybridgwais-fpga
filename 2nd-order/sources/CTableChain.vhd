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

entity CTableChain is
   generic(
      CHAIN_ID : natural := 0
   );
   port(
      stream_clk           : in  std_logic;
      stream_clk_reset     : in  std_logic;
      table_read_clk       : in  std_logic;
      table_read_clk_reset : in  std_logic;

      -- input stream data (stream_clk domain)
      genotype_in          : in  genotype_block_t; -- gts/cycle times 2bit input genotype data
      casenctrl_in         : in  std_logic; -- '1' case, '0' ctrl
      new_genotype_in      : in  std_logic; -- indicates valid genotype data
      mask_in              : in  std_logic; -- masks the current genotype such that it should not be used
      snp_done_in          : in  std_logic; -- indicates the last valid genotype for this SNP
      round_done_in        : in  std_logic; -- indicates the end of a round   
      
      -- output stream data (stream_clk domain)
      genotype_out         : out genotype_block_t; -- gts/cycle times 2bit genotype data (four genotypes per cycle)
      casenctrl_out        : out std_logic; -- '1' case, '0' ctrl
      new_genotype_out     : out std_logic; -- indicates valid genotype data
      mask_out             : out std_logic; -- masks the current genotype such that it should not be used
      snp_done_out         : out std_logic; -- indicates the last valid genotype for this SNP (asserted only one cycle!)
      round_done_out       : out std_logic; -- indicates the end of a "small" round (asserted only one cycle!)      
      
      -- output  (table_read_clk domain)
      casetable_ready_out : out std_logic;
      casetable_read_in : in std_logic;
      casetable_out : out half_table_t;
      ctrltable_ready_out : out std_logic;
      ctrltable_read_in : in std_logic;
      ctrltable_out : out half_table_t;
      table_ov_out : out std_logic;
      table_row_done_out : out std_logic; -- "row_done" flag indicates the last case table in the row
      table_round_done_out : out std_logic; -- "round_done" flag synchronized with the last control table
      
      -- stall (stream_clk domain)
      stall_out : out std_logic; -- asserted, when there's not enough space to keep 1024 tables
      
      -- DEBUG
      dbg_out : out std_logic_vector(127 downto 0)
   );
end CTableChain;

architecture Behavioral of CTableChain is

signal pairstreamer_reset          : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_genotype       : genotype_block_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_casenctrl      : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_new_genotype   : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_mask           : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_snp_done       : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_round_done     : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal pairstreamer_genotypeA      : genotype_block_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal pairstreamer_genotypeB      : genotype_block_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal pairstreamer_casenctrlAB    : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal pairstreamer_new_genotypeAB : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal pairstreamer_snp_doneAB     : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal pairstreamer_round_doneAB   : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);

signal ctable_reset       : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal ctable_casenctrl   : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal ctable_round_done  : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal ctable_counts      : counts_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal ctable_gettable    : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal ctable_table_ready : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);
signal ctable_table_busy  : std_logic_vector(NUM_PE_PER_CHAIN - 1 downto 0);

signal ctabletransport_slot_occ   : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal ctabletransport_casenctrl  : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal ctabletransport_row_done   : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal ctabletransport_round_done : std_logic_vector(NUM_PE_PER_CHAIN downto 0);
signal ctabletransport_bus_data   : counts_vector(NUM_PE_PER_CHAIN downto 0);

begin

pairstreamer_reset(0)        <= stream_clk_reset;
pairstreamer_genotype(0)     <= genotype_in;
pairstreamer_casenctrl(0)    <= casenctrl_in;
pairstreamer_new_genotype(0) <= new_genotype_in;
pairstreamer_mask(0)         <= mask_in;
pairstreamer_snp_done(0)     <= snp_done_in;
pairstreamer_round_done(0)   <= round_done_in;

genotype_out         <= pairstreamer_genotype(NUM_PE_PER_CHAIN);
casenctrl_out        <= pairstreamer_casenctrl(NUM_PE_PER_CHAIN);
new_genotype_out     <= pairstreamer_new_genotype(NUM_PE_PER_CHAIN); 
mask_out             <= pairstreamer_mask(NUM_PE_PER_CHAIN);
snp_done_out         <= pairstreamer_snp_done(NUM_PE_PER_CHAIN);
round_done_out       <= pairstreamer_round_done(NUM_PE_PER_CHAIN);
   

pe_g : for I in 0 to NUM_PE_PER_CHAIN - 1 generate
   pairstreamer_i : entity work.SNP_PairStreamer
     generic map(
       PE_ID => CHAIN_ID * NUM_CHAINS + I
     )
     port map(
        stream_clk           => stream_clk,
        stream_clk_reset_in  => pairstreamer_reset(I),
        stream_clk_reset_out => pairstreamer_reset(I + 1),
        genotype_in          => pairstreamer_genotype(I),
        casenctrl_in         => pairstreamer_casenctrl(I),
        new_genotype_in      => pairstreamer_new_genotype(I),
        mask_in              => pairstreamer_mask(I),
        snp_done_in          => pairstreamer_snp_done(I),
        round_done_in        => pairstreamer_round_done(I),
        genotype_out         => pairstreamer_genotype(I + 1),
        casenctrl_out        => pairstreamer_casenctrl(I + 1),
        new_genotype_out     => pairstreamer_new_genotype(I + 1),
        mask_out             => pairstreamer_mask(I + 1),
        snp_done_out         => pairstreamer_snp_done(I + 1),
        round_done_out       => pairstreamer_round_done(I + 1),
        genotypeA_out        => pairstreamer_genotypeA(I),
        genotypeB_out        => pairstreamer_genotypeB(I),
        casenctrlAB_out      => pairstreamer_casenctrlAB(I),
        new_genotypeAB_out   => pairstreamer_new_genotypeAB(I),
        snp_doneAB_out       => pairstreamer_snp_doneAB(I),
        round_doneAB_out     => pairstreamer_round_doneAB(I)
      );
      
   ctable_reset(I) <= pairstreamer_reset(I+1); -- so delayed by one clock cycle

   ctable_i : entity work.CTable
     generic map(
       PE_ID => CHAIN_ID * NUM_CHAINS + I
     )
      port map(
         stream_clk           => stream_clk,
         stream_clk_reset_in  => ctable_reset(I),
         genotypeA_in         => pairstreamer_genotypeA(I),
         genotypeB_in         => pairstreamer_genotypeB(I),
         casenctrlAB_in       => pairstreamer_casenctrlAB(I),
         new_genotypeAB_in    => pairstreamer_new_genotypeAB(I),
         snp_doneAB_in        => pairstreamer_snp_doneAB(I),
         round_doneAB_in      => pairstreamer_round_doneAB(I),
         table_busy_out       => ctable_table_busy(I),
         table_ready_out      => ctable_table_ready(I),
         get_table_in         => ctable_gettable(I),
         table_casenctrl_out  => ctable_casenctrl(I),
         table_round_done_out => ctable_round_done(I),
         table_counts_out     => ctable_counts(I)
      );

   ctabletransport_i : entity work.CTableTransport
      generic map(
         PE_ID => I
      )
      port map(
         stream_clk          => stream_clk,
         stream_clk_reset_in => ctable_reset(I),
         table_busy_in       => ctable_table_busy(I),
         table_ready_in      => ctable_table_ready(I),
         get_table_out       => ctable_gettable(I),
         table_casenctrl_in  => ctable_casenctrl(I), 
         table_round_done_in => ctable_round_done(I), 
         table_counts_in     => ctable_counts(I),
         slot_occ_in         => ctabletransport_slot_occ(I),
         slot_casenctrl_in   => ctabletransport_casenctrl(I),
         slot_row_done_in    => ctabletransport_row_done(I),
         slot_round_done_in  => ctabletransport_round_done(I),
         bus_data_in         => ctabletransport_bus_data(I),
         slot_occ_out        => ctabletransport_slot_occ(I + 1),
         slot_casenctrl_out  => ctabletransport_casenctrl(I + 1),
         slot_row_done_out   => ctabletransport_row_done(I + 1),
         slot_round_done_out => ctabletransport_round_done(I + 1),
         bus_data_out        => ctabletransport_bus_data(I + 1)
      );

end generate pe_g;

ctabletransport_bus_data(0)   <= (others => '0');
ctabletransport_casenctrl(0)  <= '0';
ctabletransport_row_done(0)   <= '0';
ctabletransport_round_done(0) <= '0';
ctabletransport_slot_occ(0)   <= '0';

collect_i : entity work.CollectTables
   port map(
      stream_clk           => stream_clk,
      stream_clk_reset     => stream_clk_reset,
      table_read_clk       => table_read_clk,
      table_read_clk_reset => table_read_clk_reset,
      slot_occ_in          => ctabletransport_slot_occ(NUM_PE_PER_CHAIN),
      slot_casenctrl_in    => ctabletransport_casenctrl(NUM_PE_PER_CHAIN),
      slot_row_done_in     => ctabletransport_row_done(NUM_PE_PER_CHAIN),
      slot_round_done_in   => ctabletransport_round_done(NUM_PE_PER_CHAIN),
      bus_data_in          => ctabletransport_bus_data(NUM_PE_PER_CHAIN),
      casetable_ready_out  => casetable_ready_out,
      casetable_read_in    => casetable_read_in,
      casetable_out        => casetable_out,
      ctrltable_ready_out  => ctrltable_ready_out,
      ctrltable_read_in    => ctrltable_read_in,
      ctrltable_out        => ctrltable_out,
      table_ov_out         => table_ov_out,
      table_row_done_out   => table_row_done_out, 
      table_round_done_out => table_round_done_out, 
      stall_out            => stall_out,
      dbg_out              => dbg_out
   );

end Behavioral;
