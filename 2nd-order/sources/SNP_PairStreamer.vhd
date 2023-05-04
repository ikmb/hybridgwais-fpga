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

entity SNP_PairStreamer is
generic (
   PE_ID : natural := 0
);
port (
   stream_clk : in std_logic;
   stream_clk_reset_in : in std_logic;
   stream_clk_reset_out : out std_logic;
   
   -- input stream data
   genotype_in     : in genotype_block_t; -- nx 2bit input genotype data
   casenctrl_in    : in std_logic; -- '1' case, '0' ctrl
   new_genotype_in : in std_logic; -- indicates valid genotype data
   mask_in         : in  std_logic; -- indicates if the incoming genotype should be used ('0') or not ('1')
   snp_done_in     : in  std_logic; -- indicates the last valid genotype for this SNP
   round_done_in   : in  std_logic; -- indicates the end of a round   
   
   -- output stream data
   genotype_out     : out genotype_block_t; -- nx 2bit genotype data (one genotype per cycle)
   casenctrl_out    : out std_logic; -- '1' case, '0' ctrl
   new_genotype_out : out std_logic; -- indicates valid genotype data
   mask_out         : out std_logic; -- indicates if the outgoing genotype should be used ('0') or not ('1')
   snp_done_out     : out std_logic; -- indicates the last valid genotype for this SNP (asserted only one cycle!)
   round_done_out   : out std_logic; -- indicates the end of a round (asserted only one cycle!)
   
   -- pair stream
   genotypeA_out      : out genotype_block_t;
   genotypeB_out      : out genotype_block_t;
   casenctrlAB_out    : out std_logic; -- '1' case, '0' ctrl
   new_genotypeAB_out : out std_logic;
   snp_doneAB_out     : out std_logic;
   round_doneAB_out   : out std_logic
);
end SNP_PairStreamer;

architecture Behavioral of SNP_PairStreamer is

signal genotype_i_del : genotype_block_t;
signal casenctrl_del : std_logic;
--signal new_genotype_del : std_logic;
signal snp_done_del : std_logic;
signal round_done_del : std_logic;

-- (ReReadable) FIFO access
-- A
signal fifoA_reset : std_logic := '0';
signal fifoA_genotype_i : std_logic_vector(2*GENOTYPES_PER_CYCLE-1 downto 0);
signal fifoA_genotype_o : std_logic_vector(2*GENOTYPES_PER_CYCLE-1 downto 0);
signal fifoA_en : std_logic := '0';
signal fifoA_rnw : std_logic := '0';
signal fifoA_empty : std_logic;
signal fifoA_full : std_logic;

type streamer_state_t is (FILL_A, STREAM);
signal streamer_state : streamer_state_t := FILL_A;

begin

fifoA: entity work.ReReadableFIFO_8Kx16
port map (
   clk       => stream_clk,
   reset     => fifoA_reset,
   d_in      => fifoA_genotype_i,
   d_out     => fifoA_genotype_o,
   en_in     => fifoA_en,
   rnw_in    => fifoA_rnw,
   empty_out => fifoA_empty,
   full_out  => fifoA_full
);

stream_p: process
begin
   wait until rising_edge(stream_clk);
   
   stream_clk_reset_out <= stream_clk_reset_in;
   
   -- defaults:
   fifoA_reset <= '0';
   fifoA_en    <= '0';
   
   for I in 0 to GENOTYPES_PER_CYCLE-1 loop
      fifoA_genotype_i(2 * I + 1 downto 2 * I) <= std_logic_vector(genotype_in(I));
      genotypeA_out(I)     <= genotype_t(fifoA_genotype_o(2*I+1 downto 2*I));
   end loop;
   
   genotype_i_del       <= genotype_in;
   genotype_out         <= genotype_in;
   casenctrl_out        <= casenctrl_in;
   new_genotype_out     <= new_genotype_in;
   mask_out             <= mask_in;
   snp_done_out         <= snp_done_in;
   round_done_out       <= round_done_in;
   casenctrl_del        <= casenctrl_in;
   snp_done_del         <= snp_done_in;
   round_done_del       <= round_done_in;
   genotypeB_out        <= genotype_i_del;
   casenctrlAB_out      <= casenctrl_del;
   new_genotypeAB_out   <= '0';
   snp_doneAB_out       <= '0';
   round_doneAB_out     <= '0';
   
   case streamer_state is
     when FILL_A =>
        mask_out   <= '1';          -- mask everything

        if mask_in = '0' then       -- progress only if not masked!
           -- write into FIFO
           if new_genotype_in = '1' then
              fifoA_en  <= '1';
              fifoA_rnw <= '0';     -- write
           end if;

           -- switch state if SNP is finished
           if snp_done_in = '1' then
              streamer_state <= STREAM;
           end if;
        end if;

     when STREAM =>
       
        if mask_in = '0' then       -- progress only if not masked!

           if new_genotype_in = '1' then
              fifoA_en  <= '1';
              fifoA_rnw <= '1';     -- read
           end if;

        end if;

        -- leaving this state is controlled independent outside the case environment
  end case;
  
  -- independent of the mask flag and the current state, the FIFO has to be reset if a round was done!
  if round_done_in = '1' or stream_clk_reset_in = '1' then
    -- round finished, the FIFO A has to be filled again
    fifoA_reset <= '1';

    streamer_state <= FILL_A;
  end if;
  
  -- paired stream control
  if fifoA_en = '1' and fifoA_rnw = '1' then
  -- this is a valid genotype pair
    new_genotypeAB_out <= '1';
    snp_doneAB_out <= snp_done_del;
    round_doneAB_out <= round_done_del;
  end if;
   
end process stream_p;

end Behavioral;

