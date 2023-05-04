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

entity SNP_RawInit is
   port(
      clk                  : in  std_logic;
      reset                : in  std_logic;

      -- constant initialization
      num_samples_rounded_tig_in  : in  unsigned(31 downto 0);
--      set_num_samples_in   : in  std_logic;

      -- raw genotype data
      raw_genotype_in      : in  std_logic_vector(255 downto 0);
      raw_genotype_new_in  : in  std_logic;
      raw_genotype_ack_out : out std_logic;

      -- memory interface (write only)
      dram_addr            : out std_logic_vector(29 downto 0)  := (others => '0'); -- DRAM address (BANK:ROW:COLUMN)
      dram_en              : out std_logic                      := '0'; -- DRAM start command
      dram_rdy             : in  std_logic; -- DRAM ready to start command
      dram_wdf_data        : out std_logic_vector(511 downto 0) := (others => '0'); -- DRAM write data
      dram_wdf_end         : out std_logic                      := '0'; -- DRAM write data end
      dram_wdf_rdy         : in  std_logic; -- DRAM write fifo ready to accept
      dram_wdf_wren        : out std_logic                      := '0' -- DRAM write data enable

   );
end SNP_RawInit;

architecture Behavioral of SNP_RawInit is
   signal num_sample_blocks_m1 : unsigned(23 downto 0) := (others => '0');
   signal sample_block_count   : unsigned(23 downto 0) := (others => '0');
   signal curr_snp             : unsigned(31 downto 0) := (others => '0');
   signal curr_addr          : unsigned(29 downto 0)         := (others => '0');
   signal next_addr          : unsigned(29 downto 0)         := (others => '0');
   signal curr_pciword     : unsigned(5 downto 0)  := (others => '0');

   signal raw_genotype_ack : std_logic := '0';

   signal mem_wr_data : std_logic_vector(511 downto 0);
   signal mem_wr_addr : std_logic_vector(29 downto 0);

   signal dram_en_int        : std_logic := '0';
   signal dram_wdf_wren_int  : std_logic := '0';
   signal dram_write_request : std_logic := '0';

begin

   raw_genotype_ack_out <= raw_genotype_ack;

   constant_init_p : process
--      variable num_samples_tig  : unsigned(31 downto 0); -- TIG!
   begin
      wait until rising_edge(clk);

      if num_samples_rounded_tig_in(7 downto 0) = x"00" then -- exact on RAM word boundary
         num_sample_blocks_m1 <= num_samples_rounded_tig_in(31 downto 8) - 1; -- we need the number - 1 
      else                              -- round up
         num_sample_blocks_m1 <= num_samples_rounded_tig_in(31 downto 8);     -- number + 1 - 1
      end if;

--      if set_num_samples_in = '1' then
--         num_samples_tig := num_samples_rounded_in;
--      end if;

   end process constant_init_p;

   init_genotype_p : process
      variable mem_wr_data_tmp : std_logic_vector(511 downto 0);
   begin
      wait until rising_edge(clk);

      -- defaults:
      raw_genotype_ack <= '0';

      if reset = '1' then
         curr_snp           <= (others => '0');
         curr_addr          <= (others => '0');
         next_addr          <= (others => '0');
         curr_pciword     <= (others => '0');
         sample_block_count <= (others => '0');
      else

         -- write command was successful
         if dram_en_int = '1' then
           dram_write_request <= '0';
           raw_genotype_ack <= '1';
         end if;

         if raw_genotype_new_in = '1' and dram_write_request = '0' and raw_genotype_ack = '0' then
            
            curr_pciword <= curr_pciword + 1;
            -- address and counter incrementation
            next_addr      <= next_addr + LONGS_PER_PCIEWORD; -- 64bit word addressing
            -- write into FIFO
            mem_wr_data_tmp := raw_genotype_in & mem_wr_data_tmp(511 downto 256); -- 256bit right shift 

            -- send write command if one block is full or if the number of samples of this SNP is reached
            if curr_pciword = PCIWORDS_PER_RAMWORD then
               mem_wr_addr <= std_logic_vector(curr_addr);
               mem_wr_data <= mem_wr_data_tmp;
               
               -- count sample blocks of SNP
               sample_block_count <= sample_block_count + 1;

               curr_addr          <= next_addr + LONGS_PER_PCIEWORD; -- 64bit word addressing
               dram_write_request <= '1';

               curr_pciword <= (others => '0');

               -- next SNP
               if sample_block_count = num_sample_blocks_m1 then
                  sample_block_count <= (others => '0');
                  curr_snp           <= curr_snp + 1;
               end if;
               
            else
              
              -- simply ack the incoming word (otherwise, it will onyl be ack'ed when the corresponding word is written into RAM) 
              raw_genotype_ack <= '1';

            end if;

         end if;
      end if;

   end process init_genotype_p;

   dram_wdf_data     <= mem_wr_data;
   dram_addr         <= mem_wr_addr;
   dram_wdf_wren_int <= dram_write_request and dram_wdf_rdy and dram_rdy;
   dram_wdf_wren     <= dram_wdf_wren_int;
   dram_wdf_end      <= dram_wdf_wren_int;
   dram_en_int       <= dram_wdf_wren_int;
   dram_en           <= dram_en_int;

end Behavioral;

