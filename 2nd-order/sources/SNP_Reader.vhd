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

entity SNP_Reader is
   port(
      ram_clk            : in  std_logic;
      stream_clk         : in  std_logic;
      ram_clk_reset      : in  std_logic;
      stream_clk_reset   : in  std_logic;

      -- init and status (ram_clk domain)
--      set_num_samples_in : in  std_logic;
--      set_num_cases_in   : in  std_logic;
--      set_num_snps_in    : in  std_logic;
      num_samples_rounded_tig_in : in unsigned(31 downto 0);
      num_cases_rounded_tig_in   : in unsigned(31 downto 0);
      num_snps_local_tig_in      : in unsigned(31 downto 0);
      last_init_snpidx_tig_in    : in unsigned(31 downto 0); -- inclusive
      stream_start_snpidx_tig_in : in unsigned(31 downto 0);
      stream_start_addr_tig_in   : in unsigned(29 downto 0);
      round_addr_offset_tig_in   : in unsigned(29 downto 0);
      start_reading_in   : in  std_logic;

      -- stream data (stream_clk domain)
      genotype_out       : out genotype_block_t; -- provides gts/cycle times 2bit genotype data
      casenctrl_out      : out std_logic; -- '1' case, '0' ctrl
      new_genotype_out   : out std_logic; -- indicates valid genotype data
      snp_done_out       : out std_logic; -- indicates the last valid genotype of this SNP
      round_done_out     : out std_logic; -- indicates the end of a round (asserted only one cycle!)
      busy_out           : out std_logic;
      stall_in           : in  std_logic; -- stalls streaming after current SNP (if set when next SNP is going to be loaded)

      dram_addr          : out std_logic_vector(29 downto 0) := (others => '0'); -- DRAM address (BANK:ROW:COLUMN)
      dram_en            : out std_logic                     := '0'; -- DRAM start command
      dram_rdy           : in  std_logic; -- DRAM ready to start command
      dram_rd_data       : in  std_logic_vector(511 downto 0); -- DRAM read data
      dram_rd_data_end   : in  std_logic; -- DRAM read data end
      dram_rd_data_valid : in  std_logic; -- DRAM read data valid

      dbg_out            : out std_logic_vector(127 downto 0)
   );
end SNP_Reader;

architecture Behavioral of SNP_Reader is

   type mem_ctrl_state_t is (IDLE, PREPARE, INSERT);
   signal mem_ctrl_state : mem_ctrl_state_t := IDLE;
   
   signal num_snps_local    : unsigned(31 downto 0);
   signal last_init_snpidx  : unsigned(31 downto 0); -- inclusive
   signal stream_start_snpidx : unsigned(31 downto 0);
   signal num_samples       : unsigned(31 - LOG_GENOTYPES_PER_CYCLE downto 0);
   signal num_sample_blocks_m1 : unsigned(23 downto 0);
   signal num_cases         : unsigned(31 - LOG_GENOTYPES_PER_CYCLE downto 0);
--   signal round_last_snp    : unsigned(31 downto 0);
   
   signal next_addr         : unsigned(29 downto 0) := (others => '0');
   signal next_round_addr   : unsigned(29 downto 0) := (others => '0');
   signal round_addr_offset : unsigned(29 downto 0) := (others => '0');
   signal stream_start_addr : unsigned(29 downto 0) := (others => '0');

   signal flagfifo_set_round_done : std_logic := '0';
   signal flagfifo_wr             : std_logic := '0';
   signal flagfifo_is_round_done  : std_logic := '0';
   signal flagfifo_rd             : std_logic := '0';
   signal flagfifo_empty          : std_logic := '0';
   signal flagfifo_full           : std_logic := '0';

   signal dec_din   : std_logic_vector(513 downto 0);
   signal dec_wr_en : std_logic;
   signal dec_rd_en : std_logic;
   signal dec_dout  : std_logic_vector(513 downto 0);
   signal dec_full  : std_logic;
   signal dec_empty : std_logic;

   signal genotype_block : std_logic_vector(511 downto 0) := (others => '0');
   signal new_genotype : std_logic := '0';

   signal mem_busy : std_logic := '0';
   signal stream_busy : std_logic := '0';
   signal busy_tig : std_logic := '0';

   signal mem_rd_addr       : std_logic_vector(29 downto 0);
   signal mem_rd_data       : std_logic_vector(511 downto 0);
   signal mem_rd_data_valid : std_logic;
   signal dram_read_request : std_logic := '0';
   signal dram_en_int       : std_logic;

   signal mem_req_flag  : std_logic := '0';
   signal mem_proc_flag : std_logic := '0';
   
   -- for simulation only
   signal snp_count_dbg         : unsigned(31 downto 0) := (others => '0');
   signal round_count_dbg : unsigned(31 downto 0) := (others => '0');

begin

   constant_init_p : process
--      variable num_samples_tig    : unsigned(31 downto 0); -- TIG!
--      variable num_cases_tig      : unsigned(31 downto 0); -- TIG!
--      variable num_snps_local_tig : unsigned(31 downto 0); -- TIG!
   begin
      wait until rising_edge(ram_clk);

      -- num_samples_rounded_in is already rounded to a multiple of GENOTYPES_PER_CYCLE by the host
      -- (seperately rounded for the number of cases and the number of controls!)
      num_samples <= num_samples_rounded_tig_in(31 downto LOG_GENOTYPES_PER_CYCLE);

      if num_samples_rounded_tig_in(7 downto 0) = "00000000" then -- exact on 512bit-word boundary
         num_sample_blocks_m1 <= num_samples_rounded_tig_in(31 downto 8) - 1; -- we need the number minus 1
         -- now calculated on host: round_addr_offset   <= ((LONGS_PER_RAMWORD * NUM_PE) * to_integer(num_samples_rounded_tig_in(31 downto 8)));
      else                              -- round up
         num_sample_blocks_m1 <= num_samples_rounded_tig_in(31 downto 8); -- + 1 - 1
         -- now calculated on host: round_addr_offset   <= ((LONGS_PER_RAMWORD * NUM_PE) * to_integer(num_samples_rounded_tig_in(31 downto 8))) + LONGS_PER_RAMWORD * NUM_PE;
      end if;
      -- the address space of NUM_PE SNPs (calculated on host)
      round_addr_offset <= round_addr_offset_tig_in; 

      -- num_cases_rounded_in is already rounded to a multiple of GENOTYPES_PER_CYCLE by the host
      num_cases <= num_cases_rounded_tig_in(31 downto LOG_GENOTYPES_PER_CYCLE);

      num_snps_local <= num_snps_local_tig_in;
      
      -- local SNP after which the initialization of the chain does not need to be started again
--      round_last_snp <= num_snps_local - 1;
      last_init_snpidx <= last_init_snpidx_tig_in;

      -- first local SNP that should generate pairs 
      stream_start_snpidx <= stream_start_snpidx_tig_in;      
      stream_start_addr <= stream_start_addr_tig_in;
      
      -- TIGs:

--      if set_num_samples_in = '1' then
--         num_samples_tig := num_samples_rounded_in;
--      end if;
--
--      if set_num_cases_in = '1' then
--         num_cases_tig := num_cases_rounded_in;
--      end if;
--
--      if set_num_snps_in = '1' then
--         num_snps_local_tig := num_snps_local_in;
--      end if;

   end process constant_init_p;

   mem_ctrl_p : process
      variable curr_snp           : unsigned(31 downto 0) := (others => '0');
      variable next_round_start   : unsigned(31 downto 0) := (others => '0');
      variable sample_block_count : unsigned(23 downto 0) := (others => '0');
      variable new_snp_flag       : boolean                       := false;
      variable new_round_flag     : boolean                       := false;
      variable round_done_flag    : boolean                       := false;
      variable jump_flag          : boolean                       := false;
      variable set_new_snp        : std_logic                     := '0';
      variable set_round_done     : std_logic                     := '0';
   begin
      wait until rising_edge(ram_clk);

      -- default:
      flagfifo_wr <= '0';

      if ram_clk_reset = '1' then
         mem_ctrl_state    <= IDLE;
         dram_read_request <= '0';
         mem_busy          <= '0';
         mem_req_flag      <= '0';
      else
         -- read command was successful
         if dram_en_int = '1' then
            dram_read_request <= '0';
         end if;

         case mem_ctrl_state is
           when IDLE =>
               -- only set back the busy flag if all requests have been answered.
               if mem_req_flag = mem_proc_flag then
                 mem_busy <= '0';
               end if;
               next_addr          <= (others => '0');
               next_round_addr    <= (others => '0');
               curr_snp           := (others => '0');
               next_round_start   := (others => '0');
               sample_block_count := (others => '0');
               new_snp_flag       := true;
               new_round_flag     := true;
               round_done_flag    := false;
               jump_flag          := false;
               set_new_snp        := '0';
               set_round_done     := '0';
               -- DO NOT RESET THE DRAM REQUEST HERE!!! The memory could not have acknowledged the very last request of the last run.
               --dram_read_request  <= '0';

               -- GO!
               if start_reading_in = '1' then
                  mem_busy       <= '1';
                  mem_ctrl_state <= PREPARE;
               end if;

            when PREPARE =>

               -- default: command insertion
               mem_ctrl_state <= INSERT;
               set_new_snp    := '0';
               set_round_done := '0';

               -- signalize the beginning of a new SNP
               if new_snp_flag then
                 set_new_snp := '1';
               end if;

               if round_done_flag then
                  set_round_done := '1';
               end if;
               
               -- prepare next address
               if new_round_flag then

                  -- check stop condition:
                  -- The use of > instead of >= causes the last SNP to be loaded into the chain.
                  -- Before, it was not loaded as for a complete run it did not generate a new pair.
                  -- Now, in the corner case that the last SNP of a complete run initializes
                  -- a new round, the "round_done_flag" will be missing, but this should not be a problem...
                  --if next_round_start >= round_last_snp then
                  if next_round_start > last_init_snpidx then
                     -- finished, no more commands!
                     mem_ctrl_state <= IDLE;
                  end if;

                  next_addr        <= next_round_addr;
                  next_round_addr  <= next_round_addr + round_addr_offset; -- + NUM_PE SNPs     
                  curr_snp         := next_round_start; -- do not start again with zero
                  next_round_start := next_round_start + NUM_PE;             

               elsif jump_flag then
                 
                 next_addr <= stream_start_addr;                  
                 curr_snp  := stream_start_snpidx;
                 
               else
                 
                 next_addr <= next_addr + LONGS_PER_RAMWORD;
                 
               end if;

               new_snp_flag    := false;
               new_round_flag  := false;
               round_done_flag := false;
               jump_flag       := false;
               
               -- TODO put this in an "else" environment? dependent on curr_snp, next_round_start. . .
               
               -- do some preparations for the next SNP if these are the last samples of a SNP
               if num_sample_blocks_m1 = sample_block_count then
                  curr_snp           := curr_snp + 1; -- this preliminary increment helps us to test for the next conditions
                  sample_block_count := (others => '0');
                  
                  new_snp_flag       := true; -- new SNP after this command

                  if curr_snp = num_snps_local then
                      -- begin with a new round after this command
                     new_round_flag := true;

                  elsif curr_snp = next_round_start then
                      if next_round_start < stream_start_snpidx then
                        -- the chain is initialized and we could jump to the first stream index
                        -- (only if this is a forward jump)
                        jump_flag := true;
                      end if;
                      
                  elsif curr_snp = num_snps_local-1 then
                      -- last SNP of this round
                      round_done_flag := true;
                      
                  end if;

               else -- not finished yet with this SNP
                  sample_block_count := sample_block_count + 1;
               end if;

            when INSERT =>
               if dram_read_request = '0' and (flagfifo_full = '0' or set_new_snp = '0') and mem_req_flag = mem_proc_flag then -- already processed reply
                  -- ready for a new commmand
                  -- (if one command is entered, it will be removed almost immediately by the
                  --  controller, but it takes time for the answer to appear at the read FIFO.
                  --  Hence, this condition will be true again without having received the answer
                  --  of the previous request. The second request will stay in the command FIFO then
                  --  until the first request has been processed. Therefore, the burst length
                  --  MUST NOT exceed HALF of the size of the read FIFO, i.e. burst_length <= 32.)

                  -- insert command
                  -- moved outside condition...
                  mem_rd_addr <= std_logic_vector(next_addr);

                  dram_read_request <= '1';
                  mem_req_flag      <= not mem_req_flag;

                  -- everytime a new SNP starts, insert information if it is the last one of a round as well
                  if set_new_snp = '1' then
                     flagfifo_set_round_done <= set_round_done;
                     flagfifo_wr             <= '1';
                  end if;

                  -- prepare next command
                  mem_ctrl_state <= PREPARE;
               end if;

         end case;
      end if;
      
      -- DEBUG: make counters visible for simulation
      snp_count_dbg   <= curr_snp;
      round_count_dbg <= next_round_start;
      
   end process mem_ctrl_p;

   dram_addr         <= mem_rd_addr;
   dram_en_int       <= dram_read_request and dram_rdy;
   dram_en           <= dram_en_int;
   mem_rd_data       <= dram_rd_data;
   mem_rd_data_valid <= dram_rd_data_valid;

   -- just to signalize the end of a round to the reading process 
   flagfifo_i : entity work.FWFT_FIFO
      generic map(
         WIDTH => 1,
         DEPTH => 2
      )
      port map(
         sync_reset => ram_clk_reset,
         clk        => ram_clk,
         d_in(0)    => flagfifo_set_round_done,
         wr_in      => flagfifo_wr,
         d_out(0)   => flagfifo_is_round_done,
         rd_in      => flagfifo_rd,
         empty_out  => flagfifo_empty,
         full_out   => flagfifo_full
      );

   mem_rd_p : process
      variable sample_block_count : unsigned(23 downto 0) := (others => '0');
      variable dec_set_round_done   : std_logic                   := '0';
      variable dec_set_new_snp    : std_logic                     := '0';
      variable dec_wr_req         : boolean                       := false;
      -- DEBUG
      variable dec_wr_en_cnt : unsigned(63 downto 0) := (others => '0');
   begin
      wait until rising_edge(ram_clk);

      -- defaults:
      flagfifo_rd        <= '0';
      dec_wr_en          <= '0';
      dec_set_new_snp    := '0';
      dec_set_round_done := '0';
      
      dbg_out(63 downto 0) <= std_logic_vector(dec_wr_en_cnt);

      if ram_clk_reset = '1' then
         sample_block_count := (others => '0');
         mem_proc_flag      <= '0';
         dec_wr_req         := false;
      else
         

         if dec_wr_req and dec_full = '0' then -- and dec_wr_en = '0' then
           dec_wr_en     <= '1';
           dec_wr_en_cnt := dec_wr_en_cnt + 1;
            dec_wr_req    := false;
            mem_proc_flag <= not mem_proc_flag;
         end if;
         
         if mem_rd_data_valid = '1' then

            dec_wr_req := true;

            -- test for the beginning of a new SNP
            if sample_block_count = (sample_block_count'range => '0') then
               dec_set_new_snp    := '1';
               dec_set_round_done := flagfifo_is_round_done;
               flagfifo_rd        <= '1'; -- not necessary to test if the FIFO is not empty
            end if;

            dec_din <= dec_set_round_done & dec_set_new_snp & mem_rd_data;

            -- increment block counter and test if we have completed a SNP
            if num_sample_blocks_m1 = sample_block_count then
               sample_block_count := (others => '0');
            else
               sample_block_count := sample_block_count + 1;
            end if;

         end if;
      end if;

   end process mem_rd_p;

   decFIFO : entity work.FIFO_FWFT_distr_async_16x514_Wrapper
   port map(
      wr_clk => ram_clk,
      wr_rst => ram_clk_reset,
      rd_clk => stream_clk,
      rd_rst => stream_clk_reset,
      din    => dec_din,
      wr_en  => dec_wr_en,
      rd_en  => dec_rd_en,
      dout   => dec_dout,
      full   => dec_full,
      empty  => dec_empty
   );
   
   -- The busy signal will be set back to zero if all memory requests have been ANSWERED
   -- and no words are to be streamed anymore. From there on there are only a couple of
   -- cycles until the last ctables are fetched into the output queue (according to chain length).
   -- NOTE: in the special case, the last memory answer has been received and inserted in
   -- an empty decoupling FIFO, the busy signal will be reset for a few cycles until the
   -- data is eventually visible at the output of that FIFO.     
   busy_tig_p: process
   begin
     wait until rising_edge(ram_clk);
     busy_tig <= mem_busy or stream_busy;
   end process busy_tig_p;   
   busy_out <= busy_tig;
   
   stream_p : process
      variable sample_count     : unsigned(31 - LOG_GENOTYPES_PER_CYCLE downto 0) := (0 => '1', others => '0');
      variable shiftcount : integer range 0 to (256/GENOTYPES_PER_CYCLE)-1 := 0;
      variable new_snp      : std_logic := '0';
      variable round_done   : std_logic := '0';
   begin
      wait until rising_edge(stream_clk);

      -- defaults:
      dec_rd_en        <= '0';
      new_genotype     <= '0';
      snp_done_out     <= '0';
      round_done_out   <= '0';
      
      if stream_clk_reset = '1' then
         sample_count := (others => '0');
         shiftcount   := 0;
         stream_busy <= '0';
      else
         if shiftcount = 0 then
           if dec_empty = '0' then
              stream_busy <= '1';
              if dec_rd_en = '0' then
                 genotype_block <= dec_dout(511 downto 0); -- load new data
                 new_snp := dec_dout(512);
                 
                 -- only proceed if "stall" is not set at the beginning of a new SNP
                 -- and ensure a one clock cycle break at the beginning of a new SNP 
                 if new_snp = '0' or (stall_in = '0' and new_genotype = '0') then
                    dec_rd_en      <= '1';
                    new_genotype <= '1';
                    
                    if new_snp = '1' then
                       sample_count := (others => '0'); -- reset sample counter
                       round_done   := dec_dout(513); -- indicates the last SNP of this round
                    else
                       sample_count := sample_count + 1;
                    end if;
     
                    -- last sample of SNP
                    if sample_count = num_samples - 1 then
                       snp_done_out   <= '1';
                       -- round done indicator, was set at the beginning of this SNP
                       round_done_out <= round_done;                       
                    end if;
     
                    shiftcount := (256/GENOTYPES_PER_CYCLE)-1; -- required shifts per RAM word
                 end if;
              end if;
            else -- dec_empty = '1'
              stream_busy <= '0';
            end if;
         else
           -- gts/cycle times 2bit right shift
            genotype_block(511-2*GENOTYPES_PER_CYCLE downto 0) <= genotype_block(511 downto 2*GENOTYPES_PER_CYCLE);
            shiftcount     := shiftcount - 1;
            sample_count   := sample_count + 1;
            if sample_count /= num_samples then
               new_genotype <= '1';
            else
               shiftcount := 0;
            end if;
            -- last sample of SNP (same as above)
            if sample_count = num_samples - 1 then
               snp_done_out   <= '1';
               -- round done indicator, was set at the beginning of this SNP
               round_done_out <= round_done;
            end if;
         end if;

         -- case vs. control
         if sample_count < num_cases then
            casenctrl_out <= '1';
         else
            casenctrl_out <= '0';
         end if;

      end if;

   end process stream_p;
   new_genotype_out <= new_genotype;

   gt_out_g : for I in 0 to GENOTYPES_PER_CYCLE - 1 generate
      genotype_out(I) <= genotype_block(2 * I + 1 downto 2 * I);
   end generate gt_out_g;

end Behavioral;

