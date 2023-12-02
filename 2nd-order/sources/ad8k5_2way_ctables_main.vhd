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
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

use work.ad8k5_2way_ctables_pkg.all;

entity ad8k5_2way_ctables_main is
  port(
    -- Clocks / Reset
    reset                : in  std_logic;
    pci_clk              : in  std_logic;
    ram_clk0             : in  std_logic;
    ram_clk0_reset       : in  std_logic;
    ram_clk1             : in  std_logic;
    ram_clk1_reset       : in  std_logic;

    -- PCIe DMA
    dma_din0_tdata       : in  std_logic_vector(255 downto 0);
    dma_din0_tvalid      : in  std_logic;
    dma_din0_tready      : out std_logic;
    dma_dout1_tdata      : out std_logic_vector(255 downto 0);
    dma_dout1_tvalid     : out std_logic;
    dma_dout1_tready     : in  std_logic;
    dma_dout2_tdata      : out std_logic_vector(255 downto 0);
    dma_dout2_tvalid     : out std_logic;
    dma_dout2_tready     : in  std_logic;

    -- PCIe Register Interface
    reg_clk              : out std_logic;
    reg_we               : out std_logic;
    reg_addr             : out std_logic_vector(9 downto 0) := (others => '0');
    reg_din              : in  std_logic_vector(255 downto 0);
    reg_dout             : out std_logic_vector(255 downto 0);
    
    -- Status (clock independent, use false-path constraints for reading)
    status               : out std_logic_vector(7 downto 0);

    -- DRAM
    c0_app_addr          : out std_logic_vector(29 downto 0);
    c0_app_cmd           : out std_logic_vector(2 downto 0);
    c0_app_en            : out std_logic;
    c0_app_wdf_data      : out std_logic_vector(575 downto 0);
    --c0_app_wdf_mask      : out std_logic_vector(71 downto 0);
    c0_app_wdf_end       : out std_logic;
    c0_app_wdf_wren      : out std_logic;
    c0_app_rd_data       : in  std_logic_vector(575 downto 0);
    c0_app_rd_data_end   : in  std_logic;
    c0_app_rd_data_valid : in  std_logic;
    c0_app_rdy           : in  std_logic;
    c0_app_wdf_rdy       : in  std_logic;
    c1_app_addr          : out std_logic_vector(29 downto 0);
    c1_app_cmd           : out std_logic_vector(2 downto 0);
    c1_app_en            : out std_logic;
    c1_app_wdf_data      : out std_logic_vector(575 downto 0);
    --c1_app_wdf_mask      : out std_logic_vector(71 downto 0);
    c1_app_wdf_end       : out std_logic;
    c1_app_wdf_wren      : out std_logic;
    c1_app_rd_data       : in  std_logic_vector(575 downto 0);
    c1_app_rd_data_end   : in  std_logic;
    c1_app_rd_data_valid : in  std_logic;
    c1_app_rdy           : in  std_logic;
    c1_app_wdf_rdy       : in  std_logic
  );
end ad8k5_2way_ctables_main;

architecture Behavioral of ad8k5_2way_ctables_main is

  signal stream_clk            : std_logic;
  signal stream_clk_sync_reset : std_logic := '1';
  signal dram_clk_sync_reset   : std_logic := '1';
  signal pci_clk_sync_reset    : std_logic := '1';
  signal ndcm_locked_or_host_reset_tig        : std_logic := '0';
  signal dcm_locked            : std_logic := '0';

  signal process_finished : std_logic := '0';
  signal process_finished_ramclk : std_logic := '0';

  signal inbuffer_wr_en : std_logic := '0';
  signal inbuffer_rd_en : std_logic := '0';
  signal inbuffer_full  : std_logic;
  signal inbuffer_empty : std_logic;
  signal inbuffer_dout  : std_logic_vector(255 downto 0);

  constant START_WORD : std_logic_vector(31 downto 0) := x"deadbeef";
  type indata_state_t is (WAIT_FOR_START, CONSTANTS1, CONSTANTS2, SNP_DATA);
  signal indata_state : indata_state_t := WAIT_FOR_START;

  signal start_ctables     : std_logic := '0';
  signal last_raw_gt_word  : unsigned(31 downto 0); -- index of the last 256 bit word of raw data from PCIe
  -- output buffer size in the number of output words and the number of tables respectively:
  signal ctable_io_bufsize_outwords_tig_ramclk : unsigned(31 downto 0); -- directly sampled from input with TIG constraint "to" in ramclk domain
  signal ctable_io_bufsize_tablewords_tig_ramclk : unsigned(31 downto 0); -- directly sampled from input with TIG constraint "to" in ramclk domain
  signal ctable_io_bufsize_outwords_tig_pciclk : unsigned(31 downto 0); -- sampled from *tig_ramclk with TIG constraint "to" in pciclk domain
  signal ctable_io_bufsize_tablewords_tig_pciclk : unsigned(31 downto 0); -- sampled from *tig_ramclk with TIG constraint "to" in pciclk domain
    
  signal rawinit_reset            : std_logic := '1';
  signal rawinit_raw_genotype     : std_logic_vector(255 downto 0);
  signal rawinit_raw_genotype_new : std_logic                      := '0';
  signal rawinit_raw_genotype_ack : std_logic;
  signal rawinit_dram_addr        : std_logic_vector(29 downto 0)  := (others => '0'); -- DRAM address (ROW:BANK:COLUMN)
  signal rawinit_dram_en          : std_logic                      := '0'; -- DRAM start command
  signal rawinit_dram_rdy         : std_logic; -- DRAM ready to start command
  signal rawinit_dram_wdf_data    : std_logic_vector(511 downto 0) := (others => '0'); -- DRAM write data
  signal rawinit_dram_wdf_end     : std_logic                      := '0'; -- DRAM write data end
  signal rawinit_dram_wdf_rdy     : std_logic; -- DRAM write fifo ready to accept
  signal rawinit_dram_wdf_wren    : std_logic                      := '0'; -- DRAM write data ena

  signal snpreader_start_reading      : std_logic                     := '0';
  signal snpreader_busy               : std_logic                     := '0';
  signal snpreader_genotype           : genotype_block_t;
  signal snpreader_casenctrl          : std_logic;
  signal snpreader_new_genotype       : std_logic                     := '0';
  signal snpreader_snp_done           : std_logic                     := '0';
  signal snpreader_round_done         : std_logic                     := '0';
  signal snpreader_dram_addr          : std_logic_vector(29 downto 0) := (others => '0'); -- DRAM address (ROW:BANK:COLUMN)
  signal snpreader_dram_en            : std_logic                     := '0'; -- DRAM start command
  signal snpreader_dram_rdy           : std_logic; -- DRAM ready to start command
  signal snpreader_dram_rd_data       : std_logic_vector(511 downto 0); -- DRAM read data
  signal snpreader_dram_rd_data_end   : std_logic; -- DRAM read data end
  signal snpreader_dram_rd_data_valid : std_logic; -- DRAM read data valid
  signal snpreader_debug              : std_logic_vector(127 downto 0);

  signal ctchain_genotype     : genotype_block_vector(NUM_CHAINS downto 0);
  signal ctchain_casenctrl    : std_logic_vector(NUM_CHAINS downto 0);
  signal ctchain_new_genotype : std_logic_vector(NUM_CHAINS downto 0);
  signal ctchain_mask         : std_logic_vector(NUM_CHAINS downto 0);
  signal ctchain_snp_done     : std_logic_vector(NUM_CHAINS downto 0);
  signal ctchain_round_done   : std_logic_vector(NUM_CHAINS downto 0);

  signal ctchain_casetable_ready      : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_casetable_read       : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_casetable            : half_table_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_ctrltable_ready      : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_ctrltable_read       : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_ctrltable            : half_table_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_table_ov         : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_table_row_done   : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_table_round_done : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_stall            : std_logic_vector(NUM_CHAINS - 1 downto 0);
  
  signal ctchain_table_ready : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_table_read : std_logic_vector(NUM_CHAINS - 1 downto 0);
  signal ctchain_id : id_vector(NUM_CHAINS-1 downto 0) := (others => (others => (others => '0')));
  
  signal ctchain_debug : debug_vector(NUM_CHAINS-1 downto 0);

--  signal set_num_samples : std_logic := '0';
--  signal set_num_snps    : std_logic := '0';
  signal num_samples_rounded_tig     : unsigned(31 downto 0);
  signal num_cases_rounded_tig       : unsigned(31 downto 0);
  signal num_snps_local_tig  : unsigned(31 downto 0);
  signal last_init_snpidx_tig  : unsigned(31 downto 0); --inclusive
  signal stream_start_snpidx_tig  : unsigned(31 downto 0);
  signal stream_start_addr_tig : unsigned(29 downto 0);
  signal round_addr_offset_tig : unsigned(29 downto 0);
  signal stall    : std_logic;
  
  -- table obtained from the FIFOs
  type raw_data_vector is array (natural range <>) of std_logic_vector(351 downto 0);
  signal table : raw_data_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
  
  --signal width_conv_reset  : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '0');
  signal width_conv_ready  : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '0');
  signal width_conv_tlast  : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '0');
  signal width_conv_islast  : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '0');
  signal dma_ins_pad : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '1');
  
  type dma_data_vector is array (natural range <>) of std_logic_vector(255 downto 0);
  signal dma_dout_tdata : dma_data_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
  signal dma_dout_tvalid : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '0');
  signal dma_dout_tready : std_logic_vector(NUM_CHAINS-1 downto 0) := (others => '0');
  
  constant HOST_RESET_CYCLES : integer := 268435455; -- host_reset will at least last this many ramclk cycles
  signal host_reset_sig : std_logic := '0';
  signal host_reset_ramclk : std_logic := '0'; -- TIG
  signal host_reset_pciclk : std_logic := '0'; -- TIG


  -- DEBUG
  signal snpreader_round_done_cnt   : unsigned(47 downto 0) := (others => '0');
  signal snpreader_snp_done_cnt     : unsigned(47 downto 0) := (others => '0');
  signal stall_cnt                  : unsigned(63 downto 0) := (others => '0');

  signal inbuffer_rd_cnt : unsigned(31 downto 0) := (others => '0');

  signal c0_rd_req_cnt   : unsigned(47 downto 0) := (others => '0');
  signal c0_rd_ans_cnt   : unsigned(47 downto 0) := (others => '0');
  signal c0_rd_ack_cnt   : unsigned(47 downto 0) := (others => '0');
  signal c0_conflict_cnt : unsigned(15 downto 0) := (others => '0');
  
  type dbg_count_vector is array (natural range <>) of unsigned(47 downto 0);
  signal tbuf_table_count_sig : dbg_count_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
  signal tbuf_word_count_sig : dbg_count_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
  
  signal reg_we_dbg       : std_logic;
  signal reg_addr_dbg     : std_logic_vector(9 downto 0) := (others => '0');
  signal reg_dout_dbg     : std_logic_vector(255 downto 0);
  
  signal host_reset_cycle_cnt : unsigned(47 downto 0) := (others => '0');
  signal host_reset_cnt : unsigned(15 downto 0) := (others => '0');
  -- __DEBUG

begin

  --      user_dcm_i : user_dcm
  --         port map(
  --            CLK_IN1  => clk,
  --            CLK_OUT1 => stream_clk,
  --            RESET    => reset,
  --            LOCKED   => dcm_locked
  --         );
  dcm_locked <= not reset;
  ndcm_locked_or_host_reset_tig <= not dcm_locked or host_reset_ramclk;
  
  dcm_locked_tig_p : process
  begin
    wait until rising_edge(pci_clk);
    pci_clk_sync_reset <= ndcm_locked_or_host_reset_tig;
  end process dcm_locked_tig_p;

  stream_clk <= ram_clk0;

  dram_clk_sync_reset_p : process
  begin
    wait until rising_edge(ram_clk0);
    dram_clk_sync_reset <= ndcm_locked_or_host_reset_tig;
  end process;

  stream_clk_sync_reset_p : process
  begin
    wait until rising_edge(stream_clk);
    stream_clk_sync_reset <= ndcm_locked_or_host_reset_tig;
  end process;

  host_reset_sig <= reg_din(0);
  
  host_reset_ramclk_p: process
    variable reg_we_old : std_logic := '0';
    variable timer : integer range 0 to HOST_RESET_CYCLES := 0;
    variable host_reset_old : std_logic := '0';
  begin
    wait until rising_edge(ram_clk0); -- which has to be the same as reg_clk!
    
    if reg_we_old = '0' then -- valid read data is available
      if host_reset_sig = '1' and timer = 0 then -- reset flag is set by host
        timer := HOST_RESET_CYCLES;
        host_reset_ramclk <= '1';
      elsif timer /= 0 then
        timer := timer - 1;
      else
        host_reset_ramclk <= '0';
      end if;        
    end if;
    reg_we_old := reg_we_dbg;
    
    -- DEBUG
    if host_reset_ramclk = '1' then
      host_reset_cycle_cnt <= host_reset_cycle_cnt + 1;
      if host_reset_old = '0' then
        host_reset_cnt <= host_reset_cnt + 1;
      end if;
    end if;
    host_reset_old := host_reset_ramclk;
    -- __DEBUG
      
  end process host_reset_ramclk_p;
  
  host_reset_pciclk_p: process
  begin
    wait until rising_edge(pci_clk);
    host_reset_pciclk <= host_reset_ramclk; -- path to host reset pciclk is TIG
  end process host_reset_pciclk_p;
  
  inbuffer_i : entity work.FIFO_FWFT_async_512x256_Wrapper
    port map(
      wr_clk             => pci_clk,
      wr_rst             => pci_clk_sync_reset,
      rd_clk             => ram_clk0,
      rd_rst             => dram_clk_sync_reset,
      din(255 downto 0)  => dma_din0_tdata, --_conv,
      wr_en              => inbuffer_wr_en,
      rd_en              => inbuffer_rd_en,
      dout(255 downto 0) => inbuffer_dout,
      full               => inbuffer_full,
      empty              => inbuffer_empty
    );
  inbuffer_wr_en  <= dma_din0_tvalid and not inbuffer_full;
  dma_din0_tready <= not inbuffer_full;

  io_p : process
    variable rawinit_acked         : std_logic             := '1';
    variable incounter             : unsigned(31 downto 0) := (others => '0');
    variable last_raw_gt_word_tig  : unsigned(31 downto 0);
  begin
    wait until rising_edge(ram_clk0);

    inbuffer_rd_en  <= '0';
    start_ctables   <= '0';
--    set_num_samples <= '0';
--    set_num_snps    <= '0';
    
    last_raw_gt_word  <= last_raw_gt_word_tig;
    
    if dram_clk_sync_reset = '1' then
      
      indata_state <= WAIT_FOR_START;
      
    elsif inbuffer_empty = '0' and inbuffer_rd_en = '0' then
      case indata_state is
        when WAIT_FOR_START =>
          inbuffer_rd_en <= '1';
          if inbuffer_dout(31 downto 0) = START_WORD then
            indata_state <= CONSTANTS1;
          end if;

        when CONSTANTS1 =>
          inbuffer_rd_en  <= '1';
--          set_num_samples <= '1';
--          set_num_snps <= '1';

          num_samples_rounded_tig <= unsigned(inbuffer_dout(31 downto 0));
          num_cases_rounded_tig   <= unsigned(inbuffer_dout(63 downto 32));
          num_snps_local_tig  <= unsigned(inbuffer_dout(95 downto 64));

          last_init_snpidx_tig <= unsigned(inbuffer_dout(159 downto 128));
          stream_start_snpidx_tig <= unsigned(inbuffer_dout(191 downto 160));
          
          stream_start_addr_tig <= unsigned(inbuffer_dout(221 downto 192)); -- 30bit!
          round_addr_offset_tig <= unsigned(inbuffer_dout(253 downto 224)); -- 30bit!
          
          indata_state <= CONSTANTS2;
          
        when CONSTANTS2 =>
          inbuffer_rd_en        <= '1';
          last_raw_gt_word_tig  := unsigned(inbuffer_dout(31 downto 0));
          ctable_io_bufsize_outwords_tig_ramclk <= unsigned(inbuffer_dout(95 downto 64));
          ctable_io_bufsize_tablewords_tig_ramclk <= unsigned(inbuffer_dout(127 downto 96));
          
          indata_state          <= SNP_DATA;

        when SNP_DATA =>
          -- raw genotype data
          if rawinit_acked = '1' then
            rawinit_raw_genotype_new <= '1';
            rawinit_acked            := '0';
          end if;
          
          if rawinit_raw_genotype_ack = '1' then
            rawinit_acked            := '1';
            rawinit_raw_genotype_new <= '0';
            inbuffer_rd_en           <= '1';
            if incounter = last_raw_gt_word then
              -- got all raw data, start the process and set the state machine back to await new data
              start_ctables <= '1';
              indata_state  <= WAIT_FOR_START;
              incounter     := (others => '0');
            else
              incounter := incounter + 1;
            end if;
          end if;
      end case;

    end if;
    
    -- DEBUG
    if inbuffer_rd_en = '1' then
      inbuffer_rd_cnt <= inbuffer_rd_cnt + 1;
    end if;
    -- __DEBUG
    
  end process io_p;

  pf_sync_p: process
    variable process_finished_tig : std_logic := '1';
  begin
    wait until rising_edge(ram_clk0);
    process_finished_ramclk <= process_finished_tig;
    process_finished_tig := process_finished;
  end process pf_sync_p;
  
  rawinit_raw_genotype <= inbuffer_dout;

  rawinit_reset_p: process
    variable pf_last : std_logic := '1';
  begin
    wait until rising_edge(ram_clk0);
    
    rawinit_reset <= '0';
    
    if dram_clk_sync_reset = '1' then
      pf_last := '1';
    else
      if process_finished_ramclk = '1' and pf_last = '0' then
        rawinit_reset <= '1';
      end if;
      pf_last := process_finished_ramclk;
    end if;
    
  end process rawinit_reset_p;
  
  -- raw genotype data initialization
  rawinit_i : entity work.SNP_RawInit
    port map(
      clk                  => ram_clk0,
      reset                => rawinit_reset or dram_clk_sync_reset,

      -- constant initialization
      num_samples_rounded_tig_in => num_samples_rounded_tig,

      -- raw genotype data
      raw_genotype_in      => rawinit_raw_genotype,
      raw_genotype_new_in  => rawinit_raw_genotype_new,
      raw_genotype_ack_out => rawinit_raw_genotype_ack,

      -- memory interface (write only)
      dram_addr            => rawinit_dram_addr,
      dram_en              => rawinit_dram_en,
      dram_rdy             => rawinit_dram_rdy,
      dram_wdf_data        => rawinit_dram_wdf_data,
      dram_wdf_end         => rawinit_dram_wdf_end,
      dram_wdf_rdy         => rawinit_dram_wdf_rdy,
      dram_wdf_wren        => rawinit_dram_wdf_wren
    );

  -- memory access
  c0_app_addr                   <= rawinit_dram_addr when rawinit_dram_en = '1' else snpreader_dram_addr;
  c0_app_cmd                    <= MEM_CMD_WR when rawinit_dram_en = '1' else MEM_CMD_RD;
  c0_app_en                     <= rawinit_dram_en or snpreader_dram_en;
  rawinit_dram_rdy              <= c0_app_rdy;
  snpreader_dram_rdy            <= c0_app_rdy;
  c0_app_wdf_data(511 downto 0) <= rawinit_dram_wdf_data;
  c0_app_wdf_data(575 downto 512) <= (others => '0');
  --c0_app_wdf_mask               <= (others => '0'); -- mask is not used
  c0_app_wdf_end                <= rawinit_dram_wdf_end;
  rawinit_dram_wdf_rdy          <= c0_app_wdf_rdy;
  c0_app_wdf_wren               <= rawinit_dram_wdf_wren;
  snpreader_dram_rd_data        <= c0_app_rd_data(511 downto 0);
  snpreader_dram_rd_data_end    <= c0_app_rd_data_end;
  snpreader_dram_rd_data_valid  <= c0_app_rd_data_valid;
  
  -- c1 is not used
  c1_app_addr     <= (others => '0');
  c1_app_cmd      <= (others => '0');
  c1_app_en       <= '0';
  c1_app_wdf_data <= (others => '0');
  --c1_app_wdf_mask <= (others => '0');
  c1_app_wdf_end  <= '0';
  c1_app_wdf_wren <= '0'; 

  snpreader_i : entity work.SNP_Reader
    port map(
      ram_clk            => ram_clk0,
      stream_clk         => stream_clk,
      ram_clk_reset      => dram_clk_sync_reset,
      stream_clk_reset   => stream_clk_sync_reset,
      num_samples_rounded_tig_in     => num_samples_rounded_tig,
--      set_num_samples_in => set_num_samples,
      num_cases_rounded_tig_in       => num_cases_rounded_tig,
--      set_num_cases_in   => set_num_samples,
      num_snps_local_tig_in => num_snps_local_tig,
      last_init_snpidx_tig_in => last_init_snpidx_tig,
      stream_start_snpidx_tig_in => stream_start_snpidx_tig,
      stream_start_addr_tig_in => stream_start_addr_tig,
      round_addr_offset_tig_in => round_addr_offset_tig,
--      set_num_snps_in    => set_num_snps,
      start_reading_in   => snpreader_start_reading,
      genotype_out       => snpreader_genotype,
      casenctrl_out      => snpreader_casenctrl,
      new_genotype_out   => snpreader_new_genotype,
      snp_done_out       => snpreader_snp_done,
      round_done_out     => snpreader_round_done,
      busy_out           => snpreader_busy,
      stall_in           => stall,
      dram_addr          => snpreader_dram_addr,
      dram_en            => snpreader_dram_en,
      dram_rdy           => snpreader_dram_rdy,
      dram_rd_data       => snpreader_dram_rd_data,
      dram_rd_data_end   => snpreader_dram_rd_data_end,
      dram_rd_data_valid => snpreader_dram_rd_data_valid,
      dbg_out            => snpreader_debug
    );
  snpreader_start_reading <= start_ctables;

  -- generate processing element chains

  ctchain_genotype(0)     <= snpreader_genotype;
  ctchain_casenctrl(0)    <= snpreader_casenctrl;
  ctchain_new_genotype(0) <= snpreader_new_genotype;
  ctchain_mask(0)         <= '0';
  ctchain_snp_done(0)     <= snpreader_snp_done;
  ctchain_round_done(0)   <= snpreader_round_done;

  chain_g : for C in 0 to NUM_CHAINS - 1 generate
    ctable_chain_i : entity work.CTableChain
      generic map(CHAIN_ID => C)
      port map(
        stream_clk           => stream_clk,
        stream_clk_reset     => stream_clk_sync_reset,
        table_read_clk       => pci_clk,
        table_read_clk_reset => pci_clk_sync_reset,
        genotype_in          => ctchain_genotype(C),
        casenctrl_in         => ctchain_casenctrl(C),
        new_genotype_in      => ctchain_new_genotype(C),
        mask_in              => ctchain_mask(C),
        snp_done_in          => ctchain_snp_done(C),
        round_done_in        => ctchain_round_done(C),
        genotype_out         => ctchain_genotype(C + 1),
        casenctrl_out        => ctchain_casenctrl(C + 1),
        new_genotype_out     => ctchain_new_genotype(C + 1),
        mask_out             => ctchain_mask(C + 1),
        snp_done_out         => ctchain_snp_done(C + 1),
        round_done_out       => ctchain_round_done(C + 1),
        casetable_ready_out  => ctchain_casetable_ready(C),
        casetable_read_in    => ctchain_casetable_read(C),
        casetable_out        => ctchain_casetable(C),
        ctrltable_ready_out  => ctchain_ctrltable_ready(C),
        ctrltable_read_in    => ctchain_ctrltable_read(C),
        ctrltable_out        => ctchain_ctrltable(C),
        table_ov_out         => ctchain_table_ov(C),
        table_row_done_out   => ctchain_table_row_done(C),
        table_round_done_out => ctchain_table_round_done(C),
        stall_out            => ctchain_stall(C),
        dbg_out              => ctchain_debug(C)
      );

  end generate chain_g;

  stall_p : process
    variable stall_tmp : std_logic := '0';
  begin
    wait until rising_edge(stream_clk);

    stall_tmp := '0';
    for C in 0 to NUM_CHAINS - 1 loop
      stall_tmp := stall_tmp or ctchain_stall(C);
    end loop;
    stall <= stall_tmp;

  end process stall_p;

-- ID calculation
calc_id_p: process
  type id_st_vector is array (0 to NUM_CHAINS-1) of integer;
  variable idA : id_st_vector;
  variable idBm1 : id_st_vector; -- always one behind the actual ID: idB - 1
  variable curr_first_idA : id_st_vector; -- the first idA of the current round for each chain
  variable next_round_first_idAm1 : id_st_vector; -- the first idA of the next round for the _first_ chain, but stored separately for each chain, and minus 1!
  variable stream_start_snpidx_m1 : integer;
  variable stream_start_snpidx_gt0 : boolean;
begin
  wait until rising_edge(pci_clk);
  
  if process_finished = '1' then
    
    -- initialization of IDs
    for C in 0 to NUM_CHAINS-1 loop
      curr_first_idA(C) := C * NUM_PE_PER_CHAIN;
      idA(C) := curr_first_idA(C);
--      idB(C) := curr_first_idA(C) + 1;
      idBm1(C) := curr_first_idA(C);
      next_round_first_idAm1(C) := NUM_PE-1;
    end loop;
    
  else
    
    -- for all chains
    for C in 0 to NUM_CHAINS-1 loop
    
      -- calculate ID for next data in this chain 
      if ctchain_table_read(C) = '1' then -- table has been read: calculate next ID
      
        if ctchain_table_row_done(C) = '1' then -- "row done"
          
          if ctchain_table_round_done(C) = '1' then -- also "round done"
            -- buffer A has been filled 
            curr_first_idA(C) := curr_first_idA(C) + NUM_PE;            
            -- reset B
--            idB(C) := curr_first_idA(C) + 1;
            idBm1(C) := curr_first_idA(C);
            next_round_first_idAm1(C) := next_round_first_idAm1(C) + NUM_PE;
          else -- "row done" alone
            -- increment B
            idBm1(C) := idBm1(C) + 1;
            -- jump if SNPReader also jumped here#
            -- apparently a simple signed comparison does not work here... need to check also if stream_start_snpidx is not zero
            if stream_start_snpidx_gt0 and idBm1(C) = next_round_first_idAm1(C) and next_round_first_idAm1(C) < stream_start_snpidx_m1 then
              idBm1(C) := stream_start_snpidx_m1;
            end if;
          end if;
          -- reset A
          idA(C) := curr_first_idA(C);
          
        else -- not "row done"
          
          -- incrememt A
          idA(C) := idA(C) + 1;
          
        end if;
      
      end if;
      
      -- update ID variable
      ctchain_id(C)(0) <= std_logic_vector(to_unsigned(idA(C), ID_WIDTH));
      ctchain_id(C)(1) <= std_logic_vector(to_unsigned(idBm1(C), ID_WIDTH));
      
    end loop;
    
  end if;
  
  stream_start_snpidx_m1 := to_integer(stream_start_snpidx_tig)-1;
  stream_start_snpidx_gt0 := to_integer(stream_start_snpidx_tig) > 0;
  
end process calc_id_p;

ctchain_casetable_read <= ctchain_table_read;
ctchain_ctrltable_read <= ctchain_table_read;
ctchain_table_ready <= ctchain_casetable_ready and ctchain_ctrltable_ready;


out_map_p: process
  type count_vector is array (natural range <>) of unsigned(31 downto 0);
  -- tables prepared for the transmission buffer
  variable tbuf_table_count : count_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
  -- transmission words inserted into transmission buffer
  variable tbuf_word_count : count_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
  
  -- for final padding
  variable not_busy_timer : integer range 0 to NOT_BUSY_DELAY := 0;
  variable snpreader_busy_tig : std_logic := '0';
  
begin
  wait until rising_edge(pci_clk);
  
  ctchain_table_read <= (others => '0');
  ctable_io_bufsize_outwords_tig_pciclk <= ctable_io_bufsize_outwords_tig_ramclk;
  ctable_io_bufsize_tablewords_tig_pciclk <= ctable_io_bufsize_tablewords_tig_ramclk;
  
  if pci_clk_sync_reset = '1' then 
    
    dma_ins_pad <= (others => '0');
    tbuf_table_count := (others => (others => '0'));
    tbuf_table_count_sig <= (others => (others => '0'));
    tbuf_word_count := (others => (others => '0'));
    tbuf_word_count_sig <= (others => (others => '0'));
    not_busy_timer := 0;
    width_conv_tlast <= (others  => '0');
    process_finished <= '1'; 
    
  else
  
    -- DEBUG
    -- If we get input data, that means another iteration has started. So, reset the transmission counters.
    if dma_din0_tvalid = '1' then
      tbuf_table_count_sig <= (others => (others => '0'));
      tbuf_word_count_sig <= (others => (others => '0'));
    end if; 
    -- __DEBUG
    
    for C in 0 to NUM_CHAINS-1 loop
    
      if (ctchain_table_ready(C) = '1' or 
        (process_finished = '1' and tbuf_table_count(C) /= (tbuf_table_count'range => '0')))
        -- process_finished will also be set during a host reset 
      and ctchain_table_read(C) = '0'
      and width_conv_tlast(C) = '0'
      and width_conv_ready(C) = '1' then 
        
        ctchain_table_read(C) <= '1'; -- keep in mind that this also triggers table ID update 
        tbuf_table_count(C) := tbuf_table_count(C) + 1;
        -- DEBUG
        tbuf_table_count_sig(C) <= tbuf_table_count_sig(C) + 1;
        -- __DEBUG
        
        if tbuf_table_count(C) = ctable_io_bufsize_tablewords_tig_pciclk then
        -- transmission full now (or a padding word has been inserted that indicated that the process has finished)
          width_conv_tlast(C) <= '1';
        end if;
      end if;
      
      if (dma_dout_tvalid(C) = '1' or dma_ins_pad(C) = '1') and dma_dout_tready(C) = '1' then
                
        if dma_dout_tvalid(C) = '1' and width_conv_islast(C) = '1' then
        -- have to fill up buffer now
          dma_ins_pad(C) <= '1';
        end if;
        
        tbuf_word_count(C) := tbuf_word_count(C) + 1;
        -- DEBUG
        tbuf_word_count_sig(C) <= tbuf_word_count_sig(C) + 1;
        -- __DEBUG 
        if tbuf_word_count(C) = ctable_io_bufsize_outwords_tig_pciclk then
        -- finished padding insertion
          tbuf_word_count(C) := (others => '0');
          tbuf_table_count(C) := (others => '0');
          width_conv_tlast(C) <= '0';
          dma_ins_pad(C) <= '0';
        end if;
      end if;
        
    end loop;
    
    -- after finishing the SNP reading process, start timer
    if snpreader_busy_tig = '1' then
      not_busy_timer := NOT_BUSY_DELAY;
      process_finished <= '0';
    else 
      if not_busy_timer /= 0 then
        not_busy_timer := not_busy_timer - 1;
      else
        process_finished <= '1';
      end if;
    end if;
  
  end if;
  
  snpreader_busy_tig := snpreader_busy;
  
end process out_map_p;

chain_out_g: for C in 0 to NUM_CHAINS-1 generate

-- build table from chain output + ID
table(C) <= ctchain_id(C)(0)
          & ctchain_id(C)(1)
          & ctchain_casetable(C)
          & ctchain_ctrltable(C)
          when process_finished = '0' else (others => '0'); -- for a clean padding

-- output mapping:
-- 2x4 counters + ID have to be mapped to 256 bit DMA output
-- each table is on 16 bit (i.e. 2 bytes) boundaries 
-- first port is larger than necessary since converter unit produces timing errors otherwise
width_conv_i: entity work.conv_352_256_Wrapper
  port map(
    aclk          => pci_clk,
    aresetn       => not pci_clk_sync_reset,
    s_axis_tvalid => ctchain_table_read(C),
    s_axis_tready => width_conv_ready(C),
    s_axis_tdata  => table(C), --table_dbg(C),
    s_axis_tlast  => width_conv_tlast(C),
    m_axis_tvalid => dma_dout_tvalid(C),
    m_axis_tready => dma_dout_tready(C),
    m_axis_tdata  => dma_dout_tdata(C),
    m_axis_tkeep  => open,
    m_axis_tlast  => width_conv_islast(C)
  );  
  
end generate chain_out_g;

-- map DMA channels to PE chains
dma_dout1_tdata <= dma_dout_tdata(0);
dma_dout2_tdata <= dma_dout_tdata(1);

-- send data whenever it is desired, padding is required or the DMA engine is ready during reset
dma_dout1_tvalid <= (dma_dout_tvalid(0) or dma_ins_pad(0) or host_reset_pciclk) and dma_dout_tready(0);
dma_dout2_tvalid <= (dma_dout_tvalid(1) or dma_ins_pad(1) or host_reset_pciclk) and dma_dout_tready(1);

dma_dout_tready(0) <= dma_dout1_tready;
dma_dout_tready(1) <= dma_dout2_tready;

status(0) <= snpreader_busy;
status(1) <= not process_finished_ramclk;
status(2) <= not inbuffer_empty;
status(3) <= inbuffer_full;
status(4) <= '0';
status(5) <= not dma_dout1_tready;
status(6) <= not dma_dout2_tready;
status(7) <= host_reset_ramclk;

reg_we <= reg_we_dbg;
reg_addr <= reg_addr_dbg when reg_we_dbg = '1' else (others => '0');
reg_dout <= reg_dout_dbg;

  -- DEBUG
  snpreader_dbg_p : process
  begin
    wait until rising_edge(stream_clk);
    
    -- reset counters every iteration
    if snpreader_start_reading = '1' then
--      snpreader_new_genotype_cnt     <= (others => '0');
      snpreader_snp_done_cnt   <= (others => '0');
      snpreader_round_done_cnt <= (others => '0');
    end if;

    if snpreader_snp_done = '1' then
      snpreader_snp_done_cnt <= snpreader_snp_done_cnt + 1;
    end if;

    if snpreader_round_done = '1' then
      snpreader_round_done_cnt <= snpreader_round_done_cnt + 1;
    end if;
    
    if stall = '1' then
      stall_cnt <= stall_cnt + 1;
    end if;
  end process snpreader_dbg_p;
--
  mem_dbg_p : process
  begin
    wait until rising_edge(ram_clk0);

    if rawinit_dram_en = '0' and snpreader_dram_en = '1' then
      c0_rd_req_cnt <= c0_rd_req_cnt + 1;
    end if;

    if rawinit_dram_en = '1' and snpreader_dram_en = '1' then
      c0_conflict_cnt <= c0_conflict_cnt + 1;
    end if;

    if c0_app_rd_data_valid = '1' then
      c0_rd_ans_cnt <= c0_rd_ans_cnt + 1;
    end if;

    c0_rd_ack_cnt <= unsigned(snpreader_debug(47 downto 0));

  end process mem_dbg_p;
  
  reg_dbg_p : process
    variable timer                      : unsigned(26 downto 0) := (others => '1');
    variable dbg_wr_cnt                 : unsigned(63 downto 0) := (others => '0');
    variable dbg_state                  : integer range 0 to 7          := 0;
    
    variable c0_rd_req_cnt_tig   : unsigned(47 downto 0) := (others => '0');
    variable c0_rd_ans_cnt_tig   : unsigned(47 downto 0) := (others => '0');
    variable c0_rd_ack_cnt_tig   : unsigned(47 downto 0) := (others => '0');
    variable c0_conflict_cnt_tig : unsigned(15 downto 0) := (others => '0');
    
    variable ctchain_debug_tig : debug_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
    variable tbuf_table_count_tig : dbg_count_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
    variable tbuf_word_count_tig : dbg_count_vector(NUM_CHAINS-1 downto 0) := (others => (others => '0'));
    
  begin
    wait until rising_edge(stream_clk);

    reg_we_dbg   <= '0';
    reg_dout_dbg <= (others => '0');

      case dbg_state is

        when 0 =>  
          reg_we_dbg                   <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state                := dbg_state + 1;
          
          -- to identify endianess
          reg_dout_dbg                 <= x"fffefdfcfbfaf9f8f7f6f5f4f3f2f1f00f0e0d0c0b0a09080706050403020100";
          reg_dout_dbg(63 downto 0)    <= std_logic_vector(dbg_wr_cnt); -- number of debug writes
          dbg_wr_cnt               := dbg_wr_cnt + 1;

        when 1 => 
          reg_we_dbg                   <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state                := dbg_state + 1;
          
          reg_dout_dbg(47 downto 32)   <= std_logic_vector(host_reset_cnt);
          reg_dout_dbg(95 downto 48)   <= std_logic_vector(host_reset_cycle_cnt);
          reg_dout_dbg(127 downto 96)  <= std_logic_vector(inbuffer_rd_cnt);            
          reg_dout_dbg(159 downto 128) <= std_logic_vector(ctable_io_bufsize_tablewords_tig_ramclk);
          reg_dout_dbg(191 downto 160)  <= std_logic_vector(ctable_io_bufsize_outwords_tig_ramclk);

        when 2 => 
          reg_we_dbg                   <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state                := dbg_state + 1;
          
          reg_dout_dbg(95 downto  64) <= std_logic_vector(num_cases_rounded_tig);
          reg_dout_dbg(63 downto  32) <= std_logic_vector(num_samples_rounded_tig);
          reg_dout_dbg(31 downto   0) <= std_logic_vector(num_snps_local_tig);
          
          reg_dout_dbg(253 downto 224) <= std_logic_vector(round_addr_offset_tig);
          reg_dout_dbg(221 downto 192) <= std_logic_vector(stream_start_addr_tig);
          reg_dout_dbg(191 downto 160) <= std_logic_vector(stream_start_snpidx_tig);
          reg_dout_dbg(159 downto 128) <= std_logic_vector(last_init_snpidx_tig);
  
        when 3 => 
          reg_we_dbg                  <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state                := dbg_state + 1;
          
          --  47.. 0 case_wr chain 0
          -- 111..64 ctrl_wr chain 0
          reg_dout_dbg(127 downto 0) <= ctchain_debug_tig(0);
          reg_dout_dbg(175 downto 128) <= std_logic_vector(tbuf_table_count_tig(0));
          reg_dout_dbg(239 downto 192) <= std_logic_vector(tbuf_word_count_tig(0) );
          
        when 4 => 
          reg_we_dbg                   <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state                := dbg_state + 1;
          
          --  47.. 0 case_wr chain 1
          -- 111..64 ctrl_wr chain 1
          reg_dout_dbg(127 downto 0) <= ctchain_debug_tig(1);
          reg_dout_dbg(175 downto 128) <= std_logic_vector(tbuf_table_count_tig(1));
          reg_dout_dbg(239 downto 192) <= std_logic_vector(tbuf_word_count_tig(1) );

        when 5 => 
          reg_we_dbg                   <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state                := dbg_state + 1;
          
          reg_dout_dbg(239 downto 192) <= std_logic_vector(snpreader_snp_done_cnt);
          reg_dout_dbg(175 downto 128) <= std_logic_vector(snpreader_round_done_cnt);
          reg_dout_dbg(63 downto 0)    <= std_logic_vector(stall_cnt);

        when 6 => 
          reg_we_dbg                 <= '1';
          reg_addr_dbg             <= std_logic_vector(to_unsigned(dbg_state+1, 10)); -- debug register address
          dbg_state              := dbg_state + 1;
          
          reg_dout_dbg(47 downto 0)    <= std_logic_vector(c0_rd_req_cnt_tig);
          reg_dout_dbg(111 downto 64)  <= std_logic_vector(c0_rd_ans_cnt_tig);
          reg_dout_dbg(175 downto 128) <= std_logic_vector(c0_rd_ack_cnt_tig);
          reg_dout_dbg(207 downto 192) <= std_logic_vector(c0_conflict_cnt_tig);
          

        when others =>
            if timer = 0 then
              dbg_state := 0;
              timer     := (others => '1');
            else
              timer := timer - 1;
            end if;

      end case;

    c0_rd_req_cnt_tig   := c0_rd_req_cnt;
    c0_rd_ans_cnt_tig   := c0_rd_ans_cnt;
    c0_rd_ack_cnt_tig   := c0_rd_ack_cnt;
    c0_conflict_cnt_tig := c0_conflict_cnt;
    
    ctchain_debug_tig := ctchain_debug;
    tbuf_table_count_tig := tbuf_table_count_sig;
    tbuf_word_count_tig := tbuf_word_count_sig;    
    
  end process reg_dbg_p;

  reg_clk <= stream_clk;
  
-- __DEBUG

end Behavioral;
