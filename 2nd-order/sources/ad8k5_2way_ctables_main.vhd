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
    ram_clk0             : in  std_logic; -- booth ram clocks must be the same
    ram_clk0_reset       : in  std_logic;
    ram_clk1             : in  std_logic; -- booth ram clocks must be the same
    ram_clk1_reset       : in  std_logic;
    -- PCIe DMA
    dma_din0_tdata       : in  std_logic_vector(255 downto 0);
    dma_din0_tvalid      : in  std_logic;
    dma_din0_tready      : out std_logic;
    dma_dout1_tdata      : out std_logic_vector(255 downto 0);
    dma_dout1_tvalid     : out std_logic;
    dma_dout1_tready     : in  std_logic;
    dma_din2_tdata       : in  std_logic_vector(255 downto 0);
    dma_din2_tvalid      : in  std_logic;
    dma_din2_tready      : out std_logic;
    dma_dout3_tdata      : out std_logic_vector(255 downto 0);
    dma_dout3_tvalid     : out std_logic;
    dma_dout3_tready     : in  std_logic;
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

  signal host_reset_sig : std_logic_vector(NUM_ENGINES - 1 downto 0);

  -- Status
  signal status_snpreader_busy    : std_logic_vector(NUM_ENGINES - 1 downto 0);
  signal status_process_finished  : std_logic_vector(NUM_ENGINES - 1 downto 0);
  signal status_inbuffer_empty    : std_logic_vector(NUM_ENGINES - 1 downto 0);
  signal status_inbuffer_full     : std_logic_vector(NUM_ENGINES - 1 downto 0);
  signal status_dma_out_ready     : std_logic_vector(NUM_ENGINES - 1 downto 0);
  signal status_host_reset_ramclk : std_logic_vector(NUM_ENGINES - 1 downto 0);
  signal dbg_status_0 : std_logic_vector(7 downto 0);
  signal dbg_status_1 : std_logic_vector(7 downto 0);

  -- DEBUG
  signal reg_we_intern     : std_logic;
  signal reg_we_intern_tig : std_logic;
  signal reg_addr_intern   : std_logic_vector(9 downto 0) := (others => '0');
  signal reg_dout_intern   : std_logic_vector(255 downto 0);

  signal tbuf_table_count_sig_0 : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));
  signal tbuf_word_count_sig_0  : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));
  signal tbuf_table_count_sig_1 : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));
  signal tbuf_word_count_sig_1  : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));

  signal snpreader_round_done_cnt_0 : unsigned(47 downto 0) := (others => '0');
  signal snpreader_snp_done_cnt_0   : unsigned(47 downto 0) := (others => '0');
  signal snpreader_round_done_cnt_1 : unsigned(47 downto 0) := (others => '0');
  signal snpreader_snp_done_cnt_1   : unsigned(47 downto 0) := (others => '0');
  signal stall_cnt_0                : unsigned(63 downto 0) := (others => '0');
  signal stall_cnt_1                : unsigned(63 downto 0) := (others => '0');

  signal inbuffer_rd_cnt_0 : unsigned(31 downto 0) := (others => '0');
  signal inbuffer_rd_cnt_1 : unsigned(31 downto 0) := (others => '0');

  signal dram_rd_req_cnt_0   : unsigned(47 downto 0) := (others => '0');
  signal dram_rd_ans_cnt_0   : unsigned(47 downto 0) := (others => '0');
  signal dram_conflict_cnt_0 : unsigned(15 downto 0) := (others => '0');
  signal dram_rd_req_cnt_1   : unsigned(47 downto 0) := (others => '0');
  signal dram_rd_ans_cnt_1   : unsigned(47 downto 0) := (others => '0');
  signal dram_conflict_cnt_1 : unsigned(15 downto 0) := (others => '0');

  signal host_reset_cycle_cnt_0 : unsigned(47 downto 0) := (others => '0');
  signal host_reset_cnt_0       : unsigned(15 downto 0) := (others => '0');
  signal host_reset_cycle_cnt_1 : unsigned(47 downto 0) := (others => '0');
  signal host_reset_cnt_1       : unsigned(15 downto 0) := (others => '0');

  signal ctable_io_bufsize_outwords_tig_ramclk_0   : unsigned(31 downto 0); -- directly sampled from input with TIG constraint "to" in ramclk domain
  signal ctable_io_bufsize_tablewords_tig_ramclk_0 : unsigned(31 downto 0); -- directly sampled from input with TIG constraint "to" in ramclk domain
  signal ctable_io_bufsize_outwords_tig_ramclk_1   : unsigned(31 downto 0); -- directly sampled from input with TIG constraint "to" in ramclk domain
  signal ctable_io_bufsize_tablewords_tig_ramclk_1 : unsigned(31 downto 0); -- directly sampled from input with TIG constraint "to" in ramclk domain

  signal num_samples_rounded_tig_0 : unsigned(31 downto 0);
  signal num_cases_rounded_tig_0   : unsigned(31 downto 0);
  signal num_snps_local_tig_0      : unsigned(31 downto 0);
  signal last_init_snpidx_tig_0    : unsigned(31 downto 0); --inclusive
  signal stream_start_snpidx_tig_0 : unsigned(31 downto 0);
  signal stream_start_addr_tig_0   : unsigned(29 downto 0);
  signal round_addr_offset_tig_0   : unsigned(29 downto 0);
  signal num_samples_rounded_tig_1 : unsigned(31 downto 0);
  signal num_cases_rounded_tig_1   : unsigned(31 downto 0);
  signal num_snps_local_tig_1      : unsigned(31 downto 0);
  signal last_init_snpidx_tig_1    : unsigned(31 downto 0); --inclusive
  signal stream_start_snpidx_tig_1 : unsigned(31 downto 0);
  signal stream_start_addr_tig_1   : unsigned(29 downto 0);
  signal round_addr_offset_tig_1   : unsigned(29 downto 0);

begin

  engine_0 : entity work.Engine
    port map(
      reset                                       => reset,
      pci_clk                                     => pci_clk,
      ram_clk                                     => ram_clk0, -- clk for c0

      -- PCIe DMA access
      dma_din_tdata                               => dma_din0_tdata,
      dma_din_tvalid                              => dma_din0_tvalid,
      dma_din_tready                              => dma_din0_tready,
      dma_dout_tdata                              => dma_dout1_tdata,
      dma_dout_tvalid                             => dma_dout1_tvalid,
      dma_dout_tready                             => dma_dout1_tready,
      host_reset_sig                              => host_reset_sig(0),
      -- RAM interface
      dram_addr                                   => c0_app_addr,
      dram_cmd                                    => c0_app_cmd,
      dram_en                                     => c0_app_en,
      dram_wdf_data                               => c0_app_wdf_data,
      dram_wdf_end                                => c0_app_wdf_end,
      dram_wdf_wren                               => c0_app_wdf_wren,
      dram_rd_data                                => c0_app_rd_data,
      dram_rd_data_end                            => c0_app_rd_data_end,
      dram_rd_data_valid                          => c0_app_rd_data_valid,
      dram_rdy                                    => c0_app_rdy,
      dram_wdf_rdy                                => c0_app_wdf_rdy,
      -- status
      status_snpreader_busy                       => status_snpreader_busy(0),
      status_process_finished                     => status_process_finished(0),
      status_inbuffer_empty                       => status_inbuffer_empty(0),
      status_inbuffer_full                        => status_inbuffer_full(0),
      status_dma_out_ready                        => status_dma_out_ready(0),
      status_host_reset_ramclk                    => status_host_reset_ramclk(0),
      -- DBUG signals
      dbg_tbuf_table_count                        => tbuf_table_count_sig_0,
      dbg_tbuf_word_count                         => tbuf_word_count_sig_0,
      dbg_snpreader_round_done_cnt                => snpreader_round_done_cnt_0,
      dbg_snpreader_snp_done_cnt                  => snpreader_snp_done_cnt_0,
      dbg_stall_cnt                               => stall_cnt_0,
      dbg_inbuffer_rd_cnt                         => inbuffer_rd_cnt_0,
      dbg_dram_rd_req_cnt                         => dram_rd_req_cnt_0,
      dbg_dram_rd_ans_cnt                         => dram_rd_ans_cnt_0,
      dbg_dram_conflict_cnt                       => dram_conflict_cnt_0,
      dbg_host_reset_cycle_cnt                    => host_reset_cycle_cnt_0,
      dbg_host_reset_cnt                          => host_reset_cnt_0,
      dbg_ctable_io_bufsize_outwords_tig_ramclk   => ctable_io_bufsize_outwords_tig_ramclk_0,
      dbg_ctable_io_bufsize_tablewords_tig_ramclk => ctable_io_bufsize_tablewords_tig_ramclk_0,
      dbg_num_samples_rounded_tig                 => num_samples_rounded_tig_0,
      dbg_num_cases_rounded_tig                   => num_cases_rounded_tig_0,
      dbg_num_snps_local_tig                      => num_snps_local_tig_0,
      dbg_last_init_snpidx_tig                    => last_init_snpidx_tig_0,
      dbg_stream_start_snpidx_tig                 => stream_start_snpidx_tig_0,
      dbg_stream_start_addr_tig                   => stream_start_addr_tig_0,
      dbg_round_addr_offset_tig                   => round_addr_offset_tig_0
    );

  engine_1 : entity work.Engine
    port map(
      reset                                       => reset,
      pci_clk                                     => pci_clk,
      ram_clk                                     => ram_clk1, -- clk for c1

      -- PCIe DMA access
      dma_din_tdata                               => dma_din2_tdata,
      dma_din_tvalid                              => dma_din2_tvalid,
      dma_din_tready                              => dma_din2_tready,
      dma_dout_tdata                              => dma_dout3_tdata,
      dma_dout_tvalid                             => dma_dout3_tvalid,
      dma_dout_tready                             => dma_dout3_tready,
      host_reset_sig                              => host_reset_sig(1),
      -- RAM interface
      dram_addr                                   => c1_app_addr,
      dram_cmd                                    => c1_app_cmd,
      dram_en                                     => c1_app_en,
      dram_wdf_data                               => c1_app_wdf_data,
      dram_wdf_end                                => c1_app_wdf_end,
      dram_wdf_wren                               => c1_app_wdf_wren,
      dram_rd_data                                => c1_app_rd_data,
      dram_rd_data_end                            => c1_app_rd_data_end,
      dram_rd_data_valid                          => c1_app_rd_data_valid,
      dram_rdy                                    => c1_app_rdy,
      dram_wdf_rdy                                => c1_app_wdf_rdy,
      -- status
      status_snpreader_busy                       => status_snpreader_busy(1),
      status_process_finished                     => status_process_finished(1),
      status_inbuffer_empty                       => status_inbuffer_empty(1),
      status_inbuffer_full                        => status_inbuffer_full(1),
      status_dma_out_ready                        => status_dma_out_ready(1),
      status_host_reset_ramclk                    => status_host_reset_ramclk(1),
      -- DBUG signals
      dbg_tbuf_table_count                        => tbuf_table_count_sig_1,
      dbg_tbuf_word_count                         => tbuf_word_count_sig_1,
      dbg_snpreader_round_done_cnt                => snpreader_round_done_cnt_1,
      dbg_snpreader_snp_done_cnt                  => snpreader_snp_done_cnt_1,
      dbg_stall_cnt                               => stall_cnt_1,
      dbg_inbuffer_rd_cnt                         => inbuffer_rd_cnt_1,
      dbg_dram_rd_req_cnt                         => dram_rd_req_cnt_1,
      dbg_dram_rd_ans_cnt                         => dram_rd_ans_cnt_1,
      dbg_dram_conflict_cnt                       => dram_conflict_cnt_1,
      dbg_host_reset_cycle_cnt                    => host_reset_cycle_cnt_1,
      dbg_host_reset_cnt                          => host_reset_cnt_1,
      dbg_ctable_io_bufsize_outwords_tig_ramclk   => ctable_io_bufsize_outwords_tig_ramclk_1,
      dbg_ctable_io_bufsize_tablewords_tig_ramclk => ctable_io_bufsize_tablewords_tig_ramclk_1,
      dbg_num_samples_rounded_tig                 => num_samples_rounded_tig_1,
      dbg_num_cases_rounded_tig                   => num_cases_rounded_tig_1,
      dbg_num_snps_local_tig                      => num_snps_local_tig_1,
      dbg_last_init_snpidx_tig                    => last_init_snpidx_tig_1,
      dbg_stream_start_snpidx_tig                 => stream_start_snpidx_tig_1,
      dbg_stream_start_addr_tig                   => stream_start_addr_tig_1,
      dbg_round_addr_offset_tig                   => round_addr_offset_tig_1
    );

  -- write the status register
  status(0) <= status_snpreader_busy(0);
  status(1) <= not status_process_finished(0);
  status(2) <= not status_inbuffer_empty(0);
  status(3) <= status_inbuffer_full(0);
  status(4) <= '0';
  status(5) <= not status_dma_out_ready(0);
  status(6) <= '0';
  status(7) <= status_host_reset_ramclk(0);

  dbg_status_0(0) <= status_snpreader_busy(0);
  dbg_status_0(1) <= not status_process_finished(0);
  dbg_status_0(2) <= not status_inbuffer_empty(0);
  dbg_status_0(3) <= status_inbuffer_full(0);
  dbg_status_0(4) <= '0';
  dbg_status_0(5) <= not status_dma_out_ready(0);
  dbg_status_0(6) <= '0';
  dbg_status_0(7) <= status_host_reset_ramclk(0);
  dbg_status_1(0) <= status_snpreader_busy(0);
  dbg_status_1(1) <= not status_process_finished(1);
  dbg_status_1(2) <= not status_inbuffer_empty(1);
  dbg_status_1(3) <= status_inbuffer_full(1);
  dbg_status_1(4) <= '0';
  dbg_status_1(5) <= not status_dma_out_ready(1);
  dbg_status_1(6) <= '0';
  dbg_status_1(7) <= status_host_reset_ramclk(1);

  -- reset the pipeline
  host_reset_sig(0) <= reg_din(0) and not reg_we_intern_tig; -- make sure the reg_din is valid
  host_reset_sig(1) <= reg_din(1) and not reg_we_intern_tig; -- make sure the reg_din is valid

  -- DEBUG

  reg_we   <= reg_we_intern;
  reg_addr <= reg_addr_intern when reg_we_intern = '1' else (others => '0');
  reg_dout <= reg_dout_intern;
  reg_clk  <= ram_clk0;

  reg_dbg_p : process
    variable timer      : unsigned(26 downto 0) := (others => '1');
    variable dbg_wr_cnt : unsigned(63 downto 0) := (others => '0');
    variable dbg_state  : integer range 0 to 13 := 0;

    variable dram_rd_req_cnt_tig_0   : unsigned(47 downto 0) := (others => '0');
    variable dram_rd_ans_cnt_tig_0   : unsigned(47 downto 0) := (others => '0');
    variable dram_conflict_cnt_tig_0 : unsigned(15 downto 0) := (others => '0');
    variable dram_rd_req_cnt_tig_1   : unsigned(47 downto 0) := (others => '0');
    variable dram_rd_ans_cnt_tig_1   : unsigned(47 downto 0) := (others => '0');
    variable dram_conflict_cnt_tig_1 : unsigned(15 downto 0) := (others => '0');

    variable ctchain_debug_tig      : debug_vector(NUM_CHAINS - 1 downto 0)     := (others => (others => '0'));
    variable tbuf_table_count_tig_0 : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));
    variable tbuf_word_count_tig_0  : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));
    variable tbuf_table_count_tig_1 : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));
    variable tbuf_word_count_tig_1  : dbg_count_vector(NUM_CHAINS - 1 downto 0) := (others => (others => '0'));

  begin
    wait until rising_edge(ram_clk0);

    reg_we_intern   <= '0';
    reg_dout_intern <= (others => '0');

    case dbg_state is

      when 0 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        -- to identify endianess
        reg_dout_intern              <= x"fffefdfcfbfaf9f8f7f6f5f4f3f2f1f00f0e0d0c0b0a09080706050403020100";
        reg_dout_intern(63 downto 0) <= std_logic_vector(dbg_wr_cnt); -- number of debug writes
        dbg_wr_cnt                   := dbg_wr_cnt + 1;

      when 1 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(47 downto 32)   <= std_logic_vector(host_reset_cnt_0);
        reg_dout_intern(95 downto 48)   <= std_logic_vector(host_reset_cycle_cnt_0);
        reg_dout_intern(127 downto 96)  <= std_logic_vector(inbuffer_rd_cnt_0);
        reg_dout_intern(159 downto 128) <= std_logic_vector(ctable_io_bufsize_tablewords_tig_ramclk_0);
        reg_dout_intern(191 downto 160) <= std_logic_vector(ctable_io_bufsize_outwords_tig_ramclk_0);

      when 2 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(95 downto 64) <= std_logic_vector(num_cases_rounded_tig_0);
        reg_dout_intern(63 downto 32) <= std_logic_vector(num_samples_rounded_tig_0);
        reg_dout_intern(31 downto 0)  <= std_logic_vector(num_snps_local_tig_0);

        reg_dout_intern(253 downto 224) <= std_logic_vector(round_addr_offset_tig_0);
        reg_dout_intern(221 downto 192) <= std_logic_vector(stream_start_addr_tig_0);
        reg_dout_intern(191 downto 160) <= std_logic_vector(stream_start_snpidx_tig_0);
        reg_dout_intern(159 downto 128) <= std_logic_vector(last_init_snpidx_tig_0);

      when 3 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        --  47.. 0 case_wr chain 0
        -- 111..64 ctrl_wr chain 0
        reg_dout_intern(127 downto 0)   <= ctchain_debug_tig(0);
        reg_dout_intern(175 downto 128) <= std_logic_vector(tbuf_table_count_tig_0(0));
        reg_dout_intern(239 downto 192) <= std_logic_vector(tbuf_word_count_tig_0(0));

      when 4 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

      --  47.. 0 case_wr chain 1
      -- 111..64 ctrl_wr chain 1
      -- reg_dout_dbg(127 downto 0) <= ctchain_debug_tig(1); -- TODO remove
      -- reg_dout_dbg(175 downto 128) <= std_logic_vector(tbuf_table_count_tig(1)); -- TODO remove
      -- reg_dout_dbg(239 downto 192) <= std_logic_vector(tbuf_word_count_tig(1) ); -- TODO remove
        reg_dout_intern(255 downto 248) <= dbg_status_0;

      when 5 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(239 downto 192) <= std_logic_vector(snpreader_snp_done_cnt_0);
        reg_dout_intern(175 downto 128) <= std_logic_vector(snpreader_round_done_cnt_0);
        reg_dout_intern(63 downto 0)    <= std_logic_vector(stall_cnt_0);

      when 6 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(47 downto 0)    <= std_logic_vector(dram_rd_req_cnt_tig_0);
        reg_dout_intern(111 downto 64)  <= std_logic_vector(dram_rd_ans_cnt_tig_0);
        reg_dout_intern(207 downto 192) <= std_logic_vector(dram_conflict_cnt_tig_0);

      when 7 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(47 downto 32)   <= std_logic_vector(host_reset_cnt_1);
        reg_dout_intern(95 downto 48)   <= std_logic_vector(host_reset_cycle_cnt_1);
        reg_dout_intern(127 downto 96)  <= std_logic_vector(inbuffer_rd_cnt_1);
        reg_dout_intern(159 downto 128) <= std_logic_vector(ctable_io_bufsize_tablewords_tig_ramclk_1);
        reg_dout_intern(191 downto 160) <= std_logic_vector(ctable_io_bufsize_outwords_tig_ramclk_1);

      when 8 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(95 downto 64) <= std_logic_vector(num_cases_rounded_tig_1);
        reg_dout_intern(63 downto 32) <= std_logic_vector(num_samples_rounded_tig_1);
        reg_dout_intern(31 downto 0)  <= std_logic_vector(num_snps_local_tig_1);

        reg_dout_intern(253 downto 224) <= std_logic_vector(round_addr_offset_tig_1);
        reg_dout_intern(221 downto 192) <= std_logic_vector(stream_start_addr_tig_1);
        reg_dout_intern(191 downto 160) <= std_logic_vector(stream_start_snpidx_tig_1);
        reg_dout_intern(159 downto 128) <= std_logic_vector(last_init_snpidx_tig_1);

      when 9 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        --  47.. 0 case_wr chain 0
        -- 111..64 ctrl_wr chain 0
        reg_dout_intern(127 downto 0)   <= ctchain_debug_tig(0);
        reg_dout_intern(175 downto 128) <= std_logic_vector(tbuf_table_count_tig_1(0));
        reg_dout_intern(239 downto 192) <= std_logic_vector(tbuf_word_count_tig_1(0));

      when 10 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

      --  47.. 0 case_wr chain 1
      -- 111..64 ctrl_wr chain 1
      -- reg_dout_dbg(127 downto 0) <= ctchain_debug_tig(1); -- TODO remove
      -- reg_dout_dbg(175 downto 128) <= std_logic_vector(tbuf_table_count_tig(1)); -- TODO remove
      -- reg_dout_dbg(239 downto 192) <= std_logic_vector(tbuf_word_count_tig(1) ); -- TODO remove
      reg_dout_intern(255 downto 248) <= dbg_status_1;

      when 11 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(239 downto 192) <= std_logic_vector(snpreader_snp_done_cnt_1);
        reg_dout_intern(175 downto 128) <= std_logic_vector(snpreader_round_done_cnt_1);
        reg_dout_intern(63 downto 0)    <= std_logic_vector(stall_cnt_1);

      when 12 =>
        reg_we_intern   <= '1';
        reg_addr_intern <= std_logic_vector(to_unsigned(dbg_state + 1, 10)); -- debug register address
        dbg_state       := dbg_state + 1;

        reg_dout_intern(47 downto 0)    <= std_logic_vector(dram_rd_req_cnt_tig_1);
        reg_dout_intern(111 downto 64)  <= std_logic_vector(dram_rd_ans_cnt_tig_1);
        reg_dout_intern(207 downto 192) <= std_logic_vector(dram_conflict_cnt_tig_1);

      when others =>
        if timer = 0 then
          dbg_state := 0;
          timer     := (others => '1');
        else
          timer := timer - 1;
        end if;

    end case;

    dram_rd_req_cnt_tig_0   := dram_rd_req_cnt_0;
    dram_rd_ans_cnt_tig_0   := dram_rd_ans_cnt_0;
    dram_conflict_cnt_tig_0 := dram_conflict_cnt_0;
    dram_rd_req_cnt_tig_1   := dram_rd_req_cnt_1;
    dram_rd_ans_cnt_tig_1   := dram_rd_ans_cnt_1;
    dram_conflict_cnt_tig_1 := dram_conflict_cnt_1;

    -- ctchain_debug_tig := ctchain_debug;
    tbuf_table_count_tig_0 := tbuf_table_count_sig_0;
    tbuf_word_count_tig_0  := tbuf_word_count_sig_0;
    tbuf_table_count_tig_1 := tbuf_table_count_sig_1;
    tbuf_word_count_tig_1  := tbuf_word_count_sig_1;

    reg_we_intern_tig <= reg_we_intern;

  end process reg_dbg_p;

  --  END DEBUG

end Behavioral;
