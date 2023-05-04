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

entity ad8k5_2way_ctables_tb is
end entity ad8k5_2way_ctables_tb;

architecture Behavioral of ad8k5_2way_ctables_tb is
  
  constant PCI_CLK_PERIOD : time := 4 ns;
  constant RAM_CLK_PERIOD : time := 5 ns;

  signal clk, resetn        : std_logic;
  signal dma0_m_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma0_m_axis_tvalid : std_logic;
  signal dma0_m_axis_tready : std_logic;
  signal dma1_s_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma1_s_axis_tvalid : std_logic;
  signal dma1_s_axis_tready : std_logic;
  signal dma2_s_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma2_s_axis_tvalid : std_logic;
  signal dma2_s_axis_tready : std_logic;
  
  signal bram_clk_b  : std_logic;
  signal bram_we_b   : std_logic;
  signal bram_din_b  : std_logic_vector(255 downto 0);
  signal bram_dout_b : std_logic_vector(255 downto 0);

  signal main_reg_addr : std_logic_vector(9 downto 0) := "0000000000";
  
  signal c0_app_addr          : std_logic_vector(29 downto 0);
  signal c0_app_cmd           : std_logic_vector(2 downto 0);
  signal c0_app_en            : std_logic;
  signal c0_app_wdf_data      : std_logic_vector(575 downto 0);
  signal c0_app_wdf_end       : std_logic;
  signal c0_app_wdf_wren      : std_logic;
  signal c0_app_rd_data       : std_logic_vector(575 downto 0);
  signal c0_app_rd_data_end   : std_logic;
  signal c0_app_rd_data_valid : std_logic;
  signal c0_app_rdy           : std_logic;
  signal c0_app_wdf_rdy       : std_logic;

  signal c0_ui_clk              : std_logic;
  signal c0_ui_clk_sync_rst     : std_logic;
  signal c0_calib_done        : std_logic;    
  
  signal c1_app_addr          : std_logic_vector(29 downto 0);
  signal c1_app_cmd           : std_logic_vector(2 downto 0);
  signal c1_app_en            : std_logic;
  signal c1_app_wdf_data      : std_logic_vector(575 downto 0);
  signal c1_app_wdf_end       : std_logic;
  signal c1_app_wdf_wren      : std_logic;
  signal c1_app_rd_data       : std_logic_vector(575 downto 0);
  signal c1_app_rd_data_end   : std_logic;
  signal c1_app_rd_data_valid : std_logic;
  signal c1_app_rdy           : std_logic;
  signal c1_app_wdf_rdy       : std_logic;

  signal c1_ui_clk              : std_logic;
  signal c1_ui_clk_sync_rst     : std_logic;
  
begin

  -- clk processes
  clk_p: process
  begin
    clk <= '0';
    wait for PCI_CLK_PERIOD / 2;
    clk <= '1';
    wait for PCI_CLK_PERIOD / 2;
  end process clk_p;
  
  resetn <= '0', '1' after 10 * PCI_CLK_PERIOD;
  
  -- main instance
  main_i : entity work.ad8k5_2way_ctables_main
    port map (
      -- Clock / Reset
      reset                => not resetn,
      pci_clk              => clk,
      ram_clk0             => c0_ui_clk,
      ram_clk0_reset       => c0_ui_clk_sync_rst,
      ram_clk1             => c1_ui_clk,
      ram_clk1_reset       => c1_ui_clk_sync_rst,
      -- PCIe DMA
      dma_din0_tdata       => dma0_m_axis_tdata,
      dma_din0_tready      => dma0_m_axis_tready,
      dma_din0_tvalid      => dma0_m_axis_tvalid,
      dma_dout1_tdata      => dma1_s_axis_tdata,
      dma_dout1_tready     => dma1_s_axis_tready,
      dma_dout1_tvalid     => dma1_s_axis_tvalid,
      dma_dout2_tdata      => dma2_s_axis_tdata,
      dma_dout2_tready     => dma2_s_axis_tready,
      dma_dout2_tvalid     => dma2_s_axis_tvalid,
      -- PCIe Register Interface
      reg_clk              => bram_clk_b,
      reg_addr             => main_reg_addr,
      reg_we               => bram_we_b,
      reg_din              => bram_dout_b,
      reg_dout             => bram_din_b,
      -- DRAM
      c0_app_addr          => c0_app_addr,
      c0_app_cmd           => c0_app_cmd,
      c0_app_en            => c0_app_en,
      c0_app_wdf_data      => c0_app_wdf_data,
      c0_app_wdf_end       => c0_app_wdf_end,
      c0_app_wdf_wren      => c0_app_wdf_wren,
      c0_app_rd_data       => c0_app_rd_data,
      c0_app_rd_data_end   => c0_app_rd_data_end,
      c0_app_rd_data_valid => c0_app_rd_data_valid,
      c0_app_rdy           => c0_app_rdy,
      c0_app_wdf_rdy       => c0_app_wdf_rdy,
      c1_app_addr          => c1_app_addr,
      c1_app_cmd           => c1_app_cmd,
      c1_app_en            => c1_app_en,
      c1_app_wdf_data      => c1_app_wdf_data,
      c1_app_wdf_end       => c1_app_wdf_end,
      c1_app_wdf_wren      => c1_app_wdf_wren,
      c1_app_rd_data       => c1_app_rd_data,
      c1_app_rd_data_end   => c1_app_rd_data_end,
      c1_app_rd_data_valid => c1_app_rd_data_valid,
      c1_app_rdy           => c1_app_rdy,
      c1_app_wdf_rdy       => c1_app_wdf_rdy
      );
 
 -- RAM simulation
 ram_sim_i: entity work.ad_RAM_sim
   generic map(
     RAM_SIZE => 4096,
     RAM_CLK_PERIOD => RAM_CLK_PERIOD
   )
   port map(
     dram_clk           => c0_ui_clk,
     dram_calib_done    => c0_calib_done,
     dram_reset         => c0_ui_clk_sync_rst,
     dram_addr          => c0_app_addr,
     dram_cmd           => c0_app_cmd,
     dram_en            => c0_app_en,
     dram_rdy           => c0_app_rdy,
     dram_wdf_data      => c0_app_wdf_data,
     dram_wdf_end       => c0_app_wdf_end,
     dram_wdf_rdy       => c0_app_wdf_rdy,
     dram_wdf_wren      => c0_app_wdf_wren,
     dram_rd_data       => c0_app_rd_data,
     dram_rd_data_end   => c0_app_rd_data_end,
     dram_rd_data_valid => c0_app_rd_data_valid
   );
  c1_ui_clk <= c0_ui_clk;
  c1_ui_clk_sync_rst <= c0_ui_clk_sync_rst;
  
 -- constants:

 -- DMA out always ready
 dma1_s_axis_tready <= '1';
 dma2_s_axis_tready <= '1';
 
 bram_dout_b <= (others => '0'); -- unused anyway
 
 -- Testbench process
 tb_p: process
 begin
   dma0_m_axis_tvalid <= '0';
   dma0_m_axis_tdata <= (others => '0');
   wait for 100 ns;
   wait until rising_edge(clk);
   
   -- provide all required data, ignore ready signal
   dma0_m_axis_tvalid <= '1';
   dma0_m_axis_tdata <= (others => '1'); -- garbage
   wait until rising_edge(clk);
   
   wait until rising_edge(clk);   
   dma0_m_axis_tdata <= x"deadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeef"; -- init word
   
   wait until rising_edge(clk);
   -- 6 SNPs, 16 cases, 240 controls
   dma0_m_axis_tdata <= x"0000000000000000000000000000000600000000000000100000000000000100";
   
   wait until rising_edge(clk);
   -- 1GB buffer size, 12 initialization data words
   dma0_m_axis_tdata <= x"00000000000000000000000000000000003c3c3c00800000000000000000000b";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"aaaaaaaaaaffffffffffffffffffffffffffff000005556af554000000000000";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"aaaaaaaaaaafffffffffffffffffffffffffffffffffffffffffffffffffffff";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"5555555555ffffffffffffffffffffffffffff00000aaaaaf000000000000000";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"5555555555555fffffffffffffffffffffffffffffffffffffffffffffffffff";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"f5555555555fffffffffffffffffffffffffff0005556aaaf555400000000000";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"aaaaaaaaaaaaaaffffffffffffffffffffffffffffffffffffffffffffffffff";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"6666666666ffffffffffffffffffffffffffff000005556af554000000000000";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"222222222222ffffffffffffffffffffffffffffffffffffffffffffffffffff";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"2222222222222222ffffffffffffffffffffff00000aaaaaf000000002323232";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"666666666666ffffffffffffffffffffffffffffffffffffffffffffffffffff";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"aaaaaaaaaaaaaaffffffffffffffffffffffff0005556aaaf555400044444444";
   
   wait until rising_edge(clk);
   dma0_m_axis_tdata <= x"9999999999999fffffffffffffffffffffffffffffffffffffffffffffffffff";

--   wait until rising_edge(clk);
--   -- 18 SNPs, 160 cases, 160 controls
--   dma0_m_axis_tdata <= x"0000000000000000000000000000001200000000000000A00000000000000140";
--   
--   wait until rising_edge(clk);
--   -- 1GB buffer size, 72 initialization data words
--   dma0_m_axis_tdata <= x"00000000000000000000000000000000003c3c3c008000000000000000000047";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--    wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff000005556af554000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff00000aaaaaf000000000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffff0005556aaaf555400000000000";
--   
--   wait until rising_edge(clk);
--   dma0_m_axis_tdata <= x"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
   
   wait until rising_edge(clk);
   
   dma0_m_axis_tvalid <= '0';
   
   wait;
   
 end process tb_p;
   
end Behavioral;
