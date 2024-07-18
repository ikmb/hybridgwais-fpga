library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

library UNISIM;
use UNISIM.VComponents.all;

use work.ad8k5_2way_ctables_pkg.all;
use work.ad8k5_2way_ctables_info.all;

entity ad8k5_2way_ctables_top is
  port (
    c0_ddr4_act_n : out STD_LOGIC;
    c0_ddr4_adr : out STD_LOGIC_VECTOR ( 16 downto 0 );
    c0_ddr4_ba : out STD_LOGIC_VECTOR ( 1 downto 0 );
    c0_ddr4_bg : out STD_LOGIC_VECTOR ( 1 downto 0 );
    c0_ddr4_ck_c : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_ck_t : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_cke : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_cs_n : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_dm_n : inout STD_LOGIC_VECTOR ( 8 downto 0 );
    c0_ddr4_dq : inout STD_LOGIC_VECTOR ( 71 downto 0 );
    c0_ddr4_dqs_c : inout STD_LOGIC_VECTOR ( 8 downto 0 );
    c0_ddr4_dqs_t : inout STD_LOGIC_VECTOR ( 8 downto 0 );
    c0_ddr4_odt : out STD_LOGIC_VECTOR ( 0 to 0 );
    c0_ddr4_reset_n : out STD_LOGIC;
    c0_sys_clk_n : in STD_LOGIC;
    c0_sys_clk_p : in STD_LOGIC;
    c1_ddr4_act_n : out STD_LOGIC;
    c1_ddr4_adr : out STD_LOGIC_VECTOR ( 16 downto 0 );
    c1_ddr4_ba : out STD_LOGIC_VECTOR ( 1 downto 0 );
    c1_ddr4_bg : out STD_LOGIC_VECTOR ( 1 downto 0 );
    c1_ddr4_ck_c : out STD_LOGIC_VECTOR ( 0 to 0 );
    c1_ddr4_ck_t : out STD_LOGIC_VECTOR ( 0 to 0 );
    c1_ddr4_cke : out STD_LOGIC_VECTOR ( 0 to 0 );
    c1_ddr4_cs_n : out STD_LOGIC_VECTOR ( 0 to 0 );
    c1_ddr4_dm_n : inout STD_LOGIC_VECTOR ( 8 downto 0 );
    c1_ddr4_dq : inout STD_LOGIC_VECTOR ( 71 downto 0 );
    c1_ddr4_dqs_c : inout STD_LOGIC_VECTOR ( 8 downto 0 );
    c1_ddr4_dqs_t : inout STD_LOGIC_VECTOR ( 8 downto 0 );
    c1_ddr4_odt : out STD_LOGIC_VECTOR ( 0 to 0 );
    c1_ddr4_reset_n : out STD_LOGIC;
    c1_sys_clk_n : in STD_LOGIC;
    c1_sys_clk_p : in STD_LOGIC;
    led0 : out STD_LOGIC_VECTOR ( 0 to 0 );
    led1 : out STD_LOGIC_VECTOR ( 0 to 0 );
    model_inout_tri_io : inout STD_LOGIC_VECTOR ( 45 downto 0 );
    pci_exp_rxn : in STD_LOGIC_VECTOR ( 7 downto 0 );
    pci_exp_rxp : in STD_LOGIC_VECTOR ( 7 downto 0 );
    pci_exp_txn : out STD_LOGIC_VECTOR ( 7 downto 0 );
    pci_exp_txp : out STD_LOGIC_VECTOR ( 7 downto 0 );
    pcie100_n : in STD_LOGIC;
    pcie100_p : in STD_LOGIC;
    perst_n : in STD_LOGIC;
    refclk200 : in STD_LOGIC
    );
end entity ad8k5_2way_ctables_top;

architecture Behavioral of ad8k5_2way_ctables_top is
  
  COMPONENT admpcie
  PORT (
    perst_n : IN STD_LOGIC;
    pcie100_p : IN STD_LOGIC;
    pcie100_n : IN STD_LOGIC;
    refclk200_in : IN STD_LOGIC;
    aclk : OUT STD_LOGIC;                               -- 250 MHz
    aresetn : OUT STD_LOGIC;
    pci_exp_txn : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    pci_exp_txp : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    pci_exp_rxn : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    pci_exp_rxp : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    model_inout_i : IN STD_LOGIC_VECTOR(45 DOWNTO 0);
    model_inout_o : OUT STD_LOGIC_VECTOR(45 DOWNTO 0);
    model_inout_t : OUT STD_LOGIC_VECTOR(45 DOWNTO 0);
    ds_axi_awaddr : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
    ds_axi_awlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    ds_axi_awsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    ds_axi_awburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    ds_axi_awcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    ds_axi_awprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    ds_axi_awvalid : OUT STD_LOGIC;
    ds_axi_wdata : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
    ds_axi_wstrb : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    ds_axi_wlast : OUT STD_LOGIC;
    ds_axi_wvalid : OUT STD_LOGIC;
    ds_axi_bready : OUT STD_LOGIC;
    ds_axi_araddr : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
    ds_axi_arlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    ds_axi_arsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    ds_axi_arburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    ds_axi_arcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    ds_axi_arprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
    ds_axi_arvalid : OUT STD_LOGIC;
    ds_axi_rready : OUT STD_LOGIC;
    ds_axi_awready : IN STD_LOGIC;
    ds_axi_wready : IN STD_LOGIC;
    ds_axi_bresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
    ds_axi_bvalid : IN STD_LOGIC;
    ds_axi_arready : IN STD_LOGIC;
    ds_axi_rdata : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    ds_axi_rresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
    ds_axi_rlast : IN STD_LOGIC;
    ds_axi_rvalid : IN STD_LOGIC;
    ds_axi_awlock : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    ds_axi_awqos : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    ds_axi_arlock : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    ds_axi_arqos : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    ds_axi_awregion : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    ds_axi_arregion : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    dma0_m_axis_tdata : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
    dma0_m_axis_tready : IN STD_LOGIC;
    dma0_m_axis_tvalid : OUT STD_LOGIC;
    dma1_s_axis_tdata : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    dma1_s_axis_tready : OUT STD_LOGIC;
    dma1_s_axis_tvalid : IN STD_LOGIC;
    dma2_m_axis_tdata : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
    dma2_m_axis_tready : IN STD_LOGIC;
    dma2_m_axis_tvalid : OUT STD_LOGIC;
    dma3_s_axis_tdata : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    dma3_s_axis_tready : OUT STD_LOGIC;
    dma3_s_axis_tvalid : IN STD_LOGIC;
    core_status : OUT STD_LOGIC_VECTOR(63 DOWNTO 0)
  );
  END COMPONENT;
  
  COMPONENT ddr4
  PORT (
    c0_init_calib_complete : OUT STD_LOGIC;
    dbg_clk : OUT STD_LOGIC;                                 -- must be open!!!
    c0_sys_clk_p : IN STD_LOGIC;
    c0_sys_clk_n : IN STD_LOGIC;
    dbg_bus : OUT STD_LOGIC_VECTOR(511 DOWNTO 0);            -- must be open!!!
    c0_ddr4_adr : OUT STD_LOGIC_VECTOR(16 DOWNTO 0);
    c0_ddr4_ba : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    c0_ddr4_cke : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    c0_ddr4_cs_n : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    c0_ddr4_dm_dbi_n : INOUT STD_LOGIC_VECTOR(8 DOWNTO 0);
    c0_ddr4_dq : INOUT STD_LOGIC_VECTOR(71 DOWNTO 0);
    c0_ddr4_dqs_c : INOUT STD_LOGIC_VECTOR(8 DOWNTO 0);
    c0_ddr4_dqs_t : INOUT STD_LOGIC_VECTOR(8 DOWNTO 0);
    c0_ddr4_odt : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    c0_ddr4_bg : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    c0_ddr4_reset_n : OUT STD_LOGIC;
    c0_ddr4_act_n : OUT STD_LOGIC;
    c0_ddr4_ck_c : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    c0_ddr4_ck_t : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
    c0_ddr4_ui_clk : OUT STD_LOGIC;                          -- 266 MHz
    c0_ddr4_ui_clk_sync_rst : OUT STD_LOGIC;
    c0_ddr4_app_en : IN STD_LOGIC;
    c0_ddr4_app_hi_pri : IN STD_LOGIC;
    c0_ddr4_app_wdf_end : IN STD_LOGIC;
    c0_ddr4_app_wdf_wren : IN STD_LOGIC;
    c0_ddr4_app_rd_data_end : OUT STD_LOGIC;
    c0_ddr4_app_rd_data_valid : OUT STD_LOGIC;
    c0_ddr4_app_rdy : OUT STD_LOGIC;
    c0_ddr4_app_wdf_rdy : OUT STD_LOGIC;
    c0_ddr4_app_addr : IN STD_LOGIC_VECTOR(29 DOWNTO 0);
    c0_ddr4_app_cmd : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
    c0_ddr4_app_wdf_data : IN STD_LOGIC_VECTOR(575 DOWNTO 0);
    --c0_ddr4_app_wdf_mask : IN STD_LOGIC_VECTOR(71 DOWNTO 0);
    c0_ddr4_app_rd_data : OUT STD_LOGIC_VECTOR(575 DOWNTO 0);
    sys_rst : IN STD_LOGIC
  );
  END COMPONENT;

  component IOBUF is
  port (
    I  : in    std_logic;
    O  : out   std_logic;
    T  : in    std_logic;
    IO : inout std_logic
  );
  end component IOBUF;
  
  component BUFG is
  port (
    O : out std_logic;
    I : in std_logic
  );
  end component BUFG;
  
  COMPONENT axi2bram_1kx256
  PORT (
    s_axi_aclk : IN STD_LOGIC;
    s_axi_aresetn : IN STD_LOGIC;
    s_axi_awaddr : IN STD_LOGIC_VECTOR(14 DOWNTO 0);
    s_axi_awlen : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    s_axi_awsize : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
    s_axi_awburst : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
    s_axi_awlock : IN STD_LOGIC;
    s_axi_awcache : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
    s_axi_awprot : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
    s_axi_awvalid : IN STD_LOGIC;
    s_axi_awready : OUT STD_LOGIC;
    s_axi_wdata : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    s_axi_wstrb : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    s_axi_wlast : IN STD_LOGIC;
    s_axi_wvalid : IN STD_LOGIC;
    s_axi_wready : OUT STD_LOGIC;
    s_axi_bresp : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    s_axi_bvalid : OUT STD_LOGIC;
    s_axi_bready : IN STD_LOGIC;
    s_axi_araddr : IN STD_LOGIC_VECTOR(14 DOWNTO 0);
    s_axi_arlen : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    s_axi_arsize : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
    s_axi_arburst : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
    s_axi_arlock : IN STD_LOGIC;
    s_axi_arcache : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
    s_axi_arprot : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
    s_axi_arvalid : IN STD_LOGIC;
    s_axi_arready : OUT STD_LOGIC;
    s_axi_rdata : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
    s_axi_rresp : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
    s_axi_rlast : OUT STD_LOGIC;
    s_axi_rvalid : OUT STD_LOGIC;
    s_axi_rready : IN STD_LOGIC;
    bram_rst_a : OUT STD_LOGIC;
    bram_clk_a : OUT STD_LOGIC;
    bram_en_a : OUT STD_LOGIC;
    bram_we_a : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    bram_addr_a : OUT STD_LOGIC_VECTOR(14 DOWNTO 0);
    bram_wrdata_a : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
    bram_rddata_a : IN STD_LOGIC_VECTOR(255 DOWNTO 0)
  );
  END COMPONENT;

  COMPONENT bram_tdp_1kx256
  PORT (
    clka : IN STD_LOGIC;
    rsta : IN STD_LOGIC;
    ena : IN STD_LOGIC;
    wea : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
    addra : IN STD_LOGIC_VECTOR(9 DOWNTO 0);
    dina : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    douta : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
    clkb : IN STD_LOGIC;
    web : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
    addrb : IN STD_LOGIC_VECTOR(9 DOWNTO 0);
    dinb : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    doutb : OUT STD_LOGIC_VECTOR(255 DOWNTO 0)
  );
  END COMPONENT;

  signal clk, resetn        : std_logic;
  signal dma0_m_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma0_m_axis_tvalid : std_logic;
  signal dma0_m_axis_tready : std_logic;
  signal dma1_s_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma1_s_axis_tvalid : std_logic;
  signal dma1_s_axis_tready : std_logic;
  signal dma2_m_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma2_m_axis_tvalid : std_logic;
  signal dma2_m_axis_tready : std_logic;
  signal dma3_s_axis_tdata  : std_logic_vector(255 downto 0);
  signal dma3_s_axis_tvalid : std_logic;
  signal dma3_s_axis_tready : std_logic;

  signal s_axi_awaddr      : std_logic_vector(63 downto 0);
  signal s_axi_awlen       : std_logic_vector(7 downto 0);
  signal s_axi_awsize      : std_logic_vector(2 downto 0);
  signal s_axi_awburst     : std_logic_vector(1 downto 0);
  signal s_axi_awcache     : std_logic_vector(3 downto 0);
  signal s_axi_awprot      : std_logic_vector(2 downto 0);
  signal s_axi_awvalid     : std_logic;
  signal s_axi_wdata       : std_logic_vector(255 downto 0);
  signal s_axi_wstrb       : std_logic_vector(31 downto 0);
  signal s_axi_wlast       : std_logic;
  signal s_axi_wvalid      : std_logic;
  signal s_axi_bready      : std_logic;
  signal s_axi_araddr      : std_logic_vector(63 downto 0);
  signal s_axi_arlen       : std_logic_vector(7 downto 0);
  signal s_axi_arsize      : std_logic_vector(2 downto 0);
  signal s_axi_arburst     : std_logic_vector(1 downto 0);
  signal s_axi_arcache     : std_logic_vector(3 downto 0);
  signal s_axi_arprot      : std_logic_vector(2 downto 0);
  signal s_axi_arvalid     : std_logic;
  signal s_axi_rready      : std_logic;
  signal s_axi_awready     : std_logic;
  signal s_axi_wready      : std_logic;
  signal s_axi_bresp       : std_logic_vector(1 downto 0);
  signal s_axi_bvalid      : std_logic;
  signal s_axi_arready     : std_logic;
  signal s_axi_rdata       : std_logic_vector(255 downto 0);
  signal s_axi_rresp       : std_logic_vector(1 downto 0);
  signal s_axi_rlast       : std_logic;
  signal s_axi_rvalid      : std_logic;
  signal s_axi_awlock      : std_logic_vector(0 downto 0);
  signal s_axi_arlock      : std_logic_vector(0 downto 0);
  
  signal bram_rst_a     : std_logic;
  signal bram_clk_a     : std_logic;
  signal bram_en_a     : std_logic;
  signal bram_we_a     : std_logic_vector(31 downto 0) := (others => '0'); -- comment to disable writing from host
  signal bram_addr_a   : std_logic_vector(9 downto 0) := (others => '0');
  signal bram_din_a  : std_logic_vector(255 downto 0);  -- comment to disable writing from host
  signal bram_dout_a : std_logic_vector(255 downto 0);
  signal bram_rddata_a : std_logic_vector(255 downto 0);
  signal info_replace : std_logic := '0';
  
  signal bram_clk_b  : std_logic;
  signal bram_we_b   : std_logic;
  signal bram_addr_b : std_logic_vector(9 downto 0) := (others => '0');
  signal bram_din_b  : std_logic_vector(255 downto 0);
  signal bram_dout_b : std_logic_vector(255 downto 0);  -- comment to disable reading from FPGA

  -- amount of items in one data block (multiple of 2!!!)
--  constant REG_DATA_BLOCK_SIZE : integer := 8;
--  signal reg_wr_offset : unsigned(9 downto 0) := "0000000000";
--  signal reg_wr_offset_tig : unsigned(9 downto 0) := "0000000000";
--  signal reg_wr_offset_ready_tig : std_logic := '0'; 
--  signal reg_rd_offset : unsigned(9 downto 0) := "0000000000";
  signal main_reg_addr : std_logic_vector(9 downto 0) := "0000000000";
  signal dma_reg_addr : std_logic_vector(14 downto 0) := "000000000000000";
  
  signal main_status : std_logic_vector(7 downto 0);
  signal status      : std_logic_vector(7 downto 0) := (others => '1');

  signal model_inout_i, model_inout_o, model_inout_t : std_logic_vector(model_inout_tri_io'high downto model_inout_tri_io'low);

  signal c0_app_addr          : std_logic_vector(29 downto 0);
  signal c0_app_cmd           : std_logic_vector(2 downto 0);
  signal c0_app_en            : std_logic;
  signal c0_app_wdf_data      : std_logic_vector(575 downto 0);
  --signal c0_app_wdf_mask      : std_logic_vector(71 downto 0);
  signal c0_app_wdf_end       : std_logic;
  signal c0_app_wdf_wren      : std_logic;
  signal c0_app_rd_data       : std_logic_vector(575 downto 0);
  signal c0_app_rd_data_end   : std_logic;
  signal c0_app_rd_data_valid : std_logic;
  signal c0_app_rdy           : std_logic;
  signal c0_app_wdf_rdy       : std_logic;

  signal c0_app_sr_req          : std_logic;
  signal c0_app_ref_req         : std_logic;
  signal c0_app_zq_req          : std_logic;
  signal c0_ui_clk              : std_logic;
  signal c0_ui_clk_sync_rst     : std_logic;
  signal c0_init_calib_complete : std_logic;

  signal c1_app_addr          : std_logic_vector(29 downto 0);
  signal c1_app_cmd           : std_logic_vector(2 downto 0);
  signal c1_app_en            : std_logic;
  signal c1_app_wdf_data      : std_logic_vector(575 downto 0);
  --signal c1_app_wdf_mask      : std_logic_vector(71 downto 0);
  signal c1_app_wdf_end       : std_logic;
  signal c1_app_wdf_wren      : std_logic;
  signal c1_app_rd_data       : std_logic_vector(575 downto 0);
  signal c1_app_rd_data_end   : std_logic;
  signal c1_app_rd_data_valid : std_logic;
  signal c1_app_rdy           : std_logic;
  signal c1_app_wdf_rdy       : std_logic;

  signal c1_app_sr_req          : std_logic;
  signal c1_app_ref_req         : std_logic;
  signal c1_app_zq_req          : std_logic;
  signal c1_ui_clk              : std_logic;
  signal c1_ui_clk_sync_rst     : std_logic;
  signal c1_init_calib_complete : std_logic;

begin

  led0 <= "1";
  led1 <= "1";
  
  -- tristate buffers for AD model I/O channel
  io_tristate_gen : for I in model_inout_tri_io'high downto model_inout_tri_io'low generate
    io_tristate : IOBUF
      port map (
        I  => model_inout_o(I),
        O  => model_inout_i(I),
        T  => model_inout_t(I),
        IO => model_inout_tri_io(I));
  end generate io_tristate_gen;

--  refclk_buf_i : BUFG port map (
--    O => refclk,
--    I => refclk_unbuffered);

  -- pci exress core
  admpcie_i : admpcie
    port map (
      perst_n => perst_n,
    pcie100_p => pcie100_p,
    pcie100_n => pcie100_n,
    refclk200_in => refclk200,
    aclk => clk,
    aresetn => resetn,
    pci_exp_txn => pci_exp_txn,
    pci_exp_txp => pci_exp_txp,
    pci_exp_rxn => pci_exp_rxn,
    pci_exp_rxp => pci_exp_rxp,
    model_inout_i => model_inout_i,
    model_inout_o => model_inout_o,
    model_inout_t => model_inout_t,
    ds_axi_awaddr => s_axi_awaddr,
    ds_axi_awlen => s_axi_awlen,
    ds_axi_awsize => s_axi_awsize,
    ds_axi_awburst => s_axi_awburst,
    ds_axi_awcache => s_axi_awcache,
    ds_axi_awprot => s_axi_awprot,
    ds_axi_awvalid => s_axi_awvalid,
    ds_axi_wdata => s_axi_wdata,
    ds_axi_wstrb => s_axi_wstrb,
    ds_axi_wlast => s_axi_wlast,
    ds_axi_wvalid => s_axi_wvalid,
    ds_axi_bready => s_axi_bready,
    ds_axi_araddr => s_axi_araddr,
    ds_axi_arlen => s_axi_arlen,
    ds_axi_arsize => s_axi_arsize,
    ds_axi_arburst => s_axi_arburst,
    ds_axi_arcache => s_axi_arcache,
    ds_axi_arprot => s_axi_arprot,
    ds_axi_arvalid => s_axi_arvalid,
    ds_axi_rready => s_axi_rready,
    ds_axi_awready => s_axi_awready,
    ds_axi_wready => s_axi_wready,
    ds_axi_bresp => s_axi_bresp,
    ds_axi_bvalid => s_axi_bvalid,
    ds_axi_arready => s_axi_arready,
    ds_axi_rdata => s_axi_rdata,
    ds_axi_rresp => s_axi_rresp,
    ds_axi_rlast => s_axi_rlast,
    ds_axi_rvalid => s_axi_rvalid,
    ds_axi_awlock => s_axi_awlock,
    ds_axi_awqos => open, --s_axi_awqos,
    ds_axi_arlock => s_axi_arlock,
    ds_axi_arqos => open, --s_axi_arqos,
    ds_axi_awregion => open, --s_axi_awregion,
    ds_axi_arregion => open, --s_axi_arregion,
    dma0_m_axis_tdata  => dma0_m_axis_tdata,
    dma0_m_axis_tready => dma0_m_axis_tready,
    dma0_m_axis_tvalid => dma0_m_axis_tvalid,
    dma1_s_axis_tdata  => dma1_s_axis_tdata,
    dma1_s_axis_tready => dma1_s_axis_tready,
    dma1_s_axis_tvalid => dma1_s_axis_tvalid,
    dma2_m_axis_tdata  => dma2_m_axis_tdata,
    dma2_m_axis_tready => dma2_m_axis_tready,
    dma2_m_axis_tvalid => dma2_m_axis_tvalid,
    dma3_s_axis_tdata  => dma3_s_axis_tdata,
    dma3_s_axis_tready => dma3_s_axis_tready,
    dma3_s_axis_tvalid => dma3_s_axis_tvalid,
    core_status => core_status
      );

-- DRAM core
  ddr4_0 : ddr4
  PORT MAP (
    c0_init_calib_complete => c0_init_calib_complete,
    dbg_clk => open, --dbg_clk,
    c0_sys_clk_p => c0_sys_clk_p,
    c0_sys_clk_n => c0_sys_clk_n,
    dbg_bus => open, --dbg_bus,
    c0_ddr4_adr => c0_ddr4_adr,
    c0_ddr4_ba => c0_ddr4_ba,
    c0_ddr4_cke => c0_ddr4_cke,
    c0_ddr4_cs_n => c0_ddr4_cs_n,
    c0_ddr4_dm_dbi_n => c0_ddr4_dm_n,
    c0_ddr4_dq => c0_ddr4_dq,
    c0_ddr4_dqs_c => c0_ddr4_dqs_c,
    c0_ddr4_dqs_t => c0_ddr4_dqs_t,
    c0_ddr4_odt => c0_ddr4_odt,
    c0_ddr4_bg => c0_ddr4_bg,
    c0_ddr4_reset_n => c0_ddr4_reset_n,
    c0_ddr4_act_n => c0_ddr4_act_n,
    c0_ddr4_ck_c => c0_ddr4_ck_c,
    c0_ddr4_ck_t => c0_ddr4_ck_t,
    c0_ddr4_ui_clk => c0_ui_clk,
    c0_ddr4_ui_clk_sync_rst => c0_ui_clk_sync_rst,
    c0_ddr4_app_en => c0_app_en,
    c0_ddr4_app_hi_pri => '0', --c0_app_hi_pri,
    c0_ddr4_app_wdf_end => c0_app_wdf_end,
    c0_ddr4_app_wdf_wren => c0_app_wdf_wren,
    c0_ddr4_app_rd_data_end => c0_app_rd_data_end,
    c0_ddr4_app_rd_data_valid => c0_app_rd_data_valid,
    c0_ddr4_app_rdy => c0_app_rdy,
    c0_ddr4_app_wdf_rdy => c0_app_wdf_rdy,
    c0_ddr4_app_addr => c0_app_addr,
    c0_ddr4_app_cmd => c0_app_cmd,
    c0_ddr4_app_wdf_data => c0_app_wdf_data,
    --c0_ddr4_app_wdf_mask => c0_app_wdf_mask,
    c0_ddr4_app_rd_data => c0_app_rd_data,
    sys_rst => not resetn
  );
  
  ddr4_1 : ddr4
  PORT MAP (
    c0_init_calib_complete => c1_init_calib_complete,
    dbg_clk => open,
    c0_sys_clk_p => c1_sys_clk_p,
    c0_sys_clk_n => c1_sys_clk_n,
    dbg_bus => open,
    c0_ddr4_adr => c1_ddr4_adr,
    c0_ddr4_ba => c1_ddr4_ba,
    c0_ddr4_cke => c1_ddr4_cke,
    c0_ddr4_cs_n => c1_ddr4_cs_n,
    c0_ddr4_dm_dbi_n => c1_ddr4_dm_n,
    c0_ddr4_dq => c1_ddr4_dq,
    c0_ddr4_dqs_c => c1_ddr4_dqs_c,
    c0_ddr4_dqs_t => c1_ddr4_dqs_t,
    c0_ddr4_odt => c1_ddr4_odt,
    c0_ddr4_bg => c1_ddr4_bg,
    c0_ddr4_reset_n => c1_ddr4_reset_n,
    c0_ddr4_act_n => c1_ddr4_act_n,
    c0_ddr4_ck_c => c1_ddr4_ck_c,
    c0_ddr4_ck_t => c1_ddr4_ck_t,
    c0_ddr4_ui_clk => c1_ui_clk,
    c0_ddr4_ui_clk_sync_rst => c1_ui_clk_sync_rst,
    c0_ddr4_app_en => c1_app_en,
    c0_ddr4_app_hi_pri => '0', --c1_app_hi_pri,
    c0_ddr4_app_wdf_end => c1_app_wdf_end,
    c0_ddr4_app_wdf_wren => c1_app_wdf_wren,
    c0_ddr4_app_rd_data_end => c1_app_rd_data_end,
    c0_ddr4_app_rd_data_valid => c1_app_rd_data_valid,
    c0_ddr4_app_rdy => c1_app_rdy,
    c0_ddr4_app_wdf_rdy => c1_app_wdf_rdy,
    c0_ddr4_app_addr => c1_app_addr,
    c0_ddr4_app_cmd => c1_app_cmd,
    c0_ddr4_app_wdf_data => c1_app_wdf_data,
    --c0_ddr4_app_wdf_mask => c1_app_wdf_mask,
    c0_ddr4_app_rd_data => c1_app_rd_data,
    sys_rst => not resetn
  );

-- do not request anything
  c0_app_sr_req  <= '0';
  c0_app_ref_req <= '0';
  c0_app_zq_req  <= '0';
  c1_app_sr_req  <= '0';
  c1_app_ref_req <= '0';
  c1_app_zq_req  <= '0';

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
      dma_din2_tdata       => dma2_m_axis_tdata,
      dma_din2_tready      => dma2_m_axis_tready,
      dma_din2_tvalid      => dma2_m_axis_tvalid,
      dma_dout3_tdata      => dma3_s_axis_tdata,
      dma_dout3_tready     => dma3_s_axis_tready,
      dma_dout3_tvalid     => dma3_s_axis_tvalid,
      -- PCIe Register Interface
      reg_clk              => bram_clk_b,
      reg_addr             => main_reg_addr,
      reg_we               => bram_we_b,
      reg_din              => bram_dout_b, -- (others => '-') to disable reading from FPGA
      reg_dout             => bram_din_b,
      -- Status
      status               => main_status,
      -- DRAM
      c0_app_addr          => c0_app_addr,
      c0_app_cmd           => c0_app_cmd,
      c0_app_en            => c0_app_en,
      c0_app_wdf_data      => c0_app_wdf_data,
      --c0_app_wdf_mask      => c0_app_wdf_mask,
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
      --c1_app_wdf_mask      => c1_app_wdf_mask,
      c1_app_wdf_end       => c1_app_wdf_end,
      c1_app_wdf_wren      => c1_app_wdf_wren,
      c1_app_rd_data       => c1_app_rd_data,
      c1_app_rd_data_end   => c1_app_rd_data_end,
      c1_app_rd_data_valid => c1_app_rd_data_valid,
      c1_app_rdy           => c1_app_rdy,
      c1_app_wdf_rdy       => c1_app_wdf_rdy
      );

  ----- register interface -----
  
  -- AXI 2 BRAM bridge
  axi2bram_i : axi2bram_1kx256
  PORT MAP (
    s_axi_aclk => clk,
    s_axi_aresetn => resetn,
    s_axi_awaddr => s_axi_awaddr(14 downto 0),
    s_axi_awlen => s_axi_awlen,
    s_axi_awsize => s_axi_awsize,
    s_axi_awburst => s_axi_awburst,
    s_axi_awlock => s_axi_awlock(0),
    s_axi_awcache => s_axi_awcache,
    s_axi_awprot => s_axi_awprot,
    s_axi_awvalid => s_axi_awvalid,
    s_axi_awready => s_axi_awready,
    s_axi_wdata => s_axi_wdata,
    s_axi_wstrb => s_axi_wstrb,
    s_axi_wlast => s_axi_wlast,
    s_axi_wvalid => s_axi_wvalid,
    s_axi_wready => s_axi_wready,
    s_axi_bresp => s_axi_bresp,
    s_axi_bvalid => s_axi_bvalid,
    s_axi_bready => s_axi_bready,
    s_axi_araddr => s_axi_araddr(14 downto 0),
    s_axi_arlen => s_axi_arlen,
    s_axi_arsize => s_axi_arsize,
    s_axi_arburst => s_axi_arburst,
    s_axi_arlock => s_axi_arlock(0),
    s_axi_arcache => s_axi_arcache,
    s_axi_arprot => s_axi_arprot,
    s_axi_arvalid => s_axi_arvalid,
    s_axi_arready => s_axi_arready,
    s_axi_rdata => s_axi_rdata,
    s_axi_rresp => s_axi_rresp,
    s_axi_rlast => s_axi_rlast,
    s_axi_rvalid => s_axi_rvalid,
    s_axi_rready => s_axi_rready,
    bram_rst_a => bram_rst_a,
    bram_clk_a => bram_clk_a,
    bram_en_a => bram_en_a,
    bram_we_a => bram_we_a, -- "open" to disable writing from host
    bram_addr_a => dma_reg_addr,
    bram_wrdata_a => bram_din_a, -- "open" to disable writing from host
    bram_rddata_a => bram_rddata_a
  );

dbg_reg_i : bram_tdp_1kx256
  PORT MAP (
    clka => bram_clk_a,
    rsta => bram_rst_a,
    ena => bram_en_a,
    wea(0) => bram_we_a(0), -- '0' to disable writing from host
    addra => bram_addr_a,
    dina => bram_din_a, -- (others => '-') to disable writing from host
    douta => bram_dout_a,
    clkb => bram_clk_b,
    web(0) => bram_we_b,
    addrb => bram_addr_b,
    dinb => bram_din_b,
    doutb => bram_dout_b -- "open" to disable reading from FPGA  
  );
 
bram_addr_a <= dma_reg_addr(14 downto 5);
bram_addr_b <= main_reg_addr;

-- FPGA core version information on BRAM address 0
version_info_p: process
begin
  wait until rising_edge(bram_clk_a);
  
  info_replace <= '0';
  if bram_addr_a = (bram_addr_a'range => '0') then
    info_replace <= '1';
  end if;
  
  status <= main_status; -- false path
end process version_info_p;
bram_rddata_a <= bram_dout_a when info_replace = '0' else (status & INFO_DATA); -- note that writing from host will still work

---- enable history by an offset           --, but exclude address 0
---- BRAM has stack behaviour!!
----bram_addr_a <= std_logic_vector(reg_rd_offset - unsigned(dma_reg_addr(14 downto 5))) when dma_reg_addr(14 downto 5) /= "0000000000" else "0000000000";
----bram_addr_b <= std_logic_vector(unsigned(main_reg_addr) + reg_wr_offset) when main_reg_addr /= "0000000000" else "0000000000";
--bram_addr_a <= std_logic_vector(reg_rd_offset - unsigned(dma_reg_addr(14 downto 5)));
--bram_addr_b <= std_logic_vector(unsigned(main_reg_addr) + reg_wr_offset);
--
--reg_history_w_p: process
--  variable item_cnt : integer range 0 to REG_DATA_BLOCK_SIZE-1 := 0;
--begin
--  wait until rising_edge(bram_clk_b);
--
--  reg_wr_offset_ready_tig <= '0';
--  if item_cnt = 0 then
--    reg_wr_offset_ready_tig <= '1';
--  end if;
--  
--  reg_wr_offset_tig <= reg_wr_offset;
--  
--  if bram_we_b = '1' and main_reg_addr /= "0000000000" then
--    if item_cnt = REG_DATA_BLOCK_SIZE-1 then
--      item_cnt := 0;
----      -- wrap around to save register address 0
----      if reg_wr_offset = (to_unsigned(1024 - 2*REG_DATA_BLOCK_SIZE, 10)) then
----        reg_wr_offset <= (others => '0');
----      else
--        reg_wr_offset <= reg_wr_offset + REG_DATA_BLOCK_SIZE;
----      end if;
--    else
--      item_cnt := item_cnt + 1;
--    end if;
--    
--  end if;
--
--end process reg_history_w_p;
--
--reg_history_r_p: process
--begin
--  wait until rising_edge(bram_clk_a);
--  
--  if reg_wr_offset_ready_tig = '1' then
--    reg_rd_offset <= reg_wr_offset_tig;
--  end if;  
--
--end process reg_history_r_p;
--
end Behavioral;
