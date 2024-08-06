

# RESET SIGNALS
## engine 0
set_false_path -to [get_cells main_i/engine_0/pci_clk_sync_reset_reg]
set_false_path -to [get_cells main_i/engine_0/dram_clk_sync_reset_reg]
set_false_path -to [get_cells main_i/engine_0/host_reset_ramclk_reg]
set_false_path -to [get_cells main_i/engine_0/host_reset_pciclk_reg]

## engine 1
set_false_path -to [get_cells main_i/engine_1/pci_clk_sync_reset_reg]
set_false_path -to [get_cells main_i/engine_1/dram_clk_sync_reset_reg]
set_false_path -to [get_cells main_i/engine_1/host_reset_ramclk_reg]
set_false_path -to [get_cells main_i/engine_1/host_reset_pciclk_reg]

# CONFIG SIGNALS
## engine 0
set_false_path -from [get_cells {main_i/engine_0/num_samples_rounded_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/num_cases_rounded_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/num_snps_local_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/last_init_snpidx_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/stream_start_snpidx_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/stream_start_addr_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/round_addr_offset_tig_reg[*]}]

## engine 1
set_false_path -from [get_cells {main_i/engine_1/num_samples_rounded_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/num_cases_rounded_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/num_snps_local_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/last_init_snpidx_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/stream_start_snpidx_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/stream_start_addr_tig_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/round_addr_offset_tig_reg[*]}]


# IO SIGNALS
## engine 0
set_false_path -to [get_cells {main_i/engine_0/ctable_io_bufsize_outwords_tig_ramclk_reg[*]}]
set_false_path -to [get_cells {main_i/engine_0/ctable_io_bufsize_tablewords_tig_ramclk_reg[*]}]
set_false_path -to [get_cells {main_i/engine_0/ctable_io_bufsize_outwords_tig_pciclk_reg[*]}]
set_false_path -to [get_cells {main_i/engine_0/ctable_io_bufsize_tablewords_tig_pciclk_reg[*]}]
set_false_path -from [get_cells {main_i/engine_0/io_p.last_raw_gt_word_tig_reg[*]}]

## engine 1
set_false_path -to [get_cells {main_i/engine_1/ctable_io_bufsize_outwords_tig_ramclk_reg[*]}]
set_false_path -to [get_cells {main_i/engine_1/ctable_io_bufsize_tablewords_tig_ramclk_reg[*]}]
set_false_path -to [get_cells {main_i/engine_1/ctable_io_bufsize_outwords_tig_pciclk_reg[*]}]
set_false_path -to [get_cells {main_i/engine_1/ctable_io_bufsize_tablewords_tig_pciclk_reg[*]}]
set_false_path -from [get_cells {main_i/engine_1/io_p.last_raw_gt_word_tig_reg[*]}]

# DEBUG SIGNALS
## engine 0
set_false_path -to [get_cells main_i/engine_0/stall_reg]
set_false_path -to [get_cells main_i/engine_0/snpreader_i/busy_tig_reg]
set_false_path -to [get_cells main_i/engine_0/out_map_p.snpreader_busy_tig_reg]
set_false_path -to [get_cells main_i/engine_0/pf_sync_p.process_finished_tig_reg]

set_false_path -to [get_cells {main_i/reg_dbg_p.dram_rd_req_cnt_tig_0_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.dram_rd_ans_cnt_tig_0_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.dram_conflict_cnt_tig_0_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.tbuf_table_count_tig_0_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.tbuf_word_count_tig_0_reg[*]}]

## engine 1
set_false_path -to [get_cells main_i/engine_1/stall_reg]
set_false_path -to [get_cells main_i/engine_1/snpreader_i/busy_tig_reg]
set_false_path -to [get_cells main_i/engine_1/out_map_p.snpreader_busy_tig_reg]
set_false_path -to [get_cells main_i/engine_1/pf_sync_p.process_finished_tig_reg]

set_false_path -to [get_cells {main_i/reg_dbg_p.dram_rd_req_cnt_tig_1_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.dram_rd_ans_cnt_tig_1_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.dram_conflict_cnt_tig_1_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.tbuf_table_count_tig_1_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.tbuf_word_count_tig_1_reg[*]}]


# STATUS
set_false_path -to [get_cells {status_reg[*]}]


# Auto generated
#set_false_path -to [list [get_cells main_i/engine_0/dram_clk_sync_reset_reg] [get_cells [list main_i/engine_0/dram_clk_sync_reset_reg_replica main_i/engine_0/dram_clk_sync_reset_reg_replica_1 main_i/engine_0/dram_clk_sync_reset_reg_replica_2 main_i/engine_0/dram_clk_sync_reset_reg_replica_3 main_i/engine_0/dram_clk_sync_reset_reg_replica_4 main_i/engine_0/dram_clk_sync_reset_reg_replica_5 main_i/engine_0/dram_clk_sync_reset_reg_replica_6]]]

#set_false_path -to [list [get_cells main_i/engine_1/dram_clk_sync_reset_reg] [get_cells [list main_i/engine_1/dram_clk_sync_reset_reg_replica main_i/engine_1/dram_clk_sync_reset_reg_replica_1 main_i/engine_1/dram_clk_sync_reset_reg_replica_2 main_i/engine_1/dram_clk_sync_reset_reg_replica_3 main_i/engine_1/dram_clk_sync_reset_reg_replica_4 main_i/engine_1/dram_clk_sync_reset_reg_replica_5 main_i/engine_1/dram_clk_sync_reset_reg_replica_6 main_i/engine_1/dram_clk_sync_reset_reg_replica_7 main_i/engine_1/dram_clk_sync_reset_reg_replica_8 main_i/engine_1/dram_clk_sync_reset_reg_replica_9]]]