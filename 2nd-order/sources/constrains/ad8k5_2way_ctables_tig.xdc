set_false_path -to [get_cells {status_reg[*]}]

set_false_path -to [get_cells {main_i/host_reset_ramclk_reg}]
set_false_path -to [get_cells {main_i/host_reset_pciclk_reg}]
set_false_path -to [get_cells {main_i/pci_clk_sync_reset_reg}]
set_false_path -to [get_cells {main_i/dram_clk_sync_reset_reg}]
#set_false_path -to [get_cells {main_i/stream_clk_sync_reset_reg}]

set_false_path -from [get_cells {main_i/num_samples_rounded_tig_reg[*]}]
set_false_path -from [get_cells {main_i/num_cases_rounded_tig_reg[*]}]
set_false_path -from [get_cells {main_i/num_snps_local_tig_reg[*]}]
set_false_path -from [get_cells {main_i/last_init_snpidx_tig_reg[*]}]
set_false_path -from [get_cells {main_i/stream_start_snpidx_tig_reg[*]}]
set_false_path -from [get_cells {main_i/stream_start_addr_tig_reg[*]}]
set_false_path -from [get_cells {main_i/round_addr_offset_tig_reg[*]}]

set_false_path -to [get_cells {main_i/stall_reg}]

set_false_path -from [get_cells {main_i/io_p.last_raw_gt_word_tig_reg[*]}]
set_false_path -to [get_cells {main_i/ctable_io_bufsize_outwords_tig_ramclk_reg[*]}]
set_false_path -to [get_cells {main_i/ctable_io_bufsize_tablewords_tig_ramclk_reg[*]}]
set_false_path -to [get_cells {main_i/ctable_io_bufsize_outwords_tig_pciclk_reg[*]}]
set_false_path -to [get_cells {main_i/ctable_io_bufsize_tablewords_tig_pciclk_reg[*]}]

set_false_path -to [get_cells {main_i/snpreader_i/busy_tig_reg}]
set_false_path -to [get_cells {main_i/out_map_p.snpreader_busy_tig_reg}]

set_false_path -to [get_cells {main_i/pf_sync_p.process_finished_tig_reg}]

set_false_path -to [get_cells {main_i/reg_dbg_p.c0_rd_req_cnt_tig_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.c0_rd_ans_cnt_tig_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.c0_rd_ack_cnt_tig_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.c0_conflict_cnt_tig_reg[*]}]

set_false_path -to [get_cells {main_i/reg_dbg_p.ctchain_debug_tig_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.tbuf_table_count_tig_reg[*]}]
set_false_path -to [get_cells {main_i/reg_dbg_p.tbuf_word_count_tig_reg[*]}]