
module adc (
	adc_adc_pll_clock_clk,
	adc_adc_pll_locked_export,
	adc_response_valid,
	adc_response_channel,
	adc_response_data,
	adc_response_startofpacket,
	adc_response_endofpacket,
	adc_sequencer_csr_address,
	adc_sequencer_csr_read,
	adc_sequencer_csr_write,
	adc_sequencer_csr_writedata,
	adc_sequencer_csr_readdata,
	clk_clk,
	reset_reset_n);	

	input		adc_adc_pll_clock_clk;
	input		adc_adc_pll_locked_export;
	output		adc_response_valid;
	output	[4:0]	adc_response_channel;
	output	[11:0]	adc_response_data;
	output		adc_response_startofpacket;
	output		adc_response_endofpacket;
	input		adc_sequencer_csr_address;
	input		adc_sequencer_csr_read;
	input		adc_sequencer_csr_write;
	input	[31:0]	adc_sequencer_csr_writedata;
	output	[31:0]	adc_sequencer_csr_readdata;
	input		clk_clk;
	input		reset_reset_n;
endmodule
