	adc u0 (
		.adc_adc_pll_clock_clk       (<connected-to-adc_adc_pll_clock_clk>),       //  adc_adc_pll_clock.clk
		.adc_adc_pll_locked_export   (<connected-to-adc_adc_pll_locked_export>),   // adc_adc_pll_locked.export
		.adc_response_valid          (<connected-to-adc_response_valid>),          //       adc_response.valid
		.adc_response_channel        (<connected-to-adc_response_channel>),        //                   .channel
		.adc_response_data           (<connected-to-adc_response_data>),           //                   .data
		.adc_response_startofpacket  (<connected-to-adc_response_startofpacket>),  //                   .startofpacket
		.adc_response_endofpacket    (<connected-to-adc_response_endofpacket>),    //                   .endofpacket
		.adc_sequencer_csr_address   (<connected-to-adc_sequencer_csr_address>),   //  adc_sequencer_csr.address
		.adc_sequencer_csr_read      (<connected-to-adc_sequencer_csr_read>),      //                   .read
		.adc_sequencer_csr_write     (<connected-to-adc_sequencer_csr_write>),     //                   .write
		.adc_sequencer_csr_writedata (<connected-to-adc_sequencer_csr_writedata>), //                   .writedata
		.adc_sequencer_csr_readdata  (<connected-to-adc_sequencer_csr_readdata>),  //                   .readdata
		.clk_clk                     (<connected-to-clk_clk>),                     //                clk.clk
		.reset_reset_n               (<connected-to-reset_reset_n>)                //              reset.reset_n
	);

