/*
 * ---------------------------------------------------------------------------------|
 * | Reg    | Name                  | Mode  | Description                           |
 * |--------------------------------------------------------------------------------|
 * | 0x00   | adc_config_reg        | RW    | bit0 controls if ADC is on or off     |
 * | 0x03   | ADC channel 0 low     | RO    | ADC channel 0 low byte value          |
 * | 0x04   | ADC channel 0 high    | RO    | ADC channel 0 high byte value         |
 * | 0x05   | ADC channel 1 low     | RO    | ADC channel 1 low byte value          |
 * | 0x06   | ADC channel 1 high    | RO    | ADC channel 1 high byte value         |
 * | 0x07   | ADC channel 2 low     | RO    | ADC channel 2 low byte value          |
 * | 0x08   | ADC channel 2 high    | RO    | ADC channel 2 high byte value         |
 * | 0x09   | ADC channel 3 low     | RO    | ADC channel 3 low byte value          |
 * | 0x0A   | ADC channel 3 high    | RO    | ADC channel 3 high byte value         |
 * | 0x0B   | ADC channel 4 low     | RO    | ADC channel 4 low byte value          |
 * | 0x0C   | ADC channel 4 high    | RO    | ADC channel 4 high byte value         |
 * ----------------------------------------------------------------------------------
 *
 * Channel 0: ADC1_IN3
 * Channel 1: ADC1_IN6
 * Channel 2: ADC1_IN1
 * Channel 3: ADC1_IN2
 * Channel 4: ADC1_IN4
 */

module adc_state_machine(
    clk_core,
    clk_adc,
    i2c_clk,
    i2c_sda,
    reset_n,
    locked
);

parameter adc_slave_addr = 7'h50;

input wire clk_core;
input wire clk_adc;

input wire i2c_clk;
inout wire i2c_sda;

input wire reset_n;
input wire locked;

wire adc_response_valid;

wire [11:0] adc_response_data;
wire [4:0] adc_response_channel;
reg [11:0] adc_ch0_raw_data;
reg [11:0] adc_ch1_raw_data;
reg [11:0] adc_ch2_raw_data;
reg [11:0] adc_ch3_raw_data;
reg [11:0] adc_ch4_raw_data;
wire adc_response_startofpacket;
wire adc_response_endofpacket;
reg [7:0] reg_ch0_upper, reg_ch0_lower;
reg [7:0] reg_ch1_upper, reg_ch1_lower;
reg [7:0] reg_ch2_upper, reg_ch2_lower;
reg [7:0] reg_ch3_upper, reg_ch3_lower;
reg [7:0] reg_ch4_upper, reg_ch4_lower;

reg adc_sequencer_csr_address = 1'd0;
reg adc_sequencer_csr_read = 1'd0;
reg adc_sequencer_csr_write = 1'd0;
reg [31:0] adc_sequencer_csr_readdata = 32'd0;
reg [31:0] adc_sequencer_csr_writedata = 32'd0;

reg samples_ready = 1'd0;

adc adc_inst(
    .adc_adc_pll_clock_clk(clk_adc),                               // adc_adc_pll_clock.clk
    .adc_adc_pll_locked_export(locked),                            // adc_adc_pll_locked.export
    .adc_response_valid(adc_response_valid),                       // adc_response.valid
    .adc_response_channel(adc_response_channel),                   //             .channel
    .adc_response_data(adc_response_data),                         //             .data
    .adc_response_startofpacket(adc_response_startofpacket),       //             .startofpacket
    .adc_response_endofpacket(adc_response_endofpacket),           //             .endofpacket
    .adc_sequencer_csr_address(adc_sequencer_csr_address),         // adc_sequencer_csr.address
    .adc_sequencer_csr_read(adc_sequencer_csr_read),               //                  .read
    .adc_sequencer_csr_write(adc_sequencer_csr_write),             //                  .write
    .adc_sequencer_csr_writedata(adc_sequencer_csr_writedata),     //                  .writedata
    .adc_sequencer_csr_readdata(adc_sequencer_csr_readdata),       //                  .readdata
    .clk_clk(clk_core),                                            // clk.clk
    .reset_reset_n(reset_n)                                        // reset.reset_n
);

always @(posedge clk_core) begin
    if (adc_response_valid) begin
        case (adc_response_channel)
        5'h03: adc_ch0_raw_data <= adc_response_data;
        5'h06: adc_ch1_raw_data <= adc_response_data;
        5'h01: adc_ch2_raw_data <= adc_response_data;
        5'h02: adc_ch3_raw_data <= adc_response_data;
        5'h04: adc_ch4_raw_data <= adc_response_data;
        endcase
        samples_ready <= adc_response_endofpacket;
    end

    /*
     * The last adc_response_channel and adc_response_endofpacket are set
     * in the same posedge, so a copying adc_chX_raw_data to reg_chx_upper
     * would result in copying the old value.
     * So setting samples_ready to adc_response_endofpacket above would
     * cause the samples_ready == 1 in the next posedge when all the
     * adc_chX_raw_data are syncronized.
     */
    if (samples_ready) begin
        /*
         * Only update the registers that I2C reads when slave is not asserted,
         * this avoids a non-syncronized read between lower and upper bytes.
         */
        if (!slave_asserted) begin
            { reg_ch0_upper, reg_ch0_lower } <= { 4'b0, adc_ch0_raw_data[11:0] };
            { reg_ch1_upper, reg_ch1_lower } <= { 4'b0, adc_ch1_raw_data[11:0] };
            { reg_ch2_upper, reg_ch2_lower } <= { 4'b0, adc_ch2_raw_data[11:0] };
            { reg_ch3_upper, reg_ch3_lower } <= { 4'b0, adc_ch3_raw_data[11:0] };
            { reg_ch4_upper, reg_ch4_lower } <= { 4'b0, adc_ch4_raw_data[11:0] };
            samples_ready <= 1'd0;
        end
    end
end

// I2C
wire slave_asserted;
wire slave_in_tx_mode;
reg [7:0] slave_tx_buffer;
wire slave_tx_request;
wire [7:0] slave_rx_buffer;
wire slave_rx_available;

reg [1:0] slave_tx_request_edge;
reg [1:0] slave_rx_available_edge;
reg slave_tx_request_rising_edge = 0;
reg slave_rx_available_rising_edge = 0;

reg [7:0] i2c_slave_reg;
reg [3:0] i2c_slave_bytes_rx;

// bit0 = start or stop ADC
reg [7:0] adc_config_reg;

i2c_slave i2c_slave_inst(
    .clk(clk_core),
    .master_clk(i2c_clk),
    .master_sda(i2c_sda),
    .slave_addr(adc_slave_addr),
    .slave_asserted(slave_asserted),
    .slave_in_tx_mode(slave_in_tx_mode),
    .slave_tx_buffer(slave_tx_buffer),
    .slave_tx_request(slave_tx_request),
    .slave_rx_buffer(slave_rx_buffer),
    .slave_rx_available(slave_rx_available)
);

// helper to detect edges
always @(posedge clk_core) begin
    slave_tx_request_edge <= { slave_tx_request_edge[0], slave_tx_request };
    slave_rx_available_edge <= { slave_rx_available_edge[0], slave_rx_available };
    if (slave_tx_request_edge[0] && !slave_tx_request_edge[1]) begin
        slave_tx_request_rising_edge <= 1'd1;
    end else begin
        slave_tx_request_rising_edge <= 1'd0;
    end
    if (slave_rx_available_edge[0] && !slave_rx_available_edge[1]) begin
        slave_rx_available_rising_edge <= 1'd1;
    end else begin
        slave_rx_available_rising_edge <= 1'd0;
    end
end

always @(posedge clk_core) begin
    if (slave_asserted) begin
        if (slave_in_tx_mode) begin
            if (slave_tx_request_rising_edge) begin
                case (i2c_slave_reg)
                8'd0:
                    slave_tx_buffer <= adc_config_reg;
                8'd3:
                    slave_tx_buffer <= reg_ch0_lower;
                8'd4:
                    slave_tx_buffer <= reg_ch0_upper;
                8'd5:
                    slave_tx_buffer <= reg_ch1_lower;
                8'd6:
                    slave_tx_buffer <= reg_ch1_upper;
                8'd7:
                    slave_tx_buffer <= reg_ch2_lower;
                8'd8:
                    slave_tx_buffer <= reg_ch2_upper;
                8'd9:
                    slave_tx_buffer <= reg_ch3_lower;
                8'd10:
                    slave_tx_buffer <= reg_ch3_upper;
                8'd11:
                    slave_tx_buffer <= reg_ch4_lower;
                8'd12:
                    slave_tx_buffer <= reg_ch4_upper;
                default:
                    slave_tx_buffer <= 8'd0;
                endcase;
                i2c_slave_reg <= i2c_slave_reg + 8'd1;
            end
        end else begin
            if (slave_rx_available_rising_edge) begin
                if (i2c_slave_bytes_rx == 4'd0) begin
                    i2c_slave_reg <= slave_rx_buffer;
                end else if (i2c_slave_bytes_rx == 4'd1) begin
                    if (i2c_slave_reg == 8'd0) begin
                        adc_config_reg <= slave_rx_buffer;
                        adc_sequencer_csr_writedata <= { 31'd0, slave_rx_buffer[0] };
                        adc_sequencer_csr_write <= 32'd1;
                    end
                end
                i2c_slave_bytes_rx <= i2c_slave_bytes_rx + 4'd1;
            end
        end
    end else begin
        i2c_slave_bytes_rx <= 4'd0;
    end
end

endmodule
