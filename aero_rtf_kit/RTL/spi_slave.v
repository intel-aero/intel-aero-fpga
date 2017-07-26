// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
// ----------------------------------------------------------------------------

// Implementation of SPI slave mode 0

module spi_slave(
    clk,
    sclk,
    miso,
    mosi,
    ss,
    rx_byte_available,
    rx_byte,
    tx_byte_ready_to_write,
    tx_byte,
    transaction_begin
);

input wire clk;
input wire sclk;
input wire ss;

input wire mosi;
output reg rx_byte_available = 0;
output reg [0 : 7] rx_byte = 0;
reg [2 : 0] rx_index = 3'd0;

output reg miso = 1'b0;
output reg tx_byte_ready_to_write = 1'b0;
input wire [0 : 7] tx_byte;
reg [2 : 0] tx_index = 3'd0;

output reg transaction_begin = 0;

reg [1 :0] sclk_reg;
reg [1 :0] ss_reg;
wire sclk_rissing_edge;
wire sclk_falling_edge;
wire ss_falling_edge;

assign sclk_rissing_edge = (sclk_reg == 2'b01);
assign sclk_falling_edge = (sclk_reg == 2'b10);
assign ss_falling_edge = (ss_reg == 2'b10);

always @ (posedge clk) begin
    sclk_reg[0] <= sclk;
    sclk_reg[1] <= sclk_reg[0];

    ss_reg[0] <= ss;
    ss_reg[1] <= ss_reg[0];

    transaction_begin <= ss_falling_edge;
end

// RX
always @ (posedge clk) begin
    if (ss_falling_edge) begin
        rx_index <= 3'd0;
        rx_byte_available <= 0;
    end else if (sclk_rissing_edge) begin
        rx_byte[rx_index] <= mosi;
        rx_index <= rx_index + 3'd1;
        if (rx_index == 3'd7) begin
            rx_byte_available <= 1;
        end else begin
            rx_byte_available <= 0;
        end
    end
end

// TX
always @ (posedge clk) begin
    if (ss_falling_edge) begin
        /*
         * As in mode 0, the sclk is low when the spi slave is selected:
         * tx_index <= 3'd1 because the 7th bit of the first byte is not
         * transmitted in this implementation.
         */
        tx_index <= 3'd1;
        tx_byte_ready_to_write <= 0;
    end else if (sclk_falling_edge) begin
        miso <= tx_byte[tx_index];
        tx_index <= tx_index + 3'd1;
        if (tx_index == 3'd7) begin
            tx_byte_ready_to_write <= 1;
        end else begin
            tx_byte_ready_to_write <= 0;
        end
    end
end

endmodule