/*
 * -------------------------------------------------------------------------------------|
 * | Reg    | Name                  | Mode  | Description                               |
 * |------------------------------------------------------------------------------------|
 * | 0x00   | FIRMWARE_VERSION      | RO    | FPGA firmware version                     |
 * |------------------------------------------------------------------------------------|
 * | 0x01   | UART_INVERTED         | RW    | bit0 = inverts RC UART                    |
 * |        |                       |       | bit1 = inverts telemetry UART             |
 * |------------------------------------------------------------------------------------|
 * | 0x02   | TELEMETRY_CON_SEL     | RW    | bit0 clean = enable UART and disable I2C  |
 * |        |                       |       | bit0 set = disable UART  I2C enable I2C   |
 * |------------------------------------------------------------------------------------|
 */

module fc_config(
    clk_core,
    i2c_clk,
    i2c_sda,
    fpga_firmware_version,
    uart_inverted,
    telemetry_con_sel
);

parameter config_slave_addr = 7'h51;

input wire clk_core;

input wire i2c_clk;
inout wire i2c_sda;

input wire [7:0] fpga_firmware_version;
output reg [7:0] uart_inverted = 0;
output reg [7:0] telemetry_con_sel = 0;

// internal data

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

parameter firmware_version_reg = 8'h0;
parameter uart_inverted_reg = 8'h1;
parameter telemetry_con_sel_reg = 8'h2;

i2c_slave i2c_slave_inst(
    .clk(clk_core),
    .master_clk(i2c_clk),
    .master_sda(i2c_sda),
    .slave_addr(config_slave_addr),
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
                firmware_version_reg:
                    slave_tx_buffer <= fpga_firmware_version;
                uart_inverted_reg:
                    slave_tx_buffer <= uart_inverted;
                telemetry_con_sel_reg:
                    slave_tx_buffer <= telemetry_con_sel;
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
                    case (i2c_slave_reg)
                    uart_inverted_reg:
                        uart_inverted <= slave_rx_buffer;
                    telemetry_con_sel_reg:
                        telemetry_con_sel <= slave_rx_buffer;
                    endcase;
                    i2c_slave_reg <= i2c_slave_reg + 8'd1;
                end
                i2c_slave_bytes_rx <= i2c_slave_bytes_rx + 4'd1;
            end
        end
    end else begin
        i2c_slave_bytes_rx <= 4'd0;
    end
end

endmodule