// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
// ----------------------------------------------------------------------------

`timescale 1ns/1ps

//--------------------------
//    Declaration and ports
//-----------
module Top (
        // global
        in_CLK,

        // Shared I2C(ADC and external)
        FC1_I2C_CLK,
        FC1_I2C_SDA,

        // I2C external compass
        FC1_COMPASS_CLK,
        FC1_COMPASS_SDA,
        IO_COMPASS_CLK,
        IO_COMPASS_SDA,

        // UART motors
        IO_MOTORS_Tx,
        IO_MOTORS_Rx,
        FC1_MOTORS_SCL_Tx,
        FC1_MOTORS_SDA_Rx,

        // UART GPS
        IO_GPS_Tx,
        IO_GPS_Rx,
        FC1_GPS_Tx,
        FC1_GPS_Rx,

        // UART RC receiver
        IO_REC_Rx,
        IO_REC_Tx,
        FC1_IO3_REC_Rx,
        FC1_XBEE_CTS_REC_Tx,

        // UART CHT<>FC
        FC1_XBEE_Rx,
        FC1_XBEE_Tx,
        CHT_DBG_UART_Tx,
        CHT_DBG_UART_Rx,

        // UART telemetry
        IO_TELEM_Tx,
        IO_TELEM_Rx,
        FC1_TELEM_Tx,
        FC1_TELEM_Rx,

        // SPI CHT
        SPI_SCLK,
        SPI_MISO,
        SPI_MOSI,
        SPI_SS,

        // Telemetry I2C
        IO_TELEM_I2C_CLK,
        IO_TELEM_I2C_SDA,

        BOOTLOADER_FORCE_PIN
);

parameter fpga_ver = 8'hC2;

// global
input wire in_CLK;

// Shared I2C
input wire FC1_I2C_CLK;
inout wire FC1_I2C_SDA;

// External compass I2C
output wire IO_COMPASS_CLK;
inout wire IO_COMPASS_SDA;
input wire FC1_COMPASS_CLK;
inout wire FC1_COMPASS_SDA;

output wire IO_MOTORS_Tx;
input  wire IO_MOTORS_Rx;
input  wire FC1_MOTORS_SCL_Tx;
output wire FC1_MOTORS_SDA_Rx;

output wire IO_GPS_Tx;
input  wire IO_GPS_Rx;
input  wire FC1_GPS_Tx;
output wire FC1_GPS_Rx;

input  wire IO_REC_Rx;
output wire IO_REC_Tx;
output wire FC1_IO3_REC_Rx;
input  wire FC1_XBEE_CTS_REC_Tx;

input  wire FC1_XBEE_Tx;
output wire FC1_XBEE_Rx;
input  wire CHT_DBG_UART_Tx;
output wire CHT_DBG_UART_Rx;

output wire IO_TELEM_Tx;
input  wire IO_TELEM_Rx;
input  wire FC1_TELEM_Tx;
output wire FC1_TELEM_Rx;

input wire SPI_SCLK;
output wire SPI_MISO;
input wire SPI_MOSI;
input wire SPI_SS;

output wire IO_TELEM_I2C_CLK;
inout wire IO_TELEM_I2C_SDA;

output reg BOOTLOADER_FORCE_PIN = 0;

assign FC1_MOTORS_SDA_Rx = IO_MOTORS_Rx;
assign IO_MOTORS_Tx = FC1_MOTORS_SCL_Tx;
assign IO_GPS_Tx = FC1_GPS_Tx;
assign FC1_GPS_Rx = IO_GPS_Rx;
assign IO_REC_Tx = FC1_XBEE_CTS_REC_Tx;
assign FC1_IO3_REC_Rx = IO_REC_Rx;
assign CHT_DBG_UART_Rx = FC1_XBEE_Tx;
assign FC1_XBEE_Rx = CHT_DBG_UART_Tx;
assign FC1_TELEM_Rx = IO_TELEM_Rx;
assign IO_TELEM_Tx = FC1_TELEM_Tx;

//--------------------------
// Regs and Wires
//-----------
reg reset_n0;
reg reset_n;
wire clk_core;
wire clk_adc;
wire locked;

//--------------------------
//    PLL
//-----------
pll pll_inst (
    .inclk0(in_CLK),
    .c0( clk_adc),  // 10 MHz clock dedicated to ADC
    .c1( clk_core), // 50 MHz sytem clock
    .locked(locked)
);

//--------------------------
// Reset - Synchronous deassert after PLL locked
//-----------
// Synchronize Reset
always @(posedge in_CLK) begin
    if (!locked) begin
        reset_n0 <= 1'b0;
        reset_n <= 1'b0;
    end else begin
        reset_n0 <= 1'b1;
        reset_n <= reset_n0;
    end
end

// I2C in telemetry connector
i2c_bridge_new i2c_telemetry_connector_bridge_inst(
    .clk(clk_core),
    .master_sda(FC1_I2C_SDA),
    .master_clk(FC1_I2C_CLK),
    .slave_sda(IO_TELEM_I2C_SDA),
    .slave_clk(IO_TELEM_I2C_CLK)
);

// I2C bridge external compass
i2c_bridge_new i2c_external_compass_bridge_inst(
    .clk(clk_core),
    .master_sda(FC1_COMPASS_SDA),
    .master_clk(FC1_COMPASS_CLK),
    .slave_sda(IO_COMPASS_SDA),
    .slave_clk(IO_COMPASS_CLK)
);

// ADC
adc_state_machine adc_inst(
    .clk_core(clk_core),
    .clk_adc(clk_adc),
    .i2c_clk(FC1_I2C_CLK),
    .i2c_sda(FC1_I2C_SDA),
    .reset_n(reset_n),
    .locked(locked)
);

// SPI

wire spi_rx_byte_available;
wire [7 : 0] spi_rx_byte;
wire spi_tx_ready_to_write;
reg [7 : 0] spi_tx_byte;

// SPI state machine
reg waiting_reg = 0;
reg [7 : 0] reg_received = 0;
reg [1 :0] spi_rx_byte_available_reg;
wire spi_rx_byte_available_rissing_edge;
wire spi_transaction_begin;

spi_slave spi0_inst(
    .clk(clk_core),
    .sclk(SPI_SCLK),
    .miso(SPI_MISO),
    .mosi(SPI_MOSI),
    .ss(SPI_SS),
    .rx_byte_available(spi_rx_byte_available),
    .rx_byte(spi_rx_byte),
    .tx_byte_ready_to_write(spi_tx_ready_to_write),
    .tx_byte(spi_tx_byte),
    .transaction_begin(spi_transaction_begin)
);

assign spi_rx_byte_available_rissing_edge = (spi_rx_byte_available_reg == 2'b01);

// helper to detect edges
always @ (posedge clk_core) begin
    spi_rx_byte_available_reg[0] <= spi_rx_byte_available;
    spi_rx_byte_available_reg[1] <= spi_rx_byte_available_reg[0];
end

// to keep it compatible with read version operation the 8th bit of first byte
// will be used this way:
// - set to 0 for read
// - set to 1 for write
//
// This has to be ORed with the register in the table below according to the
// operation. Bellow is the current register table (using 7bit register addressing).
// All other registers are reserved for future use.
//
// ------------------------------------------------------------------------------------
// | Reg  | Name             | Mode | Description                                     |
// |----------------------------------------------------------------------------------|
// | 0x00 | FPGA_FW_VERSION  |  RO  | FPGA firmware version                           |
// | 0x01 | AEROFC_FORCE_BT  |  RW  | Pin state to force aerofc to stay on bootloader |
// ------------------------------------------------------------------------------------

parameter fpga_ver_read_reg = 7'd0;
parameter fpga_bootloader_pin_reg = 7'd1;

// SPI state machine
always @ (posedge clk_core) begin
    if (spi_transaction_begin) begin
        waiting_reg <= 1;
        spi_tx_byte <= 0;
    end else if (spi_rx_byte_available_rissing_edge) begin
        if (waiting_reg) begin
            waiting_reg <= 0;
            reg_received <= spi_rx_byte;

            // is a read operation?
            if (spi_rx_byte[7] == 0) begin
                if (spi_rx_byte[6:0] == fpga_ver_read_reg) begin
                    // read FPGA version, write 1 byte with the version
                    spi_tx_byte <= fpga_ver;
                end
                if (spi_rx_byte[6:0] == fpga_bootloader_pin_reg) begin
                    spi_tx_byte[0] <= BOOTLOADER_FORCE_PIN;
                end
            end
        end else begin
            // is a write operation?
            if (reg_received[7] == 1) begin
                if (reg_received[6:0] == fpga_bootloader_pin_reg) begin
                    BOOTLOADER_FORCE_PIN <= spi_rx_byte[0];
                end
            end
        end
    end
end

endmodule
