// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
///
//  ADC I2C Address (7 bits) 1010xxx 
//                   xxx - 000 -  ADC Control Register (RW)
//                       - 001 -  ADC Channel 0 VBAT_SENSE
//                       - 010 -  ADC Channel 1 CUR_SENSE
//                       - 011 -  ADC Channel 2 MAIN BAT SENSE
//                       - 100 -  ADC Channel 3 RESERVE BAT SENSE
//                       - 101 -  ADC Channel 4 GPS SENSE
//                       - 110/111 - dont care - Return 0000h
//
// ADC Write 50,01 - Enable ADC
// ADC Write 50,00 - Disable ADC
// ADC Read 53, xx, xx - ADC channel0
// ADC Read 55, xx, xx - ADC channel1
// ADC Read 57, xx, xx - ADC channel2
// ADC Read 59, xx, xx - ADC channel3
// ADC Read 5B, xx, xx - ADC channel4
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

parameter fpga_ver = 8'hC1;

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
reg adc_response_valid;
reg [4:0] adc_response_channel;
reg [11:0] adc_response_data;
reg adc_response_startofpacket;
reg adc_response_endofpacket;
reg adc_sequencer_csr_address;
reg adc_sequencer_csr_read;
reg adc_sequencer_csr_write;
reg [31:0] adc_sequencer_csr_writedata;
reg [31:0] adc_sequencer_csr_readdata;
wire adc_cmd;
reg [7:0] reg_ch0_upper, reg_ch0_lower;
reg [7:0] reg_ch1_upper, reg_ch1_lower;
reg [7:0] reg_ch2_upper, reg_ch2_lower;
reg [7:0] reg_ch3_upper, reg_ch3_lower;
reg [7:0] reg_ch4_upper, reg_ch4_lower;
reg [11:0] adc_ch0_raw_data;
reg [11:0] adc_ch1_raw_data;
reg [11:0] adc_ch2_raw_data;
reg [11:0] adc_ch3_raw_data;
reg [11:0] adc_ch4_raw_data;

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

// Issue ADC sequencer start and stop commands
always @(posedge clk_core or negedge reset_n) begin
   if (!reset_n) begin
      adc_sequencer_csr_address <= 1'b0;
      adc_sequencer_csr_read <= 1'b0;
      adc_sequencer_csr_write <= 1'b0;
      adc_sequencer_csr_writedata <= 32'b0;
   end else begin
      if (adc_on) begin
          // ADC RUN
          adc_sequencer_csr_address <= 1'b0;
          adc_sequencer_csr_read <= 1'b0;
          adc_sequencer_csr_write <= 1'b1;
          adc_sequencer_csr_writedata <= 32'b1;
      end else if (~adc_on) begin
          // ADC STOP
          adc_sequencer_csr_address <= 1'b0;
          adc_sequencer_csr_read <= 1'b0;
          adc_sequencer_csr_write <= 1'b1;
          adc_sequencer_csr_writedata <= 32'b0;
       end else begin
          adc_sequencer_csr_address <= 1'b0;
          adc_sequencer_csr_read <= 1'b0;
          adc_sequencer_csr_write <= 1'b0;
          adc_sequencer_csr_writedata <= 32'b0;
      end
   end
end

//--------------------------
//    ADC- 7 channels: ch 1,2,3,4 &6
//-----------
adc adc_inst(
        .adc_adc_pll_clock_clk (clk_adc),                               //  adc_adc_pll_clock.clk
        .adc_adc_pll_locked_export (locked),                            // adc_adc_pll_locked.export
        .adc_response_valid (adc_response_valid),                       //       adc_response.valid
        .adc_response_channel (adc_response_channel),                   //                   .channel
        .adc_response_data (adc_response_data),                         //                   .data
        .adc_response_startofpacket (adc_response_startofpacket),       //                   .startofpacket
        .adc_response_endofpacket (adc_response_endofpacket),           //                   .endofpacket
        .adc_sequencer_csr_address (adc_sequencer_csr_address),         //  adc_sequencer_csr.address
        .adc_sequencer_csr_read (adc_sequencer_csr_read),               //                   .read
        .adc_sequencer_csr_write (adc_sequencer_csr_write),             //                   .write
        .adc_sequencer_csr_writedata (adc_sequencer_csr_writedata),     //                   .writedata
        .adc_sequencer_csr_readdata (adc_sequencer_csr_readdata),       //                   .readdata
        .clk_clk (clk_core),                                            //                clk.clk
        .reset_reset_n (reset_n)                                        //              reset.reset_n
    );



//--------------------------
//    Capture ADC channel data
//-----------
always @(posedge clk_core or negedge reset_n) begin
   if (!reset_n) begin
        adc_ch0_raw_data <= 12'b0;
        adc_ch1_raw_data <= 12'b0;
        adc_ch2_raw_data <= 12'b0;
        adc_ch3_raw_data <= 12'b0;
        adc_ch4_raw_data <= 12'b0;
   end else begin
      if (adc_response_valid) begin
         case (adc_response_channel)
            5'h01: adc_ch0_raw_data <= adc_response_data;
            5'h02: adc_ch1_raw_data <= adc_response_data;
            5'h03: adc_ch2_raw_data <= adc_response_data;
            5'h04: adc_ch3_raw_data <= adc_response_data;
            5'h06: adc_ch4_raw_data <= adc_response_data;
         endcase
      end
   end
end

always @(posedge clk_core or negedge reset_n) begin
    if (!reset_n) begin
      reg_ch0_upper <= 8'b0;
      reg_ch0_lower <= 8'b0;
      reg_ch1_upper <= 8'b0;
      reg_ch1_lower <= 8'b0;
      reg_ch2_upper <= 8'b0;
      reg_ch2_lower <= 8'b0;
      reg_ch3_upper <= 8'b0;
      reg_ch3_lower <= 8'b0;
      reg_ch4_upper <= 8'b0;
      reg_ch4_lower <= 8'b0;
    end else begin
       if (adc_response_endofpacket) begin
            {reg_ch0_upper, reg_ch0_lower} <= {4'b0, adc_ch0_raw_data[11:0]};
            {reg_ch1_upper, reg_ch1_lower} <= {4'b0, adc_ch1_raw_data[11:0]};
            {reg_ch2_upper, reg_ch2_lower} <= {4'b0, adc_ch2_raw_data[11:0]};
            {reg_ch3_upper, reg_ch3_lower} <= {4'b0, adc_ch3_raw_data[11:0]};
            {reg_ch4_upper, reg_ch4_lower} <= {4'b0, adc_ch4_raw_data[11:0]};
        end
    end
end

wire master_wr;
wire slave_wr;

reg [3:0] bit_count;
reg rd_wr;
reg addr_set;
reg clk_posedge;
reg clk_negedge;
reg i2c_start;
reg i2c_stop;
reg adc_sda;
reg i2c_busy;
reg [3:0] add_addr;
reg adc_cs;
reg adc_on;
reg i2c_passon;
reg adc_addr_set;
reg adc_rd_wr;
reg [7:0]slave_addr;
reg [7:0] adc_data;
reg [3:0] adc_addr;
reg [3:0] i2c_reg_addr;
reg adc_wr_set;
reg i2c_reset;

assign IO_TELEM_I2C_CLK = FC1_I2C_CLK;

assign IO_TELEM_I2C_SDA = (master_wr) ? FC1_I2C_SDA: 1'bz;

assign FC1_I2C_SDA = ((adc_ack || adc_txd) && ~i2c_reset) ? adc_sda: (slave_wr) ? IO_TELEM_I2C_SDA:  1'bz;

wire ack_on;
assign ack_on = (bit_count == 4'h9)? 1'b1: 1'b0;

reg [1:0] r_msda;
reg [1:0] r_mscl;

always @(posedge clk_core) begin
   r_msda <= {r_msda[0], FC1_I2C_SDA};
   r_mscl <= {r_mscl[0], FC1_I2C_CLK};
end

always @(posedge clk_core) begin
   i2c_start <=  r_msda[1] && ~r_msda[0] && r_mscl[0];
   i2c_stop <=  r_msda[0] && ~r_msda[1] && r_mscl[0];
end

always @(posedge clk_core) begin
    if (r_mscl[0] && ~r_mscl[1])
        clk_posedge <= 1'b1;
    else
        clk_posedge <= 1'b0;
end

always @(posedge clk_core) begin
    if (~r_mscl[0] && r_mscl[1])
        clk_negedge <= 1'b1;
    else
        clk_negedge <= 1'b0;
end

always @(posedge clk_core or negedge reset_n) begin
    if (~reset_n)
        i2c_busy <= 1'b0;
    else if (i2c_start)
        i2c_busy <= 1'b1;
    else if (i2c_stop || i2c_reset)
        i2c_busy <= 1'b0;
end


always @(posedge clk_core or negedge reset_n ) begin
   if (!reset_n) begin
      bit_count <= 4'h00;
      rd_wr <= 1'b0;
       addr_set <= 1'b0;
    end
    else if(i2c_stop || i2c_start || i2c_reset) begin
       bit_count <= 4'h00;
        rd_wr <= 1'b0;
        addr_set <= 1'b0;
    end        
    else if(~addr_set && (bit_count == 4'h8) && clk_posedge)
       rd_wr <= r_msda[0];
   else if(bit_count == 4'hA)begin
        bit_count <= 4'h1;
        addr_set <= 1'b1;
    end
    else if (clk_negedge)
        bit_count <= bit_count + 4'h1;
end

always @(posedge clk_core or negedge reset_n) begin
    if (!reset_n ) begin
        adc_cs <= 1'b0;
        slave_addr <= 8'h00;
        i2c_passon <= 1'b0;
        adc_rd_wr <= 1'b0;
        adc_addr_set <= 1'b0;
        adc_on <= 1'b0;
        adc_wr_set <= 1'b0;
        i2c_reg_addr <= 4'b0000;
    end else if(i2c_start || i2c_stop || i2c_reset) begin
        adc_cs <= 1'b0;
        slave_addr <= 8'h00;
        i2c_passon <= 1'b0;
        adc_rd_wr <= 1'b0;
        adc_addr_set <= 1'b0;
        adc_wr_set <= 1'b0;
    end else if(~adc_cs && ack_on && ~i2c_passon ) begin
        if (slave_addr[7:1] == 7'b1010000) begin
            adc_cs <= 1'b1;
            adc_rd_wr <= slave_addr[0];
        end else
           i2c_passon <= 1'b1;
    end else if((~addr_set || ~adc_rd_wr)  && clk_posedge && i2c_busy && ~i2c_passon && ~ack_on)
       slave_addr <= {slave_addr[6:0], FC1_I2C_SDA};
    else if(adc_cs && ~adc_rd_wr && addr_set && ack_on && clk_posedge) begin
        if ( ~adc_addr_set) begin
            adc_addr_set <= 1'b1;
            i2c_reg_addr <= slave_addr[3:0];
        end else if(i2c_reg_addr == 4'b0000 && ~adc_wr_set) begin
            adc_on <= slave_addr[0];
            adc_wr_set <= 1'b1;
        end
    end
end

wire adc_txd, adc_ack;
assign adc_ack =  adc_cs && ack_on  && (~addr_set  || ~adc_rd_wr);

always @(posedge clk_core or negedge reset_n)
begin
    if (!reset_n) begin
        adc_data <= 8'h00;
        adc_addr <= 4'h0;
        adc_sda <= 1'b1;
    end else if(adc_ack && ~clk_negedge) //adc_cs && ack_on && (~addr_set  || ~adc_rd_wr))
        adc_sda <= 1'b0;
    else if (~adc_cs && ack_on && ~i2c_passon)
        adc_addr <= {i2c_reg_addr[2:0],1'b0};
    else if(adc_cs && ack_on && adc_rd_wr && clk_negedge) begin
        case (adc_addr)
        4'b0000:
            adc_data <= {7'b0000001,adc_on};
        4'b0001:
            adc_data <= 8'hAF; //8'h00;
        4'b0010:
            adc_data <= reg_ch0_lower;
        4'b0011:
            adc_data <= reg_ch0_upper;
        4'b0100:
            adc_data <= reg_ch1_lower;
        4'b0101:
            adc_data <= reg_ch1_upper;
        4'b0110:
            adc_data <= reg_ch2_lower;
        4'b0111:
            adc_data <= reg_ch2_upper;
        4'b1000:
            adc_data <= reg_ch3_lower;
        4'b1001:
            adc_data <= reg_ch3_upper;
        4'b1010:
            adc_data <= reg_ch4_lower;
        4'b1011:
            adc_data <= reg_ch4_upper;
       endcase
       adc_addr <= adc_addr + 1'b1;
    end else if(adc_txd && clk_negedge) begin
        adc_data <= {adc_data[6:0],1'b0};
    end else if(adc_txd)
        adc_sda <= adc_data[7];
end

always @(posedge clk_core or negedge reset_n)
begin
   if (~reset_n)
       i2c_reset <= 1'b0;
   else if(ack_on && clk_posedge && FC1_I2C_SDA)
       i2c_reset <= 1'b1;
   else if (i2c_start || i2c_stop)
       i2c_reset <= 1'b0;
end

assign adc_txd =  adc_cs && adc_rd_wr && ~ack_on; // && clk_negedge;

assign slave_wr = (~addr_set && ack_on)  || (ack_on && ~rd_wr) || (rd_wr && addr_set && ~ack_on);
assign master_wr = ~slave_wr;

// SPI

wire spi_rx_byte_available;
wire [7 : 0] spi_rx_byte;
wire spi_tx_ready_to_write;
reg [7 : 0] spi_tx_byte;

// SPI state machine
reg waiting_reg = 0;
reg [7 : 0] reg_received = 0;
reg [1 :0] spi_rx_byte_available_reg;
reg [1 :0] ss_reg;
wire spi_rx_byte_available_rissing_edge;
wire ss_falling_edge;

spi_slave spi0_inst(
    .clk(clk_core),
    .sclk(SPI_SCLK),
    .miso(SPI_MISO),
    .mosi(SPI_MOSI),
    .ss(SPI_SS),
    .rx_byte_available(spi_rx_byte_available),
    .rx_byte(spi_rx_byte),
    .tx_byte_ready_to_write(spi_tx_ready_to_write),
    .tx_byte(spi_tx_byte)
);

assign spi_rx_byte_available_rissing_edge = (spi_rx_byte_available_reg == 2'b01);
assign ss_falling_edge = (ss_reg == 2'b10);

// helper to detect edges
always @ (posedge clk_core) begin
    spi_rx_byte_available_reg[0] <= spi_rx_byte_available;
    spi_rx_byte_available_reg[1] <= spi_rx_byte_available_reg[0];

    ss_reg[0] <= SPI_SS;
    ss_reg[1] <= ss_reg[0];
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
    if (ss_falling_edge) begin
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

// I2C bridge external compass
i2c_bridge i2c_external_compass_bridge_inst(
        .CLK(clk_core),
        .MSDA(FC1_COMPASS_SDA),
        .MSCL(FC1_COMPASS_CLK),
        .SSDA(IO_COMPASS_SDA),
        .SSCL(IO_COMPASS_CLK)
);

endmodule
