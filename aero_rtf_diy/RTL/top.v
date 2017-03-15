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

    // PWM
    pwmout,

    // I2C slave and passthtrough
    in_MSCL,
    out_SSCL,
    inout_SSDA,
    inout_MSDA,

    // gpios
    IO_MOTORS_Tx,
    IO_MOTORS_Rx,
    FC1_MOTORS_SCL_Tx,
    FC1_MOTORS_SDA_Rx,

    IO_GPS_Tx,
    IO_GPS_Rx,
    FC1_GPS_Tx,
    FC1_GPS_Rx,

    IO_REC_Rx,
    IO_REC_Tx,
    FC1_IO3_REC_Rx,
    FC1_XBEE_CTS_REC_Tx,

    FC1_XBEE_Rx,
    FC1_XBEE_Tx,
    CHT_DBG_UART_Tx,
    CHT_DBG_UART_Rx,

    IO_TELEM_Tx,
    IO_TELEM_Rx,
    FC1_TELEM_Tx,
    FC1_TELEM_Rx,

    BOOTLOADER_FORCE_PIN
);

// global
input wire in_CLK;

output wire [15:0] pwmout;

// i2c
input wire in_MSCL;
output wire out_SSCL;
inout wire inout_SSDA;
inout wire inout_MSDA;

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

// PWM  channels
output wire IO_MOTORS_Tx;
output wire IO_MOTORS_Rx;
output wire IO_TELEM_Tx;
output wire IO_TELEM_Rx;

// unused
input  wire FC1_TELEM_Tx;
output wire FC1_TELEM_Rx;

output reg BOOTLOADER_FORCE_PIN = 0;

assign IO_GPS_Tx = FC1_MOTORS_SCL_Tx;
assign FC1_GPS_Rx = IO_GPS_Rx;
assign IO_REC_Tx = FC1_XBEE_CTS_REC_Tx;
assign FC1_IO3_REC_Rx = IO_REC_Rx;
assign CHT_DBG_UART_Rx = FC1_XBEE_Tx;
assign FC1_XBEE_Rx = CHT_DBG_UART_Tx;

//--------------------------
//    Regs and Wires
//-----------
reg         reset_n0;
reg         reset_n;
wire        clk_core;
wire        clk_adc;
wire        locked;
reg         adc_response_valid;
reg [4:0]   adc_response_channel;
reg [11:0]  adc_response_data;
reg         adc_response_startofpacket;
reg         adc_response_endofpacket;
reg         adc_sequencer_csr_address;
reg         adc_sequencer_csr_read;
reg         adc_sequencer_csr_write;
reg  [31:0] adc_sequencer_csr_writedata;
reg  [31:0] adc_sequencer_csr_readdata;
wire        adc_cmd;
reg  [7:0]  reg_ch0_upper, reg_ch0_lower;
reg  [7:0]  reg_ch1_upper, reg_ch1_lower;
reg  [7:0]  reg_ch2_upper, reg_ch2_lower;
reg  [7:0]  reg_ch3_upper, reg_ch3_lower;
reg  [7:0]  reg_ch4_upper, reg_ch4_lower;
reg  [11:0] adc_ch0_raw_data;
reg  [11:0] adc_ch1_raw_data;
reg  [11:0] adc_ch2_raw_data;
reg  [11:0] adc_ch3_raw_data;
reg  [11:0] adc_ch4_raw_data;


//--------------------------
//    PLL
//-----------
pll pll_inst (
   .inclk0(in_CLK),
   .c0(clk_adc),    // 10 MHz clock dedicated to ADC
   .c1(clk_core),    // 50 MHz sytem clock
   .locked(locked)
);


//--------------------------
//    Reset - Synchronous deassert after PLL locked
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
        .adc_adc_pll_clock_clk (clk_adc),       //  adc_adc_pll_clock.clk
        .adc_adc_pll_locked_export (locked),   // adc_adc_pll_locked.export
        .adc_response_valid (adc_response_valid),          //       adc_response.valid
        .adc_response_channel (adc_response_channel),        //                   .channel
        .adc_response_data (adc_response_data),           //                   .data
        .adc_response_startofpacket (adc_response_startofpacket),  //                   .startofpacket
        .adc_response_endofpacket (adc_response_endofpacket),    //                   .endofpacket
        .adc_sequencer_csr_address (adc_sequencer_csr_address),   //  adc_sequencer_csr.address
        .adc_sequencer_csr_read (adc_sequencer_csr_read),      //                   .read
        .adc_sequencer_csr_write (adc_sequencer_csr_write),     //                   .write
        .adc_sequencer_csr_writedata (adc_sequencer_csr_writedata), //                   .writedata
        .adc_sequencer_csr_readdata (adc_sequencer_csr_readdata),  //                   .readdata
        .clk_clk (clk_core),                     //                clk.clk
        .reset_reset_n (reset_n)               //              reset.reset_n
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

assign out_SSCL = in_MSCL;
assign inout_SSDA = (master_wr)? inout_MSDA: 1'bz;
assign inout_MSDA = ((adc_ack || adc_txd) && ~i2c_reset)? adc_sda: (slave_wr)? inout_SSDA:  1'bz;

wire ack_on;
assign ack_on = (bit_count == 4'h9)? 1'b1: 1'b0;

reg [1:0] r_msda;
reg [1:0] r_mscl;


always @(posedge clk_core) begin
    r_msda <= {r_msda[0], inout_MSDA};
    r_mscl <= {r_mscl[0], in_MSCL};
end


always @(posedge clk_core) begin
    i2c_start <=  r_msda[1] && ~r_msda[0] && r_mscl[0];
    i2c_stop <=  r_msda[0] && ~r_msda[1] && r_mscl[0];
end

always @(posedge clk_core) begin
    if(r_mscl[0] && ~r_mscl[1])
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
    end else if(i2c_stop || i2c_start || i2c_reset) begin
        bit_count <= 4'h00;
        rd_wr <= 1'b0;
        addr_set <= 1'b0;
    end else if(~addr_set && (bit_count == 4'h8) && clk_posedge)
        rd_wr <= r_msda[0];
    else if(bit_count == 4'hA) begin
        bit_count <= 4'h1;
        addr_set <= 1'b1;
    end else if (clk_negedge)
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
        slave_addr <= {slave_addr[6:0], inout_MSDA};

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

always @(posedge clk_core or negedge reset_n) begin
    if(!reset_n) begin
        adc_data <= 8'h00;
        adc_addr <= 4'h0;
        adc_sda <= 1'b1;
    end else if(adc_ack && ~clk_negedge)
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
    end
    else if(adc_txd)
        adc_sda <= adc_data[7];
end

always @(posedge clk_core or negedge reset_n)
begin
   if(~reset_n)
      i2c_reset <= 1'b0;
    else if(ack_on && clk_posedge && inout_MSDA)
       i2c_reset <= 1'b1;
   else if (i2c_start || i2c_stop)
       i2c_reset <= 1'b0;
end

assign adc_txd =  adc_cs && adc_rd_wr && ~ack_on;

assign slave_wr = (~addr_set && ack_on)  || (ack_on && ~rd_wr) || (rd_wr && addr_set && ~ack_on);
assign master_wr = ~slave_wr;


// UART Routines
// UART Baudrate - 115.2Kbps, 1bit start, 8bits data, no parity, 1bit stop.

reg [15:0] uart_rx_baud_buf;
reg uart_rx_tick;
reg [3:0] uart_bit_count;
reg [7:0] uart_rx_reg;
reg [7:0] uart_rx_buf;
reg uart_rx_dat_rdy;
reg FC1_MOTORS_SCL_Tx_r;
reg [16:0]pwmFreqReg;


parameter pwmFREQ = 16'hF424;      //F424   400Hz = 25Mhz/400 = 62500 (0xF424)
parameter BAUD_DIVISOR = 8'hD9;   // Divisor = int((clock_freq / baud rate) +0.5)

integer i;

reg [7:0] regBank [0:31];
reg [7:0] regBuf[0:33];
reg[15:0] chk_sum;
reg [5:0] byte_count;
reg [5:0] rx_timeout;
reg head_start, head_stop;
reg pwmclk;


reg clk_tick;
reg [3:0] clk_tick_cnt;

always @(posedge clk_core)
   FC1_MOTORS_SCL_Tx_r <= FC1_MOTORS_SCL_Tx;

wire uart_edge = FC1_MOTORS_SCL_Tx ^ FC1_MOTORS_SCL_Tx_r;

always @(posedge clk_core or negedge reset_n) begin
    if(~reset_n) begin
        uart_rx_baud_buf <= 0;
        uart_rx_tick <= 1'b0;
    end else if (uart_rx_baud_buf == (BAUD_DIVISOR - 1) || uart_edge) begin
        uart_rx_baud_buf <= 0;
    end else if (uart_rx_baud_buf == (BAUD_DIVISOR/2)) begin
        uart_rx_tick <= 1'b1;
        uart_rx_baud_buf <= uart_rx_baud_buf + 1'b1;
    end else begin
        uart_rx_tick <= 1'b0;
        uart_rx_baud_buf <= uart_rx_baud_buf + 1'b1;
    end
end


reg [1:0] state;
parameter WAITING = 2'b00, READING = 2'b01, STOP = 2'b10, RECOVER = 2'b11;

always @(posedge clk_core or negedge reset_n) begin
    if(~reset_n) begin
        state <= WAITING;
        uart_bit_count <= 0;
        uart_rx_buf <= 0;
        uart_rx_dat_rdy <= 1'b0;
        uart_rx_reg <= 0;
    end else begin
        uart_rx_dat_rdy <= 1'b0;
        case (state)
            WAITING : begin
                // wait for a start bit (0)
                if(!uart_edge & uart_rx_tick && !FC1_MOTORS_SCL_Tx_r) begin
                    state <= READING;
                    uart_bit_count <= 0;
                end
            end
            READING : begin
                // gather data bits
                if(uart_rx_tick) begin
                    uart_rx_buf <= {FC1_MOTORS_SCL_Tx_r,uart_rx_buf[7:1]};
                    uart_bit_count <= uart_bit_count + 1'b1;
                    if (uart_bit_count == 4'h7)
                        state <= STOP;
                end
            end
            STOP    : begin
                // verify stop bit (1)
                if (uart_rx_tick) begin
                    if (FC1_MOTORS_SCL_Tx_r) begin
                        uart_rx_reg <= uart_rx_buf;
                        uart_rx_dat_rdy <= 1'b1;
                        state <= WAITING;
                    end else begin
                        // there was a framing error. Discard the byte and work on resync
                        state <= RECOVER;
                    end
                end
            end
            RECOVER  : begin
                // wait for an idle (1) then resume
                if (uart_rx_tick) begin
                    if (FC1_MOTORS_SCL_Tx_r) state <= WAITING;
                end
            end
        endcase
    end
end

always @(posedge clk_core or negedge reset_n) begin
    if(~reset_n) begin
        byte_count <= 6'b000000;
        rx_timeout <= 6'b000000;
        head_start <= 1'b0;
        head_stop <= 1'b0;
        chk_sum <= 16'h0000;
        regBuf[0] <= 8'h12;
        regBuf[1] <= 8'h34;
        for(i=2; i< 34; i=i+1)
            regBuf[i] <=0;
    end else if(uart_rx_dat_rdy) begin
        rx_timeout <= 6'b000000;
        if((~head_start) && (uart_rx_reg == 8'hA2)) begin
            head_start <= 1'b1;
            chk_sum <= 16'h0000;
            byte_count <= 6'h0000;
        end else if(byte_count == 33) begin
            head_stop <= 1'b1;
            byte_count <= byte_count+ 1'b1;
        end else if(byte_count == 32) begin
            regBuf[byte_count] <= uart_rx_reg;
            byte_count <= byte_count+ 1'b1;
        end else if (byte_count < 32)begin
            regBuf[byte_count] <= uart_rx_reg;
            byte_count <= byte_count+ 1'b1;
            chk_sum <= chk_sum + {8'b00,uart_rx_reg};
        end
    end else begin
        head_stop <= 1'b0;
        if(uart_rx_tick && head_start) begin
            if(rx_timeout == 50) 
                head_start <= 1'b0;
            else
                rx_timeout <= rx_timeout+ 1'b1;
        end
    end
end

always @(posedge clk_core or negedge reset_n) begin
    if(~reset_n) begin
        regBank[0] <= 0;
        regBank[1] <= 1;
        regBank[2] <= 1;
        regBank[3] <= 0;
        for(i=4; i< 32; i=i+1)
            regBank[i] <= 8'h00;
    end else if(head_stop) begin
        for(i=0; i<32; i=i+1)
            regBank[i] <= regBuf[i];
    end
end


always @(posedge clk_core or negedge reset_n) begin  // 400Hz
    if(~reset_n)  begin
        pwmFreqReg <= 16'h0000;
        pwmclk <= 0;
    end    
    else if (pwmFreqReg == pwmFREQ)begin
        pwmclk <= 1'b1;
        pwmFreqReg <= 16'h0000;
    end 
    else begin
        pwmFreqReg <= pwmFreqReg + 16'h0001;
        pwmclk <= 1'b0;
    end    
end

assign IO_MOTORS_Rx = pwmout[0];
assign IO_TELEM_Rx = pwmout[0];
assign IO_TELEM_Tx = pwmout[0];

always @(posedge clk_core or negedge reset_n) begin // Clock tick every 320nsec
    if(~reset_n) begin
        clk_tick <= 1'b0;
        clk_tick_cnt <= 4'b0000;
    end
    else if(pwmclk) begin
        clk_tick <= 1'b0;
        clk_tick_cnt <= 4'b0000;
    end
    else if(clk_tick_cnt == 4'b1000) begin
        clk_tick <= 1'b1;
        clk_tick_cnt <= 4'b0000;
    end
    else begin
        clk_tick <= 1'b0;
        clk_tick_cnt <= clk_tick_cnt + 1'b1;
    end
end

pwm pwm0 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[0]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[1]),
    .pwmDutyh(regBank[0])
);

pwm pwm1 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[1]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[3]),
    .pwmDutyh(regBank[2])
);

pwm pwm2 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[2]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[5]),
    .pwmDutyh(regBank[4])
);

pwm pwm3 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[3]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[7]),
    .pwmDutyh(regBank[6])
);

pwm pwm4 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[4]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[9]),
    .pwmDutyh(regBank[8])
);

pwm pwm5 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[5]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[11]),
    .pwmDutyh(regBank[10])
);

pwm pwm6 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[6]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[13]),
    .pwmDutyh(regBank[12])
);

pwm pwm7 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[7]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[15]),
    .pwmDutyh(regBank[14])
);

pwm pwm8 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[8]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[17]),
    .pwmDutyh(regBank[16])
);

pwm pwm9 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[9]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[19]),
    .pwmDutyh(regBank[18])
);

pwm pwm10 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[10]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[21]),
    .pwmDutyh(regBank[20])
);

pwm pwm11 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[11]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[23]),
    .pwmDutyh(regBank[22])
);

pwm pwm12 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[12]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[25]),
    .pwmDutyh(regBank[24])
);

pwm pwm13 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[13]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[27]),
    .pwmDutyh(regBank[26])
);

pwm pwm14 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[14]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[29]),
    .pwmDutyh(regBank[28])
);

pwm pwm15 (
    .clk(clk_core),
    .resetn(reset_n),
    .pwm(pwmout[15]),
    .clk_tick(clk_tick),
    .pwmclk(pwmclk),
    .pwmDutyl(regBank[31]),
    .pwmDutyh(regBank[30])
);

endmodule
