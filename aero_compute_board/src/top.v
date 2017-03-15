// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
//
// ----------------------------------------------------------------------------

`timescale 1ns/1ps

//--------------------------
//	Declaration and ports
//-----------
module top (
	
	//global
	input   in_CLK,
	
	//spi
	output  MISO,
	input   MOSI,
	input   SCK,
	input   SSEL,
	
	//i2c bridge
//	input   in_MSCL,
//	output  out_SSCL,
//	inout   inout_SSDA,
//	inout   inout_MSDA,
	
    output [3:0] outp,
	 input [3:0] inp,
	// gpio - uart
	output  uart_txd,
	input   uart_rxd,
	
	// gpio - i2c
	inout   i2c_sda,
	output  i2c_scl,
	
	// PWM
	output [11:0] pwmout

);


parameter UART_TX = 8'h04;
parameter UART_RX = 8'h05;
parameter I2C_TX = 8'h06;
parameter I2C_RX = 8'h07;


//--------------------------
//	Regs and Wires
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
   reg adc_cmd_r0, adc_cmd_r1;  
   reg adc_on,adc_off;
   reg [11:0] adc_ch0_raw_data;
   reg [11:0] adc_ch1_raw_data;
   reg [11:0] adc_ch2_raw_data;
   reg [11:0] adc_ch3_raw_data;
   reg [11:0] adc_ch4_raw_data;
   
	
	integer i;
   
   assign w_MOSI = uart_tx_on; //MOSI;
	assign w_MISO = i2c_tx_on; //MISO;
	assign w_SSEL = i2c_busy;
	assign w_SCK = i2c_start;
   //wire pwmout [12:0];

	
	
	
//--------------------------
//	PLL
//-----------
pll pll_inst (
   .inclk0 ( in_CLK ),
   .c0 ( clk_adc ),	// 10 MHz clock dedicated to ADC
   .c1 ( clk_core ),	// 25 MHz sytem clock
   .locked ( locked )
);


//--------------------------
//	Reset - Synchronous deassert after PLL locked
//-----------
//Synchronize Reset
always @(posedge in_CLK)
	begin
	if (!locked)
		begin
		reset_n0 <= 1'b0;
		reset_n <= 1'b0;
		end
	else
		begin
		reset_n0 <= 1'b1;
		reset_n <= reset_n0;
		end
	end



// Issue ADC sequencer start and stop commands

always @(posedge clk_core or negedge reset_n)begin
   if(~reset_n) begin
	   adc_cmd_r0 <= 1'b0;
		adc_cmd_r1 <= 1'b0;
	 end
	else begin
	   adc_cmd_r1 <= adc_cmd_r0;
		adc_cmd_r0 <= adc_cmd;
		if(adc_cmd_r0 && ~adc_cmd_r1)
      	adc_on <= 1'b1;
		else 
		   adc_on <= 1'b0;
	   if(~adc_cmd_r0 && adc_cmd_r1)
		   adc_off <= 1'b1;
		else 
		   adc_off <= 1'b0;
	end
end


always @(posedge clk_core or negedge reset_n) begin
   if (!reset_n) begin
      adc_sequencer_csr_address <= 1'b0;
      adc_sequencer_csr_read <= 1'b0;
      adc_sequencer_csr_write <= 1'b0;
      adc_sequencer_csr_writedata <= 32'b0;
   end else begin
      if (adc_on) begin
  	      adc_sequencer_csr_address <= 1'b0;
	      adc_sequencer_csr_read <= 1'b0;
	      adc_sequencer_csr_write <= 1'b1;
	      adc_sequencer_csr_writedata <= 32'b1;
      end else if (adc_off) begin
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
//	ADC- 7 channels: ch 0-5, microphone
//-----------


adc adc0(
		.adc_pll_clock_clk (clk_adc),           						//     adc_pll_clock.clk
		.adc_pll_locked_export (locked), 						      //     adc_pll_locked.export
		.adc_response_valid (adc_response_valid),          		//     adc_response.valid
		.adc_response_channel (adc_response_channel),        		//     .channel
		.adc_response_data (adc_response_data),           			//     .data
		.adc_response_startofpacket (adc_response_startofpacket),//     .startofpacket
		.adc_response_endofpacket (adc_response_endofpacket),    //     .endofpacket
		.adc_sequencer_csr_address (adc_sequencer_csr_address),  //     adc_sequencer_csr.address
		.adc_sequencer_csr_read (adc_sequencer_csr_read),        //     .read
		.adc_sequencer_csr_write (adc_sequencer_csr_write),      //     .write
		.adc_sequencer_csr_writedata (adc_sequencer_csr_writedata),//   .writedata
		.adc_sequencer_csr_readdata (adc_sequencer_csr_readdata),  //   .readdata
		.clk_clk (clk_core),                       					//     clk.clk
		.reset_reset_n  (reset_n)              						//     reset.reset_n
	);



//--------------------------
//	Capture ADC channel data
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



// SPI Slave Module

reg i2c_tx_on;
reg uart_tx_on;
reg uart_rx_on;
wire uart_rx_dat_rdy;


//assign i2c_start = regAdd[2][6];
//assign i2c_stop = regAdd[2][7];
//assign i2c_rx_on = regAdd[2][5];


parameter fpga_ver_l = 8'h00;
parameter fpga_ver_h = 8'hA1;
parameter regBank_size = 73;  //62 // 74-12 = 62


reg [7:0]regBank[0:regBank_size];


/*

Reg               B7       B6       B5       B4       B3           B2            B1          B0
I2C_STATUS   i2c_busy   i2c_start i2c_stop i2c_ack i2c_rx_on  uart_rx_dat_rdy uart_tx_busy  adc_on
PWM_STS0      pwm7_on    pwm6_on   pwm5_on pwm4_on  pwm3_on     pwm2_on        pwm1_on     pwm0_on
PWM_STS1  pwm_global_on                            pwm11_on    pwm10_on        pwm9_on     pwm8_on
  
*/
// 

// --------------------------------------------------------------------
// Regsiter and wire declaration
// --------------------------------------------------------------------
//reg [7:0] regBank[0:15]; // Register Bank
reg [2:0] curState, nxtState;


//|@Regs ;

reg [2:0] counter;
reg [1:0] mosiReg;
reg [2:0] selReg;
reg [2:0] sckReg;
reg [7:0] transmitByte;
reg [7:0] receiveByte;
reg [6:0] addr;
reg  wrRdn;
reg addrSet;

//|@Wires ; 
wire  receivedBit;
wire  sSelStartMsg;
wire  sckFallingEdge;
wire  sckRaisingEdge;
wire sSelActive;
reg regRd, regWr;
reg i2c_busy_r;
//wire  regRd;
//wire  regWr;


assign MISO = transmitByte[7];
assign adc_cmd = regBank[12][7];

wire i2c_start = regBank[12][6];
wire i2c_stop = regBank[12][5];
wire i2c_rx_on = regBank[12][4];
wire [7:0]i2c_rx_reg;
wire i2c_busy;
wire i2c_ack;
wire [7:0] uart_rx_reg;
wire uart_tx_busy;



// --------------------------------------------------------------------
// Sychronize the SPI inputs and get the data from lines
// --------------------------------------------------------------------

always @(posedge clk_core or negedge reset_n) begin
   if (~reset_n) begin
//        sckReg[2:0]  <= 3'h0;
        selReg[2:0] <= 3'h0;
   end else begin
//        sckReg[2:0]  <= { sckReg[1:0], SCK} ;
        selReg[2:0] <= { selReg[1:0], SSEL} ;
   end
end


always @(posedge clk_core or negedge reset_n) begin
   if(~reset_n)
	  sckReg[0] <= 1'b0;
   else
     sckReg[0] <= SCK;	
end



assign sckRaisingEdge =  ~sckReg[0] && SCK;   //(sckReg[2:1] == 2'b01);
assign sckFallingEdge =  sckReg[0] && ~SCK;  //(sckReg[2:1] == 2'b10);
assign sSelActive   = ~selReg[1];
assign sSelStartMsg = ( selReg[2:1] == 2'b10);
always @(posedge clk_core )
      mosiReg[1:0] <= { mosiReg[0], MOSI};

assign receivedBit = mosiReg[1];
// --------------------------------------------------------------------
// Counter for the capture bits
// --------------------------------------------------------------------

always @(posedge clk_core ) begin
   if (sSelStartMsg) begin
        counter[2:0]  <= 3'h0;
   end else if (sckRaisingEdge) begin
        counter[2:0]  <= counter[2:0] + 3'h1;
   end
end

//assign byteDone = sSelActive & sckRaisingEdge & (counter[2:0] == 3'b111);
reg byteDone;

always @(posedge clk_core) begin
   if (sSelActive & sckRaisingEdge & (counter[2:0] == 3'b111))
      byteDone <= 1'b1;
   else 
      byteDone <= 1'b0;
end


// --------------------------------------------------------------------
// Accumate the received Information
// --------------------------------------------------------------------

always @(posedge clk_core) begin
 if (sckRaisingEdge & sSelActive) begin
        receiveByte[7:0]  <= { receiveByte[6:0], receivedBit };
 end
end

//--------------------------------------------------------------------
// Load the address and control  //--------------------------------------------------------------------

always @(posedge clk_core )begin
   if (sSelStartMsg) begin
        wrRdn     <= 1'b0; // Default is read Transaction
        addr[6:0] <= 7'h0;
        addrSet <= 1'b0;
		  
   end else if (byteDone & ~addrSet) begin
       wrRdn <= receiveByte[7];
       addr[6:0]  <= receiveByte[6:0];
       addrSet <= 1'b1;
		
   end else if (regWr | regRd) begin
        addr[6:0] <= addr[6:0] + 7'h1;
   end

 	    
end


// --------------------------------------------------------------------
// Register Read/Write Logic
// --------------------------------------------------------------------

//assign regWr = addrSet & wrRdn & byteDone;
//assign regRd = addrSet & ~wrRdn & byteDone;


always @(posedge clk_core) begin
   if(byteDone & ~addrSet) begin
      regWr <= 1'b0;
      regRd <= ~receiveByte[7];
   end
   else if (byteDone)begin
     regWr <= wrRdn;
     regRd <= ~wrRdn;
   end
   else begin
      regWr <= 1'b0;
      regRd <= 1'b0;
   end
end

always @(posedge clk_core)
   i2c_busy_r <= i2c_busy;
	  
always @(posedge clk_core or negedge reset_n)begin
   if(~reset_n) begin
      i2c_tx_on <= 1'b0;
      uart_tx_on <= 1'b0;
      regBank[0] <= fpga_ver_l;
      regBank[1] <= fpga_ver_h;
//		regBank[13] <= 8'hD9;
//		regBank[17] <= 8'hFA;
      for(i=2 ; i< regBank_size; i=i+1) begin
         if(i== 13)
			  regBank[i] <= 8'hD9;
			else if (i == 17)
			   regBank[i] <= 8'hFA;
		   else
			   regBank[i] <= 8'h00;
		end

   end
   else if(i2c_busy_r && ~i2c_busy) begin
      regBank[12][6:4] <= 3'b000;
      i2c_tx_on <= 1'b0;
   end    
   else if (uart_tx_busy)
      uart_tx_on <= 1'b0;
   else if(regWr) begin
      if (addr >= 12) begin
         regBank[addr] <= receiveByte;
         if (addr == 15)
            uart_tx_on <= 1'b1;
         else if (addr == 19)
            i2c_tx_on <= 1'b1;
	   end	
   end
	else if (adc_response_endofpacket) begin
			{regBank[3], regBank[2]} <= {4'b0, adc_ch0_raw_data[11:0]}; 
			{regBank[5], regBank[4]} <= {4'b0, adc_ch1_raw_data[11:0]}; 
			{regBank[7], regBank[6]} <= {4'b0, adc_ch2_raw_data[11:0]}; 
			{regBank[9], regBank[8]} <= {4'b0, adc_ch3_raw_data[11:0]}; 
			{regBank[11], regBank[10]} <= {4'b0, adc_ch4_raw_data[11:0]}; 
	end
	
end


always @(posedge clk_core or negedge reset_n) begin
   if(~reset_n) begin 
      uart_rx_on <= 1'b0;
		transmitByte <= 8'h00;
	end
   else if (regRd) begin
      if(addr == 16) begin
         uart_rx_on <= 1'b0;
         transmitByte[7:0] <= uart_rx_reg;
      end
      else if (addr == 20)
         transmitByte[7:0] <= i2c_rx_reg;
      else if (addr == 12)
         transmitByte[7:0] <= {regBank[2][7:4],i2c_busy,i2c_ack,uart_tx_busy, uart_rx_on};
      else if (addr == 73)
		   transmitByte[7:0] <= {inp[3:0],regBank[73][3:0]};
		else
         transmitByte[7:0] <= regBank[addr];
   end      
   else if(sckFallingEdge && ~(counter ==0)) begin
      transmitByte[7:0] <= {transmitByte[6:0],1'b0};
   end
   else if(uart_rx_dat_rdy)
     uart_rx_on <= 1'b1;
end

wire pwm0_on = regBank[21][0];
wire pwm1_on = regBank[21][1];
wire pwm2_on = regBank[21][2];
wire pwm3_on = regBank[21][3];
wire pwm4_on = regBank[21][4];
wire pwm5_on = regBank[21][5];
wire pwm6_on = regBank[21][6];
wire pwm7_on = regBank[21][7];
wire pwm8_on = regBank[22][0];
wire pwm9_on = regBank[22][1];
wire pwm10_on = regBank[22][2];
wire pwm11_on = regBank[22][3];


wire pwm_enb = regBank[22][7];

assign outp = regBank[73][3:0];

i2c_master i_i2c_master (
       .clk(clk_core),
	    .resetn(reset_n),
	    .i2c_baud_clk_l(regBank[17]),
	    .i2c_baud_clk_h(regBank[18]),
	    .i2c_sda(i2c_sda),
	    .i2c_scl(i2c_scl),
	    .i2c_tx_on(i2c_tx_on),
       .i2c_start(i2c_start),
       .i2c_stop(i2c_stop),    
       .i2c_rx_on(i2c_rx_on),
       .i2c_busy(i2c_busy),
       .i2c_ack(i2c_ack),
       .i2c_tx_reg(regBank[19]),
       .i2c_rx_reg(i2c_rx_reg)
       );

uart  i_uart (
      .clk(clk_core),
	   .resetn(reset_n),  
	   .uart_baud_regl(regBank[13]),
	   .uart_baud_regh(regBank[14]),
	   .uart_tx_reg(regBank[15]),
	   .uart_rx_reg(uart_rx_reg),
	   .uart_txd(uart_txd),
	   .uart_rxd(uart_rxd),
	   .uart_tx_on(uart_tx_on),
	   .uart_rx_dat_rdy(uart_rx_dat_rdy),
	   .uart_tx_busy(uart_tx_busy)
	   );
	   

pwm pwm0 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[0]),
     .pwm_on(pwm0_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[49]),
	  .pwmDutyh(regBank[50]),
	  .pwmFreql(regBank[23]),
	  .pwmFreqh(regBank[24])
	);

pwm pwm1 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[1]),
     .pwm_on(pwm0_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[51]),
	  .pwmDutyh(regBank[52]),
	  .pwmFreql(regBank[25]),
	  .pwmFreqh(regBank[26])
	);

pwm pwm2 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[2]),
     .pwm_on(pwm1_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[53]),
	  .pwmDutyh(regBank[54]),
	  .pwmFreql(regBank[27]),
	  .pwmFreqh(regBank[28])

	);

pwm pwm3 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[3]),
     .pwm_on(pwm2_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[55]),
	  .pwmDutyh(regBank[56]),
	  .pwmFreql(regBank[29]),
	  .pwmFreqh(regBank[30])
	);

pwm pwm4 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[4]),
     .pwm_on(pwm4_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[57]),
	  .pwmDutyh(regBank[58]),
	  .pwmFreql(regBank[31]),
	  .pwmFreqh(regBank[32])
	);

pwm pwm5 (
     .clk(clk_core),
	  .resetn(reset_n),
	  .pwm(pwmout[5]),
     .pwm_on(pwm5_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[59]),
	  .pwmDutyh(regBank[60]),
	  .pwmFreql(regBank[33]),
	  .pwmFreqh(regBank[34])
	);

pwm pwm6 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[6]),
     .pwm_on(pwm6_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[61]),
	  .pwmDutyh(regBank[62]),
	  .pwmFreql(regBank[35]),
	  .pwmFreqh(regBank[36])
	);

pwm pwm7 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[7]),
     .pwm_on(pwm7_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[63]),
	  .pwmDutyh(regBank[64]),
	  .pwmFreql(regBank[37]),
	  .pwmFreqh(regBank[38])
	);

pwm pwm8 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[8]),
     .pwm_on(pwm8_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[65]),
	  .pwmDutyh(regBank[66]),
	  .pwmFreql(regBank[39]),
	  .pwmFreqh(regBank[40])
	);

pwm pwm9 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[9]),
     .pwm_on(pwm9_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[67]),
	  .pwmDutyh(regBank[68]),
	  .pwmFreql(regBank[41]),
	  .pwmFreqh(regBank[42])
	);

pwm pwm10 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[10]),
     .pwm_on(pwm10_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[69]),
	  .pwmDutyh(regBank[70]),
	  .pwmFreql(regBank[43]),
	  .pwmFreqh(regBank[44])
	);

pwm pwm11 (
     .clk(clk_core),
     .resetn(reset_n),
     .pwm(pwmout[11]),
     .pwm_on(pwm11_on),
	  .pwm_enb(pwm_enb),
	  .pwmDutyl(regBank[71]),
	  .pwmDutyh(regBank[72]),
	  .pwmFreql(regBank[45]),
	  .pwmFreqh(regBank[46])
	);
								

endmodule
