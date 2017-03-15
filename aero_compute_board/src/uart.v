//											.

// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
//
// ----------------------------------------------------------------------------


module uart(clk, resetn, uart_baud_regl, uart_baud_regh, uart_tx_reg, 
            uart_rx_reg, uart_txd, uart_rxd, uart_tx_on, uart_rx_dat_rdy, uart_tx_busy);


input clk;
input resetn;
input uart_rxd;
input [7:0] uart_tx_reg;
input uart_tx_on;
input [7:0] uart_baud_regl;
input [7:0] uart_baud_regh;

output uart_txd;
output uart_tx_busy;
output uart_rx_dat_rdy;
output [7:0] uart_rx_reg;

wire [15:0]uart_baud_reg = {uart_baud_regh,uart_baud_regl};

reg [15:0] uart_tx_baud_buf;
reg [10:0] uart_tx_buf;
reg uart_tx_tick;
reg transmit;
assign uart_tx_busy = ~transmit;

reg [15:0] uart_rx_baud_buf;
reg uart_rx_tick;
reg [3:0] uart_bit_count;
reg [7:0] uart_rx_reg;
reg [7:0] uart_rx_buf;
reg uart_rx_dat_rdy;
reg uart_rxd_r;


assign uart_txd = uart_tx_buf[0];

always @(posedge clk) begin
   if(~resetn) begin
      uart_tx_baud_buf <= 0;
      uart_tx_tick <= 1'b0;
   end
   else if (uart_tx_baud_buf == uart_baud_reg-1) begin
      uart_tx_baud_buf <= 0;
      uart_tx_tick <= 1'b1;
   end
   else begin
      uart_tx_tick <= 1'b0;
      uart_tx_baud_buf <= uart_tx_baud_buf + 1'b1;
   end
end

always @(posedge clk) begin
   if(~resetn) begin
      uart_tx_buf <= {11'b00000000001};
      transmit <= 1'b1;
   end
   else begin		
      if(!transmit & uart_tx_tick) begin
       	 uart_tx_buf <= {1'b0,uart_tx_buf[10:1]};
	       transmit <= ~|uart_tx_buf[10:1];
      end		
      else if(transmit & uart_tx_on) begin
         uart_tx_buf[10:1] <= {1'b1,uart_tx_reg,1'b0};
         transmit <= 1'b0;		
      end
      else begin
	      transmit <= ~|uart_tx_buf[10:1];
      end
   end		
end

always @(posedge clk)
   uart_rxd_r <= uart_rxd;
   
wire uart_edge = uart_rxd ^ uart_rxd_r;  

always @(posedge clk) begin
   if(~resetn) begin
      uart_rx_baud_buf <= 0;
      uart_rx_tick <= 1'b0;
   end
   else if (uart_rx_baud_buf == (uart_baud_reg - 1) || uart_edge) begin
      uart_rx_baud_buf <= 0;
   end
      else if (uart_rx_baud_buf == (uart_baud_reg/2)) begin
         uart_rx_tick <= 1'b1;
         uart_rx_baud_buf <= uart_rx_baud_buf + 1'b1;
      end
   else begin
      uart_rx_tick <= 1'b0;
      uart_rx_baud_buf <= uart_rx_baud_buf + 1'b1;
   end
end


reg [1:0] state;
parameter WAITING = 2'b00, READING = 2'b01, STOP = 2'b10, RECOVER = 2'b11;

always @(posedge clk) begin
   if(~resetn) begin
      state <= WAITING;
      uart_bit_count <= 0;
      uart_rx_buf <= 0;
      uart_rx_dat_rdy <= 1'b0;
      uart_rx_reg <= 0;
   end
   else begin
      uart_rx_dat_rdy <= 1'b0;
      case (state) 
         WAITING : begin
         	// wait for a start bit (0)
		   if(!uart_edge & uart_rx_tick && !uart_rxd_r) begin
		      state <= READING;
		      uart_bit_count <= 0;
		   end
         end
	 READING : begin
        	// gather data bits
                   if(uart_rx_tick) begin
                      uart_rx_buf <= {uart_rxd_r,uart_rx_buf[7:1]};
		      uart_bit_count <= uart_bit_count + 1'b1;
		      if (uart_bit_count == 4'h7) 
		         state <= STOP;
		   end
	 end
	 STOP    : begin
		 // verify stop bit (1)
		 if (uart_rx_tick) begin
		    if (uart_rxd_r) begin
		       uart_rx_reg <= uart_rx_buf;
		       uart_rx_dat_rdy <= 1'b1;
	               state <= WAITING;
	            end
	            else begin
	                // there was a framing error. Discard the byte and work on resync
	               state <= RECOVER;
	            end
                 end					
         end
         RECOVER  : begin
		    // wait for an idle (1) then resume
		  if (uart_rx_tick) begin
		     if (uart_rxd_r) state <= WAITING;
		  end				
	  end
      endcase
   end
end

endmodule
