//                                         .
// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
//
// ----------------------------------------------------------------------------


module i2c_master( clk, resetn, i2c_baud_clk_l, i2c_baud_clk_h,
                   i2c_sda, i2c_scl, i2c_tx_on, i2c_stop, i2c_start,
                   i2c_rx_on, i2c_ack, i2c_busy, i2c_tx_reg, i2c_rx_reg);

input clk;
input resetn;
input [7:0] i2c_baud_clk_l;
input [7:0] i2c_baud_clk_h;
inout i2c_sda;
output i2c_scl;
input i2c_tx_on;
input i2c_rx_on;
input [7:0] i2c_tx_reg;
output [7:0] i2c_rx_reg;
output i2c_busy;
input i2c_start;
input i2c_stop;
output i2c_ack;

reg [15:0] i2c_clk_cntr;
reg i2c_tick;
reg [7:0] i2c_tx_buf;
reg [3:0] i2c_bit_count;
reg [7:0] i2c_rx_reg;
reg i2c_sda_reg;
reg i2c_ack;
reg i2c_scl;
reg i2c_busy;
//reg [3:0] i2c_count;

wire [15:0] i2c_clk_reg = {i2c_baud_clk_h, i2c_baud_clk_l};

always @(posedge clk or negedge resetn)
begin
    if(~resetn) begin
       i2c_clk_cntr <= 0;
       i2c_tick <= 1'b0;
    end
    else if(i2c_clk_cntr == (i2c_clk_reg-1)) begin
       i2c_clk_cntr <= 0;
       i2c_tick <= 1'b1;
    end
    else begin
       i2c_clk_cntr <= i2c_clk_cntr + 1'b1;
       i2c_tick <= 1'b0;
    end
end

assign i2c_sda = i2c_sda_reg;


always @(posedge clk or negedge resetn)
begin
   if(~resetn) begin
      i2c_scl <= 1'b1;
      i2c_sda_reg <= 1'bz;
      i2c_bit_count <= 4'b0000;
      i2c_tx_buf <= 00;
      i2c_busy <= 1'b0;
      i2c_ack <= 1'b0;
   end
   else if(i2c_tick && i2c_busy && ~(i2c_tx_on || i2c_rx_on))begin
	   i2c_busy <= 1'b0;
      if(i2c_start)
         i2c_scl <= 1'b0;
      else
         i2c_scl <= 1'b1;
   end
   else if(i2c_start && i2c_tick) begin
      i2c_scl <= 1'b1;
      i2c_sda_reg <= 1'b0;
      i2c_busy <= 1'b1;
   end
   else if(i2c_stop && i2c_tick) begin
      i2c_scl <= 1'b1;
      i2c_sda_reg <= 1'b0;
      i2c_busy <= 1'b1;
   end
   else if(i2c_tick && i2c_tx_on && ~i2c_busy)begin  
      if(i2c_tx_reg[7])
         i2c_sda_reg <= 1'bz;
      else
         i2c_sda_reg <= 1'b0;
      i2c_tx_buf <= {i2c_tx_reg[6:0],1'b0};
      i2c_busy <= 1'b1;
      i2c_bit_count <= 4'b0000;
   end
   else if(i2c_tick && i2c_rx_on && ~i2c_busy) begin
      i2c_bit_count <= 4'b0000;
      i2c_busy <= 1'b1;
      i2c_rx_reg[0] <= i2c_sda;
   end
      
  /* else if(i2c_tick && i2c_tx_on && i2c_busy) begin
      i2c_scl <= ~i2c_scl;
      if(i2c_scl) begin
        i2c_bit_count <= 4'b0001;
        i2c_sda_reg <= i2c_tx_buf[7];
        i2c_tx_buf <= {i2c_tx_buf[6:0],1'b0};
      end
   end
*/  


   else if(i2c_tick && i2c_busy ) begin //&& ~i2c_scl) begin
      i2c_scl <= ~i2c_scl;
      if(i2c_scl) begin
         i2c_bit_count <= i2c_bit_count + 1'b1;
         if(i2c_bit_count == 4'b0111) begin
            if(i2c_tx_on)
               i2c_sda_reg <= 1'bz;
            else if(i2c_rx_on) 
               i2c_sda_reg <= 1'b0;
         end
         else if(i2c_bit_count == 4'b1000) begin
            i2c_busy <= 1'b0;
            if(i2c_rx_on) begin
               i2c_sda_reg <= 1'bz;
 //           i2c_data_rdy <= 1'b1;
	    end
            else if(i2c_tx_on)
              i2c_ack <= ~i2c_sda;
         end
         else if(i2c_tx_on) begin
            i2c_sda_reg <= i2c_tx_buf[7];
            i2c_tx_buf <= {i2c_tx_buf[6:0],1'b0};
         end
         else if(i2c_rx_on)
         i2c_rx_reg <= {i2c_rx_reg[6:0],i2c_sda};
      end
   end  
end


endmodule       
           
     
     





