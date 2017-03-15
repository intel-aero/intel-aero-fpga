//					.
// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
//
// ----------------------------------------------------------------------------



module pwm (
       input clk,
		 input resetn,
		 output pwm,
		 input pwm_on,
		 input pwm_enb,
		 input [7:0]pwmDutyl,
		 input [7:0]pwmDutyh,
		 input [7:0]pwmFreql,
		 input [7:0]pwmFreqh
		);
		
		
reg [15:0]pwmDutyReg;
reg [15:0]pwmFreqReg;
reg pwmClk;
wire [15:0]pwmFreqR;
wire [15:0]pwmDutyR;

reg [1:0]pwmClkBuf;
reg pwm_r;

assign pwm = pwm_r;


assign pwmDutyR = {pwmDutyh,pwmDutyl};
assign pwmFreqR = {pwmFreqh,pwmFreql};


always @(posedge clk or negedge resetn) begin
   if(~resetn)  begin
      pwmFreqReg <= 16'h0000;
      pwmClk <= 0;
   end	
   else if ((pwmFreqReg > pwmFreqR) || ~pwm_on || ~pwm_enb) begin
      pwmFreqReg <= 16'h0000;
      pwmClk <= 0;
   end 
	else if (pwmFreqReg == pwmFreqR)begin
      pwmClk <= ~pwmClk;
      pwmFreqReg <= 16'h0000;
   end 
	else
      pwmFreqReg <= pwmFreqReg + 16'h0001;
end


always @(posedge clk) begin
 
	pwmClkBuf[1:0] <= {pwmClkBuf[0],pwmClk}; // test
	
end

always @(posedge clk  or negedge resetn) begin
  if (~resetn)
     pwmDutyReg <= 16'h0000;     
  else if ((pwmDutyR == 0) || ~pwm_on || ~pwm_enb) 
     pwmDutyReg <= 16'h0000;
  else if( ~pwmClkBuf[1] && pwmClkBuf[0])     //clock rising edge
     pwmDutyReg <= pwmDutyReg + 16'h0000;
  else 
     pwmDutyReg <= pwmDutyReg + 16'h0001;
     
end


always @(posedge clk) begin
   if (pwmDutyReg >= pwmDutyR)
      pwm_r <= 0;
   else if(pwmDutyReg == 0)
      pwm_r <= 0;
	else 
	   pwm_r <= 1;
end

endmodule   
		
                             