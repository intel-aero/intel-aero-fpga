//					.
// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
// Authors: Rich West and Zhuoqun Cheng @ Boston University
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
		
wire [15:0]pwmFreqR;
wire [15:0]pwmDutyR;

reg pwm_d, pwm_r;
reg [15:0] ctr, ramp;
reg [31:0] pwmScale;
  
assign pwm = pwm_r;
assign pwmDutyR = {pwmDutyh,pwmDutyl};
assign pwmFreqR = {pwmFreqh,pwmFreql};  
   
always @(*) begin
   ctr = ramp + 1'b1;
   
	if (ctr >= pwmFreqR)
	  ctr = 16'h0000; 
     
	pwmScale = pwmDutyR * pwmFreqR;
   if (pwmScale >>> 16 > ramp)
     pwm_d = 1'b1;
   else
     pwm_d = 1'b0;
end
   
always @(posedge clk) begin
   if ((~resetn) || (ramp >= pwmFreqR))
     ramp <= 16'h0000;
   else
     ramp <= ctr;
    
   pwm_r <= pwm_d;
end

endmodule

