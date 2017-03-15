//                    .


module pwm (
    input clk,
    input resetn,
    output pwm,
    input clk_tick,
    input pwmclk,
    input [7:0]pwmDutyl,
    input [7:0]pwmDutyh
);



reg [15:0]pwmDutyReg;
wire [15:0]pwmDutyR;

reg pwm_r;
assign pwm = pwm_r;

assign pwmDutyR = {pwmDutyh,pwmDutyl};

always @(posedge clk or negedge resetn) begin
    if(~resetn) begin
        pwmDutyReg <= 16'h0000;
        pwm_r <= 1'b0;
    end else if((pwmDutyR == 16'h0000) || pwmclk) begin
        pwmDutyReg <= 16'h0000;
        pwm_r <= 1'b0;
    end else if(pwmDutyReg >= (pwmDutyR + 16'h098C)) begin
        pwm_r <= 1'b0;
    end else if(clk_tick) begin
        pwmDutyReg <= pwmDutyReg + 1'b1;
        pwm_r <= 1'b1;
    end
end

endmodule


