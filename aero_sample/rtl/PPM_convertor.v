
////////////////////////////////////////////


module gate_and(e1,e2,s);

	input e1,e2;

	output s;

	assign s = e1&e2;

endmodule

////////////////////////////////////////////

module counter_16bits(reset,clk,S);

	input reset,clk;

	output reg [15:0] S;
	
	always @(posedge clk or posedge reset) begin
		if (reset) begin 
			S <= 0; end
		else  begin
			S <= S + 1;
			end
		
	end
				
	
initial begin
	S = 0;
end
			
endmodule
////////////////////////////////////////////

module counter_4bits(reset,clk,S);

	input reset,clk;

	output reg [3:0] S;

	always @(posedge clk or negedge reset) begin
			if(reset == 0) begin
			S <= 15; end 			
			else if (clk == 1) begin
			S <= S + 1;end 
	end


	initial begin
		S = 15;
	end	
	
endmodule
////////////////////////////////////////////
module verrou_D(lock,clk,data_in,data_out);

	input lock,clk;
	input [15:0] data_in;

	output reg [15:0] data_out;
	
	always	@(posedge clk && lock == 0) 
		data_out <= data_in - 1;
	
endmodule
////////////////////////////////////////////

module compa_16bits(data_in,s);

	input [15:0] data_in;

	output reg s;
	
	always @(*)	
		if(data_in > 65500)
			s <= 0;
		else
			s <= 1;
	
initial begin
	s = 1;
end
endmodule
////////////////////////////////////////////

module freq_diviseur_15(clk_in,clk_out);

	input clk_in;
	
	output reg clk_out;
	

	reg tmp1,tmp2;
	
	reg [3:0] diviseur;
	
	always @(posedge clk_in) begin
			if (diviseur < 9) begin
				diviseur = diviseur + 1; 
				tmp1 <= 1; end
			else	begin
				diviseur = diviseur + 1; 
				tmp1 <= 0; end

	
		if(diviseur > 14) begin
			diviseur <= 0;end
	end
	
	
	always @(negedge clk_in) begin
	
		if(diviseur > 7 ) begin
			tmp2 <= 0; end	
			else begin
			tmp2 <= 1; end
	end
	
	
initial begin
	diviseur = 0;
end
	
	
	always @(*) begin
	clk_out = tmp1&tmp2;end
	
endmodule



////////////////////////////////////////////


module gene_pulse(PPM_in,pulse);

	input PPM_in;	
	output reg pulse;
	

	always @(posedge PPM_in) begin
			 pulse = 1;
		#1000 pulse = 0;
	end
	


endmodule

////////////////////////////////////////////
module PPM_convertor(raw_clock, PPM_input, data_output, canal_output, pulse_syncro);

input raw_clock, PPM_input;
output [15:0] data_output;
output [3:0] canal_output;
output reg pulse_syncro;
wire S_and,S_compa,clock_divi,pulse;
wire [15:0] S_counter16;


gate_and gate_and1 (.e1(clock_divi),.e2(S_compa),.s(S_and));
gene_pulse gene_pulse1(.PPM_in(PPM_input),.pulse(pulse));
counter_16bits counter16 (.reset(pulse),.clk(S_and),.S(S_counter16));
freq_diviseur_15 freq_divi (.clk_in(raw_clock),.clk_out(clock_divi));
compa_16bits compa16 (.data_in(S_counter16),.s(S_compa));
counter_4bits counter4 (.reset(S_compa),.clk(PPM_input),.S(canal_output));
verrou_D lock_D (.lock(PPM_input),.clk(clock_divi),.data_in(S_counter16),.data_out(data_output));

always @(negedge pulse) begin
	pulse_syncro = 1;
	#1000 pulse_syncro = 0;
	
end
endmodule