module i2c_bridge(
        MSDA,
        MSCL,
        SSDA,
        SSCL,
        CLK);

inout MSDA;
inout SSDA;
input MSCL;
output SSCL;
input CLK;

wire master_wr;
wire slave_wr;

reg [3:0] bit_count;
reg rd_wr;
reg addr_set;
reg clk_posedge;
reg clk_negedge;
reg i2c_start;
reg i2c_end;
reg ack_on;

assign SSCL = MSCL;
assign SSDA = (master_wr)? MSDA: 1'bz;
assign MSDA = (slave_wr)? SSDA: 1'bz;

reg [1:0] r_msda;
reg [1:0] r_mscl;

always @(posedge CLK) begin
    r_msda <= {r_msda[0], MSDA};
    r_mscl <= {r_mscl[0], MSCL};
end

always @(posedge CLK) begin
    i2c_start <= r_msda[1] && ~r_msda[0] && r_mscl[0];
    i2c_end <= r_msda[0] && ~r_msda[1] && r_mscl[0];
end

always @(posedge CLK) begin
   if (r_mscl[0] && ~r_mscl[1]) begin
        clk_posedge <= 1'b1;
    end else begin
        clk_posedge <= 1'b0;
    end
end

always @(posedge CLK) begin
    if (~r_mscl[0] && r_mscl[1]) begin
        clk_negedge <= 1'b1;
    end else begin
        clk_negedge <= 1'b0;
    end
end

always @(posedge CLK) begin
    if (i2c_end || i2c_start) begin
        bit_count <= 4'h00;
        rd_wr <= 1'b0;
        addr_set <= 1'b0;
        ack_on <= 1'b0;
    end else if(~addr_set && (bit_count == 4'h8) && clk_posedge) begin
        rd_wr <= r_msda[0];
    end else if ((bit_count == 4'h9) && ~ack_on) begin
        ack_on <= 1'b1;
    end else if(bit_count == 4'hA) begin
        bit_count <= 4'h1;
        ack_on <= 1'b0;
        addr_set <= 1'b1;
    end else if (clk_negedge) begin
        bit_count <= bit_count + 4'h1;
    end        
end

assign slave_wr = (~addr_set && ack_on) || (ack_on && ~rd_wr) || (rd_wr && addr_set && ~ack_on);
assign master_wr = ~slave_wr;
        
endmodule